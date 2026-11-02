// BeanNG: game loop, input, cameras and the physics thread.
#include <SDL3/SDL.h>
#include <epoxy/gl.h>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include "props.h"
#include "render.h"
#include "vehicle.h"

namespace {
constexpr double kPhysDt = 1.0 / 2000.0;
constexpr int kMaxStepsPerSlice = 80;  // never spiral: if we fall behind, time simply slows down

const V3 kSpawn{-90, 0, 0};

// Everything the input thread hands to the physics thread.
struct SharedInput {
  Controls c;
  bool reset = false, repair = false, paused = false;
  double timeScale = 1.0;
  int grabNode = -1;
  V3 grabTarget;
};

struct Game {
  World world;
  Sim sim;
  Vehicle car;
  CarSkin skin;
  Sim simStart;
  Vehicle carStart;

  std::mutex inputMutex;
  SharedInput input;
  std::mutex snapMutex;
  Snapshot snap;
  std::atomic<bool> running{true};
  std::atomic<bool> resetVisuals{false};

  void applyRepair() {
    // Restore the pristine car, keeping position and heading.
    const V3 cog = car.cog;
    const V3 fwd = car.fwd;
    const double yaw = std::atan2(fwd.y, fwd.x);
    const V3 c0 = carStart.cog;
    const double yaw0 = std::atan2(carStart.fwd.y, carStart.fwd.x);
    const double dy = yaw - yaw0;
    const double cy = std::cos(dy), sy = std::sin(dy);
    const Obj& o = simStart.objs[carStart.obj];
    const double ground = world.height(cog.x, cog.y);
    for (u32 i = o.nodeStart, e = i + o.nodeCount; i < e; i++) {
      const V3 r = simStart.pos[i] - c0;
      sim.pos[i] = V3{cog.x + cy * r.x - sy * r.y, cog.y + sy * r.x + cy * r.y, ground + c0.z + r.z + 0.03};
      sim.vel[i] = {};
    }
    for (u32 i = o.beamStart, e = i + o.beamCount; i < e; i++) sim.beams[i] = simStart.beams[i];
    car = carStart;
    sim.plasticEnergy = 0;
    sim.events.clear();
  }

  void fillSnapshot(double load) {
    std::lock_guard<std::mutex> lock(snapMutex);
    Snapshot& s = snap;
    s.pos = sim.pos;
    s.beamForce.resize(sim.objs[0].beamCount);
    for (u32 i = 0; i < sim.objs[0].beamCount; i++) {
      const Beam& b = sim.beams[i];
      s.beamForce[i] = (b.flags & BF_BROKEN) ? 0.f : (b.force == 0.f ? 1e-3f : b.force);
    }
    s.events.insert(s.events.end(), sim.events.begin(), sim.events.end());
    if (s.events.size() > 2000) s.events.clear();
    sim.events.clear();
    s.carCog = car.cog;
    s.carFwd = car.fwd;
    s.carUp = car.up;
    s.carLeft = car.left;
    s.carVel = car.vCog;
    for (int i = 0; i < 4; i++) {
      const Wheel& w = car.wheels[i];
      WheelView& v = s.wheels[i];
      v.hub = (sim.pos[w.axIn] + sim.pos[w.axOut]) * 0.5;
      v.axis = norm(w.left ? sim.pos[w.axOut] - sim.pos[w.axIn] : sim.pos[w.axIn] - sim.pos[w.axOut]);
      v.up = sim.pos[w.ring[0]] - v.hub;
      v.theta = w.theta;
      v.radius = lerpd(w.R, w.R - 0.095, w.deflate);
      v.width = w.width;
      v.left = w.left;
      s.flat[i] = w.deflate > 0.1;
    }
    s.speedKmh = car.speed * 3.6;
    s.rpm = car.rpm();
    s.throttle = car.throttleAct;
    s.brake = car.brakeAct;
    s.steer = car.steerAct;
    s.coolant = car.dmg.coolantTemp;
    s.accum = car.accumulator;
    s.roll = car.rollAngle;
    s.latG = car.latAcc / 9.81;
    s.lonG = car.lonAcc / 9.81;
    s.peakG = car.peakG;
    s.plastic = sim.plasticEnergy;
    s.ram = car.axle[0].ram;
    s.gear = car.gear;
    s.heightMode = car.heightMode;
    s.autoBox = car.autoBox;
    s.soft = car.hydractiveSoft;
    s.sport = car.sportMode;
    s.activa = car.activaOn;
    s.running = car.engineRunning;
    s.radiator = car.dmg.radiator;
    s.hydraulics = car.dmg.hydraulics;
    s.mount = car.dmg.engineMount;
    s.steering = car.dmg.steering;
    s.seized = car.dmg.seized;
    s.damageMsg = car.lastDamage;
    s.damageAge = car.lastDamageAge;
    s.physLoad = load;
    s.awake = int(sim.active.size());
  }

  void physicsLoop() {
    using Clock = std::chrono::steady_clock;
    auto last = Clock::now();
    double accumulator = 0;
    double busy = 0, window = 0, load = 0;
    constexpr double kSlice = 1.0 / 250.0;
    while (running.load(std::memory_order_relaxed)) {
      auto now = Clock::now();
      double elapsed = std::chrono::duration<double>(now - last).count();
      last = now;
      elapsed = std::min(elapsed, 0.05);

      SharedInput in;
      {
        std::lock_guard<std::mutex> lock(inputMutex);
        in = input;
        input.c.shiftUp = input.c.shiftDown = input.c.toggleAuto = input.c.toggleSport = false;
        input.c.cycleHeight = input.c.toggleIgnition = input.c.toggleActiva = false;
        input.c.requestGear = 99;
        input.reset = input.repair = false;
      }
      if (in.reset) {
        sim = simStart;
        car = carStart;
        resetVisuals = true;
      }
      if (in.repair) {
        applyRepair();
        resetVisuals = true;
      }
      sim.grab.node = in.grabNode;
      sim.grab.target = in.grabTarget;

      if (!in.paused) accumulator += elapsed * in.timeScale;
      int steps = 0;
      auto t0 = Clock::now();
      while (accumulator >= kPhysDt && steps < kMaxStepsPerSlice) {
        sim.forces(kPhysDt);
        car.update(sim, in.c, kPhysDt);
        sim.finish(kPhysDt);
        accumulator -= kPhysDt;
        steps++;
        // Edge-triggered inputs apply to one step only.
        in.c.shiftUp = in.c.shiftDown = in.c.toggleAuto = in.c.toggleSport = false;
        in.c.cycleHeight = in.c.toggleIgnition = in.c.toggleActiva = false;
        in.c.requestGear = 99;
      }
      if (steps == kMaxStepsPerSlice) accumulator = std::min(accumulator, kPhysDt * kMaxStepsPerSlice);
      busy += std::chrono::duration<double>(Clock::now() - t0).count();
      window += elapsed;
      if (window > 0.25) {
        load = busy / window;
        busy = window = 0;
      }
      fillSnapshot(load);
      {
        std::lock_guard<std::mutex> lock(snapMutex);
        snap.paused = in.paused;
        snap.timeScale = in.timeScale;
      }

      const auto spent = std::chrono::duration<double>(Clock::now() - now).count();
      if (spent < kSlice) std::this_thread::sleep_for(std::chrono::duration<double>(kSlice - spent));
    }
  }
};

enum CamMode { CAM_CHASE, CAM_ORBIT, CAM_HOOD, CAM_DRIVER, CAM_COUNT };
const char* kCamNames[CAM_COUNT] = {"VERFOLGER", "ORBIT", "HAUBE", "FAHRER"};

struct CameraRig {
  CamMode mode = CAM_CHASE;
  double yaw = 0, pitch = 0.22, dist = 6.8;
  V3 eyeSmooth{-98, 0, 2.5}, targetSmooth{-90, 0, 0.8};
  double headingSmooth = 0;
  bool init = false;

  void update(const Snapshot& s, double dt, Camera& out, const World& world) {
    const double heading = std::atan2(s.carFwd.y, s.carFwd.x);
    if (!init) {
      headingSmooth = heading;
      init = true;
    }
    double dh = heading - headingSmooth;
    while (dh > PI) dh -= 2 * PI;
    while (dh < -PI) dh += 2 * PI;
    headingSmooth += dh * clampd(dt * 4.0, 0, 1);

    const V3 c = s.carCog;
    switch (mode) {
      case CAM_CHASE:
      case CAM_ORBIT: {
        const double base = mode == CAM_CHASE ? headingSmooth + PI : 0.0;
        const double a = base + yaw;
        V3 eye = c + V3{std::cos(a) * std::cos(pitch), std::sin(a) * std::cos(pitch), std::sin(pitch)} * dist + V3{0, 0, 0.5};
        const double gh = world.height(eye.x, eye.y) + 0.35;
        if (eye.z < gh) eye.z = gh;
        const V3 target = c + V3{0, 0, 0.45};
        const double k = clampd(dt * 16.0, 0, 1);
        eyeSmooth = lerpv(eyeSmooth, eye, k);
        targetSmooth = lerpv(targetSmooth, target, clampd(dt * 30.0, 0, 1));
        out.eye = eyeSmooth;
        out.target = targetSmooth;
        out.up = {0, 0, 1};
        out.fovY = 58.0 * PI / 180.0;
        break;
      }
      case CAM_HOOD:
        out.eye = c + s.carFwd * 0.2 + s.carUp * 0.72;
        out.target = out.eye + s.carFwd * 10.0 + s.carUp * -0.6;
        out.up = s.carUp;
        out.fovY = 62.0 * PI / 180.0;
        break;
      case CAM_DRIVER: {
        // Left-hand drive seat, with the mouse able to look around.
        out.eye = c + s.carFwd * 0.05 + s.carLeft * 0.36 + s.carUp * 0.52;
        const double la = yaw;
        const V3 look = s.carFwd * std::cos(la) + s.carLeft * std::sin(la);
        out.target = out.eye + look * 10.0 + s.carUp * (-0.35 + pitch - 0.22);
        out.up = s.carUp;
        out.fovY = 68.0 * PI / 180.0;
        break;
      }
      default:
        break;
    }
  }
};

double applyDeadzone(double v, double dz) {
  const double a = std::fabs(v);
  if (a < dz) return 0;
  return std::copysign((a - dz) / (1.0 - dz), v);
}
}  // namespace

int main(int argc, char** argv) {
  std::string shotDir;
  int scriptMode = 0;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--shots") && i + 1 < argc) shotDir = argv[++i];
    if (!strcmp(argv[i], "--script") && i + 1 < argc) scriptMode = atoi(argv[++i]);
    if (!strcmp(argv[i], "--help")) {
      printf("beanng [--shots DIR --script N]\n");
      return 0;
    }
  }

  auto* game = new Game();
  Game& G = *game;
  printf("Baue Welt und Fahrzeug...\n");
  G.world.build();
  G.sim.world = &G.world;
  G.car = buildXantia(G.sim, kSpawn, 0.0, &G.skin);
  populateWorld(G.sim, G.world);
  G.sim.finalize();
  settleVehicle(G.sim, G.car, 4.0);
  G.simStart = G.sim;
  G.carStart = G.car;
  printf("%zu Nodes, %zu Beams, %zu Objekte\n", G.sim.pos.size(), G.sim.beams.size(), G.sim.objs.size());

  // Scripted runs have no window focus, and SDL drops joystick input without focus.
  if (scriptMode > 0) SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
  if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD)) {
    fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
    return 1;
  }
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
  SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
  SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

  SDL_Window* window = SDL_CreateWindow("BeanNG - Citroen Xantia Activa", 1600, 900, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
  if (!window) {
    fprintf(stderr, "Fenster: %s\n", SDL_GetError());
    return 1;
  }
  SDL_GLContext ctx = SDL_GL_CreateContext(window);
  if (!ctx) {
    fprintf(stderr, "OpenGL: %s\n", SDL_GetError());
    return 1;
  }
  SDL_GL_SetSwapInterval(1);
  printf("OpenGL %s auf %s\n", glGetString(GL_VERSION), glGetString(GL_RENDERER));
  glEnable(GL_MULTISAMPLE);

  Renderer renderer;
  // The renderer reads only topology from this immutable copy (rest pose, triangle list, beam end
  // points). Everything that moves arrives through snapshots, so it never races the physics.
  renderer.init(G.world, G.simStart, G.skin);

  G.fillSnapshot(0);
  std::thread physics([&] { G.physicsLoop(); });

  // Script 5 drives the real input path with a virtual gamepad, so that controller handling can be
  // verified without hardware.
  const bool scriptPad = scriptMode == 5;
  const bool scriptMode_blocksAssist = scriptMode > 0 && !scriptPad;
  SDL_Joystick* vpad = nullptr;
  if (scriptPad) {
    SDL_VirtualJoystickDesc desc;
    SDL_INIT_INTERFACE(&desc);
    desc.type = SDL_JOYSTICK_TYPE_GAMEPAD;
    desc.naxes = SDL_GAMEPAD_AXIS_COUNT;
    desc.nbuttons = SDL_GAMEPAD_BUTTON_COUNT;
    desc.axis_mask = (1u << SDL_GAMEPAD_AXIS_COUNT) - 1u;
    desc.button_mask = (1u << SDL_GAMEPAD_BUTTON_COUNT) - 1u;
    desc.name = "BeanNG Virtual Pad";
    const SDL_JoystickID id = SDL_AttachVirtualJoystick(&desc);
    vpad = SDL_OpenJoystick(id);
    if (!vpad) fprintf(stderr, "virtual pad: %s\n", SDL_GetError());
  }

  SDL_Gamepad* gamepad = nullptr;
  CameraRig rig;
  Camera cam;
  HudOptions hud;
  hud.help = true;
  double helpTimer = 8.0;
  bool rmb = false, lmb = false;
  bool mouseSteer = false;
  double mouseSteerX = 0;
  double keySteer = 0;
  double timeScale = 1.0;
  bool paused = false;
  int grabNode = -1;
  V3 grabTarget;
  V3 grabPlane;
  double mx = 0, my = 0;
  bool quit = false;
  int shotCount = 0;
  bool wantShot = false;
  bool repairDone = false;
  Snapshot view;
  auto lastFrame = std::chrono::steady_clock::now();
  const auto startTime = lastFrame;

  while (!quit) {
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - lastFrame).count();
    lastFrame = now;
    dt = std::min(dt, 0.1);
    const double tNow = std::chrono::duration<double>(now - startTime).count();

    Controls edges;
    bool doReset = false, doRepair = false;

    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      switch (e.type) {
        case SDL_EVENT_QUIT:
          quit = true;
          break;
        case SDL_EVENT_GAMEPAD_ADDED:
          if (!gamepad) {
            gamepad = SDL_OpenGamepad(e.gdevice.which);
            if (gamepad) {
              hud.gamepadName = SDL_GetGamepadName(gamepad) ? SDL_GetGamepadName(gamepad) : "";
              printf("Controller: %s\n", hud.gamepadName.c_str());
            }
          }
          break;
        case SDL_EVENT_GAMEPAD_REMOVED:
          if (gamepad && SDL_GetGamepadID(gamepad) == e.gdevice.which) {
            SDL_CloseGamepad(gamepad);
            gamepad = nullptr;
          }
          break;
        case SDL_EVENT_GAMEPAD_BUTTON_DOWN:
          switch (e.gbutton.button) {
            case SDL_GAMEPAD_BUTTON_LEFT_SHOULDER: edges.shiftDown = true; break;
            case SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER: edges.shiftUp = true; break;
            case SDL_GAMEPAD_BUTTON_NORTH: rig.mode = CamMode((rig.mode + 1) % CAM_COUNT); rig.yaw = 0; break;
            case SDL_GAMEPAD_BUTTON_WEST: edges.toggleSport = true; break;
            case SDL_GAMEPAD_BUTTON_EAST: doRepair = true; break;
            case SDL_GAMEPAD_BUTTON_BACK: doReset = true; break;
            case SDL_GAMEPAD_BUTTON_START: paused = !paused; break;
            case SDL_GAMEPAD_BUTTON_DPAD_UP: edges.cycleHeight = true; break;
            case SDL_GAMEPAD_BUTTON_DPAD_DOWN: edges.toggleActiva = true; break;
            case SDL_GAMEPAD_BUTTON_DPAD_LEFT: timeScale = std::max(0.05, timeScale * 0.5); break;
            case SDL_GAMEPAD_BUTTON_DPAD_RIGHT: timeScale = std::min(1.0, timeScale * 2.0); break;
            default: break;
          }
          break;
        case SDL_EVENT_KEY_DOWN:
          if (e.key.repeat) break;
          switch (e.key.key) {
            case SDLK_ESCAPE: quit = true; break;
            case SDLK_Q: edges.shiftDown = true; break;
            case SDLK_E: edges.shiftUp = true; break;
            case SDLK_T: edges.toggleAuto = true; break;
            case SDLK_G: edges.toggleSport = true; break;
            case SDLK_H: edges.cycleHeight = true; break;
            case SDLK_V: edges.toggleActiva = true; break;
            case SDLK_I: edges.toggleIgnition = true; break;
            case SDLK_R: doReset = true; break;
            case SDLK_F5: doRepair = true; break;
            case SDLK_C: rig.mode = CamMode((rig.mode + 1) % CAM_COUNT); rig.yaw = 0; rig.pitch = 0.22; break;
            case SDLK_M:
              mouseSteer = !mouseSteer;
              mouseSteerX = 0;
              break;
            case SDLK_P: paused = !paused; break;
            case SDLK_LEFTBRACKET: timeScale = std::max(0.05, timeScale * 0.5); break;
            case SDLK_RIGHTBRACKET: timeScale = std::min(1.0, timeScale * 2.0); break;
            case SDLK_F1: hud.help = !hud.help; helpTimer = 0; break;
            case SDLK_F2: hud.debugBeams = !hud.debugBeams; break;
            case SDLK_F12: wantShot = true; break;
            default: break;
          }
          break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
          if (e.button.button == SDL_BUTTON_RIGHT) rmb = true;
          if (e.button.button == SDL_BUTTON_LEFT) {
            lmb = true;
            int w, h;
            SDL_GetWindowSizeInPixels(window, &w, &h);
            grabNode = renderer.pickNode(view, cam, w, h, e.button.x, e.button.y, 60.0);
            if (grabNode >= 0) {
              grabPlane = view.pos[grabNode];
              grabTarget = grabPlane;
            }
          }
          break;
        case SDL_EVENT_MOUSE_BUTTON_UP:
          if (e.button.button == SDL_BUTTON_RIGHT) rmb = false;
          if (e.button.button == SDL_BUTTON_LEFT) {
            lmb = false;
            grabNode = -1;
          }
          break;
        case SDL_EVENT_MOUSE_MOTION:
          mx = e.motion.x;
          my = e.motion.y;
          if (rmb) {
            rig.yaw -= e.motion.xrel * 0.005;
            rig.pitch = clampd(rig.pitch + e.motion.yrel * 0.004, -0.25, 1.45);
          }
          if (mouseSteer && !rmb) {
            int w, h;
            SDL_GetWindowSize(window, &w, &h);
            mouseSteerX = clampd(mouseSteerX - e.motion.xrel / (double(w) * 0.25), -1, 1);
          }
          break;
        case SDL_EVENT_MOUSE_WHEEL:
          rig.dist = clampd(rig.dist * std::pow(0.9, double(e.wheel.y)), 2.5, 40.0);
          break;
        default:
          break;
      }
    }

    // Continuous inputs. Keyboard steering is rate-limited and speed-sensitive like a driver's
    // hands: at speed a key press must not slam on more lock than the tyres can use.
    const bool* keys = SDL_GetKeyboardState(nullptr);
    double keyDir = 0;
    if (keys[SDL_SCANCODE_A] || keys[SDL_SCANCODE_LEFT]) keyDir += 1;
    if (keys[SDL_SCANCODE_D] || keys[SDL_SCANCODE_RIGHT]) keyDir -= 1;
    const double speedAbs = std::fabs(view.speedKmh) / 3.6;
    const double limit = clampd(5.0 / std::max(speedAbs, 1.0), 0.09, 1.0);
    const double keyTarget = keyDir * limit;
    const double keyRate = (keyDir != 0 && keyDir * keySteer >= 0) ? 1.6 : 3.2;
    keySteer += clampd(keyTarget - keySteer, -keyRate * dt, keyRate * dt);

    const double keyGas = (keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_UP]) ? 1.0 : 0.0;
    const double keyBrake = (keys[SDL_SCANCODE_S] || keys[SDL_SCANCODE_DOWN]) ? 1.0 : 0.0;
    double steer = keySteer, gas = keyGas, brake = keyBrake, hb = keys[SDL_SCANCODE_SPACE] ? 1.0 : 0.0;
    if (mouseSteer) steer = mouseSteerX;

    hud.gamepad = false;
    if (gamepad) {
      const double sx = SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_LEFTX) / 32767.0;
      const double rt = SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER) / 32767.0;
      const double lt = SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_LEFT_TRIGGER) / 32767.0;
      const double rx = applyDeadzone(SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_RIGHTX) / 32767.0, 0.15);
      const double ry = applyDeadzone(SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_RIGHTY) / 32767.0, 0.15);
      const double stick = applyDeadzone(sx, 0.08);
      if (std::fabs(stick) > 0 || rt > 0.02 || lt > 0.02) hud.gamepad = true;
      if (std::fabs(stick) > 0) {
        // Curved for fine control around the centre, and narrowed with speed.
        const double curved = std::copysign(std::pow(std::fabs(stick), 1.6), -stick);
        steer = curved * clampd(6.5 / std::max(speedAbs, 1.0), 0.14, 1.0);
      }
      gas = std::max(gas, rt);
      brake = std::max(brake, lt);
      if (SDL_GetGamepadButton(gamepad, SDL_GAMEPAD_BUTTON_SOUTH)) hb = 1.0;
      rig.yaw -= rx * dt * 2.4;
      rig.pitch = clampd(rig.pitch + ry * dt * 1.4, -0.25, 1.45);
      if (rx == 0 && rig.mode == CAM_CHASE && !rmb) rig.yaw *= std::max(0.0, 1.0 - dt * 2.0);
    }

    // Automatic box: holding the brake at a standstill selects reverse, holding the throttle
    // selects drive. In reverse the pedals swap, so that S always means "go backwards".
    Controls sendC = edges;
    if (view.autoBox && !scriptMode_blocksAssist) {
      const bool stopped = std::fabs(view.speedKmh) < 1.5;
      if (stopped && brake > 0.1 && gas < 0.05 && view.gear >= 0) sendC.requestGear = -1;
      if (stopped && gas > 0.1 && brake < 0.05 && view.gear <= 0) sendC.requestGear = 1;
      if (view.gear < 0) std::swap(gas, brake);
    }

    // Scripted runs for automated screenshots.
    bool scripted = false;
    double scriptGas = 0, scriptSteer = 0;
    if (scriptMode > 0) {
      scripted = true;
      hud.help = false;
      switch (scriptMode) {
        case 1:  // static beauty shot, then drive into the gauntlet
          scriptGas = tNow < 3.0 ? 0.0 : (view.speedKmh < 65 ? 0.9 : 0.0);
          break;
        case 2:  // down the main lane through everything
        case 4: {  // the concrete crash wall in lane y = -30
          const double yT = scriptMode == 2 ? 0.0 : -30.0;
          const double vT = scriptMode == 2 ? 65.0 : 60.0;
          const double head = std::atan2(view.carFwd.y, view.carFwd.x);
          scriptGas = view.speedKmh < vT ? 0.9 : 0.15;
          scriptSteer = clampd(-0.05 * (view.carCog.y - yT) - 0.7 * head, -0.25, 0.25);
          break;
        }
        case 3:  // walk around the parked car
          rig.mode = CAM_ORBIT;
          rig.dist = 6.5;
          rig.pitch = 0.14;
          rig.yaw = 0.35 + double(shotCount) * PI / 4.0;
          break;
        case 5: {  // end-to-end controller test: throttle, steer, brake, reverse, repair
          const auto ax = [&](SDL_GamepadAxis a, double v) { SDL_SetJoystickVirtualAxis(vpad, a, Sint16(v * 32767.0)); };
          ax(SDL_GAMEPAD_AXIS_RIGHT_TRIGGER, tNow > 2 && tNow < 6 ? 1.0 : -1.0);
          ax(SDL_GAMEPAD_AXIS_LEFT_TRIGGER, tNow > 6 && tNow < 14 ? 1.0 : -1.0);
          ax(SDL_GAMEPAD_AXIS_LEFTX, tNow > 4 && tNow < 5.5 ? 0.6 : 0.0);
          SDL_SetJoystickVirtualButton(vpad, SDL_GAMEPAD_BUTTON_EAST, tNow > 15 && tNow < 15.2);
          break;
        }
        default:
          break;
      }
      // Mouse grabber test: pull the roof upwards for a moment.
      if (scriptMode == 3 && shotCount >= 10 && shotCount < 13) {
        grabNode = int(G.simStart.objs[0].nodeStart + 16 * 11 + 8);
        grabTarget = view.carCog + V3{0, 0, 2.2};
      } else if (scriptMode == 3) {
        grabNode = -1;
      }
      if (scriptMode == 4 && tNow > 18.5 && !repairDone) {
        doRepair = true;
        repairDone = true;
      }
    }

    {
      std::lock_guard<std::mutex> lock(G.inputMutex);
      Controls& c = G.input.c;
      c.steer = scripted && !scriptPad ? scriptSteer : steer;
      c.throttle = scripted && !scriptPad ? scriptGas : gas;
      c.brake = scripted && !scriptPad ? 0.0 : brake;
      c.handbrake = hb;
      c.shiftUp |= sendC.shiftUp;
      c.shiftDown |= sendC.shiftDown;
      c.toggleAuto |= sendC.toggleAuto;
      c.toggleSport |= sendC.toggleSport;
      c.cycleHeight |= sendC.cycleHeight;
      c.toggleIgnition |= sendC.toggleIgnition;
      c.toggleActiva |= sendC.toggleActiva;
      c.requestGear = sendC.requestGear;
      G.input.reset |= doReset;
      G.input.repair |= doRepair;
      G.input.paused = paused;
      G.input.timeScale = timeScale;
      G.input.grabNode = grabNode;
      G.input.grabTarget = grabTarget;
    }
    if (doReset || doRepair) renderer.resetDamage();

    {
      std::lock_guard<std::mutex> lock(G.snapMutex);
      view.pos = G.snap.pos;
      view.beamForce = G.snap.beamForce;
      for (int i = 0; i < 4; i++) {
        view.wheels[i] = G.snap.wheels[i];
        view.flat[i] = G.snap.flat[i];
      }
      view.events = std::move(G.snap.events);
      G.snap.events.clear();
      view.carCog = G.snap.carCog;
      view.carFwd = G.snap.carFwd;
      view.carUp = G.snap.carUp;
      view.carLeft = G.snap.carLeft;
      view.carVel = G.snap.carVel;
      view.speedKmh = G.snap.speedKmh;
      view.rpm = G.snap.rpm;
      view.throttle = G.snap.throttle;
      view.brake = G.snap.brake;
      view.steer = G.snap.steer;
      view.coolant = G.snap.coolant;
      view.accum = G.snap.accum;
      view.roll = G.snap.roll;
      view.latG = G.snap.latG;
      view.lonG = G.snap.lonG;
      view.peakG = G.snap.peakG;
      view.plastic = G.snap.plastic;
      view.ram = G.snap.ram;
      view.gear = G.snap.gear;
      view.heightMode = G.snap.heightMode;
      view.autoBox = G.snap.autoBox;
      view.soft = G.snap.soft;
      view.sport = G.snap.sport;
      view.activa = G.snap.activa;
      view.running = G.snap.running;
      view.radiator = G.snap.radiator;
      view.hydraulics = G.snap.hydraulics;
      view.mount = G.snap.mount;
      view.steering = G.snap.steering;
      view.seized = G.snap.seized;
      view.damageMsg = G.snap.damageMsg;
      view.damageAge = G.snap.damageAge;
      view.physLoad = G.snap.physLoad;
      view.awake = G.snap.awake;
      view.paused = G.snap.paused;
      view.timeScale = G.snap.timeScale;
    }

    if (lmb && grabNode >= 0) {
      int w, h;
      SDL_GetWindowSizeInPixels(window, &w, &h);
      grabTarget = renderer.unprojectOnPlane(cam, w, h, mx, my, grabPlane);
    }

    if (helpTimer > 0) {
      helpTimer -= dt;
      if (helpTimer <= 0) hud.help = false;
    }
    hud.mouseSteer = mouseSteer;
    hud.cameraName = kCamNames[rig.mode];
    rig.update(view, dt, cam, G.world);

    int w, h;
    SDL_GetWindowSizeInPixels(window, &w, &h);
    if (w > 0 && h > 0) renderer.draw(view, cam, hud, w, h, dt);

    // Scripted screenshot series.
    if (!shotDir.empty()) {
      static double nextShot = 2.0;
      if (tNow >= nextShot) {
        char path[512];
        snprintf(path, sizeof path, "%s/shot_%02d.png", shotDir.c_str(), shotCount++);
        renderer.savePng(path, w, h);
        printf("t=%.1f v=%.0f km/h x=%.1f z=%.2f steer=%+.2f gear=%d gas=%.2f br=%.2f plastic=%.1f kJ awake=%d load=%.0f%% pad=%d\n", tNow, view.speedKmh, view.carCog.x, view.carCog.z, view.steer, view.gear, view.throttle, view.brake, view.plastic / 1e3, view.awake, view.physLoad * 100, hud.gamepad);
        nextShot += 1.0;
        if (shotCount >= 19) quit = true;
      }
    }
    if (wantShot) {
      char path[128];
      snprintf(path, sizeof path, "beanng_%d.png", shotCount++);
      renderer.savePng(path, w, h);
      printf("Screenshot: %s\n", path);
      wantShot = false;
    }
    SDL_GL_SwapWindow(window);
  }

  G.running = false;
  physics.join();
  if (gamepad) SDL_CloseGamepad(gamepad);
  SDL_GL_DestroyContext(ctx);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
