// BeanNG: game loop, input, cameras and the physics thread.
#include <SDL3/SDL.h>
#include <epoxy/gl.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "audio.h"
#include "props.h"
#include "render.h"
#include "vehicle.h"

namespace {
constexpr double kPhysDt = 1.0 / 2000.0;
constexpr int kMaxStepsPerSlice = 80;  // never spiral: if we fall behind, time simply slows down

V3 kSpawn{-90, 0, 0};

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
  CarRig rig;
  CarMesh mesh;
  Sim simStart;
  Vehicle carStart;

  std::mutex inputMutex;
  SharedInput input;
  std::mutex snapMutex;
  Snapshot snap;
  std::atomic<bool> running{true};
  std::atomic<bool> resetVisuals{false};
  std::atomic<bool> debugWheel{false};
  double realtimeFactor = 1;

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
    if (s.scrapes.size() < 2000) s.scrapes.insert(s.scrapes.end(), sim.scrapes.begin(), sim.scrapes.end());
    sim.scrapes.clear();
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
      v.left = w.left;
      v.skid = w.skid;
      v.omega = w.omega;
      v.fz = w.Fz;
      v.surface = w.surface;
      v.patch = w.patchPoint;
      v.deflate = w.deflate;
      v.groundN = -w.patchNormal;
      v.groundD = w.contact ? dot(w.patchPoint - v.hub, v.groundN) : 10.0;
      std::copy(w.dent, w.dent + Wheel::kDentBins, v.dent);
      v.runoutX = w.runoutX;
      v.runoutZ = w.runoutZ;
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
    s.gearboxBad = car.dmg.gearbox || car.dmg.bellHousing;
    s.tankBad = car.dmg.tank;
    s.oilBad = car.dmg.oilLeak;
    s.lhm = car.reservoir;
    s.fuel = car.fuelLitres;
    s.gear = car.shiftPhase != 0 ? car.shiftTarget : car.gear;
    s.damageMsg = car.lastDamage;
    s.damageAge = car.lastDamageAge;
    s.physLoad = load;
    s.engineLoad = clampd((car.manifold - 0.22) / 0.78, 0, 1);
    s.inputRpm = std::fabs(car.inputOmega) * 60.0 / (2.0 * PI);
    s.starter = car.starterClock > 0;
    s.shifting = car.shiftPhase != 0;
    s.awake = int(sim.active.size());
    s.objectCount = int(sim.objs.size());
    if (s.splitGeneration != sim.splitGeneration) {
      s.splitGeneration = sim.splitGeneration;
      s.nodeObj = sim.nodeObj;
    }
  }

  void physicsLoop() {
    using Clock = std::chrono::steady_clock;
    auto last = Clock::now();
    double accumulator = 0;
    double busy = 0, window = 0, load = 0, simWindow = 0;
    constexpr double kSlice = 1.0 / 250.0;
    while (running.load(std::memory_order_relaxed)) {
      auto now = Clock::now();
      const double wallElapsed = std::chrono::duration<double>(now - last).count();
      last = now;
      const double elapsed = std::min(wallElapsed, 0.05);

      SharedInput in;
      {
        std::lock_guard<std::mutex> lock(inputMutex);
        in = input;
        input.c.shiftUp = input.c.shiftDown = input.c.toggleAuto = input.c.toggleSport = false;
        input.c.cycleHeight = input.c.toggleIgnition = input.c.toggleActiva = false;
        input.c.requestGear = 99;
        input.reset = input.repair = false;
      }
      if (debugWheel.exchange(false) && car.wheels[0].deflate == 0) {
        Wheel& w = car.wheels[0];
        w.deflate = 0.001;
        w.dent[3] = 0.02f; w.dent[4] = 0.04f; w.dent[5] = 0.045f; w.dent[6] = 0.03f; w.dent[7] = 0.012f;
        w.runoutX = 0.012f;
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
      // Load is the share of wall time spent stepping, measured over the real time that passed.
      // Also report how far simulated time keeps up, which is what the driver actually feels.
      window += wallElapsed;
      simWindow += steps * kPhysDt;
      if (window > 0.25) {
        realtimeFactor = simWindow / window;
        simWindow = 0;
        load = busy / window;
        busy = window = 0;
      }
      fillSnapshot(load);
      {
        std::lock_guard<std::mutex> lock(snapMutex);
        snap.realtime = realtimeFactor;
      }
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

// Everything that tells the driver how fast and how rough it is, read from the running simulation.
// Shared by the camera and the sound, so that what shakes is also what is heard.
struct DriveFeel {
  double fzLP[4] = {};
  double rough = 0;      // suspension activity, 0..1
  double trauma = 0;     // recent violence: crashes and hard landings, 0..1
  double lastPlastic = 0;
  double plasticRate = 0;  // joules per second of metal being bent
  double roadPhase = 0, timePhase = 0;
  double speedFx = 0;    // 0 at a walk, 1 flat out, smoothed
  double lon = 0, lat = 0;
  double thump = 0;      // biggest bump this frame, 0..1
  bool valid = false;

  void update(const Snapshot& s, double dt) {
    thump = 0;
    if (dt <= 0) return;
    const double v = len(s.carVel);
    double act = 0;
    for (int i = 0; i < 4; i++) {
      const double fz = s.wheels[i].fz;
      if (!valid) fzLP[i] = fz;
      const double delta = std::fabs(fz - fzLP[i]);
      act += delta;
      thump = std::max(thump, clampd((delta - 2500.0) / 10000.0, 0, 1));
      fzLP[i] += (fz - fzLP[i]) * clampd(dt / 0.22, 0, 1);
    }
    const double target = clampd(act / 4.0 / 2600.0, 0, 1);
    rough += (target - rough) * clampd(dt / (target > rough ? 0.04 : 0.25), 0, 1);

    const double dp = valid ? std::max(s.plastic - lastPlastic, 0.0) : 0.0;
    lastPlastic = s.plastic;
    plasticRate = dp / dt;
    // A repair clears the counter: never read that as a blow.
    trauma = std::min(1.0, trauma + std::sqrt(dp / 12000.0) * 0.5 + thump * thump * 0.22);
    trauma *= std::exp(-dt * 2.8);

    roadPhase += v * dt;
    timePhase += dt;
    const double fxTarget = clampd((v - 8.0) / 50.0, 0, 1);
    speedFx += (fxTarget - speedFx) * clampd(dt / 0.5, 0, 1);
    lon += (s.lonG - lon) * clampd(dt / 0.28, 0, 1);
    lat += (s.latG - lat) * clampd(dt / 0.28, 0, 1);
    valid = true;
  }
  // Smooth noise, -1..1, from sines at unrelated frequencies. Never repeats in any noticeable way.
  static double wobble(double t, double seed) {
    return (std::sin(t * 1.00 + seed) + 0.7 * std::sin(t * 1.73 + seed * 2.1) + 0.5 * std::sin(t * 2.91 + seed * 3.7)) / 2.2;
  }
};

struct CameraRig {
  CamMode mode = CAM_CHASE;
  double yaw = 0, pitch = 0.22, dist = 6.8;
  V3 eyeSmooth{-98, 0, 2.5}, targetSmooth{-90, 0, 0.8};
  double headingSmooth = 0;
  bool init = false;

  void update(const Snapshot& s, double dt, Camera& out, const World& world, const DriveFeel& feel, bool shakeOn) {
    const double heading = std::atan2(s.carFwd.y, s.carFwd.x);
    if (!init) {
      headingSmooth = heading;
      init = true;
    }
    double dh = heading - headingSmooth;
    while (dh > PI) dh -= 2 * PI;
    while (dh < -PI) dh += 2 * PI;
    headingSmooth += dh * clampd(dt * 7.5, 0, 1);

    const V3 c = s.carCog;
    switch (mode) {
      case CAM_CHASE:
      case CAM_ORBIT: {
        const double base = mode == CAM_CHASE ? headingSmooth + PI : 0.0;
        const double a = base + yaw;
        // Wider lens and a closer camera with speed: the car keeps its size while the road rushes past.
        const double d = dist * (1.0 - 0.10 * feel.speedFx);
        V3 eye = c + V3{std::cos(a) * std::cos(pitch), std::sin(a) * std::cos(pitch), std::sin(pitch)} * d + V3{0, 0, 0.5};
        if (mode == CAM_CHASE) {
          // The camera lags behind the car's accelerations: back under power, forward under braking,
          // out to the side in a bend.
          eye += s.carFwd * clampd(-feel.lon * 0.55, -0.5, 0.5) + s.carLeft * clampd(feel.lat * 0.28, -0.35, 0.35);
        }
        const double gh = world.height(eye.x, eye.y) + 0.35;
        if (eye.z < gh) eye.z = gh;
        const V3 target = c + V3{0, 0, 0.45};
        // Rigidly attached in position: only the heading above is smoothed. Any lag in position would
        // vary with the frame time, and on screen that looks exactly like the car shaking.
        eyeSmooth = eye;
        targetSmooth = target;
        out.eye = eye;
        out.target = target;
        out.up = {0, 0, 1};
        out.fovY = (58.0 + 16.0 * feel.speedFx) * PI / 180.0;
        break;
      }
      case CAM_HOOD:
        out.eye = c + s.carFwd * 0.2 + s.carUp * 0.72;
        out.target = out.eye + s.carFwd * 10.0 + s.carUp * -0.6;
        out.up = s.carUp;
        out.fovY = (62.0 + 12.0 * feel.speedFx) * PI / 180.0;
        break;
      case CAM_DRIVER: {
        // Left-hand drive seat, with the mouse able to look around.
        out.eye = c + s.carFwd * 0.05 + s.carLeft * 0.36 + s.carUp * 0.52;
        const double la = yaw;
        const V3 look = s.carFwd * std::cos(la) + s.carLeft * std::sin(la);
        out.target = out.eye + look * 10.0 + s.carUp * (-0.35 + pitch - 0.22);
        out.up = s.carUp;
        out.fovY = (68.0 + 8.0 * feel.speedFx) * PI / 180.0;
        break;
      }
      default:
        break;
    }
    out.inside = mode == CAM_DRIVER;
    if (shakeOn) addShake(s, out, feel);
  }

  // The camera turns about the car instead of moving through space, so the world shakes and the car
  // stays steady in the middle of the picture. A shaking car would look like a physics fault.
  void addShake(const Snapshot& s, Camera& out, const DriveFeel& f) {
    const double v = len(s.carVel);
    const double speed = clampd(v / 60.0, 0, 1.3);
    const double inCar = mode == CAM_HOOD || mode == CAM_DRIVER ? 1.6 : 1.0;
    // Road texture: frequency goes with speed, so that it is felt to speed up.
    const double fine = f.roadPhase * 2.4;
    const double fast = (0.0011 * speed * speed + 0.0028 * f.rough * (0.4 + speed)) * inCar;
    // Slow heave from the bigger waves in the road.
    const double slow = f.roadPhase * 0.55;
    const double heave = 0.0016 * f.rough * inCar;
    // A blow: large, quick, and over soon.
    const double blow = 0.040 * f.trauma * f.trauma;
    const double t = f.timePhase * 38.0;
    const double pitchA = fast * DriveFeel::wobble(fine, 1.3) + heave * DriveFeel::wobble(slow, 4.2) + blow * DriveFeel::wobble(t, 0.7);
    const double yawA = 0.6 * fast * DriveFeel::wobble(fine * 1.13, 2.9) + 0.6 * blow * DriveFeel::wobble(t * 1.21, 3.1);
    const double rollA = 1.2 * fast * DriveFeel::wobble(fine * 0.87, 5.5) + heave * DriveFeel::wobble(slow * 1.3, 0.2) + 0.8 * blow * DriveFeel::wobble(t * 0.83, 6.0);
    const V3 f0 = norm(out.target - out.eye);
    const V3 right = norm(cross(f0, out.up));
    const V3 up = cross(right, f0);
    const double D = len(out.target - out.eye);
    out.target += (right * yawA + up * pitchA) * D;
    out.up = norm(up + right * rollA);
    // Only the blow itself also throws the camera about.
    out.eye += (right * DriveFeel::wobble(t * 0.9, 1.9) + up * DriveFeel::wobble(t * 1.1, 2.3)) * (0.05 * f.trauma * f.trauma);
  }
};

// Turns the frame's simulation state into what the synth needs.
struct SoundDriver {
  AudioSynth synth;
  double volume = 0.8;
  bool muted = false;
  double metalClock = 0, glassClock = 0, thumpClock = 0;
  int lastGear = 1;
  double scrapeLP = 0;

  void update(const Snapshot& s, const Camera& cam, const DriveFeel& feel, CamMode mode, double dt) {
    AudioFeed f;
    f.rpm = float(s.rpm);
    f.load = float(s.engineLoad);
    f.engineOn = s.running;
    f.starter = s.starter;
    f.inputRpm = float(s.inputRpm);
    f.speed = float(len(s.carVel));
    double skid = 0, soft = 0, grounded = 0;
    for (const WheelView& w : s.wheels) {
      const bool loose = w.surface == SURF_GRASS || w.surface == SURF_GRAVEL;
      if (!loose) skid = std::max(skid, w.skid);
      if (w.fz > 50.0) {
        grounded += 1.0;
        if (loose) soft += 1.0;
      }
    }
    f.skid = float(skid);
    f.dirt = grounded > 0 ? float(soft / grounded) : 0.f;
    f.rough = float(feel.rough);
    const double scrapeTarget = clampd(double(s.scrapes.size()) / std::max(dt, 1e-3) / 900.0, 0, 1);
    scrapeLP += (scrapeTarget - scrapeLP) * clampd(dt / 0.05, 0, 1);
    f.scrape = float(scrapeLP);
    const double camDist = len(cam.eye - s.carCog);
    f.listener = float(1.0 / (1.0 + std::max(camDist - 8.0, 0.0) / 14.0));
    f.interior = mode == CAM_DRIVER ? 1.f : 0.f;
    f.master = (muted || s.paused) ? 0.f : float(volume);
    synth.setFeed(f);

    // Where a sound is heard from.
    const V3 fwd = norm(cam.target - cam.eye), right = norm(cross(fwd, cam.up));
    auto place = [&](V3 p, float& pan, float& gain) {
      const V3 r = p - cam.eye;
      const double d = len(r);
      pan = float(clampd(dot(r, right) / std::max(d, 1.0), -1, 1)) * 0.8f;
      gain = float(1.0 / (1.0 + d / 16.0));
    };

    metalClock -= dt;
    glassClock -= dt;
    thumpClock -= dt;
    // Bending metal, however much of it. Several per second while a crash goes on.
    if (feel.plasticRate > 4000.0 && metalClock <= 0) {
      const float st = float(clampd(std::log10(feel.plasticRate / 4000.0) / 2.5, 0.15, 1.0));
      float pan, gain;
      place(s.carCog, pan, gain);
      synth.trigger(Sfx::Metal, st, pan, gain * (0.55f + 0.55f * st));
      metalClock = 0.05;
    }
    int glassN = 0, crunchN = 0;
    V3 glassAt, crunchAt;
    for (const SimEvent& e : s.events) {
      if (e.kind == EV_GLASS) {
        glassAt = e.p;
        glassN++;
      } else if (e.kind == EV_CRUNCH) {
        crunchAt = e.p;
        crunchN++;
      }
    }
    if (glassN > 0 && glassClock <= 0) {
      float pan, gain;
      place(glassAt, pan, gain);
      synth.trigger(Sfx::Glass, clampd(glassN / 6.0, 0.3, 1.0), pan, gain);
      glassClock = 0.12;
    }
    // Something tore, also out in the scenery: fences, signs, a stone wall.
    if (crunchN > 0 && metalClock <= 0) {
      float pan, gain;
      place(crunchAt, pan, gain);
      synth.trigger(Sfx::Metal, clampd(0.2 + crunchN / 12.0, 0.2, 1.0), pan, gain * 0.9f);
      metalClock = 0.05;
    }
    if (feel.thump > 0.08 && thumpClock <= 0) {
      float pan, gain;
      place(s.carCog, pan, gain);
      synth.trigger(Sfx::Thump, float(feel.thump), pan * 0.5f, gain * 0.8f);
      thumpClock = 0.09;
    }
    if (s.gear != lastGear) {
      float pan, gain;
      place(s.carCog, pan, gain);
      synth.trigger(Sfx::Clunk, 0.7f, pan, gain);
      lastGear = s.gear;
    }
  }
};

double applyDeadzone(double v, double dz) {
  const double a = std::fabs(v);
  if (a < dz) return 0;
  return std::copysign((a - dz) / (1.0 - dz), v);
}
}  // namespace

// On a hybrid laptop GL would land on the integrated GPU. This asks for the discrete NVIDIA card
// through PRIME render offload. It has to happen before anything loads a GL driver, and it only takes
// effect if that card and its driver exist. BEANNG_GPU=integrated keeps the integrated one.
static void SDLCALL audioCallback(void* user, SDL_AudioStream* stream, int additional, int) {
  static std::vector<float> buffer;
  const int frames = additional / int(2 * sizeof(float));
  if (frames <= 0) return;
  buffer.resize(size_t(frames) * 2);
  static_cast<AudioSynth*>(user)->render(buffer.data(), frames);
  SDL_PutAudioStreamData(stream, buffer.data(), frames * int(2 * sizeof(float)));
}

// A WAV of the game's real sound, so that it can be measured without listening.
static void writeWav(const std::string& path, const std::vector<float>& samples) {
  FILE* f = fopen(path.c_str(), "wb");
  if (!f) return;
  const u32 bytes = u32(samples.size() * sizeof(float));
  auto u32le = [&](u32 v) { fwrite(&v, 4, 1, f); };
  auto u16le = [&](uint16_t v) { fwrite(&v, 2, 1, f); };
  fwrite("RIFF", 1, 4, f);
  u32le(36 + bytes);
  fwrite("WAVEfmt ", 1, 8, f);
  u32le(16);
  u16le(3);  // IEEE float
  u16le(2);
  u32le(AudioSynth::kRate);
  u32le(AudioSynth::kRate * 8);
  u16le(8);
  u16le(32);
  fwrite("data", 1, 4, f);
  u32le(bytes);
  fwrite(samples.data(), 1, bytes, f);
  fclose(f);
}

static void preferDiscreteGpu() {
  const char* choice = getenv("BEANNG_GPU");
  if (choice && !strcmp(choice, "integrated")) return;
  constexpr const char* kNvidiaEgl = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json";
  FILE* f = fopen("/proc/driver/nvidia/version", "r");
  if (f) {
    fclose(f);
    setenv("__NV_PRIME_RENDER_OFFLOAD", "1", 0);
    setenv("__GLX_VENDOR_LIBRARY_NAME", "nvidia", 0);
    setenv("__VK_LAYER_NV_optimus", "NVIDIA_only", 0);
    FILE* e = fopen(kNvidiaEgl, "r");
    if (e) {
      fclose(e);
      setenv("__EGL_VENDOR_LIBRARY_FILENAMES", kNvidiaEgl, 0);
    }
  } else {
    setenv("DRI_PRIME", "1", 0);  // AMD or nouveau discrete cards under Mesa
  }
}

int main(int argc, char** argv) {
  preferDiscreteGpu();
  std::string shotDir, wavPath;
  int scriptMode = 0;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--shots") && i + 1 < argc) shotDir = argv[++i];
    if (!strcmp(argv[i], "--script") && i + 1 < argc) scriptMode = atoi(argv[++i]);
    if (!strcmp(argv[i], "--wav") && i + 1 < argc) wavPath = argv[++i];
    if (!strcmp(argv[i], "--spawn") && i + 2 < argc) {
      kSpawn.x = atof(argv[++i]);
      kSpawn.y = atof(argv[++i]);
    }
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
  // Script 15 starts on the country road, on its long southern stretch.
  if (scriptMode == 15) {
    const RoadPoint& p = G.world.roads()[0].pts[1200];
    kSpawn.x = p.x;
    kSpawn.y = p.y;
  }
  kSpawn.z = G.world.height(kSpawn.x, kSpawn.y);
  G.car = buildXantia(G.sim, kSpawn, 0.0, &G.rig);
  populateWorld(G.sim, G.world);
  G.sim.finalize();
  settleVehicle(G.sim, G.car, 4.0);
  G.simStart = G.sim;
  G.carStart = G.car;
  if (!G.mesh.load("assets/xantia.mesh")) {
    fprintf(stderr, "assets/xantia.mesh fehlt. Starte das Spiel mit ./run.sh, das erzeugt das Modell.\n");
    return 1;
  }
  // Bind to the as-built pose: that is what the mesh was modelled against.
  G.mesh.bind(G.simStart.rest, G.rig.groups, kSpawn);
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
  SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 0);  // the renderer does its own MSAA

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
  renderer.init(G.world, G.simStart, G.rig, G.mesh, kSpawn);

  // Sound. It stays silent for automated runs, which must not make noise on the machine they run on,
  // and without a sound card the game simply runs without it.
  auto* sound = new SoundDriver();
  SDL_AudioStream* audioStream = nullptr;
  std::vector<float> wavSamples;
  double wavDebt = 0;
  if (scriptMode == 0 && wavPath.empty()) {
    if (SDL_InitSubSystem(SDL_INIT_AUDIO)) {
      SDL_SetHint(SDL_HINT_AUDIO_DEVICE_SAMPLE_FRAMES, "512");
      const SDL_AudioSpec spec{SDL_AUDIO_F32, 2, AudioSynth::kRate};
      audioStream = SDL_OpenAudioDeviceStream(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &spec, audioCallback, &sound->synth);
      if (audioStream) SDL_ResumeAudioStreamDevice(audioStream);
    }
    if (!audioStream) fprintf(stderr, "Kein Ton: %s\n", SDL_GetError());
    else printf("Ton: %s\n", SDL_GetAudioDeviceName(SDL_GetAudioStreamDevice(audioStream)));
  }

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
  DriveFeel feel;
  bool shakeOn = true;
  {
    // Short name for the HUD: "NVIDIA RTX 3500 Ada Generation Laptop GPU/PCIe/SSE2" -> "RTX 3500".
    std::string r = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
    const size_t cut = r.find('/');
    if (cut != std::string::npos) r.resize(cut);
    for (const char* drop : {"NVIDIA ", "Mesa ", " Laptop GPU", " Generation", "(R)"}) {
      const size_t at = r.find(drop);
      if (at != std::string::npos) r.erase(at, strlen(drop));
    }
    hud.gpuName = r;
  }
  double fpsClock = 0;
  int fpsFrames = 0;
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
    fpsClock += dt;
    if (++fpsFrames >= 30) {
      hud.fps = fpsFrames / std::max(fpsClock, 1e-3);
      fpsFrames = 0;
      fpsClock = 0;
    }

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
            case SDLK_B: hud.motionBlur = !hud.motionBlur; break;
            case SDLK_J: shakeOn = !shakeOn; break;
            case SDLK_N: sound->muted = !sound->muted; break;
            case SDLK_PLUS:
            case SDLK_EQUALS:
            case SDLK_KP_PLUS: sound->volume = std::min(1.5, sound->volume + 0.1); break;
            case SDLK_MINUS:
            case SDLK_KP_MINUS: sound->volume = std::max(0.0, sound->volume - 0.1); break;
            case SDLK_F1: hud.help = !hud.help; helpTimer = 0; break;
            case SDLK_F2: hud.debugBeams = !hud.debugBeams; break;
            case SDLK_F3: hud.xray = !hud.xray; break;
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
    const double keyRate = (keyDir != 0 && keyDir * keySteer >= 0) ? 2.4 : 4.0;
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
        const double curved = std::copysign(std::pow(std::fabs(stick), 1.35), -stick);
        steer = curved * clampd(8.0 / std::max(speedAbs, 1.0), 0.16, 1.0);
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
    bool scriptHand = false;
    double scriptBrake = 0;
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
        case 7:  // burst of consecutive frames with a fixed camera, to detect render flicker
          rig.mode = CAM_ORBIT;
          rig.dist = 5.5;
          rig.pitch = 0.16;
          rig.yaw = 0.6;
          break;
        case 9:  // x-ray walk around the mechanicals
          hud.xray = true;
          rig.mode = CAM_ORBIT;
          rig.dist = 4.2;
          rig.pitch = 0.42;
          rig.yaw = 0.5 + double(shotCount) * PI / 3.0;
          break;
        case 12:  // lay rubber: launch, then a hard turn with the handbrake on
          rig.mode = CAM_ORBIT;
          rig.dist = 16.0;
          rig.pitch = 0.75;
          rig.yaw = 2.6;
          scriptGas = tNow < 6.0 ? 1.0 : 0.7;
          scriptSteer = tNow < 6.0 ? 0.0 : 0.65;
          G.input.c.handbrake = 0;
          scriptHand = tNow > 6.0 && tNow < 8.0;
          break;
        case 14:
        case 13:  // scrape the left flank along a wall, then show it
          rig.mode = CAM_ORBIT;
          rig.dist = 4.6;
          rig.pitch = 0.10;
          rig.yaw = 1.75;
          scriptGas = scriptMode == 14 ? 0.0 : (view.speedKmh < 40 && view.carCog.x < 93 ? 0.6 : 0.0);
          scriptSteer = view.carCog.x < 91 ? 0.012 : 0.0;
          scriptBrake = scriptMode == 14 || view.carCog.x > 93 ? 1.0 : 0.0;
          break;
        case 15: {  // flat out along the country road, with the sound recorded
          const auto& pts = G.world.roads()[0].pts;
          size_t best = 0;
          double bd = 1e18;
          for (size_t i = 0; i < pts.size(); i++) {
            const double dd = std::hypot(pts[i].x - view.carCog.x, pts[i].y - view.carCog.y);
            if (dd < bd) {
              bd = dd;
              best = i;
            }
          }
          const double look = 12.0 + std::fabs(view.speedKmh) * 0.22;
          const RoadPoint& t = pts[(best + size_t(look / 1.4)) % pts.size()];
          const double want = std::atan2(t.y - view.carCog.y, t.x - view.carCog.x);
          double err = want - std::atan2(view.carFwd.y, view.carFwd.x);
          while (err > PI) err -= 2 * PI;
          while (err < -PI) err += 2 * PI;
          scriptSteer = clampd(err * 1.6, -0.35, 0.35);
          scriptGas = tNow < 2.0 ? 0.0 : (std::fabs(err) > 0.12 ? 0.4 : 1.0);
          scriptBrake = view.speedKmh > 175 ? 0.3 : 0.0;
          break;
        }
        case 16:  // the steering wheel from the back seat, turning
          scriptSteer = std::sin(tNow * 1.2) * 0.7;
          break;
        case 10:  // close look at a tyre going flat and a dented rim
          rig.mode = CAM_ORBIT;
          rig.dist = 2.6;
          rig.pitch = 0.08;
          rig.yaw = -1.25 + 0.0;
          G.debugWheel = true;
          break;
        case 8:  // burst while driving steadily, chase camera
          scriptGas = view.speedKmh < 50 ? 0.5 : 0.12;
          scriptSteer = clampd(-0.05 * view.carCog.y - 0.7 * std::atan2(view.carFwd.y, view.carFwd.x), -0.2, 0.2);
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
      c.brake = scripted && !scriptPad ? scriptBrake : brake;
      c.handbrake = scriptHand ? 1.0 : hb;
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
    if (doReset) renderer.clearSkidMarks();

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
      view.scrapes = std::move(G.snap.scrapes);
      G.snap.scrapes.clear();
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
      view.gearboxBad = G.snap.gearboxBad;
      view.tankBad = G.snap.tankBad;
      view.oilBad = G.snap.oilBad;
      view.lhm = G.snap.lhm;
      view.fuel = G.snap.fuel;
      view.damageMsg = G.snap.damageMsg;
      view.damageAge = G.snap.damageAge;
      view.physLoad = G.snap.physLoad;
      view.engineLoad = G.snap.engineLoad;
      view.inputRpm = G.snap.inputRpm;
      view.starter = G.snap.starter;
      view.shifting = G.snap.shifting;
      view.realtime = G.snap.realtime;
      view.awake = G.snap.awake;
      view.objectCount = G.snap.objectCount;
      view.splitGeneration = G.snap.splitGeneration;
      view.nodeObj = G.snap.nodeObj;
      view.paused = G.snap.paused;
      view.timeScale = G.snap.timeScale;
    }

    if (scriptMode == 14) {
      // Visual test: strokes along the left flank, as if dragged along a wall at two heights.
      for (double z : {0.62, 0.72, 0.48})
        for (double xd = -1.2; xd <= 1.2; xd += 0.3)
          view.scrapes.push_back({kSpawn + V3{xd, 0.87, z}, V3{1, 0, 0}, 1.f});
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
    feel.update(view, paused ? 0.0 : dt);
    if (doReset || doRepair) feel.lastPlastic = 0, feel.valid = false;
    rig.update(view, dt, cam, G.world, feel, shakeOn);
    if (scriptMode == 10) {  // close-up of the front left wheel
      cam.target = view.wheels[0].hub + V3{0, 0, -0.05};
      cam.eye = cam.target + V3{0.55, 1.15, 0.12};
    }
    if (scriptMode == 13 || scriptMode == 14) {  // close look along the left flank
      cam.target = view.carCog + view.carFwd * -0.2 + view.carUp * 0.05;
      cam.eye = cam.target + view.carLeft * 2.7 + view.carFwd * 0.5 + view.carUp * 0.25;
    }
    if (scriptMode == 16) {
      cam.eye = view.carCog + view.carFwd * -0.35 + view.carLeft * 0.36 + view.carUp * 0.55;
      cam.target = view.carCog + view.carFwd * 0.70 + view.carLeft * 0.36 + view.carUp * 0.28;
      cam.up = view.carUp;
      cam.fovY = 45.0 * PI / 180.0;
      cam.inside = true;
    }
    if (scriptMode == 17) {  // front arch, from the side
      cam.target = view.carCog + view.carFwd * 1.25 + view.carLeft * 0.8 + view.carUp * -0.1;
      cam.eye = cam.target + view.carLeft * 1.7 + view.carFwd * -0.05 + view.carUp * 0.1;
      cam.up = view.carUp;
      cam.fovY = 45.0 * PI / 180.0;
    }
    if (scriptMode == 11) {  // close-up of a rear corner
      const V3 corner = view.carCog + view.carFwd * -2.25 + view.carLeft * 0.55 + view.carUp * 0.5;
      cam.target = corner;
      cam.eye = corner + view.carFwd * (shotCount % 2 ? -1.0 : -1.3) + view.carLeft * (shotCount % 2 ? 1.1 : 0.2) + view.carUp * (shotCount % 3 == 2 ? 0.1 : 0.45);
    }

    int w, h;
    SDL_GetWindowSizeInPixels(window, &w, &h);
    sound->update(view, cam, feel, rig.mode, dt);
    if (!wavPath.empty()) {
      wavDebt += dt * AudioSynth::kRate;
      const int n = int(wavDebt);
      wavDebt -= n;
      if (n > 0) {
        const size_t at = wavSamples.size();
        wavSamples.resize(at + size_t(n) * 2);
        sound->synth.render(wavSamples.data() + at, n);
      }
    }
    if (w > 0 && h > 0) renderer.draw(view, cam, hud, w, h, dt);

    // Scripted screenshot series.
    if (!shotDir.empty()) {
      static double nextShot = 2.0;
      if (tNow >= nextShot) {
        char path[512];
        snprintf(path, sizeof path, "%s/shot_%02d.png", shotDir.c_str(), shotCount++);
        renderer.savePng(path, w, h);
        printf("fps %.0f ", hud.fps);
        printf("t=%.1f v=%.0f km/h x=%.1f z=%.2f steer=%+.2f gear=%d gas=%.2f br=%.2f plastic=%.1f kJ awake=%d load=%.0f%% pad=%d\n", tNow, view.speedKmh, view.carCog.x, view.carCog.z, view.steer, view.gear, view.throttle, view.brake, view.plastic / 1e3, view.awake, view.physLoad * 100, hud.gamepad);
        const bool burst = scriptMode == 7 || scriptMode == 8;
        nextShot += burst ? 0.0 : 1.0;
        if (burst && nextShot < 6.0) nextShot = 6.0;
        if (shotCount >= (burst ? 6 : (scriptMode == 9 ? 7 : (scriptMode == 12 ? 13 : 19)))) quit = true;
        if (scriptMode == 14 && tNow > 6.0) quit = true;
        if (scriptMode == 13 && view.carCog.x > 93 && std::fabs(view.speedKmh) < 2) quit = true;
      }
    }
    if (wantShot) {
      char path[128];
      snprintf(path, sizeof path, "beanng_%d.png", shotCount++);
      renderer.savePng(path, w, h);
      printf("Screenshot: %s\n", path);
      wantShot = false;
    }
    {
      auto ts = std::chrono::steady_clock::now();
      SDL_GL_SwapWindow(window);
      if (getenv("BEANNG_PROFILE")) {
        static int n = 0;
        if (++n % 30 == 0) fprintf(stderr, "swap %.1f ms, whole frame %.1f ms\n", std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - ts).count(), std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - now).count());
      }
    }
  }

  if (!wavPath.empty()) writeWav(wavPath, wavSamples);
  if (audioStream) SDL_DestroyAudioStream(audioStream);
  G.running = false;
  physics.join();
  if (gamepad) SDL_CloseGamepad(gamepad);
  SDL_GL_DestroyContext(ctx);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
