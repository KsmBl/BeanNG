// BeanNG: game loop, input, cameras and the physics thread.
#include <SDL3/SDL.h>
#include <epoxy/gl.h>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "audio.h"
#include "duel.h"
#include "duelnet.h"
#include "props.h"
#include "render.h"
#include "tank.h"
#include "vehicle.h"

// The game's own settings: they outlast a change of vehicle and are kept on disk between runs.
struct GameSettings {
  bool motionBlur = true, shake = true, damageView = true, stats = true, muted = false;
  bool fullscreen = false, vsync = true, farShadows = true;
  double volume = 0.8, fov = 0, mouse = 1;
};
static GameSettings gSet;
static constexpr const char* kSettingsFile = "settings.cfg";

static void loadSettings() {
  FILE* f = fopen(kSettingsFile, "r");
  if (!f) return;
  char key[64];
  double v;
  while (fscanf(f, "%63s %lf", key, &v) == 2) {
    const std::string k = key;
    if (k == "motion_blur") gSet.motionBlur = v != 0;
    else if (k == "camera_shake") gSet.shake = v != 0;
    else if (k == "damage_view") gSet.damageView = v != 0;
    else if (k == "stats") gSet.stats = v != 0;
    else if (k == "muted") gSet.muted = v != 0;
    else if (k == "fullscreen") gSet.fullscreen = v != 0;
    else if (k == "vsync") gSet.vsync = v != 0;
    else if (k == "far_shadows") gSet.farShadows = v != 0;
    else if (k == "volume") gSet.volume = clampd(v, 0, 1.5);
    else if (k == "fov") gSet.fov = clampd(v, -15, 25);
    else if (k == "mouse") gSet.mouse = clampd(v, 0.2, 3);
  }
  fclose(f);
}

static void saveSettings() {
  FILE* f = fopen(kSettingsFile, "w");
  if (!f) return;
  fprintf(f, "motion_blur %d\ncamera_shake %d\ndamage_view %d\nstats %d\nmuted %d\nfullscreen %d\nvsync %d\nfar_shadows %d\nvolume %.2f\nfov %.0f\nmouse %.2f\n",
          gSet.motionBlur, gSet.shake, gSet.damageView, gSet.stats, gSet.muted, gSet.fullscreen, gSet.vsync, gSet.farShadows, gSet.volume, gSet.fov, gSet.mouse);
  fclose(f);
}

namespace {

enum CamMode { CAM_CHASE, CAM_ORBIT, CAM_HOOD, CAM_DRIVER, CAM_COUNT };
const char* kCamNames[CAM_COUNT] = {"Verfolger", "Orbit", "Haube", "Fahrer"};

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
    // 42 tonnes hardly notice what shakes a car: a heavy vehicle reads its loads on a far coarser scale.
    const double heavy = s.tank ? 0.14 : 1.0;
    thump *= heavy;
    const double target = clampd(act / 4.0 / 2600.0 * heavy, 0, 1);
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
  bool tank = false;  // the camera is the gunner's eye: it stays on target while the hull turns
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
    if (tank) {
      updateTank(s, out, world, feel, shakeOn);
      return;
    }
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

  void updateTank(const Snapshot& s, Camera& out, const World& world, const DriveFeel& feel, bool shakeOn) {
    // yaw and pitch are in the world, so that turning the hull does not swing the aim.
    const V3 back{std::cos(yaw) * std::cos(pitch), std::sin(yaw) * std::cos(pitch), std::sin(pitch)};
    out.up = {0, 0, 1};
    out.inside = false;
    if (mode == CAM_HOOD || mode == CAM_DRIVER) {
      // Gunner's sight, and through its optics a narrower view.
      out.eye = s.sight;
      out.target = s.sight - back * 50.0;
      out.fovY = (mode == CAM_HOOD ? 18.0 : 62.0) * PI / 180.0;
    } else {
      const V3 pivot = s.carCog + V3{0, 0, 2.4};
      V3 eye = pivot + back * (dist + (mode == CAM_ORBIT ? 8.0 : 5.5));
      const double gh = world.height(eye.x, eye.y) + 0.5;
      if (eye.z < gh) eye.z = gh;
      out.eye = eye;
      out.target = pivot - back * 8.0;
      out.fovY = (60.0 + 8.0 * feel.speedFx) * PI / 180.0;
    }
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
  double metalClock = 0, glassClock = 0, thumpClock = 0, crackClock = 0;
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
    f.master = (gSet.muted || s.paused) ? 0.f : float(gSet.volume);
    synth.setFeed(f);

    // Where a sound is heard from.
    const V3 fwd = norm(cam.target - cam.eye), right = norm(cross(fwd, cam.up));
    auto place = [&](V3 p, float& pan, float& gain) {
      const V3 r = p - cam.eye;
      const double d = len(r);
      pan = float(clampd(dot(r, right) / std::max(d, 1.0), -1, 1)) * 0.8f;
      gain = float(1.0 / (1.0 + d / 16.0));
    };

    f.fire = s.scorch > 0.35 ? float(clampd(s.scorch, 0, 1)) : 0.f;
    if (s.conventional) {
      f.cylinders = 8;
      f.exhaustPitch = s.rpmMax < 5000 ? 0.72f : 1.15f;  // a big slow diesel, or the One:1
      f.turbo = float(s.boost) * (s.rpmMax < 5000 ? 0.45f : 1.0f);
    }
    if (s.tank) {
      f.cylinders = 10;
      f.exhaustPitch = 0.55f;
      f.tracks = float(clampd(f.speed / 2.0, 0, 1));
      f.skid = 0;
    }
    synth.setFeed(f);
    metalClock -= dt;
    glassClock -= dt;
    thumpClock -= dt;
    crackClock -= dt;
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
      if (e.kind == EV_BLAST) {
        float pan, gain;
        place(e.p, pan, gain);
        synth.trigger(Sfx::Boom, 1.f, pan, std::max(gain, 0.18f) * 1.4f);
      } else if (e.kind == EV_WOOD) {
        if (crackClock <= 0) {
          float pan, gain;
          place(e.p, pan, gain);
          synth.trigger(Sfx::Crack, 1.f, pan, gain * 1.2f);
          crackClock = 0.07;
        }
      } else if (e.kind == EV_BRANCH) {
        if (crackClock <= 0) {
          float pan, gain;
          place(e.p, pan, gain);
          synth.trigger(Sfx::Crack, 0.35f, pan, gain);
          crackClock = 0.05;
        }
      } else if (e.kind == EV_MUZZLE) {
        synth.trigger(Sfx::Gun, 1.f, 0.f, 1.1f);
      } else if (e.kind == EV_GLASS) {
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

// Command line, the window and the sound outlive a session with one vehicle.
static std::string shotDir, wavPath;
static int scriptMode = 0;
static size_t treeIndex = 0;
static double treeRun = 30.0, treeSpeed = 60.0;
static V3 gSpawnBase{-90, 0, 0};
static SDL_Window* window = nullptr;
static SoundDriver* sound = nullptr;
static std::vector<float> wavSamples;
static double wavDebt = 0;


// Where the server is: asked for in the menu, and remembered between runs.
static std::string gUserName = "Spieler";
static std::string gServerHost = "127.0.0.1";
static int gServerPort = net::kDefaultPort;
static bool gConnectNow = false;
static int gPickTurret = TK_STANDARD;
static constexpr const char* kServerFile = "server.cfg";

static void loadServerAddress() {
  FILE* f = fopen(kServerFile, "r");
  if (!f) return;
  char host[256], name[256];
  int port;
  const int got = fscanf(f, "%255s %d %255s", host, &port, name);
  if (got >= 2) {
    gServerHost = host;
    gServerPort = port;
  }
  if (got == 3) gUserName = name;
  fclose(f);
}

static void saveServerAddress() {
  FILE* f = fopen(kServerFile, "w");
  if (!f) return;
  // On one line without spaces; a name with a space in it comes back with underscores.
  std::string name = gUserName;
  for (char& ch : name)
    if (ch == ' ') ch = '_';
  fprintf(f, "%s %d %s\n", gServerHost.c_str(), gServerPort, name.c_str());
  fclose(f);
}

// Everything the window's thread hands to the network thread.
struct SharedInput {
  Controls c;
  bool ready = false;
  bool sprint = false;
  double yaw = 0, pitch = 0;
  u8 items[IT_COUNT] = {};
};

// The connection to the server and the copy of the world it describes. Nothing is simulated here: both tanks and
// everything else go where the server says, a tenth of a second late so that there is always a next position.
struct Game {
  Duel duel;
  World& world = duel.world;
  Sim& sim = duel.sim;
  Sim& simStart = duel.simStart;
  CarRig rig;
  CarMesh mesh;

  std::mutex inputMutex;
  SharedInput input;
  std::mutex snapMutex;
  Snapshot snap;
  std::atomic<bool> running{true};
  std::atomic<bool> resetVisuals{false};

  // Asked for from the window's thread.
  enum Command { CMD_NONE, CMD_CONNECT, CMD_LEAVE };
  std::atomic<int> command{CMD_NONE};

  // What the window's thread reads, under snapMutex.
  enum NetState { NET_OFF, NET_CONNECTING, NET_IN };
  int netState = NET_OFF;
  std::string netError;
  std::string names[2];
  int phase = Duel::WAITING;
  double phaseClock = 0;
  int slot = 0;
  int myVehicle = -1;
  bool playing = false;  // connected, and the match on
  Soldier me;            // on foot: where this machine thinks it is, ahead of what the server has answered

  void fillSnapshot(const char* message) {
    std::lock_guard<std::mutex> lock(snapMutex);
    Snapshot& s = snap;
    s.pos = sim.pos;
    s.tank = true;
    s.tankTracks.resize(Duel::kTanks);
    for (int i = 0; i < Duel::kTanks; i++) {
      const Tank& t = duel.tanks[i];
      Snapshot::TankTrack& tt = s.tankTracks[size_t(i)];
      for (int k = 0; k < 2; k++) {
        t.trackPath(sim, k, tt.path[k]);
        tt.left[k] = t.trackWidthDir(k);
        tt.phase[k] = t.trackPhase[k];
        tt.speed[k] = t.trackOmega[k] * Tank::kSprocketR;
      }
      s.parkedScorch[i] = t.destroyed ? float(clampd(t.burnTime / 3.0, 0.15, 1.0)) : float(t.damage * 0.3);
    }
    s.events.insert(s.events.end(), sim.events.begin(), sim.events.end());
    if (s.events.size() > 2000) s.events.clear();
    sim.events.clear();
    sim.scrapes.clear();
    if (s.craters.size() != world.craters().size()) s.craters = world.craters();
    s.awake = int(sim.active.size());
    s.objectCount = int(sim.objs.size());
    s.realtime = 1;
    s.paused = false;
    s.timeScale = 1;
    s.damageMsg = message;
    s.damageAge = message[0] ? 0.0 : 99.0;

    // The instruments go with whatever one is in. On foot only the camera's middle goes with one.
    const Tank& tank = duel.tanks[myVehicle >= 0 && myVehicle < Duel::kTanks ? myVehicle : Duel::tankIndex(slot, TK_STANDARD)];
    const VehicleFrame* frame = myVehicle >= 0 ? &duel.frames[myVehicle] : nullptr;
    s.foot.onFoot = playing && myVehicle < 0;
    s.foot.inCar = myVehicle >= Duel::kTanks;
    s.foot.hp = duel.players[slot].soldier.hp;
    s.foot.p = me.p;
    memcpy(s.foot.items, duel.players[slot].items, IT_COUNT);
    s.carCog = frame ? frame->cog : me.p;
    s.carFwd = frame ? frame->fwd : V3{1, 0, 0};
    s.carUp = frame ? frame->up : V3{0, 0, 1};
    s.carLeft = frame ? frame->left : V3{0, 1, 0};
    s.carVel = myVehicle >= 0 ? (frame->fwd * frame->speed) : me.v;
    s.tankArmour = clampd(1.0 - tank.damage, 0, 1);
    s.tankAmmo = tank.ammo;
    s.tankDead = tank.destroyed;
    s.enemyCog = duel.frames[Duel::kTanks].cog;
    s.parts.clear();
    const auto part = [&](const char* label, float x, float y, float w, float h, double health) { s.parts.push_back({label, x, y, w, h, float(1.0 - health)}); };
    part("WANNE", 18, 34, 64, 170, 1.0 - tank.damage);
    part("MOTOR", 26, 150, 48, 50, tank.mod[TM_ENGINE]);
    part("TURM", 30, 84, 40, 60, tank.mod[TM_RING]);
    part("ROHR", 47, 4, 6, 80, tank.mod[TM_GUN]);
    part(nullptr, 4, 34, 12, 170, tank.trackOff[0] ? 0.0 : tank.mod[TM_TRACK_L]);
    part(nullptr, 84, 34, 12, 170, tank.trackOff[1] ? 0.0 : tank.mod[TM_TRACK_R]);
    for (int i = 0; i < 2; i++) {
      s.trackLeft[i] = tank.trackWidthDir(i);
      s.trackPhase[i] = tank.trackPhase[i];
      s.trackSpeed[i] = tank.trackOmega[i] * Tank::kSprocketR;
    }
    for (int i = 0; i < 4; i++) {
      // What the camera shake and the sound read from the wheels.
      WheelView& v = s.wheels[i];
      const Tank::RoadWheel& w = tank.wheels[size_t(i) * 4 % 14];
      v.fz = w.Fn * 0.25;
      v.surface = w.soft > 0.5 ? u8(SURF_GRAVEL) : u8(SURF_ASPHALT);
      v.skid = 0;
    }
    s.speedKmh = frame ? frame->speed * 3.6 : 0.0;
    s.rpm = myVehicle >= 0 ? (s.foot.inCar ? duel.cars[size_t(myVehicle - Duel::kTanks)].rpm() : tank.rpm()) : 0;
    s.rpmMax = s.foot.inCar ? 7000 : 2500;
    s.rpmRed = s.foot.inCar ? 6100 : 2300;
    s.throttle = tank.throttleAct;
    s.brake = tank.brakeAct;
    s.steer = tank.steerAct;
    s.latG = tank.latAcc / 9.81;
    s.lonG = tank.lonAcc / 9.81;
    s.gear = s.foot.inCar ? duel.cars[size_t(myVehicle - Duel::kTanks)].gear : tank.gear;
    s.running = myVehicle >= 0;
    s.fuel = tank.fuel;
    s.engineLoad = tank.loadNow;
    s.inputRpm = 0;
    s.turretYaw = tank.turretYaw;
    s.gunPitch = tank.gunPitch;
    s.reload = std::max(tank.reload, 0.0);
    s.reloadTime = tank.reloadTime;
    s.muzzle = tank.muzzle;
    s.gunDir = tank.gunDir;
    s.sight = tank.sight;
    s.missileLive = tank.missileLive;
    s.missileP = tank.missileP;
    s.missileV = tank.missileV;
    s.missiles = tank.missiles;
    s.repairKits = duel.players[slot].items[IT_REPAIR];
  }

  struct NetFrame {
    u32 tick = 0;
    V3 cog;
    Tank::NetState state;
    std::vector<i16> offsets;
  };

  void netLoop() {
    using Clock = std::chrono::steady_clock;
    constexpr double kViewDelay = 0.10;
    const auto start = Clock::now();
    const auto clock = [&] { return std::chrono::duration<double>(Clock::now() - start).count(); };
    std::unique_ptr<net::Socket> sock;
    net::Addr server;
    u32 token = 0;
    int state = NET_OFF;
    double connectStart = 0, nextHello = 0;
    std::deque<NetFrame> frames[Duel::kTanks];
    u32 match = 0, metaTick = 0, blastsDone = 0;
    int phaseNow = Duel::WAITING, winner = -1, mySlot = 0, vehicleNow = -1;
    // What each car last said, and where it is shown: the middle follows smoothly, the shape comes with the packets.
    struct CarShown {
      V3 target, cog;
      bool known = false;
      std::vector<i16> offsets;
    } shown[Duel::kCars];
    // Walking answers at once, and is checked against the server's answer afterwards.
    Soldier shownSoldier[2];
    struct Past {
      u32 seq;
      V3 p;
    };
    std::deque<Past> past;
    double walkDebt = 0;
    u8 carrying[IT_COUNT] = {};
    double repairNow = 0;
    u32 ackedInput = 0;
    V3 serverP;
    bool freshAnswer = false;
    u32 wins[2] = {};
    float clockNow = 0;
    double timeOffset = 0;
    bool haveOffset = false;
    double lastHeard = 0, nextInput = 0, last = clock();
    net::InputMsg out;
    std::string error;

    const auto resetWorld = [&] {
      world.clearCraters();
      sim = simStart;
      for (int i = 0; i < Duel::kTanks; i++) {
        duel.tanks[i] = duel.tanksStart[i];
        frames[i].clear();
      }
      duel.cars = duel.carsStart;
      duel.placePlayers();
      duel.updateFrames();
      for (CarShown& c : shown) c = CarShown{};
      me = duel.players[mySlot].soldier;
      past.clear();
      blastsDone = 0;
      metaTick = 0;
      vehicleNow = -1;
      resetVisuals = true;
    };
    const auto disconnect = [&](const std::string& why) {
      if (sock && state == NET_IN) {
        net::Writer bye(net::P_BYE);
        bye.put(token);
        for (int i = 0; i < 3; i++) sock->send(bye.data(), bye.size(), server);
      }
      sock.reset();
      state = NET_OFF;
      error = why;
      phaseNow = Duel::WAITING;
      winner = -1;
      match = 0;
      out = net::InputMsg{};
      resetWorld();
    };

    while (running.load(std::memory_order_relaxed)) {
      const double now = clock();
      const double dt = std::max(now - last, 1e-4);
      last = now;
      const int cmd = command.exchange(CMD_NONE);
      if (cmd == CMD_LEAVE) disconnect("");
      if (cmd == CMD_CONNECT) {
        disconnect("");
        std::string host;
        int port;
        {
          std::lock_guard<std::mutex> lock(inputMutex);
          host = gServerHost;
          port = gServerPort;
        }
        sock = std::make_unique<net::Socket>();
        if (port <= 0 || port > 65535) error = "Ungültiger Port";
        else if (!net::resolve(host.c_str(), u16(port), server)) error = "Adresse nicht gefunden: " + host;
        else if (!sock->open(0)) error = "Netzwerk nicht verfügbar";
        else {
          state = NET_CONNECTING;
          connectStart = now;
          nextHello = 0;
        }
        if (state != NET_CONNECTING) sock.reset();
      }

      SharedInput in;
      {
        std::lock_guard<std::mutex> lock(inputMutex);
        in = input;
        input.c.requestGear = 99;
        input.c.fire = input.c.launch = input.c.repair = input.c.use = false;
      }
      if (in.c.fire) out.fireCount++;
      if (in.c.launch) out.launchCount++;
      if (in.c.repair) out.repairCount++;
      if (in.c.use) out.useCount++;

      if (state == NET_CONNECTING) {
        if (now - connectStart > 4.0) {
          disconnect("Keine Antwort von " + net::toString(server));
        } else if (now >= nextHello) {
          nextHello = now + 0.25;
          net::Writer w(net::P_HELLO);
          w.put(net::kProtocol);
          {
            std::lock_guard<std::mutex> lock(inputMutex);
            for (int i = 0; i < net::kNameBytes; i++) w.put(i + 1 < net::kNameBytes && i < int(gUserName.size()) ? gUserName[size_t(i)] : char(0));
          }
          while (w.size() < 64) w.put(u8(0));
          sock->send(w.data(), w.size(), server);
        }
      }

      u8 buf[net::kMaxPacket];
      net::Addr from;
      int n;
      while (sock && (n = sock->recv(buf, sizeof buf, from)) >= 0) {
        if (!(from == server)) continue;
        net::Reader r(buf, size_t(n));
        u8 kind = 0;
        if (!r.header(kind)) continue;
        if (kind == net::P_FULL) {
          u8 why = 0;
          r.get(why);
          disconnect(why ? "Der Server hat eine andere Spielversion" : "Der Server ist voll");
          break;
        }
        if (kind == net::P_WELCOME) {
          u16 protocol = 0;
          u8 s = 0;
          u32 nodes = 0, beams = 0, t = 0;
          r.get(protocol);
          r.get(s);
          r.get(t);
          r.get(nodes);
          r.get(beams);
          if (!r.ok() || protocol != net::kProtocol || s > 1 || state != NET_CONNECTING) continue;
          if (nodes != sim.pos.size() || beams != sim.beams.size()) {
            disconnect("Die Spielversion passt nicht zum Server");
            break;
          }
          mySlot = s;
          token = t;
          out.token = t;
          state = NET_IN;
          lastHeard = now;
          haveOffset = false;
          continue;
        }
        if (state != NET_IN) continue;
        u32 tick = 0, id = 0;
        r.get(tick);
        r.get(id);
        if (!r.ok() || id < match) continue;
        if (id > match) {
          resetWorld();
          match = id;
        }
        lastHeard = now;
        if (kind == net::P_TANK) {
          u8 idx = 0;
          r.get(idx);
          NetFrame f;
          f.tick = tick;
          f.cog = r.getV3();
          net::readState(r, f.state);
          u16 count = 0;
          r.get(count);
          // Comes back around every now and then whether anything has happened or not.
          if (!r.ok() || idx >= Duel::kTanks || count != duel.tankNodes) continue;
          f.offsets.resize(size_t(count) * 3);
          for (i16& v : f.offsets) r.get(v);
          if (!r.ok()) continue;
          auto& q = frames[idx];
          if (!q.empty() && q.back().tick >= tick) continue;
          q.push_back(std::move(f));
          while (q.size() > 16) q.pop_front();
          // Where this machine's clock stands against the server's, without the jitter of any one packet.
          const double seen = now - double(tick) / net::kSendRate;
          timeOffset = haveOffset ? timeOffset + (seen - timeOffset) * 0.05 : seen;
          haveOffset = true;
        } else if (kind == net::P_META) {
          if (tick <= metaTick) continue;
          u8 ph = 0, mask = 0;
          i8 win = 0;
          float pc = 0;
          r.get(ph);
          r.get(win);
          r.get(wins[0]);
          r.get(wins[1]);
          r.get(pc);
          r.get(mask);
          i8 vehicles[2] = {};
          for (int i = 0; i < 2; i++) {
            r.get(vehicles[i]);
            Soldier& sd = shownSoldier[i];
            sd.p = r.getV3();
            sd.v = r.getV3();
            float yaw = 0, pitch = 0, hp = 0, stride = 0;
            r.get(yaw);
            r.get(pitch);
            r.get(hp);
            r.get(stride);
            sd.yaw = yaw;
            sd.pitch = pitch;
            sd.hp = hp;
            sd.stride = stride;
          }
          u32 ack = 0;
          r.get(ack);
          for (int i = 0; i < IT_COUNT; i++) {
            u8 count = 0;
            r.get(count);
            carrying[i] = count;
          }
          float rc = 0;
          r.get(rc);
          repairNow = rc;
          u32 first = 0;
          u8 count = 0;
          r.get(first);
          r.get(count);
          if (!r.ok()) continue;
          metaTick = tick;
          phaseNow = ph;
          winner = win;
          clockNow = pc;
          vehicleNow = vehicles[mySlot];
          ackedInput = ack;
          serverP = shownSoldier[mySlot].p;
          freshAnswer = true;
          for (int i = 0; i < 2; i++) {
            duel.players[i].vehicle = vehicles[i];
            duel.players[i].ready = mask & (4 << i);
            duel.players[i].soldier.hp = shownSoldier[i].hp;
          }
          for (u32 i = first; i < first + count; i++) {
            const V3 c = r.getV3();
            float impulse = 0, radius = 0;
            r.get(impulse);
            r.get(radius);
            if (!r.ok()) break;
            if (i == blastsDone) {
              sim.explode(c, impulse, radius);
              blastsDone++;
            }
          }
          u8 nEv = 0;
          r.get(nEv);
          for (int i = 0; i < nEv; i++) {
            const V3 p = r.getV3();
            const V3 v = r.getV3();
            float strength = 0;
            u8 evKind = 0;
            r.get(strength);
            r.get(evKind);
            if (!r.ok() || evKind > EV_ROCK) break;
            sim.event(EventKind(evKind), p, v, strength);
          }
        } else if (kind == net::P_NAMES) {
          std::lock_guard<std::mutex> lock(snapMutex);
          for (int i = 0; i < 2; i++) {
            char name[net::kNameBytes];
            for (char& ch : name) r.get(ch);
            name[net::kNameBytes - 1] = 0;
            if (r.ok()) names[i] = name;
          }
        } else if (kind == net::P_CAR) {
          u8 k = 0;
          r.get(k);
          V3 cog = r.getV3(), fwd = r.getV3(), left = r.getV3(), up = r.getV3();
          float speed = 0, rpm = 0;
          i8 gear = 0;
          u8 exploded = 0;
          r.get(speed);
          r.get(rpm);
          r.get(gear);
          r.get(exploded);
          u32 at = 0;
          u16 count = 0;
          r.get(at);
          r.get(count);
          if (!r.ok() || k >= Duel::kCars) continue;
          const u32 first = duel.carFirst[k];
          const u32 total = (k + 1 < Duel::kCars ? duel.carFirst[k + 1] : duel.carsEnd) - first;
          if (u64(at) + count > total) continue;
          CarShown& c = shown[k];
          if (c.offsets.empty()) c.offsets.assign(size_t(total) * 3, 0);
          bool whole = true;
          for (size_t i = size_t(at) * 3; i < (size_t(at) + count) * 3; i++) r.get(c.offsets[i]);
          whole = r.ok();
          if (!whole) continue;
          c.target = cog;
          if (!c.known) c.cog = cog;
          c.known = true;
          Vehicle& v = duel.cars[k];
          v.fwd = fwd;
          v.left = left;
          v.up = up;
          v.speed = speed;
          v.gear = gear;
          v.exploded = exploded != 0;
          v.engineOmega = rpm * 2.0 * PI / 60.0;
        } else if (kind == net::P_SCENERY) {
          u8 count = 0;
          r.get(count);
          for (int i = 0; i < count; i++) {
            u32 k = 0;
            r.get(k);
            const V3 p = r.getV3();
            if (!r.ok()) break;
            if (k >= sim.firstSceneryNode && k < sim.pos.size()) sim.pos[k] = p;
          }
        }
      }
      if (state == NET_IN && now - lastHeard > 5.0) disconnect("Verbindung zum Server verloren");

      // Every tank we have word of, at the moment a little in the past that lies between two frames.
      const double viewTime = (now - timeOffset - kViewDelay) * net::kSendRate;
      u32 viewTick = 0;
      for (int side = 0; side < Duel::kTanks; side++) {
        const auto& q = frames[side];
        if (q.empty()) continue;
        size_t b = 0;
        while (b + 1 < q.size() && double(q[b].tick) < viewTime) b++;
        const NetFrame& fb = q[b];
        const NetFrame& fa = b > 0 ? q[b - 1] : fb;
        const double span = double(fb.tick) - double(fa.tick);
        const double a = span > 0 ? clampd((viewTime - double(fa.tick)) / span, 0, 1) : 1.0;
        viewTick = std::max(viewTick, fa.tick);
        Tank& t = duel.tanks[side];
        const u32 first = duel.tankFirst[side];
        const size_t count = fb.offsets.size() / 3;
        const V3 cog = fa.cog + (fb.cog - fa.cog) * a;
        for (size_t k = 0; k < count; k++) {
          const V3 oa{double(fa.offsets[k * 3]), double(fa.offsets[k * 3 + 1]), double(fa.offsets[k * 3 + 2])};
          const V3 ob{double(fb.offsets[k * 3]), double(fb.offsets[k * 3 + 1]), double(fb.offsets[k * 3 + 2])};
          sim.pos[first + k] = cog + (oa + (ob - oa) * a) * (1.0 / net::kNodeScale);
        }
        t.applyNetState(sim, fb.state, dt);
      }

      for (int k = 0; k < Duel::kCars; k++) {
        CarShown& c = shown[k];
        if (!c.known || c.offsets.empty()) continue;
        c.cog += (c.target - c.cog) * clampd(dt * 22.0, 0, 1);
        const u32 first = duel.carFirst[k];
        const size_t count = c.offsets.size() / 3;
        // Pieces of a car not yet heard of stay where they are.
        for (size_t i = 0; i < count; i++) {
          const V3 off{double(c.offsets[i * 3]), double(c.offsets[i * 3 + 1]), double(c.offsets[i * 3 + 2])};
          if (off.x != 0 || off.y != 0 || off.z != 0) sim.pos[first + i] = c.cog + off * (1.0 / net::kNodeScale);
        }
        duel.cars[size_t(k)].cog = c.cog;
      }
      duel.updateFrames();

      // On foot one's own legs answer at once.
      duel.players[mySlot].vehicle = vehicleNow;
      const bool playingNow = state == NET_IN && (phaseNow == Duel::PLAYING || phaseNow == Duel::CHOOSING || phaseNow == Duel::FINISHED);
      if (vehicleNow >= 0) {
        me = shownSoldier[mySlot];
        past.clear();
      } else if (playingNow) {
        WalkInput wi;
        wi.forward = in.c.throttle - in.c.brake;
        wi.strafe = in.c.steer;
        wi.yaw = in.yaw;
        wi.jump = in.c.handbrake > 0.5;
        wi.sprint = in.sprint;
        walkDebt += dt;
        walkDebt = std::min(walkDebt, 0.1);
        while (walkDebt >= Duel::kWalkDt) {
          walkDebt -= Duel::kWalkDt;
          duel.walk(me, wi, Duel::kWalkDt);
        }
        me.pitch = in.pitch;
        if (freshAnswer) {
          freshAnswer = false;
          // Where it thought it was when the server's answer was made, against the answer.
          while (past.size() > 1 && int32_t(past.front().seq - ackedInput) < 0) past.pop_front();
          const V3 was = past.empty() ? me.p : past.front().p;
          const V3 err = serverP - was;
          if (len(err) > 4.0) {
            me.p = serverP;
            past.clear();
          } else {
            const V3 fix = err * 0.15;
            me.p += fix;
            for (Past& q : past) q.p += fix;
          }
        }
      } else {
        me = duel.players[mySlot].soldier;
      }
      // Both soldiers drawn: oneself where one thinks one is, the other as the server last said.
      for (int i = 0; i < 2; i++) {
        const bool hidden = state != NET_IN || !playingNow || duel.players[i].vehicle >= 0;
        Duel::poseSoldier(sim, duel.soldierFirst[i], i == mySlot ? me : shownSoldier[i], hidden, 1);
      }

      if (state == NET_IN && now >= nextInput) {
        nextInput = now + 1.0 / 60.0;
        out.seq++;
        out.steer = float(in.c.steer);
        out.throttle = float(in.c.throttle);
        out.brake = float(in.c.brake);
        out.handbrake = float(in.c.handbrake);
        out.aim = in.c.aimPoint;
        out.indirect = in.c.indirect;
        out.requestGear = i8(in.c.requestGear == 99 ? 0 : in.c.requestGear);
        out.ackBlasts = blastsDone;
        out.ackMatch = match;
        out.viewTick = viewTick;
        out.yaw = float(in.yaw);
        out.pitch = float(in.pitch);
        out.sprint = in.sprint;
        if (vehicleNow < 0) past.push_back({out.seq, me.p});
        while (past.size() > 200) past.pop_front();
        out.ready = in.ready ? 1 : 0;
        for (int i = 0; i < IT_COUNT; i++) out.items[i] = in.items[i];
        net::Writer w(net::P_INPUT);
        net::write(w, out);
        sock->send(w.data(), w.size(), server);
      }

      char msg[128] = "";
      if (state == NET_IN) {
        const Tank* mine = vehicleNow >= 0 && vehicleNow < Duel::kTanks ? &duel.tanks[vehicleNow] : nullptr;
        if (phaseNow == Duel::WAITING) snprintf(msg, sizeof msg, "Warte auf den zweiten Spieler");
        else if (phaseNow == Duel::CHOOSING && in.ready) snprintf(msg, sizeof msg, "Warte auf den Gegner  ·  %.0f s", std::max(0.0, Duel::kChooseTime - double(clockNow)));
        else if (phaseNow == Duel::FINISHED) {
          std::lock_guard<std::mutex> lock(snapMutex);
          snprintf(msg, sizeof msg, "%s  ·  %s %u : %u %s", winner < 0 ? "Unentschieden" : (winner == mySlot ? "Sieg!" : "Niederlage"), names[mySlot].c_str(), wins[mySlot], wins[1 - mySlot], names[1 - mySlot].c_str());
        }
        else if (repairNow > 0) snprintf(msg, sizeof msg, "Repariere  ·  %.0f s", repairNow);
        else if (mine && mine->missileLive) snprintf(msg, sizeof msg, "Rakete in der Luft  ·  Maus lenkt  ·  Feuer sprengt");
        else if (vehicleNow >= 0) snprintf(msg, sizeof msg, "E: aussteigen");
        else if (phaseNow == Duel::PLAYING) {
          const int v = duel.nearestVehicle(me.p, 2.5);
          const char* name = v < 0 ? "" : (v < Duel::kTanks ? duel.tanks[v].spec().name : (v - Duel::kTanks) % Duel::kCarKinds == 0 ? "Xantia" : ((v - Duel::kTanks) % Duel::kCarKinds == 1 ? "One:1" : "Hummer"));
          if (v >= 0) snprintf(msg, sizeof msg, carrying[IT_REPAIR] > 0 ? "E: einsteigen  ·  R: reparieren  ·  %s" : "E: einsteigen  ·  %s", name);
        }
      }
      memcpy(duel.players[mySlot].items, carrying, IT_COUNT);
      {
        std::lock_guard<std::mutex> lock(snapMutex);
        netState = state;
        netError = error;
        phase = phaseNow;
        phaseClock = clockNow;
        slot = mySlot;
        myVehicle = vehicleNow;
        playing = state == NET_IN && phaseNow == Duel::PLAYING;
      }
      fillSnapshot(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
    if (state == NET_IN) disconnect("");
  }
};

static void runGame() {
  SDL_GLContext ctx = SDL_GL_CreateContext(window);
  if (!ctx) {
    fprintf(stderr, "OpenGL: %s\n", SDL_GetError());
    return;
  }
  glClearColor(.05f, .05f, .06f, 1.f);
  glClear(GL_COLOR_BUFFER_BIT);
  SDL_GL_SwapWindow(window);
  auto* game = new Game();
  Game& G = *game;
  printf("Baue Welt und Fahrzeuge...\n");
  G.duel.build();
  // This copy never takes a step of its own.
  G.sim.mirror = true;
  G.sim.logBlasts = false;
  G.simStart.mirror = true;
  G.simStart.logBlasts = false;
  for (int i = 0; i < Duel::kTanks; i++) {
    G.duel.tanks[i].applyNetState(G.sim, G.duel.tanks[i].netState(), 0.01);
    G.duel.tanksStart[i] = G.duel.tanks[i];
  }

  // Everything that stands on the map, in one mesh, each bound to its own nodes.
  G.rig.noseX = 0;
  G.rig.carWheels = false;
  G.rig.scratches = false;
  G.mesh.noseX = 0;
  G.rig.playerLinks = 0;
  const V3 origin{};
  static const char* carFiles[Duel::kCarKinds] = {"assets/xantia.mesh", "assets/one1.mesh", "assets/hummer.mesh"};
  for (int k = 0; k < Duel::kVehicles; k++) {
    const bool isTank = k < Duel::kTanks;
    const char* file = isTank ? turretSpec(TurretKind(k % TK_COUNT)).mesh : carFiles[(k - Duel::kTanks) % Duel::kCarKinds];
    CarMesh other;
    if (!other.load(file)) {
      fprintf(stderr, "%s fehlt. Starte das Spiel mit ./run.sh, das erzeugt die Modelle.\n", file);
      return;
    }
    const CarRig& r = G.duel.rigs[k];
    std::vector<CarMesh::ParkedHub> hubs;
    if (!isTank) {
      const Vehicle& v = G.duel.carsStart[size_t(k - Duel::kTanks)];
      for (size_t w = 0; w < v.wheels.size(); w++) {
        const Wheel& wh = v.wheels[w];
        const std::string name = std::to_string(k) + ":wheel" + std::to_string(w);
        std::vector<u32> nodes{wh.axIn, wh.axOut};
        nodes.insert(nodes.end(), std::begin(wh.ring), std::end(wh.ring));
        G.rig.groups.push_back({name, nodes});
        hubs.push_back({(G.simStart.rest[wh.axIn] + G.simStart.rest[wh.axOut]) * 0.5, wh.left, w >= 2 ? r.rearWidth : 1.f, "wheel" + std::to_string(w)});
      }
    }
    G.mesh.addParked(other, k, G.duel.spawn[k], r.noseX, origin, hubs);
    for (const NodeGroup& g : r.groups) G.rig.groups.push_back({std::to_string(k) + ":" + g.name, g.nodes});
    G.rig.links.insert(G.rig.links.end(), r.links.begin(), r.links.end());
    u32 first, end;
    if (isTank) {
      first = G.duel.tankFirst[k];
      end = first + G.duel.tankNodes;
    } else {
      first = G.simStart.objs[G.duel.carsStart[size_t(k - Duel::kTanks)].obj].nodeStart;
      end = k + 1 < Duel::kVehicles ? G.simStart.objs[G.duel.carsStart[size_t(k + 1 - Duel::kTanks)].obj].nodeStart : G.duel.carsEnd;
    }
    G.rig.parkedNodes.push_back({first, end - first});
  }
  G.mesh.bind(G.simStart.rest, G.rig.groups, origin);
  // A soldier is rods between its eight nodes: body, head, legs, arms and what it holds.
  for (int i = 0; i < 2; i++) {
    const u32 f = G.duel.soldierFirst[i];
    const u8 cloth = i == 0 ? MM_OLIVE : MM_SAND;
    G.rig.links.push_back({f, f + 1, 0.19f, cloth, 0});
    G.rig.links.push_back({f + 1, f + 2, 0.11f, MM_DARK, 0});
    G.rig.links.push_back({f, f + 3, 0.09f, cloth, 0});
    G.rig.links.push_back({f, f + 4, 0.09f, cloth, 0});
    G.rig.links.push_back({f + 1, f + 5, 0.055f, cloth, 0});
    G.rig.links.push_back({f + 1, f + 6, 0.055f, cloth, 0});
    G.rig.links.push_back({f + 6, f + 7, 0.03f, MM_BLACK, 0});
  }
  printf("%zu Nodes, %zu Beams, %zu Objekte\n", G.sim.pos.size(), G.sim.beams.size(), G.sim.objs.size());

  SDL_GL_SetSwapInterval(gSet.vsync ? 1 : 0);
  printf("OpenGL %s auf %s\n", glGetString(GL_VERSION), glGetString(GL_RENDERER));
  glEnable(GL_MULTISAMPLE);

  Renderer renderer;
  // The renderer reads only topology from this immutable copy (rest pose, triangle list, beam end
  // points). Everything that moves arrives through snapshots, so it never races the physics.
  renderer.init(G.world, G.simStart, G.rig, G.mesh, origin);

  G.me = G.duel.players[0].soldier;
  G.fillSnapshot("");
  std::thread network([&] { G.netLoop(); });
  if (gConnectNow) G.command = Game::CMD_CONNECT;

  SDL_Gamepad* gamepad = nullptr;
  CameraRig rig;
  rig.tank = true;
  rig.yaw = PI;
  rig.pitch = 0.20;
  rig.dist = 8.0;
  Camera cam;
  HudOptions hud;
  enum { PANEL_NONE, PANEL_GAME, PANEL_MAIN, PANEL_KEYS, PANEL_ONLINE, PANEL_LOADOUT };
  int panelKind = PANEL_MAIN;
  bool textEntry = false;
  int panelSel = 0;
  bool fromMain = false;  // a page opened from the main menu goes back to it
  struct Item {
    std::string label, value;
    float fill;
    std::function<void(int)> change;
  };
  std::vector<Item> items;
  bool relativeMouse = false;
  DriveFeel feel;
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
  double keySteer = 0;
  (void)keySteer;
  bool quit = false;
  int shotCount = 0;
  bool wantShot = false;
  Snapshot view;
  // What this player is going to take into the match.
  u8 pickItems[IT_COUNT] = {};
  bool pickReady = false;
  int lastPhase = -1;
  // The map, and the point on it the artillery lays on.
  bool mapOpen = false;
  bool hasMark = false;
  V3 markPoint;
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

    int netState, phase, slot, myVehicle;
    double phaseClock;
    std::string netError;
    {
      std::lock_guard<std::mutex> lock(G.snapMutex);
      netState = G.netState;
      phase = G.phase;
      phaseClock = G.phaseClock;
      slot = G.slot;
      myVehicle = G.myVehicle;
      netError = G.netError;
    }
    const bool inMatch = netState == Game::NET_IN && phase == Duel::PLAYING;
    const bool inVehicle = inMatch && myVehicle >= 0, onFoot = inMatch && myVehicle < 0;
    const bool inTank = inVehicle && myVehicle < Duel::kTanks;
    // The choosing comes up by itself; afterwards it is gone by itself.
    if (phase != lastPhase) {
      if (phase == Duel::CHOOSING && netState == Game::NET_IN) {
        pickReady = false;
        panelKind = PANEL_LOADOUT;
        panelSel = 0;
        fromMain = false;
      } else if (panelKind == PANEL_LOADOUT) {
        panelKind = PANEL_NONE;
      }
      if (phase == Duel::PLAYING) {
        // Looking across the yard towards the enemy. The camera's yaw is in the world.
        rig.yaw = G.world.base(slot).facing + PI;
        hasMark = false;
        mapOpen = false;
        rig.pitch = 0.20;
      }
      lastPhase = phase;
    }

    Controls edges;
    const auto leave = [&] { G.command = Game::CMD_LEAVE; };

    // The list on screen, rebuilt from the live state each frame.
    items.clear();
    auto toggle = [&](const char* name, bool on, std::function<void()> flip) {
      items.push_back({name, on ? "AN" : "AUS", -1.f, [flip](int) { flip(); }});
    };
    auto percent = [&](const char* name, double& val, double lo, double hi, double step) {
      char b[32];
      snprintf(b, sizeof b, "%3.0f%%", val * 100.0);
      items.push_back({name, b, float((val - lo) / (hi - lo)), [&val, lo, hi, step](int d) { val = clampd(val + d * step, lo, hi); }});
    };
    auto page = [&](const char* name, std::function<void()> act) {
      items.push_back({name, "", -1.f, [act](int) { act(); }});
    };
    auto openPage = [&](int which) {
      fromMain = panelKind == PANEL_MAIN;
      panelKind = which;
      panelSel = 0;
    };
    if (panelKind == PANEL_ONLINE) {
      items.push_back({"Benutzername", gUserName + (panelSel == 0 ? "_" : ""), -1.f, [&](int) {}});
      items.push_back({"Server-Adresse", gServerHost + (panelSel == 1 ? "_" : ""), -1.f, [&](int) {}});
      items.push_back({"Port", std::to_string(gServerPort) + (panelSel == 2 ? "_" : ""), -1.f, [&](int) {}});
      page(netState == Game::NET_CONNECTING ? "Verbinde..." : "Verbinden", [&] {
        saveServerAddress();
        G.command = Game::CMD_CONNECT;
      });
      if (!netError.empty()) items.push_back({netError, "", -1.f, [](int) {}});
    } else if (panelKind == PANEL_LOADOUT) {
      for (int i = 0; i < IT_COUNT; i++) {
        char name[64], b[16];
        snprintf(name, sizeof name, "%s  (%d %s)", kItems[i].name, kItems[i].slots, kItems[i].slots == 1 ? "Slot" : "Slots");
        snprintf(b, sizeof b, "%d", pickItems[i]);
        items.push_back({name, b, -1.f, [&, i](int d) {
                           u8 next[IT_COUNT];
                           memcpy(next, pickItems, sizeof next);
                           if (d < 0 && next[i] == 0) return;
                           next[i] = u8(int(next[i]) + d);
                           if (slotsUsed(next) <= kLoadoutSlots) memcpy(pickItems, next, sizeof next);
                           pickReady = false;
                         }});
      }
      {
        char b[32];
        snprintf(b, sizeof b, "%d / %d", slotsUsed(pickItems), kLoadoutSlots);
        items.push_back({"Belegte Slots", b, float(slotsUsed(pickItems)) / float(kLoadoutSlots), [](int) {}});
      }
      page("Bereit", [&] {
        pickReady = true;
        panelKind = PANEL_NONE;
      });
    } else if (panelKind == PANEL_MAIN) {
      if (netState == Game::NET_IN) {
        page("Weiterspielen", [&] { panelKind = PANEL_NONE; });
        if (phase == Duel::CHOOSING) page("Ausrüstung wählen", [&] { panelKind = PANEL_LOADOUT; panelSel = 0; pickReady = false; });
        page("Spiel verlassen", [&] { leave(); });
      } else {
        page("Online spielen", [&] { openPage(PANEL_ONLINE); });
      }
      page("Spiel-Einstellungen", [&] { openPage(PANEL_GAME); });
      page("Steuerung", [&] { openPage(PANEL_KEYS); });
      page("Beenden", [&] { quit = true; });
    } else if (panelKind == PANEL_GAME) {
      toggle("Vollbild", gSet.fullscreen, [&] {
        gSet.fullscreen = !gSet.fullscreen;
        SDL_SetWindowFullscreen(window, gSet.fullscreen);
      });
      toggle("VSync", gSet.vsync, [&] {
        gSet.vsync = !gSet.vsync;
        SDL_GL_SetSwapInterval(gSet.vsync ? 1 : 0);
      });
      toggle("Bewegungsunschärfe", gSet.motionBlur, [&] { gSet.motionBlur = !gSet.motionBlur; });
      toggle("Kamera-Wackeln", gSet.shake, [&] { gSet.shake = !gSet.shake; });
      toggle("Ferne Schatten", gSet.farShadows, [&] { gSet.farShadows = !gSet.farShadows; });
      {
        char b[32];
        snprintf(b, sizeof b, "%+.0f°", gSet.fov);
        items.push_back({"Sichtfeld", b, float((gSet.fov + 15.0) / 40.0), [&](int d) { gSet.fov = clampd(gSet.fov + d * 2.5, -15, 25); }});
      }
      percent("Mausempfindlichkeit", gSet.mouse, 0.2, 3.0, 0.1);
      toggle("Ton", !gSet.muted, [&] { gSet.muted = !gSet.muted; });
      percent("Lautstärke", gSet.volume, 0.0, 1.5, 0.1);
      toggle("Schadensanzeige", gSet.damageView, [&] { gSet.damageView = !gSet.damageView; });
      toggle("Leistungsanzeige", gSet.stats, [&] { gSet.stats = !gSet.stats; });
    } else if (panelKind == PANEL_KEYS) {
      auto key = [&](const char* what, const char* k) { items.push_back({what, k, -1.f, [&](int) {}}); };
      key("Gas und Bremse, im Stand rückwärts", "W S");
      key("Lenken", "A D");
      key("Bremse", "Leertaste");
      key("Zielen", "Maus");
      key("Feuer", "Linke Maustaste, F");
      key("Rakete starten, fliegt mit der Maus", "G");
      key("Einsteigen und aussteigen", "E");
      key("Rennen, springen", "Umschalt, Leertaste");
      key("Reparaturset, zu Fuß neben dem Fahrzeug", "R");
      key("Karte, Ziel für die Artillerie", "K");
      key("Kamera", "C");
      key("Screenshot", "F12");
    }
    panelSel = items.empty() ? 0 : std::clamp(panelSel, 0, int(items.size()) - 1);

    auto choosePanel = [&] {
      if (panelKind == PANEL_ONLINE && panelSel < 3) panelSel = 3;  // from a field, ENTER goes on to the button
      else if (panelKind != PANEL_KEYS && panelSel < int(items.size())) items[size_t(panelSel)].change(1);
    };
    auto closePanel = [&] {
      saveSettings();
      if (panelKind == PANEL_LOADOUT) {
        // Without a choice there is nothing else to do; the menu is one key further.
        panelKind = PANEL_MAIN;
        panelSel = 0;
        return;
      }
      if (panelKind != PANEL_MAIN && fromMain) {
        panelSel = panelKind == PANEL_ONLINE ? 0 : (netState == Game::NET_IN ? 2 : (panelKind == PANEL_GAME ? 1 : 2));
        panelKind = PANEL_MAIN;
        fromMain = false;
        return;
      }
      // With nothing to return to, the menu stays.
      if (netState == Game::NET_IN) panelKind = PANEL_NONE;
    };
    auto panelMove = [&](int d) {
      if (!items.empty()) panelSel = (panelSel + d + int(items.size())) % int(items.size());
    };
    auto panelAdjust = [&](int d) {
      if ((panelKind == PANEL_GAME || panelKind == PANEL_LOADOUT) && panelSel < int(items.size())) items[size_t(panelSel)].change(d);
    };

    // Typing goes to the address fields, and nowhere else.
    const bool wantText = panelKind == PANEL_ONLINE && panelSel < 3;
    if (wantText != textEntry) {
      textEntry = wantText;
      if (wantText) SDL_StartTextInput(window);
      else SDL_StopTextInput(window);
    }

    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (wantText && e.type == SDL_EVENT_TEXT_INPUT) {
        std::lock_guard<std::mutex> lock(G.inputMutex);
        for (const char* t = e.text.text; *t; t++) {
          const unsigned char ch = static_cast<unsigned char>(*t);
          if (panelSel == 0 && gUserName.size() + 1 < size_t(net::kNameBytes) && (ch >= 128 || std::isgraph(ch) || ch == ' ')) gUserName += char(ch);
          if (panelSel == 1 && gServerHost.size() < 63 && (std::isalnum(ch) || ch == '.' || ch == '-')) gServerHost += char(ch);
          if (panelSel == 2 && std::isdigit(ch) && gServerPort < 6553) gServerPort = gServerPort * 10 + (ch - '0');
        }
        continue;
      }
      if (wantText && e.type == SDL_EVENT_KEY_DOWN) {
        const SDL_Keycode k = e.key.key;
        if (k == SDLK_BACKSPACE) {
          std::lock_guard<std::mutex> lock(G.inputMutex);
          if (panelSel == 0 && !gUserName.empty()) {
            // A whole letter at a time, however many bytes it is.
            while (gUserName.size() > 1 && (static_cast<unsigned char>(gUserName.back()) & 0xC0) == 0x80) gUserName.pop_back();
            gUserName.pop_back();
          }
          if (panelSel == 1 && !gServerHost.empty()) gServerHost.pop_back();
          if (panelSel == 2) gServerPort /= 10;
          continue;
        }
        // These are letters here, not menu keys.
        if (k == SDLK_W || k == SDLK_A || k == SDLK_S || k == SDLK_D || k == SDLK_SPACE) continue;
      }
      // The map takes the mouse while it is open.
      if (mapOpen && panelKind == 0 && e.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
        int pw, ph;
        SDL_GetWindowSizeInPixels(window, &pw, &ph);
        float mx0, my0, ms;
        mapLayout(pw, ph, mx0, my0, ms);
        const double wx = (double(e.button.x - mx0) / ms * 2.0 - 1.0) * kMapHalf, wy = (1.0 - double(e.button.y - my0) / ms * 2.0) * kMapHalf;
        if (e.button.button == SDL_BUTTON_LEFT && std::fabs(wx) < kMapHalf && std::fabs(wy) < kMapHalf) {
          hasMark = true;
          markPoint = V3{wx, wy, G.world.height(wx, wy)};
        }
        if (e.button.button == SDL_BUTTON_RIGHT) hasMark = false;
        continue;
      }
      // A panel on screen takes the keys and buttons it needs first.
      if (panelKind != 0) {
        bool used = true;
        if (e.type == SDL_EVENT_KEY_DOWN) {
          switch (e.key.key) {
            case SDLK_UP: case SDLK_W: panelMove(-1); break;
            case SDLK_DOWN: case SDLK_S: panelMove(1); break;
            case SDLK_LEFT: case SDLK_A: panelAdjust(-1); break;
            case SDLK_RIGHT: case SDLK_D: panelAdjust(1); break;
            case SDLK_RETURN: case SDLK_SPACE: case SDLK_KP_ENTER: if (!e.key.repeat) choosePanel(); break;
            case SDLK_ESCAPE: closePanel(); break;
            default: used = false; break;
          }
        } else if (e.type == SDL_EVENT_GAMEPAD_BUTTON_DOWN) {
          switch (e.gbutton.button) {
            case SDL_GAMEPAD_BUTTON_DPAD_UP: panelMove(-1); break;
            case SDL_GAMEPAD_BUTTON_DPAD_DOWN: panelMove(1); break;
            case SDL_GAMEPAD_BUTTON_DPAD_LEFT: panelAdjust(-1); break;
            case SDL_GAMEPAD_BUTTON_DPAD_RIGHT: panelAdjust(1); break;
            case SDL_GAMEPAD_BUTTON_SOUTH: choosePanel(); break;
            case SDL_GAMEPAD_BUTTON_EAST: case SDL_GAMEPAD_BUTTON_GUIDE: closePanel(); break;
            default: used = false; break;
          }
        } else if (e.type == SDL_EVENT_MOUSE_MOTION || e.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
          int pw, ph;
          SDL_GetWindowSizeInPixels(window, &pw, &ph);
          float px, py, pwid, rowH;
          panelLayout(pw, ph, items.size(), px, py, pwid, rowH);
          const float mxp = e.type == SDL_EVENT_MOUSE_MOTION ? e.motion.x : e.button.x;
          const float myp = e.type == SDL_EVENT_MOUSE_MOTION ? e.motion.y : e.button.y;
          const int row = int(std::floor((myp - py) / rowH));
          const bool inside = mxp >= px && mxp <= px + pwid && row >= 0 && row < int(items.size());
          if (inside) panelSel = row;
          if (inside && e.type == SDL_EVENT_MOUSE_BUTTON_DOWN && e.button.button == SDL_BUTTON_LEFT) {
            // In a value row: left half of the value down, right half up.
            if ((panelKind == PANEL_GAME || panelKind == PANEL_LOADOUT) && row + 1 < int(items.size())) panelAdjust(mxp < px + pwid * 0.65f ? -1 : 1);
            else choosePanel();
          }
        } else {
          used = false;
        }
        if (used) continue;
      }
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
            case SDL_GAMEPAD_BUTTON_NORTH: rig.mode = CamMode((rig.mode + 1) % CAM_COUNT); break;
            default: break;
          }
          break;
        case SDL_EVENT_KEY_DOWN:
          if (e.key.repeat) break;
          switch (e.key.key) {
            case SDLK_ESCAPE:
              if (mapOpen) mapOpen = false;
              else {
                panelKind = PANEL_MAIN;
                panelSel = 0;
                fromMain = false;
              }
              break;
            case SDLK_R: edges.repair = true; break;
            case SDLK_G: edges.launch = true; break;
            case SDLK_K: mapOpen = !mapOpen; break;
            case SDLK_C: rig.mode = CamMode((rig.mode + 1) % CAM_COUNT); rig.pitch = 0.22; break;
            case SDLK_F: edges.fire = true; break;
            case SDLK_E: edges.use = true; break;
            case SDLK_F12: wantShot = true; break;
            default: break;
          }
          break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
          if (e.button.button == SDL_BUTTON_LEFT) edges.fire = true;
          break;
        case SDL_EVENT_MOUSE_MOTION: {
          const double sens = (rig.mode == CAM_HOOD ? 0.0009 : 0.0028) * gSet.mouse;
          if (!mapOpen) {
            rig.yaw -= e.motion.xrel * sens;
            rig.pitch = clampd(rig.pitch + e.motion.yrel * sens, -0.30, 1.2);
          }
          break;
        }
        case SDL_EVENT_MOUSE_WHEEL:
          rig.dist = clampd(rig.dist * std::pow(0.9, double(e.wheel.y)), 2.5, 40.0);
          break;
        default:
          break;
      }
    }

    // Continuous inputs.
    const bool* keys = SDL_GetKeyboardState(nullptr);
    double keyDir = 0;
    if (keys[SDL_SCANCODE_A] || keys[SDL_SCANCODE_LEFT]) keyDir += 1;
    if (keys[SDL_SCANCODE_D] || keys[SDL_SCANCODE_RIGHT]) keyDir -= 1;
    double gas = (keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_UP]) ? 1.0 : 0.0;
    double brake = (keys[SDL_SCANCODE_S] || keys[SDL_SCANCODE_DOWN]) ? 1.0 : 0.0;
    const bool sprint = keys[SDL_SCANCODE_LSHIFT];
    double steer = keyDir, hb = keys[SDL_SCANCODE_SPACE] ? 1.0 : 0.0;

    hud.gamepad = false;
    if (gamepad) {
      const double sx = SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_LEFTX) / 32767.0;
      const double rt = SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER) / 32767.0;
      const double lt = SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_LEFT_TRIGGER) / 32767.0;
      const double rx = applyDeadzone(SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_RIGHTX) / 32767.0, 0.15);
      const double ry = applyDeadzone(SDL_GetGamepadAxis(gamepad, SDL_GAMEPAD_AXIS_RIGHTY) / 32767.0, 0.15);
      const double stick = applyDeadzone(sx, 0.08);
      if (std::fabs(stick) > 0 || rt > 0.02 || lt > 0.02) hud.gamepad = true;
      if (std::fabs(stick) > 0) steer = std::copysign(std::pow(std::fabs(stick), 1.35), -stick);
      gas = std::max(gas, rt);
      brake = std::max(brake, lt);
      if (SDL_GetGamepadButton(gamepad, SDL_GAMEPAD_BUTTON_SOUTH)) hb = 1.0;
      rig.yaw -= rx * dt * 2.4;
      rig.pitch = clampd(rig.pitch + ry * dt * 1.4, -0.25, 1.45);
    }

    // Holding the brake at a standstill selects reverse, holding the throttle selects drive. In reverse the
    // pedals swap, so that S always means "go backwards".
    Controls sendC = edges;
    if (inVehicle) {
      const bool stopped = std::fabs(view.speedKmh) < 1.5;
      if (stopped && brake > 0.1 && gas < 0.05 && view.gear >= 0) sendC.requestGear = -1;
      if (stopped && gas > 0.1 && brake < 0.05 && view.gear <= 0) sendC.requestGear = 1;
      if (view.gear < 0) std::swap(gas, brake);
    }

    // While a panel is open the pedals rest: its keys are the same ones.
    if (panelKind != 0) gas = brake = steer = hb = 0;
    {
      const bool wantRelative = panelKind == 0 && !mapOpen && scriptMode == 0;
      if (wantRelative != relativeMouse) {
        SDL_SetWindowRelativeMouseMode(window, wantRelative);
        relativeMouse = wantRelative;
      }
    }
    // With no match to look at, the camera goes slowly round the yard.
    if (!inMatch && panelKind != 0) rig.yaw += dt * 0.12;

    // Scripted runs, for trying out the real thing without anyone at the controls.
    if (scriptMode == 41 && netState == Game::NET_IN) {
      if (phase == Duel::CHOOSING && tNow > 1.0 && !pickReady) {
        pickItems[IT_REPAIR] = 1;
        pickItems[IT_AR] = 2;
        pickItems[IT_SMALL_AMMO] = 6;
        pickReady = true;
        panelKind = PANEL_NONE;
      }
      if (inMatch) {
        panelKind = PANEL_NONE;
        if (onFoot) {
          // To the tank with the chosen turret, and in.
          const V3 to = G.world.base(slot).slot(gPickTurret) - view.foot.p;
          rig.yaw = std::atan2(-to.y, -to.x);
          rig.pitch = 0.2;
          gas = 1.0;
          static double lastUse = 0;
          if (len(V3{to.x, to.y, 0}) < 4.5 && tNow - lastUse > 1.0) {
            sendC.use = true;
            lastUse = tNow;
            printf("Einsteigen versucht bei t=%.1f\n", tNow);
          }
        } else {
          static double boarded = tNow;
          gas = tNow - boarded < 12.0 ? 0.5 : 0.0;
          if (std::fmod(tNow, 3.0) < dt) sendC.fire = true;
          static bool leftOnce = false;
          if (tNow - boarded > 16.0 && !leftOnce) {
            leftOnce = true;
            sendC.use = true;
            printf("Aussteigen bei t=%.1f\n", tNow);
          }
        }
      }
    }

    {
      std::lock_guard<std::mutex> lock(G.inputMutex);
      Controls& c = G.input.c;
      c.steer = steer;
      c.throttle = gas;
      c.brake = brake;
      c.handbrake = hb;
      c.fire |= sendC.fire;
      c.launch |= sendC.launch;
      c.repair |= sendC.repair;
      c.use |= sendC.use;
      G.input.yaw = rig.yaw;
      G.input.pitch = rig.pitch;
      G.input.sprint = sprint;
      if (sendC.requestGear != 99) c.requestGear = sendC.requestGear;
      G.input.ready = pickReady;
      for (int i = 0; i < IT_COUNT; i++) G.input.items[i] = pickItems[i];
      c.indirect = false;
      if (view.missileLive) {
        // The camera flies with the missile, and where it looks is where it goes.
        const V3 back{std::cos(rig.yaw) * std::cos(rig.pitch), std::sin(rig.yaw) * std::cos(rig.pitch), std::sin(rig.pitch)};
        c.aimPoint = view.missileP - back * 2000.0;
      } else if (hasMark) {
        c.aimPoint = markPoint;
        c.indirect = true;
      } else {
        // Whatever is under the crosshair is what the gun lays on.
        const V3 f = norm(cam.target - cam.eye);
        V3 p = cam.eye + f * 14.0;
        for (double d = 14.0; d < 1800.0; d += 3.0) {
          Contact con;
          if (G.world.contact(p, con)) break;
          p += f * 3.0;
        }
        c.aimPoint = p;
      }
    }

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
      view.scorch = G.snap.scorch;
      view.parts = G.snap.parts;
      view.craters = G.snap.craters;
      view.conventional = G.snap.conventional;
      view.boost = G.snap.boost;
      view.oilTemp = G.snap.oilTemp;
      view.oilLevel = G.snap.oilLevel;
      view.tank = G.snap.tank;
      for (int i = 0; i < 2; i++) {
        view.trackPath[i] = G.snap.trackPath[i];
        view.trackPhase[i] = G.snap.trackPhase[i];
        view.trackSpeed[i] = G.snap.trackSpeed[i];
      }
      view.rpmMax = G.snap.rpmMax;
      view.rpmRed = G.snap.rpmRed;
      view.turretYaw = G.snap.turretYaw;
      view.gunPitch = G.snap.gunPitch;
      view.reload = G.snap.reload;
      view.reloadTime = G.snap.reloadTime;
      view.muzzle = G.snap.muzzle;
      view.gunDir = G.snap.gunDir;
      view.sight = G.snap.sight;
      view.absOn = G.snap.absOn;
      view.espOn = G.snap.espOn;
      view.tcsOn = G.snap.tcsOn;
      view.absActive = G.snap.absActive;
      view.espActive = G.snap.espActive;
      view.tcsActive = G.snap.tcsActive;
      view.engineLoad = G.snap.engineLoad;
      view.inputRpm = G.snap.inputRpm;
      view.starter = G.snap.starter;
      view.shifting = G.snap.shifting;
      view.realtime = G.snap.realtime;
      view.awake = G.snap.awake;
      view.objectCount = G.snap.objectCount;
      view.splitGeneration = G.snap.splitGeneration;
      view.carResets = G.snap.carResets;
      view.tankTracks = G.snap.tankTracks;
      view.foot = G.snap.foot;
      view.missileLive = G.snap.missileLive;
      view.missileP = G.snap.missileP;
      view.missileV = G.snap.missileV;
      view.missiles = G.snap.missiles;
      view.repairKits = G.snap.repairKits;
      view.trackLeft[0] = G.snap.trackLeft[0];
      view.trackLeft[1] = G.snap.trackLeft[1];
      for (int i = 0; i < 16; i++) view.parkedScorch[i] = G.snap.parkedScorch[i];
      view.tankArmour = G.snap.tankArmour;
      view.tankAmmo = G.snap.tankAmmo;
      view.enemyCog = G.snap.enemyCog;
      view.tankDead = G.snap.tankDead;
      view.nodeObj = G.snap.nodeObj;
      view.paused = G.snap.paused;
      view.timeScale = G.snap.timeScale;
    }

    hud.panel.open = panelKind != 0;
    if (hud.panel.open) {
      static const char* titles[6] = {"", "Spiel-Einstellungen", "BeanNG", "Steuerung", "Online spielen", "Ausrüstung"};
      hud.panel.title = titles[panelKind];
      if (panelKind == PANEL_LOADOUT) {
        char b[64];
        snprintf(b, sizeof b, "Ausrüstung  ·  noch %.0f s", std::max(0.0, Duel::kChooseTime - phaseClock));
        hud.panel.title = b;
      }
      hud.panel.foot = panelKind == PANEL_ONLINE ? "Tippen  ·  Pfeile wählen  ·  ENTER weiter  ·  ESC zurück" : panelKind == PANEL_MAIN ? "Pfeile wählen  ·  ENTER bestätigen  ·  ESC zurück" : (panelKind == PANEL_KEYS ? "ESC zurück" : "Pfeile wählen und ändern  ·  ESC zurück");
      hud.panel.sel = panelSel;
      hud.panel.rows.clear();
      hud.panel.values.clear();
      hud.panel.fill.clear();
      for (const Item& it : items) {
        hud.panel.rows.push_back(it.label);
        hud.panel.values.push_back(it.value);
        hud.panel.fill.push_back(it.fill);
      }
    }
    hud.vehicleHud = inVehicle;
    hud.crosshair = inMatch && !mapOpen && !view.missileLive && !view.foot.inCar;
    hud.map.open = mapOpen && panelKind == 0;
    hud.map.hasMark = hasMark;
    hud.map.mark = markPoint;
    hud.map.me = view.carCog;
    hud.map.meYaw = std::atan2(view.carFwd.y, view.carFwd.x);
    hud.cameraName = kCamNames[rig.mode];
    feel.update(view, dt);
    rig.update(view, dt, cam, G.world, feel, gSet.shake);
    cam.fovY += gSet.fov * PI / 180.0;
    if (view.missileLive) {
      const V3 back{std::cos(rig.yaw) * std::cos(rig.pitch), std::sin(rig.yaw) * std::cos(rig.pitch), std::sin(rig.pitch)};
      cam.eye = view.missileP + back * 9.0 + V3{0, 0, 2.0};
      cam.target = view.missileP - back * 30.0;
      cam.up = {0, 0, 1};
    }
    if (onFoot && !mapOpen) {
      // Over the shoulder.
      const V3 back{std::cos(rig.yaw) * std::cos(rig.pitch), std::sin(rig.yaw) * std::cos(rig.pitch), std::sin(rig.pitch)};
      const V3 side{std::sin(rig.yaw), -std::cos(rig.yaw), 0};
      const V3 head = view.foot.p + V3{0, 0, 1.55} + side * 0.45;
      double reach = 3.2;
      for (double d = 0.3; d < 3.2; d += 0.2) {
        Contact con;
        if (G.world.contact(head + back * d, con)) {
          reach = std::max(d - 0.25, 0.3);
          break;
        }
      }
      cam.eye = head + back * reach;
      cam.target = head - back * 30.0;
      cam.up = {0, 0, 1};
      cam.inside = false;
      cam.fovY = 62.0 * PI / 180.0 + gSet.fov * PI / 180.0;
    }
    hud.motionBlur = gSet.motionBlur;
    hud.damageView = gSet.damageView && inTank;
    hud.stats = gSet.stats;
    hud.farShadows = gSet.farShadows;
    if (scriptMode == 43) {
      // A tour of the map, without a server: the yard, the command centre, a bridge, a barrier, and from above.
      const World::Base& b = G.world.base(0);
      const V3 bridge = G.world.bridges().empty() ? V3{} : G.world.bridges()[1 % G.world.bridges().size()];
      const V3 eyes[7][2] = {
          {b.centre + V3{55, -45, 16}, b.centre + V3{5, 0, 2}},
          {b.centre + V3{-26 + 30, -18, 8}, b.centre + V3{-26, 0, 4}},
          {b.slot(1) + V3{14, 12, 4}, b.slot(2) + V3{0, 0, 1.5}},
          {bridge + V3{-70, 45, 22}, bridge},
          {V3{380, -30, 40} + V3{-90, -60, 40}, V3{380, 0, 20}},
          {V3{-600, -200, 420}, V3{0, 0, 10}},
          {V3{0, -950, 620}, V3{0, 0, 10}},
      };
      const int i = std::min(shotCount, 6);
      cam.eye = eyes[i][0];
      cam.target = eyes[i][1];
      cam.up = {0, 0, 1};
      cam.fovY = 60.0 * PI / 180.0;
      panelKind = shotCount == 7 ? int(PANEL_NONE) : int(PANEL_NONE);
      hud.panel.open = false;
    }
    if (scriptMode == 44) {
      panelKind = PANEL_NONE;
      hud.panel.open = false;
      mapOpen = true;
      hud.map.open = true;
      hasMark = true;
      markPoint = V3{500, 100, 0};
      hud.map.hasMark = true;
      hud.map.mark = markPoint;
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
    const auto drawStart = std::chrono::steady_clock::now();
    if (w > 0 && h > 0) renderer.draw(view, cam, hud, w, h, dt);
    if (getenv("BEANNG_PROFILE")) {
      static int n = 0;
      if (++n % 30 == 0) fprintf(stderr, "draw %.1f ms, before it %.1f ms\n", std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - drawStart).count(), std::chrono::duration<double, std::milli>(drawStart - now).count());
    }

    // Scripted screenshot series.
    if (!shotDir.empty()) {
      static double nextShot = 2.0;
      if (tNow >= nextShot) {
        char path[512];
        snprintf(path, sizeof path, "%s/shot_%02d.png", shotDir.c_str(), shotCount++);
        renderer.savePng(path, w, h);
        printf("fps %.0f t=%.1f v=%.0f km/h pos %.0f %.0f %.1f Phase %d Fahrzeug %d Munition %d Leben %.0f\n", hud.fps, tNow, view.speedKmh, view.carCog.x, view.carCog.y, view.carCog.z, phase, myVehicle, view.tankAmmo, view.foot.hp);
        fflush(stdout);
        nextShot += 1.0;
        const int enough = scriptMode == 43 ? 7 : (scriptMode == 44 ? 2 : 60);
        if (shotCount >= enough) quit = true;
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
  G.running = false;
  network.join();
  if (gamepad) SDL_CloseGamepad(gamepad);
  SDL_SetWindowRelativeMouseMode(window, false);
  SDL_GL_DestroyContext(ctx);
  delete game;
}

int main(int argc, char** argv) {
  preferDiscreteGpu();
  loadSettings();
  loadServerAddress();
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--shots") && i + 1 < argc) shotDir = argv[++i];
    if (!strcmp(argv[i], "--script") && i + 1 < argc) scriptMode = atoi(argv[++i]);
    if (!strcmp(argv[i], "--wav") && i + 1 < argc) wavPath = argv[++i];
    if (!strcmp(argv[i], "--turret") && i + 1 < argc) gPickTurret = std::clamp(atoi(argv[++i]), 0, int(TK_COUNT) - 1);
    if (!strcmp(argv[i], "--connect") && i + 1 < argc) {
      // host:port, or just the host.
      std::string target = argv[++i];
      const size_t colon = target.rfind(':');
      if (colon != std::string::npos) {
        gServerPort = atoi(target.c_str() + colon + 1);
        target.resize(colon);
      }
      gServerHost = target;
      gConnectNow = true;
    }
    if (!strcmp(argv[i], "--help")) {
      printf("beanng [--connect HOST:PORT]\n");
      return 0;
    }
  }

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

  window = SDL_CreateWindow("BeanNG", 1600, 900, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
  if (!window) {
    fprintf(stderr, "Fenster: %s\n", SDL_GetError());
    return 1;
  }
  // Sound. It stays silent for automated runs, which must not make noise on the machine they run on,
  // and without a sound card the game simply runs without it.
  sound = new SoundDriver();
  SDL_AudioStream* audioStream = nullptr;
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

  if (gSet.fullscreen && scriptMode == 0) SDL_SetWindowFullscreen(window, true);
  runGame();
  if (scriptMode == 0) saveSettings();
  if (!wavPath.empty()) writeWav(wavPath, wavSamples);
  if (audioStream) SDL_DestroyAudioStream(audioStream);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
