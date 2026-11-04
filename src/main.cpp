#include <unistd.h>
// BeanNG: game loop, input, cameras and the physics thread.
#include <SDL3/SDL.h>
#include <epoxy/gl.h>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <condition_variable>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include "audio.h"
#include "duel.h"
#include "duelnet.h"
#include "shape.h"
#include "props.h"
#include "render.h"
#include "tank.h"
#include "vehicle.h"

// The game's own settings: they outlast a change of vehicle and are kept on disk between runs.
struct GameSettings {
  bool motionBlur = true, shake = true, damageView = true, stats = true, muted = false;
  bool fullscreen = false, vsync = true, farShadows = true;
  double volume = 0.8, fov = 0, mouse = 1;
  double padSens = 1;
  double grass = 1;
  bool rumble = true;
};
static GameSettings gSet;
// Whether the last thing touched was the controller: the hints on screen name its buttons then.
static std::atomic<bool> gPadHints{false};
// A controller made up for the test scripts: everything behind it is the real input path.
static SDL_Joystick* gTestPad = nullptr;
// --log: once a second what the line and the picture are doing, for finding out why a game went bad.
static FILE* gPlayLog = nullptr;
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
    else if (k == "pad_sensitivity") gSet.padSens = clampd(v, 0.3, 2.5);
    else if (k == "rumble") gSet.rumble = v != 0;
    else if (k == "grass") gSet.grass = clampd(v, 0, 1);
  }
  fclose(f);
}

static void saveSettings() {
  FILE* f = fopen(kSettingsFile, "w");
  if (!f) return;
  fprintf(f, "motion_blur %d\ncamera_shake %d\ndamage_view %d\nstats %d\nmuted %d\nfullscreen %d\nvsync %d\nfar_shadows %d\nvolume %.2f\nfov %.0f\nmouse %.2f\n",
          gSet.motionBlur, gSet.shake, gSet.damageView, gSet.stats, gSet.muted, gSet.fullscreen, gSet.vsync, gSet.farShadows, gSet.volume, gSet.fov, gSet.mouse);
  fprintf(f, "pad_sensitivity %g\nrumble %d\ngrass %g\n", gSet.padSens, gSet.rumble, gSet.grass);
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
      const double reach = dist + (mode == CAM_ORBIT ? 8.0 : 5.5);
      V3 eye = pivot + back * reach;
      // Looking steeply up the camera would end up under the hull: it stays behind it, low down.
      const double steep = smooth01(clampd((-pitch - 0.30) / 0.30, 0, 1));
      if (steep > 0) {
        const double flat = std::max(reach * std::cos(pitch), 11.0);
        eye.x = pivot.x + std::cos(yaw) * flat;
        eye.y = pivot.y + std::sin(yaw) * flat;
      }
      const double gh = world.height(eye.x, eye.y) + 0.5;
      if (eye.z < gh) eye.z = gh;
      out.eye = eye;
      // And looks exactly where it was told to: from down there, past a point over the hull it would not.
      out.target = lerpv(pivot - back * 8.0, eye - back * 20.0, steep);
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
    // Nothing in the engine bay or under the tracks unless one is sitting in it.
    f.speed = s.running ? float(len(s.carVel)) : 0.f;
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
    if (s.conventional || s.foot.inCar) {
      f.cylinders = 8;
      f.exhaustPitch = s.rpmMax < 5000 ? 0.72f : 1.15f;  // a big slow diesel, or the One:1
      f.turbo = float(s.boost) * (s.rpmMax < 5000 ? 0.45f : 1.0f);
    }
    if (s.tank && !s.foot.inCar) {
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
        // By how big the gun is and how far away it went off: a pistol is no tank, and the big rifle is close.
        float pan, gain;
        place(e.p, pan, gain);
        const float size = std::clamp(e.strength, 0.05f, 1.f);
        synth.trigger(Sfx::Gun, size, pan, std::max(gain * 1.6f, 0.03f) * (0.25f + size * 1.1f));
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
  bool fireHeld = false;
  bool aiming = false;
  bool scoped = false;
  int weapon = WP_PISTOL;
  double yaw = 0, pitch = 0;
  std::string chatOut;  // the next line to say; cleared here once it is on its way
  u8 items[IT_COUNT] = {};
};

// The connection to the server and the copy of the world it describes. Nothing is simulated here: both tanks and
// everything else go where the server says, a moment late so that there is always a next position.
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
  // What a line of sight can end on, for the window's thread.
  std::vector<VehicleFrame> sightFrames;
  std::vector<Duel::AimFoe> sightFoes;
  std::atomic<bool> running{true};
  std::atomic<bool> resetVisuals{false};
  // Every picture asks for its own moment: where things are is worked out just then, not some milliseconds before.
  std::mutex frameMutex;
  std::condition_variable frameWanted, frameReady;
  u64 frameAsked = 0, frameAnswered = 0;
  void askForFrame() {
    std::unique_lock<std::mutex> lock(frameMutex);
    const u64 mine = ++frameAsked;
    frameWanted.notify_one();
    frameReady.wait_for(lock, std::chrono::milliseconds(4), [&] { return frameAnswered >= mine; });
  }

  // Asked for from the window's thread.
  enum Command { CMD_NONE, CMD_CONNECT, CMD_LEAVE, CMD_REPLAY };
  std::atomic<int> command{CMD_NONE};

  // What the window's thread reads, under snapMutex.
  enum NetState { NET_OFF, NET_CONNECTING, NET_IN, NET_REPLAY };
  // Whose match to look at in a replay, and how fast. Asked for from the window's thread.
  std::atomic<int> replayWatch{0};
  std::atomic<int> replaySpeed{100};  // percent
  bool hasReplay = false;
  double replayLeft = 0;
  int netState = NET_OFF;
  std::string netError;
  std::string names[Duel::kMaxPlayers];
  int phase = Duel::WAITING;
  double phaseClock = 0;
  int slot = 0;
  int myVehicle = -1;
  bool playing = false;  // connected, and the match on
  Soldier me;            // on foot: where this machine thinks it is, ahead of what the server has answered

  // Where the other player's vehicle is, if they sit in one. Read by the window's thread, roughly, for the ears.
  bool enemyVehicleFrame(int me, V3 s_ear, V3& at, double& speed) {
    std::lock_guard<std::mutex> lock(snapMutex);
    // The nearest of them that is moving.
    bool any = false;
    double bestD = 1e30;
    for (int e = 0; e < Duel::kMaxPlayers; e++) {
      const int v = duel.players[e].vehicle;
      if (v < 0 || !duel.players[e].present || Duel::teamOf(e) == Duel::teamOf(me) || std::fabs(duel.frames[v].speed) < 1.5) continue;
      const double d = len2(duel.frames[v].cog - s_ear);
      if (d < bestD) {
        bestD = d;
        at = duel.frames[v].cog;
        speed = std::fabs(duel.frames[v].speed);
        any = true;
      }
    }
    return any;
  }

  void fillSnapshot(const char* message) {
    std::lock_guard<std::mutex> lock(snapMutex);
    Snapshot& s = snap;
    s.pos = sim.pos;
    s.tank = true;
    // The soldiers come between the vehicles and the wrecks in the mesh, and have no tracks.
    s.tankTracks.resize(Duel::kTanks + Duel::kWrecks);
    for (int i = 0; i < Duel::kTanks + Duel::kWrecks; i++) {
      const Tank& t = i < Duel::kTanks ? duel.tanks[i] : duel.wrecks[i - Duel::kTanks];
      Snapshot::TankTrack& tt = s.tankTracks[size_t(i)];
      for (int k = 0; k < 2; k++) {
        t.trackPath(sim, k, tt.path[k]);
        tt.left[k] = t.trackWidthDir(k);
        tt.phase[k] = t.trackPhase[k];
        tt.speed[k] = t.trackOmega[k] * Tank::kSprocketR;
      }
      s.parkedScorch[i < Duel::kTanks ? i : i - Duel::kTanks + Duel::kVehicles + Duel::kMaxPlayers] = t.destroyed ? float(clampd(t.burnTime / 3.0, 0.15, 1.0)) : float(t.damage * 0.3);
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
    // Tracks and tyres, for the marks they leave.
    s.groundBoxes.clear();
    for (int i = 0; i < Duel::kVehicles; i++) {
      const VehicleFrame& f = duel.frames[i];
      // Rolling, and on the ground: what flies over it leaves it standing.
      if (std::fabs(f.speed) < 0.3 || f.cog.z - world.height(f.cog.x, f.cog.y) > (i < Duel::kTanks ? 3.0 : 1.8)) continue;
      s.groundBoxes.push_back({f.cog, f.fwd, f.left, float(f.halfLength), float(f.halfWidth)});
    }
    s.groundMarks.clear();
    for (int i = 0; i < Duel::kTanks; i++) {
      const VehicleFrame& f = duel.frames[i];
      const Tank& t = duel.tanks[i];
      if (t.destroyed) continue;
      for (double side : {1.0, -1.0}) {
        if (t.trackOff[side > 0 ? 0 : 1]) continue;
        s.groundMarks.push_back({f.cog + f.left * (side * 1.35), f.left, 0.26f});
      }
    }
    for (int k = 0; k < Duel::kCars; k++) {
      const Vehicle& car = duel.cars[size_t(k)];
      if (car.exploded) continue;
      for (const Wheel& w : car.wheels) s.groundMarks.push_back({(sim.pos[w.axIn] + sim.pos[w.axOut]) * 0.5, sim.pos[w.axOut] - sim.pos[w.axIn], 0.1f});
    }
    s.foot.onFoot = playing && myVehicle < 0;
    s.selfInstance = myVehicle >= 0 ? myVehicle : (playing ? Duel::kVehicles + slot : -1);
    s.selfOnFoot = myVehicle < 0 && playing;
    s.selfTrack = myVehicle >= 0 && myVehicle < Duel::kTanks ? myVehicle : -1;
    s.foot.inCar = myVehicle >= Duel::kTanks;
    s.foot.hp = duel.players[slot].soldier.hp;
    s.foot.p = me.p;
    s.foot.yaw = me.yaw;
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
    part(nullptr, 34, 188, 32, 12, tank.mod[TM_RADIATOR]);
    part(nullptr, 44, 200, 12, 8, tank.mod[TM_FUEL]);
    part(nullptr, 36, 96, 10, 10, tank.mod[TM_SIGHT]);
    part(nullptr, 54, 96, 10, 10, tank.mod[TM_RANGE]);
    part(nullptr, 25, 56, 12, 8, tank.mod[TM_PERISCOPE]);
    part(nullptr, 44, 76, 12, 8, tank.mod[TM_ELEVATION]);
    part(nullptr, 44, 132, 12, 8, tank.mod[TM_ANTENNA]);
    // What has gone from this tank, for the instruments and the view.
    s.foot.noSight = myVehicle >= 0 && myVehicle < Duel::kTanks && tank.mod[TM_SIGHT] <= 0;
    s.foot.noRange = myVehicle >= 0 && myVehicle < Duel::kTanks && tank.mod[TM_RANGE] <= 0;
    s.foot.noPeriscope = myVehicle >= 0 && myVehicle < Duel::kTanks && tank.mod[TM_PERISCOPE] <= 0;
    s.foot.noRadio = myVehicle >= 0 && myVehicle < Duel::kTanks && tank.mod[TM_ANTENNA] <= 0;
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
    // The instruments of what one is in: a tank standing about has its brakes on, and that is nothing to do with a car.
    const Vehicle* car = s.foot.inCar ? &duel.cars[size_t(myVehicle - Duel::kTanks)] : nullptr;
    const bool inTankNow = myVehicle >= 0 && !car;
    s.throttle = car ? car->throttleAct : (inTankNow ? tank.throttleAct : 0.0);
    s.brake = car ? car->brakeAct : (inTankNow ? tank.brakeAct : 0.0);
    s.steer = car ? car->steerAct : (inTankNow ? tank.steerAct : 0.0);
    s.latG = (car ? car->latAcc : tank.latAcc) / 9.81;
    s.lonG = (car ? car->lonAcc : tank.lonAcc) / 9.81;
    s.gear = s.foot.inCar ? duel.cars[size_t(myVehicle - Duel::kTanks)].gear : tank.gear;
    s.running = myVehicle >= 0;
    s.fuel = car ? car->fuelLitres : tank.fuel;
    s.coolant = car ? car->dmg.coolantTemp : 90.0;
    s.foot.fuelShare = car ? car->fuelLitres / std::max(duel.carsStart[size_t(myVehicle - Duel::kTanks)].fuelLitres, 1.0) : 1.0;
    s.engineLoad = car ? car->throttleAct : (myVehicle >= 0 ? tank.loadNow : 0.0);
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
    double arrived = 0;  // on this machine's clock
    V3 cog;
    Tank::NetState state;
    std::vector<float> offsets;  // of each node against the middle, in metres
  };

  void netLoop() {
    using Clock = std::chrono::steady_clock;
    // How far back everything else is shown. Far off things come thirty times a second, so one and a half of those at
    // least; and as much on top as the line has lately held anything up. It changes only slowly, as time going
    // a little slower or faster for a moment.
    double viewDelay = 0.10, lineJitter = 0.01;
    constexpr size_t kFramesKept = size_t(net::kSendRate * 0.6);  // enough for the longest way back to be shown
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
    // A car comes in several packets to the tick. Put together again, it is shown a little late just like a tank, so
    // that there is always a next position to move towards.
    struct CarFrame {
      u32 tick = 0;
      double arrived = 0;
      V3 cog, fwd, left, up;
      float speed = 0, rpm = 0, throttle = 0, brake = 0, steer = 0, fuel = 0, coolant = 0, lat = 0, lon = 0;
      float theta[4] = {};
      u8 hurt = 0, flat = 0;
      int gear = 1;
      bool exploded = false;
      std::vector<float> offsets;
    };
    struct CarShown {
      std::deque<CarFrame> frames;
      float fuel = 0, coolant = 0;
    } shown[Duel::kCars];
    // What the server has told of each vehicle's nodes, against the part they belong to.
    net::VehicleShapes shapes;
    std::vector<i16> toldTanks, toldCars;
    Tank::NetState tankSaid[Duel::kTanks];
    // A datagram holds several packets; each is taken as it would be alone.
    std::deque<std::vector<u8>> bundled;
    // Walking answers at once, and is checked against the server's answer afterwards.
    Soldier shownSoldier[Duel::kMaxPlayers];
    // The others on foot, a little late like the vehicles: between two things the server has said, not the last of them.
    // That is also the moment the server looks at when it judges a shot at them.
    struct SoldierFrame {
      u32 tick = 0;
      Soldier s;
      int vehicle = -1;
    };
    std::deque<SoldierFrame> soldierPast[Duel::kMaxPlayers];
    Soldier drawnSoldier[Duel::kMaxPlayers];
    int drawnVehicle[Duel::kMaxPlayers];
    for (int& v : drawnVehicle) v = -1;
    // Every step taken is kept until the server has answered for it. Its answer is then taken as it is and the steps it
    // has not seen yet are walked again from there. What that changes is not shown as a jump, but melts away.
    struct WalkStep {
      u32 seq;
      WalkInput in;
    };
    std::deque<WalkStep> walkSteps;
    Soldier walkTrue;
    V3 walkOffset;
    u8 ackedSteps = 0;
    double walkDebt = 0;
    // The last match, as it arrived: everything needed to see it again, from any side.
    struct Taped {
      double at;
      std::vector<u8> bytes;
    };
    std::vector<Taped> tape, lastTape;
    size_t tapeBytes = 0;
    double tapeOffset = 0, lastTapeOffset = 0;
    size_t replayAt = 0;
    double replayNow = 0;
    bool wasReplay = false;
    // How long a question takes to come back: from when an input went out to when the server says it has used it.
    std::deque<std::pair<u32, double>> sentAt;
    double rtt = 0.05;
    // One's own vehicle, shown ahead of what the server last said: where it will be by the time one's latest input has
    // arrived. It runs on by itself, turned by the steering at once, and is drawn towards that place all the time
    // instead of being put there: the server's word moves it no more than a little with any one packet.
    struct OwnShown {
      int who = -1;
      V3 p;
      double heading = 0, seenTurn = 0;
    } own;
    const auto ownLookAhead = [&] { return clampd(rtt, 0.0, 0.2); };
    // Returns how far the heading on the screen is ahead of the last frame's, and leaves own.p where to draw it.
    const auto ownPredict = [&](int who, V3 lastCog, V3 vel, double lastHeading, double seen, double want, double age, double dtNow) {
      const double ahead = age + ownLookAhead();
      own.seenTurn = who == own.who ? own.seenTurn + (seen - own.seenTurn) * clampd(dtNow * 12.0, 0.0, 1.0) : seen;
      const V3 target = lastCog + vel * ahead;
      const double targetHeading = lastHeading + own.seenTurn * ahead;
      if (who != own.who || len(target - own.p) > 6.0) {
        own.who = who;
        own.p = target;
        own.heading = targetHeading;
      }
      // The longer the way to the server, the more the steering is taken on trust; on a short line the server's own
      // answer is nearly here already.
      const double trust = 0.7 * clampd(ownLookAhead() / 0.1, 0.0, 1.0);
      own.heading += lerpd(own.seenTurn, want, trust) * dtNow;
      const double headingError = std::remainder(targetHeading - own.heading, 2.0 * PI);
      own.heading += headingError * (1.0 - std::exp(-dtNow * 6.0));
      own.p += vel * dtNow;
      own.p += (target - own.p) * (1.0 - std::exp(-dtNow * 10.0));
      return std::remainder(own.heading - lastHeading, 2.0 * PI);
    };
    double timeOffset = 0;
    bool haveOffset = false;
    // The shape of one's own vehicle goes on at its own pace: slightly late, and between two frames, so that pitching,
    // springs and the turret do not move in steps while position and heading go on with every picture.
    const auto shapeBetween = [&](const auto& q, double nowHere, size_t& ia, size_t& ib, double& a) {
      const double at = (nowHere - timeOffset - 1.5 / net::kSendRate - lineJitter) * net::kStepRate;
      ib = 0;
      while (ib + 1 < q.size() && double(q[ib].tick) < at) ib++;
      ia = ib > 0 ? ib - 1 : ib;
      const double span = double(q[ib].tick) - double(q[ia].tick);
      a = span > 0 ? clampd((at - double(q[ia].tick)) / span, 0, 1) : 1.0;
    };
    // Between two frames along a curve that knows how fast the thing was going at both, so that it does not change its
    // speed at every frame. And past the last one for a moment, as it was going, when the next is late or lost.
    const auto curveBetween = [](V3 pa, V3 va, V3 pb, V3 vb, double a, double seconds) {
      const double a2 = a * a, a3 = a2 * a;
      return pa * (2.0 * a3 - 3.0 * a2 + 1.0) + va * ((a3 - 2.0 * a2 + a) * seconds) + pb * (3.0 * a2 - 2.0 * a3) + vb * ((a3 - a2) * seconds);
    };
    constexpr double kCoastAtMost = 0.12;
    // For trying things out on one machine: behaves like a bad line, both ways. Each figure holds for one direction:
    // BEANNG_FAKE_LAG and BEANNG_FAKE_JITTER in milliseconds, BEANNG_FAKE_LOSS in percent.
    const auto fakeSetting = [](const char* name) { return getenv(name) ? atof(getenv(name)) : 0.0; };
    const double fakeLag = fakeSetting("BEANNG_FAKE_LAG") / 1000.0, fakeJitter = fakeSetting("BEANNG_FAKE_JITTER") / 1000.0, fakeLoss = fakeSetting("BEANNG_FAKE_LOSS") / 100.0;
    const bool fakeLine = fakeLag > 0 || fakeJitter > 0 || fakeLoss > 0;
    struct Held {
      double at;
      std::vector<u8> bytes;
      net::Addr from;
    };
    std::deque<Held> held, heldOut;
    std::minstd_rand fakeDice(20261104);
    // Lost, or in the queue for when it comes out. With jitter a packet may overtake another, as on the real thing.
    const auto fakeHold = [&](std::deque<Held>& queue, double at, const void* data, size_t size, const net::Addr& from) {
      const auto chance = [&] { return double(fakeDice() - fakeDice.min()) / double(fakeDice.max() - fakeDice.min()); };
      if (chance() < fakeLoss) return;
      const double when = at + fakeLag + fakeJitter * chance();
      const u8* bytes = static_cast<const u8*>(data);
      auto place = queue.end();
      while (place != queue.begin() && std::prev(place)->at > when) --place;
      queue.insert(place, Held{when, std::vector<u8>(bytes, bytes + size), from});
    };
    // One's own shot goes off at once, flash and bang, as long as the server can be expected to agree. What it says
    // has happened still counts for everything else; its report of the same shot is then not shown a second time.
    int shotsAhead = 0;
    double shotsAheadUntil = 0, ownGunReady = 0;
    // The gun under one's own hand, ahead of the server.
    struct OwnAim {
      int who = -1;
      double yaw = 0, pitch = 0;
    } ownAim;
    const auto aimOwnGun = [&](Tank& t, int who, const Controls& c, double dtNow) {
      if (who != ownAim.who) {
        ownAim.who = who;
        ownAim.yaw = t.turretYaw;
        ownAim.pitch = t.gunPitch;
      }
      t.showOwnAim(sim, c, ownAim.yaw, ownAim.pitch, dtNow);
    };
    // What the line is like, for the display: counted over the last second.
    int lineGot = 0, lineLost = 0;
    u32 lineSeq = 0;
    size_t lineBytes = 0;
    double lineNext = 0, lineLoss = 0, lineKb = 0;
    struct RocketSeen {
      u8 what;
      V3 p, v;
    };
    struct RocketsThen {
      u32 tick = 0;
      std::vector<RocketSeen> list;
    };
    std::deque<RocketsThen> rocketPast;
    // What happens in the world waits for the moment on the screen. Told at once, a tank's muzzle flash would hang in
    // front of its gun and a blast go off before the thing that caused it had got there.
    struct Later {
      u32 tick = 0;
      enum What : u8 { BLAST, EVENT, NODE } what = EVENT;
      u8 kind = 0;
      u32 node = 0;
      V3 p, v;
      float a = 0, b = 0, c = 0;
    };
    std::deque<Later> later;
    // Scenery comes thirty times a second at most, so it slides from one word to the next.
    std::unordered_map<u32, V3> sliding;
    // A line of chat is sent again and again until the server's log says it has it.
    std::string saying;
    u32 chatSeq = 0, chatAcked = 0;
    double nextChat = 0;
    std::vector<std::pair<int, std::string>> chatLog;
    u32 chatLogFirst = 0;
    u8 carrying[IT_COUNT] = {};
    int weaponShown[Duel::kMaxPlayers] = {};
    V3 blowShown[Duel::kMaxPlayers];
    bool ghillieShown[Duel::kMaxPlayers] = {};
    struct DroneSeen {
      bool live = false;
      V3 p, v, shown;
      double battery = 0, heardAt = 0;
    } droneShown[Duel::kMaxPlayers];
    Ragdoll bodies[Duel::kMaxPlayers];
    int seatNow = 0, wantedNow = 2, hereNow = 0;
    double repairNow = 0;
    // A part just lost is said out loud for a few seconds.
    double lastMod[TM_COUNT];
    for (double& m : lastMod) m = 1;
    int lastTankWatched = -1;
    std::string alert;
    double alertUntil = 0;
    u8 carHurt[Duel::kCars] = {}, carFlat[Duel::kCars] = {};
    u8 seenHurt = 0, seenFlat = 0;
    int carWatched = -1;
    u32 ackedInput = 0;
    double serverStats[3] = {0, 1, 0};  // load, speed against real time, objects awake
    V3 logFoeAt, logFoeStep;
    double logFoeSum = 0, logFoeN = 0;
    double logWalkErr = 0, logMetaAt = 0, logMetaGap = 0, logLoopGap = 0, logLoopAt = 0, logNext = 0;
    int logMetas = 0, logPackets = 0;
    size_t logBytes = 0;
    V3 serverP;
    bool freshAnswer = false;
    u32 wins[2] = {};
    float clockNow = 0;
    double lastHeard = 0, nextInput = 0, last = clock();
    u64 clockFrame = 0;
    double clockRaw = last, clockStep = 1.0 / 60.0, shownNow = last, shownLast = last;
    net::InputMsg out;
    std::string error;
    u64 servedFrame = 0;

    const auto keepTape = [&] {
      if (tape.size() > 200) {
        lastTape = std::move(tape);
        lastTapeOffset = tapeOffset;
      }
      tape.clear();
      tapeBytes = 0;
    };
    const auto resetWorld = [&] {
      world.clearCraters();
      sim = simStart;
      for (int i = 0; i < Duel::kTanks; i++) {
        duel.tanks[i] = duel.tanksStart[i];
        frames[i].clear();
        tankSaid[i] = duel.tanksStart[i].netState();
      }
      duel.cars = duel.carsStart;
      for (Ragdoll& body : bodies) body = Ragdoll{};
      for (int i = 0; i < Duel::kWrecks; i++) duel.wrecks[i] = duel.wrecksStart[i];
      duel.placePlayers();
      duel.updateFrames();
      for (CarShown& c : shown) c = CarShown{};
      if (shapes.ready) {
        toldTanks = shapes.restTanks;
        toldCars = shapes.restCars;
      }
      bundled.clear();
      me = duel.players[mySlot].soldier;
      walkTrue = me;
      walkOffset = V3{};
      walkSteps.clear();
      for (auto& q : soldierPast) q.clear();
      later.clear();
      sliding.clear();
      rocketPast.clear();
      blastsDone = 0;
      metaTick = 0;
      vehicleNow = -1;
      resetVisuals = true;
    };
    const auto disconnect = [&](const std::string& why) {
      if (state == NET_IN) keepTape();
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
      // The moment of the picture being made. Pictures come evenly, but the wake-up of this thread does not: at 50 km/h one
      // millisecond more or less between two of them is over a centimetre. So the even beat is kept, close to the real clock.
      if (servedFrame != clockFrame) {
        clockFrame = servedFrame;
        clockStep += (clampd(now - clockRaw, 0.0, 0.1) - clockStep) * 0.02;
        clockRaw = now;
        const double beat = shownNow + clockStep;
        shownNow = std::fabs(now - beat) > 0.008 ? now : beat + (now - beat) * 0.05;
      } else if (now - clockRaw > 0.05) {
        shownNow = now;
      }
      const double dtShown = std::max(shownNow - shownLast, 0.0);
      shownLast = shownNow;
      const int cmd = command.exchange(CMD_NONE);
      if (cmd == CMD_LEAVE) disconnect("");
      if (cmd == CMD_REPLAY && !lastTape.empty()) {
        disconnect("");
        state = NET_REPLAY;
        replayAt = 0;
        replayNow = lastTape.front().at;
        timeOffset = lastTapeOffset;
        haveOffset = true;
      }
      const bool replaying = state == NET_REPLAY;
      if (replaying) {
        // Time in the replay goes at the speed asked for, and whichever player is being watched is "me" for all of it.
        replayNow += dt * double(replaySpeed.load()) / 100.0;
        mySlot = replayWatch.load() % Duel::kMaxPlayers;
        if (replayAt >= lastTape.size()) {
          state = NET_OFF;
          match = 0;
          phaseNow = Duel::WAITING;
          resetWorld();
        }
      }
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
        input.chatOut.clear();
        input.c.requestGear = 99;
        input.c.fire = input.c.launch = input.c.repair = input.c.use = input.c.setup = input.c.placeFoot = input.c.placeTank = input.c.strike = input.c.query = input.c.smoke = input.c.net = input.c.drone = input.c.medkit = false;
      }
      if (!in.chatOut.empty() && state == NET_IN) {
        saying = in.chatOut;
        chatSeq++;
        nextChat = 0;
      }
      if (state == NET_IN && !saying.empty() && now >= nextChat) {
        nextChat = now + 0.25;
        net::Writer w(net::P_CHAT);
        w.put(token);
        w.put(chatSeq);
        w.put(u8(saying.size()));
        for (char ch : saying) w.put(ch);
        sock->send(w.data(), w.size(), server);
      }
      if (state == NET_IN && phaseNow == Duel::PLAYING && !droneShown[mySlot].live) {
        if (vehicleNow >= 0 && vehicleNow < Duel::kTanks) {
          const Tank& t = duel.tanks[vehicleNow];
          bool otherGunner = false;
          for (int k = 0; k < Duel::kMaxPlayers; k++) otherGunner |= k != mySlot && duel.players[k].present && duel.players[k].vehicle == vehicleNow && duel.players[k].seat == 1;
          if (in.c.fire && !otherGunner && now >= ownGunReady && !t.destroyed && !t.missileLive && t.reload <= 0 && t.ammo != 0 && t.mod[TM_GUN] > 0) {
            ownGunReady = now + std::max(t.reloadTime * 0.5, 0.3);
            sim.event(EV_MUZZLE, t.muzzle, t.gunDir, 1.f);
            shotsAhead++;
            shotsAheadUntil = now + 1.0;
          }
        } else if (vehicleNow < 0 && shownSoldier[mySlot].hp > 0) {
          const int w = in.weapon;
          const Duel::Player& pl = duel.players[mySlot];
          const int ammoLeft = w == WP_SNIPER ? pl.sniperAmmo : (w == WP_RPG ? pl.rpgAmmo : pl.smallAmmo);
          const bool pulled = in.c.fire || (in.fireHeld && kWeapons[w].automatic);
          if (w != WP_KNIFE && pulled && pl.hasWeapon[w] && now >= ownGunReady && ammoLeft > 0 && !(w == WP_SNIPER && me.prone < 0.99)) {
            ownGunReady = now + kWeapons[w].interval;
            const V3 eye = Duel::eyeOf(me);
            const V3 dir = norm(in.c.aimPoint - eye);
            sim.event(EV_MUZZLE, eye + dir * 0.7, dir, muzzleSize(w));
            shotsAhead++;
            shotsAheadUntil = now + 1.0;
          }
        }
      }
      if (now > shotsAheadUntil) shotsAhead = 0;
      if (in.c.fire) out.fireCount++;
      if (in.c.launch) out.launchCount++;
      if (in.c.repair) out.repairCount++;
      if (in.c.use) out.useCount++;
      if (in.c.setup) out.setupCount++;
      if (in.c.placeFoot) out.mineFootCount++;
      if (in.c.placeTank) out.mineTankCount++;
      if (in.c.query) out.queryCount++;
      if (in.c.smoke) out.smokeCount++;
      if (in.c.net) out.netCount++;
      if (in.c.drone) out.droneCount++;
      if (in.c.medkit) out.medkitCount++;
      if (in.c.strike) {
        out.strikeCount++;
        out.strikeX = float(in.c.strikeAt.x);
        out.strikeY = float(in.c.strikeAt.y);
      }

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
      if (sock && fakeLine) {
        while ((n = sock->recv(buf, sizeof buf, from)) >= 0) fakeHold(held, now, buf, size_t(n), from);
        while (!heldOut.empty() && heldOut.front().at <= now) {
          sock->send(heldOut.front().bytes.data(), heldOut.front().bytes.size(), heldOut.front().from);
          heldOut.pop_front();
        }
      }
      for (;;) {
        const bool inBundle = !bundled.empty();
        if (inBundle) {
          n = int(std::min(bundled.front().size(), sizeof buf));
          memcpy(buf, bundled.front().data(), size_t(n));
          bundled.pop_front();
          from = server;
        } else if (replaying) {
          if (replayAt >= lastTape.size() || lastTape[replayAt].at > replayNow) break;
          const std::vector<u8>& b = lastTape[replayAt++].bytes;
          n = int(std::min(b.size(), sizeof buf));
          memcpy(buf, b.data(), size_t(n));
          from = server;
        } else if (fakeLine) {
          if (held.empty() || held.front().at > now || !sock) break;
          n = int(held.front().bytes.size());
          memcpy(buf, held.front().bytes.data(), size_t(n));
          from = held.front().from;
          held.pop_front();
        } else if (!sock || (n = sock->recv(buf, sizeof buf, from)) < 0) {
          break;
        }
        if (!(from == server)) continue;
        if (!inBundle) {
          logPackets++;
          logBytes += size_t(n);
          lineBytes += size_t(n);
        }
        net::Reader r(buf, size_t(n));
        u8 kind = 0;
        if (!r.header(kind)) continue;
        if (kind == net::P_BUNDLE) {
          for (;;) {
            u16 length = 0;
            r.get(length);
            if (!r.ok() || length == 0 || length > net::kMaxPacket) break;
            std::vector<u8> one(length);
            for (u8& b : one) r.get(b);
            if (!r.ok()) break;
            bundled.push_back(std::move(one));
          }
          continue;
        }
        if (kind == net::P_FULL) {
          u8 why = 0;
          r.get(why);
          disconnect(why ? "Der Server hat eine andere Spielversion" : "Der Server ist voll");
          break;
        }
        if (state == NET_IN && kind != net::P_WELCOME && tapeBytes < (size_t(300) << 20)) {
          // On tape, for seeing it again afterwards. What came in for the other tanks is on it too.
          tape.push_back({now, std::vector<u8>(buf, buf + n)});
          tapeBytes += size_t(n);
          tapeOffset = timeOffset;
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
          if (!r.ok() || protocol != net::kProtocol || s >= Duel::kMaxPlayers || state != NET_CONNECTING) continue;
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
        if (state != NET_IN && state != NET_REPLAY) continue;
        u32 tick = 0, id = 0;
        r.get(tick);
        r.get(id);
        if (!r.ok() || id < match) continue;
        if (id > match) {
          // The match before this one is finished: that is the one to look at again.
          if (state == NET_IN && match != 0) {
            std::vector<Taped> current;
            current.push_back(std::move(tape.back()));
            tape.pop_back();
            keepTape();
            tape = std::move(current);
          }
          resetWorld();
          match = id;
          // Everyone's dice are the same.
          duel.matchId = id;
          duel.rollLoot();
        }
        lastHeard = now;
        if (kind == net::P_TANK) {
          u8 idx = 0;
          r.get(idx);
          if (idx >= Duel::kTanks) continue;
          if (!shapes.ready) {
            shapes.build(duel);
            toldTanks = shapes.restTanks;
            toldCars = shapes.restCars;
          }
          NetFrame f;
          f.tick = tick;
          f.cog = r.getV3();
          net::VehiclePose pose;
          pose.cog = f.cog;
          for (i16& v : pose.q) r.get(v);
          f.state = tankSaid[idx];
          net::readState(r, f.state, f.cog);
          pose.yaw = f.state.turretYaw;
          pose.pitch = f.state.gunPitch;
          pose.recoil = f.state.recoil;
          if (!r.ok()) continue;
          auto& q = frames[idx];
          // Comes back around every now and then whether anything has happened or not.
          if (!q.empty() && q.back().tick >= tick) continue;
          tankSaid[idx] = f.state;
          i16* told = toldTanks.data() + shapes.tankAt(int(idx));
          u16 count = 0;
          r.get(count);
          for (int i = 0; i < count; i++) {
            u16 node = 0;
            i16 l[3] = {};
            r.get(node);
            r.get(l[0]);
            r.get(l[1]);
            r.get(l[2]);
            if (!r.ok() || node >= duel.tankNodes) break;
            memcpy(told + size_t(node) * 3, l, sizeof l);
          }
          f.offsets.resize(size_t(duel.tankNodes) * 3);
          shapes.tankOffsets(int(idx), pose, told, f.offsets.data());
          f.arrived = now;
          q.push_back(std::move(f));
          while (q.size() > kFramesKept) q.pop_front();
        } else if (kind == net::P_META) {
          u32 metaSeq = 0;
          r.get(metaSeq);
          if (tick <= metaTick) continue;
          if (lineSeq != 0 && !replaying && int32_t(metaSeq - lineSeq) > 0) lineLost += std::min(int(metaSeq - lineSeq) - 1, 1000);
          lineSeq = metaSeq;
          lineGot++;
          if (!replaying) {
            // Where this machine's clock stands against the server's: the quickest packets lately, for those were
            // held up by nothing. How much later than that the others come is the line's jitter.
            const double seen = now - double(tick) / net::kStepRate;
            if (!haveOffset || std::fabs(seen - timeOffset) > 1.0) {
              timeOffset = seen;
              lineJitter = 0.01;
              haveOffset = true;
            } else {
              timeOffset += (seen - timeOffset) * (seen < timeOffset ? std::min(20.0 / net::kSendRate, 1.0) : 0.5 / net::kSendRate);
              lineJitter = std::max(std::max(seen - timeOffset, 0.0), lineJitter * std::exp(-0.3 / net::kSendRate));
            }
          }
          u8 ph = 0, mask = 0;
          i8 win = 0;
          float pc = 0;
          r.get(ph);
          r.get(win);
          r.get(pc);
          r.get(mask);
          i8 vehicles[Duel::kMaxPlayers];
          u8 seats[Duel::kMaxPlayers] = {};
          u8 readyMask = 0, wantedMask = 0, toldMask = 0;
          r.get(readyMask);
          r.get(wantedMask);
          r.get(toldMask);
          wantedNow = wantedMask;
          for (int i = 0; i < Duel::kMaxPlayers; i++) {
            vehicles[i] = -1;
            duel.players[i].present = mask & (1 << i);
            if (!(mask & (1 << i))) continue;
            r.get(vehicles[i]);
            r.get(seats[i]);
          }
          for (int i = 0; i < Duel::kMaxPlayers; i++) {
            if (!(toldMask & (1 << i))) continue;
            Soldier& sd = shownSoldier[i];
            sd.p = r.getV3();
            sd.v = net::getCmV3(r);
            i16 yaw = 0, pitch = 0, hp = 0;
            float stride = 0;
            u8 prone = 0, weapon = 0, flags = 0;
            r.get(yaw);
            r.get(pitch);
            r.get(hp);
            r.get(stride);
            r.get(prone);
            r.get(weapon);
            r.get(flags);
            sd.prone = prone / 255.0;
            weaponShown[i] = std::min<int>(weapon, WP_COUNT - 1);
            ghillieShown[i] = flags & 1;
            if (flags & 4) blowShown[i] = net::getCmV3(r);
            const bool flying = flags & 2;
            droneShown[i].live = flying;
            if (flying) {
              droneShown[i].p = r.getV3();
              droneShown[i].v = r.getV3();
              float battery = 0;
              r.get(battery);
              droneShown[i].battery = battery;
              droneShown[i].heardAt = now;
            }
            sd.yaw = yaw / 32767.0 * PI;
            sd.pitch = pitch / 32767.0 * PI;
            sd.hp = hp / 10.0;
            sd.stride = stride;
          }
          u32 ack = 0;
          r.get(ack);
          u8 stepsSinceAck = 0, onGround = 1;
          r.get(stepsSinceAck);
          r.get(onGround);
          r.get(duel.players[mySlot].hitsLanded);
          r.get(duel.players[mySlot].hitsTaken);
          u32 first = 0;
          u8 count = 0;
          r.get(first);
          r.get(count);
          if (!r.ok()) continue;
          metaTick = tick;
          for (int i = 0; i < Duel::kMaxPlayers; i++) {
            if (!(toldMask & (1 << i))) continue;
            soldierPast[i].push_back({tick, shownSoldier[i], vehicles[i]});
            while (soldierPast[i].size() > 96) soldierPast[i].pop_front();
          }
          phaseNow = ph;
          winner = win;
          clockNow = pc;
          vehicleNow = vehicles[mySlot];
          seatNow = seats[mySlot];
          hereNow = 0;
          for (int i = 0; i < Duel::kMaxPlayers; i++) hereNow += (mask >> i) & 1;
          ackedInput = ack;
          ackedSteps = stepsSinceAck;
          shownSoldier[mySlot].grounded = onGround != 0;
          logMetas++;
          logMetaGap = std::max(logMetaGap, now - logMetaAt);
          logMetaAt = now;
          while (!sentAt.empty() && int32_t(sentAt.front().first - ack) < 0) sentAt.pop_front();
          if (!sentAt.empty() && sentAt.front().first == ack) rtt += (now - sentAt.front().second - rtt) * 0.1;
          serverP = shownSoldier[mySlot].p;
          freshAnswer = true;
          for (int i = 0; i < Duel::kMaxPlayers; i++) {
            duel.players[i].vehicle = vehicles[i];
            duel.players[i].seat = seats[i];
            duel.players[i].ready = readyMask & (1 << i);
            duel.players[i].soldier.hp = shownSoldier[i].hp;
          }
          for (u32 i = first; i < first + count; i++) {
            const V3 c = r.getV3();
            float impulse = 0, radius = 0, hole = 1;
            r.get(impulse);
            r.get(radius);
            r.get(hole);
            if (!r.ok()) break;
            if (i == blastsDone) {
              Later blast;
              blast.tick = tick;
              blast.what = Later::BLAST;
              blast.p = c;
              blast.a = impulse;
              blast.b = radius;
              blast.c = hole;
              later.push_back(blast);
              blastsDone++;
            }
          }
          u8 rockets = 0;
          r.get(rockets);
          RocketsThen rocketsNow;
          rocketsNow.tick = tick;
          for (int i = 0; i < rockets; i++) {
            u8 what = 0;
            r.get(what);
            const V3 p = r.getV3();
            const V3 v = r.getV3();
            if (r.ok()) rocketsNow.list.push_back({what, p, v});
          }
          if (!rocketsNow.list.empty() || (!rocketPast.empty() && !rocketPast.back().list.empty())) rocketPast.push_back(std::move(rocketsNow));
          u8 nEv = 0;
          r.get(nEv);
          for (int i = 0; i < nEv; i++) {
            const V3 p = r.getV3();
            const V3 v = r.getV3();
            float strength = 0;
            u8 evKind = 0;
            r.get(strength);
            r.get(evKind);
            if (!r.ok() || evKind > EV_SMOKE) break;
            if (evKind == EV_MUZZLE && shotsAhead > 0 && !replaying) {
              const V3 mine = vehicleNow >= 0 ? duel.frames[vehicleNow].cog : me.p;
              if (len(p - mine) < 15.0) {
                shotsAhead--;
                if (gPlayLog) fprintf(gPlayLog, "t %.1f eigener Schuss bestaetigt nach %.0f ms\n", now, (now - (shotsAheadUntil - 1.0)) * 1000.0);
                continue;
              }
            }
            Later ev;
            ev.tick = tick;
            ev.kind = evKind;
            ev.p = p;
            ev.v = v;
            ev.a = strength;
            later.push_back(ev);
          }
        } else if (kind == net::P_CHATLOG) {
          u32 ack = 0, from = 0;
          u8 count = 0;
          r.get(ack);
          r.get(from);
          r.get(count);
          if (!r.ok()) continue;
          chatAcked = ack;
          if (int32_t(chatAcked - chatSeq) >= 0) saying.clear();
          for (u32 i = from; i < from + count; i++) {
            u8 who = 0, length = 0;
            r.get(who);
            r.get(length);
            std::string text;
            for (int k = 0; k < length; k++) {
              char ch = 0;
              r.get(ch);
              text += ch;
            }
            if (!r.ok() || who > Duel::kMaxPlayers) break;
            const u32 have = chatLogFirst + u32(chatLog.size());
            if (chatLog.empty()) chatLogFirst = from;
            if (i == chatLogFirst + u32(chatLog.size()) || (have <= i && chatLog.empty())) chatLog.push_back({who, text});
            else if (i > have) {
              // Gone past the window of what comes round: start again from here.
              chatLog.clear();
              chatLogFirst = i;
              chatLog.push_back({who, text});
            }
          }
          std::lock_guard<std::mutex> lock(snapMutex);
          snap.chat.clear();
          const size_t show = std::min<size_t>(chatLog.size(), 8);
          for (size_t i = chatLog.size() - show; i < chatLog.size(); i++) snap.chat.push_back((chatLog[i].first == Duel::kMaxPlayers ? std::string("Server") : names[chatLog[i].first]) + ": " + chatLog[i].second);
        } else if (kind == net::P_NAMES) {
          std::lock_guard<std::mutex> lock(snapMutex);
          for (int i = 0; i < Duel::kMaxPlayers; i++) {
            char name[net::kNameBytes];
            for (char& ch : name) r.get(ch);
            name[net::kNameBytes - 1] = 0;
            if (r.ok()) names[i] = name;
          }
        } else if (kind == net::P_SELF) {
          r.get(wins[0]);
          r.get(wins[1]);
          for (int i = 0; i < IT_COUNT; i++) {
            u8 count = 0;
            r.get(count);
            carrying[i] = count;
          }
          float rc = 0;
          r.get(rc);
          repairNow = rc;
          i16 small = 0, sniper = 0, rpg = 0;
          u8 has = 0;
          r.get(small);
          r.get(sniper);
          r.get(rpg);
          r.get(has);
          duel.players[mySlot].smallAmmo = small;
          duel.players[mySlot].sniperAmmo = sniper;
          duel.players[mySlot].rpgAmmo = rpg;
          for (int k = 0; k < WP_COUNT; k++) duel.players[mySlot].hasWeapon[k] = has & (1 << k);
          duel.players[mySlot].hurtFrom = r.getV3();
          duel.wind = r.getV3();
          i16 shells = 0, missiles = 0, codes = 0;
          r.get(shells);
          r.get(missiles);
          r.get(codes);
          duel.players[mySlot].tankShells = shells;
          duel.players[mySlot].tankMissiles = missiles;
          duel.players[mySlot].strikeCodes = codes;
          u8 dronesHeld = 0, medkitsHeld = 0;
          r.get(dronesHeld);
          duel.players[mySlot].drones = dronesHeld;
          r.get(medkitsHeld);
          duel.players[mySlot].medkits = medkitsHeld;
          u8 serverLoad = 0, serverReal = 100;
          u16 serverAwake = 0;
          r.get(serverLoad);
          r.get(serverReal);
          r.get(serverAwake);
          serverStats[0] = serverLoad / 100.0;
          serverStats[1] = serverReal / 100.0;
          serverStats[2] = serverAwake;
          u8 tap = 0;
          float tapClock = 0, fixWait = 0, fixAge = 0;
          r.get(tap);
          r.get(tapClock);
          r.get(fixWait);
          r.get(fixAge);
          const V3 fix = r.getV3();
          Duel::Player& self = duel.players[mySlot];
          self.tapped = tap & 1;
          self.hasFix = tap & 2;
          self.tapClock = tapClock;
          self.fixWait = fixWait;
          self.fixAge = fixAge;
          self.fix = fix;
        } else if (kind == net::P_CAR) {
          u8 k = 0;
          r.get(k);
          if (k >= Duel::kCars) continue;
          if (!shapes.ready) {
            shapes.build(duel);
            toldTanks = shapes.restTanks;
            toldCars = shapes.restCars;
          }
          CarFrame f;
          f.tick = tick;
          f.cog = r.getV3();
          net::VehiclePose pose;
          pose.cog = f.cog;
          for (i16& v : pose.q) r.get(v);
          net::unpackAxes(pose.q, f.fwd, f.left, f.up);
          i16 speed = 0, rpm = 0, wheelAngle = 0;
          u8 gas = 0, brake = 0, flags = 0, slow = 0;
          i8 steer = 0, lat = 0, lon = 0, gear = 0;
          r.get(speed);
          r.get(rpm);
          r.get(gas);
          r.get(brake);
          r.get(steer);
          r.get(lat);
          r.get(lon);
          for (float& th : f.theta) {
            r.get(wheelAngle);
            th = float(wheelAngle / 32767.0 * PI);
          }
          r.get(gear);
          r.get(flags);
          r.get(f.flat);
          r.get(slow);
          CarShown& c = shown[k];
          if (slow) {
            r.get(c.fuel);
            r.get(c.coolant);
          }
          if (!r.ok()) continue;
          f.speed = speed / 100.0f;
          f.rpm = rpm;
          f.throttle = gas / 255.0f;
          f.brake = brake / 255.0f;
          f.steer = steer / 127.0f;
          f.lat = lat / 3.0f;
          f.lon = lon / 3.0f;
          f.gear = gear;
          f.exploded = flags & 1;
          f.hurt = flags >> 1;
          f.fuel = c.fuel;
          f.coolant = c.coolant;
          if (!c.frames.empty() && c.frames.back().tick >= tick) continue;
          i16* told = toldCars.data() + shapes.carAt(int(k));
          u16 count = 0;
          r.get(count);
          for (int i = 0; i < count; i++) {
            u16 node = 0;
            i16 l[3] = {};
            r.get(node);
            r.get(l[0]);
            r.get(l[1]);
            r.get(l[2]);
            if (!r.ok() || node >= shapes.carCount(int(k))) break;
            memcpy(told + size_t(node) * 3, l, sizeof l);
          }
          f.offsets.resize(size_t(shapes.carCount(int(k))) * 3);
          shapes.carOffsets(int(k), pose, told, f.offsets.data());
          f.arrived = now;
          c.frames.push_back(std::move(f));
          while (c.frames.size() > kFramesKept) c.frames.pop_front();
        } else if (kind == net::P_SCENERY) {
          u8 count = 0;
          r.get(count);
          for (int i = 0; i < count; i++) {
            u32 k = 0;
            r.get(k);
            const V3 p = r.getV3();
            if (!r.ok()) break;
            if (k < sim.firstSceneryNode || k >= sim.pos.size()) continue;
            Later moved;
            moved.tick = tick;
            moved.what = Later::NODE;
            moved.node = k;
            moved.p = p;
            later.push_back(moved);
          }
        }
      }
      if (state == NET_IN && now - lastHeard > 5.0) disconnect("Verbindung zum Server verloren");

      // Every tank we have word of, at the moment a little in the past that lies between two frames.
      if (!replaying) {
        const double wantDelay = clampd(1.5 / 30.0 + lineJitter + 0.005, 0.05, 0.25);
        viewDelay += clampd(wantDelay - viewDelay, -0.02 * dt, 0.1 * dt);
      }
      const double viewTime = ((replaying ? replayNow : shownNow) - timeOffset - viewDelay) * net::kStepRate;
      // The moment on the screen, for the server to judge by.
      const u32 viewTick = u32(std::max(viewTime, 0.0));
      while (!later.empty() && double(later.front().tick) <= viewTime) {
        const Later& l = later.front();
        if (l.what == Later::BLAST) sim.explode(l.p, l.a, l.b, l.c);
        else if (l.what == Later::EVENT) sim.event(EventKind(l.kind), l.p, l.v, l.a);
        else if (len2(sim.pos[l.node] - l.p) > 8.0 * 8.0) sim.pos[l.node] = l.p;
        else sliding[l.node] = l.p;
        later.pop_front();
      }
      for (auto it = sliding.begin(); it != sliding.end();) {
        V3& at = sim.pos[it->first];
        at += (it->second - at) * clampd(dt * 30.0, 0, 1);
        if (len2(it->second - at) < 1e-6) {
          at = it->second;
          it = sliding.erase(it);
        } else {
          ++it;
        }
      }
      for (int side = 0; side < Duel::kTanks; side++) {
        const auto& q = frames[side];
        if (q.empty()) continue;
        size_t b = 0;
        while (b + 1 < q.size() && double(q[b].tick) < viewTime) b++;
        const NetFrame& fb = q[b];
        const NetFrame& fa = b > 0 ? q[b - 1] : fb;
        const double span = double(fb.tick) - double(fa.tick);
        const double a = span > 0 ? clampd((viewTime - double(fa.tick)) / span, 0, 1) : 1.0;
        Tank& t = duel.tanks[side];
        const u32 first = duel.tankFirst[side];
        const size_t count = fb.offsets.size() / 3;
        // The driver has the gun too, as long as nobody else has taken the other seat.
        bool aimingThis = side == vehicleNow && !replaying && state == NET_IN;
        for (int who = 0; who < Duel::kMaxPlayers && aimingThis && seatNow == 0; who++) aimingThis = !(who != mySlot && duel.players[who].present && duel.players[who].vehicle == side && duel.players[who].seat == 1);
        if (side == vehicleNow && seatNow == 0 && q.size() >= 2 && !replaying) {
          // One's own tank is not shown late but early: where it will be by the time one's latest input has arrived.
          // That is what makes it answer at once instead of after the ping.
          const NetFrame& last = q.back();
          // Two frames in a row say more about the suspension than about where the thing is going.
          size_t older = q.size() - 2;
          while (older > 0 && double(last.tick) - double(q[older].tick) < 0.08 * net::kStepRate) older--;
          const NetFrame& before = q[older];
          const double ticks = std::max(1.0, double(last.tick) - double(before.tick));
          const V3 vel = last.state.vel;
          const auto heading = [&](const NetFrame& fr) {
            const auto node = [&](u32 id) {
              const size_t k = size_t(id - first);
              return V3{double(fr.offsets[k * 3]), double(fr.offsets[k * 3 + 1]), double(fr.offsets[k * 3 + 2])};
            };
            const V3 d = node(t.refFL) + node(t.refFR) - node(t.refRL) - node(t.refRR);
            return std::atan2(d.y, d.x);
          };
          const double h1 = heading(last);
          const double seen = std::remainder(h1 - heading(before), 2.0 * PI) * (net::kStepRate / ticks);
          const double absV = len(vel);
          const double want = double(in.c.steer) * (absV < 5.0 ? 1.1 : clampd(7.5 / absV, 0.32, 1.1));
          // How old the frame is against the server's steady clock, not when this one happened to come in: a packet held
          // up on the way must not move the tank, and the whole world around the camera with it.
          const double age = clampd(shownNow - timeOffset - double(last.tick) / net::kStepRate, -0.03, 0.1);
          const double turn = ownPredict(side, last.cog, vel, h1, seen, want, age, dtShown);
          const double c = std::cos(turn), sn = std::sin(turn);
          const V3 cog = own.p;
          if (scriptMode == 63) {
            static double nextSay = 0;
            if (now > nextSay) {
              printf("Laufzeit %.0f ms, Rahmenalter %.0f ms, Vorlauf Kurs %.2f Grad, Turm %.2f Grad\n", rtt * 1000.0, age * 1000.0, turn * 180.0 / PI, std::remainder(ownAim.yaw - double(last.state.turretYaw), 2.0 * PI) * 180.0 / PI);
              nextSay = now + 20.0;
            }
          }
          size_t ia = 0, ib = 0;
          double blend = 1;
          shapeBetween(q, shownNow, ia, ib, blend);
          const NetFrame& sa = q[ia];
          const NetFrame& sb = q[ib];
          // The shape as it was then, turned to the heading it has now.
          const double hA = heading(sa);
          const double shapeTurn = h1 + turn - (hA + std::remainder(heading(sb) - hA, 2.0 * PI) * blend);
          const double cs = std::cos(shapeTurn), ss = std::sin(shapeTurn);
          const size_t n3 = sb.offsets.size() / 3;
          for (size_t k = 0; k < n3; k++) {
            const V3 oa{double(sa.offsets[k * 3]), double(sa.offsets[k * 3 + 1]), double(sa.offsets[k * 3 + 2])};
            const V3 ob{double(sb.offsets[k * 3]), double(sb.offsets[k * 3 + 1]), double(sb.offsets[k * 3 + 2])};
            const V3 o = oa + (ob - oa) * blend;
            sim.pos[first + k] = cog + V3{o.x * cs - o.y * ss, o.x * ss + o.y * cs, o.z};
          }
          t.applyNetState(sim, last.state, dt);
          // What is aimed with goes where the gun is drawn.
          const auto ahead = [&](V3 d) { return V3{d.x * c - d.y * sn, d.x * sn + d.y * c, d.z}; };
          t.muzzle = cog + ahead(last.state.muzzle - last.cog);
          t.sight = cog + ahead(last.state.sight - last.cog);
          t.gunDir = ahead(last.state.gunDir);
          if (aimingThis) aimOwnGun(t, side, in.c, dtShown);
          continue;
        }
        const double spanSeconds = span / net::kStepRate;
        const double late = (viewTime - double(fb.tick)) / net::kStepRate;
        V3 cog = spanSeconds > 0 && spanSeconds < 0.25 ? curveBetween(fa.cog, fa.state.vel, fb.cog, fb.state.vel, a, spanSeconds) : fa.cog + (fb.cog - fa.cog) * a;
        if (late > 0) cog = fb.cog + fb.state.vel * std::min(late, kCoastAtMost);
        for (size_t k = 0; k < count; k++) {
          const V3 oa{double(fa.offsets[k * 3]), double(fa.offsets[k * 3 + 1]), double(fa.offsets[k * 3 + 2])};
          const V3 ob{double(fb.offsets[k * 3]), double(fb.offsets[k * 3 + 1]), double(fb.offsets[k * 3 + 2])};
          sim.pos[first + k] = cog + oa + (ob - oa) * a;
        }
        t.applyNetState(sim, fb.state, dt);
        if (aimingThis) aimOwnGun(t, side, in.c, dtShown);
      }

      for (int k = 0; k < Duel::kCars; k++) {
        const auto& q = shown[k].frames;
        if (q.empty()) continue;
        size_t b = 0;
        while (b + 1 < q.size() && double(q[b].tick) < viewTime) b++;
        const CarFrame& fb = q[b];
        const CarFrame& fa = b > 0 ? q[b - 1] : fb;
        const double span = double(fb.tick) - double(fa.tick);
        const double a = span > 0 ? clampd((viewTime - double(fa.tick)) / span, 0, 1) : 1.0;
        // How fast it was going at a frame, from those on either side of it.
        const auto carVel = [&](size_t i) {
          const CarFrame& x = q[i > 0 ? i - 1 : i];
          const CarFrame& y = q[i + 1 < q.size() ? i + 1 : i];
          const double seconds = (double(y.tick) - double(x.tick)) / net::kStepRate;
          return seconds > 0 && seconds < 0.4 ? (y.cog - x.cog) / seconds : V3{};
        };
        const double spanSeconds = span / net::kStepRate;
        const double late = (viewTime - double(fb.tick)) / net::kStepRate;
        V3 cog = spanSeconds > 0 && spanSeconds < 0.25 ? curveBetween(fa.cog, carVel(b - 1), fb.cog, carVel(b), a, spanSeconds) : fa.cog + (fb.cog - fa.cog) * a;
        if (late > 0) cog = fb.cog + carVel(b) * std::min(late, kCoastAtMost);
        const u32 first = duel.carFirst[k];
        const size_t count = fb.offsets.size() / 3;
        double ownTurn = 0;
        if (vehicleNow == Duel::kTanks + k && q.size() >= 2 && !replaying) {
          // One's own car, like one's own tank: shown ahead, not behind.
          const CarFrame& last = q.back();
          size_t older = q.size() - 2;
          while (older > 0 && double(last.tick) - double(q[older].tick) < 0.08 * net::kStepRate) older--;
          const CarFrame& before = q[older];
          const double ticks = std::max(1.0, double(last.tick) - double(before.tick));
          const V3 vel = (last.cog - before.cog) * (net::kStepRate / ticks);
          const double seen = std::remainder(std::atan2(last.fwd.y, last.fwd.x) - std::atan2(before.fwd.y, before.fwd.x), 2.0 * PI) * (net::kStepRate / ticks);
          const double want = double(in.c.steer) * 0.45 * last.speed / 2.7;  // the bicycle model, with the lock it has
          const double age = clampd(shownNow - timeOffset - double(last.tick) / net::kStepRate, -0.03, 0.1);
          ownTurn = ownPredict(Duel::kTanks + k, last.cog, vel, std::atan2(last.fwd.y, last.fwd.x), seen, want, age, dtShown);
          cog = own.p;
          size_t ia = 0, ib = 0;
          double blend = 1;
          shapeBetween(q, shownNow, ia, ib, blend);
          const CarFrame& sa = q[ia];
          const CarFrame& sb = q[ib];
          const double hA = std::atan2(sa.fwd.y, sa.fwd.x);
          const double shapeTurn = std::atan2(last.fwd.y, last.fwd.x) + ownTurn - (hA + std::remainder(std::atan2(sb.fwd.y, sb.fwd.x) - hA, 2.0 * PI) * blend);
          const double c2 = std::cos(shapeTurn), s2 = std::sin(shapeTurn);
          for (size_t i = 0; i < sb.offsets.size() / 3; i++) {
            const V3 oa{double(sa.offsets[i * 3]), double(sa.offsets[i * 3 + 1]), double(sa.offsets[i * 3 + 2])};
            const V3 ob{double(sb.offsets[i * 3]), double(sb.offsets[i * 3 + 1]), double(sb.offsets[i * 3 + 2])};
            const V3 o = oa + (ob - oa) * blend;
            sim.pos[first + i] = cog + V3{o.x * c2 - o.y * s2, o.x * s2 + o.y * c2, o.z};
          }
        } else {
          for (size_t i = 0; i < count; i++) {
            const V3 oa{double(fa.offsets[i * 3]), double(fa.offsets[i * 3 + 1]), double(fa.offsets[i * 3 + 2])};
            const V3 ob{double(fb.offsets[i * 3]), double(fb.offsets[i * 3 + 1]), double(fb.offsets[i * 3 + 2])};
            sim.pos[first + i] = cog + oa + (ob - oa) * a;
          }
        }
        Vehicle& v = duel.cars[size_t(k)];
        // In the simulation a wheel's rim stays put and only its angle goes round; here the drawn wheel goes round with it.
        for (size_t w = 0; w < v.wheels.size() && w < 4; w++) {
          const Wheel& wheel = v.wheels[w];
          // The shortest way between the two pictures, whatever the angle's wrapping has done.
          const double from = fa.theta[w], to = fb.theta[w];
          const double turn = std::remainder(to - from, 2.0 * PI);
          const double theta = from + turn * a;
          const V3 hub = (sim.pos[wheel.axIn] + sim.pos[wheel.axOut]) * 0.5;
          // Rolling forward turns a wheel about the axis pointing to the car's left, on both sides of it.
          V3 axis = norm(sim.pos[wheel.axOut] - sim.pos[wheel.axIn]);
          if (!wheel.left) axis = axis * -1.0;
          const double c = std::cos(theta), sn = std::sin(theta);
          for (u32 n : wheel.ring) {
            const V3 r = sim.pos[n] - hub;
            sim.pos[n] = hub + r * c + cross(axis, r) * sn + axis * (dot(axis, r) * (1.0 - c));
          }
        }
        v.cog = cog;
        v.fwd = norm(fa.fwd + (fb.fwd - fa.fwd) * a);
        v.left = norm(fa.left + (fb.left - fa.left) * a);
        if (ownTurn != 0) {
          const auto turned = [&](V3 d) { return V3{d.x * std::cos(ownTurn) - d.y * std::sin(ownTurn), d.x * std::sin(ownTurn) + d.y * std::cos(ownTurn), d.z}; };
          v.fwd = turned(q.back().fwd);
          v.left = turned(q.back().left);
        }
        v.up = norm(fa.up + (fb.up - fa.up) * a);
        v.speed = lerpd(fa.speed, fb.speed, a);
        v.gear = fb.gear;
        v.exploded = fb.exploded;
        carHurt[k] = fb.hurt;
        carFlat[k] = fb.flat;
        v.engineOmega = lerpd(fa.rpm, fb.rpm, a) * 2.0 * PI / 60.0;
        v.throttleAct = fb.throttle;
        v.brakeAct = fb.brake;
        v.steerAct = fb.steer;
        v.fuelLitres = fb.fuel;
        v.dmg.coolantTemp = fb.coolant;
        v.latAcc = lerpd(fa.lat, fb.lat, a);
        v.lonAcc = lerpd(fa.lon, fb.lon, a);
      }
      duel.updateFrames();

      for (int i = 0; i < Duel::kMaxPlayers; i++) {
        const auto& q = soldierPast[i];
        if (q.empty()) {
          drawnSoldier[i] = shownSoldier[i];
          drawnVehicle[i] = duel.players[i].vehicle;
          continue;
        }
        size_t b = 0;
        while (b + 1 < q.size() && double(q[b].tick) < viewTime) b++;
        const SoldierFrame& fb = q[b];
        const SoldierFrame& fa = b > 0 ? q[b - 1] : fb;
        const double span = double(fb.tick) - double(fa.tick);
        const double a = span > 0 ? clampd((viewTime - double(fa.tick)) / span, 0, 1) : 1.0;
        Soldier s = fb.s;
        // Someone who has just got out of a vehicle or come back to life has not gone there on foot.
        if (fa.vehicle == fb.vehicle && len(fb.s.p - fa.s.p) < 5.0) {
          const double seconds = span / net::kStepRate;
          s.p = seconds > 0 && seconds < 0.25 ? curveBetween(fa.s.p, fa.s.v, fb.s.p, fb.s.v, a, seconds) : fa.s.p + (fb.s.p - fa.s.p) * a;
          s.v = fa.s.v + (fb.s.v - fa.s.v) * a;
          const double late = (viewTime - double(fb.tick)) / net::kStepRate;
          if (late > 0 && fb.s.hp > 0) s.p = fb.s.p + fb.s.v * std::min(late, kCoastAtMost);
          s.yaw = fa.s.yaw + std::remainder(fb.s.yaw - fa.s.yaw, 2.0 * PI) * a;
          s.pitch = lerpd(fa.s.pitch, fb.s.pitch, a);
          s.stride = lerpd(fa.s.stride, fb.s.stride, a);
          s.prone = lerpd(fa.s.prone, fb.s.prone, a);
        }
        drawnSoldier[i] = s;
        drawnVehicle[i] = a >= 1.0 ? fb.vehicle : fa.vehicle;
      }

      if (gPlayLog) {
        // How evenly the other player moves on this screen: 0 would be perfectly smooth.
        for (int k = 0; k < Duel::kMaxPlayers; k++) {
          if (k == mySlot || !duel.players[k].present) continue;
          const V3 at = drawnVehicle[k] >= 0 ? duel.frames[drawnVehicle[k]].cog : drawnSoldier[k].p;
          if (dtShown <= 0) break;
          const V3 step = (at - logFoeAt) / dtShown;
          if (len(at - logFoeAt) < 3.0) {
            logFoeSum += len(step - logFoeStep) * dtShown;
            logFoeN += 1;
          }
          logFoeAt = at;
          logFoeStep = step;
          break;
        }
      }

      // On foot one's own legs answer at once.
      duel.players[mySlot].vehicle = vehicleNow;
      const bool playingNow = (state == NET_IN || replaying) && (phaseNow == Duel::PLAYING || phaseNow == Duel::CHOOSING || phaseNow == Duel::FINISHED);
      const bool canWalk = state == NET_IN && phaseNow == Duel::PLAYING && !droneShown[mySlot].live;
      if (vehicleNow >= 0 || replaying) {
        me = shownSoldier[mySlot];
        walkTrue = me;
        walkOffset = V3{};
        walkSteps.clear();
      } else if (playingNow) {
        WalkInput wi;
        const double aimSlow = in.aiming ? 0.55 : 1.0;
        if (canWalk) {
          wi.forward = (in.c.throttle - in.c.brake) * aimSlow;
          wi.strafe = in.c.steer * aimSlow;
          wi.jump = in.c.handbrake > 0.5;
          wi.sprint = in.sprint && !in.aiming;
          if (shownSoldier[mySlot].prone > 0) wi.forward = wi.strafe = 0, wi.jump = wi.sprint = false;
        }
        wi.yaw = in.yaw;
        walkDebt += dtShown;
        walkDebt = std::min(walkDebt, 0.1);
        while (walkDebt >= Duel::kWalkDt) {
          walkDebt -= Duel::kWalkDt;
          duel.walk(walkTrue, wi, Duel::kWalkDt);
          walkSteps.push_back({out.seq + 1, wi});
        }
        while (walkSteps.size() > 100) walkSteps.pop_front();
        if (freshAnswer) {
          freshAnswer = false;
          // What the server has walked already: all that was meant for inputs it has used, and the few steps since.
          while (!walkSteps.empty() && int32_t(walkSteps.front().seq - ackedInput) <= 0) walkSteps.pop_front();
          for (int k = 0; k < ackedSteps && !walkSteps.empty(); k++) walkSteps.pop_front();
          Soldier answer = walkTrue;
          answer.p = shownSoldier[mySlot].p;
          answer.v = shownSoldier[mySlot].v;
          answer.grounded = shownSoldier[mySlot].grounded;
          for (const WalkStep& step : walkSteps) duel.walk(answer, step.in, Duel::kWalkDt);
          answer.stride = walkTrue.stride;
          answer.yaw = walkTrue.yaw;
          const V3 err = walkTrue.p - answer.p;
          logWalkErr = std::max(logWalkErr, len(err));
          walkOffset = len(err + walkOffset) > 4.0 ? V3{} : walkOffset + err;
          walkTrue = answer;
        }
        walkOffset = walkOffset * std::exp(-dtShown * 10.0);
        // Between two steps of walking too: at a run, one step more or less in a frame is a good handful of centimetres.
        me = walkTrue;
        me.p += walkOffset + walkTrue.v * walkDebt;
        me.pitch = in.pitch;
        me.prone = shownSoldier[mySlot].prone;
      } else {
        me = duel.players[mySlot].soldier;
      }
      for (int t = 0; t < Duel::kTanks; t++) Duel::poseNet(sim, duel.netFirst[t], duel.tanks[t], duel.tanks[t].netOn);
      // Drones, a little smoothed: what comes from the server would show on a bad line.
      for (int i = 0; i < Duel::kMaxPlayers; i++) {
        DroneSeen& ds = droneShown[i];
        if (ds.live) {
          const V3 target = ds.p + ds.v * std::clamp(now - ds.heardAt, 0.0, 0.1);
          if (len(target - ds.shown) > 30.0) ds.shown = target;
          ds.shown += (target - ds.shown) * clampd(dt * 18.0, 0, 1);
        }
        // One's own would only be in the way of its camera.
        Duel::poseDrone(sim, duel.droneFirst[i], ds.live && i != mySlot, ds.shown, 0.0);
      }
      {
        std::lock_guard<std::mutex> lock(snapMutex);
        snap.foot.droneLive = droneShown[mySlot].live;
        snap.foot.droneP = droneShown[mySlot].shown;
        snap.foot.droneBattery = droneShown[mySlot].battery;
        snap.foot.drones = duel.players[mySlot].drones;
        snap.foot.medkits = duel.players[mySlot].medkits;
        sightFrames.assign(duel.frames, duel.frames + Duel::kVehicles);
        sightFoes.clear();
        for (int k = 0; k < Duel::kMaxPlayers; k++) {
          if (Duel::teamOf(k) == Duel::teamOf(mySlot) || drawnVehicle[k] >= 0 || drawnSoldier[k].hp <= 0) continue;
          sightFoes.push_back({drawnSoldier[k].p, drawnSoldier[k].prone > 0.5 ? 0.6 : Soldier::kHeight});
        }
        snap.foot.launcher = vehicleNow >= 0 && vehicleNow < Duel::kTanks ? duel.tanks[vehicleNow].smokeCharges : 0;
      }
      // A guided missile is a real thing to look at, for whoever is flying it and whoever it comes for.
      for (int side = 0; side < 2; side++) {
        for (int hull = 0; hull < HK_COUNT; hull++) {
          const Tank& launcher = duel.tanks[Duel::tankIndex(side, TK_MISSILE, hull)];
          Duel::poseMissile(sim, duel.missileFirst[side * HK_COUNT + hull], launcher.missileLive, launcher.missileP, launcher.missileV);
        }
      }
      // And rockets, carried on from where they were at the moment on the screen.
      {
        while (rocketPast.size() > 1 && double(rocketPast[1].tick) <= viewTime) rocketPast.pop_front();
        int used[2] = {0, 0};
        const int limit[2] = {Duel::kRpgRockets, Duel::kStrikeRockets};
        const int base[2] = {0, Duel::kRpgRockets};
        const double scale[2] = {Duel::kRpgScale, Duel::kStrikeScale};
        const bool rocketsDue = !rocketPast.empty() && double(rocketPast.front().tick) <= viewTime;
        const double ahead = rocketsDue ? std::clamp((viewTime - double(rocketPast.front().tick)) / net::kStepRate, 0.0, 0.1) : 0.0;
        static const std::vector<RocketSeen> kNoRockets;
        for (const RocketSeen& rs : rocketsDue ? rocketPast.front().list : kNoRockets) {
          const int k = rs.what ? 1 : 0;
          if (used[k] < limit[k]) Duel::poseMissile(sim, duel.rocketFirst[base[k] + used[k]++], true, rs.p + rs.v * ahead, rs.v, scale[k]);
        }
        for (int k = 0; k < 2; k++)
          for (int i = used[k]; i < limit[k]; i++) Duel::poseMissile(sim, duel.rocketFirst[base[k] + i], false, V3{}, V3{1, 0, 0}, scale[k]);
      }
      // The soldiers drawn: oneself where one thinks one is, the others a moment ago, as the vehicles are.
      for (int i = 0; i < Duel::kMaxPlayers; i++) {
        // Through the scope one's own head would be in the way.
        // Someone who has died on foot falls as a body from then on.
        if (!duel.players[i].present) {
          Duel::poseSoldier(sim, duel.soldierFirst[i], drawnSoldier[i], true, 0);
          continue;
        }
        const bool isMe = i == mySlot && !replaying;
        const bool deadOnFoot = (state == NET_IN || replaying) && (isMe ? duel.players[i].soldier.hp <= 0 && duel.players[i].vehicle < 0 : drawnSoldier[i].hp <= 0 && drawnVehicle[i] < 0);
        if (deadOnFoot && !bodies[i].on) {
          Soldier last = isMe ? me : drawnSoldier[i];
          Duel::ragdollStart(bodies[i], last, i == mySlot ? in.weapon : weaponShown[i], blowShown[i]);
        }
        if (bodies[i].on) {
          duel.ragdollStep(bodies[i], dt);
          Duel::poseRagdoll(sim, duel.soldierFirst[i], bodies[i], ghillieShown[i]);
          // Blood spreads out from under it once it lies, on what is beneath the middle of it.
          const V3 mid = bodies[i].p[0];
          for (int d = 0; d < Duel::kBloodDiscs; d++) {
            // Spots running together, the big ones in the middle first and the small ones further out later: no two
            // pools the same, and none of them round.
            const double ang = double(d) * 2.399 + i * 1.3;
            const double out = 0.09 * std::sqrt(double(d)) * (1.0 + 0.5 * std::sin(double(d) * 5.3 + i));
            const bool show = bodies[i].clock > 0.6 + d * 0.45;
            const V3 off{std::cos(ang) * out * 2.2, std::sin(ang) * out, 0};
            const V3 at = mid + off;
            const double ground = world.groundUnder(at.x, at.y, mid.z + 0.5) + 0.018 + d * 0.002;
            const u32 n = duel.bloodFirst[i][d];
            sim.pos[n] = show ? V3{at.x, at.y, ground} : V3{0, 0, -390.0};
            sim.pos[n + 1] = sim.pos[n] + V3{0, 0, 0.01};
          }
          continue;
        }
        const bool hidden = (state != NET_IN && !replaying) || !playingNow || (isMe ? duel.players[i].vehicle >= 0 : drawnVehicle[i] >= 0) || (isMe && in.scoped);
        Duel::poseSoldier(sim, duel.soldierFirst[i], isMe ? me : drawnSoldier[i], hidden, i == mySlot && !replaying ? in.weapon : weaponShown[i], i == mySlot && !replaying && in.aiming && !in.scoped, ghillieShown[i]);
      }

      if (state == NET_IN && now >= nextInput && sock) {
        nextInput = std::max(nextInput + 1.0 / net::kSendRate, now - 0.02);
        out.seq++;
        sentAt.push_back({out.seq, now});
        while (sentAt.size() > 240) sentAt.pop_front();
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
        out.fireHeld = in.fireHeld;
        out.aiming = in.aiming;
        out.weapon = u8(in.weapon);
        if (gPlayLog) {
          logLoopGap = std::max(logLoopGap, now - logLoopAt);
          logLoopAt = now;
          if (now > logNext) {
            double nearest = 1e9;
            const V3 mine = vehicleNow >= 0 ? duel.frames[vehicleNow].cog : me.p;
            for (int k = 0; k < Duel::kMaxPlayers; k++) {
              if (k == mySlot || !duel.players[k].present) continue;
              const V3 them = duel.players[k].vehicle >= 0 ? duel.frames[duel.players[k].vehicle].cog : shownSoldier[k].p;
              nearest = std::min(nearest, len(them - mine));
            }
            const int awake = int(duel.sim.active.size());
            fprintf(gPlayLog, "t %.0f netz slot %d phase %d fz %d pos %.0f %.0f gegner %.0f m rtt %.0f ms metas %d luecke %.0f ms eingabepause %.0f ms laufkorrektur %.2f m gegner-ruck %.1f mm pakete %d kB %.0f aktiv %d\n", now, mySlot, phaseNow, vehicleNow, mine.x, mine.y, nearest, rtt * 1000.0, logMetas, logMetaGap * 1000.0, logLoopGap * 1000.0, logWalkErr, logFoeSum / std::max(logFoeN, 1.0) * 1000.0, logPackets, double(logBytes) / 1024.0, awake);
            fflush(gPlayLog);
            logWalkErr = logMetaGap = logLoopGap = logFoeSum = logFoeN = 0;
            logMetas = logPackets = 0;
            logBytes = 0;
            logNext = now + 1.0;
          }
        }
        out.ready = in.ready ? 1 : 0;
        for (int i = 0; i < IT_COUNT; i++) out.items[i] = in.items[i];
        net::Writer w(net::P_INPUT);
        net::write(w, out);
        if (fakeLine) fakeHold(heldOut, now, w.data(), w.size(), server);
        else sock->send(w.data(), w.size(), server);
      }

      if (vehicleNow >= 0 && vehicleNow < Duel::kTanks) {
        static const char* kLost[TM_COUNT] = {"Motor ausgefallen", "Turmdrehkranz zerstört", "Kanone zerstört", nullptr, "Linke Kette abgerissen", "Rechte Kette abgerissen", "Richtoptik zerstört", "Entfernungsmesser zerstört", "Fahrer-Winkelspiegel zerstört", "Kühler durchschossen", "Treibstoffleitung durchschossen", "Höhenrichtanlage zerstört", "Funkantenne zerstört"};
        const Tank& watched = duel.tanks[vehicleNow];
        for (int i = 0; i < TM_COUNT; i++) {
          // Those that bleed slowly say so on the first hit, the others when they are gone.
          const bool leaking = i == TM_RADIATOR || i == TM_FUEL;
          const bool lost = leaking ? (watched.mod[i] < 1 && lastMod[i] >= 1) : (watched.mod[i] <= 0 && lastMod[i] > 0);
          if (lost && lastTankWatched == vehicleNow && kLost[i]) {
            alert = kLost[i];
            alertUntil = now + 4.0;
          }
          lastMod[i] = watched.mod[i];
        }
        lastTankWatched = vehicleNow;
      } else {
        lastTankWatched = -1;
      }
      if (vehicleNow >= Duel::kTanks) {
        // What has just gone wrong with the car one is sitting in.
        const int k = vehicleNow - Duel::kTanks;
        if (carWatched == k) {
          static const char* kCarLost[5] = {"Motor getroffen: er steht", "Ölwanne leck", "Kühler leck", "Tank leck", "Getriebe getroffen"};
          for (int i = 0; i < 5; i++)
            if ((carHurt[k] & (1 << i)) && !(seenHurt & (1 << i))) {
              alert = kCarLost[i];
              alertUntil = now + 4.0;
            }
          if (carFlat[k] & ~seenFlat) {
            alert = "Reifen platt";
            alertUntil = now + 4.0;
          }
        }
        carWatched = k;
        seenHurt = carHurt[k];
        seenFlat = carFlat[k];
      } else {
        carWatched = -1;
      }
      char msg[128] = "";
      if (replaying) {
        snprintf(msg, sizeof msg, "Replay  ·  noch %.0f s  ·  %s  ·  TAB wechselt die Sicht  ·  [ ] Tempo %d %%", lastTape.back().at - replayNow, names[mySlot].c_str(), replaySpeed.load());
      } else if (state == NET_IN && now < alertUntil) {
        snprintf(msg, sizeof msg, "%s", alert.c_str());
      } else if (state == NET_IN) {
        const Tank* mine = vehicleNow >= 0 && vehicleNow < Duel::kTanks ? &duel.tanks[vehicleNow] : nullptr;
        if (phaseNow == Duel::WAITING) snprintf(msg, sizeof msg, "Warte auf Spieler  ·  %d von %d da", hereNow, wantedNow);
        else if (phaseNow == Duel::CHOOSING && in.ready) snprintf(msg, sizeof msg, "Warte auf den Gegner  ·  %.0f s", std::max(0.0, Duel::kChooseTime - double(clockNow)));
        else if (phaseNow == Duel::FINISHED) {
          std::lock_guard<std::mutex> lock(snapMutex);
          if (wantedNow > 2) {
            static const char* kTeam[2] = {"Team West", "Team Ost"};
            const int myTeam = Duel::teamOf(mySlot);
            snprintf(msg, sizeof msg, "%s  ·  %s %u : %u %s", winner < 0 ? "Unentschieden" : (winner == myTeam ? "Sieg!" : "Niederlage"), kTeam[myTeam], wins[myTeam], wins[1 - myTeam], kTeam[1 - myTeam]);
          } else {
            const int me = mySlot;
            const int other = 1 - me;
            snprintf(msg, sizeof msg, "%s  ·  %s %u : %u %s", winner < 0 ? "Unentschieden" : (winner == me ? "Sieg!" : "Niederlage"), names[me].c_str(), wins[me], wins[other], names[other].c_str());
          };
        }
        else if (repairNow > 0) snprintf(msg, sizeof msg, "Repariere  ·  %.0f s", repairNow);
        else if (droneShown[mySlot].live) snprintf(msg, sizeof msg, "Drohne  ·  Akku %.0f s  ·  Feuer markiert den Gegner  ·  Q: zurückholen", droneShown[mySlot].battery);
        else if (mine && mine->netClock > 0) snprintf(msg, sizeof msg, "Tarnnetz wird aufgezogen  ·  %.0f s", mine->netClock);
        else if (mine && mine->missileLive) snprintf(msg, sizeof msg, "Rakete in der Luft  ·  Maus lenkt  ·  Feuer sprengt");
        else if (vehicleNow >= 0) snprintf(msg, sizeof msg, seatNow == 0 ? "E: aussteigen" : "E: aussteigen  ·  Du bist Richtschütze");
        else if (duel.players[mySlot].tapClock > 0) snprintf(msg, sizeof msg, "Hauptcomputer wird angezapft  ·  %.0f s", duel.players[mySlot].tapClock);
        else if (phaseNow == Duel::PLAYING && vehicleNow < 0 && !duel.players[mySlot].tapped && duel.atEnemyComputer(mySlot, me.p)) snprintf(msg, sizeof msg, "E: Hauptcomputer anzapfen");
        else if (phaseNow == Duel::PLAYING && vehicleNow < 0 && duel.players[mySlot].tapped && duel.atEnemyComputer(mySlot, me.p)) snprintf(msg, sizeof msg, "Angezapft  ·  Q: Standort des Gegners abfragen, K zeigt ihn");
        else if (phaseNow == Duel::PLAYING && duel.nearCrate(me.p + V3{0, 0, 0.3}) >= 0) {
          char what[64];
          snprintf(msg, sizeof msg, "E: nehmen  ·  %s", Duel::crateLabel(duel.crates[size_t(duel.nearCrate(me.p + V3{0, 0, 0.3}))], what, sizeof what));
        } else if (phaseNow == Duel::PLAYING) {
          const int v = duel.nearestVehicle(me.p, 2.5);
          char tankName[64] = "";
          if (v >= 0 && v < Duel::kTanks) snprintf(tankName, sizeof tankName, "%s, %s", duel.tanks[v].hullSpec().name, duel.tanks[v].spec().name);
          const char* name = v < 0 ? "" : (v < Duel::kTanks ? tankName : (v - Duel::kTanks) % Duel::kCarKinds == 0 ? "Xantia" : ((v - Duel::kTanks) % Duel::kCarKinds == 1 ? "One:1" : "Hummer"));
          if (v >= 0) snprintf(msg, sizeof msg, duel.occupied(v) ? "E: als Richtschütze einsteigen  ·  %s" : (carrying[IT_REPAIR] > 0 ? "E: einsteigen  ·  Q: Reparaturset  ·  %s" : "E: einsteigen  ·  %s"), name);
        }
      }
      if (gPadHints.load()) {
        // The same words, with the buttons of the controller in them.
        static const char* const kSwap[][2] = {{"E: ", "X: "}, {"Q: ", "RB: "}, {"K zeigt", "LB zeigt"}, {"TAB wechselt", "X wechselt"}, {"[ ] Tempo", "LB RB Tempo"}, {"Maus lenkt", "Rechter Stick lenkt"}, {"Feuer ", "RT "}};
        std::string said = msg;
        for (const auto& swap : kSwap) {
          const size_t at = said.find(swap[0]);
          if (at != std::string::npos) said.replace(at, strlen(swap[0]), swap[1]);
        }
        snprintf(msg, sizeof msg, "%s", said.c_str());
      }
      memcpy(duel.players[mySlot].items, carrying, IT_COUNT);
      {
        std::lock_guard<std::mutex> lock(snapMutex);
        snap.foot.weapon = in.weapon;
        snap.foot.slot = mySlot;
        snap.foot.prone = me.prone;
        snap.foot.wind = duel.wind;
        const Duel::Player& who = duel.players[mySlot];
        snap.foot.hitsLanded = who.hitsLanded;
        snap.foot.hitsTaken = who.hitsTaken;
        snap.foot.hurtFrom = who.hurtFrom;
        snap.foot.ammo = in.weapon == WP_KNIFE ? -1 : (in.weapon == WP_SNIPER ? who.sniperAmmo : (in.weapon == WP_RPG ? who.rpgAmmo : who.smallAmmo));
        snap.foot.tapped = who.tapped;
        snap.foot.hasFix = who.hasFix;
        snap.foot.fix = who.fix;
        snap.foot.fixAge = who.fixAge;
        snap.foot.fixWait = who.fixWait;
        snap.foot.shells = who.tankShells;
        snap.foot.missiles = who.tankMissiles;
        snap.foot.codes = who.strikeCodes;
        snap.foot.has = 0;
        for (int k = 0; k < WP_COUNT; k++) snap.foot.has |= who.hasWeapon[k] ? 1 << k : 0;
        netState = state;
        netError = error;
        phase = phaseNow;
        phaseClock = clockNow;
        slot = mySlot;
        myVehicle = vehicleNow;
        playing = (state == NET_IN || replaying) && phaseNow == Duel::PLAYING;
        hasReplay = !lastTape.empty();
      }
      fillSnapshot(msg);
      if (state == NET_IN) {
        // The figures of the server: this machine only mirrors the world and has no physics of its own.
        snap.physLoad = serverStats[0];
        snap.realtime = serverStats[1];
        snap.awake = int(serverStats[2]);
        if (now >= lineNext) {
          const int due = lineGot + lineLost;
          lineLoss = due > 0 ? double(lineLost) / due : 0.0;
          lineKb = double(lineBytes) / 1024.0 / std::max(now - lineNext + 1.0, 0.25);
          lineGot = lineLost = 0;
          lineBytes = 0;
          lineNext = now + 1.0;
        }
        snap.netOn = true;
        snap.netPing = rtt;
        snap.netLoss = lineLoss;
        snap.netDelay = viewDelay;
        snap.netKb = lineKb;
      } else {
        snap.netOn = false;
      }
      {
        std::unique_lock<std::mutex> lock(frameMutex);
        frameAnswered = servedFrame;
        frameReady.notify_all();
        // Until the next picture wants it, or a few milliseconds for the network's sake.
        frameWanted.wait_for(lock, std::chrono::milliseconds(4), [&] { return frameAsked != frameAnswered; });
        servedFrame = frameAsked;
      }
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
  Renderer renderer;
  renderer.initUi();
  static const char* const kTips[] = {
      "Tipp: Schwachstellen wie Optik, Kühler und Ketten lassen sich mit dem Scharfschützengewehr treffen.",
      "Tipp: Wer beim Gegner den Hauptcomputer anzapft, kann seinen ungefähren Standort abfragen.",
      "Tipp: In Häusern und bei alten Wracks liegen Kisten mit Waffen, Munition und Bombencodes.",
      "Tipp: Im hohen Gras und hinter Büschen bist du schwer zu sehen, mit Ghillie-Anzug noch schwerer.",
      "Tipp: Sag deinem Team mit „Vektor C5“, wo du bist. Dein aktuelles Feld steht oben links.",
      "Tipp: Reparieren geht nur von außen und nur, wenn das Fahrzeug stillsteht.",
      "Tipp: Die Artillerie schießt über die Sperrmauern, das Ziel markierst du auf der Karte.",
      "Tipp: Das Scharfschützengewehr kennt Geschossabfall, Wind und Flugzeit. Stell das Visier auf die Entfernung ein.",
  };
  const auto tipNow = [] {
    const double t = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    return kTips[size_t(t / 6.0) % (sizeof kTips / sizeof kTips[0])];
  };
  bool closedWhileLoading = false;
  const auto showLoading = [&](const char* what, double progress) {
    // The window stays alive meanwhile: it can be moved, resized and closed.
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_EVENT_QUIT) closedWhileLoading = true;
    }
    int w = 0, h = 0;
    SDL_GetWindowSizeInPixels(window, &w, &h);
    if (w > 0 && h > 0) renderer.drawLoading(w, h, what, "", tipNow(), progress);
    SDL_GL_SwapWindow(window);
    if (scriptMode == 68) printf("Laden: %s, %.0f %%\n", what, progress * 100.0);
  };
  SDL_GL_SetSwapInterval(1);
  showLoading("Die Welt entsteht", 0.0);
  auto* game = new Game();
  Game& G = *game;
  printf("Baue Welt und Fahrzeuge...\n");
  {
    // In its own thread, for this is what takes the time; the bar goes on towards where it usually ends.
    std::atomic<bool> built{false};
    std::thread builder([&] {
      G.duel.build();
      built = true;
    });
    const auto began = std::chrono::steady_clock::now();
    while (!built.load()) {
      const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
      showLoading("Die Welt entsteht", 0.6 * (1.0 - std::exp(-t / 2.5)));
    }
    builder.join();
    if (scriptMode == 68) printf("Welt fertig nach %.1f s\n", std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count());
  }
  showLoading("Fahrzeuge werden geladen", 0.62);
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
    const int ofSide = k % (HK_COUNT * TK_COUNT);
    const std::string fileName = isTank ? tankMeshFile(HullKind(ofSide / TK_COUNT), TurretKind(ofSide % TK_COUNT)) : carFiles[(k - Duel::kTanks) % Duel::kCarKinds];
    const char* file = fileName.c_str();
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
    if (isTank) {
      std::vector<u32> net;
      for (u32 n = 0; n < 8; n++) net.push_back(G.duel.netFirst[k] + n);
      G.rig.groups.push_back({std::to_string(k) + ":net", net});
    }
    G.rig.links.insert(G.rig.links.end(), r.links.begin(), r.links.end());
    G.rig.linkInstance.resize(G.rig.links.size(), k);
    u32 first, end;
    if (isTank) {
      first = G.duel.tankFirst[k];
      end = first + G.duel.tankNodes;
    } else {
      first = G.simStart.objs[G.duel.carsStart[size_t(k - Duel::kTanks)].obj].nodeStart;
      end = k + 1 < Duel::kVehicles ? G.simStart.objs[G.duel.carsStart[size_t(k + 1 - Duel::kTanks)].obj].nodeStart : G.duel.carsEnd;
    }
    // A tank's mesh is skinned when anything of it moves, and its net is a node of its own.
    G.rig.parkedNodes.push_back({first, end - first});
    G.rig.parkedExtra.push_back(isTank ? CarRig::NodeRange{G.duel.netFirst[k], 8} : CarRig::NodeRange{0, 0});
  }
  // Two soldiers, each part held by its own few nodes.
  for (int i = 0; i < Duel::kMaxPlayers; i++) {
    CarMesh body;
    const char* file = Duel::teamOf(i) == 0 ? "assets/soldier.mesh" : "assets/soldier2.mesh";
    if (!body.load(file)) {
      fprintf(stderr, "%s fehlt. Starte das Spiel mit ./run.sh, das erzeugt die Modelle.\n", file);
      return;
    }
    const int k = Duel::kVehicles + i;
    G.mesh.addParked(body, k, Duel::kSoldierRest, 0.0, origin, {});
    const u32 first = G.duel.soldierFirst[i];
    for (int part = 0; part < Duel::kSoldierParts; part++) {
      std::vector<u32> nodes;
      for (int n = 0; n < Duel::kPartNodes; n++) nodes.push_back(first + u32(part * Duel::kPartNodes + n));
      G.rig.groups.push_back({std::to_string(k) + ":" + Duel::soldierPartName(part), nodes});
    }
    G.rig.parkedNodes.push_back({first, u32(Duel::kSoldierNodes)});
    G.rig.parkedExtra.push_back({0, 0});
  }
  // And the old wrecks, after them.
  for (int i = 0; i < Duel::kWrecks; i++) {
    CarMesh wreck;
    if (!wreck.load("assets/leopard.mesh")) return;
    const int k = Duel::kVehicles + Duel::kMaxPlayers + i;
    const CarRig& r = G.duel.wreckRigs[i];
    G.mesh.addParked(wreck, k, G.duel.wreckSpawn[i], r.noseX, origin, {});
    for (const NodeGroup& g : r.groups) G.rig.groups.push_back({std::to_string(k) + ":" + g.name, g.nodes});
    {
      std::vector<u32> net;
      for (u32 n = 0; n < 8; n++) net.push_back(G.duel.netFirst[Duel::kTanks + i] + n);
      G.rig.groups.push_back({std::to_string(k) + ":net", net});
    }
    G.rig.links.insert(G.rig.links.end(), r.links.begin(), r.links.end());
    G.rig.linkInstance.resize(G.rig.links.size(), k);
    G.rig.parkedNodes.push_back({G.duel.wreckFirst[i], G.duel.tankNodes});
    G.rig.parkedExtra.push_back({0, 0});
  }
  G.mesh.bind(G.simStart.rest, G.rig.groups, origin);
  // Blood on the ground: flat discs, each a little wider than the last.
  for (int i = 0; i < Duel::kMaxPlayers; i++)
    for (int d = 0; d < Duel::kBloodDiscs; d++) {
      const float seed = std::sin(float(d) * 12.9f + float(i)) * 0.5f + 0.5f;
      G.rig.links.push_back({G.duel.bloodFirst[i][d], G.duel.bloodFirst[i][d] + 1, (d < 3 ? 0.2f : 0.07f) + 0.1f * seed, MM_PAINT, 0});
    }
  // Drones: a little body, and a cross of arms with a rotor at each end.
  for (int i = 0; i < Duel::kMaxPlayers; i++) {
    const u32 f = G.duel.droneFirst[i];
    G.rig.links.push_back({f, f + 1, 0.11f, MM_DARK, 0});
    G.rig.links.push_back({f + 2, f + 3, 0.018f, MM_BLACK, 0});
    G.rig.links.push_back({f + 4, f + 5, 0.018f, MM_BLACK, 0});
  }
  // Rockets of every kind: body, nose and fins.
  const auto rocketRods = [&](u32 f, float body, u8 bodyMat, u8 noseMat) {
    G.rig.links.push_back({f, f + 1, body, bodyMat, 2});
    G.rig.links.push_back({f + 1, f + 2, body * 0.55f, noseMat, 2});
    G.rig.links.push_back({f + 3, f + 4, body * 0.2f, MM_DARK, 0});
    G.rig.links.push_back({f + 5, f + 6, body * 0.2f, MM_DARK, 0});
  };
  G.rig.rocketLinks = G.rig.links.size();
  for (int i = 0; i < Duel::kMissileTanks; i++) rocketRods(G.duel.missileFirst[i], 0.13f, MM_WHITE, MM_ACCENT);
  for (int i = 0; i < Duel::kRpgRockets; i++) rocketRods(G.duel.rocketFirst[i], 0.05f, MM_OLIVE2, MM_DARK);
  for (int i = 0; i < Duel::kStrikeRockets; i++) rocketRods(G.duel.rocketFirst[Duel::kRpgRockets + i], 0.24f, MM_METAL, MM_ACCENT);
  printf("%zu Nodes, %zu Beams, %zu Objekte\n", G.sim.pos.size(), G.sim.beams.size(), G.sim.objs.size());

  SDL_GL_SetSwapInterval(gSet.vsync ? 1 : 0);
  printf("OpenGL %s auf %s\n", glGetString(GL_VERSION), glGetString(GL_RENDERER));
  glEnable(GL_MULTISAMPLE);

  showLoading("Grafik wird vorbereitet", 0.85);
  // The renderer reads only topology from this immutable copy (rest pose, triangle list, beam end
  // points). Everything that moves arrives through snapshots, so it never races the physics.
  renderer.init(G.world, G.simStart, G.rig, G.mesh, origin);
  showLoading("Fertig", 1.0);

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
  enum { PANEL_NONE, PANEL_GAME, PANEL_MAIN, PANEL_KEYS, PANEL_ONLINE, PANEL_LOADOUT, PANEL_TOOLS, PANEL_CHAT };
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
  bool quit = closedWhileLoading;
  double newMatchUntil = 0;
  bool loadingShown = false;
  int shotCount = 0;
  bool wantShot = false;
  Snapshot view;
  // What this player is going to take into the match.
  u8 pickItems[IT_COUNT] = {};
  bool pickReady = false;
  int lastPhase = -1;
  bool chatting = false;
  std::string chatText;
  int weaponPick = WP_PISTOL;
  bool scriptAim = false;
  double hurtFlash = 0;
  bool sprintScript = false;
  // What a heavy shot does to whoever fires it, as far as the eye can tell.
  double gunKick = 0;
  int kickAmmo = -1, kickWeapon = -1;
  // In a replay: following one of the two, or flying about freely.
  bool replayFree = false;
  V3 freeEye;
  // Along the sights, with the right mouse button: every gun, on foot and in a tank. A short weapon just comes up to
  // the eye; the big rifle's scope is its own thing, once it is set up.
  double aimBlend = 0;
  // The big rifle's scope: right mouse button once it is set up.
  bool scoped = false;
  double scopeZoom = 6.0;
  int scopeZero = 100;  // the range the sight is set for, in metres
  // The map, and the point on it the artillery lays on.
  bool mapOpen = false;
  bool hasMark = false;
  V3 markPoint;
  std::vector<VehicleFrame> aimFrames;
  std::vector<Duel::AimFoe> aimFoes;
  double figureOff = 0;  // for a test: the drawn figure's bearing against the camera's, in degrees
  bool strikePending = false;  // a strike code chosen: the next click on the map says where
  // The controller's part of all that.
  bool padActive = false, padAim = false, padFire = false, padScopeHeld = false, padSprint = false;
  double padLx = 0, padLy = 0;
  V3 mapCursor;
  bool mapWasOpen = false;
  int navX = 0, navY = 0;
  double navWait = 0;
  // Typing without a keyboard.
  struct Osk {
    bool open = false, shift = false;
    int target = 0;  // the name, the address, the port or a line of chat
    int x = 0, y = 1;
    std::string text;
  } osk;
  static const char* const kOskKeys[2][4] = {{"1234567890", "qwertzuiop", "asdfghjkl-", "yxcvbnm._:"}, {"1234567890", "QWERTZUIOP", "ASDFGHJKL-", "YXCVBNM._:"}};
  const auto oskGroup = [](int x) { return x < 2 ? 0 : (x < 6 ? 1 : (x < 8 ? 2 : 3)); };
  const auto oskType = [&](char ch) {
    if (osk.target == 1 && !(std::isalnum(static_cast<unsigned char>(ch)) || ch == '.' || ch == '-')) return;
    if (osk.target == 2 && !std::isdigit(static_cast<unsigned char>(ch))) return;
    const size_t room = osk.target == 0 ? size_t(net::kNameBytes) - 1 : (osk.target == 1 ? 63 : (osk.target == 2 ? 5 : size_t(net::kChatBytes)));
    if (osk.text.size() < room) osk.text += ch;
  };
  const auto oskBack = [&] {
    while (osk.text.size() > 1 && (static_cast<unsigned char>(osk.text.back()) & 0xC0) == 0x80) osk.text.pop_back();
    if (!osk.text.empty()) osk.text.pop_back();
  };
  const auto oskDone = [&] {
    std::lock_guard<std::mutex> lock(G.inputMutex);
    if (osk.target == 0 && !osk.text.empty()) gUserName = osk.text;
    if (osk.target == 1 && !osk.text.empty()) gServerHost = osk.text;
    if (osk.target == 2 && !osk.text.empty()) gServerPort = std::min(atoi(osk.text.c_str()), 65535);
    if (osk.target == 3 && !osk.text.empty()) G.input.chatOut = osk.text;
    osk.open = false;
  };
  const auto oskPress = [&] {
    if (osk.y < 4) {
      oskType(kOskKeys[osk.shift ? 1 : 0][osk.y][osk.x]);
    } else {
      const int group = oskGroup(osk.x);
      if (group == 0) osk.shift = !osk.shift;
      else if (group == 1) oskType(' ');
      else if (group == 2) oskBack();
      else oskDone();
    }
  };
  std::vector<const char*> toolAbout;
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
    bool hasReplayNow = false;
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
      hasReplayNow = G.hasReplay;
    }
    const bool replayOn = netState == Game::NET_REPLAY;
    const bool inMatch = (netState == Game::NET_IN || replayOn) && phase == Duel::PLAYING;
    const bool inVehicle = inMatch && myVehicle >= 0, onFoot = inMatch && myVehicle < 0;
    const bool inTank = inVehicle && myVehicle < Duel::kTanks;
    // How far one can look: negative is up. A missile can be flown straight down, and the howitzer and someone on foot
    // can look up at what they are shooting at.
    const bool lookHigh = view.missileLive || onFoot || (inTank && myVehicle % TK_COUNT == TK_ARTILLERY);
    const double pitchUp = lookHigh ? -1.4 : -0.30, pitchDown = view.missileLive ? 1.5 : 1.2;
    rig.pitch = clampd(rig.pitch, pitchUp, pitchDown);
    // Nothing to do on foot before or after the match itself: one waits where one stands.
    if (!onFoot || weaponPick != WP_SNIPER || view.foot.prone < 0.99) scoped = false;
    // The choosing comes up by itself; afterwards it is gone by itself.
    if (phase != lastPhase) {
      if (phase == Duel::CHOOSING && netState == Game::NET_IN) {
        pickReady = false;
        panelKind = PANEL_LOADOUT;
        panelSel = 0;
        fromMain = false;
      } else if (panelKind == PANEL_LOADOUT || panelKind == PANEL_TOOLS || panelKind == PANEL_CHAT) {
        panelKind = PANEL_NONE;
      }
      strikePending = false;
      if (phase == Duel::PLAYING) {
        if (pickItems[IT_AR] > 0) weaponPick = WP_AR;
        else weaponPick = WP_PISTOL;
        // Looking across the yard towards the enemy. The camera's yaw is in the world.
        rig.yaw = G.world.base(Duel::teamOf(slot)).facing + PI;
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
      items.push_back({"Immer dabei", "Pistole, Messer", -1.f, [](int) {}});
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
    } else if (panelKind == PANEL_TOOLS) {
      // Only what is actually carried. What cannot be used from here says so, and stays where it is.
      toolAbout.clear();
      const auto tool = [&](const char* name, std::string value, const char* blocked, const char* about, std::function<void()> act) {
        toolAbout.push_back(about);
        if (blocked) value += std::string(value.empty() ? "" : "  ·  ") + blocked;
        char row[64];
        snprintf(row, sizeof row, "%d   %s", int(items.size()) + 1, name);
        items.push_back({row, value, -1.f, [&panelKind, blocked, act](int) {
                           if (blocked) return;
                           act();
                           if (panelKind == PANEL_TOOLS) panelKind = PANEL_NONE;
                         }});
      };
      const char* footOnly = onFoot ? nullptr : "nur zu Fuß";
      const auto& f = view.foot;
      if (f.items[IT_FOOT_MINE] > 0) tool("Tretmine", std::to_string(f.items[IT_FOOT_MINE]), footOnly, "Legst du dir vor die Füße. Nach 3 Sekunden scharf, wer drauftritt, stirbt meist.", [&] { edges.placeFoot = true; });
      if (f.items[IT_AT_MINE] > 0) tool("Panzermine", std::to_string(f.items[IT_AT_MINE]), footOnly, "Reißt einem Panzer die Kette ab und richtet schweren Schaden an. Zu Fuß nichts zu befürchten.", [&] { edges.placeTank = true; });
      if (f.medkits > 0) tool("Medikit", std::to_string(f.medkits), nullptr, "Gibt dir 50 Leben zurück. Bei vollen Leben bleibt es im Rucksack.", [&] { edges.medkit = true; });
      if (f.items[IT_REPAIR] > 0) tool("Reparaturset", std::to_string(f.items[IT_REPAIR]), footOnly, "Bringt ein Fahrzeug wieder zum Laufen: 6 Sekunden, zu Fuß daneben, und es muss stillstehen.", [&] { edges.repair = true; });
      if (inTank) tool("Nebelwerfer", std::to_string(f.launcher), f.launcher > 0 ? nullptr : "leer", "Der Werfer auf dem Turm: eine Wolke vor den Panzer, die eine halbe Minute lang die Sicht versperrt.", [&] { edges.smoke = true; });
      else if (f.items[IT_SMOKE] > 0) tool("Nebelgranate", std::to_string(f.items[IT_SMOKE]), onFoot ? nullptr : "nur zu Fuß", "Eine Wolke, die eine halbe Minute lang die Sicht versperrt, gut gegen Scharfschützen.", [&] { edges.smoke = true; });
      if (f.items[IT_NET] > 0) tool("Tarnnetz", std::to_string(f.items[IT_NET]), inTank ? nullptr : "nur im Panzer", "Über den stehenden Panzer gezogen, dauert 3 Sekunden. Beim Losfahren bleibt es zurück.", [&] { edges.net = true; });
      if (f.droneLive) tool("Drohne zurückholen", "", nullptr, "Holt die Drohne aus der Luft. Sie ist danach verbraucht.", [&] { edges.drone = true; });
      else if (f.drones > 0) tool("Drohne", std::to_string(f.drones), footOnly, "Steigt von deinem Standort auf. Du fliegst sie selbst, Feuer markiert den Gegner.", [&] { edges.drone = true; });
      if (f.codes > 0) tool("Bombencode", std::to_string(f.codes), nullptr, "Die Karte geht auf, dort wählst du das Ziel: vier Raketen, ziemlich ungenau.", [&] {
        strikePending = true;
        mapOpen = true;
      });
      if (f.tapped) {
        char wait[32] = "bereit";
        if (f.fixWait > 0) snprintf(wait, sizeof wait, "noch %.0f s", f.fixWait);
        tool("Standort abfragen", wait, f.fixWait > 0 ? "" : nullptr, "Fragt den angezapften Computer. Der ungefähre Standort des Gegners erscheint auf der Karte.", [&] { edges.query = true; });
      }
      if (items.empty()) {
        toolAbout.push_back("Finden kannst du Werkzeug in Kisten in Häusern und bei Wracks.");
        items.push_back({"Nichts dabei", "", -1.f, [](int) {}});
      }
    } else if (panelKind == PANEL_MAIN) {
      if (netState == Game::NET_REPLAY) {
        page("Weiter ansehen", [&] { panelKind = PANEL_NONE; });
        page("Replay beenden", [&] { leave(); });
      } else if (netState == Game::NET_IN) {
        page("Weiterspielen", [&] { panelKind = PANEL_NONE; });
        if (phase == Duel::CHOOSING) page("Ausrüstung wählen", [&] { panelKind = PANEL_LOADOUT; panelSel = 0; pickReady = false; });
        page("Spiel verlassen", [&] { leave(); });
      } else {
        page("Online spielen", [&] { openPage(PANEL_ONLINE); });
        if (hasReplayNow) page("Letztes Match ansehen", [&] {
          replayFree = false;
          G.replayWatch = 0;
          G.command = Game::CMD_REPLAY;
          panelKind = PANEL_NONE;
        });
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
      percent("Grasdichte", gSet.grass, 0.0, 1.0, 0.1);
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
      percent("Controller-Empfindlichkeit", gSet.padSens, 0.3, 2.5, 0.1);
      toggle("Vibration", gSet.rumble, [&] { gSet.rumble = !gSet.rumble; });
    } else if (panelKind == PANEL_KEYS) {
      // The list of whatever is in the hands right now.
      auto key = [&](const char* what, const char* k, const char* p) { items.push_back({what, padActive ? p : k, -1.f, [&](int) {}}); };
      key("Gehen, Lenken", "W A S D", "Linker Stick");
      key("Panzer: Gas und Rückwärts", "W S", "Linker Stick");
      key("Auto: Gas und Bremse", "W S", "RT, LT");
      key("Bremse, zu Fuß springen", "Leertaste", "A");
      key("Rennen", "Umschalt", "Linken Stick drücken");
      key("Zielen, umsehen", "Maus", "Rechter Stick");
      key("Feuer", "Linke Maustaste, F", "RT");
      key("Anvisieren, in jedem Panzer und mit jedem Gewehr", "Rechte Maustaste halten", "LT halten");
      key("Einsteigen, aussteigen, nehmen, anzapfen", "E", "X");
      key("Waffe wählen", "1 2 3 4 5", "Steuerkreuz links, rechts");
      key("Werkzeuge: Minen, Medikit, Nebel, Drohne, Bombencode", "Q", "RB");
      key("Karte, Ziel für die Artillerie", "K", "LB");
      key("Rakete starten, fliegt mit dem Blick", "G", "B");
      key("Scharfschützengewehr aufbauen", "X", "B");
      key("Zielfernrohr, Zoom", "Rechte Maustaste, Mausrad", "LT, Steuerkreuz links, rechts");
      key("Visier auf Entfernung einstellen", "Pfeil hoch, runter", "Steuerkreuz hoch, runter");
      key("Kamera, Abstand", "C, Mausrad", "Y, Steuerkreuz hoch, runter");
      key("Chat", "Enter", "Back");
      key("Menü", "ESC", "Start");
      if (!padActive) key("Screenshot", "F12", "");
    } else if (panelKind == PANEL_CHAT) {
      page("Eigener Text...", [&] {
        panelKind = PANEL_NONE;
        osk = Osk{};
        osk.open = true;
        osk.target = 3;
      });
      {
        char name[4], line[40];
        vectorName(view.carCog.x, view.carCog.y, name);
        snprintf(line, sizeof line, "Ich bin in Vektor %s", name);
        const std::string said = line;
        page(line, [&, said] {
          std::lock_guard<std::mutex> lock(G.inputMutex);
          G.input.chatOut = said;
          panelKind = PANEL_NONE;
        });
      }
      for (const char* line : {"Hilfe!", "Gegner gesichtet", "Ich brauche Munition", "Steig ein, ich fahre", "Folge mir", "Bleib, wo du bist", "Danke", "Gut gemacht", "Entschuldigung"}) {
        page(line, [&, line] {
          std::lock_guard<std::mutex> lock(G.inputMutex);
          G.input.chatOut = line;
          panelKind = PANEL_NONE;
        });
      }
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
    const bool wantText = chatting || (panelKind == PANEL_ONLINE && panelSel < 3);
    if (wantText != textEntry) {
      textEntry = wantText;
      if (wantText) SDL_StartTextInput(window);
      else SDL_StopTextInput(window);
    }

    SDL_Event e;
    const auto leaveLoading = [&] {
      G.command = Game::CMD_LEAVE;
      if (panelKind == PANEL_NONE) panelKind = PANEL_MAIN;
      panelSel = 0;
    };
    const auto openMenu = [&] {
      if (mapOpen) {
        mapOpen = false;
        strikePending = false;
      } else {
        panelKind = PANEL_MAIN;
        panelSel = 0;
        fromMain = false;
      }
    };
    const auto replayNext = [&] {
      // One, the other, then free.
      if (replayFree) {
        replayFree = false;
        G.replayWatch = 0;
      } else if (G.replayWatch.load() + 1 < G.duel.playersWanted) {
        G.replayWatch = G.replayWatch.load() + 1;
      } else {
        replayFree = true;
        freeEye = cam.eye;
      }
    };
    const auto openTools = [&] {
      if (netState == Game::NET_IN && phase == Duel::PLAYING) {
        panelKind = PANEL_TOOLS;
        panelSel = 0;
        fromMain = false;
        mapOpen = false;
        strikePending = false;
      }
    };
    const auto pickOnMap = [&](double wx, double wy) {
      if (std::fabs(wx) >= kMapHalf || std::fabs(wy) >= kMapHalf) return;
      if (strikePending) {
        edges.strike = true;
        edges.strikeAt = V3{wx, wy, G.world.height(wx, wy)};
        strikePending = false;
        mapOpen = false;
      } else {
        hasMark = true;
        markPoint = V3{wx, wy, G.world.height(wx, wy)};
      }
    };
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_EVENT_KEY_DOWN || e.type == SDL_EVENT_MOUSE_BUTTON_DOWN || e.type == SDL_EVENT_MOUSE_WHEEL) padActive = false;
      if (e.type == SDL_EVENT_GAMEPAD_BUTTON_DOWN || (e.type == SDL_EVENT_GAMEPAD_AXIS_MOTION && std::abs(int(e.gaxis.value)) > 12000)) padActive = true;
      if (loadingShown && netState != Game::NET_OFF && ((e.type == SDL_EVENT_KEY_DOWN && e.key.key == SDLK_ESCAPE) || (e.type == SDL_EVENT_GAMEPAD_BUTTON_DOWN && e.gbutton.button == SDL_GAMEPAD_BUTTON_EAST))) {
        // Waiting is the only thing to do on this screen, so that is what the key ends.
        leaveLoading();
        continue;
      }
      if (osk.open) {
        // The keyboard on the screen has the buttons to itself. A real one can still leave it.
        if (e.type == SDL_EVENT_GAMEPAD_BUTTON_DOWN) {
          switch (e.gbutton.button) {
            case SDL_GAMEPAD_BUTTON_SOUTH: oskPress(); break;
            case SDL_GAMEPAD_BUTTON_EAST: osk.open = false; break;
            case SDL_GAMEPAD_BUTTON_WEST: oskBack(); break;
            case SDL_GAMEPAD_BUTTON_NORTH: osk.shift = !osk.shift; break;
            case SDL_GAMEPAD_BUTTON_START: oskDone(); break;
            default: break;
          }
          continue;
        }
        if (e.type == SDL_EVENT_KEY_DOWN) {
          if (e.key.key == SDLK_ESCAPE) osk.open = false;
          continue;
        }
        if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN) continue;
      }
      if (chatting && e.type == SDL_EVENT_TEXT_INPUT) {
        for (const char* t = e.text.text; *t; t++)
          if (chatText.size() < size_t(net::kChatBytes)) chatText += *t;
        continue;
      }
      if (chatting && e.type == SDL_EVENT_KEY_DOWN) {
        const SDL_Keycode k = e.key.key;
        if (k == SDLK_RETURN || k == SDLK_KP_ENTER) {
          if (!chatText.empty()) {
            std::lock_guard<std::mutex> lock(G.inputMutex);
            G.input.chatOut = chatText;
          }
          chatText.clear();
          chatting = false;
        } else if (k == SDLK_ESCAPE) {
          chatText.clear();
          chatting = false;
        } else if (k == SDLK_BACKSPACE && !chatText.empty()) {
          while (chatText.size() > 1 && (static_cast<unsigned char>(chatText.back()) & 0xC0) == 0x80) chatText.pop_back();
          chatText.pop_back();
        }
        continue;
      }
      if (chatting && e.type == SDL_EVENT_MOUSE_BUTTON_DOWN) continue;
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
        if (e.button.button == SDL_BUTTON_LEFT) pickOnMap(wx, wy);
        if (e.button.button == SDL_BUTTON_RIGHT) {
          // The other button takes the mark away, or keeps the code that was about to be used.
          if (strikePending) {
            strikePending = false;
            mapOpen = false;
          } else {
            hasMark = false;
          }
        }
        continue;
      }
      // The same map with the controller: the right stick moves a cursor, so the left one can go on driving.
      if (mapOpen && panelKind == 0 && e.type == SDL_EVENT_GAMEPAD_BUTTON_DOWN) {
        bool taken = true;
        switch (e.gbutton.button) {
          case SDL_GAMEPAD_BUTTON_SOUTH: pickOnMap(mapCursor.x, mapCursor.y); break;
          case SDL_GAMEPAD_BUTTON_WEST: if (!strikePending) hasMark = false; break;
          case SDL_GAMEPAD_BUTTON_EAST: case SDL_GAMEPAD_BUTTON_LEFT_SHOULDER:
            mapOpen = false;
            strikePending = false;
            break;
          default: taken = false; break;
        }
        if (taken) continue;
      }
      // A panel on screen takes the keys and buttons it needs first.
      if (panelKind != 0) {
        bool used = true;
        if (e.type == SDL_EVENT_KEY_DOWN && panelKind == PANEL_TOOLS && !e.key.repeat && e.key.key >= SDLK_1 && e.key.key <= SDLK_9) {
          // A number takes the row of that number at once.
          const int row = int(e.key.key - SDLK_1);
          if (row < int(items.size())) {
            panelSel = row;
            choosePanel();
          }
        } else if (e.type == SDL_EVENT_KEY_DOWN && panelKind == PANEL_TOOLS && e.key.key == SDLK_Q) {
          if (!e.key.repeat) panelKind = PANEL_NONE;
        } else if (e.type == SDL_EVENT_KEY_DOWN) {
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
          // The directions come by the frame, further down, so that holding one keeps going.
          switch (e.gbutton.button) {
            case SDL_GAMEPAD_BUTTON_SOUTH:
              if (panelKind == PANEL_ONLINE && panelSel < 3) {
                osk = Osk{};
                osk.open = true;
                osk.target = panelSel;
                osk.text = panelSel == 0 ? gUserName : (panelSel == 1 ? gServerHost : std::to_string(gServerPort));
              } else {
                choosePanel();
              }
              break;
            case SDL_GAMEPAD_BUTTON_WEST: if (panelKind == PANEL_LOADOUT) panelAdjust(-1); break;
            case SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER:
              if (panelKind == PANEL_TOOLS) panelKind = PANEL_NONE;
              break;
            case SDL_GAMEPAD_BUTTON_BACK:
              if (panelKind == PANEL_CHAT) panelKind = PANEL_NONE;
              break;
            case SDL_GAMEPAD_BUTTON_EAST: case SDL_GAMEPAD_BUTTON_GUIDE: case SDL_GAMEPAD_BUTTON_START: closePanel(); break;
            default: break;
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
            if ((panelKind == PANEL_GAME || panelKind == PANEL_LOADOUT) && (panelKind != PANEL_LOADOUT || row + 1 < int(items.size()))) panelAdjust(mxp < px + pwid * 0.65f ? -1 : 1);
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
            case SDL_GAMEPAD_BUTTON_NORTH: rig.mode = CamMode((rig.mode + 1) % CAM_COUNT); rig.pitch = 0.22; break;
            case SDL_GAMEPAD_BUTTON_START: case SDL_GAMEPAD_BUTTON_GUIDE: openMenu(); break;
            case SDL_GAMEPAD_BUTTON_WEST:
              if (replayOn) replayNext();
              else edges.use = true;
              break;
            case SDL_GAMEPAD_BUTTON_EAST:
              // In a vehicle the missile, on foot getting down behind the big rifle.
              if (inVehicle) edges.launch = true;
              else edges.setup = true;
              break;
            case SDL_GAMEPAD_BUTTON_LEFT_SHOULDER:
              if (replayOn) G.replaySpeed = std::max(25, G.replaySpeed.load() / 2);
              else mapOpen = !mapOpen, strikePending = false;
              break;
            case SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER:
              if (replayOn) G.replaySpeed = std::min(800, G.replaySpeed.load() * 2);
              else openTools();
              break;
            case SDL_GAMEPAD_BUTTON_BACK:
              if (netState == Game::NET_IN) {
                panelKind = PANEL_CHAT;
                panelSel = 0;
                fromMain = false;
              }
              break;
            case SDL_GAMEPAD_BUTTON_LEFT_STICK: padSprint = true; break;
            case SDL_GAMEPAD_BUTTON_DPAD_UP:
              if (scoped) scopeZero = std::min(scopeZero + 50, 2000);
              else rig.dist = clampd(rig.dist * 0.85, 2.5, 40.0);
              break;
            case SDL_GAMEPAD_BUTTON_DPAD_DOWN:
              if (scoped) scopeZero = std::max(scopeZero - 50, 100);
              else rig.dist = clampd(rig.dist / 0.85, 2.5, 40.0);
              break;
            case SDL_GAMEPAD_BUTTON_DPAD_LEFT: case SDL_GAMEPAD_BUTTON_DPAD_RIGHT:
              // Through what is carried; through the sights the same two buttons zoom instead.
              if (!scoped) {
                const int d = e.gbutton.button == SDL_GAMEPAD_BUTTON_DPAD_RIGHT ? 1 : 4;
                for (int step = 1, pick = weaponPick; step < 5; step++) {
                  pick = (pick + d) % 5;
                  if (view.foot.has & (1 << pick)) {
                    weaponPick = pick;
                    break;
                  }
                }
              }
              break;
            default: break;
          }
          break;
        case SDL_EVENT_KEY_DOWN:
          if (e.key.repeat) break;
          switch (e.key.key) {
            case SDLK_TAB:
              if (replayOn) replayNext();
              break;
            case SDLK_LEFTBRACKET: if (replayOn) G.replaySpeed = std::max(25, G.replaySpeed.load() / 2); break;
            case SDLK_RIGHTBRACKET: if (replayOn) G.replaySpeed = std::min(800, G.replaySpeed.load() * 2); break;
            case SDLK_ESCAPE: openMenu(); break;
            case SDLK_Q: openTools(); break;
            case SDLK_G: edges.launch = true; break;
            case SDLK_K:
              mapOpen = !mapOpen;
              strikePending = false;
              break;
            case SDLK_C: rig.mode = CamMode((rig.mode + 1) % CAM_COUNT); rig.pitch = 0.22; break;
            case SDLK_1: case SDLK_2: case SDLK_3: case SDLK_4: case SDLK_5: {
              const int pick = int(e.key.key - SDLK_1);
              if (view.foot.has & (1 << pick)) weaponPick = pick;
              break;
            }
            case SDLK_RETURN: case SDLK_KP_ENTER:
              if (netState == Game::NET_IN) chatting = true;
              break;
            case SDLK_F: edges.fire = true; break;
            case SDLK_E: edges.use = true; break;
            case SDLK_X: edges.setup = true; break;
            case SDLK_UP: if (scoped) scopeZero = std::min(scopeZero + 50, 2000); break;
            case SDLK_DOWN: if (scoped) scopeZero = std::max(scopeZero - 50, 100); break;
            case SDLK_F12: wantShot = true; break;
            default: break;
          }
          break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
          if (e.button.button == SDL_BUTTON_LEFT) edges.fire = true;
          if (e.button.button == SDL_BUTTON_RIGHT && onFoot && weaponPick == WP_SNIPER && view.foot.prone > 0.99) scoped = !scoped;
          break;
        case SDL_EVENT_MOUSE_MOTION: {
          const double sens = (rig.mode == CAM_HOOD ? 0.0009 : 0.0028) * gSet.mouse / (scoped ? scopeZoom * 0.5 : 1.0) * lerpd(1.0, inTank ? 0.3 : 0.55, aimBlend);
          if (!mapOpen) {
            rig.yaw -= e.motion.xrel * sens;
            rig.pitch = clampd(rig.pitch + e.motion.yrel * sens, pitchUp, pitchDown);
          }
          break;
        }
        case SDL_EVENT_MOUSE_WHEEL:
          if (scoped) scopeZoom = clampd(scopeZoom * std::pow(1.15, double(e.wheel.y)), 6.0, 24.0);
          else rig.dist = clampd(rig.dist * std::pow(0.9, double(e.wheel.y)), 2.5, 40.0);
          break;
        default:
          break;
      }
    }

    // Holding the right button brings the sights up, as far as there are any for what is in hand.
    {
      const bool held = panelKind == 0 && !mapOpen && !chatting && (padAim || (SDL_GetMouseState(nullptr, nullptr) & SDL_BUTTON_RMASK));
      const bool canAim = (onFoot && weaponPick != WP_KNIFE && weaponPick != WP_SNIPER) || (inTank && !view.foot.noSight && !view.missileLive);
      const double want = (held || scriptAim) && canAim ? 1.0 : 0.0;
      aimBlend += clampd(want - aimBlend, -dt * 7.0, dt * 7.0);
    }

    // Continuous inputs.
    const bool* keys = SDL_GetKeyboardState(nullptr);
    double keyDir = 0;
    if (keys[SDL_SCANCODE_A] || keys[SDL_SCANCODE_LEFT]) keyDir += 1;
    if (keys[SDL_SCANCODE_D] || keys[SDL_SCANCODE_RIGHT]) keyDir -= 1;
    double gas = (keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_UP]) ? 1.0 : 0.0;
    double brake = (keys[SDL_SCANCODE_S] || keys[SDL_SCANCODE_DOWN]) ? 1.0 : 0.0;
    // In a car keyboard steering goes like a driver's hands, and narrower with speed: at 100 km/h a key press must not
    // slam on more lock than the tyres can use. A tank just turns, and someone on foot just steps aside.
    {
      const double speedAbs = std::fabs(view.speedKmh) / 3.6;
      const double keyTarget = keyDir * clampd(5.0 / std::max(speedAbs, 1.0), 0.09, 1.0);
      const double keyRate = (keyDir != 0 && keyDir * keySteer >= 0) ? 2.4 : 4.0;
      keySteer += clampd(keyTarget - keySteer, -keyRate * dt, keyRate * dt);
    }
    bool sprint = keys[SDL_SCANCODE_LSHIFT] || sprintScript;
    double steer = inVehicle && myVehicle >= Duel::kTanks ? keySteer : keyDir, hb = keys[SDL_SCANCODE_SPACE] ? 1.0 : 0.0;

    hud.gamepad = false;
    padFire = false;
    if (gamepad) {
      const auto axis = [&](SDL_GamepadAxis ax) { return SDL_GetGamepadAxis(gamepad, ax) / 32767.0; };
      const auto button = [&](SDL_GamepadButton b) { return SDL_GetGamepadButton(gamepad, b); };
      const double rawX = axis(SDL_GAMEPAD_AXIS_LEFTX), rawY = axis(SDL_GAMEPAD_AXIS_LEFTY);
      const double rt = axis(SDL_GAMEPAD_AXIS_RIGHT_TRIGGER), lt = axis(SDL_GAMEPAD_AXIS_LEFT_TRIGGER);
      const double rx = applyDeadzone(axis(SDL_GAMEPAD_AXIS_RIGHTX), 0.12), ry = applyDeadzone(axis(SDL_GAMEPAD_AXIS_RIGHTY), 0.12);
      const double lx = applyDeadzone(rawX, 0.12), ly = applyDeadzone(rawY, 0.12);
      padLx = lx;
      padLy = ly;
      if (std::fabs(lx) > 0 || std::fabs(ly) > 0 || std::fabs(rx) > 0 || std::fabs(ry) > 0 || rt > 0.02 || lt > 0.02) hud.gamepad = true;

      // Lists and the keyboard on the screen: the cross or the stick, and holding either keeps going.
      const int wantX = button(SDL_GAMEPAD_BUTTON_DPAD_RIGHT) || rawX > 0.6 ? 1 : (button(SDL_GAMEPAD_BUTTON_DPAD_LEFT) || rawX < -0.6 ? -1 : 0);
      const int wantY = button(SDL_GAMEPAD_BUTTON_DPAD_DOWN) || rawY > 0.6 ? 1 : (button(SDL_GAMEPAD_BUTTON_DPAD_UP) || rawY < -0.6 ? -1 : 0);
      navWait -= dt;
      const bool navChanged = wantX != navX || wantY != navY;
      if (navChanged || (navWait <= 0 && (wantX != 0 || wantY != 0))) {
        navWait = navChanged ? 0.35 : 0.11;
        navX = wantX;
        navY = wantY;
        if (osk.open) {
          if (wantY != 0) osk.y = (osk.y + wantY + 5) % 5;
          if (wantX != 0) {
            if (osk.y < 4) {
              osk.x = (osk.x + wantX + 10) % 10;
            } else {
              // Along the bottom row the wide keys count once.
              const int group = oskGroup(osk.x);
              while (oskGroup(osk.x) == group) osk.x = (osk.x + wantX + 10) % 10;
            }
          }
        } else if (panelKind != 0) {
          if (wantY != 0) panelMove(wantY);
          if (wantX != 0) panelAdjust(wantX);
        }
      }

      // Everyone has the same hands: left stick moves, right stick looks, right trigger fires, left trigger aims.
      // Only a car has pedals, for it has nothing to shoot with.
      const bool inCar = inVehicle && !inTank;
      const bool free = panelKind == 0 && !chatting && !osk.open;
      const double curved = std::copysign(std::pow(std::fabs(lx), 1.35), -lx);
      if (std::fabs(lx) > 0) steer = curved;
      if (inCar) {
        gas = std::max(gas, rt);
        brake = std::max(brake, lt);
      } else {
        gas = std::max(gas, -ly);
        brake = std::max(brake, ly);
      }
      if (button(SDL_GAMEPAD_BUTTON_SOUTH) && !mapOpen) hb = 1.0;
      if (padSprint && std::fabs(lx) < 0.2 && std::fabs(ly) < 0.2) padSprint = false;
      if (padSprint && onFoot) sprint = true;

      if (free && !inCar && !replayOn) {
        const bool fireNow = rt > 0.5;
        static bool fireWas = false;
        if (fireNow && !fireWas) edges.fire = true;
        fireWas = fireNow;
        padFire = fireNow;
        const bool aimNow = lt > 0.4;
        const bool scopeWeapon = onFoot && weaponPick == WP_SNIPER;
        if (aimNow && !padScopeHeld && scopeWeapon && view.foot.prone > 0.99) scoped = !scoped;
        padScopeHeld = aimNow;
        padAim = aimNow && !scopeWeapon;
      } else {
        padAim = false;
      }

      if (mapOpen && panelKind == 0) {
        mapCursor.x = clampd(mapCursor.x + rx * std::fabs(rx) * dt * 520.0, -kMapHalf + 1.0, kMapHalf - 1.0);
        mapCursor.y = clampd(mapCursor.y - ry * std::fabs(ry) * dt * 520.0, -kMapHalf + 1.0, kMapHalf - 1.0);
      } else if (free || !inMatch) {
        // As gentle near the middle as a mouse: the first bit of the stick is for aiming, the rest for turning.
        const double slow = (rig.mode == CAM_HOOD ? 0.5 : 1.0) * gSet.padSens / (scoped ? scopeZoom * 0.5 : 1.0) * lerpd(1.0, inTank ? 0.3 : 0.55, aimBlend);
        rig.yaw -= std::copysign(std::pow(std::fabs(rx), 1.7), rx) * dt * 2.6 * slow;
        rig.pitch = clampd(rig.pitch + std::copysign(std::pow(std::fabs(ry), 1.7), ry) * dt * 1.6 * slow, pitchUp, pitchDown);
      }
      if (scoped) {
        const int zoom = (button(SDL_GAMEPAD_BUTTON_DPAD_RIGHT) ? 1 : 0) - (button(SDL_GAMEPAD_BUTTON_DPAD_LEFT) ? 1 : 0);
        if (zoom != 0) scopeZoom = clampd(scopeZoom * std::pow(1.9, zoom * dt), 6.0, 24.0);
      }
    } else {
      padAim = false;
      padLx = padLy = 0;
    }
    gPadHints = padActive;
    if (mapOpen && !mapWasOpen) mapCursor = hasMark ? markPoint : view.carCog;
    mapWasOpen = mapOpen;

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
    if (panelKind != 0 || chatting || osk.open) gas = brake = steer = hb = 0;
    {
      const bool wantRelative = panelKind == 0 && !mapOpen && !osk.open && !loadingShown && scriptMode == 0;
      if (wantRelative != relativeMouse) {
        SDL_SetWindowRelativeMouseMode(window, wantRelative);
        relativeMouse = wantRelative;
      }
    }
    // With no match to look at, the camera goes slowly round the yard.
    if (!inMatch && panelKind != 0) rig.yaw += dt * 0.12;

    // Scripted runs, for trying out the real thing without anyone at the controls.
    if (scriptMode == 46 && netState == Game::NET_IN) {
      // The big rifle: down behind it, then the scope at both ends of its zoom.
      if (phase == Duel::CHOOSING && !pickReady) {
        pickReady = true;
        panelKind = PANEL_NONE;
      }
      if (inMatch) {
        panelKind = PANEL_NONE;
        weaponPick = WP_SNIPER;
        rig.yaw = G.world.base(Duel::teamOf(slot)).facing + PI;
        if (!scoped) rig.pitch = 0.03;
        static double since = tNow;
        static bool down = false;
        if (tNow - since > 1.0 && !down) {
          down = true;
          sendC.setup = true;
        }
        if (view.foot.prone > 0.99) scoped = true;
        scopeZoom = 6.0;
        if (std::fmod(tNow, 2.5) < dt) sendC.fire = true;
      }
    }
    if ((scriptMode == 61 || scriptMode == 62) && gTestPad) {
      // A whole game with nothing but the controller, through SDL's own gamepad path.
      static double wake = 2.0;
      static int stage = 0;
      static SDL_GamepadButton held = SDL_GAMEPAD_BUTTON_INVALID;
      if (held != SDL_GAMEPAD_BUTTON_INVALID) {
        SDL_SetJoystickVirtualButton(gTestPad, held, false);
        held = SDL_GAMEPAD_BUTTON_INVALID;
      }
      const auto tap = [&](SDL_GamepadButton b) {
        SDL_SetJoystickVirtualButton(gTestPad, b, true);
        held = b;
      };
      const auto stick = [&](SDL_GamepadAxis a, double v) { SDL_SetJoystickVirtualAxis(gTestPad, a, Sint16(v * 32767.0)); };
      const auto trigger = [&](SDL_GamepadAxis a, bool on) { SDL_SetJoystickVirtualAxis(gTestPad, a, on ? 32767 : -32768); };
      const auto say = [&](const char* what) {
        printf("%s: Panel %d Zeile %d/%zu Karte %d Marke %d Waffe %d Munition %d Fahrzeug %d Pos %.1f %.1f km/h %.1f Blick %.2f Anvisieren %.2f Scope %d Zoom %.1f Visier %d Minen %d Codes %d OSK %d '%s' Name %s\n", what, panelKind, panelSel, items.size(), int(mapOpen), int(hasMark), weaponPick, view.foot.ammo, myVehicle, view.foot.p.x, view.foot.p.y, view.speedKmh, rig.yaw, aimBlend, int(scoped), scopeZoom, scopeZero, view.foot.items[IT_FOOT_MINE], view.foot.codes, int(osk.open), osk.text.c_str(), gUserName.c_str());
      };
      if (scriptMode == 62 && tNow > wake) {
        wake = tNow + 0.5;
        switch (stage++) {
          case 0: say("Hauptmenü"); wantShot = true; break;
          case 1: tap(SDL_GAMEPAD_BUTTON_SOUTH); break;
          case 2: say("Online"); tap(SDL_GAMEPAD_BUTTON_SOUTH); break;
          case 3: say("Tastatur auf"); tap(SDL_GAMEPAD_BUTTON_SOUTH); break;
          case 4: say("q getippt"); tap(SDL_GAMEPAD_BUTTON_DPAD_RIGHT); break;
          case 5: tap(SDL_GAMEPAD_BUTTON_NORTH); break;
          case 6: tap(SDL_GAMEPAD_BUTTON_SOUTH); wantShot = true; break;
          case 7: say("W groß"); tap(SDL_GAMEPAD_BUTTON_WEST); break;
          case 8: say("gelöscht"); tap(SDL_GAMEPAD_BUTTON_SOUTH); break;
          case 9: tap(SDL_GAMEPAD_BUTTON_START); break;
          case 10: say("fertig"); tap(SDL_GAMEPAD_BUTTON_SOUTH); break;
          case 11: say("nochmal auf"); tap(SDL_GAMEPAD_BUTTON_SOUTH); break;
          case 12: tap(SDL_GAMEPAD_BUTTON_EAST); break;
          case 13: say("abgebrochen"); tap(SDL_GAMEPAD_BUTTON_EAST); break;
          case 14: say("zurück im Menü"); stick(SDL_GAMEPAD_AXIS_LEFTY, 1.0); break;
          case 15: stick(SDL_GAMEPAD_AXIS_LEFTY, 0.0); say("Stick runter"); break;
          case 16: panelSel = 2; tap(SDL_GAMEPAD_BUTTON_SOUTH); break;
          case 17: say("Steuerung"); wantShot = true; break;
          case 18: quit = true; break;
          default: break;
        }
      }
      if (scriptMode == 61 && netState == Game::NET_IN && tNow > wake) {
        static double hold = 0;
        wake = tNow + 0.45;
        if (phase == Duel::CHOOSING) {
          static int pick = 0;
          switch (pick++) {
            case 0: say("Ausrüstung"); tap(SDL_GAMEPAD_BUTTON_DPAD_DOWN); break;
            case 1: tap(SDL_GAMEPAD_BUTTON_SOUTH); break;
            case 2: printf("Tretminen gewählt %d\n", pickItems[IT_FOOT_MINE]); tap(SDL_GAMEPAD_BUTTON_SOUTH); break;
            case 3: printf("Tretminen gewählt %d\n", pickItems[IT_FOOT_MINE]); tap(SDL_GAMEPAD_BUTTON_WEST); break;
            case 4: printf("Tretminen gewählt %d\n", pickItems[IT_FOOT_MINE]); stick(SDL_GAMEPAD_AXIS_LEFTY, 1.0); wake = tNow + 0.7; break;
            case 5: stick(SDL_GAMEPAD_AXIS_LEFTY, 0.0); say("Stick gehalten"); panelSel = int(items.size()) - 1; break;
            case 6: tap(SDL_GAMEPAD_BUTTON_SOUTH); break;
            default: break;
          }
        } else if (inMatch) {
          const auto press = [&](const char* what, SDL_GamepadButton b) { say(what); tap(b); };
          switch (stage++) {
            case 0: say("Match"); trigger(SDL_GAMEPAD_AXIS_RIGHT_TRIGGER, false); trigger(SDL_GAMEPAD_AXIS_LEFT_TRIGGER, false); stick(SDL_GAMEPAD_AXIS_LEFTY, -1.0); wake = tNow + 1.2; break;
            case 1: say("gelaufen"); stick(SDL_GAMEPAD_AXIS_LEFTY, 0); stick(SDL_GAMEPAD_AXIS_RIGHTX, 1.0); wake = tNow + 0.7; break;
            case 2: say("umgesehen"); stick(SDL_GAMEPAD_AXIS_RIGHTX, 0); trigger(SDL_GAMEPAD_AXIS_RIGHT_TRIGGER, true); wake = tNow + 0.2; break;
            case 3: trigger(SDL_GAMEPAD_AXIS_RIGHT_TRIGGER, false); trigger(SDL_GAMEPAD_AXIS_LEFT_TRIGGER, true); wake = tNow + 0.6; break;
            case 4: say("geschossen, angelegt"); wantShot = true; trigger(SDL_GAMEPAD_AXIS_LEFT_TRIGGER, false); break;
            case 5: press("Waffe vorher", SDL_GAMEPAD_BUTTON_DPAD_RIGHT); break;
            case 6: say("Waffe nachher"); tap(SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER); break;
            case 7: press("Werkzeug auf", SDL_GAMEPAD_BUTTON_SOUTH); break;
            case 8: say("Mine gelegt"); tap(SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER); break;
            case 9: tap(SDL_GAMEPAD_BUTTON_DPAD_DOWN); break;
            case 10: tap(SDL_GAMEPAD_BUTTON_EAST); break;
            case 11: press("Werkzeug zu", SDL_GAMEPAD_BUTTON_LEFT_SHOULDER); break;
            case 12: say("Karte auf"); stick(SDL_GAMEPAD_AXIS_RIGHTX, 1.0); stick(SDL_GAMEPAD_AXIS_RIGHTY, -0.8); wake = tNow + 1.0; break;
            case 13: stick(SDL_GAMEPAD_AXIS_RIGHTX, 0); stick(SDL_GAMEPAD_AXIS_RIGHTY, 0); printf("Cursor %.0f %.0f\n", mapCursor.x, mapCursor.y); tap(SDL_GAMEPAD_BUTTON_SOUTH); break;
            case 14: say("markiert"); wantShot = true; printf("Marke %.0f %.0f\n", markPoint.x, markPoint.y); tap(SDL_GAMEPAD_BUTTON_WEST); break;
            case 15: say("gelöscht"); tap(SDL_GAMEPAD_BUTTON_EAST); break;
            case 16: press("Karte zu", SDL_GAMEPAD_BUTTON_BACK); break;
            case 17: say("Chat auf"); wantShot = true; tap(SDL_GAMEPAD_BUTTON_SOUTH); break;
            case 18: press("Tastatur", SDL_GAMEPAD_BUTTON_SOUTH); break;
            case 19: press("getippt", SDL_GAMEPAD_BUTTON_START); break;
            case 20: press("gesendet", SDL_GAMEPAD_BUTTON_BACK); break;
            case 21: tap(SDL_GAMEPAD_BUTTON_DPAD_DOWN); break;
            case 22: press("Schnellnachricht", SDL_GAMEPAD_BUTTON_SOUTH); break;
            case 23: {
              // Through the tools, with a strike code on the map.
              say("Vor Bombencode");
              tap(SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER);
              break;
            }
            case 24: {
              for (size_t i = 0; i < items.size(); i++)
                if (items[i].label.find("Bombencode") != std::string::npos) panelSel = int(i);
              say("Zeile gewählt");
              tap(SDL_GAMEPAD_BUTTON_SOUTH);
              break;
            }
            case 25: say("Karte für den Code"); wantShot = true; stick(SDL_GAMEPAD_AXIS_RIGHTX, 1.0); wake = tNow + 2.5; break;
            case 26: stick(SDL_GAMEPAD_AXIS_RIGHTX, 0); tap(SDL_GAMEPAD_BUTTON_SOUTH); break;
            case 27: say("Code benutzt"); stage = weaponPick == WP_SNIPER ? 29 : 28; break;
            case 28: tap(SDL_GAMEPAD_BUTTON_DPAD_RIGHT); stage = weaponPick == WP_SNIPER ? 29 : 28; break;
            case 29: press("Scharfschütze", SDL_GAMEPAD_BUTTON_EAST); wake = tNow + 3.6; break;
            case 30: say("liegt"); trigger(SDL_GAMEPAD_AXIS_LEFT_TRIGGER, true); break;
            case 31: trigger(SDL_GAMEPAD_AXIS_LEFT_TRIGGER, false); say("Scope"); tap(SDL_GAMEPAD_BUTTON_DPAD_UP); break;
            case 32: say("Visier"); SDL_SetJoystickVirtualButton(gTestPad, SDL_GAMEPAD_BUTTON_DPAD_RIGHT, true); wake = tNow + 1.0; break;
            case 33: SDL_SetJoystickVirtualButton(gTestPad, SDL_GAMEPAD_BUTTON_DPAD_RIGHT, false); say("Zoom"); wantShot = true; trigger(SDL_GAMEPAD_AXIS_LEFT_TRIGGER, true); break;
            case 34: trigger(SDL_GAMEPAD_AXIS_LEFT_TRIGGER, false); tap(SDL_GAMEPAD_BUTTON_EAST); wake = tNow + 2.0; break;
            case 35: say("wieder auf"); hold = tNow; stage = 36; break;
            case 36: {
              // Over to the tank by hand: the script only turns the head, the stick does the walking.
              const V3 to = G.world.base(Duel::teamOf(slot)).slot(gPickTurret) - view.foot.p;
              rig.yaw = std::atan2(-to.y, -to.x);
              rig.pitch = 0.2;
              stick(SDL_GAMEPAD_AXIS_LEFTY, -1.0);
              wake = tNow + 0.2;
              if (len(V3{to.x, to.y, 0}) < 4.2 || tNow - hold > 25.0) {
                stick(SDL_GAMEPAD_AXIS_LEFTY, 0);
                press("am Panzer", SDL_GAMEPAD_BUTTON_WEST);
                wake = tNow + 1.0;
              } else {
                stage = 36;
              }
              break;
            }
            case 37: say("drin"); printf("Panzermunition %d\n", view.tankAmmo); wantShot = true; stick(SDL_GAMEPAD_AXIS_LEFTY, -1.0); stick(SDL_GAMEPAD_AXIS_LEFTX, -0.4); wake = tNow + 2.5; break;
            case 38: say("gefahren"); stick(SDL_GAMEPAD_AXIS_LEFTY, 0); stick(SDL_GAMEPAD_AXIS_LEFTX, 0); trigger(SDL_GAMEPAD_AXIS_RIGHT_TRIGGER, true); wake = tNow + 0.3; break;
            case 39: trigger(SDL_GAMEPAD_AXIS_RIGHT_TRIGGER, false); wake = tNow + 0.6; break;
            case 40: printf("Panzermunition %d\n", view.tankAmmo); press("Kanone", SDL_GAMEPAD_BUTTON_NORTH); break;
            case 41: press("Kamera", SDL_GAMEPAD_BUTTON_START); break;
            case 42: say("Menü"); wantShot = true; tap(SDL_GAMEPAD_BUTTON_EAST); break;
            case 43: say("Menü zu"); quit = true; break;
            default: break;
          }
        }
      }
    }
    if (scriptMode == 60 && netState == Game::NET_IN) {
      // The tools, through the real keys: the old shortcuts do nothing, the list does.
      if (phase == Duel::CHOOSING && !pickReady) {
        pickItems[IT_FOOT_MINE] = 3;
        pickItems[IT_NET] = 1;
        pickReady = true;
      }
      static double started = -1;
      static int stage = 0;
      if (phase != Duel::PLAYING) started = -1, stage = 0;
      else if (started < 0) started = tNow;
      const auto press = [](SDL_Keycode k) {
        SDL_Event ev{};
        ev.type = SDL_EVENT_KEY_DOWN;
        ev.key.key = k;
        SDL_PushEvent(&ev);
      };
      const double at = started < 0 ? -1 : tNow - started;
      const auto state = [&](const char* what) { printf("%s: Panel %d Karte %d Zeilen %zu Minen %d Medikits %d Codes %d\n", what, panelKind, int(mapOpen), items.size(), view.foot.items[IT_FOOT_MINE], view.foot.medkits, view.foot.codes); };
      if (stage == 0 && at > 2) { state("Start"); press(SDLK_M); press(SDLK_B); stage++; }
      if (stage == 1 && at > 3) { state("nach M und B"); press(SDLK_Q); stage++; }
      if (stage == 2 && at > 4) { state("nach Q"); for (const Item& it : items) printf("  %s | %s\n", it.label.c_str(), it.value.c_str()); wantShot = true; stage++; }
      if (stage == 3 && at > 5) { press(SDLK_1); stage++; }
      if (stage == 4 && at > 6) { state("nach 1"); press(SDLK_Q); stage++; }
      if (stage == 5 && at > 7) {
        int row = -1;
        for (size_t i = 0; i < items.size(); i++) if (items[i].label.find("Bombencode") != std::string::npos) row = int(i);
        printf("Bombencode in Zeile %d\n", row);
        if (row >= 0) press(SDL_Keycode(SDLK_1 + row));
        stage++;
      }
      if (stage == 6 && at > 8) { state("nach Bombencode"); wantShot = true; stage++; }
      if (stage == 7 && at > 9) {
        int pw, ph;
        SDL_GetWindowSizeInPixels(window, &pw, &ph);
        float mx0, my0, ms;
        mapLayout(pw, ph, mx0, my0, ms);
        SDL_Event ev{};
        ev.type = SDL_EVENT_MOUSE_BUTTON_DOWN;
        ev.button.button = SDL_BUTTON_LEFT;
        ev.button.x = mx0 + ms * 0.5f;
        ev.button.y = my0 + ms * 0.5f;
        SDL_PushEvent(&ev);
        stage++;
      }
      if (stage == 8 && at > 10) { state("nach Klick"); stage++; }
    }
    if (scriptMode == 54 && netState == Game::NET_IN && phase == Duel::CHOOSING) {
      // The loadout menu, with something chosen.
      pickItems[IT_REPAIR] = 1;
      pickItems[IT_AR] = 1;
      pickItems[IT_SMALL_AMMO] = 3;
      panelSel = 2;
    }
    if (scriptMode == 45 && netState == Game::NET_IN) {
      // A look at the figure: standing, walking, running.
      if (phase == Duel::CHOOSING && !pickReady) {
        pickReady = true;
        panelKind = PANEL_NONE;
      }
      if (inMatch) {
        panelKind = PANEL_NONE;
        rig.yaw = PI * 0.5;
        rig.pitch = 0.02;
        weaponPick = WP_SNIPER;
        static bool down = false;
        if (tNow > 2.0 && !down) {
          down = true;
          sendC.setup = true;
        }
      }
    }
    if (scriptMode == 49 && netState == Game::NET_IN && inVehicle) {
      static double launchedAt = -100;
      if (tNow - launchedAt > 25.0) {
        sendC.launch = true;
        launchedAt = tNow;
      }
      // Up and away while it flies, and between times through the gunner's sight.
      if (view.missileLive) rig.pitch = -0.25;
      else rig.pitch = 0.02;
      scriptAim = !view.missileLive && std::fmod(tNow, 4.0) > 2.0;
    }
    if (scriptMode == 53 && netState == Game::NET_IN) {
      // The launcher and a strike code, watched from the side.
      if (phase == Duel::CHOOSING && !pickReady) pickReady = true;
      panelKind = PANEL_NONE;
      if (inMatch) {
        weaponPick = WP_RPG;
        rig.yaw = G.world.base(Duel::teamOf(slot)).facing + PI;
        rig.pitch = -0.02;
        if (std::fmod(tNow, 2.0) < dt) sendC.fire = true;
        static bool called = false;
        if (!called && view.foot.codes > 0) {
          called = true;
          sendC.strike = true;
          sendC.strikeAt = G.world.base(Duel::teamOf(slot)).centre + V3{std::cos(G.world.base(Duel::teamOf(slot)).facing), std::sin(G.world.base(Duel::teamOf(slot)).facing), 0} * 70.0;
        }
      }
    }
    if (scriptMode == 57 && netState == Game::NET_IN) {
      if (phase == Duel::CHOOSING && !pickReady) {
        pickItems[IT_GHILLIE] = 1;
        pickReady = true;
      }
      panelKind = PANEL_NONE;
      if (inMatch) {
        rig.yaw = G.world.base(Duel::teamOf(slot)).facing + PI;
        rig.pitch = 0.1;
        sprintScript = true;
        if (tNow < 4.5) rig.yaw += 1.0;  // first out past the row of vehicles
        gas = 1.0;
      }
    }
    if (scriptMode == 59) {
      // A match, then seeing it again from both sides and from above.
      static bool left = false, watching = false;
      if (netState == Game::NET_IN && phase == Duel::CHOOSING && !pickReady) pickReady = true;
      panelKind = PANEL_NONE;
      if (netState == Game::NET_IN && inMatch && !left) {
        gas = 1.0;
        const V3 to = G.world.base(Duel::teamOf(slot)).slot(1) - view.foot.p;
        if (onFoot) rig.yaw = std::atan2(-to.y, -to.x);
        if (onFoot && len(V3{to.x, to.y, 0}) < 4.5) sendC.use = true;
        static double since = tNow;
        if (tNow - since > 9.0) {
          left = true;
          G.command = Game::CMD_LEAVE;
        }
      } else if (left && netState == Game::NET_OFF && hasReplayNow && !watching) {
        watching = true;
        G.command = Game::CMD_REPLAY;
        printf("Replay beginnt bei t=%.1f\n", tNow);
      }
    }
    if (scriptMode == 58 && netState == Game::NET_IN) {
      if (phase == Duel::CHOOSING && !pickReady) pickReady = true;
      panelKind = PANEL_NONE;
      if (inMatch) {
        rig.yaw = G.world.base(Duel::teamOf(slot)).facing + PI + 0.15;
        rig.pitch = 0.18;
        static double last = -100;
        if (tNow - last > 2.0 && !view.foot.droneLive) {
          sendC.drone = true;
          last = tNow;
        }
        if (view.foot.droneLive) {
          gas = 1.0;
          sprintScript = true;
        }
      }
    }
    if (scriptMode == 56 && netState == Game::NET_IN) {
      if (phase == Duel::CHOOSING && !pickReady) {
        pickItems[IT_GHILLIE] = 1;
        pickItems[IT_NET] = 1;
        pickReady = true;
      }
      panelKind = PANEL_NONE;
      if (inMatch) {
        const V3 to = G.world.base(Duel::teamOf(slot)).slot(0) - view.foot.p;
        if (onFoot) {
          rig.yaw = std::atan2(-to.y, -to.x);
          gas = 1.0;
          if (len(V3{to.x, to.y, 0}) < 4.5) sendC.use = true;
        } else {
          static double last = -100;
          if (tNow - last > 2.0) {
            sendC.net = true;
            last = tNow;
          }
        }
      }
    }
    if (scriptMode == 55 && netState == Game::NET_IN) {
      // Smoke, thrown and from a tank.
      if (phase == Duel::CHOOSING && !pickReady) {
        pickItems[IT_SMOKE] = 4;
        pickReady = true;
      }
      panelKind = PANEL_NONE;
      if (inMatch) {
        rig.yaw = G.world.base(Duel::teamOf(slot)).facing + PI;
        rig.pitch = 0.1;
        static double last = -100;
        if (tNow - last > 3.0 && view.foot.items[IT_SMOKE] > 0) {
          sendC.smoke = true;
          last = tNow;
        }
      }
    }
    if (scriptMode == 51 && netState == Game::NET_IN) {
      if (phase == Duel::CHOOSING && !pickReady) pickReady = true;
      panelKind = PANEL_NONE;
    }
    if (scriptMode == 50 && inMatch) {
      // Each way of aiming in turn: on foot with the pistol, then the rifle.
      panelKind = PANEL_NONE;
      scriptAim = std::fmod(tNow, 4.0) > 2.0;
      weaponPick = tNow < 8.0 ? WP_PISTOL : WP_AR;
      rig.yaw = G.world.base(Duel::teamOf(slot)).facing + PI + 0.5;
      rig.pitch = 0.05;
    } else if (scriptMode == 50 && phase == Duel::CHOOSING && !pickReady) {
      pickReady = true;
      panelKind = PANEL_NONE;
    }
    if ((scriptMode == 41 || scriptMode == 48 || scriptMode == 49 || scriptMode == 63 || scriptMode == 71) && netState == Game::NET_IN) {
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
          const V3 to = G.world.base(Duel::teamOf(slot)).slot(gPickTurret) - view.foot.p;
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
          if (std::fmod(tNow, 3.0) < dt && !view.missileLive) sendC.fire = true;
          static bool leftOnce = false;
          if (tNow - boarded > 16.0 && !leftOnce && scriptMode != 49 && scriptMode != 71) {
            leftOnce = true;
            sendC.use = true;
            printf("Aussteigen bei t=%.1f\n", tNow);
          }
        }
      }
    }

    if (scriptMode == 71 && inMatch) {
      // As far up as one can look: standing about first, then in the vehicle; and a missile flown up and then straight down.
      static double started = tNow, boardedAt = -1;
      if (onFoot && tNow - started < 5.0) {
        gas = 0;
        sendC.use = false;
        rig.pitch = pitchUp;
      } else if (inVehicle) {
        if (boardedAt < 0) boardedAt = tNow;
        gas = 0;
        sendC.fire = false;
        if (tNow - boardedAt > 1.0 && tNow - boardedAt < 1.0 + dt * 2.0 && !view.missileLive) sendC.launch = true;
        rig.pitch = view.missileLive && tNow - boardedAt > 6.0 ? pitchDown : pitchUp;
      }
    }

    if (scriptMode == 68) {
      // The loading screens: connecting to nothing, then waiting alone on a server that is there.
      static double shotAt = 4.0;
      if (tNow > shotAt && loadingShown) {
        wantShot = true;
        shotAt = tNow + 5.0;
      }
      static int step = 0;
      if (step == 0 && tNow > 9.0) {
        printf("Vor ESC: Netz %d, Ladebildschirm %d, Panel %d\n", netState, int(loadingShown), panelKind);
        SDL_Event ev{};
        ev.type = SDL_EVENT_KEY_DOWN;
        ev.key.key = SDLK_ESCAPE;
        SDL_PushEvent(&ev);
        step = 1;
      }
      if (step == 1 && tNow > 11.0) {
        printf("Nach ESC: Netz %d, Ladebildschirm %d, Panel %d\n", netState, int(loadingShown), panelKind);
        quit = true;
        step = 2;
      }
    }
    if (scriptMode == 67 && netState == Game::NET_IN && inMatch) {
      // The map with its squares, marked.
      panelKind = PANEL_NONE;
      mapOpen = true;
      hasMark = true;
      markPoint = V3{250, -300, 0};
      static double shot = tNow + 3.0;
      if (tNow > shot) {
        wantShot = true;
        shot = 1e30;
      }
    } else if (scriptMode == 67 && netState == Game::NET_IN && phase == Duel::CHOOSING) {
      pickReady = true;
      panelKind = PANEL_NONE;
    }
    if (scriptMode == 66 && netState == Game::NET_IN) {
      // Out of the yard, through the meadow, and a look back at what the tank has left behind.
      if (phase == Duel::CHOOSING) {
        pickReady = true;
        panelKind = PANEL_NONE;
      }
      static double started = -1, nextSay = 0;
      static V3 bush;
      if (inMatch) {
        panelKind = PANEL_NONE;
        if (onFoot && started > 0) {
          // Out of the tank: over to what it drove across, and then to a bush that nothing has touched.
          static V3 other;
          static int stage = 0;
          static double arrived = 0;
          if (stage == 0) {
            other = bush;
            stage = 1;
          }
          const V3 to{other.x - view.foot.p.x, other.y - view.foot.p.y, 0};
          rig.yaw = std::atan2(-to.y, -to.x);
          rig.pitch = 0.22;
          const bool walking = stage == 1 || stage == 3;
          gas = walking ? 1.0 : 0.0;
          if (walking && len(to) < 5.5) {
            stage++;
            arrived = tNow;
          }
          if ((stage == 2 || stage == 4) && tNow - arrived > 0.8) {
            wantShot = true;
            if (stage == 2) other = renderer.nearestBush(view.foot.p, 14.0);
            stage++;
          }
        } else if (onFoot) {
          const V3 to = G.world.base(Duel::teamOf(slot)).slot(gPickTurret) - view.foot.p;
          rig.yaw = std::atan2(-to.y, -to.x);
          rig.pitch = 0.2;
          gas = 1.0;
          static double lastUse = 0;
          if (len(V3{to.x, to.y, 0}) < 4.5 && tNow - lastUse > 1.0) {
            sendC.use = true;
            lastUse = tNow;
          }
        } else if (inTank) {
          if (started < 0) started = tNow;
          const double at = tNow - started;
          // At a bush some way off: right over it, and a look back at what is left of it.
          static bool haveBush = false;
          if (!haveBush) {
            bush = renderer.nearestBush(view.carCog + V3{std::cos(G.world.base(Duel::teamOf(slot)).facing), std::sin(G.world.base(Duel::teamOf(slot)).facing), 0} * 110.0, 25.0);
            haveBush = true;
            printf("Busch bei %.0f %.0f\n", bush.x, bush.y);
          }
          static double passedAt = -1;
          const V3 toBush{bush.x - view.carCog.x, bush.y - view.carCog.y, 0};
          const double err = std::remainder(std::atan2(toBush.y, toBush.x) - std::atan2(view.carFwd.y, view.carFwd.x), 2.0 * PI);
          if (passedAt < 0 && len(toBush) < 2.0) passedAt = at;
          // Whatever stands in the way, the look back still comes.
          if (passedAt < 0 && at > 30.0) passedAt = at;
          const bool driving = passedAt < 0 ? at < 40.0 : at < passedAt + 2.0;
          steer = passedAt < 0 ? clampd(err * 2.0, -1.0, 1.0) : 0.0;
          gas = driving ? 1.0 : 0.0;
          brake = driving ? 0.0 : 1.0;
          static bool closeShot = false;
          static double sayAt = 0;
          if (tNow > sayAt) {
            printf("Fahrt zum Busch: noch %.0f m, %.0f km/h, t=%.1f\n", len(toBush), view.speedKmh, at);
            sayAt = tNow + 4.0;
          }
          if (!closeShot && passedAt < 0 && len(toBush) < 14.0) {
            closeShot = true;
            wantShot = true;
          }
          const double along = std::atan2(view.carFwd.y, view.carFwd.x);
          rig.yaw = passedAt < 0 ? along + PI : along;
          rig.pitch = 0.3;
          if (passedAt > 0 && at > passedAt + 5.0) {
            static bool backShot = false;
            if (!backShot) {
              backShot = true;
              wantShot = true;
              printf("Zurueckgeschaut bei t=%.1f\n", at);
            } else if (at > passedAt + 6.0) {
              static bool left = false;
              if (!left) sendC.use = true;
              left = true;
            }
          }
        }
        if (tNow > nextSay) {
          int grass, bushes, tris, statics;
          double ms;
          renderer.vegetationStats(grass, bushes, tris, statics, ms);
          printf("Pflanzen: %.0f fps, Gras %d, Buesche %d, Dreiecke %d, statisch %d, Neuaufbau %.2f ms, %.0f km/h, Pos %.0f %.0f\n", hud.fps, grass, bushes, tris, statics, ms, view.speedKmh, view.carCog.x, view.carCog.y);
          nextSay = tNow + 3.0;
        }
      }
    }
    if (scriptMode == 70 && netState == Game::NET_IN) {
      // Running about without a plan: forwards, sideways, turning, jumping. What the correction of walking comes to is in the log.
      if (phase == Duel::CHOOSING) {
        pickReady = true;
        panelKind = PANEL_NONE;
      }
      if (onFoot) {
        panelKind = PANEL_NONE;
        const double part = std::fmod(tNow, 8.0);
        gas = part < 5.0 ? 1.0 : 0.0;
        steer = part > 3.0 && part < 7.0 ? (std::fmod(tNow, 2.0) < 1.0 ? 1.0 : -1.0) : 0.0;
        rig.yaw += dt * 0.6 * std::sin(tNow * 0.7);
        rig.pitch = 0.1;
        hb = std::fmod(tNow, 3.0) < 0.1 ? 1.0 : 0.0;
      }
    }
    if (scriptMode == 65 && netState == Game::NET_IN) {
      // Turning on the spot, as with the mouse: how far the figure is off the camera, and pictures of it.
      if (phase == Duel::CHOOSING) {
        pickReady = true;
        panelKind = PANEL_NONE;
      }
      if (onFoot) {
        panelKind = PANEL_NONE;
        static double sumOff = 0, sumOff2 = 0, biggest = 0, nextSay = 0, nextShot = 0;
        static int frames = 0;
        rig.yaw += dt * 2.5;
        const double off = figureOff;
        sumOff += off;
        sumOff2 += off * off;
        biggest = std::max(biggest, std::fabs(off));
        frames++;
        if (tNow > nextSay && frames > 0) {
          const double mean = sumOff / frames;
          printf("Figur gegen Kamera: Mittel %.2f Grad, Schwankung %.2f, groesste %.2f, %d Bilder\n", mean, std::sqrt(std::max(0.0, sumOff2 / frames - mean * mean)), biggest, frames);
          sumOff = sumOff2 = biggest = 0;
          frames = 0;
          nextSay = tNow + 2.0;
        }
        if (tNow > 6.0 && tNow > nextShot) {
          wantShot = true;
          nextShot = tNow + 1.3;
        }
      }
    }
    if (scriptMode == 63 && inTank) {
      // Steering to and fro, and how far the picture of one's own tank jumps from one frame to the next.
      static double lastHeading = 0, biggest = 0, nextSay = 0;
      static bool started = false;
      gas = 0.7;
      brake = 0;
      steer = std::fmod(tNow, 3.0) < 1.5 ? 1.0 : -1.0;
      const double heading = std::atan2(view.carFwd.y, view.carFwd.x);
      if (started) biggest = std::max(biggest, std::fabs(std::remainder(heading - lastHeading, 2.0 * PI)));
      lastHeading = heading;
      started = true;
      if (tNow > nextSay) {
        printf("Kurs: groesster Sprung pro Bild %.2f Grad, %.0f km/h\n", biggest * 180.0 / PI, view.speedKmh);
        biggest = 0;
        nextSay = tNow + 2.0;
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
      c.setup |= sendC.setup;
      c.placeFoot |= sendC.placeFoot;
      c.placeTank |= sendC.placeTank;
      c.query |= sendC.query;
      c.smoke |= sendC.smoke;
      c.net |= sendC.net;
      c.drone |= sendC.drone;
      c.medkit |= sendC.medkit;
      if (sendC.strike) {
        c.strike = true;
        c.strikeAt = sendC.strikeAt;
      }
      G.input.yaw = rig.yaw;
      G.input.pitch = rig.pitch;
      G.input.sprint = sprint;
      G.input.weapon = weaponPick;
      G.input.scoped = scoped;
      G.input.aiming = aimBlend > 0.5;
      G.input.fireHeld = padFire || (panelKind == 0 && !mapOpen && !chatting && (SDL_GetMouseState(nullptr, nullptr) & SDL_BUTTON_LMASK));
      if (sendC.requestGear != 99) c.requestGear = sendC.requestGear;
      G.input.ready = pickReady;
      for (int i = 0; i < IT_COUNT; i++) G.input.items[i] = pickItems[i];
      c.indirect = false;
      if (view.missileLive) {
        // The camera flies with the missile, and where it looks is where it goes.
        const V3 back{std::cos(rig.yaw) * std::cos(rig.pitch), std::sin(rig.yaw) * std::cos(rig.pitch), std::sin(rig.pitch)};
        c.aimPoint = view.missileP - back * 2000.0;
      } else if (hasMark && inTank && myVehicle % TK_COUNT == TK_ARTILLERY) {
        c.aimPoint = markPoint;
        c.indirect = true;
      } else {
        // Whatever is under the crosshair is what the gun lays on.
        const V3 f = norm(cam.target - cam.eye);
        // From beyond whatever one is sitting in, or just past one's own shoulder.
        const double clear = onFoot ? 2.6 : 14.0;
        // Exactly what the crosshair lies on: the shot starts from elsewhere than the camera, so close by it only goes
        // to the right place if that place is known.
        c.aimPoint = Duel::aimEnd(G.world, view.pos, G.simStart.blocks, aimFrames.data(), int(aimFrames.size()), myVehicle, aimFoes, cam.eye, f, clear);
        // Through the scope it goes exactly where the scope points. Drop and wind are the shooter's business.
        // Except that it is set for a range: the barrel points up by what the bullet will fall on the way there.
        if (scoped) {
          const double lift = 0.5 * std::asin(std::min(9.81 * double(scopeZero) / (kWeapons[WP_SNIPER].speed * kWeapons[WP_SNIPER].speed), 1.0));
          c.aimPoint = cam.eye + norm(f + V3{0, 0, 1} * lift) * 3000.0;
        }
      }
    }

    G.askForFrame();
    // A new match starts on clean ground.
    if (G.resetVisuals.exchange(false)) {
      if (netState == Game::NET_IN) newMatchUntil = tNow + 0.9;
      renderer.resetDamage();
      renderer.clearSkidMarks();
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
      view.netOn = G.snap.netOn;
      view.netPing = G.snap.netPing;
      view.netLoss = G.snap.netLoss;
      view.netDelay = G.snap.netDelay;
      view.netKb = G.snap.netKb;
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
      view.groundMarks = G.snap.groundMarks;
      view.groundBoxes = G.snap.groundBoxes;
      view.selfInstance = G.snap.selfInstance;
      aimFrames = G.sightFrames;
      aimFoes = G.sightFoes;
      view.selfOnFoot = G.snap.selfOnFoot;
      view.selfTrack = G.snap.selfTrack;
      view.chat = G.snap.chat;
      view.foot = G.snap.foot;
      view.missileLive = G.snap.missileLive;
      view.missileP = G.snap.missileP;
      view.missileV = G.snap.missileV;
      view.missiles = G.snap.missiles;
      view.repairKits = G.snap.repairKits;
      view.trackLeft[0] = G.snap.trackLeft[0];
      view.trackLeft[1] = G.snap.trackLeft[1];
      for (int i = 0; i < 64; i++) view.parkedScorch[i] = G.snap.parkedScorch[i];
      view.tankArmour = G.snap.tankArmour;
      view.tankAmmo = G.snap.tankAmmo;
      view.enemyCog = G.snap.enemyCog;
      view.tankDead = G.snap.tankDead;
      view.nodeObj = G.snap.nodeObj;
      view.paused = G.snap.paused;
      view.timeScale = G.snap.timeScale;
    }

    // One's own figure turns with the camera in the same frame. The picture from the network thread is a moment old and
    // keeps no step with the frames: seen from right behind it, that swings to and fro however it is smoothed.
    if (onFoot && netState == Game::NET_IN && !scoped && view.foot.hp > 0 && !view.foot.droneLive && slot >= 0) {
      const double turn = std::remainder(rig.yaw - view.foot.yaw, 2.0 * PI);
      const double c = std::cos(turn), sn = std::sin(turn);
      const u32 first = G.duel.soldierFirst[slot];
      const V3 at = view.foot.p;
      for (u32 k = 0; k < u32(Duel::kSoldierNodes) && size_t(first + k) < view.pos.size(); k++) {
        const V3 q = view.pos[first + k] - at;
        view.pos[first + k] = at + V3{q.x * c - q.y * sn, q.x * sn + q.y * c, q.z};
      }
      if (scriptMode == 65 && size_t(first) < view.pos.size()) {
        const V3 q = view.pos[first] - at;
        figureOff = std::remainder(std::atan2(q.y, q.x) - rig.yaw, 2.0 * PI) * 180.0 / PI;
      }
    }
    if (gPlayLog) {
      static double next = 0, worst = 0;
      worst = std::max(worst, dt);
      if (tNow > next) {
        fprintf(gPlayLog, "t %.0f bild fps %.0f schlechtestes %.0f ms\n", tNow, hud.fps, worst * 1000.0);
        worst = 0;
        next = tNow + 1.0;
      }
    }
    hud.panel.open = panelKind != 0;
    if (hud.panel.open) {
      static const char* titles[8] = {"", "Spiel-Einstellungen", "BeanNG", "Steuerung", "Online spielen", "Ausrüstung", "Werkzeug", "Chat"};
      hud.panel.title = titles[panelKind];
      if (panelKind == PANEL_LOADOUT) {
        char b[64];
        snprintf(b, sizeof b, "Ausrüstung  ·  noch %.0f s", std::max(0.0, Duel::kChooseTime - phaseClock));
        hud.panel.title = b;
      }
      if (padActive) hud.panel.foot = panelKind == PANEL_TOOLS ? "A benutzen  ·  B schließen" : panelKind == PANEL_CHAT ? "A senden  ·  B schließen" : panelKind == PANEL_ONLINE ? "A bearbeiten, verbinden  ·  B zurück" : panelKind == PANEL_MAIN ? "A bestätigen  ·  B zurück" : panelKind == PANEL_KEYS ? "B zurück" : panelKind == PANEL_LOADOUT ? "A mehr  ·  X weniger  ·  B Menü" : "Links, rechts ändern  ·  B zurück";
      else hud.panel.foot = panelKind == PANEL_CHAT ? "Pfeile wählen  ·  ENTER senden  ·  ESC schließen" : panelKind == PANEL_TOOLS ? "Zahl oder ENTER benutzen  ·  Q schließen" : panelKind == PANEL_ONLINE ? "Tippen  ·  Pfeile wählen  ·  ENTER weiter  ·  ESC zurück" : panelKind == PANEL_MAIN ? "Pfeile wählen  ·  ENTER bestätigen  ·  ESC zurück" : (panelKind == PANEL_KEYS ? "ESC zurück" : "Pfeile wählen und ändern  ·  ESC zurück");
      hud.panel.sel = panelSel;
      hud.panel.rows.clear();
      hud.panel.values.clear();
      hud.panel.fill.clear();
      hud.panel.detail.clear();
      hud.panel.slots.clear();
      if (panelKind == PANEL_LOADOUT) {
        static const char* kAbout[IT_COUNT] = {
            "Tretmine: liegt flach am Boden und ist nach 3 Sekunden scharf. Wer drauftritt, stirbt meist. Auch ein Fahrzeug löst sie aus. Liegt im Werkzeugmenü.",
            "Panzermine: reißt die Kette auf der Seite ab und richtet schweren Schaden an. Zu Fuß nichts zu befürchten. Liegt im Werkzeugmenü.",
            "Reparaturset: bringt ein Fahrzeug wieder zum Laufen. Dauert 6 Sekunden zu Fuß daneben, und es muss stillstehen. Liegt im Werkzeugmenü.",
            "Sturmgewehr: Dauerfeuer auf mittlere Entfernung. Schießt mit der Kleinmunition. Gegen Panzer fast wirkungslos.",
            "Kleinmunition: 60 Schuss für Pistole und Sturmgewehr, zusätzlich zu den 24, die du immer hast.",
            "Nebelgranate: eine Wolke, die eine halbe Minute lang die Sicht versperrt, gut gegen Scharfschützen. Liegt im Werkzeugmenü. Jeder Panzer hat außerdem 3 Ladungen im Werfer.",
            "Ghillie-Anzug: ein zottiger Blätteranzug. Wer ihn trägt, ist im Gras und liegend kaum zu erkennen. Wirkt von selbst, solange du lebst.",
            "Tarnnetz: im stehenden Panzer über das Werkzeugmenü drübergezogen, dauert 3 Sekunden. Beim Losfahren bleibt es zurück.",
        };
        if (panelSel >= 1 && panelSel <= IT_COUNT) hud.panel.detail = kAbout[panelSel - 1];
        else if (panelSel == 0) hud.panel.detail = "Das hast du immer dabei: Pistole mit 24 Schuss und Kampfmesser. Alles andere findest du in Kisten in Häusern und bei Wracks: Scharfschützengewehr, Raketenwerfer, Granaten und Bombencodes.";
        else hud.panel.detail = "20 Slots. Ist alles gewählt, wähle Bereit. Du startest zu Fuß in deiner Basis und nimmst dir dort ein Fahrzeug.";
        for (int k = 0; k < IT_COUNT; k++)
          for (int n = 0; n < pickItems[k] * kItems[k].slots; n++) hud.panel.slots.push_back(k);
        while (int(hud.panel.slots.size()) < kLoadoutSlots) hud.panel.slots.push_back(-1);
      }
      if (panelKind == PANEL_TOOLS && size_t(panelSel) < toolAbout.size()) hud.panel.detail = toolAbout[size_t(panelSel)];
      for (const Item& it : items) {
        hud.panel.rows.push_back(it.label);
        hud.panel.values.push_back(it.value);
        hud.panel.fill.push_back(it.fill);
      }
    }
    view.chatOpen = chatting;
    view.chatInput = chatText;
    // Being hit shows: the picture goes red for a moment.
    {
      // A hit landed: a mark in the crosshair and a tick. A hit taken: a mark on the side it came from.
      static int landedSeen = 0, takenSeen = 0;
      if (view.foot.hitsLanded != landedSeen) {
        if (view.foot.hitsLanded > landedSeen) {
          hud.hitMark = 1.f;
          sound->synth.trigger(Sfx::Crack, 0.25f, 0.f, 0.5f);
        }
        landedSeen = view.foot.hitsLanded;
      }
      if (view.foot.hitsTaken != takenSeen) {
        if (view.foot.hitsTaken > takenSeen) {
          if (gamepad && gSet.rumble) SDL_RumbleGamepad(gamepad, 52000, 30000, 260);
          hud.hurtDir = 2.2f;
          hud.hurtFrom = view.foot.hurtFrom;
        }
        takenSeen = view.foot.hitsTaken;
      }
      hud.hitMark = std::max(0.f, hud.hitMark - float(dt) * 3.5f);
      hud.hurtDir = std::max(0.f, hud.hurtDir - float(dt));
      static double lastHp = 100;
      if (view.foot.hp < lastHp - 0.5) hurtFlash = std::min(1.0, hurtFlash + (lastHp - view.foot.hp) / 40.0 + 0.3);
      lastHp = view.foot.hp;
      hurtFlash = std::max(0.0, hurtFlash - dt * 1.4);
      hud.hurt = float(hurtFlash);
    }
    // What one can hear of the others, on the screen as well: shots and blasts as they happen, engines as long as they run.
    {
      for (HudOptions::Heard& h : hud.heard) h.life -= float(dt);
      hud.heard.erase(std::remove_if(hud.heard.begin(), hud.heard.end(), [](const HudOptions::Heard& h) { return h.life <= 0 || h.life > 99.f; }), hud.heard.end());
      for (const SimEvent& e : view.events) {
        if (e.kind != EV_MUZZLE && e.kind != EV_BLAST) continue;
        const double d = len(e.p - cam.eye);
        // One's own are no news.
        if (d < 12.0 && e.kind == EV_MUZZLE) continue;
        const double reach = e.kind == EV_BLAST ? 1400.0 : (e.strength > 0.4f ? 1100.0 : (e.strength > 0.12f ? 500.0 : 280.0));
        if (d > reach) continue;
        hud.heard.push_back({e.p, float(std::clamp(1.0 - d / reach, 0.25, 1.0)), 2.0f, true});
      }
      if (hud.heard.size() > 12) hud.heard.erase(hud.heard.begin(), hud.heard.end() - 12);
      if (inMatch) {
        V3 who;
        double speed = 0;
        if (G.enemyVehicleFrame(slot, cam.eye, who, speed) && speed > 1.5) {
          const double d = len(who - cam.eye);
          if (d < 420.0) hud.heard.push_back({who, float(std::clamp(1.0 - d / 420.0, 0.2, 1.0)), 0.05f, false});
        }
      }
    }
    hud.vehicleHud = inVehicle;
    hud.crosshair = inMatch && !mapOpen && !view.missileLive && !view.foot.inCar;
    // Without its optics the gunner's sight shows nothing any closer.
    if (view.foot.noSight && rig.mode == CAM_HOOD) rig.mode = CAM_CHASE;
    hud.map.open = mapOpen && panelKind == 0;
    hud.grassDensity = gSet.grass;
    {
      // On the map a mouse has a place as well.
      hud.vector.clear();
      if (netState == Game::NET_IN && phase == Duel::PLAYING) {
        char name[4];
        vectorName(view.carCog.x, view.carCog.y, name);
        hud.vector = name;
      }
    }
    hud.map.hasMark = hasMark;
    hud.map.padCursor = padActive;
    hud.map.cursor = mapCursor;
    hud.map.mouseKnown = false;
    if (mapOpen && !padActive) {
      int pw, ph;
      SDL_GetWindowSizeInPixels(window, &pw, &ph);
      float mx0, my0, ms, mouseX, mouseY;
      mapLayout(pw, ph, mx0, my0, ms);
      SDL_GetMouseState(&mouseX, &mouseY);
      const double wx = (double(mouseX - mx0) / ms * 2.0 - 1.0) * kMapHalf, wy = (1.0 - double(mouseY - my0) / ms * 2.0) * kMapHalf;
      if (std::fabs(wx) < kMapHalf && std::fabs(wy) < kMapHalf) {
        hud.map.mouseKnown = true;
        hud.map.cursor = V3{wx, wy, 0};
      }
    }
    hud.osk.open = osk.open;
    if (osk.open) {
      static const char* kOskTitle[4] = {"Benutzername", "Server-Adresse", "Port", "Chat"};
      hud.osk.title = kOskTitle[osk.target];
      hud.osk.text = osk.text;
      hud.osk.x = osk.x;
      hud.osk.y = osk.y;
      for (int r = 0; r < 4; r++) hud.osk.keys[r] = kOskKeys[osk.shift ? 1 : 0][r];
    }
    hud.map.strike = strikePending;
    hud.map.mark = markPoint;
    hud.map.hasEnemy = view.foot.hasFix;
    hud.map.enemy = view.foot.fix;
    hud.map.fixAge = view.foot.fixAge;
    hud.map.fixWait = view.foot.tapped ? view.foot.fixWait : -1.0;
    hud.map.me = view.carCog;
    hud.map.noRadio = view.foot.noRadio;
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
    if (replayOn && replayFree) {
      // Flying about: where one looks, as fast as one likes.
      const V3 back{std::cos(rig.yaw) * std::cos(rig.pitch), std::sin(rig.yaw) * std::cos(rig.pitch), std::sin(rig.pitch)};
      const V3 right{-std::sin(rig.yaw), std::cos(rig.yaw), 0};
      const double fly = keys[SDL_SCANCODE_LSHIFT] || padSprint ? 80.0 : 25.0;
      freeEye -= back * ((keys[SDL_SCANCODE_W] ? fly : 0.0) - (keys[SDL_SCANCODE_S] ? fly : 0.0) - padLy * fly) * dt;
      freeEye += right * ((keys[SDL_SCANCODE_D] ? fly : 0.0) - (keys[SDL_SCANCODE_A] ? fly : 0.0) + padLx * fly) * dt;
      cam.eye = freeEye;
      cam.target = freeEye - back * 30.0;
      cam.up = {0, 0, 1};
      cam.inside = false;
      cam.fovY = 62.0 * PI / 180.0;
    } else if (view.foot.droneLive && !mapOpen) {
      // Through the drone's camera, looking where it is told to.
      const V3 back{std::cos(rig.yaw) * std::cos(rig.pitch), std::sin(rig.yaw) * std::cos(rig.pitch), std::sin(rig.pitch)};
      cam.eye = view.foot.droneP - V3{0, 0, 0.1};
      cam.target = cam.eye - back * 50.0;
      cam.up = {0, 0, 1};
      cam.inside = false;
      cam.fovY = 74.0 * PI / 180.0;
    } else if (onFoot && !mapOpen) {
      // Over the right shoulder, and close. Forward is along -back; right of that, seen from behind.
      const V3 back{std::cos(rig.yaw) * std::cos(rig.pitch), std::sin(rig.yaw) * std::cos(rig.pitch), std::sin(rig.pitch)};
      const V3 right{-std::sin(rig.yaw), std::cos(rig.yaw), 0};
      Soldier at;
      at.p = view.foot.p;
      at.yaw = rig.yaw;
      at.prone = view.foot.prone;
      // Looking up, further out to the side: from below the figure would fill half the picture.
      const V3 head = Duel::eyeOf(at) + V3{0, 0, 0.12} + right * (0.42 + 0.45 * smooth01(clampd((-rig.pitch - 0.3) / 0.8, 0, 1)));
      const double wanted = lerpd(1.9, 1.5, view.foot.prone);
      double reach = wanted;
      for (double d = 0.2; d < wanted; d += 0.1) {
        Contact con;
        if (G.world.contact(head + back * d, con)) {
          reach = std::max(d - 0.2, 0.2);
          break;
        }
      }
      cam.eye = head + back * reach;
      cam.target = head - back * 30.0;
      cam.up = {0, 0, 1};
      cam.inside = false;
      cam.fovY = 66.0 * PI / 180.0 + gSet.fov * PI / 180.0;
    }
    {
      // A shot gone means the ammunition went down with the same weapon in hand.
      const int ammoNow = view.foot.ammo;
      // The tank's gun counts its own shells.
      static int shellsSeen = -1;
      if (gamepad && gSet.rumble && inTank && shellsSeen >= 0 && view.tankAmmo < shellsSeen) SDL_RumbleGamepad(gamepad, 65000, 22000, 320);
      shellsSeen = inTank ? view.tankAmmo : -1;
      if (onFoot && kickWeapon == weaponPick && kickAmmo >= 0 && ammoNow == kickAmmo - 1) {
        if (gamepad && gSet.rumble) {
          const bool big = weaponPick == WP_SNIPER || weaponPick == WP_RPG;
          SDL_RumbleGamepad(gamepad, big ? 60000 : 9000, big ? 42000 : 26000, big ? 300 : 70);
        }
        if (weaponPick == WP_SNIPER) {
          gunKick = 1.0;
          rig.pitch = clampd(rig.pitch - 0.018 * (scoped ? 6.0 / scopeZoom : 1.0), pitchUp, pitchDown);
          rig.yaw += 0.004 * std::sin(tNow * 91.0) * (scoped ? 6.0 / scopeZoom : 1.0);
        } else if (weaponPick == WP_RPG) {
          gunKick = 0.7;
        } else if (weaponPick == WP_AR) {
          gunKick = std::max(gunKick, 0.12);
        } else if (weaponPick == WP_PISTOL) {
          gunKick = std::max(gunKick, 0.1);
        }
      }
      kickAmmo = ammoNow;
      kickWeapon = weaponPick;
    }
    if (aimBlend > 0.001) {
      const V3 back{std::cos(rig.yaw) * std::cos(rig.pitch), std::sin(rig.yaw) * std::cos(rig.pitch), std::sin(rig.pitch)};
      const double k = smooth01(aimBlend);
      if (inTank) {
        // Through the gunner's sight: from the camera on the turret down into it.
        const V3 eye = view.sight;
        cam.eye = cam.eye + (eye - cam.eye) * k;
        cam.target = cam.target + ((eye - back * 50.0) - cam.target) * k;
        const double wide = myVehicle % TK_COUNT == TK_LIGHT ? 26.0 : 18.0;
        cam.fovY = lerpd(cam.fovY, wide * PI / 180.0, k);
      } else if (onFoot) {
        // Over the sights: right behind the weapon, and half as wide a view.
        Soldier at;
        at.p = view.foot.p;
        at.yaw = rig.yaw;
        const V3 eye = Duel::eyeOf(at);
        cam.eye = cam.eye + (eye - cam.eye) * k;
        cam.target = cam.target + ((eye - back * 30.0) - cam.target) * k;
        cam.fovY = lerpd(cam.fovY, (weaponPick == WP_AR ? 30.0 : (weaponPick == WP_RPG ? 28.0 : 40.0)) * PI / 180.0, k);
      }
    }
    hud.scope = scoped ? scopeZoom : 0.0;
    hud.scopeZero = scopeZero;
    if (scoped) {
      const V3 back{std::cos(rig.yaw) * std::cos(rig.pitch), std::sin(rig.yaw) * std::cos(rig.pitch), std::sin(rig.pitch)};
      Soldier eyeAt;
      eyeAt.p = view.foot.p;
      eyeAt.yaw = rig.yaw;
      eyeAt.prone = view.foot.prone;
      cam.eye = Duel::eyeOf(eyeAt);
      cam.target = cam.eye - back * 100.0;
      cam.up = {0, 0, 1};
      cam.fovY = (60.0 / scopeZoom) * PI / 180.0;
    }
    if (gunKick > 0.002) {
      // The picture jumps up and shakes, and comes back in a moment. Through a scope it would leave the view otherwise.
      const double amount = scoped ? 0.035 * 6.0 / scopeZoom : 0.05;
      const double range = len(cam.target - cam.eye);
      const double rattle = std::sin(tNow * 83.0) * 0.4 * gunKick;
      cam.target += V3{0, 0, 1} * (range * amount * gunKick) + V3{std::cos(rig.yaw + 1.57), std::sin(rig.yaw + 1.57), 0} * (range * amount * rattle);
      if (!scoped) cam.eye += norm(cam.eye - cam.target) * (0.18 * gunKick);
      cam.fovY *= 1.0 + 0.05 * gunKick;
      gunKick *= std::exp(-dt * 7.0);
    } else {
      gunKick = 0;
    }
    if (scriptMode == 49 && (inTank || view.missileLive)) {
      // Beside a guided missile in the air.
      if (view.missileLive) {
        cam.target = view.missileP;
        cam.eye = view.missileP + V3{5.0, 5.0, 1.5};
        cam.up = {0, 0, 1};
        cam.fovY = 45.0 * PI / 180.0;
      }
    }
    if (scriptMode == 53) {
      const V3 mid = G.world.base(Duel::teamOf(slot)).centre + V3{std::cos(G.world.base(Duel::teamOf(slot)).facing), std::sin(G.world.base(Duel::teamOf(slot)).facing), 0} * 70.0;
      cam.target = mid + V3{0, 0, 40.0};
      cam.eye = view.foot.p + V3{0, 45.0, 12.0};
      cam.up = {0, 0, 1};
      cam.fovY = 62.0 * PI / 180.0;
    }
    if (scriptMode == 51) {
      // Close by, to see someone fall.
      const double a = 0.9 + tNow * 0.15;
      cam.target = view.foot.p + V3{0, 0, 0.5};
      cam.eye = cam.target + V3{std::cos(a) * 4.5, std::sin(a) * 4.5, 1.8};
      cam.up = {0, 0, 1};
      cam.fovY = 55.0 * PI / 180.0;
    }
    if (scriptMode == 48 && inVehicle) {
      // Beside a car on the move, to see its wheels go round.
      cam.target = view.carCog;
      cam.eye = view.carCog + view.carLeft * 4.0 + view.carFwd * 0.5 + V3{0, 0, 0.3};
      cam.up = {0, 0, 1};
      cam.fovY = 50.0 * PI / 180.0;
    }
    if (scriptMode == 45) {
      const double a = 0.6 + double(shotCount) * 0.9;
      cam.target = view.foot.p + V3{0, 0, 0.6} + V3{-std::cos(rig.yaw), -std::sin(rig.yaw), 0} * (view.foot.prone * 0.8);
      cam.eye = cam.target + V3{std::cos(a) * 3.0, std::sin(a) * 3.0, 0.6};
      cam.fovY = 55.0 * PI / 180.0;
    }
    hud.motionBlur = gSet.motionBlur;
    hud.damageView = gSet.damageView && inTank;
    hud.stats = gSet.stats;
    hud.farShadows = gSet.farShadows;
    if (scriptMode == 69) {
      // The small guns from close by: from the side and from in front, each on its own hull.
      static const int kSlots[3] = {1, 3, 9};
      const int i = std::min(shotCount, 5);
      const V3 at = G.world.base(0).slot(kSlots[i / 2]);
      cam.eye = i % 2 == 0 ? at + V3{3.2, 6.0, 2.6} : at + V3{9.0, 2.6, 3.0};
      cam.target = at + V3{2.6, 0, 2.3};
      cam.up = {0, 0, 1};
      cam.fovY = 42.0 * PI / 180.0;
      panelKind = PANEL_NONE;
      hud.panel.open = false;
    }
    if (scriptMode == 43) {
      // A tour of the map, without a server: the yard, the command centre, a bridge, a barrier, and from above.
      const World::Base& b = G.world.base(0);
      const V3 bridge = G.world.bridges().empty() ? V3{} : G.world.bridges()[1 % G.world.bridges().size()];
      const V3 eyes[7][2] = {
          {b.centre + V3{55, -45, 16}, b.centre + V3{5, 0, 2}},
          {b.centre + V3{-26 + 30, -18, 8}, b.centre + V3{-26, 0, 4}},
          {b.slot(1) + V3{14, 12, 4}, b.slot(2) + V3{0, 0, 1.5}},
          {bridge + V3{-70, 45, 22}, bridge},
          {World::houseInside(G.world.houses()[0], 4.5, -14, G.world.houseSites()[0].z + 3.5), World::houseInside(G.world.houses()[0], 4.5, 3, G.world.houseSites()[0].z + 1.5)},
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
    if (scriptMode == 47) {
      // Looking at the mountains over the map's edge while turning very slowly: anything that flickers shows up as a
      // difference between one picture and the next.
      panelKind = PANEL_NONE;
      hud.panel.open = false;
      cam.eye = V3{-300, -150, 60};
      const double a = 2.4 + 0.0004 * double(shotCount);
      cam.target = cam.eye + V3{std::cos(a), std::sin(a), 0.03} * 100.0;
      cam.up = {0, 0, 1};
      cam.fovY = 45.0 * PI / 180.0;
    }
    if (scriptMode == 57 && inMatch) {
      // A ghillie suit out on the grass, and what grows around it.
      cam.target = view.foot.p + V3{0, 0, 0.8};
      cam.eye = cam.target + V3{3.5, 3.5, 1.2};
      cam.up = {0, 0, 1};
      cam.fovY = 55.0 * PI / 180.0;
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
    {
      // What has no picture of its own yet gets a loading screen: on the way in, waiting for the others, and a new match.
      char title[128] = "", detail[160] = "";
      double progress = -1;
      const char* back = padActive ? "B: abbrechen" : "ESC: abbrechen";
      if (netState == Game::NET_CONNECTING) {
        snprintf(title, sizeof title, "Verbinde mit %s:%d", gServerHost.c_str(), gServerPort);
        snprintf(detail, sizeof detail, "%s", back);
      } else if (netState == Game::NET_IN && phase == Duel::WAITING) {
        snprintf(title, sizeof title, "%s", view.damageMsg.empty() ? "Warte auf Spieler" : view.damageMsg.c_str());
        snprintf(detail, sizeof detail, "Das Match beginnt, sobald alle da sind  ·  %s", back);
      } else if (tNow < newMatchUntil) {
        snprintf(title, sizeof title, "Neues Match");
        progress = 1.0 - (newMatchUntil - tNow) / 0.9;
      }
      loadingShown = title[0] != 0;
      if (w > 0 && h > 0) {
        if (loadingShown) renderer.drawLoading(w, h, title, detail, tipNow(), progress);
        else renderer.draw(view, cam, hud, w, h, dt);
      }
    }
    if (getenv("BEANNG_PROFILE")) {
      static int n = 0;
      if (++n % 30 == 0) fprintf(stderr, "draw %.1f ms, before it %.1f ms\n", std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - drawStart).count(), std::chrono::duration<double, std::milli>(drawStart - now).count());
    }

    // How evenly what the camera follows moves from frame to frame, in scripted runs: 0 is perfectly smooth.
    static V3 jerkLast, jerkStep;
    static double jerkSum = 0, jerkFrames = 0;
    if (!shotDir.empty()) {
      const V3 stepNow = view.carCog - jerkLast;
      if (len(stepNow) < 3.0) {
        jerkSum += len(stepNow - jerkStep);
        jerkFrames += 1;
      }
      jerkLast = view.carCog;
      jerkStep = stepNow;
    }
    // Scripted screenshot series.
    if (!shotDir.empty()) {
      static double nextShot = 2.0;
      if (tNow >= nextShot) {
        char path[512];
        snprintf(path, sizeof path, "%s/shot_%02d.png", shotDir.c_str(), shotCount++);
        renderer.savePng(path, w, h);
        printf("fps %.0f t=%.1f v=%.0f km/h pos %.0f %.0f %.1f Phase %d Fahrzeug %d Munition %d Leben %.0f Gas %.2f Bremse %.2f Ruck %.1f mm Blick %.2f\n", hud.fps, tNow, view.speedKmh, view.carCog.x, view.carCog.y, view.carCog.z, phase, myVehicle, view.tankAmmo, view.foot.hp, view.throttle, view.brake, jerkFrames > 0 ? jerkSum / jerkFrames * 1000.0 : 0.0, rig.pitch);
        jerkSum = jerkFrames = 0;
        fflush(stdout);
        nextShot += scriptMode == 47 ? 0.3 : (scriptMode == 45 || scriptMode == 51 ? 0.6 : (scriptMode == 53 ? 0.4 : 1.0));
        const int enough = scriptMode == 69 ? 6 : scriptMode == 43 ? 7 : (scriptMode == 44 ? 2 : (scriptMode == 47 ? 6 : 60));
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
    if (!strcmp(argv[i], "--log")) {
      char path[64];
      snprintf(path, sizeof path, "/tmp/beanng-client-%d.log", int(getpid()));
      gPlayLog = fopen(path, "w");
      printf("Log: %s\n", path);
    }
    if (!strcmp(argv[i], "--wav") && i + 1 < argc) wavPath = argv[++i];
    if (!strcmp(argv[i], "--turret") && i + 1 < argc) gPickTurret = std::clamp(atoi(argv[++i]), 0, World::kBaseSlots - 1);
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
  if (scriptMode == 61 || scriptMode == 62) {
    SDL_VirtualJoystickDesc desc;
    SDL_INIT_INTERFACE(&desc);
    desc.type = SDL_JOYSTICK_TYPE_GAMEPAD;
    desc.naxes = SDL_GAMEPAD_AXIS_COUNT;
    desc.nbuttons = SDL_GAMEPAD_BUTTON_COUNT;
    desc.name = "Testcontroller";
    gTestPad = SDL_OpenJoystick(SDL_AttachVirtualJoystick(&desc));
    printf("Testcontroller: %s\n", gTestPad ? "da" : SDL_GetError());
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
