// BeanNG: game loop, input, cameras and the physics thread.
#include <SDL3/SDL.h>
#include <epoxy/gl.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "audio.h"
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
constexpr double kPhysDt = 1.0 / 2000.0;
constexpr int kMaxStepsPerSlice = 80;  // never spiral: if we fall behind, time simply slows down

V3 kSpawn{-90, 0, 0};

// Everything the input thread hands to the physics thread.
enum VehicleKind { VK_XANTIA, VK_LEOPARD, VK_ONE1, VK_COUNT };

struct SharedInput {
  Tuning tuning;
  double tankReload = 4;
  Controls c;
  bool reset = false, repair = false, resetWorld = false, paused = false;
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
  int kind = VK_XANTIA;
  Tank tank, tankStart;

  std::mutex inputMutex;
  SharedInput input;
  std::mutex snapMutex;
  Snapshot snap;
  std::atomic<bool> running{true};
  std::atomic<bool> resetVisuals{false};
  std::atomic<bool> debugWheel{false};
  std::atomic<bool> debugWing{false};
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

  // ---- Damage diagram. Each part reads the beams of its node group: how much of it has torn and how
  // far the rest has been bent out of shape.
  struct Gauge {
    PartState show;
    std::vector<u32> beams;
    double fullStrain = 0.08;  // average bend counted as a write-off
    int wheel = -1;
    int flag = 0;  // 1 engine, 2 gearbox, 3 radiator, 4 tank
  };
  std::vector<Gauge> gauges;
  int gaugeTick = 0;

  void buildGauges() {
    gauges.clear();
    const std::vector<u32>* group[32] = {};
    std::vector<int> nodePart;
    auto nodesOf = [&](const char* name) -> const std::vector<u32>* {
      for (const NodeGroup& g : rig.groups)
        if (g.name == name) return &g.nodes;
      return nullptr;
    };
    (void)group;
    auto addPart = [&](const char* label, float x, float y, float w, float h, const char* name, double x0, double x1, double fullStrain, int flag) {
      Gauge g;
      g.show = {label, x, y, w, h, 0.f};
      g.fullStrain = fullStrain;
      g.flag = flag;
      const std::vector<u32>* nodes = name ? nodesOf(name) : nullptr;
      if (nodes) {
        std::vector<u8> in(simStart.pos.size(), 0);
        for (u32 n : *nodes) {
          const double design = rig.noseX - (simStart.rest[n].x - kSpawn.x);
          if (design >= x0 && design < x1) in[n] = 1;
        }
        for (size_t i = 0; i < simStart.beams.size(); i++) {
          const Beam& b = simStart.beams[i];
          if (in[b.a] && in[b.b]) g.beams.push_back(u32(i));
        }
      }
      gauges.push_back(std::move(g));
    };
    constexpr double kAll0 = -9, kAll1 = 99;
    if (kind == VK_LEOPARD) {
      addPart("WANNE", 18, 34, 64, 170, "hull", kAll0, kAll1, 0.06, 0);
      addPart("TURM", 30, 84, 40, 70, "turret", kAll0, kAll1, 0.06, 0);
      addPart("ROHR", 47, 4, 6, 80, "gun", kAll0, kAll1, 0.04, 0);
      for (int s = 0; s < 2; s++) {
        Gauge g;
        g.show = {nullptr, s == 0 ? 4.f : 84.f, 34.f, 12.f, 170.f, 0.f};
        gauges.push_back(g);
      }
      return;
    }
    if (kind == VK_ONE1) {
      addPart(nullptr, 8, 2, 84, 46, "body", kAll0, 1.0, 0.08, 0);
      addPart(nullptr, 2, 48, 96, 44, "body", 1.0, 1.75, 0.08, 0);
      addPart(nullptr, 2, 92, 11, 62, "door_l", kAll0, kAll1, 0.08, 0);
      addPart(nullptr, 87, 92, 11, 62, "door_r", kAll0, kAll1, 0.08, 0);
      addPart(nullptr, 14, 92, 72, 62, "body", 1.75, 2.95, 0.04, 0);
      addPart(nullptr, 2, 155, 96, 60, "body", 2.95, 99, 0.08, 0);
      addPart(nullptr, 6, 216, 88, 18, "wing", kAll0, kAll1, 0.06, 0);
      addPart(nullptr, 24, 6, 52, 9, "radiator", kAll0, kAll1, 0.06, 3);
      addPart(nullptr, 24, 138, 52, 14, "tank", kAll0, kAll1, 0.05, 4);
      addPart(nullptr, 26, 156, 48, 32, "engine", kAll0, kAll1, 0.05, 1);
      addPart(nullptr, 34, 189, 32, 22, "gearbox", kAll0, kAll1, 0.05, 2);
      const float wx2[2] = {0.f, 85.f};
      for (int i = 0; i < 4 && i < int(car.wheels.size()); i++) {
        const Wheel& w = car.wheels[size_t(i)];
        Gauge g;
        g.show = {nullptr, w.left ? wx2[0] : wx2[1], w.front ? 24.f : 168.f, 15.f, 34.f, 0.f};
        g.wheel = i;
        gauges.push_back(g);
      }
      return;
    }
    // Plan view, nose at the top.
    addPart(nullptr, 12, 2, 76, 16, "bumper_f", kAll0, kAll1, 0.10, 0);
    addPart(nullptr, 14, 19, 72, 52, "hood", kAll0, kAll1, 0.10, 0);
    addPart(nullptr, 2, 19, 11, 52, "body", kAll0, 1.3, 0.08, 0);   // front wings, the nose structure
    addPart(nullptr, 87, 19, 11, 52, "body", kAll0, 1.3, 0.08, 0);
    addPart(nullptr, 16, 72, 68, 14, "windscreen", kAll0, kAll1, 0.03, 0);
    addPart(nullptr, 2, 72, 13, 44, "door_fl", kAll0, kAll1, 0.10, 0);
    addPart(nullptr, 85, 72, 13, 44, "door_fr", kAll0, kAll1, 0.10, 0);
    addPart(nullptr, 2, 117, 13, 42, "door_rl", kAll0, kAll1, 0.10, 0);
    addPart(nullptr, 85, 117, 13, 42, "door_rr", kAll0, kAll1, 0.10, 0);
    addPart("DACH", 16, 87, 68, 72, "body", 1.3, 3.3, 0.06, 0);
    addPart(nullptr, 2, 160, 96, 34, "body", 3.3, 99, 0.08, 0);     // rear quarter and floor
    addPart(nullptr, 14, 164, 72, 54, "tailgate", kAll0, kAll1, 0.10, 0);
    addPart(nullptr, 12, 220, 76, 16, "bumper_r", kAll0, kAll1, 0.10, 0);
    // Inside parts, on top.
    addPart("KUEHL", 22, 21, 56, 9, "radiator", kAll0, kAll1, 0.06, 3);
    addPart("MOTOR", 22, 31, 36, 30, "engine", kAll0, kAll1, 0.05, 1);
    addPart("GETR", 59, 31, 21, 30, "gearbox", kAll0, kAll1, 0.05, 2);
    addPart("TANK", 26, 168, 48, 18, "tank", kAll0, kAll1, 0.05, 4);
    // The wheels, at their corners.
    const float wx[2] = {0.f, 86.f};
    for (int i = 0; i < 4 && i < int(car.wheels.size()); i++) {
      const Wheel& w = car.wheels[size_t(i)];
      Gauge g;
      g.show = {nullptr, w.left ? wx[0] : wx[1], w.front ? 30.f : 168.f, 14.f, 34.f, 0.f};
      g.wheel = i;
      gauges.push_back(g);
    }
  }

  void updateGauges(Snapshot& s) {
    s.parts.clear();
    for (Gauge& g : gauges) {
      double d = 0;
      if (!g.beams.empty()) {
        double strain = 0;
        int broken = 0;
        for (u32 id : g.beams) {
          const Beam& b = sim.beams[id];
          if (b.flags & BF_BROKEN) broken++;
          else strain += std::fabs(b.L0 - b.Lorig) / b.Lorig;
        }
        const double n = double(g.beams.size());
        d = broken / n * 2.5 + strain / n / g.fullStrain;
      }
      if (g.wheel >= 0) {
        const Wheel& w = car.wheels[size_t(g.wheel)];
        float dent = 0;
        for (float v : w.dent) dent = std::max(dent, v);
        d = std::max(double(dent) / 0.04, std::hypot(w.runoutX, w.runoutZ) / 0.02);
        if (w.deflate > 0) d = std::max(d, 0.7);
        if (!w.attached) d = 1.0;
      }
      if (kind != VK_LEOPARD) {
        if (g.flag == 1 && car.dmg.seized) d = 1.0;
        if (g.flag == 1 && car.dmg.oilLeak) d = std::max(d, 0.6);
        if (g.flag == 2 && (car.dmg.gearbox || car.dmg.bellHousing)) d = 1.0;
        if (g.flag == 3 && car.dmg.radiator) d = std::max(d, 0.8);
        if (g.flag == 4 && car.dmg.tank) d = std::max(d, 0.8);
      }
      PartState p = g.show;
      p.damage = float(d);
      s.parts.push_back(p);
    }
  }

  void fillSnapshot(double load) {
    std::lock_guard<std::mutex> lock(snapMutex);
    Snapshot& s = snap;
    if ((gaugeTick++ & 15) == 0) updateGauges(s);
    s.pos = sim.pos;
    s.beamForce.resize(sim.objs[0].beamCount);
    for (u32 i = 0; i < sim.objs[0].beamCount; i++) {
      const Beam& b = sim.beams[i];
      s.beamForce[i] = (b.flags & BF_BROKEN) ? 0.f : (b.force == 0.f ? 1e-3f : b.force);
    }
    s.carResets = sim.carResets;
    s.events.insert(s.events.end(), sim.events.begin(), sim.events.end());
    if (s.events.size() > 2000) s.events.clear();
    sim.events.clear();
    if (s.scrapes.size() < 2000) s.scrapes.insert(s.scrapes.end(), sim.scrapes.begin(), sim.scrapes.end());
    sim.scrapes.clear();
    if (s.craters.size() != world.craters().size()) s.craters = world.craters();
    s.physLoad = load;
    s.awake = int(sim.active.size());
    s.objectCount = int(sim.objs.size());
    if (s.splitGeneration != sim.splitGeneration) {
      s.splitGeneration = sim.splitGeneration;
      s.nodeObj = sim.nodeObj;
    }
    if (kind == VK_LEOPARD) {
      s.tank = true;
      s.carCog = tank.cog;
      s.carFwd = tank.fwd;
      s.carUp = tank.up;
      s.carLeft = tank.left;
      s.carVel = tank.vCog;
      for (int i = 0; i < 2; i++) {
        tank.trackPath(sim, i, s.trackPath[i]);
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
      s.speedKmh = tank.speed * 3.6;
      s.rpm = tank.rpm();
      s.rpmMax = 2500;
      s.rpmRed = 2300;
      s.throttle = tank.throttleAct;
      s.brake = tank.brakeAct;
      s.steer = tank.steerAct;
      s.latG = tank.latAcc / 9.81;
      s.lonG = tank.lonAcc / 9.81;
      s.plastic = sim.plasticEnergy;
      s.gear = tank.gear;
      s.autoBox = tank.autoBox;
      s.running = true;
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
      return;
    }
    s.conventional = car.spec.conventional;
    s.boost = car.boost;
    s.rpmMax = car.spec.rpmMax;
    s.rpmRed = car.spec.rpmRed;
    s.oilTemp = car.dmg.oilTemp;
    s.oilLevel = car.oilLevel;
    s.absOn = car.tune.abs;
    s.espOn = car.tune.esp;
    s.tcsOn = car.tune.tcs;
    s.absActive = car.absActive;
    s.espActive = car.espActive;
    s.tcsActive = car.tcsActive;
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
    s.scorch = car.exploded ? clampd(car.burnTime / 3.0, 0.15, 1.0) : 0.0;
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
        input.c.fire = false;
        input.reset = input.repair = input.resetWorld = false;
        car.tune = input.tuning;
        tank.reloadTime = input.tankReload;
      }
      if (debugWing.exchange(false) && !car.wingMountBeams.empty()) sim.sever(car.wingMountBeams[0]);
      if (debugWheel.exchange(false) && kind != VK_LEOPARD && car.wheels[0].deflate == 0) {
        Wheel& w = car.wheels[0];
        w.deflate = 0.001;
        w.dent[3] = 0.02f; w.dent[4] = 0.04f; w.dent[5] = 0.045f; w.dent[6] = 0.03f; w.dent[7] = 0.012f;
        w.runoutX = 0.012f;
      }
      // The vehicle goes back; the world keeps every fence it knocked down and every hole it left.
      const auto restoreVehicle = [&] {
        sim.restoreObjects(simStart, kind == VK_LEOPARD ? 2 : 1);
        car = carStart;
        tank = tankStart;
        // If the start itself has been blown into a crater, the car still has to come down on the ground.
        const double dip = world.height(kSpawn.x, kSpawn.y) - kSpawn.z;
        if (dip > 0.02) {
          const Obj& o0 = sim.objs[0];
          for (u32 i = o0.nodeStart; i < o0.nodeStart + o0.nodeCount; i++) sim.pos[i].z += dip;
        }
        resetVisuals = true;
      };
      if (in.resetWorld) {
        world.clearCraters();
        sim = simStart;
        car = carStart;
        tank = tankStart;
        resetVisuals = true;
      } else if (in.reset || (kind == VK_LEOPARD && in.repair)) {
        restoreVehicle();
      }
      // Pieces that have flown off cannot just be put back on: a torn-up car starts over.
      if (in.resetWorld || in.reset) {
        // Done above.
      } else if (in.repair && (car.exploded || sim.splitGeneration != simStart.splitGeneration)) {
        restoreVehicle();
      } else if (in.repair && kind != VK_LEOPARD) {
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
        if (kind == VK_LEOPARD) tank.update(sim, in.c, kPhysDt);
        else car.update(sim, in.c, kPhysDt);
        sim.finish(kPhysDt);
        accumulator -= kPhysDt;
        steps++;
        // Edge-triggered inputs apply to one step only.
        in.c.shiftUp = in.c.shiftDown = in.c.toggleAuto = in.c.toggleSport = false;
        in.c.cycleHeight = in.c.toggleIgnition = in.c.toggleActiva = false;
        in.c.requestGear = 99;
        in.c.fire = false;
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

    f.fire = s.scorch > 0 ? float(clampd(s.scorch, 0, 1)) : 0.f;
    if (s.conventional) {
      f.cylinders = 8;
      f.exhaustPitch = 1.15f;
      f.turbo = float(s.boost);
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

// One vehicle in one world. Returns the vehicle to continue with, or -1 to quit.
static int playSession(int kind, bool openMenu) {
  kSpawn = gSpawnBase;
  // Every session has its own GL context: closing it frees everything the renderer ever made.
  SDL_GLContext ctx = SDL_GL_CreateContext(window);
  if (!ctx) {
    fprintf(stderr, "OpenGL: %s\n", SDL_GetError());
    return -1;
  }
  glClearColor(.05f, .05f, .06f, 1.f);
  glClear(GL_COLOR_BUFFER_BIT);
  SDL_GL_SwapWindow(window);
  auto* game = new Game();
  Game& G = *game;
  printf("Baue Welt und Fahrzeug...\n");
  G.kind = kind;
  G.world.build();
  G.sim.world = &G.world;
  // Script 15 starts on the country road, on its long southern stretch.
  if (scriptMode == 15) {
    const RoadPoint& p = G.world.roads()[0].pts[1200];
    kSpawn.x = p.x;
    kSpawn.y = p.y;
  }
  // Script 19 starts straight in front of a tree.
  if (scriptMode == 19) {
    const V3 t = G.world.trees()[treeIndex];
    kSpawn.x = t.x - treeRun;
    kSpawn.y = t.y + 0.25;
  }
  kSpawn.z = G.world.height(kSpawn.x, kSpawn.y);
  // Script 24 stands on a road, so that the crater goes through the tarmac.
  if (kind == VK_LEOPARD) G.tank = buildLeopard(G.sim, kSpawn, &G.rig);
  else if (kind == VK_ONE1) G.car = buildOne1(G.sim, kSpawn, &G.rig);
  else G.car = buildXantia(G.sim, kSpawn, 0.0, &G.rig);
  populateWorld(G.sim, G.world);
  G.sim.finalize();
  if (kind == VK_LEOPARD) G.tank.settle(G.sim, 3.0);
  else settleVehicle(G.sim, G.car, 4.0);
  G.sim.craterWorld = &G.world;
  G.simStart = G.sim;
  G.carStart = G.car;
  G.tankStart = G.tank;
  const char* meshFile = kind == VK_LEOPARD ? "assets/leopard.mesh" : (kind == VK_ONE1 ? "assets/one1.mesh" : "assets/xantia.mesh");
  if (!G.mesh.load(meshFile)) {
    fprintf(stderr, "%s fehlt. Starte das Spiel mit ./run.sh, das erzeugt die Modelle.\n", meshFile);
    return -1;
  }
  G.mesh.noseX = G.rig.noseX;
  G.buildGauges();
  // Bind to the as-built pose: that is what the mesh was modelled against.
  G.mesh.bind(G.simStart.rest, G.rig.groups, kSpawn);
  printf("%zu Nodes, %zu Beams, %zu Objekte\n", G.sim.pos.size(), G.sim.beams.size(), G.sim.objs.size());

  SDL_GL_SetSwapInterval(gSet.vsync ? 1 : 0);
  printf("OpenGL %s auf %s\n", glGetString(GL_VERSION), glGetString(GL_RENDERER));
  glEnable(GL_MULTISAMPLE);

  Renderer renderer;
  // The renderer reads only topology from this immutable copy (rest pose, triangle list, beam end
  // points). Everything that moves arrives through snapshots, so it never races the physics.
  renderer.init(G.world, G.simStart, G.rig, G.mesh, kSpawn);

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
  rig.tank = kind == VK_LEOPARD;
  if (rig.tank) {
    rig.yaw = PI;
    rig.pitch = 0.20;
    rig.dist = 8.0;
  }
  Camera cam;
  HudOptions hud;
  hud.crosshair = rig.tank;
  int nextVehicle = -1;
  // Panels: 0 none, 1 vehicle menu, 2 settings.
  enum { PANEL_NONE, PANEL_VEHICLE, PANEL_CAR, PANEL_GAME, PANEL_MAIN, PANEL_KEYS };
  int panelKind = openMenu ? PANEL_MAIN : PANEL_NONE;
  int panelSel = 0;
  bool fromMain = false;      // a page opened from the main menu goes back to it
  bool menuSpin = openMenu;   // the first time, the vehicle slowly turns behind the menu
  Tuning tuning;
  double tankReload = 4;
  struct Item {
    std::string label, value;
    float fill;
    std::function<void(int)> change;
  };
  std::vector<Item> items;
  Controls panelEdges;
  bool panelRepair = false;
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
    bool doReset = false, doRepair = false, doResetWorld = false;

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
      panelSel = which == PANEL_VEHICLE ? kind : 0;
    };
    if (panelKind == PANEL_MAIN) {
      page(menuSpin ? "Losfahren" : "Weiterfahren", [&] { panelKind = PANEL_NONE; menuSpin = false; if (rig.tank) rig.yaw = PI; else rig.yaw = 0; });
      page("Fahrzeug wählen", [&] { openPage(PANEL_VEHICLE); });
      page("Fahrzeug-Einstellungen", [&] { openPage(PANEL_CAR); });
      page("Spiel-Einstellungen", [&] { openPage(PANEL_GAME); });
      page("Steuerung", [&] { openPage(PANEL_KEYS); });
      page("Welt zurücksetzen", [&] {
        doResetWorld = true;
        panelKind = PANEL_NONE;
      });
      page("Beenden", [&] { quit = true; });
    } else if (panelKind == PANEL_VEHICLE) {
      items.push_back({"Citroën Xantia Activa", "V6  ·  190 PS  ·  1,5 t", -1.f, [&](int) {}});
      items.push_back({"Leopard 1", "V10  ·  830 PS  ·  42 t", -1.f, [&](int) {}});
      items.push_back({"Koenigsegg One:1", "V8 Biturbo  ·  1360 PS  ·  1,36 t", -1.f, [&](int) {}});
    } else if (panelKind == PANEL_CAR) {
      if (kind != VK_LEOPARD) {
        toggle("ABS", tuning.abs, [&] { tuning.abs = !tuning.abs; });
        toggle("ESP", tuning.esp, [&] { tuning.esp = !tuning.esp; });
        toggle("ASR", tuning.tcs, [&] { tuning.tcs = !tuning.tcs; });
        if (kind == VK_XANTIA) {
          toggle("Activa", view.activa, [&] { panelEdges.toggleActiva = true; });
          toggle("Hydractive Sport", view.sport, [&] { panelEdges.toggleSport = true; });
          static const char* heights[3] = {"Niedrig", "Straße", "Hoch"};
          items.push_back({"Niveau", heights[std::clamp(view.heightMode, 0, 2)], -1.f, [&](int) { panelEdges.cycleHeight = true; }});
        }
        items.push_back({"Getriebe", view.autoBox ? "Automatik" : "Manuell", -1.f, [&](int) { panelEdges.toggleAuto = true; }});
        percent("Bremskraft", tuning.brake, 0.4, 1.6, 0.1);
        percent("Dämpfung", tuning.damper, 0.4, 2.0, 0.1);
        percent("Stabilisator", tuning.bar, 0.0, 2.0, 0.1);
        percent("Lenkübersetzung", tuning.steer, 0.6, 1.4, 0.05);
      } else {
        items.push_back({"Getriebe", view.autoBox ? "Automatik" : "Manuell", -1.f, [&](int) { panelEdges.toggleAuto = true; }});
        {
          char b[32];
          snprintf(b, sizeof b, "%.1f s", tankReload);
          items.push_back({"Nachladezeit", b, float(tankReload / 8.0), [&](int d) { tankReload = clampd(tankReload + d * 0.5, 0.5, 8.0); }});
        }
      }
      page("Fahrzeug reparieren", [&] { panelRepair = true; });
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
      key("Handbremse", "Leertaste");
      key("Schalten / Automatik", "Q E / T");
      key("Autobombe, Kanone", "F");
      key("Kamera", "C");
      key("Mauslenkung", "M");
      key("Zeitlupe / Pause", "[ ] / P");
      key("Fahrzeug-Einstellungen", "O");
      key("Fahrzeug wählen", "TAB");
      key("Fahrzeug zum Start / Reparieren", "R / F5");
      key("Beams / Röntgen / Screenshot", "F2 F3 F12");
      key("Hauptmenü", "ESC");
      key("Controller: Bombe / Handbremse / Gang", "X / A / LB RB");
    }
    auto choosePanel = [&] {
      if (panelKind == PANEL_VEHICLE) {
        if (panelSel != kind) {
          nextVehicle = panelSel;
          quit = true;
        }
        panelKind = PANEL_NONE;
        menuSpin = false;
        rig.yaw = rig.tank ? PI : 0;
      } else if (panelKind != PANEL_KEYS && panelSel < int(items.size())) {
        items[size_t(panelSel)].change(1);
      }
    };
    auto closePanel = [&] {
      saveSettings();
      if (panelKind != PANEL_MAIN && fromMain) {
        panelSel = panelKind == PANEL_VEHICLE ? 1 : (panelKind == PANEL_CAR ? 2 : (panelKind == PANEL_GAME ? 3 : 4));
        panelKind = PANEL_MAIN;
        fromMain = false;
        return;
      }
      panelKind = PANEL_NONE;
      menuSpin = false;
      if (rig.tank) rig.yaw = PI;
      else if (rig.yaw != 0 && rig.mode == CAM_CHASE) rig.yaw = 0;
    };
    auto panelMove = [&](int d) {
      if (!items.empty()) panelSel = (panelSel + d + int(items.size())) % int(items.size());
    };
    auto panelAdjust = [&](int d) {
      if ((panelKind == PANEL_CAR || panelKind == PANEL_GAME) && panelSel < int(items.size())) items[size_t(panelSel)].change(d);
    };

    SDL_Event e;
    while (SDL_PollEvent(&e)) {
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
            case SDLK_ESCAPE: case SDLK_O: case SDLK_TAB: closePanel(); break;
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
            if (panelKind == PANEL_CAR || panelKind == PANEL_GAME) panelAdjust(mxp < px + pwid * 0.65f ? -1 : 1);
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
            case SDL_GAMEPAD_BUTTON_LEFT_SHOULDER: edges.shiftDown = true; break;
            case SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER: edges.shiftUp = true; break;
            case SDL_GAMEPAD_BUTTON_NORTH: rig.mode = CamMode((rig.mode + 1) % CAM_COUNT); rig.yaw = 0; break;
            case SDL_GAMEPAD_BUTTON_WEST: edges.fire = true; break;
            case SDL_GAMEPAD_BUTTON_RIGHT_STICK: edges.toggleSport = true; break;
            case SDL_GAMEPAD_BUTTON_LEFT_STICK: panelKind = PANEL_CAR; panelSel = 0; fromMain = false; break;
            case SDL_GAMEPAD_BUTTON_GUIDE: panelKind = PANEL_MAIN; panelSel = 0; fromMain = false; break;
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
            case SDLK_ESCAPE: panelKind = PANEL_MAIN; panelSel = 0; fromMain = false; break;
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
            case SDLK_F: edges.fire = true; break;
            case SDLK_O: panelKind = PANEL_CAR; panelSel = 0; fromMain = false; break;
            case SDLK_TAB: panelKind = PANEL_VEHICLE; panelSel = kind; fromMain = false; break;
            case SDLK_F2: hud.debugBeams = !hud.debugBeams; break;
            case SDLK_F3: hud.xray = !hud.xray; break;
            case SDLK_F12: wantShot = true; break;
            default: break;
          }
          break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
          if (e.button.button == SDL_BUTTON_RIGHT) rmb = true;
          if (e.button.button == SDL_BUTTON_LEFT && rig.tank) {
            edges.fire = true;
          } else if (e.button.button == SDL_BUTTON_LEFT) {
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
          if (rig.tank) {
            const double sens = (rig.mode == CAM_HOOD ? 0.0009 : 0.0028) * gSet.mouse;
            rig.yaw -= e.motion.xrel * sens;
            rig.pitch = clampd(rig.pitch + e.motion.yrel * sens, -0.30, 1.2);
          } else if (rmb) {
            rig.yaw -= e.motion.xrel * 0.005 * gSet.mouse;
            rig.pitch = clampd(rig.pitch + e.motion.yrel * 0.004 * gSet.mouse, -0.25, 1.45);
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
    if (rig.tank) steer = keyDir;  // a tank has no speed-sensitive lock to protect

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
      if (rx == 0 && rig.mode == CAM_CHASE && !rmb && !rig.tank) rig.yaw *= std::max(0.0, 1.0 - dt * 2.0);
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

    // While a panel is open the pedals rest: its keys are the same ones.
    if (panelKind != 0) gas = brake = steer = hb = 0;
    {
      const bool wantRelative = rig.tank && panelKind == 0 && scriptMode == 0;
      if (wantRelative != relativeMouse) {
        SDL_SetWindowRelativeMouseMode(window, wantRelative);
        relativeMouse = wantRelative;
      }
    }
    sendC.toggleActiva |= panelEdges.toggleActiva;
    sendC.toggleSport |= panelEdges.toggleSport;
    sendC.cycleHeight |= panelEdges.cycleHeight;
    sendC.toggleAuto |= panelEdges.toggleAuto;
    panelEdges = Controls{};
    doRepair |= panelRepair;
    panelRepair = false;
    if (menuSpin) rig.yaw += dt * 0.25;
    const bool menuPause = panelKind == PANEL_MAIN || (fromMain && panelKind != PANEL_NONE);

    // Scripted runs for automated screenshots.
    bool scripted = false;
    bool scriptHand = false;
    double scriptBrake = 0;
    double scriptGas = 0, scriptSteer = 0;
    if (scriptMode > 0) {
      scripted = true;
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
        case 20:  // Leopard: drive, pivot, lay the gun and fire
          scriptGas = tNow > 2.0 && tNow < 9.0 ? 1.0 : 0.0;
          scriptSteer = tNow > 6.0 && tNow < 8.0 ? 0.6 : 0.0;
          scriptBrake = tNow > 9.0 && tNow < 11.0 ? 1.0 : 0.0;
          rig.mode = shotCount >= 17 ? CAM_HOOD : CAM_CHASE;
          rig.yaw = PI + 0.5 + 0.03 * tNow;
          rig.pitch = 0.08;
          if (tNow > 11.5 && std::fmod(tNow, 3.0) < dt) sendC.fire = true;
          break;
        case 27: {  // through the gauntlet and over the kerbs, watching the wing
          const double head = std::atan2(view.carFwd.y, view.carFwd.x);
          scriptGas = view.speedKmh < 70.0 ? 0.8 : 0.15;
          scriptSteer = clampd(-0.05 * view.carCog.y - 0.7 * head, -0.25, 0.25);
          break;
        }
        case 28:  // the wing comes off while driving: does it leave cleanly?
          scriptGas = view.speedKmh < 50.0 ? 0.6 : 0.1;
          if (tNow > 3.0 && !repairDone) {
            G.debugWing = true;
            repairDone = true;
          }
          break;
        case 29:  // through the fence and the wall, then back to the start: the wreckage must stay
          scriptGas = tNow < 7.0 ? 0.9 : 0.0;
          if (tNow > 8.0 && !repairDone) {
            doReset = true;
            repairDone = true;
          }
          scriptBrake = tNow > 8.0 ? 1.0 : 0.0;
          rig.mode = CAM_ORBIT;
          rig.dist = 9.0;
          rig.pitch = 0.35;
          rig.yaw = PI + 0.3;
          break;
        case 30:
          scriptGas = view.speedKmh < 55.0 ? 0.8 : 0.1;
          scriptSteer = tNow > 3.0 ? 0.5 : 0.0;
          break;
        case 23:  // switches vehicle in mid-game, as the menu does, and back out
          if (tNow > 4.0) {
            nextVehicle = kind == VK_XANTIA ? VK_LEOPARD : -1;
            quit = true;
          }
          break;
        case 24:  // the Activa's bomb on the road, then a fixed look at what it left
          if (tNow > 3.0 && !repairDone) {
            sendC.fire = true;
            repairDone = true;
          }
          break;
        case 21:  // the vehicle menu, then the settings
          panelKind = shotCount < 1 ? PANEL_MAIN : (shotCount < 2 ? PANEL_VEHICLE : (shotCount < 3 ? PANEL_CAR : (shotCount < 4 ? PANEL_GAME : PANEL_KEYS)));
          break;
        case 19:
          rig.mode = CAM_ORBIT;
          rig.dist = 14.0;
          rig.pitch = 0.32;
          rig.yaw = 2.3;
          scriptGas = view.speedKmh < treeSpeed && tNow > 2.0 ? 1.0 : 0.0;
          scriptSteer = 0;
          break;
        case 18:  // straight up the road, then the bomb goes off
          rig.mode = CAM_ORBIT;
          rig.dist = 13.0;
          rig.pitch = 0.2;
          rig.yaw = 0.9;
          scriptGas = tNow < 4.0 ? 0.5 : 0.0;
          scriptBrake = tNow > 4.5 ? 1.0 : 0.0;
          if (tNow > 6.0 && !repairDone) {
            sendC.fire = true;
            repairDone = true;
          }
          break;
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
      c.fire |= sendC.fire;
      c.requestGear = sendC.requestGear;
      G.input.reset |= doReset;
      G.input.repair |= doRepair;
      G.input.resetWorld |= doResetWorld;
      G.input.paused = paused || (menuPause && !menuSpin);
      G.input.timeScale = timeScale;
      G.input.grabNode = grabNode;
      G.input.grabTarget = grabTarget;
      G.input.tuning = tuning;
      G.input.tankReload = tankReload;
      if (rig.tank && scriptMode != 22) {
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
    if (doReset || doRepair) renderer.resetDamage();
    if (doResetWorld) {
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

    hud.panel.open = panelKind != 0;
    if (hud.panel.open) {
      static const char* titles[6] = {"", "Fahrzeug wählen", "Fahrzeug-Einstellungen", "Spiel-Einstellungen", "BeanNG", "Steuerung"};
      hud.panel.title = titles[panelKind];
      hud.panel.foot = panelKind == PANEL_MAIN || panelKind == PANEL_VEHICLE ? "Pfeile wählen  ·  ENTER bestätigen  ·  ESC zurück" : (panelKind == PANEL_KEYS ? "ESC zurück" : "Pfeile wählen und ändern  ·  ESC zurück");
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
    hud.mouseSteer = mouseSteer;
    hud.cameraName = kCamNames[rig.mode];
    feel.update(view, paused ? 0.0 : dt);
    if (doReset || doRepair) feel.lastPlastic = 0, feel.valid = false;
    rig.update(view, dt, cam, G.world, feel, gSet.shake);
    cam.fovY += gSet.fov * PI / 180.0;
    hud.motionBlur = gSet.motionBlur;
    hud.damageView = gSet.damageView;
    hud.stats = gSet.stats;
    hud.farShadows = gSet.farShadows;
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
    if (scriptMode == 22) {  // the Leopard from all round, gun straight ahead
      const double a = 0.4 + double(shotCount) * PI / 3.0;
      cam.target = view.carCog + V3{0, 0, 0.8};
      cam.eye = cam.target + V3{std::cos(a) * 9.5, std::sin(a) * 9.5, 1.9};
      cam.up = {0, 0, 1};
      cam.fovY = 55.0 * PI / 180.0;
    }
    if (scriptMode == 24) {
      const V3 at = kSpawn + V3{0, 0, 0.3};
      cam.target = at;
      cam.eye = at + V3{-13, 10, 8};
      cam.up = {0, 0, 1};
      cam.fovY = 55.0 * PI / 180.0;
    }
    if (scriptMode == 30) {  // circles at speed, watching the wing from behind and from the side in turn
      cam.target = view.carCog + view.carFwd * -1.9 + view.carUp * 0.5;
      cam.eye = cam.target + view.carFwd * -3.0 + view.carLeft * (shotCount % 2 ? 3.0 : 0.6) + view.carUp * 0.7;
      cam.up = {0, 0, 1};
      cam.fovY = 42.0 * PI / 180.0;
    }
    if (scriptMode == 25 || scriptMode == 26) {  // the car from outside: walk round it, close to the ground
      const double a = 0.55 + double(shotCount) * PI / 3.0;
      cam.target = view.carCog + V3{0, 0, 0.1};
      cam.eye = cam.target + V3{std::cos(a) * 5.6, std::sin(a) * 5.6, scriptMode == 25 ? 0.55 : 1.8};
      cam.up = {0, 0, 1};
      cam.fovY = 50.0 * PI / 180.0;
    }
    if (scriptMode == 27 || scriptMode == 28) {  // behind and above, to see the wing against the car
      cam.target = view.carCog + view.carFwd * -1.8 + view.carUp * 0.6;
      cam.eye = cam.target + view.carFwd * -4.0 + view.carLeft * 1.8 + view.carUp * 1.5;
      cam.up = {0, 0, 1};
      cam.fovY = 45.0 * PI / 180.0;
    }
    if (scriptMode == 19) {  // a fixed view of the tree
      const V3 t = G.world.trees()[treeIndex];
      cam.target = t + V3{0, 0, 3.5};
      cam.eye = t + V3{-9, -13, 4.5};
      cam.up = {0, 0, 1};
      cam.fovY = 62.0 * PI / 180.0;
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
        if (shotCount >= (burst ? 6 : (scriptMode == 9 || scriptMode == 22 || scriptMode == 25 || scriptMode == 26 ? 7 : (scriptMode == 12 ? 13 : 19)))) quit = true;
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
  G.running = false;
  physics.join();
  if (gamepad) SDL_CloseGamepad(gamepad);
  SDL_SetWindowRelativeMouseMode(window, false);
  SDL_GL_DestroyContext(ctx);
  delete game;
  return nextVehicle;
}

int main(int argc, char** argv) {
  preferDiscreteGpu();
  loadSettings();
  int vehicle = VK_XANTIA;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--shots") && i + 1 < argc) shotDir = argv[++i];
    if (!strcmp(argv[i], "--script") && i + 1 < argc) scriptMode = atoi(argv[++i]);
    if (!strcmp(argv[i], "--wav") && i + 1 < argc) wavPath = argv[++i];
    if (!strcmp(argv[i], "--tree") && i + 2 < argc) {
      treeIndex = size_t(atoi(argv[++i]));
      treeRun = atof(argv[++i]);
      if (i + 1 < argc && argv[i + 1][0] != '-') treeSpeed = atof(argv[++i]);
    }
    if (!strcmp(argv[i], "--spawn") && i + 2 < argc) {
      gSpawnBase.x = atof(argv[++i]);
      gSpawnBase.y = atof(argv[++i]);
    }
    if (!strcmp(argv[i], "--vehicle") && i + 1 < argc) {
      i++;
      vehicle = !strcmp(argv[i], "leopard") ? VK_LEOPARD : (!strcmp(argv[i], "one1") ? VK_ONE1 : VK_XANTIA);
    }
    if (!strcmp(argv[i], "--help")) {
      printf("beanng [--shots DIR --script N]\n");
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
  bool first = scriptMode == 0;
  int next = vehicle;
  while (next >= 0) {
    next = playSession(next, first);
    first = false;
  }
  if (scriptMode == 0) saveSettings();
  if (!wavPath.empty()) writeWav(wavPath, wavSamples);
  if (audioStream) SDL_DestroyAudioStream(audioStream);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
