// OpenGL 3.3 renderer. Reads a snapshot of node positions, never touches the live simulation.
#pragma once
#include <string>
#include <vector>
#include "vehicle.h"

struct WheelView {
  V3 hub, axis, up;
  double theta = 0;
  bool left = true;
  double deflate = 0;
  double skid = 0;
  u8 surface = 0;
  V3 patch;
  V3 groundN{0, 0, -1};     // towards the road, world space
  double groundD = 10;      // distance from hub to road along groundN; large when airborne
  float dent[Wheel::kDentBins] = {};
  float runoutX = 0, runoutZ = 0;
};

// Copied out of the physics thread once per rendered frame.
struct Snapshot {
  std::vector<V3> pos;
  std::vector<float> beamForce;  // 0 means broken, for the debug overlay
  WheelView wheels[4];
  std::vector<SimEvent> events;
  V3 carCog, carFwd{1, 0, 0}, carUp{0, 0, 1}, carLeft{0, 1, 0}, carVel;
  u32 headNode = 0;

  // HUD values.
  double speedKmh = 0, rpm = 0, throttle = 0, brake = 0, steer = 0, coolant = 0, accum = 0;
  double lhm = 0, fuel = 0, oil = 0, clutchPos = 0;
  double roll = 0, latG = 0, lonG = 0, peakG = 0, plastic = 0, ram = 0, physLoad = 0;
  int gear = 1, heightMode = 1, awake = 0;
  bool autoBox = true, soft = true, sport = false, activa = true, running = true;
  bool gearboxBad = false, tankBad = false, oilBad = false, charging = false;
  bool radiator = false, hydraulics = false, mount = false, steering = false, seized = false;
  bool flat[4] = {};
  double timeScale = 1;
  double realtime = 1;   // simulated seconds per real second
  bool paused = false;
  std::string damageMsg;
  double damageAge = 100;
};

struct Camera {
  V3 eye, target, up{0, 0, 1};
  double fovY = 60.0 * PI / 180.0;
};

struct HudOptions {
  bool help = false, xray = false, debugBeams = false, mouseSteer = false, gamepad = false;
  std::string gamepadName, cameraName, gpuName;
  double fps = 0;
};

class Renderer {
 public:
  bool init(const World& world, const Sim& sim, const CarRig& rig, const CarMesh& mesh);
  void resetDamage();
  void clearSkidMarks();
  void draw(const Snapshot& s, const Camera& cam, const HudOptions& hud, int w, int h, double dt);
  // Returns the nearest node under a screen position, for the mouse grabber.
  int pickNode(const Snapshot& s, const Camera& cam, int w, int h, double mx, double my, double maxPx) const;
  V3 unprojectOnPlane(const Camera& cam, int w, int h, double mx, double my, V3 planePoint) const;
  bool savePng(const std::string& path, int w, int h) const;

 private:
  struct Impl;
  Impl* d = nullptr;
};
