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
  double omega = 0;
  double fz = 0;
  u8 surface = 0;
  V3 patch;
  V3 groundN{0, 0, -1};     // towards the road, world space
  double groundD = 10;      // distance from hub to road along groundN; large when airborne
  float dent[Wheel::kDentBins] = {};
  float runoutX = 0, runoutZ = 0;
};

// One piece of the damage diagram: where it is in a plan view of the vehicle, and how it is doing.
struct PartState {
  const char* label;  // nullptr for none
  float x, y, w, h;   // in a box of 100 x 240 units, front at the top
  float damage;       // 0 as new .. 1 destroyed
};

// Copied out of the physics thread once per rendered frame.
struct Snapshot {
  std::vector<V3> pos;
  std::vector<float> beamForce;  // 0 means broken, for the debug overlay
  WheelView wheels[4];
  std::vector<SimEvent> events;
  std::vector<Scrape> scrapes;
  V3 carCog, carFwd{1, 0, 0}, carUp{0, 0, 1}, carLeft{0, 1, 0}, carVel;
  u32 headNode = 0;
  u32 splitGeneration = 0;
  u32 carResets = 0;
  std::vector<V3> parkedTrackPath[2];  // the Leopard that stands about
  V3 parkedTrackLeft[2];
  V3 trackLeft[2];
  float parkedScorch[4] = {};
  double tankArmour = 1;
  bool tankDead = false;
  std::vector<u16> nodeObj;
  int objectCount = 0;

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
  std::vector<PartState> parts;
  double oilTemp = 95, oilLevel = 1;
  bool tank = false;
  std::vector<V3> trackPath[2];
  double trackPhase[2] = {};
  double trackSpeed[2] = {};
  double rpmMax = 7000, rpmRed = 6100;
  double turretYaw = 0, gunPitch = 0, reload = 0, reloadTime = 4;
  V3 muzzle, gunDir{1, 0, 0}, sight;
  bool absOn = true, espOn = true, tcsOn = true, absActive = false, espActive = false, tcsActive = false;
  std::vector<World::Crater> craters;
  bool conventional = false;  // a car without Citroen hydraulics
  double boost = 0;
  double scorch = 0;  // the car has burnt out, 0..1
  // For the sound.
  double engineLoad = 0, inputRpm = 0, clutchSlip = 0;
  bool starter = false, shifting = false;
  std::string damageMsg;
  double damageAge = 100;
};

struct Camera {
  V3 eye, target, up{0, 0, 1};
  double fovY = 60.0 * PI / 180.0;
  bool inside = false;  // looking out through the glass from inside the car
};

// A list on screen: the vehicle menu and the settings overlay.
struct UiPanel {
  bool open = false;
  std::string title, foot;
  std::vector<std::string> rows;
  std::vector<std::string> values;
  std::vector<float> fill;   // bar behind the value, 0..1, or negative for none
  int sel = 0;
};
// Where a panel sits, shared by drawing it and clicking on it.
inline void panelLayout(int w, int h, size_t rows, float& x, float& y, float& pw, float& rowH) {
  const float H = float(h), W = float(w);
  const float scale = H / 720.f < 1.f ? 1.f : float(int(H / 720.f + 0.5f));
  pw = 620 * scale;
  rowH = 31 * scale;
  x = (W - pw) * 0.5f;
  y = (H - (float(rows) + 3.6f) * rowH) * 0.5f + 2.0f * rowH;
}

struct HudOptions {
  UiPanel panel;
  bool crosshair = false;
  bool damageView = true;
  bool xray = false, debugBeams = false, mouseSteer = false, gamepad = false;
  bool motionBlur = true, stats = true, farShadows = true;
  std::string gamepadName, cameraName, gpuName;
  double fps = 0;
};

class Renderer {
 public:
  ~Renderer();
  bool init(const World& world, const Sim& sim, const CarRig& rig, CarMesh& mesh, V3 spawn);
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
