// OpenGL 3.3 renderer. Reads a snapshot of node positions, never touches the live simulation.
#pragma once
#include <algorithm>
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
// What the sniper's scope shows: the plain picture, what is warm, or where to hit.
enum ScopeView { SV_PLAIN, SV_THERMAL, SV_WEAK };

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
  // Every tank on the map draws its own tracks.
  struct TankTrack {
    std::vector<V3> path[2];
    V3 left[2];
    double phase[2] = {}, speed[2] = {};
  };
  std::vector<TankTrack> tankTracks;
  // What leaves a mark on the ground as it rolls: a point on it, the way across, and how wide.
  struct GroundMark {
    V3 at, across;
    float halfWidth;
  };
  std::vector<GroundMark> groundMarks;
  // What presses down what grows under it: a moving vehicle's footprint.
  struct GroundBox {
    V3 at, fwd, left;
    float halfL, halfW;
  };
  std::vector<GroundBox> groundBoxes;
  // Which of the parked instances is what the camera follows: it moves with it, the world does not.
  int selfInstance = -1;
  bool selfOnFoot = false;  // the camera hangs on one's own figure, which so stands still in the picture
  int selfTrack = -1;
  V3 trackLeft[2];
  float parkedScorch[64] = {};
  float parkedHeat[64] = {};  // how warm each is through a thermal sight, 0..1
  struct WeakSpot {
    V3 p;
    float radius = 0;
    u8 level = 0;  // 0 ends it, 1 cripples it, 2 is a nuisance, 3 the wide area where only a shell reaches the engine
    const char* name = "";
  };
  std::vector<WeakSpot> weakSpots;  // only while someone looks for them
  std::vector<std::string> chat;  // the last lines, oldest first
  std::string chatInput;
  bool chatOpen = false;
  // The one playing, when not inside a tank.
  struct Foot {
    bool onFoot = false, inCar = false;
    double hp = 100;
    u8 items[8] = {};
    int weapon = 1;
    int ammo = 0;  // for what is in the hand, -1 if it needs none
    int has = 3;
    int hitsLanded = 0, hitsTaken = 0;
    V3 hurtFrom;
    int slot = 0;
    double fuelShare = 1;
    int shells = 0, missiles = 0, codes = 0;
    bool tapped = false, hasFix = false;
    bool droneLive = false;
    V3 droneP;
    double yaw = 0;  // of one's own figure, when this picture was made
    double droneBattery = 0;
    int drones = 0, medkits = 0;
    int launcher = 0;  // smoke charges of the tank one sits in
    bool noSight = false, noRange = false, noPeriscope = false, noRadio = false;
    V3 fix;
    double fixAge = 0, fixWait = 0;
    double prone = 0;
    V3 wind;
    V3 p;
  } foot;
  // The match, as far as the instruments care.
  bool missileLive = false;
  V3 missileP, missileV;
  int missiles = -1, repairKits = 0;
  double tankArmour = 1;
  int tankAmmo = -1;
  V3 enemyCog;
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
  // The line to the server: there and back, what got lost of it, how far back the others are shown, what comes in.
  bool netOn = false;
  double netPing = 0, netLoss = 0, netDelay = 0, netKb = 0;
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
  std::string detail;        // about what is selected
  std::vector<int> slots;    // a row of boxes, each the kind of thing filling that slot; -1 empty
  bool side = false;        // whether there is a box beside the list for those two
  int sel = 0;
};
// Where a panel sits, shared by drawing it and clicking on it.
// With a box beside the list the two together are centred, and the box gets what is left, up to a readable measure.
inline void panelLayout(int w, int h, size_t rows, float& x, float& y, float& pw, float& rowH, bool side = false, float* sideW = nullptr) {
  const float H = float(h), W = float(w);
  const float scale = H / 720.f < 1.f ? 1.f : float(int(H / 720.f + 0.5f));
  pw = 620 * scale;
  rowH = 31 * scale;
  const float gap = 14 * scale, box = side ? std::min(W - pw - 3 * gap, 320 * scale) : 0.f;
  x = (W - pw - (side ? box + gap : 0.f)) * 0.5f;
  y = (H - (float(rows) + 3.6f) * rowH) * 0.5f + 2.0f * rowH;
  if (sideW) *sideW = box;
}

// The map on screen, for the artillery's target and for knowing where one is.
struct UiMap {
  bool open = false;
  bool hasMark = false;
  bool strike = false;  // the next click calls in a strike code
  bool mouseKnown = false;  // where the mouse is over the map, in cursor
  bool padCursor = false;  // the controller is in hand: a cursor for it, and its buttons in the hint
  V3 cursor;
  V3 mark, me;
  double meYaw = 0;
  bool hasEnemy = false;
  V3 enemy;
  bool noRadio = false;
  double fixAge = 0;
  double fixWait = -1;  // until the next question may be asked; negative while the computer has not been tapped
};
constexpr double kMapHalf = 900.0;
// The map in eight by eight squares: A to H from west to east, 1 to 8 from north to south.
inline void vectorName(double x, double y, char out[4]) {
  const int col = int((x + kMapHalf) / (2.0 * kMapHalf) * 8.0), row = int((kMapHalf - y) / (2.0 * kMapHalf) * 8.0);
  out[0] = char('A' + (col < 0 ? 0 : (col > 7 ? 7 : col)));
  out[1] = char('1' + (row < 0 ? 0 : (row > 7 ? 7 : row)));
  out[2] = 0;
}
inline void mapLayout(int w, int h, float& x, float& y, float& size) {
  size = float(std::min(w, h)) * 0.88f;
  x = (float(w) - size) * .5f;
  y = (float(h) - size) * .5f;
}

// Typing without a keyboard: four rows of keys and a row of wide ones.
struct UiOsk {
  bool open = false;
  std::string title, text;
  const char* keys[4] = {};
  int x = 0, y = 0;
};

struct HudOptions {
  // Looking through the big rifle's scope, at this magnification; 0 when not.
  double scope = 0;
  int scopeZero = 100;
  int scopeView = SV_PLAIN;
  std::string scopeHint;  // which view the scope shows, and how to change it
  float hurt = 0;  // being hit: how red the picture goes
  float hitMark = 0;   // a hit landed
  // Something heard: which way, how loud, and how long ago it faded.
  struct Heard {
    V3 at;
    float strength;
    float life;
    bool shot;
  };
  std::vector<Heard> heard;
  float hurtDir = 0;   // a hit taken, seconds left to show from where
  V3 hurtFrom;
  UiPanel panel;
  UiMap map;
  UiOsk osk;
  bool vehicleHud = true;
  bool crosshair = false, gunRing = false;  // the ring shows where a tank's gun points
  bool damageView = true;
  double grassDensity = 1;
  std::string vector;  // the square one is in, in a match
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
  // What it takes to show a loading screen, long before the world is there to draw.
  void initUi();
  // progress from 0 to 1, or below 0 when nobody can say how long it takes.
  void drawLoading(int w, int h, const char* title, const char* detail, const char* tip, double progress);
  void vegetationStats(int& grass, int& bushes, int& tris, int& statics, double& ms) const;
  V3 nearestBush(V3 from, double beyond) const;
  void draw(const Snapshot& s, const Camera& cam, const HudOptions& hud, int w, int h, double dt);
  // Returns the nearest node under a screen position, for the mouse grabber.
  int pickNode(const Snapshot& s, const Camera& cam, int w, int h, double mx, double my, double maxPx) const;
  V3 unprojectOnPlane(const Camera& cam, int w, int h, double mx, double my, V3 planePoint) const;
  bool savePng(const std::string& path, int w, int h) const;

 private:
  struct Impl;
  Impl* d = nullptr;
};
