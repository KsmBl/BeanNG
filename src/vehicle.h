// Vehicle systems layered on top of the node/beam body:
//   tyres (Magic Formula with relaxation), brakes+ABS, engine, clutch, gearbox, open diff,
//   the hydropneumatic suspension (gas spheres, Hydractive stiffness switching, self-levelling)
//   and the Activa hydraulic anti-roll system.
#pragma once
#include <string>
#include <vector>
#include "carmesh.h"
#include "sim.h"
#include "world.h"

struct Controls {
  double steer = 0;     // -1..1, positive = left
  double throttle = 0;  // 0..1
  double brake = 0;     // 0..1
  double handbrake = 0;
  bool shiftUp = false, shiftDown = false;  // edge-triggered, cleared by the vehicle
  bool toggleAuto = false;
  bool toggleSport = false;
  bool cycleHeight = false;
  bool toggleIgnition = false;
  bool toggleActiva = false;
  int requestGear = 99;  // 99 = no request; used by the automatic's drive/reverse selection
  bool fire = false;     // edge-triggered: detonates the car bomb, or fires the tank's gun
  V3 aimPoint;           // where the turret should point, world space
};

// Everything the driver can set in the overlay.
struct Tuning {
  bool abs = true, esp = true, tcs = true;
  double brake = 1, power = 1, grip = 1, damper = 1, bar = 1, steer = 1;
};

struct Wheel {
  double espTorque = 0;  // brake torque from the stability control [N m]
  u32 axIn = 0, axOut = 0, ring[6] = {};  // hub nodes; the ring does not spin physically
  bool left = true, front = true, driven = false;
  double R = 0.312;      // unloaded rolling radius
  double width = 0.205;
  double I = 1.1;        // spin inertia [kg m^2]
  double kRadial = 220e3, cRadial = 450;
  double brakeMax = 2000;

  // State.
  double omega = 0, theta = 0;
  double ux = 0, uy = 0;     // tread deflection states (relaxation)
  double absFrac = 1.0;
  double deflate = 0;        // 0 = inflated, 1 = flat
  bool attached = true;      // drive shaft still connected

  // Permanent rim damage, in the wheel's own frame so that it turns with the wheel.
  static constexpr int kDentBins = 24;
  float dent[kDentBins] = {};  // radial dent depth [m] around the rim
  float runoutX = 0, runoutZ = 0;  // lateral bend of the rim, as a cosine and sine amplitude [m]
  u32 spoke[6][2] = {};        // beams from each ring node to the two axle nodes
  double spokeSeen[6][2] = {}; // plastic shortening already turned into rim damage

  // Where the tyre touches, for the visible contact patch.
  V3 patchPoint, patchNormal{0, 0, 1};
  double skid = 0;  // how hard the tread is sliding, 0..1: rubber left on the road

  // Outputs.
  double Fz = 0, Fx = 0, Fy = 0, slipX = 0, slipY = 0;
  double driveT = 0;
  bool contact = false;
  u8 surface = SURF_ASPHALT;
  V3 hub, axis, upHint{0, 0, 1};
};

// One hydropneumatic strut: a piston feeding a nitrogen sphere through a damper valve.
struct HydroStrut {
  u32 top = 0, low = 0;
  double A = 3.85e-4;       // piston area [m^2]
  double V0 = 290e-6;       // sphere gas volume at precharge [m^3]
  double p0 = 48e5;         // precharge [Pa]
  double Vf = 0;            // fluid currently in the sphere [m^3]
  double Lmin = 0, Lmax = 0, Lnom = 0;
  double bump[2] = {1850, 1000}, reb[2] = {2400, 1400};  // [firm, soft] damping
  double L = 0, vel = 0, pGas = 0, F = 0;
  bool burst = false;
};

struct HydroAxle {
  HydroStrut s[2];
  double regV0 = 400e-6, regP0 = 48e5, regVf = 0;  // Hydractive regulator sphere
  double levelErrLP = 0;
  double kBar = 30e3;     // anti-roll bar rate [N per m of stroke difference]
  double ram = 0;         // Activa ram displacement [m]
};

// Named damage flags shown on the HUD.
struct DamageState {
  bool radiator = false, hydraulics = false, engineMount = false, steering = false;
  bool seized = false;
  bool gearbox = false, bellHousing = false, tank = false, oilLeak = false;
  double coolantTemp = 90;
  double oilTemp = 95;
  double coolant = 1.0;
};

class Vehicle {
 public:
  // Node and beam references created by the builder.
  u16 obj = 0;
  std::vector<Wheel> wheels;
  HydroAxle axle[2];  // 0 front, 1 rear
  std::vector<u32> engineNodes, gearboxNodes;
  std::vector<u32> structureNodes;  // everything sprung, for shape damping
  u32 refFrontL = 0, refFrontR = 0, refRearL = 0, refRearR = 0;  // body frame reference
  u32 tieBeam[2] = {};
  double steerTravel = 0.066;  // rack travel at full lock [m]
  std::vector<u32> radiatorBeams, engineMountBeams, hydraulicBeams, engineBlockBeams, gearboxBeams, bellBeams, tankBeams;
  u32 shaftNode[2] = {};  // halfshaft flanges on the differential, left and right
  double dragArea = 0.63;      // Cd * A, Xantia ~0.30 * 2.1 m^2
  double totalMass = 0;

  // Driver-visible state.
  double steerAct = 0;
  double engineOmega = 85;
  bool engineRunning = true;
  int gear = 1;                // -1 R, 0 N, 1..5
  bool autoBox = true;
  bool sportMode = false;
  bool activaOn = true;
  int heightMode = 1;          // 0 low, 1 road, 2 high
  double clutch = 1;
  double sinceShift = 10;

  // Powertrain state: every rotating part with its own inertia.
  double crankAngle = 0;
  double manifold = 0.3;       // intake manifold fill, fraction of ambient pressure
  double combustionTorque = 0;
  double discOmega = 85, inputOmega = 0, carrierOmega = 0, spiderOmega = 0;
  double dampAngle = 0;        // clutch disc torsion damper wind-up [rad]
  double shaftAngle[2] = {};   // halfshaft twist, left and right [rad]
  double shaftLen[2] = {};     // as-built halfshaft lengths [m]
  int shiftPhase = 0;          // 0 in gear, 1 clutch opening, 2 synchronising
  int shiftTarget = 1;
  double shiftClock = 0;
  double fuelLitres = 55;
  double oilLevel = 1;
  double starterClock = 0;

  // Hydraulic circuit state.
  double accFluid = 190e-6;    // fluid in the main accumulator sphere [m^3]
  double reservoir = 3.2e-3;   // LHM in the tank [m^3]
  bool pumpCharging = true;
  double pumpTorque = 0;
  bool hydractiveSoft = true;
  double firmHold = 0;
  double accumulator = 165e5;  // main accumulator pressure [Pa]
  DamageState dmg;
  std::string lastDamage;
  double lastDamageAge = 100;

  // Body-frame telemetry.
  V3 cog, vCog, fwd{1, 0, 0}, left{0, 1, 0}, up{0, 0, 1};
  double speed = 0;      // forward speed [m/s]
  double latAcc = 0, lonAcc = 0;  // low-passed [m/s^2]
  double rollAngle = 0, pitchAngle = 0;  // body relative to the world [rad]
  double suspRoll = 0;   // roll measured by stroke difference [rad]
  double peakG = 0;      // peak |acc| in the last moments
  double rideHeight = 0;
  double engineTorqueOut = 0;
  double throttleAct = 0;
  double brakeAct = 0;

  Tuning tune;
  bool absActive = false, espActive = false, tcsActive = false;
  void update(Sim& sim, const Controls& in, double dt);
  // The bomb under the back seat.
  void detonate(Sim& sim);
  bool exploded = false;
  double burnTime = 0;   // seconds since it went off
  void settleFluid(Sim& sim);
  // Sets all rotating parts to match rolling at this speed, for test bench starts.
  void presetRolling(double v, int gearIndex);
  double rpm() const { return engineOmega * 60.0 / (2.0 * PI); }

 private:
  V3 prevV_, prevFwd_{1, 0, 0};
  V3 omegaFit_;
  u32 lastSplit_ = 0;
  double yawRate_ = 0;
  double prevThrottle_ = 0, prevSteer_ = 0;
  double ramCmd_ = 0, ramTrim_ = 0;
  double hbAct_ = 0;
  double clutchRamp_ = 1, prevBrake_ = 0, rackRate_ = 0;
  double rackPrev_ = 0;
  double stuckTime_ = 0;
  int bodyTick_ = 0;
  Controls edge_;
  double espThrottle_ = 1, tcsThrottle_ = 1, sumpContact_ = 0, absHold_ = 0, espHold_ = 0, tcsHold_ = 0;
  void updateStability(const Controls& in, double dt);
  void updateBodyFrame(Sim& sim, double dt);
  void updateHydraulics(Sim& sim, double dt);
  void updateDrivetrain(Sim& sim, const Controls& in, double dt);
  void applyBlockTorque(Sim& sim, const std::vector<u32>& nodes, double torque);
  void drawAccumulator(double volume, bool returnsToTank = true);
  void updateWheel(Sim& sim, Wheel& w, double dt);
  void updateDamage(Sim& sim, double dt);
  void shapeDamping(Sim& sim, double dt);
  void note(const char* msg);
  void addRimDamage(Wheel& w, V3 where, double depth, double lateral);
  void updateRimDamage(Sim& sim, Wheel& w);
};

// What the builder hands to the renderer: node groups the mesh binds to, and running gear drawn as
// rods between nodes, so that arms, shafts and struts visibly follow the suspension.
struct VisLink {
  u32 a, b;
  float radius;
  u8 mat;        // MeshMat
  u8 kind;       // 0 solid rod, 1 strut (thin rod near a, thick cylinder near b), 2 proud of its ends
};

struct CarRig {
  double noseX = 2.26;     // design frame origin ahead of the spawn point
  bool carWheels = true;   // draw road wheels from the mesh's wheel parts
  bool scratches = true;
  std::vector<NodeGroup> groups;
  std::vector<VisLink> links;
};


// Spawns a Citroen Xantia Activa into the sim at the given position and yaw.
Vehicle buildXantia(Sim& sim, V3 pos, double yaw, CarRig* rig);

// Runs the car for a moment so it rests at its equilibrium ride height.
void settleVehicle(Sim& sim, Vehicle& v, double seconds);
