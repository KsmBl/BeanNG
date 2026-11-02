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
};

struct Wheel {
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

  // Outputs.
  double Fz = 0, Fx = 0, Fy = 0, slipX = 0, slipY = 0;
  double driveT = 0;
  bool contact = false;
  u8 surface = SURF_ASPHALT;
  V3 hub, axis;
};

// One hydropneumatic strut: a piston feeding a nitrogen sphere through a damper valve.
struct HydroStrut {
  u32 top = 0, low = 0;
  double A = 3.85e-4;       // piston area [m^2]
  double V0 = 210e-6;       // sphere gas volume at precharge [m^3]
  double p0 = 55e5;         // precharge [Pa]
  double Vf = 0;            // fluid currently in the sphere [m^3]
  double Lmin = 0, Lmax = 0, Lnom = 0;
  double bump[2] = {1850, 1000}, reb[2] = {2400, 1400};  // [firm, soft] damping
  double L = 0, vel = 0, pGas = 0, F = 0;
  bool burst = false;
};

struct HydroAxle {
  HydroStrut s[2];
  double regV0 = 500e-6, regP0 = 55e5, regVf = 0;  // Hydractive regulator sphere
  double levelErrLP = 0;
  double kBar = 30e3;     // anti-roll bar rate [N per m of stroke difference]
  double ram = 0;         // Activa ram displacement [m]
};

// Named damage flags shown on the HUD.
struct DamageState {
  bool radiator = false, hydraulics = false, engineMount = false, steering = false;
  bool seized = false;
  double coolantTemp = 90;
  double coolant = 1.0;
};

class Vehicle {
 public:
  // Node and beam references created by the builder.
  u16 obj = 0;
  std::vector<Wheel> wheels;
  HydroAxle axle[2];  // 0 front, 1 rear
  std::vector<u32> engineNodes, gearboxNodes;
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
  double shiftTimer = 0;
  double sinceShift = 10;
  bool hydractiveSoft = true;
  double firmHold = 0;
  double accumulator = 170e5;  // hydraulic accumulator pressure [Pa]
  double hydroFluid = 1.0;     // reservoir level
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

  void update(Sim& sim, const Controls& in, double dt);
  void settleFluid(Sim& sim);
  double rpm() const { return engineOmega * 60.0 / (2.0 * PI); }

 private:
  V3 prevV_, prevFwd_{1, 0, 0};
  double yawRate_ = 0;
  double prevThrottle_ = 0, prevSteer_ = 0;
  double ramCmd_ = 0, ramTrim_ = 0;
  double hbAct_ = 0;
  double rackPrev_ = 0;
  double stuckTime_ = 0;
  int bodyTick_ = 0;
  Controls edge_;
  void updateBodyFrame(Sim& sim, double dt);
  void updateHydraulics(Sim& sim, double dt);
  void updateDrivetrain(Sim& sim, const Controls& in, double dt);
  void updateWheel(Sim& sim, Wheel& w, double dt);
  void updateDamage(Sim& sim, double dt);
  void note(const char* msg);
};

// What the builder hands to the renderer: node groups the mesh binds to, and running gear drawn as
// rods between nodes, so that arms, shafts and struts visibly follow the suspension.
struct VisLink {
  u32 a, b;
  float radius;
  u8 mat;        // MeshMat
  u8 kind;       // 0 solid rod, 1 strut (thin rod near a, thick cylinder near b)
};

struct CarRig {
  std::vector<NodeGroup> groups;
  std::vector<VisLink> links;
};

// Spawns a Citroen Xantia Activa into the sim at the given position and yaw.
Vehicle buildXantia(Sim& sim, V3 pos, double yaw, CarRig* rig);

// Runs the car for a moment so it rests at its equilibrium ride height.
void settleVehicle(Sim& sim, Vehicle& v, double seconds);
