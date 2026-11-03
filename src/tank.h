// Leopard 1: 42 tonnes of nodes and beams on torsion-bar suspension, with a turret that really turns.
//
// The hull and the turret are two separate soft bodies. A turning turret cannot be joined by beams, so
// each turret node is held to its place by a stiff spring whose anchor moves with the turret angle;
// the reaction goes back into the hull, so recoil and slewing rock the tank as they should.
#pragma once
#include <vector>
#include "vehicle.h"

enum TurretKind : u8 { TK_STANDARD, TK_LIGHT, TK_ARTILLERY, TK_MISSILE, TK_COUNT };

// What makes one turret different. All have the same nodes and beams: only where the muzzle lies, how heavy it
// all is and what it fires differ, so that the hull under it notices the weight by itself.
struct TurretSpec {
  const char* name;
  const char* mesh;
  double massScale, powerScale;
  double gunLen, recoilLen, recoilForce;
  double reload;
  int ammo;
  double shellSpeed, blastImpulse, blastRadius, shellLife;
  double pitchMin, pitchMax, yawRate, pitchRate;  // degrees, and degrees per second
  int missiles;
};
const TurretSpec& turretSpec(TurretKind k);

enum TankModule { TM_ENGINE, TM_RING, TM_GUN, TM_AMMO, TM_TRACK_L, TM_TRACK_R, TM_COUNT };

class Tank {
 public:
  static constexpr double kWheelR = 0.37;   // road wheel with track
  static constexpr double kSprocketR = 0.33;  // drive sprocket, at the rear

  struct RoadWheel {
    u32 in = 0, out = 0;
    bool left = true;
    double Fn = 0;
    double soft = 0;
  };

  std::vector<RoadWheel> wheels;
  std::vector<u32> rigidNodes_;      // everything fixed to the hull
  u16 hullObj = 0, turretObj = 0;
  std::vector<u32> turretNodes, gunNodes;
  std::vector<V3> turretRest, gunRest;  // local: turret frame, and gun frame about the trunnions
  V3 turretAxis, trunnion;              // in the hull frame, and in the turret frame
  u32 refFL = 0, refFR = 0, refRL = 0, refRR = 0, refBottom = 0;
  u32 sprocket[2][2] = {}, idler[2][2] = {};

  // State.
  double turretYaw = 0, gunPitch = 0;
  double recoil = 0, recoilTime = 10;
  double reload = 0;
  double engineOmega = 70;
  double throttleAct = 0, brakeAct = 0;
  int gear = 1;
  double shiftClock = 1;
  double trackOmega[2] = {};
  double trackPhase[2] = {};
  double steerAct = 0;
  double loadNow = 0;
  bool autoBox = true;
  double reloadTime = 4;
  double shotsFired = 0;
  int ammo = -1;  // rounds left; -1 never runs out, as in free driving

  TurretKind kind = TK_STANDARD;
  const TurretSpec& spec() const { return turretSpec(kind); }

  struct Shell {
    V3 p, v;
    double age = 0;
  };
  std::vector<Shell> shells;

  // Telemetry.
  V3 cog, vCog, fwd{1, 0, 0}, left{0, 1, 0}, up{0, 0, 1};
  V3 muzzle, gunDir{1, 0, 0}, sight;
  double speed = 0, latAcc = 0, lonAcc = 0, roll = 0;
  double fuel = 1160;
  double rpm() const { return engineOmega * 60.0 / (2.0 * PI); }

  // Damage. A direct hit does about half of what it takes; beside the running gear it throws the track off.
  // At the end the ammunition goes up, and that is the end of the tank.
  double damage = 0;
  bool destroyed = false;
  double burnTime = 0;
  bool trackOff[2] = {false, false};
  double mod[TM_COUNT] = {1, 1, 1, 1, 1, 1};  // what is left of each part; at nothing it no longer works

  // A guided missile is flown by hand towards aimPoint, one at a time. Firing again sets it off early.
  int missiles = 0;
  bool missileLive = false;
  V3 missileP, missileV;
  double missileAge = 0;

  // A repair takes a kit and standing still for a while.
  int repairKits = 0;
  double repairClock = 0;
  static constexpr double kRepairTime = 6.0;
  V3 trackWidthDir(int side) const { return trackOff[side] ? thrownLeft_[side] : left; }

  void update(Sim& sim, const Controls& in, double dt);
  void tickFire(Sim& sim, double dt);  // goes on burning while the wreck itself sleeps
  void shiftIndices(long nodes, long beams, long objects);

  // A player's machine runs no physics. This is what it needs to show a tank and fill in the instruments.
  struct NetState {
    V3 vel, muzzle, gunDir, sight;
    float turretYaw = 0, gunPitch = 0, reload = 0, reloadTime = 4, damage = 0, burnTime = 0;
    float trackPhase[2] = {}, trackOmega[2] = {};
    float engineOmega = 0, throttle = 0, brake = 0, steer = 0, speed = 0, latAcc = 0, lonAcc = 0, fuel = 0, load = 0;
    int gear = 1, ammo = -1;
    bool destroyed = false, trackOff[2] = {}, autoBox = true;
    float mod[TM_COUNT] = {1, 1, 1, 1, 1, 1};
    int missiles = 0, repairKits = 0;
    float repairClock = 0;
    bool missileLive = false;
    V3 missileP, missileV;
  };
  NetState netState() const;
  // Call once the nodes hold their new positions.
  void applyNetState(Sim& sim, const NetState& n, double dt);
  void settle(Sim& sim, double seconds);
  // The closed line of the track around sprocket, wheels and idler, in world space.
  void trackPath(const Sim& sim, int side, std::vector<V3>& out) const;

 private:
  V3 prevV_;
  V3 vSlow_;
  double crashHold_ = 0;
  u32 blastSeen_ = 0;
  std::vector<V3> thrown_[2];
  V3 thrownLeft_[2];
  void takeHit(Sim& sim, double hit, V3 at);
  void checkDamage(Sim& sim, double dt);
  void throwTrack(Sim& sim, int side);
  void destroy(Sim& sim);
  double wheelLift_[14] = {};        // road wheel height against the hull, smoothed, as drawn
  void dampHull(Sim& sim, double dt);
  double tetDet_ = 0;
  void updateFrame(Sim& sim, double dt);
  void updateWheelLift(const Sim& sim, double dt);
  void updateTurret(Sim& sim, const Controls& in, double dt);
  void updateDrive(Sim& sim, const Controls& in, double dt);
  void updateTracks(Sim& sim, double dt);
  void updateShells(Sim& sim, double dt);
  void addToHull(Sim& sim, V3 point, V3 force);
  void damageModules(Sim& sim, V3 c, double impulse);
  void updateMissile(Sim& sim, const Controls& in, double dt);
  void updateRepair(Sim& sim, const Controls& in, double dt);
  bool pathHit(const Sim& sim, V3 a, V3 b, bool skipOwn, V3& at) const;
  V3 ring_;
};

Tank buildLeopard(Sim& sim, V3 spawn, CarRig* rig, TurretKind kind = TK_STANDARD);
