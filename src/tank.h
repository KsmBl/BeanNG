// Leopard 1: 42 tonnes of nodes and beams on torsion-bar suspension, with a turret that really turns.
//
// The hull and the turret are two separate soft bodies. A turning turret cannot be joined by beams, so
// each turret node is held to its place by a stiff spring whose anchor moves with the turret angle;
// the reaction goes back into the hull, so recoil and slewing rock the tank as they should.
#pragma once
#include <string>
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

// The first six suffer from blasts. The rest, and the track pins and turret ring too, are the small places a
// heavy bullet can reach: weak spots at logical places, each taking something particular from the tank.
// What is under the turret: the same soft body, but it weighs and takes hits differently.
enum HullKind : u8 { HK_MEDIUM, HK_HEAVY, HK_FAST, HK_COUNT };
struct HullSpec {
  const char* name;
  const char* meshPart;  // what the mesh's file name has for it
  double massScale, powerScale;
  double armour;  // what a hit does to it: below 1 it takes more of them
};
const HullSpec& hullSpec(HullKind k);
std::string tankMeshFile(HullKind hull, TurretKind turret);

enum TankModule { TM_ENGINE, TM_RING, TM_GUN, TM_AMMO, TM_TRACK_L, TM_TRACK_R, TM_SIGHT, TM_RANGE, TM_PERISCOPE, TM_RADIATOR, TM_FUEL, TM_ELEVATION, TM_ANTENNA, TM_COUNT };

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
  double reloadTime = 4;
  double shotsFired = 0;
  int ammo = -1;  // rounds left; -1 never runs out, as in free driving

  TurretKind kind = TK_STANDARD;
  HullKind hull = HK_MEDIUM;
  const HullSpec& hullSpec() const { return ::hullSpec(hull); }
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
  double mod[TM_COUNT] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};  // what is left of each part; at nothing it no longer works
  double radiatorHeat = 0;  // grows with a holed radiator, and takes the engine's power with it
  // A bullet on its way from a to b: what it hits among the weak spots, with this much force (1 is the big rifle).
  bool bulletHit(Sim& sim, V3 a, V3 b, double force);
  // Where a bullet or a blast does something, as a scope would show it.
  struct WeakPoint {
    V3 at;
    double radius;
    int level;  // 0 ends the tank, 1 cripples it, 2 is a nuisance
    const char* name;
  };
  void weakPoints(const Sim& sim, std::vector<WeakPoint>& out) const;

  // A guided missile is flown by hand towards aimPoint, one at a time. Firing again sets it off early.
  int smokeCharges = 3;  // the launcher on the turret
  bool netOn = false;    // under its camouflage net, which it loses by moving off
  double netClock = 0;   // the crew is putting it up
  int missiles = 0;
  bool missileLive = false;
  V3 missileP, missileV;
  double missileAge = 0;

  V3 trackWidthDir(int side) const { return trackOff[side] ? thrownLeft_[side] : left; }

  void update(Sim& sim, const Controls& in, double dt);
  void makeWreck(Sim& sim);  // what has been lying there since some earlier battle
  void repair(Sim& sim);  // what a kit can do: everything working again, though not as new
  void tickFire(Sim& sim, double dt);  // goes on burning while the wreck itself sleeps
  void shiftIndices(long nodes, long beams, long objects);

  // A player's machine runs no physics. This is what it needs to show a tank and fill in the instruments.
  struct NetState {
    V3 vel, muzzle, gunDir, sight;
    float recoil = 0, turretYaw = 0, gunPitch = 0, reload = 0, reloadTime = 4, damage = 0, burnTime = 0;
    float trackPhase[2] = {}, trackOmega[2] = {};
    float engineOmega = 0, throttle = 0, brake = 0, steer = 0, speed = 0, latAcc = 0, lonAcc = 0, fuel = 0, load = 0;
    int gear = 1, ammo = -1;
    bool destroyed = false, trackOff[2] = {};
    float mod[TM_COUNT] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    int missiles = 0, smokeCharges = 0;
    bool netOn = false;
    float netClock = 0;
    bool missileLive = false;
    V3 missileP, missileV;
  };
  NetState netState() const;
  // Call once the nodes hold their new positions.
  void applyNetState(Sim& sim, const NetState& n, double dt);
  // For whoever is aiming this gun on their own machine: the turret follows the hand at once, and the server's
  // word pulls it back if it thinks differently. Call after applyNetState.
  void showOwnAim(Sim& sim, const Controls& in, double& yawShown, double& pitchShown, double dt);
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
  V3 ringPoint(const Sim& sim) const;
  struct BulletSpot {
    int module;
    V3 at;
    double radius, toughness;
  };
  static constexpr int kBulletSpots = 21;
  void bulletSpots(const Sim& sim, V3 ring, BulletSpot out[kBulletSpots]) const;
  void aimAngles(V3 ring, const Controls& in, double& wantYaw, double& wantPitch) const;
  double yawRateNow() const;
  double pitchRateNow() const;
  void updateDrive(Sim& sim, const Controls& in, double dt);
  void updateTracks(Sim& sim, double dt);
  void updateShells(Sim& sim, double dt);
  void addToHull(Sim& sim, V3 point, V3 force);
  void damageModules(Sim& sim, V3 c, double impulse);
  void updateMissile(Sim& sim, const Controls& in, double dt);
  bool pathHit(const Sim& sim, V3 a, V3 b, bool skipOwn, V3& at) const;
  V3 ring_;
};

Tank buildLeopard(Sim& sim, V3 spawn, CarRig* rig, TurretKind kind = TK_STANDARD, HullKind hull = HK_MEDIUM);
