// Leopard 1: 42 tonnes of nodes and beams on torsion-bar suspension, with a turret that really turns.
//
// The hull and the turret are two separate soft bodies. A turning turret cannot be joined by beams, so
// each turret node is held to its place by a stiff spring whose anchor moves with the turret angle;
// the reaction goes back into the hull, so recoil and slewing rock the tank as they should.
#pragma once
#include <vector>
#include "vehicle.h"

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
  V3 trackWidthDir(int side) const { return trackOff[side] ? thrownLeft_[side] : left; }

  void update(Sim& sim, const Controls& in, double dt);
  void tickFire(Sim& sim, double dt);  // goes on burning while the wreck itself sleeps
  void shiftIndices(long nodes, long beams, long objects);
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
  void updateTurret(Sim& sim, const Controls& in, double dt);
  void updateDrive(Sim& sim, const Controls& in, double dt);
  void updateTracks(Sim& sim, double dt);
  void updateShells(Sim& sim, double dt);
  void addToHull(Sim& sim, V3 point, V3 force);
};

Tank buildLeopard(Sim& sim, V3 spawn, CarRig* rig);
