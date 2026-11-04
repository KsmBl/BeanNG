// The packets of a duel, shared by server and player.
#pragma once
#include <algorithm>
#include <cmath>
#include "net.h"
#include "sim.h"
#include "tank.h"

namespace net {

constexpr double kSendRate = 30.0;
// Every packet says how many physics steps the server has made: the moment it shows, whatever the send loop was up to.
constexpr double kStepRate = 2000.0;
constexpr int kMaxEventsPerPacket = 18;
constexpr int kMaxBlastsPerPacket = 12;
constexpr int kMaxSceneryPerPacket = 70;
constexpr double kNodeScale = 256.0;  // a tank's nodes go as offsets of 4 mm steps from its middle, 128 m either way

inline void writeState(Writer& w, const Tank::NetState& n) {
  w.putV3(n.vel);
  w.putV3(n.muzzle);
  w.putV3(n.gunDir);
  w.putV3(n.sight);
  for (float v : {n.turretYaw, n.gunPitch, n.reload, n.reloadTime, n.damage, n.burnTime, n.trackPhase[0], n.trackPhase[1], n.trackOmega[0], n.trackOmega[1],
                  n.engineOmega, n.throttle, n.brake, n.steer, n.speed, n.latAcc, n.lonAcc, n.fuel, n.load})
    w.put(v);
  w.put(i8(n.gear));
  w.put(i16(n.ammo));
  w.put(u8((n.destroyed ? 1 : 0) | (n.trackOff[0] ? 2 : 0) | (n.trackOff[1] ? 4 : 0) | (n.missileLive ? 16 : 0)));
  for (float m : n.mod) w.put(m);
  w.put(u8(n.missiles));
  w.put(u8(n.smokeCharges));
  w.put(u8(n.netOn ? 1 : 0));
  w.put(n.netClock);
  if (n.missileLive) {
    w.putV3(n.missileP);
    w.putV3(n.missileV);
  }
}

inline void readState(Reader& r, Tank::NetState& n) {
  n.vel = r.getV3();
  n.muzzle = r.getV3();
  n.gunDir = r.getV3();
  n.sight = r.getV3();
  for (float* v : {&n.turretYaw, &n.gunPitch, &n.reload, &n.reloadTime, &n.damage, &n.burnTime, &n.trackPhase[0], &n.trackPhase[1], &n.trackOmega[0], &n.trackOmega[1],
                   &n.engineOmega, &n.throttle, &n.brake, &n.steer, &n.speed, &n.latAcc, &n.lonAcc, &n.fuel, &n.load})
    r.get(*v);
  i8 gear = 0;
  i16 ammo = 0;
  u8 flags = 0;
  r.get(gear);
  r.get(ammo);
  r.get(flags);
  n.gear = gear;
  n.ammo = ammo;
  n.destroyed = flags & 1;
  n.trackOff[0] = flags & 2;
  n.trackOff[1] = flags & 4;
  n.missileLive = flags & 16;
  for (float& m : n.mod) r.get(m);
  u8 missiles = 0;
  r.get(missiles);
  n.missiles = missiles;
  u8 charges = 0;
  r.get(charges);
  n.smokeCharges = charges;
  u8 netOn = 0;
  r.get(netOn);
  n.netOn = netOn != 0;
  r.get(n.netClock);
  if (n.missileLive) {
    n.missileP = r.getV3();
    n.missileV = r.getV3();
  }
}

inline i16 packOffset(double v) { return i16(std::clamp(std::lround(v * kNodeScale), -32767L, 32767L)); }

}  // namespace net
