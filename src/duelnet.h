// The packets of a duel, shared by server and player.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include "net.h"
#include "sim.h"
#include "tank.h"

namespace net {

constexpr double kSendRate = 120.0;
// Every packet says how many physics steps the server has made: the moment it shows, whatever the send loop was up to.
constexpr double kStepRate = 2000.0;
constexpr int kMaxEventsPerPacket = 18;
constexpr int kMaxBlastsPerPacket = 12;
constexpr int kMaxSceneryPerPacket = 70;
constexpr int kMaxNodesPerPacket = 150;
constexpr int kNodeChange = 2;  // in steps of kNodeScale: a node this far from what the player was told goes out again
constexpr int kNodeHeal = 3;    // and this many with every packet in any case, a packet may have got lost
constexpr u32 kSlowEvery = u32(kSendRate / 10.0);  // ticks between the parts that change only now and then
constexpr int kBundleBytes = 1300;
// What is near goes with every tick, what is far off only often enough to be drawn smoothly: the far end of the map needs
// no more than a player can see through a scope. In ticks between one word and the next, by distance in metres.
inline u32 sendEvery(double distance) {
  const double rate = distance < 150.0 ? kSendRate : (distance < 400.0 ? 60.0 : 30.0);
  return std::max(1u, u32(std::lround(kSendRate / rate)));
}
constexpr double kNodeScale = 256.0;  // a tank's nodes go as offsets of 4 mm steps from its middle, 128 m either way

inline i16 packOffset(double v) { return i16(std::clamp(std::lround(v * kNodeScale), -32767L, 32767L)); }
inline i16 packCm(double v) { return i16(std::clamp(std::lround(v * 100.0), -32767L, 32767L)); }  // 327 m/s either way
inline i16 packUnit(double v) { return i16(std::clamp(std::lround(v * 32767.0), -32767L, 32767L)); }
inline i16 packAngle(double a) { return packUnit(std::remainder(a, 2.0 * PI) / PI); }
inline V3 unpackOffsetV3(Reader& r) {
  i16 x = 0, y = 0, z = 0;
  r.get(x);
  r.get(y);
  r.get(z);
  return V3{x / kNodeScale, y / kNodeScale, z / kNodeScale};
}
inline void putOffsetV3(Writer& w, V3 v) {
  w.put(packOffset(v.x));
  w.put(packOffset(v.y));
  w.put(packOffset(v.z));
}
inline V3 getCmV3(Reader& r) {
  i16 x = 0, y = 0, z = 0;
  r.get(x);
  r.get(y);
  r.get(z);
  return V3{x / 100.0, y / 100.0, z / 100.0};
}
inline void putCmV3(Writer& w, V3 v) {
  w.put(packCm(v.x));
  w.put(packCm(v.y));
  w.put(packCm(v.z));
}
inline u8 packShare(double v) { return u8(std::clamp(std::lround(v * 255.0), 0L, 255L)); }

// What moves with every tick, and with some ticks (slow) what changes only now and then. A reading that does not say
// the slow part leaves those fields as they are, so hand it the last state.
inline void writeState(Writer& w, const Tank::NetState& n, V3 cog, bool slow) {
  putCmV3(w, n.vel);
  putOffsetV3(w, n.muzzle - cog);
  w.put(packUnit(n.gunDir.x));
  w.put(packUnit(n.gunDir.y));
  w.put(packUnit(n.gunDir.z));
  putOffsetV3(w, n.sight - cog);
  w.put(n.turretYaw);
  w.put(n.gunPitch);
  w.put(packOffset(n.recoil));
  w.put(n.trackPhase[0]);
  w.put(n.trackPhase[1]);
  w.put(packCm(n.trackOmega[0]));
  w.put(packCm(n.trackOmega[1]));
  w.put(i16(std::clamp(std::lround(n.engineOmega * 10.0f), -32767L, 32767L)));
  w.put(packShare(n.throttle));
  w.put(packShare(n.brake));
  w.put(i8(std::clamp(std::lround(n.steer * 127.0f), -127L, 127L)));
  w.put(packCm(n.speed));
  w.put(i8(std::clamp(std::lround(n.latAcc * 3.0f), -127L, 127L)));
  w.put(i8(std::clamp(std::lround(n.lonAcc * 3.0f), -127L, 127L)));
  w.put(i16(std::clamp(std::lround(n.reload * 100.0f), -32767L, 32767L)));
  w.put(i8(n.gear));
  w.put(u8((n.destroyed ? 1 : 0) | (n.trackOff[0] ? 2 : 0) | (n.trackOff[1] ? 4 : 0) | (n.missileLive ? 16 : 0) | (n.missileJammed ? 8 : 0) | (n.missileSpent ? 64 : 0) | (slow ? 32 : 0)));
  if (n.missileLive) {
    w.putV3(n.missileP);
    w.putV3(n.missileV);
  }
  if (!slow) return;
  for (float v : {n.reloadTime, n.damage, n.burnTime, n.load, n.netClock}) w.put(v);
  w.put(i16(n.ammo));
  for (float m : n.mod) w.put(m);
  w.put(u8(n.missiles));
  w.put(u8(n.smokeCharges));
  w.put(u8(n.netOn ? 1 : 0));
  w.put(n.bounces);
}

inline void readState(Reader& r, Tank::NetState& n, V3 cog) {
  n.vel = getCmV3(r);
  n.muzzle = cog + unpackOffsetV3(r);
  i16 gx = 0, gy = 0, gz = 0;
  r.get(gx);
  r.get(gy);
  r.get(gz);
  n.gunDir = V3{gx / 32767.0, gy / 32767.0, gz / 32767.0};
  n.sight = cog + unpackOffsetV3(r);
  r.get(n.turretYaw);
  r.get(n.gunPitch);
  i16 recoil = 0;
  r.get(recoil);
  n.recoil = float(recoil / kNodeScale);
  r.get(n.trackPhase[0]);
  r.get(n.trackPhase[1]);
  i16 om0 = 0, om1 = 0, eng = 0, speed = 0, reload = 0;
  u8 gas = 0, brake = 0, flags = 0;
  i8 steer = 0, lat = 0, lon = 0, gear = 0;
  r.get(om0);
  r.get(om1);
  r.get(eng);
  r.get(gas);
  r.get(brake);
  r.get(steer);
  r.get(speed);
  r.get(lat);
  r.get(lon);
  r.get(reload);
  r.get(gear);
  r.get(flags);
  n.trackOmega[0] = om0 / 100.0f;
  n.trackOmega[1] = om1 / 100.0f;
  n.engineOmega = eng / 10.0f;
  n.throttle = gas / 255.0f;
  n.brake = brake / 255.0f;
  n.steer = steer / 127.0f;
  n.speed = speed / 100.0f;
  n.latAcc = lat / 3.0f;
  n.lonAcc = lon / 3.0f;
  n.reload = reload / 100.0f;
  n.gear = gear;
  n.destroyed = flags & 1;
  n.trackOff[0] = flags & 2;
  n.trackOff[1] = flags & 4;
  n.missileLive = flags & 16;
  n.missileJammed = flags & 8;
  n.missileSpent = flags & 64;
  if (n.missileLive) {
    n.missileP = r.getV3();
    n.missileV = r.getV3();
  }
  if (!(flags & 32)) return;
  for (float* v : {&n.reloadTime, &n.damage, &n.burnTime, &n.load, &n.netClock}) r.get(*v);
  i16 ammo = 0;
  r.get(ammo);
  n.ammo = ammo;
  for (float& m : n.mod) r.get(m);
  u8 missiles = 0, charges = 0, netOn = 0;
  r.get(missiles);
  r.get(charges);
  r.get(netOn);
  n.missiles = missiles;
  n.smokeCharges = charges;
  n.netOn = netOn != 0;
  r.get(n.bounces);
}

}  // namespace net
