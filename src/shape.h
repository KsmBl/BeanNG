// How a vehicle goes over the wire: where it is and how it lies, and only those of its nodes that are not where the
// player was last told they are, against the part they belong to. A tank that is just driving is a few dozen bytes.
#pragma once
#include <vector>
#include "duel.h"
#include "duelnet.h"

namespace net {

// A rotation as four small numbers.
inline void packAxes(V3 F, V3 L, V3 U, i16 q[4]) {
  double w, x, y, z;
  const double t = F.x + L.y + U.z;
  if (t > 0) {
    const double s = std::sqrt(t + 1.0) * 2.0;
    w = 0.25 * s, x = (L.z - U.y) / s, y = (U.x - F.z) / s, z = (F.y - L.x) / s;
  } else if (F.x > L.y && F.x > U.z) {
    const double s = std::sqrt(1.0 + F.x - L.y - U.z) * 2.0;
    w = (L.z - U.y) / s, x = 0.25 * s, y = (L.x + F.y) / s, z = (U.x + F.z) / s;
  } else if (L.y > U.z) {
    const double s = std::sqrt(1.0 + L.y - F.x - U.z) * 2.0;
    w = (U.x - F.z) / s, x = (L.x + F.y) / s, y = 0.25 * s, z = (U.y + L.z) / s;
  } else {
    const double s = std::sqrt(1.0 + U.z - F.x - L.y) * 2.0;
    w = (F.y - L.x) / s, x = (U.x + F.z) / s, y = (U.y + L.z) / s, z = 0.25 * s;
  }
  q[0] = packUnit(w), q[1] = packUnit(x), q[2] = packUnit(y), q[3] = packUnit(z);
}

inline void unpackAxes(const i16 q[4], V3& F, V3& L, V3& U) {
  double w = q[0], x = q[1], y = q[2], z = q[3];
  const double l = std::sqrt(w * w + x * x + y * y + z * z);
  if (l < 1.0) {
    F = V3{1, 0, 0}, L = V3{0, 1, 0}, U = V3{0, 0, 1};
    return;
  }
  w /= l, x /= l, y /= l, z /= l;
  F = V3{1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)};
  L = V3{2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)};
  U = V3{2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)};
}

struct VehiclePose {
  V3 cog;
  i16 q[4] = {32767, 0, 0, 0};
  float yaw = 0, pitch = 0, recoil = 0;  // of the gun, on a tank
};

// One node, in the frame of the part it belongs to, in steps of kNodeScale. Server and player work this out alike.
class VehicleShapes {
 public:
  bool ready = false;
  std::vector<i16> restTanks, restCars;  // where everything is at the start of a match

  void build(const Duel& d) {
    tankNodes_ = d.tankNodes;
    u32 total = 0;
    for (int k = 0; k < Duel::kCars; k++) {
      carStart_[k] = total;
      carCount_[k] = (k + 1 < Duel::kCars ? d.carFirst[k + 1] : d.carsEnd) - d.carFirst[k];
      total += carCount_[k];
    }
    restTanks.resize(size_t(Duel::kTanks) * tankNodes_ * 3);
    restCars.resize(size_t(total) * 3);
    for (int i = 0; i < Duel::kTanks; i++) {
      const Tank& t = d.tanksStart[i];
      group_[i].assign(tankNodes_, u8(0));
      const u32 first = d.tankFirst[i];
      for (u32 n : t.turretNodes)
        if (n - first < tankNodes_) group_[i][n - first] = 1;
      for (u32 n : t.gunNodes)
        if (n - first < tankNodes_) group_[i][n - first] = 2;
      const V3 O0 = (d.simStart.rest[t.refFL] + d.simStart.rest[t.refFR] + d.simStart.rest[t.refRL] + d.simStart.rest[t.refRR]) * 0.25;
      ringLocal_[i] = d.simStart.rest[t.turretNodes[0]] - t.turretRest[0] - O0;
      trunnion_[i] = t.trunnion;
      tankLocals(i, tankPose(t), d.simStart.pos.data() + first, restTanks.data() + tankAt(i));
    }
    for (int k = 0; k < Duel::kCars; k++) {
      const Vehicle& v = d.carsStart[size_t(k)];
      VehiclePose pose;
      pose.cog = v.cog;
      packAxes(v.fwd, v.left, v.up, pose.q);
      carLocals(k, pose, d.simStart.pos.data() + d.carFirst[k], restCars.data() + carAt(k));
    }
    ready = true;
  }

  static VehiclePose tankPose(const Tank& t) {
    VehiclePose p;
    p.cog = t.cog;
    packAxes(t.fwd, t.left, t.up, p.q);
    p.yaw = float(t.turretYaw);
    p.pitch = float(t.gunPitch);
    p.recoil = float(t.recoil);
    return p;
  }

  size_t tankAt(int i) const { return size_t(i) * tankNodes_ * 3; }
  size_t carAt(int k) const { return size_t(carStart_[k]) * 3; }
  u32 carCount(int k) const { return carCount_[k]; }

  void tankLocals(int i, const VehiclePose& pose, const V3* nodes, i16* out) const {
    Part parts[3];
    tankParts(i, pose, parts);
    for (u32 n = 0; n < tankNodes_; n++) {
      const Part& p = parts[group_[i][n]];
      const V3 d = nodes[n] - p.at;
      out[n * 3] = packOffset(dot(d, p.x));
      out[n * 3 + 1] = packOffset(dot(d, p.y));
      out[n * 3 + 2] = packOffset(dot(d, p.z));
    }
  }
  // Back again: each node against the vehicle's middle, in the world's axes.
  void tankOffsets(int i, const VehiclePose& pose, const i16* locals, float* out) const {
    Part parts[3];
    tankParts(i, pose, parts);
    for (u32 n = 0; n < tankNodes_; n++) unpackNode(parts[group_[i][n]], pose.cog, locals + n * 3, out + n * 3);
  }
  void carLocals(int k, const VehiclePose& pose, const V3* nodes, i16* out) const {
    Part p = carPart(pose);
    for (u32 n = 0; n < carCount_[k]; n++) {
      const V3 d = nodes[n] - p.at;
      out[n * 3] = packOffset(dot(d, p.x));
      out[n * 3 + 1] = packOffset(dot(d, p.y));
      out[n * 3 + 2] = packOffset(dot(d, p.z));
    }
  }
  void carOffsets(int k, const VehiclePose& pose, const i16* locals, float* out) const {
    Part p = carPart(pose);
    for (u32 n = 0; n < carCount_[k]; n++) unpackNode(p, pose.cog, locals + n * 3, out + n * 3);
  }

 private:
  struct Part {
    V3 at, x, y, z;
  };
  static void unpackNode(const Part& p, V3 cog, const i16* l, float* out) {
    const V3 o = p.at - cog + (p.x * double(l[0]) + p.y * double(l[1]) + p.z * double(l[2])) * (1.0 / kNodeScale);
    out[0] = float(o.x), out[1] = float(o.y), out[2] = float(o.z);
  }
  static Part carPart(const VehiclePose& pose) {
    Part p;
    p.at = pose.cog;
    unpackAxes(pose.q, p.x, p.y, p.z);
    return p;
  }
  // The hull, the turret on its ring, and the gun on its trunnions.
  void tankParts(int i, const VehiclePose& pose, Part parts[3]) const {
    V3 F, L, U;
    unpackAxes(pose.q, F, L, U);
    parts[0] = Part{pose.cog, F, L, U};
    const double cy = std::cos(double(pose.yaw)), sy = std::sin(double(pose.yaw));
    const V3 eF = F * cy + L * sy, eL = L * cy - F * sy;
    const V3& q = ringLocal_[i];
    const V3 ring = pose.cog + U * 0.45 + F * q.x + L * q.y + U * q.z;
    parts[1] = Part{ring, eF, eL, U};
    const double cp = std::cos(double(pose.pitch)), sp = std::sin(double(pose.pitch));
    const V3 gF = eF * cp + U * sp, gU = U * cp - eF * sp;
    const V3& t = trunnion_[i];
    parts[2] = Part{ring + eF * t.x + eL * t.y + U * t.z - gF * double(pose.recoil), gF, eL, gU};
  }
  u32 tankNodes_ = 0;
  u32 carStart_[Duel::kCars] = {}, carCount_[Duel::kCars] = {};
  std::vector<u8> group_[Duel::kTanks];
  V3 ringLocal_[Duel::kTanks], trunnion_[Duel::kTanks];
};

}  // namespace net
