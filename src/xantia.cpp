// Citroen Xantia Activa (X1/X2, 3.0 V6) as a node/beam structure.
//
// Main dimensions follow the real car: length 4.52 m, width 1.755 m, height ~1.40 m, wheelbase
// 2.74 m, ~1500 kg. The body is lofted from cross-sections: 22 stations along the car, 9 rows
// around half a section (mirrored). Everything is built in "design" coordinates, with x measured
// from the nose rearwards, then mapped into the world with +X pointing forwards.
#include <algorithm>
#include <set>
#include "vehicle.h"

namespace {
constexpr int NS = 22, NR = 9;

// Station positions and profile data. Wheel arches sit around x = 0.90 and x = 3.64.
const double X[NS] = {0.00, 0.14, 0.33, 0.55, 0.70, 0.90, 1.10, 1.25, 1.42, 1.70, 2.00,
                      2.35, 2.70, 3.05, 3.29, 3.44, 3.64, 3.84, 3.99, 4.22, 4.42, 4.52};
const double ZTOP[NS] = {0.60, 0.69, 0.73, 0.77, 0.80, 0.84, 0.89, 0.93, 1.04, 1.20, 1.36,
                         1.395, 1.39, 1.345, 1.29, 1.24, 1.17, 1.105, 1.06, 1.03, 1.02, 0.93};
const double W[NS] = {0.58, 0.73, 0.82, 0.86, 0.872, 0.8775, 0.8775, 0.8775, 0.8775, 0.8775, 0.8775,
                      0.8775, 0.8775, 0.8775, 0.8775, 0.8775, 0.8775, 0.865, 0.85, 0.82, 0.76, 0.66};
const double ZBELT[NS] = {0.52, 0.60, 0.66, 0.70, 0.73, 0.76, 0.80, 0.84, 0.88, 0.90, 0.92,
                          0.93, 0.94, 0.96, 0.98, 0.99, 1.00, 0.995, 0.99, 0.97, 0.95, 0.85};
const double ZRAIL[NS] = {0.58, 0.67, 0.71, 0.75, 0.78, 0.82, 0.87, 0.91, 1.01, 1.16, 1.30,
                          1.315, 1.31, 1.26, 1.21, 1.17, 1.115, 1.06, 1.03, 1.02, 1.01, 0.92};
const double WRAIL[NS] = {0.40, 0.52, 0.60, 0.64, 0.66, 0.67, 0.68, 0.69, 0.70, 0.67, 0.63,
                          0.62, 0.62, 0.62, 0.62, 0.62, 0.62, 0.62, 0.62, 0.60, 0.56, 0.48};
const double ZLOW[NS] = {0.38, 0.36, 0.34, 0.43, 0.58, 0.68, 0.58, 0.43, 0.36, 0.36, 0.36,
                         0.36, 0.36, 0.36, 0.43, 0.58, 0.68, 0.58, 0.43, 0.36, 0.34, 0.38};
const double ZFLOOR[NS] = {0.33, 0.30, 0.26, 0.23, 0.21, 0.20, 0.20, 0.20, 0.19, 0.19, 0.19,
                           0.19, 0.19, 0.19, 0.20, 0.21, 0.23, 0.25, 0.26, 0.28, 0.30, 0.33};
const double WFLOOR[NS] = {0.50, 0.60, 0.66, 0.50, 0.50, 0.50, 0.50, 0.50, 0.83, 0.83, 0.83,
                           0.83, 0.83, 0.83, 0.50, 0.50, 0.50, 0.50, 0.50, 0.70, 0.66, 0.55};

inline bool archStation(int s) { return (s >= 3 && s <= 7) || (s >= 14 && s <= 18); }

// Design-space position of a grid point on the left side.
V3 design(int s, int r) {
  s = std::clamp(s, 0, NS - 1);
  const double x = X[s];
  switch (r) {
    // The transmission tunnel arches the floor. A flat sheet of beams has no out-of-plane stiffness.
    case 0: return {x, 0.0, ZFLOOR[s] + (s >= 8 && s <= 13 ? 0.09 : 0.0)};
    case 1: return {x, WFLOOR[s] * 0.5, ZFLOOR[s]};
    case 2: return {x, WFLOOR[s], ZFLOOR[s] - 0.005};
    case 3: return {x, W[s] * (s < 3 || s > 18 ? 0.98 : 1.0), ZLOW[s]};
    case 4: {
      const double z = 0.5 * (ZLOW[s] + ZBELT[s]);
      return {x, W[s] * 1.0, z};
    }
    case 5: return {x, W[s] * 0.985, ZBELT[s]};
    case 6: return {x, WRAIL[s], ZRAIL[s]};
    case 7: return {x, WRAIL[s] * 0.52, ZTOP[s] - 0.008};
    case 8: return {x, 0.0, ZTOP[s] + (s >= 10 && s <= 13 ? 0.02 : 0.0)};
  }
  return {};
}

V3 catmull(V3 a, V3 b, V3 c, V3 d, double t) {
  const double t2 = t * t, t3 = t2 * t;
  return (b * 2.0 + (c - a) * t + (a * 2.0 - b * 5.0 + c * 4.0 - d) * t2 +
          (b * 3.0 - a - c * 3.0 + d) * t3) * 0.5;
}

// Smooth reference surface, used so the visible skin is curved although the structure is coarse.
V3 surfaceAlongS(int r, double su) {
  const int s = std::clamp(int(su), 0, NS - 2);
  const double t = su - s;
  V3 a = design(s - 1, r), b = design(s, r), c = design(s + 1, r), d = design(s + 2, r);
  // Stations are unevenly spaced and the arches are sharp; keep x linear to avoid folds.
  V3 p = catmull(a, b, c, d, t);
  p.x = lerpd(b.x, c.x, t);
  return p;
}
V3 smoothSurface(double su, double rv) {
  const int r = std::clamp(int(rv), 0, NR - 2);
  const double t = rv - r;
  if (r < 3) return lerpv(surfaceAlongS(r, su), surfaceAlongS(r + 1, su), t);
  auto at = [&](int rr) {
    if (rr < 3) return surfaceAlongS(3, su);
    if (rr > 8) {  // mirror across the centreline for a continuous crown
      V3 m = surfaceAlongS(7, su);
      m.y = -m.y;
      return m;
    }
    return surfaceAlongS(rr, su);
  };
  return catmull(at(r - 1), at(r), at(r + 1), at(r + 2), t);
}

// ---- Beam materials. Stiffness in N/m, forces in N.
const Mat mBumper{0.8e6, 160, 7e3, 14e3, 2e4, 0.80, 0};
const Mat mShell{1.2e6, 240, 6e3, 14e3, 4e4, 0.45, 0};
const Mat mFender{0.9e6, 180, 4e3, 10e3, 3e4, 0.50, 0};
const Mat mRail{2.0e6, 340, 34e3, 70e3, 1.2e5, 0.60, 0};
const Mat mUpperRail{1.6e6, 280, 15e3, 40e3, 6e4, 0.55, 0};
const Mat mRadSupport{1.0e6, 180, 10e3, 22e3, 3e4, 0.40, 0};
const Mat mFloor{1.8e6, 320, 42e3, 80e3, 1.5e5, 0.40, 0};
const Mat mCabin{2.5e6, 420, 95e3, 140e3, 3e5, 0.35, 0};
const Mat mRoof{1.3e6, 240, 12e3, 26e3, 5e4, 0.45, 0};
const Mat mRoofRail{2.0e6, 340, 55e3, 90e3, 1.6e5, 0.35, 0};
const Mat mRear{1.5e6, 260, 22e3, 50e3, 8e4, 0.55, 0};
const Mat mQuarter{1.1e6, 200, 7e3, 16e3, 3e4, 0.45, 0};
const Mat mBulkhead{1.6e6, 280, 30e3, 60e3, 1e5, 0.35, 0};
const Mat mSubframe{3.0e6, 440, 70e3, 90e3, 2e5, 0.22, 0};
const Mat mInnerWing{1.8e6, 300, 32e3, 55e3, 1e5, 0.40, 0};
const Mat mAnchor{3.0e6, 440, 65e3, 85e3, 2e5, 0.20, 0};
const Mat mArm{4.0e6, 500, 55e3, 75e3, 3e5, 0.22, 0};
const Mat mHub{2.2e6, 280, 16e3, 26e3, 1.2e5, 0.25, 0};  // a wheel bends long before an arm does
const Mat mCarrier{3.2e6, 420, 90e3, 110e3, 4e5, 0.20, 0};
const Mat mTie{3.0e6, 360, 26e3, 34e3, 1e5, 0.12, 0};
const Mat mBlock{6e6, 900, 240e3, 240e3, 6e5, 0.0, 0};
const Mat mMount{4.5e5, 500, 16e3, 18e3, 3e4, 0.45, 0};
const Mat mCase{5.0e6, 800, 160e3, 160e3, 5e5, 0.0, 0};
const Mat mBellBolts{5.0e6, 800, 0, 0, 0, 0, 120e3};
const Mat mRadCore{0.5e6, 90, 6e3, 8e3, 1e4, 0.20, 0};
const Mat mRadMount{1.0e6, 160, 0, 0, 0, 0, 14e3};
const Mat mTank{1.2e6, 240, 18e3, 22e3, 3e4, 0.30, 0};
const Mat mFlange{1.5e6, 200, 0, 0, 0, 0, 0};
const Mat mBattery{0.8e6, 150, 0, 0, 0, 0, 0};
const Mat mDoor{0.8e6, 150, 5e3, 11e3, 3e4, 0.50, 0};
const Mat mDoorBar{1.0e6, 180, 28e3, 50e3, 1e5, 0.40, 0};
const Mat mHinge{1.2e6, 220, 0, 0, 0, 0, 17e3};
const Mat mLatch{0.8e6, 160, 0, 0, 0, 0, 9e3};
const Mat mPanel{0.7e6, 110, 1.8e3, 5e3, 1e4, 0.60, 0};
const Mat mPanelHinge{0.8e6, 140, 0, 0, 0, 0, 9e3};
const Mat mPanelLatch{0.5e6, 100, 0, 0, 0, 0, 5e3};
const Mat mClip{0.3e6, 70, 0, 0, 0, 0, 7e3};

struct Builder {
  Sim& sim;
  V3 origin;
  double cy, sy;
  std::set<std::pair<u32, u32>> seen;

  int body[NS][NR][2];
  int hood[NS][NR][2];
  int lid[NS][NR][2];
  struct Door { int s0, s1; int n[4][4]; std::vector<u32> inner; };  // n[station-s0][row-3]
  Door doors[2][2];  // [door][side]
  std::vector<u32> bodyNodes;

  V3 world(V3 d, int side) const {
    const double y = side == 0 ? d.y : -d.y;
    const double fx = 2.26 - d.x;
    return origin + V3{cy * fx - sy * y, sy * fx + cy * y, d.z};
  }
  V3 worldDir(double dx, double dy) const { return {cy * dx - sy * dy, sy * dx + cy * dy, 0}; }

  bool beam(u32 a, u32 b, const Mat& m, std::vector<u32>* list = nullptr, u8 flags = 0) {
    if (a == b) return false;
    auto key = std::minmax(a, b);
    if (!seen.insert({key.first, key.second}).second) return false;
    const u32 id = sim.addBeam(a, b, m, 0, flags);
    if (list) list->push_back(id);
    return true;
  }
  bool ibeam(int a, int b, const Mat& m, std::vector<u32>* list = nullptr) {
    if (a < 0 || b < 0) return false;
    return beam(u32(a), u32(b), m, list);
  }

  // Connect an auxiliary node to the n nearest body nodes on its side of the car.
  std::vector<u32> anchor(u32 node, int n, const Mat& m, std::vector<u32>* list = nullptr) {
    std::vector<std::pair<double, u32>> cand;
    const V3 p = sim.pos[node];
    for (u32 b : bodyNodes) cand.push_back({len2(sim.pos[b] - p), b});
    std::sort(cand.begin(), cand.end());
    std::vector<u32> out;
    for (auto& c : cand) {
      if (c.first < 0.0025) continue;
      beam(node, c.second, m, list);
      out.push_back(c.second);
      if (int(out.size()) >= n) break;
    }
    return out;
  }

  void orientedTri(u32 a, u32 b, u32 c, V3 want, u8 fl, float mu = 0.5f) {
    const V3 n = cross(sim.pos[b] - sim.pos[a], sim.pos[c] - sim.pos[a]);
    if (dot(n, want) >= 0)
      sim.addTri(a, b, c, fl, 0, mu);
    else
      sim.addTri(a, c, b, fl, 0, mu);
  }
};
}  // namespace

Vehicle buildXantia(Sim& sim, V3 spawn, double yaw, CarRig* rig) {
  Vehicle veh;
  veh.obj = sim.beginObj(OBJ_CAR, true);
  Builder B{sim, spawn, std::cos(yaw), std::sin(yaw), {}, {}, {}, {}, {}, {}};
  for (auto& a : B.body) for (auto& b : a) b[0] = b[1] = -1;
  for (auto& a : B.hood) for (auto& b : a) b[0] = b[1] = -1;
  for (auto& a : B.lid) for (auto& b : a) b[0] = b[1] = -1;

  const auto isFrontDoorStation = [](int s) { return s >= 8 && s <= 11; };
  const auto isRearDoorStation = [](int s) { return s >= 11 && s <= 13; };
  const auto isPillar = [](int s) { return s == 8 || s == 11 || s == 13; };
  const auto inHood = [](int s, int r) { return s >= 2 && s <= 7 && r >= 6; };
  const auto inLid = [](int s, int r) { return s >= 14 && s <= 20 && r >= 7; };

  // ---- Shell nodes.
  double massBase[NR] = {3.0, 3.0, 4.2, 3.6, 2.6, 2.8, 2.8, 2.0, 2.0};
  std::vector<u32> shell;
  for (int s = 0; s < NS; s++)
    for (int r = 0; r < NR; r++) {
      const bool doorOnly = (r == 4 || r == 5) && (isFrontDoorStation(s) || isRearDoorStation(s)) && !isPillar(s);
      if (doorOnly || inHood(s, r) || inLid(s, r)) continue;
      const bool center = r == 0 || r == 8;
      for (int side = 0; side < (center ? 1 : 2); side++) {
        const u32 id = sim.addNode(B.world(design(s, r), side), massBase[r], NF_COLLIDE, 0.45f);
        B.body[s][r][side] = int(id);
        if (center) B.body[s][r][1] = int(id);
        shell.push_back(id);
      }
    }
  B.bodyNodes = shell;

  // ---- Shell lattice.
  auto matLong = [&](int s, int r) -> const Mat& {
    const bool front = s < 8, rear = s >= 13;
    if (s <= 1) return mBumper;
    if (r == 1 || r == 2) {
      if (front) return mRail;
      if (rear) return mRear;
      return r == 2 ? mCabin : mFloor;
    }
    if (r == 0) return front ? mFender : (rear ? mRear : mFloor);
    if (r == 3) return front ? mFender : (rear ? mQuarter : mCabin);
    if (r == 4) return front ? mFender : mQuarter;
    if (r == 5) return front ? mUpperRail : mQuarter;
    if (r == 6) return rear && s >= 15 ? mQuarter : mRoofRail;
    return mRoof;
  };
  auto matAround = [&](int s, int r) -> const Mat& {
    if (s <= 1 || s >= 20) return mBumper;
    if (isPillar(s) && r >= 2 && r <= 5) return mCabin;
    if (r <= 1) return (s >= 8 && s <= 13) ? mFloor : mRadSupport;
    if (s < 8) return r <= 2 ? mRadSupport : mFender;
    if (s >= 14) return r <= 2 ? mRadSupport : mQuarter;
    return r >= 6 ? mRoof : mShell;
  };

  for (int s = 0; s < NS; s++)
    for (int r = 0; r < NR; r++)
      for (int side = 0; side < 2; side++) {
        const int a = B.body[s][r][side];
        if (a < 0) continue;
        if (s + 1 < NS) {
          B.ibeam(a, B.body[s + 1][r][side], matLong(s, r));
          if (r + 1 < NR) {
            const Mat& dm = matAround(s, r);
            B.ibeam(a, B.body[s + 1][r + 1][side], dm);
            B.ibeam(B.body[s + 1][r][side], B.body[s][r + 1][side], dm);
          }
        }
        if (r + 1 < NR) B.ibeam(a, B.body[s][r + 1][side], matAround(s, r));
        // Skip beams give the sheet some bending stiffness.
        if (s + 2 < NS) B.ibeam(a, B.body[s + 2][r][side], matLong(s, r).yC > 2e4 ? mShell : mFender);
        if (r + 2 < NR) B.ibeam(a, B.body[s][r + 2][side], mFender);
      }

  // A pillar crossing the door opening: rocker to roof rail, skipping the door-only rows.
  for (int s : {8, 11, 13})
    for (int side = 0; side < 2; side++) {
      B.ibeam(B.body[s][3][side], B.body[s][6][side], mCabin);
      B.ibeam(B.body[s][4][side], B.body[s][6][side], mCabin);
      B.ibeam(B.body[s][3][side], B.body[s][5][side], mCabin);
    }

  // Bulkheads and cross members tie the left and right halves together.
  auto across = [&](int s, int r1, int r2, const Mat& m) {
    B.ibeam(B.body[s][r1][0], B.body[s][r2][1], m);
    if (r1 != r2) B.ibeam(B.body[s][r2][0], B.body[s][r1][1], m);
  };
  for (int r1 = 1; r1 < NR - 1; r1++)
    for (int r2 = r1; r2 < std::min(NR - 1, r1 + 3); r2++) {
      across(2, r1, r2, mRadSupport);
      across(8, r1, r2, mBulkhead);
      across(14, r1, r2, mBulkhead);
      across(19, r1, r2, mRadSupport);
      across(1, r1, r2, mBumper);
      across(20, r1, r2, mBumper);
    }
  // Floor cross members and roof bows; the cabin itself stays open.
  for (int s : {9, 10, 11, 12, 13}) {
    across(s, 2, 2, mFloor);
    across(s, 3, 3, mFloor);
    across(s, 2, 3, mBulkhead);
    across(s, 6, 6, mRoof);
    across(s, 6, 7, mRoof);
  }
  // Floor tunnel: longitudinal stiffening spanning two stations.
  for (int s = 8; s < 13; s++)
    for (int side = 0; side < 2; side++) {
      B.ibeam(B.body[s][1][side], B.body[s + 2][1][side], mFloor);
      B.ibeam(B.body[s][0][side], B.body[s + 2][1][side], mFloor);
    }

  // ---- Doors.
  auto buildDoor = [&](int d, int side, int s0, int s1) {
    Builder::Door& D = B.doors[d][side];
    D.s0 = s0;
    D.s1 = s1;
    const int ns = s1 - s0 + 1;
    for (int i = 0; i < ns; i++)
      for (int r = 3; r <= 6; r++) {
        V3 p = design(s0 + i, r);
        D.n[i][r - 3] = int(sim.addNode(B.world(p, side), r == 6 ? 1.0 : 1.9, NF_COLLIDE, 0.45f));
      }
    for (int i = 0; i < ns; i++)
      for (int r = 0; r < 4; r++) {
        if (i + 1 < ns) {
          B.ibeam(D.n[i][r], D.n[i + 1][r], mDoor);
          if (r + 1 < 4) {
            B.ibeam(D.n[i][r], D.n[i + 1][r + 1], mDoor);
            B.ibeam(D.n[i + 1][r], D.n[i][r + 1], mDoor);
          }
        }
        if (r + 1 < 4) B.ibeam(D.n[i][r], D.n[i][r + 1], mDoor);
      }
    B.ibeam(D.n[0][1], D.n[ns - 1][1], mDoorBar);  // side-impact bar
    B.ibeam(D.n[0][0], D.n[ns - 1][2], mDoor);
    B.ibeam(D.n[0][2], D.n[ns - 1][0], mDoor);
    // Inner door skin gives the door depth, otherwise it would be a flat sheet.
    for (int i = 0; i < ns; i++) {
      V3 p = (design(s0 + i, 3) + design(s0 + i, 4)) * 0.5;
      p.y -= 0.11;
      const int in = int(sim.addNode(B.world(p, side), 1.6, NF_COLLIDE, 0.45f));
      D.inner.push_back(u32(in));
      B.ibeam(in, D.n[i][0], mDoor);
      B.ibeam(in, D.n[i][1], mDoor);
      B.ibeam(in, D.n[i][2], mDoor);
      if (i > 0) {
        B.ibeam(in, D.n[i - 1][0], mDoor);
        B.ibeam(in, D.n[i - 1][1], mDoor);
      }
      if (i + 1 < ns) {
        B.ibeam(in, D.n[i + 1][0], mDoor);
        B.ibeam(in, D.n[i + 1][1], mDoor);
      }
    }
    auto bd = [&](int s, int r) { return B.body[s][r][side]; };
    // Hinges on the front edge of the door.
    B.ibeam(D.n[0][1], bd(s0, 3), mHinge);
    B.ibeam(D.n[0][1], bd(s0, 5), mHinge);
    B.ibeam(D.n[0][1], bd(s0 - 1, 3) >= 0 ? bd(s0 - 1, 3) : bd(s0, 2), mHinge);
    B.ibeam(D.n[0][1], bd(s0, 2), mHinge);
    B.ibeam(D.n[0][2], bd(s0, 4), mHinge);
    B.ibeam(D.n[0][2], bd(s0, 6), mHinge);
    B.ibeam(D.n[0][0], bd(s0, 2), mHinge);
    B.ibeam(D.n[0][0], bd(s0, 4), mHinge);
    // Latch on the rear edge, plus the sill and rail contacts along the opening.
    B.ibeam(D.n[ns - 1][1], bd(s1, 3), mLatch);
    B.ibeam(D.n[ns - 1][1], bd(s1, 5), mLatch);
    B.ibeam(D.n[ns - 1][2], bd(s1, 4), mLatch);
    B.ibeam(D.n[ns - 1][2], bd(s1, 6), mLatch);
    B.ibeam(D.n[ns - 1][0], bd(s1, 2), mLatch);
    B.ibeam(D.n[ns - 1][0], bd(s1, 4), mLatch);
    for (int i = 0; i < ns; i++) {
      B.ibeam(D.n[i][0], bd(s0 + i, 2), mClip);
      if (i > 0) B.ibeam(D.n[i][0], bd(s0 + i - 1, 3), mClip);
      if (i + 1 < ns) B.ibeam(D.n[i][0], bd(s0 + i + 1, 3), mClip);
      if (i > 0) B.ibeam(D.n[i][3], bd(s0 + i - 1, 6), mClip);
      if (i + 1 < ns) B.ibeam(D.n[i][3], bd(s0 + i + 1, 6), mClip);
    }
  };
  for (int side = 0; side < 2; side++) {
    buildDoor(0, side, 8, 11);
    buildDoor(1, side, 11, 13);
  }

  // ---- Bonnet (hood) and tailgate: separate panels on hinges, latches and clips.
  for (int s = 2; s <= 7; s++)
    for (int r = 6; r <= 8; r++)
      for (int side = 0; side < (r == 8 ? 1 : 2); side++) {
        const u32 id = sim.addNode(B.world(design(s, r), side), 0.9, NF_COLLIDE, 0.45f);
        B.hood[s][r][side] = int(id);
        if (r == 8) B.hood[s][r][1] = int(id);
      }
  for (int s = 14; s <= 20; s++)
    for (int r = 7; r <= 8; r++)
      for (int side = 0; side < (r == 8 ? 1 : 2); side++) {
        const u32 id = sim.addNode(B.world(design(s, r), side), 1.3, NF_COLLIDE, 0.45f);
        B.lid[s][r][side] = int(id);
        if (r == 8) B.lid[s][r][1] = int(id);
      }
  auto panelLattice = [&](int (&P)[NS][NR][2]) {
    for (int s = 0; s < NS; s++)
      for (int r = 0; r < NR; r++)
        for (int side = 0; side < 2; side++) {
          const int a = P[s][r][side];
          if (a < 0) continue;
          if (s + 1 < NS) {
            B.ibeam(a, P[s + 1][r][side], mPanel);
            if (r + 1 < NR) {
              B.ibeam(a, P[s + 1][r + 1][side], mPanel);
              B.ibeam(P[s + 1][r][side], P[s][r + 1][side], mPanel);
            }
          }
          if (r + 1 < NR) B.ibeam(a, P[s][r + 1][side], mPanel);
          if (s + 2 < NS) B.ibeam(a, P[s + 2][r][side], mPanel);
        }
  };
  panelLattice(B.hood);
  panelLattice(B.lid);
  std::vector<u32> hoodInner, lidInner;
  auto innerFrame = [&](int (&P)[NS][NR][2], std::vector<u32>& out, std::initializer_list<int> stations, int rowOuter, double depth, double mass) {
    for (int s : stations)
      for (int side = 0; side < 2; side++) {
        V3 d = design(s, rowOuter);
        d.y *= 0.72;
        d.z -= depth;
        const u32 in = sim.addNode(B.world(d, side), mass, NF_COLLIDE, 0.45f);
        out.push_back(in);
        for (int ss = s - 1; ss <= s + 1; ss++)
          for (int r = rowOuter; r < NR; r++) B.ibeam(int(in), P[std::clamp(ss, 0, NS - 1)][r][side], mPanel);
      }
  };
  innerFrame(B.hood, hoodInner, {3, 5, 7}, 7, 0.07, 1.0);
  innerFrame(B.lid, lidInner, {15, 17, 19}, 7, 0.07, 1.2);
  // The tailgate is as wide as the roof rails, far wider than its centre rows: give it an outer edge.
  for (int s = 14; s <= 20; s++)
    for (int side = 0; side < 2; side++) {
      V3 d = design(s, 6);
      d.y -= 0.03;
      const u32 e = sim.addNode(B.world(d, side), 0.9, NF_COLLIDE, 0.45f);
      lidInner.push_back(e);
      B.ibeam(int(e), B.lid[s][7][side], mPanel);
      B.ibeam(int(e), B.lid[s][8][side], mPanel);
      if (s > 14) {
        B.ibeam(int(e), B.lid[s - 1][7][side], mPanel);
        B.beam(e, lidInner[lidInner.size() - 3], mPanel);  // same side, previous station
      }
      if (s < 20) B.ibeam(int(e), B.lid[s + 1][7][side], mPanel);
      B.ibeam(int(e), B.body[s][6][side], mClip);
    }
  for (int side = 0; side < 2; side++) {
    // Hood: hinges at the cowl, latch at the front, clips along the fenders.
    const int hr = B.hood[7][7][side];
    B.ibeam(hr, B.body[8][6][side], mPanelHinge);
    B.ibeam(hr, B.body[8][7][side], mPanelHinge);
    B.ibeam(hr, B.body[8][8][side], mPanelHinge);
    B.ibeam(hr, B.body[7][5][side], mPanelHinge);
    B.ibeam(B.hood[7][6][side], B.body[8][6][side], mPanelHinge);
    B.ibeam(B.hood[7][6][side], B.body[7][5][side], mPanelHinge);
    B.ibeam(B.hood[2][8][0], B.body[2][5][side], mPanelLatch);
    B.ibeam(B.hood[2][7][side], B.body[2][5][side], mPanelLatch);
    B.ibeam(B.hood[2][7][side], B.body[1][7][side], mPanelLatch);
    for (int s = 2; s <= 7; s++) {
      B.ibeam(B.hood[s][6][side], B.body[s][5][side], mClip);
      if (s > 2) B.ibeam(B.hood[s][6][side], B.body[s - 1][5][side], mClip);
      if (s < 7) B.ibeam(B.hood[s][6][side], B.body[s + 1][5][side], mClip);
    }
    // Tailgate: hinges at the roof, latch at the tail.
    const int lf = B.lid[14][7][side];
    B.ibeam(lf, B.body[13][6][side], mPanelHinge);
    B.ibeam(lf, B.body[13][7][side], mPanelHinge);
    B.ibeam(lf, B.body[13][8][side], mPanelHinge);
    B.ibeam(lf, B.body[14][6][side], mPanelHinge);
    B.ibeam(B.lid[14][8][0], B.body[13][8][side], mPanelHinge);
    B.ibeam(B.lid[20][7][side], B.body[20][5][side], mPanelLatch);
    B.ibeam(B.lid[20][7][side], B.body[20][6][side], mPanelLatch);
    B.ibeam(B.lid[20][7][side], B.body[21][7][side], mPanelLatch);
    B.ibeam(B.lid[20][8][0], B.body[21][8][0], mPanelLatch);
    for (int s = 14; s <= 20; s++) {
      B.ibeam(B.lid[s][7][side], B.body[s][6][side], mClip);
      if (s > 14) B.ibeam(B.lid[s][7][side], B.body[s - 1][6][side], mClip);
      if (s < 20) B.ibeam(B.lid[s][7][side], B.body[s + 1][6][side], mClip);
    }
  }

  // ---- Engine and gearbox: two heavy, nearly rigid blocks bolted together, on breakable mounts.
  auto rigidBlock = [&](double x0, double x1, double y0, double y1, double z0, double z1, double mass, std::vector<u32>& out, const Mat& m, std::vector<u32>* beamList) {
    u32 e[8];
    for (int i = 0; i < 8; i++) {
      const V3 d{(i & 1) ? x1 : x0, ((i & 2) ? y1 : y0), (i & 4) ? z1 : z0};
      e[i] = sim.addNode(B.world(d, 0), mass / 8.0, NF_COLLIDE, 0.4f);
      out.push_back(e[i]);
    }
    for (int i = 0; i < 8; i++)
      for (int j = i + 1; j < 8; j++) B.beam(e[i], e[j], m, beamList);
    static const int faces[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4}, {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
    V3 c{};
    for (u32 n : e) c += sim.pos[n];
    c = c * 0.125;
    for (auto& f : faces) {
      const V3 fc = (sim.pos[e[f[0]]] + sim.pos[e[f[2]]]) * 0.5;
      B.orientedTri(e[f[0]], e[f[1]], e[f[2]], fc - c, TF_HARD | TF_NOVIS);
      B.orientedTri(e[f[0]], e[f[2]], e[f[3]], fc - c, TF_HARD | TF_NOVIS);
    }
  };
  rigidBlock(0.52, 0.88, -0.33, 0.15, 0.24, 0.72, 165.0, veh.engineNodes, mBlock, &veh.engineBlockBeams);
  rigidBlock(0.55, 0.86, 0.19, 0.46, 0.22, 0.56, 45.0, veh.gearboxNodes, mCase, &veh.gearboxBeams);
  // Bell housing bolts: the two units only part company in a very heavy impact.
  for (int i : {2, 3, 6, 7})
    for (int j : {0, 1, 4, 5}) B.beam(veh.engineNodes[size_t(i)], veh.gearboxNodes[size_t(j)], mBellBolts, &veh.bellBeams);
  for (u32 n : veh.engineNodes) B.anchor(n, 2, mMount, &veh.engineMountBeams);
  for (u32 n : veh.gearboxNodes) B.anchor(n, 1, mMount, &veh.engineMountBeams);

  // Halfshaft output flanges on the differential.
  const u32 outL = sim.addNode(B.world({0.78, 0.46, 0.30}, 0), 4.0, 0, 0.4f);
  const u32 outR = sim.addNode(B.world({0.78, -0.30, 0.30}, 0), 4.0, 0, 0.4f);
  for (int j : {1, 3, 5, 7}) B.beam(outL, veh.gearboxNodes[size_t(j)], mFlange);
  for (int j : {1, 0, 5, 3}) B.beam(outR, veh.engineNodes[size_t(j)], mFlange);
  veh.shaftNode[0] = outL;
  veh.shaftNode[1] = outR;

  // Radiator pack: light and fragile, right behind the bumper.
  std::vector<u32> radNodes;
  rigidBlock(0.27, 0.34, -0.55, 0.55, 0.26, 0.68, 14.0, radNodes, mRadCore, &veh.radiatorBeams);
  for (u32 n : radNodes) B.anchor(n, 3, mRadMount);
  // Battery on the left of the bay, fuel tank under the rear seat.
  std::vector<u32> batNodes, tankNodes;
  rigidBlock(0.95, 1.13, 0.37, 0.63, 0.49, 0.67, 18.0, batNodes, mBattery, nullptr);
  for (u32 n : batNodes) B.anchor(n, 2, mRadMount);
  rigidBlock(2.95, 3.55, -0.56, 0.56, 0.25, 0.42, 60.0, tankNodes, mTank, &veh.tankBeams);
  for (u32 n : tankNodes) B.anchor(n, 3, mRadMount);

  // ---- Suspension and wheels.
  struct Sub { u32 uf = 0, ur = 0, rack = 0, po = 0, pi = 0; } sub[2];
  auto addLink = [&](u32 a, u32 b, float r, u8 mat, u8 kind = 0) {
    if (rig) rig->links.push_back({a, b, r, mat, kind});
  };
  auto makeWheel = [&](bool front, int side) {
    Wheel w;
    w.front = front;
    w.left = side == 0;
    w.driven = front;
    w.brakeMax = front ? 2300.0 : 900.0;
    w.R = 0.312;
    const double hx = front ? 0.90 : 3.64;
    const double hy = front ? 0.745 : 0.735;
    const double hz = 0.312;
    auto node = [&](double x, double y, double z, double m, u8 fl = NF_COLLIDE) {
      return sim.addNode(B.world({x, y, z}, side), m, fl, 0.5f);
    };
    w.axIn = node(hx, hy - 0.07, hz, 5.0, 0);
    w.axOut = node(hx, hy + 0.07, hz, 5.0, 0);
    for (int i = 0; i < 6; i++) {
      const double ang = PI / 3.0 * i;
      w.ring[i] = node(hx - std::sin(ang) * 0.24, hy, hz + std::cos(ang) * 0.24, 3.0,
                       NF_COLLIDE | NF_WHEEL);
    }
    // Hub-and-rim lattice. Plastic spokes mean a heavy kerb strike leaves a bent wheel.
    B.beam(w.axIn, w.axOut, mHub);
    for (int i = 0; i < 6; i++) {
      w.spoke[i][0] = u32(sim.beams.size());
      B.beam(w.ring[i], w.axIn, mHub);
      w.spoke[i][1] = u32(sim.beams.size());
      B.beam(w.ring[i], w.axOut, mHub);
      B.beam(w.ring[i], w.ring[(i + 1) % 6], mHub);
      B.beam(w.ring[i], w.ring[(i + 2) % 6], mHub);
    }

    HydroStrut st;
    if (front) {
      // MacPherson strut, kinematically approximated by a long virtual upper link.
      // A nearly level lower arm keeps the roll centre low. A high one would use cornering force to
      // jack the body up, and on soft gas springs that lift is large.
      const u32 kl = node(hx, hy - 0.055, 0.195, 4.0, 0);
      const u32 ku = node(hx + 0.015, hy - 0.115, 0.46, 4.0, 0);
      const u32 ks = node(hx + 0.13, hy - 0.125, 0.24, 2.5, 0);
      // The carrier braces the axle from both sides: lateral force must not twist the wheel.
      const u32 kf = node(hx - 0.13, hy - 0.10, 0.27, 2.5, 0);
      for (u32 a : {w.axIn, w.axOut}) {
        B.beam(kl, a, mCarrier);
        B.beam(ku, a, mCarrier);
        B.beam(ks, a, mCarrier);
        B.beam(kf, a, mCarrier);
      }
      B.beam(kl, ku, mCarrier);
      B.beam(kl, ks, mCarrier);
      B.beam(ku, ks, mCarrier);
      B.beam(kf, kl, mCarrier);
      B.beam(kf, ku, mCarrier);
      B.beam(kf, ks, mCarrier);
      for (int i = 0; i < 6; i++)
        for (u32 c : {kl, ku, ks, kf}) B.beam(w.ring[i], c, mHub);

      // Lower wishbone to the two rail pick-ups.
      B.ibeam(kl, B.body[4][1][side], mArm);
      B.ibeam(kl, B.body[6][1][side], mArm);

      // Virtual upper link pick-ups, braced across the car.
      const u32 uf = node(0.72, 0.24, 0.445, 4.0, 0);
      const u32 ur = node(1.08, 0.24, 0.445, 4.0, 0);
      sub[side].uf = uf;
      sub[side].ur = ur;
      B.beam(uf, ur, mAnchor);
      B.anchor(uf, 5, mAnchor);
      B.anchor(ur, 5, mAnchor);
      B.ibeam(uf, B.body[3][1][side], mAnchor);
      B.ibeam(ur, B.body[7][1][side], mAnchor);
      B.ibeam(uf, B.body[4][2][side], mAnchor);
      B.ibeam(ur, B.body[6][2][side], mAnchor);
      B.ibeam(ur, B.body[8][7][side], mAnchor);
      B.ibeam(ur, B.body[8][6][side], mAnchor);
      B.beam(ku, uf, mArm);
      B.beam(ku, ur, mArm);

      // Steering rack end.
      // The tie rod runs parallel to the lower arm, which is what keeps toe constant over the stroke.
      const u32 rack = node(1.03, 0.28, 0.25, 3.0, 0);
      sub[side].rack = rack;
      B.anchor(rack, 5, mAnchor);
      B.ibeam(rack, B.body[8][1][side], mAnchor);
      B.ibeam(rack, B.body[8][2][side], mAnchor);
      veh.tieBeam[side] = sim.addBeam(ks, rack, mTie);
      addLink(ks, rack, 0.012f, MM_METAL);
      addLink(kl, u32(B.body[4][1][side]), 0.016f, MM_DARK);
      addLink(kl, u32(B.body[6][1][side]), 0.016f, MM_DARK);
      addLink(ku, uf, 0.011f, MM_DARK);
      addLink(ku, ur, 0.011f, MM_DARK);
      addLink(w.axIn, veh.shaftNode[side], 0.019f, MM_METAL);
      addLink(kl, w.axIn, 0.03f, MM_ALU);
      addLink(ku, w.axIn, 0.03f, MM_ALU);

      // Strut tower, part of the inner wing.
      const u32 tower = node(0.92, 0.60, 0.74, 4.0, 0);
      B.anchor(tower, 4, mAnchor, &veh.hydraulicBeams);
      B.beam(tower, uf, mAnchor);
      B.beam(tower, ur, mAnchor);
      // Inner wing: the load path from the strut top into the rails and the firewall.
      for (auto [ss, rr] : {std::pair{4, 2}, {5, 2}, {6, 2}, {7, 2}, {8, 2}, {8, 3}, {8, 4}, {8, 5}, {3, 2}})
        B.ibeam(int(tower), B.body[ss][rr][side], mInnerWing);
      addLink(tower, kl, 0.022f, MM_METAL, 1);
      st.top = tower;
      st.low = kl;
      st.A = 3.85e-4;
      st.V0 = 290e-6;
      st.p0 = 48e5;
    } else {
      // Trailing arm pivoting on a transverse subframe axis.
      const u32 kt = node(hx, hy - 0.06, 0.50, 2.5, 0);
      const u32 kb = node(hx, hy - 0.06, 0.17, 3.0, 0);
      // Wide bearing base: lateral force on the wheel must not twist the arm, or the rear wheels
      // would toe out of the corner and the car would oversteer through sheer compliance.
      const u32 po = node(3.19, 0.76, 0.30, 4.0, 0);
      const u32 pi = node(3.19, 0.16, 0.30, 4.0, 0);
      sub[side].po = po;
      sub[side].pi = pi;
      for (u32 a : {w.axIn, w.axOut, kt, kb}) {
        B.beam(a, po, mArm);
        B.beam(a, pi, mArm);
      }
      for (u32 a : {w.axIn, w.axOut}) {
        B.beam(kt, a, mCarrier);
        B.beam(kb, a, mCarrier);
      }
      B.beam(kt, kb, mCarrier);
      const u32 kf = node(hx - 0.13, hy - 0.08, 0.31, 2.5, 0);
      const u32 kr = node(hx + 0.13, hy - 0.08, 0.31, 2.5, 0);
      for (u32 c : {kf, kr}) {
        B.beam(c, w.axIn, mCarrier);
        B.beam(c, w.axOut, mCarrier);
        B.beam(c, kt, mCarrier);
        B.beam(c, kb, mCarrier);
      }
      B.beam(kf, kr, mCarrier);
      B.beam(kf, po, mArm);
      B.beam(kf, pi, mArm);
      for (int i = 0; i < 6; i++)
        for (u32 c : {kt, kb, kf, kr}) B.beam(w.ring[i], c, mHub);
      B.beam(po, pi, mSubframe);
      B.anchor(po, 6, mAnchor);
      B.anchor(pi, 6, mAnchor);
      const u32 tower = node(3.64, 0.62, 0.76, 3.0, 0);
      B.anchor(tower, 4, mAnchor, &veh.hydraulicBeams);
      for (auto [ss, rr] : {std::pair{13, 2}, {13, 3}, {13, 5}, {14, 2}, {16, 2}, {18, 2}, {14, 6}, {18, 6}, {16, 1}})
        B.ibeam(int(tower), B.body[ss][rr][side], mInnerWing);
      addLink(tower, kb, 0.02f, MM_METAL, 1);
      addLink(w.axIn, po, 0.03f, MM_DARK);
      addLink(w.axIn, pi, 0.022f, MM_DARK);
      addLink(kb, po, 0.024f, MM_DARK);
      addLink(kt, po, 0.016f, MM_DARK);
      st.top = tower;
      st.low = kb;
      st.A = 2.75e-4;
      st.V0 = 300e-6;
      st.p0 = 30e5;
      st.bump[0] = 1150; st.bump[1] = 700;
      st.reb[0] = 1500; st.reb[1] = 900;
    }
    const double L = len(sim.pos[st.low] - sim.pos[st.top]);
    st.Lnom = L;
    st.L = L;
    st.Lmin = L - (front ? 0.10 : 0.12);
    st.Lmax = L + (front ? 0.11 : 0.14);
    veh.axle[front ? 0 : 1].s[side] = st;

    // Collision faces of the wheel so props and debris can hit it.
    for (int i = 0; i < 6; i++) {
      const u32 r0 = w.ring[i], r1 = w.ring[(i + 1) % 6];
      const V3 outDir = B.worldDir(0, side == 0 ? 1 : -1);
      B.orientedTri(w.axOut, r0, r1, outDir, TF_NOVIS, 0.6f);
    }
    return w;
  };
  veh.wheels.push_back(makeWheel(true, 0));
  veh.wheels.push_back(makeWheel(true, 1));
  veh.wheels.push_back(makeWheel(false, 0));
  veh.wheels.push_back(makeWheel(false, 1));
  // Subframes: stiff cross-bracing so wishbone pick-ups and rack ends move as one unit. Without
  // this the sheet metal flexes under cornering load and steers the wheels.
  {
    const int pl1 = B.body[4][1][0], pr1 = B.body[4][1][1];
    const int pl2 = B.body[6][1][0], pr2 = B.body[6][1][1];
    const u32 rl = sub[0].rack, rr = sub[1].rack;
    B.ibeam(pl1, pr1, mSubframe);
    B.ibeam(pl2, pr2, mSubframe);
    B.ibeam(pl1, pr2, mSubframe);
    B.ibeam(pl2, pr1, mSubframe);
    B.ibeam(pl1, pl2, mSubframe);
    B.ibeam(pr1, pr2, mSubframe);
    B.beam(rl, rr, mSubframe);
    addLink(rl, rr, 0.02f, MM_DARK);
    addLink(u32(pl1), u32(pr1), 0.028f, MM_DARK);
    addLink(u32(pl2), u32(pr2), 0.028f, MM_DARK);
    for (int a : {pl1, pl2, pr1, pr2}) {
      B.ibeam(int(rl), a, mSubframe);
      B.ibeam(int(rr), a, mSubframe);
    }
    // Upper link pick-ups: braced across the car, and tied down to the lower subframe.
    B.beam(sub[0].uf, sub[1].uf, mSubframe);
    B.beam(sub[0].ur, sub[1].ur, mSubframe);
    B.beam(sub[0].uf, sub[1].ur, mSubframe);
    B.beam(sub[0].ur, sub[1].uf, mSubframe);
    for (int sd = 0; sd < 2; sd++) {
      B.ibeam(int(sub[sd].uf), pl1, mSubframe);
      B.ibeam(int(sub[sd].uf), pr1, mSubframe);
      B.ibeam(int(sub[sd].ur), pl2, mSubframe);
      B.ibeam(int(sub[sd].ur), pr2, mSubframe);
    }
    // Rear: both trailing arm bearings on one crossbeam, tied to the rear floor.
    B.beam(sub[0].pi, sub[1].pi, mSubframe);
    addLink(sub[0].po, sub[1].po, 0.045f, MM_DARK);
    B.beam(sub[0].po, sub[1].po, mSubframe);
    B.beam(sub[0].po, sub[1].pi, mSubframe);
    B.beam(sub[0].pi, sub[1].po, mSubframe);
    for (int sd = 0; sd < 2; sd++)
      for (u32 n : {sub[sd].po, sub[sd].pi})
        for (auto [ss, rr2] : {std::pair{12, 1}, {13, 1}, {14, 1}, {12, 2}, {13, 2}})
          B.ibeam(int(n), B.body[ss][rr2][sd], mSubframe);
  }
  veh.axle[1].regP0 = 40e5;
  for (int i = 0; i < 2; i++)
    veh.shaftLen[i] = len(sim.pos[veh.wheels[size_t(i)].axIn] - sim.pos[veh.shaftNode[i]]);
  veh.axle[0].kBar = 30e3;
  veh.axle[1].kBar = 18e3;

  veh.refFrontL = B.body[8][2][0];
  veh.refFrontR = B.body[8][2][1];
  veh.refRearL = B.body[13][2][0];
  veh.refRearR = B.body[13][2][1];

  // ---- Mass tuning: solve a linear mass ramp over the shell for total mass and axle split.
  sim.endObj();
  {
    constexpr double kTarget = 1530.0;  // listed kerb weights are 1526 to 1543 kg
    const double frontLoad = 0.58;
    const V3 axisX = B.worldDir(-1, 0);  // towards increasing design x
    const V3 ox = B.world({0, 0, 0}, 0);
    const double cogX = 0.90 + (1.0 - frontLoad) * 2.74;
    const Obj& o = sim.objs[veh.obj];
    double mOther = 0, sOther = 0, m0 = 0, m1 = 0, m2 = 0;
    std::vector<char> isShell(o.nodeStart + o.nodeCount, 0);
    for (u32 id : shell) isShell[id] = 1;
    for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) {
      const double x = dot(sim.pos[i] - ox, axisX);
      if (isShell[i]) {
        m0 += sim.mass[i];
        m1 += sim.mass[i] * x;
        m2 += sim.mass[i] * x * x;
      } else {
        mOther += sim.mass[i];
        sOther += sim.mass[i] * x;
      }
    }
    const double Mb = kTarget - mOther, Sb = kTarget * cogX - sOther;
    const double det = m0 * m2 - m1 * m1;
    const double a = (Mb * m2 - Sb * m1) / det;
    const double b = (m0 * Sb - m1 * Mb) / det;
    for (u32 id : shell) {
      const double x = dot(sim.pos[id] - ox, axisX);
      const double f = std::max(0.5, a + b * x);
      sim.mass[id] *= f;
      sim.invMass[id] = 1.0 / sim.mass[id];
    }
    double total = 0;
    for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) total += sim.mass[i];
    veh.totalMass = total;
  }

  // ---- Bulkheads as self-collision surfaces. Their normals face the side things come from.
  // (Triangles belong to the last object begun, so adding them after endObj() is fine.)
  const V3 toNose = B.worldDir(1, 0);
  const V3 upDir{0, 0, 1};
  auto bulkhead = [&](int s, V3 facing) {
    for (int r = 0; r < NR - 1; r++) {
      const int aL = B.body[s][r][0], bL = B.body[s][r + 1][0];
      const int aR = B.body[s][r][1], bR = B.body[s][r + 1][1];
      if (aL < 0 || bL < 0 || aR < 0 || bR < 0) continue;
      if (aL != aR) B.orientedTri(u32(aL), u32(bL), u32(aR), facing, TF_HARD | TF_NOVIS);
      if (bL != bR) B.orientedTri(u32(bL), u32(bR), u32(aR), facing, TF_HARD | TF_NOVIS);
    }
  };
  bulkhead(8, toNose);
  bulkhead(14, -toNose);
  for (int s = 8; s < 14; s++)
    for (int side = 0; side < 2; side++)
      for (int r = 0; r < 2; r++) {
        const int a = B.body[s][r][side], b = B.body[s + 1][r][side];
        const int c = B.body[s + 1][r + 1][side], d = B.body[s][r + 1][side];
        B.orientedTri(u32(a), u32(b), u32(c), upDir, TF_HARD | TF_NOVIS);
        B.orientedTri(u32(a), u32(c), u32(d), upDir, TF_HARD | TF_NOVIS);
      }

  // ---- Outer collision surface. The visible mesh is far finer; collisions use the structure.
  auto nodeFor = [&](int s, int r, int side) -> int {
    if (B.hood[s][r][side] >= 0) return B.hood[s][r][side];
    if (B.lid[s][r][side] >= 0) return B.lid[s][r][side];
    return B.body[s][r][side];
  };
  auto addQuad = [&](const int n[4], int s, bool cap) {
    for (int i = 0; i < 4; i++)
      if (n[i] < 0) return;
    const V3 c = (sim.pos[n[0]] + sim.pos[n[1]] + sim.pos[n[2]] + sim.pos[n[3]]) * 0.25;
    const V3 axisPt = B.world({lerpd(X[s], X[std::min(s + 1, NS - 1)], 0.5), 0.0, 0.70}, 0);
    V3 out = c - axisPt;
    if (cap) out = s == 0 ? toNose : -toNose;
    B.orientedTri(u32(n[0]), u32(n[1]), u32(n[2]), out, TF_NOVIS, 0.5f);
    B.orientedTri(u32(n[0]), u32(n[2]), u32(n[3]), out, TF_NOVIS, 0.5f);
  };

  auto doorOf = [&](int s) { return s >= 11 ? 1 : 0; };
  for (int side = 0; side < 2; side++) {
    for (int s = 0; s < NS - 1; s++) {
      const int d = doorOf(s);
      const bool doorSpan = s >= 8 && s < 13;
      for (int r = 0; r < NR - 1; r++) {
        int n[4];
        if (doorSpan && r >= 3 && r <= 5) {
          const auto& D = B.doors[d][side];
          const int i = s - D.s0, k = r - 3;
          n[0] = D.n[i][k];
          n[1] = D.n[i + 1][k];
          n[2] = D.n[i + 1][k + 1];
          n[3] = D.n[i][k + 1];
        } else {
          n[0] = nodeFor(s, r, side);
          n[1] = nodeFor(s + 1, r, side);
          n[2] = nodeFor(s + 1, r + 1, side);
          n[3] = nodeFor(s, r + 1, side);
        }

        addQuad(n, s, false);
      }
    }
  }

  // End caps closing the nose and the tail.
  for (int s : {0, NS - 1})
    for (int r = 2; r <= 6; r++) {
      const int n[4] = {nodeFor(s, r, 0), nodeFor(s, r + 1, 0), nodeFor(s, r + 1, 1), nodeFor(s, r, 1)};
      addQuad(n, s, true);
    }

  // Everything sprung takes part in shape damping: all but wheels, arms, engine and gearbox.
  {
    std::vector<char> skip(sim.pos.size(), 0);
    for (u32 n : veh.engineNodes) skip[n] = 1;
    for (u32 n : veh.gearboxNodes) skip[n] = 1;
    for (const Wheel& w : veh.wheels) {
      skip[w.axIn] = skip[w.axOut] = 1;
      for (u32 r : w.ring) skip[r] = 1;
    }
    for (int ax = 0; ax < 2; ax++)
      for (int sd = 0; sd < 2; sd++) skip[veh.axle[ax].s[sd].low] = 1;
    for (u32 n : shell) veh.structureNodes.push_back(n);
    for (u32 i = shell.back() + 1; i < sim.pos.size(); i++)
      if (!skip[i] && sim.mass[i] < 15.0 && !(sim.flags[i] & NF_WHEEL) && (sim.flags[i] & NF_COLLIDE)) veh.structureNodes.push_back(i);
  }

  // ---- Node groups the visible mesh binds to.
  if (rig) {
    auto group = [&](const char* name, std::vector<u32> nodes) { rig->groups.push_back({name, std::move(nodes)}); };
    group("body", shell);
    auto collect = [&](int (&P)[NS][NR][2], std::vector<u32> extra) {
      std::set<u32> u(extra.begin(), extra.end());
      for (auto& a : P) for (auto& b : a) for (int v : b) if (v >= 0) u.insert(u32(v));
      return std::vector<u32>(u.begin(), u.end());
    };
    group("hood", collect(B.hood, hoodInner));
    group("tailgate", collect(B.lid, lidInner));
    auto doorNodes = [&](int d, int side) {
      std::vector<u32> v;
      const auto& D = B.doors[d][side];
      for (int i = 0; i <= D.s1 - D.s0; i++)
        for (int k = 0; k < 4; k++) v.push_back(u32(D.n[i][k]));
      v.insert(v.end(), D.inner.begin(), D.inner.end());
      return v;
    };
    group("door_fl", doorNodes(0, 0));
    group("door_fr", doorNodes(0, 1));
    group("door_rl", doorNodes(1, 0));
    group("door_rr", doorNodes(1, 1));
    std::vector<u32> bf, br;
    for (int s = 0; s <= 3; s++)
      for (int r = 2; r <= 5; r++)
        for (int side = 0; side < 2; side++)
          if (B.body[s][r][side] >= 0) bf.push_back(u32(B.body[s][r][side]));
    for (int s = 18; s < NS; s++)
      for (int r = 2; r <= 5; r++)
        for (int side = 0; side < 2; side++)
          if (B.body[s][r][side] >= 0) br.push_back(u32(B.body[s][r][side]));
    std::vector<u32> ws;  // the aperture around the windscreen
    for (int s = 8; s <= 11; s++)
      for (int r = 5; r <= 8; r++)
        for (int side = 0; side < (r == 8 ? 1 : 2); side++)
          if (B.body[s][r][side] >= 0) ws.push_back(u32(B.body[s][r][side]));
    group("windscreen", ws);
    // A stiff, three-dimensional frame for everything mounted inside the car. Sills, pillars and
    // roof rails span the whole volume, so nothing has to be extrapolated from a flat sheet.
    std::vector<u32> frame;
    for (int s = 0; s < NS; s++)
      for (int r : {2, 3, 6})
        for (int side = 0; side < 2; side++)
          if (B.body[s][r][side] >= 0) frame.push_back(u32(B.body[s][r][side]));
    group("frame", frame);
    group("bumper_f", bf);
    group("bumper_r", br);
    group("engine", veh.engineNodes);
    group("gearbox", veh.gearboxNodes);
    group("radiator", radNodes);
    group("battery", batNodes);
    group("tank", tankNodes);
  }
  return veh;
}
