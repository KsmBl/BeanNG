// Hummer H1 Alpha as a node/beam structure.
//
// Main dimensions follow the real truck: length 4.69 m, width 2.20 m, height 1.96 m, wheelbase 3.30 m, about
// 3400 kg, 41 cm of clearance under portal axles, 37 inch tyres, and a 6.6 litre turbo diesel V8. A steel
// ladder frame that shrugs off nearly anything, under a body that dents like the sheet metal it is.
// Station tables are shared with tools/gen_hummer.py: x from the nose rearwards, y left, z up.
#include <algorithm>
#include <set>
#include "vehicle.h"

namespace {
constexpr int NS = 20, NR = 7;
constexpr double kNoseX = 2.35;
constexpr double kFrontAxle = 0.74, kRearAxle = 4.04;

const double X[NS] = {0.00, 0.10, 0.28, 0.50, 0.74, 1.00, 1.25, 1.50, 1.85, 2.20, 2.55, 2.90, 3.25, 3.55, 3.80, 4.04, 4.25, 4.45, 4.60, 4.69};
const double ZTOP[NS] = {1.00, 1.10, 1.15, 1.17, 1.19, 1.21, 1.23, 1.28, 1.86, 1.93, 1.94, 1.94, 1.93, 1.90, 1.82, 1.60, 1.36, 1.26, 1.20, 1.05};
const double W[NS] = {0.96, 1.04, 1.07, 1.09, 1.10, 1.10, 1.09, 1.08, 1.08, 1.08, 1.08, 1.08, 1.08, 1.08, 1.09, 1.10, 1.10, 1.09, 1.06, 1.00};
const double ZBELT[NS] = {0.86, 0.98, 1.08, 1.12, 1.14, 1.14, 1.14, 1.14, 1.14, 1.14, 1.14, 1.14, 1.14, 1.14, 1.14, 1.14, 1.14, 1.12, 1.08, 0.98};
const double ZRAIL[NS] = {0.98, 1.06, 1.12, 1.15, 1.17, 1.19, 1.21, 1.26, 1.80, 1.87, 1.88, 1.88, 1.87, 1.84, 1.76, 1.54, 1.32, 1.22, 1.16, 1.02};
const double WRAIL[NS] = {0.70, 0.78, 0.82, 0.84, 0.85, 0.86, 0.86, 0.86, 0.80, 0.78, 0.78, 0.78, 0.78, 0.80, 0.82, 0.84, 0.84, 0.82, 0.78, 0.70};
const double ZLOW[NS] = {0.66, 0.66, 0.74, 0.92, 1.02, 0.96, 0.78, 0.66, 0.60, 0.60, 0.60, 0.60, 0.60, 0.66, 0.92, 1.02, 0.96, 0.78, 0.70, 0.68};
const double ZFLOOR[NS] = {0.68, 0.64, 0.62, 0.62, 0.62, 0.62, 0.62, 0.60, 0.58, 0.58, 0.58, 0.58, 0.58, 0.60, 0.62, 0.62, 0.62, 0.62, 0.64, 0.68};
const double WFLOOR[NS] = {0.60, 0.60, 0.50, 0.42, 0.42, 0.42, 0.42, 0.60, 0.88, 0.88, 0.88, 0.88, 0.70, 0.42, 0.42, 0.42, 0.42, 0.50, 0.60, 0.60};

V3 design(int s, int r) {
  const double x = X[s];
  switch (r) {
    case 0: return {x, 0.0, ZFLOOR[s]};
    case 1: return {x, WFLOOR[s], ZFLOOR[s]};
    case 2: return {x, W[s] * 0.97, ZLOW[s]};
    case 3: return {x, W[s], 0.5 * (ZLOW[s] + ZBELT[s])};
    case 4: return {x, W[s] * 0.985, ZBELT[s]};
    case 5: return {x, WRAIL[s], ZRAIL[s]};
    default: return {x, 0.0, ZTOP[s]};
  }
}

// The ladder frame: heavy steel, made to be driven into things.
const Mat mTub{3.2e6, 480, 170e3, 200e3, 6e5, 0.25, 0};
const Mat mTubCross{2.8e6, 420, 120e3, 150e3, 4e5, 0.25, 0};
// Ends of the frame and the bull bar, then ordinary sheet steel for the body.
const Mat mCrash{2.0e6, 300, 42e3, 60e3, 1.2e5, 0.45, 0};
const Mat mCrashCross{1.5e6, 240, 22e3, 36e3, 7e4, 0.40, 0};
const Mat mSkin{0.9e6, 150, 6e3, 11e3, 2.5e4, 0.45, 0};
const Mat mDoor{0.9e6, 150, 9e3, 14e3, 3e4, 0.25, 26e3};
const Mat mHinge{1.2e6, 220, 0, 0, 0, 0, 17e3};
const Mat mLatch{0.8e6, 160, 0, 0, 0, 0, 9e3};
// const Mat mPylon{0.5e6, 80, 0, 0, 0, 0, 12e3};


const Mat mAnchor{3.0e6, 440, 170e3, 220e3, 5e5, 0.20, 0};
const Mat mSubframe{3.0e6, 440, 190e3, 240e3, 5e5, 0.22, 0};
const Mat mArm{4.0e6, 500, 210e3, 260e3, 6e5, 0.22, 0};
const Mat mHub{2.2e6, 280, 62e3, 90e3, 2.2e5, 0.25, 0};
const Mat mKnuckle{2.2e6, 280, 230e3, 290e3, 8e5, 0.25, 0};
const Mat mCarrier{3.2e6, 420, 240e3, 290e3, 8e5, 0.20, 0};
const Mat mTie{3.0e6, 360, 110e3, 140e3, 3e5, 0.12, 0};
const Mat mBlock{6e6, 900, 240e3, 240e3, 6e5, 0.0, 0};
const Mat mMount{4.5e5, 500, 16e3, 18e3, 3e4, 0.45, 0};
const Mat mTank{1.2e6, 240, 18e3, 22e3, 3e4, 0.30, 0};
const Mat mRadCore{1.0e6, 170, 6e3, 8e3, 1.5e4, 0.30, 0};
const Mat mRadMount{1.0e6, 160, 0, 0, 0, 0, 14e3};

// Duramax 6.6 turbo diesel of the H1 Alpha, 300 PS and 705 N m: massive pull from just above idle.
const double kTorque[][2] = {{0, 300}, {700, 430}, {1200, 640}, {1600, 705}, {2400, 690}, {3000, 640}, {3300, 560}, {3500, 380}, {3700, 0}};
}  // namespace

Vehicle buildHummer(Sim& sim, V3 spawn, CarRig* rig) {
  Vehicle veh;
  veh.obj = sim.beginObj(OBJ_CAR, true);

  PowerSpec& P = veh.spec;
  P.torque = kTorque;
  P.torquePoints = int(sizeof(kTorque) / sizeof(kTorque[0]));
  P.frictionBase = 42.0;
  P.frictionPerRpm = 0.03;
  P.idleRpm = 680.0;
  P.revLimit = 3400.0;
  P.nGears = 5;
  const double gears[5] = {3.10, 1.81, 1.41, 1.00, 0.71};
  std::copy(gears, gears + 5, P.gears);
  P.reverse = 4.49;
  P.finalDrive = 5.24;      // differential and the portal gears in the hubs
  P.lowRange = 2.61;
  P.engineInertia = 0.75;
  P.carrierInertia = 1.2;   // two propshafts and the transfer case
  P.clutchCapacity = 1600.0;
  P.synchroTorque = 220.0;
  P.shaftRate = 42000.0;
  P.shaftDamp = 34.0;
  P.shaftShear = 34000.0;
  P.upshiftLow = 1300.0;
  P.upshiftHigh = 3200.0;
  P.downshiftLow = 800.0;
  P.downshiftHigh = 1650.0;
  P.tyreMu = 0.98;
  P.rimRadius = 0.325;      // run-flat inserts: it rolls on with a flat
  P.dentFz = 52000.0;
  P.burstFz = 95000.0;
  P.conventional = true;
  P.turbo = true;
  P.launchRpm = 1700.0;
  P.boostFrom = 900.0;
  P.boostSpan = 700.0;
  P.offBoost = 400.0;
  P.awd = true;
  P.looseGrip = 1.55;       // deep lugs dig in where a road tyre only skates
  P.frontShare = 0.54;
  P.lockAngle = 0.58;
  P.wheelbase = kRearAxle - kFrontAxle;
  P.understeer = 0.0042;
  P.diffPreload = 700.0;
  P.diffRamp = 0.5;
  P.rpmMax = 4000;
  P.rpmRed = 3300;
  veh.dragArea = 2.15;      // a barn door: 1.1 of Cd on nearly 2 m2 of front
  veh.sportMode = true;
  veh.hydractiveSoft = false;
  veh.activaOn = false;
  veh.engineOmega = 72;
  veh.discOmega = 72;
  veh.fuelLitres = 140;
  veh.steerTravel = 0.064;

  std::set<std::pair<u32, u32>> seen;
  const auto world = [&](V3 d, int side) { return spawn + V3{kNoseX - d.x, side == 0 ? d.y : -d.y, d.z}; };
  const auto beam = [&](u32 a, u32 b, const Mat& m, std::vector<u32>* list = nullptr) {
    if (a == b) return;
    auto key = std::minmax(a, b);
    if (!seen.insert({key.first, key.second}).second) return;
    const u32 id = sim.addBeam(a, b, m);
    if (list) list->push_back(id);
  };
  const auto ibeam = [&](int a, int b, const Mat& m) {
    if (a >= 0 && b >= 0) beam(u32(a), u32(b), m);
  };
  const auto oriented = [&](u32 a, u32 b, u32 c, V3 want, u8 fl) {
    const V3 n = cross(sim.pos[b] - sim.pos[a], sim.pos[c] - sim.pos[a]);
    if (dot(n, want) >= 0) sim.addTri(a, b, c, fl, 0, 0.5f);
    else sim.addTri(a, c, b, fl, 0, 0.5f);
  };

  // ---- Shell. The doors take the place of the side rows between the pillars.
  constexpr int kDoor0 = 9, kDoor1 = 10;  // door-only stations of the front door
  const auto doorOnly = [](int s, int r) { return s >= kDoor0 && s <= kDoor1 && (r == 3 || r == 4); };
  int body[NS][NR][2];
  for (auto& a : body)
    for (auto& b : a) b[0] = b[1] = -1;
  const double massBase[NR] = {5.0, 7.0, 8.0, 4.0, 4.0, 3.0, 2.6};
  std::vector<u32> shell;
  for (int s = 0; s < NS; s++)
    for (int r = 0; r < NR; r++) {
      if (doorOnly(s, r)) continue;
      const bool center = r == 0 || r == 6;
      for (int side = 0; side < (center ? 1 : 2); side++) {
        const u32 id = sim.addNode(world(design(s, r), side), massBase[r], NF_COLLIDE, 0.45f);
        body[s][r][side] = int(id);
        if (center) body[s][r][1] = int(id);
        shell.push_back(id);
      }
    }

  const auto inTub = [](int s) { return s >= 3 && s <= 17; };
  const auto longMat = [&](int s, int r) -> const Mat& {
    if (inTub(s)) return r <= 1 ? mTub : (r == 2 || r == 5 ? mCrash : mSkin);
    if (r <= 2) return mCrash;
    return mSkin;
  };
  const auto aroundMat = [&](int s, int r) -> const Mat& {
    if (inTub(s)) return r <= 1 ? mTubCross : (s == 8 || s == 13 ? mCrash : mSkin);
    if (r <= 1) return mCrashCross;
    return mSkin;
  };
  for (int s = 0; s < NS; s++)
    for (int r = 0; r < NR; r++)
      for (int side = 0; side < 2; side++) {
        const int a = body[s][r][side];
        if (a < 0) continue;
        if (s + 1 < NS) {
          ibeam(a, body[s + 1][r][side], longMat(s, r));
          if (r + 1 < NR) {
            ibeam(a, body[s + 1][r + 1][side], aroundMat(s, r));
            ibeam(body[s + 1][r][side], body[s][r + 1][side], aroundMat(s, r));
          }
        }
        if (r + 1 < NR) ibeam(a, body[s][r + 1][side], aroundMat(s, r));
        if (s + 2 < NS) ibeam(a, body[s + 2][r][side], longMat(s, r));
        if (r + 2 < NR) ibeam(a, body[s][r + 2][side], aroundMat(s, r));
      }
  // Across the car: this is what makes a box out of the two side walls.
  for (int s = 0; s < NS; s++) {
    const Mat& m = inTub(s) ? mTubCross : mCrashCross;
    for (int r : {1, 2, 5}) ibeam(body[s][r][0], body[s][r][1], m);
    ibeam(body[s][1][0], body[s][5][1], m);
    ibeam(body[s][1][1], body[s][5][0], m);
    ibeam(body[s][2][0], body[s][5][1], m);
    ibeam(body[s][2][1], body[s][5][0], m);
    ibeam(body[s][0][0], body[s][6][0], m);
    if (s + 1 < NS) {
      ibeam(body[s][1][0], body[s + 1][1][1], m);
      ibeam(body[s][1][1], body[s + 1][1][0], m);
      ibeam(body[s][5][0], body[s + 1][5][1], m);
      ibeam(body[s][5][1], body[s + 1][5][0], m);
    }
  }

  // ---- Anything mounted to the structure ties in at its nearest nodes.
  const auto anchor = [&](u32 node, int n, const Mat& m, std::vector<u32>* list = nullptr) {
    std::vector<std::pair<double, u32>> near;
    for (u32 b : shell) near.push_back({len2(sim.pos[b] - sim.pos[node]), b});
    std::sort(near.begin(), near.end());
    int done = 0;
    for (const auto& c : near) {
      if (c.first < 0.0025) continue;
      beam(node, c.second, m, list);
      if (++done >= n) break;
    }
  };

  // ---- Doors: dihedral synchro-helix doors in reality. Here they hang on hinges and a latch.
  std::vector<u32> doorNodes[2];
  for (int side = 0; side < 2; side++) {
    std::vector<u32> door;
    int dn[kDoor1 - kDoor0 + 1][2];
    for (int s = kDoor0; s <= kDoor1; s++)
      for (int r = 3; r <= 4; r++) {
        const u32 id = sim.addNode(world(design(s, r), side), 2.4, NF_COLLIDE, 0.45f);
        dn[s - kDoor0][r - 3] = int(id);
        door.push_back(id);
      }
    // Inner frame, which gives the door depth.
    u32 inner[kDoor1 - kDoor0 + 1];
    for (int i = 0; i <= kDoor1 - kDoor0; i++) {
      V3 d = design(kDoor0 + i, 3);
      d.y -= 0.14;
      d.z += 0.05;
      inner[i] = sim.addNode(world(d, side), 2.0, 0, 0.45f);
      door.push_back(inner[i]);
    }
    for (size_t a = 0; a < door.size(); a++)
      for (size_t b = a + 1; b < door.size(); b++) beam(door[a], door[b], mDoor);
    for (int r = 0; r < 2; r++) {
      // Hinges at the front, latch at the rear.
      for (int rr : {2, 5}) beam(u32(dn[0][r]), u32(body[kDoor0 - 1][rr][side]), mHinge);
      beam(u32(dn[0][r]), u32(body[kDoor0 - 1][3][side]), mHinge);
      beam(u32(dn[0][r]), u32(body[kDoor0 - 1][4][side]), mHinge);
      beam(u32(dn[kDoor1 - kDoor0][r]), u32(body[kDoor1 + 1][3][side]), mLatch);
      beam(u32(dn[kDoor1 - kDoor0][r]), u32(body[kDoor1 + 1][4][side]), mLatch);
    }
    // Sill and roof rail hold it in place sideways through its own frame.
    for (int i = 0; i <= kDoor1 - kDoor0; i++) {
      beam(u32(dn[i][0]), u32(body[kDoor0 + i][2][side]), mLatch);
      beam(u32(dn[i][1]), u32(body[kDoor0 + i][5][side]), mLatch);
    }
    doorNodes[side] = door;
  }

  // ---- Engine and gearbox behind the cabin, on the tub.
  const auto block = [&](double x0, double x1, double y0, double y1, double z0, double z1, double mass, std::vector<u32>& out, const Mat& m, std::vector<u32>* list) {
    u32 e[8];
    for (int i = 0; i < 8; i++) {
      const V3 d{(i & 1) ? x1 : x0, (i & 2) ? y1 : y0, (i & 4) ? z1 : z0};
      e[i] = sim.addNode(world(d, 0), mass / 8.0, NF_COLLIDE, 0.4f);
      out.push_back(e[i]);
    }
    for (int i = 0; i < 8; i++)
      for (int j = i + 1; j < 8; j++) beam(e[i], e[j], m, list);
    V3 c{};
    for (u32 n : e) c += sim.pos[n];
    c = c * 0.125;
    static const int faces[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4}, {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
    for (const auto& f : faces) {
      const V3 fc = (sim.pos[e[f[0]]] + sim.pos[e[f[2]]]) * 0.5;
      oriented(e[f[0]], e[f[1]], e[f[2]], fc - c, TF_HARD | TF_NOVIS);
      oriented(e[f[0]], e[f[2]], e[f[3]], fc - c, TF_HARD | TF_NOVIS);
    }
  };
  block(0.42, 1.30, -0.32, 0.32, 0.72, 1.24, 340.0, veh.engineNodes, mBlock, &veh.engineBlockBeams);
  block(1.32, 2.05, -0.22, 0.22, 0.62, 1.00, 170.0, veh.gearboxNodes, mBlock, &veh.gearboxBeams);
  for (int i = 1; i < 8; i += 2)
    for (int j = 0; j < 4; j++) beam(veh.engineNodes[size_t(i)], veh.gearboxNodes[size_t(j) * 2], mAnchor);
  for (u32 n : veh.engineNodes) anchor(n, 3, mMount, &veh.engineMountBeams);
  for (u32 n : veh.gearboxNodes) anchor(n, 2, mMount, &veh.engineMountBeams);
  // The two differentials, high up inside the frame, with the flanges for their halfshafts.
  for (int ax = 0; ax < 2; ax++) {
    std::vector<u32> diff;
    const double xd = ax == 0 ? kFrontAxle : kRearAxle;
    block(xd - 0.16, xd + 0.16, -0.18, 0.18, 0.60, 0.86, 70.0, diff, mBlock, nullptr);
    for (u32 n : diff) anchor(n, 4, mAnchor);
    for (int side = 0; side < 2; side++) {
      const u32 f = sim.addNode(world({xd, 0.24, 0.72}, side), 6.0, 0, 0.4f);
      for (u32 n : diff) beam(f, n, mAnchor);
      veh.shaftNode[size_t(ax) * 2 + size_t(side)] = f;
    }
  }

  // Radiators in the nose, the tank between cabin and engine, as on the real car.
  std::vector<u32> radNodes, tankNodes;
  block(0.16, 0.30, -0.56, 0.56, 0.78, 1.10, 30.0, radNodes, mRadCore, &veh.radiatorBeams);
  for (u32 n : radNodes) anchor(n, 3, mRadMount);
  block(3.30, 3.70, -0.62, 0.62, 0.68, 0.98, 130.0, tankNodes, mTank, &veh.tankBeams);
  for (u32 n : tankNodes) anchor(n, 3, mAnchor);

  // ---- Running gear: double wishbones at all four corners.
  const auto addLink = [&](u32 a, u32 b, float r, u8 mat, u8 kind = 0) {
    if (rig) rig->links.push_back({a, b, r, mat, kind});
  };
  struct Pick { u32 lf, lr, uf, ur, rack; } pick[2][2];
  const auto makeWheel = [&](bool front, int side) {
    Wheel w;
    w.front = front;
    w.left = side == 0;
    w.driven = true;
    w.brakeMax = front ? 9500.0 : 7800.0;
    w.R = 0.470;
    w.width = 0.33;
    w.I = 9.0;
    w.kRadial = 300e3;
    w.cRadial = 900;
    const double hx = front ? kFrontAxle : kRearAxle;
    const double hy = 0.91;
    const double hz = w.R;
    const auto node = [&](double x, double y, double z, double m, u8 fl = NF_COLLIDE) { return sim.addNode(world({x, y, z}, side), m, fl, 0.5f); };
    w.axIn = node(hx, hy - 0.09, hz, 12.0, 0);
    w.axOut = node(hx, hy + 0.09, hz, 12.0, 0);
    for (int i = 0; i < 6; i++) {
      const double ang = PI / 3.0 * i;
      w.ring[i] = node(hx - std::sin(ang) * 0.32, hy, hz + std::cos(ang) * 0.32, 8.0, NF_COLLIDE | NF_WHEEL);
    }
    beam(w.axIn, w.axOut, mHub);
    for (int i = 0; i < 6; i++) {
      w.spoke[i][0] = u32(sim.beams.size());
      sim.addBeam(w.ring[i], w.axIn, mHub);
      w.spoke[i][1] = u32(sim.beams.size());
      sim.addBeam(w.ring[i], w.axOut, mHub);
      beam(w.ring[i], w.ring[(i + 1) % 6], mHub);
      beam(w.ring[i], w.ring[(i + 2) % 6], mHub);
    }
    // Portal hub: the arms sit well above the middle of the wheel, which is where the clearance comes from.
    const u32 kl = node(hx, hy - 0.08, 0.60, 9.0, 0);
    const u32 ku = node(hx + 0.015, hy - 0.13, 0.90, 9.0, 0);
    const u32 ks = node(hx + 0.15, hy - 0.14, 0.62, 6.0, 0);
    const u32 kf = node(hx - 0.15, hy - 0.12, 0.66, 6.0, 0);
    for (u32 a : {w.axIn, w.axOut})
      for (u32 c : {kl, ku, ks, kf}) beam(c, a, mCarrier);
    for (u32 a : {kl, ku, ks, kf})
      for (u32 c : {kl, ku, ks, kf}) beam(a, c, mCarrier);
    for (int i = 0; i < 6; i++)
      for (u32 c : {kl, ku, ks, kf}) beam(w.ring[i], c, mKnuckle);

    // Pick-ups on a subframe node set, tied firmly into the structure.
    Pick& p = pick[front ? 0 : 1][side];
    // Level arms: leaning ones would turn every side force into a lift of the whole truck.
    p.lf = node(hx - 0.28, 0.34, 0.60, 10.0, 0);
    p.lr = node(hx + 0.28, 0.34, 0.60, 10.0, 0);
    p.uf = node(hx - 0.24, 0.40, 0.90, 9.0, 0);
    p.ur = node(hx + 0.24, 0.40, 0.90, 9.0, 0);
    p.rack = node(hx + 0.15, 0.32, 0.62, 7.0, 0);
    for (u32 n : {p.lf, p.lr, p.uf, p.ur, p.rack}) anchor(n, 5, mAnchor);
    for (u32 a : {p.lf, p.lr, p.uf, p.ur, p.rack})
      for (u32 c : {p.lf, p.lr, p.uf, p.ur, p.rack}) beam(a, c, mSubframe);
    beam(kl, p.lf, mArm);
    beam(kl, p.lr, mArm);
    beam(ku, p.uf, mArm);
    beam(ku, p.ur, mArm);
    const u32 tie = sim.addBeam(ks, p.rack, mTie);
    if (front) veh.tieBeam[side] = tie;
    addLink(ks, p.rack, 0.011f, MM_METAL);
    addLink(kl, p.lf, 0.014f, MM_DARK);
    addLink(kl, p.lr, 0.014f, MM_DARK);
    addLink(ku, p.uf, 0.011f, MM_DARK);
    addLink(ku, p.ur, 0.011f, MM_DARK);
    addLink(kl, w.axIn, 0.028f, MM_ALU);
    addLink(ku, w.axIn, 0.028f, MM_ALU);
    addLink(w.axIn, veh.shaftNode[size_t(front ? 0 : 2) + size_t(side)], 0.03f, MM_METAL);

    // A gas damper unit, with the firm rate of a track car.
    const u32 tower = node(hx, 0.62, 1.16, 8.0, 0);
    anchor(tower, 5, mAnchor);
    beam(tower, p.uf, mAnchor);
    beam(tower, p.ur, mAnchor);
    addLink(tower, kl, 0.036f, MM_METAL, 1);
    HydroStrut st;
    st.top = tower;
    st.low = kl;
    st.A = 6e-4;
    st.V0 = front ? 274e-6 : 210e-6;
    st.p0 = 60e5;
    st.bump[0] = st.bump[1] = front ? 4200 : 3800;
    st.reb[0] = st.reb[1] = front ? 7000 : 6200;
    const double L = len(sim.pos[st.low] - sim.pos[st.top]);
    st.L = st.Lnom = L;
    st.Lmin = L - 0.17;   // a lot of travel: 16 inches at the wheel on the real one
    st.Lmax = L + 0.19;
    veh.axle[front ? 0 : 1].s[side] = st;

    for (int i = 0; i < 6; i++) {
      const V3 outDir = V3{0, side == 0 ? 1.0 : -1.0, 0};
      oriented(w.axOut, w.ring[i], w.ring[(i + 1) % 6], outDir, TF_NOVIS);
    }
    return w;
  };
  veh.wheels.push_back(makeWheel(true, 0));
  veh.wheels.push_back(makeWheel(true, 1));
  veh.wheels.push_back(makeWheel(false, 0));
  veh.wheels.push_back(makeWheel(false, 1));
  // Both sides of each axle's subframe tied across the car.
  for (int ax = 0; ax < 2; ax++) {
    const Pick& a = pick[ax][0];
    const Pick& b = pick[ax][1];
    for (u32 n : {a.lf, a.lr, a.uf, a.ur, a.rack})
      for (u32 m : {b.lf, b.lr, b.uf, b.ur, b.rack}) beam(n, m, mSubframe);
    addLink(a.lf, b.lf, 0.02f, MM_DARK);
    addLink(a.lr, b.lr, 0.02f, MM_DARK);
  }
  for (size_t i = 0; i < 4; i++) veh.shaftLen[i] = len(sim.pos[veh.wheels[i].axIn] - sim.pos[veh.shaftNode[i]]);
    veh.axle[0].kBar = 15e3;
  veh.axle[1].kBar = 11e3;

  veh.refFrontL = u32(body[8][2][0]);
  veh.refFrontR = u32(body[8][2][1]);
  veh.refRearL = u32(body[13][2][0]);
  veh.refRearR = u32(body[13][2][1]);

  // ---- Outer collision surface.
  const auto faceOut = [&](const int n[4], V3 axisPt) {
    for (int i = 0; i < 4; i++)
      if (n[i] < 0) return;
    const V3 c = (sim.pos[n[0]] + sim.pos[n[1]] + sim.pos[n[2]] + sim.pos[n[3]]) * 0.25;
    const V3 out = c - axisPt;
    oriented(u32(n[0]), u32(n[1]), u32(n[2]), out, TF_NOVIS);
    oriented(u32(n[0]), u32(n[2]), u32(n[3]), out, TF_NOVIS);
  };
  for (int side = 0; side < 2; side++)
    for (int s = 0; s + 1 < NS; s++)
      for (int r = 0; r + 1 < NR; r++) {
        const int n[4] = {body[s][r][side], body[s + 1][r][side], body[s + 1][r + 1][side], body[s][r + 1][side]};
        faceOut(n, world({lerpd(X[s], X[s + 1], 0.5), 0.0, 1.10}, 0));
      }
  for (int s : {0, NS - 1})
    for (int r = 1; r + 1 < NR; r++) {
      const int n[4] = {body[s][r][0], body[s][r + 1][0], body[s][r + 1][1], body[s][r][1]};
      for (int i = 0; i < 4; i++)
        if (n[i] < 0) continue;
      const V3 out{s == 0 ? 1.0 : -1.0, 0, 0};
      if (n[0] != n[3]) {
        oriented(u32(n[0]), u32(n[1]), u32(n[2]), out, TF_NOVIS);
        if (n[1] != n[2]) oriented(u32(n[0]), u32(n[2]), u32(n[3]), out, TF_NOVIS);
      }
    }
  for (int side = 0; side < 2; side++) {
    const std::vector<u32>& d = doorNodes[side];
    for (int i = 0; i < kDoor1 - kDoor0; i++) {
      const int n[4] = {int(d[size_t(i) * 2]), int(d[size_t(i + 1) * 2]), int(d[size_t(i + 1) * 2 + 1]), int(d[size_t(i) * 2 + 1])};
      faceOut(n, world({lerpd(X[kDoor0 + i], X[kDoor0 + i + 1], 0.5), 0.0, 1.10}, 0));
    }
  }

  sim.endObj();

  // ---- Solve a linear mass ramp over the shell for the kerb weight and the axle split.
  {
    constexpr double kTarget = 3400.0;
    const double cogX = kFrontAxle + (1.0 - P.frontShare) * (kRearAxle - kFrontAxle);
    const Obj& o = sim.objs[veh.obj];
    const V3 ox = world({0, 0, 0}, 0);
    double mOther = 0, sOther = 0, m0 = 0, m1 = 0, m2 = 0;
    std::vector<char> isShell(o.nodeStart + o.nodeCount, 0);
    for (u32 id : shell) isShell[id] = 1;
    for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) {
      const double x = ox.x - sim.pos[i].x;
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
    const double a = (Mb * m2 - Sb * m1) / det, b = (m0 * Sb - m1 * Mb) / det;
    for (u32 id : shell) {
      const double f = std::max(0.5, a + b * (ox.x - sim.pos[id].x));
      sim.mass[id] *= f;
      sim.invMass[id] = 1.0 / sim.mass[id];
    }
    double total = 0;
    for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) total += sim.mass[i];
    veh.totalMass = total;
  }

  // Everything sprung takes part in shape damping.
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

  if (rig) {
    rig->noseX = kNoseX;
    rig->tyreBead = 0.300f;
    rig->tyreWall = 0.170f;
    rig->tyreHalfWidth = 0.165f;
    const auto group = [&](const char* name, std::vector<u32> nodes) { rig->groups.push_back({name, std::move(nodes)}); };
    group("body", shell);
    std::vector<u32> frame;
    for (int s = 0; s < NS; s++)
      for (int r : {2, 3, 5})
        for (int side = 0; side < 2; side++)
          if (body[s][r][side] >= 0) frame.push_back(u32(body[s][r][side]));
    group("frame", frame);
    group("door_l", doorNodes[0]);
    group("door_r", doorNodes[1]);
    group("engine", veh.engineNodes);
    group("gearbox", veh.gearboxNodes);
    group("radiator", radNodes);
    group("tank", tankNodes);
  }
  return veh;
}
