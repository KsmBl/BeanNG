// Koenigsegg One:1 as a node/beam structure.
//
// Main dimensions follow the real car: length 4.50 m, width 2.06 m, height 1.15 m, wheelbase 2.66 m,
// 1360 kg, with a 5.0 twin-turbo V8 of 1360 PS behind the seats. A carbon tub that hardly gives way,
// with crash structures front and rear and brittle carbon skin that breaks off instead of denting.
// Station tables are shared with tools/gen_one1.py: x from the nose rearwards, y left, z up.
#include <algorithm>
#include <set>
#include "vehicle.h"

namespace {
constexpr int NS = 20, NR = 7;
constexpr double kNoseX = 2.25;
constexpr double kFrontAxle = 0.98, kRearAxle = 3.64;

const double X[NS] = {0.00, 0.12, 0.30, 0.55, 0.80, 1.00, 1.25, 1.50, 1.75, 2.00, 2.30, 2.60, 2.90, 3.20, 3.45, 3.70, 3.95, 4.20, 4.40, 4.50};
const double ZTOP[NS] = {0.42, 0.50, 0.58, 0.64, 0.69, 0.72, 0.75, 0.84, 0.98, 1.10, 1.15, 1.15, 1.12, 1.05, 0.98, 0.92, 0.88, 0.86, 0.84, 0.72};
const double W[NS] = {0.62, 0.78, 0.90, 0.98, 1.015, 1.03, 1.00, 0.96, 0.94, 0.94, 0.96, 0.99, 1.02, 1.03, 1.03, 1.03, 1.02, 0.98, 0.90, 0.72};
const double ZBELT[NS] = {0.34, 0.40, 0.52, 0.68, 0.78, 0.81, 0.77, 0.69, 0.70, 0.74, 0.74, 0.76, 0.80, 0.84, 0.87, 0.88, 0.86, 0.84, 0.82, 0.70};
const double ZRAIL[NS] = {0.40, 0.46, 0.56, 0.68, 0.78, 0.81, 0.79, 0.81, 0.93, 1.05, 1.10, 1.10, 1.06, 0.99, 0.93, 0.89, 0.86, 0.85, 0.83, 0.71};
const double WRAIL[NS] = {0.30, 0.42, 0.55, 0.62, 0.66, 0.67, 0.67, 0.65, 0.60, 0.55, 0.52, 0.52, 0.55, 0.62, 0.72, 0.80, 0.82, 0.80, 0.72, 0.55};
const double ZLOW[NS] = {0.20, 0.19, 0.20, 0.27, 0.42, 0.56, 0.42, 0.22, 0.19, 0.19, 0.19, 0.19, 0.22, 0.34, 0.50, 0.56, 0.48, 0.32, 0.27, 0.27};
const double ZFLOOR[NS] = {0.20, 0.19, 0.18, 0.18, 0.18, 0.18, 0.18, 0.17, 0.17, 0.17, 0.17, 0.17, 0.18, 0.18, 0.19, 0.19, 0.20, 0.21, 0.23, 0.25};
const double WFLOOR[NS] = {0.45, 0.55, 0.55, 0.42, 0.42, 0.42, 0.42, 0.62, 0.78, 0.78, 0.78, 0.78, 0.62, 0.45, 0.45, 0.45, 0.45, 0.50, 0.60, 0.55};

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

// The tub: carbon fibre takes enormous loads and then lets go at once.
const Mat mTub{3.0e6, 420, 120e3, 150e3, 4e5, 0.10, 210e3};
const Mat mTubCross{2.6e6, 380, 90e3, 110e3, 3e5, 0.10, 170e3};
// The crash structures are made to be used up.
const Mat mCrash{1.3e6, 230, 11e3, 26e3, 5e4, 0.45, 0};
const Mat mCrashCross{1.0e6, 180, 8e3, 18e3, 3e4, 0.40, 0};
// Skin: light, stiff, and it shatters.
const Mat mSkin{0.8e6, 130, 4.5e3, 7e3, 2e4, 0.18, 11e3};
const Mat mDoor{0.9e6, 150, 9e3, 14e3, 3e4, 0.25, 26e3};
const Mat mHinge{1.2e6, 220, 0, 0, 0, 0, 17e3};
const Mat mLatch{0.8e6, 160, 0, 0, 0, 0, 9e3};
const Mat mPylon{0.8e6, 120, 0, 0, 0, 0, 5e3};

const Mat mAnchor{3.0e6, 440, 65e3, 85e3, 2e5, 0.20, 0};
const Mat mSubframe{3.0e6, 440, 70e3, 90e3, 2e5, 0.22, 0};
const Mat mArm{4.0e6, 500, 78e3, 98e3, 3e5, 0.22, 0};
const Mat mHub{2.2e6, 280, 24e3, 36e3, 1.2e5, 0.25, 0};
const Mat mKnuckle{2.2e6, 280, 85e3, 110e3, 4e5, 0.25, 0};
const Mat mCarrier{3.2e6, 420, 90e3, 110e3, 4e5, 0.20, 0};
const Mat mTie{3.0e6, 360, 40e3, 50e3, 1.5e5, 0.12, 0};
const Mat mBlock{6e6, 900, 240e3, 240e3, 6e5, 0.0, 0};
const Mat mMount{4.5e5, 500, 16e3, 18e3, 3e4, 0.45, 0};
const Mat mTank{1.2e6, 240, 18e3, 22e3, 3e4, 0.30, 0};
const Mat mRadCore{1.0e6, 170, 6e3, 8e3, 1.5e4, 0.30, 0};
const Mat mRadMount{1.0e6, 160, 0, 0, 0, 0, 14e3};

// Power curve of the real engine: 1371 N m at 6000 rpm, 1360 PS at 7500, and a limiter at 8250.
const double kTorque[][2] = {{0, 500}, {1000, 620}, {2000, 780}, {3000, 1040}, {4000, 1210}, {5000, 1320},
                             {6000, 1371}, {7000, 1330}, {7500, 1295}, {8000, 1200}, {8250, 900}, {8500, 0}};
}  // namespace

Vehicle buildOne1(Sim& sim, V3 spawn, CarRig* rig) {
  Vehicle veh;
  veh.obj = sim.beginObj(OBJ_CAR, true);

  PowerSpec& P = veh.spec;
  P.torque = kTorque;
  P.torquePoints = int(sizeof(kTorque) / sizeof(kTorque[0]));
  P.frictionBase = 32.0;
  P.frictionPerRpm = 0.016;
  P.idleRpm = 1000.0;
  P.revLimit = 8250.0;
  P.nGears = 7;
  const double gears[7] = {2.86, 2.14, 1.67, 1.33, 1.06, 0.86, 0.71};
  std::copy(gears, gears + 7, P.gears);
  P.reverse = 2.86;
  P.finalDrive = 2.73;
  P.engineInertia = 0.26;
  P.clutchCapacity = 2100.0;
  P.synchroTorque = 180.0;
  P.shaftRate = 28000.0;
  P.shaftDamp = 26.0;
  P.shaftShear = 16000.0;
  P.upshiftLow = 2800.0;
  P.upshiftHigh = 8000.0;
  P.downshiftLow = 1500.0;
  P.downshiftHigh = 4300.0;
  P.tyreMu = 1.34;
  P.rimRadius = 0.255;
  P.dentFz = 30000.0;   // a stiff, flat tyre reaches higher loads for the same blow
  P.burstFz = 52000.0;
  P.conventional = true;
  P.turbo = true;
  P.diffPreload = 350.0;
  P.diffRamp = 0.45;
  P.frontShare = 0.44;
  P.lockAngle = 0.52;
  P.wheelbase = kRearAxle - kFrontAxle;
  P.understeer = 0.0012;
  P.downforceCla = 2.0;     // about 610 kg at 260 km/h
  P.rpmMax = 9000;
  P.rpmRed = 7900;
  veh.dragArea = 0.72;      // a big wing and wide tyres cost more than the shape saves
  veh.sportMode = true;     // firm valves always: the struts below are plain gas dampers
  veh.hydractiveSoft = false;
  veh.activaOn = false;
  veh.engineOmega = 105;
  veh.discOmega = 105;
  veh.fuelLitres = 80;

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
  constexpr int kDoor0 = 9, kDoor1 = 11;  // door-only stations: 9, 10, 11
  const auto doorOnly = [](int s, int r) { return s >= kDoor0 && s <= kDoor1 && (r == 3 || r == 4); };
  int body[NS][NR][2];
  for (auto& a : body)
    for (auto& b : a) b[0] = b[1] = -1;
  const double massBase[NR] = {2.6, 3.4, 4.2, 2.6, 2.6, 2.2, 1.8};
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

  const auto inTub = [](int s) { return s >= 7 && s <= 13; };
  const auto longMat = [&](int s, int r) -> const Mat& {
    if (inTub(s)) return r <= 2 ? mTub : (r == 5 ? mTub : mSkin);
    if (r <= 2) return mCrash;
    return mSkin;
  };
  const auto aroundMat = [&](int s, int r) -> const Mat& {
    if (inTub(s)) return r <= 1 || s == 8 || s == 12 ? mTubCross : mSkin;
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
    int dn[3][2];
    for (int s = kDoor0; s <= kDoor1; s++)
      for (int r = 3; r <= 4; r++) {
        const u32 id = sim.addNode(world(design(s, r), side), 2.4, NF_COLLIDE, 0.45f);
        dn[s - kDoor0][r - 3] = int(id);
        door.push_back(id);
      }
    // Inner frame, which gives the door depth.
    u32 inner[3];
    for (int i = 0; i < 3; i++) {
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
      beam(u32(dn[2][r]), u32(body[kDoor1 + 1][3][side]), mLatch);
      beam(u32(dn[2][r]), u32(body[kDoor1 + 1][4][side]), mLatch);
    }
    // Sill and roof rail hold it in place sideways through its own frame.
    for (int i = 0; i < 3; i++) {
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
  block(2.92, 3.48, -0.30, 0.30, 0.24, 0.68, 190.0, veh.engineNodes, mBlock, &veh.engineBlockBeams);
  block(3.50, 3.98, -0.20, 0.20, 0.24, 0.52, 70.0, veh.gearboxNodes, mBlock, &veh.gearboxBeams);
  for (int i = 0; i < 8; i += 2)
    for (int j = 0; j < 4; j++) beam(veh.engineNodes[size_t(i) + 1], veh.gearboxNodes[size_t(j) * 2], mAnchor);
  for (u32 n : veh.engineNodes) anchor(n, 3, mMount, &veh.engineMountBeams);
  for (u32 n : veh.gearboxNodes) anchor(n, 2, mMount, &veh.engineMountBeams);
  // Flanges for the rear driveshafts.
  for (int side = 0; side < 2; side++) {
    const u32 f = sim.addNode(world({kRearAxle, 0.21, 0.33}, side), 4.0, 0, 0.4f);
    for (u32 n : veh.gearboxNodes) beam(f, n, mAnchor);
    veh.shaftNode[side] = f;
  }

  // Radiators in the nose, the tank between cabin and engine, as on the real car.
  std::vector<u32> radNodes, tankNodes;
  block(0.42, 0.52, -0.50, 0.50, 0.26, 0.48, 16.0, radNodes, mRadCore, &veh.radiatorBeams);
  for (u32 n : radNodes) anchor(n, 3, mRadMount);
  block(2.66, 2.88, -0.55, 0.55, 0.24, 0.62, 62.0, tankNodes, mTank, &veh.tankBeams);
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
    w.driven = !front;
    w.brakeMax = front ? 5200.0 : 3400.0;   // carbon ceramics, and all the grip to use them
    w.R = 0.340;
    w.width = front ? 0.265 : 0.345;
    w.I = front ? 1.5 : 2.0;
    w.kRadial = 340e3;
    w.cRadial = 620;
    const double hx = front ? kFrontAxle : kRearAxle;
    const double hy = front ? 0.82 : 0.81;
    const double hz = w.R;
    const auto node = [&](double x, double y, double z, double m, u8 fl = NF_COLLIDE) { return sim.addNode(world({x, y, z}, side), m, fl, 0.5f); };
    w.axIn = node(hx, hy - 0.07, hz, 5.5, 0);
    w.axOut = node(hx, hy + 0.07, hz, 5.5, 0);
    for (int i = 0; i < 6; i++) {
      const double ang = PI / 3.0 * i;
      w.ring[i] = node(hx - std::sin(ang) * 0.255, hy, hz + std::cos(ang) * 0.255, 3.5, NF_COLLIDE | NF_WHEEL);
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
    const u32 kl = node(hx, hy - 0.06, 0.20, 4.0, 0);
    const u32 ku = node(hx + 0.015, hy - 0.11, 0.50, 4.0, 0);
    const u32 ks = node(hx + 0.13, hy - 0.12, 0.26, 2.5, 0);
    const u32 kf = node(hx - 0.13, hy - 0.10, 0.30, 2.5, 0);
    for (u32 a : {w.axIn, w.axOut})
      for (u32 c : {kl, ku, ks, kf}) beam(c, a, mCarrier);
    for (u32 a : {kl, ku, ks, kf})
      for (u32 c : {kl, ku, ks, kf}) beam(a, c, mCarrier);
    for (int i = 0; i < 6; i++)
      for (u32 c : {kl, ku, ks, kf}) beam(w.ring[i], c, mKnuckle);

    // Pick-ups on a subframe node set, tied firmly into the structure.
    Pick& p = pick[front ? 0 : 1][side];
    p.lf = node(hx - 0.24, 0.40, 0.19, 5.0, 0);
    p.lr = node(hx + 0.24, 0.40, 0.19, 5.0, 0);
    p.uf = node(hx - 0.20, 0.44, 0.47, 4.5, 0);
    p.ur = node(hx + 0.20, 0.44, 0.47, 4.5, 0);
    p.rack = node(hx + 0.13, 0.36, 0.26, 3.0, 0);
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
    if (!front) addLink(w.axIn, veh.shaftNode[side], 0.02f, MM_METAL);

    // A gas damper unit, with the firm rate of a track car.
    const u32 tower = node(hx, 0.56, 0.60, 3.5, 0);
    anchor(tower, 5, mAnchor);
    beam(tower, p.uf, mAnchor);
    beam(tower, p.ur, mAnchor);
    addLink(tower, kl, 0.024f, MM_METAL, 1);
    HydroStrut st;
    st.top = tower;
    st.low = kl;
    st.A = 5e-4;
    st.V0 = front ? 45e-6 : 55e-6;
    st.p0 = 25e5;
    st.bump[0] = st.bump[1] = front ? 2400 : 2900;
    st.reb[0] = st.reb[1] = front ? 3700 : 4400;
    const double L = len(sim.pos[st.low] - sim.pos[st.top]);
    st.L = st.Lnom = L;
    st.Lmin = L - 0.075;
    st.Lmax = L + 0.085;
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
  for (int i = 0; i < 2; i++) veh.shaftLen[i] = len(sim.pos[veh.wheels[size_t(i) + 2].axIn] - sim.pos[veh.shaftNode[i]]);
  veh.steerTravel = 0.058;
  veh.axle[0].kBar = 16e3;
  veh.axle[1].kBar = 11e3;

  veh.refFrontL = u32(body[8][2][0]);
  veh.refFrontR = u32(body[8][2][1]);
  veh.refRearL = u32(body[13][2][0]);
  veh.refRearR = u32(body[13][2][1]);

  // ---- The rear wing stands on two pylons, and comes off in one piece.
  std::vector<u32> wing;
  {
    u32 wn[6];
    int k = 0;
    for (double x : {4.12, 4.46})
      for (double y : {-0.90, 0.0, 0.90}) wn[k++] = sim.addNode(world({x, y, 1.06 + (x > 4.3 ? 0.03 : 0.0)}, 0), 1.6, NF_COLLIDE, 0.4f);
    // The feet of the two pylons, down on the deck. They belong to the wing, and they also give it depth:
    // the visible wing follows these nodes, and points all in one plane say nothing about up and down.
    u32 foot[2];
    for (int side = 0; side < 2; side++) foot[side] = sim.addNode(world({4.24, 0.42, 0.87}, side), 1.4, 0, 0.4f);
    for (u32 a : wn) {
      for (u32 b : wn) beam(a, b, mSkin);
      for (u32 f : foot) beam(a, f, mSkin);
    }
    beam(foot[0], foot[1], mSkin);
    for (u32 n : wn) wing.push_back(n);
    wing.push_back(foot[0]);
    wing.push_back(foot[1]);
    for (int side = 0; side < 2; side++)
      for (int s : {15, 16, 17})
        for (int r : {5, 6}) beam(foot[side], u32(body[s][r][side]), mPylon);
  }

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
        faceOut(n, world({lerpd(X[s], X[s + 1], 0.5), 0.0, 0.52}, 0));
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
    for (int i = 0; i < 2; i++) {
      const int n[4] = {int(d[size_t(i) * 2]), int(d[size_t(i + 1) * 2]), int(d[size_t(i + 1) * 2 + 1]), int(d[size_t(i) * 2 + 1])};
      faceOut(n, world({lerpd(X[kDoor0 + i], X[kDoor0 + i + 1], 0.5), 0.0, 0.52}, 0));
    }
  }

  sim.endObj();

  // ---- Solve a linear mass ramp over the shell for the kerb weight and the axle split.
  {
    constexpr double kTarget = 1360.0;
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
    rig->tyreBead = 0.250f;
    rig->tyreWall = 0.090f;
    rig->tyreHalfWidth = 0.1325f;
    rig->rearWidth = 1.30f;
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
    group("wing", wing);
    group("engine", veh.engineNodes);
    group("gearbox", veh.gearboxNodes);
    group("radiator", radNodes);
    group("tank", tankNodes);
  }
  return veh;
}
