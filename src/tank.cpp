#include "tank.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {
// Rolled homogeneous armour, welded: takes a beating, and does not simply crumple.
const Mat mArmour{6e7, 15000, 9e6, 12e6, 2e7, 0.20, 0};
const Mat mBrace{2.5e7, 2500, 0, 0, 0, 0, 0};
const Mat mCast{5e7, 5000, 0, 0, 0, 0, 0};
const Mat mGun{3e7, 3000, 0, 0, 0, 0, 0};

constexpr double kNoseX = 3.55;  // hull nose ahead of the spawn point
constexpr double kWheelX[7] = {0.98, 1.78, 2.58, 3.38, 4.18, 4.98, 5.78};
constexpr double kTrackY = 1.35;   // centre of the track
constexpr double kSprocketX = 6.46, kSprocketZ = 0.66;
constexpr double kIdlerX = 0.32, kIdlerZ = 0.66, kIdlerR = 0.30;
constexpr double kTopRunZ = 1.03;

constexpr double kPivotYawRate = 1.1;
constexpr double kRatios[5] = {8.2, 3.6 * 4.2 / 1.0, 2.2 * 4.2, 1.45 * 4.2, 4.2};  // reverse, 1st..4th, to the sprocket

double engineTorque(double rpm) {
  // MTU MB 838 multi-fuel V10: 610 kW and about 2700 N m in reality. Raised by a quarter, so that it feels as
  // quick as a tank this size should when you are the one driving it.
  const double r = clampd(rpm, 300, 2500);
  const double shape = r < 1500 ? lerpd(0.62, 1.0, (r - 600) / 900.0) : lerpd(1.0, 0.98, (r - 1500) / 700.0);
  return 3400.0 * clampd(shape, 0.4, 1.0) * 1.0;
}

V3 toWorld(V3 spawn, double x, double y, double z) { return spawn + V3{kNoseX - x, y, z}; }

// Barycentric weights of a point in a tetrahedron. A force split by them keeps force and moment exact.
void tetWeights(const V3 n[4], V3 point, double w[4]) {
  const V3 e1 = n[1] - n[0], e2 = n[2] - n[0], e3 = n[3] - n[0];
  const double det = dot(e1, cross(e2, e3));
  const V3 q = point - n[0];
  w[1] = dot(q, cross(e2, e3)) / det;
  w[2] = dot(e1, cross(q, e3)) / det;
  w[3] = dot(e1, cross(e2, q)) / det;
  w[0] = 1.0 - w[1] - w[2] - w[3];
}
}  // namespace

const TurretSpec& turretSpec(TurretKind k) {
  //                       name               mesh                       mass power  gun   recoil  force   reload ammo speed  blast  radius life  pitch      yaw pitch missiles
  static const TurretSpec table[TK_COUNT] = {
      {"105-mm-Kanone", "assets/leopard.mesh", 1.00, 1.00, 5.45, 0.34, 200e3, 4.0, 8, 1100.0, 2200.0, 15.0, 8.0, -9, 20, 40, 22, 0},
      {"Leichte Kanone", "assets/leopard_light.mesh", 0.40, 1.00, 2.90, 0.06, 12e3, 0.3, 120, 1300.0, 170.0, 4.0, 6.0, -8, 35, 70, 45, 0},
      // Slow enough that the whole map lies inside the high arc: half a minute in the air from end to end.
      {"Artillerie", "assets/leopard_arty.mesh", 1.80, 0.70, 6.40, 0.50, 260e3, 12.0, 8, 160.0, 12000.0, 30.0, 60.0, -3, 78, 14, 10, 0},
      {"Lenkraketen", "assets/leopard_missile.mesh", 0.85, 0.95, 2.20, 0.05, 8e3, 3.0, 20, 700.0, 260.0, 5.0, 6.0, -6, 20, 40, 30, 8},
  };
  return table[k];
}

const HullSpec& hullSpec(HullKind k) {
  // A heavy one that takes twice the hits and is in no hurry, a light one that outruns everything and does not last.
  static const HullSpec table[HK_COUNT] = {
      {"Mittlerer Panzer", "", 1.0, 1.0, 1.0},
      {"Schwerer Panzer", "_heavy", 1.45, 1.05, 0.5},
      {"Schneller Panzer", "_fast", 0.62, 0.95, 1.7},
  };
  return table[k];
}

std::string tankMeshFile(HullKind hull, TurretKind turret) {
  static const char* kTurret[TK_COUNT] = {"", "_light", "_arty", "_missile"};
  return std::string("assets/leopard") + hullSpec(hull).meshPart + kTurret[turret] + ".mesh";
}

Tank buildLeopard(Sim& sim, V3 spawn, CarRig* rig, TurretKind kind, HullKind hullKind) {
  Tank T;
  T.kind = kind;
  T.hull = hullKind;
  const double heft = hullSpec(hullKind).massScale;
  // Lighter means softer to go with it, for the same reason as the turret's, and the springs carry what there is.
  const Mat mArmourH{mArmour.k * std::min(heft, 1.0), mArmour.c * std::min(heft, 1.0), mArmour.yC, mArmour.yT, mArmour.hard, mArmour.maxStrain, 0};
  const double totalScale = (heft * 33.5 + 9.0) / 42.5;
  const TurretSpec& spec = turretSpec(kind);
  T.reloadTime = spec.reload;
  T.missiles = spec.missiles;
  // A lighter turret has softer beams as well: the fastest thing in it sets the limit on the time step.
  const double stiff = std::min(spec.massScale, 1.0);
  const Mat mCastT{mCast.k * stiff, mCast.c * stiff, 0, 0, 0, 0, 0};
  const Mat mGunT{mGun.k * stiff, mGun.c * stiff, 0, 0, 0, 0, 0};
  std::vector<u32> hull;
  T.hullObj = sim.beginObj(OBJ_CAR, true);

  // ---- Hull: six stations from the nose to the tail, six nodes each.
  struct Station {
    double x, zBottom, zSponson, zTop, mass;
  };
  const Station st[6] = {
      {0.00, 0.72, 0.76, 0.82, 400},
      {0.62, 0.46, 0.84, 1.08, 500},
      {1.70, 0.46, 0.86, 1.30, 560},
      {3.60, 0.46, 0.86, 1.32, 620},
      {5.40, 0.46, 0.86, 1.32, 850},
      {7.02, 0.68, 0.88, 1.24, 750},
  };
  u32 ids[6][6];
  for (int s = 0; s < 6; s++) {
    const double ys[3] = {1.00, 1.60, 1.56};
    const double zs[3] = {st[s].zBottom, st[s].zSponson, st[s].zTop};
    for (int r = 0; r < 3; r++)
      for (int d = 0; d < 2; d++) {
        const double y = (d == 0 ? 1.0 : -1.0) * ys[r];
        ids[s][r * 2 + d] = sim.addNode(toWorld(spawn, st[s].x, y, zs[r]), st[s].mass * heft, NF_COLLIDE, 0.5f);
        hull.push_back(ids[s][r * 2 + d]);
      }
  }
  for (int s = 0; s < 6; s++) {
    for (int a = 0; a < 6; a++)
      for (int b = a + 1; b < 6; b++) sim.addBeam(ids[s][a], ids[s][b], mArmourH);
    if (s < 5)
      for (int a = 0; a < 6; a++)
        for (int b = 0; b < 6; b++) sim.addBeam(ids[s][a], ids[s + 1][b], mArmourH);
    if (s < 4)
      for (int a = 0; a < 6; a++) sim.addBeam(ids[s][a], ids[s + 2][a], mArmourH);
  }
  // Skin, for what runs into it. All facing outward.
  const V3 hullCentre = toWorld(spawn, 3.5, 0, 0.9);
  auto face = [&](u32 a, u32 b, u32 c) {
    const V3 n = cross(sim.pos[b] - sim.pos[a], sim.pos[c] - sim.pos[a]);
    if (dot(n, sim.pos[a] - hullCentre) < 0) std::swap(b, c);
    sim.addTri(a, b, c, TF_NOVIS, 0, 0.5f);
  };
  auto quad = [&](u32 a, u32 b, u32 c, u32 d) {
    face(a, b, c);
    face(a, c, d);
  };
  const int loop[6] = {0, 2, 4, 5, 3, 1};
  for (int s = 0; s < 5; s++)
    for (int k = 0; k < 6; k++) quad(ids[s][loop[k]], ids[s + 1][loop[k]], ids[s + 1][loop[(k + 1) % 6]], ids[s][loop[(k + 1) % 6]]);
  for (int s : {0, 5}) {
    quad(ids[s][0], ids[s][2], ids[s][3], ids[s][1]);
    quad(ids[s][2], ids[s][4], ids[s][5], ids[s][3]);
  }

  T.refFL = ids[1][4];
  T.refFR = ids[1][5];
  T.refRL = ids[4][4];
  T.refRR = ids[4][5];
  T.refBottom = ids[3][0];

  // Ties a new node to the nearest hull nodes.
  auto tie = [&](u32 n, int count, const Mat& m) {
    std::vector<std::pair<double, u32>> near;
    for (u32 h : hull) near.push_back({len2(sim.pos[h] - sim.pos[n]), h});
    std::sort(near.begin(), near.end());
    for (int i = 0; i < count && i < int(near.size()); i++) sim.addBeam(n, near[size_t(i)].second, m);
  };

  // ---- Running gear: seven road wheels a side on trailing arms and torsion bars. The bar is modelled
  // by a preloaded spring with the wheel rate of the real one, about 400 kN/m.
  for (int side = 0; side < 2; side++) {
    const double sg = side == 0 ? 1.0 : -1.0;
    for (int i = 0; i < 7; i++) {
      const double x = kWheelX[i];
      Tank::RoadWheel w;
      w.left = side == 0;
      w.in = sim.addNode(toWorld(spawn, x, sg * 1.09, Tank::kWheelR), 190, NF_COLLIDE, 0.6f);
      w.out = sim.addNode(toWorld(spawn, x, sg * 1.61, Tank::kWheelR), 190, NF_COLLIDE, 0.6f);
      const u32 p0 = sim.addNode(toWorld(spawn, x - 0.40, sg * 1.04, 0.62), 110, 0);
      const u32 p1 = sim.addNode(toWorld(spawn, x - 0.40, sg * 0.70, 0.60), 110, 0);
      const u32 top = sim.addNode(toWorld(spawn, x, sg * 1.20, 1.18), 80, 0);
      tie(p0, 5, mBrace);
      tie(p1, 5, mBrace);
      tie(top, 5, mBrace);
      sim.addBeam(p0, p1, mBrace);
      sim.addBeam(p0, top, mBrace);
      sim.addBeam(w.in, w.out, mBrace);
      sim.addBeam(w.in, p0, mBrace);
      sim.addBeam(w.in, p1, mBrace);
      sim.addBeam(w.out, p0, mBrace);
      sim.addBeam(w.out, p1, mBrace);
      const bool end = i == 0 || i == 6;  // real dampers sit on the first and last stations
      const Mat spring{2.0e5 * totalScale, (end ? 9000.0 : 3000.0) * totalScale, 0, 0, 0, 0, 0};
      constexpr double kPreload = 12.9e3 / 2.0e5;
      for (u32 hub : {w.in, w.out}) {
        const u32 id = sim.addBeam(hub, top, spring);
        sim.beams[id].L0 += kPreload;
      }
      T.wheels.push_back(w);
    }
    // Sprocket and idler are fixed to the hull: two nodes each, to draw them between.
    auto axle = [&](double x, double z, u32 out[2]) {
      out[0] = sim.addNode(toWorld(spawn, x, sg * 1.09, z), 90, NF_COLLIDE, 0.5f);
      out[1] = sim.addNode(toWorld(spawn, x, sg * 1.61, z), 90, NF_COLLIDE, 0.5f);
      tie(out[0], 5, mBrace);
      tie(out[1], 5, mBrace);
      sim.addBeam(out[0], out[1], mBrace);
    };
    axle(kSprocketX, kSprocketZ, T.sprocket[side]);
    axle(kIdlerX, kIdlerZ, T.idler[side]);
  }
  sim.endObj();
  sim.objs[T.hullObj].armoured = true;
  T.rigidNodes_ = hull;
  for (u32 i = sim.objs[T.hullObj].nodeStart; i < sim.objs[T.hullObj].nodeStart + sim.objs[T.hullObj].nodeCount; i++)
    if (sim.mass[i] < 200.0 && sim.mass[i] > 60.0 && std::find(hull.begin(), hull.end(), i) == hull.end()) T.rigidNodes_.push_back(i);
  std::sort(T.rigidNodes_.begin(), T.rigidNodes_.end());
  T.rigidNodes_.erase(std::unique(T.rigidNodes_.begin(), T.rigidNodes_.end()), T.rigidNodes_.end());

  // ---- Turret and gun: a body of their own, held to the hull by the turret ring.
  T.turretObj = sim.beginObj(OBJ_CAR, true);
  sim.objs[T.turretObj].armoured = true;
  const double tcx = 2.90, tcz = 1.34;  // the ring's centre, design coordinates
  T.turretAxis = V3{kNoseX - tcx, 0, tcz};
  auto addTurretNode = [&](double f, double l, double z, double m) {
    const u32 n = sim.addNode(spawn + T.turretAxis + V3{f, l, z}, m * spec.massScale, NF_COLLIDE, 0.5f);
    T.turretNodes.push_back(n);
    T.turretRest.push_back(V3{f, l, z});
    return n;
  };
  // Lower ring, upper ring and the bustle at the rear.
  std::vector<u32> tn;
  for (int i = 0; i < 8; i++) {
    const double a = 2.0 * PI * i / 8.0;
    tn.push_back(addTurretNode(std::cos(a) * 1.28, std::sin(a) * 1.28, 0.10, 560));
  }
  for (int i = 0; i < 6; i++) {
    const double a = 2.0 * PI * i / 6.0;
    tn.push_back(addTurretNode(std::cos(a) * 0.98 - 0.08, std::sin(a) * 0.95, 0.82, 460));
  }
  tn.push_back(addTurretNode(-1.72, 0.48, 0.40, 300));
  tn.push_back(addTurretNode(-1.72, -0.48, 0.40, 300));
  tn.push_back(addTurretNode(-1.55, 0, 0.84, 240));
  for (size_t a = 0; a < tn.size(); a++)
    for (size_t b = a + 1; b < tn.size(); b++) sim.addBeam(tn[a], tn[b], mCastT);
  const V3 tCentre = spawn + T.turretAxis + V3{0, 0, 0.45};
  auto tface = [&](u32 a, u32 b, u32 c) {
    const V3 n = cross(sim.pos[b] - sim.pos[a], sim.pos[c] - sim.pos[a]);
    if (dot(n, sim.pos[a] - tCentre) < 0) std::swap(b, c);
    sim.addTri(a, b, c, TF_NOVIS, 0, 0.5f);
  };
  for (int i = 0; i < 6; i++) {
    const u32 u0 = tn[size_t(8 + i)], u1 = tn[size_t(8 + (i + 1) % 6)];
    const int l0 = i * 8 / 6, l1 = (i + 1) * 8 / 6 % 8;
    tface(tn[size_t(l0)], tn[size_t(l1)], u1);
    tface(tn[size_t(l0)], u1, u0);
    tface(u0, u1, tn[16]);
  }
  // The gun: trunnions in the front of the turret, breech behind them, 5.5 m of barrel ahead.
  T.trunnion = V3{0.92, 0, 0.56};
  auto addGunNode = [&](double f, double l, double z, double m) {
    const V3 local{f, l, z};
    const u32 n = sim.addNode(spawn + T.turretAxis + T.trunnion + local, m * spec.massScale, 0, 0.5f);
    T.gunNodes.push_back(n);
    T.gunRest.push_back(local);
    return n;
  };
  const u32 g0 = addGunNode(-1.1, 0.12, 0, 420);
  const u32 g1 = addGunNode(-1.1, -0.12, 0, 420);
  const u32 g2 = addGunNode(-0.8, 0, 0.16, 280);
  const u32 g3 = addGunNode(0.45, 0.10, 0, 400);
  const u32 g4 = addGunNode(0.45, -0.10, 0, 400);
  const u32 g5 = addGunNode(spec.gunLen, 0, 0, 180);
  const u32 gn[6] = {g0, g1, g2, g3, g4, g5};
  for (int a = 0; a < 6; a++)
    for (int b = a + 1; b < 6; b++) sim.addBeam(gn[a], gn[b], mGunT);
  sim.endObj();

  double total = 0;
  for (u32 oi : {u32(T.hullObj), u32(T.turretObj)}) {
    const Obj& o = sim.objs[oi];
    for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) total += sim.mass[i];
  }
  printf("%s, %s: %.1f t\n", hullSpec(hullKind).name, spec.name, total / 1000.0);

  if (rig) {
    rig->noseX = kNoseX;
    rig->carWheels = false;
    rig->scratches = false;
    rig->groups.clear();
    rig->groups.push_back({"hull", hull});
    std::vector<u32> tur = T.turretNodes;
    rig->groups.push_back({"turret", tur});
    std::vector<u32> gun = T.gunNodes;
    rig->groups.push_back({"gun", gun});
    rig->links.clear();
    for (const Tank::RoadWheel& w : T.wheels) {
      rig->links.push_back({w.in, w.out, float(Tank::kWheelR - 0.035), MM_RUBBER, 0});
      rig->links.push_back({w.in, w.out, 0.235f, MM_OLIVE, 2});
    }
    for (int s = 0; s < 2; s++) {
      rig->links.push_back({T.sprocket[s][0], T.sprocket[s][1], float(Tank::kSprocketR - 0.02), MM_METAL, 0});
      rig->links.push_back({T.idler[s][0], T.idler[s][1], float(kIdlerR - 0.02), MM_METAL, 0});
    }
  }
  return T;
}

void Tank::addToHull(Sim& sim, V3 point, V3 force) {
  const u32 id[4] = {refFL, refFR, refRL, refBottom};
  const V3 p[4] = {sim.pos[id[0]], sim.pos[id[1]], sim.pos[id[2]], sim.pos[id[3]]};
  double w[4];
  tetWeights(p, point, w);
  for (int i = 0; i < 4; i++) sim.force[id[i]] += force * w[i];
}

// 40 t of welded plate on the stiff beams of this model would ring like a bell after a blow, and throw
// the tank back off a wall. Real armour soaks that up in its welds, fittings and in the wall itself.
// Everything that moves differently from the hull as a rigid body has that difference braked.
void Tank::dampHull(Sim& sim, double dt) {
  double M = 0;
  V3 c{}, v{};
  for (u32 n : rigidNodes_) {
    M += sim.mass[n];
    c += sim.pos[n] * sim.mass[n];
    v += sim.vel[n] * sim.mass[n];
  }
  c = c / M;
  v = v / M;
  V3 L{};
  double I[6] = {};  // xx yy zz xy xz yz
  for (u32 n : rigidNodes_) {
    const double m = sim.mass[n];
    const V3 r = sim.pos[n] - c;
    L += cross(r, sim.vel[n] - v) * m;
    I[0] += m * (r.y * r.y + r.z * r.z);
    I[1] += m * (r.x * r.x + r.z * r.z);
    I[2] += m * (r.x * r.x + r.y * r.y);
    I[3] -= m * r.x * r.y;
    I[4] -= m * r.x * r.z;
    I[5] -= m * r.y * r.z;
  }
  // Inverse of the symmetric inertia matrix, by cofactors.
  const double a = I[1] * I[2] - I[5] * I[5], b = I[4] * I[5] - I[3] * I[2], d = I[3] * I[5] - I[4] * I[1];
  const double det = I[0] * a + I[3] * b + I[4] * d;
  if (std::fabs(det) < 1e-6) return;
  const double e = I[0] * I[2] - I[4] * I[4], f = I[3] * I[4] - I[0] * I[5], g = I[0] * I[1] - I[3] * I[3];
  const V3 om{(a * L.x + b * L.y + d * L.z) / det, (b * L.x + e * L.y + f * L.z) / det, (d * L.x + f * L.y + g * L.z) / det};
  const double k = std::min(350.0, 0.3 / dt);
  for (u32 n : rigidNodes_) {
    const V3 rigid = v + cross(om, sim.pos[n] - c);
    sim.force[n] -= (sim.vel[n] - rigid) * (sim.mass[n] * k);
  }
}

void Tank::updateFrame(Sim& sim, double dt) {
  const V3 fl = sim.pos[refFL], fr = sim.pos[refFR], rl = sim.pos[refRL], rr = sim.pos[refRR];
  fwd = norm(fl + fr - rl - rr);
  const V3 l0 = fl + rl - fr - rr;
  up = norm(cross(fwd, l0));
  left = cross(up, fwd);
  cog = (fl + fr + rl + rr) * 0.25 - up * 0.45;
  vCog = (sim.vel[refFL] + sim.vel[refFR] + sim.vel[refRL] + sim.vel[refRR]) * 0.25;
  speed = dot(vCog, fwd);
  const V3 acc = (vCog - prevV_) / dt;
  prevV_ = vCog;
  const double a = clampd(dt / 0.15, 0, 1);
  lonAcc += (clampd(dot(acc, fwd), -40, 40) - lonAcc) * a;
  latAcc += (clampd(dot(acc, left), -40, 40) - latAcc) * a;
  roll = std::asin(clampd(left.z, -1, 1));
}

void Tank::updateTurret(Sim& sim, const Controls& in, double dt) {
  const V3 O = (sim.pos[refFL] + sim.pos[refFR] + sim.pos[refRL] + sim.pos[refRR]) * 0.25;
  const V3 O0 = (sim.rest[refFL] + sim.rest[refFR] + sim.rest[refRL] + sim.rest[refRR]) * 0.25;
  const V3 spawnApprox = O0;  // the frame's origin as built
  const V3 vO = vCog;
  // The ring's centre relative to the frame origin, in the hull's axes.
  const Obj& hullO = sim.objs[hullObj];
  (void)hullO;
  auto hullToWorld = [&](V3 q) { return O + fwd * q.x + left * q.y + up * q.z; };
  const V3 ringLocal = sim.rest[turretNodes[0]] - turretRest[0] - spawnApprox;

  // ---- Aim: the stabiliser holds the gun on the point the gunner is looking at.
  const double cy0 = std::cos(turretYaw), sy0 = std::sin(turretYaw);
  const V3 tF = fwd * cy0 + left * sy0, tL = left * cy0 - fwd * sy0;
  const V3 ring = hullToWorld(ringLocal);
  ring_ = ring;
  const TurretSpec& sp_ = spec();
  const V3 tp0 = ring + tF * trunnion.x + tL * trunnion.y + up * trunnion.z;
  // While a missile is in the air the same hand is flying it.
  if (len2(in.aimPoint) > 1.0 && !missileLive) {
    const V3 to = in.aimPoint - tp0;
    // The arc that lands on the point, worked out against the true vertical: on a high lob a hull leaning by one
    // degree would otherwise put the shell a hundred metres off. High over the top if asked for and possible.
    const double v2 = sp_.shellSpeed * sp_.shellSpeed, range = std::max(std::hypot(to.x, to.y), 1.0), rise = to.z;
    const double disc = v2 * v2 - 9.81 * (9.81 * range * range + 2.0 * rise * v2);
    double solved = 45.0 * PI / 180.0;  // out of reach: as far as it will go
    if (disc >= 0) {
      const double high = std::atan2(v2 + std::sqrt(disc), 9.81 * range), low = std::atan2(v2 - std::sqrt(disc), 9.81 * range);
      solved = in.indirect && high <= sp_.pitchMax * PI / 180.0 ? high : low;
    }
    const V3 away = V3{to.x, to.y, 0} / range;
    const V3 lay = away * std::cos(solved) + V3{0, 0, 1} * std::sin(solved);
    const double wantYaw = std::atan2(dot(lay, left), dot(lay, fwd));
    double e = wantYaw - turretYaw;
    while (e > PI) e -= 2 * PI;
    while (e < -PI) e += 2 * PI;
    // A damaged ring turns slowly, and one that is gone does not turn at all.
    const double kYawRate = sp_.yawRate * PI / 180.0 * (mod[TM_RING] <= 0 ? 0.0 : lerpd(0.4, 1.0, mod[TM_RING]));
    // The elevation gear shot through: the gun moves slowly, and at the end stays where it is.
    const double kPitchRate = sp_.pitchRate * PI / 180.0 * (mod[TM_ELEVATION] <= 0 ? 0.0 : lerpd(0.25, 1.0, mod[TM_ELEVATION]));
    turretYaw += clampd(e * 6.0, -kYawRate, kYawRate) * dt;
    if (turretYaw > PI) turretYaw -= 2 * PI;
    if (turretYaw < -PI) turretYaw += 2 * PI;
    const double wantPitch = clampd(std::asin(clampd(dot(lay, up), -1, 1)), sp_.pitchMin * PI / 180.0, sp_.pitchMax * PI / 180.0);
    gunPitch += clampd((wantPitch - gunPitch) * 6.0, -kPitchRate, kPitchRate) * dt;
  }

  const double cy = std::cos(turretYaw), sy = std::sin(turretYaw);
  const V3 eF = fwd * cy + left * sy, eL = left * cy - fwd * sy;
  const double cp = std::cos(gunPitch), sp = std::sin(gunPitch);
  const V3 gF = eF * cp + up * sp, gU = up * cp - eF * sp;
  const V3 trun = ring + eF * trunnion.x + eL * trunnion.y + up * trunnion.z;
  muzzle = trun + gF * sp_.gunLen;
  gunDir = gF;
  sight = ring + eF * 0.25 + eL * 0.45 + up * 1.15;

  // ---- The gun runs back in its cradle and returns.
  recoilTime += dt;
  recoil = recoilTime < 0.05 ? sp_.recoilLen * recoilTime / 0.05 : sp_.recoilLen * std::max(0.0, 1.0 - (recoilTime - 0.05) / 0.9);

  // ---- The turret ring: every node pulled to its place, the reaction into the hull.
  constexpr double kOmega = 95.0, kZeta = 0.9;
  auto hold = [&](u32 n, V3 target) {
    const double m = sim.mass[n];
    const V3 F = (target - sim.pos[n]) * (m * kOmega * kOmega) + (vO - sim.vel[n]) * (2.0 * kZeta * m * kOmega);
    sim.force[n] += F;
    addToHull(sim, sim.pos[n], F * -1.0);
  };
  for (size_t i = 0; i < turretNodes.size(); i++) {
    const V3& q = turretRest[i];
    hold(turretNodes[i], ring + eF * q.x + eL * q.y + up * q.z);
  }
  for (size_t i = 0; i < gunNodes.size(); i++) {
    const V3& q = gunRest[i];
    hold(gunNodes[i], trun + gF * (q.x - recoil) + eL * q.y + gU * q.z);
  }

  // ---- Fire.
  reload -= dt;
  if (in.fire && missileLive) {
    sim.explode(missileP, 5000.0, 20.0);
    missileLive = false;
  } else if (in.fire && reload <= 0 && ammo != 0 && mod[TM_GUN] > 0) {
    if (ammo > 0) ammo--;
    reload = reloadTime * lerpd(1.5, 1.0, mod[TM_GUN]);
    recoilTime = 0;
    shotsFired += 1;
    Shell s;
    s.p = muzzle + gF * 0.2;
    // Without its sight the gunner lays it by eye, and it shows.
    V3 shot = gF;
    if (mod[TM_SIGHT] <= 0) {
      const double wobble = double(sim.stepCount % 977) * 0.0127;
      shot = norm(gF + (eL * std::sin(wobble * 7.0) + up * std::cos(wobble * 11.0)) * 0.012);
    }
    s.v = shot * sp_.shellSpeed + vCog;
    shells.push_back(s);
    sim.event(EV_MUZZLE, muzzle, gF, 1.f);
  }
  if (recoilTime < 0.09) {
    // The recoil system stretches the blow of about 18 kN s out over 90 milliseconds.
    addToHull(sim, trun, gF * -sp_.recoilForce);
  }
}

void Tank::updateDrive(Sim& sim, const Controls& in, double dt) {
  const double absV = std::fabs(speed);
  throttleAct += clampd(in.throttle - throttleAct, -3.0 * dt, 2.0 * dt);
  brakeAct += clampd(in.brake - brakeAct, -6.0 * dt, 6.0 * dt);
  steerAct += clampd(in.steer - steerAct, -3.0 * dt, 3.0 * dt);

  if (in.requestGear == -1 && absV < 1.0) gear = -1;
  if (in.requestGear == 1 && absV < 1.0) gear = 1;
  shiftClock += dt;

  const double avgTrack = 0.5 * (trackOmega[0] + trackOmega[1]);
  const double ratio = gear < 0 ? -kRatios[0] : kRatios[std::clamp(gear, 1, 4)];
  const double inputOmega = std::fabs(avgTrack * ratio);
  if (gear > 0 && shiftClock > 1.2) {
    const double rpmIn = inputOmega * 60.0 / (2.0 * PI);
    if (rpmIn > 1780 && gear < 4) {
      gear++;
      shiftClock = 0;
    } else if (rpmIn < 1000 && gear > 1) {
      gear--;
      shiftClock = 0;
    }
  }

  // Engine and torque converter. The converter multiplies torque while it slips, and is what lets
  // 42 tonnes pull away smoothly.
  const double rpmNow = rpm();
  constexpr double kIdle = 65.0;  // rad/s, 620 rpm
  const double gov = clampd((2360.0 - rpmNow) / 110.0, 0, 1);
  const double th = std::max(throttleAct * gov, clampd((kIdle - engineOmega) / 12.0, 0, 0.5));
  loadNow += (th - loadNow) * clampd(dt / 0.25, 0, 1);
  const double slip = engineOmega - inputOmega;
  // A fluid coupling's square law, with the lock-up that real converters use at speed. Without it a third of
  // the power goes into heating the oil, and the engine sits on its governor at any road speed.
  const double speedRatioNow = engineOmega > 1.0 ? clampd(inputOmega / engineOmega, 0, 1) : 0;
  const double lockUp = clampd((speedRatioNow - 0.40) / 0.30, 0, 1);
  double Tconv = clampd(0.085 * slip * std::fabs(slip) + 180.0 * lockUp * slip, -5000.0, 5000.0);
  // Standing with the foot off: the box holds still rather than creeping off.
  if (throttleAct < 0.03 && absV < 0.7) Tconv = 0;
  const double speedRatio = engineOmega > 1.0 ? clampd(inputOmega / engineOmega, 0, 1) : 0;
  const double multiply = lerpd(2.2, 1.0, std::min(speedRatio / 0.85, 1.0));
  // What is left of the engine, and a heavy turret takes its share of what it can do.
  // A holed radiator lets it run hot in a minute or so; a cut fuel line empties the tanks in a few.
  radiatorHeat = std::min(radiatorHeat + (1.0 - mod[TM_RADIATOR]) * dt / 45.0, 1.0);
  fuel = std::max(0.0, fuel - (1.0 - mod[TM_FUEL]) * 7.0 * dt);
  const double power = (fuel <= 0 ? 0.0 : 1.0) * clampd(1.0 - radiatorHeat * 0.75, 0.25, 1.0) * spec().powerScale * hullSpec().powerScale * (mod[TM_ENGINE] <= 0 ? 0.0 : lerpd(0.35, 1.0, mod[TM_ENGINE]));
  const double Te = loadNow * engineTorque(rpmNow) * power - 80.0 - engineOmega * 0.8;
  engineOmega += (Te - Tconv) * dt / 7.0;
  engineOmega = clampd(engineOmega, 0.0, 270.0);

  // Two tracks through a steering differential: drive shared equally, and steering power moving torque
  // from one track to the other. It works at a standstill too, which is what makes a pivot turn.
  const double Tdrive = Tconv * multiply * ratio * 0.88;
  const double B = 2.7;  // distance between the tracks
  // As tight as the grip allows at speed, about 0.8 g; a tank has no steering lock to hold it back.
  const double yawWant = steerAct * (absV < 5.0 ? kPivotYawRate : clampd(7.5 / absV, 0.32, kPivotYawRate));
  const double dWant = yawWant * B / kSprocketR;  // right minus left track speed
  const double dNow = trackOmega[1] - trackOmega[0];
  const double Tsteer = clampd((dWant - dNow) * 60000.0, -520000.0, 520000.0);
  const double Tside[2] = {0.5 * Tdrive - 0.5 * Tsteer, 0.5 * Tdrive + 0.5 * Tsteer};

  // Track forces, then the tracks themselves.
  double Fsum[2] = {};
  for (const RoadWheel& w : wheels) {
    const int s = w.left ? 0 : 1;
    const V3 C = (sim.pos[w.in] + sim.pos[w.out]) * 0.5;
    const V3 vC = (sim.vel[w.in] + sim.vel[w.out]) * 0.5;
    const_cast<RoadWheel&>(w).Fn = 0;
    // The deepest of several points around the bottom of the wheel: the track rides up onto a kerb.
    V3 down = up * -1.0, ahead = fwd;
    double best = 0;
    V3 nBest{};
    u8 surf = SURF_ASPHALT;
    static const double kC[7] = {1.0, 0.92, 0.92, 0.64, 0.64, 0.26, 0.26};
    static const double kS[7] = {0.0, 0.39, -0.39, 0.77, -0.77, 0.97, -0.97};
    for (int j = 0; j < 7; j++) {
      const V3 p = C + down * (kC[j] * kWheelR) + ahead * (kS[j] * kWheelR);
      Contact c;
      if (sim.world->contact(p, c) && c.depth > best) {
        best = c.depth;
        nBest = c.n;
        surf = c.surface;
      }
    }
    if (best <= 0) continue;
    const double Fn = std::max(0.0, 3.0e6 * best - 22000.0 * dot(vC, nBest));
    const_cast<RoadWheel&>(w).Fn = Fn;
    const_cast<RoadWheel&>(w).soft = surf == SURF_GRASS || surf == SURF_GRAVEL ? 1.0 : 0.0;
    V3 t = fwd - nBest * dot(fwd, nBest);
    t = norm(t);
    const V3 l = cross(nBest, t);
    // Rubber pads and a long footprint: a track holds where a tyre would slide, most of all on loose ground.
    const double mu = 1.30 * (surf == SURF_ASPHALT || surf == SURF_CONCRETE ? surfaceFriction(surf) : 0.5 * (1.0 + surfaceFriction(surf)));
    const double Vt = trackOmega[s] * kSprocketR;
    const double slipX = Vt - dot(vC, t);
    // One grip for both directions: a track slipping along its length has none left to hold it sideways, which
    // is what lets a tank turn at all. Taken separately, the scrub of 14 wheels would simply hold it straight.
    const double sx = slipX / 0.30, sy = dot(vC, l) / 0.28;
    const double sn = std::max(1.0, std::hypot(sx, sy));
    // Without its track a wheel only rolls: no drive, no brake, and much less to hold it sideways.
    const double Fx = trackOff[s] ? -0.12 * Fn * clampd(dot(vC, t) / 0.3, -1, 1) : mu * Fn * sx / sn;
    const double Fy = (trackOff[s] ? -0.45 : -0.85) * mu * Fn * sy / sn;
    const V3 F = nBest * Fn + t * Fx + l * Fy;
    sim.force[w.in] += F * 0.5;
    sim.force[w.out] += F * 0.5;
    if (!trackOff[s]) Fsum[s] += Fx;
  }
  constexpr double kTrackInertia = 900.0;
  for (int s = 0; s < 2; s++) {
    if (trackOff[s]) {
      trackOmega[s] = 0;
      continue;
    }
    double om = trackOmega[s] + (Tside[s] - Fsum[s] * kSprocketR) * dt / kTrackInertia;
    // Rolling resistance of the track, and the brake bands. They can stop a track, not reverse it.
    double Fn = 0;
    for (const RoadWheel& w : wheels)
      if ((w.left ? 0 : 1) == s) Fn += w.Fn;
    const double Tres = (0.022 * Fn * kSprocketR + 120.0 + brakeAct * 90000.0 + in.handbrake * 90000.0);
    const double dv = Tres * dt / kTrackInertia;
    om = std::fabs(om) <= dv ? 0.0 : om - std::copysign(dv, om);
    trackOmega[s] = clampd(om, -60.0, 60.0);
    trackPhase[s] += trackOmega[s] * kSprocketR * dt;
  }
}

void Tank::updateShells(Sim& sim, double dt) {
  for (size_t i = 0; i < shells.size();) {
    Shell& s = shells[i];
    const V3 a = s.p;
    s.v.z -= 9.81 * dt;
    s.p += s.v * dt;
    s.age += dt;
    bool hit = false;
    V3 at = s.p;
    Contact c;
    if (sim.world->contact(s.p, c)) {
      hit = true;
    } else {
      hit = pathHit(sim, a, s.p, true, at);
    }
    if (hit) {
      // The 105 mm: a high-explosive round, about the 2.3 kg of filler of a HESH shell.
      sim.explode(at, spec().blastImpulse, spec().blastRadius);
      shells[i] = shells.back();
      shells.pop_back();
      continue;
    }
    if (s.age > spec().shellLife) {
      shells[i] = shells.back();
      shells.pop_back();
      continue;
    }
    if ((sim.stepCount & 7) == 0) sim.event(EV_TRACER, s.p, s.v, 1.f);
    i++;
  }
}

// Anything within half a metre of the way from a to b; the tank's own nodes only if it asks for them. Hull and
// turret lie side by side in the arrays, and whatever else is there can be hit whether built before or after.
bool Tank::pathHit(const Sim& sim, V3 a, V3 b, bool skipOwn, V3& at) const {
  const V3 d = b - a;
  const double dl2 = std::max(len2(d), 1e-9);
  const u32 ownFirst = sim.objs[hullObj].nodeStart;
  const u32 ownEnd = sim.objs[turretObj].nodeStart + sim.objs[turretObj].nodeCount;
  // Bounds first: they may be a moment old, so with something to spare. Most of the world is nowhere near.
  constexpr double kSpare = 2.5;
  const V3 lo{std::min(a.x, b.x) - kSpare, std::min(a.y, b.y) - kSpare, std::min(a.z, b.z) - kSpare};
  const V3 hi{std::max(a.x, b.x) + kSpare, std::max(a.y, b.y) + kSpare, std::max(a.z, b.z) + kSpare};
  for (const Obj& o : sim.objs) {
    if (o.bbMin.x > hi.x || o.bbMax.x < lo.x || o.bbMin.y > hi.y || o.bbMax.y < lo.y || o.bbMin.z > hi.z || o.bbMax.z < lo.z) continue;
    for (u32 k : o.nodes) {
      if (skipOwn && k >= ownFirst && k < ownEnd) continue;
      const V3 r = sim.pos[k] - a;
      const double t = clampd(dot(r, d) / dl2, 0, 1);
      if (len2(r - d * t) < 0.25) {
        at = a + d * t;
        return true;
      }
    }
  }
  return false;
}

void Tank::updateMissile(Sim& sim, const Controls& in, double dt) {
  if (in.launch && !missileLive && missiles > 0) {
    missiles--;
    missileLive = true;
    missileAge = 0;
    const double cy = std::cos(turretYaw), sy = std::sin(turretYaw);
    const V3 eF = fwd * cy + left * sy;
    missileP = ring_ + up * 1.4;
    missileV = eF * 25.0 + up * 18.0 + vCog;
    sim.event(EV_MUZZLE, missileP, up, 0.6f);
  }
  if (!missileLive) return;
  missileAge += dt;
  // It climbs clear of the launcher first; after that it goes where it is told, as fast as its fins will turn it.
  constexpr double kCruise = 95.0, kAccel = 40.0, kTurn = 1.15;
  const double sp = std::max(len(missileV), 1.0);
  V3 dir = missileV / sp;
  if (missileAge > 0.8 && len2(in.aimPoint) > 1.0) {
    const V3 want = norm(in.aimPoint - missileP);
    const double cosA = clampd(dot(dir, want), -1, 1);
    const double angle = std::acos(cosA);
    if (angle > 1e-5) dir = norm(dir + (want - dir * cosA) * (std::min(kTurn * dt, angle) / std::max(std::sin(angle), 1e-3)));
  }
  const double newSpeed = std::min(sp + kAccel * dt, kCruise);
  missileV = dir * newSpeed;
  const V3 from = missileP;
  missileP += missileV * dt;
  if ((sim.stepCount & 7) == 0) sim.event(EV_TRACER, missileP, missileV, 1.f);
  bool hit = false;
  V3 at = missileP;
  Contact c;
  if (sim.world->contact(missileP, c)) hit = true;
  else hit = pathHit(sim, from, missileP, missileAge < 1.5, at);
  // Enough fuel for the whole map and back; then it comes down wherever it is.
  if (hit || missileAge > 60.0) {
    sim.explode(at, 5000.0, 20.0);
    missileLive = false;
  }
}

bool Tank::bulletHit(Sim& sim, V3 a, V3 b, double force) {
  if (destroyed) return false;
  const V3 O = (sim.pos[refFL] + sim.pos[refFR] + sim.pos[refRL] + sim.pos[refRR]) * 0.25;
  const double cy = std::cos(turretYaw), sy = std::sin(turretYaw);
  const V3 eF = fwd * cy + left * sy, eL = left * cy - fwd * sy;
  // On the turret, and on the hull. Ahead is towards the nose; the reference point lies about 3 m back from it.
  const auto onTurret = [&](double f, double l, double z) { return ring_ + eF * f + eL * l + up * z; };
  const auto onHull = [&](double x, double l, double z) { return O + fwd * (3.0 - x) + left * l + up * z; };
  struct Spot {
    int module;
    V3 at;
    double radius, toughness;
  };
  const Spot spots[] = {
      {TM_SIGHT, onTurret(0.32, -0.46, 0.92), 0.2, 1.0},       // the gunner's sight head, on the turret roof
      {TM_RANGE, onTurret(-0.05, 1.44, 0.54), 0.14, 1.0},      // the rangefinder's ends, sticking out of both sides
      {TM_RANGE, onTurret(-0.05, -1.44, 0.54), 0.14, 1.0},
      {TM_PERISCOPE, onHull(1.05, 0.48, -0.12), 0.2, 1.0},     // the driver's periscopes on the glacis
      {TM_RADIATOR, onHull(5.55, 0.62, 0.03), 0.55, 2.0},      // the cooling grilles on the engine deck
      {TM_RADIATOR, onHull(5.55, -0.62, 0.03), 0.55, 2.0},
      {TM_FUEL, onHull(6.9, 0.95, -0.3), 0.2, 1.0},            // the fuel line low on the rear plate
      {TM_FUEL, onHull(6.9, -0.95, -0.3), 0.2, 1.0},
      {TM_RING, onTurret(1.28, 0, 0.02), 0.22, 2.0},           // the gap under the turret, front, sides and back
      {TM_RING, onTurret(0, 1.3, 0.02), 0.22, 2.0},
      {TM_RING, onTurret(0, -1.3, 0.02), 0.22, 2.0},
      {TM_RING, onTurret(-1.3, 0, 0.02), 0.22, 2.0},
      {TM_ELEVATION, onTurret(1.05, 0, 0.45), 0.25, 2.0},      // the gear at the root of the barrel
      {TM_ANTENNA, onTurret(-0.9, 0.25, 1.5), 0.14, 1.0},      // the radio aerials
      {TM_ANTENNA, onTurret(-0.9, -0.25, 1.5), 0.14, 1.0},
      {TM_TRACK_L, onHull(1.3, 1.35, -0.75), 0.24, 3.0},       // track pins: it takes a few of them
      {TM_TRACK_L, onHull(3.5, 1.35, -0.75), 0.24, 3.0},
      {TM_TRACK_L, onHull(5.6, 1.35, -0.75), 0.24, 3.0},
      {TM_TRACK_R, onHull(1.3, -1.35, -0.75), 0.24, 3.0},
      {TM_TRACK_R, onHull(3.5, -1.35, -0.75), 0.24, 3.0},
      {TM_TRACK_R, onHull(5.6, -1.35, -0.75), 0.24, 3.0},
  };
  const V3 d = b - a;
  const double dl2 = std::max(len2(d), 1e-9);
  for (const Spot& s : spots) {
    const double t = clampd(dot(s.at - a, d) / dl2, 0, 1);
    if (len2(a + d * t - s.at) > s.radius * s.radius) continue;
    mod[s.module] = std::max(0.0, mod[s.module] - force / s.toughness);
    sim.event(EV_SPARK, s.at, up, 0.5f);
    if (mod[TM_TRACK_L] <= 0) throwTrack(sim, 0);
    if (mod[TM_TRACK_R] <= 0) throwTrack(sim, 1);
    return true;
  }
  return false;
}

void Tank::makeWreck(Sim& sim) {
  destroyed = true;
  burnTime = 1000.0;  // long burnt out
  ammo = 0;
  throwTrack(sim, 0);
  throwTrack(sim, 1);
  damage = 1.0;
  // Rusted solid: it stands where it stands and stops what runs into it. Turret on hull with nothing holding it would
  // otherwise be two bodies in one place, and blow themselves apart the moment anything woke them.
  for (u16 oi : {hullObj, turretObj}) {
    Obj& o = sim.objs[oi];
    o.armoured = true;
    o.kind = OBJ_PROP;
    o.alwaysAwake = false;
    o.awake = false;
    for (u32 n : o.nodes) {
      sim.flags[n] |= NF_FIXED;
      sim.invMass[n] = 0;
      sim.vel[n] = {};
    }
  }
}

void Tank::repair(Sim& sim) {
  for (double& m : mod) m = std::max(m, 0.6);
  radiatorHeat = 0;
  trackOff[0] = trackOff[1] = false;
  damage = std::max(0.0, damage - 0.15);
  sim.event(EV_SPARK, cog + up * 1.5, up, 0.5f);
}

void Tank::throwTrack(Sim& sim, int side) {
  if (trackOff[side]) return;
  // It runs off the wheels and stays lying where this happened, whatever the tank then does.
  std::vector<V3> path;
  trackPath(sim, side, path);
  double mid = 0;
  for (const V3& p : path) mid += p.z;
  mid /= std::max<size_t>(path.size(), 1);
  for (V3& p : path) {
    const bool upper = p.z > mid;
    p.z = sim.world->height(p.x, p.y) + (upper ? 0.09 : 0.035);
  }
  thrown_[side] = std::move(path);
  thrownLeft_[side] = left;
  trackOff[side] = true;
  trackOmega[side] = 0;
  sim.event(EV_ROCK, thrown_[side][thrown_[side].size() / 2], V3{}, 0.3f);
}

void Tank::takeHit(Sim& sim, double hit, V3 at) {
  damage += hit;
  // The hull gives where it was hit: what is drawn over it follows.
  const double scale = std::min(hit / 0.5, 1.0);
  for (u16 oi : {hullObj, turretObj})
    for (u32 bi : sim.objs[oi].beamList) {
      Beam& b = sim.beams[bi];
      if (b.flags & BF_BROKEN) continue;
      const double d = len((sim.pos[b.a] + sim.pos[b.b]) * 0.5 - at);
      if (d < 2.2) b.L0 *= 1.0 - 0.07 * scale * (1.0 - d / 2.2);
    }
  if (damage >= 1.0) destroy(sim);
}

// Every part has its place on the tank and suffers by how close the blast was to it.
void Tank::damageModules(Sim& sim, V3 c, double impulse) {
  const V3 O = (sim.pos[refFL] + sim.pos[refFR] + sim.pos[refRL] + sim.pos[refRR]) * 0.25;
  const auto toSegment = [&](V3 a, V3 b) {
    const V3 d = b - a;
    const double t = clampd(dot(c - a, d) / std::max(len2(d), 1e-9), 0, 1);
    return len(a + d * t - c);
  };
  const V3 trackL = O + left * 1.35 - up * 0.75, trackR = O - left * 1.35 - up * 0.75;
  const double dist[TM_COUNT] = {
      len(O - fwd * 2.6 - up * 0.25 - c),
      len(ring_ + up * 0.05 - c),
      toSegment(ring_ + up * 0.56, muzzle),
      len(ring_ - up * 0.35 - c),
      toSegment(trackL + fwd * 2.7, trackL - fwd * 3.5),
      toSegment(trackR + fwd * 2.7, trackR - fwd * 3.5),
  };
  static constexpr double kReach[TM_COUNT] = {1.8, 1.7, 0.9, 1.2, 1.3, 1.3};
  for (int i = 0; i < TM_COUNT; i++) {
    if (dist[i] >= kReach[i]) continue;
    mod[i] = std::max(0.0, mod[i] - (impulse / 2200.0) * 1.1 * (1.0 - dist[i] / kReach[i]));
  }
  if (mod[TM_TRACK_L] <= 0) throwTrack(sim, 0);
  if (mod[TM_TRACK_R] <= 0) throwTrack(sim, 1);
  if (mod[TM_AMMO] <= 0) destroy(sim);
}

void Tank::checkDamage(Sim& sim, double dt) {
  // A blast: what counts is how close it got.
  if (sim.blastCount != blastSeen_) {
    blastSeen_ = sim.blastCount;
    const V3 c = sim.lastBlast;
    double dmin = 1e9;
    for (u16 oi : {hullObj, turretObj})
      for (u32 n : sim.objs[oi].nodes) dmin = std::min(dmin, len(sim.pos[n] - c));
    if (sim.lastBlastRadius > 0 && dmin < sim.lastBlastRadius) {
      // A heavy shell does not need to touch: anywhere close enough, and that is the end of the tank.
      const double core = sim.lastBlastImpulse >= 10000.0 ? 4.5 : 1.3;
      const double reach = std::min(core / std::max(dmin, 0.3), 1.0);
      // The armour takes several: five direct hits from the 105 mm. What gets hit on the outside is another matter.
      const double hit = 0.20 * hullSpec().armour * (sim.lastBlastImpulse / 2200.0) * reach * reach;
      if (reach > 0.2) damageModules(sim, c, sim.lastBlastImpulse * hullSpec().armour);
      if (destroyed) return;
      if (hit > 0.003) takeHit(sim, hit, c);
    }
  }
  // A crash: a sudden change in speed, against what the hull was doing a moment ago.
  crashHold_ -= dt;
  const double jump = len(vCog - vSlow_);
  vSlow_ += (vCog - vSlow_) * clampd(dt / 0.12, 0, 1);
  if (jump > 7.0 && crashHold_ <= 0) {
    crashHold_ = 0.4;
    const double over = (jump - 7.0) / 13.0;
    const V3 at = cog + norm(vSlow_ - vCog) * -3.0;
    takeHit(sim, over * over * 0.8, at);
    if (jump > 13.0) {
      throwTrack(sim, 0);
      throwTrack(sim, 1);
    }
  }
}

void Tank::destroy(Sim& sim) {
  if (destroyed) return;
  destroyed = true;
  burnTime = 0;
  engineOmega = 0;
  loadNow = throttleAct = 0;
  missileLive = false;
  throwTrack(sim, 0);
  throwTrack(sim, 1);
  const V3 O = (sim.pos[refFL] + sim.pos[refFR] + sim.pos[refRL] + sim.pos[refRR]) * 0.25;
  for (u16 oi : {hullObj, turretObj}) {
    Obj& o = sim.objs[oi];
    o.armoured = false;
    o.kind = OBJ_PROP;
    o.alwaysAwake = false;
    for (u32 bi : o.beamList) {
      Beam& b = sim.beams[bi];
      b.yC *= 0.45;
      b.yT *= 0.45;
      if (b.fBreak > 0) b.fBreak *= 0.5;
    }
  }
  // Without the track forces the wheels have to find the ground themselves.
  for (const RoadWheel& w : wheels) {
    sim.flags[w.in] |= NF_COLLIDE;
    sim.flags[w.out] |= NF_COLLIDE;
  }
  // The same for the turret: until now it was held, from here on it has to land.
  for (u32 n : sim.objs[turretObj].nodes) sim.flags[n] |= NF_COLLIDE;
  // The ammunition in the hull goes up, and the turret goes with it: nothing holds it but its own weight.
  const double a = double(sim.stepCount % 628) * 0.01;
  const V3 kick = up * 11.0 + V3{std::cos(a), std::sin(a), 0} * 2.5;
  const V3 spin = norm(V3{std::sin(a * 3.0), std::cos(a * 2.0), 0.4}) * 1.1;
  V3 tc{};
  for (u32 n : turretNodes) tc += sim.pos[n];
  tc = tc / std::max<size_t>(turretNodes.size(), 1);
  for (u32 n : sim.objs[turretObj].nodes) sim.vel[n] += kick + cross(spin, sim.pos[n] - tc);
  sim.explode(O + up * 0.2, 3500.0, 9.0);
  blastSeen_ = sim.blastCount;
}

void Tank::tickFire(Sim& sim, double dt) {
  if (destroyed) {
    burnTime += dt;
    // Once down, a turret on its edge would otherwise roll away like a coin: it is nine tonnes of steel on tarmac.
    if (burnTime > 1.0 && sim.objs[turretObj].awake)
      for (u32 n : sim.objs[turretObj].nodes)
        if (sim.pos[n].z < sim.world->height(sim.pos[n].x, sim.pos[n].y) + 2.5) sim.force[n] -= sim.vel[n] * (sim.mass[n] * 2.0);
    if ((sim.stepCount & 31) == 0 && burnTime < 75.0) {
      const double fierce = clampd(1.0 - burnTime / 75.0, 0.2, 1.0);
      sim.event(EV_FIRE, cog + fwd * ((sim.stepCount % 7) * 0.5 - 1.5) + up * 0.1, V3{}, float(fierce));
    }
  } else if (damage > 0.5 && (sim.stepCount & 127) == 0) {
    // Something inside it has started to smoulder.
    sim.event(EV_FIRE, cog + fwd * -1.5 + up * 0.6, V3{}, 0.15f);
  }
}

void Tank::update(Sim& sim, const Controls& in, double dt) {
  updateFrame(sim, dt);
  tickFire(sim, dt);
  if (destroyed) {
    updateShells(sim, dt);
    updateMissile(sim, in, dt);
    return;
  }
  checkDamage(sim, dt);
  if (destroyed) return;
  dampHull(sim, dt);
  updateWheelLift(sim, dt);
  updateTurret(sim, in, dt);
  updateDrive(sim, in, dt);
  updateShells(sim, dt);
  updateMissile(sim, in, dt);
}

void Tank::updateWheelLift(const Sim& sim, double dt) {
  // The reference nodes do not lie level on the hull: this is what was straight up when it was built.
  const V3 restFwd = norm(sim.rest[refFL] + sim.rest[refFR] - sim.rest[refRL] - sim.rest[refRR]);
  const V3 restUp = norm(cross(restFwd, sim.rest[refFL] + sim.rest[refRL] - sim.rest[refFR] - sim.rest[refRR]));
  const V3 levelUp = fwd * restFwd.z + up * restUp.z;
  // What the track follows: wheel height against the hull, within the travel it really has.
  for (size_t i = 0; i < wheels.size() && i < 14; i++) {
    const V3 hub = (sim.pos[wheels[i].in] + sim.pos[wheels[i].out]) * 0.5;
    const double lift = clampd(dot(hub - sim.pos[idler[wheels[i].left ? 0 : 1][0]], levelUp) - (kWheelR - kIdlerZ), -0.22, 0.30);
    wheelLift_[i] += (lift - wheelLift_[i]) * clampd(dt / 0.02, 0, 1);
  }
}

Tank::NetState Tank::netState() const {
  NetState n;
  n.vel = vCog;
  n.muzzle = muzzle;
  n.gunDir = gunDir;
  n.sight = sight;
  n.turretYaw = float(turretYaw);
  n.gunPitch = float(gunPitch);
  n.reload = float(reload);
  n.reloadTime = float(reloadTime);
  n.damage = float(damage);
  n.burnTime = float(burnTime);
  for (int i = 0; i < 2; i++) {
    n.trackPhase[i] = float(trackPhase[i]);
    n.trackOmega[i] = float(trackOmega[i]);
    n.trackOff[i] = trackOff[i];
  }
  n.engineOmega = float(engineOmega);
  n.throttle = float(throttleAct);
  n.brake = float(brakeAct);
  n.steer = float(steerAct);
  n.speed = float(speed);
  n.latAcc = float(latAcc);
  n.lonAcc = float(lonAcc);
  n.fuel = float(fuel);
  n.load = float(loadNow);
  n.gear = gear;
  n.ammo = ammo;
  n.destroyed = destroyed;
  for (int i = 0; i < TM_COUNT; i++) n.mod[i] = float(mod[i]);
  n.missiles = missiles;
  n.smokeCharges = smokeCharges;
  n.netOn = netOn;
  n.netClock = float(netClock);
  n.missileLive = missileLive;
  n.missileP = missileP;
  n.missileV = missileV;
  return n;
}

void Tank::applyNetState(Sim& sim, const NetState& n, double dt) {
  updateFrame(sim, dt);
  updateWheelLift(sim, dt);
  vCog = n.vel;
  muzzle = n.muzzle;
  gunDir = n.gunDir;
  sight = n.sight;
  turretYaw = n.turretYaw;
  gunPitch = n.gunPitch;
  reload = n.reload;
  reloadTime = n.reloadTime;
  damage = n.damage;
  burnTime = n.burnTime;
  for (int i = 0; i < 2; i++) {
    trackPhase[i] = n.trackPhase[i];
    trackOmega[i] = n.trackOmega[i];
    // Either it just came off, and this frame sees where it lies; or a new match has put it back.
    if (n.trackOff[i]) throwTrack(sim, i);
    else trackOff[i] = false;
  }
  engineOmega = n.engineOmega;
  throttleAct = n.throttle;
  brakeAct = n.brake;
  steerAct = n.steer;
  speed = n.speed;
  latAcc = n.latAcc;
  lonAcc = n.lonAcc;
  fuel = n.fuel;
  loadNow = n.load;
  gear = n.gear;
  ammo = n.ammo;
  destroyed = n.destroyed;
  for (int i = 0; i < TM_COUNT; i++) mod[i] = n.mod[i];
  missiles = n.missiles;
  smokeCharges = n.smokeCharges;
  netOn = n.netOn;
  netClock = n.netClock;
  missileLive = n.missileLive;
  missileP = n.missileP;
  missileV = n.missileV;
}

void Tank::shiftIndices(long dn, long, long dobj) {
  const auto N = [&](u32& i) { i = u32(long(i) + dn); };
  hullObj = u16(long(hullObj) + dobj);
  turretObj = u16(long(turretObj) + dobj);
  for (RoadWheel& w : wheels) {
    N(w.in);
    N(w.out);
  }
  for (auto* list : {&rigidNodes_, &turretNodes, &gunNodes})
    for (u32& n : *list) N(n);
  N(refFL);
  N(refFR);
  N(refRL);
  N(refRR);
  N(refBottom);
  for (auto& a : sprocket)
    for (u32& n : a) N(n);
  for (auto& a : idler)
    for (u32& n : a) N(n);
}

void Tank::settle(Sim& sim, double seconds) {
  Controls c;
  const int steps = int(seconds * 2000.0);
  for (int i = 0; i < steps; i++) {
    sim.forces(1.0 / 2000.0);
    update(sim, c, 1.0 / 2000.0);
    sim.finish(1.0 / 2000.0);
  }
  sim.events.clear();
}

void Tank::trackPath(const Sim& sim, int side, std::vector<V3>& out) const {
  if (trackOff[side]) {
    out = thrown_[side];
    return;
  }
  out.clear();
  // Everything in the hull's own frame and at its fixed place along the hull, with only the wheels'
  // smoothed height free to change. So the track keeps its length and shape whatever the hull goes
  // through, as a real one under tension does, and it still follows every bump.
  const V3 O = (sim.pos[refFL] + sim.pos[refFR] + sim.pos[refRL] + sim.pos[refRR]) * 0.25;
  const V3 O0 = (sim.rest[refFL] + sim.rest[refFR] + sim.rest[refRL] + sim.rest[refRR]) * 0.25;
  const V3 restFwd = norm(sim.rest[refFL] + sim.rest[refFR] - sim.rest[refRL] - sim.rest[refRR]);
  const V3 restUp = norm(cross(restFwd, sim.rest[refFL] + sim.rest[refRL] - sim.rest[refFR] - sim.rest[refRR]));
  const V3 restLeft = cross(restUp, restFwd);
  const double groundZ = sim.rest[idler[side][0]].z - kIdlerZ;
  const double y = (side == 0 ? 1.0 : -1.0) * kTrackY;
  const double noseWorldX = sim.rest[idler[side][0]].x + kIdlerX;  // where design x = 0 was built
  auto P = [&](double x, double z) {
    // Where it was built, seen from the reference nodes, which do not lie level on the hull.
    const V3 d{(noseWorldX - x) - O0.x, y, z + groundZ - O0.z};
    return O + fwd * dot(d, restFwd) + left * dot(d, restLeft) + up * dot(d, restUp);
  };
  auto arc = [&](double cx, double cz, double r, double a0, double a1, int n) {
    // Angle measured towards the front of the tank, which is decreasing design x.
    for (int i = 0; i <= n; i++) {
      const double a = lerpd(a0, a1, double(i) / n);
      out.push_back(P(cx - std::cos(a) * r, cz + std::sin(a) * r));
    }
  };
  // From the top of the sprocket, forward along the top run, over the idler and back along the ground.
  arc(kSprocketX, kSprocketZ, kSprocketR, 2.0, 1.57, 2);
  for (int i = 6; i >= 0; i -= 2) out.push_back(P(kWheelX[i], kTopRunZ - 0.012 * (i % 4 == 0 ? 1.0 : 0.0)));
  arc(kIdlerX, kIdlerZ, kIdlerR, 1.57, -1.57, 6);
  for (int i = 0; i < 7; i++) {
    const double z = wheelLift_[size_t(side) * 7 + size_t(i)];
    out.push_back(P(kWheelX[i] - 0.16, z));
    out.push_back(P(kWheelX[i] + 0.16, z));
  }
  arc(kSprocketX, kSprocketZ, kSprocketR, -1.57, -4.28, 6);
}
