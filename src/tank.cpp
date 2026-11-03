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

Tank buildLeopard(Sim& sim, V3 spawn, CarRig* rig) {
  Tank T;
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
        ids[s][r * 2 + d] = sim.addNode(toWorld(spawn, st[s].x, y, zs[r]), st[s].mass, NF_COLLIDE, 0.5f);
        hull.push_back(ids[s][r * 2 + d]);
      }
  }
  for (int s = 0; s < 6; s++) {
    for (int a = 0; a < 6; a++)
      for (int b = a + 1; b < 6; b++) sim.addBeam(ids[s][a], ids[s][b], mArmour);
    if (s < 5)
      for (int a = 0; a < 6; a++)
        for (int b = 0; b < 6; b++) sim.addBeam(ids[s][a], ids[s + 1][b], mArmour);
    if (s < 4)
      for (int a = 0; a < 6; a++) sim.addBeam(ids[s][a], ids[s + 2][a], mArmour);
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
      const Mat spring{2.0e5, end ? 9000.0 : 3000.0, 0, 0, 0, 0, 0};
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
    if (sim.mass[i] < 200.0 && sim.mass[i] > 60.0) T.rigidNodes_.push_back(i);
  std::sort(T.rigidNodes_.begin(), T.rigidNodes_.end());
  T.rigidNodes_.erase(std::unique(T.rigidNodes_.begin(), T.rigidNodes_.end()), T.rigidNodes_.end());

  // ---- Turret and gun: a body of their own, held to the hull by the turret ring.
  T.turretObj = sim.beginObj(OBJ_CAR, true);
  sim.objs[T.turretObj].armoured = true;
  const double tcx = 2.90, tcz = 1.34;  // the ring's centre, design coordinates
  T.turretAxis = V3{kNoseX - tcx, 0, tcz};
  auto addTurretNode = [&](double f, double l, double z, double m) {
    const u32 n = sim.addNode(spawn + T.turretAxis + V3{f, l, z}, m, NF_COLLIDE, 0.5f);
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
    for (size_t b = a + 1; b < tn.size(); b++) sim.addBeam(tn[a], tn[b], mCast);
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
    const u32 n = sim.addNode(spawn + T.turretAxis + T.trunnion + local, m, 0, 0.5f);
    T.gunNodes.push_back(n);
    T.gunRest.push_back(local);
    return n;
  };
  const u32 g0 = addGunNode(-1.1, 0.12, 0, 420);
  const u32 g1 = addGunNode(-1.1, -0.12, 0, 420);
  const u32 g2 = addGunNode(-0.8, 0, 0.16, 280);
  const u32 g3 = addGunNode(0.45, 0.10, 0, 400);
  const u32 g4 = addGunNode(0.45, -0.10, 0, 400);
  const u32 g5 = addGunNode(5.45, 0, 0, 180);
  const u32 gn[6] = {g0, g1, g2, g3, g4, g5};
  for (int a = 0; a < 6; a++)
    for (int b = a + 1; b < 6; b++) sim.addBeam(gn[a], gn[b], mGun);
  sim.endObj();

  double total = 0;
  for (u32 oi : {u32(T.hullObj), u32(T.turretObj)}) {
    const Obj& o = sim.objs[oi];
    for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) total += sim.mass[i];
  }
  printf("Leopard 1: %.1f t\n", total / 1000.0);

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
  const V3 tp0 = ring + tF * trunnion.x + tL * trunnion.y + up * trunnion.z;
  if (len2(in.aimPoint) > 1.0) {
    const V3 to = in.aimPoint - tp0;
    const double wantYaw = std::atan2(dot(to, left), dot(to, fwd));
    double e = wantYaw - turretYaw;
    while (e > PI) e -= 2 * PI;
    while (e < -PI) e += 2 * PI;
    constexpr double kYawRate = 40.0 * PI / 180.0, kPitchRate = 22.0 * PI / 180.0;
    turretYaw += clampd(e * 6.0, -kYawRate, kYawRate) * dt;
    if (turretYaw > PI) turretYaw -= 2 * PI;
    if (turretYaw < -PI) turretYaw += 2 * PI;
    const double horiz = dot(to, tF);
    const double wantPitch = clampd(std::atan2(dot(to, up), std::max(horiz, 1.0)), -9.0 * PI / 180.0, 20.0 * PI / 180.0);
    gunPitch += clampd((wantPitch - gunPitch) * 6.0, -kPitchRate, kPitchRate) * dt;
  }

  const double cy = std::cos(turretYaw), sy = std::sin(turretYaw);
  const V3 eF = fwd * cy + left * sy, eL = left * cy - fwd * sy;
  const double cp = std::cos(gunPitch), sp = std::sin(gunPitch);
  const V3 gF = eF * cp + up * sp, gU = up * cp - eF * sp;
  const V3 trun = ring + eF * trunnion.x + eL * trunnion.y + up * trunnion.z;
  muzzle = trun + gF * 5.45;
  gunDir = gF;
  sight = ring + eF * 0.25 + eL * 0.45 + up * 1.15;

  // ---- The gun runs back in its cradle and returns.
  recoilTime += dt;
  recoil = recoilTime < 0.05 ? 0.34 * recoilTime / 0.05 : 0.34 * std::max(0.0, 1.0 - (recoilTime - 0.05) / 0.9);

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
  if (in.fire && reload <= 0) {
    reload = reloadTime;
    recoilTime = 0;
    shotsFired += 1;
    Shell s;
    s.p = muzzle + gF * 0.2;
    s.v = gF * 1100.0 + vCog;
    shells.push_back(s);
    sim.event(EV_MUZZLE, muzzle, gF, 1.f);
  }
  if (recoilTime < 0.09) {
    // The recoil system stretches the blow of about 18 kN s out over 90 milliseconds.
    addToHull(sim, trun, gF * -200e3);
  }
}

void Tank::updateDrive(Sim& sim, const Controls& in, double dt) {
  const double absV = std::fabs(speed);
  throttleAct += clampd(in.throttle - throttleAct, -3.0 * dt, 2.0 * dt);
  brakeAct += clampd(in.brake - brakeAct, -6.0 * dt, 6.0 * dt);
  steerAct += clampd(in.steer - steerAct, -3.0 * dt, 3.0 * dt);

  if (in.requestGear == -1 && absV < 1.0) gear = -1;
  if (in.requestGear == 1 && absV < 1.0) gear = 1;
  if (in.toggleAuto) autoBox = !autoBox;
  if (in.shiftUp && gear > 0 && gear < 4) gear++;
  if (in.shiftDown && gear > 1) gear--;
  shiftClock += dt;

  const double avgTrack = 0.5 * (trackOmega[0] + trackOmega[1]);
  const double ratio = gear < 0 ? -kRatios[0] : kRatios[std::clamp(gear, 1, 4)];
  const double inputOmega = std::fabs(avgTrack * ratio);
  if (autoBox && gear > 0 && shiftClock > 1.2) {
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
  const double Te = loadNow * engineTorque(rpmNow) - 80.0 - engineOmega * 0.8;
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
    const double Fx = mu * Fn * sx / sn;
    const double Fy = -0.85 * mu * Fn * sy / sn;
    const V3 F = nBest * Fn + t * Fx + l * Fy;
    sim.force[w.in] += F * 0.5;
    sim.force[w.out] += F * 0.5;
    Fsum[s] += Fx;
  }
  constexpr double kTrackInertia = 900.0;
  for (int s = 0; s < 2; s++) {
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
      // Anything but the tank itself, within half a metre of this step's path.
      const V3 d = s.p - a;
      const double dl2 = std::max(len2(d), 1e-9);
      const u32 firstOther = sim.objs[turretObj].nodeStart + sim.objs[turretObj].nodeCount;
      const u32 n = u32(sim.pos.size());
      for (u32 k = firstOther; k < n; k++) {
        const V3 r = sim.pos[k] - a;
        const double t = clampd(dot(r, d) / dl2, 0, 1);
        const V3 q = r - d * t;
        if (len2(q) < 0.25) {
          hit = true;
          at = a + d * t;
          break;
        }
      }
    }
    if (hit) {
      // A high-explosive round, about the 2.3 kg of filler of a 105 mm HESH shell.
      sim.explode(at, 2200.0, 15.0);
      shells[i] = shells.back();
      shells.pop_back();
      continue;
    }
    if (s.age > 8.0) {
      shells[i] = shells.back();
      shells.pop_back();
      continue;
    }
    if ((sim.stepCount & 7) == 0) sim.event(EV_TRACER, s.p, s.v, 1.f);
    i++;
  }
}

void Tank::update(Sim& sim, const Controls& in, double dt) {
  updateFrame(sim, dt);
  dampHull(sim, dt);
  // What the track follows: wheel height against the hull, within the travel it really has.
  for (size_t i = 0; i < wheels.size() && i < 14; i++) {
    const V3 hub = (sim.pos[wheels[i].in] + sim.pos[wheels[i].out]) * 0.5;
    const double lift = clampd(dot(hub - sim.pos[idler[wheels[i].left ? 0 : 1][0]], up) - (kWheelR - kIdlerZ), -0.22, 0.30);
    wheelLift_[i] += (lift - wheelLift_[i]) * clampd(dt / 0.02, 0, 1);
  }
  updateTurret(sim, in, dt);
  updateDrive(sim, in, dt);
  updateShells(sim, dt);
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
  out.clear();
  // Everything in the hull's own frame and at its fixed place along the hull, with only the wheels'
  // smoothed height free to change. So the track keeps its length and shape whatever the hull goes
  // through, as a real one under tension does, and it still follows every bump.
  const V3 O = (sim.pos[refFL] + sim.pos[refFR] + sim.pos[refRL] + sim.pos[refRR]) * 0.25;
  const V3 O0 = (sim.rest[refFL] + sim.rest[refFR] + sim.rest[refRL] + sim.rest[refRR]) * 0.25;
  const double groundZ = sim.rest[idler[side][0]].z - kIdlerZ;
  const double y = (side == 0 ? 1.0 : -1.0) * kTrackY;
  const double noseWorldX = sim.rest[idler[side][0]].x + kIdlerX;  // where design x = 0 was built
  auto P = [&](double x, double z) {
    const double f = (noseWorldX - x) - O0.x;
    return O + fwd * f + left * y + up * (z + groundZ - O0.z);
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
