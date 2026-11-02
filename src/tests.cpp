// Headless test bench: drives the real simulation through scripted manoeuvres and prints the
// numbers, so the physics can be compared with the real car rather than judged by eye.
#include <chrono>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <array>
#include <functional>
#include <map>
#include <string>
#include <vector>
#include "props.h"
#include "vehicle.h"

namespace {
constexpr double kDt = 1.0 / 2000.0;

struct Bench {
  World world;
  Sim sim;
  Vehicle car;
  CarRig skin;

  explicit Bench(V3 pos = {-90, 0, 0}, double yaw = 0, bool props = false) {
    world.build();
    sim.world = &world;
    car = buildXantia(sim, pos, yaw, &skin);
    if (props) populateWorld(sim, world);
    sim.finalize();
    settleVehicle(sim, car, 4.0);
  }
  void run(double seconds, const std::function<void(double, Controls&)>& fn) {
    const int steps = int(seconds / kDt);
    Controls c;
    for (int i = 0; i < steps; i++) {
      const double t = i * kDt;
      c.shiftUp = c.shiftDown = c.toggleAuto = c.toggleSport = false;
      c.cycleHeight = c.toggleIgnition = c.toggleActiva = false;
      fn(t, c);
      sim.forces(kDt);
      car.update(sim, c, kDt);
      sim.finish(kDt);
    }
    sim.events.clear();
  }
  bool sane() const {
    const Obj& o = sim.objs[car.obj];
    for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++)
      if (!std::isfinite(sim.pos[i].x) || std::fabs(sim.pos[i].z) > 1000.0) return false;
    return true;
  }
  // Lays the car straight on the pad at a given speed, as if it had driven there.
  void launchAt(double v, int gear) {
    const Obj& o = sim.objs[car.obj];
    for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) sim.vel[i] = car.fwd * v;
    car.presetRolling(v, gear);
  }
};

void testStatics() {
  Bench b;
  const Vehicle& c = b.car;
  printf("== Statics ==\n");
  printf("  nodes %zu  beams %zu  tris %zu\n", b.sim.pos.size(), b.sim.beams.size(), b.sim.tris.size());
  {
    CarMesh mesh;
    if (mesh.load("assets/xantia.mesh")) {
      mesh.bind(b.sim.rest, b.skin.groups, {-90, 0, 0});
      std::string worst;
      const double err = mesh.restError(b.sim.rest, {-90, 0, 0}, &worst);
      printf("  mesh: %zu parts, %zu verts, %zu tris, max rest error %.2f mm (%s)\n", mesh.parts.size(), mesh.restPos.size() / 3, mesh.tris.size() / 3, err * 1000, worst.c_str());
    } else {
      printf("  mesh: assets/xantia.mesh NOT FOUND\n");
    }
  }
  u32 worst = 0;
  const double lam = b.sim.maxEigenvalue(b.car.obj, &worst);
  printf("  stability: largest eigenvalue %.3g, limit %.3g (%.0f %%), worst node %u mass %.2f design (%.2f %+.2f %.2f)\n", lam, 4.0 / (kDt * kDt), lam / (4.0 / (kDt * kDt)) * 100, worst, b.sim.mass[worst],
         2.26 - (b.sim.rest[worst].x + 90), b.sim.rest[worst].y, b.sim.rest[worst].z);
  printf("  velocity guard activations %llu\n", (unsigned long long)b.sim.clampedSteps);
  printf("  mass %.0f kg  cog z %.3f m\n", c.totalMass, c.cog.z);
  double fzF = c.wheels[0].Fz + c.wheels[1].Fz, fzR = c.wheels[2].Fz + c.wheels[3].Fz;
  printf("  axle load F %.0f N  R %.0f N (%.1f %% front)\n", fzF, fzR, 100.0 * fzF / (fzF + fzR));
  for (int a = 0; a < 2; a++)
    printf("  axle %d: strut L %.3f (nom %.3f) p %.1f bar  stroke %.3f\n", a, c.axle[a].s[0].L,
           c.axle[a].s[0].Lnom, c.axle[a].s[0].pGas / 1e5, c.axle[a].s[0].L - c.axle[a].s[0].Lmin);
  printf("  roll %.2f deg  pitch %.2f deg  plastic energy %.1f J  broken %u  sane %d\n",
         c.rollAngle * 57.3, c.pitchAngle * 57.3, b.sim.plasticEnergy, b.sim.brokenBeams, b.sane());
  printf("  lowest node z %.3f\n", [&] {
    double z = 1e9;
    const Obj& o = b.sim.objs[c.obj];
    for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) z = std::min(z, b.sim.pos[i].z);
    return z;
  }());
}

void testSpawn() {
  World world;
  world.build();
  Sim sim;
  sim.world = &world;
  CarRig skin;
  Vehicle car = buildXantia(sim, {-90, 0, 0}, 0, &skin);
  sim.finalize();
  car.settleFluid(sim);
  Controls c;
  double dt = kDt;
  printf("  spawn transient\n");
  for (int i = 0; i < 6000; i++) {
    sim.forces(dt);
    car.update(sim, c, dt);
    sim.finish(dt);
    if (i < 12 || i % 200 == 0) {
      printf("  i=%d cog z %.3f  Fz %.0f %.0f %.0f %.0f  hub z %.3f %.3f  strut L %.3f F %.0f / L %.3f F %.0f plastic %.0f broken %u\n", i, car.cog.z, car.wheels[0].Fz, car.wheels[1].Fz, car.wheels[2].Fz, car.wheels[3].Fz,
             car.wheels[0].hub.z, car.wheels[2].hub.z, car.axle[0].s[0].L, car.axle[0].s[0].F, car.axle[1].s[0].L, car.axle[1].s[0].F, sim.plasticEnergy, sim.brokenBeams);
    }
    if (car.wheels[0].deflate > 0 && car.wheels[0].deflate < 0.002) printf("  TYRE BURST at step %d Fz %.0f\n", i, car.wheels[0].Fz);
  }
}

void testBalance() {
  // External force balance on a car that should be at rest.
  World world;
  world.build();
  Sim sim;
  sim.world = &world;
  CarRig rig;
  Vehicle car = buildXantia(sim, {-90, 0, 0}, 0, &rig);
  sim.finalize();
  car.settleFluid(sim);
  Controls c;
  const Obj& o = sim.objs[car.obj];
  auto total = [&](bool grav) {
    V3 f{};
    for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) f += sim.force[i];
    return f;
  };
  double m = 0;
  V3 mg{};
  for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) m += sim.mass[i];
  for (int i = 0; i < 16000; i++) {
    sim.forces(kDt);
    const V3 afterBeams = total(true);
    car.update(sim, c, kDt);
    const V3 afterCar = total(true);
    std::vector<V3> before;
    if (i == 15999) before.assign(sim.force.begin(), sim.force.end());
    sim.finish(kDt);
    if (i == 15999) {
      V3 net{};
      printf("  nodes receiving contact force at the end:\n");
      for (u32 k = o.nodeStart; k < o.nodeStart + o.nodeCount; k++) {
        const V3 d = sim.force[k] - before[k];
        net += d;
        if (len(d) > 5.0)
          printf("    node %u rest (%.2f %+.2f %.2f) now (%.2f %+.2f %.3f) contact force (%+.0f %+.0f %+.0f) flags %d mass %.1f\n", k, 2.26 - (sim.rest[k].x + 90), sim.rest[k].y, sim.rest[k].z, sim.pos[k].x, sim.pos[k].y, sim.pos[k].z, d.x, d.y, d.z, sim.flags[k], sim.mass[k]);
      }
      printf("  net contact force (%+.0f %+.0f %+.0f)\n", net.x, net.y, net.z);
    }
    if (i % 2000 == 0 || i == 15999) {
      V3 fx{};
      for (const Wheel& w : car.wheels) fx.x += w.Fx, fx.y += w.Fy, fx.z += w.Fz;
      V3 p{};
      double zl = 0;
      printf("  t=%.1f beams+grav (%+.1f %+.1f %+.1f)  +vehicle (%+.1f %+.1f %+.1f)  tyres Fx %+.0f Fy %+.0f Fz %.0f  vCog (%+.3f %+.3f %+.3f) om %+.2f\n", i * kDt, afterBeams.x, afterBeams.y, afterBeams.z + m * 9.81, afterCar.x, afterCar.y, afterCar.z + m * 9.81, fx.x, fx.y, fx.z, car.vCog.x, car.vCog.y, car.vCog.z, car.wheels[0].omega);
      (void)p; (void)zl;
    }
  }
  double yc = 0;
  for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) yc += sim.pos[i].y * sim.mass[i];
  printf("  y cog %.4f  mass %.1f  FL %.0f FR %.0f RL %.0f RR %.0f  y hub %.3f %.3f\n", yc / m, m, car.wheels[0].Fz, car.wheels[1].Fz, car.wheels[2].Fz, car.wheels[3].Fz, car.wheels[0].hub.y, car.wheels[1].hub.y);
}

void testVibe() {
  Bench b;
  printf("== Tyre force statistics at rest over 1 s (mean, rms, zero crossings per second) ==\n");
  const char* nm[4] = {"FL", "FR", "RL", "RR"};
  double mFy[4] = {}, sFy[4] = {}, mFx[4] = {}, sFx[4] = {}, mFz[4] = {}, sFz[4] = {}, prevY[4] = {};
  int cross[4] = {};
  const int N = 2000;
  double mStroke[4] = {};
  b.run(N * kDt, [&](double, Controls&) {
    for (int i = 0; i < 4; i++) {
      const Wheel& w = b.car.wheels[i];
      mFy[i] += w.Fy; sFy[i] += w.Fy * w.Fy;
      mFx[i] += w.Fx; sFx[i] += w.Fx * w.Fx;
      mFz[i] += w.Fz; sFz[i] += w.Fz * w.Fz;
      const double y = w.Fy - mFy[i] / std::max(1.0, double(N));
      if (y * prevY[i] < 0) cross[i]++;
      prevY[i] = y;
    }
  });
  for (int i = 0; i < 4; i++) {
    const double ay = mFy[i] / N, ax = mFx[i] / N, az = mFz[i] / N;
    printf("  %s Fy mean %+7.0f rms %6.0f  Fx mean %+7.0f rms %6.0f  Fz mean %6.0f rms %6.0f  crossings %d  toe-ish uy %.4f\n", nm[i], ay, std::sqrt(std::max(0.0, sFy[i] / N - ay * ay)), ax, std::sqrt(std::max(0.0, sFx[i] / N - ax * ax)), az, std::sqrt(std::max(0.0, sFz[i] / N - az * az)), cross[i], b.car.wheels[i].uy);
  }
  V3 lv = b.car.vCog;
  printf("  vCog (%.3f %.3f %.3f) rms hub vel: ", lv.x, lv.y, lv.z);
  for (const Wheel& w : b.car.wheels) printf("%.3f ", len(b.sim.vel[w.axIn]));
  printf("\n");
}

void testImpulse() {
  Bench b;
  Sim& S = b.sim;
  const Obj& o = S.objs[b.car.obj];
  auto momentum = [&] {
    V3 p{};
    for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) p += S.vel[i] * S.mass[i];
    return p;
  };
  auto total = [&] {
    V3 f{};
    for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) f += S.force[i];
    return f;
  };
  V3 p0 = momentum(), iBeam{}, iVeh{}, iContact{}, tyres{};
  Controls c;
  const int N = 4000;
  std::vector<double> perNode(S.pos.size(), 0.0);
  for (int i = 0; i < N; i++) {
    S.forces(kDt);
    const V3 f1 = total();
    b.car.update(S, c, kDt);
    const V3 f2 = total();
    std::vector<V3> before = S.force;
    S.finish(kDt);
    const V3 f3 = total();
    for (u32 k = o.nodeStart; k < o.nodeStart + o.nodeCount; k++) perNode[k] += len(S.force[k] - before[k]) * kDt;
    iBeam += f1 * kDt;
    iVeh += (f2 - f1) * kDt;
    iContact += (f3 - f2) * kDt;
    for (const Wheel& w : b.car.wheels) tyres.z += w.Fz * kDt;
  }
  const double T = N * kDt;
  const V3 dp = momentum() - p0;
  printf("== Impulse balance over %.1f s (mean force in N) ==\n", T);
  printf("  beams+gravity (%+.1f %+.1f %+.1f)\n  vehicle       (%+.1f %+.1f %+.1f)\n  contacts      (%+.1f %+.1f %+.1f)\n  dp/dt         (%+.1f %+.1f %+.1f)\n  mean sum Fz %.0f\n",
         iBeam.x / T, iBeam.y / T, iBeam.z / T, iVeh.x / T, iVeh.y / T, iVeh.z / T, iContact.x / T, iContact.y / T, iContact.z / T, dp.x / T, dp.y / T, dp.z / T, tyres.z / T);
  std::vector<u32> idx;
  for (u32 k = o.nodeStart; k < o.nodeStart + o.nodeCount; k++) if (perNode[k] > 0.1) idx.push_back(k);
  std::sort(idx.begin(), idx.end(), [&](u32 a, u32 b_) { return perNode[a] > perNode[b_]; });
  for (size_t j = 0; j < idx.size() && j < 12; j++)
    printf("   contact node %u design (%.2f %+.2f %.2f) mean |F| %.0f N mass %.1f flags %d now z %.3f\n", idx[j], 2.26 - (S.rest[idx[j]].x + 90), S.rest[idx[j]].y, S.rest[idx[j]].z, perNode[idx[j]] / T, S.mass[idx[j]], S.flags[idx[j]], S.pos[idx[j]].z);
}

void testIntegrator() {
  Bench b;
  Sim& S = b.sim;
  const Obj& o = S.objs[b.car.obj];
  Controls c;
  S.forces(kDt);
  b.car.update(S, c, kDt);
  std::vector<V3> v0 = S.vel;
  std::vector<V3> f0 = S.force;
  S.finish(kDt);
  double worst = 0;
  u32 wn = 0;
  V3 sumF{}, sumDp{};
  int bad = 0;
  for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) {
    const V3 dv = S.vel[i] - v0[i];
    const V3 expect = S.force[i] * (kDt / S.mass[i]);
    const double e = len(dv - expect);
    sumF += S.force[i];
    sumDp += dv * (S.mass[i] / kDt);
    if (e > 1e-9) bad++;
    if (e > worst) {
      worst = e;
      wn = i;
    }
  }
  printf("== Integrator check: %d nodes differ, worst %.3g at node %u (mass %.2f invMass %.4f, 1/m %.4f, flags %d)\n  sum F (%.1f %.1f %.1f)  sum dp/dt (%.1f %.1f %.1f)\n", bad, worst, wn, S.mass[wn], S.invMass[wn], 1.0 / S.mass[wn], S.flags[wn], sumF.x, sumF.y, sumF.z, sumDp.x, sumDp.y, sumDp.z);
  printf("  obj nodes start %u count %u, total nodes %zu, active %zu\n", o.nodeStart, o.nodeCount, S.pos.size(), S.active.size());
  int nFixed = 0;
  for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) nFixed += S.invMass[i] == 0 ? 1 : 0;
  printf("  nodes with zero inverse mass: %d\n", nFixed);
  for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) {
    const double a = len(S.force[i]) / S.mass[i] * kDt;
    if (a > 5.0) {
      printf("   node %u mass %.2f design (%.2f %+.2f %.2f) now (%.2f %+.2f %.2f) |F| %.0f dv %.1f\n", i, S.mass[i], 2.26 - (S.rest[i].x + 90), S.rest[i].y, S.rest[i].z, S.pos[i].x, S.pos[i].y, S.pos[i].z, len(S.force[i]), a);
      for (u32 bi = o.beamStart; bi < o.beamStart + o.beamCount; bi++) {
        const Beam& B = S.beams[bi];
        if ((B.a == i || B.b == i) && std::fabs(B.force) > 30000.0)
          printf("      beam to %u k %.1e L0 %.3f now %.3f force %.0f\n", B.a == i ? B.b : B.a, B.k, B.L0, len(S.pos[B.b] - S.pos[B.a]), double(B.force));
      }
    }
  }
}

void testGlass() {
  Bench b;
  CarMesh mesh;
  mesh.load("assets/xantia.mesh");
  mesh.bind(b.sim.rest, b.skin.groups, {-90, 0, 0});
  printf("== Mesh strain at rest after settling ==\n");
  constexpr int K = CarMesh::kK;
  std::vector<V3> v(mesh.restPos.size() / 3);
  for (const MeshPart& p : mesh.parts) {
    if (!p.bound) continue;
    for (u32 i = p.firstVert; i < p.firstVert + p.vertCount; i++) {
      V3 q{};
      for (int k = 0; k < K; k++) q += b.sim.pos[mesh.bindNode[size_t(i) * K + k]] * double(mesh.bindWeight[size_t(i) * K + k]);
      v[i] = q;
    }
  }
  for (const MeshPart& p : mesh.parts) {
    if (!p.bound || (p.mat != MM_GLASS && p.name != "hood" && p.name != "gate_top" && p.name != "door_fl" && p.name != "roof")) continue;
    double worst = 0;
    int over = 0;
    for (u32 t = p.firstTri; t < p.firstTri + p.triCount; t++) {
      const u32 a = mesh.tris[t * 3], c = mesh.tris[t * 3 + 1], d = mesh.tris[t * 3 + 2];
      const double r[3] = {len2(v[c] - v[a]) / std::max<double>(mesh.restEdge2[size_t(t) * 3], 1e-7), len2(v[d] - v[c]) / std::max<double>(mesh.restEdge2[size_t(t) * 3 + 1], 1e-7), len2(v[a] - v[d]) / std::max<double>(mesh.restEdge2[size_t(t) * 3 + 2], 1e-7)};
      const double m = std::sqrt(std::max({r[0], r[1], r[2]}));
      worst = std::max(worst, m);
      over += m > 1.04;
    }
    printf("  %-12s group %-10s tris %5u  worst edge ratio %.3f  over 4%%: %d\n", p.name.c_str(), p.group.c_str(), p.triCount, worst, over);
  }
}

void testDiag() {
  testGlass();
  return;
  testImpulse();
  testVibe();
  return;
  testSpawn();
  Bench b;
  printf("  time series after settle (no input):\n");
  b.run(3.0, [&](double t, Controls&) {
    if (int(t * 2000) % 1000 == 0) {
      printf("   t=%.1f cog z %.4f v %.3f pitch %.2f roll %.2f  Fz %.0f %.0f %.0f %.0f  om %.2f %.2f %.2f %.2f rpm %.0f gear %d clutch %.2f\n", t, b.car.cog.z, b.car.speed, b.car.pitchAngle * 57.3, b.car.rollAngle * 57.3,
             b.car.wheels[0].Fz, b.car.wheels[1].Fz, b.car.wheels[2].Fz, b.car.wheels[3].Fz, b.car.wheels[0].omega, b.car.wheels[1].omega, b.car.wheels[2].omega, b.car.wheels[3].omega, b.car.rpm(), b.car.gear, b.car.clutch);
    }
  });
  const Sim& S = b.sim;
  const Obj& o = S.objs[b.car.obj];
  printf("== Diagnostics: beams that yielded or broke while settling ==\n");
  V3 cg{};
  double m = 0;
  for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) {
    cg += S.rest[i] * S.mass[i];
    m += S.mass[i];
  }
  cg = cg / m;
  printf("  rest cog design x %.3f (want 2.051), y %.4f, z %.3f, mass %.1f\n", 2.26 - (cg.x + 90), cg.y, cg.z, m);
  double minM = 1e9, maxM = 0;
  for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) {
    minM = std::min(minM, S.mass[i]);
    maxM = std::max(maxM, S.mass[i]);
  }
  printf("  node mass range %.2f .. %.2f\n", minM, maxM);
  int shown = 0;
  for (u32 i = o.beamStart; i < o.beamStart + o.beamCount; i++) {
    const Beam& B = S.beams[i];
    const double pl = (B.L0 - B.Lorig) / B.Lorig;
    const bool broken = B.flags & BF_BROKEN;
    if (!broken && std::fabs(pl) < 0.002) continue;
    if (shown++ > 25 && !broken) continue;
    const V3 a = S.rest[B.a], c = S.rest[B.b];
    printf("  %s plastic %+.3f k %.1e yC %.0f L %.2f  (%.2f %+.2f %.2f)-(%.2f %+.2f %.2f) now L %.3f\n", broken ? "BROKEN" : "yield ", pl, B.k, B.yC, B.Lorig,
           2.26 - (a.x + 90), a.y, a.z, 2.26 - (c.x + 90), c.y, c.z, len(S.pos[B.b] - S.pos[B.a]));
  }
  printf("  total affected %d\n", shown);
  for (const Wheel& w : b.car.wheels)
    printf("  wheel %s%s Fz %.0f hub z %.3f contact %d omega %.2f\n", w.front ? "F" : "R", w.left ? "L" : "R", w.Fz, w.hub.z, w.contact, w.omega);
  for (int a = 0; a < 2; a++)
    for (int s = 0; s < 2; s++) {
      const HydroStrut& H = b.car.axle[a].s[s];
      printf("  strut %d%d L %.3f Lnom %.3f F %.0f p %.1f bar Vf %.1f cc ram %.3f\n", a, s, H.L, H.Lnom, H.F, H.pGas / 1e5, H.Vf * 1e6, b.car.axle[a].ram);
    }
}

void testBounce() {
  Bench b;
  // Push the body down and let go, then count zero crossings of the height.
  const double z0 = b.car.cog.z;
  printf("== Heave (firm/soft) ==\n");
  for (int sport = 0; sport < 2; sport++) {
    Controls t;
    if (sport) {
      t.toggleSport = true;
      b.run(0.01, [&](double, Controls& c) { c.toggleSport = true; });
      b.car.sportMode = true;
      b.run(1.0, [](double, Controls&) {});
    }
    b.run(0.3, [&](double, Controls&) {
      const Obj& o = b.sim.objs[b.car.obj];
      for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++)
        if (!(b.sim.flags[i] & NF_WHEEL)) b.sim.force[i].z -= 0;
    });
    const Obj& o = b.sim.objs[b.car.obj];
    for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) b.sim.vel[i].z -= 0.35;
    double prev = b.car.vCog.z;
    int crossings = 0;
    double tFirst = -1, tLast = -1, ampMax = 0;
    b.run(4.0, [&](double t, Controls&) {
      const double d = b.car.vCog.z;
      ampMax = std::max(ampMax, std::fabs(b.car.cog.z - z0));
      if (t > 0.1 && prev * d < 0) {
        if (crossings == 0) tFirst = t;
        tLast = t;
        crossings++;
      }
      prev = d;
    });
    const double f = crossings >= 2 ? 0.5 * (crossings - 1) / (tLast - tFirst) : 0;
    printf("  %s: frequency %.2f Hz, peak %.3f m, crossings %d\n", sport ? "firm" : "soft", f, ampMax,
           crossings);
  }
}

void testAccel() {
  Bench b;
  b.world.clearBoxes();
  b.world.flatten();
  printf("== Acceleration ==\n");
  double t100 = -1, t60 = -1, tTop = 0, vTop = 0, t400 = -1;
  V3 start = b.car.cog;
  bool ok = true;
  b.run(70.0, [&](double t, Controls& c) {
    c.throttle = 1.0;
    const double kmh = b.car.speed * 3.6;
    if (t60 < 0 && kmh >= 60) t60 = t;
    if (t100 < 0 && kmh >= 100) t100 = t;
    if (t400 < 0 && len(b.car.cog - start) >= 400) t400 = t;
    if (kmh > vTop) {
      vTop = kmh;
      tTop = t;
    }
    // Stay on the terrain: the pad is short, so drive around on a big circle once fast.
    if (t > 3) c.steer = clampd(0.012 * 45.0 / std::max(b.car.speed, 10.0), 0, 0.08);  // ~400 m circle
    if ((int(t * 2000) % 20000) == 0)
      printf("  t=%4.0f v=%5.1f km/h gear %d rpm %5.0f  z=%.2f\n", t, kmh, b.car.gear, b.car.rpm(), b.car.cog.z);
  });
  ok = b.sane();
  printf("  0-60: %.2f s  0-100: %.2f s  400 m: %.2f s  vmax: %.1f km/h at t=%.0f  sane %d\n", t60, t100, t400, vTop, tTop, ok);
  printf("  (listed: about 8.2 s, 230 km/h)\n");
  printf("  fuel %.1f l, LHM %.2f l, accumulator %.0f bar, gear %d, shaft twist %.3f rad\n", b.car.fuelLitres, b.car.reservoir * 1000, b.car.accumulator / 1e5, b.car.gear, b.car.shaftAngle[0]);
}

void testBraking() {
  Bench b;
  printf("== Braking from 100 km/h ==\n");
  b.launchAt(100 / 3.6, 3);
  b.run(0.5, [](double, Controls& c) { c.throttle = 0.2; });
  const V3 start = b.car.cog;
  const double v0 = b.car.speed * 3.6;
  double stopT = -1, stopD = 0, pitchMax = 0;
  b.run(8.0, [&](double t, Controls& c) {
    c.brake = 1.0;
    pitchMax = std::max(pitchMax, std::fabs(b.car.pitchAngle));
    if (stopT < 0 && b.car.speed < 0.3) {
      stopT = t;
      stopD = len(b.car.cog - start);
    }
  });
  printf("  v0 %.1f km/h  stop %.2f s  %.1f m  (real: about 40 m), max pitch %.2f deg\n", v0, stopT,
         stopD, pitchMax * 57.3);
}

void testSteerRamp(bool activa) {
  // Constant speed, slowly increasing steering: the standard way to read an understeer curve.
  Bench b({-60, -110, 0}, 0);
  b.world.clearBoxes();
  b.car.activaOn = activa;
  const double vT = 75 / 3.6;
  b.launchAt(vT, 3);
  printf("== Steer ramp at 75 km/h, Activa %s ==\n  steer  lat g  roll deg  speed  front a  rear a  inner-front Fz\n", activa ? "ON" : "OFF");
  double next = 2.0, latMax = 0, rollAt = 0, steerAt = 0;
  double latF = 0;
  b.run(26.0, [&](double t, Controls& c) {
    c.steer = 0.05 + 0.012 * std::max(0.0, t - 2.0);
    c.throttle = clampd((vT - b.car.speed) * 0.9, 0, 1);
    latF += (b.car.latAcc - latF) * 0.0015;
    if (t > 2 && latF > latMax) {
      latMax = latF;
      rollAt = b.car.rollAngle;
      steerAt = b.car.steerAct;
    }
    if (t >= next) {
      next += 4.0;
      const auto& W = b.car.wheels;
      printf("  %.3f  %5.2f  %7.2f  %5.1f  %6.1f  %6.1f  %6.0f\n", b.car.steerAct, latF / 9.81, b.car.rollAngle * 57.3, b.car.speed * 3.6,
             0.5 * std::atan(W[0].slipY + W[1].slipY) * 57.3, 0.5 * std::atan(W[2].slipY + W[3].slipY) * 57.3, W[0].Fz);
    }
  });
  printf("  -> max sustained lateral %.2f g at steer %.2f, roll there %.2f deg, sane %d\n", latMax / 9.81, steerAt, rollAt * 57.3, b.sane());
}

void testMoose(bool activa, double kmh) {
  // Open-loop double lane change: a fast swerve left, right, and back.
  Bench b({-120, 0, 0}, 0);
  b.world.clearBoxes();
  b.car.activaOn = activa;
  const double vT = kmh / 3.6;
  b.launchAt(vT, 3);
  b.run(1.0, [&](double, Controls& c) { c.throttle = clampd((vT - b.car.speed), 0, 1); });
  double rollMax = 0, latMax = 0, slipMax = 0;
  double minFz = 1e9;
  b.run(5.0, [&](double t, Controls& c) {
    const double T = 2.0;
    c.throttle = 0;
    if (t < T) c.steer = 0.20 * std::sin(2.0 * PI * t / T) * (t < T ? 1.0 : 0.0);
    rollMax = std::max(rollMax, std::fabs(b.car.rollAngle));
    latMax = std::max(latMax, std::fabs(b.car.latAcc));
    const V3 v = b.car.vCog;
    if (len(v) > 3.0) slipMax = std::max(slipMax, std::fabs(std::asin(clampd(dot(norm(v), b.car.left), -1, 1))));
    for (const Wheel& w : b.car.wheels) minFz = std::min(minFz, w.Fz);
  });
  printf("  moose %3.0f km/h Activa %-3s: peak roll %5.2f deg  peak lat %.2f g  peak body slip %5.2f deg  final v %.1f km/h  sane %d\n", kmh, activa ? "ON" : "OFF", rollMax * 57.3, latMax / 9.81, slipMax * 57.3, b.car.speed * 3.6, b.sane());
}

void testCrash(double kmh) {
  // Real wall in the world at x = 110 - 0.6, lane y = -30.
  Bench b({85, -30, 0}, 0);
  printf("== Frontal wall crash at %.0f km/h ==\n", kmh);
  b.launchAt(kmh / 3.6, 3);
  const Obj& o = b.sim.objs[b.car.obj];
  auto noseFirewall = [&] {
    // Distance from the firewall to the rear axle: stays nearly constant unless the cabin folds.
    return std::make_pair(b.car.refFrontL, b.car.refRearL);
  };
  (void)noseFirewall;
  double lenBefore = 0, cabinBefore = len(b.sim.pos[b.car.refFrontL] - b.sim.pos[b.car.refRearL]);
  {
    double mn = 1e9, mx = -1e9;
    for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) {
      mn = std::min(mn, b.sim.pos[i].x);
      mx = std::max(mx, b.sim.pos[i].x);
    }
    lenBefore = mx - mn;
  }
  double peak = 0, tStart = -1, tEnd = -1;
  V3 prevV = b.car.vCog;
  double vPrev = b.car.speed;
  double accLP = 0;
  b.run(3.0, [&](double t, Controls& c) {
    const double a = (b.car.speed - vPrev) / kDt;
    vPrev = b.car.speed;
    accLP += (a - accLP) * 0.02;  // ~6 ms filter
    if (tStart < 0 && accLP < -30) tStart = t;
    if (tStart > 0 && tEnd < 0 && b.car.speed < 0.2) tEnd = t;
    peak = std::max(peak, -accLP);
  });
  (void)prevV;
  double mn = 1e9, mx = -1e9;
  for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) {
    mn = std::min(mn, b.sim.pos[i].x);
    mx = std::max(mx, b.sim.pos[i].x);
  }
  const double cabinAfter = len(b.sim.pos[b.car.refFrontL] - b.sim.pos[b.car.refRearL]);
  int brokenCar = 0;
  for (u32 i = o.beamStart; i < o.beamStart + o.beamCount; i++)
    brokenCar += (b.sim.beams[i].flags & BF_BROKEN) ? 1 : 0;
  const double E0 = 0.5 * b.car.totalMass * std::pow(kmh / 3.6, 2);
  printf("  peak decel %.1f g (6 ms filter)  duration %.0f ms\n", peak / 9.81, (tEnd - tStart) * 1000);
  // Crush measured from the rear floor to the nose of the rails, ignoring parts that came off.
  auto xOf = [&](int idx) { return b.sim.pos[idx].x - b.sim.pos[o.nodeStart + 16 * 13].x; };
  (void)xOf;
  printf("  overall extent %.2f -> %.2f m  cabin %.3f -> %.3f m\n", lenBefore, mx - mn, cabinBefore, cabinAfter);
  printf("  station shortening along the car (design x: change in distance to rear floor reference)\n   ");
  for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) {
    const V3 r = b.sim.rest[i];
    if (std::fabs(r.y + 30.0) > 1e-6 || r.z > 0.4) continue;  // floor centreline nodes
    const double before = r.x - b.sim.rest[b.car.refRearL].x;
    const double after = dot(b.sim.pos[i] - b.sim.pos[b.car.refRearL], b.car.fwd);
    printf(" %.2f:%+.2f", 2.26 - (r.x - 85), after - before);
  }
  printf("\n");
  {
    int radPl = 0;
    double radMax = 0;
    for (u32 id : b.car.radiatorBeams) {
      const Beam& B = b.sim.beams[id];
      const double e = std::fabs(B.L0 - B.Lorig) / B.Lorig;
      radMax = std::max(radMax, e);
      radPl += ((B.flags & BF_BROKEN) || e > 0.05);
    }
    printf("  radiator beams: %zu, damaged %d, max strain %.3f\n", b.car.radiatorBeams.size(), radPl, radMax);
    int mountBroken = 0;
    for (u32 id : b.car.engineMountBeams) mountBroken += (b.sim.beams[id].flags & BF_BROKEN) ? 1 : 0;
    printf("  engine mounts broken %d/%zu, last note '%s', dmg check tick engine-to-wall\n", mountBroken, b.car.engineMountBeams.size(), b.car.lastDamage.c_str());
  }
  printf("  plastic energy %.1f kJ of %.1f kJ kinetic, beams broken %d, final speed %.1f\n", b.sim.plasticEnergy / 1e3, E0 / 1e3, brokenCar, b.car.speed);
  for (const Wheel& w : b.car.wheels) {
    double mx = 0;
    for (float d : w.dent) mx = std::max(mx, double(d));
    printf("  wheel %s%s: flat %.2f, deepest rim dent %.1f mm, runout %.1f mm, hub height %.3f\n", w.front ? "F" : "R", w.left ? "L" : "R", w.deflate, mx * 1000, std::hypot(w.runoutX, w.runoutZ) * 1000, w.hub.z);
  }
  printf("  damage: rad %d hyd %d mount %d steer %d seized %d  sane %d\n", b.car.dmg.radiator, b.car.dmg.hydraulics, b.car.dmg.engineMount, b.car.dmg.steering, b.car.dmg.seized, b.sane());
}

void testProps() {
  Bench b({-90, 0, 0}, 0, true);
  printf("== Prop gauntlet ==\n");
  printf("  total nodes %zu, beams %zu, blocks %zu, objects %zu\n", b.sim.pos.size(), b.sim.beams.size(), b.sim.blocks.size(), b.sim.objs.size());
  const u32 carBeams = b.sim.objs[b.car.obj].beamCount;
  const u32 base = b.sim.brokenBeams;
  b.launchAt(60 / 3.6, 2);
  double maxActive = 0;
  auto t0 = std::chrono::steady_clock::now();
  double next = 0;
  double dur = 0;
  b.run(15.0, [&](double t, Controls& c) {
    c.throttle = b.car.speed < 60 / 3.6 ? 0.8 : 0.0;
    maxActive = std::max(maxActive, double(b.sim.active.size()));
    dur = t;
    if (t >= next) {
      next += 1.0;
      int broke = 0;
      for (u32 i = carBeams; i < b.sim.beams.size(); i++) broke += (b.sim.beams[i].flags & BF_BROKEN) ? 1 : 0;
      printf("  t=%4.1f x=%6.1f v=%5.1f km/h awake objs %3zu  broken prop beams %d  car plastic %.1f kJ  tripairs %zu blockpairs %zu wall %.1fs\n", t, b.car.cog.x, b.car.speed * 3.6, b.sim.active.size(), broke, b.sim.plasticEnergy / 1e3, b.sim.statTriPairs, b.sim.statBlockPairs, std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
      if (false)
        for (u32 oi : b.sim.active) {
          const Obj& O = b.sim.objs[oi];
          double vm = 0;
          for (u32 i = O.nodeStart; i < O.nodeStart + O.nodeCount; i++) vm = std::max(vm, len(b.sim.vel[i]));
          int br = 0;
          for (u32 i = O.beamStart; i < O.beamStart + O.beamCount; i++) br += (b.sim.beams[i].flags & BF_BROKEN) ? 1 : 0;
          printf("     obj %u kind %d nodes %u beams %u broken %d vmax %.1f at (%.0f %.0f) bb z %.1f..%.1f\n", oi, O.kind, O.nodeCount, O.beamCount, br, vm, b.sim.pos[O.nodeStart].x, b.sim.pos[O.nodeStart].y, O.bbMin.z, O.bbMax.z);
        }
    }
  });
  const double ms = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  printf("  %.2f s real time for %.1f s simulated (%.0f %% of one core), base %u, sane %d, max awake %d\n", ms, dur, 100 * ms / dur, base, b.sane(), int(maxActive));
}

void testRough() {
  printf("== Rough roads and abuse (structure must survive, physics must stay sane) ==\n");
  auto report = [](const char* name, Bench& b, double zMin, double zMax) {
    int broken = 0;
    const Obj& o = b.sim.objs[b.car.obj];
    for (u32 i = o.beamStart; i < o.beamStart + o.beamCount; i++) broken += (b.sim.beams[i].flags & BF_BROKEN) ? 1 : 0;
    int flat = 0;
    for (const Wheel& w : b.car.wheels) flat += w.deflate > 0 ? 1 : 0;
    double dentMax = 0;
    for (const Wheel& w : b.car.wheels)
      for (float d : w.dent) dentMax = std::max(dentMax, double(d));
    printf("  %-22s v=%5.1f km/h body z %.2f..%.2f  plastic %6.1f kJ  broken %3d  flat %d  rim dent %4.1f mm  sane %d\n", name, b.car.speed * 3.6, zMin, zMax, b.sim.plasticEnergy / 1e3, broken, flat, dentMax * 1000, b.sane());
  };
  auto drive = [](Bench& b, double seconds, double kmh, double& zMin, double& zMax, double yT, double steerOverride = 9) {
    b.run(seconds, [&](double, Controls& c) {
      c.throttle = clampd((kmh / 3.6 - b.car.speed) * 0.6, 0, 1);
      const double head = std::atan2(b.car.fwd.y, b.car.fwd.x);
      c.steer = steerOverride < 9 ? steerOverride : clampd(-0.05 * (b.car.cog.y - yT) - 0.7 * head, -0.25, 0.25);
      zMin = std::min(zMin, b.car.cog.z);
      zMax = std::max(zMax, b.car.cog.z);
    });
  };
  for (double kmh : {30.0, 70.0}) {
    Bench b({-140, 76, 0}, 0);
    b.launchAt(kmh / 3.6, 2);
    double a = 9, c = -9;
    drive(b, 70.0 / (kmh / 3.6), kmh, a, c, 76);
    report(kmh < 50 ? "pave 30 km/h" : "pave 70 km/h", b, a, c);
  }
  {
    Bench b({-140, 96, 0}, 0);
    b.launchAt(50 / 3.6, 2);
    double a = 9, c = -9;
    drive(b, 5.5, 50, a, c, 96);
    report("washboard 50 km/h", b, a, c);
  }
  {
    Bench b({-60, 70, 0}, 0);
    b.launchAt(40 / 3.6, 2);
    double a = 9, c = -9;
    double next = 0;
    double firstBreak = -1;
    b.run(8.0, [&](double t, Controls& ctl) {
      ctl.throttle = clampd((40 / 3.6 - b.car.speed) * 0.6, 0, 1);
      const double head = std::atan2(b.car.fwd.y, b.car.fwd.x);
      ctl.steer = clampd(-0.05 * (b.car.cog.y - 70) - 0.7 * head, -0.25, 0.25);
      a = std::min(a, b.car.cog.z);
      c = std::max(c, b.car.cog.z);
      double maxFz = 0;
      for (const Wheel& w : b.car.wheels) maxFz = std::max(maxFz, w.Fz);
      if (firstBreak < 0 && b.sim.brokenBeams > 0) {
        firstBreak = t;
        printf("  first break at t=%.3f x=%.2f maxFz %.0f peakG %.1f\n", t, b.car.cog.x, maxFz, b.car.peakG);
      }
      (void)next;
    });
    report("kerbs 40 km/h", b, a, c);
    std::map<std::string, int> counts;
    const Obj& o = b.sim.objs[b.car.obj];
    for (u32 i = o.beamStart; i < o.beamStart + o.beamCount; i++) {
      const Beam& B = b.sim.beams[i];
      if (!(B.flags & BF_BROKEN)) continue;
      char key[120];
      snprintf(key, sizeof key, "x%.1f-%.1f z%.1f-%.1f k%.1e fB%.0f", 2.26 - (b.sim.rest[B.a].x + 60), 2.26 - (b.sim.rest[B.b].x + 60), b.sim.rest[B.a].z, b.sim.rest[B.b].z, B.k, B.fBreak);
      counts[key]++;
    }
    int shown = 0;
    for (auto& kv : counts)
      if (shown++ < 30) printf("    %3d x %s\n", kv.second, kv.first.c_str());
  }
  {
    Bench b({-30, 40, 0}, 0);
    b.launchAt(85 / 3.6, 3);
    double a = 9, c = -9;
    drive(b, 6.0, 85, a, c, 40);
    report("jump 85 km/h", b, a, c);
  }
  {
    Bench b({-120, -60, 0}, 0);
    b.world.clearBoxes();
    b.launchAt(120 / 3.6, 4);
    double a = 9, c = -9;
    drive(b, 6.0, 120, a, c, 0, 1.0);
    report("full lock 120 km/h", b, a, c);
  }
  {
    // Reverse gear through the same request the automatic uses.
    Bench b;
    b.run(4.0, [](double, Controls& c) { c.requestGear = -1; c.throttle = 0.5; });
    printf("  reverse: gear %d v=%.1f km/h moved %.1f m  sane %d\n", b.car.gear, b.car.speed * 3.6, b.car.cog.x + 90.0, b.sane());
  }
  {
    // Parked on the road's steepest part must hold, here the handbrake on a slope.
    Bench b;
    const double x0 = b.car.cog.x;
    b.run(5.0, [](double, Controls& c) { c.handbrake = 1.0; });
    printf("  parked drift over 5 s: %.1f mm\n", std::fabs(b.car.cog.x - x0) * 1000);
  }
}

void testPowertrain() {
  printf("== Powertrain trace: launch, shifts, lift-off, engine off ==\n");
  Bench b;
  b.world.clearBoxes();
  double next = 0;
  b.world.flatten();
  auto report = [&](double t) {
    if (t >= next) {
      next += 0.5;
      printf("  t=%4.1f v=%5.1f gear %2d phase %d rpm %5.0f disc %6.1f input %6.1f carrier %6.1f spider %+6.2f clutch %.2f mani %.2f shaft %+6.0f %+6.0f Nm  acc %3.0f bar fuel %.2f\n", t, b.car.speed * 3.6, b.car.gear, b.car.shiftPhase, b.car.rpm(), b.car.discOmega, b.car.inputOmega, b.car.carrierOmega, b.car.spiderOmega, b.car.clutch, b.car.manifold, b.car.wheels[0].driveT, b.car.wheels[1].driveT, b.car.accumulator / 1e5, b.car.fuelLitres);
    }
  };
  b.run(9.0, [&](double t, Controls& c) { c.throttle = 1.0; report(t); });
  next = 0;
  next = 9.0;
  b.run(3.0, [&](double t, Controls& c) { report(t + 9.0); });
  next = 12.0;
  bool once = false;
  b.run(3.0, [&](double t, Controls& c) {
    c.toggleIgnition = !once;
    once = true;
    report(t + 12.0);
  });
  // One wheel on low grip: the open differential must send the speed to the slipping wheel.
  printf("  hydraulics after engine stop: %.0f bar, front strut %.1f bar\n", b.car.accumulator / 1e5, b.car.axle[0].s[0].pGas / 1e5);
}

// How much does the visible car shake? Tracks every node in the body frame and every mesh vertex,
// sampled at 60 Hz like the renderer, and reports frame-to-frame motion.
void testJitter(const char* label, double kmh) {
  Bench b;
  b.world.clearBoxes();
  b.world.flatten();
  CarMesh mesh;
  mesh.load("assets/xantia.mesh");
  mesh.bind(b.sim.rest, b.skin.groups, {-90, 0, 0});
  if (kmh > 0) b.launchAt(kmh / 3.6, kmh > 60 ? 3 : 2);
  const Obj& o = b.sim.objs[b.car.obj];
  auto toBody = [&](V3 p) {
    const V3 r = p - b.car.cog;
    return V3{dot(r, b.car.fwd), dot(r, b.car.left), dot(r, b.car.up)};
  };
  b.run(1.5, [&](double, Controls& c) {
    c.throttle = clampd((kmh / 3.6 - b.car.speed) * 0.5, 0, 1);
    if (kmh > 0) c.steer = 0.01;
  });
  const size_t nv = mesh.restPos.size() / 3;
  constexpr int K = CarMesh::kK;
  std::vector<V3> prevV(nv), prevN(o.nodeCount);
  std::vector<double> maxStep(mesh.parts.size(), 0.0), meanStep(mesh.parts.size(), 0.0);
  std::vector<double> nodeStep(o.nodeCount, 0.0);
  int frames = 0;
  const int stepsPerFrame = 33;
  for (int f = 0; f < 120; f++) {
    b.run(stepsPerFrame * kDt, [&](double, Controls& c) {
      c.throttle = clampd((kmh / 3.6 - b.car.speed) * 0.5, 0, 1);
      if (kmh > 0) c.steer = 0.01;
    });
    for (size_t pi = 0; pi < mesh.parts.size(); pi++) {
      const MeshPart& p = mesh.parts[pi];
      if (!p.bound) continue;
      for (u32 v = p.firstVert; v < p.firstVert + p.vertCount; v += 7) {
        V3 q{};
        for (int k = 0; k < K; k++) q += b.sim.pos[mesh.bindNode[size_t(v) * K + k]] * double(mesh.bindWeight[size_t(v) * K + k]);
        q = toBody(q);
        if (f > 0) {
          const double d = len(q - prevV[v]);
          maxStep[pi] = std::max(maxStep[pi], d);
          meanStep[pi] += d;
        }
        prevV[v] = q;
      }
    }
    for (u32 i = 0; i < o.nodeCount; i++) {
      const V3 q = toBody(b.sim.pos[o.nodeStart + i]);
      if (f > 0) nodeStep[i] = std::max(nodeStep[i], len(q - prevN[i]));
      prevN[i] = q;
    }
    frames++;
  }
  printf("== Shake at %s: frame-to-frame motion in the body frame, mm ==\n", label);
  std::vector<std::pair<double, size_t>> order;
  for (size_t pi = 0; pi < mesh.parts.size(); pi++)
    if (mesh.parts[pi].bound) order.push_back({maxStep[pi], pi});
  std::sort(order.rbegin(), order.rend());
  for (size_t i = 0; i < 10 && i < order.size(); i++) {
    const MeshPart& p = mesh.parts[order[i].second];
    printf("  %-16s group %-10s max %7.2f  mean %6.2f\n", p.name.c_str(), p.group.c_str(), order[i].first * 1000, meanStep[order[i].second] / (frames * (p.vertCount / 7 + 1)) * 1000);
  }
  std::vector<std::pair<double, u32>> no;
  for (u32 i = 0; i < o.nodeCount; i++) no.push_back({nodeStep[i], i});
  std::sort(no.rbegin(), no.rend());
  for (int i = 0; i < 6; i++) {
    const u32 n = o.nodeStart + no[size_t(i)].second;
    printf("  node %3u design (%.2f %+.2f %.2f) mass %.1f max step %.2f mm\n", n, 2.26 - (b.sim.rest[n].x + 90), b.sim.rest[n].y, b.sim.rest[n].z, b.sim.mass[n], no[size_t(i)].first * 1000);
  }
  printf("  broken %u, guard %llu\n", b.sim.brokenBeams, (unsigned long long)b.sim.clampedSteps);
  // The wheels as drawn: hub position and orientation, sampled like the renderer.
  V3 prevHub[4], prevAxis[4];
  double hubMax[4] = {}, axisMax[4] = {}, hubAvg[4] = {};
  for (int f = 0; f < 60; f++) {
    b.run(33 * kDt, [&](double, Controls& c) {
      c.throttle = clampd((kmh / 3.6 - b.car.speed) * 0.5, 0, 1);
      if (kmh > 0) c.steer = 0.01;
    });
    for (int i = 0; i < 4; i++) {
      const Wheel& w = b.car.wheels[i];
      const V3 hub = toBody((b.sim.pos[w.axIn] + b.sim.pos[w.axOut]) * 0.5);
      const V3 ax = norm(b.sim.pos[w.axOut] - b.sim.pos[w.axIn]);
      if (f > 0) {
        const double d = len(hub - prevHub[i]);
        hubMax[i] = std::max(hubMax[i], d);
        hubAvg[i] += d / 59.0;
        axisMax[i] = std::max(axisMax[i], len(ax - prevAxis[i]));
      }
      prevHub[i] = hub;
      prevAxis[i] = ax;
    }
  }
  for (int i = 0; i < 4; i++)
    printf("  wheel %d hub step max %.2f mean %.2f mm, axle direction step max %.2f deg\n", i, hubMax[i] * 1000, hubAvg[i] * 1000, axisMax[i] * 57.3);
}

void testWheelDamage() {
  printf("== Flat tyre and bent rim while rolling ==\n");
  Bench b;
  b.world.clearBoxes();
  b.world.flatten();
  const double h0 = b.car.wheels[0].hub.z;
  b.launchAt(40 / 3.6, 2);
  double zMin = 9, zMax = -9;
  b.run(2.0, [&](double, Controls& c) { c.throttle = 0.2; });
  for (int i = 0; i < 2; i++) {
    Wheel& w = b.car.wheels[0];
    if (i == 1) {
      w.deflate = 0.001;
      printf("  burst: ");
    } else {
      w.dent[5] = w.dent[6] = 0.03f;
      printf("  30 mm rim dent: ");
    }
    double aMax = 0;
    b.run(3.0, [&](double, Controls& c) {
      c.throttle = 0.2;
      zMin = std::min(zMin, b.car.wheels[0].hub.z);
      zMax = std::max(zMax, b.car.wheels[0].hub.z);
      aMax = std::max(aMax, std::fabs(b.car.pitchAngle));
    });
    printf("hub z range %.3f..%.3f (healthy %.3f), v %.1f km/h, pull: heading %.1f deg, sane %d\n", zMin, zMax, h0, b.car.speed * 3.6, std::atan2(b.car.fwd.y, b.car.fwd.x) * 57.3, b.sane());
    zMin = 9;
    zMax = -9;
  }
}

void testWheelStrike() {
  // Front left wheel driven into the end of a concrete wall, offset so that the wheel takes the hit.
  printf("== Wheel strike against the scrape wall end ==\n");
  for (double kmh : {20.0, 40.0, 70.0}) {
    Bench b({60 - 30 - 12, 140 - 0.5 - 0.72, 0}, 0);
    b.launchAt(kmh / 3.6, 2);
    b.run(2.5, [&](double, Controls& c) { c.throttle = 0.1; });
    const Wheel& w = b.car.wheels[0];
    double mx = 0;
    for (float d : w.dent) mx = std::max(mx, double(d));
    printf("  %3.0f km/h: FL flat %.2f, dent %.1f mm, runout %.1f mm, attached %d, hub z %.3f, plastic %.1f kJ, sane %d\n", kmh, w.deflate, mx * 1000, std::hypot(w.runoutX, w.runoutZ) * 1000, w.attached, w.hub.z, b.sim.plasticEnergy / 1e3, b.sane());
  }
}

void testCompliance() {
  // Where does the front axle give way? Node positions in the body frame, straight against 0.7 g.
  Bench b({-120, -80, 0}, 0);
  b.world.clearBoxes();
  b.world.flatten();
  const double vT = 80 / 3.6;
  b.launchAt(vT, 3);
  b.run(1.0, [&](double, Controls& c) { c.throttle = clampd((vT - b.car.speed) * 0.5, 0, 1); });
  const Wheel& w = b.car.wheels[1];  // front right: the loaded one in a left turn
  const u32 kl = b.car.axle[0].s[1].low;
  const u32 ku = kl + 1, ks = kl + 2;
  const Beam& tie = b.sim.beams[b.car.tieBeam[1]];
  const u32 rack = tie.a == ks ? tie.b : tie.a;
  struct N { const char* name; u32 id; };
  const std::vector<N> list = {{"axle inner", w.axIn}, {"axle outer", w.axOut}, {"lower joint", kl}, {"upper joint", ku}, {"steer arm", ks}, {"rack end", rack}, {"tower", b.car.axle[0].s[1].top}, {"ring top", w.ring[0]}, {"ring 3", w.ring[3]}};
  auto rel = [&](u32 n) {
    const V3 r = b.sim.pos[n] - b.car.cog;
    return V3{dot(r, b.car.fwd), dot(r, b.car.left), dot(r, b.car.up)};
  };
  std::vector<V3> p0;
  for (const N& n : list) p0.push_back(rel(n.id));
  // Same rack position change removed: compare with the tie rod rest length change later.
  const double tie0 = tie.L0;
  b.run(3.0, [&](double, Controls& c) { c.steer = 0.075; c.throttle = clampd((vT - b.car.speed) * 0.5, 0, 1); });
  printf("== Front right corner at %.2f g, Fy %.0f, Fz %.0f: displacement in the body frame, mm ==\n", b.car.latAcc / 9.81, w.Fy, w.Fz);
  for (size_t i = 0; i < list.size(); i++) {
    const V3 d = (rel(list[i].id) - p0[i]) * 1000.0;
    printf("  %-12s dx %+6.1f dy %+6.1f dz %+6.1f\n", list[i].name, d.x, d.y, d.z);
  }
  printf("  tie rod L0 change %+.1f mm, actual length %.1f (rest %.1f), force %.0f N\n", (tie.L0 - tie0) * 1000, len(b.sim.pos[tie.b] - b.sim.pos[tie.a]) * 1000, tie.L0 * 1000, double(tie.force));
}

void testResponse() {
  printf("== Response: how direct does the car feel? ==\n");
  auto toeOf = [](Bench& b, int i) {
    const Wheel& w = b.car.wheels[i];
    const V3 wf = norm(cross(w.axis, b.car.up));
    return std::atan2(dot(wf, b.car.left), dot(wf, b.car.fwd)) * 57.3;
  };
  for (double kmh : {50.0, 80.0, 120.0}) {
    for (double st : {0.03, 0.06}) {
      Bench b({-120, -80, 0}, 0);
      b.world.clearBoxes();
      b.world.flatten();
      const double vT = kmh / 3.6;
      b.launchAt(vT, kmh > 100 ? 4 : 3);
      b.run(1.0, [&](double, Controls& c) { c.throttle = clampd((vT - b.car.speed) * 0.5, 0, 1); });
      double prevHead = std::atan2(b.car.fwd.y, b.car.fwd.x);
      std::vector<double> yaw, lat, roll, steerA;
      double toeStart = 0.5 * (toeOf(b, 0) + toeOf(b, 1));
      double toeFinal = 0;
      const int N = 6000;
      b.run(N * kDt, [&](double, Controls& c) {
        c.steer = st;
        c.throttle = clampd((vT - b.car.speed) * 0.5, 0, 1);
        const double h = std::atan2(b.car.fwd.y, b.car.fwd.x);
        yaw.push_back((h - prevHead) / kDt);
        prevHead = h;
        lat.push_back(b.car.latAcc);
        roll.push_back(b.car.rollAngle);
        steerA.push_back(b.car.steerAct);
        toeFinal = 0.5 * (toeOf(b, 0) + toeOf(b, 1));
      });
      auto smoothAt = [&](const std::vector<double>& v, int i) {
        double sum = 0;
        int n = 0;
        for (int k = std::max(0, i - 60); k <= std::min(N - 1, i + 60); k++, n++) sum += v[size_t(k)];
        return sum / n;
      };
      const double yawEnd = smoothAt(yaw, N - 200);
      double t50 = -1, t90 = -1, peak = 0;
      for (int i = 0; i < N; i += 10) {
        const double y = smoothAt(yaw, i);
        peak = std::max(peak, y);
        if (t50 < 0 && y > 0.5 * yawEnd) t50 = i * kDt;
        if (t90 < 0 && y > 0.9 * yawEnd) t90 = i * kDt;
      }
      const double rackTime = st / 2.6;
      const double commanded = st * 31.0;
      printf("  %3.0f km/h steer %.2f (rack ready %.0f ms): yaw 50%% %3.0f ms, 90%% %3.0f ms, overshoot %3.0f %%, yaw %.1f deg/s, lat %.2f g, roll %.2f deg, wheel angle %.2f of ~%.2f deg\n", kmh, st, rackTime * 1000, t50 * 1000, t90 * 1000, (peak / yawEnd - 1) * 100, yawEnd * 57.3, smoothAt(lat, N - 200) / 9.81, smoothAt(roll, N - 200) * 57.3, toeFinal - toeStart, commanded);
    }
  }
  // Throttle.
  {
    Bench b({-120, -80, 0}, 0);
    b.world.clearBoxes();
    b.world.flatten();
    b.launchAt(60 / 3.6, 3);
    b.run(1.0, [&](double, Controls& c) { c.throttle = 0.15; });
    double t63 = -1, peak = 0, a0 = b.car.lonAcc, tTorque = -1;
    std::vector<double> acc;
    b.run(1.2, [&](double t, Controls& c) {
      c.throttle = 1.0;
      acc.push_back(b.car.lonAcc);
      peak = std::max(peak, b.car.lonAcc);
      if (tTorque < 0 && b.car.wheels[0].driveT > 400) tTorque = t;
    });
    for (size_t i = 0; i < acc.size(); i++)
      if (t63 < 0 && acc[i] > a0 + 0.63 * (peak - a0)) t63 = i * kDt;
    printf("  throttle stab at 60 km/h: shaft torque after %.0f ms, 63%% of pull after %.0f ms, peak %.2f g\n", tTorque * 1000, t63 * 1000, peak / 9.81);
  }
  // Brake.
  {
    Bench b({-120, -80, 0}, 0);
    b.world.clearBoxes();
    b.world.flatten();
    b.launchAt(80 / 3.6, 3);
    b.run(1.0, [&](double, Controls& c) { c.throttle = 0.2; });
    double t63 = -1, peak = 0, dive = 0, tForce = -1;
    std::vector<double> acc;
    b.run(1.2, [&](double t, Controls& c) {
      c.brake = 1.0;
      acc.push_back(-b.car.lonAcc);
      peak = std::max(peak, -b.car.lonAcc);
      dive = std::max(dive, std::fabs(b.car.pitchAngle));
      // Direct measure, free of the telemetry filter: braking force at the tyres.
      double fx = 0;
      for (const Wheel& w : b.car.wheels) fx += w.Fx;
      if (tForce < 0 && -fx > 0.63 * 0.9 * b.car.totalMass * 9.81) tForce = t;
    });
    for (size_t i = 0; i < acc.size(); i++)
      if (t63 < 0 && acc[i] > 0.63 * peak) t63 = i * kDt;
    printf("  brake stab at 80 km/h: tyre force at 63%% after %.0f ms (filtered decel %.0f ms), peak %.2f g, dive %.2f deg\n", tForce * 1000, t63 * 1000, peak / 9.81, dive * 57.3);
  }
  // Compliance: how far do the wheels move under a steady 0.6 g, relative to the body?
  {
    Bench b({-120, -80, 0}, 0);
    b.world.clearBoxes();
    b.world.flatten();
    const double vT = 80 / 3.6;
    b.launchAt(vT, 3);
    double st0[4], st1[4];
    b.run(1.0, [&](double, Controls& c) { c.throttle = clampd((vT - b.car.speed) * 0.5, 0, 1); });
    for (int i = 0; i < 4; i++) st0[i] = toeOf(b, i);
    b.run(3.0, [&](double, Controls& c) { c.steer = 0.075; c.throttle = clampd((vT - b.car.speed) * 0.5, 0, 1); });
    for (int i = 0; i < 4; i++) st1[i] = toeOf(b, i);
    printf("  toe change at %.2f g, steer 0.075 (rack ~2.3 deg): FL %+.2f FR %+.2f RL %+.2f RR %+.2f deg, tyre slip front %.1f rear %.1f deg\n", b.car.latAcc / 9.81, st1[0] - st0[0], st1[1] - st0[1], st1[2] - st0[2], st1[3] - st0[3], std::atan(b.car.wheels[1].slipY) * 57.3, std::atan(b.car.wheels[3].slipY) * 57.3);
  }
}

void testPropRest() {
  // Regression: every prop, woken with nothing touching it, must simply stay standing.
  Bench b({-90, 0, 0}, 0, true);
  printf("== Props at rest, all awake ==\n");
  double lam = 0;
  u32 worstObj = 0;
  for (u32 oi = 0; oi < b.sim.objs.size(); oi++) {
    const double l = b.sim.maxEigenvalue(oi);
    if (l > lam) {
      lam = l;
      worstObj = oi;
    }
  }
  printf("  stability: largest eigenvalue %.3g of %.3g (object %u)\n", lam, 4.0 / (kDt * kDt), worstObj);
  for (size_t i = 1; i < b.sim.objs.size(); i++) b.sim.wake(u16(i));
  const u32 before = b.sim.brokenBeams;
  auto t0 = std::chrono::steady_clock::now();
  b.run(1.0, [](double, Controls&) {});
  const double ms = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  double vm = 0;
  for (size_t i = 0; i < b.sim.vel.size(); i++) vm = std::max(vm, len(b.sim.vel[i]));
  for (size_t oi = 1; oi < b.sim.objs.size(); oi++) {
    const Obj& O = b.sim.objs[oi];
    double vo = 0;
    for (u32 i = O.nodeStart; i < O.nodeStart + O.nodeCount; i++) vo = std::max(vo, len(b.sim.vel[i]));
    int br = 0;
    for (u32 i = O.beamStart; i < O.beamStart + O.beamCount; i++) br += (b.sim.beams[i].flags & BF_BROKEN) ? 1 : 0;
    double under = 0;
    for (u32 i = O.nodeStart; i < O.nodeStart + O.nodeCount; i++) under = std::max(under, b.world.height(b.sim.rest[i].x, b.sim.rest[i].y) - b.sim.rest[i].z);
    if (br > 0 || vo > 0.5)
      printf("   obj %zu nodes %u beams %u broken %d vmax %.1f at (%.0f %.0f) r=%.0f max rest-depth below terrain %.3f\n", oi, O.nodeCount, O.beamCount, br, vo, b.sim.rest[O.nodeStart].x, b.sim.rest[O.nodeStart].y, std::hypot(b.sim.rest[O.nodeStart].x, b.sim.rest[O.nodeStart].y), under);
  }
  printf("  velocity guard activations %llu\n", (unsigned long long)b.sim.clampedSteps);
  printf("  after 1 s: broken %u  vmax %.3f m/s  still awake %zu of %zu  (%.1f s of wall time)\n", b.sim.brokenBeams - before, vm, b.sim.active.size(), b.sim.objs.size(), ms);
}

void testPerf() {
  Bench b;
  auto t0 = std::chrono::steady_clock::now();
  b.run(10.0, [](double, Controls& c) { c.throttle = 0.3; });
  const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  printf("== Performance ==\n  car alone: %.0f us per step, %.1f %% of one core at 2 kHz\n", s / 20000 * 1e6, s / 10.0 * 100);
}
}  // namespace

int main(int argc, char** argv) {
  auto want = [&](const char* n) {
    if (argc < 2) return true;
    for (int i = 1; i < argc; i++)
      if (!strcmp(argv[i], n)) return true;
    return false;
  };
  if (argc > 1 && !strcmp(argv[1], "diag")) {
    testDiag();
    return 0;
  }
  if (want("statics")) testStatics();
  if (want("bounce")) testBounce();
  if (want("accel")) testAccel();
  if (want("brake")) testBraking();
  if (want("skidpad")) {
    testSteerRamp(true);
    testSteerRamp(false);
    printf("== Evasive manoeuvre ==\n");
    for (double kmh : {60.0, 75.0, 85.0}) {
      testMoose(true, kmh);
      testMoose(false, kmh);
    }
  }
  if (want("crash")) {
    testCrash(50);
    testCrash(100);
  }
  if (want("jitter")) {
    testJitter("standstill", 0);
    testJitter("60 km/h", 60);
    testJitter("120 km/h", 120);
  }
  if (want("wheel")) {
    testWheelDamage();
    testWheelStrike();
  }
  if (want("compliance")) testCompliance();
  if (want("response")) testResponse();
  if (want("powertrain")) testPowertrain();
  if (want("rough")) testRough();
  if (want("rest")) testPropRest();
  if (want("props")) testProps();
  if (want("perf")) testPerf();
  return 0;
}
