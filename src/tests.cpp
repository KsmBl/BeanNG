// Headless test bench: drives the real simulation through scripted manoeuvres and prints the
// numbers, so the physics can be compared with the real car rather than judged by eye.
#include <chrono>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <array>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>
#include "bot.h"
#include "duel.h"
#include "props.h"
#include "tank.h"
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
  const Obj o = b.sim.objs[b.car.obj];  // a copy: tearing grows the list and moves its entries
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

// How easily the wheels fold: camber and toe of each wheel against the body, before and after abuse.
void wheelAttitude(const Bench& b, double out[4][2]) {
  for (int i = 0; i < 4; i++) {
    const Wheel& w = b.car.wheels[i];
    const V3 ax = norm(b.sim.pos[w.axOut] - b.sim.pos[w.axIn]);  // towards the outside
    const double sg = w.left ? 1.0 : -1.0;
    out[i][0] = std::asin(clampd(dot(ax, b.car.up), -1, 1)) * 57.3 * sg;
    out[i][1] = std::asin(clampd(dot(ax, b.car.fwd), -1, 1)) * 57.3;
  }
}
void reportBuckle(const char* name, Bench& b, double a0[4][2], double peakFz) {
  b.run(4.0, [&](double, Controls& c) { c.throttle = 0; c.steer = 0; c.brake = 1; });
  double a[4][2];
  wheelAttitude(b, a);
  double dent = 0, run = 0, dc = 0, dt = 0;
  int flat = 0;
  for (int i = 0; i < 4; i++) {
    const Wheel& w = b.car.wheels[i];
    for (float d : w.dent) dent = std::max(dent, double(d));
    run = std::max(run, double(std::hypot(w.runoutX, w.runoutZ)));
    flat += w.deflate > 0;
    dc = std::max(dc, std::fabs(a[i][0] - a0[i][0]));
    dt = std::max(dt, std::fabs(a[i][1] - a0[i][1]));
  }
  int yHub = 0, yArm = 0;
  const Obj& o = b.sim.objs[b.car.obj];
  for (const Wheel& w : b.car.wheels)
    for (int i = 0; i < 6; i++)
      for (int k = 0; k < 2; k++) yHub += std::fabs(b.sim.beams[w.spoke[i][k]].L0 - b.sim.beams[w.spoke[i][k]].Lorig) > 2e-4;
  (void)o;
  (void)yArm;
  printf("  %-26s peak Fz %5.1f kN  dent %5.1f mm  runout %4.1f mm  flat %d  camber +%4.1f  toe +%4.1f deg  yielded spokes %2d  plastic %6.1f kJ\n", name, peakFz / 1e3, dent * 1e3, run * 1e3, flat, dc, dt, yHub, b.sim.plasticEnergy / 1e3);
}
void testBuckle() {
  printf("== How easily do the wheels fold? ==\n");
  auto peak = [](Bench& b, double& p) { for (const Wheel& w : b.car.wheels) p = std::max(p, w.Fz); };
  for (double h : {0.3, 0.6, 1.0, 1.5, 2.5}) {
    Bench b;
    b.world.clearBoxes();
    double a0[4][2];
    wheelAttitude(b, a0);
    const Obj& o = b.sim.objs[b.car.obj];
    for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) b.sim.pos[i].z += h;
    double p = 0;
    b.run(2.5, [&](double, Controls&) { peak(b, p); });
    char nm[64];
    snprintf(nm, sizeof nm, "drop %.1f m", h);
    reportBuckle(nm, b, a0, p);
  }
  // Speed bumps at y = 70, in the main speed range.
  for (double kmh : {30.0, 60.0, 100.0}) {
    Bench b({-60, 70, 0}, 0);
    double a0[4][2];
    wheelAttitude(b, a0);
    b.launchAt(kmh / 3.6, kmh > 60 ? 3 : 2);
    double p = 0;
    b.run(5.0, [&](double, Controls& c) { c.throttle = 0.25; peak(b, p); });
    char nm[64];
    snprintf(nm, sizeof nm, "bars %.0f km/h", kmh);
    reportBuckle(nm, b, a0, p);
  }
  // Long kerb at y = 95, crossed at an angle of about 25 degrees.
  for (double kmh : {25.0, 50.0, 90.0}) {
    Bench b({-46, 91.0, 0}, 0.45);
    double a0[4][2];
    wheelAttitude(b, a0);
    b.launchAt(kmh / 3.6, 2);
    double p = 0;
    b.run(1.2, [&](double, Controls&) { peak(b, p); });
    char nm[64];
    snprintf(nm, sizeof nm, "kerb %.0f km/h, 25 deg", kmh);
    reportBuckle(nm, b, a0, p);
  }
  // Flank slide: the whole car thrown sideways into the kerb, the classic way to bend a wheel.
  for (double kmh : {10.0, 20.0, 35.0, 60.0}) {
    Bench b({-20, 92.5, 0}, 0);
    double a0[4][2];
    wheelAttitude(b, a0);
    const Obj& o = b.sim.objs[b.car.obj];
    for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) b.sim.vel[i] = V3{0, kmh / 3.6, 0};
    double p = 0;
    b.run(1.5, [&](double, Controls&) { peak(b, p); });
    char nm[64];
    snprintf(nm, sizeof nm, "side into kerb %.0f km/h", kmh);
    reportBuckle(nm, b, a0, p);
  }
  // Hard cornering and braking must not move anything.
  {
    Bench b;
    b.world.clearBoxes();
    b.world.flatten();
    double a0[4][2];
    wheelAttitude(b, a0);
    b.launchAt(80 / 3.6, 3);
    double p = 0;
    b.run(4.0, [&](double t, Controls& c) { c.steer = t < 3 ? 0.3 : -0.5; c.brake = t > 3 ? 1 : 0; peak(b, p); });
    reportBuckle("hard corner and brake", b, a0, p);
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

void testFreeFall() {
  // A stone dropped from 20 m must land after about 2 s, at about 20 m/s.
  Bench b({-90, 0, 0}, 0, true);
  u32 blockObj = b.sim.nodeObj[b.sim.blocks[0].n[0]];
  b.sim.wake(u16(blockObj));
  const u32 n0 = b.sim.blocks[0].n[0];
  for (u32 oi = b.sim.objs[blockObj].nodeStart, e = oi + b.sim.objs[blockObj].nodeCount; oi < e; oi++) b.sim.pos[oi].z += 20.0;
  double maxV = 0, t = 0;
  while (t < 3.0 && b.sim.pos[n0].z > 1.0) {
    b.run(0.01, [](double, Controls&) {});
    t += 0.01;
    maxV = std::max(maxV, -b.sim.vel[n0].z);
  }
  printf("== Free fall from 20 m: hit after %.2f s at %.1f m/s (ideal 2.02 s, 19.8 m/s) ==\n", t, maxV);
}

void testSplit() {
  printf("== Objects tearing into independent pieces ==\n");
  {
    // A sign post cut through its middle: two objects, the top falls, the base stays.
    Bench b({-90, 0, 0}, 0, true);
    u32 sign = 1;
    b.sim.wake(u16(sign));
    const size_t before = b.sim.objs.size();
    const Obj& o = b.sim.objs[sign];
    double zCut = 1.2;
    u32 topNode = 0;
    for (u32 n : o.nodes)
      if (b.sim.rest[n].z > 1.5) topNode = n;
    u32 baseNode = o.nodes[3];
    for (u32 bi : std::vector<u32>(o.beamList)) {
      Beam& B = b.sim.beams[bi];
      const double za = b.sim.rest[B.a].z, zb = b.sim.rest[B.b].z;
      if ((za < zCut) != (zb < zCut)) {
        B.flags |= BF_BROKEN;
        b.sim.objs[sign].dirty = true;
      }
    }
    const double zTop0 = b.sim.pos[topNode].z, zBase0 = b.sim.pos[baseNode].z;
    b.run(1.5, [](double, Controls&) {});
    printf("  sign cut: objects %zu -> %zu; top in object %u fell %.2f m, base in object %u moved %.3f m; roots %u %u\n", before, b.sim.objs.size(), b.sim.nodeObj[topNode], zTop0 - b.sim.pos[topNode].z, b.sim.nodeObj[baseNode], zBase0 - b.sim.pos[baseNode].z, b.sim.objs[b.sim.nodeObj[topNode]].root, b.sim.objs[b.sim.nodeObj[baseNode]].root);
    b.run(4.0, [](double, Controls&) {});
    printf("  after settling: awake %zu of %zu, top z %.2f (lying on the ground, not sunk: %d)\n", b.sim.active.size(), b.sim.objs.size(), b.sim.pos[topNode].z, b.sim.pos[topNode].z > -0.02);
  }
  {
    // The piece collides with what it came from: a block of wall dropped on the rest of the wall,
    // and the fence rail on the car.
    Bench b({-90, 0, 0}, 0, true);
    b.launchAt(70 / 3.6, 3);
    size_t peak = 0;
    double next = 0;
    auto t0 = std::chrono::steady_clock::now();
    b.run(16.0, [&](double t, Controls& c) {
      c.throttle = b.car.speed < 70 / 3.6 ? 0.8 : 0;
      const double head = std::atan2(b.car.fwd.y, b.car.fwd.x);
      c.steer = clampd(-0.05 * b.car.cog.y - 0.7 * head, -0.25, 0.25);
      peak = std::max(peak, b.sim.active.size());
      if (t >= next) {
        next += 4.0;
        printf("  t=%4.1f x=%6.1f v=%5.1f objects %4zu awake %3zu car nodes %zu of %u broken %u\n", t, b.car.cog.x, b.car.speed * 3.6, b.sim.objs.size(), b.sim.active.size(), b.sim.objs[0].nodes.size(), b.sim.objs[0].nodeCount, b.sim.brokenBeams);
      }
    });
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    printf("  gauntlet: %zu objects in the end, peak awake %zu, %.0f %% of real time, guard %llu, sane %d\n", b.sim.objs.size(), peak, wall / 16.0 * 100, (unsigned long long)b.sim.clampedSteps, b.sane());
  }
  {
    // Worst case for the visible mesh: the body cut in two through the middle.
    Bench b;
    CarMesh mesh;
    mesh.load("assets/xantia.mesh");
    mesh.bind(b.sim.rest, b.skin.groups, {-90, 0, 0});
    const double cut = -90.0 + 2.26 - 2.2;
    for (u32 bi : std::vector<u32>(b.sim.objs[0].beamList)) {
      Beam& B = b.sim.beams[bi];
      if ((b.sim.rest[B.a].x < cut) != (b.sim.rest[B.b].x < cut)) B.flags |= BF_BROKEN;
    }
    b.sim.objs[0].dirty = true;
    b.run(1.5, [](double, Controls&) {});
    const size_t changed = mesh.rebindAfterTear(b.sim.rest, b.skin.groups, {-90, 0, 0}, b.sim.nodeObj);
    std::string worst;
    const double err = mesh.restError(b.sim.rest, {-90, 0, 0}, &worst);
    size_t straddle = 0;
    constexpr int K = CarMesh::kK;
    for (size_t v = 0; v < mesh.restPos.size() / 3; v++) {
      u16 first = 0xffff;
      for (int k = 0; k < K; k++)
        if (mesh.bindWeight[v * K + k] != 0.f) {
          const u16 o = b.sim.nodeObj[mesh.bindNode[v * K + k]];
          if (first == 0xffff) first = o;
          else if (o != first) {
            straddle++;
            break;
          }
        }
    }
    printf("  body cut in two: %zu objects, %zu vertices re-bound, %zu still straddling, max rest error %.1f mm (%s), guard %llu, sane %d\n", b.sim.objs.size(), changed, straddle, err * 1000, worst.c_str(), (unsigned long long)b.sim.clampedSteps, b.sane());
  }
  {
    // Crash hard enough to lose parts of the car.
    Bench b({60, -30, 0}, 0);
    CarMesh mesh;
    mesh.load("assets/xantia.mesh");
    mesh.bind(b.sim.rest, b.skin.groups, {60, -30, 0});
    b.launchAt(170 / 3.6, 4);
    b.run(4.0, [](double, Controls&) {});
    size_t detached = 0;
    for (size_t i = 1; i < b.sim.objs.size(); i++) detached += b.sim.objs[i].nodes.size();
    const size_t changed = mesh.rebindAfterTear(b.sim.rest, b.skin.groups, {60, -30, 0}, b.sim.nodeObj);
    std::string worst;
    const double err = mesh.restError(b.sim.rest, {60, -30, 0}, &worst);
    printf("  wall at 170 km/h: car split into %zu objects, %zu nodes came off, %zu mesh vertices re-bound, mesh rest error %.2f mm (%s), sane %d, guard %llu\n", b.sim.objs.size(), detached, changed, err * 1000, worst.c_str(), b.sane(), (unsigned long long)b.sim.clampedSteps);
  }
}

void testWinding() {
  CarMesh m;
  m.load("assets/xantia.mesh");
  printf("== Triangle winding against stored normals, outer parts ==\n");
  int bad = 0;
  for (const MeshPart& p : m.parts) {
    double area = 0, wrongArea = 0;
    for (u32 t = p.firstTri; t < p.firstTri + p.triCount; t++) {
      const u32 ia = m.tris[t * 3], ib = m.tris[t * 3 + 1], ic = m.tris[t * 3 + 2];
      auto P = [&](u32 v) { return V3{m.restPos[v * 3], m.restPos[v * 3 + 1], m.restPos[v * 3 + 2]}; };
      auto N = [&](u32 v) { return V3{m.restNormal[v * 3], m.restNormal[v * 3 + 1], m.restNormal[v * 3 + 2]}; };
      const V3 fn = cross(P(ib) - P(ia), P(ic) - P(ia));
      const double A = len(fn);
      area += A;
      if (dot(fn, N(ia) + N(ib) + N(ic)) < 0) wrongArea += A;
    }
    if (wrongArea > 0.02 * area) {
      bad++;
      printf("  %-20s group %-10s mat %2d: %.0f %% of its area wound against its normals\n", p.name.c_str(), p.group.c_str(), p.mat, 100 * wrongArea / area);
    }
  }
  printf("  %d of %zu parts wrong\n", bad, m.parts.size());
}

void testTorn() {
  printf("== Parts torn off and thrown far away ==\n");
  Bench b({-90, 0, 0}, 0, true);
  auto measure = [&](const char* label) {
    auto t0 = std::chrono::steady_clock::now();
    b.run(0.25, [](double, Controls&) {});
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    printf("  %-34s load %6.0f %%  tripairs %6zu awake %zu\n", label, wall / 0.25 * 100, b.sim.statTriPairs, b.sim.active.size());
  };
  measure("intact");
  // Throw everything light from the front and rear of the car away, as a big crash does.
  const Obj& o = b.sim.objs[b.car.obj];
  for (u32 i = o.beamStart; i < o.beamStart + o.beamCount; i++) {
    Beam& B = b.sim.beams[i];
    const bool aHood = b.sim.mass[B.a] < 1.35, bHood = b.sim.mass[B.b] < 1.35;
    if (aHood != bHood) B.flags |= BF_BROKEN;
  }
  int thrown = 0;
  for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++)
    if (b.sim.mass[i] < 1.35) {
      b.sim.pos[i] += V3{35.0, 25.0, 3.0};
      thrown++;
    }
  printf("  threw %d panel nodes 40 m away\n", thrown);
  measure("panels thrown");
  measure("panels thrown, later");
  for (u32 oi = 1; oi < b.sim.objs.size(); oi++) b.sim.wake(u16(oi));
  measure("everything awake");
  b.sim.vel[o.nodeStart + 300] = V3{240, 0, 0};
  measure("one node at 240 m/s");
  for (int i = 0; i < 6; i++) b.run(1.0, [](double, Controls&) {});
  measure("after 6 s of settling");
  for (int i = 0; i < 10; i++) b.run(1.0, [](double, Controls&) {});
  measure("after 16 s of settling");
}

void testSlow() {
  printf("== Cost per simulated second in violent scenes (100 %% = real time) ==\n");
  struct Case { const char* name; V3 pos; double kmh; };
  const Case cases[] = {{"gauntlet 140 km/h", {-90, 0, 0}, 140}, {"wall 180 km/h", {60, -30, 0}, 180}, {"ramp into wall 160 km/h", {-30, 40, 0}, 160}};
  for (const Case& cs : cases) {
    Bench b(cs.pos, 0, true);
    b.launchAt(cs.kmh / 3.6, 4);
    printf("  %s\n", cs.name);
    for (int sec = 0; sec < 8; sec++) {
      auto t0 = std::chrono::steady_clock::now();
      b.run(1.0, [&](double, Controls& c) { c.throttle = sec < 5 ? 1.0 : 0.0; });
      const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      double vmax = 0, zmax = 0, far = 0;
      for (size_t i = 0; i < b.sim.pos.size(); i++) {
        vmax = std::max(vmax, len(b.sim.vel[i]));
        zmax = std::max(zmax, b.sim.pos[i].z);
        far = std::max(far, std::fabs(b.sim.pos[i].x));
      }
      printf("   t=%d load %5.0f %%  awake %3zu  tripairs %6zu blockpairs %6zu  vmax %5.1f zmax %6.1f farx %6.0f broken %5u  guard %llu  v %.0f\n", sec + 1, wall * 100, b.sim.active.size(), b.sim.statTriPairs, b.sim.statBlockPairs, vmax, zmax, far, b.sim.brokenBeams, (unsigned long long)b.sim.clampedSteps, b.car.speed * 3.6);
      if (wall > 6.0) {
        printf("   (too slow, stopping this case)\n");
        break;
      }
    }
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
  if (argc > 1 && !strcmp(argv[1], "tank")) {
    printf("== Leopard 1 ==\n");
    auto make = [](World& w, Sim& s, CarRig& r, V3 at) {
      w.build();
      w.clearBoxes();
      w.flatten();
      s.world = &w;
      Tank t = buildLeopard(s, at, &r);
      s.finalize();
      t.settle(s, 4.0);
      return t;
    };
    auto* w = new World();
    auto* s = new Sim();
    CarRig r;
    Tank t = make(*w, *s, r, V3{-120, 0, 0});
    printf("  rest: hull bottom z %.3f, wheel Fn %.1f kN, guard %llu, largest eigen ratio %.2f\n", s->pos[t.refBottom].z, t.wheels[3].Fn / 1e3, (unsigned long long)s->clampedSteps, s->maxEigenvalue(0) * 0.25 * kDt * kDt);
    auto run = [&](double sec, std::function<void(double, Controls&)> fn) {
      Controls c;
      for (int i = 0, n = int(sec / kDt); i < n; i++) {
        c.fire = false;
        c.requestGear = 99;
        c.shiftUp = c.shiftDown = false;
        fn(i * kDt, c);
        s->forces(kDt);
        t.update(*s, c, kDt);
        s->finish(kDt);
      }
      s->events.clear();
    };
    double t0to30 = -1, t0to50 = -1;
    double tNow = 0;
    run(30.0, [&](double, Controls& c) {
      c.throttle = 1;
      tNow += kDt;
      if (t0to30 < 0 && t.speed * 3.6 > 30) t0to30 = tNow;
      if (t0to50 < 0 && t.speed * 3.6 > 50) t0to50 = tNow;
    });
    printf("  0-30 km/h %.1f s, 0-50 km/h %.1f s, after 30 s %.1f km/h gear %d rpm %.0f (real: 0-60 about 20 s, top 65 km/h), y drift %.2f m\n", t0to30, t0to50, t.speed * 3.6, t.gear, t.rpm(), t.cog.y);
    run(6.0, [&](double, Controls& c) { c.brake = 1; });
    printf("  stopped from there: %.1f km/h\n", t.speed * 3.6);
    {
      // Foot off at walking pace, and a push when standing: it should stay where it is.
      run(1.5, [&](double, Controls& c) { c.throttle = 0.6; });
      const double rolling = t.speed * 3.6;
      const V3 from = t.cog;
      run(8.0, [&](double, Controls&) {});
      const V3 stopped = t.cog;
      for (u32 n : s->objs[t.hullObj].nodes) s->vel[n] += t.fwd * 1.0;
      run(4.0, [&](double, Controls&) {});
      double warm = 0;
      for (int i = 0; i < 20; i++) warm = warmthStep(warm, true, true, 1.0);
      printf("  warmth: driven 20 s %.2f", warm);
      for (int i = 0; i < 60; i++) warm = warmthStep(warm, true, false, 1.0);
      printf(", standing with a driver 1 min %.2f", warm);
      warm = 0.85;
      for (int i = 0; i < 60; i++) warm = warmthStep(warm, false, false, 1.0);
      printf(", from driving, left alone 1 min %.2f", warm);
      for (int i = 0; i < 240; i++) warm = warmthStep(warm, false, false, 1.0);
      printf(", 5 min %.2f\n", warm);
      printf("  coasting from %.1f km/h: rolled %.1f m; a push of 1 m/s: moved %.2f m\n", rolling, len(stopped - from), len(t.cog - stopped));
    }
    const double h0 = std::atan2(t.fwd.y, t.fwd.x);
    run(4.0, [&](double, Controls& c) { c.steer = 1; });
    printf("  pivot turn: %.0f deg in 4 s, moved %.1f km/h\n", (std::atan2(t.fwd.y, t.fwd.x) - h0) * 57.3, t.speed * 3.6);
    double pitchMax = 0;
    const V3 aim = t.cog + V3{500, 0, 3};
    run(3.0, [&](double, Controls& c) { c.steer = 0; c.aimPoint = aim; });
    const double p0 = std::asin(t.fwd.z);
    bool shot = true;
    run(1.5, [&](double, Controls& c) { c.aimPoint = aim; c.fire = shot; shot = false; pitchMax = std::max(pitchMax, std::fabs(std::asin(t.fwd.z) - p0)); });
    printf("  gun fired: turret yaw %.0f deg, hull rocks %.2f deg, shells in flight %zu, guard %llu, plastic %.1f kJ\n", t.turretYaw * 57.3, pitchMax * 57.3, t.shells.size(), (unsigned long long)s->clampedSteps, s->plasticEnergy / 1e3);
    run(5.0, [&](double, Controls& c) { c.aimPoint = aim; });
    printf("  hull after 5 s: bottom z %.3f, turret off target %.3f m, nodes sane %d\n", s->pos[t.refBottom].z, len(s->pos[t.turretNodes[0]] - s->pos[t.refFL]), std::isfinite(s->pos[t.turretNodes[0]].x));
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "one1drive")) {
    // What a driver does: full throttle with steering, at various speeds, on the real uneven road.
    auto run = [&](const char* name, double kmh, double steerAmp, double steerT, bool aids) {
      auto* w = new World();
      auto* s = new Sim();
      CarRig r;
      w->build();
      w->clearBoxes();
      const RoadPoint& p = w->roads()[0].pts[1200];
      s->world = w;
      Vehicle c0 = buildOne1(*s, {p.x, p.y, w->height(p.x, p.y)}, &r);
      s->finalize();
      settleVehicle(*s, c0, 3.0);
      c0.tune.esp = c0.tune.tcs = aids;
      const Obj& o = s->objs[0];
      for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) s->vel[i] = V3{kmh / 3.6, 0, 0};
      int g = 1;
      while (g < 7 && kmh / 3.6 / c0.wheels[0].R * c0.spec.gears[g - 1] * c0.spec.finalDrive * 9.55 > 5500) g++;
      c0.presetRolling(kmh / 3.6, g);
      Controls c;
      const double v0 = 0;
      (void)v0;
      double thSum = 0, thMin = 1, clutchMin = 1, wheelOver = 0, fzMin = 1e9;
      int esp = 0, tcs = 0, shifts = 0, lastGear = g, n = 0, flips = 0;
      double prevTh = 1;
      const double vStart = kmh;
      for (int i = 0; i < 2000 * 4; i++) {
        const double t = i * kDt;
        c.throttle = 1;
        c.steer = steerAmp * std::sin(2 * PI * t / steerT);
        s->forces(kDt);
        c0.update(*s, c, kDt);
        s->finish(kDt);
        if (t < 0.3) continue;
        n++;
        thSum += c0.throttleAct;
        thMin = std::min(thMin, c0.throttleAct);
        clutchMin = std::min(clutchMin, c0.clutch);
        esp += c0.espActive;
        tcs += c0.tcsActive;
        if (c0.gear != lastGear) {
          shifts++;
          lastGear = c0.gear;
        }
        if (std::fabs(c0.throttleAct - prevTh) > 0.002 && i % 20 == 0) flips++;
        if (i % 20 == 0) prevTh = c0.throttleAct;
        for (int k = 2; k < 4; k++) {
          wheelOver = std::max(wheelOver, c0.wheels[size_t(k)].omega * c0.wheels[size_t(k)].R - c0.speed);
          fzMin = std::min(fzMin, c0.wheels[size_t(k)].Fz);
        }
      }
      printf("  %-26s %3.0f->%3.0f km/h  gas avg %.2f min %.2f  clutch min %.2f  ESP %3.0f%% ASR %3.0f%%  shifts %d  rear overspeed %.1f m/s  rear Fz min %.0f N  gas changes %d\n", name, vStart, c0.speed * 3.6, thSum / n, thMin, clutchMin, 100.0 * esp / n, 100.0 * tcs / n, shifts, wheelOver, fzMin, flips);
    };
    for (bool aids : {true, false}) {
      printf(aids ? "Mit ESP und ASR:\n" : "Ohne:\n");
      run("geradeaus", 40, 0.0, 1, aids);
      run("geradeaus", 90, 0.0, 1, aids);
      run("sanft lenken", 40, 0.10, 3, aids);
      run("sanft lenken", 90, 0.05, 3, aids);
      run("stark lenken", 40, 0.30, 3, aids);
      run("stark lenken", 90, 0.12, 3, aids);
    }
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "one1hop")) {
    // Rolling gently along the real road: is the suspension calm?
    auto* w = new World();
    auto* s = new Sim();
    CarRig r;
    w->build();
    w->clearBoxes();
    const RoadPoint& p = w->roads()[0].pts[1200];
    s->world = w;
    Vehicle c0 = buildOne1(*s, {p.x, p.y, w->height(p.x, p.y)}, &r);
    s->finalize();
    settleVehicle(*s, c0, 3.0);
    printf("rest: rear L %.4f (nom %.4f min %.4f max %.4f) p %.0f bar Vf %.1f cc, F %.0f N, hub z %.3f, road %.3f\n", c0.axle[1].s[0].L, c0.axle[1].s[0].Lnom, c0.axle[1].s[0].Lmin, c0.axle[1].s[0].Lmax, c0.axle[1].s[0].pGas / 1e5, c0.axle[1].s[0].Vf * 1e6, c0.axle[1].s[0].F, c0.wheels[2].hub.z, w->height(c0.wheels[2].hub.x, c0.wheels[2].hub.y));
    const Obj& o = s->objs[0];
    for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) s->vel[i] = V3{60 / 3.6, 0, 0};
    c0.presetRolling(60 / 3.6, 3);
    Controls c;
    double fzMin = 1e9, fzMax = 0;
    c0.tune.esp = c0.tune.tcs = false;
    for (int i = 0; i < 2000 * 3; i++) {
      c.throttle = 1.0;
      s->forces(kDt);
      c0.update(*s, c, kDt);
      s->finish(kDt);
      fzMin = std::min(fzMin, c0.wheels[2].Fz);
      fzMax = std::max(fzMax, c0.wheels[2].Fz);
      if (i % 100 == 99) {
        const HydroStrut& st = c0.axle[1].s[0];
        printf(" t %.2f v %3.0f | RL Fz %5.0f..%5.0f pen hub z %.3f road %.3f | L %.3f F %5.0f p %3.0f Vf %.1f surf %d | om*R %5.1f gear %d rpm %.0f th %.2f | FL Fz %.0f Fx RL %.0f roll %.1f pitch %.1f y %.1f\n", (i + 1) * kDt, c0.speed * 3.6, fzMin, fzMax, c0.wheels[2].hub.z, w->height(c0.wheels[2].hub.x, c0.wheels[2].hub.y), st.L, st.F, st.pGas / 1e5, st.Vf * 1e6, c0.wheels[2].surface, c0.wheels[2].omega * c0.wheels[2].R, c0.gear, c0.rpm(), c0.throttleAct, c0.wheels[0].Fz, c0.wheels[2].Fx, c0.rollAngle * 57.3, c0.pitchAngle * 57.3, c0.cog.y);
        fzMin = 1e9;
        fzMax = 0;
      }
    }
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "one1road")) {
    // A driver following the real country road at full throttle up to a speed, steering all the way.
    for (double cap : {80.0, 130.0, 180.0}) {
      auto* w = new World();
      auto* s = new Sim();
      CarRig r;
      w->build();
      s->world = w;
      const auto& pts = w->roads()[0].pts;
      // Find a stretch that heads along +x, as the car is built.
      size_t start = 0;
      double bestH = 9;
      for (size_t i = 0; i < pts.size(); i++) {
        const auto& a = pts[i];
        const auto& b = pts[(i + 3) % pts.size()];
        const double h = std::fabs(std::atan2(b.y - a.y, b.x - a.x));
        if (h < bestH) {
          bestH = h;
          start = i;
        }
      }
      const RoadPoint& p = pts[start];
      // Built and settled on the level pad, as in the game, then moved onto the road in one piece. Built
      // straight onto a bank it would be bent before it had even started.
      const V3 pad{-90, 0, 0};
      Vehicle c0 = buildOne1(*s, pad, &r);
      populateWorld(*s, *w);
      s->finalize();
      settleVehicle(*s, c0, 3.0);
      {
        const auto& b = pts[(start + 3) % pts.size()];
        const double yaw = std::atan2(b.y - p.y, b.x - p.x), cy = std::cos(yaw), sy = std::sin(yaw);
        const V3 at{p.x, p.y, w->height(p.x, p.y) + 0.14};
        const Obj& oo = s->objs[0];
        for (u32 k = oo.nodeStart; k < oo.nodeStart + oo.nodeCount; k++) {
          const V3 q = s->pos[k] - pad;
          s->pos[k] = at + V3{cy * q.x - sy * q.y, sy * q.x + cy * q.y, q.z};
          s->vel[k] = {};
        }
      }
      {
        Controls hold;
        hold.brake = 1;
        for (int k = 0; k < 2000; k++) {
          s->forces(kDt);
          c0.update(*s, hold, kDt);
          s->finish(kDt);
        }
        printf("  auf der Strasse: Radlast FL %.0f FR %.0f RL %.0f RR %.0f, verformt %.1f J\n", c0.wheels[0].Fz, c0.wheels[1].Fz, c0.wheels[2].Fz, c0.wheels[3].Fz, s->plasticEnergy);
      }
      Controls c;
      double thSum = 0, offRoad = 0, airborne = 0, overMax = 0, steerSum = 0, offMax = 0;
      int esp = 0, tcs = 0, shifts = 0, last = c0.gear, n = 0, stutter = 0;
      double prevSpeed = 0, prevAcc = 0;
      double reached = 0, tReach = -1;
      for (int i = 0; i < 2000 * 22; i++) {
        size_t best = 0;
        double bd = 1e18;
        if (i % 8 == 0 || true) {
          for (size_t k = 0; k < pts.size(); k += 3) {
            const double d = std::hypot(pts[k].x - c0.cog.x, pts[k].y - c0.cog.y);
            if (d < bd) {
              bd = d;
              best = k;
            }
          }
        }
        const RoadPoint& g = pts[(best + size_t((10.0 + std::fabs(c0.speed) * 0.45) / 1.4)) % pts.size()];
        double err = std::atan2(g.y - c0.cog.y, g.x - c0.cog.x) - std::atan2(c0.fwd.y, c0.fwd.x);
        while (err > PI) err -= 2 * PI;
        while (err < -PI) err += 2 * PI;
        c.steer = clampd(err * 1.5, -0.45, 0.45);
        const double sp = c0.speed * 3.6;
        // Lifts for the tight bits, as anyone would.
        c.throttle = sp < cap ? (std::fabs(c.steer) > 0.25 && sp > 70 ? 0.35 : 1.0) : 0.25;
        c.brake = sp > cap + 8 ? 0.5 : 0.0;
        s->forces(kDt);
        c0.update(*s, c, kDt);
        s->finish(kDt);
        reached = std::max(reached, sp);
        if (tReach < 0 && sp > cap) tReach = i * kDt;
        n++;
        thSum += c0.throttleAct;
        esp += c0.espActive;
        tcs += c0.tcsActive;
        if (c0.gear != last) {
          shifts++;
          last = c0.gear;
        }
        offRoad = std::max(offRoad, bd);
        offMax += bd > 5.0;
        if (c0.wheels[2].Fz < 50 || c0.wheels[3].Fz < 50) airborne += 1;
        overMax = std::max(overMax, std::max(c0.wheels[2].omega * c0.wheels[2].R, c0.wheels[3].omega * c0.wheels[3].R) - c0.speed);
        steerSum += std::fabs(c.steer);
        if (cap < 100 && i > 2000 && i % 1200 == 0) {
          double low = 9;
          u32 lowN = 0;
          const Obj& oo = s->objs[0];
          for (u32 k = oo.nodeStart; k < oo.nodeStart + oo.nodeCount; k++)
            if (s->flags[k] & NF_COLLIDE && !(s->flags[k] & NF_WHEEL)) {
              const double cl = s->pos[k].z - w->height(s->pos[k].x, s->pos[k].y);
              if (cl < low) {
                low = cl;
                lowN = k;
              }
            }
          printf("   t %.1f v %3.0f g %d FL %5.0f FR %5.0f RL %5.0f RR %5.0f | Fx RL %5.0f RR %5.0f | om*R-v %5.1f %5.1f | roll %+.1f pitch %+.1f | Boden %.3f (Knoten-Design x %.2f y %.2f z %.2f) surf %d off %.1f th %.2f\n", i * kDt, sp, c0.gear, c0.wheels[0].Fz, c0.wheels[1].Fz, c0.wheels[2].Fz, c0.wheels[3].Fz, c0.wheels[2].Fx, c0.wheels[3].Fx, c0.wheels[2].omega * c0.wheels[2].R - c0.speed, c0.wheels[3].omega * c0.wheels[3].R - c0.speed, c0.rollAngle * 57.3, c0.pitchAngle * 57.3, low, 2.25 - (s->rest[lowN].x + 90), s->rest[lowN].y, s->rest[lowN].z - s->rest[oo.nodeStart].z, c0.wheels[2].surface, bd, c0.throttleAct);
        }
        if (i % 40 == 39) {
          // Longitudinal acceleration changing sign quickly while on the gas: that is stutter.
          const double acc = (c0.speed - prevSpeed) / (40 * kDt);
          if (sp < cap - 5 && c.throttle > 0.9 && prevAcc > 1.5 && acc < 0.3) stutter++;
          prevAcc = acc;
          prevSpeed = c0.speed;
        }
      }
      printf("  bis %3.0f km/h: erreicht nach %5.1f s, max %3.0f | Gas %.2f  ESP %3.0f%%  ASR %3.0f%%  Schaltungen %d | Hinterrad ohne Boden %.1f%%  max Ueberdrehen %.1f m/s  Einbrueche %d | abseits %.1f m (Anteil %.0f%%)  guard %llu\n", cap, tReach, reached, thSum / n, 100.0 * esp / n, 100.0 * tcs / n, shifts, 100.0 * airborne / n, overMax, stutter, offRoad, 100.0 * offMax / n, (unsigned long long)s->clampedSteps);
      delete s;
      delete w;
    }
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "one1load")) {
    auto* w = new World();
    auto* s = new Sim();
    CarRig r;
    w->build();
    s->world = w;
    const V3 pad{-90, 0, 0};
    Vehicle c0 = buildOne1(*s, pad, &r);
    populateWorld(*s, *w);
    s->finalize();
    settleVehicle(*s, c0, 3.0);
    auto show = [&](const char* name) {
      printf("%-30s Fz %5.0f %5.0f %5.0f %5.0f | L %.3f %.3f %.3f %.3f | F %5.0f %5.0f %5.0f %5.0f | ram %.4f %.4f | hub z-Boden %.3f %.3f %.3f %.3f | roll %.2f pitch %.2f\n", name, c0.wheels[0].Fz, c0.wheels[1].Fz, c0.wheels[2].Fz, c0.wheels[3].Fz, c0.axle[0].s[0].L, c0.axle[0].s[1].L, c0.axle[1].s[0].L, c0.axle[1].s[1].L, c0.axle[0].s[0].F, c0.axle[0].s[1].F, c0.axle[1].s[0].F, c0.axle[1].s[1].F, c0.axle[0].ram, c0.axle[1].ram, c0.wheels[0].hub.z - w->height(c0.wheels[0].hub.x, c0.wheels[0].hub.y), c0.wheels[1].hub.z - w->height(c0.wheels[1].hub.x, c0.wheels[1].hub.y), c0.wheels[2].hub.z - w->height(c0.wheels[2].hub.x, c0.wheels[2].hub.y), c0.wheels[3].hub.z - w->height(c0.wheels[3].hub.x, c0.wheels[3].hub.y), c0.rollAngle * 57.3, c0.pitchAngle * 57.3);
    };
    show("Pad nach Aufbau");
    Controls hold;
    hold.brake = 1;
    auto wait = [&](double sec) {
      for (int k = 0, n = int(sec * 2000); k < n; k++) {
        s->forces(kDt);
        c0.update(*s, hold, kDt);
        s->finish(kDt);
      }
    };
    wait(2.0);
    show("Pad nach 2 s");
    const Obj oo = s->objs[0];
    auto place = [&](V3 at, double yaw) {
      const double cy = std::cos(yaw), sy = std::sin(yaw);
      V3 c{};
      for (u32 k = oo.nodeStart; k < oo.nodeStart + oo.nodeCount; k++) c += s->pos[k];
      c = c / double(oo.nodeCount);
      for (u32 k = oo.nodeStart; k < oo.nodeStart + oo.nodeCount; k++) {
        const V3 q = s->pos[k] - V3{c.x, c.y, 0};
        s->pos[k] = at + V3{cy * q.x - sy * q.y, sy * q.x + cy * q.y, q.z};
        s->vel[k] = {};
      }
    };
    place({-60, 30, 0.03}, 0);
    wait(2.0);
    show("Pad, anderer Ort");
    place({-60, 30, 0.03}, 0.5);
    wait(2.0);
    show("Pad, gedreht");
    const auto& pts = w->roads()[0].pts;
    for (size_t idx : {1200}) {
      const RoadPoint& p = pts[idx];
      const RoadPoint& q = pts[idx + 3];
      place({p.x, p.y, w->height(p.x, p.y) + 0.14}, std::atan2(q.y - p.y, q.x - p.x));
      wait(2.5);
      char nm[64];
      snprintf(nm, sizeof nm, "Strasse %zu, Neigung %.1f", idx, std::atan(p.bank) * 57.3);
      show(nm);
    }
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "one1wing")) {
    auto* w = new World();
    auto* s = new Sim();
    CarRig r;
    w->build();
    w->clearBoxes();
    w->flatten();
    w->pave();
    s->world = w;
    Vehicle c0 = buildOne1(*s, {-400, 0, 0}, &r);
    s->finalize();
    const NodeGroup* wg = nullptr;
    for (const NodeGroup& g : r.groups)
      if (g.name == "wing") wg = &g;
    std::set<u32> wn(wg->nodes.begin(), wg->nodes.end());
    auto report = [&](const char* name) {
      int total = 0, broken = 0, toBody = 0, toBodyBroken = 0;
      double strain = 0;
      for (const Beam& b : s->beams) {
        const bool ia = wn.count(b.a), ib = wn.count(b.b);
        if (!ia && !ib) continue;
        total++;
        broken += (b.flags & BF_BROKEN) != 0;
        if (ia != ib) {
          toBody++;
          toBodyBroken += (b.flags & BF_BROKEN) != 0;
        }
        if (!(b.flags & BF_BROKEN)) strain = std::max(strain, std::fabs(len(s->pos[b.b] - s->pos[b.a]) / b.Lorig - 1.0));
      }
      // Where the wing is against the car, compared with where it was built.
      double off = 0, vmax = 0;
      const V3 c = (s->pos[c0.refRearL] + s->pos[c0.refRearR]) * 0.5, c1 = (s->rest[c0.refRearL] + s->rest[c0.refRearR]) * 0.5;
      for (u32 n : wg->nodes) {
        const V3 d = s->pos[n] - c, d0 = s->rest[n] - c1;
        const V3 d1 = c0.fwd * d0.x + c0.left * d0.y + c0.up * d0.z;
        off = std::max(off, len(d - d1));
        vmax = std::max(vmax, len(s->vel[n] - c0.vCog));
      }
      printf("%-22s Balken %d (zum Auto %d), gerissen %d (zum Auto %d), Dehnung max %.3f, Abweichung %.3f m, Knotentempo %.2f m/s, Objekte %zu\n", name, total, toBody, broken, toBodyBroken, strain, off, vmax, s->objs.size());
    };
    report("gebaut");
    settleVehicle(*s, c0, 3.0);
    report("eingefedert");
    Controls c;
    for (int k = 0; k < 2000 * 12; k++) {
      c.throttle = 1;
      c.steer = k > 2000 * 6 ? 0.04 * std::sin(k * kDt * 2) : 0;
      s->forces(kDt);
      c0.update(*s, c, kDt);
      s->finish(kDt);
      if (k % (2000 * 3) == 5999) report("Vollgas");
    }
    // Pull one bolt: the whole wing must go, and come to rest on the ground, not hang on a hinge.
    s->sever(c0.wingMountBeams[0]);
    for (int k = 0; k < 2000 * 2; k++) {
      c.throttle = 0.2;
      s->forces(kDt);
      c0.update(*s, c, kDt);
      s->finish(kDt);
    }
    report("2 s nach dem Abriss");
    printf("  Meldung: %s, Fluegel-Verbindungen uebrig: %zu, Fluegel z %.2f\n", c0.lastDamage.c_str(), c0.wingMountBeams.size(), s->pos[wg->nodes[0]].z);
    const double e = s->maxEigenvalue(0) * 0.25 * kDt * kDt;
    printf("Eigenwert %.2f, guard %llu, v %.0f km/h\n", e, (unsigned long long)s->clampedSteps, c0.speed * 3.6);
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "shellman")) {
    // A shell of each gun, flying at someone standing about, and past them at a distance.
    auto* d = new Duel();
    d->build();
    for (int kind = 0; kind < TK_COUNT; kind++) {
      for (double miss : {0.0, 1.0, 1.6}) {
        d->players[0].present = d->players[1].present = true;
        d->restart();
        d->sim.logBlasts = true;
        d->phase = Duel::PLAYING;
        Soldier& s = d->players[0].soldier;
        s.hp = 100;
        Tank& t = d->tanks[Duel::tankIndex(1, TurretKind(kind))];
        d->sim.wake(t.hullObj);
        d->sim.wake(t.turretObj);
        const V3 to = s.p + V3{0, 0, 1.0};
        // Aimed at the ground beside them if it is to miss: the blast alone.
        const V3 aim = miss > 0 ? V3{s.p.x + miss, s.p.y, d->world.height(s.p.x + miss, s.p.y)} : to;
        const V3 from = aim + V3{-5, 0, 12};  // from above: nothing in the way but what is meant
        t.shells.push_back({from, norm(aim - from) * t.spec().shellSpeed, 0.0});
        Duel::Intent in[Duel::kMaxPlayers] = {};
        double lowest = s.hp;
        const size_t blastsBefore = d->sim.blastLog.size();
        for (int i = 0; i < 400; i++) {
          d->step(in);
          lowest = std::min(lowest, s.hp);
        }
        if (getenv("SHELLMAN_DEBUG") && !d->sim.blastLog.empty()) printf("   Blast %.1f m vom Soldaten, Kraft %.0f\n", len(d->sim.blastLog.back().c - (s.p + V3{0, 0, 0.9})), d->sim.blastLog.back().impulse);
        if (getenv("SHELLMAN_DEBUG")) printf("   phase %d, Schuss übrig %zu, Blasts %zu, Soldat %.1f %.1f %.1f, tiefster Wert %.0f, Fahrzeug %d\n", int(d->phase), t.shells.size(), d->sim.blastLog.size() - blastsBefore, s.p.x, s.p.y, s.p.z, lowest, d->players[0].vehicle);
        printf("  %-16s %s %.1f m: %.0f von 100 Schaden\n", t.spec().name, miss > 0 ? "Boden daneben" : "Volltreffer  ", miss, 100.0 - lowest);
      }
    }
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "gearrepair")) {
    // Tear a wheel off and bend another, repair, drive off.
    for (int kind = 0; kind < 4; kind++) {
      auto* world = new World();
      world->build();
      world->clearBoxes();
      world->flatten();
      auto* sim = new Sim();
      sim->world = world;
      CarRig rig;
      const V3 at{-300, 0, 0};
      const bool isTank = kind == 3;
      Vehicle car;
      Tank tank;
      if (isTank) tank = buildLeopard(*sim, at, &rig);
      else car = kind == 0 ? buildXantia(*sim, at, 0.0, &rig) : (kind == 1 ? buildOne1(*sim, at, &rig) : buildHummer(*sim, at, &rig));
      sim->finalize();
      if (isTank) tank.settle(*sim, 4.0);
      else settleVehicle(*sim, car, 3.0);
      auto* start = new Sim(*sim);
      const Vehicle carBuilt = car;
      const Tank tankBuilt = tank;
      const u32 hit = isTank ? tank.wheels[0].in : car.wheels[0].axIn;
      const u32 bent = isTank ? tank.wheels[3].in : car.wheels[3].axIn;
      const auto spin = [&](double sec, double gas) {
        Controls c;
        c.throttle = gas;
        for (int i = 0, n = int(sec / kDt); i < n; i++) {
          sim->forces(kDt);
          if (isTank) tank.update(*sim, c, kDt);
          else car.update(*sim, c, kDt);
          sim->finish(kDt);
        }
      };
      const auto offset = [&](u32 n) {
        const V3 a = sim->pos[n] - (isTank ? tank.cog : car.cog);
        const V3 b = start->pos[n] - (isTank ? tankBuilt.cog : carBuilt.cog);
        const V3 f = isTank ? tank.fwd : car.fwd, l = isTank ? tank.left : car.left, u = isTank ? tank.up : car.up;
        const V3 fs = isTank ? tankBuilt.fwd : carBuilt.fwd, ls = isTank ? tankBuilt.left : carBuilt.left, us = isTank ? tankBuilt.up : carBuilt.up;
        return len(V3{dot(a, f) - dot(b, fs), dot(a, l) - dot(b, ls), dot(a, u) - dot(b, us)});
      };
      for (u32 i = 0; i < sim->beams.size(); i++) {
        Beam& b = sim->beams[i];
        if (b.a == hit || b.b == hit) sim->sever(i);
        else if (b.a == bent || b.b == bent) b.L0 *= 1.12;
      }
      spin(2.0, 0.0);
      const double before[2] = {offset(hit), offset(bent)};
      if (isTank) tank.repairRunningGear(*sim, tankBuilt, *start);
      else car.repairRunningGear(*sim, carBuilt, *start);
      spin(2.0, 0.0);
      const double after[2] = {offset(hit), offset(bent)};
      const V3 from = isTank ? tank.cog : car.cog;
      spin(4.0, 0.7);
      printf("  %-7s torn wheel %.2f m off -> %.3f, bent one %.3f -> %.3f; then 4 s of gas: %.1f m, %.0f km/h, sane %d\n", kind == 0 ? "Xantia" : (kind == 1 ? "One:1" : (kind == 2 ? "Hummer" : "Panzer")), before[0], after[0], before[1], after[1], len((isTank ? tank.cog : car.cog) - from), (isTank ? tank.speed : car.speed) * 3.6, std::isfinite(sim->pos[hit].x) && std::fabs(sim->pos[hit].z) < 100.0);
    }
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "carreset")) {
    // Smash the car and part of the scenery, put the car back: the scenery must stay smashed.
    Bench b({-90, 0, 0}, 0, true);
    auto brokenProps = [&] {
      int n = 0;
      const u32 first = b.sim.objs[0].beamCount;
      for (u32 i = first; i < b.sim.beams.size(); i++) n += (b.sim.beams[i].flags & BF_BROKEN) != 0;
      return n;
    };
    const Sim start = b.sim;
    const Vehicle carStart = b.car;
    b.launchAt(120 / 3.6, 3);
    b.run(9.0, [&](double, Controls& c) { c.throttle = 0.5; });
    int carBroken = 0;
    for (u32 i = 0; i < b.sim.objs[0].beamCount; i++) carBroken += (b.sim.beams[i].flags & BF_BROKEN) != 0;
    const size_t objsBefore = b.sim.objs.size();
    int pieces = 0;
    for (size_t i = 1; i < b.sim.objs.size(); i++) pieces += b.sim.objs[i].root == 0;
    const V3 fence = b.sim.pos[b.sim.objs[10].nodeStart];
    printf("nach der Fahrt: Auto-Balken gerissen %d, Teile vom Auto %d, Umgebung gerissen %d, Objekte %zu, x %.1f\n", carBroken, pieces, brokenProps(), objsBefore, b.car.cog.x);
    const int envBefore = brokenProps();
    b.sim.restoreObjects(start, 1);
    b.car = carStart;
    carBroken = 0;
    for (u32 i = 0; i < b.sim.objs[0].beamCount; i++) carBroken += (b.sim.beams[i].flags & BF_BROKEN) != 0;
    printf("nach dem Reset: Auto-Balken gerissen %d, Umgebung gerissen %d (war %d), Prop-Knoten bewegt %.3f m, Auto x %.1f\n", carBroken, brokenProps(), envBefore, len(b.sim.pos[b.sim.objs[10].nodeStart] - fence), b.car.cog.x);
    b.run(4.0, [&](double, Controls& c) { c.throttle = 0.4; });
    printf("danach 4 s Fahrt: %.0f km/h, x %.1f, Verformung %.1f kJ, guard %llu, sane %d, Umgebung gerissen %d, wach %zu\n", b.car.speed * 3.6, b.car.cog.x, b.sim.plasticEnergy / 1e3, (unsigned long long)b.sim.clampedSteps, b.sane(), brokenProps(), b.sim.active.size());
    for (int k = 0; k < 2; k++) {
      b.launchAt(150 / 3.6, 3);
      b.run(5.0, [&](double, Controls& c) { c.throttle = 0.5; });
      b.sim.restoreObjects(start, 1);
      b.car = carStart;
    }
    b.run(2.0, [&](double, Controls&) {});
    printf("zwei weitere Runden und Resets: sane %d, guard %llu, Objekte %zu, wach %zu, Umgebung gerissen %d\n", b.sane(), (unsigned long long)b.sim.clampedSteps, b.sim.objs.size(), b.sim.active.size(), brokenProps());
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "wingtrack")) {
    // Where is the wing against the tail, in the physics and in the mesh the player sees?
    auto* w = new World();
    auto* s = new Sim();
    CarRig r;
    w->build();
    s->world = w;
    const V3 spawn{-90, 0, 0};
    Vehicle c0 = buildOne1(*s, spawn, &r);
    populateWorld(*s, *w);
    s->finalize();
    settleVehicle(*s, c0, 3.0);
    auto* mesh = new CarMesh();
    if (!mesh->load("assets/one1.mesh")) return 1;
    mesh->noseX = r.noseX;
    mesh->bind(s->rest, r.groups, spawn);
    const NodeGroup* wg = nullptr;
    for (const NodeGroup& g : r.groups)
      if (g.name == "wing") wg = &g;
    std::vector<u32> verts;
    for (const MeshPart& p : mesh->parts)
      if (p.group == "wing")
        for (u32 v = p.firstVert; v < p.firstVert + p.vertCount; v += 3) verts.push_back(v);
    printf("Fluegel: %zu Knoten, %zu Modellpunkte geprueft\n", wg->nodes.size(), verts.size());
    // A frame made of three points of the tail.
    const u32 A = c0.refRearL, B = c0.refRearR, C = c0.refFrontL;
    auto frame = [&](const std::vector<V3>& P, V3& o, V3& e1, V3& e2, V3& e3) {
      o = (P[A] + P[B]) * 0.5;
      e1 = norm(P[B] - P[A]);
      e2 = norm(P[C] - P[A]);
      e2 = norm(e2 - e1 * dot(e2, e1));
      e3 = cross(e1, e2);
    };
    V3 ro, r1, r2, r3;
    frame(s->rest, ro, r1, r2, r3);
    auto predict = [&](V3 restP, V3 o, V3 e1, V3 e2, V3 e3) {
      const V3 d = restP - ro;
      return o + e1 * dot(d, r1) + e2 * dot(d, r2) + e3 * dot(d, r3);
    };
    u32 seenSplit = s->splitGeneration;
    auto report = [&](const char* name) {
      if (s->splitGeneration != seenSplit) {
        const size_t n = mesh->rebindAfterTear(s->rest, r.groups, spawn, s->nodeObj);
        printf("  [Modell neu gebunden: %zu Punkte]\n", n);
        seenSplit = s->splitGeneration;
      }
      V3 o, e1, e2, e3;
      frame(s->pos, o, e1, e2, e3);
      double phys = 0, vis = 0;
      for (u32 n : wg->nodes) phys = std::max(phys, len(s->pos[n] - predict(s->rest[n], o, e1, e2, e3)));
      // The same measure on the tail of the body, right under the wing: what the frame itself reads.
      double tail = 0, relative = 0;
      {
        const std::set<u32> in(wg->nodes.begin(), wg->nodes.end());
        u32 near = 0;
        double bd = 1e9;
        const V3 foot = s->rest[wg->nodes.back()];
        for (u32 k = 0; k < s->objs[0].nodeCount; k++)
          if (!in.count(k) && (s->flags[k] & NF_COLLIDE) && !(s->flags[k] & NF_WHEEL) && len2(s->rest[k] - foot) < bd) {
            bd = len2(s->rest[k] - foot);
            near = k;
          }
        const V3 dt0 = s->pos[near] - predict(s->rest[near], o, e1, e2, e3);
        tail = len(dt0);
        for (u32 n : wg->nodes) relative = std::max(relative, len(s->pos[n] - predict(s->rest[n], o, e1, e2, e3) - dt0));
      }
      printf("  [Heck selbst %.3f m, Fluegel gegen das Heck %.3f m]\n", tail, relative);
      for (u32 v : verts) {
        V3 q{};
        V3 rp = spawn + V3{r.noseX - mesh->restPos[v * 3], mesh->restPos[v * 3 + 1], mesh->restPos[v * 3 + 2]};
        for (int k = 0; k < CarMesh::kK; k++) q += s->pos[mesh->bindNode[size_t(v) * CarMesh::kK + k]] * double(mesh->bindWeight[size_t(v) * CarMesh::kK + k]);
        vis = std::max(vis, len(q - predict(rp, o, e1, e2, e3)));
      }
      int obj = s->nodeObj[wg->nodes[0]], root = s->objs[size_t(obj)].root;
      printf("  %-22s Kurs %+5.0f  %4.0f km/h  Physik-Abw. %.3f m  Modell-Abw. %.3f m  Objekt %d (root %d) halt %zu  Reifen %s  %s\n", name, std::atan2(c0.fwd.y, c0.fwd.x) * 57.3, c0.speed * 3.6, phys, vis, obj, root, c0.wingMountBeams.size(), c0.wheels[0].deflate > 0 ? "platt" : "ok", c0.lastDamage.c_str());
    };
    auto drive = [&](double sec, std::function<void(double, Controls&)> fn) {
      Controls c;
      for (int k = 0, n = int(sec / kDt); k < n; k++) {
        fn(k * kDt, c);
        s->forces(kDt);
        c0.update(*s, c, kDt);
        s->finish(kDt);
        c.fire = false;
      }
      s->events.clear();
    };
    report("Start");
    drive(3, [&](double, Controls& c) { c.throttle = 0.6; c.steer = 0; });
    report("geradeaus");
    for (int i = 0; i < 6; i++) {
      drive(2, [&](double, Controls& c) { c.throttle = clampd((50 / 3.6 - c0.speed) * 0.5, 0, 1); c.steer = 0.6; });
      report("Kreis links");
    }
    for (int i = 0; i < 3; i++) {
      drive(2, [&](double, Controls& c) { c.throttle = 1; c.steer = -0.8; });
      report("Vollgas Kreis rechts");
    }
    drive(2, [&](double, Controls& c) { c.throttle = 0; c.steer = 0; c.brake = 1; });
    report("Bremsen");
    drive(10, [&](double, Controls& c) {
      c.brake = 0;
      const double head = std::atan2(c0.fwd.y, c0.fwd.x);
      double err = std::atan2(0.0 - c0.cog.y, 60.0) - head;
      while (err > PI) err -= 2 * PI;
      while (err < -PI) err += 2 * PI;
      c.steer = clampd(err * 1.5, -0.4, 0.4);
      c.throttle = c0.speed < 22 ? 0.8 : 0.2;
    });
    report("durch die Hindernisse");
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "hummerbrake")) {
    // Frame by frame through the full stop where the front wheels fold.
    auto* w = new World();
    auto* s = new Sim();
    CarRig r;
    w->build();
    s->world = w;
    const double brakeIn = argc > 2 ? atof(argv[2]) : 1.0;
    const bool flat = argc > 3 && atoi(argv[3]) != 0;
    const int every = argc > 4 ? atoi(argv[4]) : 200;
    const double secs = argc > 5 ? atof(argv[5]) : 6.0;
    if (flat) w->flatten();
    Vehicle c = buildHummer(*s, {-200, 0, 0}, &r);
    s->finalize();
    settleVehicle(*s, c, 3.0);
    const Obj o = s->objs[0];
    for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) s->vel[i] = V3{100 / 3.6, 0, 0};
    c.presetRolling(100 / 3.6, 2);
    Controls ct;
    ct.brake = brakeIn;
    const u32 first = c.wheels[0].axIn;
    const char* names[19] = {"axIn", "axOut", "ring0", "ring1", "ring2", "ring3", "ring4", "ring5", "kl", "ku", "ks", "kf", "lf", "lr", "uf", "ur", "rack", "", ""};
    V3 rest0[17];
    for (int i = 0; i < 17; i++) {
      const V3 q = s->pos[first + u32(i)] - c.cog;
      rest0[i] = V3{dot(q, c.fwd), dot(q, c.left), dot(q, c.up)};
    }
    for (int k = 0; k < int(2000 * secs); k++) {
      s->forces(kDt);
      c.update(*s, ct, kDt);
      s->finish(kDt);
      if (k % every == 0) {
        const Wheel& W = c.wheels[0];
        const V3 ax = norm(s->pos[W.axOut] - s->pos[W.axIn]);
        const V3 hub = (s->pos[W.axIn] + s->pos[W.axOut]) * 0.5 - c.cog;
        const Beam& t = s->beams[c.tieBeam[0]];
        printf("t %.1f v %5.1f  fwd %+.2f %+.2f %+.2f left %+.2f %+.2f %+.2f | achse welt %+.2f %+.2f %+.2f | Nabe %+.2f %+.2f %+.2f | Spurst %.3f/%.3f F %5.1f kN | Fz %5.0f Fx %6.0f om %5.1f brems %d\n", k * kDt, c.speed * 3.6, c.fwd.x, c.fwd.y, c.fwd.z, c.left.x, c.left.y, c.left.z, ax.x, ax.y, ax.z, dot(hub, c.fwd), dot(hub, c.left), dot(hub, c.up), len(s->pos[t.a] - s->pos[t.b]), t.L0, t.force / 1e3, W.Fz, W.Fx, W.omega, int(c.absActive));
      }
    }
    for (int i = 0; i < 17; i++) {
      const V3 q = s->pos[first + u32(i)] - c.cog;
      printf("%-6s Start %+.3f %+.3f %+.3f  jetzt %+.3f %+.3f %+.3f\n", names[i], rest0[i].x, rest0[i].y, rest0[i].z, dot(q, c.fwd), dot(q, c.left), dot(q, c.up));
    }
    // The beams that hold the wheel nodes: how far from their length are they now?
    for (u32 bi = o.beamStart; bi < o.beamStart + o.beamCount; bi++) {
      const Beam& b = s->beams[bi];
      if (b.a < first || b.a >= first + 17 || b.b < first || b.b >= first + 17) continue;
      if (b.a < first + 8 && b.b < first + 8) continue;
      const double L = len(s->pos[b.a] - s->pos[b.b]);
      if (std::fabs(L - b.L0) / b.L0 > 0.01 || std::fabs(b.L0 - b.Lorig) / b.Lorig > 0.005 || (b.flags & BF_BROKEN))
        printf(" Balken %-6s-%-6s L %.3f L0 %.3f Lorig %.3f %s\n", names[b.a - first], names[b.b - first], L, b.L0, b.Lorig, (b.flags & BF_BROKEN) ? "GEBROCHEN" : "");
    }
    return 0;
  }
  if (argc > 1 && (!strcmp(argv[1], "hummersteer") || !strcmp(argv[1], "xantiasteer") || !strcmp(argv[1], "one1steer"))) {
    // What bends the steering? Bent tie rods, as a change of toe that stays once the wheel is straight again.
    const bool xan = !strcmp(argv[1], "xantiasteer"), one = !strcmp(argv[1], "one1steer");
    printf("== Lenkung: bleibende Verstellung (Spurstange), Lastspitze ==\n");
    struct B {
      World w;
      Sim s;
      Vehicle c;
      CarRig r;
    };
    bool rockAt = false;
    auto make = [xan, one, &rockAt](V3 at) {
      B* b = new B();
      b->w.build();
      if (rockAt) {
        at.z = b->w.height(at.x, at.y);
        b->w.addBox({at.x + 7.0, at.y + 0.91, at.z + 0.25}, {0.5, 0.45, 0.5}, 0, 0, SURF_ROCK, .5f, .5f, .5f);
      }
      b->s.world = &b->w;
      b->c = xan ? buildXantia(b->s, at, 0.0, &b->r) : (one ? buildOne1(b->s, at, &b->r) : buildHummer(b->s, at, &b->r));
      b->s.finalize();
      settleVehicle(b->s, b->c, 3.0);
      return b;
    };
    auto pose = [](B* b, double* toe, double* camber, V3* hub) {
      for (int i = 0; i < 2; i++) {
        const Wheel& w = b->c.wheels[size_t(i)];
        const V3 ax = norm(b->s.pos[w.axOut] - b->s.pos[w.axIn]) * (w.left ? 1.0 : -1.0);
        toe[i] = std::atan2(dot(ax, b->c.fwd), dot(ax, b->c.left)) * 57.3;
        camber[i] = std::asin(clampd(dot(ax, b->c.up), -1, 1)) * 57.3;
        const V3 r = (b->s.pos[w.axIn] + b->s.pos[w.axOut]) * 0.5 - b->c.cog;
        hub[i] = V3{dot(r, b->c.fwd), dot(r, b->c.left), dot(r, b->c.up)};
      }
    };
    auto scenario = [&](const char* name, V3 at, double kmh, V3 dir, std::function<void(double, Controls&)> fn, double sec) {
      B* b = make(at);
      const Obj o = b->s.objs[0];
      double t0[2], c0[2];
      V3 h0[2];
      pose(b, t0, c0, h0);
      if (kmh > 0) {
        for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) b->s.vel[i] = dir * (kmh / 3.6);
        b->c.presetRolling(kmh / 3.6, 2);
      }
      double peak[2] = {};
      Controls c;
      for (int k = 0, n = int(sec / kDt); k < n; k++) {
        fn(k * kDt, c);
        b->s.forces(kDt);
        b->c.update(b->s, c, kDt);
        b->s.finish(kDt);
        for (int i = 0; i < 2; i++) peak[i] = std::max(peak[i], double(std::fabs(b->s.beams[b->c.tieBeam[i]].force)));
      }
      Controls off;
      for (int k = 0; k < 4000; k++) {
        off.brake = 1;
        b->s.forces(kDt);
        b->c.update(b->s, off, kDt);
        b->s.finish(kDt);
      }
      double t1[2], c1[2];
      V3 h1[2];
      pose(b, t1, c1, h1);
      // Which beams gave way, grouped by what they are.
      std::map<std::pair<int, int>, std::pair<int, double>> gave;
      for (u32 bi = o.beamStart; bi < o.beamStart + o.beamCount; bi++) {
        const Beam& q = b->s.beams[bi];
        const double strain = std::fabs(q.L0 - q.Lorig) / q.Lorig;
        if (strain > 0.002 && (bi != b->c.tieBeam[0] && bi != b->c.tieBeam[1])) {
          // Only what touches the front suspension.
          const V3 mid = (b->s.rest[q.a] + b->s.rest[q.b]) * 0.5;
          if (std::fabs(mid.x - b->s.rest[b->c.wheels[0].axIn].x) > 0.6) continue;
          auto& g = gave[{int(q.yC / 1e3), int(q.k / 1e4)}];
          g.first++;
          g.second = std::max(g.second, strain);
        }
      }
      double rearToe[2];
      for (int i = 0; i < 2; i++) {
        const Wheel& w = b->c.wheels[size_t(i) + 2];
        const V3 ax = norm(b->s.pos[w.axOut] - b->s.pos[w.axIn]) * (w.left ? 1.0 : -1.0);
        rearToe[i] = std::atan2(dot(ax, b->c.fwd), dot(ax, b->c.left)) * 57.3;
      }
      printf("  %-34s hinten %+5.1f %+5.1f  Spur %+5.1f %+5.1f  Sturz %+5.1f %+5.1f Grad  Rad weg %4.0f %4.0f mm  Spurstange %3.0f %3.0f kN %d%d  v %3.0f", name, rearToe[0], rearToe[1], t1[0] - t0[0], t1[1] - t0[1], c1[0] - c0[0], c1[1] - c0[1], len(h1[0] - h0[0]) * 1e3, len(h1[1] - h0[1]) * 1e3, peak[0] / 1e3, peak[1] / 1e3, int((b->s.beams[b->c.tieBeam[0]].flags & BF_BROKEN) != 0), int((b->s.beams[b->c.tieBeam[1]].flags & BF_BROKEN) != 0), b->c.speed * 3.6);
      for (const auto& g : gave) printf("  [%dkN x%d %.0f%%]", g.first.first, g.second.first, g.second.second * 100);
      printf("\n");
      delete b;
    };
    const V3 X{1, 0, 0};
    const auto none = [](double, Controls&) {};
    scenario("im Stand voll einschlagen", {-90, 0, 0}, 0, X, [](double t, Controls& c) { c.steer = std::sin(t * 2.0); }, 6.0);
    scenario("10 km/h Vollausschlag hin und her", {-90, 0, 0}, 10, X, [](double t, Controls& c) { c.throttle = .2; c.steer = std::sin(t * 2.5) > 0 ? 1 : -1; }, 6.0);
    scenario("60 km/h Slalom", {-120, 0, 0}, 60, X, [](double t, Controls& c) { c.throttle = .3; c.steer = std::sin(t * 2.5); }, 5.0);
    scenario("100 km/h Vollbremsung", {-200, 0, 0}, 100, X, [](double, Controls& c) { c.brake = 1; }, 5.0);
    const double wallY = -35.0;
    for (double kmh : {10.0, 20.0, 30.0, 45.0, 60.0}) {
      char nm[64];
      snprintf(nm, sizeof nm, "linkes Rad gegen Wand %2.0f km/h", kmh);
      scenario(nm, {110.0 - 12.0, wallY - 0.91 + 0.05, 0}, kmh, X, none, 2.5);
    }
    for (double kmh : {15.0, 30.0, 60.0}) {
      char nm[64];
      snprintf(nm, sizeof nm, "seitlich in Bordstein %2.0f km/h", kmh);
      scenario(nm, {-20, 92, 0}, kmh, V3{0, 1, 0}, none, 1.5);
    }
    for (double kmh : {30.0, 60.0, 90.0}) {
      char nm[64];
      snprintf(nm, sizeof nm, "schraeg ueber Bordstein %2.0f km/h", kmh);
      scenario(nm, {-46, 91, 0}, kmh, V3{std::cos(0.45), std::sin(0.45), 0}, none, 2.0);
    }
    // A block in the way of the left front wheel alone, about as high as a boulder on the trail.
    for (double kmh : {15.0, 25.0, 40.0, 60.0}) {
      char nm[64];
      snprintf(nm, sizeof nm, "Fels vor linkem Rad %2.0f km/h", kmh);
      rockAt = true;
      scenario(nm, {-300, 400, 0}, kmh, X, none, 2.5);
      rockAt = false;
    }
    scenario("eingeschlagen in Bordstein 25 km/h", {-46, 91, 0}, 25, V3{std::cos(0.45), std::sin(0.45), 0}, [](double, Controls& c) { c.steer = 1; }, 2.0);
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "hummerbuckle")) {
    printf("== Hummer: wie leicht knicken die Raeder? (bleibende Aenderung) ==\n");
    struct B {
      World w;
      Sim s;
      Vehicle c;
      CarRig r;
    };
    auto make = [](V3 at, bool flat) {
      B* b = new B();
      b->w.build();
      if (flat) b->w.flatten();
      b->s.world = &b->w;
      b->c = buildHummer(b->s, at, &b->r);
      b->s.finalize();
      settleVehicle(b->s, b->c, 4.0);
      return b;
    };
    auto attitude = [](B* b, double out[4]) {
      for (int i = 0; i < 4; i++) {
        const Wheel& w = b->c.wheels[size_t(i)];
        const V3 ax = norm(b->s.pos[w.axOut] - b->s.pos[w.axIn]);
        out[i] = std::asin(clampd(dot(ax, b->c.up), -1, 1)) * 57.3 * (w.left ? 1.0 : -1.0);
      }
    };
    auto run = [](B* b, double sec, std::function<void(double, Controls&)> fn) {
      Controls c;
      for (int k = 0, n = int(sec / kDt); k < n; k++) {
        fn(k * kDt, c);
        b->s.forces(kDt);
        b->c.update(b->s, c, kDt);
        b->s.finish(kDt);
      }
      b->s.events.clear();
    };
    auto report = [&](const char* name, B* b, double a0[4]) {
      run(b, 3.0, [](double, Controls& c) { c.brake = 1; c.steer = 0; });
      double a[4], worst = 0;
      attitude(b, a);
      for (int i = 0; i < 4; i++) worst = std::max(worst, std::fabs(a[i] - a0[i]));
      float dent = 0;
      int flat = 0;
      for (const Wheel& w : b->c.wheels) {
        for (float d : w.dent) dent = std::max(dent, d);
        flat += w.deflate > 0;
      }
      printf("  %-30s Sturz +%4.1f Grad  Delle %4.1f mm  platt %d  verformt %6.1f kJ  gerissen %u  Meldung: %s\n", name, worst, dent * 1e3, flat, b->s.plasticEnergy / 1e3, b->s.brokenBeams, b->c.lastDamage.c_str());
      delete b;
    };
    for (double h : {0.5, 1.0, 2.0, 3.5}) {
      B* b = make({-400, 0, 0}, true);
      double a0[4];
      attitude(b, a0);
      const Obj o = b->s.objs[0];
      for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) b->s.pos[i].z += h;
      run(b, 3.0, [](double, Controls&) {});
      char nm[64];
      snprintf(nm, sizeof nm, "Sprung aus %.1f m", h);
      report(nm, b, a0);
    }
    // Over the kerbs at y = 95, the speed bumps at y = 70, and into the wall end.
    for (double kmh : {30.0, 60.0, 90.0}) {
      B* b = make({-46, 91, 0}, false);
      double a0[4];
      attitude(b, a0);
      const Obj o = b->s.objs[0];
      for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) b->s.vel[i] = V3{std::cos(0.45) * kmh / 3.6, std::sin(0.45) * kmh / 3.6, 0};
      b->c.presetRolling(kmh / 3.6, 2);
      run(b, 2.0, [](double, Controls&) {});
      char nm[64];
      snprintf(nm, sizeof nm, "schraeg ueber Bordstein %.0f", kmh);
      report(nm, b, a0);
    }
    for (double kmh : {15.0, 30.0, 50.0}) {
      B* b = make({-20, 92, 0}, false);
      double a0[4];
      attitude(b, a0);
      const Obj o = b->s.objs[0];
      for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) b->s.vel[i] = V3{0, kmh / 3.6, 0};
      run(b, 1.5, [](double, Controls&) {});
      char nm[64];
      snprintf(nm, sizeof nm, "seitlich in Bordstein %.0f", kmh);
      report(nm, b, a0);
    }
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "adopt")) {
    // Wreck the scenery with one vehicle and carry on with another: the scenery must be just as it was.
    auto* w = new World();
    w->build();
    auto* a = new Sim();
    CarRig ra;
    a->world = w;
    Vehicle ca = buildXantia(*a, {-90, 0, 0}, 0, &ra);
    populateWorld(*a, *w);
    a->finalize();
    settleVehicle(*a, ca, 3.0);
    Controls c;
    c.throttle = 0.9;
    for (int k = 0; k < 2000 * 11; k++) {
      c.steer = clampd(-0.05 * ca.cog.y - 0.7 * std::atan2(ca.fwd.y, ca.fwd.x), -0.25, 0.25);
      a->forces(kDt);
      ca.update(*a, c, kDt);
      a->finish(kDt);
    }
    const u32 oldCars = 1;
    int brokenA = 0, piecesA = 0;
    for (u32 i = a->objs[oldCars].beamStart; i < a->beams.size(); i++) brokenA += (a->beams[i].flags & BF_BROKEN) != 0;
    for (const Obj& o : a->objs) piecesA += o.root != 0 && (&o - a->objs.data()) != o.root;
    printf("alt: Umgebung gerissen %d, abgerissene Stuecke %d, Objekte %zu, wach %zu, Auto bei x %.0f\n", brokenA, piecesA, a->objs.size(), a->active.size(), ca.cog.x);
    for (int which = 0; which < 3; which++) {
      auto* b = new Sim();
      CarRig rb;
      b->world = w;
      Vehicle cb;
      Tank tb;
      if (which == 0) cb = buildHummer(*b, {-90, 0, 0}, &rb);
      else if (which == 1) cb = buildOne1(*b, {-90, 0, 0}, &rb);
      else tb = buildLeopard(*b, {-90, 0, 0}, &rb);
      populateWorld(*b, *w);
      b->finalize();
      if (which == 2) tb.settle(*b, 2.0);
      else settleVehicle(*b, cb, 2.0);
      const u32 newCars = which == 2 ? 2 : 1;
      const bool ok = b->adoptWorld(*a, oldCars, newCars);
      int brokenB = 0;
      for (u32 i = b->objs[newCars].beamStart; i < b->beams.size(); i++) brokenB += (b->beams[i].flags & BF_BROKEN) != 0;
      double off = 0;
      const u32 na = a->objs[oldCars].nodeStart, nb = b->objs[newCars].nodeStart;
      for (u32 i = 0; i + nb < b->pos.size(); i++) off = std::max(off, len(b->pos[nb + i] - a->pos[na + i]));
      Controls cc;
      cc.throttle = 0.3;
      for (int k = 0; k < 2000 * 4; k++) {
        b->forces(kDt);
        if (which == 2) tb.update(*b, cc, kDt);
        else cb.update(*b, cc, kDt);
        b->finish(kDt);
      }
      int brokenAfter = 0;
      for (u32 i = b->objs[newCars].beamStart; i < b->beams.size(); i++) brokenAfter += (b->beams[i].flags & BF_BROKEN) != 0;
      printf("%s: uebernommen %d, gerissen %d (war %d), Abweichung %.6f m, Objekte %zu | nach 4 s: gerissen %d, wach %zu, guard %llu, Fahrzeug %.0f km/h\n", which == 0 ? "Hummer" : (which == 1 ? "One:1 " : "Leopard"), ok, brokenB, brokenA, off, b->objs.size(), brokenAfter, b->active.size(), (unsigned long long)b->clampedSteps, (which == 2 ? tb.speed : cb.speed) * 3.6);
      delete b;
    }
    // The edge of the map: is the ground continuous across it?
    double step = 0;
    for (double t = -1000; t < 1000; t += 3.7) {
      const double in = w->height(1023.99, t), out = w->height(1024.01, t);
      step = std::max(step, std::fabs(in - out));
      const double in2 = w->height(t, -1023.99), out2 = w->height(t, -1024.01);
      step = std::max(step, std::fabs(in2 - out2));
    }
    printf("Kartenrand: groesster Hoehensprung %.3f m\n", step);
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "waterout")) {
    // A tank in the middle of the deep river, and full throttle towards the bank. Does it get out, and how fast does it go in there?
    for (int h = 0; h < HK_COUNT; h++) {
      auto* d = new Duel();
      d->playersWanted = 2;
      d->build();
      d->players[0].present = d->players[1].present = true;
      Duel::Intent in[Duel::kMaxPlayers];
      const auto run = [&](double seconds) {
        for (double t = 0; t < seconds; t += Duel::kDt) {
          d->step(in);
          std::vector<SimEvent> ev;
          d->drainEvents(ev);
        }
      };
      u8 none[IT_COUNT] = {};
      while (d->phase != Duel::CHOOSING) run(0.1);
      d->choose(0, none, true);
      d->choose(1, none, true);
      while (d->phase != Duel::PLAYING) run(0.05);
      const int v = Duel::tankIndex(0, TK_STANDARD, HullKind(h));
      Tank& t = d->tanks[v];
      const double y = 300.0;
      const V3 to{World::riverAt(y), y, d->world.height(World::riverAt(y), y) + 1.5};
      d->players[0].soldier.p = t.cog + t.left * 3.0;
      in[0].use = true;
      run(Duel::kDt * 0.5);
      in[0].use = false;
      const V3 by = to - t.cog;
      for (u16 oi : {t.hullObj, t.turretObj})
        for (u32 n : d->sim.objs[oi].nodes) d->sim.pos[n] += by;
      run(1.5);
      in[0].c.throttle = 1;
      double top = 0, outAt = -1;
      for (double at = 0; at < 60.0 && outAt < 0; at += 0.5) {
        run(0.5);
        top = std::max(top, std::fabs(d->frames[v].speed) * 3.6);
        if (std::fabs(t.cog.x - World::riverAt(t.cog.y)) > 24.0) outAt = at;
        if (int(at * 2) % 8 == 0) printf("  t %4.1f quer zum Fluss %5.1f m, %4.1f km/h, Hoehe %.1f, Wasser %d\n", at, t.cog.x - World::riverAt(t.cog.y), d->frames[v].speed * 3.6, t.cog.z, int(d->world.isWater(t.cog.x, t.cog.y, t.cog.z)));
      }
      printf("%-18s draussen nach %.1f s, schnellste %.1f km/h\n", hullSpec(HullKind(h)).name, outAt, top);
      delete d;
    }
    // A bridge: is it water up there?
    auto* d = new Duel();
    d->build();
    const World::Bridge& br = d->world.bridgeSpans()[0];
    printf("Bruecke: Deck z %.1f, Wasser auf dem Deck %d, 1.3 m darueber %d, 3 m darueber %d, unter dem Deck am Fluss %d\n", br.at.z, int(d->world.isWater(br.at.x, br.at.y, br.at.z + 0.1)), int(d->world.isWater(br.at.x, br.at.y, br.at.z + 1.3)), int(d->world.isWater(br.at.x, br.at.y, br.at.z + 3.0)), int(d->world.isWater(br.at.x, br.at.y, 2.0)));
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "tankpull")) {
    // Every hull from a standstill on the level, and then up a slope as steep as the map has for a road.
    for (int h = 0; h < HK_COUNT; h++) {
      auto* w = new World();
      auto* s = new Sim();
      CarRig r;
      w->build();
      w->clearBoxes();
      w->flatten();
      w->pave();
      s->world = w;
      Tank t = buildLeopard(*s, V3{-400, 0, 0}, &r, TK_STANDARD, HullKind(h));
      s->finalize();
      t.settle(*s, 3.0);
      Controls c;
      c.throttle = 1;
      double t20 = -1, t40 = -1, t60 = -1, top = 0, maxTconv = 0;
      for (int k = 0; k < 2000 * 45; k++) {
        s->forces(kDt);
        t.update(*s, c, kDt);
        s->finish(kDt);
        const double kmh = t.speed * 3.6, at = k * kDt;
        if (t20 < 0 && kmh >= 20) t20 = at;
        if (t40 < 0 && kmh >= 40) t40 = at;
        if (t60 < 0 && kmh >= 60) t60 = at;
        top = std::max(top, kmh);
        maxTconv = std::max(maxTconv, t.lonAcc);
      }
      s->events.clear();
      printf("%-18s 0-20 %5.1f s  0-40 %5.1f s  0-60 %5.1f s  Spitze %5.1f km/h  hoechste Beschl. %.2f m/s2\n", hullSpec(HullKind(h)).name, t20, t40, t60, top, maxTconv);
    }
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "tankrun")) {
    auto* w = new World();
    auto* s = new Sim();
    CarRig r;
    w->build();
    w->clearBoxes();
    w->flatten();
    w->pave();
    s->world = w;
    Tank t = buildLeopard(*s, V3{-400, 0, 0}, &r, TK_STANDARD, argc > 2 ? HullKind(atoi(argv[2])) : HK_MEDIUM);
    s->finalize();
    t.settle(*s, 3.0);
    Controls c;
    c.throttle = 1;
    for (int k = 0; k < 2000 * 28; k++) {
      s->forces(kDt);
      t.update(*s, c, kDt);
      s->finish(kDt);
      if (k % 3000 == 2999) printf(" t %4.1f v %5.1f km/h gear %d rpm %4.0f load %.2f | Kette %5.1f / %5.1f km/h | Radkraft %.0f N\n", (k + 1) * kDt, t.speed * 3.6, t.gear, t.rpm(), t.loadNow, t.trackOmega[0] * Tank::kSprocketR * 3.6, t.trackOmega[1] * Tank::kSprocketR * 3.6, t.wheels[3].Fn);
    }
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "tankturn")) {
    auto* w = new World();
    auto* s = new Sim();
    CarRig r;
    w->build();
    w->clearBoxes();
    w->flatten();
    w->pave();
    s->world = w;
    Tank t = buildLeopard(*s, V3{-120, 0, 0}, &r);
    s->finalize();
    t.settle(*s, 3.0);
    auto run = [&](const char* name, double kmh, double sec) {
      Controls c;
      double prev = std::atan2(t.fwd.y, t.fwd.x), sum = 0, peak = 0;
      for (int k = 0, n = int(sec / kDt); k < n; k++) {
        c.steer = 1;
        c.throttle = kmh > 1 ? clampd((kmh / 3.6 - t.speed) * 0.8, 0, 1) : 0;
        s->forces(kDt);
        t.update(*s, c, kDt);
        s->finish(kDt);
        const double h = std::atan2(t.fwd.y, t.fwd.x);
        double d = h - prev;
        while (d > PI) d -= 2 * PI;
        while (d < -PI) d += 2 * PI;
        sum += d;
        peak = std::max(peak, std::fabs(d) / kDt);
        prev = h;
      }
      printf("  %-20s %5.0f Grad in %.0f s (Schnitt %.2f rad/s), Ende: %.0f km/h, Ketten %.1f / %.1f m/s, Quer %.2f g\n", name, sum * 57.3, sec, sum / sec, t.speed * 3.6, t.trackOmega[0] * Tank::kSprocketR, t.trackOmega[1] * Tank::kSprocketR, t.latAcc / 9.81);
    };
    run("Wenden im Stand", 0, 4);
    {
      Controls c;
      for (int k = 0; k < 2000 * 2; k++) {
        c.brake = 1;
        s->forces(kDt);
        t.update(*s, c, kDt);
        s->finish(kDt);
      }
    }
    run("voll lenken 20 km/h", 20, 5);
    run("voll lenken 40 km/h", 40, 5);
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "hummerrun")) {
    auto* w = new World();
    auto* s = new Sim();
    CarRig r;
    w->build();
    w->clearBoxes();
    w->flatten();
    w->pave();
    s->world = w;
    Vehicle c0 = buildHummer(*s, {-400, 0, 0}, &r);
    s->finalize();
    settleVehicle(*s, c0, 4.0);
    Controls c;
    c.throttle = 1;
    for (int i = 0; i < 2000 * 20; i++) {
      s->forces(kDt);
      c0.update(*s, c, kDt);
      s->finish(kDt);
      if (i % 1600 == 1599) {
        printf(" t %4.1f v %5.1f g %d ph %d rpm %4.0f input %4.0f th %.2f boost %.2f T %5.0f clutch %.2f | om*R FL %5.1f RL %5.1f | Fx %6.0f %6.0f %6.0f %6.0f | Fz %5.0f %5.0f | drive %6.0f %6.0f esp %5.0f %5.0f | twist %.3f %.3f | asr %d carrier %.1f\n", (i + 1) * kDt, c0.speed * 3.6, c0.gear, c0.shiftPhase, c0.rpm(), c0.inputOmega * 9.55, c0.throttleAct, c0.boost, c0.combustionTorque, c0.clutch, c0.wheels[0].omega * c0.wheels[0].R, c0.wheels[2].omega * c0.wheels[2].R, c0.wheels[0].Fx, c0.wheels[1].Fx, c0.wheels[2].Fx, c0.wheels[3].Fx, c0.wheels[0].Fz, c0.wheels[2].Fz, c0.wheels[0].driveT, c0.wheels[2].driveT, c0.wheels[0].espTorque, c0.wheels[2].espTorque, c0.shaftAngle[0], c0.shaftAngle[2], c0.tcsActive, c0.carrierOmega);
      }
    }
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "hummer")) {
    printf("== Hummer H1 ==\n");
    struct B {
      World w;
      Sim s;
      Vehicle c;
      CarRig r;
    };
    auto make = [](V3 at, bool flat) {
      B* b = new B();
      b->w.build();
      if (flat) {
        b->w.clearBoxes();
        b->w.flatten();
        b->w.pave();
      }
      b->s.world = &b->w;
      b->c = buildHummer(b->s, at, &b->r);
      b->s.finalize();
      settleVehicle(b->s, b->c, 4.0);
      return b;
    };
    auto step = [&](B* x, double sec, std::function<void(double, Controls&)> fn) {
      Controls c;
      for (int i = 0, n = int(sec / kDt); i < n; i++) {
        fn(i * kDt, c);
        x->s.forces(kDt);
        x->c.update(x->s, c, kDt);
        x->s.finish(kDt);
        c.fire = false;
        c.requestGear = 99;
      }
      x->s.events.clear();
    };
    B* a = make({-400, 0, 0}, true);
    double low = 9;
    for (u32 i = 0; i < a->s.objs[0].nodeCount; i++)
      if (a->s.flags[i] & NF_COLLIDE && !(a->s.flags[i] & NF_WHEEL)) low = std::min(low, a->s.pos[i].z);
    printf("  Masse %.0f kg  Eigenwert %.2f  Bodenfreiheit Struktur %.3f m  Radlast %.0f %.0f %.0f %.0f  Federweg L %.3f (nom %.3f)  Hub z %.3f  verformt %.0f J  gerissen %u  guard %llu\n", a->c.totalMass, a->s.maxEigenvalue(0) * 0.25 * kDt * kDt, low, a->c.wheels[0].Fz, a->c.wheels[1].Fz, a->c.wheels[2].Fz, a->c.wheels[3].Fz, a->c.axle[0].s[0].L, a->c.axle[0].s[0].Lnom, a->c.wheels[0].hub.z, a->s.plasticEnergy, a->s.brokenBeams, (unsigned long long)a->s.clampedSteps);
    double t60 = -1, t100 = -1, tn = 0, vm = 0;
    step(a, 45, [&](double, Controls& c) {
      c.throttle = 1;
      tn += kDt;
      const double k = a->c.speed * 3.6;
      vm = std::max(vm, k);
      if (t60 < 0 && k > 60) t60 = tn;
      if (t100 < 0 && k > 100) t100 = tn;
    });
    printf("  0-60 %.1f s  0-100 %.1f s (echt etwa 13)  Spitze %.0f km/h (echt um 130)  Gang %d  %.0f U/min\n", t60, t100, vm, a->c.gear, a->c.rpm());
    {
      B* b = make({-400, 0, 0}, true);
      const Obj& o = b->s.objs[0];
      for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) b->s.vel[i] = V3{100 / 3.6, 0, 0};
      b->c.presetRolling(100 / 3.6, 3);
      const V3 p0 = b->c.cog;
      step(b, 8, [&](double, Controls& c) { c.brake = 1; });
      printf("  100-0 %.1f m\n", len(b->c.cog - p0));
    }
    // The tracks: a pursuit driver from the mountain's foot to the top, for both of them.
    for (int trailIdx = 0; trailIdx < 2; trailIdx++) {
      for (int low2 = 0; low2 < 2; low2++) {
        B* b = make({-90, 0, 0}, false);
        b->c.tune.lowRange = low2 != 0;
        b->c.tune.lockDiffs = low2 != 0;
        const auto& roads = b->w.roads();
        const Road& tr = roads[roads.size() - 2 + size_t(trailIdx)];
        // Find where the track really starts to climb.
        size_t start = 0;
        for (size_t i = 0; i < tr.pts.size(); i++)
          if (tr.pts[i].z > 3.0) {
            start = i;
            break;
          }
        double maxG = 0;
        for (size_t i = start; i + 20 < tr.pts.size(); i++) {
          double run = 0;
          for (size_t k = i; k < i + 20; k++) run += std::hypot(tr.pts[k + 1].x - tr.pts[k].x, tr.pts[k + 1].y - tr.pts[k].y);
          maxG = std::max(maxG, std::fabs(tr.pts[i + 20].z - tr.pts[i].z) / std::max(run, 1.0));
        }
        const auto& s0 = tr.pts[start];
        const auto& s1 = tr.pts[start + 6];
        const double yaw = std::atan2(s1.y - s0.y, s1.x - s0.x), cy = std::cos(yaw), sy = std::sin(yaw);
        const Obj oo = b->s.objs[0];
        const V3 at{s0.x, s0.y, b->w.height(s0.x, s0.y) + 0.30};
        for (u32 k = oo.nodeStart; k < oo.nodeStart + oo.nodeCount; k++) {
          const V3 q = b->s.pos[k] - V3{-90, 0, 0};
          b->s.pos[k] = at + V3{cy * q.x - sy * q.y, sy * q.x + cy * q.y, q.z};
          b->s.vel[k] = {};
        }
        step(b, 1.0, [&](double, Controls& c) { c.brake = 1; });
        size_t best = start;
        double tTop = -1, tt = 0, zMax = -1e9, offMax = 0;
        const double z0 = b->c.cog.z;
        step(b, 300, [&](double, Controls& c) {
          tt += kDt;
          double bd = 1e18;
          // Follows the track forward only: the next traverse may lie just across the bank.
          const size_t hi = std::min(best + 24, tr.pts.size() - 1);
          for (size_t k = best; k <= hi; k++) {
            const double d = std::hypot(tr.pts[k].x - b->c.cog.x, tr.pts[k].y - b->c.cog.y);
            if (d < bd) {
              bd = d;
              best = k;
            }
          }
          offMax = std::max(offMax, bd);
          zMax = std::max(zMax, b->c.cog.z);
          const size_t aim = std::min(best + size_t((5.0 + std::fabs(b->c.speed) * 0.35) / 1.4), tr.pts.size() - 1);
          const RoadPoint& g = tr.pts[aim];
          double err = std::atan2(g.y - b->c.cog.y, g.x - b->c.cog.x) - std::atan2(b->c.fwd.y, b->c.fwd.x);
          while (err > PI) err -= 2 * PI;
          while (err < -PI) err += 2 * PI;
          c.steer = clampd(err * 2.2, -1, 1);
          // Looks further ahead for the next bend, and slows for it.
          const RoadPoint& far = tr.pts[std::min(best + 22, tr.pts.size() - 1)];
          double bend = std::atan2(far.y - g.y, far.x - g.x) - std::atan2(b->c.fwd.y, b->c.fwd.x);
          while (bend > PI) bend -= 2 * PI;
          while (bend < -PI) bend += 2 * PI;
          const double vT = std::fabs(bend) > 0.5 || std::fabs(err) > 0.3 ? 3.5 : 9.0;
          c.throttle = clampd((vT - b->c.speed) * 0.7, 0, 1);
          c.brake = clampd((b->c.speed - vT - 0.5) * 0.3, 0, 0.8);
          if (tTop < 0 && best + 12 >= tr.pts.size()) tTop = tt;
        });
        printf("  %s, %s: Spur-Hoehe %.0f..%.0f m, max Steigung %.0f %%, Strecke %.1f km | Gipfel nach %.0f s, erreicht z +%.0f m, Fortschritt %.0f %%, abseits max %.1f m, verformt %.1f kJ, Reifen platt %d, guard %llu\n", trailIdx == 0 ? "Serpentinen" : "Steilpfad", low2 ? "Untersetzung+Sperre" : "normal", tr.pts[start].z, tr.pts.back().z, maxG * 100, double(tr.pts.size()) * 1.4 / 1000.0, tTop, zMax - z0, 100.0 * (double(best) - double(start)) / double(tr.pts.size() - start), offMax, b->s.plasticEnergy / 1e3, int(b->c.wheels[0].deflate > 0) + int(b->c.wheels[1].deflate > 0) + int(b->c.wheels[2].deflate > 0) + int(b->c.wheels[3].deflate > 0), (unsigned long long)b->s.clampedSteps);
        delete b;
      }
    }
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "one1grip")) {
    // Pulling away on every surface, full throttle, with all the aids on.
    for (int surface = 0; surface < 3; surface++) {
      auto* w = new World();
      auto* s = new Sim();
      CarRig r;
      w->build();
      w->clearBoxes();
      w->flatten();
      if (surface == 0) w->pave();
      const V3 at = surface == 2 ? V3{-400, 0, 0} : V3{-100, 0, 0};  // grass out there, concrete on the pad
      s->world = w;
      Vehicle c0 = buildOne1(*s, at, &r);
      s->finalize();
      settleVehicle(*s, c0, 3.0);
      Controls c;
      c.throttle = 1;
      printf("%s:", surface == 0 ? "Asphalt" : (surface == 1 ? "Beton" : "Gras"));
      for (int i = 0; i < 2000 * 8; i++) {
        s->forces(kDt);
        c0.update(*s, c, kDt);
        s->finish(kDt);
        if (i % 4000 == 3999) printf("  %.0f s: %.0f km/h (gear %d, %.0f rpm)", (i + 1) * kDt, c0.speed * 3.6, c0.gear, c0.rpm());
      }
      printf("\n");
    }
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "one1run")) {
    for (int tcs = 1; tcs >= 0; tcs--) {
      auto* w = new World();
      auto* s = new Sim();
      CarRig r;
      w->build();
      w->clearBoxes();
      w->flatten();
      w->pave();
      s->world = w;
      Vehicle c0 = buildOne1(*s, {-400, 0, 0}, &r);
      s->finalize();
      settleVehicle(*s, c0, 4.0);
      c0.tune.tcs = tcs;
      c0.tune.esp = tcs;
      Controls c;
      c.throttle = 1;
      printf("TCS+ESP %d\n", tcs);
      for (int i = 0; i < 2000 * 8; i++) {
        s->forces(kDt);
        c0.update(*s, c, kDt);
        s->finish(kDt);
        if (i % 800 == 799)
          printf(" t %.1f v %5.1f gear %d phase %d rpm %5.0f th %.2f boost %.2f T %5.0f | RL om*R %5.1f RR %5.1f Fz %5.0f %5.0f FL %5.0f | head %+.2f y %+.2f roll %+.1f tcs %d esp %d\n", (i + 1) * kDt, c0.speed * 3.6, c0.gear, c0.shiftPhase, c0.rpm(), c0.throttleAct, c0.boost, c0.combustionTorque, c0.wheels[2].omega * c0.wheels[2].R, c0.wheels[3].omega * c0.wheels[3].R, c0.wheels[2].Fz, c0.wheels[3].Fz, c0.wheels[0].Fz, std::atan2(c0.fwd.y, c0.fwd.x) * 57.3, c0.cog.y, c0.rollAngle * 57.3, c0.tcsActive, c0.espActive);
      }
    }
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "one1")) {
    printf("== Koenigsegg One:1 ==\n");
    auto make = [](V3 at, bool flat) {
      struct B {
        World w;
        Sim s;
        Vehicle c;
        CarRig r;
      };
      B* b = new B();
      b->w.build();
      if (flat) {
        b->w.clearBoxes();
        b->w.flatten();
        b->w.pave();
      }
      b->s.world = &b->w;
      b->c = buildOne1(b->s, at, &b->r);
      b->s.finalize();
      settleVehicle(b->s, b->c, 4.0);
      return b;
    };
    auto* B = make({-400, 0, 0}, true);
    auto step = [&](auto* x, double sec, auto fn) {
      Controls c;
      for (int i = 0, n = int(sec / kDt); i < n; i++) {
        fn(i * kDt, c);
        x->s.forces(kDt);
        x->c.update(x->s, c, kDt);
        x->s.finish(kDt);
        c.fire = false;
        c.requestGear = 99;
      }
      x->s.events.clear();
    };
    double low = 9;
    for (u32 i = 0; i < B->s.objs[0].nodeCount; i++) low = std::min(low, B->s.pos[i].z);
    printf("  Radlast FL %.0f FR %.0f RL %.0f RR %.0f\n", B->c.wheels[0].Fz, B->c.wheels[1].Fz, B->c.wheels[2].Fz, B->c.wheels[3].Fz);
    printf("  mass %.0f kg  eigen ratio %.2f  lowest node z %.3f  Fz F %.0f R %.0f  plastic %.1f J  broken %u  guard %llu\n", B->c.totalMass, B->s.maxEigenvalue(0) * 0.25 * kDt * kDt, low, B->c.wheels[0].Fz, B->c.wheels[2].Fz, B->s.plasticEnergy, B->s.brokenBeams, (unsigned long long)B->s.clampedSteps);
    double t100 = -1, t200 = -1, t300 = -1, tNow = 0, vmax = 0;
    step(B, 40.0, [&](double, Controls& c) {
      c.throttle = 1;
      tNow += kDt;
      const double k = B->c.speed * 3.6;
      vmax = std::max(vmax, k);
      if (t100 < 0 && k > 100) t100 = tNow;
      if (t200 < 0 && k > 200) t200 = tNow;
      if (t300 < 0 && k > 300) t300 = tNow;
    });
    printf("  0-100 %.2f s (real 2.8), 0-200 %.2f s, 0-300 %.2f s (real ~11.9), v after 40 s %.0f km/h, gear %d rpm %.0f, y drift %.1f, guard %llu\n", t100, t200, t300, vmax, B->c.gear, B->c.rpm(), B->c.cog.y, (unsigned long long)B->s.clampedSteps);
    {
      auto* C = make({-150, 0, 0}, true);
      C->c.presetRolling(100 / 3.6, 3);
      const Obj& o = C->s.objs[0];
      for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) C->s.vel[i] = V3{100 / 3.6, 0, 0};
      const V3 p0 = C->c.cog;
      step(C, 5.0, [&](double, Controls& c) { c.brake = 1; });
      printf("  100-0 %.1f m (real about 28 m), v %.1f\n", len(C->c.cog - p0), C->c.speed * 3.6);
    }
    {
      auto* C = make({-150, 0, 0}, true);
      for (const double steer : {0.10}) {
        const double vT = 22.0;
        C->c.presetRolling(vT, 2);
        const Obj& o = C->s.objs[0];
        for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) C->s.vel[i] = V3{vT, 0, 0};
        double lat = 0, rollMax = 0;
        step(C, 4.0, [&](double t, Controls& c) {
          c.steer = std::min(t / 1.0, 1.0) * steer * 2.6;
          c.throttle = clampd((vT - C->c.speed) * 0.5, 0, 1);
          lat = std::max(lat, std::fabs(C->c.latAcc));
          rollMax = std::max(rollMax, std::fabs(C->c.rollAngle));
        });
        printf("  cornering at 80 km/h: peak lat %.2f g, roll %.2f deg, end v %.0f km/h\n", lat / 9.81, rollMax * 57.3, C->c.speed * 3.6);
      }
    }
    for (double kmh : {50.0, 100.0}) {
      auto* C = make({85, -30, 0}, false);
      const Obj o = C->s.objs[0];
      for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) C->s.vel[i] = V3{kmh / 3.6, 0, 0};
      C->c.presetRolling(kmh / 3.6, 2);
      step(C, 3.0, [&](double, Controls&) {});
      printf("  wall at %3.0f km/h: plastic %.0f kJ, broken %u, pieces %zu, cabin %.3f m (was %.3f), guard %llu, sane %d\n", kmh, C->s.plasticEnergy / 1e3, C->s.brokenBeams, C->s.objs.size(), len(C->s.pos[C->c.refFrontL] - C->s.pos[C->c.refRearL]), len(C->s.rest[C->c.refFrontL] - C->s.rest[C->c.refRearL]), (unsigned long long)C->s.clampedSteps, std::isfinite(C->c.cog.x));
    }
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "bots")) {
    // Two to six bots against each other, without a server: does it come to a fight, and to an end?
    const int count = argc > 2 ? std::clamp(atoi(argv[2]), 2, int(Duel::kMaxPlayers)) : 2;
    const double most = argc > 3 ? atof(argv[3]) : 400.0;
    const bool cut = argc > 4 && (!strcmp(argv[4], "cut") || !strcmp(argv[4], "ford"));  // with every bridge down from the start
    const bool idle = argc > 4 && !strcmp(argv[4], "ford");  // and the other side just standing there, out of sight until one is close
    // What a guided missile does to the tank bot 0 is in: square on the nose, again and again, as long as it sits in one.
    const bool struck = argc > 4 && !strcmp(argv[4], "hit");
    const bool airOnly = argc > 4 && !strcmp(argv[4], "air");  // only helicopters: do they get up, and do they find anyone
    const int rounds = argc > 5 ? atoi(argv[5]) : 2;
    double nextStrike = 40.0;
    int strikes = 0;
    const int strikesMost = argc > 6 ? atoi(argv[6]) : 1000;
    auto* d = new Duel();
    d->playersWanted = count;
    d->build();
    std::vector<Bot> bots;
    for (int i = 0; i < count; i++) {
      bots.push_back(Bot(u32(i + 1)));
      if (airOnly && i % 2 == 0) bots.back().forceTactic(Bot::AIR);
      d->players[i].present = true;
    }
    Duel::Intent in[Duel::kMaxPlayers];
    double t = 0, nextSay = 0;
    u32 winsSeen = 0;
    const u32 startMatch = d->matchId;
    bool cutDone = false;
    const auto wall0 = std::chrono::steady_clock::now();
    while (t < most) {
      for (int i = 0; i < count; i++) {
        if (idle && i % 2 == 1) {
          in[i] = Duel::Intent{};
          u8 hide[IT_COUNT] = {};
          hide[IT_GHILLIE] = 1;
          if (d->phase == Duel::CHOOSING) d->choose(i, hide, true);
          continue;
        }
        bots[size_t(i)].step(*d, i, in[i]);
      }
      d->step(in);
      std::vector<SimEvent> ev;
      d->drainEvents(ev);
      t += Duel::kDt;
      if (cut && !cutDone && d->phase == Duel::PLAYING) {
        cutDone = true;
        for (const World::Bridge& br : d->world.bridgeSpans()) d->sim.explode(br.at + V3{std::cos(br.yaw) * 6.0, std::sin(br.yaw) * 6.0, 0.1}, 12000.0, 30.0);
        printf("Alle Bruecken unten: Maske %x\n", d->world.bridgeMask());
      }
      for (const Duel::KillNote& note : d->killNotes) printf("-> Kill: Spieler %d an %d, Waffe %d\n", note.killer, note.victim, int(note.weapon));
      d->killNotes.clear();
      if (struck && strikes < strikesMost && t >= nextStrike && d->phase == Duel::PLAYING && d->players[0].vehicle >= 0 && d->players[0].vehicle < Duel::kTanks) {
        nextStrike = t + 25.0;
        strikes++;
        Tank& hit = d->tanks[d->players[0].vehicle];
        const double lower = hit.damage;
        d->sim.blastBy = 1;
        d->sim.explode(hit.cog + hit.fwd * 3.8, 5000.0, 20.0);
        printf("  Lenkrakete auf Panzer %d bei t %.0f, Schaden vorher %.2f\n", d->players[0].vehicle, t, lower);
      }
      if (t >= nextSay) {
        nextSay = t + (struck ? 3.0 : 10.0);
        printf("t %3.0f Phase %d Match %u Stand %u:%u", t, int(d->phase), d->matchId, d->wins[0], d->wins[1]);
        for (int i = 0; i < count; i++) {
          const Duel::Player& pl = d->players[i];
          const V3 p = pl.vehicle >= 0 ? d->frames[pl.vehicle].cog : pl.soldier.p;
          printf(" | %d %s/%s: %.0f %.0f fz %d hp %.0f K%d T%d", i, Bot::tacticName(bots[size_t(i)].tactic()), bots[size_t(i)].doing(), p.x, p.y, pl.vehicle, pl.soldier.hp, int(d->scores[i].kills), int(d->scores[i].deaths));
          if (pl.vehicle >= 0 && pl.vehicle < Duel::kTanks) {
            const Tank& tk = d->tanks[pl.vehicle];
            printf(" sch %.2f mun %d v %.1f", tk.damage, tk.ammo, d->frames[pl.vehicle].speed);
            if (struck) printf(" gas %.2f brems %.2f lenk %.2f gang %d mods %.1f %.1f %.1f %.1f", in[i].c.throttle, in[i].c.brake, in[i].c.steer, tk.gear, tk.mod[TM_RING], tk.mod[TM_RADIATOR], tk.mod[TM_FUEL], tk.radiatorHeat);
            if (struck) printf(" motor %.2f ketten %d%d benzin %.0f set %d", tk.mod[TM_ENGINE], int(tk.trackOff[0]), int(tk.trackOff[1]), tk.fuel, int(pl.items[IT_REPAIR]));
          }
        }
        printf("\n");
        fflush(stdout);
      }
      if (d->wins[0] + d->wins[1] > winsSeen) {
        winsSeen = d->wins[0] + d->wins[1];
        printf("-> Match entschieden nach %.0f s: Stand %u:%u\n", t, d->wins[0], d->wins[1]);
      }
      if (d->matchId > startMatch + rounds * 2) break;
    }
    for (int i = 0; i < count; i++) printf("Bot %d: Kills %d, Tode %d, Spuren gefolgt %d, Kisten %d, Rechner %d, Rettungen %d, Fluege %d\n", i, int(d->scores[i].kills), int(d->scores[i].deaths), bots[size_t(i)].followedTracks(), bots[size_t(i)].cratesTaken(), bots[size_t(i)].computersTapped(), bots[size_t(i)].timesRescued(), bots[size_t(i)].flights());
    printf("Fertig: %.0f s gespielt in %.0f s Rechenzeit, Stand %u:%u, Matches %u\n", t, std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count(), d->wins[0], d->wins[1], d->matchId - startMatch);
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "gatcar")) {
    // The gatling on a car with someone in it: what goes when, at twenty rounds a second, a burst at a time.
    auto* d = new Duel();
    d->playersWanted = 2;
    d->build();
    d->players[0].present = d->players[1].present = true;
    Duel::Intent in[Duel::kMaxPlayers];
    const auto run = [&](double seconds) {
      for (double t = 0; t < seconds; t += Duel::kDt) {
        d->step(in);
        std::vector<SimEvent> ev;
        d->drainEvents(ev);
      }
    };
    u8 none[IT_COUNT] = {};
    while (d->phase != Duel::CHOOSING) run(0.1);
    d->choose(0, none, true);
    d->choose(1, none, true);
    while (d->phase != Duel::PLAYING) run(0.05);
    const int k = argc > 2 ? atoi(argv[2]) : 0;
    Vehicle& car = d->cars[size_t(k)];
    const VehicleFrame& f = d->frames[Duel::kTanks + k];
    d->players[0].soldier.p = f.cog + f.left * 2.0;
    in[0].use = true;
    run(Duel::kDt * 0.5);
    in[0].use = false;
    printf("Im Auto %d: %d\n", k, int(d->players[0].vehicle == Duel::kTanks + k));
    const auto say = [&](double t) {
      const auto& r = d->carRounds[k];
      printf("t %4.1f Treffer %2d Tank %d brennt %.1f | Motor fest %d Kuehler %d Oel %d Tank %d Getriebe %d Benzin %.0f Reifen %d%d%d%d | zerstoert %d Fahrer %.0f\n", t, r.hits, r.tankHits, r.burning, int(car.dmg.seized), int(car.dmg.radiator), int(car.dmg.oilLeak), int(car.dmg.tank), int(car.dmg.gearbox), car.fuelLitres, car.wheels[0].deflate > 0, car.wheels[1].deflate > 0, car.wheels[2].deflate > 0, car.wheels[3].deflate > 0, int(car.exploded), d->players[0].soldier.hp);
    };
    double t = 0;
    say(t);
    u32 seed = 7;
    for (int shot = 0; shot < 400 && !car.exploded; shot++) {
      // Roughly at the middle of it, from up front and above, the way it spreads.
      seed = seed * 1664525u + 1013904223u;
      const double a = double(seed >> 16) / 65535.0 * 6.283;
      seed = seed * 1664525u + 1013904223u;
      const double r = double(seed >> 16) / 65535.0 * 1.6;
      const V3 aim = f.cog + f.fwd * (std::cos(a) * r) + f.left * (std::sin(a) * r * 0.5);
      const V3 from = aim + f.fwd * 60.0 + V3{0, 0, 45.0};
      Duel::Bullet b;
      b.p = from;
      b.v = norm(aim - from) * kWeapons[WP_AR].speed;
      b.owner = 1;
      b.kind = WP_AR;
      b.gatling = true;
      d->bullets.push_back(b);
      run(0.05);
      t += 0.05;
      if (shot % 10 == 9) say(t);
      if (d->carRounds[k].burning >= 0) {
        // On fire now: no more shooting, to see what the fire does by itself.
        for (int w = 0; w < 24 && !car.exploded; w++) {
          run(0.5);
          t += 0.5;
          if (w % 4 == 3) say(t);
        }
        break;
      }
    }
    say(t);
    printf("Kills Schuetze %d, Tode Fahrer %d\n", int(d->scores[1].kills), int(d->scores[0].deaths));
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "gunwall")) {
    // A tank beside the command centre: the barrel swings against the wall and stops; pushed through it, the shell stays this side.
    auto* d = new Duel();
    d->playersWanted = 2;
    d->build();
    d->players[0].present = d->players[1].present = true;
    Duel::Intent in[Duel::kMaxPlayers];
    const auto run = [&](double seconds) {
      for (double t = 0; t < seconds; t += Duel::kDt) {
        d->step(in);
        std::vector<SimEvent> ev;
        d->drainEvents(ev);
      }
    };
    u8 none[IT_COUNT] = {};
    while (d->phase != Duel::CHOOSING) run(0.1);
    d->choose(0, none, true);
    d->choose(1, none, true);
    while (d->phase != Duel::PLAYING) run(0.05);
    const int v = Duel::tankIndex(0, TK_STANDARD, HK_MEDIUM);
    Tank& t = d->tanks[v];
    const World::Base& base = d->world.base(0);
    const V3 ahead{std::cos(base.facing), std::sin(base.facing), 0}, side{-ahead.y, ahead.x, 0};
    // The front wall of the command centre lies 16.4 m behind the middle of the yard.
    const double wall = -26.0 + 9.0 + 0.6;
    const auto moveTo = [&](double aheadOf, double along) {
      const V3 to = base.centre + ahead * aheadOf + side * along;
      const V3 by{to.x - t.cog.x, to.y - t.cog.y, 0};
      for (u16 oi : {t.hullObj, t.turretObj})
        for (u32 n : d->sim.objs[oi].nodes) d->sim.pos[n] += by;
    };
    moveTo(wall + 7.0, 8.0);
    d->players[0].soldier.p = t.cog + side * 3.0;
    in[0].use = true;
    run(Duel::kDt * 0.5);
    in[0].use = false;
    printf("eingestiegen %d\n", int(d->players[0].vehicle == v));
    in[0].c.brake = 1;
    const auto wallGap = [&](V3 p) { return dot(p - base.centre, ahead) - wall; };
    // First along the wall, and then swung towards it: it has to stop short.
    moveTo(wall + 4.4, 8.0);
    run(0.5);
    in[0].c.aimPoint = t.cog + side * 60.0 + V3{0, 0, 1.0};
    run(4.0);
    printf("laengs der Wand: Muendung %.2f m vor der Wand\n", wallGap(t.muzzle));
    in[0].c.aimPoint = t.cog - ahead * 60.0 + V3{0, 0, 1.0};
    run(5.0);
    printf("zur Wand geschwenkt: Muendung %.2f m vor der Wand, Rohrrichtung zur Wand %.2f\n", wallGap(t.muzzle), -dot(norm(t.gunDir), ahead));
    moveTo(wall + 7.0, 8.0);
    in[0].c.aimPoint = t.cog + ahead * 40.0 + V3{0, 0, 1.0};
    run(3.0);
    printf("zeigt weg von der Wand: Muendung %.2f m vor der Wand\n", wallGap(t.muzzle));
    in[0].c.aimPoint = t.cog - ahead * 60.0 + V3{0, 0, 1.0};
    run(5.0);
    printf("zur Wand gedreht: Muendung %.2f m vor der Wand, Rohrrichtung zur Wand %.2f\n", wallGap(t.muzzle), -dot(norm(t.gunDir), ahead));
    in[0].c.fire = true;
    run(Duel::kDt * 0.5);
    in[0].c.fire = false;
    size_t seen = d->sim.blastLog.size();
    run(1.0);
    for (size_t k = seen; k < d->sim.blastLog.size(); k++) printf("Einschlag %.2f m vor der Wand (negativ: dahinter)\n", wallGap(d->sim.blastLog[k].c));
    // And the same with the tank shoved up so close that the barrel has to be through it.
    moveTo(wall + 1.5 + 3.6, 8.0);
    run(0.3);
    printf("dicht an der Wand: Muendung %.2f m vor der Wand\n", wallGap(t.muzzle));
    seen = d->sim.blastLog.size();
    run(t.reload + 0.2);
    in[0].c.fire = true;
    run(Duel::kDt * 0.5);
    in[0].c.fire = false;
    run(1.0);
    for (size_t k = seen; k < d->sim.blastLog.size(); k++) printf("Einschlag %.2f m vor der Wand (negativ: dahinter)\n", wallGap(d->sim.blastLog[k].c));
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "heli")) {
    // Up, along, down again; then what brings it down: bullets on the parts that matter, and a shell of any size.
    auto* d = new Duel();
    d->playersWanted = 2;
    d->build();
    d->players[0].present = d->players[1].present = true;
    Duel::Intent in[Duel::kMaxPlayers];
    const auto run = [&](double seconds) {
      for (double t = 0; t < seconds; t += Duel::kDt) {
        d->step(in);
        std::vector<SimEvent> ev;
        d->drainEvents(ev);
      }
    };
    const auto begin = [&] {
      in[0] = in[1] = Duel::Intent{};
      u8 none[IT_COUNT] = {};
      while (d->phase != Duel::CHOOSING) run(0.1);
      d->choose(0, none, true);
      d->choose(1, none, true);
      while (d->phase != Duel::PLAYING) run(0.05);
      // Next to it, and in.
      d->players[0].soldier.p = d->helis[0].p + V3{2, 0, 0};
      in[0].use = true;
      run(Duel::kDt * 0.5);
      in[0].use = false;
      printf("eingestiegen %d, Hubschrauber %d\n", int(d->players[0].heli == 0), 0);
    };
    Heli& H = d->helis[0];
    const auto say = [&](const char* what) {
      printf("%-22s z ueber Boden %6.1f v %5.1f Leistung %.2f Motor %d Rotor %d Heck %d Muni %3d Absturz %d tot %d Pilot Leben %.0f\n", what, H.p.z - d->world.height(H.p.x, H.p.y), len(H.v), H.power, H.engine, H.rotor, H.tail, H.ammo, int(H.crashing), int(H.dead), d->players[0].soldier.hp);
    };
    const auto face = [&] {
      in[0].walk.yaw = H.yaw + PI;
      in[0].pitch = 0.05;
    };
    begin();
    face();
    run(2.5);
    say("Rotor laeuft an");
    in[0].walk.jump = true;
    run(8.0);
    in[0].walk.jump = false;
    say("gestiegen");
    in[0].walk.forward = 1;
    in[0].fireHeld = true;
    run(5.0);
    in[0].fireHeld = false;
    in[0].walk.forward = 0;
    say("vorwaerts, Feuer");
    run(6.0);
    say("schwebt");
    in[0].walk.crouch = true;
    for (double t = 0; t < 40 && !H.onGround; t += 0.1) run(0.1);
    in[0].walk.crouch = false;
    say("gelandet");
    // A rifle at the engine, from nearby, while it hovers.
    const auto shootAt = [&](V3 target, int shots) {
      for (int k = 0; k < shots && !H.dead; k++) {
        const V3 from = target + H.left() * 30.0;
        Duel::Bullet b;
        b.p = from;
        b.v = norm(target - from) * kWeapons[WP_AR].speed;
        b.owner = 1;
        b.kind = WP_AR;
        d->bullets.push_back(b);
        run(0.15);
      }
    };
    in[0].walk.jump = true;
    run(5.0);
    in[0].walk.jump = false;
    run(4.0);
    say("wieder oben");
    shootAt(H.at(-0.4, 0, 2.4), 5);
    say("5 Schuss Motor");
    shootAt(H.at(-0.4, 0, 2.4), 1);
    say("6. Schuss Motor");
    for (double t = 0; t < 30 && !H.dead; t += 0.1) run(0.1);
    say("abgestuerzt");
    printf("Kills Schuetze %d, Tode Pilot %d\n", int(d->scores[1].kills), int(d->scores[0].deaths));
    // Round again: down on the ground, a kit puts it right; whoever walks into the rotor or under it does not get away.
    d->restart();
    d->phase = Duel::CHOOSING;
    begin();
    face();
    d->helis[0].engine = 0;
    d->helis[0].crashing = true;
    say("Motor kaputt am Boden");
    in[0].use = true;
    run(Duel::kDt * 0.5);
    in[0].use = false;
    d->players[0].items[IT_REPAIR] = 1;
    d->players[0].soldier.p = H.p + V3{3, 0, 0};
    in[0].repair = true;
    run(Duel::kDt * 0.5);
    in[0].repair = false;
    run(6.5);
    say("repariert");
    printf("Sets uebrig %d, wieder einsteigbar %d\n", int(d->players[0].items[IT_REPAIR]), int(d->nearestHeli(d->players[0].soldier.p) == 0));
    d->players[0].soldier.p = H.p + V3{3, 0, 0};
    in[0].use = true;
    run(Duel::kDt * 0.5);
    in[0].use = false;
    in[0].walk.jump = true;
    run(3.8);
    in[0].walk.jump = false;
    H.v = V3{};
    H.p.z = d->world.height(H.p.x, H.p.y);
    H.onGround = true;
    d->players[1].soldier.p = V3{H.p.x + 3.0, H.p.y, d->world.height(H.p.x + 3.0, H.p.y)};
    d->players[1].soldier.v = V3{};
    d->players[1].soldier.crouch = 1.0;
    in[1].walk.crouch = true;
    run(0.05);
    printf("Geduckt unter dem laufenden Rotor: Leben %.0f\n", d->players[1].soldier.hp);
    in[1].walk.crouch = false;
    run(0.5);
    say("am Boden, Rotor laeuft");
    printf("Gegner unter dem Rotor auf Kopfhoehe: Leben %.0f, Kills Pilot %d, Hubschrauber z %.1f\n", d->players[1].soldier.hp, int(d->scores[0].kills), H.p.z - d->world.height(H.p.x, H.p.y));
    d->players[1].soldier.hp = 100;
    // And a vehicle in the way.
    d->restart();
    d->phase = Duel::CHOOSING;
    begin();
    face();
    in[0].walk.jump = true;
    run(4.2);
    in[0].walk.jump = false;
    in[0].walk.forward = 1;
    H.p = d->frames[0].cog + V3{0, -30, -1.0};
    H.v = V3{};
    H.yaw = PI * 0.5;
    in[0].walk.yaw = H.yaw + PI;
    for (double t = 0; t < 6 && !H.dead; t += 0.05) run(0.05);
    say("gegen einen Panzer");
    // Round again: a small shell, right on it, and that is all it takes.
    d->restart();
    d->phase = Duel::CHOOSING;
    begin();
    face();
    in[0].walk.jump = true;
    run(7.0);
    in[0].walk.jump = false;
    say("oben");
    d->sim.blastBy = 1;
    d->sim.blastIsShell = true;
    d->sim.explode(H.at(0, 0, 1.3), 170.0, 4.0);
    run(0.05);
    say("leichte Granate");
    run(8.0);
    say("Wrack unten");
    printf("Kills Schuetze %d, Tode Pilot %d\n", int(d->scores[1].kills), int(d->scores[0].deaths));
    // Round again: out in the air, without a parachute.
    d->restart();
    d->phase = Duel::CHOOSING;
    begin();
    face();
    in[0].walk.jump = true;
    run(7.0);
    in[0].walk.jump = false;
    say("oben");
    in[0].use = true;
    run(Duel::kDt * 0.5);
    in[0].use = false;
    run(8.0);
    printf("Abgesprungen: Leben %.0f, Tode %d\n", d->players[0].soldier.hp, int(d->scores[0].deaths));
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "bridge")) {
    // What it takes to bring one down, that a tank then falls, and that the next match has it back.
    auto* d = new Duel();
    d->build();
    const World::Bridge& br = d->world.bridgeSpans()[0];
    printf("%zu Bruecken, je %zu Felder\n", d->world.bridgeSpans().size(), br.spans.size());
    const double cy = std::cos(br.yaw), sy = std::sin(br.yaw);
    const V3 onDeck = br.at + V3{cy * 6.0, sy * 6.0, 0.1};
    printf("Boden auf dem Feld vorher %.1f (Deck %.1f)\n", d->world.groundUnder(onDeck.x, onDeck.y, onDeck.z + 2.0), br.at.z);
    int shots = 0;
    while (d->world.bridgeSpans()[0].open() && shots < 50) {
      d->sim.explode(onDeck, 2200.0, 15.0);
      shots++;
    }
    printf("105 mm: %d Treffer, Boden danach %.1f, Maske %x, Strasse weg %d\n", shots, d->world.groundUnder(onDeck.x, onDeck.y, onDeck.z + 2.0), d->world.bridgeMask(), int(d->world.overFallenSpan(onDeck.x, onDeck.y)));
    d->world.clearCraters();
    printf("Neues Match: Boden %.1f, Maske %x\n", d->world.groundUnder(onDeck.x, onDeck.y, onDeck.z + 2.0), d->world.bridgeMask());
    d->sim.explode(onDeck, 12000.0, 30.0);
    printf("Artillerie: Maske %x\n", d->world.bridgeMask());
    d->world.clearCraters();
    // A second world, told only by the mask, comes to the same.
    d->world.applyBridgeMask(5);
    printf("Maske uebernommen: %x, Feld 2 Boden %.1f\n", d->world.bridgeMask(), d->world.groundUnder(onDeck.x, onDeck.y, onDeck.z + 2.0));
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "armour")) {
    // The same shell at the same tank from all round: square from in front, the side and behind, then flat along the side.
    auto* w = new World();
    w->build();
    w->clearBoxes();
    auto* start = new Sim();
    start->world = w;
    CarRig r;
    Tank target = buildLeopard(*start, V3{60, 0, 0}, &r);
    Tank shooter = buildLeopard(*start, V3{60, 60, 0}, &r);
    start->finalize();
    for (int i = 0; i < 6000; i++) {
      Controls still;
      still.brake = 1;
      start->forces(kDt);
      target.update(*start, still, kDt);
      shooter.update(*start, still, kDt);
      start->finish(kDt);
    }
    auto* s = new Sim();
    const Tank target0 = target, shooter0 = shooter;
    const auto fire = [&](const char* name, V3 from, V3 to) {
      *s = *start;
      Tank tanks[2] = {target0, shooter0};
      Tank::fleet = tanks;
      Tank::fleetSize = 2;
      Tank::Shell shell;
      shell.p = from;
      shell.v = norm(to - from) * 1100.0;
      tanks[1].shells.push_back(shell);
      Controls c;
      for (int i = 0; i < 400; i++) {
        s->forces(kDt);
        tanks[0].update(*s, c, kDt);
        tanks[1].update(*s, c, kDt);
        s->finish(kDt);
      }
      printf("%-10s Schaden %.3f, Abpraller %d, Sprengungen %u\n", name, tanks[0].damage, int(tanks[1].bounces), s->blastCount);
      Tank::fleet = nullptr;
      Tank::fleetSize = 0;
    };
    const V3 mid = target.cog + V3{0, 0, 0.45}, ahead = norm(V3{target.fwd.x, target.fwd.y, 0}), side = norm(V3{target.left.x, target.left.y, 0});
    // Low on the hull, away from the turret, which has a front of its own.
    fire("vorn", mid + ahead * 40.0, mid + ahead * 3.0);
    fire("Seite", mid + ahead * 2.0 + side * 40.0, mid + ahead * 2.0);
    fire("hinten", mid - ahead * 40.0, mid - ahead * 3.0);
    fire("schraeg", mid + ahead * 30.0 + side * 25.0, mid + ahead * 2.0 + side * 1.6);
    fire("flach", mid + ahead * 40.0 + side * (1.75 + 3.5), mid + ahead * 1.0 + side * 1.75);
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "tankkill")) {
    // Three shells at it, then into a wall, on a second one: nothing may blow up numerically, and it has to die.
    auto* w = new World();
    auto* s = new Sim();
    CarRig r;
    w->build();
    s->world = w;
    s->craterWorld = w;
    Tank t = buildLeopard(*s, V3{130, 0, 0}, &r);
    s->finalize();
    t.settle(*s, 3.0);
    const auto run = [&](double secs) {
      Controls c;
      for (int i = 0; i < int(secs * 2000); i++) {
        s->forces(kDt);
        t.update(*s, c, kDt);
        s->finish(kDt);
      }
    };
    double top0 = 0;
    for (u32 n : t.turretNodes) top0 = std::max(top0, s->pos[n].z);
    for (int shot = 0; shot < 3; shot++) {
      s->explode(t.cog + t.up * 1.9, 2200.0, 15.0);
      double top = 0;
      for (int i = 0; i < 12; i++) {
        run(0.25);
        for (u32 n : t.turretNodes) top = std::max(top, s->pos[n].z);
      }
      printf("Treffer %d: Schaden %.2f, zerstoert %d, Turm hoechstens %.1f m (war %.1f), Ketten ab %d/%d\n", shot + 1, t.damage, int(t.destroyed), top, top0, int(t.trackOff[0]), int(t.trackOff[1]));
    }
    run(8.0);
    double maxV = 0;
    size_t worst = 0;
    bool bad = false;
    for (size_t i = 0; i < s->pos.size(); i++) {
      if (!(s->pos[i].x == s->pos[i].x) || std::fabs(s->pos[i].x) > 1e5) bad = true;
      if (len(s->vel[i]) > maxV) {
        maxV = len(s->vel[i]);
        worst = i;
      }
    }
    printf("schnellster Knoten %zu: Masse %.1f, z %.2f, Objekt %d\n", worst, s->mass[worst], s->pos[worst].z, int(s->nodeObj[worst]));
    printf("danach: hoechste Knotengeschw. %.2f m/s, kaputte Zahlen %d, wach %zu\n", maxV, int(bad), s->active.size());
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "rocks")) {
    // Boulders split, then vanish; stones of a wall right beside the blast are gone, those further off are not.
    auto* w = new World();
    auto* s = new Sim();
    w->build();
    s->world = w;
    s->craterWorld = w;
    populateWorld(*s, *w);
    s->finalize();
    V3 rock{};
    size_t rocksAlive = 0;
    for (const StaticBox& b : w->boxes())
      if (b.surface == SURF_ROCK && b.h.x > 0) {
        rocksAlive++;
        if (std::max({b.h.x, b.h.y, b.h.z}) > 0.5 && rock.x == 0) rock = b.c;
      }
    const auto alive = [&] {
      size_t n = 0;
      for (const StaticBox& b : w->boxes()) n += b.surface == SURF_ROCK && b.h.x > 0;
      return n;
    };
    printf("Felsen: %zu, Sprengung bei %.0f %.0f\n", rocksAlive, rock.x, rock.y);
    for (int i = 0; i < 4; i++) {
      const u32 gen = w->boxGeneration();
      auto hits = w->breakRocks(rock, 6.0);
      printf(" Durchgang %d: %zu zerbrochen, %zu Felsen da, Generation %u -> %u\n", i + 1, hits.size(), alive(), gen, w->boxGeneration());
    }
    w->clearCraters();
    printf(" nach Zuruecksetzen: %zu\n", alive());
    if (s->blocks.empty()) {
      printf("keine Mauersteine\n");
      return 0;
    }
    const auto mid = [&](const Block& b) {
      V3 m{};
      for (u32 n : b.n) m += s->pos[n];
      return m * 0.125;
    };
    const V3 at = mid(s->blocks[s->blocks.size() / 2]);
    s->explode(at + V3{0.8, 0, 0}, 2200.0, 15.0);
    for (int i = 0; i < 6000; i++) {
      s->forces(kDt);
      s->finish(kDt);
    }
    size_t gone = 0, moved = 0;
    double maxV = 0;
    for (const Block& b : s->blocks) {
      const V3 m = mid(b);
      if (m.z < -100) gone++;
      else if (len(m - s->rest[b.n[0]]) > 0.5) moved++;
    }
    for (const V3& v : s->vel) maxV = std::max(maxV, len(v));
    printf("Mauer: %zu Steine, zerstoert %zu, weggeschleudert %zu, hoechste Knotengeschw. %.1f m/s, Stabilitaets-Eingriffe %llu\n", s->blocks.size(), gone, moved, maxV, (unsigned long long)s->clampedSteps);
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "tankcrash")) {
    // Into the end wall at full speed: how wildly do the track's points jump between frames?
    auto* w = new World();
    auto* s = new Sim();
    CarRig r;
    w->build();
    s->world = w;
    Tank t = buildLeopard(*s, V3{130, 0, 0}, &r);
    s->finalize();
    t.settle(*s, 3.0);
    for (u32 i = 0; i < s->pos.size(); i++) s->vel[i] = V3{55 / 3.6, 0, 0};
    t.trackOmega[0] = t.trackOmega[1] = 55 / 3.6 / Tank::kSprocketR;
    t.gear = 4;
    std::vector<V3> prev[2], now;
    for (int frame = 0; frame < 200; frame++) {
      Controls c;
      c.throttle = 1;
      double maxOm = 0;
      for (int i = 0; i < 33; i++) {
        s->forces(kDt);
        t.update(*s, c, kDt);
        s->finish(kDt);
        maxOm = std::max({maxOm, std::fabs(t.trackOmega[0]), std::fabs(t.trackOmega[1])});
      }
      double jump = 0, len0 = 0;
      for (int sd = 0; sd < 2; sd++) {
        t.trackPath(*s, sd, now);
        double L = 0;
        for (size_t k = 0; k < now.size(); k++) L += len(now[(k + 1) % now.size()] - now[k]);
        len0 = std::max(len0, L);
        if (prev[sd].size() == now.size())
          for (size_t k = 0; k < now.size(); k++) {
            // Against the hull: what the eye sees, since the camera follows it.
            const V3 d = (now[k] - prev[sd][k]) - t.vCog * (33 * kDt);
            jump = std::max(jump, len(d));
          }
        prev[sd] = now;
      }
      if (frame % 10 == 0 || jump > 0.05)
        printf("f%3d v %5.1f km/h  track speed %5.1f m/s  path point jump %6.3f m  length %.2f m  hull x %.1f\n", frame, t.speed * 3.6, maxOm * Tank::kSprocketR, jump, len0, t.cog.x);
    }
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "esp")) {
    printf("== Stability control: open-loop swerve, same input with and without ==\n");
    for (double kmh : {80.0, 100.0, 120.0, 140.0}) {
      for (int on = 1; on >= 0; on--) {
        Bench b({-150, 0, 0}, 0);
        b.world.clearBoxes();
        b.world.flatten();
        b.car.tune.esp = on;
        const double vT = kmh / 3.6;
        b.launchAt(vT, 3);
        b.run(0.8, [&](double, Controls& c) { c.throttle = clampd(vT - b.car.speed, 0, 1); });
        double slipMax = 0, yawOut = 0;
        const double h0 = std::atan2(b.car.fwd.y, b.car.fwd.x);
        b.run(6.0, [&](double t, Controls& c) {
          const double T = 2.4;
          c.throttle = 0;
          c.steer = t < T ? 0.26 * std::sin(2.0 * PI * t / T) : 0.0;
          const V3 v = b.car.vCog;
          if (len(v) > 3.0) slipMax = std::max(slipMax, std::fabs(std::asin(clampd(dot(norm(v), b.car.left), -1, 1))));
        });
        yawOut = (std::atan2(b.car.fwd.y, b.car.fwd.x) - h0) * 57.3;
        printf("  %3.0f km/h ESP %-3s: peak body slip %5.1f deg, final heading %+6.1f deg, end speed %5.1f km/h, y %+.1f m, guard %llu\n", kmh, on ? "AN" : "AUS", slipMax * 57.3, yawOut, b.car.speed * 3.6, b.car.cog.y, (unsigned long long)b.sim.clampedSteps);
      }
    }
    printf("== Traction on low grip, full throttle ==\n");
    for (int on = 1; on >= 0; on--) {
      Bench b({-115, -95, 0}, 0);
      b.car.tune.tcs = on;
      double mxSpin = 0;
      b.run(3.0, [&](double, Controls& c) { c.throttle = 1; for (const Wheel& w : b.car.wheels) if (w.driven) mxSpin = std::max(mxSpin, w.omega * w.R - b.car.speed); });
      printf("  ASR %-3s: speed after 3 s %.1f km/h, worst wheel overspeed %.1f m/s\n", on ? "AN" : "AUS", b.car.speed * 3.6, mxSpin);
    }
    printf("== Braking 100-0 with and without ABS ==\n");
    for (int on = 1; on >= 0; on--) {
      Bench b;
      b.world.clearBoxes();
      b.world.flatten();
      b.car.tune.abs = on;
      b.launchAt(100 / 3.6, 3);
      const V3 p0 = b.car.cog;
      int locked = 0;
      b.run(9.0, [&](double, Controls& c) { c.brake = 1; for (const Wheel& w : b.car.wheels) if (b.car.speed > 3 && std::fabs(w.omega) < 1) locked++; });
      printf("  ABS %-3s: distance %.1f m, wheel-steps locked %d\n", on ? "AN" : "AUS", len(b.car.cog - p0), locked);
    }
    printf("== Braking 100-0, every car ==\n");
    for (int kind = 0; kind < 3; kind++) {
      World world;
      world.build();
      world.clearBoxes();
      world.flatten();
      Sim sim;
      sim.world = &world;
      CarRig rig;
      const V3 at{-300, 0, 0};
      Vehicle car = kind == 0 ? buildXantia(sim, at, 0.0, &rig) : (kind == 1 ? buildOne1(sim, at, &rig) : buildHummer(sim, at, &rig));
      sim.finalize();
      settleVehicle(sim, car, 3.0);
      const Obj o = sim.objs[car.obj];
      for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) sim.vel[i] = car.fwd * (100 / 3.6);
      car.presetRolling(100 / 3.6, 3);
      const V3 p0 = car.cog;
      Controls c;
      c.brake = 1;
      double when = -1;
      for (int i = 0; i < 9 * 2000 && when < 0; i++) {
        sim.forces(kDt);
        car.update(sim, c, kDt);
        sim.finish(kDt);
        if (car.speed < 0.3) when = i * kDt;
      }
      printf("  %-8s %.1f m in %.2f s\n", kind == 0 ? "Xantia" : (kind == 1 ? "One:1" : "Hummer"), len(car.cog - p0), when);
    }
    printf("== Banked bends: tightest stretch of the road, flat and banked ==\n");
    for (double scale : {0.0, 1.0}) {
      gBankScale = scale;
      World probe;
      probe.build();
      const auto& pts = probe.roads()[0].pts;
      double mx = 0;
      size_t at = 0;
      for (size_t i = 0; i < pts.size(); i++)
        if (std::fabs(pts[i].bank) > mx) {
          mx = std::fabs(pts[i].bank);
          at = i;
        }
      if (scale == 0) continue;
      printf("  largest bank %.1f deg at point %zu\n", std::atan(mx) * 57.3, at);
      for (double sc : {0.0, 1.0}) {
        for (double kmh : {100.0, 120.0, 140.0, 160.0}) {
          gBankScale = sc;
          const size_t start = (at + pts.size() - 60) % pts.size();
          World w2;
          w2.build();
          const auto& road = w2.roads()[0].pts;
          V3 sp{road[start].x, road[start].y, road[start].z};
          auto* B = new Bench(sp, 0);
          const auto& rd = B->world.roads()[0].pts;
          // Aim the car along the road, then hold the speed through the bend with a pursuit driver.
          const double yawRoad = std::atan2(rd[(start + 2) % rd.size()].y - rd[start].y, rd[(start + 2) % rd.size()].x - rd[start].x);
          const Obj& o = B->sim.objs[0];
          const V3 c0 = sp;
          for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) {
            const V3 r = B->sim.pos[i] - c0;
            B->sim.pos[i] = c0 + V3{std::cos(yawRoad) * r.x - std::sin(yawRoad) * r.y, std::sin(yawRoad) * r.x + std::cos(yawRoad) * r.y, r.z};
            B->sim.vel[i] = V3{std::cos(yawRoad), std::sin(yawRoad), 0} * (kmh / 3.6);
          }
          B->car.presetRolling(kmh / 3.6, 3);
          double off = 0, latMax = 0;
          const double vT = kmh / 3.6;
          B->run(7.0, [&](double, Controls& c) {
            size_t best = 0;
            double bd = 1e18;
            for (size_t i = 0; i < rd.size(); i += 2) {
              const double dd = std::hypot(rd[i].x - B->car.cog.x, rd[i].y - B->car.cog.y);
              if (dd < bd) {
                bd = dd;
                best = i;
              }
            }
            off = std::max(off, bd);
            latMax = std::max(latMax, std::fabs(B->car.latAcc));
            const RoadPoint& t = rd[(best + size_t((12.0 + B->car.speed * 0.3) / 1.4)) % rd.size()];
            double err = std::atan2(t.y - B->car.cog.y, t.x - B->car.cog.x) - std::atan2(B->car.fwd.y, B->car.fwd.x);
            while (err > PI) err -= 2 * PI;
            while (err < -PI) err += 2 * PI;
            c.steer = clampd(err * 1.8, -0.5, 0.5);
            c.throttle = clampd((vT - B->car.speed) * 0.5, 0, 1);
            c.brake = clampd((B->car.speed - vT) * 0.15, 0, 0.3);
          });
          printf("  bank %s %3.0f km/h: furthest off the centre line %.1f m (road edge 4 m), peak lat %.2f g, end speed %.0f km/h\n", sc > 0 ? "AN " : "AUS", kmh, off, latMax / 9.81, B->car.speed * 3.6);
          delete B;
        }
      }
    }
    gBankScale = 1.0;
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "tree")) {
    // A car into a tree at various speeds. Does the trunk stop it, or does it snap? How does it land?
    for (double kmh : {15.0, 30.0, 50.0, 80.0, 120.0}) {
      World probe;
      probe.build();
      const V3 tr = probe.trees()[3];
      Bench b({tr.x - 6.0, tr.y + 0.2, probe.height(tr.x - 6.0, tr.y + 0.2)}, 0, true);
      u16 treeObj = 0;
      for (size_t i = 0; i < b.sim.objs.size(); i++)
        if (b.sim.objs[i].nodeCount > 0 && len2(b.sim.rest[b.sim.objs[i].nodeStart] - tr) < 1.0) treeObj = u16(i);
      const u32 top = b.sim.objs[treeObj].nodeStart + 25;
      const size_t before = b.sim.objs.size();
      b.launchAt(kmh / 3.6, 2);
      double peakV = kmh / 3.6;
      double minTop = 1e9;
      u32 brokenWood = 0, brokenBranch = 0;
      b.run(9.0, [&](double, Controls& c) { c.brake = 0; minTop = std::min(minTop, b.sim.pos[top].z - tr.z); });
      const Obj& o = b.sim.objs[treeObj];
      double plastBranch = 0;
      for (u32 i = o.beamStart; i < o.beamStart + o.beamCount; i++) {
        const Beam& B = b.sim.beams[i];
        if (B.flags & BF_BROKEN) (B.tag == kTagWood ? brokenWood : brokenBranch)++;
        if (B.tag == kTagBranch) plastBranch += std::fabs(B.L0 - B.Lorig);
      }
      (void)peakV;
      printf("%3.0f km/h: car now %5.1f km/h, plastic %6.1f kJ, new pieces %zu, trunk beams broken %u, branches %u, branch set %.1f m, tree top lowest z %.1f (rest %.1f), guard %llu sane %d\n", kmh, b.car.speed * 3.6, b.sim.plasticEnergy / 1e3, b.sim.objs.size() - before, brokenWood, brokenBranch, plastBranch, minTop, b.sim.rest[top].z - tr.z, (unsigned long long)b.sim.clampedSteps, b.sane());
    }
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "buckle2")) {
    for (double kmh : {25.0}) {
      Bench b({-46, 91.0, 0}, 0.45);
      b.launchAt(kmh / 3.6, 2);
      b.run(1.2, [&](double, Controls&) {});
      b.run(4.0, [&](double, Controls& c) { c.brake = 1; });
      const Obj& o = b.sim.objs[b.car.obj];
      struct R { double d; u32 i; };
      std::vector<R> rs;
      for (u32 i = o.beamStart; i < o.beamStart + o.beamCount; i++) {
        const Beam& B = b.sim.beams[i];
        const double d = B.L0 - B.Lorig;
        if (std::fabs(d) > 5e-4) rs.push_back({d, i});
      }
      std::sort(rs.begin(), rs.end(), [](R a, R c) { return std::fabs(a.d) > std::fabs(c.d); });
      printf("%zu yielded beams\n", rs.size());
      const V3 org = b.sim.rest[o.nodeStart];
      (void)org;
      for (size_t k = 0; k < rs.size() && k < 30; k++) {
        const Beam& B = b.sim.beams[rs[k].i];
        const V3 A = b.sim.rest[B.a], C = b.sim.rest[B.b];
        const double cs = std::cos(-0.45), sn = std::sin(-0.45);
        auto loc = [&](V3 p) { p = p - V3{-46, 91, 0}; return V3{cs * p.x - sn * p.y, sn * p.x + cs * p.y, p.z}; };
        const V3 la = loc(A), lc = loc(C);
        printf("  %+6.1f mm  yC %3.0f kN  L %.3f  (%.2f %.2f %.2f) - (%.2f %.2f %.2f)\n", rs[k].d * 1e3, B.yC / 1e3, B.Lorig, la.x, la.y, la.z, lc.x, lc.y, lc.z);
      }
    }
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "buckle")) {
    testBuckle();
    return 0;
  }
  if (want("wheel")) {
    testWheelDamage();
    testWheelStrike();
  }
  if (want("compliance")) testCompliance();
  if (argc > 1 && !strcmp(argv[1], "winding")) {
    testWinding();
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "split")) {
    testSplit();
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "torn")) {
    testFreeFall();
    testTorn();
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "slow")) {
    testSlow();
    return 0;
  }
  if (want("response")) testResponse();
  if (want("powertrain")) testPowertrain();
  if (want("rough")) testRough();
  if (want("rest")) testPropRest();
  if (want("props")) testProps();
  if (want("perf")) testPerf();
  return 0;
}
