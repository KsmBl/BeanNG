// Headless test bench: drives the real simulation through scripted manoeuvres and prints the
// numbers, so the physics can be compared with the real car rather than judged by eye.
#include <chrono>
#include <cstdio>
#include <cstring>
#include <array>
#include <functional>
#include <vector>
#include "props.h"
#include "vehicle.h"

namespace {
constexpr double kDt = 1.0 / 2000.0;

struct Bench {
  World world;
  Sim sim;
  Vehicle car;
  CarSkin skin;

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
    for (Wheel& w : car.wheels) w.omega = v / (w.R - 0.006);
    car.gear = gear;
    car.engineOmega = std::max(90.0, v / 0.306 * 3.57 * (gear > 0 ? std::array<double, 5>{3.42, 1.81, 1.28, 0.97, 0.77}[gear - 1] : 3.0));
  }
};

void testStatics() {
  Bench b;
  const Vehicle& c = b.car;
  printf("== Statics ==\n");
  printf("  nodes %zu  beams %zu  tris %zu  skin quads %zu\n", b.sim.pos.size(), b.sim.beams.size(),
         b.sim.tris.size(), b.skin.quads.size());
  u32 worst = 0;
  const double lam = b.sim.maxAccelEstimate(&worst);
  printf("  Gershgorin lambda %.3g (limit %.3g), worst node %u mass %.2f at (%.2f %.2f %.2f)\n", lam,
         4.0 / (kDt * kDt), worst, b.sim.mass[worst], b.sim.pos[worst].x, b.sim.pos[worst].y,
         b.sim.pos[worst].z);
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
  CarSkin skin;
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

void testDiag() {
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
    if (t > 8) c.steer = 0.012;
    if ((int(t * 2000) % 20000) == 0)
      printf("  t=%4.0f v=%5.1f km/h gear %d rpm %5.0f  z=%.2f\n", t, kmh, b.car.gear, b.car.rpm(), b.car.cog.z);
  });
  ok = b.sane();
  printf("  0-60: %.2f s  0-100: %.2f s  400 m: %.2f s  vmax: %.1f km/h at t=%.0f  sane %d\n", t60, t100, t400, vTop, tTop, ok);
  printf("  (real car: 0-100 about 8 s, vmax about 230 km/h)\n");
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
    printf("  %-22s v=%5.1f km/h body z %.2f..%.2f  plastic %6.1f kJ  broken %3d  flat %d  sane %d\n", name, b.car.speed * 3.6, zMin, zMax, b.sim.plasticEnergy / 1e3, broken, flat, b.sane());
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
    drive(b, 8.0, 40, a, c, 70);
    report("kerbs 40 km/h", b, a, c);
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

void testPropRest() {
  // Regression: every prop, woken with nothing touching it, must simply stay standing.
  Bench b({-90, 0, 0}, 0, true);
  printf("== Props at rest, all awake ==\n");
  u32 worst = 0;
  const double lam = b.sim.maxAccelEstimate(&worst);
  printf("  stability bound %.3g of %.3g, worst node mass %.2f\n", lam, 4.0 / (kDt * kDt), b.sim.mass[worst]);
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
  if (want("rough")) testRough();
  if (want("rest")) testPropRest();
  if (want("props")) testProps();
  if (want("perf")) testPerf();
  return 0;
}
