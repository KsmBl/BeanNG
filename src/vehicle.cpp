#include "vehicle.h"
#include <algorithm>

namespace {
// ---- Engine: PSA ES9J4 3.0 V6, ~190 PS / 267 Nm. Full-load curve at the flywheel.
double fullLoadTorque(double rpm) {
  static const double tbl[][2] = {{0, 120},    {800, 175},  {1500, 215}, {2500, 245}, {3500, 261},
                                  {4000, 267}, {5000, 262}, {5500, 245}, {6000, 215}, {6500, 170},
                                  {6900, 60},  {7200, 0}};
  constexpr int n = sizeof(tbl) / sizeof(tbl[0]);
  if (rpm <= 0) return tbl[0][1];
  for (int i = 1; i < n; i++)
    if (rpm < tbl[i][0])
      return lerpd(tbl[i - 1][1], tbl[i][1], (rpm - tbl[i - 1][0]) / (tbl[i][0] - tbl[i - 1][0]));
  return 0;
}
double frictionTorque(double rpm) { return 12.0 + 0.008 * rpm; }

constexpr double kEngineInertia = 0.22;
constexpr double kClutchCapacity = 430.0;
constexpr double kIdleRpm = 800.0, kRevLimit = 6600.0;
constexpr double kGears[] = {3.42, 1.81, 1.28, 0.97, 0.77};  // close to the ML5T box
constexpr double kReverse = 3.58;
constexpr double kFinal = 3.57;
constexpr double kRpmToRad = 2.0 * PI / 60.0;

// ---- Tyre: 205/60 R15 touring tyre, Magic Formula in normalised combined slip.
constexpr double kKappaPeak = 0.11, kAlphaPeak = 0.14;  // peak slip, about 8 degrees
constexpr double kC = 1.55;                             // falls to roughly 85 % when sliding
constexpr double kSigmaX = 0.10, kSigmaY = 0.35;        // relaxation lengths [m]
constexpr double kFz0 = 4000.0;
constexpr double kTrail0 = 0.035;
const double kB = std::tan(PI / (2.0 * kC));  // puts the peak exactly at normalised slip 1

inline double magic(double s) { return std::sin(kC * std::atan(kB * s)); }

constexpr double kAirDensity = 1.2;
constexpr double kFluidLeakSinkRate = 25e-6;  // m^3/s lost per strut after a line has burst
}  // namespace

void Vehicle::note(const char* msg) {
  lastDamage = msg;
  lastDamageAge = 0;
}

// ---------------------------------------------------------------------------------------
// Tyre and wheel spin
// ---------------------------------------------------------------------------------------
void Vehicle::updateWheel(Sim& sim, Wheel& w, double dt) {
  const V3 pA = sim.pos[w.axIn], pB = sim.pos[w.axOut];
  const V3 C = (pA + pB) * 0.5;
  const V3 vC = (sim.vel[w.axIn] + sim.vel[w.axOut]) * 0.5;
  // Axis points to the car's left on both sides so that positive omega means rolling forward.
  V3 a = norm(w.left ? pB - pA : pA - pB);
  w.hub = C;
  w.axis = a;

  const double Rnow = lerpd(w.R, w.R - 0.095, w.deflate);
  const double kRad = lerpd(w.kRadial, w.kRadial * 0.12, w.deflate);

  // Sample the lower half of the carcass in three rows across the tread. Each row reacts to its
  // deepest sample, which approximates the tyre enveloping a kerb or a stone: the force direction
  // follows the obstacle, not just the ground plane.
  V3 down = V3{0, 0, -1} - a * (-a.z);
  if (len2(down) < 0.01) down = norm(cross(a, V3{1, 0, 0}));
  down = norm(down);
  const V3 fore = cross(a, down) * -1.0;  // perpendicular in the wheel plane

  static const double angC[9] = {1.0,   0.951, 0.951, 0.809, 0.809,
                                 0.588, 0.588, 0.0,   0.0};
  static const double angS[9] = {0.0,  0.309, -0.309, 0.588, -0.588,
                                 0.809, -0.809, 1.0,  -1.0};

  V3 forceSum{}, patch{};
  double penSum = 0;
  u8 surf = SURF_ASPHALT;
  double bestPen = 0;
  for (int row = 0; row < 3; row++) {
    const double off = (row - 1) * w.width * 0.38;
    const V3 base = C + a * off;
    double rowPen = 0;
    V3 rowN{}, rowP{};
    u8 rowSurf = SURF_ASPHALT;
    for (int j = 0; j < 9; j++) {
      const V3 p = base + (down * angC[j] + fore * angS[j]) * Rnow;
      Contact c;
      if (!sim.world->contact(p, c)) continue;
      if (c.depth > rowPen) {
        rowPen = c.depth;
        rowN = c.n;
        rowP = p + c.n * c.depth;
        rowSurf = c.surface;
      }
    }
    if (rowPen > 0) {
      forceSum += rowN * (kRad / 3.0 * rowPen);
      patch += rowP * rowPen;
      penSum += rowPen;
      if (rowPen > bestPen) {
        bestPen = rowPen;
        surf = rowSurf;
      }
    }
  }

  w.contact = penSum > 0;
  w.Fz = w.Fx = w.Fy = 0;

  // Barycentric coordinates of a point in the hub tetrahedron (weights may be negative). Applying
  // a force through them preserves net force and moment exactly, and reading a velocity through
  // the same weights makes the pair passive: force and measured slip velocity belong to the same
  // material point, so the tyre cannot pump energy into the hub.
  const V3 tetE1 = pB - pA, tetE2 = sim.pos[w.ring[0]] - pA, tetE3 = sim.pos[w.ring[1]] - pA;
  const V3 tetC23 = cross(tetE2, tetE3);
  const double tetDet = dot(tetE1, tetC23);
  const u32 tetN[4] = {w.axIn, w.axOut, w.ring[0], w.ring[1]};
  auto tetWeights = [&](V3 point, double out[4]) {
    const V3 q = point - pA;
    out[1] = dot(q, tetC23) / tetDet;
    out[2] = dot(q, cross(tetE3, tetE1)) / tetDet;
    out[3] = dot(q, cross(tetE1, tetE2)) / tetDet;
    out[0] = 1.0 - out[1] - out[2] - out[3];
  };

  double Tcontact = 0;
  if (w.contact && std::fabs(tetDet) > 1e-7) {
    patch = patch / penSum;
    const double fl = len(forceSum);
    const V3 n = forceSum / fl;

    double wp[4];
    tetWeights(patch, wp);
    V3 vPatch{};
    for (int i = 0; i < 4; i++) vPatch += sim.vel[tetN[i]] * wp[i];

    double Fz = fl - w.cRadial * dot(vPatch, n);
    if (Fz < 0) Fz = 0;
    for (int i = 0; i < 4; i++) sim.force[tetN[i]] += n * (Fz * wp[i]);

    V3 t = cross(a, n);
    const double tl = len(t);
    if (tl > 0.5 && Fz > 1.0) {
      t = t / tl;
      const V3 l = cross(n, t);
      const double Vx = dot(vC, t);
      const double Vy = dot(vPatch, l);
      const double Re = Rnow - bestPen * 0.33;
      const double Vsx = Vx - w.omega * Re;

      // First-order tread deflection (relaxation). Unlike textbook slip ratios this stays well
      // defined at standstill: the deflected bristles act as a spring, so a parked car holds on
      // a slope and pulling away is smooth rather than numerically chattering.
      const double absV = std::fabs(Vx);
      const double rx = absV / kSigmaX, ry = absV / kSigmaY;
      const double ex = std::exp(-rx * dt), ey = std::exp(-ry * dt);
      w.ux = w.ux * ex + Vsx * (rx > 1e-6 ? (1.0 - ex) / rx : dt);
      w.uy = w.uy * ey + Vy * (ry > 1e-6 ? (1.0 - ey) / ry : dt);
      constexpr double kMaxSlip = 1.6;
      w.ux = clampd(w.ux, -kMaxSlip * kSigmaX, kMaxSlip * kSigmaX);
      w.uy = clampd(w.uy, -kMaxSlip * kSigmaY, kMaxSlip * kSigmaY);

      const double kappa = -w.ux / kSigmaX;  // SAE slip ratio
      const double alpha = w.uy / kSigmaY;   // tan(slip angle)
      const double sx = kappa / kKappaPeak;
      const double sy = alpha / kAlphaPeak;
      const double s = std::sqrt(sx * sx + sy * sy);

      const double loadSens = 1.0 - 0.08 * (Fz / kFz0 - 1.0);
      double muS = 1.02 * clampd(loadSens, 0.6, 1.25) * surfaceFriction(surf);
      if (w.deflate > 0.5) muS *= 0.45;
      const double Ftot = muS * Fz * magic(s);
      double Fx = 0, Fy = 0;
      if (s > 1e-6) {
        Fx = Ftot * sx / s;
        Fy = -Ftot * sy / s;
      }

      // Camber thrust.
      Fy += -0.8 * Fz * dot(a, n);

      // Viscous bristle damping, which only matters near standstill. It acts on the axle, not at
      // the patch: through the lever of the hub a patch force reaches the hub nodes amplified, and
      // a damper applied there would exceed the explicit stability limit and ring at 1 kHz.
      const double lowV = clampd(1.0 - std::hypot(Vx, Vy) / 2.5, 0, 1);
      const double cLow = 2600.0 * clampd(Fz / kFz0, 0, 1.5) * lowV;
      const double VyAxle = dot(vC, l);
      Fx += -cLow * Vsx;
      const V3 Fdamp = l * (-cLow * VyAxle * 0.5);
      sim.force[w.axIn] += Fdamp;
      sim.force[w.axOut] += Fdamp;

      w.Fx = Fx;
      w.Fy = Fy;
      w.slipX = kappa;
      w.slipY = alpha;
      w.surface = surf;

      // The longitudinal force reaches the hub through the wheel bearing, at the axle: its moment
      // about the axle belongs to the wheel's spin alone (Tcontact). Giving it to the hub as well
      // would count it twice.
      sim.force[w.axIn] += t * (Fx * 0.5);
      sim.force[w.axOut] += t * (Fx * 0.5);
      Tcontact = -Fx * Re;

      // Lateral force: acts behind the patch centre by the pneumatic trail, which produces the
      // self-aligning torque that loads the tie rods.
      const double trail = kTrail0 * clampd(1.0 - s, 0, 1);
      double wl[4];
      tetWeights(patch - t * (trail * (Vx >= 0 ? 1.0 : -1.0)), wl);
      for (int i = 0; i < 4; i++) sim.force[tetN[i]] += l * (Fy * wl[i]);

      if (s > 1.3 && absV + std::fabs(w.omega * Re) > 4.0 && Fz > 800.0 &&
          (sim.stepCount & 31) == 0 && sim.events.size() < 500) {
        const bool soft = surf == SURF_GRASS || surf == SURF_GRAVEL;
        sim.events.push_back({patch, vC, float(s), soft ? EV_DUST : EV_TIRE_SMOKE});
      }
    }
    w.Fz = Fz;
  } else {
    w.ux *= 0.995;
    w.uy *= 0.995;
  }

  // Spin integration; the brake is treated as an exact stick-slip limit so that it holds at zero
  // without oscillating, however large the torque.
  const double Trr = (w.contact ? 0.011 * w.Fz * Rnow : 0.0) + 1.0;
  double omega = w.omega + (w.driveT * (w.attached ? 1.0 : 0.0) + Tcontact) * dt / w.I;
  const double rrDv = Trr * dt / w.I;
  omega = std::fabs(omega) <= rrDv ? 0.0 : omega - std::copysign(rrDv, omega);

  double Tb = 0;
  {
    double pedal = brakeAct * w.brakeMax;
    if (!w.front) {
      // Citroen brake bias: rear pressure follows rear suspension pressure, i.e. rear load.
      const double loadRatio = clampd(axle[1].s[w.left ? 0 : 1].pGas / 100e5, 0.3, 1.3);
      pedal *= loadRatio;
    }
    // Brakes are powered by the same HP circuit: lost accumulator pressure weakens them.
    pedal *= clampd(accumulator / 80e5, 0.15, 1.0);
    Tb = pedal;

    // ABS, active above walking pace.
    const double Vwheel = std::fabs(speed);
    if (Vwheel > 2.0 && brakeAct > 0.02) {
      // Slip controller around the tyre's peak (about -0.11). Proportional in both directions, so
      // it hovers near maximum grip rather than cycling deep into and out of lock-up.
      const double kap = w.contact ? (omega * Rnow - speed) / std::max(Vwheel, 1.0) : 0.0;
      const double rate = clampd((kap + 0.10) * 45.0, -16.0, 5.0);
      w.absFrac = clampd(w.absFrac + rate * dt, 0.02, 1.0);
    } else {
      w.absFrac = 1.0;
    }
    Tb *= w.absFrac;
    if (!w.front) Tb += hbAct_ * 2200.0;
  }
  const double bDv = Tb * dt / w.I;
  const double omegaBeforeBrake = omega;
  omega = std::fabs(omega) <= bDv ? 0.0 : omega - std::copysign(bDv, omega);

  // The caliper is bolted to the hub, so the brake torque actually transmitted reacts there. This
  // is the one moment about the axle the hub feels; it is what makes the suspension dive.
  const double Tcaliper = (omegaBeforeBrake - omega) * w.I / dt;
  if (std::fabs(Tcaliper) > 0.1) {
    double arm2 = 0;
    for (int i = 0; i < 6; i++) arm2 += len2(sim.pos[w.ring[i]] - C);
    for (int i = 0; i < 6; i++) {
      const V3 r = sim.pos[w.ring[i]] - C;
      sim.force[w.ring[i]] += cross(a, r) * (Tcaliper / arm2);
    }
  }

  w.omega = omega;
  w.theta = std::fmod(w.theta + omega * dt, 2.0 * PI);

  // Pinch flat: tyre compressed down onto the rim by a heavy impact.
  if (w.deflate == 0 && w.Fz > 32000.0) {
    w.deflate = 0.001;
    note("REIFEN GEPLATZT");
  }
  if (w.deflate > 0 && w.deflate < 1) w.deflate = std::min(1.0, w.deflate + dt * 2.5);
}

// ---------------------------------------------------------------------------------------
// Hydropneumatics
// ---------------------------------------------------------------------------------------
void Vehicle::updateHydraulics(Sim& sim, double dt) {
  // ---- Hydractive computer: soft unless the driver or the road demands firm.
  const double steerRate = std::fabs(steerAct - prevSteer_) / dt;
  const double throttleRate = std::fabs(throttleAct - prevThrottle_) / dt;
  prevSteer_ = steerAct;
  prevThrottle_ = throttleAct;
  const double v = std::fabs(speed);
  bool wantFirm = sportMode;
  if (v > 5.0 && steerRate * clampd(v / 20.0, 0.2, 2.0) > 0.45) wantFirm = true;
  if (std::fabs(latAcc) > 2.2) wantFirm = true;
  if (brakeAct > 0.35 || std::fabs(lonAcc) > 3.8) wantFirm = true;
  if (throttleRate > 2.2 && v > 3.0) wantFirm = true;
  if (v > 44.0) wantFirm = true;  // about 160 km/h
  if (wantFirm)
    firmHold = 1.4;
  else
    firmHold = std::max(0.0, firmHold - dt);
  hydractiveSoft = firmHold <= 0;

  // ---- Pump and accumulator.
  const double pumpOn = engineRunning && !dmg.engineMount ? 1.0 : 0.0;
  const double pumpFlowOK = pumpOn * clampd(rpm() / 800.0, 0.3, 1.0);
  if (accumulator < 170e5 && hydroFluid > 0.02)
    accumulator = std::min(170e5, accumulator + pumpFlowOK * 22e5 * dt);

  static const double kNominalL[3] = {-0.05, 0.0, 0.06};  // ride height offset by mode
  const double heightOffset = kNominalL[heightMode];

  // ---- Activa: the anti-roll bars are held by hydraulic rams. One ram command for both axles,
  // so the front/rear share of the roll moment stays fixed and the car keeps its balance.
  const double L0 = axle[0].s[0].L, R0 = axle[0].s[1].L;
  const double L1 = axle[1].s[0].L, R1 = axle[1].s[1].L;
  const double dF = L0 - R0, dR = L1 - R1;
  const double rollErr = 0.5 * (dF + dR);
  const bool activaLive = activaOn && accumulator > 90e5 && !dmg.hydraulics;
  if (activaLive) {
    // Feed-forward from lateral acceleration provides nearly all of the correction, because it
    // cannot oscillate with the body. The slow integral part trims what remains. A fast integral
    // alone would add 90 degrees of phase lag right at the roll resonance and shake the car.
    constexpr double kRollArm = 0.40;  // CoG height above the roll axis [m]
    const double barMoment = (axle[0].kBar * 1.0 + axle[1].kBar * 1.25) * 1.49;
    // Lateral acceleration of the car's path: speed times yaw rate. Measuring acceleration at the
    // centre of gravity instead would also see the body's own sway and feed the roll back into
    // the controller.
    const double ff = -totalMass * kRollArm * speed * yawRate_ / barMoment;
    const double dead = 0.002;
    const double e = std::fabs(rollErr) < dead ? 0.0 : rollErr - std::copysign(dead, rollErr);
    ramTrim_ = clampd(ramTrim_ - 0.9 * e * dt, -0.05, 0.05);
    const double target = clampd(ff + ramTrim_, -0.14, 0.14);
    const double rate = clampd((target - ramCmd_) * 25.0, -0.25, 0.25);
    ramCmd_ += rate * dt;
    accumulator -= std::fabs(rate) * 2.5e5 * dt;
  } else {
    // Rams bleed down: the bars stay passive and the car rolls like a normal one.
    ramCmd_ += (0.0 - ramCmd_) * clampd(dt * 3.0, 0, 1);
    ramTrim_ = 0;
  }
  axle[0].ram = ramCmd_;
  axle[1].ram = ramCmd_ * 1.25;  // with the bar rates this splits the roll moment ~57/43

  for (int ax = 0; ax < 2; ax++) {
    HydroAxle& A = axle[ax];
    double avgL = 0;
    int live = 0;
    for (int side = 0; side < 2; side++) {
      HydroStrut& s = A.s[side];
      const V3 d = sim.pos[s.low] - sim.pos[s.top];
      s.L = len(d);
      const V3 n = d / std::max(s.L, 1e-6);
      s.vel = dot(sim.vel[s.low] - sim.vel[s.top], n);
      if (!s.burst) {
        avgL += s.L;
        live++;
      }
      // Fluid displaced by the rod. Extension draws fluid out of the sphere.
      s.Vf -= s.A * s.vel * dt;
      if (s.burst) s.Vf -= kFluidLeakSinkRate * dt;

      double Vgas = s.V0 - s.Vf;
      Vgas = clampd(Vgas, s.V0 * 0.08, s.V0);
      double p = s.p0 * std::pow(s.V0 / Vgas, 1.35);
      if (s.Vf < 0) p *= std::max(0.0, 1.0 + s.Vf / (0.1 * s.V0));  // sphere empty, line goes slack
      if (s.burst) p *= 0.0;
      s.pGas = p;

      // Damper valve, degressive above the knee velocity.
      const int mode = hydractiveSoft ? 1 : 0;
      const double c = s.vel > 0 ? s.reb[mode] : s.bump[mode];
      constexpr double kKnee = 0.30;
      const double av = std::fabs(s.vel);
      const double fd = av < kKnee ? c * av : c * (kKnee + 0.45 * (av - kKnee));
      double F = p * s.A - std::copysign(fd, s.vel);

      // Rubber bump stop and rebound stop.
      if (s.L < s.Lmin) F += 250e3 * (s.Lmin - s.L) * (1.0 + 18.0 * (s.Lmin - s.L));
      if (s.L > s.Lmax) F -= 350e3 * (s.L - s.Lmax);

      // Anti-roll bar + ram, acting at the wheel stations.
      const double delta = (side == 0 ? 1.0 : -1.0) * (A.s[0].L - A.s[1].L - A.ram);
      F -= A.kBar * delta;

      s.F = F;
      sim.force[s.low] += n * F;
      sim.force[s.top] -= n * F;
    }

    // Regulator sphere: open in soft mode. It joins both struts of the axle, which softens heave
    // but not roll. That is the point of Hydractive.
    if (hydractiveSoft && !A.s[0].burst && !A.s[1].burst) {
      double VgasR = clampd(A.regV0 - A.regVf, A.regV0 * 0.08, A.regV0);
      const double pr = A.regP0 * std::pow(A.regV0 / VgasR, 1.35);
      constexpr double kR = 2.2e9;  // regulator damper [Pa s/m^3]
      for (int side = 0; side < 2; side++) {
        const double Q = clampd((A.s[side].pGas - pr) / kR, -1.2e-3, 1.2e-3);
        A.s[side].Vf -= Q * dt;
        A.regVf += Q * dt;
      }
    }

    // Height corrector: slow, deliberately filtered so it ignores bumps and only sees real load
    // changes or a change of ride-height mode.
    if (live > 0) {
      avgL /= live;
      const double err = (A.s[0].Lnom + heightOffset) - avgL;
      A.levelErrLP += (err - A.levelErrLP) * clampd(dt * 2.0, 0, 1);
      if (std::fabs(A.levelErrLP) > 0.003) {
        const bool canAdd = accumulator > 90e5 && hydroFluid > 0.02;
        // Proportional, so that it converges instead of hunting around the target height.
        constexpr double kLevelGain = 0.45;     // 1/s
        constexpr double kLevelMaxFlow = 18e-6;  // m^3/s per strut
        double flow = clampd(A.s[0].A * kLevelGain * A.levelErrLP, -kLevelMaxFlow, kLevelMaxFlow);
        if (flow > 0 && !canAdd) flow = 0;
        for (int side = 0; side < 2; side++) {
          if (A.s[side].burst) continue;
          A.s[side].Vf += flow * dt;
        }
        if (flow > 0) accumulator -= flow * 2.2e11 * dt;
      }
    }
  }

  if (dmg.hydraulics) {
    hydroFluid = std::max(0.0, hydroFluid - dt * 0.08);
    accumulator = std::max(0.0, accumulator - dt * 18e5);
  }
}

void Vehicle::settleFluid(Sim& sim) {
  // Preset the sphere volumes for the static load so the first frame already sits near height.
  for (int ax = 0; ax < 2; ax++) {
    const double corner = (ax == 0 ? 0.58 : 0.42) * totalMass * 9.81 * 0.5 - 380.0;
    for (int side = 0; side < 2; side++) {
      HydroStrut& s = axle[ax].s[side];
      const double p = corner / s.A;
      const double Vg = s.V0 * std::pow(s.p0 / p, 1.0 / 1.35);
      s.Vf = s.V0 - Vg;
      s.pGas = p;
    }
    const double p = corner / axle[ax].s[0].A;
    axle[ax].regVf = axle[ax].regV0 - axle[ax].regV0 * std::pow(axle[ax].regP0 / p, 1.0 / 1.35);
  }
}

// ---------------------------------------------------------------------------------------
// Engine, clutch, gearbox, differential
// ---------------------------------------------------------------------------------------
void Vehicle::updateDrivetrain(Sim& sim, const Controls& in, double dt) {
  // Mode buttons are edge-triggered upstream.
  if (in.toggleAuto) autoBox = !autoBox;
  if (in.toggleSport) sportMode = !sportMode;
  if (in.toggleActiva) activaOn = !activaOn;
  if (in.cycleHeight) heightMode = (heightMode + 1) % 3;
  if (in.toggleIgnition) {
    engineRunning = !engineRunning;
    if (engineRunning) engineOmega = std::max(engineOmega, kIdleRpm * kRpmToRad);
  }

  int nDriven = 0;
  double omegaDiff = 0;
  for (const Wheel& w : wheels)
    if (w.driven && w.attached) {
      omegaDiff += w.omega;
      nDriven++;
    }
  if (nDriven) omegaDiff /= nDriven;

  const double dir = gear < 0 ? -1.0 : 1.0;
  const double ratio = gear == 0 ? 0.0 : (gear > 0 ? kGears[gear - 1] : -kReverse) * kFinal;
  const double omegaIn = ratio * omegaDiff;

  // ---- Shifting.
  sinceShift += dt;
  auto engage = [&](int g) {
    if (g == gear) return;
    gear = g;
    shiftTimer = 0.22;
    sinceShift = 0;
  };
  if (shiftTimer > 0) shiftTimer -= dt;
  if (in.requestGear != 99) engage(in.requestGear);
  if (in.shiftUp) engage(std::min(5, gear + 1));
  if (in.shiftDown) engage(std::max(-1, gear - 1));
  if (autoBox && shiftTimer <= 0 && sinceShift > 0.9 && gear >= 1) {
    const double r = omegaIn / kRpmToRad;
    const double up = lerpd(2300, 6150, throttleAct);
    const double dn = lerpd(1250, 3400, throttleAct);
    if (r > up && gear < 5)
      engage(gear + 1);
    else if (r < dn && gear > 1)
      engage(gear - 1);
  }

  // ---- Throttle/idle/rev limiter.
  const double rpmNow = rpm();
  double th = throttleAct;
  if (engineRunning) {
    if (rpmNow < kIdleRpm + 100) th = std::max(th, clampd((kIdleRpm + 100 - rpmNow) / 250.0, 0, 0.55));
    if (rpmNow > kRevLimit) th = 0;
    if (dmg.seized) th = 0;
    if (dmg.coolantTemp > 118.0) th *= clampd(1.0 - (dmg.coolantTemp - 118.0) / 15.0, 0.2, 1.0);
  } else {
    th = 0;
    if (throttleAct > 0.15 && !dmg.seized) {  // restart by pressing the pedal
      engineRunning = true;
      engineOmega = std::max(engineOmega, 650 * kRpmToRad);
    }
  }
  double Te = th * fullLoadTorque(rpmNow) - (1.0 - th) * frictionTorque(rpmNow);
  if (dmg.seized) Te -= 900.0;
  engineTorqueOut = Te;

  // ---- Clutch, automated. It slips at launch and opens during shifts.
  if (shiftTimer > 0 || gear == 0) {
    clutch = 0;
  } else {
    const double rIn = std::fabs(omegaIn) / kRpmToRad;
    if (rIn > 1150.0) {
      clutch = 1;
    } else {
      const double e = smoothstepd(1000.0, 2100.0, rpmNow);
      clutch = throttleAct > 0.03 ? e : 0.0;
    }
  }

  // Velocity-level clutch solve: the torque needed to equalise both sides this step, limited by
  // clutch capacity. A stiff torsional spring here would need a far smaller time step.
  double Tc = 0;
  double wheelI = 0;
  for (const Wheel& w : wheels)
    if (w.driven) wheelI = w.I;
  if (ratio != 0 && nDriven > 0 && clutch > 0) {
    const double J = 1.0 / kEngineInertia + ratio * ratio / (nDriven * wheelI);
    const double need = ((engineOmega + Te * dt / kEngineInertia) - omegaIn) / (dt * J);
    const double cap = kClutchCapacity * clutch;
    Tc = clampd(need, -cap, cap);
  }
  engineOmega += (Te - Tc) * dt / kEngineInertia;
  if (engineOmega < 0) engineOmega = 0;
  if (engineRunning && engineOmega < 380.0 * kRpmToRad) {
    engineRunning = false;
    note("MOTOR ABGEWUERGT");
  }

  // Open differential: equal torque to both driven wheels.
  double totalT = 0;
  for (Wheel& w : wheels) {
    w.driveT = 0;
    if (w.driven && w.attached && nDriven > 0) {
      w.driveT = Tc * ratio / nDriven;
      totalT += w.driveT;
    }
  }

  // Reaction on the engine and gearbox block: launching rocks the engine on its mounts.
  if (!engineNodes.empty() && std::fabs(totalT) > 0.1) {
    V3 c{};
    for (u32 n : engineNodes) c += sim.pos[n];
    c = c / double(engineNodes.size());
    double arm2 = 0;
    for (u32 n : engineNodes) {
      const V3 r = sim.pos[n] - c;
      arm2 += len2(r - left * dot(r, left));
    }
    for (u32 n : engineNodes) sim.force[n] += cross(left, sim.pos[n] - c) * (-totalT / arm2);
  }
  (void)dir;
}

// ---------------------------------------------------------------------------------------
// Body frame, telemetry, damage monitors
// ---------------------------------------------------------------------------------------
void Vehicle::updateBodyFrame(Sim& sim, double dt) {
  const V3 fL = sim.pos[refFrontL], fR = sim.pos[refFrontR], rL = sim.pos[refRearL],
           rR = sim.pos[refRearR];
  const V3 f = norm((fL + fR) - (rL + rR));
  V3 l = (fL + rL) - (fR + rR);
  l = norm(l - f * dot(l, f));
  fwd = f;
  left = l;
  up = cross(f, l);

  const Obj& o = sim.objs[obj];
  V3 c{}, vm{};
  double m = 0;
  for (u32 i = o.nodeStart, e = i + o.nodeCount; i < e; i++) {
    c += sim.pos[i] * sim.mass[i];
    vm += sim.vel[i] * sim.mass[i];
    m += sim.mass[i];
  }
  cog = c / m;
  vCog = vm / m;
  speed = dot(vCog, fwd);

  const double period = dt;
  const V3 acc = (vCog - prevV_) / period;
  prevV_ = vCog;
  const double alpha = clampd(period * 12.0, 0, 1);
  latAcc += (dot(acc, left) - latAcc) * alpha;
  lonAcc += (dot(acc, fwd) - lonAcc) * alpha;
  const double aNow = len(acc) / 9.81;
  peakG = std::max(aNow > 200 ? peakG : aNow, peakG * (1.0 - period * 0.5));

  const double yawNow = dot(cross(prevFwd_, fwd), up) / period;
  prevFwd_ = fwd;
  yawRate_ += (clampd(yawNow, -5.0, 5.0) - yawRate_) * clampd(period * 15.0, 0, 1);

  rollAngle = std::atan2(left.z, up.z);
  pitchAngle = std::atan2(fwd.z, up.z);

  const double dLf = axle[0].s[0].L - axle[0].s[1].L;
  const double dLr = axle[1].s[0].L - axle[1].s[1].L;
  suspRoll = 0.5 * (dLf + dLr) / 1.49;
  rideHeight = cog.z;
}

void Vehicle::updateDamage(Sim& sim, double dt) {
  auto plastified = [&](const std::vector<u32>& list, double strain) {
    int n = 0;
    for (u32 id : list) {
      const Beam& b = sim.beams[id];
      if ((b.flags & BF_BROKEN) || std::fabs(b.L0 - b.Lorig) / b.Lorig > strain) n++;
    }
    return n;
  };

  if (!dmg.radiator && plastified(radiatorBeams, 0.05) >= 2) {
    dmg.radiator = true;
    note("KUEHLER BESCHAEDIGT");
  }
  if (!dmg.engineMount) {
    if (plastified(engineMountBeams, 0.20) * 2 > int(engineMountBeams.size())) {
      dmg.engineMount = true;
      note("MOTORLAGER GERISSEN");
    }
  }
  // The engine is rammed into the firewall when the crumple zone is used up: ancillaries, sump and
  // block do not survive that. Measured against the as-built distance.
  if (!dmg.seized && !engineNodes.empty()) {
    V3 c{}, c0{};
    for (u32 n : engineNodes) {
      c += sim.pos[n];
      c0 += sim.rest[n];
    }
    c = c / double(engineNodes.size());
    c0 = c0 / double(engineNodes.size());
    const V3 fw = (sim.pos[refFrontL] + sim.pos[refFrontR]) * 0.5;
    const V3 fw0 = (sim.rest[refFrontL] + sim.rest[refFrontR]) * 0.5;
    if (len(fw0 - c0) - len(fw - c) > 0.16 || plastified(engineBlockBeams, 0.03) >= 3) {
      dmg.seized = true;
      engineRunning = false;
      note("MOTOR ZERSTOERT");
    }
  }
  if (!dmg.hydraulics && plastified(hydraulicBeams, 0.06) >= 2) {
    dmg.hydraulics = true;
    note("STOP: HYDRAULIK LECK");
  }
  for (int i = 0; i < 2; i++) {
    const Beam& b = sim.beams[tieBeam[i]];
    if ((b.flags & BF_BROKEN) && !dmg.steering) {
      dmg.steering = true;
      note("SPURSTANGE GEBROCHEN");
    }
  }

  // A strut whose mount or wheel is gone loses its fluid. Wheels whose hub has torn too far from
  // the gearbox lose their drive shaft.
  for (int ax = 0; ax < 2; ax++)
    for (int side = 0; side < 2; side++) {
      HydroStrut& s = axle[ax].s[side];
      if (!s.burst && s.L > s.Lmax + 0.22) {
        s.burst = true;
        dmg.hydraulics = true;
        note("STOP: FEDERBEIN ZERSTOERT");
      }
    }
  for (Wheel& w : wheels) {
    if (w.driven && w.attached && !engineNodes.empty()) {
      if (len(sim.pos[w.axIn] - sim.pos[engineNodes[0]]) > 1.25) {
        w.attached = false;
        note("ANTRIEBSWELLE AB");
      }
    }
  }

  // Cooling.
  if (dmg.radiator) dmg.coolant = std::max(0.0, dmg.coolant - dt * 0.04);
  const double heat = engineRunning ? (0.8 + 2.2 * throttleAct * rpm() / 6000.0) : -1.0;
  const double cooling = (dmg.coolant > 0.05 ? (dmg.coolantTemp - 88.0) * 0.45 * dmg.coolant : 0.0);
  dmg.coolantTemp += (heat - cooling) * dt * (dmg.coolant > 0.05 ? 1.0 : 1.5);
  dmg.coolantTemp = std::max(dmg.coolantTemp, 20.0);
  if (dmg.coolantTemp > 135.0 && !dmg.seized) {
    dmg.seized = true;
    note("MOTOR UEBERHITZT - FESTGEGANGEN");
  }
  lastDamageAge += dt;
}

void Vehicle::update(Sim& sim, const Controls& in, double dt) {
  bodyTick_++;
  updateBodyFrame(sim, dt);

  // Rack and pinion: rate-limited, and the rack is moved by changing the tie rod rest lengths.
  const double maxRate = 2.6 * dt;
  steerAct += clampd(in.steer - steerAct, -maxRate, maxRate);
  if (!dmg.steering) {
    // Incremental, so that a tie rod bent in a crash stays bent and the car pulls to one side.
    const double delta = steerAct * steerTravel;
    Beam& tl = sim.beams[tieBeam[0]];
    Beam& tr = sim.beams[tieBeam[1]];
    if (!(tl.flags & BF_BROKEN)) tl.L0 -= delta - rackPrev_;
    if (!(tr.flags & BF_BROKEN)) tr.L0 += delta - rackPrev_;
    rackPrev_ = delta;
  }
  throttleAct += clampd(in.throttle - throttleAct, -8.0 * dt, 6.0 * dt);
  brakeAct += clampd(in.brake - brakeAct, -10.0 * dt, 9.0 * dt);
  hbAct_ = in.handbrake;

  // Aerodynamics: applied to the whole body by mass, so it needs no attitude model.
  const double v2 = len2(vCog);
  if (v2 > 0.25) {
    const V3 Fa = vCog * (-0.5 * kAirDensity * dragArea * std::sqrt(v2));
    const double perKg = 1.0 / totalMass;
    const Obj& o = sim.objs[obj];
    for (u32 i = o.nodeStart, e = i + o.nodeCount; i < e; i++)
      sim.force[i] += Fa * (sim.mass[i] * perKg);
  }

  updateDrivetrain(sim, in, dt);
  for (Wheel& w : wheels) updateWheel(sim, w, dt);
  updateHydraulics(sim, dt);
  if (bodyTick_ % 20 == 0) updateDamage(sim, dt * 20.0);
}

void settleVehicle(Sim& sim, Vehicle& v, double seconds) {
  Controls c;
  constexpr double dt = 1.0 / 2000.0;
  v.settleFluid(sim);
  const int steps = int(seconds / dt);
  for (int i = 0; i < steps; i++) {
    sim.forces(dt);
    v.update(sim, c, dt);
    sim.finish(dt);
  }
  sim.events.clear();
  v.peakG = 0;
  v.lastDamage.clear();
  v.lastDamageAge = 100;
}
