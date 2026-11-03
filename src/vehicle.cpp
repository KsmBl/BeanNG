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

// Rotating inertias [kg m^2] and shaft rates [Nm/rad]. Plausible for this size of car, not OEM data.
constexpr double kDiscInertia = 0.012;    // clutch disc
constexpr double kInputInertia = 0.016;   // input shaft, cluster and synchro hubs
constexpr double kSpiderInertia = 0.025;  // side gears, referred to the shafts
constexpr double kDamperRate = 1500.0, kDamperDamp = 3.0;    // clutch torsion damper
constexpr double kRpmToRad = 2.0 * PI / 60.0;

// ---- Tyre: 205/60 R15 touring tyre, Magic Formula in normalised combined slip.
constexpr double kKappaPeak = 0.11, kAlphaPeak = 0.14;  // peak slip, about 8 degrees
constexpr double kC = 1.40;                             // falls to roughly 85 % when sliding
constexpr double kSigmaX = 0.10, kSigmaY = 0.35;        // relaxation lengths [m]
constexpr double kFz0 = 4000.0;
constexpr double kTrail0 = 0.035;
const double kB = std::tan(PI / (2.0 * kC));  // puts the peak exactly at normalised slip 1

inline double magic(double s) { return std::sin(kC * std::atan(kB * s)); }

constexpr double kAirDensity = 1.2;
constexpr double kFluidLeakSinkRate = 25e-6;  // m^3/s lost per strut after a line has burst

// Main hydraulic circuit. Accumulator data from owner forums, pump size estimated.
constexpr double kAccV0 = 400e-6, kAccP0 = 65e5;
constexpr double kCutIn = 145e5, kCutOut = 170e5;  // pressure regulator
constexpr double kPumpDisplacement = 3.0e-6;       // [m^3 per crank revolution]
constexpr double kRamArea = 6e-4;                  // Activa ram piston [m^2]
constexpr double kRackArea = 8e-4;                 // power steering piston [m^2]
constexpr double kBrakeVolume = 14e-6;             // fluid for a full brake application [m^3]
}  // namespace

namespace {
double rimDentAt(const Wheel& w, double phi) {
  constexpr int n = Wheel::kDentBins;
  double f = (phi / (2.0 * PI) + 1000.0) * n;
  f -= std::floor(f / n) * n;
  const int i = int(f) % n;
  const double t = f - std::floor(f);
  return lerpd(w.dent[i], w.dent[(i + 1) % n], t);
}
}  // namespace

// Adds a dent at a point on the wheel, spread over the rim like the buckle a kerb leaves behind.
void Vehicle::addRimDamage(Wheel& w, V3 where, double depth, double lateral) {
  const V3 yAxis = w.axis * (w.left ? 1.0 : -1.0);
  const V3 upV = (w.hub + w.upHint) - w.hub;
  const V3 xAxis = norm(upV - yAxis * dot(upV, yAxis));
  const V3 zAxis = cross(xAxis, yAxis);
  const V3 r = where - w.hub;
  const double phi = std::atan2(dot(r, zAxis), dot(r, xAxis)) - w.theta * (w.left ? 1.0 : -1.0);
  constexpr int n = Wheel::kDentBins;
  double f = (phi / (2.0 * PI) + 1000.0) * n;
  f -= std::floor(f / n) * n;
  for (int k = -3; k <= 3; k++) {
    const int i = (int(f) + k + 1000 * n) % n;
    const double g = std::exp(-0.5 * k * k / (1.3 * 1.3));
    w.dent[i] = float(std::min(0.055, double(w.dent[i]) + depth * g));
  }
  w.runoutX = float(clampd(w.runoutX + lateral * std::cos(phi), -0.03, 0.03));
  w.runoutZ = float(clampd(w.runoutZ + lateral * std::sin(phi), -0.03, 0.03));
}

// Spokes that have yielded mean the structure of the wheel was hit there.
void Vehicle::updateRimDamage(Sim& sim, Wheel& w) {
  for (int i = 0; i < 6; i++) {
    const Beam& bi = sim.beams[w.spoke[i][0]];
    const Beam& bo = sim.beams[w.spoke[i][1]];
    const double si = std::max(0.0, bi.Lorig - bi.L0), so = std::max(0.0, bo.Lorig - bo.L0);
    const double di = si - w.spokeSeen[i][0], dio = so - w.spokeSeen[i][1];
    if (di < 1e-4 && dio < 1e-4) continue;
    w.spokeSeen[i][0] = si;
    w.spokeSeen[i][1] = so;
    addRimDamage(w, sim.pos[w.ring[i]], 0.5 * (std::max(di, 0.0) + std::max(dio, 0.0)), 0.5 * (dio - di));
  }
}

// A knuckle turns on its ball joints, held by nothing but its tie rod. Rod and arm form a toggle: past the point
// where they line up the wheel has found a second resting place, turned a hundred degrees, with no part any the
// worse for it. Real ones have steering stops, so that it can never get that far.
void Vehicle::applySteerStops(Sim& sim, double) {
  const V3 f0 = norm(sim.rest[refFrontL] + sim.rest[refFrontR] - sim.rest[refRearL] - sim.rest[refRearR]);
  const V3 side0 = sim.rest[refFrontL] + sim.rest[refRearL] - sim.rest[refFrontR] - sim.rest[refRearR];
  const V3 u0 = norm(cross(f0, side0));
  const V3 l0 = cross(u0, f0);
  for (Wheel& w : wheels) {
    if (w.stopLimit <= 0) continue;
    const V3 axisV = sim.pos[w.stopKu] - sim.pos[w.stopKl];
    const V3 armV = sim.pos[w.stopKs] - sim.pos[w.stopKl];
    const V3 arm0 = sim.rest[w.stopKs] - sim.rest[w.stopKl];
    // Torn apart there is nothing left to stop.
    if (len2(axisV) > 1.5 * len2(sim.rest[w.stopKu] - sim.rest[w.stopKl]) || len2(armV) > 1.5 * len2(arm0)) continue;
    const V3 e = norm(axisV);
    const V3 arm = armV - e * dot(armV, e);
    const V3 straight = fwd * dot(arm0, f0) + left * dot(arm0, l0) + up * dot(arm0, u0);
    const V3 ref = straight - e * dot(straight, e);
    const double ang = std::atan2(dot(e, cross(ref, arm)), dot(ref, arm));
    const double over = std::fabs(ang) - w.stopLimit;
    if (over <= 0) continue;
    const double L = std::max(len(arm), 1e-3);
    // Pushes the arm's end back along its arc, as stiff as the rod itself, and into the body at the rod's start.
    const V3 back = cross(e, arm) * (-std::copysign(1.0, ang) / L);
    const double vOut = -dot(sim.vel[w.stopKs] - sim.vel[w.stopRack], back);
    const double F = clampd(3.0e6 * over * L + 500.0 * std::max(vOut, 0.0), 0.0, 400e3);
    sim.force[w.stopKs] += back * F;
    sim.force[w.stopRack] -= back * F;
  }
}

void Vehicle::shiftIndices(long dn, long db, long dobj) {
  const auto N = [&](u32& i) { i = u32(long(i) + dn); };
  const auto B = [&](u32& i) { i = u32(long(i) + db); };
  obj = u16(long(obj) + dobj);
  for (Wheel& w : wheels) {
    N(w.axIn);
    N(w.axOut);
    for (u32& r : w.ring) N(r);
    N(w.stopKl);
    N(w.stopKu);
    N(w.stopKs);
    N(w.stopRack);
    for (auto& s : w.spoke)
      for (u32& e : s) B(e);
  }
  for (HydroAxle& a : axle)
    for (HydroStrut& s : a.s) {
      N(s.top);
      N(s.low);
    }
  for (auto* list : {&engineNodes, &gearboxNodes, &structureNodes})
    for (u32& n : *list) N(n);
  for (auto* list : {&radiatorBeams, &engineMountBeams, &hydraulicBeams, &engineBlockBeams, &gearboxBeams, &bellBeams, &tankBeams, &wingMountBeams})
    for (u32& b : *list) B(b);
  N(refFrontL);
  N(refFrontR);
  N(refRearL);
  N(refRearR);
  for (u32& b : tieBeam) B(b);
  for (u32& n : shaftNode) N(n);
}

void Vehicle::wreck(Sim& sim) {
  if (exploded) return;
  exploded = true;
  burnTime = 1000.0;  // no fire: it was only broken
  engineRunning = false;
  for (Wheel& w : wheels) w.attached = false;
  for (Obj& o : sim.objs)
    if (o.root == obj) {
      o.kind = OBJ_PROP;
      o.alwaysAwake = false;
    }
}

// Electronic stability control and traction control, working the same brakes as the driver.
void Vehicle::updateStability(const Controls& in, double dt) {
  for (Wheel& w : wheels) w.espTorque = 0;
  absHold_ -= dt;
  espHold_ -= dt;
  tcsHold_ -= dt;
  absActive = absHold_ > 0;
  espActive = espHold_ > 0;
  tcsActive = tcsHold_ > 0;
  double wantEsp = 1.0, wantTcs = 1.0;
  const double v = std::max(dot(vCog, fwd), 0.0);
  if (tune.esp && v > 5.0 && !dmg.hydraulics) {
    // What the driver asks for: a yaw rate from steering and speed, as far as the tyres can deliver.
        const double delta = steerAct * tune.steer * spec.lockAngle;
    const double limit = 0.9 * spec.tyreMu * 9.81 / v;
    const double wanted = clampd(v * delta / (spec.wheelbase * (1.0 + spec.understeer * v * v)), -limit, limit);
    const double err = yawRate_ - wanted;
    const double slip = std::asin(clampd(dot(norm(vCog), left), -1, 1));
    // The tail is coming round when the car turns further than asked, or slides well past the line.
    const bool oversteer = std::fabs(yawRate_) > std::fabs(wanted) || std::fabs(slip) > 0.10;
    // Body slip counts too: it is the one thing a yaw rate alone does not show in a slow spin.
    const double slipErr = std::fabs(slip) > 0.07 ? (slip - std::copysign(0.07, slip)) * -2.5 : 0.0;
    // Bumps and ordinary steering make the measured values twitch. Only what lasts is a real slide.
    espErrLP_ += (err + slipErr - espErrLP_) * clampd(dt / 0.04, 0, 1);
    const double e = espErrLP_;
    // A bend that is merely a little wide needs no help at all: it takes real understeer to step in.
    const double dead = oversteer ? 0.10 : 0.28;
    if (std::fabs(e) > dead) {
      const double torque = clampd((std::fabs(e) - dead) * 3200.0, 0.0, 2100.0);
      // Too much yaw to the left wants a moment to the right: brake a wheel on the right.
      const bool brakeLeft = e < 0;
      for (Wheel& w : wheels)
        if (w.left == brakeLeft && w.front == oversteer) w.espTorque = oversteer ? torque : torque * 0.55;
      // Gas is only taken away from a car that is really coming round.
      if (oversteer) wantEsp = clampd(1.0 - (std::fabs(e) - dead) * 1.4, 0.35, 1.0);
      if (torque > 200.0) espHold_ = 0.3;
    }
  }
  if (tune.tcs) {
    // Traction control: what counts is the driven axle as a whole. One wheel running away on its own is
    // braked, which the differential turns into drive at the other one. Only when both spin is the
    // engine pulled back, and a moment's spin over a bump is let go.
    double mean = 0, worst = 0;
    int driven = 0;
    const double allowed = std::max(1.5, 0.16 * v);
    for (Wheel& w : wheels) {
      if (!w.driven) continue;
      const double overrun = w.omega * w.R - v;
      mean += overrun;
      driven++;
      worst = std::max(worst, overrun);
    }
    mean /= std::max(driven, 1);
    for (Wheel& w : wheels) {
      if (!w.driven) continue;
      const double extra = (w.omega * w.R - v) - mean;
      if (extra > 1.0 && worst > allowed) w.espTorque += clampd((extra - 1.0) * 220.0, 0.0, 900.0);
    }
    spinLP_ += (std::max(mean - allowed, 0.0) - spinLP_) * clampd(dt / 0.035, 0, 1);
    if (spinLP_ > 0.15) {
      wantTcs = clampd(1.0 - spinLP_ * 0.28, 0.25, 1.0);
      tcsHold_ = 0.25;
    }
  }
  espThrottle_ += (wantEsp - espThrottle_) * clampd(dt * (wantEsp < espThrottle_ ? 30.0 : 4.0), 0, 1);
  tcsThrottle_ += (wantTcs - tcsThrottle_) * clampd(dt * (wantTcs < tcsThrottle_ ? 25.0 : 7.0), 0, 1);
  (void)in;
}

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
  w.upHint = sim.pos[w.ring[0]] - C;

  // A flat tyre does not shrink: the carcass collapses between rim and road and the car comes down
  // onto it, so it still carries load, only on a much softer and lower cushion.
  const double Rflat = lerpd(w.R, spec.rimRadius, w.deflate);
  const double kRad = lerpd(w.kRadial, w.kRadial * 0.12, w.deflate);
  // A dented rim is out of round. The dent rolls past the road once per revolution, and the car feels it.
  double Rnow = Rflat;
  {
    const double sideSign = w.left ? 1.0 : -1.0;
    const V3 yAxis = a * sideSign;
    const V3 upV = sim.pos[w.ring[0]] - C;
    const V3 xAxis = norm(upV - yAxis * dot(upV, yAxis));
    const V3 zAxis = cross(xAxis, yAxis);
    const double phi = std::atan2(dot(V3{0, 0, -1}, zAxis), dot(V3{0, 0, -1}, xAxis)) - w.theta * sideSign;
    Rnow -= rimDentAt(w, phi);
  }

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
      double muS = spec.tyreMu * clampd(loadSens, 0.6, 1.25) * surfaceFriction(surf);
      if (surf != SURF_ASPHALT && surf != SURF_CONCRETE) muS *= spec.looseGrip;
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

      // Tread on the limit lays rubber; on a flat tyre the carcass scrubs at any slip.
      const double slipSpeed = std::hypot(Vsx, Vy);
      w.skid = smoothstepd(0.95, 1.5, s) * clampd(Fz / 1500.0, 0, 1) * smoothstepd(0.8, 2.5, slipSpeed);
      if (w.deflate > 0.5) w.skid = std::max(w.skid, 0.5 * smoothstepd(1.0, 4.0, absV));

      if (s > 1.3 && absV + std::fabs(w.omega * Re) > 4.0 && Fz > 800.0 &&
          (sim.stepCount & 31) == 0 && sim.events.size() < 500) {
        const bool soft = surf == SURF_GRASS || surf == SURF_GRAVEL || surf == SURF_SNOW;
        sim.events.push_back({patch, vC, float(s), soft ? EV_DUST : EV_TIRE_SMOKE});
      }
    }
    w.Fz = Fz;
    w.patchPoint = patch;
    w.patchNormal = n;
    // A heavy blow through the tyre leaves its mark on the rim, right where it struck.
    if (Fz > spec.dentFz && (sim.stepCount & 3) == 0) addRimDamage(w, patch, (Fz - spec.dentFz) * 8e-9, 0.0);
  } else {
    w.skid = 0;
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
    double pedal = brakeAct * w.brakeMax * tune.brake + w.espTorque;
    if (!w.front) {
      // Citroen brake bias: rear pressure follows rear suspension pressure, i.e. rear load.
      const double loadRatio = clampd(axle[1].s[w.left ? 0 : 1].pGas / 100e5, 0.3, 1.3);
      if (!spec.conventional) pedal *= loadRatio;
    }
    // Brakes are powered by the same HP circuit: lost accumulator pressure weakens them.
    if (!spec.conventional) pedal *= clampd(accumulator / 80e5, 0.15, 1.0);
    Tb = pedal;

    // ABS, active above walking pace.
    const double Vwheel = std::fabs(speed);
    if (tune.abs && Vwheel > 2.0 && (brakeAct > 0.02 || w.espTorque > 0)) {
      // Slip controller around the tyre's peak (about -0.11). Proportional in both directions, so
      // it hovers near maximum grip rather than cycling deep into and out of lock-up.
      const double kap = w.contact ? (omega * Rnow - speed) / std::max(Vwheel, 1.0) : 0.0;
      const double rate = clampd((kap + 0.10) * 45.0, -16.0, 5.0);
      w.absFrac = clampd(w.absFrac + rate * dt, 0.02, 1.0);
      if (w.absFrac < 0.75 && brakeAct > 0.02) absHold_ = 0.25;
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
  // A deep dent lifts the bead off the rim and the tyre loses its air.
  double deepest = 0;
  for (float d : w.dent) deepest = std::max(deepest, double(d));
  if (w.deflate == 0 && (w.Fz > spec.burstFz || deepest > 0.016)) {
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
  if (v > 33.0) wantFirm = true;  // about 120 km/h
  if (wantFirm)
    firmHold = 1.4;
  else
    firmHold = std::max(0.0, firmHold - dt);
  hydractiveSoft = firmHold <= 0;

  // ---- Pump, pressure regulator and accumulator: real volumes, gas law for the pressure.
  {
    const double Vg = std::max(kAccV0 - accFluid, kAccV0 * 0.05);
    accumulator = accFluid <= 0 ? std::max(0.0, kAccP0 * (1.0 + accFluid / 20e-6)) : kAccP0 * std::pow(kAccV0 / Vg, 1.35);
    if (accumulator < kCutIn) pumpCharging = true;
    if (accumulator > kCutOut) pumpCharging = false;
    const bool drive = engineRunning && !dmg.engineMount && reservoir > 2e-4;
    const double pumpFlow = drive ? kPumpDisplacement * rpm() / 60.0 : 0.0;
    if (pumpCharging && pumpFlow > 0) {
      accFluid += pumpFlow * dt;
      reservoir -= pumpFlow * dt;
      pumpTorque = accumulator * kPumpDisplacement / (2.0 * PI);
    } else {
      pumpTorque = drive ? 0.6 : 0.0;
    }
    if (dmg.hydraulics) {
      // A burst line empties the accumulator and the reservoir onto the road.
      const double leak = std::min(accFluid, 90e-6 * dt);
      accFluid -= leak;
      reservoir = std::max(0.0, reservoir - 40e-6 * dt);
    }
    // Brakes run from the same circuit: only pressing the pedal further costs fluid.
    const double pedalRate = std::max(0.0, brakeAct - prevBrake_) / dt;
    prevBrake_ = brakeAct;
    drawAccumulator(pedalRate * kBrakeVolume * dt);
    // Power steering: fluid goes with rack movement.
    drawAccumulator(std::fabs(rackRate_) * kRackArea * dt);
  }

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
    drawAccumulator(std::fabs(rate) * kRamArea * 2.0 * dt);
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
      // A sealed damper has a fixed charge, so its gas volume follows from its length alone. Summing up
      // small changes would drift, and on such a small gas volume that twists the whole car on its springs.
      if (spec.conventional) s.Vf = s.VfRest + s.A * (s.Lnom - s.L);
      else s.Vf -= s.A * s.vel * dt;
      if (s.burst) s.Vf -= kFluidLeakSinkRate * dt;

      double Vgas = s.V0 - s.Vf;
      Vgas = clampd(Vgas, s.V0 * 0.08, s.V0);
      double p = s.p0 * std::pow(s.V0 / Vgas, 1.35);
      if (s.Vf < 0) p *= std::max(0.0, 1.0 + s.Vf / (0.1 * s.V0));  // sphere empty, line goes slack
      if (s.burst) p *= 0.0;
      s.pGas = p;

      // Damper valve, degressive above the knee velocity.
      const int mode = hydractiveSoft ? 1 : 0;
      const double c = (s.vel > 0 ? s.reb[mode] : s.bump[mode]) * tune.damper;
      constexpr double kKnee = 0.30;
      const double av = std::fabs(s.vel);
      const double fd = av < kKnee ? c * av : c * (kKnee + 0.45 * (av - kKnee));
      double F = p * s.A - std::copysign(fd, s.vel);

      // Rubber bump stop and rebound stop.
      if (s.L < s.Lmin) F += 250e3 * (s.Lmin - s.L) * (1.0 + 18.0 * (s.Lmin - s.L));
      if (s.L > s.Lmax) F -= 350e3 * (s.L - s.Lmax);

      // Anti-roll bar + ram, acting at the wheel stations.
      const double delta = (side == 0 ? 1.0 : -1.0) * (A.s[0].L - A.s[1].L - A.ram);
      F -= A.kBar * tune.bar * delta;

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
      // A car on steel-and-gas dampers has no height correction to chase the road with.
      if (std::fabs(A.levelErrLP) > 0.003 && !spec.conventional) {
        const bool canAdd = accumulator > A.s[0].pGas + 10e5;
        // Proportional, so that it converges instead of hunting around the target height.
        constexpr double kLevelGain = 0.45;     // 1/s
        constexpr double kLevelMaxFlow = 18e-6;  // m^3/s per strut
        double flow = clampd(A.s[0].A * kLevelGain * A.levelErrLP, -kLevelMaxFlow, kLevelMaxFlow);
        if (flow > 0 && !canAdd) flow = 0;
        for (int side = 0; side < 2; side++) {
          if (A.s[side].burst) continue;
          A.s[side].Vf += flow * dt;
        }
        const double total = flow * 2.0 * dt;
        if (flow > 0)
          drawAccumulator(total, false);  // stays in the suspension
        else
          reservoir -= total;  // height corrector returning fluid to the tank
      }
    }
  }

}

void Vehicle::drawAccumulator(double volume, bool returnsToTank) {
  accFluid -= volume;
  if (returnsToTank) reservoir += volume;
}

void Vehicle::settleFluid(Sim& sim) {
  // Preset the sphere volumes for the static load so the first frame already sits near height.
  for (int ax = 0; ax < 2; ax++) {
    const double corner = (ax == 0 ? spec.frontShare : 1.0 - spec.frontShare) * totalMass * 9.81 * 0.5 - 380.0;
    for (int side = 0; side < 2; side++) {
      HydroStrut& s = axle[ax].s[side];
      const double p = corner / s.A;
      const double Vg = s.V0 * std::pow(s.p0 / p, 1.0 / 1.35);
      s.Vf = s.V0 - Vg;
      s.VfRest = s.Vf;
      s.pGas = p;
    }
    const double p = corner / axle[ax].s[0].A;
    axle[ax].regVf = axle[ax].regV0 - axle[ax].regV0 * std::pow(axle[ax].regP0 / p, 1.0 / 1.35);
  }
}

// ---------------------------------------------------------------------------------------
// Engine, clutch, gearbox, differential
// ---------------------------------------------------------------------------------------
void Vehicle::applyBlockTorque(Sim& sim, const std::vector<u32>& nodes, double torque) {
  // Reaction torque on a rigid block, about the car's transverse axis, as a pure couple.
  if (nodes.empty()) return;
  V3 c{};
  for (u32 n : nodes) c += sim.pos[n];
  c = c / double(nodes.size());
  double arm2 = 0;
  for (u32 n : nodes) {
    const V3 r = sim.pos[n] - c;
    arm2 += len2(r - left * dot(r, left));
  }
  if (arm2 < 1e-6) return;
  for (u32 n : nodes) sim.force[n] += cross(left, sim.pos[n] - c) * (torque / arm2);
}

void Vehicle::presetRolling(double v, int gearIndex) {
  gear = gearIndex;
  shiftTarget = gearIndex;
  shiftPhase = 0;
  double wheelOmega = 0;
  for (Wheel& w : wheels) {
    w.omega = v / (w.R - 0.006);
    wheelOmega = w.omega;
  }
  carrierOmega = wheelOmega;
  spiderOmega = 0;
  const double r = gear == 0 ? 0.0 : (gear > 0 ? spec.gears[gear - 1] : -spec.reverse) * spec.finalDrive;
  inputOmega = r * carrierOmega;
  discOmega = inputOmega;
  engineOmega = std::max(90.0, inputOmega);
  clutch = 1;
  clutchRamp_ = 1;
}

void Vehicle::updateDrivetrain(Sim& sim, const Controls& in, double dt) {
  // Mode buttons are edge-triggered upstream.
  if (in.toggleAuto) autoBox = !autoBox;
  if (in.toggleSport) sportMode = !sportMode;
  if (in.toggleActiva) activaOn = !activaOn;
  if (in.cycleHeight) heightMode = (heightMode + 1) % 3;
  if (in.toggleIgnition) {
    if (engineRunning)
      engineRunning = false;
    else if (!dmg.seized)
      starterClock = 1.2;
  }

  const double rangeRatio = (tune.lowRange && spec.lowRange > 0) ? spec.lowRange : 1.0;
  auto ratioOf = [this, rangeRatio](int g) { return g == 0 ? 0.0 : (g > 0 ? spec.gears[g - 1] : -spec.reverse) * spec.finalDrive * rangeRatio; };

  // ---- Gear selection. A shift is a sequence of physical steps: open the clutch, pull the box into
  // neutral, let the synchro bring the input shaft to the new speed, engage, close the clutch. How
  // long it takes follows from the speeds and inertias involved.
  sinceShift += dt;
  auto request = [&](int g) {
    if (g == shiftTarget || dmg.gearbox) return;
    shiftTarget = g;
    shiftPhase = 1;
    shiftClock = 0;
    sinceShift = 0;
  };
  if (in.requestGear != 99) request(in.requestGear);
  if (in.shiftUp) request(std::min(5, shiftTarget + 1));
  if (in.shiftDown) request(std::max(-1, shiftTarget - 1));
  // The box goes by the driver's foot, smoothed a little. It must not follow what the traction control does
  // to the throttle, or each cut would call for a shift, and each shift for another cut.
  shiftPedal_ += (in.throttle - shiftPedal_) * clampd(dt / 0.15, 0, 1);
  if (autoBox && shiftPhase == 0 && sinceShift > 0.9 && gear >= 1) {
    const double r = std::fabs(inputOmega) / kRpmToRad;
    if (r > lerpd(spec.upshiftLow, spec.upshiftHigh, shiftPedal_) && gear < spec.nGears)
      request(gear + 1);
    else if (r < lerpd(spec.downshiftLow, spec.downshiftHigh, shiftPedal_) && gear > 1)
      request(gear - 1);
  }
  double syncTorque = 0;
  if (shiftPhase == 1) {
    shiftClock += dt;
    if (shiftClock > 0.07) {
      gear = 0;
      shiftPhase = shiftTarget == 0 ? 0 : 2;
    }
  }
  if (shiftPhase == 2) {
    const double wanted = ratioOf(shiftTarget) * carrierOmega;
    const double slip = wanted - inputOmega;
    // Reverse has no synchro, so it only goes in when everything has nearly stopped.
    if (shiftTarget < 0 && std::fabs(carrierOmega) > 4.0) {
      syncTorque = 0;
    } else if (std::fabs(slip) < 4.0) {
      gear = shiftTarget;
      inputOmega = wanted;
      shiftPhase = 0;
      clutchRamp_ = 0;
    } else {
      syncTorque = clampd(slip * kInputInertia / dt, -spec.synchroTorque, spec.synchroTorque);
    }
  }
  if (dmg.gearbox) gear = 0;
  const double ratio = ratioOf(gear);

  // ---- Combustion: throttle plate, manifold filling, then torque from the air actually in the
  // cylinders. That lag is why a real engine does not react instantly to the pedal.
  const double rpmNow = rpm();
  double th = throttleAct;
  bool ignition = engineRunning;
  if (fuelLitres <= 0 || dmg.seized) ignition = false;
  if (throttleAct > 0.15 && !engineRunning && !dmg.seized && starterClock <= 0 && fuelLitres > 0) starterClock = 1.2;
  // Automated shifting manages the revs: throttle closed on the way up, a blip to match on the way down.
  if (shiftPhase != 0) {
    const double wanted = std::fabs(ratioOf(shiftTarget) * carrierOmega);
    th = shiftPhase == 2 && wanted > engineOmega + 15.0 ? 0.45 : 0.0;
  }
  if (ignition) {
    if (rpmNow < spec.idleRpm + 100) th = std::max(th, clampd((spec.idleRpm + 100 - rpmNow) / 250.0, 0, 0.55));
    if (rpmNow > spec.revLimit) th = 0;
    if (dmg.coolantTemp > 118.0) th *= clampd(1.0 - (dmg.coolantTemp - 118.0) / 15.0, 0.2, 1.0);
  } else {
    th = 0;
  }
  constexpr double kManifoldMin = 0.22;
  const double manifoldTarget = kManifoldMin + (1.0 - kManifoldMin) * std::pow(th, 0.75);
  manifold += (manifoldTarget - manifold) * clampd(dt / 0.04, 0, 1);
  const double load = clampd((manifold - kManifoldMin) / (1.0 - kManifoldMin), 0, 1);

  // Six firing strokes per two revolutions: three torque pulses per crank revolution. This pulsation
  // is what shakes the block on its mounts at idle.
  crankAngle = std::fmod(crankAngle + engineOmega * dt, 2.0 * PI);
  const double fric = (spec.frictionBase + spec.frictionPerRpm * rpmNow) * (oilLevel < 0.2 ? 2.5 : 1.0);
  double fullTorque = fullLoadTorque(rpmNow);
  if (spec.torque) {
    fullTorque = 0;
    for (int i = 1; i < spec.torquePoints; i++)
      if (rpmNow < spec.torque[i][0]) {
        fullTorque = lerpd(spec.torque[i - 1][1], spec.torque[i][1], (rpmNow - spec.torque[i - 1][0]) / (spec.torque[i][0] - spec.torque[i - 1][0]));
        break;
      }
  }
  if (spec.turbo) {
    // The turbos need exhaust flow first: below that it is a 5-litre V8, and then the shove arrives.
    const double wantBoost = load * clampd((rpmNow - spec.boostFrom) / spec.boostSpan, 0.0, 1.0);
    boost += (wantBoost - boost) * clampd(dt / (wantBoost > boost ? 0.45 : 0.12), 0, 1);
    // Off boost it is still a 5-litre engine, with about what such an engine makes unaided.
    fullTorque = lerpd(std::min(fullTorque, spec.offBoost), fullTorque, boost);
  }
  double comb = ignition ? load * (fullTorque + fric) : 0.0;
  comb *= 1.0 + 0.25 * std::cos(3.0 * crankAngle);
  combustionTorque = comb;
  double Te = comb - fric - 2.0 - pumpTorque;
  if (dmg.seized) Te -= 900.0 * clampd(engineOmega / 5.0, 0, 1);
  if (starterClock > 0) {
    starterClock -= dt;
    Te += 55.0;
    if (rpmNow > 500.0 && !dmg.seized && fuelLitres > 0) {
      engineRunning = true;
      starterClock = 0;
    }
  }
  engineTorqueOut = Te;
  if (ignition) fuelLitres = std::max(0.0, fuelLitres - load * rpmNow * 1.6e-6 * dt);

  // ---- Clutch: automated. Slips at launch, opens for a shift, closes progressively afterwards.
  clutchRamp_ = std::min(1.0, clutchRamp_ + dt * 11.0);
  if (shiftPhase != 0 || gear == 0 || dmg.bellHousing) {
    clutch = 0;
  } else {
    const double rIn = std::fabs(inputOmega) / kRpmToRad;
    double e;
    if (rIn > 1150.0)
      e = 1;
    else
      e = throttleAct > 0.03 ? smoothstepd(1000.0, 2100.0, rpmNow) : 0.0;
    // Launch control: the clutch keeps slipping until the turbos have something to work with.
    // It only applies pulling away from rest. Once the clutch has locked, it stays out of it until the car
    // stops again: lifting or the traction control must never reopen the clutch at speed.
    if (std::fabs(speed) < 1.5 && in.throttle < 0.05) launching_ = true;
    if (rIn > spec.launchRpm * 0.8 || (launching_ && rpmNow - rIn < 250.0 && rIn > 1150.0) || std::fabs(speed) > 20.0) launching_ = false;
    if (spec.turbo && launching_ && gear == 1) {
      // How high it holds follows the driver's foot.
      const double gate = lerpd(1500.0, spec.launchRpm, in.throttle * in.throttle);
      e = std::min(e, smoothstepd(gate - std::min(1100.0, gate * 0.45), gate, rpmNow));
    }
    clutch = e * clutchRamp_;
  }

  // Torsion damper between clutch disc and input shaft.
  const double Td = kDamperRate * dampAngle + kDamperDamp * (discOmega - inputOmega);

  // Friction at the clutch, as an exact stick-slip limit between flywheel and disc.
  const double wePred = engineOmega + Te * dt / spec.engineInertia;
  const double wdPred = discOmega - Td * dt / kDiscInertia;
  const double need = (wePred - wdPred) / (dt * (1.0 / spec.engineInertia + 1.0 / kDiscInertia));
  const double cap = spec.clutchCapacity * clutch;
  const double Tc = clampd(need, -cap, cap);
  engineOmega = std::max(0.0, engineOmega + (Te - Tc) * dt / spec.engineInertia);
  discOmega += (Tc - Td) * dt / kDiscInertia;
  if (engineRunning && engineOmega < 380.0 * kRpmToRad) {
    engineRunning = false;
    note(dmg.seized ? "MOTOR STEHT" : "MOTOR ABGEWUERGT");
  }

  // ---- Halfshafts: torsion springs from the side gears to the wheels. Open differential: carrier
  // speed is the mean of the side gears, and the spider gears freewheel between them.
  // One differential per driven axle. With all-wheel drive both hang on the same carrier: the transfer
  // case is locked, which is what gets a heavy truck up a loose slope.
  double TL = 0, TR = 0;
  const auto driveAxle = [&](int ax, double& spider) {
    Wheel* wL = nullptr;
    Wheel* wR = nullptr;
    for (Wheel& w : wheels)
      if (w.driven && (!spec.awd || (w.front ? 0 : 1) == ax)) (w.left ? wL : wR) = &w;
    const double wsL = carrierOmega + spider, wsR = carrierOmega - spider;
    double tl = 0, tr = 0;
    const int base = ax * 2;
    if (wL && wL->attached) {
      shaftAngle[base] += (wsL - wL->omega) * dt;
      tl = spec.shaftRate * shaftAngle[base] + spec.shaftDamp * (wsL - wL->omega);
    }
    if (wR && wR->attached) {
      shaftAngle[base + 1] += (wsR - wR->omega) * dt;
      tr = spec.shaftRate * shaftAngle[base + 1] + spec.shaftDamp * (wsR - wR->omega);
    }
    if (wL) wL->driveT = tl;
    if (wR) wR->driveT = tr;
    // A shaft shears when overloaded, typically with a locked wheel and a dropped clutch.
    for (int i = 0; i < 2; i++) {
      Wheel* w = i == 0 ? wL : wR;
      if (w && w->attached && std::fabs(i == 0 ? tl : tr) > spec.shaftShear) {
        w->attached = false;
        note("ANTRIEBSWELLE ABGESCHERT");
      }
    }
    spider += ((tr - tl) / kSpiderInertia) * dt;
    if (tune.lockDiffs) {
      // Locked: both wheels of the axle turn as one, whatever grip each of them has.
      spider = 0;
    } else if (spec.diffPreload > 0) {
      // Limited slip: clutch packs hold the two sides together, harder the more torque goes through.
      // A wheel going light on a bump cannot take all the power and just spin up.
      const double cap = spec.diffPreload + spec.diffRamp * std::fabs(tl + tr);
      const double stop = spider * kSpiderInertia / dt;
      spider -= clampd(stop, -cap, cap) * dt / kSpiderInertia;
    }
    spider *= 1.0 - clampd(dt * 0.6, 0, 1);
    TL += tl;
    TR += tr;
  };
  driveAxle(0, spiderOmega);
  if (spec.awd) driveAxle(1, spiderOmega2);


  // Gear mesh: in gear, input shaft and final drive turn as one. Oil churning and bearing friction
  // cost torque, more of it the faster things spin.
  const double loss = 0.8 + 0.018 * std::fabs(inputOmega);
  if (ratio != 0) {
    const double J = spec.carrierInertia + kInputInertia * ratio * ratio;
    const double Tloss = std::fabs(carrierOmega) > 0.2 ? std::copysign(loss * std::fabs(ratio), carrierOmega) : 0.0;
    carrierOmega += (Td * ratio - Tloss - TL - TR) * dt / J;
    inputOmega = ratio * carrierOmega;
  } else {
    // Neutral: shafts run free, and the synchro is the only link while selecting a gear.
    const double r2 = ratioOf(shiftTarget);
    inputOmega += (Td + syncTorque - std::copysign(std::min(loss, std::fabs(inputOmega) * kInputInertia / dt), inputOmega)) * dt / kInputInertia;
    carrierOmega += (-TL - TR - syncTorque * r2) * dt / spec.carrierInertia;
  }
  dampAngle += (discOmega - inputOmega) * dt;

  // Reaction torques on the housings: the engine rocks against combustion, the gearbox against what
  // it passes on to the shafts. Together that is the drive torque reacted through the mounts.
  if (!engineNodes.empty()) {
    applyBlockTorque(sim, engineNodes, -(comb - fric));
    applyBlockTorque(sim, gearboxNodes, Tc - (TL + TR));
  }
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
  for (u32 i : o.nodes) {
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
  if (!wingMountBeams.empty()) {
    bool gone = false;
    for (u32 id : wingMountBeams) gone |= (sim.beams[id].flags & BF_BROKEN) != 0;
    if (gone) {
      for (u32 id : wingMountBeams) sim.sever(id);
      wingMountBeams.clear();
      note("HECKFLUEGEL ABGERISSEN");
    }
  }
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
  if (!dmg.gearbox && plastified(gearboxBeams, 0.02) >= 2) {
    dmg.gearbox = true;
    note("GETRIEBEGEHAEUSE GEBROCHEN");
  }
  if (!dmg.bellHousing) {
    int broken = 0;
    for (u32 id : bellBeams) broken += (sim.beams[id].flags & BF_BROKEN) ? 1 : 0;
    if (broken * 2 >= int(bellBeams.size()) && !bellBeams.empty()) {
      dmg.bellHousing = true;
      note("GETRIEBE VOM MOTOR GERISSEN");
    }
  }
  if (!dmg.tank && plastified(tankBeams, 0.08) >= 3) {
    dmg.tank = true;
    note("TANK LECK");
  }
  if (dmg.tank) fuelLitres = std::max(0.0, fuelLitres - dt * 0.4);
  // Oil temperature: it carries the bearings' heat away. As the level drops the same heat goes into
  // less and less oil, and at the end nothing lubricates the bearings. That, not the coolant, seizes it.
  {
    const double work = 95.0 + 25.0 * clampd(rpm() / 6000.0, 0, 1) * clampd(throttleAct + 0.3, 0, 1);
    const double starved = oilLevel < 0.35 ? (0.35 - oilLevel) / 0.35 : 0.0;
    const double target = (engineRunning ? work : 40.0) + 170.0 * starved;
    dmg.oilTemp += (target - dmg.oilTemp) * clampd(dt / 18.0, 0, 1);
  }
  // The sump is the lowest part of the engine: dragging it over the road holes it.
  if (!dmg.oilLeak && !engineNodes.empty()) {
    const V3& sumpN = sim.pos[engineNodes[0]];
    // Its steel skid plate takes a knock: a touchdown leaves a dent, only grinding along holes it.
    const double clearance = sumpN.z - sim.world->height(sumpN.x, sumpN.y);
    const double closing = -dot(sim.vel[engineNodes[0]], V3{0, 0, 1});
    if (clearance < 0.01) sumpContact_ += std::sqrt(std::max(len2(vCog), 0.0)) * dt;
    else sumpContact_ = std::max(0.0, sumpContact_ - dt * 0.4);
    if (sumpContact_ > 2.5 || (clearance < 0.005 && closing > 7.0)) {
      dmg.oilLeak = true;
      note("OELWANNE AUFGERISSEN");
    }
  }
  if (dmg.oilLeak) {
    // A holed sump empties in about a minute.
    oilLevel = std::max(0.0, oilLevel - dt * 0.017);
    if (oilLevel <= 0 && engineRunning && !dmg.seized) {
      dmg.seized = true;
      note("MOTOR OHNE OEL FESTGEGANGEN");
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
  for (Wheel& w : wheels) updateRimDamage(sim, w);

  // Halfshafts have plunge and angle limits at their CV joints: pulled out, they come apart.
  int si = 0;
  for (Wheel& w : wheels) {
    if (!w.driven) continue;
    const int idx = (spec.awd && !w.front ? 2 : 0) + (w.left ? 0 : 1);
    if (w.attached && std::fabs(len(sim.pos[w.axIn] - sim.pos[shaftNode[idx]]) - shaftLen[idx]) > (spec.awd ? 0.28 : 0.10)) {
      w.attached = false;
      note("ANTRIEBSWELLE RAUSGERISSEN");
    }
    si++;
  }
  (void)si;

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

// Beams only damp along their own axis, so a flat sheet of them has nothing to stop it from ringing
// across its plane. Shape damping fits the rigid motion of the whole structure and brakes each node's
// deviation from it. Rigid motion is untouched by construction, since it is what gets subtracted, and
// the sum of the forces is zero. Only slow deviations count: above a few m/s a node is taking part in
// a crash or has come off, and that must not be held back.
void Vehicle::shapeDamping(Sim& sim, double dt) {
  double M = 0;
  V3 c{}, V{};
  if (lastSplit_ != sim.splitGeneration) {
    // Pieces that have torn off are on their own now.
    lastSplit_ = sim.splitGeneration;
    structureNodes.erase(std::remove_if(structureNodes.begin(), structureNodes.end(), [&](u32 n) { return sim.nodeObj[n] != obj; }), structureNodes.end());
  }
  for (u32 n : structureNodes) {
    const double m = sim.mass[n];
    M += m;
    c += sim.pos[n] * m;
    V += sim.vel[n] * m;
  }
  if (M <= 0) return;
  c = c / M;
  V = V / M;
  if (bodyTick_ % 4 == 0) {
    // Angular velocity from angular momentum and the inertia tensor.
    double I[3][3] = {};
    V3 L{};
    for (u32 n : structureNodes) {
      const double m = sim.mass[n];
      const V3 r = sim.pos[n] - c;
      L += cross(r, sim.vel[n] - V) * m;
      const double xx = r.x * r.x, yy = r.y * r.y, zz = r.z * r.z;
      I[0][0] += m * (yy + zz);
      I[1][1] += m * (xx + zz);
      I[2][2] += m * (xx + yy);
      I[0][1] -= m * r.x * r.y;
      I[0][2] -= m * r.x * r.z;
      I[1][2] -= m * r.y * r.z;
    }
    I[1][0] = I[0][1];
    I[2][0] = I[0][2];
    I[2][1] = I[1][2];
    const double c00 = I[1][1] * I[2][2] - I[1][2] * I[2][1], c01 = I[1][2] * I[2][0] - I[1][0] * I[2][2], c02 = I[1][0] * I[2][1] - I[1][1] * I[2][0];
    const double det = I[0][0] * c00 + I[0][1] * c01 + I[0][2] * c02;
    if (std::fabs(det) > 1e-6) {
      const double c11 = I[0][0] * I[2][2] - I[0][2] * I[2][0], c12 = I[0][1] * I[2][0] - I[0][0] * I[2][1], c22 = I[0][0] * I[1][1] - I[0][1] * I[1][0];
      omegaFit_ = V3{c00 * L.x + c01 * L.y + c02 * L.z, c01 * L.x + c11 * L.y + c12 * L.z, c02 * L.x + c12 * L.y + c22 * L.z} / det;
    }
  }
  constexpr double kTau = 0.012;           // decay time of slow deformation [s]
  constexpr double kFull = 1.5, kOff = 4;  // deviation speeds [m/s] for full effect and none
  for (u32 n : structureNodes) {
    const V3 rel = sim.vel[n] - (V + cross(omegaFit_, sim.pos[n] - c));
    const double s2 = len2(rel);
    if (s2 > kOff * kOff) continue;
    const double s = std::sqrt(s2);
    const double g = s < kFull ? 1.0 : (kOff - s) / (kOff - kFull);
    sim.force[n] -= rel * (sim.mass[n] * g / kTau);
  }
  (void)dt;
}

void Vehicle::update(Sim& sim, const Controls& in, double dt) {
  if (exploded) {
    // The wreck burns for a good while, fiercely at first.
    burnTime += dt;
    if (burnTime > 4.0 && burnTime - dt <= 4.0)
      for (Obj& o : sim.objs)
        if (o.root == obj) o.scrap = true;
    if ((sim.stepCount & 31) == 0 && burnTime < 90.0) {
      const double fierce = clampd(1.0 - burnTime / 70.0, 0.15, 1.0);
      sim.event(EV_FIRE, cog + fwd * ((sim.stepCount % 7) * 0.25 - 0.75) + up * 0.25, V3{}, float(fierce));
    }
    // Nothing works any more: no suspension, drive or tyre model on a structure in pieces, whose
    // geometry they could not make sense of anyway. What is left is steel hitting the ground.
    updateBodyFrame(sim, dt);
    speed = dot(vCog, fwd);
    return;
  }
  bodyTick_++;
  updateBodyFrame(sim, dt);
  shapeDamping(sim, dt);

  // Rack and pinion: rate-limited, and the rack is moved by changing the tie rod rest lengths.
  // Without pressure there is no power steering: the driver can only heave the rack round slowly.
  const double maxRate = (accumulator > 60e5 ? 2.6 : 0.5) * dt;
  const double steerBefore = steerAct;
  steerAct += clampd(in.steer - steerAct, -maxRate, maxRate);
  rackRate_ = (steerAct - steerBefore) / dt * steerTravel;
  if (!dmg.steering) {
    // Incremental, so that a tie rod bent in a crash stays bent and the car pulls to one side.
    const double delta = steerAct * steerTravel * tune.steer;
    Beam& tl = sim.beams[tieBeam[0]];
    Beam& tr = sim.beams[tieBeam[1]];
    if (!(tl.flags & BF_BROKEN)) tl.L0 -= delta - rackPrev_;
    if (!(tr.flags & BF_BROKEN)) tr.L0 += delta - rackPrev_;
    rackPrev_ = delta;
  }
  applySteerStops(sim, dt);
  updateStability(in, dt);
  throttleAct += clampd(in.throttle * espThrottle_ * tcsThrottle_ - throttleAct, -14.0 * dt, 14.0 * dt);
  // The Citroen brake is powered and has almost no pedal travel: it bites at once.
  brakeAct += clampd(in.brake - brakeAct, -14.0 * dt, 20.0 * dt);
  hbAct_ = in.handbrake;

  // Aerodynamics: applied to the whole body by mass, so it needs no attitude model.
  const double v2 = len2(vCog);
  if (v2 > 0.25) {
    const V3 Fa = vCog * (-0.5 * kAirDensity * dragArea * std::sqrt(v2));
    const double perKg = 1.0 / totalMass;
    const Obj& o = sim.objs[obj];
    // Wing and diffuser press down in proportion to the speed along the car, shared evenly.
    const double vFwd = dot(vCog, fwd);
    const V3 Fd = up * (-0.5 * kAirDensity * spec.downforceCla * vFwd * vFwd);
    for (u32 i : o.nodes) sim.force[i] += (Fa + Fd) * (sim.mass[i] * perKg);
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
