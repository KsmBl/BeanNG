#include "duel.h"

#include <cmath>
#include <cstring>
#include "props.h"

namespace {
// Turns what was just built where it stands. It must not have been simulated yet.
void turnNodes(Sim& sim, u32 first, u32 end, V3 about, double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (u32 i = first; i < end; i++) {
    const V3 q = sim.pos[i] - about;
    sim.pos[i] = V3{about.x + c * q.x - s * q.y, about.y + s * q.x + c * q.y, sim.pos[i].z};
  }
}
}  // namespace

void Duel::build() {
  world.buildArena();
  sim.world = &world;
  int at = 0;
  for (int side = 0; side < 2; side++) {
    const World::Base& base = world.base(side);
    for (int k = 0; k < TK_COUNT; k++, at++) {
      const V3 p = base.slot(k);
      spawn[at] = p;
      yaw[at] = base.facing;
      tanks[at] = buildLeopard(sim, p, &rigs[at], TurretKind(k));
      tanks[at].ammo = tanks[at].spec().ammo;
      const u32 first = sim.objs[tanks[at].hullObj].nodeStart;
      tankFirst[at] = first;
      turnNodes(sim, first, u32(sim.pos.size()), p, base.facing);
    }
  }
  tankNodes = tankFirst[1] - tankFirst[0];
  tanksEnd = u32(sim.pos.size());
  for (int side = 0; side < 2; side++) {
    const World::Base& base = world.base(side);
    for (int k = 0; k < kCarKinds; k++, at++) {
      const V3 p = base.slot(TK_COUNT + k);
      spawn[at] = p;
      yaw[at] = base.facing;
      const u32 first = u32(sim.pos.size());
      carFirst[at - kTanks] = first;
      if (k == 0) cars.push_back(buildXantia(sim, p, 0.0, &rigs[at]));
      else if (k == 1) cars.push_back(buildOne1(sim, p, &rigs[at]));
      else cars.push_back(buildHummer(sim, p, &rigs[at]));
      turnNodes(sim, first, u32(sim.pos.size()), p, base.facing);
    }
  }
  carsEnd = u32(sim.pos.size());
  // The soldiers' nodes stand deep underground whenever nobody is drawn with them.
  for (int i = 0; i < 2; i++) {
    sim.beginObj(OBJ_PROP, false);
    soldierFirst[i] = u32(sim.pos.size());
    for (int k = 0; k < kSoldierNodes; k++) sim.addNode(V3{}, 1.0, NF_FIXED);
    poseSoldier(sim, soldierFirst[i], Soldier{}, true, 0);
    for (int k = 0; k < kSoldierNodes; k++) sim.rest[soldierFirst[i] + u32(k)] = sim.pos[soldierFirst[i] + u32(k)];
    sim.endObj();
  }
  sim.firstSceneryNode = u32(sim.pos.size());
  populateArena(sim, world);
  sim.finalize();

  // Each comes to rest on its own, and then it sleeps until something touches it.
  const auto scenery = sim.active;
  const auto alone = [&](std::vector<u32> ids, auto settle) {
    sim.active.clear();
    for (u32 id : ids) {
      sim.objs[id].awake = true;
      sim.active.push_back(id);
    }
    settle();
    for (u32 id : ids) {
      Obj& o = sim.objs[id];
      o.alwaysAwake = false;
      o.awake = false;
      o.sleepTimer = 0;
      for (u32 n : o.nodes) sim.vel[n] = {};
    }
  };
  for (Tank& t : tanks) alone({u32(t.hullObj), u32(t.turretObj)}, [&] { t.settle(sim, 2.0); });
  for (Vehicle& v : cars) {
    v.engineRunning = false;
    alone({u32(v.obj)}, [&] { settleVehicle(sim, v, 2.5); });
  }
  sim.active.clear();
  for (u32 id : scenery) {
    const Obj& o = sim.objs[id];
    if (o.nodeStart >= sim.firstSceneryNode) sim.active.push_back(id);
  }
  sim.events.clear();
  sim.scrapes.clear();
  sim.craterWorld = &world;
  sim.logBlasts = true;
  for (int i = 0; i < kTanks; i++) tanksStart[i] = tanks[i];
  carsStart = cars;
  updateFrames();
  updateFrames();
  placePlayers();
  simStart = sim;
  blastSeen = sim.blastCount;
}

void Duel::updateFrames() {
  for (int i = 0; i < kTanks; i++) {
    const Tank& t = tanks[i];
    VehicleFrame& f = frames[i];
    f.cog = t.cog + t.up * 0.45;
    f.fwd = t.fwd;
    f.left = t.left;
    f.up = t.up;
    f.speed = t.speed;
    f.halfLength = 3.6;
    f.halfWidth = 1.75;
    f.height = 2.6;
  }
  for (size_t i = 0; i < cars.size(); i++) {
    const Vehicle& v = cars[i];
    VehicleFrame& f = frames[kTanks + int(i)];
    f.cog = v.cog;
    f.fwd = v.fwd;
    f.left = v.left;
    f.up = v.up;
    f.speed = v.speed;
    f.halfLength = 2.3;
    f.halfWidth = 1.0;
    f.height = 1.6;
  }
}

const V3 Duel::kSoldierRest{0, 0, -400};

namespace {
enum SoldierPart { SP_PELVIS, SP_TORSO, SP_HEAD, SP_THIGH_L, SP_THIGH_R, SP_SHIN_L, SP_SHIN_R, SP_UPPER_L, SP_UPPER_R, SP_FORE_L, SP_FORE_R, SP_GUN0 };
constexpr double kProneAngle = 1.5;
constexpr double kHip = 0.93, kKnee = 0.50, kShoulder = 1.42, kElbow = 1.13, kWrist = 0.87, kHipY = 0.10, kShoulderY = 0.215;
// The middle of each part, in the bind pose: x forward, y left, z up from the ground.
const V3 kPartMid[Duel::kSoldierParts] = {
    {0, 0, 0.93}, {0, 0, 1.25}, {0, 0, 1.62}, {0, kHipY, 0.72}, {0, -kHipY, 0.72}, {0, kHipY, 0.27}, {0, -kHipY, 0.27},
    {0, kShoulderY, 1.28}, {0, -kShoulderY, 1.28}, {0, kShoulderY, 1.0}, {0, -kShoulderY, 1.0},
    {0, -kShoulderY, 0.83}, {0, -kShoulderY, 0.83}, {0, -kShoulderY, 0.83}, {0, -kShoulderY, 0.83}, {0, -kShoulderY, 0.83}, {0, -kShoulderY, 0.83},
};
// Turned about the sideways axis. A positive angle swings what hangs down forward, and lifts what points forward.
V3 swingF(V3 q, double a) {
  const double c = std::cos(a), s = std::sin(a);
  return V3{q.x * c - q.z * s, q.y, q.x * s + q.z * c};
}
// A part in the body's own frame, moved from the bind pose by turning it about a joint and bringing that joint along.
struct Limb {
  V3 joint, to;
  double angle;
  double inward = 0;  // turned about the vertical first: an arm reaching across the chest
  V3 at(V3 q) const {
    const V3 d = q - joint;
    const double c = std::cos(inward), s = std::sin(inward);
    return to + swingF(V3{d.x * c - d.y * s, d.x * s + d.y * c, d.z}, angle);
  }
};
}  // namespace

const char* Duel::soldierPartName(int part) {
  static const char* names[kSoldierParts] = {"pelvis", "torso", "head", "thigh_l", "thigh_r", "shin_l", "shin_r", "upper_l", "upper_r", "fore_l", "fore_r", "gun_knife", "gun_pistol", "gun_ar", "gun_sniper", "gun_rpg", "gun_none"};
  return names[part];
}

void Duel::poseSoldier(Sim& sim, u32 first, const Soldier& s, bool hidden, int weapon) {
  const double moving = std::min(std::hypot(s.v.x, s.v.y) / 2.0, 1.0);
  const double run = std::min(std::hypot(s.v.x, s.v.y) / 7.0, 1.0);
  const double phase = s.stride * 2.1;
  const double swingAmp = (0.45 + 0.35 * run) * moving;
  const double aL = std::sin(phase) * swingAmp, aR = -std::sin(phase) * swingAmp;
  // The knee bends as the leg comes through, more at a run.
  const double kneeL = std::max(0.0, -std::cos(phase)) * (0.7 + 0.5 * run) * moving + 0.05, kneeR = std::max(0.0, std::cos(phase)) * (0.7 + 0.5 * run) * moving + 0.05;
  const double dropByStride = 0.02 * moving * std::fabs(std::sin(phase));
  // Where it looks: positive upward. The body follows a little, the head and the weapon all the way.
  const double look = clampd(-s.pitch, -0.9, 0.9);
  const bool longGun = weapon >= 2;
  const double lean = look * 0.25 + 0.06 * run + (longGun ? 0.05 : 0.0);
  const V3 pelvisJoint{0, 0, kHip};
  const Limb torso{pelvisJoint, pelvisJoint, lean};
  const Limb head{V3{0, 0, 1.5}, torso.at(V3{0, 0, 1.5}), look + s.prone * 1.3};
  const Limb thighL{V3{0, kHipY, kHip}, V3{0, kHipY, kHip}, aL}, thighR{V3{0, -kHipY, kHip}, V3{0, -kHipY, kHip}, aR};
  const Limb shinL{V3{0, kHipY, kKnee}, thighL.at(V3{0, kHipY, kKnee}), aL - kneeL}, shinR{V3{0, -kHipY, kKnee}, thighR.at(V3{0, -kHipY, kKnee}), aR - kneeR};
  // The arms hold what is carried up in front: a long gun with both hands, a short one further out, nothing swings them.
  const double raiseR = weapon == 0 ? 0.25 + aL * 0.5 : (longGun ? 0.55 : 1.15);
  const double bendR = weapon == 0 ? 0.5 : (longGun ? 1.15 : 0.35);
  const double raiseL = weapon == 0 ? aR * 0.8 : (longGun ? 1.0 : 1.0);
  const double bendL = weapon == 0 ? 0.3 : (longGun ? 0.55 : 0.6);
  const V3 shoulderL{0, kShoulderY, kShoulder}, shoulderR{0, -kShoulderY, kShoulder};
  const double acrossL = weapon == 0 ? 0.0 : (longGun ? 0.9 : 0.75), acrossR = weapon == 0 ? 0.0 : 0.35;
  const Limb upperL{shoulderL, torso.at(shoulderL), raiseL + (weapon ? look : 0.0), acrossL}, upperR{shoulderR, torso.at(shoulderR), raiseR + (weapon ? look : 0.0), acrossR};
  const V3 elbowL{0, kShoulderY, kElbow}, elbowR{0, -kShoulderY, kElbow};
  const Limb foreL{elbowL, upperL.at(elbowL), upperL.angle + bendL, acrossL}, foreR{elbowR, upperR.at(elbowR), upperR.angle + bendR, acrossR};
  const V3 grip{0, -kShoulderY, kWrist - 0.04};
  // Lying down turns the figure forward, and what it looks along has to come back up by as much.
  const double liftBack = s.prone * kProneAngle;
  const Limb gun{grip, foreR.at(grip), weapon == 0 ? foreR.angle - 1.2 : look + liftBack};

  // Then the whole figure: turned to where it faces, and laid down when it is lying.
  const double cy = std::cos(s.yaw), sy = std::sin(s.yaw);
  const V3 ahead{-cy, -sy, 0};
  const V3 left = V3{-ahead.y, ahead.x, 0};
  const auto world = [&](V3 q) {
    if (s.prone > 0) {
      // About the feet, flat onto the belly with the head towards where it looks.
      q = swingF(q, -s.prone * kProneAngle);
      q.z += s.prone * 0.18;
    }
    q.z -= dropByStride;
    return s.p + ahead * q.x + left * q.y + V3{0, 0, q.z};
  };
  const auto corner = [](int part, int k) {
    return kPartMid[part] + V3{(k & 1) ? 0.06 : -0.06, (k & 2) ? 0.06 : -0.06, (k & 4) ? 0.06 : -0.06};
  };
  const auto place = [&](int part, const Limb* limb) {
    for (int k = 0; k < kPartNodes; k++) {
      const u32 n = first + u32(part * kPartNodes + k);
      sim.pos[n] = hidden || !limb ? kSoldierRest + corner(part, k) : world(limb->at(corner(part, k)));
    }
  };
  const Limb still{V3{}, V3{}, 0.0};
  place(SP_PELVIS, &still);
  place(SP_TORSO, &torso);
  place(SP_HEAD, &head);
  place(SP_THIGH_L, &thighL);
  place(SP_THIGH_R, &thighR);
  place(SP_SHIN_L, &shinL);
  place(SP_SHIN_R, &shinR);
  place(SP_UPPER_L, &upperL);
  place(SP_UPPER_R, &upperR);
  place(SP_FORE_L, &foreL);
  place(SP_FORE_R, &foreR);
  for (int g = 0; g < kSoldierParts - SP_GUN0; g++) {
    const int part = SP_GUN0 + g;
    place(part, g == weapon ? &gun : nullptr);
  }
}

void Duel::walk(Soldier& s, const WalkInput& in, double dt) const {
  // Forward is away from the camera, which looks along -(cos yaw, sin yaw).
  const double cy = std::cos(in.yaw), sy = std::sin(in.yaw);
  const V3 ahead{-cy, -sy, 0}, side{sy, -cy, 0};
  V3 want = ahead * in.forward + side * in.strafe;
  const double wl = len(want);
  if (wl > 1.0) want = want / wl;
  const double top = in.sprint ? 7.0 : 4.2;
  const double rate = s.grounded ? 40.0 : 6.0;
  const V3 flat{s.v.x, s.v.y, 0};
  V3 dv = want * top - flat;
  const double dl = len(dv), most = rate * dt;
  if (dl > most) dv = dv * (most / dl);
  s.v.x += dv.x;
  s.v.y += dv.y;
  if (s.grounded && in.jump) {
    s.v.z = 4.6;
    s.grounded = false;
  }
  s.v.z -= 9.81 * dt;
  s.p += s.v * dt;
  s.stride += len(flat) * dt;
  s.yaw = in.yaw;

  // Pushed out of whatever it has walked into. Feet and head at the middle, and a ring around at knee and chest.
  s.grounded = false;
  const double r = Soldier::kRadius;
  const auto resolve = [&](V3 offset, bool feet) {
    Contact c;
    if (!world.contact(s.p + offset, c)) return;
    if (c.n.z > 0.62) {
      // Ground. Steep ground does not count: that is how the mountains end the map.
      if (feet) {
        s.p.z += c.depth / std::max(c.n.z, 0.3);
        if (s.v.z < 0) s.v.z = 0;
        s.grounded = true;
      }
      return;
    }
    if (c.n.z < -0.6) {
      s.p.z -= c.depth;
      if (s.v.z > 0) s.v.z = 0;
      return;
    }
    s.p += c.n * c.depth;
    const double into = dot(s.v, c.n);
    if (into < 0) s.v -= c.n * into;
  };
  resolve(V3{0, 0, 0}, true);
  for (double z : {0.55, 1.5}) {
    for (int i = 0; i < 8; i++) {
      const double a = double(i) * PI * 0.25;
      resolve(V3{std::cos(a) * r, std::sin(a) * r, z}, false);
    }
  }
  resolve(V3{0, 0, Soldier::kHeight}, false);

  // Stones and rubble, as what they are for someone on foot: something to go round.
  for (const Block& b : sim.blocks) {
    const V3 a = sim.pos[b.n[0]], d = sim.pos[b.n[7]];
    const V3 mid = (a + d) * 0.5;
    if (mid.z < s.p.z - 0.2 || mid.z > s.p.z + Soldier::kHeight) continue;
    const V3 to{s.p.x - mid.x, s.p.y - mid.y, 0};
    const double reach = len(V3{a.x - d.x, a.y - d.y, 0}) * 0.42 + r;
    const double l2 = len2(to);
    if (l2 >= reach * reach || l2 < 1e-9) continue;
    const double l = std::sqrt(l2);
    const V3 out = to / l;
    s.p += out * (reach - l);
    const double into = dot(s.v, out);
    if (into < 0) s.v -= out * into;
  }

  // And vehicles, each as a box in its own axes.
  for (const VehicleFrame& f : frames) {
    const V3 q = s.p - f.cog;
    if (std::fabs(q.z) > f.height + 1.0) continue;
    const double lf = dot(q, f.fwd), ll = dot(q, f.left);
    const double ox = f.halfLength + r - std::fabs(lf), oy = f.halfWidth + r - std::fabs(ll);
    if (ox <= 0 || oy <= 0) continue;
    const V3 out = ox < oy ? f.fwd * (lf > 0 ? 1.0 : -1.0) : f.left * (ll > 0 ? 1.0 : -1.0);
    s.p += out * std::min(ox, oy);
    const double into = dot(s.v, out);
    if (into < 0) s.v -= out * into;
  }
}

V3 Duel::vehicleExit(int v) const {
  // Beside it, on whichever side is free. World walls and the ground are sorted out by the first step of walking.
  const VehicleFrame& f = frames[v];
  const double out = f.halfWidth + 0.8;
  for (double sg : {1.0, -1.0}) {
    V3 p = f.cog + f.left * (sg * out);
    p.z = world.height(p.x, p.y) + 0.05;
    Contact c;
    if (!world.contact(p + V3{0, 0, 1.0}, c)) return p;
  }
  V3 p = f.cog - f.fwd * (f.halfLength + 1.0);
  p.z = world.height(p.x, p.y) + 0.05;
  return p;
}

int Duel::nearestVehicle(V3 p, double reach) const {
  int best = -1;
  double bestD = 1e30;
  for (int v = 0; v < kVehicles; v++) {
    if (vehicleDead(v) || occupied(v)) continue;
    const VehicleFrame& f = frames[v];
    const V3 q = p - f.cog;
    // From the edge of the vehicle, not its middle: a tank is a long thing to walk round.
    const double lf = std::max(std::fabs(dot(q, f.fwd)) - f.halfLength, 0.0), ll = std::max(std::fabs(dot(q, f.left)) - f.halfWidth, 0.0);
    const double d = std::hypot(lf, ll);
    if (d < reach && d < bestD) {
      best = v;
      bestD = d;
    }
  }
  return best;
}

int Duel::nearestToRepair(V3 p) const {
  int best = -1;
  double bestD = 3.0;
  for (int v = 0; v < kVehicles; v++) {
    if (vehicleDead(v)) continue;
    const VehicleFrame& f = frames[v];
    const V3 q = p - f.cog;
    const double d = std::hypot(std::max(std::fabs(dot(q, f.fwd)) - f.halfLength, 0.0), std::max(std::fabs(dot(q, f.left)) - f.halfWidth, 0.0));
    if (d < bestD) {
      best = v;
      bestD = d;
    }
  }
  return best;
}

void Duel::boardVehicle(int p, int v) {
  players[p].vehicle = v;
  const u16 a = v < kTanks ? tanks[v].hullObj : cars[size_t(v - kTanks)].obj;
  const u16 b = v < kTanks ? tanks[v].turretObj : a;
  sim.objs[a].alwaysAwake = true;
  sim.objs[b].alwaysAwake = true;
  sim.wake(a);
  sim.wake(b);
  if (v >= kTanks) cars[size_t(v - kTanks)].engineRunning = true;
}

void Duel::leaveVehicle(int p) {
  const int v = players[p].vehicle;
  if (v < 0) return;
  Soldier& s = players[p].soldier;
  s.p = vehicleExit(v);
  s.v = V3{};
  const u16 a = v < kTanks ? tanks[v].hullObj : cars[size_t(v - kTanks)].obj;
  const u16 b = v < kTanks ? tanks[v].turretObj : a;
  sim.objs[a].alwaysAwake = false;
  sim.objs[b].alwaysAwake = false;
  if (v >= kTanks) cars[size_t(v - kTanks)].engineRunning = false;
  players[p].vehicle = -1;
}

void Duel::stepPlayers(const Intent in[2]) {
  // Blasts reach those on foot.
  for (; blastsDone < sim.blastLog.size(); blastsDone++) {
    const Sim::BlastRecord& b = sim.blastLog[blastsDone];
    for (Player& pl : players) {
      if (pl.vehicle >= 0) continue;
      const double d = len(pl.soldier.p + V3{0, 0, 0.9} - b.c);
      const double kill = b.radius * 0.55;
      if (d < kill) pl.soldier.hp -= 140.0 * (1.0 - d / kill) * std::min(b.impulse / 1000.0, 1.0);
    }
  }
  for (int i = 0; i < 2; i++) {
    Player& pl = players[i];
    Soldier& s = pl.soldier;
    if (in[i].use) {
      if (pl.vehicle >= 0) {
        leaveVehicle(i);
      } else if (phase == PLAYING) {
        const int v = nearestVehicle(s.p, 2.5);
        if (v >= 0) boardVehicle(i, v);
      }
    }
    if (pl.vehicle >= 0) {
      // Inside what goes up, one goes up with it.
      if (vehicleDead(pl.vehicle)) {
        s.hp = 0;
        s.p = frames[pl.vehicle].cog;
      }
      s.yaw = in[i].walk.yaw;
      s.pitch = in[i].pitch;
      continue;
    }
    // Repairs are done on foot, beside the thing, and it has to hold still for them.
    if (in[i].repair && pl.repairClock <= 0 && pl.items[IT_REPAIR] > 0 && phase == PLAYING) {
      const int v = nearestToRepair(s.p);
      if (v >= 0) {
        pl.repairing = v;
        pl.repairClock = kRepairTime;
      }
    }
    if (pl.repairClock > 0) {
      const VehicleFrame& f = frames[pl.repairing];
      const V3 q = s.p - f.cog;
      const double off = std::hypot(std::max(std::fabs(dot(q, f.fwd)) - f.halfLength, 0.0), std::max(std::fabs(dot(q, f.left)) - f.halfWidth, 0.0));
      if (off > 3.0 || std::fabs(f.speed) > 1.0 || vehicleDead(pl.repairing)) {
        pl.repairClock = 0;
      } else {
        pl.repairClock -= kDt;
        if (pl.repairClock <= 0) {
          pl.repairClock = 0;
          pl.items[IT_REPAIR]--;
          if (pl.repairing < kTanks) {
            tanks[pl.repairing].repair(sim);
          } else {
            Vehicle& car = cars[size_t(pl.repairing - kTanks)];
            car.dmg = decltype(car.dmg){};
            for (Wheel& w : car.wheels) {
              w.deflate = 0;
              w.runoutX = w.runoutZ = 0;
            }
          }
        }
      }
    }
    pl.walkDebt += kDt;
    while (pl.walkDebt >= kWalkDt) {
      pl.walkDebt -= kWalkDt;
      walk(s, in[i].walk, kWalkDt);
    }
    s.pitch = in[i].pitch;
    pl.lastInput = in[i].seq;
    // What drives into someone at speed does not stop to apologise.
    for (const VehicleFrame& f : frames) {
      if (std::fabs(f.speed) < 3.0) continue;
      const V3 q = s.p - f.cog;
      if (std::fabs(q.z) > f.height + 1.0) continue;
      if (std::fabs(dot(q, f.fwd)) < f.halfLength + 0.5 && std::fabs(dot(q, f.left)) < f.halfWidth + 0.5) s.hp -= std::fabs(f.speed) * 40.0 * kDt * 20.0;
    }
  }
}

void Duel::restart() {
  world.clearCraters();
  sim = simStart;
  for (int i = 0; i < kTanks; i++) tanks[i] = tanksStart[i];
  cars = carsStart;
  winner = -1;
  phaseClock = 0;
  blastSeen = sim.blastCount;
  matchId++;
  blastsDone = 0;
  placePlayers();
  updateFrames();
}

void Duel::placePlayers() {
  for (int i = 0; i < 2; i++) {
    Player& p = players[i];
    p.vehicle = -1;
    p.ready = false;
    p.repairClock = 0;
    p.repairing = -1;
    p.walkDebt = 0;
    const World::Base& base = world.base(i);
    const double cx = std::cos(base.facing), sx = std::sin(base.facing);
    p.soldier = Soldier{};
    p.soldier.p = base.centre + V3{cx, sx, 0} * 4.0;
    p.soldier.yaw = base.facing + PI;
  }
}

bool Duel::choose(int player, const u8 items[IT_COUNT], bool ready) {
  if (phase != CHOOSING || player < 0 || player > 1) return false;
  if (slotsUsed(items) > kLoadoutSlots) return false;
  Player& p = players[player];
  for (int i = 0; i < IT_COUNT; i++) p.items[i] = items[i];
  p.ready = ready;
  return true;
}

void Duel::startMatch() {
  phase = PLAYING;
  phaseClock = 0;
}

void Duel::step(const Intent in[2]) {
  stepPlayers(in);
  sim.forces(kDt);
  Controls still;
  still.brake = 1;
  for (int i = 0; i < kTanks; i++) {
    Tank& t = tanks[i];
    const Controls* c = &still;
    bool owned = false;
    for (int p = 0; p < 2; p++)
      if (players[p].vehicle == i) {
        c = &in[p].c;
        owned = true;
      }
    if (owned || sim.objs[t.hullObj].awake || sim.objs[t.turretObj].awake) {
      if (!t.destroyed) {
        sim.wake(t.hullObj);
        sim.wake(t.turretObj);
      }
      t.update(sim, *c, kDt);
    } else {
      t.tickFire(sim, kDt);
    }
  }
  const bool blast = sim.blastCount != blastSeen;
  blastSeen = sim.blastCount;
  Controls parked;
  parked.handbrake = 1.0;
  for (size_t i = 0; i < cars.size(); i++) {
    Vehicle& v = cars[i];
    // What a shell has torn open right beside it no longer works as a vehicle.
    if (blast && !v.exploded && len(v.cog - sim.lastBlast) < sim.lastBlastRadius * 0.30) v.wreck(sim);
    const Controls* c = &parked;
    for (int p = 0; p < 2; p++)
      if (players[p].vehicle == kTanks + int(i)) c = &in[p].c;
    if (sim.objs[v.obj].awake) v.update(sim, *c, kDt);
  }
  sim.finish(kDt);
  updateFrames();
  phaseClock += kDt;

  const bool both = players[0].present && players[1].present;
  switch (phase) {
    case WAITING:
      if (both) {
        restart();
        phase = CHOOSING;
      }
      break;
    case CHOOSING:
      if (!both) {
        restart();
        phase = WAITING;
      } else if ((players[0].ready && players[1].ready) || phaseClock > kChooseTime) {
        // What they chose survives the reset of everything else.
        u8 kept[2][IT_COUNT];
        for (int i = 0; i < 2; i++) memcpy(kept[i], players[i].items, IT_COUNT);
        restart();
        for (int i = 0; i < 2; i++) memcpy(players[i].items, kept[i], IT_COUNT);
        startMatch();
      }
      break;
    case PLAYING: {
      if (!both) {
        restart();
        phase = WAITING;
        break;
      }
      // Whoever dies loses, whatever they were doing at the time.
      const bool dead[2] = {players[0].soldier.hp <= 0, players[1].soldier.hp <= 0};
      if (dead[0] || dead[1]) {
        if (dead[0] != dead[1]) {
          winner = dead[0] ? 1 : 0;
          wins[winner]++;
        }
        phase = FINISHED;
        phaseClock = 0;
      }
      break;
    }
    case FINISHED:
      if (!both) {
        restart();
        phase = WAITING;
      } else if (phaseClock > kEndPause) {
        restart();
        phase = CHOOSING;
      }
      break;
  }
}

void Duel::drainEvents(std::vector<SimEvent>& out) {
  out.insert(out.end(), sim.events.begin(), sim.events.end());
  sim.events.clear();
  sim.scrapes.clear();
}
