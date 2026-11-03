#include "duel.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
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
  for (int k = 0; k < 2; k++)
    for (int i = 0; i < kMinesEach; i++) mineFirst[k][i] = addMarker(sim, k == 0 ? V3{0.13, 0.13, 0.035} : V3{0.24, 0.24, 0.06}, COL_STEEL);
  {
    // The wrecks stand in the scenery's range: they never move by themselves, so the same few messages keep them right.
    double yaw0 = 0.7;
    for (int i = 0; i < kWrecks; i++) {
      const V3 site = world.wreckSites()[size_t(i)];
      wreckSpawn[i] = site;
      wrecks[i] = buildLeopard(sim, site, &wreckRigs[i], TK_STANDARD);
      wreckFirst[i] = sim.objs[wrecks[i].hullObj].nodeStart;
      turnNodes(sim, wreckFirst[i], u32(sim.pos.size()), site, yaw0);
      yaw0 += 1.9;
    }
  }
  // Crates: a couple in every house, one beside each wreck.
  const auto addCrate = [&](V3 at, bool wreck) {
    Crate c;
    c.p = at;
    c.wreckSite = wreck;
    c.marker = addMarker(sim, V3{0.32, 0.22, 0.2}, COL_WOOD);
    crates.push_back(c);
  };
  for (const World::House& h : world.houses()) {
    const double floor = world.height(World::houseInside(h, 4.5, 3.75, 0).x, World::houseInside(h, 4.5, 3.75, 0).y);
    addCrate(World::houseInside(h, 1.2, 6.0, floor + 0.21), false);
    addCrate(World::houseInside(h, 7.8, 1.0, floor + 0.21), false);
  }
  for (int i = 0; i < kWrecks; i++) {
    V3 at = wreckSpawn[i] + V3{4.5, 3.0, 0};
    at.z = world.height(at.x, at.y) + 0.21;
    addCrate(at, true);
  }
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
  for (Tank& t : wrecks) {
    alone({u32(t.hullObj), u32(t.turretObj)}, [&] { t.settle(sim, 2.0); });
    t.makeWreck(sim);
  }
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
  for (int i = 0; i < kWrecks; i++) wrecksStart[i] = wrecks[i];
  carsStart = cars;
  for (const Crate& c : crates) moveMarker(sim, c.marker, c.p);
  rollLoot();
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

  // Stones and rubble, as what they are for someone on foot: something to go round. What lies lower than a knee can
  // be stepped over.
  for (const Block& b : sim.blocks) {
    const V3 a = sim.pos[b.n[0]];
    const V3 ex = sim.pos[b.n[1]] - a, ey = sim.pos[b.n[2]] - a, ez = sim.pos[b.n[4]] - a;
    const V3 mid = a + (ex + ey + ez) * 0.5;
    if (std::fabs(mid.x - s.p.x) > 6.0 || std::fabs(mid.y - s.p.y) > 6.0) continue;
    const V3 body{s.p.x, s.p.y, clampd(mid.z, s.p.z + 0.4, s.p.z + Soldier::kHeight)};
    const V3 q = body - a;
    const double lx = clampd(dot(q, ex) / len2(ex), 0, 1), ly = clampd(dot(q, ey) / len2(ey), 0, 1), lz = clampd(dot(q, ez) / len2(ez), 0, 1);
    const V3 near = a + ex * lx + ey * ly + ez * lz;
    V3 out = body - near;
    out.z = 0;
    if (near.z < s.p.z + 0.35 || near.z > s.p.z + Soldier::kHeight) continue;
    const double l2 = len2(out);
    if (l2 >= r * r || l2 < 1e-12) continue;
    const double l = std::sqrt(l2);
    out = out / l;
    s.p += out * (r - l);
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

void Duel::rollLoot() {
  struct Roll {
    const char* name;
    int weight, count;
  };
  // Mostly ammunition, and now and then something that changes the match.
  static const Roll kTable[] = {
      {"kleinmunition", 26, 60}, {"granaten", 13, 4}, {"lenkraketen", 7, 2}, {"raketen", 8, 2}, {"raketenwerfer", 7, 1},
      {"scharfschuetzengewehr", 6, 1}, {"scharfschuetzenmunition", 8, 5}, {"reparaturset", 8, 1}, {"tretmine", 6, 2},
      {"panzermine", 4, 1}, {"bombencode", 4, 1}, {"leben", 6, 50},
  };
  int total = 0;
  for (const Roll& r : kTable) total += r.weight;
  u32 seed = matchId * 2654435761u + 977u;
  for (Crate& c : crates) {
    seed = seed * 1664525u + 1013904223u;
    if (c.wreckSite) {
      // What an old tank still has in it: its own kind of ammunition.
      const bool missile = (seed >> 16) % 4 == 0;
      for (int k = 0; k < kGiveKinds; k++)
        if (!strcmp(kGiveNames[k], missile ? "lenkraketen" : "granaten")) c.what = k;
      c.count = missile ? 2 : 5;
      continue;
    }
    int pick = int((seed >> 16) % u32(total));
    for (const Roll& r : kTable) {
      if (pick < r.weight) {
        for (int k = 0; k < kGiveKinds; k++)
          if (!strcmp(kGiveNames[k], r.name)) c.what = k;
        c.count = r.count;
        break;
      }
      pick -= r.weight;
    }
  }
}

int Duel::nearCrate(V3 p) const {
  int best = -1;
  double bestD = 2.2 * 2.2;
  for (size_t i = 0; i < crates.size(); i++) {
    if (crateGone(crates[i])) continue;
    const double d = len2(crates[i].p - p);
    if (d < bestD) {
      bestD = d;
      best = int(i);
    }
  }
  return best;
}

const char* Duel::crateLabel(const Crate& c, char* buf, size_t n) {
  static const char* kLabels[kGiveKinds] = {"Tretmine", "Panzermine", "Reparaturset", "Sturmgewehr", "Kleinmunition", "Scharfschützengewehr", "Scharfschützenmunition", "Raketenwerfer", "Raketen", "Panzergranaten", "Lenkraketen", "Bombencode", "Verbandskasten"};
  snprintf(buf, n, c.count > 1 ? "%s  x%d" : "%s", kLabels[c.what], c.count);
  return buf;
}

const char* const Duel::kGiveNames[kGiveKinds] = {
    "tretmine", "panzermine", "reparaturset", "sturmgewehr", "kleinmunition", "scharfschuetzengewehr", "scharfschuetzenmunition",
    "raketenwerfer", "raketen", "granaten", "lenkraketen", "bombencode", "leben",
};

bool Duel::give(int player, const char* name, int count) {
  if (player < 0 || player > 1 || count < 1) return false;
  Player& p = players[player];
  const std::string n = name;
  const auto addItem = [&](ItemKind k) { p.items[k] = u8(std::min(int(p.items[k]) + count, 99)); };
  if (n == "tretmine") addItem(IT_FOOT_MINE);
  else if (n == "panzermine") addItem(IT_AT_MINE);
  else if (n == "reparaturset") addItem(IT_REPAIR);
  else if (n == "sturmgewehr") p.hasWeapon[WP_AR] = true;
  else if (n == "kleinmunition") p.smallAmmo += count;
  else if (n == "scharfschuetzengewehr") {
    p.hasWeapon[WP_SNIPER] = true;
    p.sniperAmmo += 10 * count;
  } else if (n == "scharfschuetzenmunition") p.sniperAmmo += count;
  else if (n == "raketenwerfer") {
    p.hasWeapon[WP_RPG] = true;
    p.rpgAmmo += 2 * count;
  } else if (n == "raketen") p.rpgAmmo += count;
  else if (n == "granaten") p.tankShells += count;
  else if (n == "lenkraketen") p.tankMissiles += count;
  else if (n == "bombencode") p.strikeCodes += count;
  else if (n == "leben") p.soldier.hp = std::min(p.soldier.hp + count, 100.0);
  else return false;
  return true;
}

V3 Duel::eyeOf(const Soldier& s) {
  // Standing, or stretched out on the ground with the head towards the enemy.
  const double cy = std::cos(s.yaw), sy = std::sin(s.yaw);
  return s.p + V3{-cy, -sy, 0} * (1.32 * s.prone) + V3{0, 0, lerpd(1.45, 0.32, s.prone)};
}

V3 Duel::soldierAt(int p, double lag) const {
  const int back = std::clamp(int(lag * 100.0), 0, kPastSteps - 1);
  return past[(pastAt - back + kPastSteps * 2) % kPastSteps].soldier[p];
}

void Duel::fireWeapon(int p, const Intent& in) {
  Player& pl = players[p];
  const int w = pl.weapon;
  if (!pl.hasWeapon[w] || pl.shotClock > 0) return;
  if (!(in.fireFoot || (in.fireHeld && kWeapons[w].automatic))) return;
  const Soldier& s = pl.soldier;
  const V3 eye = eyeOf(s);
  if (w == WP_KNIFE) {
    pl.shotClock = kWeapons[w].interval;
    const V3 target = soldierAt(1 - p, in.lag);
    Soldier& other = players[1 - p].soldier;
    if (players[1 - p].vehicle < 0 && len(V3{target.x - s.p.x, target.y - s.p.y, target.z - s.p.z}) < 2.2) other.hp -= kWeapons[w].damage;
    return;
  }
  int& ammo = w == WP_SNIPER ? pl.sniperAmmo : (w == WP_RPG ? pl.rpgAmmo : pl.smallAmmo);
  if (ammo <= 0) return;
  // The big rifle takes setting up: it fires lying still behind its bipod, or not at all.
  if (w == WP_SNIPER && s.prone < 0.99) return;
  ammo--;
  pl.shotClock = kWeapons[w].interval;
  V3 dir = norm(in.c.aimPoint - eye);
  // Off the line by a random angle: the same on no two shots, and worse for moving.
  const auto chance = [&] {
    randState = randState * 1664525u + 1013904223u;
    return double(randState >> 16) / 65535.0 * 2.0 - 1.0;
  };
  const double spread = kWeapons[w].spread * (1.0 + (w == WP_SNIPER ? 0.0 : std::min(len(s.v) * 0.5, 2.0)));
  dir = norm(dir + V3{chance(), chance(), chance()} * spread);
  Bullet b;
  b.p = eye + dir * 0.7;
  b.v = dir * kWeapons[w].speed;
  b.owner = p;
  b.kind = w;
  b.lag = in.lag;
  bullets.push_back(b);
  sim.event(EV_MUZZLE, b.p, dir, w == WP_RPG ? 0.5f : (w == WP_SNIPER ? 0.45f : 0.12f));
}

void Duel::stepBullets(double dt) {
  for (size_t i = 0; i < bullets.size();) {
    Bullet& b = bullets[i];
    b.age += dt;
    if (b.kind == kKindStrike) {
      // Straight down the last of the way.
    } else if (b.kind == WP_RPG) {
      // A rocket wanders, more the longer it flies.
      const double wander = 0.25 * std::min(b.age, 3.0);
      b.v += V3{std::sin(b.age * 7.0 + b.p.x), std::cos(b.age * 5.0 + b.p.y), std::sin(b.age * 6.0)} * (wander * 12.0 * dt);
    } else {
      // Gravity, and the wind: the air pushes it sideways by as much as the bullet is slower than it across the wind.
      b.v.z -= 9.81 * dt;
      b.v += wind * (0.28 * dt);
    }
    const V3 from = b.p;
    b.p += b.v * dt;
    if (((b.kind == WP_RPG || b.kind == kKindStrike) && int(b.age * 200.0) % 4 == 0) || (b.kind != WP_RPG && int(b.age * 200.0) % 8 == 0)) sim.event(EV_TRACER, b.p, b.v, 1.f);
    bool stop = b.age > (b.kind == kKindStrike ? 20.0 : 6.0);
    V3 at = b.p;
    const V3 seg = b.p - from;
    const double L = len(seg);
    const int samples = std::max(1, int(L));
    // Those it may hit: the other player, where the shooter saw them.
    if (!stop) {
      const int q = 1 - b.owner;
      if (players[q].vehicle < 0 && b.kind != kKindStrike) {
        const V3 foot = soldierAt(q, b.lag);
        const double t = clampd(dot(V3{foot.x - from.x, foot.y - from.y, 0}, V3{seg.x, seg.y, 0}) / std::max(seg.x * seg.x + seg.y * seg.y, 1e-9), 0, 1);
        const V3 near = from + seg * t;
        if (std::hypot(near.x - foot.x, near.y - foot.y) < 0.38 && near.z > foot.z && near.z < foot.z + Soldier::kHeight) {
          stop = true;
          at = near;
          if (b.kind < WP_RPG) {
            // In the head it counts for more than in the boots.
            const double z = near.z - foot.z;
            players[q].soldier.hp -= kWeapons[b.kind].damage * (z > 1.5 ? 2.0 : (z < 0.7 ? 0.7 : 1.0));
          }
        }
      }
    }
    // Vehicles stop it. What a bullet can do to them comes with their weak spots.
    if (!stop)
      for (int v = 0; v < kVehicles && !stop; v++) {
        const VehicleFrame& f = frames[v];
        const V3 q = b.p - f.cog;
        if (std::fabs(dot(q, f.fwd)) < f.halfLength && std::fabs(dot(q, f.left)) < f.halfWidth && std::fabs(dot(q, f.up)) < f.height * 0.5) {
          stop = true;
          at = b.p;
        }
      }
    // And the world, every metre of the way.
    for (int k = 1; k <= samples && !stop; k++) {
      const V3 p = from + seg * (double(k) / samples);
      Contact c;
      if (world.contact(p, c)) {
        stop = true;
        at = p;
        break;
      }
      for (const Block& bl : sim.blocks) {
        const V3 a = sim.pos[bl.n[0]];
        if (std::fabs(a.x - p.x) > 5.0 || std::fabs(a.y - p.y) > 5.0) continue;
        const V3 ex = sim.pos[bl.n[1]] - a, ey = sim.pos[bl.n[2]] - a, ez = sim.pos[bl.n[4]] - a, q = p - a;
        const double lx = dot(q, ex) / len2(ex), ly = dot(q, ey) / len2(ey), lz = dot(q, ez) / len2(ez);
        if (lx >= 0 && lx <= 1 && ly >= 0 && ly <= 1 && lz >= 0 && lz <= 1) {
          stop = true;
          at = p;
          break;
        }
      }
    }
    if (stop) {
      if (b.kind == WP_RPG) sim.explode(at, 2800.0, 10.0);
      else if (b.kind == kKindStrike) sim.explode(at, 6500.0, 24.0);
      else if (b.age < 6.0) sim.event(EV_SPARK, at, b.v * -0.01, 0.3f);
      bullets[i] = bullets.back();
      bullets.pop_back();
      continue;
    }
    i++;
  }
}

void Duel::callStrike(int p, V3 at) {
  Player& pl = players[p];
  if (pl.strikeCodes <= 0) return;
  pl.strikeCodes--;
  for (int k = 0; k < kStrikeRockets; k++) {
    // Four rockets at a point, none of them on it: anywhere in a circle round it, a good stone's throw across.
    randState = randState * 1664525u + 1013904223u;
    const double a = double(randState >> 16) / 65535.0 * 2.0 * PI;
    randState = randState * 1664525u + 1013904223u;
    const double r = std::sqrt(double(randState >> 16) / 65535.0) * kStrikeSpread;
    Incoming in;
    in.at = at + V3{std::cos(a) * r, std::sin(a) * r, 0};
    in.in = kStrikeDelay + k * kStrikeGap;
    incoming.push_back(in);
  }
}

void Duel::placeMine(int p, int kind) {
  Player& pl = players[p];
  u8& have = pl.items[kind == 0 ? IT_FOOT_MINE : IT_AT_MINE];
  if (have == 0 || pl.vehicle >= 0) return;
  for (int i = 0; i < kMinesEach; i++) {
    Mine& m = mines[kind][i];
    if (m.live) continue;
    const double cy = std::cos(pl.soldier.yaw), sy = std::sin(pl.soldier.yaw);
    // On the ground, a step ahead. A few seconds before it is live, for getting away from it.
    V3 at = pl.soldier.p + V3{-cy, -sy, 0} * 1.0;
    at.z = world.height(at.x, at.y) + (kind == 0 ? 0.03 : 0.05);
    m.live = true;
    m.p = at;
    m.owner = p;
    m.armed = 3.0;
    moveMarker(sim, mineFirst[kind][i], at);
    have--;
    return;
  }
}

void Duel::stepMines(double dt) {
  for (int kind = 0; kind < 2; kind++)
    for (int i = 0; i < kMinesEach; i++) {
      Mine& m = mines[kind][i];
      if (!m.live) continue;
      if (m.armed > 0) {
        m.armed -= dt;
        continue;
      }
      bool off = false;
      if (kind == 0) {
        for (const Player& pl : players)
          if (pl.vehicle < 0 && std::hypot(pl.soldier.p.x - m.p.x, pl.soldier.p.y - m.p.y) < 0.8 && std::fabs(pl.soldier.p.z - m.p.z) < 1.0) off = true;
      }
      // Anything on wheels or tracks sets off either kind by rolling over it.
      for (const VehicleFrame& f : frames) {
        const V3 q = m.p - f.cog;
        if (std::fabs(q.z) < f.height + 1.0 && std::fabs(dot(q, f.fwd)) < f.halfLength && std::fabs(dot(q, f.left)) < f.halfWidth && std::fabs(f.speed) > 0.3) off = true;
      }
      if (!off) continue;
      m.live = false;
      moveMarker(sim, mineFirst[kind][i], V3{0, 0, kMarkerDepth});
      if (kind == 0) sim.explode(m.p + V3{0, 0, 0.2}, 1400.0, 10.0);
      else {
        // Right under it, so the track on that side is gone whatever the armour makes of the blast.
        for (int v = 0; v < kTanks; v++) {
          const VehicleFrame& f = frames[v];
          const V3 q = m.p - f.cog;
          if (std::fabs(dot(q, f.fwd)) < f.halfLength && std::fabs(dot(q, f.left)) < f.halfWidth) tanks[v].mod[dot(q, f.left) > 0 ? TM_TRACK_L : TM_TRACK_R] = 0;
        }
        sim.explode(m.p + V3{0, 0, 0.4}, 6000.0, 12.0);
      }
    }
}

void Duel::boardVehicle(int p, int v) {
  players[p].vehicle = v;
  const u16 a = v < kTanks ? tanks[v].hullObj : cars[size_t(v - kTanks)].obj;
  const u16 b = v < kTanks ? tanks[v].turretObj : a;
  sim.objs[a].alwaysAwake = true;
  sim.objs[b].alwaysAwake = true;
  sim.wake(a);
  sim.wake(b);
  if (v >= kTanks) {
    // Started for whoever gets in, at idle: below that a diesel simply stalls again.
    Vehicle& car = cars[size_t(v - kTanks)];
    car.engineRunning = true;
    car.engineOmega = std::max(car.engineOmega, car.spec.idleRpm * 2.0 * PI / 60.0);
  }
  // What one has been carrying for it goes into the tank.
  if (v < kTanks) {
    Player& pl = players[p];
    Tank& t = tanks[v];
    if (t.ammo >= 0) t.ammo += pl.tankShells;
    pl.tankShells = 0;
    if (t.spec().missiles > 0) {
      t.missiles += pl.tankMissiles;
      pl.tankMissiles = 0;
    }
  }
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
  pastAt = (pastAt + (int(sim.stepCount % 20) == 0 ? 1 : 0)) % kPastSteps;
  for (int i = 0; i < 2; i++) past[pastAt].soldier[i] = players[i].soldier.p;
  bulletDebt += kDt;
  while (bulletDebt >= 0.005) {
    bulletDebt -= 0.005;
    stepBullets(0.005);
    stepMines(0.005);
    for (size_t k = 0; k < incoming.size();) {
      incoming[k].in -= 0.005;
      if (incoming[k].in > 0) {
        k++;
        continue;
      }
      // From high up, already on its way down: there is just time to hear it.
      Bullet b;
      const V3 g{incoming[k].at.x, incoming[k].at.y, world.height(incoming[k].at.x, incoming[k].at.y)};
      b.p = g + V3{40.0, 25.0, 400.0};
      b.v = norm(g - b.p) * 190.0;
      b.kind = kKindStrike;
      b.owner = 0;
      bullets.push_back(b);
      incoming[k] = incoming.back();
      incoming.pop_back();
    }
  }
  for (int i = 0; i < 2; i++) {
    Player& pl = players[i];
    Soldier& s = pl.soldier;
    pl.shotClock = std::max(0.0, pl.shotClock - kDt);
    if (in[i].weapon >= 0 && in[i].weapon < WP_COUNT && pl.hasWeapon[in[i].weapon]) pl.weapon = in[i].weapon;
    // Getting down and up again takes its time, and only the big rifle asks for it.
    if (in[i].setup && pl.weapon == WP_SNIPER && pl.vehicle < 0) pl.wantProne = !pl.wantProne;
    if (pl.weapon != WP_SNIPER || pl.vehicle >= 0) pl.wantProne = false;
    s.prone = clampd(s.prone + (pl.wantProne ? 1.0 : -1.5) * kDt / kSetupTime * (pl.wantProne ? 1.0 : 3.0), 0.0, 1.0);
    if (in[i].strike && phase == PLAYING) callStrike(i, in[i].strikeAt);
    if (pl.vehicle < 0 && phase == PLAYING) {
      fireWeapon(i, in[i]);
      if (in[i].placeFoot) placeMine(i, 0);
      if (in[i].placeTank) placeMine(i, 1);
    }
    if (in[i].use) {
      if (pl.vehicle >= 0) {
        leaveVehicle(i);
      } else if (phase == PLAYING) {
        // A crate within reach comes first; it is the smaller thing to aim for.
        const int c = nearCrate(s.p + V3{0, 0, 0.3});
        if (c >= 0) {
          give(i, kGiveNames[crates[size_t(c)].what], crates[size_t(c)].count);
          moveMarker(sim, crates[size_t(c)].marker, V3{0, 0, kMarkerDepth});
        } else {
          const int v = nearestVehicle(s.p, 2.5);
          if (v >= 0) boardVehicle(i, v);
        }
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
      // Whoever lies down, or is getting up, is not going anywhere; only turning the rifle a little.
      WalkInput wi = in[i].walk;
      // Before and after the match itself one waits where one stands.
      if (phase != PLAYING) wi.forward = wi.strafe = 0, wi.jump = false, wi.sprint = false;
      if (s.prone > 0) wi.forward = wi.strafe = 0, wi.jump = false, wi.sprint = false;
      walk(s, wi, kWalkDt);
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
  for (int i = 0; i < kWrecks; i++) wrecks[i] = wrecksStart[i];
  cars = carsStart;
  winner = -1;
  phaseClock = 0;
  bullets.clear();
  incoming.clear();
  for (auto& row : mines)
    for (Mine& m : row) m.live = false;
  {
    // Up to a stiff breeze, from anywhere.
    randState = randState * 1664525u + 1013904223u + matchId;
    const double a = double(randState >> 16) / 65535.0 * 2.0 * PI;
    randState = randState * 1664525u + 1013904223u;
    const double strength = 1.0 + double(randState >> 16) / 65535.0 * 9.0;
    wind = V3{std::cos(a), std::sin(a), 0} * strength;
  }
  blastSeen = sim.blastCount;
  matchId++;
  blastsDone = 0;
  rollLoot();
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
    p.weapon = WP_PISTOL;
    p.smallAmmo = 24;
    p.sniperAmmo = p.rpgAmmo = 0;
    p.tankShells = p.tankMissiles = p.strikeCodes = 0;
    p.shotClock = 0;
    p.wantProne = false;
    for (int k = 0; k < WP_COUNT; k++) p.hasWeapon[k] = k <= WP_PISTOL;
    if (giveAll) {
      for (bool& h : p.hasWeapon) h = true;
      p.sniperAmmo = 20;
      p.rpgAmmo = 6;
    }
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
    Controls driven = parked;
    for (int p = 0; p < 2; p++)
      if (players[p].vehicle == kTanks + int(i)) {
        driven = in[p].c;
      }
    if (sim.objs[v.obj].awake) v.update(sim, driven, kDt);
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
        for (int i = 0; i < 2; i++) {
          memcpy(players[i].items, kept[i], IT_COUNT);
          players[i].smallAmmo += 60 * players[i].items[IT_SMALL_AMMO];
          if (players[i].items[IT_AR] > 0) {
            players[i].hasWeapon[WP_AR] = true;
            players[i].weapon = WP_AR;
          }
        }
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
