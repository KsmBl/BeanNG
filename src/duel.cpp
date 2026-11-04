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
    for (int k = 0; k < HK_COUNT * TK_COUNT; k++, at++) {
      const V3 p = base.slot(k);
      spawn[at] = p;
      yaw[at] = base.facing;
      tanks[at] = buildLeopard(sim, p, &rigs[at], TurretKind(k % TK_COUNT), HullKind(k / TK_COUNT));
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
      const V3 p = base.slot(HK_COUNT * TK_COUNT + k);
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
  for (int i = 0; i < kMaxPlayers; i++) {
    sim.beginObj(OBJ_PROP, false);
    soldierFirst[i] = u32(sim.pos.size());
    for (int k = 0; k < kSoldierNodes; k++) sim.addNode(V3{}, 1.0, NF_FIXED);
    poseSoldier(sim, soldierFirst[i], Soldier{}, true, 0);
    for (int k = 0; k < kSoldierNodes; k++) sim.rest[soldierFirst[i] + u32(k)] = sim.pos[soldierFirst[i] + u32(k)];
    sim.endObj();
  }
  for (int i = 0; i < 2; i++) {
    sim.beginObj(OBJ_PROP, false);
    missileFirst[i] = u32(sim.pos.size());
    for (int k = 0; k < kMissileNodes; k++) sim.addNode(V3{}, 1.0, NF_FIXED);
    poseMissile(sim, missileFirst[i], false, V3{}, V3{1, 0, 0});
    for (int k = 0; k < kMissileNodes; k++) sim.rest[missileFirst[i] + u32(k)] = sim.pos[missileFirst[i] + u32(k)];
    sim.endObj();
  }
  for (int i = 0; i < kMaxPlayers; i++)
    for (int d = 0; d < kBloodDiscs; d++) {
      sim.beginObj(OBJ_PROP, false);
      bloodFirst[i][d] = u32(sim.pos.size());
      sim.addNode(V3{0, 0, -390.0}, 1.0, NF_FIXED);
      sim.addNode(V3{0, 0, -389.99}, 1.0, NF_FIXED);
      sim.endObj();
    }
  for (int i = 0; i < kMaxPlayers; i++) {
    sim.beginObj(OBJ_PROP, false);
    droneFirst[i] = u32(sim.pos.size());
    for (int k = 0; k < kDroneNodes; k++) sim.addNode(V3{}, 1.0, NF_FIXED);
    poseDrone(sim, droneFirst[i], false, V3{}, 0);
    for (int k = 0; k < kDroneNodes; k++) sim.rest[droneFirst[i] + u32(k)] = sim.pos[droneFirst[i] + u32(k)];
    sim.endObj();
  }
  for (int i = 0; i < kRpgRockets + kStrikeRockets; i++) {
    sim.beginObj(OBJ_PROP, false);
    rocketFirst[i] = u32(sim.pos.size());
    for (int k = 0; k < kMissileNodes; k++) sim.addNode(V3{}, 1.0, NF_FIXED);
    poseMissile(sim, rocketFirst[i], false, V3{}, V3{1, 0, 0}, i < kRpgRockets ? kRpgScale : kStrikeScale);
    for (int k = 0; k < kMissileNodes; k++) sim.rest[rocketFirst[i] + u32(k)] = sim.pos[rocketFirst[i] + u32(k)];
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
  // The nets, carried by a box of nodes around where each tank was built.
  for (int i = 0; i < kTanks + kWrecks; i++) {
    const Tank& t = i < kTanks ? tanks[i] : wrecks[i - kTanks];
    sim.beginObj(OBJ_PROP, false);
    netFirst[i] = u32(sim.pos.size());
    const V3 o = (sim.rest[t.refFL] + sim.rest[t.refFR] + sim.rest[t.refRL] + sim.rest[t.refRR]) * 0.25;
    for (int k = 0; k < 8; k++) {
      const V3 r{(k & 1) ? 3.0 : -3.0, (k & 2) ? 2.0 : -2.0, (k & 4) ? 1.5 : -1.5};
      sim.addNode(o + r + V3{0, 0, -400}, 1.0, NF_FIXED);
      sim.rest[netFirst[i] + u32(k)] = o + r;
    }
    sim.endObj();
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
    if (o.nodeStart >= sim.firstSceneryNode && o.awake) sim.active.push_back(id);
  }
  sim.events.clear();
  sim.scrapes.clear();
  sim.craterWorld = &world;
  sim.logBlasts = true;
  for (int i = 0; i < kTanks; i++) tanksStart[i] = tanks[i];
  for (int i = 0; i < kWrecks; i++) wrecksStart[i] = wrecks[i];
  carsStart = cars;
  for (const Crate& c : crates) moveMarker(sim, c.marker, c.p);
  for (int k = 0; k < kCars; k++)
    for (const NodeGroup& g : rigs[kTanks + k].groups) {
      if (g.name == "engine") carParts[k].engine = g.nodes;
      if (g.name == "radiator") carParts[k].radiator = g.nodes;
      if (g.name == "tank") carParts[k].tank = g.nodes;
      if (g.name == "gearbox") carParts[k].gearbox = g.nodes;
    }
  rollLoot();
  updateFrames();
  updateFrames();
  placePlayers();
  // A tank's two bodies rest together, or each would keep waking the other.
  for (const Tank& t : tanks) {
    sim.objs[t.hullObj].partner = t.turretObj;
    sim.objs[t.turretObj].partner = t.hullObj;
  }
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
enum SoldierPart { SP_PELVIS, SP_TORSO, SP_HEAD, SP_THIGH_L, SP_THIGH_R, SP_SHIN_L, SP_SHIN_R, SP_UPPER_L, SP_UPPER_R, SP_FORE_L, SP_FORE_R, SP_GUN0, SP_GHILLIE = SP_GUN0 + 6 };
constexpr double kHip = 0.93, kKnee = 0.50, kShoulder = 1.42, kElbow = 1.13, kWrist = 0.87, kHipY = 0.10, kShoulderY = 0.215;
// The middle of each part, in the bind pose: x forward, y left, z up from the ground.
const V3 kPartMid[Duel::kSoldierParts] = {
    {0, 0, 0.93}, {0, 0, 1.25}, {0, 0, 1.62}, {0, kHipY, 0.72}, {0, -kHipY, 0.72}, {0, kHipY, 0.27}, {0, -kHipY, 0.27},
    {0, kShoulderY, 1.28}, {0, -kShoulderY, 1.28}, {0, kShoulderY, 1.0}, {0, -kShoulderY, 1.0},
    {0, -kShoulderY, 0.83}, {0, -kShoulderY, 0.83}, {0, -kShoulderY, 0.83}, {0, -kShoulderY, 0.83}, {0, -kShoulderY, 0.83}, {0, -kShoulderY, 0.83},
    {0, 0, 1.25}, {0, 0, 1.62}, {0, kHipY, 0.72}, {0, -kHipY, 0.72},  // the ghillie suit, over torso, head and thighs
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
  static const char* names[kSoldierParts] = {"pelvis", "torso", "head", "thigh_l", "thigh_r", "shin_l", "shin_r", "upper_l", "upper_r", "fore_l", "fore_r", "gun_knife", "gun_pistol", "gun_ar", "gun_sniper", "gun_rpg", "gun_none", "gh_torso", "gh_head", "gh_thigh_l", "gh_thigh_r"};
  return names[part];
}

namespace {
// Where every part of the body is, in its own frame: from the feet, forward along x, left along y.
struct BodyLimbs {
  Limb pelvis, torso, head, thighL, thighR, shinL, shinR, upperL, upperR, foreL, foreR, gun;
  double drop = 0, look = 0;
  V3 at, ahead, left;
  V3 world(V3 q) const {
    q.z -= drop;
    return at + ahead * q.x + left * q.y + V3{0, 0, q.z};
  }
};
BodyLimbs bodyPose(const Soldier& s, int weapon) {
  BodyLimbs out;
  const double moving = std::min(std::hypot(s.v.x, s.v.y) / 2.0, 1.0);
  const double run = std::min(std::hypot(s.v.x, s.v.y) / 8.5, 1.0);
  const double phase = s.stride * 2.1;
  const double swingAmp = (0.45 + 0.35 * run) * moving;
  const double aL = std::sin(phase) * swingAmp, aR = -std::sin(phase) * swingAmp;
  // The knee bends as the leg comes through, more at a run.
  const double kneeL = std::max(0.0, -std::cos(phase)) * (0.7 + 0.5 * run) * moving + 0.05, kneeR = std::max(0.0, std::cos(phase)) * (0.7 + 0.5 * run) * moving + 0.05;
  out.drop = 0.02 * moving * std::fabs(std::sin(phase));
  // Where it looks: positive upward. The body follows a little, the head and the weapon all the way.
  const double look = clampd(-s.pitch, -0.9, 0.9);
  const bool longGun = weapon >= 2;
  const double lean = look * 0.25 - 0.08 * run - (longGun ? 0.05 : 0.0);
  // The arms hold what is carried up in front: a long gun with both hands, a short one further out.
  const double raiseR = weapon == 0 ? 0.25 + aL * 0.5 : (longGun ? 0.55 : 1.15) + look;
  const double bendR = weapon == 0 ? 0.5 : (longGun ? 1.15 : 0.35);
  const double raiseL = weapon == 0 ? aR * 0.8 : 1.0 + look;
  const double bendL = weapon == 0 ? 0.3 : (longGun ? 0.55 : 0.6);

  // Getting down goes in stages: down on the knees, forward onto the hands, then stretched out flat on the belly,
  // up on the elbows behind the bipod. Every joint goes from one of these poses to the next; getting up is the reverse.
  struct Pose {
    double hipX, hipZ, torso, thigh, knee, upperR, foreR, upperL, foreL;
  };
  const Pose stand{0, kHip, lean, 0, 0, raiseR, raiseR + bendR, raiseL, raiseL + bendL};
  static constexpr Pose kKneel{0.05, 0.44, -0.45, 1.25, 2.35, 0.70, 1.80, 0.90, 1.70};
  static constexpr Pose kHands{0.45, 0.34, -1.05, 0.20, 1.10, 1.00, 1.15, 0.60, 1.15};
  static constexpr Pose kFlat{0.88, 0.20, -1.30, -1.50, 0.12, 1.05, 1.50, 0.95, 1.45};
  const auto between = [&](double from, double kneel, double hands, double flat) {
    const double p = s.prone;
    if (p <= 0) return from;
    if (p < 0.35) return lerpd(from, kneel, smooth01(p / 0.35));
    if (p < 0.7) return lerpd(kneel, hands, smooth01((p - 0.35) / 0.35));
    return lerpd(hands, flat, smooth01((p - 0.7) / 0.3));
  };
#define POSE(field, from) between(from, kKneel.field, kHands.field, kFlat.field)
  const V3 hipAt{POSE(hipX, stand.hipX), 0, POSE(hipZ, stand.hipZ)};
  const double torsoAngle = POSE(torso, stand.torso);
  const double lookFlat = s.prone * look;  // lying, the arms still follow the rifle up and down
  const double aThighL = POSE(thigh, aL), aThighR = POSE(thigh, aR);
  const double aShinL = aThighL - POSE(knee, kneeL), aShinR = aThighR - POSE(knee, kneeR);
  const double aUpperR = POSE(upperR, stand.upperR) + lookFlat, aForeR = POSE(foreR, stand.foreR) + lookFlat;
  const double aUpperL = POSE(upperL, stand.upperL) + lookFlat, aForeL = POSE(foreL, stand.foreL) + lookFlat;
#undef POSE
  const V3 pelvisJoint{0, 0, kHip};
  out.pelvis = Limb{pelvisJoint, hipAt, torsoAngle * 0.8};
  out.torso = Limb{pelvisJoint, hipAt, torsoAngle};
  out.head = Limb{V3{0, 0, 1.5}, out.torso.at(V3{0, 0, 1.5}), look};
  const V3 hipL{0, kHipY, kHip}, hipR{0, -kHipY, kHip}, kneeJL{0, kHipY, kKnee}, kneeJR{0, -kHipY, kKnee};
  out.thighL = Limb{hipL, out.pelvis.at(hipL), aThighL};
  out.thighR = Limb{hipR, out.pelvis.at(hipR), aThighR};
  out.shinL = Limb{kneeJL, out.thighL.at(kneeJL), aShinL};
  out.shinR = Limb{kneeJR, out.thighR.at(kneeJR), aShinR};
  const V3 shoulderL{0, kShoulderY, kShoulder}, shoulderR{0, -kShoulderY, kShoulder};
  const double acrossL = weapon == 0 ? 0.0 : (longGun ? 0.9 : 0.75), acrossR = weapon == 0 ? 0.0 : 0.35;
  out.upperL = Limb{shoulderL, out.torso.at(shoulderL), aUpperL, acrossL};
  out.upperR = Limb{shoulderR, out.torso.at(shoulderR), aUpperR, acrossR};
  const V3 elbowL{0, kShoulderY, kElbow}, elbowR{0, -kShoulderY, kElbow};
  out.foreL = Limb{elbowL, out.upperL.at(elbowL), aForeL, acrossL};
  out.foreR = Limb{elbowR, out.upperR.at(elbowR), aForeR, acrossR};
  const V3 grip{0, -kShoulderY, kWrist - 0.04};
  out.gun = Limb{grip, out.foreR.at(grip), weapon == 0 ? out.foreR.angle - 1.2 : look};
  out.look = look;

  const double cy = std::cos(s.yaw), sy = std::sin(s.yaw);
  out.at = s.p;
  out.ahead = V3{-cy, -sy, 0};
  out.left = V3{-out.ahead.y, out.ahead.x, 0};
  return out;
}
V3 partCorner(int part, int k) {
  return kPartMid[part] + V3{(k & 1) ? 0.06 : -0.06, (k & 2) ? 0.06 : -0.06, (k & 4) ? 0.06 : -0.06};
}
}  // namespace

void Duel::poseSoldier(Sim& sim, u32 first, const Soldier& s, bool hiddenIn, int weapon, bool sightsOnly, bool ghillie) {
  const bool hidden = hiddenIn || sightsOnly;
  const BodyLimbs L = bodyPose(s, weapon);
  const auto place = [&](int part, const Limb* limb) {
    for (int k = 0; k < kPartNodes; k++) {
      const u32 n = first + u32(part * kPartNodes + k);
      sim.pos[n] = hidden || !limb ? kSoldierRest + partCorner(part, k) : L.world(limb->at(partCorner(part, k)));
    }
  };
  place(SP_PELVIS, &L.pelvis);
  place(SP_TORSO, &L.torso);
  place(SP_HEAD, &L.head);
  place(SP_THIGH_L, &L.thighL);
  place(SP_THIGH_R, &L.thighR);
  place(SP_SHIN_L, &L.shinL);
  place(SP_SHIN_R, &L.shinR);
  place(SP_UPPER_L, &L.upperL);
  place(SP_UPPER_R, &L.upperR);
  place(SP_FORE_L, &L.foreL);
  place(SP_FORE_R, &L.foreR);
  place(SP_GHILLIE, ghillie ? &L.torso : nullptr);
  place(SP_GHILLIE + 1, ghillie ? &L.head : nullptr);
  place(SP_GHILLIE + 2, ghillie ? &L.thighL : nullptr);
  place(SP_GHILLIE + 3, ghillie ? &L.thighR : nullptr);
  for (int g = 0; g < SP_GHILLIE - SP_GUN0; g++) {
    const int part = SP_GUN0 + g;
    if (sightsOnly && g == weapon && !hiddenIn) {
      // Held out under the eye, a little to the right, pointing where it looks.
      const V3 grip{0, -kShoulderY, kWrist - 0.04};
      const Limb held{grip, V3{0.62, -0.17, 1.45 - 0.2}, L.look};
      for (int k = 0; k < kPartNodes; k++) sim.pos[first + u32(part * kPartNodes + k)] = L.world(held.at(partCorner(part, k)));
      continue;
    }
    place(part, g == weapon ? &L.gun : nullptr);
  }
}

// ---- A dead body: points joined by sticks, dropped from the pose it was in and pushed by whatever killed it.
namespace {
enum Joint { J_PELVIS, J_NECK, J_HEAD, J_HIP_L, J_HIP_R, J_KNEE_L, J_KNEE_R, J_FOOT_L, J_FOOT_R, J_SHOULDER_L, J_SHOULDER_R, J_ELBOW_L, J_ELBOW_R, J_HAND_L, J_HAND_R };
// The same joints standing, arms down, as the mesh was modelled.
const V3 kJointRest[Ragdoll::kJoints] = {
    {0, 0, kHip}, {0, 0, 1.5}, {0, 0, 1.72}, {0, kHipY, kHip}, {0, -kHipY, kHip}, {0, kHipY, kKnee}, {0, -kHipY, kKnee}, {0, kHipY, 0.06}, {0, -kHipY, 0.06},
    {0, kShoulderY, kShoulder}, {0, -kShoulderY, kShoulder}, {0, kShoulderY, kElbow}, {0, -kShoulderY, kElbow}, {0, kShoulderY, kWrist - 0.06}, {0, -kShoulderY, kWrist - 0.06},
};
struct Stick {
  int a, b;
};
const Stick kSticks[] = {
    // The trunk holds its shape, with the head on it.
    {J_PELVIS, J_NECK}, {J_HIP_L, J_HIP_R}, {J_HIP_L, J_NECK}, {J_HIP_R, J_NECK}, {J_PELVIS, J_HIP_L}, {J_PELVIS, J_HIP_R}, {J_SHOULDER_L, J_SHOULDER_R},
    {J_SHOULDER_L, J_NECK}, {J_SHOULDER_R, J_NECK}, {J_SHOULDER_L, J_PELVIS}, {J_SHOULDER_R, J_PELVIS}, {J_SHOULDER_L, J_HIP_L}, {J_SHOULDER_R, J_HIP_R}, {J_SHOULDER_L, J_HIP_R}, {J_SHOULDER_R, J_HIP_L},
    {J_HEAD, J_NECK}, {J_HEAD, J_SHOULDER_L}, {J_HEAD, J_SHOULDER_R},
    // Arms and legs swing as they like.
    {J_HIP_L, J_KNEE_L}, {J_KNEE_L, J_FOOT_L}, {J_HIP_R, J_KNEE_R}, {J_KNEE_R, J_FOOT_R},
    {J_SHOULDER_L, J_ELBOW_L}, {J_ELBOW_L, J_HAND_L}, {J_SHOULDER_R, J_ELBOW_R}, {J_ELBOW_R, J_HAND_R},
};
// How a part follows two joints and a sideways line: where it was in the bind pose, in the frame they make now.
struct Bone {
  V3 restA, x0, y0, z0, a, x1, y1, z1;
  V3 at(V3 q) const {
    const V3 d = q - restA;
    return a + x1 * dot(d, x0) + y1 * dot(d, y0) + z1 * dot(d, z0);
  }
};
Bone makeBone(V3 restA, V3 restB, V3 restSide, V3 a, V3 b, V3 side) {
  const auto frame = [](V3 along, V3 sideways, V3& x, V3& y, V3& z) {
    z = norm(along);
    y = sideways - z * dot(sideways, z);
    y = len2(y) > 1e-8 ? norm(y) : norm(cross(z, std::fabs(z.z) < 0.9 ? V3{0, 0, 1} : V3{1, 0, 0}));
    x = cross(y, z);
  };
  Bone out;
  out.restA = restA;
  out.a = a;
  frame(restB - restA, restSide, out.x0, out.y0, out.z0);
  frame(b - a, side, out.x1, out.y1, out.z1);
  return out;
}
}  // namespace

void Duel::ragdollStart(Ragdoll& r, const Soldier& s, int weapon, V3 push) {
  const BodyLimbs L = bodyPose(s, weapon);
  const auto w = [&](const Limb& limb, int j) { return L.world(limb.at(kJointRest[j])); };
  r.p[J_PELVIS] = L.world(L.pelvis.at(kJointRest[J_PELVIS]));
  r.p[J_NECK] = w(L.torso, J_NECK);
  r.p[J_HEAD] = w(L.head, J_HEAD);
  r.p[J_HIP_L] = w(L.pelvis, J_HIP_L);
  r.p[J_HIP_R] = w(L.pelvis, J_HIP_R);
  r.p[J_KNEE_L] = w(L.thighL, J_KNEE_L);
  r.p[J_KNEE_R] = w(L.thighR, J_KNEE_R);
  r.p[J_FOOT_L] = w(L.shinL, J_FOOT_L);
  r.p[J_FOOT_R] = w(L.shinR, J_FOOT_R);
  r.p[J_SHOULDER_L] = w(L.torso, J_SHOULDER_L);
  r.p[J_SHOULDER_R] = w(L.torso, J_SHOULDER_R);
  r.p[J_ELBOW_L] = w(L.upperL, J_ELBOW_L);
  r.p[J_ELBOW_R] = w(L.upperR, J_ELBOW_R);
  r.p[J_HAND_L] = w(L.foreL, J_HAND_L);
  r.p[J_HAND_R] = w(L.foreR, J_HAND_R);
  // The blow lands high up, and the feet stay behind: that is what throws someone over.
  constexpr double kStep = 1.0 / 120.0;
  for (int j = 0; j < Ragdoll::kJoints; j++) {
    const double height = clampd((r.p[j].z - s.p.z) / 1.6, 0.0, 1.0);
    const V3 v = s.v + push * (0.45 + 0.75 * height);
    r.prev[j] = r.p[j] - v * kStep;
  }
  r.on = true;
  r.clock = 0;
  r.debt = 0;
  r.weapon = weapon;
  r.bloodAt = (r.p[J_PELVIS]);
}

void Duel::ragdollStep(Ragdoll& r, double dt) const {
  constexpr double kStep = 1.0 / 120.0;
  r.clock += dt;
  r.debt = std::min(r.debt + dt, 0.1);
  while (r.debt >= kStep) {
    r.debt -= kStep;
    for (int j = 0; j < Ragdoll::kJoints; j++) {
      const V3 v = (r.p[j] - r.prev[j]) * 0.997;
      r.prev[j] = r.p[j];
      r.p[j] += v + V3{0, 0, -9.81 * kStep * kStep};
    }
    for (int it = 0; it < 8; it++) {
      for (const Stick& st : kSticks) {
        const double want = len(kJointRest[st.a] - kJointRest[st.b]);
        V3 d = r.p[st.b] - r.p[st.a];
        const double l = std::max(len(d), 1e-6);
        const V3 fix = d * (0.5 * (l - want) / l);
        r.p[st.a] += fix;
        r.p[st.b] -= fix;
      }
      // Knees and elbows only go so far: a foot does not come back up through the hip.
      const auto keepApart = [&](int a, int b, double most) {
        V3 d = r.p[b] - r.p[a];
        const double l = std::max(len(d), 1e-6);
        if (l >= most) return;
        const V3 fix = d * (0.5 * (l - most) / l);
        r.p[a] += fix;
        r.p[b] -= fix;
      };
      keepApart(J_HIP_L, J_FOOT_L, 0.55);
      keepApart(J_HIP_R, J_FOOT_R, 0.55);
      keepApart(J_SHOULDER_L, J_HAND_L, 0.32);
      keepApart(J_SHOULDER_R, J_HAND_R, 0.32);
      for (int j = 0; j < Ragdoll::kJoints; j++) {
        // The ground, and whatever stands on it.
        const double rad = j == J_HEAD || j == J_PELVIS ? 0.12 : 0.07;
        Contact c;
        const V3 under = r.p[j] - V3{0, 0, rad};
        if (world.contact(under, c)) {
          r.p[j] += c.n * c.depth;
          // It drags, and does not slide on.
          const V3 moved = r.p[j] - r.prev[j];
          const V3 along = moved - c.n * dot(moved, c.n);
          r.prev[j] += along * 0.35;
        }
      }
    }
  }
}

void Duel::poseRagdoll(Sim& sim, u32 first, const Ragdoll& r, bool ghillie) {
  const V3 shoulders = r.p[J_SHOULDER_L] - r.p[J_SHOULDER_R], hips = r.p[J_HIP_L] - r.p[J_HIP_R];
  const V3 restSide{0, 1, 0};
  const auto bone = [&](int a, int b, V3 side) { return makeBone(kJointRest[a], kJointRest[b], restSide, r.p[a], r.p[b], side); };
  const Bone bones[11] = {
      bone(J_PELVIS, J_NECK, hips), bone(J_PELVIS, J_NECK, shoulders), bone(J_NECK, J_HEAD, shoulders),
      bone(J_HIP_L, J_KNEE_L, hips), bone(J_HIP_R, J_KNEE_R, hips), bone(J_KNEE_L, J_FOOT_L, hips), bone(J_KNEE_R, J_FOOT_R, hips),
      bone(J_SHOULDER_L, J_ELBOW_L, shoulders), bone(J_SHOULDER_R, J_ELBOW_R, shoulders), bone(J_ELBOW_L, J_HAND_L, shoulders), bone(J_ELBOW_R, J_HAND_R, shoulders),
  };
  for (int part = 0; part < kSoldierParts; part++) {
    const bool body = part < SP_GUN0;
    for (int k = 0; k < kPartNodes; k++) {
      const u32 n = first + u32(part * kPartNodes + k);
      // Nothing to hold it any more: what was in the hand is put away.
      static constexpr int kGhillieBone[4] = {1, 2, 3, 4};
      if (part >= SP_GHILLIE && ghillie) sim.pos[n] = bones[kGhillieBone[part - SP_GHILLIE]].at(partCorner(part, k));
      else sim.pos[n] = body ? bones[part].at(partCorner(part, k)) : kSoldierRest + partCorner(part, k);
    }
  }
}

void Duel::poseDrone(Sim& sim, u32 first, bool live, V3 at, double yaw) {
  if (!live) at = V3{0, 0, -385.0};
  const double c = std::cos(yaw), s = std::sin(yaw);
  constexpr double kArm = 0.42;
  // The body, and four rotors out on their arms.
  const V3 nodes[kDroneNodes] = {
      at + V3{0, 0, -0.06}, at + V3{0, 0, 0.06},
      at + V3{(c - s) * kArm, (s + c) * kArm, 0}, at + V3{(-c + s) * kArm, (-s - c) * kArm, 0},
      at + V3{(c + s) * kArm, (s - c) * kArm, 0}, at + V3{(-c - s) * kArm, (-s + c) * kArm, 0},
  };
  for (int k = 0; k < kDroneNodes; k++) sim.pos[first + u32(k)] = nodes[k];
}

void Duel::stepDrone(int p, const Intent& in, double dt) {
  Player& pl = players[p];
  if (in.drone) {
    if (pl.droneLive) {
      pl.droneLive = false;  // back to the hand, without being any use again
    } else if (pl.drones > 0 && pl.vehicle < 0 && phase == PLAYING) {
      pl.drones--;
      pl.droneLive = true;
      pl.droneBattery = kDroneBattery;
      pl.droneP = pl.soldier.p + V3{0, 0, 2.2};
      pl.droneV = V3{0, 0, 3.0};
    }
  }
  if (!pl.droneLive) return;
  pl.droneBattery -= dt;
  // Flown like a camera: it goes where one looks, and sideways as told.
  const double cy = std::cos(in.walk.yaw), sy = std::sin(in.walk.yaw), cp = std::cos(in.pitch), sp = std::sin(in.pitch);
  const V3 ahead{-cy * cp, -sy * cp, -sp}, side{sy, -cy, 0};
  const V3 want = (ahead * in.walk.forward + side * in.walk.strafe) * (in.walk.sprint ? kDroneSpeed : kDroneSpeed * 0.55);
  pl.droneV += (want - pl.droneV) * clampd(dt * 3.0, 0, 1);
  pl.droneP += pl.droneV * dt;
  const double ground = world.height(pl.droneP.x, pl.droneP.y);
  // It keeps above the ground by itself, and stays under the sky it can reach.
  pl.droneP.z = clampd(pl.droneP.z, ground + 1.5, ground + 170.0);
  Contact c;
  if (pl.droneBattery <= 0 || world.contact(pl.droneP, c)) {
    pl.droneLive = false;
    sim.event(EV_SPARK, pl.droneP, V3{0, 0, 1}, 1.f);
    return;
  }
  // Firing marks the other player, if the crosshair is near enough to them.
  if (in.fireFoot) {
    for (int e = 0; e < kMaxPlayers; e++) {
      const Player& other = players[e];
      if (!other.present || teamOf(e) == teamOf(p) || other.soldier.hp <= 0) continue;
      const V3 where = other.vehicle >= 0 ? frames[other.vehicle].cog : other.soldier.p + V3{0, 0, 1.0};
      const V3 to = where - pl.droneP;
      const double d = len(to);
      if (d < kDroneMarkRange && dot(norm(to), ahead) > 0.97) {
        pl.fix = where;
        pl.hasFix = true;
        pl.fixAge = 0;
        noteHit(-1, p, where);
        break;
      }
    }
  }
}

void Duel::poseNet(Sim& sim, u32 first, const Tank& t, bool on) {
  const V3 rest0 = (sim.rest[t.refFL] + sim.rest[t.refFR] + sim.rest[t.refRL] + sim.rest[t.refRR]) * 0.25;
  const V3 now = (sim.pos[t.refFL] + sim.pos[t.refFR] + sim.pos[t.refRL] + sim.pos[t.refRR]) * 0.25;
  for (u32 n = 0; n < 8; n++) {
    const V3 r = sim.rest[first + n] - rest0;
    sim.pos[first + n] = on ? now + t.fwd * r.x + t.left * r.y + t.up * r.z : sim.rest[first + n] + V3{0, 0, -400};
  }
}

void Duel::poseMissile(Sim& sim, u32 first, bool live, V3 at, V3 dir, double scale) {
  // Tail, the body's end, the tip, and two pairs of fins across the tail. Always the same shape, underground when idle.
  if (!live) {
    at = V3{0, 0, -395.0};
    dir = V3{1, 0, 0};
  }
  dir = norm(dir);
  V3 u = cross(dir, std::fabs(dir.z) < 0.9 ? V3{0, 0, 1} : V3{1, 0, 0});
  u = norm(u);
  const V3 v = cross(dir, u);
  const V3 tail = at - dir * (1.1 * scale);
  const double fin = 0.42 * scale;
  const V3 nodes[kMissileNodes] = {tail, at + dir * (0.75 * scale), at + dir * (1.15 * scale), tail + u * fin, tail - u * fin, tail + v * fin, tail - v * fin};
  for (int k = 0; k < kMissileNodes; k++) sim.pos[first + u32(k)] = nodes[k];
}

void Duel::walk(Soldier& s, const WalkInput& in, double dt) const {
  // Forward is away from the camera, which looks along -(cos yaw, sin yaw).
  const double cy = std::cos(in.yaw), sy = std::sin(in.yaw);
  const V3 ahead{-cy, -sy, 0}, side{sy, -cy, 0};
  V3 want = ahead * in.forward + side * in.strafe;
  const double wl = len(want);
  if (wl > 1.0) want = want / wl;
  // Wading is hard going.
  const double top = (in.sprint ? 8.5 : 5.0) * (world.isWater(s.p.x, s.p.y, s.p.z) ? 0.45 : 1.0);
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
  const double from = f.cog.z + 1.0;
  for (double sg : {1.0, -1.0}) {
    V3 p = f.cog + f.left * (sg * out);
    p.z = world.groundUnder(p.x, p.y, from) + 0.05;
    // Not over the railing and down into the river.
    if (p.z < f.cog.z - 4.0) continue;
    Contact c;
    if (!world.contact(p + V3{0, 0, 1.0}, c)) return p;
  }
  V3 p = f.cog - f.fwd * (f.halfLength + 1.0);
  p.z = world.groundUnder(p.x, p.y, from) + 0.05;
  return p;
}

int Duel::nearestVehicle(V3 p, double reach) const {
  int best = -1;
  double bestD = 1e30;
  for (int v = 0; v < kVehicles; v++) {
    if (vehicleDead(v) || full(v)) continue;
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
      {"panzermine", 4, 1}, {"bombencode", 4, 1}, {"medikit", 6, 1}, {"nebelgranate", 7, 2}, {"ghillie", 4, 1}, {"tarnnetz", 4, 1}, {"drohne", 5, 1},
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
  static const char* kLabels[kGiveKinds] = {"Tretmine", "Panzermine", "Reparaturset", "Sturmgewehr", "Kleinmunition", "Scharfschützengewehr", "Scharfschützenmunition", "Raketenwerfer", "Raketen", "Panzergranaten", "Lenkraketen", "Bombencode", "Medikit", "Nebelgranaten", "Ghillie-Anzug", "Tarnnetz", "Drohne"};
  snprintf(buf, n, c.count > 1 ? "%s  x%d" : "%s", kLabels[c.what], c.count);
  return buf;
}

const char* const Duel::kGiveNames[kGiveKinds] = {
    "tretmine", "panzermine", "reparaturset", "sturmgewehr", "kleinmunition", "scharfschuetzengewehr", "scharfschuetzenmunition",
    "raketenwerfer", "raketen", "granaten", "lenkraketen", "bombencode", "medikit", "nebelgranate", "ghillie", "tarnnetz", "drohne",
};

bool Duel::give(int player, const char* name, int count) {
  if (player < 0 || player >= kMaxPlayers || count < 1) return false;
  Player& p = players[player];
  const std::string n = name;
  const auto addItem = [&](ItemKind k) { p.items[k] = u8(std::min(int(p.items[k]) + count, 99)); };
  if (n == "drohne") p.drones += count;
  else if (n == "nebelgranate") addItem(IT_SMOKE);
  else if (n == "ghillie") addItem(IT_GHILLIE);
  else if (n == "tarnnetz") addItem(IT_NET);
  else if (n == "tretmine") addItem(IT_FOOT_MINE);
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
  else if (n == "medikit") p.medkits += count;
  else return false;
  return true;
}

V3 Duel::eyeOf(const Soldier& s) {
  // Standing, or stretched out on the ground with the head towards the enemy.
  const double cy = std::cos(s.yaw), sy = std::sin(s.yaw);
  return s.p + V3{-cy, -sy, 0} * (1.5 * s.prone) + V3{0, 0, lerpd(1.45, 0.42, s.prone)};
}

V3 Duel::soldierAt(int p, double lag) const {
  const int back = std::clamp(int(lag * 100.0), 0, kPastSteps - 1);
  return past[(pastAt - back + kPastSteps * 2) % kPastSteps].soldier[p];
}

int Duel::enemyNearest(int p, V3 from) const {
  int best = -1;
  double bestD = 1e30;
  for (int e = 0; e < kMaxPlayers; e++) {
    if (!players[e].present || teamOf(e) == teamOf(p) || players[e].soldier.hp <= 0) continue;
    const double d = len2(players[e].soldier.p - from);
    if (d < bestD) {
      bestD = d;
      best = e;
    }
  }
  return best;
}

V3 Duel::aimEnd(const World& world, const std::vector<V3>& pos, const std::vector<Block>& blocks, const VehicleFrame* frames, int vehicles, int skipVehicle,
                 const std::vector<AimFoe>& foes, V3 from, V3 dir, double clear) {
  constexpr double kFar = 1800.0;
  double end = kFar;
  for (int v = 0; v < vehicles; v++) {
    if (v == skipVehicle) continue;
    const VehicleFrame& f = frames[v];
    const V3 q = from - f.cog;
    const V3 ax[3] = {f.fwd, f.left, f.up};
    const double half[3] = {f.halfLength, f.halfWidth, f.height};
    double in = -1e30, out = 1e30;
    for (int k = 0; k < 3; k++) {
      const double o = dot(q, ax[k]), d = dot(dir, ax[k]);
      if (std::fabs(d) < 1e-9) {
        if (std::fabs(o) > half[k]) in = 1e30;
        continue;
      }
      const double a = (-half[k] - o) / d, b = (half[k] - o) / d;
      in = std::max(in, std::min(a, b));
      out = std::min(out, std::max(a, b));
    }
    if (in > out || out < clear) continue;
    end = std::min(end, std::max(in, clear));
  }
  for (const AimFoe& foe : foes) {
    const double h2 = dir.x * dir.x + dir.y * dir.y;
    if (h2 < 1e-9) continue;
    const double t = ((foe.p.x - from.x) * dir.x + (foe.p.y - from.y) * dir.y) / h2;
    if (t < clear || t > end) continue;
    const V3 at = from + dir * t;
    if (std::hypot(at.x - foe.p.x, at.y - foe.p.y) < 0.38 && at.z > foe.p.z && at.z < foe.p.z + foe.height) end = t;
  }
  // The stones near the line, once, so that going along it asks only those.
  std::vector<const Block*> near;
  for (const Block& b : blocks) {
    if (size_t(b.n[0]) >= pos.size()) continue;
    const V3 a = pos[b.n[0]] - from;
    const double t = dot(a, dir);
    if (t > -6.0 && t < end + 6.0 && len2(a - dir * t) < 36.0) near.push_back(&b);
  }
  // Finely close by, where it matters; further off a bullet is not going to notice.
  double d = clear;
  while (d < end) {
    const V3 p = from + dir * d;
    Contact c;
    if (world.contact(p, c)) return p;
    for (const Block* b : near) {
      const V3 a = pos[b->n[0]];
      const V3 ex = pos[b->n[1]] - a, ey = pos[b->n[2]] - a, ez = pos[b->n[4]] - a, q = p - a;
      const double lx = dot(q, ex) / len2(ex), ly = dot(q, ey) / len2(ey), lz = dot(q, ez) / len2(ez);
      if (lx >= 0 && lx <= 1 && ly >= 0 && ly <= 1 && lz >= 0 && lz <= 1) return p;
    }
    d += d < 100.0 ? 0.1 : (d < 400.0 ? 1.0 : 3.0);
  }
  return from + dir * end;
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
    const int foe = enemyNearest(p, s.p);
    const V3 target = foe >= 0 ? soldierAt(foe, in.lag) : s.p + V3{0, 0, 999};
    if (foe >= 0 && players[foe].vehicle < 0 && len(V3{target.x - s.p.x, target.y - s.p.y, target.z - s.p.z}) < 2.2) {
      Soldier& other = players[foe].soldier;
      other.hp -= kWeapons[w].damage;
      players[foe].lastBlow = norm(target - s.p + V3{0, 0, 0.3}) * 2.0;
      noteHit(foe, p, s.p);
      sim.event(EV_BLOOD, target + V3{0, 0, 1.1}, norm(target - s.p) * 2.0, 0.7f);
    }
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
  const double spread = kWeapons[w].spread * (in.aiming ? 0.35 : 1.0) * (1.0 + (w == WP_SNIPER ? 0.0 : std::min(len(s.v) * 0.5, 2.0)));
  dir = norm(dir + V3{chance(), chance(), chance()} * spread);
  Bullet b;
  b.p = eye + dir * 0.7;
  b.v = dir * kWeapons[w].speed;
  b.owner = p;
  b.kind = w;
  b.lag = in.lag;
  bullets.push_back(b);
  sim.event(EV_MUZZLE, b.p, dir, w == WP_RPG ? 0.5f : (w == WP_SNIPER ? 1.0f : (w == WP_AR ? 0.14f : 0.1f)));
  if (w == WP_SNIPER) {
    // The blast from the muzzle brake raises the dust around it.
    for (int k = 0; k < 6; k++) sim.event(EV_DUST, eye + dir * 1.0 + V3{chance() * 1.2, chance() * 1.2, -0.3}, V3{chance() * 3.0, chance() * 3.0, 1.0}, 1.f);
  }
}

bool Duel::bulletInCar(int k, V3 from, V3 dir, int kind) {
  Vehicle& car = cars[size_t(k)];
  const VehicleFrame& f = frames[kTanks + k];
  const auto middle = [&](const std::vector<u32>& nodes, V3 fallback) {
    if (nodes.empty()) return fallback;
    V3 sum{};
    for (u32 n : nodes) sum += sim.pos[n];
    return sum / double(nodes.size());
  };
  enum Part { ENGINE, OILPAN, RADIATOR, TANK, GEARBOX, DRIVER, WHEEL0 };
  struct Spot {
    int part;
    V3 at;
    double radius;
  };
  std::vector<Spot> spots;
  const CarParts& parts = carParts[k];
  const V3 engine = middle(parts.engine, f.cog + f.fwd * 1.3);
  spots.push_back({ENGINE, engine, 0.3});
  spots.push_back({OILPAN, engine - f.up * 0.3, 0.2});                                    // under the engine: it takes a good shot, or a low one
  spots.push_back({RADIATOR, middle(parts.radiator, f.cog + f.fwd * 1.9), 0.32});
  spots.push_back({TANK, middle(parts.tank, f.cog - f.fwd * 1.4 - f.up * 0.25), 0.34});
  spots.push_back({GEARBOX, middle(parts.gearbox, engine + f.left * 0.3), 0.2});
  spots.push_back({DRIVER, f.cog + f.left * 0.36 + f.up * 0.25, 0.28});
  for (size_t w = 0; w < car.wheels.size(); w++) spots.push_back({WHEEL0 + int(w), (sim.pos[car.wheels[w].axIn] + sim.pos[car.wheels[w].axOut]) * 0.5, 0.33});
  // The first thing along its way through the car.
  const Spot* first = nullptr;
  double firstT = 1e30;
  constexpr double kThrough = 5.0;  // right across the car
  for (const Spot& s : spots) {
    const double t = clampd(dot(s.at - from, dir), 0.0, kThrough);
    if (len2(from + dir * t - s.at) < s.radius * s.radius && t < firstT) {
      first = &s;
      firstT = t;
    }
  }
  if (!first) return false;
  const bool heavy = kind == WP_SNIPER;
  // Small arms go for the tyres, and for whoever is sitting there; the big rifle for all of it.
  if (!heavy && first->part != DRIVER && first->part < WHEEL0) return false;
  sim.wake(car.obj);
  sim.event(EV_SPARK, first->at, dir * -2.0, 1.f);
  switch (first->part) {
    case ENGINE:
      car.dmg.seized = true;
      break;
    case OILPAN:
      car.dmg.oilLeak = true;
      break;
    case RADIATOR:
      car.dmg.radiator = true;
      break;
    case TANK:
      car.dmg.tank = true;
      // Through a full tank a bullet this size leaves a big hole.
      car.fuelLitres = std::max(0.0, car.fuelLitres - 8.0);
      break;
    case GEARBOX:
      car.dmg.gearbox = true;
      break;
    case DRIVER:
      for (Player& pl : players)
        if (pl.vehicle == kTanks + k) {
          pl.soldier.hp -= kWeapons[kind].damage * 0.8;
          sim.event(EV_BLOOD, first->at, dir * 2.0, 0.7f);
          noteHit(int(&pl - players), -1, first->at - dir * 5.0);
        }
      break;
    default:
      if (car.wheels[size_t(first->part - WHEEL0)].deflate == 0) car.wheels[size_t(first->part - WHEEL0)].deflate = 0.001;
      break;
  }
  return true;
}

void Duel::stepBullets(double dt) {
  for (size_t i = 0; i < bullets.size();) {
    Bullet& b = bullets[i];
    b.age += dt;
    if (b.kind == kKindStrike) {
      // Straight down the last of the way.
    } else if (b.kind == kKindSmoke) {
      b.v.z -= 9.81 * dt;
      Contact c;
      if (world.contact(b.p + b.v * dt, c)) {
        startSmoke(b.p);
        bullets[i] = bullets.back();
        bullets.pop_back();
        continue;
      }
      b.p += b.v * dt;
      i++;
      continue;
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
    if (b.kind == WP_SNIPER) sim.event(EV_TRACER, b.p, b.v, 2.f);  // bright enough to follow all the way, every few metres
    else if (((b.kind == WP_RPG || b.kind == kKindStrike) && int(b.age * 200.0) % 4 == 0) || (b.kind != WP_RPG && int(b.age * 200.0) % 8 == 0)) sim.event(EV_TRACER, b.p, b.v, 1.f);
    bool inCar = false;
    bool stop = b.age > (b.kind == kKindStrike ? 20.0 : 6.0);
    V3 at = b.p;
    const V3 seg = b.p - from;
    const double L = len(seg);
    const int samples = std::max(1, int(L));
    // A drone is small, and does not take much.
    if (!stop && b.kind != kKindStrike)
      for (Player& pl : players)
        if (pl.droneLive && len2(pl.droneP - (from + seg * clampd(dot(pl.droneP - from, seg) / std::max(len2(seg), 1e-9), 0, 1))) < 0.7 * 0.7) {
          pl.droneLive = false;
          sim.event(EV_SPARK, pl.droneP, V3{0, 0, 1}, 1.f);
          noteHit(-1, b.owner, from);
          stop = true;
          at = pl.droneP;
        }
    // Those it may hit: the other player, where the shooter saw them.
    for (int q = 0; q < kMaxPlayers && !stop; q++) {
      // Not oneself, and in the same uniform not either: nobody is out to shoot their own side in the back.
      if (!players[q].present || teamOf(q) == teamOf(b.owner)) continue;
      if (players[q].vehicle < 0 && b.kind != kKindStrike) {
        const V3 foot = soldierAt(q, b.lag);
        const double t = clampd(dot(V3{foot.x - from.x, foot.y - from.y, 0}, V3{seg.x, seg.y, 0}) / std::max(seg.x * seg.x + seg.y * seg.y, 1e-9), 0, 1);
        V3 near = from + seg * t;
        bool struck = std::hypot(near.x - foot.x, near.y - foot.y) < 0.38 && near.z > foot.z && near.z < foot.z + Soldier::kHeight;
        const Soldier& target = players[q].soldier;
        if (target.prone > 0.5) {
          // Stretched out along the ground, from the feet to the head: low, and long.
          struck = false;
          const V3 along{-std::cos(target.yaw), -std::sin(target.yaw), 0};
          for (double d = 0.15; d < 1.75 && !struck; d += 0.3) {
            const V3 c = foot + along * d + V3{0, 0, 0.22};
            const double ts = clampd(dot(c - from, seg) / std::max(len2(seg), 1e-9), 0, 1);
            if (len2(from + seg * ts - c) < 0.3 * 0.3) {
              struck = true;
              near = from + seg * ts;
            }
          }
        }
        if (struck) {
          stop = true;
          at = near;
          if (b.kind < WP_RPG) {
            // In the head it counts for more than in the boots.
            const double z = near.z - foot.z;
            players[q].soldier.hp -= kWeapons[b.kind].damage * (z > 1.5 ? 2.0 : (z < 0.7 ? 0.7 : 1.0));
            players[q].lastBlow = norm(b.v) * (b.kind == WP_SNIPER ? 11.0 : 2.5);
            noteHit(q, b.owner, from);
            sim.event(EV_BLOOD, near, b.v * 0.02, b.kind == WP_SNIPER ? 1.f : 0.45f);
          }
        }
      }
    }
    // A weak spot on the tank it was aimed at, as far as the tank was when the shooter saw it. Armour does nothing for
    // the small things sticking out of it, and the armour itself does not care.
    if (!stop && b.kind < WP_RPG) {
      const double force = b.kind == WP_SNIPER ? 1.0 : (b.kind == WP_AR ? 0.1 : 0.04);
      const int backSteps = std::clamp(int(b.lag * 100.0), 0, kPastSteps - 1);
      for (int v = 0; v < kTanks && !stop; v++) {
        const V3 moved = frames[v].cog - past[(pastAt - backSteps + kPastSteps * 2) % kPastSteps].tank[v];
        if (len2(b.p - frames[v].cog + moved * -1.0) > 15.0 * 15.0 && len2(from - frames[v].cog) > 15.0 * 15.0) continue;
        if (tanks[v].bulletHit(sim, from + moved, b.p + moved, force)) {
          stop = true;
          at = b.p;
          noteHit(-1, b.owner, from);
          for (int who = 0; who < kMaxPlayers; who++)
            if (players[who].vehicle == v) noteHit(who, -1, from);
        }
      }
    }
    // Vehicles stop it.
    if (!stop)
      for (int v = 0; v < kVehicles && !stop; v++) {
        const VehicleFrame& f = frames[v];
        if (len2(from - f.cog) > (L + 8.0) * (L + 8.0)) continue;
        // Half a metre at a time: at this speed a step is longer than a car is wide.
        const int checks = std::max(1, int(L / 0.5));
        for (int c = 1; c <= checks && !stop; c++) {
          const V3 p = from + seg * (double(c) / checks);
          const V3 q = p - f.cog;
          if (std::fabs(dot(q, f.fwd)) >= f.halfLength || std::fabs(dot(q, f.left)) >= f.halfWidth || std::fabs(dot(q, f.up)) >= f.height * 0.5) continue;
          stop = true;
          at = p;
          if (v >= kTanks && !vehicleDead(v) && b.kind < WP_RPG) {
            inCar = true;
            bulletInCar(v - kTanks, p - norm(seg) * 0.5, norm(seg), b.kind);
          }
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
      else if (b.kind == WP_SNIPER && b.age < 6.0) {
        // A shell of its own, near enough: a hole in the ground, stone gone, a dent in whatever it hits, and whoever
        // stands right beside it feels it too. The dust goes up high, for the shooter to see where it went.
        // In a car it is the part it hit that suffers, not the whole thing.
        if (!inCar) sim.explode(at, 650.0, 4.0, 2.4);
        sim.event(EV_IMPACT, at, V3{double(b.owner), 0, 0}, 1.f);
      }
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

void Duel::noteHit(int victim, int attacker, V3 from) {
  if (victim >= 0) {
    players[victim].hitsTaken++;
    players[victim].hurtFrom = from;
  }
  if (attacker >= 0 && attacker != victim) players[attacker].hitsLanded++;
}

void Duel::startSmoke(V3 at) {
  if (smoke.size() >= 12) smoke.erase(smoke.begin());
  SmokeCloud c;
  c.p = at;
  c.left = kSmokeTime;
  smoke.push_back(c);
}

void Duel::useSmoke(int p) {
  Player& pl = players[p];
  if (pl.vehicle >= 0 && pl.vehicle < kTanks) {
    // Four clouds in a fan around the front of the turret, a stone's throw off, between the tank and whoever it faces.
    Tank& t = tanks[pl.vehicle];
    if (t.smokeCharges <= 0) return;
    t.smokeCharges--;
    const double cy = std::cos(t.turretYaw), sy = std::sin(t.turretYaw);
    const V3 eF = t.fwd * cy + t.left * sy, eL = t.left * cy - t.fwd * sy;
    for (double a : {-1.0, -0.33, 0.33, 1.0}) {
      V3 at = frames[pl.vehicle].cog + eF * (18.0 * std::cos(a)) + eL * (18.0 * std::sin(a));
      at.z = world.groundUnder(at.x, at.y, frames[pl.vehicle].cog.z + 2.0) + 0.5;
      startSmoke(at);
    }
    sim.event(EV_MUZZLE, frames[pl.vehicle].cog + t.up * 1.5, t.up, 0.15f);
    return;
  }
  if (pl.vehicle >= 0 || pl.items[IT_SMOKE] == 0) return;
  // A throw: as far as an arm goes, in the air for a moment.
  pl.items[IT_SMOKE]--;
  const Soldier& s = pl.soldier;
  const V3 ahead{-std::cos(s.yaw) * std::cos(s.pitch), -std::sin(s.yaw) * std::cos(s.pitch), -std::sin(s.pitch)};
  Bullet b;
  b.p = eyeOf(s) + ahead * 0.5;
  b.v = (ahead + V3{0, 0, 0.35}) * 19.0 + s.v;
  b.kind = kKindSmoke;
  b.owner = p;
  bullets.push_back(b);
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
    at.z = world.groundUnder(at.x, at.y, pl.soldier.p.z + 0.5) + (kind == 0 ? 0.03 : 0.05);
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
  players[p].seat = seatHolder(v, 0) < 0 ? 0 : 1;
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
  players[p].vehicle = -1;
  // The other one can stay; the gunner moves up to drive.
  if (occupied(v)) {
    const int left = seatHolder(v, 1);
    if (left >= 0) players[left].seat = 0;
    return;
  }
  const u16 a = v < kTanks ? tanks[v].hullObj : cars[size_t(v - kTanks)].obj;
  const u16 b = v < kTanks ? tanks[v].turretObj : a;
  sim.objs[a].alwaysAwake = false;
  sim.objs[b].alwaysAwake = false;
  if (v >= kTanks) cars[size_t(v - kTanks)].engineRunning = false;
}

void Duel::stepPlayers(const Intent in[kMaxPlayers]) {
  // Blasts reach those on foot.
  for (; blastsDone < sim.blastLog.size(); blastsDone++) {
    const Sim::BlastRecord& b = sim.blastLog[blastsDone];
    for (Player& pl : players) {
      if (pl.vehicle >= 0) continue;
      const double d = len(pl.soldier.p + V3{0, 0, 0.9} - b.c);
      const double kill = b.radius * 0.55;
      if (d < kill) {
        const double share = (1.0 - d / kill) * std::min(b.impulse / 1000.0, 1.0);
        pl.soldier.hp -= 140.0 * share;
        noteHit(int(&pl - players), -1, b.c);
        // Thrown away from it, and up.
        pl.lastBlow = norm(pl.soldier.p + V3{0, 0, 0.9} - b.c + V3{0, 0, 0.7}) * clampd(5.0 + 14.0 * share, 0.0, 18.0);
        sim.event(EV_BLOOD, pl.soldier.p + V3{0, 0, 1.0}, pl.lastBlow * 0.3, float(clampd(share, 0.2, 1.0)));
      }
    }
  }
  pastAt = (pastAt + (int(sim.stepCount % 20) == 0 ? 1 : 0)) % kPastSteps;
  for (int i = 0; i < kMaxPlayers; i++) past[pastAt].soldier[i] = players[i].soldier.p;
  for (int i = 0; i < kTanks; i++) past[pastAt].tank[i] = frames[i].cog;
  bulletDebt += kDt;
  while (bulletDebt >= 0.005) {
    bulletDebt -= 0.005;
    stepBullets(0.005);
    stepMines(0.005);
    for (size_t k = 0; k < smoke.size();) {
      SmokeCloud& c = smoke[k];
      c.left -= 0.005;
      c.nextPuff -= 0.005;
      if (c.nextPuff <= 0) {
        c.nextPuff = 0.5;
        sim.event(EV_SMOKE, c.p, V3{}, float(clampd(c.left / 6.0, 0.15, 1.0)));
      }
      if (c.left <= 0) {
        smoke[k] = smoke.back();
        smoke.pop_back();
      } else {
        k++;
      }
    }
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
  for (int i = 0; i < kMaxPlayers; i++) {
    Player& pl = players[i];
    if (!pl.present) continue;
    Soldier& s = pl.soldier;
    pl.lastInput = in[i].seq;
    pl.shotClock = std::max(0.0, pl.shotClock - kDt);
    if (in[i].weapon >= 0 && in[i].weapon < WP_COUNT && pl.hasWeapon[in[i].weapon]) pl.weapon = in[i].weapon;
    // Getting down and up again takes its time, and only the big rifle asks for it.
    if (in[i].setup && pl.weapon == WP_SNIPER && pl.vehicle < 0) pl.wantProne = !pl.wantProne;
    if (pl.weapon != WP_SNIPER || pl.vehicle >= 0) pl.wantProne = false;
    s.prone = clampd(s.prone + (pl.wantProne ? 1.0 : -1.5) * kDt / kSetupTime * (pl.wantProne ? 1.0 : 3.0), 0.0, 1.0);
    if (in[i].strike && phase == PLAYING) callStrike(i, in[i].strikeAt);
    if (in[i].smoke && phase == PLAYING) useSmoke(i);
    // Half a life back, and it is not spent on someone who has nothing to mend.
    if (in[i].medkit && phase == PLAYING && pl.medkits > 0 && s.hp > 0 && s.hp < 100.0) {
      pl.medkits--;
      s.hp = std::min(s.hp + kMedkitHeal, 100.0);
    }
    if (in[i].net && phase == PLAYING && pl.vehicle >= 0 && pl.vehicle < kTanks && pl.items[IT_NET] > 0) {
      Tank& t = tanks[pl.vehicle];
      if (!t.netOn && t.netClock <= 0 && std::fabs(t.speed) < 1.0) t.netClock = 3.0;
    }
    stepDrone(i, in[i], kDt);
    if (pl.vehicle < 0 && phase == PLAYING && !pl.droneLive) {
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
        if (!pl.tapped && atEnemyComputer(i, s.p)) {
          pl.tapClock = kTapTime;
        } else if (c >= 0) {
          give(i, kGiveNames[crates[size_t(c)].what], crates[size_t(c)].count);
          moveMarker(sim, crates[size_t(c)].marker, V3{0, 0, kMarkerDepth});
        } else {
          const int v = nearestVehicle(s.p, 2.5);
          if (v >= 0) boardVehicle(i, v);
        }
      }
    }
    // Tapping takes staying at the keyboard.
    if (pl.tapClock > 0) {
      if (pl.vehicle >= 0 || !atEnemyComputer(i, s.p)) {
        pl.tapClock = 0;
      } else {
        pl.tapClock -= kDt;
        if (pl.tapClock <= 0) {
          pl.tapClock = 0;
          pl.tapped = true;
        }
      }
    }
    pl.fixWait = std::max(0.0, pl.fixWait - kDt);
    pl.fixAge += kDt;
    // In a tank the question goes by radio, and that needs its aerials.
    const bool radioGone = pl.vehicle >= 0 && pl.vehicle < kTanks && tanks[pl.vehicle].mod[TM_ANTENNA] <= 0;
    if (in[i].query && pl.tapped && pl.fixWait <= 0 && phase == PLAYING && !radioGone) {
      // Whichever of them is nearest.
      const int foe = enemyNearest(i, s.p);
      const Player& other = players[foe >= 0 ? foe : i];
      const V3 where = other.vehicle >= 0 ? frames[other.vehicle].cog : other.soldier.p;
      randState = randState * 1664525u + 1013904223u;
      const double a = double(randState >> 16) / 65535.0 * 2.0 * PI;
      randState = randState * 1664525u + 1013904223u;
      const double r = double(randState >> 16) / 65535.0 * kFixError;
      pl.fix = where + V3{std::cos(a) * r, std::sin(a) * r, 0};
      pl.hasFix = true;
      pl.fixAge = 0;
      pl.fixWait = kQueryWait;
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
      if (in[i].aiming) wi.sprint = false, wi.forward *= 0.55, wi.strafe *= 0.55;
      if (pl.droneLive) wi.forward = wi.strafe = 0, wi.jump = false, wi.sprint = false;
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
      const double along = dot(q, f.fwd);
      // Only what it is actually driving at: brushing along its side just pushes one away, and so does what is behind it.
      if (along * f.speed <= 0 || std::fabs(along) < f.halfLength * 0.5) continue;
      if (std::fabs(along) < f.halfLength + 0.5 && std::fabs(dot(q, f.left)) < f.halfWidth + 0.5) {
        s.hp -= std::fabs(f.speed) * 40.0 * kDt * 20.0;
        pl.lastBlow = f.fwd * (f.speed * 0.9) + V3{0, 0, 2.0};
      }
    }
  }
}

void Duel::restart() {
  world.clearCraters();
  sim = simStart;
  for (int i = 0; i < kTanks; i++) {
    tanks[i] = tanksStart[i];
    tankDamageSeen[i] = 0;
  }
  for (int i = 0; i < kWrecks; i++) wrecks[i] = wrecksStart[i];
  cars = carsStart;
  winner = -1;
  phaseClock = 0;
  bullets.clear();
  incoming.clear();
  smoke.clear();
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
  for (int i = 0; i < kMaxPlayers; i++) {
    Player& p = players[i];
    p.seat = 0;
    p.vehicle = -1;
    p.ready = false;
    p.repairClock = 0;
    p.repairing = -1;
    p.lastBlow = V3{};
    p.droneLive = false;
    p.drones = 0;
    p.hurtFrom = V3{};
    p.tapped = false;
    p.tapClock = p.fixWait = p.fixAge = 0;
    p.hasFix = false;
    p.weapon = WP_PISTOL;
    p.smallAmmo = 24;
    p.sniperAmmo = p.rpgAmmo = 0;
    p.tankShells = p.tankMissiles = p.strikeCodes = 0;
    p.medkits = 0;
    p.shotClock = 0;
    p.wantProne = false;
    for (int k = 0; k < WP_COUNT; k++) p.hasWeapon[k] = k <= WP_PISTOL;
    if (giveAll) {
      for (bool& h : p.hasWeapon) h = true;
      p.sniperAmmo = 20;
      p.rpgAmmo = 6;
    }
    p.walkDebt = 0;
    const World::Base& base = world.base(teamOf(i));
    const double cx = std::cos(base.facing), sx = std::sin(base.facing);
    p.soldier = Soldier{};
    // Side by side in front of their command centre.
    p.soldier.p = base.centre + V3{cx, sx, 0} * -7.0 + V3{-sx, cx, 0} * (double(i / 2) - 1.0) * 3.0;
    p.soldier.yaw = base.facing + PI;
  }
}

bool Duel::choose(int player, const u8 items[IT_COUNT], bool ready) {
  if (phase != CHOOSING || player < 0 || player >= kMaxPlayers) return false;
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

void Duel::step(const Intent in[kMaxPlayers]) {
  stepPlayers(in);
  sim.forces(kDt);
  Controls still;
  still.brake = 1;
  for (int i = 0; i < kTanks; i++) {
    Tank& t = tanks[i];
    // The driver has the pedals, the gunner the gun, and whoever is alone in it has both.
    Controls crew = still;
    const int driver = seatHolder(i, 0), gunner = seatHolder(i, 1);
    const int aimer = gunner >= 0 ? gunner : driver;
    if (driver >= 0) {
      crew = in[driver].c;
    } else if (gunner >= 0) {
      crew.brake = 1.0;
    }
    if (aimer >= 0) {
      crew.aimPoint = in[aimer].c.aimPoint;
      crew.indirect = in[aimer].c.indirect;
      crew.fire = in[aimer].c.fire;
      crew.launch = in[aimer].c.launch;
    }
    const Controls* c = &crew;
    const bool owned = driver >= 0 || gunner >= 0;
    if (owned || sim.objs[t.hullObj].awake || sim.objs[t.turretObj].awake) {
      if (!t.destroyed) {
        sim.wake(t.hullObj);
        sim.wake(t.turretObj);
      }
      t.update(sim, *c, kDt);
    } else {
      // Asleep nothing reads its speed any more, and what it last said must not outlive it: someone walking up to a
      // parked tank would be run over by a number.
      t.speed = 0;
      t.tickFire(sim, kDt);
    }
  }
  // Putting a net up takes a moment and a tank that stands still; and driving off leaves it behind.
  for (int v = 0; v < kTanks; v++) {
    Tank& t = tanks[v];
    if (t.netClock > 0) {
      if (std::fabs(t.speed) > 1.5) {
        t.netClock = 0;
      } else {
        t.netClock -= kDt;
        if (t.netClock <= 0) {
          t.netClock = 0;
          t.netOn = true;
          for (Player& pl : players)
            if (pl.vehicle == v && pl.items[IT_NET] > 0) pl.items[IT_NET]--;
        }
      }
    }
    if (t.netOn && std::fabs(t.speed) > 1.5) t.netOn = false;
  }
  // A tank that has just lost armour: whoever sits in it feels it, the one on the other side gets the credit.
  for (int v = 0; v < kTanks; v++) {
    if (tanks[v].damage > tankDamageSeen[v] + 0.005) {
      if (occupied(v)) {
        for (int who = 0; who < kMaxPlayers; who++) {
          if (players[who].vehicle == v) noteHit(who, -1, sim.lastBlast);
          // Everyone on the other side gets to see it land.
          else if (players[who].present && teamOf(who) != teamOf(seatHolder(v, 0) >= 0 ? seatHolder(v, 0) : seatHolder(v, 1))) noteHit(-1, who, sim.lastBlast);
        }
      }
    }
    tankDamageSeen[v] = tanks[v].damage;
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
    for (int p = 0; p < kMaxPlayers; p++)
      if (players[p].vehicle == kTanks + int(i)) {
        driven = in[p].c;
      }
    if (sim.objs[v.obj].awake) v.update(sim, driven, kDt);
    else v.speed = 0;
  }
  // Water holds back whatever goes through it. A tank just grinds on; a car, with less weight on it, hardly at all.
  for (int v = 0; v < kVehicles; v++) {
    const VehicleFrame& f = frames[v];
    if (!world.isWater(f.cog.x, f.cog.y, f.cog.z)) continue;
    const double k = v < kTanks ? 9.0 : 7.0;
    const u16 a = v < kTanks ? tanks[v].hullObj : cars[size_t(v - kTanks)].obj;
    if (!sim.objs[a].awake) continue;
    for (u32 n : sim.objs[a].nodes) sim.force[n] -= sim.vel[n] * (sim.mass[n] * k);
  }
  sim.finish(kDt);
  updateFrames();
  phaseClock += kDt;

  int here = 0, readyNow = 0;
  for (const Player& pl : players) {
    if (pl.present) here++;
    if (pl.present && pl.ready) readyNow++;
  }
  const bool both = here >= playersWanted;
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
      } else if (readyNow >= here || phaseClock > kChooseTime) {
        // What they chose survives the reset of everything else.
        u8 kept[kMaxPlayers][IT_COUNT];
        for (int i = 0; i < kMaxPlayers; i++) memcpy(kept[i], players[i].items, IT_COUNT);
        restart();
        for (int i = 0; i < kMaxPlayers; i++) {
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
      // A side is finished when nobody of it is left standing.
      bool dead[2] = {true, true};
      for (int i = 0; i < kMaxPlayers; i++)
        if (players[i].present && players[i].soldier.hp > 0) dead[teamOf(i)] = false;
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
