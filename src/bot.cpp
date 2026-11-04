#include "bot.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr double kThinkEvery = 0.05;
constexpr double kSightTank = 480.0, kSightFoot = 280.0, kSightHidden = 90.0;
constexpr double kReaction = 0.9;     // from seeing someone to the first shot
constexpr double kAimError = 0.0035;  // of the range, each shot its own
constexpr double kHintEvery = 30.0, kHintError = 110.0;
constexpr double kHoldRange = 320.0;  // a tank closer than this stands and shoots

V3 flat(V3 v) { return V3{v.x, v.y, 0}; }
}  // namespace

double Bot::chance(double a, double b) {
  rand_ = rand_ * 1664525u + 1013904223u;
  return a + (b - a) * double(rand_ >> 16) / 65535.0;
}

void Bot::step(Duel& d, int me, Duel::Intent& out) {
  clock_ += Duel::kDt;
  if (clock_ >= kThinkEvery) {
    clock_ -= kThinkEvery;
    think(d, me);
  }
  out = held_;
  // Pressed for one step, the way a key arrives.
  out.c.fire = pulse_.c.fire;
  out.c.launch = pulse_.c.launch;
  out.fireFoot = pulse_.fireFoot;
  out.use = pulse_.use;
  out.repair = pulse_.repair;
  out.smoke = pulse_.smoke;
  out.medkit = pulse_.medkit;
  out.c.requestGear = pulse_.c.requestGear;
  pulse_ = Duel::Intent{};
  pulse_.c.requestGear = 99;
}

void Bot::chooseLoadout(Duel& d, int me) {
  // A repair kit, a rifle with plenty for it and some smoke.
  u8 items[IT_COUNT] = {};
  items[IT_REPAIR] = 1;
  items[IT_AR] = 1;
  items[IT_SMALL_AMMO] = 6;
  items[IT_SMOKE] = 2;
  d.choose(me, items, true);
}

void Bot::think(Duel& d, int me) {
  Duel::Player& pl = d.players[me];
  if (d.matchId != match_) {
    const u32 keepRand = rand_;
    *this = Bot();
    rand_ = keepRand;
    match_ = d.matchId;
    pulse_.c.requestGear = 99;
  }
  held_ = Duel::Intent{};
  held_.c.brake = 1;
  held_.walk.yaw = pl.soldier.yaw;
  if (d.phase == Duel::CHOOSING && !pl.ready) chooseLoadout(d, me);
  if (d.phase != Duel::PLAYING || pl.soldier.hp <= 0) return;

  useClock_ -= kThinkEvery;
  shotClock_ -= kThinkEvery;
  pathAge_ += kThinkEvery;
  hintAge_ += kThinkEvery;
  lastSeenAge_ += kThinkEvery;
  lookClock_ -= kThinkEvery;
  held_.weapon = pl.weapon;

  // What lies about in reach gets picked up on the way, and a medkit is for using.
  if (pl.medkits > 0 && pl.soldier.hp < 55.0) pulse_.medkit = true;

  // Being hit says where from, if nothing else does.
  if (pl.hitsTaken != hitsSeen_) {
    hitsSeen_ = pl.hitsTaken;
    if (len2(pl.hurtFrom) > 1.0) {
      lastSeen_ = pl.hurtFrom;
      lastSeenAge_ = 0;
    }
    if (pl.vehicle >= 0 && pl.vehicle < Duel::kTanks && d.tanks[pl.vehicle].smokeCharges > 0 && d.tanks[pl.vehicle].damage > 0.45 && chance(0, 1) < 0.5) pulse_.smoke = true;
  }

  // From where the gun is: what the gun cannot reach is not worth a shell.
  const V3 eye = pl.vehicle >= 0 && pl.vehicle < Duel::kTanks ? d.tanks[pl.vehicle].muzzle : Duel::eyeOf(pl.soldier);
  if (lookClock_ <= 0) {
    lookClock_ = 0.25;
    lookAround(d, me, eye);
  }
  if (hintAge_ > kHintEvery) {
    // Whoever of them is nearest, about as well as a tapped computer would say it.
    double best = 1e30;
    for (int e = 0; e < Duel::kMaxPlayers; e++) {
      const Duel::Player& other = d.players[e];
      if (!other.present || Duel::teamOf(e) == Duel::teamOf(me) || other.soldier.hp <= 0) continue;
      const V3 where = other.vehicle >= 0 ? d.frames[other.vehicle].cog : other.soldier.p;
      const double dist = len2(where - eye);
      if (dist < best) {
        best = dist;
        const double a = chance(0, 2.0 * PI), r = chance(0, kHintError);
        hint_ = where + V3{std::cos(a) * r, std::sin(a) * r, 0};
        hintAge_ = 0;
      }
    }
  }
  if (pl.vehicle >= 0 && pl.vehicle < Duel::kTanks) inTank(d, me);
  else if (pl.vehicle >= Duel::kTanks) pulse_.use = true;  // not one for cars
  else onFoot(d, me);
}

bool Bot::canSee(const Duel& d, V3 from, V3 to, int foe) const {
  const V3 line = to - from;
  const double dist = len(line);
  if (dist < 1.0) return true;
  const Duel::Player& other = d.players[foe];
  double range = other.vehicle >= 0 ? kSightTank : kSightFoot;
  if (other.vehicle >= 0 && other.vehicle < Duel::kTanks && d.tanks[other.vehicle].netOn) range = kSightHidden * 2.0;
  if (other.vehicle < 0 && other.items[IT_GHILLIE] > 0) range = kSightHidden;
  if (other.vehicle < 0 && other.soldier.prone > 0.5) range *= 0.6;
  if (dist > range) return false;
  const V3 dir = line / dist;
  for (const Duel::SmokeCloud& c : d.smoke) {
    const double t = clampd(dot(c.p - from, dir), 0.0, dist);
    if (len2(from + dir * t - c.p) < 10.0 * 10.0) return false;
  }
  // The land and what stands on it, up to just before them.
  for (double s = 3.0; s < dist - 3.0; s += 4.0) {
    Contact c;
    if (d.world.contact(from + dir * s, c)) return false;
  }
  return true;
}

void Bot::lookAround(const Duel& d, int me, V3 eye) {
  int bestFoe = -1;
  double best = 1e30;
  for (int e = 0; e < Duel::kMaxPlayers; e++) {
    const Duel::Player& other = d.players[e];
    if (!other.present || Duel::teamOf(e) == Duel::teamOf(me) || other.soldier.hp <= 0) continue;
    const V3 at = other.vehicle >= 0 ? d.frames[other.vehicle].cog + d.frames[other.vehicle].up * 0.8 : other.soldier.p + V3{0, 0, 1.1};
    const double dist = len2(at - eye);
    if (dist < best && canSee(d, eye, at, e)) {
      best = dist;
      bestFoe = e;
      foeAt_ = at;
      foeVel_ = other.vehicle >= 0 ? d.frames[other.vehicle].fwd * d.frames[other.vehicle].speed : other.soldier.v;
    }
  }
  if (bestFoe >= 0) {
    if (!seesFoe_ || bestFoe != foe_) seenFor_ = 0;
    else seenFor_ += 0.25;
    foe_ = bestFoe;
    seesFoe_ = true;
    lastSeen_ = foeAt_;
    lastSeenAge_ = 0;
  } else {
    seesFoe_ = false;
    seenFor_ = 0;
  }
}

V3 Bot::goalFor(const Duel& d, int me) {
  if (lastSeenAge_ < 25.0) return lastSeen_;
  if (hintAge_ < 1e8) return hint_;
  return d.world.base(1 - Duel::teamOf(me)).centre;
}

// ---- Finding the way.

bool Bot::straightWay(const World& w, V3 a, V3 b) const {
  const V3 line = flat(b - a);
  const double dist = len(line);
  if (dist > 500.0) return false;
  if (dist < 1.0) return true;
  const V3 dir = line / dist;
  const double side = a.x - World::riverAt(a.y);
  for (double s = 0; s < dist; s += 6.0) {
    const V3 p = a + dir * s;
    if ((p.x - World::riverAt(p.y)) * side < 0) return false;
    if (w.terrainNormal(p.x, p.y).z < 0.8) return false;
    Contact c;
    const double ground = w.height(p.x, p.y);
    if (w.contact(V3{p.x, p.y, ground + 1.2}, c)) return false;
    if (w.isWater(p.x, p.y, ground)) return false;
  }
  return true;
}

bool Bot::roadWay(const World& w, V3 a, V3 b, std::vector<V3>& out) const {
  double bestCost = 1e30;
  const auto& roads = w.roads();
  for (size_t r = 0; r < roads.size(); r++) {
    const auto& pts = roads[r].pts;
    size_t from = 0, to = 0;
    double dFrom = 1e30, dTo = 1e30;
    for (size_t k = 0; k < pts.size(); k += 2) {
      const double da = std::hypot(pts[k].x - a.x, pts[k].y - a.y), db = std::hypot(pts[k].x - b.x, pts[k].y - b.y);
      if (da < dFrom) dFrom = da, from = k;
      if (db < dTo) dTo = db, to = k;
    }
    bool cut = false;
    for (const World::Bridge& br : w.bridgeSpans())
      if (br.road == int(r) && !br.open() && br.point >= std::min(from, to) && br.point <= std::max(from, to)) cut = true;
    if (cut) continue;
    // Away from the road and on to it again costs more than along it.
    const double along = std::fabs(double(to) - double(from)) * 1.5;
    const double cost = along + (dFrom + dTo) * 1.6;
    if (cost >= bestCost) continue;
    bestCost = cost;
    out.clear();
    const int stepBy = to >= from ? 8 : -8;
    for (long k = long(from); stepBy > 0 ? k < long(to) : k > long(to); k += stepBy) out.push_back(V3{pts[size_t(k)].x, pts[size_t(k)].y, pts[size_t(k)].z});
    out.push_back(V3{pts[to].x, pts[to].y, pts[to].z});
  }
  return bestCost < 1e30;
}

void Bot::planTo(const Duel& d, V3 from, V3 goal) {
  const World& w = d.world;
  path_.clear();
  pathAt_ = 0;
  pathGoal_ = goal;
  pathAge_ = 0;
  fallsSeen_ = w.bridgeFalls();
  if (straightWay(w, from, goal)) {
    path_.push_back(goal);
    return;
  }
  const bool sameSide = (from.x - World::riverAt(from.y)) * (goal.x - World::riverAt(goal.y)) > 0;
  std::vector<V3> way;
  if (roadWay(w, from, goal, way)) {
    bool crosses = false;
    for (size_t k = 1; k < way.size(); k++) crosses |= (way[k - 1].x - World::riverAt(way[k - 1].y)) * (way[k].x - World::riverAt(way[k].y)) < 0;
    if (sameSide || crosses) {
      path_ = std::move(way);
      path_.push_back(goal);
      return;
    }
  }
  // No road across any more: through whichever ford is the shorter way, and back onto the roads on either side.
  int ford = 0;
  double best = 1e30;
  for (int f = 0; f < 2; f++) {
    const V3 p = World::fordPoint(f);
    const double cost = len(flat(p - from)) + len(flat(goal - p));
    if (cost < best) best = cost, ford = f;
  }
  V3 mid = World::fordPoint(ford);
  mid.z = w.height(mid.x, mid.y);
  // Which bank one comes from is settled by where one is going, not by how far across one has got.
  const double near = goal.x > mid.x ? -1.0 : 1.0;
  V3 in = mid + V3{near * 75.0, 0, 0}, out = mid + V3{-near * 75.0, 0, 0};
  in.z = w.height(in.x, in.y);
  out.z = w.height(out.x, out.y);
  // Already at it, there is no going back to where one comes in.
  const bool atFord = std::fabs(from.x - mid.x) < 85.0 && std::fabs(from.y - mid.y) < 45.0;
  if (!atFord) {
    if (!straightWay(w, from, in) && roadWay(w, from, in, way)) path_ = way;
    path_.push_back(in);
  }
  path_.push_back(mid);
  path_.push_back(out);
  if (!straightWay(w, out, goal) && roadWay(w, out, goal, way)) path_.insert(path_.end(), way.begin(), way.end());
  path_.push_back(goal);
}

V3 Bot::nextPoint(V3 at, double reach) {
  while (pathAt_ + 1 < path_.size() && len(flat(path_[pathAt_] - at)) < reach) pathAt_++;
  return path_.empty() ? at : path_[pathAt_];
}

// ---- On foot.

void Bot::walkTowards(const Duel& d, int me, V3 at, bool sprint) {
  const Soldier& s = d.players[me].soldier;
  const V3 to = flat(at - s.p);
  const double dist = len(to);
  held_.c.brake = 0;
  if (dist > 0.3) held_.walk.yaw = std::atan2(-to.y, -to.x);
  held_.walk.forward = dist > 0.6 ? 1.0 : 0.0;
  held_.walk.sprint = sprint && dist > 8.0;
  held_.pitch = 0;
  // Not getting anywhere: off to one side for a moment, and a hop.
  stuckClock_ += kThinkEvery;
  if (stuckClock_ > 1.5) {
    if (len(flat(s.p - stuckAt_)) < 1.2 && dist > 1.5) {
      unstickClock_ = 1.2;
      unstickSteer_ = chance(0, 1) < 0.5 ? -1.0 : 1.0;
    }
    stuckAt_ = s.p;
    stuckClock_ = 0;
  }
  if (unstickClock_ > 0) {
    unstickClock_ -= kThinkEvery;
    held_.walk.strafe = unstickSteer_;
    held_.walk.jump = chance(0, 1) < 0.1;
  }
}

void Bot::onFoot(Duel& d, int me) {
  Duel::Player& pl = d.players[me];
  const Soldier& s = pl.soldier;
  if (useClock_ <= 0 && d.nearCrate(s.p + V3{0, 0, 0.3}) >= 0) {
    pulse_.use = true;
    useClock_ = 0.6;
    return;
  }

  // Finishing a repair comes first, unless someone is shooting at close range.
  if (repairing_ && !(seesFoe_ && len(foeAt_ - s.p) < 60.0)) {
    if (repairTank_ < 0 || d.vehicleDead(repairTank_) || pl.items[IT_REPAIR] == 0) {
      repairing_ = false;
    } else if (pl.repairClock > 0) {
      return;  // standing at it
    } else {
      const Tank& t = d.tanks[repairTank_];
      const bool broken = t.trackOff[0] || t.trackOff[1] || t.mod[TM_ENGINE] <= 0 || t.mod[TM_GUN] <= 0;
      if (broken) {
        if (d.nearestToRepair(s.p) == repairTank_) pulse_.repair = true;
        else walkTowards(d, me, d.frames[repairTank_].cog, false);
        return;
      }
      repairing_ = false;
      wantVehicle_ = repairTank_;
    }
  }

  const bool foeOnFoot = seesFoe_ && d.players[foe_].vehicle < 0;
  const bool foeInTank = seesFoe_ && d.players[foe_].vehicle >= 0;
  int gun = pl.hasWeapon[WP_AR] && pl.smallAmmo > 0 ? WP_AR : (pl.smallAmmo > 0 ? WP_PISTOL : WP_KNIFE);
  if (foeInTank && pl.hasWeapon[WP_RPG] && pl.rpgAmmo > 0) gun = WP_RPG;
  held_.weapon = gun;

  if (seesFoe_ && (foeOnFoot || gun == WP_RPG)) {
    const V3 eye = Duel::eyeOf(s);
    const double dist = len(foeAt_ - eye);
    const double reach = gun == WP_RPG ? 180.0 : (gun == WP_AR ? 220.0 : 90.0);
    if (dist < reach && !(gun == WP_KNIFE && dist > 2.0)) {
      // Standing, along the sights, and in bursts.
      V3 aim = foeAt_ + foeVel_ * (dist / std::max(kWeapons[gun].speed, 100.0)) + aimOff_;
      const double flight = dist / std::max(kWeapons[gun].speed, 100.0);
      if (gun != WP_RPG) aim.z += 4.9 * flight * flight;
      const V3 to = aim - eye;
      held_.walk.yaw = std::atan2(-to.y, -to.x);
      held_.pitch = std::asin(clampd(-to.z / std::max(len(to), 1e-6), -1, 1));
      held_.c.aimPoint = aim;
      held_.aiming = dist > 18.0;
      held_.c.brake = 0;
      if (seenFor_ > kReaction * 0.7) {
        if (shotClock_ <= 0) {
          const double spread = dist * kAimError * 1.4 + 0.15;
          aimOff_ = V3{chance(-spread, spread), chance(-spread, spread), chance(-spread, spread) * 0.6};
          pulse_.fireFoot = true;
          shotClock_ = gun == WP_AR ? 0.14 : kWeapons[gun].interval + 0.1;
          if (gun == WP_AR && chance(0, 1) < 0.2) shotClock_ = 0.6;
        }
        held_.fireHeld = gun == WP_AR && shotClock_ < 0.2;
      }
      if (gun == WP_KNIFE) walkTowards(d, me, foeAt_, true);
      return;
    }
  }

  // Find a tank: anything of use that nobody is in, on either side.
  if (wantVehicle_ < 0 || d.vehicleDead(wantVehicle_) || d.occupied(wantVehicle_) || d.tanks[wantVehicle_].ammo == 0) {
    wantVehicle_ = -1;
    double best = 900.0 * 900.0;
    for (int v = 0; v < Duel::kTanks; v++) {
      const Tank& t = d.tanks[v];
      if (t.destroyed || d.occupied(v) || t.ammo == 0 || t.mod[TM_GUN] <= 0) continue;
      if (t.kind != TK_STANDARD && t.kind != TK_LIGHT) continue;  // a howitzer or a missile wants a different head
      if (t.trackOff[0] || t.trackOff[1]) continue;
      double dist = len2(d.frames[v].cog - s.p);
      // Each bot leans to another one, and the front row is the easy way out of the yard.
      dist *= 1.0 + double((v + me * 5) % 7) * 0.08;
      if (t.kind == TK_LIGHT) dist *= 30.0;  // only if there is nothing better near
      if (dist < best) best = dist, wantVehicle_ = v;
    }
    pathAge_ = 1e9;
  }
  if (wantVehicle_ >= 0) {
    if (useClock_ <= 0 && d.nearestVehicle(s.p, 2.5) == wantVehicle_) {
      pulse_.use = true;
      useClock_ = 0.8;
      return;
    }
    const V3 at = d.frames[wantVehicle_].cog;
    if (pathAge_ > 6.0 || d.world.bridgeFalls() != fallsSeen_) planTo(d, s.p, at);
    walkTowards(d, me, nextPoint(s.p, 3.0), true);
    return;
  }
  // Nothing to drive: after them on foot.
  const V3 goal = goalFor(d, me);
  if (pathAge_ > 10.0 || len(flat(goal - pathGoal_)) > 50.0 || d.world.bridgeFalls() != fallsSeen_) planTo(d, s.p, goal);
  walkTowards(d, me, nextPoint(s.p, 3.0), true);
}

// ---- In a tank.

void Bot::driveTowards(const Duel& d, int me, V3 at, double wantSpeed) {
  const VehicleFrame& f = d.frames[d.players[me].vehicle];
  const V3 to = flat(at - f.cog);
  double err = std::atan2(dot(to, f.left), dot(to, f.fwd));
  // Around what stands in the way: other vehicles are not on the map of the roads.
  for (int v = 0; v < Duel::kVehicles; v++) {
    if (v == d.players[me].vehicle) continue;
    const V3 q = flat(d.frames[v].cog - f.cog);
    const double ahead = dot(q, f.fwd), side = dot(q, f.left);
    if (ahead > 0 && ahead < 16.0 && std::fabs(side) < d.frames[v].halfWidth + f.halfWidth + 1.2) {
      err += (side > 0 ? -1.0 : 1.0) * 0.8 * (1.0 - ahead / 16.0) + (side > 0 ? -0.15 : 0.15);
      wantSpeed = std::min(wantSpeed, 6.0);
    }
  }
  held_.c.brake = 0;
  held_.c.steer = clampd(err * 2.0, -1.0, 1.0);
  // Round a sharp bend slowly, and turn on the spot if it is sharper still.
  const double turning = std::fabs(err);
  const double aim = turning > 1.1 ? 0.0 : wantSpeed * clampd(1.25 - turning * 1.1, 0.25, 1.0);
  const double speed = f.speed;
  held_.c.throttle = speed < aim ? clampd((aim - speed) * 0.6 + 0.25, 0.0, 1.0) : 0.0;
  if (speed > aim + 2.0) held_.c.brake = clampd((speed - aim) * 0.25, 0.0, 1.0);
  if (speed < 1.0) pulse_.c.requestGear = 1;

  // Pressing on without moving: back off, turning.
  stuckClock_ += kThinkEvery;
  if (stuckClock_ > 2.5) {
    // Wading is slow going, and that is not the same as being stuck.
    const double least = d.world.isWater(f.cog.x, f.cog.y, f.cog.z) ? 0.3 : 1.5;
    if (len(flat(f.cog - stuckAt_)) < least && wantSpeed > 1.0 && turning < 1.1) {
      unstickClock_ = 3.0;
      unstickSteer_ = chance(0, 1) < 0.5 ? -0.8 : 0.8;
    }
    stuckAt_ = f.cog;
    stuckClock_ = 0;
  }
  if (unstickClock_ > 0) {
    unstickClock_ -= kThinkEvery;
    held_.c.steer = unstickSteer_;
    held_.c.throttle = 0.9;
    held_.c.brake = 0;
    if (std::fabs(speed) < 1.0) pulse_.c.requestGear = -1;
    if (unstickClock_ <= 0) pathAge_ = 1e9;
  }
}

void Bot::aimAt(const Duel& d, int me, double shellSpeed, V3 from) {
  const double dist = len(foeAt_ - from);
  const double flight = dist / shellSpeed;
  held_.c.aimPoint = foeAt_ + foeVel_ * flight + aimOff_;
}

void Bot::inTank(Duel& d, int me) {
  Duel::Player& pl = d.players[me];
  const int v = pl.vehicle;
  const Tank& t = d.tanks[v];
  const VehicleFrame& f = d.frames[v];
  const bool driver = pl.seat == 0;
  const bool lame = t.trackOff[0] || t.trackOff[1] || t.mod[TM_ENGINE] <= 0 || t.fuel <= 0;
  const bool gunGone = t.mod[TM_GUN] <= 0;

  // What a kit can put right is worth getting out for, once nobody is close.
  if ((lame || gunGone) && pl.items[IT_REPAIR] > 0 && !(seesFoe_ && len(foeAt_ - f.cog) < 120.0) && std::fabs(f.speed) < 1.0 && useClock_ <= 0) {
    repairing_ = true;
    repairTank_ = v;
    wantVehicle_ = -1;
    pulse_.use = true;
    useClock_ = 0.8;
    return;
  }
  // Nothing left to fight with, and no way of mending that: out, and on to another one.
  if ((t.ammo == 0 || gunGone) && useClock_ <= 0 && std::fabs(f.speed) < 2.0) {
    wantVehicle_ = -1;
    pulse_.use = true;
    useClock_ = 0.8;
    return;
  }

  if (seesFoe_) {
    aimAt(d, me, t.spec().shellSpeed, t.muzzle);
    const V3 wanted = norm(flat(held_.c.aimPoint - t.muzzle));
    const V3 lay = norm(flat(t.gunDir));
    const double off = std::acos(clampd(dot(wanted, lay), -1.0, 1.0));
    if (seenFor_ > kReaction && off < 0.012 && t.reload <= 0 && t.ammo != 0 && !gunGone && shotClock_ <= 0) {
      pulse_.c.fire = true;
      shotClock_ = 0.4;
      const double spread = len(foeAt_ - t.muzzle) * kAimError + 0.2;
      aimOff_ = V3{chance(-spread, spread), chance(-spread, spread), chance(-spread, spread) * 0.5};
    }
  } else if (lastSeenAge_ < 12.0) {
    held_.c.aimPoint = lastSeen_;
  } else {
    held_.c.aimPoint = f.cog + f.fwd * 60.0 + V3{0, 0, 2.0};
  }
  if (!driver || lame) return;

  if (seesFoe_ && len(foeAt_ - f.cog) < kHoldRange) {
    // Close enough. Stand, and turn the thick end to them.
    const V3 to = flat(foeAt_ - f.cog);
    const double err = std::atan2(dot(to, f.left), dot(to, f.fwd));
    held_.c.throttle = 0;
    held_.c.brake = std::fabs(f.speed) > 0.5 ? 1.0 : 0.0;
    held_.c.steer = std::fabs(err) > 0.3 ? clampd(err * 1.5, -0.8, 0.8) : 0.0;
    stuckClock_ = 0;
    stuckAt_ = f.cog;
    return;
  }
  const V3 goal = goalFor(d, me);
  if (pathAge_ > 30.0 || len(flat(goal - pathGoal_)) > 150.0 || d.world.bridgeFalls() != fallsSeen_) planTo(d, f.cog, goal);
  const double left = len(flat(goal - f.cog));
  driveTowards(d, me, nextPoint(f.cog, 13.0), left < 40.0 ? 3.0 : (seesFoe_ ? 7.0 : 13.0));
}
