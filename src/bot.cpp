#include "bot.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr double kThinkEvery = 0.05;
constexpr double kSightTank = 480.0, kSightFoot = 280.0, kSightHidden = 90.0;
constexpr double kReaction = 0.9;     // from seeing someone to the first shot
constexpr double kAimError = 0.0035;  // of the range, each shot its own
constexpr double kHintEvery = 30.0, kHintError = 110.0;
constexpr double kHoldRange = 320.0;   // a tank closer than this stands and shoots
constexpr double kAmbushRange = 380.0;  // whoever lies in wait lets them come this close first
constexpr double kTrackSight = 40.0;    // how far off a track in the grass can be made out
constexpr double kWalkToCrate = 130.0;  // further than this is a job for the tank

V3 flat(V3 v) { return V3{v.x, v.y, 0}; }

// The trees by where they stand: there are too many to go through for every look.
struct TreeGrid {
  static constexpr int kCells = 64;
  static constexpr double kCell = 2.0 * World::kHalf / kCells;
  const World* world = nullptr;
  std::vector<std::vector<V3>> cells;
  static int index(double v) { return std::clamp(int((v + World::kHalf) / kCell), 0, kCells - 1); }
  const std::vector<V3>& at(double x, double y) const { return cells[size_t(index(y)) * kCells + size_t(index(x))]; }
};
const TreeGrid& treesOf(const World& w) {
  static TreeGrid grid;
  if (grid.world != &w) {
    grid.world = &w;
    grid.cells.assign(TreeGrid::kCells * TreeGrid::kCells, {});
    for (const V3& t : w.trees()) grid.cells[size_t(TreeGrid::index(t.y)) * TreeGrid::kCells + size_t(TreeGrid::index(t.x))].push_back(t);
  }
  return grid;
}
int treesAround(const World& w, V3 p, double reach) {
  const TreeGrid& grid = treesOf(w);
  int n = 0;
  for (double dx : {-reach, 0.0, reach})
    for (double dy : {-reach, 0.0, reach}) {
      // Each cell once, however many of the three corners fall into it.
      const int cx = TreeGrid::index(p.x + dx), cy = TreeGrid::index(p.y + dy);
      if ((dx != -reach && cx == TreeGrid::index(p.x + dx - reach)) || (dy != -reach && cy == TreeGrid::index(p.y + dy - reach))) continue;
      for (const V3& t : grid.at(p.x + dx, p.y + dy))
        if (std::hypot(t.x - p.x, t.y - p.y) < reach) n++;
    }
  return n;
}
}  // namespace

const char* Bot::tacticName(Tactic t) {
  static const char* const kNames[kTacticCount] = {"Sturm", "Flanke", "Hinterhalt", "Scharfschuetze", "Pluenderer", "Spion", "Lenkrakete"};
  return t < kTacticCount ? kNames[t] : "?";
}

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
  out.net = pulse_.net;
  out.query = pulse_.query;
  out.setup = pulse_.setup;
  out.c.requestGear = pulse_.c.requestGear;
  pulse_ = Duel::Intent{};
  pulse_.c.requestGear = 99;
}

// ---- The plan for the round.

void Bot::pickTactic(const Duel& d, int me) {
  (void)d;
  (void)me;
  lastTactic_ = picked_ ? tactic_ : lastTactic_;
  static constexpr double kWeight[kTacticCount] = {3.0, 3.0, 3.0, 2.0, 2.0, 1.5, 2.0};
  // Never the same way twice running: whoever was caught out by it last time is ready for it now.
  double total = 0;
  for (int t = 0; t < kTacticCount; t++)
    if (t != lastTactic_) total += kWeight[t];
  double roll = chance(0, total);
  tactic_ = STORM;
  for (int t = 0; t < kTacticCount; t++) {
    if (t == lastTactic_) continue;
    if (roll < kWeight[t]) {
      tactic_ = Tactic(t);
      break;
    }
    roll -= kWeight[t];
  }
  picked_ = true;
  tacticClock_ = 0;
  // How long each way of playing gets, before it goes looking for them after all.
  tacticLimit_ = 1e9;
  if (tactic_ == AMBUSH) tacticLimit_ = chance(110.0, 220.0);
  if (tactic_ == SNIPER) tacticLimit_ = chance(170.0, 280.0);
  if (tactic_ == SCAVENGE) tacticLimit_ = chance(90.0, 150.0);
  if (tactic_ == SPY) tacticLimit_ = 240.0;
  cratesWanted_ = tactic_ == SCAVENGE ? int(chance(3.0, 5.99)) : 0;
  flankSide_ = chance(0, 1) < 0.5 ? -1 : 1;
}

void Bot::chooseLoadout(Duel& d, int me) {
  u8 items[IT_COUNT] = {};
  switch (tactic_) {
    case AMBUSH:  // to sit under, and to get away from it with
      items[IT_REPAIR] = 1, items[IT_NET] = 1, items[IT_AR] = 1, items[IT_SMALL_AMMO] = 3, items[IT_SMOKE] = 2;
      break;
    case SNIPER:  // the rifle itself has to be found
      items[IT_GHILLIE] = 1, items[IT_AR] = 1, items[IT_SMALL_AMMO] = 6, items[IT_SMOKE] = 3;
      break;
    case SCAVENGE:  // travelling light: what it needs is out there
      items[IT_AR] = 1, items[IT_SMALL_AMMO] = 3, items[IT_SMOKE] = 2;
      break;
    case FLANK: case SPY: case MISSILE:
      items[IT_REPAIR] = 1, items[IT_AR] = 1, items[IT_SMALL_AMMO] = 4, items[IT_SMOKE] = 4;
      break;
    default:
      items[IT_REPAIR] = 1, items[IT_AR] = 1, items[IT_SMALL_AMMO] = 6, items[IT_SMOKE] = 2;
      break;
  }
  d.choose(me, items, true);
}

void Bot::think(Duel& d, int me) {
  Duel::Player& pl = d.players[me];
  if (d.matchId != match_) {
    // The match has a new number when the choosing starts and again when the shooting does: the plan outlives the second.
    Bot fresh;
    fresh.rand_ = rand_;
    fresh.aimSkill_ = aimSkill_;
    fresh.reactSkill_ = reactSkill_;
    fresh.tactic_ = tactic_;
    fresh.lastTactic_ = lastTactic_;
    fresh.picked_ = picked_;
    fresh.tacticLimit_ = tacticLimit_;
    fresh.cratesWanted_ = cratesWanted_;
    fresh.flankSide_ = flankSide_;
    fresh.tracksFollowed_ = tracksFollowed_;
    fresh.cratesAllTime_ = cratesAllTime_;
    fresh.tapsAllTime_ = tapsAllTime_;
    fresh.rescuesAllTime_ = rescuesAllTime_;
    *this = fresh;
    match_ = d.matchId;
    crateTried_.assign(d.crates.size(), 0);
    pulse_.c.requestGear = 99;
    if (d.phase == Duel::CHOOSING) {
      lastTactic_ = picked_ ? tactic_ : lastTactic_;
      picked_ = false;
    }
  }
  held_ = Duel::Intent{};
  held_.c.brake = 1;
  held_.walk.yaw = pl.soldier.yaw;
  if (d.phase == Duel::CHOOSING) {
    if (!picked_) pickTactic(d, me);
    if (!pl.ready) chooseLoadout(d, me);
  }
  if (d.phase != Duel::PLAYING || pl.soldier.hp <= 0) return;
  if (!picked_) pickTactic(d, me);

  now_ += kThinkEvery;
  useClock_ -= kThinkEvery;
  shotClock_ -= kThinkEvery;
  pathAge_ += kThinkEvery;
  hintAge_ += kThinkEvery;
  lastSeenAge_ += kThinkEvery;
  lookClock_ -= kThinkEvery;
  tacticClock_ += kThinkEvery;
  dodgeClock_ -= kThinkEvery;
  shiftClock_ -= kThinkEvery;
  queryClock_ -= kThinkEvery;
  held_.weapon = pl.weapon;
  // Waited long enough, whatever the plan was: after them.
  if (tacticClock_ > tacticLimit_ && tactic_ != STORM) {
    tactic_ = STORM;
    hasPost_ = atPost_ = false;
    pathAge_ = 1e9;
  }

  if (pl.medkits > 0 && pl.soldier.hp < 55.0) pulse_.medkit = true;

  // From where the gun is: what the gun cannot reach is not worth a shell.
  const V3 eye = pl.vehicle >= 0 && pl.vehicle < Duel::kTanks ? d.tanks[pl.vehicle].muzzle : Duel::eyeOf(pl.soldier);
  const V3 here = pl.vehicle >= 0 ? d.frames[pl.vehicle].cog : pl.soldier.p;
  noteHits(d, me, eye);
  if (lookClock_ <= 0) {
    lookClock_ = 0.25;
    lookAround(d, me, eye);
  }
  noteTracks(d, me, here);
  checkIdle(d, me, here);
  if (pl.tapped && !wasTapped_) {
    wasTapped_ = true;
    tapsAllTime_++;
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
  // Having tapped their computer, it asks; and that is better than any hint.
  if (pl.tapped && queryClock_ <= 0) {
    pulse_.query = true;
    queryClock_ = 2.0;
    // It says where all of them are; the nearest is the one to go for.
    double nearest = 1e30;
    for (const Duel::Player::Fix& f : pl.fixes) {
      if (!f.on || f.age > 3.0 || len2(f.at - eye) >= nearest) continue;
      nearest = len2(f.at - eye);
      hint_ = f.at;
      hintAge_ = 0;
    }
  }
  if (pl.vehicle >= 0 && pl.vehicle < Duel::kTanks) inTank(d, me);
  else if (pl.vehicle >= Duel::kTanks) pulse_.use = true;  // not one for cars
  else onFoot(d, me);
}

// Whatever went wrong, and it does not have to be understood: no way of playing is worth standing still for without a
// reason. First the plan goes, then the vehicle.
void Bot::checkIdle(Duel& d, int me, V3 here) {
  const Duel::Player& pl = d.players[me];
  if (len2(flat(here - idleAt_)) > 12.0 * 12.0) {
    idleAt_ = here;
    idleClock_ = 0;
    idleStrikes_ = 0;
    return;
  }
  if (seesFoe_ || atPost_ || pl.repairClock > 0 || pl.tapClock > 0) {
    idleClock_ = 0;
    return;
  }
  idleClock_ += kThinkEvery;
  if (idleClock_ < 30.0) return;
  idleClock_ = 0;
  idleStrikes_++;
  rescuesAllTime_++;
  if (crate_ >= 0 && size_t(crate_) < crateTried_.size()) crateTried_[size_t(crate_)] = 1;
  crate_ = -1;
  tactic_ = STORM;
  hasPost_ = atPost_ = false;
  flanked_ = true;
  onTrack_ = false;
  lastSeenAge_ = 1e9;
  repairing_ = false;
  pathAge_ = 1e9;
  if (pl.vehicle < 0 && wantVehicle_ >= 0) {
    givenUp_[wantVehicle_] = true;
    wantVehicle_ = -1;
  }
  if (pl.vehicle >= 0 && pl.vehicle < Duel::kTanks && idleStrikes_ >= 2) bail(pl.vehicle);
}

// ---- What it notices.

void Bot::noteHits(const Duel& d, int me, V3 eye) {
  const Duel::Player& pl = d.players[me];
  if (pl.hitsTaken == hitsSeen_) return;
  hitsSeen_ = pl.hitsTaken;
  // Where the shot landed says nothing about where it came from, and standing on that spot looking for them helps nobody.
  // A person sees the trail or hears the gun: that is about where they are, and no more than that.
  const bool inTankNow = pl.vehicle >= 0 && pl.vehicle < Duel::kTanks;
  int by = -1;
  if (inTankNow) by = d.tanks[pl.vehicle].hitBy;
  else if (d.hurtByAge[me] < 3.0) by = d.hurtBy[me];
  if (by >= 0 && by != me && Duel::teamOf(by) != Duel::teamOf(me) && d.players[by].present) {
    const Duel::Player& other = d.players[by];
    const V3 where = other.vehicle >= 0 ? d.frames[other.vehicle].cog : other.soldier.p;
    const double err = clampd(len(where - eye) * 0.12, 5.0, 70.0);
    const double a = chance(0, 2.0 * PI), r = chance(0, err);
    lastSeen_ = where + V3{std::cos(a) * r, std::sin(a) * r, 0};
    lastSeenAge_ = 0;
  }
  if (inTankNow) {
    // Not where they last saw it, either.
    dodgeClock_ = chance(2.5, 4.5);
    dodgeSide_ = chance(0, 1) < 0.5 ? -1.0 : 1.0;
    const Tank& t = d.tanks[pl.vehicle];
    if (t.smokeCharges > 0 && t.damage > 0.45 && chance(0, 1) < 0.6) pulse_.smoke = true;
    shotsFromPost_ += 3;  // found out: this place is no good any more
  }
}

bool Bot::canSee(const Duel& d, V3 from, V3 to, int foe) const {
  const V3 line = to - from;
  const double dist = len(line);
  if (dist < 1.0) return true;
  const Duel::Player& other = d.players[foe];
  double range = other.vehicle >= 0 ? kSightTank : kSightFoot;
  // What moves catches the eye. What stands still among the bushes does not, from so far.
  if (other.vehicle >= 0 && std::fabs(d.frames[other.vehicle].speed) < 0.5) range *= 0.7;
  if (other.vehicle >= 0 && (other.vehicle < Duel::kTanks ? d.tanks[other.vehicle].netOn : d.cars[size_t(other.vehicle - Duel::kTanks)].netOn)) range = kSightHidden * 2.0;
  if (other.vehicle < 0 && other.soldier.crouch > 0.5) range *= 0.8;
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
  // The houses are not on the map of the land, but nobody sees through them. Not unless one of the two is inside.
  for (const World::House& h : d.world.houses()) {
    const V3 mid = World::houseInside(h, World::kHouseLength * 0.5, World::kHouseDepth * 0.5, 0);
    const double along = clampd(dot(flat(mid - from), dir) / std::max(dir.x * dir.x + dir.y * dir.y, 1e-6), 0.0, dist);
    if (len2(flat(from + dir * along - mid)) > 7.0 * 7.0) continue;
    const double c = std::cos(h.yaw), s = std::sin(h.yaw);
    const auto inside = [&](V3 p) {
      const double u = (p.x - h.x) * c + (p.y - h.y) * s, v = -(p.x - h.x) * s + (p.y - h.y) * c;
      return u > 0 && u < World::kHouseLength && v > 0 && v < World::kHouseDepth;
    };
    if (inside(from) || inside(to)) continue;
    for (double t = along - 7.0; t < along + 7.0; t += 1.0) {
      const V3 p = from + dir * clampd(t, 0.0, dist);
      if (inside(p) && p.z - d.world.height(p.x, p.y) < 3.2) return false;
    }
  }
  // Trees: one in the way close to either end hides everything, and several in a row do as well.
  const TreeGrid& grid = treesOf(d.world);
  int crowns = 0;
  for (double s = 4.0; s < dist; s += 8.0) {
    const V3 p = from + dir * s;
    for (const V3& t : grid.at(p.x, p.y)) {
      if (p.z - t.z > 11.0 || std::hypot(t.x - p.x, t.y - p.y) > 3.2) continue;
      if (s < 12.0 || dist - s < 12.0 || ++crowns >= 2) return false;
    }
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
    onTrack_ = false;
  } else {
    seesFoe_ = false;
    seenFor_ = 0;
  }
}

// What the enemy drives over the grass stays pressed flat: on the road or the gravel nothing shows.
void Bot::noteTracks(const Duel& d, int me, V3 at) {
  (void)at;
  trackLook_ -= kThinkEvery;
  if (std::fmod(now_, 1.0) >= kThinkEvery) return;
  for (int v = 0; v < Duel::kVehicles; v++) {
    const int holder = d.seatHolder(v, 0) >= 0 ? d.seatHolder(v, 0) : d.seatHolder(v, 1);
    if (holder < 0 || Duel::teamOf(holder) == Duel::teamOf(me)) continue;
    const VehicleFrame& f = d.frames[v];
    if (std::fabs(f.speed) < 1.0 || len2(flat(f.cog - markedAt_[v])) < 10.0 * 10.0) continue;
    if (d.world.surfaceAt(f.cog.x, f.cog.y) != SURF_GRASS || d.world.isWater(f.cog.x, f.cog.y, f.cog.z)) continue;
    markedAt_[v] = f.cog;
    if (marks_.size() >= 4000) marks_.erase(marks_.begin(), marks_.begin() + 500);
    marks_.push_back(Mark{f.cog, v, now_});
  }
}

// Across a track, it goes the way the track goes. Only what it can see from where it is, and only forwards.
bool Bot::lookForTracks(const Duel& d, V3 eye) {
  if (trackLook_ > 0) return onTrack_;
  trackLook_ = 1.5;
  if (onTrack_ && len2(flat(trackGoal_ - eye)) < 18.0 * 18.0) onTrack_ = false;
  const Mark* found = nullptr;
  for (const Mark& m : marks_) {
    if (m.when <= trackWhen_ || (found && m.when <= found->when)) continue;
    const V3 to = flat(m.at - eye);
    if (len2(to) > kTrackSight * kTrackSight) continue;
    bool clear = true;
    const double dist = len(m.at - eye);
    for (double s = 3.0; s < dist - 2.0 && clear; s += 4.0) {
      Contact c;
      if (d.world.contact(eye + (m.at + V3{0, 0, 1.0} - eye) * (s / std::max(dist, 1.0)), c)) clear = false;
    }
    if (clear) found = &m;
  }
  if (!found) return onTrack_;
  if (!onTrack_) tracksFollowed_++;
  // On along it as far as it goes in sight.
  V3 ahead = found->at;
  for (const Mark& m : marks_)
    if (m.vehicle == found->vehicle && m.when > found->when && len2(flat(m.at - found->at)) < 70.0 * 70.0) ahead = m.at;
  trackGoal_ = ahead;
  trackWhen_ = found->when;
  trackVehicle_ = found->vehicle;
  onTrack_ = true;
  pathAge_ = 1e9;
  return true;
}

V3 Bot::goalFor(const Duel& d, int me, V3 at) {
  if (lastSeenAge_ < 25.0) {
    // Been there, and nobody is: on with whatever else there is to go on.
    if (len2(flat(lastSeen_ - at)) > 30.0 * 30.0) return lastSeen_;
    lastSeenAge_ = 1e9;
  }
  if (onTrack_) return trackGoal_;
  if (hintAge_ < 1e8) {
    if (len2(flat(hint_ - at)) > 35.0 * 35.0) return hint_;
    // They were roughly here. Along some road until the next word comes.
    const auto& roads = d.world.roads();
    if (!roads.empty()) {
      const Road& r = roads[size_t(chance(0, double(roads.size()) - 0.01))];
      const RoadPoint& p = r.pts[size_t(chance(0, double(r.pts.size()) - 0.01))];
      hint_ = V3{p.x, p.y, p.z};
      return hint_;
    }
  }
  return d.world.base(1 - Duel::teamOf(me)).centre;
}

// ---- Places to lie in wait.

// Beside a road, among trees or by a house, looking along it. Near the road for a reason: every metre of grass on the
// way in stays pressed flat and points at it, so the shorter that bit is the better, and hard ground is better still.
bool Bot::findPost(const Duel& d, int me, V3 from, bool high) {
  const World& w = d.world;
  const auto& roads = w.roads();
  if (roads.empty()) return false;
  const double side = from.x - World::riverAt(from.y);
  const V3 enemy = w.base(1 - Duel::teamOf(me)).centre;
  double best = -1e30;
  bool found = false;
  for (int tries = 0; tries < 120; tries++) {
    const Road& r = roads[size_t(chance(0, double(roads.size()) - 0.01))];
    if (r.pts.size() < 80) continue;
    const size_t k = size_t(chance(30.0, double(r.pts.size()) - 31.0));
    const RoadPoint& a = r.pts[k];
    const RoadPoint& b = r.pts[k + 3];
    const V3 onRoad{a.x, a.y, a.z};
    if (w.overFallenSpan(a.x, a.y)) continue;
    const V3 along = norm(flat(V3{b.x - a.x, b.y - a.y, 0}));
    const V3 across{-along.y, along.x, 0};
    const double off = chance(high ? 15.0 : 12.0, high ? 90.0 : 40.0) * (chance(0, 1) < 0.5 ? -1.0 : 1.0);
    V3 q = onRoad + across * off;
    if (std::fabs(q.x) > World::kArenaLimit - 40.0 || std::fabs(q.y) > World::kArenaLimit - 40.0) continue;
    if ((q.x - World::riverAt(q.y)) * side < 0) continue;
    const double far = len(flat(q - from));
    if (far > (high ? 420.0 : 650.0) || far < 30.0) continue;
    q.z = w.height(q.x, q.y);
    if (w.terrainNormal(q.x, q.y).z < (high ? 0.78 : 0.86) || w.isWater(q.x, q.y, q.z)) continue;
    bool room = true;
    for (double dx : {-4.0, 4.0})
      for (double dy : {-4.0, 4.0}) {
        Contact c;
        if (w.contact(V3{q.x + dx, q.y + dy, w.height(q.x + dx, q.y + dy) + 1.2}, c)) room = false;
      }
    if (!room) continue;
    // A bit of road towards them for the gun to look at, and it has to be able to.
    const size_t lookAt = enemy.x > a.x ? (a.x < r.pts[k + 25].x ? k + 25 : k - 25) : (a.x > r.pts[k + 25].x ? k + 25 : k - 25);
    const V3 look{r.pts[lookAt].x, r.pts[lookAt].y, r.pts[lookAt].z + 1.5};
    const V3 eye = q + V3{0, 0, 2.2};
    bool sees = true;
    const double lookDist = len(look - eye);
    for (double s = 3.0; s < lookDist - 2.0 && sees; s += 5.0) {
      Contact c;
      if (w.contact(eye + (look - eye) * (s / lookDist), c)) sees = false;
    }
    int cover = std::min(treesAround(w, q, 16.0), 3);
    for (const World::House& h : w.houses())
      if (std::hypot(h.x - q.x, h.y - q.y) < 22.0) cover = std::min(cover + 1, 3);
    // What the way in across the grass would give away.
    double grass = 0;
    for (double s = 0; s < std::fabs(off); s += 4.0) {
      const V3 p = onRoad + across * (s * (off > 0 ? 1.0 : -1.0));
      if (w.surfaceAt(p.x, p.y) == SURF_GRASS) grass += 4.0;
    }
    double score = double(cover) * 2.0 - grass * 0.05 + chance(0, 2.5) + (sees ? 2.0 : -3.0);
    if (high) score += (q.z - a.z) * 0.3;
    // The bridges are where they have to come through.
    for (const World::Bridge& br : w.bridgeSpans())
      if (len2(flat(br.at - q)) < 180.0 * 180.0) score += 1.5;
    if (score > best) {
      best = score;
      post_ = q;
      watch_ = look;
      found = true;
    }
  }
  if (found) {
    hasPost_ = true;
    atPost_ = false;
    shotsFromPost_ = 0;
    pathAge_ = 1e9;
  }
  return found;
}

// ---- Crates.

int Bot::pickCrate(const Duel& d, V3 from, bool ammunitionOnly) const {
  int best = -1;
  double bestD = 1e30;
  for (size_t i = 0; i < d.crates.size(); i++) {
    const Duel::Crate& c = d.crates[i];
    if (d.crateGone(c) || (i < crateTried_.size() && crateTried_[i])) continue;
    if (ammunitionOnly && !c.wreckSite) continue;
    if ((c.p.x - World::riverAt(c.p.y)) * (from.x - World::riverAt(from.y)) < 0) continue;  // not worth the crossing
    const double dist = len2(flat(c.p - from));
    if (dist < bestD) {
      bestD = dist;
      best = int(i);
    }
  }
  return best;
}

// What this way of playing still lacks, and what anyone would go and look for.
bool Bot::wantsLoot(const Duel& d, int me) const {
  const Duel::Player& pl = d.players[me];
  if (tactic_ == SCAVENGE && cratesTaken_ < cratesWanted_) return true;
  if (tactic_ == SNIPER && (!pl.hasWeapon[WP_SNIPER] || pl.sniperAmmo == 0)) return true;
  if (seesFoe_) return false;
  if (pl.soldier.hp < 55.0 && pl.medkits == 0) return true;
  if (pl.vehicle < 0 && pl.smallAmmo == 0 && pl.rpgAmmo == 0) return true;
  return false;
}

// A crate in a house means going round to the door. The houses are not on the map of the land.
void Bot::planToCrate(const Duel& d, V3 from, V3 at) {
  for (const World::House& h : d.world.houses()) {
    const double c = std::cos(h.yaw), s = std::sin(h.yaw);
    const double u = (at.x - h.x) * c + (at.y - h.y) * s, v = -(at.x - h.x) * s + (at.y - h.y) * c;
    if (u < 0 || u > World::kHouseLength || v < 0 || v > World::kHouseDepth) continue;
    const double fv = -(from.x - h.x) * s + (from.y - h.y) * c, fu = (from.x - h.x) * c + (from.y - h.y) * s;
    if (fu > 0 && fu < World::kHouseLength && fv > 0.3 && fv < World::kHouseDepth) break;  // in already
    const double z = at.z;
    // In front of the door, and from whichever side, around the corner first if one comes from behind.
    const V3 before = World::houseInside(h, World::kHouseLength * 0.5, -3.5, z);
    planTo(d, from, before);
    if (fv > 0) {
      path_.clear();
      pathAt_ = 0;
      const double outU = fu > World::kHouseLength * 0.5 ? World::kHouseLength + 3.0 : -3.0;
      path_.push_back(World::houseInside(h, outU, std::max(fv, 1.0), z));
      path_.push_back(World::houseInside(h, outU, -3.5, z));
      path_.push_back(before);
    }
    path_.push_back(World::houseInside(h, World::kHouseLength * 0.5, 1.0, z));
    path_.push_back(World::houseInside(h, World::kHouseLength * 0.5, 3.5, z));
    path_.push_back(at);
    pathGoal_ = at;
    return;
  }
  planTo(d, from, at);
}

// On foot, and something within walking distance. False if there is nothing to go for, or it is too far to walk.
bool Bot::lootOnFoot(Duel& d, int me) {
  const Soldier& s = d.players[me].soldier;
  crateClock_ += kThinkEvery;
  if (crate_ >= 0 && (size_t(crate_) >= d.crates.size() || d.crateGone(d.crates[size_t(crate_)]) || crateClock_ > 110.0)) {
    if (size_t(crate_) < crateTried_.size()) crateTried_[size_t(crate_)] = 1;
    crate_ = -1;
  }
  if (crate_ < 0) {
    crate_ = pickCrate(d, s.p, false);
    crateClock_ = 0;
    pathAge_ = 1e9;
    if (crate_ < 0) return false;
  }
  const V3 at = d.crates[size_t(crate_)].p;
  if (len2(flat(at - s.p)) > kWalkToCrate * kWalkToCrate) {
    // Too far on foot: this one is for the tank to go to, if there is a tank.
    crate_ = -1;
    return false;
  }
  doing_ = "pluendert";
  if (pathAge_ > 12.0) planToCrate(d, s.p, at);
  walkTowards(d, me, nextPoint(s.p, 2.0), true);
  return true;
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

// ---- Which tank.

bool Bot::goodTank(const Duel& d, int v) const {
  const Tank& t = d.tanks[v];
  if (t.destroyed || givenUp_[v] || d.occupied(v) || t.ammo == 0 || t.mod[TM_GUN] <= 0) return false;
  if (t.kind == TK_ARTILLERY) return false;  // a howitzer wants a different head
  return !t.trackOff[0] && !t.trackOff[1] && t.mod[TM_ENGINE] > 0 && t.fuel > 0;
}

void Bot::chooseTank(const Duel& d, int me, V3 from) {
  const int wasWanted = wantVehicle_;
  wantClock_ += kThinkEvery;
  // Can't get to it in all this time: something is in the way.
  if (wantVehicle_ >= 0 && wantClock_ > 70.0) givenUp_[wantVehicle_] = true;
  if (wantVehicle_ >= 0 && wantVehicle_ < Duel::kTanks && goodTank(d, wantVehicle_)) return;
  wantVehicle_ = -1;
  const TurretKind wantTurret = tactic_ == MISSILE ? TK_MISSILE : TK_STANDARD;
  const HullKind wantHull = tactic_ == FLANK || tactic_ == SPY || tactic_ == SCAVENGE || tactic_ == SNIPER ? HK_FAST : (tactic_ == AMBUSH ? HK_HEAVY : ((me & 1) ? HK_HEAVY : HK_MEDIUM));
  double best = 1200.0 * 1200.0 * 40.0;
  for (int v = 0; v < Duel::kTanks; v++) {
    if (!goodTank(d, v)) continue;
    const Tank& t = d.tanks[v];
    double dist = std::max(len2(d.frames[v].cog - from), 100.0);
    // Each bot leans to another one, and the front row is the easy way out of the yard.
    dist *= 1.0 + double((v + me * 5) % 7) * 0.08;
    if (t.kind != wantTurret) dist *= t.kind == TK_LIGHT ? 30.0 : 3.0;
    if (t.hull != wantHull) dist *= 1.6;
    if (dist < best) best = dist, wantVehicle_ = v;
  }
  if (wantVehicle_ != wasWanted) {
    wantClock_ = 0;
    pathAge_ = 1e9;
  }
}

void Bot::bail(int v) {
  givenUp_[v] = true;
  pulse_.use = true;
  useClock_ = 0.8;
  wantVehicle_ = -1;
  repairing_ = false;
  stuckCount_ = 0;
}

// ---- On foot.

void Bot::walkTowards(const Duel& d, int me, V3 at, bool sprint) {
  const Soldier& s = d.players[me].soldier;
  // Houses are not on the map of the land. Inside one, and going elsewhere: out by the door, the only way there is.
  const auto local = [](const World::House& h, V3 p, double& u, double& v) {
    const double c = std::cos(h.yaw), sn = std::sin(h.yaw);
    u = (p.x - h.x) * c + (p.y - h.y) * sn;
    v = -(p.x - h.x) * sn + (p.y - h.y) * c;
  };
  const auto within = [](double u, double v, double margin) { return u > -margin && u < World::kHouseLength + margin && v > -margin && v < World::kHouseDepth + margin; };
  for (const World::House& h : d.world.houses()) {
    double u, v, au, av;
    local(h, s.p, u, v);
    if (!within(u, v, 0.0)) continue;
    local(h, at, au, av);
    if (within(au, av, 0.0) || (std::fabs(au - World::kHouseLength * 0.5) < 1.0 && av > -5.0 && av < 0.0)) break;  // staying in, or on the way in
    at = World::houseInside(h, World::kHouseLength * 0.5, std::fabs(u - World::kHouseLength * 0.5) < 0.9 ? -3.0 : 1.0, s.p.z);
    break;
  }
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
  // Around whatever stands right in the way and is not what it is walking to: a house, or a parked tank.
  if (dist > 0.5) {
    const V3 ahead = to / dist;
    const V3 leftOf{-ahead.y, ahead.x, 0};
    const V3 probe = s.p + ahead * 3.0;
    const auto sideStep = [&](V3 mid) { held_.walk.strafe = dot(flat(mid - s.p), leftOf) > 0 ? -1.0 : 1.0; };
    for (const World::House& h : d.world.houses()) {
      double u, v, au, av;
      local(h, probe, u, v);
      if (!within(u, v, 0.8)) continue;
      local(h, at, au, av);
      if (within(au, av, 1.0) || (std::fabs(au - World::kHouseLength * 0.5) < 1.5 && av > -5.0 && av < 0.0)) break;
      sideStep(World::houseInside(h, World::kHouseLength * 0.5, World::kHouseDepth * 0.5, 0));
      break;
    }
    for (int v = 0; v < Duel::kVehicles; v++) {
      const VehicleFrame& f = d.frames[v];
      const V3 q = probe - f.cog;
      if (std::fabs(q.z) > 3.0 || std::fabs(dot(q, f.fwd)) > f.halfLength + 0.8 || std::fabs(dot(q, f.left)) > f.halfWidth + 0.8) continue;
      if (len2(flat(at - f.cog)) < (f.halfLength + 1.5) * (f.halfLength + 1.5)) continue;
      sideStep(f.cog);
      break;
    }
  }
}

void Bot::onFoot(Duel& d, int me) {
  Duel::Player& pl = d.players[me];
  const Soldier& s = pl.soldier;
  doing_ = "zu Fuss";
  // What lies within reach gets taken, on the way past or not.
  const int nearBy = d.nearCrate(s.p + V3{0, 0, 0.3});
  if (useClock_ <= 0 && nearBy >= 0) {
    pulse_.use = true;
    useClock_ = 0.6;
    if (size_t(nearBy) < crateTried_.size()) crateTried_[size_t(nearBy)] = 1;
    cratesTaken_++;
    cratesAllTime_++;
    if (crate_ == nearBy) crate_ = -1;
    pathAge_ = 1e9;
    return;
  }

  // Finishing a repair comes first, unless someone is shooting at close range.
  if (repairing_ && !(seesFoe_ && len(foeAt_ - s.p) < 60.0)) {
    repairClock_ += kThinkEvery;
    if (repairTank_ < 0 || d.vehicleDead(repairTank_) || pl.items[IT_REPAIR] == 0 || repairClock_ > 25.0) {
      if (repairTank_ >= 0 && repairTank_ < Duel::kTanks && repairClock_ > 25.0) givenUp_[repairTank_] = true;
      repairing_ = false;
    } else if (pl.repairClock > 0) {
      doing_ = "repariert";
      return;  // standing at it
    } else {
      const Tank& t = d.tanks[repairTank_];
      const bool broken = t.trackOff[0] || t.trackOff[1] || t.mod[TM_ENGINE] <= 0 || t.mod[TM_GUN] <= 0;
      if (broken) {
        doing_ = "repariert";
        if (d.nearestToRepair(s.p) == repairTank_) pulse_.repair = true;
        else walkTowards(d, me, d.frames[repairTank_].cog, false);
        return;
      }
      repairing_ = false;
      wantVehicle_ = repairTank_;
      wantClock_ = 0;
    }
  }

  const bool foeOnFoot = seesFoe_ && d.players[foe_].vehicle < 0;
  const bool foeInTank = seesFoe_ && d.players[foe_].vehicle >= 0;
  const bool hasRifle = pl.hasWeapon[WP_SNIPER] && pl.sniperAmmo > 0;
  const double foeDist = seesFoe_ ? len(foeAt_ - Duel::eyeOf(s)) : 1e9;
  int gun = pl.hasWeapon[WP_AR] && pl.smallAmmo > 0 ? WP_AR : (pl.smallAmmo > 0 ? WP_PISTOL : WP_KNIFE);
  if (foeInTank && pl.hasWeapon[WP_RPG] && pl.rpgAmmo > 0) gun = WP_RPG;
  else if (hasRifle && (foeDist > 35.0 || !seesFoe_)) gun = WP_SNIPER;
  held_.weapon = gun;

  if (seesFoe_ && (foeOnFoot || gun == WP_RPG || gun == WP_SNIPER)) {
    const V3 eye = Duel::eyeOf(s);
    const double dist = len(foeAt_ - eye);
    const double reach = gun == WP_RPG ? 180.0 : (gun == WP_AR ? 220.0 : (gun == WP_SNIPER ? 470.0 : 90.0));
    if (dist < reach && !(gun == WP_KNIFE && dist > 2.0)) {
      // Standing, along the sights, and in bursts. Against a tank the big rifle goes for the tracks.
      V3 target = foeAt_;
      if (gun == WP_SNIPER && foeInTank) {
        const VehicleFrame& f = d.frames[d.players[foe_].vehicle];
        target = f.cog - f.up * 0.75 + f.left * (dot(eye - f.cog, f.left) > 0 ? 1.35 : -1.35);
      }
      V3 aim = target + foeVel_ * (dist / std::max(kWeapons[gun].speed, 100.0)) + aimOff_;
      const double flight = dist / std::max(kWeapons[gun].speed, 100.0);
      if (gun != WP_RPG) aim.z += 4.9 * flight * flight;
      const V3 to = aim - eye;
      held_.walk.yaw = std::atan2(-to.y, -to.x);
      held_.pitch = std::asin(clampd(-to.z / std::max(len(to), 1e-6), -1, 1));
      held_.c.aimPoint = aim;
      held_.aiming = dist > 18.0;
      held_.c.brake = 0;
      if (seenFor_ > kReaction * 0.7 * reactSkill_) {
        if (shotClock_ <= 0) {
          const double spread = (dist * kAimError * 1.4 + 0.15) * aimSkill_;
          aimOff_ = V3{chance(-spread, spread), chance(-spread, spread), chance(-spread, spread) * 0.6};
          pulse_.fireFoot = true;
          shotClock_ = gun == WP_AR ? 0.14 : kWeapons[gun].interval + 0.1;
          if (gun == WP_AR && chance(0, 1) < 0.2) shotClock_ = 0.6;
          if (gun == WP_SNIPER) shotsFromPost_++;
        }
        held_.fireHeld = gun == WP_AR && shotClock_ < 0.2;
      }
      if (gun == WP_KNIFE) walkTowards(d, me, foeAt_, true);
      return;
    }
  }

  // The plans that are carried out on foot.
  if (tactic_ == SPY && !pl.tapped) {
    const World::Base& theirs = d.world.base(1 - Duel::teamOf(me));
    // That far is a job for a tank, as long as there is one.
    const bool close = len2(flat(theirs.door - s.p)) < 60.0 * 60.0;
    if (!close) chooseTank(d, me, s.p);
    if (!close && wantVehicle_ < 0) {
      doing_ = "zum Rechner";
      if (pathAge_ > 10.0) planTo(d, s.p, theirs.door);
      walkTowards(d, me, nextPoint(s.p, 3.0), true);
      return;
    }
    if (close) {
      doing_ = "zum Rechner";
      if (pl.tapClock > 0) return;  // at the keyboard
      if (d.atEnemyComputer(me, s.p)) {
        if (useClock_ <= 0) {
          pulse_.use = true;
          useClock_ = 0.5;
        }
        return;
      }
      // Round the blast wall in front of the door, in through the door, and to the back of the room.
      const V3 out = norm(flat(theirs.door - theirs.computer));
      const V3 side{-out.y, out.x, 0};
      const double sideOf = dot(flat(s.p - theirs.door), side) >= 0 ? 1.0 : -1.0;
      const V3 steps[3] = {theirs.door + side * (sideOf * 6.0) + out * 0.3, theirs.door - out * 3.5, theirs.computer + out * 1.2};
      if (spyStage_ < 3 && len2(flat(steps[spyStage_] - s.p)) < 1.6 * 1.6) spyStage_++;
      walkTowards(d, me, spyStage_ == 0 ? steps[0] : (spyStage_ == 1 ? steps[1] : steps[2]), spyStage_ == 0);
      if (spyStage_ == 0 && len2(flat(theirs.door - s.p)) < 2.0 * 2.0) spyStage_ = 1;
      return;
    }
  }
  if (tactic_ == SNIPER && hasRifle) {
    if (!hasPost_ && !findPost(d, me, s.p, true)) {
      tactic_ = STORM;
    } else {
      if (shotsFromPost_ >= 5) {
        // That is enough from here.
        hasPost_ = false;
        if (pl.wantProne) pulse_.setup = true;
        return;
      }
      const double left = len(flat(post_ - s.p));
      if (left > 3.0 && !atPost_) {
        doing_ = "zum Posten";
        if (pl.wantProne) pulse_.setup = true;
        if (pathAge_ > 8.0) planTo(d, s.p, post_);
        walkTowards(d, me, nextPoint(s.p, 3.0), left > 30.0);
        return;
      }
      doing_ = "auf dem Posten";
      atPost_ = true;
      const V3 to = watch_ - Duel::eyeOf(s);
      held_.walk.yaw = std::atan2(-to.y, -to.x);
      if (!pl.wantProne && pl.weapon == WP_SNIPER) pulse_.setup = true;
      return;
    }
  }
  // Got out for a crate, or wants one: either way, to it, and no getting back in first.
  if ((crate_ >= 0 || wantsLoot(d, me)) && lootOnFoot(d, me)) return;

  chooseTank(d, me, s.p);
  if (wantVehicle_ >= 0) {
    doing_ = "zum Panzer";
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
  // Nothing left to drive. What is lying about may yet help: a launcher, or at least something to shoot with.
  if (!pl.hasWeapon[WP_RPG] && lootOnFoot(d, me)) return;
  doing_ = "jagt zu Fuss";
  if (!seesFoe_) lookForTracks(d, s.p);
  const V3 goal = goalFor(d, me, s.p);
  if (pathAge_ > 10.0 || len(flat(goal - pathGoal_)) > 50.0 || d.world.bridgeFalls() != fallsSeen_) planTo(d, s.p, goal);
  walkTowards(d, me, nextPoint(s.p, 3.0), true);
}

// ---- In a tank.

void Bot::driveTowards(const Duel& d, int me, V3 at, double wantSpeed) {
  const int mine = d.players[me].vehicle;
  const VehicleFrame& f = d.frames[mine];
  const V3 to = flat(at - f.cog);
  double err = std::atan2(dot(to, f.left), dot(to, f.fwd));
  // Around what stands in the way: other vehicles are not on the map of the roads.
  for (int v = 0; v < Duel::kVehicles; v++) {
    if (v == mine) continue;
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
  // Still rolling backwards from getting unstuck: stop that first, or the throttle only makes it worse.
  if (speed < -1.0) {
    held_.c.throttle = 0;
    held_.c.brake = 1;
  }

  // Asking it to move, and it neither goes anywhere nor turns: back off, the other way each time, and in the end give up
  // on the thing. Whatever keeps it there, a blast's crater or a wall.
  stuckClock_ += kThinkEvery;
  if (stuckClock_ > 3.0) {
    // Wading is slow going, and that is not the same as being stuck.
    const double least = d.world.isWater(f.cog.x, f.cog.y, f.cog.z) ? 0.3 : 1.5;
    const double heading = std::atan2(f.fwd.y, f.fwd.x);
    const double moved = len(flat(f.cog - stuckAt_));
    const double turned = std::fabs(std::remainder(heading - stuckHeading_, 2.0 * PI));
    if (moved < least && turned < 0.3 && wantSpeed > 1.0 && unstickClock_ <= 0) {
      stuckCount_++;
      unstickClock_ = 3.0;
      unstickSteer_ = stuckCount_ % 2 ? -0.8 : 0.8;
      pathAge_ = 1e9;
    } else if (moved > 6.0) {
      stuckCount_ = 0;
    }
    stuckAt_ = f.cog;
    stuckHeading_ = heading;
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
  (void)d;
  (void)me;
  const double dist = len(foeAt_ - from);
  const double flight = dist / shellSpeed;
  held_.c.aimPoint = foeAt_ + foeVel_ * flight + aimOff_;
}

void Bot::fightInTank(Duel& d, int me) {
  const Duel::Player& pl = d.players[me];
  const Tank& t = d.tanks[pl.vehicle];
  const VehicleFrame& f = d.frames[pl.vehicle];
  const bool gunGone = t.mod[TM_GUN] <= 0;
  // A missile on its way goes where the eye is on.
  if (t.missileLive) {
    if (seesFoe_) held_.c.aimPoint = foeAt_ + foeVel_ * 0.3;
    else if (lastSeenAge_ < 12.0) held_.c.aimPoint = lastSeen_;
    return;
  }
  if (seesFoe_) {
    aimAt(d, me, t.spec().shellSpeed, t.muzzle);
    const double dist = len(foeAt_ - t.muzzle);
    // Lying in wait: nothing that gives it away until they are well inside.
    const bool holdFire = tactic_ == AMBUSH && atPost_ && dist > kAmbushRange && shotsFromPost_ == 0;
    const V3 wanted = norm(flat(held_.c.aimPoint - t.muzzle));
    const V3 lay = norm(flat(t.gunDir));
    const double off = std::acos(clampd(dot(wanted, lay), -1.0, 1.0));
    const bool ready = seenFor_ > kReaction * reactSkill_ && !holdFire && shotClock_ <= 0;
    if (ready && t.missiles > 0 && dist > 90.0) {
      pulse_.c.launch = true;
      shotClock_ = 4.0;
    } else if (ready && off < 0.012 && t.reload <= 0 && t.ammo != 0 && !gunGone) {
      pulse_.c.fire = true;
      shotClock_ = 0.4;
      if (atPost_) shotsFromPost_++;
      const double spread = (dist * kAimError + 0.2) * aimSkill_;
      aimOff_ = V3{chance(-spread, spread), chance(-spread, spread), chance(-spread, spread) * 0.5};
    }
  } else if (lastSeenAge_ < 12.0) {
    held_.c.aimPoint = lastSeen_;
  } else if (atPost_) {
    held_.c.aimPoint = watch_;
  } else {
    held_.c.aimPoint = f.cog + f.fwd * 60.0 + V3{0, 0, 2.0};
  }
}

void Bot::inTank(Duel& d, int me) {
  Duel::Player& pl = d.players[me];
  const int v = pl.vehicle;
  const Tank& t = d.tanks[v];
  const VehicleFrame& f = d.frames[v];
  const bool driver = pl.seat == 0;
  const bool cantDrive = t.trackOff[0] || t.trackOff[1] || t.mod[TM_ENGINE] <= 0 || t.fuel <= 0;
  const bool gunGone = t.mod[TM_GUN] <= 0;
  const double foeDist = seesFoe_ ? len(foeAt_ - f.cog) : 1e9;
  const bool stopped = std::fabs(f.speed) < 1.5;
  doing_ = "im Panzer";

  fightInTank(d, me);
  if (useClock_ > 0) return;

  // Upside down or on its side: that one is finished.
  flippedClock_ = f.up.z < 0.35 ? flippedClock_ + kThinkEvery : 0;
  if (flippedClock_ > 3.0) {
    bail(v);
    return;
  }
  // The driver is gone, and the gunner has no pedals: out, and in again at the front.
  if (!driver && d.seatHolder(v, 0) < 0 && stopped) {
    pulse_.use = true;
    useClock_ = 0.8;
    wantVehicle_ = v;
    return;
  }

  // What a kit can put right is worth getting out for, once nobody is close. An empty tank it cannot fill.
  const bool mendable = (t.trackOff[0] || t.trackOff[1] || t.mod[TM_ENGINE] <= 0 || gunGone) && t.fuel > 0 && pl.items[IT_REPAIR] > 0;
  brokenClock_ = cantDrive || gunGone ? brokenClock_ + kThinkEvery : 0;
  if (mendable && stopped && (foeDist > 120.0 || brokenClock_ > 14.0)) {
    repairing_ = true;
    repairTank_ = v;
    repairClock_ = 0;
    wantVehicle_ = -1;
    pulse_.use = true;
    useClock_ = 0.8;
    return;
  }
  // Stuck for good and nothing to mend it with. As long as the gun has someone in its sights it can stay a little; then
  // out, and on to something else. Sitting in it only waits for the next shot.
  if (!mendable && (cantDrive || gunGone) && stopped && (!seesFoe_ || gunGone || brokenClock_ > 8.0)) {
    bail(v);
    return;
  }
  // As good as gone, with them right there: better on foot than in it when it goes up.
  if (t.damage > 0.82 && seesFoe_ && foeDist < 200.0 && stopped && !t.missileLive) {
    bail(v);
    return;
  }
  if (!driver || cantDrive) return;
  if (t.ammo == 0 && t.missiles == 0 && !seesFoe_) {
    // The old wrecks have shells in them, as anyone knows who has been out here before.
    if (crate_ < 0) {
      crate_ = pickCrate(d, f.cog, true);
      crateClock_ = 0;
      pathAge_ = 1e9;
    }
    if (crate_ < 0) {
      bail(v);
      return;
    }
  }

  // A shell has just landed: moving, and not along the line it came on.
  if (dodgeClock_ > 0) {
    doing_ = "weicht aus";
    const V3 away = seesFoe_ || lastSeenAge_ < 10.0 ? norm(flat(f.cog - (seesFoe_ ? foeAt_ : lastSeen_))) : f.fwd * -1.0;
    const V3 across{-away.y, away.x, 0};
    driveTowards(d, me, f.cog + (across * dodgeSide_ + away * 0.5) * 40.0, 8.0);
    return;
  }

  const auto standAndFace = [&](V3 at) {
    // Stand, and turn the thick end to them. With the turret stuck, the whole tank has to be the gun.
    const V3 to = flat(at - f.cog);
    const V3 ref = t.mod[TM_RING] <= 0 ? flat(t.gunDir) : f.fwd;
    const V3 refLeft{-ref.y, ref.x, 0};
    const double err = std::atan2(dot(to, refLeft), dot(to, ref));
    held_.c.throttle = 0;
    held_.c.brake = std::fabs(f.speed) > 0.5 ? 1.0 : 0.0;
    const double dead = t.mod[TM_RING] <= 0 ? 0.01 : 0.3;
    held_.c.steer = std::fabs(err) > dead ? clampd(err * 1.5, -0.8, 0.8) : 0.0;
    stuckClock_ = 0;
    stuckAt_ = f.cog;
    stuckHeading_ = std::atan2(f.fwd.y, f.fwd.x);
  };

  // Stuck over and over, whatever it tries: this tank is not going anywhere any more.
  if (stuckCount_ >= 5 && stopped) {
    bail(v);
    return;
  }

  // Going for a crate: as close as a tank gets, and the rest on foot.
  if (crate_ < 0 && !seesFoe_ && wantsLoot(d, me)) {
    crate_ = pickCrate(d, f.cog, false);
    crateClock_ = 0;
    pathAge_ = 1e9;
    // Nothing more to find on this side: the plan is off.
    if (crate_ < 0 && (tactic_ == SCAVENGE || tactic_ == SNIPER)) {
      tactic_ = STORM;
      hasPost_ = false;
    }
  }
  if (crate_ >= 0 && !seesFoe_) {
    crateClock_ += kThinkEvery;
    const bool gone = size_t(crate_) >= d.crates.size() || d.crateGone(d.crates[size_t(crate_)]) || crateClock_ > 90.0;
    if (gone) {
      if (size_t(crate_) < crateTried_.size()) crateTried_[size_t(crate_)] = 1;
      crate_ = -1;
      return;
    }
    doing_ = "faehrt zur Kiste";
    const V3 at = d.crates[size_t(crate_)].p;
    if (len2(flat(at - f.cog)) < 16.0 * 16.0 || stuckCount_ >= 2) {
      if (stopped) {
        // Parked, not given up on: it wants this one back.
        pulse_.use = true;
        useClock_ = 0.8;
        wantVehicle_ = v;
        wantClock_ = 0;
        stuckCount_ = 0;
        pathAge_ = 1e9;
      } else {
        held_.c.brake = 1;
        held_.c.throttle = 0;
      }
      return;
    }
    if (pathAge_ > 20.0 || len(flat(at - pathGoal_)) > 20.0 || d.world.bridgeFalls() != fallsSeen_) planTo(d, f.cog, at);
    driveTowards(d, me, nextPoint(f.cog, 13.0), len(flat(at - f.cog)) < 40.0 ? 4.0 : 13.0);
    return;
  }

  // The rifle in hand: the tank has done its part once it is near the post.
  if (tactic_ == SNIPER && pl.hasWeapon[WP_SNIPER] && pl.sniperAmmo > 0) {
    if (!hasPost_ && !findPost(d, me, f.cog, true)) {
      tactic_ = STORM;
    } else if (hasPost_) {
      doing_ = "zum Posten";
      if (len2(flat(post_ - f.cog)) < 30.0 * 30.0 || stuckCount_ >= 2) {
        if (stopped) {
          pulse_.use = true;
          useClock_ = 0.8;
          wantVehicle_ = -1;
        } else {
          held_.c.brake = 1;
        }
        return;
      }
      if (pathAge_ > 20.0 || d.world.bridgeFalls() != fallsSeen_) planTo(d, f.cog, post_);
      driveTowards(d, me, nextPoint(f.cog, 13.0), 13.0);
      return;
    }
  }

  // To their yard, and in to their computer on foot.
  if (tactic_ == SPY && !pl.tapped) {
    const World::Base& theirs = d.world.base(1 - Duel::teamOf(me));
    const V3 outside = theirs.door + flat(f.cog - theirs.door) / std::max(len(flat(f.cog - theirs.door)), 1.0) * 25.0;
    if (len2(flat(theirs.door - f.cog)) < 40.0 * 40.0 || stuckCount_ >= 3) {
      if (stopped) {
        pulse_.use = true;
        useClock_ = 0.8;
        wantVehicle_ = v;
        wantClock_ = 0;
        stuckCount_ = 0;
      } else {
        held_.c.brake = 1;
      }
      return;
    }
    if (!seesFoe_ || foeDist > kHoldRange) {
      doing_ = "faehrt zu ihrem Rechner";
      if (pathAge_ > 20.0 || d.world.bridgeFalls() != fallsSeen_) planTo(d, f.cog, outside);
      driveTowards(d, me, nextPoint(f.cog, 13.0), 13.0);
      return;
    }
  }

  // Lying in wait.
  if (tactic_ == AMBUSH) {
    if (!hasPost_ && !findPost(d, me, f.cog, false)) tactic_ = STORM;
    // Found out, and they have seen where it fires from: somewhere else, or straight at them.
    if (hasPost_ && shotsFromPost_ >= 3) {
      if (chance(0, 1) < 0.5) {
        tactic_ = STORM;
        hasPost_ = atPost_ = false;
      } else if (!findPost(d, me, f.cog, false)) {
        tactic_ = STORM;
        hasPost_ = atPost_ = false;
      }
    }
    if (tactic_ == AMBUSH) {
      if (!atPost_ && len2(flat(post_ - f.cog)) < 7.0 * 7.0) atPost_ = true;
      if (atPost_) {
        doing_ = "lauert";
        standAndFace(seesFoe_ ? foeAt_ : watch_);
        if (stopped && !t.netOn && t.netClock <= 0 && pl.items[IT_NET] > 0 && std::fabs(f.speed) < 0.5) pulse_.net = true;
        return;
      }
      if (seesFoe_ && foeDist < kHoldRange) {
        standAndFace(foeAt_);
        return;
      }
      doing_ = "zum Versteck";
      if (pathAge_ > 25.0 || d.world.bridgeFalls() != fallsSeen_) planTo(d, f.cog, post_);
      const double left = len(flat(post_ - f.cog));
      driveTowards(d, me, nextPoint(f.cog, left < 30.0 ? 5.0 : 13.0), left < 40.0 ? 3.5 : 13.0);
      if (stuckCount_ >= 3) atPost_ = true;  // close enough
      return;
    }
  }

  // Round the side of them, to where the armour is thin.
  if (tactic_ == FLANK) {
    if (seesFoe_) {
      const Duel::Player& other = d.players[foe_];
      V3 spot = foeAt_;
      if (other.vehicle >= 0) spot = foeAt_ + d.frames[other.vehicle].left * (dot(f.cog - foeAt_, d.frames[other.vehicle].left) > 0 ? 110.0 : -110.0);
      if (foeDist < 90.0 || len2(flat(spot - f.cog)) < 40.0 * 40.0) {
        doing_ = "feuert von der Seite";
        standAndFace(foeAt_);
      } else {
        doing_ = "umgeht";
        if (pathAge_ > 6.0) planTo(d, f.cog, spot);
        driveTowards(d, me, nextPoint(f.cog, 13.0), 8.0);
      }
      return;
    }
    // A way round that does not get anywhere is no way round.
    if (!flanked_ && (stuckCount_ >= 2 || tacticClock_ > 120.0)) {
      flanked_ = true;
      pathAge_ = 1e9;
    }
    if (!flanked_) {
      const V3 ahead = goalFor(d, me, f.cog);
      const V3 line = flat(ahead - f.cog);
      const double far = len(line);
      // A wide way round, for as long as there is any distance to go round.
      if (far > 220.0) {
        const V3 unit = line / far;
        V3 wide = ahead - unit * 120.0 + V3{-unit.y, unit.x, 0} * (flankSide_ * 280.0);
        wide.x = clampd(wide.x, -World::kArenaLimit + 60.0, World::kArenaLimit - 60.0);
        wide.y = clampd(wide.y, -World::kArenaLimit + 60.0, World::kArenaLimit - 60.0);
        if (len2(flat(wide - f.cog)) > 60.0 * 60.0) {
          doing_ = "umgeht";
          if (pathAge_ > 30.0 || d.world.bridgeFalls() != fallsSeen_) planTo(d, f.cog, wide);
          driveTowards(d, me, nextPoint(f.cog, 13.0), 13.0);
          return;
        }
      }
      flanked_ = true;
    }
  }

  // Everyone else: find them, and get on with it.
  const double holdAt = tactic_ == MISSILE ? kSightTank : kHoldRange;
  // In sight and nothing lands, whatever the reason: a crest or a tree that the shells find and the eye does not.
  if (pl.hitsLanded != landedSeen_) {
    landedSeen_ = pl.hitsLanded;
    noEffect_ = 0;
  } else if (seesFoe_) {
    noEffect_ += kThinkEvery;
  }
  if (noEffect_ > 12.0) {
    noEffect_ = 0;
    moveClock_ = chance(6.0, 10.0);
    dodgeSide_ = chance(0, 1) < 0.5 ? -1.0 : 1.0;
  }
  if (moveClock_ > 0 && seesFoe_) {
    moveClock_ -= kThinkEvery;
    doing_ = "sucht besseres Schussfeld";
    const V3 to = flat(foeAt_ - f.cog);
    const V3 across = V3{-to.y, to.x, 0} / std::max(len(to), 1.0);
    driveTowards(d, me, foeAt_ + across * (dodgeSide_ * std::max(40.0, foeDist * 0.5)), 7.0);
    return;
  }
  if (seesFoe_ && foeDist < holdAt) {
    // Not forever on one spot, though.
    if (shiftClock_ <= 0 && !t.missileLive) {
      shiftClock_ = chance(7.0, 14.0);
      dodgeClock_ = chance(1.8, 3.2);
      dodgeSide_ = chance(0, 1) < 0.5 ? -1.0 : 1.0;
    }
    doing_ = "kaempft";
    standAndFace(foeAt_);
    return;
  }
  doing_ = "jagt";
  if (!seesFoe_ && lookForTracks(d, f.cog)) doing_ = "folgt einer Spur";
  const V3 goal = goalFor(d, me, f.cog);
  if (pathAge_ > 30.0 || len(flat(goal - pathGoal_)) > 150.0 || (onTrack_ && pathAge_ > 4.0) || d.world.bridgeFalls() != fallsSeen_) planTo(d, f.cog, goal);
  const double left = len(flat(goal - f.cog));
  driveTowards(d, me, nextPoint(f.cog, 13.0), left < 40.0 ? 4.0 : (seesFoe_ ? 7.0 : (onTrack_ ? 9.0 : 13.0)));
}
