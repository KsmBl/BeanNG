// One match, without any graphics. Each side's yard holds one tank with every turret and one of every car; the
// players choose a turret and what to carry, then play it out until one of the chosen tanks is gone, then the
// next match. The dedicated server steps this. A player's machine builds the same thing and only shows it.
#pragma once
#include <vector>
#include "sim.h"
#include "tank.h"
#include "vehicle.h"
#include "world.h"

enum ItemKind : u8 { IT_FOOT_MINE, IT_AT_MINE, IT_REPAIR, IT_AR, IT_SMALL_AMMO, IT_SMOKE, IT_GHILLIE, IT_NET, IT_PISTOL, IT_SNIPER, IT_SNIPER_AMMO, IT_COUNT };
struct ItemInfo {
  const char* name;
  int slots;
  bool once = false;  // one of it is all that makes sense: a box to tick, not a number
};
inline constexpr ItemInfo kItems[IT_COUNT] = {
    {"Tretmine", 2}, {"Panzermine", 5}, {"Reparaturset", 10}, {"Sturmgewehr", 2, true}, {"Kleinmunition (60 Schuss)", 1}, {"Nebelgranate", 1}, {"Ghillie-Anzug", 3, true}, {"Tarnnetz", 3},
    {"Pistole", 1, true}, {"Scharfschützengewehr", 5, true}, {"Scharfschützenmunition (20 Schuss)", 2},
};
inline constexpr int kLoadoutSlots = 20;
inline int slotsUsed(const u8 items[IT_COUNT]) {
  int n = 0;
  for (int i = 0; i < IT_COUNT; i++) n += int(items[i]) * kItems[i].slots;
  return n;
}

// The player when not inside anything: a point with a body drawn around it, moved kinematically.
struct Soldier {
  V3 p;     // the feet
  V3 v;
  double yaw = 0, pitch = 0;  // where the camera looks; yaw as the camera's, in the world
  double hp = 100;
  double stride = 0;          // how far the legs have walked, for drawing them
  double prone = 0;           // 0 standing .. 1 flat on the ground, behind the big rifle
  double crouch = 0;          // 0 standing .. 1 down on the haunches
  bool grounded = true;
  static constexpr double kRadius = 0.32, kHeight = 1.8;
};
// What is left when someone dies on foot. Worked out on each player's machine: it only has to look right.
struct Ragdoll {
  static constexpr int kJoints = 15;
  V3 p[kJoints], prev[kJoints];
  bool on = false;
  double clock = 0, debt = 0;
  int weapon = 0;
  V3 bloodAt;
};
struct WalkInput {
  double forward = 0, strafe = 0;  // -1..1; strafe positive to the left
  double yaw = 0;
  bool jump = false, sprint = false, crouch = false;
};

// Where a vehicle is, as far as those on foot are concerned. The same on the server and on a player's machine.
// How warm a vehicle is through a thermal sight, 0..1. Driving warms it up within a quarter of a minute. Everything else
// takes minutes to cool: left alone down to nothing, with someone sitting in it down to what an idling engine gives.
inline double warmthStep(double warm, bool occupied, bool moving, double dt) {
  const double want = occupied ? (moving ? 0.85 : 0.2) : 0.0;
  const double rate = want > warm ? 1.0 / 5.0 : 1.0 / 90.0;
  return warm + (want - warm) * std::min(dt * rate, 1.0);
}

struct VehicleFrame {
  V3 cog, fwd{1, 0, 0}, left{0, 1, 0}, up{0, 0, 1};
  double speed = 0;
  double halfLength = 2.3, halfWidth = 1.0, height = 1.6;
};

enum WeaponKind : u8 { WP_KNIFE, WP_PISTOL, WP_AR, WP_SNIPER, WP_RPG, WP_COUNT };
// The found sights travel with the weapons one has, in the same byte.
inline constexpr int kHasThermal = 1 << 6, kHasWeak = 1 << 7;
struct WeaponInfo {
  const char* name;
  double interval;  // seconds between shots
  bool automatic;
  double speed, damage, spread;  // m/s, health points, radians
};
inline constexpr WeaponInfo kWeapons[WP_COUNT] = {
    {"Kampfmesser", 0.6, false, 0, 60, 0},
    {"Pistole", 0.28, false, 360, 26, 0.006},
    {"Sturmgewehr", 0.1, true, 800, 28, 0.009},
    {"Scharfschützengewehr", 2.2, false, 950, 220, 0.0004},
    {"Raketenwerfer", 4.0, false, 130, 0, 0.012},  // loose at long range: it wanders as it flies
};

// How big a shot looks and sounds.
inline float muzzleSize(int weapon) { return weapon == WP_RPG ? 0.5f : (weapon == WP_SNIPER ? 1.0f : (weapon == WP_AR ? 0.14f : 0.1f)); }

// What a shell did on its way, for the kill cam to fly along with it.
struct ShellFlight {
  bool on = false;
  V3 from, at;
  double flight = 0;
};

// A helicopter is not a soft body like the rest: it flies by a few numbers, and is drawn from three rigid parts.
struct Heli {
  V3 p, v;  // p is on the ground between the skids
  double yaw = 0, pitch = 0, roll = 0;
  double power = 0;           // the rotor's speed, 0..1
  double spin = 0, tailSpin = 0;
  double gunSpin = 0;   // the barrels: only ever turned on the machines that draw them
  bool firing = false;  // and whether they have reason to
  // Those are what a bullet can harm; the armour of a helicopter is no armour at all.
  int engine = kEngineHits, rotor = kRotorHits, tail = kTailHits;
  int ammo = kGunAmmo;
  bool crashing = false, dead = false, onGround = true;
  double gunClock = 0, burn = 0, smokeClock = 0;
  int hitBy = -1;
  bool hitByMissile = false;
  u8 hitWeapon = KW_NONE;
  ShellFlight hitShot;
  static constexpr int kEngineHits = 6, kRotorHits = 6, kTailHits = 4, kGunAmmo = 200;
  int fullAmmo() const { return kGunAmmo; }
  static constexpr double kMuzzleAhead = 4.05, kMuzzleHigh = 0.58;
  static constexpr double kBoardReach = 5.5, kGunInterval = 0.05, kCameraBack = 17.0, kCameraUp = 5.5;
  V3 fwd() const {
    const double cy = std::cos(yaw), sy = std::sin(yaw), cp = std::cos(pitch), sp = std::sin(pitch);
    return V3{cy * cp, sy * cp, sp};
  }
  V3 left() const {
    const double cy = std::cos(yaw), sy = std::sin(yaw), cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch);
    return V3{-sy * cr + (-cy * sp) * sr, cy * cr + (-sy * sp) * sr, cp * sr};
  }
  V3 up() const { return cross(fwd(), left()); }
  V3 at(double ahead, double side, double high) const { return p + fwd() * ahead + left() * side + up() * high; }
};

class Duel {
 public:
  static constexpr double kDt = 1.0 / 2000.0;
  static constexpr double kEndPause = 8.0;    // seconds to look at the wreck before the next match
  static constexpr double kChooseTime = 60.0;  // whoever has not decided by then goes with what is selected
  static constexpr int kBuggies = 4;  // after the tanks: two to a side
  static constexpr int kTanks = 2 * HK_COUNT * TK_COUNT + kBuggies;
  static constexpr int kCarKinds = 3;
  static constexpr int kCars = 2 * kCarKinds;
  static constexpr double kMedkitHeal = 50.0;
  static constexpr int kMaxPlayers = 12;  // six a side at most; what stands in the yard stays the same
  static int teamOf(int player) { return player % 2; }
  static constexpr int kVehicles = kTanks + kCars;

  enum Phase : u8 { WAITING, CHOOSING, PLAYING, FINISHED };

  struct Player {
    bool present = false, ready = false;
    u8 items[IT_COUNT] = {};
    Soldier soldier;
    int vehicle = -1;  // what they sit in, -1 on foot
    int seat = 0;      // 0 drives, 1 is the gunner
    double walkDebt = 0;
    // A drone from a crate. While it flies, its owner stands there holding the controls.
    int drones = 0;
    int medkits = 0;
    bool droneLive = false;
    V3 droneP, droneV;
    double droneBattery = 0;
    V3 lastBlow;  // the last thing to hit them, for how they fall
    // For the screen: how many hits this player has landed, how many they have taken, and from where the last came.
    u16 hitsLanded = 0, hitsTaken = 0;
    V3 hurtFrom;
    // The enemy's main computer: a few seconds at it, and from then on it says roughly where each of them is.
    bool tapped = false;
    double tapClock = 0;
    double fixWait = 0;
    struct Fix {
      bool on = false;
      V3 at;
      double age = 0;
    };
    Fix fixes[kMaxPlayers];  // by player, from the computer or marked by a drone
    int weapon = WP_PISTOL;
    bool hasWeapon[WP_COUNT] = {true, true, false, false, false};
    int smallAmmo = 24;  // pistol and assault rifle: what one starts with, and what the loadout adds
    int sniperAmmo = 0, rpgAmmo = 0;
    bool thermalSight = false, weakSight = false;  // found: extra views through the rifle's scope
    int tankShells = 0, tankMissiles = 0;  // carried, and loaded when getting into a tank that takes them
    int strikeCodes = 0;                   // each calls down four rockets on a point of the map
    bool wantProne = false;
    double shotClock = 0;
    double repairClock = 0;  // working on a vehicle from outside, which takes a kit and being there
    int repairing = -1;
    int heli = -1;              // which helicopter they are flying
    bool fallFromHeli = false;  // stepped out in the air, and there is no parachute
    u32 lastInput = 0;  // the input the soldier has last been moved by, for the player's prediction
    u8 walkedSinceInput = 0;  // and how many steps of walking it has been since that one came in
  };
  // What a player does this step beyond the pedals of a vehicle.
  struct Intent {
    Controls c;
    WalkInput walk;
    double pitch = 0;
    bool use = false;  // E: get in, get out
    bool fireFoot = false;  // a press of the trigger
    bool fireHeld = false;
    bool aiming = false;    // along the sights: steadier, and slower on the feet
    bool placeFoot = false, placeTank = false;
    bool strike = false;    // a code used on the point marked on the map
    bool query = false;     // asks a tapped computer where the enemy is
    bool smoke = false;     // a grenade on foot, the launcher in a tank
    bool net = false;       // throws the camouflage net over the tank one sits in
    bool drone = false;     // sends a drone up, or calls it off
    bool medkit = false;    // uses a carried medkit
    V3 strikeAt;
    bool setup = false;     // lies down behind the big rifle, or gets up again
    int weapon = WP_PISTOL;
    double lag = 0;         // how far in the past this player sees the others, in seconds
    bool repair = false;
    u32 seq = 0;
  };

  World world;
  Sim sim, simStart;
  Tank tanks[kTanks], tanksStart[kTanks];
  std::vector<Vehicle> cars, carsStart;
  CarRig rigs[kVehicles];
  V3 spawn[kVehicles];
  double yaw[kVehicles] = {};
  u32 tankFirst[kTanks] = {}, tankNodes = 0, tanksEnd = 0, carsEnd = 0;
  u32 carFirst[kCars] = {};
  // Where the parts of each car lie, by the nodes that carry them.
  struct CarParts {
    std::vector<u32> engine, radiator, tank, gearbox;
  };
  CarParts carParts[kCars];
  VehicleFrame frames[kVehicles];
  // Which never touch anything and only get drawn: a soldier is not part of the soft body world. A small cube of
  // nodes to each rigid part of the body, as the mesh's binding wants, and each weapon counts as a part.
  static constexpr int kPartNodes = 8, kSoldierParts = 21, kSoldierNodes = kSoldierParts * kPartNodes;
  static const char* soldierPartName(int part);
  static const V3 kSoldierRest;  // where the bind pose stands
  u32 soldierFirst[kMaxPlayers] = {};
  size_t blastsDone = 0;
  // What flies from hand weapons. Hits are judged against where things were when the shooter fired, as they saw it.
  struct Bullet {
    V3 p, v;
    int owner = 0;
    int kind = WP_PISTOL;
    double age = 0, lag = 0;
    int by = -1;  // whose it is, for what flies with nobody as its owner
    // A thin line of smoke behind it, handed out in lengths as it flies: the gatling's. For the first second only.
    bool trail = false;
    bool gatling = false;  // a 20 mm round: more to a car than any rifle's
    bool mg = false;       // from a buggy's machine gun: as a rifle does, but the log says so
    V3 trailFrom;
    double trailClock = 0;
  };
  std::vector<Bullet> bullets;
  // Rockets ordered by a strike code, on their way. They come one after the other, each somewhere near the point.
  struct Incoming {
    V3 at;
    double in = 0;
    int by = -1;
  };
  std::vector<Incoming> incoming;
  static constexpr double kStrikeDelay = 6.0, kStrikeSpread = 38.0, kStrikeGap = 1.4;
  static constexpr int kKindStrike = WP_COUNT, kKindSmoke = WP_COUNT + 1;
  // Clouds of smoke: where they are, and how long they go on rising.
  struct SmokeCloud {
    V3 p;
    double left = 0, nextPuff = 0;
  };
  std::vector<SmokeCloud> smoke;
  static constexpr double kSmokeTime = 26.0;
  void startSmoke(V3 at);
  void useSmoke(int p);
  struct Past {
    V3 soldier[kMaxPlayers];
    V3 tank[kTanks];  // the middle of each: how far it has gone on since the shooter saw it
  };
  static constexpr int kPastSteps = 32;  // every hundredth of a second
  Past past[kPastSteps];
  int pastAt = 0;
  double bulletDebt = 0;
  u32 randState = 12345;
  // The wind of this match: at ground level, the same everywhere, and it carries what flies far enough.
  V3 wind;
  // Old wrecks lying about the map, in the same mesh as the rest, and crates in houses and beside them.
  static constexpr int kWrecks = 7;
  Tank wrecks[kWrecks], wrecksStart[kWrecks];
  u32 wreckFirst[kWrecks] = {};
  V3 wreckSpawn[kWrecks];
  // A net for every tank and wreck: eight nodes that carry it, underground unless it is on.
  u32 netFirst[kTanks + kWrecks] = {};
  static void poseNet(Sim& sim, u32 first, const Tank& t, bool on);
  // A helicopter in each yard.
  // One helicopter for each place to start from, and after them two buggies for each side.
  static constexpr int kFlying = 6, kHelis = kFlying, kHeliNodes = 32;  // three to a side, however many play
  double heliDebt[kHelis] = {};
  // Returns true if it has just hit the ground in a way it does not survive. No pilot, or none who may fly yet: null.
  static bool flyHeli(Heli& H, const World& world, const WalkInput* in, double dt);
  int heliToRepair(V3 p) const;
  static const V3 kHeliRest;  // where its nodes lie for the mesh to be bound
  Heli helis[kHelis], helisStart[kHelis];
  u32 heliFirst[kHelis] = {};
  static void poseHeli(Sim& sim, u32 first, const Heli& h);
  int heliPilot(int h) const {
    for (int i = 0; i < kMaxPlayers; i++)
      if (players[i].present && players[i].heli == h && players[i].soldier.hp > 0) return i;
    return -1;
  }
  int nearestHeli(V3 p) const;
  // A tarpaulin for every car, stretched to fit it: what it measures, from the ground, is worked out when it is built.
  static constexpr int kTarpNodes = 8;
  static const V3 kTarpRest;  // where the sheet lies out of sight, and the mesh is bound
  u32 tarpFirst[kCars] = {};
  V3 tarpSize[kCars];
  void poseTarp(Sim& sim, int car, bool on) const;
  CarRig wreckRigs[kWrecks];
  struct Crate {
    V3 p;
    u32 marker = 0;
    int what = 0, count = 1;
    bool wreckSite = false;
  };
  std::vector<Crate> crates;
  void rollLoot();  // what is in them this match: every machine works it out the same from the match's number
  static const char* crateLabel(const Crate& c, char* buf, size_t n);
  bool crateGone(const Crate& c) const { return sim.pos[c.marker].z < -100.0; }
  int nearCrate(V3 p) const;
  // Mines: anyone's foot sets off the small kind, anything with wheels or tracks the big one.
  static constexpr int kMinesEach = 10;
  struct Mine {
    bool live = false;
    V3 p;
    int owner = 0;
    double armed = 0;
  };
  Mine mines[2][kMinesEach];  // [0] foot, [1] anti-tank
  u32 mineFirst[2][kMinesEach] = {};
  bool giveAll = false;  // for trying things out: every weapon from the start
  // What can be handed out by name: from the server's shell, and what loot is made of.
  static constexpr int kGiveKinds = 19;
  static constexpr double kDroneBattery = 50.0, kDroneSpeed = 24.0, kDroneMarkRange = 300.0;
  static constexpr int kDroneNodes = 6;
  u32 droneFirst[kMaxPlayers] = {};
  static void poseDrone(Sim& sim, u32 first, bool live, V3 at, double yaw);
  void stepDrone(int p, const Intent& in, double dt);
  static const char* const kGiveNames[kGiveKinds];
  bool give(int player, const char* name, int count);
  static V3 eyeOf(const Soldier& s);

  Player players[kMaxPlayers];
  // The score, over all the matches of a server. A slot starts again when someone new takes it.
  struct Score {
    u16 kills = 0, deaths = 0;
  };
  Score scores[kMaxPlayers];
  // Who last hurt each player, and how long ago: it only counts as their kill if it is still fresh.
  static constexpr double kKillCredit = 30.0;
  int hurtBy[kMaxPlayers];  // set to nobody in build()
  double hurtByAge[kMaxPlayers] = {};
  bool hurtByMissile[kMaxPlayers] = {};  // with a guided missile: the kill cam flies with it
  bool carHitByMissile[kCars] = {};
  u8 carHitWeapon[kCars] = {};
  u8 hurtWeapon[kMaxPlayers] = {};
  // Who killed whom and with what, for the server to hand on to everyone. Whoever has none died of something else.
  struct KillNote {
    int killer, victim;
    u8 weapon;
  };
  std::vector<KillNote> killNotes;
  static u8 weaponOfBullet(int kind, bool gatling) { return gatling ? KW_GATLING : (kind == WP_KNIFE ? KW_KNIFE : (kind == WP_PISTOL ? KW_PISTOL : (kind == WP_AR ? KW_RIFLE : (kind == WP_SNIPER ? KW_SNIPER : KW_RPG)))); }
  // A car under cannon fire does not go up at once: holes, then smoke, then fire, and only the fire ends it.
  struct CarRounds {
    int hits = 0, tankHits = 0;
    double burning = -1;   // since when it has been on fire; negative while it is not
    double smokeClock = 0;
  };
  CarRounds carRounds[kCars];
  static constexpr int kCarSmokeHits = 8, kCarFireHits = 22, kCarFireTankHits = 4;
  static constexpr double kCarBurnTime = 9.0;
  void stepCarRounds(double dt);
  // And if it was a shell, its flight, for the kill cam to follow.
  using Shot = ShellFlight;
  Shot hurtShot[kMaxPlayers];
  int killerOf(int p) const { return hurtBy[p] >= 0 && hurtBy[p] != p && hurtByAge[p] < kKillCredit ? hurtBy[p] : -1; }
  int carHitBy[kCars] = {-1, -1, -1, -1, -1, -1};
  int playersWanted = 2;  // how many it takes for a match, half on each side
  Phase phase = WAITING;
  int winner = -1;  // -1 until decided; a draw stays -1 through FINISHED
  u32 matchId = 1;
  u32 wins[2] = {};
  double phaseClock = 0;
  u32 blastSeen = 0;

  // Tanks first, then cars, then the scenery. Every machine ends up with the same node at the same index.
  void build();
  void step(const Intent in[kMaxPlayers]);
  void updateFrames();
  // weapon: 0 knife, 1 pistol, 2 assault rifle, 3 big rifle, 4 rocket launcher
  // sightsOnly: looking along one's own weapon. No body in the way, just the gun held up under the eye.
  // Where a line of sight ends, in what a bullet would stop at: the world, stones, vehicles and enemies on foot.
  struct AimFoe {
    V3 p;
    double height;
  };
  static V3 aimEnd(const World& world, const std::vector<V3>& pos, const std::vector<Block>& blocks, const VehicleFrame* frames, int vehicles, int skipVehicle,
                   const std::vector<AimFoe>& foes, V3 from, V3 dir, double clear);
  static void poseSoldier(Sim& sim, u32 first, const Soldier& s, bool hidden, int weapon, bool sightsOnly = false, bool ghillie = false);
  V3 vehicleExit(int v) const;
  static void ragdollStart(Ragdoll& r, const Soldier& s, int weapon, V3 push);
  void ragdollStep(Ragdoll& r, double dt) const;
  static void poseRagdoll(Sim& sim, u32 first, const Ragdoll& r, bool ghillie = false);
  // Pools of blood under a body: three flat discs, one after the other as it spreads.
  static constexpr int kBloodDiscs = 11;
  u32 bloodFirst[kMaxPlayers][kBloodDiscs] = {};
  int nearestVehicle(V3 p, double reach) const;
  // A tank has room for a driver and a gunner, a car for a driver.
  int occupants(int v) const {
    int n = 0;
    for (const Player& pl : players)
      if (pl.vehicle == v) n++;
    return n;
  }
  bool occupied(int v) const { return occupants(v) > 0; }
  bool full(int v) const { return occupants(v) >= (v < kTanks ? 2 : 1); }
  int seatHolder(int v, int seat) const {
    for (int i = 0; i < kMaxPlayers; i++)
      if (players[i].vehicle == v && players[i].seat == seat) return i;
    return -1;
  }
  bool vehicleDead(int v) const { return v < kTanks ? tanks[v].destroyed : cars[size_t(v - kTanks)].exploded; }
  // Moves a soldier one time step. Used by the server, and by a player's own machine to answer at once.
  void walk(Soldier& s, const WalkInput& in, double dt) const;
  static constexpr double kWalkDt = 0.01;
  // A few fixed nodes for the one guided missile each launcher tank can have in the air, drawn as rods.
  static constexpr int kMissileNodes = 7;
  // One for each tank that can launch one: every side has one on every hull.
  static constexpr int kMissileTanks = 2 * HK_COUNT;
  u32 missileFirst[kMissileTanks] = {};
  static void poseMissile(Sim& sim, u32 first, bool live, V3 at, V3 dir, double scale = 1.0);
  // The same shape for what else flies under its own power: from a launcher on the shoulder, and from a strike code.
  static constexpr int kRpgRockets = 3, kStrikeRockets = 4;
  static constexpr double kRpgScale = 0.42, kStrikeScale = 1.5;
  u32 rocketFirst[kRpgRockets + kStrikeRockets] = {};
  static constexpr double kTapTime = 5.0, kQueryWait = 30.0, kFixError = 70.0, kComputerReach = 2.6;
  bool atEnemyComputer(int p, V3 at) const { return len(at + V3{0, 0, 1.0} - world.base(1 - teamOf(p)).computer) < kComputerReach; }
  static constexpr double kSetupTime = 3.0;  // to get down and have the big rifle on its bipod
  static constexpr double kRepairTime = 6.0;
  int nearestToRepair(V3 p) const;
  void restart();
  void placePlayers();
  bool choose(int player, const u8 items[IT_COUNT], bool ready);  // false if it does not fit
  static int tankIndex(int side, int turret, int hull = HK_MEDIUM) { return side * HK_COUNT * TK_COUNT + hull * TK_COUNT + turret; }
  void drainEvents(std::vector<SimEvent>& out);

 private:
  void startMatch();
  void stepPlayers(const Intent in[kMaxPlayers]);
  int enemyNearest(int p, V3 from) const;
  void boardVehicle(int p, int v);
  void fireWeapon(int p, const Intent& in);
  void stepBullets(double dt);
  // A bullet going into a car does not take it apart: it goes through whatever lies on its way. True if it hit a part.
  bool bulletInCar(int k, V3 from, V3 dir, int kind, int owner, bool gatling = false);
public:
  void creditHurt(int victim, int by, bool missile = false, const Shot& shot = Shot{}, u8 weapon = KW_NONE);
private:
  void stepHeli(int h, const Intent in[kMaxPlayers], double dt);
  bool bulletInHeli(int h, V3 from, V3 to, const Bullet& b);
  void explodeHeli(int h);
  void leaveHeli(int p);
  void countDeaths();
  bool deathCounted[kMaxPlayers] = {};
  void placeMine(int p, int kind);
  u32 bridgeFallsSeen = 0;
  void noteHit(int victim, int attacker, V3 from);
  double tankDamageSeen[kTanks] = {};
  void callStrike(int p, V3 at);
  void stepMines(double dt);
  V3 soldierAt(int p, double lag) const;
  void leaveVehicle(int p);
};
