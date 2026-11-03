// One match, without any graphics. Each side's yard holds one tank with every turret and one of every car; the
// players choose a turret and what to carry, then play it out until one of the chosen tanks is gone, then the
// next match. The dedicated server steps this. A player's machine builds the same thing and only shows it.
#pragma once
#include <vector>
#include "sim.h"
#include "tank.h"
#include "vehicle.h"
#include "world.h"

enum ItemKind : u8 { IT_FOOT_MINE, IT_AT_MINE, IT_REPAIR, IT_AR, IT_SMALL_AMMO, IT_COUNT };
struct ItemInfo {
  const char* name;
  int slots;
};
inline constexpr ItemInfo kItems[IT_COUNT] = {
    {"Tretmine", 2}, {"Panzermine", 5}, {"Reparaturset", 10}, {"Sturmgewehr", 2}, {"Kleinmunition (60 Schuss)", 1},
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
  bool grounded = true;
  static constexpr double kRadius = 0.32, kHeight = 1.8;
};
struct WalkInput {
  double forward = 0, strafe = 0;  // -1..1; strafe positive to the left
  double yaw = 0;
  bool jump = false, sprint = false;
};

// Where a vehicle is, as far as those on foot are concerned. The same on the server and on a player's machine.
struct VehicleFrame {
  V3 cog, fwd{1, 0, 0}, left{0, 1, 0}, up{0, 0, 1};
  double speed = 0;
  double halfLength = 2.3, halfWidth = 1.0, height = 1.6;
};

enum WeaponKind : u8 { WP_KNIFE, WP_PISTOL, WP_AR, WP_SNIPER, WP_RPG, WP_COUNT };
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
    {"Scharfschützengewehr", 2.2, false, 950, 130, 0.0004},
    {"Raketenwerfer", 4.0, false, 130, 0, 0.012},  // loose at long range: it wanders as it flies
};

class Duel {
 public:
  static constexpr double kDt = 1.0 / 2000.0;
  static constexpr double kEndPause = 8.0;    // seconds to look at the wreck before the next match
  static constexpr double kChooseTime = 60.0;  // whoever has not decided by then goes with what is selected
  static constexpr int kTanks = 2 * TK_COUNT;
  static constexpr int kCarKinds = 3;
  static constexpr int kCars = 2 * kCarKinds;
  static constexpr int kVehicles = kTanks + kCars;

  enum Phase : u8 { WAITING, CHOOSING, PLAYING, FINISHED };

  struct Player {
    bool present = false, ready = false;
    u8 items[IT_COUNT] = {};
    Soldier soldier;
    int vehicle = -1;  // what they sit in, -1 on foot
    double walkDebt = 0;
    int weapon = WP_PISTOL;
    bool hasWeapon[WP_COUNT] = {true, true, false, false, false};
    int smallAmmo = 24;  // pistol and assault rifle: what one starts with, and what the loadout adds
    int sniperAmmo = 0, rpgAmmo = 0;
    double shotClock = 0;
    double repairClock = 0;  // working on a vehicle from outside, which takes a kit and being there
    int repairing = -1;
    u32 lastInput = 0;  // the input the soldier has last been moved by, for the player's prediction
  };
  // What a player does this step beyond the pedals of a vehicle.
  struct Intent {
    Controls c;
    WalkInput walk;
    double pitch = 0;
    bool use = false;  // E: get in, get out
    bool fireFoot = false;  // a press of the trigger
    bool fireHeld = false;
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
  VehicleFrame frames[kVehicles];
  // Which never touch anything and only get drawn: a soldier is not part of the soft body world. A small cube of
  // nodes to each rigid part of the body, as the mesh's binding wants, and each weapon counts as a part.
  static constexpr int kPartNodes = 8, kSoldierParts = 17, kSoldierNodes = kSoldierParts * kPartNodes;
  static const char* soldierPartName(int part);
  static const V3 kSoldierRest;  // where the bind pose stands
  u32 soldierFirst[2] = {};
  size_t blastsDone = 0;
  // What flies from hand weapons. Hits are judged against where things were when the shooter fired, as they saw it.
  struct Bullet {
    V3 p, v;
    int owner = 0;
    int kind = WP_PISTOL;
    double age = 0, lag = 0;
  };
  std::vector<Bullet> bullets;
  struct Past {
    V3 soldier[2];
  };
  static constexpr int kPastSteps = 32;  // every hundredth of a second
  Past past[kPastSteps];
  int pastAt = 0;
  double bulletDebt = 0;
  u32 randState = 12345;

  Player players[2];
  Phase phase = WAITING;
  int winner = -1;  // -1 until decided; a draw stays -1 through FINISHED
  u32 matchId = 1;
  u32 wins[2] = {};
  double phaseClock = 0;
  u32 blastSeen = 0;

  // Tanks first, then cars, then the scenery. Every machine ends up with the same node at the same index.
  void build();
  void step(const Intent in[2]);
  void updateFrames();
  // weapon: 0 knife, 1 pistol, 2 assault rifle, 3 big rifle, 4 rocket launcher
  static void poseSoldier(Sim& sim, u32 first, const Soldier& s, bool hidden, int weapon);
  V3 vehicleExit(int v) const;
  int nearestVehicle(V3 p, double reach) const;
  bool occupied(int v) const { return players[0].vehicle == v || players[1].vehicle == v; }
  bool vehicleDead(int v) const { return v < kTanks ? tanks[v].destroyed : cars[size_t(v - kTanks)].exploded; }
  // Moves a soldier one time step. Used by the server, and by a player's own machine to answer at once.
  void walk(Soldier& s, const WalkInput& in, double dt) const;
  static constexpr double kWalkDt = 0.01;
  static constexpr double kRepairTime = 6.0;
  int nearestToRepair(V3 p) const;
  void restart();
  void placePlayers();
  bool choose(int player, const u8 items[IT_COUNT], bool ready);  // false if it does not fit
  static int tankIndex(int side, int turret) { return side * TK_COUNT + turret; }
  void drainEvents(std::vector<SimEvent>& out);

 private:
  void startMatch();
  void stepPlayers(const Intent in[2]);
  void boardVehicle(int p, int v);
  void fireWeapon(int p, const Intent& in);
  void stepBullets(double dt);
  V3 soldierAt(int p, double lag) const;
  void leaveVehicle(int p);
};
