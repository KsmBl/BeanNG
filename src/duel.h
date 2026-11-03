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
    int turret = TK_STANDARD;
    u8 items[IT_COUNT] = {};
    int tank = -1;  // which of the tanks is theirs, once the match runs
  };

  World world;
  Sim sim, simStart;
  Tank tanks[kTanks], tanksStart[kTanks];
  std::vector<Vehicle> cars, carsStart;
  CarRig rigs[kVehicles];
  V3 spawn[kVehicles];
  double yaw[kVehicles] = {};
  u32 tankFirst[kTanks] = {}, tankNodes = 0, tanksEnd = 0;

  Player players[2];
  Phase phase = WAITING;
  int winner = -1;  // -1 until decided; a draw stays -1 through FINISHED
  u32 matchId = 1;
  u32 wins[2] = {};
  double phaseClock = 0;
  u32 blastSeen = 0;

  // Tanks first, then cars, then the scenery. Every machine ends up with the same node at the same index.
  void build();
  void step(const Controls in[2]);
  void restart();
  bool choose(int player, int turret, const u8 items[IT_COUNT], bool ready);  // false if it does not fit
  static int tankIndex(int side, int turret) { return side * TK_COUNT + turret; }
  void drainEvents(std::vector<SimEvent>& out);

 private:
  void startMatch();
};
