// A player the server plays itself. It only ever hands in what a person could: the same intents, nothing else. What it
// knows beyond that: roughly where someone is every half a minute, the way a tapped computer would tell it, so that a
// match with someone hiding still comes to an end.
#pragma once
#include <vector>
#include "duel.h"

class Bot {
 public:
  explicit Bot(u32 seed = 1) : rand_(seed * 2654435761u + 12345u) {}
  // Fills in what this player does for this physics step.
  void step(Duel& d, int me, Duel::Intent& out);

 private:
  void think(Duel& d, int me);
  void chooseLoadout(Duel& d, int me);
  void lookAround(const Duel& d, int me, V3 eye);
  bool canSee(const Duel& d, V3 from, V3 to, int foe) const;
  void onFoot(Duel& d, int me);
  void inTank(Duel& d, int me);
  void aimAt(const Duel& d, int me, double shellSpeed, V3 from);
  // Getting about: along the roads, over a bridge that still stands, or through a ford.
  void planTo(const Duel& d, V3 from, V3 goal);
  bool straightWay(const World& w, V3 a, V3 b) const;
  bool roadWay(const World& w, V3 a, V3 b, std::vector<V3>& out) const;
  V3 nextPoint(V3 at, double reach);
  void walkTowards(const Duel& d, int me, V3 at, bool sprint);
  void driveTowards(const Duel& d, int me, V3 at, double wantSpeed);
  V3 goalFor(const Duel& d, int me);
  double chance(double a, double b);

  Duel::Intent held_;
  Duel::Intent pulse_;  // what goes for one step only
  double clock_ = 0;
  u32 rand_ = 1;
  u32 match_ = 0;

  int foe_ = -1;
  bool seesFoe_ = false;
  double seenFor_ = 0;
  V3 foeAt_, foeVel_, aimOff_;
  V3 lastSeen_;
  double lastSeenAge_ = 1e9;
  double lookClock_ = 0;
  V3 hint_;
  double hintAge_ = 1e9;
  u16 hitsSeen_ = 0;
  double shotClock_ = 0;

  std::vector<V3> path_;
  size_t pathAt_ = 0;
  V3 pathGoal_;
  double pathAge_ = 1e9;
  u32 fallsSeen_ = 0;

  int wantVehicle_ = -1;
  bool repairing_ = false;
  int repairTank_ = -1;
  double useClock_ = 0;

  V3 stuckAt_;
  double stuckClock_ = 0, unstickClock_ = 0, unstickSteer_ = 0;
};
