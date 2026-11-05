// A player the server plays itself. It only ever hands in what a person could: the same intents, nothing else. Every
// round it settles on a way to play, and takes along and loots what that way needs. What it knows beyond a person: roughly
// where someone is every half a minute, as a tapped computer would tell it, so that a match with someone hiding still
// comes to an end; roughly where a shot came from; and which way a track in the grass runs.
#pragma once
#include <vector>
#include "duel.h"

class Bot {
 public:
  enum Tactic : u8 { STORM, FLANK, AMBUSH, SNIPER, SCAVENGE, SPY, MISSILE, AIR, RAID, kTacticCount };
  void forceTactic(Tactic t) { forced_ = t; }  // for the test bench
  static const char* tacticName(Tactic t);
  explicit Bot(u32 seed = 1) {
    // Well stirred: numbers next to each other must not make bots that start out alike.
    rand_ = seed * 2654435761u + 12345u;
    rand_ = (rand_ ^ (rand_ >> 15)) * 2246822519u;
    rand_ = (rand_ ^ (rand_ >> 13)) * 3266489917u;
    rand_ ^= rand_ >> 16;
    // No two play quite alike: some are quicker, some shoot straighter.
    aimSkill_ = chance(0.7, 1.35);
    reactSkill_ = chance(0.75, 1.3);
  }
  // Fills in what this player does for this physics step.
  void step(Duel& d, int me, Duel::Intent& out);
  Tactic tactic() const { return tactic_; }
  bool planned() const { return picked_; }
  // How it is getting on, for the test bench.
  const char* doing() const { return doing_; }
  int followedTracks() const { return tracksFollowed_; }
  int cratesTaken() const { return cratesAllTime_; }
  int computersTapped() const { return tapsAllTime_; }
  int timesRescued() const { return rescuesAllTime_; }
  int flights() const { return flightsAllTime_; }

 private:
  void pickTactic(const Duel& d, int me);
  void noteHits(const Duel& d, int me, V3 eye);
  void noteTracks(const Duel& d, int me, V3 at);
  bool lookForTracks(const Duel& d, V3 eye);
  bool goodTank(const Duel& d, int v) const;
  void chooseTank(const Duel& d, int me, V3 from);
  // Getting out of what no longer works, and not back into it.
  void bail(int v);
  int pickCrate(const Duel& d, V3 from, bool ammunitionOnly) const;
  bool wantsLoot(const Duel& d, int me) const;
  bool lootOnFoot(Duel& d, int me);
  void planToCrate(const Duel& d, V3 from, V3 at);
  void checkIdle(Duel& d, int me, V3 here);
  bool findPost(const Duel& d, int me, V3 from, bool high);
  void fightInTank(Duel& d, int me);
  void inHeli(Duel& d, int me);
  bool toHeli(Duel& d, int me);
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
  V3 goalFor(const Duel& d, int me, V3 at);
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
  double wantClock_ = 0;
  bool repairing_ = false;
  int repairTank_ = -1;
  double repairClock_ = 0;
  double useClock_ = 0;
  bool givenUp_[Duel::kVehicles] = {};  // what does not work, and is no use getting back into

  V3 stuckAt_;
  double stuckHeading_ = 0, stuckClock_ = 0, unstickClock_ = 0, unstickSteer_ = 0, flippedClock_ = 0, brokenClock_ = 0;
  int stuckCount_ = 0;
  // The last word on getting stuck: whatever the plan says, nobody stands about doing nothing for long.
  V3 idleAt_;
  double idleClock_ = 0;
  int idleStrikes_ = 0;
  int spyStage_ = 0;
  bool wasTapped_ = false;
  int cratesAllTime_ = 0, tapsAllTime_ = 0, rescuesAllTime_ = 0, flightsAllTime_ = 0;
  // In the air, and what comes with it.
  Tactic forced_ = kTacticCount;
  int wantHeli_ = -1, mendHeli_ = -1, flownHeli_ = -1;
  double lookYaw_ = 0, lookPitch_ = 0;
  bool lookSet_ = false;
  double wantHeliClock_ = 0, mendClock_ = 0, orbitSide_ = 1, orbitClock_ = 0, wantHigh_ = 45.0;

  // The plan for this round.
  double burstClock_ = 0;
  Tactic tactic_ = STORM, lastTactic_ = kTacticCount;
  bool picked_ = false;
  double tacticClock_ = 0, tacticLimit_ = 1e9;
  const char* doing_ = "";
  double aimSkill_ = 1, reactSkill_ = 1;
  // A place to lie in wait, and the bit of road it watches.
  bool hasPost_ = false, atPost_ = false;
  V3 post_, watch_;
  int shotsFromPost_ = 0;
  double dodgeClock_ = 0, dodgeSide_ = 1, shiftClock_ = 8;
  double noEffect_ = 0, moveClock_ = 0;  // shooting from here does nothing: from somewhere else, then
  u16 landedSeen_ = 0;
  int flankSide_ = 1;
  bool flanked_ = false;
  double queryClock_ = 0;
  // Crates: which one it is going for, and which are not worth another look.
  int crate_ = -1;
  double crateClock_ = 0;
  int cratesTaken_ = 0, cratesWanted_ = 0;
  std::vector<u8> crateTried_;
  // The tracks the enemy has left in the grass, once it has come across them.
  struct Mark {
    V3 at;
    int vehicle;
    double when;
  };
  std::vector<Mark> marks_;
  V3 markedAt_[Duel::kVehicles];
  double now_ = 0, trackLook_ = 0, trackWhen_ = -1;
  bool onTrack_ = false;
  V3 trackGoal_;
  int trackVehicle_ = -1, tracksFollowed_ = 0;
};
