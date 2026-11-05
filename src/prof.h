// Where the time of a step goes: off unless a bench turns it on, and then a clock reading at each end of a stage.
#pragma once
#include <chrono>
enum ProfStage { PS_PLAYERS, PS_BEAMS, PS_VEHICLES, PS_PAIRS, PS_CONTACT, PS_WORLD, PS_INTEGRATE, PS_SLEEP, PS_BOTS, PS_COUNT };
inline bool gProfOn = false;
inline double gProf[PS_COUNT] = {};
struct ProfScope {
  ProfStage stage;
  std::chrono::steady_clock::time_point at;
  bool stopped = false;
  explicit ProfScope(ProfStage s) : stage(s) {
    if (gProfOn) at = std::chrono::steady_clock::now();
  }
  void stop() {
    if (gProfOn && !stopped) gProf[stage] += std::chrono::duration<double>(std::chrono::steady_clock::now() - at).count();
    stopped = true;
  }
  ~ProfScope() { stop(); }
};
