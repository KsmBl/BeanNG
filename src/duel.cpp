#include "duel.h"

#include <cmath>
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
      if (k == 0) cars.push_back(buildXantia(sim, p, 0.0, &rigs[at]));
      else if (k == 1) cars.push_back(buildOne1(sim, p, &rigs[at]));
      else cars.push_back(buildHummer(sim, p, &rigs[at]));
      turnNodes(sim, first, u32(sim.pos.size()), p, base.facing);
    }
  }
  sim.firstSceneryNode = u32(sim.pos.size());
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
  carsStart = cars;
  simStart = sim;
  blastSeen = sim.blastCount;
}

void Duel::restart() {
  world.clearCraters();
  sim = simStart;
  for (int i = 0; i < kTanks; i++) tanks[i] = tanksStart[i];
  cars = carsStart;
  winner = -1;
  phaseClock = 0;
  blastSeen = sim.blastCount;
  matchId++;
  for (Player& p : players) {
    p.tank = -1;
    p.ready = false;
  }
}

bool Duel::choose(int player, int turret, const u8 items[IT_COUNT], bool ready) {
  if (phase != CHOOSING || player < 0 || player > 1 || turret < 0 || turret >= TK_COUNT) return false;
  if (slotsUsed(items) > kLoadoutSlots) return false;
  Player& p = players[player];
  p.turret = turret;
  for (int i = 0; i < IT_COUNT; i++) p.items[i] = items[i];
  p.ready = ready;
  return true;
}

void Duel::startMatch() {
  for (int i = 0; i < 2; i++) {
    Player& p = players[i];
    p.tank = tankIndex(i, p.turret);
    Tank& t = tanks[p.tank];
    t.repairKits = p.items[IT_REPAIR];
    sim.objs[t.hullObj].alwaysAwake = true;
    sim.objs[t.turretObj].alwaysAwake = true;
    sim.wake(t.hullObj);
    sim.wake(t.turretObj);
  }
  phase = PLAYING;
  phaseClock = 0;
}

void Duel::step(const Controls in[2]) {
  sim.forces(kDt);
  const Controls still;
  for (int i = 0; i < kTanks; i++) {
    Tank& t = tanks[i];
    const Controls* c = &still;
    bool owned = false;
    for (int p = 0; p < 2; p++)
      if (players[p].tank == i) {
        c = &in[p];
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
  for (Vehicle& v : cars) {
    // What a shell has torn open right beside it no longer works as a vehicle.
    if (blast && !v.exploded && len(v.cog - sim.lastBlast) < sim.lastBlastRadius * 0.30) v.wreck(sim);
    if (sim.objs[v.obj].awake) v.update(sim, parked, kDt);
  }
  sim.finish(kDt);
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
        restart();
        startMatch();
      }
      break;
    case PLAYING: {
      if (!both) {
        restart();
        phase = WAITING;
        break;
      }
      const bool dead[2] = {tanks[players[0].tank].destroyed, tanks[players[1].tank].destroyed};
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
