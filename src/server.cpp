// The duel server. Run it on any machine both players can reach:  beanng-server [port]
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <thread>
#include "duel.h"
#include "duelnet.h"

using namespace net;

namespace {

volatile std::sig_atomic_t gStop = 0;
void onSignal(int) { gStop = 1; }

constexpr double kTimeout = 5.0;

struct Client {
  bool used = false;
  Addr addr;
  u32 token = 0;
  double lastHeard = 0;
  InputMsg in;
  bool haveInput = false;
  // One-shot controls, each seen once however many packets carry the same count.
  bool fire = false, launch = false, repair = false, use = false;
  // Scenery that has moved, as far as this player has been told, and where the slow sweep has got to.
  std::vector<float> told;
  u32 sweep = 0;
};

}  // namespace

int main(int argc, char** argv) {
  int port = kDefaultPort;
  if (argc > 1) port = atoi(argv[1]);
  if (port <= 0 || port > 65535) {
    fprintf(stderr, "Benutzung: beanng-server [port]\n");
    return 1;
  }
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  auto* duel = new Duel();
  printf("Baue die Welt...\n");
  duel->build();
  Socket sock;
  if (!sock.open(u16(port))) {
    fprintf(stderr, "Port %d laesst sich nicht oeffnen\n", port);
    return 1;
  }
  printf("%zu Knoten, %zu Balken.\n", duel->sim.pos.size(), duel->sim.beams.size());
  printf("Der Server wartet auf UDP-Port %d.\n", port);
  fflush(stdout);

  Client clients[2];
  std::random_device entropy;
  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::now();
  const auto now = [&] { return std::chrono::duration<double>(Clock::now() - t0).count(); };

  const auto sendWelcome = [&](int slot) {
    Writer w(P_WELCOME);
    w.put(kProtocol);
    w.put(u8(slot));
    w.put(clients[slot].token);
    w.put(u32(duel->sim.pos.size()));
    w.put(u32(duel->sim.beams.size()));
    sock.send(w.data(), w.size(), clients[slot].addr);
  };

  const auto receive = [&] {
    u8 buf[kMaxPacket];
    Addr from;
    int n;
    while ((n = sock.recv(buf, sizeof buf, from)) >= 0) {
      Reader r(buf, size_t(n));
      u8 kind = 0;
      if (!r.header(kind)) continue;
      int slot = -1;
      for (int i = 0; i < 2; i++)
        if (clients[i].used && clients[i].addr == from) slot = i;
      if (kind == P_HELLO) {
        u16 protocol = 0;
        r.get(protocol);
        // A welcome is about as long as a hello has to be, so nobody can use this to flood a third party.
        if (!r.ok() || n < 64) continue;
        if (protocol != kProtocol) {
          Writer w(P_FULL);
          w.put(u8(1));
          sock.send(w.data(), w.size(), from);
          continue;
        }
        if (slot < 0) {
          for (int i = 0; i < 2 && slot < 0; i++)
            if (!clients[i].used) slot = i;
          if (slot < 0) {
            Writer w(P_FULL);
            w.put(u8(0));
            sock.send(w.data(), w.size(), from);
            continue;
          }
          clients[slot] = Client{};
          clients[slot].used = true;
          clients[slot].addr = from;
          clients[slot].token = u32(entropy());
          clients[slot].told.assign(duel->sim.pos.size() * 3, 0.f);
          for (size_t i = 0; i < duel->sim.pos.size(); i++) {
            clients[slot].told[i * 3] = float(duel->simStart.pos[i].x);
            clients[slot].told[i * 3 + 1] = float(duel->simStart.pos[i].y);
            clients[slot].told[i * 3 + 2] = float(duel->simStart.pos[i].z);
          }
          printf("Spieler %d verbunden: %s\n", slot + 1, toString(from).c_str());
          fflush(stdout);
        }
        clients[slot].lastHeard = now();
        sendWelcome(slot);
      } else if (slot >= 0 && kind == P_INPUT) {
        InputMsg m;
        if (!read(r, m) || m.token != clients[slot].token) continue;
        Client& c = clients[slot];
        c.lastHeard = now();
        // Out of order on the way: only ever forward.
        if (c.haveInput && int32_t(m.seq - c.in.seq) <= 0) continue;
        if (c.haveInput) {
          if (m.fireCount != c.in.fireCount) c.fire = true;
          if (m.launchCount != c.in.launchCount) c.launch = true;
          if (m.repairCount != c.in.repairCount) c.repair = true;
          if (m.useCount != c.in.useCount) c.use = true;
        }
        c.in = m;
        c.haveInput = true;
        if (duel->phase == Duel::CHOOSING) {
          // A loadout that does not fit is simply not taken: the client knows the rules, a cheat gets nothing.
          u8 items[IT_COUNT];
          for (int k = 0; k < IT_COUNT; k++) items[k] = m.items[k];
          duel->choose(slot, items, m.ready != 0);
        }
      } else if (slot >= 0 && kind == P_BYE) {
        u32 token = 0;
        r.get(token);
        if (r.ok() && token == clients[slot].token) {
          printf("Spieler %d hat das Spiel verlassen\n", slot + 1);
          fflush(stdout);
          clients[slot].used = false;
        }
      }
    }
  };

  u32 tick = 0;
  u32 seenMatch = duel->matchId;
  std::vector<SimEvent> events;
  std::vector<SimEvent> pending;
  u32 tracerCount = 0;

  const auto sendTank = [&](const Addr& to, int i) {
    Writer w(P_TANK);
    w.put(tick);
    w.put(duel->matchId);
    w.put(u8(i));
    const Tank& t = duel->tanks[i];
    w.putV3(t.cog);
    writeState(w, t.netState());
    w.put(u16(duel->tankNodes));
    for (u32 k = 0; k < duel->tankNodes; k++) {
      const V3 d = duel->sim.pos[duel->tankFirst[i] + k] - t.cog;
      w.put(packOffset(d.x));
      w.put(packOffset(d.y));
      w.put(packOffset(d.z));
    }
    sock.send(w.data(), w.size(), to);
  };

  const auto sendCar = [&](const Addr& to, int k) {
    const u32 first = duel->carFirst[k];
    const u32 end = k + 1 < Duel::kCars ? duel->carFirst[k + 1] : duel->carsEnd;
    const VehicleFrame& f = duel->frames[Duel::kTanks + k];
    const Vehicle& v = duel->cars[size_t(k)];
    constexpr u32 kPerPacket = 160;
    for (u32 at = first; at < end; at += kPerPacket) {
      const u32 count = std::min(end - at, kPerPacket);
      Writer w(P_CAR);
      w.put(tick);
      w.put(duel->matchId);
      w.put(u8(k));
      w.putV3(f.cog);
      w.putV3(f.fwd);
      w.putV3(f.left);
      w.putV3(f.up);
      w.put(float(f.speed));
      w.put(float(v.rpm()));
      w.put(i8(v.gear));
      w.put(u8(v.exploded ? 1 : 0));
      w.put(at - first);
      w.put(u16(count));
      for (u32 n = at; n < at + count; n++) {
        const V3 d = duel->sim.pos[n] - f.cog;
        w.put(packOffset(d.x));
        w.put(packOffset(d.y));
        w.put(packOffset(d.z));
      }
      sock.send(w.data(), w.size(), to);
    }
  };

  const auto sendMeta = [&](const Client& c, int slot) {
    Writer w(P_META);
    w.put(tick);
    w.put(duel->matchId);
    w.put(u8(duel->phase));
    w.put(i8(duel->winner));
    w.put(duel->wins[0]);
    w.put(duel->wins[1]);
    w.put(float(duel->phaseClock));
    w.put(u8((clients[0].used ? 1 : 0) | (clients[1].used ? 2 : 0) | (duel->players[0].ready ? 4 : 0) | (duel->players[1].ready ? 8 : 0)));
    for (const Duel::Player& pl : duel->players) {
      w.put(i8(pl.vehicle));
      w.putV3(pl.soldier.p);
      w.putV3(pl.soldier.v);
      w.put(float(pl.soldier.yaw));
      w.put(float(pl.soldier.pitch));
      w.put(float(pl.soldier.hp));
      w.put(float(pl.soldier.stride));
    }
    w.put(duel->players[slot].lastInput);
    // What this player carries, as it stands now.
    for (int k = 0; k < IT_COUNT; k++) w.put(duel->players[slot].items[k]);
    w.put(float(duel->players[slot].repairClock));
    // Blasts stay in the packet until the player says they have gone off there.
    const auto& log = duel->sim.blastLog;
    const u32 ack = c.in.ackMatch == duel->matchId ? c.in.ackBlasts : 0;
    const u32 first = std::min(ack, u32(log.size()));
    const u32 count = std::min(u32(log.size()) - first, u32(kMaxBlastsPerPacket));
    w.put(first);
    w.put(u8(count));
    for (u32 i = first; i < first + count; i++) {
      w.putV3(log[i].c);
      w.put(float(log[i].impulse));
      w.put(float(log[i].radius));
    }
    const size_t nEv = std::min(events.size(), size_t(kMaxEventsPerPacket));
    w.put(u8(nEv));
    for (size_t i = 0; i < nEv; i++) {
      w.putV3(events[i].p);
      w.putV3(events[i].v);
      w.put(events[i].strength);
      w.put(u8(events[i].kind));
    }
    sock.send(w.data(), w.size(), c.addr);
  };

  const auto sendScenery = [&](Client& c) {
    const Sim& s = duel->sim;
    const u32 first = s.firstSceneryNode;  // vehicles go whole, in their own packets
    const u32 n = u32(s.pos.size());
    std::vector<u32> moved;
    const auto farFromTold = [&](u32 k) {
      const double dx = s.pos[k].x - c.told[k * 3], dy = s.pos[k].y - c.told[k * 3 + 1], dz = s.pos[k].z - c.told[k * 3 + 2];
      return dx * dx + dy * dy + dz * dz > 0.015 * 0.015;
    };
    for (u32 oi : s.active) {
      const Obj& o = s.objs[oi];
      for (u32 k : o.nodes)
        if (k >= first && farFromTold(k) && moved.size() < size_t(kMaxSceneryPerPacket) * 4) moved.push_back(k);
    }
    // And slowly past everything that is not where it was built: a packet may have got lost on the way.
    int healed = 0;
    for (int i = 0; i < 3000 && healed < 8; i++) {
      if (c.sweep < first || c.sweep >= n) c.sweep = first;
      const u32 k = c.sweep++;
      if (len2(s.pos[k] - duel->simStart.pos[k]) > 0.015 * 0.015) {
        moved.push_back(k);
        healed++;
      }
    }
    for (size_t at = 0; at < moved.size(); at += kMaxSceneryPerPacket) {
      const size_t count = std::min(moved.size() - at, size_t(kMaxSceneryPerPacket));
      Writer w(P_SCENERY);
      w.put(tick);
      w.put(duel->matchId);
      w.put(u8(count));
      for (size_t i = at; i < at + count; i++) {
        const u32 k = moved[i];
        w.put(k);
        w.putV3(s.pos[k]);
        c.told[k * 3] = float(s.pos[k].x);
        c.told[k * 3 + 1] = float(s.pos[k].y);
        c.told[k * 3 + 2] = float(s.pos[k].z);
      }
      sock.send(w.data(), w.size(), c.addr);
    }
  };

  double simTime = 0;
  double nextSend = 0;
  double busy = 0, lastReport = now();
  double wall0 = now();
  while (!gStop) {
    receive();
    const double t = now();
    for (int i = 0; i < 2; i++) {
      if (clients[i].used && t - clients[i].lastHeard > kTimeout) {
        printf("Spieler %d: keine Verbindung mehr\n", i + 1);
        fflush(stdout);
        clients[i].used = false;
      }
    }
    // Never more than a fifth of a second to catch up: past that, time simply stands still for a moment.
    if (t - wall0 - simTime > 0.2) simTime = t - wall0 - 0.2;
    const auto q0 = Clock::now();
    int done = 0;
    while (simTime < t - wall0 && done < 40) {
      Duel::Intent in[2];
      for (int i = 0; i < 2; i++) {
        Client& c = clients[i];
        if (!c.used || !c.haveInput) {
          in[i].c.brake = 1;
          continue;
        }
        Controls& k = in[i].c;
        k.steer = c.in.steer;
        k.throttle = c.in.throttle;
        k.brake = c.in.brake;
        k.handbrake = c.in.handbrake;
        k.aimPoint = c.in.aim;
        k.indirect = c.in.indirect;
        if (c.in.requestGear == 1 || c.in.requestGear == -1) k.requestGear = c.in.requestGear;
        k.launch = c.launch;
        k.fire = c.fire;
        in[i].use = c.use;
        in[i].repair = c.repair;
        in[i].seq = c.in.seq;
        in[i].pitch = c.in.pitch;
        in[i].walk.forward = c.in.throttle - c.in.brake;
        in[i].walk.strafe = c.in.steer;
        in[i].walk.yaw = c.in.yaw;
        in[i].walk.jump = c.in.handbrake > 0.5;
        in[i].walk.sprint = c.in.sprint;
        c.fire = c.launch = c.repair = c.use = false;
      }
      for (int i = 0; i < 2; i++) duel->players[i].present = clients[i].used;
      duel->step(in);
      simTime += Duel::kDt;
      done++;
    }
    busy += std::chrono::duration<double>(Clock::now() - q0).count();
    if (duel->matchId != seenMatch) {
      seenMatch = duel->matchId;
      printf("Match %u: Stand %u : %u\n", duel->matchId, duel->wins[0], duel->wins[1]);
      fflush(stdout);
      pending.clear();
      for (Client& c : clients)
        if (c.used) {
          for (size_t i = 0; i < duel->sim.pos.size(); i++) {
            c.told[i * 3] = float(duel->simStart.pos[i].x);
            c.told[i * 3 + 1] = float(duel->simStart.pos[i].y);
            c.told[i * 3 + 2] = float(duel->simStart.pos[i].z);
          }
        }
    }
    {
      std::vector<SimEvent> fresh;
      duel->drainEvents(fresh);
      for (const SimEvent& e : fresh) {
        // The player's machine sets off its own bang and its own cracked rocks from the blast list; the wall
        // stones' dust is worth sending. A tracer has plenty of company.
        if (e.kind == EV_BLAST) continue;
        if (e.kind == EV_TRACER && (tracerCount++ & 3) != 0) continue;
        if (pending.size() < 400) pending.push_back(e);
      }
    }
    if (t >= nextSend) {
      nextSend = std::max(nextSend + 1.0 / kSendRate, t - 0.1);
      tick++;
      events.clear();
      const size_t nEv = std::min(pending.size(), size_t(kMaxEventsPerPacket));
      events.assign(pending.begin(), pending.begin() + long(nEv));
      pending.erase(pending.begin(), pending.begin() + long(nEv));
      for (int i = 0; i < 2; i++) {
        Client& c = clients[i];
        if (!c.used) continue;
        sendMeta(c, i);
        // The tanks in play every time, and now and then the others, should a packet have gone astray.
        for (int k = 0; k < Duel::kTanks; k++) {
          const Tank& t = duel->tanks[k];
          if (duel->occupied(k) || duel->sim.objs[t.hullObj].awake || duel->sim.objs[t.turretObj].awake || int(tick % 15) == k) sendTank(c.addr, k);
        }
        // Cars, in as many pieces as it takes, while they move and now and then otherwise.
        for (int k = 0; k < Duel::kCars; k++) {
          const Vehicle& v = duel->cars[size_t(k)];
          if (duel->sim.objs[v.obj].awake || int((tick + 7) % 60) == k) sendCar(c.addr, k);
        }
        sendScenery(c);
      }
    }
    if (t - lastReport > 30.0) {
      printf("Physik-Last %.0f %%, wach %zu, Spieler %d\n", busy / (t - lastReport) * 100.0, duel->sim.active.size(), int(clients[0].used) + int(clients[1].used));
      fflush(stdout);
      busy = 0;
      lastReport = t;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }
  printf("Server beendet\n");
  delete duel;
  return 0;
}
