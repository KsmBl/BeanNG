// The duel server. Run it on any machine both players can reach:  beanng-server [port]
#include <chrono>
#include <csignal>
#include <deque>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <functional>
#include <string>
#include <vector>
#include "duel.h"
#include "duelnet.h"
#include "shape.h"

using namespace net;

namespace {

// The console: a prompt that stays at the bottom, with history and tab completion. What the game reports is written
// above it. When the input is not a terminal, say under a service manager, there is simply no prompt.
class Shell {
 public:
  void start() {
    interactive_ = isatty(STDIN_FILENO);
    if (!interactive_) return;
    tcgetattr(STDIN_FILENO, &saved_);
    termios raw = saved_;
    raw.c_lflag &= tcflag_t(~(ICANON | ECHO));  // Ctrl-C keeps working: it ends the server properly
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    active_ = true;
    redraw();
  }
  void stop() {
    if (!active_) return;
    active_ = false;
    tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
    fputs("\r\033[K", stdout);
    fflush(stdout);
  }
  // A line of output above the prompt.
  void print(const std::string& text) {
    if (active_) fputs("\r\033[K", stdout);
    puts(text.c_str());
    redraw();
  }
  std::function<std::vector<std::string>(const std::vector<std::string>& words)> complete;
  // Returns true and fills the line when one has been entered.
  bool poll(std::string& out) {
    if (!active_) return false;
    char c;
    while (read(STDIN_FILENO, &c, 1) == 1) {
      if (escape_ == 1) {
        escape_ = c == '[' ? 2 : 0;
        continue;
      }
      if (escape_ == 2) {
        escape_ = 0;
        if (c == 'A' && history_ > 0) setLine(past_[size_t(--history_)]);
        if (c == 'B' && history_ < int(past_.size())) {
          history_++;
          setLine(history_ == int(past_.size()) ? "" : past_[size_t(history_)]);
        }
        continue;
      }
      if (c == 27) {
        escape_ = 1;
      } else if (c == '\n' || c == '\r') {
        out = line_;
        line_.clear();
        if (!out.empty()) past_.push_back(out);
        history_ = int(past_.size());
        fputs("\r\033[K", stdout);
        printf("> %s\n", out.c_str());
        redraw();
        return true;
      } else if (c == 127 || c == 8) {
        // A whole letter at a time, however many bytes it is.
        while (line_.size() > 1 && (static_cast<unsigned char>(line_.back()) & 0xC0) == 0x80) line_.pop_back();
        if (!line_.empty()) line_.pop_back();
        redraw();
      } else if (c == '\t') {
        tab();
      } else if (c == 21) {  // Ctrl-U clears the line
        line_.clear();
        redraw();
      } else if (static_cast<unsigned char>(c) >= 32) {
        line_ += c;
        redraw();
      }
    }
    return false;
  }

 private:
  void redraw() {
    if (!active_) {
      fflush(stdout);
      return;
    }
    printf("\r\033[K> %s", line_.c_str());
    fflush(stdout);
  }
  void setLine(const std::string& s) {
    line_ = s;
    redraw();
  }
  void tab() {
    if (!complete) return;
    std::vector<std::string> words;
    std::string word;
    for (char ch : line_) {
      if (ch == ' ') {
        words.push_back(word);
        word.clear();
      } else {
        word += ch;
      }
    }
    words.push_back(word);  // the one being typed, perhaps still empty
    const std::string& typed = words.back();
    std::vector<std::string> hits;
    for (const std::string& c : complete(words))
      if (c.compare(0, typed.size(), typed) == 0) hits.push_back(c);
    if (hits.empty()) return;
    std::string common = hits[0];
    for (const std::string& h : hits) {
      size_t k = 0;
      while (k < common.size() && k < h.size() && common[k] == h[k]) k++;
      common.resize(k);
    }
    line_.resize(line_.size() - typed.size());
    line_ += common;
    if (hits.size() == 1) {
      line_ += ' ';
    } else {
      std::string all;
      for (const std::string& h : hits) all += h + "  ";
      print(all);
    }
    redraw();
  }
  bool interactive_ = false, active_ = false;
  termios saved_{};
  std::string line_;
  std::vector<std::string> past_;
  int history_ = 0;
  int escape_ = 0;
};

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
  char name[kNameBytes] = {};
  u32 chatSeq = 0;  // the last line of theirs that has gone in the log
  // One-shot controls, each seen once however many packets carry the same count.
  bool fire = false, launch = false, repair = false, use = false, setup = false, mineFoot = false, mineTank = false, strike = false, query = false, smoke = false, net = false, drone = false, medkit = false;
  // Scenery that has moved, as far as this player has been told, and where the slow sweep has got to.
  std::vector<float> told;
  u32 sweep = 0;
  // The same for the nodes of the vehicles, against the part they belong to.
  std::vector<i16> toldTanks, toldCars;
  u32 scanTank[Duel::kTanks] = {}, healTank[Duel::kTanks] = {}, scanCar[Duel::kCars] = {}, healCar[Duel::kCars] = {};
  // What goes in one datagram together, and the datagrams waiting to go out. Sent one by one over the tick and not all at
  // once: a small router's queue overflows from a burst, not from the amount.
  std::vector<u8> bundle;
  std::deque<std::vector<u8>> waiting;
  double nextOut = 0, outGap = 0;
};

}  // namespace

// --log: once a second what the server is doing.
static FILE* playLog = nullptr;

int main(int argc, char** argv) {
  int port = kDefaultPort;
  bool giveAll = false;
  int wanted = 2;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--log")) {
      char path[64];
      snprintf(path, sizeof path, "/tmp/beanng-server-%d.log", int(getpid()));
      playLog = fopen(path, "w");
      printf("Log: %s\n", path);
    } else if (!strcmp(argv[i], "--give-all")) giveAll = true;  // every weapon from the start, for trying things out
    else if (!strcmp(argv[i], "--players") && i + 1 < argc) wanted = std::clamp(atoi(argv[++i]) / 2 * 2, 2, int(Duel::kMaxPlayers));  // 2, 4 or 6, half to a side
    else port = atoi(argv[i]);
  }
  if (port <= 0 || port > 65535) {
    fprintf(stderr, "Benutzung: beanng-server [port] [--give-all] [--players 2|4|6] [--log]\n");
    return 1;
  }
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  auto* duel = new Duel();
  printf("Baue die Welt...\n");
  duel->giveAll = giveAll;
  duel->playersWanted = wanted;
  duel->build();
  Socket sock;
  if (!sock.open(u16(port))) {
    fprintf(stderr, "Port %d laesst sich nicht oeffnen\n", port);
    return 1;
  }
  printf("%zu Knoten, %zu Balken.\n", duel->sim.pos.size(), duel->sim.beams.size());
  auto* shapes = new VehicleShapes();
  shapes->build(*duel);
  printf("Der Server wartet auf UDP-Port %d. /help zeigt die Befehle, TAB ergaenzt sie.\n", port);
  fflush(stdout);
  Shell shell;
  const auto say = [&](const char* fmt, auto... args) {
    char text[512];
    snprintf(text, sizeof text, fmt, args...);
    shell.print(text);
  };

  Client clients[Duel::kMaxPlayers];
  struct ChatLine {
    int who;
    std::string text;
  };
  std::vector<ChatLine> chat;
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
      for (int i = 0; i < Duel::kMaxPlayers; i++)
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
          for (int i = 0; i < duel->playersWanted && slot < 0; i++)
            if (!clients[i].used) slot = i;
          if (slot < 0) {
            Writer w(P_FULL);
            w.put(u8(0));
            sock.send(w.data(), w.size(), from);
            continue;
          }
          clients[slot] = Client{};
          clients[slot].used = true;
          for (int k = 0; k < kNameBytes; k++) {
            char ch = 0;
            r.get(ch);
            // Nothing that could upset a terminal or the screen; UTF-8 may pass.
            clients[slot].name[k] = (static_cast<unsigned char>(ch) < 32 && ch != 0) ? '?' : ch;
          }
          clients[slot].name[kNameBytes - 1] = 0;
          if (!clients[slot].name[0]) snprintf(clients[slot].name, kNameBytes, "Spieler %d", slot + 1);
          clients[slot].addr = from;
          clients[slot].token = u32(entropy());
          clients[slot].toldTanks = shapes->restTanks;
          clients[slot].toldCars = shapes->restCars;
          clients[slot].told.assign(duel->sim.pos.size() * 3, 0.f);
          for (size_t i = 0; i < duel->sim.pos.size(); i++) {
            clients[slot].told[i * 3] = float(duel->simStart.pos[i].x);
            clients[slot].told[i * 3 + 1] = float(duel->simStart.pos[i].y);
            clients[slot].told[i * 3 + 2] = float(duel->simStart.pos[i].z);
          }
          say("Spieler %d verbunden: %s, %s", slot + 1, clients[slot].name, toString(from).c_str());
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
          if (m.setupCount != c.in.setupCount) c.setup = true;
          if (m.mineFootCount != c.in.mineFootCount) c.mineFoot = true;
          if (m.mineTankCount != c.in.mineTankCount) c.mineTank = true;
          if (m.strikeCount != c.in.strikeCount) c.strike = true;
          if (m.queryCount != c.in.queryCount) c.query = true;
          if (m.smokeCount != c.in.smokeCount) c.smoke = true;
          if (m.netCount != c.in.netCount) c.net = true;
          if (m.droneCount != c.in.droneCount) c.drone = true;
          if (m.medkitCount != c.in.medkitCount) c.medkit = true;
        }
        c.in = m;
        c.haveInput = true;
        if (duel->phase == Duel::CHOOSING) {
          // A loadout that does not fit is simply not taken: the client knows the rules, a cheat gets nothing.
          u8 items[IT_COUNT];
          for (int k = 0; k < IT_COUNT; k++) items[k] = m.items[k];
          duel->choose(slot, items, m.ready != 0);
        }
      } else if (slot >= 0 && kind == P_CHAT) {
        u32 token = 0, seq = 0;
        u8 length = 0;
        r.get(token);
        r.get(seq);
        r.get(length);
        Client& c = clients[slot];
        if (!r.ok() || token != c.token || length > kChatBytes || int32_t(seq - c.chatSeq) <= 0) continue;
        std::string text;
        for (int k = 0; k < length; k++) {
          char ch = 0;
          r.get(ch);
          text += static_cast<unsigned char>(ch) < 32 ? ' ' : ch;
        }
        if (!r.ok()) continue;
        c.chatSeq = seq;
        chat.push_back({slot, text});
        say("<%s> %s", c.name, text.c_str());
      } else if (slot >= 0 && kind == P_BYE) {
        u32 token = 0;
        r.get(token);
        if (r.ok() && token == clients[slot].token) {
          say("Spieler %d hat das Spiel verlassen", slot + 1);
          clients[slot].used = false;
        }
      }
    }
  };

  u32 tick = 0;  // how many times everyone has been told
  u32 simStep = 0;  // the moment that is on the packets
  u32 seenMatch = duel->matchId;
  double statLoad = 0, statReal = 1;
  std::vector<SimEvent> events;
  std::vector<SimEvent> pending;
  u32 tracerCount = 0;

  // Everything for one player in as few datagrams as it fits: the line counts packets as well as bytes.
  const auto flush = [&](Client& c) {
    if (c.bundle.empty()) return;
    c.waiting.push_back(std::move(c.bundle));
    c.bundle.clear();
  };
  const auto post = [&](Client& c, const Writer& w) {
    if (!c.bundle.empty() && c.bundle.size() + w.size() + 2 > size_t(kBundleBytes)) flush(c);
    if (c.bundle.empty()) {
      Writer head(P_BUNDLE);
      c.bundle.assign(head.data(), head.data() + head.size());
    }
    const u16 length = u16(w.size());
    const size_t at = c.bundle.size();
    c.bundle.resize(at + 2 + w.size());
    memcpy(c.bundle.data() + at, &length, 2);
    memcpy(c.bundle.data() + at + 2, w.data(), w.size());
  };

  // What has moved against what this player was last told, and a few nodes in any case.
  std::vector<u32> nodeList;
  const auto writeNodes = [&](Writer& w, const i16* now, i16* told, u32 count, u32& scan, u32& heal) {
    nodeList.clear();
    for (u32 n = 0; n < count && nodeList.size() < size_t(kMaxNodesPerPacket); n++) {
      const u32 k = (scan + n) % count;
      if (std::abs(now[k * 3] - told[k * 3]) >= kNodeChange || std::abs(now[k * 3 + 1] - told[k * 3 + 1]) >= kNodeChange || std::abs(now[k * 3 + 2] - told[k * 3 + 2]) >= kNodeChange) nodeList.push_back(k);
    }
    // Whatever did not fit comes first next time.
    if (nodeList.size() == size_t(kMaxNodesPerPacket)) scan = (nodeList.back() + 1) % count;
    for (int n = 0; n < kNodeHeal; n++) {
      nodeList.push_back(heal);
      heal = (heal + 1) % count;
    }
    w.put(u16(nodeList.size()));
    for (u32 k : nodeList) {
      w.put(u16(k));
      for (int a = 0; a < 3; a++) {
        w.put(now[k * 3 + u32(a)]);
        told[k * 3 + u32(a)] = now[k * 3 + u32(a)];
      }
    }
  };

  // Worked out once a tick, for however many players want it.
  struct PoseNow {
    u32 tick = 0xffffffffu;
    VehiclePose pose;
    std::vector<i16> locals;
  };
  PoseNow tankNow[Duel::kTanks], carNow[Duel::kCars];

  const auto sendTank = [&](Client& c, int i) {
    const Tank& t = duel->tanks[i];
    PoseNow& pn = tankNow[i];
    if (pn.tick != tick) {
      pn.tick = tick;
      pn.pose = VehicleShapes::tankPose(t);
      pn.locals.resize(size_t(duel->tankNodes) * 3);
      shapes->tankLocals(i, pn.pose, duel->sim.pos.data() + duel->tankFirst[i], pn.locals.data());
    }
    Writer w(P_TANK);
    w.put(simStep);
    w.put(duel->matchId);
    w.put(u8(i));
    w.putV3(pn.pose.cog);
    for (i16 v : pn.pose.q) w.put(v);
    writeState(w, t.netState(), pn.pose.cog, (tick + u32(i)) % kSlowEvery == 0);
    writeNodes(w, pn.locals.data(), c.toldTanks.data() + shapes->tankAt(i), duel->tankNodes, c.scanTank[i], c.healTank[i]);
    post(c, w);
  };

  const auto sendCar = [&](Client& c, int k) {
    const VehicleFrame& f = duel->frames[Duel::kTanks + k];
    const Vehicle& v = duel->cars[size_t(k)];
    const u32 count = shapes->carCount(k);
    PoseNow& pn = carNow[k];
    if (pn.tick != tick) {
      pn.tick = tick;
      pn.pose.cog = f.cog;
      packAxes(f.fwd, f.left, f.up, pn.pose.q);
      pn.locals.resize(size_t(count) * 3);
      shapes->carLocals(k, pn.pose, duel->sim.pos.data() + duel->carFirst[k], pn.locals.data());
    }
    Writer w(P_CAR);
    w.put(simStep);
    w.put(duel->matchId);
    w.put(u8(k));
    w.putV3(pn.pose.cog);
    for (i16 q : pn.pose.q) w.put(q);
    w.put(packCm(f.speed));
    w.put(i16(std::clamp(std::lround(v.rpm()), -32767L, 32767L)));
    w.put(packShare(v.throttleAct));
    w.put(packShare(v.brakeAct));
    w.put(i8(std::clamp(std::lround(v.steerAct * 127.0), -127L, 127L)));
    w.put(i8(std::clamp(std::lround(v.latAcc * 3.0), -127L, 127L)));
    w.put(i8(std::clamp(std::lround(v.lonAcc * 3.0), -127L, 127L)));
    for (size_t t = 0; t < 4; t++) w.put(packAngle(t < v.wheels.size() ? v.wheels[t].theta : 0.0));
    w.put(i8(v.gear));
    u8 flat = 0;
    for (size_t t = 0; t < v.wheels.size() && t < 8; t++) flat |= u8(v.wheels[t].deflate > 0 ? 1 << t : 0);
    w.put(u8((v.exploded ? 1 : 0) | (v.dmg.seized ? 2 : 0) | (v.dmg.oilLeak ? 4 : 0) | (v.dmg.radiator ? 8 : 0) | (v.dmg.tank ? 16 : 0) | (v.dmg.gearbox ? 32 : 0)));
    w.put(flat);
    const bool slow = (tick + u32(k)) % kSlowEvery == 0;
    w.put(u8(slow ? 1 : 0));
    if (slow) {
      w.put(float(v.fuelLitres));
      w.put(float(v.dmg.coolantTemp));
    }
    writeNodes(w, pn.locals.data(), c.toldCars.data() + shapes->carAt(k), count, c.scanCar[k], c.healCar[k]);
    post(c, w);
  };

  // Where this player is looking from.
  const auto viewerAt = [&](int slot) {
    const Duel::Player& pl = duel->players[slot];
    if (pl.droneLive) return pl.droneP;
    return pl.vehicle >= 0 ? duel->frames[pl.vehicle].cog : pl.soldier.p;
  };

  const auto sendMeta = [&](Client& c, int slot) {
    Writer w(P_META);
    w.put(simStep);
    w.put(duel->matchId);
    w.put(tick);
    w.put(u8(duel->phase));
    w.put(i8(duel->winner));
    w.put(float(duel->phaseClock));
    u8 here = 0;
    {
      u8 ready = 0;
      for (int k = 0; k < Duel::kMaxPlayers; k++) {
        if (clients[k].used) here |= u8(1 << k);
        if (duel->players[k].ready) ready |= u8(1 << k);
      }
      w.put(here);
      w.put(ready);
      w.put(u8(duel->playersWanted));
    }
    // Oneself with every tick, the others by how far away they are.
    u8 told = 0;
    for (int k = 0; k < Duel::kMaxPlayers; k++) {
      if (!clients[k].used) continue;
      if (k == slot || (tick + u32(k)) % sendEvery(len(duel->players[k].soldier.p - viewerAt(slot))) == 0) told |= u8(1 << k);
    }
    w.put(told);
    for (int k = 0; k < Duel::kMaxPlayers; k++) {
      if (!clients[k].used) continue;
      w.put(i8(duel->players[k].vehicle));
      w.put(u8(duel->players[k].seat));
    }
    for (int k = 0; k < Duel::kMaxPlayers; k++) {
      if (!(told & (1 << k))) continue;
      const Duel::Player& pl = duel->players[k];
      w.putV3(pl.soldier.p);
      putCmV3(w, pl.soldier.v);
      w.put(packAngle(pl.soldier.yaw));
      w.put(packAngle(pl.soldier.pitch));
      // Rounded up: only who has really nothing left counts as dead.
      w.put(i16(std::clamp(std::ceil(pl.soldier.hp * 10.0), -32000.0, 32000.0)));
      w.put(float(pl.soldier.stride));
      w.put(packShare(pl.soldier.prone));
      w.put(u8(pl.weapon));
      const bool dead = pl.soldier.hp <= 0;
      w.put(u8((pl.items[IT_GHILLIE] > 0 ? 1 : 0) | (pl.droneLive ? 2 : 0) | (dead ? 4 : 0)));
      if (dead) putCmV3(w, pl.lastBlow);
      if (pl.droneLive) {
        w.putV3(pl.droneP);
        w.putV3(pl.droneV);
        w.put(float(pl.droneBattery));
      }
    }
    w.put(duel->players[slot].lastInput);
    w.put(duel->players[slot].walkedSinceInput);
    w.put(u8(duel->players[slot].soldier.grounded ? 1 : 0));
    w.put(duel->players[slot].hitsLanded);
    w.put(duel->players[slot].hitsTaken);
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
      w.put(float(log[i].hole));
    }
    const size_t nEv = std::min(events.size(), size_t(kMaxEventsPerPacket));
    {
      // What flies under its own power, to be drawn where it is.
      u8 rockets = 0;
      for (const Duel::Bullet& b : duel->bullets)
        if ((b.kind == WP_RPG || b.kind == Duel::kKindStrike) && rockets < 8) rockets++;
      w.put(rockets);
      int sent = 0;
      for (const Duel::Bullet& b : duel->bullets)
        if ((b.kind == WP_RPG || b.kind == Duel::kKindStrike) && sent < 8) {
          w.put(u8(b.kind == WP_RPG ? 0 : 1));
          w.putV3(b.p);
          w.putV3(b.v);
          sent++;
        }
    }
    w.put(u8(nEv));
    for (size_t i = 0; i < nEv; i++) {
      w.putV3(events[i].p);
      w.putV3(events[i].v);
      w.put(events[i].strength);
      w.put(u8(events[i].kind));
    }
    post(c, w);
  };

  // What this player has and how things stand: it need not come with every tick.
  const auto sendSelf = [&](Client& c, int slot) {
    Writer w(P_SELF);
    w.put(simStep);
    w.put(duel->matchId);
    w.put(duel->wins[0]);
    w.put(duel->wins[1]);
    const Duel::Player& me = duel->players[slot];
    for (int k = 0; k < IT_COUNT; k++) w.put(me.items[k]);
    w.put(float(me.repairClock));
    w.put(i16(me.smallAmmo));
    w.put(i16(me.sniperAmmo));
    w.put(i16(me.rpgAmmo));
    u8 has = 0;
    for (int k = 0; k < WP_COUNT; k++) has |= u8(me.hasWeapon[k] ? 1 << k : 0);
    has |= u8((me.thermalSight ? kHasThermal : 0) | (me.weakSight ? kHasWeak : 0));
    w.put(has);
    w.putV3(me.hurtFrom);
    w.putV3(duel->wind);
    w.put(i16(me.tankShells));
    w.put(i16(me.tankMissiles));
    w.put(i16(me.strikeCodes));
    w.put(u8(me.drones));
    w.put(u8(me.medkits));
    // How the server is doing: the player's own machine has no physics to tell by.
    w.put(u8(std::min(statLoad * 100.0, 255.0)));
    w.put(u8(std::min(statReal * 100.0, 255.0)));
    w.put(u16(duel->sim.active.size()));
    w.put(u8((me.tapped ? 1 : 0) | (me.hasFix ? 2 : 0)));
    w.put(float(me.tapClock));
    w.put(float(me.fixWait));
    w.put(float(me.fixAge));
    w.putV3(me.fix);
    post(c, w);
  };

  const auto letOut = [&](Client& c, double at) {
    while (!c.waiting.empty() && at >= c.nextOut) {
      sock.send(c.waiting.front().data(), c.waiting.front().size(), c.addr);
      c.waiting.pop_front();
      c.nextOut = std::max(c.nextOut + c.outGap, at - 0.002);
    }
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
      w.put(simStep);
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
      post(c, w);
    }
  };

  // ---- The commands of the console.
  const auto playerByName = [&](const std::string& n) {
    for (int i = 0; i < Duel::kMaxPlayers; i++)
      if (n == "player" + std::to_string(i + 1) || n == std::to_string(i + 1)) return i;
    for (int i = 0; i < Duel::kMaxPlayers; i++)
      if (clients[i].used && n == clients[i].name) return i;
    return -1;
  };
  shell.complete = [&](const std::vector<std::string>& words) {
    std::vector<std::string> out;
    if (words.size() == 1) {
      for (const char* c : {"/give", "/kill", "/kick", "/players", "/restart", "/wind", "/help", "/quit"}) out.push_back(c);
    } else if (words[0] == "/give" || words[0] == "/kick" || words[0] == "/kill") {
      if (words.size() == 2) {
        for (int i = 0; i < duel->playersWanted; i++) out.push_back("player" + std::to_string(i + 1));
        for (const Client& c : clients)
          if (c.used) out.push_back(c.name);
      } else if (words[0] == "/give" && words.size() == 3) {
        for (const char* n : Duel::kGiveNames) out.push_back(n);
      }
    }
    return out;
  };
  const auto command = [&](const std::string& line) {
    if (line.empty()) return;
    if (line[0] != '/') {
      // Anything else is said to everybody.
      chat.push_back({Duel::kMaxPlayers, line.substr(0, kChatBytes)});
      return;
    }
    std::vector<std::string> words;
    {
      std::string word;
      for (char ch : line + " ") {
        if (ch == ' ') {
          if (!word.empty()) words.push_back(word);
          word.clear();
        } else {
          word += ch;
        }
      }
    }
    const std::string& cmd = words[0];
    if (cmd == "/help") {
      shell.print("Text ohne / geht in den Chat. /give <spieler> <sache> [anzahl]  /kill <spieler>  /kick <spieler>  /players  /restart  /wind <m/s>  /quit");
      std::string all = "Sachen:";
      for (const char* n : Duel::kGiveNames) all += std::string(" ") + n;
      shell.print(all);
    } else if (cmd == "/give") {
      if (words.size() < 3) {
        shell.print("/give <spieler> <sache> [anzahl]");
        return;
      }
      const int who = playerByName(words[1]);
      const int count = words.size() > 3 ? atoi(words[3].c_str()) : 1;
      if (who < 0) say("Spieler %s gibt es nicht", words[1].c_str());
      else if (!duel->give(who, words[2].c_str(), count)) say("Das geht nicht: %s %d", words[2].c_str(), count);
      else say("%s bekommt %d x %s", clients[who].used ? clients[who].name : words[1].c_str(), count, words[2].c_str());
    } else if (cmd == "/kick") {
      const int who = words.size() > 1 ? playerByName(words[1]) : -1;
      if (who < 0 || !clients[who].used) {
        shell.print("Den Spieler gibt es nicht");
      } else {
        // The client notices by itself when nothing comes any more.
        clients[who].used = false;
        say("Spieler %d entfernt", who + 1);
      }
    } else if (cmd == "/kill") {
      const int who = words.size() > 1 ? playerByName(words[1]) : -1;
      if (who < 0 || !clients[who].used) {
        shell.print("Den Spieler gibt es nicht");
      } else {
        Duel::Player& pl = duel->players[who];
        pl.lastBlow = V3{-std::cos(pl.soldier.yaw), -std::sin(pl.soldier.yaw), 0.3} * -7.0;
        pl.soldier.hp = 0;
        duel->sim.event(EV_BLOOD, pl.soldier.p + V3{0, 0, 1.2}, pl.lastBlow * 0.5, 1.f);
        say("Spieler %d stirbt", who + 1);
      }
    } else if (cmd == "/players") {
      for (int i = 0; i < duel->playersWanted; i++) {
        if (!clients[i].used) {
          say("player%d: frei", i + 1);
          continue;
        }
        const Duel::Player& pl = duel->players[i];
        say("player%d: %s, %s, Leben %.0f, Fahrzeug %d, Position %.0f %.0f", i + 1, clients[i].name, toString(clients[i].addr).c_str(), pl.soldier.hp, pl.vehicle, pl.soldier.p.x, pl.soldier.p.y);
      }
    } else if (cmd == "/restart") {
      duel->restart();
      int here = 0;
      for (const Client& who : clients) here += who.used ? 1 : 0;
      duel->phase = here >= duel->playersWanted ? Duel::CHOOSING : Duel::WAITING;
      shell.print("Neues Match");
    } else if (cmd == "/wind") {
      if (words.size() > 1) duel->wind = norm(duel->wind) * atof(words[1].c_str());
      say("Wind %.1f m/s", len(duel->wind));
    } else if (cmd == "/quit") {
      gStop = 1;
    } else {
      say("Unbekannt: %s. /help zeigt die Befehle.", cmd.c_str());
    }
  };
  shell.start();

  double simTime = 0;
  double nextSend = 0;
  double busy = 0, lastReport = now();
  double logBusy = 0, logLoopGap = 0, logLoopAt = now(), logNext = 0;
  u64 logSteps = 0, logPackets = 0, logBytes = 0;
  double hasteBusy = 0, hasteNext = 0;
  u64 hasteSteps = 0;
  double wall0 = now();
  while (!gStop) {
    receive();
    {
      std::string line;
      while (shell.poll(line)) command(line);
    }
    const double t = now();
    for (int i = 0; i < Duel::kMaxPlayers; i++) {
      if (clients[i].used && t - clients[i].lastHeard > kTimeout) {
        say("Spieler %d: keine Verbindung mehr", i + 1);
        clients[i].used = false;
      }
    }
    // Never more than a fifth of a second to catch up: past that, time simply stands still for a moment.
    if (t - wall0 - simTime > 0.2) simTime = t - wall0 - 0.2;
    const auto q0 = Clock::now();
    int done = 0;
    while (simTime < t - wall0 && done < 40) {
      Duel::Intent in[Duel::kMaxPlayers];
      for (int i = 0; i < Duel::kMaxPlayers; i++) {
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
        in[i].setup = c.setup;
        in[i].placeFoot = c.mineFoot;
        in[i].placeTank = c.mineTank;
        in[i].strike = c.strike;
        in[i].query = c.query;
        in[i].smoke = c.smoke;
        in[i].net = c.net;
        in[i].drone = c.drone;
        in[i].medkit = c.medkit;
        in[i].strikeAt = V3{c.in.strikeX, c.in.strikeY, 0};
        in[i].fireFoot = c.fire;
        in[i].fireHeld = c.in.fireHeld;
        in[i].aiming = c.in.aiming;
        in[i].weapon = c.in.weapon;
        // How far behind this player's screen is: the moment they were looking at when this input went out.
        in[i].lag = std::clamp((double(simStep) - double(c.in.viewTick)) / kStepRate, 0.0, 0.3);
        in[i].repair = c.repair;
        in[i].seq = c.in.seq;
        in[i].pitch = c.in.pitch;
        in[i].walk.forward = c.in.throttle - c.in.brake;
        in[i].walk.strafe = c.in.steer;
        in[i].walk.yaw = c.in.yaw;
        in[i].walk.jump = c.in.handbrake > 0.5;
        in[i].walk.sprint = c.in.sprint;
        c.fire = c.launch = c.repair = c.use = c.setup = c.mineFoot = c.mineTank = c.strike = c.query = c.smoke = c.net = c.drone = c.medkit = false;
      }
      for (int i = 0; i < Duel::kMaxPlayers; i++) duel->players[i].present = clients[i].used;
      duel->step(in);
      simStep++;
      simTime += Duel::kDt;
      done++;
    }
    busy += std::chrono::duration<double>(Clock::now() - q0).count();
    logBusy += std::chrono::duration<double>(Clock::now() - q0).count();
    // Falling behind slows the whole world down for everyone, which is worse than anything lying still a little early.
    hasteBusy += std::chrono::duration<double>(Clock::now() - q0).count();
    hasteSteps += u64(done);
    if (t > hasteNext) {
      const double share = hasteBusy / 0.5;
      statLoad = share;
      statReal = double(hasteSteps) * Duel::kDt / 0.5;
      hasteSteps = 0;
      if (share > 0.6) duel->sim.sleepHaste = std::min(duel->sim.sleepHaste * 2.0, 8.0);
      else if (share < 0.3) duel->sim.sleepHaste = std::max(duel->sim.sleepHaste * 0.5, 1.0);
      hasteBusy = 0;
      hasteNext = t + 0.5;
    }
    logSteps += u64(done);
    if (duel->matchId != seenMatch) {
      seenMatch = duel->matchId;
      say("Match %u: Stand %u : %u", duel->matchId, duel->wins[0], duel->wins[1]);
      pending.clear();
      for (Client& c : clients)
        if (c.used) {
          c.toldTanks = shapes->restTanks;
          c.toldCars = shapes->restCars;
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
        if (e.kind == EV_TRACER && e.strength < 1.5f && (tracerCount++ & 3) != 0) continue;
        // Spend the room on what matters: a marked shot never waits behind sparks.
        if (e.kind == EV_IMPACT) pending.insert(pending.begin(), e);
        else if (pending.size() < 400) pending.push_back(e);
      }
    }
    if (t >= nextSend) {
      nextSend = std::max(nextSend + 1.0 / kSendRate, t - 0.1);
      tick++;
      events.clear();
      const size_t nEv = std::min(pending.size(), size_t(kMaxEventsPerPacket));
      events.assign(pending.begin(), pending.begin() + long(nEv));
      pending.erase(pending.begin(), pending.begin() + long(nEv));
      for (int i = 0; i < Duel::kMaxPlayers; i++) {
        Client& c = clients[i];
        if (!c.used) continue;
        sendMeta(c, i);
        if (tick % kSlowEvery == 0) sendSelf(c, i);
        if (tick % u32(kSendRate / 5.0) == 0 && (!chat.empty() || c.chatSeq)) {
          // The last few lines, again and again: whatever one missed comes round in the next one.
          Writer w(P_CHATLOG);
          w.put(simStep);
          w.put(duel->matchId);
          w.put(c.chatSeq);
          const u32 total = u32(chat.size());
          const u32 from = total > 5 ? total - 5 : 0;
          w.put(from);
          w.put(u8(total - from));
          for (u32 k = from; k < total; k++) {
            w.put(u8(chat[k].who));
            w.put(u8(chat[k].text.size()));
            for (char ch : chat[k].text) w.put(ch);
          }
          post(c, w);
        }
        if (tick % u32(kSendRate / 2.0) == 0) {
          Writer w(P_NAMES);
          w.put(simStep);
          w.put(duel->matchId);
          for (const Client& who : clients)
            for (int k = 0; k < kNameBytes; k++) w.put(who.used ? who.name[k] : char(0));
          post(c, w);
        }
        // The tanks in play every time, and now and then the others, should a packet have gone astray.
        for (int k = 0; k < Duel::kTanks; k++) {
          const Tank& t = duel->tanks[k];
          const bool inPlay = duel->occupied(k) || duel->sim.objs[t.hullObj].awake || duel->sim.objs[t.turretObj].awake;
          const bool mine = duel->players[i].vehicle == k;
          if ((inPlay && (mine || (tick + u32(k)) % sendEvery(len(t.cog - viewerAt(i))) == 0)) || tick % u32(kSendRate) == u32(k) * u32(kSendRate) / u32(Duel::kTanks)) sendTank(c, k);
        }
        // Cars while they move and now and then otherwise.
        for (int k = 0; k < Duel::kCars; k++) {
          const Vehicle& v = duel->cars[size_t(k)];
          const bool mine = duel->players[i].vehicle == Duel::kTanks + k;
          if ((duel->sim.objs[v.obj].awake && (mine || (tick + u32(k)) % sendEvery(len(duel->frames[Duel::kTanks + k].cog - viewerAt(i))) == 0)) || (tick + 7) % u32(2.0 * kSendRate) == u32(k)) sendCar(c, k);
        }
        if (tick % u32(kSendRate / 30.0) == 0) sendScenery(c);
        flush(c);
        // Within three quarters of the tick, whatever is left of the last one first.
        c.outGap = 0.75 / kSendRate / double(std::max<size_t>(c.waiting.size(), 1));
        c.nextOut = std::min(c.nextOut, t);
      }
    }
    for (Client& c : clients)
      if (c.used) letOut(c, now());
    if (playLog) {
      logLoopGap = std::max(logLoopGap, t - logLoopAt);
      logLoopAt = t;
      if (t > logNext) {
        int awakeVehicles = 0;
        for (int k = 0; k < Duel::kVehicles; k++) {
          const u16 hull = k < Duel::kTanks ? duel->tanks[k].hullObj : duel->cars[size_t(k - Duel::kTanks)].obj;
          if (duel->sim.objs[hull].awake) awakeVehicles++;
        }
        double nearest = 1e9;
        for (int a = 0; a < Duel::kMaxPlayers; a++)
          for (int b = a + 1; b < Duel::kMaxPlayers; b++) {
            if (!clients[a].used || !clients[b].used) continue;
            const auto where = [&](int k) { return duel->players[k].vehicle >= 0 ? duel->frames[duel->players[k].vehicle].cog : duel->players[k].soldier.p; };
            nearest = std::min(nearest, len(where(a) - where(b)));
          }
        fprintf(playLog, "t %.0f phase %d spieler-abstand %.0f m schritte %llu arbeit %.0f ms schleifenpause %.1f ms aktiv %zu eile %.0f fahrzeuge-wach %d gesendet %llu pakete %.0f kB abgelehnt %llu\n", t, duel->phase, nearest, (unsigned long long)logSteps, logBusy * 1000.0, logLoopGap * 1000.0, duel->sim.active.size(), duel->sim.sleepHaste, awakeVehicles, (unsigned long long)(sock.sentPackets - logPackets), double(sock.sentBytes - logBytes) / 1024.0, (unsigned long long)sock.sendFailed);
        fflush(playLog);
        logSteps = 0;
        logBusy = 0;
        logLoopGap = 0;
        logPackets = sock.sentPackets;
        logBytes = sock.sentBytes;
        logNext = t + 1.0;
      }
    }
    if (t - lastReport > 30.0) {
      if (getenv("BEANNG_LOAD")) say("Physik-Last %.0f %%, wach %zu, Spieler %d", busy / (t - lastReport) * 100.0, duel->sim.active.size(), int(clients[0].used) + int(clients[1].used));
      busy = 0;
      lastReport = t;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }
  shell.stop();
  printf("Server beendet\n");
  delete shapes;
  delete duel;
  return 0;
}
