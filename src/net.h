// The wire between the duel server and its two players: plain UDP, no dependencies.
//
// The server alone runs the physics. A player sends what the controls are doing and gets back where everything
// is. Nothing here is resent as such: every packet carries the latest state, so a lost one is simply replaced by
// the next. What must not get lost (a blast, which digs a crater on every machine) stays in the state until the
// player has acknowledged it. One-shot controls are counters for the same reason.
#pragma once
#include <cstring>
#include <string>
#include <vector>
#include "mathx.h"

namespace net {

constexpr u32 kMagic = 0x31474e42;  // "BNG1"
constexpr u16 kProtocol = 13;
constexpr u16 kDefaultPort = 27015;
constexpr size_t kMaxPacket = 1200;  // safely under any path's MTU
constexpr int kNameBytes = 24;
constexpr int kChatBytes = 120;

enum PacketKind : u8 { P_HELLO, P_WELCOME, P_FULL, P_INPUT, P_TANK, P_META, P_SCENERY, P_BYE, P_CAR, P_NAMES, P_CHAT, P_CHATLOG };

struct Addr {
  u32 ip = 0;
  u16 port = 0;
  bool operator==(const Addr& o) const { return ip == o.ip && port == o.port; }
};
// Accepts a dotted address or a host name. Blocks while it looks the name up.
bool resolve(const char* host, u16 port, Addr& out);
std::string toString(const Addr& a);

class Socket {
 public:
  Socket() = default;
  ~Socket();
  Socket(const Socket&) = delete;
  Socket& operator=(const Socket&) = delete;
  bool open(u16 port);  // 0 for any free port; the socket never blocks
  int recv(void* buf, size_t cap, Addr& from);  // bytes, or -1 when nothing is waiting
  void send(const void* buf, size_t n, const Addr& to);

 private:
  int fd_ = -1;
};

// Little-endian on every machine this builds for, so the bytes go as they are.
class Writer {
 public:
  explicit Writer(PacketKind kind) {
    put(kMagic);
    put(u8(kind));
  }
  template <class T>
  void put(const T& v) {
    const size_t at = data_.size();
    data_.resize(at + sizeof(T));
    memcpy(data_.data() + at, &v, sizeof(T));
  }
  void putV3(V3 v) {
    put(float(v.x));
    put(float(v.y));
    put(float(v.z));
  }
  size_t size() const { return data_.size(); }
  const u8* data() const { return data_.data(); }

 private:
  std::vector<u8> data_;
};

// Whatever arrives may be nonsense: every read is checked, and after one failure all further reads give zeros.
class Reader {
 public:
  Reader(const void* p, size_t n) : p_(static_cast<const u8*>(p)), n_(n) {}
  // True if this is one of ours; the kind comes out of the header.
  bool header(u8& kind) {
    u32 magic = 0;
    get(magic);
    get(kind);
    return ok_ && magic == kMagic;
  }
  template <class T>
  void get(T& v) {
    if (!ok_ || at_ + sizeof(T) > n_) {
      ok_ = false;
      v = T{};
      return;
    }
    memcpy(&v, p_ + at_, sizeof(T));
    at_ += sizeof(T);
  }
  V3 getV3() {
    float x = 0, y = 0, z = 0;
    get(x);
    get(y);
    get(z);
    return V3{x, y, z};
  }
  bool ok() const { return ok_; }

 private:
  const u8* p_;
  size_t n_, at_ = 0;
  bool ok_ = true;
};

// What a player does, sixty times a second.
struct InputMsg {
  u32 token = 0;      // from the welcome; a stranger cannot steer someone else's tank by guessing the port
  u32 seq = 0;
  float steer = 0, throttle = 0, brake = 0, handbrake = 0;
  V3 aim;
  u16 fireCount = 0, launchCount = 0, repairCount = 0, useCount = 0, setupCount = 0, mineFootCount = 0, mineTankCount = 0, strikeCount = 0, queryCount = 0;
  float strikeX = 0, strikeY = 0;
  float yaw = 0, pitch = 0;  // the camera: where one walks and looks on foot
  bool sprint = false;
  bool fireHeld = false;
  u8 weapon = 1;
  bool indirect = false;  // aim is a point marked on the map
  i8 requestGear = 0;  // 0 nothing, otherwise forward or reverse, for as long as it is wanted
  // What this player has chosen for the match, in every packet until the match has begun.
  u8 ready = 0;
  u8 items[8] = {};
  u32 ackBlasts = 0;  // blasts of this match this player has already set off
  u32 ackMatch = 0;
  u32 viewTick = 0;   // which server moment the player is looking at: hits by hand weapons will be judged there
};
void write(Writer& w, const InputMsg& m);
bool read(Reader& r, InputMsg& m);  // false if malformed; values are sanitised and always safe to use

}  // namespace net
