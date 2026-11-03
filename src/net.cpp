#include "net.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cmath>
#include <cstdio>

namespace net {

bool resolve(const char* host, u16 port, Addr& out) {
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  addrinfo* res = nullptr;
  if (getaddrinfo(host, nullptr, &hints, &res) != 0 || !res) return false;
  const auto* in = reinterpret_cast<const sockaddr_in*>(res->ai_addr);
  out.ip = ntohl(in->sin_addr.s_addr);
  out.port = port;
  freeaddrinfo(res);
  return true;
}

std::string toString(const Addr& a) {
  char b[32];
  snprintf(b, sizeof b, "%u.%u.%u.%u:%u", (a.ip >> 24) & 255, (a.ip >> 16) & 255, (a.ip >> 8) & 255, a.ip & 255, a.port);
  return b;
}

Socket::~Socket() {
  if (fd_ >= 0) close(fd_);
}

bool Socket::open(u16 port) {
  fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) return false;
  fcntl(fd_, F_SETFL, fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);
  // Room for a burst, should the receiving thread be held up for a moment.
  const int room = 1 << 20;
  setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &room, sizeof room);
  setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &room, sizeof room);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  a.sin_port = htons(port);
  return bind(fd_, reinterpret_cast<const sockaddr*>(&a), sizeof a) == 0;
}

int Socket::recv(void* buf, size_t cap, Addr& from) {
  sockaddr_in a{};
  socklen_t len = sizeof a;
  const ssize_t n = recvfrom(fd_, buf, cap, 0, reinterpret_cast<sockaddr*>(&a), &len);
  if (n < 0) return -1;
  from.ip = ntohl(a.sin_addr.s_addr);
  from.port = ntohs(a.sin_port);
  return int(n);
}

void Socket::send(const void* buf, size_t n, const Addr& to) {
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = htonl(to.ip);
  a.sin_port = htons(to.port);
  sendto(fd_, buf, n, 0, reinterpret_cast<const sockaddr*>(&a), sizeof a);
}

void write(Writer& w, const InputMsg& m) {
  w.put(m.token);
  w.put(m.seq);
  w.put(m.steer);
  w.put(m.throttle);
  w.put(m.brake);
  w.put(m.handbrake);
  w.putV3(m.aim);
  w.put(m.fireCount);
  w.put(m.launchCount);
  w.put(m.repairCount);
  w.put(u8(m.indirect ? 1 : 0));
  w.put(m.requestGear);
  w.put(m.turret);
  w.put(m.ready);
  for (u8 v : m.items) w.put(v);
  w.put(m.ackBlasts);
  w.put(m.ackMatch);
  w.put(m.viewTick);
}

static float sane(float v, float lo, float hi) {
  if (!std::isfinite(v)) return 0;
  return v < lo ? lo : (v > hi ? hi : v);
}

bool read(Reader& r, InputMsg& m) {
  r.get(m.token);
  r.get(m.seq);
  r.get(m.steer);
  r.get(m.throttle);
  r.get(m.brake);
  r.get(m.handbrake);
  m.aim = r.getV3();
  r.get(m.fireCount);
  r.get(m.launchCount);
  r.get(m.repairCount);
  u8 indirect = 0;
  r.get(indirect);
  m.indirect = indirect != 0;
  r.get(m.requestGear);
  r.get(m.turret);
  r.get(m.ready);
  for (u8& v : m.items) r.get(v);
  r.get(m.ackBlasts);
  r.get(m.ackMatch);
  r.get(m.viewTick);
  if (!r.ok()) return false;
  // The server must survive anything a player's machine sends it.
  m.steer = sane(m.steer, -1, 1);
  m.throttle = sane(m.throttle, 0, 1);
  m.brake = sane(m.brake, 0, 1);
  m.handbrake = sane(m.handbrake, 0, 1);
  m.aim = V3{sane(float(m.aim.x), -1e5f, 1e5f), sane(float(m.aim.y), -1e5f, 1e5f), sane(float(m.aim.z), -1e5f, 1e5f)};
  return true;
}

}  // namespace net
