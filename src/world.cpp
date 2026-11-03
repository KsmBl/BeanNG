#include "world.h"
#include <algorithm>

namespace {
double hash2(int x, int y) {
  u32 h = u32(x) * 374761393u + u32(y) * 668265263u;
  h = (h ^ (h >> 13)) * 1274126177u;
  return double((h ^ (h >> 16)) & 0xffffff) / double(0xffffff);
}
double vnoise(double x, double y) {
  const int xi = int(std::floor(x)), yi = int(std::floor(y));
  const double fx = smooth01(x - xi), fy = smooth01(y - yi);
  const double a = hash2(xi, yi), b = hash2(xi + 1, yi), c = hash2(xi, yi + 1),
               d = hash2(xi + 1, yi + 1);
  return lerpd(lerpd(a, b, fx), lerpd(c, d, fx), fy) * 2.0 - 1.0;
}
// The mountain: in the north-east corner, clear of the country road. Its slope flattens out at the foot, and
// ridges and gullies run down it so that it does not look turned on a lathe.
constexpr double kPeakX = 620, kPeakY = 620, kPeakR = 400, kPeakH = 270;
double mountain(double x, double y) {
  const double dx = x - kPeakX, dy = y - kPeakY, d = std::hypot(dx, dy) / kPeakR;
  if (d >= 1.0) return 0.0;
  const double a = std::atan2(dy, dx);
  const double e = 1.0 - d * d;
  const double cone = std::pow(e, 1.55);
  const double ridges = 0.05 * std::sin(a * 3.0 + 0.7) + 0.025 * std::sin(a * 7.0 + 2.0 + d * 3.0);
  const double rough = 6.0 * vnoise(x / 47.0 + 9.0, y / 47.0 - 3.0) + 1.6 * vnoise(x / 13.0, y / 13.0 + 21.0);
  return kPeakH * cone * (1.0 + ridges * d * 2.2) + rough * e * (0.2 + d);
}

double baseTerrain(double x, double y) {
  return 6.5 * vnoise(x / 210.0 + 3.1, y / 210.0 - 7.7) + 2.0 * vnoise(x / 80.0, y / 80.0 + 11.0) +
         0.35 * vnoise(x / 23.0 + 5.0, y / 23.0);
}
// Central test pad is exactly flat at z = 0.
constexpr double kPadR = 165.0;
double padBlend(double x, double y) { return smoothstepd(kPadR, kPadR + 70.0, std::hypot(x, y)); }


// ---- The duel's map. Two kilometres across, a valley running west to east between the bases, hills to hide behind,
// and a rim of mountains all round that nothing can climb. The land is the same beyond the grid's edge.
constexpr double kBaseX = 730.0, kYardHalfX = 42.0, kYardHalfY = 62.0, kYardZ = 16.0;
// The river runs north to south through the middle, in a gully too steep to climb out of: the bridges are the only
// way across. Far out it dies away into the rim.
constexpr double kWaterZ = 2.5, kBedZ = -1.5, kRiverHalf = 13.0, kBankOuter = 21.0;
double riverX(double y) { return 34.0 * std::sin(y / 170.0 + 0.6) + 12.0 * std::sin(y / 61.0); }
// Two places where the banks lie flat and the water is shallow: anything can cross here, slowly, and in plain sight.
constexpr double kFords[2] = {-330.0, 345.0};
double fordAt(double y) {
  double f = 0;
  for (double yf : kFords) f = std::max(f, 1.0 - smoothstepd(16.0, 34.0, std::fabs(y - yf)));
  return f;
}
double riverBedAt(double y) { return lerpd(kBedZ, kWaterZ - 0.75, fordAt(y)); }
double riverCut(double x, double y) {
  // 1 in the water, 0 up on the land.
  const double outer = lerpd(kBankOuter, 70.0, fordAt(y));
  return (1.0 - smoothstepd(kRiverHalf, outer, std::fabs(x - riverX(y)))) * (1.0 - smoothstepd(790.0, 840.0, std::fabs(y)));
}
// Concrete barriers either side of the middle, higher than any gun can look over: past them only artillery reaches,
// or whoever goes round by one of the gates.
constexpr double kBarrierX = 380.0, kBarrierHigh = 10.0, kBarrierSection = 16.0;
double yardBlend(double x, double y) {
  // 1 on the yards, falling off to nothing over the ground around them.
  double out = 0;
  for (double bx : {-kBaseX, kBaseX}) {
    const double dx = std::max(std::fabs(x - bx) - kYardHalfX, 0.0), dy = std::max(std::fabs(y) - kYardHalfY, 0.0);
    out = std::max(out, 1.0 - smoothstepd(0.0, 45.0, std::hypot(dx, dy)));
  }
  return out;
}
double arenaTerrain(double x, double y) {
  // Long rolling ground, and a few real hills between the bases.
  double h = 19.0 + 13.0 * vnoise(x / 260.0 + 3.1, y / 260.0 - 7.7) + 4.5 * vnoise(x / 90.0, y / 90.0 + 11.0) + 0.5 * vnoise(x / 23.0 + 5.0, y / 23.0);
  // {x, y, radius, height}: big ones to go round, and many small ones a tank can just about hide behind.
  static constexpr double kHills[22][4] = {
      {-210, 300, 190, 46}, {240, -310, 210, 52}, {-40, -60, 130, 22}, {430, 330, 170, 38}, {-470, -330, 180, 40}, {60, 560, 200, 34},
      {-560, 210, 70, 13}, {-420, 90, 60, 11}, {-280, -170, 65, 12}, {-150, 170, 55, 10}, {130, -120, 60, 11}, {300, 130, 70, 13},
      {520, -160, 65, 12}, {590, 130, 55, 10}, {-610, -130, 60, 11}, {-330, 500, 75, 15}, {330, -520, 75, 15}, {130, 300, 60, 11},
      {-110, -320, 70, 13}, {470, -360, 60, 10}, {-500, 560, 80, 16}, {560, 520, 80, 16},
  };
  for (const auto& hill : kHills) {
    const double d = std::hypot(x - hill[0], y - hill[1]) / hill[2];
    if (d < 1.0) h += hill[3] * std::pow(1.0 - d * d, 2.0);
  }
  // Earth banks along the fields, as long as a street and higher than a tank: {x0, y0, x1, y1}.
  static constexpr double kBanks[14][4] = {
      {-640, 34, -640, 92}, {640, 34, 640, 92}, {-500, 110, -400, 190}, {-500, -110, -400, -190}, {400, 190, 500, 110}, {400, -190, 500, -110},
      {-250, -80, -190, 30}, {190, -30, 250, 80}, {-260, 150, -150, 110}, {150, -110, 260, -150}, {-80, 170, 70, 200}, {-70, -190, 80, -170},
      {-330, -330, -220, -420}, {220, 420, 330, 330},
  };
  for (const auto& bank : kBanks) {
    const double dx = bank[2] - bank[0], dy = bank[3] - bank[1];
    const double t = clampd(((x - bank[0]) * dx + (y - bank[1]) * dy) / (dx * dx + dy * dy), 0, 1);
    const double d = std::hypot(x - (bank[0] + dx * t), y - (bank[1] + dy * t)) / 11.0;
    if (d < 1.0) {
      const double ends = smoothstepd(0.0, 0.12, t) * smoothstepd(0.0, 0.12, 1.0 - t);
      h += 4.6 * ends * std::pow(1.0 - d * d, 1.5);
    }
  }
  // The rim. It starts gently, so that it looks like land, and ends far too steep to drive.
  // Past the map's edge it stays up there, as a ridge with peaks on it: nobody gets to see behind it.
  const double out = std::min(std::max({std::fabs(x), std::fabs(y)}) - 820.0, 240.0);
  if (out > 0) {
    const double t = out / 200.0;
    h += 300.0 * t * t * (1.0 + 0.15 * vnoise(x / 70.0, y / 70.0)) + 90.0 * t * (0.5 + 0.5 * vnoise(x / 150.0 + 40.0, y / 150.0 - 11.0));
  }
  const double yard = yardBlend(x, y);
  return lerpd(h, kYardZ, yard);
}

constexpr double kPaveX0 = -130, kPaveX1 = -60, kPaveY0 = 70, kPaveY1 = 82;
constexpr double kWashX0 = -130, kWashX1 = -60, kWashY0 = 90, kWashY1 = 102;
}  // namespace

double World::micro(double x, double y) const {
  if (arena_) return 0.0;
  // Rough test surfaces are analytic because they are far finer than the 1 m grid.
  if (y > kPaveY0 && y < kPaveY1 && x > kPaveX0 && x < kPaveX1) {
    // Belgian pave: irregular cobbles about 30 cm across, up to 2.5 cm high.
    const double nx = std::floor(x / 0.3), ny = std::floor(y / 0.3);
    const double fx = x / 0.3 - nx - 0.5, fy = y / 0.3 - ny - 0.5;
    const double crown = 1.0 - 4.0 * std::max(fx * fx, fy * fy);
    return 0.025 * (0.4 + 0.6 * hash2(int(nx), int(ny))) * crown;
  }
  if (y > kWashY0 && y < kWashY1 && x > kWashX0 && x < kWashX1)
    return 0.03 * (1.0 + std::sin(x * 2.0 * PI / 1.3));  // washboard
  return 0.0;
}

double World::outerHeight(double x, double y) const {
  if (arena_) return arenaTerrain(x, y);
  return levelled_ ? 0.0 : baseTerrain(x, y);
}

double World::height(double x, double y) const {
  // The grid ends at exactly these hills, so the two meet without a step.
  if (x <= -kHalf || x >= kHalf || y <= -kHalf || y >= kHalf) return outerHeight(x, y);
  const double fx = clampd(x + kHalf, 0.0, double(N - 1) - 1e-6);
  const double fy = clampd(y + kHalf, 0.0, double(N - 1) - 1e-6);
  const int ix = int(fx), iy = int(fy);
  const double tx = fx - ix, ty = fy - iy;
  const float* row = &h_[size_t(iy) * N + ix];
  const double a = row[0], b = row[1], c = row[N], d = row[N + 1];
  // Two triangles, so the surface is continuous and matches the rendered mesh.
  double z;
  if (tx + ty < 1.0)
    z = a + (b - a) * tx + (c - a) * ty;
  else
    z = d + (c - d) * (1.0 - tx) + (b - d) * (1.0 - ty);
  return z + micro(x, y);
}

V3 World::terrainNormal(double x, double y) const {
  constexpr double e = 0.5;
  const double dx = height(x + e, y) - height(x - e, y);
  const double dy = height(x, y + e) - height(x, y - e);
  return norm({-dx, -dy, 2.0 * e});
}

u8 World::surfaceAt(double x, double y) const {
  const int ix = std::clamp(int(x + kHalf + 0.5), 0, N - 1);
  const int iy = std::clamp(int(y + kHalf + 0.5), 0, N - 1);
  return surf_[size_t(iy) * N + ix];
}

bool World::contact(V3 p, Contact& out) const {
  bool hit = false;
  const double hz = height(p.x, p.y);
  if (p.z < hz + 0.5) {  // cheap test first
    const double d0 = hz - p.z;
    if (d0 > 0) {
      out.n = terrainNormal(p.x, p.y);
      out.depth = d0 * out.n.z;
      out.surface = surfaceAt(p.x, p.y);
      hit = true;
    }
  }
  const int gx = int((p.x + kHalf) / kBoxCell), gy = int((p.y + kHalf) / kBoxCell);
  if (gx >= 0 && gy >= 0 && gx < kBoxGrid && gy < kBoxGrid) {
    const auto& cell = boxGrid_[size_t(gy) * kBoxGrid + gx];
    for (u16 bi : cell) {
      const StaticBox& b = boxes_[bi];
      const V3 q = p - b.c;
      const double lx = dot(q, b.ax[0]), ly = dot(q, b.ax[1]), lz = dot(q, b.ax[2]);
      const double px = b.h.x - std::fabs(lx), py = b.h.y - std::fabs(ly), pz = b.h.z - std::fabs(lz);
      if (px <= 0 || py <= 0 || pz <= 0) continue;
      double depth;
      V3 n;
      // Prefer pushing out of the top so a node entering at speed does not exit sideways
      // through a thin slab unless it really is closest to that face.
      if (pz <= px && pz <= py) {
        depth = pz;
        n = lz > 0 ? b.ax[2] : -b.ax[2];
      } else if (px <= py) {
        depth = px;
        n = lx > 0 ? b.ax[0] : -b.ax[0];
      } else {
        depth = py;
        n = ly > 0 ? b.ax[1] : -b.ax[1];
      }
      if (!hit || depth > out.depth) {
        out.depth = depth;
        out.n = n;
        out.surface = b.surface;
        hit = true;
      }
    }
  }
  return hit;
}

void World::flatten() {
  levelled_ = true;
  std::fill(h_.begin(), h_.end(), 0.f);
}

void World::addCrater(double x, double y, double r, double depth) {
  if (h0_.empty()) {
    h0_ = h_;
    surf0_ = surf_;
  }
  craters_.push_back({x, y, r});
  const double reach = r * 1.5;
  const int x0 = std::clamp(int(x + kHalf - reach), 0, N - 1), x1 = std::clamp(int(x + kHalf + reach) + 1, 0, N - 1);
  const int y0 = std::clamp(int(y + kHalf - reach), 0, N - 1), y1 = std::clamp(int(y + kHalf + reach) + 1, 0, N - 1);
  for (int iy = y0; iy <= y1; iy++)
    for (int ix = x0; ix <= x1; ix++) {
      const double d = std::hypot(ix - kHalf - x, iy - kHalf - y) / r;
      if (d >= 1.5) continue;
      const size_t id = size_t(iy) * N + ix;
      // A bowl, and the dug-out earth thrown up in a low ring around it.
      double dz = 0;
      if (d < 1.0) {
        const double k = 1.0 - d * d;
        dz = -depth * k * k;
      }
      const double ring = (d - 1.02) / 0.34;
      dz += 0.16 * depth * std::exp(-ring * ring);
      // Repeated hits deepen it, but never dig through the bottom of the world.
      h_[id] = std::max(float(h_[id] + dz), float(h0_[id] - 7.0));
      if (d < 1.1) surf_[id] = SURF_GRAVEL;
    }
}

void World::clearCraters() {
  if (!boxes0_.empty()) {
    // assign() keeps the storage: the renderer may be looking at this list.
    boxes_.assign(boxes0_.begin(), boxes0_.end());
    boxGrid_ = boxGrid0_;
    boxes0_.clear();
    boxGeneration_++;
  }
  if (h0_.empty()) return;
  h_ = h0_;
  surf_ = surf0_;
  craters_.clear();
}

std::vector<std::pair<V3, double>> World::breakRocks(V3 c, double reach) {
  std::vector<std::pair<V3, double>> out;
  const auto rnd = [&](double a, double b) {
    chipSeed_ = chipSeed_ * 1664525u + 1013904223u;
    return a + (b - a) * double(chipSeed_ >> 16) / 65535.0;
  };
  const size_t count = boxes_.size();
  for (size_t i = 0; i < count; i++) {
    StaticBox& b = boxes_[i];
    if (b.surface != SURF_ROCK || b.h.x <= 0) continue;
    const double size = std::max({b.h.x, b.h.y, b.h.z});
    if (len(b.c - c) > reach + size) continue;
    if (boxes0_.empty()) {
      boxes0_ = boxes_;
      boxGrid0_ = boxGrid_;
    }
    const StaticBox old = b;
    out.push_back({old.c, size});
    // Taken out of the collision grid, and with no size left it is not drawn either.
    const double rad = len(old.h) + 1.0;
    const int x0 = std::max(0, int((old.c.x - rad + kHalf) / kBoxCell)), x1 = std::min(kBoxGrid - 1, int((old.c.x + rad + kHalf) / kBoxCell));
    const int y0 = std::max(0, int((old.c.y - rad + kHalf) / kBoxCell)), y1 = std::min(kBoxGrid - 1, int((old.c.y + rad + kHalf) / kBoxCell));
    for (int y = y0; y <= y1; y++)
      for (int x = x0; x <= x1; x++) {
        auto& cell = boxGrid_[size_t(y) * kBoxGrid + x];
        cell.erase(std::remove(cell.begin(), cell.end(), u16(i)), cell.end());
      }
    b.h = {0, 0, 0};
    if (size < 0.45 || boxes_.size() > 60000) continue;
    const int chips = 4 + int(rnd(0, 3));
    for (int k = 0; k < chips; k++) {
      const double a = rnd(0, 6.283), d = size * rnd(0.3, 1.1);
      const double x = old.c.x + std::cos(a) * d, y = old.c.y + std::sin(a) * d;
      const double chip = size * rnd(0.22, 0.42);
      addBox({x, y, height(x, y) + chip * 0.3}, {chip * rnd(0.7, 1.3), chip * rnd(0.7, 1.2), chip * rnd(0.5, 0.9)}, rnd(0, 3.1), rnd(-0.4, 0.4), SURF_ROCK, old.r, old.g, old.b);
    }
  }
  if (!out.empty()) boxGeneration_++;
  return out;
}

void World::pave() { std::fill(surf_.begin(), surf_.end(), u8(SURF_ASPHALT)); }

void World::clearBoxes() {
  boxes_.clear();
  for (auto& c : boxGrid_) c.clear();
}

void World::addBox(V3 c, V3 half, double yaw, double pitch, u8 surf, float r, float g, float b) {
  StaticBox s;
  s.c = c;
  s.h = half;
  s.surface = surf;
  s.r = r;
  s.g = g;
  s.b = b;
  const double cy = std::cos(yaw), sy = std::sin(yaw), cp = std::cos(pitch), sp = std::sin(pitch);
  s.ax[0] = {cy * cp, sy * cp, -sp};
  s.ax[1] = {-sy, cy, 0};
  s.ax[2] = cross(s.ax[0], s.ax[1]);
  const u16 id = u16(boxes_.size());
  boxes_.push_back(s);

  const double rad = len(half) + 1.0;
  const int x0 = std::max(0, int((c.x - rad + kHalf) / kBoxCell));
  const int x1 = std::min(kBoxGrid - 1, int((c.x + rad + kHalf) / kBoxCell));
  const int y0 = std::max(0, int((c.y - rad + kHalf) / kBoxCell));
  const int y1 = std::min(kBoxGrid - 1, int((c.y + rad + kHalf) / kBoxCell));
  for (int y = y0; y <= y1; y++)
    for (int x = x0; x <= x1; x++) boxGrid_[size_t(y) * kBoxGrid + x].push_back(id);
}

void World::rasteriseRoads(std::vector<float>& dist, std::vector<float>& roadH, std::vector<u8>& onTrail) const {
  const auto wx = [](int i) { return double(i) - kHalf; };
  for (const Road& r : roads_) {
    const size_t cnt = r.pts.size();
    const size_t segs = r.closed ? cnt : cnt - 1;
    for (size_t s = 0; s < segs; s++) {
      const RoadPoint& a = r.pts[s];
      const RoadPoint& b = r.pts[(s + 1) % cnt];
      const double margin = r.halfWidth + 16.0;
      const int x0 = std::max(0, int(std::min(a.x, b.x) - margin + kHalf));
      const int x1 = std::min(N - 1, int(std::max(a.x, b.x) + margin + kHalf));
      const int y0 = std::max(0, int(std::min(a.y, b.y) - margin + kHalf));
      const int y1 = std::min(N - 1, int(std::max(a.y, b.y) + margin + kHalf));
      const double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
      for (int iy = y0; iy <= y1; iy++)
        for (int ix = x0; ix <= x1; ix++) {
          const double px = wx(ix) - a.x, py = wx(iy) - a.y;
          const double t = clampd((px * dx + py * dy) / l2, 0, 1);
          const double d = std::hypot(px - t * dx, py - t * dy) - r.halfWidth;
          const size_t id = size_t(iy) * N + ix;
          if (d < dist[id]) {
            dist[id] = float(d);
            // Signed distance to the left of the road's centre line, levelling off beyond the shoulder.
            const double tl = std::sqrt(l2);
            const double side = clampd((-dy * px + dx * py) / tl, -(r.halfWidth + 2.0), r.halfWidth + 2.0);
            roadH[id] = float(lerpd(a.z, b.z, t) + side * lerpd(a.bank, b.bank, t));
            onTrail[id] = r.trail ? 1 : 0;
          }
        }
    }
  }
}

void World::addVisualBox(V3 c, V3 half, double yaw, float r, float g, float b) {
  StaticBox s;
  s.c = c;
  s.h = half;
  s.surface = SURF_GRAVEL;
  s.r = r;
  s.g = g;
  s.b = b;
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  s.ax[0] = {cy, sy, 0};
  s.ax[1] = {-sy, cy, 0};
  s.ax[2] = {0, 0, 1};
  boxes_.push_back(s);
}

void World::build() {
  h_.assign(size_t(N) * N, 0.f);
  surf_.assign(size_t(N) * N, SURF_GRASS);
  boxGrid_.assign(size_t(kBoxGrid) * kBoxGrid, {});
  boxes_.reserve(62000);  // never moves: the renderer reads it while blasts add to it

  // Base terrain, flattened towards the test pad.
  std::vector<float> dist(size_t(N) * N, 1e9f), roadH(size_t(N) * N, 0.f);
  auto wx = [](int i) { return double(i) - kHalf; };
  auto baseAt = [](double x, double y) { return baseTerrain(x, y) * padBlend(x, y) + mountain(x, y); };

  // A wavy closed country road around the pad.
  Road loop;
  loop.closed = true;
  loop.halfWidth = 4.0;
  constexpr int kLoop = 1600;
  std::vector<double> raw(kLoop);
  std::vector<std::pair<double, double>> xy(kLoop);
  for (int i = 0; i < kLoop; i++) {
    const double th = 2.0 * PI * i / kLoop;
    const double r = 345.0 + 55.0 * std::sin(3.0 * th + 0.7) + 28.0 * std::sin(5.0 * th + 2.1) +
                     12.0 * std::sin(11.0 * th);
    xy[i] = {r * std::cos(th), r * std::sin(th)};
    raw[i] = baseAt(xy[i].first, xy[i].second);
  }
  // Roads are graded: smooth the profile along the road so it does not follow every hill.
  for (int i = 0; i < kLoop; i++) {
    double sum = 0;
    constexpr int kWin = 28;
    for (int j = -kWin; j <= kWin; j++) sum += raw[(i + j + kLoop) % kLoop];
    loop.pts.push_back({xy[i].first, xy[i].second, sum / (2 * kWin + 1)});
  }
  // Bends lean into the turn, like a well-built country road. The slope makes up for part of the
  // cornering force at the design speed, and is limited to a gentle 7 degrees.
  {
    constexpr double kDesignSpeed = 34.0, kMaxSlope = 0.123;
    std::vector<double> want(kLoop);
    for (int i = 0; i < kLoop; i++) {
      constexpr int kSpan = 8;  // about 11 m either way, so the wiggles of the line do not count
      const RoadPoint& a = loop.pts[(i - kSpan + kLoop) % kLoop];
      const RoadPoint& b = loop.pts[i];
      const RoadPoint& c = loop.pts[(i + kSpan) % kLoop];
      const double ux = b.x - a.x, uy = b.y - a.y, vx = c.x - b.x, vy = c.y - b.y;
      const double turn = std::atan2(ux * vy - uy * vx, ux * vx + uy * vy);
      const double curvature = turn / std::max(0.5 * (std::hypot(ux, uy) + std::hypot(vx, vy)), 0.1);
      // Turning left means the left side is the inside: it is the lower one, so the slope is negative.
      want[i] = -clampd(curvature * kDesignSpeed * kDesignSpeed / 9.81 * 0.5, -kMaxSlope, kMaxSlope);
    }
    for (int i = 0; i < kLoop; i++) {
      // Spread out, so that the road twists gradually instead of kinking at the start of a bend.
      constexpr int kWin = 26;
      double sum = 0;
      for (int j = -kWin; j <= kWin; j++) sum += want[(i + j + kLoop) % kLoop];
      loop.pts[i].bank = gBankScale * sum / (2 * kWin + 1);
    }
  }
  roads_.push_back(loop);

  // Two straight connectors from the pad out to the loop.
  auto connector = [&](double dirY) {
    double best = 1e9;
    int bi = 0;
    for (int i = 0; i < kLoop; i++) {
      const double d = std::hypot(loop.pts[i].x, loop.pts[i].y - dirY * 345.0);
      if (d < best) {
        best = d;
        bi = i;
      }
    }
    const RoadPoint end = loop.pts[bi];
    Road r;
    r.halfWidth = 3.5;
    constexpr int kN = 120;
    for (int i = 0; i <= kN; i++) {
      const double t = double(i) / kN;
      const double x = end.x * t;
      const double y = lerpd(dirY * 120.0, end.y, t);
      r.pts.push_back({x, y, end.z * smooth01(t * 1.3 - 0.3)});
    }
    roads_.push_back(r);
  };
  connector(1.0);
  connector(-1.0);

  // ---- The tracks up the mountain. Each is walked up the slope step by step: traversing, climbing at a set
  // grade, and swinging round in a hairpin at the end of each traverse.
  {
    const Road& lp = roads_[0];
    const auto climb = [&](double a0, double range, double grade, double turnR, std::vector<RoadPoint>& pts) {
      // phi is the direction of travel against the contour: 0 along it one way, pi the other way, and
      // pi/2 straight up the hill. A hairpin is phi swinging from one to the other at a fixed radius.
      double r = kPeakR * 0.96, a = a0, phi = 0.0, phiTarget = 0.0;
      constexpr double kStep = 1.4;
      while (r > 16.0) {
        phi += clampd(phiTarget - phi, -kStep / turnR, kStep / turnR);
        const double along = std::cos(phi), up = std::sin(phi);
        a += along * kStep * std::sqrt(1.0 - grade * grade) / std::max(r, 12.0);
        r -= kStep * std::max(up, grade * (1.0 - std::fabs(up)));
        const double limit = range * clampd(r / (kPeakR * 0.5), 0.25, 1.0);
        if (a > a0 + limit) phiTarget = PI;
        if (a < a0 - limit) phiTarget = 0.0;
        const double x = kPeakX + std::cos(a) * r, y = kPeakY + std::sin(a) * r;
        pts.push_back({x, y, baseAt(x, y)});
      }
    };
    const auto finish = [&](std::vector<RoadPoint>& raw, double halfWidth, double maxGrade) {
      // Graded: the track does not follow every lump of the hillside, and is never steeper than it was
      // built for. Where the hill differs, the bank is cut away or filled up.
      Road t;
      t.trail = true;
      t.halfWidth = halfWidth;
      const int n = int(raw.size());
      std::vector<double> z(static_cast<size_t>(n));
      for (int i = 0; i < n; i++) {
        double sum = 0;
        for (int j = -22; j <= 22; j++) sum += raw[size_t(std::clamp(i + j, 0, n - 1))].z;
        z[size_t(i)] = sum / 45.0;
      }
      const auto dist = [&](int i) { return std::hypot(raw[size_t(i)].x - raw[size_t(i - 1)].x, raw[size_t(i)].y - raw[size_t(i - 1)].y); };
      for (int pass = 0; pass < 3; pass++) {
        for (int i = 1; i < n; i++) z[size_t(i)] = clampd(z[size_t(i)], z[size_t(i - 1)] - maxGrade * dist(i), z[size_t(i - 1)] + maxGrade * dist(i));
        for (int i = n - 2; i >= 0; i--) z[size_t(i)] = clampd(z[size_t(i)], z[size_t(i + 1)] - maxGrade * dist(i + 1), z[size_t(i + 1)] + maxGrade * dist(i + 1));
      }
      for (int i = 0; i < n; i++) t.pts.push_back({raw[size_t(i)].x, raw[size_t(i)].y, z[size_t(i)]});
      roads_.push_back(t);
    };
    const auto connect = [&](const RoadPoint& from, double a, std::vector<RoadPoint>& pts) {
      const double tx = kPeakX + std::cos(a) * kPeakR * 0.96, ty = kPeakY + std::sin(a) * kPeakR * 0.96;
      const int n = int(std::hypot(tx - from.x, ty - from.y) / 1.4);
      for (int i = 0; i < n; i++) {
        const double t = double(i) / n;
        const double x = lerpd(from.x, tx, t) + 16.0 * std::sin(t * 9.0) * std::sin(t * PI);
        const double y = lerpd(from.y, ty, t) - 16.0 * std::sin(t * 9.0) * std::sin(t * PI);
        pts.push_back({x, y, baseAt(x, y)});
      }
    };
    const size_t cnt = lp.pts.size();
    const RoadPoint& gate = lp.pts[cnt / 8];  // the point of the loop that faces the mountain
    const double aFace = std::atan2(gate.y - kPeakY, gate.x - kPeakX);
    // The switchbacks: long traverses at a steady 14 %.
    std::vector<RoadPoint> easy;
    connect(gate, aFace, easy);
    const size_t foot = easy.size();
    climb(aFace, 1.05, 0.14, 12.0, easy);
    // The hard way: short and steep, about 35 %, straight up the other flank.
    std::vector<RoadPoint> hard;
    const double aHard = aFace - 1.55;
    {
      // Starts from the same foot, round the bottom of the mountain.
      for (double a = aFace; a > aHard; a -= 1.4 / (kPeakR * 0.97)) {
        const double x = kPeakX + std::cos(a) * kPeakR * 0.97, y = kPeakY + std::sin(a) * kPeakR * 0.97;
        hard.push_back({x, y, baseAt(x, y)});
      }
      climb(aHard, 0.34, 0.33, 9.0, hard);
    }
    finish(easy, 2.9, 0.20);
    finish(hard, 2.3, 0.48);
    const Road& e = roads_[roads_.size() - 2];
    const auto heading = [&](size_t i) { return std::atan2(e.pts[i + 3].y - e.pts[i].y, e.pts[i + 3].x - e.pts[i].x); };
    places_.push_back({"Testgelände", -90, 0, 0});
    places_.push_back({"Bergfuß", e.pts[foot - 40].x, e.pts[foot - 40].y, heading(foot - 40)});
    const size_t top = e.pts.size() - 8;
    places_.push_back({"Gipfel", e.pts[top].x, e.pts[top].y, heading(top - 3)});
  }

  // Rasterise roads into the distance/height fields.
  std::vector<u8> onTrail(size_t(N) * N, 0);
  rasteriseRoads(dist, roadH, onTrail);

  for (int iy = 0; iy < N; iy++)
    for (int ix = 0; ix < N; ix++) {
      const double x = wx(ix), y = wx(iy);
      const size_t id = size_t(iy) * N + ix;
      const double d = dist[id];
      const double blend = 1.0 - smoothstepd(0.0, 13.0, d);
      h_[id] = float(lerpd(baseAt(x, y), double(roadH[id]), blend));
      const bool trail = onTrail[id] != 0;
      const double pr = std::hypot(x, y);
      if (trail && d < 1.6) {
        // Rutted and lumpy: this is what the long travel is for.
        surf_[id] = SURF_GRAVEL;
        h_[id] += float(0.09 * (vnoise(x / 2.1, y / 2.1) - 0.5) + 0.05 * (vnoise(x / 0.9 + 4.0, y / 0.9) - 0.5));
      } else if (!trail && d < 0.3) {
        surf_[id] = SURF_ASPHALT;
      } else if (d < 2.0) {
        surf_[id] = SURF_GRAVEL;
      } else if (pr < kPadR) {
        surf_[id] = SURF_CONCRETE;
      } else {
        surf_[id] = SURF_GRASS;
        // Above the tree line the mountain turns to bare rock, and to snow at the top.
        const double z = h_[id];
        const double line = vnoise(x / 31.0, y / 31.0 + 8.0) * 22.0;
        if (z > 228.0 + line) surf_[id] = SURF_SNOW;
        else if (z > 150.0 + line) surf_[id] = SURF_ROCK;
      }
      if (x > kPaveX0 && x < kPaveX1 && y > kPaveY0 && y < kPaveY1) surf_[id] = SURF_PAVE;
      if (x > kWashX0 && x < kWashX1 && y > kWashY0 && y < kWashY1) surf_[id] = SURF_GRAVEL;
      if (x > -130 && x < -60 && y > -110 && y < -80) surf_[id] = SURF_GRASS;  // low-grip patch
    }

  // Trees along the country road. Something has to rush past, or the speed is only a number on the
  // dial. They are built as soft bodies in populateWorld: this only decides where they stand.
  {
    u32 seed = 90210;
    auto rnd = [&](double a, double b) {
      seed = seed * 1664525u + 1013904223u;
      return a + (b - a) * double(seed >> 16) / 65535.0;
    };
    const Road& lp = roads_[0];
    const size_t cnt = lp.pts.size();
    for (int n = 0; n < 300; n++) {
      const size_t i = size_t(rnd(0, double(cnt - 1)));
      const RoadPoint& a = lp.pts[i];
      const RoadPoint& b = lp.pts[(i + 1) % cnt];
      const double tx = b.x - a.x, ty = b.y - a.y, tl = std::hypot(tx, ty);
      const double side = rnd(0, 1) < 0.5 ? -1.0 : 1.0;
      const double off = side * (lp.halfWidth + 5.0 + std::pow(rnd(0, 1), 1.6) * 34.0);
      const double x = a.x - ty / tl * off, y = a.y + tx / tl * off;
      bool clear = std::hypot(x, y) > kPadR + 30.0;
      for (const Road& r : roads_)
        for (size_t k = 0; clear && k < r.pts.size(); k += 2)
          if (std::hypot(r.pts[k].x - x, r.pts[k].y - y) < r.halfWidth + 5.0) clear = false;
      if (!clear) continue;
      const double zb = height(x, y);
      trees_.push_back({x, y, zb});
    }
  }

  // The mountain's woods and boulders, kept clear of the tracks.
  {
    u32 seed = 424242;
    auto rnd = [&](double a, double b) {
      seed = seed * 1664525u + 1013904223u;
      return a + (b - a) * double(seed >> 16) / 65535.0;
    };
    const auto clearOfTracks = [&](double x, double y, double margin) {
      for (const Road& r : roads_)
        for (size_t k = 0; k < r.pts.size(); k += 3)
          if (std::hypot(r.pts[k].x - x, r.pts[k].y - y) < r.halfWidth + margin) return false;
      return true;
    };
    for (int n = 0; n < 340; n++) {
      const double a = rnd(0, 6.283), rr = kPeakR * std::sqrt(rnd(0.12, 1.25));
      const double x = kPeakX + std::cos(a) * rr, y = kPeakY + std::sin(a) * rr;
      if (std::fabs(x) > kHalf - 8 || std::fabs(y) > kHalf - 8) continue;
      const double zb = height(x, y);
      if (zb > 135.0 || !clearOfTracks(x, y, 5.0)) continue;
      trees_.push_back({x, y, zb});
    }
    for (int n = 0; n < 300; n++) {
      const double a = rnd(0, 6.283), rr = kPeakR * std::sqrt(rnd(0.0, 0.92));
      const double x = kPeakX + std::cos(a) * rr, y = kPeakY + std::sin(a) * rr;
      const double size = 0.4 + std::pow(rnd(0, 1), 2.5) * 3.2;
      if (!clearOfTracks(x, y, 2.0 + size)) continue;
      const float g = float(rnd(0.28, 0.46));
      addBox({x, y, height(x, y) + size * 0.25}, {size * rnd(0.7, 1.3), size * rnd(0.7, 1.2), size * rnd(0.5, 0.9)}, rnd(0, 3.1), rnd(-0.3, 0.3), SURF_ROCK, g, g * 0.97f, g * 0.92f);
    }
  }

  // Static features on the pad. Forward lanes run along +X.
  const float wr = .62f, wg = .60f, wb = .56f;
  addBox({110, -30, 1.25}, {0.6, 5.0, 1.25}, 0, 0, SURF_CONCRETE, wr, wg, wb);    // crash wall
  addBox({150, 0, 1.5}, {0.8, 14.0, 1.5}, 0, 0, SURF_CONCRETE, wr, wg, wb);       // end of gauntlet
  addBox({60, 140, 0.6}, {30.0, 0.5, 0.6}, 0, 0, SURF_CONCRETE, wr, wg, wb);      // scrape wall

  // Jump: ramp up, gap, landing.
  const double rampPitch = -0.16;
  addBox({25, 40, 0.0}, {11.0, 4.0, 0.9}, 0, rampPitch, SURF_CONCRETE, .7f, .55f, .3f);
  addBox({61, 40, -0.35}, {12.0, 4.5, 0.9}, 0, -rampPitch, SURF_CONCRETE, .7f, .55f, .3f);

  // Kerbs and speed bumps.
  for (int i = 0; i < 6; i++)
    addBox({-40.0 + i * 9.0, 70, 0.03}, {0.25, 4.5, 0.09}, 0, 0, SURF_CONCRETE, .8f, .75f, .2f);
  for (int i = 0; i < 14; i++)
    addBox({-40.0 + i * 3.0, 95, 0.0}, {1.45, 0.25, 0.11}, 0, 0, SURF_CONCRETE, .75f, .75f, .75f);
}

V3 World::Base::slot(int i) const {
  // Across the yard in a row, the guns towards the enemy, wide enough apart to drive out between them.
  const double along = (double(i) - double(kBaseSlots - 1) * 0.5) * 13.0;
  const double cx = std::cos(facing), sx = std::sin(facing);
  return centre + V3{cx * 14.0 - sx * along, sx * 14.0 + cx * along, 0};
}

void World::addCommandCentre(Base& b) {
  // Metre-thick concrete, with one way in: the door on the side towards the yard. What is drawn is what stops you.
  const double cx = std::cos(b.facing), sx = std::sin(b.facing);
  const V3 at = V3{b.centre.x, b.centre.y, 0} - V3{cx, sx, 0} * 26.0;
  const auto place = [&](double fwd, double side, double z, double hf, double hs, double hz, float r, float g, float bl) {
    addBox(at + V3{cx * fwd - sx * side, sx * fwd + cx * side, z}, {hf, hs, hz}, b.facing, 0, SURF_CONCRETE, r, g, bl);
  };
  constexpr double kF = 9.0, kS = 13.0, kH = 3.2, kWall = 0.6, kDoor = 1.6;
  constexpr float wr = .46f, wg = .47f, wb = .45f;
  place(0, 0, kYardZ + 2 * kH + kWall, kF + kWall, kS + kWall, kWall, wr * 0.9f, wg * 0.9f, wb * 0.9f);  // the roof
  place(-kF, 0, kYardZ + kH, kWall, kS, kH, wr, wg, wb);                                                // the back
  place(0, kS, kYardZ + kH, kF, kWall, kH, wr, wg, wb);                                                 // the sides
  place(0, -kS, kYardZ + kH, kF, kWall, kH, wr, wg, wb);
  const double piece = (kS - kDoor) * 0.5;
  place(kF, kDoor + piece, kYardZ + kH, kWall, piece, kH, wr, wg, wb);                                  // the front, either side of the door
  place(kF, -kDoor - piece, kYardZ + kH, kWall, piece, kH, wr, wg, wb);
  place(kF, 0, kYardZ + kH + 2.4, kWall, kDoor, kH - 2.4, wr, wg, wb);                                  // above the door
  // The main computer against the back wall, and the blast walls that keep a shell from flying straight in.
  place(-kF + 1.4, 0, kYardZ + 1.0, 0.7, 2.6, 1.0, .18f, .2f, .24f);
  place(kF + 5.0, 0, kYardZ + 1.4, 0.5, 4.0, 1.4, wr, wg, wb);
  b.door = at + V3{cx, sx, 0} * (kF + 2.0) + V3{0, 0, kYardZ};
  b.computer = at - V3{cx, sx, 0} * (kF - 2.6) + V3{0, 0, kYardZ + 1.0};
}

V3 World::fordPoint(int i) { return V3{riverX(kFords[i]), kFords[i], 0}; }

bool World::isWater(double x, double y) const {
  return arena_ && std::fabs(x - riverX(y)) < kRiverHalf + 3.0 && std::fabs(y) < 830.0 && height(x, y) < kWaterZ;
}

void World::buildArena() {
  arena_ = true;
  h_.assign(size_t(N) * N, 0.f);
  surf_.assign(size_t(N) * N, SURF_GRASS);
  boxGrid_.assign(size_t(kBoxGrid) * kBoxGrid, {});
  boxes_.reserve(62000);
  std::vector<float> dist(size_t(N) * N, 1e9f), roadH(size_t(N) * N, 0.f);
  std::vector<u8> onTrail(size_t(N) * N, 0);
  const auto wx = [](int i) { return double(i) - kHalf; };

  bases_[0].centre = {-kBaseX, 0, kYardZ};
  bases_[0].facing = 0.0;
  bases_[1].centre = {kBaseX, 0, kYardZ};
  bases_[1].facing = PI;

  // The ways between the bases: one road down the valley through the middle, and a dirt track round each flank.
  const auto way = [&](std::initializer_list<std::pair<double, double>> via, double halfWidth, bool trail) {
    const std::vector<std::pair<double, double>> v(via);
    std::vector<RoadPoint> pts;
    // A smooth line through the points (Catmull-Rom), about a metre and a half to the step.
    for (size_t i = 0; i + 1 < v.size(); i++) {
      const auto& p0 = v[i > 0 ? i - 1 : i];
      const auto& p1 = v[i];
      const auto& p2 = v[i + 1];
      const auto& p3 = v[i + 2 < v.size() ? i + 2 : i + 1];
      const int steps = std::max(2, int(std::hypot(p2.first - p1.first, p2.second - p1.second) / 1.5));
      for (int k = 0; k < steps; k++) {
        const double t = double(k) / steps, t2 = t * t, t3 = t2 * t;
        const double x = 0.5 * (2 * p1.first + (p2.first - p0.first) * t + (2 * p0.first - 5 * p1.first + 4 * p2.first - p3.first) * t2 + (3 * p1.first - p0.first - 3 * p2.first + p3.first) * t3);
        const double y = 0.5 * (2 * p1.second + (p2.second - p0.second) * t + (2 * p0.second - 5 * p1.second + 4 * p2.second - p3.second) * t2 + (3 * p1.second - p0.second - 3 * p2.second + p3.second) * t3);
        pts.push_back({x, y, arenaTerrain(x, y)});
      }
    }
    // Graded: smoothed along the way and never steeper than a tank likes.
    Road r;
    r.halfWidth = halfWidth;
    r.trail = trail;
    const int n = int(pts.size());
    std::vector<double> z(static_cast<size_t>(n));
    for (int i = 0; i < n; i++) {
      double sum = 0;
      for (int j = -30; j <= 30; j++) sum += pts[size_t(std::clamp(i + j, 0, n - 1))].z;
      z[size_t(i)] = sum / 61.0;
    }
    constexpr double kGrade = 0.18 * 1.5;
    for (int pass = 0; pass < 3; pass++) {
      for (int i = 1; i < n; i++) z[size_t(i)] = clampd(z[size_t(i)], z[size_t(i - 1)] - kGrade, z[size_t(i - 1)] + kGrade);
      for (int i = n - 2; i >= 0; i--) z[size_t(i)] = clampd(z[size_t(i)], z[size_t(i + 1)] - kGrade, z[size_t(i + 1)] + kGrade);
    }
    // Into a yard it comes at the yard's own height.
    for (int i = 0; i < n; i++) z[size_t(i)] = lerpd(z[size_t(i)], kYardZ, yardBlend(pts[size_t(i)].x, pts[size_t(i)].y));
    // Over a river the way runs dead level, from bank to bank and a little beyond, well clear of the water.
    for (int i = 1; i < n; i++) {
      const double a = pts[size_t(i - 1)].x - riverX(pts[size_t(i - 1)].y), b = pts[size_t(i)].x - riverX(pts[size_t(i)].y);
      if ((a < 0) == (b < 0)) continue;
      constexpr int kFlat = 32, kRamp = 40;
      const double deck = std::max(z[size_t(i)], kWaterZ + 5.0);
      for (int j = -kFlat - kRamp; j <= kFlat + kRamp; j++) {
        if (i + j < 0 || i + j >= n) continue;
        const double t = clampd((std::abs(j) - double(kFlat)) / kRamp, 0, 1);
        z[size_t(i + j)] = lerpd(deck, z[size_t(i + j)], smooth01(t));
      }
    }
    for (int i = 0; i < n; i++) r.pts.push_back({pts[size_t(i)].x, pts[size_t(i)].y, z[size_t(i)]});
    roads_.push_back(r);
  };
  const double gate = kBaseX - kYardHalfX;
  way({{-gate, 0}, {-520, 30}, {-330, -40}, {-140, 60}, {60, 90}, {250, 20}, {430, -30}, {600, 15}, {gate, 0}}, 4.0, false);
  way({{-gate, 30}, {-610, 240}, {-420, 450}, {-210, 610}, {30, 660}, {270, 590}, {480, 430}, {620, 220}, {gate, 30}}, 3.0, true);
  way({{-gate, -30}, {-610, -240}, {-470, -520}, {-180, -620}, {60, -560}, {290, -610}, {520, -450}, {620, -220}, {gate, -30}}, 3.0, true);
  way({{-140, 60}, {-110, 240}, {-210, 610}}, 3.0, true);
  way({{250, 20}, {280, -220}, {290, -610}}, 3.0, true);
  rasteriseRoads(dist, roadH, onTrail);

  for (int iy = 0; iy < N; iy++)
    for (int ix = 0; ix < N; ix++) {
      const double x = wx(ix), y = wx(iy);
      const size_t id = size_t(iy) * N + ix;
      const double d = dist[id];
      const double blend = 1.0 - smoothstepd(0.0, 13.0, d);
      const double yard = yardBlend(x, y);
      h_[id] = float(yard > 0.98 ? kYardZ : lerpd(arenaTerrain(x, y), double(roadH[id]), blend));
      const bool flatYard = (std::fabs(std::fabs(x) - kBaseX) < kYardHalfX && std::fabs(y) < kYardHalfY);
      if (flatYard) surf_[id] = SURF_CONCRETE;
      else if (onTrail[id] && d < 1.6) surf_[id] = SURF_GRAVEL;
      else if (!onTrail[id] && d < 0.3) surf_[id] = SURF_ASPHALT;
      else if (d < 2.0) surf_[id] = SURF_GRAVEL;
      else surf_[id] = h_[id] > 95.0 + vnoise(x / 31.0, y / 31.0) * 22.0 ? SURF_ROCK : SURF_GRASS;
      if (h_[id] > 215.0 + vnoise(x / 31.0, y / 31.0 + 8.0) * 20.0) surf_[id] = SURF_SNOW;
    }

  // What is drawn as road lies exactly on the ground the wheels roll on, wherever that was changed around it.
  for (Road& r : roads_)
    for (RoadPoint& p : r.pts) p.z = height(p.x, p.y);
  // The river is cut last, through whatever land and roads lie in its way. Where a road crosses it stands a bridge.
  for (int iy = 0; iy < N; iy++)
    for (int ix = 0; ix < N; ix++) {
      const double x = wx(ix), y = wx(iy);
      const double cut = riverCut(x, y);
      if (cut <= 0) continue;
      const size_t id = size_t(iy) * N + ix;
      h_[id] = float(lerpd(double(h_[id]), riverBedAt(y), cut));
      if (cut > 0.5) surf_[id] = SURF_GRAVEL;
    }
  addCommandCentre(bases_[0]);
  addCommandCentre(bases_[1]);
  // The water: flat, drawn, and nothing that holds anything up.
  for (double y = -830.0; y < 830.0; y += 8.0) {
    const double x0 = riverX(y), x1 = riverX(y + 8.0);
    const double yaw = std::atan2(8.0, x1 - x0) - PI * 0.5;
    addVisualBox({(x0 + x1) * 0.5, y + 4.0, kWaterZ - 0.5}, {kRiverHalf + 5.0, 4.6, 0.5}, yaw, .13f, .29f, .36f);
  }
  for (const Road& r : roads_) {
    for (size_t i = 1; i < r.pts.size(); i++) {
      const double a = r.pts[i - 1].x - riverX(r.pts[i - 1].y), b = r.pts[i].x - riverX(r.pts[i].y);
      if ((a < 0) == (b < 0)) continue;
      const RoadPoint& p = r.pts[i];
      const double yaw = std::atan2(p.y - r.pts[i - 1].y, p.x - r.pts[i - 1].x);
      const double cy = std::cos(yaw), sy = std::sin(yaw);
      const double half = kBankOuter + 9.0, w = r.halfWidth + 2.0;
      const double z = p.z;
      addBox({p.x, p.y, z - 0.6}, {half, w, 0.6}, yaw, 0, SURF_CONCRETE, .55f, .53f, .5f);
      for (double sg : {-1.0, 1.0})
        addBox({p.x - sy * sg * w, p.y + cy * sg * w, z + 0.55}, {half, 0.25, 0.55}, yaw, 0, SURF_CONCRETE, .48f, .46f, .43f);
      for (double along : {-kRiverHalf, 0.0, kRiverHalf})
        addBox({p.x + cy * along, p.y + sy * along, (z + kBedZ) * 0.5 - 0.6}, {1.2, w - 0.5, (z - kBedZ) * 0.5}, yaw, 0, SURF_CONCRETE, .44f, .42f, .4f);
      bridges_.push_back({p.x, p.y, z});
    }
  }

  u32 seed = 271828;
  const auto rnd = [&](double a, double b) {
    seed = seed * 1664525u + 1013904223u;
    return a + (b - a) * double(seed >> 16) / 65535.0;
  };
  const auto nearRoad = [&](double x, double y, double margin) {
    for (const Road& r : roads_)
      for (size_t k = 0; k < r.pts.size(); k += 3)
        if (std::hypot(r.pts[k].x - x, r.pts[k].y - y) < r.halfWidth + margin) return true;
    return false;
  };
  const auto nearBase = [&](double x, double y, double margin) {
    return yardBlend(x, y) > 0.0 || std::hypot(std::fabs(x) - kBaseX, y) < margin;
  };
  const auto inTheWay = [&](double x, double y, double margin) {
    return std::fabs(x - riverX(y)) < lerpd(kBankOuter, 70.0, fordAt(y)) + margin || std::fabs(std::fabs(x) - kBarrierX) < margin;
  };

  // The barriers, section by section, following the lie of the land, with a gate wherever a road goes through.
  for (double side : {-1.0, 1.0})
    for (double y = -812.0; y < 812.0; y += kBarrierSection) {
      const double x = side * kBarrierX, yc = y + kBarrierSection * 0.5;
      if (nearRoad(x, yc, 14.0)) continue;
      const double z = std::max({height(x, y), height(x, yc), height(x, y + kBarrierSection)});
      addBox({x, yc, z + kBarrierHigh * 0.5 - 2.0}, {1.2, kBarrierSection * 0.5, kBarrierHigh * 0.5 + 2.0}, 0, 0, SURF_CONCRETE, .5f, .5f, .47f);
    }

  // Four villages on the ways, and the ground around their houses kept clear. The houses themselves come later.
  const double villages[5][2] = {{-335, -30}, {62, 92}, {430, -30}, {-420, 450}, {290, -610}};
  std::vector<std::pair<double, double>> sites;
  for (const auto& v : villages) {
    for (int n = 0; n < 7; n++) {
      for (int tries = 0; tries < 30; tries++) {
        const double a = rnd(0, 6.283), r = rnd(18.0, 80.0);
        const double x = v[0] + std::cos(a) * r, y = v[1] + std::sin(a) * r;
        if (nearRoad(x, y, 14.0) || inTheWay(x, y, 16.0)) continue;
        bool free = true;
        for (const auto& s2 : sites)
          if (std::hypot(s2.first - x, s2.second - y) < 24.0) free = false;
        if (!free) continue;
        sites.push_back({x, y});
        // Its door towards the middle of the village, and the ground under it levelled.
        const double yaw = std::atan2(y - v[1], x - v[0]) + PI * 0.5;
        const World::House house{x, y, yaw};
        const V3 mid = houseInside(house, kHouseLength * 0.5, kHouseDepth * 0.5, 0);
        const double z = height(mid.x, mid.y);
        for (int iy = std::max(0, int(mid.y + kHalf - 14)); iy <= std::min(N - 1, int(mid.y + kHalf + 14)); iy++)
          for (int ix = std::max(0, int(mid.x + kHalf - 14)); ix <= std::min(N - 1, int(mid.x + kHalf + 14)); ix++) {
            const double px = wx(ix) - mid.x, py = wx(iy) - mid.y;
            const double cy = std::cos(yaw), sy = std::sin(yaw);
            const double lu = std::fabs(px * cy + py * sy), lv = std::fabs(-px * sy + py * cy);
            const double dd = std::hypot(std::max(lu - kHouseLength * 0.5 - 1.0, 0.0), std::max(lv - kHouseDepth * 0.5 - 1.0, 0.0));
            const size_t id = size_t(iy) * N + ix;
            const double blend = 1.0 - smoothstepd(0.0, 5.0, dd);
            if (blend > 0) h_[id] = float(lerpd(double(h_[id]), z, blend));
          }
        houses_.push_back(house);
        houseSites_.push_back({mid.x, mid.y, z});
        break;
      }
    }
  }
  const auto nearSite = [&](double x, double y, double margin) {
    for (const auto& s2 : sites)
      if (std::hypot(s2.first - x, s2.second - y) < margin) return true;
    return false;
  };

  // Old tank wrecks from some earlier battle, out in the open and on the hills.
  const double wrecks[7][2] = {{-560, 130}, {-250, -210}, {-30, 290}, {130, -180}, {330, 200}, {560, -130}, {-100, -420}};
  for (const auto& w : wrecks) wreckSites_.push_back({w[0], w[1], height(w[0], w[1])});

  // Woods in clumps, the only cover there is besides the hills. Each tree is a soft body, so there are not too many.
  constexpr int kClumps = 95;
  for (int c = 0; c < kClumps; c++) {
    const double cx = rnd(-790, 790), cy = rnd(-790, 790), radius = rnd(22, 85);
    const int count = int(radius * 0.42);
    for (int n = 0; n < count; n++) {
      const double a = rnd(0, 6.283), r = radius * std::sqrt(rnd(0, 1));
      const double x = cx + std::cos(a) * r, y = cy + std::sin(a) * r;
      if (std::max(std::fabs(x), std::fabs(y)) > 830.0 || nearRoad(x, y, 5.0) || nearBase(x, y, 70.0) || nearSite(x, y, 14.0) || inTheWay(x, y, 5.0)) continue;
      trees_.push_back({x, y, height(x, y)});
    }
  }

  // Hedgerows: lines of trees across the open ground, so that nobody can see from one end of a field to the other.
  constexpr int kRows = 46;
  for (int c = 0; c < kRows; c++) {
    const double cx = rnd(-760, 760), cy = rnd(-760, 760), a = rnd(0, 3.1416), length = rnd(50, 170);
    const int count = int(length / 6.5);
    for (int n = 0; n < count; n++) {
      const double t = (double(n) / count - 0.5) * length;
      const double x = cx + std::cos(a) * t + rnd(-1.5, 1.5), y = cy + std::sin(a) * t + rnd(-1.5, 1.5);
      if (std::max(std::fabs(x), std::fabs(y)) > 830.0 || nearRoad(x, y, 5.0) || nearBase(x, y, 70.0) || nearSite(x, y, 14.0) || inTheWay(x, y, 5.0)) continue;
      trees_.push_back({x, y, height(x, y)});
    }
  }

  // Outcrops of big rock: cover that stays, until something hits it hard.
  constexpr int kOutcrops = 42;
  for (int c = 0; c < kOutcrops; c++) {
    const double cx = rnd(-720, 720), cy = rnd(-720, 720);
    if (nearRoad(cx, cy, 10.0) || nearBase(cx, cy, 90.0) || nearSite(cx, cy, 18.0) || inTheWay(cx, cy, 10.0)) continue;
    const float g = float(rnd(0.3, 0.44));
    const int pieces = 3 + int(rnd(0, 4));
    for (int n = 0; n < pieces; n++) {
      const double x = cx + rnd(-5, 5), y = cy + rnd(-5, 5), size = rnd(1.6, 3.6);
      addBox({x, y, height(x, y) + size * 0.45}, {size * rnd(0.8, 1.3), size * rnd(0.7, 1.2), size * rnd(0.8, 1.2)}, rnd(0, 3.1), rnd(-0.2, 0.2), SURF_ROCK, g, g * 0.97f, g * 0.92f);
    }
  }

  // Boulders, thicker towards the rim.
  for (int n = 0; n < 420; n++) {
    const double x = rnd(-960, 960), y = rnd(-960, 960);
    const double edge = std::max(std::fabs(x), std::fabs(y));
    if (edge < 500.0 && rnd(0, 1) < 0.7) continue;
    const double size = 0.4 + std::pow(rnd(0, 1), 2.5) * 3.4;
    if (nearRoad(x, y, 2.0 + size) || nearBase(x, y, 70.0) || nearSite(x, y, 10.0) || inTheWay(x, y, 3.0 + size)) continue;
    const float g = float(rnd(0.28, 0.46));
    addBox({x, y, height(x, y) + size * 0.25}, {size * rnd(0.7, 1.3), size * rnd(0.7, 1.2), size * rnd(0.5, 0.9)}, rnd(0, 3.1), rnd(-0.3, 0.3), SURF_ROCK, g, g * 0.97f, g * 0.92f);
  }
}
