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
double baseTerrain(double x, double y) {
  return 6.5 * vnoise(x / 210.0 + 3.1, y / 210.0 - 7.7) + 2.0 * vnoise(x / 80.0, y / 80.0 + 11.0) +
         0.35 * vnoise(x / 23.0 + 5.0, y / 23.0);
}
// Central test pad is exactly flat at z = 0.
constexpr double kPadR = 165.0;
double padBlend(double x, double y) { return smoothstepd(kPadR, kPadR + 70.0, std::hypot(x, y)); }

constexpr double kPaveX0 = -130, kPaveX1 = -60, kPaveY0 = 70, kPaveY1 = 82;
constexpr double kWashX0 = -130, kWashX1 = -60, kWashY0 = 90, kWashY1 = 102;
}  // namespace

double World::micro(double x, double y) const {
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

double World::height(double x, double y) const {
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
  std::fill(h_.begin(), h_.end(), 0.f);
}

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

void World::build() {
  h_.assign(size_t(N) * N, 0.f);
  surf_.assign(size_t(N) * N, SURF_GRASS);
  boxGrid_.assign(size_t(kBoxGrid) * kBoxGrid, {});

  // Base terrain, flattened towards the test pad.
  std::vector<float> dist(size_t(N) * N, 1e9f), roadH(size_t(N) * N, 0.f);
  auto wx = [](int i) { return double(i) - kHalf; };
  auto baseAt = [](double x, double y) { return baseTerrain(x, y) * padBlend(x, y); };

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

  // Rasterise roads into the distance/height fields.
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
            roadH[id] = float(lerpd(a.z, b.z, t));
          }
        }
    }
  }

  for (int iy = 0; iy < N; iy++)
    for (int ix = 0; ix < N; ix++) {
      const double x = wx(ix), y = wx(iy);
      const size_t id = size_t(iy) * N + ix;
      const double d = dist[id];
      const double blend = 1.0 - smoothstepd(0.0, 13.0, d);
      h_[id] = float(lerpd(baseAt(x, y), double(roadH[id]), blend));
      const double pr = std::hypot(x, y);
      if (d < 0.3)
        surf_[id] = SURF_ASPHALT;
      else if (d < 2.0)
        surf_[id] = SURF_GRAVEL;
      else if (pr < kPadR)
        surf_[id] = SURF_CONCRETE;
      else
        surf_[id] = SURF_GRASS;
      if (x > kPaveX0 && x < kPaveX1 && y > kPaveY0 && y < kPaveY1) surf_[id] = SURF_PAVE;
      if (x > kWashX0 && x < kWashX1 && y > kWashY0 && y < kWashY1) surf_[id] = SURF_GRAVEL;
      if (x > -130 && x < -60 && y > -110 && y < -80) surf_[id] = SURF_GRASS;  // low-grip patch
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
