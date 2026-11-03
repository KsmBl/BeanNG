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

double World::outerHeight(double x, double y) const { return levelled_ ? 0.0 : baseTerrain(x, y); }

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
      h_[id] = std::max(float(h_[id] + dz), float(h0_[id] - 3.0));
      if (d < 1.1) surf_[id] = SURF_GRAVEL;
    }
}

void World::clearCraters() {
  if (h0_.empty()) return;
  h_ = h0_;
  surf_ = surf0_;
  craters_.clear();
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

void World::build() {
  h_.assign(size_t(N) * N, 0.f);
  surf_.assign(size_t(N) * N, SURF_GRASS);
  boxGrid_.assign(size_t(kBoxGrid) * kBoxGrid, {});

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
