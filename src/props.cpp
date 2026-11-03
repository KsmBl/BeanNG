#include "props.h"
#include <array>
#include <vector>

namespace {
// Steel sign post: bends plastically, tears out of the ground if hit hard.
const Mat mPost{0.6e6, 120, 3.5e3, 4.5e3, 8e3, 0.30, 0};
const Mat mPostRoot{0.6e6, 120, 0, 0, 0, 0, 6e3};
const Mat mPlate{0.3e6, 50, 2e3, 3e3, 4e3, 0.40, 0};
const Mat mPlateBolt{0.3e6, 50, 0, 0, 0, 0, 2.5e3};
// Wood: stiff and brittle, with almost no plastic range before it snaps.
const Mat mWood{0.8e6, 160, 0, 0, 0, 0, 7e3};
const Mat mWoodRail{0.5e6, 90, 0, 0, 0, 0, 3.0e3};
const Mat mWoodRoot{0.8e6, 160, 0, 0, 0, 0, 11e3};
const Mat mNail{0.3e6, 60, 0, 0, 0, 0, 2.2e3};
// Stone block interior is effectively rigid; mortar is weak, especially in tension.
const Mat mStone{1.0e6, 220, 0, 0, 0, 0, 0};
const Mat mMortar{0.5e6, 110, 0, 0, 0, 0, 5.5e3};
const Mat mConcrete{5e6, 900, 0, 0, 0, 0, 0};
// Guardrail: ductile steel that deforms a long way and catches the car.
const Mat mRail{1.5e6, 220, 24e3, 30e3, 6e4, 0.35, 0};
const Mat mRailPost{1.0e6, 180, 12e3, 16e3, 3e4, 0.30, 0};
const Mat mRailRoot{1.0e6, 180, 0, 0, 0, 0, 30e3};
const Mat mRailBolt{0.8e6, 140, 0, 0, 0, 0, 22e3};
const Mat mCone{0.05e6, 20, 0, 0, 0, 0, 0};

struct Frame {
  V3 o;
  double c, s;
  V3 at(double x, double y, double z) const { return o + V3{c * x - s * y, s * x + c * y, z}; }
};
Frame frameAt(const World& w, double x, double y, double yaw) {
  return {{x, y, w.height(x, y)}, std::cos(yaw), std::sin(yaw)};
}

void quadTris(Sim& sim, u32 a, u32 b, u32 c, u32 d, u8 color, u8 fl = 0) {
  sim.addTri(a, b, c, fl, color, 0.5f);
  sim.addTri(a, c, d, fl, color, 0.5f);
}

// Closed visual/collision surface of a box defined by 8 corner nodes (bit order x,y,z).
void boxTris(Sim& sim, const u32 n[8], u8 color, u8 fl = 0) {
  quadTris(sim, n[0], n[2], n[3], n[1], color, fl);  // -z
  quadTris(sim, n[4], n[5], n[7], n[6], color, fl);  // +z
  quadTris(sim, n[0], n[1], n[5], n[4], color, fl);  // -y
  quadTris(sim, n[2], n[6], n[7], n[3], color, fl);  // +y
  quadTris(sim, n[0], n[4], n[6], n[2], color, fl);  // -x
  quadTris(sim, n[1], n[3], n[7], n[5], color, fl);  // +x
}

void lattice8(Sim& sim, const u32 n[8], const Mat& m) {
  for (int i = 0; i < 8; i++)
    for (int j = i + 1; j < 8; j++) sim.addBeam(n[i], n[j], m);
}

// Three-sided truss column from z0 to z1. Returns node ids as levels*3.
std::vector<u32> column(Sim& sim, const Frame& f, double x, double y, double z0, double z1,
                        double r, int levels, double nodeMass, const Mat& m, const Mat& root,
                        u8 color) {
  std::vector<u32> ids;
  for (int l = 0; l < levels; l++) {
    const double z = lerpd(z0, z1, double(l) / (levels - 1));
    for (int k = 0; k < 3; k++) {
      const double a = 2.0 * PI * k / 3.0;
      const u8 fl = l == 0 ? u8(NF_FIXED) : u8(NF_COLLIDE);
      ids.push_back(sim.addNode(f.at(x + r * std::cos(a), y + r * std::sin(a), z), nodeMass, fl, 0.5f));
    }
  }
  for (int l = 0; l < levels; l++)
    for (int k = 0; k < 3; k++) {
      const u32 a = ids[size_t(l) * 3 + k];
      const u32 b = ids[size_t(l) * 3 + (k + 1) % 3];
      if (l > 0) sim.addBeam(a, b, m);
      if (l + 1 < levels) {
        const u32 up = ids[size_t(l + 1) * 3 + k];
        const u32 up2 = ids[size_t(l + 1) * 3 + (k + 1) % 3];
        const Mat& mm = l == 0 ? root : m;
        sim.addBeam(a, up, mm);
        sim.addBeam(a, up2, mm);
        sim.addBeam(b, up, mm);
        quadTris(sim, a, b, up2, up, color);
      }
    }
  return ids;
}
}  // namespace

void addTree(Sim& sim, const World& w, const V3& foot, u32 seed) {
  auto rnd = [&](double a, double b) {
    seed = seed * 1664525u + 1013904223u;
    return a + (b - a) * double(seed >> 16) / 65535.0;
  };
  sim.beginObj(OBJ_PROP, false);
  const double scale = rnd(0.8, 1.5);
  const double H = 8.2 * scale;
  // Green wood is brittle across the grain: a trunk takes a lot, then goes all at once with a crack.
  // Thin young trees give way where a car hits them, and thick ones stop it. No two grow alike.
  const double girth = rnd(0.75, 1.25) * scale;
  const double bole = 50e3 * girth * girth;
  constexpr int kLevels = 5, kSides = 5;
  const double levelZ[kLevels] = {-0.05, 0.45, 1.25, 0.34 * H, 0.72 * H};
  const double levelR[kLevels] = {0.135, 0.115, 0.10, 0.075, 0.04};
  u32 ring[kLevels][kSides];
  double nodeMass = 12.0 * girth;
  for (int l = 0; l < kLevels; l++) {
    const double r = levelR[l] * girth;
    for (int k = 0; k < kSides; k++) {
      const double a = 2.0 * PI * k / kSides + l * 0.4;
      ring[l][k] = sim.addNode(foot + V3{r * std::cos(a), r * std::sin(a), levelZ[l]}, nodeMass, l == 0 ? u8(NF_FIXED) : u8(NF_COLLIDE), 0.5f);
    }
    nodeMass *= 0.72;
  }
  const u32 top = sim.addNode(foot + V3{0, 0, H}, 2.5, NF_COLLIDE, 0.5f);
  for (int l = 0; l < kLevels; l++) {
    // The wood weakens with height. Each level gets its own flaw.
    Mat wood{4.5e6, 400, 0, 0, 0, 0, bole * (l == 0 ? 1.6 : 1.0) * std::pow(0.42, l - 1 < 0 ? 0 : l - 1) * rnd(0.8, 1.25)};
    for (int k = 0; k < kSides; k++) {
      const u32 a = ring[l][k], b = ring[l][(k + 1) % kSides];
      if (l > 0) {
        Mat cross = wood;
        cross.k = 2.5e6;
        sim.addBeam(a, b, cross, kTagWood);
        sim.addBeam(a, ring[l][(k + 2) % kSides], cross, kTagWood);
      }
      if (l + 1 < kLevels) {
        sim.addBeam(a, ring[l + 1][k], wood, kTagWood);
        sim.addBeam(a, ring[l + 1][(k + 1) % kSides], wood, kTagWood);
        sim.addBeam(b, ring[l + 1][k], wood, kTagWood);
        quadTris(sim, a, b, ring[l + 1][(k + 1) % kSides], ring[l + 1][k], COL_BARK);
      } else {
        Mat tip{1.5e6, 150, 0, 0, 0, 0, 5e3};
        sim.addBeam(a, top, tip, kTagWood);
        sim.addTri(a, b, top, 0, COL_BARK, 0.5f);
      }
    }
  }

  // The crown is what bends: springy branches that give way plastically and snap, and needles on them.
  // Nothing in it collides with the car as a surface, only the branch tips do, which weigh little.
  const Mat branch{2.0e4, 30, 210, 380, 500, 0.45, 0};
  const Mat brace{0.6e4, 12, 85, 150, 150, 0.45, 0};
  const u8 needle = u8(COL_NEEDLES + (seed >> 20) % 3);
  auto nearestLevel = [&](double z) {
    int best = 1;
    for (int l = 1; l < kLevels; l++)
      if (std::fabs(levelZ[l] - z) < std::fabs(levelZ[best] - z)) best = l;
    return best;
  };
  constexpr int kBranches = 7;
  for (int layer = 0; layer < 2; layer++) {
    const double r0 = (layer == 0 ? 2.6 : 1.9) * scale;
    const double zb = (layer == 0 ? 2.2 : 4.6) * scale;
    const double zt = layer == 0 ? 6.4 * scale : H;
    const double zm = lerpd(zb, zt, 0.42);
    const u32 apex = layer == 0 ? ring[kLevels - 1][0] : top;
    u32 mid[kBranches], tip[kBranches];
    const double spin = rnd(0, 6.28);
    for (int i = 0; i < kBranches; i++) {
      const double a = 2.0 * PI * i / kBranches + spin;
      const double wob = rnd(0.88, 1.12);
      mid[i] = sim.addNode(foot + V3{std::cos(a) * r0 * 0.56, std::sin(a) * r0 * 0.56, zm}, 6.0, NF_COLLIDE, 0.7f);
      tip[i] = sim.addNode(foot + V3{std::cos(a) * r0 * wob, std::sin(a) * r0 * wob, zb + rnd(-0.15, 0.15)}, 4.5, NF_COLLIDE, 0.7f);
    }
    const int lv = nearestLevel(zm);
    for (int i = 0; i < kBranches; i++) {
      const int j = (i + 1) % kBranches;
      sim.addBeam(mid[i], ring[lv][i % kSides], branch, kTagBranch);
      sim.addBeam(mid[i], ring[std::max(lv - 1, 1)][(i + 2) % kSides], branch, kTagBranch);
      sim.addBeam(mid[i], apex, branch, kTagBranch);
      sim.addBeam(mid[i], tip[i], branch, kTagBranch);
      sim.addBeam(tip[i], ring[lv][(i + 3) % kSides], brace, kTagBranch);
      sim.addBeam(mid[i], mid[j], brace, kTagBranch);
      sim.addBeam(tip[i], tip[j], brace, kTagBranch);
      sim.addBeam(tip[i], mid[j], brace, kTagBranch);
      sim.addTri(apex, mid[i], mid[j], TF_VISUAL, needle, 0.5f);
      sim.addTri(mid[i], tip[i], tip[j], TF_VISUAL, needle, 0.5f);
      sim.addTri(mid[i], tip[j], mid[j], TF_VISUAL, needle, 0.5f);
    }
  }
  sim.endObj();
  (void)w;
}

void addSign(Sim& sim, const World& w, double x, double y, double yaw, u8 color) {
  sim.beginObj(OBJ_PROP, false);
  const Frame f = frameAt(w, x, y, yaw);
  auto post = column(sim, f, 0, 0, 0.0, 2.3, 0.035, 5, 1.0, mPost, mPostRoot, COL_STEEL);
  // Plate: a light square, bolted at two heights.
  const double hw = 0.32, z0 = 1.65, z1 = 2.35;
  u32 p[4][2];
  for (int i = 0; i < 4; i++) {
    const double yy = lerpd(-hw, hw, i / 3.0);
    p[i][0] = sim.addNode(f.at(0.05, yy, z0), 0.7, NF_COLLIDE);
    p[i][1] = sim.addNode(f.at(0.05, yy, z1), 0.7, NF_COLLIDE);
  }
  for (int i = 0; i < 4; i++) {
    sim.addBeam(p[i][0], p[i][1], mPlate);
    if (i < 3) {
      sim.addBeam(p[i][0], p[i + 1][0], mPlate);
      sim.addBeam(p[i][1], p[i + 1][1], mPlate);
      sim.addBeam(p[i][0], p[i + 1][1], mPlate);
      sim.addBeam(p[i + 1][0], p[i][1], mPlate);
      quadTris(sim, p[i][0], p[i + 1][0], p[i + 1][1], p[i][1], color);
      quadTris(sim, p[i][0], p[i][1], p[i + 1][1], p[i + 1][0], color);
    }
  }
  const u32 lvl3 = post[3 * 3], lvl4 = post[4 * 3];
  for (u32 q : {p[1][0], p[2][0], p[1][1], p[2][1]}) {
    sim.addBeam(q, lvl3, mPlateBolt);
    sim.addBeam(q, lvl4, mPlateBolt);
    sim.addBeam(q, post[3 * 3 + 1], mPlateBolt);
  }
  sim.endObj();
}

void addFence(Sim& sim, const World& w, double x, double y, double yaw, int bays) {
  sim.beginObj(OBJ_PROP, false);
  const Frame f = frameAt(w, x, y, yaw);
  constexpr double kBay = 2.4, kPostH = 1.15, kR = 0.06;
  std::vector<std::vector<u32>> posts;
  for (int i = 0; i <= bays; i++) {
    const double px = i * kBay;
    const double dz = w.height(f.at(px, 0, 0).x, f.at(px, 0, 0).y) - f.o.z;
    posts.push_back(column(sim, f, px, 0, dz, dz + kPostH, kR, 4, 1.4, mWood, mWoodRoot,
                           i % 2 ? COL_WOOD : COL_WOOD_DARK));
  }
  // Two rails per bay, each a chain of short sections so that it snaps somewhere along its span.
  for (int i = 0; i < bays; i++) {
    const double px = i * kBay;
    const double dz0 = sim.pos[posts[i][0]].z - f.o.z;
    const double dz1 = sim.pos[posts[i + 1][0]].z - f.o.z;
    for (int level = 1; level <= 2; level++) {
      constexpr int kSeg = 3;
      std::vector<std::array<u32, 2>> chain;
      for (int s = 0; s <= kSeg; s++) {
        const double t = double(s) / kSeg;
        const double zz = lerpd(dz0, dz1, t) + level * kPostH / 3.0;
        const u32 a = sim.addNode(f.at(px + t * kBay, 0.09, zz - 0.05), 0.9, NF_COLLIDE);
        const u32 b = sim.addNode(f.at(px + t * kBay, 0.09, zz + 0.05), 0.9, NF_COLLIDE);
        chain.push_back({a, b});
        sim.addBeam(a, b, mWoodRail);
      }
      for (int s = 0; s < kSeg; s++) {
        sim.addBeam(chain[s][0], chain[s + 1][0], mWoodRail);
        sim.addBeam(chain[s][1], chain[s + 1][1], mWoodRail);
        sim.addBeam(chain[s][0], chain[s + 1][1], mWoodRail);
        sim.addBeam(chain[s][1], chain[s + 1][0], mWoodRail);
        const u8 c = COL_WOOD;
        quadTris(sim, chain[s][0], chain[s + 1][0], chain[s + 1][1], chain[s][1], c);
        quadTris(sim, chain[s][0], chain[s][1], chain[s + 1][1], chain[s + 1][0], c);
      }
      const size_t lvl = size_t(level) * 3;
      sim.addBeam(chain[0][0], posts[i][lvl], mNail);
      sim.addBeam(chain[0][1], posts[i][lvl + 1], mNail);
      sim.addBeam(chain[kSeg][0], posts[i + 1][lvl], mNail);
      sim.addBeam(chain[kSeg][1], posts[i + 1][lvl + 1], mNail);
    }
  }
  sim.endObj();
}

void addStoneWall(Sim& sim, const World& w, double x, double y, double yaw, int columns, int rows) {
  sim.beginObj(OBJ_PROP, false);
  const Frame f = frameAt(w, x, y, yaw);
  constexpr double kL = 0.40, kD = 0.22, kH = 0.20, kGap = 0.003;
  constexpr double kMass = 18.0;  // light aerated stone, per block
  struct Stone { u32 n[8]; int col, row; };
  std::vector<Stone> st;
  // Stack like a real wall on uneven ground: each stone rests on whatever is highest beneath it,
  // tracked in half-stone wide slots along the wall.
  std::vector<double> slotTop(size_t(columns) * 2 + 4, -1e9);
  for (int row = 0; row < rows; row++) {
    const double shift = row % 2 ? kL * 0.5 : 0.0;
    const int cols = columns - (row % 2);
    for (int col = 0; col < cols; col++) {
      Stone s{};
      s.col = col;
      s.row = row;
      const double x0 = col * (kL + kGap) + shift;
      const int slot = int(std::lround(x0 / (kL * 0.5 + kGap * 0.5)));
      double z0 = std::max(slotTop[slot], slotTop[slot + 1]);
      const V3 e0 = f.at(x0, 0, 0), e1 = f.at(x0 + kL, 0, 0), e2 = f.at(x0, kD, 0), e3 = f.at(x0 + kL, kD, 0);
      for (const V3& e : {e0, e1, e2, e3}) z0 = std::max(z0, w.height(e.x, e.y) - f.o.z);
      z0 += kGap;
      slotTop[slot] = slotTop[slot + 1] = z0 + kH;
      for (int i = 0; i < 8; i++) {
        const V3 p = f.at(x0 + ((i & 1) ? kL : 0), ((i & 2) ? kD : 0), z0 + ((i & 4) ? kH : 0));
        s.n[i] = sim.addNode(p, kMass / 8.0, NF_COLLIDE, 0.7f);
      }
      lattice8(sim, s.n, mStone);
      sim.addBlock(s.n);
      const u8 c = (col * 7 + row * 3) % 3 == 0 ? COL_STONE2 : COL_STONE;
      boxTris(sim, s.n, c, TF_VISUAL);
      st.push_back(s);
    }
  }
  // Mortar joints between neighbouring stones, and a footing joint on the ground row.
  auto mortar = [&](u32 a, u32 b) { sim.addBeam(a, b, mMortar); };
  for (size_t i = 0; i < st.size(); i++) {
    for (size_t j = i + 1; j < st.size(); j++) {
      const Stone& a = st[i];
      const Stone& b = st[j];
      if (a.row == b.row && b.col == a.col + 1) {
        for (int yz : {0, 2, 4, 6}) mortar(a.n[1 | yz], b.n[0 | yz]);
        mortar(a.n[1], b.n[2]);
        mortar(a.n[3], b.n[0]);
        mortar(a.n[5], b.n[6]);
        mortar(a.n[7], b.n[4]);
      } else if (b.row == a.row + 1) {
        const double sa = a.row % 2 ? kL * 0.5 : 0.0, sb = b.row % 2 ? kL * 0.5 : 0.0;
        const double ca = a.col * (kL + kGap) + sa + kL * 0.5;
        const double cb = b.col * (kL + kGap) + sb + kL * 0.5;
        if (std::fabs(ca - cb) < kL * 0.8) {
          const bool bRight = cb > ca;
          // Bed joint: the top of one stone to the bottom of the course above.
          const int t0 = bRight ? 5 : 4, t1 = bRight ? 7 : 6;
          const int b0 = bRight ? 0 : 1, b1 = bRight ? 2 : 3;
          mortar(a.n[t0], b.n[b0]);
          mortar(a.n[t1], b.n[b1]);
          mortar(a.n[t0], b.n[b1]);
          mortar(a.n[t1], b.n[b0]);
        }
      }
    }
  }
  sim.endObj();
}

void addGuardrail(Sim& sim, const World& w, double x, double y, double yaw, int bays) {
  sim.beginObj(OBJ_PROP, false);
  const Frame f = frameAt(w, x, y, yaw);
  constexpr double kBay = 2.0, kH = 0.75;
  constexpr int kSec = 2;  // rail sections per bay
  std::vector<std::vector<u32>> posts;
  for (int i = 0; i <= bays; i++) {
    const double px = i * kBay;
    const double dz = w.height(f.at(px, 0, 0).x, f.at(px, 0, 0).y) - f.o.z;
    posts.push_back(column(sim, f, px, 0, dz, dz + kH, 0.05, 3, 2.0, mRailPost, mRailRoot, COL_STEEL));
  }
  // W-beam as a triangular chain, in front of the posts (towards -y of the local frame).
  const int total = bays * kSec;
  std::vector<std::array<u32, 3>> chain;
  for (int i = 0; i <= total; i++) {
    const double t = double(i) / kSec;
    const double px = t * kBay;
    const V3 base = f.at(px, 0, 0);
    const double dz = w.height(base.x, base.y) - f.o.z;
    const u32 a = sim.addNode(f.at(px, -0.11, dz + kH - 0.17), 2.2, NF_COLLIDE, 0.3f);
    const u32 b = sim.addNode(f.at(px, -0.11, dz + kH + 0.0), 2.2, NF_COLLIDE, 0.3f);
    const u32 c = sim.addNode(f.at(px, -0.04, dz + kH - 0.09), 2.2, NF_COLLIDE, 0.3f);
    chain.push_back({a, b, c});
    sim.addBeam(a, b, mRail);
    sim.addBeam(b, c, mRail);
    sim.addBeam(a, c, mRail);
  }
  for (int i = 0; i < total; i++) {
    for (int k = 0; k < 3; k++) {
      sim.addBeam(chain[i][k], chain[i + 1][k], mRail);
      sim.addBeam(chain[i][k], chain[i + 1][(k + 1) % 3], mRail);
    }
    quadTris(sim, chain[i][0], chain[i + 1][0], chain[i + 1][1], chain[i][1], COL_GUARDRAIL);
    quadTris(sim, chain[i][0], chain[i][1], chain[i + 1][1], chain[i + 1][0], COL_GUARDRAIL);
  }
  for (int i = 0; i <= bays; i++) {
    const auto& rail = chain[size_t(i) * kSec];
    const size_t top = (3 - 1) * 3;
    sim.addBeam(rail[0], posts[i][top], mRailBolt);
    sim.addBeam(rail[1], posts[i][top], mRailBolt);
    sim.addBeam(rail[2], posts[i][top + 1], mRailBolt);
    sim.addBeam(rail[2], posts[i][top + 2], mRailBolt);
  }
  sim.endObj();
}

void addCone(Sim& sim, const World& w, double x, double y) {
  sim.beginObj(OBJ_PROP, false);
  const Frame f = frameAt(w, x, y, 0);
  constexpr double r = 0.16, h = 0.72;
  u32 b[4];
  for (int i = 0; i < 4; i++) {
    const double a = PI * 0.5 * i;
    b[i] = sim.addNode(f.at(r * std::cos(a), r * std::sin(a), 0.002), 0.8, NF_COLLIDE, 0.8f);
  }
  const u32 t = sim.addNode(f.at(0, 0, h), 0.4, NF_COLLIDE, 0.5f);
  for (int i = 0; i < 4; i++) {
    sim.addBeam(b[i], t, mCone);
    sim.addBeam(b[i], b[(i + 1) % 4], mCone);
    sim.addTri(b[i], b[(i + 1) % 4], t, 0, COL_CONE, 0.5f);
  }
  sim.addBeam(b[0], b[2], mCone);
  sim.addBeam(b[1], b[3], mCone);
  sim.endObj();
}

void addBarrier(Sim& sim, const World& w, double x, double y, double yaw) {
  sim.beginObj(OBJ_PROP, false);
  const Frame f = frameAt(w, x, y, yaw);
  constexpr double kL = 2.0, kD = 0.45, kH = 0.8, kMass = 450.0;
  u32 n[8];
  for (int i = 0; i < 8; i++)
    n[i] = sim.addNode(f.at((i & 1) ? kL : 0, ((i & 2) ? kD : 0) - kD * 0.5, ((i & 4) ? kH : 0) + 0.002),
                       kMass / 8.0, NF_COLLIDE, 0.7f);
  lattice8(sim, n, mConcrete);
  sim.addBlock(n);
  boxTris(sim, n, COL_CONCRETE, TF_VISUAL);
  sim.endObj();
}

void populateWorld(Sim& sim, const World& w) {
  constexpr double kHalfPi = PI * 0.5;

  // The woods along the country road.
  {
    u32 seed = 31337;
    for (const V3& t : w.trees()) {
      seed = seed * 2654435761u + 17u;
      addTree(sim, w, t, seed);
    }
  }

  // Gauntlet in the main lane along y = 0, from the spawn point towards the end wall.
  addSign(sim, w, -28, 0.5, 0, COL_SIGN_RED);
  addSign(sim, w, -28, -2.2, 0, COL_SIGN_BLUE);
  addSign(sim, w, -28, 3.2, 0, COL_SIGN_WHITE);
  addFence(sim, w, -8, -7.2, kHalfPi, 6);             // wooden fence across the lane
  for (int i = 0; i < 7; i++) addCone(sim, w, 18, -6.0 + i * 2.0);
  addStoneWall(sim, w, 48, -3.2, kHalfPi, 16, 6);     // stone wall across the lane
  addGuardrail(sim, w, 85, -7.0, kHalfPi, 7);         // guardrail across the lane
  for (int i = 0; i < 6; i++) addBarrier(sim, w, 120, -6.0 + i * 2.05, kHalfPi);

  // Row of signs and a garden wall in a second lane.
  for (int i = 0; i < 8; i++) addSign(sim, w, -20.0 + i * 12.0, 18.0, 0, i % 2 ? COL_SIGN_RED : COL_SIGN_BLUE);
  addStoneWall(sim, w, -20, 24, 0, 25, 3);
  addFence(sim, w, 25, 24, 0, 8);

  // Moose test course, marked with cones along lane y = -60.
  const double mx = -40;
  auto gate = [&](double x, double yC, double wd) {
    addCone(sim, w, x, yC - wd * 0.5);
    addCone(sim, w, x, yC + wd * 0.5);
  };
  for (int i = 0; i < 3; i++) gate(mx + i * 6.0, -60.0, 2.6);
  for (int i = 0; i < 3; i++) gate(mx + 12.0 + 13.5 + i * 5.5, -60.0 + 3.3, 3.0);
  for (int i = 0; i < 3; i++) gate(mx + 12.0 + 13.5 + 11.0 + 12.5 + i * 6.5, -60.0, 3.0);

  // Roadside furniture along the loop.
  const Road& loop = w.roads()[0];
  const size_t n = loop.pts.size();
  for (size_t i = 0; i < n; i += 34) {
    const RoadPoint& a = loop.pts[i];
    const RoadPoint& b = loop.pts[(i + 1) % n];
    const double tx = b.x - a.x, ty = b.y - a.y, tl = std::hypot(tx, ty);
    const double nx = -ty / tl, ny = tx / tl;
    const double yaw = std::atan2(ty, tx);
    const int kind = int(i / 34);
    const double side = kind % 2 ? 1.0 : -1.0;
    const double off = loop.halfWidth + 2.0;
    const double px = a.x + nx * off * side, py = a.y + ny * off * side;
    if (kind % 7 == 3) {
      addGuardrail(sim, w, px, py, yaw + (side > 0 ? 0.0 : PI), 6);
    } else if (kind % 7 == 5) {
      addFence(sim, w, px, py, yaw, 5);
    } else if (kind % 11 == 9) {
      addStoneWall(sim, w, px, py, yaw, 14, 4);
    } else {
      addSign(sim, w, px, py, yaw + kHalfPi, u8(COL_SIGN_RED + kind % 3));
    }
  }
}

void populateArena(Sim& sim, const World& w) {
  u32 seed = 31337;
  for (const V3& t : w.trees()) {
    seed = seed * 2654435761u + 17u;
    addTree(sim, w, t, seed);
  }
  // A wall or a fence by each house site, for something to drive through until the houses stand.
  int n = 0;
  for (const V3& h : w.houseSites()) {
    const double yaw = double(n) * 0.9;
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    if (n % 3 == 0) addStoneWall(sim, w, h.x + cy * 8.0, h.y + sy * 8.0, yaw + PI * 0.5, 10, 3);
    else addFence(sim, w, h.x + cy * 8.0, h.y + sy * 8.0, yaw + PI * 0.5, 5);
    n++;
  }
}

u32 addMarker(Sim& sim, V3 half, u8 color) {
  sim.beginObj(OBJ_PROP, false);
  u32 n[8];
  for (int i = 0; i < 8; i++) n[i] = sim.addNode(V3{(i & 1) ? half.x : -half.x, (i & 2) ? half.y : -half.y, kMarkerDepth + ((i & 4) ? half.z : -half.z)}, 1.0, NF_FIXED);
  boxTris(sim, n, color, TF_VISUAL);
  sim.endObj();
  return n[0];
}

void moveMarker(Sim& sim, u32 first, V3 at) {
  for (u32 i = 0; i < 8; i++) {
    const V3 r = sim.rest[first + i];
    sim.pos[first + i] = at + V3{r.x, r.y, r.z - kMarkerDepth};
  }
}
