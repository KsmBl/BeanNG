#include "sim.h"
#include <algorithm>
#include "world.h"

namespace {
constexpr double kContactOmega = 1000.0;  // contact stiffness = m * omega^2, stable at 2 kHz
constexpr double kContactZeta = 0.35;
constexpr double kMaxSpeed = 250.0;
constexpr double kMaxDv = 25.0;
// Contact force saturates with depth. A node found deep inside a surface (it arrived from the side,
// or the surface moved over it) must be pushed out, not fired out: an unsaturated penalty spring
// would release m*(omega*d)^2/2 of energy, which at 14 cm depth is 140 m/s.
constexpr double kMaxPenPair = 0.025;
constexpr double kMaxPenWorld = 0.05;
constexpr int kHashBits = 14;
constexpr double kCell = 0.7;
constexpr int kPairInterval = 8;

inline u32 cellHash(int x, int y, int z) {
  u32 h = u32(x) * 73856093u ^ u32(y) * 19349663u ^ u32(z) * 83492791u;
  return h >> (32 - kHashBits);
}
inline int cellOf(double v) { return int(std::floor(v / kCell)); }
}  // namespace

u16 Sim::beginObj(ObjKind kind, bool alwaysAwake) {
  Obj o;
  o.kind = kind;
  o.alwaysAwake = alwaysAwake;
  o.awake = alwaysAwake;
  o.nodeStart = u32(pos.size());
  o.beamStart = u32(beams.size());
  objs.push_back(o);
  return u16(objs.size() - 1);
}

void Sim::endObj() {
  Obj& o = objs.back();
  o.nodeCount = u32(pos.size()) - o.nodeStart;
  o.beamCount = u32(beams.size()) - o.beamStart;
  computeBounds(o);
}

u32 Sim::addNode(V3 p, double m, u8 fl, float friction) {
  pos.push_back(p);
  rest.push_back(p);
  vel.push_back({});
  force.push_back({});
  mass.push_back(m);
  invMass.push_back((fl & NF_FIXED) ? 0.0 : 1.0 / m);
  mu.push_back(friction);
  flags.push_back(fl);
  nodeObj.push_back(u16(objs.size() - 1));
  return u32(pos.size() - 1);
}

u32 Sim::addBeam(u32 a, u32 b, const Mat& m, u8 tag, u8 fl) {
  Beam bm{};
  bm.a = a;
  bm.b = b;
  bm.L0 = bm.Lorig = len(pos[b] - pos[a]);
  bm.k = m.k;
  bm.c = m.c;
  bm.yC = m.yC;
  bm.yT = m.yT;
  bm.hard = m.hard;
  bm.maxStrain = m.maxStrain;
  bm.fBreak = m.fBreak;
  bm.tag = tag;
  bm.flags = fl;
  beams.push_back(bm);
  return u32(beams.size() - 1);
}

void Sim::addTri(u32 a, u32 b, u32 c, u8 fl, u8 color, float friction) {
  Tri t{};
  t.a = a;
  t.b = b;
  t.c = c;
  t.obj = u16(objs.size() - 1);
  t.flags = fl;
  t.color = color;
  t.mu = friction;
  tris.push_back(t);
}

void Sim::addBlock(const u32 n[8]) {
  Block b{};
  for (int i = 0; i < 8; i++) b.n[i] = n[i];
  b.obj = u16(objs.size() - 1);
  blocks.push_back(b);
}

void Sim::finalize() {
  active.clear();
  for (size_t i = 0; i < objs.size(); i++)
    if (objs[i].awake) active.push_back(u32(i));
  stamp.assign(tris.size() + blocks.size(), 0xffffffffu);
  hashHead.assign(size_t(1) << kHashBits, -1);
}

void Sim::computeBounds(Obj& o) {
  V3 mn{1e30, 1e30, 1e30}, mx{-1e30, -1e30, -1e30};
  for (u32 i = o.nodeStart; i < o.nodeStart + o.nodeCount; i++) {
    const V3& p = pos[i];
    mn = {std::min(mn.x, p.x), std::min(mn.y, p.y), std::min(mn.z, p.z)};
    mx = {std::max(mx.x, p.x), std::max(mx.y, p.y), std::max(mx.z, p.z)};
  }
  o.bbMin = mn;
  o.bbMax = mx;
}

void Sim::wake(u16 id) {
  Obj& o = objs[id];
  if (o.awake) return;
  o.awake = true;
  o.sleepTimer = 0;
  active.push_back(id);
}

double Sim::maxAccelEstimate(u32* worstNode) const {
  // Upper bound on the largest eigenvalue of M^-1 K. A symplectic Euler step is stable while
  // lambda < (2/dt)^2. Summing |k| per node overestimates by roughly the 3D projection factor.
  std::vector<double> sum(pos.size(), 0.0);
  for (const Beam& b : beams) {
    sum[b.a] += b.k;
    sum[b.b] += b.k;
  }
  double best = 0;
  for (size_t i = 0; i < pos.size(); i++) {
    double l = sum[i] * invMass[i];
    if (l > best) {
      best = l;
      if (worstNode) *worstNode = u32(i);
    }
  }
  return best;
}

void Sim::forces(double dt) {
  const V3 gv{0, 0, -g};
  for (u32 oi : active) {
    const Obj& o = objs[oi];
    for (u32 i = o.nodeStart, e = o.nodeStart + o.nodeCount; i < e; i++) force[i] = gv * mass[i];
  }

  double plastic = 0;
  for (u32 oi : active) {
    const Obj& o = objs[oi];
    const bool isCar = o.kind == OBJ_CAR;
    double objPlastic = 0;
    for (u32 bi = o.beamStart, be = o.beamStart + o.beamCount; bi < be; bi++) {
      Beam& b = beams[bi];
      if (b.flags & BF_BROKEN) continue;
      const V3 d = pos[b.b] - pos[b.a];
      const double l2 = len2(d);
      if (l2 < 1e-10) continue;
      const double L = std::sqrt(l2);
      const V3 n = d / L;
      double f = b.k * (L - b.L0);

      if (f > b.yT && b.yT > 0) {
        // Tensile plastic flow: the beam stretches permanently and hardens.
        const double dl = (f - b.yT) / b.k;
        b.L0 += dl;
        objPlastic += b.yT * dl;
        const double strain = (b.L0 - b.Lorig) / b.Lorig;
        b.yT += b.hard * dl / b.Lorig;
        f = b.yT;
        if (b.maxStrain > 0 && strain > b.maxStrain) {
          b.flags |= BF_BROKEN;
          brokenBeams++;
          if (isCar) event(EV_CRUNCH, pos[b.a], vel[b.a], float(f));
          continue;
        }
      } else if (f < -b.yC && b.yC > 0) {
        // Compressive plastic flow (buckling). Sheet folds up until it stacks solid, at which
        // point the force climbs steeply again: that lock-up is what stops a crumple zone.
        const double dl = (-f - b.yC) / b.k;
        b.L0 -= dl;
        objPlastic += b.yC * dl;
        const double ratio = b.L0 / b.Lorig;
        const double lockup = ratio < 0.45 ? 1.0 + 30.0 * (0.45 - ratio) : 1.0;
        b.yC += b.hard * lockup * dl / b.Lorig;
        f = -b.yC;
      }

      if (b.flags & (BF_COMPRESS_ONLY | BF_TENSION_ONLY)) {
        if ((b.flags & BF_COMPRESS_ONLY) && f > 0) continue;
        if ((b.flags & BF_TENSION_ONLY) && f < 0) continue;
      }

      if (b.fBreak > 0 && std::fabs(f) > b.fBreak) {
        b.flags |= BF_BROKEN;
        brokenBeams++;
        if (isCar) event(EV_CRUNCH, pos[b.a], vel[b.a], float(std::fabs(f)));
        continue;
      }

      const double rel = dot(vel[b.b] - vel[b.a], n);
      f += b.c * rel;
      b.force = float(f);
      const V3 fv = n * f;
      force[b.a] += fv;
      force[b.b] -= fv;
    }
    if (isCar) plastic += objPlastic;
  }
  plasticEnergy += plastic;

  if (grab.node >= 0) {
    // Critically damped spring towards the mouse target; capped so it cannot rip the car apart
    // by itself in one step.
    const u32 n = u32(grab.node);
    V3 f = (grab.target - pos[n]) * (mass[n] * 400.0) - vel[n] * (mass[n] * 40.0);
    const double fl = len(f);
    if (fl > 15000.0) f = f * (15000.0 / fl);
    force[n] += f;
    wake(nodeObj[n]);
  }
}

void Sim::finish(double dt) {
  stepCount++;
  if (stepCount % kPairInterval == 1) rebuildPairs(dt);
  collidePairs(dt);
  collideWorld(dt);
  integrate(dt);
  if (stepCount % 64 == 0) sleepWake();
}

void Sim::event(EventKind k, V3 p, V3 v, float s) {
  if (events.size() < 512) events.push_back({p, v, s, k});
}

bool Sim::triPairAllowed(u32 node, const Tri& t) const {
  if (node == t.a || node == t.b || node == t.c) return false;
  // A surface whose beams have torn is no longer a surface: without this, a stretched triangle
  // between a flung piece and its anchor would act as an invisible wall.
  constexpr double kTear2 = 1.5 * 1.5;
  if (len2(pos[t.b] - pos[t.a]) > kTear2 * len2(rest[t.b] - rest[t.a]) ||
      len2(pos[t.c] - pos[t.b]) > kTear2 * len2(rest[t.c] - rest[t.b]) ||
      len2(pos[t.a] - pos[t.c]) > kTear2 * len2(rest[t.a] - rest[t.c]))
    return false;
  if (nodeObj[node] != t.obj) return !(t.flags & TF_HARD);  // others only touch the outer skin
  // Self-collision: only bulkhead-type surfaces, and only for nodes that started out on the
  // front side of the surface. One-sided surfaces would otherwise "pull through" everything
  // that merely sits behind them, such as the whole cabin behind the firewall.
  if (!(t.flags & TF_HARD)) return false;
  const V3 n = cross(rest[t.b] - rest[t.a], rest[t.c] - rest[t.a]);
  return dot(rest[node] - rest[t.a], n) > 0.0;
}

void Sim::rebuildPairs(double dt) {
  statTriPairs = triPairs.size();
  statBlockPairs = blockPairs.size();
  triPairs.clear();
  blockPairs.clear();
  std::fill(hashHead.begin(), hashHead.end(), -1);
  hashEntries.clear();

  // Margin must cover the relative motion until the next rebuild.
  double vmax = 0;
  for (u32 oi : active) {
    const Obj& o = objs[oi];
    for (u32 i = o.nodeStart, e = o.nodeStart + o.nodeCount; i < e; i += 7)
      vmax = std::max(vmax, len2(vel[i]));
  }
  vmax = std::sqrt(vmax);
  pairMargin = 0.05 + 2.2 * vmax * dt * kPairInterval;

  auto insertBox = [&](u32 prim, V3 mn, V3 mx) {
    const int x0 = cellOf(mn.x - pairMargin), x1 = cellOf(mx.x + pairMargin);
    const int y0 = cellOf(mn.y - pairMargin), y1 = cellOf(mx.y + pairMargin);
    const int z0 = cellOf(mn.z - pairMargin), z1 = cellOf(mx.z + pairMargin);
    for (int x = x0; x <= x1; x++)
      for (int y = y0; y <= y1; y++)
        for (int z = z0; z <= z1; z++) {
          const u32 h = cellHash(x, y, z);
          hashEntries.push_back({prim, hashHead[h]});
          hashHead[h] = int(hashEntries.size() - 1);
        }
  };

  const u32 nTris = u32(tris.size());
  for (u32 i = 0; i < nTris; i++) {
    const Tri& t = tris[i];
    if ((t.flags & TF_VISUAL) || !objs[t.obj].awake) continue;
    const V3& a = pos[t.a];
    const V3& b = pos[t.b];
    const V3& c = pos[t.c];
    insertBox(i,
              {std::min({a.x, b.x, c.x}), std::min({a.y, b.y, c.y}), std::min({a.z, b.z, c.z})},
              {std::max({a.x, b.x, c.x}), std::max({a.y, b.y, c.y}), std::max({a.z, b.z, c.z})});
  }
  for (u32 i = 0; i < blocks.size(); i++) {
    const Block& bl = blocks[i];
    if (!objs[bl.obj].awake) continue;
    V3 mn = pos[bl.n[0]], mx = mn;
    for (int k = 1; k < 8; k++) {
      const V3& p = pos[bl.n[k]];
      mn = {std::min(mn.x, p.x), std::min(mn.y, p.y), std::min(mn.z, p.z)};
      mx = {std::max(mx.x, p.x), std::max(mx.y, p.y), std::max(mx.z, p.z)};
    }
    insertBox(nTris + i, mn, mx);
  }

  for (u32 oi : active) {
    const Obj& o = objs[oi];
    for (u32 i = o.nodeStart, e = o.nodeStart + o.nodeCount; i < e; i++) {
      if (!(flags[i] & NF_COLLIDE)) continue;
      const V3& p = pos[i];
      const u32 h = cellHash(cellOf(p.x), cellOf(p.y), cellOf(p.z));
      for (int it = hashHead[h]; it >= 0; it = hashEntries[it].next) {
        const u32 prim = hashEntries[it].prim;
        if (stamp[prim] == i) continue;  // a primitive spanning hash-colliding cells
        stamp[prim] = i;
        // The hash only says "same cell". Most candidates are far apart, and every pair kept here
        // is tested on each of the next steps, so an exact box test now pays for itself many times.
        if (prim < nTris) {
          const Tri& t = tris[prim];
          const V3& a = pos[t.a];
          const V3& b = pos[t.b];
          const V3& c = pos[t.c];
          const double m = pairMargin;
          if (p.x < std::min({a.x, b.x, c.x}) - m || p.x > std::max({a.x, b.x, c.x}) + m ||
              p.y < std::min({a.y, b.y, c.y}) - m || p.y > std::max({a.y, b.y, c.y}) + m ||
              p.z < std::min({a.z, b.z, c.z}) - m || p.z > std::max({a.z, b.z, c.z}) + m)
            continue;
          if (triPairAllowed(i, t)) triPairs.push_back({i, prim});
        } else {
          const Block& bl = blocks[prim - nTris];
          V3 mn = pos[bl.n[0]], mx = mn;
          bool own = bl.n[0] == i;
          for (int k = 1; k < 8; k++) {
            const V3& q = pos[bl.n[k]];
            mn = {std::min(mn.x, q.x), std::min(mn.y, q.y), std::min(mn.z, q.z)};
            mx = {std::max(mx.x, q.x), std::max(mx.y, q.y), std::max(mx.z, q.z)};
            own |= bl.n[k] == i;
          }
          const double m = pairMargin;
          if (own || p.x < mn.x - m || p.x > mx.x + m || p.y < mn.y - m || p.y > mx.y + m ||
              p.z < mn.z - m || p.z > mx.z + m)
            continue;
          blockPairs.push_back({i, prim - nTris});
        }
      }
    }
    // stamp stores the last node seen; node ids are unique across objects, so no reset needed.
  }
  std::fill(stamp.begin(), stamp.end(), 0xffffffffu);
}

void Sim::contactFriction(u32 n, V3 nrm, double fn, double muv, double dt, V3 relVel, double mEff,
                          V3* out) {
  // Coulomb friction with an exact stick limit. Predicting the tangential velocity at the end of
  // the step and never removing more than all of it gives true static friction: no creep on a
  // slope and no sign-flip jitter.
  V3 vt = relVel + force[n] * (invMass[n] * dt);
  vt -= nrm * dot(vt, nrm);
  const double vtl = len(vt);
  if (vtl < 1e-9) {
    *out = {};
    return;
  }
  const double need = mEff * vtl / dt;
  const double ft = std::min(muv * fn, need);
  *out = vt * (-ft / vtl);
}

// Contact resolution runs in two passes. Each contact is stable on its own, but stiffness and
// damping of simultaneous contacts add up on a shared node, and a light prop node touched by ten
// car triangles would exceed the explicit stability limit. So: first find all active contacts and
// count how many load each node, then share the contact stiffness among them.
void Sim::collidePairs(double dt) {
  constexpr double kDepth = 0.14;  // surfaces are one-sided with this much "thickness"
  triHits.clear();
  blockHits.clear();
  if (contactLoad.size() != pos.size()) contactLoad.assign(pos.size(), 0.f);

  auto addLoad = [&](u32 n, float v) {
    if (contactLoad[n] == 0.f) touched.push_back(n);
    contactLoad[n] += v;
  };

  for (u32 pi = 0; pi < triPairs.size(); pi++) {
    const Pair& pr = triPairs[pi];
    const Tri& t = tris[pr.prim];
    const V3& p = pos[pr.node];
    const V3& pa = pos[t.a];
    const V3 e1 = pos[t.b] - pa;
    const V3 e2 = pos[t.c] - pa;
    const V3 nn = cross(e1, e2);
    const double nl2 = len2(nn);
    if (nl2 < 1e-12) continue;
    const V3 ap = p - pa;
    const double dn = dot(ap, nn);
    if (dn >= 0) continue;  // in front of the surface
    const double nl = std::sqrt(nl2);
    const double d = dn / nl;
    if (d < -kDepth) continue;

    // Barycentric coordinates of the projected point.
    const double inv = 1.0 / nl2;
    const double w1 = dot(cross(ap, e2), nn) * inv;
    const double w2 = dot(cross(e1, ap), nn) * inv;
    const double w0 = 1.0 - w1 - w2;
    if (w0 < -0.02 || w1 < -0.02 || w2 < -0.02) continue;

    triHits.push_back({pi, w0, w1, w2, d, nn / nl});
    addLoad(pr.node, 1.f);
    addLoad(t.a, float(std::max(w0, 0.0)));
    addLoad(t.b, float(std::max(w1, 0.0)));
    addLoad(t.c, float(std::max(w2, 0.0)));
  }

  for (u32 pi = 0; pi < blockPairs.size(); pi++) {
    const Pair& pr = blockPairs[pi];
    const Block& b = blocks[pr.prim];
    const V3& p = pos[pr.node];
    V3 cs[8];
    V3 c{};
    for (int i = 0; i < 8; i++) {
      cs[i] = pos[b.n[i]];
      c += cs[i];
    }
    c = c * 0.125;
    // Reject before doing any normalisation, the vast majority of pairs are not touching.
    const double rough = len2(cs[7] - cs[0]) * 0.25;
    if (len2(p - c) > rough) continue;

    V3 ex = ((cs[1] + cs[3] + cs[5] + cs[7]) - (cs[0] + cs[2] + cs[4] + cs[6])) * 0.25;
    V3 ey = ((cs[2] + cs[3] + cs[6] + cs[7]) - (cs[0] + cs[1] + cs[4] + cs[5])) * 0.25;
    V3 ez = ((cs[4] + cs[5] + cs[6] + cs[7]) - (cs[0] + cs[1] + cs[2] + cs[3])) * 0.25;
    const double lx = len(ex), ly = len(ey), lz = len(ez);
    if (lx < 1e-4 || ly < 1e-4 || lz < 1e-4) continue;
    ex = ex / lx;
    ey = ey / ly;
    ez = ez / lz;
    const V3 q = p - c;
    const double qx = dot(q, ex), qy = dot(q, ey), qz = dot(q, ez);
    const double hx = lx * 0.5, hy = ly * 0.5, hz = lz * 0.5;
    const double px = hx - std::fabs(qx), py = hy - std::fabs(qy), pz = hz - std::fabs(qz);
    if (px <= 0 || py <= 0 || pz <= 0) continue;

    BlockHit h;
    h.pair = pi;
    if (px < py && px < pz) {
      h.n = qx > 0 ? ex : -ex;
      h.depth = px;
    } else if (py < pz) {
      h.n = qy > 0 ? ey : -ey;
      h.depth = py;
    } else {
      h.n = qz > 0 ? ez : -ez;
      h.depth = pz;
    }
    // Distribute the reaction to the corners with trilinear weights.
    const double u = clampd(qx / lx + 0.5, 0, 1), v = clampd(qy / ly + 0.5, 0, 1),
                 w = clampd(qz / lz + 0.5, 0, 1);
    for (int i = 0; i < 8; i++) {
      const double aa = (i & 1) ? u : 1 - u, bb = (i & 2) ? v : 1 - v, cc = (i & 4) ? w : 1 - w;
      h.wt[i] = aa * bb * cc;
      addLoad(b.n[i], float(h.wt[i]));
    }
    addLoad(pr.node, 1.f);
    blockHits.push_back(h);
  }

  const double kBase = kContactOmega * kContactOmega;
  const double cBase = 2.0 * kContactZeta * kContactOmega;

  for (const TriHit& h : triHits) {
    const Pair& pr = triPairs[h.pair];
    const Tri& t = tris[pr.prim];
    float load = contactLoad[pr.node];
    if (h.w0 > 0.15) load = std::max(load, contactLoad[t.a]);
    if (h.w1 > 0.15) load = std::max(load, contactLoad[t.b]);
    if (h.w2 > 0.15) load = std::max(load, contactLoad[t.c]);
    const double share = 1.0 / std::max(1.0, double(load));

    const double invTri = h.w0 * h.w0 * invMass[t.a] + h.w1 * h.w1 * invMass[t.b] + h.w2 * h.w2 * invMass[t.c];
    const double invSum = invMass[pr.node] + invTri;
    if (invSum <= 0) continue;
    const double mEff = 1.0 / invSum;
    const V3 vt = vel[t.a] * h.w0 + vel[t.b] * h.w1 + vel[t.c] * h.w2;
    const V3 vrel = vel[pr.node] - vt;
    const double vn = dot(vrel, h.n);
    const double fn = share * mEff * (kBase * std::min(-h.d, kMaxPenPair) - cBase * vn);
    if (fn <= 0) continue;

    V3 ff;
    contactFriction(pr.node, h.n, fn, std::min(double(t.mu), double(mu[pr.node])), dt, vrel, mEff * share, &ff);
    const V3 F = h.n * fn + ff;
    force[pr.node] += F;
    force[t.a] -= F * h.w0;
    force[t.b] -= F * h.w1;
    force[t.c] -= F * h.w2;

    if (std::fabs(vn) > 6.0 && fn > 5000.0) event(EV_SPARK, pos[pr.node], vel[pr.node], float(std::fabs(vn)));
  }

  for (const BlockHit& h : blockHits) {
    const Pair& pr = blockPairs[h.pair];
    const Block& b = blocks[pr.prim];
    float load = contactLoad[pr.node];
    double invTri = 0;
    V3 vb{};
    for (int i = 0; i < 8; i++) {
      if (h.wt[i] > 0.15) load = std::max(load, contactLoad[b.n[i]]);
      invTri += h.wt[i] * h.wt[i] * invMass[b.n[i]];
      vb += vel[b.n[i]] * h.wt[i];
    }
    const double share = 1.0 / std::max(1.0, double(load));
    const double invSum = invMass[pr.node] + invTri;
    if (invSum <= 0) continue;
    const double mEff = 1.0 / invSum;
    const V3 vrel = vel[pr.node] - vb;
    const double vn = dot(vrel, h.n);
    const double fn = share * mEff * (kBase * std::min(h.depth, kMaxPenPair) - cBase * vn);
    if (fn <= 0) continue;
    V3 ff;
    contactFriction(pr.node, h.n, fn, 0.6, dt, vrel, mEff * share, &ff);
    const V3 F = h.n * fn + ff;
    force[pr.node] += F;
    for (int i = 0; i < 8; i++) force[b.n[i]] -= F * h.wt[i];
  }

  for (u32 n : touched) contactLoad[n] = 0.f;
  touched.clear();
}

void Sim::collideWorld(double dt) {
  for (u32 oi : active) {
    const Obj& o = objs[oi];
    for (u32 i = o.nodeStart, e = o.nodeStart + o.nodeCount; i < e; i++) {
      const u8 fl = flags[i];
      if ((fl & NF_FIXED) || !(fl & NF_COLLIDE)) continue;
      Contact c;
      if (!world->contact(pos[i], c)) continue;
      const double m = mass[i];
      const double vn = dot(vel[i], c.n);
      const double fn = m * kContactOmega * kContactOmega * std::min(c.depth, kMaxPenWorld) -
                        2.0 * kContactZeta * m * kContactOmega * vn;
      if (fn <= 0) continue;
      V3 ff;
      const double muv = double(mu[i]) * surfaceFriction(c.surface);
      contactFriction(i, c.n, fn, muv, dt, vel[i], m, &ff);
      force[i] += c.n * fn + ff;

      const double vt2 = len2(vel[i]) - vn * vn;
      if (vt2 > 16.0 && fn > 300.0 && (stepCount & 15) == 0) {
        const bool soft = c.surface == SURF_GRASS || c.surface == SURF_GRAVEL;
        event(soft ? EV_DUST : EV_SPARK, pos[i], vel[i], float(std::sqrt(vt2)));
      }
    }
  }
}

void Sim::integrate(double dt) {
  for (u32 oi : active) {
    const Obj& o = objs[oi];
    for (u32 i = o.nodeStart, e = o.nodeStart + o.nodeCount; i < e; i++) {
      V3 dv = force[i] * (invMass[i] * dt);
      // Last line of defence: no legitimate force changes a node's speed this much in 0.5 ms. It
      // keeps one numerically bad contact from running away and taking the whole car with it.
      const double dv2 = len2(dv);
      if (dv2 > kMaxDv * kMaxDv) dv = dv * (kMaxDv / std::sqrt(dv2));
      V3 v = vel[i] + dv;
      const double s2 = len2(v);
      if (s2 > kMaxSpeed * kMaxSpeed) v = v * (kMaxSpeed / std::sqrt(s2));
      vel[i] = v;
      pos[i] += v * dt;
    }
  }
}

void Sim::sleepWake() {
  constexpr double kSleepSpeed2 = 0.05 * 0.05;
  constexpr double kPeriod = 64.0 / 2000.0;

  // Wake sleepers touched by anything that is actually moving. A resting prop next to another
  // resting prop must not keep waking its neighbour, or everything ends up awake forever.
  const size_t nActive = active.size();
  for (size_t ai = 0; ai < nActive; ai++) {
    Obj& o = objs[active[ai]];
    computeBounds(o);
    double vmax2 = 0;
    for (u32 i = o.nodeStart, e = o.nodeStart + o.nodeCount; i < e; i++)
      vmax2 = std::max(vmax2, len2(vel[i]));

    if (vmax2 > 0.1 * 0.1) {
      // Test individual moving nodes: one splinter flying off must not stretch a bounding box
      // across half the map and wake everything inside it.
      constexpr double kMargin = 0.25;
      for (size_t j = 0; j < objs.size(); j++) {
        const Obj& s = objs[j];
        if (s.awake) continue;
        if (s.bbMin.x > o.bbMax.x + kMargin || s.bbMax.x < o.bbMin.x - kMargin ||
            s.bbMin.y > o.bbMax.y + kMargin || s.bbMax.y < o.bbMin.y - kMargin ||
            s.bbMin.z > o.bbMax.z + kMargin || s.bbMax.z < o.bbMin.z - kMargin)
          continue;
        for (u32 i = o.nodeStart, e = o.nodeStart + o.nodeCount; i < e; i++) {
          if (len2(vel[i]) < 0.1 * 0.1) continue;
          const V3& p = pos[i];
          if (p.x > s.bbMin.x - kMargin && p.x < s.bbMax.x + kMargin && p.y > s.bbMin.y - kMargin &&
              p.y < s.bbMax.y + kMargin && p.z > s.bbMin.z - kMargin && p.z < s.bbMax.z + kMargin) {
            wake(u16(j));
            break;
          }
        }
      }
    }

    if (o.alwaysAwake) continue;
    o.sleepTimer = vmax2 < kSleepSpeed2 ? o.sleepTimer + float(kPeriod) : 0.f;
  }

  // Put settled props to sleep (done separately because wake() above may grow the active list).
  for (size_t ai = 0; ai < active.size();) {
    Obj& o = objs[active[ai]];
    if (!o.alwaysAwake && o.sleepTimer > 1.5f) {
      for (u32 i = o.nodeStart, e = o.nodeStart + o.nodeCount; i < e; i++) vel[i] = {};
      o.awake = false;
      active[ai] = active.back();
      active.pop_back();
    } else {
      ai++;
    }
  }
}
