#include "sim.h"
#include <algorithm>
#include <random>
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
  o.root = u16(objs.size() - 1);
  o.nodes.resize(o.nodeCount);
  for (u32 i = 0; i < o.nodeCount; i++) o.nodes[i] = o.nodeStart + i;
  o.beamList.resize(o.beamCount);
  for (u32 i = 0; i < o.beamCount; i++) o.beamList[i] = o.beamStart + i;
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
  // Explicit dampers have a stability limit of their own: dt * sum(c) / m must stay well below 1 at
  // every node. Many stiff beams meeting at a light node would otherwise turn their dampers into a
  // source of energy. Scaling here keeps that true for anything anyone ever builds.
  constexpr double kPhysDt = 1.0 / 2000.0;
  constexpr double kMaxDampRatio = 0.35;
  std::vector<double> sumC(pos.size(), 0.0);
  for (const Beam& b : beams) {
    sumC[b.a] += b.c;
    sumC[b.b] += b.c;
  }
  std::vector<double> scale(pos.size(), 1.0);
  for (size_t i = 0; i < pos.size(); i++)
    if (invMass[i] > 0 && sumC[i] > 0) scale[i] = std::min(1.0, kMaxDampRatio * mass[i] / (kPhysDt * sumC[i]));
  for (Beam& b : beams) b.c *= std::min(scale[b.a], scale[b.b]);

  active.clear();
  for (size_t i = 0; i < objs.size(); i++)
    if (objs[i].awake) active.push_back(u32(i));
  stamp.assign(tris.size() + blocks.size(), 0xffffffffu);
  hashHead.assign(size_t(1) << kHashBits, -1);
}

void Sim::computeBounds(Obj& o) {
  V3 mn{1e30, 1e30, 1e30}, mx{-1e30, -1e30, -1e30};
  for (u32 i : o.nodes) {
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

double Sim::maxEigenvalue(u32 obj, u32* worstNode) const {
  const Obj& o = objs[obj];
  const u32 n = o.nodeCount;
  std::vector<V3> x(n), y(n);
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> uni(-1, 1);
  for (u32 i = 0; i < n; i++) x[i] = {uni(rng), uni(rng), uni(rng)};
  double lambda = 0;
  for (int it = 0; it < 400; it++) {
    std::fill(y.begin(), y.end(), V3{});
    for (u32 bi = o.beamStart; bi < o.beamStart + o.beamCount; bi++) {
      const Beam& b = beams[bi];
      if (b.flags & BF_BROKEN) continue;
      const V3 d = norm(pos[b.b] - pos[b.a]);
      const u32 ia = b.a - o.nodeStart, ib = b.b - o.nodeStart;
      const double ext = dot(x[ib] - x[ia], d) * b.k;
      y[ia] += d * ext;
      y[ib] -= d * ext;
    }
    double norm2 = 0;
    for (u32 i = 0; i < n; i++) {
      y[i] = -y[i] * invMass[o.nodeStart + i];
      norm2 += len2(y[i]);
    }
    lambda = std::sqrt(norm2);
    if (lambda < 1e-9) return 0;
    for (u32 i = 0; i < n; i++) x[i] = y[i] / lambda;
  }
  if (worstNode) {
    double best = 0;
    for (u32 i = 0; i < n; i++)
      if (len2(x[i]) > best) {
        best = len2(x[i]);
        *worstNode = o.nodeStart + i;
      }
  }
  return lambda;
}

void Sim::forces(double dt) {
  const V3 gv{0, 0, -g};
  for (u32 oi : active) {
    const Obj& o = objs[oi];
    if (o.kind == OBJ_PROP) {
      // Loose objects lose energy to internal friction, and so come to rest instead of humming on
      // their springs for ever, which is what lets them go back to sleep. It acts on slow motion
      // only: anything really flying or falling must not be held back.
      const double vSlow = o.scrap ? 7.0 : 1.5;
      const double drag = o.scrap ? 20.0 : 4.0;
      for (u32 i : o.nodes) {
        force[i] = gv * mass[i];
        const double v2 = len2(vel[i]);
        if (v2 < vSlow * vSlow) force[i] -= vel[i] * (mass[i] * drag * (1.0 - v2 / (vSlow * vSlow)));
        // The air takes hold of anything flying: a panel torn off at speed does not keep up with the car.
        else if (v2 > 100.0) force[i] -= vel[i] * (0.03 * std::sqrt(v2));
      }
    } else {
      for (u32 i : o.nodes) force[i] = gv * mass[i];
    }
  }

  double plastic = 0;
  for (u32 oi : active) {
    const Obj& o = objs[oi];
    const bool isCar = o.kind == OBJ_CAR;
    double objPlastic = 0;
    for (u32 bi : o.beamList) {
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
          breakBeam(b, isCar, f);
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
        breakBeam(b, isCar, std::fabs(f));
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
  applyBlasts(dt);

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

bool Sim::adoptWorld(const Sim& old, u32 oldCars, u32 newCars) {
  const u32 oldNode = old.objs[oldCars].nodeStart, newNode = objs[newCars].nodeStart;
  const u32 oldBeam = old.objs[oldCars].beamStart, newBeam = objs[newCars].beamStart;
  if (old.pos.size() - oldNode != pos.size() - newNode || old.beams.size() - oldBeam != beams.size() - newBeam) return false;
  if (old.blocks.size() != blocks.size()) return false;
  const long dNode = long(newNode) - long(oldNode), dBeam = long(newBeam) - long(oldBeam);
  // Originals keep their order. Pieces torn off them come after, and pieces of the old vehicle go.
  std::vector<int> objMap(old.objs.size(), -1);
  const size_t originals = objs.size();
  std::vector<Obj> keep(objs.begin(), objs.begin() + newCars);
  int next = int(newCars);
  for (size_t i = oldCars; i < old.objs.size(); i++)
    if (old.objs[i].root >= oldCars) objMap[i] = next++;
  (void)originals;
  objs = std::move(keep);
  objs.resize(size_t(next));
  for (size_t i = oldCars; i < old.objs.size(); i++) {
    if (objMap[i] < 0) continue;
    Obj o = old.objs[i];
    o.nodeStart = u32(long(o.nodeStart) + dNode);
    o.beamStart = u32(long(o.beamStart) + dBeam);
    for (u32& n : o.nodes) n = u32(long(n) + dNode);
    for (u32& b : o.beamList) b = u32(long(b) + dBeam);
    o.root = u16(objMap[o.root]);
    objs[size_t(objMap[i])] = std::move(o);
  }
  for (size_t i = oldNode; i < old.pos.size(); i++) {
    const size_t k = size_t(long(i) + dNode);
    pos[k] = old.pos[i];
    vel[k] = old.vel[i];
    force[k] = old.force[i];
    mass[k] = old.mass[i];
    invMass[k] = old.invMass[i];
    flags[k] = old.flags[i];
    nodeObj[k] = u16(objMap[old.nodeObj[i]] >= 0 ? objMap[old.nodeObj[i]] : 0);
  }
  for (size_t i = oldBeam; i < old.beams.size(); i++) {
    Beam b = old.beams[i];
    b.a = u32(long(b.a) + dNode);
    b.b = u32(long(b.b) + dNode);
    beams[size_t(long(i) + dBeam)] = b;
  }
  // The scenery's triangles, in the same order in both. Those of the vehicle stay as they are.
  size_t oi = 0;
  for (Tri& t : tris) {
    if (t.a < newNode) continue;
    while (oi < old.tris.size() && old.tris[oi].a < oldNode) oi++;
    if (oi >= old.tris.size()) return false;
    t = old.tris[oi++];
    t.a = u32(long(t.a) + dNode);
    t.b = u32(long(t.b) + dNode);
    t.c = u32(long(t.c) + dNode);
    t.obj = u16(objMap[t.obj] >= 0 ? objMap[t.obj] : t.obj);
  }
  for (size_t i = 0; i < blocks.size(); i++) {
    Block b = old.blocks[i];
    for (u32& n : b.n) n = u32(long(n) + dNode);
    b.obj = u16(objMap[b.obj] >= 0 ? objMap[b.obj] : b.obj);
    blocks[i] = b;
  }
  active.clear();
  for (size_t i = 0; i < objs.size(); i++)
    if (objs[i].awake) active.push_back(u32(i));
  blasts.clear();
  plasticEnergy = 0;
  splitGeneration = old.splitGeneration + 1;  // what the visible meshes follow has to be looked at afresh
  triObjDirty = true;
  stamp.assign(tris.size() + blocks.size(), 0xffffffffu);
  return true;
}

void Sim::restoreObjects(const Sim& start, u32 count) {
  const Obj& last = start.objs[count - 1];
  const u32 nodeEnd = last.nodeStart + last.nodeCount, beamEnd = last.beamStart + last.beamCount;
  // Pieces that tore off are gone: their nodes go back to the objects they came from.
  for (size_t i = count; i < objs.size(); i++)
    if (objs[i].root < count) {
      objs[i].nodes.clear();
      objs[i].beamList.clear();
      objs[i].awake = objs[i].alwaysAwake = objs[i].dirty = false;
    }
  active.erase(std::remove_if(active.begin(), active.end(), [&](u32 a) { return a < count || objs[a].root < count; }), active.end());
  for (u32 i = 0; i < count; i++) {
    objs[i] = start.objs[i];
    if (objs[i].alwaysAwake) active.push_back(i);
  }
  std::copy(start.pos.begin(), start.pos.begin() + nodeEnd, pos.begin());
  std::copy(start.vel.begin(), start.vel.begin() + nodeEnd, vel.begin());
  std::copy(start.mass.begin(), start.mass.begin() + nodeEnd, mass.begin());
  std::copy(start.invMass.begin(), start.invMass.begin() + nodeEnd, invMass.begin());
  std::copy(start.flags.begin(), start.flags.begin() + nodeEnd, flags.begin());
  std::copy(start.nodeObj.begin(), start.nodeObj.begin() + nodeEnd, nodeObj.begin());
  std::copy(start.beams.begin(), start.beams.begin() + beamEnd, beams.begin());
  for (size_t i = 0; i < tris.size(); i++)
    if (tris[i].a < nodeEnd) tris[i] = start.tris[i];
  blasts.clear();
  plasticEnergy = 0;
  grab.node = -1;
  carResets++;
}

void Sim::restoreGear(const Sim& start, u16 obj, const std::vector<u32>& nodes, BodyPose then, BodyPose now, V3 vel) {
  std::vector<char> gear(pos.size(), 0);
  for (u32 n : nodes) gear[n] = 1;
  Obj& body = objs[obj];
  for (u32 n : nodes) {
    const V3 q = start.pos[n] - then.cog;
    pos[n] = now.cog + now.fwd * dot(q, then.fwd) + now.left * dot(q, then.left) + now.up * dot(q, then.up);
    Sim::vel[n] = vel;
    const u16 was = nodeObj[n];
    if (was != obj) {
      std::vector<u32>& theirs = objs[was].nodes;
      theirs.erase(std::remove(theirs.begin(), theirs.end(), n), theirs.end());
      nodeObj[n] = obj;
      body.nodes.push_back(n);
    }
  }
  const auto touches = [&](u32 bi) { return gear[beams[bi].a] || gear[beams[bi].b]; };
  // Beams may have gone off with a piece: they all come back to the body.
  for (size_t i = 0; i < objs.size(); i++) {
    if (i == obj || objs[i].root != body.root) continue;
    std::vector<u32>& theirs = objs[i].beamList;
    theirs.erase(std::remove_if(theirs.begin(), theirs.end(), touches), theirs.end());
    if (objs[i].nodes.empty()) {
      theirs.clear();
      objs[i].awake = objs[i].alwaysAwake = objs[i].dirty = false;
      active.erase(std::remove(active.begin(), active.end(), u32(i)), active.end());
    } else {
      computeBounds(objs[i]);
    }
  }
  std::vector<char> listed(beams.size(), 0);
  for (u32 bi : body.beamList) listed[bi] = 1;
  for (u32 bi = body.beamStart; bi < body.beamStart + body.beamCount; bi++) {
    if (!touches(bi)) continue;
    beams[bi] = start.beams[bi];
    if (!listed[bi]) body.beamList.push_back(bi);
  }
  triObjDirty = true;
  computeBounds(body);
  wake(obj);
}

static constexpr double kBlastEnd = 450.0;

void Sim::explode(V3 c, double impulse, double radius, double hole) {
  blasts.push_back({c, impulse, radius, 0.0});
  blastCount++;
  lastBlast = c;
  lastBlastRadius = radius;
  lastBlastImpulse = impulse;
  lastBlastDir = blastDir;
  blastDir = V3{};
  if (logBlasts) blastLog.push_back({c, impulse, radius, hole});
  if (craterWorld) {
    // Only what goes off close to the ground digs into it.
    const double above = c.z - craterWorld->height(c.x, c.y);
    // The heavier the charge, the bigger the hole, beyond what its reach alone would say.
    const double heavy = hole * std::max(1.0, std::sqrt(impulse / 2200.0));
    if (above < 3.0) craterWorld->addCrater(c.x, c.y, radius * 0.18 * heavy, radius * 0.05 * heavy * (1.0 - std::max(above, 0.0) / 3.0));
    // Boulders crack apart.
    // On a player's machine the server has already sent the dust.
    for (const auto& hit : craterWorld->breakRocks(c, radius * 0.42))
      if (!mirror) event(EV_ROCK, hit.first, V3{}, float(hit.second));
    // A bridge comes down, and whatever was standing on it does not stay up there asleep.
    for (const V3& fell : craterWorld->hitBridges(c, impulse, radius)) {
      if (mirror) continue;
      for (int k = -2; k <= 2; k++) event(EV_ROCK, fell + V3{k * 3.0, k * -2.0, -1.0}, V3{}, 3.f);
      for (size_t oi = 0; oi < objs.size(); oi++) {
        const Obj& o = objs[oi];
        if (o.bbMin.x < fell.x + 22.0 && o.bbMax.x > fell.x - 22.0 && o.bbMin.y < fell.y + 22.0 && o.bbMax.y > fell.y - 22.0 && o.bbMax.z > fell.z - 3.0 && o.bbMin.z < fell.z + 6.0) wake(u16(oi));
      }
    }
  }
  if (mirror) {
    blasts.clear();
    event(EV_BLAST, c, V3{}, float(radius));
    return;
  }
  // Stones of a wall right beside it are simply pulverised: their nodes leave the world, deep under the ground,
  // and what shows is the dust and rubble. Further out the blast only throws them.
  {
    const double crush = radius * 0.30;
    for (Block& bl : blocks) {
      V3 mid{};
      for (u32 n : bl.n) mid += pos[n];
      mid = mid * 0.125;
      const double d = len(mid - c);
      if (d > crush) continue;
      if (double(blastRng() & 0xffff) / 65535.0 > 1.25 * (1.0 - d / crush) + 0.15) continue;
      if (flags[bl.n[0]] & NF_FIXED) continue;
      Obj& owner = objs[nodeObj[bl.n[0]]];
      for (u32 bi : owner.beamList) {
        Beam& b = beams[bi];
        if (b.flags & BF_BROKEN) continue;
        for (u32 n : bl.n)
          if (b.a == n || b.b == n) {
            breakBeam(b, false, 0);
            break;
          }
      }
      event(EV_ROCK, mid, V3{}, 0.25f);
      for (u32 n : bl.n) {
        pos[n].z -= 400.0;
        vel[n] = {};
        flags[n] = u8((flags[n] | NF_FIXED) & ~NF_COLLIDE);
      }
    }
  }
  event(EV_BLAST, c, V3{}, float(radius));
  const double r2 = radius * radius;
  // Everything in reach wakes up, and the structure right at the blast is simply torn.
  const double core = radius * 0.32;
  auto chance = [&] { return double(blastRng() & 0xffff) / 65535.0; };
  for (size_t oi = 0; oi < objs.size(); oi++) {
    Obj& o = objs[oi];
    bool near = false;
    for (u32 n : o.nodes)
      if (len2(pos[n] - c) < r2) {
        near = true;
        break;
      }
    if (!near) continue;
    wake(u16(oi));
    if (o.armoured) continue;
    for (u32 bi : o.beamList) {
      Beam& b = beams[bi];
      if (b.flags & BF_BROKEN) continue;
      // What was built never to give way, such as the inside of a stone block, stays whole.
      if (b.yT <= 0 && b.fBreak <= 0) continue;
      const double d = len((pos[b.a] + pos[b.b]) * 0.5 - c);
      if (d > core * 2.2) continue;
      // Shock-loaded steel gives way far earlier than under a slow load.
      b.yC *= 0.55;
      b.yT *= 0.55;
      if (b.fBreak > 0) b.fBreak *= 0.5;
      if (d < core && chance() < 0.55 * (1.0 - d / core) + 0.08) breakBeam(b, false, 0);
    }
  }
}

void Sim::applyBlasts(double dt) {
  if (blasts.empty()) return;
  constexpr double kFrontSpeed = 450.0;   // m/s, an average over the fast near field and sound far out
  constexpr double kPush = 0.006;         // seconds the wave takes to pass a node
  constexpr double kNodeArea = 0.05;      // m^2 of surface a node stands for
  for (Blast& b : blasts) {
    b.age += dt;
    const double r2 = b.radius * b.radius;
    for (u32 oi : active) {
      for (u32 n : objs[oi].nodes) {
        const V3 r = pos[n] - b.c;
        const double d2 = len2(r);
        if (d2 > r2) continue;
        const double d = std::sqrt(d2);
        const double since = b.age - d / kFrontSpeed;
        if (since < 0 || since > kPush) continue;
        // The ground throws the wave back up, which is why things fly upwards.
        const double dist = std::max(d, 0.45);
        const V3 dir = norm(r + V3{0, 0, 0.45 * dist});
        const double fade = std::max(1.0 - d / b.radius, 0.0);
        // A light panel can only be thrown so fast: air drag eats anything beyond that within metres.
        constexpr double kThrowLimit = 26.0;
        const double J = std::min(b.impulse * kNodeArea * fade / dist, mass[n] * kThrowLimit * (0.35 + 0.65 * fade));
        force[n] += dir * (J / kPush);
      }
    }
  }
  blasts.erase(std::remove_if(blasts.begin(), blasts.end(), [](const Blast& b) { return b.age > b.radius / kBlastEnd + 0.01; }), blasts.end());
}

void Sim::breakBeam(Beam& b, bool notify, double f) {
  b.flags |= BF_BROKEN;
  brokenBeams++;
  objs[nodeObj[b.a]].dirty = true;
  if (b.tag == kTagWood) {
    // A crack runs: the fibres next to a failed one suddenly carry its load and go too, which is why
    // wood snaps across the whole trunk instead of giving way strand by strand.
    for (u32 bi : objs[nodeObj[b.a]].beamList) {
      Beam& o = beams[bi];
      if (o.tag != kTagWood || (o.flags & BF_BROKEN)) continue;
      // Only across the trunk at this height, not along the grain.
      if (std::fabs(rest[o.a].z + rest[o.b].z - rest[b.a].z - rest[b.b].z) > 0.5) continue;
      o.fBreak *= 0.5;
    }
    event(EV_WOOD, pos[b.a], vel[b.a], float(f));
  }
  else if (b.tag == kTagBranch) event(EV_BRANCH, pos[b.a], vel[b.a], float(f));
  else if (notify) event(EV_CRUNCH, pos[b.a], vel[b.a], float(f));
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
  const u16 nodeO = nodeObj[node];
  if (nodeO != t.obj) {
    if (t.flags & TF_HARD) return false;  // others only touch the outer skin
    // Two pieces of the same object were neighbours until a moment ago. Where they parted they still
    // overlap, and that must not fire them apart. Everywhere else they are simply two objects.
    if (objs[nodeO].root == objs[t.obj].root) {
      const V3 c = (rest[t.a] + rest[t.b] + rest[t.c]) * (1.0 / 3.0);
      if (len2(rest[node] - c) < 0.30 * 0.30) return false;
    }
    return true;
  }
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
    for (size_t k = 0; k < o.nodes.size(); k += 7) vmax = std::max(vmax, len2(vel[o.nodes[k]]));
  }
  vmax = std::sqrt(vmax);
  // Capped: one fragment thrown off at speed must not inflate the search for everything else.
  pairMargin = std::min(0.05 + 2.2 * vmax * dt * kPairInterval, 0.45);

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

  listPrimitives();
  const u32 nTris = u32(tris.size());
  for (u32 oi : active) {
    for (u32 i : objTris[oi]) {
      const Tri& t = tris[i];
      const V3& a = pos[t.a];
      const V3& b = pos[t.b];
      const V3& c = pos[t.c];
      // A triangle whose corners have been torn apart is no surface any more. It must not even reach
      // the grid: spanning tens of metres, it would land in tens of thousands of cells at every rebuild.
      constexpr double kTear2 = 1.5 * 1.5;
      if (len2(b - a) > kTear2 * len2(rest[t.b] - rest[t.a]) || len2(c - b) > kTear2 * len2(rest[t.c] - rest[t.b]) ||
          len2(a - c) > kTear2 * len2(rest[t.a] - rest[t.c]))
        continue;
      insertBox(i,
                {std::min({a.x, b.x, c.x}), std::min({a.y, b.y, c.y}), std::min({a.z, b.z, c.z})},
                {std::max({a.x, b.x, c.x}), std::max({a.y, b.y, c.y}), std::max({a.z, b.z, c.z})});
    }
    for (u32 i : objBlocks[oi]) {
      const Block& bl = blocks[i];
      V3 mn = pos[bl.n[0]], mx = mn;
      for (int k = 1; k < 8; k++) {
        const V3& p = pos[bl.n[k]];
        mn = {std::min(mn.x, p.x), std::min(mn.y, p.y), std::min(mn.z, p.z)};
        mx = {std::max(mx.x, p.x), std::max(mx.y, p.y), std::max(mx.z, p.z)};
      }
      insertBox(nTris + i, mn, mx);
    }
  }

  for (u32 oi : active) {
    const Obj& o = objs[oi];
    for (u32 i : o.nodes) {
      if (!(flags[i] & NF_COLLIDE)) continue;
      const V3& p = pos[i];
      const u32 h = cellHash(cellOf(p.x), cellOf(p.y), cellOf(p.z));
      for (int it = hashHead[h]; it >= 0; it = hashEntries[it].next) {
        const u32 prim = hashEntries[it].prim;
        if (stamp[prim] == i) continue;  // a primitive spanning hash-colliding cells
        if (stamp[prim] == 0xffffffffu) stamped.push_back(prim);
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
  for (u32 prim : stamped) stamp[prim] = 0xffffffffu;
  stamped.clear();
}

void Sim::listPrimitives() {
  // Objects are split and primitives added rarely; the lists hold until then.
  if (primsListed == tris.size() + blocks.size() + objs.size() && primsSplit == splitGeneration) return;
  primsListed = tris.size() + blocks.size() + objs.size();
  primsSplit = splitGeneration;
  objTris.assign(objs.size(), {});
  objBlocks.assign(objs.size(), {});
  for (u32 i = 0; i < u32(tris.size()); i++)
    if (!(tris[i].flags & TF_VISUAL)) objTris[tris[i].obj].push_back(i);
  for (u32 i = 0; i < u32(blocks.size()); i++) objBlocks[blocks[i].obj].push_back(i);
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

  // Most block pairs do not touch. Corners sit on the block's diagonal sphere, so a cheap check
  // against a centre computed once per step throws them out before any further work.
  if (blockCentre.size() != blocks.size()) {
    blockCentre.resize(blocks.size());
    blockRad2.resize(blocks.size());
    blockStep.assign(blocks.size(), 0);
  }
  for (u32 pi = 0; pi < blockPairs.size(); pi++) {
    const Pair& pr = blockPairs[pi];
    const Block& b = blocks[pr.prim];
    const V3& p = pos[pr.node];
    if (blockStep[pr.prim] != stepCount) {
      V3 cc{};
      for (int i = 0; i < 8; i++) cc += pos[b.n[i]];
      blockCentre[pr.prim] = cc * 0.125;
      blockRad2[pr.prim] = len2(pos[b.n[7]] - pos[b.n[0]]) * 0.25;
      blockStep[pr.prim] = stepCount;
    }
    if (len2(p - blockCentre[pr.prim]) > blockRad2[pr.prim]) continue;
    V3 cs[8];
    const V3 c = blockCentre[pr.prim];
    for (int i = 0; i < 8; i++) cs[i] = pos[b.n[i]];

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

    // Scratches, purely visual. Whichever side of the contact is the car gets marked.
    if (((stepCount + pr.node) & 7) == 0 && scrapes.size() < 400) {
      const bool nodeCar = objs[nodeObj[pr.node]].root == 0, triCar = objs[t.obj].root == 0;
      if (nodeCar != triCar) {
        const V3 slide = vrel - h.n * vn;
        const double sp = len(slide);
        if (sp > 1.0 && fn > 150.0) {
          const V3 at = nodeCar ? rest[pr.node] : rest[t.a] * h.w0 + rest[t.b] * h.w1 + rest[t.c] * h.w2;
          scrapes.push_back({at, slide / sp, float(clampd(fn / 4000.0, 0.15, 1.0))});
        }
      }
    }
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

// Share of the contact frequency for rim nodes: 0.65 means 42 % of the normal stiffness, about 1.3 MN/m.
static constexpr double kRimCushion = 0.65;

void Sim::collideWorld(double dt) {
  for (u32 oi : active) {
    const Obj& o = objs[oi];
    for (u32 i : o.nodes) {
      const u8 fl = flags[i];
      if ((fl & NF_FIXED) || !(fl & NF_COLLIDE)) continue;
      Contact c;
      if (!world->contact(pos[i], c)) continue;
      const double m = mass[i];
      const double vn = dot(vel[i], c.n);
      // A rim never meets a kerb bare: the crushed tyre is still between them. Without that cushion
      // the hub takes a hammer blow, and the slightest kerb folds the wheel over.
      const double omega = (fl & NF_WHEEL) ? kContactOmega * kRimCushion : kContactOmega;
      // Armour hitting a wall loses its energy in the blow instead of springing back off it.
      const double zeta = o.armoured ? 0.9 : kContactZeta;
      const double fn = m * omega * omega * std::min(c.depth, kMaxPenWorld) - 2.0 * zeta * m * omega * vn;
      if (fn <= 0) continue;
      V3 ff;
      const double muv = double(mu[i]) * surfaceFriction(c.surface);
      contactFriction(i, c.n, fn, muv, dt, vel[i], m, &ff);
      force[i] += c.n * fn + ff;

      const double vt2 = len2(vel[i]) - vn * vn;
      if (((stepCount + i) & 7) == 0 && !(fl & NF_WHEEL) && vt2 > 1.0 && fn > 150.0 && objs[oi].root == 0 && scrapes.size() < 400)
        scrapes.push_back({rest[i], norm(vel[i] - c.n * vn), float(clampd(fn / 4000.0, 0.15, 1.0))});
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
    for (u32 i : o.nodes) {
      V3 dv = force[i] * (invMass[i] * dt);
      // Last line of defence: no legitimate force changes a node's speed this much in 0.5 ms. It
      // keeps one numerically bad contact from running away and taking the whole car with it.
      const double dv2 = len2(dv);
      if (dv2 > kMaxDv * kMaxDv) {
        dv = dv * (kMaxDv / std::sqrt(dv2));
        clampedSteps++;
      }
      V3 v = vel[i] + dv;
      const double s2 = len2(v);
      if (s2 > kMaxSpeed * kMaxSpeed) v = v * (kMaxSpeed / std::sqrt(s2));
      vel[i] = v;
      pos[i] += v * dt;
    }
  }
}

// Finds out whether a damaged object still hangs together. If it has come apart, every separate piece
// becomes an object of its own, with its own sleep state and full collision against the others.
void Sim::splitObjects() {
  std::vector<int> parent;
  auto find = [&](int x) {
    while (parent[size_t(x)] != x) {
      parent[size_t(x)] = parent[size_t(parent[size_t(x)])];
      x = parent[size_t(x)];
    }
    return x;
  };
  const size_t nActive = active.size();
  for (size_t ai = 0; ai < nActive; ai++) {
    const u32 oid = active[ai];
    if (!objs[oid].dirty) continue;
    objs[oid].dirty = false;

    // Node ids are global: a scratch map from node id to its slot in this object.
    if (nodeSlot.size() != pos.size()) nodeSlot.assign(pos.size(), -1);
    const size_t n = objs[oid].nodes.size();
    parent.resize(n);
    for (size_t i = 0; i < n; i++) {
      parent[i] = int(i);
      nodeSlot[objs[oid].nodes[i]] = int(i);
    }
    for (u32 bi : objs[oid].beamList) {
      const Beam& b = beams[bi];
      if (b.flags & BF_BROKEN) continue;
      // The ground does not hold things together: two posts anchored to it are separate objects.
      if ((flags[b.a] | flags[b.b]) & NF_FIXED) continue;
      const int ia = nodeSlot[b.a], ib = nodeSlot[b.b];
      if (ia < 0 || ib < 0) continue;
      const int ra = find(ia), rb = find(ib);
      if (ra != rb) parent[size_t(ra)] = rb;
    }

    // Collect the pieces. Anchored nodes join the piece they still have a beam to.
    std::vector<std::vector<u32>> pieces;
    std::vector<int> pieceOf(n, -1);
    std::vector<int> rootPiece(n, -1);
    for (size_t i = 0; i < n; i++) {
      if (flags[objs[oid].nodes[i]] & NF_FIXED) continue;
      const int r = find(int(i));
      if (rootPiece[size_t(r)] < 0) {
        rootPiece[size_t(r)] = int(pieces.size());
        pieces.emplace_back();
      }
      pieces[size_t(rootPiece[size_t(r)])].push_back(objs[oid].nodes[i]);
      pieceOf[i] = rootPiece[size_t(r)];
    }
    for (u32 bi : objs[oid].beamList) {
      const Beam& b = beams[bi];
      if (b.flags & BF_BROKEN) continue;
      for (int e = 0; e < 2; e++) {
        const u32 fixedN = e ? b.b : b.a, otherN = e ? b.a : b.b;
        if (!(flags[fixedN] & NF_FIXED) || (flags[otherN] & NF_FIXED)) continue;
        const int si = nodeSlot[fixedN], oi = nodeSlot[otherN];
        if (si >= 0 && oi >= 0 && pieceOf[size_t(si)] < 0 && pieceOf[size_t(oi)] >= 0) {
          pieceOf[size_t(si)] = pieceOf[size_t(oi)];
          pieces[size_t(pieceOf[size_t(oi)])].push_back(fixedN);
        }
      }
    }
    // Anchors with nothing left simply stay behind in the main piece.
    for (u32 v : objs[oid].nodes) nodeSlot[v] = -1;
    if (pieces.size() < 2) continue;

    // The heaviest piece keeps the identity of the object: for the car, that is the car.
    size_t keep = 0;
    double best = -1;
    for (size_t k = 0; k < pieces.size(); k++) {
      double m = 0;
      for (u32 v : pieces[k]) m += mass[v];
      if (m > best) {
        best = m;
        keep = k;
      }
    }
    std::vector<u32> leftovers;
    for (size_t i = 0; i < n; i++)
      if (pieceOf[i] < 0) leftovers.push_back(objs[oid].nodes[i]);

    std::vector<u16> pieceObj(pieces.size());
    for (size_t k = 0; k < pieces.size(); k++) {
      if (k == keep) {
        pieceObj[k] = u16(oid);
        continue;
      }
      Obj no;
      no.kind = OBJ_PROP;
      no.awake = true;
      no.root = objs[oid].root;
      no.nodes = pieces[k];
      objs.push_back(std::move(no));
      pieceObj[k] = u16(objs.size() - 1);
      active.push_back(u32(objs.size() - 1));
      for (u32 v : pieces[k]) nodeObj[v] = pieceObj[k];
    }
    // Beams go with their nodes; broken ones are of no more use to anyone.
    std::vector<u32> keptBeams;
    const std::vector<u32> oldBeams = std::move(objs[oid].beamList);
    for (u32 bi : oldBeams) {
      const Beam& b = beams[bi];
      if (b.flags & BF_BROKEN) continue;
      u16 target = nodeObj[b.a];
      if (flags[b.a] & NF_FIXED) target = nodeObj[b.b];
      if (target == oid)
        keptBeams.push_back(bi);
      else
        objs[target].beamList.push_back(bi);
    }
    objs[oid].beamList = std::move(keptBeams);
    objs[oid].nodes = pieces[keep];
    objs[oid].nodes.insert(objs[oid].nodes.end(), leftovers.begin(), leftovers.end());
    for (size_t k = 0; k < pieces.size(); k++) computeBounds(objs[pieceObj[k]]);
    triObjDirty = true;
    splitGeneration++;
  }

  if (triObjDirty) {
    triObjDirty = false;
    for (Tri& t : tris) {
      const u16 oa = nodeObj[t.a];
      t.obj = (flags[t.a] & NF_FIXED) ? nodeObj[t.b] : oa;
    }
    for (Block& b : blocks) b.obj = nodeObj[b.n[0]];
    stamp.assign(tris.size() + blocks.size(), 0xffffffffu);
  }
}

void Sim::sleepWake() {
  splitObjects();

  // Under load whatever has more or less come to rest counts as resting, and sooner.
  const double kSleepSpeed2 = 0.20 * 0.20 * sleepHaste * sleepHaste;
  const double kPeriod = 64.0 / 2000.0 * sleepHaste;

  // Wake sleepers touched by anything that is actually moving. A resting prop next to another
  // resting prop must not keep waking its neighbour, or everything ends up awake forever.
  const size_t nActive = active.size();
  for (size_t ai = 0; ai < nActive; ai++) {
    Obj& o = objs[active[ai]];
    computeBounds(o);
    double vmax2 = 0;
    for (u32 i : o.nodes) vmax2 = std::max(vmax2, len2(vel[i]));

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
        for (u32 i : o.nodes) {
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
    // Debris lying about far from the car is not worth simulating unless something big is happening.
    const Obj& car = objs[0];
    const bool farAway = o.bbMin.x > car.bbMax.x + 25.0 || o.bbMax.x < car.bbMin.x - 25.0 || o.bbMin.y > car.bbMax.y + 25.0 || o.bbMax.y < car.bbMin.y - 25.0;
    if (farAway && vmax2 < 2.0 * 2.0) o.sleepTimer += float(kPeriod) * 3.0f;
  }

  // Put settled props to sleep (done separately because wake() above may grow the active list).
  for (size_t ai = 0; ai < active.size();) {
    Obj& o = objs[active[ai]];
    const bool partnerReady = o.partner < 0 || !objs[size_t(o.partner)].awake || objs[size_t(o.partner)].sleepTimer > 1.0f;
    if (!o.alwaysAwake && o.sleepTimer > 1.0f && partnerReady) {
      for (u32 i : o.nodes) vel[i] = {};
      o.awake = false;
      active[ai] = active.back();
      active.pop_back();
    } else {
      ai++;
    }
  }
}
