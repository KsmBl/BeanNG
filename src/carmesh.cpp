#include "asset.h"
#include "carmesh.h"
#include <algorithm>
#include <cstdio>
#include <cstring>

namespace {
// Design x runs rearwards from the nose; the world car at yaw 0 faces +X.
inline V3 designToWorld(double x, double y, double z, V3 spawn, double nose) { return spawn + V3{nose - x, y, z}; }

// Solves the symmetric 4x4 system A s = e0 by Gaussian elimination with partial pivoting.
bool solve4(double A[4][4], double s[4]) {
  double b[4] = {1, 0, 0, 0};
  for (int i = 0; i < 4; i++) {
    int piv = i;
    for (int r = i + 1; r < 4; r++)
      if (std::fabs(A[r][i]) > std::fabs(A[piv][i])) piv = r;
    if (std::fabs(A[piv][i]) < 1e-14) return false;
    if (piv != i) {
      for (int c = 0; c < 4; c++) std::swap(A[i][c], A[piv][c]);
      std::swap(b[i], b[piv]);
    }
    for (int r = i + 1; r < 4; r++) {
      const double f = A[r][i] / A[i][i];
      for (int c = i; c < 4; c++) A[r][c] -= f * A[i][c];
      b[r] -= f * b[i];
    }
  }
  for (int i = 3; i >= 0; i--) {
    double v = b[i];
    for (int c = i + 1; c < 4; c++) v -= A[i][c] * s[c];
    s[i] = v / A[i][i];
  }
  return true;
}
}  // namespace

bool CarMesh::load(const std::string& path) {
  AssetFile f(path.c_str());
  if (!f) return false;
  char magic[4];
  u32 count = 0;
  if (f.read(magic, 1, 4) != 4 || memcmp(magic, "XMSH", 4) != 0 || f.read(&count, 4, 1) != 1) {
    return false;
  }
  parts.clear();
  restPos.clear();
  restNormal.clear();
  tris.clear();
  for (u32 i = 0; i < count; i++) {
    char name[41] = {}, group[25] = {};
    u32 hdr[3];
    if (f.read(name, 1, 40) != 40 || f.read(group, 1, 24) != 24 || f.read(hdr, 4, 3) != 3) {
      return false;
    }
    MeshPart p;
    p.name = name;
    p.group = group;
    p.mat = u8(hdr[0]);
    p.firstVert = u32(restPos.size() / 3);
    p.vertCount = hdr[1];
    p.firstTri = u32(tris.size() / 3);
    p.triCount = hdr[2];
    const size_t vOld = restPos.size(), tOld = tris.size();
    restPos.resize(vOld + size_t(hdr[1]) * 3);
    restNormal.resize(vOld + size_t(hdr[1]) * 3);
    tris.resize(tOld + size_t(hdr[2]) * 3);
    bool ok = f.read(&restPos[vOld], 12, hdr[1]) == hdr[1];
    ok = ok && f.read(&restNormal[vOld], 12, hdr[1]) == hdr[1];
    ok = ok && f.read(&tris[tOld], 12, hdr[2]) == hdr[2];
    if (!ok) {
      return false;
    }
    for (size_t k = tOld; k < tris.size(); k++) tris[k] += p.firstVert;
    parts.push_back(std::move(p));
  }

  restEdge2.resize(tris.size());
  auto vpos = [&](u32 v) { return V3{restPos[v * 3], restPos[v * 3 + 1], restPos[v * 3 + 2]}; };
  for (size_t t = 0; t < tris.size() / 3; t++) {
    const V3 a = vpos(tris[t * 3]), b = vpos(tris[t * 3 + 1]), c = vpos(tris[t * 3 + 2]);
    restEdge2[t * 3] = float(len2(b - a));
    restEdge2[t * 3 + 1] = float(len2(c - b));
    restEdge2[t * 3 + 2] = float(len2(a - c));
  }
  return true;
}

void CarMesh::addParked(const CarMesh& o, int k, V3 oSpawn, double oNose, V3 mySpawn, const std::vector<ParkedHub>& hubs) {
  // Into my own design frame, which runs backwards from my nose, wherever in the world that lands it.
  const auto toMine = [&](V3 w, float* out) {
    out[0] = float(noseX - (w.x - mySpawn.x));
    out[1] = float(w.y - mySpawn.y);
    out[2] = float(w.z - mySpawn.z);
  };
  const auto append = [&](const MeshPart& p, const std::string& group, const std::string& suffix, auto mapPos, auto mapNormal, bool wheel) {
    MeshPart q;
    q.name = "p" + std::to_string(k) + "_" + p.name + suffix;
    q.group = std::to_string(k) + ":" + group;
    q.mat = p.mat;
    q.instance = k;
    q.firstVert = u32(restPos.size() / 3);
    q.vertCount = p.vertCount;
    q.firstTri = u32(tris.size() / 3);
    q.triCount = p.triCount;
    for (u32 v = p.firstVert; v < p.firstVert + p.vertCount; v++) {
      const V3 d{o.restPos[v * 3], o.restPos[v * 3 + 1], o.restPos[v * 3 + 2]};
      const V3 n{o.restNormal[v * 3], o.restNormal[v * 3 + 1], o.restNormal[v * 3 + 2]};
      float out[3];
      toMine(mapPos(d), out);
      restPos.insert(restPos.end(), out, out + 3);
      const V3 nw = mapNormal(n);
      restNormal.push_back(float(-nw.x));
      restNormal.push_back(float(nw.y));
      restNormal.push_back(float(nw.z));
    }
    for (u32 t = p.firstTri; t < p.firstTri + p.triCount; t++) {
      u32 a = o.tris[t * 3] - p.firstVert + q.firstVert, b = o.tris[t * 3 + 1] - p.firstVert + q.firstVert, c = o.tris[t * 3 + 2] - p.firstVert + q.firstVert;
      if (wheel) {
        // The renderer takes the normal from the winding, seen in the mirrored design frame. Put it right.
        const auto P = [&](u32 i) { return V3{restPos[i * 3], restPos[i * 3 + 1], restPos[i * 3 + 2]}; };
        const V3 cr = cross(P(b) - P(a), P(c) - P(a));
        const V3 nA{restNormal[a * 3], restNormal[a * 3 + 1], restNormal[a * 3 + 2]};
        if (dot(cr, nA) < 0) std::swap(b, c);
      }
      tris.push_back(a);
      tris.push_back(b);
      tris.push_back(c);
      const auto Q = [&](u32 i) { return V3{restPos[i * 3], restPos[i * 3 + 1], restPos[i * 3 + 2]}; };
      restEdge2.push_back(float(len2(Q(b) - Q(a))));
      restEdge2.push_back(float(len2(Q(c) - Q(b))));
      restEdge2.push_back(float(len2(Q(a) - Q(c))));
    }
    parts.push_back(q);
  };
  for (const MeshPart& p : o.parts) {
    if (p.group.rfind("wheel", 0) != 0) {
      append(p, p.group, "", [&](V3 d) { return oSpawn + V3{oNose - d.x, d.y, d.z}; }, [](V3 n) { return V3{-n.x, n.y, n.z}; }, false);
      continue;
    }
    for (size_t h = 0; h < hubs.size(); h++) {
      const ParkedHub& hub = hubs[h];
      const double side = hub.left ? 1.0 : -1.0;
      // The wheel's own frame: x up, y out of its face, z along the car.
      append(p, hub.group, "_" + std::to_string(h), [&, side](V3 d) { return hub.at + V3{-side * d.z, side * d.y * hub.width, d.x}; }, [side](V3 n) { return V3{-side * n.z, side * n.y, n.x}; }, true);
    }
  }
}

void CarMesh::bind(const std::vector<V3>& rest, const std::vector<NodeGroup>& groups, V3 spawn) {
  const size_t nv = restPos.size() / 3;
  bindNode.assign(nv * kK, 0);
  bindWeight.assign(nv * kK, 0.f);

  struct Cand {
    double d2;
    u32 n;
  };
  std::vector<Cand> cand;
  for (MeshPart& part : parts) {
    const NodeGroup* g = nullptr;
    for (const NodeGroup& ng : groups)
      if (ng.name == part.group) g = &ng;
    part.bound = g != nullptr && !g->nodes.empty();
    if (!part.bound) {
      if (part.group.rfind("wheel", 0) != 0) fprintf(stderr, "mesh part %s: no node group '%s'\n", part.name.c_str(), part.group.c_str());
      continue;
    }
    const size_t colon = part.group.find(':');
    const std::string plain = colon == std::string::npos ? part.group : part.group.substr(colon + 1);
    double partH = g->nodes.size() <= kK ? 50.0 : (plain == "frame" ? 0.9 : (plain == "hull" ? 3.0 : 0.45));
    for (int pass = 0; pass < 2; pass++)
    for (u32 v = part.firstVert; v < part.firstVert + part.vertCount; v++) {
      const V3 p = designToWorld(restPos[v * 3], restPos[v * 3 + 1], restPos[v * 3 + 2], spawn, noseX);
      cand.clear();
      for (u32 n : g->nodes) cand.push_back({len2(rest[n] - p), n});
      std::sort(cand.begin(), cand.end(), [](const Cand& a, const Cand& b) { return a.d2 < b.d2; });

      // Fits weights to a node set with a fixed support radius; returns the error at rest.
      auto fit = [&](const std::vector<Cand>& set, double h2, double* phiOut) {
        const int k = int(set.size());
        double wgt[kK];
        double A[4][4] = {};
        double basis[kK][4];
        for (int i = 0; i < k; i++) {
          const double q = std::max(1.0 - set[i].d2 / h2, 0.0);
          wgt[i] = q * q * q + 1e-6;
          const V3 d = rest[set[i].n] - p;
          basis[i][0] = 1;
          basis[i][1] = d.x;
          basis[i][2] = d.y;
          basis[i][3] = d.z;
          for (int r = 0; r < 4; r++)
            for (int c = 0; c < 4; c++) A[r][c] += wgt[i] * basis[i][r] * basis[i][c];
        }
        for (int r = 1; r < 4; r++) A[r][r] += 1e-9;
        double sol[4] = {1, 0, 0, 0};
        if (!solve4(A, sol)) sol[1] = sol[2] = sol[3] = 0;
        double sum = 0;
        for (int i = 0; i < k; i++) {
          phiOut[i] = wgt[i] * (basis[i][0] * sol[0] + basis[i][1] * sol[1] + basis[i][2] * sol[2] + basis[i][3] * sol[3]);
          sum += phiOut[i];
        }
        V3 q{};
        for (int i = 0; i < k; i++) {
          if (std::fabs(sum) > 1e-9) phiOut[i] /= sum;
          q += rest[set[i].n] * phiOut[i];
        }
        return len(q - p);
      };

      // Support radius: one value for the whole part, because a radius that changed from vertex to
      // vertex would itself be a discontinuity. First find the smallest that works everywhere.
      auto attemptAt = [&](double h, std::vector<Cand>& set, double* phi) {
        const double h2 = h * h;
        set.clear();
        for (const Cand& c : cand) {
          if (c.d2 >= h2 || set.size() >= size_t(kK)) break;
          set.push_back(c);
        }
        if (set.size() < 6) return 1e9;
        return fit(set, h2, phi);
      };
      std::vector<Cand> set;
      double phi[kK] = {};
      if (pass == 0) {
        while (partH < 40.0 && attemptAt(partH, set, phi) > 1e-3) partH *= 1.15;
      } else {
        attemptAt(partH, set, phi);
        for (size_t i = 0; i < set.size(); i++) {
          bindNode[size_t(v) * kK + i] = set[i].n;
          bindWeight[size_t(v) * kK + i] = float(phi[i]);
        }
      }
    }
  }
}

size_t CarMesh::rebindAfterTear(const std::vector<V3>& rest, const std::vector<NodeGroup>& groups, V3 spawn, const std::vector<u16>& nodeObj) {
  size_t changed = 0;
  std::vector<std::pair<u16, double>> share;
  struct Cand {
    double d2;
    u32 n;
  };
  std::vector<Cand> cand;
  for (const MeshPart& part : parts) {
    if (!part.bound) continue;
    const NodeGroup* g = nullptr;
    for (const NodeGroup& ng : groups)
      if (ng.name == part.group) g = &ng;
    if (!g) continue;

    // Pieces of this group big enough to carry geometry at all, with their node count.
    std::vector<std::pair<u16, int>> sizes;
    for (u32 n : g->nodes) {
      bool found = false;
      for (auto& sz : sizes)
        if (sz.first == nodeObj[n]) {
          sz.second++;
          found = true;
        }
      if (!found) sizes.push_back({nodeObj[n], 1});
    }

    for (u32 v = part.firstVert; v < part.firstVert + part.vertCount; v++) {
      share.clear();
      for (int i = 0; i < kK; i++) {
        const float w = bindWeight[size_t(v) * kK + i];
        if (w == 0.f) continue;
        const u16 o = nodeObj[bindNode[size_t(v) * kK + i]];
        bool found = false;
        for (auto& sh : share)
          if (sh.first == o) {
            sh.second += std::fabs(w);
            found = true;
          }
        if (!found) share.push_back({o, std::fabs(double(w))});
      }
      if (share.size() < 2) continue;

      // Follow the piece that dominates this vertex, as long as it can carry geometry.
      u16 owner = share[0].first;
      double best = -1;
      for (const auto& sh : share) {
        int count = 0;
        for (const auto& sz : sizes)
          if (sz.first == sh.first) count = sz.second;
        if (count >= 6 && sh.second > best) {
          best = sh.second;
          owner = sh.first;
        }
      }
      if (best < 0) continue;

      const V3 p = designToWorld(restPos[v * 3], restPos[v * 3 + 1], restPos[v * 3 + 2], spawn, noseX);
      cand.clear();
      for (u32 n : g->nodes)
        if (nodeObj[n] == owner) cand.push_back({len2(rest[n] - p), n});
      std::sort(cand.begin(), cand.end(), [](const Cand& a, const Cand& b) { return a.d2 < b.d2; });
      const int k = std::min<int>(kK, int(cand.size()));
      const double h2 = cand[size_t(k - 1)].d2 * 1.3 + 1e-3;
      double wgt[kK], basis[kK][4];
      double A[4][4] = {};
      for (int i = 0; i < k; i++) {
        const double q = std::max(1.0 - cand[size_t(i)].d2 / h2, 0.0);
        wgt[i] = q * q + 1e-3;
        const V3 d = rest[cand[size_t(i)].n] - p;
        basis[i][0] = 1;
        basis[i][1] = d.x;
        basis[i][2] = d.y;
        basis[i][3] = d.z;
        for (int r = 0; r < 4; r++)
          for (int c = 0; c < 4; c++) A[r][c] += wgt[i] * basis[i][r] * basis[i][c];
      }
      for (int r = 1; r < 4; r++) A[r][r] += 1e-7;
      double sol[4] = {1, 0, 0, 0};
      if (!solve4(A, sol)) sol[1] = sol[2] = sol[3] = 0;
      double sum = 0;
      double phi[kK] = {};
      for (int i = 0; i < k; i++) {
        phi[i] = wgt[i] * (basis[i][0] * sol[0] + basis[i][1] * sol[1] + basis[i][2] * sol[2] + basis[i][3] * sol[3]);
        sum += phi[i];
      }
      for (int i = 0; i < kK; i++) {
        bindNode[size_t(v) * kK + i] = i < k ? cand[size_t(i)].n : cand[0].n;
        bindWeight[size_t(v) * kK + i] = i < k ? float(phi[i] / sum) : 0.f;
      }
      changed++;
    }
  }
  return changed;
}

double CarMesh::restError(const std::vector<V3>& rest, V3 spawn, std::string* worst) const {
  double maxErr = 0;
  for (const MeshPart& part : parts) {
    if (!part.bound) continue;
    for (u32 v = part.firstVert; v < part.firstVert + part.vertCount; v++) {
      V3 q{};
      for (int i = 0; i < kK; i++) q += rest[bindNode[size_t(v) * kK + i]] * double(bindWeight[size_t(v) * kK + i]);
      const double e = len(q - designToWorld(restPos[v * 3], restPos[v * 3 + 1], restPos[v * 3 + 2], spawn, noseX));
      if (e > maxErr) {
        maxErr = e;
        if (worst) *worst = part.name;
      }
    }
  }
  return maxErr;
}
