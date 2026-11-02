#include "carmesh.h"
#include <algorithm>
#include <cstdio>
#include <cstring>

namespace {
// Design x runs rearwards from the nose; the world car at yaw 0 faces +X.
inline V3 designToWorld(double x, double y, double z, V3 spawn) { return spawn + V3{2.26 - x, y, z}; }

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
  FILE* f = fopen(path.c_str(), "rb");
  if (!f) return false;
  char magic[4];
  u32 count = 0;
  if (fread(magic, 1, 4, f) != 4 || memcmp(magic, "XMSH", 4) != 0 || fread(&count, 4, 1, f) != 1) {
    fclose(f);
    return false;
  }
  parts.clear();
  restPos.clear();
  restNormal.clear();
  tris.clear();
  for (u32 i = 0; i < count; i++) {
    char name[41] = {}, group[25] = {};
    u32 hdr[3];
    if (fread(name, 1, 40, f) != 40 || fread(group, 1, 24, f) != 24 || fread(hdr, 4, 3, f) != 3) {
      fclose(f);
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
    bool ok = fread(&restPos[vOld], 12, hdr[1], f) == hdr[1];
    ok = ok && fread(&restNormal[vOld], 12, hdr[1], f) == hdr[1];
    ok = ok && fread(&tris[tOld], 12, hdr[2], f) == hdr[2];
    if (!ok) {
      fclose(f);
      return false;
    }
    for (size_t k = tOld; k < tris.size(); k++) tris[k] += p.firstVert;
    parts.push_back(std::move(p));
  }
  fclose(f);

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
    double partH = g->nodes.size() <= kK ? 50.0 : 0.45;
    for (int pass = 0; pass < 2; pass++)
    for (u32 v = part.firstVert; v < part.firstVert + part.vertCount; v++) {
      const V3 p = designToWorld(restPos[v * 3], restPos[v * 3 + 1], restPos[v * 3 + 2], spawn);
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

double CarMesh::restError(const std::vector<V3>& rest, V3 spawn, std::string* worst) const {
  double maxErr = 0;
  for (const MeshPart& part : parts) {
    if (!part.bound) continue;
    for (u32 v = part.firstVert; v < part.firstVert + part.vertCount; v++) {
      V3 q{};
      for (int i = 0; i < kK; i++) q += rest[bindNode[size_t(v) * kK + i]] * double(bindWeight[size_t(v) * kK + i]);
      const double e = len(q - designToWorld(restPos[v * 3], restPos[v * 3 + 1], restPos[v * 3 + 2], spawn));
      if (e > maxErr) {
        maxErr = e;
        if (worst) *worst = part.name;
      }
    }
  }
  return maxErr;
}
