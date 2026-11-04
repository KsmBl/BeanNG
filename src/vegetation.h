// Grass and bushes: instanced, a draw call each, and pressed flat by whatever drives over them.
#pragma once
#include <functional>
#include <unordered_map>
#include <vector>
#include "mathx.h"
#include "render.h"
#include "world.h"

class Vegetation {
 public:
  // groundColor gives the colour of the ground at a grid vertex, for the grass to grow out of it unseen.
  void init(const World& world, std::function<void(int, int, float&, float&, float&)> groundColor);
  // What has happened on the ground, and what is in view.
  void update(const Snapshot& s, V3 eye, V3 viewDir, double dt);
  void clearMarks();
  void setDensity(double d);  // of the grass, 0 to 1
  V3 nearestBush(V3 from, double beyond) const;  // for the test scripts
  void drawShadow(const float* lightVP);
  void draw(const float* vp, const float* prevVP, const float* lightVP, V3 eye, V3 sun);

  // For the record: what the last frame sent to the graphics card.
  int grassInstances() const { return grassCount_; }
  int bushInstances() const { return bushCount_; }
  static constexpr int kGrassTris = 14, kBushTris = 270;
  double lastRebuildMs() const { return rebuildMs_; }

 private:
#pragma pack(push, 1)
  struct Inst {
    float x, y, z, seed;
    u8 r, g, b, size;
  };
#pragma pack(pop)
  struct Cell {
    std::vector<Inst> grass;
  };
  static constexpr double kCell = 32.0, kGrassRange = 140.0, kBushRange = 430.0;
  static constexpr int kMapSize = 2048;  // a texel a metre
  const World* world_ = nullptr;
  std::function<void(int, int, float&, float&, float&)> groundColor_;
  std::unordered_map<u64, Cell> cells_;
  std::vector<Inst> bushes_;
  std::vector<std::pair<u32, u32>> bushCells_;  // ranges of bushes_, one for each cell that has any
  std::vector<V3> bushCellMid_;
  std::vector<Inst> scratch_;
  std::vector<u8> marks_;  // red: pressed flat, green: burnt away
  size_t cratersSeen_ = 0;
  V3 builtEye_{1e9, 0, 0}, builtDir_{1, 0, 0};
  double clock_ = 0, rebuildMs_ = 0, density_ = 1;
  int grassCount_ = 0, bushCount_ = 0;
  u32 progGrass_ = 0, progBush_ = 0, progBushShadow_ = 0;
  u32 grassVao_ = 0, grassMesh_ = 0, grassInst_ = 0, bushVao_ = 0, bushMesh_ = 0, bushInst_ = 0, marksTex_ = 0;
  int grassVerts_ = 0, bushVerts_ = 0;
  void buildMeshes();
  void placeBushes();
  const Cell& cellAt(int cx, int cy, int& budget, bool& missing);
  void stamp(V3 at, V3 fwd, V3 left, double halfL, double halfW);
  void rebuild(V3 eye, V3 viewDir);
};
