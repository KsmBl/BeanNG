// Static world: heightfield terrain with roads, plus static oriented boxes.
#pragma once
#include <vector>
#include "mathx.h"

enum Surface : u8 { SURF_ASPHALT, SURF_GRASS, SURF_GRAVEL, SURF_CONCRETE, SURF_PAVE, SURF_ROCK, SURF_SNOW };

inline double surfaceFriction(u8 s) {
  switch (s) {
    case SURF_ASPHALT: return 1.0;
    case SURF_GRASS: return 0.55;
    case SURF_GRAVEL: return 0.65;
    case SURF_CONCRETE: return 0.95;
    case SURF_PAVE: return 0.85;
    case SURF_ROCK: return 0.80;
    case SURF_SNOW: return 0.40;
  }
  return 1.0;
}

struct Contact {
  double depth = 0;  // > 0 when penetrating
  V3 n{0, 0, 1};
  u8 surface = SURF_ASPHALT;
};

struct StaticBox {
  V3 c;
  V3 ax[3];  // orthonormal axes
  V3 h;      // half extents
  u8 surface = SURF_CONCRETE;
  float r = .6f, g = .6f, b = .6f;
};

struct RoadPoint {
  double x, y, z;
  double bank = 0;  // road height gained per metre to the left of the direction of travel
};

// Scales how much bends are banked. 1 in the game, 0 lets the test bench compare against a flat road.
inline double gBankScale = 1.0;
struct Road {
  std::vector<RoadPoint> pts;
  double halfWidth = 4;
  bool closed = false;
  bool trail = false;  // a dirt track: no tarmac and no road markings
};

class World {
 public:
  static constexpr int N = 2049;          // heightfield vertices per side
  static constexpr double kHalf = 1024.0;  // metres from centre to edge, 1 m grid

  void build();

  // Physics queries (hot path).
  double height(double x, double y) const;
  // The land beyond the detailed part of the map: the same hills carried on, for ever.
  double outerHeight(double x, double y) const;
  V3 terrainNormal(double x, double y) const;
  bool contact(V3 p, Contact& out) const;
  u8 surfaceAt(double x, double y) const;

  // A hole blown into the ground: the height field itself changes, so wheels drop into it. Called from
  // the physics thread, between steps; everyone else only ever reads single floats of it.
  struct Crater {
    double x, y, r;
  };
  void addCrater(double x, double y, double r, double depth);
  // Boulders in reach of a blast split into smaller ones, and the small ones are gone. Returns where it happened.
  std::vector<std::pair<V3, double>> breakRocks(V3 c, double reach);
  u32 boxGeneration() const { return boxGeneration_; }
  void clearCraters();
  const std::vector<Crater>& craters() const { return craters_; }

  void clearBoxes();  // for the test bench: an empty, obstacle-free pad
  void flatten();     // for the test bench: a level plane for top speed runs
  void pave();        // for the test bench: tarmac everywhere

  // Renderer access.
  double gridHeight(int ix, int iy) const { return h_[size_t(iy) * N + ix]; }
  u8 gridSurface(int ix, int iy) const { return surf_[size_t(iy) * N + ix]; }
  const std::vector<StaticBox>& boxes() const { return boxes_; }
  const std::vector<Road>& roads() const { return roads_; }
  const std::vector<V3>& trees() const { return trees_; }  // foot of each tree
  struct Place {
    const char* name;
    double x, y, yaw;
  };
  const std::vector<Place>& places() const { return places_; }  // where the driver can start from

 private:
  std::vector<float> h_;
  std::vector<u8> surf_;
  std::vector<StaticBox> boxes_;
  std::vector<Road> roads_;
  std::vector<V3> trees_;
  bool levelled_ = false;
  std::vector<Place> places_;
  std::vector<Crater> craters_;
  std::vector<StaticBox> boxes0_;
  std::vector<std::vector<u16>> boxGrid0_;
  u32 boxGeneration_ = 0;
  u32 chipSeed_ = 77;
  std::vector<float> h0_;
  std::vector<u8> surf0_;
  static constexpr double kBoxCell = 8.0;
  static constexpr int kBoxGrid = 256;
  std::vector<std::vector<u16>> boxGrid_;

 public:
  void addBox(V3 c, V3 half, double yaw, double pitch, u8 surf, float r, float g, float b);
 private:
  double micro(double x, double y) const;
};
