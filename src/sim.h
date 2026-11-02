// Node/beam soft-body solver.
//
// Everything deformable is a set of point masses (nodes) joined by springs (beams).
// Beams are elasto-plastic: past the yield force the rest length changes permanently
// (with work hardening), and past the ductile limit they tear. Integrated explicitly
// at a high fixed rate, so crashes emerge from the structure rather than from scripts.
#pragma once
#include <vector>
#include "mathx.h"

class World;

enum NodeFlags : u8 {
  NF_FIXED = 1,    // anchored to the world
  NF_COLLIDE = 2,  // collides with world and other objects
  NF_WHEEL = 4,    // part of a wheel assembly
};

enum TriFlags : u8 {
  TF_HARD = 1,     // also collides with nodes of its own object (bulkheads, engine block)
  TF_VISUAL = 2,   // render only, no collision
  TF_NOVIS = 4,    // collision only, not drawn in the generic prop pass
};

enum BeamFlags : u8 {
  BF_BROKEN = 1,
  BF_COMPRESS_ONLY = 2,
  BF_TENSION_ONLY = 4,
};

// Mechanical properties of a beam.
struct Mat {
  double k = 1e6;       // axial stiffness [N/m]
  double c = 200;       // axial damping [N s/m]
  double yC = 0;        // compressive yield [N], 0 = stays elastic
  double yT = 0;        // tensile yield [N], 0 = stays elastic
  double hard = 0;      // work hardening [N per unit plastic strain]
  double maxStrain = 0; // tensile plastic strain at which it tears, 0 = never
  double fBreak = 0;    // absolute force at which it snaps, 0 = never
};

struct Beam {
  u32 a, b;
  double L0;      // current rest length (changes with plastic flow)
  double Lorig;   // as-built length
  double k, c;
  double yC, yT;
  double hard;
  double maxStrain;
  double fBreak;
  float force;    // last axial force, positive in tension (for the debug overlay)
  u8 flags;
  u8 tag;         // user tag for damage monitoring
};

struct Tri {
  u32 a, b, c;
  u16 obj;
  u8 flags;
  u8 color;
  float mu;
};

// Eight-node hexahedral rigid-ish block (stone, concrete). Corner index bits: x=1 y=2 z=4.
// Colliding against the block as a box is far cheaper than 12 triangles, which is what
// makes a whole wall of loose stones affordable.
struct Block {
  u32 n[8];
  u16 obj;
};

enum ObjKind : u8 { OBJ_CAR, OBJ_PROP };

struct Obj {
  u32 nodeStart = 0, nodeCount = 0;
  u32 beamStart = 0, beamCount = 0;
  ObjKind kind = OBJ_PROP;
  bool awake = false;
  bool alwaysAwake = false;
  float sleepTimer = 0;
  V3 bbMin, bbMax;
};

enum EventKind : u8 { EV_SPARK, EV_DUST, EV_GLASS, EV_CRUNCH, EV_TIRE_SMOKE };
struct SimEvent {
  V3 p, v;
  float strength;
  EventKind kind;
};

struct Grab {  // mouse "node grabber"
  int node = -1;
  V3 target;
};

class Sim {
 public:
  // Node data, structure-of-arrays for cache-friendly tight loops.
  std::vector<V3> pos, vel, force, rest;
  std::vector<double> mass, invMass;
  std::vector<float> mu;
  std::vector<u8> flags;
  std::vector<u16> nodeObj;

  std::vector<Beam> beams;
  std::vector<Tri> tris;
  std::vector<Block> blocks;
  std::vector<Obj> objs;
  std::vector<u32> active;  // indices of awake objects
  std::vector<SimEvent> events;

  const World* world = nullptr;
  Grab grab;
  double g = 9.81;
  double plasticEnergy = 0;  // cumulative plastic work done on the car [J]
  u32 brokenBeams = 0;
  u64 stepCount = 0;
  size_t statTriPairs = 0, statBlockPairs = 0;
  u64 clampedSteps = 0;  // times the per-step velocity guard had to act: should stay at zero

  // Building.
  u16 beginObj(ObjKind kind, bool alwaysAwake);
  void endObj();
  u32 addNode(V3 p, double m, u8 fl = NF_COLLIDE, float friction = 0.6f);
  u32 addBeam(u32 a, u32 b, const Mat& m, u8 tag = 0, u8 fl = 0);
  void addTri(u32 a, u32 b, u32 c, u8 fl = 0, u8 color = 0, float friction = 0.5f);
  void addBlock(const u32 n[8]);
  void finalize();  // call after all objects exist

  // One step is: forces() -> vehicle systems add their forces -> finish().
  void forces(double dt);
  void finish(double dt);

  void wake(u16 obj);
  // Largest eigenvalue of M^-1 K for one object at its current shape, by power iteration. A
  // symplectic Euler step is stable only while this stays below (2/dt)^2.
  double maxEigenvalue(u32 obj, u32* worstNode = nullptr) const;

 private:
  struct Pair { u32 node, prim; };
  struct TriHit { u32 pair; double w0, w1, w2, d; V3 n; };
  struct BlockHit { u32 pair; double depth; V3 n; double wt[8]; };
  std::vector<TriHit> triHits;
  std::vector<BlockHit> blockHits;
  std::vector<float> contactLoad;
  std::vector<u32> touched;
  std::vector<Pair> triPairs, blockPairs;
  std::vector<int> hashHead;
  struct HashEntry { u32 prim; int next; };
  std::vector<HashEntry> hashEntries;
  std::vector<u32> stamp;
  double pairMargin = 0.1;

  void rebuildPairs(double dt);
  void collidePairs(double dt);
  void collideWorld(double dt);
  void integrate(double dt);
  void sleepWake();
  void computeBounds(Obj& o);
  bool triPairAllowed(u32 node, const Tri& t) const;
  void contactFriction(u32 n, V3 nrm, double fn, double muv, double dt, V3 relVel, double mEff,
                       V3* out);
  void event(EventKind k, V3 p, V3 v, float s);
};
