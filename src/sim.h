// Node/beam soft-body solver.
//
// Everything deformable is a set of point masses (nodes) joined by springs (beams).
// Beams are elasto-plastic: past the yield force the rest length changes permanently
// (with work hardening), and past the ductile limit they tear. Integrated explicitly
// at a high fixed rate, so crashes emerge from the structure rather than from scripts.
#pragma once
#include <random>
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
  // As built. Static: used by analysis tools, not by the solver.
  u32 nodeStart = 0, nodeCount = 0;
  u32 beamStart = 0, beamCount = 0;
  // What the object consists of right now. Tearing moves entries from here into new objects.
  std::vector<u32> nodes, beamList;
  u16 root = 0;        // the object this piece originally belonged to
  bool dirty = false;  // a beam broke: check whether it has come apart
  ObjKind kind = OBJ_PROP;
  bool awake = false;
  bool alwaysAwake = false;
  bool scrap = false;     // a burnt-out wreck: whatever still jiggles in it is quickly damped away
  bool armoured = false;  // a blast pushes it, but does not simply tear it apart
  float sleepTimer = 0;
  V3 bbMin, bbMax;
};

// Beam tags of trees: their breaking is heard and seen differently from steel.
constexpr u8 kTagWood = 200, kTagBranch = 201;

enum EventKind : u8 { EV_SPARK, EV_DUST, EV_GLASS, EV_CRUNCH, EV_TIRE_SMOKE, EV_BLAST, EV_FIRE, EV_MUZZLE, EV_TRACER, EV_WOOD, EV_BRANCH, EV_ROCK, EV_IMPACT };
struct SimEvent {
  V3 p, v;
  float strength;
  EventKind kind;
};

// Purely visual: the car touched something here, in its as-built position, moving this way.
struct Scrape {
  V3 restPoint, dir;
  float strength;
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
  std::vector<Scrape> scrapes;

  const World* world = nullptr;
  World* craterWorld = nullptr;  // set where explosions may change the ground (the game, not the tests)
  Grab grab;
  double g = 9.81;
  double plasticEnergy = 0;  // cumulative plastic work done on the car [J]
  u32 brokenBeams = 0;
  u64 stepCount = 0;
  size_t statTriPairs = 0, statBlockPairs = 0;
  u32 blastCount = 0;   // for whoever wants to know what just went off
  V3 lastBlast;
  double lastBlastRadius = 0, lastBlastImpulse = 0;
  // For the duel server: what went off, for the players' machines to set off as well.
  struct BlastRecord {
    V3 c;
    double impulse, radius, hole;
  };
  bool logBlasts = false;
  std::vector<BlastRecord> blastLog;
  // A player's copy of the world, which is never stepped: a blast digs its crater and cracks its rocks here,
  // because those are the same everywhere, and all that moves comes from the server.
  bool mirror = false;
  u32 firstSceneryNode = 0;  // what comes before is drawn as vehicles, what comes after from its triangles
  u32 splitGeneration = 0;  // goes up whenever an object tears into pieces
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
  // Puts the first count objects back as they were in start, wherever pieces of them have got to, and
  // leaves everything else in the world exactly as it is.
  void restoreObjects(const Sim& start, u32 count);
  // Takes over the state of everything but the vehicle from a simulation of the same world, in which the
  // vehicle had oldCars objects where this one has newCars. Returns false if the worlds do not match.
  bool adoptWorld(const Sim& old, u32 oldCars, u32 newCars);
  u32 carResets = 0;  // goes up each time that happens: the visible mesh must rebind too
  // Parts a beam at once, as a bolt shearing does.
  void sever(u32 beam) {
    Beam& b = beams[beam];
    if (!(b.flags & BF_BROKEN)) {
      b.flags |= BF_BROKEN;
      brokenBeams++;
      objs[nodeObj[b.a]].dirty = true;
    }
  }
  // A detonation: a pressure wave running out from c. impulse is in N s per square metre of surface
  // at 1 m from the centre, radius is where it has faded out. Tears what is nearest, throws the rest.
  void explode(V3 c, double impulse, double radius, double hole = 1.0);  // hole: how much bigger its crater than its reach says
  void event(EventKind k, V3 p, V3 v, float s);
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
  std::vector<int> nodeSlot;
  bool triObjDirty = false;
  std::vector<V3> blockCentre;
  std::vector<double> blockRad2;
  std::vector<u64> blockStep;
  std::vector<Pair> triPairs, blockPairs;
  std::vector<int> hashHead;
  struct HashEntry { u32 prim; int next; };
  std::vector<HashEntry> hashEntries;
  std::vector<u32> stamp;
  std::vector<u32> stamped;  // which entries of it are in use, so that only those are put back
  // What can be touched, object by object, so that a pair search looks only at what is awake.
  std::vector<std::vector<u32>> objTris, objBlocks;
  size_t primsListed = size_t(-1);
  u32 primsSplit = u32(-1);
  void listPrimitives();
  double pairMargin = 0.1;

  void rebuildPairs(double dt);
  void collidePairs(double dt);
  void collideWorld(double dt);
  void integrate(double dt);
  void sleepWake();
  void splitObjects();
  void breakBeam(Beam& b, bool notify, double f);
  void computeBounds(Obj& o);
  bool triPairAllowed(u32 node, const Tri& t) const;
  void contactFriction(u32 n, V3 nrm, double fn, double muv, double dt, V3 relVel, double mEff,
                       V3* out);
  struct Blast {
    V3 c;
    double impulse, radius, age;
  };
  std::vector<Blast> blasts;
  std::minstd_rand blastRng{2024};
  void applyBlasts(double dt);
};
