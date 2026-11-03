// Destructible roadside objects, built from the same nodes and beams as the car.
#pragma once
#include "sim.h"
#include "world.h"

enum PropColor : u8 {
  COL_STEEL, COL_SIGN_RED, COL_SIGN_WHITE, COL_SIGN_BLUE, COL_WOOD, COL_WOOD_DARK, COL_STONE,
  COL_STONE2, COL_CONCRETE, COL_GUARDRAIL, COL_CONE, COL_BARK, COL_NEEDLES, COL_NEEDLES2, COL_NEEDLES3, COL_COUNT
};

// A conifer: a brittle trunk and a crown of soft branches. Rooted in the ground.
void addTree(Sim& sim, const World& w, const V3& foot, u32 seed);
void addSign(Sim& sim, const World& w, double x, double y, double yaw, u8 color);
void addFence(Sim& sim, const World& w, double x, double y, double yaw, int bays);
void addStoneWall(Sim& sim, const World& w, double x, double y, double yaw, int columns, int rows);
void addGuardrail(Sim& sim, const World& w, double x, double y, double yaw, int bays);
void addCone(Sim& sim, const World& w, double x, double y);
void addBarrier(Sim& sim, const World& w, double x, double y, double yaw);

// Places the full set: test gauntlet on the pad and furniture along the road.
void populateWorld(Sim& sim, const World& w);
// The duel's map: its trees, and the walls and fences of its villages.
void populateArena(Sim& sim, const World& w);
// A plain box that touches nothing, deep underground until it is moved to where it is wanted: mines and crates.
// Returns its first node of eight.
u32 addMarker(Sim& sim, V3 half, u8 color);
inline constexpr double kMarkerDepth = -400.0;
void moveMarker(Sim& sim, u32 first, V3 at);
