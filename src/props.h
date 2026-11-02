// Destructible roadside objects, built from the same nodes and beams as the car.
#pragma once
#include "sim.h"
#include "world.h"

enum PropColor : u8 {
  COL_STEEL, COL_SIGN_RED, COL_SIGN_WHITE, COL_SIGN_BLUE, COL_WOOD, COL_WOOD_DARK, COL_STONE,
  COL_STONE2, COL_CONCRETE, COL_GUARDRAIL, COL_CONE, COL_COUNT
};

void addSign(Sim& sim, const World& w, double x, double y, double yaw, u8 color);
void addFence(Sim& sim, const World& w, double x, double y, double yaw, int bays);
void addStoneWall(Sim& sim, const World& w, double x, double y, double yaw, int columns, int rows);
void addGuardrail(Sim& sim, const World& w, double x, double y, double yaw, int bays);
void addCone(Sim& sim, const World& w, double x, double y);
void addBarrier(Sim& sim, const World& w, double x, double y, double yaw);

// Places the full set: test gauntlet on the pad and furniture along the road.
void populateWorld(Sim& sim, const World& w);
