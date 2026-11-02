// High-detail car mesh ("flexbody") bound to the physics nodes.
//
// Each vertex follows the nearby nodes of its part's group through moving-least-squares weights with
// a fixed support radius, so the weights vary smoothly from vertex to vertex and no seams can open
// up inside a panel. The weights reproduce any affine map exactly, so a rigidly moving part stays perfectly rigid while
// a deforming structure bends the mesh smoothly. There are no bones and no per-part transforms.
#pragma once
#include <string>
#include <vector>
#include "mathx.h"

class Sim;

enum MeshMat : u8 {
  MM_PAINT, MM_GLASS, MM_BLACK, MM_LINER, MM_CHROME, MM_LENS, MM_TAIL, MM_AMBER, MM_RUBBER, MM_ALLOY,
  MM_FABRIC, MM_CARPET, MM_PLATE, MM_METAL, MM_ALU, MM_LHM, MM_DARK, MM_DISC, MM_WHITE, MM_REFLECTOR,
  MM_RUST, MM_COUNT
};

struct NodeGroup {
  std::string name;
  std::vector<u32> nodes;
};

struct MeshPart {
  std::string name, group;
  u8 mat = 0;
  u32 firstVert = 0, vertCount = 0;
  u32 firstTri = 0, triCount = 0;
  bool bound = false;  // false for wheel parts, which are drawn with a matrix
};

struct CarMesh {
  static constexpr int kK = 24;
  std::vector<MeshPart> parts;
  std::vector<float> restPos;     // xyz per vertex, design coordinates
  std::vector<float> restNormal;  // xyz per vertex
  std::vector<u32> tris;          // 3 per triangle, indexing the whole vertex array
  std::vector<float> restEdge2;   // 3 per triangle, squared rest edge lengths
  std::vector<u32> bindNode;      // kK per vertex
  std::vector<float> bindWeight;  // kK per vertex

  bool load(const std::string& path);
  // Computes the weights. toWorld maps a design position to the world position at rest.
  void bind(const std::vector<V3>& restNodes, const std::vector<NodeGroup>& groups, V3 spawn);
  // After a tear: each vertex that follows nodes of several pieces is re-fitted to one of them, so the
  // mesh parts cleanly along the tear. Returns the number of vertices changed.
  size_t rebindAfterTear(const std::vector<V3>& restNodes, const std::vector<NodeGroup>& groups, V3 spawn, const std::vector<u16>& nodeObj);
  // Largest error of the bound mesh at rest, in metres (should be essentially zero).
  double restError(const std::vector<V3>& restNodes, V3 spawn, std::string* worst = nullptr) const;
};
