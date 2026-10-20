#pragma once

#include "CoreMinimal.h"
#include "swarm/format/scene.hpp"

// The parts of building a Landscape from the simulation's heightfield that need no engine objects,
// so they can be tested directly.
//
// A Landscape is a grid of components, each of NumSubsections x NumSubsections subsections of
// SubsectionSizeQuads quads per side, so the quad count along an axis is a whole number of
// components and the subsection size is one of a fixed set. The simulation's grid has any size, so
// the Landscape is made at least as large and the surplus is filled by repeating the edge height.
// The surplus lies outside the simulation's world and is cosmetic.
//
// Orientation. Landscape vertex (u, v) lies at the Landscape's origin plus (u, v) cells, so u runs
// along Unreal +X and v along +Y. Unreal Y is the simulation's -y (SwarmCoordinates.h), so v runs
// toward the simulation's -y: vertex (u, v) takes the simulation's sample (i, j) with
//   i = min(u, cells_x)     j = max(0, cells_y - v)
// and the Landscape's origin is placed at the simulation point (origin_x, origin_y + extent_y).

namespace SwarmSim {
struct FLandscapeLayout {
  int32 SubsectionSizeQuads = 0;
  int32 NumSubsections = 0;
  int32 ComponentsX = 0;
  int32 ComponentsY = 0;

  int32 ComponentQuads() const { return SubsectionSizeQuads * NumSubsections; }
  int32 QuadsX() const { return ComponentsX * ComponentQuads(); }
  int32 QuadsY() const { return ComponentsY * ComponentQuads(); }
  int32 VerticesX() const { return QuadsX() + 1; }
  int32 VerticesY() const { return QuadsY() + 1; }
};

// The layout with the fewest surplus quads that covers `CellsX` by `CellsY` cells; larger
// subsections win a tie, because they mean fewer components. False if the grid is empty or larger
// than a Landscape can be.
bool ChooseLandscapeLayout(uint32 CellsX, uint32 CellsY, FLandscapeLayout& OutLayout);

// The simulation sample behind Landscape vertex (U, V), by the rule above.
void LandscapeVertexToSample(const swarm::Heightfield& Terrain, int32 U, int32 V, int32& OutI,
                             int32& OutJ);

// Heights as Landscape stores them, one uint16 per vertex, u varying fastest. The Landscape actor's
// Z scale is chosen to fit the terrain's range and returned in OutScaleZ.
void EncodeLandscapeHeights(const swarm::Heightfield& Terrain, const FLandscapeLayout& Layout,
                            TArray<uint16>& OutHeights, float& OutScaleZ);

// The height in centimeters that an encoded value stands for at a given Z scale.
double DecodeLandscapeHeightCm(uint16 Value, float ScaleZ);

enum class EGround : uint8 {
  Grass,
  Rock,
  Mud,
  ForestA,
  ForestB,
};

struct FGroundRules {
  float RockSlope = 0.55f;
  float RockBlendWidth = 0.25f;
};

// Per-vertex weights (0 to 255) for the ground types in `Present`, in the Landscape's vertex order.
// The weights of the types present sum to 255 at every vertex. Rock follows slope, mud the lowest
// ground, and the rest is grass with patches of the two forest floors, from noise that depends on
// the grid position and `Seed` only. A type not in `Present` has its share given to grass if grass
// is present, and otherwise to the first type present.
void ComputeGroundWeights(const swarm::Heightfield& Terrain, const FLandscapeLayout& Layout,
                          uint64 Seed, const FGroundRules& Rules, const TArray<EGround>& Present,
                          TMap<EGround, TArray<uint8>>& OutWeights);
}  // namespace SwarmSim
