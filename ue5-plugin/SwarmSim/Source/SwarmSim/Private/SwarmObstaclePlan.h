#pragma once

#include "CoreMinimal.h"
#include "swarm/format/scene.hpp"

// Where the scene's obstacles are drawn and how the meshes are fitted to them, with no engine
// objects, so it can be tested directly.
//
// A cylinder obstacle becomes a tree and a box obstacle becomes a rock. The simulation's LiDAR sees
// the cylinder and the box, so the tree and the rock are fitted to them:
//   * A tree is the mesh whose height is closest to the cylinder's, scaled uniformly to that
//   height,
//     standing on the cylinder's base at the cylinder's x and y, and turned to a yaw that depends
//     on the position and the seed only, so that trees of one mesh do not all face one way.
//   * A rock is scaled in each axis so that the mesh's bounds are exactly the box's bounds. The box
//     is aligned with the axes, so the rock is not turned. A box much taller than it is wide is
//     filled by several rocks stacked up to its height, so that none is stretched into a pillar.
// A mesh's bounds are its local center and half sizes in centimeters, as the engine gives them.

namespace SwarmSim {
struct FMeshFit {
  FVector Origin = FVector::ZeroVector;  // center of the mesh's bounds
  FVector Extent = FVector::OneVector;   // half size of the mesh's bounds
  // A tree normally keeps its mesh's proportions. A stand-in cone is instead scaled to the
  // cylinder's radius as well as its height, so that it is as thin as the cylinder.
  bool bFitTreeRadius = false;
};

struct FObstaclePlan {
  TArray<TArray<FTransform>> TreeInstances;  // one list per tree mesh
  TArray<TArray<FTransform>> RockInstances;  // one list per rock mesh; a stacked box has several
  int32 TreeCount = 0;
  int32 RockCount = 0;  // obstacles, not instances
};

// Plans every obstacle of the scene. A kind with no mesh to use is left out (its count stays zero).
// A cylinder's scale is limited to [MinTreeScale, MaxTreeScale] so a mesh that is far from the
// wanted height is not stretched absurdly.
void PlanObstacles(const swarm::SceneData& Scene, const TArray<FMeshFit>& Trees,
                   const TArray<FMeshFit>& Rocks, uint64 Seed, FObstaclePlan& OutPlan);

constexpr double MinTreeScale = 0.4;
constexpr double MaxTreeScale = 3.0;
// The ratio of a rock height scale to its footprint scale above which rocks are stacked.
constexpr double MaxRockStretch = 1.6;
}  // namespace SwarmSim
