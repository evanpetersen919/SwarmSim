#pragma once

#include "CoreMinimal.h"
#include "swarm/format/scene.hpp"

// The parts of exporting a scene from a running level that need no engine objects, so they can be
// tested directly.
//
// The exporter turns a level into the scene file the simulation can run on: a heightfield of the
// ground and cylinders and boxes for the trees and rocks, in the simulation's coordinates. The
// simulation's origin is at the anchor, a point of the level in centimeters, and the scene covers a
// square centered on it.
//
// Command line options (the anchor is the viewer's -SwarmAnchor):
//   -SwarmExportScene=PATH   write the scene file here and quit
//   -SwarmExportSize=M       side of the square, in meters (default 1000)
//   -SwarmExportCell=M       distance between height samples, in meters (default 8)
//   -SwarmExportWait=S       seconds to let the level stream in before sampling (default 45)
//   -SwarmExportTrees=A,B    meshes whose name contains one of these are trees
//   -SwarmExportRocks=A,B    meshes whose name contains one of these are rocks
//   -SwarmExportGround=A,B   meshes whose name contains one of these are ground, like the
//                            Landscape (cliffs and terrain meshes that cover holes in it)
//   -SwarmExportMinTree=M    a tree must be at least this tall, in meters (default 3)
//   -SwarmExportMinRock=M    a rock must be at least this large in some direction, in meters
//                            (default 1)
//   -SwarmExportMaxGap=F     largest share of samples that may have no ground (default 0.01)
//   -SwarmExportTrunk=F      a tree's trunk radius as this fraction of its half width
//                            (default 0.15)

namespace SwarmSim {
struct FExportOptions {
  FString OutPath;
  FVector2D AnchorCm = FVector2D::ZeroVector;
  double SizeM = 1000.0;
  double CellM = 8.0;
  double WaitSeconds = 45.0;
  TArray<FString> TreeNames;
  TArray<FString> RockNames;
  TArray<FString> GroundNames;
  double MinTreeHeightM = 3.0;
  double MinRockSizeM = 1.0;
  double MaxRockSizeM = 60.0;
  double MaxGapFraction = 0.01;
  double TrunkFraction = 0.15;
  double MinTrunkRadiusM = 0.2;
  double MaxTrunkRadiusM = 1.5;
  bool bValid = true;  // false if a value could not be read
  FString Problem;
};

FExportOptions ParseExportOptions(const TCHAR* CommandLine);

// Whether `Name` contains one of the patterns, ignoring case. Empty patterns match nothing.
bool NameMatchesAny(const FString& Name, const TArray<FString>& Patterns);

enum class EObstacleClass : uint8 {
  None,
  Tree,
  Rock,
};

// Whether a mesh is a tree or a rock by its name: the first list that has a name contained in
// `MeshName` (ignoring case) decides, trees before rocks. Neither gives None.
EObstacleClass ClassifyMesh(const FString& MeshName, const TArray<FString>& TreeNames,
                            const TArray<FString>& RockNames);

// The number of cells along a side: the size over the cell size, rounded up, at least 1 and at most
// the scene format's limit.
int32 CellsFor(double SizeM, double CellM);

// The level position, in centimeters, of height sample (i, j), for a grid of `Cells` cells a side.
// Sample (0, 0) is at the simulation's (-Cells * CellM / 2, -Cells * CellM / 2).
FVector2D GridPointCm(const FExportOptions& Options, int32 Cells, int32 I, int32 J);

// Gives each sample that has no height the mean of its neighbors that have one, repeatedly, until
// every sample has one. Returns the number filled, or -1 if no sample had a height.
int32 FillMissingHeights(TArray<double>& Heights, TArray<bool>& Known, int32 SamplesX,
                         int32 SamplesY);

// The obstacle a mesh's world bounds (in centimeters, in the level's frame) stand for. A tree is a
// cylinder from the bottom of the bounds to its top, with a radius of `TrunkFraction` of the half
// width limited to the trunk limits; a rock is the box of the bounds. False if the bounds are not
// a tree or a rock of the allowed size.
bool ObstacleFromBounds(EObstacleClass Kind, const FBox& BoundsCm, const FExportOptions& Options,
                        swarm::Obstacle& Out);

// The scene from sampled heights (meters, x varying fastest) and the obstacles. False and `Error`
// if the result is not a valid scene.
bool AssembleScene(const FExportOptions& Options, int32 Cells, const TArray<double>& HeightsM,
                   const TArray<swarm::Obstacle>& Obstacles, swarm::SceneData& Out, FString& Error);
}  // namespace SwarmSim
