#pragma once

#include "CoreMinimal.h"
#include "SwarmLandscapeBuilder.h"
#include "swarm/format/scene.hpp"

class ASwarmScene;
class UMaterialParameterCollection;
class UStaticMesh;

namespace SwarmSim {
// Everything the scene is made of. Any part may be absent: no tree or rock meshes give the
// engine's cone and cube, no collections leave the wind and snow alone, and no terrain look gives
// an untextured Landscape.
struct FSceneAssets {
  FTerrainLook Terrain;
  TArray<UStaticMesh*> TreeMeshes;
  TArray<UStaticMesh*> RockMeshes;
  UMaterialParameterCollection* WindCollection = nullptr;
  UMaterialParameterCollection* SnowCollection = nullptr;
  float WindIntensity = 2.0f;
  float WindXSpeed = 12.5f;
  float WindYSpeed = 5.0f;
  float WindVariationTiling = 5.0f;
  float SnowCoverage = 0.0f;
};

// The assets named in the SwarmSim viewer settings, loaded.
FSceneAssets LoadSceneAssets();

// Builds the scene's contents into `Actor`: the terrain (editor only), the trees and rocks, and
// the wind and snow. The names of the pack's scalar parameters are fixed in the implementation.
bool BuildSceneContents(ASwarmScene* Actor, const swarm::SceneData& Scene,
                        const FSceneAssets& Assets, FString& OutError);

// Sets a scalar parameter of a collection in a world. False if the collection or parameter is
// missing.
bool SetCollectionScalar(UWorld* World, UMaterialParameterCollection* Collection, FName Name,
                         float Value);
}  // namespace SwarmSim
