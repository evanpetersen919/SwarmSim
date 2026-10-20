#pragma once

#include "CoreMinimal.h"
#include "SwarmLandscapePlan.h"
#include "swarm/format/scene.hpp"

class ALandscape;
class UWorld;
class UMaterialInterface;
class ULandscapeLayerInfoObject;

namespace SwarmSim {
// What the terrain is made of. All of it may be absent: with no material and no layers the
// Landscape is untextured and still has the right shape.
struct FTerrainLook {
  UMaterialInterface* Material = nullptr;
  TMap<EGround, ULandscapeLayerInfoObject*> Layers;
  FGroundRules Rules;
};

// Loads a scene file (swarm/format/scene.hpp). False with a message on failure.
bool LoadSceneFile(const FString& Path, swarm::SceneData& OutScene, FString& OutError);

#if WITH_EDITOR
// Builds the terrain of a scene as a Landscape actor in `World`, positioned so that the
// simulation's heightfield lies where SwarmCoordinates.h puts it. The Landscape's grid is the
// simulation's, one Landscape vertex per heightfield sample (see SwarmLandscapePlan.h). Returns
// null with a message on failure.
ALandscape* BuildLandscape(UWorld* World, const swarm::SceneData& Scene, const FTerrainLook& Look,
                           FString& OutError);

// The largest difference, in centimeters, between the Landscape's height at a heightfield sample
// and the sample's height, over all samples. A negative value means the Landscape could not be
// queried.
double MaxLandscapeHeightErrorCm(ALandscape* Landscape, const swarm::SceneData& Scene);
#endif
}  // namespace SwarmSim
