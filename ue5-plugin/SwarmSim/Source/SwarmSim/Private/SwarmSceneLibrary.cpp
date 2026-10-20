#include "SwarmSceneLibrary.h"

#include "Engine/World.h"
#include "Materials/MaterialInterface.h"
#include "SwarmLandscapeBuilder.h"
#include "SwarmSimSettings.h"

#if WITH_EDITOR
#include "Landscape.h"
#include "LandscapeLayerInfoObject.h"
#endif

ALandscape* USwarmSceneLibrary::BuildTerrainFromSceneFile(UObject* WorldContextObject,
                                                          const FString& ScenePath,
                                                          FString& OutError) {
#if WITH_EDITOR
  UWorld* World = GEngine != nullptr ? GEngine->GetWorldFromContextObject(
                                           WorldContextObject, EGetWorldErrorMode::ReturnNull)
                                     : nullptr;
  if (World == nullptr) {
    OutError = TEXT("no world for the terrain");
    return nullptr;
  }
  swarm::SceneData Scene;
  if (!SwarmSim::LoadSceneFile(ScenePath, Scene, OutError)) {
    return nullptr;
  }

  const USwarmSimSettings* Settings = GetDefault<USwarmSimSettings>();
  SwarmSim::FTerrainLook Look;
  Look.Rules.RockSlope = Settings->RockSlope;
  Look.Rules.RockBlendWidth = Settings->RockBlendWidth;
  Look.Material = Settings->LandscapeMaterial.LoadSynchronous();
  const auto Add = [&Look](SwarmSim::EGround Ground,
                           const TSoftObjectPtr<ULandscapeLayerInfoObject>& Soft) {
    if (ULandscapeLayerInfoObject* Info = Soft.LoadSynchronous()) {
      Look.Layers.Add(Ground, Info);
    }
  };
  Add(SwarmSim::EGround::Grass, Settings->GrassLayer);
  Add(SwarmSim::EGround::Rock, Settings->RockLayer);
  Add(SwarmSim::EGround::Mud, Settings->MudLayer);
  Add(SwarmSim::EGround::ForestA, Settings->ForestGroundALayer);
  Add(SwarmSim::EGround::ForestB, Settings->ForestGroundBLayer);
  return SwarmSim::BuildLandscape(World, Scene, Look, OutError);
#else
  OutError = TEXT("terrain can only be built in the editor");
  return nullptr;
#endif
}

double USwarmSceneLibrary::MeasureTerrainHeightErrorCm(ALandscape* Landscape,
                                                       const FString& ScenePath) {
#if WITH_EDITOR
  swarm::SceneData Scene;
  FString Error;
  if (!SwarmSim::LoadSceneFile(ScenePath, Scene, Error)) {
    return -1.0;
  }
  return SwarmSim::MaxLandscapeHeightErrorCm(Landscape, Scene);
#else
  return -1.0;
#endif
}
