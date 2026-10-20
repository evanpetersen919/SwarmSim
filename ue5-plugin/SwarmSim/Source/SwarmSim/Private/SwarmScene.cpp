#include "SwarmScene.h"

#include "Components/SceneComponent.h"
#include "Materials/MaterialParameterCollection.h"
#include "SwarmLandscapeBuilder.h"
#include "SwarmSceneBuilder.h"
#include "SwarmSimSettings.h"

ASwarmScene::ASwarmScene() {
  USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
  SetRootComponent(Root);
}

bool ASwarmScene::BuildFromSceneFile(const FString& ScenePath, FString& OutError) {
  swarm::SceneData Scene;
  if (!SwarmSim::LoadSceneFile(ScenePath, Scene, OutError)) {
    return false;
  }
  return SwarmSim::BuildSceneContents(this, Scene, SwarmSim::LoadSceneAssets(), OutError);
}

void ASwarmScene::SetWind(float Intensity, float XSpeed, float YSpeed, float VariationTiling) {
  const USwarmSimSettings* Settings = GetDefault<USwarmSimSettings>();
  UMaterialParameterCollection* Collection = Settings->WindCollection.LoadSynchronous();
  UWorld* World = GetWorld();
  SwarmSim::SetCollectionScalar(World, Collection, TEXT("Wind_Intensity"), Intensity);
  SwarmSim::SetCollectionScalar(World, Collection, TEXT("Wind_X_Speed"), XSpeed);
  SwarmSim::SetCollectionScalar(World, Collection, TEXT("Wind_Y_Speed"), YSpeed);
  SwarmSim::SetCollectionScalar(World, Collection, TEXT("Wind_Variation_Tiling"), VariationTiling);
}

void ASwarmScene::SetSnowCoverage(float Coverage) {
  const USwarmSimSettings* Settings = GetDefault<USwarmSimSettings>();
  SwarmSim::SetCollectionScalar(GetWorld(), Settings->SnowCollection.LoadSynchronous(),
                                TEXT("Snow_Coverage"), FMath::Clamp(Coverage, 0.0f, 1.0f));
}
