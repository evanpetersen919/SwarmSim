#include "SwarmSceneBuilder.h"

#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Kismet/KismetMaterialLibrary.h"
#include "Materials/MaterialParameterCollection.h"
#include "Materials/MaterialParameterCollectionInstance.h"
#include "SwarmObstaclePlan.h"
#include "SwarmScene.h"
#include "SwarmSimSettings.h"

#if WITH_EDITOR
#include "LandscapeLayerInfoObject.h"
#endif

namespace SwarmSim {
namespace {
// The pack's parameter names.
const FName kWindIntensity(TEXT("Wind_Intensity"));
const FName kWindXSpeed(TEXT("Wind_X_Speed"));
const FName kWindYSpeed(TEXT("Wind_Y_Speed"));
const FName kWindTiling(TEXT("Wind_Variation_Tiling"));
const FName kSnowCoverage(TEXT("Snow_Coverage"));

UStaticMesh* LoadEngineShape(const TCHAR* Path) { return LoadObject<UStaticMesh>(nullptr, Path); }

FMeshFit FitOf(const UStaticMesh* Mesh, bool bFitTreeRadius) {
  const FBoxSphereBounds Bounds = Mesh->GetBounds();
  FMeshFit Fit;
  Fit.Origin = Bounds.Origin;
  Fit.Extent = Bounds.BoxExtent;
  Fit.bFitTreeRadius = bFitTreeRadius;
  return Fit;
}

UHierarchicalInstancedStaticMeshComponent* AddInstances(ASwarmScene* Actor, UStaticMesh* Mesh,
                                                        const TArray<FTransform>& Transforms,
                                                        const TCHAR* Name) {
  UHierarchicalInstancedStaticMeshComponent* Component =
      NewObject<UHierarchicalInstancedStaticMeshComponent>(Actor, FName(Name));
  Component->SetStaticMesh(Mesh);
  Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
  Component->SetupAttachment(Actor->GetRootComponent());
  Component->RegisterComponent();
  Actor->AddInstanceComponent(Component);
  Component->AddInstances(Transforms, /*bShouldReturnIndices=*/false, /*bWorldSpace=*/true);
  return Component;
}
}  // namespace

bool SetCollectionScalar(UWorld* World, UMaterialParameterCollection* Collection, FName Name,
                         float Value) {
  if (World == nullptr || Collection == nullptr) {
    return false;
  }
  bool bKnown = false;
  for (const FCollectionScalarParameter& P : Collection->ScalarParameters) {
    bKnown = bKnown || P.ParameterName == Name;
  }
  if (!bKnown) {
    return false;
  }
  UKismetMaterialLibrary::SetScalarParameterValue(World, Collection, Name, Value);
  return true;
}

FSceneAssets LoadSceneAssets() {
  FSceneAssets Assets;
  const USwarmSimSettings* Settings = GetDefault<USwarmSimSettings>();
  Assets.Terrain.Rules.RockSlope = Settings->RockSlope;
  Assets.Terrain.Rules.RockBlendWidth = Settings->RockBlendWidth;
  Assets.Terrain.Material = Settings->LandscapeMaterial.LoadSynchronous();
#if WITH_EDITOR
  const auto AddLayer = [&Assets](EGround Ground,
                                  const TSoftObjectPtr<ULandscapeLayerInfoObject>& Soft) {
    if (ULandscapeLayerInfoObject* Info = Soft.LoadSynchronous()) {
      Assets.Terrain.Layers.Add(Ground, Info);
    }
  };
  AddLayer(EGround::Grass, Settings->GrassLayer);
  AddLayer(EGround::Rock, Settings->RockLayer);
  AddLayer(EGround::Mud, Settings->MudLayer);
  AddLayer(EGround::ForestA, Settings->ForestGroundALayer);
  AddLayer(EGround::ForestB, Settings->ForestGroundBLayer);
#endif
  for (const TSoftObjectPtr<UStaticMesh>& Soft : Settings->TreeMeshes) {
    if (UStaticMesh* Mesh = Soft.LoadSynchronous()) {
      Assets.TreeMeshes.Add(Mesh);
    }
  }
  for (const TSoftObjectPtr<UStaticMesh>& Soft : Settings->RockMeshes) {
    if (UStaticMesh* Mesh = Soft.LoadSynchronous()) {
      Assets.RockMeshes.Add(Mesh);
    }
  }
  Assets.WindCollection = Settings->WindCollection.LoadSynchronous();
  Assets.SnowCollection = Settings->SnowCollection.LoadSynchronous();
  Assets.WindIntensity = Settings->WindIntensity;
  Assets.WindXSpeed = Settings->WindXSpeed;
  Assets.WindYSpeed = Settings->WindYSpeed;
  Assets.WindVariationTiling = Settings->WindVariationTiling;
  Assets.SnowCoverage = Settings->SnowCoverage;
  return Assets;
}

bool BuildSceneContents(ASwarmScene* Actor, const swarm::SceneData& Scene,
                        const FSceneAssets& Assets, FString& OutError) {
  UWorld* World = Actor != nullptr ? Actor->GetWorld() : nullptr;
  if (World == nullptr) {
    OutError = TEXT("the scene actor is not in a world");
    return false;
  }
  if (Actor->GetRootComponent() == nullptr) {
    OutError = TEXT("the scene actor has no root component");
    return false;
  }

#if WITH_EDITOR
  Actor->Terrain = BuildLandscape(World, Scene, Assets.Terrain, OutError);
  if (Actor->Terrain == nullptr) {
    return false;
  }
#endif

  // Trees and rocks: the pack's meshes, or the engine's cone and cube.
  TArray<UStaticMesh*> TreeMeshes = Assets.TreeMeshes;
  TArray<UStaticMesh*> RockMeshes = Assets.RockMeshes;
  bool bTreeStandIn = false;
  Actor->bUsingStandIns = false;
  if (TreeMeshes.IsEmpty()) {
    if (UStaticMesh* Cone = LoadEngineShape(TEXT("/Engine/BasicShapes/Cone.Cone"))) {
      TreeMeshes.Add(Cone);
      bTreeStandIn = true;
      Actor->bUsingStandIns = true;
    }
  }
  if (RockMeshes.IsEmpty()) {
    if (UStaticMesh* Cube = LoadEngineShape(TEXT("/Engine/BasicShapes/Cube.Cube"))) {
      RockMeshes.Add(Cube);
      Actor->bUsingStandIns = true;
    }
  }
  TArray<FMeshFit> TreeFits;
  for (const UStaticMesh* Mesh : TreeMeshes) {
    TreeFits.Add(FitOf(Mesh, bTreeStandIn));
  }
  TArray<FMeshFit> RockFits;
  for (const UStaticMesh* Mesh : RockMeshes) {
    RockFits.Add(FitOf(Mesh, false));
  }

  FObstaclePlan Plan;
  PlanObstacles(Scene, TreeFits, RockFits, swarm::scene_hash(Scene), Plan);
  Actor->TreeCount = Plan.TreeCount;
  Actor->RockCount = Plan.RockCount;
  for (int32 I = 0; I < TreeMeshes.Num(); ++I) {
    if (!Plan.TreeInstances[I].IsEmpty()) {
      Actor->TreeComponents.Add(AddInstances(Actor, TreeMeshes[I], Plan.TreeInstances[I],
                                             *FString::Printf(TEXT("Trees%d"), I)));
    }
  }
  for (int32 I = 0; I < RockMeshes.Num(); ++I) {
    if (!Plan.RockInstances[I].IsEmpty()) {
      Actor->RockComponents.Add(AddInstances(Actor, RockMeshes[I], Plan.RockInstances[I],
                                             *FString::Printf(TEXT("Rocks%d"), I)));
    }
  }

  // Wind and snow.
  SetCollectionScalar(World, Assets.WindCollection, kWindIntensity, Assets.WindIntensity);
  SetCollectionScalar(World, Assets.WindCollection, kWindXSpeed, Assets.WindXSpeed);
  SetCollectionScalar(World, Assets.WindCollection, kWindYSpeed, Assets.WindYSpeed);
  SetCollectionScalar(World, Assets.WindCollection, kWindTiling, Assets.WindVariationTiling);
  SetCollectionScalar(World, Assets.SnowCollection, kSnowCoverage, Assets.SnowCoverage);
  return true;
}
}  // namespace SwarmSim
