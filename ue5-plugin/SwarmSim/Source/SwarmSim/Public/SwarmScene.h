#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SwarmScene.generated.h"

class ALandscape;
class UHierarchicalInstancedStaticMeshComponent;

// The static scene of a recorded run, built from code: the terrain, the trees and rocks standing
// where the simulation's obstacles are, and the wind and snow of the material parameter
// collections. Nothing is placed by hand.
UCLASS()
class SWARMSIM_API ASwarmScene : public AActor {
  GENERATED_BODY()

 public:
  ASwarmScene();

  // Reads a scene file (written by `swarm_sim export-scene`) and builds the scene using the assets
  // named in the SwarmSim viewer settings. Returns false and sets `OutError` on failure. The
  // terrain can only be built in the editor.
  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  bool BuildFromSceneFile(const FString& ScenePath, FString& OutError);

  // Sets the wind of the pack's materials: intensity, the speed of the gust pattern along x and y,
  // and its tiling. Does nothing if the collection is not installed.
  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  void SetWind(float Intensity, float XSpeed, float YSpeed, float VariationTiling);

  // Sets the fraction of the scene covered in snow, from 0 (summer) to 1.
  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  void SetSnowCoverage(float Coverage);

  UPROPERTY(BlueprintReadOnly, Category = "SwarmSim")
  TObjectPtr<ALandscape> Terrain;

  UPROPERTY(BlueprintReadOnly, Category = "SwarmSim")
  TArray<TObjectPtr<UHierarchicalInstancedStaticMeshComponent>> TreeComponents;

  UPROPERTY(BlueprintReadOnly, Category = "SwarmSim")
  TArray<TObjectPtr<UHierarchicalInstancedStaticMeshComponent>> RockComponents;

  // How many trees and rocks were placed: the scene's cylinder and box obstacles.
  UPROPERTY(BlueprintReadOnly, Category = "SwarmSim")
  int32 TreeCount = 0;

  UPROPERTY(BlueprintReadOnly, Category = "SwarmSim")
  int32 RockCount = 0;

  // True if the trees or rocks are the engine's cone and cube because no mesh asset was available.
  UPROPERTY(BlueprintReadOnly, Category = "SwarmSim")
  bool bUsingStandIns = false;
};
