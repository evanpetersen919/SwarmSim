#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "SwarmSceneLibrary.generated.h"

class ALandscape;

// Entry points that Blueprints and the editor's Python can call. The work is done by plain C++
// in the plugin's private code.
UCLASS()
class SWARMSIM_API USwarmSceneLibrary : public UBlueprintFunctionLibrary {
  GENERATED_BODY()

 public:
  // Reads a scene file (written by `swarm_sim export-scene`) and builds its terrain as a Landscape
  // in the world of `WorldContextObject`, using the assets named in the SwarmSim viewer settings.
  // Returns null and sets `OutError` on failure. Landscapes can only be built in the editor.
  UFUNCTION(BlueprintCallable, Category = "SwarmSim", meta = (WorldContext = "WorldContextObject"))
  static ALandscape* BuildTerrainFromSceneFile(UObject* WorldContextObject,
                                               const FString& ScenePath, FString& OutError);

  // The largest difference in centimeters between the Landscape's height and the scene file's
  // heightfield over all samples; negative if it cannot be measured.
  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  static double MeasureTerrainHeightErrorCm(ALandscape* Landscape, const FString& ScenePath);
};
