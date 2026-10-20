#pragma once

#include "CoreMinimal.h"
#include "Engine/DeveloperSettings.h"
#include "SwarmSimSettings.generated.h"

class UMaterialInterface;
class ULandscapeLayerInfoObject;

// Which assets the viewer uses. Every entry is a soft reference, and every entry may be empty or
// point at an asset that is not installed: the viewer then falls back to what it can build from
// code alone (an untextured landscape, stand-in meshes). The defaults name the assets of the
// Procedural Nature Pack Vol. 2 by PurePolygons, which is not part of this repository and is
// installed into the host project's Content folder separately (see docs/DEVELOPMENT.md).
UCLASS(Config = SwarmSim, DefaultConfig, meta = (DisplayName = "SwarmSim Viewer"))
class SWARMSIM_API USwarmSimSettings : public UDeveloperSettings {
  GENERATED_BODY()

 public:
  USwarmSimSettings();

  // Material of the terrain. It must be a landscape material.
  UPROPERTY(Config, EditAnywhere, Category = "Terrain")
  TSoftObjectPtr<UMaterialInterface> LandscapeMaterial;

  // Layer info objects for the terrain's ground types, by role. A role whose asset is missing is
  // left out; with no layers the terrain gets the material's default look.
  UPROPERTY(Config, EditAnywhere, Category = "Terrain")
  TSoftObjectPtr<ULandscapeLayerInfoObject> GrassLayer;

  UPROPERTY(Config, EditAnywhere, Category = "Terrain")
  TSoftObjectPtr<ULandscapeLayerInfoObject> RockLayer;

  UPROPERTY(Config, EditAnywhere, Category = "Terrain")
  TSoftObjectPtr<ULandscapeLayerInfoObject> MudLayer;

  UPROPERTY(Config, EditAnywhere, Category = "Terrain")
  TSoftObjectPtr<ULandscapeLayerInfoObject> ForestGroundALayer;

  UPROPERTY(Config, EditAnywhere, Category = "Terrain")
  TSoftObjectPtr<ULandscapeLayerInfoObject> ForestGroundBLayer;

  // Slope (rise over run) above which the terrain is rock, and the slope range over which grass
  // gives way to rock.
  UPROPERTY(Config, EditAnywhere, Category = "Terrain",
            meta = (ClampMin = "0.05", ClampMax = "5.0"))
  float RockSlope = 0.55f;

  UPROPERTY(Config, EditAnywhere, Category = "Terrain",
            meta = (ClampMin = "0.01", ClampMax = "2.0"))
  float RockBlendWidth = 0.25f;

  virtual FName GetCategoryName() const override { return FName(TEXT("Plugins")); }
};
