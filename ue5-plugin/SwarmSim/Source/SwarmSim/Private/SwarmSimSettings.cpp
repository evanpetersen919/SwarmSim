#include "SwarmSimSettings.h"

#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialParameterCollection.h"

namespace {
constexpr const TCHAR* kPack = TEXT("/Game/ProceduralNtr_vol2");
}

USwarmSimSettings::USwarmSimSettings() {
  CategoryName = TEXT("Plugins");
  const FString Pack(kPack);
  LandscapeMaterial = TSoftObjectPtr<UMaterialInterface>(FSoftObjectPath(
      Pack +
      TEXT("/pnp_v2_assets/Materials/landscape_materials/landscape_seasonal.landscape_seasonal")));
  GrassLayer = TSoftObjectPtr<ULandscapeLayerInfoObject>(FSoftObjectPath(
      Pack +
      TEXT("/maps/Landscape_Layer_Info/L2_Snow_or_Grass_LayerInfo.L2_Snow_or_Grass_LayerInfo")));
  RockLayer = TSoftObjectPtr<ULandscapeLayerInfoObject>(FSoftObjectPath(
      Pack + TEXT("/maps/Landscape_Layer_Info/"
                  "L3_Snow_Rough_or_Rocks_LayerInfo.L3_Snow_Rough_or_Rocks_LayerInfo")));
  MudLayer = TSoftObjectPtr<ULandscapeLayerInfoObject>(FSoftObjectPath(
      Pack + TEXT("/maps/Landscape_Layer_Info/"
                  "L4_Snow_Slush_or_Mud_LayerInfo.L4_Snow_Slush_or_Mud_LayerInfo")));
  ForestGroundALayer = TSoftObjectPtr<ULandscapeLayerInfoObject>(FSoftObjectPath(
      Pack +
      TEXT(
          "/maps/Landscape_Layer_Info/L5_Forest_Ground_a_LayerInfo.L5_Forest_Ground_a_LayerInfo")));
  ForestGroundBLayer = TSoftObjectPtr<ULandscapeLayerInfoObject>(FSoftObjectPath(
      Pack +
      TEXT(
          "/maps/Landscape_Layer_Info/L6_Forest_Ground_b_LayerInfo.L6_Forest_Ground_b_LayerInfo")));
  const auto Mesh = [&Pack](const TCHAR* Path) {
    return TSoftObjectPtr<UStaticMesh>(FSoftObjectPath(Pack + Path));
  };
  TreeMeshes = {
      Mesh(TEXT("/pnp_v2_assets/Assets/pine_trees_full/pine_tree_full_a.pine_tree_full_a")),
      Mesh(TEXT(
          "/pnp_v2_assets/Assets/pine_trees_full/pine_tree_tall_full_b.pine_tree_tall_full_b")),
      Mesh(TEXT(
          "/pnp_v2_assets/Assets/pine_trees_full/pine_tree_tall_full_a.pine_tree_tall_full_a")),
  };
  RockMeshes = {
      Mesh(TEXT("/pnp_v2_assets/Assets/rocks/rock_a/rock_a.rock_a")),
  };
  WindCollection = TSoftObjectPtr<UMaterialParameterCollection>(FSoftObjectPath(
      Pack + TEXT("/pnp_v2_assets/Materials/wind_function/global_wind.global_wind")));
  SnowCollection = TSoftObjectPtr<UMaterialParameterCollection>(FSoftObjectPath(
      Pack + TEXT("/pnp_v2_assets/Materials/snow/master_snow_adjustment.master_snow_adjustment")));
}
