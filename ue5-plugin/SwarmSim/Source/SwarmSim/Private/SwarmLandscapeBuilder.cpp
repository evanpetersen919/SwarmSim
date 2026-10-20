#include "SwarmLandscapeBuilder.h"

#include "Engine/World.h"
#include "HAL/PlatformFileManager.h"
#include "Logging/LogMacros.h"
#include "Misc/FileHelper.h"
#include "SwarmCoordinates.h"

#if WITH_EDITOR
#include "Landscape.h"
#include "LandscapeEdit.h"
#include "LandscapeInfo.h"
#include "LandscapeLayerInfoObject.h"
#include "LandscapeProxy.h"
#endif

namespace SwarmSim {
bool LoadSceneFile(const FString& Path, swarm::SceneData& OutScene, FString& OutError) {
  TArray<uint8> Bytes;
  if (!FFileHelper::LoadFileToArray(Bytes, *Path)) {
    OutError = FString::Printf(TEXT("cannot read %s"), *Path);
    return false;
  }
  const swarm::SceneStatus Status =
      swarm::parse_scene(Bytes.GetData(), static_cast<size_t>(Bytes.Num()), OutScene);
  if (Status != swarm::SceneStatus::kOk) {
    OutError = FString::Printf(TEXT("%s is not a valid scene file: %s"), *Path,
                               UTF8_TO_TCHAR(swarm::to_string(Status)));
    return false;
  }
  return true;
}

#if WITH_EDITOR
namespace {
FName LayerNameOf(const ULandscapeLayerInfoObject* Info) {
  return Info != nullptr ? Info->LayerName : NAME_None;
}
}  // namespace

ALandscape* BuildLandscape(UWorld* World, const swarm::SceneData& Scene, const FTerrainLook& Look,
                           FString& OutError) {
  if (World == nullptr) {
    OutError = TEXT("no world");
    return nullptr;
  }
  const swarm::Heightfield& Terrain = Scene.terrain;
  FLandscapeLayout Layout;
  if (!ChooseLandscapeLayout(Terrain.cells_x, Terrain.cells_y, Layout)) {
    OutError =
        FString::Printf(TEXT("a heightfield of %u x %u cells cannot be made into a Landscape"),
                        Terrain.cells_x, Terrain.cells_y);
    return nullptr;
  }

  TArray<uint16> Heights;
  float ScaleZ = 100.0f;
  EncodeLandscapeHeights(Terrain, Layout, Heights, ScaleZ);

  // Ground weights for the layers that were given.
  TArray<EGround> Present;
  for (const TPair<EGround, ULandscapeLayerInfoObject*>& Pair : Look.Layers) {
    if (Pair.Value != nullptr) {
      Present.Add(Pair.Key);
    }
  }
  Present.Sort([](EGround A, EGround B) { return static_cast<uint8>(A) < static_cast<uint8>(B); });
  TMap<EGround, TArray<uint8>> Weights;
  ComputeGroundWeights(Terrain, Layout, swarm::scene_hash(Scene), Look.Rules, Present, Weights);

  TArray<FLandscapeImportLayerInfo> ImportLayers;
  for (EGround Ground : Present) {
    FLandscapeImportLayerInfo Info(LayerNameOf(Look.Layers[Ground]));
    Info.LayerInfo = Look.Layers[Ground];
    Info.LayerData = MoveTemp(Weights[Ground]);
    ImportLayers.Add(MoveTemp(Info));
  }

  // The Landscape's vertex (0, 0) is the simulation point (origin_x, origin_y + extent_y).
  const FVector Origin = ToUnreal(Terrain.origin_x, Terrain.origin_y + Terrain.extent_y(), 0.0);
  const double CellCm = Terrain.cell_size * MetersToCentimeters;

  const FGuid LandscapeGuid = FGuid::NewGuid();
  ALandscape* Landscape = World->SpawnActor<ALandscape>(Origin, FRotator::ZeroRotator);
  if (Landscape == nullptr) {
    OutError = TEXT("the Landscape actor could not be spawned");
    return nullptr;
  }
  Landscape->SetActorRelativeScale3D(FVector(CellCm, CellCm, ScaleZ));
  // Import sets the Landscape's guid and sizes itself. Setting the guid first would register the
  // Landscape with the world while its component size is still zero, and the import then fails.
  if (Look.Material != nullptr) {
    Landscape->LandscapeMaterial = Look.Material;
  }
  Landscape->SetActorLabel(TEXT("SwarmTerrain"));

  TMap<FGuid, TArray<uint16>> HeightData;
  HeightData.Add(FGuid(), MoveTemp(Heights));
  TMap<FGuid, TArray<FLandscapeImportLayerInfo>> LayerData;
  LayerData.Add(FGuid(), MoveTemp(ImportLayers));

  Landscape->Import(LandscapeGuid, 0, 0, Layout.QuadsX(), Layout.QuadsY(), Layout.NumSubsections,
                    Layout.SubsectionSizeQuads, HeightData, nullptr, LayerData,
                    ELandscapeImportAlphamapType::Additive);

  if (ULandscapeInfo* Info = Landscape->GetLandscapeInfo()) {
    Info->UpdateLayerInfoMap(Landscape);
  }
  for (EGround Ground : Present) {
    Landscape->EditorLayerSettings.Add(FLandscapeEditorLayerSettings(Look.Layers[Ground]));
  }
  return Landscape;
}

double MaxLandscapeHeightErrorCm(ALandscape* Landscape, const swarm::SceneData& Scene) {
  if (Landscape == nullptr || Landscape->GetLandscapeInfo() == nullptr) {
    return -1.0;
  }
  const swarm::Heightfield& Terrain = Scene.terrain;
  FLandscapeLayout Layout;
  if (!ChooseLandscapeLayout(Terrain.cells_x, Terrain.cells_y, Layout)) {
    return -1.0;
  }
  double Worst = 0.0;

  // 1. The heights the Landscape stores, at every one of its vertices, against the heightfield,
  //    through the editor's data interface. This is exact and does not depend on collision.
  const int32 Nx = Layout.VerticesX();
  const int32 Ny = Layout.VerticesY();
  TArray<uint16> Stored;
  Stored.SetNumZeroed(Nx * Ny);
  FLandscapeEditDataInterface EditData(Landscape->GetLandscapeInfo());
  EditData.GetHeightDataFast(0, 0, Nx - 1, Ny - 1, Stored.GetData(), 0);
  const float ScaleZ = static_cast<float>(Landscape->GetActorScale3D().Z);
  for (int32 V = 0; V < Ny; ++V) {
    for (int32 U = 0; U < Nx; ++U) {
      int32 I = 0;
      int32 J = 0;
      LandscapeVertexToSample(Terrain, U, V, I, J);
      const double Expected = Terrain.heights[Terrain.index(I, J)] * MetersToCentimeters;
      Worst = FMath::Max(
          Worst, FMath::Abs(DecodeLandscapeHeightCm(Stored[V * Nx + U], ScaleZ) - Expected));
    }
  }

  // 2. Where the Landscape is in the world: the engine's own height query at the heightfield's
  //    samples, converted to world positions with the conversion in SwarmCoordinates.h. Samples
  //    on the Landscape's outer edge are left out, because the engine returns nothing, or a
  //    neighbouring height, for a point exactly on that edge.
  for (uint32 J = 0; J <= Terrain.cells_y; ++J) {
    for (uint32 I = 0; I <= Terrain.cells_x; ++I) {
      const bool OnFarEdgeX = static_cast<int32>(I) == Layout.QuadsX();
      const bool OnFarEdgeY = static_cast<int32>(Terrain.cells_y - J) == Layout.QuadsY();
      if (OnFarEdgeX || OnFarEdgeY) {
        continue;
      }
      const FVector Where = ToUnreal(Terrain.origin_x + I * Terrain.cell_size,
                                     Terrain.origin_y + J * Terrain.cell_size, 0.0);
      const TOptional<float> Height = Landscape->GetHeightAtLocation(Where);
      if (!Height.IsSet()) {
        return -1.0;
      }
      const double Expected = Terrain.heights[Terrain.index(I, J)] * MetersToCentimeters;
      Worst = FMath::Max(Worst, FMath::Abs(static_cast<double>(Height.GetValue()) - Expected));
    }
  }
  return Worst;
}
#endif
}  // namespace SwarmSim
