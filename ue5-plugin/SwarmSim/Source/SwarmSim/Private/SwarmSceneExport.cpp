#include "SwarmSceneExport.h"

#include "Components/InstancedStaticMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/HitResult.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "Landscape.h"
#include "LandscapeHeightfieldCollisionComponent.h"
#include "SwarmCoordinates.h"
#include "SwarmLog.h"

namespace SwarmSim {
namespace {
// Far above and below the ground of a level of this kind, 2 km each way.
constexpr double kTraceHalfLengthCm = 200000.0;

// The Landscape, or a static mesh whose name is one of the ground names.
bool IsGround(const FHitResult& Hit, const TArray<FString>& GroundNames) {
  if (Cast<ULandscapeHeightfieldCollisionComponent>(Hit.GetComponent()) != nullptr ||
      Cast<ALandscapeProxy>(Hit.GetActor()) != nullptr) {
    return true;
  }
  if (GroundNames.IsEmpty()) {
    return false;
  }
  const UStaticMeshComponent* Component = Cast<UStaticMeshComponent>(Hit.GetComponent());
  const UStaticMesh* Mesh = Component != nullptr ? Component->GetStaticMesh() : nullptr;
  return Mesh != nullptr && NameMatchesAny(Mesh->GetName(), GroundNames);
}

// The height of the ground at a point, in centimeters: the highest hit that is ground. A trace
// stops at the first thing that blocks it, a tree or a boulder above the ground, so when a hit is
// not ground the trace is started again just below it. Complex collision is used because terrain
// meshes of a level may have no simple collision.
bool GroundHeightCm(UWorld* World, const FExportOptions& Options, const FVector2D& PointCm,
                    double& OutZ) {
  constexpr int32 kMaxHitsPerColumn = 400;
  constexpr double kStepBelowHitCm = 1.0;
  FCollisionQueryParams Params(SCENE_QUERY_STAT(SwarmExport), /*bTraceComplex=*/true);
  double Top = kTraceHalfLengthCm;
  for (int32 N = 0; N < kMaxHitsPerColumn; ++N) {
    FHitResult Hit;
    const FVector Start(PointCm.X, PointCm.Y, Top);
    const FVector End(PointCm.X, PointCm.Y, -kTraceHalfLengthCm);
    if (!World->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility, Params)) {
      return false;
    }
    if (IsGround(Hit, Options.GroundNames)) {
      OutZ = Hit.ImpactPoint.Z;
      return true;
    }
    Top = Hit.ImpactPoint.Z - kStepBelowHitCm;
  }
  return false;
}

struct FSurveyEntry {
  int32 Count = 0;
  EObstacleClass Class = EObstacleClass::None;
};
}  // namespace

void FGroundSamples::Reset(int32 InCells) {
  Cells = InCells;
  const int32 N = Samples();
  HeightsM.Init(0.0, N * N);
  Known.Init(false, N * N);
  RowsDone = 0;
}

void SampleGroundRows(UWorld* World, const FExportOptions& Options, int32 RowCount,
                      FGroundSamples& Samples) {
  const int32 N = Samples.Samples();
  const int32 Last = FMath::Min(Samples.RowsDone + RowCount, N);
  for (int32 J = Samples.RowsDone; J < Last; ++J) {
    for (int32 I = 0; I < N; ++I) {
      double Z = 0.0;
      if (GroundHeightCm(World, Options, GridPointCm(Options, Samples.Cells, I, J), Z)) {
        Samples.HeightsM[J * N + I] = Z / MetersToCentimeters;
        Samples.Known[J * N + I] = true;
      }
    }
  }
  Samples.RowsDone = Last;
}

bool FinishExport(UWorld* World, const FExportOptions& Options, FGroundSamples& Ground,
                  swarm::SceneData& OutScene, FExportReport& OutReport, FString& OutError) {
  if (World == nullptr) {
    OutError = TEXT("there is no world to export");
    return false;
  }
  OutReport = FExportReport();
  const int32 Cells = Ground.Cells;
  const int32 Samples = Ground.Samples();
  TArray<double>& HeightsM = Ground.HeightsM;

  OutReport.HeightSamples = Samples * Samples;
  {
    // Where the gaps are, as the share of samples with no ground in each of 4 by 4 blocks, so a
    // level that has streamed in only part of the area shows which part.
    const int32 Blocks = 4;
    TArray<int32> Missing;
    TArray<int32> Total;
    Missing.Init(0, Blocks * Blocks);
    Total.Init(0, Blocks * Blocks);
    for (int32 J = 0; J < Samples; ++J) {
      for (int32 I = 0; I < Samples; ++I) {
        const int32 Block = (J * Blocks / Samples) * Blocks + I * Blocks / Samples;
        ++Total[Block];
        Missing[Block] += Ground.Known[J * Samples + I] ? 0 : 1;
      }
    }
    for (int32 Row = 0; Row < Blocks; ++Row) {
      FString Line;
      for (int32 Col = 0; Col < Blocks; ++Col) {
        const int32 B = Row * Blocks + Col;
        Line += FString::Printf(TEXT(" %3d%%"), 100 * Missing[B] / FMath::Max(Total[B], 1));
      }
      UE_LOG(LogSwarmSim, Display, TEXT("export: no ground, rows from j = %d:%s"),
             Row * Samples / Blocks, *Line);
    }
  }
  const int32 Filled = FillMissingHeights(HeightsM, Ground.Known, Samples, Samples);
  if (Filled < 0) {
    OutError = TEXT(
        "no ground was found in the area: the Landscape is not loaded there, or has no "
        "collision");
    return false;
  }
  OutReport.HeightMisses = Filled;
  if (static_cast<double>(Filled) > Options.MaxGapFraction * OutReport.HeightSamples) {
    OutError = FString::Printf(
        TEXT("no ground was found at %d of %d samples: the level has not finished loading the "
             "area, the anchor is outside the Landscape, or the ground there is not a Landscape "
             "(see -SwarmExportMaxGap)"),
        Filled, OutReport.HeightSamples);
    return false;
  }
  OutReport.MinHeightM = HeightsM[0];
  OutReport.MaxHeightM = HeightsM[0];
  for (const double H : HeightsM) {
    OutReport.MinHeightM = FMath::Min(OutReport.MinHeightM, H);
    OutReport.MaxHeightM = FMath::Max(OutReport.MaxHeightM, H);
  }
  UE_LOG(LogSwarmSim, Display,
         TEXT("export: ground sampled, %d of %d samples had none, %.1f to %.1f m"), Filled,
         OutReport.HeightSamples, OutReport.MinHeightM, OutReport.MaxHeightM);

  // Obstacles.
  const double HalfCm = 0.5 * static_cast<double>(Cells) * Options.CellM * MetersToCentimeters;
  const FBox Area(FVector(Options.AnchorCm.X - HalfCm, Options.AnchorCm.Y - HalfCm, -1.0e9),
                  FVector(Options.AnchorCm.X + HalfCm, Options.AnchorCm.Y + HalfCm, 1.0e9));
  TArray<swarm::Obstacle> Obstacles;
  TMap<FString, FSurveyEntry> Survey;
  int32 ComponentsConsidered = 0;
  int64 InstancesInArea = 0;
  for (TActorIterator<AActor> It(World); It; ++It) {
    TInlineComponentArray<UStaticMeshComponent*> Components;
    It->GetComponents(Components);
    for (const UStaticMeshComponent* Component : Components) {
      const UStaticMesh* Mesh = Component->GetStaticMesh();
      if (Mesh == nullptr || !Component->Bounds.GetBox().Intersect(Area)) {
        continue;
      }
      ++ComponentsConsidered;
      // The name, the class, and the survey entry are looked up once for the component, not for
      // each of its instances: a level's instanced components can hold millions.
      const FString MeshName = Mesh->GetName();
      const EObstacleClass Kind = ClassifyMesh(MeshName, Options.TreeNames, Options.RockNames);
      const FBox LocalBounds = Mesh->GetBounds().GetBox();
      int32 InArea = 0;
      const auto Consider = [&](const FTransform& Transform) {
        const FVector Location = Transform.GetLocation();
        if (Location.X < Area.Min.X || Location.X > Area.Max.X || Location.Y < Area.Min.Y ||
            Location.Y > Area.Max.Y) {
          return;
        }
        ++InArea;
        if (Kind == EObstacleClass::None) {
          return;
        }
        swarm::Obstacle Obstacle;
        if (ObstacleFromBounds(Kind, LocalBounds.TransformBy(Transform), Options, Obstacle)) {
          Obstacles.Add(Obstacle);
          (Kind == EObstacleClass::Tree ? OutReport.Trees : OutReport.Rocks) += 1;
        }
      };
      if (const UInstancedStaticMeshComponent* Instanced =
              Cast<UInstancedStaticMeshComponent>(Component)) {
        for (int32 Index = 0; Index < Instanced->GetInstanceCount(); ++Index) {
          FTransform Transform;
          if (Instanced->GetInstanceTransform(Index, Transform, /*bWorldSpace=*/true)) {
            Consider(Transform);
          }
        }
      } else {
        Consider(Component->GetComponentTransform());
      }
      if (InArea > 0) {
        FSurveyEntry& Entry = Survey.FindOrAdd(MeshName);
        Entry.Class = Kind;
        Entry.Count += InArea;
        InstancesInArea += InArea;
      }
    }
  }
  UE_LOG(LogSwarmSim, Display,
         TEXT("export: %d static mesh components touch the area, %lld instances in it"),
         ComponentsConsidered, InstancesInArea);

  Survey.ValueSort([](const FSurveyEntry& A, const FSurveyEntry& B) { return A.Count > B.Count; });
  for (const TPair<FString, FSurveyEntry>& Pair : Survey) {
    const TCHAR* Class = Pair.Value.Class == EObstacleClass::Tree   ? TEXT("tree")
                         : Pair.Value.Class == EObstacleClass::Rock ? TEXT("rock")
                                                                    : TEXT("not exported");
    OutReport.Survey.Add(FString::Printf(TEXT("%s: %d (%s)"), *Pair.Key, Pair.Value.Count, Class));
  }
  if (static_cast<uint64>(Obstacles.Num()) > swarm::kMaxSceneObstacles) {
    OutError = TEXT("too many obstacles for a scene file");
    return false;
  }
  return AssembleScene(Options, Cells, HeightsM, Obstacles, OutScene, OutError);
}

bool ExportSceneFromWorld(UWorld* World, const FExportOptions& Options, swarm::SceneData& OutScene,
                          FExportReport& OutReport, FString& OutError) {
  if (World == nullptr) {
    OutError = TEXT("there is no world to export");
    return false;
  }
  FGroundSamples Ground;
  Ground.Reset(CellsFor(Options.SizeM, Options.CellM));
  SampleGroundRows(World, Options, Ground.Samples(), Ground);
  return FinishExport(World, Options, Ground, OutScene, OutReport, OutError);
}
}  // namespace SwarmSim
