#include "SwarmExportPlan.h"

#include "Misc/DefaultValueHelper.h"
#include "Misc/Parse.h"
#include "SwarmCoordinates.h"

namespace SwarmSim {
namespace {
void ParseNames(const TCHAR* CommandLine, const TCHAR* Key, TArray<FString>& Out) {
  FString Text;
  if (FParse::Value(CommandLine, Key, Text, /*bShouldStopOnSeparator=*/false)) {
    Text.ParseIntoArray(Out, TEXT(","), /*InCullEmpty=*/true);
    for (FString& Name : Out) {
      Name.TrimStartAndEndInline();
    }
  }
}

}  // namespace

bool NameMatchesAny(const FString& Name, const TArray<FString>& Patterns) {
  for (const FString& Pattern : Patterns) {
    if (!Pattern.IsEmpty() && Name.Contains(Pattern, ESearchCase::IgnoreCase)) {
      return true;
    }
  }
  return false;
}

FExportOptions ParseExportOptions(const TCHAR* CommandLine) {
  FExportOptions Options;
  FParse::Value(CommandLine, TEXT("SwarmExportScene="), Options.OutPath);
  FParse::Value(CommandLine, TEXT("SwarmAuditScene="), Options.AuditPath);

  FString Text;
  if (FParse::Value(CommandLine, TEXT("SwarmAnchor="), Text, /*bShouldStopOnSeparator=*/false)) {
    FString X;
    FString Y;
    double Px = 0.0;
    double Py = 0.0;
    if (Text.Split(TEXT(","), &X, &Y) && FDefaultValueHelper::ParseDouble(X, Px) &&
        FDefaultValueHelper::ParseDouble(Y, Py)) {
      Options.AnchorCm = FVector2D(Px, Py);
    } else {
      Options.bValid = false;
      Options.Problem =
          FString::Printf(TEXT("-SwarmAnchor must be X,Y in centimeters, got '%s'"), *Text);
    }
  }
  const auto ReadNumber = [&](const TCHAR* Key, double Low, double High, double& Out) {
    FString Value;
    if (!FParse::Value(CommandLine, Key, Value)) {
      return;
    }
    double Number = 0.0;
    if (FDefaultValueHelper::ParseDouble(Value, Number) && Number >= Low && Number <= High) {
      Out = Number;
    } else {
      Options.bValid = false;
      Options.Problem = FString::Printf(TEXT("%s must be a number from %g to %g, got '%s'"), Key,
                                        Low, High, *Value);
    }
  };
  ReadNumber(TEXT("SwarmExportSize="), 10.0, 20000.0, Options.SizeM);
  ReadNumber(TEXT("SwarmExportCell="), 0.5, 200.0, Options.CellM);
  ReadNumber(TEXT("SwarmExportWait="), 0.0, 3600.0, Options.WaitSeconds);
  ReadNumber(TEXT("SwarmExportMinTree="), 0.0, 200.0, Options.MinTreeHeightM);
  ReadNumber(TEXT("SwarmExportMinRock="), 0.0, 200.0, Options.MinRockSizeM);
  ReadNumber(TEXT("SwarmExportMaxGap="), 0.0, 1.0, Options.MaxGapFraction);
  ReadNumber(TEXT("SwarmExportTrunk="), 0.01, 1.0, Options.TrunkFraction);
  ParseNames(CommandLine, TEXT("SwarmExportTrees="), Options.TreeNames);
  ParseNames(CommandLine, TEXT("SwarmExportRocks="), Options.RockNames);
  ParseNames(CommandLine, TEXT("SwarmExportGround="), Options.GroundNames);
  return Options;
}

EObstacleClass ClassifyMesh(const FString& MeshName, const TArray<FString>& TreeNames,
                            const TArray<FString>& RockNames) {
  if (NameMatchesAny(MeshName, TreeNames)) {
    return EObstacleClass::Tree;
  }
  if (NameMatchesAny(MeshName, RockNames)) {
    return EObstacleClass::Rock;
  }
  return EObstacleClass::None;
}

int32 CellsFor(double SizeM, double CellM) {
  if (!(CellM > 0.0) || !(SizeM > 0.0)) {
    return 1;
  }
  const double Cells = FMath::CeilToDouble(SizeM / CellM);
  return static_cast<int32>(
      FMath::Clamp(Cells, 1.0, static_cast<double>(swarm::kMaxSceneCellsPerAxis)));
}

FVector2D GridPointCm(const FExportOptions& Options, int32 Cells, int32 I, int32 J) {
  const double Origin = -0.5 * static_cast<double>(Cells) * Options.CellM;
  const FVector Point = ToUnreal(Origin + static_cast<double>(I) * Options.CellM,
                                 Origin + static_cast<double>(J) * Options.CellM, 0.0);
  return FVector2D(Options.AnchorCm.X + Point.X, Options.AnchorCm.Y + Point.Y);
}

int32 FillMissingHeights(TArray<double>& Heights, TArray<bool>& Known, int32 SamplesX,
                         int32 SamplesY) {
  const int32 Total = SamplesX * SamplesY;
  if (Heights.Num() != Total || Known.Num() != Total) {
    return -1;
  }
  int32 Missing = 0;
  for (bool K : Known) {
    Missing += K ? 0 : 1;
  }
  if (Missing == Total) {
    return -1;
  }
  const int32 Filled = Missing;
  while (Missing > 0) {
    const TArray<bool> Before = Known;
    TArray<double> Next = Heights;
    for (int32 J = 0; J < SamplesY; ++J) {
      for (int32 I = 0; I < SamplesX; ++I) {
        const int32 Index = J * SamplesX + I;
        if (Before[Index]) {
          continue;
        }
        double Sum = 0.0;
        int32 Count = 0;
        const int32 Neighbors[4][2] = {{I - 1, J}, {I + 1, J}, {I, J - 1}, {I, J + 1}};
        for (const int32* N : Neighbors) {
          if (N[0] >= 0 && N[0] < SamplesX && N[1] >= 0 && N[1] < SamplesY &&
              Before[N[1] * SamplesX + N[0]]) {
            Sum += Heights[N[1] * SamplesX + N[0]];
            ++Count;
          }
        }
        if (Count > 0) {
          Next[Index] = Sum / Count;
          Known[Index] = true;
          --Missing;
        }
      }
    }
    Heights = Next;
  }
  return Filled;
}

bool ObstacleFromBounds(EObstacleClass Kind, const FBox& BoundsCm, const FExportOptions& Options,
                        swarm::Obstacle& Out) {
  if (Kind == EObstacleClass::None || !BoundsCm.IsValid) {
    return false;
  }
  const FVector Size = BoundsCm.GetSize() / MetersToCentimeters;
  const FVector Center = BoundsCm.GetCenter();
  double Sim[3];
  ToSimulation(FVector(Center.X - Options.AnchorCm.X, Center.Y - Options.AnchorCm.Y, 0.0), Sim);
  Out.cx = Sim[0];
  Out.cy = Sim[1];
  Out.base_z = BoundsCm.Min.Z / MetersToCentimeters;
  Out.height = Size.Z;
  const double Width = FMath::Max(Size.X, Size.Y);
  if (Kind == EObstacleClass::Tree) {
    if (Size.Z < Options.MinTreeHeightM || !(Width > 0.0)) {
      return false;
    }
    Out.kind = swarm::ObstacleKind::kCylinder;
    Out.half_x = FMath::Clamp(Options.TrunkFraction * 0.5 * Width, Options.MinTrunkRadiusM,
                              Options.MaxTrunkRadiusM);
    Out.half_y = 0.0;
    return true;
  }
  const double Largest = FMath::Max(Width, Size.Z);
  if (Largest < Options.MinRockSizeM || Largest > Options.MaxRockSizeM || !(Size.Z > 0.0) ||
      !(Size.X > 0.0) || !(Size.Y > 0.0)) {
    return false;
  }
  Out.kind = swarm::ObstacleKind::kBox;
  Out.half_x = 0.5 * Size.X;
  Out.half_y = 0.5 * Size.Y;
  return true;
}

bool AssembleScene(const FExportOptions& Options, int32 Cells, const TArray<double>& HeightsM,
                   const TArray<swarm::Obstacle>& Obstacles, swarm::SceneData& Out,
                   FString& Error) {
  swarm::SceneData Scene;
  Scene.terrain.cells_x = static_cast<uint32>(Cells);
  Scene.terrain.cells_y = static_cast<uint32>(Cells);
  Scene.terrain.cell_size = Options.CellM;
  Scene.terrain.origin_x = -0.5 * static_cast<double>(Cells) * Options.CellM;
  Scene.terrain.origin_y = Scene.terrain.origin_x;
  Scene.terrain.heights.assign(HeightsM.GetData(), HeightsM.GetData() + HeightsM.Num());
  Scene.obstacles.assign(Obstacles.GetData(), Obstacles.GetData() + Obstacles.Num());
  std::string Reason;
  if (!swarm::validate(Scene, &Reason)) {
    Error = FString::Printf(TEXT("the exported scene is not valid: %hs"), Reason.c_str());
    return false;
  }
  Out = std::move(Scene);
  return true;
}
}  // namespace SwarmSim
