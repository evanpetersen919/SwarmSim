#include "SwarmObstaclePlan.h"

#include "SwarmCoordinates.h"

namespace SwarmSim {
namespace {
// Hash of a position and a seed to [0, 1).
double HashUnit(double X, double Y, uint64 Seed) {
  uint64 H = Seed;
  H ^= static_cast<uint64>(static_cast<int64>(X * 1000.0)) * 0x9E3779B97F4A7C15ULL;
  H ^= static_cast<uint64>(static_cast<int64>(Y * 1000.0)) * 0xC2B2AE3D27D4EB4FULL;
  H ^= H >> 33;
  H *= 0xFF51AFD7ED558CCDULL;
  H ^= H >> 33;
  H *= 0xC4CEB9FE1A85EC53ULL;
  H ^= H >> 33;
  return static_cast<double>(H >> 11) / 9007199254740992.0;
}

int32 ClosestHeight(const TArray<FMeshFit>& Meshes, double HeightCm) {
  int32 Best = 0;
  double BestDifference = TNumericLimits<double>::Max();
  for (int32 I = 0; I < Meshes.Num(); ++I) {
    const double Difference = FMath::Abs(2.0 * Meshes[I].Extent.Z - HeightCm);
    if (Difference < BestDifference) {
      BestDifference = Difference;
      Best = I;
    }
  }
  return Best;
}
}  // namespace

void PlanObstacles(const swarm::SceneData& Scene, const TArray<FMeshFit>& Trees,
                   const TArray<FMeshFit>& Rocks, uint64 Seed, FObstaclePlan& OutPlan) {
  OutPlan = FObstaclePlan();
  OutPlan.TreeInstances.SetNum(Trees.Num());
  OutPlan.RockInstances.SetNum(Rocks.Num());

  for (const swarm::Obstacle& O : Scene.obstacles) {
    const double BaseCm = O.base_z * MetersToCentimeters;
    const double HeightCm = O.height * MetersToCentimeters;
    const FVector Center = ToUnreal(O.cx, O.cy, 0.0);

    if (O.kind == swarm::ObstacleKind::kCylinder) {
      if (Trees.IsEmpty()) {
        continue;
      }
      const int32 Index = ClosestHeight(Trees, HeightCm);
      const FMeshFit& Fit = Trees[Index];
      const double MeshHeight = 2.0 * Fit.Extent.Z;
      FVector Scale(FMath::Clamp(HeightCm / MeshHeight, MinTreeScale, MaxTreeScale));
      if (Fit.bFitTreeRadius) {
        Scale = FVector(O.half_x * MetersToCentimeters / Fit.Extent.X,
                        O.half_x * MetersToCentimeters / Fit.Extent.Y, HeightCm / MeshHeight);
      }
      const double Yaw = 360.0 * HashUnit(O.cx, O.cy, Seed);
      const FRotator Rotation(0.0, Yaw, 0.0);
      // Place the mesh so that the middle of its base is at the cylinder's base: the base point
      // in the mesh's own frame is (Origin.X, Origin.Y, Origin.Z - Extent.Z).
      const FVector BaseLocal(Fit.Origin.X, Fit.Origin.Y, Fit.Origin.Z - Fit.Extent.Z);
      FTransform Transform(Rotation, FVector::ZeroVector, Scale);
      const FVector Rotated = Transform.TransformPosition(BaseLocal);
      Transform.SetLocation(FVector(Center.X, Center.Y, BaseCm) - Rotated);
      OutPlan.TreeInstances[Index].Add(Transform);
      ++OutPlan.TreeCount;
    } else {
      if (Rocks.IsEmpty()) {
        continue;
      }
      // Rocks are chosen by position, so a field of boxes does not repeat one mesh.
      const int32 Index = FMath::Min(
          static_cast<int32>(HashUnit(O.cx, O.cy, Seed ^ 0x51) * Rocks.Num()), Rocks.Num() - 1);
      const FMeshFit& Fit = Rocks[Index];
      // A boulder stretched to a tall box looks like a standing stone. A box taller than
      // MaxRockStretch times the width of its footprint is made of boulders stacked up to its
      // height instead, each squashed by the same amount and together filling the box.
      const double ScaleX = O.half_x * MetersToCentimeters / Fit.Extent.X;
      const double ScaleY = O.half_y * MetersToCentimeters / Fit.Extent.Y;
      const double FullScaleZ = HeightCm / (2.0 * Fit.Extent.Z);
      const double Stretch = FullScaleZ / FMath::Max(ScaleX, ScaleY);
      const int32 Pieces = FMath::Max(1, FMath::CeilToInt(Stretch / MaxRockStretch));
      const double PieceHeightCm = HeightCm / Pieces;
      const FVector Scale(ScaleX, ScaleY, FullScaleZ / Pieces);
      // The mesh bounds, scaled, must span the box: put the scaled bounds center at the box
      // center in x and y and its bottom at the base of the piece.
      const FVector Local(Fit.Origin.X, Fit.Origin.Y, Fit.Origin.Z - Fit.Extent.Z);
      const FVector Scaled = Local * Scale;
      for (int32 Piece = 0; Piece < Pieces; ++Piece) {
        const FVector Location(Center.X - Scaled.X, Center.Y - Scaled.Y,
                               BaseCm + Piece * PieceHeightCm - Scaled.Z);
        OutPlan.RockInstances[Index].Add(FTransform(FRotator::ZeroRotator, Location, Scale));
      }
      ++OutPlan.RockCount;
    }
  }
}
}  // namespace SwarmSim
