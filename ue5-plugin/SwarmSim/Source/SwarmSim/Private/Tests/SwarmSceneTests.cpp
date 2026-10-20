// Automation tests for the obstacles and the scene actor. See SwarmTerrainTests.cpp for how to
// run them.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "SwarmCoordinates.h"
#include "SwarmObstaclePlan.h"
#include "SwarmScene.h"
#include "SwarmSceneBuilder.h"

#if WITH_EDITOR
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Editor.h"
#include "Engine/World.h"
#include "Materials/MaterialParameterCollection.h"
#include "Materials/MaterialParameterCollectionInstance.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "SwarmSimSettings.h"
#include "Tests/AutomationEditorCommon.h"
#endif

using namespace SwarmSim;

namespace {
swarm::Obstacle Cylinder(double X, double Y, double Base, double Height, double Radius) {
  swarm::Obstacle O;
  O.kind = swarm::ObstacleKind::kCylinder;
  O.cx = X;
  O.cy = Y;
  O.base_z = Base;
  O.height = Height;
  O.half_x = Radius;
  return O;
}

swarm::Obstacle Box(double X, double Y, double Base, double Height, double HalfX, double HalfY) {
  swarm::Obstacle O;
  O.kind = swarm::ObstacleKind::kBox;
  O.cx = X;
  O.cy = Y;
  O.base_z = Base;
  O.height = Height;
  O.half_x = HalfX;
  O.half_y = HalfY;
  return O;
}

swarm::SceneData SceneWith(const TArray<swarm::Obstacle>& Obstacles) {
  swarm::SceneData Scene;
  swarm::Heightfield& T = Scene.terrain;
  T.cells_x = 60;
  T.cells_y = 40;
  T.cell_size = 8.0;
  T.origin_x = -240.0;
  T.origin_y = -160.0;
  T.heights.assign(T.samples_x() * T.samples_y(), 1.5);
  for (const swarm::Obstacle& O : Obstacles) {
    Scene.obstacles.push_back(O);
  }
  return Scene;
}

// An off-center mesh, so that a fit that ignores the bounds' center fails.
FMeshFit Mesh(double Height, double Radius, FVector Origin = FVector(12.0, -7.0, 30.0)) {
  FMeshFit Fit;
  Fit.Origin = Origin;
  Fit.Extent = FVector(Radius, Radius * 0.8, Height * 0.5);
  return Fit;
}
}  // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSwarmObstaclePlanTest, "SwarmSim.Scene.ObstaclePlan",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)
bool FSwarmObstaclePlanTest::RunTest(const FString& Parameters) {
  TArray<swarm::Obstacle> Obstacles;
  Obstacles.Add(Cylinder(10.0, 20.0, 1.5, 12.0, 1.0));   // tall
  Obstacles.Add(Cylinder(-30.0, 5.0, -2.0, 5.0, 0.8));   // short
  Obstacles.Add(Box(40.0, -25.0, 0.5, 9.0, 3.0, 5.0));   // a rock
  Obstacles.Add(Box(-60.0, -50.0, 3.0, 4.0, 2.0, 2.0));  // another
  const swarm::SceneData Scene = SceneWith(Obstacles);

  // Two tree meshes of different heights (400 and 1300 cm) and two rock meshes.
  const TArray<FMeshFit> Trees = {Mesh(400.0, 150.0), Mesh(1300.0, 200.0)};
  const TArray<FMeshFit> Rocks = {Mesh(100.0, 120.0), Mesh(80.0, 90.0)};
  FObstaclePlan Plan;
  PlanObstacles(Scene, Trees, Rocks, 99, Plan);

  TestEqual(TEXT("one tree per cylinder"), Plan.TreeCount, 2);
  TestEqual(TEXT("one rock per box"), Plan.RockCount, 2);
  int32 TreeInstances = 0;
  for (const TArray<FTransform>& List : Plan.TreeInstances) {
    TreeInstances += List.Num();
  }
  int32 RockInstances = 0;
  for (const TArray<FTransform>& List : Plan.RockInstances) {
    RockInstances += List.Num();
  }
  TestEqual(TEXT("tree instances add up"), TreeInstances, 2);
  TestTrue(TEXT("a rock instance for every box, more for tall ones"), RockInstances >= 2);

  // The mesh whose height is closest is used: 1200 cm takes the 1300 cm mesh, 500 cm the 400 cm
  // one.
  TestEqual(TEXT("the tall cylinder takes the tall mesh"), Plan.TreeInstances[1].Num(), 1);
  TestEqual(TEXT("the short cylinder takes the short mesh"), Plan.TreeInstances[0].Num(), 1);

  // Each tree's base sits on its cylinder's base, at its x and y, and its top is at the cylinder's
  // height (when the scale is not clamped). The positions are written out here, not computed with
  // the conversion helpers: the simulation's y is Unreal's -Y.
  const FTransform& Tall = Plan.TreeInstances[1][0];
  const FMeshFit& TallFit = Trees[1];
  const FVector TallBase = Tall.TransformPosition(
      FVector(TallFit.Origin.X, TallFit.Origin.Y, TallFit.Origin.Z - TallFit.Extent.Z));
  TestTrue(TEXT("tall tree base x"), FMath::IsNearlyEqual(TallBase.X, 1000.0, 0.01));
  TestTrue(TEXT("tall tree base y"), FMath::IsNearlyEqual(TallBase.Y, -2000.0, 0.01));
  TestTrue(TEXT("tall tree base z"), FMath::IsNearlyEqual(TallBase.Z, 150.0, 0.01));
  const FVector TallTop = Tall.TransformPosition(
      FVector(TallFit.Origin.X, TallFit.Origin.Y, TallFit.Origin.Z + TallFit.Extent.Z));
  TestTrue(TEXT("tall tree height is the cylinder's"),
           FMath::IsNearlyEqual(TallTop.Z - TallBase.Z, 1200.0, 0.01));
  TestTrue(TEXT("trees are scaled uniformly"),
           Tall.GetScale3D().Equals(FVector(Tall.GetScale3D().X), 1e-6));

  const FTransform& Short = Plan.TreeInstances[0][0];
  const FMeshFit& ShortFit = Trees[0];
  const FVector ShortBase = Short.TransformPosition(
      FVector(ShortFit.Origin.X, ShortFit.Origin.Y, ShortFit.Origin.Z - ShortFit.Extent.Z));
  TestTrue(TEXT("short tree base"), ShortBase.Equals(FVector(-3000.0, -500.0, -200.0), 0.01));

  // The rocks of a box together fill it exactly. Box (40, -25) with half sizes 3 and 5, from z =
  // 0.5 m to 9.5 m, lies at X in [3700, 4300], Y in [2000, 3000] (y is mirrored), Z in [50, 950].
  // It is 9 m tall on a 6 m by 10 m footprint, which needs more than one boulder.
  int32 FirstBoxPieces = 0;
  double LowestZ = TNumericLimits<double>::Max();
  double HighestZ = TNumericLimits<double>::Lowest();
  for (int32 I = 0; I < Plan.RockInstances.Num(); ++I) {
    for (const FTransform& T : Plan.RockInstances[I]) {
      const FMeshFit& Fit = Rocks[I];
      const FVector Low = T.TransformPosition(Fit.Origin - Fit.Extent);
      const FVector High = T.TransformPosition(Fit.Origin + Fit.Extent);
      if (!FMath::IsNearlyEqual(Low.X, 3700.0, 0.01)) {
        continue;
      }
      ++FirstBoxPieces;
      TestTrue(TEXT("rock x range"), FMath::IsNearlyEqual(High.X, 4300.0, 0.01));
      TestTrue(TEXT("rock y range"), FMath::IsNearlyEqual(Low.Y, 2000.0, 0.01) &&
                                         FMath::IsNearlyEqual(High.Y, 3000.0, 0.01));
      TestTrue(TEXT("a rock is not turned"), T.GetRotation().Equals(FQuat::Identity, 1e-6));
      TestTrue(TEXT("no piece is stretched into a pillar"),
               T.GetScale3D().Z / FMath::Max(T.GetScale3D().X, T.GetScale3D().Y) <=
                   MaxRockStretch + 1e-9);
      LowestZ = FMath::Min(LowestZ, Low.Z);
      HighestZ = FMath::Max(HighestZ, High.Z);
    }
  }
  TestTrue(TEXT("a tall box is a stack of rocks"), FirstBoxPieces > 1);
  TestTrue(TEXT("the stack starts at the base of the box"),
           FMath::IsNearlyEqual(LowestZ, 50.0, 0.01));
  TestTrue(TEXT("and ends at its top"), FMath::IsNearlyEqual(HighestZ, 950.0, 0.01));

  // The plan depends on the scene and the seed only. Another seed turns trees and picks other rock
  // meshes but never moves a tree's base.
  FObstaclePlan Again;
  PlanObstacles(Scene, Trees, Rocks, 99, Again);
  TestTrue(TEXT("deterministic"),
           Again.TreeInstances[1][0].Equals(Plan.TreeInstances[1][0], 0.0) &&
               Again.TreeInstances[0][0].Equals(Plan.TreeInstances[0][0], 0.0));
  FObstaclePlan Other;
  PlanObstacles(Scene, Trees, Rocks, 100, Other);
  const FTransform& OtherTall = Other.TreeInstances[1][0];
  TestTrue(TEXT("another seed turns the tree"),
           !OtherTall.GetRotation().Equals(Tall.GetRotation(), 1e-6));
  const FVector OtherBase = OtherTall.TransformPosition(
      FVector(TallFit.Origin.X, TallFit.Origin.Y, TallFit.Origin.Z - TallFit.Extent.Z));
  TestTrue(TEXT("but not where it stands"), OtherBase.Equals(TallBase, 0.01));

  // A tree is clamped, not stretched without limit: a 20 m cylinder and a 400 cm mesh.
  swarm::SceneData Huge = SceneWith({Cylinder(0.0, 0.0, 0.0, 50.0, 1.0)});
  FObstaclePlan Clamped;
  PlanObstacles(Huge, {Mesh(400.0, 150.0)}, {}, 1, Clamped);
  TestTrue(TEXT("clamped scale"),
           FMath::IsNearlyEqual(Clamped.TreeInstances[0][0].GetScale3D().X, MaxTreeScale, 1e-9));

  // A stand-in cone is fitted to the cylinder's radius too.
  FMeshFit Cone = Mesh(100.0, 50.0, FVector::ZeroVector);
  Cone.Extent = FVector(50.0, 50.0, 50.0);
  Cone.bFitTreeRadius = true;
  FObstaclePlan Spike;
  PlanObstacles(SceneWith({Cylinder(0.0, 0.0, 0.0, 10.0, 1.5)}), {Cone}, {}, 1, Spike);
  const FVector Scale = Spike.TreeInstances[0][0].GetScale3D();
  TestTrue(TEXT("stand-in radius 1.5 m"), FMath::IsNearlyEqual(Scale.X, 3.0, 1e-9));
  TestTrue(TEXT("stand-in height 10 m"), FMath::IsNearlyEqual(Scale.Z, 10.0, 1e-9));

  // A kind with no mesh is left out, without a crash.
  FObstaclePlan NoTrees;
  PlanObstacles(Scene, {}, Rocks, 1, NoTrees);
  TestEqual(TEXT("no tree meshes, no trees"), NoTrees.TreeCount, 0);
  TestEqual(TEXT("rocks are still placed"), NoTrees.RockCount, 2);
  FObstaclePlan Nothing;
  PlanObstacles(Scene, {}, {}, 1, Nothing);
  TestEqual(TEXT("no meshes, nothing placed"), Nothing.TreeCount + Nothing.RockCount, 0);
  return true;
}

#if WITH_EDITOR
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSwarmSceneStandInTest, "SwarmSim.Scene.StandIns",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)
bool FSwarmSceneStandInTest::RunTest(const FString& Parameters) {
  UWorld* World = FAutomationEditorCommonUtils::CreateNewMap();
  if (!TestNotNull(TEXT("a test world"), World)) {
    return false;
  }
  TArray<swarm::Obstacle> Obstacles;
  for (int32 I = 0; I < 7; ++I) {
    Obstacles.Add(Cylinder(-100.0 + 25.0 * I, -40.0 + 9.0 * I, 1.5, 5.0 + I, 0.8));
  }
  for (int32 I = 0; I < 4; ++I) {
    Obstacles.Add(Box(-150.0 + 40.0 * I, 60.0, 1.5, 6.0 + I, 3.0, 2.0 + I));
  }
  const swarm::SceneData Scene = SceneWith(Obstacles);

  ASwarmScene* Actor = World->SpawnActor<ASwarmScene>();
  if (!TestNotNull(TEXT("a scene actor"), Actor)) {
    return false;
  }
  FString Error;
  FSceneAssets None;  // nothing installed: no meshes, no material, no collections
  TestTrue(FString::Printf(TEXT("the scene builds (%s)"), *Error),
           BuildSceneContents(Actor, Scene, None, Error));
  TestTrue(TEXT("the engine's shapes stand in"), Actor->bUsingStandIns);
  TestEqual(TEXT("a tree per cylinder"), Actor->TreeCount, 7);
  TestEqual(TEXT("a rock per box"), Actor->RockCount, 4);
  TestNotNull(TEXT("the terrain"), Actor->Terrain.Get());

  int32 Trees = 0;
  for (const UHierarchicalInstancedStaticMeshComponent* C : Actor->TreeComponents) {
    Trees += C->GetInstanceCount();
  }
  int32 Rocks = 0;
  for (const UHierarchicalInstancedStaticMeshComponent* C : Actor->RockComponents) {
    Rocks += C->GetInstanceCount();
  }
  TestEqual(TEXT("tree instances on the components"), Trees, 7);
  TestTrue(TEXT("rock instances on the components"), Rocks >= 4);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSwarmSceneAssetsTest, "SwarmSim.Scene.WithAssets",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)
bool FSwarmSceneAssetsTest::RunTest(const FString& Parameters) {
  const FSceneAssets Assets = LoadSceneAssets();
  if (Assets.TreeMeshes.IsEmpty() || Assets.RockMeshes.IsEmpty() ||
      Assets.WindCollection == nullptr) {
    AddInfo(TEXT("The assets named in the SwarmSim settings are not installed; skipped."));
    return true;
  }
  UWorld* World = FAutomationEditorCommonUtils::CreateNewMap();
  if (!TestNotNull(TEXT("a test world"), World)) {
    return false;
  }
  TArray<swarm::Obstacle> Obstacles;
  for (int32 I = 0; I < 12; ++I) {
    Obstacles.Add(Cylinder(-200.0 + 30.0 * I, -100.0 + 15.0 * I, 1.5, 4.0 + I, 1.0));
  }
  Obstacles.Add(Box(100.0, 100.0, 1.5, 12.0, 4.0, 3.0));
  const swarm::SceneData Scene = SceneWith(Obstacles);

  // Through the scene file, as the viewer will do it.
  std::vector<uint8_t> Serialized;
  TestEqual(TEXT("the scene serializes"),
            static_cast<int32>(swarm::serialize_scene(Scene, Serialized)),
            static_cast<int32>(swarm::SceneStatus::kOk));
  TArray<uint8> Bytes;
  Bytes.Append(Serialized.data(), static_cast<int32>(Serialized.size()));
  const FString Path = FPaths::ProjectSavedDir() / TEXT("SwarmTest/scene.swscene");
  TestTrue(TEXT("the scene file is written"), FFileHelper::SaveArrayToFile(Bytes, *Path));

  ASwarmScene* Actor = World->SpawnActor<ASwarmScene>();
  if (!TestNotNull(TEXT("a scene actor"), Actor)) {
    return false;
  }
  FString Error;
  TestTrue(FString::Printf(TEXT("the scene builds from its file (%s)"), *Error),
           Actor->BuildFromSceneFile(Path, Error));
  TestFalse(TEXT("the pack's meshes are used, not stand-ins"), Actor->bUsingStandIns);
  TestEqual(TEXT("a tree per cylinder"), Actor->TreeCount, 12);
  TestEqual(TEXT("a rock per box"), Actor->RockCount, 1);
  TestNotNull(TEXT("the terrain"), Actor->Terrain.Get());
  int32 Trees = 0;
  for (const UHierarchicalInstancedStaticMeshComponent* C : Actor->TreeComponents) {
    Trees += C->GetInstanceCount();
    TestNotNull(TEXT("a mesh on the tree component"), C->GetStaticMesh().Get());
  }
  TestEqual(TEXT("tree instances on the components"), Trees, 12);

  // The wind and snow reach the material parameter collections.
  UMaterialParameterCollectionInstance* Wind =
      World->GetParameterCollectionInstance(Assets.WindCollection);
  float Value = 0.0f;
  TestTrue(TEXT("wind intensity is readable"),
           Wind->GetScalarParameterValue(TEXT("Wind_Intensity"), Value));
  TestEqual(TEXT("wind intensity is the setting"), Value,
            GetDefault<USwarmSimSettings>()->WindIntensity);
  Actor->SetWind(7.5f, 3.0f, -2.0f, 4.0f);
  Wind->GetScalarParameterValue(TEXT("Wind_Intensity"), Value);
  TestEqual(TEXT("SetWind intensity"), Value, 7.5f);
  Wind->GetScalarParameterValue(TEXT("Wind_Y_Speed"), Value);
  TestEqual(TEXT("SetWind y speed"), Value, -2.0f);
  if (Assets.SnowCollection != nullptr) {
    Actor->SetSnowCoverage(0.6f);
    UMaterialParameterCollectionInstance* Snow =
        World->GetParameterCollectionInstance(Assets.SnowCollection);
    Snow->GetScalarParameterValue(TEXT("Snow_Coverage"), Value);
    TestEqual(TEXT("SetSnowCoverage"), Value, 0.6f);
    Actor->SetSnowCoverage(3.0f);
    Snow->GetScalarParameterValue(TEXT("Snow_Coverage"), Value);
    TestEqual(TEXT("snow is clamped to 1"), Value, 1.0f);
  }

  // A file that is not a scene is an error, not a crash.
  FString Bad;
  ASwarmScene* Second = World->SpawnActor<ASwarmScene>();
  TestFalse(TEXT("a missing file"),
            Second->BuildFromSceneFile(TEXT("does/not/exist.swscene"), Bad));
  TestFalse(TEXT("with a message"), Bad.IsEmpty());
  return true;
}
#endif

#endif  // WITH_DEV_AUTOMATION_TESTS
