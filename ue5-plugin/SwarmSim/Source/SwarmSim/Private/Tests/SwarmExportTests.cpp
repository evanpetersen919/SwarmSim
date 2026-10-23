#include "Misc/AutomationTest.h"
#include "SwarmExportPlan.h"
#include "SwarmSceneExport.h"

#if WITH_EDITOR
#include "Editor.h"
#include "Engine/World.h"
#include "SwarmScene.h"
#include "SwarmSceneBuilder.h"
#include "Tests/AutomationEditorCommon.h"
#endif

using namespace SwarmSim;

namespace {
FExportOptions TestOptions() {
  FExportOptions Options;
  Options.TreeNames = {TEXT("Pine"), TEXT("Birch")};
  Options.RockNames = {TEXT("Rock")};
  return Options;
}
}  // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSwarmExportPlanTest, "SwarmSim.Export.Plan",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::ClientContext |
                                     EAutomationTestFlags::EngineFilter)
bool FSwarmExportPlanTest::RunTest(const FString& Parameters) {
  // Command line.
  const FExportOptions Parsed = ParseExportOptions(
      TEXT("-SwarmExportScene=C:/x/a.swscene -SwarmAnchor=81187,-14778 -SwarmExportSize=600 "
           "-SwarmExportCell=4 -SwarmExportTrees=Pine,Birch -SwarmExportRocks=Rock "
           "-SwarmExportMinTree=5 -SwarmExportWait=10"));
  TestTrue(TEXT("the options are valid"), Parsed.bValid);
  TestEqual(TEXT("path"), Parsed.OutPath, FString(TEXT("C:/x/a.swscene")));
  TestTrue(TEXT("no audit by default"), Parsed.AuditPath.IsEmpty());
  {
    const FExportOptions Skipped =
        ParseExportOptions(TEXT("-SwarmExportScene=a -SwarmExportSkipActors=Background,Preview"));
    TestEqual(TEXT("two actor names to skip"), Skipped.SkipActors.Num(), 2);
    TestTrue(TEXT("an actor is matched by part of its name"),
             NameMatchesAny(TEXT("PCGDemo_BackgroundBP_C_UAID_1"), Skipped.SkipActors));
    TestFalse(TEXT("another actor is not"),
              NameMatchesAny(TEXT("PCGPartitionActor_51200_1_0_0"), Skipped.SkipActors));
    TestTrue(TEXT("none are skipped by default"), Parsed.SkipActors.IsEmpty());
  }
  TestEqual(TEXT("audit path"),
            ParseExportOptions(TEXT("-SwarmAuditScene=C:/x/audit.csv -SwarmAnchor=1,2")).AuditPath,
            FString(TEXT("C:/x/audit.csv")));
  TestEqual(TEXT("anchor x"), Parsed.AnchorCm.X, 81187.0);
  TestEqual(TEXT("anchor y"), Parsed.AnchorCm.Y, -14778.0);
  TestEqual(TEXT("size"), Parsed.SizeM, 600.0);
  TestEqual(TEXT("cell"), Parsed.CellM, 4.0);
  TestEqual(TEXT("minimum tree"), Parsed.MinTreeHeightM, 5.0);
  TestEqual(TEXT("wait"), Parsed.WaitSeconds, 10.0);
  TestEqual(TEXT("two tree names"), Parsed.TreeNames.Num(), 2);
  TestEqual(TEXT("names are trimmed"), Parsed.TreeNames.Last(), FString(TEXT("Birch")));
  TestEqual(TEXT("one rock name"), Parsed.RockNames.Num(), 1);
  const FExportOptions Grounded = ParseExportOptions(
      TEXT("-SwarmExportScene=a -SwarmExportGround=Cliff,Terrain -SwarmExportMaxGap=0.3"));
  TestEqual(TEXT("two ground names"), Grounded.GroundNames.Num(), 2);
  TestEqual(TEXT("the allowed share of gaps"), Grounded.MaxGapFraction, 0.3);
  TestEqual(TEXT("one percent of gaps by default"), Parsed.MaxGapFraction, 0.01);
  TestTrue(TEXT("a name contains a pattern, ignoring case"),
           NameMatchesAny(TEXT("SM_cliff_08"), Grounded.GroundNames));
  TestFalse(TEXT("a name without one"), NameMatchesAny(TEXT("SM_Fern_01"), Grounded.GroundNames));
  TestFalse(TEXT("no patterns match nothing"), NameMatchesAny(TEXT("SM_Cliff"), {}));
  TestTrue(TEXT("defaults when absent"),
           ParseExportOptions(TEXT("-SwarmExportScene=a")).SizeM == 1000.0);
  for (const TCHAR* Bad : {TEXT("-SwarmExportScene=a -SwarmExportSize=5"),
                           TEXT("-SwarmExportScene=a -SwarmExportCell=zero"),
                           TEXT("-SwarmExportScene=a -SwarmAnchor=1")}) {
    TestFalse(FString::Printf(TEXT("rejected: %s"), Bad), ParseExportOptions(Bad).bValid);
  }

  // Names.
  const FExportOptions Options = TestOptions();
  TestTrue(TEXT("a pine is a tree"), ClassifyMesh(TEXT("SM_pine_tree_03"), Options.TreeNames,
                                                  Options.RockNames) == EObstacleClass::Tree);
  TestTrue(TEXT("a rock is a rock"), ClassifyMesh(TEXT("SM_Rock_B"), Options.TreeNames,
                                                  Options.RockNames) == EObstacleClass::Rock);
  TestTrue(
      TEXT("anything else is neither"),
      ClassifyMesh(TEXT("SM_Bush"), Options.TreeNames, Options.RockNames) == EObstacleClass::None);
  TestTrue(TEXT("no names is none"), ClassifyMesh(TEXT("SM_pine"), {}, {}) == EObstacleClass::None);

  // Grid.
  TestEqual(TEXT("cells round up"), CellsFor(1000.0, 8.0), 125);
  TestEqual(TEXT("cells round up from a remainder"), CellsFor(1001.0, 8.0), 126);
  TestEqual(TEXT("at least one cell"), CellsFor(1.0, 100.0), 1);
  TestEqual(TEXT("at most the format's limit"), CellsFor(1.0e9, 1.0),
            static_cast<int32>(swarm::kMaxSceneCellsPerAxis));
  // Written out by hand: sample (2, 3) of a 10 by 10 grid of 8 m cells is at simulation
  // x = -40 + 16 = -24, y = -40 + 24 = -16, which is Unreal X = anchor + (-2400) and Y = anchor
  // + 1600.
  FExportOptions Anchored;
  Anchored.AnchorCm = FVector2D(1000.0, 2000.0);
  Anchored.CellM = 8.0;
  const FVector2D Point = GridPointCm(Anchored, 10, 2, 3);
  TestEqual(TEXT("grid point x"), Point.X, 1000.0 - 2400.0);
  TestEqual(TEXT("grid point y, mirrored"), Point.Y, 2000.0 + 1600.0);

  // Filling holes.
  TArray<double> Heights = {1.0, 0.0, 3.0, 0.0, 0.0, 0.0, 7.0, 0.0, 9.0};
  TArray<bool> Known = {true, false, true, false, false, false, true, false, true};
  TestEqual(TEXT("five filled"), FillMissingHeights(Heights, Known, 3, 3), 5);
  TestEqual(TEXT("the first hole is the mean of its known neighbors"), Heights[1], 2.0);
  TestEqual(TEXT("and so is the second"), Heights[3], 4.0);
  for (const bool K : Known) {
    TestTrue(TEXT("every sample has a height"), K);
  }
  TArray<double> NoHeights = {0.0, 0.0, 0.0, 0.0};
  TArray<bool> NoneKnown = {false, false, false, false};
  TestEqual(TEXT("nothing to fill from"), FillMissingHeights(NoHeights, NoneKnown, 2, 2), -1);

  // Obstacles. A tree 10 m tall and 8 m wide, its base 2 m up, centered at level (anchor + 500,
  // anchor - 300) cm, i.e. simulation (5, 3).
  Anchored.AnchorCm = FVector2D(1000.0, 2000.0);
  Anchored.TreeNames = Options.TreeNames;
  swarm::Obstacle Tree;
  const FBox TreeBounds(FVector(1500.0 - 400.0, 1700.0 - 400.0, 200.0),
                        FVector(1500.0 + 400.0, 1700.0 + 400.0, 1200.0));
  TestTrue(TEXT("a tree"), ObstacleFromBounds(EObstacleClass::Tree, TreeBounds, Anchored, Tree));
  TestTrue(TEXT("a cylinder"), Tree.kind == swarm::ObstacleKind::kCylinder);
  TestEqual(TEXT("x"), Tree.cx, 5.0);
  TestEqual(TEXT("y, mirrored"), Tree.cy, 3.0);
  TestEqual(TEXT("base"), Tree.base_z, 2.0);
  TestEqual(TEXT("height"), Tree.height, 10.0);
  TestEqual(TEXT("radius is a fraction of the half width"), Tree.half_x, 0.15 * 4.0);
  // A crown is a second cylinder, only when asked for: from 35 percent of the height to the top,
  // with a radius of the given fraction of the half width of the bounds.
  swarm::Obstacle Crown;
  TestFalse(TEXT("no crown by default"), CrownFromBounds(TreeBounds, Anchored, Crown));
  FExportOptions WithCrown = Anchored;
  WithCrown.CrownFraction = 0.75;
  TestTrue(TEXT("a crown"), CrownFromBounds(TreeBounds, WithCrown, Crown));
  TestTrue(TEXT("a cylinder"), Crown.kind == swarm::ObstacleKind::kCylinder);
  TestEqual(TEXT("crown x"), Crown.cx, Tree.cx);
  TestEqual(TEXT("crown y"), Crown.cy, Tree.cy);
  TestEqual(TEXT("crown base: 35 percent up the tree"), Crown.base_z, 2.0 + 0.35 * 10.0);
  TestEqual(TEXT("crown height: the rest"), Crown.height, 0.65 * 10.0);
  TestEqual(TEXT("crown radius"), Crown.half_x, 0.75 * 4.0);
  swarm::Obstacle WideCrown;
  TestTrue(TEXT("a very wide crown"),
           CrownFromBounds(FBox(FVector(0.0, 0.0, 0.0), FVector(8000.0, 8000.0, 3000.0)), WithCrown,
                           WideCrown));
  TestEqual(TEXT("the crown radius is limited"), WideCrown.half_x, 15.0);
  swarm::Obstacle NoCrown;
  TestFalse(TEXT("a bush has no crown"),
            CrownFromBounds(FBox(FVector(0.0, 0.0, 0.0), FVector(100.0, 100.0, 200.0)), WithCrown,
                            NoCrown));
  TestTrue(TEXT("the options are read"),
           ParseExportOptions(
               TEXT("-SwarmExportScene=a -SwarmExportCrown=0.6 -SwarmExportCrownBase=0.5"))
                       .CrownFraction == 0.6 &&
               ParseExportOptions(
                   TEXT("-SwarmExportScene=a -SwarmExportCrown=0.6 -SwarmExportCrownBase=0.5"))
                       .CrownBaseFraction == 0.5);
  TestFalse(TEXT("a crown larger than the mesh is refused"),
            ParseExportOptions(TEXT("-SwarmExportScene=a -SwarmExportCrown=1.5")).bValid);
  swarm::Obstacle Small;
  TestFalse(TEXT("a bush is not a tree"),
            ObstacleFromBounds(EObstacleClass::Tree,
                               FBox(FVector(0.0, 0.0, 0.0), FVector(100.0, 100.0, 200.0)), Anchored,
                               Small));
  swarm::Obstacle Thick;
  TestTrue(TEXT("a very wide tree"),
           ObstacleFromBounds(EObstacleClass::Tree,
                              FBox(FVector(0.0, 0.0, 0.0), FVector(6000.0, 6000.0, 3000.0)),
                              Anchored, Thick));
  TestEqual(TEXT("the radius is limited"), Thick.half_x, 1.5);
  swarm::Obstacle Rock;
  TestTrue(TEXT("a rock"),
           ObstacleFromBounds(EObstacleClass::Rock,
                              FBox(FVector(0.0, 0.0, 0.0), FVector(400.0, 200.0, 300.0)), Anchored,
                              Rock));
  TestTrue(TEXT("a box"), Rock.kind == swarm::ObstacleKind::kBox);
  TestEqual(TEXT("half x"), Rock.half_x, 2.0);
  TestEqual(TEXT("half y"), Rock.half_y, 1.0);
  TestEqual(TEXT("height"), Rock.height, 3.0);
  TestFalse(TEXT("a cliff is too big for a rock"),
            ObstacleFromBounds(EObstacleClass::Rock,
                               FBox(FVector(0.0, 0.0, 0.0), FVector(20000.0, 3000.0, 3000.0)),
                               Anchored, Small));
  TestFalse(TEXT("none is not an obstacle"),
            ObstacleFromBounds(EObstacleClass::None, TreeBounds, Anchored, Small));

  // The scene is the sampled heights and obstacles, and must be valid.
  FExportOptions Grid;
  Grid.CellM = 10.0;
  TArray<double> Flat;
  Flat.Init(4.0, 5 * 5);
  swarm::SceneData Scene;
  FString Error;
  TestTrue(TEXT("a valid scene"), AssembleScene(Grid, 4, Flat, {Tree}, Scene, Error));
  TestEqual(TEXT("origin"), Scene.terrain.origin_x, -20.0);
  TestEqual(TEXT("one obstacle"), static_cast<int32>(Scene.obstacles.size()), 1);
  TestFalse(TEXT("the wrong number of heights"), AssembleScene(Grid, 4, {1.0}, {}, Scene, Error));
  return true;
}

#if WITH_EDITOR
// A scene built into a level by the viewer is exported again and is the same scene: the
// heights where the Landscape has them, and a tree and a rock for each obstacle.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSwarmExportRoundTripTest, "SwarmSim.Export.RoundTrip",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)
bool FSwarmExportRoundTripTest::RunTest(const FString& Parameters) {
  UWorld* World = FAutomationEditorCommonUtils::CreateNewMap();
  if (!TestNotNull(TEXT("a test world"), World)) {
    return false;
  }
  swarm::SceneData Scene;
  Scene.terrain.cells_x = 40;
  Scene.terrain.cells_y = 40;
  Scene.terrain.cell_size = 8.0;
  Scene.terrain.origin_x = -160.0;
  Scene.terrain.origin_y = -160.0;
  Scene.terrain.heights.resize(41 * 41);
  for (int J = 0; J < 41; ++J) {
    for (int I = 0; I < 41; ++I) {
      Scene.terrain.heights[Scene.terrain.index(I, J)] =
          12.0 + 5.0 * FMath::Sin(0.15 * I) + 3.0 * FMath::Cos(0.2 * J);
    }
  }
  for (int I = 0; I < 5; ++I) {
    swarm::Obstacle Tree;
    Tree.kind = swarm::ObstacleKind::kCylinder;
    Tree.cx = -90.0 + 40.0 * I;
    Tree.cy = -50.0 + 17.0 * I;
    Tree.base_z = 14.0;
    Tree.height = 8.0 + I;
    Tree.half_x = 0.7;
    Scene.obstacles.push_back(Tree);
  }
  swarm::Obstacle Box;
  Box.kind = swarm::ObstacleKind::kBox;
  Box.cx = 30.0;
  Box.cy = 70.0;
  Box.base_z = 14.0;
  Box.height = 3.0;
  Box.half_x = 3.0;
  Box.half_y = 2.0;
  Scene.obstacles.push_back(Box);

  ASwarmScene* Actor = World->SpawnActor<ASwarmScene>();
  if (!TestNotNull(TEXT("a scene actor"), Actor)) {
    return false;
  }
  FString Error;
  FSceneAssets None;  // the engine's cone and cube stand in
  if (!TestTrue(FString::Printf(TEXT("the viewer builds the scene (%s)"), *Error),
                BuildSceneContents(Actor, Scene, None, Error))) {
    return false;
  }

  FExportOptions Options;
  Options.SizeM = 320.0;
  Options.CellM = 8.0;
  Options.TreeNames = {TEXT("Cone")};
  Options.RockNames = {TEXT("Cube")};
  swarm::SceneData Exported;
  FExportReport Report;
  if (!TestTrue(FString::Printf(TEXT("the exporter reads the level (%s)"), *Error),
                ExportSceneFromWorld(World, Options, Exported, Report, Error))) {
    return false;
  }
  TestEqual(TEXT("cells x"), static_cast<int32>(Exported.terrain.cells_x), 40);
  TestEqual(TEXT("origin"), Exported.terrain.origin_x, -160.0);
  TestEqual(TEXT("no sample without ground"), Report.HeightMisses, 0);

  double Worst = 0.0;
  for (size_t K = 0; K < Scene.terrain.heights.size(); ++K) {
    Worst = FMath::Max(Worst, FMath::Abs(Scene.terrain.heights[K] - Exported.terrain.heights[K]));
  }
  TestTrue(FString::Printf(TEXT("the ground heights agree (worst %.4f m)"), Worst), Worst < 0.05);

  TestEqual(TEXT("a tree for each cylinder"), Report.Trees, 5);
  TestTrue(TEXT("a rock for the box"), Report.Rocks >= 1);
  int32 Matched = 0;
  for (const swarm::Obstacle& Original : Scene.obstacles) {
    if (Original.kind != swarm::ObstacleKind::kCylinder) {
      continue;
    }
    for (const swarm::Obstacle& Found : Exported.obstacles) {
      if (Found.kind == swarm::ObstacleKind::kCylinder &&
          FMath::Abs(Found.cx - Original.cx) < 0.2 && FMath::Abs(Found.cy - Original.cy) < 0.2 &&
          FMath::Abs(Found.height - Original.height) < 0.2 &&
          FMath::Abs(Found.base_z - Original.base_z) < 0.2) {
        ++Matched;
        break;
      }
    }
  }
  TestEqual(TEXT("each tree is where its cylinder was, as tall, on the same base"), Matched, 5);
  return true;
}
#endif
