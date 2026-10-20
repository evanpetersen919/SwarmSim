// Automation tests for the terrain. Run them without a window with
//   UnrealEditor-Cmd.exe UnrealHost.uproject -ExecCmds="Automation RunTests SwarmSim; Quit"
//     -unattended -nullrhi -nosplash
// See docs/DEVELOPMENT.md.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "SwarmCoordinates.h"
#include "SwarmLandscapeBuilder.h"
#include "SwarmLandscapePlan.h"
#include "SwarmSimSettings.h"

#if WITH_EDITOR
#include "Editor.h"
#include "Landscape.h"
#include "LandscapeLayerInfoObject.h"
#include "Materials/MaterialInterface.h"
#include "Tests/AutomationEditorCommon.h"
#endif

using namespace SwarmSim;

namespace {
// A terrain with a known, irregular shape: smooth hills plus a per-sample term, so that no two
// samples are alike and a transposed or mirrored grid cannot pass by accident.
swarm::SceneData MakeScene(uint32 CellsX, uint32 CellsY, double Cell, double OriginX,
                           double OriginY) {
  swarm::SceneData Scene;
  swarm::Heightfield& T = Scene.terrain;
  T.cells_x = CellsX;
  T.cells_y = CellsY;
  T.cell_size = Cell;
  T.origin_x = OriginX;
  T.origin_y = OriginY;
  T.heights.resize(T.samples_x() * T.samples_y());
  for (uint32 J = 0; J <= CellsY; ++J) {
    for (uint32 I = 0; I <= CellsX; ++I) {
      const double Hills = 9.0 * FMath::Sin(0.21 * I) * FMath::Cos(0.17 * J);
      const double Grain = 0.05 * static_cast<double>((I * 7 + J * 13) % 11);
      T.heights[T.index(I, J)] = Hills + Grain;
    }
  }
  return Scene;
}
}  // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSwarmLayoutTest, "SwarmSim.Terrain.Layout",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)
bool FSwarmLayoutTest::RunTest(const FString& Parameters) {
  // A grid that is exactly one valid Landscape needs no surplus: 150 = 10 components of 15 quads.
  FLandscapeLayout Layout;
  TestTrue(TEXT("150 x 150 has a layout"), ChooseLandscapeLayout(150, 150, Layout));
  TestEqual(TEXT("150 x 150 has no surplus along x"), Layout.QuadsX(), 150);
  TestEqual(TEXT("150 x 150 has no surplus along y"), Layout.QuadsY(), 150);

  // Every layout covers its grid, uses an allowed subsection size, and stays within the limits.
  const uint32 Sizes[] = {1,   2,   7,   8,   15,  16,   61,   100,  127,
                          128, 150, 255, 256, 500, 1000, 2047, 4096, 8128};
  for (uint32 Cx : Sizes) {
    for (uint32 Cy : {1U, 61U, 150U, 513U}) {
      if (!ChooseLandscapeLayout(Cx, Cy, Layout)) {
        TestTrue(FString::Printf(TEXT("%u x %u is only rejected when too large"), Cx, Cy),
                 Cx > 8128 || Cy > 8128);
        continue;
      }
      TestTrue(TEXT("covers x"), Layout.QuadsX() >= static_cast<int32>(Cx));
      TestTrue(TEXT("covers y"), Layout.QuadsY() >= static_cast<int32>(Cy));
      TestTrue(TEXT("less than one component of surplus along x"),
               Layout.QuadsX() - static_cast<int32>(Cx) < Layout.ComponentQuads());
      TestTrue(TEXT("within the largest Landscape"),
               Layout.VerticesX() <= 8129 && Layout.VerticesY() <= 8129);
      const int32 Size = Layout.SubsectionSizeQuads;
      TestTrue(TEXT("allowed subsection size"),
               Size == 7 || Size == 15 || Size == 31 || Size == 63 || Size == 127 || Size == 255);
      TestTrue(TEXT("one or two subsections"),
               Layout.NumSubsections == 1 || Layout.NumSubsections == 2);
    }
  }
  TestFalse(TEXT("an empty grid has no layout"), ChooseLandscapeLayout(0, 10, Layout));
  TestFalse(TEXT("a grid larger than a Landscape has no layout"),
            ChooseLandscapeLayout(9000, 10, Layout));
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSwarmEncodingTest, "SwarmSim.Terrain.HeightEncoding",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)
bool FSwarmEncodingTest::RunTest(const FString& Parameters) {
  const swarm::SceneData Scene = MakeScene(61, 41, 8.0, -200.0, -150.0);
  FLandscapeLayout Layout;
  TestTrue(TEXT("layout"), ChooseLandscapeLayout(61, 41, Layout));
  TArray<uint16> Heights;
  float ScaleZ = 0.0f;
  EncodeLandscapeHeights(Scene.terrain, Layout, Heights, ScaleZ);
  TestEqual(TEXT("a normal terrain uses the base Z scale"), ScaleZ, 100.0f);
  TestEqual(TEXT("one value per vertex"), Heights.Num(), Layout.VerticesX() * Layout.VerticesY());

  double Worst = 0.0;
  for (int32 V = 0; V < Layout.VerticesY(); ++V) {
    for (int32 U = 0; U < Layout.VerticesX(); ++U) {
      int32 I = 0;
      int32 J = 0;
      LandscapeVertexToSample(Scene.terrain, U, V, I, J);
      const double Expected = Scene.terrain.heights[Scene.terrain.index(I, J)] * 100.0;
      Worst = FMath::Max(
          Worst, FMath::Abs(DecodeLandscapeHeightCm(Heights[V * Layout.VerticesX() + U], ScaleZ) -
                            Expected));
    }
  }
  TestTrue(FString::Printf(TEXT("quantization error %.4f cm is under 0.5 cm"), Worst), Worst < 0.5);

  // The orientation rule: v = 0 is the simulation's largest y, and the surplus repeats the edge.
  int32 I = 0;
  int32 J = 0;
  LandscapeVertexToSample(Scene.terrain, 0, 0, I, J);
  TestEqual(TEXT("v = 0 is the last row"), J, 41);
  LandscapeVertexToSample(Scene.terrain, 3, 41, I, J);
  TestEqual(TEXT("v = cells_y is the first row"), J, 0);
  TestEqual(TEXT("u is the column"), I, 3);
  LandscapeVertexToSample(Scene.terrain, Layout.QuadsX(), Layout.QuadsY(), I, J);
  TestEqual(TEXT("past the last column the edge repeats"), I, 61);
  TestEqual(TEXT("past the first row the edge repeats"), J, 0);

  // A very tall terrain gets a larger scale so that the values still fit.
  swarm::SceneData Tall = MakeScene(8, 8, 8.0, 0.0, 0.0);
  Tall.terrain.heights[5] = 900.0;
  EncodeLandscapeHeights(Tall.terrain, Layout, Heights, ScaleZ);
  TestTrue(TEXT("tall terrain scale"), ScaleZ > 100.0f);
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSwarmWeightsTest, "SwarmSim.Terrain.GroundWeights",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)
bool FSwarmWeightsTest::RunTest(const FString& Parameters) {
  // A flat plain with a steep ridge across the middle and a pit at one corner.
  swarm::SceneData Scene = MakeScene(40, 40, 8.0, 0.0, 0.0);
  swarm::Heightfield& T = Scene.terrain;
  for (uint32 J = 0; J <= 40; ++J) {
    for (uint32 I = 0; I <= 40; ++I) {
      double H = 5.0;
      if (I >= 18 && I <= 22) {
        H = 5.0 + 30.0 * (1.0 - FMath::Abs(static_cast<double>(I) - 20.0) / 2.0);  // steep
      }
      if (I < 4 && J < 4) {
        H = -10.0;
      }
      T.heights[T.index(I, J)] = H;
    }
  }
  FLandscapeLayout Layout;
  TestTrue(TEXT("layout"), ChooseLandscapeLayout(40, 40, Layout));
  const TArray<EGround> All = {EGround::Grass, EGround::Rock, EGround::Mud, EGround::ForestA,
                               EGround::ForestB};
  TMap<EGround, TArray<uint8>> Weights;
  FGroundRules Rules;
  ComputeGroundWeights(T, Layout, 7, Rules, All, Weights);
  TestEqual(TEXT("a weightmap per ground type"), Weights.Num(), 5);

  const int32 Nx = Layout.VerticesX();
  bool SumsTo255 = true;
  for (int32 K = 0; K < Nx * Layout.VerticesY(); ++K) {
    int32 Sum = 0;
    for (EGround G : All) {
      Sum += Weights[G][K];
    }
    SumsTo255 = SumsTo255 && Sum == 255;
  }
  TestTrue(TEXT("weights sum to 255 at every vertex"), SumsTo255);

  // Rock on the ridge's flank (u = 19, any row; its apex at u = 20 is flat by symmetry), none on
  // the plain; mud in the pit.
  const auto At = [&](EGround G, int32 U, int32 V) {
    return static_cast<int32>(Weights[G][V * Nx + U]);
  };
  TestTrue(TEXT("the ridge flank is rock"), At(EGround::Rock, 19, 20) > 200);
  TestEqual(TEXT("the plain is not rock"), At(EGround::Rock, 30, 20), 0);
  TestTrue(TEXT("the pit is mud"), At(EGround::Mud, 1, 40 - 1) > 150);  // sample j = 1 is v = 39

  // The same input gives the same weights; a different seed changes the forest patches only.
  TMap<EGround, TArray<uint8>> Again;
  ComputeGroundWeights(T, Layout, 7, Rules, All, Again);
  TestTrue(TEXT("deterministic"), Again[EGround::Grass] == Weights[EGround::Grass]);
  TMap<EGround, TArray<uint8>> Other;
  ComputeGroundWeights(T, Layout, 8, Rules, All, Other);
  TestTrue(TEXT("another seed moves the patches"),
           Other[EGround::ForestA] != Weights[EGround::ForestA]);
  TestTrue(TEXT("but not the rock"), Other[EGround::Rock] == Weights[EGround::Rock]);

  // Missing types give their share to grass, and the weights still sum to 255.
  TMap<EGround, TArray<uint8>> Some;
  ComputeGroundWeights(T, Layout, 7, Rules, {EGround::Grass, EGround::Rock}, Some);
  TestEqual(TEXT("only the types given"), Some.Num(), 2);
  bool Sums = true;
  for (int32 K = 0; K < Nx * Layout.VerticesY(); ++K) {
    Sums = Sums && static_cast<int32>(Some[EGround::Grass][K]) + Some[EGround::Rock][K] == 255;
  }
  TestTrue(TEXT("two types sum to 255"), Sums);
  TMap<EGround, TArray<uint8>> None;
  ComputeGroundWeights(T, Layout, 7, Rules, {}, None);
  TestEqual(TEXT("no types, no weights"), None.Num(), 0);
  return true;
}

#if WITH_EDITOR
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSwarmLandscapeTest, "SwarmSim.Terrain.Landscape",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)
bool FSwarmLandscapeTest::RunTest(const FString& Parameters) {
  UWorld* World = FAutomationEditorCommonUtils::CreateNewMap();
  if (!TestNotNull(TEXT("a test world"), World)) {
    return false;
  }
  const swarm::SceneData Scene = MakeScene(61, 41, 8.0, -200.0, -150.0);

  FString Error;
  FTerrainLook Look;  // no material and no layers: the shape is what is tested
  ALandscape* Landscape = BuildLandscape(World, Scene, Look, Error);
  if (!TestNotNull(FString::Printf(TEXT("a Landscape (%s)"), *Error), Landscape)) {
    return false;
  }

  // Every sample of the heightfield is where the Landscape has the same height.
  const double Worst = MaxLandscapeHeightErrorCm(Landscape, Scene);
  TestTrue(FString::Printf(TEXT("height error %.4f cm is within 1 cm"), Worst),
           Worst >= 0.0 && Worst < 1.0);

  // And the orientation, checked with numbers written out here and not through the conversion
  // helpers: a spike at simulation sample (i = 3, j = 5) must be at Unreal
  //   X = 100 * (origin_x + 3 * cell) = 100 * (-200 + 24) = -17600
  //   Y = -100 * (origin_y + 5 * cell) = -100 * (-150 + 40) = 11000.
  swarm::SceneData Spiked = MakeScene(61, 41, 8.0, -200.0, -150.0);
  for (double& H : Spiked.terrain.heights) {
    H = 0.0;
  }
  Spiked.terrain.heights[Spiked.terrain.index(3, 5)] = 40.0;
  ALandscape* SpikeLandscape = BuildLandscape(World, Spiked, Look, Error);
  if (TestNotNull(TEXT("a second Landscape"), SpikeLandscape)) {
    const TOptional<float> AtSpike =
        SpikeLandscape->GetHeightAtLocation(FVector(-17600.0, 11000.0, 0.0));
    const TOptional<float> Mirrored =
        SpikeLandscape->GetHeightAtLocation(FVector(-17600.0, -11000.0, 0.0));
    const TOptional<float> Transposed =
        SpikeLandscape->GetHeightAtLocation(FVector(11000.0, -17600.0, 0.0));
    TestTrue(TEXT("the spike is at the expected place"),
             AtSpike.IsSet() && FMath::Abs(AtSpike.GetValue() - 4000.0f) < 1.0f);
    TestTrue(TEXT("and not at its mirror image"),
             !Mirrored.IsSet() || FMath::Abs(Mirrored.GetValue()) < 1.0f);
    TestTrue(TEXT("or its transpose"),
             !Transposed.IsSet() || FMath::Abs(Transposed.GetValue()) < 1.0f);
  }
  return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSwarmLandscapeWithAssetsTest,
                                 "SwarmSim.Terrain.LandscapeWithAssets",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)
bool FSwarmLandscapeWithAssetsTest::RunTest(const FString& Parameters) {
  // The terrain with the material and ground layers named in the settings. They come from an
  // asset pack that is not part of the repository, so without it this test only reports that.
  const USwarmSimSettings* Settings = GetDefault<USwarmSimSettings>();
  FTerrainLook Look;
  Look.Material = Settings->LandscapeMaterial.LoadSynchronous();
  const auto Add = [&Look](EGround Ground, const TSoftObjectPtr<ULandscapeLayerInfoObject>& Soft) {
    if (ULandscapeLayerInfoObject* Info = Soft.LoadSynchronous()) {
      Look.Layers.Add(Ground, Info);
    }
  };
  Add(EGround::Grass, Settings->GrassLayer);
  Add(EGround::Rock, Settings->RockLayer);
  Add(EGround::Mud, Settings->MudLayer);
  Add(EGround::ForestA, Settings->ForestGroundALayer);
  Add(EGround::ForestB, Settings->ForestGroundBLayer);
  if (Look.Material == nullptr || Look.Layers.IsEmpty()) {
    AddInfo(TEXT("The terrain assets named in the SwarmSim settings are not installed; skipped."));
    return true;
  }

  UWorld* World = FAutomationEditorCommonUtils::CreateNewMap();
  if (!TestNotNull(TEXT("a test world"), World)) {
    return false;
  }
  const swarm::SceneData Scene = MakeScene(150, 150, 8.0, -600.0, -600.0);
  FString Error;
  ALandscape* Landscape = BuildLandscape(World, Scene, Look, Error);
  if (!TestNotNull(FString::Printf(TEXT("a Landscape with the pack's assets (%s)"), *Error),
                   Landscape)) {
    return false;
  }
  TestTrue(TEXT("the material is set"), Landscape->LandscapeMaterial == Look.Material);
  TestEqual(TEXT("an exact 10 x 10 component grid"), Landscape->LandscapeComponents.Num(), 100);
  const double Worst = MaxLandscapeHeightErrorCm(Landscape, Scene);
  TestTrue(FString::Printf(TEXT("height error %.4f cm is within 1 cm"), Worst),
           Worst >= 0.0 && Worst < 1.0);
  return true;
}
#endif

#endif  // WITH_DEV_AUTOMATION_TESTS
