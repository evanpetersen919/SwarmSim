#pragma once

#include "CoreMinimal.h"
#include "SwarmExportPlan.h"
#include "swarm/format/scene.hpp"

class UWorld;

namespace SwarmSim {
// What the exporter found, for the log and for tests.
struct FExportReport {
  int32 HeightSamples = 0;
  int32 HeightMisses = 0;  // samples with no ground under them, filled from their neighbors
  double MinHeightM = 0.0;
  double MaxHeightM = 0.0;
  int32 Trees = 0;
  int32 Crowns = 0;  // second cylinders, one per tree, when crowns are on
  int32 Rocks = 0;
  // Static meshes found inside the area, most numerous first: "name: count (class)". Meant for
  // choosing the name patterns for a level the exporter has not seen.
  TArray<FString> Survey;
};

// Ground heights sampled so far, a row of the grid at a time. The grid has `Cells` cells and so
// `Cells + 1` samples a side; sample (i, j) is HeightsM[j * (Cells + 1) + i].
struct FGroundSamples {
  int32 Cells = 0;
  TArray<double> HeightsM;
  TArray<bool> Known;  // false where there was no ground under the sample
  int32 RowsDone = 0;

  void Reset(int32 InCells);
  int32 Samples() const { return Cells + 1; }
  bool Complete() const { return RowsDone >= Samples(); }
};

// The top surface of the level at each sample of the grid, found by one trace straight down from
// far above: the highest thing an aircraft over that point would meet, tree, rock, or ground. The
// audit of an export compares it with the scene the simulation runs on.
struct FTopSamples {
  int32 Cells = 0;
  TArray<double> HeightsM;
  TArray<bool> Known;
  TArray<FString> Names;  // the static mesh (or component) that was hit
  int32 RowsDone = 0;

  void Reset(int32 InCells);
  int32 Samples() const { return Cells + 1; }
  bool Complete() const { return RowsDone >= Samples(); }
};

// Samples the next `RowCount` rows of the top surface (fewer at the end).
void SampleTopRows(UWorld* World, const FExportOptions& Options, int32 RowCount,
                   FTopSamples& Samples);

// The audit as CSV text: a header, then one line per sample with x and y in the simulation's
// frame in meters, the top height in meters, and the name. Samples with no hit are left out.
FString TopSamplesToCsv(const FExportOptions& Options, const FTopSamples& Samples);

// Samples the next `RowCount` rows (fewer at the end). The ground is found by tracing straight
// down at each sample and keeping the Landscape's surface, so trees and rocks above it do not
// count as ground.
void SampleGroundRows(UWorld* World, const FExportOptions& Options, int32 RowCount,
                      FGroundSamples& Samples);

// With every row sampled: fills the samples that have no ground from their neighbors, collects the
// obstacles (the static mesh instances, and single static mesh components, in the area whose mesh
// name matches the options), and builds the scene. Fails if no ground was found, or if more than 1
// percent of the samples have none.
bool FinishExport(UWorld* World, const FExportOptions& Options, FGroundSamples& Samples,
                  swarm::SceneData& OutScene, FExportReport& OutReport, FString& OutError);

// All of it at once, for tests and small areas.
bool ExportSceneFromWorld(UWorld* World, const FExportOptions& Options, swarm::SceneData& OutScene,
                          FExportReport& OutReport, FString& OutError);
}  // namespace SwarmSim
