#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SwarmSceneExporter.generated.h"

namespace SwarmSim {
struct FExportOptions;
struct FGroundSamples;
}  // namespace SwarmSim

// Writes the scene file of the level it is in, then quits the game.
//
// Spawned by the playback subsystem when `-SwarmExportScene=PATH` is on the command line (the
// options are listed in SwarmExportPlan.h). It first moves the player's pawn to the anchor, so that
// a World Partition level streams the area in, waits `-SwarmExportWait` seconds, then samples the
// ground and collects the trees and rocks (SwarmSceneExport.h) and writes the file.
UCLASS()
class SWARMSIM_API ASwarmSceneExporter : public AActor {
  GENERATED_BODY()

 public:
  ASwarmSceneExporter();

  virtual void Tick(float DeltaSeconds) override;

  // Reads the options from the command line. False and `OutError` if one is invalid.
  bool Configure(const FString& CommandLine, FString& OutError);

 private:
  void MovePlayerToAnchor() const;
  void Export();

  TSharedPtr<SwarmSim::FExportOptions> Options;
  TSharedPtr<SwarmSim::FGroundSamples> Ground;
  double Clock = 0.0;
  double MoveClock = 0.0;
  bool bDone = false;
};
