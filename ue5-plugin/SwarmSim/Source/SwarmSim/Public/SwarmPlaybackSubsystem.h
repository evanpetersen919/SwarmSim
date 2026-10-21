#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "SwarmPlaybackSubsystem.generated.h"

class ASwarmPlayback;

// Starts a replay in whatever level is loaded, when the command line asks for one:
//   -SwarmLog=PATH        the run log to play
//   -SwarmAnchor=X,Y      where the simulation's origin is in the level, in centimeters
//   -SwarmSpeed=N         playback speed
//   -SwarmScale=N         how much larger than life the drones are drawn
//   -SwarmSceneFile=PATH  a scene file to build the terrain, trees and rocks from (editor only)
// Without -SwarmLog it is not created and does nothing. The level is not edited: the playback
// actor is spawned when play begins.
UCLASS()
class SWARMSIM_API USwarmPlaybackSubsystem : public UWorldSubsystem {
  GENERATED_BODY()

 public:
  virtual bool ShouldCreateSubsystem(UObject* Outer) const override;
  virtual void OnWorldBeginPlay(UWorld& InWorld) override;

  UPROPERTY(BlueprintReadOnly, Category = "SwarmSim")
  TObjectPtr<ASwarmPlayback> Playback;
};
