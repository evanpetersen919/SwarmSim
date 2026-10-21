#include "SwarmPlaybackSubsystem.h"

#include "Engine/World.h"
#include "Misc/CommandLine.h"
#include "SwarmLog.h"
#include "SwarmPlayback.h"
#include "SwarmPlaybackMath.h"
#include "SwarmScene.h"

bool USwarmPlaybackSubsystem::ShouldCreateSubsystem(UObject* Outer) const {
  const UWorld* World = Cast<UWorld>(Outer);
  if (World == nullptr || !World->IsGameWorld()) {
    return false;
  }
  return FParse::Param(FCommandLine::Get(), TEXT("SwarmLog")) ||
         !SwarmSim::ParseLaunchOptions(FCommandLine::Get()).LogPath.IsEmpty();
}

void USwarmPlaybackSubsystem::OnWorldBeginPlay(UWorld& InWorld) {
  Super::OnWorldBeginPlay(InWorld);
  const SwarmSim::FLaunchOptions Options = SwarmSim::ParseLaunchOptions(FCommandLine::Get());
  if (!Options.bValid) {
    UE_LOG(LogSwarmSim, Error, TEXT("%s"), *Options.Problem);
    return;
  }
  if (Options.LogPath.IsEmpty()) {
    return;
  }

#if WITH_EDITOR
  if (!Options.SceneFile.IsEmpty()) {
    ASwarmScene* Scene = InWorld.SpawnActor<ASwarmScene>();
    FString Error;
    if (Scene == nullptr || !Scene->BuildFromSceneFile(Options.SceneFile, Error)) {
      UE_LOG(LogSwarmSim, Error, TEXT("scene file %s: %s"), *Options.SceneFile, *Error);
    }
  }
#endif

  Playback = InWorld.SpawnActor<ASwarmPlayback>();
  if (Playback == nullptr) {
    UE_LOG(LogSwarmSim, Error, TEXT("the playback actor could not be spawned"));
    return;
  }
  Playback->DroneScale = Options.DroneScale;
  Playback->SetAnchor(Options.AnchorCm);
  Playback->SetSpeed(Options.Speed);
  FString Error;
  if (!Playback->LoadLog(Options.LogPath, Error)) {
    UE_LOG(LogSwarmSim, Error, TEXT("%s"), *Error);
    return;
  }
  if (!Options.ShotDir.IsEmpty()) {
    Playback->ConfigureScreenshots(Options.ShotDir, Options.ShotDelay, Options.ShotEvery,
                                   Options.ShotCount,
                                   /*bQuitAfter=*/true);
  }
  if (!Options.FrameDir.IsEmpty()) {
    Playback->ConfigureFrames(Options.FrameDir, Options.FrameFps, Options.FrameSeconds);
  }
  if (Options.Camera == TEXT("orbit")) {
    Playback->SetCameraMode(ESwarmCameraMode::Orbit);
  } else if (Options.Camera == TEXT("follow")) {
    Playback->SetCameraMode(ESwarmCameraMode::Follow);
  } else if (Options.Camera == TEXT("cinematic")) {
    Playback->SetCameraMode(ESwarmCameraMode::Cinematic);
  }
}
