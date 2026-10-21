#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SwarmPlayback.generated.h"

class ADefaultPawn;
class APlayerCameraManager;
class UCameraComponent;
class UMaterialInstanceDynamic;
class UMaterialInterface;
class USceneComponent;
class UStaticMesh;
class UStaticMeshComponent;

namespace swarm {
class Replay;
}

UENUM(BlueprintType)
enum class ESwarmCameraMode : uint8 {
  Free,       // the player's own camera
  Orbit,      // circles the swarm
  Follow,     // behind and above one drone
  Cinematic,  // a slow, varying orbit
};

// Plays a recorded run: a drone for each agent in the log, moved along its logged path.
//
// The viewer shows what was recorded and computes nothing about the simulation. Between the logged
// frames a drone follows the replay reader's smooth curve through the logged positions and
// velocities; heading, lean, and rotor spin are for display only, because the log holds no
// orientation. The heads-up display shows the tick, the time, the speed, the number of drones
// alive, and the log's own state hash for the current frame, which at the last tick is the
// trailer's final hash that `swarm_sim` prints for the same log.
//
// Keys, in a game: Space pauses, Up and Down double and halve the speed, Left and Right step back
// and forward ten seconds, R restarts, 1 to 4 choose the camera (free, orbit, follow, cinematic),
// and Tab chooses the drone to follow.
UCLASS()
class SWARMSIM_API ASwarmPlayback : public AActor {
  GENERATED_BODY()

 public:
  ASwarmPlayback();

  virtual void Tick(float DeltaSeconds) override;
  virtual void BeginPlay() override;

  // Reads a run log and creates a drone for each agent. Returns false and sets `OutError` if the
  // file is not a usable log. Replaces any log loaded before.
  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  bool LoadLog(const FString& Path, FString& OutError);

  // Where the simulation's origin is in the level, in centimeters (x and y; heights are absolute).
  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  void SetAnchor(FVector2D AnchorCm);

  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  void SetPlaybackTick(double Tick);

  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  double GetPlaybackTick() const { return PlaybackTick; }

  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  void SetSpeed(double NewSpeed);

  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  void SetPaused(bool bPause) { bPaused = bPause; }

  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  void TogglePause() { bPaused = !bPaused; }

  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  void Restart();

  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  void SetCameraMode(ESwarmCameraMode Mode);

  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  void SelectNextDrone();

  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  void StepSeconds(double Seconds);

  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  int32 GetDroneCount() const { return Drones.Num(); }

  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  int32 GetAliveCount() const { return AliveCount; }

  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  FVector GetDroneLocation(int32 Index) const;

  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  FRotator GetDroneRotation(int32 Index) const;

  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  FLinearColor GetDroneColor(int32 Index) const;

  // The state hash of the log frame at or before the current tick, as sixteen hexadecimal digits.
  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  FString GetHashString() const;

  // The heads-up display's lines.
  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  TArray<FString> GetHudLines() const;

  // Saves screenshots of the game's own render (never of the desktop) into `Directory`: the first
  // after `DelaySeconds` of play and then one every `EverySeconds`, `Count` in all. The game quits
  // after the last when `bQuitAfter` is set.
  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  void ConfigureScreenshots(const FString& Directory, double DelaySeconds, double EverySeconds,
                            int32 Count, bool bQuitAfter);

  // Writes numbered frames into `Directory` for a video: the playback advances by exactly
  // 1/`Fps` of play time for each frame and waits until the frame is saved, so the video does not
  // depend on how fast the machine renders. The game quits after `Seconds` of video.
  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  void ConfigureFrames(const FString& Directory, double Fps, double Seconds);

  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  double GetSpeed() const { return Speed; }

  UFUNCTION(BlueprintCallable, Category = "SwarmSim")
  bool IsPaused() const { return bPaused; }

  // How much larger than life the drones are drawn, so that a drone half a meter across can be seen
  // from across a level. Takes effect for drones made after it is set.
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SwarmSim")
  double DroneScale = 4.0;

  // The display lean limit, in degrees.
  // A content folder whose static meshes are the pieces of one aircraft, drawn in place of the
  // stand-in quadcopter. Empty keeps the stand-in. Takes effect for drones made after it is set.
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SwarmSim")
  FString DroneModelPath;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SwarmSim")
  double MaxTiltDeg = 22.0;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SwarmSim")
  double RotorSpinDegPerSecond = 3600.0;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SwarmSim")
  bool bShowHud = true;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SwarmSim")
  bool bLoop = false;

  UPROPERTY(BlueprintReadOnly, Category = "SwarmSim")
  ESwarmCameraMode CameraMode = ESwarmCameraMode::Free;

 private:
  struct FDrone {
    TObjectPtr<USceneComponent> Root;
    TArray<TObjectPtr<UStaticMeshComponent>> Rotors;
    TObjectPtr<UMaterialInstanceDynamic> Paint;
    FLinearColor Color = FLinearColor::Black;
    double YawDeg = 0.0;
    double PitchDeg = 0.0;
    double RollDeg = 0.0;
    double SpinDeg = 0.0;
    bool bAlive = true;
  };

  void BuildDrones(int32 Count);
  void DestroyDrones();
  void ApplyFrame(double DeltaSeconds, bool bFirst);
  void UpdateCamera(double DeltaSeconds);
  void DrawHud() const;
  void BindKeys();
  void SpeedUp();
  void SlowDown();
  void StepBack() { StepSeconds(-10.0); }
  void StepForward() { StepSeconds(10.0); }
  void ChooseFree() { SetCameraMode(ESwarmCameraMode::Free); }
  void ChooseOrbit() { SetCameraMode(ESwarmCameraMode::Orbit); }
  void ChooseFollow() { SetCameraMode(ESwarmCameraMode::Follow); }
  void ChooseCinematic() { SetCameraMode(ESwarmCameraMode::Cinematic); }
  FVector DroneWorldLocation(double X, double Y, double Z) const;

  TSharedPtr<swarm::Replay> Replay;
  TArray<FDrone> Drones;

  UPROPERTY()
  TObjectPtr<UCameraComponent> Camera;

  UPROPERTY()
  TObjectPtr<APlayerCameraManager> PlainCameraManager;

  UPROPERTY()
  TObjectPtr<ADefaultPawn> FreePawn;

  UPROPERTY()
  TObjectPtr<USceneComponent> Root;

  UPROPERTY()
  TArray<TObjectPtr<UStaticMesh>> Shapes;

  UPROPERTY()
  TObjectPtr<UMaterialInterface> BasicMaterial;

  UPROPERTY()
  TObjectPtr<UMaterialInstanceDynamic> RotorPaint;

  void TakeScheduledShot(double DeltaSeconds);

  FString FrameDirectory;
  double FrameFps = 30.0;
  int32 FrameTotal = 0;
  int32 FramesRequested = 0;

  FString ShotDirectory;
  double ShotDelay = 0.0;
  double ShotEvery = 0.0;
  int32 ShotCount = 0;
  int32 ShotsTaken = 0;
  double ShotClock = 0.0;
  double ShotQuitClock = 0.0;
  bool bQuitAfterShots = false;

  FVector2D Anchor = FVector2D::ZeroVector;
  double PlaybackTick = 0.0;
  double Speed = 1.0;
  bool bPaused = false;
  bool bKeysBound = false;
  int32 AliveCount = 0;
  int32 FollowIndex = 0;
  double CameraClock = 0.0;
  FVector CameraLocation = FVector::ZeroVector;
};
