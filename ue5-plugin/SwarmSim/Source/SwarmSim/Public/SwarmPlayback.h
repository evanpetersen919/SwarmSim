#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SwarmPlayback.generated.h"

class ADefaultPawn;
class APlayerCameraManager;
class UCameraComponent;
class UMaterialInstanceDynamic;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
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
class UCanvas;

UCLASS()
class SWARMSIM_API ASwarmPlayback : public AActor {
  GENERATED_BODY()

 public:
  ASwarmPlayback();

  virtual void Tick(float DeltaSeconds) override;
  virtual void BeginPlay() override;
  virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

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

  // The color of each line of GetHudLines: the fallback lines are red while any drone is out of the
  // nominal state, and green while none is.
  TArray<FColor> GetHudColors() const;

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

  // Real seconds at the start of a frame capture during which the playback is held and nothing is
  // saved, so a World Partition level can stream in and the exposure can settle.
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SwarmSim")
  double FrameWarmupSeconds = 0.0;

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

  // Draws a line between the sender and the receiver of every message sent in the last 0.6 s of
  // play time: green when it will be delivered, yellow when the loss model dropped it, orange when
  // the rate limit did, and red when a jammer did. M toggles it; -SwarmMessages starts with it on.
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SwarmSim")
  bool bShowMessages = false;

  // A small top-down map in the upper right corner: a dot for every drone, colored by its fallback
  // state, with its heading, a short trail, and a ring around the drone the camera follows. North
  // (the simulation's +Y) is up. N toggles it; -SwarmNoMap starts without it.
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SwarmSim")
  bool bShowMap = true;

  // The square of the simulation's plane, in meters, that the map shows: every position of the log
  // with a margin. Empty before a log is loaded.
  FBox2D GetMapBounds() const;

  // A small window under the map with a camera on the drone the main camera follows, looking along
  // its heading. It is the viewer's own rendered view of the level, with the level's fog, and not
  // the simulation's sensor: the simulation's camera fix is a position model and draws nothing.
  // Known limit: in the bright, fogged Electric Dreams level the window is exposed far too high and
  // shows mostly white; it looks right in the Demo_Scene level. C toggles it; -SwarmNoPip starts
  // without it.
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SwarmSim")
  bool bShowPip = true;

  // A strip at the lower left: the altitude and speed of the followed drone over the last 30 s of
  // play, and a band of its logged fallback state. T toggles it; -SwarmNoTelemetry starts without
  // it.
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SwarmSim")
  bool bShowTelemetry = true;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SwarmSim")
  bool bLoop = false;

  UPROPERTY(BlueprintReadOnly, Category = "SwarmSim")
  ESwarmCameraMode CameraMode = ESwarmCameraMode::Free;

 private:
  struct FDrone {
    TObjectPtr<USceneComponent> Root;
    TArray<TObjectPtr<UStaticMeshComponent>> Rotors;
    // The propeller of a folder model, turned about the fuselage axis; null for the stand-in.
    TObjectPtr<USceneComponent> Propeller;
    TObjectPtr<UMaterialInstanceDynamic> Paint;
    FLinearColor Color = FLinearColor::Black;
    double YawDeg = 0.0;
    double PitchDeg = 0.0;
    double RollDeg = 0.0;
    double SpinDeg = 0.0;
    bool bAlive = true;
    uint8 FallbackState = 0;  // the logged fallback state, 0 nominal to 3 safe hold
    double SpeedMps = 0.0;    // the speed in the log, in meters per second
    double AltitudeM = 0.0;   // the logged height (z), in meters
    double ClimbMps = 0.0;    // the logged vertical speed, in meters per second
    double HeadingDeg = 0.0;  // compass heading of the velocity, north (+Y) 0, east 90
  };

  // Whether the drone model is an aircraft, which is tilted by its flight path and not by its
  // acceleration. True when a folder model is drawn.
  bool bAircraftAttitude = false;

  void BuildDrones(int32 Count);
  void DestroyDrones();
  void ApplyFrame(double DeltaSeconds, bool bFirst);
  void UpdateCamera(double DeltaSeconds);
  void DrawHud() const;
  void DrawMessages();
  void DrawMap(UCanvas* Canvas, APlayerController* Controller);
  void ToggleMap() { bShowMap = !bShowMap; }
  void TogglePip() { bShowPip = !bShowPip; }
  void ToggleTelemetry() { bShowTelemetry = !bShowTelemetry; }
  void UpdatePip();
  void DrawPanels(UCanvas* Canvas, APlayerController* Controller);
  void DrawPip(UCanvas* Canvas, double MapSide) const;
  void DrawTelemetry(UCanvas* Canvas) const;
  void HideMessageLines();
  void ToggleMessages() { bShowMessages = !bShowMessages; }
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

  // The lines between drones are thin cylinders from a pool, not debug lines: the exposure of a
  // bright level darkens debug lines to almost black. One paint per outcome.
  UPROPERTY()
  TArray<TObjectPtr<UStaticMeshComponent>> MessageLines;

  UPROPERTY()
  TArray<TObjectPtr<UMaterialInstanceDynamic>> MessagePaints;

  UPROPERTY()
  TObjectPtr<UStaticMesh> MessageShape;

  bool bMessagesDrawn = false;

  // The map's extent in simulation meters, from every position of the log, and its draw handle.
  double MapMinX = 0.0;
  double MapMinY = 0.0;
  double MapSize = 0.0;
  FDelegateHandle MapDrawHandle;
  FDelegateHandle PanelDrawHandle;

  // The viewer camera window: a capture component on the followed drone, drawing into a render
  // target that the canvas shows.
  UPROPERTY()
  TObjectPtr<USceneCaptureComponent2D> PipCapture;

  UPROPERTY()
  TObjectPtr<UTextureRenderTarget2D> PipTarget;

  int32 PipDrone = -1;  // the drone whose parts the capture hides, so it does not see itself
  // Height of the followed drone above the level's surface straight below it, by a trace down, in
  // meters; negative when nothing is below.
  double HeightAboveTerrainM = -1.0;

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
  double WarmupClock = 0.0;

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
  // Alive drones by fallback state: nominal, degraded comms, degraded navigation, safe hold.
  int32 StateCounts[4] = {0, 0, 0, 0};
  int32 FollowIndex = 0;

  void BuildHud(TArray<FString>& Lines, TArray<FColor>& Colors) const;
  double CameraClock = 0.0;
  FVector CameraLocation = FVector::ZeroVector;

  // The visibility in meters that the loaded run's scenario gives, or zero if it has none. When it
  // is under SwarmSim::kDrawnFogMaxVisibilityM the level's exponential height fog is set to a
  // uniform fog of that visibility (or one is spawned), so the viewer shows the weather the run was
  // flown in.
  double VisibilityM = 0.0;
  void ApplyWeather();
};
