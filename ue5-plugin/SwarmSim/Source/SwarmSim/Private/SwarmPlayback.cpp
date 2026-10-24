#include "SwarmPlayback.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "Camera/CameraComponent.h"
#include "Camera/PlayerCameraManager.h"
#include "CanvasItem.h"
#include "Components/InputComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Debug/DebugDrawService.h"
#include "Engine/Canvas.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Engine/Engine.h"
#include "Engine/ExponentialHeightFog.h"
#include "Engine/StaticMesh.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/DefaultPawn.h"
#include "GameFramework/PlayerController.h"
#include "HAL/PlatformMisc.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Math/RotationMatrix.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "SwarmCoordinates.h"
#include "SwarmLog.h"
#include "SwarmPlaybackMath.h"
#include "UnrealClient.h"
#include "swarm/format/replay.hpp"

DEFINE_LOG_CATEGORY(LogSwarmSim);

namespace {
const FName kColorParameter(TEXT("Color"));
// Wingspan of the drone model in centimeters: 12 ft, the figure press reports for the ALTIUS-700.
constexpr double kModelSpanCm = 366.0;
// The turn that points the model's nose (+Y in the file) along +X.
FRotator kModelTurn() { return FRotator(0.0, -90.0, 0.0); }
// How many ticks back the acceleration is taken over: 5 ticks is 0.1 s at the usual 0.02 s.
constexpr double kAccelerationWindowTicks = 5.0;
// Time constant of the display attitude's smoothing, in seconds.
constexpr double kAttitudeSmoothing = 0.2;
// The propeller of a folder model turns this many degrees per meter per second of airspeed: about
// 26 turns per second at 60 km/h. At the frame rate of a capture a spin this fast looks like a
// blur or a slow, steady turn, as a real propeller does on film.
constexpr double kPropellerDegPerMps = 560.0;
// Mesh-name prefixes of the ALTIUS model's pusher propeller: two thin blades (Cylinder006 and
// Cylinder012) and the four small cylinders at their roots, at the very tail of the fuselage. The
// spinner (Mesh002) is nearly round about the axis and stays still. The glTF gives its nodes
// generic names; these were found from the world-space bounds of each mesh. C-Ext001 and C-Ext002,
// with Cylinder015 and Cylinder016, are tail surfaces further forward and must not turn.
const TCHAR* const kPropellerParts[] = {TEXT("Cylinder006"), TEXT("Cylinder007"),
                                        TEXT("Cylinder008"), TEXT("Cylinder010"),
                                        TEXT("Cylinder011"), TEXT("Cylinder012")};
// The root cylinders; the center of their bounds is the propeller's hub on the fuselage axis.
const TCHAR* const kPropellerHubs[] = {TEXT("Cylinder007"), TEXT("Cylinder008"),
                                       TEXT("Cylinder010"), TEXT("Cylinder011")};

bool NameStartsWithAny(const UStaticMesh* Mesh, const TCHAR* const* Prefixes, int32 Count) {
  const FString Name = Mesh->GetName();
  for (int32 I = 0; I < Count; ++I) {
    if (Name.StartsWith(Prefixes[I])) {
      return true;
    }
  }
  return false;
}
constexpr uint64 kHudKeyBase = 7100;
// The size of the viewer camera's render target, in pixels.
constexpr int32 kPipWidth = 480;
constexpr int32 kPipHeight = 270;
// How much play time the telemetry strip shows, and how many samples it takes over it.
constexpr double kTelemetrySeconds = 30.0;
constexpr int32 kTelemetrySamples = 60;
}  // namespace

ASwarmPlayback::ASwarmPlayback() {
  PrimaryActorTick.bCanEverTick = true;
  Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
  SetRootComponent(Root);
  Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
  Camera->SetupAttachment(Root);
  Camera->SetFieldOfView(70.0f);
  PipCapture = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("PipCapture"));
  PipCapture->SetupAttachment(Root);
  PipCapture->FOVAngle = 70.0f;
  PipCapture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
  PipCapture->bCaptureEveryFrame = false;
  PipCapture->bCaptureOnMovement = false;
}

void ASwarmPlayback::BeginPlay() {
  Super::BeginPlay();
  BindKeys();
  // The map is drawn on the game's canvas, by the engine's debug draw service.
  MapDrawHandle = UDebugDrawService::Register(
      TEXT("Game"), FDebugDrawDelegate::CreateUObject(this, &ASwarmPlayback::DrawMap));
  PanelDrawHandle = UDebugDrawService::Register(
      TEXT("Game"), FDebugDrawDelegate::CreateUObject(this, &ASwarmPlayback::DrawPanels));
  PipTarget = NewObject<UTextureRenderTarget2D>(this);
  PipTarget->InitCustomFormat(kPipWidth, kPipHeight, PF_B8G8R8A8, false);
  PipTarget->UpdateResourceImmediate(true);
  PipCapture->TextureTarget = PipTarget;
}

void ASwarmPlayback::EndPlay(const EEndPlayReason::Type EndPlayReason) {
  if (MapDrawHandle.IsValid()) {
    UDebugDrawService::Unregister(MapDrawHandle);
    MapDrawHandle.Reset();
  }
  if (PanelDrawHandle.IsValid()) {
    UDebugDrawService::Unregister(PanelDrawHandle);
    PanelDrawHandle.Reset();
  }
  Super::EndPlay(EndPlayReason);
}

FBox2D ASwarmPlayback::GetMapBounds() const {
  if (!(MapSize > 0.0)) {
    return FBox2D(ForceInit);
  }
  return FBox2D(FVector2D(MapMinX, MapMinY), FVector2D(MapMinX + MapSize, MapMinY + MapSize));
}

void ASwarmPlayback::BindKeys() {
  if (bKeysBound || GetWorld() == nullptr) {
    return;
  }
  APlayerController* Controller = GetWorld()->GetFirstPlayerController();
  if (Controller == nullptr) {
    return;
  }
  EnableInput(Controller);
  if (InputComponent == nullptr) {
    return;
  }
  InputComponent->BindKey(EKeys::SpaceBar, IE_Pressed, this, &ASwarmPlayback::TogglePause);
  InputComponent->BindKey(EKeys::Up, IE_Pressed, this, &ASwarmPlayback::SpeedUp);
  InputComponent->BindKey(EKeys::Down, IE_Pressed, this, &ASwarmPlayback::SlowDown);
  InputComponent->BindKey(EKeys::Left, IE_Pressed, this, &ASwarmPlayback::StepBack);
  InputComponent->BindKey(EKeys::Right, IE_Pressed, this, &ASwarmPlayback::StepForward);
  InputComponent->BindKey(EKeys::R, IE_Pressed, this, &ASwarmPlayback::Restart);
  InputComponent->BindKey(EKeys::One, IE_Pressed, this, &ASwarmPlayback::ChooseFree);
  InputComponent->BindKey(EKeys::Two, IE_Pressed, this, &ASwarmPlayback::ChooseOrbit);
  InputComponent->BindKey(EKeys::Three, IE_Pressed, this, &ASwarmPlayback::ChooseFollow);
  InputComponent->BindKey(EKeys::Four, IE_Pressed, this, &ASwarmPlayback::ChooseCinematic);
  InputComponent->BindKey(EKeys::Tab, IE_Pressed, this, &ASwarmPlayback::SelectNextDrone);
  InputComponent->BindKey(EKeys::M, IE_Pressed, this, &ASwarmPlayback::ToggleMessages);
  InputComponent->BindKey(EKeys::N, IE_Pressed, this, &ASwarmPlayback::ToggleMap);
  InputComponent->BindKey(EKeys::C, IE_Pressed, this, &ASwarmPlayback::TogglePip);
  InputComponent->BindKey(EKeys::T, IE_Pressed, this, &ASwarmPlayback::ToggleTelemetry);
  bKeysBound = true;
}

bool ASwarmPlayback::LoadLog(const FString& Path, FString& OutError) {
  TArray<uint8> Bytes;
  if (!FFileHelper::LoadFileToArray(Bytes, *Path)) {
    OutError = FString::Printf(TEXT("cannot read %s"), *Path);
    return false;
  }
  TSharedPtr<swarm::Replay> Loaded = MakeShared<swarm::Replay>();
  const swarm::ReplayStatus Status =
      Loaded->load(Bytes.GetData(), static_cast<size_t>(Bytes.Num()));
  if (Status != swarm::ReplayStatus::kOk) {
    OutError = FString::Printf(TEXT("%s is not a usable run log: %s"), *Path,
                               UTF8_TO_TCHAR(swarm::to_string(Status)));
    return false;
  }
  Replay = Loaded;
  // The map shows every position of the log, so it does not move or rescale during playback.
  double MinX = 0.0, MinY = 0.0, MaxX = 0.0, MaxY = 0.0;
  bool bAny = false;
  for (const swarm::ReplayFrame& Frame : Replay->frames()) {
    for (const swarm::AgentSnapshot& Agent : Frame.snapshot.agents) {
      MinX = bAny ? FMath::Min(MinX, Agent.position[0]) : Agent.position[0];
      MaxX = bAny ? FMath::Max(MaxX, Agent.position[0]) : Agent.position[0];
      MinY = bAny ? FMath::Min(MinY, Agent.position[1]) : Agent.position[1];
      MaxY = bAny ? FMath::Max(MaxY, Agent.position[1]) : Agent.position[1];
      bAny = true;
    }
  }
  const SwarmSim::FMapExtent Extent = SwarmSim::MapExtentFor(MinX, MinY, MaxX, MaxY);
  MapMinX = Extent.Min.X;
  MapMinY = Extent.Min.Y;
  MapSize = bAny ? Extent.Size : 0.0;
  DestroyDrones();
  BuildDrones(static_cast<int32>(Replay->header().agent_count));
  VisibilityM = 0.0;
  SwarmSim::VisibilityFromConfig(UTF8_TO_TCHAR(Replay->header().config.c_str()), VisibilityM);
  ApplyWeather();
  PlaybackTick = static_cast<double>(Replay->first_tick());
  FollowIndex = 0;
  ApplyFrame(0.0, true);
  UE_LOG(LogSwarmSim, Display, TEXT("Loaded %s: %d drones, %d frames, ticks %llu to %llu"), *Path,
         Drones.Num(), static_cast<int32>(Replay->frames().size()),
         static_cast<uint64>(Replay->first_tick()), static_cast<uint64>(Replay->last_tick()));
  return true;
}

void ASwarmPlayback::ApplyWeather() {
  UWorld* World = GetWorld();
  if (World == nullptr || !(VisibilityM > 0.0) || VisibilityM > SwarmSim::kDrawnFogMaxVisibilityM) {
    return;
  }
  AExponentialHeightFog* Fog = nullptr;
  for (TActorIterator<AExponentialHeightFog> It(World); It; ++It) {
    Fog = *It;
    break;
  }
  if (Fog == nullptr) {
    Fog = World->SpawnActor<AExponentialHeightFog>(FVector::ZeroVector, FRotator::ZeroRotator);
  }
  UExponentialHeightFogComponent* Component = Fog != nullptr ? Fog->GetComponent() : nullptr;
  if (Component == nullptr) {
    return;
  }
  // The simulation's fog is uniform, so the fog does not thin out with height, and it reaches the
  // observer at once.
  const double Density = SwarmSim::FogDensityForVisibility(VisibilityM);
  Component->SetFogDensity(static_cast<float>(Density));
  Component->SetFogHeightFalloff(0.0f);
  Component->SetStartDistance(0.0f);
  Component->SetFogMaxOpacity(1.0f);
  UE_LOG(LogSwarmSim, Display, TEXT("Weather: visibility %.0f m, fog density %.4f"), VisibilityM,
         Density);
}

void ASwarmPlayback::DestroyDrones() {
  for (FDrone& Drone : Drones) {
    if (Drone.Root != nullptr) {
      TArray<USceneComponent*> Parts;
      Drone.Root->GetChildrenComponents(true, Parts);
      for (USceneComponent* Part : Parts) {
        Part->DestroyComponent();
      }
      Drone.Root->DestroyComponent();
    }
  }
  Drones.Reset();
  AliveCount = 0;
  for (int32& InState : StateCounts) {
    InState = 0;
  }
}

void ASwarmPlayback::BuildDrones(int32 Count) {
  UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
  UStaticMesh* Cylinder =
      LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
  UStaticMesh* Sphere = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
  UMaterialInterface* Basic = LoadObject<UMaterialInterface>(
      nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
  if (Basic != nullptr) {
    RotorPaint = UMaterialInstanceDynamic::Create(Basic, this);
    RotorPaint->SetVectorParameterValue(kColorParameter, FLinearColor(0.02f, 0.02f, 0.02f, 1.0f));
  }
  UMaterialInstanceDynamic* NosePaint = nullptr;
  if (Basic != nullptr) {
    NosePaint = UMaterialInstanceDynamic::Create(Basic, this);
    NosePaint->SetVectorParameterValue(kColorParameter, FLinearColor(0.95f, 0.95f, 0.95f, 1.0f));
  }

  const auto AddPart = [this](USceneComponent* Parent, UStaticMesh* Mesh, const FVector& Location,
                              const FRotator& Rotation, const FVector& Scale,
                              UMaterialInterface* Material) -> UStaticMeshComponent* {
    if (Mesh == nullptr) {
      return nullptr;
    }
    UStaticMeshComponent* Part = NewObject<UStaticMeshComponent>(this);
    Part->SetStaticMesh(Mesh);
    Part->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Part->SetupAttachment(Parent);
    Part->RegisterComponent();
    AddInstanceComponent(Part);
    Part->SetRelativeLocationAndRotation(Location, Rotation);
    Part->SetRelativeScale3D(Scale);
    if (Material != nullptr) {
      Part->SetMaterial(0, Material);
    }
    return Part;
  };

  // A named model is every static mesh in its folder, each kept at the position it was authored
  // at, so the pieces assemble the aircraft. The model is centered, turned so that its nose (+Y in
  // the file) points along +X, and scaled so its wingspan (along X in the file) is kModelSpanCm.
  TArray<UStaticMesh*> ModelMeshes;
  FBox ModelBox(ForceInit);
  if (!DroneModelPath.IsEmpty()) {
    // A game does not scan content folders ahead of time, so scan this one before asking.
    IAssetRegistry::Get()->ScanPathsSynchronous({DroneModelPath}, /*bForceRescan=*/false);
    FARFilter Filter;
    Filter.ClassPaths.Add(UStaticMesh::StaticClass()->GetClassPathName());
    Filter.PackagePaths.Add(FName(*DroneModelPath));
    Filter.bRecursivePaths = true;
    TArray<FAssetData> Assets;
    IAssetRegistry::Get()->GetAssets(Filter, Assets);
    Assets.Sort([](const FAssetData& A, const FAssetData& B) {
      return A.AssetName.LexicalLess(B.AssetName);
    });
    for (const FAssetData& Asset : Assets) {
      if (UStaticMesh* Mesh = Cast<UStaticMesh>(Asset.GetAsset())) {
        ModelMeshes.Add(Mesh);
        ModelBox += Mesh->GetBounds().GetBox();
      }
    }
    if (ModelMeshes.IsEmpty() || ModelBox.GetSize().X <= 0.0) {
      UE_LOG(LogSwarmSim, Warning, TEXT("no static meshes under %s, using the stand-in drone"),
             *DroneModelPath);
      ModelMeshes.Reset();
    }
  }
  // The propeller's hub, in the model's own space: the center of its two hub meshes.
  FBox HubBox(ForceInit);
  for (const UStaticMesh* Mesh : ModelMeshes) {
    if (NameStartsWithAny(Mesh, kPropellerHubs, UE_ARRAY_COUNT(kPropellerHubs))) {
      HubBox += Mesh->GetBounds().GetBox();
    }
  }
  const FVector PropellerHub = HubBox.IsValid ? HubBox.GetCenter() : ModelBox.GetCenter();
  const double ModelScale = ModelMeshes.IsEmpty() ? 1.0 : kModelSpanCm / ModelBox.GetSize().X;
  const FRotator ModelTurn = kModelTurn();
  const FVector ModelOffset = ModelTurn.RotateVector(-ModelBox.GetCenter() * ModelScale);

  bAircraftAttitude = !ModelMeshes.IsEmpty();
  Drones.SetNum(Count);
  for (int32 I = 0; I < Count; ++I) {
    FDrone& Drone = Drones[I];
    Drone.Root = NewObject<USceneComponent>(this);
    Drone.Root->SetupAttachment(GetRootComponent());
    Drone.Root->RegisterComponent();
    AddInstanceComponent(Drone.Root);
    Drone.Root->SetWorldScale3D(FVector(DroneScale));

    if (!ModelMeshes.IsEmpty()) {
      // The propeller hangs from a pivot at its hub, which is turned about the model's Y axis.
      // The pivot carries the model's own transform, and its parts sit at minus the hub so that
      // each lands where the other parts do.
      Drone.Propeller = NewObject<USceneComponent>(this);
      Drone.Propeller->SetupAttachment(Drone.Root);
      Drone.Propeller->RegisterComponent();
      AddInstanceComponent(Drone.Propeller);
      Drone.Propeller->SetRelativeLocationAndRotation(
          ModelOffset + ModelTurn.RotateVector(PropellerHub * ModelScale), ModelTurn);
      Drone.Propeller->SetRelativeScale3D(FVector(ModelScale));
      for (UStaticMesh* Mesh : ModelMeshes) {
        if (NameStartsWithAny(Mesh, kPropellerParts, UE_ARRAY_COUNT(kPropellerParts))) {
          AddPart(Drone.Propeller, Mesh, -PropellerHub, FRotator::ZeroRotator, FVector::OneVector,
                  nullptr);
        } else {
          AddPart(Drone.Root, Mesh, ModelOffset, ModelTurn, FVector(ModelScale), nullptr);
        }
      }
      // The model's materials have no color parameter, and a lamp on the fuselage looked wrong on
      // the aircraft, so a folder model carries no state color: the heads-up display shows the
      // fallback state instead (see BuildHud).
      Drone.Color = FLinearColor::Black;
      continue;
    }
    if (Basic != nullptr) {
      Drone.Paint = UMaterialInstanceDynamic::Create(Basic, this);
    }

    // A stand-in quadcopter from the engine's shapes, about 60 cm across at scale 1: a flat body,
    // two crossed arms, four rotor discs, and a white nose to show the heading.
    AddPart(Drone.Root, Cube, FVector::ZeroVector, FRotator::ZeroRotator, FVector(0.30, 0.30, 0.08),
            Drone.Paint);
    AddPart(Drone.Root, Cube, FVector::ZeroVector, FRotator(0.0, 45.0, 0.0),
            FVector(0.64, 0.035, 0.03), Drone.Paint);
    AddPart(Drone.Root, Cube, FVector::ZeroVector, FRotator(0.0, -45.0, 0.0),
            FVector(0.64, 0.035, 0.03), Drone.Paint);
    AddPart(Drone.Root, Cube, FVector(17.0, 0.0, 0.0), FRotator::ZeroRotator,
            FVector(0.08, 0.12, 0.06), NosePaint);
    const double Arm = 22.6;
    const FVector Corners[4] = {FVector(Arm, Arm, 4.0), FVector(Arm, -Arm, 4.0),
                                FVector(-Arm, Arm, 4.0), FVector(-Arm, -Arm, 4.0)};
    for (const FVector& Corner : Corners) {
      UStaticMeshComponent* Rotor = AddPart(Drone.Root, Cylinder, Corner, FRotator::ZeroRotator,
                                            FVector(0.24, 0.24, 0.012), RotorPaint);
      if (Rotor != nullptr) {
        Drone.Rotors.Add(Rotor);
      }
    }
    Drone.Color = FLinearColor::Black;
  }
}

FVector ASwarmPlayback::DroneWorldLocation(double X, double Y, double Z) const {
  const FVector Local = SwarmSim::ToUnreal(X, Y, Z);
  return FVector(Local.X + Anchor.X, Local.Y + Anchor.Y, Local.Z);
}

void ASwarmPlayback::SetAnchor(FVector2D AnchorCm) {
  Anchor = AnchorCm;
  if (Replay.IsValid()) {
    ApplyFrame(0.0, true);
  }
}

void ASwarmPlayback::SetPlaybackTick(double Tick) {
  if (!Replay.IsValid()) {
    return;
  }
  PlaybackTick = FMath::Clamp(Tick, static_cast<double>(Replay->first_tick()),
                              static_cast<double>(Replay->last_tick()));
  ApplyFrame(0.0, true);
}

void ASwarmPlayback::SetSpeed(double NewSpeed) { Speed = FMath::Clamp(NewSpeed, 1.0 / 64.0, 64.0); }

void ASwarmPlayback::SpeedUp() { SetSpeed(Speed * 2.0); }
void ASwarmPlayback::SlowDown() { SetSpeed(Speed * 0.5); }

void ASwarmPlayback::Restart() {
  if (Replay.IsValid()) {
    SetPlaybackTick(static_cast<double>(Replay->first_tick()));
  }
}

void ASwarmPlayback::StepSeconds(double Seconds) {
  if (Replay.IsValid() && Replay->header().dt > 0.0) {
    SetPlaybackTick(PlaybackTick + Seconds / Replay->header().dt);
  }
}

void ASwarmPlayback::SelectNextDrone() {
  if (!Drones.IsEmpty()) {
    FollowIndex = (FollowIndex + 1) % Drones.Num();
  }
}

void ASwarmPlayback::SetCameraMode(ESwarmCameraMode Mode) {
  CameraMode = Mode;
  CameraClock = 0.0;
  APlayerController* Controller =
      GetWorld() != nullptr ? GetWorld()->GetFirstPlayerController() : nullptr;
  if (Controller == nullptr) {
    return;
  }
  // A level may bring its own camera manager and pawn (Electric Dreams does, and its manager
  // ignores the view target and reports the world origin), so the playback uses a plain manager.
  if (PlainCameraManager == nullptr) {
    PlainCameraManager = GetWorld()->SpawnActor<APlayerCameraManager>();
    if (PlainCameraManager != nullptr) {
      PlainCameraManager->InitializeFor(Controller);
    }
  }
  if (PlainCameraManager != nullptr) {
    Controller->PlayerCameraManager = PlainCameraManager;
  }
  if (Mode == ESwarmCameraMode::Free) {
    if (FreePawn == nullptr) {
      FVector Start = FVector(0.0, 0.0, 20000.0);
      if (!Drones.IsEmpty()) {
        Start = GetDroneLocation(0) + FVector(-4000.0, 0.0, 1500.0);
      }
      FActorSpawnParameters Params;
      Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
      FreePawn = GetWorld()->SpawnActor<ADefaultPawn>(Start, FRotator::ZeroRotator, Params);
    }
    if (FreePawn != nullptr) {
      Controller->Possess(FreePawn);
      Controller->SetViewTargetWithBlend(FreePawn, 0.0f);
    }
  } else {
    Controller->SetViewTargetWithBlend(this, 0.0f);
  }
}

void ASwarmPlayback::Tick(float DeltaSeconds) {
  Super::Tick(DeltaSeconds);
  if (!Replay.IsValid()) {
    return;
  }
  const bool bRecording = !FrameDirectory.IsEmpty() && FrameTotal > 0;
  if (bRecording) {
    // One saved frame at a time; the playback does not move until the last one is on disk.
    if (FScreenshotRequest::IsScreenshotRequested()) {
      return;
    }
    if (FramesRequested >= FrameTotal) {
      FPlatformMisc::RequestExit(false);
      return;
    }
    if (FramesRequested == 0 && WarmupClock < FrameWarmupSeconds) {
      // Hold the first frame while the level streams in; nothing is saved yet.
      WarmupClock += DeltaSeconds;
      BindKeys();
      ApplyFrame(DeltaSeconds, false);
      UpdateCamera(DeltaSeconds);
      UpdatePip();
      return;
    }
    DeltaSeconds = static_cast<float>(1.0 / FrameFps);
  }
  BindKeys();  // the controller may not have existed at BeginPlay
  const double First = static_cast<double>(Replay->first_tick());
  const double Last = static_cast<double>(Replay->last_tick());
  if (!bPaused) {
    PlaybackTick =
        SwarmSim::AdvanceTick(PlaybackTick, DeltaSeconds, Speed, Replay->header().dt, First, Last);
    if (PlaybackTick >= Last) {
      if (bLoop) {
        PlaybackTick = First;
      } else {
        bPaused = true;
      }
    }
  }
  CameraClock += DeltaSeconds;
  TakeScheduledShot(DeltaSeconds);
  ApplyFrame(DeltaSeconds, false);
  UpdateCamera(DeltaSeconds);
  UpdatePip();
  if (bShowMessages) {
    DrawMessages();
  } else if (bMessagesDrawn) {
    HideMessageLines();
  }
  if (bShowHud) {
    DrawHud();
  }
  if (bRecording) {
    const FString Path = FPaths::ConvertRelativePathToFull(
        FrameDirectory / FString::Printf(TEXT("frame_%05d.png"), FramesRequested));
    FScreenshotRequest::RequestScreenshot(Path, /*bInShowUI=*/true, /*bAddFilenameSuffix=*/false);
    ++FramesRequested;
  }
}

void ASwarmPlayback::ApplyFrame(double DeltaSeconds, bool bFirst) {
  if (!Replay.IsValid()) {
    return;
  }
  const double Dt = Replay->header().dt;
  std::vector<swarm::AgentSnapshot> Now;
  std::vector<swarm::AgentSnapshot> Before;
  if (!Replay->sample(PlaybackTick, Now)) {
    return;
  }
  const double Earlier = FMath::Max(static_cast<double>(Replay->first_tick()),
                                    PlaybackTick - kAccelerationWindowTicks);
  Replay->sample(Earlier, Before);
  const double Window = (PlaybackTick - Earlier) * Dt;

  AliveCount = 0;
  for (int32& InState : StateCounts) {
    InState = 0;
  }
  const int32 Count = FMath::Min(Drones.Num(), static_cast<int32>(Now.size()));
  for (int32 I = 0; I < Count; ++I) {
    FDrone& Drone = Drones[I];
    const swarm::AgentSnapshot& A = Now[I];
    if (A.alive) {
      ++AliveCount;
      ++StateCounts[FMath::Min<int32>(A.fallback_state, 3)];
    }
    Drone.FallbackState = A.fallback_state;
    Drone.SpeedMps = FVector(A.velocity[0], A.velocity[1], A.velocity[2]).Size();
    Drone.AltitudeM = A.position[2];
    Drone.ClimbMps = A.velocity[2];
    if (A.velocity[0] != 0.0 || A.velocity[1] != 0.0) {
      Drone.HeadingDeg = FMath::Fmod(
          FMath::RadiansToDegrees(FMath::Atan2(A.velocity[0], A.velocity[1])) + 360.0, 360.0);
    }
    const FVector Location = DroneWorldLocation(A.position[0], A.position[1], A.position[2]);

    // Display attitude from the velocity and the change of velocity over the last window.
    const FVector Velocity(A.velocity[0], -A.velocity[1], A.velocity[2]);
    FVector Acceleration = FVector::ZeroVector;
    if (Window > 0.0 && static_cast<size_t>(I) < Before.size()) {
      const swarm::AgentSnapshot& B = Before[I];
      Acceleration = FVector((A.velocity[0] - B.velocity[0]) / Window,
                             -(A.velocity[1] - B.velocity[1]) / Window,
                             (A.velocity[2] - B.velocity[2]) / Window);
    }
    const SwarmSim::FDroneAttitude Target =
        bAircraftAttitude
            ? SwarmSim::AircraftAttitudeFromMotion(Velocity, Acceleration, Drone.YawDeg, MaxTiltDeg)
            : SwarmSim::AttitudeFromMotion(Velocity, Acceleration, Drone.YawDeg, MaxTiltDeg);
    if (bFirst) {
      Drone.YawDeg = Target.YawDeg;
      Drone.PitchDeg = Target.PitchDeg;
      Drone.RollDeg = Target.RollDeg;
    } else {
      Drone.YawDeg =
          SwarmSim::SmoothAngleDeg(Drone.YawDeg, Target.YawDeg, DeltaSeconds, kAttitudeSmoothing);
      Drone.PitchDeg = SwarmSim::SmoothAngleDeg(Drone.PitchDeg, Target.PitchDeg, DeltaSeconds,
                                                kAttitudeSmoothing);
      Drone.RollDeg =
          SwarmSim::SmoothAngleDeg(Drone.RollDeg, Target.RollDeg, DeltaSeconds, kAttitudeSmoothing);
    }
    if (Drone.Root != nullptr) {
      Drone.Root->SetWorldLocationAndRotation(
          Location, FRotator(Drone.PitchDeg, Drone.YawDeg, Drone.RollDeg));
    }

    // State color, only when it changes.
    const FLinearColor Color = SwarmSim::StateColor(A.fallback_state, A.alive);
    if (!Color.Equals(Drone.Color, 1e-4f)) {
      Drone.Color = Color;
      if (Drone.Paint != nullptr) {
        Drone.Paint->SetVectorParameterValue(kColorParameter, Color);
      }
    }
    Drone.bAlive = A.alive;

    // Rotors spin while the drone is alive.
    if (A.alive && !bFirst) {
      const double Rate =
          Drone.Propeller != nullptr
              ? kPropellerDegPerMps * FVector(A.velocity[0], A.velocity[1], A.velocity[2]).Size()
              : RotorSpinDegPerSecond;
      Drone.SpinDeg = FMath::Fmod(Drone.SpinDeg + Rate * DeltaSeconds, 360.0);
    }
    if (Drone.Propeller != nullptr) {
      Drone.Propeller->SetRelativeRotation(
          kModelTurn().Quaternion() *
          FQuat(FVector::YAxisVector, FMath::DegreesToRadians(Drone.SpinDeg)));
    }
    for (int32 R = 0; R < Drone.Rotors.Num(); ++R) {
      if (Drone.Rotors[R] != nullptr) {
        Drone.Rotors[R]->SetRelativeRotation(
            FRotator(0.0, (R % 2 == 0 ? 1.0 : -1.0) * Drone.SpinDeg, 0.0));
      }
    }
  }
}

void ASwarmPlayback::UpdateCamera(double DeltaSeconds) {
  if (CameraMode == ESwarmCameraMode::Free || Drones.IsEmpty()) {
    return;
  }
  // The centroid of the drones that are alive (of all, if none is).
  FVector Centroid = FVector::ZeroVector;
  int32 Used = 0;
  for (const FDrone& Drone : Drones) {
    if (Drone.Root != nullptr && Drone.bAlive) {
      Centroid += Drone.Root->GetComponentLocation();
      ++Used;
    }
  }
  if (Used == 0) {
    for (const FDrone& Drone : Drones) {
      if (Drone.Root != nullptr) {
        Centroid += Drone.Root->GetComponentLocation();
        ++Used;
      }
    }
  }
  if (Used == 0) {
    return;
  }
  Centroid /= Used;

  // How far the drones spread from the centroid, so that a swarm on orbits several hundred meters
  // across stays in frame; the fixed distances below suit a swarm of tens of meters. Counted over
  // the same drones as the centroid, so an aircraft that failed and flew on does not pull it out.
  double Extent = 0.0;
  for (const FDrone& Drone : Drones) {
    if (Drone.Root != nullptr && (Drone.bAlive || AliveCount == 0)) {
      Extent = FMath::Max(Extent, FVector::Dist2D(Drone.Root->GetComponentLocation(), Centroid));
    }
  }

  FVector Target = Centroid;
  FVector Wanted = CameraLocation;
  const double T = CameraClock;
  switch (CameraMode) {
    case ESwarmCameraMode::Orbit: {
      const double Angle = FMath::DegreesToRadians(6.0 * T);
      const double Radius = FMath::Max(8000.0, 1.5 * Extent);
      Wanted =
          Centroid + FVector(FMath::Cos(Angle) * Radius, FMath::Sin(Angle) * Radius, 0.45 * Radius);
      break;
    }
    case ESwarmCameraMode::Follow: {
      const FDrone& Drone = Drones[FMath::Clamp(FollowIndex, 0, Drones.Num() - 1)];
      if (Drone.Root == nullptr) {
        return;
      }
      Target = Drone.Root->GetComponentLocation();
      const FVector Forward = FRotator(0.0, Drone.YawDeg, 0.0).Vector();
      Wanted = Target - Forward * 1500.0 + FVector(0.0, 0.0, 500.0);
      break;
    }
    case ESwarmCameraMode::Cinematic: {
      const double Angle = FMath::DegreesToRadians(3.0 * T);
      const double Radius = FMath::Max((110.0 + 50.0 * FMath::Sin(0.05 * T)) * 100.0, 1.5 * Extent);
      const double Height =
          FMath::Max((30.0 + 25.0 * FMath::Sin(0.031 * T + 1.0)) * 100.0, 0.4 * Extent);
      Wanted = Centroid + FVector(FMath::Cos(Angle) * Radius, FMath::Sin(Angle) * Radius, Height);
      break;
    }
    default:
      return;
  }
  // A camera never sits inside the level: when something is between the target and the wanted
  // place, the camera stops in front of it.
  if (UWorld* World = GetWorld()) {
    FHitResult Hit;
    FCollisionQueryParams Params(SCENE_QUERY_STAT(SwarmCamera), false, this);
    if (World->LineTraceSingleByChannel(Hit, Target, Wanted, ECC_Visibility, Params)) {
      Wanted = Hit.Location + (Target - Wanted).GetSafeNormal() * 300.0;
    }
  }
  CameraLocation =
      CameraClock < 1e-6 ? Wanted : FMath::VInterpTo(CameraLocation, Wanted, DeltaSeconds, 3.0f);
  Camera->SetWorldLocationAndRotation(CameraLocation, (Target - CameraLocation).Rotation());
}

FVector ASwarmPlayback::GetDroneLocation(int32 Index) const {
  return Drones.IsValidIndex(Index) && Drones[Index].Root != nullptr
             ? Drones[Index].Root->GetComponentLocation()
             : FVector::ZeroVector;
}

FRotator ASwarmPlayback::GetDroneRotation(int32 Index) const {
  return Drones.IsValidIndex(Index) && Drones[Index].Root != nullptr
             ? Drones[Index].Root->GetComponentRotation()
             : FRotator::ZeroRotator;
}

FLinearColor ASwarmPlayback::GetDroneColor(int32 Index) const {
  return Drones.IsValidIndex(Index) ? Drones[Index].Color : FLinearColor::Black;
}

FString ASwarmPlayback::GetHashString() const {
  if (!Replay.IsValid()) {
    return FString();
  }
  const swarm::ReplayFrame* Frame = Replay->frame_at_or_before(PlaybackTick);
  return Frame != nullptr ? SwarmSim::HashString(Frame->hash) : FString();
}

namespace {

// How far back the messages shown and counted reach, in seconds of play time.
constexpr double kMessageDrawSeconds = 0.6;
constexpr double kMessageCountSeconds = 1.0;

// Calls `Visit` with every message event of the frames in the last `WindowSeconds` before `Tick`,
// and the age of its frame in seconds. The frames are in tick order.
void VisitRecentMessages(const swarm::Replay& Replay, double Tick, double Dt, double WindowSeconds,
                         TFunctionRef<void(const swarm::SnapshotEvent&, double)> Visit) {
  const swarm::ReplayFrame* Last = Replay.frame_at_or_before(Tick);
  if (Last == nullptr || !(Dt > 0.0)) {
    return;
  }
  const std::vector<swarm::ReplayFrame>& Frames = Replay.frames();
  const double WindowTicks = WindowSeconds / Dt;
  for (std::ptrdiff_t I = Last - Frames.data(); I >= 0; --I) {
    const double Age = Tick - static_cast<double>(Frames[static_cast<size_t>(I)].tick);
    if (Age > WindowTicks) {
      break;
    }
    for (const swarm::SnapshotEvent& Event : Frames[static_cast<size_t>(I)].snapshot.events) {
      Visit(Event, Age * Dt);
    }
  }
}

// The paint of a message line by outcome: delivered, lost, rate limited, jammed.
int32 MessagePaintIndex(swarm::SnapshotEventKind Kind) {
  switch (Kind) {
    case swarm::SnapshotEventKind::kScheduled:
      return 0;
    case swarm::SnapshotEventKind::kDroppedLoss:
      return 1;
    case swarm::SnapshotEventKind::kDroppedRate:
      return 2;
    default:
      return 3;
  }
}

// The most lines drawn at once, and how wide a line is: a fixed fraction of its distance from the
// camera, so that it stays a couple of pixels wide whether the camera is 15 m or 500 m away, within
// limits in centimeters.
constexpr int32 kMaxMessageLines = 400;
constexpr double kMessageLineWidthPerCm = 0.005;
constexpr double kMessageLineMinCm = 2.0;
constexpr double kMessageLineMaxCm = 120.0;

const TCHAR* FallbackName(int32 State) {
  switch (State) {
    case 0:
      return TEXT("nominal");
    case 1:
      return TEXT("degraded comms");
    case 2:
      return TEXT("degraded navigation");
    default:
      return TEXT("safe hold");
  }
}

}  // namespace

void ASwarmPlayback::BuildHud(TArray<FString>& Lines, TArray<FColor>& Colors) const {
  const auto Add = [&](const FString& Text, const FColor& Color) {
    Lines.Add(Text);
    Colors.Add(Color);
  };
  if (!Replay.IsValid()) {
    Add(TEXT("SwarmSim replay: no log loaded"), FColor::Cyan);
    return;
  }
  const double Dt = Replay->header().dt;
  const double Total = static_cast<double>(Replay->last_tick()) * Dt;
  Add(TEXT("SwarmSim replay"), FColor::Cyan);
  Add(FString::Printf(TEXT("tick %.0f of %llu   time %.1f s of %.1f s"), PlaybackTick,
                      static_cast<uint64>(Replay->last_tick()), PlaybackTick * Dt, Total),
      FColor::White);
  Add(FString::Printf(TEXT("speed %gx%s"), Speed, bPaused ? TEXT("  (paused)") : TEXT("")),
      FColor::White);
  Add(FString::Printf(TEXT("drones alive %d of %d"), AliveCount, Drones.Num()), FColor::White);
  Add(FString::Printf(TEXT("state hash %s"), *GetHashString()), FColor::White);
  static const TCHAR* Names[] = {TEXT("free"), TEXT("orbit"), TEXT("follow"), TEXT("cinematic")};
  Add(FString::Printf(TEXT("camera %s   seed %llu"), Names[static_cast<int32>(CameraMode)],
                      static_cast<uint64>(Replay->header().seed)),
      FColor::White);
  if (Drones.IsEmpty()) {
    return;
  }

  // The speed in the log of the drone the camera follows (Tab chooses it).
  const int32 Followed = FMath::Clamp(FollowIndex, 0, Drones.Num() - 1);
  const FDrone& Own = Drones[Followed];
  Add(FString::Printf(TEXT("drone %d speed %.1f m/s (%.0f km/h, %.0f mph)"), Followed, Own.SpeedMps,
                      Own.SpeedMps * 3.6, Own.SpeedMps * 2.2369363),
      FColor::White);
  Add(FString::Printf(TEXT("drone %d altitude %.0f m   climb %+.1f m/s   heading %03d deg"),
                      Followed, Own.AltitudeM, Own.ClimbMps,
                      FMath::RoundToInt(Own.HeadingDeg) % 360),
      FColor::White);

  // The fallback states. A model drawn from a folder has no color to show them, so they are words
  // here: green while every drone is nominal, red as soon as one is not.
  const int32 Failed = Drones.Num() - AliveCount;
  if (StateCounts[1] + StateCounts[2] + StateCounts[3] + Failed == 0) {
    Add(TEXT("fallback: all drones nominal"), FColor::Green);
  } else {
    FString Text = TEXT("FALLBACK ACTIVE");
    for (int32 State = 1; State <= 3; ++State) {
      if (StateCounts[State] > 0) {
        Text += FString::Printf(TEXT("   %s %d"), FallbackName(State), StateCounts[State]);
      }
    }
    if (Failed > 0) {
      Text += FString::Printf(TEXT("   failed %d"), Failed);
    }
    Add(Text, FColor::Red);
  }
  // The messages of the last second: how many were delivered, lost, and jammed.
  int32 Delivered = 0;
  int32 Lost = 0;
  int32 Jammed = 0;
  VisitRecentMessages(*Replay, PlaybackTick, Dt, kMessageCountSeconds,
                      [&](const swarm::SnapshotEvent& Event, double) {
                        switch (Event.kind) {
                          case swarm::SnapshotEventKind::kScheduled:
                            ++Delivered;
                            break;
                          case swarm::SnapshotEventKind::kDroppedJam:
                            ++Jammed;
                            break;
                          default:
                            ++Lost;
                            break;
                        }
                      });
  Add(FString::Printf(TEXT("messages, last 1 s: delivered %d   lost %d   jammed %d%s"), Delivered,
                      Lost, Jammed, bShowMessages ? TEXT("") : TEXT("   (M: 3D lines)")),
      Jammed > 0 ? FColor::Red : FColor::White);
  const bool bOut = !Own.bAlive || Own.FallbackState != 0;
  const FString OwnState = Own.bAlive ? FString(FallbackName(Own.FallbackState)) : TEXT("failed");
  Add(FString::Printf(TEXT("drone %d state: %s"), Followed,
                      *(bOut ? OwnState.ToUpper() : OwnState)),
      bOut ? FColor::Red : FColor::White);
}

TArray<FString> ASwarmPlayback::GetHudLines() const {
  TArray<FString> Lines;
  TArray<FColor> Colors;
  BuildHud(Lines, Colors);
  return Lines;
}

TArray<FColor> ASwarmPlayback::GetHudColors() const {
  TArray<FString> Lines;
  TArray<FColor> Colors;
  BuildHud(Lines, Colors);
  return Colors;
}

void ASwarmPlayback::HideMessageLines() {
  for (UStaticMeshComponent* Line : MessageLines) {
    if (Line != nullptr) {
      Line->SetVisibility(false);
    }
  }
  bMessagesDrawn = false;
}

void ASwarmPlayback::DrawMessages() {
  if (!Replay.IsValid()) {
    return;
  }
  if (MessagePaints.IsEmpty()) {
    MessageShape = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
    UMaterialInterface* Basic = LoadObject<UMaterialInterface>(
        nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
    // The colors of the fallback states: delivered green, lost yellow, rate limited orange,
    // jammed red.
    const FLinearColor Colors[4] = {SwarmSim::StateColor(0, true), SwarmSim::StateColor(1, true),
                                    SwarmSim::StateColor(2, true), SwarmSim::StateColor(3, true)};
    for (const FLinearColor& Color : Colors) {
      UMaterialInstanceDynamic* Paint =
          Basic != nullptr ? UMaterialInstanceDynamic::Create(Basic, this) : nullptr;
      if (Paint != nullptr) {
        Paint->SetVectorParameterValue(kColorParameter, Color);
      }
      MessagePaints.Add(Paint);
    }
  }
  if (MessageShape == nullptr) {
    return;
  }
  int32 Used = 0;
  VisitRecentMessages(
      *Replay, PlaybackTick, Replay->header().dt, kMessageDrawSeconds,
      [&](const swarm::SnapshotEvent& Event, double) {
        if (Used >= kMaxMessageLines || Event.sender >= static_cast<uint32>(Drones.Num()) ||
            Event.receiver >= static_cast<uint32>(Drones.Num())) {
          return;
        }
        const FDrone& From = Drones[static_cast<int32>(Event.sender)];
        const FDrone& To = Drones[static_cast<int32>(Event.receiver)];
        if (From.Root == nullptr || To.Root == nullptr) {
          return;
        }
        const FVector Start = From.Root->GetComponentLocation();
        const FVector Direction = To.Root->GetComponentLocation() - Start;
        const FVector Seen =
            Camera != nullptr ? Camera->GetComponentLocation() : FVector::ZeroVector;
        const double Near =
            FMath::Min(FVector::Dist(Seen, Start), FVector::Dist(Seen, Start + Direction));
        const double WidthCm =
            FMath::Clamp(Near * kMessageLineWidthPerCm, kMessageLineMinCm, kMessageLineMaxCm);
        const double Length = Direction.Size();
        if (Length < 1.0) {
          return;
        }
        if (Used >= MessageLines.Num()) {
          UStaticMeshComponent* Line = NewObject<UStaticMeshComponent>(this);
          Line->SetStaticMesh(MessageShape);
          Line->SetCollisionEnabled(ECollisionEnabled::NoCollision);
          Line->SetCastShadow(false);
          Line->SetupAttachment(GetRootComponent());
          Line->RegisterComponent();
          AddInstanceComponent(Line);
          MessageLines.Add(Line);
        }
        UStaticMeshComponent* Line = MessageLines[Used++];
        // The cylinder is 100 cm tall and 100 cm across, centered, standing along Z.
        Line->SetWorldLocationAndRotation(Start + 0.5 * Direction,
                                          FRotationMatrix::MakeFromZ(Direction).Rotator());
        Line->SetWorldScale3D(FVector(WidthCm / 100.0, WidthCm / 100.0, Length / 100.0));
        Line->SetMaterial(0, MessagePaints[MessagePaintIndex(Event.kind)]);
        Line->SetVisibility(true);
      });
  for (int32 I = Used; I < MessageLines.Num(); ++I) {
    if (MessageLines[I] != nullptr) {
      MessageLines[I]->SetVisibility(false);
    }
  }
  bMessagesDrawn = true;
}

void ASwarmPlayback::DrawMap(UCanvas* Canvas, APlayerController*) {
  if (!bShowMap || Canvas == nullptr || !Replay.IsValid() || Drones.IsEmpty() || !(MapSize > 0.0) ||
      Canvas->DefaultTexture == nullptr) {
    return;
  }
  const SwarmSim::FMapExtent Extent{FVector2D(MapMinX, MapMinY), MapSize};
  const double Side = FMath::Clamp(0.3 * Canvas->SizeY, 140.0, 380.0);
  const FVector2D Origin(Canvas->SizeX - Side - 16.0, 16.0);

  const auto Tile = [&](const FVector2D& Position, const FVector2D& Size,
                        const FLinearColor& Color) {
    FCanvasTileItem Item(Position, Canvas->DefaultTexture->GetResource(), Size, Color);
    Item.BlendMode = SE_BLEND_Translucent;
    Canvas->DrawItem(Item);
  };
  const auto Line = [&](const FVector2D& From, const FVector2D& To, const FLinearColor& Color,
                        float Thickness) {
    FCanvasLineItem Item(From, To);
    Item.SetColor(Color);
    Item.LineThickness = Thickness;
    Canvas->DrawItem(Item);
  };
  const auto Text = [&](const FString& Label, const FVector2D& Position,
                        const FLinearColor& Color) {
    FCanvasTextItem Item(Position, FText::FromString(Label), GEngine->GetSmallFont(), Color);
    Item.EnableShadow(FLinearColor::Black);
    Canvas->DrawItem(Item);
  };

  // The background, a border, and north.
  Tile(Origin, FVector2D(Side, Side), FLinearColor(0.0f, 0.0f, 0.0f, 0.55f));
  const FLinearColor Border(0.8f, 0.8f, 0.8f, 0.8f);
  Line(Origin, Origin + FVector2D(Side, 0.0), Border, 1.0f);
  Line(Origin + FVector2D(Side, 0.0), Origin + FVector2D(Side, Side), Border, 1.0f);
  Line(Origin + FVector2D(Side, Side), Origin + FVector2D(0.0, Side), Border, 1.0f);
  Line(Origin + FVector2D(0.0, Side), Origin, Border, 1.0f);
  Text(TEXT("N"), Origin + FVector2D(0.5 * Side - 3.0, 3.0), FLinearColor::White);

  // A scale bar of a round length, about a third of the width.
  const double BarMeters = SwarmSim::ScaleBarMeters(Extent.Size / 3.0);
  if (BarMeters > 0.0) {
    const double BarPixels = BarMeters / Extent.Size * Side;
    const FVector2D BarStart = Origin + FVector2D(8.0, Side - 12.0);
    Line(BarStart, BarStart + FVector2D(BarPixels, 0.0), FLinearColor::White, 2.0f);
    Text(FString::Printf(TEXT("%.0f m"), BarMeters), BarStart + FVector2D(0.0, -14.0),
         FLinearColor::White);
  }

  // Trails: the last few seconds of every drone, dimmer the older.
  const double Dt = Replay->header().dt;
  const int32 TrailSteps = 12;
  const double StepTicks = Dt > 0.0 ? 0.5 / Dt : 25.0;
  std::vector<swarm::AgentSnapshot> Past;
  std::vector<std::vector<FVector2D>> Trails(static_cast<size_t>(Drones.Num()));
  for (int32 K = 1; K <= TrailSteps; ++K) {
    const double Tick = FMath::Max(static_cast<double>(Replay->first_tick()),
                                   PlaybackTick - static_cast<double>(K) * StepTicks);
    if (!Replay->sample(Tick, Past)) {
      break;
    }
    for (int32 I = 0; I < Drones.Num() && static_cast<size_t>(I) < Past.size(); ++I) {
      Trails[static_cast<size_t>(I)].push_back(
          SwarmSim::MapPoint(Extent, Origin, Side, Past[static_cast<size_t>(I)].position[0],
                             Past[static_cast<size_t>(I)].position[1]));
    }
  }

  std::vector<swarm::AgentSnapshot> Now;
  if (!Replay->sample(PlaybackTick, Now)) {
    return;
  }
  // Messages of the last moments, as lines between the drones in the colors of the 3D lines:
  // delivered faint, the others (lost, rate limited, jammed) strong, so a jam shows as red lines.
  VisitRecentMessages(
      *Replay, PlaybackTick, Dt, kMessageDrawSeconds,
      [&](const swarm::SnapshotEvent& Event, double) {
        if (Event.sender >= Now.size() || Event.receiver >= Now.size()) {
          return;
        }
        const swarm::AgentSnapshot& From = Now[Event.sender];
        const swarm::AgentSnapshot& To = Now[Event.receiver];
        FLinearColor Paint = SwarmSim::StateColor(MessagePaintIndex(Event.kind), true);
        const bool bDelivered = Event.kind == swarm::SnapshotEventKind::kScheduled;
        Paint.A = bDelivered ? 0.25f : 0.9f;
        Line(SwarmSim::MapPoint(Extent, Origin, Side, From.position[0], From.position[1]),
             SwarmSim::MapPoint(Extent, Origin, Side, To.position[0], To.position[1]), Paint,
             bDelivered ? 1.0f : 1.5f);
      });
  const int32 Followed = FMath::Clamp(FollowIndex, 0, Drones.Num() - 1);
  for (int32 I = 0; I < Drones.Num() && static_cast<size_t>(I) < Now.size(); ++I) {
    const swarm::AgentSnapshot& A = Now[static_cast<size_t>(I)];
    const FLinearColor Color = SwarmSim::StateColor(A.fallback_state, A.alive);
    const FVector2D Here = SwarmSim::MapPoint(Extent, Origin, Side, A.position[0], A.position[1]);
    FVector2D Previous = Here;
    const std::vector<FVector2D>& Trail = Trails[static_cast<size_t>(I)];
    for (size_t K = 0; K < Trail.size(); ++K) {
      const float Fade = 0.6f * (1.0f - static_cast<float>(K) / static_cast<float>(TrailSteps));
      Line(Previous, Trail[K], FLinearColor(Color.R, Color.G, Color.B, Fade), 1.0f);
      Previous = Trail[K];
    }
    if (!A.alive) {
      // A failed drone is a gray cross where it stopped.
      const FLinearColor Gray(0.6f, 0.6f, 0.6f, 1.0f);
      Line(Here + FVector2D(-4.0, -4.0), Here + FVector2D(4.0, 4.0), Gray, 2.0f);
      Line(Here + FVector2D(-4.0, 4.0), Here + FVector2D(4.0, -4.0), Gray, 2.0f);
      continue;
    }
    // Heading: the velocity, with +Y up the screen.
    const double PlaneSpeed =
        FMath::Sqrt(A.velocity[0] * A.velocity[0] + A.velocity[1] * A.velocity[1]);
    if (PlaneSpeed > 0.5) {
      Line(Here, Here + FVector2D(A.velocity[0], -A.velocity[1]) / PlaneSpeed * 14.0, Color, 2.0f);
    }
    Tile(Here - FVector2D(4.0, 4.0), FVector2D(8.0, 8.0), Color);
    if (I == Followed) {
      const FLinearColor White = FLinearColor::White;
      Line(Here + FVector2D(-8.0, -8.0), Here + FVector2D(8.0, -8.0), White, 1.5f);
      Line(Here + FVector2D(8.0, -8.0), Here + FVector2D(8.0, 8.0), White, 1.5f);
      Line(Here + FVector2D(8.0, 8.0), Here + FVector2D(-8.0, 8.0), White, 1.5f);
      Line(Here + FVector2D(-8.0, 8.0), Here + FVector2D(-8.0, -8.0), White, 1.5f);
    }
  }
}

void ASwarmPlayback::UpdatePip() {
  if (PipCapture == nullptr || Drones.IsEmpty()) {
    return;
  }
  const int32 Followed = FMath::Clamp(FollowIndex, 0, Drones.Num() - 1);
  const FDrone& Own = Drones[Followed];
  if (Own.Root == nullptr) {
    return;
  }
  const FVector Location = Own.Root->GetComponentLocation();

  // The height above the surface below the followed drone, for the window and the strip.
  HeightAboveTerrainM = -1.0;
  if (UWorld* World = GetWorld()) {
    FHitResult Hit;
    FCollisionQueryParams Params(SCENE_QUERY_STAT(SwarmHeight), false, this);
    if (World->LineTraceSingleByChannel(Hit, Location, Location - FVector(0.0, 0.0, 200000.0),
                                        ECC_Visibility, Params)) {
      HeightAboveTerrainM = Hit.Distance / 100.0;
    }
  }

  // Capture only while the window is shown.
  PipCapture->bCaptureEveryFrame = bShowPip;
  if (!bShowPip) {
    return;
  }
  if (PipDrone != Followed) {
    // The capture does not draw the drone it sits on.
    PipCapture->ClearHiddenComponents();
    TArray<USceneComponent*> Parts;
    Own.Root->GetChildrenComponents(true, Parts);
    Parts.Add(Own.Root);
    for (USceneComponent* Part : Parts) {
      if (UPrimitiveComponent* Primitive = Cast<UPrimitiveComponent>(Part)) {
        PipCapture->HideComponent(Primitive);
      }
    }
    PipDrone = Followed;
  }
  PipCapture->SetWorldLocationAndRotation(Location,
                                          FRotator(Own.PitchDeg, Own.YawDeg, Own.RollDeg));
}

namespace {
void CanvasTile(UCanvas* Canvas, const FVector2D& Position, const FVector2D& Size,
                const FLinearColor& Color) {
  FCanvasTileItem Item(Position, Canvas->DefaultTexture->GetResource(), Size, Color);
  Item.BlendMode = SE_BLEND_Translucent;
  Canvas->DrawItem(Item);
}

void CanvasLine(UCanvas* Canvas, const FVector2D& From, const FVector2D& To,
                const FLinearColor& Color, float Thickness) {
  FCanvasLineItem Item(From, To);
  Item.SetColor(Color);
  Item.LineThickness = Thickness;
  Canvas->DrawItem(Item);
}

void CanvasText(UCanvas* Canvas, const FString& Label, const FVector2D& Position,
                const FLinearColor& Color) {
  FCanvasTextItem Item(Position, FText::FromString(Label), GEngine->GetSmallFont(), Color);
  Item.EnableShadow(FLinearColor::Black);
  Canvas->DrawItem(Item);
}

void CanvasBox(UCanvas* Canvas, const FVector2D& Origin, const FVector2D& Size,
               const FLinearColor& Color) {
  CanvasLine(Canvas, Origin, Origin + FVector2D(Size.X, 0.0), Color, 1.0f);
  CanvasLine(Canvas, Origin + FVector2D(Size.X, 0.0), Origin + Size, Color, 1.0f);
  CanvasLine(Canvas, Origin + Size, Origin + FVector2D(0.0, Size.Y), Color, 1.0f);
  CanvasLine(Canvas, Origin + FVector2D(0.0, Size.Y), Origin, Color, 1.0f);
}
}  // namespace

void ASwarmPlayback::DrawPanels(UCanvas* Canvas, APlayerController*) {
  if (Canvas == nullptr || !Replay.IsValid() || Drones.IsEmpty() ||
      Canvas->DefaultTexture == nullptr) {
    return;
  }
  const double MapSide = bShowMap ? FMath::Clamp(0.3 * Canvas->SizeY, 140.0, 380.0) : 0.0;
  if (bShowPip) {
    DrawPip(Canvas, MapSide);
  }
  if (bShowTelemetry) {
    DrawTelemetry(Canvas);
  }
}

void ASwarmPlayback::DrawPip(UCanvas* Canvas, double MapSide) const {
  if (PipTarget == nullptr) {
    return;
  }
  FTextureRenderTargetResource* Resource = PipTarget->GameThread_GetRenderTargetResource();
  if (Resource == nullptr) {
    return;
  }
  const int32 Followed = FMath::Clamp(FollowIndex, 0, Drones.Num() - 1);
  const FDrone& Own = Drones[Followed];
  const double Width = FMath::Clamp(0.3 * Canvas->SizeX, 200.0, 480.0);
  const FVector2D Size(Width, Width * kPipHeight / kPipWidth);
  const FVector2D Origin(Canvas->SizeX - Size.X - 16.0,
                         16.0 + (MapSide > 0.0 ? MapSide + 10.0 : 0.0));

  FCanvasTileItem Image(Origin, Resource, Size, FLinearColor::White);
  Image.BlendMode = SE_BLEND_Opaque;
  Canvas->DrawItem(Image);
  CanvasBox(Canvas, Origin, Size, FLinearColor(0.8f, 0.8f, 0.8f, 0.8f));

  // A small cross at the center, and the labels. The window is the viewer's view, not a sensor.
  const FVector2D Center = Origin + 0.5 * Size;
  const FLinearColor Cross(1.0f, 1.0f, 1.0f, 0.6f);
  CanvasLine(Canvas, Center + FVector2D(-10.0, 0.0), Center + FVector2D(-3.0, 0.0), Cross, 1.0f);
  CanvasLine(Canvas, Center + FVector2D(3.0, 0.0), Center + FVector2D(10.0, 0.0), Cross, 1.0f);
  CanvasLine(Canvas, Center + FVector2D(0.0, -10.0), Center + FVector2D(0.0, -3.0), Cross, 1.0f);
  CanvasLine(Canvas, Center + FVector2D(0.0, 3.0), Center + FVector2D(0.0, 10.0), Cross, 1.0f);
  CanvasText(Canvas, FString::Printf(TEXT("VIEWER CAMERA   drone %d"), Followed),
             Origin + FVector2D(6.0, 4.0), FLinearColor::White);
  FString Lower =
      FString::Printf(TEXT("%03d deg   %.0f m/s   alt %.0f m"),
                      FMath::RoundToInt(Own.HeadingDeg) % 360, Own.SpeedMps, Own.AltitudeM);
  if (HeightAboveTerrainM >= 0.0) {
    Lower += FString::Printf(TEXT("   above terrain %.0f m"), HeightAboveTerrainM);
  }
  CanvasText(Canvas, Lower, Origin + FVector2D(6.0, Size.Y - 18.0), FLinearColor::White);
}

void ASwarmPlayback::DrawTelemetry(UCanvas* Canvas) const {
  const double Dt = Replay->header().dt;
  if (!(Dt > 0.0)) {
    return;
  }
  const int32 Followed = FMath::Clamp(FollowIndex, 0, Drones.Num() - 1);
  const double Width = FMath::Clamp(0.3 * Canvas->SizeX, 260.0, 520.0);
  const double Height = FMath::Clamp(0.17 * Canvas->SizeY, 100.0, 170.0);
  const FVector2D Size(Width, Height);
  const FVector2D Origin(16.0, Canvas->SizeY - Height - 16.0);

  // Samples of the followed drone, oldest first, over the last kTelemetrySeconds.
  struct FSample {
    double Altitude;
    double Speed;
    FLinearColor State;
  };
  std::vector<FSample> Samples;
  std::vector<swarm::AgentSnapshot> Frame;
  const double StepTicks = kTelemetrySeconds / kTelemetrySamples / Dt;
  for (int32 K = kTelemetrySamples - 1; K >= 0; --K) {
    const double Tick = FMath::Max(static_cast<double>(Replay->first_tick()),
                                   PlaybackTick - static_cast<double>(K) * StepTicks);
    if (!Replay->sample(Tick, Frame) || static_cast<size_t>(Followed) >= Frame.size()) {
      return;
    }
    const swarm::AgentSnapshot& A = Frame[static_cast<size_t>(Followed)];
    Samples.push_back({A.position[2], FVector(A.velocity[0], A.velocity[1], A.velocity[2]).Size(),
                       SwarmSim::StateColor(A.fallback_state, A.alive)});
  }
  if (Samples.size() < 2) {
    return;
  }
  double AltLow = Samples[0].Altitude;
  double AltHigh = AltLow;
  double SpeedLow = Samples[0].Speed;
  double SpeedHigh = SpeedLow;
  for (const FSample& S : Samples) {
    AltLow = FMath::Min(AltLow, S.Altitude);
    AltHigh = FMath::Max(AltHigh, S.Altitude);
    SpeedLow = FMath::Min(SpeedLow, S.Speed);
    SpeedHigh = FMath::Max(SpeedHigh, S.Speed);
  }
  // A flat trace is not stretched to fill the panel.
  const auto Widen = [](double& Low, double& High, double MinSpan) {
    if (High - Low < MinSpan) {
      const double Middle = 0.5 * (High + Low);
      Low = Middle - 0.5 * MinSpan;
      High = Middle + 0.5 * MinSpan;
    }
  };
  Widen(AltLow, AltHigh, 20.0);
  Widen(SpeedLow, SpeedHigh, 10.0);
  if (SpeedLow < 0.0) {
    SpeedHigh -= SpeedLow;
    SpeedLow = 0.0;
  }

  CanvasTile(Canvas, Origin, Size, FLinearColor(0.0f, 0.0f, 0.0f, 0.55f));
  CanvasBox(Canvas, Origin, Size, FLinearColor(0.8f, 0.8f, 0.8f, 0.8f));
  const double Top = Origin.Y + 18.0;
  const double Bottom = Origin.Y + Size.Y - 12.0;
  const double Left = Origin.X + 6.0;
  const double Right = Origin.X + Size.X - 6.0;
  const double Span = static_cast<double>(Samples.size() - 1);
  const FLinearColor AltColor(0.3f, 0.85f, 1.0f, 1.0f);
  const FLinearColor SpeedColor(1.0f, 0.7f, 0.2f, 1.0f);
  const auto Plot = [&](bool bAltitude, double Low, double High, const FLinearColor& Color) {
    FVector2D Previous = FVector2D::ZeroVector;
    for (size_t I = 0; I < Samples.size(); ++I) {
      const double Value = bAltitude ? Samples[I].Altitude : Samples[I].Speed;
      const FVector2D Point(Left + (Right - Left) * static_cast<double>(I) / Span,
                            Bottom - (Bottom - Top) * (Value - Low) / (High - Low));
      if (I > 0) {
        CanvasLine(Canvas, Previous, Point, Color, 1.5f);
      }
      Previous = Point;
    }
  };
  Plot(true, AltLow, AltHigh, AltColor);
  Plot(false, SpeedLow, SpeedHigh, SpeedColor);

  // The band under the traces: the followed drone's logged fallback state.
  for (size_t I = 0; I + 1 < Samples.size(); ++I) {
    const double X0 = Left + (Right - Left) * static_cast<double>(I) / Span;
    const double X1 = Left + (Right - Left) * static_cast<double>(I + 1) / Span;
    CanvasTile(Canvas, FVector2D(X0, Bottom + 3.0), FVector2D(X1 - X0 + 1.0, 5.0),
               Samples[I].State);
  }
  CanvasText(Canvas,
             FString::Printf(TEXT("drone %d, last %.0f s   alt %.0f to %.0f m   speed %.0f to %.0f"),
                             Followed, kTelemetrySeconds, AltLow, AltHigh, SpeedLow, SpeedHigh),
             Origin + FVector2D(6.0, 3.0), FLinearColor::White);
}

void ASwarmPlayback::DrawHud() const {
  if (GEngine == nullptr) {
    return;
  }
  TArray<FString> Lines;
  TArray<FColor> Colors;
  BuildHud(Lines, Colors);
  for (int32 I = 0; I < Lines.Num(); ++I) {
    GEngine->AddOnScreenDebugMessage(kHudKeyBase + static_cast<uint64>(I), 1.0f, Colors[I],
                                     Lines[I], false, FVector2D(1.4, 1.4));
  }
}

void ASwarmPlayback::ConfigureScreenshots(const FString& Directory, double DelaySeconds,
                                          double EverySeconds, int32 Count, bool bQuitAfter) {
  ShotDirectory = Directory;
  ShotDelay = DelaySeconds;
  ShotEvery = EverySeconds;
  ShotCount = Count;
  ShotsTaken = 0;
  ShotClock = 0.0;
  bQuitAfterShots = bQuitAfter;
}

void ASwarmPlayback::TakeScheduledShot(double DeltaSeconds) {
  if (ShotDirectory.IsEmpty() || ShotCount <= 0) {
    return;
  }
  ShotClock += DeltaSeconds;
  if (ShotsTaken < ShotCount) {
    if (SwarmSim::ShotDue(ShotClock, ShotDelay, ShotEvery, ShotsTaken, ShotCount)) {
      const FString Path = FPaths::ConvertRelativePathToFull(
          ShotDirectory / FString::Printf(TEXT("swarm_shot_%02d.png"), ShotsTaken));
      // The engine's own render of the game, not a capture of the desktop.
      if (APlayerController* Controller = GetWorld()->GetFirstPlayerController()) {
        Controller->ConsoleCommand(
            FString::Printf(TEXT("HighResShot 1600x900 filename=\"%s\""), *Path));
      }
      UE_LOG(LogSwarmSim, Display, TEXT("Screenshot %d of %d requested: %s"), ShotsTaken + 1,
             ShotCount, *Path);
      ++ShotsTaken;
      if (ShotsTaken == ShotCount) {
        ShotQuitClock = 0.0;
      }
    }
    return;
  }
  // After the last request, give the engine time to write the file before quitting.
  if (bQuitAfterShots) {
    ShotQuitClock += DeltaSeconds;
    if (ShotQuitClock > 8.0) {
      FPlatformMisc::RequestExit(false);
    }
  }
}

void ASwarmPlayback::ConfigureFrames(const FString& Directory, double Fps, double Seconds) {
  FrameDirectory = Directory;
  FrameFps = Fps;
  FrameTotal = static_cast<int32>(Seconds * Fps);
  FramesRequested = 0;
}
