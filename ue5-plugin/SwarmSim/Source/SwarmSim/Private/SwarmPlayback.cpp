#include "SwarmPlayback.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "Camera/CameraComponent.h"
#include "Camera/PlayerCameraManager.h"
#include "Components/InputComponent.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/DefaultPawn.h"
#include "GameFramework/PlayerController.h"
#include "HAL/PlatformMisc.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
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
// The propeller of a folder model turns this many degrees per meter per second of airspeed.
constexpr double kPropellerDegPerMps = 60.0;
// Mesh-name prefixes of the ALTIUS model's propeller: two blades and their two hubs. The glTF
// gives its nodes generic names; these were found from the world-space bounds of each mesh.
const TCHAR* const kPropellerParts[] = {TEXT("C-Ext001"), TEXT("C-Ext002"), TEXT("Cylinder015"),
                                        TEXT("Cylinder016")};
const TCHAR* const kPropellerHubs[] = {TEXT("Cylinder015"), TEXT("Cylinder016")};

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
}  // namespace

ASwarmPlayback::ASwarmPlayback() {
  PrimaryActorTick.bCanEverTick = true;
  Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
  SetRootComponent(Root);
  Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
  Camera->SetupAttachment(Root);
  Camera->SetFieldOfView(70.0f);
}

void ASwarmPlayback::BeginPlay() {
  Super::BeginPlay();
  BindKeys();
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
  DestroyDrones();
  BuildDrones(static_cast<int32>(Replay->header().agent_count));
  PlaybackTick = static_cast<double>(Replay->first_tick());
  FollowIndex = 0;
  ApplyFrame(0.0, true);
  UE_LOG(LogSwarmSim, Display, TEXT("Loaded %s: %d drones, %d frames, ticks %llu to %llu"), *Path,
         Drones.Num(), static_cast<int32>(Replay->frames().size()),
         static_cast<uint64>(Replay->first_tick()), static_cast<uint64>(Replay->last_tick()));
  return true;
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
      // The model's materials have no color parameter, so a lamp on top of the fuselage carries
      // the state color. The lamp's component holds its material, which keeps it from being
      // collected (FDrone is not a UPROPERTY).
      if (Basic != nullptr && Sphere != nullptr) {
        Drone.Paint = UMaterialInstanceDynamic::Create(Basic, this);
        AddPart(Drone.Root, Sphere, FVector(0.0, 0.0, 0.5 * kModelSpanCm * 0.18),
                FRotator::ZeroRotator, FVector(0.45), Drone.Paint);
      }
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
  const int32 Count = FMath::Min(Drones.Num(), static_cast<int32>(Now.size()));
  for (int32 I = 0; I < Count; ++I) {
    FDrone& Drone = Drones[I];
    const swarm::AgentSnapshot& A = Now[I];
    if (A.alive) {
      ++AliveCount;
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
        SwarmSim::AttitudeFromMotion(Velocity, Acceleration, Drone.YawDeg, MaxTiltDeg);
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

TArray<FString> ASwarmPlayback::GetHudLines() const {
  TArray<FString> Lines;
  if (!Replay.IsValid()) {
    Lines.Add(TEXT("SwarmSim replay: no log loaded"));
    return Lines;
  }
  const double Dt = Replay->header().dt;
  const double Total = static_cast<double>(Replay->last_tick()) * Dt;
  Lines.Add(TEXT("SwarmSim replay"));
  Lines.Add(FString::Printf(TEXT("tick %.0f of %llu   time %.1f s of %.1f s"), PlaybackTick,
                            static_cast<uint64>(Replay->last_tick()), PlaybackTick * Dt, Total));
  Lines.Add(FString::Printf(TEXT("speed %gx%s"), Speed, bPaused ? TEXT("  (paused)") : TEXT("")));
  Lines.Add(FString::Printf(TEXT("drones alive %d of %d"), AliveCount, Drones.Num()));
  Lines.Add(FString::Printf(TEXT("state hash %s"), *GetHashString()));
  static const TCHAR* Names[] = {TEXT("free"), TEXT("orbit"), TEXT("follow"), TEXT("cinematic")};
  Lines.Add(FString::Printf(TEXT("camera %s   seed %llu"), Names[static_cast<int32>(CameraMode)],
                            static_cast<uint64>(Replay->header().seed)));
  return Lines;
}

void ASwarmPlayback::DrawHud() const {
  if (GEngine == nullptr) {
    return;
  }
  const TArray<FString> Lines = GetHudLines();
  for (int32 I = 0; I < Lines.Num(); ++I) {
    GEngine->AddOnScreenDebugMessage(kHudKeyBase + static_cast<uint64>(I), 1.0f,
                                     I == 0 ? FColor::Cyan : FColor::White, Lines[I], false,
                                     FVector2D(1.4, 1.4));
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
