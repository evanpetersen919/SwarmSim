#include "SwarmSceneExporter.h"

#include "Engine/World.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "HAL/PlatformMisc.h"
#include "Misc/FileHelper.h"
#include "SwarmExportPlan.h"
#include "SwarmLog.h"
#include "SwarmSceneExport.h"
#include "swarm/format/scene.hpp"

ASwarmSceneExporter::ASwarmSceneExporter() { PrimaryActorTick.bCanEverTick = true; }

bool ASwarmSceneExporter::Configure(const FString& CommandLine, FString& OutError) {
  Ground = MakeShared<SwarmSim::FGroundSamples>();
  Top = MakeShared<SwarmSim::FTopSamples>();
  Options = MakeShared<SwarmSim::FExportOptions>(SwarmSim::ParseExportOptions(*CommandLine));
  if (!Options->bValid) {
    OutError = Options->Problem;
    return false;
  }
  if (Options->OutPath.IsEmpty() && Options->AuditPath.IsEmpty()) {
    OutError = TEXT("-SwarmExportScene or -SwarmAuditScene needs a file path");
    return false;
  }
  return true;
}

void ASwarmSceneExporter::MovePlayerToAnchor() const {
  const APlayerController* Controller =
      GetWorld() != nullptr ? GetWorld()->GetFirstPlayerController() : nullptr;
  APawn* Pawn = Controller != nullptr ? Controller->GetPawn() : nullptr;
  if (Pawn == nullptr || Options == nullptr) {
    return;
  }
  // Only x and y: the area streams in by distance in the plane.
  const FVector Here = Pawn->GetActorLocation();
  Pawn->SetActorLocation(FVector(Options->AnchorCm.X, Options->AnchorCm.Y, Here.Z), false, nullptr,
                         ETeleportType::TeleportPhysics);
}

void ASwarmSceneExporter::Tick(float DeltaSeconds) {
  Super::Tick(DeltaSeconds);
  if (bDone || Options == nullptr) {
    return;
  }
  Clock += DeltaSeconds;
  MoveClock += DeltaSeconds;
  if (MoveClock >= 1.0 || Clock == DeltaSeconds) {
    MoveClock = 0.0;
    MovePlayerToAnchor();
  }
  if (Clock < Options->WaitSeconds) {
    return;
  }
  if (!Options->AuditPath.IsEmpty()) {
    if (!Top->Complete() || Top->HeightsM.IsEmpty()) {
      if (Top->HeightsM.IsEmpty()) {
        Top->Reset(SwarmSim::CellsFor(Options->SizeM, Options->CellM));
        UE_LOG(LogSwarmSim, Display, TEXT("audit: sampling %d x %d top surfaces"), Top->Samples(),
               Top->Samples());
      }
      SwarmSim::SampleTopRows(GetWorld(), *Options, 10, *Top);
      if (Top->RowsDone % 20 == 0 || Top->Complete()) {
        UE_LOG(LogSwarmSim, Display, TEXT("audit: %d of %d rows"), Top->RowsDone, Top->Samples());
      }
      if (!Top->Complete()) {
        return;
      }
    }
    bDone = true;
    WriteAudit();
    FPlatformMisc::RequestExit(false);
    return;
  }
  // The ground is sampled a few rows a frame, so the game keeps running and the log shows how far
  // it has got; a whole level's area in one frame can take minutes.
  if (!Ground->Complete()) {
    if (Ground->HeightsM.IsEmpty()) {
      Ground->Reset(SwarmSim::CellsFor(Options->SizeM, Options->CellM));
      UE_LOG(LogSwarmSim, Display, TEXT("export: sampling %d x %d ground heights"),
             Ground->Samples(), Ground->Samples());
    }
    SwarmSim::SampleGroundRows(GetWorld(), *Options, 2, *Ground);
    if (Ground->RowsDone % 20 == 0 || Ground->Complete()) {
      UE_LOG(LogSwarmSim, Display, TEXT("export: %d of %d rows"), Ground->RowsDone,
             Ground->Samples());
    }
    return;
  }
  bDone = true;
  Export();
  FPlatformMisc::RequestExit(false);
}

void ASwarmSceneExporter::WriteAudit() {
  const FString Csv = SwarmSim::TopSamplesToCsv(*Options, *Top);
  if (!FFileHelper::SaveStringToFile(Csv, *Options->AuditPath)) {
    UE_LOG(LogSwarmSim, Error, TEXT("cannot write %s"), *Options->AuditPath);
    return;
  }
  int32 Known = 0;
  for (bool B : Top->Known) {
    Known += B ? 1 : 0;
  }
  UE_LOG(LogSwarmSim, Display, TEXT("Wrote the audit to %s: %d of %d samples have a surface"),
         *Options->AuditPath, Known, Top->Known.Num());
}

void ASwarmSceneExporter::Export() {
  swarm::SceneData Scene;
  SwarmSim::FExportReport Report;
  FString Error;
  if (!SwarmSim::FinishExport(GetWorld(), *Options, *Ground, Scene, Report, Error)) {
    UE_LOG(LogSwarmSim, Error, TEXT("scene export failed: %s"), *Error);
    return;
  }
  for (int32 I = 0; I < FMath::Min(Report.Survey.Num(), 60); ++I) {
    UE_LOG(LogSwarmSim, Display, TEXT("survey %s"), *Report.Survey[I]);
  }
  std::vector<std::uint8_t> Bytes;
  const swarm::SceneStatus Status = swarm::serialize_scene(Scene, Bytes);
  if (Status != swarm::SceneStatus::kOk) {
    UE_LOG(LogSwarmSim, Error, TEXT("cannot serialize the scene: %hs"), swarm::to_string(Status));
    return;
  }
  TArray<uint8> Data;
  Data.Append(Bytes.data(), static_cast<int32>(Bytes.size()));
  if (!FFileHelper::SaveArrayToFile(Data, *Options->OutPath)) {
    UE_LOG(LogSwarmSim, Error, TEXT("cannot write %s"), *Options->OutPath);
    return;
  }
  UE_LOG(LogSwarmSim, Display,
         TEXT("Exported scene to %s: %d x %d cells of %g m, ground from %.1f to %.1f m (%d of %d "
              "samples had none), %d trees, %d rocks, hash=%016llx"),
         *Options->OutPath, Scene.terrain.cells_x, Scene.terrain.cells_y, Scene.terrain.cell_size,
         Report.MinHeightM, Report.MaxHeightM, Report.HeightMisses, Report.HeightSamples,
         Report.Trees, Report.Rocks, swarm::scene_hash(Scene));
}
