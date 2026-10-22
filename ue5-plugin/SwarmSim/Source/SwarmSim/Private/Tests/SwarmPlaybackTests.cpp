// Automation tests for playback. See SwarmTerrainTests.cpp for how to run them.

#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "SwarmPlaybackMath.h"
#include "swarm/format/log.hpp"
#include "swarm/format/snapshot.hpp"

#if WITH_EDITOR
#include "Editor.h"
#include "Engine/World.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "SwarmPlayback.h"
#include "Tests/AutomationEditorCommon.h"
#endif

using namespace SwarmSim;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSwarmPlaybackMathTest, "SwarmSim.Playback.Math",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)
bool FSwarmPlaybackMathTest::RunTest(const FString& Parameters) {
  // Heading follows the horizontal velocity, in Unreal's axes.
  TestTrue(
      TEXT("east is yaw 0"),
      FMath::IsNearlyEqual(
          AttitudeFromMotion(FVector(5, 0, 0), FVector::ZeroVector, 77.0, 30.0).YawDeg, 0.0, 1e-9));
  TestTrue(
      TEXT("toward +Y is yaw 90"),
      FMath::IsNearlyEqual(
          AttitudeFromMotion(FVector(0, 5, 0), FVector::ZeroVector, 0.0, 30.0).YawDeg, 90.0, 1e-9));
  TestTrue(
      TEXT("toward -X is yaw 180"),
      FMath::IsNearlyEqual(
          FMath::Abs(AttitudeFromMotion(FVector(-5, 0, 0), FVector::ZeroVector, 0.0, 30.0).YawDeg),
          180.0, 1e-9));
  TestTrue(TEXT("a drone that is almost still keeps its heading"),
           FMath::IsNearlyEqual(
               AttitudeFromMotion(FVector(0.1, 0.1, 0), FVector::ZeroVector, 123.0, 30.0).YawDeg,
               123.0, 1e-9));
  TestTrue(TEXT("climbing alone does not turn it"),
           FMath::IsNearlyEqual(
               AttitudeFromMotion(FVector(0, 0, 8), FVector::ZeroVector, 40.0, 30.0).YawDeg, 40.0,
               1e-9));

  // An aircraft's pitch is its flight-path angle: climbing at 45 degrees is 45 degrees nose up,
  // forward acceleration does not tip it, and a turn still banks it.
  const FDroneAttitude Climb =
      AircraftAttitudeFromMotion(FVector(10, 0, 10), FVector(kGravity, 0, 0), 0.0, 60.0);
  TestTrue(TEXT("an aircraft's pitch is its flight-path angle"),
           FMath::IsNearlyEqual(Climb.PitchDeg, 45.0, 1e-6));
  TestTrue(TEXT("forward acceleration does not change it"),
           FMath::IsNearlyEqual(
               AircraftAttitudeFromMotion(FVector(10, 0, 0), FVector(kGravity, 0, 0), 0.0, 60.0)
                   .PitchDeg,
               0.0, 1e-9));
  TestTrue(
      TEXT("an aircraft banks into a turn"),
      FMath::IsNearlyEqual(
          AircraftAttitudeFromMotion(FVector(10, 0, 0), FVector(0, kGravity, 0), 0.0, 60.0).RollDeg,
          45.0, 1e-6));
  TestTrue(
      TEXT("and the pitch is limited"),
      FMath::IsNearlyEqual(
          AircraftAttitudeFromMotion(FVector(1, 0, 10), FVector::ZeroVector, 0.0, 30.0).PitchDeg,
          30.0, 1e-9));

  // No acceleration, no lean. Forward acceleration of one g tips the nose down 45 degrees; right
  // acceleration of one g rolls right 45 degrees; the lean is limited.
  const FDroneAttitude Level = AttitudeFromMotion(FVector(5, 0, 0), FVector::ZeroVector, 0.0, 30.0);
  TestTrue(TEXT("level flight is level"), FMath::IsNearlyEqual(Level.PitchDeg, 0.0, 1e-9) &&
                                              FMath::IsNearlyEqual(Level.RollDeg, 0.0, 1e-9));
  const FDroneAttitude Forward =
      AttitudeFromMotion(FVector(5, 0, 0), FVector(kGravity, 0, 0), 0.0, 60.0);
  TestTrue(TEXT("forward acceleration pitches the nose down"),
           FMath::IsNearlyEqual(Forward.PitchDeg, -45.0, 1e-6));
  TestTrue(TEXT("and does not roll"), FMath::IsNearlyEqual(Forward.RollDeg, 0.0, 1e-6));
  const FDroneAttitude Right =
      AttitudeFromMotion(FVector(5, 0, 0), FVector(0, kGravity, 0), 0.0, 60.0);
  TestTrue(TEXT("acceleration to the right rolls right"),
           FMath::IsNearlyEqual(Right.RollDeg, 45.0, 1e-6));
  TestTrue(TEXT("and does not pitch"), FMath::IsNearlyEqual(Right.PitchDeg, 0.0, 1e-6));
  // The lean is measured in the drone's own frame: flying toward +Y, +X is to its right? Heading
  // 90 degrees has forward = +Y and right = -X, so acceleration along -X rolls right.
  const FDroneAttitude Turned =
      AttitudeFromMotion(FVector(0, 5, 0), FVector(-kGravity, 0, 0), 0.0, 60.0);
  TestTrue(TEXT("the lean follows the heading"), FMath::IsNearlyEqual(Turned.RollDeg, 45.0, 1e-6));
  const FDroneAttitude Limited =
      AttitudeFromMotion(FVector(5, 0, 0), FVector(100.0, 100.0, 0), 0.0, 20.0);
  TestTrue(TEXT("lean is limited"), FMath::Abs(Limited.PitchDeg) <= 20.0 + 1e-9 &&
                                        FMath::Abs(Limited.RollDeg) <= 20.0 + 1e-9);

  // Smoothing takes the short way round, and two half steps equal one full step.
  const double Across = SmoothAngleDeg(170.0, -170.0, 0.1, 0.1);
  TestTrue(TEXT("through 180, not back through 0"), Across > 170.0 && Across < 190.0);
  const double Whole = SmoothAngleDeg(0.0, 90.0, 0.2, 0.3);
  const double Halves = SmoothAngleDeg(SmoothAngleDeg(0.0, 90.0, 0.1, 0.3), 90.0, 0.1, 0.3);
  TestTrue(TEXT("independent of the frame rate"), FMath::IsNearlyEqual(Whole, Halves, 1e-9));
  TestTrue(TEXT("no time constant means the target"), SmoothAngleDeg(0.0, 90.0, 0.1, 0.0) == 90.0);

  // Time advances by wall time times speed over the tick length, and stops at the ends.
  TestTrue(TEXT("0.5 s at 2x with 0.1 s ticks is 10 ticks"),
           FMath::IsNearlyEqual(AdvanceTick(0.0, 0.5, 2.0, 0.1, 0.0, 100.0), 10.0, 1e-9));
  TestTrue(TEXT("stops at the last tick"), AdvanceTick(99.0, 5.0, 1.0, 0.1, 0.0, 100.0) == 100.0);
  TestTrue(TEXT("does not go before the first"),
           AdvanceTick(1.0, -5.0, 1.0, 0.1, 0.0, 100.0) == 0.0);

  // Colors: each fallback state is distinct, and a failed drone is dark.
  TArray<FLinearColor> Colors;
  for (uint8 State = 0; State < 4; ++State) {
    Colors.Add(StateColor(State, true));
  }
  for (int32 A = 0; A < 4; ++A) {
    for (int32 B = A + 1; B < 4; ++B) {
      TestFalse(TEXT("distinct state colors"), Colors[A].Equals(Colors[B], 0.05f));
    }
  }
  TestTrue(TEXT("a failed drone is dark"), StateColor(0, false).GetLuminance() < 0.1f);

  TestEqual(TEXT("hash digits"), HashString(0x44c), FString(TEXT("000000000000044c")));
  TestEqual(TEXT("a full hash"), HashString(0xbe147c54241751fULL),
            FString(TEXT("0be147c54241751f")));

  // The screenshot schedule: the first at the delay, then one every interval, then no more.
  TestFalse(TEXT("not before the delay"), ShotDue(9.9, 10.0, 5.0, 0, 3));
  TestTrue(TEXT("the first at the delay"), ShotDue(10.0, 10.0, 5.0, 0, 3));
  TestFalse(TEXT("the second is not due yet"), ShotDue(14.9, 10.0, 5.0, 1, 3));
  TestTrue(TEXT("the second after an interval"), ShotDue(15.0, 10.0, 5.0, 1, 3));
  TestTrue(TEXT("the third"), ShotDue(20.0, 10.0, 5.0, 2, 3));
  TestFalse(TEXT("and no fourth"), ShotDue(1000.0, 10.0, 5.0, 3, 3));

  // Command line options.
  const FLaunchOptions None = ParseLaunchOptions(TEXT("-foo=1 -bar"));
  TestTrue(TEXT("no log, no replay"), None.LogPath.IsEmpty() && None.bValid);
  const FLaunchOptions Full =
      ParseLaunchOptions(TEXT("-SwarmLog=C:/runs/a.swlog -SwarmAnchor=81187.5,-14778 -SwarmSpeed=2 "
                              "-SwarmScale=8 -SwarmCamera=Orbit -SwarmSceneFile=C:/x/s.swscene"));
  TestTrue(TEXT("valid"), Full.bValid);
  TestEqual(TEXT("log path"), Full.LogPath, FString(TEXT("C:/runs/a.swlog")));
  TestEqual(TEXT("scene path"), Full.SceneFile, FString(TEXT("C:/x/s.swscene")));
  TestTrue(TEXT("anchor"), Full.AnchorCm.Equals(FVector2D(81187.5, -14778.0), 1e-9));
  TestTrue(TEXT("speed and scale"), Full.Speed == 2.0 && Full.DroneScale == 8.0);
  TestEqual(TEXT("camera, lowercased"), Full.Camera, FString(TEXT("orbit")));
  TestEqual(TEXT("the default camera is free"), ParseLaunchOptions(TEXT("-SwarmLog=a")).Camera,
            FString(TEXT("free")));
  for (const TCHAR* Bad :
       {TEXT("-SwarmLog=a -SwarmAnchor=12"), TEXT("-SwarmLog=a -SwarmAnchor=x,y"),
        TEXT("-SwarmLog=a -SwarmSpeed=0"), TEXT("-SwarmLog=a -SwarmSpeed=abc"),
        TEXT("-SwarmLog=a -SwarmScale=1000"), TEXT("-SwarmLog=a -SwarmCamera=spin")}) {
    const FLaunchOptions Options = ParseLaunchOptions(Bad);
    TestFalse(FString::Printf(TEXT("rejects %s"), Bad), Options.bValid);
    TestFalse(TEXT("with a reason"), Options.Problem.IsEmpty());
  }
  return true;
}

#if WITH_EDITOR
namespace {
// A log of three drones at 0.1 s per tick, frames at ticks 0, 10, ..., 100 (the last tick):
//   drone 0 flies along +x at 5 m/s from (0, 0, 20);
//   drone 1 holds at (10, 10, 20) in fallback state 2;
//   drone 2 is still until tick 50 and failed from then on, frozen at (-5, 0, 20).
// A frame's hash is 1000 + its tick.
FString WriteLog(const FString& Name) {
  swarm::LogHeader Header;
  Header.seed = 5;
  Header.dt = 0.1;
  Header.agent_count = 3;
  Header.config = "hand built";
  swarm::LogWriter Writer;
  Writer.begin(Header);
  for (uint64 Tick = 0; Tick <= 100; Tick += 10) {
    swarm::Snapshot Snapshot;
    swarm::AgentSnapshot A;
    A.position = {5.0 * Tick * 0.1, 0.0, 20.0};
    A.velocity = {5.0, 0.0, 0.0};
    swarm::AgentSnapshot B;
    B.fallback_state = 2;
    B.position = {10.0, 10.0, 20.0};
    swarm::AgentSnapshot C;
    C.alive = Tick < 50;
    C.position = {-5.0, 0.0, 20.0};
    Snapshot.agents = {A, B, C};
    swarm::LogRecord Record;
    Record.tick = Tick;
    Record.hash = 1000 + Tick;
    swarm::encode_snapshot(Snapshot, Record.blob);
    Writer.append(Record);
  }
  Writer.finish();
  TArray<uint8> Bytes;
  Bytes.Append(Writer.bytes().data(), static_cast<int32>(Writer.bytes().size()));
  const FString Path = FPaths::ProjectSavedDir() / TEXT("SwarmTest") / Name;
  FFileHelper::SaveArrayToFile(Bytes, *Path);
  return Path;
}
}  // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSwarmPlaybackActorTest, "SwarmSim.Playback.Actor",
                                 EAutomationTestFlags::EditorContext |
                                     EAutomationTestFlags::EngineFilter)
bool FSwarmPlaybackActorTest::RunTest(const FString& Parameters) {
  UWorld* World = FAutomationEditorCommonUtils::CreateNewMap();
  if (!TestNotNull(TEXT("a test world"), World)) {
    return false;
  }
  ASwarmPlayback* Playback = World->SpawnActor<ASwarmPlayback>();
  if (!TestNotNull(TEXT("a playback actor"), Playback)) {
    return false;
  }
  Playback->bShowHud = false;

  // A log that is not there, and one that is not a log, are errors and leave nothing behind.
  FString Error;
  TestFalse(TEXT("a missing log"), Playback->LoadLog(TEXT("does/not/exist.swlog"), Error));
  TestFalse(TEXT("with a message"), Error.IsEmpty());
  const FString Garbage = FPaths::ProjectSavedDir() / TEXT("SwarmTest/garbage.swlog");
  FFileHelper::SaveStringToFile(TEXT("not a log"), *Garbage);
  TestFalse(TEXT("garbage"), Playback->LoadLog(Garbage, Error));
  TestEqual(TEXT("no drones after a failure"), Playback->GetDroneCount(), 0);

  const FString Path = WriteLog(TEXT("hand.swlog"));
  TestTrue(FString::Printf(TEXT("the log loads (%s)"), *Error), Playback->LoadLog(Path, Error));
  TestEqual(TEXT("a drone per agent"), Playback->GetDroneCount(), 3);
  TestEqual(TEXT("all alive at the start"), Playback->GetAliveCount(), 3);

  // Positions, with the numbers written out here and not computed with the conversion helpers:
  // the simulation's y is Unreal's -Y, meters become centimeters. Drone 0 is at x = 5 m/s * 3 s
  // = 15 m at tick 30.
  Playback->SetPaused(true);
  Playback->SetPlaybackTick(30.0);
  TestTrue(TEXT("drone 0 at tick 30"),
           Playback->GetDroneLocation(0).Equals(FVector(1500.0, 0.0, 2000.0), 0.01));
  TestTrue(TEXT("drone 1 holds, with y mirrored"),
           Playback->GetDroneLocation(1).Equals(FVector(1000.0, -1000.0, 2000.0), 0.01));
  TestTrue(TEXT("between frames it is on the line"),
           (Playback->SetPlaybackTick(35.0),
            Playback->GetDroneLocation(0).Equals(FVector(1750.0, 0.0, 2000.0), 0.01)));

  // Heading and state color.
  Playback->SetPlaybackTick(30.0);
  TestTrue(TEXT("drone 0 faces +X"),
           FMath::IsNearlyEqual(Playback->GetDroneRotation(0).Yaw, 0.0, 0.01));
  TestTrue(TEXT("level in steady flight"),
           FMath::Abs(Playback->GetDroneRotation(0).Pitch) < 0.5 &&
               FMath::Abs(Playback->GetDroneRotation(0).Roll) < 0.5);
  TestTrue(TEXT("drone 1 is in state 2"),
           Playback->GetDroneColor(1).Equals(StateColor(2, true), 1e-4f));
  TestTrue(TEXT("drone 0 is nominal"),
           Playback->GetDroneColor(0).Equals(StateColor(0, true), 1e-4f));

  // A drone fails: dark, and no longer counted.
  Playback->SetPlaybackTick(80.0);
  TestEqual(TEXT("two alive after the failure"), Playback->GetAliveCount(), 2);
  TestTrue(TEXT("the failed drone is dark"),
           Playback->GetDroneColor(2).Equals(StateColor(0, false), 1e-4f));
  TestTrue(TEXT("and stays where it stopped"),
           Playback->GetDroneLocation(2).Equals(FVector(-500.0, 0.0, 2000.0), 0.01));

  // The hash on screen is the log's: the frame at or before the tick, and at the last tick the
  // final hash. Tick 100 has hash 1100 = 0x44c.
  Playback->SetPlaybackTick(1.0e9);
  TestEqual(TEXT("the last tick shows the final hash"), Playback->GetHashString(),
            FString(TEXT("000000000000044c")));
  Playback->SetPlaybackTick(35.0);
  TestEqual(TEXT("between frames it shows the earlier one"), Playback->GetHashString(),
            HashString(1030));
  TestTrue(TEXT("the heads-up display carries it"),
           Playback->GetHudLines().ContainsByPredicate([](const FString& Line) {
             return Line.Contains(TEXT("state hash 0000000000000406"));
           }));

  // The anchor moves the whole replay and nothing else.
  Playback->SetAnchor(FVector2D(500.0, -200.0));
  Playback->SetPlaybackTick(30.0);
  TestTrue(TEXT("anchored"),
           Playback->GetDroneLocation(0).Equals(FVector(2000.0, -200.0, 2000.0), 0.01));
  Playback->SetAnchor(FVector2D::ZeroVector);

  // Time: unpaused, half a second at twice the speed is ten ticks.
  Playback->Restart();
  TestEqual(TEXT("restart goes to the first tick"), Playback->GetPlaybackTick(), 0.0);
  Playback->SetPaused(false);
  Playback->SetSpeed(2.0);
  Playback->Tick(0.5f);
  TestTrue(TEXT("0.5 s at 2x is 10 ticks"),
           FMath::IsNearlyEqual(Playback->GetPlaybackTick(), 10.0, 1e-6));
  Playback->SetPaused(true);
  Playback->Tick(0.5f);
  TestTrue(TEXT("paused does not advance"),
           FMath::IsNearlyEqual(Playback->GetPlaybackTick(), 10.0, 1e-6));
  Playback->StepSeconds(2.0);
  TestTrue(TEXT("a step of 2 s is 20 ticks"),
           FMath::IsNearlyEqual(Playback->GetPlaybackTick(), 30.0, 1e-6));
  Playback->StepSeconds(-100.0);
  TestEqual(TEXT("stepping back stops at the start"), Playback->GetPlaybackTick(), 0.0);

  // The end: playback stops there, or wraps if it loops.
  Playback->SetPaused(false);
  Playback->SetPlaybackTick(99.0);
  Playback->Tick(10.0f);
  TestEqual(TEXT("stops at the last tick"), Playback->GetPlaybackTick(), 100.0);
  TestTrue(TEXT("and pauses"), Playback->IsPaused());
  Playback->bLoop = true;
  Playback->SetPaused(false);
  Playback->SetPlaybackTick(99.0);
  Playback->Tick(10.0f);
  TestEqual(TEXT("a looping replay goes back to the start"), Playback->GetPlaybackTick(), 0.0);

  // Speed is limited.
  Playback->SetSpeed(1000.0);
  TestEqual(TEXT("speed has an upper limit"), Playback->GetSpeed(), 64.0);
  Playback->SetSpeed(0.0);
  TestTrue(TEXT("and a lower one"), Playback->GetSpeed() > 0.0);

  // Loading another log replaces the drones.
  TestTrue(TEXT("a second load"), Playback->LoadLog(Path, Error));
  TestEqual(TEXT("still three drones"), Playback->GetDroneCount(), 3);
  return true;
}
#endif

#endif  // WITH_DEV_AUTOMATION_TESTS
