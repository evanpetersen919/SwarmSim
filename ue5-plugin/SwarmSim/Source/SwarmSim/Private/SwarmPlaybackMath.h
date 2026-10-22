#pragma once

#include "CoreMinimal.h"

// The parts of playback that need no engine objects, so they can be tested directly.

namespace SwarmSim {
// A drone's display attitude, in Unreal's convention: yaw about +Z from +X toward +Y, pitch
// positive nose up, roll positive right side down.
struct FDroneAttitude {
  double YawDeg = 0.0;
  double PitchDeg = 0.0;
  double RollDeg = 0.0;
};

// Standard gravity in m/s^2, for turning an acceleration into a lean.
constexpr double kGravity = 9.80665;

// The speed in m/s below which the heading is kept and not recomputed from the velocity.
constexpr double kMinHeadingSpeed = 0.5;

// A drone's attitude from its motion, for display only: the log holds no orientation.
//   * Heading is the direction of the horizontal velocity (kept from `PreviousYawDeg` when the
//   drone
//     is almost still).
//   * Lean is the angle whose tangent is the horizontal acceleration over gravity, split into
//     pitch (forward acceleration tips the nose down) and roll (acceleration to the right rolls to
//     the right), and limited to `MaxTiltDeg`.
// `VelocityUe` and `AccelerationUe` are in meters per second and per second squared, in Unreal's
// axes (the simulation's y negated).
FDroneAttitude AttitudeFromMotion(const FVector& VelocityUe, const FVector& AccelerationUe,
                                  double PreviousYawDeg, double MaxTiltDeg);

// The attitude of a fixed-wing aircraft, for display only. Heading and roll are as above (an
// aircraft in a coordinated turn banks by the angle whose tangent is the sideways acceleration over
// gravity), but pitch is the flight-path angle, positive nose up when climbing, because a wing
// does not tip its nose down to speed up. The pitch is limited to `MaxTiltDeg`.
FDroneAttitude AircraftAttitudeFromMotion(const FVector& VelocityUe, const FVector& AccelerationUe,
                                          double PreviousYawDeg, double MaxTiltDeg);

// Moves `Current` toward `Target` by the fraction 1 - exp(-DeltaSeconds / TimeConstant), so the
// result does not depend on the frame rate. Angles are shortest-way.
double SmoothAngleDeg(double Current, double Target, double DeltaSeconds, double TimeConstant);

// The tick after `DeltaSeconds` of wall time at `Speed` times real time, for a log with `Dt`
// seconds per tick, limited to [FirstTick, LastTick].
double AdvanceTick(double Tick, double DeltaSeconds, double Speed, double Dt, double FirstTick,
                   double LastTick);

// Display color of a drone's fallback state (0 nominal, 1 degraded comms, 2 degraded navigation,
// 3 safe hold), or dark gray for a drone that has failed.
FLinearColor StateColor(uint8 FallbackState, bool bAlive);

// A state hash as sixteen lowercase hexadecimal digits, as `swarm_sim` prints it.
FString HashString(uint64 Hash);

// Options for a replay given on the command line:
//   -SwarmLog=PATH        the run log to play (required to play)
//   -SwarmAnchor=X,Y      where the simulation's origin is in the level, in centimeters
//   -SwarmSpeed=N         playback speed, 1 is real time
//   -SwarmSceneFile=PATH  a scene file to build the terrain, trees, and rocks from (editor only)
//   -SwarmScale=N         how much larger than life the drones are drawn
//   -SwarmCamera=NAME     free, orbit, follow, or cinematic
//   -SwarmDroneModel=PATH a content folder (for example /Game/Altius) whose static meshes are the
//                         pieces of one aircraft, drawn in place of the stand-in drone
//   -SwarmShotDir=DIR     save screenshots of the game's own render here (not of the desktop)
//   -SwarmShotDelay=S     seconds before the first screenshot (default 30)
//   -SwarmShotEvery=S     seconds between screenshots (default 20)
//   -SwarmShotCount=N     how many to take (default 1); the game quits after the last
//   -SwarmFrameDir=DIR    write numbered frames for a video here, one per 1/fps of play time,
//                         waiting for each to be saved, then quit
//   -SwarmFrameFps=N      frames per second of the video (default 30)
//   -SwarmFrameSeconds=S  length of the video in seconds (default 30)
struct FLaunchOptions {
  FString LogPath;
  FString SceneFile;
  FVector2D AnchorCm = FVector2D::ZeroVector;
  double Speed = 1.0;
  double DroneScale = 4.0;
  FString Camera = TEXT("free");
  FString DroneModel;
  FString ShotDir;
  double ShotDelay = 30.0;
  double ShotEvery = 20.0;
  int32 ShotCount = 1;
  FString FrameDir;
  double FrameFps = 30.0;
  double FrameSeconds = 30.0;
  bool bValid = true;  // false if a value could not be read
  FString Problem;
};

FLaunchOptions ParseLaunchOptions(const TCHAR* CommandLine);

// Whether the next screenshot is due, after `Clock` seconds of play with `Taken` taken so far: the
// first at `Delay`, then every `Every` seconds, until `Count` have been taken.
bool ShotDue(double Clock, double Delay, double Every, int32 Taken, int32 Count);
}  // namespace SwarmSim
