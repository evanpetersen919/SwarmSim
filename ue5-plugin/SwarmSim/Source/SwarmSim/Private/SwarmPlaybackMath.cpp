#include "SwarmPlaybackMath.h"

#include "Misc/DefaultValueHelper.h"
#include "Misc/Parse.h"

namespace SwarmSim {
FDroneAttitude AttitudeFromMotion(const FVector& VelocityUe, const FVector& AccelerationUe,
                                  double PreviousYawDeg, double MaxTiltDeg) {
  FDroneAttitude Out;
  const double Speed = FMath::Sqrt(VelocityUe.X * VelocityUe.X + VelocityUe.Y * VelocityUe.Y);
  Out.YawDeg = Speed >= kMinHeadingSpeed
                   ? FMath::RadiansToDegrees(FMath::Atan2(VelocityUe.Y, VelocityUe.X))
                   : PreviousYawDeg;

  const double Yaw = FMath::DegreesToRadians(Out.YawDeg);
  const double Forward = AccelerationUe.X * FMath::Cos(Yaw) + AccelerationUe.Y * FMath::Sin(Yaw);
  const double Right = -AccelerationUe.X * FMath::Sin(Yaw) + AccelerationUe.Y * FMath::Cos(Yaw);
  const double Limit = FMath::Max(MaxTiltDeg, 0.0);
  Out.PitchDeg =
      FMath::Clamp(-FMath::RadiansToDegrees(FMath::Atan2(Forward, kGravity)), -Limit, Limit);
  Out.RollDeg = FMath::Clamp(FMath::RadiansToDegrees(FMath::Atan2(Right, kGravity)), -Limit, Limit);
  return Out;
}

FDroneAttitude AircraftAttitudeFromMotion(const FVector& VelocityUe, const FVector& AccelerationUe,
                                          double PreviousYawDeg, double MaxTiltDeg) {
  FDroneAttitude Out = AttitudeFromMotion(VelocityUe, AccelerationUe, PreviousYawDeg, MaxTiltDeg);
  const double Horizontal = FMath::Sqrt(VelocityUe.X * VelocityUe.X + VelocityUe.Y * VelocityUe.Y);
  const double Limit = FMath::Max(MaxTiltDeg, 0.0);
  Out.PitchDeg =
      FMath::Clamp(FMath::RadiansToDegrees(FMath::Atan2(VelocityUe.Z, Horizontal)), -Limit, Limit);
  return Out;
}

double SmoothAngleDeg(double Current, double Target, double DeltaSeconds, double TimeConstant) {
  if (!(TimeConstant > 0.0) || !(DeltaSeconds > 0.0)) {
    return Target;
  }
  const double Alpha = 1.0 - FMath::Exp(-DeltaSeconds / TimeConstant);
  const double Difference = FMath::FindDeltaAngleDegrees(Current, Target);
  return Current + Difference * Alpha;
}

double AdvanceTick(double Tick, double DeltaSeconds, double Speed, double Dt, double FirstTick,
                   double LastTick) {
  if (!(Dt > 0.0)) {
    return FMath::Clamp(Tick, FirstTick, LastTick);
  }
  return FMath::Clamp(Tick + DeltaSeconds * Speed / Dt, FirstTick, LastTick);
}

FLinearColor StateColor(uint8 FallbackState, bool bAlive) {
  if (!bAlive) {
    return FLinearColor(0.04f, 0.04f, 0.04f, 1.0f);
  }
  switch (FallbackState) {
    case 0:
      return FLinearColor(0.10f, 0.80f, 0.25f, 1.0f);  // nominal
    case 1:
      return FLinearColor(0.95f, 0.85f, 0.10f, 1.0f);  // degraded comms
    case 2:
      return FLinearColor(0.95f, 0.45f, 0.05f, 1.0f);  // degraded navigation
    default:
      return FLinearColor(0.90f, 0.08f, 0.08f, 1.0f);  // safe hold
  }
}

FString HashString(uint64 Hash) { return FString::Printf(TEXT("%016llx"), Hash); }

FLaunchOptions ParseLaunchOptions(const TCHAR* CommandLine) {
  FLaunchOptions Options;
  FParse::Value(CommandLine, TEXT("SwarmLog="), Options.LogPath);
  FParse::Value(CommandLine, TEXT("SwarmSceneFile="), Options.SceneFile);

  FString Text;
  if (FParse::Value(CommandLine, TEXT("SwarmAnchor="), Text, /*bShouldStopOnSeparator=*/false)) {
    FString X;
    FString Y;
    double Px = 0.0;
    double Py = 0.0;
    if (Text.Split(TEXT(","), &X, &Y) && FDefaultValueHelper::ParseDouble(X, Px) &&
        FDefaultValueHelper::ParseDouble(Y, Py)) {
      Options.AnchorCm = FVector2D(Px, Py);
    } else {
      Options.bValid = false;
      Options.Problem =
          FString::Printf(TEXT("-SwarmAnchor must be X,Y in centimeters, got '%s'"), *Text);
    }
  }
  if (FParse::Value(CommandLine, TEXT("SwarmSpeed="), Text)) {
    double Value = 0.0;
    if (FDefaultValueHelper::ParseDouble(Text, Value) && Value > 0.0 && Value <= 64.0) {
      Options.Speed = Value;
    } else {
      Options.bValid = false;
      Options.Problem = FString::Printf(
          TEXT("-SwarmSpeed must be a number above 0 and at most 64, got '%s'"), *Text);
    }
  }
  if (FParse::Value(CommandLine, TEXT("SwarmScale="), Text)) {
    double Value = 0.0;
    if (FDefaultValueHelper::ParseDouble(Text, Value) && Value >= 0.1 && Value <= 100.0) {
      Options.DroneScale = Value;
    } else {
      Options.bValid = false;
      Options.Problem =
          FString::Printf(TEXT("-SwarmScale must be from 0.1 to 100, got '%s'"), *Text);
    }
  }
  if (FParse::Value(CommandLine, TEXT("SwarmCamera="), Text)) {
    Text = Text.ToLower();
    if (Text == TEXT("free") || Text == TEXT("orbit") || Text == TEXT("follow") ||
        Text == TEXT("cinematic")) {
      Options.Camera = Text;
    } else {
      Options.bValid = false;
      Options.Problem = FString::Printf(
          TEXT("-SwarmCamera must be free, orbit, follow, or cinematic, got '%s'"), *Text);
    }
  }
  FParse::Value(CommandLine, TEXT("SwarmDroneModel="), Options.DroneModel);
  Options.bMessages = FParse::Param(CommandLine, TEXT("SwarmMessages"));
  Options.bNoMap = FParse::Param(CommandLine, TEXT("SwarmNoMap"));
  Options.bNoPip = FParse::Param(CommandLine, TEXT("SwarmNoPip"));
  Options.bNoTelemetry = FParse::Param(CommandLine, TEXT("SwarmNoTelemetry"));
  FParse::Value(CommandLine, TEXT("SwarmShotDir="), Options.ShotDir);
  const auto ReadNumber = [&](const TCHAR* Key, double Low, double High, double& Out) {
    FString Value;
    if (!FParse::Value(CommandLine, Key, Value)) {
      return;
    }
    double Number = 0.0;
    if (FDefaultValueHelper::ParseDouble(Value, Number) && Number >= Low && Number <= High) {
      Out = Number;
    } else {
      Options.bValid = false;
      Options.Problem = FString::Printf(TEXT("%s must be a number from %g to %g, got '%s'"), Key,
                                        Low, High, *Value);
    }
  };
  FParse::Value(CommandLine, TEXT("SwarmFrameDir="), Options.FrameDir);
  ReadNumber(TEXT("SwarmFrameFps="), 1.0, 120.0, Options.FrameFps);
  ReadNumber(TEXT("SwarmFrameSeconds="), 0.1, 3600.0, Options.FrameSeconds);
  double Count = static_cast<double>(Options.ShotCount);
  ReadNumber(TEXT("SwarmShotDelay="), 0.0, 3600.0, Options.ShotDelay);
  ReadNumber(TEXT("SwarmShotEvery="), 1.0, 3600.0, Options.ShotEvery);
  ReadNumber(TEXT("SwarmShotCount="), 1.0, 1000.0, Count);
  Options.ShotCount = static_cast<int32>(Count);
  return Options;
}

bool ShotDue(double Clock, double Delay, double Every, int32 Taken, int32 Count) {
  return Taken < Count && Clock >= Delay + Every * static_cast<double>(Taken);
}

FMapExtent MapExtentFor(double MinX, double MinY, double MaxX, double MaxY, double Margin) {
  const double Span = FMath::Max3(MaxX - MinX, MaxY - MinY, 1.0) * (1.0 + 2.0 * Margin);
  FMapExtent Extent;
  Extent.Size = Span;
  Extent.Min = FVector2D(0.5 * (MinX + MaxX) - 0.5 * Span, 0.5 * (MinY + MaxY) - 0.5 * Span);
  return Extent;
}

FVector2D MapPoint(const FMapExtent& Extent, const FVector2D& MapOrigin, double MapSize, double X,
                   double Y) {
  const double U = (X - Extent.Min.X) / Extent.Size;
  const double V = (Y - Extent.Min.Y) / Extent.Size;
  return FVector2D(MapOrigin.X + U * MapSize, MapOrigin.Y + (1.0 - V) * MapSize);
}

bool VisibilityFromConfig(const FString& Config, double& OutVisibilityM) {
  // The canonical text writes `weather:` at the left margin and its keys indented under it, one per
  // line, so the line that starts with `weather:` is followed by the indented lines of the section.
  TArray<FString> Lines;
  Config.ParseIntoArrayLines(Lines, false);
  bool bInWeather = false;
  for (const FString& Line : Lines) {
    if (Line.StartsWith(TEXT("weather:"), ESearchCase::CaseSensitive)) {
      bInWeather = true;
      continue;
    }
    if (!bInWeather) {
      continue;
    }
    if (!Line.IsEmpty() && !Line.StartsWith(TEXT(" "))) {
      return false;  // the next top-level key: the section ended without a visibility
    }
    const FString Trimmed = Line.TrimStartAndEnd();
    if (!Trimmed.StartsWith(TEXT("visibility_m:"), ESearchCase::CaseSensitive)) {
      continue;
    }
    const FString Number = Trimmed.Mid(FCString::Strlen(TEXT("visibility_m:"))).TrimStartAndEnd();
    if (!Number.IsNumeric()) {
      return false;
    }
    const double Value = FCString::Atod(*Number);
    if (!(Value > 0.0)) {
      return false;
    }
    OutVisibilityM = Value;
    return true;
  }
  return false;
}

double FogDensityForVisibility(double VisibilityM) {
  if (!(VisibilityM > 0.0)) {
    return 0.0;
  }
  return FMath::Clamp(39.12 / VisibilityM, 0.0, 10.0);
}

double ScaleBarMeters(double MaxMeters) {
  if (!(MaxMeters > 0.0)) {
    return 0.0;
  }
  const double Base = FMath::Pow(10.0, FMath::FloorToDouble(FMath::LogX(10.0, MaxMeters)));
  const double Mantissa = MaxMeters / Base;
  return Base * (Mantissa >= 5.0 ? 5.0 : (Mantissa >= 2.0 ? 2.0 : 1.0));
}

}  // namespace SwarmSim
