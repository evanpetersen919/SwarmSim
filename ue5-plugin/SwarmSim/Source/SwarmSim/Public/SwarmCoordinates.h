#pragma once

#include "CoreMinimal.h"

// Conversion between the simulation's coordinates and Unreal's, in one place.
//
// The simulation uses meters in a right-handed frame with +z up (docs/MODELS.md). Unreal uses
// centimeters in a left-handed frame with +Z up. A conversion between handedness has to mirror one
// horizontal axis; this one negates y, so that Unreal X is the simulation's x, Unreal Y is the
// simulation's -y, and Unreal Z is z. Nothing else in the plugin converts coordinates by hand.
namespace SwarmSim {
constexpr double MetersToCentimeters = 100.0;

inline FVector ToUnreal(double X, double Y, double Z) {
  return FVector(X * MetersToCentimeters, -Y * MetersToCentimeters, Z * MetersToCentimeters);
}

inline FVector ToUnreal(const double (&Meters)[3]) {
  return ToUnreal(Meters[0], Meters[1], Meters[2]);
}

inline void ToSimulation(const FVector& Centimeters, double (&OutMeters)[3]) {
  OutMeters[0] = Centimeters.X / MetersToCentimeters;
  OutMeters[1] = -Centimeters.Y / MetersToCentimeters;
  OutMeters[2] = Centimeters.Z / MetersToCentimeters;
}
}  // namespace SwarmSim
