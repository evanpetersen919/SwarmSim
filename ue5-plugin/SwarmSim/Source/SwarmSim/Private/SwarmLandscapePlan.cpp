#include "SwarmLandscapePlan.h"

namespace SwarmSim {
namespace {
constexpr int32 kSubsectionSizes[] = {255, 127, 63, 31, 15, 7};  // quads, largest first
constexpr int32 kSubsectionCounts[] = {1, 2};
constexpr int32 kMaxQuads = 8128;  // a Landscape is at most 8129 vertices per side

int32 CeilDiv(int32 A, int32 B) { return (A + B - 1) / B; }

double SampleHeight(const swarm::Heightfield& Terrain, int32 I, int32 J) {
  return Terrain.heights[Terrain.index(static_cast<size_t>(I), static_cast<size_t>(J))];
}

// Hash of two grid coordinates and a seed to [0, 1).
double Lattice(int64 X, int64 Y, uint64 Seed) {
  uint64 H = Seed ^ (static_cast<uint64>(X) * 0x9E3779B97F4A7C15ULL) ^
             (static_cast<uint64>(Y) * 0xC2B2AE3D27D4EB4FULL);
  H ^= H >> 33;
  H *= 0xFF51AFD7ED558CCDULL;
  H ^= H >> 33;
  H *= 0xC4CEB9FE1A85EC53ULL;
  H ^= H >> 33;
  return static_cast<double>(H >> 11) / 9007199254740992.0;  // 2^53
}

// Smooth value noise in [0, 1) with the given feature size in cells.
double ValueNoise(double X, double Y, double Feature, uint64 Seed) {
  const double Fx = X / Feature;
  const double Fy = Y / Feature;
  const int64 X0 = static_cast<int64>(FMath::FloorToDouble(Fx));
  const int64 Y0 = static_cast<int64>(FMath::FloorToDouble(Fy));
  const double Tx = Fx - static_cast<double>(X0);
  const double Ty = Fy - static_cast<double>(Y0);
  const double Sx = Tx * Tx * (3.0 - 2.0 * Tx);
  const double Sy = Ty * Ty * (3.0 - 2.0 * Ty);
  const double A = Lattice(X0, Y0, Seed);
  const double B = Lattice(X0 + 1, Y0, Seed);
  const double C = Lattice(X0, Y0 + 1, Seed);
  const double D = Lattice(X0 + 1, Y0 + 1, Seed);
  return FMath::Lerp(FMath::Lerp(A, B, Sx), FMath::Lerp(C, D, Sx), Sy);
}

double SmoothStep(double Edge0, double Edge1, double X) {
  const double T = FMath::Clamp((X - Edge0) / (Edge1 - Edge0), 0.0, 1.0);
  return T * T * (3.0 - 2.0 * T);
}
}  // namespace

bool ChooseLandscapeLayout(uint32 CellsX, uint32 CellsY, FLandscapeLayout& OutLayout) {
  if (CellsX == 0 || CellsY == 0 || CellsX > static_cast<uint32>(kMaxQuads) ||
      CellsY > static_cast<uint32>(kMaxQuads)) {
    return false;
  }
  bool Found = false;
  int64 BestSurplus = 0;
  for (int32 Size : kSubsectionSizes) {
    for (int32 Count : kSubsectionCounts) {
      const int32 Component = Size * Count;
      const int32 CompsX = CeilDiv(static_cast<int32>(CellsX), Component);
      const int32 CompsY = CeilDiv(static_cast<int32>(CellsY), Component);
      if (CompsX * Component > kMaxQuads || CompsY * Component > kMaxQuads) {
        continue;
      }
      const int64 Surplus = static_cast<int64>(CompsX) * Component * CompsY * Component -
                            static_cast<int64>(CellsX) * CellsY;
      // Sizes are visited largest first, so a tie keeps the earlier, larger choice.
      if (!Found || Surplus < BestSurplus) {
        Found = true;
        BestSurplus = Surplus;
        OutLayout.SubsectionSizeQuads = Size;
        OutLayout.NumSubsections = Count;
        OutLayout.ComponentsX = CompsX;
        OutLayout.ComponentsY = CompsY;
      }
    }
  }
  return Found;
}

void LandscapeVertexToSample(const swarm::Heightfield& Terrain, int32 U, int32 V, int32& OutI,
                             int32& OutJ) {
  OutI = FMath::Min(U, static_cast<int32>(Terrain.cells_x));
  OutJ = FMath::Max(0, static_cast<int32>(Terrain.cells_y) - V);
}

void EncodeLandscapeHeights(const swarm::Heightfield& Terrain, const FLandscapeLayout& Layout,
                            TArray<uint16>& OutHeights, float& OutScaleZ) {
  double MaxAbsMeters = 0.0;
  for (double H : Terrain.heights) {
    MaxAbsMeters = FMath::Max(MaxAbsMeters, FMath::Abs(H));
  }
  // Height in centimeters = (value - 32768) / 128 * ScaleZ. A scale of 100 covers +-256 m at
  // 0.78 cm per step; a taller terrain gets a larger scale and coarser steps.
  OutScaleZ = static_cast<float>(FMath::Max(100.0, MaxAbsMeters * 100.0 * 128.0 / 32000.0));

  const int32 Nx = Layout.VerticesX();
  const int32 Ny = Layout.VerticesY();
  OutHeights.SetNumUninitialized(static_cast<int32>(static_cast<int64>(Nx) * Ny));
  for (int32 V = 0; V < Ny; ++V) {
    for (int32 U = 0; U < Nx; ++U) {
      int32 I = 0;
      int32 J = 0;
      LandscapeVertexToSample(Terrain, U, V, I, J);
      const double Cm = SampleHeight(Terrain, I, J) * 100.0;
      const double Value = 32768.0 + Cm / OutScaleZ * 128.0;
      OutHeights[V * Nx + U] =
          static_cast<uint16>(FMath::Clamp(FMath::RoundToInt(Value), 0, 65535));
    }
  }
}

double DecodeLandscapeHeightCm(uint16 Value, float ScaleZ) {
  return (static_cast<double>(Value) - 32768.0) / 128.0 * static_cast<double>(ScaleZ);
}

void ComputeGroundWeights(const swarm::Heightfield& Terrain, const FLandscapeLayout& Layout,
                          uint64 Seed, const FGroundRules& Rules, const TArray<EGround>& Present,
                          TMap<EGround, TArray<uint8>>& OutWeights) {
  OutWeights.Reset();
  if (Present.IsEmpty()) {
    return;
  }
  const int32 Nx = Layout.VerticesX();
  const int32 Ny = Layout.VerticesY();
  const int64 Count = static_cast<int64>(Nx) * Ny;
  for (EGround Ground : Present) {
    OutWeights.Add(Ground).SetNumZeroed(static_cast<int32>(Count));
  }

  double MinH = Terrain.heights.empty() ? 0.0 : Terrain.heights[0];
  double MaxH = MinH;
  for (double H : Terrain.heights) {
    MinH = FMath::Min(MinH, H);
    MaxH = FMath::Max(MaxH, H);
  }
  const double Range = FMath::Max(MaxH - MinH, 1e-6);
  const double MudTop = MinH + 0.12 * Range;
  const double MudWidth = 0.08 * Range;

  // Where a type is missing its share goes to grass, or to the first type present.
  const EGround Fallback = Present.Contains(EGround::Grass) ? EGround::Grass : Present[0];
  const auto Home = [&](EGround Ground) { return Present.Contains(Ground) ? Ground : Fallback; };

  const double Cell = Terrain.cell_size;
  const int32 Cx = static_cast<int32>(Terrain.cells_x);
  const int32 Cy = static_cast<int32>(Terrain.cells_y);

  for (int32 V = 0; V < Ny; ++V) {
    for (int32 U = 0; U < Nx; ++U) {
      int32 I = 0;
      int32 J = 0;
      LandscapeVertexToSample(Terrain, U, V, I, J);

      // Slope from central differences, one-sided at the edges.
      const int32 Il = FMath::Max(I - 1, 0);
      const int32 Ir = FMath::Min(I + 1, Cx);
      const int32 Jl = FMath::Max(J - 1, 0);
      const int32 Jr = FMath::Min(J + 1, Cy);
      const double Dx = (SampleHeight(Terrain, Ir, J) - SampleHeight(Terrain, Il, J)) /
                        (static_cast<double>(Ir - Il) * Cell);
      const double Dy = (SampleHeight(Terrain, I, Jr) - SampleHeight(Terrain, I, Jl)) /
                        (static_cast<double>(Jr - Jl) * Cell);
      const double Slope = FMath::Sqrt(Dx * Dx + Dy * Dy);
      const double Height = SampleHeight(Terrain, I, J);

      const double Rock = SmoothStep(Rules.RockSlope - 0.5 * Rules.RockBlendWidth,
                                     Rules.RockSlope + 0.5 * Rules.RockBlendWidth, Slope);
      const double Mud =
          (1.0 - Rock) * (1.0 - SmoothStep(MudTop - MudWidth, MudTop + MudWidth, Height));
      const double Rest = 1.0 - Rock - Mud;
      const double NoiseA = ValueNoise(I, J, 9.0, Seed ^ 0xA1);
      const double NoiseB = ValueNoise(I, J, 5.0, Seed ^ 0xB2);
      const double ForestA = Rest * SmoothStep(0.55, 0.75, NoiseA);
      const double ForestB = (Rest - ForestA) * SmoothStep(0.60, 0.80, NoiseB);
      const double Grass = Rest - ForestA - ForestB;

      double Share[5] = {0, 0, 0, 0, 0};
      Share[static_cast<int32>(Home(EGround::Rock))] += Rock;
      Share[static_cast<int32>(Home(EGround::Mud))] += Mud;
      Share[static_cast<int32>(Home(EGround::ForestA))] += ForestA;
      Share[static_cast<int32>(Home(EGround::ForestB))] += ForestB;
      Share[static_cast<int32>(Home(EGround::Grass))] += Grass;

      // Round to whole weights that sum to 255, giving the remainder to the largest share.
      int32 Weights[5] = {0, 0, 0, 0, 0};
      int32 Sum = 0;
      int32 Largest = 0;
      for (int32 K = 0; K < 5; ++K) {
        Weights[K] = FMath::Clamp(FMath::RoundToInt(Share[K] * 255.0), 0, 255);
        Sum += Weights[K];
        if (Share[K] > Share[Largest]) {
          Largest = K;
        }
      }
      Weights[Largest] = FMath::Clamp(Weights[Largest] + (255 - Sum), 0, 255);

      const int32 Index = V * Nx + U;
      for (EGround Ground : Present) {
        OutWeights[Ground][Index] = static_cast<uint8>(Weights[static_cast<int32>(Ground)]);
      }
    }
  }
}
}  // namespace SwarmSim
