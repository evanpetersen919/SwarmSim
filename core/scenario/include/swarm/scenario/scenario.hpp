#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "swarm/control/command.hpp"
#include "swarm/control/formation.hpp"
#include "swarm/control/mission.hpp"
#include "swarm/deterministic_channel.hpp"
#include "swarm/fixed_wing.hpp"
#include "swarm/obstacles.hpp"
#include "swarm/terrain.hpp"
#include "swarm/vehicle.hpp"
#include "swarm/weather.hpp"

namespace swarm {

// Where the drones start: a grid of `columns` columns, `spacing_m` apart, centered on the first
// waypoint of the mission, each drone moved by up to `jitter_m` in x and y and by up to
// `altitude_jitter_m` in z around the altitude `altitude_m`.
struct StartLayout {
  std::uint32_t columns = 4;
  double spacing_m = 35.0;
  double jitter_m = 8.0;
  double altitude_m = 20.0;
  double altitude_jitter_m = 3.0;
};

// How a drone estimates its neighbors. The first three are the predictors in
// swarm/estimation/neighbor_predictor.hpp. The oracle uses the neighbors' true state and is a
// baseline for evaluation; it is implemented on the vehicle side, outside the brain.
enum class PredictorChoice {
  kKalman,
  kDeadReckoning,
  kHoldLast,
  kOracle,
};

// The drone with index `drone` fails at `tick`.
struct Fault {
  std::uint32_t drone = 0;
  std::uint64_t tick = 0;
};

// A command a ground station sends to every drone at `tick`.
struct ScenarioCommand {
  std::uint64_t tick = 0;
  CommandKind kind = CommandKind::kHold;
};

// A scene read from a file in the scene format (swarm/format/scene.hpp) instead of generated from
// the terrain and obstacle settings. The file is named by `path`, relative to the working directory
// of the program that runs the scenario. `hash` is the scene's content hash (scene_hash). A
// scenario file may leave it out; the world builder checks it when it is given, and the canonical
// text, and so the log header, always holds it, so a log can be checked against the file it was run
// on.
struct SceneFileSource {
  bool used = false;
  std::string path;
  std::optional<std::uint64_t> hash;
};

struct MetricsSettings {
  double settle_s = 20.0;  // formation error is also reported from this time on
  double collision_radius_m = 1.0;
  double recovery_threshold_m = 1.0;
  double recovery_hold_s = 2.0;
};

// The vehicle every drone of the swarm flies. Written to the canonical text only when it is not a
// quad, which needs scenario version 2.
struct VehicleSettings {
  VehicleKind kind = VehicleKind::kQuad;
  FixedWingParams fixed_wing;  // used when `kind` is kFixedWing
};

// Everything that defines a run except the thread count, which must never matter. Times are in
// ticks; the file states them in seconds and the loader converts them.
struct Scenario {
  std::string name;
  std::uint64_t seed = 0;
  double dt = 0.02;
  std::uint64_t duration_ticks = 0;

  TerrainConfig terrain;
  ObstacleConfig obstacles;
  SceneFileSource scene_file;  // when used, `terrain` and `obstacles` are not

  std::uint32_t drones = 0;
  FormationShape formation_shape = FormationShape::kWedge;
  double formation_spacing_m = 20.0;
  StartLayout start;
  Mission mission;

  VehicleSettings vehicle;

  Weather weather;

  std::uint32_t lidar_beams = 8;
  std::uint32_t lidar_azimuth_samples = 36;
  std::uint64_t fix_period_ticks = 5;

  // `channel.seed` is not part of a scenario: the simulation seeds the channel from `seed`.
  ChannelConfig channel;

  PredictorChoice predictor = PredictorChoice::kKalman;
  // Ablation switches (see BrainConfig). Written to the canonical text only when off.
  bool fallback_enabled = true;
  bool separation_enabled = true;
  std::uint64_t report_period_ticks = 5;
  std::uint64_t staleness_ticks = 50;

  std::vector<Fault> faults;
  // Sorted by tick. When not empty, the world has a ground station as its last endpoint.
  std::vector<ScenarioCommand> commands;
  MetricsSettings metrics;
  std::uint64_t snapshot_every_ticks = 1;

  Scenario();
};

struct ScenarioError {
  std::string path;  // dotted path of the offending key, empty for the document as a whole
  int line = 0;      // 1-based, 0 if unknown
  std::string message;
};

// "path (line N): message", or just the message when there is no path.
std::string to_string(const ScenarioError& error);

struct LoadResult {
  Scenario scenario;
  std::vector<ScenarioError> errors;
  bool ok() const { return errors.empty(); }
};

// Versions of the scenario file format this build reads. Version 2 adds the `vehicle` section; a
// file that does not use it is the same in both, and the canonical text of such a scenario says
// version 1, so logs written before version 2 existed still verify.
inline constexpr std::uint64_t kScenarioVersion = 2;
inline constexpr std::uint64_t kScenarioVehicleVersion = 2;

// Parses and validates scenario text. Never throws: every problem, including malformed YAML, comes
// back as an entry in `errors`, in file order where there is one. Unknown keys are errors.
// `default_name` is used when the file has no `name`.
LoadResult load_scenario(const std::string& text, const std::string& default_name = "scenario");

// The scenario as text that load_scenario reads back to an identical scenario. Numbers are written
// with enough digits to round-trip exactly.
std::string canonical_text(const Scenario& scenario);

}  // namespace swarm
