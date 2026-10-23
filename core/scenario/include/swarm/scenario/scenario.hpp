#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "swarm/camera_fix.hpp"
#include "swarm/control/avoid.hpp"
#include "swarm/control/command.hpp"
#include "swarm/control/formation.hpp"
#include "swarm/control/hazard.hpp"
#include "swarm/control/mission.hpp"
#include "swarm/control/orbit.hpp"
#include "swarm/control/path.hpp"
#include "swarm/deterministic_channel.hpp"
#include "swarm/fixed_wing.hpp"
#include "swarm/fixed_wing_energy.hpp"
#include "swarm/forward_sensor.hpp"
#include "swarm/obstacles.hpp"
#include "swarm/radar_altimeter.hpp"
#include "swarm/terrain.hpp"
#include "swarm/vehicle.hpp"
#include "swarm/weather.hpp"
#include "swarm/wind.hpp"

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
  kTurning,
  kHoldLast,
  kOracle,
};

// What goes wrong with a drone at `tick`: it fails, or (ADR 0016) it carries on flying and starts
// sending false hazard reports.
enum class FaultKind { kFail, kFalseHazards };

// The drone with index `drone` fails, or turns faulty, at `tick`.
struct Fault {
  std::uint32_t drone = 0;
  std::uint64_t tick = 0;
  FaultKind kind = FaultKind::kFail;
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
//
// With `dynamics` set to kEnergy (version 3) the aircraft is flown by the energy model. Its
// minimum speed is then not a free setting: it is the hold speed of the wing, derived from
// `energy`, and the loader writes it into `fixed_wing.min_speed`.
struct VehicleSettings {
  VehicleKind kind = VehicleKind::kQuad;
  FixedWingParams fixed_wing;  // used when `kind` is kFixedWing
  FixedWingDynamics dynamics = FixedWingDynamics::kSimple;
  EnergyParams energy;  // used when `dynamics` is kEnergy
};

// Everything that defines a run except the thread count, which must never matter. Times are in
// ticks; the file states them in seconds and the loader converts them.
// A rectangle, in meters, in which the prior map of the terrain shows nothing that rises above the
// lowest height on its edge.
struct OmitRect {
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;
};

// Errors of the prior map a fixed-wing aircraft carries (ADR 0013), with the keys of
// `control.terrain`. The map is exact unless one is set. They are applied in this order: the
// offset, the smooth random error, then the omitted rectangles. The scene, and so every sensor,
// is not affected.
struct TerrainMapFaults {
  double offset_m = 0.0;  // added to every height of the map
  // A smooth random error: independent normal values of this standard deviation on a lattice with
  // this spacing, interpolated with a smoothstep. Zero means none. The standard deviation holds at
  // the lattice nodes; between them the interpolated error is smaller.
  double error_sigma_m = 0.0;
  double error_correlation_m = 100.0;
  std::vector<OmitRect> omit;

  bool used() const { return offset_m != 0.0 || error_sigma_m > 0.0 || !omit.empty(); }
};

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
  // The air a fixed-wing aircraft flies in (version 3). Written to the canonical text only when
  // it has a mean or gusts.
  WindParams wind;
  // Whether an aircraft that reaches the ground stops there for good (version 3). Written to the
  // canonical text only when on.
  bool ground_contact_crashes = false;

  std::uint32_t lidar_beams = 8;
  std::uint32_t lidar_azimuth_samples = 36;
  std::uint64_t fix_period_ticks = 5;
  // The camera fix of a fixed-wing vehicle, which has no LiDAR. Written to the canonical text
  // only for a fixed-wing vehicle.
  CameraFixConfig camera;
  // The radar altimeter of a fixed-wing vehicle (ADR 0012), with the keys of
  // `sensors.radar_altimeter`. `altimeter_used` is set by that section; the canonical text writes
  // it only when set.
  bool altimeter_used = false;
  RadarAltimeterConfig altimeter;
  // The forward obstacle sensor of a fixed-wing vehicle (ADR 0013), with the keys of
  // `sensors.forward`. `forward_used` is set by that section; the canonical text writes it only
  // when set.
  bool forward_used = false;
  ForwardSensorConfig forward;
  // Turning and slowing for what climbing cannot clear (ADR 0014), with the keys of
  // `control.avoid`; `avoid.enabled` is set by that section. The canonical text writes it only
  // when set. It needs `forward_used`.
  AvoidParams avoid;
  // Sharing what the forward sensor sees (ADR 0016), with the keys of `control.share`; `mode` is
  // off unless that section is present. Needs `avoid.enabled`.
  ShareParams share;
  // Orbit guidance of a fixed-wing vehicle. Only the shape is set here (radius, steps, gains,
  // separation distance); the center, altitude, speed, and speed limits come from the mission
  // and the vehicle through orbit_params(). Written to the canonical text only for a
  // fixed-wing vehicle.
  OrbitParams orbit;
  // Terrain-aware altitude for orbit guidance (version 3). The aircraft carries the scene's
  // heightfield as a map. `climb_rate` is set from the vehicle. Written to the canonical text only
  // when enabled.
  TerrainGuidanceParams terrain_guidance;
  TerrainMapFaults map_faults;
  // The pattern a fixed-wing vehicle flies (version 3); the default is the orbit above. Written to
  // the canonical text only when it is not the orbit.
  PatternParams pattern;

  // `channel.seed` is not part of a scenario: the simulation seeds the channel from `seed`.
  ChannelConfig channel;

  PredictorChoice predictor = PredictorChoice::kKalman;
  // Ablation switches (see BrainConfig). Written to the canonical text only when off.
  bool fallback_enabled = true;
  bool separation_enabled = true;
  std::uint64_t report_period_ticks = 5;
  std::uint64_t staleness_ticks = 50;
  // Formation guidance: a neighbor not heard from for more than this many ticks is left out of the
  // formation consensus (see BrainConfig). 0 keeps every neighbor. Written to the canonical text
  // only when set.
  std::uint64_t drop_stale_after_ticks = 0;

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
// version 1, so logs written before version 2 existed still verify. Version 3 adds the flight
// realism of ADR 0011 (`vehicle.dynamics`, `vehicle.energy`, `environment`); a scenario that uses
// none of it is written as version 2 or 1, as before.
inline constexpr std::uint64_t kScenarioVersion = 3;
inline constexpr std::uint64_t kScenarioVehicleVersion = 2;
inline constexpr std::uint64_t kScenarioFlightVersion = 3;

// The version the canonical text of `scenario` says: the lowest that can express it.
std::uint64_t scenario_version(const Scenario& scenario);

// True when the scenario has a mean wind or gusts.
bool wind_used(const Scenario& scenario);

// The full orbit parameters of a fixed-wing scenario: `scenario.orbit` with the center and
// altitude of the mission's first waypoint, the mission speed, and the vehicle's speed limits.
OrbitParams orbit_params(const Scenario& scenario);

// The climb rate in m/s that terrain guidance may plan against for a fixed-wing scenario: the
// vehicle's climb limit, and with the energy model also the climb its propulsion can hold at the
// mission speed. Zero or negative when the aircraft cannot climb at that speed.
double terrain_climb_rate(const Scenario& scenario);

// Parses and validates scenario text. Never throws: every problem, including malformed YAML, comes
// back as an entry in `errors`, in file order where there is one. Unknown keys are errors.
// `default_name` is used when the file has no `name`.
LoadResult load_scenario(const std::string& text, const std::string& default_name = "scenario");

// The scenario as text that load_scenario reads back to an identical scenario. Numbers are written
// with enough digits to round-trip exactly.
std::string canonical_text(const Scenario& scenario);

}  // namespace swarm
