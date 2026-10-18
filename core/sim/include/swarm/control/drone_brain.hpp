#pragma once

// Controller headers may include only this directory, swarm/estimation/, sensor_data.hpp,
// vec3.hpp, drone_command.hpp, and swarm/format/{byte_io,hash}.hpp. They must never include
// vehicle truth.
// Enforced by scripts/check_core_invariants.sh.

#include <cstdint>
#include <memory>
#include <vector>

#include "swarm/control/fallback.hpp"
#include "swarm/control/formation_control.hpp"
#include "swarm/control/mission.hpp"
#include "swarm/control/separation.hpp"
#include "swarm/drone_command.hpp"
#include "swarm/estimation/neighbor_message.hpp"
#include "swarm/estimation/neighbor_predictor.hpp"
#include "swarm/estimation/neighbor_table.hpp"
#include "swarm/estimation/pose_estimator.hpp"
#include "swarm/format/hash.hpp"
#include "swarm/sensor_data.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

struct BrainConfig {
  std::uint32_t id = 0;          // this drone, which is also its formation slot
  std::uint32_t num_drones = 1;  // slots 0..num_drones-1
  double dt = 0.01;              // seconds per tick; must equal pose.dt and predictor_config.dt

  std::vector<Vec3> offsets;  // one formation offset per drone
  Mission mission;

  PoseEstimatorConfig pose;
  PoseLaunch launch;
  PredictorKind predictor = PredictorKind::kKalman;
  NeighborPredictorConfig predictor_config;

  FormationParams formation;
  SeparationParams separation;  // separation.max_accel is also the airframe limit of the command
  FallbackParams fallback;

  std::uint64_t report_period_ticks = 10;  // a report is broadcast on ticks divisible by this
  std::uint64_t staleness_ticks = 100;     // a neighbor report older than this is not fresh
};

// A message payload delivered to this drone and who sent it.
struct InboundReport {
  std::uint32_t sender = 0;
  std::vector<std::uint8_t> payload;
};

struct BrainInput {
  ImuSample imu;
  PositionFix fix;
  std::vector<InboundReport> inbox;
};

struct BrainOutput {
  DroneCommand command;
  bool has_report = false;
  std::vector<std::uint8_t> report;  // an encoded NeighborReport, when has_report is set
};

// The decision-making half of a drone. Its inputs are the IMU sample, the position fix, and the
// payloads delivered this tick; it never sees the vehicle's true state.
//
// Each step it advances the pose estimator, files the neighbor reports, updates the fallback
// machine, and computes an acceleration command:
//
//   formation pull (target from the mission reference and neighbor consensus, scaled by the
//   current fallback state) plus separation, with separation taking priority within the limit.
//
// In safe hold the target is the position held at the moment the state was entered, at zero
// velocity. Otherwise the fallback state scales the cruise speed (a speed limiter removes forward
// acceleration above the limit and brakes the excess), the separation margin, the formation gains,
// and the weight on the mission reference. The separation margin also widens by the drone's own
// position uncertainty.
class DroneBrain {
 public:
  // Throws std::invalid_argument for an invalid configuration.
  explicit DroneBrain(const BrainConfig& config);

  // Ticks must not decrease. Throws std::invalid_argument for non-finite inputs or a decreasing
  // tick. Malformed or unknown-sender payloads are counted and ignored, not errors.
  BrainOutput step(const BrainInput& input, std::uint64_t tick);

  FallbackState state() const { return fsm_.state(); }
  PoseEstimate pose() const { return pose_.estimate(); }
  const PoseEstimatorCounters& pose_counters() const { return pose_.counters(); }
  const NeighborTableCounters& table_counters() const { return table_.counters(); }
  // The latest estimate of neighbor `slot` as computed in the last step.
  const NeighborEstimate& neighbor(std::uint32_t slot) const { return estimates_.at(slot); }
  std::uint32_t fresh_neighbors() const { return fresh_; }
  const FormationTarget& target() const { return target_; }
  const DroneCommand& last_command() const { return last_command_; }

  // Folds everything that decides the brain's behavior into `hash`: the fallback machine, the
  // pose estimate and counters, the neighbor table counters and report ages, each neighbor's
  // current estimate, the hold position, the last target and the last command. Estimator internals
  // (covariances) enter only through the sigmas they report.
  void hash_state(Fnv1a64& hash) const;

 private:
  BrainConfig config_;
  MissionPath path_;
  PoseEstimator pose_;
  NeighborTable table_;
  std::vector<std::unique_ptr<INeighborPredictor>> predictors_;  // null for this drone's own slot
  FallbackFsm fsm_;

  std::vector<NeighborEstimate> estimates_;
  std::uint32_t fresh_ = 0;
  std::uint64_t last_fix_tick_ = 0;
  std::uint64_t last_tick_ = 0;
  bool started_ = false;
  bool has_hold_ = false;
  Vec3 hold_position_;
  FormationTarget target_;
  DroneCommand last_command_;
};

}  // namespace swarm
