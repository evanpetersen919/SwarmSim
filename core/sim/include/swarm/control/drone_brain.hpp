#pragma once

// Controller headers may include only this directory, swarm/estimation/, sensor_data.hpp,
// vec3.hpp, drone_command.hpp, message.hpp, and swarm/format/{byte_io,hash}.hpp. They must
// never include vehicle truth.
// Enforced by scripts/check_core_invariants.sh.

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "swarm/control/command.hpp"
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
#include "swarm/message.hpp"
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

  // The speed limit in the nominal state, as a multiple of the mission's cruise speed. It bounds
  // the speed at which a drone closes a gap and so the closing speed that separation has to stop.
  // Must be at least 1.
  double speed_limit_factor = 2.0;

  // A neighbor not heard from for this many ticks may have stopped where it was last heard, while
  // its prediction carries it on. From then on separation also keeps clear of its last reported
  // position. 0 turns this off. It does not apply while the drone is holding, whether commanded
  // or in safe hold.
  std::uint64_t last_known_after_ticks = 150;

  // Switches for ablation studies; both default to on. With fallback off the state machine still
  // runs and reports its state, but the drone always behaves as in the nominal state, apart from
  // a commanded hold, which is not a fallback. With separation off the drone feels no repulsion
  // from its neighbors. Turning off prediction is the hold-last predictor, not a switch.
  bool fallback_enabled = true;
  bool separation_enabled = true;

  std::uint64_t report_period_ticks = 10;  // a report is broadcast on ticks divisible by this
  // A neighbor report older than this is not fresh. Comms is not judged during the first this many
  // ticks, so a drone is not degraded merely because the first reports are still in flight.
  std::uint64_t staleness_ticks = 100;
};

// A message payload delivered to this drone and who sent it.
struct InboundReport {
  std::uint32_t sender = 0;
  std::vector<std::uint8_t> payload;
  // A beacon payload is a neighbor report, a command payload is a Command.
  MessageType type = MessageType::kBeacon;
};

struct BrainInput {
  ImuSample imu;
  PositionFix fix;
  std::vector<InboundReport> inbox;
  // Neighbor estimates supplied from outside, indexed by drone id (the entry for this drone is
  // ignored). When present they replace the predictors, every neighbor counts as fresh, and the
  // channel plays no part in what the brain does. The brain cannot tell where they came from.
  std::optional<std::vector<NeighborEstimate>> neighbor_override;
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
// velocity. Otherwise the fallback state scales the speed limit (a speed limiter removes forward
// acceleration above the limit and brakes the excess), the separation margin, the formation gains,
// and the weight on the mission reference. The separation margin also widens by the drone's own
// position uncertainty.
struct CommandCounters {
  std::uint64_t applied = 0;    // changed the commanded state or repeated it with a newer tick
  std::uint64_t stale = 0;      // older than, or as old as, the last command applied
  std::uint64_t malformed = 0;  // did not decode
};

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

  // While a drone is holding (commanded or in safe hold), separation counts only neighbors heard
  // from within `staleness_ticks`, and the last-known-position rule is off: a hold is a hard
  // promise to stay put, and a prediction that cannot be checked must not break it.
  // True while a hold command is in force. A commanded hold uses the same stopping behavior as
  // the fallback machine's safe hold but is not a fallback state, and the machine is unaffected.
  bool holding_commanded() const { return commanded_hold_; }
  const CommandCounters& command_counters() const { return command_counters_; }

  // Folds everything that decides the brain's behavior into `hash`: the fallback machine, the
  // pose estimate and counters, the neighbor table counters and report ages, each neighbor's
  // current estimate, the hold position, the last target and the last command. Estimator internals
  // (covariances) enter only through the sigmas they report.
  void hash_state(Fnv1a64& hash) const;

 private:
  void apply_command(const InboundReport& in, const Vec3& position);

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
  bool commanded_hold_ = false;
  bool has_command_ = false;
  std::uint64_t last_command_tick_ = 0;
  CommandCounters command_counters_;
  Vec3 hold_position_;
  FormationTarget target_;
  DroneCommand last_command_;
};

}  // namespace swarm
