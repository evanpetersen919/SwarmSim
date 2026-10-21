#pragma once

// Estimator headers may include only this directory, sensor_data.hpp, swarm/format/byte_io.hpp, and
// vector types. They must never include vehicle truth. Enforced by
// scripts/check_core_invariants.sh.

#include <cstdint>
#include <memory>

#include "swarm/estimation/neighbor_message.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

struct NeighborEstimate {
  bool valid = false;  // false until at least one report has been received
  Vec3 position;
  Vec3 velocity;
  double position_sigma = 0.0;  // per-axis standard deviation, meters
  std::uint64_t age = 0;        // ticks since the latest accepted report was delivered
};

// Estimates where a neighbor is now from the reports received so far. Implementations are driven
// by the same two calls, so a scenario can switch between them from configuration.
class INeighborPredictor {
 public:
  virtual ~INeighborPredictor() = default;

  // A report delivered at `receipt_tick`. A report whose send tick is not newer than the latest
  // accepted one is ignored. Receipt ticks must not decrease (std::invalid_argument otherwise).
  virtual void on_report(const NeighborReport& report, std::uint64_t receipt_tick) = 0;

  // The estimate at `now_tick`, which must not be earlier than the latest receipt
  // (std::invalid_argument otherwise).
  virtual NeighborEstimate predict(std::uint64_t now_tick) const = 0;
};

enum class PredictorKind {
  kHoldLast,       // reports the last received state unchanged
  kDeadReckoning,  // extrapolates the last report at its reported velocity
  kKalman,         // constant-velocity Kalman filter with delay compensation
  kTurning,        // extrapolates the last report along a circle of the estimated turn rate
};

struct NeighborPredictorConfig {
  double dt = 0.01;  // seconds per tick
  // Standard deviation of a report's velocity, meters per second. Reports carry only a position
  // sigma, so this is assumed. Used by dead reckoning to grow its uncertainty and by the Kalman
  // predictor as the measurement noise on reported velocity.
  double report_velocity_sigma = 0.3;
  // Kalman only: spectral density of white acceleration noise, m^2/s^3. After t seconds without
  // information the velocity uncertainty is about sqrt(q t).
  double process_noise_density = 1.0;
  // Kalman only: extrapolate each report forward by its delivery delay before fusing it.
  bool compensate_delay = true;
  // Turning only. The turn rate is the change of the reported heading (the direction of the
  // horizontal velocity) over the reports of the last `turn_window_s` seconds, and is used once
  // the reports span at least `turn_min_span_s`; until then the neighbor is assumed to fly
  // straight. A heading is not defined below `turn_min_speed` m/s of horizontal speed. The turn
  // rate is limited to +-`turn_max_rate`, and its uncertainty `turn_rate_sigma` (rad/s) grows
  // the position uncertainty with the square of the prediction time. All five are assumptions.
  double turn_window_s = 3.0;
  double turn_min_span_s = 1.0;
  double turn_min_speed = 1.0;
  double turn_max_rate = 0.8;
  double turn_rate_sigma = 0.03;
};

// Throws std::invalid_argument for an invalid configuration.
std::unique_ptr<INeighborPredictor> make_neighbor_predictor(PredictorKind kind,
                                                            const NeighborPredictorConfig& config);

}  // namespace swarm
