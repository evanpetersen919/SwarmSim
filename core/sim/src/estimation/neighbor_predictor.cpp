#include "swarm/estimation/neighbor_predictor.hpp"

#include <cmath>
#include <optional>
#include <stdexcept>

#include "swarm/estimation/kalman.hpp"

namespace swarm {

namespace {

void validate(const NeighborPredictorConfig& c) {
  if (!(std::isfinite(c.dt) && c.dt > 0.0)) {
    throw std::invalid_argument("NeighborPredictorConfig: dt must be finite and positive");
  }
  if (!(std::isfinite(c.report_velocity_sigma) && c.report_velocity_sigma >= 0.0)) {
    throw std::invalid_argument(
        "NeighborPredictorConfig: report_velocity_sigma must be finite and >= 0");
  }
  if (!(std::isfinite(c.process_noise_density) && c.process_noise_density >= 0.0)) {
    throw std::invalid_argument(
        "NeighborPredictorConfig: process_noise_density must be finite and >= 0");
  }
}

// Shared bookkeeping for the predictors that keep only the latest accepted report.
class LatestReport : public INeighborPredictor {
 public:
  void on_report(const NeighborReport& report, std::uint64_t receipt_tick) final {
    if (held_.has_value()) {
      if (receipt_tick < held_->receipt_tick) {
        throw std::invalid_argument("on_report: receipt ticks must not decrease");
      }
      if (report.send_tick <= held_->report.send_tick) {
        return;  // stale or duplicate
      }
    }
    held_ = Held{report, receipt_tick};
  }

 protected:
  struct Held {
    NeighborReport report;
    std::uint64_t receipt_tick;
  };

  std::uint64_t age_at(std::uint64_t now_tick) const {
    if (now_tick < held_->receipt_tick) {
      throw std::invalid_argument("predict: now_tick is earlier than the latest receipt");
    }
    return now_tick - held_->receipt_tick;
  }

  std::optional<Held> held_;
};

class HoldLastPredictor final : public LatestReport {
 public:
  NeighborEstimate predict(std::uint64_t now_tick) const override {
    NeighborEstimate e;
    if (!held_.has_value()) {
      return e;
    }
    e.age = age_at(now_tick);
    e.valid = true;
    e.position = held_->report.position;
    e.velocity = held_->report.velocity;
    e.position_sigma = held_->report.position_sigma;
    return e;
  }
};

class DeadReckoningPredictor final : public LatestReport {
 public:
  explicit DeadReckoningPredictor(const NeighborPredictorConfig& config) : config_(config) {}

  NeighborEstimate predict(std::uint64_t now_tick) const override {
    NeighborEstimate e;
    if (!held_.has_value()) {
      return e;
    }
    e.age = age_at(now_tick);
    // Extrapolate from the time the report was made, not the time it arrived.
    const std::uint64_t send_tick = held_->report.send_tick;
    const std::uint64_t since_send = now_tick > send_tick ? now_tick - send_tick : 0;
    const double elapsed = static_cast<double>(since_send) * config_.dt;
    const NeighborReport& r = held_->report;
    e.valid = true;
    e.position = r.position + r.velocity * elapsed;
    e.velocity = r.velocity;
    const double drift = config_.report_velocity_sigma * elapsed;
    e.position_sigma = std::sqrt(r.position_sigma * r.position_sigma + drift * drift);
    return e;
  }

 private:
  NeighborPredictorConfig config_;
};

class KalmanNeighborPredictor final : public INeighborPredictor {
 public:
  explicit KalmanNeighborPredictor(const NeighborPredictorConfig& config) : config_(config) {}

  void on_report(const NeighborReport& report, std::uint64_t receipt_tick) override {
    if (filter_.has_value()) {
      if (receipt_tick < state_tick_) {
        throw std::invalid_argument("on_report: receipt ticks must not decrease");
      }
      if (report.send_tick <= last_send_tick_) {
        return;  // stale or duplicate
      }
    }

    // The report is a measurement of the neighbor's state at its send tick.
    StateVector<6> z;
    z << report.position.x, report.position.y, report.position.z, report.velocity.x,
        report.velocity.y, report.velocity.z;
    SquareMatrix<6> r = SquareMatrix<6>::Zero();
    const double pos_var = report.position_sigma * report.position_sigma;
    const double vel_var = config_.report_velocity_sigma * config_.report_velocity_sigma;
    r.diagonal() << pos_var, pos_var, pos_var, vel_var, vel_var, vel_var;

    // Delay compensation: carry the report forward to the receipt tick. The extrapolation adds
    // the process noise of the delay to the measurement noise. The measurement and the filter's
    // prior share that process noise, which this treats as independent; the effect is a slightly
    // overconfident estimate.
    if (config_.compensate_delay && receipt_tick > report.send_tick) {
      const double tau = static_cast<double>(receipt_tick - report.send_tick) * config_.dt;
      const SquareMatrix<6> f = constant_velocity_transition(tau);
      z = f * z;
      r = f * r * f.transpose() +
          constant_velocity_process_noise(tau, config_.process_noise_density);
    }

    if (!filter_.has_value()) {
      filter_.emplace(z, r);
    } else {
      advance_to(*filter_, state_tick_, receipt_tick);
      filter_->update<6>(z, SquareMatrix<6>::Identity(), r);
    }
    state_tick_ = receipt_tick;
    last_send_tick_ = report.send_tick;
  }

  NeighborEstimate predict(std::uint64_t now_tick) const override {
    NeighborEstimate e;
    if (!filter_.has_value()) {
      return e;
    }
    if (now_tick < state_tick_) {
      throw std::invalid_argument("predict: now_tick is earlier than the latest receipt");
    }
    LinearKalmanFilter<6> projected = *filter_;
    advance_to(projected, state_tick_, now_tick);

    const StateVector<6>& x = projected.state();
    const SquareMatrix<6>& p = projected.covariance();
    e.valid = true;
    e.age = now_tick - state_tick_;
    e.position = Vec3{x(0), x(1), x(2)};
    e.velocity = Vec3{x(3), x(4), x(5)};
    e.position_sigma = std::sqrt((p(0, 0) + p(1, 1) + p(2, 2)) / 3.0);
    return e;
  }

 private:
  // The constant-velocity model composes exactly, so one long prediction equals many short ones.
  void advance_to(LinearKalmanFilter<6>& filter, std::uint64_t from, std::uint64_t to) const {
    if (to <= from) {
      return;
    }
    const double tau = static_cast<double>(to - from) * config_.dt;
    filter.predict(constant_velocity_transition(tau),
                   constant_velocity_process_noise(tau, config_.process_noise_density));
  }

  NeighborPredictorConfig config_;
  std::optional<LinearKalmanFilter<6>> filter_;
  std::uint64_t state_tick_ = 0;      // the tick the filter state is valid at
  std::uint64_t last_send_tick_ = 0;  // send tick of the latest accepted report
};

}  // namespace

std::unique_ptr<INeighborPredictor> make_neighbor_predictor(PredictorKind kind,
                                                            const NeighborPredictorConfig& config) {
  validate(config);
  switch (kind) {
    case PredictorKind::kHoldLast:
      return std::make_unique<HoldLastPredictor>();
    case PredictorKind::kDeadReckoning:
      return std::make_unique<DeadReckoningPredictor>(config);
    case PredictorKind::kKalman:
      return std::make_unique<KalmanNeighborPredictor>(config);
  }
  throw std::invalid_argument("make_neighbor_predictor: unknown predictor kind");
}

}  // namespace swarm
