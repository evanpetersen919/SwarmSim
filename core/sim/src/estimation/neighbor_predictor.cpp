#include "swarm/estimation/neighbor_predictor.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
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
  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  if (!positive(c.turn_window_s) || !positive(c.turn_min_span_s) ||
      c.turn_min_span_s > c.turn_window_s) {
    throw std::invalid_argument(
        "NeighborPredictorConfig: turn_min_span_s must be positive and at most turn_window_s");
  }
  if (!positive(c.turn_min_speed) || !positive(c.turn_max_rate) ||
      !(std::isfinite(c.turn_rate_sigma) && c.turn_rate_sigma >= 0.0)) {
    throw std::invalid_argument(
        "NeighborPredictorConfig: turn_min_speed and turn_max_rate must be positive and "
        "turn_rate_sigma not negative");
  }
}

constexpr double kPi = 3.14159265358979323846;

double wrap_angle(double angle) {
  double wrapped = std::fmod(angle + kPi, 2.0 * kPi);
  if (wrapped <= 0.0) {
    wrapped += 2.0 * kPi;
  }
  return wrapped - kPi;
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

// Constant speed and constant turn rate. The reports carry no turn rate, so it is taken from how
// the reported heading changed over the recent reports. An aircraft on a circle is predicted along
// that circle; a straight flight (no measurable turn) is predicted as dead reckoning does.
class TurningPredictor final : public INeighborPredictor {
 public:
  explicit TurningPredictor(const NeighborPredictorConfig& config) : config_(config) {}

  void on_report(const NeighborReport& report, std::uint64_t receipt_tick) override {
    if (!history_.empty()) {
      if (receipt_tick < receipt_tick_) {
        throw std::invalid_argument("on_report: receipt ticks must not decrease");
      }
      if (report.send_tick <= history_.back().send_tick) {
        return;  // stale or duplicate
      }
    }
    history_.push_back(report);
    receipt_tick_ = receipt_tick;
    const double window = config_.turn_window_s / config_.dt;
    while (history_.size() > 2 &&
           static_cast<double>(report.send_tick - history_.front().send_tick) > window) {
      history_.pop_front();
    }
    turn_rate_ = estimate_turn_rate();
  }

  NeighborEstimate predict(std::uint64_t now_tick) const override {
    NeighborEstimate e;
    if (history_.empty()) {
      return e;
    }
    if (now_tick < receipt_tick_) {
      throw std::invalid_argument("predict: now_tick is earlier than the latest receipt");
    }
    const NeighborReport& r = history_.back();
    const std::uint64_t since_send = now_tick > r.send_tick ? now_tick - r.send_tick : 0;
    const double tau = static_cast<double>(since_send) * config_.dt;
    const double speed = std::hypot(r.velocity.x, r.velocity.y);
    const double heading = std::atan2(r.velocity.y, r.velocity.x);
    const double omega = turn_rate_;

    e.valid = true;
    e.age = now_tick - receipt_tick_;
    if (speed < config_.turn_min_speed || std::abs(omega) < 1e-9) {
      e.position = r.position + r.velocity * tau;
      e.velocity = r.velocity;
    } else {
      const double turned = heading + omega * tau;
      e.position = {r.position.x + speed / omega * (std::sin(turned) - std::sin(heading)),
                    r.position.y + speed / omega * (std::cos(heading) - std::cos(turned)),
                    r.position.z + r.velocity.z * tau};
      e.velocity = {speed * std::cos(turned), speed * std::sin(turned), r.velocity.z};
    }
    const double drift = config_.report_velocity_sigma * tau;
    const double bend = 0.5 * speed * config_.turn_rate_sigma * tau * tau;
    e.position_sigma = std::sqrt(r.position_sigma * r.position_sigma + drift * drift + bend * bend);
    return e;
  }

 private:
  // The heading change from the oldest report in the window to the newest, over their time apart.
  double estimate_turn_rate() const {
    const NeighborReport& newest = history_.back();
    const NeighborReport& oldest = history_.front();
    const double span = static_cast<double>(newest.send_tick - oldest.send_tick) * config_.dt;
    if (span < config_.turn_min_span_s) {
      return 0.0;
    }
    const double s_new = std::hypot(newest.velocity.x, newest.velocity.y);
    const double s_old = std::hypot(oldest.velocity.x, oldest.velocity.y);
    if (s_new < config_.turn_min_speed || s_old < config_.turn_min_speed) {
      return 0.0;
    }
    const double change = wrap_angle(std::atan2(newest.velocity.y, newest.velocity.x) -
                                     std::atan2(oldest.velocity.y, oldest.velocity.x));
    return std::clamp(change / span, -config_.turn_max_rate, config_.turn_max_rate);
  }

  NeighborPredictorConfig config_;
  std::deque<NeighborReport> history_;
  std::uint64_t receipt_tick_ = 0;
  double turn_rate_ = 0.0;
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
    case PredictorKind::kTurning:
      return std::make_unique<TurningPredictor>(config);
  }
  throw std::invalid_argument("make_neighbor_predictor: unknown predictor kind");
}

}  // namespace swarm
