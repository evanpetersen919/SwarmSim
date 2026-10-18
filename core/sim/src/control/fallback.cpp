#include "swarm/control/fallback.hpp"

#include <cmath>
#include <stdexcept>

namespace swarm {

namespace {

bool fraction(double v) { return std::isfinite(v) && v >= 0.0 && v <= 1.0; }
bool positive(double v) { return std::isfinite(v) && v > 0.0; }

void validate(const FallbackBehavior& b) {
  const auto ok = [](double v) { return std::isfinite(v) && v >= 0.0; };
  if (!ok(b.reference_boost) || b.reference_boost > 1.0 || !ok(b.speed_scale) ||
      !ok(b.margin_scale) || !ok(b.gain_scale)) {
    throw std::invalid_argument("FallbackBehavior: scales must be finite and non-negative");
  }
}

}  // namespace

FallbackFsm::FallbackFsm(const FallbackParams& p) : params_(p) {
  if (!fraction(p.comms_enter_fraction) || !fraction(p.comms_exit_fraction) ||
      p.comms_exit_fraction < p.comms_enter_fraction) {
    throw std::invalid_argument("FallbackParams: comms fractions must satisfy enter <= exit");
  }
  if (!positive(p.sigma_enter) || !positive(p.sigma_exit) || !positive(p.sigma_abort) ||
      p.sigma_exit > p.sigma_enter || p.sigma_enter >= p.sigma_abort) {
    throw std::invalid_argument("FallbackParams: sigmas must satisfy exit <= enter < abort");
  }
  if (p.fix_exit_ticks > p.fix_enter_ticks) {
    throw std::invalid_argument("FallbackParams: fix_exit_ticks must not exceed fix_enter_ticks");
  }
  validate(p.nominal);
  validate(p.degraded_comms);
  validate(p.degraded_navigation);
  validate(p.safe_hold);
}

const FallbackBehavior& FallbackFsm::behavior_of(FallbackState state) const {
  switch (state) {
    case FallbackState::kDegradedComms:
      return params_.degraded_comms;
    case FallbackState::kDegradedNavigation:
      return params_.degraded_navigation;
    case FallbackState::kSafeHold:
      return params_.safe_hold;
    case FallbackState::kNominal:
      break;
  }
  return params_.nominal;
}

void FallbackFsm::hash_state(Fnv1a64& hash) const {
  hash.update_u64(static_cast<std::uint64_t>(state_));
  hash.update_u64(comms_bad_ ? 1 : 0);
  hash.update_u64(navigation_bad_ ? 1 : 0);
  hash.update_u64(has_dwell_ ? 1 : 0);
  hash.update_u64(state_since_);
  hash.update_u64(both_bad_ ? 1 : 0);
  hash.update_u64(both_bad_since_);
  hash.update_u64(all_good_ ? 1 : 0);
  hash.update_u64(all_good_since_);
  hash.update_u64(started_ ? 1 : 0);
  hash.update_u64(last_tick_);
}

void FallbackFsm::enter(FallbackState next, std::uint64_t tick) {
  state_ = next;
  state_since_ = tick;
  has_dwell_ = true;
}

FallbackState FallbackFsm::update(const FallbackInputs& in, std::uint64_t tick) {
  if (started_ && tick < last_tick_) {
    throw std::invalid_argument("FallbackFsm::update: ticks must not decrease");
  }
  if (in.fresh_neighbors > in.total_neighbors) {
    throw std::invalid_argument("FallbackFsm::update: fresh neighbors exceed the total");
  }
  if (!(in.pose_sigma >= 0.0)) {  // also rejects NaN
    throw std::invalid_argument("FallbackFsm::update: pose sigma must be non-negative");
  }
  started_ = true;
  last_tick_ = tick;

  // Each condition is a latch with separate enter and exit thresholds.
  if (in.total_neighbors == 0) {
    comms_bad_ = false;
  } else {
    const double fresh = static_cast<double>(in.fresh_neighbors) / in.total_neighbors;
    comms_bad_ = fresh < (comms_bad_ ? params_.comms_exit_fraction : params_.comms_enter_fraction);
  }
  if (navigation_bad_) {
    navigation_bad_ =
        in.ticks_since_fix > params_.fix_exit_ticks || in.pose_sigma > params_.sigma_exit;
  } else {
    navigation_bad_ =
        in.ticks_since_fix > params_.fix_enter_ticks || in.pose_sigma > params_.sigma_enter;
  }

  const bool both = comms_bad_ && navigation_bad_;
  if (both && !both_bad_) {
    both_bad_since_ = tick;
  }
  both_bad_ = both;
  const bool good = !comms_bad_ && !navigation_bad_;
  if (good && !all_good_) {
    all_good_since_ = tick;
  }
  all_good_ = good;

  if (state_ == FallbackState::kSafeHold) {
    // Leaving needs the conditions to have stayed good for the recovery window.
    if (all_good_ && tick - all_good_since_ >= params_.recovery_ticks &&
        tick - state_since_ >= params_.min_dwell_ticks) {
      enter(FallbackState::kNominal, tick);
    }
    return state_;
  }

  // Safe hold is entered without waiting for the dwell time.
  if (in.pose_sigma > params_.sigma_abort ||
      (both_bad_ && tick - both_bad_since_ >= params_.hold_ticks)) {
    enter(FallbackState::kSafeHold, tick);
    return state_;
  }

  FallbackState wanted = FallbackState::kNominal;
  if (navigation_bad_) {
    wanted = FallbackState::kDegradedNavigation;  // also while both are bad but not yet held
  } else if (comms_bad_) {
    wanted = FallbackState::kDegradedComms;
  }
  if (wanted != state_ && (!has_dwell_ || tick - state_since_ >= params_.min_dwell_ticks)) {
    enter(wanted, tick);
  }
  return state_;
}

}  // namespace swarm
