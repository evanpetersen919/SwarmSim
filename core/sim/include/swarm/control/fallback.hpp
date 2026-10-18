#pragma once

// Controller headers may include only this directory, swarm/estimation/, sensor_data.hpp,
// vec3.hpp, drone_command.hpp, and swarm/format/{byte_io,hash}.hpp. They must never include
// vehicle truth.
// Enforced by scripts/check_core_invariants.sh.

#include <cstdint>

#include "swarm/format/hash.hpp"

namespace swarm {

enum class FallbackState : std::uint8_t {
  kNominal = 0,
  kDegradedComms = 1,
  kDegradedNavigation = 2,
  kSafeHold = 3,
};

constexpr int kFallbackStateCount = 4;

// What the machine is told each tick. Counts are of the neighbors this drone expects to hear from.
struct FallbackInputs {
  std::uint32_t fresh_neighbors =
      0;  // neighbors whose last report is newer than the staleness limit
  std::uint32_t total_neighbors = 0;
  std::uint64_t ticks_since_fix = 0;
  double pose_sigma = 0.0;  // meters, the pose estimator's position uncertainty
};

// How a state changes the controller. The drone brain applies the scales.
struct FallbackBehavior {
  double reference_boost = 0.0;  // fraction of (1 - alpha) moved onto the mission reference
  double speed_scale = 1.0;      // multiplies the cruise speed
  double margin_scale = 1.0;     // multiplies the separation margin
  double gain_scale = 1.0;       // multiplies the formation gains
  bool hold = false;             // decelerate to a stop and hold position
};

struct FallbackParams {
  // Comms is bad when the fraction of fresh neighbors falls below comms_enter_fraction and is good
  // again only when it reaches comms_exit_fraction. With no expected neighbors comms is good.
  double comms_enter_fraction = 0.5;
  double comms_exit_fraction = 0.75;

  // Navigation is bad when ticks since the last fix exceed fix_enter_ticks or the pose sigma
  // exceeds sigma_enter. It is good again only when both are back inside the exit values.
  std::uint64_t fix_enter_ticks = 100;
  std::uint64_t fix_exit_ticks = 25;
  double sigma_enter = 3.0;
  double sigma_exit = 1.5;

  // A pose sigma above this enters safe hold at once, ignoring dwell.
  double sigma_abort = 15.0;

  std::uint64_t min_dwell_ticks = 50;  // applies to every transition except those into safe hold
  std::uint64_t hold_ticks = 150;      // both conditions bad for this long enters safe hold
  std::uint64_t recovery_ticks = 150;  // both good for this long leaves safe hold

  FallbackBehavior nominal{0.0, 1.0, 1.0, 1.0, false};
  FallbackBehavior degraded_comms{0.5, 0.6, 1.25, 1.0, false};
  FallbackBehavior degraded_navigation{0.0, 0.5, 1.25, 0.5, false};
  FallbackBehavior safe_hold{0.0, 0.0, 1.25, 0.0, true};
};

class FallbackFsm {
 public:
  // Throws std::invalid_argument if an exit threshold is on the wrong side of its enter threshold,
  // a fraction is outside [0, 1], the sigmas do not satisfy exit <= enter < abort, or a behavior
  // scale is negative or not finite.
  explicit FallbackFsm(const FallbackParams& params);

  // Advances the machine to `tick` and returns the state. Throws std::invalid_argument if ticks
  // decrease, if fresh exceeds total, or if the sigma is negative or NaN.
  FallbackState update(const FallbackInputs& inputs, std::uint64_t tick);

  FallbackState state() const { return state_; }
  bool comms_bad() const { return comms_bad_; }
  bool navigation_bad() const { return navigation_bad_; }
  const FallbackBehavior& behavior() const { return behavior_of(state_); }
  const FallbackBehavior& behavior_of(FallbackState state) const;

  // Folds every field that affects the machine's future behavior into `hash`.
  void hash_state(Fnv1a64& hash) const;

 private:
  void enter(FallbackState next, std::uint64_t tick);

  FallbackParams params_;
  FallbackState state_ = FallbackState::kNominal;
  bool comms_bad_ = false;
  bool navigation_bad_ = false;
  bool has_dwell_ = false;  // false until the first transition, so the start is not held back
  std::uint64_t state_since_ = 0;
  bool both_bad_ = false;
  std::uint64_t both_bad_since_ = 0;
  bool all_good_ = true;
  std::uint64_t all_good_since_ = 0;
  bool started_ = false;
  std::uint64_t last_tick_ = 0;
};

}  // namespace swarm
