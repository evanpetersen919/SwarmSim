#include "swarm/control/drone_brain.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace swarm {

namespace {

double max_axis(const Vec3& v) { return std::max({v.x, v.y, v.z}); }

void hash_vec(Fnv1a64& hash, const Vec3& v) {
  hash.update_f64(v.x);
  hash.update_f64(v.y);
  hash.update_f64(v.z);
}

void validate(const BrainConfig& c) {
  if (c.num_drones == 0 || c.id >= c.num_drones) {
    throw std::invalid_argument("BrainConfig: id must be below num_drones");
  }
  if (c.offsets.size() != c.num_drones) {
    throw std::invalid_argument("BrainConfig: one formation offset per drone is required");
  }
  if (!(std::isfinite(c.dt) && c.dt > 0.0) || c.pose.dt != c.dt || c.predictor_config.dt != c.dt) {
    throw std::invalid_argument("BrainConfig: dt must be positive and match pose and predictors");
  }
  // Slots closer than the largest separation margin would repel one another permanently.
  for (std::size_t a = 0; a < c.offsets.size(); ++a) {
    for (std::size_t b = a + 1; b < c.offsets.size(); ++b) {
      if (!(distance(c.offsets[a], c.offsets[b]) > c.separation.max_margin)) {
        throw std::invalid_argument(
            "BrainConfig: formation slots must be farther apart than max_margin");
      }
    }
  }
  if (!(std::isfinite(c.speed_limit_factor) && c.speed_limit_factor >= 1.0)) {
    throw std::invalid_argument("BrainConfig: speed_limit_factor must be at least 1");
  }
  if (c.report_period_ticks == 0 || c.staleness_ticks == 0) {
    throw std::invalid_argument("BrainConfig: report period and staleness must be positive");
  }
}

}  // namespace

DroneBrain::DroneBrain(const BrainConfig& config)
    : config_((validate(config), config)),
      path_(config.mission),
      pose_(config.pose, config.launch),
      table_(config.num_drones),
      fsm_(config.fallback),
      estimates_(config.num_drones) {
  predictors_.resize(config_.num_drones);
  for (std::uint32_t j = 0; j < config_.num_drones; ++j) {
    if (j != config_.id) {
      predictors_[j] = make_neighbor_predictor(config_.predictor, config_.predictor_config);
    }
  }
  // Validates the formation, separation, and offset parameters now rather than on the first step.
  const FormationParams& f = config_.formation;
  formation_target(config_.id, config_.launch.position, config_.launch.velocity, 0.0, {},
                   config_.offsets, MissionReference{}, f);
  separation_accel(config_.id, config_.launch.position, {}, config_.separation);
}

BrainOutput DroneBrain::step(const BrainInput& input, std::uint64_t tick) {
  if (started_ && tick < last_tick_) {
    throw std::invalid_argument("DroneBrain::step: ticks must not decrease");
  }
  started_ = true;
  last_tick_ = tick;

  // Estimation.
  pose_.predict(input.imu);
  pose_.update(input.fix);
  if (input.fix.available) {
    last_fix_tick_ = tick;
  }
  const PoseEstimate pose = pose_.estimate();
  const double own_sigma = max_axis(pose.position_sigma);

  // Neighbor reports. The table filters out stale, duplicate, and malformed payloads.
  for (const InboundReport& in : input.inbox) {
    const ReceiveResult result =
        table_.on_message(in.sender, in.payload.data(), in.payload.size(), tick);
    if (result == ReceiveResult::kAccepted && in.sender != config_.id) {
      predictors_[in.sender]->on_report(table_.get(in.sender)->report, tick);
    }
  }
  fresh_ = 0;
  for (std::uint32_t j = 0; j < config_.num_drones; ++j) {
    if (j == config_.id) {
      continue;
    }
    estimates_[j] = predictors_[j]->predict(tick);
    const auto age = table_.age(j, tick);
    if (age && *age <= config_.staleness_ticks) {
      ++fresh_;
    }
  }

  // Fallback state.
  const FallbackState before = fsm_.state();
  FallbackInputs fb;
  // Before the first staleness window has passed there has been no time to hear from anyone, so
  // comms is not judged yet.
  fb.fresh_neighbors = tick < config_.staleness_ticks ? config_.num_drones - 1 : fresh_;
  fb.total_neighbors = config_.num_drones - 1;
  fb.ticks_since_fix = tick - last_fix_tick_;
  fb.pose_sigma = own_sigma;
  const FallbackState state = fsm_.update(fb, tick);
  const FallbackBehavior& behavior = fsm_.behavior();
  if (state == FallbackState::kSafeHold && before != FallbackState::kSafeHold) {
    hold_position_ = pose.position;
    has_hold_ = true;
  }

  // Control.
  FormationParams formation = config_.formation;
  Vec3 pull;
  if (behavior.hold) {
    FormationTarget hold;
    hold.position = hold_position_;
    pull = formation_accel(hold, pose.position, pose.velocity, formation);
    target_ = hold;
  } else {
    std::vector<FormationNeighbor> neighbors;
    for (std::uint32_t j = 0; j < config_.num_drones; ++j) {
      if (j == config_.id || !estimates_[j].valid) {
        continue;
      }
      neighbors.push_back({j, estimates_[j]});
    }
    formation.alpha += (1.0 - formation.alpha) * behavior.reference_boost;
    const MissionReference reference = path_.at(static_cast<double>(tick) * config_.dt);
    target_ = formation_target(config_.id, pose.position, pose.velocity, own_sigma, neighbors,
                               config_.offsets, reference, formation);

    formation.kp *= behavior.gain_scale;
    formation.kd *= behavior.gain_scale;
    pull = formation_accel(target_, pose.position, pose.velocity, formation);

    // Speed limiter: above the state's limit, drop the forward acceleration and brake. The nominal
    // limit is a multiple of the cruise speed, because closing a gap needs a brief excess over it.
    const double limit = config_.mission.speed * config_.speed_limit_factor * behavior.speed_scale;
    const double speed = norm(pose.velocity);
    if (speed > limit && speed > 0.0) {
      const Vec3 heading = pose.velocity / speed;
      const double forward = dot(pull, heading);
      if (forward > 0.0) {
        pull -= heading * forward;
      }
      pull -= heading * (config_.formation.kd * (speed - limit));
    }
  }

  // Separation, with the margin widened by this drone's own uncertainty.
  SeparationParams separation = config_.separation;
  separation.d_safe *= behavior.margin_scale;
  std::vector<SeparationNeighbor> others;
  for (std::uint32_t j = 0; j < config_.num_drones; ++j) {
    if (j != config_.id && estimates_[j].valid) {
      others.push_back({j, estimates_[j].position, estimates_[j].position_sigma + own_sigma});
    }
  }
  const Vec3 push = separation_accel(config_.id, pose.position, others, separation);

  BrainOutput out;
  out.command.accel = combine_accel(pull, push, separation.max_accel);
  last_command_ = out.command;

  // Broadcast the drone's own estimate, never truth.
  if (tick % config_.report_period_ticks == 0) {
    NeighborReport report;
    report.send_tick = tick;
    report.position = pose.position;
    report.velocity = pose.velocity;
    report.position_sigma = own_sigma;
    out.report = encode_neighbor_report(report);
    out.has_report = true;
  }
  return out;
}

void DroneBrain::hash_state(Fnv1a64& hash) const {
  fsm_.hash_state(hash);
  hash.update_u64(started_ ? 1 : 0);
  hash.update_u64(last_tick_);
  hash.update_u64(last_fix_tick_);
  hash.update_u64(fresh_);
  hash.update_u64(has_hold_ ? 1 : 0);
  hash_vec(hash, hold_position_);

  const PoseEstimate p = pose_.estimate();
  hash_vec(hash, p.position);
  hash_vec(hash, p.velocity);
  hash_vec(hash, p.accel_bias);
  hash_vec(hash, p.position_sigma);
  hash_vec(hash, p.velocity_sigma);
  const PoseEstimatorCounters& pc = pose_.counters();
  hash.update_u64(pc.fixes_applied);
  hash.update_u64(pc.fixes_gated);
  hash.update_u64(pc.fixes_singular);
  hash.update_u64(pc.fixes_unavailable);

  const NeighborTableCounters& tc = table_.counters();
  hash.update_u64(tc.accepted);
  hash.update_u64(tc.stale);
  hash.update_u64(tc.duplicate);
  hash.update_u64(tc.malformed);
  hash.update_u64(tc.unknown_sender);
  for (std::uint32_t j = 0; j < config_.num_drones; ++j) {
    const NeighborRecord* record = table_.get(j);
    hash.update_u64(record != nullptr ? record->receipt_tick + 1 : 0);
    const NeighborEstimate& e = estimates_[j];
    hash.update_u64(e.valid ? 1 : 0);
    hash_vec(hash, e.position);
    hash_vec(hash, e.velocity);
    hash.update_f64(e.position_sigma);
    hash.update_u64(e.age);
  }

  hash_vec(hash, target_.position);
  hash_vec(hash, target_.velocity);
  hash.update_u64(target_.neighbors_used);
  hash_vec(hash, last_command_.accel);
  hash.update_f64(last_command_.yaw_rate);
}

}  // namespace swarm
