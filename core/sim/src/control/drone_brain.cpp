#include "swarm/control/drone_brain.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace swarm {

namespace {

// An orbiting aircraft that is farther than this fraction of the radius from its circle is joining
// it, and the terrain ahead along its heading counts as well as the circle.
constexpr double kTerrainStraightFraction = 0.25;

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
  if (c.avoid.enabled) {
    validate(c.avoid);
    if (!c.use_forward_sensor) {
      throw std::invalid_argument("BrainConfig: avoidance needs the forward sensor");
    }
  }
  if (c.report_period_ticks == 0 || c.staleness_ticks == 0) {
    throw std::invalid_argument("BrainConfig: report period and staleness must be positive");
  }
  if (c.guidance == Guidance::kOrbit) {
    swarm::validate(c.orbit);
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
  if (input.altimeter.available) {
    altimeter_ = input.altimeter;
    altimeter_tick_ = tick;
  }
  const PoseEstimate pose = pose_.estimate();
  const double own_sigma = max_axis(pose.position_sigma);
  if (input.forward.available) {
    forward_ = input.forward;
    forward_position_ = pose.position;
    forward_tick_ = tick;
  }

  // Neighbor reports. The table filters out stale, duplicate, and malformed payloads.
  for (const InboundReport& in : input.inbox) {
    if (in.type == MessageType::kCommand) {
      apply_command(in, pose.position);
      continue;
    }
    const ReceiveResult result =
        table_.on_message(in.sender, in.payload.data(), in.payload.size(), tick);
    if (result == ReceiveResult::kAccepted && in.sender != config_.id) {
      predictors_[in.sender]->on_report(table_.get(in.sender)->report, tick);
    }
  }
  const bool supplied = input.neighbor_override.has_value();
  if (supplied && input.neighbor_override->size() != config_.num_drones) {
    throw std::invalid_argument("DroneBrain::step: one neighbor estimate per drone is required");
  }
  fresh_ = 0;
  for (std::uint32_t j = 0; j < config_.num_drones; ++j) {
    if (j == config_.id) {
      continue;
    }
    if (supplied) {
      // Estimates from outside replace the predictors, and every neighbor counts as fresh.
      estimates_[j] = (*input.neighbor_override)[j];
      ++fresh_;
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
  // A commanded hold behaves like safe hold: stop, hold the position, and keep a wide margin.
  const FallbackBehavior& behavior = commanded_hold_ ? fsm_.behavior_of(FallbackState::kSafeHold)
                                     : config_.fallback_enabled
                                         ? fsm_.behavior()
                                         : fsm_.behavior_of(FallbackState::kNominal);
  if (state == FallbackState::kSafeHold && before != FallbackState::kSafeHold) {
    hold_position_ = pose.position;
    has_hold_ = true;
  }

  BrainOutput out;
  if (config_.guidance == Guidance::kOrbit) {
    out.command.accel = orbit_command(pose, own_sigma, behavior, tick);
    last_command_ = out.command;
    return finish_step(std::move(out), pose, own_sigma, tick);
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
      if (config_.drop_stale_after_ticks > 0 && !supplied) {
        const auto age = table_.age(j, tick);
        if (age && *age > config_.drop_stale_after_ticks) {
          continue;
        }
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
    // A holding drone is not moved by a prediction it cannot check: a neighbor that has gone silent
    // is extrapolated at the speed it had, which for a swarm told to hold together is wrong, and
    // would push the holding drone around. While holding, only neighbors heard from recently
    // count. (Estimates supplied from outside are always current.)
    bool counts = j != config_.id && estimates_[j].valid;
    if (counts && behavior.hold && !supplied) {
      const auto age = table_.age(j, tick);
      counts = age && *age <= config_.staleness_ticks;
    }
    if (counts) {
      others.push_back({j, estimates_[j].position, estimates_[j].position_sigma + own_sigma});
    }
    // A neighbor silent for long enough may have stopped where it was last heard. The prediction
    // carries it onward, so keep its last reported position as a second point to stay clear of.
    // This protects a drone that is moving; a drone that is holding cannot fly into anything, and
    // the last positions of neighbors that were moving when they went silent are often inside its
    // margin, which would push it around for no reason.
    if (j != config_.id && !supplied && !behavior.hold && config_.last_known_after_ticks > 0) {
      const NeighborRecord* record = table_.get(j);
      if (record != nullptr && tick - record->receipt_tick >= config_.last_known_after_ticks) {
        others.push_back({j, record->report.position, record->report.position_sigma + own_sigma});
      }
    }
  }
  const Vec3 push = config_.separation_enabled
                        ? separation_accel(config_.id, pose.position, others, separation)
                        : Vec3{};

  out.command.accel = combine_accel(pull, push, separation.max_accel);
  last_command_ = out.command;
  return finish_step(std::move(out), pose, own_sigma, tick);
}

BrainOutput DroneBrain::finish_step(BrainOutput out, const PoseEstimate& pose, double own_sigma,
                                    std::uint64_t tick) const {
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

// Every neighbor with an estimate counts, silent or not: an aircraft keeps moving, so a
// prediction that cannot be checked is still the best account of where it is.
std::vector<OrbitNeighbor> DroneBrain::neighbors_of(double own_sigma) const {
  std::vector<OrbitNeighbor> others;
  for (std::uint32_t j = 0; j < config_.num_drones; ++j) {
    if (j != config_.id && estimates_[j].valid) {
      others.push_back({j, estimates_[j].position, estimates_[j].velocity,
                        estimates_[j].position_sigma + own_sigma});
    }
  }
  return others;
}

double DroneBrain::terrain_floor_for(const PoseEstimate& pose, double cx, double cy,
                                     double radius) const {
  const TerrainGuidanceParams& params = config_.terrain_guidance;
  if (!params.enabled || config_.terrain == nullptr) {
    return -std::numeric_limits<double>::infinity();
  }
  const double off = std::abs(std::hypot(pose.position.x - cx, pose.position.y - cy) - radius);
  return terrain_floor_over(pose, circle_path(pose.position, cx, cy, radius, params), off, radius);
}

double DroneBrain::altimeter_floor(const PoseEstimate& pose, std::uint64_t tick) const {
  if (!config_.use_altimeter || !altimeter_.available ||
      tick - altimeter_tick_ > config_.altimeter_valid_ticks) {
    return -std::numeric_limits<double>::infinity();
  }
  const double measured = altimeter_.height - config_.altimeter_margin_sigmas * altimeter_.sigma;
  const double shortfall = config_.terrain_guidance.clearance - measured;
  if (!(shortfall > 0.0)) {
    return -std::numeric_limits<double>::infinity();
  }
  return pose.position.z + shortfall;
}

double DroneBrain::forward_floor(const PoseEstimate& pose, std::uint64_t tick) const {
  constexpr double kNone = -std::numeric_limits<double>::infinity();
  if (!config_.use_forward_sensor || !forward_.available ||
      tick - forward_tick_ > config_.forward_valid_ticks) {
    return kNone;
  }
  const TerrainGuidanceParams& params = config_.terrain_guidance;
  const double speed =
      std::max(std::hypot(pose.velocity.x, pose.velocity.y), config_.orbit.min_speed);
  const double budget = params.climb_fraction * params.climb_rate;
  double floor = kNone;
  for (const ForwardRay& ray : forward_.rays) {
    if (!ray.hit) {
      continue;
    }
    // The higher end of the range's error: farther along a rising ray, nearer along a falling one.
    const double margin = config_.forward_margin_sigmas * ray.sigma;
    const double range = std::max(0.0, ray.range + (ray.direction.z > 0.0 ? margin : -margin));
    const Vec3 point = forward_position_ + ray.direction * range;
    const double distance = std::hypot(point.x - pose.position.x, point.y - pose.position.y);
    floor = std::max(floor, point.z + params.clearance - budget * distance / speed);
  }
  return floor;
}

double DroneBrain::sensor_floor(const PoseEstimate& pose, std::uint64_t tick) const {
  return std::max(altimeter_floor(pose, tick), forward_floor(pose, tick));
}

double DroneBrain::terrain_floor_over(const PoseEstimate& pose, std::vector<PathSample> path,
                                      double off, double scale) const {
  const TerrainGuidanceParams& params = config_.terrain_guidance;
  if (!params.enabled || config_.terrain == nullptr) {
    return -std::numeric_limits<double>::infinity();
  }
  const double horizontal = std::hypot(pose.velocity.x, pose.velocity.y);
  if (horizontal > 1e-9 && off > kTerrainStraightFraction * scale) {
    const std::vector<PathSample> ahead = straight_path(pose.position, pose.velocity.x / horizontal,
                                                        pose.velocity.y / horizontal, params);
    path.insert(path.end(), ahead.begin(), ahead.end());
  }
  const double speed = std::max(horizontal, config_.orbit.min_speed);
  return terrain_floor(*config_.terrain, params, path, speed);
}

Vec3 DroneBrain::orbit_command(const PoseEstimate& pose, double own_sigma,
                               const FallbackBehavior& behavior, std::uint64_t tick) {
  const OrbitParams& orbit = config_.orbit;
  Vec3 track;
  if (behavior.hold) {
    if (!hold_orbit_valid_) {
      // Fly a circle through where the aircraft is now: the center is one radius to its left.
      const double speed_h = std::hypot(pose.velocity.x, pose.velocity.y);
      const Vec3 left = speed_h > 1e-9
                            ? Vec3{-pose.velocity.y / speed_h, pose.velocity.x / speed_h, 0.0}
                            : Vec3{0.0, 1.0, 0.0};
      hold_center_ = pose.position + left * orbit.radius;
      hold_z_ = pose.position.z;
      hold_orbit_valid_ = true;
    }
    const double z_hold =
        std::max({hold_z_, terrain_floor_for(pose, hold_center_.x, hold_center_.y, orbit.radius),
                  sensor_floor(pose, tick)});
    track = orbit_guidance(pose.position, pose.velocity, hold_center_, orbit.radius, orbit.speed,
                           z_hold, 0.0, orbit, 1.0);
    target_ = FormationTarget{};
    target_.position = pose.position;
  } else if (config_.pattern.kind == PatternKind::kFormation) {
    hold_orbit_valid_ = false;
    const double time = static_cast<double>(tick) * config_.dt;
    const PatternReference ref =
        pattern_reference(config_.id, config_.num_drones, time, config_.pattern, orbit);
    const double slot_offset = ref.position.z - orbit.center.z;
    double floor = -std::numeric_limits<double>::infinity();
    if (config_.terrain_guidance.enabled && config_.terrain != nullptr) {
      floor = terrain_floor_over(pose,
                                 formation_samples(config_.id, time, config_.pattern, orbit,
                                                   config_.terrain_guidance),
                                 0.0, 1.0) +
              slot_offset;
    }
    const double z_ref = std::max({ref.position.z, floor, sensor_floor(pose, tick)});
    track = formation_guidance(pose.position, pose.velocity, ref, z_ref, config_.pattern, orbit,
                               behavior.gain_scale);
    target_ = FormationTarget{};
    target_.position = ref.position;
    target_.velocity = ref.velocity;
  } else if (config_.pattern.kind != PatternKind::kOrbit) {
    hold_orbit_valid_ = false;
    const double time = static_cast<double>(tick) * config_.dt;
    const PatternReference ref =
        pattern_reference(config_.id, config_.num_drones, time, config_.pattern, orbit);
    const StadiumPath path = pattern_path(config_.id, config_.num_drones, config_.pattern, orbit);
    const double phase_error = path_phase_error(path, pose.position, ref.s);
    const double slot_offset = ref.position.z - orbit.center.z;
    double floor = -std::numeric_limits<double>::infinity();
    if (config_.terrain_guidance.enabled && config_.terrain != nullptr) {
      floor = terrain_floor_over(pose, path_samples(path, pose.position, config_.terrain_guidance),
                                 path.nearest(pose.position.x, pose.position.y).distance,
                                 path.radius()) +
              slot_offset;
    }
    const double z_ref = std::max({ref.position.z, floor, sensor_floor(pose, tick)});
    track = path_guidance(pose.position, pose.velocity, path, ref.speed, z_ref, phase_error,
                          config_.pattern, orbit, behavior.gain_scale);
    target_ = FormationTarget{};
    target_.position = ref.position;
    target_.velocity = ref.velocity;
  } else {
    hold_orbit_valid_ = false;
    const double time = static_cast<double>(tick) * config_.dt;
    const OrbitReference ref = orbit_reference(config_.id, config_.num_drones, time, orbit);
    double phase_error = orbit_phase_error(pose.position, orbit.center, ref.angle);
    // What the neighbors say about this drone's phase, blended with the shared clock's.
    double consensus = 0.0;
    if (orbit.consensus_weight > 0.0 &&
        orbit_consensus_phase_error(config_.id, config_.num_drones, time, pose.position,
                                    neighbors_of(own_sigma), orbit, consensus)) {
      phase_error =
          (1.0 - orbit.consensus_weight) * phase_error + orbit.consensus_weight * consensus;
    }
    // Over high ground the altitude is raised to clear it, and the slot keeps its offset from the
    // bottom layer so the layers stay apart.
    const double slot_offset = ref.position.z - orbit.center.z;
    const double z_ref =
        std::max({ref.position.z,
                  terrain_floor_for(pose, orbit.center.x, orbit.center.y, ref.radius) + slot_offset,
                  sensor_floor(pose, tick)});
    track = orbit_guidance(pose.position, pose.velocity, orbit.center, ref.radius, ref.speed, z_ref,
                           phase_error, orbit, behavior.gain_scale);
    target_ = FormationTarget{};
    target_.position = ref.position;
    target_.velocity = ref.velocity;
  }

  if (config_.avoid.enabled) {
    track = apply_avoidance(track, pose, tick);
  }

  Vec3 push;
  if (config_.separation_enabled) {
    push = orbit_separation_accel(config_.id, pose.position, pose.velocity, neighbors_of(own_sigma),
                                  orbit, behavior.margin_scale);
  }
  return combine_accel(track, push, orbit.max_accel);
}

Vec3 DroneBrain::apply_avoidance(const Vec3& track, const PoseEstimate& pose, std::uint64_t tick) {
  const AvoidParams& params = config_.avoid;
  AvoidResult result;
  if (config_.use_forward_sensor && forward_.available &&
      tick - forward_tick_ <= config_.forward_valid_ticks) {
    AvoidInput input;
    input.sample = &forward_;
    input.sample_position = forward_position_;
    input.position = pose.position;
    input.velocity = pose.velocity;
    input.clearance = config_.terrain_guidance.clearance;
    input.climb_budget =
        config_.terrain_guidance.climb_fraction * config_.terrain_guidance.climb_rate;
    input.climb_limit = config_.terrain_guidance.climb_rate;
    input.guidance_accel = track;
    input.margin_sigmas = config_.forward_margin_sigmas;
    input.min_speed = config_.orbit.min_speed;
    result = assess_avoidance(params, input);
  }
  const double horizontal = std::hypot(pose.velocity.x, pose.velocity.y);
  if (result.threatened) {
    avoid_armed_ = true;
    avoid_until_tick_ = tick + static_cast<std::uint64_t>(std::ceil(params.hold_s / config_.dt));
  } else if (!avoid_armed_ || tick > avoid_until_tick_ || !(horizontal > 1e-9)) {
    return track;
  }
  // The aircraft's heading and its left. While a threat is seen the sideways acceleration turns
  // toward the chosen column and the speed comes down if there is no time; after it the heading is
  // held, with no horizontal acceleration, until the hold runs out.
  const double hx = pose.velocity.x / horizontal;
  const double hy = pose.velocity.y / horizontal;
  double sideways = 0.0;
  double along = 0.0;
  if (result.threatened) {
    sideways = result.turn_accel;
    if (result.slow) {
      const double target_speed = config_.orbit.min_speed + config_.orbit.speed_margin;
      along = -std::max(0.0, horizontal - target_speed);
    }
  }
  const double vertical =
      std::clamp(track.z, -params.max_vertical_accel, params.max_vertical_accel);
  return Vec3{hx * along - hy * sideways, hy * along + hx * sideways, vertical};
}

void DroneBrain::apply_command(const InboundReport& in, const Vec3& position) {
  Command command;
  if (decode_command(in.payload.data(), in.payload.size(), command) != CommandStatus::kOk) {
    ++command_counters_.malformed;
    return;
  }
  // Only a command issued later than the last one applied counts, so a reordered pair or a
  // duplicate cannot leave the drone in the wrong state.
  if (has_command_ && command.issue_tick <= last_command_tick_) {
    ++command_counters_.stale;
    return;
  }
  has_command_ = true;
  last_command_tick_ = command.issue_tick;
  ++command_counters_.applied;
  if (command.kind == CommandKind::kHold) {
    const bool safe_hold = config_.fallback_enabled && fsm_.state() == FallbackState::kSafeHold;
    if (!commanded_hold_ && !safe_hold) {
      hold_position_ = position;  // hold where the drone is now
      has_hold_ = true;
    }
    commanded_hold_ = true;
  } else {
    commanded_hold_ = false;
  }
}

void DroneBrain::hash_state(Fnv1a64& hash) const {
  fsm_.hash_state(hash);
  hash.update_u64(started_ ? 1 : 0);
  hash.update_u64(last_tick_);
  hash.update_u64(last_fix_tick_);
  hash.update_u64(fresh_);
  hash.update_u64(has_hold_ ? 1 : 0);
  hash_vec(hash, hold_position_);
  hash.update_u64(commanded_hold_ ? 1 : 0);
  if (config_.guidance == Guidance::kOrbit) {
    hash.update_u64(hold_orbit_valid_ ? 1 : 0);
    hash_vec(hash, hold_center_);
    hash.update_f64(hold_z_);
  }
  if (config_.avoid.enabled) {
    hash.update_u64(avoid_armed_ ? 1 : 0);
    hash.update_u64(avoid_until_tick_);
  }
  hash.update_u64(has_command_ ? 1 : 0);
  hash.update_u64(last_command_tick_);
  hash.update_u64(command_counters_.applied);
  hash.update_u64(command_counters_.stale);
  hash.update_u64(command_counters_.malformed);

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
