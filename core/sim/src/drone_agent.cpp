#include "swarm/drone_agent.hpp"

#include <stdexcept>
#include <utility>
#include <vector>

namespace swarm {

std::unique_ptr<const IVehicle> DroneAgent::make_vehicle(const DroneAgentConfig& config) {
  if (config.vehicle_kind == VehicleKind::kFixedWing) {
    return std::make_unique<FixedWingVehicle>(config.fixed_wing);
  }
  return std::make_unique<QuadVehicle>(config.vehicle);
}

DroneAgentConfig DroneAgent::prepared(DroneAgentConfig config) {
  if (config.fix_period_ticks == 0) {
    throw std::invalid_argument("DroneAgentConfig: fix_period_ticks must be positive");
  }
  if (config.vehicle_kind == VehicleKind::kFixedWing) {
    validate(config.fixed_wing);
  }
  config.brain.launch.position = config.start;
  config.brain.launch.velocity = config.start_velocity;
  config.brain.pose.accel_noise_density = config.imu.accel_noise_density;
  config.brain.pose.accel_bias_walk = config.imu.accel_bias_walk;
  config.brain.pose.gravity = config.imu.gravity;
  return config;
}

DroneAgent::DroneAgent(const DroneAgentConfig& config, const IScene& scene)
    : config_(prepared(config)),
      scene_(scene),
      vehicle_(make_vehicle(config_)),
      imu_(config_.imu, config_.brain.dt, config_.seed, config_.brain.id),
      brain_(config_.brain) {
  if (config_.vehicle_kind == VehicleKind::kFixedWing) {
    camera_.emplace(config_.camera, config_.seed, config_.brain.id);
  } else {
    lidar_.emplace(config_.lidar, config_.seed, config_.brain.id);
    fixer_.emplace(config_.fix, config_.seed, config_.brain.id);
  }
  config_.weather.validate();
  truth_.pos = config_.start;
  truth_.vel = config_.start_velocity;
}

void DroneAgent::step(AgentContext& ctx) {
  if (failed_ || ctx.tick >= config_.fail_tick) {
    const bool first_tick = !failed_;
    failed_ = true;
    if (config_.vehicle_kind == VehicleKind::kFixedWing) {
      // An aircraft cannot hang in the air: with no one controlling it, it flies on with no
      // command, straight and at constant speed, until the end of the run. It senses and sends
      // nothing.
      truth_ = vehicle_->step(truth_, DroneCommand{}, config_.brain.dt);
    } else if (first_tick) {
      truth_.vel = Vec3{};
      truth_.accel = Vec3{};
      truth_.yaw_rate = 0.0;
    }
    return;
  }

  // Sensors read the truth; the brain gets only what they report.
  BrainInput in;
  in.imu = imu_.sample(truth_);
  if (ctx.tick % config_.fix_period_ticks == 0) {
    if (camera_) {
      in.fix = camera_->measure(truth_.pos, scene_.ground_height(truth_.pos.x, truth_.pos.y),
                                config_.weather);
    } else {
      const Scan scan = lidar_->scan(truth_.pos, truth_.yaw, scene_, config_.weather);
      in.fix = fixer_->measure(truth_.pos, scan);
    }
  }
  in.inbox.reserve(ctx.inbox.size());
  for (Message& m : ctx.inbox) {
    in.inbox.push_back({m.sender, std::move(m.payload), m.type});
  }

  if (config_.oracle) {
    // The previous tick's true state of every other drone, with zero uncertainty and zero age.
    std::vector<NeighborEstimate> truth(config_.brain.num_drones);
    for (std::uint32_t j = 0; j < config_.brain.num_drones; ++j) {
      if (j != config_.brain.id) {
        truth[j].valid = true;
        truth[j].position = ctx.peers[j].position;
        truth[j].velocity = ctx.peers[j].velocity;
      }
    }
    in.neighbor_override = std::move(truth);
  }

  const BrainOutput out = brain_.step(in, ctx.tick);
  truth_ = vehicle_->step(truth_, out.command, config_.brain.dt);

  if (out.has_report) {
    Message m;
    m.payload = out.report;
    for (std::uint32_t other = 0; other < config_.brain.num_drones; ++other) {
      if (other != config_.brain.id) {
        m.receiver = other;
        ctx.outbox.push_back(m);
      }
    }
  }
}

void DroneAgent::hash_state(Fnv1a64& hash) const {
  hash.update_f64(truth_.pos.x);
  hash.update_f64(truth_.pos.y);
  hash.update_f64(truth_.pos.z);
  hash.update_f64(truth_.vel.x);
  hash.update_f64(truth_.vel.y);
  hash.update_f64(truth_.vel.z);
  hash.update_f64(truth_.accel.x);
  hash.update_f64(truth_.accel.y);
  hash.update_f64(truth_.accel.z);
  hash.update_f64(truth_.yaw);
  hash.update_f64(truth_.yaw_rate);
  hash.update_u64(failed_ ? 1 : 0);
  const Vec3 bias = imu_.accel_bias();
  hash.update_f64(bias.x);
  hash.update_f64(bias.y);
  hash.update_f64(bias.z);
  hash.update_f64(imu_.gyro_bias());
  brain_.hash_state(hash);
}

}  // namespace swarm
