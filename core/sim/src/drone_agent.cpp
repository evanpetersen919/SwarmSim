#include "swarm/drone_agent.hpp"

#include <stdexcept>
#include <utility>
#include <vector>

namespace swarm {

DroneAgentConfig DroneAgent::prepared(DroneAgentConfig config) {
  if (config.fix_period_ticks == 0) {
    throw std::invalid_argument("DroneAgentConfig: fix_period_ticks must be positive");
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
      imu_(config_.imu, config_.brain.dt, config_.seed, config_.brain.id),
      lidar_(config_.lidar, config_.seed, config_.brain.id),
      fixer_(config_.fix, config_.seed, config_.brain.id),
      brain_(config_.brain) {
  config_.weather.validate();
  truth_.pos = config_.start;
  truth_.vel = config_.start_velocity;
}

void DroneAgent::step(AgentContext& ctx) {
  if (failed_ || ctx.tick >= config_.fail_tick) {
    if (!failed_) {
      failed_ = true;
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
    const Scan scan = lidar_.scan(truth_.pos, truth_.yaw, scene_, config_.weather);
    in.fix = fixer_.measure(truth_.pos, scan);
  }
  in.inbox.reserve(ctx.inbox.size());
  for (Message& m : ctx.inbox) {
    in.inbox.push_back({m.sender, std::move(m.payload)});
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
  truth_ = swarm::step(truth_, out.command, config_.vehicle, config_.brain.dt);

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
