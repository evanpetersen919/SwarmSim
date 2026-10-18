#pragma once

// A test agent that exercises dynamics, IMU, LiDAR, weather, and messaging together on the
// lockstep loop. Not part of the library: real drone agents arrive in a later phase.

#include <cstdint>
#include <cstring>
#include <vector>

#include "swarm/agent.hpp"
#include "swarm/dynamics.hpp"
#include "swarm/imu.hpp"
#include "swarm/lidar.hpp"
#include "swarm/scene.hpp"
#include "swarm/weather.hpp"

namespace swarm::testing {

class SensedDrone : public IAgent {
 public:
  SensedDrone(std::uint32_t id, std::uint32_t count, std::uint64_t seed, double dt,
              const Scene& scene, const Weather& weather)
      : id_(id),
        count_(count),
        scene_(scene),
        weather_(weather),
        imu_(ImuParams::typical_mems(), dt, seed, id),
        lidar_(lidar_config(), seed, id),
        dt_(dt) {
    state_.pos = {10.0 * static_cast<double>(id) - 80.0, 0.0, 25.0};
    state_.vel = {1.0, 0.0, 0.0};
  }

  void step(AgentContext& ctx) override {
    for (const Message& m : ctx.inbox) {
      if (m.payload.size() == 24) {
        double x = 0.0;
        std::memcpy(&x, m.payload.data(), sizeof(x));
        neighbor_x_sum_ += x;
        ++received_;
      }
    }

    // A noisy random-walk command, with a push upward when close to the ground.
    DroneCommand cmd;
    cmd.accel = {3.0 * ctx.rng.normal(), 3.0 * ctx.rng.normal(), 1.0 * ctx.rng.normal()};
    cmd.yaw_rate = 0.5 * ctx.rng.normal();
    const double above_ground = state_.pos.z - scene_.ground_height(state_.pos.x, state_.pos.y);
    if (above_ground < 8.0) {
      cmd.accel.z += 6.0;
    }
    state_ = swarm::step(state_, cmd, params_, dt_);

    imu_last_ = imu_.sample(state_);

    if (ctx.tick % 5 == 0) {
      const Scan scan = lidar_.scan(state_.pos, state_.yaw, scene_, weather_);
      for (const LidarReturn& r : scan.returns) {
        if (r.valid) {
          ++valid_returns_;
          range_sum_ += r.range + r.intensity;
        }
        ++total_rays_;
      }
    }

    Message to_next;
    to_next.receiver = (id_ + 1U) % count_;
    to_next.payload.resize(24);
    std::memcpy(to_next.payload.data(), &state_.pos, sizeof(Vec3));
    ctx.outbox.push_back(to_next);

    Message to_random = to_next;
    to_random.receiver = ctx.rng.below(count_);
    ctx.outbox.push_back(to_random);
  }

  Vec3 position() const override { return state_.pos; }

  void hash_state(Fnv1a64& hash) const override {
    hash.update_f64(state_.pos.x);
    hash.update_f64(state_.pos.y);
    hash.update_f64(state_.pos.z);
    hash.update_f64(state_.vel.x);
    hash.update_f64(state_.vel.y);
    hash.update_f64(state_.vel.z);
    hash.update_f64(state_.yaw);
    hash.update_f64(imu_last_.accel.x);
    hash.update_f64(imu_last_.accel.y);
    hash.update_f64(imu_last_.accel.z);
    hash.update_f64(imu_last_.gyro_z);
    hash.update_u64(valid_returns_);
    hash.update_u64(total_rays_);
    hash.update_f64(range_sum_);
    hash.update_u64(received_);
    hash.update_f64(neighbor_x_sum_);
  }

  std::uint64_t valid_returns() const { return valid_returns_; }
  std::uint64_t total_rays() const { return total_rays_; }
  std::uint64_t received() const { return received_; }

 private:
  static LidarConfig lidar_config() {
    LidarConfig c;
    c.dropout_probability = 0.01;
    return c;
  }

  std::uint32_t id_;
  std::uint32_t count_;
  const Scene& scene_;
  Weather weather_;
  Imu imu_;
  Lidar lidar_;
  double dt_;
  DroneParams params_;
  DroneState state_;
  ImuSample imu_last_;
  std::uint64_t valid_returns_ = 0;
  std::uint64_t total_rays_ = 0;
  double range_sum_ = 0.0;
  std::uint64_t received_ = 0;
  double neighbor_x_sum_ = 0.0;
};

}  // namespace swarm::testing
