#pragma once

#include <cstdint>

#include "swarm/agent.hpp"
#include "swarm/control/drone_brain.hpp"
#include "swarm/dynamics.hpp"
#include "swarm/imu.hpp"
#include "swarm/lidar.hpp"
#include "swarm/position_fix.hpp"
#include "swarm/scene.hpp"
#include "swarm/weather.hpp"

namespace swarm {

struct DroneAgentConfig {
  // The brain's configuration. `launch` is overwritten from `start` and `start_velocity`, and the
  // pose estimator's noise constants are copied from `imu`, the sensor specification.
  BrainConfig brain;
  Vec3 start;
  Vec3 start_velocity;

  DroneParams vehicle;
  ImuParams imu = ImuParams::typical_mems();
  LidarConfig lidar;
  PositionFixConfig fix;
  Weather weather;
  std::uint64_t seed = 0;
  std::uint64_t fix_period_ticks = 5;  // a scan, and so a fix attempt, every this many ticks
};

// The vehicle half of a drone (ADR 0005). It owns the true state and the sensor models, runs the
// sensors on the truth, hands only their outputs to the brain, and applies the brain's command
// through the dynamics. The brain never sees the truth.
//
// Every tick: IMU sample and, on the fix period, a LiDAR scan and position fix; the delivered
// messages; one brain step; the dynamics step; and, when the brain asks, a broadcast of its report
// to every other drone.
class DroneAgent : public IAgent {
 public:
  // `scene` must outlive the agent. Throws std::invalid_argument for an invalid configuration.
  DroneAgent(const DroneAgentConfig& config, const IScene& scene);

  void step(AgentContext& ctx) override;
  Vec3 position() const override { return truth_.pos; }
  // Folds the true state, the IMU bias, and the brain's hash. The sensors' random generators are
  // not hashed directly: their draws reach the hash through the brain's estimates.
  void hash_state(Fnv1a64& hash) const override;

  // For metrics and tests, read from one thread after the barrier. Controllers never see these.
  const DroneState& truth() const { return truth_; }
  const DroneBrain& brain() const { return brain_; }

 private:
  static DroneAgentConfig prepared(DroneAgentConfig config);

  DroneAgentConfig config_;
  const IScene& scene_;
  Imu imu_;
  Lidar lidar_;
  PositionFixSensor fixer_;
  DroneBrain brain_;
  DroneState truth_;
};

}  // namespace swarm
