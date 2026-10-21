#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <optional>

#include "swarm/agent.hpp"
#include "swarm/camera_fix.hpp"
#include "swarm/control/drone_brain.hpp"
#include "swarm/dynamics.hpp"
#include "swarm/fixed_wing.hpp"
#include "swarm/imu.hpp"
#include "swarm/lidar.hpp"
#include "swarm/position_fix.hpp"
#include "swarm/scene.hpp"
#include "swarm/vehicle.hpp"
#include "swarm/weather.hpp"

namespace swarm {

struct DroneAgentConfig {
  // The brain's configuration. `launch` is overwritten from `start` and `start_velocity`, and the
  // pose estimator's noise constants are copied from `imu`, the sensor specification.
  BrainConfig brain;
  Vec3 start;
  Vec3 start_velocity;

  // Which vehicle this drone is. `vehicle` holds the limits of a quad and `fixed_wing` those of a
  // fixed-wing aircraft; the one that does not match `vehicle_kind` is not used.
  VehicleKind vehicle_kind = VehicleKind::kQuad;
  DroneParams vehicle;
  FixedWingParams fixed_wing;
  ImuParams imu = ImuParams::typical_mems();
  // A quad senses with the LiDAR and fixes its position from the scan (`lidar`, `fix`). A
  // fixed-wing aircraft has no LiDAR: it fixes its position with a downward camera (`camera`).
  LidarConfig lidar;
  PositionFixConfig fix;
  CameraFixConfig camera;
  Weather weather;
  std::uint64_t seed = 0;
  std::uint64_t fix_period_ticks = 5;  // a scan, and so a fix attempt, every this many ticks

  // The oracle baseline: instead of predicting its neighbors from their reports, the drone is told
  // their true position and velocity (from the previous tick), with no uncertainty. It ignores the
  // channel entirely. This is an evaluation baseline, an upper bound, and not a realistic drone.
  bool oracle = false;

  // From this tick on the drone has failed. The default never fails.
  std::uint64_t fail_tick = std::numeric_limits<std::uint64_t>::max();
};

// The vehicle half of a drone (ADR 0005). It owns the true state and the sensor models, runs the
// sensors on the truth, hands only their outputs to the brain, and applies the brain's command
// through the dynamics. The brain never sees the truth.
//
// Every tick: IMU sample and, on the fix period, a LiDAR scan and position fix; the delivered
// messages; one brain step; the dynamics step; and, when the brain asks, a broadcast of its report
// to every other drone.
//
// A failed drone is frozen and silent. From `fail_tick` on it does not step: it stops where it is
// with zero velocity and acceleration, samples nothing, ignores its inbox, and sends nothing. It
// stays in the world as a physical object, so its position still counts for jamming zones and for
// the separation metric. It does not fall or hover; this is a stated, simple failure model.
//
// A failed fixed-wing aircraft cannot hang in the air, so it is not frozen: from `fail_tick` on it
// flies on with no command, straight at its last horizontal speed and climb rate, still silent and
// still sensing nothing. It does not descend, turn, or stall; this is also a simple model.
class DroneAgent : public IAgent {
 public:
  // `scene` must outlive the agent. Throws std::invalid_argument for an invalid configuration.
  DroneAgent(const DroneAgentConfig& config, const IScene& scene);

  void step(AgentContext& ctx) override;
  Vec3 position() const override { return truth_.pos; }
  Vec3 velocity() const override { return truth_.vel; }
  // Folds the true state, the IMU bias, and the brain's hash. The sensors' random generators are
  // not hashed directly: their draws reach the hash through the brain's estimates.
  void hash_state(Fnv1a64& hash) const override;

  // For metrics and tests, read from one thread after the barrier. Controllers never see these.
  const DroneState& truth() const { return truth_; }
  const DroneBrain& brain() const { return brain_; }
  // False from the tick the drone fails; true before.
  bool alive() const { return !failed_; }

 private:
  static DroneAgentConfig prepared(DroneAgentConfig config);
  static std::unique_ptr<const IVehicle> make_vehicle(const DroneAgentConfig& config);

  DroneAgentConfig config_;
  const IScene& scene_;
  std::unique_ptr<const IVehicle> vehicle_;
  Imu imu_;
  std::optional<Lidar> lidar_;
  std::optional<PositionFixSensor> fixer_;
  std::optional<CameraFixSensor> camera_;
  DroneBrain brain_;
  DroneState truth_;
  bool failed_ = false;
};

}  // namespace swarm
