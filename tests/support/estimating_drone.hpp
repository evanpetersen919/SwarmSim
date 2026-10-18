#pragma once

// A test agent that runs the whole sensing and estimation stack on the lockstep loop: motion, IMU,
// LiDAR, position fixes, a pose estimator, neighbor reports, and all three neighbor predictors. Not
// part of the library: real drone agents arrive in a later phase. The motion here is scripted from
// ground truth, which is fine for a test; the estimators themselves never see it.

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "swarm/agent.hpp"
#include "swarm/dynamics.hpp"
#include "swarm/estimation/neighbor_predictor.hpp"
#include "swarm/estimation/neighbor_table.hpp"
#include "swarm/estimation/pose_estimator.hpp"
#include "swarm/imu.hpp"
#include "swarm/lidar.hpp"
#include "swarm/position_fix.hpp"
#include "swarm/scene.hpp"
#include "swarm/weather.hpp"

namespace swarm::testing {

constexpr std::uint64_t kFixPeriodTicks = 5;
constexpr std::uint64_t kReportPeriodTicks = 10;
constexpr std::uint64_t kCommandPeriodTicks = 50;

class EstimatingDrone : public IAgent {
 public:
  static constexpr std::size_t kPredictorKinds = 3;  // hold last, dead reckoning, Kalman

  EstimatingDrone(std::uint32_t id, std::uint32_t count, std::uint64_t seed, double dt,
                  const Scene& scene, const Weather& weather)
      : id_(id),
        count_(count),
        dt_(dt),
        scene_(scene),
        weather_(weather),
        imu_(ImuParams::typical_mems(), dt, seed, id),
        lidar_(lidar_config(), seed, id),
        fixer_(PositionFixConfig{}, seed, id),
        estimator_(estimator_config(dt), launch_for(id)),
        table_(count) {
    truth_.pos = launch_for(id).position;
    truth_.vel = launch_for(id).velocity;

    NeighborPredictorConfig cfg;
    cfg.dt = dt;
    predictors_.resize(count);
    for (std::uint32_t other = 0; other < count; ++other) {
      predictors_[other][0] = make_neighbor_predictor(PredictorKind::kHoldLast, cfg);
      predictors_[other][1] = make_neighbor_predictor(PredictorKind::kDeadReckoning, cfg);
      predictors_[other][2] = make_neighbor_predictor(PredictorKind::kKalman, cfg);
    }
  }

  void step(AgentContext& ctx) override {
    last_tick_ = ctx.tick;

    // Neighbor reports first: they were sent in earlier ticks.
    for (const Message& m : ctx.inbox) {
      if (table_.on_message(m.sender, m.payload.data(), m.payload.size(), ctx.tick) ==
          ReceiveResult::kAccepted) {
        const NeighborReport& report = table_.get(m.sender)->report;
        for (auto& predictor : predictors_[m.sender]) {
          predictor->on_report(report, ctx.tick);
        }
      }
    }

    // Scripted motion: a gentle random walk in the horizontal plane, holding altitude.
    if (ctx.tick % kCommandPeriodTicks == 0) {
      horizontal_ = {0.7 * ctx.rng.normal(), 0.7 * ctx.rng.normal(), 0.0};
    }
    DroneCommand cmd;
    cmd.accel = horizontal_;
    const double target_z = scene_.ground_height(truth_.pos.x, truth_.pos.y) + kAltitude;
    cmd.accel.z = 2.0 * (target_z - truth_.pos.z) - 1.5 * truth_.vel.z;
    truth_ = swarm::step(truth_, cmd, params_, dt_);

    // Sensing and estimation.
    estimator_.predict(imu_.sample(truth_));
    if (ctx.tick % kFixPeriodTicks == 0) {
      const Scan scan = lidar_.scan(truth_.pos, truth_.yaw, scene_, weather_);
      estimator_.update(fixer_.measure(truth_.pos, scan));
    }

    // Tell the neighbors what this drone believes, not what is true.
    if (ctx.tick % kReportPeriodTicks == 0) {
      const PoseEstimate e = estimator_.estimate();
      NeighborReport report;
      report.send_tick = ctx.tick;
      report.position = e.position;
      report.velocity = e.velocity;
      report.position_sigma =
          std::max(e.position_sigma.x, std::max(e.position_sigma.y, e.position_sigma.z));
      Message m;
      m.payload = encode_neighbor_report(report);
      for (std::uint32_t other = 0; other < count_; ++other) {
        if (other != id_) {
          m.receiver = other;
          ctx.outbox.push_back(m);
        }
      }
    }
  }

  Vec3 position() const override { return truth_.pos; }

  void hash_state(Fnv1a64& hash) const override {
    hash.update_f64(truth_.pos.x);
    hash.update_f64(truth_.pos.y);
    hash.update_f64(truth_.pos.z);
    hash.update_f64(truth_.vel.x);
    hash.update_f64(truth_.vel.y);
    hash.update_f64(truth_.vel.z);
    const PoseEstimate e = estimator_.estimate();
    hash.update_f64(e.position.x);
    hash.update_f64(e.position.y);
    hash.update_f64(e.position.z);
    hash.update_f64(e.velocity.x);
    hash.update_f64(e.accel_bias.x);
    hash.update_u64(estimator_.counters().fixes_applied);
    hash.update_u64(estimator_.counters().fixes_gated);
    hash.update_u64(estimator_.counters().fixes_unavailable);
    hash.update_u64(table_.counters().accepted);
    hash.update_u64(table_.counters().stale);
    for (std::uint32_t other = 0; other < count_; ++other) {
      const NeighborEstimate k = predictors_[other][2]->predict(last_tick_);
      hash.update_u64(k.valid ? 1U : 0U);
      hash.update_f64(k.position.x);
      hash.update_f64(k.position_sigma);
    }
  }

  // For the test harness, which reads them from a single thread after the barrier.
  const DroneState& truth() const { return truth_; }
  PoseEstimate estimate() const { return estimator_.estimate(); }
  const PoseEstimatorCounters& pose_counters() const { return estimator_.counters(); }
  const NeighborTableCounters& table_counters() const { return table_.counters(); }
  NeighborEstimate predicted(std::uint32_t neighbor, std::size_t kind, std::uint64_t tick) const {
    return predictors_[neighbor][kind]->predict(tick);
  }

 private:
  static constexpr double kAltitude = 15.0;

  static LidarConfig lidar_config() {
    LidarConfig c;
    c.beams = 8;
    c.azimuth_samples = 36;
    return c;
  }

  static PoseEstimatorConfig estimator_config(double dt) {
    const ImuParams imu = ImuParams::typical_mems();
    PoseEstimatorConfig c;
    c.dt = dt;
    c.accel_noise_density = imu.accel_noise_density;
    c.accel_bias_walk = imu.accel_bias_walk;
    return c;
  }

  // Eight start positions on a grid, with eight start velocities.
  static PoseLaunch launch_for(std::uint32_t id) {
    static constexpr double kVel[8][2] = {{3, 0},  {2, 2},   {0, 3},  {-2, 2},
                                          {-3, 0}, {-2, -2}, {0, -3}, {2, -2}};
    PoseLaunch l;
    l.position = {12.0 * static_cast<double>(id % 4) - 18.0,
                  12.0 * static_cast<double>(id / 4) - 6.0, kAltitude};
    l.velocity = {kVel[id % 8][0], kVel[id % 8][1], 0.0};
    return l;
  }

  std::uint32_t id_;
  std::uint32_t count_;
  double dt_;
  const Scene& scene_;
  Weather weather_;
  Imu imu_;
  Lidar lidar_;
  PositionFixSensor fixer_;
  PoseEstimator estimator_;
  NeighborTable table_;
  std::vector<std::array<std::unique_ptr<INeighborPredictor>, kPredictorKinds>> predictors_;
  DroneParams params_;
  DroneState truth_;
  Vec3 horizontal_;
  std::uint64_t last_tick_ = 0;
};

}  // namespace swarm::testing
