#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "swarm/estimation/neighbor_predictor.hpp"
#include "swarm/rng.hpp"

using swarm::INeighborPredictor;
using swarm::NeighborEstimate;
using swarm::NeighborPredictorConfig;
using swarm::NeighborReport;
using swarm::PredictorKind;
using swarm::Vec3;

namespace {

constexpr double kDt = 0.05;  // 20 Hz ticks
const Vec3 kStart{10.0, -5.0, 30.0};
const Vec3 kVelocity{3.0, -2.0, 0.5};

NeighborPredictorConfig config() {
  NeighborPredictorConfig c;
  c.dt = kDt;
  return c;
}

Vec3 true_position(std::uint64_t tick) {
  return kStart + kVelocity * (static_cast<double>(tick) * kDt);
}

struct Delivery {
  NeighborReport report;
  std::uint64_t receipt_tick;
};

// Reports sent every `period` ticks from tick 0 up to (not including) `until`, each delivered
// `latency` ticks later, with Gaussian noise on position and velocity.
std::vector<Delivery> make_reports(swarm::Rng& rng, std::uint64_t until, std::uint64_t period,
                                   std::uint64_t latency, double pos_sigma, double vel_sigma) {
  std::vector<Delivery> out;
  for (std::uint64_t t = 0; t < until; t += period) {
    NeighborReport r;
    r.send_tick = t;
    const double a = rng.normal();
    const double b = rng.normal();
    const double c = rng.normal();
    const double d = rng.normal();
    const double e = rng.normal();
    const double f = rng.normal();
    r.position = true_position(t) + Vec3{pos_sigma * a, pos_sigma * b, pos_sigma * c};
    r.velocity = kVelocity + Vec3{vel_sigma * d, vel_sigma * e, vel_sigma * f};
    r.position_sigma = pos_sigma;
    out.push_back({r, t + latency});
  }
  return out;
}

std::unique_ptr<INeighborPredictor> fed(PredictorKind kind, const NeighborPredictorConfig& cfg,
                                        const std::vector<Delivery>& deliveries) {
  auto p = swarm::make_neighbor_predictor(kind, cfg);
  for (const Delivery& d : deliveries) {
    p->on_report(d.report, d.receipt_tick);
  }
  return p;
}

double position_error(const NeighborEstimate& e, std::uint64_t tick) {
  return swarm::norm(e.position - true_position(tick));
}

struct MeanErrors {
  double hold_last = 0.0;
  double dead_reckoning = 0.0;
  double kalman = 0.0;
};

// Mean position error `gap` ticks after the last delivery, over many seeded trials.
MeanErrors dropout_errors(int trials, std::uint64_t gap, double pos_sigma, double vel_sigma,
                          const NeighborPredictorConfig& cfg) {
  MeanErrors sum;
  for (int trial = 0; trial < trials; ++trial) {
    swarm::Rng rng(1000 + static_cast<std::uint64_t>(trial), 0);
    const auto deliveries = make_reports(rng, 200, 10, 2, pos_sigma, vel_sigma);
    const std::uint64_t now = deliveries.back().receipt_tick + gap;
    sum.hold_last +=
        position_error(fed(PredictorKind::kHoldLast, cfg, deliveries)->predict(now), now);
    sum.dead_reckoning +=
        position_error(fed(PredictorKind::kDeadReckoning, cfg, deliveries)->predict(now), now);
    sum.kalman += position_error(fed(PredictorKind::kKalman, cfg, deliveries)->predict(now), now);
  }
  sum.hold_last /= trials;
  sum.dead_reckoning /= trials;
  sum.kalman /= trials;
  return sum;
}

}  // namespace

TEST(NeighborPredictors, EveryKindCanBeCreatedAndSatisfiesTheSameContract) {
  for (PredictorKind kind :
       {PredictorKind::kHoldLast, PredictorKind::kDeadReckoning, PredictorKind::kKalman}) {
    auto p = swarm::make_neighbor_predictor(kind, config());
    EXPECT_FALSE(p->predict(10).valid);  // nothing received yet

    NeighborReport r;
    r.send_tick = 4;
    r.position = {1.0, 2.0, 3.0};
    r.velocity = {0.0, 0.0, 0.0};
    r.position_sigma = 0.5;
    p->on_report(r, 6);
    const NeighborEstimate e = p->predict(16);
    EXPECT_TRUE(e.valid);
    EXPECT_EQ(e.age, 10U);
    EXPECT_NEAR(e.position.x, 1.0, 1e-9);  // stationary neighbor
    EXPECT_GT(e.position_sigma, 0.0);

    EXPECT_THROW(p->predict(5), std::invalid_argument);  // before the receipt
  }
}

TEST(NeighborPredictors, StaleAndDuplicateReportsAreIgnoredByEveryKind) {
  for (PredictorKind kind :
       {PredictorKind::kHoldLast, PredictorKind::kDeadReckoning, PredictorKind::kKalman}) {
    auto p = swarm::make_neighbor_predictor(kind, config());
    NeighborReport fresh;
    fresh.send_tick = 10;
    fresh.position = {5.0, 0.0, 0.0};
    fresh.position_sigma = 0.1;
    p->on_report(fresh, 12);
    const NeighborEstimate before = p->predict(14);

    NeighborReport stale = fresh;
    stale.send_tick = 8;
    stale.position = {99.0, 0.0, 0.0};
    p->on_report(stale, 13);
    NeighborReport duplicate = fresh;
    duplicate.position = {-99.0, 0.0, 0.0};
    p->on_report(duplicate, 13);

    const NeighborEstimate after = p->predict(14);
    EXPECT_EQ(after.position, before.position);
    EXPECT_EQ(after.age, before.age);  // neither resets the age
  }
}

TEST(NeighborPredictors, DecreasingReceiptTicksAreRejected) {
  for (PredictorKind kind :
       {PredictorKind::kHoldLast, PredictorKind::kDeadReckoning, PredictorKind::kKalman}) {
    auto p = swarm::make_neighbor_predictor(kind, config());
    NeighborReport r;
    r.send_tick = 1;
    r.position_sigma = 0.1;
    p->on_report(r, 20);
    r.send_tick = 2;
    EXPECT_THROW(p->on_report(r, 19), std::invalid_argument);
  }
}

TEST(NeighborPredictors, HoldLastNeverMovesAndDeadReckoningExtrapolatesFromTheSendTick) {
  NeighborReport r;
  r.send_tick = 100;
  r.position = {0.0, 0.0, 0.0};
  r.velocity = {2.0, 0.0, -1.0};
  r.position_sigma = 0.2;

  auto hold = swarm::make_neighbor_predictor(PredictorKind::kHoldLast, config());
  hold->on_report(r, 104);
  EXPECT_EQ(hold->predict(104).position, Vec3{});
  EXPECT_EQ(hold->predict(400).position, Vec3{});

  auto dead = swarm::make_neighbor_predictor(PredictorKind::kDeadReckoning, config());
  dead->on_report(r, 104);
  // 200 ticks after the send tick is 10 s at 20 Hz.
  const NeighborEstimate e = dead->predict(300);
  EXPECT_NEAR(e.position.x, 20.0, 1e-9);
  EXPECT_NEAR(e.position.z, -10.0, 1e-9);
  EXPECT_GT(e.position_sigma, 0.2);  // uncertainty grows with the extrapolation
}

TEST(NeighborPredictors,
     ExactReportsLetDeadReckoningAndKalmanTrackAfterADropoutWhileHoldLastFalls) {
  swarm::Rng rng(1, 0);
  const auto deliveries = make_reports(rng, 200, 10, 2, 1e-9, 1e-9);
  const std::uint64_t now = deliveries.back().receipt_tick + 60;  // 3 s with no reports

  const double hold =
      position_error(fed(PredictorKind::kHoldLast, config(), deliveries)->predict(now), now);
  const double dead =
      position_error(fed(PredictorKind::kDeadReckoning, config(), deliveries)->predict(now), now);
  const double kalman =
      position_error(fed(PredictorKind::kKalman, config(), deliveries)->predict(now), now);
  EXPECT_LT(dead, 0.01);
  EXPECT_LT(kalman, 0.01);
  // Hold-last lags by speed times the time since the report was made (62 ticks, 3.1 s).
  EXPECT_NEAR(hold, swarm::norm(kVelocity) * 62 * kDt, 1e-6);
}

TEST(NeighborPredictors, WithNoisyReportsTheKalmanPredictorBeatsDeadReckoningWhichBeatsHoldLast) {
  const MeanErrors m = dropout_errors(300, 60, 0.5, 0.3, config());
  // Reported here so a change in the ordering is visible in the test output.
  RecordProperty("hold_last_mean_error_m", std::to_string(m.hold_last));
  RecordProperty("dead_reckoning_mean_error_m", std::to_string(m.dead_reckoning));
  RecordProperty("kalman_mean_error_m", std::to_string(m.kalman));
  EXPECT_LT(m.kalman, m.dead_reckoning);
  EXPECT_LT(m.dead_reckoning, m.hold_last);
  // Measured at the default process noise: Kalman about 20% below dead reckoning, dead reckoning
  // about 85% below hold-last. The bounds leave room for those figures without being tight.
  EXPECT_LT(m.kalman, 0.9 * m.dead_reckoning);
  EXPECT_LT(m.dead_reckoning, 0.3 * m.hold_last);
}

TEST(NeighborPredictors, TheKalmanAdvantageDependsOnTheProcessNoiseMatchingTheNeighbor) {
  // The target flies perfectly straight, so a smaller process noise (trusting the constant
  // velocity model more) should help, and a larger one should hurt. The default of 1 m^2/s^3 is
  // chosen for a swarm that maneuvers, which is why it is only modestly better here.
  double previous = 0.0;
  for (double q : {0.01, 0.1, 1.0, 10.0}) {
    NeighborPredictorConfig cfg = config();
    cfg.process_noise_density = q;
    const MeanErrors m = dropout_errors(200, 60, 0.5, 0.3, cfg);
    RecordProperty("kalman_error_q_" + std::to_string(q), std::to_string(m.kalman));
    EXPECT_GT(m.kalman, previous) << "q " << q;  // error rises with q for a straight-flying target
    previous = m.kalman;
    if (q == 0.01) {
      EXPECT_LT(m.kalman, 0.6 * m.dead_reckoning);
    }
  }
}

TEST(NeighborPredictors, TheKalmanUncertaintyGrowsDuringADropoutAndCoversTheTrueError) {
  int covered = 0;
  const int trials = 300;
  for (int trial = 0; trial < trials; ++trial) {
    swarm::Rng rng(5000 + static_cast<std::uint64_t>(trial), 0);
    const auto deliveries = make_reports(rng, 200, 10, 2, 0.5, 0.3);
    auto p = fed(PredictorKind::kKalman, config(), deliveries);
    const std::uint64_t last = deliveries.back().receipt_tick;

    double previous = p->predict(last).position_sigma;
    for (std::uint64_t gap = 5; gap <= 60; gap += 5) {
      const double sigma = p->predict(last + gap).position_sigma;
      ASSERT_GT(sigma, previous) << "trial " << trial << " gap " << gap;
      previous = sigma;
    }

    const std::uint64_t now = last + 60;
    const NeighborEstimate e = p->predict(now);
    const Vec3 err = e.position - true_position(now);
    if (std::abs(err.x) <= 3.0 * e.position_sigma && std::abs(err.y) <= 3.0 * e.position_sigma &&
        std::abs(err.z) <= 3.0 * e.position_sigma) {
      ++covered;
    }
  }
  EXPECT_GE(static_cast<double>(covered) / trials, 0.95);
}

TEST(NeighborPredictors, DelayCompensationReducesTheKalmanErrorWhenReportsArriveLate) {
  NeighborPredictorConfig with = config();
  NeighborPredictorConfig without = config();
  without.compensate_delay = false;

  double error_with = 0.0;
  double error_without = 0.0;
  const int trials = 100;
  for (int trial = 0; trial < trials; ++trial) {
    swarm::Rng rng(9000 + static_cast<std::uint64_t>(trial), 0);
    const auto deliveries = make_reports(rng, 200, 10, 10, 0.3, 0.2);  // 0.5 s of delay
    const std::uint64_t now = deliveries.back().receipt_tick;          // just after a report
    error_with += position_error(fed(PredictorKind::kKalman, with, deliveries)->predict(now), now);
    error_without +=
        position_error(fed(PredictorKind::kKalman, without, deliveries)->predict(now), now);
  }
  // Without compensation the estimate lags by roughly speed times delay (about 1.9 m).
  EXPECT_LT(error_with / trials, 0.5 * (error_without / trials));
}

TEST(NeighborPredictors, ReportsThatArriveOutOfOrderDoNotCorruptTheKalmanState) {
  swarm::Rng rng(3, 0);
  const auto in_order = make_reports(rng, 100, 10, 2, 0.5, 0.3);

  // Same reports, but report 5 is delivered again late, after report 6 has been seen.
  std::vector<Delivery> with_straggler = in_order;
  Delivery late = in_order[4];
  late.receipt_tick = in_order[6].receipt_tick;
  with_straggler.insert(with_straggler.begin() + 7, late);

  const std::uint64_t now = in_order.back().receipt_tick + 30;
  const NeighborEstimate a = fed(PredictorKind::kKalman, config(), in_order)->predict(now);
  const NeighborEstimate b = fed(PredictorKind::kKalman, config(), with_straggler)->predict(now);
  EXPECT_EQ(a.position, b.position);
  EXPECT_EQ(a.velocity, b.velocity);
  EXPECT_EQ(a.position_sigma, b.position_sigma);
}

TEST(NeighborPredictors, AManeuveringNeighborIsHarderForEveryPredictor) {
  // The neighbor flies straight, then accelerates during the dropout. None of the predictors can
  // know that, so every error is larger than for the straight-flight case. Reported honestly: this
  // test does not claim an ordering, only that the error rises.
  const MeanErrors straight = dropout_errors(100, 60, 0.5, 0.3, config());

  MeanErrors maneuver;
  const int trials = 100;
  for (int trial = 0; trial < trials; ++trial) {
    swarm::Rng rng(1000 + static_cast<std::uint64_t>(trial), 0);
    const auto deliveries = make_reports(rng, 200, 10, 2, 0.5, 0.3);
    const std::uint64_t last = deliveries.back().receipt_tick;
    const std::uint64_t now = last + 60;
    // True position with a constant 2 m/s^2 acceleration along x starting at the last report.
    const double t_since = static_cast<double>(now - last) * kDt;
    const Vec3 truth = true_position(now) + Vec3{0.5 * 2.0 * t_since * t_since, 0.0, 0.0};
    const auto err = [&](PredictorKind kind) {
      return swarm::norm(fed(kind, config(), deliveries)->predict(now).position - truth);
    };
    maneuver.hold_last += err(PredictorKind::kHoldLast);
    maneuver.dead_reckoning += err(PredictorKind::kDeadReckoning);
    maneuver.kalman += err(PredictorKind::kKalman);
  }
  EXPECT_GT(maneuver.hold_last / trials, straight.hold_last);
  EXPECT_GT(maneuver.dead_reckoning / trials, straight.dead_reckoning);
  EXPECT_GT(maneuver.kalman / trials, straight.kalman);
}

TEST(NeighborPredictors, InvalidConfigurationIsRejected) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto rejects = [](NeighborPredictorConfig c) {
    EXPECT_THROW(swarm::make_neighbor_predictor(PredictorKind::kKalman, c), std::invalid_argument);
  };
  NeighborPredictorConfig c = config();
  c.dt = 0.0;
  rejects(c);
  c = config();
  c.dt = nan;
  rejects(c);
  c = config();
  c.report_velocity_sigma = -0.1;
  rejects(c);
  c = config();
  c.process_noise_density = nan;
  rejects(c);
}
