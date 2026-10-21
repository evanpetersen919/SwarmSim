#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <stdexcept>

#include "swarm/estimation/neighbor_predictor.hpp"
#include "swarm/rng.hpp"

using swarm::INeighborPredictor;
using swarm::NeighborEstimate;
using swarm::NeighborPredictorConfig;
using swarm::NeighborReport;
using swarm::PredictorKind;
using swarm::Vec3;

namespace {

constexpr double kDt = 0.02;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadius = 150.0;
constexpr double kSpeed = 17.0;
constexpr double kOmega = kSpeed / kRadius;

NeighborPredictorConfig config() {
  NeighborPredictorConfig c;
  c.dt = kDt;
  return c;
}

// An aircraft on a circle around the origin, counterclockwise, starting at angle `phase`.
struct Circle {
  double phase = 0.0;
  double z = 100.0;
  Vec3 position(double t) const {
    const double a = phase + kOmega * t;
    return {kRadius * std::cos(a), kRadius * std::sin(a), z};
  }
  Vec3 velocity(double t) const {
    const double a = phase + kOmega * t;
    return {-kSpeed * std::sin(a), kSpeed * std::cos(a), 0.0};
  }
};

NeighborReport report_at(const Circle& c, std::uint64_t tick, const Vec3& pos_noise = {},
                         const Vec3& vel_noise = {}) {
  NeighborReport r;
  r.send_tick = tick;
  r.position = c.position(static_cast<double>(tick) * kDt) + pos_noise;
  r.velocity = c.velocity(static_cast<double>(tick) * kDt) + vel_noise;
  r.position_sigma = 1.0;
  return r;
}

// Reports every 5 ticks (0.1 s) for `seconds`, delivered 2 ticks late. Returns the last delivery.
std::uint64_t feed(INeighborPredictor& p, const Circle& c, double seconds,
                   swarm::Rng* rng = nullptr, double noise = 0.0) {
  std::uint64_t last = 0;
  for (std::uint64_t tick = 5; static_cast<double>(tick) * kDt <= seconds; tick += 5) {
    Vec3 pn;
    Vec3 vn;
    if (rng != nullptr) {
      pn = {noise * rng->normal(), noise * rng->normal(), 0.0};
      vn = {0.3 * rng->normal(), 0.3 * rng->normal(), 0.0};
    }
    p.on_report(report_at(c, tick, pn, vn), tick + 2);
    last = tick + 2;
  }
  return last;
}

double error_after(INeighborPredictor& p, const Circle& c, std::uint64_t last_delivery,
                   double age_s) {
  const std::uint64_t now = last_delivery + static_cast<std::uint64_t>(age_s / kDt);
  const NeighborEstimate e = p.predict(now);
  EXPECT_TRUE(e.valid);
  return swarm::distance(e.position, c.position(static_cast<double>(now) * kDt));
}

}  // namespace

TEST(TurningPredictor, ANeighborOnACircleIsPredictedAlongItForTenSeconds) {
  const Circle circle{0.4, 100.0};
  const auto turning = swarm::make_neighbor_predictor(PredictorKind::kTurning, config());
  const auto straight = swarm::make_neighbor_predictor(PredictorKind::kDeadReckoning, config());
  const auto hold = swarm::make_neighbor_predictor(PredictorKind::kHoldLast, config());
  std::uint64_t last = 0;
  for (auto* p : {turning.get(), straight.get(), hold.get()}) {
    last = feed(*p, circle, 4.0);
  }
  for (const double age : {1.0, 5.0, 10.0}) {
    const double t_err = error_after(*turning, circle, last, age);
    const double d_err = error_after(*straight, circle, last, age);
    const double h_err = error_after(*hold, circle, last, age);
    // With exact reports the turn rate is exact, so the circle is followed to well under a meter.
    EXPECT_LT(t_err, 0.5) << age;
    EXPECT_GT(d_err, 10.0 * t_err) << age;
    EXPECT_GT(h_err, d_err) << age;
  }
}

TEST(TurningPredictor, StraightFlightIsPredictedLikeDeadReckoning) {
  const auto turning = swarm::make_neighbor_predictor(PredictorKind::kTurning, config());
  const auto straight = swarm::make_neighbor_predictor(PredictorKind::kDeadReckoning, config());
  for (std::uint64_t tick = 5; tick <= 200; tick += 5) {
    NeighborReport r;
    r.send_tick = tick;
    r.position = {10.0 + 17.0 * tick * kDt, -5.0, 80.0};
    r.velocity = {17.0, 0.0, 0.5};
    r.position_sigma = 1.0;
    turning->on_report(r, tick + 1);
    straight->on_report(r, tick + 1);
  }
  const NeighborEstimate a = turning->predict(1000);
  const NeighborEstimate b = straight->predict(1000);
  EXPECT_NEAR(a.position.x, b.position.x, 1e-9);
  EXPECT_NEAR(a.position.y, b.position.y, 1e-9);
  EXPECT_NEAR(a.position.z, b.position.z, 1e-9);
  EXPECT_NEAR(a.velocity.x, b.velocity.x, 1e-9);
}

TEST(TurningPredictor, NoisyReportsStillBeatStraightLinePredictionOnACircle) {
  const Circle circle{2.0, 100.0};
  swarm::Rng rng(11, 5);
  const auto turning = swarm::make_neighbor_predictor(PredictorKind::kTurning, config());
  const auto straight = swarm::make_neighbor_predictor(PredictorKind::kDeadReckoning, config());
  swarm::Rng rng_b(11, 5);  // the same noise for both predictors
  const std::uint64_t last = feed(*turning, circle, 4.0, &rng, 2.0);
  feed(*straight, circle, 4.0, &rng_b, 2.0);
  for (const double age : {5.0, 10.0}) {
    EXPECT_LT(error_after(*turning, circle, last, age), error_after(*straight, circle, last, age))
        << age;
  }
}

TEST(TurningPredictor, TheHeadingWrapsAcrossPlusAndMinusPi) {
  // Heading pi at the start of the reports: the turn is measured through the wrap.
  const Circle circle{kPi / 2.0 - 0.05, 100.0};  // heading = phase + pi/2, so about pi
  const auto turning = swarm::make_neighbor_predictor(PredictorKind::kTurning, config());
  const std::uint64_t last = feed(*turning, circle, 4.0);
  EXPECT_LT(error_after(*turning, circle, last, 8.0), 0.5);
}

TEST(TurningPredictor, UntilTheReportsSpanTheMinimumTheNeighborIsAssumedStraight) {
  const Circle circle{0.0, 100.0};
  const auto turning = swarm::make_neighbor_predictor(PredictorKind::kTurning, config());
  const auto straight = swarm::make_neighbor_predictor(PredictorKind::kDeadReckoning, config());
  const std::uint64_t last_t = feed(*turning, circle, 0.6);
  feed(*straight, circle, 0.6);
  const NeighborEstimate a = turning->predict(last_t + 200);
  const NeighborEstimate b = straight->predict(last_t + 200);
  EXPECT_NEAR(a.position.x, b.position.x, 1e-9);
  EXPECT_NEAR(a.position.y, b.position.y, 1e-9);
}

TEST(TurningPredictor, TheUncertaintyGrowsWithTheSquareOfThePredictionTime) {
  const Circle circle{0.0, 100.0};
  const auto turning = swarm::make_neighbor_predictor(PredictorKind::kTurning, config());
  const std::uint64_t last = feed(*turning, circle, 4.0);
  const double s1 = turning->predict(last + 50).position_sigma;
  const double s5 = turning->predict(last + 250).position_sigma;
  const double s10 = turning->predict(last + 500).position_sigma;
  EXPECT_LT(s1, s5);
  EXPECT_LT(s5, s10);
  // The bend term dominates after a few seconds: 0.5 * speed * sigma_omega * t^2.
  EXPECT_NEAR(s10, 0.5 * kSpeed * 0.03 * 100.0, 0.15 * 0.5 * kSpeed * 0.03 * 100.0);
}

TEST(TurningPredictor, TheTurnRateIsLimited) {
  // A neighbor that appears to spin very fast is limited to turn_max_rate.
  NeighborPredictorConfig c = config();
  c.turn_max_rate = 0.5;
  const auto turning = swarm::make_neighbor_predictor(PredictorKind::kTurning, c);
  for (std::uint64_t tick = 5; tick <= 200; tick += 5) {
    NeighborReport r;
    r.send_tick = tick;
    r.position = {0.0, 0.0, 0.0};
    const double heading = 3.0 * tick * kDt;  // 3 rad/s
    r.velocity = {17.0 * std::cos(heading), 17.0 * std::sin(heading), 0.0};
    r.position_sigma = 1.0;
    turning->on_report(r, tick);
  }
  const NeighborEstimate now = turning->predict(200);
  const NeighborEstimate later = turning->predict(300);  // 2 s on
  const double h0 = std::atan2(now.velocity.y, now.velocity.x);
  const double h1 = std::atan2(later.velocity.y, later.velocity.x);
  double turned = h1 - h0;
  while (turned > kPi) {
    turned -= 2.0 * kPi;
  }
  while (turned < -kPi) {
    turned += 2.0 * kPi;
  }
  EXPECT_NEAR(std::abs(turned), 0.5 * 2.0, 1e-9);
}

TEST(TurningPredictor, StaleReportsAreIgnoredAndReceiptsMustNotDecrease) {
  const auto turning = swarm::make_neighbor_predictor(PredictorKind::kTurning, config());
  EXPECT_FALSE(turning->predict(10).valid);
  const Circle circle{0.0, 100.0};
  turning->on_report(report_at(circle, 50), 52);
  turning->on_report(report_at(circle, 40), 53);  // older send tick: ignored
  const NeighborEstimate e = turning->predict(60);
  EXPECT_EQ(e.age, 8U);
  EXPECT_THROW(turning->on_report(report_at(circle, 60), 51), std::invalid_argument);
  EXPECT_THROW(turning->predict(40), std::invalid_argument);
}

TEST(TurningPredictor, InvalidConfigurationsAreRejected) {
  NeighborPredictorConfig c = config();
  c.turn_min_span_s = 5.0;  // above the window
  EXPECT_THROW(swarm::make_neighbor_predictor(PredictorKind::kTurning, c), std::invalid_argument);
  c = config();
  c.turn_max_rate = 0.0;
  EXPECT_THROW(swarm::make_neighbor_predictor(PredictorKind::kTurning, c), std::invalid_argument);
  c = config();
  c.turn_rate_sigma = -1.0;
  EXPECT_THROW(swarm::make_neighbor_predictor(PredictorKind::kTurning, c), std::invalid_argument);
}
