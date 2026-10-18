#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "swarm/control/formation.hpp"
#include "swarm/control/formation_control.hpp"
#include "swarm/dynamics.hpp"
#include "swarm/rng.hpp"

using swarm::FormationNeighbor;
using swarm::FormationParams;
using swarm::FormationTarget;
using swarm::MissionReference;
using swarm::NeighborEstimate;
using swarm::Vec3;

namespace {

void expect_near(const Vec3& a, const Vec3& b, double tol) {
  EXPECT_NEAR(a.x, b.x, tol);
  EXPECT_NEAR(a.y, b.y, tol);
  EXPECT_NEAR(a.z, b.z, tol);
}

std::vector<Vec3> offsets() {
  return swarm::make_formation(swarm::FormationShape::kWedge, 5, 10.0);
}

MissionReference reference_at(Vec3 position, Vec3 velocity = {}) {
  MissionReference r;
  r.position = position;
  r.velocity = velocity;
  return r;
}

// A neighbor in `slot` whose center estimate is exactly `center`.
FormationNeighbor neighbor(const std::vector<Vec3>& slots, std::uint32_t slot, Vec3 center,
                           double sigma, Vec3 velocity = {}) {
  FormationNeighbor n;
  n.slot = slot;
  n.estimate.valid = true;
  n.estimate.position = center + slots[slot];
  n.estimate.velocity = velocity;
  n.estimate.position_sigma = sigma;
  return n;
}

FormationTarget target_for(std::uint32_t self, const Vec3& own_center, double own_sigma,
                           const std::vector<FormationNeighbor>& neighbors,
                           const MissionReference& ref, const FormationParams& params) {
  const auto slots = offsets();
  return swarm::formation_target(self, own_center + slots[self], Vec3{}, own_sigma, neighbors,
                                 slots, ref, params);
}

}  // namespace

TEST(FormationControl, WithNoUsableNeighborsTheTargetIsTheReferencePlusTheOffset) {
  const auto slots = offsets();
  for (double alpha : {0.0, 0.3, 1.0}) {
    FormationParams p;
    p.alpha = alpha;
    const FormationTarget t =
        target_for(2, {5, 5, 5}, 0.2, {}, reference_at({1, 2, 3}, {4, 5, 6}), p);
    expect_near(t.position, Vec3{1, 2, 3} + slots[2], 1e-12);
    expect_near(t.velocity, {4, 5, 6}, 1e-12);
    EXPECT_EQ(t.neighbors_used, 0U);
  }
}

TEST(FormationControl, ADroneThatAlreadyAgreesWithItsNeighborsAndTheReferenceKeepsItsTarget) {
  const auto slots = offsets();
  const Vec3 center{10, -4, 20};
  std::vector<FormationNeighbor> neighbors;
  for (std::uint32_t slot : {0U, 1U, 3U, 4U}) {
    neighbors.push_back(neighbor(slots, slot, center, 0.3));
  }
  for (double alpha : {0.0, 0.5, 1.0}) {
    FormationParams p;
    p.alpha = alpha;
    const FormationTarget t = target_for(2, center, 0.3, neighbors, reference_at(center), p);
    expect_near(t.position, center + slots[2], 1e-12);
    EXPECT_EQ(t.neighbors_used, 4U);
  }
}

TEST(FormationControl, ANeighborWithAnOffCenterEstimatePullsTheTargetByItsWeight) {
  const auto slots = offsets();
  FormationParams p;
  p.alpha = 0.0;  // consensus only
  const Vec3 center{0, 0, 0};
  // The neighbor believes the center is 4 m further along x. With equal sigma the weights are equal
  // and the consensus center moves halfway.
  const auto n = neighbor(slots, 1, {4, 0, 0}, 0.5);
  const FormationTarget t = target_for(2, center, 0.5, {n}, reference_at(center), p);
  expect_near(t.position, Vec3{2, 0, 0} + slots[2], 1e-12);
}

TEST(FormationControl, AnUncertainNeighborCountsForLessInProportionToItsInverseVariance) {
  const auto slots = offsets();
  FormationParams p;
  p.alpha = 0.0;
  const double floor_sq = p.sigma_floor * p.sigma_floor;
  const auto n = neighbor(slots, 1, {4, 0, 0}, 10.0);  // very old prediction
  const FormationTarget t = target_for(2, {0, 0, 0}, 0.0, {n}, reference_at({}), p);

  const double w_own = 1.0 / floor_sq;
  const double w_neighbor = 1.0 / (100.0 + floor_sq);
  const double shift = 4.0 * w_neighbor / (w_own + w_neighbor);
  expect_near(t.position, Vec3{shift, 0, 0} + slots[2], 1e-12);
  EXPECT_LT(shift, 0.02);  // barely moved
}

TEST(FormationControl, AWeightFloorKeepsAPerfectEstimateFromTakingAllTheWeight) {
  const auto slots = offsets();
  FormationParams p;
  p.alpha = 0.0;
  // Both sigmas are zero; the floor makes the weights equal, so the center moves halfway.
  const auto n = neighbor(slots, 1, {6, 0, 0}, 0.0);
  const FormationTarget t = target_for(2, {0, 0, 0}, 0.0, {n}, reference_at({}), p);
  expect_near(t.position, Vec3{3, 0, 0} + slots[2], 1e-12);
}

TEST(FormationControl, UnusableNeighborsAreExcluded) {
  const auto slots = offsets();
  FormationParams p;
  p.alpha = 0.0;
  const double nan = std::numeric_limits<double>::quiet_NaN();

  auto invalid = neighbor(slots, 1, {50, 0, 0}, 0.5);
  invalid.estimate.valid = false;
  auto not_finite = neighbor(slots, 3, {50, 0, 0}, 0.5);
  not_finite.estimate.position.y = nan;
  auto negative_sigma = neighbor(slots, 4, {50, 0, 0}, -1.0);
  auto outside = neighbor(slots, 1, {50, 0, 0}, 0.5);
  outside.slot = 99;
  auto myself = neighbor(slots, 2, {50, 0, 0}, 0.5);

  const FormationTarget t =
      target_for(2, {0, 0, 0}, 0.5, {invalid, not_finite, negative_sigma, outside, myself},
                 reference_at({7, 7, 7}), p);
  EXPECT_EQ(t.neighbors_used, 0U);
  expect_near(t.position, Vec3{7, 7, 7} + slots[2], 1e-12);  // fell back to the reference
}

TEST(FormationControl, TheBlendWeightSelectsBetweenTheReferenceAndTheConsensus) {
  const auto slots = offsets();
  const auto n = neighbor(slots, 1, {10, 0, 0}, 0.5);  // consensus center = 5 (equal weights)
  const auto ref = reference_at({100, 0, 0});
  FormationParams p;

  p.alpha = 1.0;
  expect_near(target_for(2, {0, 0, 0}, 0.5, {n}, ref, p).position, Vec3{100, 0, 0} + slots[2],
              1e-12);
  p.alpha = 0.0;
  expect_near(target_for(2, {0, 0, 0}, 0.5, {n}, ref, p).position, Vec3{5, 0, 0} + slots[2], 1e-12);
  p.alpha = 0.25;
  expect_near(target_for(2, {0, 0, 0}, 0.5, {n}, ref, p).position,
              Vec3{0.25 * 100 + 0.75 * 5, 0, 0} + slots[2], 1e-12);
}

TEST(FormationControl, TheTargetVelocityIsBlendedLikeThePosition) {
  const auto slots = offsets();
  FormationParams p;
  p.alpha = 0.5;
  const auto n = neighbor(slots, 1, {0, 0, 0}, 0.5, {4, 0, 0});  // moving at 4 m/s
  const FormationTarget t = target_for(2, {0, 0, 0}, 0.5, {n}, reference_at({}, {2, 0, 0}), p);
  // consensus velocity = mean of own (0) and neighbor (4) = 2; blended with reference 2 -> 2.
  EXPECT_NEAR(t.velocity.x, 2.0, 1e-12);
}

TEST(FormationControl, InvalidInputsAreRejected) {
  const auto slots = offsets();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const MissionReference ref = reference_at({});
  EXPECT_THROW(swarm::formation_target(9, {}, {}, 0.1, {}, slots, ref, FormationParams{}),
               std::invalid_argument);
  EXPECT_THROW(swarm::formation_target(0, {nan, 0, 0}, {}, 0.1, {}, slots, ref, FormationParams{}),
               std::invalid_argument);
  EXPECT_THROW(swarm::formation_target(0, {}, {}, -1.0, {}, slots, ref, FormationParams{}),
               std::invalid_argument);

  const auto rejects = [&](FormationParams p) {
    EXPECT_THROW(swarm::formation_target(0, {}, {}, 0.1, {}, slots, ref, p), std::invalid_argument);
  };
  FormationParams p;
  p.alpha = 1.5;
  rejects(p);
  p = FormationParams{};
  p.alpha = nan;
  rejects(p);
  p = FormationParams{};
  p.kp = -1.0;
  rejects(p);
  p = FormationParams{};
  p.kd = nan;
  rejects(p);
  p = FormationParams{};
  p.sigma_floor = 0.0;
  rejects(p);
  p = FormationParams{};
  p.max_accel = 0.0;
  rejects(p);
}

TEST(FormationControl, TheControlLawIsZeroAtTheTargetAndPointsTowardIt) {
  FormationTarget t;
  t.position = {10, 0, 0};
  t.velocity = {2, 0, 0};
  const FormationParams p;
  expect_near(swarm::formation_accel(t, {10, 0, 0}, {2, 0, 0}, p), {0, 0, 0}, 0.0);

  const Vec3 a = swarm::formation_accel(t, {4, 0, 0}, {2, 0, 0}, p);  // 6 m short, right speed
  EXPECT_NEAR(a.x, p.kp * 6.0, 1e-12);
  EXPECT_NEAR(a.y, 0.0, 1e-12);

  const Vec3 slow = swarm::formation_accel(t, {10, 0, 0}, {0, 0, 0}, p);  // on target, too slow
  EXPECT_NEAR(slow.x, p.kd * 2.0, 1e-12);
}

TEST(FormationControl, TheCommandIsClampedToTheAirframeLimit) {
  FormationTarget t;
  t.position = {1000, 0, 0};
  FormationParams p;
  p.max_accel = 5.0;
  const Vec3 a = swarm::formation_accel(t, {0, 0, 0}, {0, 0, 0}, p);
  EXPECT_NEAR(swarm::norm(a), 5.0, 1e-12);
  EXPECT_GT(a.x, 0.0);
}

TEST(FormationControl, AStationaryTargetIsReachedWithoutOvershootOrOscillation) {
  // One point-mass drone with perfect information. A damping ratio of 0.9 overshoots by well under
  // one percent; the bound allows 2%.
  FormationParams p;
  p.max_accel = 1000.0;
  swarm::DroneParams vehicle;
  vehicle.max_accel = 1000.0;
  vehicle.max_speed = 1000.0;

  const Vec3 target_position{50, 0, 0};
  swarm::DroneState state;
  double furthest = 0.0;
  for (int i = 0; i < 2000; ++i) {  // 40 s at 50 Hz
    FormationTarget t;
    t.position = target_position;
    swarm::DroneCommand cmd;
    cmd.accel = swarm::formation_accel(t, state.pos, state.vel, p);
    state = swarm::step(state, cmd, vehicle, 0.02);
    furthest = std::max(furthest, state.pos.x);
  }
  EXPECT_NEAR(state.pos.x, 50.0, 1e-3);
  EXPECT_LT(furthest, 50.0 * 1.02);
}

TEST(FormationControl, ASwarmWithPerfectInformationConvergesToTheFormationAroundAMovingReference) {
  const std::uint32_t n = 6;
  const auto slots = swarm::make_formation(swarm::FormationShape::kGrid, n, 10.0);
  FormationParams p;
  p.alpha = 0.3;
  swarm::DroneParams vehicle;
  vehicle.max_accel = 8.0;
  vehicle.max_speed = 30.0;

  swarm::Rng rng(1, 0);
  std::vector<swarm::DroneState> drones(n);
  for (auto& d : drones) {
    d.pos = {rng.uniform(-60, 60), rng.uniform(-60, 60), rng.uniform(10, 30)};
  }

  const Vec3 ref_velocity{2.0, 1.0, 0.0};
  const double dt = 0.02;
  for (int step = 0; step < 4000; ++step) {  // 80 s
    const double time = step * dt;
    const MissionReference ref = reference_at(Vec3{0, 0, 20} + ref_velocity * time, ref_velocity);

    std::vector<swarm::DroneCommand> commands(n);
    for (std::uint32_t i = 0; i < n; ++i) {
      std::vector<FormationNeighbor> neighbors;
      for (std::uint32_t j = 0; j < n; ++j) {
        if (j != i) {
          FormationNeighbor nb;
          nb.slot = j;
          nb.estimate.valid = true;
          nb.estimate.position = drones[j].pos;
          nb.estimate.velocity = drones[j].vel;
          nb.estimate.position_sigma = 0.1;
          neighbors.push_back(nb);
        }
      }
      const FormationTarget t =
          swarm::formation_target(i, drones[i].pos, drones[i].vel, 0.1, neighbors, slots, ref, p);
      commands[i].accel = swarm::formation_accel(t, drones[i].pos, drones[i].vel, p);
    }
    for (std::uint32_t i = 0; i < n; ++i) {
      drones[i] = swarm::step(drones[i], commands[i], vehicle, dt);
    }
  }

  const double final_time = 4000 * dt;
  const Vec3 center = Vec3{0, 0, 20} + ref_velocity * final_time;
  for (std::uint32_t i = 0; i < n; ++i) {
    EXPECT_LT(swarm::distance(drones[i].pos, center + slots[i]), 0.1) << "drone " << i;
    EXPECT_LT(swarm::distance(drones[i].vel, ref_velocity), 0.05) << "drone " << i;
  }
}
