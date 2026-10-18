#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <set>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "swarm/control/formation.hpp"
#include "swarm/drone_command.hpp"
#include "swarm/dynamics.hpp"

using swarm::FormationShape;
using swarm::Vec3;

namespace {

const FormationShape kShapes[] = {FormationShape::kLine, FormationShape::kWedge,
                                  FormationShape::kGrid};
const std::size_t kCounts[] = {0, 1, 2, 3, 4, 5, 7, 8, 9, 16, 20, 50};

double closest_pair(const std::vector<Vec3>& slots) {
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < slots.size(); ++i) {
    for (std::size_t j = i + 1; j < slots.size(); ++j) {
      best = std::min(best, swarm::distance(slots[i], slots[j]));
    }
  }
  return best;
}

}  // namespace

TEST(Formation, EveryShapeGivesTheRequestedNumberOfSlotsCenteredOnTheOrigin) {
  for (FormationShape shape : kShapes) {
    for (std::size_t n : kCounts) {
      const auto slots = swarm::make_formation(shape, n, 12.0);
      ASSERT_EQ(slots.size(), n);
      Vec3 mean;
      for (const Vec3& s : slots) {
        mean += s;
        EXPECT_EQ(s.z, 0.0);
      }
      if (n > 0) {
        mean = mean / static_cast<double>(n);
        EXPECT_LT(swarm::norm(mean), 1e-12) << "n " << n;
      }
    }
  }
}

TEST(Formation, TheClosestPairOfSlotsIsExactlyTheSpacingApart) {
  for (FormationShape shape : kShapes) {
    for (std::size_t n : kCounts) {
      if (n < 2) {
        continue;
      }
      const auto slots = swarm::make_formation(shape, n, 12.0);
      EXPECT_NEAR(closest_pair(slots), 12.0, 1e-9)
          << "shape " << static_cast<int>(shape) << " n " << n;
    }
  }
}

TEST(Formation, SlotsAreDistinctAndDeterministic) {
  for (FormationShape shape : kShapes) {
    const auto a = swarm::make_formation(shape, 30, 7.5);
    const auto b = swarm::make_formation(shape, 30, 7.5);
    ASSERT_EQ(a.size(), b.size());
    std::set<std::tuple<double, double>> unique;
    for (std::size_t i = 0; i < a.size(); ++i) {
      EXPECT_EQ(a[i], b[i]);
      unique.insert({a[i].x, a[i].y});
    }
    EXPECT_EQ(unique.size(), a.size());
  }
}

TEST(Formation, SpacingScalesTheWholeShape) {
  for (FormationShape shape : kShapes) {
    const auto small = swarm::make_formation(shape, 9, 1.0);
    const auto large = swarm::make_formation(shape, 9, 5.0);
    for (std::size_t i = 0; i < small.size(); ++i) {
      EXPECT_NEAR(large[i].x, 5.0 * small[i].x, 1e-12);
      EXPECT_NEAR(large[i].y, 5.0 * small[i].y, 1e-12);
    }
  }
}

TEST(Formation, ALineRunsAlongYInSlotOrder) {
  const auto slots = swarm::make_formation(FormationShape::kLine, 5, 10.0);
  for (std::size_t k = 0; k < slots.size(); ++k) {
    EXPECT_NEAR(slots[k].x, 0.0, 1e-12);
    EXPECT_NEAR(slots[k].y, 10.0 * (static_cast<double>(k) - 2.0), 1e-12);
  }
}

TEST(Formation, AWedgeHasItsApexInFrontAndSymmetricArms) {
  const auto slots = swarm::make_formation(FormationShape::kWedge, 7, 10.0);
  // Slot 0 is the apex: the only slot with the largest x.
  for (std::size_t k = 1; k < slots.size(); ++k) {
    EXPECT_LT(slots[k].x, slots[0].x);
  }
  // Slots 1 and 2, 3 and 4, 5 and 6 are mirror images across the wedge's axis.
  for (std::size_t k = 1; k + 1 < slots.size(); k += 2) {
    EXPECT_NEAR(slots[k].x, slots[k + 1].x, 1e-12);
    EXPECT_NEAR(slots[k].y, -slots[k + 1].y, 1e-12);
  }
  // Each row is farther back than the one before it.
  EXPECT_LT(slots[3].x, slots[1].x);
  EXPECT_LT(slots[5].x, slots[3].x);
}

TEST(Formation, AGridUsesTheSmallestSquareishColumnCount) {
  // 7 slots need 3 columns (2 would not be enough: 2 * 2 < 7), so 3 distinct y values.
  const auto slots = swarm::make_formation(FormationShape::kGrid, 7, 10.0);
  std::set<long long> ys;
  std::set<long long> xs;
  for (const Vec3& s : slots) {
    ys.insert(std::llround(s.y * 1000.0));
    xs.insert(std::llround(s.x * 1000.0));
  }
  EXPECT_EQ(ys.size(), 3U);
  EXPECT_EQ(xs.size(), 3U);  // 3 rows: 3 + 3 + 1

  const auto four = swarm::make_formation(FormationShape::kGrid, 4, 10.0);
  std::set<long long> four_ys;
  for (const Vec3& s : four) {
    four_ys.insert(std::llround(s.y * 1000.0));
  }
  EXPECT_EQ(four_ys.size(), 2U);  // exactly 2 x 2
}

TEST(Formation, InvalidArgumentsAreRejected) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (FormationShape shape : kShapes) {
    EXPECT_THROW(swarm::make_formation(shape, 4, 0.0), std::invalid_argument);
    EXPECT_THROW(swarm::make_formation(shape, 4, -1.0), std::invalid_argument);
    EXPECT_THROW(swarm::make_formation(shape, 4, nan), std::invalid_argument);
    EXPECT_THROW(swarm::make_formation(shape, swarm::kMaxFormationSlots + 1, 1.0),
                 std::invalid_argument);
    EXPECT_NO_THROW(swarm::make_formation(shape, swarm::kMaxFormationSlots, 1.0));
  }
}

TEST(DroneCommand, IsAvailableFromItsOwnHeaderAndThroughDynamics) {
  // The command type moved out of dynamics.hpp so controllers can use it without including the
  // truth-side header. Both include paths must still give the same type.
  swarm::DroneCommand c;
  c.accel = {1.0, 2.0, 3.0};
  c.yaw_rate = 0.5;
  swarm::DroneState state;
  const swarm::DroneState next = swarm::step(state, c, swarm::DroneParams{}, 0.01);
  EXPECT_GT(next.vel.x, 0.0);
}
