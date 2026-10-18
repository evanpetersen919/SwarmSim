#include <gtest/gtest.h>

#include <cmath>

#include "swarm/vec3.hpp"

using swarm::Vec3;

TEST(Vec3, ArithmeticOperators) {
  const Vec3 a{1, 2, 3};
  const Vec3 b{4, -5, 6};
  EXPECT_EQ(a + b, (Vec3{5, -3, 9}));
  EXPECT_EQ(a - b, (Vec3{-3, 7, -3}));
  EXPECT_EQ(-a, (Vec3{-1, -2, -3}));
  EXPECT_EQ(a * 2.0, (Vec3{2, 4, 6}));
  EXPECT_EQ(2.0 * a, (Vec3{2, 4, 6}));
  EXPECT_EQ(b / 2.0, (Vec3{2, -2.5, 3}));

  Vec3 c = a;
  c += b;
  EXPECT_EQ(c, a + b);
  c -= b;
  EXPECT_EQ(c, a);
  c *= 3.0;
  EXPECT_EQ(c, (Vec3{3, 6, 9}));
  EXPECT_NE(a, b);
}

TEST(Vec3, DefaultIsZero) { EXPECT_EQ(Vec3{}, (Vec3{0, 0, 0})); }

TEST(Vec3, DotProduct) {
  EXPECT_DOUBLE_EQ(dot(Vec3{1, 2, 3}, Vec3{4, -5, 6}), 12.0);
  EXPECT_DOUBLE_EQ(dot(Vec3{1, 0, 0}, Vec3{0, 1, 0}), 0.0);
}

TEST(Vec3, CrossProductIsRightHandedAndAntiCommutative) {
  const Vec3 x{1, 0, 0};
  const Vec3 y{0, 1, 0};
  const Vec3 z{0, 0, 1};
  EXPECT_EQ(cross(x, y), z);
  EXPECT_EQ(cross(y, z), x);
  EXPECT_EQ(cross(z, x), y);
  const Vec3 a{1, 2, 3};
  const Vec3 b{-4, 5, 0.5};
  EXPECT_EQ(cross(a, b), -cross(b, a));
  EXPECT_DOUBLE_EQ(dot(cross(a, b), a), 0.0);
  EXPECT_DOUBLE_EQ(dot(cross(a, b), b), 0.0);
}

TEST(Vec3, NormAndDistance) {
  EXPECT_DOUBLE_EQ(norm(Vec3{3, 4, 0}), 5.0);
  EXPECT_DOUBLE_EQ(norm_squared(Vec3{1, 2, 2}), 9.0);
  EXPECT_DOUBLE_EQ(distance(Vec3{1, 1, 1}, Vec3{4, 5, 1}), 5.0);
}

TEST(Vec3, NormalizedHasUnitLengthAndZeroStaysZero) {
  const Vec3 n = normalized(Vec3{0, 3, 4});
  EXPECT_DOUBLE_EQ(norm(n), 1.0);
  EXPECT_DOUBLE_EQ(n.y, 0.6);
  EXPECT_EQ(normalized(Vec3{}), Vec3{});
}

TEST(Vec3, ClampNormLeavesShortVectorsAlone) {
  const Vec3 v{1, 2, 2};  // length 3
  EXPECT_EQ(swarm::clamp_norm(v, 3.0), v);
  EXPECT_EQ(swarm::clamp_norm(v, 10.0), v);
}

TEST(Vec3, ClampNormShortensLongVectorsKeepingDirection) {
  const Vec3 v{3, 4, 12};  // length 13
  const Vec3 c = swarm::clamp_norm(v, 6.5);
  EXPECT_NEAR(norm(c), 6.5, 1e-12);
  EXPECT_NEAR(c.x, 1.5, 1e-12);
  EXPECT_NEAR(c.y, 2.0, 1e-12);
  EXPECT_NEAR(c.z, 6.0, 1e-12);
}

TEST(Vec3, ClampNormWithNonPositiveLimitGivesZero) {
  EXPECT_EQ(swarm::clamp_norm(Vec3{1, 2, 3}, 0.0), Vec3{});
  EXPECT_EQ(swarm::clamp_norm(Vec3{1, 2, 3}, -1.0), Vec3{});
  EXPECT_EQ(swarm::clamp_norm(Vec3{1, 2, 3}, std::nan("")), Vec3{});
}
