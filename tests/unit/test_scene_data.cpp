#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "swarm/format/scene.hpp"

using swarm::Obstacle;
using swarm::ObstacleKind;
using swarm::SceneData;
using swarm::SceneStatus;

namespace {

using Bytes = std::vector<std::uint8_t>;

SceneData tiny_scene() {
  SceneData s;
  s.terrain.cells_x = 1;
  s.terrain.cells_y = 1;
  s.terrain.origin_x = -5.0;
  s.terrain.origin_y = -5.0;
  s.terrain.cell_size = 10.0;
  s.terrain.heights = {0.0, 1.0, 2.0, 3.0};

  Obstacle box;
  box.kind = ObstacleKind::kBox;
  box.cx = 1.0;
  box.cy = 2.0;
  box.base_z = 0.0;
  box.height = 3.0;
  box.half_x = 0.5;
  box.half_y = 0.25;

  Obstacle cylinder;
  cylinder.kind = ObstacleKind::kCylinder;
  cylinder.cx = -1.0;
  cylinder.cy = -2.0;
  cylinder.base_z = 0.5;
  cylinder.height = 4.0;
  cylinder.half_x = 1.5;
  cylinder.half_y = 0.0;

  s.obstacles = {box, cylinder};
  return s;
}

void u32(Bytes& b, std::uint32_t v) {
  for (int i = 0; i < 4; ++i) {
    b.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFFU));
  }
}

// Appends an IEEE 754 binary64 given by its bit pattern, little-endian.
void f64_bits(Bytes& b, std::uint64_t bits) {
  for (int i = 0; i < 8; ++i) {
    b.push_back(static_cast<std::uint8_t>((bits >> (8 * i)) & 0xFFU));
  }
}

// Hand-written expected bytes for tiny_scene(). Changing the format means changing this on
// purpose and bumping kSceneVersion.
Bytes expected_tiny_bytes() {
  Bytes b = {'S', 'W', 'S', 'C'};
  u32(b, 1);                        // version
  u32(b, 1);                        // cells_x
  u32(b, 1);                        // cells_y
  f64_bits(b, 0xC014000000000000);  // origin_x -5.0
  f64_bits(b, 0xC014000000000000);  // origin_y -5.0
  f64_bits(b, 0x4024000000000000);  // cell_size 10.0
  u32(b, 2);                        // obstacle count
  f64_bits(b, 0x0000000000000000);  // 0.0
  f64_bits(b, 0x3FF0000000000000);  // 1.0
  f64_bits(b, 0x4000000000000000);  // 2.0
  f64_bits(b, 0x4008000000000000);  // 3.0
  // box
  b.push_back(0);
  f64_bits(b, 0x3FF0000000000000);  // cx 1.0
  f64_bits(b, 0x4000000000000000);  // cy 2.0
  f64_bits(b, 0x0000000000000000);  // base_z 0.0
  f64_bits(b, 0x4008000000000000);  // height 3.0
  f64_bits(b, 0x3FE0000000000000);  // half_x 0.5
  f64_bits(b, 0x3FD0000000000000);  // half_y 0.25
  // cylinder
  b.push_back(1);
  f64_bits(b, 0xBFF0000000000000);  // cx -1.0
  f64_bits(b, 0xC000000000000000);  // cy -2.0
  f64_bits(b, 0x3FE0000000000000);  // base_z 0.5
  f64_bits(b, 0x4010000000000000);  // height 4.0
  f64_bits(b, 0x3FF8000000000000);  // radius 1.5
  f64_bits(b, 0x0000000000000000);  // half_y 0.0
  return b;
}

Bytes serialize(const SceneData& s) {
  Bytes bytes;
  EXPECT_EQ(swarm::serialize_scene(s, bytes), SceneStatus::kOk);
  return bytes;
}

SceneStatus parse(const Bytes& b, SceneData* out = nullptr) {
  SceneData scratch;
  return swarm::parse_scene(b.data(), b.size(), out != nullptr ? *out : scratch);
}

constexpr std::size_t kHeightsAt = 4 + 4 + 4 + 4 + 8 + 8 + 8 + 4;
constexpr std::size_t kObstaclesAt = kHeightsAt + 4 * 8;

}  // namespace

TEST(SceneData, SerializesToTheDocumentedBytes) {
  EXPECT_EQ(serialize(tiny_scene()), expected_tiny_bytes());
}

TEST(SceneData, ParsesTheDocumentedBytes) {
  SceneData out;
  ASSERT_EQ(parse(expected_tiny_bytes(), &out), SceneStatus::kOk);
  const SceneData expected = tiny_scene();
  EXPECT_EQ(out.terrain.cells_x, 1U);
  EXPECT_EQ(out.terrain.heights, expected.terrain.heights);
  ASSERT_EQ(out.obstacles.size(), 2U);
  EXPECT_EQ(out.obstacles[1].kind, ObstacleKind::kCylinder);
  EXPECT_DOUBLE_EQ(out.obstacles[1].half_x, 1.5);
}

TEST(SceneData, RoundTripIsBitExact) {
  SceneData s = tiny_scene();
  s.terrain.cells_x = 7;
  s.terrain.cells_y = 5;
  s.terrain.heights.resize(8 * 6);
  for (std::size_t i = 0; i < s.terrain.heights.size(); ++i) {
    s.terrain.heights[i] = 0.1 * static_cast<double>(i) - 1.0 / 3.0;
  }
  SceneData back;
  ASSERT_EQ(parse(serialize(s), &back), SceneStatus::kOk);
  ASSERT_EQ(back.terrain.heights.size(), s.terrain.heights.size());
  EXPECT_EQ(0, std::memcmp(back.terrain.heights.data(), s.terrain.heights.data(),
                           s.terrain.heights.size() * sizeof(double)));
  EXPECT_EQ(swarm::scene_hash(back), swarm::scene_hash(s));
}

TEST(SceneData, SceneWithoutObstaclesRoundTrips) {
  SceneData s = tiny_scene();
  s.obstacles.clear();
  SceneData back;
  ASSERT_EQ(parse(serialize(s), &back), SceneStatus::kOk);
  EXPECT_TRUE(back.obstacles.empty());
}

TEST(SceneData, EveryProperPrefixIsTruncated) {
  const Bytes full = expected_tiny_bytes();
  for (std::size_t length = 0; length < full.size(); ++length) {
    const Bytes prefix(full.begin(), full.begin() + static_cast<std::ptrdiff_t>(length));
    EXPECT_EQ(parse(prefix), SceneStatus::kTruncated) << "length " << length;
  }
}

TEST(SceneData, BadMagicAndVersionAreRejected) {
  Bytes b = expected_tiny_bytes();
  b[0] = 'X';
  EXPECT_EQ(parse(b), SceneStatus::kBadMagic);
  b = expected_tiny_bytes();
  b[4] = 2;
  EXPECT_EQ(parse(b), SceneStatus::kUnsupportedVersion);
}

TEST(SceneData, TrailingBytesAreRejected) {
  Bytes b = expected_tiny_bytes();
  b.push_back(0);
  EXPECT_EQ(parse(b), SceneStatus::kCorrupt);
}

TEST(SceneData, OverLimitCountsAreRejectedBeforeAllocating) {
  Bytes b = expected_tiny_bytes();
  b[8 + 1] = 0x14;  // cells_x = 0x1401 = 5121 > 4096
  EXPECT_EQ(parse(b), SceneStatus::kLimitExceeded);

  b = expected_tiny_bytes();
  // obstacle_count field sits right before the heights; set it to 2,000,000.
  const std::uint32_t too_many = 2000000;
  for (int i = 0; i < 4; ++i) {
    b[kHeightsAt - 4 + static_cast<std::size_t>(i)] =
        static_cast<std::uint8_t>((too_many >> (8 * i)) & 0xFFU);
  }
  EXPECT_EQ(parse(b), SceneStatus::kLimitExceeded);
}

TEST(SceneData, InvalidContentsAreCorrupt) {
  Bytes b = expected_tiny_bytes();
  // NaN in the first height.
  const std::uint8_t nan_bytes[8] = {0, 0, 0, 0, 0, 0, 0xF8, 0x7F};
  std::memcpy(&b[kHeightsAt], nan_bytes, 8);
  EXPECT_EQ(parse(b), SceneStatus::kCorrupt);

  b = expected_tiny_bytes();
  b[kObstaclesAt] = 7;  // unknown obstacle kind
  EXPECT_EQ(parse(b), SceneStatus::kCorrupt);

  b = expected_tiny_bytes();
  b[8] = 0;  // cells_x = 0 (the heights array then no longer matches either)
  EXPECT_NE(parse(b), SceneStatus::kOk);
}

TEST(SceneData, ValidateAcceptsAGoodSceneAndExplainsRejections) {
  EXPECT_TRUE(swarm::validate(tiny_scene()));

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  std::string reason;

  SceneData s = tiny_scene();
  s.terrain.heights[2] = nan;
  EXPECT_FALSE(swarm::validate(s, &reason));
  EXPECT_NE(reason.find("height"), std::string::npos);

  s = tiny_scene();
  s.terrain.heights[0] = inf;
  EXPECT_FALSE(swarm::validate(s));

  s = tiny_scene();
  s.terrain.cell_size = 0.0;
  EXPECT_FALSE(swarm::validate(s, &reason));
  EXPECT_NE(reason.find("cell size"), std::string::npos);

  s = tiny_scene();
  s.terrain.cell_size = -1.0;
  EXPECT_FALSE(swarm::validate(s));

  s = tiny_scene();
  s.terrain.heights.pop_back();
  EXPECT_FALSE(swarm::validate(s, &reason));
  EXPECT_NE(reason.find("heights"), std::string::npos);

  s = tiny_scene();
  s.terrain.cells_x = 0;
  EXPECT_FALSE(swarm::validate(s));

  s = tiny_scene();
  s.terrain.origin_x = nan;
  EXPECT_FALSE(swarm::validate(s));

  s = tiny_scene();
  s.obstacles[0].height = 0.0;
  EXPECT_FALSE(swarm::validate(s));

  s = tiny_scene();
  s.obstacles[0].half_y = -0.1;
  EXPECT_FALSE(swarm::validate(s));

  s = tiny_scene();
  s.obstacles[1].half_x = 0.0;  // cylinder radius
  EXPECT_FALSE(swarm::validate(s));

  s = tiny_scene();
  s.obstacles[0].cx = inf;
  EXPECT_FALSE(swarm::validate(s));

  s = tiny_scene();
  s.obstacles[0].kind = static_cast<ObstacleKind>(9);
  EXPECT_FALSE(swarm::validate(s));
}

TEST(SceneData, SerializingAnInvalidSceneIsAnError) {
  SceneData s = tiny_scene();
  s.terrain.cell_size = 0.0;
  Bytes out{1, 2, 3};
  EXPECT_EQ(swarm::serialize_scene(s, out), SceneStatus::kInvalid);
  EXPECT_EQ(out, (Bytes{1, 2, 3}));  // untouched on failure
}

TEST(SceneData, HashIsStableForEqualScenesAndSensitiveToEveryField) {
  const SceneData base = tiny_scene();
  const std::uint64_t h = swarm::scene_hash(base);
  EXPECT_EQ(swarm::scene_hash(tiny_scene()), h);

  const auto differs = [&](const SceneData& changed) { return swarm::scene_hash(changed) != h; };

  for (std::size_t i = 0; i < base.terrain.heights.size(); ++i) {
    SceneData s = base;
    s.terrain.heights[i] += 1e-9;
    EXPECT_TRUE(differs(s)) << "height " << i;
  }
  {
    SceneData s = base;
    s.terrain.origin_x += 1.0;
    EXPECT_TRUE(differs(s));
    s = base;
    s.terrain.origin_y += 1.0;
    EXPECT_TRUE(differs(s));
    s = base;
    s.terrain.cell_size += 1.0;
    EXPECT_TRUE(differs(s));
    s = base;
    s.terrain.cells_x = 2;
    s.terrain.heights.resize(6);
    EXPECT_TRUE(differs(s));
  }
  for (std::size_t i = 0; i < base.obstacles.size(); ++i) {
    SceneData s = base;
    s.obstacles[i].cx += 1.0;
    EXPECT_TRUE(differs(s));
    s = base;
    s.obstacles[i].cy += 1.0;
    EXPECT_TRUE(differs(s));
    s = base;
    s.obstacles[i].base_z += 1.0;
    EXPECT_TRUE(differs(s));
    s = base;
    s.obstacles[i].height += 1.0;
    EXPECT_TRUE(differs(s));
    s = base;
    s.obstacles[i].half_x += 1.0;
    EXPECT_TRUE(differs(s));
    s = base;
    s.obstacles[i].half_y += 1.0;
    EXPECT_TRUE(differs(s));
  }
  {
    SceneData s = base;
    s.obstacles[0].kind = ObstacleKind::kCylinder;
    EXPECT_TRUE(differs(s));
    s = base;
    s.obstacles.pop_back();
    EXPECT_TRUE(differs(s));
    s = base;
    std::swap(s.obstacles[0], s.obstacles[1]);
    EXPECT_TRUE(differs(s));
  }
}

TEST(SceneData, StatusNamesAreDistinct) {
  EXPECT_STREQ(swarm::to_string(SceneStatus::kTruncated), "truncated");
  EXPECT_STRNE(swarm::to_string(SceneStatus::kBadMagic), swarm::to_string(SceneStatus::kCorrupt));
}

TEST(SceneData, HeightfieldIndexingHelpers) {
  swarm::Heightfield t;
  t.cells_x = 3;
  t.cells_y = 2;
  t.cell_size = 2.5;
  EXPECT_EQ(t.samples_x(), 4U);
  EXPECT_EQ(t.samples_y(), 3U);
  EXPECT_EQ(t.index(0, 0), 0U);
  EXPECT_EQ(t.index(3, 0), 3U);
  EXPECT_EQ(t.index(0, 1), 4U);
  EXPECT_EQ(t.index(3, 2), 11U);
  EXPECT_DOUBLE_EQ(t.extent_x(), 7.5);
  EXPECT_DOUBLE_EQ(t.extent_y(), 5.0);
}
