#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "swarm/agent.hpp"
#include "swarm/drone_agent.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"
#include "swarm/simulation.hpp"
#include "test_scale.hpp"

using swarm::DroneAgent;
using swarm::PredictorChoice;
using swarm::Scenario;
using swarm::Vec3;

namespace {

constexpr std::uint64_t kTicks = 600;
constexpr std::uint32_t kDrones = 6;

Scenario base(PredictorChoice predictor) {
  std::ifstream in(std::string(SWARM_SCENARIO_DIR) + "/formation_jam.yaml", std::ios::binary);
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const swarm::LoadResult r = swarm::load_scenario(text);
  EXPECT_TRUE(r.ok());
  Scenario s = r.scenario;
  s.drones = kDrones;
  s.duration_ticks = kTicks;
  s.predictor = predictor;
  return s;
}

// A channel that drops nothing and one that is as hostile as the scenario format allows.
void clean_channel(Scenario& s) {
  s.channel.loss_model = swarm::LossModel::kNone;
  s.channel.jammers.clear();
  s.channel.max_msgs_per_sender_per_tick = 0;
}

void hostile_channel(Scenario& s) {
  s.channel.loss_model = swarm::LossModel::kBurst;
  s.channel.loss = 0.5;
  s.channel.mean_burst = 4.0;
  s.channel.max_msgs_per_sender_per_tick = 2;
  // The scenario's own jam (ticks 100 to 400, every link) stays.
}

struct Pose {
  Vec3 pos;
  Vec3 vel;
};

// The true pose of every drone at the end of every tick.
std::vector<std::vector<Pose>> trajectory(const Scenario& s) {
  const swarm::ScenarioWorld world = swarm::build_world(s);
  const auto sim = world.make_simulation(2);
  std::vector<std::vector<Pose>> out;
  for (std::uint64_t tick = 0; tick < kTicks; ++tick) {
    sim->step();
    std::vector<Pose> poses;
    for (std::size_t i = 0; i < kDrones; ++i) {
      const auto& d = dynamic_cast<const DroneAgent&>(sim->agent(i));
      poses.push_back({d.truth().pos, d.truth().vel});
    }
    out.push_back(poses);
  }
  return out;
}

bool identical(const std::vector<std::vector<Pose>>& a, const std::vector<std::vector<Pose>>& b) {
  for (std::size_t t = 0; t < a.size(); ++t) {
    for (std::size_t i = 0; i < kDrones; ++i) {
      if (!(a[t][i].pos == b[t][i].pos) || !(a[t][i].vel == b[t][i].vel)) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

TEST(Oracle, TheScenarioSelectsEachOfTheFourPredictorKinds) {
  const char* names[] = {"kalman", "dead_reckoning", "hold_last", "oracle"};
  const PredictorChoice kinds[] = {PredictorChoice::kKalman, PredictorChoice::kDeadReckoning,
                                   PredictorChoice::kHoldLast, PredictorChoice::kOracle};
  for (int k = 0; k < 4; ++k) {
    const std::string text =
        std::string("version: 1\nseed: 1\nduration_s: 2\nswarm: {drones: 3}\n") +
        "mission: {waypoints: [[0, 0, 20]]}\ncontrol: {predictor: " + names[k] + "}\n";
    const swarm::LoadResult r = swarm::load_scenario(text);
    ASSERT_TRUE(r.ok()) << names[k];
    EXPECT_EQ(r.scenario.predictor, kinds[k]);
    EXPECT_NE(swarm::canonical_text(r.scenario).find(std::string("predictor: ") + names[k]),
              std::string::npos);
    const swarm::ScenarioWorld w = swarm::build_world(r.scenario);
    for (const auto& a : w.agents) {
      EXPECT_EQ(a.oracle, kinds[k] == PredictorChoice::kOracle);
    }
  }
  const swarm::LoadResult bad = swarm::load_scenario(
      "version: 1\nseed: 1\nduration_s: 2\nswarm: {drones: 3}\n"
      "mission: {waypoints: [[0, 0, 20]]}\ncontrol: {predictor: psychic}\n");
  ASSERT_FALSE(bad.ok());
  EXPECT_EQ(bad.errors[0].path, "control.predictor");
}

TEST(Oracle, UnderTheOracleTrueTrajectoriesDoNotDependOnTheChannelAtAll) {
  Scenario clean = base(PredictorChoice::kOracle);
  clean_channel(clean);
  Scenario hostile = base(PredictorChoice::kOracle);
  hostile_channel(hostile);
  EXPECT_TRUE(identical(trajectory(clean), trajectory(hostile)));
}

TEST(Oracle, WithoutTheOracleTheSameTwoChannelsGiveDifferentTrajectories) {
  // The control for the test above: the comparison is able to see a difference.
  Scenario clean = base(PredictorChoice::kKalman);
  clean_channel(clean);
  Scenario hostile = base(PredictorChoice::kKalman);
  hostile_channel(hostile);
  EXPECT_FALSE(identical(trajectory(clean), trajectory(hostile)));
}

TEST(Oracle, ABrainIsToldTheTruthOfThePreviousTickWithNoUncertaintyAndNoAge) {
  Scenario s = base(PredictorChoice::kOracle);
  hostile_channel(s);
  const swarm::ScenarioWorld world = swarm::build_world(s);
  const auto sim = world.make_simulation(1);

  std::vector<Pose> previous;  // truth at the end of the previous tick; at tick 0, the start
  for (std::size_t i = 0; i < kDrones; ++i) {
    previous.push_back({world.agents[i].start, world.agents[i].start_velocity});
  }
  for (std::uint64_t tick = 0; tick < 300; ++tick) {
    sim->step();
    for (std::size_t i = 0; i < kDrones; ++i) {
      const auto& d = dynamic_cast<const DroneAgent&>(sim->agent(i));
      for (std::size_t j = 0; j < kDrones; ++j) {
        if (j == i) {
          continue;
        }
        const swarm::NeighborEstimate& e = d.brain().neighbor(static_cast<std::uint32_t>(j));
        ASSERT_TRUE(e.valid);
        ASSERT_EQ(e.position, previous[j].pos) << "tick " << tick << " " << i << "->" << j;
        ASSERT_EQ(e.velocity, previous[j].vel);
        ASSERT_EQ(e.position_sigma, 0.0);
        ASSERT_EQ(e.age, 0U);
      }
      ASSERT_EQ(d.brain().fresh_neighbors(), kDrones - 1);
    }
    for (std::size_t i = 0; i < kDrones; ++i) {
      const auto& d = dynamic_cast<const DroneAgent&>(sim->agent(i));
      previous[i] = {d.truth().pos, d.truth().vel};
    }
  }
}

TEST(Oracle, TheOracleNeverLeavesNominalBecauseOfTheChannel) {
  Scenario s = base(PredictorChoice::kOracle);
  hostile_channel(s);
  const swarm::ScenarioWorld world = swarm::build_world(s);
  const auto sim = world.make_simulation(1);
  for (std::uint64_t tick = 0; tick < kTicks; ++tick) {
    sim->step();
    for (std::size_t i = 0; i < kDrones; ++i) {
      ASSERT_EQ(dynamic_cast<const DroneAgent&>(sim->agent(i)).brain().state(),
                swarm::FallbackState::kNominal)
          << "tick " << tick;
    }
  }
}

namespace {

struct DropoutErrors {
  double oracle = 0.0;
  double hold = 0.0;
  double dead = 0.0;
  double kalman = 0.0;
};

// Mean distance, over the last second of a 3 s jam of every link and over every ordered pair,
// between a drone's estimate of a neighbor and the neighbor's truth at the end of the previous
// tick. `jam_start` is the first jammed tick: early (during assembly) or late (steady flight).
double mean_estimate_error(PredictorChoice predictor, std::uint64_t jam_start) {
  Scenario s = base(predictor);
  s.channel.loss_model = swarm::LossModel::kNone;
  s.channel.max_msgs_per_sender_per_tick = 0;
  s.channel.jammers[0].start_tick = jam_start;
  s.channel.jammers[0].end_tick = jam_start + 150;
  s.duration_ticks = jam_start + 150;
  const swarm::ScenarioWorld world = swarm::build_world(s);
  const auto sim = world.make_simulation(1);
  std::vector<Vec3> previous;
  for (std::size_t i = 0; i < kDrones; ++i) {
    previous.push_back(world.agents[i].start);
  }
  double sum = 0.0;
  std::size_t count = 0;
  for (std::uint64_t tick = 0; tick < jam_start + 150; ++tick) {
    sim->step();
    if (tick >= jam_start + 100) {
      for (std::size_t i = 0; i < kDrones; ++i) {
        const auto& d = dynamic_cast<const DroneAgent&>(sim->agent(i));
        for (std::size_t j = 0; j < kDrones; ++j) {
          if (j != i) {
            sum += swarm::distance(d.brain().neighbor(static_cast<std::uint32_t>(j)).position,
                                   previous[j]);
            ++count;
          }
        }
      }
    }
    for (std::size_t i = 0; i < kDrones; ++i) {
      previous[i] = dynamic_cast<const DroneAgent&>(sim->agent(i)).truth().pos;
    }
  }
  return sum / static_cast<double>(count);
}

DropoutErrors dropout(std::uint64_t jam_start) {
  DropoutErrors e;
  e.oracle = mean_estimate_error(PredictorChoice::kOracle, jam_start);
  e.hold = mean_estimate_error(PredictorChoice::kHoldLast, jam_start);
  e.dead = mean_estimate_error(PredictorChoice::kDeadReckoning, jam_start);
  e.kalman = mean_estimate_error(PredictorChoice::kKalman, jam_start);
  return e;
}

}  // namespace

TEST(Oracle, DuringADropoutInSteadyFlightTheBaselinesDifferAndTheOracleIsExact) {
  const DropoutErrors e = dropout(static_cast<std::uint64_t>(swarm::testing::scaled(1500, 1000)));
  RecordProperty("steady_oracle_error_m", std::to_string(e.oracle));
  RecordProperty("steady_hold_last_error_m", std::to_string(e.hold));
  RecordProperty("steady_dead_reckoning_error_m", std::to_string(e.dead));
  RecordProperty("steady_kalman_error_m", std::to_string(e.kalman));
  EXPECT_EQ(e.oracle, 0.0);
  EXPECT_GT(e.dead, 0.0);
  EXPECT_GT(e.kalman, 0.0);
  EXPECT_GT(e.hold, 3.0 * e.dead);  // holding the last state is far worse than extrapolating it
  EXPECT_GT(e.hold, 3.0 * e.kalman);
  EXPECT_NE(e.dead, e.kalman);
}

TEST(Oracle, DuringADropoutWhileTheFormationAssemblesTheBaselinesStillDiffer) {
  // A 3 s jam at 2 s, while the drones are converging fast and slowing down. The ordering of the
  // predictors is not the same as in steady flight, and the numbers are recorded, not asserted.
  const DropoutErrors e = dropout(100);
  RecordProperty("assembly_oracle_error_m", std::to_string(e.oracle));
  RecordProperty("assembly_hold_last_error_m", std::to_string(e.hold));
  RecordProperty("assembly_dead_reckoning_error_m", std::to_string(e.dead));
  RecordProperty("assembly_kalman_error_m", std::to_string(e.kalman));
  EXPECT_EQ(e.oracle, 0.0);
  EXPECT_GT(e.hold, 0.0);
  EXPECT_NE(e.hold, e.dead);
  EXPECT_NE(e.dead, e.kalman);
  EXPECT_NE(e.hold, e.kalman);
}

namespace {

// An agent that moves along x by 10 m per tick and records what it was told about the others.
class Recorder : public swarm::IAgent {
 public:
  Recorder(double start, double velocity) : pos_(start), vel_(velocity) {}
  void step(swarm::AgentContext& ctx) override {
    seen.push_back(ctx.peers);
    pos_ += vel_;
  }
  Vec3 position() const override { return Vec3{pos_, 0, 0}; }
  Vec3 velocity() const override { return Vec3{vel_, 0, 0}; }
  void hash_state(swarm::Fnv1a64& hash) const override { hash.update_f64(pos_); }
  std::vector<std::vector<swarm::PeerTruth>> seen;

 private:
  double pos_;
  double vel_;
};

}  // namespace

TEST(Oracle, TheSimulationPublishesThePreviousTicksTruthToEveryAgent) {
  std::vector<std::unique_ptr<swarm::IAgent>> agents;
  agents.push_back(std::make_unique<Recorder>(0.0, 10.0));
  agents.push_back(std::make_unique<Recorder>(100.0, -5.0));
  swarm::SimConfig config;
  swarm::Simulation sim(config, std::move(agents), 2);
  for (int i = 0; i < 4; ++i) {
    sim.step();
  }
  for (std::size_t a = 0; a < 2; ++a) {
    const auto& r = dynamic_cast<const Recorder&>(sim.agent(a));
    ASSERT_EQ(r.seen.size(), 4U);
    for (std::size_t tick = 0; tick < 4; ++tick) {
      ASSERT_EQ(r.seen[tick].size(), 2U);
      // At tick t the agents have stepped t times: agent 0 is at 10 t, agent 1 at 100 - 5 t.
      EXPECT_EQ(r.seen[tick][0].position.x, 10.0 * static_cast<double>(tick));
      EXPECT_EQ(r.seen[tick][1].position.x, 100.0 - 5.0 * static_cast<double>(tick));
      EXPECT_EQ(r.seen[tick][0].velocity.x, 10.0);
      EXPECT_EQ(r.seen[tick][1].velocity.x, -5.0);
    }
  }
}

TEST(Oracle, AnAgentThatIsNotTheOracleIsUnaffectedByWhatIsPublished) {
  const Scenario s = base(PredictorChoice::kKalman);
  const swarm::ScenarioWorld world = swarm::build_world(s);
  const std::vector<swarm::PeerTruth> truthful(kDrones,
                                               swarm::PeerTruth{Vec3{1, 2, 3}, Vec3{4, 5, 6}});
  const std::vector<swarm::PeerTruth> nonsense(
      kDrones, swarm::PeerTruth{Vec3{-9e5, 7e4, 3}, Vec3{99, 88, 77}});

  DroneAgent a(world.agents[2], *world.scene);
  DroneAgent b(world.agents[2], *world.scene);
  swarm::Rng rng_a(1, 0);
  swarm::Rng rng_b(1, 0);
  for (std::uint64_t tick = 0; tick < 300; ++tick) {
    std::vector<swarm::Message> out_a;
    std::vector<swarm::Message> out_b;
    swarm::AgentContext ca{tick, 0.0, 2, rng_a, {}, out_a, truthful};
    swarm::AgentContext cb{tick, 0.0, 2, rng_b, {}, out_b, nonsense};
    a.step(ca);
    b.step(cb);
    ASSERT_EQ(a.truth().pos, b.truth().pos) << "tick " << tick;
    ASSERT_EQ(out_a.size(), out_b.size());
  }
  swarm::Fnv1a64 ha;
  swarm::Fnv1a64 hb;
  a.hash_state(ha);
  b.hash_state(hb);
  EXPECT_EQ(ha.digest(), hb.digest());
}

TEST(Oracle, AnOracleAgentDoesUseWhatIsPublishedSoItIsTheOnlyOneThatCan) {
  Scenario s = base(PredictorChoice::kOracle);
  const swarm::ScenarioWorld world = swarm::build_world(s);
  const std::vector<swarm::PeerTruth> near_formation(kDrones,
                                                     swarm::PeerTruth{Vec3{1, 2, 3}, Vec3{}});
  const std::vector<swarm::PeerTruth> elsewhere(kDrones,
                                                swarm::PeerTruth{Vec3{500, 500, 50}, Vec3{}});
  DroneAgent a(world.agents[2], *world.scene);
  DroneAgent b(world.agents[2], *world.scene);
  swarm::Rng rng_a(1, 0);
  swarm::Rng rng_b(1, 0);
  std::vector<swarm::Message> out;
  swarm::AgentContext ca{0, 0.0, 2, rng_a, {}, out, near_formation};
  swarm::AgentContext cb{0, 0.0, 2, rng_b, {}, out, elsewhere};
  a.step(ca);
  b.step(cb);
  EXPECT_NE(a.brain().neighbor(0).position, b.brain().neighbor(0).position);
  EXPECT_EQ(a.brain().neighbor(0).position, (Vec3{1, 2, 3}));
}
