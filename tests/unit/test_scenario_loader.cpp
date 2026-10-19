#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include "swarm/scenario/scenario.hpp"

using swarm::load_scenario;
using swarm::LoadResult;
using swarm::Scenario;

namespace {

// The smallest valid file, with room to append more sections.
const char* kMinimal =
    "version: 1\n"
    "seed: 1\n"
    "duration_s: 20\n"
    "swarm:\n"
    "  drones: 4\n"
    "mission:\n"
    "  waypoints:\n"
    "    - [0, 0, 20]\n";

std::string read_file(const std::string& name) {
  std::ifstream in(std::string(SWARM_SCENARIO_DIR) + "/" + name, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

// True if some error is at `path` and its text contains `fragment`.
bool has_error(const LoadResult& r, const std::string& path, const std::string& fragment = "") {
  for (const auto& e : r.errors) {
    if (e.path == path && e.message.find(fragment) != std::string::npos) {
      return true;
    }
  }
  return false;
}

std::string describe(const LoadResult& r) {
  std::string out;
  for (const auto& e : r.errors) {
    out += swarm::to_string(e) + "\n";
  }
  return out;
}

// kMinimal with a replacement: `from` must occur in it.
std::string with(const std::string& from, const std::string& to) {
  std::string text = kMinimal;
  const std::size_t at = text.find(from);
  EXPECT_NE(at, std::string::npos) << from;
  text.replace(at, from.size(), to);
  return text;
}

}  // namespace

TEST(ScenarioLoader, AMinimalFileLoadsWithEveryDocumentedDefault) {
  const LoadResult r = load_scenario(kMinimal, "from_test");
  ASSERT_TRUE(r.ok()) << describe(r);
  const Scenario& s = r.scenario;
  EXPECT_EQ(s.name, "from_test");
  EXPECT_EQ(s.seed, 1U);
  EXPECT_DOUBLE_EQ(s.dt, 0.02);
  EXPECT_EQ(s.duration_ticks, 1000U);
  EXPECT_EQ(s.drones, 4U);
  EXPECT_EQ(s.terrain.type, swarm::TerrainType::kField);
  EXPECT_DOUBLE_EQ(s.terrain.size_x, 1000.0);
  EXPECT_DOUBLE_EQ(s.terrain.cell_size, 8.0);
  EXPECT_DOUBLE_EQ(s.obstacles.density_per_m2, 0.0);
  EXPECT_EQ(s.formation_shape, swarm::FormationShape::kWedge);
  EXPECT_DOUBLE_EQ(s.formation_spacing_m, 20.0);
  EXPECT_EQ(s.start.columns, 4U);
  ASSERT_EQ(s.mission.waypoints.size(), 1U);
  EXPECT_DOUBLE_EQ(s.mission.speed, 5.0);
  EXPECT_FALSE(s.mission.loop);
  EXPECT_DOUBLE_EQ(s.weather.visibility_m, 20000.0);
  EXPECT_EQ(s.lidar_beams, 8U);
  EXPECT_EQ(s.lidar_azimuth_samples, 36U);
  EXPECT_EQ(s.fix_period_ticks, 5U);
  EXPECT_EQ(s.channel.latency_ticks, 1U);
  EXPECT_EQ(s.channel.loss_model, swarm::LossModel::kNone);
  EXPECT_TRUE(s.channel.jammers.empty());
  EXPECT_EQ(s.predictor, swarm::PredictorChoice::kKalman);
  EXPECT_EQ(s.report_period_ticks, 5U);
  EXPECT_EQ(s.staleness_ticks, 50U);
  EXPECT_TRUE(s.faults.empty());
  EXPECT_DOUBLE_EQ(s.metrics.settle_s, 20.0);
  EXPECT_EQ(s.snapshot_every_ticks, 1U);
}

TEST(ScenarioLoader, ACompleteFileLoadsEveryField) {
  const LoadResult r = load_scenario(read_file("hills_fog_fault.yaml"));
  ASSERT_TRUE(r.ok()) << describe(r);
  const Scenario& s = r.scenario;
  EXPECT_EQ(s.name, "hills_fog_fault");
  EXPECT_EQ(s.seed, 21U);
  EXPECT_EQ(s.duration_ticks, 4500U);
  EXPECT_EQ(s.terrain.type, swarm::TerrainType::kHills);
  EXPECT_DOUBLE_EQ(s.terrain.size_x, 1200.0);
  EXPECT_DOUBLE_EQ(s.terrain.amplitude, 12.0);
  EXPECT_DOUBLE_EQ(s.obstacles.density_per_m2, 0.0002);
  EXPECT_EQ(s.drones, 6U);
  EXPECT_EQ(s.formation_shape, swarm::FormationShape::kGrid);
  EXPECT_DOUBLE_EQ(s.formation_spacing_m, 25.0);
  EXPECT_EQ(s.start.columns, 3U);
  ASSERT_EQ(s.mission.waypoints.size(), 3U);
  EXPECT_DOUBLE_EQ(s.mission.waypoints[2].y, 100.0);
  EXPECT_DOUBLE_EQ(s.weather.visibility_m, 400.0);
  EXPECT_DOUBLE_EQ(s.weather.rain_rate_mm_h, 4.0);
  EXPECT_EQ(s.channel.loss_model, swarm::LossModel::kIndependent);
  EXPECT_DOUBLE_EQ(s.channel.loss, 0.1);
  ASSERT_EQ(s.channel.jammers.size(), 1U);
  const swarm::Jammer& j = s.channel.jammers[0];
  EXPECT_EQ(j.mode, swarm::JamMode::kPulsed);
  EXPECT_EQ(j.start_tick, 1000U);
  EXPECT_EQ(j.end_tick, 3000U);
  EXPECT_EQ(j.pulse_period_ticks, 100U);
  EXPECT_DOUBLE_EQ(j.pulse_duty, 0.5);
  EXPECT_DOUBLE_EQ(j.loss_inside, 0.9);
  EXPECT_EQ(j.affects, swarm::JamAffects::kReceiver);
  EXPECT_EQ(s.predictor, swarm::PredictorChoice::kDeadReckoning);
  ASSERT_EQ(s.faults.size(), 1U);
  EXPECT_EQ(s.faults[0].drone, 2U);
  EXPECT_EQ(s.faults[0].tick, 2250U);
}

TEST(ScenarioLoader, TheShippedExamplesAllLoad) {
  for (const char* name :
       {"minimal.yaml", "formation_jam.yaml", "hills_fog_fault.yaml", "hold_resume.yaml"}) {
    const std::string text = read_file(name);
    ASSERT_FALSE(text.empty()) << name;
    const LoadResult r = load_scenario(text);
    EXPECT_TRUE(r.ok()) << name << "\n" << describe(r);
  }
}

TEST(ScenarioLoader, AnInfiniteJammerRadiusIsAccepted) {
  const LoadResult r = load_scenario(read_file("formation_jam.yaml"));
  ASSERT_TRUE(r.ok()) << describe(r);
  ASSERT_EQ(r.scenario.channel.jammers.size(), 1U);
  EXPECT_TRUE(std::isinf(r.scenario.channel.jammers[0].radius));
  EXPECT_EQ(r.scenario.channel.jammers[0].start_tick, 100U);
  EXPECT_EQ(r.scenario.channel.jammers[0].end_tick, 400U);
}

TEST(ScenarioLoader, UnknownKeysAreErrorsAtEveryLevel) {
  EXPECT_TRUE(
      has_error(load_scenario(std::string(kMinimal) + "colour: red\n"), "colour", "unknown key"));
  EXPECT_TRUE(has_error(load_scenario(std::string(kMinimal) + "comms:\n  lossy: 0.1\n"),
                        "comms.lossy", "unknown key"));
  EXPECT_TRUE(
      has_error(load_scenario(std::string(kMinimal) + "scene:\n  obstacles:\n    density: 0.1\n"),
                "scene.obstacles.density", "unknown key"));
  EXPECT_TRUE(has_error(load_scenario(with("drones: 4", "drones: 4\n  size: 3")), "swarm.size",
                        "unknown key"));
}

TEST(ScenarioLoader, EveryRequiredKeyIsRequired) {
  EXPECT_TRUE(has_error(load_scenario("seed: 1\nduration_s: 5\nswarm: {drones: 2}\nmission: "
                                      "{waypoints: [[0, 0, 0]]}\n"),
                        "version", "required"));
  EXPECT_TRUE(has_error(load_scenario("version: 1\nduration_s: 5\nswarm: {drones: 2}\nmission: "
                                      "{waypoints: [[0, 0, 0]]}\n"),
                        "seed", "required"));
  EXPECT_TRUE(has_error(load_scenario("version: 1\nseed: 1\nswarm: {drones: 2}\nmission: "
                                      "{waypoints: [[0, 0, 0]]}\n"),
                        "duration_s", "required"));
  EXPECT_TRUE(has_error(load_scenario("version: 1\nseed: 1\nduration_s: 5\nmission: "
                                      "{waypoints: [[0, 0, 0]]}\n"),
                        "swarm", "required"));
  EXPECT_TRUE(has_error(load_scenario("version: 1\nseed: 1\nduration_s: 5\nswarm: {}\nmission: "
                                      "{waypoints: [[0, 0, 0]]}\n"),
                        "swarm.drones", "required"));
  EXPECT_TRUE(has_error(load_scenario("version: 1\nseed: 1\nduration_s: 5\nswarm: {drones: 2}\n"),
                        "mission", "required"));
  EXPECT_TRUE(has_error(
      load_scenario("version: 1\nseed: 1\nduration_s: 5\nswarm: {drones: 2}\nmission: {}\n"),
      "mission.waypoints", "required"));
}

TEST(ScenarioLoader, AnUnsupportedVersionIsRejected) {
  const LoadResult r = load_scenario(with("version: 1", "version: 2"));
  EXPECT_TRUE(has_error(r, "version", "unsupported version 2"));
}

TEST(ScenarioLoader, WrongTypesAreErrors) {
  EXPECT_TRUE(has_error(load_scenario(with("seed: 1", "seed: abc")), "seed", "whole number"));
  EXPECT_TRUE(has_error(load_scenario(with("seed: 1", "seed: 1.5")), "seed", "whole number"));
  EXPECT_TRUE(has_error(load_scenario(with("seed: 1", "seed: -3")), "seed", "whole number"));
  EXPECT_TRUE(has_error(load_scenario(with("duration_s: 20", "duration_s: [20]")), "duration_s",
                        "single value"));
  EXPECT_TRUE(
      has_error(load_scenario(with("swarm:\n  drones: 4", "swarm: 4")), "swarm", "mapping"));
  EXPECT_TRUE(
      has_error(load_scenario(with("[0, 0, 20]", "[0, 0]")), "mission.waypoints", "[x, y, z]"));
  EXPECT_TRUE(
      has_error(load_scenario(with("[0, 0, 20]", "[0, 0, up]")), "mission.waypoints", "[x, y, z]"));
  EXPECT_TRUE(has_error(load_scenario(with("waypoints:", "waypoints: 5\n  ignored:")),
                        "mission.waypoints", "non-empty list"));
  EXPECT_TRUE(has_error(load_scenario(std::string(kMinimal) + "comms:\n  loss_model: sometimes\n"),
                        "comms.loss_model", "must be one of none, independent, burst"));
  EXPECT_TRUE(has_error(load_scenario(std::string(kMinimal) + "comms:\n  jammers: 3\n"),
                        "comms.jammers", "list"));
}

TEST(ScenarioLoader, RangesAreChecked) {
  const auto bad = [&](const std::string& extra, const std::string& path) {
    const LoadResult r = load_scenario(std::string(kMinimal) + extra);
    EXPECT_TRUE(has_error(r, path)) << extra << "\n" << describe(r);
  };
  bad("dt: 0\n", "dt");
  bad("dt: -0.01\n", "dt");
  bad("dt: 2\n", "dt");
  bad("comms:\n  loss: 1.5\n", "comms.loss");
  bad("comms:\n  loss: -0.1\n", "comms.loss");
  bad("comms:\n  latency_ticks: 0\n", "comms.latency_ticks");
  bad("comms:\n  mean_burst: 0.5\n", "comms.mean_burst");
  bad("weather:\n  visibility_m: 0\n", "weather.visibility_m");
  bad("weather:\n  rain_mm_h: -1\n", "weather.rain_mm_h");
  bad("scene:\n  cell_m: 0\n", "scene.cell_m");
  bad("scene:\n  size_m: [100, -5]\n", "scene.size_m");
  bad("sensors:\n  fix_period_ticks: 0\n", "sensors.fix_period_ticks");
  bad("sensors:\n  lidar:\n    beams: 0\n", "sensors.lidar.beams");
  bad("control:\n  staleness_ticks: 0\n", "control.staleness_ticks");
  bad("metrics:\n  collision_radius_m: 0\n", "metrics.collision_radius_m");
  bad("log:\n  snapshot_every_ticks: 0\n", "log.snapshot_every_ticks");

  EXPECT_TRUE(has_error(load_scenario(with("drones: 4", "drones: 0")), "swarm.drones"));
  EXPECT_TRUE(has_error(load_scenario(with("drones: 4", "drones: 4097")), "swarm.drones"));
  EXPECT_TRUE(has_error(load_scenario(with("duration_s: 20", "duration_s: 0")), "duration_s"));
  EXPECT_TRUE(has_error(load_scenario(with("waypoints:\n    - [0, 0, 20]",
                                           "speed_mps: 0\n  waypoints:\n    - [0, 0, 20]")),
                        "mission.speed_mps"));
  EXPECT_TRUE(has_error(load_scenario(with("waypoints:\n    - [0, 0, 20]", "waypoints: []")),
                        "mission.waypoints", "non-empty"));
}

TEST(ScenarioLoader, JammerWindowsAndModesAreChecked) {
  const auto jam = [&](const std::string& body) {
    return load_scenario(std::string(kMinimal) + "comms:\n  jammers:\n    - {" + body + "}\n");
  };
  EXPECT_TRUE(jam("radius_m: 50, from_s: 1, to_s: 2").ok());
  EXPECT_TRUE(
      has_error(jam("radius_m: 50, from_s: 5, to_s: 2"), "comms.jammers[0]", "not be before"));
  EXPECT_TRUE(has_error(jam("from_s: 1"), "comms.jammers[0].radius_m", "required"));
  EXPECT_TRUE(has_error(jam("radius_m: -5"), "comms.jammers[0].radius_m", "inf or a number"));
  EXPECT_TRUE(has_error(jam("radius_m: 5, duty: 0.5"), "comms.jammers[0].duty", "only valid"));
  EXPECT_TRUE(
      has_error(jam("radius_m: 5, mode: pulsed"), "comms.jammers[0].period_ticks", "required"));
  EXPECT_TRUE(has_error(jam("radius_m: 5, mode: pulsed, period_ticks: 10, duty: 0"),
                        "comms.jammers[0].duty"));
  EXPECT_TRUE(has_error(jam("radius_m: 5, loss: 2"), "comms.jammers[0].loss"));
  EXPECT_TRUE(
      has_error(jam("radius_m: 5, direction: up"), "comms.jammers[0].direction", "unknown key"));
}

TEST(ScenarioLoader, FaultsAreCheckedAgainstTheSwarm) {
  const auto with_faults = [&](const std::string& list) {
    return load_scenario(std::string(kMinimal) + "faults:\n" + list);
  };
  const LoadResult ok = with_faults("  - {drone: 1, at_s: 5}\n  - {drone: 3, at_s: 6.5}\n");
  ASSERT_TRUE(ok.ok()) << describe(ok);
  ASSERT_EQ(ok.scenario.faults.size(), 2U);
  EXPECT_EQ(ok.scenario.faults[1].tick, 325U);
  EXPECT_TRUE(has_error(with_faults("  - {drone: 4, at_s: 5}\n"), "faults[0].drone",
                        "below the number of drones"));
  EXPECT_TRUE(has_error(with_faults("  - {drone: 1, at_s: 5}\n  - {drone: 1, at_s: 9}\n"),
                        "faults[1].drone", "already fails"));
  EXPECT_TRUE(has_error(with_faults("  - {drone: 1}\n"), "faults[0].at_s", "required"));
}

TEST(ScenarioLoader, SeveralMistakesGiveSeveralErrorsInFileOrderWithLines) {
  const std::string text =
      "version: 1\n"      // 1
      "seed: x\n"         // 2
      "duration_s: 20\n"  // 3
      "swarm:\n"          // 4
      "  drones: 0\n"     // 5
      "mission:\n"        // 6
      "  waypoints: [[0, 0, 0]]\n"
      "comms:\n"       // 8
      "  loss: 3\n"    // 9
      "  lossy: 1\n";  // 10
  const LoadResult r = load_scenario(text);
  ASSERT_EQ(r.errors.size(), 4U) << describe(r);
  EXPECT_EQ(r.errors[0].path, "seed");
  EXPECT_EQ(r.errors[0].line, 2);
  EXPECT_EQ(r.errors[1].path, "swarm.drones");
  EXPECT_EQ(r.errors[1].line, 5);
  EXPECT_EQ(r.errors[2].path, "comms.loss");
  EXPECT_EQ(r.errors[2].line, 9);
  EXPECT_EQ(r.errors[3].path, "comms.lossy");
  EXPECT_EQ(r.errors[3].line, 10);
  EXPECT_EQ(swarm::to_string(r.errors[2]),
            "comms.loss (line 9): must be a number in [0, 1], got 3");
}

TEST(ScenarioLoader, ATimeThatIsNotAWholeNumberOfTicksIsRejected) {
  const std::string extra = "comms:\n  jammers:\n    - {radius_m: 5, from_s: 1.01, to_s: 3}\n";
  const LoadResult r = load_scenario(std::string(kMinimal) + extra);
  EXPECT_TRUE(has_error(r, "comms.jammers[0].from_s", "whole number of ticks"));

  // 1.02 s is 51 ticks at 0.02 s, up to rounding in the division.
  const LoadResult exact = load_scenario(std::string(kMinimal) +
                                         "comms:\n  jammers:\n    - {radius_m: 5, from_s: 1.02}\n");
  ASSERT_TRUE(exact.ok()) << describe(exact);
  EXPECT_EQ(exact.scenario.channel.jammers[0].start_tick, 51U);

  EXPECT_TRUE(has_error(load_scenario(with("duration_s: 20", "duration_s: 20.005")), "duration_s",
                        "whole number of ticks"));
  // dt = 0.1: 0.3 s is three ticks.
  const LoadResult coarse = load_scenario(with("duration_s: 20", "dt: 0.1\nduration_s: 0.3"));
  ASSERT_TRUE(coarse.ok()) << describe(coarse);
  EXPECT_EQ(coarse.scenario.duration_ticks, 3U);
}

TEST(ScenarioLoader, ScalarsAreParsedStrictlyByTheProject) {
  for (const char* bad : {"nan", ".inf", "inf", "0x10", "1e999", "1_000", "+-1", "1e", "--5"}) {
    const LoadResult r = load_scenario(with("duration_s: 20", std::string("duration_s: ") + bad));
    EXPECT_TRUE(has_error(r, "duration_s")) << bad;
  }
  // inf is accepted for a jammer radius and nowhere else.
  EXPECT_TRUE(
      load_scenario(std::string(kMinimal) + "comms:\n  jammers:\n    - {radius_m: inf}\n").ok());
  EXPECT_TRUE(has_error(load_scenario(std::string(kMinimal) + "weather:\n  visibility_m: inf\n"),
                        "weather.visibility_m"));
  // Booleans are true or false only.
  for (const char* bad : {"yes", "on", "1", "True"}) {
    const LoadResult r =
        load_scenario(with("[0, 0, 20]", "[0, 0, 20]\n  loop: " + std::string(bad)));
    EXPECT_TRUE(has_error(r, "mission.loop", "true or false")) << bad;
  }
  EXPECT_TRUE(load_scenario(with("[0, 0, 20]", "[0, 0, 20]\n  loop: true")).ok());
  // Scientific notation is fine.
  EXPECT_TRUE(load_scenario(with("duration_s: 20", "duration_s: 2e1")).ok());
}

TEST(ScenarioLoader, MalformedYamlIsAnErrorNotAnException) {
  for (const char* text :
       {"version: [1\n", "a: b: c\n", "\t- x\n", ": :\n", "- 1\n- 2\n", "", "just text"}) {
    LoadResult r;
    EXPECT_NO_THROW(r = load_scenario(text)) << text;
    EXPECT_FALSE(r.ok()) << text;
  }
  const LoadResult r = load_scenario("version: [1\n");
  ASSERT_FALSE(r.errors.empty());
  EXPECT_NE(r.errors[0].message.find("malformed YAML"), std::string::npos);
}

TEST(ScenarioLoader, ModelValidatorsRejectWhatTheFieldChecksMiss) {
  // A cell larger than the terrain passes every per-key range check and fails the terrain's own.
  EXPECT_TRUE(has_error(
      load_scenario(std::string(kMinimal) + "scene:\n  size_m: [100, 100]\n  cell_m: 500\n"),
      "scene", "cell size"));
  // A burst length too short for the requested mean loss.
  EXPECT_TRUE(
      has_error(load_scenario(std::string(kMinimal) +
                              "comms:\n  loss_model: burst\n  loss: 0.9\n  mean_burst: 1\n"),
                "comms"));
  // A mission waypoint count of zero cannot reach here, but a non-positive speed from the file can.
  EXPECT_FALSE(load_scenario(with("[0, 0, 20]", "[0, 0, 20]\n  speed_mps: 0")).ok());
}

TEST(ScenarioLoader, CanonicalTextReadsBackToTheSameScenario) {
  for (const char* name :
       {"minimal.yaml", "formation_jam.yaml", "hills_fog_fault.yaml", "hold_resume.yaml"}) {
    const LoadResult first = load_scenario(read_file(name));
    ASSERT_TRUE(first.ok()) << name << describe(first);
    const std::string text = swarm::canonical_text(first.scenario);
    const LoadResult second = load_scenario(text);
    ASSERT_TRUE(second.ok()) << name << "\n" << describe(second) << text;
    EXPECT_EQ(swarm::canonical_text(second.scenario), text) << name;
    EXPECT_EQ(second.scenario.name, first.scenario.name);
    EXPECT_EQ(second.scenario.duration_ticks, first.scenario.duration_ticks);
    EXPECT_EQ(second.scenario.channel.jammers.size(), first.scenario.channel.jammers.size());
  }
}

TEST(ScenarioLoader, CanonicalTextKeepsAwkwardNamesAndFullPrecision) {
  Scenario s = load_scenario(kMinimal).scenario;
  s.name = "a \"quoted\" name\\with\ttab";
  s.dt = 0.0123456789012345;
  s.duration_ticks = 12345;
  s.mission.speed = 0.1 + 0.2;  // not exactly 0.3
  const LoadResult back = load_scenario(swarm::canonical_text(s));
  ASSERT_TRUE(back.ok()) << describe(back);
  EXPECT_EQ(back.scenario.name, s.name);
  EXPECT_EQ(back.scenario.dt, s.dt);
  EXPECT_EQ(back.scenario.duration_ticks, s.duration_ticks);
  EXPECT_EQ(back.scenario.mission.speed, s.mission.speed);
}

TEST(ScenarioLoader, ACommandsListLoadsAndIsSortedByTime) {
  const LoadResult r = load_scenario(std::string(kMinimal) +
                                     "commands:\n"
                                     "  - {at_s: 12, kind: resume}\n"
                                     "  - {at_s: 5, kind: hold}\n"
                                     "  - {at_s: 12, kind: hold}\n");
  ASSERT_TRUE(r.ok()) << describe(r);
  const auto& c = r.scenario.commands;
  ASSERT_EQ(c.size(), 3U);
  EXPECT_EQ(c[0].tick, 250U);  // the hold at 5 s, moved first
  EXPECT_EQ(c[0].kind, swarm::CommandKind::kHold);
  EXPECT_EQ(c[1].tick, 600U);  // the two at 12 s keep the order they were written in
  EXPECT_EQ(c[1].kind, swarm::CommandKind::kResume);
  EXPECT_EQ(c[2].kind, swarm::CommandKind::kHold);
}

TEST(ScenarioLoader, WithoutACommandsListThereAreNoCommands) {
  EXPECT_TRUE(load_scenario(kMinimal).scenario.commands.empty());
  EXPECT_TRUE(load_scenario(std::string(kMinimal) + "commands: []\n").scenario.commands.empty());
}

TEST(ScenarioLoader, BadCommandsAreErrorsWithTheirPaths) {
  const auto commands = [&](const std::string& body) {
    return load_scenario(std::string(kMinimal) + "commands:\n" + body);
  };
  EXPECT_TRUE(has_error(commands("  - {at_s: 5, kind: land}\n"), "commands[0].kind",
                        "must be one of hold, resume"));
  EXPECT_TRUE(has_error(commands("  - {kind: hold}\n"), "commands[0].at_s", "required"));
  EXPECT_TRUE(has_error(commands("  - {at_s: 5}\n"), "commands[0].kind", "required"));
  EXPECT_TRUE(has_error(commands("  - {at_s: -1, kind: hold}\n"), "commands[0].at_s"));
  EXPECT_TRUE(has_error(commands("  - {at_s: 5.005, kind: hold}\n"), "commands[0].at_s",
                        "whole number of ticks"));
  EXPECT_TRUE(has_error(commands("  - {at_s: 5, kind: hold, who: all}\n"), "commands[0].who",
                        "unknown key"));
  EXPECT_TRUE(has_error(commands("  - 5\n"), "commands[0]", "mapping"));
  EXPECT_TRUE(
      has_error(load_scenario(std::string(kMinimal) + "commands: hold\n"), "commands", "list"));
  // Two bad entries give two errors.
  const LoadResult two = commands("  - {at_s: 5, kind: x}\n  - {at_s: 7, kind: y}\n");
  EXPECT_EQ(two.errors.size(), 2U);
}

TEST(ScenarioLoader, CommandsRoundTripThroughCanonicalText) {
  const LoadResult first = load_scenario(read_file("hold_resume.yaml"));
  ASSERT_TRUE(first.ok()) << describe(first);
  ASSERT_EQ(first.scenario.commands.size(), 2U);
  EXPECT_EQ(first.scenario.commands[0].tick, 1500U);
  EXPECT_EQ(first.scenario.commands[1].tick, 1800U);
  const std::string text = swarm::canonical_text(first.scenario);
  const LoadResult second = load_scenario(text);
  ASSERT_TRUE(second.ok()) << describe(second);
  EXPECT_EQ(swarm::canonical_text(second.scenario), text);
  ASSERT_EQ(second.scenario.commands.size(), 2U);
  EXPECT_EQ(second.scenario.commands[1].kind, swarm::CommandKind::kResume);
}
