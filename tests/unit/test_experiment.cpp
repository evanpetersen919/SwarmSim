#include <gtest/gtest.h>

#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <vector>

#include "experiment_files.hpp"
#include "swarm/experiment/experiment.hpp"

using swarm::Expansion;
using swarm::Experiment;
using swarm::LoadedExperiment;

namespace {

const char* kBase = R"(version: 1
name: tiny
seed: 9
dt: 0.02
duration_s: 5
scene: {type: field, size_m: [400, 400], cell_m: 8}
swarm:
  drones: 4
  formation: {shape: wedge, spacing_m: 20}
  start: {columns: 2, spacing_m: 30, jitter_m: 4, altitude_m: 30, altitude_jitter_m: 2}
mission:
  waypoints:
    - [-50, 0, 30]
    - [50, 0, 30]
  speed_mps: 3
comms: {latency_ticks: 2, loss_model: independent, loss: 0.0}
)";

swarm::ReadFile files() {
  return [](const std::string& path, std::string& text) {
    if (path == "base.yaml") {
      text = kBase;
      return true;
    }
    return false;
  };
}

LoadedExperiment load(const std::string& text) { return swarm::load_experiment(text, files()); }

std::string error_text(const LoadedExperiment& l) {
  std::string out;
  for (const auto& e : l.errors) {
    out += swarm::to_string(e) + "\n";
  }
  return out;
}

// `part` is "path: message start", just a message, or text with a line number, matched against the
// whole report.
bool has_error(const LoadedExperiment& l, const std::string& part) {
  if (part.find(" (line ") != std::string::npos) {
    return error_text(l).find(part) != std::string::npos;
  }
  const std::size_t colon = part.find(": ");
  const bool has_path = colon != std::string::npos;
  const std::string path = has_path ? part.substr(0, colon) : "";
  const std::string message = has_path ? part.substr(colon + 2) : part;
  for (const auto& e : l.errors) {
    if ((!has_path || e.path == path) && e.message.find(message) != std::string::npos) {
      return true;
    }
  }
  return false;
}

const char* kSmall = R"(version: 1
name: small
base: base.yaml
seeds: {first: 10, count: 3}
axes:
  - name: loss
    levels:
      - {label: p00, set: {comms: {loss: 0.0}}}
      - {label: p30, set: {comms: {loss: 0.3}}}
  - name: size
    levels:
      - {label: n04, set: {}}
      - {label: n06, set: {swarm: {drones: 6, start: {columns: 3}}}}
      - {label: n08, set: {swarm: {drones: 8, start: {columns: 4}}}}
)";

const std::string kHead = "version: 1\nname: x\nbase: base.yaml\nseeds: {count: 1}\n";

}  // namespace

TEST(ExperimentLoad, AValidFileLoads) {
  const LoadedExperiment l = load(kSmall);
  ASSERT_TRUE(l.ok()) << error_text(l);
  EXPECT_EQ(l.experiment.name, "small");
  EXPECT_EQ(l.experiment.first_seed, 10U);
  EXPECT_EQ(l.experiment.seed_count, 3U);
  ASSERT_EQ(l.experiment.axes.size(), 2U);
  EXPECT_EQ(l.experiment.axes[1].levels.size(), 3U);
  EXPECT_EQ(l.experiment.axes[0].levels[1].label, "p30");
  EXPECT_EQ(swarm::cell_count(l.experiment), 6U);
}

TEST(ExperimentLoad, ErrorsNameThePathAndTheLine) {
  const LoadedExperiment l = load(
      "version: 1\nname: x\nbase: base.yaml\nseeds: {first: 1, count: 0}\naxes: []\nextra: 1\n");
  EXPECT_FALSE(l.ok());
  EXPECT_TRUE(has_error(l, "seeds.count (line 4): must be a whole number in [1, 10000], got 0"))
      << error_text(l);
  EXPECT_TRUE(has_error(l, "extra (line 6): unknown key")) << error_text(l);
}

TEST(ExperimentLoad, RequiredKeysAndTheVersionAreChecked) {
  EXPECT_TRUE(has_error(load("name: x\n"), "version: is required"));
  EXPECT_TRUE(has_error(load("version: 2\nname: x\nbase: base.yaml\nseeds: {count: 1}\naxes: []\n"),
                        "unsupported version 2"));
  EXPECT_TRUE(has_error(load("version: 1\nbase: base.yaml\nseeds: {count: 1}\naxes: []\n"),
                        "name: is required"));
  EXPECT_TRUE(
      has_error(load("version: 1\nname: x\nseeds: {count: 1}\naxes: []\n"), "base: is required"));
  EXPECT_TRUE(
      has_error(load("version: 1\nname: x\nbase: base.yaml\naxes: []\n"), "seeds: is required"));
  EXPECT_TRUE(has_error(load(kHead), "axes: is required"));
}

TEST(ExperimentLoad, ABaseThatCannotBeReadIsReported) {
  const LoadedExperiment l =
      load("version: 1\nname: x\nbase: missing.yaml\nseeds: {count: 1}\naxes: []\n");
  EXPECT_TRUE(has_error(l, "cannot read the base scenario missing.yaml")) << error_text(l);
}

TEST(ExperimentLoad, MalformedYamlAndTheWrongShapesAreErrorsNotCrashes) {
  EXPECT_FALSE(load("version: [").ok());
  EXPECT_FALSE(load("- a\n- b\n").ok());
  EXPECT_FALSE(load("").ok());
  EXPECT_TRUE(has_error(load(kHead + "axes: 3\n"), "axes: must be a list"));
  EXPECT_TRUE(has_error(load(kHead + "axes: [3]\n"), "axes[0]: must be a mapping"));
  EXPECT_TRUE(has_error(load(kHead + "axes: [{name: a}]\n"), "axes[0].levels: is required"));
  EXPECT_TRUE(
      has_error(load(kHead + "axes: [{name: a, levels: []}]\n"), "must be a non-empty list"));
  EXPECT_TRUE(has_error(load(kHead + "axes: [{name: a, levels: [{label: l}]}]\n"),
                        "axes[0].levels[0].set: is required"));
  EXPECT_TRUE(has_error(load(kHead + "axes: [{name: a, levels: [{label: l, set: 3}]}]\n"),
                        "axes[0].levels[0].set: must be a mapping"));
}

TEST(ExperimentLoad, NamesAndLabelsAreRestrictedAndUnique) {
  EXPECT_TRUE(has_error(load(kHead + "axes: [{name: Loss, levels: [{label: a, set: {}}]}]\n"),
                        "axes[0].name: must be 1 to 32 lowercase"));
  EXPECT_TRUE(has_error(load(kHead + "axes: [{name: a, levels: [{label: \"a b\", set: {}}]}]\n"),
                        "must be 1 to 32 letters, digits"));
  EXPECT_TRUE(has_error(load(kHead + "axes: [{name: a, levels: [{label: a-b, set: {}}]}]\n"),
                        "must be 1 to 32 letters, digits"));
  EXPECT_TRUE(has_error(
      load(kHead + "axes: [{name: a, levels: [{label: l, set: {}}, {label: l, set: {}}]}]\n"),
      "duplicate label l"));
  EXPECT_TRUE(has_error(load(kHead + "axes: [{name: a, levels: [{label: l, set: {}}]},"
                                     " {name: a, levels: [{label: l, set: {}}]}]\n"),
                        "duplicate axis a"));
}

TEST(ExperimentExpand, TheCellsAreTheProductWithTheLastAxisFastest) {
  const LoadedExperiment l = load(kSmall);
  ASSERT_TRUE(l.ok());
  const Expansion x = swarm::expand(l.experiment);
  ASSERT_TRUE(x.ok());
  ASSERT_EQ(x.cells.size(), 6U);
  const char* names[] = {"p00-n04", "p00-n06", "p00-n08", "p30-n04", "p30-n06", "p30-n08"};
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_EQ(x.cells[i].index, i);
    EXPECT_EQ(x.cells[i].name, names[i]);
    EXPECT_EQ(x.cells[i].scenario.name, std::string("small.") + names[i]);
  }
  EXPECT_EQ(x.cells[4].levels, (std::vector<std::uint32_t>{1, 1}));
}

TEST(ExperimentExpand, APatchChangesOnlyWhatItNames) {
  const Expansion x = swarm::expand(load(kSmall).experiment);
  ASSERT_TRUE(x.ok());
  const swarm::Scenario& a = x.cells[0].scenario;  // p00-n04: nothing changed
  const swarm::Scenario& b = x.cells[4].scenario;  // p30-n06
  EXPECT_EQ(a.drones, 4U);
  EXPECT_EQ(b.drones, 6U);
  EXPECT_EQ(a.channel.loss, 0.0);
  EXPECT_EQ(b.channel.loss, 0.3);
  EXPECT_EQ(b.start.columns, 3U);
  EXPECT_EQ(b.start.spacing_m, a.start.spacing_m);  // the rest of the layout is the base's
  EXPECT_EQ(b.duration_ticks, a.duration_ticks);
  EXPECT_EQ(b.channel.latency_ticks, a.channel.latency_ticks);
}

TEST(ExperimentExpand, NoCellsAreIdentical) {
  const Expansion x = swarm::expand(load(kSmall).experiment);
  ASSERT_TRUE(x.ok());
  std::set<std::string> texts;
  for (const auto& c : x.cells) {
    swarm::Scenario s = c.scenario;
    s.name = "x";  // the name differs by construction; compare everything else
    texts.insert(swarm::canonical_text(s));
  }
  EXPECT_EQ(texts.size(), 6U);
}

TEST(ExperimentExpand, NoAxesIsOneCellOfTheBase) {
  Experiment e = load(kSmall).experiment;
  e.axes.clear();
  const Expansion x = swarm::expand(e);
  ASSERT_TRUE(x.ok());
  ASSERT_EQ(x.cells.size(), 1U);
  EXPECT_EQ(x.cells[0].name, "base");
  EXPECT_EQ(x.cells[0].scenario.drones, 4U);
}

TEST(ExperimentExpand, ReplicatesGetConsecutiveSeedsAndNothingElseChanges) {
  const Experiment e = load(kSmall).experiment;
  const Expansion x = swarm::expand(e);
  ASSERT_TRUE(x.ok());
  const swarm::Scenario r0 = swarm::scenario_for_replicate(e, x.cells[2], 0);
  const swarm::Scenario r2 = swarm::scenario_for_replicate(e, x.cells[2], 2);
  EXPECT_EQ(r0.seed, 10U);  // not the base file's 9
  EXPECT_EQ(r2.seed, 12U);
  swarm::Scenario same = r2;
  same.seed = r0.seed;
  EXPECT_EQ(swarm::canonical_text(same), swarm::canonical_text(r0));
}

TEST(ExperimentExpand, ALevelThatBreaksTheScenarioIsReportedWithItsCell) {
  Experiment e = load(kSmall).experiment;
  e.axes[0].levels[1].patch = "comms: {loss: 3.0}\n";
  const Expansion x = swarm::expand(e);
  EXPECT_FALSE(x.ok());
  EXPECT_TRUE(x.cells.empty());
  ASSERT_FALSE(x.errors.empty());
  EXPECT_EQ(x.errors[0].path.rfind("cell p30-", 0), 0U) << x.errors[0].path;
  EXPECT_NE(x.errors[0].message.find("loss"), std::string::npos) << x.errors[0].message;
}

TEST(ExperimentExpand, AMisspelledKeyInALevelIsAnError) {
  Experiment e = load(kSmall).experiment;
  e.axes[0].levels[1].patch = "comms: {lss: 0.3}\n";
  const Expansion x = swarm::expand(e);
  ASSERT_FALSE(x.ok());
  EXPECT_NE(x.errors[0].message.find("unknown key"), std::string::npos) << x.errors[0].message;
}

TEST(ExperimentExpand, ThePatchesCannotSetTheSeedTheNameOrTheVersion) {
  for (const char* key : {"seed: 5\n", "name: other\n", "version: 1\n"}) {
    Experiment e = load(kSmall).experiment;
    e.axes[0].levels[0].patch = key;
    const Expansion x = swarm::expand(e);
    ASSERT_FALSE(x.ok()) << key;
    EXPECT_NE(x.errors[0].message.find("must not set"), std::string::npos) << key;
  }
}

TEST(ExperimentExpand, TwoAxesWritingTheSameValueAreRejected) {
  Experiment e = load(kSmall).experiment;
  e.axes[1].levels[1].patch = "comms: {loss: 0.2}\n";
  const Expansion x = swarm::expand(e);
  ASSERT_FALSE(x.ok());
  EXPECT_NE(x.errors[0].message.find("writes comms.loss"), std::string::npos)
      << x.errors[0].message;
}

TEST(ExperimentExpand, TwoAxesMayWriteDifferentValuesInTheSameSection) {
  Experiment e = load(kSmall).experiment;
  e.axes[1].levels[1].patch = "comms: {latency_ticks: 3}\n";
  EXPECT_TRUE(swarm::expand(e).ok());
}

TEST(ExperimentExpand, ALevelThatIsNotAMappingOrAnUnparsableBaseIsAnError) {
  Experiment e = load(kSmall).experiment;
  e.axes[0].levels[0].patch = "[1, 2]";
  EXPECT_FALSE(swarm::expand(e).ok());
  Experiment f = load(kSmall).experiment;
  f.base = "version: [";
  EXPECT_FALSE(swarm::expand(f).ok());
}

TEST(ExperimentExpand, TooManyCellsAreRejectedBeforeAnyAreBuilt) {
  Experiment e = load(kSmall).experiment;
  e.axes.clear();
  for (int a = 0; a < 6; ++a) {
    swarm::Axis axis;
    axis.name = "a" + std::to_string(a);
    for (int l = 0; l < 10; ++l) {
      axis.levels.push_back({"l" + std::to_string(l), "{}"});
    }
    e.axes.push_back(axis);
  }
  EXPECT_GT(swarm::cell_count(e), swarm::kMaxCells);
  const Expansion x = swarm::expand(e);
  ASSERT_FALSE(x.ok());
  EXPECT_NE(x.errors[0].message.find("more than"), std::string::npos);
}

TEST(ExperimentExpand, TheCellCountDoesNotOverflow) {
  Experiment e;
  for (int a = 0; a < 16; ++a) {
    swarm::Axis axis;
    axis.name = "a" + std::to_string(a);
    for (int l = 0; l < 1000; ++l) {
      axis.levels.push_back({"l", "{}"});
    }
    e.axes.push_back(axis);
  }
  EXPECT_GT(swarm::cell_count(e), swarm::kMaxCells);
}

TEST(ExperimentFiles, ThePlannedMatrixHas432CellsAndEveryCellLoads) {
  const swarm::ReadFile read = swarm::testing::read_from(SWARM_EXPERIMENT_DIR);
  std::string text;
  ASSERT_TRUE(read("matrix.yaml", text));
  const LoadedExperiment l = swarm::load_experiment(text, read);
  ASSERT_TRUE(l.ok()) << error_text(l);
  EXPECT_EQ(l.experiment.seed_count, 5U);
  EXPECT_EQ(swarm::cell_count(l.experiment), 432U);
  const Expansion x = swarm::expand(l.experiment);
  ASSERT_TRUE(x.ok()) << (x.errors.empty() ? "" : swarm::to_string(x.errors[0]));
  ASSERT_EQ(x.cells.size(), 432U);
  EXPECT_EQ(x.cells.front().name, "p00-clear-none-n05-none-field");
  EXPECT_EQ(x.cells.back().name, "p50-rain-pulsed-n20-one-hills");

  std::set<std::string> names;
  for (const auto& c : x.cells) {
    names.insert(c.name);
  }
  EXPECT_EQ(names.size(), 432U);

  // Spot checks that the levels mean what they are called.
  const auto find = [&](const std::string& name) -> const swarm::Scenario& {
    for (const auto& c : x.cells) {
      if (c.name == name) {
        return c.scenario;
      }
    }
    ADD_FAILURE() << "no cell " << name;
    return x.cells[0].scenario;
  };
  const swarm::Scenario& s = find("p30-fog-pulsed-n20-one-hills");
  EXPECT_EQ(s.drones, 20U);
  EXPECT_EQ(s.channel.loss, 0.3);
  EXPECT_EQ(s.faults.size(), 1U);
  EXPECT_EQ(s.channel.jammers.size(), 1U);
  EXPECT_EQ(s.terrain.type, swarm::TerrainType::kHills);
  EXPECT_LT(s.weather.visibility_m, 1000.0);
  const swarm::Scenario& t = find("p00-clear-none-n05-none-field");
  EXPECT_EQ(t.drones, 5U);
  EXPECT_TRUE(t.faults.empty());
  EXPECT_TRUE(t.channel.jammers.empty());
}

namespace {

Expansion expand_file(const std::string& name) {
  const swarm::ReadFile read = swarm::testing::read_from(SWARM_EXPERIMENT_DIR);
  std::string text;
  EXPECT_TRUE(read(name, text)) << name;
  const LoadedExperiment l = swarm::load_experiment(text, read);
  EXPECT_TRUE(l.ok()) << name << "\n" << error_text(l);
  const Expansion x = swarm::expand(l.experiment);
  EXPECT_TRUE(x.ok()) << name;
  return x;
}

}  // namespace

TEST(ExperimentFiles, EachAblationVariantDisablesExactlyOneComponent) {
  const Expansion x = expand_file("ablation.yaml");
  ASSERT_EQ(x.cells.size(), 48U);
  // Variant is the first axis and varies slowest: the cells of one variant are contiguous, and the
  // same position in each block has the same loss, jamming, and faults.
  const std::size_t block = 12;
  for (std::size_t i = 0; i < block; ++i) {
    const swarm::Scenario& full = x.cells[i].scenario;
    const swarm::Scenario& no_prediction = x.cells[block + i].scenario;
    const swarm::Scenario& no_fallback = x.cells[2 * block + i].scenario;
    const swarm::Scenario& no_separation = x.cells[3 * block + i].scenario;
    EXPECT_EQ(full.predictor, swarm::PredictorChoice::kKalman);
    EXPECT_TRUE(full.fallback_enabled);
    EXPECT_TRUE(full.separation_enabled);

    EXPECT_EQ(no_prediction.predictor, swarm::PredictorChoice::kHoldLast);
    EXPECT_TRUE(no_prediction.fallback_enabled);
    EXPECT_TRUE(no_prediction.separation_enabled);

    EXPECT_EQ(no_fallback.predictor, swarm::PredictorChoice::kKalman);
    EXPECT_FALSE(no_fallback.fallback_enabled);
    EXPECT_TRUE(no_fallback.separation_enabled);

    EXPECT_EQ(no_separation.predictor, swarm::PredictorChoice::kKalman);
    EXPECT_TRUE(no_separation.fallback_enabled);
    EXPECT_FALSE(no_separation.separation_enabled);

    // Nothing else differs: undo the one change and the text matches the full controller's.
    const auto text_of = [](swarm::Scenario s) {
      s.name = "x";
      return swarm::canonical_text(s);
    };
    swarm::Scenario undo = no_prediction;
    undo.predictor = swarm::PredictorChoice::kKalman;
    EXPECT_EQ(text_of(undo), text_of(full));
    undo = no_fallback;
    undo.fallback_enabled = true;
    EXPECT_EQ(text_of(undo), text_of(full));
    undo = no_separation;
    undo.separation_enabled = true;
    EXPECT_EQ(text_of(undo), text_of(full));
  }
}

TEST(ExperimentFiles, TheBaselinesDifferOnlyInThePredictor) {
  const Expansion x = expand_file("baselines.yaml");
  ASSERT_EQ(x.cells.size(), 48U);
  const swarm::PredictorChoice order[] = {
      swarm::PredictorChoice::kHoldLast, swarm::PredictorChoice::kDeadReckoning,
      swarm::PredictorChoice::kKalman, swarm::PredictorChoice::kOracle};
  for (std::size_t v = 0; v < 4; ++v) {
    for (std::size_t i = 0; i < 12; ++i) {
      swarm::Scenario s = x.cells[v * 12 + i].scenario;
      EXPECT_EQ(s.predictor, order[v]);
      s.predictor = swarm::PredictorChoice::kKalman;
      s.name = "x";
      swarm::Scenario ref = x.cells[2 * 12 + i].scenario;
      ref.name = "x";
      EXPECT_EQ(swarm::canonical_text(s), swarm::canonical_text(ref));
    }
  }
}

TEST(ExperimentLoad, AnAxisCannotBeNamedLikeAColumnOfTheResultsTable) {
  for (const char* name : {"drones", "seed", "status", "cell", "replicate", "scenario", "ticks"}) {
    const std::string axis =
        std::string("axes: [{name: ") + name + ", levels: [{label: a, set: {}}]}]\n";
    EXPECT_TRUE(has_error(load(kHead + axis), "is a column of the results table")) << name;
  }
  EXPECT_FALSE(has_error(load(kHead + "axes: [{name: size, levels: [{label: a, set: {}}]}]\n"),
                         "is a column"));
}
