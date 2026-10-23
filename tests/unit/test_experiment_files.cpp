#include <gtest/gtest.h>

#include <cstddef>
#include <map>
#include <string>

#include "experiment_files.hpp"
#include "swarm/experiment/experiment.hpp"

namespace {

std::string error_text(const swarm::LoadedExperiment& l) {
  std::string out;
  for (const auto& e : l.errors) {
    out += swarm::to_string(e) + "\n";
  }
  return out;
}

}  // namespace

// Every experiment file in experiments/ must load and expand, and its cell count must be the one
// experiments/README.md states (smoke.yaml is four cells of two seeds each, the eight short runs
// the README names). A file that goes stale when the scenario format changes fails here
// and not in a batch run that nobody started.
TEST(ExperimentFiles, EveryExperimentFileLoadsAndExpandsToTheDocumentedCellCount) {
  const std::map<std::string, std::size_t> documented = {
      {"smoke.yaml", 4},
      {"matrix.yaml", 432},
      {"ablation.yaml", 48},
      {"ablation_fog.yaml", 16},
      {"baselines.yaml", 48},
      {"baselines_burst.yaml", 48},
      {"baselines_drop.yaml", 96},
      {"baselines_burst_drop.yaml", 96},
      {"fog_convention.yaml", 12},
      {"matrix_fixed_wing.yaml", 216},
      {"baselines_fixed_wing.yaml", 72},
      {"ablation_fixed_wing.yaml", 48},
      {"probe_fixed_wing.yaml", 384},
      {"pilot_fixed_wing.yaml", 8},
      {"pilot_fixed_wing_comms.yaml", 24},
      {"pilot_fixed_wing_altimeter.yaml", 4},
      {"probe_fixed_wing_map_errors.yaml", 13},
      {"fixed_wing_forward.yaml", 64},
      {"fixed_wing_forward_sensitivity.yaml", 36},
      {"probe_fixed_wing_avoid.yaml", 12},
      {"fixed_wing_avoid.yaml", 96},
      {"fixed_wing_avoid_sensitivity.yaml", 12},
      {"fixed_wing_avoid_rejoin.yaml", 112},
      {"fixed_wing_avoid_urgent.yaml", 48},
      {"fixed_wing_avoid_turnfirst.yaml", 36},
      {"fixed_wing_avoid_clearance.yaml", 6},
      {"fixed_wing_share.yaml", 108},
      {"fixed_wing_share_sensitivity.yaml", 6},
      {"fixed_wing_share_trust.yaml", 32},
  };
  const swarm::ReadFile read = swarm::testing::read_from(SWARM_EXPERIMENT_DIR);
  for (const auto& [file, cells] : documented) {
    std::string text;
    ASSERT_TRUE(read(file, text)) << file;
    const swarm::LoadedExperiment l = swarm::load_experiment(text, read);
    ASSERT_TRUE(l.ok()) << file << ": " << error_text(l);
    EXPECT_EQ(swarm::cell_count(l.experiment), cells) << file;
    const swarm::Expansion x = swarm::expand(l.experiment);
    ASSERT_TRUE(x.ok()) << file << ": " << (x.errors.empty() ? "" : swarm::to_string(x.errors[0]));
    EXPECT_EQ(x.cells.size(), cells) << file;
  }
}
