#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "swarm/scenario/scenario.hpp"

namespace swarm {

// An experiment is a base scenario, a list of seeds, and axes. An axis has a name and levels; a
// level has a label and a fragment of scenario text (`set`) that is merged over the base: maps are
// merged key by key, and any other value (a number, a list) replaces the base's. The cells are
// the product of the axes' levels. Every cell's scenario is checked by the scenario loader, so a
// misspelled key or an out-of-range value in a level is an error at expansion, before any run.

struct ExperimentError {
  std::string path;  // where in the experiment file, or which cell, empty for the whole document
  int line = 0;      // 1-based, 0 if unknown
  std::string message;
};

std::string to_string(const ExperimentError& error);

struct Level {
  std::string label;  // letters, digits, '_' and '.', at most 32 characters
  std::string patch;  // YAML text of a mapping, possibly empty (no change)
};

struct Axis {
  std::string name;  // lowercase letters, digits and '_', at most 32 characters
  std::vector<Level> levels;
};

struct Experiment {
  std::string name;  // letters, digits, '_' and '.', at most 64 characters
  std::string base;  // scenario text
  std::uint64_t first_seed = 1;
  std::uint32_t seed_count = 1;
  std::vector<Axis> axes;
};

// Version of the experiment file format this build reads.
inline constexpr std::uint64_t kExperimentVersion = 1;

// Reads the text of a file named in the experiment, relative to wherever the caller resolves
// from. Returns false if it cannot.
using ReadFile = std::function<bool(const std::string& path, std::string& text)>;

struct LoadedExperiment {
  Experiment experiment;
  std::vector<ExperimentError> errors;
  bool ok() const { return errors.empty(); }
};

// Parses and validates experiment text, reading the base scenario through `read_file`. Never
// throws; unknown keys are errors. It does not expand the cells.
LoadedExperiment load_experiment(const std::string& text, const ReadFile& read_file);

// The largest number of cells an experiment may have.
inline constexpr std::uint64_t kMaxCells = 100000;

// The number of cells: the product of the axes' level counts, and 1 with no axes.
std::uint64_t cell_count(const Experiment& experiment);

struct Cell {
  std::uint64_t index = 0;
  std::vector<std::uint32_t> levels;  // one level index per axis, in the axes' order
  std::string name;                   // labels joined by '-', or "base" with no axes
  Scenario scenario;                  // named "<experiment>.<cell>", seeded with first_seed
};

struct Expansion {
  std::vector<Cell> cells;
  std::vector<ExperimentError> errors;
  bool ok() const { return errors.empty(); }
};

// The cells in order, the last axis varying fastest. Errors: a patch that is not a mapping or
// sets `seed`, `name` or `version`, two axes writing the same key, a cell whose scenario does not
// load, and more than kMaxCells cells.
Expansion expand(const Experiment& experiment);

// The scenario of one replicate of a cell: the cell's scenario with seed first_seed + replicate.
Scenario scenario_for_replicate(const Experiment& experiment, const Cell& cell,
                                std::uint32_t replicate);

}  // namespace swarm
