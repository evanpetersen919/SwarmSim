#include "swarm/experiment/experiment.hpp"

#include <yaml-cpp/yaml.h>

#include <cerrno>
#include <cstdlib>
#include <set>
#include <utility>

#include "swarm/scenario/metrics_csv.hpp"

namespace swarm {

std::string to_string(const ExperimentError& error) {
  std::string out = error.path;
  if (error.line > 0) {
    out += out.empty() ? "line " : " (line ";
    out += std::to_string(error.line);
    if (!error.path.empty()) {
      out += ")";
    }
  }
  if (!out.empty()) {
    out += ": ";
  }
  return out + error.message;
}

namespace {

constexpr std::size_t kMaxExperimentName = 64;
constexpr std::size_t kMaxLabel = 32;
constexpr std::size_t kMaxAxes = 16;
constexpr std::uint32_t kMaxSeeds = 10000;
constexpr std::uint64_t kMaxFirstSeed = std::uint64_t{1} << 62;
constexpr std::size_t kMaxReportedCellErrors = 20;

int line_of(const YAML::Node& node) {
  const YAML::Mark mark = node.Mark();
  return mark.is_null() ? 0 : mark.line + 1;
}

bool is_digit(char c) { return c >= '0' && c <= '9'; }
bool is_lower(char c) { return c >= 'a' && c <= 'z'; }
bool is_upper(char c) { return c >= 'A' && c <= 'Z'; }

bool valid_name(const std::string& s, std::size_t max_length, bool allow_upper_and_dot) {
  if (s.empty() || s.size() > max_length) {
    return false;
  }
  for (char c : s) {
    const bool ok = is_lower(c) || is_digit(c) || c == '_' ||
                    (allow_upper_and_dot && (is_upper(c) || c == '.'));
    if (!ok) {
      return false;
    }
  }
  return true;
}

// A mapping whose keys are checked against the ones the caller reads.
class Mapping {
 public:
  Mapping(std::vector<ExperimentError>& errors, const YAML::Node& node, std::string path)
      : errors_(&errors), node_(node), path_(std::move(path)) {
    if (!node.IsMap()) {
      add(path_, line_of(node), "must be a mapping");
      return;
    }
    valid_ = true;
    for (auto it = node.begin(); it != node.end(); ++it) {
      if (!it->first.IsScalar()) {
        add(path_, line_of(it->first), "keys must be text");
        valid_ = false;
      }
    }
  }

  bool valid() const { return valid_; }
  int line() const { return line_of(node_); }
  std::string sub(const std::string& key) const { return path_.empty() ? key : path_ + "." + key; }
  void add(const std::string& path, int line, const std::string& message) {
    errors_->push_back({path, line, message});
  }

  YAML::Node find(const std::string& key) {
    used_.insert(key);
    if (!valid_) {
      return YAML::Node();
    }
    const YAML::Node& constant = node_;
    return constant[key];
  }

  // The scalar text at `key`, or false after adding an error (absent keys are errors if required).
  bool text(const std::string& key, std::string& out, bool required) {
    const YAML::Node node = find(key);
    if (!node.IsDefined()) {
      if (required && valid_) {
        add(sub(key), line(), "is required");
      }
      return false;
    }
    if (!node.IsScalar()) {
      add(sub(key), line_of(node), "must be a single value");
      return false;
    }
    out = node.Scalar();
    return true;
  }

  bool whole_number(const std::string& key, std::uint64_t lo, std::uint64_t hi, std::uint64_t& out,
                    bool required) {
    std::string t;
    if (!text(key, t, required)) {
      return false;
    }
    bool ok = !t.empty() && t.size() <= 20;
    for (char c : t) {
      ok = ok && is_digit(c);
    }
    std::uint64_t v = 0;
    if (ok) {
      errno = 0;
      char* end = nullptr;
      v = std::strtoull(t.c_str(), &end, 10);
      ok = errno != ERANGE && end == t.c_str() + t.size() && v >= lo && v <= hi;
    }
    if (!ok) {
      add(sub(key), line_of(find(key)),
          "must be a whole number in [" + std::to_string(lo) + ", " + std::to_string(hi) +
              "], got " + t);
      return false;
    }
    out = v;
    return true;
  }

  void finish() {
    if (!valid_) {
      return;
    }
    for (auto it = node_.begin(); it != node_.end(); ++it) {
      if (used_.count(it->first.Scalar()) == 0) {
        add(sub(it->first.Scalar()), line_of(it->first), "unknown key");
      }
    }
  }

 private:
  std::vector<ExperimentError>* errors_;
  YAML::Node node_;
  std::string path_;
  bool valid_ = false;
  std::set<std::string> used_;
};

// Names of the results table's own columns: an axis with one of these would give the table two
// columns of the same name.
bool is_reserved_column(const std::string& name) {
  static const std::set<std::string> reserved = [] {
    std::set<std::string> names = {"experiment", "cell", "replicate", "status", "verified"};
    const std::string header = metrics_csv_header();
    std::size_t start = 0;
    while (start < header.size()) {
      std::size_t end = header.find_first_of(",\n", start);
      if (end == std::string::npos) {
        end = header.size();
      }
      names.insert(header.substr(start, end - start));
      start = end + 1;
    }
    return names;
  }();
  return reserved.count(name) != 0;
}

std::string emit(const YAML::Node& node) {
  YAML::Emitter out;
  out << node;
  return out.c_str();
}

// Merges `src` into `dst`: maps key by key, anything else by replacement.
void merge(YAML::Node dst, const YAML::Node& src) {
  for (auto it = src.begin(); it != src.end(); ++it) {
    const std::string key = it->first.Scalar();
    const YAML::Node value = it->second;
    if (value.IsMap() && dst[key].IsMap()) {
      merge(dst[key], value);
    } else {
      dst[key] = YAML::Clone(value);
    }
  }
}

// The dotted paths of the values that a patch writes; a list or a scalar is one value.
void leaf_paths(const YAML::Node& node, const std::string& prefix, std::set<std::string>& out) {
  for (auto it = node.begin(); it != node.end(); ++it) {
    const std::string path =
        prefix.empty() ? it->first.Scalar() : prefix + "." + it->first.Scalar();
    if (it->second.IsMap() && it->second.size() > 0) {
      leaf_paths(it->second, path, out);
    } else {
      out.insert(path);
    }
  }
}

bool parse_map(const std::string& text, YAML::Node& out, std::string& why) {
  try {
    out = YAML::Load(text);
  } catch (const YAML::Exception& e) {
    why = e.msg;
    return false;
  }
  if (!out.IsMap() && !out.IsNull()) {
    why = "must be a mapping";
    return false;
  }
  if (out.IsNull()) {
    out = YAML::Node(YAML::NodeType::Map);
  }
  return true;
}

}  // namespace

LoadedExperiment load_experiment(const std::string& text, const ReadFile& read_file) {
  LoadedExperiment result;
  std::vector<ExperimentError>& errors = result.errors;
  Experiment& e = result.experiment;

  YAML::Node doc;
  try {
    doc = YAML::Load(text);
  } catch (const YAML::Exception& ex) {
    errors.push_back({"", ex.mark.is_null() ? 0 : ex.mark.line + 1, ex.msg});
    return result;
  }
  Mapping root(errors, doc, "");
  if (!root.valid()) {
    return result;
  }

  std::uint64_t version = 0;
  if (root.whole_number("version", 0, ~std::uint64_t{0}, version, true) &&
      version != kExperimentVersion) {
    root.add("version", line_of(root.find("version")),
             "unsupported version " + std::to_string(version) + "; this build reads version " +
                 std::to_string(kExperimentVersion));
  }

  std::string name;
  if (root.text("name", name, true)) {
    if (!valid_name(name, kMaxExperimentName, true)) {
      root.add("name", line_of(root.find("name")), "must be 1 to 64 letters, digits, '_' or '.'");
    } else {
      e.name = name;
    }
  }

  std::string base_path;
  if (root.text("base", base_path, true)) {
    std::string base_text;
    if (!read_file || !read_file(base_path, base_text)) {
      root.add("base", line_of(root.find("base")), "cannot read the base scenario " + base_path);
    } else {
      e.base = base_text;
    }
  }

  const YAML::Node seeds = root.find("seeds");
  if (seeds.IsDefined()) {
    Mapping sm(errors, seeds, "seeds");
    std::uint64_t first = 1;
    std::uint64_t count = 1;
    if (sm.valid()) {
      sm.whole_number("first", 0, kMaxFirstSeed, first, false);
      sm.whole_number("count", 1, kMaxSeeds, count, true);
      sm.finish();
    }
    e.first_seed = first;
    e.seed_count = static_cast<std::uint32_t>(count);
  } else {
    root.add("seeds", root.line(), "is required");
  }

  const YAML::Node axes = root.find("axes");
  if (axes.IsDefined()) {
    if (!axes.IsSequence()) {
      root.add("axes", line_of(axes), "must be a list");
    } else if (axes.size() > kMaxAxes) {
      root.add("axes", line_of(axes), "has more than " + std::to_string(kMaxAxes) + " axes");
    } else {
      std::set<std::string> axis_names;
      for (std::size_t i = 0; i < axes.size(); ++i) {
        const std::string apath = "axes[" + std::to_string(i) + "]";
        Mapping am(errors, axes[i], apath);
        if (!am.valid()) {
          continue;
        }
        Axis axis;
        if (am.text("name", axis.name, true)) {
          if (!valid_name(axis.name, kMaxLabel, false)) {
            am.add(apath + ".name", line_of(am.find("name")),
                   "must be 1 to 32 lowercase letters, digits or '_'");
          } else if (is_reserved_column(axis.name)) {
            am.add(apath + ".name", line_of(am.find("name")),
                   axis.name + " is a column of the results table; choose another name");
          } else if (!axis_names.insert(axis.name).second) {
            am.add(apath + ".name", line_of(am.find("name")), "duplicate axis " + axis.name);
          }
        }
        const YAML::Node levels = am.find("levels");
        if (!levels.IsDefined()) {
          am.add(apath + ".levels", am.line(), "is required");
        } else if (!levels.IsSequence() || levels.size() == 0) {
          am.add(apath + ".levels", line_of(levels), "must be a non-empty list");
        } else {
          std::set<std::string> labels;
          for (std::size_t j = 0; j < levels.size(); ++j) {
            const std::string lpath = apath + ".levels[" + std::to_string(j) + "]";
            Mapping lm(errors, levels[j], lpath);
            if (!lm.valid()) {
              continue;
            }
            Level level;
            if (lm.text("label", level.label, true)) {
              if (!valid_name(level.label, kMaxLabel, true)) {
                lm.add(lpath + ".label", line_of(lm.find("label")),
                       "must be 1 to 32 letters, digits, '_' or '.'");
              } else if (!labels.insert(level.label).second) {
                lm.add(lpath + ".label", line_of(lm.find("label")),
                       "duplicate label " + level.label);
              }
            }
            const YAML::Node set = lm.find("set");
            if (!set.IsDefined()) {
              lm.add(lpath + ".set", lm.line(), "is required (use {} for no change)");
            } else if (!set.IsMap()) {
              lm.add(lpath + ".set", line_of(set), "must be a mapping");
            } else {
              level.patch = emit(set);
            }
            lm.finish();
            axis.levels.push_back(std::move(level));
          }
        }
        am.finish();
        e.axes.push_back(std::move(axis));
      }
    }
  } else {
    root.add("axes", root.line(), "is required (use [] for a single cell)");
  }

  root.finish();
  return result;
}

std::uint64_t cell_count(const Experiment& experiment) {
  std::uint64_t count = 1;
  for (const Axis& axis : experiment.axes) {
    const std::uint64_t n = axis.levels.size();
    if (n != 0 && count > kMaxCells * 2 / n + 1) {
      return kMaxCells * 2;  // far over the limit; stops the product from overflowing
    }
    count *= n;
  }
  return count;
}

Expansion expand(const Experiment& experiment) {
  Expansion result;
  std::vector<ExperimentError>& errors = result.errors;

  const std::uint64_t total = cell_count(experiment);
  if (total > kMaxCells) {
    errors.push_back({"", 0, "more than " + std::to_string(kMaxCells) + " cells"});
    return result;
  }

  YAML::Node base;
  std::string why;
  if (!parse_map(experiment.base, base, why)) {
    errors.push_back({"base", 0, why});
    return result;
  }

  // Parse every patch once, reject the keys the experiment owns, and find two axes that write the
  // same value, whose order would otherwise decide the outcome.
  std::vector<std::vector<YAML::Node>> patches(experiment.axes.size());
  std::vector<std::set<std::string>> written(experiment.axes.size());
  for (std::size_t a = 0; a < experiment.axes.size(); ++a) {
    const Axis& axis = experiment.axes[a];
    for (const Level& level : axis.levels) {
      const std::string where = "axis " + axis.name + ", level " + level.label;
      YAML::Node patch;
      if (!parse_map(level.patch, patch, why)) {
        errors.push_back({where, 0, why});
        patch = YAML::Node(YAML::NodeType::Map);
      }
      const YAML::Node& read_only = patch;
      for (const char* owned : {"seed", "name", "version"}) {
        if (read_only[owned].IsDefined()) {
          errors.push_back({where, 0, std::string("must not set '") + owned + "'"});
        }
      }
      leaf_paths(patch, "", written[a]);
      patches[a].push_back(patch);
    }
    for (std::size_t b = 0; b < a; ++b) {
      for (const std::string& path : written[a]) {
        if (written[b].count(path) != 0) {
          errors.push_back(
              {"axis " + axis.name, 0,
               "writes " + path + ", which axis " + experiment.axes[b].name + " also writes"});
        }
      }
    }
  }
  if (!errors.empty()) {
    return result;
  }

  std::size_t cell_errors = 0;
  result.cells.reserve(static_cast<std::size_t>(total));
  for (std::uint64_t index = 0; index < total; ++index) {
    Cell cell;
    cell.index = index;
    cell.levels.resize(experiment.axes.size());
    std::uint64_t rest = index;
    for (std::size_t a = experiment.axes.size(); a-- > 0;) {
      const std::uint64_t n = experiment.axes[a].levels.size();
      cell.levels[a] = static_cast<std::uint32_t>(rest % n);
      rest /= n;
    }
    for (std::size_t a = 0; a < experiment.axes.size(); ++a) {
      cell.name += (a == 0 ? "" : "-") + experiment.axes[a].levels[cell.levels[a]].label;
    }
    if (experiment.axes.empty()) {
      cell.name = "base";
    }

    YAML::Node merged = YAML::Clone(base);
    for (std::size_t a = 0; a < experiment.axes.size(); ++a) {
      merge(merged, patches[a][cell.levels[a]]);
    }
    LoadResult loaded = load_scenario(emit(merged), experiment.name + "." + cell.name);
    if (!loaded.ok()) {
      for (const ScenarioError& se : loaded.errors) {
        if (cell_errors++ < kMaxReportedCellErrors) {
          errors.push_back({"cell " + cell.name, 0, to_string(se)});
        }
      }
      continue;
    }
    cell.scenario = std::move(loaded.scenario);
    cell.scenario.name = experiment.name + "." + cell.name;
    cell.scenario.seed = experiment.first_seed;
    result.cells.push_back(std::move(cell));
  }
  if (!errors.empty()) {
    result.cells.clear();
  }
  return result;
}

Scenario scenario_for_replicate(const Experiment& experiment, const Cell& cell,
                                std::uint32_t replicate) {
  Scenario scenario = cell.scenario;
  scenario.seed = experiment.first_seed + replicate;
  return scenario;
}

}  // namespace swarm
