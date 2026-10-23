#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "swarm/lidar.hpp"
#include "swarm/scenario/scenario.hpp"

namespace swarm {

Scenario::Scenario() {
  terrain.size_x = 1000.0;
  terrain.size_y = 1000.0;
  terrain.cell_size = 8.0;
  obstacles.density_per_m2 = 0.0;
  obstacles.spawn_clear_radius = 0.0;
}

std::string to_string(const ScenarioError& error) {
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

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr std::uint32_t kMaxDrones = 4096;
constexpr std::size_t kMaxNameLength = 256;
constexpr std::size_t kMaxCommands = 1000;

struct Context {
  std::vector<ScenarioError> errors;
  void add(const std::string& path, int line, const std::string& message) {
    errors.push_back({path, line, message});
  }
};

int line_of(const YAML::Node& node) {
  const YAML::Mark mark = node.Mark();
  return mark.is_null() ? 0 : mark.line + 1;
}

std::string number_text(double v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%g", v);
  return buf;
}

// A numeric bound with open or closed ends, and the words for it in an error message.
struct Bounds {
  double lo = -kInf;
  double hi = kInf;
  bool lo_open = false;
  bool hi_open = false;

  bool contains(double v) const {
    return (lo_open ? v > lo : v >= lo) && (hi_open ? v < hi : v <= hi);
  }
  std::string describe() const {
    if (std::isinf(hi)) {
      return (lo_open ? "greater than " : "at least ") + number_text(lo);
    }
    if (std::isinf(lo)) {
      return (hi_open ? "less than " : "at most ") + number_text(hi);
    }
    return std::string("in ") + (lo_open ? "(" : "[") + number_text(lo) + ", " + number_text(hi) +
           (hi_open ? ")" : "]");
  }
};

Bounds at_least(double lo) { return {lo, kInf, false, false}; }
Bounds greater_than(double lo) { return {lo, kInf, true, false}; }
Bounds closed(double lo, double hi) { return {lo, hi, false, false}; }
Bounds unit_interval() { return closed(0.0, 1.0); }

bool is_digit(char c) { return c >= '0' && c <= '9'; }

// Decimal notation only: digits, a sign, a point, and an exponent. This excludes `inf`, `nan`,
// hexadecimal, and anything else strtod would accept.
bool parse_number(const std::string& text, double& out) {
  if (text.empty()) {
    return false;
  }
  for (char c : text) {
    if (!(is_digit(c) || c == '+' || c == '-' || c == '.' || c == 'e' || c == 'E')) {
      return false;
    }
  }
  errno = 0;
  char* end = nullptr;
  const double v = std::strtod(text.c_str(), &end);
  if (end != text.c_str() + text.size() || errno == ERANGE || !std::isfinite(v)) {
    return false;
  }
  out = v;
  return true;
}

// Exactly sixteen hexadecimal digits, as scene hashes are written.
bool parse_hash(const std::string& text, std::uint64_t& out) {
  if (text.size() != 16) {
    return false;
  }
  std::uint64_t v = 0;
  for (char c : text) {
    std::uint64_t digit = 0;
    if (c >= '0' && c <= '9') {
      digit = static_cast<std::uint64_t>(c - '0');
    } else if (c >= 'a' && c <= 'f') {
      digit = static_cast<std::uint64_t>(c - 'a') + 10U;
    } else {
      return false;
    }
    v = (v << 4U) | digit;
  }
  out = v;
  return true;
}

bool parse_uint(const std::string& text, std::uint64_t& out) {
  if (text.empty() || text.size() > 20) {
    return false;
  }
  for (char c : text) {
    if (!is_digit(c)) {
      return false;
    }
  }
  errno = 0;
  char* end = nullptr;
  const unsigned long long v = std::strtoull(text.c_str(), &end, 10);
  if (end != text.c_str() + text.size() || errno == ERANGE) {
    return false;
  }
  out = v;
  return true;
}

// A mapping whose keys are tracked, so keys nobody asked for can be reported.
class Section {
 public:
  Section(Context& ctx, const YAML::Node& node, std::string path)
      : ctx_(&ctx), node_(node), path_(std::move(path)) {
    if (!node.IsMap()) {
      ctx.add(path_, line_of(node), "must be a mapping");
      return;
    }
    valid_ = true;
    for (auto it = node.begin(); it != node.end(); ++it) {
      if (!it->first.IsScalar()) {
        ctx.add(path_, line_of(it->first), "keys must be text");
        valid_ = false;
      }
    }
  }

  bool valid() const { return valid_; }
  Context& ctx() const { return *ctx_; }
  const std::string& path() const { return path_; }
  int line() const { return line_of(node_); }
  std::string sub(const std::string& key) const { return path_.empty() ? key : path_ + "." + key; }

  YAML::Node find(const std::string& key) {
    used_.insert(key);
    if (!valid_) {
      return YAML::Node();
    }
    const YAML::Node& constant = node_;
    return constant[key];
  }

  void require(const std::string& key) { ctx_->add(sub(key), line(), "is required"); }

  void finish() {
    if (!valid_) {
      return;
    }
    for (auto it = node_.begin(); it != node_.end(); ++it) {
      const std::string key = it->first.Scalar();
      if (used_.count(key) == 0) {
        ctx_->add(sub(key), line_of(it->first), "unknown key");
      }
    }
  }

 private:
  Context* ctx_;
  YAML::Node node_;
  std::string path_;
  bool valid_ = false;
  std::set<std::string> used_;
};

// Typed readers. Each returns true if the key was present and valid; a present but invalid key
// adds an error, an absent optional key adds nothing, and an absent required key adds "is
// required".

bool scalar_text(Section& sec, const std::string& key, const YAML::Node& node, std::string& out) {
  if (!node.IsScalar()) {
    sec.ctx().add(sec.sub(key), line_of(node), "must be a single value");
    return false;
  }
  out = node.Scalar();
  return true;
}

bool read_double(Section& sec, const std::string& key, double& out, const Bounds& bounds,
                 bool required = false) {
  const YAML::Node node = sec.find(key);
  if (!node.IsDefined()) {
    if (required) {
      sec.require(key);
    }
    return false;
  }
  std::string text;
  if (!scalar_text(sec, key, node, text)) {
    return false;
  }
  double v = 0.0;
  if (!parse_number(text, v) || !bounds.contains(v)) {
    sec.ctx().add(sec.sub(key), line_of(node),
                  "must be a number " + bounds.describe() + ", got " + text);
    return false;
  }
  out = v;
  return true;
}

bool read_uint(Section& sec, const std::string& key, std::uint64_t& out, std::uint64_t lo,
               std::uint64_t hi, bool required = false) {
  const YAML::Node node = sec.find(key);
  if (!node.IsDefined()) {
    if (required) {
      sec.require(key);
    }
    return false;
  }
  std::string text;
  if (!scalar_text(sec, key, node, text)) {
    return false;
  }
  std::uint64_t v = 0;
  if (!parse_uint(text, v) || v < lo || v > hi) {
    sec.ctx().add(sec.sub(key), line_of(node),
                  "must be a whole number in [" + std::to_string(lo) + ", " + std::to_string(hi) +
                      "], got " + text);
    return false;
  }
  out = v;
  return true;
}

bool read_u32(Section& sec, const std::string& key, std::uint32_t& out, std::uint32_t lo,
              std::uint32_t hi, bool required = false) {
  std::uint64_t v = 0;
  if (!read_uint(sec, key, v, lo, hi, required)) {
    return false;
  }
  out = static_cast<std::uint32_t>(v);
  return true;
}

bool read_bool(Section& sec, const std::string& key, bool& out) {
  const YAML::Node node = sec.find(key);
  if (!node.IsDefined()) {
    return false;
  }
  std::string text;
  if (!scalar_text(sec, key, node, text)) {
    return false;
  }
  if (text != "true" && text != "false") {
    sec.ctx().add(sec.sub(key), line_of(node), "must be true or false, got " + text);
    return false;
  }
  out = text == "true";
  return true;
}

bool read_text(Section& sec, const std::string& key, std::string& out) {
  const YAML::Node node = sec.find(key);
  if (!node.IsDefined()) {
    return false;
  }
  std::string text;
  if (!scalar_text(sec, key, node, text)) {
    return false;
  }
  if (text.empty() || text.size() > kMaxNameLength) {
    sec.ctx().add(sec.sub(key), line_of(node),
                  "must be between 1 and " + std::to_string(kMaxNameLength) + " characters");
    return false;
  }
  out = text;
  return true;
}

template <typename T>
bool read_choice(Section& sec, const std::string& key, T& out,
                 const std::vector<std::pair<std::string, T>>& choices) {
  const YAML::Node node = sec.find(key);
  if (!node.IsDefined()) {
    return false;
  }
  std::string text;
  if (!scalar_text(sec, key, node, text)) {
    return false;
  }
  for (const auto& choice : choices) {
    if (choice.first == text) {
      out = choice.second;
      return true;
    }
  }
  std::string names;
  for (const auto& choice : choices) {
    names += (names.empty() ? "" : ", ") + choice.first;
  }
  sec.ctx().add(sec.sub(key), line_of(node), "must be one of " + names + ", got " + text);
  return false;
}

// A list of exactly `n` numbers.
bool read_numbers(Section& sec, const std::string& key, std::size_t n, std::vector<double>& out,
                  bool required = false) {
  const YAML::Node node = sec.find(key);
  if (!node.IsDefined()) {
    if (required) {
      sec.require(key);
    }
    return false;
  }
  if (!node.IsSequence() || node.size() != n) {
    sec.ctx().add(sec.sub(key), line_of(node),
                  "must be a list of " + std::to_string(n) + " numbers");
    return false;
  }
  std::vector<double> values;
  for (std::size_t i = 0; i < n; ++i) {
    double v = 0.0;
    if (!node[i].IsScalar() || !parse_number(node[i].Scalar(), v)) {
      sec.ctx().add(sec.sub(key), line_of(node[i]), "entries must be numbers");
      return false;
    }
    values.push_back(v);
  }
  out = values;
  return true;
}

// Seconds in the file, whole ticks in the scenario. A time that is not a whole number of ticks is
// an error: nothing is rounded without being said.
class Clock {
 public:
  explicit Clock(double dt) : dt_(dt) {}
  bool known() const { return dt_ > 0.0; }

  bool read(Section& sec, const std::string& key, std::uint64_t& out, bool required = false) const {
    double seconds = 0.0;
    if (!read_double(sec, key, seconds, at_least(0.0), required)) {
      return false;
    }
    return convert(sec, key, seconds, out);
  }

  bool convert(Section& sec, const std::string& key, double seconds, std::uint64_t& out) const {
    if (!known()) {
      return false;
    }
    const double ticks = seconds / dt_;
    const double nearest = std::floor(ticks + 0.5);
    if (std::abs(ticks - nearest) > 1e-6 || nearest > 9.0e15) {
      sec.ctx().add(sec.sub(key), sec.line(),
                    "is not a whole number of ticks at dt = " + number_text(dt_) + " s (got " +
                        number_text(seconds) + " s)");
      return false;
    }
    out = static_cast<std::uint64_t>(nearest);
    return true;
  }

 private:
  double dt_;
};

// `scene: {source: file, path: ..., hash: ...}`. The terrain and obstacle keys belong to a
// generated scene and are errors here, because a file defines the whole scene.
void read_scene_file(Section& sec, Scenario& s) {
  s.scene_file.used = true;
  std::string path;
  if (read_text(sec, "path", path)) {
    s.scene_file.path = path;
  } else if (!sec.find("path").IsDefined()) {
    sec.require("path");
  }
  const YAML::Node hash = sec.find("hash");
  if (hash.IsDefined()) {
    std::string text;
    std::uint64_t value = 0;
    if (scalar_text(sec, "hash", hash, text)) {
      if (parse_hash(text, value)) {
        s.scene_file.hash = value;
      } else {
        sec.ctx().add(sec.sub("hash"), line_of(hash),
                      "must be sixteen lowercase hexadecimal digits, got " + text);
      }
    }
  }
  for (const char* key : {"type", "size_m", "cell_m", "amplitude_m", "obstacles"}) {
    const YAML::Node node = sec.find(key);
    if (node.IsDefined()) {
      sec.ctx().add(sec.sub(key), line_of(node), "is not used when the scene comes from a file");
    }
  }
  sec.finish();
}

void read_scene(Section& root, Scenario& s) {
  const YAML::Node node = root.find("scene");
  if (!node.IsDefined()) {
    return;
  }
  Section sec(root.ctx(), node, "scene");
  if (!sec.valid()) {
    return;
  }
  bool from_file = false;
  read_choice<bool>(sec, "source", from_file, {{"generated", false}, {"file", true}});
  if (from_file) {
    read_scene_file(sec, s);
    return;
  }
  read_choice<TerrainType>(sec, "type", s.terrain.type,
                           {{"field", TerrainType::kField}, {"hills", TerrainType::kHills}});
  std::vector<double> size;
  if (read_numbers(sec, "size_m", 2, size)) {
    if (size[0] > 0.0 && size[1] > 0.0) {
      s.terrain.size_x = size[0];
      s.terrain.size_y = size[1];
    } else {
      sec.ctx().add(sec.sub("size_m"), sec.line(), "entries must be greater than 0");
    }
  }
  read_double(sec, "cell_m", s.terrain.cell_size, greater_than(0.0));
  read_double(sec, "amplitude_m", s.terrain.amplitude, at_least(0.0));

  const YAML::Node obstacles = sec.find("obstacles");
  if (obstacles.IsDefined()) {
    Section o(root.ctx(), obstacles, "scene.obstacles");
    if (o.valid()) {
      read_double(o, "density_per_m2", s.obstacles.density_per_m2, at_least(0.0));
      read_double(o, "spawn_clear_radius_m", s.obstacles.spawn_clear_radius, at_least(0.0));
      o.finish();
    }
  }
  sec.finish();
}

void read_swarm(Section& root, Scenario& s) {
  const YAML::Node node = root.find("swarm");
  if (!node.IsDefined()) {
    root.require("swarm");
    return;
  }
  Section sec(root.ctx(), node, "swarm");
  if (!sec.valid()) {
    return;
  }
  read_u32(sec, "drones", s.drones, 1, kMaxDrones, true);

  const YAML::Node formation = sec.find("formation");
  if (formation.IsDefined()) {
    Section f(root.ctx(), formation, "swarm.formation");
    if (f.valid()) {
      read_choice<FormationShape>(f, "shape", s.formation_shape,
                                  {{"line", FormationShape::kLine},
                                   {"wedge", FormationShape::kWedge},
                                   {"grid", FormationShape::kGrid}});
      read_double(f, "spacing_m", s.formation_spacing_m, greater_than(0.0));
      f.finish();
    }
  }
  const YAML::Node start = sec.find("start");
  if (start.IsDefined()) {
    Section st(root.ctx(), start, "swarm.start");
    if (st.valid()) {
      read_u32(st, "columns", s.start.columns, 1, kMaxDrones);
      read_double(st, "spacing_m", s.start.spacing_m, at_least(0.0));
      read_double(st, "jitter_m", s.start.jitter_m, at_least(0.0));
      read_double(st, "altitude_m", s.start.altitude_m, closed(-1.0e6, 1.0e6));
      read_double(st, "altitude_jitter_m", s.start.altitude_jitter_m, at_least(0.0));
      st.finish();
    }
  }
  sec.finish();
}

void read_mission(Section& root, Scenario& s) {
  const YAML::Node node = root.find("mission");
  if (!node.IsDefined()) {
    root.require("mission");
    return;
  }
  Section sec(root.ctx(), node, "mission");
  if (!sec.valid()) {
    return;
  }
  const YAML::Node waypoints = sec.find("waypoints");
  if (!waypoints.IsDefined()) {
    sec.require("waypoints");
  } else if (!waypoints.IsSequence() || waypoints.size() == 0) {
    root.ctx().add("mission.waypoints", line_of(waypoints),
                   "must be a non-empty list of [x, y, z]");
  } else {
    std::vector<Vec3> points;
    bool ok = true;
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
      const YAML::Node& w = waypoints[i];
      double c[3] = {0.0, 0.0, 0.0};
      bool good = w.IsSequence() && w.size() == 3;
      for (std::size_t k = 0; good && k < 3; ++k) {
        good = w[k].IsScalar() && parse_number(w[k].Scalar(), c[k]);
      }
      if (!good) {
        root.ctx().add("mission.waypoints", line_of(w), "each waypoint must be [x, y, z] numbers");
        ok = false;
        continue;
      }
      points.push_back({c[0], c[1], c[2]});
    }
    if (ok) {
      s.mission.waypoints = points;
    }
  }
  read_double(sec, "speed_mps", s.mission.speed, greater_than(0.0));
  read_bool(sec, "loop", s.mission.loop);
  sec.finish();
}

// `dynamics: energy` and its `energy:` overrides, from a version 3 file. With the energy model the
// minimum speed follows from the wing, so a file may not set it.
void read_dynamics(Section& sec, Scenario& s, std::uint64_t version) {
  const YAML::Node dynamics = sec.find("dynamics");
  const YAML::Node energy = sec.find("energy");
  if (!dynamics.IsDefined() && !energy.IsDefined()) {
    return;
  }
  if (version < kScenarioFlightVersion) {
    const YAML::Node at = dynamics.IsDefined() ? dynamics : energy;
    sec.ctx().add(dynamics.IsDefined() ? "vehicle.dynamics" : "vehicle.energy", line_of(at),
                  "needs version " + std::to_string(kScenarioFlightVersion));
    return;
  }
  read_choice<FixedWingDynamics>(
      sec, "dynamics", s.vehicle.dynamics,
      {{"simple", FixedWingDynamics::kSimple}, {"energy", FixedWingDynamics::kEnergy}});
  if (energy.IsDefined()) {
    if (s.vehicle.dynamics != FixedWingDynamics::kEnergy) {
      sec.ctx().add("vehicle.energy", line_of(energy), "needs dynamics: energy");
      return;
    }
    Section e(sec.ctx(), energy, "vehicle.energy");
    if (e.valid()) {
      EnergyParams& p = s.vehicle.energy;
      read_double(e, "mass_kg", p.mass, greater_than(0.0));
      read_double(e, "wing_area_m2", p.wing_area, greater_than(0.0));
      read_double(e, "wingspan_m", p.wingspan, greater_than(0.0));
      read_double(e, "cd0", p.cd0, greater_than(0.0));
      read_double(e, "oswald", p.oswald, Bounds{0.0, 1.0, true, false});
      read_double(e, "cl_max", p.cl_max, greater_than(0.0));
      read_double(e, "air_density_kg_m3", p.air_density, greater_than(0.0));
      read_double(e, "max_thrust_n", p.max_thrust, greater_than(0.0));
      read_double(e, "max_power_w", p.max_power, greater_than(0.0));
      read_double(e, "stall_margin", p.stall_margin, at_least(1.0));
      read_double(e, "max_roll_rate_rad_s", p.max_roll_rate, greater_than(0.0));
      read_double(e, "roll_tau_s", p.roll_tau, greater_than(0.0));
      read_double(e, "pitch_tau_s", p.pitch_tau, greater_than(0.0));
      read_double(e, "max_path_rate_rad_s", p.max_path_rate, greater_than(0.0));
      read_double(e, "max_path_angle_rad", p.max_path_angle, Bounds{0.0, 1.5, true, false});
      e.finish();
    }
  }
}

// `vehicle:` is read only from a version 2 file. `type: quad` takes no other key; `type:
// fixed_wing` takes overrides of the ALTIUS-class defaults, in the units shown by the key names.
void read_vehicle(Section& root, Scenario& s, std::uint64_t version) {
  const YAML::Node node = root.find("vehicle");
  if (!node.IsDefined()) {
    return;
  }
  Section sec(root.ctx(), node, "vehicle");
  if (!sec.valid()) {
    return;
  }
  read_choice<VehicleKind>(sec, "type", s.vehicle.kind,
                           {{"quad", VehicleKind::kQuad}, {"fixed_wing", VehicleKind::kFixedWing}});
  if (s.vehicle.kind == VehicleKind::kFixedWing) {
    FixedWingParams& p = s.vehicle.fixed_wing;
    bool min_speed_given = false;
    if (read_double(sec, "min_speed_mps", p.min_speed, greater_than(0.0))) {
      min_speed_given = true;
    }
    read_double(sec, "max_speed_mps", p.max_speed, greater_than(0.0));
    read_double(sec, "max_accel_mps2", p.max_accel, greater_than(0.0));
    read_double(sec, "max_decel_mps2", p.max_decel, greater_than(0.0));
    double bank_deg = p.max_bank * 180.0 / kPi;
    if (read_double(sec, "max_bank_deg", bank_deg, Bounds{0.0, 89.0, true, false})) {
      p.max_bank = bank_deg * kPi / 180.0;
    }
    read_double(sec, "max_climb_rate_mps", p.max_climb_rate, greater_than(0.0));
    read_double(sec, "max_descent_rate_mps", p.max_descent_rate, greater_than(0.0));
    read_double(sec, "max_vertical_accel_mps2", p.max_vertical_accel, greater_than(0.0));
    read_dynamics(sec, s, version);
    if (s.vehicle.dynamics == FixedWingDynamics::kEnergy) {
      if (min_speed_given) {
        sec.ctx().add("vehicle.min_speed_mps", line_of(sec.find("min_speed_mps")),
                      "with dynamics: energy the minimum speed follows from the wing; set "
                      "vehicle.energy instead");
      }
      p.min_speed = s.vehicle.energy.hold_speed(p.gravity);
    }
  }
  sec.finish();
}

// `environment:` (version 3) holds the wind: a mean vector and gusts.
void read_environment(Section& root, Scenario& s, std::uint64_t version) {
  const YAML::Node node = root.find("environment");
  if (!node.IsDefined()) {
    return;
  }
  if (version < kScenarioFlightVersion) {
    root.ctx().add("environment", line_of(node),
                   "needs version " + std::to_string(kScenarioFlightVersion));
    return;
  }
  Section sec(root.ctx(), node, "environment");
  if (!sec.valid()) {
    return;
  }
  const YAML::Node wind = sec.find("wind");
  if (wind.IsDefined()) {
    Section w(sec.ctx(), wind, "environment.wind");
    if (w.valid()) {
      std::vector<double> mean;
      if (read_numbers(w, "mean_mps", 2, mean)) {
        s.wind.mean = {mean[0], mean[1], 0.0};
      }
      read_double(w, "gust_mps", s.wind.gust_sigma, at_least(0.0));
      read_double(w, "gust_scale_m", s.wind.gust_scale, greater_than(0.0));
      read_double(w, "vertical_ratio", s.wind.vertical_ratio, Bounds{0.0, 1.0, false, false});
      w.finish();
    }
  }
  read_bool(sec, "ground_contact_crashes", s.ground_contact_crashes);
  sec.finish();
}

void read_weather(Section& root, Scenario& s) {
  const YAML::Node node = root.find("weather");
  if (!node.IsDefined()) {
    return;
  }
  Section sec(root.ctx(), node, "weather");
  if (!sec.valid()) {
    return;
  }
  read_double(sec, "visibility_m", s.weather.visibility_m, greater_than(0.0));
  read_double(sec, "rain_mm_h", s.weather.rain_rate_mm_h, at_least(0.0));
  read_double(sec, "fog_constant", s.weather.fog_constant, Bounds{0.0, 20.0, true, false});
  sec.finish();
}

void read_camera(Section& sensors, Scenario& s) {
  const YAML::Node node = sensors.find("camera");
  if (!node.IsDefined()) {
    return;
  }
  if (s.vehicle.kind != VehicleKind::kFixedWing) {
    sensors.ctx().add("sensors.camera", line_of(node), "needs vehicle type fixed_wing");
    return;
  }
  Section c(sensors.ctx(), node, "sensors.camera");
  if (!c.valid()) {
    return;
  }
  CameraFixConfig& f = s.camera;
  read_double(c, "sigma_floor_m", f.sigma_floor_m, greater_than(0.0));
  read_double(c, "sigma_per_m_agl", f.sigma_per_m_agl, greater_than(0.0));
  read_double(c, "min_agl_m", f.min_agl_m, greater_than(0.0));
  read_double(c, "max_agl_m", f.max_agl_m, greater_than(0.0));
  read_double(c, "min_transmission", f.min_transmission, Bounds{0.0, 1.0, true, true});
  read_double(c, "max_sigma_m", f.max_sigma_m, greater_than(0.0));
  read_double(c, "dropout_probability", f.dropout_probability, Bounds{0.0, 1.0, false, true});
  c.finish();
}

// `sensors.radar_altimeter` (version 3): a nadir height sensor on a fixed-wing vehicle.
void read_altimeter(Section& sensors, Scenario& s, std::uint64_t version) {
  const YAML::Node node = sensors.find("radar_altimeter");
  if (!node.IsDefined()) {
    return;
  }
  if (version < kScenarioFlightVersion) {
    sensors.ctx().add("sensors.radar_altimeter", line_of(node),
                      "needs version " + std::to_string(kScenarioFlightVersion));
    return;
  }
  if (s.vehicle.kind != VehicleKind::kFixedWing) {
    sensors.ctx().add("sensors.radar_altimeter", line_of(node), "needs vehicle type fixed_wing");
    return;
  }
  Section a(sensors.ctx(), node, "sensors.radar_altimeter");
  if (!a.valid()) {
    return;
  }
  s.altimeter_used = true;
  RadarAltimeterConfig& c = s.altimeter;
  read_double(a, "sigma_floor_m", c.sigma_floor_m, greater_than(0.0));
  read_double(a, "sigma_per_m", c.sigma_per_m, greater_than(0.0));
  read_double(a, "max_range_m", c.max_range_m, greater_than(0.0));
  read_double(a, "dropout_probability", c.dropout_probability, Bounds{0.0, 1.0, false, true});
  read_u32(a, "period_ticks", c.period_ticks, 1, 1000000);
  a.finish();
}

// `sensors.forward` (version 3): a forward obstacle sensor on a fixed-wing vehicle (ADR 0013).
void read_forward(Section& sensors, Scenario& s, std::uint64_t version) {
  const YAML::Node node = sensors.find("forward");
  if (!node.IsDefined()) {
    return;
  }
  if (version < kScenarioFlightVersion) {
    sensors.ctx().add("sensors.forward", line_of(node),
                      "needs version " + std::to_string(kScenarioFlightVersion));
    return;
  }
  if (s.vehicle.kind != VehicleKind::kFixedWing) {
    sensors.ctx().add("sensors.forward", line_of(node), "needs vehicle type fixed_wing");
    return;
  }
  Section f(sensors.ctx(), node, "sensors.forward");
  if (!f.valid()) {
    return;
  }
  s.forward_used = true;
  ForwardSensorConfig& c = s.forward;
  read_double(f, "horizontal_fov_deg", c.horizontal_fov_deg, Bounds{0.0, 180.0, true, false});
  read_double(f, "vertical_fov_deg", c.vertical_fov_deg, Bounds{0.0, 90.0, false, true});
  read_double(f, "pitch_deg", c.pitch_deg, Bounds{-90.0, 90.0, true, true});
  read_u32(f, "azimuth_rays", c.azimuth_rays, 1, 64);
  read_u32(f, "elevation_rays", c.elevation_rays, 1, 32);
  read_double(f, "max_range_m", c.max_range_m, greater_than(0.0));
  read_double(f, "sigma_floor_m", c.sigma_floor_m, greater_than(0.0));
  read_double(f, "sigma_per_m", c.sigma_per_m, greater_than(0.0));
  read_double(f, "dropout_probability", c.dropout_probability, Bounds{0.0, 1.0, false, true});
  read_double(f, "min_transmission", c.min_transmission, Bounds{0.0, 1.0, true, false});
  read_u32(f, "period_ticks", c.period_ticks, 1, 1000000);
  f.finish();
}

void read_sensors(Section& root, Scenario& s, std::uint64_t version) {
  const YAML::Node node = root.find("sensors");
  if (!node.IsDefined()) {
    return;
  }
  Section sec(root.ctx(), node, "sensors");
  if (!sec.valid()) {
    return;
  }
  const YAML::Node lidar = sec.find("lidar");
  if (lidar.IsDefined() && s.vehicle.kind == VehicleKind::kFixedWing) {
    root.ctx().add("sensors.lidar", line_of(lidar), "a fixed-wing vehicle has no LiDAR");
  } else if (lidar.IsDefined()) {
    Section l(root.ctx(), lidar, "sensors.lidar");
    if (l.valid()) {
      read_u32(l, "beams", s.lidar_beams, 1, 256);
      read_u32(l, "azimuth_samples", s.lidar_azimuth_samples, 1, 4096);
      l.finish();
    }
  }
  read_camera(sec, s);
  read_altimeter(sec, s, version);
  read_forward(sec, s, version);
  read_uint(sec, "fix_period_ticks", s.fix_period_ticks, 1, 1000000);
  sec.finish();
}

void read_jammer(Section& list_owner, const YAML::Node& node, std::size_t index, const Clock& clock,
                 Scenario& s) {
  const std::string path = "comms.jammers[" + std::to_string(index) + "]";
  Section sec(list_owner.ctx(), node, path);
  if (!sec.valid()) {
    return;
  }
  Jammer j;
  std::vector<double> center;
  if (read_numbers(sec, "center", 3, center)) {
    j.center = {center[0], center[1], center[2]};
  }

  const YAML::Node radius = sec.find("radius_m");
  if (!radius.IsDefined()) {
    sec.require("radius_m");
  } else if (radius.IsScalar() && radius.Scalar() == "inf") {
    j.radius = kInf;
  } else {
    double r = 0.0;
    if (!radius.IsScalar() || !parse_number(radius.Scalar(), r) || !(r > 0.0)) {
      list_owner.ctx().add(path + ".radius_m", line_of(radius),
                           "must be inf or a number greater than 0, got " +
                               (radius.IsScalar() ? radius.Scalar() : std::string("a non-value")));
    } else {
      j.radius = r;
    }
  }

  read_choice<JamMode>(sec, "mode", j.mode,
                       {{"constant", JamMode::kConstant}, {"pulsed", JamMode::kPulsed}});
  clock.read(sec, "from_s", j.start_tick);
  clock.read(sec, "to_s", j.end_tick);
  if (j.mode == JamMode::kPulsed) {
    read_uint(sec, "period_ticks", j.pulse_period_ticks, 1, 1000000, true);
    read_double(sec, "duty", j.pulse_duty, Bounds{0.0, 1.0, true, false});
    read_uint(sec, "offset_ticks", j.pulse_offset_ticks, 0, 1000000);
  } else {
    for (const char* key : {"period_ticks", "duty", "offset_ticks"}) {
      if (sec.find(key).IsDefined()) {
        list_owner.ctx().add(path + "." + key, sec.line(), "is only valid for mode pulsed");
      }
    }
  }
  read_double(sec, "loss", j.loss_inside, unit_interval());
  read_choice<JamAffects>(sec, "affects", j.affects,
                          {{"receiver", JamAffects::kReceiver}, {"either", JamAffects::kEither}});
  sec.finish();

  if (j.end_tick < j.start_tick) {
    list_owner.ctx().add(path, sec.line(), "to_s must not be before from_s");
  }
  s.channel.jammers.push_back(j);
}

void read_comms(Section& root, Scenario& s, const Clock& clock) {
  const YAML::Node node = root.find("comms");
  if (!node.IsDefined()) {
    return;
  }
  Section sec(root.ctx(), node, "comms");
  if (!sec.valid()) {
    return;
  }
  read_u32(sec, "latency_ticks", s.channel.latency_ticks, 1, 1000000);
  read_u32(sec, "jitter_ticks", s.channel.jitter_ticks, 0, 1000000);
  read_choice<LossModel>(sec, "loss_model", s.channel.loss_model,
                         {{"none", LossModel::kNone},
                          {"independent", LossModel::kIndependent},
                          {"burst", LossModel::kBurst}});
  read_double(sec, "loss", s.channel.loss, unit_interval());
  read_double(sec, "mean_burst", s.channel.mean_burst, at_least(1.0));
  read_u32(sec, "rate_limit_per_tick", s.channel.max_msgs_per_sender_per_tick, 0, 1000000);

  const YAML::Node jammers = sec.find("jammers");
  if (jammers.IsDefined()) {
    if (!jammers.IsSequence()) {
      root.ctx().add("comms.jammers", line_of(jammers), "must be a list");
    } else {
      for (std::size_t i = 0; i < jammers.size(); ++i) {
        read_jammer(sec, jammers[i], i, clock, s);
      }
    }
  }
  sec.finish();
}

void read_orbit(Section& control, Scenario& s) {
  const YAML::Node node = control.find("orbit");
  if (!node.IsDefined()) {
    return;
  }
  if (s.vehicle.kind != VehicleKind::kFixedWing) {
    control.ctx().add("control.orbit", line_of(node), "needs vehicle type fixed_wing");
    return;
  }
  Section o(control.ctx(), node, "control.orbit");
  if (!o.valid()) {
    return;
  }
  OrbitParams& p = s.orbit;
  read_double(o, "radius_m", p.radius, greater_than(0.0));
  read_double(o, "radius_step_m", p.radius_step, at_least(0.0));
  read_double(o, "altitude_step_m", p.altitude_step, at_least(0.0));
  read_double(o, "phase_gain", p.phase_gain, at_least(0.0));
  read_double(o, "radial_gain", p.radial_gain, at_least(0.0));
  read_double(o, "lookahead_s", p.lookahead_s, greater_than(0.0));
  read_double(o, "consensus_weight", p.consensus_weight, Bounds{0.0, 1.0, false, false});
  double safe = p.separation.d_safe;
  if (read_double(o, "safe_distance_m", safe, greater_than(0.0))) {
    p.separation.d_safe = safe;
    p.separation.max_margin = 2.0 * safe;
  }
  o.finish();
}

// `control.pattern.formation`: the shapes of a formation pattern and when it takes them.
void read_formation_plan(Section& pattern, PatternParams& q) {
  const YAML::Node node = pattern.find("formation");
  if (!node.IsDefined()) {
    return;
  }
  Section f(pattern.ctx(), node, "control.pattern.formation");
  if (!f.valid()) {
    return;
  }
  FormationPlan& plan = q.formation;
  read_double(f, "along_m", plan.along, at_least(0.0));
  read_double(f, "across_m", plan.across, at_least(0.0));
  read_double(f, "transition_s", plan.transition_s, greater_than(0.0));
  read_double(f, "position_gain", plan.position_gain, at_least(0.0));
  read_double(f, "velocity_gain", plan.velocity_gain, at_least(0.0));
  read_double(f, "altitude_stagger_m", plan.altitude_stagger, at_least(0.0));
  const YAML::Node rake = f.find("rake");
  if (rake.IsDefined()) {
    Section r(pattern.ctx(), rake, "control.pattern.formation.rake");
    if (r.valid()) {
      read_double(r, "agl_m", plan.rake_agl, greater_than(0.0));
      read_double(r, "hfov_deg", plan.rake_hfov_deg, Bounds{0.0, 180.0, true, true});
      read_double(r, "overlap", plan.rake_overlap, Bounds{0.0, 1.0, false, true});
      r.finish();
    }
  }
  const YAML::Node phases = f.find("phases");
  if (phases.IsDefined()) {
    if (!phases.IsSequence()) {
      pattern.ctx().add("control.pattern.formation.phases", line_of(phases), "must be a list");
    } else {
      plan.phases.clear();
      for (std::size_t i = 0; i < phases.size(); ++i) {
        Section ph(pattern.ctx(), phases[i],
                   "control.pattern.formation.phases[" + std::to_string(i) + "]");
        if (!ph.valid()) {
          continue;
        }
        FormationPhase phase;
        read_double(ph, "at_s", phase.at_s, at_least(0.0), true);
        read_choice<FormationStyle>(ph, "shape", phase.shape,
                                    {{"trail", FormationStyle::kTrail},
                                     {"echelon_right", FormationStyle::kEchelonRight},
                                     {"echelon_left", FormationStyle::kEchelonLeft},
                                     {"vee", FormationStyle::kVee},
                                     {"rake", FormationStyle::kRake}});
        ph.finish();
        plan.phases.push_back(phase);
      }
    }
  }
  f.finish();
}

// `control.pattern` (version 3): the pattern a fixed-wing vehicle flies. The orbit section still
// gives the center, the layers, the speed, the gains, and the separation.
void read_pattern(Section& control, Scenario& s, std::uint64_t version) {
  const YAML::Node node = control.find("pattern");
  if (!node.IsDefined()) {
    return;
  }
  if (version < kScenarioFlightVersion) {
    control.ctx().add("control.pattern", line_of(node),
                      "needs version " + std::to_string(kScenarioFlightVersion));
    return;
  }
  if (s.vehicle.kind != VehicleKind::kFixedWing) {
    control.ctx().add("control.pattern", line_of(node), "needs vehicle type fixed_wing");
    return;
  }
  Section p(control.ctx(), node, "control.pattern");
  if (!p.valid()) {
    return;
  }
  PatternParams& q = s.pattern;
  read_choice<PatternKind>(p, "type", q.kind,
                           {{"orbit", PatternKind::kOrbit},
                            {"racetrack", PatternKind::kRacetrack},
                            {"lanes", PatternKind::kLanes},
                            {"formation", PatternKind::kFormation}});
  double heading_deg = q.heading * 180.0 / kPi;
  if (read_double(p, "heading_deg", heading_deg, Bounds{-360.0, 360.0, false, false})) {
    q.heading = heading_deg * kPi / 180.0;
  }
  read_double(p, "leg_length_m", q.leg_length, at_least(0.0));
  read_double(p, "turn_radius_m", q.turn_radius, greater_than(0.0));
  read_double(p, "lane_spacing_m", q.lane_spacing, greater_than(0.0));
  read_double(p, "stagger_s", q.stagger_s, at_least(0.0));
  read_double(p, "join_fraction", q.join_fraction, greater_than(0.0));
  read_formation_plan(p, q);
  p.finish();
}

// `control.terrain` (version 3): terrain-aware altitude for orbit guidance.
// One rectangle of `control.terrain.omit`.
void read_omit(Section& list_owner, const YAML::Node& node, std::size_t index, Scenario& s) {
  const std::string path = "control.terrain.omit[" + std::to_string(index) + "]";
  Section r(list_owner.ctx(), node, path);
  if (!r.valid()) {
    return;
  }
  std::vector<double> x;
  std::vector<double> y;
  const bool have_x = read_numbers(r, "x_m", 2, x, true);
  const bool have_y = read_numbers(r, "y_m", 2, y, true);
  r.finish();
  if (!have_x || !have_y) {
    return;
  }
  if (!(x[0] < x[1]) || !(y[0] < y[1])) {
    list_owner.ctx().add(path, r.line(), "x_m and y_m must each be [min, max] with min below max");
    return;
  }
  s.map_faults.omit.push_back({x[0], x[1], y[0], y[1]});
}

void read_terrain_guidance(Section& control, Scenario& s, std::uint64_t version) {
  const YAML::Node node = control.find("terrain");
  if (!node.IsDefined()) {
    return;
  }
  if (version < kScenarioFlightVersion) {
    control.ctx().add("control.terrain", line_of(node),
                      "needs version " + std::to_string(kScenarioFlightVersion));
    return;
  }
  if (s.vehicle.kind != VehicleKind::kFixedWing) {
    control.ctx().add("control.terrain", line_of(node), "needs vehicle type fixed_wing");
    return;
  }
  Section t(control.ctx(), node, "control.terrain");
  if (!t.valid()) {
    return;
  }
  TerrainGuidanceParams& p = s.terrain_guidance;
  p.enabled = true;
  read_double(t, "clearance_m", p.clearance, at_least(0.0));
  read_double(t, "footprint_m", p.footprint, at_least(0.0));
  read_double(t, "climb_fraction", p.climb_fraction, Bounds{0.0, 1.0, true, false});
  read_double(t, "sample_spacing_m", p.sample_spacing, greater_than(0.0));
  read_double(t, "lookahead_m", p.lookahead, greater_than(0.0));
  // Errors of the prior map (ADR 0013).
  TerrainMapFaults& faults = s.map_faults;
  read_double(t, "offset_m", faults.offset_m, at_least(-1.0e6));
  read_double(t, "error_sigma_m", faults.error_sigma_m, at_least(0.0));
  read_double(t, "error_correlation_m", faults.error_correlation_m, greater_than(0.0));
  const YAML::Node omit = t.find("omit");
  if (omit.IsDefined()) {
    if (!omit.IsSequence()) {
      control.ctx().add("control.terrain.omit", line_of(omit), "must be a list");
    } else {
      for (std::size_t i = 0; i < omit.size(); ++i) {
        read_omit(t, omit[i], i, s);
      }
    }
  }
  t.finish();
}

// `control.avoid` (version 3): turning and slowing for what climbing cannot clear (ADR 0014).
void read_avoid(Section& control, Scenario& s, std::uint64_t version) {
  const YAML::Node node = control.find("avoid");
  if (!node.IsDefined()) {
    return;
  }
  if (version < kScenarioFlightVersion) {
    control.ctx().add("control.avoid", line_of(node),
                      "needs version " + std::to_string(kScenarioFlightVersion));
    return;
  }
  if (s.vehicle.kind != VehicleKind::kFixedWing) {
    control.ctx().add("control.avoid", line_of(node), "needs vehicle type fixed_wing");
    return;
  }
  if (!s.forward_used) {
    control.ctx().add("control.avoid", line_of(node), "needs sensors.forward");
    return;
  }
  Section a(control.ctx(), node, "control.avoid");
  if (!a.valid()) {
    return;
  }
  s.avoid.enabled = true;
  read_double(a, "horizon_s", s.avoid.horizon_s, greater_than(0.0));
  read_double(a, "hold_s", s.avoid.hold_s, greater_than(0.0));
  read_double(a, "bank_fraction", s.avoid.bank_fraction, Bounds{0.0, 1.0, true, false});
  read_bool(a, "rejoin", s.avoid.rejoin);
  read_choice<AvoidFeasibility>(
      a, "feasibility", s.avoid.feasibility,
      {{"budget", AvoidFeasibility::kBudget}, {"kinematic", AvoidFeasibility::kKinematic}});
  read_double(a, "urgent_s", s.avoid.urgent_s, at_least(0.0));
  read_bool(a, "turn_first", s.avoid.turn_first);
  double clearance = 0.0;
  if (read_double(a, "clearance_m", clearance, at_least(0.0))) {
    s.avoid.clearance_m = clearance;
  }
  a.finish();
}

// `control.share` (version 3): sharing what the forward sensor sees (ADR 0016).
void read_share(Section& control, Scenario& s, std::uint64_t version) {
  const YAML::Node node = control.find("share");
  if (!node.IsDefined()) {
    return;
  }
  if (version < kScenarioFlightVersion) {
    control.ctx().add("control.share", line_of(node),
                      "needs version " + std::to_string(kScenarioFlightVersion));
    return;
  }
  if (!s.avoid.enabled) {
    control.ctx().add("control.share", line_of(node), "needs control.avoid");
    return;
  }
  Section sh(control.ctx(), node, "control.share");
  if (!sh.valid()) {
    return;
  }
  s.share.mode = ShareMode::kShare;
  read_choice<ShareMode>(
      sh, "mode", s.share.mode,
      {{"off", ShareMode::kOff}, {"memory", ShareMode::kMemory}, {"share", ShareMode::kShare}});
  read_double(sh, "ttl_s", s.share.ttl_s, greater_than(0.0));
  read_u32(sh, "points_per_message", s.share.points_per_message, 1,
           static_cast<std::uint32_t>(kMaxHazardPoints));
  std::uint32_t period = 0;
  if (read_u32(sh, "period_ticks", period, 1, 1000000)) {
    s.share.period_ticks = period;
  }
  read_double(sh, "repeat_s", s.share.repeat_s, greater_than(0.0));
  read_u32(sh, "min_sources", s.share.min_sources, 1, 64);
  sh.finish();
}

void read_control(Section& root, Scenario& s, std::uint64_t version) {
  const YAML::Node node = root.find("control");
  if (!node.IsDefined()) {
    return;
  }
  Section sec(root.ctx(), node, "control");
  if (!sec.valid()) {
    return;
  }
  read_choice<PredictorChoice>(sec, "predictor", s.predictor,
                               {{"kalman", PredictorChoice::kKalman},
                                {"dead_reckoning", PredictorChoice::kDeadReckoning},
                                {"turning", PredictorChoice::kTurning},
                                {"hold_last", PredictorChoice::kHoldLast},
                                {"oracle", PredictorChoice::kOracle}});
  read_bool(sec, "fallback", s.fallback_enabled);
  read_bool(sec, "separation", s.separation_enabled);
  read_orbit(sec, s);
  read_terrain_guidance(sec, s, version);
  read_avoid(sec, s, version);
  read_share(sec, s, version);
  read_pattern(sec, s, version);
  read_uint(sec, "report_period_ticks", s.report_period_ticks, 1, 1000000);
  read_uint(sec, "staleness_ticks", s.staleness_ticks, 1, 1000000);
  read_uint(sec, "drop_stale_after_ticks", s.drop_stale_after_ticks, 0, 1000000);
  sec.finish();
}

void read_faults(Section& root, Scenario& s, const Clock& clock) {
  const YAML::Node node = root.find("faults");
  if (!node.IsDefined()) {
    return;
  }
  if (!node.IsSequence()) {
    root.ctx().add("faults", line_of(node), "must be a list");
    return;
  }
  std::set<std::uint32_t> seen;
  for (std::size_t i = 0; i < node.size(); ++i) {
    const std::string path = "faults[" + std::to_string(i) + "]";
    Section sec(root.ctx(), node[i], path);
    if (!sec.valid()) {
      continue;
    }
    Fault f;
    const bool has_drone = read_u32(sec, "drone", f.drone, 0, kMaxDrones - 1, true);
    const bool has_time = clock.read(sec, "at_s", f.tick, true);
    read_choice<FaultKind>(
        sec, "kind", f.kind,
        {{"fail", FaultKind::kFail}, {"false_hazards", FaultKind::kFalseHazards}});
    sec.finish();
    if (!has_drone || !has_time) {
      continue;
    }
    if (s.drones > 0 && f.drone >= s.drones) {
      root.ctx().add(path + ".drone", sec.line(),
                     "must be below the number of drones (" + std::to_string(s.drones) + ")");
      continue;
    }
    if (!seen.insert(f.drone).second) {
      root.ctx().add(path + ".drone", sec.line(),
                     "drone " + std::to_string(f.drone) + " already fails");
      continue;
    }
    s.faults.push_back(f);
  }
}

void read_commands(Section& root, Scenario& s, const Clock& clock) {
  const YAML::Node node = root.find("commands");
  if (!node.IsDefined()) {
    return;
  }
  if (!node.IsSequence()) {
    root.ctx().add("commands", line_of(node), "must be a list");
    return;
  }
  if (node.size() > kMaxCommands) {
    root.ctx().add("commands", line_of(node),
                   "has too many entries (at most " + std::to_string(kMaxCommands) + ")");
    return;
  }
  for (std::size_t i = 0; i < node.size(); ++i) {
    const std::string path = "commands[" + std::to_string(i) + "]";
    Section sec(root.ctx(), node[i], path);
    if (!sec.valid()) {
      continue;
    }
    ScenarioCommand c;
    const bool has_time = clock.read(sec, "at_s", c.tick, true);
    bool has_kind = read_choice<CommandKind>(
        sec, "kind", c.kind, {{"hold", CommandKind::kHold}, {"resume", CommandKind::kResume}});
    if (!has_kind && !sec.find("kind").IsDefined()) {
      sec.require("kind");
    }
    sec.finish();
    if (has_time && has_kind) {
      s.commands.push_back(c);
    }
  }
  // Authors may list commands in any order; the station sends them in time order, and commands at
  // the same time keep the order they were written in.
  std::stable_sort(
      s.commands.begin(), s.commands.end(),
      [](const ScenarioCommand& a, const ScenarioCommand& b) { return a.tick < b.tick; });
}

void read_metrics(Section& root, Scenario& s) {
  const YAML::Node node = root.find("metrics");
  if (!node.IsDefined()) {
    return;
  }
  Section sec(root.ctx(), node, "metrics");
  if (!sec.valid()) {
    return;
  }
  read_double(sec, "settle_s", s.metrics.settle_s, at_least(0.0));
  read_double(sec, "collision_radius_m", s.metrics.collision_radius_m, greater_than(0.0));
  read_double(sec, "recovery_threshold_m", s.metrics.recovery_threshold_m, greater_than(0.0));
  read_double(sec, "recovery_hold_s", s.metrics.recovery_hold_s, at_least(0.0));
  sec.finish();
}

void read_log(Section& root, Scenario& s) {
  const YAML::Node node = root.find("log");
  if (!node.IsDefined()) {
    return;
  }
  Section sec(root.ctx(), node, "log");
  if (!sec.valid()) {
    return;
  }
  read_uint(sec, "snapshot_every_ticks", s.snapshot_every_ticks, 1, 1000000);
  sec.finish();
}

// Checks that the existing validators of the model structs accept what was read, so a loaded
// scenario cannot construct an invalid simulation.
template <typename Check>
void validate_with(Context& ctx, const std::string& path, int line, Check&& check) {
  try {
    check();
  } catch (const std::exception& e) {
    ctx.add(path, line, e.what());
  }
}

void cross_validate(Context& ctx, const Scenario& s) {
  if (!s.scene_file.used) {
    validate_with(ctx, "scene", 0, [&] { validate(s.terrain); });
    validate_with(ctx, "scene.obstacles", 0, [&] { validate(s.obstacles); });
  }
  validate_with(ctx, "weather", 0, [&] { s.weather.validate(); });
  validate_with(ctx, "environment.wind", 0, [&] { validate(s.wind); });
  if (s.terrain_guidance.enabled) {
    validate_with(ctx, "control.terrain", 0, [&] {
      TerrainGuidanceParams p = s.terrain_guidance;
      p.climb_rate = terrain_climb_rate(s);
      if (!(p.climb_rate > 0.0)) {
        throw std::invalid_argument(
            "the aircraft cannot climb at the mission speed with the thrust and power of the "
            "energy model, so terrain guidance has no climb to plan with");
      }
      validate(p);
    });
  }
  if (s.vehicle.kind == VehicleKind::kFixedWing) {
    validate_with(ctx, "vehicle", 0, [&] { validate(s.vehicle.fixed_wing); });
    if (s.vehicle.dynamics == FixedWingDynamics::kEnergy) {
      validate_with(ctx, "vehicle.energy", 0, [&] { validate(s.vehicle.energy); });
    }
    validate_with(ctx, "sensors.camera", 0, [&] { CameraFixSensor sensor(s.camera, 0, 0); });
    if (s.mission.waypoints.size() != 1) {
      ctx.add("mission.waypoints", 0,
              "a fixed-wing vehicle orbits the first waypoint, so there must be exactly one");
    } else {
      if (s.pattern.kind == PatternKind::kFormation) {
        validate_with(ctx, "control.pattern", 0, [&] {
          const OrbitParams o = orbit_params(s);
          validate(o);
          validate(s.pattern);
          const FixedWingParams& wing = s.vehicle.fixed_wing;
          // The widest the formation gets, to the side of the lead, bounds the speeds of the
          // wingmen when the lead turns: the inside ones slow down and the outside ones speed up.
          const FormationPlan& plan = s.pattern.formation;
          bool rake = false;
          for (const FormationPhase& phase : plan.phases) {
            rake = rake || phase.shape == FormationStyle::kRake;
          }
          const double side = rake ? std::max(plan.across, rake_spacing(plan)) : plan.across;
          const double widest = static_cast<double>(s.drones - 1U) * side;
          const double radius = s.pattern.turn_radius;
          const double inner = o.speed * (radius - widest) / radius;
          const double outer = o.speed * (radius + widest) / radius;
          if (!(radius > widest) || inner < wing.min_speed + o.speed_margin ||
              outer > wing.max_speed - o.speed_margin) {
            throw std::invalid_argument(
                "the lead's turn radius is too small for the width of the formation: a wingman "
                "on the inside would fly under the minimum speed or one on the outside over the "
                "maximum");
          }
          if (o.speed * o.speed / radius > 0.8 * wing.max_lateral_accel()) {
            throw std::invalid_argument(
                "the lead's turn needs more than 80 percent of the bank limit");
          }
          // Neighbors must hold outside the separation margin.
          const double margin = o.separation.max_margin;
          bool trail = plan.phases.empty();
          for (const FormationPhase& phase : plan.phases) {
            trail = trail || phase.shape == FormationStyle::kTrail;
          }
          if (s.drones > 1 && rake && rake_spacing(plan) <= margin) {
            throw std::invalid_argument("the rake spacing must exceed the separation margin of " +
                                        std::to_string(margin) + " m");
          }
          if (s.drones > 1 && (std::hypot(plan.along, plan.across) <= margin ||
                               (s.drones > 2 && 2.0 * plan.across <= margin) ||
                               (trail && s.pattern.stagger_s * o.speed <= margin))) {
            throw std::invalid_argument(
                "formation spacing (along, across, or stagger_s times the speed) must exceed the "
                "separation margin of " +
                std::to_string(margin) + " m");
          }
        });
      } else if (s.pattern.kind != PatternKind::kOrbit) {
        validate_with(ctx, "control.pattern", 0, [&] {
          const OrbitParams o = orbit_params(s);
          validate(o);
          validate(s.pattern);
          const double a_limit = 0.8 * s.vehicle.fixed_wing.max_lateral_accel();
          for (std::uint32_t i = 0; i < s.drones; ++i) {
            const PatternReference ref = pattern_reference(i, s.drones, 0.0, s.pattern, o);
            const double radius = pattern_path(i, s.drones, s.pattern, o).radius();
            if (ref.speed * ref.speed / radius > a_limit) {
              throw std::invalid_argument(
                  "a turn of the pattern needs more than 80 percent of the bank limit");
            }
            if (ref.speed > s.vehicle.fixed_wing.max_speed - o.speed_margin ||
                ref.speed < s.vehicle.fixed_wing.min_speed + o.speed_margin) {
              throw std::invalid_argument("a slot's speed on the pattern is outside the limits");
            }
          }
          if (s.pattern.kind == PatternKind::kLanes && s.drones > 1 &&
              s.pattern.lane_spacing < 2.0 * s.pattern.turn_radius + o.separation.d_safe) {
            throw std::invalid_argument(
                "lane_spacing must exceed twice the turn radius plus the safe distance");
          }
        });
      } else
        validate_with(ctx, "control.orbit", 0, [&] {
          const OrbitParams o = orbit_params(s);
          validate(o);
          // The aircraft must be able to fly the circle it is asked to: the sideways acceleration
          // of the outer circle, omega^2 * (radius + step), within 80 percent of the bank limit,
          // and its speed within the speed limit.
          const double omega = o.speed / o.radius;
          const double outer = o.radius + (s.drones > 1 ? o.radius_step : 0.0);
          if (omega * omega * outer > 0.8 * s.vehicle.fixed_wing.max_lateral_accel()) {
            throw std::invalid_argument("the orbit needs more than 80 percent of the bank limit");
          }
          if (omega * outer > s.vehicle.fixed_wing.max_speed - o.speed_margin) {
            throw std::invalid_argument("the outer orbit needs more than the maximum speed");
          }
        });
    }
  }
  validate_with(ctx, "mission", 0, [&] { MissionPath path(s.mission); });
  validate_with(ctx, "comms", 0, [&] {
    ChannelConfig c = s.channel;
    DeterministicChannel channel(s.drones, c);
  });
  validate_with(ctx, "sensors.lidar", 0, [&] {
    LidarConfig c;
    c.beams = s.lidar_beams;
    c.azimuth_samples = s.lidar_azimuth_samples;
    Lidar lidar(c, 0, 0);
  });
}

}  // namespace

LoadResult load_scenario(const std::string& text, const std::string& default_name) {
  LoadResult result;
  Scenario& s = result.scenario;
  s.name = default_name;
  Context ctx;

  YAML::Node doc;
  try {
    doc = YAML::Load(text);
  } catch (const YAML::Exception& e) {
    ctx.add("", e.mark.is_null() ? 0 : e.mark.line + 1, std::string("malformed YAML: ") + e.msg);
    result.errors = ctx.errors;
    return result;
  }
  if (!doc.IsMap()) {
    ctx.add("", line_of(doc), "the document must be a mapping");
    result.errors = ctx.errors;
    return result;
  }

  Section root(ctx, doc, "");
  std::uint64_t version = 0;
  if (read_uint(root, "version", version, 0, std::numeric_limits<std::uint64_t>::max(), true) &&
      (version < 1 || version > kScenarioVersion)) {
    ctx.add("version", line_of(root.find("version")),
            "unsupported version " + std::to_string(version) + "; this build reads versions 1 to " +
                std::to_string(kScenarioVersion));
  }
  read_text(root, "name", s.name);
  read_uint(root, "seed", s.seed, 0, std::numeric_limits<std::uint64_t>::max(), true);
  const bool dt_given = root.find("dt").IsDefined();
  const bool dt_valid = read_double(root, "dt", s.dt, Bounds{0.0, 1.0, true, false});
  // Times are converted with dt; if dt was given and is invalid, they are skipped, not misread.
  const Clock clock(dt_valid || !dt_given ? s.dt : 0.0);

  double duration_s = 0.0;
  if (read_double(root, "duration_s", duration_s, greater_than(0.0), true)) {
    if (!clock.convert(root, "duration_s", duration_s, s.duration_ticks)) {
      s.duration_ticks = 0;
    } else if (s.duration_ticks == 0) {
      ctx.add("duration_s", line_of(root.find("duration_s")), "must be at least one tick");
    }
  }

  read_scene(root, s);
  read_swarm(root, s);
  read_mission(root, s);
  if (version >= kScenarioVehicleVersion) {
    read_vehicle(root, s, version);
  } else if (root.find("vehicle").IsDefined()) {
    ctx.add("vehicle", line_of(root.find("vehicle")),
            "needs version " + std::to_string(kScenarioVehicleVersion));
  }
  read_environment(root, s, version);
  read_weather(root, s);
  read_sensors(root, s, version);
  read_comms(root, s, clock);
  read_control(root, s, version);
  read_faults(root, s, clock);
  read_commands(root, s, clock);
  read_metrics(root, s);
  read_log(root, s);
  root.finish();

  if (ctx.errors.empty()) {
    cross_validate(ctx, s);
  }

  std::stable_sort(ctx.errors.begin(), ctx.errors.end(),
                   [](const ScenarioError& a, const ScenarioError& b) { return a.line < b.line; });
  result.errors = ctx.errors;
  return result;
}

}  // namespace swarm
