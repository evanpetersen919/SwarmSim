#pragma once
// Controller headers may include only this directory, swarm/estimation/, sensor_data.hpp,
// vec3.hpp, drone_command.hpp, message.hpp, and swarm/format/{byte_io,hash}.hpp. They must
// never include vehicle truth.
// Enforced by scripts/check_core_invariants.sh.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "swarm/vec3.hpp"

namespace swarm {

// The elevation data an aircraft carries (ADR 0011): a copy of the scene's heightfield, treated
// as prior map data and not as a sensor. It is exact by default; a scenario that wants a map with
// errors would add them when it builds the map. It is built by the scenario world from the scene,
// because controllers do not include the scene.
//
// Samples sit at (origin_x + i * cell_size, origin_y + j * cell_size), x varying fastest, as in
// swarm/format/scene.hpp. A query outside the grid uses the nearest point of the grid boundary.
class TerrainMap {
 public:
  // Throws std::invalid_argument unless the grid has at least one cell on each axis, the cell size
  // is positive and finite, `heights` holds (cells_x + 1) * (cells_y + 1) finite values, and the
  // origin is finite.
  TerrainMap(std::uint32_t cells_x, std::uint32_t cells_y, double origin_x, double origin_y,
             double cell_size, std::vector<double> heights);

  // The highest sample whose cell touches the disc of radius `radius` around (x, y). The terrain
  // is a triangle mesh whose height never exceeds the highest corner of its cell, so no ground in
  // the disc is higher than the result. Returns NaN for a non-finite argument or a negative
  // radius.
  double max_height_near(double x, double y, double radius) const;

  double cell_size() const { return cell_size_; }

 private:
  std::uint32_t cells_x_;
  std::uint32_t cells_y_;
  double origin_x_;
  double origin_y_;
  double cell_size_;
  std::vector<double> heights_;
};

// How the altitude command is raised over terrain.
struct TerrainGuidanceParams {
  bool enabled = false;
  double clearance = 60.0;       // m above the highest ground in the footprint
  double footprint = 40.0;       // m, radius around each path point that is checked
  double climb_rate = 3.0;       // m/s, the vehicle's climb limit; set by the scenario builder
  double climb_fraction = 0.6;   // the part of it that a planned climb may use
  double sample_spacing = 20.0;  // m along the path between checks
  double lookahead = 600.0;      // m of straight path ahead that is checked
};

// Throws std::invalid_argument unless every value is finite, the clearance is not negative, the
// footprint is not negative, the climb rate and spacing and lookahead are positive, and the climb
// fraction is in (0, 1].
void validate(const TerrainGuidanceParams& params);

// A point on a planned path and the distance along the path to it from the aircraft.
struct PathSample {
  double x = 0.0;
  double y = 0.0;
  double distance = 0.0;
};

// The lowest altitude that clears the terrain along `path` for an aircraft flying at `speed`:
//   max over samples of  highest ground near the sample + clearance - climb_budget * distance /
//   speed
// where climb_budget = climb_fraction * climb_rate. A peak far ahead lowers the floor by the
// height that a climb at the budget rate gains before reaching it, so the aircraft starts to climb
// early enough and no earlier. Returns -infinity for an empty path. Throws std::invalid_argument
// for invalid parameters or a non-positive or non-finite speed.
double terrain_floor(const TerrainMap& map, const TerrainGuidanceParams& params,
                     const std::vector<PathSample>& path, double speed);

// The samples of the straight line from `position` along the unit horizontal direction
// (dir_x, dir_y), `params.lookahead` long, `params.sample_spacing` apart, starting at the
// position. A zero direction gives the position alone.
std::vector<PathSample> straight_path(const Vec3& position, double dir_x, double dir_y,
                                      const TerrainGuidanceParams& params);

// The samples of a counterclockwise circle of `radius` around (cx, cy) from the point of the
// circle nearest the position, once around, `params.sample_spacing` apart. The distance counts
// from the position to that point and then along the arc.
std::vector<PathSample> circle_path(const Vec3& position, double cx, double cy, double radius,
                                    const TerrainGuidanceParams& params);

}  // namespace swarm
