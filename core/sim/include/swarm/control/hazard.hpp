#pragma once
// Controller headers may include only this directory, swarm/estimation/, sensor_data.hpp,
// vec3.hpp, drone_command.hpp, message.hpp, and swarm/format/{byte_io,hash}.hpp. They must
// never include vehicle truth.
// Enforced by scripts/check_core_invariants.sh.

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "swarm/format/hash.hpp"
#include "swarm/vec3.hpp"

namespace swarm {

// Sharing what the forward sensor sees (ADR 0016).
enum class ShareMode {
  kOff,     // nothing is stored, sent, or received
  kMemory,  // the aircraft's own returns are stored and used; nothing is sent or received
  kShare,   // memory, and reports are sent and received
};

struct ShareParams {
  ShareMode mode = ShareMode::kOff;
  // A stored point expires this long after it was last seen.
  double ttl_s = 120.0;
  // At most this many points in a report, at most kMaxHazardPoints.
  std::uint32_t points_per_message = 8;
  // A report is sent on ticks divisible by this. 0 means the neighbor report period; the scenario
  // builder fills it in.
  std::uint64_t period_ticks = 0;
  // A point that was seen is sent again in every report for this long after it was last seen.
  double repeat_s = 2.0;
  // A point reported by others is used when at least this many distinct aircraft reported it, at
  // the height that the min_sources-th highest of their reports gives; the aircraft's own sensing
  // is always trusted. 1 uses any report.
  std::uint32_t min_sources = 1;
  // The width of a cell of the store and the most cells it holds.
  double cell_m = 20.0;
  std::size_t max_cells = 512;
  // The standard deviation of a returned point, meters, added in quadrature to the sender's own
  // position sigma for a point that was received.
  double point_sigma_m = 2.0;
  // The sensor's horizontal field of view and range, copied by the scenario builder: stored points
  // outside the field raise no altitude floor, and none beyond the range does either.
  double fov_deg = 40.0;
  double max_range_m = 800.0;
  // A faulty sender: from this tick on the aircraft sends false points (see make_false_report).
  std::optional<std::uint64_t> false_from_tick;
  // The seed that picks where the false points are, set by the scenario builder.
  std::uint64_t false_seed = 0;
};

// Throws std::invalid_argument unless the times are finite and positive, the number of points is
// in [1, kMaxHazardPoints], the cell, point sigma, field, and range are finite and positive, the
// field is at most 180 degrees, min_sources is at least 1, and the store holds at least one cell.
void validate(const ShareParams& params);

// A point of the ground that matters at flight level, as the sensor returned it, and how well it
// is known.
struct HazardPoint {
  Vec3 position;
  double sigma = 0.0;
};

inline constexpr std::size_t kMaxHazardPoints = 8;
inline constexpr std::uint8_t kHazardReportVersion = 1;

struct HazardReport {
  std::uint64_t send_tick = 0;
  double sender_sigma = 0.0;  // the sender's per-axis position uncertainty, meters
  std::vector<Vec3> points;
};

inline constexpr std::size_t kHazardHeaderBytes = 1 + 8 + 4 + 1;
inline constexpr std::size_t kHazardPointBytes = 3 * 4;

// Layout, little-endian: u8 version | u64 send_tick | f32 sender_sigma | u8 count | count x (f32 x,
// f32 y, f32 z). At most kMaxHazardPoints, so at most 110 bytes. Positions are rounded to single
// precision. Throws std::invalid_argument for too many points, a value that is not finite or does
// not fit a single, or a negative sigma.
std::vector<std::uint8_t> encode_hazard_report(const HazardReport& report);

enum class HazardStatus {
  kOk,
  kBadSize,
  kBadVersion,
  kTooMany,       // the count is above kMaxHazardPoints
  kInvalidValue,  // a value is not finite, or the sigma is negative
};

// On any status but kOk, `out` is left unchanged.
HazardStatus decode_hazard_report(const std::uint8_t* data, std::size_t size, HazardReport& out);

// The report a faulty sender makes (ADR 0016, decision 7): `count` points spread over a 90 degree
// arc of the circle of `radius` around `center`, at height `z`, starting at an angle that is a
// function of `seed`, `sender`, and `index` (the number of the report), so it is the same on every
// platform and every run. The points are where they block the others' path.
std::vector<Vec3> make_false_points(const Vec3& center, double radius, double z, std::size_t count,
                                    std::uint64_t seed, std::uint32_t sender, std::uint64_t index);

// What an aircraft keeps of the hazards it knows of, one point for each cell of a grid (decision
// 3). A cell holds the aircraft's own highest point and, separately, the highest point of each
// other aircraft that reported it; the points are used as the `min_sources` rule says. The store is
// a map keyed by cell, so its order, and every result that depends on it, is a function of the
// data.
class HazardStore {
 public:
  // Throws std::invalid_argument for invalid parameters.
  explicit HazardStore(const ShareParams& params, std::uint64_t ttl_ticks);

  void add_own(const Vec3& point, double sigma, std::uint64_t tick);
  void add_remote(std::uint32_t sender, const Vec3& point, double sigma, std::uint64_t tick);
  // Drops what was last seen more than the ttl before `tick`, and then the oldest cells beyond the
  // bound.
  void expire(std::uint64_t tick);
  // The points that may be used now, in cell order. For each cell, the aircraft's own point or the
  // point that `min_sources` other aircraft support, whichever is higher.
  std::vector<HazardPoint> usable() const;

  std::size_t cells() const { return cells_.size(); }
  void hash_state(Fnv1a64& hash) const;

 private:
  struct Entry {
    Vec3 position;
    double sigma = 0.0;
    std::uint64_t tick = 0;
  };
  struct Cell {
    std::optional<Entry> own;
    std::map<std::uint32_t, Entry> remote;
    std::uint64_t last_tick = 0;
  };
  using Key = std::pair<std::int64_t, std::int64_t>;

  Key key_of(const Vec3& point) const;
  static void keep_highest(std::optional<Entry>& slot, const Entry& entry);

  ShareParams params_;
  std::uint64_t ttl_ticks_;
  std::map<Key, Cell> cells_;
};

}  // namespace swarm
