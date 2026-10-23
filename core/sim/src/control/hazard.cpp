#include "swarm/control/hazard.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "swarm/format/byte_io.hpp"

namespace swarm {

namespace {

constexpr double kPi = 3.14159265358979323846;

bool finite(const Vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool fits_single(double v) {
  return std::isfinite(v) && std::abs(v) <= static_cast<double>(std::numeric_limits<float>::max());
}

std::uint64_t mix(std::uint64_t x) {
  x += 0x9E3779B97F4A7C15ULL;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
  return x ^ (x >> 31);
}

}  // namespace

void validate(const ShareParams& p) {
  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  if (!positive(p.ttl_s) || !positive(p.repeat_s) || !positive(p.cell_m) ||
      !positive(p.point_sigma_m) || !positive(p.fov_deg) || !positive(p.max_range_m)) {
    throw std::invalid_argument(
        "ShareParams: times, cell, sigma, field, and range must be positive");
  }
  if (p.fov_deg > 180.0) {
    throw std::invalid_argument("ShareParams: the field of view must be at most 180 degrees");
  }
  if (p.points_per_message < 1 || p.points_per_message > kMaxHazardPoints) {
    throw std::invalid_argument("ShareParams: points_per_message must be in [1, 8]");
  }
  if (p.min_sources < 1) {
    throw std::invalid_argument("ShareParams: min_sources must be at least 1");
  }
  if (p.max_cells < 1) {
    throw std::invalid_argument("ShareParams: the store must hold at least one cell");
  }
}

std::vector<std::uint8_t> encode_hazard_report(const HazardReport& report) {
  if (report.points.size() > kMaxHazardPoints) {
    throw std::invalid_argument("encode_hazard_report: too many points");
  }
  if (!fits_single(report.sender_sigma) || report.sender_sigma < 0.0) {
    throw std::invalid_argument("encode_hazard_report: the sigma must be finite and not negative");
  }
  for (const Vec3& p : report.points) {
    if (!fits_single(p.x) || !fits_single(p.y) || !fits_single(p.z)) {
      throw std::invalid_argument("encode_hazard_report: a point is not a finite single");
    }
  }
  std::vector<std::uint8_t> out;
  out.reserve(kHazardHeaderBytes + kHazardPointBytes * report.points.size());
  detail::put_u8(out, kHazardReportVersion);
  detail::put_u64(out, report.send_tick);
  detail::put_f32(out, static_cast<float>(report.sender_sigma));
  detail::put_u8(out, static_cast<std::uint8_t>(report.points.size()));
  for (const Vec3& p : report.points) {
    detail::put_f32(out, static_cast<float>(p.x));
    detail::put_f32(out, static_cast<float>(p.y));
    detail::put_f32(out, static_cast<float>(p.z));
  }
  return out;
}

HazardStatus decode_hazard_report(const std::uint8_t* data, std::size_t size, HazardReport& out) {
  if (size < kHazardHeaderBytes) {
    return HazardStatus::kBadSize;
  }
  if (data[0] != kHazardReportVersion) {
    return HazardStatus::kBadVersion;
  }
  const std::size_t count = data[13];
  if (count > kMaxHazardPoints) {
    return HazardStatus::kTooMany;
  }
  if (size != kHazardHeaderBytes + kHazardPointBytes * count) {
    return HazardStatus::kBadSize;
  }
  HazardReport report;
  report.send_tick = detail::get_u64(data + 1);
  report.sender_sigma = static_cast<double>(detail::get_f32(data + 9));
  if (!std::isfinite(report.sender_sigma) || report.sender_sigma < 0.0) {
    return HazardStatus::kInvalidValue;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const std::uint8_t* p = data + kHazardHeaderBytes + kHazardPointBytes * i;
    const Vec3 point{static_cast<double>(detail::get_f32(p)),
                     static_cast<double>(detail::get_f32(p + 4)),
                     static_cast<double>(detail::get_f32(p + 8))};
    if (!finite(point)) {
      return HazardStatus::kInvalidValue;
    }
    report.points.push_back(point);
  }
  out = std::move(report);
  return HazardStatus::kOk;
}

std::vector<Vec3> make_false_points(const Vec3& center, double radius, double z, std::size_t count,
                                    std::uint64_t seed, std::uint32_t sender, std::uint64_t index) {
  const std::uint64_t bits = mix(seed ^ (static_cast<std::uint64_t>(sender) << 40) ^ mix(index));
  const double u = static_cast<double>(bits >> 11) * (1.0 / 9007199254740992.0);
  const double start = 2.0 * kPi * u;
  const double arc = 0.5 * kPi;
  std::vector<Vec3> points;
  points.reserve(count);
  for (std::size_t k = 0; k < count; ++k) {
    const double share = count == 1 ? 0.5 : static_cast<double>(k) / static_cast<double>(count - 1);
    const double angle = start + arc * share;
    points.push_back({center.x + radius * std::cos(angle), center.y + radius * std::sin(angle), z});
  }
  return points;
}

HazardStore::HazardStore(const ShareParams& params, std::uint64_t ttl_ticks)
    : params_(params), ttl_ticks_(ttl_ticks) {
  validate(params_);
  if (ttl_ticks_ == 0) {
    throw std::invalid_argument("HazardStore: the ttl must be at least one tick");
  }
}

HazardStore::Key HazardStore::key_of(const Vec3& point) const {
  return {static_cast<std::int64_t>(std::floor(point.x / params_.cell_m)),
          static_cast<std::int64_t>(std::floor(point.y / params_.cell_m))};
}

void HazardStore::keep_highest(std::optional<Entry>& slot, const Entry& entry) {
  if (!slot || entry.position.z >= slot->position.z) {
    slot = entry;
    return;
  }
  slot->tick = entry.tick;  // seen again, lower: the point stays and is refreshed
}

void HazardStore::add_own(const Vec3& point, double sigma, std::uint64_t tick) {
  if (!finite(point) || !std::isfinite(sigma) || sigma < 0.0) {
    return;
  }
  Cell& cell = cells_[key_of(point)];
  keep_highest(cell.own, Entry{point, sigma, tick});
  cell.last_tick = std::max(cell.last_tick, tick);
}

void HazardStore::add_remote(std::uint32_t sender, const Vec3& point, double sigma,
                             std::uint64_t tick) {
  if (!finite(point) || !std::isfinite(sigma) || sigma < 0.0) {
    return;
  }
  Cell& cell = cells_[key_of(point)];
  const auto found = cell.remote.find(sender);
  if (found == cell.remote.end()) {
    cell.remote.emplace(sender, Entry{point, sigma, tick});
  } else if (point.z >= found->second.position.z) {
    found->second = Entry{point, sigma, tick};
  } else {
    found->second.tick = tick;
  }
  cell.last_tick = std::max(cell.last_tick, tick);
}

void HazardStore::expire(std::uint64_t tick) {
  const auto old = [&](const Entry& e) { return tick >= e.tick && tick - e.tick > ttl_ticks_; };
  for (auto it = cells_.begin(); it != cells_.end();) {
    Cell& cell = it->second;
    if (cell.own && old(*cell.own)) {
      cell.own.reset();
    }
    for (auto r = cell.remote.begin(); r != cell.remote.end();) {
      r = old(r->second) ? cell.remote.erase(r) : std::next(r);
    }
    if (!cell.own && cell.remote.empty()) {
      it = cells_.erase(it);
    } else {
      ++it;
    }
  }
  if (cells_.size() > params_.max_cells) {
    std::vector<std::pair<std::uint64_t, Key>> by_age;
    by_age.reserve(cells_.size());
    for (const auto& [key, cell] : cells_) {
      by_age.emplace_back(cell.last_tick, key);
    }
    std::sort(by_age.begin(), by_age.end());
    const std::size_t drop = cells_.size() - params_.max_cells;
    for (std::size_t i = 0; i < drop; ++i) {
      cells_.erase(by_age[i].second);
    }
  }
}

std::vector<HazardPoint> HazardStore::usable() const {
  std::vector<HazardPoint> out;
  out.reserve(cells_.size());
  for (const auto& [key, cell] : cells_) {
    const Entry* best = cell.own ? &*cell.own : nullptr;
    if (cell.remote.size() >= params_.min_sources) {
      std::vector<const Entry*> others;
      others.reserve(cell.remote.size());
      for (const auto& [sender, entry] : cell.remote) {
        others.push_back(&entry);  // in sender order
      }
      std::stable_sort(others.begin(), others.end(), [](const Entry* a, const Entry* b) {
        return a->position.z > b->position.z;
      });
      const Entry* supported = others[params_.min_sources - 1];
      if (best == nullptr || supported->position.z > best->position.z) {
        best = supported;
      }
    }
    if (best != nullptr) {
      out.push_back({best->position, best->sigma});
    }
  }
  return out;
}

void HazardStore::hash_state(Fnv1a64& hash) const {
  hash.update_u64(cells_.size());
  for (const auto& [key, cell] : cells_) {
    hash.update_u64(static_cast<std::uint64_t>(key.first));
    hash.update_u64(static_cast<std::uint64_t>(key.second));
    hash.update_u64(cell.own ? cell.own->tick + 1 : 0);
    if (cell.own) {
      hash.update_f64(cell.own->position.x);
      hash.update_f64(cell.own->position.y);
      hash.update_f64(cell.own->position.z);
    }
    hash.update_u64(cell.remote.size());
    for (const auto& [sender, entry] : cell.remote) {
      hash.update_u64(sender);
      hash.update_u64(entry.tick);
      hash.update_f64(entry.position.z);
    }
  }
}

}  // namespace swarm
