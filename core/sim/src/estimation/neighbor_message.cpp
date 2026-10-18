#include "swarm/estimation/neighbor_message.hpp"

#include <cmath>
#include <stdexcept>

#include "swarm/format/byte_io.hpp"

namespace swarm {

namespace {

bool finite(const Vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool valid(const NeighborReport& r) {
  return finite(r.position) && finite(r.velocity) && std::isfinite(r.position_sigma) &&
         r.position_sigma >= 0.0;
}

}  // namespace

std::vector<std::uint8_t> encode_neighbor_report(const NeighborReport& report) {
  if (!valid(report)) {
    throw std::invalid_argument(
        "encode_neighbor_report: values must be finite and the sigma non-negative");
  }
  std::vector<std::uint8_t> out;
  out.reserve(kNeighborReportBytes);
  detail::put_u8(out, kNeighborReportVersion);
  detail::put_u64(out, report.send_tick);
  detail::put_f64(out, report.position.x);
  detail::put_f64(out, report.position.y);
  detail::put_f64(out, report.position.z);
  detail::put_f64(out, report.velocity.x);
  detail::put_f64(out, report.velocity.y);
  detail::put_f64(out, report.velocity.z);
  detail::put_f64(out, report.position_sigma);
  return out;
}

DecodeStatus decode_neighbor_report(const std::uint8_t* data, std::size_t size,
                                    NeighborReport& out) {
  if (size != kNeighborReportBytes) {
    return DecodeStatus::kBadSize;
  }
  if (data[0] != kNeighborReportVersion) {
    return DecodeStatus::kBadVersion;
  }
  NeighborReport r;
  r.send_tick = detail::get_u64(data + 1);
  r.position = {detail::get_f64(data + 9), detail::get_f64(data + 17), detail::get_f64(data + 25)};
  r.velocity = {detail::get_f64(data + 33), detail::get_f64(data + 41), detail::get_f64(data + 49)};
  r.position_sigma = detail::get_f64(data + 57);
  if (!valid(r)) {
    return DecodeStatus::kInvalidValue;
  }
  out = r;
  return DecodeStatus::kOk;
}

}  // namespace swarm
