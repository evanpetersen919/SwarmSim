#pragma once

// Little-endian byte helpers shared by the project's binary formats: the run log, scene data, and
// neighbor messages. Fixed byte order regardless of the host, no struct copying, no exceptions.

#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace swarm::detail {

static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559,
              "the binary formats store doubles as IEEE 754 binary64");

inline void put_u8(std::vector<std::uint8_t>& out, std::uint8_t v) { out.push_back(v); }

inline void put_u16(std::vector<std::uint8_t>& out, std::uint16_t v) {
  out.push_back(static_cast<std::uint8_t>(v & 0xFFU));
  out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFFU));
}

inline void put_u32(std::vector<std::uint8_t>& out, std::uint32_t v) {
  for (int i = 0; i < 4; ++i) {
    out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFFU));
  }
}

inline void put_u64(std::vector<std::uint8_t>& out, std::uint64_t v) {
  for (int i = 0; i < 8; ++i) {
    out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFFU));
  }
}

inline void put_f64(std::vector<std::uint8_t>& out, double v) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));
  put_u64(out, bits);
}

inline std::uint16_t get_u16(const std::uint8_t* p) {
  return static_cast<std::uint16_t>(static_cast<unsigned>(p[0]) |
                                    (static_cast<unsigned>(p[1]) << 8));
}

inline std::uint32_t get_u32(const std::uint8_t* p) {
  std::uint32_t v = 0;
  for (int i = 0; i < 4; ++i) {
    v |= static_cast<std::uint32_t>(p[i]) << (8 * i);
  }
  return v;
}

inline std::uint64_t get_u64(const std::uint8_t* p) {
  std::uint64_t v = 0;
  for (int i = 0; i < 8; ++i) {
    v |= static_cast<std::uint64_t>(p[i]) << (8 * i);
  }
  return v;
}

inline double get_f64(const std::uint8_t* p) {
  const std::uint64_t bits = get_u64(p);
  double v = 0.0;
  std::memcpy(&v, &bits, sizeof(v));
  return v;
}

}  // namespace swarm::detail
