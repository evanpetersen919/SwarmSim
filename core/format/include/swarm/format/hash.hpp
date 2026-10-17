#pragma once

#include <cstddef>
#include <cstdint>

namespace swarm {

// 64-bit FNV-1a over raw bytes. Used to hash simulation state each tick so two runs
// can be compared exactly. Results depend on byte order; logs are little-endian only.
class Fnv1a64 {
 public:
  static constexpr std::uint64_t kOffsetBasis = 14695981039346656037ULL;
  static constexpr std::uint64_t kPrime = 1099511628211ULL;

  void update(const void* data, std::size_t size);
  void update_u64(std::uint64_t value);
  void update_f64(double value);

  std::uint64_t digest() const { return state_; }

 private:
  std::uint64_t state_ = kOffsetBasis;
};

}  // namespace swarm
