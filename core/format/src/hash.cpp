#include "swarm/format/hash.hpp"

#include <cstring>

namespace swarm {

void Fnv1a64::update(const void* data, std::size_t size) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t i = 0; i < size; ++i) {
    state_ ^= bytes[i];
    state_ *= kPrime;
  }
}

void Fnv1a64::update_u64(std::uint64_t value) {
  // Feed bytes in a fixed little-endian order regardless of host byte order.
  unsigned char bytes[8];
  for (int i = 0; i < 8; ++i) {
    bytes[i] = static_cast<unsigned char>((value >> (8 * i)) & 0xFFU);
  }
  update(bytes, sizeof(bytes));
}

void Fnv1a64::update_f64(double value) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  update_u64(bits);
}

}  // namespace swarm
