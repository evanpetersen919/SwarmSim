// Placeholder runner. It exercises the clock, RNG, and state hash so the determinism
// check in CI has something to compare. Replace with the scenario runner.

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "swarm/format/hash.hpp"
#include "swarm/rng.hpp"
#include "swarm/sim_clock.hpp"

int main(int argc, char** argv) {
  std::uint64_t seed = 42;
  std::uint64_t ticks = 1000;
  for (int i = 1; i + 1 < argc; i += 2) {
    if (std::strcmp(argv[i], "--seed") == 0) {
      seed = std::strtoull(argv[i + 1], nullptr, 10);
    } else if (std::strcmp(argv[i], "--ticks") == 0) {
      ticks = std::strtoull(argv[i + 1], nullptr, 10);
    }
  }

  swarm::SimClock clock(0.01);
  swarm::Rng rng(seed, /*stream=*/0);
  swarm::Fnv1a64 hash;
  for (std::uint64_t t = 0; t < ticks; ++t) {
    hash.update_f64(rng.normal());
    clock.advance();
  }

  std::cout << "swarm_sim " << SWARM_VERSION << " seed=" << seed << " ticks=" << clock.tick()
            << " hash=" << std::hex << hash.digest() << "\n";
  return 0;
}
