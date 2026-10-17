#include "swarm/sim_clock.hpp"

#include <stdexcept>

namespace swarm {

SimClock::SimClock(double dt_seconds) : dt_(dt_seconds) {
  if (!(dt_seconds > 0.0)) {
    throw std::invalid_argument("SimClock: dt must be positive");
  }
}

}  // namespace swarm
