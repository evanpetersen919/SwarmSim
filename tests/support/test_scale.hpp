#pragma once

// Sanitizer builds are Debug builds and run tens of times slower than release, so the heaviest
// tests use a smaller size there. The assertions are unchanged and the full size runs in the
// release jobs. A test that is cheap should not use this.

namespace swarm::testing {

#ifdef SWARM_SANITIZED
inline constexpr bool kSanitized = true;
#else
inline constexpr bool kSanitized = false;
#endif

constexpr int scaled(int full, int sanitized) { return kSanitized ? sanitized : full; }

}  // namespace swarm::testing
