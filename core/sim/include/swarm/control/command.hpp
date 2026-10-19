#pragma once

// Controller headers may include only this directory, swarm/estimation/, sensor_data.hpp,
// vec3.hpp, drone_command.hpp, message.hpp, and swarm/format/{byte_io,hash}.hpp. They must
// never include vehicle truth.
// Enforced by scripts/check_core_invariants.sh.

#include <cstddef>
#include <cstdint>
#include <vector>

namespace swarm {

// What a ground station can tell a drone. Hold: stop at the current position and stay there.
// Resume: go back to flying the formation.
enum class CommandKind : std::uint8_t {
  kHold = 1,
  kResume = 2,
};

struct Command {
  CommandKind kind = CommandKind::kHold;
  std::uint64_t issue_tick = 0;  // the tick the station issued it, used to order commands
};

inline constexpr std::uint8_t kCommandVersion = 1;
inline constexpr std::size_t kCommandBytes = 1 + 1 + 8;

// Layout, little-endian: u8 version | u8 kind | u64 issue_tick. Throws std::invalid_argument if
// the kind is not one of the two values.
std::vector<std::uint8_t> encode_command(const Command& command);

enum class CommandStatus {
  kOk,
  kBadSize,
  kBadVersion,
  kBadKind,
};

// On any status but kOk, `out` is left unchanged.
CommandStatus decode_command(const std::uint8_t* data, std::size_t size, Command& out);

}  // namespace swarm
