#include "swarm/control/command.hpp"

#include <stdexcept>

#include "swarm/format/byte_io.hpp"

namespace swarm {

namespace {

bool valid_kind(std::uint8_t kind) {
  return kind == static_cast<std::uint8_t>(CommandKind::kHold) ||
         kind == static_cast<std::uint8_t>(CommandKind::kResume);
}

}  // namespace

std::vector<std::uint8_t> encode_command(const Command& command) {
  if (!valid_kind(static_cast<std::uint8_t>(command.kind))) {
    throw std::invalid_argument("encode_command: unknown command kind");
  }
  std::vector<std::uint8_t> out;
  out.reserve(kCommandBytes);
  detail::put_u8(out, kCommandVersion);
  detail::put_u8(out, static_cast<std::uint8_t>(command.kind));
  detail::put_u64(out, command.issue_tick);
  return out;
}

CommandStatus decode_command(const std::uint8_t* data, std::size_t size, Command& out) {
  if (size != kCommandBytes) {
    return CommandStatus::kBadSize;
  }
  if (data[0] != kCommandVersion) {
    return CommandStatus::kBadVersion;
  }
  if (!valid_kind(data[1])) {
    return CommandStatus::kBadKind;
  }
  out.kind = static_cast<CommandKind>(data[1]);
  out.issue_tick = detail::get_u64(data + 2);
  return CommandStatus::kOk;
}

}  // namespace swarm
