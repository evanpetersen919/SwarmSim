#include "swarm/net/wire.hpp"

#include <stdexcept>
#include <string>
#include <utility>

#include "swarm/format/byte_io.hpp"

namespace swarm::net {

namespace {

bool valid_type(std::uint8_t type) {
  return type == static_cast<std::uint8_t>(MessageType::kBeacon) ||
         type == static_cast<std::uint8_t>(MessageType::kCommand) ||
         type == static_cast<std::uint8_t>(MessageType::kHazard);
}

// Checks the header fields that are present in the first `available` bytes, in order, so the first
// wrong field is reported as soon as it can be seen.
WireStatus check_prefix(const std::uint8_t* data, std::size_t available) {
  if (available >= 1 && data[0] != kWireMagic0) {
    return WireStatus::kBadMagic;
  }
  if (available >= 2 && data[1] != kWireMagic1) {
    return WireStatus::kBadMagic;
  }
  if (available >= 3 && data[2] != kWireVersion) {
    return WireStatus::kBadVersion;
  }
  if (available >= 4 && !valid_type(data[3])) {
    return WireStatus::kUnknownType;
  }
  if (available >= kWireHeaderBytes && detail::get_u16(data + 28) > kMaxWirePayload) {
    return WireStatus::kOversized;
  }
  return WireStatus::kOk;
}

// Fills `out` from a complete, checked message of `total` bytes.
void read_message(const std::uint8_t* data, std::size_t payload_length, Message& out) {
  out.type = static_cast<MessageType>(data[3]);
  out.sender = detail::get_u32(data + 4);
  out.receiver = detail::get_u32(data + 8);
  out.seq = detail::get_u64(data + 12);
  out.send_tick = detail::get_u64(data + 20);
  out.deliver_tick = 0;
  out.payload.assign(data + kWireHeaderBytes, data + kWireHeaderBytes + payload_length);
}

}  // namespace

const char* to_string(WireStatus status) {
  switch (status) {
    case WireStatus::kOk:
      return "ok";
    case WireStatus::kTruncated:
      return "truncated";
    case WireStatus::kBadMagic:
      return "bad magic";
    case WireStatus::kBadVersion:
      return "bad version";
    case WireStatus::kUnknownType:
      return "unknown type";
    case WireStatus::kOversized:
      return "oversized payload";
    case WireStatus::kTrailingBytes:
      return "trailing bytes";
  }
  return "unknown";
}

std::vector<std::uint8_t> encode_message(const Message& message) {
  if (!valid_type(static_cast<std::uint8_t>(message.type))) {
    throw std::invalid_argument("encode_message: unknown message type");
  }
  if (message.payload.size() > kMaxWirePayload) {
    throw std::invalid_argument("encode_message: payload is longer than " +
                                std::to_string(kMaxWirePayload) + " bytes");
  }
  std::vector<std::uint8_t> out;
  out.reserve(kWireHeaderBytes + message.payload.size());
  detail::put_u8(out, kWireMagic0);
  detail::put_u8(out, kWireMagic1);
  detail::put_u8(out, kWireVersion);
  detail::put_u8(out, static_cast<std::uint8_t>(message.type));
  detail::put_u32(out, message.sender);
  detail::put_u32(out, message.receiver);
  detail::put_u64(out, message.seq);
  detail::put_u64(out, message.send_tick);
  detail::put_u16(out, static_cast<std::uint16_t>(message.payload.size()));
  out.insert(out.end(), message.payload.begin(), message.payload.end());
  return out;
}

WireStatus decode_message(const std::uint8_t* data, std::size_t size, Message& out) {
  const WireStatus prefix = check_prefix(data, size < kWireHeaderBytes ? size : kWireHeaderBytes);
  if (prefix != WireStatus::kOk) {
    return prefix;
  }
  if (size < kWireHeaderBytes) {
    return WireStatus::kTruncated;
  }
  const std::size_t payload_length = detail::get_u16(data + 28);
  if (size < kWireHeaderBytes + payload_length) {
    return WireStatus::kTruncated;
  }
  if (size > kWireHeaderBytes + payload_length) {
    return WireStatus::kTrailingBytes;
  }
  read_message(data, payload_length, out);
  return WireStatus::kOk;
}

WireStatus FrameDecoder::feed(const std::uint8_t* data, std::size_t size) {
  if (failed()) {
    return status_;
  }
  buffer_.insert(buffer_.end(), data, data + size);

  std::size_t consumed = 0;
  for (;;) {
    const std::size_t left = buffer_.size() - consumed;
    const std::uint8_t* at = buffer_.data() + consumed;
    const WireStatus prefix = check_prefix(at, left < kWireHeaderBytes ? left : kWireHeaderBytes);
    if (prefix != WireStatus::kOk) {
      status_ = prefix;
      break;
    }
    if (left < kWireHeaderBytes) {
      break;
    }
    const std::size_t payload_length = detail::get_u16(at + 28);
    if (left < kWireHeaderBytes + payload_length) {
      break;
    }
    Message message;
    read_message(at, payload_length, message);
    done_.push_back(std::move(message));
    consumed += kWireHeaderBytes + payload_length;
  }

  if (failed()) {
    buffer_.clear();
  } else {
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(consumed));
  }
  return status_;
}

std::vector<Message> FrameDecoder::take() {
  std::vector<Message> out = std::move(done_);
  done_.clear();
  return out;
}

}  // namespace swarm::net
