#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "swarm/message.hpp"

// The wire format shared by the UDP and TCP transports. One message is a fixed header followed by
// its payload, little-endian:
//
//   u8  magic 'S' | u8 magic 'N' | u8 version = 1 | u8 type
//   u32 sender | u32 receiver | u64 seq | u64 send_tick | u16 payload_length | payload
//
// A UDP datagram holds exactly one message. A TCP stream holds messages back to back; the length in
// the header is enough to find the end of each, so there is no second length prefix. There is no
// checksum: UDP and TCP have their own. `deliver_tick` is not on the wire; the receiving side
// learns it from its own record of the send.
namespace swarm::net {

inline constexpr std::uint8_t kWireMagic0 = 'S';
inline constexpr std::uint8_t kWireMagic1 = 'N';
inline constexpr std::uint8_t kWireVersion = 1;
inline constexpr std::size_t kWireHeaderBytes = 30;
// Keeps a datagram under a typical path MTU of 1,280 bytes with room to spare.
inline constexpr std::size_t kMaxWirePayload = 1100;
inline constexpr std::size_t kMaxWireMessageBytes = kWireHeaderBytes + kMaxWirePayload;

enum class WireStatus {
  kOk,
  kTruncated,  // the bytes end before the message does
  kBadMagic,
  kBadVersion,
  kUnknownType,
  kOversized,      // the declared payload is longer than kMaxWirePayload
  kTrailingBytes,  // a datagram holds more than one message
};

const char* to_string(WireStatus status);

// Serializes one message. Throws std::invalid_argument if the type is not a MessageType value or
// the payload is longer than kMaxWirePayload.
std::vector<std::uint8_t> encode_message(const Message& message);

// Decodes exactly one message from exactly `size` bytes (a datagram). On any status but kOk,
// `out` is left unchanged. `deliver_tick` is set to zero.
WireStatus decode_message(const std::uint8_t* data, std::size_t size, Message& out);

// Reassembles messages from a byte stream that arrives in pieces of any size, as TCP delivers it.
// A problem in the header cannot be skipped over, because there is no way to find the next
// message, so the decoder fails and stays failed: later bytes are ignored. A header is checked as
// soon as the bytes that make up a field have arrived, so garbage is noticed at its first byte.
class FrameDecoder {
 public:
  // Adds bytes. Returns the decoder's status after processing them: kOk, or the status it failed
  // with. Messages completed before a failure are kept and can still be taken.
  WireStatus feed(const std::uint8_t* data, std::size_t size);

  // Removes and returns the messages completed so far, in order.
  std::vector<Message> take();

  bool failed() const { return status_ != WireStatus::kOk; }
  WireStatus status() const { return status_; }
  // Bytes held for a message that has not completed yet. Never more than kMaxWireMessageBytes - 1
  // after a call to feed.
  std::size_t buffered() const { return buffer_.size(); }

 private:
  std::vector<std::uint8_t> buffer_;
  std::vector<Message> done_;
  WireStatus status_ = WireStatus::kOk;
};

}  // namespace swarm::net
