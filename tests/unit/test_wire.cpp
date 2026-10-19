#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "swarm/net/wire.hpp"
#include "swarm/rng.hpp"

using swarm::Message;
using swarm::MessageType;
using swarm::net::FrameDecoder;
using swarm::net::WireStatus;

namespace {

using Bytes = std::vector<std::uint8_t>;

Message make(MessageType type, std::uint32_t sender, std::uint32_t receiver, std::uint64_t seq,
             std::uint64_t send_tick, Bytes payload) {
  Message m;
  m.type = type;
  m.sender = sender;
  m.receiver = receiver;
  m.seq = seq;
  m.send_tick = send_tick;
  m.payload = std::move(payload);
  return m;
}

bool same(const Message& a, const Message& b) {
  return a.type == b.type && a.sender == b.sender && a.receiver == b.receiver && a.seq == b.seq &&
         a.send_tick == b.send_tick && a.payload == b.payload;
}

// Hand-checked bytes of one message. Changing the format means changing this on purpose and
// bumping kWireVersion.
Bytes sample_bytes() {
  return {
      'S',  'N',  0x01, 0x02,                          // magic, version 1, command
      0x04, 0x03, 0x02, 0x01,                          // sender 0x01020304
      0x05, 0x00, 0x00, 0x00,                          // receiver 5
      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,  // seq 0x0807060504030201
      0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // send_tick 9
      0x03, 0x00,                                      // payload length 3
      0xAA, 0xBB, 0xCC,                                // payload
  };
}

Message sample() {
  return make(MessageType::kCommand, 0x01020304U, 5, 0x0807060504030201ULL, 9, {0xAA, 0xBB, 0xCC});
}

Message random_message(swarm::Rng& rng) {
  const std::size_t length = rng.below(swarm::net::kMaxWirePayload + 1);
  Bytes payload(length);
  for (auto& b : payload) {
    b = static_cast<std::uint8_t>(rng.next_u32() & 0xFFU);
  }
  return make(rng.bernoulli(0.5) ? MessageType::kBeacon : MessageType::kCommand, rng.next_u32(),
              rng.next_u32(), (static_cast<std::uint64_t>(rng.next_u32()) << 32) | rng.next_u32(),
              (static_cast<std::uint64_t>(rng.next_u32()) << 32) | rng.next_u32(),
              std::move(payload));
}

}  // namespace

TEST(Wire, EncodesToTheDocumentedBytes) {
  EXPECT_EQ(swarm::net::encode_message(sample()), sample_bytes());
  EXPECT_EQ(sample_bytes().size(), swarm::net::kWireHeaderBytes + 3);
  EXPECT_EQ(swarm::net::kWireHeaderBytes, 30U);
}

TEST(Wire, DecodesTheDocumentedBytes) {
  const Bytes bytes = sample_bytes();
  Message out;
  ASSERT_EQ(swarm::net::decode_message(bytes.data(), bytes.size(), out), WireStatus::kOk);
  EXPECT_TRUE(same(out, sample()));
  EXPECT_EQ(out.deliver_tick, 0U);  // not on the wire
}

TEST(Wire, RoundTripsRandomMessagesIncludingTheExtremes) {
  swarm::Rng rng(5, 0);
  for (int n = 0; n < 500; ++n) {
    const Message m = random_message(rng);
    const Bytes bytes = swarm::net::encode_message(m);
    ASSERT_EQ(bytes.size(), swarm::net::kWireHeaderBytes + m.payload.size());
    Message out;
    ASSERT_EQ(swarm::net::decode_message(bytes.data(), bytes.size(), out), WireStatus::kOk) << n;
    EXPECT_TRUE(same(out, m)) << n;
  }
  const Message empty = make(MessageType::kBeacon, 0, 0, 0, 0, {});
  const Message largest =
      make(MessageType::kCommand, 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFFFFFFFFFULL,
           0xFFFFFFFFFFFFFFFFULL, Bytes(swarm::net::kMaxWirePayload, 0x5A));
  for (const Message& m : {empty, largest}) {
    const Bytes bytes = swarm::net::encode_message(m);
    Message out;
    ASSERT_EQ(swarm::net::decode_message(bytes.data(), bytes.size(), out), WireStatus::kOk);
    EXPECT_TRUE(same(out, m));
  }
  EXPECT_EQ(swarm::net::encode_message(largest).size(), swarm::net::kMaxWireMessageBytes);
}

TEST(Wire, EncodingRejectsWhatTheFormatCannotCarry) {
  Message big = sample();
  big.payload.assign(swarm::net::kMaxWirePayload + 1, 0);
  EXPECT_THROW(swarm::net::encode_message(big), std::invalid_argument);
  Message bad_type = sample();
  bad_type.type = static_cast<MessageType>(7);
  EXPECT_THROW(swarm::net::encode_message(bad_type), std::invalid_argument);
  bad_type.type = static_cast<MessageType>(0);
  EXPECT_THROW(swarm::net::encode_message(bad_type), std::invalid_argument);
}

TEST(Wire, EveryTruncationOfADatagramIsRejectedAsTruncated) {
  const Bytes full = sample_bytes();
  for (std::size_t length = 0; length < full.size(); ++length) {
    // A copy of exactly `length` bytes, so a read past the end is caught by the sanitizers.
    const Bytes cut(full.begin(), full.begin() + static_cast<std::ptrdiff_t>(length));
    Message out = make(MessageType::kBeacon, 77, 0, 0, 0, {});
    EXPECT_EQ(swarm::net::decode_message(cut.data(), cut.size(), out), WireStatus::kTruncated)
        << "length " << length;
    EXPECT_EQ(out.sender, 77U);  // untouched on an error
  }
}

TEST(Wire, EachMalformedHeaderFieldHasItsOwnStatus) {
  const auto decode = [](Bytes bytes) {
    Message out;
    return swarm::net::decode_message(bytes.data(), bytes.size(), out);
  };
  Bytes b = sample_bytes();
  b[0] = 'X';
  EXPECT_EQ(decode(b), WireStatus::kBadMagic);
  b = sample_bytes();
  b[1] = 'X';
  EXPECT_EQ(decode(b), WireStatus::kBadMagic);
  for (int version : {0, 2, 255}) {
    b = sample_bytes();
    b[2] = static_cast<std::uint8_t>(version);
    EXPECT_EQ(decode(b), WireStatus::kBadVersion) << version;
  }
  for (int type : {0, 3, 255}) {
    b = sample_bytes();
    b[3] = static_cast<std::uint8_t>(type);
    EXPECT_EQ(decode(b), WireStatus::kUnknownType) << type;
  }
  b = sample_bytes();
  b[28] = 0x4D;  // 1,101: one more than the limit
  b[29] = 0x04;
  EXPECT_EQ(decode(b), WireStatus::kOversized);
  b = sample_bytes();
  b[28] = 0x4C;  // 1,100: allowed by the limit, but the datagram does not hold that much
  b[29] = 0x04;
  EXPECT_EQ(decode(b), WireStatus::kTruncated);
}

TEST(Wire, ExtraBytesAfterTheMessageInADatagramAreRejected) {
  Bytes b = sample_bytes();
  b.push_back(0);
  Message out;
  EXPECT_EQ(swarm::net::decode_message(b.data(), b.size(), out), WireStatus::kTrailingBytes);
  // A declared length shorter than the bytes present is the same thing.
  b = sample_bytes();
  b[28] = 2;
  EXPECT_EQ(swarm::net::decode_message(b.data(), b.size(), out), WireStatus::kTrailingBytes);
}

TEST(Wire, StatusNamesAreDistinct) {
  const WireStatus all[] = {WireStatus::kOk,           WireStatus::kTruncated,
                            WireStatus::kBadMagic,     WireStatus::kBadVersion,
                            WireStatus::kUnknownType,  WireStatus::kOversized,
                            WireStatus::kTrailingBytes};
  for (std::size_t i = 0; i < 7; ++i) {
    for (std::size_t j = i + 1; j < 7; ++j) {
      EXPECT_STRNE(swarm::net::to_string(all[i]), swarm::net::to_string(all[j]));
    }
  }
}

namespace {

Bytes concatenated(const std::vector<Message>& messages) {
  Bytes all;
  for (const Message& m : messages) {
    const Bytes b = swarm::net::encode_message(m);
    all.insert(all.end(), b.begin(), b.end());
  }
  return all;
}

std::vector<Message> random_messages(int count, std::uint64_t seed) {
  swarm::Rng rng(seed, 0);
  std::vector<Message> out;
  for (int i = 0; i < count; ++i) {
    out.push_back(random_message(rng));
  }
  return out;
}

void expect_same(const std::vector<Message>& a, const std::vector<Message>& b) {
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_TRUE(same(a[i], b[i])) << "message " << i;
  }
}

}  // namespace

TEST(FrameDecoder, ReturnsTheSameMessagesWhateverTheChunking) {
  const std::vector<Message> sent = random_messages(60, 8);
  const Bytes stream = concatenated(sent);

  {  // all at once
    FrameDecoder d;
    EXPECT_EQ(d.feed(stream.data(), stream.size()), WireStatus::kOk);
    expect_same(d.take(), sent);
    EXPECT_EQ(d.buffered(), 0U);
  }
  {  // one byte at a time
    FrameDecoder d;
    std::vector<Message> got;
    for (std::uint8_t byte : stream) {
      ASSERT_EQ(d.feed(&byte, 1), WireStatus::kOk);
      const std::vector<Message> some = d.take();
      got.insert(got.end(), some.begin(), some.end());
    }
    expect_same(got, sent);
  }
  swarm::Rng rng(9, 0);
  for (int trial = 0; trial < 20; ++trial) {  // random chunk sizes, including zero
    FrameDecoder d;
    std::vector<Message> got;
    std::size_t at = 0;
    while (at < stream.size()) {
      const std::size_t n = std::min<std::size_t>(rng.below(3000), stream.size() - at);
      ASSERT_EQ(d.feed(stream.data() + at, n), WireStatus::kOk);
      at += n;
      const std::vector<Message> some = d.take();
      got.insert(got.end(), some.begin(), some.end());
    }
    expect_same(got, sent);
  }
}

TEST(FrameDecoder, SplitsEveryPossibleBoundaryOfTwoMessages) {
  const std::vector<Message> sent = {sample(),
                                     make(MessageType::kBeacon, 1, 2, 3, 4, {1, 2, 3, 4, 5})};
  const Bytes stream = concatenated(sent);
  for (std::size_t cut = 0; cut <= stream.size(); ++cut) {
    FrameDecoder d;
    ASSERT_EQ(d.feed(stream.data(), cut), WireStatus::kOk) << cut;
    ASSERT_EQ(d.feed(stream.data() + cut, stream.size() - cut), WireStatus::kOk) << cut;
    expect_same(d.take(), sent);
  }
}

TEST(FrameDecoder, TwoMessagesInOneChunkBothComeOut) {
  const std::vector<Message> sent = {sample(), sample()};
  const Bytes stream = concatenated(sent);
  FrameDecoder d;
  d.feed(stream.data(), stream.size());
  EXPECT_EQ(d.take().size(), 2U);
  EXPECT_TRUE(d.take().empty());  // taken messages are not returned again
}

TEST(FrameDecoder, HoldsAtMostOneIncompleteMessage) {
  const Bytes stream = concatenated(random_messages(30, 3));
  FrameDecoder d;
  for (std::size_t at = 0; at < stream.size(); at += 7) {
    const std::size_t n = std::min<std::size_t>(7, stream.size() - at);
    d.feed(stream.data() + at, n);
    EXPECT_LT(d.buffered(), swarm::net::kMaxWireMessageBytes);
    d.take();
  }
}

TEST(FrameDecoder, AFailureIsStickyAndKeepsTheMessagesCompletedBeforeIt) {
  Bytes stream = concatenated({sample(), sample()});
  stream.push_back('X');  // garbage where a third header should begin
  FrameDecoder d;
  EXPECT_EQ(d.feed(stream.data(), stream.size()), WireStatus::kBadMagic);
  EXPECT_TRUE(d.failed());
  EXPECT_EQ(d.status(), WireStatus::kBadMagic);
  EXPECT_EQ(d.buffered(), 0U);
  EXPECT_EQ(d.take().size(), 2U);  // the two good messages before the garbage

  const Bytes good = swarm::net::encode_message(sample());
  EXPECT_EQ(d.feed(good.data(), good.size()), WireStatus::kBadMagic);  // still failed
  EXPECT_TRUE(d.take().empty());
}

TEST(FrameDecoder, GarbageIsNoticedAsSoonAsTheFieldHasArrived) {
  const Bytes good = swarm::net::encode_message(sample());
  {
    FrameDecoder d;
    const std::uint8_t x = 'X';
    EXPECT_EQ(d.feed(&x, 1), WireStatus::kBadMagic);  // after one byte
  }
  {
    FrameDecoder d;
    EXPECT_EQ(d.feed(good.data(), 2), WireStatus::kOk);
    const std::uint8_t bad_version = 9;
    EXPECT_EQ(d.feed(&bad_version, 1), WireStatus::kBadVersion);  // after three bytes
  }
  {
    FrameDecoder d;
    EXPECT_EQ(d.feed(good.data(), 3), WireStatus::kOk);
    const std::uint8_t bad_type = 0;
    EXPECT_EQ(d.feed(&bad_type, 1), WireStatus::kUnknownType);  // after four bytes
  }
  {
    // An oversized length is seen when the whole header has arrived, before any payload.
    Bytes b = good;
    b[28] = 0xFF;
    b[29] = 0xFF;
    FrameDecoder d;
    EXPECT_EQ(d.feed(b.data(), swarm::net::kWireHeaderBytes - 1), WireStatus::kOk);
    EXPECT_EQ(d.feed(b.data() + swarm::net::kWireHeaderBytes - 1, 1), WireStatus::kOversized);
  }
}
