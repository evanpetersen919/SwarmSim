#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "swarm/net/socket.hpp"
#include "swarm/net/udp_endpoints.hpp"
#include "swarm/net/wire.hpp"
#include "swarm/rng.hpp"

using swarm::Message;
using swarm::MessageType;
using swarm::net::UdpEndpoints;

namespace {

using Bytes = std::vector<std::uint8_t>;

Message make(std::uint32_t from, std::uint32_t to, std::uint64_t seq, Bytes payload = {},
             MessageType type = MessageType::kBeacon) {
  Message m;
  m.type = type;
  m.sender = from;
  m.receiver = to;
  m.seq = seq;
  m.send_tick = seq * 3 + 1;
  m.payload = std::move(payload);
  return m;
}

bool same(const Message& a, const Message& b) {
  return a.type == b.type && a.sender == b.sender && a.receiver == b.receiver && a.seq == b.seq &&
         a.send_tick == b.send_tick && a.payload == b.payload;
}

// Collects messages until `count` have arrived or the time is up.
std::vector<Message> collect(UdpEndpoints& net, std::size_t count, int total_ms = 3000) {
  std::vector<Message> all;
  const auto start = std::chrono::steady_clock::now();
  while (all.size() < count) {
    const auto spent = std::chrono::steady_clock::now() - start;
    if (spent > std::chrono::milliseconds(total_ms)) {
      break;
    }
    std::vector<Message> some = net.receive(50);
    all.insert(all.end(), some.begin(), some.end());
  }
  return all;
}

}  // namespace

TEST(UdpEndpoints, TwoEndpointsExchangeMessagesWithTheWireFormatIntact) {
  UdpEndpoints net(2);
  ASSERT_EQ(net.count(), 2U);
  EXPECT_NE(net.address(0).port, net.address(1).port);

  const Message to_one = make(0, 1, 7, {1, 2, 3, 4}, MessageType::kBeacon);
  const Message to_zero = make(1, 0, 9, {}, MessageType::kCommand);
  net.send(to_one);
  net.send(to_zero);
  const std::vector<Message> got = collect(net, 2);
  ASSERT_EQ(got.size(), 2U);
  for (const Message& m : got) {
    EXPECT_TRUE(same(m, m.receiver == 1 ? to_one : to_zero));
  }
  EXPECT_EQ(net.stats().sent, 2U);
  EXPECT_EQ(net.stats().received, 2U);
}

TEST(UdpEndpoints, RandomPayloadsOfEverySizeArriveWhole) {
  UdpEndpoints net(2);
  swarm::Rng rng(3, 0);
  std::vector<Message> sent;
  for (std::size_t size : {0U, 1U, 2U, 100U, 1000U, 1099U, 1100U}) {
    Bytes payload(size);
    for (auto& b : payload) {
      b = static_cast<std::uint8_t>(rng.next_u32() & 0xFFU);
    }
    sent.push_back(make(0, 1, sent.size(), payload));
    net.send(sent.back());
  }
  const std::vector<Message> got = collect(net, sent.size());
  ASSERT_EQ(got.size(), sent.size());
  std::map<std::uint64_t, Message> by_seq;
  for (const Message& m : got) {
    by_seq[m.seq] = m;
  }
  for (const Message& m : sent) {
    ASSERT_EQ(by_seq.count(m.seq), 1U);
    EXPECT_TRUE(same(by_seq[m.seq], m)) << "payload size " << m.payload.size();
  }
}

TEST(UdpEndpoints, ManyEndpointsEachReceiveExactlyWhatWasSentToThem) {
  const std::uint32_t n = 32;
  UdpEndpoints net(n);
  std::set<std::tuple<std::uint32_t, std::uint32_t, std::uint64_t>> expected;
  std::uint64_t seq = 0;
  for (std::uint32_t from = 0; from < n; ++from) {
    for (std::uint32_t to = 0; to < n; ++to) {
      if (from != to) {
        net.send(
            make(from, to, seq, {static_cast<std::uint8_t>(from), static_cast<std::uint8_t>(to)}));
        expected.insert({from, to, seq});
        ++seq;
      }
    }
  }
  const std::vector<Message> got = collect(net, expected.size());
  ASSERT_EQ(got.size(), expected.size());
  std::set<std::tuple<std::uint32_t, std::uint32_t, std::uint64_t>> seen;
  for (const Message& m : got) {
    // Each message arrived once, at the endpoint it was addressed to, with its payload intact.
    EXPECT_TRUE(seen.insert({m.sender, m.receiver, m.seq}).second);
    ASSERT_EQ(m.payload.size(), 2U);
    EXPECT_EQ(m.payload[0], m.sender);
    EXPECT_EQ(m.payload[1], m.receiver);
  }
  EXPECT_EQ(seen, expected);
  EXPECT_EQ(net.stats().malformed + net.stats().foreign + net.stats().misdelivered, 0U);
}

TEST(UdpEndpoints, ABurstOfThousandsOfMessagesToOneReceiverAllArrive) {
  UdpEndpoints net(2);
  const std::uint64_t count = 6000;
  for (std::uint64_t i = 0; i < count; ++i) {
    net.send(make(0, 1, i, Bytes(80, static_cast<std::uint8_t>(i & 0xFFU))));
  }
  const std::vector<Message> got = collect(net, count, 10000);
  EXPECT_EQ(got.size(), count);  // none lost: send drains often enough that the queue never fills
  std::set<std::uint64_t> seqs;
  for (const Message& m : got) {
    seqs.insert(m.seq);
  }
  EXPECT_EQ(seqs.size(), count);
  EXPECT_EQ(net.stats().sent, count);
  EXPECT_EQ(net.stats().received, count);
}

TEST(UdpEndpoints, BadDatagramsAreCountedAndDoNotStopTheTransport) {
  UdpEndpoints net(3);

  // Garbage, an empty datagram, and one that is too long to be a message.
  const Bytes garbage = {1, 2, 3, 4, 5};
  net.send_raw(0, 1, garbage.data(), garbage.size());
  net.send_raw(0, 1, garbage.data(), 0);
  const Bytes too_long(3000, 'S');
  net.send_raw(2, 1, too_long.data(), too_long.size());

  // A well-formed message whose sender field names endpoint 2 but which came from endpoint 0.
  const Bytes spoofed = swarm::net::encode_message(make(2, 1, 1));
  net.send_raw(0, 1, spoofed.data(), spoofed.size());

  // A well-formed message from endpoint 0, correctly attributed, but addressed to endpoint 2 and
  // delivered to endpoint 1.
  const Bytes wrong_door = swarm::net::encode_message(make(0, 2, 2));
  net.send_raw(0, 1, wrong_door.data(), wrong_door.size());

  // And a good one after all of that.
  net.send(make(0, 1, 99, {42}));

  const std::vector<Message> got = collect(net, 1);
  // Allow the bad ones time to be taken in too.
  std::vector<Message> more = collect(net, 1, 300);
  ASSERT_EQ(got.size() + more.size(), 1U);
  EXPECT_EQ((got.empty() ? more : got)[0].seq, 99U);

  EXPECT_EQ(net.stats().malformed, 3U);     // garbage, empty, and too long
  EXPECT_EQ(net.stats().foreign, 1U);       // the spoofed sender
  EXPECT_EQ(net.stats().misdelivered, 1U);  // the wrong door
  EXPECT_EQ(net.stats().received, 1U);
}

TEST(UdpEndpoints, ADatagramFromAProgramThatIsNotAnEndpointIsForeignWhateverItContains) {
  UdpEndpoints net(2);
  swarm::net::UdpSocket outsider = swarm::net::UdpSocket::bind_loopback();
  const Bytes valid = swarm::net::encode_message(make(0, 1, 5, {1}));
  outsider.send_to(net.address(1), valid.data(), valid.size());
  EXPECT_TRUE(collect(net, 1, 300).empty());
  EXPECT_EQ(net.stats().foreign, 1U);
  EXPECT_EQ(net.stats().received, 0U);

  // The transport still works.
  net.send(make(0, 1, 6));
  EXPECT_EQ(collect(net, 1).size(), 1U);
}

TEST(UdpEndpoints, AReceiveWithNothingWaitingTimesOutEmptyAndPollsWithZero) {
  UdpEndpoints net(2);
  const auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(net.receive(80).empty());
  EXPECT_GE(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(60));
  EXPECT_TRUE(net.receive(0).empty());
}

TEST(UdpEndpoints, MessagesTakenInBySendAreReturnedByTheNextReceive) {
  UdpEndpoints net(2);
  // More than kDrainEvery sends, so some are drained inside send() and the rest are not.
  for (std::uint64_t i = 0; i < 40; ++i) {
    net.send(make(0, 1, i));
  }
  EXPECT_EQ(collect(net, 40).size(), 40U);
  EXPECT_TRUE(net.receive(0).empty());  // and they are returned once
}

TEST(UdpEndpoints, MisuseIsRejectedWithClearErrors) {
  EXPECT_THROW(UdpEndpoints(0), std::invalid_argument);
  EXPECT_THROW(UdpEndpoints(UdpEndpoints::kMaxEndpoints + 1), std::invalid_argument);

  UdpEndpoints net(2);
  EXPECT_THROW(net.send(make(2, 0, 0)), std::out_of_range);
  EXPECT_THROW(net.send(make(0, 2, 0)), std::out_of_range);
  EXPECT_THROW(net.address(2), std::out_of_range);
  EXPECT_THROW(net.send(make(0, 1, 0, Bytes(swarm::net::kMaxWirePayload + 1))),
               std::invalid_argument);
  const std::uint8_t byte = 0;
  EXPECT_THROW(net.send_raw(0, 5, &byte, 1), std::out_of_range);
  EXPECT_EQ(net.stats().sent, 0U);  // nothing was sent by any of the above
}
