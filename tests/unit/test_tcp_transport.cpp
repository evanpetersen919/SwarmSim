#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <vector>

#include "swarm/net/tcp_connections.hpp"
#include "swarm/net/wire.hpp"
#include "swarm/rng.hpp"

using swarm::Message;
using swarm::MessageType;
using swarm::net::TcpConnections;

namespace {

using Bytes = std::vector<std::uint8_t>;

Message make(std::uint32_t from, std::uint32_t to, std::uint64_t seq, Bytes payload = {},
             MessageType type = MessageType::kCommand) {
  Message m;
  m.type = type;
  m.sender = from;
  m.receiver = to;
  m.seq = seq;
  m.send_tick = seq + 5;
  m.payload = std::move(payload);
  return m;
}

bool same(const Message& a, const Message& b) {
  return a.type == b.type && a.sender == b.sender && a.receiver == b.receiver && a.seq == b.seq &&
         a.send_tick == b.send_tick && a.payload == b.payload;
}

std::vector<Message> collect(TcpConnections& net, std::size_t count, int total_ms = 5000) {
  std::vector<Message> all;
  const auto start = std::chrono::steady_clock::now();
  while (all.size() < count) {
    if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(total_ms)) {
      break;
    }
    std::vector<Message> some = net.receive(50);
    all.insert(all.end(), some.begin(), some.end());
  }
  return all;
}

}  // namespace

TEST(TcpConnections, ACommandArrivesWhole) {
  TcpConnections net(3);
  const Message m = make(2, 0, 1, {9, 8, 7, 6, 5});
  net.send(m);
  const std::vector<Message> got = collect(net, 1);
  ASSERT_EQ(got.size(), 1U);
  EXPECT_TRUE(same(got[0], m));
  EXPECT_EQ(net.stats().sent, 1U);
  EXPECT_EQ(net.stats().received, 1U);
  EXPECT_EQ(net.stats().connections_opened, 1U);
}

TEST(TcpConnections, TenThousandMessagesArriveInOrderWholeAndNoneIsLost) {
  TcpConnections net(2);
  swarm::Rng rng(6, 0);
  std::vector<Message> sent;
  for (std::uint64_t i = 0; i < 10000; ++i) {
    Bytes payload(rng.below(200));
    for (auto& b : payload) {
      b = static_cast<std::uint8_t>(rng.next_u32() & 0xFFU);
    }
    sent.push_back(make(0, 1, i, payload));
    net.send(sent.back());
  }
  const std::vector<Message> got = collect(net, sent.size(), 20000);
  ASSERT_EQ(got.size(), sent.size());
  for (std::size_t i = 0; i < sent.size(); ++i) {
    ASSERT_TRUE(same(got[i], sent[i])) << "message " << i;  // same order, same bytes
  }
  EXPECT_EQ(net.stats().broken + net.stats().misdelivered + net.stats().misattributed, 0U);
}

TEST(TcpConnections, ASenderThatWritesFarMoreThanTheBuffersHoldDoesNotBlockForever) {
  // Twenty thousand messages written before anything is received: the sender drains as it goes.
  TcpConnections net(2);
  const std::uint64_t count = 20000;
  for (std::uint64_t i = 0; i < count; ++i) {
    net.send(make(0, 1, i, Bytes(100, 1)));
  }
  EXPECT_EQ(collect(net, count, 20000).size(), count);
}

TEST(TcpConnections, MessagesWrittenOneByteAtATimeAreReassembled) {
  TcpConnections net(2);
  std::vector<Message> sent;
  for (std::uint64_t i = 0; i < 30; ++i) {
    sent.push_back(make(0, 1, i, Bytes(i + 1, static_cast<std::uint8_t>(i))));
    const Bytes bytes = swarm::net::encode_message(sent.back());
    for (std::uint8_t byte : bytes) {
      net.send_raw(0, 1, &byte, 1);
    }
  }
  const std::vector<Message> got = collect(net, sent.size());
  ASSERT_EQ(got.size(), sent.size());
  for (std::size_t i = 0; i < sent.size(); ++i) {
    EXPECT_TRUE(same(got[i], sent[i])) << i;
  }
}

TEST(TcpConnections, OneConnectionIsOpenedPerPairInEachDirectionAndReused) {
  TcpConnections net(3);
  for (std::uint64_t i = 0; i < 100; ++i) {
    net.send(make(0, 1, i));
  }
  for (std::uint64_t i = 0; i < 50; ++i) {
    net.send(make(1, 0, i));
  }
  for (std::uint64_t i = 0; i < 20; ++i) {
    net.send(make(0, 2, i));
  }
  EXPECT_EQ(collect(net, 170).size(), 170U);
  EXPECT_EQ(net.stats().connections_opened, 3U);  // 0 to 1, 1 to 0, and 0 to 2
}

TEST(TcpConnections, AStationCommandingManyDronesHoldsOneConnectionToEach) {
  const std::uint32_t drones = 40;
  TcpConnections net(drones + 1);  // the station is the last endpoint
  for (std::uint32_t round = 0; round < 3; ++round) {
    for (std::uint32_t d = 0; d < drones; ++d) {
      net.send(make(drones, d, round));
    }
  }
  EXPECT_EQ(collect(net, drones * 3).size(), drones * 3U);
  EXPECT_EQ(net.stats().connections_opened, drones);
}

TEST(TcpConnections, ADifferentPairsOrderIsKeptPerPair) {
  TcpConnections net(3);
  for (std::uint64_t i = 0; i < 500; ++i) {
    net.send(make(0, 2, i));
    net.send(make(1, 2, i));
  }
  const std::vector<Message> got = collect(net, 1000);
  ASSERT_EQ(got.size(), 1000U);
  std::map<std::uint32_t, std::uint64_t> next;  // the next sequence number expected per sender
  for (const Message& m : got) {
    EXPECT_EQ(m.seq, next[m.sender]++);
  }
}

TEST(TcpConnections, AClosedConnectionIsReportedAndOthersAreUnaffectedAndSendingReopensIt) {
  TcpConnections net(3);
  net.send(make(0, 1, 1));
  net.send(make(2, 1, 1));
  ASSERT_EQ(collect(net, 2).size(), 2U);

  net.close_connection(0, 1);
  net.send(make(2, 1, 2));  // the other connection carries on
  EXPECT_EQ(collect(net, 1).size(), 1U);
  EXPECT_EQ(net.stats().closed, 1U);

  net.send(make(0, 1, 3));  // a new connection in the closed direction
  const std::vector<Message> got = collect(net, 1);
  ASSERT_EQ(got.size(), 1U);
  EXPECT_EQ(got[0].seq, 3U);
  EXPECT_EQ(net.stats().connections_opened, 3U);
}

TEST(TcpConnections, ACorruptStreamIsReportedAndClosedAndOthersAreUnaffected) {
  TcpConnections net(3);
  net.send(make(0, 1, 1));
  net.send(make(2, 1, 1));
  ASSERT_EQ(collect(net, 2).size(), 2U);

  const Bytes garbage = {1, 2, 3, 4, 5, 6, 7, 8};
  net.send_raw(0, 1, garbage.data(), garbage.size());
  net.send(make(2, 1, 2));  // arrives, though the 0 to 1 stream is broken
  const std::vector<Message> got = collect(net, 1);
  ASSERT_EQ(got.size(), 1U);
  EXPECT_EQ(got[0].sender, 2U);
  std::vector<Message> more = collect(net, 1, 300);  // let the garbage be read
  EXPECT_TRUE(more.empty());
  EXPECT_EQ(net.stats().broken, 1U);

  // The receiver closed the broken connection. The sender learns that from a failed write, so a
  // message written right after may be lost; messages written afterwards arrive.
  bool arrived = false;
  for (std::uint64_t seq = 10; seq < 30 && !arrived; ++seq) {
    net.send(make(0, 1, seq));
    for (const Message& m : collect(net, 1, 200)) {
      arrived = arrived || m.sender == 0;
    }
  }
  EXPECT_TRUE(arrived);
}

TEST(TcpConnections, AMessageForAnotherEndpointAndAFalseSenderAreCountedAndDropped) {
  TcpConnections net(3);
  net.send(make(0, 1, 1));  // the first message fixes this connection's sender as 0
  ASSERT_EQ(collect(net, 1).size(), 1U);

  const Bytes wrong_door = swarm::net::encode_message(make(0, 2, 2));  // addressed to endpoint 2
  net.send_raw(0, 1, wrong_door.data(), wrong_door.size());
  const Bytes false_sender = swarm::net::encode_message(make(2, 1, 3));  // claims to be from 2
  net.send_raw(0, 1, false_sender.data(), false_sender.size());
  net.send(make(0, 1, 4));
  const std::vector<Message> got = collect(net, 1);
  ASSERT_EQ(got.size(), 1U);
  EXPECT_EQ(got[0].seq, 4U);
  EXPECT_EQ(net.stats().misdelivered, 1U);
  EXPECT_EQ(net.stats().misattributed, 1U);
}

TEST(TcpConnections, AReceiveWithNothingWaitingTimesOutEmpty) {
  TcpConnections net(2);
  const auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(net.receive(80).empty());
  EXPECT_GE(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(60));
  EXPECT_TRUE(net.receive(0).empty());
}

TEST(TcpConnections, MisuseIsRejectedWithClearErrors) {
  EXPECT_THROW(TcpConnections(0), std::invalid_argument);
  EXPECT_THROW(TcpConnections(TcpConnections::kMaxEndpoints + 1), std::invalid_argument);
  TcpConnections net(2);
  EXPECT_THROW(net.send(make(2, 0, 0)), std::out_of_range);
  EXPECT_THROW(net.send(make(0, 2, 0)), std::out_of_range);
  EXPECT_THROW(net.address(2), std::out_of_range);
  EXPECT_THROW(net.send(make(0, 1, 0, Bytes(swarm::net::kMaxWirePayload + 1))),
               std::invalid_argument);
  EXPECT_EQ(net.stats().sent, 0U);
  EXPECT_EQ(net.stats().connections_opened, 0U);  // a rejected message opens no connection
}
