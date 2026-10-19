#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <utility>
#include <vector>

#include "swarm/net/socket.hpp"
#include "swarm/rng.hpp"

using swarm::net::Address;
using swarm::net::IoStatus;
using swarm::net::NetError;
using swarm::net::TcpListener;
using swarm::net::TcpStream;
using swarm::net::UdpSocket;

namespace {

using Bytes = std::vector<std::uint8_t>;

Bytes pattern(std::size_t size, std::uint64_t seed) {
  swarm::Rng rng(seed, 0);
  Bytes out(size);
  for (auto& b : out) {
    b = static_cast<std::uint8_t>(rng.next_u32() & 0xFFU);
  }
  return out;
}

// A connected TCP pair on the loopback interface.
std::pair<TcpStream, TcpStream> connected_pair() {
  TcpListener listener = TcpListener::listen_loopback();
  TcpStream client = TcpStream::connect(listener.local_address());
  std::optional<TcpStream> server = listener.accept(2000);
  EXPECT_TRUE(server.has_value());
  return {std::move(client), std::move(*server)};
}

}  // namespace

TEST(Socket, AUdpSocketBindsToTheLoopbackOnAPortTheSystemChooses) {
  UdpSocket a = UdpSocket::bind_loopback();
  UdpSocket b = UdpSocket::bind_loopback();
  ASSERT_TRUE(a.valid() && b.valid());
  const Address aa = a.local_address();
  const Address bb = b.local_address();
  EXPECT_EQ(aa.ip, 0x7F000001U);  // 127.0.0.1
  EXPECT_NE(aa.port, 0);
  EXPECT_NE(bb.port, 0);
  EXPECT_NE(aa.port, bb.port);
  EXPECT_EQ(aa.to_string().rfind("127.0.0.1:", 0), 0U);
  EXPECT_GT(a.receive_buffer_bytes(), 0);
}

TEST(Socket, ADatagramGoesFromOneSocketToAnotherWithTheSenderAddress) {
  UdpSocket a = UdpSocket::bind_loopback();
  UdpSocket b = UdpSocket::bind_loopback();
  const Bytes sent = pattern(300, 1);
  a.send_to(b.local_address(), sent.data(), sent.size());

  Bytes buffer(2048);
  const UdpSocket::Received r = b.receive(buffer.data(), buffer.size(), 2000);
  ASSERT_EQ(r.status, IoStatus::kOk);
  EXPECT_FALSE(r.truncated);
  ASSERT_EQ(r.size, sent.size());
  EXPECT_TRUE(std::equal(sent.begin(), sent.end(), buffer.begin()));
  EXPECT_EQ(r.from, a.local_address());
}

TEST(Socket, AnEmptyDatagramIsADatagram) {
  UdpSocket a = UdpSocket::bind_loopback();
  UdpSocket b = UdpSocket::bind_loopback();
  const std::uint8_t none = 0;
  a.send_to(b.local_address(), &none, 0);
  Bytes buffer(16);
  const UdpSocket::Received r = b.receive(buffer.data(), buffer.size(), 2000);
  EXPECT_EQ(r.status, IoStatus::kOk);
  EXPECT_EQ(r.size, 0U);
  EXPECT_FALSE(r.truncated);
}

TEST(Socket, ADatagramLargerThanTheBufferIsReportedTruncatedNotLostSilently) {
  UdpSocket a = UdpSocket::bind_loopback();
  UdpSocket b = UdpSocket::bind_loopback();
  const Bytes sent = pattern(1000, 2);
  a.send_to(b.local_address(), sent.data(), sent.size());
  Bytes small(100);
  const UdpSocket::Received r = b.receive(small.data(), small.size(), 2000);
  ASSERT_EQ(r.status, IoStatus::kOk);
  EXPECT_TRUE(r.truncated);
  EXPECT_EQ(r.size, 100U);
  EXPECT_TRUE(std::equal(small.begin(), small.end(), sent.begin()));

  // The datagram is gone: the next receive times out, it is not delivered again.
  EXPECT_EQ(b.receive(small.data(), small.size(), 50).status, IoStatus::kTimeout);
}

TEST(Socket, ManyDatagramsKeepTheirBoundariesAndOrderOnLoopback) {
  UdpSocket a = UdpSocket::bind_loopback();
  UdpSocket b = UdpSocket::bind_loopback();
  for (std::uint8_t i = 0; i < 100; ++i) {
    const Bytes d(static_cast<std::size_t>(i) + 1, i);  // datagram i is i + 1 bytes of value i
    a.send_to(b.local_address(), d.data(), d.size());
  }
  Bytes buffer(256);
  for (std::uint8_t i = 0; i < 100; ++i) {
    const UdpSocket::Received r = b.receive(buffer.data(), buffer.size(), 2000);
    ASSERT_EQ(r.status, IoStatus::kOk) << int{i};
    ASSERT_EQ(r.size, static_cast<std::size_t>(i) + 1) << int{i};
    ASSERT_EQ(buffer[0], i);
  }
}

TEST(Socket, AReceiveWithNothingToReadTimesOutAsAStatusNotAnException) {
  UdpSocket b = UdpSocket::bind_loopback();
  Bytes buffer(64);
  UdpSocket::Received r;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_NO_THROW(r = b.receive(buffer.data(), buffer.size(), 80));
  const auto waited = std::chrono::steady_clock::now() - start;
  EXPECT_EQ(r.status, IoStatus::kTimeout);
  EXPECT_GE(waited, std::chrono::milliseconds(60));  // it did wait, give or take the clock
  EXPECT_NO_THROW(r = b.receive(buffer.data(), buffer.size(), 0));  // 0 polls
  EXPECT_EQ(r.status, IoStatus::kTimeout);
}

TEST(Socket, AReceiveWithDataWaitingReturnsAtOnce) {
  UdpSocket a = UdpSocket::bind_loopback();
  UdpSocket b = UdpSocket::bind_loopback();
  const Bytes sent = pattern(10, 3);
  a.send_to(b.local_address(), sent.data(), sent.size());
  Bytes buffer(64);
  // A long timeout that is not used up.
  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(b.receive(buffer.data(), buffer.size(), 5000).status, IoStatus::kOk);
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(2000));
}

TEST(Socket, TwoSocketsCannotShareAPortAndTheErrorSaysWhy) {
  UdpSocket a = UdpSocket::bind_loopback();
  try {
    UdpSocket b = UdpSocket::bind_loopback(a.local_address().port);
    FAIL() << "expected the second bind to fail";
  } catch (const NetError& e) {
    EXPECT_NE(e.code(), 0);
    EXPECT_NE(std::string(e.what()).find("bind()"), std::string::npos);
  }
}

TEST(Socket, ClosingASocketFreesItsPortAndASocketIsMoveOnly) {
  std::uint16_t port = 0;
  {
    UdpSocket a = UdpSocket::bind_loopback();
    port = a.local_address().port;
    UdpSocket moved = std::move(a);
    EXPECT_FALSE(a.valid());  // NOLINT: checking the moved-from state is the point
    EXPECT_TRUE(moved.valid());
    EXPECT_EQ(moved.local_address().port, port);
  }  // closed here
  UdpSocket again = UdpSocket::bind_loopback(port);  // the port is free again
  EXPECT_EQ(again.local_address().port, port);

  UdpSocket x = UdpSocket::bind_loopback();
  x.close();
  EXPECT_FALSE(x.valid());
  x.close();  // closing twice is harmless
}

TEST(Socket, UsingAClosedSocketThrowsInsteadOfDoingSomethingUndefined) {
  UdpSocket x = UdpSocket::bind_loopback();
  x.close();
  Bytes buffer(8);
  const std::uint8_t byte = 1;
  EXPECT_THROW(x.send_to(Address::loopback(9), &byte, 1), NetError);
  EXPECT_THROW(x.receive(buffer.data(), buffer.size(), 0), NetError);
  EXPECT_THROW(x.local_address(), NetError);
  UdpSocket never;  // default constructed
  EXPECT_FALSE(never.valid());
  EXPECT_THROW(never.local_address(), NetError);
}

TEST(Socket, ATcpConnectionCarriesALargeBufferExactly) {
  auto pair = connected_pair();
  TcpStream& client = pair.first;
  TcpStream& server = pair.second;
  const Bytes sent = pattern(4 << 20, 4);  // four MiB: more than any socket buffer holds

  std::atomic<bool> sender_ok{false};
  std::thread sender(
      [&] { sender_ok = client.send_all(sent.data(), sent.size()) == IoStatus::kOk; });

  Bytes received(sent.size());
  std::size_t got = 0;
  const IoStatus status = server.receive_exact(received.data(), received.size(), got, 10000);
  sender.join();
  EXPECT_EQ(status, IoStatus::kOk);
  EXPECT_EQ(got, sent.size());
  EXPECT_TRUE(sender_ok.load());
  EXPECT_EQ(received, sent);
}

TEST(Socket, TcpDoesNotKeepWriteBoundariesSoReadsMayReturnPiecesOfAWrite) {
  auto pair = connected_pair();
  TcpStream& client = pair.first;
  TcpStream& server = pair.second;
  const Bytes sent = pattern(10000, 5);
  ASSERT_EQ(client.send_all(sent.data(), sent.size()), IoStatus::kOk);
  Bytes received;
  Bytes chunk(777);
  while (received.size() < sent.size()) {
    std::size_t got = 0;
    ASSERT_EQ(server.receive_some(chunk.data(), chunk.size(), got, 2000), IoStatus::kOk);
    ASSERT_GT(got, 0U);
    received.insert(received.end(), chunk.begin(),
                    chunk.begin() + static_cast<std::ptrdiff_t>(got));
  }
  EXPECT_EQ(received, sent);
}

TEST(Socket, ReceivingFromAConnectionWhosePeerClosedReportsClosed) {
  auto pair = connected_pair();
  TcpStream& client = pair.first;
  TcpStream& server = pair.second;
  const std::uint8_t bytes[3] = {1, 2, 3};
  ASSERT_EQ(client.send_all(bytes, 3), IoStatus::kOk);
  client.close();

  Bytes buffer(16);
  std::size_t got = 0;
  // The bytes sent before the close still arrive, then the close is reported.
  ASSERT_EQ(server.receive_exact(buffer.data(), 3, got, 2000), IoStatus::kOk);
  EXPECT_EQ(got, 3U);
  EXPECT_EQ(server.receive_some(buffer.data(), buffer.size(), got, 2000), IoStatus::kClosed);
  EXPECT_EQ(got, 0U);
  EXPECT_EQ(server.receive_exact(buffer.data(), 1, got, 2000), IoStatus::kClosed);
}

TEST(Socket, SendingToAClosedPeerEventuallyReportsClosedAndNeverKillsTheProcess) {
  auto pair = connected_pair();
  TcpStream& client = pair.first;
  TcpStream& server = pair.second;
  server.close();
  const Bytes data(65536, 7);
  IoStatus status = IoStatus::kOk;
  for (int i = 0; i < 200 && status == IoStatus::kOk; ++i) {
    status = client.send_all(data.data(), data.size());
  }
  EXPECT_EQ(status, IoStatus::kClosed);
}

TEST(Socket, AReceiveExactThatRunsOutOfTimeSaysHowMuchArrived) {
  auto pair = connected_pair();
  TcpStream& client = pair.first;
  TcpStream& server = pair.second;
  const std::uint8_t bytes[4] = {9, 8, 7, 6};
  ASSERT_EQ(client.send_all(bytes, 4), IoStatus::kOk);
  Bytes buffer(10);
  std::size_t got = 0;
  EXPECT_EQ(server.receive_exact(buffer.data(), 10, got, 100), IoStatus::kTimeout);
  EXPECT_EQ(got, 4U);
  EXPECT_EQ(buffer[0], 9);
  EXPECT_EQ(buffer[3], 6);
  // And a plain wait for nothing times out with nothing.
  EXPECT_EQ(server.receive_some(buffer.data(), buffer.size(), got, 30), IoStatus::kTimeout);
  EXPECT_EQ(got, 0U);
}

TEST(Socket, AListenerAcceptsConnectionsInOrderAndTimesOutWhenThereAreNone) {
  TcpListener listener = TcpListener::listen_loopback();
  EXPECT_FALSE(listener.accept(50).has_value());  // a timeout, not an exception

  TcpStream first = TcpStream::connect(listener.local_address());
  TcpStream second = TcpStream::connect(listener.local_address());
  std::optional<TcpStream> a = listener.accept(2000);
  std::optional<TcpStream> b = listener.accept(2000);
  ASSERT_TRUE(a.has_value() && b.has_value());
  const std::uint8_t one = 1;
  const std::uint8_t two = 2;
  ASSERT_EQ(first.send_all(&one, 1), IoStatus::kOk);
  ASSERT_EQ(second.send_all(&two, 1), IoStatus::kOk);
  std::uint8_t got_a = 0;
  std::uint8_t got_b = 0;
  std::size_t n = 0;
  ASSERT_EQ(a->receive_some(&got_a, 1, n, 2000), IoStatus::kOk);
  ASSERT_EQ(b->receive_some(&got_b, 1, n, 2000), IoStatus::kOk);
  EXPECT_EQ(got_a, 1);  // the first connection was accepted first
  EXPECT_EQ(got_b, 2);
}

TEST(Socket, ConnectingToAPortNobodyListensOnThrowsWithTheReason) {
  std::uint16_t port = 0;
  {
    TcpListener closed = TcpListener::listen_loopback();
    port = closed.local_address().port;
  }  // the listener is gone; nothing listens on that port now
  try {
    TcpStream::connect(Address::loopback(port));
    FAIL() << "expected the connection to be refused";
  } catch (const NetError& e) {
    EXPECT_NE(e.code(), 0);
    EXPECT_NE(std::string(e.what()).find("connect()"), std::string::npos);
  }
}

TEST(Socket, WaitReadableSaysWhichSocketsHaveDataAndTimesOutOtherwise) {
  UdpSocket a = UdpSocket::bind_loopback();
  UdpSocket b = UdpSocket::bind_loopback();
  UdpSocket c = UdpSocket::bind_loopback();
  const std::vector<const swarm::net::Socket*> all = {&a, &b, &c};

  EXPECT_TRUE(swarm::net::wait_readable(all, 30).empty());
  EXPECT_TRUE(swarm::net::wait_readable({}, 30).empty());

  const std::uint8_t byte = 1;
  UdpSocket sender = UdpSocket::bind_loopback();
  sender.send_to(b.local_address(), &byte, 1);
  const std::vector<std::size_t> ready = swarm::net::wait_readable(all, 2000);
  ASSERT_EQ(ready.size(), 1U);
  EXPECT_EQ(ready[0], 1U);  // b

  sender.send_to(a.local_address(), &byte, 1);
  sender.send_to(c.local_address(), &byte, 1);
  // Both a and c have data; b still does too, since its datagram was not read.
  std::vector<std::size_t> now;
  for (int i = 0; i < 100 && now.size() < 3; ++i) {
    now = swarm::net::wait_readable(all, 20);
  }
  EXPECT_EQ(now.size(), 3U);
}

TEST(Socket, AListenerIsReadableWhenAConnectionIsWaiting) {
  TcpListener listener = TcpListener::listen_loopback();
  const std::vector<const swarm::net::Socket*> one = {&listener};
  EXPECT_TRUE(swarm::net::wait_readable(one, 30).empty());
  TcpStream client = TcpStream::connect(listener.local_address());
  EXPECT_EQ(swarm::net::wait_readable(one, 2000).size(), 1U);
}

TEST(Socket, AddressesPrintAndCompare) {
  const Address a = Address::loopback(5000);
  EXPECT_EQ(a.to_string(), "127.0.0.1:5000");
  EXPECT_EQ(a, Address::loopback(5000));
  EXPECT_NE(a, Address::loopback(5001));
  Address other;
  other.ip = 0x0A000102;
  other.port = 65535;
  EXPECT_EQ(other.to_string(), "10.0.1.2:65535");
}
