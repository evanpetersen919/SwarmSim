#include <gtest/gtest.h>

#include <cstdint>
#include <utility>
#include <vector>

#include "swarm/delivery_queue.hpp"
#include "swarm/rng.hpp"

namespace {

swarm::Message make(std::uint64_t tick, std::uint32_t sender, std::uint64_t seq,
                    std::uint32_t receiver, std::uint8_t tag = 0) {
  swarm::Message m;
  m.deliver_tick = tick;
  m.sender = sender;
  m.seq = seq;
  m.receiver = receiver;
  m.payload = {tag};
  return m;
}

}  // namespace

TEST(DeliveryQueue, EmptyQueueReleasesNothing) {
  swarm::DeliveryQueue queue;
  EXPECT_TRUE(queue.empty());
  EXPECT_TRUE(queue.pop_ready(100).empty());
}

TEST(DeliveryQueue, ReleasesInDeliverTickOrder) {
  swarm::DeliveryQueue queue;
  queue.push(make(5, 0, 0, 1));
  queue.push(make(2, 0, 1, 1));
  queue.push(make(9, 0, 2, 1));
  const auto out = queue.pop_ready(10);
  ASSERT_EQ(out.size(), 3U);
  EXPECT_EQ(out[0].deliver_tick, 2U);
  EXPECT_EQ(out[1].deliver_tick, 5U);
  EXPECT_EQ(out[2].deliver_tick, 9U);
}

TEST(DeliveryQueue, SameTickLowerSenderFirst) {
  swarm::DeliveryQueue queue;
  queue.push(make(3, 7, 0, 1));
  queue.push(make(3, 2, 0, 1));
  queue.push(make(3, 5, 0, 1));
  const auto out = queue.pop_ready(3);
  ASSERT_EQ(out.size(), 3U);
  EXPECT_EQ(out[0].sender, 2U);
  EXPECT_EQ(out[1].sender, 5U);
  EXPECT_EQ(out[2].sender, 7U);
}

TEST(DeliveryQueue, SameTickAndSenderLowerSeqFirst) {
  swarm::DeliveryQueue queue;
  queue.push(make(3, 1, 9, 0));
  queue.push(make(3, 1, 4, 0));
  const auto out = queue.pop_ready(3);
  ASSERT_EQ(out.size(), 2U);
  EXPECT_EQ(out[0].seq, 4U);
  EXPECT_EQ(out[1].seq, 9U);
}

TEST(DeliveryQueue, SameSenderAndSeqLowerReceiverFirst) {
  swarm::DeliveryQueue queue;
  queue.push(make(3, 1, 4, 8));
  queue.push(make(3, 1, 4, 2));
  const auto out = queue.pop_ready(3);
  ASSERT_EQ(out.size(), 2U);
  EXPECT_EQ(out[0].receiver, 2U);
  EXPECT_EQ(out[1].receiver, 8U);
}

TEST(DeliveryQueue, IdenticalKeysKeepInsertionOrder) {
  swarm::DeliveryQueue queue;
  for (std::uint8_t tag = 0; tag < 5; ++tag) {
    queue.push(make(3, 1, 4, 2, tag));
  }
  const auto out = queue.pop_ready(3);
  ASSERT_EQ(out.size(), 5U);
  for (std::uint8_t tag = 0; tag < 5; ++tag) {
    EXPECT_EQ(out[tag].payload[0], tag);
  }
}

TEST(DeliveryQueue, FutureMessagesStayQueued) {
  swarm::DeliveryQueue queue;
  queue.push(make(2, 0, 0, 1));
  queue.push(make(6, 0, 1, 1));
  const auto first = queue.pop_ready(4);
  ASSERT_EQ(first.size(), 1U);
  EXPECT_EQ(first[0].deliver_tick, 2U);
  EXPECT_EQ(queue.size(), 1U);
  EXPECT_TRUE(queue.pop_ready(5).empty());
  const auto second = queue.pop_ready(6);
  ASSERT_EQ(second.size(), 1U);
  EXPECT_EQ(second[0].deliver_tick, 6U);
  EXPECT_TRUE(queue.empty());
}

TEST(DeliveryQueue, PayloadSurvivesUnchanged) {
  swarm::DeliveryQueue queue;
  swarm::Message m = make(1, 0, 0, 1);
  m.payload = {1, 2, 3, 250, 0, 7};
  queue.push(m);
  const auto out = queue.pop_ready(1);
  ASSERT_EQ(out.size(), 1U);
  EXPECT_EQ(out[0].payload, m.payload);
}

TEST(DeliveryQueue, InsertionOrderDoesNotChangeOutputForUniqueKeys) {
  std::vector<swarm::Message> messages;
  for (std::uint32_t sender = 0; sender < 6; ++sender) {
    for (std::uint64_t tick = 1; tick <= 4; ++tick) {
      messages.push_back(make(tick, sender, tick * 10 + sender, sender % 3));
    }
  }

  const auto drain = [](const std::vector<swarm::Message>& input) {
    swarm::DeliveryQueue queue;
    for (const auto& m : input) {
      queue.push(m);
    }
    return queue.pop_ready(100);
  };
  const auto reference = drain(messages);

  swarm::Rng rng(11, 0);
  for (int trial = 0; trial < 20; ++trial) {
    auto shuffled = messages;
    for (std::size_t i = shuffled.size() - 1; i > 0; --i) {
      const auto j = static_cast<std::size_t>(rng.next_u32() % (i + 1));
      std::swap(shuffled[i], shuffled[j]);
    }
    const auto out = drain(shuffled);
    ASSERT_EQ(out.size(), reference.size());
    for (std::size_t k = 0; k < out.size(); ++k) {
      EXPECT_EQ(out[k].deliver_tick, reference[k].deliver_tick);
      EXPECT_EQ(out[k].sender, reference[k].sender);
      EXPECT_EQ(out[k].seq, reference[k].seq);
      EXPECT_EQ(out[k].receiver, reference[k].receiver);
    }
  }
}
