#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <tuple>
#include <vector>

#include "swarm/message.hpp"

namespace swarm {

// Holds in-flight messages and releases them in a fixed total order:
// (deliver_tick, sender, seq, receiver, insertion index). The insertion index is internal and
// makes the order total even when callers supply identical keys.
//
// Not thread-safe. The channel only touches it from the single-threaded barrier step.
class DeliveryQueue {
 public:
  void push(Message msg);

  // Removes and returns every message with deliver_tick <= tick, in key order.
  std::vector<Message> pop_ready(std::uint64_t tick);

  std::size_t size() const { return queue_.size(); }
  bool empty() const { return queue_.empty(); }

 private:
  using Key = std::tuple<std::uint64_t, std::uint32_t, std::uint64_t, std::uint32_t, std::uint64_t>;

  std::map<Key, Message> queue_;
  std::uint64_t next_insertion_ = 0;
};

}  // namespace swarm
