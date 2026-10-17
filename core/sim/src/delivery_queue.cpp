#include "swarm/delivery_queue.hpp"

#include <utility>

namespace swarm {

void DeliveryQueue::push(Message msg) {
  const Key key{msg.deliver_tick, msg.sender, msg.seq, msg.receiver, next_insertion_++};
  queue_.emplace(key, std::move(msg));
}

std::vector<Message> DeliveryQueue::pop_ready(std::uint64_t tick) {
  std::vector<Message> ready;
  auto it = queue_.begin();
  while (it != queue_.end() && std::get<0>(it->first) <= tick) {
    ready.push_back(std::move(it->second));
    it = queue_.erase(it);
  }
  return ready;
}

}  // namespace swarm
