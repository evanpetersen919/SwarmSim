#include "swarm/estimation/neighbor_table.hpp"

#include <stdexcept>

namespace swarm {

NeighborTable::NeighborTable(std::uint32_t max_neighbors) : records_(max_neighbors) {}

ReceiveResult NeighborTable::on_message(std::uint32_t sender, const std::uint8_t* payload,
                                        std::size_t size, std::uint64_t receipt_tick) {
  if (sender >= records_.size()) {
    ++counters_.unknown_sender;
    return ReceiveResult::kUnknownSender;
  }

  NeighborReport report;
  if (decode_neighbor_report(payload, size, report) != DecodeStatus::kOk) {
    ++counters_.malformed;
    return ReceiveResult::kMalformed;
  }

  std::optional<NeighborRecord>& held = records_[sender];
  if (held.has_value()) {
    if (report.send_tick == held->report.send_tick) {
      ++counters_.duplicate;
      return ReceiveResult::kDuplicate;
    }
    if (report.send_tick < held->report.send_tick) {
      ++counters_.stale;
      return ReceiveResult::kStale;
    }
  }

  held = NeighborRecord{report, receipt_tick};
  ++counters_.accepted;
  return ReceiveResult::kAccepted;
}

const NeighborRecord* NeighborTable::get(std::uint32_t sender) const {
  if (sender >= records_.size() || !records_[sender].has_value()) {
    return nullptr;
  }
  return &*records_[sender];
}

std::optional<std::uint64_t> NeighborTable::age(std::uint32_t sender,
                                                std::uint64_t now_tick) const {
  const NeighborRecord* record = get(sender);
  if (record == nullptr) {
    return std::nullopt;
  }
  if (now_tick < record->receipt_tick) {
    throw std::invalid_argument("NeighborTable::age: now_tick is earlier than the receipt");
  }
  return now_tick - record->receipt_tick;
}

}  // namespace swarm
