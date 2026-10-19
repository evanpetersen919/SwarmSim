#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "swarm/format/log.hpp"
#include "swarm/run.hpp"
#include "swarm/scenario/scenario.hpp"
#include "swarm/scenario/world.hpp"

namespace swarm {

// The first line of the log header's config text for a scenario log. The rest of the text is the
// scenario in canonical form. A log whose config does not start with this is a demo log (the
// key=value form of swarm/run.hpp).
inline constexpr const char* kScenarioLogTag = "swarmsim-scenario 1\n";

bool is_scenario_log(const LogHeader& header);

// Runs the scenario for its duration and writes a log with one record per tick. Each record holds
// the tick, the state hash, and a snapshot blob (swarm/format/snapshot.hpp) with the messages sent
// in that tick, and with every drone's true state on ticks divisible by
// `snapshot_every_ticks` and on the last tick. If `final_hash` is not null it receives the hash of
// the last tick.
//
// Throws std::invalid_argument if the scenario cannot be built (see build_world).
//
// With a channel factory the run goes over the channel it makes, for example one that carries the
// messages over real sockets; without one, over the deterministic channel. The log does not record
// which, because the channel is not part of what defines the run, as the thread count is not. If
// `channel_stats` is not null it receives the channel's statistics at the end of the run, which
// include any messages a real network lost. Such a run differs from the simulated one, and its
// log then fails verification.
LogStatus run_scenario(const Scenario& scenario, std::size_t threads,
                       std::vector<std::uint8_t>& log_bytes, std::uint64_t* final_hash = nullptr,
                       const ChannelFactory& channel_factory = {},
                       ChannelStats* channel_stats = nullptr);

// Verifies a scenario log by re-running its scenario and comparing, for every record, the tick,
// the hash, and the snapshot bytes. The thread count is free.
VerifyResult verify_scenario_log(const std::uint8_t* data, std::size_t size, std::size_t threads);

// Verifies either kind of log: a scenario log by re-running its scenario, a demo log with
// verify_log.
VerifyResult verify_run_log(const std::uint8_t* data, std::size_t size, std::size_t threads);

}  // namespace swarm
