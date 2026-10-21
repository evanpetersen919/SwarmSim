#include "swarm/scenario/run.hpp"

#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "swarm/drone_agent.hpp"
#include "swarm/format/snapshot.hpp"
#include "swarm/scenario/world.hpp"

namespace swarm {

namespace {

const std::string kTag = kScenarioLogTag;

LogHeader make_header(const Scenario& s) {
  LogHeader h;
  h.seed = s.seed;
  h.dt = s.dt;
  h.agent_count = s.drones;
  h.config = kTag + canonical_text(s);
  return h;
}

// The record for the tick that was just stepped. The runner and the verifier both build it here,
// so they cannot disagree about the bytes. The events are consumed.
LogRecord make_record(const Simulation& sim, std::vector<ChannelEvent>& events, std::uint64_t tick,
                      const Scenario& s) {
  Snapshot snapshot;
  const bool with_agents = tick % s.snapshot_every_ticks == 0 || tick + 1 == s.duration_ticks;
  if (with_agents) {
    // The drones only: with commands there is also a ground station, after them.
    snapshot.agents.reserve(s.drones);
    for (std::size_t i = 0; i < s.drones; ++i) {
      const DroneAgent& drone = dynamic_cast<const DroneAgent&>(sim.agent(i));
      AgentSnapshot a;
      a.alive = drone.alive();
      a.fallback_state = static_cast<std::uint8_t>(drone.brain().state());
      const DroneState& truth = drone.truth();
      a.position = {truth.pos.x, truth.pos.y, truth.pos.z};
      a.velocity = {truth.vel.x, truth.vel.y, truth.vel.z};
      snapshot.agents.push_back(a);
    }
  }
  snapshot.events.reserve(events.size());
  for (const ChannelEvent& e : events) {
    snapshot.events.push_back({static_cast<SnapshotEventKind>(e.outcome), e.sender, e.receiver,
                               e.send_tick, e.deliver_tick});
  }
  events.clear();

  LogRecord record;
  record.tick = tick;
  record.hash = sim.state_hash();
  if (encode_snapshot(snapshot, record.blob) != SnapshotStatus::kOk) {
    throw std::logic_error("run_scenario: the snapshot could not be encoded");
  }
  return record;
}

VerifyResult failure(VerifyStatus status, std::string message, std::uint64_t tick = 0) {
  VerifyResult result;
  result.status = status;
  result.message = std::move(message);
  result.tick = tick;
  return result;
}

}  // namespace

bool is_scenario_log(const LogHeader& header) {
  return header.config.compare(0, kTag.size(), kTag) == 0;
}

LogStatus run_scenario(const Scenario& scenario, std::size_t threads,
                       std::vector<std::uint8_t>& log_bytes, std::uint64_t* final_hash,
                       const ChannelFactory& channel_factory, ChannelStats* channel_stats) {
  const ScenarioWorld world = build_world(scenario);
  const std::unique_ptr<Simulation> sim = world.make_simulation(threads, channel_factory);
  std::vector<ChannelEvent> events;
  sim->set_event_sink(&events);

  LogWriter writer;
  LogStatus status = writer.begin(make_header(world.scenario));
  if (status != LogStatus::kOk) {
    return status;
  }
  std::uint64_t last_hash = 0;
  for (std::uint64_t tick = 0; tick < scenario.duration_ticks; ++tick) {
    sim->step();
    const LogRecord record = make_record(*sim, events, tick, scenario);
    last_hash = record.hash;
    status = writer.append(record);
    if (status != LogStatus::kOk) {
      return status;
    }
  }
  status = writer.finish();
  if (status != LogStatus::kOk) {
    return status;
  }
  log_bytes = writer.bytes();
  if (final_hash != nullptr) {
    *final_hash = last_hash;
  }
  if (channel_stats != nullptr) {
    *channel_stats = sim->channel().stats();
  }
  return LogStatus::kOk;
}

VerifyResult verify_scenario_log(const std::uint8_t* data, std::size_t size, std::size_t threads) {
  LogReader reader(data, size);
  LogStatus status = reader.read_header();
  if (status != LogStatus::kOk) {
    VerifyResult result = failure(VerifyStatus::kLogError, to_string(status));
    result.log_status = status;
    return result;
  }
  const LogHeader& header = reader.header();
  if (!is_scenario_log(header)) {
    return failure(VerifyStatus::kBadConfig, "the log does not hold a scenario");
  }

  const LoadResult loaded = load_scenario(header.config.substr(kTag.size()), "scenario");
  if (!loaded.ok()) {
    return failure(VerifyStatus::kBadConfig,
                   "the scenario in the log is invalid: " + to_string(loaded.errors.front()));
  }
  const Scenario& scenario = loaded.scenario;
  if (header.seed != scenario.seed || header.dt != scenario.dt ||
      header.agent_count != scenario.drones) {
    return failure(VerifyStatus::kBadConfig,
                   "the log header disagrees with the scenario it contains");
  }

  std::unique_ptr<ScenarioWorld> world;
  std::unique_ptr<Simulation> sim;
  std::vector<ChannelEvent> events;
  try {
    world = std::make_unique<ScenarioWorld>(build_world(scenario));
    sim = world->make_simulation(threads);
    sim->set_event_sink(&events);
  } catch (const std::exception& e) {
    return failure(VerifyStatus::kBadConfig, e.what());
  }

  std::uint64_t expected_tick = 0;
  LogRecord record;
  for (;;) {
    status = reader.next(record);
    if (status == LogStatus::kEndOfLog) {
      break;
    }
    if (status != LogStatus::kOk) {
      VerifyResult result = failure(VerifyStatus::kLogError, to_string(status));
      result.log_status = status;
      return result;
    }
    if (record.tick != expected_tick) {
      return failure(VerifyStatus::kTickMismatch,
                     "record has tick " + std::to_string(record.tick) + " where tick " +
                         std::to_string(expected_tick) + " was expected",
                     expected_tick);
    }
    if (expected_tick >= scenario.duration_ticks) {
      return failure(VerifyStatus::kLengthMismatch, "log has more records than the " +
                                                        std::to_string(scenario.duration_ticks) +
                                                        " ticks its scenario specifies");
    }

    try {
      sim->step();
    } catch (const std::exception& e) {
      return failure(VerifyStatus::kBadConfig, e.what());
    }
    const LogRecord expected = make_record(*sim, events, expected_tick, scenario);
    if (expected.hash != record.hash) {
      return failure(VerifyStatus::kHashMismatch,
                     "state hash differs at tick " + std::to_string(expected_tick), expected_tick);
    }
    if (expected.blob != record.blob) {
      return failure(VerifyStatus::kSnapshotMismatch,
                     "snapshot differs at tick " + std::to_string(expected_tick), expected_tick);
    }
    ++expected_tick;
  }

  if (expected_tick != scenario.duration_ticks) {
    return failure(VerifyStatus::kLengthMismatch,
                   "log has " + std::to_string(expected_tick) + " records but its scenario " +
                       "specifies " + std::to_string(scenario.duration_ticks) + " ticks");
  }
  return VerifyResult{};
}

VerifyResult verify_run_log(const std::uint8_t* data, std::size_t size, std::size_t threads) {
  LogReader reader(data, size);
  if (reader.read_header() == LogStatus::kOk && is_scenario_log(reader.header())) {
    return verify_scenario_log(data, size, threads);
  }
  return verify_log(data, size, threads);
}

}  // namespace swarm
