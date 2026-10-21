#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "swarm/experiment/batch.hpp"
#include "swarm/experiment/experiment.hpp"
#include "swarm/scenario/run.hpp"

using swarm::BatchOptions;
using swarm::BatchResult;
using swarm::Expansion;
using swarm::Experiment;

namespace {

const char* kBase = R"(version: 1
name: tiny
seed: 9
dt: 0.02
duration_s: 4
scene: {type: field, size_m: [400, 400], cell_m: 8}
swarm:
  drones: 4
  formation: {shape: wedge, spacing_m: 20}
  start: {columns: 2, spacing_m: 30, jitter_m: 4, altitude_m: 30, altitude_jitter_m: 2}
mission:
  waypoints:
    - [-50, 0, 30]
    - [50, 0, 30]
  speed_mps: 3
comms: {latency_ticks: 2, loss_model: independent, loss: 0.0}
metrics: {settle_s: 2}
)";

const char* kTwoByTwo = R"(version: 1
name: tiny
base: base.yaml
seeds: {first: 100, count: 3}
axes:
  - name: loss
    levels:
      - {label: p00, set: {comms: {loss: 0.0}}}
      - {label: p30, set: {comms: {loss: 0.3}}}
  - name: size
    levels:
      - {label: n04, set: {}}
      - {label: n06, set: {swarm: {drones: 6, start: {columns: 3}}}}
)";

struct Fixture {
  Experiment experiment;
  Expansion expansion;
};

Fixture make(const std::string& text = kTwoByTwo) {
  const swarm::ReadFile read = [](const std::string&, std::string& out) {
    out = kBase;
    return true;
  };
  swarm::LoadedExperiment l = swarm::load_experiment(text, read);
  EXPECT_TRUE(l.ok());
  Fixture f;
  f.experiment = l.experiment;
  f.expansion = swarm::expand(f.experiment);
  EXPECT_TRUE(f.expansion.ok());
  return f;
}

std::size_t fields(const std::string& line) {
  return static_cast<std::size_t>(std::count(line.begin(), line.end(), ',')) + 1;
}

std::vector<std::string> lines(const std::string& text) {
  std::vector<std::string> out;
  std::size_t start = 0;
  while (start < text.size()) {
    const std::size_t end = text.find('\n', start);
    out.push_back(text.substr(start, end - start));
    start = end == std::string::npos ? text.size() : end + 1;
  }
  return out;
}

}  // namespace

TEST(Batch, EveryCellAndSeedBecomesARowInTaskOrder) {
  const Fixture f = make();
  const BatchResult r = swarm::run_batch(f.experiment, f.expansion, BatchOptions{});
  ASSERT_EQ(r.rows.size(), 12U);  // 4 cells x 3 seeds
  EXPECT_TRUE(r.ok());
  for (std::size_t i = 0; i < r.rows.size(); ++i) {
    const swarm::BatchRow& row = r.rows[i];
    EXPECT_EQ(row.task.index, i);
    EXPECT_EQ(row.task.cell, i / 3);
    EXPECT_EQ(row.task.replicate, i % 3);
    EXPECT_EQ(row.task.seed, 100U + i % 3);
    EXPECT_EQ(row.status, "ok");
    ASSERT_TRUE(row.has_metrics);
    EXPECT_EQ(row.metrics.seed, row.task.seed);
    EXPECT_EQ(row.metrics.scenario_name, f.expansion.cells[i / 3].scenario.name);
    EXPECT_EQ(row.metrics.drones, f.expansion.cells[i / 3].scenario.drones);
  }
}

TEST(Batch, TheTableIsTheSameForAnyNumberOfJobs) {
  const Fixture f = make();
  BatchOptions one;
  one.jobs = 1;
  const std::string reference =
      swarm::batch_csv(f.experiment, f.expansion, swarm::run_batch(f.experiment, f.expansion, one));
  for (std::uint32_t jobs : {2U, 3U, 6U, 40U}) {
    BatchOptions many;
    many.jobs = jobs;
    const std::string table = swarm::batch_csv(f.experiment, f.expansion,
                                               swarm::run_batch(f.experiment, f.expansion, many));
    EXPECT_EQ(table, reference) << jobs << " jobs";
  }
}

TEST(Batch, VerificationIsSampledByTaskIndex) {
  const Fixture f = make();
  const auto verified = [&](std::uint32_t every) {
    BatchOptions o;
    o.jobs = 3;
    o.verify_every = every;
    std::set<std::uint64_t> set;
    const BatchResult r = swarm::run_batch(f.experiment, f.expansion, o);
    EXPECT_TRUE(r.ok());
    for (const auto& row : r.rows) {
      if (row.verified) {
        set.insert(row.task.index);
      }
    }
    return set;
  };
  EXPECT_TRUE(verified(0).empty());
  EXPECT_EQ(verified(1).size(), 12U);
  EXPECT_EQ(verified(5), (std::set<std::uint64_t>{0, 5, 10}));
}

TEST(Batch, AFailedRunIsARowWithAStatusAndTheOthersStillRun) {
  Fixture f = make();
  f.expansion.cells[1].scenario.drones = 0;  // runs, but there is nothing to measure
  BatchOptions o;
  o.jobs = 4;
  const BatchResult r = swarm::run_batch(f.experiment, f.expansion, o);
  ASSERT_EQ(r.rows.size(), 12U);
  EXPECT_FALSE(r.ok());
  EXPECT_EQ(r.failed, 3U);
  for (const auto& row : r.rows) {
    if (row.task.cell == 1) {
      EXPECT_EQ(row.status, "metrics failed: bad scenario");  // an empty swarm has no metrics
      EXPECT_FALSE(row.has_metrics);
    } else {
      EXPECT_EQ(row.status, "ok");
    }
  }

  // The failed row has the same number of columns, with empty metric cells.
  const std::vector<std::string> table = lines(swarm::batch_csv(f.experiment, f.expansion, r));
  ASSERT_EQ(table.size(), 13U);
  for (const std::string& line : table) {
    EXPECT_EQ(fields(line), fields(table[0])) << line;
  }
  EXPECT_NE(table[4].find("metrics failed"), std::string::npos);  // row of task 3, cell 1
  EXPECT_NE(table[4].find(",,,,,"), std::string::npos);
}

TEST(Batch, TheLogSinkGetsEveryLogAndEachOneVerifies) {
  const Fixture f = make();
  std::mutex mutex;
  std::set<std::uint64_t> seen;
  std::size_t bad = 0;
  BatchOptions o;
  o.jobs = 4;
  o.verify_every = 0;
  o.log_sink = [&](const swarm::BatchTask& task, const std::vector<std::uint8_t>& log) {
    const swarm::VerifyResult v = swarm::verify_scenario_log(log.data(), log.size(), 1);
    const std::lock_guard<std::mutex> lock(mutex);
    seen.insert(task.index);
    if (v.status != swarm::VerifyStatus::kOk) {
      ++bad;
    }
  };
  ASSERT_TRUE(swarm::run_batch(f.experiment, f.expansion, o).ok());
  EXPECT_EQ(seen.size(), 12U);
  EXPECT_EQ(bad, 0U);
}

TEST(Batch, ProgressReportsEveryTaskOnceAndEndsAtTheTotal) {
  const Fixture f = make();
  std::vector<std::uint64_t> done;
  BatchOptions o;
  o.jobs = 4;
  o.progress = [&](std::uint64_t d, std::uint64_t total) {
    EXPECT_EQ(total, 12U);
    done.push_back(d);
  };
  swarm::run_batch(f.experiment, f.expansion, o);
  ASSERT_EQ(done.size(), 12U);
  for (std::size_t i = 0; i < done.size(); ++i) {
    EXPECT_EQ(done[i], i + 1);
  }
}

TEST(Batch, ProgressDoesNotChangeTheTable) {
  const Fixture f = make();
  BatchOptions quiet;
  BatchOptions loud;
  loud.progress = [](std::uint64_t, std::uint64_t) {};
  EXPECT_EQ(swarm::batch_csv(f.experiment, f.expansion,
                             swarm::run_batch(f.experiment, f.expansion, quiet)),
            swarm::batch_csv(f.experiment, f.expansion,
                             swarm::run_batch(f.experiment, f.expansion, loud)));
}

TEST(Batch, TheHeaderAndRowsHaveTheDocumentedColumns) {
  const Fixture f = make();
  const std::string header = swarm::batch_csv_header(f.experiment);
  EXPECT_EQ(header.rfind("experiment,cell,loss,size,replicate,status,verified,scenario,seed,", 0),
            0U)
      << header;
  const BatchResult r = swarm::run_batch(f.experiment, f.expansion, BatchOptions{});
  const std::string row = swarm::batch_csv_row(f.experiment, f.expansion, r.rows[4]);
  EXPECT_EQ(row.rfind("tiny,1,p00,n06,1,ok,0,tiny.p00-n06,101,", 0), 0U) << row;
  EXPECT_EQ(row.back(), '\n');
  // 27 metric columns plus experiment, cell, two axes, replicate, status, and verified.
  EXPECT_EQ(fields(header), 27U + 7U);
  EXPECT_EQ(fields(row.substr(0, row.size() - 1)), fields(header.substr(0, header.size() - 1)));
}

TEST(Batch, AnExperimentWithNoAxesIsOneCell) {
  const Fixture f =
      make("version: 1\nname: solo\nbase: base.yaml\nseeds: {first: 5, count: 2}\naxes: []\n");
  const BatchResult r = swarm::run_batch(f.experiment, f.expansion, BatchOptions{});
  ASSERT_EQ(r.rows.size(), 2U);
  EXPECT_EQ(r.rows[1].task.seed, 6U);
  EXPECT_NE(swarm::batch_csv(f.experiment, f.expansion, r).find("solo,0,1,ok"), std::string::npos);
}

TEST(Batch, DifferentSeedsGiveDifferentRunsAndTheSameSeedGivesTheSameRun) {
  const Fixture f = make();
  const BatchResult a = swarm::run_batch(f.experiment, f.expansion, BatchOptions{});
  const BatchResult b = swarm::run_batch(f.experiment, f.expansion, BatchOptions{});
  EXPECT_EQ(swarm::batch_csv(f.experiment, f.expansion, a),
            swarm::batch_csv(f.experiment, f.expansion, b));
  // Cell 3 (p30, n06) has loss, so its replicates differ in what was delivered.
  std::set<std::uint64_t> delivered;
  for (std::size_t i = 9; i < 12; ++i) {
    delivered.insert(a.rows[i].metrics.messages_delivered);
  }
  EXPECT_GT(delivered.size(), 1U);
}

TEST(Batch, BadArgumentsAreRejected) {
  const Fixture f = make();
  BatchOptions o;
  o.jobs = 0;
  EXPECT_THROW(swarm::run_batch(f.experiment, f.expansion, o), std::invalid_argument);
  o.jobs = 257;
  EXPECT_THROW(swarm::run_batch(f.experiment, f.expansion, o), std::invalid_argument);
  Expansion broken;
  broken.errors.push_back({"cell x", 0, "bad"});
  EXPECT_THROW(swarm::run_batch(f.experiment, broken, BatchOptions{}), std::invalid_argument);
}
