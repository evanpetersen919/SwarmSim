#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "swarm/experiment/experiment.hpp"
#include "swarm/scenario/metrics.hpp"

namespace swarm {

// One run of one cell with one seed. Tasks are numbered cell by cell: index = cell * seeds +
// replicate.
struct BatchTask {
  std::uint64_t index = 0;
  std::uint64_t cell = 0;
  std::uint32_t replicate = 0;
  std::uint64_t seed = 0;
};

struct BatchOptions {
  // Worker threads, 1 to 256. The result does not depend on it.
  std::uint32_t jobs = 1;
  // Re-execute and compare the log of every Nth task, by task index: 0 for none, 1 for all.
  std::uint32_t verify_every = 10;
  // Called with each task's log once it is written, from the worker thread, so it must be safe to
  // call from several threads at once. Not called for a task that failed to run.
  std::function<void(const BatchTask&, const std::vector<std::uint8_t>&)> log_sink;
  // Called after each task with the number done and the total, under a lock. Progress only: it
  // must not influence the result.
  std::function<void(std::uint64_t done, std::uint64_t total)> progress;
};

struct BatchRow {
  BatchTask task;
  // "ok", or what went wrong: "run failed: ...", "verification failed: ...", "metrics failed: ...".
  std::string status;
  bool verified = false;
  bool has_metrics = false;
  MetricsResult metrics;
};

struct BatchResult {
  std::vector<BatchRow> rows;  // in task order, whatever order the tasks finished in
  std::uint64_t failed = 0;    // rows whose status is not "ok"
  bool ok() const { return failed == 0; }
};

// Runs every (cell, replicate) of an expanded experiment. Each task runs its scenario on one
// thread over the deterministic channel and computes the metrics from the log. Tasks share
// nothing, so the rows are the same for any number of jobs. A task that fails becomes a row with
// a status and no metrics; it does not stop the others. Throws std::invalid_argument for a job
// count out of range or an expansion that has errors.
BatchResult run_batch(const Experiment& experiment, const Expansion& expansion,
                      const BatchOptions& options);

// The header of the results table: experiment, cell, one column per axis (the level label),
// replicate, status, verified, and then the columns of metrics_csv_header.
std::string batch_csv_header(const Experiment& experiment);

// One row, ending with a line feed. A row without metrics has empty cells for them.
std::string batch_csv_row(const Experiment& experiment, const Expansion& expansion,
                          const BatchRow& row);

// The whole table: the header and every row.
std::string batch_csv(const Experiment& experiment, const Expansion& expansion,
                      const BatchResult& result);

}  // namespace swarm
