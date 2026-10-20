#include "swarm/experiment/batch.hpp"

#include <algorithm>
#include <atomic>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "swarm/scenario/metrics_csv.hpp"
#include "swarm/scenario/run.hpp"

namespace swarm {

namespace {

constexpr std::uint32_t kMaxJobs = 256;

BatchRow run_task(const Experiment& experiment, const Cell& cell, const BatchTask& task,
                  const BatchOptions& options) {
  BatchRow row;
  row.task = task;
  try {
    const Scenario scenario = scenario_for_replicate(experiment, cell, task.replicate);
    std::vector<std::uint8_t> log;
    const LogStatus status = run_scenario(scenario, 1, log);
    if (status != LogStatus::kOk) {
      row.status = std::string("run failed: ") + to_string(status);
      return row;
    }
    if (options.log_sink) {
      options.log_sink(task, log);
    }
    if (options.verify_every > 0 && task.index % options.verify_every == 0) {
      const VerifyResult v = verify_scenario_log(log.data(), log.size(), 1);
      if (v.status != VerifyStatus::kOk) {
        row.status = std::string("verification failed: ") + to_string(v.status);
        return row;
      }
      row.verified = true;
    }
    const MetricsOutcome outcome = analyze_log(log.data(), log.size());
    if (!outcome.ok()) {
      row.status = std::string("metrics failed: ") + to_string(outcome.status);
      return row;
    }
    row.metrics = outcome.result;
    row.has_metrics = true;
    row.status = "ok";
  } catch (const std::exception&) {
    // The text of an exception can hold anything, so the table gets a fixed phrase.
    row.status = "run failed: scenario could not be built";
  }
  return row;
}

std::string csv_field(const std::string& text) {
  if (text.find_first_of(",\"\r\n") == std::string::npos) {
    return text;
  }
  std::string out = "\"";
  for (char c : text) {
    out += c;
    if (c == '"') {
      out += '"';
    }
  }
  return out + "\"";
}

std::size_t column_count(const std::string& header_line) {
  return static_cast<std::size_t>(std::count(header_line.begin(), header_line.end(), ',')) + 1;
}

}  // namespace

BatchResult run_batch(const Experiment& experiment, const Expansion& expansion,
                      const BatchOptions& options) {
  if (options.jobs < 1 || options.jobs > kMaxJobs) {
    throw std::invalid_argument("jobs must be between 1 and " + std::to_string(kMaxJobs));
  }
  if (!expansion.ok()) {
    throw std::invalid_argument("the experiment does not expand");
  }
  const std::uint64_t seeds = experiment.seed_count;
  const std::uint64_t total = static_cast<std::uint64_t>(expansion.cells.size()) * seeds;

  BatchResult result;
  result.rows.resize(static_cast<std::size_t>(total));
  std::atomic<std::uint64_t> next{0};
  std::uint64_t done = 0;
  std::mutex progress_mutex;

  const auto worker = [&]() {
    for (;;) {
      const std::uint64_t index = next.fetch_add(1);
      if (index >= total) {
        return;
      }
      BatchTask task;
      task.index = index;
      task.cell = index / seeds;
      task.replicate = static_cast<std::uint32_t>(index % seeds);
      task.seed = experiment.first_seed + task.replicate;
      // Each task writes its own slot, so the result needs no lock.
      result.rows[static_cast<std::size_t>(index)] =
          run_task(experiment, expansion.cells[static_cast<std::size_t>(task.cell)], task, options);
      if (options.progress) {
        const std::lock_guard<std::mutex> lock(progress_mutex);
        options.progress(++done, total);
      }
    }
  };

  const std::uint64_t workers =
      std::min<std::uint64_t>(options.jobs, std::max<std::uint64_t>(total, 1));
  std::vector<std::thread> threads;
  for (std::uint64_t i = 1; i < workers; ++i) {
    threads.emplace_back(worker);
  }
  worker();
  for (std::thread& t : threads) {
    t.join();
  }

  for (const BatchRow& row : result.rows) {
    if (row.status != "ok") {
      ++result.failed;
    }
  }
  return result;
}

std::string batch_csv_header(const Experiment& experiment) {
  std::string out = "experiment,cell";
  for (const Axis& axis : experiment.axes) {
    out += "," + axis.name;
  }
  return out + ",replicate,status,verified," + metrics_csv_header();
}

std::string batch_csv_row(const Experiment& experiment, const Expansion& expansion,
                          const BatchRow& row) {
  const Cell& cell = expansion.cells[static_cast<std::size_t>(row.task.cell)];
  std::string out = csv_field(experiment.name) + "," + std::to_string(cell.index);
  for (std::size_t a = 0; a < experiment.axes.size(); ++a) {
    out += "," + csv_field(experiment.axes[a].levels[cell.levels[a]].label);
  }
  out += "," + std::to_string(row.task.replicate) + "," + csv_field(row.status) + "," +
         (row.verified ? "1" : "0") + ",";
  if (row.has_metrics) {
    return out + metrics_csv_row(row.metrics);
  }
  const std::size_t columns = column_count(metrics_csv_header());
  return out + std::string(columns - 1, ',') + "\n";
}

std::string batch_csv(const Experiment& experiment, const Expansion& expansion,
                      const BatchResult& result) {
  std::string out = batch_csv_header(experiment);
  for (const BatchRow& row : result.rows) {
    out += batch_csv_row(experiment, expansion, row);
  }
  return out;
}

}  // namespace swarm
