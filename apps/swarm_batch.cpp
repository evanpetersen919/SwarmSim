// Runs an experiment: every cell for every seed, in parallel, and writes one results table.
//
//   swarm_batch EXPERIMENT --out FILE [--jobs N] [--seeds N] [--verify-every N] [--keep-logs DIR]
//
// The table is the same for any --jobs. A failed run is a row with a status, not a missing row.
// A text file next to the table, FILE.meta, records the experiment and the build.
//
// Exit status: 0 every run succeeded, 1 some run failed, 2 usage or input error.

#include <charconv>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <mutex>
#include <string>
#include <system_error>
#include <vector>

#include "swarm/experiment/batch.hpp"

namespace {

constexpr int kExitOk = 0;
constexpr int kExitFailedRuns = 1;
constexpr int kExitUsage = 2;

namespace fs = std::filesystem;

void print_usage() {
  std::cout << "usage: swarm_batch EXPERIMENT --out FILE [options]\n"
               "  --out FILE            the results table (CSV); FILE.meta describes the run\n"
               "  --jobs N (1)          worker threads; the table does not depend on it\n"
               "  --seeds N             override the experiment's seed count\n"
               "  --verify-every N (10) re-execute and compare every Nth run's log; 0 none, 1 all\n"
               "  --keep-logs DIR       write each run's log to DIR\n"
               "exit status: 0 all runs succeeded, 1 a run failed, 2 usage or input error\n";
}

template <typename T>
bool parse(const char* text, T& out) {
  const char* end = text + std::strlen(text);
  const auto result = std::from_chars(text, end, out);
  return result.ec == std::errc() && result.ptr == end;
}

bool read_text(const fs::path& path, std::string& text) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return false;
  }
  text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  return true;
}

bool write_bytes(const fs::path& path, const char* data, std::size_t size) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out.write(data, static_cast<std::streamsize>(size));
  return static_cast<bool>(out);
}

const char* platform_name() {
#if defined(_WIN32)
  return "Windows";
#elif defined(__linux__)
  return "Linux";
#elif defined(__APPLE__)
  return "macOS";
#else
  return "unknown";
#endif
}

const char* compiler_name() {
#if defined(_MSC_VER) && !defined(__clang__)
  return "MSVC";
#elif defined(__clang__)
  return "Clang";
#elif defined(__GNUC__)
  return "GCC";
#else
  return "unknown";
#endif
}

}  // namespace

int main(int argc, char** argv) {
  std::string experiment_path;
  std::string out_path;
  std::string logs_dir;
  swarm::BatchOptions options;
  std::uint32_t seeds_override = 0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      print_usage();
      return kExitOk;
    }
    if (arg.rfind("--", 0) != 0) {
      if (!experiment_path.empty()) {
        std::cerr << "swarm_batch: unexpected argument " << arg << "\n";
        return kExitUsage;
      }
      experiment_path = arg;
      continue;
    }
    if (i + 1 >= argc) {
      std::cerr << "swarm_batch: missing value for " << arg << "\n";
      return kExitUsage;
    }
    const char* value = argv[++i];
    bool ok = true;
    if (arg == "--out") {
      out_path = value;
    } else if (arg == "--jobs") {
      ok = parse(value, options.jobs);
    } else if (arg == "--seeds") {
      ok = parse(value, seeds_override) && seeds_override >= 1;
    } else if (arg == "--verify-every") {
      ok = parse(value, options.verify_every);
    } else if (arg == "--keep-logs") {
      logs_dir = value;
    } else {
      std::cerr << "swarm_batch: unknown option " << arg << "\n";
      return kExitUsage;
    }
    if (!ok) {
      std::cerr << "swarm_batch: invalid value '" << value << "' for " << arg << "\n";
      return kExitUsage;
    }
  }
  if (experiment_path.empty() || out_path.empty()) {
    std::cerr << "swarm_batch: needs an experiment file and --out FILE\n";
    return kExitUsage;
  }
  if (options.jobs < 1 || options.jobs > 256) {
    std::cerr << "swarm_batch: --jobs must be between 1 and 256\n";
    return kExitUsage;
  }

  std::string text;
  if (!read_text(experiment_path, text)) {
    std::cerr << "swarm_batch: cannot read " << experiment_path << "\n";
    return kExitUsage;
  }
  const fs::path base_dir = fs::path(experiment_path).parent_path();
  const swarm::ReadFile read_file = [&base_dir](const std::string& path, std::string& out) {
    return read_text(base_dir / path, out);
  };
  swarm::LoadedExperiment loaded = swarm::load_experiment(text, read_file);
  if (!loaded.ok()) {
    for (const swarm::ExperimentError& e : loaded.errors) {
      std::cerr << experiment_path << ": " << swarm::to_string(e) << "\n";
    }
    return kExitUsage;
  }
  if (seeds_override > 0) {
    loaded.experiment.seed_count = seeds_override;
  }
  const swarm::Experiment& experiment = loaded.experiment;
  const swarm::Expansion expansion = swarm::expand(experiment);
  if (!expansion.ok()) {
    for (const swarm::ExperimentError& e : expansion.errors) {
      std::cerr << experiment_path << ": " << swarm::to_string(e) << "\n";
    }
    return kExitUsage;
  }

  if (!logs_dir.empty()) {
    std::error_code ec;
    fs::create_directories(logs_dir, ec);
    if (ec) {
      std::cerr << "swarm_batch: cannot create " << logs_dir << "\n";
      return kExitUsage;
    }
    options.log_sink = [&](const swarm::BatchTask& task, const std::vector<std::uint8_t>& log) {
      const swarm::Cell& cell = expansion.cells[static_cast<std::size_t>(task.cell)];
      const fs::path path = fs::path(logs_dir) / (experiment.name + "." + cell.name + ".s" +
                                                  std::to_string(task.seed) + ".swlog");
      write_bytes(path, reinterpret_cast<const char*>(log.data()), log.size());
    };
  }

  std::mutex print_mutex;
  options.progress = [&print_mutex](std::uint64_t done, std::uint64_t total) {
    if (done % 25 == 0 || done == total) {
      const std::lock_guard<std::mutex> lock(print_mutex);
      std::cerr << "swarm_batch: " << done << " of " << total << " runs\n";
    }
  };

  const std::uint64_t total = expansion.cells.size() * experiment.seed_count;
  std::cerr << "swarm_batch: " << expansion.cells.size() << " cells, " << experiment.seed_count
            << " seeds, " << total << " runs, " << options.jobs << " jobs\n";

  swarm::BatchResult result;
  try {
    result = swarm::run_batch(experiment, expansion, options);
  } catch (const std::exception& e) {
    std::cerr << "swarm_batch: " << e.what() << "\n";
    return kExitUsage;
  }

  const std::string table = swarm::batch_csv(experiment, expansion, result);
  if (!write_bytes(out_path, table.data(), table.size())) {
    std::cerr << "swarm_batch: cannot write " << out_path << "\n";
    return kExitUsage;
  }

  std::string meta = "swarm_batch " + std::string(SWARM_VERSION) + "\n";
  meta += std::string("platform: ") + platform_name() + ", " + compiler_name() + "\n";
  meta += "cells: " + std::to_string(expansion.cells.size()) + "\n";
  meta += "seeds: " + std::to_string(experiment.seed_count) + " from " +
          std::to_string(experiment.first_seed) + "\n";
  meta += "runs: " + std::to_string(total) + "\n";
  meta += "failed: " + std::to_string(result.failed) + "\n";
  meta += "verify_every: " + std::to_string(options.verify_every) + "\n";
  meta += "\nexperiment file:\n" + text;
  write_bytes(out_path + ".meta", meta.data(), meta.size());

  std::cout << "wrote " << result.rows.size() << " rows to " << out_path << " (" << result.failed
            << " failed)\n";
  return result.failed == 0 ? kExitOk : kExitFailedRuns;
}
