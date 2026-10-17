#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace swarm {

// Runs one batch of independent tasks across a fixed set of threads and returns only when every
// task has finished, which is the per-tick barrier of the simulation loop.
//
// Which thread runs which index is not fixed and must not matter: each task may touch only state
// owned by its own index. Calls to for_each() must not overlap or be nested.
class LockstepPool {
 public:
  // `num_threads` counts the calling thread, so a pool of N starts N - 1 workers. A pool of 1
  // runs everything inline on the caller.
  explicit LockstepPool(std::size_t num_threads);
  ~LockstepPool();

  LockstepPool(const LockstepPool&) = delete;
  LockstepPool& operator=(const LockstepPool&) = delete;

  std::size_t thread_count() const { return workers_.size() + 1; }

  // Calls fn(i) exactly once for each i in [0, count), then returns. If any call throws, the
  // remaining tasks still run, and the exception from the lowest index is rethrown afterwards.
  void for_each(std::size_t count, const std::function<void(std::size_t)>& fn);

 private:
  void worker_loop();
  void run_indices();
  void stop_workers();

  std::vector<std::thread> workers_;

  std::mutex mutex_;
  std::condition_variable work_cv_;
  std::condition_variable done_cv_;
  const std::function<void(std::size_t)>* fn_ = nullptr;
  std::size_t count_ = 0;
  std::size_t workers_remaining_ = 0;
  std::uint64_t generation_ = 0;
  bool stop_ = false;

  std::atomic<std::size_t> next_{0};

  std::mutex error_mutex_;
  std::exception_ptr error_;
  std::size_t error_index_ = 0;
};

}  // namespace swarm
