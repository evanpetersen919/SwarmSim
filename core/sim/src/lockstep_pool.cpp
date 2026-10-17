#include "swarm/lockstep_pool.hpp"

#include <stdexcept>

namespace swarm {

LockstepPool::LockstepPool(std::size_t num_threads) {
  if (num_threads < 1) {
    throw std::invalid_argument("LockstepPool: num_threads must be at least 1");
  }
  workers_.reserve(num_threads - 1);
  try {
    for (std::size_t i = 1; i < num_threads; ++i) {
      workers_.emplace_back([this] { worker_loop(); });
    }
  } catch (...) {
    stop_workers();
    throw;
  }
}

LockstepPool::~LockstepPool() { stop_workers(); }

void LockstepPool::stop_workers() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  work_cv_.notify_all();
  for (auto& worker : workers_) {
    worker.join();
  }
  workers_.clear();
}

void LockstepPool::for_each(std::size_t count, const std::function<void(std::size_t)>& fn) {
  if (count == 0) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    fn_ = &fn;
    count_ = count;
    next_.store(0, std::memory_order_relaxed);
    error_ = nullptr;
    error_index_ = 0;
    workers_remaining_ = workers_.size();
    ++generation_;
  }
  work_cv_.notify_all();

  run_indices();

  {
    std::unique_lock<std::mutex> lock(mutex_);
    done_cv_.wait(lock, [this] { return workers_remaining_ == 0; });
    fn_ = nullptr;
  }

  if (error_) {
    const std::exception_ptr error = error_;
    error_ = nullptr;
    std::rethrow_exception(error);
  }
}

void LockstepPool::worker_loop() {
  std::uint64_t seen = 0;
  for (;;) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      work_cv_.wait(lock, [this, seen] { return stop_ || generation_ != seen; });
      if (stop_) {
        return;
      }
      seen = generation_;
    }

    run_indices();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (--workers_remaining_ == 0) {
        done_cv_.notify_one();
      }
    }
  }
}

void LockstepPool::run_indices() {
  for (;;) {
    const std::size_t index = next_.fetch_add(1, std::memory_order_relaxed);
    if (index >= count_) {
      return;
    }
    try {
      (*fn_)(index);
    } catch (...) {
      std::lock_guard<std::mutex> lock(error_mutex_);
      if (!error_ || index < error_index_) {
        error_ = std::current_exception();
        error_index_ = index;
      }
    }
  }
}

}  // namespace swarm
