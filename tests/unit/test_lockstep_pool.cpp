#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "swarm/lockstep_pool.hpp"

namespace {

const std::size_t kThreadCounts[] = {1, 2, 4, 8};

}  // namespace

TEST(LockstepPool, ReportsThreadCount) {
  for (std::size_t threads : kThreadCounts) {
    swarm::LockstepPool pool(threads);
    EXPECT_EQ(pool.thread_count(), threads);
  }
}

TEST(LockstepPool, ZeroThreadsIsRejected) {
  EXPECT_THROW(swarm::LockstepPool(0), std::invalid_argument);
}

TEST(LockstepPool, EveryIndexRunsExactlyOnce) {
  for (std::size_t threads : kThreadCounts) {
    swarm::LockstepPool pool(threads);
    for (std::size_t count : {0U, 1U, 3U, 8U, 100U, 1000U}) {
      std::vector<std::atomic<int>> hits(count);
      for (auto& h : hits) {
        h.store(0);
      }
      pool.for_each(count, [&](std::size_t i) { hits[i].fetch_add(1); });
      for (std::size_t i = 0; i < count; ++i) {
        EXPECT_EQ(hits[i].load(), 1) << "threads " << threads << " count " << count << " i " << i;
      }
    }
  }
}

TEST(LockstepPool, ReturnsOnlyAfterAllTasksFinish) {
  swarm::LockstepPool pool(4);
  for (int round = 0; round < 200; ++round) {
    std::atomic<std::size_t> finished{0};
    pool.for_each(64, [&](std::size_t) {
      volatile std::uint64_t sink = 0;
      for (int k = 0; k < 2000; ++k) {
        sink = sink + static_cast<std::uint64_t>(k);
      }
      finished.fetch_add(1);
    });
    ASSERT_EQ(finished.load(), 64U);
  }
}

TEST(LockstepPool, ManyConsecutiveCallsDoNotDeadlock) {
  for (std::size_t threads : kThreadCounts) {
    swarm::LockstepPool pool(threads);
    std::atomic<std::uint64_t> total{0};
    for (int call = 0; call < 10000; ++call) {
      pool.for_each(16, [&](std::size_t i) { total.fetch_add(i); });
    }
    EXPECT_EQ(total.load(), 10000U * (15U * 16U / 2U)) << "threads " << threads;
  }
}

TEST(LockstepPool, ResultsDoNotDependOnThreadCount) {
  const auto compute = [](std::size_t threads) {
    swarm::LockstepPool pool(threads);
    std::vector<std::uint64_t> out(500, 0);
    for (int round = 0; round < 20; ++round) {
      pool.for_each(out.size(), [&](std::size_t i) { out[i] = out[i] * 31U + i * i + 7U; });
    }
    return out;
  };
  const auto reference = compute(1);
  for (std::size_t threads : {2U, 4U, 8U}) {
    EXPECT_EQ(compute(threads), reference) << "threads " << threads;
  }
}

TEST(LockstepPool, LowestIndexExceptionIsRethrownAndAllTasksStillRun) {
  for (std::size_t threads : kThreadCounts) {
    swarm::LockstepPool pool(threads);
    std::atomic<int> ran{0};
    std::string what;
    try {
      pool.for_each(50, [&](std::size_t i) {
        ran.fetch_add(1);
        if (i == 7 || i == 3 || i == 40) {
          throw std::runtime_error("task " + std::to_string(i));
        }
      });
    } catch (const std::runtime_error& e) {
      what = e.what();
    }
    EXPECT_EQ(what, "task 3") << "threads " << threads;
    EXPECT_EQ(ran.load(), 50) << "threads " << threads;
  }
}

TEST(LockstepPool, IsUsableAfterAnException) {
  swarm::LockstepPool pool(4);
  EXPECT_THROW(pool.for_each(10, [](std::size_t) { throw std::runtime_error("boom"); }),
               std::runtime_error);
  std::atomic<int> ran{0};
  pool.for_each(10, [&](std::size_t) { ran.fetch_add(1); });
  EXPECT_EQ(ran.load(), 10);
}

TEST(LockstepPool, ConstructAndDestroyRepeatedly) {
  for (int i = 0; i < 50; ++i) {
    swarm::LockstepPool pool(4);
    if (i % 2 == 0) {
      std::atomic<int> ran{0};
      pool.for_each(8, [&](std::size_t) { ran.fetch_add(1); });
      ASSERT_EQ(ran.load(), 8);
    }
  }
}
