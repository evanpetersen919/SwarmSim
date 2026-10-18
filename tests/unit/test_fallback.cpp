#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>

#include "swarm/control/fallback.hpp"
#include "swarm/rng.hpp"

using swarm::FallbackFsm;
using swarm::FallbackInputs;
using swarm::FallbackParams;
using swarm::FallbackState;

namespace {

FallbackInputs inputs(std::uint32_t fresh, std::uint32_t total, std::uint64_t since_fix,
                      double sigma) {
  FallbackInputs in;
  in.fresh_neighbors = fresh;
  in.total_neighbors = total;
  in.ticks_since_fix = since_fix;
  in.pose_sigma = sigma;
  return in;
}

FallbackInputs good() { return inputs(4, 4, 0, 0.5); }

}  // namespace

TEST(Fallback, StartsNominalAndStaysThereWhileEverythingIsFine) {
  FallbackFsm fsm{FallbackParams{}};
  EXPECT_EQ(fsm.state(), FallbackState::kNominal);
  for (std::uint64_t t = 0; t < 500; ++t) {
    EXPECT_EQ(fsm.update(good(), t), FallbackState::kNominal);
  }
}

TEST(Fallback, CommsGoesBadWhenTheFreshFractionDropsBelowTheEnterValue) {
  FallbackFsm at_threshold{FallbackParams{}};
  EXPECT_EQ(at_threshold.update(inputs(2, 4, 0, 0.5), 0),
            FallbackState::kNominal);  // 0.5 is not below
  FallbackFsm below{FallbackParams{}};
  EXPECT_EQ(below.update(inputs(1, 4, 0, 0.5), 0), FallbackState::kDegradedComms);
  EXPECT_TRUE(below.comms_bad());
  EXPECT_FALSE(below.navigation_bad());
}

TEST(Fallback, NavigationGoesBadAtTheFixAgeOrSigmaThresholds) {
  const FallbackParams p;
  const auto state_after = [&](FallbackInputs in) {
    FallbackFsm fsm{p};
    return fsm.update(in, 0);
  };
  EXPECT_EQ(state_after(inputs(4, 4, p.fix_enter_ticks, 0.5)), FallbackState::kNominal);
  EXPECT_EQ(state_after(inputs(4, 4, p.fix_enter_ticks + 1, 0.5)),
            FallbackState::kDegradedNavigation);
  EXPECT_EQ(state_after(inputs(4, 4, 0, p.sigma_enter)), FallbackState::kNominal);
  EXPECT_EQ(state_after(inputs(4, 4, 0, p.sigma_enter + 0.01)), FallbackState::kDegradedNavigation);
}

TEST(Fallback, NoExpectedNeighborsMeansCommsIsNeverBad) {
  FallbackFsm fsm{FallbackParams{}};
  for (std::uint64_t t = 0; t < 300; ++t) {
    EXPECT_EQ(fsm.update(inputs(0, 0, 0, 0.5), t), FallbackState::kNominal);
  }
  EXPECT_FALSE(fsm.comms_bad());
}

TEST(Fallback, CommsHysteresisNeedsTheExitFractionToRecover) {
  FallbackFsm fsm{FallbackParams{}};  // enter below 0.5, exit at 0.75
  std::uint64_t t = 0;
  EXPECT_EQ(fsm.update(inputs(3, 8, 0, 0.5), t), FallbackState::kDegradedComms);  // 0.375
  // Between the thresholds the condition stays bad, however long the signal sits there.
  for (int i = 0; i < 5; ++i) {
    t += 100;
    EXPECT_EQ(fsm.update(inputs(5, 8, 0, 0.5), t), FallbackState::kDegradedComms);  // 0.625
  }
  t += 100;
  EXPECT_EQ(fsm.update(inputs(6, 8, 0, 0.5), t), FallbackState::kNominal);  // 0.75
}

TEST(Fallback, NavigationHysteresisNeedsBothValuesInsideTheExitThresholds) {
  FallbackFsm fsm{FallbackParams{}};
  std::uint64_t t = 0;
  EXPECT_EQ(fsm.update(inputs(4, 4, 0, 4.0), t), FallbackState::kDegradedNavigation);
  t += 100;
  EXPECT_EQ(fsm.update(inputs(4, 4, 0, 2.0), t),
            FallbackState::kDegradedNavigation);  // sigma between
  t += 100;
  EXPECT_EQ(fsm.update(inputs(4, 4, 50, 0.5), t),
            FallbackState::kDegradedNavigation);  // fix age between
  t += 100;
  EXPECT_EQ(fsm.update(inputs(4, 4, 25, 1.5), t), FallbackState::kNominal);
}

TEST(Fallback, AWobblingSignalDoesNotFlap) {
  FallbackFsm fsm{FallbackParams{}};
  int changes = 0;
  FallbackState previous = fsm.state();
  for (std::uint64_t t = 0; t < 4000; t += 10) {
    // The fresh count alternates between 3 and 5 of 8: either side of the enter value of 0.5 but
    // always below the exit value of 0.75.
    const FallbackState s = fsm.update(inputs((t / 10) % 2 == 0 ? 3 : 5, 8, 0, 0.5), t);
    changes += s != previous ? 1 : 0;
    previous = s;
  }
  EXPECT_EQ(changes, 1);
  EXPECT_EQ(fsm.state(), FallbackState::kDegradedComms);
}

TEST(Fallback, AStateIsHeldForTheMinimumDwell) {
  const FallbackParams p;  // dwell 50
  FallbackFsm fsm{p};
  EXPECT_EQ(fsm.update(inputs(0, 4, 0, 0.5), 0), FallbackState::kDegradedComms);
  EXPECT_EQ(fsm.update(good(), 10), FallbackState::kDegradedComms);
  EXPECT_EQ(fsm.update(good(), p.min_dwell_ticks - 1), FallbackState::kDegradedComms);
  EXPECT_EQ(fsm.update(good(), p.min_dwell_ticks), FallbackState::kNominal);
}

TEST(Fallback, BothConditionsBadForTheHoldTimeEntersSafeHold) {
  const FallbackParams p;  // hold 150
  FallbackFsm fsm{p};
  const FallbackInputs both = inputs(0, 4, 500, 0.5);
  EXPECT_EQ(fsm.update(both, 0), FallbackState::kDegradedNavigation);
  EXPECT_EQ(fsm.update(both, p.hold_ticks - 1), FallbackState::kDegradedNavigation);
  EXPECT_EQ(fsm.update(both, p.hold_ticks), FallbackState::kSafeHold);
}

TEST(Fallback, TheHoldTimerRestartsWhenOneConditionRecovers) {
  const FallbackParams p;
  FallbackFsm fsm{p};
  const FallbackInputs both = inputs(0, 4, 500, 0.5);
  fsm.update(both, 0);
  fsm.update(both, 100);
  fsm.update(inputs(4, 4, 500, 0.5), 110);  // comms recovers, navigation stays bad
  fsm.update(both, 120);                    // both bad again: the timer starts at 120
  EXPECT_EQ(fsm.update(both, 120 + p.hold_ticks - 1), FallbackState::kDegradedNavigation);
  EXPECT_EQ(fsm.update(both, 120 + p.hold_ticks), FallbackState::kSafeHold);
}

TEST(Fallback, AnUncertaintyAboveTheAbortLimitEntersSafeHoldAtOnceIgnoringDwell) {
  const FallbackParams p;
  FallbackFsm fsm{p};
  EXPECT_EQ(fsm.update(inputs(0, 4, 0, 0.5), 0), FallbackState::kDegradedComms);
  EXPECT_EQ(fsm.update(inputs(0, 4, 0, p.sigma_abort), 1), FallbackState::kDegradedComms);  // dwell
  EXPECT_EQ(fsm.update(inputs(0, 4, 0, p.sigma_abort + 0.01), 2), FallbackState::kSafeHold);
}

TEST(Fallback, LeavingSafeHoldNeedsTheRecoveryWindowOfGoodConditions) {
  const FallbackParams p;  // recovery 150
  FallbackFsm fsm{p};
  EXPECT_EQ(fsm.update(inputs(4, 4, 0, 50.0), 10), FallbackState::kSafeHold);
  EXPECT_EQ(fsm.update(good(), 20), FallbackState::kSafeHold);
  EXPECT_EQ(fsm.update(good(), 20 + p.recovery_ticks - 1), FallbackState::kSafeHold);
  EXPECT_EQ(fsm.update(good(), 20 + p.recovery_ticks), FallbackState::kNominal);
}

TEST(Fallback, ABriefRelapseRestartsTheRecoveryWindow) {
  const FallbackParams p;
  FallbackFsm fsm{p};
  fsm.update(inputs(4, 4, 0, 50.0), 10);
  fsm.update(good(), 20);
  fsm.update(inputs(4, 4, 0, 4.0), 100);  // navigation bad again
  fsm.update(good(), 110);                // good from 110
  EXPECT_EQ(fsm.update(good(), 110 + p.recovery_ticks - 1), FallbackState::kSafeHold);
  EXPECT_EQ(fsm.update(good(), 110 + p.recovery_ticks), FallbackState::kNominal);
}

TEST(Fallback, BehaviorScalesFollowTheDocumentedTable) {
  FallbackFsm fsm{FallbackParams{}};
  const auto& nominal = fsm.behavior_of(FallbackState::kNominal);
  EXPECT_EQ(nominal.reference_boost, 0.0);
  EXPECT_EQ(nominal.speed_scale, 1.0);
  EXPECT_EQ(nominal.margin_scale, 1.0);
  EXPECT_EQ(nominal.gain_scale, 1.0);
  EXPECT_FALSE(nominal.hold);

  const auto& comms = fsm.behavior_of(FallbackState::kDegradedComms);
  EXPECT_GT(comms.reference_boost, 0.0);  // more weight on the mission reference
  EXPECT_LT(comms.speed_scale, 1.0);
  EXPECT_GT(comms.margin_scale, 1.0);

  const auto& nav = fsm.behavior_of(FallbackState::kDegradedNavigation);
  EXPECT_LT(nav.speed_scale, 1.0);
  EXPECT_GT(nav.margin_scale, 1.0);
  EXPECT_LT(nav.gain_scale, 1.0);

  const auto& hold = fsm.behavior_of(FallbackState::kSafeHold);
  EXPECT_TRUE(hold.hold);
  EXPECT_EQ(hold.speed_scale, 0.0);
  EXPECT_GE(hold.margin_scale, comms.margin_scale);

  fsm.update(inputs(0, 4, 0, 0.5), 0);
  EXPECT_EQ(&fsm.behavior(), &fsm.behavior_of(FallbackState::kDegradedComms));
}

TEST(Fallback, InvalidParametersAreRejected) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto rejects = [](FallbackParams p) {
    EXPECT_THROW(FallbackFsm{p}, std::invalid_argument);
  };
  FallbackParams p;
  p.comms_exit_fraction = 0.4;  // below the enter value
  rejects(p);
  p = FallbackParams{};
  p.comms_enter_fraction = 1.5;
  rejects(p);
  p = FallbackParams{};
  p.fix_exit_ticks = p.fix_enter_ticks + 1;
  rejects(p);
  p = FallbackParams{};
  p.sigma_exit = p.sigma_enter + 1.0;
  rejects(p);
  p = FallbackParams{};
  p.sigma_abort = p.sigma_enter;
  rejects(p);
  p = FallbackParams{};
  p.sigma_enter = nan;
  rejects(p);
  p = FallbackParams{};
  p.degraded_comms.speed_scale = -0.1;
  rejects(p);
  p = FallbackParams{};
  p.safe_hold.gain_scale = nan;
  rejects(p);
}

TEST(Fallback, InvalidInputsAreRejected) {
  FallbackFsm fsm{FallbackParams{}};
  fsm.update(good(), 10);
  EXPECT_THROW(fsm.update(good(), 9), std::invalid_argument);
  EXPECT_THROW(fsm.update(inputs(5, 4, 0, 0.5), 11), std::invalid_argument);
  EXPECT_THROW(fsm.update(inputs(4, 4, 0, -1.0), 11), std::invalid_argument);
  EXPECT_THROW(fsm.update(inputs(4, 4, 0, std::numeric_limits<double>::quiet_NaN()), 11),
               std::invalid_argument);
}

TEST(Fallback, NoTransitionOutOfAStateBeatsTheDwellExceptIntoSafeHold) {
  const FallbackParams p;
  for (std::uint64_t seed = 0; seed < 20; ++seed) {
    swarm::Rng rng(seed, 0x600);
    FallbackFsm fsm{p};
    FallbackState previous = fsm.state();
    std::uint64_t changed_at = 0;
    bool changed = false;
    for (std::uint64_t t = 0; t < 5000; t += 1 + rng.next_u32() % 7) {
      const std::uint32_t total = rng.next_u32() % 6;
      const std::uint32_t fresh = total == 0 ? 0 : rng.next_u32() % (total + 1);
      const FallbackInputs in = inputs(fresh, total, rng.next_u32() % 300, rng.uniform(0.0, 20.0));
      const FallbackState s = fsm.update(in, t);
      if (s != previous) {
        if (changed && s != FallbackState::kSafeHold) {
          ASSERT_GE(t - changed_at, p.min_dwell_ticks) << "seed " << seed << " tick " << t;
        }
        changed = true;
        changed_at = t;
        previous = s;
      }
    }
  }
}
