#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

#include "swarm/scenario/metrics.hpp"
#include "swarm/scenario/metrics_csv.hpp"

using swarm::MetricsResult;
using swarm::Recovery;

namespace {

// A small CSV reader for one line: splits on commas outside quotes and undoes the quoting.
std::vector<std::string> split_row(const std::string& line) {
  std::vector<std::string> cells;
  std::string cell;
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (quoted) {
      if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') {
        cell += '"';
        ++i;
      } else if (c == '"') {
        quoted = false;
      } else {
        cell += c;
      }
    } else if (c == '"') {
      quoted = true;
    } else if (c == ',') {
      cells.push_back(cell);
      cell.clear();
    } else if (c != '\n') {
      cell += c;
    }
  }
  cells.push_back(cell);
  return cells;
}

MetricsResult known() {
  MetricsResult r;
  r.scenario_name = "demo";
  r.seed = 7;
  r.drones = 8;
  r.dt = 0.02;
  r.ticks = 3000;
  r.formation_rmse = 0.25;
  r.formation_rmse_settled = 0.125;
  r.alive_at_end = 7;
  r.min_separation = 2.5;
  r.collision_episodes = 0;
  r.recoveries = {{swarm::DisturbanceKind::kJamEnd, 400, 4.5, false},
                  {swarm::DisturbanceKind::kFault, 1500, 12.5, true}};
  r.recovery_longest = 12.5;
  r.any_recovery_censored = true;
  r.messages_sent = 1440;
  r.messages_delivered = 1000;
  r.messages_in_flight = 10;
  r.dropped_loss = 300;
  r.dropped_rate = 100;
  r.dropped_jam = 30;
  r.delivery_ratio = 0.7;
  r.latency_p50_s = 0.04;
  r.latency_p95_s = 0.06;
  return r;
}

MetricsResult nothing() {
  MetricsResult r;
  r.scenario_name = "empty";
  r.seed = 1;
  r.drones = 1;
  r.dt = 0.5;
  r.ticks = 4;
  return r;
}

}  // namespace

TEST(MetricsCsv, TheHeaderIsExactlyTheDocumentedColumns) {
  EXPECT_EQ(swarm::metrics_csv_header(),
            "scenario,seed,drones,dt_s,ticks,duration_s,"
            "formation_rmse_m,formation_rmse_settled_m,alive_at_end,"
            "min_separation_m,collision_episodes,"
            "recovery_count,recovery_longest_s,recovery_censored,"
            "messages_sent,messages_delivered,messages_in_flight,"
            "dropped_loss,dropped_rate,dropped_jam,"
            "delivery_ratio,latency_p50_s,latency_p95_s\n");
}

TEST(MetricsCsv, TheRowForAKnownResultIsExactlyThisText) {
  EXPECT_EQ(swarm::metrics_csv_row(known()),
            "demo,7,8,0.02,3000,60,0.25,0.125,7,2.5,0,2,12.5,1,1440,1000,10,300,100,30,0.7,0.04,"
            "0.06\n");
}

TEST(MetricsCsv, ValuesThatDoNotExistAreEmptyCellsAndNotZeros) {
  // No state, no disturbance, no messages: every optional cell is empty; counts are zero.
  EXPECT_EQ(swarm::metrics_csv_row(nothing()), "empty,1,1,0.5,4,2,,,0,,0,0,,,0,0,0,0,0,0,,,\n");
}

TEST(MetricsCsv, TheCensoringFlagIsOneZeroOrEmpty) {
  MetricsResult r = known();
  EXPECT_EQ(split_row(swarm::metrics_csv_row(r))[13], "1");
  r.any_recovery_censored = false;
  EXPECT_EQ(split_row(swarm::metrics_csv_row(r))[13], "0");
  r.recoveries.clear();
  r.recovery_longest.reset();
  const auto cells = split_row(swarm::metrics_csv_row(r));
  EXPECT_EQ(cells[11], "0");  // recovery_count
  EXPECT_EQ(cells[12], "");   // recovery_longest_s
  EXPECT_EQ(cells[13], "");   // recovery_censored
}

TEST(MetricsCsv, EveryRowHasAsManyCellsAsTheHeader) {
  const std::size_t columns = split_row(swarm::metrics_csv_header()).size();
  EXPECT_EQ(columns, 23U);
  EXPECT_EQ(split_row(swarm::metrics_csv_row(known())).size(), columns);
  EXPECT_EQ(split_row(swarm::metrics_csv_row(nothing())).size(), columns);
}

TEST(MetricsCsv, LinesEndWithALineFeedSoFilesCanBeConcatenated) {
  const std::string header = swarm::metrics_csv_header();
  const std::string row = swarm::metrics_csv_row(known());
  EXPECT_EQ(header.back(), '\n');
  EXPECT_EQ(row.back(), '\n');
  EXPECT_EQ(row.find('\r'), std::string::npos);
  EXPECT_EQ(std::count(row.begin(), row.end(), '\n'), 1);
}

TEST(MetricsCsv, NamesWithSeparatorsOrQuotesAreQuotedAndComeBackIntact) {
  for (const std::string name :
       {"plain", "a,b", "say \"hi\"", "line\nbreak", "\"", ",", "a,\"b\",c"}) {
    MetricsResult r = known();
    r.scenario_name = name;
    const std::string row = swarm::metrics_csv_row(r);
    // The embedded line feed is inside quotes, so the only unquoted line feed is the last one.
    const std::vector<std::string> cells = split_row(row.substr(0, row.size() - 1));
    ASSERT_EQ(cells.size(), 23U) << name;
    EXPECT_EQ(cells[0], name);
    EXPECT_EQ(cells[1], "7");
  }
  MetricsResult r = known();
  r.scenario_name = "plain";
  EXPECT_EQ(swarm::metrics_csv_row(r).substr(0, 6), "plain,");  // no quotes when none are needed
}

TEST(MetricsCsv, NumbersRoundTripToNineSignificantDigits) {
  MetricsResult r = known();
  r.dt = 1.0 / 3.0;
  r.formation_rmse = 1.0 / 7.0;
  r.formation_rmse_settled = 123456.789012345;
  r.min_separation = 1.0e-7;
  r.delivery_ratio = 2.0 / 3.0;
  r.latency_p50_s = 0.123456789123;
  r.latency_p95_s = 9.87654321987e5;
  r.recovery_longest = 12.34567890123;
  const std::vector<std::string> cells = split_row(swarm::metrics_csv_row(r));

  const auto near = [](const std::string& cell, double expected) {
    const double parsed = std::strtod(cell.c_str(), nullptr);
    return std::abs(parsed - expected) <= 1e-8 * std::abs(expected);
  };
  EXPECT_TRUE(near(cells[3], r.dt)) << cells[3];
  EXPECT_TRUE(near(cells[6], *r.formation_rmse)) << cells[6];
  EXPECT_TRUE(near(cells[7], *r.formation_rmse_settled)) << cells[7];
  EXPECT_TRUE(near(cells[9], *r.min_separation)) << cells[9];
  EXPECT_TRUE(near(cells[12], *r.recovery_longest)) << cells[12];
  EXPECT_TRUE(near(cells[20], *r.delivery_ratio)) << cells[20];
  EXPECT_TRUE(near(cells[21], *r.latency_p50_s)) << cells[21];
  EXPECT_TRUE(near(cells[22], *r.latency_p95_s)) << cells[22];
  // Whole numbers are exact.
  EXPECT_EQ(cells[1], "7");
  EXPECT_EQ(cells[4], "3000");
  EXPECT_EQ(cells[14], "1440");
}

TEST(MetricsCsv, LargeWholeNumbersAreNotMangledByNumberFormatting) {
  MetricsResult r = known();
  r.seed = 18446744073709551615ULL;
  r.messages_sent = 123456789012ULL;
  const std::vector<std::string> cells = split_row(swarm::metrics_csv_row(r));
  EXPECT_EQ(cells[1], "18446744073709551615");
  EXPECT_EQ(cells[14], "123456789012");
}
