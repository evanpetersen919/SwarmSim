"""Prints the tables of the start experiment (ADR 0020) and applies its criteria.

Usage, from the analysis directory, after running the experiment (see docs/RESULTS.md):

    python results_avoid_start.py ../results

It reads fixed_wing_avoid_start_30.csv from the directory given. Intervals are 95 percent Student t
intervals for the mean over runs. A paired difference is cleared - free over the runs that share
every other axis level and the seed.

The criteria of ADR 0020, written before the experiment was run.
  Z1  Launch contacts are 0 for cleared in every cell.
  Z2  In fog with no map on 400 and 480 m hills: total ground contacts lower with an interval
      entirely below zero.
  Z3  In clear air with no map on 400 and 480 m hills: the same.
  Z4  On the 40 m control and with the exact map: no ground contacts of cleared, and a corridor
      share not lower than free by more than 0.02.
  Z5  Reported: late contacts of both arms, and the costs.
"""

import copy
import os
import sys

from swarmsim_analysis.aggregate import paired_difference, summarize_groups
from swarmsim_analysis.table import load_table

METRICS = ["ground_contacts", "launch_contacts", "late_contacts", "orbit_corridor_share",
           "time_on_station", "orbit_radial_rmse_m", "mean_agl_m", "collision_episodes",
           "min_separation_m"]


def add_late_contacts(table):
    for row in table.rows:
        contacts = row.get("ground_contacts")
        launch = row.get("launch_contacts")
        if isinstance(contacts, (int, float)) and isinstance(launch, (int, float)):
            row["late_contacts"] = contacts - launch
        else:
            row["late_contacts"] = None
    if "late_contacts" not in table.metrics:
        table.metrics.append("late_contacts")


def part(table, **levels):
    out = copy.copy(table)
    out.rows = [r for r in table.rows if all(r[k] == v for k, v in levels.items())]
    return out


def fmt(s, digits=2):
    return f"{s.mean:+.{digits}f} [{s.ci_low:+.{digits}f}, {s.ci_high:+.{digits}f}]"


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else "../results"
    table = load_table(os.path.join(directory, "fixed_wing_avoid_start_30.csv"))
    add_late_contacts(table)
    print(f"Start experiment  ({len(table.rows)} runs, {len(table.failed_rows())} failed)")

    for weather in table.levels("weather"):
        for amplitude in table.levels("amplitude"):
            for map_level in table.levels("map"):
                cell = part(table, weather=weather, amplitude=amplitude, map=map_level)
                groups = {g.key[0]: g for g in summarize_groups(cell, ["start"], METRICS)}
                print(f"\n {weather} {amplitude} map={map_level}")
                for arm in ("free", "cleared"):
                    values = "  ".join(f"{m}={groups[arm].metrics[m].mean:.2f}" for m in METRICS)
                    print(f"  {arm:8s} {values}")
                total = paired_difference(cell, "start", "cleared", "free", "ground_contacts")
                late = paired_difference(cell, "start", "cleared", "free", "late_contacts")
                corridor = paired_difference(cell, "start", "cleared", "free",
                                             "orbit_corridor_share")
                print(f"  cleared - free: total {fmt(total)}  late {fmt(late)}  "
                      f"corridor {fmt(corridor, 3)}")

    print("\nCriteria of ADR 0020 (cleared against free):")
    worst_launch = 0.0
    for row in table.rows:
        if row["start"] == "cleared" and isinstance(row.get("launch_contacts"), (int, float)):
            worst_launch = max(worst_launch, row["launch_contacts"])
    print(f"  Z1: {'pass' if worst_launch == 0.0 else 'FAIL'}  most launch contacts in any run "
          f"{worst_launch:.0f}")
    for weather, label in (("fog", "Z2"), ("clear", "Z3")):
        for amplitude in ("a400", "a480"):
            cell = part(table, weather=weather, amplitude=amplitude, map="none")
            d = paired_difference(cell, "start", "cleared", "free", "ground_contacts")
            print(f"  {label} {weather} {amplitude}: {'pass' if d.ci_high < 0.0 else 'FAIL'}  "
                  f"total {fmt(d)}")
    worst_total = 0.0
    worst_gap = 0.0
    for weather in table.levels("weather"):
        for amplitude, map_level in [("a40", "none"), ("a40", "exact"), ("a400", "exact"),
                                     ("a480", "exact")]:
            cell = part(table, weather=weather, amplitude=amplitude, map=map_level)
            groups = {g.key[0]: g for g in summarize_groups(cell, ["start"], METRICS)}
            worst_total = max(worst_total, groups["cleared"].metrics["ground_contacts"].mean)
            worst_gap = min(worst_gap, groups["cleared"].metrics["orbit_corridor_share"].mean
                            - groups["free"].metrics["orbit_corridor_share"].mean)
    print(f"  Z4 (40 m control and exact map): "
          f"{'pass' if worst_total == 0.0 and worst_gap >= -0.02 else 'FAIL'}  most ground "
          f"contacts of cleared {worst_total:.2f}, worst corridor gap {worst_gap:+.3f}")


if __name__ == "__main__":
    main()
