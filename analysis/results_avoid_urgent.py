"""Prints the tables of the urgent-turn experiment (ADR 0017) and applies its criteria.

Usage, from the analysis directory, after running the experiment (see docs/RESULTS.md):

    python results_avoid_urgent.py ../results

It reads fixed_wing_avoid_urgent_30.csv from the directory given. Intervals are 95 percent Student t
intervals for the mean over runs. A paired difference is a - b over the runs that share every other
axis level and the seed.

The criteria of ADR 0017, written before the experiment was run, for the arm urgent6 against all3.
  U1  In fog with no map on 400 and 480 m hills: late contacts lower with an interval entirely
      below zero.
  U2  On the 40 m control and with the exact map: no late contacts, and a corridor share not lower
      than all3 by more than 0.02.
  U3  In clear air with no map on 400 and 480 m hills: late contacts not higher beyond the paired
      interval (an interval entirely above zero is a harm).
  U4  Collision episodes over every cell: no increase whose interval is entirely above zero.
  U5  The costs are reported for all cells, with the sweep over urgent_s.
"""

import copy
import os
import sys

from swarmsim_analysis.aggregate import paired_difference, summarize_groups
from swarmsim_analysis.table import load_table

ARMS = ["all3", "urgent4", "urgent6", "urgent8"]
METRICS = ["late_contacts", "orbit_corridor_share", "time_on_station", "orbit_radial_rmse_m",
           "min_agl_m", "mean_agl_m", "collision_episodes", "min_separation_m"]


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
    path = os.path.join(directory, "fixed_wing_avoid_urgent_30.csv")
    table = load_table(path)
    add_late_contacts(table)
    print(f"Urgent-turn experiment  ({len(table.rows)} runs, {len(table.failed_rows())} failed)")

    for weather in table.levels("weather"):
        for amplitude in table.levels("amplitude"):
            for map_level in table.levels("map"):
                cell = part(table, weather=weather, amplitude=amplitude, map=map_level)
                groups = {g.key[0]: g for g in summarize_groups(cell, ["arm"], METRICS)}
                print(f"\n {weather} {amplitude} map={map_level}")
                for arm in ARMS:
                    values = "  ".join(f"{m}={groups[arm].metrics[m].mean:.2f}" for m in METRICS)
                    print(f"  {arm:8s} {values}")
                for arm in ARMS[1:]:
                    late = paired_difference(cell, "arm", arm, "all3", "late_contacts")
                    corridor = paired_difference(cell, "arm", arm, "all3", "orbit_corridor_share")
                    print(f"  {arm} - all3: late {fmt(late)}  corridor {fmt(corridor, 3)}")

    print("\nCriteria of ADR 0017 (urgent6 against all3):")
    for amplitude in ("a400", "a480"):
        cell = part(table, weather="fog", amplitude=amplitude, map="none")
        late = paired_difference(cell, "arm", "urgent6", "all3", "late_contacts")
        print(f"  U1 fog {amplitude}: {'pass' if late.ci_high < 0.0 else 'FAIL'}  late {fmt(late)}")
    worst_late = 0.0
    worst_gap = 0.0
    for weather in table.levels("weather"):
        for amplitude, map_level in [("a40", "none"), ("a40", "exact"), ("a400", "exact"),
                                     ("a480", "exact")]:
            cell = part(table, weather=weather, amplitude=amplitude, map=map_level)
            groups = {g.key[0]: g for g in summarize_groups(cell, ["arm"], METRICS)}
            worst_late = max(worst_late, groups["urgent6"].metrics["late_contacts"].mean)
            gap = (groups["urgent6"].metrics["orbit_corridor_share"].mean
                   - groups["all3"].metrics["orbit_corridor_share"].mean)
            worst_gap = min(worst_gap, gap)
    print(f"  U2 (40 m control and exact map): "
          f"{'pass' if worst_late == 0.0 and worst_gap >= -0.02 else 'FAIL'}  "
          f"most late contacts of urgent6 {worst_late:.2f}, worst corridor gap {worst_gap:+.3f}")
    for amplitude in ("a400", "a480"):
        cell = part(table, weather="clear", amplitude=amplitude, map="none")
        late = paired_difference(cell, "arm", "urgent6", "all3", "late_contacts")
        print(f"  U3 clear {amplitude}: {'pass' if not late.ci_low > 0.0 else 'HARM'}  "
              f"late {fmt(late)}")
    harms = []
    for weather in table.levels("weather"):
        for amplitude in table.levels("amplitude"):
            for map_level in table.levels("map"):
                cell = part(table, weather=weather, amplitude=amplitude, map=map_level)
                d = paired_difference(cell, "arm", "urgent6", "all3", "collision_episodes")
                if d.ci_low > 0.0:
                    harms.append(f"{weather} {amplitude} {map_level}: {fmt(d)}")
    print(f"  U4 (collision episodes): {'pass' if not harms else 'FAIL'}"
          + ("" if not harms else "  harms: " + "; ".join(harms)))


if __name__ == "__main__":
    main()
