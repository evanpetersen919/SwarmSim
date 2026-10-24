"""Prints the tables of the ladder experiment (fixed_wing_final.yaml).

Usage, from the analysis directory, after running the experiment (see docs/RESULTS.md):

    python results_final.py ../results

It reads fixed_wing_final_30.csv from the directory given. Intervals are 95 percent Student t
intervals for the mean over runs. A paired difference is a - b over the runs that share every other
axis level and the seed. There are no criteria: the experiment describes the stages of the avoidance
work on one setup. The steep cells with no map (400 and 480 m hills) carry the headline.
"""

import copy
import os
import sys

from swarmsim_analysis.aggregate import paired_difference, summarize_groups
from swarmsim_analysis.table import load_table

ARMS = ["forward", "avoid", "all3", "turnfirst", "cap3", "urgent6", "share"]
METRICS = ["ground_contacts", "late_contacts", "orbit_corridor_share", "time_on_station",
           "orbit_radial_rmse_m", "mean_agl_m", "collision_episodes", "min_separation_m"]


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
    table = load_table(os.path.join(directory, "fixed_wing_final_30.csv"))
    add_late_contacts(table)
    print(f"Ladder experiment  ({len(table.rows)} runs, {len(table.failed_rows())} failed)")
    for weather in table.levels("weather"):
        for amplitude in table.levels("amplitude"):
            for map_level in table.levels("map"):
                cell = part(table, weather=weather, amplitude=amplitude, map=map_level)
                groups = {g.key[0]: g for g in summarize_groups(cell, ["arm"], METRICS)}
                print(f"\n {weather} {amplitude} map={map_level}")
                for arm in ARMS:
                    values = "  ".join(f"{m[:12]}={groups[arm].metrics[m].mean:.2f}"
                                       for m in METRICS)
                    print(f"  {arm:10s} {values}")
                if map_level == "none" and amplitude != "a40":
                    for a, b in zip(ARMS[1:], ARMS[:-1]):
                        total = paired_difference(cell, "arm", a, b, "ground_contacts")
                        corridor = paired_difference(cell, "arm", a, b, "orbit_corridor_share")
                        print(f"   {a} - {b}: total {fmt(total)}  corridor {fmt(corridor, 3)}")
                    total = paired_difference(cell, "arm", "share", "forward", "ground_contacts")
                    print(f"   share - forward: total {fmt(total)}")


if __name__ == "__main__":
    main()
