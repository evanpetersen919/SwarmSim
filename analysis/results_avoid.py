"""Prints the tables of the avoidance experiment (ADR 0014).

Usage, from the analysis directory, after running the experiments (see docs/RESULTS.md):

    python results_avoid.py ../results

It reads fixed_wing_avoid_30.csv and fixed_wing_avoid_sensitivity_30.csv from the directory given.
Intervals are 95 percent Student t intervals for the mean over runs. A paired difference is a - b
over the runs that share every other axis level and the seed.

Safety measures: ground_contacts (aircraft of six that reached the ground), late_contacts (those
that were not already on the ground in the first record: ground_contacts - launch_contacts), and
min_agl_m. Cost measures: time_on_station, orbit_radial_rmse_m, mean_agl_m, collision_episodes,
and min_separation_m.
"""

import copy
import os
import sys

from swarmsim_analysis.aggregate import paired_difference, summarize_groups
from swarmsim_analysis.table import load_table

SAFETY = ["ground_contacts", "late_contacts", "min_agl_m"]
COST = ["time_on_station", "orbit_radial_rmse_m", "mean_agl_m", "collision_episodes",
        "min_separation_m"]
ALL = SAFETY + COST


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


def pair(table, axis, a, b, metric):
    s = paired_difference(table, axis, a, b, metric)
    return f"{s.mean:+.2f} [{s.ci_low:+.2f}, {s.ci_high:+.2f}]"


def main_experiment(directory):
    path = os.path.join(directory, "fixed_wing_avoid_30.csv")
    if not os.path.exists(path):
        return
    table = load_table(path)
    add_late_contacts(table)
    print(f"\nAvoidance experiment  ({len(table.rows)} runs, {len(table.failed_rows())} failed)")
    for weather in table.levels("weather"):
        for map_level in table.levels("map"):
            for amplitude in table.levels("amplitude"):
                cell = part(table, weather=weather, map=map_level, amplitude=amplitude)
                print(f"\n weather={weather} map={map_level} amplitude={amplitude}")
                for g in summarize_groups(cell, ["sensors"], ALL):
                    values = "  ".join(f"{m}={g.metrics[m].mean:.2f}" for m in ALL)
                    print(f"  {g.key[0]:8s} runs={g.runs}  {values}")
                for a, b in (("forward", "none"), ("avoid", "none"), ("avoid", "forward")):
                    cells = "  ".join(f"{m} {pair(cell, 'sensors', a, b, m)}" for m in ALL)
                    print(f"  {a} - {b}: {cells}")


def sensitivity(directory):
    path = os.path.join(directory, "fixed_wing_avoid_sensitivity_30.csv")
    if not os.path.exists(path):
        return
    table = load_table(path)
    add_late_contacts(table)
    print(f"\nAvoidance sensitivity  ({len(table.rows)} runs, {len(table.failed_rows())} failed)")
    for map_level in table.levels("map"):
        print(f"\n map={map_level}: by horizon x bank fraction")
        for g in summarize_groups(part(table, map=map_level), ["horizon", "bank"], ALL):
            values = "  ".join(f"{m}={g.metrics[m].mean:.2f}" for m in ALL)
            print(f"  {g.key[0]:4s} {g.key[1]:4s} runs={g.runs}  {values}")


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else "../results"
    main_experiment(directory)
    sensitivity(directory)


if __name__ == "__main__":
    main()
