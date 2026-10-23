"""Prints the tables of the forward obstacle sensor experiment (ADR 0013).

Usage, from the analysis directory, after running the experiments (see docs/RESULTS.md):

    python results_forward.py ../results

It reads fixed_wing_forward_30.csv and fixed_wing_forward_sensitivity_30.csv from the directory
given. Intervals are 95 percent Student t intervals for the mean over runs. A paired difference is
a - b over the runs that share every other axis level and the seed. Safety measures are
ground_contacts (aircraft of twelve that reached the ground) and min_agl_m; cost measures are
mean_agl_m, orbit_altitude_rmse_m, and time_on_station.
"""

import copy
import os
import sys

from swarmsim_analysis.aggregate import paired_difference, summarize_groups
from swarmsim_analysis.table import load_table

SAFETY = ["ground_contacts", "min_agl_m"]
COST = ["mean_agl_m", "orbit_altitude_rmse_m", "time_on_station"]
ALL = SAFETY + COST


def part(table, **levels):
    out = copy.copy(table)
    out.rows = [r for r in table.rows if all(r[k] == v for k, v in levels.items())]
    return out


def pair(table, axis, a, b, metric):
    s = paired_difference(table, axis, a, b, metric)
    return f"{s.mean:+.2f} [{s.ci_low:+.2f}, {s.ci_high:+.2f}]"


def main_experiment(directory):
    path = os.path.join(directory, "fixed_wing_forward_30.csv")
    if not os.path.exists(path):
        return
    table = load_table(path)
    print(f"\nForward sensor experiment  ({len(table.rows)} runs, {len(table.failed_rows())} failed)")
    for weather in table.levels("weather"):
        for map_level in table.levels("map"):
            cell = part(table, weather=weather, map=map_level)
            print(f"\n weather={weather} map={map_level}")
            for g in summarize_groups(cell, ["sensors"], ALL):
                values = "  ".join(f"{m}={g.metrics[m].mean:.2f}" for m in ALL)
                print(f"  {g.key[0]:9s} runs={g.runs}  {values}")
            for a, b in (("altimeter", "none"), ("forward", "none"), ("both", "none"),
                         ("forward", "altimeter")):
                cells = "  ".join(f"{m} {pair(cell, 'sensors', a, b, m)}" for m in ALL)
                print(f"  {a} - {b}: {cells}")


def sensitivity(directory):
    path = os.path.join(directory, "fixed_wing_forward_sensitivity_30.csv")
    if not os.path.exists(path):
        return
    table = load_table(path)
    print(f"\nForward sensor sensitivity  ({len(table.rows)} runs, {len(table.failed_rows())} failed)")
    for map_level in table.levels("map"):
        print(f"\n map={map_level}: mean ground_contacts / mean_agl_m by range x dropout")
        for g in summarize_groups(part(table, map=map_level), ["range", "dropout"],
                                  ["ground_contacts", "mean_agl_m"]):
            print(f"  {g.key[0]:6s} {g.key[1]:4s} runs={g.runs}  "
                  f"contacts={g.metrics['ground_contacts'].mean:.2f}  "
                  f"mean_agl={g.metrics['mean_agl_m'].mean:.1f}")


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else "../results"
    main_experiment(directory)
    sensitivity(directory)


if __name__ == "__main__":
    main()
