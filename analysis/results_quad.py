"""Prints the tables of docs/RESULTS.md from the result tables of the quadrotor experiments.

Usage, from the analysis directory, after running the experiments (see docs/RESULTS.md):

    python results_quad.py ../results

It reads baselines_quad_30.csv, baselines_burst_quad_30.csv, ablation_quad_30.csv, and
matrix_quad_30.csv from the directory given. Intervals are 95 percent Student t intervals for the
mean over runs. A paired difference is a - b over runs that share every other axis level and the
seed, so it removes the variation between cells.
"""

import os
import sys

from swarmsim_analysis.aggregate import paired_difference, summarize_groups
from swarmsim_analysis.table import load_table

FORMATION = "formation_rmse_settled_m"
SEPARATION = "min_separation_m"
COLLISIONS = "collision_episodes"


def show(table, by, metrics):
    for group in summarize_groups(table, by, metrics):
        cells = []
        for metric in metrics:
            s = group.metrics[metric]
            cells.append(f"{metric}={s.mean:.2f} [{s.ci_low:.2f}, {s.ci_high:.2f}]")
        print(f"  {' / '.join(group.key)}  runs={group.runs}  " + "  ".join(cells))


def paired(table, axis, a, b, metric):
    s = paired_difference(table, axis, a, b, metric)
    print(f"  {a} - {b}, {metric}: {s.mean:.3f} [{s.ci_low:.3f}, {s.ci_high:.3f}]  pairs={s.n}")


def baselines(directory, name, title):
    table = load_table(os.path.join(directory, name))
    print(f"\n{title}  ({name}, {len(table.rows)} runs, {len(table.failed_rows())} failed)")
    print(" by predictor:")
    show(table, ["predictor"], [FORMATION, SEPARATION, COLLISIONS])
    print(" by predictor and loss:")
    show(table, ["predictor", "loss"], [FORMATION, SEPARATION])
    for metric in (FORMATION, SEPARATION):
        print(f" paired differences, {metric}:")
        paired(table, "predictor", "kalman", "hold_last", metric)
        paired(table, "predictor", "dead_reckoning", "hold_last", metric)
        paired(table, "predictor", "kalman", "dead_reckoning", metric)
        paired(table, "predictor", "oracle", "kalman", metric)


def ablation(directory):
    table = load_table(os.path.join(directory, "ablation_quad_30.csv"))
    print(f"\nAblation  (ablation_quad_30.csv, {len(table.rows)} runs, "
          f"{len(table.failed_rows())} failed)")
    print(" by variant:")
    show(table, ["variant"], [FORMATION, SEPARATION, COLLISIONS])
    for metric in (FORMATION, SEPARATION, COLLISIONS):
        print(f" paired differences against the full controller, {metric}:")
        for level in table.levels("variant"):
            if level != "full":
                paired(table, "variant", level, "full", metric)


def matrix(directory):
    table = load_table(os.path.join(directory, "matrix_quad_30.csv"))
    print(f"\nMatrix  (matrix_quad_30.csv, {len(table.rows)} runs, {len(table.failed_rows())} failed)")
    metrics = [FORMATION, SEPARATION, COLLISIONS, "alive_at_end"]
    for axis in table.axes:
        print(f" by {axis}:")
        show(table, [axis], metrics)


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else "../results"
    baselines(directory, "baselines_quad_30.csv", "Baselines, independent loss")
    baselines(directory, "baselines_burst_quad_30.csv", "Baselines, bursty loss")
    ablation(directory)
    matrix(directory)


if __name__ == "__main__":
    main()
