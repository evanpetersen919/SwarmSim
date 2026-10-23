"""Prints the tables of the rejoin experiment (ADR 0015) and applies its criteria.

Usage, from the analysis directory, after running the experiments (see docs/RESULTS.md):

    python results_avoid_rejoin.py ../results

It reads fixed_wing_avoid_rejoin_30.csv and fixed_wing_avoid_clearance_30.csv from the directory
given. Intervals are 95 percent Student t intervals for the mean over runs. A paired difference is
a - b over the runs that share every other axis level and the seed.

The criteria of ADR 0015, written before the experiment was run. An arm is better than `avoid` if
  (A) on the steep hills with no map (400 and 480 m, clear air): its late contacts are not higher
      than those of `avoid` beyond the paired interval (an increase whose interval is above zero is
      a loss of safety), its orbit_corridor_share is higher with an interval above zero, and its
      orbit_radial_rmse_m is lower with an interval below zero; and
  (B) with an exact map: its time_on_station is at least 0.90 at every steepness level, and its
      corridor share is not below that of the forward sensor alone by more than 0.05 (as a mean over
      the steepness levels).
"""

import copy
import os
import sys

from swarmsim_analysis.aggregate import paired_difference, summarize_groups
from swarmsim_analysis.table import load_table

ARMS = ["forward", "avoid", "rejoin", "kinematic", "clear30", "all3", "h6"]
METRICS = ["late_contacts", "orbit_corridor_share", "time_on_station", "orbit_radial_rmse_m",
           "min_agl_m", "mean_agl_m", "collision_episodes"]


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


def diff(table, axis, a, b, metric):
    return paired_difference(table, axis, a, b, metric)


def means(cell, axis, metrics):
    return {g.key[0]: g for g in summarize_groups(cell, [axis], metrics)}


def judge(table, arm):
    """The criteria for `arm` against `avoid`: a list of (name, passed, detail)."""
    out = []
    for amplitude in ("a400", "a480"):
        cell = part(table, weather="clear", map="none", amplitude=amplitude)
        contacts = diff(cell, "arm", arm, "avoid", "late_contacts")
        corridor = diff(cell, "arm", arm, "avoid", "orbit_corridor_share")
        radial = diff(cell, "arm", arm, "avoid", "orbit_radial_rmse_m")
        out.append((f"A {amplitude}: no loss of safety", not (contacts.ci_low > 0.0),
                    f"late contacts {contacts.mean:+.2f} [{contacts.ci_low:+.2f}, {contacts.ci_high:+.2f}]"))
        out.append((f"A {amplitude}: corridor share higher", corridor.ci_low > 0.0,
                    f"{corridor.mean:+.3f} [{corridor.ci_low:+.3f}, {corridor.ci_high:+.3f}]"))
        out.append((f"A {amplitude}: radial error lower", radial.ci_high < 0.0,
                    f"{radial.mean:+.1f} [{radial.ci_low:+.1f}, {radial.ci_high:+.1f}] m"))
    time_levels = []
    corridor_gaps = []
    for amplitude in table.levels("amplitude"):
        cell = part(table, weather="clear", map="exact", amplitude=amplitude)
        g = means(cell, "arm", ["time_on_station", "orbit_corridor_share"])
        time_levels.append(g[arm].metrics["time_on_station"].mean)
        corridor_gaps.append(g[arm].metrics["orbit_corridor_share"].mean
                             - g["forward"].metrics["orbit_corridor_share"].mean)
    out.append(("B exact map: time on station at least 0.90 everywhere", min(time_levels) >= 0.90,
                "minimum " + f"{min(time_levels):.2f}"))
    gap = sum(corridor_gaps) / len(corridor_gaps)
    out.append(("B exact map: corridor share within 0.05 of the forward sensor", gap >= -0.05,
                f"mean difference {gap:+.3f}"))
    return out


def main_experiment(directory):
    path = os.path.join(directory, "fixed_wing_avoid_rejoin_30.csv")
    if not os.path.exists(path):
        return
    table = load_table(path)
    add_late_contacts(table)
    print(f"\nRejoin experiment  ({len(table.rows)} runs, {len(table.failed_rows())} failed)")
    for weather in table.levels("weather"):
        for map_level in table.levels("map"):
            for amplitude in table.levels("amplitude"):
                cell = part(table, weather=weather, map=map_level, amplitude=amplitude)
                print(f"\n weather={weather} map={map_level} amplitude={amplitude}")
                g = means(cell, "arm", METRICS)
                for arm in ARMS:
                    if arm in g:
                        values = "  ".join(f"{m}={g[arm].metrics[m].mean:.2f}" for m in METRICS)
                        print(f"  {arm:9s} {values}")
                for arm in ARMS[2:]:
                    cells = "  ".join(
                        f"{m} {diff(cell, 'arm', arm, 'avoid', m).mean:+.2f}"
                        for m in ("late_contacts", "orbit_corridor_share", "orbit_radial_rmse_m"))
                    print(f"  {arm} - avoid: {cells}")
    print("\nCriteria of ADR 0015 against `avoid`:")
    for arm in ARMS[2:]:
        results = judge(table, arm)
        verdict = "better" if all(passed for _, passed, _ in results) else "not better on all"
        print(f"\n {arm}: {verdict}")
        for name, passed, detail in results:
            print(f"   {'pass' if passed else 'FAIL'}  {name}: {detail}")


def clearance(directory):
    path = os.path.join(directory, "fixed_wing_avoid_clearance_30.csv")
    if not os.path.exists(path):
        return
    table = load_table(path)
    add_late_contacts(table)
    print(f"\nAvoidance clearance sweep  ({len(table.rows)} runs, {len(table.failed_rows())} failed)")
    for map_level in table.levels("map"):
        print(f"\n map={map_level}")
        for g in summarize_groups(part(table, map=map_level), ["clearance"], METRICS):
            values = "  ".join(f"{m}={g.metrics[m].mean:.2f}" for m in METRICS)
            print(f"  {g.key[0]:4s} runs={g.runs}  {values}")


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else "../results"
    main_experiment(directory)
    clearance(directory)


if __name__ == "__main__":
    main()
