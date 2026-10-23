"""Prints the tables of the sharing experiments (ADR 0016) and applies their criteria.

Usage, from the analysis directory, after running the experiments (see docs/RESULTS.md):

    python results_share.py ../results

It reads fixed_wing_share_30.csv, fixed_wing_share_sensitivity_30.csv, and
fixed_wing_share_trust_30.csv from the directory given. Intervals are 95 percent Student t intervals
for the mean over runs. A paired difference is a - b over the runs that share every other axis level
and the seed.

The criteria of ADR 0016, written before the experiments were run.
  C1  In fog on 400 and 480 m hills at loss 0.05 with no jam, `share` against `memory` has late
      contacts lower with an interval entirely below zero, or a corridor share higher with an
      interval entirely above zero and late contacts not higher beyond the interval.
  C2  On the 40 m control, `share` against `memory` and against `avoid` has no late contacts and a
      corridor share not lower by more than 0.02 (over every communication condition).
  C3  At every communication condition, `share` against `memory` has late contacts not higher
      beyond the paired interval (an interval entirely above zero is a harm).
  C4  Memory and share are both reported against `avoid`.
  T1  `share` with the liar against `share` without it on the 40 m hills: a corridor share lower with
      an interval entirely below zero, or late contacts higher with an interval entirely above zero.
  T2  `confirmed` with the liar against `confirmed` without it, on 40 m and 400 m hills: late
      contacts not higher beyond the interval and a corridor share not lower by more than 0.02.
  T3  `confirmed` against `share`, both with no liar, in fog on 400 m hills: what the rule costs.
"""

import copy
import os
import sys

from swarmsim_analysis.aggregate import paired_difference, summarize_groups
from swarmsim_analysis.table import load_table

METRICS = ["late_contacts", "orbit_corridor_share", "time_on_station", "orbit_radial_rmse_m",
           "min_agl_m", "mean_agl_m", "collision_episodes", "messages_sent", "delivery_ratio"]


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


def fmt(s, digits=2):
    return f"{s.mean:+.{digits}f} [{s.ci_low:+.{digits}f}, {s.ci_high:+.{digits}f}]"


def means(cell, axis, metrics):
    return {g.key[0]: g for g in summarize_groups(cell, [axis], metrics)}


def main_experiment(directory):
    path = os.path.join(directory, "fixed_wing_share_30.csv")
    if not os.path.exists(path):
        return
    table = load_table(path)
    add_late_contacts(table)
    print(f"\nSharing experiment  ({len(table.rows)} runs, {len(table.failed_rows())} failed)")

    # Every cell, compactly.
    print("\n weather loss jam    amp  | late contacts avoid/memory/share | corridor share a/m/s |"
          " share-memory late [95%] | share-memory corridor [95%] | messages sent a/m/s")
    for weather in table.levels("weather"):
        for amplitude in table.levels("amplitude"):
            for loss in table.levels("loss"):
                for jam in table.levels("jam"):
                    cell = part(table, weather=weather, amplitude=amplitude, loss=loss, jam=jam)
                    g = means(cell, "arm", METRICS)
                    late = diff(cell, "arm", "share", "memory", "late_contacts")
                    corridor = diff(cell, "arm", "share", "memory", "orbit_corridor_share")
                    print(f" {weather:5s} {loss:4s} {jam:6s} {amplitude:4s} | "
                          f"{g['avoid'].metrics['late_contacts'].mean:4.2f} "
                          f"{g['memory'].metrics['late_contacts'].mean:4.2f} "
                          f"{g['share'].metrics['late_contacts'].mean:4.2f}  | "
                          f"{g['avoid'].metrics['orbit_corridor_share'].mean:.2f} "
                          f"{g['memory'].metrics['orbit_corridor_share'].mean:.2f} "
                          f"{g['share'].metrics['orbit_corridor_share'].mean:.2f} | "
                          f"{fmt(late)} | {fmt(corridor, 3)} | "
                          f"{g['avoid'].metrics['messages_sent'].mean:.0f} "
                          f"{g['memory'].metrics['messages_sent'].mean:.0f} "
                          f"{g['share'].metrics['messages_sent'].mean:.0f}")

    print("\nC4, memory and share against avoid (late contacts, corridor share), fog, no jam, loss 0.05:")
    for amplitude in ("a400", "a480"):
        cell = part(table, weather="fog", amplitude=amplitude, loss="l05", jam="none")
        for arm in ("memory", "share"):
            late = diff(cell, "arm", arm, "avoid", "late_contacts")
            corridor = diff(cell, "arm", arm, "avoid", "orbit_corridor_share")
            print(f"  {amplitude} {arm} - avoid: late {fmt(late)}  corridor {fmt(corridor, 3)}")

    print("\nCriteria of ADR 0016:")
    # C1
    for amplitude in ("a400", "a480"):
        cell = part(table, weather="fog", amplitude=amplitude, loss="l05", jam="none")
        late = diff(cell, "arm", "share", "memory", "late_contacts")
        corridor = diff(cell, "arm", "share", "memory", "orbit_corridor_share")
        passed = late.ci_high < 0.0 or (corridor.ci_low > 0.0 and not late.ci_low > 0.0)
        print(f"  C1 {amplitude}: {'pass' if passed else 'FAIL'}  late {fmt(late)}  "
              f"corridor {fmt(corridor, 3)}")
    # C2
    worst_late = 0.0
    worst_gap_memory = 0.0
    worst_gap_avoid = 0.0
    for weather in table.levels("weather"):
        for loss in table.levels("loss"):
            for jam in table.levels("jam"):
                cell = part(table, weather=weather, amplitude="a40", loss=loss, jam=jam)
                g = means(cell, "arm", ["late_contacts", "orbit_corridor_share"])
                worst_late = max(worst_late, g["share"].metrics["late_contacts"].mean)
                worst_gap_memory = min(worst_gap_memory,
                                       g["share"].metrics["orbit_corridor_share"].mean
                                       - g["memory"].metrics["orbit_corridor_share"].mean)
                worst_gap_avoid = min(worst_gap_avoid,
                                      g["share"].metrics["orbit_corridor_share"].mean
                                      - g["avoid"].metrics["orbit_corridor_share"].mean)
    passed = worst_late == 0.0 and worst_gap_memory >= -0.02 and worst_gap_avoid >= -0.02
    print(f"  C2 (40 m control): {'pass' if passed else 'FAIL'}  most late contacts of share "
          f"{worst_late:.2f}, worst corridor gap against memory {worst_gap_memory:+.3f} and "
          f"against avoid {worst_gap_avoid:+.3f}")
    # C3
    harms = []
    for weather in table.levels("weather"):
        for amplitude in table.levels("amplitude"):
            for loss in table.levels("loss"):
                for jam in table.levels("jam"):
                    cell = part(table, weather=weather, amplitude=amplitude, loss=loss, jam=jam)
                    late = diff(cell, "arm", "share", "memory", "late_contacts")
                    if late.ci_low > 0.0:
                        harms.append(f"{weather} {amplitude} {loss} {jam}: {fmt(late)}")
    print(f"  C3 (never worse than memory): {'pass' if not harms else 'FAIL'}"
          + ("" if not harms else "  harms: " + "; ".join(harms)))

    print("\nAdvantage of share over memory by loss and jam (late contacts, corridor share):")
    for weather in table.levels("weather"):
        for amplitude in ("a400", "a480"):
            for jam in table.levels("jam"):
                row = []
                for loss in table.levels("loss"):
                    cell = part(table, weather=weather, amplitude=amplitude, loss=loss, jam=jam)
                    late = diff(cell, "arm", "share", "memory", "late_contacts")
                    corridor = diff(cell, "arm", "share", "memory", "orbit_corridor_share")
                    row.append(f"{loss}: late {late.mean:+.2f} corridor {corridor.mean:+.3f}")
                print(f"  {weather} {amplitude} jam={jam}:  " + "   ".join(row))


def sensitivity(directory):
    path = os.path.join(directory, "fixed_wing_share_sensitivity_30.csv")
    if not os.path.exists(path):
        return
    table = load_table(path)
    add_late_contacts(table)
    print(f"\nSharing sensitivity  ({len(table.rows)} runs, {len(table.failed_rows())} failed)")
    for g in summarize_groups(table, ["ttl", "points"], METRICS):
        values = "  ".join(f"{m}={g.metrics[m].mean:.2f}" for m in METRICS[:7])
        print(f"  {g.key[0]:5s} {g.key[1]:3s} runs={g.runs}  {values}")


def trust(directory):
    path = os.path.join(directory, "fixed_wing_share_trust_30.csv")
    if not os.path.exists(path):
        return
    table = load_table(path)
    add_late_contacts(table)
    print(f"\nTrust experiment  ({len(table.rows)} runs, {len(table.failed_rows())} failed)")
    for weather in table.levels("weather"):
        for amplitude in table.levels("amplitude"):
            print(f"\n weather={weather} amplitude={amplitude}")
            for fault in table.levels("fault"):
                cell = part(table, weather=weather, amplitude=amplitude, fault=fault)
                g = means(cell, "arm", METRICS[:7])
                for arm in ("avoid", "memory", "share", "confirmed"):
                    values = "  ".join(f"{m}={g[arm].metrics[m].mean:.2f}" for m in METRICS[:7])
                    print(f"  fault={fault:5s} {arm:9s} {values}")

    print("\nCriteria of ADR 0016:")
    # T1
    for weather in table.levels("weather"):
        cell = part(table, arm="share", amplitude="a40", weather=weather)
        corridor = diff(cell, "fault", "liar", "none", "orbit_corridor_share")
        late = diff(cell, "fault", "liar", "none", "late_contacts")
        harm = corridor.ci_high < 0.0 or late.ci_low > 0.0
        print(f"  T1 a40 {weather}: {'harm shown' if harm else 'no measurable harm'}  "
              f"corridor {fmt(corridor, 3)}  late {fmt(late)}")
    # T2
    for amplitude in ("a40", "a400"):
        for weather in table.levels("weather"):
            cell = part(table, arm="confirmed", amplitude=amplitude, weather=weather)
            late = diff(cell, "fault", "liar", "none", "late_contacts")
            corridor = diff(cell, "fault", "liar", "none", "orbit_corridor_share")
            passed = not late.ci_low > 0.0 and corridor.mean >= -0.02
            print(f"  T2 {amplitude} {weather}: {'pass' if passed else 'FAIL'}  late {fmt(late)}  "
                  f"corridor {fmt(corridor, 3)}")
    # T3
    cell = part(table, fault="none", amplitude="a400", weather="fog")
    late = diff(cell, "arm", "confirmed", "share", "late_contacts")
    corridor = diff(cell, "arm", "confirmed", "share", "orbit_corridor_share")
    print(f"  T3 confirmed - share, no liar, fog, 400 m: late {fmt(late)}  "
          f"corridor {fmt(corridor, 3)}")
    # The check on the experiment: memory never receives.
    worst = 0.0
    for amplitude in table.levels("amplitude"):
        for weather in table.levels("weather"):
            cell = part(table, arm="memory", amplitude=amplitude, weather=weather)
            for metric in ("late_contacts", "orbit_corridor_share", "orbit_radial_rmse_m"):
                worst = max(worst, abs(diff(cell, "fault", "liar", "none", metric).mean))
    print(f"  check: the largest difference a liar makes to the memory arm: {worst:.6f}")


def main():
    directory = sys.argv[1] if len(sys.argv) > 1 else "../results"
    main_experiment(directory)
    sensitivity(directory)
    trust(directory)


if __name__ == "__main__":
    main()
