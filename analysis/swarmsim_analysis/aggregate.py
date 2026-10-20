"""Grouping a results table and summarizing its metrics."""

import math
from dataclasses import dataclass

from .stats import Summary, summarize
from .table import Table


@dataclass(frozen=True)
class GroupSummary:
    """One group of runs: the axis levels that define it and a Summary for each metric."""

    key: tuple
    runs: int          # runs in the group that succeeded
    failed: int        # runs in the group that did not
    metrics: dict      # metric name -> Summary


def summarize_groups(table, by, metrics=None, confidence=0.95):
    """Summarizes `metrics` (default: all) for each combination of levels of the axes in `by`.

    Groups come back in the order their first run appears in the table, which is cell order.
    Failed runs are counted in `failed` and kept out of the statistics. An empty `by` gives one
    group of everything.
    """
    for axis in by:
        if axis not in table.axes:
            raise KeyError(f"no axis {axis!r}; the axes are {table.axes}")
    chosen = list(table.metrics if metrics is None else metrics)
    for metric in chosen:
        if metric not in table.metrics:
            raise KeyError(f"no metric {metric!r}")

    order = []
    members = {}
    for row in table.rows:
        key = tuple(row[a] for a in by)
        if key not in members:
            members[key] = []
            order.append(key)
        members[key].append(row)

    groups = []
    for key in order:
        rows = members[key]
        good = [r for r in rows if r["status"] == "ok"]
        summaries = {m: summarize([r[m] for r in good], confidence) for m in chosen}
        groups.append(GroupSummary(key, len(good), len(rows) - len(good), summaries))
    return groups


def paired_difference(table, axis, level_a, level_b, metric, confidence=0.95):
    """The difference `a - b` of a metric between two levels of one axis, over matched runs.

    Runs are matched when they agree on every other axis and on the replicate, so they share a
    seed. Only pairs in which both runs succeeded and both have the metric are used. Returns a
    Summary of the differences; its `n` is the number of pairs.
    """
    if axis not in table.axes:
        raise KeyError(f"no axis {axis!r}; the axes are {table.axes}")
    if metric not in table.metrics:
        raise KeyError(f"no metric {metric!r}")
    if level_a == level_b:
        raise ValueError("the two levels are the same")
    others = [a for a in table.axes if a != axis]

    def key(row):
        return tuple(row[a] for a in others) + (row["replicate"],)

    side_a = {key(r): r for r in table.ok_rows() if r[axis] == level_a}
    side_b = {key(r): r for r in table.ok_rows() if r[axis] == level_b}
    if not side_a and not side_b:
        raise KeyError(f"neither {level_a!r} nor {level_b!r} is a level of {axis!r}")

    differences = []
    for k in side_a:
        if k in side_b:
            differences.append(side_a[k][metric] - side_b[k][metric])
    return summarize([d for d in differences], confidence) if differences else summarize([], confidence)


def summary_rows(groups, by, metrics):
    """Flattens group summaries to rows of plain values, for writing as CSV."""
    header = list(by) + ["runs", "failed"]
    for metric in metrics:
        header += [f"{metric}_{field}" for field in
                   ("n", "missing", "mean", "std", "ci_low", "ci_high", "median", "min", "max")]
    rows = [header]
    for group in groups:
        row = list(group.key) + [group.runs, group.failed]
        for metric in metrics:
            s = group.metrics[metric]
            row += [s.n, s.missing, s.mean, s.std, s.ci_low, s.ci_high, s.median, s.minimum,
                    s.maximum]
        rows.append(row)
    return rows


def format_number(value):
    """Numbers as the CSV writes them: empty for NaN, else 9 significant digits."""
    if isinstance(value, float):
        return "" if math.isnan(value) else f"{value:.9g}"
    return str(value)
