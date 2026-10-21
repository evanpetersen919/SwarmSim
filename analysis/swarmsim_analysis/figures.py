"""Figures from a results table.

Every figure is a function from a table to a file. Sizes, fonts, and colours are fixed, so the same
table gives the same picture. A point is the mean over the runs in its group with a 95 percent
Student t interval; a point with a single run has no interval, and a group with no value for the
metric is left out, so a line joins only points that were measured. Axes that the figure does not
show are pooled, and the subtitle says which.
"""

import re

import matplotlib

matplotlib.use("Agg")

from matplotlib.backends.backend_agg import FigureCanvasAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

from .aggregate import summarize_groups  # noqa: E402

WIDTH_PX = 640
HEIGHT_PX = 400
DPI = 100

# Okabe and Ito palette, readable with the common kinds of colour blindness.
PALETTE = ("#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00", "#56B4E9", "#F0E442", "#000000")

METRIC_LABELS = {
    "formation_rmse_m": "Formation error, whole run (m)",
    "formation_rmse_settled_m": "Settled formation error (m)",
    "min_separation_m": "Closest approach (m)",
    "collision_episodes": "Collision episodes per run",
    "recovery_longest_s": "Longest recovery time (s)",
    "recovery_censored": "Fraction of runs with a censored recovery",
    "delivery_ratio": "Delivery ratio",
    "latency_p50_s": "Median message latency (s)",
    "latency_p95_s": "95th percentile message latency (s)",
    "orbit_radial_rmse_m": "Orbit radial error (m)",
    "orbit_altitude_rmse_m": "Orbit altitude error (m)",
    "orbit_phase_rmse_m": "Orbit phase error, arc length (m)",
    "time_on_station": "Fraction of time on station",
}

AXIS_LABELS = {
    "loss": "Packet loss",
    "weather": "Weather",
    "jamming": "Jamming",
    "size": "Number of drones",
    "faults": "Drone failures",
    "scene": "Scene",
    "variant": "Controller variant",
    "predictor": "Neighbor predictor",
}


class FigureError(ValueError):
    """The table does not have what the figure needs."""


def metric_label(metric):
    return METRIC_LABELS.get(metric, metric)


def axis_label(axis):
    return AXIS_LABELS.get(axis, axis)


def level_label(label):
    """Level labels as they appear on an axis: p30 is 30 %, n10 is 10, and others are kept."""
    percent = re.fullmatch(r"p(\d+)", label)
    if percent:
        return f"{int(percent.group(1))} %"
    count = re.fullmatch(r"n(\d+)", label)
    if count:
        return str(int(count.group(1)))
    return label.replace("_", " ")


def select(table, where):
    """A copy of the table with only the runs whose axis levels match `where`."""
    for axis, level in where.items():
        if axis not in table.axes:
            raise FigureError(f"no axis {axis!r}; the axes are {table.axes}")
        if level not in table.levels(axis):
            raise FigureError(f"axis {axis!r} has no level {level!r}")
    from .table import Table

    kept = [r for r in table.rows if all(r[a] == v for a, v in where.items())]
    return Table(axes=list(table.axes), metrics=list(table.metrics), rows=kept)


def _subtitle(table, shown, where):
    pooled = [a for a in table.axes if a not in shown and a not in where]
    parts = []
    if where:
        parts.append("fixed: " + ", ".join(f"{a} = {level_label(v)}" for a, v in where.items()))
    if pooled:
        parts.append("pooled over: " + ", ".join(pooled))
    runs = len(table.ok_rows())
    parts.append(f"{runs} runs, mean and 95 % interval")
    return "; ".join(parts)


def _new_figure(title, subtitle):
    fig = Figure(figsize=(WIDTH_PX / DPI, HEIGHT_PX / DPI), dpi=DPI)
    FigureCanvasAgg(fig)
    ax = fig.add_axes([0.11, 0.17, 0.86, 0.62])
    fig.text(0.11, 0.93, title, fontsize=11, fontweight="bold", ha="left", va="center")
    fig.text(0.11, 0.865, subtitle, fontsize=7.5, ha="left", va="center", color="#444444")
    ax.grid(axis="y", color="#dddddd", linewidth=0.6)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    return fig, ax


def _save(fig, path):
    fig.savefig(path, format="png", dpi=DPI, metadata={"Software": None})


def _point(summary):
    """(mean, lower error, upper error) or None when the group has no value."""
    if summary.n == 0:
        return None
    if summary.n < 2 or summary.ci_low != summary.ci_low:
        return summary.mean, 0.0, 0.0
    return summary.mean, summary.mean - summary.ci_low, summary.ci_high - summary.mean


def plot_lines(table, x_axis, metric, path, series_axis=None, where=None, title=None):
    """The metric against the levels of `x_axis`, one line per level of `series_axis`.

    Levels are placed at equal spacing in the order they appear in the table, which is the order
    the experiment lists them. Lines join only neighbouring levels that both have a value.
    """
    where = dict(where or {})
    data = select(table, where) if where else table
    if x_axis not in data.axes:
        raise FigureError(f"no axis {x_axis!r}; the axes are {data.axes}")
    if series_axis is not None and series_axis not in data.axes:
        raise FigureError(f"no axis {series_axis!r}; the axes are {data.axes}")
    if metric not in data.metrics:
        raise FigureError(f"no metric {metric!r}")
    if not data.ok_rows():
        raise FigureError("there are no successful runs to plot")

    shown = [x_axis] + ([series_axis] if series_axis else [])
    groups = summarize_groups(data, shown, [metric])
    by_key = {g.key: g.metrics[metric] for g in groups}
    x_levels = data.levels(x_axis)
    series_levels = data.levels(series_axis) if series_axis else [None]

    fig, ax = _new_figure(title or f"{metric_label(metric)} by {axis_label(x_axis).lower()}",
                          _subtitle(data, shown, where))
    for index, series in enumerate(series_levels):
        xs, ys, lower, upper = [], [], [], []
        for position, level in enumerate(x_levels):
            key = (level,) if series is None else (level, series)
            point = _point(by_key[key]) if key in by_key else None
            if point is not None:
                xs.append(position)
                ys.append(point[0])
                lower.append(point[1])
                upper.append(point[2])
        color = PALETTE[index % len(PALETTE)]
        label = None if series is None else level_label(series)
        # Join only neighbouring positions that were both measured.
        start = 0
        for i in range(1, len(xs) + 1):
            if i == len(xs) or xs[i] != xs[i - 1] + 1:
                ax.plot([x + (index - (len(series_levels) - 1) / 2.0) * 0.05 for x in xs[start:i]],
                        ys[start:i], color=color, linewidth=1.4)
                start = i
        # Series are shifted a little sideways so that points with equal values stay visible.
        shift = (index - (len(series_levels) - 1) / 2.0) * 0.05
        ax.errorbar([x + shift for x in xs], ys, yerr=[lower, upper], fmt="o", color=color,
                    markersize=4, capsize=3, linewidth=1.0, label=label)
    ax.set_xticks(range(len(x_levels)))
    ax.set_xticklabels([level_label(v) for v in x_levels], fontsize=8)
    ax.set_xlim(-0.4, len(x_levels) - 0.6)
    ax.set_xlabel(axis_label(x_axis), fontsize=9)
    ax.set_ylabel(metric_label(metric), fontsize=9)
    ax.tick_params(axis="y", labelsize=8)
    if series_axis:
        ax.legend(title=axis_label(series_axis), fontsize=7.5, title_fontsize=8, frameon=False)
    _save(fig, path)


def plot_bars(table, group_axis, series_axis, metric, path, where=None, title=None):
    """The metric as bars: one cluster per level of `group_axis`, one bar per `series_axis` level."""
    where = dict(where or {})
    data = select(table, where) if where else table
    for axis in (group_axis, series_axis):
        if axis not in data.axes:
            raise FigureError(f"no axis {axis!r}; the axes are {data.axes}")
    if metric not in data.metrics:
        raise FigureError(f"no metric {metric!r}")
    if not data.ok_rows():
        raise FigureError("there are no successful runs to plot")

    shown = [group_axis, series_axis]
    groups = summarize_groups(data, shown, [metric])
    by_key = {g.key: g.metrics[metric] for g in groups}
    clusters = data.levels(group_axis)
    series = data.levels(series_axis)
    width = 0.8 / len(series)

    fig, ax = _new_figure(
        title or f"{metric_label(metric)} by {axis_label(series_axis).lower()}",
        _subtitle(data, shown, where))
    for index, level in enumerate(series):
        xs, ys, lower, upper = [], [], [], []
        for position, cluster in enumerate(clusters):
            summary = by_key.get((cluster, level))
            point = _point(summary) if summary is not None else None
            if point is not None:
                xs.append(position - 0.4 + width * (index + 0.5))
                ys.append(point[0])
                lower.append(point[1])
                upper.append(point[2])
        color = PALETTE[index % len(PALETTE)]
        ax.bar(xs, ys, width=width * 0.92, color=color, label=level_label(level))
        ax.errorbar(xs, ys, yerr=[lower, upper], fmt="none", ecolor="#222222", capsize=2.5,
                    linewidth=0.9)
    ax.set_xticks(range(len(clusters)))
    ax.set_xticklabels([level_label(v) for v in clusters], fontsize=8)
    ax.set_xlim(-0.5, len(clusters) - 0.5)
    ax.set_xlabel(axis_label(group_axis), fontsize=9)
    ax.set_ylabel(metric_label(metric), fontsize=9)
    ax.tick_params(axis="y", labelsize=8)
    # Below the plot, so it never covers a bar.
    ax.set_position([0.11, 0.27, 0.86, 0.52])
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, title=axis_label(series_axis), loc="lower center",
               bbox_to_anchor=(0.54, 0.0), ncols=min(len(series), 4), fontsize=7.5,
               title_fontsize=8, frameon=False)
    _save(fig, path)
