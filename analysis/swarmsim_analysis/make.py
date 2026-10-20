"""Chooses the figures that a results table supports and writes them to a directory."""

import math
import os

from .figures import FigureError, plot_bars, plot_lines


def _has_values(table, metric):
    return any(not math.isnan(r[metric]) for r in table.ok_rows())


def make_figures(table, out_dir, where=None):
    """Writes every figure the table's axes and metrics allow.

    Returns (written, skipped): the file paths, and a list of (file name, reason) for figures that
    were not made. Raises FigureError if no figure could be made.
    """
    where = dict(where or {})
    os.makedirs(out_dir, exist_ok=True)
    written = []
    skipped = []
    axes = set(table.axes) - set(where)
    comparison = next((a for a in ("variant", "predictor") if a in axes), None)

    def lines(name, x_axis, metric, series):
        if not _has_values(table, metric):
            skipped.append((name, f"no run has a value for {metric}"))
            return
        path = os.path.join(out_dir, name)
        plot_lines(table, x_axis, metric, path, series_axis=series, where=where)
        written.append(path)

    def bars(name, group_axis, series_axis, metric):
        if not _has_values(table, metric):
            skipped.append((name, f"no run has a value for {metric}"))
            return
        path = os.path.join(out_dir, name)
        plot_bars(table, group_axis, series_axis, metric, path, where=where)
        written.append(path)

    if "loss" in axes:
        lines("error_vs_loss.png", "loss", "formation_rmse_settled_m", comparison)
        lines("delivery_vs_loss.png", "loss", "delivery_ratio", comparison)
    if "jamming" in axes:
        lines("separation_vs_jamming.png", "jamming", "min_separation_m", comparison)
        lines("recovery_vs_jamming.png", "jamming", "recovery_longest_s", comparison)
    if comparison and "loss" in axes:
        stem = "ablation" if comparison == "variant" else "baselines"
        bars(f"{stem}_error.png", "loss", comparison, "formation_rmse_settled_m")
        bars(f"{stem}_separation.png", "loss", comparison, "min_separation_m")

    if not written:
        raise FigureError(
            "this table has no axis that a figure is defined for (loss, jamming, or variant or "
            f"predictor with loss); its axes are {table.axes}")
    return written, skipped
