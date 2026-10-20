"""Command line: python -m swarmsim_analysis summarize|figures TABLE.csv ..."""

import argparse
import csv
import sys

from .aggregate import format_number, summarize_groups, summary_rows
from .figures import FigureError
from .make import make_figures
from .table import TableError, load_table

DEFAULT_METRICS = (
    "formation_rmse_settled_m",
    "min_separation_m",
    "collision_episodes",
    "recovery_longest_s",
    "recovery_censored",
    "delivery_ratio",
    "latency_p50_s",
    "latency_p95_s",
)


def _split(text):
    return [part for part in text.split(",") if part]


def _figures(args):
    where = {}
    for pair in _split(args.where):
        axis, separator, level = pair.partition("=")
        if not separator or not axis or not level:
            print(f"swarmsim_analysis: --where needs axis=level pairs, got {pair!r}",
                  file=sys.stderr)
            return 2
        where[axis] = level
    try:
        table = load_table(args.table)
        written, skipped = make_figures(table, args.out, where)
    except OSError as error:
        print(f"swarmsim_analysis: {error}", file=sys.stderr)
        return 2
    except (TableError, FigureError, KeyError) as error:
        print(f"swarmsim_analysis: {error.args[0]}", file=sys.stderr)
        return 2
    for path in written:
        print(f"wrote {path}")
    for name, reason in skipped:
        print(f"skipped {name}: {reason}", file=sys.stderr)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python -m swarmsim_analysis")
    commands = parser.add_subparsers(dest="command", required=True)
    summarize_cmd = commands.add_parser(
        "summarize", help="mean, interval, median and range per group of runs")
    summarize_cmd.add_argument("table", help="the CSV that swarm_batch wrote")
    summarize_cmd.add_argument("--by", default="", help="comma-separated axes to group by")
    summarize_cmd.add_argument("--metrics", default=",".join(DEFAULT_METRICS),
                               help="comma-separated metric columns")
    summarize_cmd.add_argument("--out", help="write the summary here instead of standard output")
    figures_cmd = commands.add_parser(
        "figures", help="write the figures that the table's axes support")
    figures_cmd.add_argument("table", help="the CSV that swarm_batch wrote")
    figures_cmd.add_argument("--out", required=True, help="directory for the PNG files")
    figures_cmd.add_argument("--where", default="",
                             help="comma-separated axis=level pairs to fix, for example "
                                  "faults=none,scene=field")
    args = parser.parse_args(argv)

    if args.command == "figures":
        return _figures(args)

    try:
        table = load_table(args.table)
        by = _split(args.by)
        metrics = _split(args.metrics)
        groups = summarize_groups(table, by, metrics)
    except OSError as error:
        print(f"swarmsim_analysis: {error}", file=sys.stderr)
        return 2
    except (TableError, KeyError) as error:
        print(f"swarmsim_analysis: {error.args[0]}", file=sys.stderr)
        return 2

    rows = summary_rows(groups, by, metrics)
    out = open(args.out, "w", encoding="utf-8", newline="") if args.out else sys.stdout
    try:
        writer = csv.writer(out, lineterminator="\n")
        writer.writerow(rows[0])
        for row in rows[1:]:
            writer.writerow([format_number(v) for v in row])
    finally:
        if args.out:
            out.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
