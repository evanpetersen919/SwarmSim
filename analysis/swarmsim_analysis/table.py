"""Reading the results table that swarm_batch writes.

The table has one row per run. The columns are, in order: experiment, cell, one column per axis
(the level label), replicate, status, verified, and then the metric columns. The axes are whatever
lies between `cell` and `replicate`. A metric cell that is empty means the value does not exist
for that run (a failed run, or a metric that could not be computed) and is read as NaN, never as
zero.
"""

import csv
import math
from dataclasses import dataclass, field

FIXED_LEADING = ("experiment", "cell")
FIXED_TRAILING = ("replicate", "status", "verified")
# Columns of the metric part that are text or identifiers rather than measurements.
NON_METRIC = ("scenario", "seed")


class TableError(ValueError):
    """The file is not a results table."""


@dataclass
class Table:
    axes: list
    metrics: list
    rows: list = field(default_factory=list)  # one dict per run: axis labels, replicate, status, ...

    def ok_rows(self):
        return [r for r in self.rows if r["status"] == "ok"]

    def failed_rows(self):
        return [r for r in self.rows if r["status"] != "ok"]

    def levels(self, axis):
        """The labels of an axis in the order they first appear."""
        if axis not in self.axes:
            raise KeyError(f"no axis {axis!r}; the axes are {self.axes}")
        seen = []
        for row in self.rows:
            if row[axis] not in seen:
                seen.append(row[axis])
        return seen


def _number(text):
    if text == "":
        return math.nan
    try:
        return float(text)
    except ValueError as error:
        raise TableError(f"not a number: {text!r}") from error


def parse_table(text):
    """Parses the text of a results table."""
    reader = csv.reader(text.splitlines())
    try:
        header = next(reader)
    except StopIteration as error:
        raise TableError("the table is empty") from error

    if tuple(header[:2]) != FIXED_LEADING:
        raise TableError(f"the first columns must be {list(FIXED_LEADING)}, got {header[:2]}")
    try:
        end_of_axes = header.index("replicate")
    except ValueError as error:
        raise TableError("there is no replicate column") from error
    if tuple(header[end_of_axes:end_of_axes + 3]) != FIXED_TRAILING:
        raise TableError(f"expected {list(FIXED_TRAILING)} after the axes")
    if len(set(header)) != len(header):
        raise TableError("the table has two columns of the same name")

    axes = header[2:end_of_axes]
    metric_start = end_of_axes + 3
    metrics = [c for c in header[metric_start:] if c not in NON_METRIC]
    table = Table(axes=axes, metrics=metrics)

    for number, record in enumerate(reader, start=2):
        if len(record) != len(header):
            raise TableError(
                f"line {number} has {len(record)} cells, the header has {len(header)}")
        row = dict(zip(header, record))
        parsed = {
            "experiment": row["experiment"],
            "cell": int(row["cell"]),
            "replicate": int(row["replicate"]),
            "status": row["status"],
            "verified": row["verified"] == "1",
            "scenario": row["scenario"],
        }
        for axis in axes:
            parsed[axis] = row[axis]
        parsed["seed"] = int(row["seed"]) if row["seed"] != "" else None
        for metric in metrics:
            parsed[metric] = _number(row[metric])
        table.rows.append(parsed)
    return table


def load_table(path):
    with open(path, encoding="utf-8", newline="") as handle:
        return parse_table(handle.read())
