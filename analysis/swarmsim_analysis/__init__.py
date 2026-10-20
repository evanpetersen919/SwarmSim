"""Analysis of SwarmSim batch results: reading the table, summaries, and figures."""

from .aggregate import GroupSummary, paired_difference, summarize_groups
from .stats import Summary, summarize
from .table import Table, TableError, load_table, parse_table

__all__ = [
    "GroupSummary",
    "Summary",
    "Table",
    "TableError",
    "load_table",
    "paired_difference",
    "parse_table",
    "summarize",
    "summarize_groups",
]
