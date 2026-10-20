"""Summary statistics for the values of one metric over seeds."""

import math
from dataclasses import dataclass

from scipy import stats as scipy_stats

CONFIDENCE = 0.95


@dataclass(frozen=True)
class Summary:
    """The values of one metric in one group.

    `n` counts the values that exist; `missing` counts the runs in the group that had none (an
    empty cell, for example a recovery time when nothing needed recovering). The interval is a
    two-sided Student t interval for the mean and is NaN with fewer than two values, because one
    value says nothing about spread.
    """

    n: int
    missing: int
    mean: float
    std: float
    ci_low: float
    ci_high: float
    median: float
    minimum: float
    maximum: float


def _nan_summary(missing):
    nan = math.nan
    return Summary(0, missing, nan, nan, nan, nan, nan, nan, nan)


def _median(sorted_values):
    n = len(sorted_values)
    middle = n // 2
    if n % 2 == 1:
        return sorted_values[middle]
    return (sorted_values[middle - 1] + sorted_values[middle]) / 2.0


def summarize(values, confidence=CONFIDENCE):
    """Summarizes a list of numbers. NaN entries are counted as missing and left out."""
    if not 0.0 < confidence < 1.0:
        raise ValueError("confidence must be between 0 and 1")
    present = sorted(v for v in values if not math.isnan(v))
    missing = len(values) - len(present)
    n = len(present)
    if n == 0:
        return _nan_summary(missing)

    mean = math.fsum(present) / n
    if n == 1:
        return Summary(1, missing, mean, math.nan, math.nan, math.nan, present[0], present[0],
                       present[0])

    variance = math.fsum((v - mean) ** 2 for v in present) / (n - 1)
    std = math.sqrt(variance)
    critical = scipy_stats.t.ppf(0.5 + confidence / 2.0, df=n - 1)
    half_width = critical * std / math.sqrt(n)
    return Summary(n, missing, mean, std, mean - half_width, mean + half_width,
                   _median(present), present[0], present[-1])
