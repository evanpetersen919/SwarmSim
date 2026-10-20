import csv
import io
import math
import pathlib

import pytest

from swarmsim_analysis import paired_difference, parse_table, summarize_groups
from swarmsim_analysis.__main__ import main

FIXTURE = pathlib.Path(__file__).parent / "fixtures" / "smoke.csv"

HEADER = "experiment,cell,variant,loss,replicate,status,verified,scenario,seed,error,recovery"


def build(rows):
    """rows: (variant, loss, replicate, status, error, recovery) tuples, in table order."""
    lines = [HEADER]
    cell = {}
    for variant, loss, replicate, status, error, recovery in rows:
        index = cell.setdefault((variant, loss), len(cell))
        lines.append(
            f"t,{index},{variant},{loss},{replicate},{status},0,t.{index},{100 + replicate},"
            f"{error},{recovery}")
    return parse_table("\n".join(lines) + "\n")


# Two variants x two loss levels x three seeds. The error of "b" is that of "a" plus 1, 2, 3 for
# the three seeds, so the paired differences are known.
def sample():
    rows = []
    for loss, base in (("p10", 10.0), ("p30", 20.0)):
        for r in range(3):
            rows.append(("a", loss, r, "ok", base + r, ""))
        for r in range(3):
            rows.append(("b", loss, r, "ok", base + r + (r + 1), "5"))
    return build(rows)


def test_groups_come_back_in_table_order_with_their_statistics():
    groups = summarize_groups(sample(), ["variant"], ["error"])
    assert [g.key for g in groups] == [("a",), ("b",)]
    a, b = groups
    assert a.runs == 6 and a.failed == 0
    assert a.metrics["error"].n == 6
    assert a.metrics["error"].mean == pytest.approx((10 + 11 + 12 + 20 + 21 + 22) / 6)
    assert b.metrics["error"].mean == pytest.approx(a.metrics["error"].mean + 2.0)


def test_grouping_by_two_axes_makes_a_group_per_combination():
    groups = summarize_groups(sample(), ["variant", "loss"], ["error"])
    assert [g.key for g in groups] == [("a", "p10"), ("b", "p10"), ("a", "p30"), ("b", "p30")]
    assert groups[0].metrics["error"].mean == 11.0
    assert groups[0].metrics["error"].n == 3
    expected_std = 1.0  # values 10, 11, 12
    assert groups[0].metrics["error"].std == pytest.approx(expected_std)


def test_no_grouping_axes_gives_one_group_of_everything():
    groups = summarize_groups(sample(), [], ["error"])
    assert len(groups) == 1
    assert groups[0].runs == 12


def test_failed_runs_are_counted_and_kept_out_of_the_statistics():
    table = build([
        ("a", "p10", 0, "ok", 1.0, ""),
        ("a", "p10", 1, "run failed: x", "", ""),
        ("a", "p10", 2, "ok", 3.0, ""),
    ])
    (group,) = summarize_groups(table, ["variant"], ["error"])
    assert group.runs == 2 and group.failed == 1
    assert group.metrics["error"].mean == 2.0
    assert group.metrics["error"].n == 2


def test_a_metric_that_exists_in_some_runs_reports_how_many_it_covers():
    table = build([
        ("a", "p10", 0, "ok", 1.0, "4"),
        ("a", "p10", 1, "ok", 2.0, ""),
        ("a", "p10", 2, "ok", 3.0, "6"),
    ])
    (group,) = summarize_groups(table, ["variant"], ["recovery"])
    summary = group.metrics["recovery"]
    assert summary.n == 2 and summary.missing == 1
    assert summary.mean == 5.0


def test_a_group_with_no_successful_runs_has_nan_statistics():
    table = build([("a", "p10", 0, "run failed: x", "", "")])
    (group,) = summarize_groups(table, ["variant"], ["error"])
    assert group.runs == 0 and group.failed == 1
    assert math.isnan(group.metrics["error"].mean)


def test_unknown_axes_and_metrics_are_key_errors():
    with pytest.raises(KeyError):
        summarize_groups(sample(), ["nope"], ["error"])
    with pytest.raises(KeyError):
        summarize_groups(sample(), ["variant"], ["nope"])


def test_paired_difference_matches_runs_on_the_other_axes_and_the_replicate():
    d = paired_difference(sample(), "variant", "b", "a", "error")
    assert d.n == 6
    assert d.mean == pytest.approx(2.0)  # each pair differs by 1, 2, 3, twice
    assert d.minimum == 1.0 and d.maximum == 3.0
    reverse = paired_difference(sample(), "variant", "a", "b", "error")
    assert reverse.mean == pytest.approx(-2.0)


def test_pairs_with_a_failed_or_missing_run_are_dropped():
    rows = []
    for r in range(3):
        rows.append(("a", "p10", r, "ok", 10.0 + r, ""))
    rows.append(("b", "p10", 0, "ok", 11.0, ""))
    rows.append(("b", "p10", 1, "run failed: x", "", ""))
    # replicate 2 of b is absent altogether
    d = paired_difference(build(rows), "variant", "b", "a", "error")
    assert d.n == 1
    assert d.mean == 1.0


def test_paired_difference_with_no_pairs_is_nan_and_bad_arguments_are_rejected():
    table = build([("a", "p10", 0, "ok", 1.0, ""), ("b", "p30", 0, "ok", 2.0, "")])
    assert paired_difference(table, "variant", "a", "b", "error").n == 0  # different loss levels
    with pytest.raises(ValueError):
        paired_difference(table, "variant", "a", "a", "error")
    with pytest.raises(KeyError):
        paired_difference(table, "variant", "x", "y", "error")
    with pytest.raises(KeyError):
        paired_difference(table, "nope", "a", "b", "error")


def test_the_command_line_writes_a_summary_csv(tmp_path, capsys):
    out = tmp_path / "summary.csv"
    status = main(["summarize", str(FIXTURE), "--by", "loss,size", "--metrics",
                   "formation_rmse_settled_m,min_separation_m", "--out", str(out)])
    assert status == 0
    rows = list(csv.reader(io.StringIO(out.read_text())))
    header = rows[0]
    assert header[:4] == ["loss", "size", "runs", "failed"]
    assert "formation_rmse_settled_m_ci_low" in header
    assert len(rows) == 1 + 4  # four groups
    assert rows[1][:4] == ["p00", "n04", "2", "0"]
    assert all(len(r) == len(header) for r in rows)


def test_the_command_line_prints_to_standard_output_without_out(capsys):
    assert main(["summarize", str(FIXTURE), "--metrics", "delivery_ratio"]) == 0
    text = capsys.readouterr().out
    assert text.splitlines()[0].startswith("runs,failed,delivery_ratio_n")
    assert len(text.splitlines()) == 2


def test_the_command_line_reports_bad_input_with_status_two(tmp_path, capsys):
    assert main(["summarize", str(tmp_path / "missing.csv")]) == 2
    bad = tmp_path / "bad.csv"
    bad.write_text("a,b\n1,2\n")
    assert main(["summarize", str(bad)]) == 2
    assert main(["summarize", str(FIXTURE), "--by", "nope"]) == 2
    assert main(["summarize", str(FIXTURE), "--metrics", "nope"]) == 2
    assert "swarmsim_analysis:" in capsys.readouterr().err
