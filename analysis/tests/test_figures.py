import itertools
import pathlib
import struct

import pytest

from swarmsim_analysis import parse_table
from swarmsim_analysis import load_table
from swarmsim_analysis.__main__ import main
from swarmsim_analysis.figures import FigureError, level_label, plot_bars, plot_lines, select
from swarmsim_analysis.make import make_figures

FIXTURE = pathlib.Path(__file__).parent / "fixtures" / "smoke.csv"
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"

HEADER = ("experiment,cell,variant,loss,jamming,replicate,status,verified,scenario,seed,"
          "formation_rmse_settled_m,min_separation_m,delivery_ratio,recovery_longest_s")


def synthetic(recovery=True, failed_cell=None):
    """Two variants x three loss levels x two jamming levels x three seeds, with simple values."""
    lines = [HEADER]
    cell = 0
    for variant, loss, jamming in itertools.product(["full", "no_fallback"], ["p10", "p30", "p50"],
                                                    ["none", "pulsed"]):
        for r in range(3):
            level = int(loss[1:]) / 10.0
            error = level * (2.0 if variant == "no_fallback" else 1.0) + 0.1 * r
            recov = f"{3.0 + r}" if recovery and jamming == "pulsed" else ""
            status = "run failed: x" if failed_cell == cell and r == 0 else "ok"
            values = "" if status != "ok" else f"{error},{4.0 - level / 4},{1.0 - level / 10},{recov}"
            if status != "ok":
                values = ",,,"
            lines.append(f"t,{cell},{variant},{loss},{jamming},{r},{status},0,t.{cell},{r},{values}")
        cell += 1
    return parse_table("\n".join(lines) + "\n")


def png_size(path):
    data = pathlib.Path(path).read_bytes()
    assert data[:8] == PNG_SIGNATURE
    assert data[12:16] == b"IHDR"
    return struct.unpack(">II", data[16:24])


def test_a_line_figure_is_a_png_of_the_fixed_size(tmp_path):
    path = tmp_path / "lines.png"
    plot_lines(synthetic(), "loss", "formation_rmse_settled_m", path, series_axis="variant")
    assert png_size(path) == (640, 400)
    assert path.stat().st_size > 2000


def test_a_bar_figure_is_a_png_of_the_fixed_size(tmp_path):
    path = tmp_path / "bars.png"
    plot_bars(synthetic(), "loss", "variant", "min_separation_m", path)
    assert png_size(path) == (640, 400)


def test_the_same_table_gives_the_same_bytes(tmp_path):
    a = tmp_path / "a.png"
    b = tmp_path / "b.png"
    plot_lines(synthetic(), "loss", "delivery_ratio", a, series_axis="variant")
    plot_lines(synthetic(), "loss", "delivery_ratio", b, series_axis="variant")
    assert a.read_bytes() == b.read_bytes()


def test_a_different_table_gives_a_different_picture(tmp_path):
    a = tmp_path / "a.png"
    b = tmp_path / "b.png"
    plot_lines(synthetic(), "loss", "formation_rmse_settled_m", a)
    plot_lines(synthetic(), "loss", "min_separation_m", b)
    assert a.read_bytes() != b.read_bytes()


def test_figures_work_on_a_real_batch_table(tmp_path):
    table = load_table(FIXTURE)
    path = tmp_path / "real.png"
    plot_lines(table, "loss", "formation_rmse_settled_m", path, series_axis="size")
    assert png_size(path) == (640, 400)


def test_a_point_with_one_run_and_a_failed_run_still_draw(tmp_path):
    table = synthetic(failed_cell=0)
    path = tmp_path / "failed.png"
    plot_lines(table, "loss", "formation_rmse_settled_m", path, series_axis="variant")
    assert png_size(path) == (640, 400)


def test_select_keeps_only_matching_runs_and_rejects_unknown_axes_and_levels():
    table = synthetic()
    only = select(table, {"jamming": "pulsed"})
    assert len(only.rows) == len(table.rows) // 2
    assert {r["jamming"] for r in only.rows} == {"pulsed"}
    with pytest.raises(FigureError):
        select(table, {"nope": "x"})
    with pytest.raises(FigureError):
        select(table, {"jamming": "nope"})


def test_figures_reject_missing_axes_and_metrics_and_empty_tables(tmp_path):
    table = synthetic()
    path = tmp_path / "x.png"
    with pytest.raises(FigureError):
        plot_lines(table, "nope", "delivery_ratio", path)
    with pytest.raises(FigureError):
        plot_lines(table, "loss", "nope", path)
    with pytest.raises(FigureError):
        plot_lines(table, "loss", "delivery_ratio", path, series_axis="nope")
    with pytest.raises(FigureError):
        plot_bars(table, "loss", "nope_axis", "delivery_ratio", path)
    all_failed = parse_table(HEADER + "\nt,0,full,p10,none,0,run failed: x,0,t.0,,,,,\n")
    with pytest.raises(FigureError):
        plot_lines(all_failed, "loss", "delivery_ratio", path)


def test_level_labels_read_naturally():
    assert level_label("p30") == "30 %"
    assert level_label("n05") == "5"
    assert level_label("no_fallback") == "no fallback"
    assert level_label("fog") == "fog"


def test_make_figures_writes_the_set_the_axes_support(tmp_path):
    written, skipped = make_figures(synthetic(), tmp_path / "out")
    names = sorted(pathlib.Path(p).name for p in written)
    assert names == [
        "ablation_error.png",
        "ablation_separation.png",
        "delivery_vs_loss.png",
        "error_vs_loss.png",
        "recovery_vs_jamming.png",
        "separation_vs_jamming.png",
    ]
    assert skipped == []
    for path in written:
        assert png_size(path) == (640, 400)


def test_make_figures_skips_a_figure_whose_metric_has_no_values(tmp_path):
    written, skipped = make_figures(synthetic(recovery=False), tmp_path / "out")
    assert "recovery_vs_jamming.png" not in [pathlib.Path(p).name for p in written]
    assert [name for name, _ in skipped] == ["recovery_vs_jamming.png"]
    assert "recovery_longest_s" in skipped[0][1]


def test_make_figures_on_a_table_with_only_a_loss_axis(tmp_path):
    written, _ = make_figures(load_table(FIXTURE), tmp_path / "out")
    assert sorted(pathlib.Path(p).name for p in written) == [
        "delivery_vs_loss.png", "error_vs_loss.png"]


def test_make_figures_fixes_axes_with_where(tmp_path):
    written, _ = make_figures(synthetic(), tmp_path / "out", {"jamming": "none"})
    names = [pathlib.Path(p).name for p in written]
    assert "error_vs_loss.png" in names
    assert not any("jamming" in n for n in names)  # that axis is fixed, not plotted


def test_make_figures_with_nothing_to_draw_is_an_error(tmp_path):
    text = ("experiment,cell,color,replicate,status,verified,scenario,seed,delivery_ratio\n"
            "t,0,red,0,ok,0,t.a,1,0.5\n")
    with pytest.raises(FigureError):
        make_figures(parse_table(text), tmp_path / "out")


def test_the_command_line_writes_figures(tmp_path, capsys):
    out = tmp_path / "figs"
    assert main(["figures", str(FIXTURE), "--out", str(out)]) == 0
    assert sorted(p.name for p in out.iterdir()) == ["delivery_vs_loss.png", "error_vs_loss.png"]
    assert "wrote" in capsys.readouterr().out


def test_the_command_line_reports_bad_input_with_status_two(tmp_path, capsys):
    out = str(tmp_path / "figs")
    assert main(["figures", str(tmp_path / "missing.csv"), "--out", out]) == 2
    assert main(["figures", str(FIXTURE), "--out", out, "--where", "loss"]) == 2
    assert main(["figures", str(FIXTURE), "--out", out, "--where", "nope=x"]) == 2
    assert main(["figures", str(FIXTURE), "--out", out, "--where", "loss=p99"]) == 2
    assert "swarmsim_analysis:" in capsys.readouterr().err
