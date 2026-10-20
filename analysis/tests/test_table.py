import math
import pathlib

import pytest

from swarmsim_analysis import TableError, load_table, parse_table

FIXTURE = pathlib.Path(__file__).parent / "fixtures" / "smoke.csv"

HEADER = "experiment,cell,loss,size,replicate,status,verified,scenario,seed,score,note_count"


def make(*rows):
    return "\n".join([HEADER, *rows]) + "\n"


def test_a_batch_output_loads_with_its_axes_and_metrics():
    table = load_table(FIXTURE)
    assert table.axes == ["loss", "size", "length"]
    assert "formation_rmse_settled_m" in table.metrics
    assert "scenario" not in table.metrics and "seed" not in table.metrics
    assert len(table.rows) == 8  # 4 cells x 2 seeds
    assert len(table.ok_rows()) == 8
    assert table.levels("loss") == ["p00", "p30"]
    assert table.levels("size") == ["n04", "n06"]
    first = table.rows[0]
    assert first["cell"] == 0 and first["replicate"] == 0 and first["seed"] == 1
    assert isinstance(first["formation_rmse_m"], float)


def test_an_empty_metric_cell_is_nan_not_zero():
    table = parse_table(make("t,0,p00,n04,0,ok,0,t.a,1,,3"))
    row = table.rows[0]
    assert math.isnan(row["score"])
    assert row["note_count"] == 3.0


def test_a_failed_row_has_a_status_and_empty_metrics():
    table = parse_table(make("t,0,p00,n04,0,ok,0,t.a,1,1.5,2", "t,1,p00,n06,0,run failed: x,0,t.b,,,"))
    assert len(table.ok_rows()) == 1
    assert len(table.failed_rows()) == 1
    failed = table.failed_rows()[0]
    assert failed["seed"] is None
    assert math.isnan(failed["score"])


def test_axes_are_the_columns_between_cell_and_replicate():
    table = parse_table(make("t,0,p00,n04,0,ok,0,t.a,1,1.5,2"))
    assert table.axes == ["loss", "size"]
    assert table.metrics == ["score", "note_count"]


def test_no_axes_is_allowed():
    text = "experiment,cell,replicate,status,verified,scenario,seed,score\nt,0,0,ok,1,t.base,1,2.5\n"
    table = parse_table(text)
    assert table.axes == []
    assert table.rows[0]["score"] == 2.5
    assert table.rows[0]["verified"] is True


def test_malformed_tables_raise_table_error():
    cases = {
        "": "empty",
        "a,b,c\n1,2,3\n": "first columns",
        "experiment,cell,loss\n": "replicate",
        "experiment,cell,replicate,verified,status\n": "after the axes",
        "experiment,cell,loss,loss,replicate,status,verified\n": "same name",
        make("t,0,p00,n04,0,ok"): "cells",
        make("t,0,p00,n04,0,ok,0,t.a,1,abc,2"): "not a number",
    }
    for text, expected in cases.items():
        with pytest.raises(TableError) as info:
            parse_table(text)
        assert expected in str(info.value), (text, str(info.value))


def test_levels_of_an_unknown_axis_is_a_key_error():
    table = parse_table(make("t,0,p00,n04,0,ok,0,t.a,1,1.5,2"))
    with pytest.raises(KeyError):
        table.levels("nope")
