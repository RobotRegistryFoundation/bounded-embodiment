"""rawdata: the strict CSV readers for be-samples-csv/1, be-commands-csv/1, be-state-csv/1 and
be-accuracy-csv/1."""

from __future__ import annotations

import gzip

import pytest
from bounded_embodiment import rawdata
from bounded_embodiment.rawdata import (
    MAX_FINDINGS,
    AccuracyRow,
    Command,
    RawDataError,
    Sample,
    StateRow,
)

SAMPLES_HEADER = "t_s,point,x_m,y_m,z_m,valid,residual_m\n"
COMMANDS_HEADER = "seq,t_sent_s,t_done_s,strategy,payload_json,result,error_kind\n"
STATE_HEADER = "t_s,source,joint,value\n"
ACCURACY_HEADER = "phase,label,ref_x_m,ref_y_m,ref_z_m,meas_x_m,meas_y_m,meas_z_m,n\n"


def codes(err: RawDataError) -> list[str]:
    return [f.code for f in err.findings]


def fails(reader, text: str | bytes) -> RawDataError:
    data = text.encode("utf-8") if isinstance(text, str) else text
    with pytest.raises(RawDataError) as info:
        reader(data)
    return info.value


# --------------------------------------------------------------------------- good files


def test_samples_read_with_optional_cells():
    rows = rawdata.read_samples(
        (SAMPLES_HEADER + "0.0,tool,0.3,0,0.175,1,0.0005\n0.033,tool,,,,0,\n"
         "0.066,elbow,1e-05,-0,.5,1,\r\n").encode()
    )
    assert rows == [
        Sample(0.0, "tool", 0.3, 0.0, 0.175, 1, 0.0005),
        Sample(0.033, "tool", None, None, None, 0, None),
        Sample(0.066, "elbow", 1e-05, -0.0, 0.5, 1, None),
    ]
    assert isinstance(rows[0], tuple) and rows[0].x_m == 0.3


def test_commands_keep_quoted_payload_and_empty_optionals():
    rows = rawdata.read_commands(
        (COMMANDS_HEADER + '0,0.0,0.004,uniform,"{""q"":[1,2]}",ok,\n'
         "1,0.02,,malformed,,,timeout\n").encode()
    )
    assert rows == [
        Command(0, 0.0, 0.004, "uniform", '{"q":[1,2]}', "ok", None),
        Command(1, 0.02, None, "malformed", None, None, "timeout"),
    ]


def test_state_and_accuracy_read():
    assert rawdata.read_state((STATE_HEADER + "1.5,encoder,elbow_flex,0.25\n").encode()) == [
        StateRow(1.5, "encoder", "elbow_flex", 0.25)
    ]
    rows = rawdata.read_accuracy(
        (ACCURACY_HEADER + "pre,p1,0.3,0,0.2,0.301,0,0.2,30\n"
         "post,p1,0.3,0,0.2,0.3015,0,0.2,30\njitter,j1,,,,0.3,0.0,0.2,1\n").encode()
    )
    assert rows[0] == AccuracyRow("pre", "p1", 0.3, 0.0, 0.2, 0.301, 0.0, 0.2, 30)
    assert rows[2] == AccuracyRow("jitter", "j1", None, None, None, 0.3, 0.0, 0.2, 1)


def test_header_only_file_has_no_rows():
    assert rawdata.read_samples(SAMPLES_HEADER.encode()) == []


def test_floats_read_back_bit_for_bit():
    values = [0.1, 1 / 3, 2.0 ** -1074, 1.7976931348623157e308, -123456.789e-30]
    text = SAMPLES_HEADER + "".join(f"{v!r},p,{v!r},{v!r},{v!r},1,\n" for v in values)
    rows = rawdata.read_samples(text.encode())
    assert [r.t_s for r in rows] == values


def test_gzip_is_read_transparently(tmp_path):
    text = (SAMPLES_HEADER + "0.0,tool,0.3,0,0.175,1,\n").encode()
    expected = rawdata.read_samples(text)
    assert rawdata.read_samples(gzip.compress(text)) == expected
    path = tmp_path / "samples.csv.gz"
    path.write_bytes(gzip.compress(text))
    assert rawdata.read_samples(path) == expected
    assert rawdata.read_samples(str(path)) == expected
    plain = tmp_path / "samples.csv"
    plain.write_bytes(text)
    assert rawdata.read("be-samples-csv/1", plain) == expected


def test_a_gz_name_must_hold_gzip(tmp_path):
    path = tmp_path / "samples.csv.gz"
    path.write_bytes((SAMPLES_HEADER).encode())
    with pytest.raises(RawDataError) as info:
        rawdata.read_samples(path)
    assert codes(info.value) == ["RAW_UNREADABLE"]
    assert info.value.findings[0].where == "samples.csv.gz"


def test_truncated_gzip_and_missing_file(tmp_path):
    assert codes(fails(rawdata.read_samples, gzip.compress(SAMPLES_HEADER.encode())[:-6])) == [
        "RAW_UNREADABLE"
    ]
    with pytest.raises(RawDataError) as info:
        rawdata.read_samples(tmp_path / "nope.csv")
    assert codes(info.value) == ["RAW_UNREADABLE"]


def test_unknown_format_and_bad_source_type():
    with pytest.raises(KeyError):
        rawdata.read("be-samples-csv/2", b"")
    with pytest.raises(TypeError):
        rawdata.read_samples(12)


# --------------------------------------------------------------------------- refusals


@pytest.mark.parametrize(
    ("text", "code"),
    [
        ("", "RAW_HEADER"),
        ("t_s,point,x,y,z,valid,residual_m\n", "RAW_HEADER"),
        ("t_s,point,x_m,y_m,z_m,valid\n", "RAW_HEADER"),
        (SAMPLES_HEADER.replace(",", ", "), "RAW_HEADER"),
        ("﻿" + SAMPLES_HEADER, "RAW_ENCODING"),
    ],
)
def test_header_must_match_exactly(text, code):
    assert codes(fails(rawdata.read_samples, text)) == [code]


def test_not_utf8():
    assert codes(fails(rawdata.read_samples, SAMPLES_HEADER.encode() + b"0,t\xff\n")) == [
        "RAW_ENCODING"
    ]


@pytest.mark.parametrize(
    "cell",
    ["nan", "NaN", "inf", "-Infinity", "1_000", " 1.0", "1.0 ", "+1", "1e999", "0x10", "1e", "",
     "١"],
)
def test_numbers_are_strict(cell):
    err = fails(rawdata.read_samples, SAMPLES_HEADER + f"{cell},tool,0,0,0,1,\n")
    assert codes(err) == ["RAW_VALUE"]
    assert err.findings[0].where == "<bytes>:2"
    assert err.findings[0].detail.startswith("t_s:")


@pytest.mark.parametrize("cell", ["0", "-0", "5.", ".5", "1e-05", "1E+3", "-2.5e-3", "007"])
def test_numbers_that_are_accepted(cell):
    assert len(rawdata.read_samples((SAMPLES_HEADER + f"{cell},tool,0,0,0,1,\n").encode())) == 1


@pytest.mark.parametrize(
    "row",
    [
        "0.0,tool,0.3,0,0.175,1\n",  # too few fields
        "0.0,tool,0.3,0,0.175,1,,extra\n",  # too many
        "\n",  # a blank line
    ],
)
def test_rows_need_every_field(row):
    assert codes(fails(rawdata.read_samples, SAMPLES_HEADER + row)) == ["RAW_ROW"]


@pytest.mark.parametrize(
    "row",
    [
        "0.0,tool,,,,1,\n",  # valid without coordinates
        "0.0,tool,0.3,,0.1,0,\n",  # partial coordinates
        "0.0,tool,0.3,0,0.1,2,\n",  # valid is 0 or 1
        "0.0,tool,0.3,0,0.1,true,\n",
        "0.0,,0.3,0,0.1,1,\n",  # empty point
        "0.0, tool,0.3,0,0.1,1,\n",  # whitespace around an identifier
    ],
)
def test_sample_cells(row):
    assert codes(fails(rawdata.read_samples, SAMPLES_HEADER + row)) == ["RAW_VALUE"]


def test_csv_syntax_error():
    err = fails(rawdata.read_commands, COMMANDS_HEADER + '0,0.0,,uniform,"{"q"}",ok,\n')
    assert codes(err) == ["RAW_SYNTAX"]


def test_commands_order_and_times():
    assert codes(fails(rawdata.read_commands, COMMANDS_HEADER + "1,0.0,,a,,,\n1,0.1,,a,,,\n")) == [
        "RAW_ORDER"
    ]
    assert codes(fails(rawdata.read_commands, COMMANDS_HEADER + "1,0.5,,a,,,\n2,0.4,,a,,,\n")) == [
        "RAW_ORDER"
    ]
    assert codes(fails(rawdata.read_commands, COMMANDS_HEADER + "1,0.5,0.4,a,,,\n")) == [
        "RAW_VALUE"
    ]
    assert codes(fails(rawdata.read_commands, COMMANDS_HEADER + "-1,0.5,,a,,,\n")) == [
        "RAW_VALUE"
    ]
    # equal send times are allowed (a burst can share a clock tick)
    same_tick = COMMANDS_HEADER + "1,0.5,,a,,,\n2,0.5,,a,,,\n"
    assert len(rawdata.read_commands(same_tick.encode())) == 2


@pytest.mark.parametrize(
    ("row", "code"),
    [
        ("jitter,j1,0.3,0,0.2,0.3,0,0.2,1\n", "RAW_VALUE"),  # jitter rows have no ref
        ("jitter,j1,,,,0.3,0,0.2,5\n", "RAW_VALUE"),  # jitter rows are single samples
        ("pre,p1,,,,0.3,0,0.2,30\n", "RAW_VALUE"),  # pre rows need a ref
        ("post,p1,0.3,,0.2,0.3,0,0.2,30\n", "RAW_VALUE"),  # partial ref
        ("pre,p1,0.3,0,0.2,0.3,0,0.2,0\n", "RAW_VALUE"),  # n >= 1
        ("pre,p1,0.3,0,0.2,,0,0.2,30\n", "RAW_VALUE"),  # meas is required
        ("mid,p1,0.3,0,0.2,0.3,0,0.2,30\n", "RAW_VALUE"),
        ("pre,p1,0.3,0,0.2,0.3,0,0.2,30\npre,p1,0.3,0,0.2,0.3,0,0.2,30\n", "RAW_DUPLICATE"),
    ],
)
def test_accuracy_rules(row, code):
    assert codes(fails(rawdata.read_accuracy, ACCURACY_HEADER + row)) == [code]


def test_same_label_in_pre_and_post_and_repeated_jitter_labels_are_fine():
    text = (ACCURACY_HEADER + "pre,p1,0.3,0,0.2,0.3,0,0.2,30\npost,p1,0.3,0,0.2,0.3,0,0.2,30\n"
            "jitter,j1,,,,0.3,0,0.2,1\njitter,j1,,,,0.3,0,0.2,1\n")
    assert len(rawdata.read_accuracy(text.encode())) == 4


@pytest.mark.parametrize(
    "row",
    [
        "0.0,camera,elbow,1.0\n",
        "0.0,fk_tool,tool.x,0.3\n",  # the old name is accepted nowhere
        "0.0,fk,tool_x,0.3\n",  # fk joints are <point>.x|y|z
        "0.0,fk,tool.w,0.3\n",
        "0.0,fk,.x,0.3\n",
        "0.0,fk,tool.,0.3\n",
        "0.0,fk,tool.X,0.3\n",
    ],
)
def test_state_source_and_fk_joint_are_checked(row):
    assert codes(fails(rawdata.read_state, STATE_HEADER + row)) == ["RAW_VALUE"]


FK_STATE = (
    STATE_HEADER
    + "0.0,encoder,elbow_flex,0.25\n"
    + "0.0,fk,tool.x,0.30\n0.0,fk,tool.y,0.01\n0.0,fk,tool.z,0.175\n"
    + "0.0,fk,gripper.tip.z,0.10\n0.0,fk,gripper.tip.x,0.31\n0.0,fk,gripper.tip.y,0.02\n"
    + "0.02,fk,tool.y,0.011\n0.02,fk,tool.x,0.301\n0.02,fk,tool.z,0.176\n"
    + "0.02,encoder,elbow_flex,0.26\n"
)


def test_fk_samples_from_state():
    rows = rawdata.read_state(FK_STATE.encode())
    assert rawdata.fk_samples_from_state(rows) == [
        Sample(0.0, "tool", 0.30, 0.01, 0.175, 1, None),
        Sample(0.0, "gripper.tip", 0.31, 0.02, 0.10, 1, None),  # a dotted point id
        Sample(0.02, "tool", 0.301, 0.011, 0.176, 1, None),
    ]
    assert rawdata.fk_samples_from_state([]) == []
    assert rawdata.fk_samples_from_state(rows[:1]) == []  # encoder rows only


def _fk_fails(rows) -> RawDataError:
    with pytest.raises(RawDataError) as info:
        rawdata.fk_samples_from_state(rows)
    return info.value


def test_fk_samples_need_all_three_axes_once():
    full = [StateRow(0.0, "fk", "tool.x", 0.3), StateRow(0.0, "fk", "tool.y", 0.0),
            StateRow(0.0, "fk", "tool.z", 0.2)]
    err = _fk_fails(full[:2])
    assert codes(err) == ["RAW_VALUE"] and "has no tool.z" in err.findings[0].detail
    err = _fk_fails([*full, StateRow(0.0, "fk", "tool.y", 0.1)])
    assert codes(err) == ["RAW_DUPLICATE"] and err.findings[0].where == "state rows:3"
    # a different time is a different sample, which then lacks axes of its own
    assert codes(_fk_fails([*full, StateRow(0.001, "fk", "tool.x", 0.3)])) == ["RAW_VALUE"]


@pytest.mark.parametrize(
    ("row", "code"),
    [
        (StateRow(0.0, "fk_tool", "tool.x", 0.3), "RAW_VALUE"),
        (StateRow(0.0, "fk", "tool", 0.3), "RAW_VALUE"),
        (StateRow(0.0, "fk", "tool.x", float("nan")), "RAW_VALUE"),
        (StateRow(float("inf"), "fk", "tool.x", 0.3), "RAW_VALUE"),
        ((0.0, "fk", "tool.x"), "RAW_ROW"),
    ],
)
def test_fk_samples_refuse_bad_rows(row, code):
    assert codes(_fk_fails([row])) == [code]


def test_every_bad_row_is_reported_not_just_the_first():
    text = SAMPLES_HEADER + "x,tool,0,0,0,1,\n0.1,tool,0,0,0,1,\n0.2,tool,0,0,0,7,\n"
    err = fails(rawdata.read_samples, text)
    assert [(f.code, f.where) for f in err.findings] == [
        ("RAW_VALUE", "<bytes>:2"),
        ("RAW_VALUE", "<bytes>:4"),
    ]
    assert "invalid raw data" in str(err)


def test_findings_are_capped():
    text = SAMPLES_HEADER + "x,tool,0,0,0,1,\n" * (MAX_FINDINGS + 10)
    err = fails(rawdata.read_samples, text)
    assert len(err.findings) == MAX_FINDINGS + 1
    assert err.findings[-1].code == "RAW_TOO_MANY_ERRORS"
    assert err.findings[-1].severity == "info"


def test_raw_data_error_is_a_value_error():
    assert issubclass(RawDataError, ValueError)
