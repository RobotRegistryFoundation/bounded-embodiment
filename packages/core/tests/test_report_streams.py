"""The streams a report's generator can name (be-hostile, be-control, be-waypoints), their
digests, the check of raw/commands.csv against them, and the EV-10 waypoints file."""

from __future__ import annotations

import copy
import json

import pytest
from _report_helpers import RUNS, golden, in_repo, read_report
from bounded_embodiment import generator
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.report import streams
from bounded_embodiment.report.waypoints import (
    WaypointsError,
    check_commands,
    load_waypoints,
)

pytestmark = in_repo


def space():
    return load_command_space(golden().COMMAND_SPACE)


def rows_of(proposals) -> list[tuple]:
    return [(p.seq, p.strategy, generator.encode_payload(p.payload)) for p in proposals]


# --------------------------------------------------------------------------- be-hostile


def test_hostile_digest_is_the_generators() -> None:
    g = golden()
    params = generator.normalize_params(g.HOSTILE_PARAMS)
    assert streams.stream_digest(streams.HOSTILE, space(), 11, params) == generator.stream_sha256(
        space(), 11, params
    )


def test_check_whole_stream_prefix_and_mismatch() -> None:
    g = golden()
    proposals = list(generator.hostile_stream(space(), 11, g.HOSTILE_PARAMS))
    rows = rows_of(proposals)
    whole = streams.check("be-hostile", "1", space(), 11, g.HOSTILE_PARAMS, rows)
    assert whole.findings == () and whole.commands_compared == len(rows) and whole.aborted is False
    prefix = streams.check("be-hostile", "1", space(), 11, g.HOSTILE_PARAMS, rows[:10])
    assert [f.code for f in prefix.findings] == ["STREAM_PREFIX"] and prefix.aborted is True
    assert prefix.stream_sha256 == whole.stream_sha256 and prefix.commands_compared == 10
    bad = list(rows)
    bad[3] = (3, "uniform", "{}")
    mismatch = streams.check("be-hostile", "1", space(), 11, g.HOSTILE_PARAMS, bad)
    assert [f.code for f in mismatch.findings] == ["STREAM_MISMATCH"]
    assert mismatch.commands_compared is None and mismatch.aborted is None
    assert mismatch.stream_sha256 == whole.stream_sha256


def test_compare_rows_is_compare_to_commands_for_any_stream() -> None:
    g = golden()
    rows = rows_of(generator.hostile_stream(space(), 11, g.HOSTILE_PARAMS))
    for cut in (rows, rows[:5], [*rows[:5], (5, "max", "{}")], [(1, "uniform", "{}")]):
        want = generator.compare_to_commands(space(), 11, g.HOSTILE_PARAMS, cut)
        assert (
            generator.compare_rows(generator.hostile_stream(space(), 11, g.HOSTILE_PARAMS), cut)
            == want
        )


@pytest.mark.parametrize(
    ("name", "version", "seed", "params", "code"),
    [
        ("be-hostile", "2", 7, {}, "GENERATOR_UNSUPPORTED"),
        ("be-random", "1", 7, {}, "GENERATOR_UNSUPPORTED"),
        ("be-hostile", "1", -1, {}, "STREAM_PARAMS_INVALID"),
        ("be-hostile", "1", True, {}, "STREAM_PARAMS_INVALID"),
        ("be-hostile", "1", 7, {"seconds": 0}, "STREAM_PARAMS_INVALID"),
        ("be-hostile", "1", 7, "not params", "STREAM_PARAMS_INVALID"),
        ("be-hostile", "1", 7, {"seconds": 86400.0, "base_hz": 10000.0}, "STREAM_TOO_LARGE"),
        (
            "be-control",
            "1",
            0,
            {"target": "nowhere", "seconds": 1.0, "hz": 50.0, "step_cap": None},
            "STREAM_PARAMS_INVALID",
        ),
        ("be-control", "1", 0, {"target": "into_table", "seconds": 1.0}, "STREAM_PARAMS_INVALID"),
        (
            "be-control",
            "1",
            0,
            {"target": "into_table", "seconds": 1.0, "hz": 50.0, "step_cap": {"x": 0.1}},
            "STREAM_PARAMS_INVALID",
        ),
        (
            "be-waypoints",
            "1",
            0,
            {"cycles": 1, "commands_per_visit": 1, "hz": 50.0},
            "STREAM_PARAMS_INVALID",
        ),
        (
            "be-waypoints",
            "1",
            0,
            {"cycles": 0, "commands_per_visit": 1, "hz": 50.0},
            "STREAM_PARAMS_INVALID",
        ),
    ],
)
def test_check_refuses_bad_declarations(name, version, seed, params, code) -> None:
    result = streams.check(name, version, space(), seed, params, [])
    assert [f.code for f in result.findings] == [code]
    assert result.stream_sha256 is None


# --------------------------------------------------------------------------- be-control


def test_control_stream_is_the_generators() -> None:
    p = golden().CONTROL_PARAMS
    mine = list(streams.control_stream(space(), p))
    theirs = list(
        generator.control_stream(space(), p["target"], p["seconds"], p["hz"], p["step_cap"])
    )
    assert mine == theirs and len(mine) == 100 and {c.strategy for c in mine} == {"drive_out"}


def test_control_digest_matches_the_golden_report() -> None:
    report = read_report(RUNS / "ev03-control")
    digest = streams.stream_digest(streams.CONTROL, space(), 0, golden().CONTROL_PARAMS)
    assert report["generator"]["stream_sha256"] == digest
    # the digest pins the seed, the params and the space
    assert streams.stream_digest(streams.CONTROL, space(), 1, golden().CONTROL_PARAMS) != digest
    capped = dict(golden().CONTROL_PARAMS, step_cap={"x": 0.01, "y": 0.01, "z": 0.01})
    assert streams.stream_digest(streams.CONTROL, space(), 0, capped) != digest


def test_control_params_normalize_numbers() -> None:
    a = streams.normalize(
        "be-control", {"target": "into_table", "seconds": 2, "hz": 50, "step_cap": {"x": 1}}
    )
    assert a == {"target": "into_table", "seconds": 2.0, "hz": 50.0, "step_cap": {"x": 1.0}}
    assert streams.normalize("be-control", a) == a


# --------------------------------------------------------------------------- be-waypoints


def waypoints():
    return load_waypoints(golden().waypoints_doc())


def test_waypoint_tour() -> None:
    tour = list(
        streams.waypoint_stream(
            space(), waypoints(), {"cycles": 2, "commands_per_visit": 3, "hz": 10.0}
        )
    )
    assert [p.strategy for p in tour] == [
        f"waypoint:{w}" for _ in range(2) for w in "abcd" for _ in range(3)
    ]
    assert [p.seq for p in tour] == list(range(24))
    assert [p.t_s for p in tour[:4]] == [0.0, 0.1, 0.2, 0.3]
    assert tour[0].payload == {"x": 0.25, "y": -0.05, "z": 0.15}
    assert tour[-1].payload == {"x": 0.25, "y": 0.05, "z": 0.2}


def test_waypoint_digest_pins_the_waypoints_file() -> None:
    params = {"cycles": 1, "commands_per_visit": 2, "hz": 50.0}
    digest = streams.stream_digest(streams.WAYPOINTS, space(), 0, params, waypoints())
    doc = golden().waypoints_doc()
    doc["description"] = "changed text, same tour"
    assert (
        streams.stream_digest(streams.WAYPOINTS, space(), 0, params, load_waypoints(doc)) != digest
    )
    report = read_report(RUNS / "ev10-pass")
    assert report["generator"]["stream_sha256"] == streams.stream_digest(
        streams.WAYPOINTS, space(), 0, golden().WAYPOINT_PARAMS, waypoints()
    )


def test_waypoints_need_a_position_space() -> None:
    velocity = load_command_space(
        {
            "command_space_version": "0.1",
            "kind": "velocity",
            "axes": [{"key": "x", "unit": "m/s", "lo": -1.0, "hi": 1.0, "home": 0.0}],
        }
    )
    with pytest.raises(ValueError, match="position command space"):
        list(
            streams.waypoint_stream(
                velocity, waypoints(), {"cycles": 1, "commands_per_visit": 1, "hz": 1.0}
            )
        )


def test_waypoint_params_are_exact() -> None:
    for bad in (
        {"cycles": 1, "commands_per_visit": 1},
        {"cycles": 1, "commands_per_visit": 1, "hz": 1.0, "dwell_s": 1.0},
        {"cycles": True, "commands_per_visit": 1, "hz": 1.0},
        {"cycles": 1.5, "commands_per_visit": 1, "hz": 1.0},
    ):
        with pytest.raises(ValueError):
            streams.normalize("be-waypoints", bad)


# --------------------------------------------------------------------------- the waypoints file


def test_waypoints_file() -> None:
    w = waypoints()
    assert [x.id for x in w.waypoints] == ["a", "b", "c", "d"] and w.point == "tool"
    assert w.for_scoring()[0] == {"id": "a", "position_m": [0.25, -0.05, 0.15]}
    assert check_commands(w, space()) == []
    assert load_waypoints(json.dumps(golden().waypoints_doc()).encode()).digest == w.digest


@pytest.mark.parametrize(
    ("change", "code"),
    [
        (lambda d: d.update(waypoints_version="0.2"), "WAYPOINTS_VERSION_UNSUPPORTED"),
        (
            lambda d: d["waypoints"].append(copy.deepcopy(d["waypoints"][0])),
            "WAYPOINTS_DUPLICATE_ID",
        ),
        (lambda d: d["waypoints"][0].update(position_m=[0.1, 0.2]), "SCHEMA_INVALID"),
        (lambda d: d["waypoints"][0].update(extra=1), "SCHEMA_INVALID"),
        (lambda d: d["waypoints"][0].update(id="has space"), "SCHEMA_INVALID"),
    ],
)
def test_bad_waypoints_files(change, code) -> None:
    doc = golden().waypoints_doc()
    change(doc)
    with pytest.raises(WaypointsError) as exc:
        load_waypoints(doc)
    assert code in [f.code for f in exc.value.findings]


@pytest.mark.parametrize(
    ("command", "code"),
    [
        ({"x": 0.3, "y": 0.0}, "WAYPOINTS_COMMAND_MISSING_AXIS"),
        ({"x": 0.3, "y": 0.0, "z": 0.1, "w": 1.0}, "WAYPOINTS_COMMAND_UNKNOWN_AXIS"),
        ({"x": 0.3, "y": 0.0, "z": 9.0}, "WAYPOINTS_COMMAND_OUTSIDE_LIMITS"),
    ],
)
def test_waypoint_commands_against_the_space(command, code) -> None:
    doc = golden().waypoints_doc()
    doc["waypoints"][0]["command"] = command
    assert [f.code for f in check_commands(load_waypoints(doc), space())] == [code]


def test_waypoints_parse_strictly() -> None:
    with pytest.raises(WaypointsError) as exc:
        load_waypoints(b'{"waypoints_version": "0.1", "waypoints_version": "0.1"}')
    assert [f.code for f in exc.value.findings] == ["WAYPOINTS_PARSE_ERROR"]
