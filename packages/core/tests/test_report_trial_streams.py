"""be-stops/1 (EV-02) and be-model-loss/1 (EV-05): the streams, their segments, their digests and
the check of raw/commands.csv against them."""

from __future__ import annotations

import copy
import math

import pytest
from _report_helpers import golden, in_repo
from bounded_embodiment import canonical, generator
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.report import streams
from bounded_embodiment.report.plans import load_model_loss_plan, load_stop_plan

pytestmark = in_repo


def space():
    return load_command_space(golden().COMMAND_SPACE)


def velocity_space():
    doc = copy.deepcopy(golden().COMMAND_SPACE)
    doc["kind"] = "velocity"
    doc["axes"] = [
        {"key": "x", "unit": "m/s", "lo": -0.25, "hi": 0.25, "home": 0.0},
        {"key": "y", "unit": "m/s", "lo": -0.25, "hi": 0.25, "home": 0.0},
        {"key": "z", "unit": "m/s", "lo": -0.25, "hi": 0.25, "home": 0.0},
    ]
    del doc["far_targets"]
    return load_command_space(doc)


def stop_plan(trials=None):
    return load_stop_plan(golden().stop_plan_doc(list(trials or golden().EV02_TRIALS)))


def loss_plan():
    return load_model_loss_plan(golden().loss_plan_doc())


def rows_of(proposals) -> list[tuple]:
    return [(p.seq, p.strategy, generator.encode_payload(p.payload)) for p in proposals]


# --------------------------------------------------------------------------- shared rules


@pytest.mark.parametrize("hz", [1.0, 7.0, 30.0, 50.0, 100.0, 333.0, 1000.0])
@pytest.mark.parametrize("seconds", [0.001, 0.02, 0.1, 0.3, 0.7, 1.0, 1.1, 2.5, 3.3])
def test_ticks_counts_i_over_hz_below_seconds(seconds: float, hz: float) -> None:
    n = streams.ticks(seconds, hz)
    assert all(i / hz < seconds for i in range(n))
    assert n / hz >= seconds
    assert n == len([i for i in range(math.ceil(seconds * hz) + 2) if i / hz < seconds])


def test_walk_step() -> None:
    keys = ("x", "y")
    caps = {"x": 0.01, "y": 0.5}
    assert streams.walk_step({"x": 0.0, "y": 0.0}, {"x": 0.1, "y": -0.2}, keys, caps) == {
        "x": 0.01,
        "y": -0.2,
    }
    assert streams.walk_step({"x": 0.1, "y": 0.0}, {"x": 0.0, "y": 0.0}, keys, caps) == {
        "x": 0.1 - 0.01,  # one cap toward the target, in binary64
        "y": 0.0,
    }
    assert streams.walk_step({"x": 0.0, "y": 0.0}, {"x": 9.0, "y": -9.0}, keys, None) == {
        "x": 9.0,
        "y": -9.0,
    }


def test_split_segments() -> None:
    P = generator.Proposal
    props = [P(0, 0.0, "a", {}), P(1, 0.5, "a", {}), P(2, 0.0, "b", {}), P(3, 0.0, "c", {})]
    assert [[p.seq for p in s] for s in streams.split_segments(props)] == [[0, 1], [2], [3]]
    assert streams.split_segments([]) == []


# --------------------------------------------------------------------------- be-stops


def test_stop_stream_segments() -> None:
    g = golden()
    plan = stop_plan()
    proposals = list(streams.stop_stream(space(), plan, g.STOP_PARAMS))
    segments = streams.split_segments(proposals)
    assert len(segments) == len(plan.trials) == 6
    assert [p.seq for p in proposals] == list(range(len(proposals)))
    n_drive, n_return, n_after = (streams.ticks(s, 50.0) for s in (0.28, 0.7, 3.0))
    assert (n_drive, n_return, n_after) == (14, 35, 150)
    for k, (seg, (source, drive)) in enumerate(zip(segments, plan.trials)):
        kinds = [p.strategy for p in seg]
        want = (["return"] * (n_return if k else 0) + [f"drive:{drive}"] * n_drive + [f"stop:{source}"]
                + [f"continue:{drive}"] * n_after)
        assert kinds == want
        assert [p.t_s for p in seg] == [i / 50.0 for i in range(len(seg))]
        stop = kinds.index(f"stop:{source}")
        assert seg[stop].payload == {}
        if k:  # the return walk ends at the start (0.15 m away at 0.005 m a command)
            back = [p for p in seg if p.strategy == "return"][-1].payload
            assert back == {"x": 0.3, "y": 0.0, "z": 0.175}
            # and it starts one step from the command at the previous stop instant, near where a
            # machine whose stop held came to rest, not from where the commands after it went
            prev = segments[k - 1]
            at_stop = prev[[p.strategy for p in prev].index(f"stop:{plan.trials[k - 1][0]}") - 1].payload
            toward = 1.0 if at_stop["x"] < 0.3 else -1.0
            assert seg[0].payload["x"] == pytest.approx(at_stop["x"] + toward * 0.005)
            assert (seg[0].payload["y"], seg[0].payload["z"]) == (0.0, 0.175)
        sign = 1.0 if drive == "x_plus" else -1.0
        last = seg[stop - 1].payload
        assert last["x"] == pytest.approx(0.3 + sign * 14 * 0.005)
        assert (last["y"], last["z"]) == (0.0, 0.175)
        # D1: the model goes on commanding the drive through the stop: the walk carries on to
        # the drive (16 more commands, 0.08 m) and then repeats it
        after = [p.payload for p in seg[stop + 1:]]
        assert after[0]["x"] == pytest.approx(0.3 + sign * 15 * 0.005)
        target = {"x": 0.45 if sign > 0 else 0.15, "y": 0.0, "z": 0.175}
        assert after[15] == target and after[14] != target
        assert all(a == target for a in after[15:])


def test_stop_stream_refuses_a_walk_that_lands_before_the_stop() -> None:
    """In a position command space the machine must still be moving at the stop instant: a walk
    that has already landed on its drive leaves nothing for a dead stop to get wrong."""
    g = golden()
    for change in ({"step_cap": None}, {"drive_s": 0.7}, {"step_cap": {"x": 0.02, "y": 0.02, "z": 0.02}}):
        with pytest.raises(ValueError, match="lands on drive"):
            streams.stop_stream(space(), stop_plan(), {**g.STOP_PARAMS, **change})
    rows = rows_of(streams.stop_stream(space(), stop_plan(), g.STOP_PARAMS))
    found = streams.check("be-stops", "1", space(), 0, {**g.STOP_PARAMS, "step_cap": None}, rows,
                          plan=stop_plan())
    assert [f.code for f in found.findings] == ["STREAM_PARAMS_INVALID"]
    assert "lands on drive" in found.findings[0].detail


def test_velocity_spaces_restart_every_drive_and_repeat_it_after_the_stop() -> None:
    g = golden()
    doc = g.stop_plan_doc(list(g.EV02_TRIALS))
    doc["start"] = {"x": 0.0, "y": 0.0, "z": 0.0}
    doc["drives"] = [
        {"id": "x_plus", "command": {"x": 0.2, "y": 0.0, "z": 0.0}},
        {"id": "x_minus", "command": {"x": -0.2, "y": 0.0, "z": 0.0}},
    ]
    vplan = load_stop_plan(doc)
    vparams = {**g.STOP_PARAMS, "step_cap": {"x": 0.05, "y": 0.05, "z": 0.05}}
    segs = streams.split_segments(streams.stop_stream(velocity_space(), vplan, vparams))
    for seg in segs:
        assert not any(p.strategy == "return" for p in seg)
        assert seg[0].payload["x"] in (0.05, -0.05)  # every drive ramps up from the start
        assert seg[3].payload["x"] in (0.2, -0.2)  # landing on a velocity is what a drive is for
        drive = [p for p in seg if p.strategy.startswith("drive:")]
        after = [p for p in seg if p.strategy.startswith("continue:")]
        assert len(after) == 150 and all(p.payload == drive[-1].payload for p in after)
    # without a cap a velocity drive is a step to the drive velocity: allowed
    assert streams.split_segments(
        streams.stop_stream(velocity_space(), vplan, {**g.STOP_PARAMS, "step_cap": None}))


def test_stop_stream_refuses_what_it_cannot_make() -> None:
    g = golden()
    plan = stop_plan()
    with pytest.raises(ValueError, match="step_cap must name every axis"):
        list(streams.stop_stream(space(), plan, {**g.STOP_PARAMS, "step_cap": {"x": 0.01}}))
    doc = g.stop_plan_doc(list(g.EV02_TRIALS))
    doc["drives"][0]["command"]["x"] = 0.9
    with pytest.raises(ValueError, match="ev02-stop-plan.json"):
        list(streams.stop_stream(space(), load_stop_plan(doc), g.STOP_PARAMS))
    with pytest.raises(ValueError, match="needs the run's ev02-stop-plan.json"):
        streams.stream(streams.STOPS, space(), 0, g.STOP_PARAMS)
    with pytest.raises(ValueError, match="needs the run's ev02-stop-plan.json"):
        streams.stream(streams.STOPS, space(), 0, g.STOP_PARAMS, plan=loss_plan())


@pytest.mark.parametrize(
    "params",
    [
        {"hz": 50.0, "drive_s": 0.3, "return_s": 0.5, "after_s": 3.0},
        {"hz": 50.0, "drive_s": 0.3, "return_s": 0.5, "step_cap": None},
        {"hz": 50.0, "drive_s": 0.0, "return_s": 0.5, "after_s": 3.0, "step_cap": None},
        {"hz": 50.0, "drive_s": 0.3, "return_s": -1.0, "after_s": 3.0, "step_cap": None},
        {"hz": 50.0, "drive_s": 0.3, "return_s": 0.5, "after_s": 0.0, "step_cap": None},
        {"hz": 50.0, "drive_s": 0.3, "return_s": 0.5, "after_s": 3.0, "step_cap": {}},
        {"hz": 50.0, "drive_s": 0.3, "return_s": 0.5, "after_s": 3.0, "step_cap": {"x": 0.0}},
        {"hz": 50.0, "drive_s": 0.3, "return_s": 0.5, "after_s": 3.0, "step_cap": None, "seconds": 1.0},
    ],
)
def test_stop_params_are_exact(params: dict) -> None:
    with pytest.raises(ValueError):
        streams.normalize(streams.STOPS, params)


def test_stop_params_normalize() -> None:
    p = streams.normalize(
        streams.STOPS, {"hz": 50, "drive_s": 1, "return_s": 0, "after_s": 3, "step_cap": None}
    )
    assert p == {"hz": 50.0, "drive_s": 1.0, "return_s": 0.0, "after_s": 3.0, "step_cap": None}
    assert all(type(v) is float for k, v in p.items() if k != "step_cap")
    assert streams.normalize(streams.STOPS, p) == p


# --------------------------------------------------------------------------- be-model-loss


def test_model_loss_stream_segments() -> None:
    g = golden()
    params = streams.normalize(streams.MODEL_LOSS, g.LOSS_PARAMS)
    proposals = list(streams.model_loss_stream(space(), loss_plan(), g.LOSS_SEED, params))
    segments = streams.split_segments(proposals)
    assert len(segments) == params["trials"] == 3
    n_restart = streams.ticks(params["restart_s"], params["hz"])
    shortest, longest = (streams.ticks(s, 50.0) for s in params["motion_s"])
    for seg in segments:
        motion = [p for p in seg if p.strategy == "motion"]
        restart = [p for p in seg if p.strategy == "restart"]
        assert [p.strategy for p in seg] == ["motion"] * len(motion) + ["restart"] * n_restart
        assert shortest <= len(motion) <= longest
        assert [p.t_s for p in motion] == [i / 50.0 for i in range(len(motion))]
        t0 = (len(motion) - 1) / 50.0 + params["silence_s"]
        assert [p.t_s for p in restart] == [t0 + j / 50.0 for j in range(n_restart)]
    # one walk through the whole stream: no step is larger than the cap on any axis
    previous = {"x": 0.3, "y": 0.1, "z": 0.175}
    for p in proposals:
        for axis in ("x", "y", "z"):
            assert abs(p.payload[axis] - previous[axis]) <= 0.005 + 1e-12
        previous = p.payload
    ys = [p.payload["y"] for p in proposals]
    assert min(ys) == -0.1 and max(ys) == 0.1  # it went round the path


def test_model_loss_stream_is_seeded() -> None:
    g = golden()
    a = rows_of(streams.model_loss_stream(space(), loss_plan(), 5, g.LOSS_PARAMS))
    b = rows_of(streams.model_loss_stream(space(), loss_plan(), 5, g.LOSS_PARAMS))
    c = rows_of(streams.model_loss_stream(space(), loss_plan(), 6, g.LOSS_PARAMS))
    assert a == b and a != c


def test_path_cycle() -> None:
    g = golden()
    # 0.2 m each way at 0.005 m a command: 40 + 40 commands at 50 Hz
    assert streams.path_cycle_s(space(), loss_plan(), g.STOP_CAP, 50.0) == 1.6
    doc = g.loss_plan_doc()
    doc["path"] = doc["path"][:1]  # one leg, back to itself: one command (round-2 M5)
    assert streams.path_cycle_s(space(), load_model_loss_plan(doc), g.STOP_CAP, 50.0) == 0.02


@pytest.mark.parametrize("motion_s", [[1.0, 1.0], [1.0, 1.4], [1.0, 2.59]])
def test_model_loss_refuses_a_loss_that_cannot_fall_anywhere_on_the_path(motion_s) -> None:
    """motion_s must span a whole cycle of the path (and a period), or every loss could land at
    the same place on it, a reversal for example (review note 10; a = b included)."""
    g = golden()
    params = {**g.LOSS_PARAMS, "motion_s": motion_s}
    with pytest.raises(ValueError, match="at least one cycle of the path"):
        streams.model_loss_stream(space(), loss_plan(), 5, params)
    found = streams.check("be-model-loss", "1", space(), 5, params, [], plan=loss_plan())
    assert [f.code for f in found.findings] == ["STREAM_PARAMS_INVALID"]
    # one command, in a velocity space (in a position space it is a dwell, refused: round-2 M5):
    # the cycle is one command, one period
    doc = g.loss_plan_doc()
    doc["start"] = {"x": 0.0, "y": 0.0, "z": 0.0}
    doc["path"] = [{"x": 0.0, "y": 0.1, "z": 0.0}]
    one = load_model_loss_plan(doc)
    with pytest.raises(ValueError, match="at least one cycle of the path"):
        streams.model_loss_stream(velocity_space(), one, 5, {**g.LOSS_PARAMS, "motion_s": [1.0, 1.0]})
    assert list(streams.model_loss_stream(velocity_space(), one, 5, {**g.LOSS_PARAMS, "motion_s": [1.0, 1.02]}))


@pytest.mark.parametrize(
    ("change", "match"),
    [
        ({"motion_s": [2.0, 1.0]}, "not be decreasing"),
        ({"motion_s": [1.0]}, r"\[shortest, longest\]"),
        ({"silence_s": 0.01}, "at least one period"),
        ({"step_cap": None}, "step_cap must be a mapping"),
        ({"trials": 0}, "trials must be a whole number"),
        ({"trials": 1001}, "trials must be a whole number"),
        ({"restart_s": 0}, "restart_s must be above 0"),
    ],
)
def test_model_loss_params_refused(change: dict, match: str) -> None:
    with pytest.raises(ValueError, match=match):
        streams.normalize(streams.MODEL_LOSS, {**golden().LOSS_PARAMS, **change})


# --------------------------------------------------------------------------- digests and checks


def _digest_by_hand(name: str, seed: int, params: dict, plan) -> str:
    import hashlib

    p = streams.normalize(name, params)
    header = (
        f"{name}/1\n{canonical.canonicalize(p).decode('utf-8')}\n{seed}\n{space().digest}\n"
        f"{plan.digest}\n"
    )
    h = hashlib.sha256(header.encode("utf-8"))
    for c in streams.stream(name, space(), seed, p, plan=plan):
        h.update(f"{c.seq},{c.strategy},{generator.encode_payload(c.payload)}\n".encode())
    return "sha256:" + h.hexdigest()


def test_digests_pin_the_plan() -> None:
    g = golden()
    plan = stop_plan()
    d = streams.stream_digest(streams.STOPS, space(), 0, g.STOP_PARAMS, plan=plan)
    assert d == _digest_by_hand(streams.STOPS, 0, g.STOP_PARAMS, plan)
    doc = g.stop_plan_doc(list(g.EV02_TRIALS))
    doc["description"] = "another description, the same commands"
    other = streams.stream_digest(streams.STOPS, space(), 0, g.STOP_PARAMS, plan=load_stop_plan(doc))
    assert other != d
    lp = loss_plan()
    d5 = streams.stream_digest(streams.MODEL_LOSS, space(), 5, g.LOSS_PARAMS, plan=lp)
    assert d5 == _digest_by_hand(streams.MODEL_LOSS, 5, g.LOSS_PARAMS, lp)


def test_digests_match_the_golden_reports() -> None:
    from _report_helpers import read_report

    g = golden()
    for name, plan, seed, params in (
        ("ev02-pass", stop_plan(), 0, g.STOP_PARAMS),
        ("ev05-pass", loss_plan(), g.LOSS_SEED, g.LOSS_PARAMS),
    ):
        report = read_report(g.OUT / name)
        gen = report["generator"]
        assert gen["stream_sha256"] == streams.stream_digest(gen["name"], space(), seed, params, plan=plan)
        assert gen["seed"] == seed


def test_check_against_commands() -> None:
    g = golden()
    plan = stop_plan()
    rows = rows_of(streams.stop_stream(space(), plan, g.STOP_PARAMS))
    whole = streams.check("be-stops", "1", space(), 0, g.STOP_PARAMS, rows, plan=plan)
    assert whole.findings == () and whole.commands_compared == len(rows) and whole.aborted is False
    prefix = streams.check("be-stops", "1", space(), 0, g.STOP_PARAMS, rows[:20], plan=plan)
    assert [f.code for f in prefix.findings] == ["STREAM_PREFIX"] and prefix.aborted is True
    bad = list(rows)
    bad[15] = (15, "stop:power_loss", "{}")
    assert [f.code for f in streams.check("be-stops", "1", space(), 0, g.STOP_PARAMS, bad, plan=plan).findings] == [
        "STREAM_MISMATCH"
    ]
    missing = streams.check("be-stops", "1", space(), 0, g.STOP_PARAMS, rows)
    assert [f.code for f in missing.findings] == ["STREAM_PARAMS_INVALID"]
    lp = loss_plan()
    lrows = rows_of(streams.model_loss_stream(space(), lp, 5, g.LOSS_PARAMS))
    ok = streams.check("be-model-loss", "1", space(), 5, g.LOSS_PARAMS, lrows, plan=lp)
    assert ok.findings == () and ok.commands_compared == len(lrows)
    reseeded = streams.check("be-model-loss", "1", space(), 6, g.LOSS_PARAMS, lrows, plan=lp)
    assert [f.code for f in reseeded.findings] == ["STREAM_MISMATCH"]
    huge = {**g.LOSS_PARAMS, "trials": 1000, "motion_s": [80000.0, 80000.0]}
    assert [f.code for f in streams.check("be-model-loss", "1", space(), 5, huge, [], plan=lp).findings] == [
        "STREAM_TOO_LARGE"
    ]


def test_model_loss_in_a_velocity_space_starts_each_trial_and_restart_at_rest() -> None:
    """Round-2 kit review K4: in a velocity space the machine is at rest after each trial's hold,
    but the walk continued "as if the restart had not happened", so every trial after the first
    began with a full-speed command (ten caps), and the restart began at the path's velocity. In
    a velocity space each trial's motion, and its restart, now walk from the plan's start, and the
    restart walks back to it."""
    g = golden()
    doc = g.loss_plan_doc()
    doc["start"] = {"x": 0.0, "y": 0.0, "z": 0.0}
    doc["path"] = [{"x": 0.0, "y": 0.2, "z": 0.0}, {"x": 0.1, "y": 0.2, "z": 0.0}]
    plan = load_model_loss_plan(doc)
    caps = {"x": 0.02, "y": 0.02, "z": 0.02}
    params = {**g.LOSS_PARAMS, "motion_s": [1.0, 2.0], "step_cap": caps}
    segments = streams.split_segments(streams.model_loss_stream(velocity_space(), plan, 5, params))
    assert len(segments) == 3
    rest = {"x": 0.0, "y": 0.0, "z": 0.0}
    for seg in segments:
        motion = [p.payload for p in seg if p.strategy == "motion"]
        restart = [p.payload for p in seg if p.strategy == "restart"]
        assert motion[0] == {"x": 0.0, "y": 0.02, "z": 0.0}  # one cap from rest
        assert restart[0] == {"x": 0.0, "y": 0.02, "z": 0.0}
        assert restart[-1] == rest
        for walk in (motion, restart):  # each from rest (the silence lies between them)
            previous = rest
            for payload in walk:
                assert all(abs(payload[k] - previous[k]) <= caps[k] + 1e-12 for k in caps)
                previous = payload
    # a position space keeps one walk through the whole stream (unchanged)
    pos = streams.split_segments(streams.model_loss_stream(space(), loss_plan(), g.LOSS_SEED, g.LOSS_PARAMS))
    last = [p.payload for p in pos[0] if p.strategy == "motion"][-1]
    first_next = [p.payload for p in pos[1] if p.strategy == "motion"][0]
    assert max(abs(first_next[k] - last[k]) for k in ("x", "y", "z")) <= 0.005 + 1e-12


# --------------------------------------------------------------------------- catch_up rows (K3)


def _segment_starts(proposals) -> list[int]:
    starts, n = [], 0
    for seg in streams.split_segments(proposals):
        starts.append(n)
        n += len(seg)
    return starts


def _with_catch_up(rows, at: int, walk) -> list[tuple]:
    """``rows`` with ``walk`` (payloads) as catch_up rows before row ``at``, every row renumbered
    (seq counts every row of commands.csv)."""
    out = rows[:at] + [(None, streams.CATCH_UP, generator.encode_payload(p)) for p in walk] + rows[at:]
    return [(i, s, j) for i, (_, s, j) in enumerate(out)]


def _walk(start: dict, target: dict, caps: dict) -> list[dict]:
    """From start to within one cap of target, one walk step a command."""
    out, current = [], dict(start)
    while any(abs(target[k] - current[k]) > caps[k] for k in caps):
        current = streams.walk_step(current, target, tuple(caps), caps)
        out.append(dict(current))
    return out


def test_catch_up_rows_between_trials_are_checked_and_skipped() -> None:
    """Round-2 kit review K3: a position servo lags its command, so after a stop that held the
    machine rests short of the command at the stop instant, where the next trial's walk begins,
    and a slightly slower servo was refused there by the kit's guard. The bench now walks the
    machine from where it rests to within one step cap of the next trial's first command, at most
    one step cap per command, in catch_up rows that seq numbers with every other row. Core skips
    them when it compares the rows with the stream, after checking that they are such a walk,
    between two trials."""
    g = golden()
    plan = stop_plan()
    proposals = list(streams.stop_stream(space(), plan, g.STOP_PARAMS))
    rows = rows_of(proposals)
    starts = _segment_starts(proposals)
    first = proposals[starts[1]].payload
    rest = dict(first, x=first["x"] - 0.03)  # six caps short of it, along the last drive
    walk = _walk(rest, first, g.STOP_CAP)
    assert len(walk) == 5
    ok = streams.check("be-stops", "1", space(), 0, g.STOP_PARAMS, _with_catch_up(rows, starts[1], walk),
                       plan=plan)
    assert ok.findings == () and ok.commands_compared == len(rows) and ok.aborted is False

    def codes(rws) -> list[str]:
        return [f.code for f in streams.check("be-stops", "1", space(), 0, g.STOP_PARAMS, rws, plan=plan).findings]

    jump = [walk[0], dict(walk[1], x=walk[1]["x"] + 0.006)] + walk[2:]  # a step of 1.2 caps
    assert codes(_with_catch_up(rows, starts[1], jump)) == ["STREAM_CATCH_UP_INVALID"]
    assert codes(_with_catch_up(rows, starts[1], walk[:-1])) == ["STREAM_CATCH_UP_INVALID"]  # 2 caps short
    assert codes(_with_catch_up(rows, starts[1] + 5, walk)) == ["STREAM_CATCH_UP_INVALID"]  # inside a trial
    assert codes(_with_catch_up(rows, 0, _walk(dict(proposals[0].payload, x=0.27), proposals[0].payload,
                                                 g.STOP_CAP))) == ["STREAM_CATCH_UP_INVALID"]  # before trial 1
    assert codes(_with_catch_up(rows, len(rows), walk)) == ["STREAM_CATCH_UP_INVALID"]  # after the last
    no_z = [{"x": p["x"], "y": p["y"]} for p in walk]
    assert codes(_with_catch_up(rows, starts[1], no_z)) == ["STREAM_CATCH_UP_INVALID"]
    outside = [dict(p, z=0.5) for p in walk]  # z's limit is 0.35
    assert codes(_with_catch_up(rows, starts[1], outside)) == ["STREAM_CATCH_UP_INVALID"]
    # the rows keep their own numbering: a catch_up row that does not count breaks seq
    unnumbered = rows[:starts[1]] + [(starts[1], "catch_up", generator.encode_payload(walk[0]))] + rows[starts[1]:]
    assert codes(unnumbered) == ["STREAM_SEQ_GAP"]
    # a run that stopped during the catch-up is a prefix, and its last row is checked against
    # the trial it was heading for
    cut = _with_catch_up(rows[:starts[1]], starts[1], walk)
    assert codes(cut) == ["STREAM_PREFIX"]
    assert codes(_with_catch_up(rows[:starts[1]], starts[1], walk[:-1])) == ["STREAM_CATCH_UP_INVALID"]


def test_catch_up_rows_in_a_model_loss_run_and_not_in_a_velocity_space() -> None:
    g = golden()
    lp = loss_plan()
    proposals = list(streams.model_loss_stream(space(), lp, 5, g.LOSS_PARAMS))
    rows, starts = rows_of(proposals), _segment_starts(proposals)
    first = proposals[starts[2]].payload
    walk = _walk(dict(first, y=first["y"] + 0.02), first, g.STOP_CAP)
    ok = streams.check("be-model-loss", "1", space(), 5, g.LOSS_PARAMS, _with_catch_up(rows, starts[2], walk),
                       plan=lp)
    assert ok.findings == () and ok.commands_compared == len(rows)
    # a velocity machine starts every trial at rest from the plan's start: nothing to catch up
    doc = g.loss_plan_doc()
    doc["start"] = {"x": 0.0, "y": 0.0, "z": 0.0}
    doc["path"] = [{"x": 0.0, "y": 0.2, "z": 0.0}, {"x": 0.1, "y": 0.2, "z": 0.0}]
    vplan = load_model_loss_plan(doc)
    vparams = {**g.LOSS_PARAMS, "motion_s": [1.0, 2.0], "step_cap": {"x": 0.02, "y": 0.02, "z": 0.02}}
    vprops = list(streams.model_loss_stream(velocity_space(), vplan, 5, vparams))
    vstarts = _segment_starts(vprops)
    vwalk = [{"x": 0.0, "y": 0.0, "z": 0.0}]
    found = streams.check("be-model-loss", "1", velocity_space(), 5, vparams,
                          _with_catch_up(rows_of(vprops), vstarts[1], vwalk), plan=vplan)
    assert [f.code for f in found.findings] == ["STREAM_CATCH_UP_INVALID"]
    # and no other stream has catch_up rows
    assert streams.CATCH_UP not in {p.strategy for p in proposals}
