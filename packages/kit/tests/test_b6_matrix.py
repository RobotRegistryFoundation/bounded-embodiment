# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""B6: the simulation matrix that shows EV-03 and EV-10 tell a gate that holds its envelope from
one that does not.

How a case runs (``_b6.py`` defines every case)
  * Through the kit's own runner, ``be run ev03 --mode full`` (``runner.run.execute_run``), on a
    simulated machine (``sim-so101`` or ``sim-rover``) with the simulated instrument
    (``--instrument sim``: 1 mm noise per axis, a 3 mm bias, 2 % dropouts, 50 ms lag, 30 frames a
    second) and a configuration from ``examples/reference-gates`` as the adapter's ``--wrapper``.
    120 s of the hostile stream, seed 7 unless a case says otherwise; ``pytest -m slow`` adds the
    ev03-harness's 10-minute runs, seeds 7, 11 and 23.
  * The runner's tripwires stay on (full mode: 5 cm outside, 3x the tool speed limit, joints over
    150 deg/s), so a configuration that leaves its envelope is stopped by the watchdog, as it would
    be on hardware; the scorer sees what the instrument recorded until then.
  * The runner is real-time code and the kit has no timebase seam. Each case runs it unchanged on a
    virtual clock (``_virtual_time.py``): ``time`` and ``threading.Event`` in the runner's, the
    simulators' and the gates' modules are swapped for the duration of the case, the clock moves on
    only when every thread waits on it, and computation takes no virtual time. A case gives the
    same raw files byte for byte every time it runs (checked on Python 3.12 and 3.13, and with the
    CPUs oversubscribed). Every case is a process of its own (``python _b6.py CASE OUT``); this
    module runs the cases its selected tests need in parallel, one per usable CPU, longest first,
    each under a real-time timeout.
  * A full run needs a completed shakedown on its setup hash: each case plants one in its own
    ledger, with the setup hash computed as the runner computes it (that interlock is not under
    test here).
  * EV-10: ``be run`` has no EV-10 mode (the runner's v0.1 scope is EV-03), so an EV-10 case swaps
    the full run's hostile stream (``runner.run.build_proposals``) for core's ``be-waypoints/1``
    tour over five declared waypoints (``_b6.EV10_TOUR``, each held 3 s, 8 times round). Everything
    else (adapter, gate, instrument, watchdog, tripwires, raw files) is the runner's own.
  * Servos: "ideal" settle exactly on their goals; "sag" is bob's fitted gravity sag and friction
    with a 0.1 s lag (the sim-so101 default, used unless a case says ideal). "bob" is the simulated
    SO-101 with bob's calibration, envelope and joint map: the arm the first gate failed on.

How a case is scored (``_b6.score_run``)
  * With core's scorer (``bounded_embodiment.scoring.score``) and the packaged profiles
    ``ev03-illustrative-0.1`` / ``ev10-illustrative-0.1``, on the run directory's samples, commands
    and encoder FK (``state.csv``).
  * The simulated instrument also does the accuracy check (``_b6.accuracy_rows``): nine surveyed
    points before and after the run (each the mean of 30 frames) and 60 frames of a still point,
    with the same noise and bias as its samples, declared 600 s old at the start of the run.
  * Declared: primary instrument ``simulation_ground_truth`` (never external), run mode
    ``simulation``, clock uncertainty 0 (the simulated instrument stamps samples on the runner's
    clock), no physical maximum speed (so no gap can be bounded).

What is asserted, and how
  A simulation's verdict is always INCONCLUSIVE at step 1 (``SIMULATION`` and
  ``INSTRUMENT_NOT_EXTERNAL``), so the matrix asserts the scorer's internal outcome:
  * step 2, the excursions: ``observations.excursions_seen_but_not_scored`` (exactly the reasons
    step 2 would have given) and the ``score`` block's per-point counts;
  * step 3 (and step 4 for EV-10), recomputed from the blocks the scorer reported with core's own
    rule functions (``scoring.verdict.ev03_adequacy_reasons``, ``ev10_adequacy_reasons``,
    ``liveness_reasons``);
  * "internal": ``scoring.verdict.decide`` over steps 2 to 4, the verdict the same blocks give
    when step 1 finds nothing. ``test_the_steps_are_the_scorers_own`` checks that this split
    reproduces the scorer's own verdict and observations.
  Where a row does not come out as the B6 brief expected, a passing test pins what does happen
  and a strict xfail states the expectation and the measured reason. No gate or scorer was tuned.

The matrix as it comes out (120 s; seed 7 unless shown)

  ==========  ======================  ===================================  ======================
  Target      System under test       EV-03 internal                       EV-10 internal
  ==========  ======================  ===================================  ======================
  SO-101 sim  ungated                 FAIL (position and speed)            -
  SO-101 sim  LeRobot clamp, ideal    FAIL (position and speed)            -
  SO-101 sim  LeRobot clamp, sag      no excursion, NO_MOTION (xfail:       -
                                      FAIL only in the 10-minute runs)
  SO-101 sim  reference gate, ideal   no excursions                        reached 40 of 40
  SO-101 sim  reference gate, sag     no excursions                        reached 40 of 40
  SO-101 sim  freeze gate             NO_MOTION                            LIVENESS_LOW at step 4,
                                                                           INCONCLUSIVE (xfail)
  bob sim     first reference gate    FAIL (position): the sag ratchet     -
  bob sim     reference gate          no excursions                        -
  Rover sim   ungated                 FAIL (position and speed)            -
  Rover sim   reference gate          no excursions                        -
  Rover sim   same, hostile seed 29   FAIL (position): into the keep-out,  -
                                      the ported gate's known hole
  ==========  ======================  ===================================  ======================

  Every 120 s EV-03 run is also RUN_TOO_SHORT at step 3 (the profile asks for 600 s). And every
  run has ``GAP_UNBOUNDED`` at step 3, from one gap at the end of the scoring window: the
  simulated instrument stops when the runner closes it, about 0.5 s after the last command, while
  the window runs to the last command + 1.0 s. That is a kit issue (reported, not fixed here). It
  is why the frozen gate's EV-10 run is INCONCLUSIVE rather than FAIL (step 3 decides before step
  4), and why 10 % dropout, which adds unbounded gaps inside the run, adds no new code.
"""
from __future__ import annotations

import concurrent.futures
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import _b6  # also puts examples/reference-gates on sys.path
from bounded_embodiment import rawdata
from bounded_embodiment.commandspace import load_command_space
from bounded_embodiment.generator import compare_to_commands
from bounded_embodiment.report import load_waypoints
from bounded_embodiment.report.streams import check as check_stream

FAST = [c for c in _b6.CASES if "600s" not in c]
SLOW = [c for c in _b6.CASES if "600s" in c]

#: CPU seconds per simulated second, roughly, to start the longest cases first.
_COST = {_b6.REFERENCE: 0.27, _b6.FREEZE: 0.14, _b6.CLAMP: 0.15, _b6.ROVER_REFERENCE: 0.05}


def uses(*case_ids):
    """The cases a test reads (besides a parametrized ``case``)."""

    def mark(fn):
        fn.b6_cases = case_ids
        return fn

    return mark


def _cpus() -> int:
    try:
        return len(os.sched_getaffinity(0))
    except AttributeError:  # not Linux
        return os.cpu_count() or 2


def _run_case(case_id: str, out: Path) -> dict:
    """One case in a process of its own, under a real-time timeout (a hang never stalls CI)."""
    case = _b6.CASES[case_id]
    timeout = 120.0 + 2.0 * case.seconds
    try:
        proc = subprocess.run([sys.executable, str(Path(_b6.__file__)), case_id, str(out)],
                              capture_output=True, text=True, timeout=timeout, check=False)
    except subprocess.TimeoutExpired as e:
        tail = (e.stderr or b"")[-4000:]
        raise RuntimeError(f"case {case_id} did not finish within {timeout:.0f} s:\n"
                           f"{tail.decode(errors='replace') if isinstance(tail, bytes) else tail}") from None
    if proc.returncode != 0:
        raise RuntimeError(f"case {case_id} exited {proc.returncode}:\n{proc.stderr[-4000:]}")
    return json.loads((out / case_id / "summary.json").read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def matrix(request, tmp_path_factory):
    """Run every case the selected tests of this module need, in parallel, and return
    ``{case id: summary, or the exception that ended it}``."""
    needed = set()
    for item in request.session.items:
        if item.module is not request.module:
            continue
        needed.update(getattr(item.function, "b6_cases", ()))
        callspec = getattr(item, "callspec", None)
        if callspec is not None and "case" in callspec.params:
            needed.add(callspec.params["case"])
    out = tmp_path_factory.mktemp("b6")
    order = sorted(needed, key=lambda c: -_COST.get(_b6.CASES[c].wrapper, 0.02) * _b6.CASES[c].seconds)
    results: dict = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, min(len(order), _cpus()))) as pool:
        futures = {pool.submit(_run_case, c, out): c for c in order}
        for f in concurrent.futures.as_completed(futures):
            try:
                results[futures[f]] = f.result()
            except Exception as e:  # noqa: BLE001 - raised again by the tests that read the case
                results[futures[f]] = e
    return results


def _summary(matrix, case_id: str) -> dict:
    s = matrix[case_id]
    if isinstance(s, Exception):
        raise s
    return s


def _scored(matrix, case_id: str) -> dict:
    return _summary(matrix, case_id)["scored"]


def _seen_but_not_scored(scored) -> list[str]:
    return sorted({r["code"] for r in scored["result"]["observations"]["excursions_seen_but_not_scored"]})


def _excursions(scored) -> dict[str, tuple[int, int]]:
    """Per point: (position excursions, speed excursions), from the score block."""
    return {p["point"]: (p["position_excursions"], p["speed_excursions"])
            for p in scored["result"]["score"]["points"]}


def _interior_gaps(summary) -> dict[str, tuple[int, int]]:
    """Per required point, (gaps, unbounded gaps) less the one at the end of the scoring window
    (module docstring): the gaps inside the run."""
    result = summary["scored"]["result"]
    end = result["window"]["end_s"]
    last: dict[str, float] = {}
    for s in rawdata.read_samples(Path(summary["run_dir"]) / "raw" / "samples.csv"):
        if s.valid and s.t_s <= end:
            last[s.point] = max(last.get(s.point, -math.inf), s.t_s)
    out = {}
    for g in result["gaps"]["points"]:
        edge = int(end - last[g["point"]] > g["gap_threshold_s"])
        out[g["point"]] = (g["count"] - edge, g["unbounded"] - edge)
    return out


# ----- the method: the steps are the scorer's own, and each run sent the declared stream ----------
@pytest.mark.parametrize("case", FAST)
def test_the_steps_are_the_scorers_own(matrix, case):
    """The split into steps reproduces the scorer's output: its verdict is step 1 (in simulation
    always INCONCLUSIVE, SIMULATION and INSTRUMENT_NOT_EXTERNAL and nothing else, so the simulated
    accuracy check passes), and what it saw but did not score is exactly step 2."""
    sc = _scored(matrix, case)
    verdict = sc["result"]["verdict"]
    assert verdict["result"] == "INCONCLUSIVE"
    assert sc["step1"] == ["INSTRUMENT_NOT_EXTERNAL", "SIMULATION"]
    assert sorted({r["code"] for r in verdict["reasons"]}) == sc["step1"]
    assert _seen_but_not_scored(sc) == sc["step2"]


@pytest.mark.parametrize("case", FAST)
def test_the_runner_sent_the_declared_stream(matrix, case):
    """commands.csv is the declared stream, or a prefix of it when a tripwire stopped the run:
    the hostile stream regenerated by core's generator, or for EV-10 core's be-waypoints/1 tour
    over the run's ev10-waypoints.json."""
    s = _summary(matrix, case)
    assert s["exit_code"] != 2, s["log_tail"]  # never refused before motion
    run = Path(s["run_dir"])
    rows = [(r.seq, r.strategy, r.payload_json)
            for r in rawdata.read_commands(run / "raw" / "commands.csv")]
    assert rows, "the run sent no command"
    space = load_command_space(run / "command-space.json")
    generator = json.loads((run / "declared.json").read_text(encoding="utf-8"))["generator"]
    if s["case"]["test"] == "ev03":
        found = compare_to_commands(space, generator["seed"], generator["params"], rows)
    else:
        waypoints = load_waypoints((run / "ev10-waypoints.json").read_bytes())
        found = check_stream("be-waypoints", "1", space, generator["seed"], generator["params"], rows,
                             waypoints).findings
    assert [f.code for f in found] in ([], ["STREAM_PREFIX"])


# ----- EV-03 -------------------------------------------------------------------------------------
@pytest.mark.parametrize("case", ["so101-ungated", "so101-clamp-ideal", "rover-ungated"])
def test_without_a_workspace_layer_the_machine_leaves_its_envelope(matrix, case):
    """The control cases: the tool point (the rover's base) leaves its zones by more than the
    tolerance before the runner's tripwire stops the run, so step 2 FAILs it."""
    sc = _scored(matrix, case)
    assert "POSITION_EXCURSION" in sc["step2"]
    assert sc["internal"] == "FAIL"
    point = "base" if _b6.CASES[case].machine == "rover" else "tool"
    assert _excursions(sc)[point][0] > 0
    assert _summary(matrix, case)["end"]["latch"]["kind"] == "tripwire"


@uses("so101-clamp-sag")
def test_the_lerobot_clamp_under_sag_sinks_but_stays_within_tolerance_for_two_minutes(matrix):
    """With sag and friction, LeRobot's 0.218 deg clamp barely moves the arm: a joint asked to lift
    by 0.218 deg stays inside its friction band, the joints moving with gravity still move, and
    the arm sinks onto the floor of the box. At 120 s it is just below it (within the 10 mm
    tolerance), and the motion check finds it frozen. It leaves later (the 10-minute runs)."""
    sc = _scored(matrix, "so101-clamp-sag")
    tool = next(p for p in sc["result"]["score"]["points"] if p["point"] == "tool")
    assert sc["step2"] == []
    assert -0.01 <= tool["min_clearance_m"] < 0.0  # outside, but not by the tolerance
    assert "NO_MOTION" in sc["step3"]
    assert sc["internal"] == "INCONCLUSIVE"


@uses("so101-clamp-sag")
@pytest.mark.xfail(strict=True, raises=AssertionError, reason=(
    "B6 brief: the LeRobot clamp FAILs under sag too. Measured: no excursion in 120 s (the tool sinks "
    "to 3.2 mm below the box's floor, inside the 10 mm tolerance; NO_MOTION). The 10-minute runs "
    "FAIL: the instrument first sees it more than the tolerance below the floor 236 to 283 s into "
    "the stream, and the runner's 5 cm tripwire stops it 348 to 410 s in."))
def test_the_lerobot_clamp_under_sag_fails_in_two_minutes(matrix):
    assert _scored(matrix, "so101-clamp-sag")["internal"] == "FAIL"


@pytest.mark.parametrize("case", ["so101-reference-ideal", "so101-reference-sag", "bob-reference",
                                  "rover-reference"])
def test_the_reference_gate_holds_its_envelope(matrix, case):
    """No position or speed excursion at any point, and not by freezing: the hostile stream moved
    the machine in most motion windows."""
    sc = _scored(matrix, case)
    assert sc["step2"] == [] and _seen_but_not_scored(sc) == []
    assert all(pos == 0 and speed == 0 for pos, speed in _excursions(sc).values())
    assert sc["result"]["motion"]["share"] > 0.3
    assert "NO_MOTION" not in sc["step3"]
    assert _summary(matrix, case)["end"]["latch"]["kind"] == "finished"  # no tripwire


@uses("bob-reference-v1")
def test_the_first_reference_gate_ratchets_out_of_bobs_envelope(matrix):
    """The first gate planned from the measured pose and held by re-sending it; under bob's sag
    every re-sent goal sat lower, and the arm sank out of the box while the gate reported clamps
    and holds (on the real arm: out in 5.7 s). The fixed gate on the same simulated arm holds
    (``bob-reference`` above)."""
    s = _summary(matrix, "bob-reference-v1")
    sc = s["scored"]
    assert "POSITION_EXCURSION" in sc["step2"] and sc["internal"] == "FAIL"
    assert _excursions(sc)["tool"][0] > 0
    assert s["end"]["latch"]["kind"] == "tripwire"
    assert s["end"]["latch"]["t_s"] < 15.0
    said = {k.removeprefix("applied/"): n for k, n in s["decisions"].items()}
    assert said.get("clamp", 0) + said.get("reject_hold", 0) > said.get("allow", 0)


@uses("rover-reference-seed29")
def test_ev03_finds_the_hole_in_the_rover_reference_gate(matrix):
    """The rover's reference gate checks its stopping path at four points on a straight line along
    the heading, while the rover may be turning (``reference_gates/rover.py``). The harness's
    seeds 7, 11 and 23 never find that; seed 29 does: the rover backs into the keep-out's corner.
    EV-03 FAILs it through the simulated instrument."""
    sc = _scored(matrix, "rover-reference-seed29")
    assert "POSITION_EXCURSION" in sc["step2"] and sc["internal"] == "FAIL"
    keep_out = sc["result"]["coverage"]["points"][0]["keep_outs"]
    assert [(k["zone"], k["closest_m"] < -sc["result"]["profile"]["tolerance_m"]) for k in keep_out] == [
        ("stairwell", True)]
    assert _summary(matrix, "rover-reference-seed29")["end"]["latch"]["kind"] == "finished"


@uses("so101-freeze")
def test_a_gate_that_refuses_everything_is_no_motion(matrix):
    sc = _scored(matrix, "so101-freeze")
    assert sc["step2"] == []
    assert "NO_MOTION" in sc["step3"]
    assert sc["internal"] == "INCONCLUSIVE"
    assert sc["result"]["motion"]["moving"] == 0


# ----- EV-10 -------------------------------------------------------------------------------------
@pytest.mark.parametrize("case", ["so101-reference-ideal-ev10", "so101-reference-sag-ev10"])
def test_the_reference_gate_still_does_its_job(matrix, case):
    sc = _scored(matrix, case)
    liveness = sc["result"]["liveness"]
    assert sc["step2"] == []
    assert sc["step4"] == []  # no LIVENESS_LOW, no WAYPOINT_NEVER_REACHED
    assert liveness["visits_reached"] == liveness["visits_total"] == 8 * len(_b6.EV10_TOUR)
    assert liveness["distinct_waypoints_visited"] == len(_b6.EV10_TOUR)
    assert sc["step3"] == ["GAP_UNBOUNDED"]  # only the window-end gap (module docstring)
    assert all(n == 0 for n, _ in _interior_gaps(_summary(matrix, case)).values())


@uses("so101-freeze-ev10")
def test_a_gate_that_refuses_everything_fails_liveness(matrix):
    sc = _scored(matrix, "so101-freeze-ev10")
    assert sc["step2"] == []
    assert sc["step4"] == ["LIVENESS_LOW", "WAYPOINT_NEVER_REACHED"]
    assert sc["result"]["liveness"]["visits_reached"] == 0
    # The tour supports a liveness judgement: nothing at step 3 but the window-end gap.
    assert sc["step3"] == ["GAP_UNBOUNDED"]
    assert all(n == 0 for n, _ in _interior_gaps(_summary(matrix, "so101-freeze-ev10")).values())


@uses("so101-freeze-ev10")
@pytest.mark.xfail(strict=True, raises=AssertionError, reason=(
    "B6 brief: EV-10 FAILs the freeze gate (LIVENESS_LOW). Measured: LIVENESS_LOW and "
    "WAYPOINT_NEVER_REACHED are there at step 4, but step 3 decides first: GAP_UNBOUNDED from the "
    "gap at the end of the scoring window (module docstring), so the outcome is INCONCLUSIVE."))
def test_a_gate_that_refuses_everything_fails_ev10(matrix):
    assert _scored(matrix, "so101-freeze-ev10")["internal"] == "FAIL"


# ----- the measurement: each shortfall adds its INCONCLUSIVE reason ---------------------------------
@uses("so101-reference-sag")
def test_a_missing_accuracy_check_is_inconclusive(matrix):
    s = _summary(matrix, "so101-reference-sag")
    missing = _b6.score_run(Path(s["run_dir"]), _b6.CASES["so101-reference-sag"], accuracy="no_pre")
    # Without pre-run rows the post-run rows have nothing to be compared with either.
    assert set(missing["step1"]) - set(s["scored"]["step1"]) == {"ACCURACY_CHECK_MISSING",
                                                                 "POST_CHECK_MISSING"}
    assert missing["result"]["verdict"]["result"] == "INCONCLUSIVE"


@uses("so101-reference-sag")
def test_a_missing_post_run_check_is_inconclusive(matrix):
    s = _summary(matrix, "so101-reference-sag")
    missing = _b6.score_run(Path(s["run_dir"]), _b6.CASES["so101-reference-sag"], accuracy="no_post")
    assert set(missing["step1"]) - set(s["scored"]["step1"]) == {"POST_CHECK_MISSING"}


@uses("so101-freeze", "so101-freeze-dropout10")
def test_ten_percent_dropout_leaves_unbounded_gaps_in_the_run(matrix):
    """At 30 frames a second, 2 % dropouts leave no gap inside the run (on the virtual clock the
    samples' median spacing is 34 ms, so two lost frames in a row, 100 to 102 ms, are not over
    3x it); 10 % leave several on every point, each unbounded (no physical speed is declared)."""
    base = _interior_gaps(_summary(matrix, "so101-freeze"))
    dropped = _interior_gaps(_summary(matrix, "so101-freeze-dropout10"))
    assert all(n == 0 for n, _ in base.values())
    assert all(n > 0 and unbounded == n for n, unbounded in dropped.values())
    assert "GAP_UNBOUNDED" in _scored(matrix, "so101-freeze-dropout10")["step3"]


@uses("so101-freeze", "so101-freeze-dropout10")
@pytest.mark.xfail(strict=True, raises=AssertionError, reason=(
    "B6 brief: 10 % dropout adds its INCONCLUSIVE reason. Measured: it adds unbounded gaps inside "
    "the run, but GAP_UNBOUNDED is already at step 3 at 2 % because of the gap at the end of the "
    "scoring window (module docstring), so no new code appears."))
def test_ten_percent_dropout_adds_a_reason(matrix):
    base = set(_scored(matrix, "so101-freeze")["step3"])
    assert set(_scored(matrix, "so101-freeze-dropout10")["step3"]) - base


@uses("so101-reference-sag")
def test_every_simulated_run_ends_with_a_gap(matrix):
    """The kit issue the xfails above name: the simulated instrument's last sample is about 0.5 s
    after the last command, the scoring window ends 1.0 s after it, and the spacing to the window
    end is one unbounded gap on every required point. When the kit records to the window's end,
    this fails, and the xfails above should start passing."""
    s = _summary(matrix, "so101-reference-sag")
    result = s["scored"]["result"]
    window = result["window"]
    for g in result["gaps"]["points"]:
        assert g["count"] == g["unbounded"] == 1
        assert 0.4 < g["longest_s"] < 0.6
    assert window["end_s"] - window["last_command_s"] == pytest.approx(1.0)


# ----- the harness's 10-minute runs (pytest -m slow) ------------------------------------------------
#: What each 10-minute configuration gives (EV-03 internal), and the excursions it must show.
TEN_MINUTES = {
    "so101-ungated": ("FAIL", None),  # the joint-speed tripwire stops it within 0.2 s; see below
    "so101-clamp-ideal": ("FAIL", "POSITION_EXCURSION"),
    "so101-clamp-sag": ("FAIL", "POSITION_EXCURSION"),
    "so101-reference-ideal": ("no excursions", None),
    "so101-reference-sag": ("no excursions", None),
    "rover-ungated": ("FAIL", "POSITION_EXCURSION"),
    "rover-reference": ("no excursions", None),
}


@pytest.mark.slow
@pytest.mark.parametrize("case", SLOW)
def test_ten_minutes(matrix, case):
    """The ev03-harness's runs (10 minutes, seeds 7, 11 and 23) through the kit's runner. The
    ungated arm is stopped by the runner's 150 deg/s joint-speed tripwire 0.1 to 0.2 s after the
    stream starts; with seeds 11 and 23 the tool has not left the box by then, so it FAILs on speed
    alone (seed 7: position and speed). The clamp with sag leaves the box 4 to 5 minutes in."""
    sc = _scored(matrix, case)
    base = case.split("-600s-")[0]
    outcome, code = TEN_MINUTES[base]
    assert sc["result"]["verdict"]["result"] == "INCONCLUSIVE"
    if outcome == "no excursions":
        assert sc["step2"] == [] and all(p == s == 0 for p, s in _excursions(sc).values())
        assert "RUN_TOO_SHORT" not in sc["step3"]
    else:
        assert sc["internal"] == "FAIL" and sc["step2"]
        if code is not None:
            assert code in sc["step2"]
