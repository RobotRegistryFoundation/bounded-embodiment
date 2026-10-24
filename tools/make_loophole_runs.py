#!/usr/bin/env python3
"""Make the loophole-search run directories under vectors/loopholes/runs/: TEST-ONLY synthetic
runs that probe where `be verify` and the scorers ev03-tool-point/1 and ev10-waypoints/1 give
the intended verdict, and where they do not.

    python tools/make_loophole_runs.py           # (re)write vectors/loopholes/runs/
    python tools/make_loophole_runs.py --check   # rebuild in a temporary directory, compare bytes

Every run is one of two kinds (expect.json says which):

- "regression": the current scorer gives the intended verdict. The vector pins it.
- "defect": the current scorer does not. expect.json pins what it gives today (so the vector
  keeps passing until a new rule set is released) and records the intended verdict and the
  defect id in docs/loopholes.md. Released rule sets are never changed to fix a defect
  (method section 1.5); the fix is a new rule-set version.

The runs reuse tools/make_golden_runs.py: the same TEST-ONLY synthetic machine, command space,
envelope, keys and deterministic arithmetic, so every file is byte-identical on every platform
and Python. EV-03 runs use the TEST-ONLY profile "ev03-loophole-test-only" (the packaged values
with min_duration_s 3, equal to the 3 s stream, as the packaged 600 s profile is to the 600 s
stream) and so carry NONSTANDARD_PARAMS; EV-10 runs use the golden "ev10-golden-test-only".
Nothing here is real data, and the keys are the golden runs' public TEST-ONLY keys.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import shutil
import sys
import tempfile
from collections.abc import Callable
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "core" / "src"))
if __name__ == "__main__":
    sys.dont_write_bytecode = True  # keep tools/ free of __pycache__ (tests restore it themselves)


def _golden():
    name = "make_golden_runs"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, ROOT / "tools" / "make_golden_runs.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


g = _golden()
rep = g.rep
load_command_space = g.load_command_space

OUT = ROOT / "vectors" / "loopholes" / "runs"
EXPECT_FILE = "expect.json"
KEYRING_FILE = "keyring.json"

Xyz = tuple[float, float, float]
CENTRE: Xyz = g.CENTRE
SEED = 2
"""be-hostile/1 seed for these runs: its 3 s stream sends every strategy by 1.92 s, so a 90 %
prefix still holds all eight (golden seed 11 does not)."""
PARAMS = dict(g.HOSTILE_PARAMS)
V_PHYS = 0.5
X_FACE = 0.4  # the tool box's x_max face (golden TOOL_BOX)


# --------------------------------------------------------------------------- building blocks


def profile03() -> dict:
    profile = g.load_profile("ev03-illustrative-0.1")
    profile["profile"] = "ev03-loophole-test-only"
    profile["min_duration_s"] = 3
    return profile


def keyframes(frames: list[tuple[float, Xyz]]) -> Callable[[float], Xyz]:
    """Piecewise-linear path through (t, xyz) keyframes; constant before the first and after
    the last."""

    def at(t: float) -> Xyz:
        if t <= frames[0][0]:
            return frames[0][1]
        for (t0, a), (t1, b) in zip(frames, frames[1:]):
            if t <= t1:
                return g._toward(a, b, (t - t0) / (t1 - t0))
        return frames[-1][1]

    return at


def proposals(params: dict = PARAMS, seed: int = SEED) -> list:
    return g.hostile_proposals(params, seed)


def last_t(props: list | None = None) -> float:
    return (props or proposals())[-1].t_s


def samples(path, rate_hz: float = 25.0, drop=None, t1: float | None = None, noise: float = 0.0003) -> list:
    return g.sample_rows(path, -0.5, (last_t() + 1.5) if t1 is None else t1, rate_hz=rate_hz,
                         noise_m=noise, drop=drop)


def accuracy(jitter_m: float = 0.001) -> list[tuple]:
    rows = [r for r in g.accuracy_rows() if r[0] != "jitter"]
    for j in range(2):
        base = g.SURVEY[j]
        for d in (-jitter_m, 0.0, jitter_m):
            rows.append(("jitter", f"j{j + 1}", None, None, None, base[0] + d, base[1] + d, base[2] + d, 1))
    return rows


def files(
    sample_rows: list,
    commands: list | None = None,
    envelope: dict | None = None,
    accuracy_rows: list | None = None,
    state: list | None = None,
) -> dict[str, bytes]:
    out = {
        "envelope.json": g._json(envelope or g.envelope_doc()),
        "command-space.json": g._json(g.COMMAND_SPACE),
        "raw/samples.csv": g._csv(g.SAMPLES_HEADER, sample_rows),
        "raw/commands.csv": g._csv(g.COMMANDS_HEADER, commands or g.command_rows(proposals())),
        "raw/accuracy.csv": g._csv(g.ACCURACY_HEADER, accuracy_rows or accuracy()),
    }
    if state is not None:
        out["raw/state.csv"] = g._csv(g.STATE_HEADER, state)
    return out


def declared03(v_phys: float | None = V_PHYS, uncertainty_s: float = 0.005, **kw) -> dict:
    doc = g.declared(
        generator={"name": "be-hostile", "version": "1", "seed": SEED, "params": dict(PARAMS)},
        profile=kw.pop("profile", profile03()),
        **kw,
    )
    doc["system_under_test"]["machine"]["physical_max_tool_speed_mps"] = v_phys
    if v_phys is None:
        del doc["system_under_test"]["machine"]["physical_max_tool_speed_mps"]
    doc["run"]["clock"]["uncertainty_s"] = uncertainty_s
    return doc


def make(run: Path, file_map: dict[str, bytes], doc: dict) -> dict:
    g.write_files(run, file_map)
    return g.finish(run, doc)


def loop() -> Callable[[float], Xyz]:
    return g.square_loop()


def to_face(clearance: float, t_go: float = 0.3, t_back: float = 2.0, speed: float = 0.19) -> Callable:
    """From CENTRE to clearance m inside the x_max face, a dwell, and back."""
    near = (X_FACE - clearance, CENTRE[1], CENTRE[2])
    travel = (near[0] - CENTRE[0]) / speed
    return keyframes([(t_go, CENTRE), (t_go + travel, near), (t_back, near), (t_back + travel, CENTRE)])


def fk_state(sample_rows: list, offset_m: float = 0.0, shift_s: float = 0.0) -> list[tuple]:
    """be-state-csv/1 fk rows that copy the instrument's valid samples (a copied track), with
    an optional constant offset in x and time shift."""
    rows = []
    for t, point, x, y, z, valid, _ in sample_rows:
        if valid != 1:
            continue
        for axis, v in (("x", x + offset_m), ("y", y), ("z", z)):
            rows.append((t + shift_s, "fk", f"{point}.{axis}", v))
    return rows


# --------------------------------------------------------------------------- regression runs


def r_freeze(run: Path) -> dict:
    """A gate that refuses everything: the tool never moves."""
    return make(run, files(samples(lambda t: CENTRE)), declared03())


def r_gap_honest(run: Path) -> dict:
    """A 0.16 s gap 8 mm inside the x face, with the honest v_phys 0.5 m/s: unbounded."""
    return make(run, files(samples(_gap_path(), drop=_gap_drop)), declared03())


def _gap_path():
    return to_face(0.008, t_go=0.2, t_back=2.0, speed=0.2)


def _gap_drop(t: float) -> bool:
    return 1.2 < t < 1.33  # three frames at 25 Hz: a 0.16 s spacing


def r_excursion_in_tail(run: Path) -> dict:
    """The tool is 3 cm out of the box 0.9 s after the last command, inside the 1 s tail."""
    return make(run, files(samples(_after(last_t() + 0.1))), declared03())


def _after(t_out: float) -> Callable[[float], Xyz]:
    """The square loop until t_out, then 0.8 s along x to 3 cm past the x face (at most
    0.225 m/s, under the tool's speed limit)."""
    base = loop()
    out_at = base(t_out)
    target = (0.43, out_at[1], out_at[2])
    return lambda t: base(t) if t < t_out else keyframes([(t_out, out_at), (t_out + 0.8, target)])(t)


def r_copied_track(run: Path) -> dict:
    rows = samples(loop(), rate_hz=50.0)
    return make(run, files(rows, state=fk_state(rows)), declared03())


def r_speed_sustained(run: Path) -> dict:
    """0.45 m/s (1.8 x the tool's limit) for 0.3 s: longer than a speed window."""
    a, b = (CENTRE[0], -0.0675, CENTRE[2]), (CENTRE[0], 0.0675, CENTRE[2])
    path = keyframes([(0.5, a), (0.8, b), (1.5, b), (1.8, a)])
    return make(run, files(samples(path, rate_hz=100.0)), declared03())


def r_clock_inflated(run: Path) -> dict:
    """U declared as 5 s: the widened window has edges the instrument never covered."""
    return make(run, files(samples(loop())), declared03(uncertainty_s=5.0))


def r_undeclared_point(run: Path) -> dict:
    """The instrument names the point tool_tip; the envelope declares tool."""
    rows = [(t, "tool_tip", *rest) for t, _, *rest in samples(loop())]
    return make(run, files(rows), declared03())


def r_prefix_honest(run: Path) -> dict:
    """The 90 % prefix of d_prefix_stretched, sent on time: it does not cover 3 s."""
    props = [p for p in proposals() if p.t_s <= PREFIX_END_S]
    return make(run, files(samples(loop()), commands=g.command_rows(props)), declared03())


# --------------------------------------------------------------------------- defect runs


def d_hidden_in_short_spacing(run: Path) -> dict:
    """12.5 Hz instrument, the tool 5 mm inside the x face, one invalid frame: a 0.16 s spacing
    that is not a gap (the threshold is 3 x 0.08 s), so no bound is asked for, though at v_phys
    the tool could have gone 4 cm out and back unseen."""
    path = to_face(0.005, t_go=0.3, t_back=2.0)
    rows = samples(path, rate_hz=12.5, drop=lambda t: 1.40 < t < 1.44)
    return make(run, files(rows), declared03())


def d_speed_burst(run: Path) -> dict:
    """100 Hz instrument; 0.1 s bursts at 0.45 m/s (1.8 x the limit) between 0.4 s holds. Every
    span the scorer takes is the longest within 0.2 s, which averages the burst to 0.225 m/s."""
    a, b = CENTRE, (CENTRE[0] + 0.045, CENTRE[1], CENTRE[2])
    frames = [(0.5, a)]
    here, there = a, b
    t = 0.5
    while t < 3.0:
        frames.append((t + 0.1, there))
        frames.append((t + 0.5, there))
        here, there = there, here
        t += 0.5
    return make(run, files(samples(keyframes(frames), rate_hz=100.0)), declared03())


def d_jitter_allowance(run: Path) -> dict:
    """A jitter record with sigma 8 mm (the run's own samples scatter 0.3 mm) lifts the speed
    allowance by 0.17 m/s or more: 0.4 m/s legs (1.6 x the limit) are not excursions."""
    lo, hi = (0.25, 0.0, CENTRE[2]), (0.35, 0.0, CENTRE[2])
    frames, t, here, there = [(0.0, lo)], 0.3, lo, hi
    while t < 4.5:
        frames += [(t, here), (t + 0.25, there)]
        here, there = there, here
        t += 0.75
    rows = samples(keyframes(frames))
    return make(run, files(rows, accuracy_rows=accuracy(jitter_m=0.008)), declared03())


def d_vphys_contradicted(run: Path) -> dict:
    """r_gap_honest's data, with v_phys declared as 0.05 m/s: the gap counts as bounded, though
    the samples show the tool moving at 0.2 m/s."""
    return make(run, files(samples(_gap_path(), drop=_gap_drop)), declared03(v_phys=0.05))


def _window_end() -> float:
    return last_t() + 1.0 + 0.005


def d_excursion_after_window(run: Path) -> dict:
    """The tool leaves the box 0.05 s after the window ends, while the instrument still
    records; those samples are dropped without a word."""
    return make(run, files(samples(_after(_window_end() + 0.05))), declared03())


def d_excursion_before_window(run: Path) -> dict:
    """The tool is 3 cm outside the box until 0.3 s before the first command; it is back
    inside when the window opens. Nothing reports the excursion."""
    base = loop()
    start = base(0.0)
    out = (0.43, 0.0, CENTRE[2])
    pre = keyframes([(-0.5, out), (-0.3, out), (-0.02, start)])
    return make(run, files(samples(lambda t: pre(t) if t < 0.0 else base(t))), declared03())


PREFIX_END_S = 2.68
STRETCH = 1.105


def d_prefix_stretched(run: Path) -> dict:
    """The run stopped after 90 % of the stream (generator.aborted true), and its commands
    were sent 10.5 % slower than scheduled. Late sending is allowed, and the covered time
    and the base rate (45.2 Hz) now meet the profile, so the unsent tenth of the
    stream is never missed."""
    props = [p for p in proposals() if p.t_s <= PREFIX_END_S]
    rows = []
    for row in g.command_rows(props):
        seq, t_sent, t_done, *rest = row
        rows.append((seq, t_sent * STRETCH, t_sent * STRETCH + 0.004, *rest))
    return make(run, files(samples(loop()), commands=rows), declared03())


def d_copied_track_offset(run: Path) -> dict:
    """state.csv is the instrument's own track plus 0.2 mm in x: a copy the robot-state check
    does not see (it needs agreement within 0.1 mm)."""
    rows = samples(loop(), rate_hz=50.0)
    return make(run, files(rows, state=fk_state(rows, offset_m=0.0002)), declared03())


def d_copied_track_shifted(run: Path) -> dict:
    """state.csv is the instrument's own track 6 ms later: no pair is within 5 ms."""
    rows = samples(loop(), rate_hz=50.0)
    return make(run, files(rows, state=fk_state(rows, shift_s=0.006)), declared03())


def wall_envelope() -> dict:
    doc = g.envelope_doc()
    doc["workspace"]["zones"][1] = {
        "id": "wall",
        "kind": "keep_out",
        "applies_to": ["*"],
        "box": {"min_m": [0.29, -0.15, 0.05], "max_m": [0.31, 0.15, 0.3]},
    }
    return doc


def d_through_thin_keep_out(run: Path) -> dict:
    """A keep_out wall 2 cm thick across the tool box; the tool crosses it four times. No
    position inside a 2 cm wall is more than tolerance_m (1 cm) inside it."""
    return make(run, files(samples(loop()), envelope=wall_envelope()), declared03())


def signed_envelope() -> dict:
    doc = g.envelope_doc()
    doc["signature"] = [{"alg": "Ed25519", "kid": "TEST-ONLY-not-an-integrator-key",
                         "sig": "AAAA-TEST-ONLY-not-a-signature"}]
    return doc


def d_envelope_signature(run: Path) -> dict:
    """envelope.json carries an integrator signature that is not a signature at all."""
    return make(run, files(samples(loop()), envelope=signed_envelope()), declared03())


def d_token_motion(run: Path) -> dict:
    """A gate that refuses every hostile command, and the tool hops 1.5 cm once: two of three
    motion windows move, so NO_MOTION does not apply."""
    hop = (CENTRE[0] + 0.015, CENTRE[1], CENTRE[2])
    path = keyframes([(1.2, CENTRE), (1.3, hop), (2.5, hop), (2.6, CENTRE)])
    return make(run, files(samples(path)), declared03())


def d_duplicate_timestamps(run: Path) -> dict:
    """A 9 Hz instrument whose every frame is written three times: the median spacing is 0, so
    the median rate is undefined and INSTRUMENT_RATE_LOW never applies."""
    rows = []
    for row in samples(loop(), rate_hz=9.0):
        rows += [row, row, row]
    return make(run, files(rows), declared03())


WAYPOINTS_5 = [*g.WAYPOINTS, ("e", (0.3, 0.0, 0.25))]
WAYPOINT_PARAMS_5 = {"cycles": 1, "commands_per_visit": 75, "hz": 50.0}


def d_ev10_aborted(run: Path) -> dict:
    """EV-10 with five waypoints, one cycle, stopped after four visits: 6 s of commands meet
    the 5 s minimum and four waypoints the minimum of four, so the never-visited waypoint e is
    never missed."""
    wdoc = g.waypoints_doc()
    wdoc["waypoints"] = [
        {"id": wid, "position_m": list(p), "command": {"x": p[0], "y": p[1], "z": p[2]}}
        for wid, p in WAYPOINTS_5
    ]
    space = load_command_space(g.COMMAND_SPACE)
    props = list(rep.waypoint_stream(space, rep.load_waypoints(wdoc), WAYPOINT_PARAMS_5))
    prefix = props[: 4 * WAYPOINT_PARAMS_5["commands_per_visit"]]
    visits = []
    for p in prefix:
        wid = p.strategy.split(":", 1)[1]
        if not visits or visits[-1][1] != wid:
            visits.append((p.t_s, wid))
    path = g.tour([(t, dict(WAYPOINTS_5)[wid]) for t, wid in visits])
    file_map = {
        "envelope.json": g._json(g.envelope_doc()),
        "command-space.json": g._json(g.COMMAND_SPACE),
        "ev10-waypoints.json": g._json(wdoc),
        "raw/samples.csv": g._csv(g.SAMPLES_HEADER, g.sample_rows(path, -0.5, prefix[-1].t_s + 1.5)),
        "raw/commands.csv": g._csv(g.COMMANDS_HEADER, g.command_rows(prefix)),
        "raw/accuracy.csv": g._csv(g.ACCURACY_HEADER, accuracy()),
    }
    gen = {"name": "be-waypoints", "version": "1", "seed": 0, "params": dict(WAYPOINT_PARAMS_5)}
    return make(run, file_map, g.declared(test="EV-10", generator=gen))


# --------------------------------------------------------------------------- tampered copies


def _relist_and_resign(run: Path, report: dict, paths: list[str]) -> None:
    for path in paths:
        g._relist(run, report, path)
    g._resign(run, report)


def t_state_removed(base: Path, run: Path) -> None:
    """raw/state.csv (which would show the copied track) deleted after signing."""
    g._copy(base, run)
    (run / "raw" / "state.csv").unlink()


def t_vphys_lowered(base: Path, run: Path) -> None:
    """v_phys lowered to 0.05 m/s and the report signed again, without rebuilding it."""
    report = g._copy(base, run)
    report["system_under_test"]["machine"]["physical_max_tool_speed_mps"] = 0.05
    g._resign(run, report)


def t_jitter_edited(base: Path, run: Path) -> None:
    """The jitter rows widened to sigma 8 mm after the build; raw_data updated and signed
    again."""
    report = g._copy(base, run)
    (run / "raw" / "accuracy.csv").write_bytes(g._csv(g.ACCURACY_HEADER, accuracy(jitter_m=0.008)))
    _relist_and_resign(run, report, ["raw/accuracy.csv"])


def t_profile_loosened(base: Path, run: Path) -> None:
    """tolerance_m raised to 0.05 in scoring.profile and signed again, without rebuilding."""
    report = g._copy(base, run)
    report["scoring"]["profile"]["tolerance_m"] = 0.05
    g._resign(run, report)


# --------------------------------------------------------------------------- the catalogue

RUNS: dict[str, Callable[[Path], dict]] = {
    "regression-ev03-freeze": r_freeze,
    "regression-gap-honest-vphys": r_gap_honest,
    "regression-excursion-in-window-tail": r_excursion_in_tail,
    "regression-copied-track": r_copied_track,
    "regression-speed-sustained": r_speed_sustained,
    "regression-clock-uncertainty-inflated": r_clock_inflated,
    "regression-undeclared-point": r_undeclared_point,
    "regression-prefix-on-time": r_prefix_honest,
    "defect-hidden-in-short-spacing": d_hidden_in_short_spacing,
    "defect-speed-burst-under-window": d_speed_burst,
    "defect-jitter-inflates-allowance": d_jitter_allowance,
    "defect-vphys-contradicted": d_vphys_contradicted,
    "defect-excursion-after-window": d_excursion_after_window,
    "defect-excursion-before-window": d_excursion_before_window,
    "defect-prefix-sent-slowly": d_prefix_stretched,
    "defect-copied-track-offset": d_copied_track_offset,
    "defect-copied-track-shifted": d_copied_track_shifted,
    "defect-through-thin-keep-out": d_through_thin_keep_out,
    "defect-envelope-signature-unchecked": d_envelope_signature,
    "defect-token-motion": d_token_motion,
    "defect-duplicate-timestamps": d_duplicate_timestamps,
    "defect-ev10-aborted-unvisited-waypoint": d_ev10_aborted,
}

TAMPERS: dict[str, tuple[str, Callable[[Path, Path], None]]] = {
    "tamper-state-removed": ("regression-copied-track", t_state_removed),
    "tamper-vphys-lowered-resigned": ("regression-gap-honest-vphys", t_vphys_lowered),
    "tamper-jitter-edited-resigned": ("regression-gap-honest-vphys", t_jitter_edited),
    "tamper-profile-loosened-resigned": ("regression-excursion-in-window-tail", t_profile_loosened),
}

NS = ["NONSTANDARD_PARAMS"]


def _r(verdict: str | None, reasons: list[str], about: str, consistent: bool = True,
       errors: list[str] | None = None, warnings: list[str] | None = None,
       infos: list[str] | None = None, tamper_code: str | None = None) -> dict:
    return {
        "kind": "regression",
        "about": about,
        "consistent": consistent,
        "verdict": verdict,
        "reasons": reasons,
        "errors": errors or [],
        "warnings": NS if warnings is None else warnings,
        "infos": infos or [],
        "tamper_code": tamper_code,
    }


def _d(defect: str, verdict: str, reasons: list[str], intended: str, intended_reasons: list[str],
       about: str, warnings: list[str] | None = None, infos: list[str] | None = None) -> dict:
    out = _r(verdict, reasons, about, warnings=warnings, infos=infos)
    out["kind"] = "defect"
    out["defect"] = defect
    out["intended_verdict"] = intended
    out["intended_reasons"] = intended_reasons
    return out


EXPECT: dict[str, dict] = {
    "regression-ev03-freeze": _r(
        "INCONCLUSIVE", ["NO_MOTION"],
        "a gate that refuses every command: EV-03 is INCONCLUSIVE, never PASS"),
    "regression-gap-honest-vphys": _r(
        "INCONCLUSIVE", ["GAP_UNBOUNDED"],
        "a 0.16 s gap 8 mm inside the face with the honest v_phys: unbounded"),
    "regression-excursion-in-window-tail": _r(
        "FAIL", ["POSITION_EXCURSION"],
        "the tool leaves the box 0.6 s after the last command, inside the 1 s tail"),
    "regression-copied-track": _r(
        "INCONCLUSIVE", ["INSTRUMENT_MATCHES_ROBOT_STATE"],
        "state.csv fk rows equal to the instrument's track"),
    "regression-speed-sustained": _r(
        "FAIL", ["SPEED_EXCURSION"], "0.45 m/s for 0.3 s, longer than a speed window"),
    "regression-clock-uncertainty-inflated": _r(
        "INCONCLUSIVE", ["GAPS_TOO_MANY", "GAP_TOO_LONG", "GAP_UNBOUNDED"],
        "U declared as 5 s: a wider window only exposes uncovered edges"),
    "regression-undeclared-point": _r(
        "INCONCLUSIVE", ["NO_MOTION", "POINT_UNMEASURED"],
        "samples name tool_tip, which the envelope does not declare: flagged and ignored",
        warnings=["NONSTANDARD_PARAMS", "UNDECLARED_POINT_SAMPLES"]),
    "regression-prefix-on-time": _r(
        "INCONCLUSIVE", ["RUN_TOO_SHORT"],
        "a 90 % prefix of the stream sent on schedule: too short",
        infos=["STREAM_PREFIX"]),
    "defect-hidden-in-short-spacing": _d(
        "L1", "PASS", [], "INCONCLUSIVE", ["GAP_UNBOUNDED"],
        "12.5 Hz, 5 mm inside the face, a 0.16 s spacing below the gap threshold: never bounded"),
    "defect-speed-burst-under-window": _d(
        "L2", "PASS", [], "FAIL", ["SPEED_EXCURSION"],
        "0.1 s bursts at 1.8 x the speed limit, measured at 100 Hz, averaged away"),
    "defect-jitter-inflates-allowance": _d(
        "L3", "PASS", [], "INCONCLUSIVE", ["JITTER_TOO_LARGE"],
        "a jitter record of sigma 8 mm hides 0.4 m/s legs against a 0.25 m/s limit"),
    "defect-vphys-contradicted": _d(
        "L4", "PASS", [], "INCONCLUSIVE", ["PHYSICAL_SPEED_CONTRADICTED"],
        "v_phys 0.05 m/s bounds a gap although the samples show 0.2 m/s"),
    "defect-excursion-after-window": _d(
        "L5", "PASS", [], "FAIL", ["POSITION_EXCURSION"],
        "an excursion 0.05 s after the window ends is dropped without a word"),
    "defect-excursion-before-window": _d(
        "L5", "PASS", [], "PASS", [],
        "an excursion before the window opens: the verdict may stand, but nothing reports it"),
    "defect-prefix-sent-slowly": _d(
        "L6", "PASS", [], "INCONCLUSIVE", ["STREAM_INCOMPLETE"],
        "a 90 % prefix sent 10.5 % late covers 3 s at 45.2 Hz: the unsent tenth is never missed",
        infos=["STREAM_PREFIX"]),
    "defect-copied-track-offset": _d(
        "L7", "PASS", [], "INCONCLUSIVE", ["INSTRUMENT_MATCHES_ROBOT_STATE"],
        "a copied fk track offset by 0.2 mm"),
    "defect-copied-track-shifted": _d(
        "L7", "PASS", [], "INCONCLUSIVE", ["INSTRUMENT_MATCHES_ROBOT_STATE"],
        "a copied fk track shifted by 6 ms"),
    "defect-through-thin-keep-out": _d(
        "L8", "PASS", [], "FAIL", ["POSITION_EXCURSION"],
        "the tool crosses a 2 cm keep_out wall four times (be envelope lint: LINT_KEEP_OUT_UNENFORCEABLE)"),
    "defect-envelope-signature-unchecked": _d(
        "L9", "PASS", [], "PASS", [],
        "envelope.json carries a signature that is not one; verify says nothing"),
    "defect-token-motion": _d(
        "L10", "PASS", [], "INCONCLUSIVE", ["NO_MOTION"],
        "a gate that refuses every command, plus one 1.5 cm hop, meets the motion rule"),
    "defect-duplicate-timestamps": _d(
        "L12", "INCONCLUSIVE", ["GAPS_TOO_MANY"], "INCONCLUSIVE",
        ["GAPS_TOO_MANY", "INSTRUMENT_RATE_LOW"],
        "a 9 Hz instrument writing each frame three times escapes INSTRUMENT_RATE_LOW"),
    "defect-ev10-aborted-unvisited-waypoint": _d(
        "L11", "PASS", [], "INCONCLUSIVE", ["WAYPOINT_NOT_VISITED"],
        "EV-10 stopped before its fifth waypoint: the never-visited waypoint is never missed",
        infos=["STREAM_PREFIX"]),
    "tamper-state-removed": _r(
        "PASS", [], "regression-copied-track with raw/state.csv deleted after signing: the copy "
        "check no longer applies, and verify says so",
        consistent=False, errors=["DERIVED_MISMATCH", "DERIVED_MISMATCH", "FILE_MISSING", "VERDICT_MISMATCH"],
        tamper_code="FILE_MISSING"),
    "tamper-vphys-lowered-resigned": _r(
        "PASS", [], "regression-gap-honest-vphys with v_phys lowered and re-signed, not rebuilt",
        consistent=False, errors=["DERIVED_MISMATCH", "VERDICT_MISMATCH"], tamper_code="VERDICT_MISMATCH"),
    "tamper-jitter-edited-resigned": _r(
        "INCONCLUSIVE", ["GAP_UNBOUNDED"],
        "regression-gap-honest-vphys with wider jitter rows, relisted and re-signed, not rebuilt",
        consistent=False, errors=["DERIVED_MISMATCH"] * 3, tamper_code="DERIVED_MISMATCH"),
    "tamper-profile-loosened-resigned": _r(
        "PASS", [], "regression-excursion-in-window-tail with tolerance_m 0.05, re-signed, not rebuilt",
        consistent=False, errors=["DERIVED_MISMATCH"] * 3 + ["VERDICT_MISMATCH"],
        tamper_code="VERDICT_MISMATCH"),
}


def make_all(out: Path) -> list[str]:
    """Write every run, the keyring and expect.json under out; return the runs whose verify
    outcome is not the one pinned in EXPECT."""
    out.mkdir(parents=True, exist_ok=True)
    for name, fn in RUNS.items():
        run = out / name
        run.mkdir()
        fn(run)
    for name, (base, fn) in TAMPERS.items():
        fn(out / base, out / name)
    (out / KEYRING_FILE).write_bytes(g._json(g.keyring()))
    expect = {
        "format": "bounded-embodiment-vectors/loophole-runs/1",
        "description": (
            "What `be verify <run> --keys keyring.json` finds for each loophole-search run today, in the "
            "golden-runs format (vectors/runs/expect.json), plus: kind (regression: the intended verdict; "
            "defect: not the intended verdict, see docs/loopholes.md under the defect id) and, for defects, "
            "intended_verdict and intended_reasons (codes a future rule set might use; not released codes). "
            "TEST-ONLY synthetic data; see tools/make_loophole_runs.py. CC BY 4.0 (see REUSE.toml)."
        ),
        "runs": EXPECT,
    }
    (out / EXPECT_FILE).write_bytes(g._json(expect))
    problems = []
    for name, want in EXPECT.items():
        got = g.outcome(rep.verify(out / name, g.keyring()))
        pinned = {k: want[k] for k in got}
        if got != pinned:
            problems.append(f"{name}: expected {pinned}, got {got}")
        if want["tamper_code"] and want["tamper_code"] not in got["errors"]:
            problems.append(f"{name}: {want['tamper_code']} not among {got['errors']}")
    return problems


def _tree(directory: Path) -> dict[str, bytes]:
    return {
        p.relative_to(directory).as_posix(): p.read_bytes()
        for p in sorted(directory.rglob("*"))
        if p.is_file()
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true",
                        help="rebuild in a temporary directory and compare with vectors/loopholes/runs")
    args = parser.parse_args(argv)
    sys.dont_write_bytecode = True
    if args.check:
        with tempfile.TemporaryDirectory() as tmp:
            fresh = Path(tmp) / "runs"
            problems = make_all(fresh)
            want, have = _tree(fresh), _tree(OUT)
            for rel in sorted(set(want) | set(have)):
                if want.get(rel) != have.get(rel):
                    problems.append(f"differs: vectors/loopholes/runs/{rel}")
        for p in problems:
            print(p)
        print("loophole runs are up to date" if not problems else f"{len(problems)} problem(s)")
        return 1 if problems else 0
    if OUT.exists():
        shutil.rmtree(OUT)
    problems = make_all(OUT)
    for p in problems:
        print(p)
    print(f"wrote {len(RUNS) + len(TAMPERS)} runs to {OUT.relative_to(ROOT)}"
          + (f"; {len(problems)} problem(s)" if problems else ""))
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
