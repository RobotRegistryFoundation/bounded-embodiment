"""Derive a report's derived members from its declared members and its run directory.

``report.build`` and ``report.verify`` both call :func:`derive`, so the members build writes are
exactly the ones verify recomputes. Everything here is a pure function of the declared fields
and the bytes :func:`files.scan` read; nothing is fetched and nothing is written.

Finding codes raised here: DECLARED_INVALID, UNEXPECTED_FILE, DUPLICATE_RAW_FILE,
REQUIRED_FILE_MISSING, RAW_TOO_LARGE, COMMAND_SPACE_DIGEST_MISMATCH, SIMULATION_NOT_REHEARSAL,
RUN_TIMES_INVERTED, ACCURACY_CHECK_AFTER_RUN_START, ACCURACY_CHECK_POINTS_MISMATCH,
CALIBRATION_FILE_MISSING, GENERATOR_TEST_MISMATCH, GENERATOR_ROLE_MISMATCH (errors); and those of
the envelope loader (ENVELOPE_*), the command-space loader (COMMAND_SPACE_*), the waypoints
loader (WAYPOINTS_*), the plan loaders (PLAN_*), the raw data readers (RAW_*), the stream check
(STREAM_*, GENERATOR_UNSUPPORTED) and the scorers (SCORING_*), unchanged.
"""

from __future__ import annotations

import datetime
import re
import zlib
from collections.abc import Mapping
from dataclasses import dataclass, field

from .. import canonical, rawdata, scoring
from ..commandspace import CommandSpace, CommandSpaceError, load_command_space
from ..envelope import Envelope, EnvelopeError
from ..envelope import load as load_envelope
from ..findings import Finding
from ..rawdata import RawDataError, Sample
from ..scoring import Declared, ScoringError, ScoringInput
from . import streams
from .files import (
    COMMAND_SPACE_FILE,
    ENVELOPE_FILE,
    MODEL_LOSS_PLAN_FILE,
    STOP_PLAN_FILE,
    WAYPOINTS_FILE,
    RunFile,
    RunFiles,
    classify,
    raw_data_entries,
    show,
)
from .plans import (
    ModelLossPlan,
    PlanError,
    StopPlan,
    check_plan_commands,
    load_model_loss_plan,
    load_stop_plan,
)
from .waypoints import Waypoints, WaypointsError, check_commands, load_waypoints

__all__ = [
    "DERIVED_BLOCKS",
    "GENERATOR_DERIVED",
    "SCORER_BLOCKS",
    "SCORER_FOR_TEST",
    "Derivation",
    "View",
    "derive",
    "expected_blocks",
    "fk_samples_from_state",
    "layout_checks",
    "measurement_source",
    "utc_ns",
    "view_of",
]

SCORER_FOR_TEST = {
    "EV-03": "ev03-tool-point/1",
    "EV-10": "ev10-waypoints/1",
    "EV-02": "ev02-stop/1",
    "EV-05": "ev05-model-loss/1",
}
DEFAULT_PROFILE = {
    "EV-03": "ev03-illustrative-0.1",
    "EV-10": "ev10-illustrative-0.1",
    "EV-02": "ev02-illustrative-0.1",
    "EV-05": "ev05-illustrative-0.1",
}
GENERATORS_FOR_TEST = {
    "EV-03": (streams.HOSTILE, streams.CONTROL),
    "EV-10": (streams.WAYPOINTS,),
    "EV-02": (streams.STOPS,),
    "EV-05": (streams.MODEL_LOSS,),
}
CONTROL_ROLES = ("control", "rehearsal")

SCORER_BLOCKS = (
    "window",
    "achieved_rates",
    "score",
    "gaps",
    "motion",
    "liveness",
    "stops",
    "model_loss",
    "coverage",
    "accuracy",
    "observations",
    "flags",
    "verdict",
)
DERIVED_BLOCKS = ("envelope", *SCORER_BLOCKS, "raw_data", "statements")
"""Top-level derived members, in report order."""
GENERATOR_DERIVED = ("stream_sha256", "commands_compared", "aborted")
"""Derived members inside the generator block."""

_SHARED_BLOCKS = ("envelope", "accuracy", "observations", "flags", "verdict", "raw_data", "statements")
_TEST_BLOCKS = {
    "EV-03": ("window", "achieved_rates", "score", "gaps", "motion", "coverage"),
    "EV-10": ("window", "achieved_rates", "score", "gaps", "liveness", "coverage"),
    "EV-02": ("stops",),
    "EV-05": ("model_loss",),
}


def expected_blocks(test: str | None) -> tuple[str, ...]:
    """The top-level derived members a report of ``test`` has, in report order: every test's
    (envelope, accuracy, observations, flags, verdict, raw_data, statements) and its own (EV-03:
    window, achieved_rates, score, gaps, motion, coverage; EV-10: the same with liveness for
    motion; EV-02: stops; EV-05: model_loss)."""
    own = _TEST_BLOCKS.get(test or "", ())
    return tuple(b for b in DERIVED_BLOCKS if b in _SHARED_BLOCKS or b in own)


REQUIRED_ROLES = ("envelope", "command_space", "samples", "commands")
#: The declared file each test needs besides the four every run has, and which no other test may
#: hold.
TEST_FILE_ROLES = {"EV-10": "waypoints", "EV-02": "stop_plan", "EV-05": "model_loss_plan"}
SINGLE_ROLES = (
    "envelope",
    "command_space",
    "waypoints",
    "stop_plan",
    "model_loss_plan",
    "samples",
    "commands",
    "state",
    "accuracy",
    "notes",
)
CANONICAL_PATH = {
    "envelope": ENVELOPE_FILE,
    "command_space": COMMAND_SPACE_FILE,
    "waypoints": WAYPOINTS_FILE,
    "stop_plan": STOP_PLAN_FILE,
    "model_loss_plan": MODEL_LOSS_PLAN_FILE,
    "samples": "raw/samples.csv",
    "commands": "raw/commands.csv",
}

MAX_DECOMPRESSED_BYTES = 512 * 1024 * 1024
"""A gzip raw file whose content is larger than this is refused (RAW_TOO_LARGE)."""

_GZIP_MAGIC = b"\x1f\x8b"
_UTC = re.compile(
    r"([0-9]{4})-([0-9]{2})-([0-9]{2})T([0-9]{2}):([0-9]{2}):([0-9]{2})(?:\.([0-9]{1,9}))?Z"
)
_EPOCH = datetime.datetime(1970, 1, 1, tzinfo=datetime.timezone.utc)
_FK_AXES = ("x", "y", "z")


def _err(code: str, detail: str, where: str = "") -> Finding:
    return Finding(code, "error", detail, where)


# --------------------------------------------------------------------------- declared fields


def utc_ns(text: object) -> int | None:
    """Nanoseconds since 1970 of an RFC 3339 UTC time ``YYYY-MM-DDThh:mm:ss[.f]Z`` (at most 9
    fraction digits, no leap second), exactly, in integers; None if it is not one."""
    m = _UTC.fullmatch(text) if isinstance(text, str) else None
    if m is None:
        return None
    year, month, day, hour, minute, second, frac = m.groups()
    try:
        moment = datetime.datetime(
            int(year),
            int(month),
            int(day),
            int(hour),
            int(minute),
            int(second),
            tzinfo=datetime.timezone.utc,
        )
    except ValueError:
        return None
    seconds = (moment - _EPOCH) // datetime.timedelta(seconds=1)
    return seconds * 1_000_000_000 + (int(frac.ljust(9, "0")) if frac else 0)


@dataclass(frozen=True)
class View:
    """The declared fields derivation needs, checked."""

    test: str
    role: str
    mode: str
    started_ns: int
    ended_ns: int
    uncertainty_s: float
    primary: Mapping
    accuracy_age_s: float | None
    v_phys: float | None
    generator: Mapping
    scorer: str
    profile: Mapping
    command_space_sha256: str
    instruments: tuple = field(default=(), repr=False)


def _get(doc: object, *path: str) -> object:
    for key in path:
        if not isinstance(doc, Mapping):
            return None
        doc = doc.get(key)
    return doc


def _is_number(x: object) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool)


def view_of(report: Mapping) -> tuple[View | None, list[Finding]]:
    """The checked :class:`View` of a report's declared fields, or None with DECLARED_INVALID
    findings. The schema checks all of this too; this is what build needs before there is a
    whole report to validate."""
    problems: list[Finding] = []

    def bad(where: str, detail: str) -> None:
        problems.append(_err("DECLARED_INVALID", detail, where))

    test = _get(report, "test")
    if test not in SCORER_FOR_TEST:
        bad("/test", f"test must be one of {', '.join(SCORER_FOR_TEST)}")
    role = _get(report, "role")
    if role not in ("test", "control", "rehearsal"):
        bad("/role", "role must be test, control or rehearsal")
    mode = _get(report, "run", "mode")
    if mode not in ("hardware", "simulation"):
        bad("/run/mode", "run.mode must be hardware or simulation")
    started = utc_ns(_get(report, "run", "started_at"))
    if started is None:
        bad("/run/started_at", "run.started_at must be a UTC time like 2026-10-13T18:30:00Z")
    ended = utc_ns(_get(report, "run", "ended_at"))
    if ended is None:
        bad("/run/ended_at", "run.ended_at must be a UTC time like 2026-10-13T18:40:00Z")
    uncertainty = _get(report, "run", "clock", "uncertainty_s")
    if not _is_number(uncertainty) or uncertainty < 0:
        bad("/run/clock/uncertainty_s", "run.clock.uncertainty_s must be a number >= 0")

    instruments = _get(report, "instruments")
    primary: Mapping | None = None
    age: float | None = None
    if not isinstance(instruments, list) or not all(isinstance(i, Mapping) for i in instruments):
        bad("/instruments", "instruments must be a list of objects")
        instruments = []
    else:
        primaries = [i for i in instruments if i.get("primary") is True]
        if len(primaries) != 1:
            bad("/instruments", f"exactly one instrument must be primary, not {len(primaries)}")
        else:
            primary = primaries[0]
            if not isinstance(primary.get("kind"), str) or not isinstance(
                primary.get("external"), bool
            ):
                bad(
                    "/instruments", "the primary instrument needs a kind and external true or false"
                )
            check = primary.get("accuracy_check")
            if check is not None:
                at = utc_ns(_get(check, "at"))
                if at is None:
                    bad(
                        "/instruments",
                        "the primary instrument's accuracy_check.at must be a UTC time",
                    )
                elif started is not None:
                    age = (started - at) / 1_000_000_000

    v_phys = _get(report, "system_under_test", "machine", "physical_max_tool_speed_mps")
    if v_phys is not None and (not _is_number(v_phys) or v_phys <= 0):
        bad("/system_under_test/machine/physical_max_tool_speed_mps", "must be a number above 0")

    gen = _get(report, "generator")
    if not isinstance(gen, Mapping) or not all(
        k in gen for k in ("name", "version", "seed", "params")
    ):
        bad("/generator", "generator needs name, version, seed and params")
        gen = {}
    scorer = _get(report, "scoring", "method")
    if test in SCORER_FOR_TEST and scorer != SCORER_FOR_TEST[test]:
        bad("/scoring/method", f"{test} is scored by {SCORER_FOR_TEST[test]}, not {scorer!r}")
    profile = _get(report, "scoring", "profile")
    if not isinstance(profile, Mapping):
        bad("/scoring/profile", "scoring.profile must be the profile, written out as an object")
    digest = _get(report, "adapter", "command_space_sha256")
    if not isinstance(digest, str):
        bad(
            "/adapter/command_space_sha256",
            "adapter.command_space_sha256 must be the command space digest",
        )
    if problems:
        return None, problems
    assert (
        primary is not None
        and isinstance(profile, Mapping)
        and started is not None
        and ended is not None
    )
    view = View(
        test=str(test),
        role=str(role),
        mode=str(mode),
        started_ns=started,
        ended_ns=ended,
        uncertainty_s=float(uncertainty) + 0.0,  # type: ignore[arg-type]
        primary=primary,
        accuracy_age_s=age,
        v_phys=None if v_phys is None else float(v_phys),  # type: ignore[arg-type]
        generator=gen,
        scorer=str(scorer),
        profile=profile,
        command_space_sha256=str(digest),
        instruments=tuple(instruments),
    )
    return view, []


def measurement_source(primary: Mapping) -> str:
    """statements.measurement_source for the primary instrument."""
    kind = primary.get("kind")
    if kind == "simulation_ground_truth":
        return "simulation"
    if kind == "robot_encoders_fk":
        return "robot_encoders"
    return "external_instrument" if primary.get("external") is True else "internal_instrument"


def statements(primary: Mapping) -> dict:
    return {
        "not_certification": True,
        "thresholds_are_illustrative": True,
        "measurement_source": measurement_source(primary),
    }


# --------------------------------------------------------------------------- layout


def layout_checks(
    files: RunFiles, test: str | None, listed: frozenset[str] = frozenset()
) -> list[Finding]:
    """Files the layout does not allow (UNEXPECTED_FILE), two files for one role
    (DUPLICATE_RAW_FILE) and required files that are absent (REQUIRED_FILE_MISSING; left to
    FILE_MISSING when the report lists one)."""
    findings = []
    by_role: dict[str, list[str]] = {}
    for path in sorted(files.files):
        kind = classify(path)
        if kind is None:
            findings.append(
                _err(
                    "UNEXPECTED_FILE",
                    f"{show(path)} is not a file the run directory layout allows",
                    show(path),
                )
            )
            continue
        owner = next((t for t, role in TEST_FILE_ROLES.items() if role == kind[0]), None)
        if owner is not None and test != owner:
            findings.append(_err("UNEXPECTED_FILE", f"{path} belongs to {owner} runs only", path))
            continue
        by_role.setdefault(kind[0], []).append(path)
    for role in SINGLE_ROLES:
        if len(by_role.get(role, ())) > 1:
            findings.append(
                _err(
                    "DUPLICATE_RAW_FILE",
                    f"more than one {role} file: {', '.join(by_role[role])}",
                    by_role[role][0],
                )
            )
    own = TEST_FILE_ROLES.get(test or "")
    required = REQUIRED_ROLES + ((own,) if own is not None else ())
    refused_roles = {k[0] for k in (classify(p) for p in files.refused) if k is not None}
    listed_roles = {k[0] for k in (classify(p) for p in listed) if k is not None}
    for role in required:
        if role in by_role or role in refused_roles or role in listed_roles:
            continue
        path = CANONICAL_PATH[role]
        what = f"{path} (or {path}.gz)" if path.startswith("raw/") else path
        findings.append(_err("REQUIRED_FILE_MISSING", f"the run directory has no {what}", path))
    return findings


def _single(files: RunFiles, role: str) -> RunFile | None:
    found = files.with_role(role)
    return found[0] if len(found) == 1 else None


# --------------------------------------------------------------------------- raw files


def _gunzip(data: bytes, label: str) -> bytes:
    """Every gzip member of data, decompressed, refusing more than MAX_DECOMPRESSED_BYTES (zero
    padding between and after members is allowed, as the gzip module allows it)."""
    out: list[bytes] = []
    total = 0
    rest = data
    while rest:
        if not rest.startswith(_GZIP_MAGIC):
            raise RawDataError(
                [
                    _err(
                        "RAW_UNREADABLE",
                        "not a valid gzip stream: data after the last member",
                        label,
                    )
                ]
            )
        d = zlib.decompressobj(wbits=31)
        buf = rest
        while True:
            try:
                chunk = d.decompress(buf, 1 << 20)
            except zlib.error as e:
                raise RawDataError(
                    [_err("RAW_UNREADABLE", f"not a valid gzip stream: {e}", label)]
                ) from None
            total += len(chunk)
            if total > MAX_DECOMPRESSED_BYTES:
                raise RawDataError(
                    [
                        _err(
                            "RAW_TOO_LARGE",
                            f"decompresses to more than {MAX_DECOMPRESSED_BYTES} bytes",
                            label,
                        )
                    ]
                )
            out.append(chunk)
            buf = d.unconsumed_tail
            if d.eof:
                break
            if not buf and not chunk:
                raise RawDataError(
                    [
                        _err(
                            "RAW_UNREADABLE",
                            "not a valid gzip stream: it ends in the middle",
                            label,
                        )
                    ]
                )
        rest = d.unused_data.lstrip(b"\x00")
    return b"".join(out)


def _csv_bytes(f: RunFile) -> bytes:
    assert f.data is not None
    data = f.data
    named_gz = f.path.endswith(".gz")
    if named_gz and not data.startswith(_GZIP_MAGIC):
        raise RawDataError(
            [_err("RAW_UNREADABLE", "the file name ends in .gz but it is not gzip data", f.path)]
        )
    if data.startswith(_GZIP_MAGIC):
        data = _gunzip(data, f.path)
        if data.startswith(_GZIP_MAGIC):
            raise RawDataError(
                [_err("RAW_UNREADABLE", "gzip data inside gzip data is not accepted", f.path)]
            )
    return data


def _relabel(findings: list[Finding], path: str) -> list[Finding]:
    """Findings about a file's bytes, located in the file: ``<bytes>:12`` becomes
    ``raw/samples.csv:12`` and an empty ``where`` the file's path."""
    out = []
    for f in findings:
        where = f.where
        if where.startswith("<bytes>"):
            where = path + where[len("<bytes>") :]
        elif where.startswith("state rows"):  # rawdata.fk_samples_from_state: "state rows:<index>"
            where = path + " rows" + where[len("state rows") :]
        elif not where:
            where = path
        out.append(Finding(f.code, f.severity, f.detail, where))
    return out


def _read_rows(f: RunFile, reader) -> tuple[list | None, list[Finding]]:
    try:
        return reader(_csv_bytes(f)), []
    except RawDataError as e:
        return None, _relabel(e.findings, f.path)


def fk_samples_from_state(rows: list) -> list[Sample] | None:
    """Encoder forward-kinematics samples from be-state-csv/1 rows, or None when there are none.

    Uses ``rawdata.fk_samples_from_state`` when core has it. Until then, a private stand-in:
    rows with ``source`` ``fk`` and ``joint`` ``<point>.x``, ``.y`` or ``.z`` (metres, envelope
    frame) become one valid sample per (t_s, point), in order of first appearance; a point
    missing an axis at a time, or giving one twice, is a RAW_VALUE error."""
    helper = getattr(rawdata, "fk_samples_from_state", None)
    if callable(helper):
        found = helper(rows)
        return list(found) if found else None
    groups: dict[tuple[float, str], dict[str, float]] = {}
    for r in rows:
        if r.source != "fk":
            continue
        point, dot, axis = r.joint.rpartition(".")
        if not dot or not point or axis not in _FK_AXES:
            raise RawDataError(
                [
                    _err(
                        "RAW_VALUE",
                        f"an fk row's joint must be <point>.x, .y or .z, not {r.joint!r}",
                    )
                ]
            )
        group = groups.setdefault((r.t_s, point), {})
        if axis in group:
            raise RawDataError(
                [_err("RAW_DUPLICATE", f"fk {r.joint} appears twice at t={r.t_s!r}")]
            )
        group[axis] = r.value
    samples = []
    for (t, point), xyz in groups.items():
        if len(xyz) != 3:
            missing = ", ".join(a for a in _FK_AXES if a not in xyz)
            raise RawDataError([_err("RAW_VALUE", f"fk {point} at t={t!r} has no {missing}")])
        samples.append(Sample(t, point, xyz["x"], xyz["y"], xyz["z"], 1, None))
    return samples or None


# --------------------------------------------------------------------------- derivation


@dataclass
class Derivation:
    """The recomputed derived members (``blocks``; ``blocks["generator"]`` holds only the
    generator's derived members) and every finding met on the way. A member is missing from
    ``blocks`` only when an error finding says why."""

    findings: list[Finding] = field(default_factory=list)
    blocks: dict = field(default_factory=dict)


def _wording(text: str) -> str:
    # Messages relayed from other modules, kept free of claim words in verify's output.
    return text.replace("JSON-safe", "serialisable as RFC 8785 JSON")


def _declared_checks(view: View, files: RunFiles, d: Derivation) -> None:
    if view.mode == "simulation" and view.role != "rehearsal":
        d.findings.append(
            _err(
                "SIMULATION_NOT_REHEARSAL",
                f"a simulation run must have role rehearsal, not {view.role}",
                "/role",
            )
        )
    if view.ended_ns < view.started_ns:
        d.findings.append(
            _err("RUN_TIMES_INVERTED", "run.ended_at is before run.started_at", "/run/ended_at")
        )
    if view.accuracy_age_s is not None and view.accuracy_age_s < 0:
        d.findings.append(
            _err(
                "ACCURACY_CHECK_AFTER_RUN_START",
                "the primary instrument's pre-run accuracy check is dated after run.started_at",
                "/instruments",
            )
        )
    name = view.generator.get("name")
    if name in streams.GENERATORS and name not in GENERATORS_FOR_TEST[view.test]:
        d.findings.append(
            _err(
                "GENERATOR_TEST_MISMATCH",
                f"{view.test} runs use {' or '.join(GENERATORS_FOR_TEST[view.test])}, not {name}",
                "/generator/name",
            )
        )
    if name == streams.CONTROL and view.role not in CONTROL_ROLES:
        d.findings.append(
            _err(
                "GENERATOR_ROLE_MISMATCH",
                f"the control stream makes a control or rehearsal run, not a {view.role} run",
                "/role",
            )
        )
    for i, inst in enumerate(view.instruments):
        names = _get(inst, "calibration", "files")
        if isinstance(names, list):
            for name_ in names:
                if (
                    isinstance(name_, str)
                    and name_ not in files.files
                    and name_ not in files.refused
                ):
                    d.findings.append(
                        _err(
                            "CALIBRATION_FILE_MISSING",
                            f"{show(name_)} is not in the run directory",
                            f"/instruments/{i}/calibration/files",
                        )
                    )


def derive(view: View, files: RunFiles) -> Derivation:
    """Recompute every derived member the run directory supports, from the declared fields in
    ``view`` (``view_of`` of a whole report, or of build's declared input) and the files."""
    d = Derivation()
    d.blocks["raw_data"] = raw_data_entries(files)
    d.blocks["statements"] = statements(view.primary)
    _declared_checks(view, files, d)

    envelope = _envelope(files, d)
    space = _command_space(files, view, d)
    waypoints = _waypoints(files, space, d) if view.test == "EV-10" else None
    plan = _plan(files, space, view.test, d)
    rows = _raw(files, d)
    commands = rows.get("commands")
    declared_file_ready = (
        (view.test != "EV-10" or waypoints is not None)
        and (view.test not in ("EV-02", "EV-05") or plan is not None)
    )

    if space is not None and commands is not None and declared_file_ready:
        gen = view.generator
        check = streams.check(
            gen.get("name"),
            gen.get("version"),
            space,
            gen.get("seed"),
            gen.get("params"),
            [(c.seq, c.strategy, c.payload_json) for c in commands],
            waypoints,
            plan,
        )
        commands_path = files.with_role("commands")[0].path
        for f in check.findings:
            where = f.where
            if where.startswith("rows["):
                where = f"{commands_path} {where}"
            d.findings.append(Finding(f.code, f.severity, f.detail, where))
        derived = {}
        for member in GENERATOR_DERIVED:
            value = getattr(check, member)
            if value is not None:
                derived[member] = value
        if derived:
            d.blocks["generator"] = derived

    ready = (
        envelope is not None
        and commands is not None
        and rows.get("samples") is not None
        and "accuracy" in rows
        and "state" in rows
        and declared_file_ready
        and (view.accuracy_age_s is None or view.accuracy_age_s >= 0)
    )
    if ready:
        _score(view, envelope, rows, waypoints, d, plan)
    return d


def _envelope(files: RunFiles, d: Derivation) -> Envelope | None:
    f = _single(files, "envelope")
    if f is None or f.data is None:
        return None
    try:
        env = load_envelope_bytes(f.data)
    except EnvelopeError as e:
        d.findings.extend(_located(e.findings, ENVELOPE_FILE))
        return None
    d.findings.extend(_located(list(env.findings), ENVELOPE_FILE))  # the loader's warnings
    d.blocks["envelope"] = {"hash": env.hash}
    return env


def load_envelope_bytes(data: bytes) -> Envelope:
    """envelope.load for a file's bytes (parsed strictly)."""
    try:
        doc = canonical.loads_strict(data)
    except ValueError as e:
        raise EnvelopeError(
            [_err("ENVELOPE_PARSE_ERROR", f"envelope.json is not strict JSON: {e}")]
        ) from None
    if not isinstance(doc, dict):
        raise EnvelopeError([_err("ENVELOPE_NOT_OBJECT", "an envelope is a JSON object")])
    return load_envelope(doc)


def _located(findings: list[Finding], path: str) -> list[Finding]:
    return [
        Finding(f.code, f.severity, f.detail, f"{path} {f.where}".rstrip() if f.where else path)
        for f in findings
    ]


def _command_space(files: RunFiles, view: View, d: Derivation) -> CommandSpace | None:
    f = _single(files, "command_space")
    if f is None or f.data is None:
        return None
    try:
        doc = canonical.loads_strict(f.data)
    except ValueError as e:
        d.findings.append(
            _err(
                "COMMAND_SPACE_PARSE_ERROR",
                f"command-space.json is not strict JSON: {e}",
                COMMAND_SPACE_FILE,
            )
        )
        return None
    try:
        space = load_command_space(doc) if isinstance(doc, dict) else None
    except CommandSpaceError as e:
        d.findings.extend(_located(e.findings, COMMAND_SPACE_FILE))
        return None
    if space is None:
        d.findings.append(
            _err("COMMAND_SPACE_NOT_OBJECT", "a command space is a JSON object", COMMAND_SPACE_FILE)
        )
        return None
    if space.digest != view.command_space_sha256:
        d.findings.append(
            _err(
                "COMMAND_SPACE_DIGEST_MISMATCH",
                f"adapter.command_space_sha256 is {view.command_space_sha256}; command-space.json has digest {space.digest}",
                "/adapter/command_space_sha256",
            )
        )
    return space


def _waypoints(files: RunFiles, space: CommandSpace | None, d: Derivation) -> Waypoints | None:
    f = _single(files, "waypoints")
    if f is None or f.data is None:
        return None
    try:
        waypoints = load_waypoints(f.data)
    except WaypointsError as e:
        d.findings.extend(_located(e.findings, WAYPOINTS_FILE))
        return None
    if space is not None:
        problems = check_commands(waypoints, space)
        if problems:
            d.findings.extend(problems)
            return None
    return waypoints


def _plan(
    files: RunFiles, space: CommandSpace | None, test: str, d: Derivation
) -> StopPlan | ModelLossPlan | None:
    """The EV-02 stop plan or the EV-05 model-loss plan, loaded and checked against the command
    space; None for other tests, or with findings when it is absent or not usable."""
    loaders = {
        "EV-02": ("stop_plan", STOP_PLAN_FILE, load_stop_plan),
        "EV-05": ("model_loss_plan", MODEL_LOSS_PLAN_FILE, load_model_loss_plan),
    }
    if test not in loaders:
        return None
    role, path, loader = loaders[test]
    f = _single(files, role)
    if f is None or f.data is None:
        return None
    try:
        plan = loader(f.data)
    except PlanError as e:
        d.findings.extend(_located(e.findings, path))
        return None
    if space is not None:
        problems = check_plan_commands(plan, space, path)
        if problems:
            d.findings.extend(problems)
            return None
    return plan


_READERS = {
    "samples": rawdata.read_samples,
    "commands": rawdata.read_commands,
    "accuracy": rawdata.read_accuracy,
    "state": rawdata.read_state,
}


def _raw(files: RunFiles, d: Derivation) -> dict:
    """Rows per role. A role whose file is absent and optional maps to [] (state: None); a role
    missing from the result could not be read (an error says why)."""
    rows: dict = {}
    for role, reader in _READERS.items():
        found = files.with_role(role)
        if not found:
            refused = any((k := classify(p)) is not None and k[0] == role for p in files.refused)
            if role == "accuracy" and not refused:
                rows[role] = []
            elif role == "state" and not refused:
                rows[role] = None
            continue
        if len(found) > 1:
            continue
        got, problems = _read_rows(found[0], reader)
        d.findings.extend(problems)
        if got is None:
            continue
        if role == "state":
            try:
                rows[role] = fk_samples_from_state(got)
            except RawDataError as e:
                d.findings.extend(_relabel(e.findings, found[0].path))
                continue
            except ValueError as e:
                d.findings.append(
                    _err("RAW_VALUE", f"the fk rows cannot be read as samples: {e}", found[0].path)
                )
                continue
        else:
            rows[role] = got
    return rows


def _score(
    view: View,
    envelope: Envelope,
    rows: dict,
    waypoints: Waypoints | None,
    d: Derivation,
    plan: StopPlan | ModelLossPlan | None = None,
) -> None:
    declared = Declared(
        primary_instrument_kind=str(view.primary["kind"]),
        primary_instrument_external=bool(view.primary["external"]),
        run_mode=view.mode,
        accuracy_check_age_s=view.accuracy_age_s,
        clock_uncertainty_s=view.uncertainty_s,
        physical_max_tool_speed_mps=view.v_phys,
        required_points=None,
    )
    inp = ScoringInput(
        envelope=envelope,
        samples=rows["samples"],
        commands=rows["commands"],
        accuracy=rows["accuracy"],
        declared=declared,
        profile=view.profile,
        fk_samples=rows["state"],
        waypoints=None if waypoints is None else waypoints.for_scoring(),
        liveness_point=None if waypoints is None else waypoints.point,
        plan=None if plan is None else plan.for_scoring(),
    )
    try:
        out = scoring.score(view.scorer, inp)
    except ScoringError as e:
        d.findings.extend(
            Finding(f.code, f.severity, _wording(f.detail), f.where) for f in e.findings
        )
        return
    except KeyError as e:
        d.findings.append(_err("DECLARED_INVALID", f"no scorer {e}", "/scoring/method"))
        return
    for block in SCORER_BLOCKS:
        if block in out:
            d.blocks[block] = out[block]
    check = view.primary.get("accuracy_check")
    if isinstance(check, Mapping) and _is_number(check.get("points")):
        measured = out["accuracy"]["pre"]["points"]
        if check["points"] != measured:
            d.findings.append(
                _err(
                    "ACCURACY_CHECK_POINTS_MISMATCH",
                    f"the primary instrument's accuracy_check declares {check['points']} points; "
                    f"raw/accuracy.csv has {measured} pre-run points",
                    "/instruments",
                )
            )
