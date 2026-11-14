"""``be``, the Bounded Embodiment command line (also installed as ``bounded-embodiment``).

Core commands::

    be keygen --out PATH --lab NAME --contact EMAIL [--added YYYY-MM-DD]
    be canon FILE [--unsigned] [--sha256]
    be envelope validate FILE [--json]
    be envelope hash FILE
    be envelope lint FILE [--json] [--physical-max-speed MPS]
    be envelope convert --from harness-arm FILE --machine-id ID --stop JSON --heartbeat JSON
    be verify ...          (arguments are handed to bounded_embodiment.report.cli_verify)

``verify`` and ``envelope`` import their modules only when run, and say plainly
when those modules are not available yet.

Extensions: each entry point in the group ``bounded_embodiment.cli`` is loaded
and called with the argparse subparsers object. It adds its own subcommands,
each with ``set_defaults(func=handler)``, where ``handler(args)`` returns an exit
code (``None`` means 0). An extension cannot replace a command that is already
registered, and one that fails to load is reported without breaking the rest.

Exit codes: 0 success; 1 the input was checked and found wanting (or could not
be read); 2 usage error or a command that is not available.
"""

from __future__ import annotations

import argparse
import contextlib
import datetime
import hashlib
import importlib
import json
import os
import sys
from importlib import metadata
from pathlib import Path
from collections.abc import Callable
from typing import Any

from . import __version__
from .canonical import CanonicalError, canonicalize, loads_strict

ENTRY_POINT_GROUP = "bounded_embodiment.cli"
EXIT_OK, EXIT_FAIL, EXIT_USAGE = 0, 1, 2

# Commands whose arguments belong to another module: name -> (module, hook).
PASSTHROUGH = {"verify": ("bounded_embodiment.report", "cli_verify")}

# Commands that the lab bench, bounded-embodiment-kit, adds. Only used to say so when it is missing.
KIT_COMMANDS = frozenset({"record", "fit", "instrument", "run", "goto-start", "doctor", "adapter",
                          "report", "submit"})
DOCS_URL = "https://github.com/RobotRegistryFoundation/bounded-embodiment"


def _err(message: str) -> None:
    print(f"be: {message}", file=sys.stderr)


def _not_available(command: str, why: str) -> int:
    _err(f"'{command}' is not available yet: {why}")
    return EXIT_USAGE


def _print_findings(findings: list, as_json: bool, fallback: str = "") -> None:
    if as_json:
        print(json.dumps({"valid": False, "findings": [f.as_dict() for f in findings], "error": fallback},
                         indent=2))
        return
    for f in findings:
        print(f"{f.severity}: {f.code} at {f.where or '/'}: {f.detail}", file=sys.stderr)
    if not findings and fallback:
        _err(fallback)


def _read_bytes(name: str) -> bytes:
    return sys.stdin.buffer.read() if name == "-" else Path(name).read_bytes()


def _json_value(text: str, flag: str) -> object:
    """A flag value that is inline JSON (starting with { or [) or a path to a JSON file."""
    stripped = text.lstrip()
    try:
        if stripped.startswith(("{", "[")):
            return loads_strict(stripped)
        return loads_strict(Path(text).read_bytes())
    except (OSError, CanonicalError) as exc:
        raise ValueError(f"{flag}: {exc}") from None


def _write_new(path: Path, text: str, *, mode: int = 0o644, force: bool = False) -> None:
    """Create path with text; refuse to replace an existing file unless force."""
    flags = os.O_WRONLY | os.O_CREAT | (os.O_TRUNC if force else os.O_EXCL)
    fd = os.open(path, flags, mode)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(text)
    except BaseException:
        with contextlib.suppress(OSError):
            path.unlink()
        raise


# --------------------------------------------------------------------------- keygen

_ENTRY_FLAGS = {"/lab": "--lab", "/contact": "--contact", "/added": "--added"}
# What a first-time lab needs to hear instead of the schema's regular expression.
_ENTRY_HINTS = {
    "/lab": "a lab id is 1 to 64 lower-case letters, digits and hyphens, starting and ending with a "
            "letter or digit (for example my-lab)",
    "/contact": "give an email address, for example lab@example.org",
    "/added": "give a real date as YYYY-MM-DD",
}


def _cmd_keygen(args: argparse.Namespace) -> int:
    from . import signing

    added = args.added or datetime.datetime.now(datetime.timezone.utc).date().isoformat()
    private, public = signing.generate_key()
    entry = signing.key_entry(public, args.lab, args.contact, added)
    problems = signing.check_key_entry(entry)
    if problems:
        values = {"/lab": args.lab, "/contact": args.contact, "/added": added}
        told: set[str] = set()
        for f in problems:
            flag = _ENTRY_FLAGS.get(f.where, f.where or "entry")
            if f.where in _ENTRY_HINTS:
                if f.where not in told:
                    told.add(f.where)
                    _err(f"{flag}: {values[f.where]!r} is not accepted: {_ENTRY_HINTS[f.where]}")
            else:
                _err(f"{flag}: {f.detail}")
        return EXIT_USAGE
    path = Path(args.out)
    key_file = {"kid": entry["kid"], **private}
    try:
        _write_new(path, json.dumps(key_file, indent=2) + "\n", mode=0o600)
        if hasattr(os, "chmod"):
            os.chmod(path, 0o600)
    except FileExistsError:
        _err(f"{path} already exists; refusing to overwrite a key file")
        return EXIT_FAIL
    except OSError as exc:
        _err(f"cannot write {path}: {exc.strerror or exc}")
        return EXIT_FAIL
    print(json.dumps(entry, indent=2))
    print(
        f"Wrote the private key to {path} (mode 0600). Keep it secret and never commit it.\n"
        f"Add the key entry printed above to labs/{args.lab}/lab.json in the results repository.",
        file=sys.stderr,
    )
    return EXIT_OK


# --------------------------------------------------------------------------- canon


def _cmd_canon(args: argparse.Namespace) -> int:
    try:
        value = loads_strict(_read_bytes(args.file))
        if args.unsigned and isinstance(value, dict):
            value = {k: v for k, v in value.items() if k != "signature"}
        out = canonicalize(value)
    except OSError as exc:
        _err(f"cannot read {args.file}: {exc.strerror or exc}")
        return EXIT_FAIL
    except CanonicalError as exc:
        _err(f"{args.file}: {exc}")
        return EXIT_FAIL
    if args.sha256:
        print("sha256:" + hashlib.sha256(out).hexdigest())
        return EXIT_OK
    binary = getattr(sys.stdout, "buffer", None)
    if binary is None:  # a text-only stdout (some IDE consoles)
        sys.stdout.write(out.decode("utf-8"))
        return EXIT_OK
    sys.stdout.flush()
    binary.write(out)
    binary.flush()
    return EXIT_OK


# --------------------------------------------------------------------------- envelope


def _envelope_module(command: str) -> Any:
    try:
        module = importlib.import_module("bounded_embodiment.envelope")
    except ImportError as exc:
        _not_available(command, f"bounded_embodiment.envelope cannot be imported ({exc})")
        return None
    if not callable(getattr(module, "load", None)):
        _not_available(command, "bounded_embodiment.envelope has no load() in this build")
        return None
    return module


def _envelope_errors(module: Any) -> tuple[type[BaseException], ...]:
    candidates = (getattr(module, "EnvelopeError", None), CanonicalError, OSError)
    return tuple(c for c in candidates if isinstance(c, type) and issubclass(c, BaseException))


def _load_envelope(args: argparse.Namespace, command: str) -> tuple[int, Any]:
    module = _envelope_module(command)
    if module is None:
        return EXIT_USAGE, None
    try:
        return EXIT_OK, module.load(Path(args.file))
    except _envelope_errors(module) as exc:
        _print_findings(list(getattr(exc, "findings", ()) or ()), getattr(args, "json", False), f"{args.file}: {exc}")
        return EXIT_FAIL, None


def _cmd_envelope_validate(args: argparse.Namespace) -> int:
    code, envelope = _load_envelope(args, "envelope validate")
    if envelope is None:
        return code
    findings = list(getattr(envelope, "findings", ()) or ())
    if args.json:
        print(json.dumps({"valid": True, "version": envelope.version, "hash": envelope.hash,
                          "findings": [f.as_dict() for f in findings]}, indent=2))
    else:
        print(f"valid envelope {envelope.version} {envelope.hash}")
        for f in findings:
            print(f"{f.severity}: {f.code} at {f.where or '/'}: {f.detail}")
    return EXIT_OK


def _cmd_envelope_hash(args: argparse.Namespace) -> int:
    code, envelope = _load_envelope(args, "envelope hash")
    if envelope is None:
        return code
    print(envelope.hash)
    return EXIT_OK


def _cmd_envelope_lint(args: argparse.Namespace) -> int:
    code, envelope = _load_envelope(args, "envelope lint")
    if envelope is None:
        return code
    try:
        lint_module = importlib.import_module("bounded_embodiment.envelope.lint")
    except ImportError as exc:
        return _not_available("envelope lint", f"bounded_embodiment.envelope.lint cannot be imported ({exc})")
    v_phys = args.physical_max_speed
    if v_phys is not None and not (v_phys > 0 and v_phys < float("inf")):
        _err("--physical-max-speed must be a number above 0")
        return EXIT_USAGE
    loader = list(getattr(envelope, "findings", ()) or ())
    found = lint_module.lint(envelope, physical_max_speed_mps=v_phys)
    errors = [f for f in found if f.severity == "error"]
    if args.json:
        print(json.dumps({"valid": True, "version": envelope.version, "hash": envelope.hash,
                          "findings": [f.as_dict() for f in loader],
                          "lint": [f.as_dict() for f in found],
                          "lint_errors": len(errors)}, indent=2))
    else:
        for f in loader + found:
            print(f"{f.severity}: {f.code} at {f.where or '/'}: {f.detail}")
        warns = len([f for f in found if f.severity == "warn"])
        print(f"{len(errors)} lint error(s), {warns} warning(s), "
              f"{len(found) - len(errors) - warns} note(s) for envelope {envelope.hash}")
    return EXIT_FAIL if errors else EXIT_OK


def _cmd_envelope_convert(args: argparse.Namespace) -> int:
    command = "envelope convert"
    if _envelope_module(command) is None:
        return EXIT_USAGE
    try:
        convert = importlib.import_module("bounded_embodiment.envelope.convert")
    except ImportError as exc:
        return _not_available(command, f"bounded_embodiment.envelope.convert cannot be imported ({exc})")
    from_harness_arm = getattr(convert, "from_harness_arm", None)
    if not callable(from_harness_arm):
        return _not_available(command, "bounded_embodiment.envelope.convert has no from_harness_arm()")
    try:
        source = loads_strict(_read_bytes(args.file))
        options: dict[str, object] = {
            "machine_id": args.machine_id,
            "stop": _json_value(args.stop, "--stop"),
            "heartbeat": _json_value(args.heartbeat, "--heartbeat"),
        }
    except OSError as exc:
        _err(f"cannot read {args.file}: {exc.strerror or exc}")
        return EXIT_FAIL
    except (CanonicalError, ValueError) as exc:
        _err(str(exc))
        return EXIT_FAIL
    for name in ("frame", "machine_class", "notes"):
        if getattr(args, name) is not None:
            options[name] = getattr(args, name)
    envelope_module = importlib.import_module("bounded_embodiment.envelope")
    failures: tuple[type[BaseException], ...] = (ValueError, *_envelope_errors(envelope_module))
    try:
        envelope_doc, kit_config = from_harness_arm(source, **options)
    except failures as exc:
        _print_findings(list(getattr(exc, "findings", ()) or ()), False, f"{args.file}: {exc}")
        return EXIT_FAIL
    text = json.dumps(envelope_doc, indent=2, ensure_ascii=False) + "\n"
    if not args.force:
        for target in (args.out, args.kit_config):
            if target and os.path.lexists(target):
                _err(f"{target} already exists; pass --force to replace it")
                return EXIT_FAIL
    try:
        if args.out:
            _write_new(Path(args.out), text, force=args.force)
        else:
            sys.stdout.write(text)
        if args.kit_config:
            _write_new(Path(args.kit_config), json.dumps(kit_config, indent=2, ensure_ascii=False) + "\n",
                       force=args.force)
    except FileExistsError as exc:
        _err(f"{exc.filename} already exists; pass --force to replace it")
        return EXIT_FAIL
    except OSError as exc:
        _err(f"cannot write {exc.filename}: {exc.strerror or exc}")
        return EXIT_FAIL
    if kit_config and not args.kit_config:
        _err("note: tolerances, start pose and joint map are kit configuration, not envelope content; "
             "pass --kit-config PATH to keep them")
    return EXIT_OK


# --------------------------------------------------------------------------- pass-through commands


def _passthrough(name: str, argv: list[str]) -> int:
    module_name, hook_name = PASSTHROUGH[name]
    try:
        module = importlib.import_module(module_name)
    except ImportError as exc:
        return _not_available(name, f"{module_name} cannot be imported ({exc})")
    hook = getattr(module, hook_name, None)
    if not callable(hook):
        return _not_available(name, f"{module_name} has no {hook_name}() in this build")
    return int(hook(argv) or 0)


# --------------------------------------------------------------------------- parser and extensions


class _GuardedSubparsers:
    """The subparsers object handed to extensions: adding an existing name is an error."""

    def __init__(self, action: Any) -> None:
        self._action = action

    def add_parser(self, name: str, **kwargs: Any) -> argparse.ArgumentParser:
        for candidate in (name, *kwargs.get("aliases", ())):
            if candidate in self._action.choices:
                raise ValueError(f"subcommand {candidate!r} is already registered")
        return self._action.add_parser(name, **kwargs)

    def __getattr__(self, attribute: str) -> Any:
        return getattr(self._action, attribute)


def _entry_points() -> list[Any]:
    found: dict[tuple[str, str], Any] = {}
    for ep in metadata.entry_points(group=ENTRY_POINT_GROUP):
        found.setdefault((ep.name, ep.value), ep)
    return sorted(found.values(), key=lambda ep: (ep.name, ep.value))


def _load_extensions(subparsers: Any) -> None:
    guarded = _GuardedSubparsers(subparsers)
    for ep in _entry_points():
        try:
            register: Callable[[Any], object] = ep.load()
            register(guarded)
        except Exception as exc:  # an extension must not take the core commands down with it
            _err(f"warning: CLI extension {ep.name!r} ({ep.value}) was not loaded: {exc}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="be",
        description="The Bounded Embodiment test method: lab keys, envelopes, and checking a signed "
        "report against its raw data (be verify). The lab bench, bounded-embodiment-kit, adds the "
        "commands that record, run, measure and package a test.",
        epilog=f"Lab guide, method text and source: {DOCS_URL}. Run 'be COMMAND --help' for a command's options.",
    )
    parser.add_argument("--version", action="version", version=f"bounded-embodiment {__version__}")
    sub = parser.add_subparsers(title="commands", metavar="COMMAND")

    p = sub.add_parser("keygen", help="create an Ed25519 signing key for a lab",
                       description="Create a lab's Ed25519 signing key. The private key goes to --out; the "
                       "public key entry for the results repository's labs/<lab>/lab.json is printed.")
    p.add_argument("--out", required=True,
                   help="where to write the private JWK (created with mode 0600, never overwritten)")
    p.add_argument("--lab", required=True, help="lab identifier, as in labs/<lab>/ of the results repository")
    p.add_argument("--contact", required=True, help="email address of the person responsible for the key")
    p.add_argument("--added", help="date the key is added, YYYY-MM-DD (default: today, UTC)")
    p.set_defaults(func=_cmd_keygen)

    p = sub.add_parser("canon", help="print the RFC 8785 canonical form of a JSON file (debugging)",
                       description="Print the RFC 8785 (JCS) canonical form of a JSON file, the bytes that "
                       "hashes and signatures cover.")
    p.add_argument("file", help="JSON file, or - for standard input")
    p.add_argument("--unsigned", action="store_true", help="drop the top-level signature member first")
    p.add_argument("--sha256", action="store_true", help="print sha256:<hex> of the canonical bytes instead")
    p.set_defaults(func=_cmd_canon)

    p = sub.add_parser("envelope", help="validate, hash or convert an envelope")
    env = p.add_subparsers(title="envelope commands", metavar="ACTION")
    p.set_defaults(func=lambda args, _p=p: (_p.print_help(sys.stderr), EXIT_USAGE)[1])
    q = env.add_parser("validate", help="check an envelope (0.1 or 0.2) and print its hash")
    q.add_argument("file", help="the envelope JSON file")
    q.add_argument("--json", action="store_true", help="machine-readable output")
    q.set_defaults(func=_cmd_envelope_validate)
    q = env.add_parser("hash", help="print the envelope hash (sha256 of its JCS without signature)")
    q.add_argument("file", help="the envelope JSON file")
    q.set_defaults(func=_cmd_envelope_hash)
    q = env.add_parser("lint", help="look for limits the schema accepts but that are unsafe, contradictory "
                       "or vacuous",
                       description="Load an envelope (0.1 or 0.2) and report semantic problems with stable "
                       "LINT_* codes: keep-out zones that cover the keep-in region or are too thin to "
                       "enforce, stop distances longer than the region is deep, proximity rules that "
                       "allow more speed closer in, and more. Exit status 1 when any problem is an error. "
                       "Lint is advice: no verdict depends on it.")
    q.add_argument("file", help="the envelope JSON file")
    q.add_argument("--json", action="store_true", help="machine-readable output")
    q.add_argument("--physical-max-speed", type=float, default=None, metavar="MPS",
                   help="the machine's physical maximum speed (v_phys), to flag speed limits above it")
    q.set_defaults(func=_cmd_envelope_lint)
    q = env.add_parser("convert", help="convert another envelope format to envelope 0.2")
    q.add_argument("--from", dest="source_format", required=True, choices=["harness-arm"],
                   help="format of the input file")
    q.add_argument("file", help="the envelope to convert, or - for standard input")
    q.add_argument("--machine-id", required=True, help="machine.id for the new envelope")
    q.add_argument("--stop", required=True, help="the stop block, as inline JSON or a JSON file")
    q.add_argument("--heartbeat", required=True, help="the heartbeat block, as inline JSON or a JSON file")
    q.add_argument("--frame", default=None, help="the envelope frame (default base_link)")
    q.add_argument("--class", dest="machine_class", default=None,
                   help="machine.class: arm (default), mobile_base, mobile_manipulator, legged or other")
    q.add_argument("--notes", default=None, help="free text appended to the envelope's notes")
    q.add_argument("--out", help="write the envelope here instead of standard output")
    q.add_argument("--kit-config", help="write the kit configuration (tolerances, start pose, joint map) here")
    q.add_argument("--force", action="store_true", help="replace existing output files")
    q.set_defaults(func=_cmd_envelope_convert)

    sub.add_parser("verify", help="check that a signed report agrees with its files and recompute its verdict",
                   add_help=False)

    _load_extensions(sub)
    return parser


def _registered(parser: argparse.ArgumentParser) -> set[str]:
    """The subcommand names the parser knows, extensions included."""
    for action in parser._actions:  # noqa: SLF001 - argparse has no public accessor for subparsers
        if isinstance(action, argparse._SubParsersAction):  # noqa: SLF001
            return set(action.choices)
    return set()


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    try:
        if argv and argv[0] in PASSTHROUGH:
            return _passthrough(argv[0], argv[1:])
        parser = build_parser()
        if argv and argv[0] in KIT_COMMANDS and argv[0] not in _registered(parser):
            _err(f"'{argv[0]}' comes with the lab bench, which is not installed here: "
                 "python -m pip install bounded-embodiment-kit")
            return EXIT_USAGE
        args = parser.parse_args(argv)
        func = getattr(args, "func", None)
        if func is None:
            parser.print_help(sys.stderr)
            return EXIT_USAGE
        result = func(args)
        return EXIT_OK if result is None else int(result)
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
