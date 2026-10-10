# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""`be submit`: check a submission against a results repository's rules and copy it in.

    be submit SUBMISSION_DIR --results-repo PATH --lab LAB [--allow-dev-core]

It only checks and copies; it never commits, pushes or opens a pull request. In order:

1. the lab's keys: ``labs/<LAB>/lab.json`` in the clone (revoked keys left out), through core's
   ``report.load_keyring``. CI reads the same file from ``main``, so the lab must be merged first;
2. core's ``report.verify`` with those keys: refused unless consistent;
3. a simulation (run mode, primary instrument kind or measurement source) must be a rehearsal;
4. the core version the report names: a release listed in the clone's ``kits.lock``. An
   unreleased core (``.dev``, a local build) is refused unless ``--allow-dev-core``, which is
   for dry runs and only under a ``test-*`` lab (CI refuses such a run either way);
5. the limits the clone states (``MAX_FILE_BYTES`` and ``MAX_RUN_BYTES`` in tools/path_guard.py,
   else POLICY.md's "No file over N MB, and no run over M MB"), and no private key or Git LFS
   pointer in any file;
6. the copy, to ``results/<LAB>/<test>-<UTC>-s<seed>/`` (a new directory), verified again;
7. the clone's own ``tools/path_guard.py`` (paths mode: where the files go) and
   ``tools/claim_lint.py`` on the copy, and ``tools/pin_core.py`` for a released core, with their
   output shown. If one rejects the run, the copy is removed.

Then it prints the git and ``gh pr create`` commands to run. Exit status: 0 copied; 1 refused;
2 usage error.
"""
from __future__ import annotations

import ast
import json
import re
import shlex
import shutil
import subprocess
import sys
from collections.abc import Callable, Mapping
from pathlib import Path

from bounded_embodiment import __version__ as core_version
from bounded_embodiment import report as core_report

from . import ids
from .files import looks_like_lfs_pointer, looks_like_private_key

__all__ = ["EXIT_OK", "EXIT_REFUSED", "EXIT_USAGE", "RELEASE_RE", "Limits", "Refused",
           "execute_submit", "repo_limits"]

EXIT_OK, EXIT_REFUSED, EXIT_USAGE = 0, 1, 2
# Releases only, as the results repository's tools/pin_core.py accepts them.
RELEASE_RE = re.compile(r"\d+\.\d+\.\d+(?:(?:a|b|rc)\d+)?(?:\.post\d+)?")
DRY_RUN_PREFIX = "test-"
CORE_PACKAGE = "bounded-embodiment"
DEFAULT_FILE_BYTES = 20 * 1024 * 1024
DEFAULT_RUN_BYTES = 50 * 1024 * 1024
TOOL_TIMEOUT_S = 600


class Refused(Exception):
    pass


def _stderr(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


# ----- the results repository ----------------------------------------------------------------------
class Limits:
    def __init__(self, file_bytes: int, run_bytes: int, source: str):
        self.file_bytes, self.run_bytes, self.source = file_bytes, run_bytes, source

    def describe(self) -> str:
        return (f"{self.file_bytes / 2**20:g} MiB per file, {self.run_bytes / 2**20:g} MiB per run "
                f"({self.source})")


def _int_expr(node: ast.AST) -> int | None:
    """The value of an expression of integer literals, * and +; None for anything else."""
    if isinstance(node, ast.Constant) and type(node.value) is int:
        return node.value
    if isinstance(node, ast.BinOp) and isinstance(node.op, (ast.Mult, ast.Add)):
        left, right = _int_expr(node.left), _int_expr(node.right)
        if left is not None and right is not None:
            return left * right if isinstance(node.op, ast.Mult) else left + right
    return None


def repo_limits(repo: Path) -> Limits:
    """The per-file and per-run limits the results repository states: its path guard's constants
    (read, never run), else POLICY.md's sentence (MB taken as 10^6 bytes, the stricter
    reading), else 20 MiB and 50 MiB."""
    guard = repo / "tools" / "path_guard.py"
    try:
        tree = ast.parse(guard.read_text(encoding="utf-8"))
        found = {}
        for node in tree.body:
            if (isinstance(node, ast.Assign) and len(node.targets) == 1
                    and isinstance(node.targets[0], ast.Name)
                    and node.targets[0].id in ("MAX_FILE_BYTES", "MAX_RUN_BYTES")):
                found[node.targets[0].id] = _int_expr(node.value)
        if found.get("MAX_FILE_BYTES") and found.get("MAX_RUN_BYTES"):
            return Limits(found["MAX_FILE_BYTES"], found["MAX_RUN_BYTES"], "tools/path_guard.py")
    except (OSError, SyntaxError, ValueError):
        pass
    try:
        policy = (repo / "POLICY.md").read_text(encoding="utf-8")
        m = re.search(r"No file over (\d+) MB, and no run over (\d+) MB", policy)
        if m:
            return Limits(int(m.group(1)) * 10**6, int(m.group(2)) * 10**6, "POLICY.md")
    except OSError:
        pass
    return Limits(DEFAULT_FILE_BYTES, DEFAULT_RUN_BYTES, "the kit's defaults")


def _check_repo(repo: Path) -> None:
    needed = ["tools/path_guard.py", "tools/claim_lint.py", "labs", "results"]
    missing = [n for n in needed if not (repo / n).exists()]
    if missing:
        raise Refused(f"{repo} does not look like a clone of the results repository "
                      f"(no {', '.join(missing)})")


def _kits_lock(repo: Path) -> dict:
    path = repo / "kits.lock"
    try:
        doc = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        raise Refused(f"cannot read {path}: {e}") from None
    entries = doc.get(CORE_PACKAGE) if isinstance(doc, Mapping) else None
    if not isinstance(entries, Mapping):
        raise Refused(f"{path} has no {CORE_PACKAGE!r} entries")
    return dict(entries)


def _tool(repo: Path, script: str, argv: list[str], stdin: str | None = None) -> tuple[int, str]:
    """Run one of the results repository's own tools (standard library only) from its root."""
    try:
        proc = subprocess.run([sys.executable, str(repo / "tools" / script), *argv], cwd=repo,
                              input=stdin, capture_output=True, text=True, timeout=TOOL_TIMEOUT_S,
                              check=False)
    except (OSError, subprocess.SubprocessError) as e:
        return 2, f"could not run tools/{script}: {e}"
    return proc.returncode, (proc.stdout + proc.stderr).rstrip()


def _simulation(report: Mapping) -> str | None:
    """Why the report is a simulation, or None."""
    if (report.get("run") or {}).get("mode") == "simulation":
        return "run.mode is simulation"
    for inst in report.get("instruments") or []:
        if isinstance(inst, Mapping) and inst.get("primary") is True and \
                inst.get("kind") == "simulation_ground_truth":
            return "the primary instrument is simulation_ground_truth"
    if (report.get("statements") or {}).get("measurement_source") == "simulation":
        return "statements.measurement_source is simulation"
    return None


# ----- the command ---------------------------------------------------------------------------------
def _findings(result) -> list[str]:
    return [f"  {f.severity}: {f.code}{' at ' + f.where if f.where else ''}: {f.detail}"
            for f in result.findings if f.severity != "info"]


def _checks(args, say) -> tuple[Path, Path, dict, list[str], dict, bool]:
    """Everything before the copy; Refused on the first rule broken."""
    submission = Path(args.submission_dir)
    repo = Path(args.results_repo)
    lab = args.lab
    if not ids.LAB_ID_RE.fullmatch(lab or ""):
        raise Refused(f"--lab {lab!r} is not a lab id (lower-case letters, digits and inner "
                      "hyphens, 1 to 64 characters)")
    _check_repo(repo)
    lab_file = repo / "labs" / lab / "lab.json"
    if not lab_file.is_file() or lab_file.is_symlink():
        raise Refused(f"{lab_file} is missing: register the lab first (README.md, step 2); CI reads "
                      "its keys from main")
    try:
        keyring = core_report.load_keyring(lab_file)
    except core_report.KeyringError as e:
        raise Refused(f"{lab_file}: {e}") from None
    if not keyring:
        raise Refused(f"{lab_file} has no unrevoked key")

    result = core_report.verify(submission, keyring)
    if not result.consistent or result.report is None:
        raise Refused(f"be verify: {submission} is not consistent with {len(keyring)} key(s) from "
                      f"{lab_file}:\n" + "\n".join(_findings(result)))
    report = result.report
    verdict = result.verdict or {}
    say(f"be verify: consistent under core {core_version} with the keys of labs/{lab}/lab.json; "
        f"the verdict core derives from the files: {verdict.get('result')}.")
    for line in _findings(result):
        say(line)

    role = report.get("role")
    why = _simulation(report)
    if why and role != "rehearsal":
        raise Refused(f"a simulation is a rehearsal ({why}), but the report's role is {role!r}")

    stated = (report.get("provenance") or {}).get("core_version")
    released = isinstance(stated, str) and RELEASE_RE.fullmatch(stated) is not None
    if not released:
        if not args.allow_dev_core:
            raise Refused(f"the report names core {stated!r}, which is not a release; CI only "
                          "verifies under released cores listed in kits.lock (a dry run can pass "
                          "--allow-dev-core under a test-* lab)")
        if not lab.startswith(DRY_RUN_PREFIX):
            raise Refused(f"--allow-dev-core is for dry runs, which go under a {DRY_RUN_PREFIX}* "
                          f"lab, not {lab!r}")
        say(f"note: core {stated} is not a release: a dry run. CI's pin_core.py refuses it.")
    elif stated not in _kits_lock(repo):
        raise Refused(f"core {stated} is not in {repo / 'kits.lock'}; CI refuses reports from cores "
                      "it does not list (wait for the maintainer's kits.lock update)")
    if stated != core_version:
        say(f"note: checked here with core {core_version}; CI checks with core {stated}, the one "
            "the report names.")

    limits = repo_limits(repo)
    files = [core_report.REPORT_FILE] + [e["path"] for e in report.get("raw_data", [])]
    total = 0
    for rel in files:
        try:
            data = (submission / rel).read_bytes()
        except OSError as e:
            raise Refused(f"cannot read {rel}: {e.strerror or e}") from None
        total += len(data)
        if len(data) > limits.file_bytes:
            raise Refused(f"{rel} is {len(data)} bytes; the limit is {limits.describe()}")
        if looks_like_private_key(data):
            raise Refused(f"{rel} looks like a private key; it must never be submitted")
        if looks_like_lfs_pointer(data):
            raise Refused(f"{rel} is a Git LFS pointer; the results repository takes the file itself")
    if total > limits.run_bytes:
        raise Refused(f"the run is {total} bytes in all; the limit is {limits.describe()}")
    say(f"Sizes: {len(files)} files, {total} bytes; limits {limits.describe()}.")

    try:
        name = ids.run_dir_name(report)
    except ids.IdError as e:
        raise Refused(str(e)) from None
    dest = repo / "results" / lab / name
    if dest.exists() or dest.is_symlink():
        raise Refused(f"{dest} already exists; results are permanent (POLICY.md), so a new run "
                      "goes in a new directory")
    return submission, dest, report, files, keyring, released


def _copy(submission: Path, dest: Path, files: list[str]) -> None:
    dest.mkdir(parents=True)
    for rel in files:
        target = dest / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(submission / rel, target)


def _remove(dest: Path, created_lab_dir: bool) -> None:
    shutil.rmtree(dest, ignore_errors=True)
    if created_lab_dir:
        try:
            dest.parent.rmdir()
        except OSError:
            pass


def _next_steps(repo: Path, rel_dest: str, report: Mapping, lab: str, dry_run: bool) -> str:
    run = rel_dest.rsplit("/", 1)[-1]
    test, role = report.get("test"), report.get("role")
    verdict = (report.get("verdict") or {}).get("result")
    core = (report.get("provenance") or {}).get("core_version")
    title = f"Results: {lab}/{run} ({test}, {role})"
    body = (f"Self-run {test} run, role {role}. The verdict core {core} derives from the raw data: "
            f"{verdict}. Illustrative thresholds. Copied with `be submit`."
            + (" A dry run under a test lab." if dry_run else ""))
    branch = f"results/{lab}/{run}"
    q = shlex.quote
    return "\n".join([
        f"cd {q(str(repo.resolve()))}",
        f"git switch -c {q(branch)}",
        f"git add {q(rel_dest)}",
        f"git commit -m {q(title)}",
        f"git push -u origin {q(branch)}",
        f"gh pr create --base main --head {q(branch)} --title {q(title)} --body {q(body)}",
    ])


def execute_submit(args, *, say: Callable[[str], None] = _stderr) -> int:
    try:
        submission, dest, report, files, keyring, released = _checks(args, say)
    except Refused as e:
        say(f"be submit: refused: {e}")
        return EXIT_REFUSED
    repo = Path(args.results_repo)
    rel_dest = dest.relative_to(repo).as_posix()
    created_lab_dir = not dest.parent.exists()
    try:
        _copy(submission, dest, files)
        again = core_report.verify(dest, keyring)
        if not again.consistent:
            raise Refused("the copy is not consistent:\n" + "\n".join(_findings(again)))
        say(f"Copied to {rel_dest}.")
        paths = "\n".join(f"{rel_dest}/{rel}" for rel in sorted(files)) + "\n"
        tools = [("path_guard.py", ["--paths-from", "-"], paths),
                 ("claim_lint.py", [rel_dest], None)]
        if released:
            tools.append(("pin_core.py", ["--run-dir", rel_dest], None))
        else:
            say("tools/pin_core.py: not run (an unreleased core; CI refuses it).")
        failed = []
        for script, argv, stdin in tools:
            code, output = _tool(repo, script, argv, stdin)
            say(f"tools/{script} (exit {code}):")
            for line in output.splitlines() or [""]:
                say(f"  {line}")
            if code != 0:
                failed.append(script)
        if failed:
            raise Refused(f"the results repository's {', '.join(failed)} rejected the run; the copy "
                          "was removed")
    except Refused as e:
        _remove(dest, created_lab_dir)
        say(f"be submit: refused: {e}")
        return EXIT_REFUSED
    except BaseException:
        _remove(dest, created_lab_dir)
        raise
    say("\nNothing was committed or pushed. To open the pull request:")
    print(_next_steps(repo, rel_dest, report, args.lab, not released))
    return EXIT_OK
