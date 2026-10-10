# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""`be submit`: the copy, and every refusal, against a results repository with stand-in tools
(the end-to-end test runs the real ones)."""
from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import _submission_support as sup
from bounded_embodiment import report as core_report
from bounded_embodiment import signing
from bounded_embodiment_kit.submission import report as kitreport
from bounded_embodiment_kit.submission import submit
from bounded_embodiment_kit.submission.scrub import Identity

LAB = "test-lab"


@pytest.fixture(scope="module")
def made(tmp_path_factory):
    """One consistent submission (a hardware EV-03 from the golden ev03-pass) and its key."""
    tmp = tmp_path_factory.mktemp("submit")
    saved = Identity.this_machine
    Identity.this_machine = classmethod(lambda cls, redact=(): Identity(("benchhost",), ("ana",)))
    try:
        key_path, entry = sup.make_key(tmp / "lab.key.json", LAB)
        run = sup.runner_run(tmp)
        (tmp / "NOTES.md").write_text("Camera on a tripod, 1.2 m from the arm.\n")
        args = sup.report_args(tmp, run, key_path, extra=["--notes", tmp / "NOTES.md"])
        assert kitreport.execute_report(args, say=lambda m: None) == 0
    finally:
        Identity.this_machine = saved
    jwk = {k: v for k, v in json.loads(key_path.read_text()).items() if k != "kid"}
    return tmp / "submission", entry, jwk


@pytest.fixture
def sub(made, tmp_path):
    """A fresh copy of the submission to change."""
    copy = tmp_path / "submission"
    shutil.copytree(made[0], copy)
    return copy


def _args(sub_dir, repo, lab=LAB, dev=True):
    argv = ["submit", sub_dir, "--results-repo", repo, "--lab", lab]
    return sup.be(*argv, *(["--allow-dev-core"] if dev else []))


def _run(args):
    logs: list[str] = []
    return submit.execute_submit(args, say=logs.append), logs


def _refused(args, needle: str) -> list[str]:
    code, logs = _run(args)
    assert code == submit.EXIT_REFUSED, logs
    assert any(needle in m for m in logs), logs
    results = Path(args.results_repo) / "results"
    if results.is_dir():
        assert not [p for p in results.rglob("*") if p.is_file() and p.name != "README.md"], \
            "a refused submission left files behind"
    return logs


def rebuild(sub_dir: Path, jwk: dict, mutate=None, add: dict | None = None,
            core: str | None = None) -> None:
    """Build and sign the submission's report again with core after changing its declared fields
    (``mutate``) or adding files (``add``: path -> bytes). ``core`` names another core version
    in the signed report, as a report built by a released core would (verify then warns
    CORE_VERSION_DIFFERS, nothing more)."""
    rep = json.loads((sub_dir / "report.json").read_text())
    declared = {k: v for k, v in rep.items()
                if k not in core_report.DERIVED_BLOCKS and k != "signature"}
    declared["generator"] = {k: v for k, v in rep["generator"].items()
                             if k not in core_report.GENERATOR_DERIVED}
    if mutate:
        mutate(declared)
    for rel, data in (add or {}).items():
        (sub_dir / rel).parent.mkdir(parents=True, exist_ok=True)
        (sub_dir / rel).write_bytes(data)
    (sub_dir / "report.json").unlink()
    built = core_report.build(declared, sub_dir)
    if core is not None:
        built["provenance"]["core_version"] = core
    (sub_dir / "report.json").write_bytes(core_report.to_json(core_report.sign_report(built, jwk)))


def test_a_consistent_dry_run_is_copied_and_checked_and_nothing_is_committed(made, sub, tmp_path,
                                                                            capsys):
    repo = sup.fake_results_repo(tmp_path, LAB, [made[1]])
    code, logs = _run(_args(sub, repo))
    assert code == submit.EXIT_OK, logs
    rep = json.loads((sub / "report.json").read_text())
    dest = repo / "results" / LAB / "ev03-20261010T180000Z-s11"
    assert dest.is_dir()
    copied = sorted(p.relative_to(dest).as_posix() for p in dest.rglob("*") if p.is_file())
    assert copied == sorted(["report.json", *(e["path"] for e in rep["raw_data"])])
    for rel in copied:
        assert (dest / rel).read_bytes() == (sub / rel).read_bytes()
    text = "\n".join(logs)
    assert "be verify: consistent" in text and "path-guard: accepted" in text
    assert "claim-lint: no claim words" in text and "a dry run" in text
    assert "pin_core.py: not run" in text
    out = capsys.readouterr().out
    assert "gh pr create --base main" in out and f"git add results/{LAB}/ev03-20261010T180000Z-s11" in out
    assert not (repo / ".git").exists()


def test_a_released_core_listed_in_kits_lock_is_accepted_and_pinned(made, sub, tmp_path):
    rebuild(sub, made[2], core="0.1.0")
    repo = sup.fake_results_repo(tmp_path, LAB, [made[1]],
                                 kits={"0.1.0": {"wheel_sha256": "0" * 64, "released": "2026-10-12"}})
    code, logs = _run(_args(sub, repo, dev=False))
    assert code == submit.EXIT_OK, logs
    text = "\n".join(logs)
    assert "tools/pin_core.py (exit 0)" in text and "CI checks with core 0.1.0" in text


# ----- refusals ------------------------------------------------------------------------------------
def test_refused_without_a_results_repository_or_a_registered_lab(made, sub, tmp_path):
    _refused(_args(sub, tmp_path), "does not look like a clone of the results repository")
    repo = sup.fake_results_repo(tmp_path, LAB, [made[1]])
    _refused(_args(sub, repo, lab="Test_Lab"), "is not a lab id")
    _refused(_args(sub, repo, lab="test-other"), "register the lab first")
    broken = dict(made[1], kid="A" * 43)
    sup.write_json(repo / "labs" / LAB / "lab.json", {"keys": [broken]})
    _refused(_args(sub, repo), "KID_MISMATCH")
    sup.write_json(repo / "labs" / LAB / "lab.json", {"keys": [dict(made[1], revoked="2026-10-05")]})
    _refused(_args(sub, repo), "has no unrevoked key")


def test_refused_unless_consistent_with_the_labs_keys(made, sub, tmp_path):
    other = signing.key_entry(signing.generate_key()[1], LAB, "lab@example.org", "2026-10-01")
    repo = sup.fake_results_repo(tmp_path, LAB, [other])
    _refused(_args(sub, repo), "UNKNOWN_KID")
    repo2 = sup.fake_results_repo(tmp_path / "two", LAB, [made[1]])
    with (sub / "notes" / "NOTES.md").open("a") as fh:
        fh.write("edited after signing\n")
    _refused(_args(sub, repo2), "FILE_HASH_MISMATCH")


def test_a_simulation_must_be_a_rehearsal(made, sub, tmp_path):
    def simulated_instrument(declared):
        inst = declared["instruments"][0]
        inst.update(kind="simulation_ground_truth", external=False)

    rebuild(sub, made[2], simulated_instrument)
    assert core_report.verify(sub, {made[1]["kid"]: made[1]["public_jwk"]}).consistent
    repo = sup.fake_results_repo(tmp_path, LAB, [made[1]])
    _refused(_args(sub, repo), "a simulation is a rehearsal (the primary instrument is "
                               "simulation_ground_truth), but the report's role is 'test'")


def test_core_versions(made, sub, tmp_path):
    repo = sup.fake_results_repo(tmp_path, LAB, [made[1]])
    _refused(_args(sub, repo, dev=False), "which is not a release")
    prod = sup.fake_results_repo(tmp_path / "prod", "acme-robotics", [made[1]])
    _refused(_args(sub, prod, lab="acme-robotics"), "--allow-dev-core is for dry runs")
    rebuild(sub, made[2], core="0.1.1")
    _refused(_args(sub, repo, dev=False), "core 0.1.1 is not in")
    (repo / "kits.lock").unlink()
    _refused(_args(sub, repo, dev=False), "kits.lock")
    pinning = sup.fake_results_repo(tmp_path / "pin", LAB, [made[1]], pin_refuses=True,
                                    kits={"0.1.1": {"wheel_sha256": "0" * 64, "released": "2026-10-12"}})
    logs = _refused(_args(sub, pinning, dev=False), "pin_core.py rejected the run")
    assert any("pin-core: refused" in m for m in logs)


def test_a_copy_that_does_not_verify_is_removed(made, sub, tmp_path, monkeypatch):
    repo = sup.fake_results_repo(tmp_path, LAB, [made[1]])
    real = core_report.verify
    calls = []

    def second_call_fails(run_dir, keyring):
        calls.append(run_dir)
        if len(calls) == 2:  # the copy: as if a byte changed on the way
            Path(run_dir, "raw", "accuracy.csv").write_bytes(b"changed")
        return real(run_dir, keyring)

    monkeypatch.setattr(submit.core_report, "verify", second_call_fails)
    _refused(_args(sub, repo), "the copy is not consistent")
    assert len(calls) == 2 and not (repo / "results" / LAB).exists()


def test_size_limits_come_from_the_repository(made, sub, tmp_path):
    small_files = sup.fake_results_repo(tmp_path / "a", LAB, [made[1]], file_bytes="4 * 1024")
    _refused(_args(sub, small_files), "the limit is 0.00390625 MiB per file")
    small_run = sup.fake_results_repo(tmp_path / "b", LAB, [made[1]], run_bytes="20000 + 0")
    _refused(_args(sub, small_run), "bytes in all; the limit is")


def test_repo_limits_fall_back_to_policy_then_defaults(tmp_path):
    repo = sup.fake_results_repo(tmp_path, LAB, [], guard=False)
    (repo / "tools" / "path_guard.py").write_text("MAX_FILE_BYTES = compute()\n")
    limits = submit.repo_limits(repo)
    assert (limits.file_bytes, limits.run_bytes, limits.source) == (20 * 10**6, 50 * 10**6, "POLICY.md")
    (repo / "POLICY.md").unlink()
    limits = submit.repo_limits(repo)
    assert (limits.file_bytes, limits.run_bytes) == (20 * 2**20, 50 * 2**20)
    (repo / "tools" / "path_guard.py").write_text("MAX_FILE_BYTES = 3 * 1024\nMAX_RUN_BYTES = 9 * 1024\n")
    assert (submit.repo_limits(repo).file_bytes, submit.repo_limits(repo).run_bytes) == (3072, 9216)


def test_refused_with_a_private_key_or_an_lfs_pointer_inside(made, sub, tmp_path):
    repo = sup.fake_results_repo(tmp_path, LAB, [made[1]])
    leaked = json.dumps({"kty": "OKP", "crv": "Ed25519", "x": "x" * 43, "d": "d" * 43}).encode()
    rebuild(sub, made[2], add={"raw/instrument/key.json": leaked})
    _refused(_args(sub, repo), "raw/instrument/key.json looks like a private key")
    lfs_sub = sub.parent / "lfs"
    shutil.copytree(made[0], lfs_sub)
    rebuild(lfs_sub, made[2], add={"raw/instrument/video.bin":
                                   b"version https://git-lfs.github.com/spec/v1\noid sha256:00\n"})
    _refused(_args(lfs_sub, repo), "is a Git LFS pointer")


def test_refused_when_the_run_is_already_there(made, sub, tmp_path):
    repo = sup.fake_results_repo(tmp_path, LAB, [made[1]])
    dest = repo / "results" / LAB / "ev03-20261010T180000Z-s11"
    dest.mkdir(parents=True)
    code, logs = _run(_args(sub, repo))
    assert code == submit.EXIT_REFUSED and any("already exists" in m for m in logs)
    assert dest.is_dir() and not any(dest.iterdir())


def test_the_repositorys_tools_can_reject_and_the_copy_is_removed(made, sub, tmp_path):
    guard = sup.fake_results_repo(tmp_path / "a", LAB, [made[1]], reject=True)
    _refused(_args(sub, guard), "path_guard.py rejected the run; the copy was removed")
    assert not (guard / "results" / LAB).exists()

    claims = sup.fake_results_repo(tmp_path / "b", LAB, [made[1]])
    (sub / "notes" / "NOTES.md").write_text("The arm is safe.\n")
    rebuild(sub, made[2])
    logs = _refused(_args(sub, claims), "claim_lint.py rejected the run")
    assert any("claim-lint: 1 claim word(s) found" in m for m in logs)
