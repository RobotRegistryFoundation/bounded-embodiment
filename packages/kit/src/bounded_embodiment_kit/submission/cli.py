# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""``be report`` and ``be submit`` (entry-point group ``bounded_embodiment.cli``, through
``bounded_embodiment_kit.cli``). The work is in ``report.py`` and ``submit.py``, imported when a
command runs."""
from __future__ import annotations

import argparse

__all__ = ["register"]

REPORT_HELP = """\
Package a run directory made by `be run` into a signed submission: the instrument's samples
brought onto the run timebase, hostnames, usernames, home paths and IP addresses scrubbed from
every text file and declared field, raw/*.csv gzipped (raw/accuracy.csv stays plain), photos
stripped of their metadata; then core builds the report (every derived member, the verdict
among them), signs it with KEY and checks it. Only a consistent submission is written.

Clock methods (--clock), for --samples:
  monotonic  the instrument ran on the runner's host and stamped CLOCK_MONOTONIC (be fit's
             samples.csv on the robot's computer); U 1 us
  wall       the recorder's clock.csv (--clock-csv, or SESSION/clock.csv beside
             SESSION/fit/samples.csv) against the runner's wall clock; needs
             --wall-clock-agreement-s (0 when one host ran both)
  run        already on the run timebase (default for the runner's simulated instrument)

Put RUN_DIR first: --samples, --photo and --instrument-file take several paths.
Exit: 0 written and consistent; 1 refused; 2 usage error. docs/seams/submission.md."""

SUBMIT_HELP = """\
Check a submission made by `be report` against a clone of the results repository and copy it to
results/LAB/<test>-<UTC>-s<seed>/: core's be verify with labs/LAB/lab.json's keys, simulations
only as rehearsals, a released core listed in kits.lock (--allow-dev-core: dry runs under a
test-* lab only), the repository's size limits; then its tools/path_guard.py and
tools/claim_lint.py on the copy. It never commits, pushes or opens a pull request: it prints the
commands. Exit: 0 copied; 1 refused; 2 usage error."""


def _cmd_report(args) -> int:
    from .report import execute_report

    return execute_report(args)


def _cmd_submit(args) -> int:
    from .submit import execute_submit

    return execute_submit(args)


def register(subparsers) -> None:
    p = subparsers.add_parser("report", help="package a run into a signed submission",
                              description=REPORT_HELP,
                              formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("run_dir", help="the run directory `be run` wrote")
    p.add_argument("--lab-config", required=True, metavar="LAB.json",
                   help="the declared fields the runner cannot know (be-lab-config/0.1)")
    p.add_argument("--key", required=True, metavar="KEY.json",
                   help="the private key `be keygen` wrote; never copied")
    p.add_argument("--out", required=True, metavar="SUBMISSION_DIR",
                   help="where the submission goes (must not exist, or be empty)")
    p.add_argument("--samples", nargs="+", action="extend", default=[], metavar="PATH",
                   help="the primary instrument's be-samples-csv/1 (gzip or not), on its own clock")
    p.add_argument("--clock", choices=["monotonic", "wall", "run"],
                   help="how --samples' timestamps reach the run timebase (above)")
    p.add_argument("--clock-csv", metavar="PATH", help="the recorder's clock.csv")
    p.add_argument("--wall-clock-agreement-s", type=float, metavar="S",
                   help="--clock wall: the most the two hosts' wall clocks can differ, seconds")
    p.add_argument("--accuracy", metavar="PATH", help="the instrument's be-accuracy-csv/1")
    p.add_argument("--instrument-file", nargs="+", action="extend", default=[], metavar="PATH",
                   help="calibration and setup files for raw/instrument/ (registration.json, "
                        "fit_meta.json, ...)")
    p.add_argument("--notes", metavar="NOTES.md", help="notes/NOTES.md")
    p.add_argument("--photo", nargs="+", action="extend", default=[], metavar="JPG",
                   help="setup photos (no people) for notes/photos/; their metadata is removed")
    p.add_argument("--test", choices=["EV-03", "EV-10", "EV-02", "EV-05"], help="checked against the run directory")
    p.add_argument("--role", choices=["test", "control", "rehearsal"],
                   help="default: rehearsal for a simulation or a shakedown, control for a control "
                        "run, else test")
    p.add_argument("--reproduces", metavar="RESULT", help="the result this run reproduces, "
                   "<lab>/<run directory>")
    p.add_argument("--control-run", metavar="RUN", help="the control run paired with this one")
    p.add_argument("--redact", action="append", default=[], metavar="TEXT",
                   help="more text to replace with [redacted] (another host's name, a room)")
    p.set_defaults(func=_cmd_report)

    s = subparsers.add_parser("submit", help="check a submission and copy it into the results "
                              "repository", description=SUBMIT_HELP,
                              formatter_class=argparse.RawDescriptionHelpFormatter)
    s.add_argument("submission_dir", help="a submission made by `be report`")
    s.add_argument("--results-repo", required=True, metavar="PATH",
                   help="your clone of the results repository")
    s.add_argument("--lab", required=True, help="your lab id (labs/<lab>/lab.json)")
    s.add_argument("--allow-dev-core", action="store_true",
                   help="accept an unreleased core: dry runs only, under a test-* lab")
    s.set_defaults(func=_cmd_submit)
