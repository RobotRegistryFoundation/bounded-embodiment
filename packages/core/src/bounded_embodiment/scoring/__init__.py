"""Scoring: the scorers and the verdict, as pure functions of explicit inputs.

``report.build`` and the results CI both call ``score(name, inp)`` on the raw data of a run, so
a verdict is always recomputed from measurements and never taken from what a kit wrote.

    from bounded_embodiment import rawdata
    from bounded_embodiment.envelope import load
    from bounded_embodiment.profiles import load_profile
    from bounded_embodiment.scoring import Declared, ScoringInput, score

    inp = ScoringInput(
        envelope=load("envelope.json"),
        samples=rawdata.read_samples("raw/samples.csv.gz"),
        commands=rawdata.read_commands("raw/commands.csv.gz"),
        accuracy=rawdata.read_accuracy("raw/accuracy.csv"),
        declared=Declared(primary_instrument_kind="optical_marker_tracker",
                          primary_instrument_external=True, run_mode="hardware",
                          accuracy_check_age_s=1800.0, clock_uncertainty_s=0.005,
                          physical_max_tool_speed_mps=1.0),
        profile=load_profile("ev03-illustrative-0.1"),
    )
    result = score("ev03-tool-point/1", inp)   # result["verdict"]["result"]

docs/seams/scoring.md documents the inputs, the output and every reason code.
"""

from __future__ import annotations

from collections.abc import Callable

from . import ev03, ev10
from .common import Declared, ScoringError, ScoringInput

__all__ = ["SCORERS", "Declared", "ScoringError", "ScoringInput", "ev03", "ev10", "score"]

SCORERS: dict[str, Callable[[ScoringInput], dict]] = {
    ev03.SCORER: ev03.score,
    ev10.SCORER: ev10.score,
}
"""Scorer function by scorer name (``report.scoring.method``)."""


def score(scorer: str, inp: ScoringInput) -> dict:
    """Run the named scorer. ``KeyError`` for an unknown name."""
    try:
        fn = SCORERS[scorer]
    except KeyError:
        raise KeyError(f"unknown scorer {scorer!r}; known: {', '.join(SCORERS)}") from None
    return fn(inp)
