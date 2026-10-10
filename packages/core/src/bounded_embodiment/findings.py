"""Findings: the one result type every check in core returns."""
from __future__ import annotations

from dataclasses import dataclass

SEVERITIES = ("error", "warn", "info")


@dataclass(frozen=True)
class Finding:
    code: str
    severity: str
    detail: str
    where: str = ""

    def __post_init__(self) -> None:
        if self.severity not in SEVERITIES:
            raise ValueError(f"severity must be one of {SEVERITIES}, got {self.severity!r}")

    def as_dict(self) -> dict:
        d = {"code": self.code, "severity": self.severity, "detail": self.detail}
        if self.where:
            d["where"] = self.where
        return d


def errors(findings: list[Finding]) -> list[Finding]:
    return [f for f in findings if f.severity == "error"]
