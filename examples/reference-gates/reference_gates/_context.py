# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""What the gates read from the kit's wrapper context (plain data: mode, speed_factor and the run's
file paths)."""
from __future__ import annotations

import json
from collections.abc import Mapping
from pathlib import Path

__all__ = ["load_kit_config", "tool_point"]


def load_kit_config(context: Mapping) -> dict:
    """The kit config the run uses (tolerance_m, joint_map, start_pose_deg), or {} without one."""
    path = context.get("kit_config_path")
    if not path:
        return {}
    doc = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(doc, dict):
        raise ValueError(f"{path}: a kit config is a JSON object")  # noqa: TRY004 (a bad file)
    return doc


def tool_point(envelope) -> str:
    """The point a gate plans for: "tool" if the envelope declares it, else its first point (the
    kit's runner uses the same rule)."""
    return "tool" if "tool" in envelope.points else next(iter(envelope.points))
