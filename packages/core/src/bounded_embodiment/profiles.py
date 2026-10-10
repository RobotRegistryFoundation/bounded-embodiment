"""Scoring profiles: the thresholds a scorer applies, shipped as package data.

The repository's ``profiles/*.json`` (CC BY 4.0) are mirrored byte for byte into the package's
``bounded_embodiment/profiles/`` directory by ``tools/sync_package_data.py``. That directory holds
data only; this module (``profiles.py``) is what ``import bounded_embodiment.profiles`` finds,
because a module takes precedence over a directory without ``__init__.py`` (PEP 420).

``load_profile(name)`` parses the packaged copy strictly and returns a fresh dict each call, so a
caller may change it without affecting anyone else.
"""

from __future__ import annotations

import functools
import re
from importlib import resources

from .canonical import loads_strict

__all__ = ["load_profile", "profile_names"]

_NAME = re.compile(r"[a-z0-9][a-z0-9._-]*")


def _directory():
    return resources.files("bounded_embodiment").joinpath("profiles")


@functools.cache
def _profile_bytes(name: str) -> bytes:
    return _directory().joinpath(name + ".json").read_bytes()


def profile_names() -> list[str]:
    """The names ``load_profile`` accepts, sorted."""
    directory = _directory()
    if not directory.is_dir():
        return []
    return sorted(
        entry.name[: -len(".json")]
        for entry in directory.iterdir()
        if entry.name.endswith(".json") and entry.is_file()
    )


def load_profile(name: str) -> dict:
    """The packaged profile called ``name`` (for example ``"ev03-illustrative-0.1"``).

    Raises ``KeyError`` for a name that is not shipped, ``CanonicalError`` (a ``ValueError``)
    if the packaged file is not strict JSON, and ``TypeError`` if it is not a JSON object.
    """
    if not isinstance(name, str) or _NAME.fullmatch(name) is None or name.endswith(".json"):
        raise KeyError(f"{name!r} is not a profile name")
    if name not in profile_names():
        raise KeyError(
            f"no packaged profile named {name!r}; known profiles: "
            f"{', '.join(profile_names()) or 'none'}"
        )
    profile = loads_strict(_profile_bytes(name))
    if not isinstance(profile, dict):
        raise TypeError(f"packaged profile {name!r} is not a JSON object")
    return profile
