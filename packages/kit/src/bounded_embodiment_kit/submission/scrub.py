# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""Scrub hostnames, usernames, home-directory paths and IP addresses from text (`be report`).

Replaced, in this order:

1. Home-directory paths: ``/home/<user>/...`` (also under ``/var``, ``/export`` and ``/usr``),
   ``/Users/<user>/...``, ``/root/...``, ``C:\\Users\\<user>\\...``, ``/mnt/c/Users/<user>/...``,
   ``~<user>/...``, and this machine's own home directory wherever it is. The path becomes ``~``
   plus its last component, the directories between dropped:
   ``/home/ana/runs/a/notes.md`` -> ``~/.../notes.md``; ``/home/ana/x.urdf`` -> ``~/x.urdf``.
2. IP addresses: IPv6 (validated by ``ipaddress``; at least four hex digits, so ``::1`` and
   clock times like ``18:00:00`` are left alone) and IPv4 (four octets, each at most 255; a
   version string such as ``4.10.0.84`` looks the same and is replaced too): ``[ip]``.
3. Local network names (``*.local``, ``*.lan``, ``*.localdomain``, ``*.internal``,
   ``*.home.arpa``): ``[host]``.
4. The hostnames in the :class:`Identity` (this machine's, by default), whole words, ignoring
   case: ``[host]``.
5. The usernames in the :class:`Identity`, whole words, matching case (so a signer named
   "Ana" survives the login name "ana"): ``[user]``.
6. Words the lab asks to redact (``be report --redact``), ignoring case: ``[redacted]``.

Generic account and host names (``pi``, ``root``, ``ubuntu``, ``raspberrypi``, ``robot`` ...) and
names shorter than three characters are not looked for: they identify nobody and would
replace ordinary words. "Whole word" means not next to a letter or digit (``_``, ``-`` and ``.``
are separators).

Deterministic: the result depends only on the text and the :class:`Identity`, never on the
time or on the order things were found; scrubbing twice gives the same text as once.

Structured files keep their structure: JSON is parsed and every string (keys included) is
scrubbed, then written again only if something changed; in CSV only text cells are scrubbed,
never a number, never the header, and never a column the caller protects (the generated
``strategy`` and ``payload_json`` of be-commands-csv/1, point and joint ids). A file nothing
was found in is returned byte for byte.
"""
from __future__ import annotations

import contextlib
import csv
import getpass
import io
import ipaddress
import json
import os
import platform
import re
import socket
from collections import Counter
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path

__all__ = ["GENERIC_HOSTS", "GENERIC_USERS", "HOME", "HOST", "IP", "REDACTED", "USER", "Identity",
           "Scrubber"]

HOME = "~"
HOST = "[host]"
USER = "[user]"
IP = "[ip]"
REDACTED = "[redacted]"

MIN_NAME = 3
GENERIC_USERS = frozenset({
    "admin", "administrator", "centos", "debian", "dietpi", "ec2-user", "fedora", "guest", "home",
    "host", "ip", "jetson", "lab", "nobody", "nvidia", "odroid", "operator", "orangepi", "radxa",
    "redacted", "robot", "rock", "root", "runner", "student", "test", "ubuntu", "user", "users",
    "vagrant",
})
GENERIC_HOSTS = frozenset({
    "arm", "computer", "controller", "debian", "desktop", "fedora", "gateway", "home", "host", "ip",
    "jetson", "lab", "laptop", "local", "localhost", "localhost.localdomain", "machine", "nano",
    "node", "orin", "raspberrypi", "redacted", "robot", "runner", "server", "ubuntu", "user",
    "workstation",
})

# One path component: no whitespace, separators, quotes, angle brackets, pipe or backtick.
_SEG = r"[^\s/\\'\"<>|`]"
_STOP = r"(?=[/\\\s'\"<>|`]|$)"
_HOME_RE = re.compile(
    r"(?<![\w.~-])(?:"
    rf"(?:/(?:var/|export/|usr/)?home|/Users)/{_SEG}+"
    rf"|(?:[A-Za-z]:|/mnt/[A-Za-z])[\\/]+(?i:users)[\\/]+{_SEG}+"
    rf"|/root{_STOP}"
    rf"|~[A-Za-z_][\w.-]*{_STOP}"
    rf")(?P<rest>(?:[/\\]{_SEG}*)*)"
)
_TRAILING = ".,;:!?)]}"
_IPV6_RE = re.compile(
    r"(?<![\w:.])(?=[0-9A-Fa-f:.]*:[0-9A-Fa-f:.]*:)[0-9A-Fa-f:.]*[0-9A-Fa-f:]"
    r"(?:%[A-Za-z0-9_.-]*[A-Za-z0-9_-])?(?![\w:])"
)
_IPV4_RE = re.compile(r"(?<![\w.])(?:\d{1,3}\.){3}\d{1,3}(?!\.?\d)(?!\w)")
_LOCAL_RE = re.compile(
    r"(?<![\w.-])(?:[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?\.)+"
    r"(?:local|lan|localdomain|internal|home\.arpa)(?![\w-])(?!\.[A-Za-z0-9])",
    re.IGNORECASE,
)
_HOSTNAME_OK = re.compile(r"[A-Za-z0-9](?:[A-Za-z0-9.-]*[A-Za-z0-9])?")
_NUMBER = re.compile(r"-?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?")


def _words(names: Iterable[str], generic: frozenset[str], *, ignore_case: bool) -> list[str]:
    """The names worth looking for, longest first then alphabetically (so a longer name wins)."""
    found = set()
    for name in names:
        name = (name or "").strip()
        key = name.lower()
        if len(name) < MIN_NAME or key in generic:
            continue
        found.add(key if ignore_case else name)
    return sorted(found, key=lambda n: (-len(n), n))


def _alternation(words: list[str], flags: int = 0) -> re.Pattern | None:
    if not words:
        return None
    body = "|".join(re.escape(w) for w in words)
    return re.compile(rf"(?<![A-Za-z0-9])(?:{body})(?![A-Za-z0-9])", flags)


@dataclass(frozen=True)
class Identity:
    """The names a :class:`Scrubber` looks for besides the patterns: hostnames, login names,
    home directories, and words to redact. :meth:`this_machine` reads them from this machine."""

    hostnames: tuple[str, ...] = ()
    usernames: tuple[str, ...] = ()
    homes: tuple[str, ...] = ()
    redact: tuple[str, ...] = ()

    @classmethod
    def this_machine(cls, *, redact: Iterable[str] = ()) -> Identity:
        hosts = {socket.gethostname(), platform.node()}
        users = {os.environ.get(k, "") for k in ("USER", "LOGNAME", "USERNAME")}
        with contextlib.suppress(Exception):  # no passwd entry in some containers
            users.add(getpass.getuser())
        homes = {os.environ.get(k, "") for k in ("HOME", "USERPROFILE")}
        with contextlib.suppress(Exception):
            homes.add(str(Path.home()))
        for name in sorted(hosts):
            if "." in name:
                hosts.add(name.split(".", 1)[0])
        return cls(tuple(sorted(h for h in hosts if h)), tuple(sorted(u for u in users if u)),
                   tuple(sorted(h for h in homes if h)), tuple(redact))


class Scrubber:
    """Scrubs text, JSON and CSV for one :class:`Identity`. ``counts`` tallies what was
    replaced, by ``(where, category)``; categories are ``home_path``, ``ip``, ``hostname``,
    ``username`` and ``redacted``."""

    def __init__(self, identity: Identity) -> None:
        self.identity = identity
        homes = set()
        for home in identity.homes:
            home = home.rstrip("/\\")
            # A home directory of one component ("/", "/home", "/root", "C:") is not personal
            # (and /root is a pattern of its own).
            if len([p for p in re.split(r"[/\\]", home) if p and not p.endswith(":")]) >= 2:
                homes.add(home)
        self._homes = [re.compile(rf"(?<![\w.~-]){re.escape(h)}(?P<rest>(?:[/\\]{_SEG}*)*){_STOP}")
                       for h in sorted(homes, key=lambda h: (-len(h), h))]
        hosts = []
        for name in identity.hostnames:
            if _HOSTNAME_OK.fullmatch(name or ""):
                hosts += [name, name.split(".", 1)[0]]
        self._host = _alternation(_words(hosts, GENERIC_HOSTS, ignore_case=True), re.IGNORECASE)
        self._user = _alternation(_words(identity.usernames, GENERIC_USERS, ignore_case=False))
        redact = sorted({w.strip() for w in identity.redact if w.strip()},
                        key=lambda w: (-len(w), w.lower()))
        self._redact = _alternation(redact, re.IGNORECASE)
        self.counts: Counter[tuple[str, str]] = Counter()

    # ----- text --------------------------------------------------------------------------------
    def text(self, s: str, where: str = "", *, record: bool = True) -> str:
        """``s`` with everything this scrubber looks for replaced."""
        tally: Counter[str] = Counter()
        for pattern in self._homes:
            s = pattern.sub(lambda m: self._home(m, tally), s)
        s = _HOME_RE.sub(lambda m: self._home(m, tally), s)
        s = _IPV6_RE.sub(lambda m: self._ipv6(m, tally), s)
        s = _IPV4_RE.sub(lambda m: self._ipv4(m, tally), s)
        s = self._sub(_LOCAL_RE, HOST, "hostname", s, tally)
        s = self._sub(self._host, HOST, "hostname", s, tally)
        s = self._sub(self._user, USER, "username", s, tally)
        s = self._sub(self._redact, REDACTED, "redacted", s, tally)
        if record:
            for category, n in tally.items():
                self.counts[(where, category)] += n
        return s

    def finds(self, s: str) -> Counter[str]:
        """What :meth:`text` would replace in ``s``, by category, without recording it."""
        tally: Counter[str] = Counter()
        saved = self.counts
        self.counts = Counter()
        try:
            self.text(s, "")
            for (_, category), n in self.counts.items():
                tally[category] += n
        finally:
            self.counts = saved
        return tally

    @staticmethod
    def _sub(pattern: re.Pattern | None, token: str, category: str, s: str,
             tally: Counter[str]) -> str:
        if pattern is None:
            return s

        def replace(m: re.Match) -> str:
            tally[category] += 1
            return token

        return pattern.sub(replace, s)

    @staticmethod
    def _home(m: re.Match, tally: Counter[str]) -> str:
        # Punctuation that ends a sentence is not part of the path: give it back.
        whole, trail = m.group(0), ""
        while len(whole) > 1 and whole[-1] in _TRAILING:
            trail = whole[-1] + trail
            whole = whole[:-1]
        cut = m.start("rest") - m.start()
        rest = whole[cut:] if len(whole) > cut else ""
        parts = [p for p in re.split(r"[/\\]", rest) if p]
        tally["home_path"] += 1
        if not parts:
            return HOME + trail
        if len(parts) == 1:
            return f"{HOME}/{parts[0]}{trail}"
        return f"{HOME}/.../{parts[-1]}{trail}"

    @staticmethod
    def _ipv6(m: re.Match, tally: Counter[str]) -> str:
        text = m.group(0)
        if sum(c in "0123456789abcdefABCDEF" for c in text.split("%", 1)[0]) < 4:
            return text
        try:
            ipaddress.IPv6Address(text)
        except ValueError:
            return text
        tally["ip"] += 1
        return IP

    @staticmethod
    def _ipv4(m: re.Match, tally: Counter[str]) -> str:
        text = m.group(0)
        if any(int(part) > 255 for part in text.split(".")):
            return text
        tally["ip"] += 1
        return IP

    # ----- structured files --------------------------------------------------------------------
    def json_bytes(self, data: bytes, where: str) -> bytes:
        """A JSON file with every string (keys included) scrubbed. Written again, indented, only
        if something changed; a file that is not JSON (or repeats a key) is scrubbed as text."""
        text = data.decode("utf-8")
        duplicate = False

        def pairs(items):
            nonlocal duplicate
            keys = [k for k, _ in items]
            duplicate = duplicate or len(keys) != len(set(keys))
            return dict(items)

        try:
            doc = json.loads(text, object_pairs_hook=pairs)
        except ValueError:
            doc, duplicate = None, True
        if duplicate:
            new = self.text(text, where)
            return data if new == text else new.encode("utf-8")
        before = sum(self.counts.values())
        scrubbed = self.value(doc, where)
        if sum(self.counts.values()) == before:
            return data
        return (json.dumps(scrubbed, indent=2, ensure_ascii=False) + "\n").encode("utf-8")

    def value(self, obj, where: str = ""):
        """A JSON value with its strings scrubbed (keys of objects too)."""
        if isinstance(obj, str):
            return self.text(obj, where)
        if isinstance(obj, list):
            return [self.value(v, where) for v in obj]
        if isinstance(obj, dict):
            out = {}
            for key, val in obj.items():
                new_key = self.text(key, where) if isinstance(key, str) else key
                if new_key in out:  # two keys scrubbed to one: keep them apart, deterministically
                    n = 2
                    while f"{new_key} ({n})" in out:
                        n += 1
                    new_key = f"{new_key} ({n})"
                out[new_key] = self.value(val, where)
            return out
        return obj

    def csv_bytes(self, data: bytes, where: str, protected: Iterable[str] = ()) -> bytes:
        """A CSV file with its text cells scrubbed: never the header row, a number, an empty cell
        or a column named in ``protected``. Written again only if a cell changed."""
        text = data.decode("utf-8")
        if not self.finds(text):
            return data
        try:
            rows = list(csv.reader(io.StringIO(text, newline=""), strict=True))
        except csv.Error:  # not CSV after all: as text (no number can match a pattern)
            return self.text_bytes(data, where)
        if not rows:
            return data
        names = set(protected)
        keep = {i for i, name in enumerate(rows[0]) if name in names}
        changed = False
        for row in rows[1:]:
            for i, cell in enumerate(row):
                if i in keep or not cell or _NUMBER.fullmatch(cell):
                    continue
                new = self.text(cell, where)
                if new != cell:
                    row[i] = new
                    changed = True
        if not changed:
            return data
        buf = io.StringIO()
        csv.writer(buf, lineterminator="\n").writerows(rows)
        return buf.getvalue().encode("utf-8")

    def text_bytes(self, data: bytes, where: str) -> bytes:
        """A UTF-8 text file scrubbed; byte for byte when nothing was found."""
        text = data.decode("utf-8")
        new = self.text(text, where)
        return data if new == text else new.encode("utf-8")

    def summary(self) -> list[str]:
        """One line per file: what was replaced there, by category (never the values)."""
        per: dict[str, Counter[str]] = {}
        for (where, category), n in sorted(self.counts.items()):
            per.setdefault(where or "(declared fields)", Counter())[category] += n
        return [f"{where}: " + ", ".join(f"{n} {category.replace('_', ' ')}"
                                         for category, n in sorted(per[where].items()))
                for where in sorted(per)]
