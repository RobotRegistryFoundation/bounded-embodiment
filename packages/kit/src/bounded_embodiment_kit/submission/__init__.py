# SPDX-FileCopyrightText: 2026 Craig Merry and contributors
# SPDX-License-Identifier: Apache-2.0
"""From a run directory to a signed submission (`be report`) and into a results repository
(`be submit`). docs/seams/submission.md describes both.

Modules: ``scrub`` (hostnames, usernames, home paths, IP addresses), ``files`` (deterministic
gzip, JPEG metadata, private-key detection), ``timebase`` (instrument clock to run timebase),
``labconfig`` (the lab's declared fields), ``report`` (`be report`), ``submit`` (`be submit`),
``cli`` (registers both). The kit never writes a score or a verdict: core's ``report.build``
derives them from the files.
"""
