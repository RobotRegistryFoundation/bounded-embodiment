"""Envelope 0.2: the declared physical limits a test scores a machine against.

``load`` accepts envelope 0.1 (upgraded in memory) and 0.2 documents; see CONTRACT.md and
``schemas/envelope-0.2.json``. ``bounded_embodiment.envelope.convert`` turns other envelope formats
into 0.2 documents.
"""

from .load import load
from .model import Envelope, EnvelopeError, Point, Zone

__all__ = ["Envelope", "EnvelopeError", "Point", "Zone", "load"]
