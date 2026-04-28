from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Union

import numpy as np

from scripts.fitter_prototype.parser import Arc, Marker, Move, Token


@dataclass
class Polyline:
    points: np.ndarray  # shape (n, 2)
    line_range: tuple[int, int]


@dataclass
class ArcSegment:
    start: np.ndarray
    end: np.ndarray
    center: np.ndarray
    clockwise: bool
    line_no: int


GeometricSegment = Union[Polyline, ArcSegment]


def _flush_polyline(
    accum: list[tuple[float, float]],
    line_lo: Optional[int],
    line_hi: Optional[int],
    out: list[GeometricSegment],
) -> None:
    if len(accum) >= 2:
        out.append(
            Polyline(
                points=np.asarray(accum, dtype=float),
                line_range=(line_lo or 0, line_hi or 0),
            )
        )


def reduce_tokens(tokens: list[Token]) -> list[GeometricSegment]:
    out: list[GeometricSegment] = []
    accum: list[tuple[float, float]] = []
    line_lo: Optional[int] = None
    line_hi: Optional[int] = None
    cur_x: float = 0.0
    cur_y: float = 0.0
    last_was_arc: bool = False

    for tok in tokens:
        if isinstance(tok, Move):
            new_x = tok.x if tok.x is not None else cur_x
            new_y = tok.y if tok.y is not None else cur_y
            # If starting a new polyline after an arc, prepend the arc's
            # endpoint.
            if not accum and last_was_arc:
                accum.append((cur_x, cur_y))
                line_lo = tok.line_no
            elif not accum:
                line_lo = tok.line_no
            accum.append((new_x, new_y))
            line_hi = tok.line_no
            cur_x, cur_y = new_x, new_y
            last_was_arc = False
        elif isinstance(tok, Arc):
            _flush_polyline(accum, line_lo, line_hi, out)
            accum = []
            line_lo = line_hi = None
            new_x = tok.x if tok.x is not None else cur_x
            new_y = tok.y if tok.y is not None else cur_y
            i = tok.i if tok.i is not None else 0.0
            j = tok.j if tok.j is not None else 0.0
            center = np.array([cur_x + i, cur_y + j])
            out.append(
                ArcSegment(
                    start=np.array([cur_x, cur_y]),
                    end=np.array([new_x, new_y]),
                    center=center,
                    clockwise=(tok.kind == "G2"),
                    line_no=tok.line_no,
                )
            )
            cur_x, cur_y = new_x, new_y
            last_was_arc = True
        elif isinstance(tok, Marker):
            _flush_polyline(accum, line_lo, line_hi, out)
            accum = []
            line_lo = line_hi = None
            last_was_arc = False
            # Position does NOT update on a marker; G0 destinations are unknown
            # to the geometric pipeline by design.

    _flush_polyline(accum, line_lo, line_hi, out)
    return out
