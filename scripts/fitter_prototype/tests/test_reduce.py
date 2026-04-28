from __future__ import annotations

import numpy as np

from scripts.fitter_prototype.parser import Arc, Marker, Move
from scripts.fitter_prototype.reduce import (
    Polyline,
    reduce_tokens,
)


def test_simple_polyline():
    tokens = [
        Move("G1", 0.0, 0.0, 1),
        Move("G1", 10.0, 0.0, 2),
        Move("G1", 10.0, 10.0, 3),
    ]
    segs = reduce_tokens(tokens)
    assert len(segs) == 1
    assert isinstance(segs[0], Polyline)
    np.testing.assert_array_equal(
        segs[0].points,
        np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]]),
    )


def test_marker_splits_polyline():
    tokens = [
        Move("G1", 0.0, 0.0, 1),
        Move("G1", 10.0, 0.0, 2),
        Marker("G0", 3),
        Move("G1", 50.0, 50.0, 4),
        Move("G1", 60.0, 60.0, 5),
    ]
    segs = reduce_tokens(tokens)
    polylines = [s for s in segs if isinstance(s, Polyline)]
    assert len(polylines) == 2
    assert polylines[0].points.shape == (2, 2)
    assert polylines[1].points.shape == (2, 2)


def test_arc_passthrough_breaks_polyline():
    tokens = [
        Move("G1", 0.0, 0.0, 1),
        Move("G1", 10.0, 0.0, 2),
        Arc("G2", 10.0, 10.0, 0.0, 5.0, 3),
        Move("G1", 0.0, 10.0, 4),
    ]
    segs = reduce_tokens(tokens)
    assert [type(s).__name__ for s in segs] == [
        "Polyline",
        "ArcSegment",
        "Polyline",
    ]
    arc = segs[1]
    np.testing.assert_array_equal(arc.start, [10.0, 0.0])
    np.testing.assert_array_equal(arc.end, [10.0, 10.0])
    np.testing.assert_array_equal(arc.center, [10.0, 5.0])
    assert arc.clockwise is True


def test_carry_forward_missing_xy():
    tokens = [
        Move("G1", 0.0, 0.0, 1),
        Move("G1", 10.0, None, 2),  # Y carries forward as 0
        Move("G1", None, 10.0, 3),  # X carries forward as 10
    ]
    segs = reduce_tokens(tokens)
    poly = segs[0]
    np.testing.assert_array_equal(
        poly.points,
        np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]]),
    )


def test_zero_or_one_point_polyline_is_dropped():
    tokens = [
        Move("G1", 0.0, 0.0, 1),
        Marker("G0", 2),
        Move("G1", 5.0, 5.0, 3),
        Move("G1", 6.0, 6.0, 4),
    ]
    segs = reduce_tokens(tokens)
    polylines = [s for s in segs if isinstance(s, Polyline)]
    # First sub-polyline has 1 point only → dropped.
    # Second sub-polyline has 2 points → kept.
    assert len(polylines) == 1
    assert polylines[0].points.shape == (2, 2)
