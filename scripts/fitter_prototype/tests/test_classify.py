from __future__ import annotations

import numpy as np

from scripts.fitter_prototype.classify import (
    VertexLabel,
    classify_polyline,
)
from scripts.fitter_prototype.params import FitterParams


def test_straight_polyline_all_smooth():
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
    labels = classify_polyline(pts, FitterParams())
    # 2 interior vertices, both smooth.
    assert labels == [VertexLabel.SMOOTH, VertexLabel.SMOOTH]


def test_sharp_corner_classified_hard():
    # Right-angle turn: 90° change.
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    labels = classify_polyline(pts, FitterParams())
    assert labels == [VertexLabel.HARD_CORNER]


def test_gentle_corner_smoothable():
    # 30° change — between θ_smooth=15° and θ_hard=60°.
    pts = np.array(
        [
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0 + np.cos(np.deg2rad(30)), np.sin(np.deg2rad(30))],
        ]
    )
    labels = classify_polyline(pts, FitterParams())
    assert labels == [VertexLabel.SMOOTHABLE_CORNER]


def test_below_smooth_threshold_is_smooth():
    # 5° change — below θ_smooth=15°.
    pts = np.array(
        [
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0 + np.cos(np.deg2rad(5)), np.sin(np.deg2rad(5))],
        ]
    )
    labels = classify_polyline(pts, FitterParams())
    assert labels == [VertexLabel.SMOOTH]


def test_short_polyline_no_interior():
    pts = np.array([[0.0, 0.0], [1.0, 0.0]])
    labels = classify_polyline(pts, FitterParams())
    assert labels == []


def test_zero_length_segment_treated_as_smooth():
    # Repeated point — degenerate segment. Classifier shouldn't crash.
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    labels = classify_polyline(pts, FitterParams())
    assert len(labels) == 2
    # Behavior: zero-length adjacent segment → angle undefined; label as smooth.
    assert all(label == VertexLabel.SMOOTH for label in labels)
