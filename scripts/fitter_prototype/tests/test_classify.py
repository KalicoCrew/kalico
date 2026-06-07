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
    assert labels == [VertexLabel.SMOOTH, VertexLabel.SMOOTH]


def test_sharp_corner_classified_hard():
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    labels = classify_polyline(pts, FitterParams())
    assert labels == [VertexLabel.HARD_CORNER]


def test_gentle_corner_smoothable():
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
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    labels = classify_polyline(pts, FitterParams())
    assert len(labels) == 2
    assert all(label == VertexLabel.SMOOTH for label in labels)
