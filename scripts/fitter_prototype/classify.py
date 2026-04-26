from __future__ import annotations

from enum import Enum

import numpy as np

from scripts.fitter_prototype.params import FitterParams


class VertexLabel(str, Enum):
    SMOOTH = "smooth"
    SMOOTHABLE_CORNER = "smoothable_corner"
    HARD_CORNER = "hard_corner"


def _angle_between(t_in: np.ndarray, t_out: np.ndarray) -> float:
    """Angle in degrees between two non-zero 2D vectors.

    Returns 0 if either is zero.
    """
    n_in = np.linalg.norm(t_in)
    n_out = np.linalg.norm(t_out)
    if n_in < 1e-12 or n_out < 1e-12:
        return 0.0
    cos = float(np.dot(t_in, t_out) / (n_in * n_out))
    cos = max(-1.0, min(1.0, cos))
    return float(np.degrees(np.arccos(cos)))


def classify_polyline(
    points: np.ndarray, params: FitterParams
) -> list[VertexLabel]:
    """Label each interior vertex of a polyline.

    Returns a list of length len(points) - 2 (no labels for endpoints).
    """
    labels: list[VertexLabel] = []
    n = len(points)
    if n < 3:
        return labels
    for i in range(1, n - 1):
        t_in = points[i] - points[i - 1]
        t_out = points[i + 1] - points[i]
        theta = _angle_between(t_in, t_out)
        if theta > params.theta_hard_deg:
            labels.append(VertexLabel.HARD_CORNER)
        elif theta > params.theta_smooth_deg:
            labels.append(VertexLabel.SMOOTHABLE_CORNER)
        else:
            labels.append(VertexLabel.SMOOTH)
    return labels
