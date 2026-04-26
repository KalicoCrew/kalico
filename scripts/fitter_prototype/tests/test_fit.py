from __future__ import annotations

import numpy as np

from scripts.fitter_prototype.fit import (
    build_basis_matrix,
    chord_length_parameterize,
    lspia_fit,
    make_clamped_knot_vector,
)
from scripts.fitter_prototype.params import FitterParams


def test_chord_length_parameterize():
    pts = np.array([[0.0, 0.0], [3.0, 0.0], [3.0, 4.0]])  # lengths 3, 4 → cum 0, 3, 7
    t = chord_length_parameterize(pts)
    np.testing.assert_allclose(t, [0.0, 3.0 / 7.0, 1.0])


def test_clamped_knot_vector_shape():
    knots = make_clamped_knot_vector(degree=3, n_interior=2)
    # Expected: [0,0,0,0, k1,k2, 1,1,1,1] — len = 4 + 2 + 4 = 10
    assert len(knots) == 10
    assert (knots[:4] == 0.0).all()
    assert (knots[-4:] == 1.0).all()
    assert knots[4] == 1.0 / 3.0  # uniform interior placement
    assert knots[5] == 2.0 / 3.0


def test_basis_matrix_partition_of_unity():
    knots = make_clamped_knot_vector(degree=3, n_interior=2)
    n_control = len(knots) - 3 - 1
    t = np.linspace(0.0, 1.0, 11)
    B = build_basis_matrix(t, knots, degree=3, n_control=n_control)
    # Each row sums to 1 (partition of unity).
    np.testing.assert_allclose(B.sum(axis=1), np.ones(11), atol=1e-9)


def test_lspia_fits_a_straight_line_exactly():
    # Sample a line — LSPIA should recover it within numerical noise.
    pts = np.array([[i * 0.5, i * 0.5] for i in range(20)])
    cps, knots, t = lspia_fit(pts, FitterParams())
    from scipy.interpolate import BSpline
    spline = BSpline(knots, cps, FitterParams().degree, extrapolate=False)
    eval_pts = spline(t)
    residuals = np.linalg.norm(eval_pts - pts, axis=1)
    assert residuals.max() < 1e-6


def test_lspia_fits_a_circle_within_tolerance():
    # Sample a unit circle quadrant.
    angles = np.linspace(0.0, np.pi / 2, 30)
    pts = np.column_stack([np.cos(angles), np.sin(angles)])
    params = FitterParams(n_init_interior=6)
    cps, knots, t = lspia_fit(pts, params)
    from scipy.interpolate import BSpline
    spline = BSpline(knots, cps, params.degree, extrapolate=False)
    residuals = np.linalg.norm(spline(t) - pts, axis=1)
    # Cubic NURBS approximation of a circle quadrant: realistic floor ~1e-3.
    assert residuals.max() < 5e-3
