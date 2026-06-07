from __future__ import annotations

import numpy as np

from scripts.fitter_prototype.fit import (
    build_basis_matrix,
    chord_length_parameterize,
    fit_smooth_run,
    lspia_fit,
    make_clamped_knot_vector,
    measure_chord_error_per_piece,
)
from scripts.fitter_prototype.params import FitterParams


def test_chord_length_parameterize():
    pts = np.array([[0.0, 0.0], [3.0, 0.0], [3.0, 4.0]])
    t = chord_length_parameterize(pts)
    np.testing.assert_allclose(t, [0.0, 3.0 / 7.0, 1.0])


def test_clamped_knot_vector_shape():
    knots = make_clamped_knot_vector(degree=3, n_interior=2)
    assert len(knots) == 10
    assert (knots[:4] == 0.0).all()
    assert (knots[-4:] == 1.0).all()
    assert knots[4] == 1.0 / 3.0
    assert knots[5] == 2.0 / 3.0


def test_basis_matrix_partition_of_unity():
    knots = make_clamped_knot_vector(degree=3, n_interior=2)
    n_control = len(knots) - 3 - 1
    t = np.linspace(0.0, 1.0, 11)
    B = build_basis_matrix(t, knots, degree=3, n_control=n_control)
    np.testing.assert_allclose(B.sum(axis=1), np.ones(11), atol=1e-9)


def test_lspia_fits_a_straight_line_exactly():
    pts = np.array([[i * 0.5, i * 0.5] for i in range(20)])
    cps, knots, t = lspia_fit(pts, FitterParams())
    from scipy.interpolate import BSpline

    spline = BSpline(knots, cps, FitterParams().degree, extrapolate=False)
    eval_pts = spline(t)
    residuals = np.linalg.norm(eval_pts - pts, axis=1)
    assert residuals.max() < 1e-6


def test_lspia_fits_a_circle_within_tolerance():
    angles = np.linspace(0.0, np.pi / 2, 30)
    pts = np.column_stack([np.cos(angles), np.sin(angles)])
    params = FitterParams(n_init_interior=6)
    cps, knots, t = lspia_fit(pts, params)
    from scipy.interpolate import BSpline

    spline = BSpline(knots, cps, params.degree, extrapolate=False)
    residuals = np.linalg.norm(spline(t) - pts, axis=1)
    assert residuals.max() < 5e-3


def test_fit_smooth_run_returns_fitted_nurbs_within_tolerance():
    angles = np.linspace(0.0, np.pi, 60)
    pts = np.column_stack([np.cos(angles), np.sin(angles)])
    params = FitterParams(eps_chord_mm=0.025, max_refine_iter=20)
    fit = fit_smooth_run(pts, source_vertex_range=(0, 60), params=params)
    assert fit.max_residual <= params.eps_chord_mm * 1.05


def test_chord_error_decreases_with_refinement():
    angles = np.linspace(0.0, np.pi, 60)
    pts = np.column_stack([np.cos(angles), np.sin(angles)])
    params_loose = FitterParams(eps_chord_mm=0.05, max_refine_iter=20)
    params_tight = FitterParams(eps_chord_mm=0.001, max_refine_iter=30)
    fit_loose = fit_smooth_run(pts, (0, 60), params_loose)
    fit_tight = fit_smooth_run(pts, (0, 60), params_tight)
    assert fit_tight.max_residual < fit_loose.max_residual
    assert len(fit_tight.knots) > len(fit_loose.knots)


def test_measure_chord_error_basic():
    pts = np.array([[i * 0.5, i * 0.5] for i in range(20)])
    params = FitterParams()
    cps, knots, t = lspia_fit(pts, params)
    errors = measure_chord_error_per_piece(
        cps, knots, params.degree, params.n_chord_samples
    )
    assert max(errors) < 1e-6
