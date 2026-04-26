from __future__ import annotations

import numpy as np
from scipy.interpolate import BSpline

from scripts.fitter_prototype.params import FitterParams


def chord_length_parameterize(points: np.ndarray) -> np.ndarray:
    """Cumulative chord-length parameterization, normalized to [0, 1]."""
    diffs = np.diff(points, axis=0)
    chord_lengths = np.linalg.norm(diffs, axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(chord_lengths)])
    total = cumulative[-1]
    if total < 1e-12:
        return np.zeros(len(points))
    return cumulative / total


def make_clamped_knot_vector(
    degree: int, n_interior: int
) -> np.ndarray:
    """Clamped knot vector on [0, 1] with `n_interior` uniformly-spaced
    interior knots. Total length = 2*(degree+1) + n_interior; total control
    points = degree + 1 + n_interior."""
    interior = np.linspace(0.0, 1.0, n_interior + 2)[1:-1]
    return np.concatenate([
        np.zeros(degree + 1),
        interior,
        np.ones(degree + 1),
    ])


def build_basis_matrix(
    t: np.ndarray,
    knots: np.ndarray,
    degree: int,
    n_control: int,
) -> np.ndarray:
    """B[i, j] = N_j(t_i). Uses scipy BSpline with one-hot control
    coefficients."""
    n_data = len(t)
    B = np.zeros((n_data, n_control))
    for j in range(n_control):
        c = np.zeros(n_control)
        c[j] = 1.0
        spline = BSpline(knots, c, degree, extrapolate=False)
        vals = spline(t)
        # BSpline returns NaN outside the parametric domain; clamp.
        B[:, j] = np.nan_to_num(vals, nan=0.0)
    # Boundary fix: BSpline's right-clamp at t = knots[-1] sometimes returns
    # zero everywhere; force partition of unity at the right endpoint.
    last_row_sum = B[-1].sum()
    if last_row_sum < 0.5:  # numerical hint that we hit the boundary issue
        B[-1, -1] = 1.0
    return B


def lspia_fit(
    points: np.ndarray,
    params: FitterParams,
    knots_override: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """LSPIA fit. Returns (control_points, knots, t_params).

    Bi 2019 §3 fixed-point iteration. Provably contracts to LSQ solution.
    """
    t = chord_length_parameterize(points)
    if knots_override is not None:
        knots = knots_override
        n_control = len(knots) - params.degree - 1
    else:
        knots = make_clamped_knot_vector(
            params.degree, params.n_init_interior
        )
        n_control = params.degree + 1 + params.n_init_interior
    B = build_basis_matrix(t, knots, params.degree, n_control)

    # Initial CP via LSQ.
    cps, *_ = np.linalg.lstsq(B, points, rcond=None)

    # Per-CP normalization for the LSPIA update.
    diag = (B * B).sum(axis=0)
    diag = np.where(diag < 1e-12, 1.0, diag)

    for _ in range(params.max_lspia_iter):
        residuals = points - B @ cps
        update = (B.T @ residuals) / diag[:, None]
        max_update = float(np.max(np.linalg.norm(update, axis=1)))
        cps = cps + update
        if max_update < params.eps_iter_mm:
            break

    return cps, knots, t


def evaluate_fit(
    cps: np.ndarray,
    knots: np.ndarray,
    degree: int,
    t: np.ndarray,
) -> np.ndarray:
    spline = BSpline(knots, cps, degree, extrapolate=False)
    vals = spline(t)
    # Right-boundary fix mirroring build_basis_matrix.
    if np.any(np.isnan(vals[-1])):
        vals[-1] = cps[-1]
    return vals


def max_residual(
    cps: np.ndarray,
    knots: np.ndarray,
    degree: int,
    t: np.ndarray,
    points: np.ndarray,
) -> float:
    eval_pts = evaluate_fit(cps, knots, degree, t)
    return float(np.linalg.norm(eval_pts - points, axis=1).max())
