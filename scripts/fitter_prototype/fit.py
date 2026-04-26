from __future__ import annotations

import numpy as np
from scipy.interpolate import BSpline

from scripts.fitter_prototype.output import FittedNurbs
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


def _unique_interior_breakpoints(
    knots: np.ndarray, degree: int
) -> np.ndarray:
    """Strictly interior breakpoints (not the clamped endpoints), with
    duplicates collapsed."""
    interior = knots[degree + 1 : -(degree + 1)]
    if len(interior) == 0:
        return interior
    return np.unique(interior)


def _piece_breakpoints(knots: np.ndarray, degree: int) -> np.ndarray:
    """Full breakpoint list including clamped start and end."""
    interior = _unique_interior_breakpoints(knots, degree)
    return np.concatenate(
        [[knots[degree]], interior, [knots[-degree - 1]]]
    )


def measure_chord_error_per_piece(
    cps: np.ndarray,
    knots: np.ndarray,
    degree: int,
    n_samples: int,
) -> list[float]:
    """For each piece between adjacent breakpoints, sample the curve and
    return max distance from sampled points to the chord between piece
    endpoints.

    This is a sample-based approximation of the analytical chord-bound.
    It overestimates a tiny bit — fine for prototype, replace with proper
    convex-hull bound when porting to Rust.
    """
    breakpoints = _piece_breakpoints(knots, degree)
    spline = BSpline(knots, cps, degree, extrapolate=False)
    errors: list[float] = []
    for k in range(len(breakpoints) - 1):
        t0, t1 = breakpoints[k], breakpoints[k + 1]
        ts = np.linspace(t0, t1, n_samples)
        pts = spline(ts)
        # Right-boundary safety.
        if np.any(np.isnan(pts[-1])):
            pts[-1] = cps[-1]
        chord_start, chord_end = pts[0], pts[-1]
        chord_vec = chord_end - chord_start
        chord_len = float(np.linalg.norm(chord_vec))
        if chord_len < 1e-12:
            errors.append(0.0)
            continue
        chord_dir = chord_vec / chord_len
        offsets = pts - chord_start
        parallel = (offsets @ chord_dir)[:, None] * chord_dir
        perp = offsets - parallel
        dists = np.linalg.norm(perp, axis=1)
        errors.append(float(dists.max()))
    return errors


def _worst_piece_param(
    cps: np.ndarray,
    knots: np.ndarray,
    degree: int,
    piece_idx: int,
    n_samples: int,
) -> float:
    breakpoints = _piece_breakpoints(knots, degree)
    t0, t1 = breakpoints[piece_idx], breakpoints[piece_idx + 1]
    ts = np.linspace(t0, t1, n_samples)
    spline = BSpline(knots, cps, degree, extrapolate=False)
    pts = spline(ts)
    if np.any(np.isnan(pts[-1])):
        pts[-1] = cps[-1]
    chord_start, chord_end = pts[0], pts[-1]
    chord_vec = chord_end - chord_start
    chord_len = float(np.linalg.norm(chord_vec))
    if chord_len < 1e-12:
        return float((t0 + t1) / 2.0)
    chord_dir = chord_vec / chord_len
    offsets = pts - chord_start
    parallel = (offsets @ chord_dir)[:, None] * chord_dir
    perp = offsets - parallel
    dists = np.linalg.norm(perp, axis=1)
    return float(ts[int(np.argmax(dists))])


def fit_smooth_run(
    points: np.ndarray,
    source_vertex_range: tuple[int, int],
    params: FitterParams,
) -> FittedNurbs:
    """LSPIA + chord-bound refinement."""
    cps, knots, t = lspia_fit(points, params)

    for _ in range(params.max_refine_iter):
        errors = measure_chord_error_per_piece(
            cps, knots, params.degree, params.n_chord_samples,
        )
        worst_err = max(errors) if errors else 0.0
        if worst_err <= params.eps_chord_mm:
            break
        worst_idx = int(np.argmax(errors))
        new_knot = _worst_piece_param(
            cps, knots, params.degree, worst_idx, params.n_chord_samples,
        )
        # Insert at the worst-residual parameter location.
        knots = np.sort(np.concatenate([knots, [new_knot]]))
        cps, knots, t = lspia_fit(points, params, knots_override=knots)

    return FittedNurbs(
        control_points=cps,
        knots=knots,
        degree=params.degree,
        source_vertex_range=source_vertex_range,
        max_residual=max_residual(cps, knots, params.degree, t, points),
    )
