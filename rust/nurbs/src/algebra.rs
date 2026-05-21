//! Algebraic operations on NURBS. Host-only.
//! See spec §algebra module.

use crate::bezier::binomial;
use crate::{AlgebraError, Float};

/// Multiply control points by a scalar. Weights, knots, degree unchanged.
#[cfg(feature = "host")]
pub fn scalar_multiply<T: Float>(
    curve: &crate::ScalarNurbs<T>,
    scalar: T,
) -> crate::ScalarNurbs<T> {
    let new_cps: Vec<T> = curve.control_points().iter().map(|c| *c * scalar).collect();
    let weights = curve.weights().map(<[T]>::to_vec);
    crate::ScalarNurbs::try_new(curve.degree(), curve.knots().to_vec(), new_cps, weights)
        .expect("scalar_multiply preserves invariants")
}

/// Add two scalar NURBS pointwise. v1 requires identical degree and identical
/// knot vectors; mismatched cases return `KnotMismatch` and the caller is
/// expected to align via knot insertion (follow-up implementation).
///
/// Weights: v1 supports unweighted-only. Weighted addition is non-trivial
/// (requires homogeneous lift) and is deferred to a follow-up spec.
#[cfg(feature = "host")]
pub fn add<T: Float>(
    a: &crate::ScalarNurbs<T>,
    b: &crate::ScalarNurbs<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    if a.degree() != b.degree() {
        return Err(AlgebraError::KnotMismatch);
    }
    if a.knots() != b.knots() {
        return Err(AlgebraError::KnotMismatch);
    }
    if a.weights().is_some() || b.weights().is_some() {
        return Err(AlgebraError::NotImplemented(
            "weighted add — homogeneous lift required",
        ));
    }
    let new_cps: Vec<T> = a
        .control_points()
        .iter()
        .zip(b.control_points().iter())
        .map(|(x, y)| *x + *y)
        .collect();
    crate::ScalarNurbs::try_new(a.degree(), a.knots().to_vec(), new_cps, None)
        .map_err(|_| AlgebraError::KnotMismatch)
}

/// Add two scalar NURBS curves pointwise, first aligning their knot vectors
/// via a Bézier-piece union.
///
/// Unlike [`add`], this function does not require `a` and `b` to share a knot
/// vector — it extracts both as Bézier pieces, refines each to the union of
/// breakpoints (exact knot insertion, no approximation), then adds the control
/// points piece-by-piece and recomposites the result.
///
/// **Fast path:** when `a.knots() == b.knots()` the function delegates directly
/// to [`add`] without going through extract → refine → recompose.
///
/// # Errors
///
/// - `AlgebraError::KnotMismatch` — degrees differ, or either curve has no pieces.
/// - `AlgebraError::SupportMismatch` — the two curves span different parameter domains.
/// - `AlgebraError::NotImplemented` — either operand carries non-unit weights (weighted
///   addition requires a homogeneous lift and is deferred).
///
/// # Panics
///
/// After a successful knot-union pass the internal [`add`] call is guaranteed to
/// succeed; if it returns `Err` despite the union (bridge invariant violation), this
/// function panics with a diagnostic.
///
/// # Example
///
/// ```rust
/// # use nurbs::algebra::add_with_knot_union;
/// # use nurbs::ScalarNurbs;
/// // X: two Bézier pieces. Y: one piece.
/// let x = ScalarNurbs::try_new(
///     1,
///     vec![0.0_f64, 0.0, 0.5, 1.0, 1.0],
///     vec![0.0, 5.0, 10.0],
///     None,
/// ).unwrap();
/// let y = ScalarNurbs::try_new(
///     1,
///     vec![0.0_f64, 0.0, 1.0, 1.0],
///     vec![20.0, 20.0],
///     None,
/// ).unwrap();
/// let sum = add_with_knot_union(&x, &y).unwrap();
/// // At u=0: 0+20=20; at u=1: 10+20=30.
/// let v0 = nurbs::eval::eval(&sum.as_view(), 0.0_f64);
/// let v1 = nurbs::eval::eval(&sum.as_view(), 1.0_f64);
/// assert!((v0 - 20.0).abs() < 1e-12);
/// assert!((v1 - 30.0).abs() < 1e-12);
/// ```
#[cfg(feature = "host")]
pub fn add_with_knot_union<T: Float>(
    a: &crate::ScalarNurbs<T>,
    b: &crate::ScalarNurbs<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    if a.degree() != b.degree() {
        return Err(AlgebraError::KnotMismatch);
    }
    if a.weights().is_some() || b.weights().is_some() {
        return Err(AlgebraError::NotImplemented(
            "add_with_knot_union weighted — homogeneous lift required",
        ));
    }

    // Fast path: already compatible — delegate directly.
    if a.knots() == b.knots() {
        return add(a, b);
    }

    let a_pieces = crate::bezier::extract_bezier_pieces(a);
    let b_pieces = crate::bezier::extract_bezier_pieces(b);

    if a_pieces.is_empty() || b_pieces.is_empty() {
        return Err(AlgebraError::KnotMismatch);
    }

    // Verify both curves span the same parameter domain.
    let a_start = a_pieces[0].u_start;
    let a_end = a_pieces[a_pieces.len() - 1].u_end;
    let b_start = b_pieces[0].u_start;
    let b_end = b_pieces[b_pieces.len() - 1].u_end;
    // Tolerance chosen to be robust against rounding in knot construction
    // while still catching genuinely mismatched segments.
    let domain_tol = T::from_f64(1e-12);
    if (a_start - b_start).abs() > domain_tol || (a_end - b_end).abs() > domain_tol {
        return Err(AlgebraError::SupportMismatch);
    }

    // Union of breakpoints: sorted, deduplicated by T::total_cmp (no float tolerance
    // — exact representation is guaranteed by knot construction).
    let breakpoints = union_breakpoints(&a_pieces, &b_pieces);
    let a_refined = refine_pieces_to_breakpoints(&a_pieces, &breakpoints);
    let b_refined = refine_pieces_to_breakpoints(&b_pieces, &breakpoints);

    // These invariants hold by construction of union_breakpoints +
    // refine_pieces_to_breakpoints: the union visit produces one output
    // piece per input piece boundary, and both piece lists are refined
    // against the same breakpoint set. Use release-active asserts so a
    // future refine_pieces_to_breakpoints regression surfaces immediately
    // rather than silently zip-truncating.
    assert_eq!(
        a_refined.len(),
        b_refined.len(),
        "add_with_knot_union: refine produced mismatched piece counts \
         (a_refined={}, b_refined={}); this is an internal invariant violation",
        a_refined.len(),
        b_refined.len(),
    );

    // Add control points piece-by-piece. Piece count and CP count per piece
    // match by the invariant above.
    let sum_pieces: Vec<crate::bezier::BezierPiece<T>> = a_refined
        .iter()
        .zip(b_refined.iter())
        .map(|(ap, bp)| {
            assert_eq!(
                ap.coeffs.len(),
                bp.coeffs.len(),
                "add_with_knot_union: CP count mismatch after union refine \
                 (ap.coeffs={}, bp.coeffs={})",
                ap.coeffs.len(),
                bp.coeffs.len(),
            );
            let coeffs: Vec<T> = ap
                .coeffs
                .iter()
                .zip(bp.coeffs.iter())
                .map(|(ac, bc)| *ac + *bc)
                .collect();
            crate::bezier::BezierPiece {
                u_start: ap.u_start,
                u_end: ap.u_end,
                coeffs,
            }
        })
        .collect();

    Ok(crate::bezier::bezier_pieces_to_nurbs(&sum_pieces))
}

/// Error from `fit_x_to_arc_length_piece`. Distinct from `AlgebraError` —
/// fit failure is a recoverable signal to the caller (split + recurse), not
/// a planner-level invariant violation.
#[cfg(feature = "host")]
#[derive(Debug, Clone, PartialEq)]
pub enum FitError {
    /// Reached `max_degree` without satisfying tolerance — caller should split
    /// the piece (recurse with two halves) or return a hard planner error if at
    /// `max_recursion_depth`.
    ToleranceNotReached { achieved_mm: f64, at_degree: u8 },
    /// Pathological input — table inversion or geometry evaluation failed.
    DegenerateInput { reason: &'static str },
}

/// Adaptive polynomial fit of a vector NURBS path `geometry` reparameterized
/// by arc length `s ∈ [s_lo, s_hi]`. Per axis, returns a single Bézier piece
/// (Pascal-shifted-monomial basis at `u_start = s_lo`) of degree `d`, where
/// `d` is the smallest integer in `[target_degree, max_degree]` for which the
/// L∞ residual at `4·(d+1)` uniform sample points is `≤ tolerance_mm`.
///
/// **Algorithm (per spec §4.5):**
/// 1. Generate `d+1` Chebyshev-of-the-second-kind nodes in `[s_lo, s_hi]`
///    (these include the endpoints by construction).
/// 2. Query `u(s)` from the arc-length table and evaluate `geometry(u)` at
///    each node.
/// 3. Solve Lagrange interpolation per axis on the Pascal-shifted-monomial
///    Vandermonde matrix `A[i][j] = (s_nodes[i] − s_lo)^j` via Gauss
///    elimination with partial pivoting.
/// 4. Verify L∞ residual at `4·(d+1)` uniform samples; on failure bump `d`
///    and retry until `d == max_degree`.
///
/// Returns `FitError::ToleranceNotReached` if convergence fails by `max_degree`
/// (caller's responsibility to split + recurse), or `FitError::DegenerateInput`
/// for `s_hi ≤ s_lo`, non-finite endpoints, or `target_degree > max_degree`.
#[cfg(feature = "host")]
pub fn fit_x_to_arc_length_piece<const D: usize>(
    geometry: &crate::VectorNurbs<f64, D>,
    table: &crate::ArcLengthTableRef<'_, f64>,
    s_lo: f64,
    s_hi: f64,
    target_degree: u8,
    max_degree: u8,
    tolerance_mm: f64,
) -> Result<[crate::bezier::BezierPiece<f64>; D], FitError>
where
    [(); D]:,
{
    // Tiny ULP tolerance for endpoint queries the caller may construct via
    // arc_length_from_param round-trips. Larger violations indicate a stale
    // arc-length table or off-by-one grid range — fail closed.
    const RANGE_EPS: f64 = 1e-9;

    // Up-front guards.
    if !s_lo.is_finite() || !s_hi.is_finite() {
        return Err(FitError::DegenerateInput {
            reason: "s_lo or s_hi is non-finite",
        });
    }
    if s_hi <= s_lo {
        return Err(FitError::DegenerateInput {
            reason: "s_hi <= s_lo",
        });
    }
    if target_degree > max_degree {
        return Err(FitError::DegenerateInput {
            reason: "target_degree > max_degree",
        });
    }

    let s_max = table.s_max();
    if s_lo < -RANGE_EPS || s_hi > s_max + RANGE_EPS {
        return Err(FitError::DegenerateInput {
            reason: "s_lo/s_hi out of arc-length table range",
        });
    }

    let mut d = target_degree;
    loop {
        let d_usize = d as usize;
        let n_nodes = d_usize + 1;

        // Step 1: Chebyshev-of-the-second-kind nodes in [s_lo, s_hi].
        // s_i = (s_lo + s_hi)/2 + (s_hi - s_lo)/2 * cos(i * π / d) for i = 0..=d.
        // For d == 0 we have a single node at the midpoint (the cos is undefined
        // when d=0 since i*π/0 is indeterminate). Practically a degree-0 fit is a
        // constant; we fix it at the midpoint.
        let mid = 0.5 * (s_lo + s_hi);
        let half = 0.5 * (s_hi - s_lo);
        let mut s_nodes: Vec<f64> = Vec::with_capacity(n_nodes);
        if d == 0 {
            s_nodes.push(mid);
        } else {
            for i in 0..=d_usize {
                let angle = (i as f64) * std::f64::consts::PI / f64::from(d);
                // i=0 → cos(0)=1 → s_lo+halflen-actually ordering: i=0 yields s_hi side.
                // Per spec: s_i = mid + half * cos(i π / d). i=0 gives mid + half = s_hi;
                // i=d gives mid - half = s_lo. Order is s_hi → s_lo descending.
                s_nodes.push(mid + half * angle.cos());
            }
        }

        // Step 2: query u(s) and evaluate geometry(u) at each node.
        let mut samples: Vec<[f64; D]> = Vec::with_capacity(n_nodes);
        for &s in &s_nodes {
            // Clamp s into the table's valid range; at s_lo / s_hi the Chebyshev
            // formula can produce values that round to ±1ULP outside [s_lo, s_hi]
            // due to cos(π) ≠ −1 exactly, so the table-lookup clamp is load-bearing.
            let s_clamped = s.clamp(0.0, table.s_max());
            let u = crate::arc_length::param_from_arc_length(table, s_clamped);
            let x = crate::eval::vector_eval(geometry, u);
            samples.push(x);
        }

        // Step 3: Lagrange interpolation, Pascal-shifted basis at s_lo.
        let coeffs_per_axis = lagrange_interpolation_pascal_shifted::<D>(&s_nodes, &samples, s_lo);

        // Step 4: verify L∞ residual at 4·(d+1) uniform samples.
        let n_check = 4 * n_nodes;
        let mut max_err = 0.0_f64;
        for i in 0..=n_check {
            let t = (i as f64) / (n_check as f64);
            let s = s_lo + (s_hi - s_lo) * t;
            let s_clamped = s.clamp(0.0, table.s_max());
            let u = crate::arc_length::param_from_arc_length(table, s_clamped);
            let truth = crate::eval::vector_eval(geometry, u);
            for axis in 0..D {
                let p_val = horner_pascal_shifted(&coeffs_per_axis[axis], s, s_lo);
                let err = (truth[axis] - p_val).abs();
                if err > max_err {
                    max_err = err;
                }
            }
        }

        if max_err <= tolerance_mm {
            // Pack each axis's coefficients into a BezierPiece.
            let pieces_vec: Vec<crate::bezier::BezierPiece<f64>> = (0..D)
                .map(|axis| crate::bezier::BezierPiece {
                    u_start: s_lo,
                    u_end: s_hi,
                    coeffs: coeffs_per_axis[axis].clone(),
                })
                .collect();
            return pieces_vec.try_into().map_err(|_: Vec<_>| {
                // Unreachable: built exactly D pieces.
                FitError::DegenerateInput {
                    reason: "fit_x_to_arc_length_piece: array length mismatch (unreachable)",
                }
            });
        }

        if d >= max_degree {
            return Err(FitError::ToleranceNotReached {
                achieved_mm: max_err,
                at_degree: d,
            });
        }
        d += 1;
    }
}

/// Merge a sequence of exact polynomial pieces into fewer, lower-degree pieces
/// with C¹ continuity at boundaries and bounded L∞ position error.
///
/// **Use case:** TOPP-RA produces ~25 grid pieces per segment. After composition
/// with geometry, each piece is degree 6. This function reduces to ~5-15
/// degree-`target_degree` pieces at ≤ `tolerance_mm` error, preserving velocity
/// (C¹) continuity at output boundaries. Fewer pieces = lower-degree output after
/// convolution = cheaper MCU evaluation.
///
/// **Algorithm (merge-and-bisect):**
/// 1. Start by trying to merge ALL input pieces into a single output piece.
/// 2. For a candidate merge region `[u_lo, u_hi]` covering input pieces `i..j`:
///    a. Fit a degree-`target_degree` Hermite polynomial matching position and
///       velocity at both endpoints (4 constraints; free DOFs set to 0 for MVP).
///    b. Check L∞ residual at `4*(target_degree+1)` uniform sample points.
///    c. Accept if residual ≤ tolerance; otherwise bisect at the midpoint input
///       piece boundary and recursively fit each half.
///
/// Merge boundaries are axis-independent (shared across all D axes); the WORST
/// axis residual drives accept/bisect decisions.
///
/// Returns per-axis `Vec<BezierPiece<f64>>`, or `FitError` if tolerance cannot be
/// met (when a single input piece already exceeds tolerance).
#[cfg(feature = "host")]
pub fn fit_hermite_c1<const D: usize>(
    pieces: &[[crate::bezier::BezierPiece<f64>; D]],
    tolerance_mm: f64,
    target_degree: u8,
) -> Result<[Vec<crate::bezier::BezierPiece<f64>>; D], FitError> {
    if pieces.is_empty() {
        return Err(FitError::DegenerateInput {
            reason: "fit_hermite_c1: empty input",
        });
    }
    if !tolerance_mm.is_finite() || tolerance_mm <= 0.0 {
        return Err(FitError::DegenerateInput {
            reason: "fit_hermite_c1: tolerance must be finite and positive",
        });
    }
    if target_degree < 3 {
        return Err(FitError::DegenerateInput {
            reason: "fit_hermite_c1: target_degree must be >= 3 (need 4 Hermite constraints)",
        });
    }

    // Validate contiguity: pieces[i][axis].u_end == pieces[i+1][axis].u_start for all axes.
    for w in pieces.windows(2) {
        for axis in 0..D {
            if (w[0][axis].u_end - w[1][axis].u_start).abs() > 1e-12 {
                return Err(FitError::DegenerateInput {
                    reason: "fit_hermite_c1: non-contiguous input pieces",
                });
            }
        }
    }

    // Recursive merge-and-bisect, producing output pieces with shared boundaries across axes.
    let mut result: [Vec<crate::bezier::BezierPiece<f64>>; D] = std::array::from_fn(|_| Vec::new());

    hermite_fit_recursive::<D>(
        pieces,
        0,
        pieces.len(),
        tolerance_mm,
        target_degree,
        &mut result,
    )?;

    Ok(result)
}

/// Recursive helper for `fit_hermite_c1`. Tries to fit `pieces[lo..hi]` into a
/// single degree-`target_degree` piece per axis. On failure, bisects at the
/// midpoint input piece boundary and recurses on each half.
#[cfg(feature = "host")]
fn hermite_fit_recursive<const D: usize>(
    pieces: &[[crate::bezier::BezierPiece<f64>; D]],
    lo: usize,
    hi: usize,
    tolerance_mm: f64,
    target_degree: u8,
    result: &mut [Vec<crate::bezier::BezierPiece<f64>>; D],
) -> Result<(), FitError> {
    debug_assert!(lo < hi);

    let u_lo = pieces[lo][0].u_start;
    let u_hi = pieces[hi - 1][0].u_end;

    // Try fitting the entire range [lo, hi) into one output piece per axis.
    let mut candidate = hermite_fit_one_piece::<D>(pieces, lo, hi, target_degree);
    let max_residual = hermite_check_residual::<D>(pieces, lo, hi, &candidate, target_degree);

    if max_residual <= tolerance_mm {
        // Snap output endpoints to the exact input boundary values so that
        // adjacent recursion branches produce bit-exactly contiguous pieces.
        // hermite_fit_one_piece computes u_end as `u_lo + h`, which can drift
        // by 1 ULP from the input's u_hi due to the subtract-then-add pattern;
        // bezier_pieces_to_nurbs's strict-equality contiguity assert catches
        // that drift. Snapping costs nothing (the polynomial still evaluates
        // correctly inside the interval).
        for axis in 0..D {
            candidate[axis].u_start = pieces[lo][axis].u_start;
            candidate[axis].u_end = pieces[hi - 1][axis].u_end;
            result[axis].push(candidate[axis].clone());
        }
        return Ok(());
    }

    // If we're down to a single input piece and it still doesn't fit, that's a
    // failure — can't bisect further.
    if hi - lo == 1 {
        return Err(FitError::ToleranceNotReached {
            achieved_mm: max_residual,
            at_degree: target_degree,
        });
    }

    // Bisect at the midpoint input piece boundary.
    let mid = lo + (hi - lo) / 2;
    let _ = (u_lo, u_hi); // suppress unused warnings

    hermite_fit_recursive::<D>(pieces, lo, mid, tolerance_mm, target_degree, result)?;
    hermite_fit_recursive::<D>(pieces, mid, hi, tolerance_mm, target_degree, result)?;

    Ok(())
}

/// Fit a single degree-`target_degree` Hermite polynomial to `pieces[lo..hi]` for
/// each axis, with the free DOF (c₂ for degree 4) optimized to minimize the maximum
/// residual across all axes.
///
/// For degree `d` with 4 Hermite constraints (position + velocity at both endpoints),
/// there are `d - 3` free DOFs. For the primary use case (d=4), there is exactly 1
/// free DOF (c₂). The residual at each sample point depends linearly on c₂, so the
/// optimal c₂ is found by a 1D Chebyshev minimax search.
///
/// For degree 3, the system is fully determined (no free DOFs). For degree > 4, only
/// c₂ is optimized; remaining free DOFs are set to 0.
#[cfg(feature = "host")]
fn hermite_fit_one_piece<const D: usize>(
    pieces: &[[crate::bezier::BezierPiece<f64>; D]],
    lo: usize,
    hi: usize,
    target_degree: u8,
) -> [crate::bezier::BezierPiece<f64>; D] {
    let u_lo = pieces[lo][0].u_start;
    let u_hi = pieces[hi - 1][0].u_end;
    let h = u_hi - u_lo;
    let d = target_degree as usize;

    // Collect endpoint constraints per axis.
    let constraints: Vec<(f64, f64, f64, f64)> = (0..D)
        .map(|axis| {
            let f_lo = pieces[lo][axis].evaluate(u_lo);
            let df_lo = pieces[lo][axis].differentiate().evaluate(u_lo);
            let f_hi = pieces[hi - 1][axis].evaluate(u_hi);
            let df_hi = pieces[hi - 1][axis].differentiate().evaluate(u_hi);
            (f_lo, df_lo, f_hi, df_hi)
        })
        .collect();

    // For degree 3 or degenerate h, no free DOF — just solve directly.
    if d <= 3 || h.abs() < 1e-300 {
        return std::array::from_fn(|axis| {
            let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
            hermite_construct_poly(f_lo, df_lo, f_hi, df_hi, u_lo, h, d, 0.0)
        });
    }

    // For degree >= 4: optimize the free DOF c₂ to minimize the maximum residual.
    //
    // The polynomial p(u; c₂) depends linearly on c₂ (see hermite_construct_poly).
    // So at each sample point: residual = ref(u) - p(u; c₂) = A - B·c₂
    // where A = ref(u) - p(u; 0) and B = p(u; 1) - p(u; 0).
    //
    // We want c₂ that minimizes max_over_all_axes_and_samples |A_i - B_i * c₂|.
    // This is a 1D Chebyshev minimax problem solvable by interval bisection.

    // Build sample points.
    let n_check = 4 * (d + 1);
    let mut sample_u: Vec<f64> = Vec::with_capacity(n_check + 1);
    let mut sample_piece_idx: Vec<usize> = Vec::with_capacity(n_check + 1);
    for i in 0..=n_check {
        let t = i as f64 / n_check as f64;
        let u = u_lo + (u_hi - u_lo) * t;
        sample_u.push(u);
        sample_piece_idx.push(hermite_find_piece_at(pieces, lo, hi, u));
    }

    // Build the two basis candidates: p(u; c₂=0) and p(u; c₂=1).
    let cand_0: Vec<crate::bezier::BezierPiece<f64>> = (0..D)
        .map(|axis| {
            let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
            hermite_construct_poly(f_lo, df_lo, f_hi, df_hi, u_lo, h, d, 0.0)
        })
        .collect();
    let cand_1: Vec<crate::bezier::BezierPiece<f64>> = (0..D)
        .map(|axis| {
            let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
            hermite_construct_poly(f_lo, df_lo, f_hi, df_hi, u_lo, h, d, 1.0)
        })
        .collect();

    // Compute (A_i, B_i) for each (sample, axis) pair.
    // residual = A_i - B_i * c₂
    let mut a_vals: Vec<f64> = Vec::new();
    let mut b_vals: Vec<f64> = Vec::new();
    for (si, &u) in sample_u.iter().enumerate() {
        let pidx = sample_piece_idx[si];
        for axis in 0..D {
            let ref_val = pieces[pidx][axis].evaluate(u);
            let p0 = cand_0[axis].evaluate(u);
            let p1 = cand_1[axis].evaluate(u);
            a_vals.push(ref_val - p0);
            b_vals.push(p1 - p0);
        }
    }

    // Find optimal c₂ by minimax: minimize max_i |a_i - b_i * c₂|.
    // This is equivalent to finding c₂ such that the envelope of lines
    // y_i(c₂) = a_i - b_i * c₂ has minimum max absolute value.
    //
    // Approach: iterate over candidate c₂ values where pairs of constraints
    // cross, i.e., where |a_i - b_i*c₂| = |a_j - b_j*c₂| and they have
    // opposite sign. For small sample counts, we can use a simple approach:
    // the optimal c₂ lies at a crossing of the upper and lower envelopes.
    let optimal_c2 = minimax_1d(&a_vals, &b_vals);

    std::array::from_fn(|axis| {
        let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
        hermite_construct_poly(f_lo, df_lo, f_hi, df_hi, u_lo, h, d, optimal_c2)
    })
}

/// Solve the 1D minimax problem: find `x` that minimizes `max_i |a_i - b_i * x|`.
///
/// The function `|a_i - b_i * x|` is V-shaped in `x` (minimum at `x = a_i/b_i`).
/// The max of V-shapes is piecewise-linear and convex, so the minimum is unique.
#[cfg(feature = "host")]
fn minimax_1d(a: &[f64], b: &[f64]) -> f64 {
    debug_assert_eq!(a.len(), b.len());

    // If all b_i are ~0, c₂ doesn't affect the residual; return 0.
    let max_b = b.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
    if max_b < 1e-30 {
        return 0.0;
    }

    // Evaluate max|a_i - b_i * x| at a given x.
    let eval_max_err = |x: f64| -> f64 {
        a.iter()
            .zip(b.iter())
            .map(|(&ai, &bi)| (ai - bi * x).abs())
            .fold(0.0_f64, f64::max)
    };

    // Collect all candidate x values where lines cross: x = (a_i - a_j) / (b_i - b_j)
    // and where individual lines cross zero: x = a_i / b_i.
    // The optimal x must be at one of these crossings (piecewise-linear convex function).
    let mut candidates: Vec<f64> = Vec::new();
    candidates.push(0.0);
    let n = a.len();
    for i in 0..n {
        if b[i].abs() > 1e-30 {
            candidates.push(a[i] / b[i]);
        }
    }
    // Also check crossings of pairs of upper/lower envelope lines.
    // For lines: y_i = a_i - b_i*x and y_j = -(a_j - b_j*x) = -a_j + b_j*x
    // crossing: a_i - b_i*x = -a_j + b_j*x => x = (a_i + a_j) / (b_i + b_j)
    for i in 0..n {
        for j in 0..n {
            let denom = b[i] + b[j];
            if denom.abs() > 1e-30 {
                candidates.push((a[i] + a[j]) / denom);
            }
            let denom2 = b[i] - b[j];
            if denom2.abs() > 1e-30 {
                candidates.push((a[i] - a[j]) / denom2);
            }
        }
    }

    // Find the candidate with minimum max error.
    let mut best_x = 0.0;
    let mut best_err = eval_max_err(0.0);
    for x in candidates {
        if !x.is_finite() {
            continue;
        }
        let err = eval_max_err(x);
        if err < best_err {
            best_err = err;
            best_x = x;
        }
    }

    best_x
}

/// Construct a single Hermite polynomial in Pascal-shifted basis at `u_lo`.
///
/// For degree `d` with constraints: `c₀ = f_lo`, `c₁ = df_lo` (position +
/// velocity at `u_lo`); `p(u_hi) = f_hi`, `p'(u_hi) = df_hi` (position +
/// velocity at `u_hi`).
///
/// `c2_val` is the value of the free DOF c₂ (used for degree >= 4).
/// For degree 3 (4 unknowns), the system is fully determined and `c2_val` is ignored.
#[cfg(feature = "host")]
#[allow(clippy::too_many_arguments, clippy::cast_possible_wrap)]
fn hermite_construct_poly(
    f_lo: f64,
    df_lo: f64,
    f_hi: f64,
    df_hi: f64,
    u_lo: f64,
    h: f64,
    d: usize,
    c2_val: f64,
) -> crate::bezier::BezierPiece<f64> {
    let mut coeffs = vec![0.0f64; d + 1];

    // c₀ = f_lo (position at start)
    coeffs[0] = f_lo;
    // c₁ = df_lo (velocity at start)
    coeffs[1] = df_lo;

    // Set the free DOF c₂ (only used for d >= 4).
    if d >= 4 {
        coeffs[2] = c2_val;
    }

    // Compute the position and derivative residuals after subtracting known terms.
    let mut pos_residual = f_hi - coeffs[0] - coeffs[1] * h;
    let mut vel_residual = df_hi - coeffs[1];

    // Subtract contributions from fixed coefficients c₂..c_{d-2}.
    let mut h_pow = h * h; // h^2
    let mut h_pow_deriv = h; // h^1 (for derivative: k*c_k*h^{k-1})
    for k in 2..d.saturating_sub(1) {
        pos_residual -= coeffs[k] * h_pow;
        vel_residual -= (k as f64) * coeffs[k] * h_pow_deriv;
        h_pow *= h;
        h_pow_deriv *= h;
    }

    // Solve the 2x2 system for c_{d-1}, c_d:
    //   c_{d-1} * h^{d-1} + c_d * h^d = pos_residual
    //   (d-1) * c_{d-1} * h^{d-2} + d * c_d * h^{d-1} = vel_residual
    //
    // Determinant = (d - (d-1)) * h^{2d-2} = h^{2d-2}
    let h_dm2 = h.powi(d as i32 - 2);
    let h_dm1 = h_dm2 * h;
    let h_d = h_dm1 * h;
    let det = h.powi(2 * d as i32 - 2);

    if det.abs() < 1e-300 {
        return crate::bezier::BezierPiece {
            u_start: u_lo,
            u_end: u_lo + h,
            coeffs,
        };
    }

    let d_f = d as f64;
    let dm1_f = (d - 1) as f64;
    let c_dm1 = (pos_residual * d_f * h_dm1 - h_d * vel_residual) / det;
    let c_d = (h_dm1 * vel_residual - dm1_f * h_dm2 * pos_residual) / det;

    coeffs[d - 1] = c_dm1;
    coeffs[d] = c_d;

    crate::bezier::BezierPiece {
        u_start: u_lo,
        u_end: u_lo + h,
        coeffs,
    }
}

/// Check the L∞ residual of a candidate fit against the reference (input pieces).
/// Returns the maximum residual across all axes.
#[cfg(feature = "host")]
fn hermite_check_residual<const D: usize>(
    pieces: &[[crate::bezier::BezierPiece<f64>; D]],
    lo: usize,
    hi: usize,
    candidate: &[crate::bezier::BezierPiece<f64>; D],
    target_degree: u8,
) -> f64 {
    let n_check = 4 * (target_degree as usize + 1);
    let u_lo = pieces[lo][0].u_start;
    let u_hi = pieces[hi - 1][0].u_end;
    let mut max_err = 0.0_f64;

    for i in 0..=n_check {
        let t = i as f64 / n_check as f64;
        let u = u_lo + (u_hi - u_lo) * t;
        let piece_idx = hermite_find_piece_at(pieces, lo, hi, u);

        for axis in 0..D {
            let ref_val = pieces[piece_idx][axis].evaluate(u);
            let fit_val = candidate[axis].evaluate(u);
            let err = (ref_val - fit_val).abs();
            if err > max_err {
                max_err = err;
            }
        }
    }

    max_err
}

/// Find which input piece index in `[lo, hi)` contains parameter `u`.
#[cfg(feature = "host")]
fn hermite_find_piece_at<const D: usize>(
    pieces: &[[crate::bezier::BezierPiece<f64>; D]],
    lo: usize,
    hi: usize,
    u: f64,
) -> usize {
    // Linear search — piece count is small (~25 max).
    for i in lo..hi {
        // Use axis 0 for domain queries (all axes share the same domain).
        if u <= pieces[i][0].u_end + 1e-12 {
            return i;
        }
    }
    // Clamp to last piece for numerical edge cases.
    hi - 1
}

/// Solve Lagrange interpolation per axis on the Pascal-shifted-monomial-basis
/// Vandermonde system `A[i][j] = (s_nodes[i] − s_origin)^j`, RHS = sample
/// per axis. Tiny matrix (max ~10×10); Gauss elimination with partial pivoting.
///
/// Returns `Vec<Vec<f64>>` of length `D`, each axis's coefficient vector
/// length = `s_nodes.len()`.
#[cfg(feature = "host")]
fn lagrange_interpolation_pascal_shifted<const D: usize>(
    s_nodes: &[f64],
    samples: &[[f64; D]],
    s_origin: f64,
) -> Vec<Vec<f64>> {
    let n = s_nodes.len();
    debug_assert_eq!(samples.len(), n);

    // Build Vandermonde augmented with D right-hand-side columns.
    // Layout: aug[i] = [A[i][0..n] | rhs[i][0..D]].
    let mut aug: Vec<Vec<f64>> = Vec::with_capacity(n);
    for i in 0..n {
        let mut row = Vec::with_capacity(n + D);
        let dx = s_nodes[i] - s_origin;
        let mut pow = 1.0;
        for _ in 0..n {
            row.push(pow);
            pow *= dx;
        }
        for axis in 0..D {
            row.push(samples[i][axis]);
        }
        aug.push(row);
    }

    // Gauss elimination with partial pivoting.
    for k in 0..n {
        // Find pivot row.
        let mut pivot = k;
        let mut pivot_abs = aug[k][k].abs();
        for i in (k + 1)..n {
            let v = aug[i][k].abs();
            if v > pivot_abs {
                pivot = i;
                pivot_abs = v;
            }
        }
        if pivot != k {
            aug.swap(k, pivot);
        }
        // If pivot is effectively zero we proceed anyway — degenerate node
        // pattern, will surface as a bad residual on verification.
        let pivot_val = aug[k][k];
        if pivot_val.abs() < 1e-300 {
            continue;
        }
        for i in (k + 1)..n {
            let factor = aug[i][k] / pivot_val;
            if factor == 0.0 {
                continue;
            }
            for j in k..(n + D) {
                aug[i][j] -= factor * aug[k][j];
            }
        }
    }

    // Back-substitution per RHS column.
    let mut out: Vec<Vec<f64>> = (0..D).map(|_| vec![0.0; n]).collect();
    for axis in 0..D {
        let rhs_col = n + axis;
        for k in (0..n).rev() {
            let mut sum = aug[k][rhs_col];
            for j in (k + 1)..n {
                sum -= aug[k][j] * out[axis][j];
            }
            let pivot_val = aug[k][k];
            if pivot_val.abs() < 1e-300 {
                out[axis][k] = 0.0;
            } else {
                out[axis][k] = sum / pivot_val;
            }
        }
    }
    out
}

/// Evaluate `Σ coeffs[k] * (s − s_origin)^k` via Horner's method.
#[cfg(feature = "host")]
fn horner_pascal_shifted(coeffs: &[f64], s: f64, s_origin: f64) -> f64 {
    let dx = s - s_origin;
    let mut acc = 0.0;
    for c in coeffs.iter().rev() {
        acc = acc * dx + *c;
    }
    acc
}

/// Polynomial kernel for convolution. Pieces are contiguous and ordered.
/// Each piece is a polynomial in the Pascal-shifted monomial basis.
#[cfg(feature = "host")]
#[derive(Debug, Clone)]
pub struct PiecewisePolynomialKernel<T: Float> {
    pub pieces: Vec<crate::bezier::BezierPiece<T>>,
}

#[cfg(feature = "host")]
impl<T: Float> PiecewisePolynomialKernel<T> {
    /// Build a single-piece kernel from monomial coefficients
    /// `coeffs[k] * (u - u_start)^k` on the interval `support`.
    pub fn single_poly(coeffs: Vec<T>, support: (T, T)) -> Self {
        let piece = crate::bezier::BezierPiece {
            u_start: support.0,
            u_end: support.1,
            coeffs,
        };
        Self {
            pieces: vec![piece],
        }
    }

    /// Build a single-piece kernel from coefficients in **absolute monomial basis**:
    /// `Σ coeffs[k] * u^k`. Internally converts to the Pascal-shifted-at-u_start basis
    /// that `single_poly` expects.
    ///
    /// Use this when your kernel coefficients come from sources that don't know
    /// about Bézier-piece basis conventions — e.g. Klipper's `init_smoother`,
    /// scipy / sympy expressions, or the bleeding-edge-v2 smooth-shaper polynomials.
    #[allow(clippy::needless_pass_by_value)] // API symmetry with `single_poly`
    pub fn single_poly_from_absolute(coeffs: Vec<T>, support: (T, T)) -> Self {
        let shifted = absolute_to_pascal_shift(&coeffs, support.0);
        Self::single_poly(shifted, support)
    }

    /// Total support of the kernel: from first piece's `u_start` to last piece's `u_end`.
    pub fn support(&self) -> (T, T) {
        (
            self.pieces.first().unwrap().u_start,
            self.pieces.last().unwrap().u_end,
        )
    }

    /// Build a multi-piece kernel from already-constructed pieces.
    /// Validates non-empty + contiguous (`pieces[i].u_end == pieces[i+1].u_start`).
    /// Returns `SupportMismatch` if pieces are non-contiguous.
    pub fn from_pieces(pieces: Vec<crate::bezier::BezierPiece<T>>) -> Result<Self, AlgebraError> {
        if pieces.is_empty() {
            return Err(AlgebraError::SupportMismatch);
        }
        for w in pieces.windows(2) {
            if w[0].u_end != w[1].u_start {
                return Err(AlgebraError::SupportMismatch);
            }
        }
        Ok(Self { pieces })
    }
}

/// Multiply two scalar NURBS pointwise: `c(u) = a(u) * b(u)`.
/// Result degree = `degree(a) + degree(b)`.
///
/// Polynomial inputs only in v1; rational inputs return `RationalNotSupported`.
#[cfg(feature = "host")]
pub fn multiply<T: Float>(
    a: &crate::ScalarNurbs<T>,
    b: &crate::ScalarNurbs<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    if a.weights().is_some() || b.weights().is_some() {
        return Err(AlgebraError::RationalNotSupported {
            operation: "multiply",
            workaround: "use polynomial_refit (Layer 3 utility) before calling",
        });
    }
    // Capture original interior knot multiplicities BEFORE Bezier extraction
    // lifts everything to full multiplicity. The post-pass needs the original
    // multiplicities to compute Mørken Eq. (1) target multiplicities for the
    // product; without this, an unbounded knot-removal can peel below the
    // natural multiplicity at a shared C⁰ kink and produce the wrong curve.
    let a_mults = collect_interior_multiplicities(a);
    let b_mults = collect_interior_multiplicities(b);

    let a_pieces = crate::bezier::extract_bezier_pieces(a);
    let b_pieces = crate::bezier::extract_bezier_pieces(b);

    // Refine to common breakpoint set.
    let breakpoints = union_breakpoints(&a_pieces, &b_pieces);
    let a_refined = refine_pieces_to_breakpoints(&a_pieces, &breakpoints);
    let b_refined = refine_pieces_to_breakpoints(&b_pieces, &breakpoints);
    debug_assert_eq!(a_refined.len(), b_refined.len());

    // Per-piece product.
    let mut out_pieces = Vec::with_capacity(a_refined.len());
    for (a_p, b_p) in a_refined.iter().zip(b_refined.iter()) {
        let coeffs = poly_multiply(&a_p.coeffs, &b_p.coeffs);
        out_pieces.push(crate::bezier::BezierPiece {
            u_start: a_p.u_start,
            u_end: a_p.u_end,
            coeffs,
        });
    }

    let mut result = crate::bezier::bezier_pieces_to_nurbs(&out_pieces);

    // Compute Mørken target multiplicity per interior breakpoint of the
    // product. For a breakpoint that is interior in only one factor, the
    // other factor's multiplicity is 0 — the breakpoint may still appear in
    // the product because Bezier extraction created it. The Mørken formula
    // with m=0 in one factor reduces to "degree of the other factor + m of
    // this factor", which is the natural multiplicity contributed by the
    // factor that has a real knot there.
    let d_a = a.degree() as usize;
    let d_b = b.degree() as usize;
    let p = result.degree() as usize;
    let interior_breakpoints = collect_interior_breakpoints(&result);
    let targets: Vec<(T, usize)> = interior_breakpoints
        .into_iter()
        .map(|u| {
            let m_a = a_mults
                .iter()
                .find(|(uu, _)| *uu == u)
                .map_or(0, |(_, m)| *m);
            let m_b = b_mults
                .iter()
                .find(|(uu, _)| *uu == u)
                .map_or(0, |(_, m)| *m);
            let target = morken_multiplicity(d_a, m_a, d_b, m_b);
            debug_assert!(
                target <= p,
                "Mørken target {target} exceeds product degree {p}"
            );
            (u, target)
        })
        .collect();

    knot_remove_to_morken_targets(&mut result, &targets, T::from_f64(1e-12));
    Ok(result)
}

/// Mørken Eq. (1) target multiplicity for a shared breakpoint in the product
/// of two polynomial NURBS. Returns 0 if the breakpoint isn't a knot of
/// either factor (which shouldn't happen for a real product breakpoint, but
/// is a safe no-op).
#[cfg(feature = "host")]
fn morken_multiplicity(d_a: usize, m_a: usize, d_b: usize, m_b: usize) -> usize {
    match (m_a > 0, m_b > 0) {
        (true, true) => (d_a + m_b).max(d_b + m_a),
        (false, true) => d_a + m_b,
        (true, false) => d_b + m_a,
        (false, false) => 0,
    }
}

/// Collect `(knot value, multiplicity)` for each unique INTERIOR knot value
/// (i.e., excluding the clamped endpoints).
#[cfg(feature = "host")]
fn collect_interior_multiplicities<T: Float>(curve: &crate::ScalarNurbs<T>) -> Vec<(T, usize)> {
    let p = curve.degree() as usize;
    let knots = curve.knots();
    if knots.len() <= 2 * (p + 1) {
        return Vec::new();
    }
    let interior_slice = &knots[p + 1..knots.len() - p - 1];
    let mut out: Vec<(T, usize)> = Vec::new();
    for &k in interior_slice {
        if let Some(entry) = out.iter_mut().find(|(u, _)| *u == k) {
            entry.1 += 1;
        } else {
            out.push((k, 1));
        }
    }
    out
}

/// List of unique interior knot values in `curve` (no multiplicities).
#[cfg(feature = "host")]
fn collect_interior_breakpoints<T: Float>(curve: &crate::ScalarNurbs<T>) -> Vec<T> {
    collect_interior_multiplicities(curve)
        .into_iter()
        .map(|(u, _)| u)
        .collect()
}

/// Reduce each interior knot's multiplicity to the Mørken target — never
/// below. `target_mults` maps breakpoint value to the target multiplicity
/// per Mørken Eq. (1). Tolerance is for the inner Tiller A5.8 numerical
/// check; if removal is rejected within tolerance for a knot that should
/// be removable per Mørken, that's a numerical-precision issue (widen tol
/// or accept the redundant knot — both are safer than peeling past the
/// target).
#[cfg(feature = "host")]
fn knot_remove_to_morken_targets<T: Float>(
    curve: &mut crate::ScalarNurbs<T>,
    target_mults: &[(T, usize)],
    tol: T,
) {
    for &(u, target) in target_mults {
        let current = curve.knots().iter().filter(|k| **k == u).count();
        if current > target {
            let n_to_remove = current - target;
            let (new_curve, _actually_removed) =
                crate::knot::remove_knot(curve, u, n_to_remove, tol);
            *curve = new_curve;
            // Note: we don't assert `_actually_removed == n_to_remove`. If the
            // inner chord-error check rejects a removal we'd want, the
            // redundant knot stays in place — wrong knot vector but correct
            // curve. A future Fix 2 (target-aware bezier_pieces_to_nurbs)
            // eliminates this case entirely.
        }
    }
}

/// Compute the union of distinct breakpoints from two piecewise representations.
#[cfg(feature = "host")]
fn union_breakpoints<T: Float>(
    a: &[crate::bezier::BezierPiece<T>],
    b: &[crate::bezier::BezierPiece<T>],
) -> Vec<T> {
    let mut breaks: Vec<T> = Vec::new();
    let push_unique = |u: T, breaks: &mut Vec<T>| {
        if !breaks.contains(&u) {
            breaks.push(u);
        }
    };
    for piece in a {
        push_unique(piece.u_start, &mut breaks);
        push_unique(piece.u_end, &mut breaks);
    }
    for piece in b {
        push_unique(piece.u_start, &mut breaks);
        push_unique(piece.u_end, &mut breaks);
    }
    breaks.sort_by(|x, y| T::total_cmp(*x, *y));
    breaks
}

/// Refine a list of contiguous Bézier pieces so that the piece boundaries
/// coincide with the given (sorted) breakpoints.
#[cfg(feature = "host")]
fn refine_pieces_to_breakpoints<T: Float>(
    pieces: &[crate::bezier::BezierPiece<T>],
    breakpoints: &[T],
) -> Vec<crate::bezier::BezierPiece<T>> {
    let mut result: Vec<crate::bezier::BezierPiece<T>> = Vec::new();
    for piece in pieces {
        let mut current = piece.clone();
        let mut interior: Vec<T> = breakpoints
            .iter()
            .filter(|&&b| b > current.u_start && b < current.u_end)
            .copied()
            .collect();
        interior.sort_by(|x, y| T::total_cmp(*x, *y));
        for u in interior {
            let (left, right) = crate::bezier::split_piece_at(&current, u);
            result.push(left);
            current = right;
        }
        result.push(current);
    }
    result
}

/// Convolve a polynomial NURBS with a piecewise polynomial kernel:
/// `y(u) = ∫ x(s) w(u - s) ds`.
///
/// Output domain = Minkowski sum of input and kernel supports. Caller
/// (Layer 3) handles cross-segment stitching for trajectories.
///
/// Polynomial inputs only in v1.
#[cfg(feature = "host")]
pub fn convolve<T: Float>(
    curve: &crate::ScalarNurbs<T>,
    kernel: &PiecewisePolynomialKernel<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    if curve.weights().is_some() {
        return Err(AlgebraError::RationalNotSupported {
            operation: "convolve",
            workaround: "use polynomial_refit (Layer 3 utility) before calling",
        });
    }
    let x_pieces = crate::bezier::extract_bezier_pieces(curve);
    let w_pieces = &kernel.pieces;

    // Compute output breakpoints: cross-sum of input and kernel breakpoints.
    let x_breaks: Vec<T> = {
        let mut v: Vec<T> = Vec::new();
        for p in &x_pieces {
            if !v.contains(&p.u_start) {
                v.push(p.u_start);
            }
        }
        v.push(x_pieces.last().unwrap().u_end);
        v
    };
    let w_breaks: Vec<T> = {
        let mut v: Vec<T> = Vec::new();
        for p in w_pieces {
            if !v.contains(&p.u_start) {
                v.push(p.u_start);
            }
        }
        v.push(w_pieces.last().unwrap().u_end);
        v
    };
    let mut out_breaks: Vec<T> = Vec::new();
    for xb in &x_breaks {
        for wb in &w_breaks {
            let s = *xb + *wb;
            if !out_breaks.contains(&s) {
                out_breaks.push(s);
            }
        }
    }
    out_breaks.sort_by(|a, b| T::total_cmp(*a, *b));

    let degree = x_pieces[0].degree() + w_pieces[0].degree() + 1;

    let mut out_pieces: Vec<crate::bezier::BezierPiece<T>> =
        Vec::with_capacity(out_breaks.len() - 1);
    for win in out_breaks.windows(2) {
        let alpha = win[0];
        let beta = win[1];
        let mut accum = crate::bezier::BezierPiece::<T>::zero(alpha, beta, degree);

        for x_p in &x_pieces {
            for w_p in w_pieces {
                let u_mid = (alpha + beta) * T::from_f64(0.5);
                let s_lo = (x_p.u_start).max(u_mid - w_p.u_end);
                let s_hi = (x_p.u_end).min(u_mid - w_p.u_start);
                if s_lo >= s_hi {
                    continue;
                }

                let contribution = integrate_product_piece(x_p, w_p, alpha, beta);
                accum = (&accum + &contribution).expect("same-support accumulation");
            }
        }
        out_pieces.push(accum);
    }

    let mut result = crate::bezier::bezier_pieces_to_nurbs(&out_pieces);
    knot_remove_redundant(&mut result, T::from_f64(1e-12));
    Ok(result)
}

/// Polynomial-of-polynomial composition for a fixed-axis-count vector outer
/// against a scalar inner. For each axis a, computes `outer_a(inner(t))` in
/// the Pascal-shifted monomial basis native to `BezierPiece`.
///
/// Result domain is `[inner.u_start, inner.u_end]`; output basis is shifted
/// at `inner.u_start`. Per-axis output degree = `outer.degree() × inner.degree()`.
///
/// **Algorithm (direct substitution-and-collect in monomial basis):**
/// 1. Build `shifted_inner = inner.coeffs` with `shifted_inner[0] -= outer_a.u_start`.
///    This is `inner(t) − outer_a.u_start` as a polynomial in `(t − inner.u_start)`,
///    which is exactly what we substitute for `(s − outer_a.u_start)` in `outer_a`.
/// 2. Build powers `shifted_inner^0 … shifted_inner^d_outer` via `poly_multiply`.
/// 3. Accumulate `result[k] = Σ_i outer_a.coeffs[i] × powers[i][k]`.
///
/// **Precondition (runtime-checked):** `outer[a].u_start == inner.evaluate(inner.u_start)`
/// and `outer[a].u_end == inner.evaluate(inner.u_end)` for every axis (within `1e-9`).
/// In other words, the inner's image must align with the outer's s-domain.
/// Violation returns `Err(AlgebraError::SupportMismatch)` in both debug and release builds.
///
/// Returns `Ok` with no work done if `D == 0`.
#[cfg(feature = "host")]
pub fn compose_vector_piece<const D: usize>(
    outer: &[&crate::bezier::BezierPiece<f64>; D],
    inner: &crate::bezier::BezierPiece<f64>,
) -> Result<[crate::bezier::BezierPiece<f64>; D], AlgebraError> {
    const ENDPOINT_TOL: f64 = 1e-9;
    let inner_at_start = inner.evaluate(inner.u_start);
    let inner_at_end = inner.evaluate(inner.u_end);
    for outer_axis in outer {
        if (outer_axis.u_start - inner_at_start).abs() > ENDPOINT_TOL
            || (outer_axis.u_end - inner_at_end).abs() > ENDPOINT_TOL
        {
            return Err(AlgebraError::SupportMismatch);
        }
    }

    let pieces: Vec<crate::bezier::BezierPiece<f64>> = outer
        .iter()
        .map(|outer_axis| {
            let d_outer = outer_axis.degree();

            // shifted_inner(t) = inner(t) - outer_axis.u_start, expressed in
            // basis (t - inner.u_start). Subtract from the constant term.
            let mut shifted_inner = inner.coeffs.clone();
            if shifted_inner.is_empty() {
                shifted_inner.push(-outer_axis.u_start);
            } else {
                shifted_inner[0] -= outer_axis.u_start;
            }

            // Build powers: powers[i] = shifted_inner^i in basis (t - inner.u_start).
            // powers[0] = [1] (the constant 1 polynomial).
            let mut powers: Vec<Vec<f64>> = Vec::with_capacity(d_outer + 1);
            powers.push(vec![1.0]);
            for i in 1..=d_outer {
                let next = poly_multiply(&powers[i - 1], &shifted_inner);
                powers.push(next);
            }

            // Result length = d_outer * d_inner + 1.
            let d_inner = inner.degree();
            let result_len = d_outer * d_inner + 1;
            let mut result_coeffs = vec![0.0_f64; result_len];
            for (i, c_outer) in outer_axis.coeffs.iter().enumerate() {
                let pow = &powers[i];
                for (k, p_k) in pow.iter().enumerate() {
                    result_coeffs[k] += *c_outer * *p_k;
                }
            }

            crate::bezier::BezierPiece {
                u_start: inner.u_start,
                u_end: inner.u_end,
                coeffs: result_coeffs,
            }
        })
        .collect();

    pieces.try_into().map_err(|_: Vec<_>| {
        // Unreachable: we built exactly D pieces from a D-length array.
        AlgebraError::NotImplemented("compose_vector_piece: array length mismatch (unreachable)")
    })
}

/// Polynomial coefficient convolution: out[k] = Σ_{i+j=k} a[i] * b[j].
#[cfg(feature = "host")]
fn poly_multiply<T: Float>(a: &[T], b: &[T]) -> Vec<T> {
    let mut out = vec![T::ZERO; a.len() + b.len() - 1];
    for (i, ai) in a.iter().enumerate() {
        for (j, bj) in b.iter().enumerate() {
            out[i + j] = out[i + j] + *ai * *bj;
        }
    }
    out
}

/// Integrate `∫ x(s) w(u - s) ds` over the (s, u) region where x's piece i and
/// w's piece j are simultaneously active, for u in `[α, β]`. Returns the
/// contribution as a `BezierPiece` on `[α, β]` with degree `d_x + d_w + 1`.
///
/// Algorithm sketch (per spec §6.4):
/// 1. Re-express `w(u-s)` in s-basis with u-dependent coefficients (binomial expansion).
/// 2. Multiply by `x(s)`; result is a polynomial in s with u-dependent coefficients.
/// 3. Integrate `s^k → s^(k+1)/(k+1)`, evaluate at `s_hi(u)` and `s_lo(u)`.
/// 4. Both `s_lo` and `s_hi` are linear in u, so output is polynomial in u.
#[cfg(feature = "host")]
fn integrate_product_piece<T: Float>(
    x: &crate::bezier::BezierPiece<T>,
    w: &crate::bezier::BezierPiece<T>,
    alpha: T,
    beta: T,
) -> crate::bezier::BezierPiece<T> {
    let d_x = x.degree();
    let d_w = w.degree();
    let out_degree = d_x + d_w + 1;

    // Integration limits as polynomials in u (degree 1, in absolute u, NOT shifted).
    // s_lo(u) = max(x.u_start, u - w.u_end)
    // s_hi(u) = min(x.u_end,   u - w.u_start)
    //
    // For u in [α, β] by construction of out_breaks, the active branch of max/min
    // is constant; we can determine it from the value at u = (α + β) / 2.
    let u_mid = (alpha + beta) * T::from_f64(0.5);
    let lo_branch_curve = u_mid - w.u_end > x.u_start; // true → s_lo(u) = u - w.u_end
    let hi_branch_curve = u_mid - w.u_start < x.u_end; // true → s_hi(u) = u - w.u_start

    // -------------------------------------------------------------------
    // Numerical conditioning: do all arithmetic in a frame shifted by α.
    //
    // The naive approach (build polynomials in absolute u and s, then
    // re-shift to Pascal-at-α at the end) suffers from catastrophic
    // cancellation when α is large and the output piece is narrow:
    // intermediate u^k ≈ 2^k coefficients up to k = d_x + d_w + 1 grow to
    // 1e2..1e3, then `absolute_to_pascal_shift` (which sums binomial
    // products of `α^(n-k)` against those coefficients) yields alternating
    // huge magnitudes whose cancellations destroy ~10 digits of accuracy
    // in the trailing piece. The trailing piece's polynomial value at
    // u = β then disagrees with the value just below by ~10 mm in the
    // motion-bridge regression scenario (bug #18).
    //
    // Working in v = u − α (so v ∈ [0, β − α]) and r = s − α keeps every
    // intermediate coefficient O(width^k) instead of O(α^k). The final
    // result is already in the basis (u − α)^k, which is exactly what the
    // output `BezierPiece` stores — no re-shift required.
    // -------------------------------------------------------------------

    // Shifted x: x(s) = Σ x.coeffs[k] (s − x.u_start)^k, want absolute-r basis
    // where r = s − α, so (s − x.u_start) = r − (x.u_start − α).
    let x_abs_r = pascal_shift_to_absolute(&x.coeffs, x.u_start - alpha);

    // Shifted w: w(z) where z = u − s = v − r. We want w in absolute-z basis,
    // and z is unchanged by the α-shift since u and s shift together. So
    // w_abs_z is the same as the un-shifted version.
    let w_abs_z = pascal_shift_to_absolute(&w.coeffs, w.u_start);
    // Expand z^j = (v − r)^j via binomial, giving polynomial in v and r.

    // Shifted integration limits:
    //   r_lo(v) = s_lo(u) − α
    //   r_hi(v) = s_hi(u) − α
    //   if lo_branch_curve: s_lo = u − w.u_end → r_lo = v − w.u_end
    //   else:               s_lo = x.u_start  → r_lo = (x.u_start − α)
    //   if hi_branch_curve: s_hi = u − w.u_start → r_hi = v − w.u_start
    //   else:               s_hi = x.u_end  → r_hi = (x.u_end − α)
    let (r_lo_c, r_lo_v): (T, T) = if lo_branch_curve {
        (-w.u_end, T::ONE)
    } else {
        (x.u_start - alpha, T::ZERO)
    };
    let (r_hi_c, r_hi_v): (T, T) = if hi_branch_curve {
        (-w.u_start, T::ONE)
    } else {
        (x.u_end - alpha, T::ZERO)
    };

    // Build 2D coefficient table: integrand[m][n] = coefficient of v^m · r^n.
    let max_m = d_w;
    let max_n = d_x + d_w;
    let mut integrand = vec![vec![T::ZERO; max_n + 1]; max_m + 1];

    for j in 0..=d_w {
        for l in 0..=j {
            let m = j - l;
            let sign = if l % 2 == 0 { T::ONE } else { -T::ONE };
            let c_jl = T::from_f64(binomial(j, l) as f64);
            let coef = sign * c_jl * w_abs_z[j];
            for i in 0..=d_x {
                let n = l + i;
                integrand[m][n] = integrand[m][n] + coef * x_abs_r[i];
            }
        }
    }

    // Integrate r^n → r^(n+1) / (n+1), evaluate at r_hi(v) − r_lo(v).
    // Output `y_v[k]` = coefficient of v^k in the result; this is *exactly*
    // the Pascal-shifted-at-α coefficient that BezierPiece stores.
    let mut y_v = vec![T::ZERO; out_degree + 1];
    for m in 0..=max_m {
        for n in 0..=max_n {
            if integrand[m][n] == T::ZERO {
                continue;
            }
            let inv = integrand[m][n] / T::from_f64((n + 1) as f64);
            let hi_pow = power_of_linear(r_hi_c, r_hi_v, n + 1);
            let lo_pow = power_of_linear(r_lo_c, r_lo_v, n + 1);
            for k in 0..hi_pow.len() {
                let target = k + m;
                if target <= out_degree {
                    y_v[target] = y_v[target] + inv * (hi_pow[k] - lo_pow[k]);
                }
            }
        }
    }

    crate::bezier::BezierPiece {
        u_start: alpha,
        u_end: beta,
        coeffs: y_v,
    }
}

/// Expand `(c + a*u)^p` as a polynomial in u (length `p+1`, ascending power).
#[cfg(feature = "host")]
fn power_of_linear<T: Float>(c: T, a: T, p: usize) -> Vec<T> {
    let mut out = vec![T::ZERO; p + 1];
    let mut c_pow = vec![T::ONE; p + 1];
    let mut a_pow = vec![T::ONE; p + 1];
    for k in 1..=p {
        c_pow[k] = c_pow[k - 1] * c;
        a_pow[k] = a_pow[k - 1] * a;
    }
    for k in 0..=p {
        let bin = T::from_f64(binomial(p, k) as f64);
        out[k] = bin * c_pow[p - k] * a_pow[k];
    }
    out
}

/// Convert Pascal-shifted-at-`shift` coefficients to absolute monomial.
/// `p(u) = Σ c_k * (u - shift)^k → Σ c'_n * u^n`
#[cfg(feature = "host")]
fn pascal_shift_to_absolute<T: Float>(shifted: &[T], shift: T) -> Vec<T> {
    let d = shifted.len() - 1;
    let mut out = vec![T::ZERO; d + 1];
    for k in 0..=d {
        // (u - shift)^k = (-shift + u)^k
        let exp = power_of_linear(-shift, T::ONE, k);
        for n in 0..exp.len() {
            out[n] = out[n] + shifted[k] * exp[n];
        }
    }
    out
}

/// Inverse: convert absolute monomial to Pascal-shifted-at-`shift`.
/// `Σ c_n * u^n → Σ c'_k * (u - shift)^k` where
/// `u^n = Σ_k C(n, k) * shift^(n-k) * (u - shift)^k`.
#[cfg(feature = "host")]
fn absolute_to_pascal_shift<T: Float>(absolute: &[T], shift: T) -> Vec<T> {
    let d = absolute.len() - 1;
    let mut out = vec![T::ZERO; d + 1];
    let mut shift_pow = vec![T::ONE; d + 1];
    for k in 1..=d {
        shift_pow[k] = shift_pow[k - 1] * shift;
    }
    for n in 0..=d {
        for k in 0..=n {
            let bin = T::from_f64(binomial(n, k) as f64);
            out[k] = out[k] + absolute[n] * bin * shift_pow[n - k];
        }
    }
    out
}

/// Extract the portion of a `ScalarNurbs` on the sub-interval `[t_lo, t_hi]`.
///
/// Pieces fully outside the interval are discarded; pieces partially overlapping
/// are split at the boundary. Returns `SupportMismatch` if `t_lo >= t_hi` or if
/// no pieces overlap the requested interval.
#[cfg(feature = "host")]
pub fn restrict_to_domain<T: Float>(
    curve: &crate::ScalarNurbs<T>,
    t_lo: T,
    t_hi: T,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    use crate::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces, split_piece_at};

    if t_lo >= t_hi {
        return Err(AlgebraError::SupportMismatch);
    }

    let pieces = extract_bezier_pieces(curve);
    let mut result = Vec::new();

    for piece in &pieces {
        if piece.u_end <= t_lo || piece.u_start >= t_hi {
            continue;
        }

        let mut p = piece.clone();

        if p.u_start < t_lo {
            let (_, right) = split_piece_at(&p, t_lo);
            p = right;
        }

        if p.u_end > t_hi {
            let (left, _) = split_piece_at(&p, t_hi);
            p = left;
        }

        result.push(p);
    }

    if result.is_empty() {
        return Err(AlgebraError::SupportMismatch);
    }

    Ok(bezier_pieces_to_nurbs(&result))
}

/// Iterate over interior knots and apply `remove_knot` with the given tolerance,
/// dropping knots whose removal preserves the curve within `tol`. Used by
/// `multiply` and `convolve` to expose natural smoothness of the result.
#[cfg(feature = "host")]
pub(crate) fn knot_remove_redundant<T: Float>(curve: &mut crate::ScalarNurbs<T>, tol: T) {
    let p = curve.degree() as usize;
    loop {
        let knots: Vec<T> = curve.knots().to_vec();
        let interior: Vec<T> = {
            let mut seen: Vec<T> = Vec::new();
            for &k in &knots[p + 1..knots.len() - p - 1] {
                if !seen.contains(&k) {
                    seen.push(k);
                }
            }
            seen
        };

        let mut removed_any = false;
        for u in interior {
            let (new_curve, count) = crate::knot::remove_knot(curve, u, 1, tol);
            if count > 0 {
                *curve = new_curve;
                removed_any = true;
            }
        }
        if !removed_any {
            break;
        }
    }
}

#[cfg(all(test, feature = "host"))]
#[allow(clippy::float_cmp)] // tests assert exact stored coords / round-trip values, not arithmetic results
mod tests {
    use super::*;
    use crate::eval::eval;

    #[test]
    fn convolve_linear_input_with_constant_kernel_yields_correct_integral() {
        // x(s) = s on [0, 1], w(t) = 1 on [-0.25, 0.25].
        // y(u) = ∫_{u-0.25}^{u+0.25} s ds = (1/2) * ((u+0.25)^2 - (u-0.25)^2) = u/2
        // for u in [0.25, 0.75] (kernel window fully inside x's support).
        // y(0.5) = 0.5/2 = 0.25.  Equivalently: width * average = 0.5 * 0.5 = 0.25.
        let x =
            crate::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None)
                .unwrap();
        let kernel = PiecewisePolynomialKernel::single_poly(vec![1.0_f64], (-0.25, 0.25));

        let y = convolve(&x, &kernel).unwrap();

        let val = eval(&y.as_view(), 0.5);
        assert!((val - 0.25).abs() < 1e-10, "y(0.5) = {val}, expected 0.25");
    }

    #[test]
    fn breakpoint_sort_handles_nan_without_panicking() {
        let mut out_breaks = vec![0.0_f64, f64::NAN, 1.0];
        out_breaks.sort_by(|a, b| <f64 as crate::Float>::total_cmp(*a, *b));
        assert_eq!(out_breaks.len(), 3);
    }

    #[test]
    fn convolve_constant_input_with_constant_kernel_gives_triangle() {
        // x(s) = 2 on [0, 1], w(t) = 3 on [-0.5, 0.5].
        // Convolution support: [0 + (-0.5), 1 + 0.5] = [-0.5, 1.5].
        // Output: triangle peaking in [0.5, 0.5] at value 6, sloping linearly to 0 at boundaries.
        let x =
            crate::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![2.0, 2.0], None)
                .unwrap();
        let kernel = PiecewisePolynomialKernel::single_poly(vec![3.0_f64], (-0.5, 0.5));

        let y = convolve(&x, &kernel).unwrap();

        // Spot-check: at u = 0.5, the kernel window [0.0, 1.0] is fully inside x's support,
        // so y(0.5) = ∫_{0}^{1} 2 * 3 ds = 6.
        let val = eval(&y.as_view(), 0.5);
        assert!((val - 6.0).abs() < 1e-10, "y(0.5) = {val}, expected 6");

        // At u = -0.5 (left boundary of output), y = 0.
        let val_lo = eval(&y.as_view(), -0.5);
        assert!(val_lo.abs() < 1e-10, "y(-0.5) = {val_lo}, expected 0");

        // At u = 1.5 (right boundary), y = 0.
        let val_hi = eval(&y.as_view(), 1.5);
        assert!(val_hi.abs() < 1e-10, "y(1.5) = {val_hi}, expected 0");
    }

    #[test]
    fn integrate_product_constant_input_constant_kernel_yields_linear_result() {
        // x(s) = 2 on [0, 1], w(t) = 3 on [-0.5, 0.5].
        // y(u) = ∫ x(s) w(u - s) ds, integration range = intersection of
        // [u - 0.5, u + 0.5] with [0, 1].
        // For u ∈ [0.5, 0.5] (single point), y = 2*3*1 = 6.
        // Generally y(u) = 6 * (length of overlap window).

        let x = crate::bezier::BezierPiece::<f64> {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![2.0], // constant 2
        };
        let w = crate::bezier::BezierPiece::<f64> {
            u_start: -0.5,
            u_end: 0.5,
            coeffs: vec![3.0], // constant 3
        };

        // Integrate over output sub-interval [0.5, 1.0] where the kernel window
        // shrinks linearly (from full overlap to half overlap at u=1.0).
        let contribution = integrate_product_piece(&x, &w, 0.5, 1.0);

        // Expected: y(u) = 6 * (1.0 - (u - 0.5)) for u ∈ [0.5, 1.0]
        //                = 6 * (1.5 - u)
        //                = 9 - 6u
        // In Pascal-shifted basis at α = 0.5: y(u) = 9 - 6u = 9 - 6*(0.5 + (u - 0.5))
        //                                          = 6 - 6 * (u - 0.5)
        // So coeffs at u_start = 0.5 should be [6.0, -6.0].
        assert!((contribution.coeffs[0] - 6.0).abs() < 1e-10);
        assert!((contribution.coeffs[1] - (-6.0)).abs() < 1e-10);
    }

    #[test]
    fn convolve_rejects_rational_input() {
        let curve = crate::ScalarNurbs::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0_f64, 1.0],
            Some(vec![1.0, 1.0]),
        )
        .unwrap();
        let kernel = PiecewisePolynomialKernel::single_poly(vec![1.0_f64], (-0.1, 0.1));
        let result = convolve(&curve, &kernel);
        assert!(matches!(
            result,
            Err(AlgebraError::RationalNotSupported {
                operation: "convolve",
                ..
            })
        ));
    }

    #[test]
    fn single_poly_kernel_constructs_one_piece() {
        let k = PiecewisePolynomialKernel::single_poly(vec![1.0, 0.5_f64], (-1.0, 1.0));
        assert_eq!(k.pieces.len(), 1);
        assert_eq!(k.pieces[0].u_start, -1.0);
        assert_eq!(k.pieces[0].u_end, 1.0);
        assert_eq!(k.pieces[0].coeffs, vec![1.0, 0.5]);
    }

    #[test]
    fn kernel_support_returns_endpoints() {
        let k = PiecewisePolynomialKernel::single_poly(vec![1.0_f64], (-0.5, 0.5));
        assert_eq!(k.support(), (-0.5, 0.5));
    }

    #[test]
    fn knot_remove_redundant_simplifies_overproduct() {
        let a =
            crate::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None)
                .unwrap();
        let b = a.clone();
        let mut c = multiply(&a, &b).unwrap();
        let initial_knot_count = c.knots().len();

        knot_remove_redundant(&mut c, 1e-10);

        // For a single-piece input, no interior knots to remove; result unchanged.
        assert_eq!(c.knots().len(), initial_knot_count);
        // Eval still correct.
        for u in [0.0, 0.5, 1.0] {
            let exp = eval(&a.as_view(), u) * eval(&b.as_view(), u);
            let got = eval(&c.as_view(), u);
            assert!((exp - got).abs() < 1e-10);
        }
    }

    #[test]
    fn multiply_curves_with_different_interior_knots() {
        // a has interior knot at 0.4, b has interior knot at 0.7.
        let a = crate::ScalarNurbs::<f64>::try_new(
            2,
            vec![0.0, 0.0, 0.0, 0.4, 1.0, 1.0, 1.0],
            vec![0.0, 1.0, 2.0, 3.0],
            None,
        )
        .unwrap();
        let b = crate::ScalarNurbs::<f64>::try_new(
            2,
            vec![0.0, 0.0, 0.0, 0.7, 1.0, 1.0, 1.0],
            vec![1.0, 2.0, 0.0, 1.0],
            None,
        )
        .unwrap();
        let c = multiply(&a, &b).unwrap();
        assert_eq!(c.degree(), 4);
        for u in [0.0, 0.2, 0.4, 0.5, 0.7, 0.9, 1.0] {
            let exp = eval(&a.as_view(), u) * eval(&b.as_view(), u);
            let got = eval(&c.as_view(), u);
            assert!((exp - got).abs() < 1e-10, "u={u}: exp={exp}, got={got}");
        }
    }

    #[test]
    fn multiply_two_linear_curves_gives_quadratic() {
        // a(u) = u, b(u) = 2u + 1, expected c(u) = u(2u + 1) = 2u^2 + u.
        let a =
            crate::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None)
                .unwrap();
        let b =
            crate::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![1.0, 3.0], None)
                .unwrap();
        let c = multiply(&a, &b).unwrap();
        assert_eq!(c.degree(), 2);
        for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let exp = eval(&a.as_view(), u) * eval(&b.as_view(), u);
            let got = eval(&c.as_view(), u);
            assert!((exp - got).abs() < 1e-12, "u={u}: exp={exp}, got={got}");
        }
    }

    #[test]
    fn scalar_multiply_doubles_evaluation() {
        let curve =
            crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None).unwrap();
        let doubled = scalar_multiply(&curve, 2.0_f64);
        assert!((eval(&doubled.as_view(), 0.5_f64) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn scalar_multiply_preserves_weights() {
        let curve = crate::ScalarNurbs::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![1.0, 2.0],
            Some(vec![1.0, 1.0]),
        )
        .unwrap();
        let result = scalar_multiply(&curve, 3.0_f64);
        assert_eq!(result.weights().unwrap(), &[1.0, 1.0]);
        assert_eq!(result.control_points(), &[3.0, 6.0]);
    }

    #[test]
    fn add_two_compatible_curves() {
        let a =
            crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None).unwrap();
        let b =
            crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![2.0, 3.0], None).unwrap();
        let sum = add(&a, &b).unwrap();
        // At u=0.5: 0.5 + 2.5 = 3.0
        assert!((eval(&sum.as_view(), 0.5_f64) - 3.0).abs() < 1e-12);
    }

    #[test]
    fn add_rejects_mismatched_degree() {
        let a =
            crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None).unwrap();
        let b = crate::ScalarNurbs::try_new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.5, 1.0],
            None,
        )
        .unwrap();
        let result = add(&a, &b);
        assert!(matches!(result, Err(crate::AlgebraError::KnotMismatch)));
    }

    #[test]
    fn multiply_rejects_rational_input() {
        let a = crate::ScalarNurbs::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 1.0],
            Some(vec![1.0, 1.0]),
        )
        .unwrap();
        let b = a.clone();
        let result = multiply(&a, &b);
        assert!(matches!(
            result,
            Err(crate::AlgebraError::RationalNotSupported {
                operation: "multiply",
                ..
            })
        ));
    }

    #[test]
    fn from_pieces_accepts_contiguous_kernel() {
        let pieces = vec![
            crate::bezier::BezierPiece {
                u_start: -0.5,
                u_end: 0.0,
                coeffs: vec![1.0_f64],
            },
            crate::bezier::BezierPiece {
                u_start: 0.0,
                u_end: 0.5,
                coeffs: vec![2.0_f64],
            },
        ];
        let k = PiecewisePolynomialKernel::from_pieces(pieces).unwrap();
        assert_eq!(k.pieces.len(), 2);
        assert_eq!(k.support(), (-0.5, 0.5));
    }

    #[test]
    fn from_pieces_rejects_non_contiguous() {
        let pieces = vec![
            crate::bezier::BezierPiece {
                u_start: -0.5_f64,
                u_end: 0.0,
                coeffs: vec![1.0],
            },
            crate::bezier::BezierPiece {
                u_start: 0.1,
                u_end: 0.5,
                coeffs: vec![2.0],
            }, // gap
        ];
        let result = PiecewisePolynomialKernel::from_pieces(pieces);
        assert!(matches!(result, Err(AlgebraError::SupportMismatch)));
    }

    #[test]
    fn from_pieces_rejects_empty() {
        let result = PiecewisePolynomialKernel::<f64>::from_pieces(vec![]);
        assert!(matches!(result, Err(AlgebraError::SupportMismatch)));
    }

    #[test]
    fn pascal_shift_round_trip_preserves_polynomial() {
        // Cubic with non-zero shift — exercises both directions of basis conversion.
        let coeffs = vec![1.0, 2.0, 3.0, -1.5_f64];
        let shift = 0.7;
        let absolute = pascal_shift_to_absolute(&coeffs, shift);
        let back = absolute_to_pascal_shift(&absolute, shift);
        for i in 0..coeffs.len() {
            assert!(
                (back[i] - coeffs[i]).abs() < 1e-12,
                "coeff[{i}]: original {} != round-tripped {}",
                coeffs[i],
                back[i],
            );
        }
    }

    #[test]
    fn single_poly_from_absolute_constructs_kernel_with_correct_polynomial() {
        // Absolute form: w(t) = 1 + 2t on [0.5, 1.5].
        // Pascal-shifted at u_start = 0.5: w(u) = 1 + 2*(u - 0.5 + 0.5) = 2 + 2*(u - 0.5).
        let k = PiecewisePolynomialKernel::single_poly_from_absolute(
            vec![1.0_f64, 2.0], // 1 + 2t in absolute t
            (0.5, 1.5),
        );
        assert_eq!(k.pieces.len(), 1);
        assert_eq!(k.pieces[0].u_start, 0.5);
        assert_eq!(k.pieces[0].u_end, 1.5);
        // Pascal-shifted coeffs at u_start=0.5: c_0 = 1 + 2*0.5 = 2.0, c_1 = 2.0.
        assert!((k.pieces[0].coeffs[0] - 2.0).abs() < 1e-12);
        assert!((k.pieces[0].coeffs[1] - 2.0).abs() < 1e-12);
        // Polynomial value at t=0.5 (u=1.0) should be 1 + 2*0.5 = 2.0 in original basis.
        // In Pascal-shifted basis: c_0 + c_1*(1 - 0.5) = 2 + 2*0.5 = 3.0... wait.
        // Let me recheck: w_absolute(t) = 1 + 2t evaluated at t=1.0 = 3.0.
        // In Pascal-shifted basis with u_start=0.5: shifted polynomial is the same function,
        // i.e., w(u) = 1 + 2u (the absolute t IS the absolute u for a kernel),
        // expressed as Σ c_k * (u - 0.5)^k.
        // At u=1.0 (which is t=1.0 since t=u for a kernel): expected 1 + 2*1.0 = 3.0.
        // Pascal: c_0 + c_1*(1 - 0.5) = 2 + 2*0.5 = 3.0 ✓
        let val_at_one = k.pieces[0].evaluate(1.0);
        assert!((val_at_one - 3.0).abs() < 1e-12);
    }

    #[test]
    fn single_poly_from_absolute_round_trips_via_evaluate() {
        // Quadratic kernel: w(t) = 1 - 2t + 3t^2, on [-0.5, 0.5].
        let k = PiecewisePolynomialKernel::single_poly_from_absolute(
            vec![1.0_f64, -2.0, 3.0],
            (-0.5, 0.5),
        );
        // Sample at three points and confirm Pascal-shifted eval == absolute eval.
        for t in [-0.5_f64, 0.0, 0.25, 0.5] {
            let absolute_val = 1.0 - 2.0 * t + 3.0 * t * t;
            let pascal_val = k.pieces[0].evaluate(t);
            assert!(
                (absolute_val - pascal_val).abs() < 1e-12,
                "t={t}: absolute={absolute_val}, pascal={pascal_val}"
            );
        }
    }

    #[test]
    fn multiply_regression_proptest_shrunk_failing_input() {
        // Captured from algebra_proptest::multiply_multi_piece_eval_matches_pointwise_product
        // pre-Fix-1 (Mørken-bounded knot removal). At u=0.1, b has C⁰ kink (m_b=1, d_b=1)
        // and a has interior multiplicity-1 knot (m_a=1, d_a=3). Per Mørken Eq. (1):
        // μ_target(0.1) = max(3+1, 1+1) = 4. Pre-Fix-1 the unbounded knot_remove_redundant
        // peeled below 4, smearing the C⁰ kink and producing wrong eval at u=0.1.
        let a = crate::ScalarNurbs::<f64>::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 0.1, 0.55, 1.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 0.0, 0.181_828_016_839_598_23, 0.0, 0.0],
            None,
        )
        .unwrap();
        let b = crate::ScalarNurbs::<f64>::try_new(
            1,
            vec![0.0, 0.0, 0.1, 1.0, 1.0],
            vec![0.0, 4.267_190_258_636_853, 0.0],
            None,
        )
        .unwrap();
        let c = multiply(&a, &b).unwrap();
        // Pointwise product at u=0.1 should be ≈ 0.014107177131003477.
        // Pre-fix `multiply` returned ≈ 0.007758947422051913.
        let exp = eval(&a.as_view(), 0.1) * eval(&b.as_view(), 0.1);
        let got = eval(&c.as_view(), 0.1);
        assert!(
            (exp - got).abs() < 1e-10,
            "u=0.1: pointwise={exp}, multiply={got} (regression)"
        );
    }

    #[test]
    fn multiply_quadratic_x_linear_gives_cubic() {
        // a(u) = u^2 (Bernstein cps [0, 0, 1] for monomial u^2 on [0, 1]).
        let a = crate::ScalarNurbs::<f64>::try_new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.0, 1.0],
            None,
        )
        .unwrap();
        // b(u) = u, same as before.
        let b =
            crate::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None)
                .unwrap();
        let c = multiply(&a, &b).unwrap();
        assert_eq!(c.degree(), 3);
        // Expected: c(u) = u^3.
        for u in [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0] {
            let exp = u * u * u;
            let got = eval(&c.as_view(), u);
            assert!(
                (exp - got).abs() < 1e-12,
                "u={u}: u^3={exp}, multiply={got}"
            );
        }
    }

    /// Diagnostic: convolve preserves the natural multiplicity at cross-sum
    /// kink images, and produces correct eval, on a multi-piece input with a
    /// C0 kink. This is the convolve analog of the multi-piece multiply
    /// regression test from Fix 1 (Mørken-bounded knot removal).
    ///
    /// Mathematical setup (per the convolution-continuity rule):
    /// - `x` is degree 2 with interior knot at `u_x = 0.3` of multiplicity
    ///   `m_x = 2` (a C0 kink — slope jumps across `u = 0.3`).
    /// - `w` is the constant kernel `1` on `[-0.1, 0.1]` (single-piece, no
    ///   interior knots, kernel degree `d_w = 0`).
    /// - Output `y` has degree `p_y = d_x + d_w + 1 = 3`.
    /// - The kink images in `y` are at `u = u_x ± t_w_endpoint = {0.2, 0.4}`.
    ///   Per the convolution-continuity rule, `μ_y` at each kink image equals
    ///   `m_x = 2` (input C0 → output `C^{0 + d_w + 1}` = C1 → `μ_y = p_y − 1 = 2`).
    ///
    /// This test asserts BOTH:
    ///   (A) `μ_y(0.2) = μ_y(0.4) = 2` — the kink images are preserved at the
    ///       natural multiplicity, not over-peeled like multiply pre-Fix 1.
    ///   (B) `y(0.2)` and `y(0.4)` match closed-form integrals of `x` against
    ///       the constant kernel — the load-bearing eval check.
    ///
    /// Note: this test deliberately does NOT assert `μ_y = 0` at the boundary
    /// cross-sums `u ∈ {0.1, 0.9}`. The spec's convolution-continuity rule
    /// predicts `μ_y = 0` there (no real continuity break), but in practice the
    /// post-pass `knot_remove_redundant` (Tiller A5.8 with chord-error tol)
    /// can only peel a knot when both polynomial pieces match as polynomial
    /// expressions, not just as functions agreeing at the join. At a boundary
    /// cross-sum the left and right pieces of `y` differ by `(u − u_break)^k`
    /// terms that vanish at the join but not elsewhere, so Tiller refuses
    /// removal even though geometrically the curve is C-infinity there. This
    /// leaves extra multiplicity at boundary cross-sums; harmless (eval is
    /// correct, downstream ops don't care) and not the bug class tested here.
    #[test]
    fn convolve_multi_piece_input_with_c0_kink_preserves_natural_multiplicity() {
        // x: degree 2, knots [0,0,0, 0.3,0.3, 1,1,1], asymmetric CPs.
        // Two pieces, sharing x(0.3) = 4 with a slope jump (C⁰ kink at 0.3).
        let x = crate::ScalarNurbs::<f64>::try_new(
            2,
            vec![0.0, 0.0, 0.0, 0.3, 0.3, 1.0, 1.0, 1.0],
            vec![0.0, 1.0, 4.0, 0.5, 0.2],
            None,
        )
        .unwrap();
        // w(t) = 1 on [-0.1, 0.1] — single piece, no interior knots.
        let kernel = PiecewisePolynomialKernel::single_poly(vec![1.0_f64], (-0.1, 0.1));

        let y = convolve(&x, &kernel).unwrap();

        // Output degree = d_x + d_w + 1 = 2 + 0 + 1 = 3.
        assert_eq!(y.degree(), 3, "output degree");

        // (A) Multiplicity at kink images: μ_y(0.2) = μ_y(0.4) = m_x = 2.
        let p = y.degree() as usize;
        let interior = &y.knots()[p + 1..y.knots().len() - p - 1];
        let mult_at_02 = interior.iter().filter(|k| (**k - 0.2).abs() < 1e-9).count();
        let mult_at_04 = interior.iter().filter(|k| (**k - 0.4).abs() < 1e-9).count();
        assert_eq!(
            mult_at_02, 2,
            "expected μ_y(0.2) = m_x = 2 (kink image), got {mult_at_02}; full interior = {interior:?}",
        );
        assert_eq!(
            mult_at_04, 2,
            "expected μ_y(0.4) = m_x = 2 (kink image), got {mult_at_04}; full interior = {interior:?}",
        );

        // (B) Pointwise eval check vs hand-computed integrals.
        // x_1(s) = (20/3) s + (200/9) s² on [0, 0.3];
        // x_2(s) = 4 − 10 (s − 0.3) + (320/49) (s − 0.3)² on [0.3, 1.0].
        //   y(0.2) = ∫_{0.1}^{0.3} x_1(s) ds
        //          = (10/3)(0.09 − 0.01) + (200/27)(0.027 − 0.001)
        //          = 0.8/3 + 5.2/27.
        //   y(0.4) = ∫_{0.3}^{0.5} x_2(s) ds
        //          = 4·0.2 − 5·(0.2)² + (320/147)·(0.2)³
        //          = 0.6 + 2.56/147.
        let exp_02 = 0.8 / 3.0 + 5.2 / 27.0;
        let exp_04 = 0.6 + 2.56 / 147.0;
        let got_02 = eval(&y.as_view(), 0.2);
        let got_04 = eval(&y.as_view(), 0.4);
        assert!(
            (got_02 - exp_02).abs() < 1e-10,
            "y(0.2): expected {exp_02}, got {got_02}, diff {}",
            (got_02 - exp_02).abs(),
        );
        assert!(
            (got_04 - exp_04).abs() < 1e-10,
            "y(0.4): expected {exp_04}, got {got_04}, diff {}",
            (got_04 - exp_04).abs(),
        );

        // Also spot-check eval at a non-kink interior point — confirms the
        // entire shape (not just kink-image samples) matches the convolution
        // integral. At u = 0.5 the kernel window [0.4, 0.6] is fully inside
        // x's piece-2 support [0.3, 1.0], so:
        //   y(0.5) = ∫_{0.4}^{0.6} x_2(s) ds
        //          = ∫_{0.1}^{0.3} (4 − 10·v + (320/49)·v²) dv  (v = s − 0.3)
        //          = 4·0.2 − 5·(0.09 − 0.01) + (320/147)·(0.027 − 0.001)
        //          = 0.8 − 0.4 + (320·0.026)/147
        //          = 0.4 + 8.32/147.
        let exp_05 = 0.4 + 8.32 / 147.0;
        let got_05 = eval(&y.as_view(), 0.5);
        assert!(
            (got_05 - exp_05).abs() < 1e-10,
            "y(0.5): expected {exp_05}, got {got_05}, diff {}",
            (got_05 - exp_05).abs(),
        );
    }

    // ── add_with_knot_union tests ────────────────────────────────────────────

    /// Identical knot vectors → fast path: delegates to `add`, no refine.
    #[test]
    fn add_with_knot_union_identical_knots_fast_path() {
        let a = crate::ScalarNurbs::try_new(
            1,
            vec![0.0_f64, 0.0, 1.0, 1.0],
            vec![0.0, 1.0],
            None,
        )
        .unwrap();
        let b = crate::ScalarNurbs::try_new(
            1,
            vec![0.0_f64, 0.0, 1.0, 1.0],
            vec![2.0, 3.0],
            None,
        )
        .unwrap();
        let sum = add_with_knot_union(&a, &b).unwrap();
        // At u=0: 0+2=2. At u=1: 1+3=4. At u=0.5 (midpoint): 0.5+2.5=3.
        assert!((eval(&sum.as_view(), 0.0_f64) - 2.0).abs() < 1e-12, "fast-path u=0");
        assert!((eval(&sum.as_view(), 0.5_f64) - 3.0).abs() < 1e-12, "fast-path u=0.5");
        assert!((eval(&sum.as_view(), 1.0_f64) - 4.0).abs() < 1e-12, "fast-path u=1");
    }

    /// Mismatched knot vectors → knot-union path: piece counts and eval values correct.
    ///
    /// `a` has two pieces (linear 0→5 on [0,0.5] then 5→10 on [0.5,1]).
    /// `b` has one piece (constant 20). After union, both are 2 pieces, and
    /// the sum evaluates to 20→25→30.
    #[test]
    fn add_with_knot_union_mismatched_knots_union_path() {
        use crate::bezier::{BezierPiece, bezier_pieces_to_nurbs};

        // Two-piece linear curve on [0,1]: 0→5 then 5→10.
        // Pascal-shifted coefficients at u_start.
        let a = bezier_pieces_to_nurbs(&[
            BezierPiece::<f64> { u_start: 0.0, u_end: 0.5, coeffs: vec![0.0, 10.0] },
            BezierPiece::<f64> { u_start: 0.5, u_end: 1.0, coeffs: vec![5.0, 10.0] },
        ]);
        // Single-piece constant 20.
        let b = crate::ScalarNurbs::try_new(
            1,
            vec![0.0_f64, 0.0, 1.0, 1.0],
            vec![20.0, 20.0],
            None,
        )
        .unwrap();

        let sum = add_with_knot_union(&a, &b).unwrap();
        // Check at domain boundary and midpoint of each piece.
        let cases = [(0.0_f64, 20.0), (0.25, 22.5), (0.5, 25.0), (0.75, 27.5), (1.0, 30.0)];
        for (u, expected) in cases {
            let got = eval(&sum.as_view(), u);
            assert!(
                (got - expected).abs() < 1e-10,
                "union-path u={u}: expected {expected}, got {got}",
            );
        }
    }

    /// Degree mismatch → `KnotMismatch` error (same as `add`'s contract).
    #[test]
    fn add_with_knot_union_rejects_degree_mismatch() {
        let a = crate::ScalarNurbs::try_new(
            1,
            vec![0.0_f64, 0.0, 1.0, 1.0],
            vec![0.0, 1.0],
            None,
        )
        .unwrap();
        let b = crate::ScalarNurbs::try_new(
            2,
            vec![0.0_f64, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.5, 1.0],
            None,
        )
        .unwrap();
        let result = add_with_knot_union(&a, &b);
        assert!(
            matches!(result, Err(crate::AlgebraError::KnotMismatch)),
            "expected KnotMismatch, got {result:?}",
        );
    }

    /// Weighted curve → `NotImplemented` error (homogeneous lift deferred).
    #[test]
    fn add_with_knot_union_rejects_weighted_curves() {
        let a = crate::ScalarNurbs::try_new(
            1,
            vec![0.0_f64, 0.0, 1.0, 1.0],
            vec![0.0, 1.0],
            Some(vec![1.0, 2.0]),
        )
        .unwrap();
        let b = crate::ScalarNurbs::try_new(
            1,
            vec![0.0_f64, 0.0, 1.0, 1.0],
            vec![0.0, 1.0],
            None,
        )
        .unwrap();
        let result = add_with_knot_union(&a, &b);
        assert!(
            matches!(result, Err(crate::AlgebraError::NotImplemented(_))),
            "expected NotImplemented for weighted operand, got {result:?}",
        );
        // Also: b weighted, a not.
        let result2 = add_with_knot_union(&b, &a);
        assert!(
            matches!(result2, Err(crate::AlgebraError::NotImplemented(_))),
            "expected NotImplemented for weighted operand (swapped), got {result2:?}",
        );
    }
}
