# Smooth Shaper Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the shaper's dense piecewise-linear output + weak Hermite refit (which turns a 100 mm straight move into ~429 staircase pieces) with an adaptive **C2 cubic-spline fit of the true smooth convolution**, yielding ~tens of genuinely-smooth cubic pieces at the same 0.1 µm fidelity.

**Architecture:** A new evaluator `ShapedSignal` exposes the smooth convolution value `(x ⊛ w)(t)` at any `t` (extracted from `convolve_discrete`'s `fir_at`). A new `smooth_fit` module fits a clamped C2 interpolating cubic spline to that evaluator with adaptive knot insertion until max error ≤ tolerance. `emit_shaped` calls these two instead of `shape_axis` + `refit_to_cubic`. Stays uniform cubic; touches only the shaper/refit/emit stage — no planner, TOPP, geometry, or MCU changes.

**Tech Stack:** Rust, `nurbs` crate (`ScalarNurbs`, `BezierPiece`, `bezier_pieces_to_nurbs`), `cargo nextest` from `rust/`. Spec: `docs/superpowers/specs/2026-06-09-smooth-shaper-design.md`.

---

## Background the implementer needs

- `BezierPiece { u_start, u_end, coeffs }` evaluates in **local monomial basis**:
  `p(u) = Σ coeffs[k]·(u − u_start)^k` (see `nurbs/src/bezier.rs:17`). A cubic piece
  has `coeffs.len() == 4`.
- `bezier_pieces_to_nurbs(&[BezierPiece])` (`nurbs/src/bezier.rs:431`) stitches
  contiguous pieces into a `ScalarNurbs`; `extract_bezier_pieces(&curve)`
  (`:403`) is the inverse and is how we count pieces in tests.
- The smooth convolution value is already computed accurately by the `fir_at`
  closure inside `convolve_discrete` (`trajectory/src/shaper.rs:73`) using
  `INPUT_SAMPLES_PER_KERNEL_WIDTH = 40` quadrature. The defect is only the
  12-samples/kernel-width **linear** output + the endpoint-only Hermite refit.
- Tolerance: reuse `REFIT_TOLERANCE_MM = 1e-4` mm (`trajectory/src/refit.rs:6`).
- Run a single test: `cargo nextest run -p trajectory -E 'test(NAME)'` from `rust/`.
- Run all trajectory tests: `cargo nextest run -p trajectory` from `rust/`.

---

## File structure

- **Create** `rust/trajectory/src/smooth_fit.rs` — Thomas tridiagonal solver,
  clamped C2 cubic-spline builder, and the adaptive `fit_c2_cubic` entry point.
- **Create** `rust/trajectory/src/smooth_fit/tests.rs` — unit tests.
- **Modify** `rust/trajectory/src/shaper.rs` — add `ShapedSignal` evaluator.
- **Modify** `rust/trajectory/src/emit_shaped.rs` — call `ShapedSignal` +
  `fit_c2_cubic` instead of `shape_axis` + `refit_to_cubic`.
- **Modify** `rust/trajectory/src/lib.rs` — add `mod smooth_fit;`.
- **Delete (final task, after confirming no other callers)** `shape_axis` /
  `convolve_discrete` in `shaper.rs` and `refit_to_cubic` in `refit.rs`.

---

## Task 1: Tridiagonal (Thomas) solver

**Files:**
- Create: `rust/trajectory/src/smooth_fit.rs`
- Create: `rust/trajectory/src/smooth_fit/tests.rs`
- Modify: `rust/trajectory/src/lib.rs` (add `mod smooth_fit;` after `mod refit;`)

- [ ] **Step 1: Register the module**

In `rust/trajectory/src/lib.rs`, add after the line `mod refit;`:

```rust
mod smooth_fit;
```

- [ ] **Step 2: Write the failing test**

Create `rust/trajectory/src/smooth_fit/tests.rs`:

```rust
use super::*;

#[test]
fn thomas_solves_known_system() {
    // Tridiagonal system:
    // [ 2 1 0 ] [x0]   [3]
    // [ 1 2 1 ] [x1] = [4]   -> solution x = [1, 1, 1]
    // [ 0 1 2 ] [x2]   [3]
    let a = [0.0, 1.0, 1.0]; // sub-diagonal (a[0] unused)
    let b = [2.0, 2.0, 2.0]; // diagonal
    let c = [1.0, 1.0, 0.0]; // super-diagonal (c[n-1] unused)
    let d = [3.0, 4.0, 3.0];
    let x = solve_tridiagonal(&a, &b, &c, &d);
    for xi in &x {
        assert!((xi - 1.0).abs() < 1e-12, "x = {x:?}");
    }
}
```

- [ ] **Step 3: Implement the solver and wire the test module**

Create `rust/trajectory/src/smooth_fit.rs`:

```rust
/// Thomas algorithm for a tridiagonal system. `a` is the sub-diagonal
/// (a[0] ignored), `b` the diagonal, `c` the super-diagonal (c[n-1] ignored),
/// `d` the right-hand side. Returns the solution vector.
fn solve_tridiagonal(a: &[f64], b: &[f64], c: &[f64], d: &[f64]) -> Vec<f64> {
    let n = b.len();
    debug_assert!(n > 0 && a.len() == n && c.len() == n && d.len() == n);
    let mut cp = vec![0.0; n];
    let mut dp = vec![0.0; n];
    cp[0] = c[0] / b[0];
    dp[0] = d[0] / b[0];
    for i in 1..n {
        let m = b[i] - a[i] * cp[i - 1];
        cp[i] = c[i] / m;
        dp[i] = (d[i] - a[i] * dp[i - 1]) / m;
    }
    let mut x = vec![0.0; n];
    x[n - 1] = dp[n - 1];
    for i in (0..n - 1).rev() {
        x[i] = dp[i] - cp[i] * x[i + 1];
    }
    x
}

#[cfg(test)]
mod tests;
```

- [ ] **Step 4: Run the test**

Run: `cargo nextest run -p trajectory -E 'test(thomas_solves_known_system)'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/trajectory/src/lib.rs rust/trajectory/src/smooth_fit.rs rust/trajectory/src/smooth_fit/tests.rs
git commit -m "trajectory: add tridiagonal solver for smooth-shaper spline fit"
```

---

## Task 2: Clamped C2 cubic-spline builder

Builds the cubic pieces of the clamped interpolating spline through `knots`/`values`
with prescribed end slopes. C2 across interior joints by construction.

**Files:**
- Modify: `rust/trajectory/src/smooth_fit.rs`
- Test: `rust/trajectory/src/smooth_fit/tests.rs`

- [ ] **Step 1: Write the failing test**

Append to `rust/trajectory/src/smooth_fit/tests.rs`:

```rust
#[test]
fn clamped_spline_interpolates_and_is_c2() {
    // Fit f(t) = sin(t) on [0, PI] with 5 equal knots, clamped to f'=cos at ends.
    let knots: Vec<f64> = (0..5).map(|i| std::f64::consts::PI * i as f64 / 4.0).collect();
    let values: Vec<f64> = knots.iter().map(|t| t.sin()).collect();
    let yp0 = 0.0_f64.cos();
    let ypn = std::f64::consts::PI.cos();
    let pieces = build_clamped_spline(&knots, &values, yp0, ypn);

    assert_eq!(pieces.len(), 4);

    // Interpolation: each piece hits its endpoint knot values.
    for (i, p) in pieces.iter().enumerate() {
        assert!((p.evaluate(knots[i]) - values[i]).abs() < 1e-12);
        assert!((p.evaluate(knots[i + 1]) - values[i + 1]).abs() < 1e-12);
    }
    // C2: 2nd derivative continuous across interior joints.
    for i in 0..pieces.len() - 1 {
        let left = pieces[i].differentiate().differentiate();
        let right = pieces[i + 1].differentiate().differentiate();
        let j = knots[i + 1];
        assert!(
            (left.evaluate(j) - right.evaluate(j)).abs() < 1e-9,
            "2nd-deriv jump at knot {i}",
        );
    }
}
```

- [ ] **Step 2: Run it to confirm it fails**

Run: `cargo nextest run -p trajectory -E 'test(clamped_spline_interpolates_and_is_c2)'`
Expected: FAIL — `build_clamped_spline` not found.

- [ ] **Step 3: Implement the spline builder**

Add to `rust/trajectory/src/smooth_fit.rs` (above the `#[cfg(test)] mod tests;`):

```rust
use nurbs::bezier::BezierPiece;

/// Clamped interpolating cubic spline through `knots` (strictly increasing,
/// len m+1 >= 2) with values `y`, matching first derivative `yp0` at the start
/// and `ypn` at the end. Returns `m` cubic `BezierPiece`s in local monomial
/// basis. C2-continuous across interior joints by construction.
fn build_clamped_spline(knots: &[f64], y: &[f64], yp0: f64, ypn: f64) -> Vec<BezierPiece<f64>> {
    let m = knots.len() - 1;
    debug_assert!(m >= 1 && y.len() == knots.len());

    let h: Vec<f64> = (0..m).map(|i| knots[i + 1] - knots[i]).collect();

    // Solve for second derivatives M[0..=m] (clamped boundary conditions).
    let n = m + 1;
    let mut a = vec![0.0; n]; // sub-diagonal
    let mut b = vec![0.0; n]; // diagonal
    let mut c = vec![0.0; n]; // super-diagonal
    let mut d = vec![0.0; n]; // rhs

    // Start clamped: 2 h0 M0 + h0 M1 = 6((y1-y0)/h0 - yp0)
    b[0] = 2.0 * h[0];
    c[0] = h[0];
    d[0] = 6.0 * ((y[1] - y[0]) / h[0] - yp0);

    // Interior i=1..m-1: h[i-1] M[i-1] + 2(h[i-1]+h[i]) M[i] + h[i] M[i+1] = rhs
    for i in 1..m {
        a[i] = h[i - 1];
        b[i] = 2.0 * (h[i - 1] + h[i]);
        c[i] = h[i];
        d[i] = 6.0 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1]);
    }

    // End clamped: h[m-1] M[m-1] + 2 h[m-1] M[m] = 6(ypn - (ym - y[m-1])/h[m-1])
    a[m] = h[m - 1];
    b[m] = 2.0 * h[m - 1];
    d[m] = 6.0 * (ypn - (y[m] - y[m - 1]) / h[m - 1]);

    let mm = solve_tridiagonal(&a, &b, &c, &d);

    // Build each cubic piece in local monomial basis (x = t - knots[i]):
    //   S_i(x) = y_i + b_i x + (M_i/2) x^2 + ((M_{i+1}-M_i)/(6 h_i)) x^3
    //   b_i = (y_{i+1}-y_i)/h_i - h_i (2 M_i + M_{i+1})/6
    (0..m)
        .map(|i| {
            let bi = (y[i + 1] - y[i]) / h[i] - h[i] * (2.0 * mm[i] + mm[i + 1]) / 6.0;
            BezierPiece {
                u_start: knots[i],
                u_end: knots[i + 1],
                coeffs: vec![
                    y[i],
                    bi,
                    mm[i] / 2.0,
                    (mm[i + 1] - mm[i]) / (6.0 * h[i]),
                ],
            }
        })
        .collect()
}
```

- [ ] **Step 4: Run the test**

Run: `cargo nextest run -p trajectory -E 'test(clamped_spline_interpolates_and_is_c2)'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/trajectory/src/smooth_fit.rs rust/trajectory/src/smooth_fit/tests.rs
git commit -m "trajectory: add clamped C2 cubic-spline builder"
```

---

## Task 3: Adaptive `fit_c2_cubic` entry point

Wraps the spline builder in an adaptive knot-insertion loop: interpolate, find the
worst inter-knot error against the target function, insert a knot there, repeat
until max error ≤ tolerance. End slopes from finite differences of the target.

**Files:**
- Modify: `rust/trajectory/src/smooth_fit.rs`
- Test: `rust/trajectory/src/smooth_fit/tests.rs`

- [ ] **Step 1: Write the failing test**

Append to `rust/trajectory/src/smooth_fit/tests.rs`:

```rust
use nurbs::bezier::extract_bezier_pieces;
use nurbs::eval::eval;

#[test]
fn fit_c2_cubic_matches_smooth_fn_with_few_pieces() {
    // Target: a smooth bump on [0, 1]. Fit to 0.1 um tolerance.
    let f = |t: f64| (3.0 * t).sin() * (1.0 - t) * t;
    let tol = 1e-4;
    let curve = fit_c2_cubic(&f, 0.0, 1.0, tol).expect("fit succeeds");

    // Accuracy sampled densely WITHIN pieces (not just at knots).
    for i in 0..=2000 {
        let t = i as f64 / 2000.0;
        assert!(
            (eval(&curve.as_view(), t) - f(t)).abs() <= tol,
            "error at t={t}",
        );
    }
    // Compactness: a smooth bump needs few pieces, nowhere near hundreds.
    let n = extract_bezier_pieces(&curve).len();
    assert!(n < 40, "expected few pieces, got {n}");
}
```

- [ ] **Step 2: Run it to confirm it fails**

Run: `cargo nextest run -p trajectory -E 'test(fit_c2_cubic_matches_smooth_fn_with_few_pieces)'`
Expected: FAIL — `fit_c2_cubic` not found.

- [ ] **Step 3: Implement the adaptive fitter**

Add to `rust/trajectory/src/smooth_fit.rs`:

```rust
use nurbs::bezier::bezier_pieces_to_nurbs;
use nurbs::ScalarNurbs;

const MAX_KNOTS: usize = 4096;
const SAMPLES_PER_INTERVAL: usize = 16;

#[derive(Debug)]
pub struct FitError {
    pub achieved_mm: f64,
}

/// Adaptive clamped C2 cubic-spline fit of `f` on `[t_start, t_end]` to
/// `tolerance`. Knots are inserted at the worst-error location until the max
/// deviation (sampled within intervals) is within tolerance. End slopes are
/// taken from finite differences of `f`. Fails loudly if `MAX_KNOTS` is
/// exhausted before tolerance is met.
pub fn fit_c2_cubic<F: Fn(f64) -> f64>(
    f: &F,
    t_start: f64,
    t_end: f64,
    tolerance: f64,
) -> Result<ScalarNurbs<f64>, FitError> {
    let span = t_end - t_start;
    debug_assert!(span > 0.0 && tolerance > 0.0);

    let fd = (span * 1e-4).max(f64::MIN_POSITIVE);
    let yp0 = (f(t_start + fd) - f(t_start)) / fd;
    let ypn = (f(t_end) - f(t_end - fd)) / fd;

    // Start with start, midpoint, end so the first spline is non-degenerate.
    let mut knots = vec![t_start, t_start + 0.5 * span, t_end];

    loop {
        let values: Vec<f64> = knots.iter().map(|&t| f(t)).collect();
        let pieces = build_clamped_spline(&knots, &values, yp0, ypn);

        // Find the interval with the worst sampled error and the t of that max.
        let mut worst_err = 0.0_f64;
        let mut worst_t = f64::NAN;
        let mut worst_interval = 0usize;
        for (i, p) in pieces.iter().enumerate() {
            let (a, b) = (knots[i], knots[i + 1]);
            for s in 1..SAMPLES_PER_INTERVAL {
                let t = a + (b - a) * (s as f64 / SAMPLES_PER_INTERVAL as f64);
                let e = (p.evaluate(t) - f(t)).abs();
                if e > worst_err {
                    worst_err = e;
                    worst_t = t;
                    worst_interval = i;
                }
            }
        }

        if worst_err <= tolerance {
            return Ok(bezier_pieces_to_nurbs(&pieces));
        }
        if knots.len() >= MAX_KNOTS || !worst_t.is_finite() {
            return Err(FitError { achieved_mm: worst_err });
        }
        knots.insert(worst_interval + 1, worst_t);
    }
}
```

- [ ] **Step 4: Run the test**

Run: `cargo nextest run -p trajectory -E 'test(fit_c2_cubic_matches_smooth_fn_with_few_pieces)'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/trajectory/src/smooth_fit.rs rust/trajectory/src/smooth_fit/tests.rs
git commit -m "trajectory: add adaptive C2 cubic-spline fitter"
```

---

## Task 4: `ShapedSignal` smooth convolution evaluator

Extract `convolve_discrete`'s `fir_at` into a reusable evaluator so the fitter can
query the true smooth convolution `(x ⊛ w)(t)` at arbitrary `t`.

**Files:**
- Modify: `rust/trajectory/src/shaper.rs`
- Test: `rust/trajectory/src/shaper/tests.rs` (exists)

- [ ] **Step 1: Write the failing test**

Append to `rust/trajectory/src/shaper/tests.rs`:

```rust
#[test]
fn shaped_signal_eval_matches_convolve_output_samples() {
    use crate::kernel::build_smooth_zv_kernel;
    use nurbs::bezier::{bezier_pieces_to_nurbs, BezierPiece};
    use nurbs::eval::eval;

    // Smooth input s(t) on [0, 0.5].
    let t_end = 0.5_f64;
    let s = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 0.0,
        u_end: t_end,
        coeffs: vec![0.0, 0.0, 300.0 / t_end.powi(2), -200.0 / t_end.powi(3)],
    }]);
    let kernel = build_smooth_zv_kernel(0.8025 / 40.0);

    let linear = shape_axis(&s, &kernel, 0.0, t_end);
    let sig = ShapedSignal::new(&s, &kernel, 0.0, t_end);

    // The dense-linear output samples lie on the smooth convolution: ShapedSignal
    // must agree with the linear curve at its own knots (where linear == fir_at).
    for &u in linear.knots().iter() {
        if u >= 0.0 && u <= t_end {
            assert!(
                (sig.eval(u) - eval(&linear.as_view(), u)).abs() < 1e-9,
                "mismatch at u={u}",
            );
        }
    }
}
```

- [ ] **Step 2: Run it to confirm it fails**

Run: `cargo nextest run -p trajectory -E 'test(shaped_signal_eval_matches_convolve_output_samples)'`
Expected: FAIL — `ShapedSignal` not found.

- [ ] **Step 3: Implement `ShapedSignal`**

Add to `rust/trajectory/src/shaper.rs` (reusing the existing `eval_clamped` and
`eval_kernel` private fns):

```rust
pub struct ShapedSignal<'a> {
    input_samples: Vec<f64>,
    input_lo: f64,
    dt_in: f64,
    n_input: usize,
    kernel: &'a PiecewisePolynomialKernel<f64>,
    k_lo: f64,
    k_hi: f64,
}

impl<'a> ShapedSignal<'a> {
    pub fn new(
        padded: &ScalarNurbs<f64>,
        kernel: &'a PiecewisePolynomialKernel<f64>,
        t_start: f64,
        t_end: f64,
    ) -> Self {
        let (k_lo, k_hi) = kernel.support();
        let kernel_width = k_hi - k_lo;
        let dt_in = kernel_width / (INPUT_SAMPLES_PER_KERNEL_WIDTH as f64);
        let input_lo = t_start + k_lo;
        let input_hi = t_end + k_hi;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let n_input = ((input_hi - input_lo) / dt_in).ceil() as usize + 1;
        let input_samples = (0..n_input)
            .map(|i| eval_clamped(padded, input_lo + (i as f64) * dt_in))
            .collect();
        Self { input_samples, input_lo, dt_in, n_input, kernel, k_lo, k_hi }
    }

    pub fn eval(&self, t: f64) -> f64 {
        let j_lo_f = (t - self.k_hi - self.input_lo) / self.dt_in;
        let j_hi_f = (t - self.k_lo - self.input_lo) / self.dt_in;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let j_lo = (j_lo_f.floor() as isize).max(0) as usize;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let j_hi = ((j_hi_f.ceil() as isize) + 1).min(self.n_input as isize) as usize;
        let mut acc = 0.0_f64;
        for j in j_lo..j_hi {
            let t_j = self.input_lo + (j as f64) * self.dt_in;
            let w = eval_kernel(self.kernel, t - t_j);
            acc += self.input_samples[j] * w * self.dt_in;
        }
        acc
    }
}
```

- [ ] **Step 4: Run the test**

Run: `cargo nextest run -p trajectory -E 'test(shaped_signal_eval_matches_convolve_output_samples)'`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/trajectory/src/shaper.rs rust/trajectory/src/shaper/tests.rs
git commit -m "trajectory: add ShapedSignal smooth convolution evaluator"
```

---

## Task 5: Wire `emit_shaped` to the smooth fitter

Replace the `shape_axis` + `refit_to_cubic` pair with `ShapedSignal` + `fit_c2_cubic`.

**Files:**
- Modify: `rust/trajectory/src/emit_shaped.rs:88-101`

- [ ] **Step 1: Read the current shaping block**

In `rust/trajectory/src/emit_shaped.rs`, locate the per-axis block that calls
`shape_axis(&padded, kernel, t_start, t_end)` and then `refit_to_cubic(...)`
(around lines 88-101).

- [ ] **Step 2: Replace it**

Change the shaping/refit block so the shaped axis is produced by the smooth fit.
Replace:

```rust
                shape_axis(&padded, kernel, t_start, t_end)
            } else {
                fitted.axes[axis].clone()
            };

            if !axis_is_constant {
                axis_shaped =
                    refit_to_cubic(&axis_shaped, REFIT_TOLERANCE_MM).map_err(|detail| {
                        ShapeError::FitFailure {
                            index: seg_idx,
                            detail,
                        }
                    })?;
            }
```

with:

```rust
                let sig = crate::shaper::ShapedSignal::new(&padded, kernel, t_start, t_end);
                crate::smooth_fit::fit_c2_cubic(&|t| sig.eval(t), t_start, t_end, REFIT_TOLERANCE_MM)
                    .map_err(|e| ShapeError::FitFailure {
                        index: seg_idx,
                        detail: nurbs::algebra::FitError::ToleranceNotReached {
                            achieved_mm: e.achieved_mm,
                            at_degree: 3,
                        },
                    })?
            } else {
                fitted.axes[axis].clone()
            };
```

Remove the now-dead `use crate::shaper::shape_axis;` and
`use crate::refit::{refit_to_cubic, REFIT_TOLERANCE_MM};` imports, and add
`use crate::refit::REFIT_TOLERANCE_MM;` (the tolerance constant is still used).
Adjust the surrounding `let axis_shaped = if !axis_is_constant { ... }` binding so
it no longer needs `mut`.

- [ ] **Step 3: Build and run the existing emit_shaped tests**

Run: `cargo nextest run -p trajectory -E 'test(emit_shaped)'`
Expected: PASS (existing emit behavior preserved within tolerance).

- [ ] **Step 4: Run the full trajectory suite**

Run: `cargo nextest run -p trajectory`
Expected: PASS, except possibly piece-count assertions in old tests that hard-code
the dense-linear count — if any fail, they are asserting the old defect; update
them to the new (smaller) counts and note it in the commit.

- [ ] **Step 5: Commit**

```bash
git add rust/trajectory/src/emit_shaped.rs
git commit -m "trajectory: emit shaped axes via C2 cubic-spline fit of smooth convolution"
```

---

## Task 6: Regression test — piece count, accuracy, smoothness

Lock in the fix end-to-end through `emit_shaped`.

**Files:**
- Test: `rust/trajectory/src/emit_shaped/tests.rs` (create if absent; otherwise append)

- [ ] **Step 1: Write the test**

Add a test that runs `emit_shaped` for a single straight 100 mm X move with a
40 Hz SmoothZv kernel and asserts the three properties. Build the `FittedSegment`
input as the pre-shaper `s(t)` cubic and a passthrough Y/Z. Use the public
`emit_shaped` API. Concretely:

```rust
#[test]
fn straight_move_emits_few_smooth_pieces() {
    use crate::fit::FittedSegment;
    use crate::kernel::build_smooth_zv_kernel;
    use nurbs::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces, BezierPiece};

    let t_end = 0.8_f64;
    let x = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 0.0,
        u_end: t_end,
        coeffs: vec![0.0, 0.0, 300.0 / t_end.powi(2), -200.0 / t_end.powi(3)],
    }]);
    let constant = bezier_pieces_to_nurbs(&[BezierPiece {
        u_start: 0.0,
        u_end: t_end,
        coeffs: vec![0.0],
    }]);
    let fitted = FittedSegment {
        axes: [x, constant.clone(), constant],
        t_start: 0.0,
        t_end,
    };
    let kernel = build_smooth_zv_kernel(0.8025 / 40.0);
    let kernels = [Some(kernel.clone()), Some(kernel.clone()), Some(kernel), None];
    let meta = [EmitSegmentMeta {
        e_mode: geometry::segment::EMode::CoupledToXy,
        extrusion_per_xy_mm: 0.0,
    }];

    let out = emit_shaped(
        &[fitted],
        &meta,
        &kernels,
        &[],
        &PerAxisHistory::empty(),
        0.0,
        t_end,
    )
    .expect("emit_shaped");

    // Compactness: the X axis must be a handful of pieces, not ~hundreds.
    let n = extract_bezier_pieces(&out[0].axes[0]).len();
    assert!(n < 50, "expected << 50 pieces for a straight move, got {n}");

    // Smoothness (C2): 2nd derivative continuous across X-axis piece joints.
    let pieces = extract_bezier_pieces(&out[0].axes[0]);
    for i in 0..pieces.len() - 1 {
        let l = pieces[i].differentiate().differentiate();
        let r = pieces[i + 1].differentiate().differentiate();
        let j = pieces[i].u_end;
        let scale = l.evaluate(j).abs().max(1.0);
        assert!(
            (l.evaluate(j) - r.evaluate(j)).abs() <= 1e-6 * scale,
            "accel step at joint {i}",
        );
    }
}
```

If `FittedSegment`'s fields differ, read `rust/trajectory/src/fit.rs` for the exact
struct shape and adjust the literal. If `emit_shaped/tests.rs` does not exist, add
`#[cfg(test)] mod tests;` at the bottom of `emit_shaped.rs` and create the file
with `use super::*;` plus the test.

- [ ] **Step 2: Run the test**

Run: `cargo nextest run -p trajectory -E 'test(straight_move_emits_few_smooth_pieces)'`
Expected: PASS (n is a handful; no accel steps).

- [ ] **Step 3: Commit**

```bash
git add rust/trajectory/src/emit_shaped.rs rust/trajectory/src/emit_shaped/tests.rs
git commit -m "trajectory: regression test for compact, C2-smooth shaped output"
```

---

## Task 7: Remove the dead linear-shaper / weak-refit code

**Files:**
- Modify: `rust/trajectory/src/shaper.rs` (remove `shape_axis`, `convolve_discrete`, now-unused consts)
- Modify: `rust/trajectory/src/refit.rs` (remove `refit_to_cubic`, `split_at_midpoints`, keep `REFIT_TOLERANCE_MM`)
- Modify: their `tests.rs` (remove tests of the removed fns)

- [ ] **Step 1: Confirm no remaining callers**

Run from `rust/`:
```bash
grep -rn "shape_axis\|convolve_discrete\|refit_to_cubic" trajectory/src --include=*.rs | grep -v "fn shape_axis\|fn convolve_discrete\|fn refit_to_cubic"
```
Expected: no non-definition hits outside test modules that you are removing. If
there are other callers, STOP and reassess — do not delete.

- [ ] **Step 2: Delete the dead functions and their tests**

Remove `shape_axis` and `convolve_discrete` from `shaper.rs` (keep `ShapedSignal`,
`eval_clamped`, `eval_kernel`, `OUTPUT_SAMPLES_PER_KERNEL_WIDTH` only if still
referenced — otherwise remove). Remove `refit_to_cubic` and `split_at_midpoints`
from `refit.rs` but keep `pub const REFIT_TOLERANCE_MM`. Delete the corresponding
tests in `shaper/tests.rs` and `refit/tests.rs`.

- [ ] **Step 3: Build and run the whole trajectory suite**

Run: `cargo nextest run -p trajectory`
Expected: PASS, no dead-code or unused-import warnings.

- [ ] **Step 4: Run the workspace suite to catch downstream fallout**

Run: `cargo nextest run`
Expected: PASS except the known pre-existing failures unrelated to this work
(`fixture_4`, `fixture_7`, and the bridge `shutdown` test).

- [ ] **Step 5: Commit**

```bash
git add rust/trajectory/src/shaper.rs rust/trajectory/src/shaper/tests.rs rust/trajectory/src/refit.rs rust/trajectory/src/refit/tests.rs
git commit -m "trajectory: remove dead linear-shaper output and weak Hermite refit"
```

---

## Self-review notes (for the planner, not steps)

- **Spec coverage:** smooth-fit-of-true-convolution (Tasks 3-5), C2 (Tasks 2/3/6),
  compactness (Tasks 3/6), 0.1 µm accuracy (Tasks 3/6), uniform cubic (spline is
  cubic), within-segment only (per-segment in `emit_shaped`), drop linear output
  (Task 7). All covered.
- **Open questions resolved:** new C2 spline fitter (not `fit_hermite_c1`, which is
  endpoint-only at degree 3 and C1); fit target is `ShapedSignal::eval` (the
  accurate `fir_at` quadrature); fit-sampling density is `SAMPLES_PER_INTERVAL`
  within the adaptive loop.
- **Type consistency:** `fit_c2_cubic` returns `Result<ScalarNurbs<f64>, FitError>`
  with `FitError { achieved_mm }`; `emit_shaped` maps it into the existing
  `ShapeError::FitFailure` (verify the variant fields against `lib.rs` when wiring).
- **Watch:** confirm `FittedSegment` field names (Task 6) and `ShapeError::FitFailure`
  shape (Task 5) against the source before writing the literals.
