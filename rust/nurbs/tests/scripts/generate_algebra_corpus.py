#!/usr/bin/env python3
"""Generate symbolic-reference corpus for NURBS algebra ops.

Uses sympy.integrate(...) for non-trivial convolution references and the
Cox–de Boor recursion for multi-piece NURBS evaluation. Hand-derived
arithmetic is used only where the closed form is a textbook polynomial
identity (e.g., u * u = u**2).

Run with:
    pip install sympy
    python rust/nurbs/tests/scripts/generate_algebra_corpus.py > rust/nurbs/tests/data/algebra_corpus.json
"""

import json

import sympy as sp


def linear_curve_data():
    return {
        "degree": 1,
        "knots": [0.0, 0.0, 1.0, 1.0],
        "control_points": [0.0, 1.0],
        "weights": None,
    }


def quadratic_curve_data():
    return {
        "degree": 2,
        "knots": [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        "control_points": [0.0, 0.0, 1.0],
        "weights": None,
    }


def multiply_fixture_linear_x_linear():
    """a(u) = u, b(u) = u, expected c(u) = u^2 (textbook identity)."""
    a = linear_curve_data()
    b = linear_curve_data()
    samples_u = [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0]
    samples = [{"u": u, "value": u * u} for u in samples_u]
    return {
        "name": "multiply_linear_squared",
        "operation": "multiply",
        "a": a,
        "b": b,
        "samples": samples,
    }


def multiply_fixture_quadratic_x_linear():
    """a(u) = u^2, b(u) = u, expected c(u) = u^3 (textbook identity)."""
    a = quadratic_curve_data()
    b = linear_curve_data()
    samples_u = [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0]
    samples = [{"u": u, "value": u ** 3} for u in samples_u]
    return {
        "name": "multiply_quadratic_x_linear",
        "operation": "multiply",
        "a": a,
        "b": b,
        "samples": samples,
    }


def cox_de_boor_basis(i, p, knots, var):
    """Symbolic Cox–de Boor basis function N_{i,p}(var) as a sympy Piecewise."""
    if p == 0:
        return sp.Piecewise(
            (sp.Integer(1), sp.And(var >= knots[i], var < knots[i + 1])),
            (sp.Integer(0), True),
        )
    left_denom = knots[i + p] - knots[i]
    right_denom = knots[i + p + 1] - knots[i + 1]
    left = sp.Integer(0)
    if left_denom != 0:
        left = (var - knots[i]) / left_denom * cox_de_boor_basis(i, p - 1, knots, var)
    right = sp.Integer(0)
    if right_denom != 0:
        right = (knots[i + p + 1] - var) / right_denom * cox_de_boor_basis(i + 1, p - 1, knots, var)
    return left + right


def evaluate_nurbs_symbolic(degree, knots_sym, cps, var, u_val):
    """Evaluate a polynomial NURBS at u_val via Cox–de Boor.

    For boundary u_val == last knot, reuse left-limit semantics by clamping
    slightly inside the rightmost span (the sympy basis uses half-open spans).
    """
    last = knots_sym[-1]
    # Half-open spans cause N_{i,p}(last) = 0 trivially. Approach from the left.
    if u_val == float(last):
        eps = sp.Rational(1, 10 ** 12)
        u_eval = sp.Float(u_val) - eps
    else:
        u_eval = sp.Float(u_val)
    n = len(cps)
    val = sp.Integer(0)
    for i in range(n):
        n_ip = cox_de_boor_basis(i, degree, knots_sym, var)
        val += sp.Float(cps[i]) * n_ip.subs(var, u_eval)
    return float(sp.simplify(val))


def sanity_check_evaluator():
    """Verify the Cox-de Boor evaluator against a known closed-form case before
    using it to generate fixture reference values. If this fires, the corpus
    is suspect — fix the evaluator before regenerating fixtures."""
    u = sp.symbols("u", real=True)
    # Single-piece quadratic: knots [0,0,0,1,1,1], cps [0,0,1] → polynomial u^2.
    # Eval at u=0.5 should give 0.25.
    knots = [sp.Rational(0), sp.Rational(0), sp.Rational(0),
             sp.Rational(1), sp.Rational(1), sp.Rational(1)]
    cps = [0.0, 0.0, 1.0]
    val = evaluate_nurbs_symbolic(2, knots, cps, u, 0.5)
    expected = 0.25  # 0.5^2 = 0.25
    assert abs(val - expected) < 1e-12, (
        f"Cox-de Boor evaluator failed sanity check: u=0.5 of u^2 returned {val}, expected {expected}"
    )

    # Linear: knots [0,0,1,1], cps [0,1] → polynomial u.
    knots_lin = [sp.Rational(0), sp.Rational(0), sp.Rational(1), sp.Rational(1)]
    cps_lin = [0.0, 1.0]
    val_lin = evaluate_nurbs_symbolic(1, knots_lin, cps_lin, u, 1.0 / 3.0)
    expected_lin = 1.0 / 3.0
    assert abs(val_lin - expected_lin) < 1e-12, (
        f"Cox-de Boor evaluator failed sanity check: u=1/3 of u returned {val_lin}, expected {expected_lin}"
    )


def multiply_fixture_with_interior_knot():
    """Multiply curves where one has an interior knot. Reference values via
    Cox–de Boor evaluation in sympy (so this test catches real bugs in our
    Rust pipeline, not just round-trip identities)."""
    a = {
        "degree": 2,
        "knots": [0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
        "control_points": [0.0, 1.0, 2.0, 3.0],
        "weights": None,
    }
    b = quadratic_curve_data()  # b(u) = u^2 via Bernstein cps [0, 0, 1]

    u = sp.symbols("u", real=True)
    a_knots = [sp.Rational(k).limit_denominator(1000) for k in a["knots"]]
    b_knots = [sp.Rational(k).limit_denominator(1000) for k in b["knots"]]

    samples = []
    for u_val in [0.1, 0.3, 0.5, 0.7, 0.9]:
        a_val = evaluate_nurbs_symbolic(a["degree"], a_knots, a["control_points"], u, u_val)
        b_val = evaluate_nurbs_symbolic(b["degree"], b_knots, b["control_points"], u, u_val)
        samples.append({"u": u_val, "value": a_val * b_val})
    return {
        "name": "multiply_with_interior_knot",
        "operation": "multiply",
        "a": a,
        "b": b,
        "samples": samples,
    }


def convolve_fixture_constant_x_constant():
    """x(u) = 2 on [0, 1], w(t) = 3 on [-0.5, 0.5]; closed-form triangle."""
    samples = []
    for u_val in [-0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5]:
        s_lo = max(0, u_val - 0.5)
        s_hi = min(1, u_val + 0.5)
        y = max(0, (s_hi - s_lo) * 2 * 3)
        samples.append({"u": u_val, "value": y})
    return {
        "name": "convolve_constant_x_constant",
        "operation": "convolve",
        "curve": {
            "degree": 1,
            "knots": [0.0, 0.0, 1.0, 1.0],
            "control_points": [2.0, 2.0],
            "weights": None,
        },
        "kernel": {
            "pieces": [
                {"u_start": -0.5, "u_end": 0.5, "coeffs": [3.0]},
            ],
        },
        "samples": samples,
    }


def absolute_to_pascal_shift(absolute, shift):
    """Convert absolute-monomial coefficients (Σ a_n * u^n) to
    Pascal-shifted-at-`shift` coefficients (Σ c_k * (u - shift)^k).

    Mirrors `algebra::absolute_to_pascal_shift` in the Rust crate. The kernel
    storage convention in `PiecewisePolynomialKernel::single_poly` is
    Pascal-shifted at the piece's `u_start`, so kernels expressed naturally
    around t=0 (e.g., bleeding-edge-v2's smooth_zv) need this conversion
    before they can be passed to `convolve`."""
    from math import comb

    d = len(absolute) - 1
    out = [0.0] * (d + 1)
    shift_pow = [1.0] * (d + 1)
    for k in range(1, d + 1):
        shift_pow[k] = shift_pow[k - 1] * shift
    for n in range(d + 1):
        for k in range(n + 1):
            out[k] += absolute[n] * comb(n, k) * shift_pow[n - k]
    return out


def convolve_fixture_smooth_zv_x_linear():
    """x(s) = s on [0, 1], w = bleeding-edge-v2 smooth_zv kernel (shaper_freq=1).

    Reference values via sympy.integrate — this is the whole point of having
    an oracle: cross-check our convolve against an independent symbolic
    integrator, not against a hand-derived closed form.

    The kernel coefficients are computed in absolute monomial form around t=0
    (Σ c_i * t^i) per Klipper's `init_smoother`, then converted to the
    Pascal-shifted-at-u_start basis used by `PiecewisePolynomialKernel`."""
    # Match init_smoother(...) for shaper_freq=1: descending raw coeffs become
    # ascending normalized coeffs c[i] for c[i] * t^i.
    raw_coeffs = [
        -118.4265334338076,
        5.861885495127615,
        29.52796003014231,
        -1.465471373781904,
        0.01966833207740377,
    ]
    smooth_time = 0.8025
    inv_t = 1.0 / smooth_time
    inv_t_n = inv_t
    n = len(raw_coeffs)
    c = [0.0] * n  # Σ c[i] * t^i  (absolute monomial around t=0)
    for i in range(n - 1, -1, -1):
        c[n - i - 1] = raw_coeffs[i] * inv_t_n
        inv_t_n *= inv_t
    half = smooth_time / 2

    s, u, t = sp.symbols("s u t", real=True)
    x_sym = s  # input x(s) = s on [0, 1]
    w_sym = sum(sp.Float(c[i]) * t ** i for i in range(n))
    integrand_full = x_sym * w_sym.subs(t, u - s)

    samples = []
    for u_val in [0.0, 0.5, 1.0]:
        s_lo = max(0.0, u_val - half)
        s_hi = min(1.0, u_val + half)
        if s_lo >= s_hi:
            samples.append({"u": u_val, "value": 0.0})
            continue
        integrand = integrand_full.subs(u, sp.Float(u_val))
        y_val = float(sp.integrate(integrand, (s, sp.Float(s_lo), sp.Float(s_hi))))
        samples.append({"u": u_val, "value": y_val})

    # Convert the absolute-monomial-around-t=0 coefficients to the kernel's
    # Pascal-shifted basis (shift = u_start = -half).
    c_shifted = absolute_to_pascal_shift(c, -half)

    kernel = {
        "pieces": [
            {"u_start": -half, "u_end": half, "coeffs": c_shifted},
        ],
    }
    return {
        "name": "convolve_smooth_zv_x_linear",
        "operation": "convolve",
        "curve": linear_curve_data(),
        "kernel": kernel,
        "samples": samples,
    }


def main():
    sanity_check_evaluator()
    fixtures = [
        multiply_fixture_linear_x_linear(),
        multiply_fixture_quadratic_x_linear(),
        multiply_fixture_with_interior_knot(),
        convolve_fixture_constant_x_constant(),
        convolve_fixture_smooth_zv_x_linear(),
    ]
    print(json.dumps({"fixtures": fixtures}, indent=2))


if __name__ == "__main__":
    main()
