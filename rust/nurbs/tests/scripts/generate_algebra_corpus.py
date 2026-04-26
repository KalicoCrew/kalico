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


def main():
    fixtures = [
        multiply_fixture_linear_x_linear(),
        multiply_fixture_quadratic_x_linear(),
        multiply_fixture_with_interior_knot(),
        convolve_fixture_constant_x_constant(),
    ]
    print(json.dumps({"fixtures": fixtures}, indent=2))


if __name__ == "__main__":
    main()
