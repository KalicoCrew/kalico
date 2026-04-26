#!/usr/bin/env python3
"""Generate symbolic-reference corpus for NURBS algebra ops via sympy.

Each fixture provides:
- multiply(a, b): NURBS a, NURBS b, sample evaluations of c = a*b
- convolve(curve, kernel): NURBS, kernel, sample evaluations of y = curve*kernel

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


def convolve_fixture_constant_x_constant():
    samples = []
    s, u = sp.symbols('s u', real=True)
    x_sym = sp.Piecewise((2, sp.And(s >= 0, s <= 1)), (0, True))
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
        convolve_fixture_constant_x_constant(),
    ]
    print(json.dumps({"fixtures": fixtures}, indent=2))


if __name__ == "__main__":
    main()
