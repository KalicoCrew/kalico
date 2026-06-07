#!/usr/bin/env python3
"""Generate a fixed test corpus by evaluating curves with NURBS-Python (geomdl).

The output JSON is checked into source control as the ground truth for the
oracle test in tests/geomdl_oracle.rs.

Run with:
    pip install geomdl
    python tests/scripts/generate_geomdl_corpus.py > tests/data/geomdl_corpus.json
"""

import json

from geomdl import NURBS, BSpline


def linear_curve():
    c = BSpline.Curve()
    c.degree = 1
    c.ctrlpts = [[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]]
    c.knotvector = [0.0, 0.0, 1.0, 1.0]
    return c


def quadratic_arc():
    c = NURBS.Curve()
    c.degree = 2
    c.ctrlptsw = [
        [1.0, 0.0, 0.0, 1.0],
        [0.7071067811865476, 0.7071067811865476, 0.0, 0.7071067811865476],
        [0.0, 1.0, 0.0, 1.0],
    ]
    c.knotvector = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]
    return c


def cubic_curve():
    c = BSpline.Curve()
    c.degree = 3
    c.ctrlpts = [
        [0.0, 0.0, 0.0],
        [1.0, 2.0, 0.0],
        [3.0, 2.0, 1.0],
        [4.0, 0.0, 0.0],
    ]
    c.knotvector = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]
    return c


def serialize(name, curve):
    return {
        "name": name,
        "degree": curve.degree,
        "knots": list(curve.knotvector),
        "control_points": [list(p) for p in curve.ctrlpts],
        "weights": list(curve.weights)
        if hasattr(curve, "weights") and curve.weights
        else None,
        "samples": [
            {"u": u, "point": curve.evaluate_single(u)}
            for u in [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0]
        ],
    }


def main():
    corpus = {
        "curves": [
            serialize("linear", linear_curve()),
            serialize("quadratic_arc_rational", quadratic_arc()),
            serialize("cubic_bspline", cubic_curve()),
        ],
    }
    print(json.dumps(corpus, indent=2))


if __name__ == "__main__":
    main()
