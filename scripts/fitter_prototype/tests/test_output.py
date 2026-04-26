from __future__ import annotations

import json

import numpy as np

from scripts.fitter_prototype.output import (
    ArcPassthrough,
    CornerBlendSlot,
    FittedNurbs,
    JunctionDeviation,
    deserialize,
    serialize,
)


def test_round_trip_fitted_nurbs():
    seg = FittedNurbs(
        control_points=np.array([[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]]),
        knots=np.array([0.0, 0.0, 0.0, 1.0, 1.0, 1.0]),
        degree=2,
        source_vertex_range=(0, 10),
        max_residual=1.5e-3,
    )
    js = json.dumps(serialize([seg]))
    rt = deserialize(json.loads(js))
    assert len(rt) == 1
    assert isinstance(rt[0], FittedNurbs)
    assert rt[0].degree == 2
    assert rt[0].source_vertex_range == (0, 10)
    np.testing.assert_array_equal(rt[0].control_points, seg.control_points)


def test_round_trip_mixed():
    segs = [
        FittedNurbs(
            control_points=np.zeros((4, 2)),
            knots=np.array([0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]),
            degree=3,
            source_vertex_range=(0, 5),
            max_residual=0.0,
        ),
        CornerBlendSlot(
            position=np.array([1.0, 2.0]),
            t_in=np.array([1.0, 0.0]),
            t_out=np.array([0.0, 1.0]),
            seg_len_in=0.5,
            seg_len_out=0.7,
            tolerance_budget=0.05,
        ),
        JunctionDeviation(position=np.array([3.0, 4.0]), angle_deg=90.0),
        ArcPassthrough(
            start=np.array([0.0, 0.0]),
            end=np.array([1.0, 0.0]),
            center=np.array([0.5, 0.0]),
            clockwise=True,
        ),
    ]
    rt = deserialize(json.loads(json.dumps(serialize(segs))))
    assert [type(x).__name__ for x in rt] == [
        "FittedNurbs",
        "CornerBlendSlot",
        "JunctionDeviation",
        "ArcPassthrough",
    ]
