from __future__ import annotations

import numpy as np

from scripts.fitter_prototype.corner_blend import (
    make_slot,
    placeholder_finalize,
)
from scripts.fitter_prototype.params import FitterParams


def test_make_slot_unit_tangents():
    prev_pt = np.array([0.0, 0.0])
    corner = np.array([1.0, 0.0])
    next_pt = np.array([1.0, 1.0])
    slot = make_slot(prev_pt, corner, next_pt, FitterParams())
    np.testing.assert_array_equal(slot.position, corner)
    np.testing.assert_allclose(slot.t_in, [1.0, 0.0])
    np.testing.assert_allclose(slot.t_out, [0.0, 1.0])
    assert slot.seg_len_in == 1.0
    assert slot.seg_len_out == 1.0
    assert slot.tolerance_budget == FitterParams().blend_tolerance_mm


def test_placeholder_finalize_returns_4_control_points():
    prev_pt = np.array([0.0, 0.0])
    corner = np.array([1.0, 0.0])
    next_pt = np.array([1.0, 1.0])
    slot = make_slot(prev_pt, corner, next_pt, FitterParams())
    cps = placeholder_finalize(slot)
    assert cps.shape == (4, 2)
    # First and last control points are 1/3 along incident segments from corner.
    np.testing.assert_allclose(cps[0], corner - slot.t_in * slot.seg_len_in / 3)
    np.testing.assert_allclose(
        cps[-1], corner + slot.t_out * slot.seg_len_out / 3
    )
    # Middle two collapse to the corner (Pateloup default).
    np.testing.assert_allclose(cps[1], corner)
    np.testing.assert_allclose(cps[2], corner)
