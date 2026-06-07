from __future__ import annotations

import numpy as np

from scripts.fitter_prototype.output import FittedNurbs
from scripts.fitter_prototype.params import FitterParams
from scripts.fitter_prototype.run import process_gcode


def test_end_to_end_smooth_polyline_produces_fit():
    text = "\n".join(f"G1 X{i * 0.1} Y{i * 0.1}" for i in range(50))
    segs = process_gcode(text, FitterParams())
    fitted = [s for s in segs if isinstance(s, FittedNurbs)]
    assert len(fitted) == 1
    assert fitted[0].max_residual < 1e-6


def test_end_to_end_with_arc_and_corner():
    text = """
G1 X0 Y0
G1 X10 Y0
G1 X10 Y10
G2 X20 Y20 I10 J0
G1 X20 Y30
"""
    segs = process_gcode(text, FitterParams())
    kinds = [type(s).__name__ for s in segs]
    assert "JunctionDeviation" in kinds
    assert "ArcPassthrough" in kinds


def test_end_to_end_smoothable_corner_emits_slot():
    angle_30_deg = np.deg2rad(30)
    text = f"""
G1 X0 Y0
G1 X10 Y0
G1 X{10 + 10 * np.cos(angle_30_deg)} Y{10 * np.sin(angle_30_deg)}
G1 X{20 + 10 * np.cos(angle_30_deg)} Y{20 * np.sin(angle_30_deg)}
"""
    segs = process_gcode(text, FitterParams())
    kinds = [type(s).__name__ for s in segs]
    assert "CornerBlendSlot" in kinds
