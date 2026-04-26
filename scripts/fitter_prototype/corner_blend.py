from __future__ import annotations

import numpy as np

from scripts.fitter_prototype.output import CornerBlendSlot
from scripts.fitter_prototype.params import FitterParams


def make_slot(
    prev_point: np.ndarray,
    corner: np.ndarray,
    next_point: np.ndarray,
    params: FitterParams,
) -> CornerBlendSlot:
    in_vec = corner - prev_point
    out_vec = next_point - corner
    in_len = float(np.linalg.norm(in_vec))
    out_len = float(np.linalg.norm(out_vec))
    t_in = in_vec / in_len if in_len > 1e-12 else np.zeros(2)
    t_out = out_vec / out_len if out_len > 1e-12 else np.zeros(2)
    return CornerBlendSlot(
        position=np.asarray(corner, dtype=float),
        t_in=np.asarray(t_in, dtype=float),
        t_out=np.asarray(t_out, dtype=float),
        seg_len_in=in_len,
        seg_len_out=out_len,
        tolerance_budget=params.blend_tolerance_mm,
    )


def placeholder_finalize(slot: CornerBlendSlot) -> np.ndarray:
    """Pateloup 2004 default cubic Bezier: control points at 1/3 along incident
    segments, middle two collapsed to the corner. NOT production shape selection
    — Layer 3 will replace this with dynamic-limit-aware shape selection per
    Tajima & Sencer 2016. Used only for prototype plotting.
    """
    p0 = slot.position - slot.t_in * (slot.seg_len_in / 3.0)
    p1 = slot.position
    p2 = slot.position
    p3 = slot.position + slot.t_out * (slot.seg_len_out / 3.0)
    return np.vstack([p0, p1, p2, p3])
