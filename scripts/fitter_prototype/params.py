from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FitterParams:
    theta_smooth_deg: float = 15.0
    theta_hard_deg: float = 60.0
    seg_len_collapse_mm: float = 0.05
    degree: int = 3
    n_init_interior: int = 4
    eps_chord_mm: float = 0.025
    eps_iter_mm: float = 1e-9
    max_lspia_iter: int = 100
    max_refine_iter: int = 20
    n_chord_samples: int = 50
    blend_tolerance_mm: float = 0.050
