from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from scripts.fitter_prototype.classify import VertexLabel, classify_polyline
from scripts.fitter_prototype.corner_blend import make_slot
from scripts.fitter_prototype.fit import fit_smooth_run
from scripts.fitter_prototype.output import (
    ArcPassthrough,
    JunctionDeviation,
    Segment,
    serialize,
)
from scripts.fitter_prototype.params import FitterParams
from scripts.fitter_prototype.parser import parse
from scripts.fitter_prototype.reduce import ArcSegment, Polyline, reduce_tokens


def _process_polyline(
    poly: Polyline,
    params: FitterParams,
) -> list[Segment]:
    pts = poly.points
    out: list[Segment] = []
    if len(pts) < 2:
        return out
    if len(pts) == 2:
        # Single segment — no interior labels. The prototype skips trivial
        # 2-point polylines (a 2-point polyline has no corners and is
        # effectively a degenerate fit with one piece).
        return out
    labels = classify_polyline(pts, params)

    # Walk along, accumulating smooth runs; emit at each non-smooth label.
    # labels[i] corresponds to interior vertex pts[i+1].
    run_start = 0
    for i, label in enumerate(labels):
        v_idx = i + 1  # interior vertex index in pts
        if label == VertexLabel.SMOOTH:
            continue
        # Close the current run including v_idx as its endpoint.
        if v_idx - run_start >= 2:
            run_pts = pts[run_start : v_idx + 1]
            offset = poly.line_range[0]
            out.append(
                fit_smooth_run(
                    run_pts,
                    source_vertex_range=(
                        offset + run_start,
                        offset + v_idx,
                    ),
                    params=params,
                )
            )
        # Emit corner-blend or junction-deviation at v_idx.
        prev_pt = pts[v_idx - 1]
        corner = pts[v_idx]
        next_pt = pts[v_idx + 1]
        if label == VertexLabel.SMOOTHABLE_CORNER:
            out.append(make_slot(prev_pt, corner, next_pt, params))
        else:  # HARD_CORNER
            t_in = corner - prev_pt
            t_out = next_pt - corner
            cos = float(
                np.dot(t_in, t_out)
                / (
                    np.linalg.norm(t_in) * np.linalg.norm(t_out)
                    + 1e-12
                )
            )
            cos = max(-1.0, min(1.0, cos))
            angle = float(np.degrees(np.arccos(cos)))
            out.append(
                JunctionDeviation(
                    position=np.asarray(corner, dtype=float),
                    angle_deg=angle,
                )
            )
        run_start = v_idx  # next run starts at this corner

    # Close the final run.
    if len(pts) - 1 - run_start >= 2:
        offset = poly.line_range[0]
        run_pts = pts[run_start:]
        out.append(
            fit_smooth_run(
                run_pts,
                source_vertex_range=(
                    offset + run_start,
                    offset + len(pts) - 1,
                ),
                params=params,
            )
        )

    return out


def process_gcode(text: str, params: FitterParams) -> list[Segment]:
    segments: list[Segment] = []
    for geo in reduce_tokens(parse(text)):
        if isinstance(geo, Polyline):
            segments.extend(_process_polyline(geo, params))
        elif isinstance(geo, ArcSegment):
            segments.append(
                ArcPassthrough(
                    start=geo.start,
                    end=geo.end,
                    center=geo.center,
                    clockwise=geo.clockwise,
                )
            )
    return segments


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "gcode", type=Path, nargs="+", help="input .gcode file(s)"
    )
    ap.add_argument(
        "--out", type=Path, required=True, help="output directory"
    )
    ap.add_argument("--eps-chord-mm", type=float, default=0.025)
    ap.add_argument("--theta-smooth-deg", type=float, default=15.0)
    ap.add_argument("--theta-hard-deg", type=float, default=60.0)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    params = FitterParams(
        eps_chord_mm=args.eps_chord_mm,
        theta_smooth_deg=args.theta_smooth_deg,
        theta_hard_deg=args.theta_hard_deg,
    )
    for path in args.gcode:
        text = path.read_text()
        segments = process_gcode(text, params)
        out_path = args.out / f"{path.stem}.segments.json"
        out_path.write_text(json.dumps(serialize(segments), indent=2))
        print(f"{path.name}: {len(segments)} segments → {out_path}")


if __name__ == "__main__":
    main()
