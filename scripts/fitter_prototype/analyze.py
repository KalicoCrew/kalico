from __future__ import annotations

import json
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import BSpline

from scripts.fitter_prototype.corner_blend import placeholder_finalize
from scripts.fitter_prototype.output import (
    ArcPassthrough,
    CornerBlendSlot,
    FittedNurbs,
    JunctionDeviation,
    Segment,
)


def compute_stats(segments: list[Segment]) -> dict:
    counts = Counter(type(s).__name__ for s in segments)
    fitted = [s for s in segments if isinstance(s, FittedNurbs)]
    pieces_per_fit = [
        max(0, len(np.unique(s.knots[s.degree + 1 : -(s.degree + 1)])) + 1)
        for s in fitted
    ]
    return {
        "segment_counts": dict(counts),
        "fits": {
            "count": len(fitted),
            "max_residual_mm": [s.max_residual for s in fitted],
            "max_residual_p50": float(
                np.median([s.max_residual for s in fitted])
            )
            if fitted
            else 0.0,
            "max_residual_p95": float(
                np.percentile([s.max_residual for s in fitted], 95)
            )
            if fitted
            else 0.0,
            "pieces_per_fit_p50": float(np.median(pieces_per_fit))
            if pieces_per_fit
            else 0.0,
            "pieces_per_fit_p95": float(np.percentile(pieces_per_fit, 95))
            if pieces_per_fit
            else 0.0,
            "vertex_count_per_fit_p50": float(
                np.median(
                    [
                        s.source_vertex_range[1] - s.source_vertex_range[0]
                        for s in fitted
                    ]
                )
            )
            if fitted
            else 0.0,
        },
    }


def write_stats(segments: list[Segment], out_path: Path) -> None:
    out_path.write_text(json.dumps(compute_stats(segments), indent=2))


def plot_residual_histogram(segments: list[Segment], out_path: Path) -> None:
    residuals = [s.max_residual for s in segments if isinstance(s, FittedNurbs)]
    if not residuals:
        return
    residuals_um = np.asarray(residuals) * 1e3
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.hist(residuals_um, bins=50)
    ax.set_xlabel("max residual per fitted run (µm)")
    ax.set_ylabel("# runs")
    ax.set_title("Fitted run residual distribution")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_piece_count_histogram(segments: list[Segment], out_path: Path) -> None:
    fitted = [s for s in segments if isinstance(s, FittedNurbs)]
    if not fitted:
        return
    pieces = [
        max(0, len(np.unique(s.knots[s.degree + 1 : -(s.degree + 1)])) + 1)
        for s in fitted
    ]
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.hist(pieces, bins=range(1, max(pieces) + 2))
    ax.set_xlabel("Bezier pieces per fitted run")
    ax.set_ylabel("# runs")
    ax.set_title("Fit piece-count distribution")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_classification_breakdown(
    segments: list[Segment], out_path: Path
) -> None:
    counts = Counter(type(s).__name__ for s in segments)
    if not counts:
        return
    fig, ax = plt.subplots(figsize=(7, 4))
    labels = list(counts.keys())
    values = [counts[k] for k in labels]
    ax.bar(labels, values)
    ax.set_ylabel("# segments")
    ax.set_title("Output segment kind breakdown")
    fig.autofmt_xdate(rotation=30)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_geometry_overlay(
    segments: list[Segment],
    out_path: Path,
    max_runs: int = 50,
) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    n_drawn = 0
    for seg in segments:
        if isinstance(seg, FittedNurbs) and n_drawn < max_runs:
            t = np.linspace(
                seg.knots[seg.degree],
                seg.knots[-seg.degree - 1],
                200,
            )
            spline = BSpline(
                seg.knots,
                seg.control_points,
                seg.degree,
                extrapolate=False,
            )
            curve = spline(t)
            if np.any(np.isnan(curve[-1])):
                curve[-1] = seg.control_points[-1]
            ax.plot(curve[:, 0], curve[:, 1], linewidth=0.7, color="C0")
            n_drawn += 1
        elif isinstance(seg, CornerBlendSlot):
            cps = placeholder_finalize(seg)
            ax.plot(cps[:, 0], cps[:, 1], linewidth=0.5, color="C1", alpha=0.6)
        elif isinstance(seg, JunctionDeviation):
            ax.plot(
                seg.position[0],
                seg.position[1],
                "x",
                color="C3",
                markersize=4,
            )
        elif isinstance(seg, ArcPassthrough):
            r = float(np.linalg.norm(seg.start - seg.center))
            a0 = float(np.arctan2(*(seg.start - seg.center)[::-1]))
            a1 = float(np.arctan2(*(seg.end - seg.center)[::-1]))
            if seg.clockwise and a1 > a0:
                a1 -= 2 * np.pi
            elif not seg.clockwise and a1 < a0:
                a1 += 2 * np.pi
            angles = np.linspace(a0, a1, 50)
            xs = seg.center[0] + r * np.cos(angles)
            ys = seg.center[1] + r * np.sin(angles)
            ax.plot(xs, ys, linewidth=0.6, color="C2", alpha=0.7)
    ax.set_aspect("equal")
    ax.set_title(
        "Geometry overlay"
        " (fits=blue, blends=orange, junctions=red×, arcs=green)"
    )
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def render_all(segments: list[Segment], out_dir: Path, stem: str) -> None:
    plot_residual_histogram(segments, out_dir / f"{stem}.residuals.png")
    plot_piece_count_histogram(segments, out_dir / f"{stem}.pieces.png")
    plot_classification_breakdown(segments, out_dir / f"{stem}.kinds.png")
    plot_geometry_overlay(segments, out_dir / f"{stem}.overlay.png")
    write_stats(segments, out_dir / f"{stem}.stats.json")
