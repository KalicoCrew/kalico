from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.interpolate import BSpline

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.fitter_prototype.classify import VertexLabel, classify_polyline
from scripts.fitter_prototype.fit import (
    chord_length_parameterize,
    fit_smooth_run,
)
from scripts.fitter_prototype.params import FitterParams


_PARAM_RE = re.compile(r"([A-Za-z])([-+]?(?:\d+(?:\.\d*)?|\.\d+))")


@dataclass
class Modal:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    e: float = 0.0
    f: float | None = None
    absolute_xyz: bool = True
    absolute_e: bool = True


@dataclass
class G1Point:
    raw: str
    x: float
    y: float
    z: float
    e: float
    f: float | None
    line_no: int


@dataclass
class Run:
    start_x: float
    start_y: float
    start_z: float
    start_e: float
    start_f: float | None
    absolute_e: bool
    points: list[G1Point]


@dataclass(frozen=True)
class Stats:
    runs_converted: int = 0
    g1_lines_replaced: int = 0
    g5_lines_emitted: int = 0


def _strip_comment(line: str) -> tuple[str, str]:
    if ";" not in line:
        return line.rstrip("\n"), ""
    code, comment = line.rstrip("\n").split(";", 1)
    return code.rstrip(), ";" + comment


def _head_and_params(line: str) -> tuple[str, dict[str, float]]:
    code, _ = _strip_comment(line)
    parts = code.strip().split(maxsplit=1)
    if not parts:
        return "", {}
    head = parts[0].upper()
    rest = parts[1] if len(parts) > 1 else ""
    return head, {m.group(1).upper(): float(m.group(2)) for m in _PARAM_RE.finditer(rest)}


def _fmt(value: float) -> str:
    if abs(value) < 0.0000005:
        value = 0.0
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    return text if text and text != "-0" else "0"


def _format_g5(
    x: float,
    y: float,
    i: float,
    j: float,
    p: float,
    q: float,
    z: float | None,
    e: float | None,
    f: float | None,
) -> str:
    fields = [
        "G5",
        f"X{_fmt(x)}",
        f"Y{_fmt(y)}",
        f"I{_fmt(i)}",
        f"J{_fmt(j)}",
        f"P{_fmt(p)}",
        f"Q{_fmt(q)}",
    ]
    if z is not None:
        fields.append(f"Z{_fmt(z)}")
    if e is not None:
        fields.append(f"E{_fmt(e)}")
    if f is not None:
        fields.append(f"F{_fmt(f)}")
    return " ".join(fields)


def _is_convertible_g1(
    line: str,
    modal: Modal,
    include_travel: bool,
) -> G1Point | None:
    head, params = _head_and_params(line)
    if head not in {"G1", "G01"}:
        return None
    if not modal.absolute_xyz:
        return None
    has_xy = "X" in params or "Y" in params
    if not has_xy:
        return None
    x = params.get("X", modal.x)
    y = params.get("Y", modal.y)
    z = params.get("Z", modal.z)
    e_param = params.get("E")
    e = modal.e if e_param is None else (
        e_param if modal.absolute_e else modal.e + e_param
    )
    f = params.get("F", modal.f)
    if not include_travel and e <= modal.e + 1e-9:
        return None
    if math.hypot(x - modal.x, y - modal.y) < 1e-9:
        return None
    return G1Point(raw=line, x=x, y=y, z=z, e=e, f=f, line_no=0)


def _update_modal_from_line(line: str, modal: Modal) -> None:
    head, params = _head_and_params(line)
    if not head:
        return
    if head in {"G90"}:
        modal.absolute_xyz = True
    elif head in {"G91"}:
        modal.absolute_xyz = False
    elif head in {"M82"}:
        modal.absolute_e = True
    elif head in {"M83"}:
        modal.absolute_e = False
    elif head in {"G92"}:
        if "X" in params:
            modal.x = params["X"]
        if "Y" in params:
            modal.y = params["Y"]
        if "Z" in params:
            modal.z = params["Z"]
        if "E" in params:
            modal.e = params["E"]
    elif head in {"G0", "G00", "G1", "G01", "G2", "G02", "G3", "G03", "G5"}:
        if modal.absolute_xyz:
            modal.x = params.get("X", modal.x)
            modal.y = params.get("Y", modal.y)
            modal.z = params.get("Z", modal.z)
        if "E" in params:
            modal.e = params["E"] if modal.absolute_e else modal.e + params["E"]
        if "F" in params:
            modal.f = params["F"]


def _run_has_smooth_interior(points: np.ndarray, params: FitterParams) -> bool:
    if len(points) < 3:
        return False
    labels = classify_polyline(points, params)
    return bool(labels) and all(label == VertexLabel.SMOOTH for label in labels)


def _emit_fit(run: Run, params: FitterParams) -> list[str] | None:
    pts = np.asarray(
        [[run.start_x, run.start_y], *[[p.x, p.y] for p in run.points]],
        dtype=float,
    )
    if len(pts) < 4 or not _run_has_smooth_interior(pts, params):
        return None

    fitted = fit_smooth_run(
        pts,
        source_vertex_range=(run.points[0].line_no, run.points[-1].line_no),
        params=params,
    )
    if fitted.max_residual > params.eps_chord_mm:
        return None

    spline = BSpline(
        fitted.knots,
        fitted.control_points,
        fitted.degree,
        extrapolate=False,
    )
    derivative = spline.derivative()
    breakpoints = np.unique(
        fitted.knots[fitted.degree : -(fitted.degree)]
    )
    breakpoints = breakpoints[
        (breakpoints >= fitted.knots[fitted.degree] - 1e-12)
        & (breakpoints <= fitted.knots[-fitted.degree - 1] + 1e-12)
    ]
    if len(breakpoints) < 2:
        return None

    t_data = chord_length_parameterize(pts)
    z_data = np.asarray([run.start_z, *[p.z for p in run.points]], dtype=float)
    e_data = np.asarray([run.start_e, *[p.e for p in run.points]], dtype=float)
    f_values = [p.f for p in run.points if p.f is not None]
    feed = f_values[-1] if f_values else run.start_f

    lines: list[str] = []
    last_xy = pts[0]
    last_e_abs = run.start_e
    for idx, (t0, t1) in enumerate(zip(breakpoints[:-1], breakpoints[1:])):
        dt = float(t1 - t0)
        if dt <= 1e-12:
            continue
        p0 = np.asarray(spline(t0), dtype=float)
        p3 = np.asarray(spline(t1), dtype=float)
        if idx == 0:
            p0 = pts[0]
        if idx == len(breakpoints) - 2:
            p3 = pts[-1]
        d0 = np.asarray(derivative(t0), dtype=float) * dt
        d1 = np.asarray(derivative(t1), dtype=float) * dt
        i, j = d0 / 3.0
        pp, qq = -d1 / 3.0
        z = float(np.interp(t1, t_data, z_data))
        e_abs = float(np.interp(t1, t_data, e_data))
        e = e_abs if run.absolute_e else e_abs - last_e_abs
        if math.hypot(*(p3 - last_xy)) < 1e-9:
            continue
        lines.append(
            _format_g5(
                x=float(p3[0]),
                y=float(p3[1]),
                i=float(i),
                j=float(j),
                p=float(pp),
                q=float(qq),
                z=z if abs(z - run.start_z) > 1e-9 else None,
                e=e,
                f=feed if idx == 0 else None,
            )
        )
        last_xy = p3
        last_e_abs = e_abs
    return lines or None


def postprocess(
    text: str,
    params: FitterParams,
    min_points: int,
    include_travel: bool,
) -> tuple[str, Stats]:
    modal = Modal()
    out: list[str] = []
    run: Run | None = None
    stats = Stats()

    def flush() -> None:
        nonlocal run, stats
        if run is None:
            return
        replacement = None
        if len(run.points) + 1 >= min_points:
            replacement = _emit_fit(run, params)
        if replacement is None:
            out.extend(p.raw.rstrip("\n") for p in run.points)
        else:
            out.extend(replacement)
            stats = Stats(
                runs_converted=stats.runs_converted + 1,
                g1_lines_replaced=stats.g1_lines_replaced + len(run.points),
                g5_lines_emitted=stats.g5_lines_emitted + len(replacement),
            )
        run = None

    for line_no, line in enumerate(text.splitlines(), start=1):
        candidate = _is_convertible_g1(line, modal, include_travel)
        if candidate is None:
            flush()
            out.append(line.rstrip("\n"))
            _update_modal_from_line(line, modal)
            continue
        candidate.line_no = line_no
        if run is None:
            run = Run(
                start_x=modal.x,
                start_y=modal.y,
                start_z=modal.z,
                start_e=modal.e,
                start_f=modal.f,
                absolute_e=modal.absolute_e,
                points=[],
            )
        run.points.append(candidate)
        _update_modal_from_line(line, modal)

    flush()
    return "\n".join(out) + ("\n" if text.endswith("\n") else ""), stats


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Prototype post-processor: replace smooth dense G1 XY runs with G5 cubic splines."
    )
    ap.add_argument("input", type=Path)
    ap.add_argument("output", type=Path)
    ap.add_argument("--eps-chord-mm", type=float, default=0.025)
    ap.add_argument("--theta-smooth-deg", type=float, default=15.0)
    ap.add_argument("--min-points", type=int, default=8)
    ap.add_argument(
        "--include-travel",
        action="store_true",
        help="also convert non-extruding XY G1 runs; default converts only increasing-E runs",
    )
    args = ap.parse_args()

    params = FitterParams(
        eps_chord_mm=args.eps_chord_mm,
        theta_smooth_deg=args.theta_smooth_deg,
    )
    output, stats = postprocess(
        args.input.read_text(),
        params=params,
        min_points=args.min_points,
        include_travel=args.include_travel,
    )
    args.output.write_text(output)
    print(
        f"converted {stats.runs_converted} runs; "
        f"replaced {stats.g1_lines_replaced} G1 lines with "
        f"{stats.g5_lines_emitted} G5 lines"
    )


if __name__ == "__main__":
    main()
