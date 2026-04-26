from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Union

import numpy as np


@dataclass
class FittedNurbs:
    control_points: np.ndarray
    knots: np.ndarray
    degree: int
    source_vertex_range: tuple[int, int]
    max_residual: float


@dataclass
class CornerBlendSlot:
    position: np.ndarray
    t_in: np.ndarray
    t_out: np.ndarray
    seg_len_in: float
    seg_len_out: float
    tolerance_budget: float
    default_family: str = "cubic_bezier"


@dataclass
class JunctionDeviation:
    position: np.ndarray
    angle_deg: float


@dataclass
class ArcPassthrough:
    start: np.ndarray
    end: np.ndarray
    center: np.ndarray
    clockwise: bool


Segment = Union[FittedNurbs, CornerBlendSlot, JunctionDeviation, ArcPassthrough]


def _to_jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, tuple):
        return list(value)
    return value


def serialize(segments: list[Segment]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for seg in segments:
        kind = type(seg).__name__
        body = {k: _to_jsonable(v) for k, v in seg.__dict__.items()}
        out.append({"kind": kind, "data": body})
    return out


def deserialize(payload: list[dict[str, Any]]) -> list[Segment]:
    out: list[Segment] = []
    for item in payload:
        kind = item["kind"]
        data = dict(item["data"])
        if kind == "FittedNurbs":
            out.append(
                FittedNurbs(
                    control_points=np.asarray(
                        data["control_points"], dtype=float
                    ),
                    knots=np.asarray(data["knots"], dtype=float),
                    degree=int(data["degree"]),
                    source_vertex_range=tuple(data["source_vertex_range"]),
                    max_residual=float(data["max_residual"]),
                )
            )
        elif kind == "CornerBlendSlot":
            out.append(
                CornerBlendSlot(
                    position=np.asarray(data["position"], dtype=float),
                    t_in=np.asarray(data["t_in"], dtype=float),
                    t_out=np.asarray(data["t_out"], dtype=float),
                    seg_len_in=float(data["seg_len_in"]),
                    seg_len_out=float(data["seg_len_out"]),
                    tolerance_budget=float(data["tolerance_budget"]),
                    default_family=str(
                        data.get("default_family", "cubic_bezier")
                    ),
                )
            )
        elif kind == "JunctionDeviation":
            out.append(
                JunctionDeviation(
                    position=np.asarray(data["position"], dtype=float),
                    angle_deg=float(data["angle_deg"]),
                )
            )
        elif kind == "ArcPassthrough":
            out.append(
                ArcPassthrough(
                    start=np.asarray(data["start"], dtype=float),
                    end=np.asarray(data["end"], dtype=float),
                    center=np.asarray(data["center"], dtype=float),
                    clockwise=bool(data["clockwise"]),
                )
            )
        else:
            raise ValueError(f"unknown segment kind: {kind}")
    return out
