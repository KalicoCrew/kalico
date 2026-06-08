from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional, Union


@dataclass
class Move:
    kind: str
    x: Optional[float]
    y: Optional[float]
    line_no: int


@dataclass
class Arc:
    kind: str
    x: Optional[float]
    y: Optional[float]
    i: Optional[float]
    j: Optional[float]
    line_no: int


@dataclass
class Marker:
    reason: str
    line_no: int


Token = Union[Move, Arc, Marker]

_PARAM_RE = re.compile(r"([A-Z])(-?\d+(?:\.\d+)?)")


def _strip_comment(line: str) -> str:
    if ";" in line:
        line = line.split(";", 1)[0]
    return line.strip()


def _parse_params(rest: str) -> dict[str, float]:
    return {m.group(1): float(m.group(2)) for m in _PARAM_RE.finditer(rest)}


def parse(text: str) -> list[Token]:
    tokens: list[Token] = []
    for line_no, raw in enumerate(text.splitlines(), start=1):
        line = _strip_comment(raw)
        if not line:
            continue
        head, _, rest = line.partition(" ")
        head = head.upper()
        if head == "G1":
            params = _parse_params(rest)
            x = params.get("X")
            y = params.get("Y")
            if x is None and y is None:
                tokens.append(Marker(reason="Z_only", line_no=line_no))
            else:
                tokens.append(Move(kind="G1", x=x, y=y, line_no=line_no))
        elif head in ("G2", "G3"):
            params = _parse_params(rest)
            tokens.append(
                Arc(
                    kind=head,
                    x=params.get("X"),
                    y=params.get("Y"),
                    i=params.get("I"),
                    j=params.get("J"),
                    line_no=line_no,
                )
            )
        elif head == "G0":
            tokens.append(Marker(reason="G0", line_no=line_no))
        elif head and head[0] in ("G", "M", "T"):
            tokens.append(Marker(reason=head, line_no=line_no))
        # Anything else: silently skip (e.g., raw values, header garbage).
    return tokens
