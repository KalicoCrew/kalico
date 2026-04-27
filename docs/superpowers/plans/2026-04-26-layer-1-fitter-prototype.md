# Layer 1 Fitter Prototype Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Python prototype of the Layer 1 spline fitter (G-code parser + vertex classifier + LSPIA fit + parameterized corner-blend emission) that runs on real OrcaSlicer corpora and produces measured findings for the spike writeup.

**Architecture:** Offline batch pipeline: `gcode → tokens → polyline/arc segments → vertex labels → fitted NURBS + corner-blend slots + junction-deviation markers → JSON + plots`. Each module is pure-function-shaped; CLI runner orchestrates. No production-runtime concerns; correctness and inspectability are the goals.

**Tech Stack:** Python 3.9+, numpy (already in pyproject), scipy.interpolate.BSpline, matplotlib, pytest, ruff. Dependencies isolated in a `prototype` dependency group so they don't affect the main Kalico install.

**Spec:** `docs/superpowers/specs/2026-04-26-layer-1-fitter-prototype-design.md`

**Reference:** `CLAUDE.md` (Layer 1 description), `docs/superpowers/specs/2026-04-26-nurbs-algebra-design.md` (Layer 0 algebra design).

---

## File map

```
scripts/fitter_prototype/
├── __init__.py                — empty package marker
├── README.md                  — usage notes
├── corpus/                    — already populated with two .gcode files
├── params.py                  — config dataclass with defaults
├── output.py                  — typed segment dataclasses + JSON
├── parser.py                  — G-code tokenizer
├── reduce.py                  — tokens → polyline/arc segments
├── classify.py                — vertex labeler (CMLT-style)
├── corner_blend.py            — slot emitter + placeholder finalizer
├── fit.py                     — LSPIA + chord-bound refinement
├── analyze.py                 — corpus stats + matplotlib plots
├── run.py                     — CLI orchestration
└── tests/
    ├── __init__.py
    ├── test_parser.py
    ├── test_reduce.py
    ├── test_classify.py
    ├── test_corner_blend.py
    ├── test_fit.py
    ├── test_output.py
    └── test_end_to_end.py
```

Plus: `pyproject.toml` (add `prototype` dep group); eventual `docs/superpowers/spikes/2026-04-26-layer-1-fitter-spike.md` (D3 deliverable).

---

## Task 1: Project setup and dependency group

**Files:**
- Modify: `pyproject.toml`
- Create: `scripts/fitter_prototype/__init__.py`
- Create: `scripts/fitter_prototype/tests/__init__.py`
- Create: `scripts/fitter_prototype/README.md`

- [ ] **Step 1: Add prototype dependency group to pyproject.toml**

In `pyproject.toml`, inside the `[dependency-groups]` block, add a `prototype` group below the existing `dev` group:

```toml
[dependency-groups]
dev = [
  "ruff>=0.9.3",
  "pre-commit>=4.0.1",
  "pytest-xdist>=3.6.1",
  "pytest>=8.3.4",
]
prototype = [
  "scipy>=1.13",
  "matplotlib>=3.8",
]
```

- [ ] **Step 2: Sync dependencies**

Run: `uv sync --group dev --group prototype`
Expected: scipy and matplotlib installed; lockfile updated.

- [ ] **Step 3: Create empty package init files**

```bash
touch scripts/__init__.py
touch scripts/fitter_prototype/__init__.py
touch scripts/fitter_prototype/tests/__init__.py
```

The `scripts/__init__.py` is needed so `python -m scripts.fitter_prototype.run`
works. Existing scripts in `scripts/` are invoked as standalone files and
don't import from `scripts.*`, so adding the package marker is non-disruptive.

- [ ] **Step 4: Create README skeleton**

Create `scripts/fitter_prototype/README.md`:

```markdown
# Layer 1 Fitter Prototype

Python prototype for Layer 1 of the Kalico motion-planner rewrite. See
`docs/superpowers/specs/2026-04-26-layer-1-fitter-prototype-design.md` for
the design context.

## Run

    uv sync --group prototype
    uv run python -m scripts.fitter_prototype.run \
        scripts/fitter_prototype/corpus/voron_cube_arc_fitted.gcode \
        --out results/

## Test

    uv run pytest scripts/fitter_prototype/tests/
```

- [ ] **Step 5: Verify pytest discovers the new tests dir**

Run: `uv run pytest scripts/fitter_prototype/tests/ -v`
Expected: `no tests ran` (empty test dir, but no error).

- [ ] **Step 6: Commit**

```bash
git add pyproject.toml uv.lock scripts/fitter_prototype/
git commit -m "fitter: scaffold prototype package with dependency group"
```

---

## Task 2: Configuration dataclass

**Files:**
- Create: `scripts/fitter_prototype/params.py`

A single `FitterParams` dataclass holds all the knobs from the spec, with documented defaults.

- [ ] **Step 1: Write the dataclass**

Create `scripts/fitter_prototype/params.py`:

```python
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FitterParams:
    # --- Classifier ---
    theta_smooth_deg: float = 15.0
    theta_hard_deg: float = 60.0
    seg_len_collapse_mm: float = 0.05

    # --- Fitter ---
    degree: int = 3
    n_init_interior: int = 4
    eps_chord_mm: float = 0.025
    eps_iter_mm: float = 1e-9
    max_lspia_iter: int = 100
    max_refine_iter: int = 20
    n_chord_samples: int = 50

    # --- Corner blend ---
    blend_tolerance_mm: float = 0.050
```

- [ ] **Step 2: Smoke-test that it imports and instantiates**

```bash
uv run python -c "from scripts.fitter_prototype.params import FitterParams; print(FitterParams())"
```

Expected: prints the dataclass with defaults.

- [ ] **Step 3: Commit**

```bash
git add scripts/fitter_prototype/params.py
git commit -m "fitter: FitterParams config with documented defaults"
```

---

## Task 3: Output dataclasses + JSON serializer

**Files:**
- Create: `scripts/fitter_prototype/output.py`
- Create: `scripts/fitter_prototype/tests/test_output.py`

- [ ] **Step 1: Write the failing test**

Create `scripts/fitter_prototype/tests/test_output.py`:

```python
from __future__ import annotations

import json

import numpy as np

from scripts.fitter_prototype.output import (
    ArcPassthrough,
    CornerBlendSlot,
    FittedNurbs,
    JunctionDeviation,
    serialize,
    deserialize,
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest scripts/fitter_prototype/tests/test_output.py -v`
Expected: FAIL — module `output` does not exist.

- [ ] **Step 3: Implement output.py**

Create `scripts/fitter_prototype/output.py`:

```python
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
            out.append(FittedNurbs(
                control_points=np.asarray(data["control_points"], dtype=float),
                knots=np.asarray(data["knots"], dtype=float),
                degree=int(data["degree"]),
                source_vertex_range=tuple(data["source_vertex_range"]),
                max_residual=float(data["max_residual"]),
            ))
        elif kind == "CornerBlendSlot":
            out.append(CornerBlendSlot(
                position=np.asarray(data["position"], dtype=float),
                t_in=np.asarray(data["t_in"], dtype=float),
                t_out=np.asarray(data["t_out"], dtype=float),
                seg_len_in=float(data["seg_len_in"]),
                seg_len_out=float(data["seg_len_out"]),
                tolerance_budget=float(data["tolerance_budget"]),
                default_family=str(data.get("default_family", "cubic_bezier")),
            ))
        elif kind == "JunctionDeviation":
            out.append(JunctionDeviation(
                position=np.asarray(data["position"], dtype=float),
                angle_deg=float(data["angle_deg"]),
            ))
        elif kind == "ArcPassthrough":
            out.append(ArcPassthrough(
                start=np.asarray(data["start"], dtype=float),
                end=np.asarray(data["end"], dtype=float),
                center=np.asarray(data["center"], dtype=float),
                clockwise=bool(data["clockwise"]),
            ))
        else:
            raise ValueError(f"unknown segment kind: {kind}")
    return out
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest scripts/fitter_prototype/tests/test_output.py -v`
Expected: PASS for both tests.

- [ ] **Step 5: Commit**

```bash
git add scripts/fitter_prototype/output.py scripts/fitter_prototype/tests/test_output.py
git commit -m "fitter: typed segment dataclasses with JSON round-trip"
```

---

## Task 4: G-code tokenizer

**Files:**
- Create: `scripts/fitter_prototype/parser.py`
- Create: `scripts/fitter_prototype/tests/test_parser.py`

The parser handles the OrcaSlicer subset: G0/G1/G2/G3 (with X/Y/I/J/Z), comments (line-leading or trailing `;`), and "marker" tokens for any non-motion command (M-codes, T-codes, layer markers, G92, etc.). Position is 2D (XY); Z is captured separately for marker-break logic but does not enter the polyline.

- [ ] **Step 1: Write the failing test**

Create `scripts/fitter_prototype/tests/test_parser.py`:

```python
from __future__ import annotations

from scripts.fitter_prototype.parser import (
    Arc,
    Marker,
    Move,
    parse,
)


def test_parse_simple_g1_sequence():
    text = """
G1 X10 Y20 F1500
G1 X20 Y20
G1 X20 Y10 ; trailing comment
"""
    tokens = parse(text)
    assert all(isinstance(t, Move) for t in tokens)
    assert [t.kind for t in tokens] == ["G1", "G1", "G1"]
    assert tokens[0].x == 10.0
    assert tokens[0].y == 20.0
    assert tokens[2].x == 20.0


def test_parse_arc_with_ij():
    text = "G2 X10 Y0 I5 J0\n"
    tokens = parse(text)
    assert len(tokens) == 1
    assert isinstance(tokens[0], Arc)
    assert tokens[0].kind == "G2"
    assert tokens[0].x == 10.0
    assert tokens[0].i == 5.0


def test_parse_marker_for_nonmotion():
    text = """
G1 X1 Y1
M104 S210
G1 X2 Y2
"""
    tokens = parse(text)
    assert [type(t).__name__ for t in tokens] == ["Move", "Marker", "Move"]
    assert tokens[1].reason == "M104"


def test_parse_g0_is_marker():
    # G0 breaks the polyline (non-extrusion travel)
    text = "G1 X1 Y1\nG0 X5 Y5\nG1 X6 Y6\n"
    tokens = parse(text)
    assert [type(t).__name__ for t in tokens] == ["Move", "Marker", "Move"]
    assert tokens[1].reason == "G0"


def test_parse_strips_comments_and_blank_lines():
    text = "; pure comment\n\nG1 X1 Y1\n; another\n"
    tokens = parse(text)
    assert len(tokens) == 1
    assert isinstance(tokens[0], Move)


def test_parse_z_change_is_marker():
    # A move with Z change but no XY also breaks the polyline.
    text = "G1 X1 Y1\nG1 Z0.4\nG1 X2 Y2\n"
    tokens = parse(text)
    kinds = [type(t).__name__ for t in tokens]
    assert kinds == ["Move", "Marker", "Move"]
    assert tokens[1].reason == "Z_only"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest scripts/fitter_prototype/tests/test_parser.py -v`
Expected: FAIL — module not found.

- [ ] **Step 3: Implement parser.py**

Create `scripts/fitter_prototype/parser.py`:

```python
from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional, Union


@dataclass
class Move:
    kind: str          # "G0", "G1"  (G0 not actually emitted as Move — see Marker)
    x: Optional[float]
    y: Optional[float]
    line_no: int


@dataclass
class Arc:
    kind: str          # "G2" or "G3"
    x: Optional[float]
    y: Optional[float]
    i: Optional[float]
    j: Optional[float]
    line_no: int


@dataclass
class Marker:
    reason: str        # "G0", "M104", "Z_only", "G92", etc.
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
                # Z-only or E-only move: marker, breaks polyline
                tokens.append(Marker(reason="Z_only", line_no=line_no))
            else:
                tokens.append(Move(kind="G1", x=x, y=y, line_no=line_no))
        elif head in ("G2", "G3"):
            params = _parse_params(rest)
            tokens.append(Arc(
                kind=head,
                x=params.get("X"),
                y=params.get("Y"),
                i=params.get("I"),
                j=params.get("J"),
                line_no=line_no,
            ))
        elif head == "G0":
            tokens.append(Marker(reason="G0", line_no=line_no))
        elif head and head[0] in ("G", "M", "T"):
            tokens.append(Marker(reason=head, line_no=line_no))
        # Anything else: silently skip (e.g., raw values, header garbage).
    return tokens
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest scripts/fitter_prototype/tests/test_parser.py -v`
Expected: PASS for all six tests.

- [ ] **Step 5: Sanity-check on real corpus**

```bash
uv run python -c "
from scripts.fitter_prototype.parser import parse, Move, Arc, Marker
text = open('scripts/fitter_prototype/corpus/voron_cube_arc_fitted.gcode').read()
toks = parse(text)
moves = sum(1 for t in toks if isinstance(t, Move))
arcs = sum(1 for t in toks if isinstance(t, Arc))
markers = sum(1 for t in toks if isinstance(t, Marker))
print(f'moves={moves} arcs={arcs} markers={markers}')
"
```

Expected: counts roughly matching `grep -c "^G1"` (≈132k moves) and `grep -c "^G2\\|^G3"` (≈9.7k arcs). Markers will include G0s and other non-motion lines.

- [ ] **Step 6: Commit**

```bash
git add scripts/fitter_prototype/parser.py scripts/fitter_prototype/tests/test_parser.py
git commit -m "fitter: G-code tokenizer for OrcaSlicer subset"
```

---

## Task 5: Geometric reduction

**Files:**
- Create: `scripts/fitter_prototype/reduce.py`
- Create: `scripts/fitter_prototype/tests/test_reduce.py`

`reduce` walks the token stream, accumulating `Move` tokens into `Polyline` objects (XY points). `Arc` tokens become standalone `ArcSegment` passthrough segments. `Marker` tokens close the current polyline.

The X/Y of moves are *absolute* (G90 absolute mode is default in OrcaSlicer output; the prototype assumes absolute and ignores G91/G90 toggles for now). Missing X or Y on a Move means "carry forward from previous position" — handled here.

- [ ] **Step 1: Write the failing test**

Create `scripts/fitter_prototype/tests/test_reduce.py`:

```python
from __future__ import annotations

import numpy as np

from scripts.fitter_prototype.parser import Arc, Marker, Move
from scripts.fitter_prototype.reduce import (
    ArcSegment,
    Polyline,
    reduce_tokens,
)


def test_simple_polyline():
    tokens = [
        Move("G1", 0.0, 0.0, 1),
        Move("G1", 10.0, 0.0, 2),
        Move("G1", 10.0, 10.0, 3),
    ]
    segs = reduce_tokens(tokens)
    assert len(segs) == 1
    assert isinstance(segs[0], Polyline)
    np.testing.assert_array_equal(
        segs[0].points,
        np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]]),
    )


def test_marker_splits_polyline():
    tokens = [
        Move("G1", 0.0, 0.0, 1),
        Move("G1", 10.0, 0.0, 2),
        Marker("G0", 3),
        Move("G1", 50.0, 50.0, 4),
        Move("G1", 60.0, 60.0, 5),
    ]
    segs = reduce_tokens(tokens)
    polylines = [s for s in segs if isinstance(s, Polyline)]
    assert len(polylines) == 2
    assert polylines[0].points.shape == (2, 2)
    assert polylines[1].points.shape == (2, 2)


def test_arc_passthrough_breaks_polyline():
    tokens = [
        Move("G1", 0.0, 0.0, 1),
        Move("G1", 10.0, 0.0, 2),
        Arc("G2", 10.0, 10.0, 0.0, 5.0, 3),
        Move("G1", 0.0, 10.0, 4),
    ]
    segs = reduce_tokens(tokens)
    assert [type(s).__name__ for s in segs] == ["Polyline", "ArcSegment", "Polyline"]
    arc = segs[1]
    np.testing.assert_array_equal(arc.start, [10.0, 0.0])
    np.testing.assert_array_equal(arc.end, [10.0, 10.0])
    np.testing.assert_array_equal(arc.center, [10.0, 5.0])
    assert arc.clockwise is True


def test_carry_forward_missing_xy():
    tokens = [
        Move("G1", 0.0, 0.0, 1),
        Move("G1", 10.0, None, 2),       # Y carries forward as 0
        Move("G1", None, 10.0, 3),       # X carries forward as 10
    ]
    segs = reduce_tokens(tokens)
    poly = segs[0]
    np.testing.assert_array_equal(
        poly.points,
        np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]]),
    )


def test_zero_or_one_point_polyline_is_dropped():
    tokens = [
        Move("G1", 0.0, 0.0, 1),
        Marker("G0", 2),
        Move("G1", 5.0, 5.0, 3),
    ]
    segs = reduce_tokens(tokens)
    # Only the first sub-polyline has 1 point — dropped.
    polylines = [s for s in segs if isinstance(s, Polyline)]
    assert len(polylines) == 1  # the second one (only 1 point too — wait, also dropped)
    # Actually both sub-polylines have only 1 point — both dropped.
    assert len(polylines) == 0
```

Wait — re-read the test: both sub-polylines have only 1 point. Both should be dropped. Let me fix that test to be clear:

```python
def test_zero_or_one_point_polyline_is_dropped():
    tokens = [
        Move("G1", 0.0, 0.0, 1),
        Marker("G0", 2),
        Move("G1", 5.0, 5.0, 3),
        Move("G1", 6.0, 6.0, 4),
    ]
    segs = reduce_tokens(tokens)
    polylines = [s for s in segs if isinstance(s, Polyline)]
    # First sub-polyline has 1 point only → dropped.
    # Second sub-polyline has 2 points → kept.
    assert len(polylines) == 1
    assert polylines[0].points.shape == (2, 2)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest scripts/fitter_prototype/tests/test_reduce.py -v`
Expected: FAIL — module not found.

- [ ] **Step 3: Implement reduce.py**

Create `scripts/fitter_prototype/reduce.py`:

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Union

import numpy as np

from scripts.fitter_prototype.parser import Arc, Marker, Move, Token


@dataclass
class Polyline:
    points: np.ndarray  # shape (n, 2)
    line_range: tuple[int, int]


@dataclass
class ArcSegment:
    start: np.ndarray
    end: np.ndarray
    center: np.ndarray
    clockwise: bool
    line_no: int


GeometricSegment = Union[Polyline, ArcSegment]


def _flush_polyline(
    accum: list[tuple[float, float]],
    line_lo: Optional[int],
    line_hi: Optional[int],
    out: list[GeometricSegment],
) -> None:
    if len(accum) >= 2:
        out.append(Polyline(
            points=np.asarray(accum, dtype=float),
            line_range=(line_lo or 0, line_hi or 0),
        ))


def reduce_tokens(tokens: list[Token]) -> list[GeometricSegment]:
    out: list[GeometricSegment] = []
    accum: list[tuple[float, float]] = []
    line_lo: Optional[int] = None
    line_hi: Optional[int] = None
    cur_x: float = 0.0
    cur_y: float = 0.0

    for tok in tokens:
        if isinstance(tok, Move):
            new_x = tok.x if tok.x is not None else cur_x
            new_y = tok.y if tok.y is not None else cur_y
            if not accum:
                accum.append((cur_x, cur_y))
                line_lo = tok.line_no
            accum.append((new_x, new_y))
            line_hi = tok.line_no
            cur_x, cur_y = new_x, new_y
        elif isinstance(tok, Arc):
            _flush_polyline(accum, line_lo, line_hi, out)
            accum = []
            line_lo = line_hi = None
            new_x = tok.x if tok.x is not None else cur_x
            new_y = tok.y if tok.y is not None else cur_y
            i = tok.i if tok.i is not None else 0.0
            j = tok.j if tok.j is not None else 0.0
            center = np.array([cur_x + i, cur_y + j])
            out.append(ArcSegment(
                start=np.array([cur_x, cur_y]),
                end=np.array([new_x, new_y]),
                center=center,
                clockwise=(tok.kind == "G2"),
                line_no=tok.line_no,
            ))
            cur_x, cur_y = new_x, new_y
        elif isinstance(tok, Marker):
            _flush_polyline(accum, line_lo, line_hi, out)
            accum = []
            line_lo = line_hi = None
            # Position does NOT update on a marker; G0 destinations are unknown
            # to the geometric pipeline by design (we drop non-extrusion moves).
            # If a real Z-change or G92 happens, downstream classify treats the
            # next polyline as a fresh run.

    _flush_polyline(accum, line_lo, line_hi, out)
    return out
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest scripts/fitter_prototype/tests/test_reduce.py -v`
Expected: PASS for all five tests.

- [ ] **Step 5: Sanity-check on real corpus**

```bash
uv run python -c "
from scripts.fitter_prototype.parser import parse
from scripts.fitter_prototype.reduce import reduce_tokens, Polyline, ArcSegment
text = open('scripts/fitter_prototype/corpus/voron_cube_arc_fitted.gcode').read()
segs = reduce_tokens(parse(text))
polys = [s for s in segs if isinstance(s, Polyline)]
arcs = [s for s in segs if isinstance(s, ArcSegment)]
print(f'polylines={len(polys)} arcs={len(arcs)}')
print(f'polyline length stats: min={min(len(p.points) for p in polys)} max={max(len(p.points) for p in polys)} median={sorted(len(p.points) for p in polys)[len(polys)//2]}')
"
```

Expected: hundreds-to-thousands of polylines, ~9.7k arcs, polyline lengths varying widely. Note these numbers — they inform the next step's classifier work.

- [ ] **Step 6: Commit**

```bash
git add scripts/fitter_prototype/reduce.py scripts/fitter_prototype/tests/test_reduce.py
git commit -m "fitter: token-stream → polyline/arc reduction with marker breaks"
```

---

## Task 6: Vertex classifier (CMLT-style)

**Files:**
- Create: `scripts/fitter_prototype/classify.py`
- Create: `scripts/fitter_prototype/tests/test_classify.py`

The classifier labels each *interior* vertex of a polyline. Endpoints are implicitly run boundaries.

- [ ] **Step 1: Write the failing test**

Create `scripts/fitter_prototype/tests/test_classify.py`:

```python
from __future__ import annotations

import numpy as np

from scripts.fitter_prototype.classify import (
    VertexLabel,
    classify_polyline,
)
from scripts.fitter_prototype.params import FitterParams


def test_straight_polyline_all_smooth():
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
    labels = classify_polyline(pts, FitterParams())
    # 2 interior vertices, both smooth.
    assert labels == [VertexLabel.SMOOTH, VertexLabel.SMOOTH]


def test_sharp_corner_classified_hard():
    # Right-angle turn: 90° change.
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    labels = classify_polyline(pts, FitterParams())
    assert labels == [VertexLabel.HARD_CORNER]


def test_gentle_corner_smoothable():
    # 30° change — between θ_smooth=15° and θ_hard=60°.
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0 + np.cos(np.deg2rad(30)), np.sin(np.deg2rad(30))]])
    labels = classify_polyline(pts, FitterParams())
    assert labels == [VertexLabel.SMOOTHABLE_CORNER]


def test_below_smooth_threshold_is_smooth():
    # 5° change — below θ_smooth=15°.
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0 + np.cos(np.deg2rad(5)), np.sin(np.deg2rad(5))]])
    labels = classify_polyline(pts, FitterParams())
    assert labels == [VertexLabel.SMOOTH]


def test_short_polyline_no_interior():
    pts = np.array([[0.0, 0.0], [1.0, 0.0]])
    labels = classify_polyline(pts, FitterParams())
    assert labels == []


def test_zero_length_segment_treated_as_smooth():
    # Repeated point — degenerate segment. Classifier shouldn't crash.
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    labels = classify_polyline(pts, FitterParams())
    assert len(labels) == 2
    # Behavior: zero-length adjacent segment → angle undefined; label as smooth
    # (the run continues; degenerate vertex is effectively a no-op).
    assert all(label == VertexLabel.SMOOTH for label in labels)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest scripts/fitter_prototype/tests/test_classify.py -v`
Expected: FAIL — module not found.

- [ ] **Step 3: Implement classify.py**

Create `scripts/fitter_prototype/classify.py`:

```python
from __future__ import annotations

from enum import Enum

import numpy as np

from scripts.fitter_prototype.params import FitterParams


class VertexLabel(str, Enum):
    SMOOTH = "smooth"
    SMOOTHABLE_CORNER = "smoothable_corner"
    HARD_CORNER = "hard_corner"


def _angle_between(t_in: np.ndarray, t_out: np.ndarray) -> float:
    """Angle in degrees between two non-zero 2D vectors. Returns 0 if either is zero."""
    n_in = np.linalg.norm(t_in)
    n_out = np.linalg.norm(t_out)
    if n_in < 1e-12 or n_out < 1e-12:
        return 0.0
    cos = float(np.dot(t_in, t_out) / (n_in * n_out))
    cos = max(-1.0, min(1.0, cos))
    return float(np.degrees(np.arccos(cos)))


def classify_polyline(points: np.ndarray, params: FitterParams) -> list[VertexLabel]:
    """Label each interior vertex of a polyline.

    Returns a list of length len(points) - 2 (no labels for endpoints).
    """
    labels: list[VertexLabel] = []
    n = len(points)
    if n < 3:
        return labels
    for i in range(1, n - 1):
        t_in = points[i] - points[i - 1]
        t_out = points[i + 1] - points[i]
        theta = _angle_between(t_in, t_out)
        if theta > params.theta_hard_deg:
            labels.append(VertexLabel.HARD_CORNER)
        elif theta > params.theta_smooth_deg:
            labels.append(VertexLabel.SMOOTHABLE_CORNER)
        else:
            labels.append(VertexLabel.SMOOTH)
    return labels
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest scripts/fitter_prototype/tests/test_classify.py -v`
Expected: PASS for all six tests.

- [ ] **Step 5: Commit**

```bash
git add scripts/fitter_prototype/classify.py scripts/fitter_prototype/tests/test_classify.py
git commit -m "fitter: CMLT-style vertex classifier"
```

---

## Task 7: Corner-blend slot emitter

**Files:**
- Create: `scripts/fitter_prototype/corner_blend.py`
- Create: `scripts/fitter_prototype/tests/test_corner_blend.py`

The emitter takes a corner vertex with its incident segments and produces a `CornerBlendSlot` (parameterized; no shape selection — that's Layer 3). It also provides a `placeholder_finalize` for plotting that puts cubic-Bezier control points at 1/3 along incident segment lengths.

- [ ] **Step 1: Write the failing test**

Create `scripts/fitter_prototype/tests/test_corner_blend.py`:

```python
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
    np.testing.assert_allclose(cps[-1], corner + slot.t_out * slot.seg_len_out / 3)
    # Middle two collapse to the corner (Pateloup default).
    np.testing.assert_allclose(cps[1], corner)
    np.testing.assert_allclose(cps[2], corner)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest scripts/fitter_prototype/tests/test_corner_blend.py -v`
Expected: FAIL — module not found.

- [ ] **Step 3: Implement corner_blend.py**

Create `scripts/fitter_prototype/corner_blend.py`:

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest scripts/fitter_prototype/tests/test_corner_blend.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add scripts/fitter_prototype/corner_blend.py scripts/fitter_prototype/tests/test_corner_blend.py
git commit -m "fitter: parameterized corner-blend slot emitter with Pateloup placeholder"
```

---

## Task 8: LSPIA fitter — initial fit

**Files:**
- Create: `scripts/fitter_prototype/fit.py`
- Create: `scripts/fitter_prototype/tests/test_fit.py`

This task implements the LSPIA core. Chord-bound refinement comes in Task 9.

The implementation follows Bi 2019 §3:
- Chord-length parameterize input points to [0, 1].
- Build a clamped knot vector with `n_init_interior` uniformly-spaced interior knots.
- Build basis matrix `B[i,j] = N_j(t_i)`.
- Initialize control points by least-squares.
- Iterate: `C ← C + B^T (P - B C) / diag(B^T B)`. Stop on convergence or iteration cap.

- [ ] **Step 1: Write the failing test**

Create `scripts/fitter_prototype/tests/test_fit.py`:

```python
from __future__ import annotations

import numpy as np

from scripts.fitter_prototype.fit import (
    build_basis_matrix,
    chord_length_parameterize,
    lspia_fit,
    make_clamped_knot_vector,
)
from scripts.fitter_prototype.params import FitterParams


def test_chord_length_parameterize():
    pts = np.array([[0.0, 0.0], [3.0, 0.0], [3.0, 4.0]])  # lengths 3, 4 → cum 0, 3, 7
    t = chord_length_parameterize(pts)
    np.testing.assert_allclose(t, [0.0, 3.0 / 7.0, 1.0])


def test_clamped_knot_vector_shape():
    knots = make_clamped_knot_vector(degree=3, n_interior=2)
    # Expected: [0,0,0,0, k1,k2, 1,1,1,1] — len = 4 + 2 + 4 = 10
    assert len(knots) == 10
    assert (knots[:4] == 0.0).all()
    assert (knots[-4:] == 1.0).all()
    assert knots[4] == 1.0 / 3.0  # uniform interior placement
    assert knots[5] == 2.0 / 3.0


def test_basis_matrix_partition_of_unity():
    knots = make_clamped_knot_vector(degree=3, n_interior=2)
    n_control = len(knots) - 3 - 1
    t = np.linspace(0.0, 1.0, 11)
    B = build_basis_matrix(t, knots, degree=3, n_control=n_control)
    # Each row sums to 1 (partition of unity).
    np.testing.assert_allclose(B.sum(axis=1), np.ones(11), atol=1e-9)


def test_lspia_fits_a_straight_line_exactly():
    # Sample a line — LSPIA should recover it within numerical noise.
    pts = np.array([[i * 0.5, i * 0.5] for i in range(20)])
    cps, knots, t = lspia_fit(pts, FitterParams())
    from scipy.interpolate import BSpline
    spline = BSpline(knots, cps, FitterParams().degree, extrapolate=False)
    eval_pts = spline(t)
    residuals = np.linalg.norm(eval_pts - pts, axis=1)
    assert residuals.max() < 1e-6


def test_lspia_fits_a_circle_within_tolerance():
    # Sample a unit circle quadrant.
    angles = np.linspace(0.0, np.pi / 2, 30)
    pts = np.column_stack([np.cos(angles), np.sin(angles)])
    params = FitterParams(n_init_interior=6)
    cps, knots, t = lspia_fit(pts, params)
    from scipy.interpolate import BSpline
    spline = BSpline(knots, cps, params.degree, extrapolate=False)
    residuals = np.linalg.norm(spline(t) - pts, axis=1)
    # Cubic NURBS approximation of a circle quadrant: realistic floor ~1e-3.
    assert residuals.max() < 5e-3
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest scripts/fitter_prototype/tests/test_fit.py -v`
Expected: FAIL — module not found.

- [ ] **Step 3: Implement fit.py — initial fit only**

Create `scripts/fitter_prototype/fit.py`:

```python
from __future__ import annotations

import numpy as np
from scipy.interpolate import BSpline

from scripts.fitter_prototype.output import FittedNurbs
from scripts.fitter_prototype.params import FitterParams


def chord_length_parameterize(points: np.ndarray) -> np.ndarray:
    """Cumulative chord-length parameterization, normalized to [0, 1]."""
    diffs = np.diff(points, axis=0)
    chord_lengths = np.linalg.norm(diffs, axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(chord_lengths)])
    total = cumulative[-1]
    if total < 1e-12:
        return np.zeros(len(points))
    return cumulative / total


def make_clamped_knot_vector(degree: int, n_interior: int) -> np.ndarray:
    """Clamped knot vector on [0, 1] with `n_interior` uniformly-spaced
    interior knots. Total length = 2*(degree+1) + n_interior; total control
    points = degree + 1 + n_interior."""
    interior = np.linspace(0.0, 1.0, n_interior + 2)[1:-1]
    return np.concatenate([
        np.zeros(degree + 1),
        interior,
        np.ones(degree + 1),
    ])


def build_basis_matrix(
    t: np.ndarray,
    knots: np.ndarray,
    degree: int,
    n_control: int,
) -> np.ndarray:
    """B[i, j] = N_j(t_i). Uses scipy BSpline with one-hot control coefficients."""
    n_data = len(t)
    B = np.zeros((n_data, n_control))
    for j in range(n_control):
        c = np.zeros(n_control)
        c[j] = 1.0
        spline = BSpline(knots, c, degree, extrapolate=False)
        vals = spline(t)
        # BSpline returns NaN outside the parametric domain; clamp.
        B[:, j] = np.nan_to_num(vals, nan=0.0)
    # Boundary fix: BSpline's right-clamp at t = knots[-1] sometimes returns
    # zero everywhere; force partition of unity at the right endpoint.
    last_row_sum = B[-1].sum()
    if last_row_sum < 0.5:  # numerical hint that we hit the boundary issue
        B[-1, -1] = 1.0
    return B


def lspia_fit(
    points: np.ndarray,
    params: FitterParams,
    knots_override: np.ndarray = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """LSPIA fit. Returns (control_points, knots, t_params).

    Bi 2019 §3 fixed-point iteration. Provably contracts to LSQ solution.
    """
    t = chord_length_parameterize(points)
    if knots_override is not None:
        knots = knots_override
        n_control = len(knots) - params.degree - 1
    else:
        knots = make_clamped_knot_vector(params.degree, params.n_init_interior)
        n_control = params.degree + 1 + params.n_init_interior
    B = build_basis_matrix(t, knots, params.degree, n_control)

    # Initial CP via LSQ.
    cps, *_ = np.linalg.lstsq(B, points, rcond=None)

    # Per-CP normalization for the LSPIA update.
    diag = (B * B).sum(axis=0)
    diag = np.where(diag < 1e-12, 1.0, diag)

    for _ in range(params.max_lspia_iter):
        residuals = points - B @ cps
        update = (B.T @ residuals) / diag[:, None]
        max_update = float(np.max(np.linalg.norm(update, axis=1)))
        cps = cps + update
        if max_update < params.eps_iter_mm:
            break

    return cps, knots, t


def evaluate_fit(
    cps: np.ndarray,
    knots: np.ndarray,
    degree: int,
    t: np.ndarray,
) -> np.ndarray:
    spline = BSpline(knots, cps, degree, extrapolate=False)
    vals = spline(t)
    # Right-boundary fix mirroring build_basis_matrix.
    if np.any(np.isnan(vals[-1])):
        vals[-1] = cps[-1]
    return vals


def max_residual(
    cps: np.ndarray,
    knots: np.ndarray,
    degree: int,
    t: np.ndarray,
    points: np.ndarray,
) -> float:
    eval_pts = evaluate_fit(cps, knots, degree, t)
    return float(np.linalg.norm(eval_pts - points, axis=1).max())
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest scripts/fitter_prototype/tests/test_fit.py -v`
Expected: PASS for all five tests. If `test_lspia_fits_a_circle_within_tolerance` fails with residual just above 5e-3, increase `n_init_interior` to 8 in that test (cubic NURBS quarter-circle approximation is well-studied; floor depends on knot count).

- [ ] **Step 5: Commit**

```bash
git add scripts/fitter_prototype/fit.py scripts/fitter_prototype/tests/test_fit.py
git commit -m "fitter: LSPIA initial fit (Bi 2019 §3)"
```

---

## Task 9: Chord-bound refinement loop

**Files:**
- Modify: `scripts/fitter_prototype/fit.py`
- Modify: `scripts/fitter_prototype/tests/test_fit.py`

Add `fit_smooth_run` which wraps `lspia_fit` with chord-bound checking and refinement. Per the spec §3.2: sample each piece (between adjacent unique interior knots) densely, measure max distance from sampled curve points to the chord between piece endpoints, insert a knot at the parameter of worst residual where the bound exceeds tolerance, and re-fit.

- [ ] **Step 1: Write the failing test**

Append to `scripts/fitter_prototype/tests/test_fit.py`:

```python
from scripts.fitter_prototype.fit import (
    fit_smooth_run,
    measure_chord_error_per_piece,
)


def test_fit_smooth_run_returns_fitted_nurbs_within_tolerance():
    # Sample a half-circle. With initial 4 interior knots the chord error
    # exceeds 25 µm; refinement must add knots until tolerance is met.
    angles = np.linspace(0.0, np.pi, 60)
    pts = np.column_stack([np.cos(angles), np.sin(angles)])
    params = FitterParams(eps_chord_mm=0.025, max_refine_iter=20)
    fit = fit_smooth_run(pts, source_vertex_range=(0, 60), params=params)
    assert fit.max_residual <= params.eps_chord_mm * 1.05  # 5% slack on numerical eval


def test_chord_error_decreases_with_refinement():
    # Same half-circle: track that refinement reduces error.
    angles = np.linspace(0.0, np.pi, 60)
    pts = np.column_stack([np.cos(angles), np.sin(angles)])
    params_loose = FitterParams(eps_chord_mm=0.05, max_refine_iter=20)
    params_tight = FitterParams(eps_chord_mm=0.001, max_refine_iter=30)
    fit_loose = fit_smooth_run(pts, (0, 60), params_loose)
    fit_tight = fit_smooth_run(pts, (0, 60), params_tight)
    # Tighter tolerance produces more knots and smaller max residual.
    assert fit_tight.max_residual < fit_loose.max_residual
    assert len(fit_tight.knots) > len(fit_loose.knots)


def test_measure_chord_error_basic():
    # Straight-line input: all per-piece errors should be ~0.
    pts = np.array([[i * 0.5, i * 0.5] for i in range(20)])
    params = FitterParams()
    cps, knots, t = lspia_fit(pts, params)
    errors = measure_chord_error_per_piece(cps, knots, params.degree, params.n_chord_samples)
    assert max(errors) < 1e-6
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest scripts/fitter_prototype/tests/test_fit.py -v`
Expected: FAIL — `fit_smooth_run` and `measure_chord_error_per_piece` not yet defined.

- [ ] **Step 3: Append to fit.py**

Append to `scripts/fitter_prototype/fit.py`:

```python
def _unique_interior_breakpoints(knots: np.ndarray, degree: int) -> np.ndarray:
    """Strictly interior breakpoints (not the clamped endpoints), with
    duplicates collapsed."""
    interior = knots[degree + 1 : -(degree + 1)]
    if len(interior) == 0:
        return interior
    return np.unique(interior)


def _piece_breakpoints(knots: np.ndarray, degree: int) -> np.ndarray:
    """Full breakpoint list including clamped start and end."""
    interior = _unique_interior_breakpoints(knots, degree)
    return np.concatenate([[knots[degree]], interior, [knots[-degree - 1]]])


def measure_chord_error_per_piece(
    cps: np.ndarray,
    knots: np.ndarray,
    degree: int,
    n_samples: int,
) -> list[float]:
    """For each piece between adjacent breakpoints, sample the curve and return
    max distance from sampled points to the chord between piece endpoints.

    This is a sample-based approximation of the analytical chord-bound. It
    overestimates a tiny bit — fine for prototype, replace with proper
    convex-hull bound when porting to Rust.
    """
    breakpoints = _piece_breakpoints(knots, degree)
    spline = BSpline(knots, cps, degree, extrapolate=False)
    errors: list[float] = []
    for k in range(len(breakpoints) - 1):
        t0, t1 = breakpoints[k], breakpoints[k + 1]
        ts = np.linspace(t0, t1, n_samples)
        pts = spline(ts)
        # Right-boundary safety.
        if np.any(np.isnan(pts[-1])):
            pts[-1] = cps[-1]
        chord_start, chord_end = pts[0], pts[-1]
        chord_vec = chord_end - chord_start
        chord_len = float(np.linalg.norm(chord_vec))
        if chord_len < 1e-12:
            errors.append(0.0)
            continue
        chord_dir = chord_vec / chord_len
        offsets = pts - chord_start
        parallel = (offsets @ chord_dir)[:, None] * chord_dir
        perp = offsets - parallel
        dists = np.linalg.norm(perp, axis=1)
        errors.append(float(dists.max()))
    return errors


def _worst_piece_param(
    cps: np.ndarray,
    knots: np.ndarray,
    degree: int,
    piece_idx: int,
    n_samples: int,
) -> float:
    breakpoints = _piece_breakpoints(knots, degree)
    t0, t1 = breakpoints[piece_idx], breakpoints[piece_idx + 1]
    ts = np.linspace(t0, t1, n_samples)
    spline = BSpline(knots, cps, degree, extrapolate=False)
    pts = spline(ts)
    if np.any(np.isnan(pts[-1])):
        pts[-1] = cps[-1]
    chord_start, chord_end = pts[0], pts[-1]
    chord_vec = chord_end - chord_start
    chord_len = float(np.linalg.norm(chord_vec))
    if chord_len < 1e-12:
        return float((t0 + t1) / 2.0)
    chord_dir = chord_vec / chord_len
    offsets = pts - chord_start
    parallel = (offsets @ chord_dir)[:, None] * chord_dir
    perp = offsets - parallel
    dists = np.linalg.norm(perp, axis=1)
    return float(ts[int(np.argmax(dists))])


def fit_smooth_run(
    points: np.ndarray,
    source_vertex_range: tuple[int, int],
    params: FitterParams,
) -> FittedNurbs:
    """LSPIA + chord-bound refinement."""
    cps, knots, t = lspia_fit(points, params)

    for _ in range(params.max_refine_iter):
        errors = measure_chord_error_per_piece(
            cps, knots, params.degree, params.n_chord_samples,
        )
        worst_err = max(errors) if errors else 0.0
        if worst_err <= params.eps_chord_mm:
            break
        worst_idx = int(np.argmax(errors))
        new_knot = _worst_piece_param(
            cps, knots, params.degree, worst_idx, params.n_chord_samples,
        )
        # Insert at the worst-residual parameter location.
        knots = np.sort(np.concatenate([knots, [new_knot]]))
        cps, knots, t = lspia_fit(points, params, knots_override=knots)

    return FittedNurbs(
        control_points=cps,
        knots=knots,
        degree=params.degree,
        source_vertex_range=source_vertex_range,
        max_residual=max_residual(cps, knots, params.degree, t, points),
    )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest scripts/fitter_prototype/tests/test_fit.py -v`
Expected: PASS. If `test_fit_smooth_run_returns_fitted_nurbs_within_tolerance` exceeds the budget, raise `max_refine_iter` to 30 — half-circle on cubic NURBS at 25 µm tolerance is borderline.

- [ ] **Step 5: Commit**

```bash
git add scripts/fitter_prototype/fit.py scripts/fitter_prototype/tests/test_fit.py
git commit -m "fitter: chord-bound refinement loop on top of LSPIA"
```

---

## Task 10: Pipeline orchestrator (run.py without analyze yet)

**Files:**
- Create: `scripts/fitter_prototype/run.py`
- Create: `scripts/fitter_prototype/tests/test_end_to_end.py`

`run.py` ties the modules together: parse → reduce → for each polyline, classify and split into smooth runs, fit each smooth run, emit corner-blend slots and junction-deviation markers; arcs pass through. Output is JSON.

`analyze.py` (plots and stats) is the separate next task.

- [ ] **Step 1: Write the failing test**

Create `scripts/fitter_prototype/tests/test_end_to_end.py`:

```python
from __future__ import annotations

from scripts.fitter_prototype.output import (
    ArcPassthrough,
    CornerBlendSlot,
    FittedNurbs,
    JunctionDeviation,
)
from scripts.fitter_prototype.params import FitterParams
from scripts.fitter_prototype.run import process_gcode


def test_end_to_end_smooth_polyline_produces_fit():
    text = "\n".join(f"G1 X{i*0.1} Y{i*0.1}" for i in range(50))
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
    # The L-shaped corner at (10,0) is hard (90°) → JunctionDeviation.
    # Then arc → ArcPassthrough.
    # Last polyline is just two points — no fit, no classifier work.
    assert "JunctionDeviation" in kinds
    assert "ArcPassthrough" in kinds


def test_end_to_end_smoothable_corner_emits_slot():
    # 30° corner.
    import numpy as np
    a = np.deg2rad(30)
    text = f"""
G1 X0 Y0
G1 X10 Y0
G1 X{10 + 10 * np.cos(a)} Y{10 * np.sin(a)}
G1 X{20 + 10 * np.cos(a)} Y{20 * np.sin(a)}
"""
    segs = process_gcode(text, FitterParams())
    kinds = [type(s).__name__ for s in segs]
    assert "CornerBlendSlot" in kinds
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest scripts/fitter_prototype/tests/test_end_to_end.py -v`
Expected: FAIL — `process_gcode` not defined.

- [ ] **Step 3: Implement run.py**

Create `scripts/fitter_prototype/run.py`:

```python
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
        # Single segment — no interior labels. Emit as a degree-1 fitted NURBS.
        # (The fitter doesn't handle 2 points cleanly; emit a trivial fit.)
        return out  # the prototype skips trivial 2-point polylines
    labels = classify_polyline(pts, params)

    # Walk along, accumulating smooth runs; emit at each non-smooth label.
    run_start = 0
    for i, label in enumerate(labels):
        v_idx = i + 1  # interior vertex index in pts
        if label == VertexLabel.SMOOTH:
            continue
        # Close the current run, including v_idx as its endpoint.
        if v_idx - run_start >= 2:
            run_pts = pts[run_start : v_idx + 1]
            offset = poly.line_range[0]
            out.append(fit_smooth_run(
                run_pts,
                source_vertex_range=(offset + run_start, offset + v_idx),
                params=params,
            ))
        # Emit corner-blend or junction-deviation at v_idx.
        prev_pt = pts[v_idx - 1]
        corner = pts[v_idx]
        next_pt = pts[v_idx + 1]
        if label == VertexLabel.SMOOTHABLE_CORNER:
            out.append(make_slot(prev_pt, corner, next_pt, params))
        else:  # HARD_CORNER
            t_in = corner - prev_pt
            t_out = next_pt - corner
            cos = float(np.dot(t_in, t_out) / (
                np.linalg.norm(t_in) * np.linalg.norm(t_out) + 1e-12
            ))
            cos = max(-1.0, min(1.0, cos))
            angle = float(np.degrees(np.arccos(cos)))
            out.append(JunctionDeviation(
                position=np.asarray(corner, dtype=float),
                angle_deg=angle,
            ))
        run_start = v_idx  # next run starts at this corner

    # Close the final run.
    if len(pts) - 1 - run_start >= 2:
        offset = poly.line_range[0]
        run_pts = pts[run_start:]
        out.append(fit_smooth_run(
            run_pts,
            source_vertex_range=(offset + run_start, offset + len(pts) - 1),
            params=params,
        ))

    return out


def process_gcode(text: str, params: FitterParams) -> list[Segment]:
    segments: list[Segment] = []
    for geo in reduce_tokens(parse(text)):
        if isinstance(geo, Polyline):
            segments.extend(_process_polyline(geo, params))
        elif isinstance(geo, ArcSegment):
            segments.append(ArcPassthrough(
                start=geo.start,
                end=geo.end,
                center=geo.center,
                clockwise=geo.clockwise,
            ))
    return segments


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("gcode", type=Path, nargs="+", help="input .gcode file(s)")
    ap.add_argument("--out", type=Path, required=True, help="output directory")
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest scripts/fitter_prototype/tests/test_end_to_end.py -v`
Expected: PASS for all three tests.

- [ ] **Step 5: Run the full test suite — everything still green**

Run: `uv run pytest scripts/fitter_prototype/tests/ -v`
Expected: all green.

- [ ] **Step 6: Commit**

```bash
git add scripts/fitter_prototype/run.py scripts/fitter_prototype/tests/test_end_to_end.py
git commit -m "fitter: end-to-end orchestrator (parse → reduce → classify → fit → JSON)"
```

---

## Task 11: Analyze module (stats + plots)

**Files:**
- Create: `scripts/fitter_prototype/analyze.py`
- Modify: `scripts/fitter_prototype/run.py` (wire analyze into the CLI)

Two responsibilities, both pure read-from-segments:

1. **Stats:** counts and distributions written to a JSON summary.
2. **Plots:** matplotlib figures saved as PNG.

No tests for plotting code beyond a smoke test that runs without raising.

- [ ] **Step 1: Implement analyze.py**

Create `scripts/fitter_prototype/analyze.py`:

```python
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
        # interior knots count + 1 = piece count.
        max(0, len(np.unique(s.knots[s.degree + 1 : -(s.degree + 1)])) + 1)
        for s in fitted
    ]
    return {
        "segment_counts": dict(counts),
        "fits": {
            "count": len(fitted),
            "max_residual_mm": [s.max_residual for s in fitted],
            "max_residual_p50": float(np.median([s.max_residual for s in fitted])) if fitted else 0.0,
            "max_residual_p95": float(np.percentile([s.max_residual for s in fitted], 95)) if fitted else 0.0,
            "pieces_per_fit_p50": float(np.median(pieces_per_fit)) if pieces_per_fit else 0.0,
            "pieces_per_fit_p95": float(np.percentile(pieces_per_fit, 95)) if pieces_per_fit else 0.0,
            "vertex_count_per_fit_p50": float(np.median([
                s.source_vertex_range[1] - s.source_vertex_range[0] for s in fitted
            ])) if fitted else 0.0,
        },
    }


def write_stats(segments: list[Segment], out_path: Path) -> None:
    out_path.write_text(json.dumps(compute_stats(segments), indent=2))


def plot_residual_histogram(segments: list[Segment], out_path: Path) -> None:
    residuals = [s.max_residual for s in segments if isinstance(s, FittedNurbs)]
    if not residuals:
        return
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.hist(np.asarray(residuals) * 1e3, bins=50)  # mm → µm
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


def plot_classification_breakdown(segments: list[Segment], out_path: Path) -> None:
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


def plot_geometry_overlay(segments: list[Segment], out_path: Path, max_runs: int = 50) -> None:
    """First N fitted runs overlaid with their corner-blend placeholders and arcs."""
    fig, ax = plt.subplots(figsize=(8, 8))
    n_drawn = 0
    for seg in segments:
        if isinstance(seg, FittedNurbs) and n_drawn < max_runs:
            t = np.linspace(seg.knots[seg.degree], seg.knots[-seg.degree - 1], 200)
            spline = BSpline(seg.knots, seg.control_points, seg.degree, extrapolate=False)
            curve = spline(t)
            if np.any(np.isnan(curve[-1])):
                curve[-1] = seg.control_points[-1]
            ax.plot(curve[:, 0], curve[:, 1], linewidth=0.7, color="C0")
            n_drawn += 1
        elif isinstance(seg, CornerBlendSlot):
            cps = placeholder_finalize(seg)
            ax.plot(cps[:, 0], cps[:, 1], linewidth=0.5, color="C1", alpha=0.6)
        elif isinstance(seg, JunctionDeviation):
            ax.plot(seg.position[0], seg.position[1], "x", color="C3", markersize=4)
        elif isinstance(seg, ArcPassthrough):
            # Approximate the arc by a polyline for plotting.
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
    ax.set_title("Geometry overlay (fits=blue, blends=orange, junctions=red×, arcs=green)")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def render_all(segments: list[Segment], out_dir: Path, stem: str) -> None:
    plot_residual_histogram(segments, out_dir / f"{stem}.residuals.png")
    plot_piece_count_histogram(segments, out_dir / f"{stem}.pieces.png")
    plot_classification_breakdown(segments, out_dir / f"{stem}.kinds.png")
    plot_geometry_overlay(segments, out_dir / f"{stem}.overlay.png")
    write_stats(segments, out_dir / f"{stem}.stats.json")
```

- [ ] **Step 2: Wire analyze into run.py main()**

In `scripts/fitter_prototype/run.py`, modify the `main()` loop to call `render_all` after writing the JSON. Replace:

```python
for path in args.gcode:
    text = path.read_text()
    segments = process_gcode(text, params)
    out_path = args.out / f"{path.stem}.segments.json"
    out_path.write_text(json.dumps(serialize(segments), indent=2))
    print(f"{path.name}: {len(segments)} segments → {out_path}")
```

with:

```python
from scripts.fitter_prototype.analyze import render_all  # noqa: E402

for path in args.gcode:
    text = path.read_text()
    segments = process_gcode(text, params)
    out_path = args.out / f"{path.stem}.segments.json"
    out_path.write_text(json.dumps(serialize(segments), indent=2))
    render_all(segments, args.out, path.stem)
    print(f"{path.name}: {len(segments)} segments → {out_path} (+ plots)")
```

(Move the import to the file-top to satisfy ruff I001; the `# noqa` above is illustrative — the actual edit places `from scripts.fitter_prototype.analyze import render_all` in the import block.)

- [ ] **Step 3: Smoke-test on a synthetic input**

```bash
mkdir -p /tmp/fitter_smoke
uv run python -c "
from pathlib import Path
import numpy as np
from scripts.fitter_prototype.analyze import render_all
from scripts.fitter_prototype.params import FitterParams
from scripts.fitter_prototype.run import process_gcode

text = '\n'.join(f'G1 X{0.5*i} Y{0.5*i}' for i in range(50))
segs = process_gcode(text, FitterParams())
render_all(segs, Path('/tmp/fitter_smoke'), 'synthetic')
print('plots written')
"
ls /tmp/fitter_smoke/
```

Expected: four PNGs and a `.stats.json` written without error.

- [ ] **Step 4: Run full test suite**

Run: `uv run pytest scripts/fitter_prototype/tests/ -v`
Expected: all green (analyze has no tests, but nothing else regressed).

- [ ] **Step 5: Commit**

```bash
git add scripts/fitter_prototype/analyze.py scripts/fitter_prototype/run.py
git commit -m "fitter: analyze module — residual/piece/kind histograms, geometry overlay"
```

---

## Task 12: First corpus run — arc-fitted file

**Files:**
- Create: `scripts/fitter_prototype/corpus_results/` (output dir, will be gitignored)
- Modify: `.gitignore`

- [ ] **Step 1: Add corpus_results/ to .gitignore**

Add to repo-root `.gitignore`:

```
scripts/fitter_prototype/corpus_results/
```

- [ ] **Step 2: Run on the arc-fitted corpus**

```bash
uv run python -m scripts.fitter_prototype.run \
    scripts/fitter_prototype/corpus/voron_cube_arc_fitted.gcode \
    --out scripts/fitter_prototype/corpus_results/
```

Expected: completes without crashing. May take several minutes (132k G1 commands → many polylines → many fits, each running LSPIA + chord-bound refinement). Output prints segment count.

If it crashes: capture the traceback. Common causes — degenerate polyline (one point), parser hitting an unexpected G-code, fitter on a smooth run with collinear points (chord-length parameterization → zero total length, divides by zero — should be handled but verify).

- [ ] **Step 3: Inspect outputs**

```bash
ls scripts/fitter_prototype/corpus_results/
cat scripts/fitter_prototype/corpus_results/voron_cube_arc_fitted.stats.json
```

Open the four PNGs and the overlay. The geometry overlay should show recognizable Benchy + cube outlines.

- [ ] **Step 4: Capture the run output for the writeup**

```bash
cp scripts/fitter_prototype/corpus_results/voron_cube_arc_fitted.stats.json \
   /tmp/arc_fitted_stats.json
```

These numbers feed Task 14 (the spike writeup).

- [ ] **Step 5: Commit only the .gitignore change**

```bash
git add .gitignore
git commit -m "fitter: gitignore corpus_results directory"
```

---

## Task 13: Second corpus run + arc-fitted vs straight-line comparison

- [ ] **Step 1: Run on the straight-line corpus**

```bash
uv run python -m scripts.fitter_prototype.run \
    scripts/fitter_prototype/corpus/voron_cube_straight_line.gcode \
    --out scripts/fitter_prototype/corpus_results/
```

Expected: completes. Will produce more fitted runs than the arc-fitted file (no slicer-side arcs to take the curved sections out of the polyline stream).

- [ ] **Step 2: Inspect side-by-side**

```bash
diff <(jq -S . scripts/fitter_prototype/corpus_results/voron_cube_arc_fitted.stats.json) \
     <(jq -S . scripts/fitter_prototype/corpus_results/voron_cube_straight_line.stats.json) | head -60
```

Compare:

- Total fits: straight-line should have more.
- Median residual: should be similar (both bounded by `eps_chord_mm`).
- Median pieces per fit: straight-line likely has more pieces per fit (curved regions that arc-fit collapsed are now polylines that need many B-spline pieces).
- Reject-category counts: smoothable-corner and hard-corner counts should be roughly comparable (slicer-arc-fit doesn't change corner geometry, just curves).

- [ ] **Step 3: Capture for writeup**

```bash
cp scripts/fitter_prototype/corpus_results/voron_cube_straight_line.stats.json \
   /tmp/straight_line_stats.json
```

- [ ] **Step 4: No commit (no code changed)**

Both stats JSONs are gitignored; nothing to commit at this step.

---

## Task 14: D3 spike findings writeup

**Files:**
- Create: `docs/superpowers/spikes/2026-04-26-layer-1-fitter-spike.md`

The writeup synthesizes lit findings + measured numbers into per-question conclusions for the Layer 1 architecture spec (F0). It is the artifact the original brainstorm planned.

- [ ] **Step 1: Compose the writeup**

Create `docs/superpowers/spikes/2026-04-26-layer-1-fitter-spike.md`. Use the structure agreed during brainstorming. Do not pad — each section should be tight.

```markdown
# Layer 1 Spline Fitter — Spike Findings

**Date:** 2026-04-26
**Status:** Findings (literature + measurement on OrcaSlicer corpora)
**Layer:** 1 (Geometry pipeline)
**Driver:** Inputs to the F0 Layer 1 architecture spec

## 1. Context

[2 paragraphs: what the spike asked, that the lit triangulation was folded into
the prototype run, what corpora were used.]

## 2. Method

- Literature triangulation across 18 sources (CNC streaming-fit, offline
  fitting with chord-error constraints, knot placement, reject criteria,
  hobby-firmware practical reports). See §10.
- Python prototype (`scripts/fitter_prototype/`) on two OrcaSlicer 2.3.2 files
  of Voron Design Cube + 3D Benchy: arc-fitted vs straight-line.
- Default fitter parameters: degree=3, `ε_chord = 25 µm`, classifier
  thresholds θ_smooth=15°, θ_hard=60°.

## 3. The slicer-G1 corpus, characterized

[Quantitative description from the run: vertex density per mm of path,
segment-length distribution, angle-change distribution, ratio of arc to G1
under arc-fit-on. Pull from /tmp/arc_fitted_stats.json and /tmp/straight_line_stats.json.]

## 4. Findings per question

### Q1: Achievable residual tolerance

[Measured median + p95 from prototype run. Compare to lit's 0.5×-input-tolerance
heuristic. State the working default and why.]

### Q2: Reject anatomy

[The reject metadata payload: t_in, t_out, seg_len_in/out, tolerance_budget,
classification. Per-mm-of-print rates for each category from corpus run.
Lit precedent: Sun 2018 CMLT.]

### Q3: Output structure (one-per-run, refined inside)

[Measured pieces-per-fit distribution. Confirm/refute "one NURBS per smooth
run, internally refined" architecture. Lit ref: Bi 2019.]

### Q4: Streaming lookahead

[Bounded by run-end markers + post-reject corner-context buffer + max-window
cap. Estimate from corpus: typical run length, max run length encountered.]

### Q5: Configuration knobs

[Five knobs total: 3 fitter (residual tol, corner threshold, max window),
2 corner-rounder (blend tolerance budget, quality target). State the
measured-default justification for each.]

## 5. Three-path coordination

[Layer 1 (geometric emission) → Layer 3 (shape selection under dynamic limits).
Cite Tajima & Sencer 2016 for the load-bearing physics. CLAUDE.md updated
2026-04-26 to reflect this split.]

## 6. Algorithm-family recommendation

LSPIA + chord-bound refinement (Bi 2019) for fittable regions, CMLT classifier
upstream (Sun 2018), parameterized cubic-Bezier slot for smoothable corners
(Zhao 2013 placement, Tajima/Sencer 2016 dynamic-limit shape selection in L3),
junction-deviation for hard corners. FIR-convolution path (Tajima & Sencer
2017) rejected — incompatible with the algebraic-closure pipeline.

## 7. Open questions for F0

- [Whether `θ_smooth=15°` / `θ_hard=60°` are still right after measuring on
  Bambu/Prusa corpora — only OrcaSlicer measured here]
- [Per-mm reject-rate floor when corpus diversifies]
- [Whether the max-window cap ever fires in practice (didn't observe it in
  the OrcaSlicer corpus)]
- [Whether placeholder cubic-Bezier from Pateloup 2004 will be the dominant
  shape in practice or whether dynamic-limit-aware selection meaningfully
  changes the picture — Layer 3 question, not L1's]

## 8. Language-stack recommendation

[Python for prototype (already done — this writeup is its byproduct), Rust
for production. Three risks called out in the prototype design spec §1: port
becomes its own project, vectorized numpy patterns don't map, drift between
prototype and Rust. Mitigations: keep Python surgical, scalar-loop inner code,
cross-check Rust port against Python on a fixed corpus.]

## 9. References

[Pull from the lit-scan output in conversation. Group: streaming fitters,
offline fitters, knot placement, reject criteria, slicer/practical reports.
Cite the architectural ones (Tajima/Sencer 2016, Bi 2019, Sun 2018, Zhao 2013)
prominently; the rest as background.]
```

Fill each `[bracketed instruction]` with content from the prototype run's
JSON outputs and the lit-scan results in the conversation transcript. Keep
the writeup under ~1500 words.

- [ ] **Step 2: Self-review the writeup**

Skim once more for placeholders/contradictions. Specifically check:

- Q1's measured number is consistent with the working default
- Q4's "max-window cap" claim is consistent with observed run lengths
- §5 matches the CLAUDE.md edits made earlier in this conversation
- §8 matches the prototype design spec §1 reasoning

- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/spikes/2026-04-26-layer-1-fitter-spike.md
git commit -m "spike: Layer 1 fitter findings — lit + OrcaSlicer corpus measurements"
```

---

## Final verification

- [ ] **Step 1: All tests still green**

Run: `uv run pytest scripts/fitter_prototype/tests/ -v`
Expected: all green.

- [ ] **Step 2: Ruff clean**

Run: `uv run ruff check scripts/fitter_prototype/`
Expected: clean (no lint errors). Fix any I001 import-order issues; ruff is configured to enforce that.

- [ ] **Step 3: Verify committed artifacts**

```bash
git log --oneline -20
ls scripts/fitter_prototype/
ls docs/superpowers/spikes/
```

Expected: ~14 commits with `fitter:` prefix, full module set under `scripts/fitter_prototype/`, spike findings doc in `docs/superpowers/spikes/`.

---

## Spec coverage check

Each section of the design spec mapped to a task:

- §2 In/Out scope — Tasks 3 (output) + 4 (parser) + 5 (reduce) + 6 (classify) + 7 (corner-blend) + 8/9 (fit) + 11 (analyze)
- §3.1 Classifier — Task 6
- §3.2 LSPIA + chord-bound — Tasks 8 + 9
- §3.3 Corner-blend slot — Task 7
- §4 Module layout — Tasks 1, 3–11
- §5 Output types — Task 3
- §6 Corpus — committed before plan execution; Tasks 12, 13 use it
- §7.1 Synthetic tests — Task 8 (LSPIA on line/circle), Task 9 (refinement on half-circle)
- §7.2 Corpus automated — Task 10 (end-to-end test) + Task 12, 13 (real corpus runs without crashing)
- §7.3 Visual — Task 11 (analyze plots)
- §8 Phasing — Tasks 1–11 = D2; Tasks 12–14 = D3
- §9 Open questions — Task 14 §7
- §10 Dependencies — Task 1
- §11 References — Task 14 §9
