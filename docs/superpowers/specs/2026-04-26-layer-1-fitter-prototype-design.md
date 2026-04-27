# Layer 1 Spline Fitter — Python Prototype Design

**Date:** 2026-04-26
**Status:** Spec — design approved, implementation to follow
**Layer:** 1 (Geometry pipeline)
**Driver:** Validate windowed-fit + parameterized-corner-blend architecture before
Rust port (build-order step 7)

## 1. Context

The Layer 1 spline fitter is the highest-risk item in the rewrite per CLAUDE.md.
Slicer-emitted G1 sequences are academically underserved; the fitter must
classify each vertex (smooth / smoothable corner / hard corner / marker break),
fit smooth runs as NURBS within a chord-error tolerance, and emit
*parameterized* corner-blend slots for Layer 3 to finalize against dynamic
limits.

A literature triangulation (originally planned as a free-standing spike doc) is
folded into this prototype: running the prototype on real OrcaSlicer corpora
produces the measurements that validate the architectural calls, so the spike
findings are a byproduct of D3 rather than a separate paper artifact.

Python is chosen for the prototype:

- The fitter algorithm is unsettled — we will iterate on the LSPIA + chord-bound
  + classifier interaction in ways that benefit from edit-rerun-look-at-plot
  speed
- Performance is uncritical — receive-time on Pi 5, no inner loops at 40 kHz
- scipy + matplotlib provide both a reference baseline and inspection tooling
  for free
- CLAUDE.md's critical-path observations explicitly endorse "prototype early in
  Python or similar before committing"

The prototype is not production. Once the algorithm settles, F0 ports the
behavior (which the prototype's test suite fixes) to Rust as part of step 7.

## 2. Scope

### In

- G-code parser: G0/G1/G2/G3, ignore M-codes, comments, extrusion (geometry
  only)
- Geometric reduction: G2/G3 → arc descriptor (passthrough — not fitted);
  G0/G1 → polyline
- Vertex classifier: per-transition label
  `{smooth, smoothable_corner, hard_corner, marker_break}` (Sun 2018 CMLT-style)
- LSPIA fitter with chord-bound local refinement (Bi 2019)
- Parameterized cubic-Bezier corner-blend slot emitter (Zhao 2013 control-point
  pattern; *no* shape selection)
- Typed segment dataclasses + JSON-serializable output
- Corpus statistics: residual histogram, piece-count distribution,
  reject-category frequencies
- matplotlib diagnostic plots

### Out

- Layer 3 corner-blend shape finalization (deferred — needs dynamic limits)
- TOPP-RA, kinematic constraints
- Streaming / incremental processing — offline batch only
- Rust algebra contract compatibility — JSON output is for inspection, not
  contract-locked
- Multi-slicer corpus (OrcaSlicer only per current direction)
- Performance optimization

## 3. Algorithm choices

### 3.1 Vertex classifier — CMLT-style (Sun 2018)

Per-vertex label based on local geometric features. Single forward pass over the
polyline.

For each interior vertex `v_i` with neighbors `v_{i-1}, v_{i+1}`:

- Tangent vectors `t_in = v_i − v_{i-1}`, `t_out = v_{i+1} − v_i`
- Angle change `θ = angle(t_in, t_out)`
- Local segment lengths `|t_in|, |t_out|`

Classification rules (defaults — to be measured/tuned in D3):

- `marker_break` — set by parser when a non-G1 instruction interrupted the
  polyline (G0, layer change, M-code)
- `hard_corner` — `θ > θ_hard` (default 60°). Will route to junction-deviation.
- `smoothable_corner` — `θ_smooth < θ ≤ θ_hard` (default `θ_smooth = 15°`).
  Corner-blend slot emitted.
- `smooth` — `θ ≤ θ_smooth`. Continues current run.

Two adjacent smoothable corners separated by a very short segment (`<
seg_len_collapse`, default 0.05 mm) collapse into a single corner-blend slot.

### 3.2 Smooth-run fitter — LSPIA + chord-bound (Bi 2019)

A "smooth run" is a maximal sequence of consecutive `smooth`-classified
vertices.

Algorithm:

1. **Initialization.** Place `n_init` interior knots uniformly across the run's
   parameter domain (chord-length parameterization). Initial control points by
   least-squares fit (numpy `lstsq`).
2. **LSPIA iteration.** Adjust control points via the fixed-point iteration in
   Bi 2019 §3, which has a provable contraction. Stop when max residual change
   between iterations is below `ε_iter` (default 1e-9 mm) or iteration cap
   reached (default 100).
3. **Chord-bound refinement.** For each Bézier piece (between adjacent interior
   knots), use the strong convex-hull property to bound chord error
   analytically. If a piece's bound exceeds `ε_chord`, insert a knot at the
   parameter of the worst residual; restart LSPIA on the refined knot vector.
4. **Termination.** All pieces below tolerance, or iteration cap.

Output is a `FittedNurbs` (control points + knot vector + degree, see §5);
trivially convertible to `scipy.interpolate.BSpline` for evaluation/plotting.

Defaults: degree 3 (cubic), `ε_chord = 25 µm`. Both configurable via the CLI /
params object.

### 3.3 Parameterized corner-blend slot — Zhao 2013

For each `smoothable_corner`, emit a `CornerBlendSlot` containing:

- `position` — the corner vertex
- `t_in, t_out` — incident unit tangent vectors
- `seg_len_in, seg_len_out` — incident segment lengths (bounds control-point
  placement)
- `tolerance_budget` — deviation budget for the blend, default 50 µm
- `default_family` — `"cubic_bezier"`; Layer 3 may override per shape selection

The slot is *parameterized*, not finalized. Layer 3 (post-shape-selection)
reads the slot and produces a finalized cubic-Bezier (or other family) NURBS in
its place. The prototype emits the slot but does not finalize.

For prototype plotting, a *placeholder* finalization (control points at
`seg_len_in/3, seg_len_out/3` along the tangents — Pateloup 2004 default) is
generated. The placeholder is clearly labeled in code as
non-production-shape-selection.

## 4. Module layout

```
scripts/fitter_prototype/
├── __init__.py
├── README.md
├── corpus/
│   ├── voron_cube_arc_fitted.gcode
│   └── voron_cube_straight_line.gcode
├── parser.py
├── reduce.py
├── classify.py
├── fit.py
├── corner_blend.py
├── output.py
├── analyze.py
├── run.py
└── tests/
    ├── __init__.py
    ├── test_parser.py
    ├── test_classify.py
    ├── test_fit_synthetic.py
    └── test_end_to_end.py
```

Per-module responsibility:

- `parser.py` — `parse(text: str) -> list[GCodeToken]`. Token types:
  `Move(kind, x, y, ...)`, `Arc(kind, x, y, i, j, ...)`, `Marker(reason)`.
  Strict on unexpected commands within the OrcaSlicer subset; raises
  `ParseError` with line number.
- `reduce.py` — `reduce(tokens) -> list[GeometricSegment]`. Output is either
  `Polyline` (sequence of XY points, with marker boundaries between them) or
  `ArcSegment` (start, end, center, direction, passthrough).
- `classify.py` — `classify(polyline, params) -> list[VertexLabel]`. Pure
  function, no allocation of NURBS. One label per interior vertex.
- `fit.py` — `fit_smooth_run(points: ndarray, params) -> FittedNurbs`. LSPIA
  + chord-bound refinement.
- `corner_blend.py` — `make_slot(corner_vertex, tangents, lengths, params) ->
  CornerBlendSlot`. Plus `placeholder_finalize(slot) -> ndarray` for plotting.
- `output.py` — dataclasses and JSON serializer.
- `analyze.py` — corpus stats + matplotlib plots, given a list of typed output
  segments.
- `run.py` — CLI: takes one or more `.gcode` files, writes JSON output and
  analysis plots to a results directory.

## 5. Output types

```python
from dataclasses import dataclass
import numpy as np

@dataclass
class FittedNurbs:
    control_points: np.ndarray        # shape (n, 2) for XY
    knots: np.ndarray                 # full knot vector (clamped)
    degree: int
    source_vertex_range: tuple[int, int]
    max_residual: float

@dataclass
class CornerBlendSlot:
    position: np.ndarray              # shape (2,)
    t_in: np.ndarray                  # shape (2,) unit
    t_out: np.ndarray                 # shape (2,) unit
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
```

Output per gcode file: a list of these (in path order), JSON-serializable.

## 6. Corpus

Two files committed at `scripts/fitter_prototype/corpus/`:

- `voron_cube_arc_fitted.gcode` (~4.0 MB) — Voron Design Cube + 3D Benchy,
  OrcaSlicer 2.3.2, arc-fitting **on**. 132 156 G1 + 9 710 G2/G3 commands.
- `voron_cube_straight_line.gcode` (~5.3 MB) — same model, arc-fitting
  **off**. 196 860 G1 commands, 0 arcs.

Provenance: OrcaSlicer 2.3.2, ABS profile, both files generated 2026-04-26.

The two files share a model but differ in slicer post-processing — their
side-by-side comparison answers the "how much does slicer arc-fit reduce our
work" question.

Bambu Studio + PrusaSlicer corpora deferred per current direction.

## 7. Validation

Three tiers, in order of stringency.

### 7.1 Synthetic tests

Known smooth curves (unit circle, published Bézier, etc.) sampled into G1
polylines at varying density. Fit must recover the original geometry within
tolerance, with piece count near the analytical minimum.

### 7.2 Corpus, automated

Run on both OrcaSlicer files. Asserts:

- All vertices classified
- All smooth runs fit within `ε_chord`
- All G2/G3 arcs passed through unchanged
- Output is JSON-deserializable into the same dataclasses
- No crashes

### 7.3 Corpus, visual

matplotlib plots:

- Residuals along path (per smooth run)
- Classifier decisions colored on the polyline
- Output NURBS overlaid on input polyline
- Histogram of piece counts per smooth run
- Histogram of reject categories per file

For visual inspection only — confidence-establishing, not pass/fail.

### 7.4 Out of scope

- Performance benchmarks (will revisit at Rust port)
- Comparison to scipy.splprep or ArcWelder (skipped per the language-decision
  conversation — we already know they fail in characteristic ways; reproducing
  isn't the goal)

## 8. Phasing

Two deliverables, ~4 working days total.

### D2 — Algorithm implementation (~3 days)

1. parser + reduce — handle the OrcaSlicer subset; validate on both corpus
   files.
2. classifier — CMLT with default thresholds.
3. LSPIA fitter — initial knot placement, fit, chord-bound refinement loop.
4. Corner-blend slot emitter — with placeholder finalization for plots.
5. Output dataclasses + JSON serialization.
6. CLI entry-point.

End state:
`python scripts/fitter_prototype/run.py corpus/voron_cube_arc_fitted.gcode --out
results/` produces JSON + plots.

### D3 — Corpus run + writeup (~1 day)

1. Run on both corpus files.
2. Comparison statistics (arc-fitted vs straight-line, same model).
3. Spike findings writeup at
   `docs/superpowers/spikes/2026-04-26-layer-1-fitter-spike.md` with measured
   numbers backing the architectural calls.

End state: spike findings doc committed; prototype validated against the
architectural questions in §9.

## 9. Open questions answered by the prototype

These are the spike's measurement-needing questions that lit could not answer:

- Q1: Achievable residual on slicer-emitted G1 — does the fitter actually hit
  25 µm on Voron Cube + Benchy? What's the realistic floor?
- Per-mm-of-print rate of each reject category (sanity check on
  smoothable-vs-hard threshold)
- Piece-count distribution per smooth run
- Whether `θ_smooth = 15°` and `θ_hard = 60°` defaults are reasonable on
  OrcaSlicer output
- How much does OrcaSlicer's arc-fit reduce our fitting work? (Direct
  arc-fitted vs straight-line comparison)

If the prototype reveals an architectural call (Layer 1/Layer 3 split,
three-path fallback, classifier label set) needs revision, that is a finding
for F0 to absorb before the Rust port.

## 10. Dependencies

Add a `prototype` dependency group to `pyproject.toml`:

```toml
[dependency-groups]
prototype = [
  "scipy>=1.13",
  "matplotlib>=3.8",
]
```

`numpy` is already a project dep.

## 11. References

Fitter-specific. Foundational NURBS/B-spline references (Piegl & Tiller, de
Boor, Farin) live in the Layer 0 algebra spec.

- Bi, Huang, Lu, Zhu, Ding (2019). "A general, fast and robust B-spline fitting
  scheme for micro-line tool path under chord error constraint." *Sci. China
  Tech. Sci.* 62. — LSPIA + chord-bound refinement.
- Sun, Yu, Wang, Xie (2018). "A smooth tool path generation and real-time
  interpolation algorithm based on B-spline curves." *Adv. Mech. Eng.* 10. —
  CMLT classifier.
- Zhao, Zhu, Ding (2013). "A real-time look-ahead interpolation methodology
  with curvature-continuous B-spline transition scheme for CNC machining of
  short line segments." *Int. J. Machine Tools & Manufacture* 65. — Cubic
  B-spline corner blend.
- Tajima, Sencer (2016). "Kinematic corner smoothing for high-speed machine
  tools." *Int. J. Machine Tools & Manufacture* 108. — Architectural reference
  for dynamic-limit-dependent corner shape (Layer 3 finalization, not in this
  prototype).
- Park, Lee (2007). "B-spline curve fitting based on adaptive curve refinement
  using dominant points." *Computer-Aided Design* 39(6). — Knot placement
  reference.
- He, Ou, Yan, Lee (2015). "A chord error conforming tool path B-spline fitting
  method for NC machining based on energy minimization and LSPIA." *J. Comp.
  Design & Eng.* 2(4). — Tolerance budgeting (50%-margin convention).
- Pateloup, Duc, Ray (2004). "Corner optimization for pocket machining." *Int.
  J. Machine Tools & Manufacture*. — Cubic Bezier corner placeholder default.
