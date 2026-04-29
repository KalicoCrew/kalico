# Step 13 — Compatibility Layer: Offline Legacy G-code → G5-only Normalizer

## Overview

An offline preprocessor that converts legacy G-code (G0/G1/G2/G3/G5.1) into
G5-only output consumable by kalico's live pipeline. Pure text-to-text
transform: G-code file in, G-code file out.

The tool exists so that legacy slicers (OrcaSlicer, PrusaSlicer, Cura, etc.)
can be used with kalico without modification. Kalico-aware slicers emit G5
directly and never invoke the compat layer.

## Crate structure

- **Crate**: `rust/compat/` in the workspace.
- **Binary**: `kalico-compat`.
- **Dependencies**: `gcode` (lexer only). Does NOT depend on `geometry`,
  `nurbs`, `temporal`, or any other planner crate.

### CLI interface

```
kalico-compat [OPTIONS] <input.gcode> -o <output.gcode>
```

| Flag | Default | Description |
|------|---------|-------------|
| `--tolerance <µm>` | `5.0` | Max deviation from original geometry, applies to both spline fitting and arc conversion (µm) |
| `-o <path>` | stdout | Output file path |
| `<input>` | required (`-` for stdin) | Input G-code file |

Exit codes: `0` = clean success, `1` = success with warnings (some fallbacks
triggered), `2` = fatal error (unsupported plane, I/O failure).

## Architecture

**Single-pass streaming with boundary-tangent handoff (Approach C).**

```
stdin/file → gcode::lex() → Converter → G5 text writer → stdout/file
```

The `Converter` is a stateful iterator adapter that:

1. Maintains full modal state (position, E, F, active plane, distance mode,
   extrusion mode, G5 chain state).
2. Buffers consecutive G1/G0 moves into fittable runs.
3. On buffer flush (non-G1 token or run-break condition), runs the spline
   fitter with boundary tangent constraints, emits G5 lines.
4. Converts G2/G3 immediately via Goldapp, stores endpoint tangent for the
   subsequent G1 run.
5. Converts G5.1 immediately via exact degree elevation.
6. Canonicalizes source G5 passthrough (resolves implicit I/J to explicit).
7. Passes non-motion tokens through verbatim.

The iterator is built on `std::iter::Peekable` to support one-token lookahead
for end-of-run boundary tangent extraction from adjacent G2/G3 arcs.

## Modal state

The converter tracks:

| State | Updated by | Purpose |
|-------|-----------|---------|
| Position `[x, y, z]` | G0/G1/G2/G3/G5/G5.1/G92 | Absolute current position |
| `e` (absolute cumulative) | G1/G2/G3/G5/G5.1/G92/M82/M83 | Extrusion tracking |
| Distance mode | G90 (absolute) / G91 (relative) | Normalize XYZ to absolute |
| Extrusion mode | M82 (absolute) / M83 (relative) | Normalize E to absolute |
| Active plane | G17/G18/G19 | Arc plane selection |
| Feedrate | F-word on any motion | Current modal feedrate |
| `prev_g5_pq` | G5 chain tracking | Resolve implicit I/J on source G5 |
| `prev_tangent` | Arc endpoint / fitted-run endpoint | Boundary tangent handoff |

**Output normalization**: the output file begins with `G90` and `M82`.
All output X/Y/Z/E values are in absolute mode. G5 I/J/P/Q are explicit
relative control-point offsets per the LinuxCNC G5 convention (not absolute
coordinates).

**Initial feedrate**: if the first motion command in the input has no F-word
and no prior F has been seen, the preprocessor emits a fatal error (exit 2).
A valid input file must establish a feedrate before the first motion.

## Conversion rules

### G0 → G5 (collinear cubic)

Same as G1. G0 becomes a G5 with collinear control points. No special rapid
semantics in the output. F preserved if present.

### G1 → G5

Consecutive G1 moves with XY motion are grouped into fittable runs (see "Run
segmentation" below). Processing depends on run length:

- **Run length 1**: exact degree elevation to collinear cubic. Control points
  at 1/3 and 2/3 lerp between start and end. Zero error. E preserved exactly.
- **Run length 2–3**: per-segment collinear G5 (underdetermined for LS fitting).
- **Run length ≥ 4**: global cubic B-spline fitter (see "Spline fitter" below).

G1 moves with no XY motion are handled separately:
- **E-only** (retraction/prime): collinear G5 with zero XY delta.
- **Z-only**: collinear G5 with zero XY delta.

### G2/G3 → G5 (Goldapp 1991 multi-piece cubic Bézier)

Closed-form circular-arc-to-cubic-Bézier approximation.

- **XY plane only** (G17). G18/G19 arcs produce a fatal error — the live
  pipeline rejects non-XY G5.
- **I/J center-offset format only** (matching Klipper main branch; R-format
  is not supported by Klipper and produces an error).
- **Adaptive piece count**: `n = ceil(|θ| / θ_max(r, tol))` where `θ_max` is
  derived from the Goldapp per-piece error bound for the configured tolerance.
  Not a fixed quarter-arc split.
- **Full circles** (start == end): `θ = -2π` for G2 (clockwise), `θ = +2π`
  for G3 (counter-clockwise). Split into `n` pieces by the same adaptive
  formula.
- **Helical arcs**: the non-planar axis (Z for G17) linearly interpolates
  across the output G5 segments. Matches Klipper's `gcode_arcs.py` behavior.
- **Radius validation**: verify `|r_start - r_end| / r_avg < 0.001` (0.1%
  relative) before converting. If within threshold, snap endpoint to target
  (Klipper-compatible). If beyond threshold, warn and snap.
- **E distribution**: proportional to arc length across output G5 pieces.
- **Tangent storage**: the converter stores the arc's endpoint tangent
  (direction at the final point) for handoff to the subsequent G1 run's fitter.

### G5.1 → G5 (exact degree elevation)

Quadratic Bézier (3 CPs) → cubic Bézier (4 CPs):
- CP₁_new = (1/3)·P₀ + (2/3)·P₁
- CP₂_new = (2/3)·P₁ + (1/3)·P₂

Zero error, exact. E and F preserved.

### G5 → G5 (canonicalized passthrough)

Source G5 segments pass through with implicit I/J resolved to explicit values
using source-level G5-chain tracking (`prev_g5_pq`). This prevents chain
breakage when converted G5 segments are interleaved with source G5 segments
in the output.

### Non-motion tokens

| Token | Handling |
|-------|----------|
| G90/G91 | Update modal state. NOT emitted (output forced G90). |
| M82/M83 | Update modal state. NOT emitted (output forced M82). |
| G92 | Update modal state AND pass through. |
| G17 | Update modal state. Pass through. |
| G18/G19 | Update modal state. Pass through only if no subsequent motion uses G5 in that plane (the live pipeline rejects non-XY G5). If G18/G19 is followed by G2/G3 arcs, those arcs produce a fatal error. A `G17` is emitted before the first G5 after any G18/G19 to ensure the live pipeline sees XY plane. |
| M-codes | Pass through verbatim. |
| T-codes | Pass through verbatim. |
| Full-line comments | Pass through verbatim. |
| Marker comments | Pass through verbatim. |
| Inline comments | Dropped (lexer strips them during tokenization). |
| G54-G59, G10, G53 | Not tracked. Passed through verbatim if present. FDM slicers do not emit work-coordinate commands. If a file uses them, the preprocessor's position tracking will be incorrect — this is documented as unsupported. |
| Other G-codes | Passed through verbatim. Unknown G-codes do not affect modal state. |

## Run segmentation

Consecutive G1/G0 moves with XY motion are grouped into a fittable run. A run
breaks at:

- Any non-G1/G0 token (G2/G3, G5, G5.1, M-code, T-code, G92, comment)
- Feedrate (F) change
- Extrusion ratio (E_delta / XY_path_length) change beyond a threshold
- No XY motion (E-only or Z-only G1 moves)

Z variation does NOT break a run — the fitter operates in 3D and handles Z
as part of the fitted curve. Each output G5 piece has endpoint Z with interior
control-point Z at 1/3/2/3 linear interpolation (the G5 format constraint);
tolerance enforcement naturally produces shorter pieces where Z varies.

## Spline fitter

### Algorithm: global cubic B-spline approximation (Piegl-Tiller ch. 9/12)

**Input**: ordered 3D waypoints `W₀..Wₙ` from a fittable run, optional
start/end tangent direction constraints from adjacent arcs or fitted runs.

**Corner detection**: before fitting, scan deflection angles between
consecutive segments. Split at corners where `L·tan(θ/4) > tolerance` (L =
shorter adjacent segment length). Each sub-run is fitted independently;
tangent handoff between adjacent sub-runs preserves C1 at splits.

**Minimum sub-run length**: sub-runs with ≤ 3 waypoints emit per-segment
collinear G5 (exact degree elevation). No fitting attempted.

**Fitting procedure**:

1. **Parameterize** waypoints by cumulative chord length, normalized to [0,1].
2. **Initial fit**: global cubic B-spline with minimal knot count.
3. **Boundary conditions**: clamped tangent direction at sub-run start/end if
   adjacent arc/fitted-run tangent is available; otherwise from the first/last
   G1 segment direction.
4. **Solve** via QR decomposition in Bernstein basis with centered/scaled
   coordinates. NOT normal equations (avoid squaring the condition number).
5. **Error check**: polyline-to-curve 3D Hausdorff distance via recursive
   Bézier subdivision. Not just point-at-parameter residuals — the curve
   must not bulge between sample points.
6. **Refinement**: if max deviation > tolerance, insert knots where error is
   worst. Re-fit globally. Repeat until tolerance is met or max knot count
   reached.
7. **Decompose**: accepted B-spline spans → cubic Bézier segments via knot
   insertion to multiplicity 3 at each interior knot → individual G5 pieces.

**Fallback**: if a sub-run can't meet tolerance after max iterations, fall back
to collinear G5 for just that sub-run. Warn to stderr. Adjacent accepted spans
are preserved.

**Continuity**: C2 at interior knots (enforced by the global B-spline
representation). C1 at sub-run boundaries (enforced by tangent handoff). C0
minimum at all joints.

### E handling

Runs split at E-ratio changes, so within a fitted run the extrusion ratio
(E per mm of path) is constant. Each output G5 segment gets E proportional
to its arc length as a fraction of the total fitted arc length:

```
E_segment = total_ΔE_input × (arc_length_segment / total_arc_length_fitted)
```

This preserves constant extrusion per mm of actual toolhead travel. If the
fitted path is shorter or longer than the input polyline, the extrusion rate
per mm adjusts but total ΔE for the run is preserved exactly — output modal
E position stays consistent with input, preventing drift in subsequent
absolute E commands.

### Z handling

The fitter operates in 3D (XYZ). The XY components are fitted as a cubic
B-spline. Z is constrained by the output format: each G5 piece has endpoint Z
with interior control-point Z at linear 1/3/2/3 interpolation. The fitter
verifies the 3D deviation (including Z error from the linear constraint)
against the configured tolerance. Where Z varies significantly (vase mode,
layer transitions), tolerance enforcement naturally produces shorter pieces so
the per-piece linear Z approximation stays within tolerance.

### Tangent handoff (Approach C)

Boundary tangent sources:

| Transition | Start tangent source | End tangent source |
|------------|--------------------|--------------------|
| Arc → G1 run | Arc endpoint tangent (from Goldapp) | Peek at next token |
| G1 run → Arc | Stored prev_tangent | Arc start tangent (from Goldapp) |
| Fitted run → Fitted run | Previous run's final tangent | Next run's first G1 direction |
| G5/G5.1 → G1 run | G5/G5.1 endpoint tangent (from control points) | Peek at next token |
| G1 run → G5/G5.1 | Stored prev_tangent | G5/G5.1 start tangent (from control points via peek) |
| No neighbor | First G1 direction (natural-ish) | Last G1 direction (natural-ish) |

The peekable iterator provides one-token lookahead for the end-of-run boundary.
The trailing-boundary case (arc → G1) requires no lookahead — the arc's
endpoint tangent is stored during Goldapp conversion.

## Output format

### File preamble

```gcode
; Generated by kalico-compat from <input_filename>
; Tolerance: <tolerance_µm> µm
G90
M82
```

### G5 line format

```
G5 X<x> Y<y> Z<z> I<i> J<j> P<p> Q<q> E<e> F<f>
```

Geometric parameters always explicit: X/Y/Z/I/J/P/Q/E on every G5 line (no
implicit I/J chains). Coordinates to 3 decimal places (µm precision), E to 5
decimal places.

F follows modal persistence: emitted on the first G5 of each constant-feedrate
run, omitted on subsequent G5 segments at the same feedrate.

### Passthrough lines

Non-motion tokens (M-codes, T-codes, G92, full-line comments, markers)
are reconstructed from the parsed token or emitted from a parallel raw-line
index for byte-identical passthrough.

## Comment handling

- **Full-line comments** (`; ...`): preserved, passed through verbatim.
- **Marker comments** (`;LAYER:5`, `;TYPE:`, etc.): preserved as markers.
- **Inline comments** (on motion lines): dropped. The lexer strips them during
  tokenization. These are typically slicer debug annotations that become
  meaningless after conversion.

## Error handling

### Input errors

| Error | Behavior |
|-------|----------|
| Malformed G-code (lexer error) | Warn to stderr with line number, skip line, continue. If the malformed line is a motion command (G0/G1/G2/G3/G5/G5.1), fatal error (exit 2) to uphold the output guarantee. |
| G2/G3 in G18/G19 plane | Fatal error (exit 2). Live pipeline rejects non-XY G5 |
| G2/G3 with inconsistent radius | Warn, snap endpoint (Klipper-compatible), continue |
| G2/G3 with zero radius (I=J=0) | Fatal error (exit 2). Matches Klipper behavior ("G2/G3 requires IJ parameters"). |
| R-format arc (G2/G3 R...) | Fatal error (exit 2). Klipper rejects R-format. |

### Fitter errors

| Error | Behavior |
|-------|----------|
| Fit exceeds tolerance after max iterations | Warn, fall back to collinear G5 for that sub-run |
| Numerically singular LS system | Warn, fall back to collinear |
| Empty run / degenerate geometry | Skip silently (no motion to emit) |

### Output guarantee

The preprocessor always produces a valid G5-only output file. It never
silently drops motion commands. Every input motion command produces at least
one output G5 command.

## Testing strategy

### Unit tests

**Per conversion rule:**
- G0→G5: collinear cubic is geometrically exact
- G5.1→G5: degree elevation exact (verify CP formula)
- G2/G3→G5: Goldapp pieces — max radial error vs. analytic circle at various
  radii and sweep angles. Quarter-arc, half-arc, full circle, small arc (<5°),
  helical
- G5 passthrough: implicit I/J canonicalized to explicit

**Spline fitter:**
- Straight-line G1 sequence → collinear output (no spurious curvature)
- Circular-arc G1 sequence → within tolerance of polyline and underlying circle
- Corner detection — verify split triggers at sharp corners
- Short runs (≤3 waypoints) → collinear fallback
- Boundary tangent constraints — C1 at arc↔fitted-run junctions
- E redistribution — total E = ratio × fitted arc length
- 3D fitting with Z variation — Z within tolerance

### Modal state tests

- G90/G91 mode switching with G1 moves — verify absolute position computed correctly
- M82/M83 mode switching — verify E normalization to absolute
- G92 E reset — verify subsequent E values are correct
- Missing initial feedrate — verify fatal error
- G18 followed by G1 then G17 — verify G17 emitted before G5 output

### Integration tests on real G-code

- Round-trip `voron_cube_arc_fitted.gcode` (~161K lines, G1 + G2/G3)
- Round-trip `voron_cube_straight_line.gcode` (~216K lines, G1 only)
- Verify output parses through `gcode::lex` → `geometry::reduce` without errors
- Verify output contains only G5/G92/M/T/comment tokens
- Verify total E within expected bounds (ΔE preserved per run)
- Verify absolute E position at end of file matches expected
- Spot-check geometric deviation on sampled segments
- Source G5 with implicit I/J chain through converted segments — verify canonicalization

### Performance

Process the 216K-line corpus in under a few seconds on a development machine.

## References

- Goldapp, M. (1991). "Approximation of circular arcs by cubic polynomials."
  Closed-form constants for cubic Bézier approximation of circular arcs.
- Piegl, L. & Tiller, W. (1997). "The NURBS Book," 2nd ed. Ch. 9 (B-spline
  curve approximation), Ch. 12 (automatic knot placement).
- Beudaert, X. et al. (2012). "5-axis local corner rounding of linear tool
  path discontinuities." B-spline fitting for CNC tool paths.
- Tajima, S. & Sencer, B. (2016). "Kinematic corner smoothing for high speed
  machine tools." Corner smoothing under tolerance and axis limits.
- LinuxCNC RS274NGC §3.5.5 — G5 cubic Bézier spline specification.
