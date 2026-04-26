# Layer 1 Spline Fitter — Spike Findings

**Date:** 2026-04-26
**Status:** Findings (literature triangulation + measured data on OrcaSlicer corpora)
**Layer:** 1 (Geometry pipeline)
**Driver:** Inputs to the F0 Layer 1 architecture spec

## 1. Context

The original spike asked five architectural questions about the Layer 1
windowed/streaming fitter. Rather than answer them from literature alone, the
work was folded into a Python prototype run end-to-end on real OrcaSlicer
output, so each architectural call is backed by measured numbers in addition
to lit triangulation.

The brainstorm-time correction about corner-blend shape selection — that it is
genuinely dynamic-limit-dependent, not invariant — landed in `CLAUDE.md` and
in the prototype's parameterized-corner-blend-slot design before the prototype
was built. The Layer 1 / Layer 3 split is reflected in the spec and the code.

## 2. Method

- **Literature triangulation** across 18 sources covering streaming CNC
  fitters, offline B-spline fitting under chord-error constraints, knot
  placement strategies, reject criteria, and practical hobby-firmware reports.
  Full grouping below in §9.
- **Python prototype** at `scripts/fitter_prototype/` implementing the full
  Layer 1 pipeline: G-code parser → geometric reduction → CMLT-style vertex
  classifier → LSPIA + chord-bound refinement fitter → parameterized
  corner-blend slot emitter → JSON output + matplotlib diagnostic plots.
- **Two corpora** of the same model (Voron Design Cube + 3D Benchy), sliced
  by OrcaSlicer 2.3.2 with arc-fitting on and off respectively. Same model,
  same profile, only the slicer's G2/G3 emission toggled.
- **Default fitter parameters:** degree 3, `n_init_interior = 4`,
  `ε_chord = 25 µm`, classifier thresholds `θ_smooth = 15°`, `θ_hard = 60°`.

The prototype is `scripts/fitter_prototype/`; the spec it implements is
`docs/superpowers/specs/2026-04-26-layer-1-fitter-prototype-design.md`; the
plan executed is at
`docs/superpowers/plans/2026-04-26-layer-1-fitter-prototype.md`.

## 3. The slicer-G1 corpus, characterized

| | arc-fitted | straight-line |
|---|---|---|
| File size | 4.0 MB | 5.3 MB |
| G1 commands | 132 156 | 196 860 |
| G2/G3 commands | 9 710 | 0 |
| After reduction: polylines | 17 573 | (≈ similar) |
| Polyline length: min / median / max | 2 / 3 / 214 | (similar) |
| Output segments | 61 227 | 65 442 |

Two corpus-shape findings dominate everything else:

1. **Polylines are short.** Median length is 3 vertices. The slicer chops the
   tool path into many small polylines separated by markers (G0, layer change,
   M-code, Z-only G1). Long smooth runs are rare; most fittable inputs have
   3–10 vertices.
2. **OrcaSlicer's arc-fit is conservative.** Total segment count drops only
   ~6 % from straight-line to arc-fitted (61k vs 65k). The arc-fit replaces
   curved sections with G2/G3, but most G1 (e.g. polygon walls, infill,
   short transitions) is left untouched. ArcWelder reports 75 % command
   reduction on Benchy first-layer; OrcaSlicer's arc-fit is much less
   aggressive than that.

## 4. Findings per question

### Q1 — Achievable residual

**Working answer: default `ε_chord = 25 µm` is comfortable; 96.6 % of fits
land within 5–8 µm of input geometry, deep inside the budget.**

Arc-fitted run, fit residuals:

- p50: ≈ 0 µm
- p95: 4.9 µm
- p99 (healthy fits): ~few × 10 µm

Same shape on straight-line run (p95 8.3 µm). Both far below the 25 µm
budget — the fitter is rarely binding on tolerance.

**Caveat: ~3.4 % of fits are pathological** (max residual > 1 mm, up to 10^39
mm in the worst cases). Cause: LSPIA with `n_init_interior = 4` produces 8
control points; on a smooth-run polyline with only 3–4 input vertices the
linear system is underdetermined, and the minimum-norm LSQ solution combined
with the LSPIA fixed-point iteration can produce control points many orders of
magnitude away from the input. Chord-bound refinement keeps the curve close to
the input *at sample points*, but cannot pull control points back. **F0 must
fix this before measuring real residual numbers** — cap `n_init_interior` at
`max(0, vertex_count − degree − 1)` so the LSQ is never underdetermined. The
3.4 % rate is the same in arc-fitted and straight-line runs, confirming an
algorithmic bug rather than a corpus issue.

### Q2 — Reject anatomy

**The reject-metadata contract works as designed.** Each non-smooth vertex
emits either a `CornerBlendSlot` (with in/out tangents, segment lengths,
tolerance budget) or a `JunctionDeviation` (position + angle). Both are
JSON-serializable and contain everything the corner-path coordinator needs to
route to the cubic-Bezier blend (Layer 3) or to junction-deviation handling
without re-parsing geometry.

Distribution on the arc-fitted corpus (44 074 corners total):

- Hard (`θ > 60°`, junction-deviation): 79.3 %
- Smoothable (`15° < θ ≤ 60°`, corner-blend slot): 20.7 %

Straight-line run: 74.2 % hard / 25.8 % smoothable. The slicer-arc-fit
removes some smoothable corners by replacing them with G2/G3, shifting the
remaining-corner distribution slightly toward hard.

That hard-corner fraction is much higher than the lit suggests for
CAM-emitted polylines (which are usually nearly all smoothable, since CAM
chord-tolerance is microns). Slicer-G1 has many *intentional* hard corners:
object outlines (cube edges), polygon walls, layer-change geometry. The
default `θ_hard = 60°` looks reasonable on this corpus — it cleanly
separates "real corner" from "decimation noise."

### Q3 — Output structure (one NURBS per smooth run, refined inside)

**Confirmed in shape, but piece counts are inflated by the under-determination
bug.** Median 11 Bezier pieces per fit, p95 25, on a median 4-vertex input run.
That's ~3 pieces per input segment — way more than the geometry warrants.

Once `n_init_interior` is capped per-run (Q1 caveat), piece counts will drop
substantially for short runs. The architectural call (one NURBS per smooth
run, refined internally to honor chord-error) holds; the prototype's specific
piece-count distribution doesn't generalize until the bug is fixed.

### Q4 — Streaming lookahead

**Run-end markers dominate as natural lookahead boundaries; a max-window cap
is unnecessary on this corpus.** Polylines are bounded above by 214 vertices
(observed max) and the corpus is dominated by ≤ 10-vertex runs. The
configured-but-unused max-window-vertex cap (default 64–128 in the spec) was
never approached.

The corner-context buffer (~3–5 vertices past a reject, needed so the
corner-rounder gets a stable outbound tangent) was implicitly satisfied because
polylines themselves are short; the natural boundary is closer than any soft
cap would force. **F0 should keep the cap parameter for safety** (a vase-mode
print could produce a single multi-thousand-vertex run), but operationally
it's a guard against pathology, not a routine constraint.

### Q5 — Configuration knobs

The five-knob set holds:

- **Fitter:** `eps_chord_mm` (residual tol), `theta_smooth_deg` /
  `theta_hard_deg` (corner classifier), max window vertices (latency cap).
- **Corner-rounder:** blend tolerance budget, quality target (Layer 3 input,
  Tajima/Sencer 2016 shape selection).

Measured-default justification:

- `eps_chord_mm = 25 µm` is comfortably under-utilized on this corpus (p95
  fits at 5–8 µm). Could plausibly tighten to 10 µm without hurting piece
  count materially — verify after the under-determination fix.
- `theta_smooth_deg = 15°` and `theta_hard_deg = 60°` produce a sensible
  ~20/80 smoothable/hard split on slicer-G1. Diversify the corpus before
  changing.
- `max_window_vertices` was never binding; default 64 is fine.
- Blend tolerance budget defaults to 50 µm (≈ 2× fitter tolerance) per the
  half-margin convention.

## 5. Three-path coordination (Layer 1 / Layer 3 split)

The CLAUDE.md edits committed earlier in this conversation make the split
explicit:

- **Layer 1** emits parameterized corner-blend slots — in/out tangents,
  segment lengths, tolerance budget — but does *not* finalize the curve
  family or control-point placement. Cubic Bezier is the default family, but
  not the fixed shape.
- **Layer 3** finalizes shape against dynamic limits (accel + jerk) per
  Tajima & Sencer 2016, which establishes that optimal-time-through-corner
  curve shape genuinely depends on the dynamic-limit context. The same
  geometric corner has different optimal control-point placements at
  different a/j ratios.

The prototype's `placeholder_finalize` (Pateloup 2004 default, control points
at 1/3 along incident segments, middle two collapsed at the corner) is used
*only* for plotting and is clearly labeled as non-production shape selection.

## 6. Algorithm-family recommendation

For the F0 Rust port:

- **Smooth-run fits:** LSPIA (Bi 2019 §3) with chord-bound refinement.
  Provably convergent, simple to implement, well-suited to streaming use
  once the under-determination bug is fixed.
- **Vertex classification:** CMLT-style (Sun 2018), single forward pass with
  per-vertex angle threshold.
- **Smoothable corners:** parameterized cubic-Bezier slot emission only at
  Layer 1; shape selection deferred to Layer 3 (Tajima/Sencer 2016).
- **Hard corners and marker breaks:** junction-deviation downstream, with
  velocity reduction at receive time.
- **Arc passthrough:** G2/G3 emit `ArcSegment` directly without going through
  the fitter.

**FIR-convolution path (Tajima/Sencer 2017) rejected.** It is mathematically
elegant but incompatible with the algebraic-closure pipeline (the convolution
is in time, downstream of TOPP-RA, not at receive time on geometry) and it
doesn't respect deliberate sharp corners — they get smoothed whether the user
wants them or not. Documented for future-me so it doesn't get re-litigated.

## 7. Open questions for F0

- **Fix the under-determination first.** Cap `n_init_interior` per-run before
  re-measuring residual or piece-count distributions.
- Diversify the corpus. OrcaSlicer is the only slicer measured here; Bambu
  Studio and PrusaSlicer outputs may shift the corner-classification
  distribution and the segment-length characteristics.
- Add a curved-surface-dominated model (vase, sphere, organic shape). The
  Voron cube + Benchy corpus is corner-rich; vase-mode prints have long
  smooth runs and would stress the lookahead-window question more.
- Measure piece-count distributions and per-mm reject rates again after the
  under-determination fix. Current numbers (median 11 pieces) are inflated.
- Whether the placeholder cubic-Bezier finalization (Pateloup 2004) gives
  visibly worse motion than dynamic-limit-aware Layer 3 selection is a
  Layer 3 question, not Layer 1's.

## 8. Language-stack recommendation

**Python for prototype, Rust for production.** Per the conversation that
produced this spike, the prototype is the Python that already exists at
`scripts/fitter_prototype/`. F0 will port the algorithm — once the
under-determination fix lands and the algorithm is settled — to Rust as
build-order step 7.

Three risks were called out at decision time:

- **Port becomes its own project.** Mitigation: keep Python surgical (already
  done — fitter logic only, no analysis-tooling drift into algorithm code).
- **Vectorized numpy patterns don't map.** Mitigation: the inner LSPIA loop
  in `fit.py` is sequential by construction; the heavy use of `lstsq` and
  `BSpline` is bounded to the fit primitive itself.
- **Drift between prototype and Rust port.** Mitigation: cross-check the Rust
  port against the Python prototype on a fixed corpus, sympy-oracle style.
  The corpus is committed at `scripts/fitter_prototype/corpus/`.

## 9. References

Streaming CNC fitters:

- Zhao, Zhu, Ding (2013). "A real-time look-ahead interpolation methodology
  with curvature-continuous B-spline transition scheme for CNC machining of
  short line segments." *Int. J. Mach. Tools Manuf.* 65: 88–98.
- Bi, Huang, Lu, Zhu, Ding (2019). "A general, fast and robust B-spline
  fitting scheme for micro-line tool path under chord error constraint."
  *Sci. China Tech. Sci.* 62. — **LSPIA + chord-bound refinement**, the
  primary algorithm reference for the prototype.
- Sun, Yu, Wang, Xie (2018). "A smooth tool path generation and real-time
  interpolation algorithm based on B-spline curves." *Adv. Mech. Eng.* 10. —
  **CMLT classifier**, the per-vertex segmentation precedent.
- Lin, Tsai, Yau (2007). "Development of a real-time look-ahead interpolation
  methodology with spline-fitting technique for high-speed machining." *Int.
  J. Adv. Manuf. Tech.* 47.

Corner blends and shape selection:

- **Tajima & Sencer (2016). "Kinematic corner smoothing for high-speed
  machine tools." *Int. J. Mach. Tools Manuf.* 108: 27–43.** — load-bearing
  reference for the Layer 3 dynamic-limit-aware shape selection.
- Tajima & Sencer (2017). "Global tool-path smoothing for CNC machine tools
  with uninterrupted acceleration." *Int. J. Mach. Tools Manuf.* 121: 81–95.
  — FIR-based alternative, considered and rejected.
- Pateloup, Duc, Ray (2004). "Corner optimization for pocket machining."
  *Int. J. Mach. Tools Manuf.* — Pateloup default cubic-Bezier placement,
  used as the prototype's placeholder finalization.

Knot placement and reject criteria:

- Park, Lee (2007). "B-spline curve fitting based on adaptive curve
  refinement using dominant points." *Computer-Aided Design* 39(6).
- He, Ou, Yan, Lee (2015). "A chord error conforming tool path B-spline
  fitting method for NC machining based on energy minimization and LSPIA."
  *J. Comp. Design Eng.* 2(4). — 50%-margin tolerance convention.
- "Tool-path continuity determination based on machine learning method"
  (2021). *Int. J. Adv. Manuf. Tech.* — soft-classifier reject precedent.

Practical / hobby-firmware:

- ArcWelderLib (FormerLurker), open-source. Default chord tolerance 50 µm,
  reports ~75 % command reduction on Benchy.
- Bambu Studio / OrcaSlicer arc-fit. Three-point circumscribed-circle fit;
  OrcaSlicer's default is documented as conservative, consistent with the
  6 % segment reduction measured here.

Background (foundational, cited in the Layer 0 algebra spec, not repeated
here): Piegl & Tiller, de Boor, Farin, Schoenberg, Eilers & Marx, Dierckx
FITPACK.
