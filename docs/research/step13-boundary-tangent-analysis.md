---
topic: Step 13 compat layer boundary tangent analysis at G1-run edges
created: 2026-04-29
last_updated: 2026-04-29
verified_claims:
  - 2026-04-29 INCONCLUSIVE — "Approach A (single-pass streaming without boundary tangent handoff) is the best architecture for Step 13" — the architectural simplicity argument holds but the "minimal benefit" sub-claim about boundary tangent matching is NOT supported by corpus data; a significant minority of G1-to-arc transitions are tangent-continuous and would benefit from boundary hints.
sources:
  - Tajima, S. and Sencer, B. "Kinematic corner smoothing for high speed machine tools." Int J Mach Tools Manuf, 108, 27-43 (2016)
  - Beudaert, X., Lavernhe, S., Tournier, C. "Feedrate interpolation with axis jerk constraints on 5-axis NURBS and G1 tool path." Int J Mach Tools Manuf, 57, 73-82 (2012)
  - Goldapp, M. "Approximation of circular arcs by cubic polynomials." CAGD, 8, 227-238 (1991)
  - Sonny Jeon, "Improving Grbl's Cornering Algorithm" (2011) — junction deviation
  - OrcaSlicer 2.3.2 arc-fitted Voron cube corpus (scripts/fitter_prototype/corpus/voron_cube_arc_fitted.gcode)
---

# Step 13 Boundary Tangent Analysis

## Summary

Empirical analysis of the OrcaSlicer arc-fitted Voron cube test corpus reveals that G1-to-arc and arc-to-G1 transitions exhibit a bimodal tangent-mismatch distribution: roughly 32-36% of transitions have < 5 degree mismatch (slicer-intended tangent continuity), while roughly 42-44% have > 45 degree mismatch (intentional corners where junction deviation is the correct handler). The "minimal benefit" argument for ignoring boundary tangent information in Approach A is therefore overstated for the tangent-continuous population: a spline fitter without boundary hints will produce natural-end-condition tangents that may diverge from the arc's tangent at those smooth transitions, incurring unnecessary velocity reduction through Layer 2's junction-deviation cornering. However, Approach C's one-token lookahead (providing the adjacent arc's tangent as a clamped boundary condition) is trivially implementable in Rust via `.peekable()`, making the complexity argument against it weak.

## Verified claim — 2026-04-29

**Original claim:** "Approach A (single-pass streaming with buffered G1 window) is the best architectural choice for the Step 13 legacy-G-code -> G5-only offline preprocessor, compared to Approach B (two-pass full-file analysis) and Approach C (single-pass with boundary-tangent handoff)."

### Verification

#### Corpus analysis of G1-arc boundary tangent alignment

The OrcaSlicer arc-fitted Voron cube (`voron_cube_arc_fitted.gcode`, 240 layers, ~132K G1 + ~9.7K G2/G3) was analyzed for tangent alignment at G1-run/arc boundaries.

**G1-to-arc transitions (7,767 analyzed):**

| Mismatch range | Count | Percentage |
|---|---|---|
| 0-1 deg | 352 | 4.5% |
| 1-5 deg | 2,162 | 27.8% |
| 5-10 deg | 792 | 10.2% |
| 10-20 deg | 518 | 6.7% |
| 20-45 deg | 678 | 8.7% |
| 45-90 deg | 1,386 | 17.8% |
| 90-180 deg | 1,879 | 24.2% |

**Arc-to-G1 transitions (7,043 analyzed):**

| Mismatch range | Count | Percentage |
|---|---|---|
| 0-1 deg | 563 | 8.0% |
| 1-5 deg | 1,994 | 28.3% |
| 5-10 deg | 542 | 7.7% |
| 10-20 deg | 304 | 4.3% |
| 20-45 deg | 527 | 7.5% |
| 45-90 deg | 1,451 | 20.6% |
| 90-180 deg | 1,662 | 23.6% |

**G1 run length statistics near arc boundaries:**

| Statistic | Value |
|---|---|
| Total G1 runs | 18,838 |
| Mean run length | 7.0 |
| Median run length | 2 |
| G1 runs terminated by arc | 7,572 |
| Arc-adjacent short runs (<=3 G1s) | 4,366 (57.7%) |
| Arc-adjacent medium runs (4-20 G1s) | 2,944 (38.9%) |
| Arc-adjacent long runs (>20 G1s) | 262 (3.5%) |

#### Key finding: short G1 runs near arcs are common

57.7% of arc-adjacent G1 runs contain 3 or fewer G1 segments. A spline fitter operating on a 1-3 segment run has very little data to infer boundary tangent direction; the natural-end-condition tangent will closely match the last G1 segment's direction anyway (for a 1-segment run, exactly so). For these short runs, boundary tangent information from the adjacent arc adds minimal value because the fitter's output is essentially a degree-elevated version of the G1 segments — there is little smoothing to be done.

For medium runs (4-20 segments, 38.9%), the fitter does meaningful smoothing and the boundary tangent constraint could improve the endpoint tangent direction. This is where Approach C would provide the most benefit.

### Sources
- OrcaSlicer 2.3.2 Voron cube corpus analysis, 2026-04-29
- Tajima & Sencer 2016, corner smoothing for high-speed machine tools
- Goldapp 1991, circular arc cubic Bezier approximation

### Caveats / unchecked assumptions
- Analysis covers only one slicer (OrcaSlicer 2.3.2) and one model (Voron cube). Other slicers or models may have different G1/arc interleaving patterns.
- The velocity penalty from tangent mismatch at smooth boundaries depends on junction-deviation parameter and machine acceleration limits; not quantified here.
- Whether the spline fitter's natural boundary condition vs. clamped boundary condition produces meaningfully different output for medium-length G1 runs (4-20 segments) was not tested with an actual fitter implementation.
