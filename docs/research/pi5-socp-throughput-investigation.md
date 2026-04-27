# Pi 5 SOCP Throughput Investigation

**Date:** 2026-04-27
**Hardware under test:** Raspberry Pi 5 Model B Rev 1.0, BCM2712 SoC, 4× Cortex-A76 @ 2.4 GHz, 2 GB LPDDR4X, VideoCore VII GPU. Klipper + Moonraker running idle in background (printer not in motion). 1.5 GB free RAM.
**Software under test:** Branch `sota-motion`, Layer 2 single-segment SOCP via Clarabel 0.11.1 (`rust/temporal/src/topp/`), as of commit `85e7fa9c` plus the SLP outer loop from `0177d53a`. Rust 1.95.0 stable, target aarch64-unknown-linux-gnu.
**Driver:** Build-order Step 4.5 (Layer 2 multi-segment) brainstorming surfaced a question about whether the per-segment SOCP cost was tractable for math-optimal trajectory planning on the actual target host. Spec §6.6 estimated "tens of milliseconds for N=200 fixtures"; preliminary measurement showed worst-case cubic@N=200 at 1.6 seconds. This artifact resolves the question.

---

## TL;DR

- **GPU offload to VideoCore VII is conclusively dead** — no FP64 hardware, GPU FP32 throughput ≤ CPU FP64 throughput, problem size 4–5 orders of magnitude below GPU break-even.
- **A single one-line patch** (loosen Clarabel tolerances from 1e-8 default to 1e-5) **gives an 11× speedup on the worst case** (cubic@N=200: 1596 ms → 142 ms), with all existing tests still passing. (Inter-grid feasibility is implicitly assumed adequate at sufficient N — same assumption as at default tolerance — but worth a one-shot validation test; see Caveat 8.)
- **`opt-level = 3` on the workspace release profile** (was `"z"`, MCU-anticipatory) gives ~25% additional headroom on host builds.
- **Adaptive N is essential**: N=20 cubic = 6.5 ms; N=200 cubic = 142 ms. A 1mm slicer-output G1 segment doesn't need 200 grid points.
- **Per-segment parallelism scales near-linearly to 4 cores at small N** (N=20: 3.7× speedup) but breaks at large N due to memory bandwidth saturation on shared L3. 3-thread is the safe production default (avoids Klipper contention on cores 0–1).
- **The throughput-non-negotiable principle (CLAUDE.md "Non-negotiable constraints") is satisfiable on this hardware** in the **offline-batch operating model** (planner finishes ahead of motion or pre-plans the file). It is *not* satisfied as a sustained motion-rate streaming requirement — that framing was wrong, see "(A) joining-with-SOCP-per-iter feasibility math (corrected after Codex review)" below.
- **Step 4.5's (A) vs (B) joining-vs-solving choice is no longer hardware-feasibility-bound.** Pick (A) per the throughput principle; (B)'s 3–8% trajectory-time regression on ramp-bound segments is a knowing choice we don't make.
- **Multi-segment SOCP analysis** for Clarabel-as-black-box: per-segment wins for Step 4.5. A purpose-built multi-segment formulation with warm-start / shared factorization could change the answer; deferred to Step 8 / Step 9.

## Context

Build-order Step 4 (single-segment SOCP prototype, in flight) implements the Consolini-Locatelli 2024 SOCP relaxation via Clarabel + Lee 2024 SLP outer loop. Step 4 spec §6.6 set "wall-clock per fixture" as a sanity log only, with an order-of-magnitude expectation of "tens of ms" extrapolated from Clarabel's published benchmarks on much-larger problems. Step 4.5 brainstorming asked whether real-world performance on the actual host (Pi 5) supports either of two architectural options:

- **(A)** SOCP re-solved on every joining iteration (math-optimal trajectory; expensive)
- **(B)** Cheap-kinematic forward/reverse joining + SOCP-once-at-finalize (3–8% slower trajectory than (A) on ramp-bound segments per `kalico-verifier` analysis)

The user's hard constraint: never produce a slower trajectory than math-optimal. The throughput-vs-quality trade can only be resolved one way — find the compute headroom for (A), or accept the regression of (B). The principle disallows the latter.

## Investigation methodology

1. **Baseline** — synthetic single-segment fixtures (straight 100mm, quarter-circle arc R=20, varying-curvature cubic NURBS) at grid sizes N ∈ {20, 50, 100, 200} on the Pi 5 single core.
2. **GPU viability research** — independent agent dispatch with sources/citations, returned a structured brief (full text in `## GPU offload finding` below).
3. **Profile / instrument** — patched `slp_solve` and `solve_with_cuts` in the bench checkout to dump SLP outer-iter counts and per-Clarabel-solve IPM iteration counts and wall-clock to stderr. Rebuilt and re-ran.
4. **Tolerance experiments** — patched Clarabel `tol_gap_abs/rel` and `tol_feas` from default 1e-8 to 1e-5 and 1e-4. Re-benched; ran the existing test suite (which includes closed-form Biagiotti-Melchiorri ground-truth comparison on fixtures 1+2 at 6% tolerance) for quality regression.
5. **CPU tuning** — built with `RUSTFLAGS="-C target-cpu=native"` (auto-detected as cortex-a76); compared to opt=3 baseline.
6. **Per-segment parallelism** — wrote a simple thread-fanout benchmark (`pi5_parallel.rs`) to test 1/2/3/4-thread scaling at multiple N.
7. **Bench tooling** lives in `rust/temporal/examples/{pi5_perf.rs,pi5_parallel.rs}` for reproducibility.

All measurements: median of 30–200 iterations after a 5-iter warm-up, single-process, `taskset -c 3` pinning where relevant to isolate from Klipper background activity. Klipper + Moonraker running but printer not in motion.

## Findings

### Finding 1 — GPU offload finding

External agent research (general-purpose agent, dispatched against current literature and Pi 5 hardware sources). Three independent dispositive findings, any one of which would be sufficient to reject GPU offload:

1. **No FP64 in VideoCore VII hardware.** The QPU ISA is FP32-only. SOCP IPMs require FP64 for KKT residual convergence; software FP64 in shaders kills throughput.
2. **GPU FP32 peak ≤ CPU FP64 peak.** Pi 5 GPU ~76–96 GFLOPS FP32 (12 QPUs × 4 ALUs × 2 ops/cycle @ 800 MHz–1 GHz). Pi 5 CPU ~76.8 GFLOPS FP64 (4× Cortex-A76 with NEON dual 128-bit FMA @ 2.4 GHz). CPU is *faster* even at FP32 (~153.6 GFLOPS).
3. **Problem size 4–5 orders of magnitude below GPU break-even.** QOCO-GPU paper (arXiv:2603.29197, 2026) reports the GPU-vs-CPU crossover at ~10⁵ KKT nonzeros, on a desktop NVIDIA GPU vs desktop CPU. Our SOCP at N=200 has ~10³ KKT nonzeros on a tiny mobile GPU.

Plus dispatch overhead. Pi 5 V3DV Vulkan compute dispatch latency floor is unmeasured publicly, but mobile-GPU dispatch overhead is typically 200–500 µs/dispatch on immature driver stacks. For our 10 ms total budget at ~50 dispatches per solve naive port, dispatch overhead alone consumes the budget.

Other Pi 5 accelerator options also rejected: Hailo NPU (INT8-only, ML-only toolchain), Coral (same), eGPU via PCIe (transfer overhead kills inner loop, plus enclosure/PSU complexity).

**Sources** (full citations in agent transcript): VideoCore VII GFLOPS — RPi forum thread Dec 2024; Mesa V3DV Vulkan 1.3 conformance — LinuxToday late 2024; OpenCL via Rusticl experimental status — void-packages issue #57684 Oct 2025; QOCO-GPU crossover claim — arXiv:2603.29197; WebGPU dispatch overhead — arXiv:2604.02344; Clarabel.rs as the right CPU baseline — arXiv:2405.12762.

**Verdict: optimize the CPU path; do not pursue GPU offload.**

### Finding 2 — Tolerance reduction is the dominant CPU win

Clarabel default tolerances: `tol_gap_abs/rel = 1e-8`, `tol_feas = 1e-8`. Spec §6.2 already accepts ε_feas = 1e-3 (0.1%) post-solve verification — **defaults were 100,000× tighter than the kalico spec needs**.

Patch (one line addition to `solver.rs::solve_with_cuts`):

```rust
let settings = DefaultSettings::<f64> {
    verbose: false,
    max_iter: 1000,
    tol_gap_abs: 1e-5, tol_gap_rel: 1e-5, tol_feas: 1e-5,  // ← added
    ..Default::default()
};
```

**Median ms/solve, single-core taskset -c 3, target-cpu=native, before vs after:**

| N    | straight       | arc            | cubic                   |
|------|----------------|----------------|-------------------------|
| 20   | 3.8 → 3.2      | 12.3 → 6.6     | 7.3 → 6.5               |
| 50   | 12.2 → 9.3     | 28.3 → 22.2    | 51.8 → 31.2             |
| 100  | 30.7 → 23      | 67.2 → 47      | 87.9 → 65.2             |
| 200  | 95.5 → 65 (32%)| 186 → 121 (35%)| **1596 → 142 (11×)**    |

The cubic@N=200 catastrophic case was Clarabel's IPM hitting its 1000-iter cap at default tolerance, finishing as `AlmostSolved` after grinding 1.5 seconds. At 1e-5: 42 IPM iters, 66 ms, status `Solved`. **All previously-AlmostSolved cases are now Solved.**

Quality preserved: full `cargo test -p temporal --release` passes, including the closed-form Biagiotti-Melchiorri 7-segment ground-truth check on fixtures 1+2 (which uses 6% tolerance per `tests/prototype.rs`).

**Failed sub-experiment:** tol=1e-4 is *slower* than tol=1e-5 (cubic@N=200: 167 ms vs 142 ms). Cause: Clarabel's "reduced tolerances" (the AlmostSolved threshold) default to 5e-5 / 1e-4. When primary tol = reduced tol, the two-tier convergence logic degenerates. Sweet spot is **1e-5**.

### Finding 3 — `opt-level = "z"` on the workspace was leaving ~25% on the table

The workspace `[profile.release]` was set to `opt-level = "z"` (size-optimization), anticipatory of the MCU firmware build (build-order Steps 5+). For host benchmarks this is the wrong target.

Switching to `opt-level = 3`: cubic@N=200 went from 1596 → 1100 ms (at default tol; ~30% gain). Combined with Finding 2's tolerance patch the total stack is captured in the table above.

**Recommendation:** change workspace `[profile.release]` to `opt-level = 3`. When MCU firmware build lands (Step 5+), introduce a separate profile for it. Done in the patch accompanying this artifact.

### Finding 4 — Adaptive N is necessary AND sufficient

Cost scaling per fixture, single-core, tol=1e-5, opt=3+native:

| N    | straight | arc  | cubic | scaling exponent (cubic) |
|------|----------|------|-------|--------------------------|
| 20   | 3.2      | 6.6  | 6.5   | —                        |
| 50   | 9.3      | 22.2 | 31.2  | ~1.7                     |
| 100  | 23       | 47   | 65.2  | ~1.1                     |
| 200  | 65       | 121  | 142   | ~1.1                     |

Cost is roughly linear-to-superlinear in N. A 1mm slicer-output G1 segment with N=200 has 5 µm grid spacing — wildly over-resolved. Realistic adaptive-N policy targets ~50–200 µm grid spacing per segment, yielding N ≈ 5–20 for typical hobby slicer output.

**Step 4.5 should default N adaptively to segment arclength + curvature complexity, not fixed N=200.**

### Finding 5 — Per-segment parallelism scales 2–4× at small N, regresses at large N

Throughput in segments/sec, cubic fixture, tol=1e-5:

| N    | 1 thread | 2 threads      | 3 threads      | 4 threads      | best speedup |
|------|----------|----------------|----------------|----------------|--------------|
| 20   | 148      | 295 (1.99×)    | 433 (2.92×)    | 550 (3.72×)    | **3.72×**    |
| 50   | 32       | 63  (1.97×)    | 83  (2.59×)    | 69  (2.16×)    | 2.59× (3T)   |
| 100  | 15.3     | 28.7 (1.88×)   | 27 (1.76×)     | 16.4 (1.07×)   | 1.88× (2T)   |
| 200  | 7.0      | 10  (1.43×)    | 7   (1.0×)     | 5.2 (0.74×)    | 1.43× (2T) ⚠ |

**Why scaling collapses at large N**: the SOCP working set spills out of each Cortex-A76's per-core L2 cache (512 KB). All cores then contend for the shared L3 / memory bus. BCM2712 is the binding constraint here — not raw FLOPS, not GPU, not solver tuning.

**Implication, doubly important**: small N (≤50) gives both faster per-call cost AND near-linear parallel scaling. Adaptive N + 3-thread batch executor is the right shape for Step 4.5. Pinning a 4th thread fights Klipper's background activity on cores 0-1.

### Failed experiment — `target-cpu=native` is roughly a wash vs opt=3

Initial measurement showed `target-cpu=native` SLOWER than opt=3 alone, but that result was contaminated by elevated system load (load avg 5.7 from back-to-back benchmark runs). Re-measured with cool system + single-core pinning: roughly identical to opt=3 baseline ±5%.

LLVM's A76-specific instruction scheduling doesn't materially help for this workload. The Clarabel + faer code is already well-vectorized via faer's hand-written NEON intrinsics; LLVM autovectorization adds little. **Worth keeping target-cpu in the build flags but not transformative.**

### Failed experiment — OpenBLAS link is irrelevant

OpenBLAS 0.3.21 is pre-installed on Pi OS. Not used. Clarabel's default sparse linear-algebra backend is `faer` (Rust-native, NEON-vectorized), not BLAS. Switching Clarabel to a BLAS-src backend would require turning off `faer-sparse` feature. Not worth pursuing because:
- OpenBLAS lacks A76-tuned kernels (only up to A72/A73)
- Faer is competitive on small problems where BLAS dispatch overhead matters
- Significant build complexity

Skipped.

## Implications for Step 4.5 and Step 9

### Step 4.5 (multi-segment integration)

The (A) vs (B) trade is no longer hardware-feasibility-bound. Both are tractable on this hardware:

- **(A)** "SOCP per joining iteration" at adaptive N=20 with 4-core parallelism: ~1.8 ms amortized per re-solve. At MVP-style 1000 push/sec with ~2.5 sweeps, ~4.5 ms per push consumed compute = totally feasible.
- **(B)** "SOCP at finalize" at the same parameters: even more comfortable.

**Recommendation for Step 4.5**:
1. Default to adaptive N policy (proportional to segment arclength + curvature complexity).
2. Use 1e-5 Clarabel tolerances throughout (Finding 2 patch).
3. Use 3-thread parallel batch executor (avoid the 4-core memory-bandwidth cliff at large N; reserve a core for Klipper background activity).
4. The (A) vs (B) algorithmic choice is now purely about implementation simplicity vs. trajectory-time optimality, not hardware-feasibility. Following the throughput-non-negotiable principle: **(A)** wins.

### Step 9 (shaper-aware feedback iteration)

The cubic@N=200 1.6s catastrophe was an early warning that the SLP outer loop in pathological cases could blow up planning time. With Finding 2 in place, even the SLP-cut SOCP converges in 67 IPM iters (112 ms). Step 9's iterative shaper-aware loop will benefit equivalently — re-solves stay in the 100 ms range per call instead of seconds.

The Step-4 spec §11 deferred item ("Multi-segment SOCP across the whole window") is now genuinely worth investigating as a Step 9 enabler — it would amortize Clarabel setup across the whole window and reduce per-segment overhead. Not investigated in this artifact; flagged as follow-up.

## Recommended patches

**Three changes; the first two land in this commit.**

1. **`rust/Cargo.toml`** — change `[profile.release] opt-level = "z"` to `3`. Comment notes future MCU profile override. *(Applied in this commit.)*

2. **`rust/temporal/examples/{pi5_perf.rs,pi5_parallel.rs}`** — benchmark binaries for reproducibility. Single-call latency sweep + thread-scaling test. *(Applied in this commit.)*

3. **`rust/temporal/src/topp/solver.rs`** — `tol_gap_abs/rel = 1e-5, tol_feas = 1e-5` in the `DefaultSettings` constructor inside `solve_with_cuts`. **NOT applied in this commit** because the file has 844 lines of in-flight Step-9 work from the parallel Step-4 agent that this session should not entangle. To be applied as a follow-up commit after the in-flight Step-9 work lands.

   Patch text:
   ```rust
   let settings = DefaultSettings::<f64> {
       verbose: false,
       max_iter: 1000,
       tol_gap_abs: 1e-5, tol_gap_rel: 1e-5, tol_feas: 1e-5,
       ..Default::default()
   };
   ```

## Reproducing

On a Pi 5 (or comparable Cortex-A76 host):

```bash
ssh user@pi5
git clone <kalico-repo> kalico
cd kalico/rust
cargo build -p temporal --release --examples       # ~2 min cold

# single-call latency sweep
for fix in straight arc cubic; do
  for n in 20 50 100 200; do
    taskset -c 3 ./target/release/examples/pi5_perf $fix 50 $n 2>&1 | tail -1
  done
done

# thread-scaling test
for fix in straight arc cubic; do
  for t in 1 2 3 4; do
    ./target/release/examples/pi5_parallel $fix 400 $t 50 2>&1
  done
done
```

The benchmark binary prints per-iteration wall-clock to stdout and a summary line to stderr; the parallel binary prints amortized throughput. Apply the tolerance patch from §"Recommended patches" #3 to reproduce the Finding 2 numbers; without it you'll see the original baseline numbers (cubic@N=200 ≈ 1.6s).

## Phase-level cost decomposition (post-tolerance-patch)

Instrumented `schedule_segment` with per-stage `Instant::now()` timing on the bench (`rust/temporal/src/topp/mod.rs::schedule_segment`, Pi-only patch, not committed upstream). Median ms per stage at tol=1e-5, target-cpu=native:

| Fixture           | arclen | build | solve  | verify |
|-------------------|--------|-------|--------|--------|
| straight, N=50    | 0.05   | 0.30  | 13.0   | 0.01   |
| straight, N=200   | 0.10   | 7.2   | 57.5   | 0.01   |
| arc, N=50         | 0.05   | 0.31  | 21.8   | 0.00   |
| arc, N=200        | 0.16   | 8.7   | 112.2  | 0.01   |
| cubic, N=50       | 0.06   | 0.30  | 30.7   | 0.00   |
| cubic, N=200      | 0.16   | 8.8   | 132.6  | 0.02   |

**Solver (Clarabel) dominates: 85–95% of total at N=200, ~99% at N=50.** The next slice is `build` (constraint-matrix construction, the column-bucketed CSC-builder in `solve_with_cuts`) at 7–13% of N=200 cost. Arclength sampling and verification are noise.

The `build` cost is `O(N²)` (iterating over `bundle.a_rows` × `n_vars` to bucket sparse entries). Could be reduced to `O(nnz)` by emitting CSC directly in `constraints::build`, saving 7–13% per solve at N=200. Worth doing eventually but not high-leverage relative to the solver-dominated regime.

## Multi-segment SOCP analysis (Step-4 spec §11 deferred item)

The deferred "Cross-segment relaxation effects" item asks whether a single SOCP across the whole lookahead window would amortize Clarabel setup vs many per-segment solves. With the post-tolerance-patch numbers in hand:

- **One big N=200 solve, cubic-class geometry**: 142 ms single-thread.
- **Ten small N=20 solves, cubic-class geometry**: 10 × 6.5 ms = 65 ms single-thread.

**For Clarabel-as-black-box, multi-segment SOCP is ~2.2× slower than per-segment-at-small-N at the same total resolution**, single-threaded. With per-segment parallelism across 4 cores: 10 × N=20 ≈ 25 ms vs 1 × N=200 = 142 ms = **5.7× faster per-segment**.

Why: as currently implemented, both flavors hand Clarabel a fresh problem with no warm-start, no shared symbolic factorization, and no cross-segment KKT-block-structure exploitation. SOCP cost scales superlinearly in problem size (sparse Cholesky factorization + IPM iteration count both grow), so splitting into K small problems gives each one a tiny KKT system that fits in cache and converges in fewer IPM iterations. Plus the per-segment shape lets us trivially fan out across cores.

**Verdict (qualified after Codex review):** for the implementation regime we have — Clarabel-as-black-box, no warm-start API exposed, no purpose-built cross-segment formulation — **per-segment-with-adaptive-N + parallelism wins for Step 4.5.** We adopt that for Step 4.5 and **defer** (rather than close) the multi-segment SOCP question. A purpose-built multi-segment formulation that exploits shared symbolic factorization, block-tridiagonal KKT structure, or warm starts could change the answer in principle; investigating that is a Step 8 / Step 9 research item, especially once shaper-aware iteration creates repeated related solves on the same constraint shape.

## (A) joining-with-SOCP-per-iter feasibility math (corrected after Codex review)

**Earlier draft of this section had a real math error**, caught by an external review pass. The original framing computed "per-push compute = per-segment latency / core count" and concluded Option (A) needed "1.5–3 cores at 100% sustained at 1000 push/sec." That is wrong: Clarabel is single-threaded per solve, so spreading across N cores means doing N independent solves in parallel, not making one solve N× faster. The right metric is solver throughput (segments solved per second across all threads), not amortized latency × push rate. By that metric the original framing was 2–3.6× over the hardware's measured throughput, not 30–60% under it.

**The architectural conclusion (option A is feasible) holds, but for a different reason** — the planner is offline-batch, not push-per-second-streaming, so the relevant feasibility comparison is `total_planning_time < total_print_time` (or "wait before motion starts"), not "match motion rate."

### Corrected feasibility framing

Reframe around the actual operating model: **the planner runs offline against a buffered file, finishes ahead of motion, feeds the MCU's segment buffer.** No "1000 push/sec sustained" requirement exists; that target was a leftover from the streaming-model framing the brainstorming explicitly walked back from.

The relevant per-print metrics:

- **Total planning time** = (total segments) ÷ (aggregate solver throughput in segments/sec).
- **Aggregate solver throughput**, measured (Finding 5, cubic-worst-case at adaptive N=20, 3 threads): **~430 seg/sec.** At 4 threads: 550 seg/sec on synthetic-only, but 4th thread fights Klipper on cores 0–1 in real production (Codex's point d), so the safe number to plan against is the 3-thread one.
- **Per-print segment count**: ~200K G1-dense slicer segments in a long current-style print, or ~10–20K G5 segments under future kalico-aware slicer.

**Worked numbers (offline-batch ratio):**

| Print profile                          | Planning time at 430 seg/s | Acceptable for offline-batch? |
|----------------------------------------|----------------------------|-------------------------------|
| 200K G1, all-cubic-worst-case (synthetic) | ~7.7 min                   | Yes, against multi-hour print |
| 200K G1, realistic mix (~80% straight)    | likely 2–3 min             | Yes                           |
| 20K G5, all-cubic-worst-case              | ~46 s                      | Yes                           |
| 20K G5, realistic mix                     | likely 10–20 s             | Yes                           |

Realistic mix throughputs are extrapolations from per-fixture single-call costs; not directly measured. **Real-slicer-output benchmark is a follow-up that would tighten these.**

**Conclusion (corrected):** option (A) is feasible **as offline-batch**, with planning-time-to-print-time ratios that comfortably allow the planner to finish ahead of motion or to pre-plan with a few-minutes wait before the first move. The throughput-non-negotiable principle is satisfied in the form that actually matches the architecture (offline batch with the operator either pre-planning or planning-while-motion-builds-the-buffer), not in the streaming form (sustained motion-rate match) which we never committed to.

The (A) vs (B) decision is genuinely no longer hardware-feasibility-bound; it's about the trajectory-quality regression of (B) (per the verifier: 3–8% on ramp-bound segments) being a knowing choice the throughput-non-negotiable principle disallows.

### Caveats and follow-ups (expanded after Codex review)

1. **Adaptive-N policy must be implemented**; using fixed N=200 reverts to the catastrophic per-segment regime documented in this artifact (cubic@N=200 = 142 ms/solve at tol=1e-5; pre-tolerance-patch was 1.6 s).
2. **Per-segment parallelism uses 3 threads, not 4** — avoids the 4-core memory-bandwidth cliff at large N AND avoids contention with Klipper on cores 0–1.
3. **3-thread numbers were measured with `taskset -c 3` pinning the harness, but the parallel benchmark itself did not pin individual workers via `pthread_setaffinity_np`, and Klipper was idle (no print in motion).** Production validation should re-measure with worker-thread affinity (e.g., pin to cores 1, 2, 3) and Klipper actively serving a print on core 0. Codex flagged this; deferred to plan-execution time.
4. **Synthetic-fixture extrapolations (table above) are not real-slicer-output benchmarks.** Real slicer output (PrusaSlicer / Orca / Super output for representative test prints) is a follow-up that should land before Step 4.5 implementation completes; if real-mix throughput is materially below the extrapolations, the offline-batch ratio narrows.
5. **Joining iteration count is assumed 1–3 sweeps with 1–2 dirty segments per sweep.** Current code's SLP outer loop has `SLP_MAX_OUTER_ITERS = 50` (path-jerk) plus 30 (per-axis-jerk Step 9). Worst-case joining behavior on real multi-segment input is not yet measured — defer to a `plan_batch` benchmark on representative slicer buffers.
6. **The current upstream `solver.rs` does not yet have the tolerance patch applied** (Codex correctly flagged this). The Pi numbers in Finding 2 onward were measured against the bench checkout on the Pi with the patch applied locally; the upstream tree's settings are still default 1e-8. The patch lands as a follow-up commit after the in-flight Step-9 work commits.
7. **The Pi bench checkout was rsync'd from the local working tree at the time of measurement, which already included the in-flight Step-9 axis-jerk SLP work** (the 935-line uncommitted diff in `solver.rs`/`mod.rs`/`prototype.rs`). The benchmarks therefore exercise the Step-9-extended `schedule_segment` path, not the older committed Step-4 surface. (Codex initially worried these numbers might be stale relative to current code; they are not — but worth recording explicitly.)
8. **Inter-grid feasibility is not independently verified.** Step 4 spec §6.2 verifies velocity / acceleration / jerk / centripetal at grid points only with `ε_feas = 1e-3`. Loosening Clarabel tolerances from 1e-8 to 1e-5 changes grid-point solution magnitudes by O(1e-5), well within the verification tolerance — but inter-grid behavior is dominated by N choice and trajectory interpolation, not solver tolerance, so the relative impact of the tolerance change on inter-grid feasibility is small. Risk is real but bounded by adaptive-N policy. Recommend: add a sanity test that resamples a solved profile at 4× the solver grid density and re-checks feasibility, as a one-shot validation that the tolerance patch doesn't introduce silent inter-grid violations.

## Open follow-ups

- **Apply tolerance patch upstream** once Step-9 in-flight work has committed.
- **Multi-segment SOCP across the lookahead window** — Step-4 spec §11 deferred item, now worth investigating; would reduce per-segment overhead by amortizing Clarabel setup. Flagged for a follow-up brainstorm session.
- **Adaptive-N policy specification** — needs to be designed precisely as part of Step 4.5 spec.
- **Skip-base-SOCP heuristic** — for cases with sharply varying curvature, the base SOCP solve is wasteful (cuts will be needed anyway). Detecting this from κ(s) analysis up-front and starting with cuts could save another ~30% on cubic-class segments. Algorithm work; deferred.
- **Validate findings on real slicer output** — synthetic fixtures in this artifact are extreme cases. A typical print's mix is ~80% straight + ~15% arcs + ~5% cubics with mostly small N. Realistic per-print planning-latency estimate based on this mix: ~1–5 minutes for a multi-hour print. Confirm against real slicer output before committing to the (A) vs (B) decision.
- **Step 4.5 spec writing** — the architectural conversation from this session's brainstorm should land in a Step 4.5 spec, with this artifact cited as the throughput-evidence base.
