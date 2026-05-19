# Stepping engine redesign — implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the per-step Newton-ring stepping path with the fixed-rate-sampling + secant-slope sub-sample-timing architecture described in `docs/superpowers/specs/2026-05-19-stepping-redesign-design.md`. Both Pulse and Phase modes implemented; all unit + klipper-sim integration tests passing offline. Bench bring-up (Stages 1-7 in the spec) is a separate follow-up plan.

**Architecture:** TIM5 ISR fires at a configurable sample rate (40 kHz H7 / 20 kHz F446), evaluates a cubic Bezier per motor axis via monomial-form Horner, computes integer step deltas per sample, and queues secant-slope-interpolated sub-sample step times into a C-owned SPSC queue per axis. A per-axis Klipper SysTick-scheduled timer fires one step pulse per dispatch (mainline pattern), never early. Pulse mode toggles step pins; Phase mode writes TMC5160 XDIRECT coil currents via SPI.

**Tech Stack:** Rust (`rust/runtime`, `rust/nurbs`) for math + ISR bodies; C (`src/`) for shared-memory storage, Klipper scheduler integration, GPIO emission; Kconfig for build-time configuration; the existing klipper-sim simulator (`~/Developer/klipper-sim/`) for offline integration validation.

**Scope decisions:**
- This plan covers firmware-side rewrite + offline validation only.
- Bench bring-up (Stages 1-7 from spec) lives in a separate validation plan that runs after this plan completes successfully.
- Both Pulse and Phase mode dispatch are implemented in this plan because they share the TIM5 ISR body and splitting would duplicate boilerplate.
- TMC config-register writes (chopconf, stallguard config) are host-side responsibility per the spec and outside firmware scope.

---

## File map

**Create:**
- `src/step_queue.h` — C-side StepEntry + StepQueue type, accessors, sizeof asserts
- `src/step_queue.c` — Four `step_queues[4]` storage instances, section attributes
- `rust/runtime/src/step_queue.rs` — `#[repr(C)]` Rust mirror with size/offset asserts, push/pop wrappers
- `rust/runtime/src/monomial.rs` — Bernstein→monomial conversion, Horner position+velocity evaluator
- `rust/runtime/src/sub_sample_timing.rs` — Secant-slope step-time formula + DISPLACEMENT_THRESHOLD fallback
- `rust/runtime/src/per_axis_timer.rs` — Rust `extern "C"` consumer body for per-axis Klipper timers
- `rust/runtime/src/tick.rs` — New unified TIM5 ISR body (replaces old `runtime_tick` for stepping)
- `rust/runtime/src/phase_lut.rs` — 1024-entry sinusoid LUT for TMC5160 coil currents
- `rust/runtime/tests/monomial_eval.rs` — Unit tests for math kernel
- `rust/runtime/tests/sub_sample_timing.rs` — Unit tests for sub-sample formula
- `rust/runtime/tests/step_queue.rs` — Property tests for SPSC queue
- `rust/runtime/tests/tick_integration.rs` — End-to-end TIM5 ISR tests
- `tests/klipper_sim/stepping_redesign_test.py` — klipper-sim integration test harness

**Modify:**
- `src/runtime_tick.c` — replace `step_time_event`, `runtime_producer_event`, `init_step_time_timers`; remove SF_RESCHEDULE_FLOOR=100µs and EMPTY_POLL_CYCLES=100ms
- `src/stepper.c` — replace `command_config_runtime_stepper` with `kalico_configure_axis` (broader signature)
- `src/Kconfig` (or equivalent) — add `CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ`
- `rust/runtime/src/engine.rs` — remove `producer_step` Newton-fill loop, `compute_next_step_time` machinery, `step_rings: [StepRing; 4]`, `PRODUCER_BATCH_CAP`
- `rust/runtime/src/shared.rs` — add new fault codes + telemetry counters
- `rust/runtime/src/lib.rs` — wire up new modules
- `rust/kalico-c-api/src/runtime_ffi.rs` — add new `extern "C"` entry points

**Delete:**
- `rust/runtime/src/step_ring.rs` (replaced by `step_queue.rs`)
- `rust/runtime/src/compute_next_step_time.rs` (or wherever Newton lives) — replaced by sub_sample_timing
- All `step_time_event` / `runtime_producer_event` / per-stepper-Klipper-timer code paths in `runtime_tick.c`

---

## Tasks

### Task 1: Branch setup + Kconfig sample rate

**Files:**
- Modify: `src/Kconfig` (around the existing KALICO_RUNTIME section)
- Create: `docs/kalico-rewrite/sample-rate-kconfig.md` (brief notes)

- [ ] **Step 1: Verify branch state**

Run: `git status && git log --oneline -5`
Expected: branch `sota-motion`, clean working tree, recent commits include the spec commits ending with `9438c2c55` (the final self-review pass).

- [ ] **Step 2: Add Kconfig option for sample rate**

Find the existing `config KALICO_RUNTIME` block in `src/Kconfig` (or `src/stm32/Kconfig` / wherever runtime-related kconfig lives). Add immediately after:

```
config KALICO_MOTION_SAMPLE_RATE_HZ
    int "Motion engine sample rate in Hz (TIM5 ISR fire rate)"
    depends on KALICO_RUNTIME
    default 40000 if MACH_STM32H7
    default 20000 if MACH_STM32F4
    default 10000
    range 1000 100000
    help
      Rate at which the TIM5 ISR evaluates motion curves and emits step
      pulses. Higher rates give better velocity-extrapolation accuracy
      but cost more CPU. Defaults to 40 kHz on H7 (520 MHz) and 20 kHz
      on F4 (180 MHz); see docs/superpowers/specs/2026-05-19-stepping-
      redesign-design.md for the per-MCU step-rate ceilings these
      enable.
```

- [ ] **Step 3: Verify Kconfig syntax**

Run: `make menuconfig` (cancel without changes after navigating to the new option)
Expected: new option appears under KALICO_RUNTIME submenu with the default value matching the active MCU.

- [ ] **Step 4: Commit**

```bash
git add src/Kconfig
git commit -m "build(kconfig): add CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ for stepping redesign"
```

---

### Task 2: Bernstein-to-monomial conversion

**Files:**
- Create: `rust/runtime/src/monomial.rs`
- Create: `rust/runtime/tests/monomial_eval.rs`
- Modify: `rust/runtime/src/lib.rs`

- [ ] **Step 1: Write the failing conversion test**

Create `rust/runtime/tests/monomial_eval.rs`:

```rust
use kalico_runtime::monomial::{bernstein_to_monomial, BezierPieceMonomial};

#[test]
fn bernstein_to_monomial_constant_curve() {
    // Bernstein control points all equal → constant polynomial
    // P(t) = 5 for all t. Monomial form: c0 = 5, c1 = c2 = c3 = 0.
    let bp = [5.0f32, 5.0, 5.0, 5.0];
    let m = bernstein_to_monomial(bp);
    assert_eq!(m.coeffs[0], 5.0);
    assert_eq!(m.coeffs[1], 0.0);
    assert_eq!(m.coeffs[2], 0.0);
    assert_eq!(m.coeffs[3], 0.0);
}

#[test]
fn bernstein_to_monomial_linear_curve() {
    // Bernstein CPs [0, 1/3, 2/3, 1] = linear P(t) = t.
    // Monomial: c0=0, c1=1, c2=0, c3=0.
    let bp = [0.0f32, 1.0/3.0, 2.0/3.0, 1.0];
    let m = bernstein_to_monomial(bp);
    assert!((m.coeffs[0]).abs() < 1e-6);
    assert!((m.coeffs[1] - 1.0).abs() < 1e-6);
    assert!((m.coeffs[2]).abs() < 1e-5);
    assert!((m.coeffs[3]).abs() < 1e-5);
}

#[test]
fn bernstein_to_monomial_roundtrip_against_de_casteljau() {
    use kalico_runtime::monomial::eval_position;
    // Random-ish cubic Bezier
    let bp = [0.0f32, 0.3, 0.7, 1.0];
    let m = bernstein_to_monomial(bp);

    // Reference de Casteljau evaluator
    fn de_casteljau(bp: &[f32; 4], t: f32) -> f32 {
        let s = 1.0 - t;
        let b01 = s * bp[0] + t * bp[1];
        let b11 = s * bp[1] + t * bp[2];
        let b21 = s * bp[2] + t * bp[3];
        let b02 = s * b01 + t * b11;
        let b12 = s * b11 + t * b21;
        s * b02 + t * b12
    }

    for i in 0..=100 {
        let t = i as f32 / 100.0;
        let m_val = eval_position(&m, t);
        let dc_val = de_casteljau(&bp, t);
        assert!((m_val - dc_val).abs() < 1e-4,
            "monomial vs de Casteljau mismatch at t={}: monomial={}, dc={}",
            t, m_val, dc_val);
    }
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cargo test -p kalico-runtime --test monomial_eval 2>&1 | tail -20`
Expected: FAIL with "error[E0432]: unresolved import `kalico_runtime::monomial`".

- [ ] **Step 3: Write the monomial module**

Create `rust/runtime/src/monomial.rs`:

```rust
//! Cubic Bezier in monomial form for fast per-sample evaluation.
//!
//! Bernstein form (stored in pieces from the host) is convenient for
//! geometric reasoning but slow to evaluate. Monomial form (Horner) is
//! ~3x faster for position+velocity. We convert once per piece-load
//! and cache the result in `BezierPieceMonomial`.

/// Cubic Bezier piece in monomial form: P(t) = c0 + c1·t + c2·t² + c3·t³.
/// Velocity coefficients pre-baked: V(t) = vc0 + vc1·t + vc2·t².
#[derive(Clone, Copy, Debug)]
pub struct BezierPieceMonomial {
    pub coeffs: [f32; 4],      // c0, c1, c2, c3 for position
    pub vel_coeffs: [f32; 3],  // vc0=c1, vc1=2·c2, vc2=3·c3
    pub duration: f32,          // seconds in this piece
}

/// Convert Bernstein control points [b0, b1, b2, b3] to monomial form.
///
/// Identities for cubic Bezier:
///   c0 = b0
///   c1 = 3·(b1 - b0)
///   c2 = 3·(b2 - 2·b1 + b0)
///   c3 = b3 - 3·b2 + 3·b1 - b0
#[inline]
pub fn bernstein_to_monomial(bp: [f32; 4]) -> BezierPieceMonomial {
    let c0 = bp[0];
    let c1 = 3.0 * (bp[1] - bp[0]);
    let c2 = 3.0 * (bp[2] - 2.0 * bp[1] + bp[0]);
    let c3 = bp[3] - 3.0 * bp[2] + 3.0 * bp[1] - bp[0];

    BezierPieceMonomial {
        coeffs: [c0, c1, c2, c3],
        vel_coeffs: [c1, 2.0 * c2, 3.0 * c3],
        duration: 1.0,  // caller overrides; default unit duration
    }
}

/// Evaluate position via Horner: c0 + t·(c1 + t·(c2 + t·c3)).
/// ~10 cycles on H7 FPU (3 muladds).
#[inline]
pub fn eval_position(m: &BezierPieceMonomial, t: f32) -> f32 {
    let c = &m.coeffs;
    c[0] + t * (c[1] + t * (c[2] + t * c[3]))
}

/// Evaluate velocity (dP/dt) via Horner: vc0 + t·(vc1 + t·vc2).
/// ~7 cycles on H7 FPU (2 muladds).
#[inline]
pub fn eval_velocity(m: &BezierPieceMonomial, t: f32) -> f32 {
    let v = &m.vel_coeffs;
    v[0] + t * (v[1] + t * v[2])
}

/// Combined position+velocity for the hot path (saves one t multiply).
#[inline]
pub fn eval_position_velocity(m: &BezierPieceMonomial, t: f32) -> (f32, f32) {
    let c = &m.coeffs;
    let v = &m.vel_coeffs;
    let pos = c[0] + t * (c[1] + t * (c[2] + t * c[3]));
    let vel = v[0] + t * (v[1] + t * v[2]);
    (pos, vel)
}
```

- [ ] **Step 4: Wire the module into the crate**

Add to `rust/runtime/src/lib.rs`:

```rust
pub mod monomial;
```

(Insert in alphabetical order alongside other `pub mod` statements.)

- [ ] **Step 5: Run the test to verify it passes**

Run: `cargo test -p kalico-runtime --test monomial_eval 2>&1 | tail -10`
Expected: 3 tests passed.

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/monomial.rs rust/runtime/tests/monomial_eval.rs rust/runtime/src/lib.rs
git commit -m "feat(monomial): cubic Bezier Horner evaluator with Bernstein conversion"
```

---

### Task 3: Sub-sample step timing (secant slope)

**Files:**
- Create: `rust/runtime/src/sub_sample_timing.rs`
- Create: `rust/runtime/tests/sub_sample_timing.rs`
- Modify: `rust/runtime/src/lib.rs`

- [ ] **Step 1: Write the failing test**

Create `rust/runtime/tests/sub_sample_timing.rs`:

```rust
use kalico_runtime::sub_sample_timing::{
    compute_step_times, StepTimeInputs, StepTimingResult,
};

#[test]
fn step_times_in_sample_for_constant_velocity() {
    // Constant velocity, 4 steps within sample period 25 µs.
    // Expected: steps evenly spaced at sample_period/4 intervals.
    let inputs = StepTimeInputs {
        p_start: 0.0,
        p_end: 1.0,                       // mm (1 mm in 25 µs = 40 m/s)
        prev_step_count: 0,
        target_step_count: 4,
        microstep_distance: 0.25,         // 0.25 mm per microstep
        sample_period_sec: 25e-6,
        sample_start_cycles: 1000,
        cycles_per_second: 520_000_000.0, // H7 clock
        displacement_threshold: 0.001,
    };

    let result = compute_step_times(&inputs);
    let StepTimingResult::SecantSlope(times) = result else {
        panic!("expected SecantSlope variant, got {:?}", result);
    };
    assert_eq!(times.len(), 4);
    // Step k at t = (k+1)/4 * sample_period for constant velocity
    // (k+1 because step_pos_k = (prev + k+1) * mstep_distance)
    for (k, &cycle_abs) in times.iter().enumerate() {
        let dt_cycles = cycle_abs.wrapping_sub(1000);
        let expected_dt_cycles =
            ((k + 1) as f64 / 4.0 * 25e-6 * 520e6) as u32;
        let drift = (dt_cycles as i32 - expected_dt_cycles as i32).abs();
        assert!(drift < 10,
            "step {} drift {} cycles too large (expected {}, got {})",
            k, drift, expected_dt_cycles, dt_cycles);
    }
}

#[test]
fn step_times_within_sample_for_decelerating() {
    // Verify the secant-slope formula keeps step times within [0, period]
    // even for a deceleration profile where trapezoidal-average would
    // place the last step outside the sample.
    let inputs = StepTimeInputs {
        p_start: 0.0,
        p_end: 1.0,
        prev_step_count: 0,
        target_step_count: 4,
        microstep_distance: 0.25,
        sample_period_sec: 25e-6,
        sample_start_cycles: 5000,
        cycles_per_second: 520_000_000.0,
        displacement_threshold: 0.001,
    };

    let result = compute_step_times(&inputs);
    let StepTimingResult::SecantSlope(times) = result else {
        panic!("expected SecantSlope variant");
    };
    let period_cycles = (25e-6 * 520e6) as u32; // 13000
    for &cycle_abs in &times {
        let dt = cycle_abs.wrapping_sub(5000);
        assert!(dt <= period_cycles,
            "step time {} cycles past sample period {}", dt, period_cycles);
    }
}

#[test]
fn falls_back_to_uniform_when_displacement_too_small() {
    let inputs = StepTimeInputs {
        p_start: 0.0,
        p_end: 0.0001,                    // sub-threshold displacement
        prev_step_count: 0,
        target_step_count: 3,
        microstep_distance: 0.001,
        sample_period_sec: 25e-6,
        sample_start_cycles: 0,
        cycles_per_second: 520_000_000.0,
        displacement_threshold: 0.001,    // 1 microstep
    };

    let result = compute_step_times(&inputs);
    let StepTimingResult::Uniform(times) = result else {
        panic!("expected Uniform variant for near-zero displacement");
    };
    assert_eq!(times.len(), 3);
    // Uniform: times at (k+1) / (n+1) of sample period
    let period_cycles = (25e-6 * 520e6) as u32;
    for (k, &cycle_abs) in times.iter().enumerate() {
        let expected = (period_cycles as u64 * (k as u64 + 1) / 4) as u32;
        let drift = (cycle_abs as i32 - expected as i32).abs();
        assert!(drift < 10, "uniform step {} drift {}", k, drift);
    }
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cargo test -p kalico-runtime --test sub_sample_timing 2>&1 | tail -10`
Expected: FAIL with unresolved import.

- [ ] **Step 3: Implement the module**

Create `rust/runtime/src/sub_sample_timing.rs`:

```rust
//! Sub-sample step times via secant-slope linear interpolation.
//!
//! See docs/superpowers/specs/2026-05-19-stepping-redesign-design.md
//! "TIM5 ISR" section. Formula:
//!   t_k = (step_pos_k - P_start) · sample_period / (P_end - P_start)
//!
//! By construction t_k ∈ [0, sample_period] when step_pos_k ∈
//! [P_start, P_end], which is always the case because n_steps came
//! from rounding within that interval.

use heapless::Vec;

pub const MAX_STEPS_PER_SAMPLE: usize = 16;  // > peak 13 at 500 kHz / 40 kHz

#[derive(Clone, Copy, Debug)]
pub struct StepTimeInputs {
    pub p_start: f32,
    pub p_end: f32,
    pub prev_step_count: i32,
    pub target_step_count: i32,
    pub microstep_distance: f32,
    pub sample_period_sec: f32,
    pub sample_start_cycles: u32,
    pub cycles_per_second: f32,
    pub displacement_threshold: f32,
}

#[derive(Debug)]
pub enum StepTimingResult {
    /// Secant-slope sub-sample times (normal motion)
    SecantSlope(Vec<u32, MAX_STEPS_PER_SAMPLE>),
    /// Uniform-within-sample fallback (near-zero displacement)
    Uniform(Vec<u32, MAX_STEPS_PER_SAMPLE>),
    /// No steps this sample
    NoSteps,
}

/// Compute absolute cycle_abs values for each step this sample.
/// All u32 arithmetic uses wrapping_add (cycle counter wraps every
/// ~8.3 s on H7).
pub fn compute_step_times(inp: &StepTimeInputs) -> StepTimingResult {
    let n_steps_signed = inp.target_step_count - inp.prev_step_count;
    let n_steps_abs = n_steps_signed.unsigned_abs() as usize;
    if n_steps_abs == 0 {
        return StepTimingResult::NoSteps;
    }
    let sign: i32 = if n_steps_signed >= 0 { 1 } else { -1 };

    let displacement = inp.p_end - inp.p_start;
    let sample_period_cycles =
        (inp.sample_period_sec * inp.cycles_per_second) as u32;

    if displacement.abs() <= inp.displacement_threshold {
        // Uniform fallback
        let mut times = Vec::<u32, MAX_STEPS_PER_SAMPLE>::new();
        for k in 0..n_steps_abs {
            let dt_cycles = (sample_period_cycles as u64
                * (k as u64 + 1)
                / (n_steps_abs as u64 + 1)) as u32;
            let cycle_abs = inp.sample_start_cycles.wrapping_add(dt_cycles);
            let _ = times.push(cycle_abs);
        }
        return StepTimingResult::Uniform(times);
    }

    // Secant-slope: t_k = (step_pos_k - p_start) · sample_period / displacement
    let mut times = Vec::<u32, MAX_STEPS_PER_SAMPLE>::new();
    for k in 0..n_steps_abs {
        let step_idx = inp.prev_step_count + (k as i32 + 1) * sign;
        let step_pos_k = step_idx as f32 * inp.microstep_distance;
        let t_local_sec = (step_pos_k - inp.p_start) * inp.sample_period_sec
            / displacement;
        let dt_cycles = (t_local_sec * inp.cycles_per_second) as u32;
        let cycle_abs = inp.sample_start_cycles.wrapping_add(dt_cycles);
        let _ = times.push(cycle_abs);
    }
    StepTimingResult::SecantSlope(times)
}
```

- [ ] **Step 4: Add to lib.rs**

In `rust/runtime/src/lib.rs`, add:

```rust
pub mod sub_sample_timing;
```

- [ ] **Step 5: Run tests**

Run: `cargo test -p kalico-runtime --test sub_sample_timing 2>&1 | tail -10`
Expected: 3 tests passed.

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/sub_sample_timing.rs rust/runtime/tests/sub_sample_timing.rs rust/runtime/src/lib.rs
git commit -m "feat(timing): secant-slope sub-sample step time formula"
```

---

### Task 4: C-side SPSC step queue

**Files:**
- Create: `src/step_queue.h`
- Create: `src/step_queue.c`
- Modify: `src/Makefile` (or equivalent — add step_queue.c to KALICO_RUNTIME conditional)

- [ ] **Step 1: Write the C header**

Create `src/step_queue.h`:

```c
// SPSC step queue per motor axis. Producer = TIM5 ISR (Rust);
// consumer = per-axis Klipper timer (Rust extern "C", called from
// Klipper SysTick dispatch). Storage C-owned per architectural
// invariant B2/B3 in docs/kalico-rewrite/mcu-c-rust-boundary.md.
//
// 32-entry SPSC ring with u16 head/tail counters using wrapping
// subtraction for length. Power-of-2 depth allows mask indexing
// (& STEP_QUEUE_DEPTH_MASK) on the hot path.

#ifndef __KALICO_STEP_QUEUE_H
#define __KALICO_STEP_QUEUE_H

#include <stdint.h>
#include <stddef.h>

#define STEP_QUEUE_DEPTH       32
#define STEP_QUEUE_DEPTH_MASK  0x1F  // depth - 1; power-of-2 invariant
#define N_AXIS_STEP_QUEUES     4     // A, B, Z, E

typedef struct {
    uint32_t cycle_abs;   // lower 32 bits of DWT CYCCNT; wrap-aware compare only
    int8_t   dir;         // +1 / -1
    uint8_t  _pad[3];     // explicit padding, matches Rust #[repr(C)]
} StepEntry;

typedef struct {
    volatile uint16_t tail;
    volatile uint16_t head;
    uint8_t  _pad[4];
    StepEntry buf[STEP_QUEUE_DEPTH];
} StepQueue;

extern StepQueue step_queues[N_AXIS_STEP_QUEUES];

_Static_assert(sizeof(StepEntry) == 8, "StepEntry layout drift");
_Static_assert(sizeof(StepQueue) == 264, "StepQueue layout drift");
_Static_assert(offsetof(StepQueue, buf) == 8, "StepQueue.buf offset drift");
_Static_assert((STEP_QUEUE_DEPTH & STEP_QUEUE_DEPTH_MASK) == 0,
               "STEP_QUEUE_DEPTH must be power of 2");

#endif // __KALICO_STEP_QUEUE_H
```

- [ ] **Step 2: Write the C source with section placement**

Create `src/step_queue.c`:

```c
// See step_queue.h for the design.
//
// Storage placement:
//   H7: DTCM-mapped .bss (non-cached, eliminates cache coherency between
//       TIM5 ISR producer and SysTick consumer). Q-LINKER open question:
//       confirm the existing H7 linker script's DTCM region name; if no
//       dedicated DTCM region exists, fall back to default .bss (which
//       may live in cached AXI SRAM and require explicit cache maintenance).
//   F4: default .bss (no DTCM/cache concern).

#include "autoconf.h"
#include "step_queue.h"

#if CONFIG_MACH_STM32H7
// TODO Q-LINKER: confirm section name. Default placement uses .bss
// in DTCM if the linker script maps it so. If a dedicated section is
// needed, add it via __attribute__((section(".dtcm_bss"))).
StepQueue step_queues[N_AXIS_STEP_QUEUES];
#else
StepQueue step_queues[N_AXIS_STEP_QUEUES];
#endif
```

- [ ] **Step 3: Add to the build**

Find the existing `Makefile` (or the relevant `obj-$(CONFIG_KALICO_RUNTIME)` list). Add `step_queue.o` to the list of objects compiled under `CONFIG_KALICO_RUNTIME`. Search for an existing entry like `obj-$(CONFIG_KALICO_RUNTIME) += runtime_tick.o` and add `step_queue.o` on the same line or nearby.

- [ ] **Step 4: Verify it compiles**

Run: `make clean && make -j$(nproc) 2>&1 | tail -20`
Expected: clean build for the active MCU; sizeof asserts pass at compile time.

- [ ] **Step 5: Commit**

```bash
git add src/step_queue.h src/step_queue.c src/Makefile
git commit -m "feat(step_queue): C-side SPSC step queue storage and layout"
```

---

### Task 5: Rust mirror of StepQueue

**Files:**
- Create: `rust/runtime/src/step_queue.rs`
- Create: `rust/runtime/tests/step_queue.rs`
- Modify: `rust/runtime/src/lib.rs`

- [ ] **Step 1: Write the SPSC property test**

Create `rust/runtime/tests/step_queue.rs`:

```rust
use kalico_runtime::step_queue::{StepEntry, StepQueue, push, pop, len};
use std::cell::UnsafeCell;

#[test]
fn fifo_order_under_random_push_pop() {
    let q = UnsafeCell::new(StepQueue::new());
    let q_ptr = q.get();

    // Push 30 entries, then pop them back; verify FIFO order
    for i in 0..30u32 {
        unsafe {
            assert!(push(q_ptr, StepEntry { cycle_abs: i, dir: 1, _pad: [0; 3] }).is_ok());
        }
    }
    for i in 0..30u32 {
        let entry = unsafe { pop(q_ptr).expect("nonempty") };
        assert_eq!(entry.cycle_abs, i);
    }
    assert_eq!(unsafe { len(q_ptr) }, 0);
}

#[test]
fn overflow_detected_at_full_capacity() {
    let q = UnsafeCell::new(StepQueue::new());
    let q_ptr = q.get();
    for i in 0..32u32 {
        unsafe {
            assert!(push(q_ptr, StepEntry { cycle_abs: i, dir: 1, _pad: [0; 3] }).is_ok());
        }
    }
    // 33rd push must fail
    unsafe {
        assert!(push(q_ptr, StepEntry { cycle_abs: 32, dir: 1, _pad: [0; 3] }).is_err());
    }
}

#[test]
fn wraparound_u16_counters_correct() {
    let q = UnsafeCell::new(StepQueue::new());
    let q_ptr = q.get();
    // Push 25, pop 25, push 25 — counters wrap u16 cleanly via wrapping subtract
    for round in 0..3 {
        for i in 0..25u32 {
            unsafe { push(q_ptr, StepEntry { cycle_abs: round * 100 + i, dir: 1, _pad: [0;3] }).unwrap() };
        }
        for i in 0..25u32 {
            let e = unsafe { pop(q_ptr).unwrap() };
            assert_eq!(e.cycle_abs, round * 100 + i);
        }
    }
}

#[test]
fn empty_pop_returns_none() {
    let q = UnsafeCell::new(StepQueue::new());
    assert!(unsafe { pop(q.get()) }.is_none());
}
```

- [ ] **Step 2: Run the test to see it fail**

Run: `cargo test -p kalico-runtime --test step_queue 2>&1 | tail -10`
Expected: FAIL with unresolved import.

- [ ] **Step 3: Implement the Rust mirror**

Create `rust/runtime/src/step_queue.rs`:

```rust
//! Rust mirror of the C-side StepQueue (src/step_queue.h).
//! Storage is owned by C; this module provides typed accessors.

use core::sync::atomic::{fence, Ordering};
use core::ptr;

pub const STEP_QUEUE_DEPTH: usize = 32;
pub const STEP_QUEUE_DEPTH_MASK: u16 = (STEP_QUEUE_DEPTH as u16) - 1;
pub const N_AXIS_STEP_QUEUES: usize = 4;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct StepEntry {
    pub cycle_abs: u32,
    pub dir: i8,
    pub _pad: [u8; 3],
}

#[repr(C)]
pub struct StepQueue {
    pub tail: u16,
    pub head: u16,
    _pad: [u8; 4],
    pub buf: [StepEntry; STEP_QUEUE_DEPTH],
}

impl StepQueue {
    #[cfg(any(test, feature = "host"))]
    pub fn new() -> Self {
        StepQueue {
            tail: 0,
            head: 0,
            _pad: [0; 4],
            buf: [StepEntry { cycle_abs: 0, dir: 0, _pad: [0; 3] }; STEP_QUEUE_DEPTH],
        }
    }
}

const _: () = {
    assert!(core::mem::size_of::<StepEntry>() == 8);
    assert!(core::mem::size_of::<StepQueue>() == 264);
    assert!(STEP_QUEUE_DEPTH.is_power_of_two());
};

// On MCU build, storage is the C-declared `step_queues` symbol.
#[cfg(not(any(test, feature = "host")))]
extern "C" {
    pub static step_queues: core::cell::UnsafeCell<[StepQueue; N_AXIS_STEP_QUEUES]>;
}

#[derive(Debug, PartialEq, Eq)]
pub struct StepQueueFull;

/// Push an entry. Producer-side; only the TIM5 ISR calls this.
///
/// # Safety
/// Caller must ensure exclusive access from one producer context.
pub unsafe fn push(q: *mut StepQueue, entry: StepEntry) -> Result<(), StepQueueFull> {
    let tail = ptr::read_volatile(&(*q).tail);
    let head = ptr::read_volatile(&(*q).head);
    if tail.wrapping_sub(head) >= STEP_QUEUE_DEPTH as u16 {
        return Err(StepQueueFull);
    }
    let slot = (tail & STEP_QUEUE_DEPTH_MASK) as usize;
    ptr::write_volatile(&mut (*q).buf[slot], entry);
    fence(Ordering::Release);
    ptr::write_volatile(&mut (*q).tail, tail.wrapping_add(1));
    Ok(())
}

/// Pop the head entry. Consumer-side.
///
/// # Safety
/// Caller must ensure exclusive access from one consumer context.
pub unsafe fn pop(q: *mut StepQueue) -> Option<StepEntry> {
    let tail = ptr::read_volatile(&(*q).tail);
    let head = ptr::read_volatile(&(*q).head);
    if tail == head {
        return None;
    }
    fence(Ordering::Acquire);
    let slot = (head & STEP_QUEUE_DEPTH_MASK) as usize;
    let entry = ptr::read_volatile(&(*q).buf[slot]);
    fence(Ordering::Release);
    ptr::write_volatile(&mut (*q).head, head.wrapping_add(1));
    Some(entry)
}

/// Peek the head without popping.
///
/// # Safety
/// Caller must ensure no concurrent pop.
pub unsafe fn peek(q: *mut StepQueue) -> Option<StepEntry> {
    let tail = ptr::read_volatile(&(*q).tail);
    let head = ptr::read_volatile(&(*q).head);
    if tail == head {
        return None;
    }
    fence(Ordering::Acquire);
    let slot = (head & STEP_QUEUE_DEPTH_MASK) as usize;
    Some(ptr::read_volatile(&(*q).buf[slot]))
}

/// Current length (wrapping subtract).
///
/// # Safety
/// Inherently racy if called concurrently; safe for observation.
pub unsafe fn len(q: *mut StepQueue) -> u16 {
    let tail = ptr::read_volatile(&(*q).tail);
    let head = ptr::read_volatile(&(*q).head);
    tail.wrapping_sub(head)
}
```

- [ ] **Step 4: Wire up lib.rs**

Add to `rust/runtime/src/lib.rs`:

```rust
pub mod step_queue;
```

- [ ] **Step 5: Run tests**

Run: `cargo test -p kalico-runtime --test step_queue 2>&1 | tail -10`
Expected: 4 tests passed.

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/step_queue.rs rust/runtime/tests/step_queue.rs rust/runtime/src/lib.rs
git commit -m "feat(step_queue): Rust mirror with push/pop/peek and SPSC ordering"
```

---

### Task 6: State shapes (StepperRef, AxisConfig, TickCaches)

**Files:**
- Create: `rust/runtime/src/stepping_state.rs`
- Modify: `rust/runtime/src/lib.rs`
- Modify: `rust/runtime/src/shared.rs` (add new fault codes)

- [ ] **Step 1: Write the state definitions**

Create `rust/runtime/src/stepping_state.rs`:

```rust
//! State shapes for the unified stepping architecture.
//! See docs/superpowers/specs/2026-05-19-stepping-redesign-design.md
//! "State" section for the design rationale.

use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8};
use heapless::Vec;
use crate::monomial::BezierPieceMonomial;

pub const N_AXES: usize = 4;
pub const MAX_STEPPERS_PER_AXIS: usize = 4;

#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum StepMode {
    Pulse = 0,
    Phase = 1,
}

pub struct StepperRef {
    pub step_pin: u32,                       // GPIO pin handle (opaque to Rust)
    pub dir_pin: u32,
    pub dir_invert: bool,

    pub position_count: AtomicI32,           // signed; checked_add fault on overflow

    // Phase mode only:
    pub tmc_cs: Option<u32>,
    pub last_coil_A: AtomicI16,
    pub last_coil_B: AtomicI16,
    pub phase_offset_microsteps: AtomicI32,  // current offset
    pub phase_offset_target: AtomicI32,      // target offset (ramps toward)
    pub last_phase_target: AtomicI32,        // axis_pos + offset at last sample
}

pub struct AxisConfig {
    pub mode: AtomicU8,                                  // StepMode::Pulse=0, Phase=1
    pub steppers: Vec<StepperRef, MAX_STEPPERS_PER_AXIS>,
    pub piece: Option<BezierPieceMonomial>,
    pub piece_start_time_cycles: u64,
    pub last_step_count: i32,
    pub microstep_distance: f32,
    pub extrusion_per_xy_mm: f32,                        // axis E only; 0 for others
}

/// Per-sample state held across TIM5 ISR fires.
pub struct TickCaches {
    pub p_prev: [f32; N_AXES],
    pub v_prev: [f32; N_AXES],
    pub v_xy_prev: f32,
    pub ds_xy_segment: f32,

    /// Computed in phase 2 of each ISR fire (after A and B); consumed in
    /// phase 3 (E). Lives only one ISR pass.
    pub v_xy_this: f32,
    pub vdot_xy_accelerating: bool,
}

impl TickCaches {
    pub const fn new() -> Self {
        Self {
            p_prev: [0.0; N_AXES],
            v_prev: [0.0; N_AXES],
            v_xy_prev: 0.0,
            ds_xy_segment: 0.0,
            v_xy_this: 0.0,
            vdot_xy_accelerating: false,
        }
    }
}
```

- [ ] **Step 2: Add fault codes to shared.rs**

In `rust/runtime/src/shared.rs`, find the existing `FaultCode` enum (or wherever fault codes live) and add:

```rust
StepQueueOverflow,        // 16-bit upper word carries axis_idx
SpiQueueOverflow,         // upper = bus_idx
MathNonFinite,            // upper = axis_idx
PieceAdvanceUnderflow,    // upper = axis_idx
SampleRateMisconfigured,
PositionCountOverflow,    // upper = stepper_idx
JogParametersInvalid,
StepRateExceedsMcuCeiling, // upper = axis_idx
```

Also add the new telemetry counters:

```rust
pub queue_high_water: [AtomicU32; 4],
pub queue_overflow_count: [AtomicU32; 4],
pub spi_saturated_samples: AtomicU32,
pub sample_isr_peak_cycles: AtomicU32,
pub per_axis_consumer_peak_cycles: [AtomicU32; 4],
```

- [ ] **Step 3: Add to lib.rs**

```rust
pub mod stepping_state;
```

- [ ] **Step 4: Verify it compiles**

Run: `cargo build -p kalico-runtime --target thumbv7em-none-eabihf 2>&1 | tail -10`
Expected: clean build.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/stepping_state.rs rust/runtime/src/shared.rs rust/runtime/src/lib.rs
git commit -m "feat(stepping_state): types + fault codes + telemetry for redesign"
```

---

### Task 7: TIM5 ISR — dispatch_axis (Pulse + Phase branches)

**Files:**
- Create: `rust/runtime/src/tick.rs` (Phase 1 — just dispatch_axis)
- Create: `rust/runtime/src/phase_lut.rs`
- Modify: `rust/runtime/src/lib.rs`

- [ ] **Step 1: Generate the 1024-entry phase LUT**

Create `rust/runtime/src/phase_lut.rs`:

```rust
//! 1024-entry sinusoid LUT for TMC5160 phase stepping.
//! See spec "Phase mode" section. Index range [0, 1024) maps to one
//! electrical cycle (= 4 full steps).

pub const PHASE_LUT_SIZE: usize = 1024;
pub const COIL_AMPLITUDE: i16 = 248;  // TMC5160 XDIRECT range: ±248

/// Precomputed (coil_A, coil_B) pairs. coil_A = cos, coil_B = sin.
/// Generated at compile time via const-fn arithmetic — no float math
/// needed at runtime.
pub static PHASE_LUT: [(i16, i16); PHASE_LUT_SIZE] = {
    let mut lut = [(0i16, 0i16); PHASE_LUT_SIZE];
    let mut i = 0;
    while i < PHASE_LUT_SIZE {
        // angle = 2π · i / 1024; cos/sin via small-angle table + identities
        // For now, use libm at build time via build.rs alternative would be
        // cleaner; in-place: defer to a build.rs-generated file.
        // (Placeholder — see Step 2 for the build.rs generation.)
        lut[i] = (0, 0);
        i += 1;
    }
    lut
};
```

- [ ] **Step 2: Switch to build.rs-generated LUT**

Replace `rust/runtime/src/phase_lut.rs` with:

```rust
//! 1024-entry sinusoid LUT for TMC5160 phase stepping.

pub const PHASE_LUT_SIZE: usize = 1024;
pub const COIL_AMPLITUDE: i16 = 248;

// Generated by build.rs from sin/cos at compile time.
include!(concat!(env!("OUT_DIR"), "/phase_lut.rs"));
```

Modify (or create) `rust/runtime/build.rs`:

```rust
use std::env;
use std::fs;
use std::path::Path;

fn main() {
    let out_dir = env::var("OUT_DIR").unwrap();
    let dest = Path::new(&out_dir).join("phase_lut.rs");

    let mut s = String::new();
    s.push_str("pub static PHASE_LUT: [(i16, i16); PHASE_LUT_SIZE] = [\n");
    for i in 0..1024 {
        let angle = 2.0 * std::f64::consts::PI * (i as f64) / 1024.0;
        let cos = (angle.cos() * COIL_AMPLITUDE as f64).round() as i16;
        let sin = (angle.sin() * COIL_AMPLITUDE as f64).round() as i16;
        s.push_str(&format!("    ({}, {}),\n", cos, sin));
    }
    s.push_str("];\n");

    fs::write(dest, s).unwrap();
}
```

- [ ] **Step 3: Verify LUT compiles**

Run: `cargo build -p kalico-runtime 2>&1 | tail -10`
Expected: clean build. Lookup `PHASE_LUT[0]` should be `(248, 0)`; `PHASE_LUT[256]` should be `(0, 248)`.

- [ ] **Step 4: Write the dispatch_axis function**

Create `rust/runtime/src/tick.rs`:

```rust
//! TIM5 ISR body — the unified motion evaluator.
//! See docs/superpowers/specs/2026-05-19-stepping-redesign-design.md
//! "TIM5 ISR — the unified evaluator" section.

use core::sync::atomic::Ordering;
use crate::stepping_state::{AxisConfig, StepMode, N_AXES};
use crate::sub_sample_timing::{compute_step_times, StepTimeInputs, StepTimingResult};
use crate::step_queue::{push as queue_push, StepEntry};
use crate::phase_lut::{PHASE_LUT, COIL_AMPLITUDE};

pub const DISPLACEMENT_THRESHOLD_MM: f32 = 1e-4;  // ~1 microstep at typical configs

/// Per-axis dispatch invoked from the main TIM5 loop.
/// Updates step queue (Pulse) or SPI queue (Phase), and per-stepper
/// position counters.
pub fn dispatch_axis(
    axis_idx: usize,
    axis: &mut AxisConfig,
    p_end: f32,
    v_end: f32,
    p_sample_start: f32,
    sample_period_sec: f32,
    sample_start_cycles: u32,
    cycles_per_second: f32,
) {
    match axis.mode.load(Ordering::Acquire) {
        m if m == StepMode::Pulse as u8 => dispatch_pulse(
            axis_idx, axis, p_end, p_sample_start,
            sample_period_sec, sample_start_cycles, cycles_per_second,
        ),
        m if m == StepMode::Phase as u8 => dispatch_phase(
            axis_idx, axis, p_end,
        ),
        _ => { /* invalid mode; should be impossible — caught at config time */ }
    }
}

fn dispatch_pulse(
    axis_idx: usize,
    axis: &mut AxisConfig,
    p_end: f32,
    p_sample_start: f32,
    sample_period_sec: f32,
    sample_start_cycles: u32,
    cycles_per_second: f32,
) {
    let prev_step_count = axis.last_step_count;
    let target_step_count = (p_end / axis.microstep_distance).round() as i32;
    let n_steps = target_step_count - prev_step_count;
    axis.last_step_count = target_step_count;

    if n_steps == 0 {
        return;
    }

    let inputs = StepTimeInputs {
        p_start: p_sample_start,
        p_end,
        prev_step_count,
        target_step_count,
        microstep_distance: axis.microstep_distance,
        sample_period_sec,
        sample_start_cycles,
        cycles_per_second,
        displacement_threshold: DISPLACEMENT_THRESHOLD_MM,
    };

    let result = compute_step_times(&inputs);
    let (times, dir_sign) = match result {
        StepTimingResult::SecantSlope(t) | StepTimingResult::Uniform(t) => {
            (t, if n_steps > 0 { 1i8 } else { -1i8 })
        }
        StepTimingResult::NoSteps => return,
    };

    let queue_ptr = unsafe {
        crate::step_queue::step_queues.get().cast::<crate::step_queue::StepQueue>()
            .add(axis_idx)
    };
    for cycle_abs in times {
        let entry = StepEntry { cycle_abs, dir: dir_sign, _pad: [0; 3] };
        if unsafe { queue_push(queue_ptr, entry) }.is_err() {
            // Fault: queue overflow. Set the shared fault flag.
            // (Fault propagation details in Task 13.)
            crate::shared::set_fault_step_queue_overflow(axis_idx);
            return;
        }
    }

    // Update per-stepper position counters
    for stepper in axis.steppers.iter() {
        match stepper.position_count.load(Ordering::Acquire).checked_add(n_steps) {
            Some(new) => stepper.position_count.store(new, Ordering::Release),
            None => crate::shared::set_fault_position_count_overflow(axis_idx),
        }
    }
}

fn dispatch_phase(axis_idx: usize, axis: &mut AxisConfig, p_end: f32) {
    let target_microsteps_axis =
        (p_end / axis.microstep_distance).round() as i32;
    axis.last_step_count = target_microsteps_axis;

    for stepper in axis.steppers.iter() {
        let offset = stepper.phase_offset_microsteps.load(Ordering::Acquire);
        let target_stepper = target_microsteps_axis + offset;
        let prev_stepper = stepper.last_phase_target.load(Ordering::Acquire);
        let delta_stepper = target_stepper - prev_stepper;
        stepper.last_phase_target.store(target_stepper, Ordering::Release);

        let phase = (target_stepper as u32 & 0x3FF) as usize;
        let (coil_a, coil_b) = PHASE_LUT[phase];

        stepper.last_coil_A.store(coil_a, Ordering::Release);
        stepper.last_coil_B.store(coil_b, Ordering::Release);

        // SPI dispatch (queue push) — covered in Task 14.
        // For now, the per-stepper coil values are cached; the actual
        // XDIRECT SPI write is the responsibility of the SPI queue
        // (separate task) which reads `last_coil_A/B`.

        // position_count update
        match stepper.position_count.load(Ordering::Acquire).checked_add(delta_stepper) {
            Some(new) => stepper.position_count.store(new, Ordering::Release),
            None => crate::shared::set_fault_position_count_overflow(axis_idx),
        }
    }
}
```

- [ ] **Step 5: Add stub fault helpers to shared.rs**

In `rust/runtime/src/shared.rs`, add (or extend existing fault helpers):

```rust
pub fn set_fault_step_queue_overflow(axis_idx: usize) {
    // Encode: FaultCode::StepQueueOverflow as u16 | (axis_idx << 16)
    let code = FaultCode::StepQueueOverflow as u32 | ((axis_idx as u32 & 0xFF) << 16);
    SHARED.fault.store(code, Ordering::Release);
}

pub fn set_fault_position_count_overflow(stepper_idx: usize) {
    let code = FaultCode::PositionCountOverflow as u32 | ((stepper_idx as u32 & 0xFF) << 16);
    SHARED.fault.store(code, Ordering::Release);
}

pub fn set_fault_math_non_finite(axis_idx: usize) {
    let code = FaultCode::MathNonFinite as u32 | ((axis_idx as u32 & 0xFF) << 16);
    SHARED.fault.store(code, Ordering::Release);
}
```

(Adjust names if the existing shared.rs convention differs.)

- [ ] **Step 6: Add modules to lib.rs**

```rust
pub mod phase_lut;
pub mod tick;
```

- [ ] **Step 7: Verify compiles**

Run: `cargo build -p kalico-runtime --target thumbv7em-none-eabihf 2>&1 | tail -10`
Expected: clean build.

- [ ] **Step 8: Commit**

```bash
git add rust/runtime/src/tick.rs rust/runtime/src/phase_lut.rs rust/runtime/build.rs rust/runtime/src/shared.rs rust/runtime/src/lib.rs
git commit -m "feat(tick): dispatch_axis with Pulse + Phase branches, 1024-entry LUT"
```

---

### Task 8: TIM5 ISR — full per-sample evaluator (phases 1-5)

**Files:**
- Modify: `rust/runtime/src/tick.rs`

- [ ] **Step 1: Implement `runtime_tick_sample`**

Add to `rust/runtime/src/tick.rs`:

```rust
use crate::stepping_state::{TickCaches, N_AXES};

const AXIS_A: usize = 0;
const AXIS_B: usize = 1;
const AXIS_Z: usize = 2;
const AXIS_E: usize = 3;

pub struct TickContext<'a> {
    pub axes: &'a mut [AxisConfig; N_AXES],
    pub caches: &'a mut TickCaches,
    pub sample_period_sec: f32,
    pub sample_period_cycles: u32,
    pub cycles_per_second: f32,
    pub k_xy: f32,                          // 1.0 cart, 1/sqrt(2) CoreXY
    pub advance_accel: f32,                 // PA coefficient (s)
    pub advance_decel: f32,
    pub now_cycles: u32,                    // sample_start_cycles
    pub t_sample_end_global: f32,           // wall-time in seconds
}

/// Main TIM5 ISR body. Phases 1-5 per the spec.
pub fn runtime_tick_sample(ctx: &mut TickContext) {
    let mut p_end_axis = [0.0f32; N_AXES];
    let mut v_end_axis = [0.0f32; N_AXES];

    // Phase 1: evaluate motion axes A, B, Z
    for &axis_idx in &[AXIS_A, AXIS_B, AXIS_Z] {
        let axis = &mut ctx.axes[axis_idx];
        let p_sample_start = ctx.caches.p_prev[axis_idx];

        // (1) Piece advancement (skipped here — handled by piece manager)
        let piece = match axis.piece {
            Some(p) => p,
            None => {
                p_end_axis[axis_idx] = p_sample_start;
                v_end_axis[axis_idx] = 0.0;
                continue;
            }
        };
        let t_local = ctx.t_sample_end_global
            - (axis.piece_start_time_cycles as f32 / ctx.cycles_per_second);

        // (2) Polynomial eval
        let (p_end, v_end) =
            crate::monomial::eval_position_velocity(&piece, t_local);
        if !p_end.is_finite() || !v_end.is_finite() {
            crate::shared::set_fault_math_non_finite(axis_idx);
            continue;
        }

        // (3) Endstop sample
        // External hook: kalico_endstop_tick_step_time(handle, now_cycles)
        // call deferred to the C-side ISR wrapper.

        p_end_axis[axis_idx] = p_end;
        v_end_axis[axis_idx] = v_end;

        // (4) Per-axis dispatch
        dispatch_axis(
            axis_idx, axis, p_end, v_end, p_sample_start,
            ctx.sample_period_sec, ctx.now_cycles, ctx.cycles_per_second,
        );
    }

    // Phase 2: XY-derived quantities (Cartesian arc length, accel sign)
    let xy_active = ctx.axes[AXIS_A].piece.is_some() || ctx.axes[AXIS_B].piece.is_some();
    if xy_active {
        let v_motor_sq = v_end_axis[AXIS_A].powi(2) + v_end_axis[AXIS_B].powi(2);
        let v_xy_this = v_motor_sq.sqrt() * ctx.k_xy;
        ctx.caches.vdot_xy_accelerating = v_xy_this >= ctx.caches.v_xy_prev;
        ctx.caches.ds_xy_segment += v_xy_this * ctx.sample_period_sec;
        ctx.caches.v_xy_prev = v_xy_this;
        ctx.caches.v_xy_this = v_xy_this;
    } else {
        ctx.caches.v_xy_this = 0.0;
        ctx.caches.vdot_xy_accelerating = false;
    }

    // Phase 3: evaluate E with PA
    {
        let axis = &mut ctx.axes[AXIS_E];
        let p_sample_start = ctx.caches.p_prev[AXIS_E];
        if let Some(piece) = axis.piece {
            let t_local = ctx.t_sample_end_global
                - (axis.piece_start_time_cycles as f32 / ctx.cycles_per_second);
            let (p_end_intrinsic, v_end) =
                crate::monomial::eval_position_velocity(&piece, t_local);

            if !p_end_intrinsic.is_finite() || !v_end.is_finite() {
                crate::shared::set_fault_math_non_finite(AXIS_E);
            } else {
                let pa_k = if ctx.caches.vdot_xy_accelerating {
                    ctx.advance_accel
                } else {
                    ctx.advance_decel
                };
                let p_end = p_end_intrinsic
                    + axis.extrusion_per_xy_mm * ctx.caches.ds_xy_segment
                    + pa_k * axis.extrusion_per_xy_mm * ctx.caches.v_xy_this;

                p_end_axis[AXIS_E] = p_end;
                v_end_axis[AXIS_E] = v_end;

                dispatch_axis(
                    AXIS_E, axis, p_end, v_end, p_sample_start,
                    ctx.sample_period_sec, ctx.now_cycles, ctx.cycles_per_second,
                );
            }
        } else {
            p_end_axis[AXIS_E] = p_sample_start;
            v_end_axis[AXIS_E] = 0.0;
        }
    }

    // Phase 4: per-sample bookkeeping
    for i in 0..N_AXES {
        ctx.caches.p_prev[i] = p_end_axis[i];
        ctx.caches.v_prev[i] = v_end_axis[i];
    }

    // Phase 5: segment retirement check (TODO Task 9 — piece advancement logic)
}
```

- [ ] **Step 2: Add a basic integration test**

Create `rust/runtime/tests/tick_integration.rs`:

```rust
use kalico_runtime::tick::{runtime_tick_sample, TickContext};
use kalico_runtime::stepping_state::{AxisConfig, StepMode, TickCaches, StepperRef};
use kalico_runtime::monomial::{bernstein_to_monomial, BezierPieceMonomial};
use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8, Ordering};
use heapless::Vec;

fn make_stepper() -> StepperRef {
    StepperRef {
        step_pin: 0,
        dir_pin: 0,
        dir_invert: false,
        position_count: AtomicI32::new(0),
        tmc_cs: None,
        last_coil_A: AtomicI16::new(0),
        last_coil_B: AtomicI16::new(0),
        phase_offset_microsteps: AtomicI32::new(0),
        phase_offset_target: AtomicI32::new(0),
        last_phase_target: AtomicI32::new(0),
    }
}

#[test]
fn constant_velocity_produces_expected_step_count() {
    // 1 mm move at constant velocity over 25 µs sample.
    // With 0.25 mm/microstep, expect 4 steps.
    let mut piece = bernstein_to_monomial([0.0, 0.333, 0.667, 1.0]);
    piece.duration = 25e-6;
    let mut steppers_a = Vec::new();
    let _ = steppers_a.push(make_stepper());
    let mut axes = [
        AxisConfig {
            mode: AtomicU8::new(StepMode::Pulse as u8),
            steppers: steppers_a,
            piece: Some(piece),
            piece_start_time_cycles: 0,
            last_step_count: 0,
            microstep_distance: 0.25,
            extrusion_per_xy_mm: 0.0,
        },
        // ... B, Z, E configured as idle
        AxisConfig {
            mode: AtomicU8::new(StepMode::Pulse as u8),
            steppers: Vec::new(),
            piece: None,
            piece_start_time_cycles: 0,
            last_step_count: 0,
            microstep_distance: 0.25,
            extrusion_per_xy_mm: 0.0,
        },
        AxisConfig {
            mode: AtomicU8::new(StepMode::Pulse as u8),
            steppers: Vec::new(),
            piece: None,
            piece_start_time_cycles: 0,
            last_step_count: 0,
            microstep_distance: 0.25,
            extrusion_per_xy_mm: 0.0,
        },
        AxisConfig {
            mode: AtomicU8::new(StepMode::Pulse as u8),
            steppers: Vec::new(),
            piece: None,
            piece_start_time_cycles: 0,
            last_step_count: 0,
            microstep_distance: 0.25,
            extrusion_per_xy_mm: 0.0,
        },
    ];
    let mut caches = TickCaches::new();

    let mut ctx = TickContext {
        axes: &mut axes,
        caches: &mut caches,
        sample_period_sec: 25e-6,
        sample_period_cycles: 13000,
        cycles_per_second: 520e6,
        k_xy: 1.0,
        advance_accel: 0.0,
        advance_decel: 0.0,
        now_cycles: 0,
        t_sample_end_global: 25e-6,
    };
    runtime_tick_sample(&mut ctx);

    // Axis A should have last_step_count == 4 (rounded position 1.0 / 0.25)
    assert_eq!(ctx.axes[0].last_step_count, 4);
    // Stepper position_count tracks lockstep
    assert_eq!(ctx.axes[0].steppers[0].position_count.load(Ordering::Acquire), 4);
}
```

- [ ] **Step 3: Run integration test**

Run: `cargo test -p kalico-runtime --test tick_integration 2>&1 | tail -10`
Expected: 1 test passed.

- [ ] **Step 4: Commit**

```bash
git add rust/runtime/src/tick.rs rust/runtime/tests/tick_integration.rs
git commit -m "feat(tick): TIM5 ISR phases 1-4 (motion eval, XY arc, E+PA, bookkeeping)"
```

---

### Task 9: Piece advancement + segment retirement

**Files:**
- Modify: `rust/runtime/src/tick.rs`
- Modify: `rust/runtime/src/stepping_state.rs` (add piece queue)

- [ ] **Step 1: Add a piece-advancement helper**

In `rust/runtime/src/tick.rs`, before `runtime_tick_sample`, add:

```rust
/// Advance the axis's active piece if the sample time has moved past
/// the current piece's duration. Returns true if a piece advance
/// happened (caller may use this to decide on segment retirement).
fn advance_piece_if_needed(
    axis: &mut AxisConfig,
    t_sample_end_global: f32,
    cycles_per_second: f32,
) -> bool {
    let mut advanced = false;
    let mut iters = 0;
    loop {
        let Some(piece) = axis.piece else { break; };
        let piece_start_sec =
            axis.piece_start_time_cycles as f32 / cycles_per_second;
        let t_local = t_sample_end_global - piece_start_sec;
        if t_local <= piece.duration {
            break;
        }
        // Advance: the next piece (if any) starts where this one ended.
        let leftover = t_local - piece.duration;
        axis.piece_start_time_cycles +=
            (piece.duration * cycles_per_second) as u64;
        // Fetch next piece from the segment's piece list.
        // (Wire this to AxisConfig::next_piece() — to be added; for now
        // mark axis idle so test harness can verify the advancement
        // event fired.)
        axis.piece = None;
        advanced = true;
        iters += 1;
        if iters > 4 {
            crate::shared::set_fault_piece_advance_underflow(axis.steppers.iter().count());
            break;
        }
        let _ = leftover;
    }
    advanced
}
```

- [ ] **Step 2: Wire piece-advance into the main ISR**

In `runtime_tick_sample`, replace the simple piece-extract block in Phase 1's loop with a call to `advance_piece_if_needed` before the polynomial eval. Same for Phase 3 (E).

For each axis:

```rust
// Inside Phase 1 loop, immediately after `let axis = &mut ctx.axes[axis_idx];`:
advance_piece_if_needed(axis, ctx.t_sample_end_global, ctx.cycles_per_second);
```

- [ ] **Step 3: Add segment retirement logic (Phase 5)**

In `runtime_tick_sample`, replace the existing `// Phase 5: segment retirement check (TODO ...)` comment with:

```rust
// Phase 5: segment retirement check.
// All axes that participate in the active segment must have advanced
// past the segment's last piece. We don't carry a segment manifest
// at the engine level (the spec defers that to the producer/host);
// this hook just publishes the retirement counter for the host.
//
// For now, if all axes have piece == None and the previous sample
// did have any active piece, treat as retirement event.
let any_active = ctx.axes.iter().any(|a| a.piece.is_some());
if !any_active && ctx.caches.ds_xy_segment > 0.0 {
    // Publish: increment retired counter, reset segment caches.
    crate::shared::increment_retired_segment_count();
    ctx.caches.ds_xy_segment = 0.0;
}
```

- [ ] **Step 4: Add fault helper**

In `rust/runtime/src/shared.rs`:

```rust
pub fn set_fault_piece_advance_underflow(axis_idx: usize) {
    let code = FaultCode::PieceAdvanceUnderflow as u32 | ((axis_idx as u32 & 0xFF) << 16);
    SHARED.fault.store(code, Ordering::Release);
}

pub fn increment_retired_segment_count() {
    SHARED.retired_through_segment_id.fetch_add(1, Ordering::Release);
}
```

- [ ] **Step 5: Verify compiles**

Run: `cargo build -p kalico-runtime --target thumbv7em-none-eabihf 2>&1 | tail -10`
Expected: clean build.

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/tick.rs rust/runtime/src/shared.rs
git commit -m "feat(tick): piece advancement + segment retirement (Phase 5)"
```

---

### Task 10: Per-axis Klipper timer consumer (Rust extern "C")

**Files:**
- Create: `rust/runtime/src/per_axis_timer.rs`
- Modify: `rust/runtime/src/lib.rs`
- Modify: `src/runtime_tick.c` (wire C-side `struct timer` to Rust callback)

- [ ] **Step 1: Write the Rust consumer body**

Create `rust/runtime/src/per_axis_timer.rs`:

```rust
//! Per-axis Klipper SysTick consumer. Mainline pattern: fire one entry
//! per dispatch when its cycle_abs has arrived; never early.
//!
//! Body called from C-side struct timer.func via extern "C".

use crate::step_queue::{StepQueue, pop as queue_pop, peek as queue_peek, step_queues};
use crate::shared::{SHARED, get_dispatcher_floor_cycles, get_sample_period_cycles};

extern "C" {
    /// C-side helper: returns current u32 cycle counter.
    fn timer_read_time() -> u32;
    /// C-side helper: signed-delta comparison.
    /// Returns 1 if a is before b in cycle-counter time, else 0.
    fn timer_is_before(a: u32, b: u32) -> u8;
    /// C-side GPIO emission for an axis's bound steppers.
    fn runtime_emit_step_pulses(axis_idx: u8, n_steps: i32);
}

#[repr(u8)]
pub enum SfReschedule { Reschedule = 1, Done = 0 }

/// Rust body for the per-axis struct timer.func pointer.
/// Returns the next waketime (u32 cycle absolute) via the C-side
/// struct timer.waketime field, written by the caller wrapper.
#[no_mangle]
pub extern "C" fn kalico_per_axis_step_event(axis_idx: u8) -> u32 {
    let now = unsafe { timer_read_time() };
    let queue_ptr = unsafe {
        step_queues.get().cast::<StepQueue>().add(axis_idx as usize)
    };

    // Pop ONE entry if its cycle_abs has arrived.
    if let Some(entry) = unsafe { queue_peek(queue_ptr) } {
        // entry.cycle_abs <= now <=> !timer_is_before(now, entry.cycle_abs)
        if unsafe { timer_is_before(now, entry.cycle_abs) } == 0 {
            // Pop and emit.
            let _ = unsafe { queue_pop(queue_ptr) };
            unsafe { runtime_emit_step_pulses(axis_idx, entry.dir as i32) };
        }
    }

    // Determine next waketime.
    let floor_cycles = get_dispatcher_floor_cycles();
    let floor_time = now.wrapping_add(floor_cycles);
    let sample_period = get_sample_period_cycles();
    let next_sample = now.wrapping_add(sample_period);

    match unsafe { queue_peek(queue_ptr) } {
        Some(next) => {
            // max(next.cycle_abs, floor_time), wrap-aware.
            // timer_is_before(next.cycle_abs, floor_time) ⇒ next is earlier
            if unsafe { timer_is_before(next.cycle_abs, floor_time) } != 0 {
                floor_time
            } else {
                next.cycle_abs
            }
        }
        None => next_sample,
    }
}
```

- [ ] **Step 2: Add helper getters to shared.rs**

```rust
// In rust/runtime/src/shared.rs

pub fn get_dispatcher_floor_cycles() -> u32 {
    SHARED.dispatcher_floor_cycles.load(Ordering::Acquire)
}
pub fn get_sample_period_cycles() -> u32 {
    SHARED.sample_period_cycles.load(Ordering::Acquire)
}
```

Add to `SharedRuntime` struct:
```rust
pub dispatcher_floor_cycles: AtomicU32,
pub sample_period_cycles: AtomicU32,
```

- [ ] **Step 3: Wire C-side timer to Rust callback**

In `src/runtime_tick.c`, find `init_step_time_timers()` (or replace if removed). Replace with new per-axis timer setup:

```c
#include "step_queue.h"

extern uint32_t kalico_per_axis_step_event(uint8_t axis_idx);

// One struct timer per motor axis (A=0, B=1, Z=2, E=3).
static struct timer per_axis_timers[4];
static uint8_t per_axis_timer_idx[4] = {0, 1, 2, 3};

// Wrapper: extracts axis_idx from container_of-style storage and
// invokes the Rust callback. Updates the timer's waketime from the
// Rust return value.
static uint_fast8_t
per_axis_timer_event_wrapper_0(struct timer *t) {
    t->waketime = kalico_per_axis_step_event(0);
    return SF_RESCHEDULE;
}
// ... similar for 1, 2, 3
static uint_fast8_t (*const per_axis_handlers[4])(struct timer *) = {
    per_axis_timer_event_wrapper_0,
    per_axis_timer_event_wrapper_1,
    per_axis_timer_event_wrapper_2,
    per_axis_timer_event_wrapper_3,
};

void
init_per_axis_step_timers(void) {
    if (!runtime_handle) return;
    uint32_t now = timer_read_time();
    for (int i = 0; i < 4; i++) {
        per_axis_timers[i].func = per_axis_handlers[i];
        per_axis_timers[i].waketime = now + timer_from_us(1000);  // 1 ms initial poll
        sched_add_timer(&per_axis_timers[i]);
    }
}
```

(Concrete wrapper variants 1, 2, 3 follow the same pattern — copy-paste with the axis index changed.)

- [ ] **Step 4: Add per_axis_timer to lib.rs**

```rust
pub mod per_axis_timer;
```

- [ ] **Step 5: Verify build**

Run: `cargo build -p kalico-runtime --target thumbv7em-none-eabihf 2>&1 | tail -10 && make -j$(nproc) 2>&1 | tail -10`
Expected: clean build, link succeeds with `kalico_per_axis_step_event` resolved.

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/per_axis_timer.rs rust/runtime/src/shared.rs rust/runtime/src/lib.rs src/runtime_tick.c
git commit -m "feat(per_axis_timer): mainline-pattern Rust consumer for step queue"
```

---

### Task 11: New command handlers (configure_axis, configure_kinematics, configure_pressure_advance)

**Files:**
- Modify: `src/stepper.c` (replace `config_runtime_stepper` with `kalico_configure_axis`)
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` (new extern "C" entry points)
- Modify: `rust/runtime/src/engine.rs` (or new module — handler bodies)

- [ ] **Step 1: Define the C command signatures**

In `src/stepper.c` (or a new `src/stepping_commands.c`), add the Klipper command declarations:

```c
DECL_COMMAND(command_kalico_configure_axis,
             "kalico_configure_axis axis_idx=%c mode=%c microstep_distance=%u"
             " extrusion_per_xy_mm=%u stepper_count=%c");

DECL_COMMAND(command_kalico_configure_kinematics,
             "kalico_configure_kinematics k_xy=%u");

DECL_COMMAND(command_kalico_configure_pressure_advance,
             "kalico_configure_pressure_advance advance_accel=%u advance_decel=%u");
```

Implement handlers that decode args, call `kalico_runtime_configure_axis` / `_configure_kinematics` / `_configure_pressure_advance` in Rust via FFI.

- [ ] **Step 2: Add Rust FFI entry points**

In `rust/kalico-c-api/src/runtime_ffi.rs`, add:

```rust
#[no_mangle]
pub extern "C" fn kalico_runtime_configure_axis(
    handle: *mut KalicoRuntime,
    axis_idx: u8,
    mode: u8,
    microstep_distance_f32_bits: u32,
    extrusion_per_xy_mm_f32_bits: u32,
    stepper_count: u8,
) -> i32 {
    // Decode f32 from u32 wire format
    let mstep_dist = f32::from_bits(microstep_distance_f32_bits);
    let extrusion = f32::from_bits(extrusion_per_xy_mm_f32_bits);
    let mode_enum = match mode { 0 => StepMode::Pulse, 1 => StepMode::Phase, _ => return -1 };
    // ... configure the engine's axis state
    let rt = unsafe { &mut *handle };
    rt.configure_axis(axis_idx, mode_enum, mstep_dist, extrusion, stepper_count)
}

#[no_mangle]
pub extern "C" fn kalico_runtime_configure_kinematics(
    handle: *mut KalicoRuntime,
    k_xy_f32_bits: u32,
) -> i32 {
    let k = f32::from_bits(k_xy_f32_bits);
    let rt = unsafe { &mut *handle };
    rt.configure_kinematics(k)
}

#[no_mangle]
pub extern "C" fn kalico_runtime_configure_pressure_advance(
    handle: *mut KalicoRuntime,
    advance_accel_f32_bits: u32,
    advance_decel_f32_bits: u32,
) -> i32 {
    let aa = f32::from_bits(advance_accel_f32_bits);
    let ad = f32::from_bits(advance_decel_f32_bits);
    let rt = unsafe { &mut *handle };
    rt.configure_pressure_advance(aa, ad)
}
```

- [ ] **Step 3: Implement handlers in the engine**

In `rust/runtime/src/engine.rs` (or wherever `KalicoRuntime` is defined):

```rust
impl KalicoRuntime {
    pub fn configure_axis(&mut self, axis_idx: u8, mode: StepMode,
                          microstep_distance: f32, extrusion_per_xy_mm: f32,
                          stepper_count: u8) -> i32 {
        if axis_idx as usize >= N_AXES { return -1; }
        if !microstep_distance.is_finite() || microstep_distance <= 0.0 { return -1; }
        let axis = &mut self.axes[axis_idx as usize];
        axis.mode.store(mode as u8, Ordering::Release);
        axis.microstep_distance = microstep_distance;
        axis.extrusion_per_xy_mm = extrusion_per_xy_mm;
        axis.steppers.clear();
        // Steppers are bound in a separate command (or this command's
        // payload — split if needed for ABI clarity).
        let _ = stepper_count;
        0  // KALICO_OK
    }

    pub fn configure_kinematics(&mut self, k_xy: f32) -> i32 {
        if !k_xy.is_finite() || k_xy <= 0.0 { return -1; }
        self.k_xy = k_xy;
        0
    }

    pub fn configure_pressure_advance(&mut self, advance_accel: f32, advance_decel: f32) -> i32 {
        self.advance_accel = advance_accel;
        self.advance_decel = advance_decel;
        0
    }
}
```

(Add `k_xy`, `advance_accel`, `advance_decel` fields to `KalicoRuntime` struct.)

- [ ] **Step 4: Build and verify**

Run: `make clean && make -j$(nproc) 2>&1 | tail -10`
Expected: clean build with new commands compiled in.

- [ ] **Step 5: Commit**

```bash
git add src/stepper.c rust/kalico-c-api/src/runtime_ffi.rs rust/runtime/src/engine.rs
git commit -m "feat(commands): kalico_configure_axis/kinematics/pressure_advance handlers"
```

---

### Task 12: set_stepper_offset + set_axis_mode commands

**Files:**
- Modify: `src/stepper.c`
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs`
- Modify: `rust/runtime/src/engine.rs`

- [ ] **Step 1: Add command declarations**

```c
DECL_COMMAND(command_kalico_set_axis_mode,
             "kalico_set_axis_mode axis_idx=%c mode=%c");

DECL_COMMAND(command_kalico_set_stepper_offset,
             "kalico_set_stepper_offset stepper_idx=%c delta_microsteps=%i"
             " max_microsteps_per_sample=%hu");
```

- [ ] **Step 2: Rust handlers**

```rust
#[no_mangle]
pub extern "C" fn kalico_runtime_set_axis_mode(
    handle: *mut KalicoRuntime,
    axis_idx: u8,
    new_mode: u8,
) -> i32 {
    let rt = unsafe { &mut *handle };
    rt.set_axis_mode(axis_idx, new_mode)
}

#[no_mangle]
pub extern "C" fn kalico_runtime_set_stepper_offset(
    handle: *mut KalicoRuntime,
    stepper_idx: u8,
    delta_microsteps: i32,
    max_microsteps_per_sample: u16,
) -> i32 {
    let rt = unsafe { &mut *handle };
    rt.set_stepper_offset(stepper_idx, delta_microsteps, max_microsteps_per_sample)
}
```

- [ ] **Step 3: Engine `set_axis_mode` with full sequence**

```rust
impl KalicoRuntime {
    pub fn set_axis_mode(&mut self, axis_idx: u8, new_mode: u8) -> i32 {
        const ERR_MOTION_IN_PROGRESS: i32 = -2;
        const KALICO_OK: i32 = 0;

        if axis_idx as usize >= N_AXES { return -1; }

        // Step 1: idle-only verification
        if self.any_segment_active() {
            return ERR_MOTION_IN_PROGRESS;
        }

        let axis = &mut self.axes[axis_idx as usize];
        let target = match new_mode { 0 => StepMode::Pulse, 1 => StepMode::Phase, _ => return -1 };

        // Step 2: flush step queue
        unsafe {
            let q = crate::step_queue::step_queues.get()
                .cast::<crate::step_queue::StepQueue>()
                .add(axis_idx as usize);
            (*q).head = 0;
            (*q).tail = 0;
        }
        // Step 3: flush SPI queue (Task 14)
        // crate::spi_queue::flush_axis(axis_idx);

        // Step 4: counter resync
        match target {
            StepMode::Phase => {
                for stepper in axis.steppers.iter() {
                    let offset = stepper.phase_offset_microsteps.load(Ordering::Acquire);
                    stepper.last_phase_target.store(axis.last_step_count + offset, Ordering::Release);
                }
            }
            StepMode::Pulse => {
                // last_step_count already in sync (Phase samples maintain it)
            }
        }

        // Step 5: store new mode
        axis.mode.store(target as u8, Ordering::Release);
        KALICO_OK
    }

    pub fn set_stepper_offset(&mut self, stepper_idx: u8, delta: i32, max_per_sample: u16) -> i32 {
        const KALICO_OK: i32 = 0;
        if delta == 0 { return KALICO_OK; }
        if max_per_sample == 0 || max_per_sample > 256 {
            // Bounds: rate limit must be sane
            crate::shared::set_fault_jog_parameters_invalid();
            return -1;
        }
        // Find the stepper by global index
        let stepper = self.find_stepper(stepper_idx)?;
        let new_target = stepper.phase_offset_target.load(Ordering::Acquire) + delta;
        stepper.phase_offset_target.store(new_target, Ordering::Release);
        // Ramping happens in TIM5 ISR: phase_offset_microsteps moves toward
        // phase_offset_target by at most max_per_sample per sample.
        // (Ramp implementation in Task 13.)
        KALICO_OK
    }
}
```

- [ ] **Step 4: Build and commit**

```bash
make clean && make -j$(nproc) 2>&1 | tail -10
git add src/stepper.c rust/kalico-c-api/src/runtime_ffi.rs rust/runtime/src/engine.rs
git commit -m "feat(commands): set_axis_mode (with resync) and set_stepper_offset"
```

---

### Task 13: Phase-offset ramping in TIM5 ISR

**Files:**
- Modify: `rust/runtime/src/tick.rs`

- [ ] **Step 1: Add ramp helper**

In `rust/runtime/src/tick.rs`, before `dispatch_phase`:

```rust
/// Phase-mode offset ramp: bring phase_offset_microsteps toward
/// phase_offset_target by at most `max_per_sample` per call.
fn ramp_phase_offset(stepper: &StepperRef, max_per_sample: i32) {
    let current = stepper.phase_offset_microsteps.load(Ordering::Acquire);
    let target = stepper.phase_offset_target.load(Ordering::Acquire);
    if current == target { return; }
    let delta = target - current;
    let step = if delta.abs() <= max_per_sample {
        delta
    } else if delta > 0 {
        max_per_sample
    } else {
        -max_per_sample
    };
    stepper.phase_offset_microsteps.store(current + step, Ordering::Release);
}
```

- [ ] **Step 2: Call ramp in dispatch_phase before reading offset**

In `dispatch_phase`, immediately inside the per-stepper loop (before the offset read):

```rust
ramp_phase_offset(stepper, MAX_PHASE_OFFSET_RAMP_PER_SAMPLE);
```

Where `MAX_PHASE_OFFSET_RAMP_PER_SAMPLE: i32 = 8` (configurable via shared state if needed).

- [ ] **Step 3: Commit**

```bash
git add rust/runtime/src/tick.rs
git commit -m "feat(tick): phase_offset ramping toward target at max_per_sample rate"
```

---

### Task 14: SPI write queue for Phase mode

**Files:**
- Create: `src/spi_queue.h`, `src/spi_queue.c`
- Create: `rust/runtime/src/spi_queue.rs`
- Modify: `rust/runtime/src/lib.rs`
- Modify: `rust/runtime/src/tick.rs` (push coil writes from dispatch_phase)

- [ ] **Step 1: C-side queue definition**

`src/spi_queue.h`:

```c
#ifndef __KALICO_SPI_QUEUE_H
#define __KALICO_SPI_QUEUE_H

#include <stdint.h>

#define SPI_QUEUE_DEPTH       16
#define SPI_QUEUE_DEPTH_MASK  0x0F
#define N_SPI_BUSES           3   // Octopus Pro has SPI1 + others

typedef struct {
    uint32_t cs_pin;       // GPIO handle for chip-select
    uint8_t  reg;          // TMC register address (XDIRECT = 0x2D)
    uint8_t  _pad[3];
    int32_t  value;        // packed (coil_A << 16) | (coil_B & 0xFFFF)
} SpiWrite;

typedef struct {
    volatile uint16_t tail;
    volatile uint16_t head;
    uint8_t _pad[4];
    SpiWrite buf[SPI_QUEUE_DEPTH];
} SpiQueue;

extern SpiQueue spi_queues[N_SPI_BUSES];

_Static_assert(sizeof(SpiWrite) == 12, "SpiWrite layout drift");
_Static_assert(sizeof(SpiQueue) == 200, "SpiQueue layout drift");

#endif
```

- [ ] **Step 2: C-side storage**

`src/spi_queue.c`:

```c
#include "spi_queue.h"
SpiQueue spi_queues[N_SPI_BUSES];
```

- [ ] **Step 3: Rust mirror**

`rust/runtime/src/spi_queue.rs`: follow the StepQueue pattern (push/pop/peek/len). Reuse the same SPSC ordering.

- [ ] **Step 4: Wire dispatch_phase to push SPI writes**

In `rust/runtime/src/tick.rs` `dispatch_phase`, replace the placeholder comment "SPI dispatch (queue push) — covered in Task 14" with an actual push:

```rust
if let Some(cs) = stepper.tmc_cs {
    // Pack coil_a (high 16 bits, signed) and coil_b (low 16 bits, signed)
    let packed = ((coil_a as u32) << 16) | (coil_b as u32 & 0xFFFF);
    let bus_idx = (cs >> 16) as usize;  // top byte encodes bus assignment
    // ... or per-stepper config carries bus_idx separately
    let entry = crate::spi_queue::SpiWrite {
        cs_pin: cs,
        reg: 0x2D,  // XDIRECT
        _pad: [0; 3],
        value: packed as i32,
    };
    if unsafe { crate::spi_queue::push(bus_idx, entry) }.is_err() {
        crate::shared::set_fault_spi_queue_overflow(bus_idx);
    }
}
```

- [ ] **Step 5: Foreground SPI drain task**

Add to `src/runtime_tick.c` a periodic foreground task that polls the SPI queue and dispatches writes through Klipper's existing `spidev` / `bus.c` plumbing. Schedule it from `runtime_drain` or as its own struct timer firing at 1-10 kHz.

- [ ] **Step 6: Verify build and commit**

```bash
make clean && make -j$(nproc) 2>&1 | tail -10
git add src/spi_queue.{h,c} rust/runtime/src/spi_queue.rs rust/runtime/src/tick.rs rust/runtime/src/lib.rs src/runtime_tick.c
git commit -m "feat(spi_queue): per-bus SPI write queue for Phase mode XDIRECT"
```

---

### Task 15: klipper-sim integration test

**Files:**
- Create: `tests/klipper_sim/stepping_redesign_test.py`
- Create: `tests/klipper_sim/run_stepping_redesign.sh`

- [ ] **Step 1: Write the test harness**

Create `tests/klipper_sim/stepping_redesign_test.py`:

```python
"""
klipper-sim integration test for the redesigned stepping engine.

Per the spec, step times from our engine should match mainline klipper-sim's
within the local-linear-extrapolation tolerance: < 500 ns per step at typical
accel (the dispatcher jitter under the mainline-style one-pulse-per-fire
consumer).
"""
import sys, os, subprocess

KLIPPER_SIM_DIR = os.path.expanduser("~/Developer/klipper-sim/")
THIS_FORK = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

TEST_GCODE = """
G28 X Y
G1 X10 F600
G1 X0 F600
G1 X50 Y50 F12000
G1 X0 Y0 F12000
"""

def run_sim(klipper_root, label):
    # Invokes klipper-sim with the specified Klipper checkout and feeds
    # TEST_GCODE; returns list of (axis, step_time_us) tuples.
    # Implementation specific to klipper-sim's actual CLI — see
    # ~/Developer/klipper-sim/README for invocation.
    raise NotImplementedError("Wire to actual klipper-sim CLI")

def main():
    mainline = run_sim("/path/to/mainline/klipper", "mainline")
    ours = run_sim(THIS_FORK, "redesign")

    # Per-step time comparison
    assert len(mainline) == len(ours), f"step count mismatch: {len(mainline)} vs {len(ours)}"
    max_drift = 0
    for (m, o) in zip(mainline, ours):
        assert m[0] == o[0], f"axis mismatch: {m} vs {o}"
        drift_ns = abs(m[1] - o[1]) * 1000  # convert µs to ns
        max_drift = max(max_drift, drift_ns)
    assert max_drift < 500, f"max step-time drift {max_drift} ns exceeds 500 ns threshold"
    print(f"PASS: max drift {max_drift:.1f} ns")

if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Write the run script**

Create `tests/klipper_sim/run_stepping_redesign.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
exec python3 "$(dirname "$0")/stepping_redesign_test.py"
```

`chmod +x` the file.

- [ ] **Step 3: Document the test infrastructure dependency**

The klipper-sim integration test depends on the user's `~/Developer/klipper-sim/` checkout. Note this in the test header. The test should fail loudly with a clear error if klipper-sim isn't found.

- [ ] **Step 4: Commit**

```bash
git add tests/klipper_sim/
git commit -m "test(klipper_sim): integration harness for stepping redesign vs mainline"
```

---

### Task 16: Delete the old paths

**Files:**
- Delete: `rust/runtime/src/step_ring.rs`
- Delete: code in `runtime_tick.c` for `step_time_event`, `runtime_producer_event`, old `init_step_time_timers`
- Modify: `rust/runtime/src/engine.rs` (delete `producer_step` Newton-fill loop, `compute_next_step_time`, `PRODUCER_BATCH_CAP`)
- Modify: `rust/runtime/src/lib.rs` (drop `pub mod step_ring`)

- [ ] **Step 1: Delete step_ring.rs**

```bash
git rm rust/runtime/src/step_ring.rs
```

- [ ] **Step 2: Strip step_ring import from lib.rs**

Remove `pub mod step_ring;` from `rust/runtime/src/lib.rs`.

- [ ] **Step 3: Delete the Newton solver code in engine.rs**

Search for `producer_step` in `rust/runtime/src/engine.rs`. Delete the entire `producer_step` method body and any helpers used only by it (`compute_next_step_time`, `solve_monotone_cubic_root`, the per-piece Newton loop).

Search for `PRODUCER_BATCH_CAP` and remove its definition and all references.

- [ ] **Step 4: Strip old timer logic in runtime_tick.c**

In `src/runtime_tick.c`, delete:
- `step_time_event` function
- `runtime_producer_event` function
- The old `init_step_time_timers` body (replace with the new per-axis-timer init)
- `SF_RESCHEDULE_FLOOR` and `EMPTY_POLL_CYCLES` defines
- `STEP_RING_LOW_WATER` define
- `arm_producer_timer_*` helpers if unused after the above

- [ ] **Step 5: Verify build**

```bash
make clean && make -j$(nproc) 2>&1 | tail -20
cargo build -p kalico-runtime --target thumbv7em-none-eabihf 2>&1 | tail -10
```

Expected: clean builds; no leftover references to deleted symbols.

- [ ] **Step 6: Commit**

```bash
git commit -am "refactor(stepping): delete old Newton-ring + producer-timer + step_time_event"
```

---

### Task 17: TIM5 ISR C-side entry point + Kconfig wiring

**Files:**
- Modify: `src/runtime_tick.c`
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs`
- Modify: `rust/runtime/src/engine.rs`

- [ ] **Step 1: Define the new tick entry point**

In `rust/kalico-c-api/src/runtime_ffi.rs`:

```rust
#[no_mangle]
pub extern "C" fn kalico_runtime_tick_sample(handle: *mut KalicoRuntime) {
    let rt = unsafe { &mut *handle };
    rt.tick_sample();
}
```

- [ ] **Step 2: Implement engine-side tick**

In `rust/runtime/src/engine.rs`:

```rust
impl KalicoRuntime {
    pub fn tick_sample(&mut self) {
        let now = unsafe { timer_read_time() };
        let t_global = now as f32 / self.cycles_per_second;
        let mut ctx = crate::tick::TickContext {
            axes: &mut self.axes,
            caches: &mut self.tick_caches,
            sample_period_sec: self.sample_period_sec,
            sample_period_cycles: self.sample_period_cycles,
            cycles_per_second: self.cycles_per_second,
            k_xy: self.k_xy,
            advance_accel: self.advance_accel,
            advance_decel: self.advance_decel,
            now_cycles: now,
            t_sample_end_global: t_global,
        };
        crate::tick::runtime_tick_sample(&mut ctx);
    }
}

extern "C" {
    fn timer_read_time() -> u32;
}
```

- [ ] **Step 3: C-side TIM5 ISR**

In `src/runtime_tick.c`, install the TIM5 ISR per the existing modulated-path pattern:

```c
extern void kalico_runtime_tick_sample(struct KalicoRuntime *);

void TIM5_IRQHandler(void) {
    TIM5->SR = ~TIM_SR_UIF;
    if (runtime_handle)
        kalico_runtime_tick_sample(runtime_handle);
}
```

Configure TIM5's ARR / prescaler from `CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ` in the TIM5 init function (where the existing modulated path sets it up). Use:

```c
uint32_t arr = CONFIG_CLOCK_FREQ / CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ - 1;
TIM5->ARR = arr;
TIM5->PSC = 0;
TIM5->DIER = TIM_DIER_UIE;
TIM5->CR1 = TIM_CR1_CEN;
```

- [ ] **Step 4: Build and verify**

```bash
make clean && make -j$(nproc) 2>&1 | tail -10
```

Expected: clean build; firmware ready to flash.

- [ ] **Step 5: Commit**

```bash
git add src/runtime_tick.c rust/kalico-c-api/src/runtime_ffi.rs rust/runtime/src/engine.rs
git commit -m "feat(tim5): wire ISR to Rust tick_sample, set rate from Kconfig"
```

---

### Task 18: Final cross-check + open items resolution

- [ ] **Step 1: Run all tests**

```bash
cargo test -p kalico-runtime 2>&1 | tail -20
make clean && make -j$(nproc) 2>&1 | tail -10
```

Expected: all unit + property tests pass, F4 and H7 builds both compile.

- [ ] **Step 2: Resolve Q-LINKER**

Inspect `src/stm32/stm32h7.ld` (or wherever the H7 linker script lives). Confirm the DTCM section is named for `.bss` placement. Update `src/step_queue.c`'s section attribute to match the actual section name. Document the resolution in `docs/superpowers/specs/2026-05-19-stepping-redesign-design.md` Q-LINKER entry.

- [ ] **Step 3: Verify spec coverage**

Walk through each open question (Q1-Q6 in the spec) and confirm each is either resolved in the implementation or explicitly deferred to a follow-up plan with a rationale.

- [ ] **Step 4: Commit the resolution**

```bash
git add src/step_queue.c docs/superpowers/specs/2026-05-19-stepping-redesign-design.md
git commit -m "spec(stepping): resolve Q-LINKER (DTCM section name confirmed)"
```

---

## Bench bring-up

Stages 1-7 of the bench bring-up (per the spec) are out of scope for this plan. They'll be a separate **validation plan** that takes the flashed firmware through:

- Stage 1: Boot + idle telemetry
- Stage 2: Single-stepper jog correctness
- Stage 3: Pure-X G1 (CoreXY: both A and B step together)
- Stage 3b: Pure-Y G1 (CoreXY: A and B opposite)
- Stage 4: Multi-motor stress (diagonal at high feedrate)
- Stage 5: Phase mode bring-up (SPI logic-analyzer validation)
- Stage 6: Sensorless homing via mode switch
- Stage 7: Long-print soak

That plan will be authored when this plan's offline tests pass.
