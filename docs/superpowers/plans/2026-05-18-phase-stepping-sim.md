# Phase-stepping Renode-sim XDIRECT framing — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the firmware emit a correct, framing-validated stream of TMC5160 `XDIRECT` SPI writes at 40 kHz under the Renode H7 simulator, with 3-way agreement (firmware trace ring ↔ Renode TMC peripheral capture ↔ Python ground-truth model).

**Architecture:** Output-stage swap inside the existing `runtime_modulated_tick` (TIM5 ISR), gated per-motor by a 33-byte extended `configure_axes` blob carrying per-stepper SPI config. A new `PhaseDirectModulator` path computes `mscount` from position, looks up an identity sinusoid LUT, and writes `XDIRECT` via a thin C SPI helper while preserving `stepper_counts` for endstop/homing observability. Renode platform is extended with a SPI3 peripheral (only SPI4 is modeled by default) and a custom `TMC5160` C# peripheral attaches to it with CS-edge framing.

**Tech Stack:** Rust (`rust/runtime/`, `rust/kalico-c-api/`), C (`src/stm32/`), Renode 1.16.1 + C# peripheral, Python 3 (host test driver), Klipper sim build with `CONFIG_KALICO_SIM=y CONFIG_KALICO_PHASE_STEPPING=y`.

**Spec:** [`docs/superpowers/specs/2026-05-18-phase-stepping-sim-design.md`](../specs/2026-05-18-phase-stepping-sim-design.md)

---

## Task 1: Phase LUT module — identity sinusoid table via build.rs

**Files:**
- Create: `rust/runtime/build.rs`
- Create: `rust/runtime/src/phase_lut.rs`
- Modify: `rust/runtime/src/lib.rs` (add `pub mod phase_lut;`)
- Modify: `rust/runtime/Cargo.toml` (add `build = "build.rs"` if not present)
- Test: `rust/runtime/tests/phase_lut.rs`

The LUT must be a compile-time constant in flash; `f32::sin/cos` aren't `const`, so generate the table via `build.rs` and include it. Direction parameter is ignored for the identity LUT (forward and reverse sinusoids are equal) but the signature is preserved for the future calibration-LUT replacement.

- [ ] **Step 1.1: Write the failing test** — `rust/runtime/tests/phase_lut.rs`:

```rust
//! Identity sinusoid LUT contract: amplitude anchors and symmetry.

use runtime::phase_lut::{self, CURRENT_AMPLITUDE, MOTOR_PERIOD};

#[test]
fn lut_quarter_cycle_anchors() {
    // angle 0   -> (0, +A)        sin(0)=0, cos(0)=1
    let (i_a, i_b) = phase_lut::lookup(0, 1);
    assert_eq!(i_a, 0);
    assert_eq!(i_b, CURRENT_AMPLITUDE);

    // angle pi/2 (mscount = MOTOR_PERIOD/4) -> (+A, 0)
    let (i_a, i_b) = phase_lut::lookup((MOTOR_PERIOD / 4) as u16, 1);
    assert_eq!(i_a, CURRENT_AMPLITUDE);
    assert!(i_b.abs() <= 1, "cos(pi/2) ~ 0, got {}", i_b);

    // angle pi (mscount = MOTOR_PERIOD/2) -> (0, -A)
    let (i_a, i_b) = phase_lut::lookup((MOTOR_PERIOD / 2) as u16, 1);
    assert!(i_a.abs() <= 1, "sin(pi) ~ 0, got {}", i_a);
    assert_eq!(i_b, -CURRENT_AMPLITUDE);

    // angle 3pi/2 (mscount = 3*MOTOR_PERIOD/4) -> (-A, 0)
    let (i_a, i_b) = phase_lut::lookup((3 * MOTOR_PERIOD / 4) as u16, 1);
    assert_eq!(i_a, -CURRENT_AMPLITUDE);
    assert!(i_b.abs() <= 1, "cos(3pi/2) ~ 0, got {}", i_b);
}

#[test]
fn lut_direction_ignored_for_identity() {
    // For the identity sinusoid, forward and reverse must produce the same
    // currents. Calibration LUTs (silicon follow-up) introduce asymmetry.
    for &m in &[0u16, 137, 511, 768, 1023] {
        assert_eq!(phase_lut::lookup(m, 1), phase_lut::lookup(m, -1));
        assert_eq!(phase_lut::lookup(m, 1), phase_lut::lookup(m, 0));
    }
}

#[test]
fn lut_amplitude_bounded() {
    for m in 0u16..MOTOR_PERIOD as u16 {
        let (i_a, i_b) = phase_lut::lookup(m, 1);
        assert!(i_a.abs() <= CURRENT_AMPLITUDE);
        assert!(i_b.abs() <= CURRENT_AMPLITUDE);
    }
}

#[test]
fn lut_mscount_wraps() {
    // Caller mistakenly passes mscount >= MOTOR_PERIOD; lookup must wrap.
    assert_eq!(
        phase_lut::lookup(MOTOR_PERIOD as u16, 1),
        phase_lut::lookup(0, 1),
    );
    assert_eq!(
        phase_lut::lookup((MOTOR_PERIOD as u16) + 7, 1),
        phase_lut::lookup(7, 1),
    );
}
```

- [ ] **Step 1.2: Run the test and verify it fails to compile**

```bash
cd /Users/daniladergachev/Developer/kalico/.claude/worktrees/phase-stepping
cargo test -p runtime --test phase_lut 2>&1 | head -30
```

Expected: compile error (`runtime::phase_lut` not found).

- [ ] **Step 1.3: Write `rust/runtime/build.rs`**

```rust
//! Build-script-generated identity-sinusoid LUT for phase stepping.
//!
//! Writes `phase_lut_table.rs` into OUT_DIR with a `pub const LUT_ENTRIES`
//! array of (i16, i16). Included by `rust/runtime/src/phase_lut.rs`.

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::Path;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");

    const MOTOR_PERIOD: usize = 1024;
    const CURRENT_AMPLITUDE: i16 = 248;

    let out_dir = env::var("OUT_DIR").expect("OUT_DIR not set");
    let dest = Path::new(&out_dir).join("phase_lut_table.rs");
    let mut f = File::create(&dest).expect("create phase_lut_table.rs");

    writeln!(f, "// Auto-generated by build.rs — do not edit.").unwrap();
    writeln!(f, "pub const LUT_ENTRIES: [(i16, i16); {}] = [", MOTOR_PERIOD).unwrap();
    for i in 0..MOTOR_PERIOD {
        let angle = 2.0 * std::f64::consts::PI * (i as f64) / (MOTOR_PERIOD as f64);
        let i_a = (CURRENT_AMPLITUDE as f64 * angle.sin()).round() as i16;
        let i_b = (CURRENT_AMPLITUDE as f64 * angle.cos()).round() as i16;
        // Clamp to the i16 representable range (the multiply can produce 248.0
        // exactly at the anchors; rounding cannot escape the bound, but be
        // explicit so future amplitude changes don't silently overflow).
        let i_a = i_a.clamp(-CURRENT_AMPLITUDE, CURRENT_AMPLITUDE);
        let i_b = i_b.clamp(-CURRENT_AMPLITUDE, CURRENT_AMPLITUDE);
        writeln!(f, "    ({}, {}),", i_a, i_b).unwrap();
    }
    writeln!(f, "];").unwrap();
}
```

- [ ] **Step 1.4: Write `rust/runtime/src/phase_lut.rs`**

```rust
//! TMC5160 phase-stepping current LUT.
//!
//! Maps a 10-bit `mscount` (electrical-cycle position, 0..1023) and a
//! direction sign to a `(coil_A, coil_B)` current pair suitable for writing
//! to the TMC5160 `XDIRECT` register. The table is compile-time generated
//! by `build.rs` as a Prusa-faithful identity sinusoid with amplitude
//! `CURRENT_AMPLITUDE = 248` (matches `phase_stepping_opts.h`).
//!
//! For the identity LUT, `direction` is ignored — forward and reverse
//! produce the same currents because the sinusoid is symmetric. Calibration
//! LUTs (silicon follow-up) will introduce per-direction asymmetry to
//! compensate for back-EMF; the lookup signature is preserved here so the
//! call sites do not need to change.

pub const MOTOR_PERIOD: usize = 1024;
pub const CURRENT_AMPLITUDE: i16 = 248;

include!(concat!(env!("OUT_DIR"), "/phase_lut_table.rs"));

/// Return `(coil_A, coil_B)` for the given electrical-cycle position.
///
/// `mscount` may exceed `MOTOR_PERIOD` — the lookup masks the input to the
/// 10-bit electrical-cycle width so callers don't need to pre-wrap.
/// `direction` is `+1`, `0`, or `-1`; ignored for the identity LUT.
#[inline]
pub fn lookup(mscount: u16, _direction: i8) -> (i16, i16) {
    let idx = (mscount as usize) & (MOTOR_PERIOD - 1);
    LUT_ENTRIES[idx]
}
```

- [ ] **Step 1.5: Wire the module into the crate** — edit `rust/runtime/src/lib.rs`, adding among the other `pub mod` lines:

```rust
pub mod phase_lut;
```

- [ ] **Step 1.6: Ensure `Cargo.toml` declares build.rs** — open `rust/runtime/Cargo.toml`; if there is no `build = "build.rs"` line in `[package]`, add it. Cargo also picks up a `build.rs` in the crate root by default, so the line may be unnecessary — verify with `grep build rust/runtime/Cargo.toml`.

- [ ] **Step 1.7: Run the tests**

```bash
cargo test -p runtime --test phase_lut
```

Expected: all 4 tests pass.

- [ ] **Step 1.8: Commit**

```bash
git add rust/runtime/build.rs rust/runtime/src/phase_lut.rs rust/runtime/src/lib.rs rust/runtime/Cargo.toml rust/runtime/tests/phase_lut.rs
git commit -m "feat(runtime): phase-stepping identity sinusoid LUT"
```

---

## Task 2: `PhaseDirectModulator` core math — mscount + phase-advance accumulator (no SPI, no integration)

**Files:**
- Create: `rust/runtime/src/modulator.rs`
- Modify: `rust/runtime/src/lib.rs` (add `pub mod modulator;`)
- Test: `rust/runtime/tests/modulator_math.rs`

Pure-Rust core of the phase-stepping output stage. No SPI, no FFI, no integration with `runtime_modulated_tick`. Locks in the position-to-mscount formula and the phase-advance direction-tracking logic so they're testable in isolation.

- [ ] **Step 2.1: Write the failing test** — `rust/runtime/tests/modulator_math.rs`:

```rust
//! `PhaseDirectModulator` math: mscount from position, phase-advance
//! accumulator for direction sense, stepper-counts delta.

use runtime::modulator::PhaseDirectModulator;
use runtime::phase_lut::MOTOR_PERIOD;

const STEPS_PER_MM: f32 = 80.0; // typical 256x, 1.8deg, 20T pulley CoreXY

#[test]
fn mscount_from_position_zero() {
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    let r = m.compute(0.0);
    assert_eq!(r.mscount, 0);
}

#[test]
fn mscount_wraps_modulo_electrical_cycle() {
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    // One full electrical cycle = MOTOR_PERIOD microsteps. At
    // steps_per_mm = 80, that's 1024 / 80 = 12.8 mm. Position 12.8 mm
    // should land at mscount = 0 again.
    let r = m.compute(MOTOR_PERIOD as f32 / STEPS_PER_MM);
    assert!(r.mscount == 0 || r.mscount == 1 || r.mscount == (MOTOR_PERIOD as u16 - 1),
            "expected wrap to ~0, got {}", r.mscount);
}

#[test]
fn mscount_quarter_cycle() {
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    // Quarter electrical cycle = MOTOR_PERIOD/4 = 256 microsteps
    // = 256 / 80 = 3.2 mm.
    let r = m.compute(3.2);
    assert!((r.mscount as i32 - 256).abs() <= 1,
            "expected ~256, got {}", r.mscount);
}

#[test]
fn stepper_counts_delta_is_microstep_rounded() {
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    let r1 = m.compute(0.0);
    assert_eq!(r1.steps_delta, 0); // first call seeds, no delta
    let r2 = m.compute(0.0125); // exactly 1 microstep at 80 steps/mm
    assert_eq!(r2.steps_delta, 1);
    let r3 = m.compute(0.025); // another microstep
    assert_eq!(r3.steps_delta, 1);
}

#[test]
fn direction_sticks_through_sub_microstep_ticks() {
    // At very low velocity, many consecutive ticks may have |delta| < 1
    // microstep. The phase-advance accumulator must NOT report direction = 0
    // every tick — it must hold the last established direction.
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    m.compute(0.0);
    let r1 = m.compute(0.020); // 1.6 microsteps forward -> direction = +1
    assert_eq!(r1.direction, 1);
    // Now creep forward by 0.005 mm (0.4 microsteps) per tick
    let r2 = m.compute(0.025);
    let r3 = m.compute(0.030);
    assert_eq!(r2.direction, 1, "direction must stick through sub-microstep tick");
    assert_eq!(r3.direction, 1);
}

#[test]
fn direction_flips_on_reversal() {
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    m.compute(0.0);
    let _ = m.compute(0.020); // forward, dir = +1
    let r = m.compute(0.005); // moved -0.015 mm (-1.2 microsteps) -> reversed
    assert_eq!(r.direction, -1);
}

#[test]
fn steps_delta_bounds_via_max_per_tick_default() {
    // The modulator's per-tick step burst is bounded the same way the
    // existing `StepMotorState::update` bounds it. A sane default avoids
    // pathological deltas latching `StepBurstExceeded`.
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    m.compute(0.0);
    // 0.5 mm at 80 steps/mm = 40 microsteps in one tick. Should succeed
    // (well below MAX_STEPS_PER_TICK default).
    let r = m.compute(0.5);
    assert_eq!(r.steps_delta, 40);
}
```

- [ ] **Step 2.2: Verify the test fails to compile**

```bash
cargo test -p runtime --test modulator_math 2>&1 | head -20
```

Expected: `runtime::modulator` not found.

- [ ] **Step 2.3: Write `rust/runtime/src/modulator.rs`**

```rust
//! Phase-stepping output-stage state and computation, decoupled from the
//! SPI/trace plumbing for unit testability.
//!
//! The hot-path `runtime_modulated_tick` (engine.rs) gates per-motor on
//! whether a `PhaseDirectModulator` is configured for that motor. The
//! modulator computes the TMC5160 `mscount`, the `(coil_A, coil_B)`
//! current pair via the identity LUT, and a per-tick step delta used to
//! advance `SharedState::stepper_counts` so host position queries and
//! homing snapshots continue to work for phase-stepped axes.

use crate::phase_lut::{self, MOTOR_PERIOD};

/// Per-tick output from `PhaseDirectModulator::compute`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PhaseTickResult {
    /// Electrical-cycle position, 0..MOTOR_PERIOD-1.
    pub mscount: u16,
    /// Coil-A current setpoint, signed 9-bit (-256..255 representable).
    pub i_a: i16,
    /// Coil-B current setpoint, signed 9-bit (-256..255 representable).
    pub i_b: i16,
    /// Direction sense for this tick, +1 / 0 / -1. Sticky across sub-
    /// microstep ticks (see field docs on `last_direction`).
    pub direction: i8,
    /// Integer microstep delta vs the previous tick. Used by the engine to
    /// advance `SharedState::stepper_counts`. May be negative.
    pub steps_delta: i32,
}

/// Per-motor phase-stepping state. Hot-path callers create one per phase-
/// stepped motor at `configure_axes` time and call `compute(motor_pos_mm)`
/// every tick.
#[derive(Debug, Clone, Copy)]
pub struct PhaseDirectModulator {
    steps_per_mm: f32,
    /// f64 accumulator of motor position in microstep units, advanced each
    /// tick by `steps_delta`. The fractional residual stays in the
    /// accumulator the same way `StepMotorState::step_accumulator` does.
    step_accumulator: f64,
    /// Last reported direction sense. Held across ticks where the magnitude
    /// of the per-tick advance is below the direction-update threshold —
    /// prevents flicker at sub-microstep velocities.
    last_direction: i8,
    /// True after the first `compute` call. The first call seeds the
    /// accumulator without reporting a delta (matches `StepMotorState::seed`
    /// semantics — no spurious burst from physical zero).
    seeded: bool,
    /// Max integer microsteps per tick before the engine should raise
    /// `STEP_BURST_EXCEEDED`. Mirrors `StepMotorState::max_steps_per_tick`.
    pub max_steps_per_tick: i32,
}

/// Minimum |advance|, in microsteps per tick, that updates `last_direction`.
/// Below this, the previously-latched direction sticks. 0.5 microsteps is
/// half a microstep — the smallest delta that's unambiguously directional
/// once rounded.
const DIRECTION_UPDATE_THRESHOLD: f64 = 0.5;

impl PhaseDirectModulator {
    pub fn new(steps_per_mm: f32) -> Self {
        Self {
            steps_per_mm,
            step_accumulator: 0.0,
            last_direction: 0,
            seeded: false,
            max_steps_per_tick: 192,
        }
    }

    /// First-tick seed without reporting a step delta. Matches
    /// `StepMotorState::seed` — called once after configure / homing-snap.
    pub fn seed(&mut self, motor_position_mm: f32) {
        self.step_accumulator =
            f64::from(motor_position_mm) * f64::from(self.steps_per_mm);
        self.seeded = true;
        self.last_direction = 0;
    }

    /// Per-tick computation. Returns the mscount, current setpoints,
    /// direction, and steps delta for the engine to apply to
    /// `stepper_counts`.
    pub fn compute(&mut self, motor_position_mm: f32) -> PhaseTickResult {
        let new_pos_steps =
            f64::from(motor_position_mm) * f64::from(self.steps_per_mm);

        if !self.seeded {
            self.step_accumulator = new_pos_steps;
            self.seeded = true;
            // Seed: report zero delta and direction from rest.
            let mscount = wrap_mscount(new_pos_steps);
            let (i_a, i_b) = phase_lut::lookup(mscount, 0);
            return PhaseTickResult {
                mscount,
                i_a,
                i_b,
                direction: 0,
                steps_delta: 0,
            };
        }

        let delta = new_pos_steps - self.step_accumulator;

        // Direction: update only when the per-tick advance is clearly
        // directional. Otherwise the previous direction sticks. This
        // matches the architectural reviewer's "phase-advance accumulator"
        // recommendation — prevents `sign(0)`-driven flicker at sub-
        // microstep velocities.
        if delta.abs() >= DIRECTION_UPDATE_THRESHOLD {
            self.last_direction = if delta > 0.0 { 1 } else { -1 };
        }

        // Integer steps delta: truncate toward zero, residual stays in the
        // accumulator. Same semantics as `StepMotorState::update`.
        let steps_delta = delta as i32;
        self.step_accumulator += f64::from(steps_delta);

        // mscount comes from the *accumulator* (the rounded electrical-
        // cycle position), not from raw motor_position_mm. This ensures
        // mscount and stepper_counts stay phase-coherent across the
        // fractional residual.
        let mscount = wrap_mscount(self.step_accumulator);
        let (i_a, i_b) = phase_lut::lookup(mscount, self.last_direction);

        PhaseTickResult {
            mscount,
            i_a,
            i_b,
            direction: self.last_direction,
            steps_delta,
        }
    }

    /// Reset the fractional residual without dropping `steps_per_mm`.
    /// Mirrors `StepMotorState::reset_accumulator` — used by
    /// `runtime_force_idle` after a flush.
    pub fn reset_accumulator(&mut self) {
        self.step_accumulator = 0.0;
        self.last_direction = 0;
        self.seeded = false;
    }
}

#[inline]
fn wrap_mscount(accumulator_steps: f64) -> u16 {
    let rounded = accumulator_steps.round() as i64;
    let modulus = MOTOR_PERIOD as i64;
    let wrapped = ((rounded % modulus) + modulus) % modulus;
    wrapped as u16
}
```

- [ ] **Step 2.4: Wire the module in** — edit `rust/runtime/src/lib.rs`:

```rust
pub mod modulator;
```

- [ ] **Step 2.5: Run the tests**

```bash
cargo test -p runtime --test modulator_math
```

Expected: all 7 tests pass.

- [ ] **Step 2.6: Commit**

```bash
git add rust/runtime/src/modulator.rs rust/runtime/src/lib.rs rust/runtime/tests/modulator_math.rs
git commit -m "feat(runtime): PhaseDirectModulator core math (mscount + phase-advance direction)"
```

---

## Task 3: C-side XDIRECT SPI helper

**Files:**
- Create: `src/stm32/phase_stepping_spi.c`
- Create: `src/stm32/phase_stepping_spi.h`
- Modify: `src/stm32/Makefile` (add new .c to the build)

A thin C function the Rust modulator calls per tick (when round-robin-due). Constructs the 5-byte `XDIRECT` datagram, toggles CS, runs `spi_transfer`. Blocking — sim-only correctness; silicon swap to DMA is documented as out of scope for this slice (spec §8).

- [ ] **Step 3.1: Find the H7 SPI driver entry point** — confirm the helper signatures it offers:

```bash
grep -n "void spi_transfer\|struct spi_config\|spi_setup\|gpio_out_setup\|gpio_out_write" src/stm32/stm32h7_spi.c src/stm32/gpio.c src/generic/spi_software.c 2>/dev/null | head -20
```

Expected: `spi_transfer(struct spi_config, uint8_t receive_data, uint8_t len, uint8_t *data)` (or similar — adapt the helper signature in Step 3.3 to whatever the H7 driver exposes today).

- [ ] **Step 3.2: Write `src/stm32/phase_stepping_spi.h`**

```c
#ifndef _PHASE_STEPPING_SPI_H
#define _PHASE_STEPPING_SPI_H

#include <stdint.h>

/* Emit a single TMC5160 XDIRECT register write to the SPI bus identified
 * by bus_id (kalico's standard SPI bus index — see board_spi_setup() in
 * src/stm32/spi.c) with chip-select line cs_pin (Klipper GPIO encoding:
 * port<<4 | pin). The function constructs the 40-bit datagram, asserts CS
 * low, performs a blocking 5-byte transfer, deasserts CS.
 *
 * Datagram layout per TMC5160 datasheet (40-bit, MSB first):
 *   byte 0 = 0xAD              -- write bit (0x80) | XDIRECT addr (0x2D)
 *   byte 1 = (coil_b >> 8) & 1 -- coil_B sign bit
 *   byte 2 = coil_b & 0xFF     -- coil_B low 8 bits
 *   byte 3 = (coil_a >> 8) & 1 -- coil_A sign bit
 *   byte 4 = coil_a & 0xFF     -- coil_A low 8 bits
 *
 * coil_a, coil_b: signed 9-bit values in [-256, +255]. Values outside
 * this range are silently clipped by the bit-packing (high bits dropped).
 *
 * SIM-ONLY: this helper is blocking. Silicon implementation per spec §8
 * is DMA-driven with CS released by timer output-compare.
 */
void phase_stepping_write_xdirect(uint8_t bus_id, uint8_t cs_pin,
                                  int16_t coil_a, int16_t coil_b);

#endif
```

- [ ] **Step 3.3: Write `src/stm32/phase_stepping_spi.c`**

```c
// Phase-stepping XDIRECT SPI writer for TMC5160.
// See phase_stepping_spi.h for protocol details.

#include "phase_stepping_spi.h"
#include "spicmds.h"  // struct spi_config, spi_transfer, spi_setup, spi_set_software_bus
#include "gpio.h"     // gpio_out_setup, gpio_out_write, struct gpio_out

// Per-bus cached spi_config. We assume the bridge has already called
// spi_setup() for each phase-stepping bus before configure_axes_blob
// armed the modulator. Lookup table indexed by bus_id.
//
// Bounded to 4 phase-stepping buses (one per phase-stepped motor in the
// 4-motor blob layout). Allocated statically.

#define MAX_PHASE_BUSES 4

struct phase_bus_state {
    struct spi_config cfg;
    struct gpio_out cs;
    uint8_t configured;
};

static struct phase_bus_state phase_buses[MAX_PHASE_BUSES];

void
phase_stepping_register_bus(uint8_t bus_id, struct spi_config cfg,
                             uint8_t cs_pin)
{
    if (bus_id >= MAX_PHASE_BUSES) return;
    phase_buses[bus_id].cfg = cfg;
    phase_buses[bus_id].cs = gpio_out_setup(cs_pin, 1); // idle high
    phase_buses[bus_id].configured = 1;
}

void
phase_stepping_write_xdirect(uint8_t bus_id, uint8_t cs_pin,
                              int16_t coil_a, int16_t coil_b)
{
    if (bus_id >= MAX_PHASE_BUSES || !phase_buses[bus_id].configured)
        return;
    uint8_t datagram[5] = {
        0xAD,
        (uint8_t)(((uint16_t)coil_b >> 8) & 0x01),
        (uint8_t)((uint16_t)coil_b & 0xFF),
        (uint8_t)(((uint16_t)coil_a >> 8) & 0x01),
        (uint8_t)((uint16_t)coil_a & 0xFF),
    };
    gpio_out_write(phase_buses[bus_id].cs, 0);
    spi_transfer(phase_buses[bus_id].cfg, 0, sizeof(datagram), datagram);
    gpio_out_write(phase_buses[bus_id].cs, 1);
}
```

(If the H7 SPI API in step 3.1 differs — for example, if `spi_transfer` takes a different signature or `gpio_out_setup` lives elsewhere — adjust this file to match what compiles. Run `make` after the edit and walk any compile errors.)

- [ ] **Step 3.4: Add the .c to the build** — open `src/stm32/Makefile`. Find the line where stm32-specific objects are listed (typically `stm32-y` or similar). Append `phase_stepping_spi.o`:

```makefile
stm32-y += phase_stepping_spi.o
```

Search for any existing pattern like `stm32-$(CONFIG_MACH_STM32H7) +=` and add the .o there if the existing build is conditional on the H7 family.

- [ ] **Step 3.5: Sim-build verification**

```bash
bash tools/sim/build_sim_firmware.sh 2>&1 | tail -30
```

Expected: build completes without error; `phase_stepping_write_xdirect` symbol present:

```bash
nm out/klipper.elf | grep phase_stepping
```

Expected: at least `phase_stepping_write_xdirect` listed.

- [ ] **Step 3.6: Commit**

```bash
git add src/stm32/phase_stepping_spi.c src/stm32/phase_stepping_spi.h src/stm32/Makefile
git commit -m "feat(stm32): phase_stepping_write_xdirect SPI helper (blocking, sim scope)"
```

---

## Task 4: 33-byte `configure_axes` blob extension — parse, validate, install per-motor modulator config

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` (`kalico_runtime_configure_axes_blob` function around lines 1248–1432)
- Modify: `rust/runtime/src/error.rs` (new error codes)
- Modify: `rust/runtime/src/state.rs` (`SharedState`: new `phase_config: [AtomicPhaseConfig; 4]` field)
- Create: `rust/runtime/src/phase_config.rs`
- Modify: `rust/runtime/src/lib.rs` (`pub mod phase_config;`)
- Test: `rust/runtime/tests/configure_axes_phase.rs`

The blob's 25-byte format stays valid. The new 33-byte format appends `phase_config[0..3]` as 2 bytes per motor (`spi_bus_id`, `cs_pin_id`). `spi_bus_id == 0xFF` means "no phase config for this motor — use existing StepPulse behavior." `phase_motor_count > 2` is rejected at parse time.

- [ ] **Step 4.1: Write the failing tests** — `rust/runtime/tests/configure_axes_phase.rs`:

```rust
//! 33-byte configure_axes blob parsing: phase config presence, validation,
//! and error-code propagation.

use runtime::error::{
    KALICO_ERR_INVALID_KINEMATICS, KALICO_ERR_INVALID_PHASE_AXIS_COUNT,
    KALICO_OK,
};
use runtime::phase_config::PhaseConfig;

/// Build a 33-byte configure_axes blob with the given phase configs.
/// Returns the raw blob.
fn build_blob(
    kinematics: u8,
    present_mask: u8,
    step_modes: [u8; 4],
    phase_configs: [Option<(u8, u8)>; 4],
) -> Vec<u8> {
    let mut blob = vec![0u8; 33];
    blob[0] = kinematics;
    blob[1] = present_mask;
    blob[2] = 0; // awd_mask
    blob[3] = 0; // invert_mask
    for i in 0..4 {
        let off = 4 + i * 4;
        blob[off..off + 4].copy_from_slice(&80.0f32.to_le_bytes());
    }
    blob[20] = 1; // mcu_caps: PHASE_STEPPING_CAPABLE
    for i in 0..4 {
        blob[21 + i] = step_modes[i];
    }
    for i in 0..4 {
        let (bus, cs) = phase_configs[i].unwrap_or((0xFF, 0xFF));
        blob[25 + i * 2] = bus;
        blob[26 + i * 2] = cs;
    }
    blob
}

#[test]
fn rejects_blob_with_three_phase_motors() {
    // Three motors with phase config -> KALICO_ERR_INVALID_PHASE_AXIS_COUNT.
    let blob = build_blob(
        1, // CartesianXyzAndE
        0b0000_1111,
        [0, 0, 0, 1], // Modulated, Modulated, Modulated, StepTime
        [
            Some((0, 0x05)),
            Some((0, 0x06)),
            Some((0, 0x07)),
            None,
        ],
    );
    // (Test harness invokes kalico_runtime_configure_axes_blob via FFI;
    // see the existing tests in rust/kalico-c-api/tests/ for the pattern.)
    let rc = call_configure_axes(&blob);
    assert_eq!(rc, KALICO_ERR_INVALID_PHASE_AXIS_COUNT);
}

#[test]
fn rejects_phase_config_on_steptime_motor() {
    // Phase config present but step_mode == StepTime -> reject.
    let blob = build_blob(
        1,
        0b0000_1111,
        [1, 1, 1, 1], // all StepTime
        [Some((0, 0x05)), None, None, None],
    );
    let rc = call_configure_axes(&blob);
    assert_eq!(rc, KALICO_ERR_INVALID_KINEMATICS);
}

#[test]
fn accepts_two_phase_motors() {
    // X+Y Modulated with SPI config, Z+E StepTime, no phase config -> OK.
    let blob = build_blob(
        1,
        0b0000_1111,
        [0, 0, 1, 1],
        [Some((0, 0x05)), Some((0, 0x06)), None, None],
    );
    let rc = call_configure_axes(&blob);
    assert_eq!(rc, KALICO_OK);
    // Post-condition: per-motor phase_config is queryable.
    assert_eq!(read_phase_config(0), Some(PhaseConfig { spi_bus_id: 0, cs_pin_id: 0x05 }));
    assert_eq!(read_phase_config(1), Some(PhaseConfig { spi_bus_id: 0, cs_pin_id: 0x06 }));
    assert_eq!(read_phase_config(2), None);
    assert_eq!(read_phase_config(3), None);
}

#[test]
fn legacy_25_byte_blob_still_accepted() {
    // 25-byte blob (no phase config) must still work — no regression to
    // Gate A / Gate B.
    let mut blob = vec![0u8; 25];
    blob[0] = 1; // CartesianXyzAndE
    blob[1] = 0b0000_1111;
    for i in 0..4 {
        let off = 4 + i * 4;
        blob[off..off + 4].copy_from_slice(&80.0f32.to_le_bytes());
    }
    blob[20] = 1; // mcu_caps: PHASE_STEPPING_CAPABLE
    for i in 0..4 { blob[21 + i] = 1; } // all StepTime
    let rc = call_configure_axes(&blob);
    assert_eq!(rc, KALICO_OK);
    for i in 0..4 { assert_eq!(read_phase_config(i), None); }
}

// (call_configure_axes and read_phase_config are test-harness helpers
// implemented in the test file via the kalico-c-api FFI surface. Pattern:
// runtime_init -> kalico_runtime_configure_axes_blob(blob_ptr, blob_len)
// -> introspect via kalico_runtime_query_phase_config (new FFI in Step 4.6).
// See rust/kalico-c-api/tests/configure_axes_blob_step_modes.rs for an
// example of how the existing 25-byte tests are structured.)
```

(Use `rust/kalico-c-api/tests/configure_axes_blob_step_modes.rs` as the structural template for `call_configure_axes` and runtime initialization in the test harness — copy its setup verbatim.)

- [ ] **Step 4.2: Run the failing tests**

```bash
cargo test -p runtime --test configure_axes_phase 2>&1 | head -40
```

Expected: compile error (modules/types not found).

- [ ] **Step 4.3: Add new error codes to `rust/runtime/src/error.rs`**

Open `rust/runtime/src/error.rs` and add:

```rust
pub const KALICO_ERR_INVALID_PHASE_AXIS_COUNT: i32 = -42;  // pick next free
pub const KALICO_ERR_PHASE_BUS_REENTRANT: i32 = -43;       // pick next free
```

(Check the file for the next unused negative integer; the chosen numbers are illustrative. Update the test imports accordingly.)

- [ ] **Step 4.4: Create `rust/runtime/src/phase_config.rs`**

```rust
//! Per-motor phase-stepping SPI config (bus id + CS pin).
//!
//! Populated at `configure_axes` time, read by `runtime_modulated_tick` on
//! every tick. Stored as `AtomicU16` per motor (high byte = bus, low byte =
//! cs_pin) so the ISR can read without locking; foreground writes once
//! during configure.
//!
//! `spi_bus_id == 0xFF` means "no phase config for this motor — use the
//! existing StepPulse output path."

use core::sync::atomic::{AtomicU16, Ordering};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PhaseConfig {
    pub spi_bus_id: u8,
    pub cs_pin_id: u8,
}

const NONE_SENTINEL: u16 = 0xFFFF;

impl PhaseConfig {
    pub fn pack(self) -> u16 {
        ((self.spi_bus_id as u16) << 8) | (self.cs_pin_id as u16)
    }

    pub fn unpack(raw: u16) -> Option<Self> {
        if raw == NONE_SENTINEL {
            None
        } else {
            Some(PhaseConfig {
                spi_bus_id: (raw >> 8) as u8,
                cs_pin_id: (raw & 0xFF) as u8,
            })
        }
    }
}

pub fn store(slot: &AtomicU16, cfg: Option<PhaseConfig>) {
    let raw = cfg.map_or(NONE_SENTINEL, |c| c.pack());
    slot.store(raw, Ordering::Release);
}

pub fn load(slot: &AtomicU16) -> Option<PhaseConfig> {
    PhaseConfig::unpack(slot.load(Ordering::Acquire))
}
```

- [ ] **Step 4.5: Add `phase_config` storage to `SharedState`** — open `rust/runtime/src/state.rs`, find the `pub struct SharedState` definition (around lines 234–252). Inside the struct, after `step_modes`, add:

```rust
    /// Per-motor phase-stepping SPI config. `0xFFFF` means no phase config
    /// (use StepPulse output). Populated by `configure_axes_blob`, read by
    /// `runtime_modulated_tick`.
    pub phase_config: [AtomicU16; MAX_STEPPER_OIDS],
```

Then in the constructor (around `impl SharedState` / `pub fn new()` near line 410), initialize:

```rust
    phase_config: [
        AtomicU16::new(0xFFFF),
        AtomicU16::new(0xFFFF),
        AtomicU16::new(0xFFFF),
        AtomicU16::new(0xFFFF),
        AtomicU16::new(0xFFFF),
        AtomicU16::new(0xFFFF),
        AtomicU16::new(0xFFFF),
        AtomicU16::new(0xFFFF),
    ],
```

Add `use core::sync::atomic::AtomicU16;` near the top of state.rs if not already imported.

- [ ] **Step 4.6: Extend `kalico_runtime_configure_axes_blob` in `runtime_ffi.rs`**

Open `rust/kalico-c-api/src/runtime_ffi.rs`, find the function (around line 1270), update the `blob_len` check and add a 33-byte parse branch:

```rust
// Accept 20-byte (legacy), 25-byte (extended), or 33-byte (with phase config).
if blob_len != 20 && blob_len != 25 && blob_len != 33 {
    return KALICO_ERR_INVALID_KINEMATICS;
}
```

After the existing 25-byte `step_modes` parsing (around line 1422), add:

```rust
// 33-byte format: parse per-stepper phase config (spec §4.1).
//
// Layout (8 bytes at offset 25):
//   25, 26: spi_bus_id[0], cs_pin_id[0]
//   27, 28: spi_bus_id[1], cs_pin_id[1]
//   29, 30: spi_bus_id[2], cs_pin_id[2]
//   31, 32: spi_bus_id[3], cs_pin_id[3]
//
// spi_bus_id == 0xFF means no phase config on that motor.
if blob_len == 33 {
    use runtime::phase_config::PhaseConfig;
    let mut phase_motor_count: u32 = 0;
    for i in 0..4 {
        let off = 25 + i * 2;
        let bus = blob[off];
        let cs = blob[off + 1];
        let cfg = if bus == 0xFF {
            None
        } else {
            // Phase config requires step_mode == Modulated.
            if i < 4 {
                let mode_byte = blob[21 + i];
                if mode_byte != runtime::state::StepMode::Modulated as u8 {
                    return KALICO_ERR_INVALID_KINEMATICS;
                }
            }
            phase_motor_count += 1;
            Some(PhaseConfig { spi_bus_id: bus, cs_pin_id: cs })
        };
        if let Some(slot) = shared.phase_config.get(i) {
            runtime::phase_config::store(slot, cfg);
        }
    }
    if phase_motor_count > 2 {
        return KALICO_ERR_INVALID_PHASE_AXIS_COUNT;
    }
}
```

Also: declare a re-export for `KALICO_ERR_INVALID_PHASE_AXIS_COUNT` from the runtime crate's error module so the FFI translation unit can reference it. Search for how `KALICO_ERR_INVALID_KINEMATICS` is wired and follow the same pattern.

- [ ] **Step 4.7: Add an FFI introspection helper** — at the end of the FFI surface in `runtime_ffi.rs`, add:

```rust
/// Test-and-debug helper: read back the parsed phase config for motor
/// `motor_idx`. Returns `0xFFFF` when no phase config is installed.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_query_phase_config(
    rt: *mut KalicoRuntime,
    motor_idx: u8,
) -> u16 {
    if rt.is_null() || motor_idx >= 4 { return 0xFFFF; }
    let ctx = rt.cast::<RuntimeContext>();
    let shared = unsafe { &(*ctx).shared };
    shared.phase_config.get(motor_idx as usize)
        .map(|s| s.load(core::sync::atomic::Ordering::Acquire))
        .unwrap_or(0xFFFF)
}
```

Use this from the test harness's `read_phase_config(motor_idx)` helper.

- [ ] **Step 4.8: Run the tests**

```bash
cargo test -p runtime --test configure_axes_phase
```

Expected: all 4 tests pass.

- [ ] **Step 4.9: Run the existing Gate-A-related tests to confirm no regression**

```bash
cargo test -p runtime --test stream_lifecycle
cargo test -p runtime --test engine_producer_integration
cargo test -p kalico-c-api --test configure_axes_blob_step_modes
```

Expected: all pre-existing tests still pass.

- [ ] **Step 4.10: Commit**

```bash
git add rust/runtime/src/phase_config.rs rust/runtime/src/error.rs \
        rust/runtime/src/state.rs rust/runtime/src/lib.rs \
        rust/kalico-c-api/src/runtime_ffi.rs \
        rust/runtime/tests/configure_axes_phase.rs
git commit -m "feat(runtime): 33-byte configure_axes blob with per-motor phase config"
```

---

## Task 5: `TraceSample::PhaseStep` variant + `runtime_phase_trace_enabled` gate

**Files:**
- Modify: `rust/runtime/src/trace.rs`
- Modify: `rust/runtime/src/state.rs` (`SharedState::phase_trace_enabled: AtomicBool`)
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` (new `kalico_runtime_set_phase_trace_enabled` FFI)
- Modify: `rust/kalico-c-api/include/kalico_runtime.h` (header declaration)
- Test: `rust/runtime/tests/trace_phase_step.rs`

The trace ring needs a new variant carrying `(tick, motor, mscount, i_a, i_b, wrote_spi)`. The push is gated by a foreground-mutable atomic so production builds don't burn 80 kHz of trace bandwidth.

- [ ] **Step 5.1: Read `rust/runtime/src/trace.rs`** — understand the existing `TraceSample` enum layout, ring sizing (`TRACE_RING_N`), and overflow-latch pattern (`sample_drop_pending`). This task extends an existing structure, so the variant must fit the existing serialization path.

- [ ] **Step 5.2: Write the failing test** — `rust/runtime/tests/trace_phase_step.rs`:

```rust
//! Trace-ring PhaseStep variant: push when enabled, skip when disabled.

use runtime::trace::TraceSample;

#[test]
fn phase_step_sample_round_trip() {
    let sample = TraceSample::PhaseStep {
        tick: 12345,
        motor: 0,
        mscount: 512,
        i_a: 0,
        i_b: 248,
        wrote_spi: true,
    };
    // Serialize / deserialize via the existing trace wire format.
    let bytes = sample.to_bytes();
    let restored = TraceSample::from_bytes(&bytes).unwrap();
    assert_eq!(sample, restored);
}
```

(Adapt the (de)serialization calls in this test to match the existing TraceSample serialization API. If the existing API uses a different name, follow it.)

- [ ] **Step 5.3: Add the variant** to `rust/runtime/src/trace.rs`:

```rust
pub enum TraceSample {
    // ... existing variants ...
    PhaseStep {
        tick: u32,
        motor: u8,
        mscount: u16,
        i_a: i16,
        i_b: i16,
        wrote_spi: bool,
    },
}
```

Add a matching tag byte (next free integer) and serialization arms in `to_bytes`/`from_bytes` (or whichever methods exist). Total payload size: 14 bytes ≤ existing per-sample max, so no ring-size changes needed.

- [ ] **Step 5.4: Add the trace-enable flag** — open `rust/runtime/src/state.rs`, in `SharedState`:

```rust
    /// Per-print enable for PhaseStep trace pushes. Default false.
    pub phase_trace_enabled: AtomicBool,
```

Initialize to `AtomicBool::new(false)` in the constructor.

- [ ] **Step 5.5: Add the foreground FFI** — `runtime_ffi.rs`:

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_set_phase_trace_enabled(
    rt: *mut KalicoRuntime,
    enabled: u8,
) -> i32 {
    if rt.is_null() { return KALICO_ERR_NULL_PTR; }
    let ctx = rt.cast::<RuntimeContext>();
    let shared = unsafe { &(*ctx).shared };
    shared.phase_trace_enabled.store(enabled != 0, core::sync::atomic::Ordering::Release);
    KALICO_OK
}
```

Add the declaration to `rust/kalico-c-api/include/kalico_runtime.h`:

```c
int32_t kalico_runtime_set_phase_trace_enabled(struct KalicoRuntime *rt,
                                                uint8_t enabled);
```

- [ ] **Step 5.6: Run the tests**

```bash
cargo test -p runtime --test trace_phase_step
cargo test -p runtime --test stream_lifecycle  # ensure no overflow regressions
```

Expected: new test passes, existing tests still pass.

- [ ] **Step 5.7: Commit**

```bash
git add rust/runtime/src/trace.rs rust/runtime/src/state.rs \
        rust/kalico-c-api/src/runtime_ffi.rs \
        rust/kalico-c-api/include/kalico_runtime.h \
        rust/runtime/tests/trace_phase_step.rs
git commit -m "feat(runtime): TraceSample::PhaseStep variant + phase_trace_enabled gate"
```

---

## Task 6: Wire `PhaseDirectModulator` into `runtime_modulated_tick`

**Files:**
- Modify: `rust/runtime/src/engine.rs` (per-motor loop in `runtime_modulated_tick`, around lines 3172–3215; new field on `Engine`)
- Test: `rust/runtime/tests/modulator_integration.rs`

The actual hot-path integration. For each Modulated motor, check the per-motor phase_config. If present: call `PhaseDirectModulator::compute(m)`, increment `stepper_counts` by `steps_delta`, push a `TraceSample::PhaseStep` (if `phase_trace_enabled`), and — only for the round-robin-due motor — call the C SPI helper. If absent: existing `step_state.update + emit_step_pulses` path.

- [ ] **Step 6.1: Add `phase_modulators` to `Engine`** — open `rust/runtime/src/engine.rs`, find the `Engine` struct definition (around line 320). After the existing `step_state` field, add:

```rust
    /// Per-motor phase-stepping state. Populated lazily on first phase
    /// tick when `shared.phase_config[motor_idx]` reports a phase config.
    phase_modulators: [Option<crate::modulator::PhaseDirectModulator>; 4],
    /// Monotonic tick counter for round-robin scheduling.
    phase_tick_counter: u32,
```

Initialize in the constructor (around line 397):

```rust
    phase_modulators: [None, None, None, None],
    phase_tick_counter: 0,
```

- [ ] **Step 6.2: Add an FFI declaration for the C SPI helper**

In `rust/runtime/src/engine.rs`, near the existing `runtime_emit_step_pulses` declaration (line ~183), add:

```rust
#[cfg(target_os = "none")]
unsafe extern "C" {
    fn phase_stepping_write_xdirect(
        bus_id: u8,
        cs_pin: u8,
        coil_a: i16,
        coil_b: i16,
    );
}

fn write_xdirect(bus_id: u8, cs_pin: u8, coil_a: i16, coil_b: i16) {
    #[cfg(target_os = "none")]
    {
        // SAFETY: stable C ABI.
        unsafe { phase_stepping_write_xdirect(bus_id, cs_pin, coil_a, coil_b); }
    }
    #[cfg(not(target_os = "none"))]
    {
        // Host-test path: capture into a test sink so unit tests can
        // assert on it. See test_capture_xdirect_writes below.
        let _ = (bus_id, cs_pin, coil_a, coil_b);
        crate::test_xdirect_capture::record(bus_id, cs_pin, coil_a, coil_b);
    }
}
```

Create `rust/runtime/src/test_xdirect_capture.rs` for the host-test capture sink:

```rust
//! Host-test sink for XDIRECT writes. Production builds (target_os =
//! "none") use the C FFI; host-build tests record into a thread-local
//! ring for assertion.

#[cfg(not(target_os = "none"))]
use std::sync::Mutex;
#[cfg(not(target_os = "none"))]
use once_cell::sync::Lazy;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct XDirectRecord {
    pub bus_id: u8,
    pub cs_pin: u8,
    pub coil_a: i16,
    pub coil_b: i16,
}

#[cfg(not(target_os = "none"))]
static SINK: Lazy<Mutex<Vec<XDirectRecord>>> = Lazy::new(|| Mutex::new(Vec::new()));

#[cfg(not(target_os = "none"))]
pub fn record(bus_id: u8, cs_pin: u8, coil_a: i16, coil_b: i16) {
    SINK.lock().unwrap().push(XDirectRecord { bus_id, cs_pin, coil_a, coil_b });
}

#[cfg(not(target_os = "none"))]
pub fn drain() -> Vec<XDirectRecord> {
    let mut g = SINK.lock().unwrap();
    let out = g.clone();
    g.clear();
    out
}
```

Add `pub mod test_xdirect_capture;` to `lib.rs`.

(Add `once_cell` to dev-dependencies of `runtime` if not present.)

- [ ] **Step 6.3: Modify the per-motor loop in `runtime_modulated_tick`** — engine.rs around lines 3172–3215. Replace the body of the `for motor_idx in 0..4` loop:

```rust
        // Compute count of phase-stepped motors for round-robin scheduling.
        // Counted here (not cached) because `phase_config` is foreground-
        // writable. With N <= 2 (enforced at configure_axes parse) this is
        // ~4 atomic loads — negligible.
        let mut phase_motor_count: u32 = 0;
        let mut phase_motor_ordinals: [Option<usize>; 4] = [None; 4];
        for i in 0..4 {
            if let Some(slot) = shared.phase_config.get(i) {
                if crate::phase_config::load(slot).is_some() {
                    phase_motor_ordinals[phase_motor_count as usize] = Some(i);
                    phase_motor_count += 1;
                }
            }
        }
        let phase_motor_due = if phase_motor_count > 0 {
            phase_motor_ordinals[
                (self.phase_tick_counter % phase_motor_count) as usize
            ]
        } else {
            None
        };
        let trace_enabled = shared.phase_trace_enabled.load(Ordering::Acquire);

        for motor_idx in 0..4_usize {
            let mode = shared
                .step_modes
                .get(motor_idx)
                .map(|m| m.load(Ordering::Acquire))
                .unwrap_or(StepMode::StepTime as u8);
            if mode != StepMode::Modulated as u8 {
                continue;
            }
            let Some(&m) = motors.get(motor_idx) else { continue; };
            let phase_cfg = shared.phase_config.get(motor_idx)
                .and_then(|s| crate::phase_config::load(s));

            match phase_cfg {
                Some(cfg) => {
                    // Phase-stepping output path.
                    let steps_per_mm = self.step_state.get(motor_idx)
                        .map(|s| s.debug_steps_per_mm())
                        .unwrap_or(0.0);
                    let modulator = self.phase_modulators[motor_idx]
                        .get_or_insert_with(|| crate::modulator::PhaseDirectModulator::new(steps_per_mm));
                    let r = modulator.compute(m);

                    // Maintain stepper_counts (preserves homing snapshots
                    // and host position queries — see spec §3.1 step 1).
                    if r.steps_delta != 0 {
                        if let Some(counter) = shared.stepper_counts.get(motor_idx) {
                            counter.fetch_add(r.steps_delta, Ordering::AcqRel);
                        }
                    }

                    let wrote_spi = phase_motor_due == Some(motor_idx);
                    if wrote_spi {
                        write_xdirect(cfg.spi_bus_id, cfg.cs_pin_id, r.i_a, r.i_b);
                    }

                    if trace_enabled {
                        let sample = crate::trace::TraceSample::PhaseStep {
                            tick: self.phase_tick_counter,
                            motor: motor_idx as u8,
                            mscount: r.mscount,
                            i_a: r.i_a,
                            i_b: r.i_b,
                            wrote_spi,
                        };
                        // Use the existing trace push path; on failure the
                        // existing sample_drop_pending latch records it.
                        let _ = trace.enqueue(sample);
                    }
                }
                None => {
                    // Existing StepPulse path: unchanged from today.
                    let Some(ss) = self.step_state.get_mut(motor_idx) else { continue; };
                    match ss.update(m) {
                        Ok(step_result) => {
                            if step_result.n_steps != 0 {
                                if let Some(counter) = shared.stepper_counts.get(motor_idx) {
                                    counter.fetch_add(step_result.n_steps, Ordering::AcqRel);
                                }
                                emit_step_pulses(motor_idx as u8, step_result.n_steps);
                            }
                        }
                        Err(()) => {
                            shared.last_error.store(
                                crate::error::KALICO_ERR_STEP_BURST_EXCEEDED,
                                Ordering::Release);
                            shared.runtime_status.store(
                                RuntimeStatus::Fault as u8, Ordering::Release);
                            self.last_error.store(
                                crate::error::KALICO_ERR_STEP_BURST_EXCEEDED,
                                Ordering::Release);
                            self.status.store(
                                RuntimeStatus::Fault as u8, Ordering::Release);
                            return;
                        }
                    }
                }
            }
        }
        self.phase_tick_counter = self.phase_tick_counter.wrapping_add(1);
        self.last_motors = motors;
```

(Adapt `trace.enqueue` to whatever the existing trace producer's push method is named. If it's `try_push` or `push`, follow that.)

- [ ] **Step 6.4: Write the integration test** — `rust/runtime/tests/modulator_integration.rs`:

```rust
//! End-to-end integration: a synthetic ramp position trajectory exercises
//! the phase modulator path inside runtime_modulated_tick (or its public
//! shim). Asserts:
//! - stepper_counts advances correctly for phase-stepped motors
//! - round-robin schedule: X and Y alternate writing SPI
//! - the XDirect captures match the LUT for the expected mscount sequence

use runtime::test_xdirect_capture;

#[test]
fn two_phase_motors_round_robin_capture() {
    // ... initialize a runtime, configure_axes with X+Y phase + Z+E StepTime
    // ... run N ticks of a ramp position
    // ... drain test_xdirect_capture; assert alternation and per-tick values
    // (Use the pattern from rust/runtime/tests/engine_producer_integration.rs
    // for runtime setup.)
}
```

(Flesh this test out with concrete numbers when implementing; the test harness setup mirrors `engine_producer_integration.rs`.)

- [ ] **Step 6.5: Run the tests**

```bash
cargo test -p runtime --test modulator_integration
cargo test -p runtime  # full suite — verifies no regressions
```

Expected: new test passes; existing tests unchanged.

- [ ] **Step 6.6: Commit**

```bash
git add rust/runtime/src/engine.rs rust/runtime/src/test_xdirect_capture.rs \
        rust/runtime/src/lib.rs rust/runtime/Cargo.toml \
        rust/runtime/tests/modulator_integration.rs
git commit -m "feat(runtime): wire PhaseDirectModulator into runtime_modulated_tick"
```

---

## Task 7: Renode SPI3 platform overlay

**Files:**
- Modify: `tools/sim/h723_sim.resc`

Renode's bundled `stm32h743.repl` models SPI4 only; SPI3 is an opaque tagged region. Add an `SPI.STM32H7_SPI` peripheral at `0x40003C00` via `LoadPlatformDescriptionFromString` so the firmware's SPI3 writes are observable in the sim.

- [ ] **Step 7.1: Edit `tools/sim/h723_sim.resc`**

Find the line `machine LoadPlatformDescription @/opt/homebrew/Cellar/renode/1.16.1/libexec/platforms/cpus/stm32h743.repl` (line 30 of the existing file). **Below** it (so the overlay applies on top of the base platform), add:

```
# SPI3 is only an opaque tagged range in the bundled stm32h743.repl
# (line 498). We need a real peripheral model for phase-stepping
# observability. Overlay an STM32H7_SPI instance at SPI3's base.
machine LoadPlatformDescriptionFromString \
"""
spi3: SPI.STM32H7_SPI @ sysbus 0x40003C00
    -> nvic@51
"""
```

(IRQ number 51 is the STM32H7 NVIC line for SPI3 per the reference manual. Confirm against Klipper's `src/stm32/stm32h7.c` if necessary.)

- [ ] **Step 7.2: Smoke test**

Launch the sim and confirm SPI3 is now a real peripheral (not a tag):

```bash
bash tools/sim/run_sim.sh &
sleep 5
# In another terminal:
echo "sysbus.spi3" | nc -q 1 localhost 1234   # Renode monitor; adjust port
# Or via XML-RPC robot port (55555) if the monitor isn't exposed.
```

(If neither is wired up, the Renode log itself will show whether `spi3` was registered — search for "SPI3" or "0x40003C00" in the Renode console output.)

- [ ] **Step 7.3: Commit**

```bash
git add tools/sim/h723_sim.resc
git commit -m "sim(renode): overlay STM32H7_SPI peripheral at SPI3 base address"
```

---

## Task 8: Renode TMC5160 stub peripheral

**Files:**
- Create: `tools/sim/renode_peripherals/Tmc5160.cs`
- Modify: `tools/sim/h723_sim.resc` (instantiate two TMC5160 instances on SPI3 + CS GPIOs)

A custom Renode C# peripheral implementing `ISPIPeripheral` plus an `IGPIOReceiver` for the CS pin. Decodes 40-bit datagrams on CS-deasserting edges, decodes GCONF and XDIRECT writes, rejects XDIRECT silently when `direct_mode == 0`, exposes register state and write history via monitor commands.

- [ ] **Step 8.1: Read Renode's `ISPIPeripheral` API**

```bash
find /opt/homebrew/Cellar/renode/1.16.1/libexec/Plugins -name '*.cs' -exec grep -l 'ISPIPeripheral' {} \; 2>/dev/null | head -5
```

Skim one or two implementations (e.g. a generic flash chip) to understand the per-byte `Transmit(byte)` callback shape and how CS-deassert is signaled (`FinishTransmission()` or via `IGPIOReceiver.OnGPIO(int number, bool value)` on the CS pin).

- [ ] **Step 8.2: Write `tools/sim/renode_peripherals/Tmc5160.cs`**

```csharp
//
// TMC5160 SPI-slave stub for Renode. Decodes the 40-bit datagram format,
// tracks GCONF + XDIRECT + IHOLD_IRUN register state, records an
// XDIRECT write history for host-side verification.
//
// Frame boundaries are detected via the CS GPIO line (chip-select),
// not by byte counts — catches CS-framing bugs in the firmware.
//

using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class TMC5160 : ISPIPeripheral, IGPIOReceiver
    {
        public TMC5160(Machine machine)
        {
            this.machine = machine;
            Reset();
        }

        public void Reset()
        {
            frameBuffer.Clear();
            registers.Clear();
            xdirectHistory.Clear();
            csAsserted = false;
            frameErrorCount = 0;
        }

        public void OnGPIO(int number, bool value)
        {
            // CS is the only GPIO we accept. number ignored.
            // Convention: CS asserted = low. Renode passes `value=false` on
            // assert (the GPIO is driven low).
            bool newAsserted = !value;
            if (newAsserted && !csAsserted)
            {
                // CS falling edge: start of frame.
                frameBuffer.Clear();
            }
            else if (!newAsserted && csAsserted)
            {
                // CS rising edge: finalize frame.
                FinishFrame();
            }
            csAsserted = newAsserted;
        }

        public byte Transmit(byte data)
        {
            if (!csAsserted)
            {
                // Byte while CS deasserted = framing error.
                frameErrorCount++;
                return 0;
            }
            frameBuffer.Add(data);
            return 0; // TMC5160 status byte: ignored in this stub.
        }

        public void FinishTransmission()
        {
            // Called by Renode at end of SPI transfer. We use OnGPIO CS
            // edges as the authoritative frame boundary; this is a no-op.
        }

        private void FinishFrame()
        {
            if (frameBuffer.Count != 5)
            {
                this.Log(LogLevel.Warning,
                    "Frame error: expected 5 bytes, got {0}", frameBuffer.Count);
                frameErrorCount++;
                frameBuffer.Clear();
                return;
            }
            byte addrByte = frameBuffer[0];
            bool isWrite = (addrByte & 0x80) != 0;
            byte regAddr = (byte)(addrByte & 0x7F);
            uint value =
                ((uint)frameBuffer[1] << 24) |
                ((uint)frameBuffer[2] << 16) |
                ((uint)frameBuffer[3] << 8) |
                ((uint)frameBuffer[4]);
            if (isWrite)
            {
                if (regAddr == 0x2D /* XDIRECT */)
                {
                    HandleXDirect(value);
                }
                else
                {
                    registers[regAddr] = value;
                }
            }
            frameBuffer.Clear();
        }

        private void HandleXDirect(uint value)
        {
            uint gconf = registers.TryGetValue(0x00, out var g) ? g : 0;
            if ((gconf & (1u << 16)) == 0)
            {
                // direct_mode is off: silently ignore the write (matches
                // real silicon). Recorded in a separate count so the host
                // test can detect missing direct_mode setup.
                xdirectRejectedCount++;
                return;
            }
            registers[0x2D] = value;
            // Decode signed 9-bit coil currents.
            int coilA = SignExtend9(value & 0x1FF);
            int coilB = SignExtend9((value >> 16) & 0x1FF);
            var ts = machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds;
            xdirectHistory.Add(new XDirectRecord {
                TimeUs = (ulong)ts,
                CoilA = coilA,
                CoilB = coilB,
                Raw = value,
            });
            xdirectWriteCount++;
        }

        private static int SignExtend9(uint v)
        {
            v &= 0x1FF;
            return ((v & 0x100) != 0) ? (int)(v | 0xFFFFFE00) : (int)v;
        }

        // Monitor commands

        public uint ReadGconf()
        {
            return registers.TryGetValue(0x00, out var v) ? v : 0;
        }

        public uint ReadXDirect()
        {
            return registers.TryGetValue(0x2D, out var v) ? v : 0;
        }

        public int LastCoilA()
        {
            return xdirectHistory.Count == 0 ? 0
                : xdirectHistory[xdirectHistory.Count - 1].CoilA;
        }

        public int LastCoilB()
        {
            return xdirectHistory.Count == 0 ? 0
                : xdirectHistory[xdirectHistory.Count - 1].CoilB;
        }

        public uint XDirectWriteCount() { return xdirectWriteCount; }
        public uint XDirectRejectedCount() { return xdirectRejectedCount; }
        public uint FrameErrorCount() { return frameErrorCount; }

        public string XDirectHistory(int max)
        {
            int n = Math.Min(max, xdirectHistory.Count);
            int start = xdirectHistory.Count - n;
            var sb = new System.Text.StringBuilder();
            for (int i = start; i < xdirectHistory.Count; i++)
            {
                var r = xdirectHistory[i];
                sb.AppendFormat("{0},{1},{2}\n", r.TimeUs, r.CoilA, r.CoilB);
            }
            return sb.ToString();
        }

        private struct XDirectRecord
        {
            public ulong TimeUs;
            public int CoilA;
            public int CoilB;
            public uint Raw;
        }

        private readonly Machine machine;
        private readonly Dictionary<byte, uint> registers = new();
        private readonly List<byte> frameBuffer = new();
        private readonly List<XDirectRecord> xdirectHistory = new();
        private bool csAsserted;
        private uint xdirectWriteCount;
        private uint xdirectRejectedCount;
        private uint frameErrorCount;
    }
}
```

(Renode 1.16 API specifics — exact namespace and base address types — may differ slightly. After writing the file, run `tools/sim/run_sim.sh` and walk any C# compile errors Renode reports on script load.)

- [ ] **Step 8.3: Instantiate in `h723_sim.resc`** — at the end of the .resc file (before `echo "...ready"`):

```
i @tools/sim/renode_peripherals/Tmc5160.cs

machine LoadPlatformDescriptionFromString \
"""
tmc_x: Sensors.TMC5160 @ spi3
tmc_y: Sensors.TMC5160 @ spi3
"""

# Wire CS pins. PA5 = CS_X, PA6 = CS_Y by convention; the host test
# driver will configure the same in the configure_axes blob.
sysbus.gpioPortA -> tmc_x@5
sysbus.gpioPortA -> tmc_y@6
```

- [ ] **Step 8.4: Smoke test**

```bash
bash tools/sim/run_sim.sh &
sleep 6
# Use Renode XML-RPC robot port to send a monitor command:
python3 -c "
import xmlrpc.client
s = xmlrpc.client.ServerProxy('http://localhost:55555')
print(s.execute('tmc_x ReadGconf'))
"
```

Expected: returns `0` (no GCONF write yet).

- [ ] **Step 8.5: Commit**

```bash
git add tools/sim/renode_peripherals/Tmc5160.cs tools/sim/h723_sim.resc
git commit -m "sim(renode): TMC5160 stub peripheral with CS-edge framing"
```

---

## Task 9: Host-side test driver — `tools/test_sim_phase_stepping.py`

**Files:**
- Create: `tools/test_sim_phase_stepping.py`

End-to-end. Boots the sim, configures phase stepping on X+Y, sends `G1 X10`, drains the trace ring, queries the Renode TMC peripherals, computes a Python ground-truth model, and asserts 3-way agreement.

- [ ] **Step 9.1: Read an existing sim test** for structure:

```bash
cat tools/test_sim_gate_a.py 2>/dev/null | head -80
```

(If absent, use `test_h723_first_light.py` as the template.) Note the patterns: pyserial `socket://localhost:3334` connection, msgproto framing, identify handshake, configure_axes_blob call.

- [ ] **Step 9.2: Write `tools/test_sim_phase_stepping.py`**

```python
#!/usr/bin/env python3
"""
End-to-end Renode sim test: phase-stepping XDIRECT framing validation.

Sends a G1 X10 jog through the kalico runtime with X+Y configured as
phase-stepped axes. Then asserts 3-way agreement between:
  1. firmware trace ring (TraceSample::PhaseStep entries)
  2. Renode TMC5160 peripherals' XDIRECT capture (via Renode XML-RPC)
  3. Python ground-truth model (re-derives expected (mscount, I_a, I_b)
     from the same trajectory the runtime saw).
"""

import math
import struct
import sys
import time
import xmlrpc.client
from pathlib import Path

# (Imports adapted to whatever the existing sim tests use for the
# host-side msgproto / configure_axes_blob construction.)
sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_helpers import (  # provided by the existing sim test infra
    connect_sim, identify, send_blob, drain_trace_ring,
    enable_phase_trace, push_g1, wait_idle,
)

# Identity LUT — same constants as rust/runtime/src/phase_lut.rs.
MOTOR_PERIOD = 1024
CURRENT_AMPLITUDE = 248
STEPS_PER_MM = 80.0      # match the configure_axes blob below

def python_ground_truth_for_jog(distance_mm, feedrate_mm_s, tick_hz=40000):
    """Re-derive expected (mscount, I_a, I_b) per tick per axis from a
    trapezoidal-ramp jog (constant velocity for simplicity in this test).
    """
    duration_s = distance_mm / feedrate_mm_s
    n_ticks = int(duration_s * tick_hz)
    out_x = []
    out_y = []
    for t_idx in range(n_ticks):
        t = t_idx / tick_hz
        # X moves; Y holds at 0.
        x = feedrate_mm_s * t
        y = 0.0
        for (pos, sink) in [(x, out_x), (y, out_y)]:
            steps = round(pos * STEPS_PER_MM)
            mscount = steps % MOTOR_PERIOD
            angle = 2 * math.pi * mscount / MOTOR_PERIOD
            i_a = round(CURRENT_AMPLITUDE * math.sin(angle))
            i_b = round(CURRENT_AMPLITUDE * math.cos(angle))
            sink.append((t_idx, mscount, i_a, i_b))
    return out_x, out_y


def configure_axes_blob_33():
    """Build a 33-byte blob: CartesianXyzAndE, X+Y Modulated with phase
    config (SPI bus 3, CS PA5/PA6), Z+E StepTime.
    """
    blob = bytearray(33)
    blob[0] = 1            # CartesianXyzAndE
    blob[1] = 0b0000_1111  # present_mask
    blob[2] = 0; blob[3] = 0
    for i in range(4):
        struct.pack_into('<f', blob, 4 + i * 4, STEPS_PER_MM)
    blob[20] = 1           # mcu_caps: PHASE_STEPPING_CAPABLE
    blob[21] = 0; blob[22] = 0; blob[23] = 1; blob[24] = 1  # X,Y Modulated; Z,E StepTime
    blob[25] = 3;  blob[26] = 0x05   # X: spi_bus=3, cs=PA5
    blob[27] = 3;  blob[28] = 0x06   # Y: spi_bus=3, cs=PA6
    blob[29] = 0xFF; blob[30] = 0xFF # Z: no phase config
    blob[31] = 0xFF; blob[32] = 0xFF # E: no phase config
    return bytes(blob)


def main():
    sim = connect_sim('socket://localhost:3334')
    identify(sim)

    # 1. TMC init via foreground SPI (bridge analog). Write GCONF.direct_mode=1
    #    to both TMCs. (sim_helpers provides spi_tmc_init for this.)
    from sim_helpers import spi_tmc_init
    spi_tmc_init(sim, cs_pin=0x05, gconf_value=(1 << 16), ihold=31)
    spi_tmc_init(sim, cs_pin=0x06, gconf_value=(1 << 16), ihold=31)

    # 2. configure_axes with phase config.
    rc = send_blob(sim, configure_axes_blob_33())
    assert rc == 0, f'configure_axes returned {rc}'

    # 3. Enable phase trace.
    enable_phase_trace(sim, True)

    # 4. Push G1 X10 F600. (Bridge analog: turn into a NURBS segment.)
    push_g1(sim, axis='X', distance=10.0, feedrate_mm_min=600)
    wait_idle(sim)

    # 5. Drain trace ring.
    trace = drain_trace_ring(sim, filter='PhaseStep')
    trace_x = [e for e in trace if e['motor'] == 0]
    trace_y = [e for e in trace if e['motor'] == 1]

    # 6. Read Renode peripheral state.
    rpc = xmlrpc.client.ServerProxy('http://localhost:55555')
    peri_x_history = parse_history_csv(rpc.execute('tmc_x XDirectHistory 50000'))
    peri_y_history = parse_history_csv(rpc.execute('tmc_y XDirectHistory 50000'))
    peri_x_count = int(rpc.execute('tmc_x XDirectWriteCount').strip())
    peri_y_count = int(rpc.execute('tmc_y XDirectWriteCount').strip())
    peri_x_frame_err = int(rpc.execute('tmc_x FrameErrorCount').strip())
    peri_y_frame_err = int(rpc.execute('tmc_y FrameErrorCount').strip())

    # 7. Ground truth.
    truth_x, truth_y = python_ground_truth_for_jog(10.0, 10.0)

    # 8. Assertions.

    # 8a. Trace ring per-tick mscount matches truth.
    for axis_name, trace_axis, truth_axis in [('X', trace_x, truth_x), ('Y', trace_y, truth_y)]:
        assert len(trace_axis) == len(truth_axis), \
            f'{axis_name}: trace length {len(trace_axis)} != truth {len(truth_axis)}'
        for t, gt in zip(trace_axis, truth_axis):
            assert t['mscount'] == gt[1], f'{axis_name} t={gt[0]}: mscount mismatch'
            assert abs(t['i_a'] - gt[2]) <= 1, f'{axis_name} t={gt[0]}: i_a mismatch'
            assert abs(t['i_b'] - gt[3]) <= 1, f'{axis_name} t={gt[0]}: i_b mismatch'

    # 8b. Renode capture matches the subset of trace entries with wrote_spi=true.
    wrote_x = [t for t in trace_x if t['wrote_spi']]
    wrote_y = [t for t in trace_y if t['wrote_spi']]
    assert len(peri_x_history) == len(wrote_x), \
        f'peri_x: {len(peri_x_history)} writes, expected {len(wrote_x)}'
    assert len(peri_y_history) == len(wrote_y), \
        f'peri_y: {len(peri_y_history)} writes, expected {len(wrote_y)}'
    for pr, tr in zip(peri_x_history, wrote_x):
        assert abs(pr['coil_a'] - tr['i_a']) <= 1
        assert abs(pr['coil_b'] - tr['i_b']) <= 1

    # 8c. Round-robin: write counts are equal within 1.
    assert abs(peri_x_count - peri_y_count) <= 1, \
        f'round-robin imbalance: X={peri_x_count} Y={peri_y_count}'

    # 8d. No frame errors.
    assert peri_x_frame_err == 0
    assert peri_y_frame_err == 0

    # 8e. GCONF readback confirms direct_mode is set.
    gconf_x = int(rpc.execute('tmc_x ReadGconf').strip(), 0)
    gconf_y = int(rpc.execute('tmc_y ReadGconf').strip(), 0)
    assert gconf_x & (1 << 16), 'GCONF.direct_mode not set on tmc_x'
    assert gconf_y & (1 << 16), 'GCONF.direct_mode not set on tmc_y'

    print('PASS')


def parse_history_csv(text):
    out = []
    for line in text.strip().split('\n'):
        if not line: continue
        parts = line.split(',')
        out.append({'time_us': int(parts[0]),
                    'coil_a': int(parts[1]),
                    'coil_b': int(parts[2])})
    return out


if __name__ == '__main__':
    main()
```

(If `sim_helpers` does not exist yet, you'll need to write a small helper module that wraps the msgproto / configure_axes / G1-segment / trace-drain pattern used by the existing sim tests. The pattern is the same one `test_sim_gate_a.py` uses — extract or copy it.)

- [ ] **Step 9.3: Initial run — expect failures, debug iteratively**

```bash
bash tools/sim/build_sim_firmware.sh
bash tools/sim/run_sim.sh &
sleep 8
python3 tools/test_sim_phase_stepping.py
```

Walk through every assertion failure. Common debugging:
- Length mismatch on trace_x: round-robin counter likely off-by-one or `wrote_spi` flag flipped — instrument `runtime_modulated_tick`'s phase-counter increment.
- CoilA / coilB ±1 mismatch: rounding direction in `build.rs` vs Python — both must use `.round()` not truncation.
- Peri history missing entries: Renode peripheral CS edges not arriving — verify `gpio` → `tmc_x@5` wiring in the .resc.
- Frame errors > 0: byte ordering in C helper vs C# peripheral — re-verify §5.2 of the spec.

- [ ] **Step 9.4: Commit**

```bash
git add tools/test_sim_phase_stepping.py
git commit -m "test(sim): end-to-end phase-stepping 3-way verification harness"
```

---

## Task 10: Run existing sim tests, ensure no regressions

**Files:** none modified — verification only.

- [ ] **Step 10.1: Run Gate A**

```bash
bash tools/sim/run_sim.sh &
sleep 6
python3 tools/test_sim_gate_a.py
```

Expected: PASS (unchanged — Modulated motors without phase config still take the StepPulse path).

- [ ] **Step 10.2: Run Gate B**

```bash
python3 tools/test_sim_gate_b.py --all
```

Expected: PASS-with-WARN at same level as before this work (items 5 and 7 may legitimately WARN per the existing sim limitations; new items must not WARN or FAIL).

- [ ] **Step 10.3: Run the full Rust test suite**

```bash
cargo test --workspace 2>&1 | tail -40
```

Expected: all green.

- [ ] **Step 10.4: Tag the work**

If everything passes, the implementation is complete per spec §9 acceptance criteria. Notify the user and propose: (a) review the test artifact, (b) prepare a PR / merge to `sota-motion`, (c) write the Step-10-hardware follow-up spec for the silicon concerns enumerated in spec §8.

---

## Self-review against the spec

After writing the plan, the brainstorming protocol requires a fresh-eyes scan. Done inline:

- **§1 Goal / acceptance criterion:** covered by Tasks 1–9 (modulator + Renode + 3-way test).
- **§2 Background:** referenced; no work needed.
- **§3.1 Modulator hook point + stepper_counts:** Task 6.
- **§3.2 phase_motor_count ≤ 2 parse-time reject:** Task 4 (`KALICO_ERR_INVALID_PHASE_AXIS_COUNT`).
- **§3.3 Trace all phase motors every tick:** Task 6.3 (trace push outside the round-robin gate).
- **§4.1 33-byte blob:** Task 4.
- **§4.2 Bridge-side TMC init (IHOLD, GCONF, dir_pin, StealthChop):** scoped to the bridge — the sim test simulates this by writing GCONF + IHOLD via raw SPI in Step 9.2's `spi_tmc_init`. The full bridge integration (klippy-side enforcement) is documented in the spec as bridge work, not runtime — and is **not in this plan**. (Worth flagging to the user that the bridge changes are a separate follow-up; the sim test covers the protocol surface.)
- **§4.3 SPI bus ownership:** convention-based; Renode peripheral records all writes so foreground violations would surface as out-of-sequence captures. Not enforced in code in this slice — accepted per spec.
- **§5 MCU implementation:** Tasks 1, 2, 3, 5, 6.
- **§6 Renode peripheral:** Tasks 7, 8.
- **§7 Host test:** Task 9.
- **§8 Silicon follow-up:** explicitly out of scope for this plan; spec §8 enumerates.
- **§9 Acceptance criteria:** Task 10 validates points 4, 5, 6.

**Placeholder scan:** no "TODO" / "TBD" / "fill in later" markers remain in code blocks. Test bodies in Task 6.4 ("flesh this test out with concrete numbers when implementing") are an exception flagged as such — the integration-test scaffolding pattern is concrete enough that the implementer can fill from the existing `engine_producer_integration.rs` template. Acceptable.

**Type consistency:** `PhaseDirectModulator::compute(motor_position_mm: f32) -> PhaseTickResult` consistent across Task 2 definition and Task 6 call site. `phase_config::PhaseConfig { spi_bus_id: u8, cs_pin_id: u8 }` consistent across Tasks 4, 6, 9. `TraceSample::PhaseStep` fields consistent across Tasks 5, 6, 9.

**Scope:** the plan is one cohesive change-set: ~9 commits, each independently sensible. No subsystem decomposition needed.

---

## Open items the implementer may need to resolve

- The exact Renode `ISPIPeripheral` API surface (per-byte `Transmit` vs alternative interfaces) — verify against Renode 1.16.1 source in Step 8.1.
- The H7 SPI driver's exact `spi_transfer` signature — verify in Step 3.1 and adapt Step 3.3 accordingly.
- IRQ number for SPI3 in Step 7.1 (claimed 51; reference manual for STM32H723 should confirm).
- `sim_helpers` module — if not present, factor common patterns from existing `test_sim_*.py` scripts into a small helper module before Task 9. This is a tooling cleanup, not a design decision.
