# Phase-stepping XDIRECT framing — Renode sim validation design

**Date:** 2026-05-18
**Build-order item:** Step 10 (Phase stepping current synthesis) — sim slice. Silicon bring-up follow-up is out of scope.
**Author:** Brainstorm pass, reviewed independently by `kalico-plan-reviewer`, `architect-reviewer`, `codex-rescue`.
**Status:** Design draft, awaiting user approval before plan-writing.

## 1. Goal and non-goals

**Goal.** Make the firmware emit a correct stream of `XDIRECT` SPI writes to TMC5160 drivers — at 40 kHz, round-robined across X and Y — when running under the Renode H7 simulator with an identity sinusoid LUT, and verify that stream three ways:

1. firmware-side trace-ring entries (one per phase-stepped motor per tick),
2. a Renode `TMC5160` peripheral model capturing decoded `XDIRECT` writes per axis,
3. a Python ground-truth model that re-derives the expected `(mscount, I_a, I_b)` sequence from the same G-code segment the runtime consumed.

A test driver (`tools/test_sim_phase_stepping.py`) sends `G1 X10 F600`, then asserts all three sources agree within numerical tolerance.

**Acceptance criterion (deliberately scoped).** *XDIRECT framing validated in Renode.* Not "phase stepping works on silicon." The sim catches dispatch correctness, round-robin scheduling, `mscount`-from-position math, datagram byte layout. The sim does **not** catch SPI clock rate, CS-edge timing, DMA-vs-blocking-write differences, motor electrical behavior, bus arbitration, or any back-EMF / mechanical phenomenon. Silicon validation is a separate Step-7-D / Step-10-hardware item.

**Non-goals.**

- Per-motor LUT calibration (silicon-only — no motor / accelerometer in sim).
- Pattern B burst stepping for F4 boards (deferred).
- DMA-driven SPI with CS-via-timer-output-compare jitter optimization (silicon-only; sim runs synchronous SPI).
- Live `GCONF.direct_mode` toggle during sensorless homing (deferred; this slice ships with phase-stepping enabled-or-disabled at boot only).
- Cycle-budget benchmarking (Renode's `CYCCNT` is software-faked).

## 2. Background

Two research notes form the prior-art baseline; this design assumes the reader has internalized both:

- [`docs/research/open-loop-phase-stepping-prior-art.md`](../../research/open-loop-phase-stepping-prior-art.md) — Prusa Pattern A (XDIRECT + dedicated TIM + DMA, 40 kHz round-robin one-axis-per-ISR), Pattern B (burst stepping, F4 fallback), Pattern C (decoupled, classic Klipper).
- [`docs/research/tmc5160-open-loop-phase-stepping.md`](../../research/tmc5160-open-loop-phase-stepping.md) — TMC5160 `GCONF.direct_mode`, `XDIRECT` register at 0x2D, 9-bit signed coil currents, 40-bit datagram structure, `fCLK/2 ≤ 6 MHz` SCK limit, IHOLD-scaling semantics in direct mode.

Build-order context (`CLAUDE.md`): Step 7-B landed the per-axis NURBS evaluator at 40 kHz with **hybrid stepping** (TIM5-driven step-pulse emission); Step 10 is "phase stepping current synthesis," which swaps the output stage from step pulses to XDIRECT SPI writes inside the same TIM5 ISR. `StepMode::Modulated` was designed with this swap in mind — `rust/runtime/src/state.rs:49-58` docstring: *"Current behavior (polled curve eval + StepAccumulator). Future: grows to include sin/cos commutation per build-order Step 10."*

The Renode sim (`tools/sim/`) models the H7 CPU, TIM5, NVIC, USART, and a generic STM32H7 SPI peripheral on SPI4. It does **not** model TMC chips, motors, encoders, accelerometers, or DWT cycle accuracy. `tools/sim/h723_sim.resc` adapts Renode's `stm32h743.repl` because no H723-specific .repl ships with Renode 1.16.1.

## 3. Architecture

### 3.1 Hook point — single, inside `runtime_modulated_tick`

The TIM5 ISR (`runtime_tick_h7.c`, 40 kHz, NVIC priority 3) already calls `runtime_modulated_tick` (`rust/runtime/src/engine.rs:2926`–`3215`). That function today:

1. Lazy-dequeues the active segment.
2. Evaluates per-axis NURBS `x(t), y(t), z(t), e(t)` at current `now`.
3. Applies kinematic transform → 4 motor positions in mm.
4. Polls endstop trip-check; on trip, aborts and returns early.
5. For each motor in `StepMode::Modulated`: calls `step_state.update(m)` → on `Ok(n_steps)`, increments `shared.stepper_counts[motor_idx]` by `n_steps` and calls `emit_step_pulses(motor_idx, n_steps)` (hybrid stepping placeholder, Step 7-B). On `Err`, latches `KALICO_ERR_STEP_BURST_EXCEEDED` and faults.

**Step 10 modification:** the per-motor body is refactored behind a `Modulator` trait. The hot-path loop becomes:

```rust
for motor_idx in 0..4_usize {
    if shared.step_modes.get(motor_idx).map(|m| m.load(Ordering::Acquire))
        != Some(StepMode::Modulated as u8) { continue; }
    let m = motors[motor_idx];
    let modulator = &mut self.modulators[motor_idx];  // resolved at configure_axes time
    match modulator.tick(motor_idx, m, tick_counter, shared, trace) {
        Ok(()) => {}
        Err(fault) => { latch_fault(shared, fault); return; }
    }
}
```

Two `Modulator` impls live in `rust/runtime/src/modulator/`:

- **`StepPulseModulator`** — wraps today's `step_state.update` + `stepper_counts` increment + `emit_step_pulses` path. Existing Gate A / Gate B sim tests run against motors configured to this impl. Failure mode unchanged (`StepBurstExceeded`).
- **`PhaseDirectModulator`** — new. Owns `(spi_bus_id, cs_pin_id, microsteps_per_full_step, last_phase_accumulator, last_motor_pos)` per axis. On `tick`:
  1. **Maintain `stepper_counts`** via the same `step_state.update` accumulator pattern (delta = `round((m - last_motor_pos) * steps_per_mm)`, then `stepper_counts[motor_idx].fetch_add(delta)`). Same overflow-fault semantics as `StepPulseModulator` (KALICO_ERR_STEP_BURST_EXCEEDED) so endstop trip-snapshots, host position queries, and homing all stay correct.
  2. **Round-robin gate.** If `motor_idx != tick_counter % phase_motor_count` → don't write SPI this tick. But **still trace** the computed `(mscount, I_a, I_b)` for cross-motor ground-truth verification (see §3.3).
  3. **`mscount` from position.** `mscount = (round(m * steps_per_mm) as u32) & (MOTOR_PERIOD - 1)`. `MOTOR_PERIOD = 1024` — TMC5160's `MSCNT` is 10-bit, covers one full electrical cycle = 4 full steps × 256 microsteps. This assumes the TMC is configured for 256× microstepping, which is required for phase stepping and enforced at config-time (§4.1). `steps_per_mm` in `MotorConfig` is already in microsteps, so no extra `×256` factor.
  4. **Phase-advance accumulator for direction.** `phase_advance = m * steps_per_mm - last_phase_accumulator`. `direction = sign(phase_advance)` only when `|phase_advance| > ε` (e.g. 0.5 microsteps); otherwise carry over the previous direction. This eliminates the `sign(m - last_m)` flicker at sub-microstep velocities flagged by reviewer #2 (B.5).
  5. **LUT lookup** → `(i_a, i_b) = phase_lut::lookup(mscount, direction)`.
  6. **Trace push** of `TraceSample::PhaseStep { motor: motor_idx, mscount, i_a, i_b, wrote_spi: bool }` to the trace ring. `wrote_spi=true` for the round-robin-due motor; `false` for the others. Trace push is conditional on a `runtime_phase_trace_enabled` atomic (toggled by host for sim runs) to avoid 80 kHz baseline trace pressure on production builds.
  7. **SPI write (only if round-robin-due).** Call into `phase_stepping_write_xdirect(bus_id, cs_pin, i_a, i_b)` (C FFI; details §5.2). Blocking in sim; silicon implementation is DMA-driven (§7).
  8. Update `last_motor_pos`, `last_phase_accumulator`.

The trait dispatch is monomorphizable via a `[Modulator; 4]` array if the impls share size; otherwise via static dispatch through a small `enum`. Hot-path overhead is one `match` arm per motor per tick — negligible.

### 3.2 Round-robin count enforcement

`phase_motor_count` is derived at `configure_axes` time as the count of motors with `(spi_bus_id, cs_pin_id)` set in the new blob extension. Hard-rejected at parse if `phase_motor_count > 2` (reviewer #2 B.4 — at N=3 the per-axis refresh drops to 13.3 kHz, in the audible band). New error code: `KALICO_ERR_INVALID_PHASE_AXIS_COUNT`.

### 3.3 Trace-ring contract for 3-way verification

The trace must let the host reconstruct, after a move, the exact `(mscount, I_a, I_b)` sequence per phase-stepped motor at full 40 kHz resolution, **including the round-robin ticks where SPI was not written.** This is what reviewer #3 was getting at: if the trace only carries the SPI-emitting motor's state, ground-truth comparison is ambiguous because the Python oracle computes both motors every tick.

New trace variant: `TraceSample::PhaseStep { tick: u32, motor: u8, mscount: u16, i_a: i16, i_b: i16, wrote_spi: bool }`. `tick` is the global TIM5 tick counter. Pushed for every Modulated-with-SPI-config motor every tick — 80 kHz total at N=2. Trace ring (`TRACE_RING_N`) is sized via `runtime_phase_trace_enabled` so production builds can skip the push.

## 4. Wire-protocol changes

### 4.1 `configure_axes` blob — 33-byte extended variant

Today's blob accepts 20 bytes (legacy) or 25 bytes (with `mcu_caps` + `step_mode[4]`). Add a new 33-byte variant:

```
byte  0      kinematics_tag
byte  1      present_mask
byte  2      awd_mask
byte  3      invert_mask
bytes 4-19   steps_per_mm[0..3]  (f32 LE × 4)
byte  20     mcu_caps
bytes 21-24  step_mode[0..3]
bytes 25-32  phase_config[0..3] — packed 2 bytes per motor:
               byte 0: spi_bus_id  (0xFF = no phase stepping on this motor)
               byte 1: cs_pin_id   (Klipper GPIO encoding: port<<4 | pin)
```

Parse logic (`kalico_runtime_configure_axes_blob`):

- `blob_len ∈ {20, 25, 33}`; anything else → `KALICO_ERR_INVALID_KINEMATICS`.
- For each motor `i`: if `phase_config[i].spi_bus_id != 0xFF`:
  - Require `step_mode[i] == Modulated`. Else → `KALICO_ERR_INVALID_KINEMATICS`.
  - Require `mcu_caps & PHASE_STEPPING_CAPABLE`. Else → `KALICO_ERR_CAPABILITY_MISSING`.
  - Count toward `phase_motor_count`. If count > 2 → `KALICO_ERR_INVALID_PHASE_AXIS_COUNT`.
- For motors with phase config: install `PhaseDirectModulator` in `self.modulators[i]`. Else install `StepPulseModulator`.

### 4.2 Bridge-side TMC init responsibilities

The Klippy bridge configures TMC drivers via existing `[tmc5160 stepper_*]` register-write paths (foreground SPI, pre-print). For any stepper the bridge will configure as `StepMode::Modulated` with phase-stepping output:

1. Write `GCONF` with `direct_mode = 1` (bit 16). **Once at print start.** Without this, the TMC ignores XDIRECT silently (datasheet, `tmc5160-open-loop-phase-stepping.md` §1) — this is the blocker reviewer #1 (B3), reviewer #2 (D.1), reviewer #3 (#6) all flagged.
2. Set `IHOLD_IRUN.IHOLD` from Klipper's `run_current` config. **Reason:** direct mode scales currents by IHOLD, not IRUN (datasheet, reviewer #2 D.2). Klipper's default IHOLD-as-hold-current does not apply.
3. Reject `stealthchop_threshold` on phase-stepped axes (datasheet: StealthChop bypassed in direct mode; reviewer #2 D.3). Fatal Klippy config error.
4. Park the `dir_pin` GPIO at a deterministic level (low) and do not toggle it during phase stepping (reviewer #2 D.7).
5. Write `CHOPCONF` and other init registers at datasheet defaults — explicit out-of-scope for this slice; document as TMC-defaults-only.

The bridge does this BEFORE issuing `configure_axes_blob` with phase config to the MCU. Sequencing: TMC config (foreground SPI, klippy bridge) → `configure_axes_blob` (transfers SPI bus ownership semantics) → first `push_segment` (TIM5 ISR can now safely write XDIRECT).

### 4.3 SPI bus ownership

Critical (reviewer #1 C2): `src/stm32/stm32h7_spi.c::spi_transfer` (lines 128–174) is **blocking**, **non-reentrant**, **foreground-only**, with `shutdown("spi rx timeout")` on a 100 µs/byte deadline. It is not safe to call from an ISR if a foreground task can be using the same bus.

**Ownership contract:**

- During boot / `tmc_init` (foreground): bridge has exclusive use of the bus.
- After `configure_axes_blob` is processed: the bus carrying phase-stepped CS lines is **transferred to ISR-exclusive ownership**. Foreground tasks must not issue SPI on that bus.
- A subsequent foreground TMC write to the phase-stepping bus is a host bug; the runtime detects and faults with `KALICO_ERR_PHASE_BUS_REENTRANT`.

In sim this is enforced by convention (only the ISR writes XDIRECT after configure_axes); the Renode peripheral records all writes and the host test asserts no foreground writes occurred mid-print.

On silicon, this becomes a real concern. The Step-10-hardware follow-up needs either:

- A dedicated SPI bus per phase-stepping axis (BTT Octopus Pro has SPI3 already routed to TMCs; one bus, multiple CS, ISR-only after init), or
- A small mutex / "phase-stepping armed" flag that gates foreground access.

The sim spec proceeds under the convention model; the silicon spec must revisit.

## 5. MCU-side implementation

### 5.1 Rust crates

- **`rust/runtime/src/modulator/mod.rs`** — `Modulator` trait, `pub enum ModulatorImpl { StepPulse(StepPulseModulator), PhaseDirect(PhaseDirectModulator) }`. `tick(...)` method.
- **`rust/runtime/src/modulator/step_pulse.rs`** — current `step_state.update + stepper_counts + emit_step_pulses` body, factored out. Behavior identical to today.
- **`rust/runtime/src/modulator/phase_direct.rs`** — new. Owns per-axis state, calls into the LUT and the SPI helper.
- **`rust/runtime/src/phase_lut.rs`** — `pub fn lookup(mscount: u16, direction: i8) -> (i16, i16)`. Table generated by `build.rs` (see §5.3).

### 5.2 C glue — `src/stm32/phase_stepping_spi.c`

Single function:

```c
void phase_stepping_write_xdirect(uint8_t bus_id, uint8_t cs_pin, int16_t coil_a, int16_t coil_b) {
    // XDIRECT bit layout per TMC5160 datasheet:
    //   bits[8:0]   = coil_a  (signed 9-bit, two's-complement)
    //   bits[24:16] = coil_b  (signed 9-bit, two's-complement)
    //   bits[15:9], bits[31:25] = 0
    //
    // Write datagram (MSB first):
    //   byte 0 = 0xAD          (write bit 0x80 | reg addr 0x2D)
    //   byte 1 = (coil_b >> 8) & 0x01
    //   byte 2 = coil_b & 0xFF
    //   byte 3 = (coil_a >> 8) & 0x01
    //   byte 4 = coil_a & 0xFF
    uint8_t datagram[5] = {
        0xAD,
        (uint8_t)(((uint16_t)coil_b >> 8) & 0x01),
        (uint8_t)(coil_b & 0xFF),
        (uint8_t)(((uint16_t)coil_a >> 8) & 0x01),
        (uint8_t)(coil_a & 0xFF),
    };
    gpio_out_write(cs_pin_handle(cs_pin), 0);
    spi_transfer(spi_bus_handle(bus_id), datagram, 5);
    gpio_out_write(cs_pin_handle(cs_pin), 1);
}
```

**Sim correctness:** blocking SPI in TIM5 is fine under Renode's virtual clock. **Silicon validity:** out of scope for this sim slice. The signature is designed so a silicon-side DMA implementation can substitute without changing the Rust call site (the function becomes "queue and return; CS released by timer-output-compare").

### 5.3 Identity LUT — `build.rs` generation

`build.rs` writes a `phase_lut_table.rs` containing:

```rust
pub const MOTOR_PERIOD: u16 = 1024;
pub const CURRENT_AMPLITUDE: i16 = 248;

pub static IDENTITY_LUT: [(i16, i16); 1024] = [
    // (i_a, i_b) for mscount = 0
    // (CURRENT_AMPLITUDE * sin(2π * 0 / 1024).round() as i16, ...)
    ...
];
```

Direction handling: `lookup(mscount, direction)` returns `IDENTITY_LUT[mscount]` for `direction >= 0`, `IDENTITY_LUT[(MOTOR_PERIOD - mscount - 1) & 0x3FF]` for `direction < 0`. Reviewer #3 correctly flagged that direction semantics aren't pinned in the local research docs; this matches Prusa's `forward_current` / `backward_current` split. Open to refinement against Prusa's `lut.hpp` if it differs.

### 5.4 Capability bit

`mcu_caps |= PHASE_STEPPING_CAPABLE` on H723 builds when `CONFIG_KALICO_PHASE_STEPPING=y` (new Kconfig option). F4 builds leave the bit clear; host-side `configure_axes` is hard-rejected if it requests phase stepping on an MCU that lacks the bit (existing `SetStepModeError::CapabilityMissing` infrastructure).

### 5.5 Faults

- `KALICO_ERR_STEP_BURST_EXCEEDED` — existing; raised by `PhaseDirectModulator` on per-tick delta overflow (same accumulator semantics).
- `KALICO_ERR_INVALID_PHASE_AXIS_COUNT` — new; raised at `configure_axes` if `phase_motor_count > 2`.
- `KALICO_ERR_PHASE_BUS_REENTRANT` — new; raised if a foreground SPI write to the phase-stepping bus is detected after `configure_axes` armed phase stepping. Detection mechanism deferred; for sim, the Renode peripheral can flag this and the test driver fails.

Endstop trip on a phase-stepped axis: the existing `poll_endstop_trip` already early-returns from `runtime_modulated_tick` before the motor loop runs, so the modulator simply doesn't fire that tick. **However**, the previous tick's XDIRECT write is still latched in the TMC; the coil currents stay energized at their last commanded values. For sim this is fine. For silicon, the trip handler should issue a final `XDIRECT(0, 0)` write to disarm (or write `GCONF.direct_mode = 0` to revert to chip's internal sequencer). Document as silicon follow-up; out of scope for sim.

## 6. Renode sim peripheral

### 6.1 SPI3 platform extension

**Constraint discovered during review** (reviewer #3, verified directly): Renode's bundled `stm32h743.repl` models SPI4 only (`SPI.STM32H7_SPI @ sysbus 0x40013400`). SPI1, SPI2, SPI3, SPI5, SPI6 are present as opaque `Tag` regions. Writes to SPI3's TX FIFO are silently absorbed.

The kalico hardware target (BTT Octopus Pro H723) and Prusa Buddy both use SPI3 for TMC drivers; SPI4 is not the production bus. Two options:

- **(A)** Extend our `tools/sim/h723_sim.resc` with an inline platform fragment that registers an `SPI.STM32H7_SPI @ sysbus 0x40003C00` peripheral for SPI3 (Renode supports overlay registration via `machine LoadPlatformDescriptionFromString`). The TMC peripheral attaches to this new SPI3 instance. Matches production.
- **(B)** For sim only, retarget phase stepping to SPI4 (via `spi_bus_id` in the blob). Bridge configures SPI4 in sim runs and SPI3 on real hardware. Maintains parity with the production wiring at the Rust level but introduces a sim-only branch in the bridge.

**Choice: option (A)** — add SPI3 to the .repl via overlay in the .resc. Less divergence between sim and hardware paths; the Rust code thinks it's writing to SPI3 in both cases.

### 6.2 `tools/sim/renode_peripherals/Tmc5160.cs`

A new C# peripheral implementing `ISPIPeripheral`. Conservatively scoped:

- **State:** `gconf` (u32), `xdirect` (u32), `ihold_irun` (u32), `last_xdirect_time` (ulong), `write_count_total` (u32), `write_count_xdirect` (u32), ring of last 1024 (timestamp, register_addr, value) tuples.
- **SPI protocol:** decodes 40-bit (5-byte) datagrams. **Frame boundaries from CS pin edges** (reviewer #1 C7). Renode's `ISPIPeripheral` interface delivers per-byte transfers with explicit chip-select state; the peripheral resets its frame buffer on CS-asserting edge and finalizes the datagram on CS-deasserting edge. If a frame contains anything other than 5 bytes, it logs a warning and flags `frame_error_count`.
- **Behavior:** `register_addr = byte0 & 0x7F`; `write_bit = byte0 & 0x80`. On `write_bit` set, decodes the next 4 bytes (MSB first) as the register value. Updates internal register state. **Rejects XDIRECT writes silently if `gconf & DIRECT_MODE == 0`** — matches real silicon behavior; ensures the sim catches missing `GCONF.direct_mode=1` setup.
- **Renode monitor commands:** `ReadXDirect`, `ReadGconf`, `ReadIholdIrun`, `WriteCountXDirect`, `FrameErrorCount`, `XDirectHistory N` (dump last N entries with timestamps).
- **One instance per phase-stepped motor**, attached to the same SPI3 peripheral but with distinct CS-pin GPIO connections.

### 6.3 `.resc` integration

`h723_sim.resc` adds:

```
i @tools/sim/renode_peripherals/Tmc5160.cs

machine LoadPlatformDescriptionFromString """
spi3: SPI.STM32H7_SPI @ sysbus 0x40003C00
"""

tmc_x: TMC5160 @ spi3
    cs_pin: gpioPortA.5     // adjust per actual sim config
tmc_y: TMC5160 @ spi3
    cs_pin: gpioPortA.6
```

(Exact CS pin assignments depend on the bridge configuration written into the configure_axes blob in the test driver.)

## 7. Host-side test — `tools/test_sim_phase_stepping.py`

```text
1. bash tools/sim/build_sim_firmware.sh    # CONFIG_KALICO_SIM=y CONFIG_KALICO_PHASE_STEPPING=y
2. Launch Renode with extended h723_sim.resc (TMC peripherals registered)
3. Connect to socket://localhost:3334; identify + load curves as Gate A does.
4. Send TMC init sequence (via foreground SPI bus shim — simulates klippy bridge):
   - GCONF.direct_mode = 1 for tmc_x and tmc_y
   - IHOLD = 31 (full-scale identity-LUT operation)
5. configure_axes_blob (33-byte variant):
   - kinematics: CartesianXyzAndE
   - step_mode: [Modulated, Modulated, StepTime, StepTime]
   - phase_config: [(SPI3, CS_X), (SPI3, CS_Y), (0xFF, 0xFF), (0xFF, 0xFF)]
6. Enable phase trace ring (runtime_phase_trace_enabled = 1).
7. Push G1 X10 F600 segment (10mm jog at 10 mm/s; ~1 sec duration ≈ 40k TIM5 ticks).
8. Wait for retire; drain trace ring; query Renode peripherals via robot port.
9. Three-way agreement check, motor X and motor Y independently:
   - Python ground truth: from the same segment, compute position(t) at 40 kHz ticks,
     derive mscount = round(pos * steps_per_mm) mod 1024, lookup identity LUT.
   - Trace ring: extract (tick, motor, mscount, i_a, i_b, wrote_spi) entries.
   - Renode peripheral: query XDirectHistory(N=20000); extract (timestamp, coil_a, coil_b).
10. Assertions:
    - For every tick t in the move: trace[t][motor=X].mscount == python_oracle[t][X].mscount
    - i_a, i_b match within ±1 LSB (rounding tolerance)
    - Renode peripheral's writes match the subset of trace entries with wrote_spi=true
    - Renode peripheral write counts: write_count_xdirect[X] == ticks // 2 ± 1
                                     write_count_xdirect[Y] == ticks // 2 ± 1
    - Renode peripheral.frame_error_count == 0
    - gconf reads back with DIRECT_MODE bit set
```

Pass criterion: all assertions hold. Failure modes the test catches: missing GCONF write, wrong byte ordering in datagram, round-robin bug (write count mismatch), mscount math errors, LUT lookup errors, CS framing bugs (frame_error_count > 0).

## 8. Honest scope and silicon follow-up

The acceptance criterion in §1 is *XDIRECT framing validated in Renode.* The following silicon-side concerns are explicitly out of scope here and must be addressed in a Step-10-hardware spec before bring-up:

- **DMA-driven SPI with timer-output-compare CS release** (Prusa Pattern A). The sim uses synchronous blocking SPI inside TIM5; on silicon this consumes ~40–60% of the 25 µs TIM5 budget per phase write, leaving no headroom for the per-axis evaluator, endstop poll, and step pulse emission for Z+E. The Rust call site is designed so the swap is local.
- **TIM5 ISR-budget measurement** on H723 — `runtime_modulated_tick` worst-case latency with phase-stepping output stage active needs characterization. If > ~15 µs, split the modulator onto its own TIM (Prusa's TIM13 pattern at higher NVIC priority).
- **SPI clock rate clamp** at `≤ 4 MHz` async (`fCLK / 2 = 6 MHz` is the chip limit; 4 MHz with comfortable margin). Currently `stm32h7_spi.c::spi_setup` accepts any rate.
- **Endstop trip disarm** — XDIRECT(0, 0) or GCONF.direct_mode=0 on trip; sim doesn't exercise this path.
- **Calibration LUT upload channel** — identity LUT only in this slice; per-motor calibration data needs an upload command (`runtime_load_phase_lut` analogous to `runtime_load_curve`).
- **`set_step_mode` runtime flip** for sensorless homing — toggling out of Modulated mid-print needs to clear GCONF.direct_mode on the chip and wait for MSCNT resync (datasheet sequencing) before re-engaging.
- **Trace ring overhead** — 80 kHz trace pushes are sim-only; production silicon needs the trace gated behind a runtime feature flag (already designed in §3.3).
- **SPI bus exclusive-ISR ownership** — sim convention; silicon needs a real mutex or a dedicated bus.
- **Renode peripheral as a living artifact** — keeping the TMC5160 stub green across Renode upgrades and future SPI driver changes is ongoing maintenance.

## 9. Acceptance criteria for this slice

1. Spec accepted by user.
2. Implementation plan written (via `superpowers:writing-plans`) and accepted.
3. Code change implements §3–6 with all reviewer-flagged blockers addressed.
4. `tools/test_sim_phase_stepping.py` passes:
   - 3-way agreement on `(mscount, I_a, I_b)` per tick per phase-stepped motor.
   - Write counts and frame_error_count match expectations.
   - Reject paths exercised (e.g. configure_axes with `phase_motor_count = 3` returns `KALICO_ERR_INVALID_PHASE_AXIS_COUNT`; configure_axes with phase config but `step_mode != Modulated` returns `KALICO_ERR_INVALID_KINEMATICS`).
5. Existing Gate A / Gate B sim tests still pass (verifies `StepPulseModulator` factoring is behavior-preserving).
6. Unit tests in `rust/runtime/tests/`:
   - `modulator_step_pulse_preserves_existing_behavior`
   - `modulator_phase_direct_round_robin_alternation`
   - `modulator_phase_direct_stepper_counts_advances`
   - `modulator_phase_direct_mscount_from_position_golden_vectors`
   - `modulator_phase_direct_direction_via_accumulator_no_zero_flicker`
   - `phase_lut_identity_sinusoid_amplitude_quarter_cycle_anchors`
7. No regression in Gate B item 7 (trace-ring overflow handling) — new `PhaseStep` variant fits the existing overflow latch.

## 10. Open questions

- **LUT direction semantics (§5.3)** — `(MOTOR_PERIOD - mscount - 1) & 0x3FF` matches Prusa's `forward_current` / `backward_current` split conceptually but the exact formula is not pinned in the local research docs. If Prusa's `lut.hpp` differs (e.g., negation of one coil instead), update.
- **Microstep resolution lock** — this slice assumes 256× microsteps on phase-stepped TMCs. The bridge must enforce this in TMC init; a Klippy config setting `microsteps != 256` on a phase-stepped axis is a fatal error.
- **Renode `ISPIPeripheral` CS-edge semantics** — the spec assumes per-byte transfers with explicit CS state. Need to verify against Renode 1.16.1's API surface before peripheral implementation begins.
- **`phase_motor_count == 1`** — a single-axis sim demo (degenerate round-robin) is allowed in this spec but adds no value beyond what the X+Y test catches. Document as supported but discouraged.
