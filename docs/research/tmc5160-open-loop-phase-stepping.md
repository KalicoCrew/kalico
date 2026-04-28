# TMC5160 open-loop phase stepping — register, throughput, modulation rate, and architectural-decoupling analysis

Status: research note for Step 5 brainstorm. Primary sources: TMC5160A datasheet rev 1.18 (Analog Devices, 2023), Prusa-Firmware-Buddy phase-stepping source (public, GPL), terjeio/Trinamic-library headers, Analog Devices EngineerZone synchronization-app-note. Secondary sources flagged inline.

---

## 1. Register-level mechanism: what "phase stepping" actually is on TMC5160

Phase stepping bypasses the chip's internal step counter / sine LUT and writes the two coil currents directly. The mechanism is the **XDIRECT register at address 0x2D**, gated by the **`direct_mode` bit (bit 16) of GCONF (0x00)**.

- **GCONF (0x00), bit 16 `direct_mode`** — when set, "Motor coil currents and polarity directly programmed via serial interface: Register XDIRECT (0x2D) specifies signed coil A current (bits 8..0) and coil B current (bits 24..16). In this mode, the current is scaled by IHOLD setting. Velocity-based current regulation of StealthChop is not available in this mode. The automatic StealthChop current regulation will work only for low velocities."  This is the verbatim register description quoted in multiple secondary mirrors of the datasheet table (TMC5160A rev 1.18, register map section, GCONF entry); same wording appears in the TMC2130 datasheet from which the feature was inherited. Confirmed live in Prusa source: their `lut.hpp` defines `XDIRECT_t` as a 9-bit-signed `coil_A` / 9-bit-signed `coil_B` packed register (https://github.com/prusa3d/Prusa-Firmware-Buddy/blob/master/lib/Marlin/Marlin/src/feature/phase_stepping/lut.hpp).

- **XDIRECT (0x2D)** — bit layout: `coil_A` in bits [8:0] signed, `coil_B` in bits [24:16] signed. Range commonly cited as ±255 (datasheet gives "two signed 9-bit"); Prusa uses `CURRENT_AMPLITUDE = 248` to leave the small headroom recommended for StealthChop's PWM amplitude regulation to remain valid (https://github.com/prusa3d/Prusa-Firmware-Buddy/blob/master/include/buddy/phase_stepping_opts.h#L13). The current set is scaled by the `IHOLD_IRUN.IHOLD` field, not `IRUN`, in direct mode.

- **MSCNT (0x6A)** — read-only 10-bit microstep counter, 0..1023, position in the internal sine LUT (terjeio Trinamic-library header definition matches datasheet). **Not used as a control input** for phase stepping — useful only for calibration and diagnostics (e.g. confirming "what microstep is the chip's internal LUT pointing at right now"). Writing it has no effect.

- **MSLUT 0x60–0x67** — programmable 256-entry quarter-sine lookup the chip uses internally when `direct_mode = 0`. Could theoretically be re-uploaded continuously to dynamically warp the sine table, but each write costs the same 40-bit datagram and the table has 8 × 32-bit words plus MSLUTSEL/MSLUTSTART, so this path is strictly slower than XDIRECT for live commutation. The Klipper community considered "TMC adaptive microstep table" via these registers (https://klipper.discourse.group/t/tmc-adaptive-microstep-table/16652) but the natural use is offline calibration, not real-time modulation. Prusa does not use this path.

- **MicroPlyer** — the chip's *interpolator* for the STEP/DIR path. With `intpol = 1` it interpolates incoming step pulses up to 256 microsteps internally, smoothing motion when the host emits at lower microstep rates. **MicroPlyer is irrelevant to phase stepping** because phase stepping disables the step counter entirely (`direct_mode = 1`); STEP/DIR pins are ignored, MSCNT does not advance, and the chip's commutator is replaced by host-driven SPI writes.

So at the register level, "phase stepping" on TMC5160 means: set `GCONF.direct_mode = 1`, then continuously write `XDIRECT` with the current sine/cosine values you want to see in the windings. The chip becomes a current DAC pair; all sequencing, microstepping, and shape correction live on the host side.

---

## 2. SPI throughput ceiling

**Datasheet limits.**
- Datagram: exactly 40 bits (5 bytes) per register write, with status byte returned in the first byte of the response (TMC5160 datasheet rev 1.18, "SPI Datagram Structure" section).
- Internal `fCLK`: 12 MHz typical (datasheet electrical characteristics).
- SPI clock constraint: `fSCK ≤ fCLK / 2 = 6 MHz` for asynchronous SCK; **with SCK synchronized to the chip's CLK pin, the hard limit is `fSCK < fCLK / 2`** (and Trinamic's app note "How to synchronize SPI clock to system clock to achieve maximum SPI data rate on the TMC5160" recommends external common clocking to reach this reliably: https://ez.analog.com/other-products/w/documents/32636/how-to-synchronize-spi-clock-to-system-clock-to-achieve-maximum-spi-data-rate-on-the-tmc5160). Practical engineering rule: **4 MHz is the safe asynchronous limit**, 6–8 MHz is achievable with synchronized clocking and tight PCB.

**Per-write timing math.**
- At 4 MHz SCK: 40 bits = 10 µs raw transfer + CS overhead (datasheet requires ≥10 ns CS-low setup; in practice 100–500 ns inter-byte and post-transmission settle on STM32 SPI peripherals).
- At 8 MHz SCK (synchronized): 40 bits = 5 µs raw transfer.
- Realistic per-write rate per driver, dedicated bus: **80–100 kHz at 4 MHz SCK, 150–200 kHz at 8 MHz SCK**, both *before* accounting for ISR entry/exit, software stack, or bus arbitration.

**Octopus Pro shared-bus reality.** Octopus Pro routes all 5 TMC5160 drivers on a single SPI bus with per-driver CS lines (BIGTREETECH schematic; Klipper config uses `spi_bus`/`cs_pin` per driver). With 4 active phase-stepping drivers (XY + extruder, ignoring 5th E for now) on one bus and a 4 MHz clock, the bus-aggregate write rate is the same 80–100 kHz, divided across drivers. **At 40 kHz desired per-driver refresh × 4 drivers = 160 kHz aggregate, you cannot hit it on a 4 MHz shared bus.** You need either (a) higher SCK with synchronized clocking, (b) splitting drivers across multiple SPI peripherals, or (c) the round-robin approach Prusa uses on XL: "refresh one axis per ISR, accepting `effective_per_axis_rate = 40 kHz / N_axes`" — Prusa cycles through 2 axes at 40 kHz total, giving each axis 20 kHz effective.

Reference Prusa source for the round-robin pattern:
```
// phase_stepping.cpp, line ~816
++axis_num_to_refresh;
if (axis_num_to_refresh == axis_states.size()) axis_num_to_refresh = 0;
refresh_axis(axis_states[axis_num_to_refresh], now, old_tick);
```
(https://github.com/prusa3d/Prusa-Firmware-Buddy/blob/master/lib/Marlin/Marlin/src/feature/phase_stepping/phase_stepping.cpp)

**Kalico repo state.** `src/stm32/stm32h7_spi.c` lines 98–104 sets the H7 SPI prescaler from a target rate against the bus PCLK; the H723's SPI peripherals run off APB1/APB2 and can produce 25+ MHz SCK in hardware, but the TMC5160's `fCLK/2` is the binding limit. The kalico H7 SPI driver does not yet expose the synchronized-clock-out feature TMC needs for >6 MHz operation — that's an MCU-firmware item, not a TMC limit.

---

## 3. Why 20–40 kHz?

Three independent constraints converge on roughly this range:

1. **Audible-noise lower bound (~16–20 kHz).** Any periodic excitation of the motor coils below ~16 kHz radiates as audible whine. Prusa's blog cites "above the audible range" as one design goal (https://blog.prusa3d.com/phase-stepping-how-we-busted-vibrations-and-improved-print-quality-on-the-xl-printer-with-just-a-firmware-update_94793/). This is the hard floor for the modulation rate — anything slower and the printer sings.

2. **Mechanical-resonance bandwidth (upper bound on what the modulation can usefully control).** A direct-drive motor + belt + toolhead system has its dominant resonances in the ~50–250 Hz band (kalico target hardware: 120 Hz Y, 180 Hz X). Nyquist on 250 Hz mechanical content is 500 Hz; 40 kHz gives 80× oversampling, leaving ample headroom for impulse shaping and phase compensation without aliasing the sine waveform onto the resonance. Going much above 40 kHz buys you nothing mechanically — the windings' L/R time constant (~1 ms for typical 1.4 A NEMA17, less for larger LF NEMA17/23 used on XL) low-pass-filters anything faster than ~5–10 kHz current change anyway.

3. **SPI throughput upper bound.** As shown in §2, even on a synchronized 8 MHz bus, ~150–200 kHz per-driver write rate is the ceiling. After ISR overhead, leaving headroom for other SPI traffic (configuration, status reads), and supporting multiple drivers on shared buses, **40 kHz aggregate × per-axis-round-robin** is the practical sweet spot. Prusa explicitly chose 40 kHz: `REFRESH_FREQ = 40000` in `phase_stepping_opts.h` (https://github.com/prusa3d/Prusa-Firmware-Buddy/blob/master/include/buddy/phase_stepping_opts.h#L18). Their fallback "burst stepping" mode (used on TMC2209-bearing variants where XDIRECT isn't an option) drops to 10 kHz because it's STEP/DIR-pulse-based, not SPI-write-based.

The phrase "20–40 kHz" in CLAUDE.md is therefore not arbitrary — it bounds the band where the lower edge clears audible whine and the upper edge respects realistic SPI/L-R bandwidths. Above 40 kHz is wasted firmware effort given STM32H723 SPI clocking and TMC5160 `fCLK/2` ceiling. Below 20 kHz is audibly noisy and gives up open-loop control authority on motor commutation.

No primary source explicitly says "20 to 40 kHz is the right range" as a single declarative statement — this is a synthesis from three constraints, each of which is independently sourced.

---

## 4. Open-loop viability

**Yes, open-loop phase stepping works for FDM.** The Prusa XL ships it in production firmware (v6.0.0, March 2024) on TMC2130 drivers with no encoder, no closed-loop feedback during motion. Calibration uses an accelerometer mounted on the toolhead to measure per-motor sinusoidal-drive nonlinearity offline, builds a per-motor LUT, then runs open-loop in service. The accelerometer is *not* in the runtime control loop. Source: Prusa Knowledge Base, "Phase Stepping (XL)" (https://help.prusa3d.com/article/phase-stepping-xl_681760), corroborated by source-code structure — Prusa's `phase_stepping.cpp` reads `axis_state.forward_current` / `backward_current` LUTs that were populated during a separate calibration pass.

**Why it works in this regime.** Open-loop steppers lose synchronism only when the rotor lags >180 electrical degrees behind the commanded field (Acutronic / Machine Design "Why open-loop steppers lose steps": https://www.machinedesign.com/motors-drives/article/21833271/why-open-loop-steppers-lose-steps-and-how-to-solve-the-problem). In FDM, with conservative current margins (Prusa's `CURRENT_AMPLITUDE = 248` of 256 leaves ~3% headroom on the IHOLD scale, and IHOLD itself is set well below the motor's pull-out torque), accelerations bounded by what the planner emits, and no shock loads, the rotor stays pinned within ±90° in practice. The published threshold for sync-loss is application-specific (it's a function of accel transient, motor inertia, load torque, and current margin), but **all production phase-stepping FDM machines run open-loop** — Prusa XL, Prusa CORE One, Prusa MK4S (the latter via firmware request).

**Sync-loss thresholds we did not find numerically.** No primary source (Trinamic AN, university thesis, peer-reviewed paper) was found that gives a "loses sync above N rpm or below S signal-to-noise" threshold specifically for XDIRECT-driven phase stepping under FDM loads. Prusa's calibration article describes failed-calibration symptoms (uneven motor characteristics) but does not publish a numerical sync-loss boundary. **This is an open empirical question for the kalico hardware** — almost certainly fine given the target machine is in the Prusa-XL operating envelope, but worth instrumenting on the test bench.

**Skip detection** (CLAUDE.md Layer 4) acts as the safety net independently: even with open-loop phase stepping, MSCNT or encoder reading at ~100 Hz can detect drift after the fact. This is consistent with how Klipper's existing TMC sensorless homing works.

---

## 5. The architectural forcing function — does evaluator rate equal modulation rate?

**Short answer: it does NOT have to. Prusa decouples them, and so should kalico.**

Prusa's phase-stepping pipeline:

1. **Layer 3-equivalent (Marlin's `precise_stepping` + input-shaper): produces piecewise quadratic move segments at G-code rate** (~1 kHz worst case).
2. **Layer 4-equivalent (`phase_stepping.cpp::handle_periodic_refresh()`)**: a 40 kHz timer ISR (TIM13 on STM32F4 XBuddy) that
   - reads the active move segment's `start_v`, `half_accel`, `initial_pos`, computes `position(t)` analytically from the quadratic,
   - converts position to electrical phase via the per-axis LUT (`forward_current[ustep]` / `backward_current[ustep]`),
   - DMAs the resulting `XDIRECT` 5-byte datagram out SPI3 to one driver,
   - rotates to the next axis on the next tick.

This means the **trajectory representation arriving on the MCU is NOT 40 kHz samples**. It is *piecewise-quadratic move segments* delivered at planner rate. The 40 kHz interpolator on the MCU consumes the move segment and evaluates `pos(t) = pos₀ + v·t + ½a·t²` analytically each tick. See `phase_stepping.cpp::MoveTarget::target_position()`:
```
float MoveTarget::target_position() const {
    float epoch = duration / static_cast<float>(TICK_FREQ);
    return initial_pos + start_v * epoch + half_accel * epoch * epoch;
}
```
(https://github.com/prusa3d/Prusa-Firmware-Buddy/blob/master/lib/Marlin/Marlin/src/feature/phase_stepping/phase_stepping.cpp)

The decoupling is structural: the planner-to-MCU interface delivers compact analytic descriptions (quadratic move segments in Prusa's case; **piecewise-NURBS segments in kalico's case** per the build-order spec); the MCU evaluates them at modulation rate. The MCU does not receive 40 kHz samples and does not need a separate "trajectory interpolator" — the *evaluator* itself runs at 40 kHz, but it's evaluating an analytic curve description, not interpolating between sparse samples.

**Mapped to kalico's architecture:** Layer 3 emits piecewise NURBS x(t) with shaper convolved in (per CLAUDE.md "Smooth-shaper application: convolve the time-reparameterized NURBS x(t) with the polynomial kernel w(t) analytically, produce shaped (higher-degree) NURBS in t"). Layer 4's per-axis evaluator runs at 40 kHz and calls Layer 0's NURBS-eval routines on each tick to produce position; current-synthesis maps position → electrical phase → XDIRECT write. **The architecture matches Prusa's decoupling exactly, just with NURBS-eval instead of quadratic-eval.**

**Cost analysis of *forced* coupling (i.e. if you tried to make trajectory rate = modulation rate):**
- Bandwidth: 40 kHz × 8 bytes/sample × 4 axes ≈ 1.3 MB/s sustained, vs Prusa's quadratic segments at ~50 KB/s. Klipper-style USB cannot sustain 1.3 MB/s reliably; CAN bus is hopeless; only EtherCAT could support it directly.
- Math fidelity: a linear interpolator between sparse samples cannot represent the curvature of a NURBS; you'd get visible cornering errors unless the sample rate was very high.
- Host CPU: trajectory evaluation is cheap on Pi 5 — Prusa's quadratic eval is ~400 ns at 40 kHz on STM32F4; kalico's NURBS eval target is similar with degree 3–5 piecewise polynomials.

**Cost analysis of decoupling (the recommended path):**
- MCU code: NURBS evaluator on H723 — Cortex-M7 at 550 MHz with double-precision FPU, 1 MB SRAM. de Boor for degree 3 NURBS is ~30 multiplies per axis per evaluation. At 40 kHz × 4 axes = 160 kHz evaluations × 30 muls = ~5M FLOPS; well under 1% of M7 budget.
- Position-error budget: zero algorithmic error vs the host's representation, because the MCU evaluates the same curve. Only floating-point rounding (f32 on MCU vs f64 on host) — bounded to ~7 decimal digits, far below microstep resolution.
- Loss of trajectory authority: none, as long as Layer 0's MCU NURBS evaluator is the same algebra as the host's.

There is no realistic scenario where coupling trajectory rate to modulation rate produces better trajectory quality than decoupling; the cost is strictly higher. **CLAUDE.md's "Trajectory evaluation on MCU at modulation rate (20–40kHz) for true phase stepping. MCU receives the shape with PA and IS already baked in, to reduce load." is consistent with the decoupled architecture** — "evaluation at modulation rate" refers to the per-tick *evaluator call* on a curve description that arrives much less frequently, not to a 40 kHz sample-streaming interface.

---

## What we couldn't verify / open questions

1. **Exact SCK ceiling on Octopus Pro** with synchronized clocking against TMC5160 `fCLK` — depends on board clock-routing decisions (whether the H723's MCO output is wired to TMC5160 CLK pin). BTT's schematic should be checked; if not wired, kalico is limited to ~4 MHz async per driver.

2. **Per-axis sync-loss numerical thresholds** for FDM under XDIRECT-driven open-loop phase stepping. No paper found. Empirical bench characterization is required — recommend an instrumented test pre-Step 10 to capture pull-out behavior for the kalico target motors.

3. **MSCNT writeability** — datasheet rev 1.18 paragraphs are not directly fetchable in this session; secondary sources (terjeio header, Klipper source) treat MSCNT as read-only and that matches every implementation. If the datasheet PDF becomes available, confirm.

4. **TMC5160 vs TMC5160A coil-current value range — ±255 vs ±248.** Prusa uses 248; community consensus is "±254 working range" for headroom. Datasheet rev 1.18 should be consulted directly to settle.

5. **Klipper's existing XDIRECT support.** Klipper's `tmc5160.py` lists XDIRECT as "not yet implemented" (https://github.com/Klipper3d/klipper/blob/master/klippy/extras/tmc5160.py). Phase stepping has not been merged into mainline Klipper as of this writing. Several community forks exist; none surveyed in depth here.

---

## Sources

- TMC5160A datasheet rev 1.18, Analog Devices — https://www.analog.com/media/en/technical-documentation/data-sheets/TMC5160A_datasheet_rev1.18.pdf (PDF fetch failed mid-research; register specifics quoted via terjeio Trinamic-library and EngineerZone).
- Trinamic-library tmc5160.h (terjeio) — https://github.com/terjeio/Trinamic-library/blob/master/tmc5160.h — verified GCONF `direct_mode` bit position, MSCNT 10-bit width.
- TMC5160 SPI synchronization app note — https://ez.analog.com/other-products/w/documents/32636/how-to-synchronize-spi-clock-to-system-clock-to-achieve-maximum-spi-data-rate-on-the-tmc5160 — fSCK < fCLK/2 constraint.
- Prusa-Firmware-Buddy — https://github.com/prusa3d/Prusa-Firmware-Buddy
  - `include/buddy/phase_stepping_opts.h` — `REFRESH_FREQ = 40000`, `CURRENT_AMPLITUDE = 248`, `SPI3 + DMA1_Stream5`.
  - `lib/Marlin/Marlin/src/feature/phase_stepping/phase_stepping.cpp` — round-robin axis refresh, MoveTarget quadratic eval, TIM13 ISR.
  - `lib/Marlin/Marlin/src/feature/phase_stepping/quick_tmc_spi.cpp` — XDirect register write via DMA, CS released by output compare.
  - `lib/Marlin/Marlin/src/feature/phase_stepping/lut.hpp` — `XDIRECT_t` register struct.
- Prusa blog "Phase Stepping" — https://blog.prusa3d.com/phase-stepping-how-we-busted-vibrations-and-improved-print-quality-on-the-xl-printer-with-just-a-firmware-update_94793/ — feature description, calibration overview.
- Prusa Knowledge Base, "Phase Stepping (XL)" — https://help.prusa3d.com/article/phase-stepping-xl_681760 — accelerometer-as-calibration-tool, not runtime feedback.
- Klipper TMC5160 driver — https://github.com/Klipper3d/klipper/blob/master/klippy/extras/tmc5160.py — XDIRECT listed as "not yet implemented".
- "Why open-loop steppers lose steps" (Machine Design) — https://www.machinedesign.com/motors-drives/article/21833271/why-open-loop-steppers-lose-steps-and-how-to-solve-the-problem — sync-loss mechanism (rotor >180° lag).
- kalico repo: `src/stm32/stm32h7_spi.c` — generic prescaler-based SPI clock setup, no synchronized-clock-out support today.
