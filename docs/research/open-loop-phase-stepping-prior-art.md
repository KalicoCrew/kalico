# Open-loop phase stepping in 3D-printer / CNC firmware — prior art (2026)

Scope: implementations that *modulate the stepper driver phase currents in software at high rate* (versus letting the chip's internal microstep table run from STEP/DIR pulses). Specifically directed at TMC5160-class drivers, since the user's Octopus Pro hardware is built around them. Consulted to inform Step 5 ("MCU framework with stub NURBS evaluator") of the Kalico-rewrite build order, where the trajectory→current pipeline architecture has to be designed before Step 10 (phase-stepping current synthesis) lands.

The companion document at [`firmware-survey.md`](firmware-survey.md) covers the broader 2026 firmware landscape — planner architectures, input shaping, junction deviation — and does not discuss phase stepping (zero hits for "phase step" in that file). This document fills that gap.

## TL;DR pattern synthesis

Surveying the field, **only two production-grade open-loop phase-stepping implementations exist** (Prusa Buddy, RepRapFirmware MB6HC), plus one community-rejected approach (mainline Klipper's hybrid anti-cogging sketch) and several closed-loop FOC alternatives (TMC4671, Duet 1HCL). Three architectural patterns emerge:

- **Pattern A — Direct evaluator at modulation rate, SPI XDIRECT.** Trajectory evaluator runs *at* the modulation rate (40 kHz for Prusa) inside a high-priority timer ISR; each tick computes electrical-angle, looks up A/B coil currents from a per-motor-calibrated LUT, writes XDIRECT register over a dedicated SPI bus via DMA. **Used by Prusa Buddy phase stepping (XL, Core One, MK4 family).** Requires a dedicated SPI line per ~3–4 motors and an MCU with both a hard-real-time timer ISR and DMA-ready SPI.
- **Pattern B — Burst stepping (high-rate STEP/DIR, no SPI current control).** Trajectory evaluator still runs at 10 kHz, but each tick emits a *burst of step pulses* sized to drive the chip's internal microstep counter (`MSCNT`) to the desired phase, deferring the actual coil-current synthesis to the chip's microstep table. **Used by Prusa Buddy phase stepping in `HAS_BURST_STEPPING` configurations** (`burst_stepper.cpp`/`.hpp`). Same trajectory pipeline as Pattern A, just a different output stage. Lets you avoid SPI bus saturation when many motors share one bus.
- **Pattern C — Decoupled rate (trajectory-rate planner + driver-side commutation).** Host plans positions at an arbitrary rate, MCU schedules conventional STEP pulses at `ceil(steps_per_second)`, the TMC chip handles sine commutation internally. This is **vanilla Klipper / RRF / Marlin / grblHAL**. There is *no* high-rate current modulation; per-motor calibration is impossible from this architecture. The mainline Klipper community has explicitly considered Pattern A and rejected it on bandwidth grounds (see "Annex/Klipper community findings" below).

**Implication for Kalico Step 5 brainstorm:** Pattern A and Pattern B are *the same trajectory architecture* with a swappable output stage. The architectural commitment that matters at Step 5 is "trajectory evaluator runs at modulation rate inside a hard-real-time ISR with a buffer of 2–3 adjacent segments." That commitment is identical for both A and B and lines up with the build-order's existing Step 5 description. The output-stage choice (SPI XDIRECT vs. burst STEP) can be deferred to Step 10. RRF 3.6's MB6HC implementation is the second data point confirming Pattern A is the load-bearing pattern for high-end open-loop phase stepping.

## Survey table

| Project | Status | MCU class | Modulation rate | Output stage | Trajectory↔current decoupling | Open/closed loop | Validation |
|---|---|---|---|---|---|---|---|
| **Prusa Buddy — phase stepping (Pattern A, default)** | Shipping (XL since FW 4.7, late 2023; MK4/MK4S, Core One, Mini in 2024–25) | STM32H723 / STM32G0 (XLBuddy) | **40 kHz** (`REFRESH_FREQ` constant in `phase_stepping_opts.h`) | TMC5160 `XDIRECT` over **SPI3 + DMA1_Stream5**, CS released by timer-OC for jitter-free latching | Direct: ISR computes 2nd-order trajectory + LUT lookup + SPI in 7 µs at 40 kHz; refreshes one axis per ISR (so each axis at ~20 kHz with 2 axes) | Open | Accelerometer + audible noise comparison; >96 % vibration-amplitude reduction claimed by Prusa (XL blog post) |
| **Prusa Buddy — burst stepping (Pattern B, conditional `HAS_BURST_STEPPING`)** | Shipping (Mini, MK3.5/MK3.9 lineage that lacks dedicated TMC SPI per motor) | STM32F4xx | **10 kHz** (same constant, `option::has_burst_stepping ? 10000 : 40000`) | STEP pulses via DMA2_Stream1, sized to drive `MSCNT` to the computed phase | Direct trajectory eval, but **chip does the sine commutation** | Open | Same calibration pipeline as Pattern A, lower vibration reduction headroom |
| **RepRapFirmware MB6HC — phase stepping (Pattern A)** | Shipping since RRF 3.6.0 (mid-2025), bug-fix releases through 3.6.2 | Atmel SAME70 (Cortex-M7 @ 300 MHz) | Not officially published; M970 enables, no rate G-code | TMC5160 SPI register writes (the MB6HC has TMC5160 onboard) | Direct (per object-model docs: `move.axes[].phaseStep == 1` reports the trajectory is being evaluated for phase output) | Open (closed-loop is a separate path, M569.1 for 1HCL boards) | Not publicly documented in detail |
| **Mainline Klipper** | **Not implemented**, explicitly rejected as a v1 architecture | — | — | — | — | — | Stepper-phase-aware *endstops* only (`Endstop_Phase.md`) — uses static phase value at homing, not dynamic phase modulation |
| **Kalico (KalicoCrew/kalico, the upstream we forked)** | **Not implemented** | — | — | — | — | — | Inherits Klipper's stepper-phase endstop only; no `phase_stepping`-style feature in `docs/Features.md` or `docs/Bleeding_Edge.md` as of April 2026 |
| **Marlin FT_MOTION** | Shipping | Cortex-M0/M3/M4 32-bit | trajectory rate ≈ 1–10 kHz (varies), STEP rate per-axis | STEP/DIR (Pattern C) — uses double-edge stepping if Trinamic | Decoupled: a "Stepper Events" buffer between planner and ISR; ISR is the dumb output stage, planner runs in main loop | Open, no per-coil current control | Print quality, audible noise |
| **grblHAL Trinamic plugin** | Shipping | various 32-bit | STEP rate (~50–200 kHz max) | STEP/DIR; SPI is config-only | Decoupled | Open | — |
| **Klipper TMC4671 driver (andrewmcgr/tmc-4671, also Ouroboros / Isik's Tech)** | Shipping (3rd party) | TMC4671 has its own onboard FOC engine; host MCU is RP2040/STM32 | FOC inside the chip at ~25 kHz | TMC4671 takes target *position*, runs internal FOC | **Position-target Pattern**: host pushes step→position, FOC chip closes the loop locally | **Closed loop** (encoder required) | Real prints; closed loop |
| **Duet 3 1HCL closed loop** | Shipping | SAMC21 + magnetic encoder | — | — | Stepper position computed as float full-steps in the loop, no STEP pulses | **Closed loop** | Real prints |
| **Mellow FlySSP / "Smooth Stepper Plus"** | **Could not find evidence such a product exists.** Mellow ships Fly-Super8Pro H723 (TMC5160 SPI mode supported, but only as standard interpolation; no phase-stepping feature in their docs) | — | — | — | — | — | — |
| **Annex Engineering** | Uses Kalico, no phase-stepping firmware. The community Discord historically hosted early Klipper-on-stepper-phase discussions but produced no shipping firmware fork | — | — | — | — | — | — |
| **Daksh Adhar / GalvoFW** | **Could not find evidence of phase-stepping firmware.** GalvoStep / OPAL projects use steppers for galvo control but do not modulate driver phase currents at high rate | — | — | — | — | — | — |

Codebase paths quoted directly:

- Prusa Buddy `include/buddy/phase_stepping_opts.h`: defines `REFRESH_FREQ = option::has_burst_stepping ? 10000 : 40000`, `MOTOR_PERIOD = 1024`, `SIN_FRACTION = 4`, `CORRECTION_HARMONICS = 16`, `SUPPORTED_AXIS_COUNT = 2`. The timer is `TIM8_UP_TIM13_IRQHandler`. SPI is hardcoded to `SPI3` + `DMA1_Stream5`.
- Prusa Buddy `lib/Marlin/Marlin/src/feature/phase_stepping/quick_tmc_spi.hpp`: explicitly states *"This module has one responsibility: set XDirect register for given TMC driver as quick as possible with as little CPU intervention as possible. To do so we perform minimalistic and direct setup of the peripherals (SPI & DMA). We avoid any interrupts. Also, to ensure precise timing with minimal jitter, the CS line for 2130 is released by output compare of the main phase stepping timer."*
- Prusa Buddy `phase_stepping.cpp` `handle_periodic_refresh()` documents the 7 µs ISR budget: *"time + move advancement handling: 970 ns (happy path); position computation: 1.9 µs; post to phase: 1.3 µs; current lookup: 800 ns; Quick transmission: 900 ns + 1 µs transaction termination."*
- Prusa Buddy `phase_stepping.hpp` `AxisState` carries a `pending_targets` ring of 32 `MoveTarget`s, with a `StealableTarget` slot for the upcoming move (atomic-handoff between the slow-stepping ISR and the phase-stepping ISR). This is the **2–3-segment buffer** the Kalico build-order explicitly calls for in Step 5.

## Failure modes — published evidence

I found **three categories** of documented failure:

1. **Calibration tied to fixed speed.** [Prusa Buddy issue #3808](https://github.com/prusa3d/Prusa-Firmware-Buddy/issues/3808): user reports phase stepping eliminates VFAs at fast-perimeter speed but they reappear at slow external-perimeter speed — the per-motor LUT was identified at one speed and back-EMF distorts the open-loop sine wave at others. Prusa's response was the universal-calibration sweep (multiple speeds) that arrived in FW 6.4 and was refined in 6.4.1.
2. **Calibration regression after FW changes.** [Buddy issues #4726, #4973, #5146, #5149](https://github.com/prusa3d/Prusa-Firmware-Buddy/issues/4726) — the FW 6.4 change to a "universal" calibration broke calibration on some XL units (frequency sweep failed to find peaks, calibration produced inconsistent results). Forced backports of fixes from 6.5 into 6.4.1. Lesson: the per-machine LUT is sensitive to mechanical resonance frequency and vibration coupling; a "one calibration to rule them all" approach can regress quietly.
3. **Bandwidth ceiling — not a failure mode in deployment, but a *forecasted* one.** dmbutyugin (Klipper Discourse, [post #14 of "Motion analysis by stepper phase"](https://klipper.discourse.group/t/motion-analysis-by-stepper-phase/1876?page=2)): *"this will substantially increase the traffic between host and MCU, which will rule out serial connections and perhaps CAN bus."* And: *"at very high velocities the sine wave through the coils gets severely distorted due to back-EMF... so at those speeds it does not make sense to do any phase adjustments — the actual current won't follow them anyways."*

I found **no documented evidence of**:
- Sync loss / step skip *caused by* open-loop phase stepping (it actually appears to *reduce* skip risk in calibrated operation, since vibration coupling is reduced).
- Thermal drift in the LUT outside of motor-replacement scenarios. Prusa explicitly says recalibration is needed only on motor swap; Buddy FW does not auto-recalibrate.
- Microstep aliasing artifacts.

Empty here is itself a finding: the calibration is mechanical-resonance-tied, not thermal-tied, and the failure modes that *do* exist are calibration-pipeline regressions, not in-print runtime failures.

## Pattern synthesis (the headline finding)

### Pattern A — Direct evaluator at modulation rate, SPI XDIRECT (Prusa default, RRF MB6HC)

```
[planner host] --segments--> [MCU buffer of 2–3 segments]
                                  |
                                  v
                          [40 kHz timer ISR]
                                  |
                       trajectory eval at this rate
                                  |
                       physical_position(t) -> rotor_phase(angle)
                                  |
                       LUT(angle, direction) -> (I_a, I_b)
                                  |
                       set_xdirect(axis, currents) via DMA-SPI
```

- Requires: hard-real-time timer ISR, DMA-driven SPI bus per ~3–4 motors, ~7–10 µs of ISR budget per axis at 40 kHz.
- Cost: SPI bandwidth scales with N_motors × modulation_rate × 40-bit-frame.
- Reward: full per-motor harmonic correction (16 harmonics in Prusa); >96 % vibration reduction reported.

### Pattern B — Burst stepping (Prusa fallback)

Same trajectory eval, same rate. Only difference: instead of writing XDIRECT, compute *step delta* between current `MSCNT` driver phase and desired phase, fire that many STEP pulses via DMA in the next 100 µs window. Chip's internal sine table does the actual coil current synthesis. **Buys you per-motor LUT compensation in the *position* domain (LUT shifts the target electrical angle) without per-motor compensation in the *current waveform* domain.** Sufficient on cheaper boards without dedicated SPI per stepper.

Direct quote from `phase_stepping_opts.h`:
```
static constexpr int REFRESH_FREQ = option::has_burst_stepping ? 10000 : 40000;
```

### Pattern C — Decoupled (mainline Klipper / RRF non-MB6HC / Marlin)

Host plans high-level moves; MCU emits step pulses on traditional `step_compress`/STEP-DIR scheduling; the chip handles commutation. **No per-motor LUT compensation possible** since you never address rotor phase directly — you only address microstep count. This is what the rest of the firmware ecosystem does, and it's why Klipper's mainline still ships only stepper-phase-aware *endstops* (a static read at homing) and not phase-stepping current modulation.

### Pattern D-ish — Closed-loop FOC (TMC4671 / Duet 1HCL)

For completeness: with a TMC4671 chip the host pushes a position target, the chip closes its own current loop with encoder feedback at ~25 kHz inside the chip. The host MCU rate is decoupled. This is structurally a *different* problem — it's closed-loop and requires encoders — and not a substitute for open-loop phase stepping unless you accept the encoder cost.

## Annex / Klipper community findings

The Klipper Discourse thread [Motion analysis by stepper phase](https://klipper.discourse.group/t/motion-analysis-by-stepper-phase/1876) (May 2024 onward, posts 8–18) is the canonical record of mainline Klipper considering and *rejecting* Pattern A.

Key quotes:

- **dmbutyugin (post #14):** *"this will substantially increase the traffic between host and MCU, which will rule out serial connections and perhaps CAN bus."* — the architectural objection. Mainline Klipper's commitment to USB/CAN as the host↔MCU transport doesn't have headroom for 40 kHz × N-axis × ~50-byte frames.
- **dmbutyugin (post #14):** *"at very high velocities the sine wave through the coils gets severely distorted due to back-EMF... at those speeds it does not make sense to do any phase adjustments — the actual current won't follow them anyways."* — the physics objection. Phase-stepping correction works at low and mid speeds; it stops mattering above ~half rated speed because the L/R electrical time constant of the motor + back-EMF eat the commanded current waveform.
- **koconnor (post #16):** *"it may be possible for the host to use 'anti-cogging step scheduling' at slow speeds, and use regular step scheduling at medium/high speeds"* — the proposed Klipper-shaped compromise: keep Pattern C as the default, fold Pattern-A-like LUT correction into the *step time scheduling* at low speeds only. As far as I can tell, this has not been implemented.
- **dmbutyugin (alternative direction, post #14):** *"propose modifying trajectory kinematics with sinusoidal position corrections, avoiding hardware-specific implementations."* — i.e., do anti-cogging in the position domain at the trajectory level, not in the current domain at the driver level.

The Annex Engineering Discord, where some early phase-stepping prototypes were rumored, has produced **no public firmware fork** with phase stepping — Annex's K3 (Gasherbrum) printer ships on Kalico stock and the only motion-related work I could find on their GitHub is the [`Belay`](https://github.com/Annex-Engineering/Belay) sync sensor module. So the Discord conversations did not become production code.

## Caveats

- The two production implementations (Prusa Buddy, RRF MB6HC) use *different MCUs* — STM32H723 / STM32G0 vs. SAME70 — but both Cortex-M7 class. Pattern A appears to require Cortex-M7-class headroom; I have no evidence of it being made to work on F4-class hardware *without* burst-stepping fallback.
- Prusa's per-motor LUT is identified by an *accelerometer-driven calibration sweep at print start*. The user's spec already calls for accelerometer-mounted calibration of the rotor angle LUT for phase stepping (see "Nice to have" in CLAUDE.md), which lines up with Prusa's approach.
- The 96 % vibration-reduction figure from Prusa is per-motor amplitude in calibration; it is *not* a 96 % print-quality improvement and Prusa explicitly says noise reduction is much smaller.
- The mainline Klipper bandwidth objection assumes the existing Klipper transport. The Kalico-rewrite spec's "Real time communication with MCUs, no queue-based offload" already breaks with that assumption, and "Trajectory evaluation on MCU at modulation rate (20-40kHz)" already commits to Pattern A. So the Klipper community's bandwidth rejection does *not* apply to the Kalico rewrite — it applies to the *retrofit* of phase stepping onto Klipper's queue-based transport.
- I could not confirm the existence of "Mellow FlySSP / Smooth Stepper Plus." The closest matches were the Fly-Super8Pro H723 (which uses TMC5160 SPI but only for config) and Mach3-era SmoothStepper hardware (irrelevant). If this product exists it's either unreleased or named differently.
- I did not find a Daksh Adhar phase-stepping repo. GalvoStep / OPAL are stepper-driven galvo projects, not phase-stepping firmware.

## Sources

- [Prusa Buddy `phase_stepping_opts.h`](https://github.com/prusa3d/Prusa-Firmware-Buddy/blob/master/include/buddy/phase_stepping_opts.h) — REFRESH_FREQ, SPI3, DMA1_Stream5, MOTOR_PERIOD constants
- [Prusa Buddy `quick_tmc_spi.hpp`](https://github.com/prusa3d/Prusa-Firmware-Buddy/blob/master/lib/Marlin/Marlin/src/feature/phase_stepping/quick_tmc_spi.hpp) — XDIRECT contract and CS-via-timer-output-compare design
- [Prusa Buddy `phase_stepping.cpp`](https://github.com/prusa3d/Prusa-Firmware-Buddy/blob/master/lib/Marlin/Marlin/src/feature/phase_stepping/phase_stepping.cpp) — `handle_periodic_refresh()` ISR with documented timing budget
- [Prusa Buddy `phase_stepping.hpp`](https://github.com/prusa3d/Prusa-Firmware-Buddy/blob/master/lib/Marlin/Marlin/src/feature/phase_stepping/phase_stepping.hpp) — `AxisState`, `MoveTarget`, `StealableTarget` 2nd-order trajectory buffer
- [Prusa Buddy `burst_stepper.hpp`](https://github.com/prusa3d/Prusa-Firmware-Buddy/blob/master/lib/Marlin/Marlin/src/feature/phase_stepping/burst_stepper.hpp) — Pattern B
- [Prusa blog — Phase Stepping (XL)](https://blog.prusa3d.com/phase-stepping-how-we-busted-vibrations-and-improved-print-quality-on-the-xl-printer-with-just-a-firmware-update_94793/)
- [Prusa Knowledge Base — Phase Stepping (XL)](https://help.prusa3d.com/article/phase-stepping-xl_681760)
- [Prusa Knowledge Base — Phase Stepping (Core One)](https://help.prusa3d.com/article/phase-stepping-core-one-l-core-one_914247)
- [Prusa Buddy issues #3808, #4461, #4664, #4726, #4973, #5146, #5149](https://github.com/prusa3d/Prusa-Firmware-Buddy/issues/3808) — calibration regression and slow-speed VFA failure modes
- [Klipper Discourse — Motion analysis by stepper phase, posts 8–18](https://klipper.discourse.group/t/motion-analysis-by-stepper-phase/1876) — koconnor and dmbutyugin on bandwidth, back-EMF, and the proposed slow-speed-only hybrid
- [RepRapFirmware Changelog 3.x](https://github.com/Duet3D/RepRapFirmware/wiki/Changelog-RRF-3.x) — phase stepping introduced in RRF 3.6.0 on Duet 3 MB6HC, M970 enable, refined in 3.6.1/3.6.2
- [Marlin FT_MOTION docs](https://marlinfw.org/docs/features/ft_motion.html) — Pattern C example
- [andrewmcgr/tmc-4671](https://github.com/andrewmcgr/tmc-4671) and [Ouroboros / Isik's Tech](https://store.isiks.tech/products/ouroboros) — TMC4671 closed-loop alternative
- [Duet 3 1HCL closed loop](https://docs.duet3d.com/Duet3D_hardware/Duet_3_family/Duet_3_Expansion_1HCL) — closed-loop magnetic-encoder alternative
- [Kalico `docs/Features.md`](https://github.com/KalicoCrew/kalico/blob/main/docs/Features.md) and [`docs/Bleeding_Edge.md`](https://github.com/KalicoCrew/kalico/blob/main/docs/Bleeding_Edge.md) — confirmed: no phase-stepping current modulation in upstream Kalico
- [Annex-Engineering GitHub](https://github.com/Annex-Engineering) — confirmed: no phase-stepping firmware fork
- [Mellow Fly-Super8Pro H723 docs](https://mellow-3d.github.io/fly_super8_pro_h723_general.html) — TMC5160 SPI is config-only
