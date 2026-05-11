# Investigation: "First motion only energizes, doesn't move" — `sota-motion` 2026-05-11

## Symptom

On a fresh klippy attach, the first jog button press energizes the
steppers (motors hold position with current) but generates zero step
pulses. Subsequent jogs work normally. Reproducible on every restart.

## What the host-side bench-repro harness shows

The host-side harness in `rust/motion-bridge/tests/bench_repro.rs` drives
the production `PlannerHandle` → `ShapedSegment` → `runtime::Engine` path
end-to-end on the host:

- `first_jog_after_stream_open_emits_step_pulses` — **PASSES** (motor A and
  motor B both see hundreds of step pulses on the very first 10 mm X jog
  after `kalico_stream_open`).
- `consecutive_short_jogs_produce_consistent_motion` — **PASSES** (10 × 5 mm
  jogs all produce consistent motor-A step counts).
- `no_step_burst_violations_under_rapid_jogs` — **PASSES** (20 rapid jogs
  through the engine with no latched fault).

**Conclusion: the planner + Layer-4 engine pipeline is not the source of
the first-motion-no-movement symptom.** The bug must live in the
host↔firmware boundary or the firmware-side wire processing.

## Hypothesis — WidenState undercounts wraps on first push

**The H723 firmware never calls `runtime_handle_seed_widen`.** It's only
wired in `src/linux/runtime_tick_host.c:153`. On real hardware the
firmware boots, klippy's clock-sync starts ticking (giving the host an
accurate widened MCU clock that includes accumulated 2³² wraps), but the
firmware's `WidenState` stays at `high = 0` until the **first ISR tick**
publishes a widened sample.

### The first-push timing race

In `rust/kalico-c-api/src/runtime_ffi.rs::push_segment_impl` (lines
340–366), on the very first push from a fresh boot:

1. Foreground sees `cur_status == Idle`, takes the re-enable branch.
2. Reads `last_widened = runtime::clock::read_widened_now(shared)`.
   On a fresh boot, no ISR has run yet, so `widened_now_lo/hi == 0` and
   **`last_widened == 0`**.
3. Reads the live `raw_cyccnt` (some large u32, e.g. ~2 × 10⁹ if the
   firmware has been up ~4 s since boot).
4. Calls `widen_state.reinit(raw, 0)` → `WidenState { high: 0, last_low: raw }`.
5. Re-enables TIM5.

Meanwhile, the host's bridge dispatch (`rust/motion-bridge/src/bridge.rs`,
lines 1520–1551) computes the segment's `t_start` from:

```rust
let now_clock = router.compute_ack_clock(mcu_h)?;
let lead_cycles = (freq * 0.250).round() as u64;
let mcu_base_clock = max(prev_tail, now_clock + lead_cycles);
let rel_start = (seg.t_start * freq).round().max(0.0) as u64;
let t_start_clock = mcu_base_clock + rel_start;
```

`compute_ack_clock` returns a **widened u64 in MCU-clock space** that
already includes any 2³² wraps that have elapsed since boot (clock-sync
on the host tracks these). After ~4 s at 520 MHz, the firmware has wrapped
CYCCNT roughly once (CYCCNT wraps every 2³²/520M ≈ 8.26 s), so this could
be ~2 × 10⁹ pre-wrap or ~6.3 × 10⁹ post-wrap. Pick the post-wrap case.

Now the engine ticks. ISR receives a fresh `raw_cyccnt` (say ~2 × 10⁹
just after the wrap). `widen_state.widen(raw)` runs with
`high = 0, last_low = ~2 × 10⁹`. Since `raw >= last_low`, no wrap is
detected, so `widen` returns `0 | raw = ~2 × 10⁹`.

The Engine then checks: `t_segment = now.saturating_sub(t_start)`
(`rust/runtime/src/engine.rs::tick_with_current`, line 525). With
`now ≈ 2 × 10⁹` and `t_start ≈ 6.3 × 10⁹ + 0.25s_in_clocks`, we get
`t_segment = 0` (saturating). The engine sits on the segment, status
`Running`, but the curve is evaluated at `u = 0`, so every tick samples
the same starting position. **Step accumulator deltas are zero → zero
step pulses emitted.**

The engine only "catches up" once `widen` observes a CYCCNT wrap and
bumps `high` to `1 << 32`. That takes roughly 4 s (~half the wrap
period) from the first ISR tick. Once `high` matches the host's view,
the segment finally activates and step pulses fire.

By that point the user has either pressed the jog button again (resetting
the perceived problem to "intermittent") or given up. The "intermittent
no-motion" symptom is the **same bug** observed at a different boot phase
— if the firmware was just past a CYCCNT wrap when the first push lands,
the misalignment is smaller and the catch-up faster, so motion eventually
materializes.

## Code citations

- `rust/runtime/src/clock.rs:46-58` — `WidenState::seed_high` /
  `WidenState::reinit`. `seed_high` is the documented mechanism for
  bridging the boot-time wrap-count gap.
- `rust/runtime/src/clock.rs:60-69` — `WidenState::widen` is monotonic but
  cannot recover past wraps without an external seed.
- `rust/kalico-c-api/src/runtime_ffi.rs:866-868` — `runtime_handle_seed_widen`
  FFI exists.
- `src/linux/runtime_tick_host.c:153` — only call site is the Linux sim.
  **No H723 / F446 firmware caller.**
- `rust/kalico-c-api/src/runtime_ffi.rs:340-366` — `push_segment_impl`
  re-enable branch reads `last_widened = read_widened_now(shared)` which
  is `0` on first push from a fresh boot.
- `rust/motion-bridge/src/bridge.rs:1520-1551` — `compute_ack_clock`-based
  `t_start_clock` is widened from boot, while the firmware's first widen
  return is not. This is the misalignment.
- `rust/runtime/src/engine.rs:525` — `t_segment = now.saturating_sub(current.t_start)`
  silently saturates at zero when `now < t_start`. No diagnostic; the
  engine looks healthy from the foreground's perspective.

## Why bench-repro doesn't catch it

The host-side `run_segments_through_engine` helper in
`tests/bench_repro.rs` shifts every segment so the first one starts at
clock 0 (`let t_offset = segs[0].t_start;` … `t_start_clock = (rel_start_s * CLOCK_FREQ) as u64`).
It also constructs a fresh `WidenState::default()` (`high=0, last_low=0`)
and immediately ticks from `now = 0` upward, so widening starts from a
known-aligned state.

In the real wire-format the host stamps segments with a u64 derived from
**clock-sync's widened view of the MCU**, but the firmware's widen state
starts uncalibrated. There is no host-side test that exercises this
mismatch — `streaming_replan.rs` ends at the `ShapedSegment` boundary;
`sim_motion.rs` doesn't compile on `sota-motion` HEAD; the runtime
crate's own `engine_tick.rs` tests use small absolute clock values that
never exceed 2³².

## Recommended next-step fix targets, ranked

### 1. Seed WidenState from clock-sync on H7/F4 (high impact, low risk)

Add a call to `runtime_handle_seed_widen` in the H7 + F4 init paths,
right after `runtime_handle_init` completes and before TIM5 is first
enabled. The host already publishes the baseline through the clock-sync
handshake; a single u64 read of `timer_read_time() | (high << 32)` (with
`high` carried in foreground state since boot) is the canonical fix.

**Risk**: low. The Linux sim has been using this path for ~6 months
without issue. The H7 case is structurally identical — there's nothing
H7-specific about a u64 baseline write to `WidenState.high`.

**Where**: `src/stm32/runtime_tick_h7.c` runtime init, before
`runtime_tick_enable()`. Mirror the Linux pattern in
`src/linux/runtime_tick_host.c:148-156`.

### 2. Feedrate plumbing into TOPP-RA v_max (high impact, medium risk)

`PlannerLimits.max_velocity` flows through to `Limits.v_max`, but the
per-move `CubicSegment.feedrate_mm_s` is captured at classify time and
**never consulted by the temporal layer** (`rust/temporal/`). Bench
trace confirms: every 25 mm @ feedrate=100 jog runs at peak ≈ 330 mm/s.
Test 2 (`feedrate_caps_trajectory_velocity`) reproduces with peak
v = 284 mm/s for feedrate=50.

**Fix shape**: per-segment v_max passed into TOPP-RA as
`min(PlannerLimits.max_velocity, segment.feedrate_mm_s)` for each
trajectory direction. The temporal `Limits` struct currently carries
machine-axis-frame caps; we'd add a per-segment scalar that scales the
caps inside the joining loop.

**Risk**: medium. The β-medium outer iteration assumes v_max is global,
not per-segment. Need to check whether the iteration converges with a
tighter per-segment cap. The streaming-replan layer also makes
assumptions about v_max being constant across the lookahead window
(`emit_committed`'s held-back decel boundary).

### 3. Investigate mid-segment velocity discontinuity (high impact, unclear risk)

Test 4 (`single_segment_has_monotone_velocity_profile`) detects a
281 765 mm/s² mid-segment acceleration spike on a single 25 mm @ feed=100
jog. That's ~4 × max_accel (70 000 mm/s²), well outside any β-medium
shaper-aware derate margin. Either:
- The trajectory crate is emitting a curve with a non-physical
  acceleration profile (β-medium underconverging, peak-accel-finder
  missing an internal peak), or
- The C¹ Hermite refit is introducing the spike at a seam between
  internal pieces, and the shaper convolution is amplifying it.

**Why this is bigger than it looks**: peak v at the spike is ~290 mm/s,
which on the host trace is suspiciously close to the bench's observed
~330 mm/s peak on 25 mm @ feed=100. Both numbers are ~3 × the commanded
feedrate. If feedrate were respected (fix #2), the velocity spike's
magnitude would drop proportionally — but the **discontinuity at this
moment in time** would still exist, just at a smaller absolute scale.

**Action**: print the per-piece breakpoints of the shaped X NURBS at the
spike location. The trace already captures duration; cross-reference the
spike's sample index (62/250 ≈ 0.248) against the post-shape NURBS knot
vector for the segment.

### 4. step-burst threshold + bench rapid-jog soak (medium impact, low risk)

Test 5 passes on the host. If it fails on hardware it's because the
real MCU runs at 520 MHz and faster wall-clock progression compounds
the WidenState misalignment from fix #1 with the velocity-profile bug
from fix #3. After fixes #1 + #3 land, re-run rapid_jogs on hardware as
a soak.

## What the live trace already tells us

The bench-side `[planner-trace]` lines confirm the planner is doing its
job:

```
Move dist=25.000mm feed=100.0 nominal_s=0.250000 replan_us=70564 ...
  drained=1 drained_dur_s=0.063488 ...
T_commit fire since_arm_us=50059
commit drained=1 dur_s=0.074300 ...
```

Total duration: 0.0635 + 0.0743 = 0.138 s. For 25 mm, that's an average
velocity of 181 mm/s; given the trapezoid shape, peak is ~330 mm/s. The
planner ships the right curve for the limits it was given — the limits
themselves are wrong because feedrate is being ignored.

The trace **does not** record what the host computed for `t_start_clock`
or what the firmware-side first widen produced. Adding those diagnostics
to the bridge dispatch + the FFI's push_segment_impl would directly
confirm fix #1's hypothesis on hardware. Both are one `eprintln!` away
in their respective files.
