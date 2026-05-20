# Stepping redesign — bench bring-up implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Drive the bench printer (`dderg@trident.local`, BTT Octopus Pro H723 + Octopus F446) with the new stepping-redesign firmware path (Tasks 1-18 from `docs/superpowers/plans/2026-05-19-stepping-redesign-implementation.md`), walking through Stages 1-7 from `docs/superpowers/specs/2026-05-19-stepping-redesign-design.md` §"Bench bring-up stages".

**Architecture:** Firmware HEAD `747e66717` boots, enumerates, accepts host commands, and pushes/retires segments via the **legacy** push_segment protocol — but no GPIO step pulses fire. Reason: the new TIM5 ISR body (`kalico_runtime_tick_sample`) only produces motion if `AxisConfig.piece` is populated; the legacy `push_segment` path doesn't populate it. We will (a) keep the legacy path linkable but switch the bench to the new path by wiring host-side glue, then (b) walk each stage. Each stage is small enough to flash once, observe, fault-find, and iterate.

**Tech stack:** Klippy (Python) + motion_bridge (Rust cdylib loaded into klippy) + kalico-host-rt (Rust transport) + firmware (C+Rust). FFI surface: existing `kalico_runtime_*` from `rust/kalico-c-api/src/runtime_ffi.rs` + the new Task 7-17 entries (`kalico_runtime_configure_axis`, `_configure_kinematics`, `_configure_pressure_advance`, `_set_axis_mode`, `_set_stepper_offset`, `_tick_sample`).

**Bench prerequisites** (per memory `reference_flash_h7`):
- Pi: `dderg@trident.local`, repo at `~/klipper` (origin `dderg/kalico`)
- H7 MCU serial: `/dev/serial/by-id/usb-Klipper_stm32h723xx_490017000851323235363233-if00`
- F4 MCU serial: `/dev/serial/by-id/usb-Klipper_stm32f446xx_2C0036000851313133353932-if00`
- H7 build config: `.config.h7.bak` (USBSERIAL=y, USB=y, STACK_SIZE=8192, KALICO_SIM=n)
- Flash: `make flash FLASH_DEVICE=0483:df11` from DFU (BOOT0+RESET on H7)
- Iteration flow (CLAUDE.md): commit → push → pull → compile on Pi → flash

**Out of scope:**
- F4 (bottom) firmware is unchanged by this plan. Z stepper lives on F4 and uses the legacy step-time path.
- klipper-sim integration test (Task 15 of the firmware plan) — that's offline validation, not bench.
- TMC5160 register pre-config (chopconf, stallguard) — host-side responsibility, treated as already-correct by Stage 5.

---

## Architectural decision: keep both paths in firmware, route per-axis

The firmware currently has BOTH the legacy stepping path (`producer_step` / `step_time_event` / per-stepper Klipper timers — addressed by Task 16 of the firmware plan but deferred) and the new path (`tick_sample` / `dispatch_axis` / per-axis SPSC + Klipper timer). The cleanest bench-bring-up choice:

- **X / Y / E axes (on H7)** → run on the **new path** (`kalico_runtime_tick_sample`). This is what we want to validate.
- **Z axis (on F4)** → leave alone. Z uses `step_modes[2] = 1` (StepTime) via the legacy stepper timer. F4 firmware has none of the new code paths (its build excludes them via target features).

This means host-side wiring needs to send `kalico_configure_axis(axis_idx, mode, ...)` for X/Y/E on the H7, and NOT for Z (or send mode=Pulse and don't install per-axis timer for Z). The TIM5 ISR + per-axis timers run on H7 only.

---

## File map

**Modify:**
- `rust/runtime/src/engine.rs` — add `push_piece_for_axis(axis_idx, piece)` method that populates `stepping_axes[axis_idx].piece` from a cubic Bezier
- `rust/kalico-c-api/src/runtime_ffi.rs` — add `kalico_runtime_push_piece` FFI entry
- `rust/kalico-c-api/include/kalico_runtime.h` — declare the new FFI
- `src/stepping_commands.c` (new) OR `src/stepper.c` — add `DECL_COMMAND(command_kalico_push_piece, ...)`
- `src/runtime_tick.c` — call `init_per_axis_step_timers()` from `runtime_init` so the new per-axis Klipper timers actually fire
- `src/stm32/runtime_tick_h7.c` — unconditionally enable TIM5 when at least one axis is configured (replace `count_modulated_steppers == 0` short-circuit with `count_configured_axes == 0`)
- `klippy/motion_toolhead.py` — at startup, after `configure_axes` (existing call), also send `kalico_configure_axis(axis_idx, mode=Pulse, ...)` for X/Y/E on H7 + `kalico_configure_kinematics(k_xy)` + `kalico_configure_pressure_advance(0,0)`. Convert per-segment G1/G5 motion into cubic Bezier pieces and emit `kalico_push_piece(axis_idx, bp[4], t_start_cycles, duration_sec)` for each axis.
- `klippy/motion_bridge.py` — Python wrapper for the new piece-push FFI

**No deletions** — the legacy path stays linkable for Z and as a fallback during bisect.

---

## Tasks

### Task 0: Verify current baseline + cleanup diag

**Files:**
- Modify: `src/runtime_tick.c` (revert the 0xB0..0xB5 diag markers from commits `1735799ee`/`747e66717`)

- [ ] **Step 1: Confirm current bench state**

Run on Pi: `ssh dderg@trident.local 'systemctl is-active klipper && curl -s http://localhost:7125/printer/info | head'`
Expected: klipper active, printer info returned.

- [ ] **Step 2: Pull current klippy log + filter for engine_status, faults**

`ssh dderg@trident.local 'tail -200 ~/printer_data/logs/klippy.log | grep -E "engine_status|last_fault.*[1-9]|Shutdown|Traceback" | tail -20'`

Confirm: `engine_status: 0` (idle), `last_fault: 0`, no shutdowns. This is the "no GPIO motion, segments retiring instantly" baseline we'll improve from.

- [ ] **Step 3: Remove the 0xB0..0xB5 diag markers**

In `src/runtime_tick.c`, delete the marker calls (`runtime_diag_progress(0xB0, 0, 0)` etc.) and the `RUNTIME_INIT_STUB` block. Restore the original `runtime_init` shape, keeping only the prior-magic capture and the substantive work.

- [ ] **Step 4: Commit**

```
git add src/runtime_tick.c
git commit -m "diag(runtime_init): remove 0xB0..0xB5 bisect markers (crash root-caused to USB config)"
```

- [ ] **Step 5: Push + rebuild on Pi**

```
git push origin sota-motion
ssh dderg@trident.local 'cd ~/klipper && git pull && make -j$(nproc) 2>&1 | tail -5'
```

Expected: clean build, ROM ~53%, AXI ~99%.

- [ ] **Step 6: Verify NO re-flash needed**

We can skip re-flashing for this commit since it's diagnostic-only and the markers were harmless. Flash only when needed by subsequent tasks (BOOT0+RESET costs the user effort).

---

### Task 1: New FFI — kalico_runtime_push_piece

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs`
- Modify: `rust/kalico-c-api/include/kalico_runtime.h`
- Modify: `rust/runtime/src/engine.rs`

- [ ] **Step 1: Write the engine method**

In `rust/runtime/src/engine.rs`, add to the `impl<P, I> Engine<P, I>` block (the one with `configure_axis`):

```rust
/// Push a cubic Bezier piece onto the given axis.
///
/// Spec §"State" / "TIM5 ISR — the unified evaluator": axis.piece holds the
/// active polynomial that dispatch_axis evaluates per sample. The host
/// computes Bernstein control points + piece_start_time + duration once
/// per logical move per axis and pushes them here; the TIM5 ISR consumes.
///
/// Returns 0 on success, -1 on invalid args.
pub fn push_piece_for_axis(
    &mut self,
    axis_idx: u8,
    bernstein: [f32; 4],
    piece_start_time_cycles: u64,
    duration_sec: f32,
) -> i32 {
    if (axis_idx as usize) >= crate::stepping_state::N_AXES {
        return -1;
    }
    if !duration_sec.is_finite() || duration_sec <= 0.0 {
        return -1;
    }
    if !bernstein.iter().all(|x| x.is_finite()) {
        return -1;
    }
    let mut piece = crate::monomial::bernstein_to_monomial(bernstein);
    piece.duration = duration_sec;
    let axis = &mut self.stepping_axes[axis_idx as usize];
    axis.piece = Some(piece);
    axis.piece_start_time_cycles = piece_start_time_cycles;
    0
}
```

- [ ] **Step 2: Write the FFI entry point**

In `rust/kalico-c-api/src/runtime_ffi.rs`, near the other `kalico_runtime_configure_*` entries:

```rust
/// Push a cubic Bezier piece onto an axis. Wire format: Bernstein control
/// points as f32-as-u32-bits, t_start as u64 cycles, duration as f32 seconds.
///
/// See `runtime::engine::Engine::push_piece_for_axis`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_push_piece(
    rt: *mut KalicoRuntime,
    axis_idx: u8,
    bp0_bits: u32, bp1_bits: u32, bp2_bits: u32, bp3_bits: u32,
    piece_start_lo: u32, piece_start_hi: u32,
    duration_sec_bits: u32,
) -> i32 {
    let Some(isr) = (unsafe { resolve_isr_state_mut(rt) }) else {
        return KALICO_ERR_NULL_PTR;
    };
    let bp = [
        f32::from_bits(bp0_bits),
        f32::from_bits(bp1_bits),
        f32::from_bits(bp2_bits),
        f32::from_bits(bp3_bits),
    ];
    let piece_start = ((piece_start_hi as u64) << 32) | (piece_start_lo as u64);
    let dur = f32::from_bits(duration_sec_bits);
    isr.engine.push_piece_for_axis(axis_idx, bp, piece_start, dur)
}
```

If `resolve_isr_state_mut` doesn't exist with that exact name, mirror the inline projection pattern already used by `kalico_runtime_configure_axis` (Task 11 of the firmware plan).

- [ ] **Step 3: Declare in the C header**

In `rust/kalico-c-api/include/kalico_runtime.h`, near other `kalico_runtime_*` declarations:

```c
int32_t kalico_runtime_push_piece(
    void *handle,
    uint8_t axis_idx,
    uint32_t bp0_bits, uint32_t bp1_bits, uint32_t bp2_bits, uint32_t bp3_bits,
    uint32_t piece_start_lo, uint32_t piece_start_hi,
    uint32_t duration_sec_bits);
```

- [ ] **Step 4: Verify build**

```
cd rust && cargo build -p runtime --lib 2>&1 | tail -5
cd rust && cargo build --workspace 2>&1 | tail -5
```

Expected: clean.

- [ ] **Step 5: Commit**

```
git add rust/runtime/src/engine.rs rust/kalico-c-api/src/runtime_ffi.rs rust/kalico-c-api/include/kalico_runtime.h
git commit -m "feat(ffi): kalico_runtime_push_piece for direct cubic Bezier load"
```

---

### Task 2: C-side command handler for push_piece

**Files:**
- Modify: `src/stepper.c` (after the Task 11 / 12 command handlers added in the firmware plan)

- [ ] **Step 1: Add the DECL_COMMAND handler**

Append to `src/stepper.c`:

```c
// Stepping-redesign bench-bringup: cubic Bezier piece push for a single axis.

extern int32_t kalico_runtime_push_piece(
    void *handle, uint8_t axis_idx,
    uint32_t bp0_bits, uint32_t bp1_bits, uint32_t bp2_bits, uint32_t bp3_bits,
    uint32_t piece_start_lo, uint32_t piece_start_hi,
    uint32_t duration_sec_bits);

void
command_kalico_push_piece(uint32_t *args)
{
    if (!runtime_handle) shutdown("kalico_push_piece before runtime init");
    uint8_t axis_idx = args[0];
    uint32_t bp0 = args[1], bp1 = args[2], bp2 = args[3], bp3 = args[4];
    uint32_t ps_lo = args[5], ps_hi = args[6];
    uint32_t dur = args[7];
    int32_t rc = kalico_runtime_push_piece(runtime_handle, axis_idx,
                                            bp0, bp1, bp2, bp3,
                                            ps_lo, ps_hi, dur);
    if (rc != 0)
        shutdown("kalico_push_piece rejected by runtime");
}
DECL_COMMAND(command_kalico_push_piece,
             "kalico_push_piece axis_idx=%c bp0=%u bp1=%u bp2=%u bp3=%u"
             " piece_start_lo=%u piece_start_hi=%u duration_bits=%u");
```

- [ ] **Step 2: Verify build (Pi)**

```
git add src/stepper.c
git commit -m "feat(commands): kalico_push_piece dispatch shim"
git push origin sota-motion
ssh dderg@trident.local 'cd ~/klipper && git pull && make -j$(nproc) 2>&1 | tail -5'
```

Expected: clean build.

---

### Task 3: Install per-axis Klipper timers + enable TIM5

**Files:**
- Modify: `src/runtime_tick.c`
- Modify: `src/stm32/runtime_tick_h7.c`
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` (add `kalico_runtime_count_configured_axes` helper)
- Modify: `rust/runtime/src/engine.rs` (impl method on Engine)

- [ ] **Step 1: Add engine helper to count configured axes**

In `rust/runtime/src/engine.rs`, after `configure_axis`:

```rust
/// Number of axes that have a non-zero microstep_distance (i.e., have been
/// configured by the host). Used by the TIM5 enable gate to avoid running
/// the ISR before any axis is live.
pub fn count_configured_axes(&self) -> u8 {
    let mut n: u8 = 0;
    for axis in &self.stepping_axes {
        if axis.microstep_distance > 0.0 && axis.microstep_distance.is_finite() {
            n = n.saturating_add(1);
        }
    }
    n
}
```

- [ ] **Step 2: Add FFI accessor**

In `rust/kalico-c-api/src/runtime_ffi.rs`:

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_count_configured_axes(
    rt: *mut KalicoRuntime,
) -> u8 {
    let Some(isr) = (unsafe { resolve_isr_state(rt) }) else { return 0; };
    isr.engine.count_configured_axes()
}
```

Use the read-only projection helper (mirror `kalico_runtime_count_modulated_steppers`'s pattern; copy if needed).

- [ ] **Step 3: Wire init_per_axis_step_timers into runtime_init**

In `src/runtime_tick.c`, in `runtime_init` after `runtime_tick_init();` and `sched_add_timer(&runtime_drain_timer);`, add:

```c
    // Bench bring-up Task 3: install per-axis step consumer timers
    // alongside the legacy step_time_event timers. The new path is
    // only used by axes that have been configured via
    // kalico_configure_axis; until then the per-axis timers see empty
    // step_queues and just reschedule for next sample (cheap).
    extern void init_per_axis_step_timers(void);
    init_per_axis_step_timers();
```

- [ ] **Step 4: Replace the TIM5 enable gate**

In `src/stm32/runtime_tick_h7.c::runtime_tick_enable`, replace:

```c
    if (kalico_runtime_count_modulated_steppers(runtime_handle) == 0) {
        return;
    }
```

With:

```c
    // Bench bring-up Task 3: enable TIM5 when at least one axis is
    // configured for either path. count_modulated_steppers covers the
    // legacy modulated step-mode; count_configured_axes covers the new
    // unified-tick path. Either is sufficient.
    extern uint8_t kalico_runtime_count_configured_axes(void *rt);
    if (kalico_runtime_count_modulated_steppers(runtime_handle) == 0
        && kalico_runtime_count_configured_axes(runtime_handle) == 0) {
        return;
    }
```

Do the same in `src/stm32/runtime_tick_f4.c`. (Even though F4 won't use the new path, the gate stays consistent.)

- [ ] **Step 5: Verify build + commit**

```
cd rust && cargo build --workspace 2>&1 | tail -5
git add rust/runtime/src/engine.rs rust/kalico-c-api/src/runtime_ffi.rs src/runtime_tick.c src/stm32/runtime_tick_h7.c src/stm32/runtime_tick_f4.c
git commit -m "feat(tim5): install per-axis timers + extend enable gate to configured axes"
git push origin sota-motion
ssh dderg@trident.local 'cd ~/klipper && git pull && make -j$(nproc) 2>&1 | tail -5'
```

Expected: clean build.

---

### Task 4: Stage 1 — boot + idle telemetry verification

**Goal:** Flash, verify TIM5 fires at configured rate (40 kHz on H7), all step queues stay at depth 0, no faults.

- [ ] **Step 1: Ask user to BOOT0+RESET H7**

(User effort — only ask once we have a meaningful change to flash.)

- [ ] **Step 2: Flash**

```
ssh dderg@trident.local 'lsusb | grep 0483:df11 && cd ~/klipper && make flash FLASH_DEVICE=0483:df11 2>&1 | tail -3'
```

- [ ] **Step 3: Wait for enumeration + restart klipper**

```
ssh dderg@trident.local 'sleep 10 && ls /dev/serial/by-id/ | grep stm32h723 && sudo systemctl restart klipper && sleep 8 && systemctl is-active klipper'
```

Expected: H7 enumerates, klipper active.

- [ ] **Step 4: Inspect kalico_status frames**

```
ssh dderg@trident.local 'tail -50 ~/printer_data/logs/klippy.log | grep -E "engine_status|queue_depth|last_fault.*[1-9]|Loaded MCU|MCU.*config" | tail -10'
```

Expected:
- `engine_status: 0` (idle)
- `queue_depth: 0`
- `last_fault: 0`
- Both MCUs loaded with their command sets

- [ ] **Step 5: Verify TIM5 is firing**

The kalico_status frame exposes `tim5_n` (TIM5 fire count). Query it via klippy and check it's incrementing:

```
ssh dderg@trident.local 'tail -100 ~/printer_data/logs/klippy.log | grep -oE "tim5_n [0-9]+" | tail -5'
```

Expected: monotonically increasing values. If TIM5 isn't enabled yet (no axes configured), counter stays at 0 — that's OK for this stage; we'll re-check after Task 5.

- [ ] **Step 6: Document Stage 1 result**

If telemetry looks healthy, proceed. If faults latch or queues fill spontaneously, capture the fault_detail value and bisect.

---

### Task 5: Stage 1.5 — host sends configure_axis for X/Y/E on H7

**Files:**
- Modify: `klippy/motion_bridge.py` — Python wrappers for new FFIs
- Modify: `klippy/motion_toolhead.py` — emit configure_axis/_kinematics after configure_axes

- [ ] **Step 1: Add Python wrappers**

In `klippy/motion_bridge.py`, add methods to the bridge facade class:

```python
def kalico_configure_axis(self, mcu_handle, axis_idx, mode, microstep_distance,
                          extrusion_per_xy_mm, stepper_count):
    """Send kalico_configure_axis command. Mode: 0=Pulse, 1=Phase."""
    return self._bridge.kalico_configure_axis(
        mcu_handle, axis_idx, mode,
        microstep_distance, extrusion_per_xy_mm, stepper_count,
    )

def kalico_configure_kinematics(self, mcu_handle, k_xy):
    return self._bridge.kalico_configure_kinematics(mcu_handle, k_xy)

def kalico_configure_pressure_advance(self, mcu_handle, advance_accel, advance_decel):
    return self._bridge.kalico_configure_pressure_advance(
        mcu_handle, advance_accel, advance_decel,
    )

def kalico_push_piece(self, mcu_handle, axis_idx, bernstein_4,
                     piece_start_cycles, duration_sec):
    """Bernstein: list of 4 f32 control points. piece_start_cycles: u64 absolute."""
    return self._bridge.kalico_push_piece(
        mcu_handle, axis_idx, bernstein_4,
        piece_start_cycles, duration_sec,
    )
```

The underlying `self._bridge` is the `motion_bridge_native` cdylib. PyO3 binding for each new method goes in the next step.

- [ ] **Step 2: Add PyO3 bindings in motion-bridge Rust**

Inspect `rust/motion-bridge/src/lib.rs` and the `#[pymethods]` impl block. Add four bindings that call through to `kalico-host-rt` which in turn sends the Klipper protocol command to the MCU. Pattern:

```rust
#[pymethods]
impl Bridge {
    fn kalico_configure_axis(&self, mcu_handle: u64, axis_idx: u8, mode: u8,
                              microstep_distance: f32, extrusion_per_xy_mm: f32,
                              stepper_count: u8) -> PyResult<i32> {
        let rc = self.with_host_io(mcu_handle, |io| {
            io.send_command(
                "kalico_configure_axis",
                &[axis_idx as u32, mode as u32,
                  microstep_distance.to_bits(),
                  extrusion_per_xy_mm.to_bits(),
                  stepper_count as u32],
            )
        })?;
        Ok(rc)
    }
    // ... configure_kinematics, configure_pressure_advance, push_piece
}
```

Adjust to the actual API of `kalico-host-rt`'s send_command / host_io accessor. Read existing send-command patterns in the bridge (e.g., how `configure_axes` is sent) and mirror them.

- [ ] **Step 3: Wire configure_axis emission into MotionToolhead startup**

In `klippy/motion_toolhead.py`, in `_configure_axes_per_mcu` (line ~631), after the existing `self.bridge.configure_axes(...)` call (line ~888), add:

```python
# Bench bring-up Task 5: send the new stepping-redesign configure commands
# alongside the legacy configure_axes blob. Only H7 (main MCU) gets these;
# F4 (bottom) sticks to the legacy path.
if name == "mcu":  # H7 main MCU
    # X/Y/E live on H7. Z lives on F4 (legacy path).
    AXIS_X, AXIS_Y, AXIS_E = 0, 1, 3
    MODE_PULSE = 0
    # Compute microstep_distance from steps_per_mm.
    # axis index → klippy stepper name
    axis_steppers_per_mm = {
        AXIS_X: steps_per_mm[0],
        AXIS_Y: steps_per_mm[1],
        AXIS_E: steps_per_mm[3],
    }
    for axis_idx, spm in axis_steppers_per_mm.items():
        if spm <= 0:
            continue
        microstep_dist = 1.0 / spm
        extrusion_per_xy = 0.0  # set by activate_extruder later
        stepper_count = sum(1 for b in runtime_bindings
                            if b[0] == axis_idx)
        rc = self.bridge.kalico_configure_axis(
            mcu_handle, axis_idx, MODE_PULSE,
            microstep_dist, extrusion_per_xy, stepper_count,
        )
        if rc != 0:
            raise self.printer.config_error(
                f"kalico_configure_axis failed for axis {axis_idx}: rc={rc}"
            )
    # CoreXY kinematics: k_xy = 1/sqrt(2)
    import math
    k_xy = 1.0 / math.sqrt(2.0) if self.is_corexy else 1.0
    rc = self.bridge.kalico_configure_kinematics(mcu_handle, k_xy)
    if rc != 0:
        raise self.printer.config_error(
            f"kalico_configure_kinematics failed: rc={rc}"
        )
    # PA: zero until extruder activate
    self.bridge.kalico_configure_pressure_advance(mcu_handle, 0.0, 0.0)
```

`self.is_corexy` may need to be derived from `self.kin` / `printer.cfg`'s `[printer] kinematics`. Inspect existing MotionToolhead init for how it knows kinematics, and reuse.

- [ ] **Step 4: Verify build + restart klipper**

```
git add klippy/motion_bridge.py klippy/motion_toolhead.py rust/motion-bridge/src/lib.rs
git commit -m "feat(host): emit kalico_configure_axis/_kinematics/_pressure_advance at startup"
git push origin sota-motion
ssh dderg@trident.local 'cd ~/klipper && git pull && make -f Makefile.kalico motion-bridge 2>&1 | tail -5 && sudo systemctl restart klipper && sleep 8'
```

Expected: motion-bridge rebuilds; klipper restarts cleanly.

- [ ] **Step 5: Verify configure commands landed**

```
ssh dderg@trident.local 'tail -50 ~/printer_data/logs/klippy.log | grep -E "kalico_configure|tim5_n" | tail -10'
```

Expected:
- Klippy log shows configure_axis calls succeeded (rc=0)
- `tim5_n` counter starts incrementing (TIM5 now enabled because count_configured_axes > 0)
- No shutdowns or fault latches

If `tim5_n` doesn't increment, debug: check `runtime_tick_enable` gate (Task 3 Step 4), confirm `count_configured_axes` returns ≥1, check klippy actually reached the configure call.

---

### Task 6: Stage 2 — single-stepper jog (set_stepper_offset)

**Goal:** Make ONE physical step pulse fire by ramping `phase_offset_target` on stepper 0. This is the simplest possible motion test.

**Note:** Per spec, set_stepper_offset is for Phase mode (TMC5160 XDIRECT calibration). In Pulse mode the offset isn't directly observable. Adapt: instead of set_stepper_offset, use **kalico_push_piece** with a linear ramp piece that produces exactly 10 microsteps on X (axis 0) over 100 ms — slow, visible, observable in position_count telemetry.

**Files:**
- Modify: `klippy/extras/kalico_motion_test.py` (NEW) — gcode macro `KALICO_STEPPER_JOG` that pushes a known piece

- [ ] **Step 1: Write the macro module**

Create `klippy/extras/kalico_motion_test.py`:

```python
# Bench bring-up Stage 2: direct piece-push jog primitive.
#
# Bypasses the trapq + planner; sends a single cubic Bezier piece
# directly to the runtime via kalico_push_piece. Used for stepping-
# redesign validation. Not a production motion path.

class KalicoMotionTest:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "KALICO_STEPPER_JOG",
            self.cmd_KALICO_STEPPER_JOG,
            desc="Direct piece push for stepping-redesign validation",
        )

    def cmd_KALICO_STEPPER_JOG(self, gcmd):
        axis_idx = gcmd.get_int("AXIS", 0, minval=0, maxval=3)
        delta_mm = gcmd.get_float("DELTA", 1.0)
        duration_s = gcmd.get_float("DURATION", 0.1, above=0.001)
        # Construct linear-position cubic Bezier:
        # P(t) = (delta_mm/duration_s) * t  in seconds-domain
        # Bernstein [0, delta/3, 2*delta/3, delta] scaled to seconds:
        # We want P(duration) = delta, so scale = delta/duration; then
        # at t=duration the polynomial yields delta. The eval is
        # P(t_local_sec) so coefficients are in mm/s units when stored
        # against duration_sec. Per Task 8 of the firmware plan:
        # for P(duration)=delta with P(0)=0, linear Bernstein
        # control points = [0, delta/3, 2*delta/3, delta] but
        # interpreted at t_local in seconds yields
        # P(duration) = delta — confirmed by the tick_integration test.
        bernstein = [0.0, delta_mm / 3.0, 2.0 * delta_mm / 3.0, delta_mm]
        motion_toolhead = self.printer.lookup_object("motion_toolhead")
        # piece_start_cycles = "now + 10ms" so the runtime has time to
        # ingest before the polynomial starts evaluating.
        mcu = motion_toolhead.get_mcu("mcu")
        clock = mcu.print_time_to_clock(motion_toolhead.print_time + 0.010)
        bridge = motion_toolhead.bridge
        mcu_handle = motion_toolhead.get_bridge_handle("mcu")
        rc = bridge.kalico_push_piece(
            mcu_handle, axis_idx, bernstein, int(clock), duration_s,
        )
        if rc != 0:
            raise gcmd.error(f"kalico_push_piece rc={rc}")
        gcmd.respond_info(
            f"Pushed piece: axis={axis_idx} delta={delta_mm}mm dur={duration_s}s"
        )

def load_config(config):
    return KalicoMotionTest(config)
```

Adapt to actual `motion_toolhead` API for `print_time`, `mcu_handle`, etc. Inspect those during writing.

- [ ] **Step 2: Register the module in printer.cfg**

User's `printer.cfg` needs `[kalico_motion_test]` somewhere. Per memory `feedback_user_owns_test_config`, **don't edit it** — ask the user to add the section.

Add to plan: agent must prompt user to add this two-line section:
```
[kalico_motion_test]
```

- [ ] **Step 3: Restart klipper, send a tiny test jog**

After user confirms config edit, send `KALICO_STEPPER_JOG AXIS=0 DELTA=0.05 DURATION=0.5` (50 microns, slow). This is a tiny enough motion that the safety risk on the test bench is zero.

Verify:
```
ssh dderg@trident.local 'curl -s http://localhost:7125/printer/objects/query?motion_report'
```

Expected: `live_position[0]` advances. position_count telemetry from kalico_status v6 frame should increase too.

If the motor physically moves: Stage 2 PASSED. If not: bisect — check tim5_n incrementing, check queue_high_water counter, check whether per_axis_timer is firing (need a diag counter for that).

- [ ] **Step 4: Commit**

```
git add klippy/extras/kalico_motion_test.py
git commit -m "feat(motion_test): KALICO_STEPPER_JOG primitive for redesign bench bringup"
git push origin sota-motion
```

---

### Task 7: Stage 3 — pure-X G1 in CoreXY (A and B in lockstep)

**Goal:** Issue `G1 X10 F600` and observe both A and B (X primary + X1 partner on CoreXY mapping) stepping in lockstep. Same direction, same magnitude.

**Files:**
- Modify: `klippy/motion_toolhead.py` — instrument the normal G-code path to also emit `kalico_push_piece` for X moves.

This task requires the planner's per-move cubic Bezier representation to be converted into the per-axis Bernstein control points the runtime expects. Approach: hook the existing motion_kinematics.motor_deltas path; for each move, compute per-axis Bernstein from the planner's curve at the points (start, 1/3, 2/3, end) — sample positions four points, that defines a cubic.

- [ ] **Step 1: Inspect motion_kinematics' segment emission**

Read `klippy/motion_kinematics.py` (or wherever motion_toolhead drives per-axis segment generation). Locate the segment-emit callsite that pushes via the existing `push_segment` FFI. Add a parallel `kalico_push_piece` emit at the same point.

- [ ] **Step 2: Implement Bernstein-from-cubic-sample helper**

Where the existing planner emits its per-axis cubic Bezier (or equivalent), compute the Bernstein control points. The host planner's pieces are degree-3 polynomials in seconds domain. Given polynomial coefficients `p(t) = c0 + c1·t + c2·t² + c3·t³`, the Bernstein for the [0, duration] domain is computed via the standard monomial-to-Bernstein matrix. Helper:

```python
def monomial_to_bernstein_cubic(c0, c1, c2, c3, duration):
    """Cubic monomial → Bernstein control points over [0, duration]."""
    d = duration
    return (
        c0,
        c0 + c1 * d / 3.0,
        c0 + 2.0 * c1 * d / 3.0 + c2 * d * d / 3.0,
        c0 + c1 * d + c2 * d * d + c3 * d * d * d,
    )
```

Verify: at t=0 the polynomial returns the first CP; at t=duration it returns the fourth CP. Cross-check with the runtime's `bernstein_to_monomial` (rust/runtime/src/monomial.rs).

- [ ] **Step 3: Wire the emit point**

After the existing per-axis segment push, emit:

```python
for axis_idx in (0, 1, 3):  # X, Y, E on H7
    bernstein = monomial_to_bernstein_cubic(
        axis_poly[axis_idx].c0, axis_poly[axis_idx].c1,
        axis_poly[axis_idx].c2, axis_poly[axis_idx].c3,
        move.duration_sec,
    )
    self.bridge.kalico_push_piece(
        self.h7_handle, axis_idx, bernstein,
        move.piece_start_cycles, move.duration_sec,
    )
```

Adapt to actual motion_kinematics field names.

- [ ] **Step 4: Send the test G-code**

After flashing + restart, issue:
```
SET_KINEMATIC_POSITION X=20 Y=100 Z=10
G1 X30 F600
G1 X20 F600
```

Watch klippy log + motion_report.live_position. Expected: both A and B step in same direction same amount on each G1.

- [ ] **Step 5: Verify telemetry**

The kalico_status frame's `position_count` (per-stepper) should match: after `G1 X30 F600` (10mm at 160 steps/mm = 1600 microsteps), A and B both at +1600.

- [ ] **Step 6: Commit**

```
git add klippy/motion_kinematics.py klippy/motion_toolhead.py
git commit -m "feat(motion): emit kalico_push_piece alongside legacy push_segment"
```

---

### Task 8: Stage 3b — pure-Y G1 (CoreXY: A and B opposite)

**Goal:** Issue `G1 Y10 F600` and observe A and B steppers stepping in OPPOSITE directions, same magnitude.

This validates the CoreXY kinematic transformation (k_xy direction-dependence). No new code: same emit path as Task 7. Just verify A's position_count increments while B's decrements (or vice versa, depending on AWD mapping).

- [ ] **Step 1: Issue Y move**

```
G1 Y110 F600
G1 Y100 F600
```

- [ ] **Step 2: Verify telemetry**

```
ssh dderg@trident.local 'curl -s http://localhost:7125/printer/objects/query?motion_report'
```

Expected: `stepper_x` position_count and `stepper_x1` position_count differ in sign (one increments, other decrements).

- [ ] **Step 3: Document Stage 3b result**

---

### Task 9: Stage 4 — multi-motor stress

**Goal:** Sustain motion at higher feedrate (F12000 = 200 mm/s), diagonal move, multiple back-and-forth. No queue overflows, no faults latched.

- [ ] **Step 1: Sustained square pattern**

```
G1 X30 Y130 F12000
G1 X10 Y130 F12000
G1 X10 Y110 F12000
G1 X30 Y110 F12000
G1 X30 Y130 F12000
```

Repeat 10 times.

- [ ] **Step 2: Monitor telemetry**

```
ssh dderg@trident.local 'tail -200 ~/printer_data/logs/klippy.log | grep -E "queue_high_water|queue_overflow|last_fault.*[1-9]|engine_status.*[2-9]"'
```

Expected: no overflow_count increments, no fault_codes, queue_high_water stays bounded (e.g., ≤ 8 of 32 capacity).

- [ ] **Step 3: Document Stage 4 result**

---

### Task 10: Stage 5 — Phase mode for X

**Goal:** Switch X (axis 0) from Pulse to Phase mode. Verify TMC5160 XDIRECT writes via SPI. Position tracking matches Pulse mode after the switch.

**Files:**
- Modify: `klippy/extras/kalico_motion_test.py` — add `KALICO_SET_AXIS_MODE` macro
- Modify: `klippy/motion_toolhead.py` — wire `tmc_cs` pin handles onto StepperRef during configure

**Pre-requisite:** TMC5160 chopconf / motor current / etc. must already be set via the user's existing TMC config in printer.cfg. This task only handles the XDIRECT write loop.

- [ ] **Step 1: Add the mode-switch macro**

In `klippy/extras/kalico_motion_test.py`:

```python
def cmd_KALICO_SET_AXIS_MODE(self, gcmd):
    axis_idx = gcmd.get_int("AXIS", 0, minval=0, maxval=3)
    mode = gcmd.get_int("MODE", 0, minval=0, maxval=1)  # 0=Pulse, 1=Phase
    motion_toolhead = self.printer.lookup_object("motion_toolhead")
    rc = motion_toolhead.bridge.kalico_set_axis_mode(
        motion_toolhead.get_bridge_handle("mcu"), axis_idx, mode,
    )
    if rc != 0:
        raise gcmd.error(f"kalico_set_axis_mode rc={rc}")
    gcmd.respond_info(f"Set axis {axis_idx} mode={mode}")
```

- [ ] **Step 2: Wire tmc_cs (SPI CS pin) into per-stepper config**

The runtime's `StepperRef.tmc_cs: Option<u32>` needs to be populated for Phase mode SPI dispatch (Task 14 of the firmware plan). Currently `configure_axis` clears steppers; we need a separate `bind_stepper` command that sets `step_pin`, `dir_pin`, `dir_invert`, `tmc_cs` for each stepper on each axis.

This is substantial: add `kalico_runtime_bind_stepper(axis_idx, stepper_local_idx, step_pin, dir_pin, dir_invert, tmc_cs_pin)` FFI + DECL_COMMAND. Klippy emits at startup after configure_axis.

- [ ] **Step 3: Verify Phase mode swap**

After flashing:
```
KALICO_SET_AXIS_MODE AXIS=0 MODE=1
G1 X25 F300
```

Watch with a logic analyzer (per spec): SPI MOSI traffic to TMC5160 should carry XDIRECT register (0x2D) writes with the expected coil sequence.

If no logic analyzer: at minimum, verify klippy log shows no faults, `position_count` for stepper_x increments matching what Pulse mode produced for the same G1.

- [ ] **Step 4: Commit**

---

### Task 11: Stage 6 — sensorless homing via mode switch

**Goal:** X primary axis runs Phase mode normally but switches to Pulse for homing (which uses TMC5160 stallguard requiring step pulses). After home, switches back to Phase.

**Files:**
- Modify: `klippy/extras/homing.py` or similar — issue `KALICO_SET_AXIS_MODE` at homing start/end

Depends on Stage 5 working. Wire into klippy's homing path: when G28 X starts, call `kalico_set_axis_mode(0, 0)` (Pulse); after home complete, call `kalico_set_axis_mode(0, 1)` (Phase). Stallguard configuration on the TMC5160 is the user's existing config — outside this plan's scope.

- [ ] **Step 1: Wire mode switch into G28**

Detailed steps depend on klippy homing internals. Read `klippy/extras/homing.py` and add the calls.

- [ ] **Step 2: Test G28 X**

After flashing:
```
KALICO_SET_AXIS_MODE AXIS=0 MODE=1  # Start in Phase
G28 X
```

Expected: homing fires stallguard via Pulse-mode stepper motion. After complete, axis 0 returns to Phase mode (verify via subsequent G1 X… motion working).

- [ ] **Step 3: Document Stage 6 result**

---

### Task 12: Stage 7 — long-print soak

**Goal:** Run a real CoreXY print of ≥1 hour. No faults, no peak-cycle drift, `position_count` returns to expected total motion at end.

- [ ] **Step 1: Pick a small calibration print**

Use a simple test print (e.g., a 1-hour bed-level cube or a small benchy). Slice with the user's normal slicer.

- [ ] **Step 2: Start print, monitor telemetry**

Watch `~/printer_data/logs/klippy.log` for any fault_detail values exceeding sanity (look for `last_fault.*[1-9]`). Watch `kalico_status` `queue_overflow_count` for any axis bumping.

- [ ] **Step 3: After print, verify total position_count matches expected**

Total expected steps = total_extrusion_mm * steps_per_mm for E, total move distance * steps_per_mm for X/Y. Compare against the final position_count values from the last kalico_status frame.

- [ ] **Step 4: Document Stage 7 result + close the bench bring-up**

---

## Self-Review

**Spec coverage**: Stages 1-7 from the design doc each map to a Task (4-12). The architectural decision to keep both paths in firmware (X/Y/E on new, Z on legacy) is articulated at the top.

**Placeholder scan**: 
- Task 5 Step 2 references "PyO3 bindings in motion-bridge Rust" with the pattern but acknowledges adapting to the actual bridge API. This is necessary because the bridge has substantial existing code that needs inspection — not a placeholder, but a "read first, then code".
- Task 6 Step 2 asks user to add `[kalico_motion_test]` to printer.cfg. Per memory `feedback_user_owns_test_config`, this is correct — we don't edit it.
- Task 5 Step 3 has `axis_steppers_per_mm` with klippy-side computation that depends on motion_toolhead's exact data structure; adaptation is necessary.

**Type consistency**: `kalico_push_piece` signature is `(axis_idx, bp0..bp3 bits, piece_start_lo, piece_start_hi, duration_bits)` across the engine method (Task 1), FFI (Task 1), C handler (Task 2), Python wrapper (Task 5), and macro (Task 6). All consistent.

**Risk areas:**
- Task 5 + 7 require klippy code changes that interact with the existing motion-bridge planner. The plan calls out "adapt to actual API" — execution may need additional discovery.
- Stage 5 (Phase mode) requires `kalico_runtime_bind_stepper` FFI which the plan adds in Task 10 Step 2 but doesn't fully specify. Execution time may surface complexity around how klippy already binds steppers via `command_config_runtime_stepper`.
- The `print_time` → `piece_start_cycles` conversion in Task 6 assumes klippy's clock_freq matches the runtime's `cycles_per_second`. Should verify.

These are flagged for the executor's attention, not gaps to fill upfront.

---

## Open items deferred

- TMC5160 register pre-config (chopconf, stallguard) — user's existing printer.cfg handles this. Not touched by this plan.
- klipper-sim integration (Task 15 of firmware plan) — offline validation, separate workstream.
- F4 (bottom) firmware migration to new path — left on legacy for Z. Future plan.
- Position-tracking observability beyond `position_count` (e.g., per-tick trace ring sampling) — telemetry is already there via kalico_status v6 frame; specific dashboards out of scope.
- Performance characterization (per-tick CPU ceiling) — spec §"Per-MCU step-rate ceilings" defines targets; measurement is left as ongoing telemetry observation during stages.
