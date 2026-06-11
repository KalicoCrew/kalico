# Phase Stepping Restore Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore TMC5160 XDIRECT phase stepping, broken when the legacy `configure_axes_blob` path was deleted (commit `96b80edab`) without re-wiring its two side effects into the live per-axis config path.

**Architecture:** Two independent breaks, fixed through the live config path: (1) the host hardcodes `MODE_PULSE` in `kalico_configure_axis` and nothing ever sets the axis to Phase mode — fix by sending the real per-axis mode byte; (2) `shared.phase_slot_idx` / `shared.phase_motor_count` (the motor→kinematic-slot map `dispatch_phase` needs to address SPI writes) have no production writer — fix by carrying `slot_idx` on the existing `runtime_register_phase_motor` command into a new FFI setter. Plus: make the unmapped-motor case fail loudly instead of silently skipping, and delete the dead `ConfigureAxes` bridge path so the same data can never have two half-true sources.

**Tech Stack:** Rust (`rust/runtime`, `rust/kalico-c-api`, `rust/motion-bridge`), C MCU firmware (`src/`), Klippy Python host (`klippy/`). Tests via `cargo nextest run` from `rust/`.

**Background for the implementer (zero-context summary):**
- In XDIRECT mode the TMC5160 ignores step/dir pins and only obeys coil currents written over SPI. The host (`klippy/extras/tmc5160.py`) puts drivers in XDIRECT at connect when `phase_stepping` is configured.
- The MCU runtime's TIM5 ISR calls `dispatch_axis` (`rust/runtime/src/dispatch_stepper.rs:61`); per the axis `mode` byte it either emits step pulses (`dispatch_pulse`) or computes coil currents and calls the C `phase_stepping_write_xdirect` (`dispatch_phase`).
- `dispatch_phase` must translate a stepper to a C-side "motor index" (registered via `runtime_register_phase_motor`); it does this by scanning `shared.phase_slot_idx[0..phase_motor_count]` for entries matching the axis index. That table is currently never written ⇒ lookup fails ⇒ sentinel `0xFF` ⇒ `src/stm32/phase_stepping_spi.c:120` returns without writing ⇒ no motion, no error.
- Mode-byte encodings differ by layer and are easy to fumble:
  - Host `step_modes` list and runtime `state::StepMode`: **0 = Modulated (phase), 1 = StepTime (pulse)**.
  - `kalico_configure_axis` wire mode byte and runtime `stepping_state::StepMode`: **0 = Pulse, 1 = Phase**.
- Protocol caution: changing any `DECL_COMMAND` string changes the MCU command dictionary — both bench MCUs (H723 + F446) must be reflashed together, with `make clean` between builds.

---

### Task 1: New fault code `PhaseMotorUnmapped` (-313) + fix missing `-312` decode

**Files:**
- Modify: `rust/runtime/src/error.rs` (enum at line ~108, `from_u16` at ~216, `code_name` at ~294, const block at ~104)
- Modify: `rust/runtime/src/fault_helpers.rs` (append next to `raise_unknown_step_mode`, ~line 170)

- [ ] **Step 1: Extend the `from_u16` doctest to cover -312 and -313 (failing first)**

In `rust/runtime/src/error.rs`, add to the `from_u16` doctest examples block (after the `0xFEC9` line):

```rust
/// // UnknownStepMode = -312; -312i16 as u16 = 0xFEC8
/// assert_eq!(FaultCode::from_u16(0xFEC8), Some(FaultCode::UnknownStepMode));
/// // PhaseMotorUnmapped = -313; -313i16 as u16 = 0xFEC7
/// assert_eq!(FaultCode::from_u16(0xFEC7), Some(FaultCode::PhaseMotorUnmapped));
```

- [ ] **Step 2: Run the doctest, verify it fails**

Run from `rust/`: `cargo test --doc -p runtime error`
Expected: FAIL — `PhaseMotorUnmapped` does not exist; `from_u16(0xFEC8)` returns `None` (the `-312` arm is missing from `from_u16` today — pre-existing decode bug).

- [ ] **Step 3: Implement the enum + const + decode + name entries**

In `rust/runtime/src/error.rs`:

```rust
// after KALICO_ERR_UNKNOWN_STEP_MODE
/// `dispatch_phase` found a Phase-mode stepper with a TMC CS binding but no
/// entry in `phase_slot_idx[0..phase_motor_count]` maps it to a registered
/// SPI motor. Detail: `((axis_idx & 0xFF) << 16) | stepper_oid`.
pub const KALICO_ERR_PHASE_MOTOR_UNMAPPED: i32 = -313;
```

In the `FaultCode` enum after `UnknownStepMode = -312,`:

```rust
    PhaseMotorUnmapped = -313,
```

In `from_u16`, after the `-311` arm:

```rust
            -312 => Self::UnknownStepMode,
            -313 => Self::PhaseMotorUnmapped,
```

In `code_name`, alongside the other `-3xx` entries:

```rust
            Self::PhaseMotorUnmapped => "PhaseMotorUnmapped",
```

In `rust/runtime/src/fault_helpers.rs`, after `raise_unknown_step_mode`:

```rust
/// Latch a `PhaseMotorUnmapped` fault. Detail:
/// `((axis_idx & 0xFF) << 16) | stepper_oid`.
#[inline]
pub fn raise_phase_motor_unmapped(shared: &SharedState, axis_idx: usize, stepper_oid: u8) {
    let detail = ((axis_idx as u32 & 0xFF) << 16) | u32::from(stepper_oid);
    shared.fault_detail.store(detail, Ordering::Release);
    shared
        .last_error
        .store(FaultCode::PhaseMotorUnmapped.as_i32(), Ordering::Release);
    emit_fault_log(FaultCode::PhaseMotorUnmapped, detail);
}
```

- [ ] **Step 4: Run doctests + the runtime crate suite**

Run from `rust/`: `cargo test --doc -p runtime && cargo nextest run -p runtime`
Expected: PASS. (`code_name` exhaustive-match tests in `rust/runtime/src/log_codes/tests.rs` may reference fault names — if one fails listing expected names, add `PhaseMotorUnmapped` there too.)

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/error.rs rust/runtime/src/fault_helpers.rs
git commit -m "runtime: PhaseMotorUnmapped fault (-313); fix missing -312 in FaultCode::from_u16"
```

---

### Task 2: `dispatch_phase` fails loudly on unmapped motor

**Files:**
- Modify: `rust/runtime/src/dispatch_stepper.rs:342` (the `unwrap_or(0xFF)` sentinel)
- Test: `rust/runtime/tests/phase_xdirect_dispatch.rs` (the `phase_dispatch_empty_slot_table_uses_sentinel_motor_idx` test, ~line 276)

- [ ] **Step 1: Rewrite the empty-slot-table test to expect a latched fault and zero SPI captures**

Replace `phase_dispatch_empty_slot_table_uses_sentinel_motor_idx` in `rust/runtime/tests/phase_xdirect_dispatch.rs` with (reuse the file's existing `make_phase_stepper` / `make_phase_axis` / `q_ptr_from` helpers):

```rust
#[test]
fn phase_dispatch_empty_slot_table_latches_phase_motor_unmapped() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    assert_eq!(shared.phase_motor_count.load(Ordering::Acquire), 0);

    let mut q = StepQueue::new();
    let stepper = make_phase_stepper(0, 7);
    let mut axis = make_phase_axis(0.0125, stepper);

    dispatch_axis(
        0,
        &mut axis,
        q_ptr_from(&mut q),
        &shared,
        256.0 * 0.0125,
        0.0,
        0.0,
        25e-6,
        0,
        520_000_000.0,
    );

    let records = test_xdirect_capture::drain();
    assert!(records.is_empty(), "no SPI write may reach an unmapped motor");
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        runtime::error::FaultCode::PhaseMotorUnmapped.as_i32(),
        "unmapped phase motor must latch PhaseMotorUnmapped"
    );
    let detail = shared.fault_detail.load(Ordering::Acquire);
    assert_eq!(detail >> 16, 0, "axis_idx in detail high bits");
    assert_eq!(detail & 0xFFFF, 0, "stepper_oid in detail low bits");
}
```

- [ ] **Step 2: Run it, verify it fails**

Run from `rust/`: `cargo nextest run -p runtime -E 'test(phase_dispatch_empty_slot_table)'`
Expected: FAIL — current code records a capture with sentinel `0xFF` and latches nothing.

- [ ] **Step 3: Replace the sentinel with a loud fault**

In `rust/runtime/src/dispatch_stepper.rs`, replace:

```rust
            let motor_idx = found_motor_idx.unwrap_or(0xFF);
```

with:

```rust
            let Some(motor_idx) = found_motor_idx else {
                crate::fault_helpers::raise_phase_motor_unmapped(
                    shared,
                    axis_idx,
                    stepper.stepper_oid,
                );
                return;
            };
```

Then delete the now-stale half of the SAFETY comment above the `phase_stepping_write_xdirect` call ("motor_idx 0xFF is the 'no slot found' sentinel the C side skips gracefully") — `motor_idx` is now always a found slot.

- [ ] **Step 4: Run the full runtime suite**

Run from `rust/`: `cargo nextest run -p runtime`
Expected: PASS, including the other `phase_xdirect_dispatch` tests (they configure slots via `configure_phase_slot` so they are unaffected).

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/dispatch_stepper.rs rust/runtime/tests/phase_xdirect_dispatch.rs
git commit -m "runtime: fail loudly when a Phase stepper has no registered SPI motor"
```

---

### Task 3: `bind_phase_motor` — the production writer for the slot table

**Files:**
- Modify: `rust/runtime/src/state.rs` (next to `set_step_mode`, ~line 540)
- Test: `rust/runtime/src/state/tests.rs`

- [ ] **Step 1: Write failing unit tests**

Append to `rust/runtime/src/state/tests.rs`:

```rust
#[test]
fn bind_phase_motor_installs_slot_and_grows_count() {
    let shared = SharedState::new();
    assert_eq!(super::bind_phase_motor(&shared, 0, 2), Ok(()));
    assert_eq!(super::bind_phase_motor(&shared, 1, 2), Ok(()));
    use core::sync::atomic::Ordering;
    assert_eq!(shared.phase_slot_idx[0].load(Ordering::Acquire), 2);
    assert_eq!(shared.phase_slot_idx[1].load(Ordering::Acquire), 2);
    assert_eq!(shared.phase_motor_count.load(Ordering::Acquire), 2);
    assert_eq!(
        shared.step_modes[2].load(Ordering::Acquire),
        super::StepMode::Modulated as u8,
        "binding a motor marks its kinematic slot Modulated"
    );
}

#[test]
fn bind_phase_motor_is_idempotent_on_count() {
    let shared = SharedState::new();
    assert_eq!(super::bind_phase_motor(&shared, 1, 0), Ok(()));
    assert_eq!(super::bind_phase_motor(&shared, 0, 1), Ok(()));
    use core::sync::atomic::Ordering;
    assert_eq!(
        shared.phase_motor_count.load(Ordering::Acquire),
        2,
        "count is max(motor_idx)+1, not number of calls"
    );
}

#[test]
fn bind_phase_motor_rejects_out_of_range() {
    let shared = SharedState::new();
    assert_eq!(
        super::bind_phase_motor(&shared, super::MAX_STEPPER_OIDS as u8, 0),
        Err(super::SetStepModeError::OutOfRange)
    );
    assert_eq!(
        super::bind_phase_motor(&shared, 0, 4),
        Err(super::SetStepModeError::OutOfRange)
    );
}
```

- [ ] **Step 2: Run them, verify they fail to compile (no such fn)**

Run from `rust/`: `cargo nextest run -p runtime -E 'test(bind_phase_motor)'`
Expected: compile error — `bind_phase_motor` not found.

- [ ] **Step 3: Implement**

In `rust/runtime/src/state.rs`, after `set_step_mode`:

```rust
/// Install the `motor_idx → kinematic slot` mapping `dispatch_phase` uses to
/// address XDIRECT SPI writes, and mark the slot Modulated. Called from the
/// foreground command path (C `runtime_register_phase_motor`), never the ISR.
/// `Release` stores pair with the ISR's `Acquire` loads.
pub fn bind_phase_motor(
    shared: &SharedState,
    motor_idx: u8,
    slot_idx: u8,
) -> Result<(), SetStepModeError> {
    if (motor_idx as usize) >= MAX_STEPPER_OIDS
        || (slot_idx as usize) >= crate::stepping_state::MAX_AXES
    {
        return Err(SetStepModeError::OutOfRange);
    }
    // SAFETY of indexing: both bounds checked above.
    #[allow(clippy::indexing_slicing)]
    shared.phase_slot_idx[motor_idx as usize]
        .store(slot_idx, core::sync::atomic::Ordering::Release);
    let count = shared
        .phase_motor_count
        .load(core::sync::atomic::Ordering::Acquire);
    if motor_idx + 1 > count {
        shared
            .phase_motor_count
            .store(motor_idx + 1, core::sync::atomic::Ordering::Release);
    }
    #[allow(clippy::indexing_slicing)]
    shared.step_modes[slot_idx as usize].store(
        StepMode::Modulated as u8,
        core::sync::atomic::Ordering::Release,
    );
    Ok(())
}
```

(If `crate::stepping_state::MAX_AXES` is not importable from `state.rs` due to module ordering, use the literal bound the engine uses — `stepping_state.rs` defines `MAX_AXES` with `N_AXES` as alias; both are `pub`.)

- [ ] **Step 4: Run the tests**

Run from `rust/`: `cargo nextest run -p runtime -E 'test(bind_phase_motor)'`
Expected: 3 PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/state.rs rust/runtime/src/state/tests.rs
git commit -m "runtime: bind_phase_motor installs phase_slot_idx/phase_motor_count"
```

---

### Task 4: FFI `kalico_runtime_bind_phase_motor` + reset clears the phase map

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` (new fn next to `kalico_runtime_set_step_mode` ~line 520; extend `kalico_runtime_reset` ~line 786)
- Modify: `rust/kalico-c-api/include/kalico_runtime.h`, `rust/kalico-c-api/include/kalico_nurbs.h` (mirror the existing `kalico_runtime_set_step_mode` declaration style)

- [ ] **Step 1: Add the FFI wrapper**

In `rust/kalico-c-api/src/runtime_ffi.rs`, after `kalico_runtime_set_step_mode`:

```rust
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_bind_phase_motor(
        rt: *mut KalicoRuntime,
        motor_idx: u8,
        slot_idx: u8,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_INVALID_HANDLE;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: phase_slot_idx/phase_motor_count/step_modes are atomics in
        // SharedState; shared &SharedState, no &mut. Foreground-only caller.
        unsafe {
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            match runtime::state::bind_phase_motor(shared, motor_idx, slot_idx) {
                Ok(()) => KALICO_OK,
                Err(_) => KALICO_ERR_INVALID_ARG,
            }
        }
    }
```

- [ ] **Step 2: Clear the phase map in `kalico_runtime_reset`**

In `kalico_runtime_reset` (same file, ~line 786), after `(*isr_ptr).engine.reset();` and inside the same `unsafe` block, add:

```rust
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            for slot in shared.phase_slot_idx.iter() {
                slot.store(0xFF, Ordering::Release);
            }
            shared.phase_motor_count.store(0, Ordering::Release);
            for m in shared.step_modes.iter() {
                m.store(runtime::state::StepMode::StepTime as u8, Ordering::Release);
            }
```

Rationale: `kalico_runtime_reset` is sent by the host on every `klippy:connect` before re-registering motors and re-configuring axes (Task 6 moves registration after reset). Without clearing, a config change that removes a phase motor would leave a stale mapping live across reconnects.

- [ ] **Step 3: Declare in both headers**

In `rust/kalico-c-api/include/kalico_runtime.h` and `rust/kalico-c-api/include/kalico_nurbs.h`, next to the existing `kalico_runtime_set_step_mode` declaration, add (match the file's exact typedef name for the runtime handle — copy the neighboring declaration's parameter style):

```c
int32_t kalico_runtime_bind_phase_motor(struct KalicoRuntime *rt,
                                        uint8_t motor_idx, uint8_t slot_idx);
```

- [ ] **Step 4: Build the c-api crate and run its tests**

Run from `rust/`: `cargo nextest run -p kalico-c-api`
Expected: PASS (no behavior change to existing tests; new fn is compile-checked).

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-c-api/src/runtime_ffi.rs rust/kalico-c-api/include/kalico_runtime.h rust/kalico-c-api/include/kalico_nurbs.h
git commit -m "c-api: kalico_runtime_bind_phase_motor; reset clears phase map and step modes"
```

---

### Task 5: Carry `slot_idx` on `runtime_register_phase_motor` (C + bridge + py wrapper)

**Files:**
- Modify: `src/runtime_commands.c:104-119` (`command_runtime_register_phase_motor`)
- Modify: `rust/motion-bridge/src/bridge.rs:1982-2030` (`register_phase_motor` pymethod)
- Modify: `klippy/motion_bridge.py:326-340` (`register_phase_motor` wrapper)
- Modify: `klippy/motion_toolhead.py:780-797` (registration loop — pass the slot)

**Protocol note:** this changes a `DECL_COMMAND` string ⇒ new command dictionary ⇒ both MCUs (H723 + F446) must be reflashed before the next bench session, `make clean` between the two builds. Old firmware + new host will fail command lookup loudly at connect — acceptable on the bench.

- [ ] **Step 1: Extend the C command**

In `src/runtime_commands.c`, replace `command_runtime_register_phase_motor` and its `DECL_COMMAND`:

```c
void
command_runtime_register_phase_motor(uint32_t *args)
{
#if CONFIG_MACH_STM32 || CONFIG_MACH_LINUX
    uint8_t motor_idx = (uint8_t)args[0];
    uint8_t bus_id    = (uint8_t)args[1];
    uint8_t cs_pin_id = (uint8_t)args[2];
    uint8_t slot_idx  = (uint8_t)args[3];
    extern void *runtime_handle;
    if (!runtime_handle)
        shutdown("register_phase_motor before runtime init");
    phase_stepping_register_motor(motor_idx, bus_id, cs_pin_id);
    int32_t rc = kalico_runtime_bind_phase_motor(runtime_handle,
                                                 motor_idx, slot_idx);
    if (rc != 0)
        shutdown("register_phase_motor bind rejected by runtime");
    sendf("kalico_register_phase_motor_response result=%i", 0);
#else
    (void)args;
    sendf("kalico_register_phase_motor_response result=%i", -88);
#endif
}
DECL_COMMAND(command_runtime_register_phase_motor,
    "runtime_register_phase_motor motor_idx=%c bus_id=%c cs_pin_id=%c"
    " slot_idx=%c");
```

The `kalico_runtime_bind_phase_motor` prototype comes from the kalico runtime header already included by this file (it includes the runtime FFI header for the other `kalico_runtime_*` calls — match whichever of `kalico_runtime.h`/`kalico_nurbs.h` the file already uses; add the `#include` only if neither is present).

- [ ] **Step 2: Extend the bridge sender**

In `rust/motion-bridge/src/bridge.rs` `register_phase_motor`: add `slot_idx: u8` to the signature after `cs_pin_id: u8` (and to the `#[pyo3(signature = ...)]` attribute if one lists the parameters), and change the message format to:

```rust
        let msg = format!(
            "runtime_register_phase_motor motor_idx={motor_idx} \
             bus_id={bus_id} cs_pin_id={cs_pin_id} slot_idx={slot_idx}"
        );
```

- [ ] **Step 3: Extend the Python wrapper**

In `klippy/motion_bridge.py`:

```python
    def register_phase_motor(
        self, mcu_handle, motor_idx, bus_id, cs_pin_id, slot_idx, timeout_s=5.0
    ):
        """Call once per phase-stepped motor, AFTER register_phase_bus.
        slot_idx is the kinematic slot whose commanded position drives this
        motor's XDIRECT output."""
        return self._bridge.register_phase_motor(
            mcu_handle,
            motor_idx,
            bus_id,
            cs_pin_id,
            slot_idx,
            timeout_s,
        )
```

(Drop the "BEFORE configure_axes ... configure_axes blob's phase section" sentence from the old docstring — that blob no longer exists.)

- [ ] **Step 4: Pass the slot from the registration loop**

In `klippy/motion_toolhead.py` (~line 780), change the loop variable `_slot_idx` to `slot_idx` and pass it:

```python
                for motor_idx, (bus_id, cs_pin_id, slot_idx) in enumerate(
                    phase_configs,
                ):
                    if bus_id == 0xFF:
                        continue
                    logging.info(
                        "register_phase_motor mcu=%s motor=%d bus=%d cs=%d "
                        "slot=%d",
                        name,
                        motor_idx,
                        bus_id,
                        cs_pin_id,
                        slot_idx,
                    )
                    self.bridge.register_phase_motor(
                        mcu_handle,
                        motor_idx,
                        bus_id,
                        cs_pin_id,
                        slot_idx,
                    )
```

- [ ] **Step 5: Build the workspace, run bridge + runtime tests**

Run from `rust/`: `cargo nextest run -p motion-bridge -p runtime -p kalico-c-api`
Expected: PASS. (If a motion-bridge test constructs `register_phase_motor` calls, update it for the new parameter.)

- [ ] **Step 6: Commit**

```bash
git add src/runtime_commands.c rust/motion-bridge/src/bridge.rs klippy/motion_bridge.py klippy/motion_toolhead.py
git commit -m "phase-stepping: carry slot_idx through runtime_register_phase_motor into the runtime slot table"
```

---

### Task 6: Host sends the real mode byte; registration runs after reset

**Files:**
- Modify: `klippy/motion_toolhead.py` (`_configure_axes_per_mcu`, ~lines 760-886)

- [ ] **Step 1: Move the `any_phase_stepping` registration block after the reset**

Today the order inside the per-MCU loop is: register buses/motors (~760-797) → lookup `kalico_configure_axis` (~803) → send `kalico_runtime_reset` (~822-831) → send `kalico_configure_axis` per axis (~841-886). Since Task 4 makes reset wipe the phase map, registration must follow reset.

Cut the entire `if any_phase_stepping:` block (lines ~760-797, as amended by Task 5) and paste it immediately **after** the `reset_cmd` send block (after line ~831, i.e. after `logging.info("MotionToolhead: sent kalico_runtime_reset ...")`), keeping it before the `axis_bindings` loop. No content changes — pure move. Note the block must stay after the `configure_axis_cmd` lookup `try/except` so an MCU lacking the runtime command path still `continue`s before any registration.

- [ ] **Step 2: Send the real per-axis mode**

In the same function, replace the hardcoded constants and the send call. Where it currently reads:

```python
            MODE_PULSE = 0
            TMC_CS_OID_NONE = 0xFF
            FLAGS_DEFAULT = 0
```

change to:

```python
            MODE_PULSE = 0  # wire encoding: 0=Pulse, 1=Phase
            MODE_PHASE = 1  # (host step_modes list: 0=Modulated, 1=StepTime)
            TMC_CS_OID_NONE = 0xFF
            FLAGS_DEFAULT = 0
```

and in `configure_axis_cmd.send(...)` (~line 876) replace the literal `MODE_PULSE` argument:

```python
                axis_mode = (
                    MODE_PHASE if step_modes[axis_idx] == 0 else MODE_PULSE
                )
                configure_axis_cmd.send(
                    [
                        axis_idx,
                        axis_mode,
                        microstep_bits,
                        extrusion_bits,
                        len(bindings),
                        ring_depth,
                        bytes(blob),
                    ]
                )
```

(`step_modes[axis_idx] == 0` means Modulated, i.e. phase stepping — the guard at line ~683 sets it only when `phase_stepping=True` and capability checks passed. The wire/runtime side maps `1 → stepping_state::StepMode::Phase` in `kalico_runtime_configure_axis`, `rust/kalico-c-api/src/runtime_ffi.rs:738`.)

- [ ] **Step 3: Syntax check**

Run: `python3 -m py_compile klippy/motion_toolhead.py klippy/motion_bridge.py`
Expected: no output, exit 0.

- [ ] **Step 4: Commit**

```bash
git add klippy/motion_toolhead.py
git commit -m "host: send Phase mode byte in kalico_configure_axis; register phase motors after runtime reset"
```

---

### Task 7: Delete the dead `ConfigureAxes` bridge path and the unused `phase_config` storage

The `ConfigureAxes` (0x0030) kalico-protocol message is dead: the MCU's `kalico_dispatch_frame` (`src/kalico_dispatch.c:180`) only handles Identify / QueryRuntimeCaps / Stop / ResumeStream, and no Python caller invokes the wrapper. Leaving it means a second, never-delivered carrier of `phase_configs`. Keep the `MessageKind` enum entries and `src/kalico_protocol_schema.h` defines (wire-stable ID table); delete only the dead host-side surface.

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` — delete `build_configure_axes_body` (~line 517), the `configure_axes` pymethod (~line 1811), and the `mod build_configure_axes_body_tests;` declaration (~line 3917)
- Delete: `rust/motion-bridge/src/bridge/build_configure_axes_body_tests.rs`
- Modify: `klippy/motion_bridge.py` — delete the `configure_axes` method (~line 283) and the `"configure_axes"` entry in the method-name list (~line 43)
- Modify: `rust/runtime/src/state.rs` — delete the `phase_config` field (~line 125) and its 16-element initializer (~line 300); `rust/runtime/src/lib.rs` — delete `pub mod phase_config;` (line 41)
- Delete: `rust/runtime/src/phase_config.rs` and `rust/runtime/src/phase_config/` (its tests)

- [ ] **Step 1: Delete the bridge method, body builder, and tests module**

Remove the three bridge.rs items and delete the tests file listed above.

- [ ] **Step 2: Delete the Python wrapper + whitelist entry**

Remove `configure_axes` from `klippy/motion_bridge.py` (method and the string in the delegated-methods list).

- [ ] **Step 3: Delete `phase_config` storage**

Remove the `SharedState.phase_config` field and initializer, the `pub mod phase_config;` export, and the module files. Run `grep -rn "phase_config" rust/ klippy/ src/ --include="*.rs" --include="*.py" --include="*.c"` — remaining hits must only be `phase_configs` (plural, the motion_toolhead local) and this plan document.

- [ ] **Step 4: Build + full workspace test**

Run from `rust/`: `cargo nextest run`
Run: `python3 -m py_compile klippy/motion_bridge.py`
Expected: PASS / clean. Fix any straggler references the compiler finds (e.g. `kalico-c-api` tests touching `phase_config`).

- [ ] **Step 5: Commit**

```bash
git add -A rust/motion-bridge rust/runtime klippy/motion_bridge.py
git commit -m "remove: dead ConfigureAxes bridge path and unused SharedState.phase_config"
```

---

### Task 8: Full verification, flash, bench check

- [ ] **Step 1: Full suite + doctests + fmt**

Run from `rust/`:
```bash
cargo nextest run
cargo test --doc
cargo fmt --all --check
```
Expected: all green. `cargo fmt --all --check` is the last gate — re-run it after any late edit.

- [ ] **Step 2: Push and flash both MCUs**

Use the `flashing-trident-mcus` skill (canonical commit → push → pull on Pi → build host cdylib → build + flash H723 from `.config.h7.bak` and F446 from `.config.f446.test`, `make clean` between). Both MCUs must be flashed — the `DECL_COMMAND` change in Task 5 changes the command dictionary on both.

- [ ] **Step 3: Bench validation — diagnostics before motion**

After the printer reconnects, with no G-code issued:
1. Query VictoriaLogs (query-logs skill) for the new session: expect `register_phase_motor ... slot=N` host log lines, no `PhaseMotorUnmapped` fault, no `configure_axis rejected`.
2. `KALICO_DIAG_DUMP` (mcu-diagnostics skill): confirm the axis mode shows Phase for the phase-stepped axes and `isr_phase_call_count` is incrementing while idle-holding.
3. Only then ask the user to jog — **never issue motion G-code without explicit per-command permission**. Success: physical motion on the phase-stepped axis; `phase_spi_skip_count` in `kalico_status` near zero; no fault latched.

- [ ] **Step 4: Open PR**

Target branch `sota-motion` (not `main`). Run `cargo fmt --all --check` once more if anything changed since Step 1. No Claude/Anthropic attribution in commits or PR body.

---

## Self-review notes

- Spec coverage: mode byte (Task 6), slot mapping writer (Tasks 3-5), fail-loudly (Tasks 1-2), dead-path removal (Task 7), encoding-mismatch hazard documented in header + Task 6, reconnect/reset ordering (Tasks 4 + 6), dual-MCU protocol compatibility (Task 5 note + Task 8).
- Known judgment calls an executor must not "fix" silently: `kalico_runtime_configure_axis`'s Modulated→Phase promotion block stays (it now agrees with `bind_phase_motor`'s `step_modes` store — defense in depth, single direction); `MessageKind::ConfigureAxes` enum entries stay (wire-stable ID table).
- `bind_phase_motor` writes `step_modes[slot] = Modulated` so `SharedState` stays truthful even though the host now also sends the explicit mode byte; reset reverts both.
