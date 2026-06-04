# MCU Runtime Reset Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an idempotent `Engine::reset()` the host issues on every `klippy:connect` (before reconfiguring axes) so the MCU's never-reset ring bump allocator can't overflow on a reconnect-without-reboot.

**Architecture:** A pure `Engine::reset()` rewinds `ring_alloc_cursor` and clears all per-axis motion state to its post-construction baseline (preserving the immutable clock config + running tick counter). An FFI `kalico_runtime_reset` projects `&mut IsrState`, calls `engine.reset()`, and (MCU builds only) zeroes the four C-owned per-axis step queues. A thin C `DECL_COMMAND` wraps the FFI call in an `irq_save`/`irq_restore` critical section (blocks both the TIM5 sample ISR and the per-axis step-event timers). The host sends the command once per MCU before its `configure_axis` loop.

**Tech Stack:** Rust (`no_std` engine + `staticlib` FFI), cbindgen-generated C header, Klipper C MCU command surface, Klipper Python host (`klippy/`).

**Spec:** `docs/superpowers/specs/2026-05-30-mcu-runtime-reset-design.md`

**Local vs bench split:** Tasks 1–3 are Rust, fully host-testable locally (`cargo test`). Tasks 4–5 (C handler, Python host) are validated by the firmware build + bench, per the project rule "commit → push → pull → compile on Pi → flash" (never cross-compile MCU firmware locally). Task 6 is the bench build/flash/verify.

---

### Task 1: `Engine::reset()` (pure, host-tested)

**Files:**
- Modify: `rust/runtime/src/engine.rs` (add `reset` method to the `impl Engine` block, near `configure_axis` ~line 230)
- Test: `rust/runtime/tests/runtime_reset.rs` (create)

- [ ] **Step 1: Write the failing test**

Create `rust/runtime/tests/runtime_reset.rs`:

```rust
//! Coverage for `Engine::reset()` — the host-issued, idempotent clean-state
//! reset that rewinds the ring bump allocator on every (re)connect.

use runtime::engine::{Engine, RuntimeStatus};
use runtime::stepping_state::{StepMode, StepperBindingRust, TMC_CS_OID_NONE};

fn new_engine() -> Engine {
    Engine::new(520_000_000, 40_000)
}

fn pulse_binding() -> StepperBindingRust {
    StepperBindingRust { stepper_oid: 0, tmc_cs_oid: TMC_CS_OID_NONE, _pad: [0; 2] }
}

#[test]
fn reset_clears_axis_state() {
    let mut e = new_engine();
    let b = pulse_binding();
    assert_eq!(e.configure_axis(0, StepMode::Pulse, 0.0125, 64, &[b], 512), 0);
    assert_eq!(e.configure_axis(1, StepMode::Pulse, 0.0125, 64, &[b], 512), 0);
    assert_eq!(e.num_axes, 2);

    e.reset();

    assert_eq!(e.num_axes, 0, "num_axes not cleared");
    assert!(e.stepping_axes.iter().all(|a| a.is_none()), "axes not cleared");
    assert_eq!(e.status(), RuntimeStatus::Idle, "status not Idle");
    assert_eq!(e.last_error(), 0, "last_error not cleared");
}

#[test]
fn reset_reclaims_ring_allocation() {
    let mut e = new_engine();
    let b = pulse_binding();
    // Fill the 512-piece pool: two 256-deep axes -> cursor at 512.
    assert_eq!(e.configure_axis(0, StepMode::Pulse, 0.0125, 256, &[b], 512), 0);
    assert_eq!(e.configure_axis(1, StepMode::Pulse, 0.0125, 256, &[b], 512), 0);
    // A third allocation must now overflow (the bug, pre-reset).
    assert_ne!(
        e.configure_axis(2, StepMode::Pulse, 0.0125, 256, &[b], 512), 0,
        "expected RING_FULL before reset"
    );

    e.reset();

    // After reset the cursor is rewound, so the same configuration fits again.
    assert_eq!(e.configure_axis(0, StepMode::Pulse, 0.0125, 256, &[b], 512), 0);
    assert_eq!(e.configure_axis(1, StepMode::Pulse, 0.0125, 256, &[b], 512), 0);
}

#[test]
fn reset_is_idempotent_on_fresh_engine() {
    let mut e = new_engine();
    e.reset();
    let b = pulse_binding();
    // A fresh reset must not consume allocator space — a full-pool axis fits.
    assert_eq!(e.configure_axis(0, StepMode::Pulse, 0.0125, 512, &[b], 512), 0);
}

#[test]
fn reset_preserves_clock_config() {
    let mut e = new_engine();
    let sample_period = e.sample_period_cycles;
    let cps = e.cycles_per_second;
    e.reset();
    assert_eq!(e.sample_period_cycles, sample_period, "sample period changed");
    assert!((e.cycles_per_second - cps).abs() < f32::EPSILON, "cycles/s changed");
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cargo test -p runtime --test runtime_reset`
Expected: FAIL — compile error `no method named reset found for struct Engine`.

- [ ] **Step 3: Write the minimal implementation**

In `rust/runtime/src/engine.rs`, add to the `impl Engine` block (after `configure_axis`, ~line 230):

```rust
    /// Reset the engine to a clean, just-initialized motion state.
    ///
    /// Issued by the host on every (re)connect before reconfiguring axes, so
    /// the bump allocator (`ring_alloc_cursor`) and all per-axis state start
    /// fresh regardless of whether the MCU was rebooted. Idempotent: on a
    /// freshly-constructed engine this is a no-op.
    ///
    /// Preserves the immutable hardware config (`sample_period_cycles`,
    /// `cycles_per_second`) and the running `tick_counter` clock — resetting
    /// those would desync the ISR time base.
    ///
    /// The per-axis C step queues live outside the engine and are cleared
    /// separately by the FFI caller (`kalico_runtime_reset`).
    pub fn reset(&mut self) {
        self.ring_alloc_cursor = 0;
        self.stepping_axes = [const { None }; MAX_AXES];
        self.num_axes = 0;
        self.step_state = [StepMotorState::default(); MAX_AXES];
        self.last_motors = [0.0; MAX_AXES];
        self.tick_caches = crate::stepping_state::TickCaches::new();
        self.status.store(RuntimeStatus::Idle as u8, Ordering::Release);
        self.last_error.store(0, Ordering::Release);
    }
```

(`Ordering`, `MAX_AXES`, `StepMotorState`, `RuntimeStatus` are already imported at the top of `engine.rs`.)

- [ ] **Step 4: Run the test to verify it passes**

Run: `cargo test -p runtime --test runtime_reset`
Expected: PASS (4 tests).

- [ ] **Step 5: Run the existing engine tests to confirm no regression**

Run: `cargo test -p runtime`
Expected: PASS (existing `configure_axis`, `piece_tick`, etc. unaffected).

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/engine.rs rust/runtime/tests/runtime_reset.rs
git commit -m "feat(runtime): add Engine::reset() to rewind ring bump allocator"
```

---

### Task 2: Step-queue clear (`StepQueue::clear` + MCU-gated `reset_all_queues`)

**Files:**
- Modify: `rust/runtime/src/step_queue.rs` (add `clear` method, `reset_all_queues` fn, `#[cfg(test)] mod tests`)

- [ ] **Step 1: Write the failing test**

At the end of `rust/runtime/src/step_queue.rs`, add:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clear_empties_queue() {
        let mut q = StepQueue::new();
        // 3 entries outstanding (tail ahead of head).
        q.tail = 5;
        q.head = 2;
        assert_ne!(q.tail, q.head);
        q.clear();
        assert_eq!(q.tail, 0);
        assert_eq!(q.head, 0);
    }
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cargo test -p runtime --lib step_queue`
Expected: FAIL — compile error `no method named clear found`.

- [ ] **Step 3: Write the minimal implementation**

In `rust/runtime/src/step_queue.rs`, add a `clear` method to the existing `impl StepQueue` block (after `new`, ~line 81):

```rust
impl StepQueue {
    /// Empty the queue by resetting both SPSC counters to 0.
    ///
    /// The caller must hold exclusive access (an IRQ guard): both the producer
    /// (writes `tail`) and the consumer (writes `head`) must be quiescent.
    #[inline]
    pub fn clear(&mut self) {
        self.tail = 0;
        self.head = 0;
    }
}
```

Then add the MCU-only bulk reset (after `queue_for_axis`, ~line 124):

```rust
/// Clear all per-axis step queues. MCU-only — host/test builds keep their
/// queues in `Engine::test_queue_ptrs` and have no `step_queues` global.
///
/// The caller (`kalico_runtime_reset`) holds the IRQ guard, so no producer
/// ISR or consumer timer runs concurrently with these writes.
#[cfg(not(any(test, feature = "host")))]
pub fn reset_all_queues() {
    for i in 0..N_AXIS_STEP_QUEUES {
        let q = queue_for_axis(i);
        // SAFETY: `i < N_AXIS_STEP_QUEUES` so `q` is non-null and points at a
        // live `StepQueue`; the IRQ guard guarantees exclusive access.
        unsafe { (*q).clear(); }
    }
}
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cargo test -p runtime --lib step_queue`
Expected: PASS. (`reset_all_queues` is `#[cfg]`-compiled out under `test`; it is compile-checked by the firmware build in Task 6.)

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/step_queue.rs
git commit -m "feat(runtime): add StepQueue::clear + MCU reset_all_queues"
```

---

### Task 3: FFI `kalico_runtime_reset` + regenerated header

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` (add the extern fn after `kalico_runtime_configure_axis`, ~line 1433)
- Modify: `rust/kalico-c-api/include/kalico_runtime.h` (regenerated — do NOT hand-edit)
- Test: `rust/kalico-c-api/tests/runtime_reset.rs` (create)

- [ ] **Step 1: Write the failing test**

Create `rust/kalico-c-api/tests/runtime_reset.rs`:

```rust
//! FFI tests for `kalico_runtime_reset`.
//!
//! Each integration-test file is its own binary, so `INIT_DONE` (a
//! process-global) is fresh here. The shared handle + TEST_LOCK pattern mirrors
//! `write_piece.rs`.

#![allow(unsafe_code, non_upper_case_globals)]

use std::sync::{Mutex, OnceLock};

// --- Host-side linker stubs (each test binary links independently) ----------
#[unsafe(no_mangle)]
pub static runtime_clock_freq: u32 = 520_000_000;
#[unsafe(no_mangle)]
pub static runtime_sample_rate_hz: u32 = 40_000;
#[unsafe(no_mangle)]
pub extern "C" fn runtime_cyccnt_read() -> u32 { 0 }
#[unsafe(no_mangle)]
pub extern "C" fn runtime_diag_progress(_tag: u32, _stage: u32, _value: u32) {}

// --- Handle setup -----------------------------------------------------------
static TEST_LOCK: Mutex<()> = Mutex::new(());

struct RtHandle(*mut kalico_c_api::KalicoRuntime);
// SAFETY: all FFI calls are serialised by TEST_LOCK; the FFI's own INIT_DONE +
// null-ptr guards add a second layer. See write_piece.rs for the full rationale.
unsafe impl Send for RtHandle {}
unsafe impl Sync for RtHandle {}

static RUNTIME: OnceLock<RtHandle> = OnceLock::new();

fn rt() -> *mut kalico_c_api::KalicoRuntime {
    RUNTIME
        .get_or_init(|| {
            let handle = kalico_c_api::runtime_handle_create();
            assert!(!handle.is_null(), "runtime_handle_create returned null");
            RtHandle(handle)
        })
        .0
}

// --- Tests ------------------------------------------------------------------

#[test]
fn reset_reclaims_allocation_across_many_configures() {
    let _g = TEST_LOCK.lock().unwrap();
    let handle = rt();
    // Without a working reset, repeated allocation exhausts the bump allocator
    // (total pool is ~1984 pieces). With reset before each configure, all 128
    // iterations (128*64 = 8192 pieces of demand) succeed.
    for i in 0..128 {
        unsafe {
            let rc = kalico_c_api::kalico_runtime_reset(handle);
            assert_eq!(rc, kalico_c_api::KALICO_OK, "reset failed at iter {i}");
            let rc = kalico_c_api::kalico_runtime_configure_axis(
                handle,
                0,                               // axis_idx
                0,                               // mode = Pulse
                (1.0_f32 / 160.0_f32).to_bits(), // microstep_distance bits
                64,                              // ring_depth
                core::ptr::null(),               // bindings_ptr
                0,                               // stepper_count
            );
            assert_eq!(rc, kalico_c_api::KALICO_OK, "configure failed at iter {i}");
        }
    }
}

#[test]
fn reset_null_rt_is_null_ptr_error() {
    let _g = TEST_LOCK.lock().unwrap();
    unsafe {
        let rc = kalico_c_api::kalico_runtime_reset(core::ptr::null_mut());
        assert_eq!(rc, kalico_c_api::KALICO_ERR_NULL_PTR);
    }
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cargo test -p kalico-c-api --test runtime_reset`
Expected: FAIL — `cannot find function kalico_runtime_reset in crate kalico_c_api`.

- [ ] **Step 3: Write the FFI implementation**

In `rust/kalico-c-api/src/runtime_ffi.rs`, add after `kalico_runtime_configure_axis` (~line 1433):

```rust
    /// Reset the motion engine to a clean state — issued by the host on every
    /// (re)connect before reconfiguring axes. Rewinds the ring bump allocator
    /// and clears all per-axis state so re-sent `configure_axis` commands never
    /// overflow `piece_storage` on a reconnect-without-reboot.
    ///
    /// Must be called from foreground inside an IRQ-disabled window (the C
    /// command handler holds `irq_save`/`irq_restore`): the engine state and the
    /// per-axis step queues this clears are concurrently touched by the TIM5
    /// sample ISR and the per-axis step-event timers.
    ///
    /// Returns `KALICO_OK` (0), `KALICO_ERR_NULL_PTR`, or `KALICO_ERR_NOT_INIT`.
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_reset(rt: *mut KalicoRuntime) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: foreground-only entry under the C-side IRQ guard; spec §11.2
        // raw-pointer projection. No other `&mut IsrState` may be live.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.reset();
        }
        // Clear the C-owned per-axis step queues (MCU build only); host/test
        // builds have no `step_queues` global.
        #[cfg(not(any(test, feature = "host")))]
        runtime::step_queue::reset_all_queues();
        KALICO_OK
    }
```

(`KALICO_ERR_NULL_PTR`, `KALICO_ERR_NOT_INIT`, `KALICO_OK`, `INIT_DONE`, `Ordering`, `RuntimeContext`, `IsrState`, `UnsafeCell` are already in scope — used by the sibling FFI fns.)

- [ ] **Step 4: Regenerate the cbindgen header**

Run: `./tools/regen_headers.sh`
Expected: `rust/kalico-c-api/include/kalico_runtime.h` now declares
`int32_t kalico_runtime_reset(struct KalicoRuntime *rt);`.

- [ ] **Step 5: Run the FFI test + header-drift test**

Run: `cargo test -p kalico-c-api --test runtime_reset`
Expected: PASS (2 tests).

Run: `cargo test -p kalico-c-api --no-default-features --features host,header-runtime --test headers_no_drift`
Expected: PASS (committed header matches regenerated).

- [ ] **Step 6: Commit**

```bash
git add rust/kalico-c-api/src/runtime_ffi.rs \
        rust/kalico-c-api/include/kalico_runtime.h \
        rust/kalico-c-api/tests/runtime_reset.rs
git commit -m "feat(ffi): add kalico_runtime_reset (engine + step-queue clear)"
```

---

### Task 4: C command `kalico_runtime_reset` (IRQ-guarded)

**Files:**
- Modify: `src/stepper.c` (add handler + `DECL_COMMAND` after the `configure_axis` `DECL_COMMAND`, ~line 324)

This is firmware C; it is compile-validated by the Pi build in Task 6 (no local cross-compile). `irq_save`/`irq_restore`/`irqstatus_t` come from `board/irq.h` (already included, `stepper.c:16`); `kalico_runtime_reset` is declared in the regenerated `kalico_runtime.h` (already included, `stepper.c:22`); `runtime_handle` is the `extern void *` at `stepper.c:217`.

- [ ] **Step 1: Add the command handler**

In `src/stepper.c`, after the `DECL_COMMAND(command_kalico_configure_axis, ...)` block (~line 324):

```c
// Host-issued clean-state reset. Sent once per MCU on every klippy:connect,
// before the per-axis configure_axis calls, so the Rust engine's ring bump
// allocator (and all per-axis state) starts fresh whether or not the MCU was
// rebooted. Idempotent: a no-op on a freshly-booted MCU.
//
// IRQ guard: the reset clears engine state + the per-axis step queues, both of
// which are concurrently touched by the always-armed TIM5 sample ISR and the
// per-axis step-event timers. irq_save() blocks both for the bounded reset.
void
command_kalico_runtime_reset(uint32_t *args)
{
    (void)args;
    if (!runtime_handle)
        shutdown("runtime reset before runtime init");
    irqstatus_t flag = irq_save();
    int32_t rc = kalico_runtime_reset(runtime_handle);
    irq_restore(flag);
    if (rc != 0)
        shutdown("runtime reset rejected");
}
DECL_COMMAND(command_kalico_runtime_reset, "kalico_runtime_reset");
```

- [ ] **Step 2: Commit (build validation deferred to Task 6)**

```bash
git add src/stepper.c
git commit -m "feat(mcu): add kalico_runtime_reset command (IRQ-guarded)"
```

---

### Task 5: Host wiring — send reset before the configure loop

**Files:**
- Modify: `klippy/motion_toolhead.py` (in `_configure_axes_per_mcu`, after the `configure_axis_cmd` lookup ~line 1166, before the `axis_bindings` build ~line 1168)

- [ ] **Step 1: Insert the per-MCU reset send**

In `klippy/motion_toolhead.py`, immediately after the `configure_axis_cmd` lookup `try/except` block (the one ending with the `continue` at ~line 1166) and before `# Group bind_list by axis` (~line 1168), add:

```python
            # Clean-state reset before (re)configuring this MCU's axes. The
            # engine's ring bump allocator never frees, and configure_axis is
            # re-sent on every klippy:connect; without this reset a plain
            # RESTART / systemctl restart / crash-reconnect (which does NOT
            # reboot bridge MCUs) overflows the pool -> KALICO_ERR_RING_FULL.
            # Idempotent: a no-op on a freshly-booted MCU. Same command queue,
            # so it is processed before the configure_axis commands below.
            try:
                reset_cmd = mcu_obj.lookup_command("kalico_runtime_reset")
            except Exception:
                reset_cmd = None
            if reset_cmd is not None:
                reset_cmd.send([])
                logging.info(
                    "MotionToolhead: sent kalico_runtime_reset to mcu=%s",
                    name,
                )
```

- [ ] **Step 2: Sanity-check Python parses**

Run: `python3 -c "import ast; ast.parse(open('klippy/motion_toolhead.py').read())"`
Expected: no output (file parses).

- [ ] **Step 3: Commit**

```bash
git add klippy/motion_toolhead.py
git commit -m "feat(host): reset MCU runtime on connect before configure_axis"
```

---

### Task 6: Bench build, flash, and verify

Per the project rule: commit → push → pull on the Pi → compile on the Pi → flash. Both MCUs run the fork's firmware and must be flashed (H7 from `.config.h7.bak`, F446 from `.config.f446.test`). `make clean` between the two C builds.

- [ ] **Step 1: Push the branch**

```bash
git push
```

- [ ] **Step 2: On the Pi — pull and build the H7**

```bash
ssh dderg@trident.local
cd ~/<repo>            # the fork checkout on the Pi
git fetch && git checkout simple-mcu-contract && git pull
cp .config.h7.bak .config
make clean
make -j$(nproc)
```
Expected: build succeeds; `kalico_runtime_reset` C handler + FFI link cleanly. (This is where `reset_all_queues` and the C handler are first compiled.)

- [ ] **Step 3: Flash the H7**, then build + flash the F446

```bash
# flash H7 per the saved flow, then:
cp .config.f446.test .config
make clean
make -j$(nproc)
# flash F446 per the saved flow
```

- [ ] **Step 4: Verify the regression is fixed (no motion required)**

With klippy running, restart it twice in a row without a firmware restart, e.g.:
```
RESTART
```
then again `RESTART` (or `systemctl restart klipper` twice). Before this change the second connect shut the MCU down with `configure_axis rejected by runtime`.

Fetch and inspect the log:
```bash
cp ~/printer_data/logs/klippy.log /tmp/klippy-$(date +%s).log
```
Expected in the log across both restarts:
- `MotionToolhead: sent kalico_runtime_reset to mcu=...` once per MCU per connect.
- `configure_axes mcu=...` succeeds each connect.
- No `configure_axis rejected by runtime` / `RING_FULL` shutdown.

- [ ] **Step 5: Confirm a normal FIRMWARE_RESTART path still works**

```
FIRMWARE_RESTART
```
Expected: MCUs reboot and reconnect cleanly (reset is a harmless no-op on the freshly-booted engine).

---

## Self-Review

**Spec coverage:**
- `Engine::reset()` field clear/preserve list → Task 1 (impl + `reset_preserves_clock_config`). ✓
- FFI `kalico_runtime_reset` + projection + MCU step-queue clear → Task 3 + Task 2. ✓
- Step-queue clear (`StepQueue::clear`, `reset_all_queues`) → Task 2. ✓
- IRQ guard in C command handler → Task 4. ✓
- Host send once per MCU before the configure loop → Task 5. ✓
- cbindgen header regeneration + drift test → Task 3 Steps 4–5. ✓
- Consistency invariant (host pump rebuilt by `init_planner`) — no code change required; documented in spec. ✓
- Regression reproduction (configure → reset → configure) → Task 1 `reset_reclaims_ring_allocation` + Task 3 FFI loop + Task 6 bench. ✓

**Placeholder scan:** No TBD/TODO; every code step shows complete code; every run step has an exact command + expected result. ✓

**Type consistency:** `Engine::reset()` (Task 1) ↔ `engine.reset()` (Task 3 FFI). `StepQueue::clear` (Task 2) ↔ `(*q).clear()` (Task 2 `reset_all_queues`). `kalico_runtime_reset` name identical across Task 3 (FFI), Task 4 (C handler call + `DECL_COMMAND` string), Task 5 (host `lookup_command`). `reset_all_queues` gating `#[cfg(not(any(test, feature = "host")))]` identical in Task 2 (definition) and Task 3 (call site). ✓
