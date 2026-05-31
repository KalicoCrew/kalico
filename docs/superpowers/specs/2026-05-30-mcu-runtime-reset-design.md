# MCU runtime reset on host (re)connect

**Date:** 2026-05-30
**Status:** Design — approved, pending spec review
**Area:** MCU motion engine (`rust/runtime`, `rust/kalico-c-api`), C command surface (`src/stepper.c`), host init (`klippy/motion_toolhead.py`)

## Problem

The MCU engine allocates each axis's piece-ring region with a **bump allocator** that
only ever grows:

```rust
// rust/runtime/src/engine.rs:196-201
if self.ring_alloc_cursor + ring_depth > total_ring_pieces {
    return KALICO_ERR_RING_FULL;
}
let offset = self.ring_alloc_cursor;
self.ring_alloc_cursor += ring_depth;
```

`kalico_configure_axis` is a runtime `DECL_COMMAND` (not a CRC-gated config command),
and the host re-emits it for every axis on **every `klippy:connect`**
(`motion_toolhead.py:367-368` → `_init_planner` → `_configure_axes_per_mcu` →
`configure_axis_cmd.send(...)`).

`ring_alloc_cursor` is only zeroed when the engine is constructed — i.e. on an MCU
reboot (`Engine::new` / `init_in_place`). A plain host restart does **not** reboot the
MCU: bridge MCUs deliberately survive it (`mcu.py:1841-1844` early-returns
`_firmware_restart` for bridge MCUs unless forced; the bridge registers its restart
only on `klippy:firmware_restart`, `mcu.py:1575`). Only `FIRMWARE_RESTART` reboots
them.

So on any reconnect-without-reboot — the in-console `RESTART` gcode, a
`systemctl restart klipper`, or a klippy crash + reconnect — the cursor is already at
or near `total_ring_pieces`, the re-emitted `configure_axis` overflows, returns
`KALICO_ERR_RING_FULL`, and the C handler calls
`shutdown("configure_axis rejected by runtime")`.

The case that actually bites in practice is the **service / cold-process restart**: a
fresh klippy process connects to an MCU that is still running with stale engine state.

## How mainline avoids this (and why we differ)

Mainline Klipper's MCU allocator is *also* a never-freed bump allocator (`alloc_end`
in `src/basecmd.c:31-39`). Mainline does not deallocate; it makes allocation
**one-shot-per-boot** and gates re-config on a reset:

- All allocation lives in the config command list, sent once per boot. On reconnect
  the host reads back `is_config` + `crc` via `get_config` (`mcu.py:1377-1384`).
- **CRC match** (plain restart, unchanged config): the host sends only `_restart_cmds`
  and **skips** the config list / `allocate_oids` (`mcu.py:1359-1363`). Allocation is
  never re-run.
- **CRC mismatch / `is_config=0`**: the host triggers a **firmware restart**
  (`mcu.py:1262-1271`), then sends config fresh against a zeroed allocator.
- Re-allocating a live, already-configured MCU is structurally blocked by
  `is_finalized()` (`!!move_count`, `basecmd.c:146,236`). `config_reset`
  (`basecmd.c:295-310`) *does* rewind `alloc_end`, but only from MCU shutdown state.

Our bridge MCUs bypass this machinery entirely: `kalico_configure_axis` is sent as a
runtime command on every connect, with no CRC reuse and no finalize gate. That is the
divergence that exposes the bug.

## Decision

Add a **soft, idempotent runtime reset** that the host issues on every
`klippy:connect`, before reconfiguring. It returns the MCU motion engine to its
just-booted clean state **without a reboot**.

This is the live-reconnect analog of mainline's `config_reset` (which also rewinds the
bump allocator) — except we can run it on a *live* MCU under a brief IRQ-disabled
window, because the motion engine's state is small and cleanly re-initializable; we do
not need the full shutdown+reboot mainline requires.

### Alternatives rejected

- **Adopt mainline's CRC-gated config model (option B).** Most "pure," but: (1) our
  configure path is deliberately dynamic — ring depth derived from runtime caps,
  shaper type/frequency the user can tune — and the CRC model forces a full firmware
  reboot on *any* such change; (2) even with CRC-reuse we would still need a
  per-connect runtime reset of ring cursors / `consumed` counters to stay consistent
  with the host's freshly-rebuilt pump accounting (mainline's `_restart_cmds` are
  exactly that). So B is strictly more work and still needs this reset underneath.
  Large lift for a worse fit.

- **Reboot the MCU on every host restart (option C).** Conceptually "fresh MCU every
  restart," but it cannot be a simple gate-flip: a service / cold-process restart fires
  **no** restart event (the new process just connects), so the reboot would have to be
  driven from the connect path conditioned on detecting stale state — and because a
  reboot drops the link, it needs loop-prevention (connect → reboot → reconnect must
  not reboot again). That is mainline's `is_config` / `start_reason` state machine
  reimplemented for the bridge — *more* logic than option A, not less. A soft reset has
  no such side effect: it is idempotent (a no-op on a freshly-booted MCU) so it can be
  sent unconditionally on every connect with no detection and no loop.

- **Reset MCUs on host shutdown.** Fragile: `systemctl restart`, `kill -9`, a crash,
  or a Pi reboot that does not power-cycle the MCU all skip any shutdown hook. The
  robust contract lives at connect (always happens), not shutdown.

## Design

### 1. `Engine::reset()` — `rust/runtime/src/engine.rs`

Pure, host-testable method that resets the engine's mutable motion state to its
post-construction values while preserving hardware-derived immutables.

**Clear:**
- `ring_alloc_cursor → 0` (the fix)
- `stepping_axes → [None; MAX_AXES]`
- `num_axes → 0`
- `step_state → [StepMotorState::default(); MAX_AXES]`
- `last_motors → [0.0; MAX_AXES]`
- `tick_caches → TickCaches::new()`
- `status → RuntimeStatus::Idle` (store)
- `last_error → 0` (store)

**Preserve:**
- `sample_period_cycles`, `cycles_per_second` — immutable hardware config set at init.
- `tick_counter` — the hardware time base; resetting it would desync the ISR clock.

**Not touched:**
- `piece_storage` bytes — ring cursors are zeroed, so stale slots are never read; no
  need to clear ~62 KB.
- `SharedState` fault subsystem — separate lifecycle with its own clear path.
- `test_queue_ptrs` — host/test-only field.

The method is queue-agnostic (it does not know about the C-side step queues); the FFI
orchestrates step-queue clearing separately (§2) so `Engine::reset()` stays a pure,
host-testable unit.

This is a "full reset" per the agreed scope: it returns the engine to the same state
`init_in_place` leaves it in, minus the immutable HW config and the running clock.

### 2. FFI `kalico_runtime_reset` — `rust/kalico-c-api/src/runtime_ffi.rs`

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_reset(rt: *mut KalicoRuntime) -> i32
```

- Null + `INIT_DONE` checks (return `KALICO_ERR_NULL_PTR` / `KALICO_ERR_NOT_INIT`),
  mirroring `kalico_runtime_configure_axis`.
- Project `&mut IsrState` via `UnsafeCell::raw_get(addr_of!((*ctx).isr))` (same pattern
  as `configure_axis`), call `(*isr_ptr).engine.reset()`.
- Clear the per-axis step queues (MCU build only): see §3.
- Return `KALICO_OK`.

The FFI body assumes it runs inside the IRQ-disabled window the C caller establishes
(§4); it performs only memory writes, no blocking.

After regenerating, the prototype lands in the cbindgen header (do **not** hand-edit;
regenerate via `cargo run -p kalico-c-api --bin gen-headers`).

### 3. Step-queue clear — `rust/runtime/src/step_queue.rs`

The per-axis step queues (`[StepQueue; N_AXIS_STEP_QUEUES]` in `.axi_bss`) sit *outside*
the engine: filled by `engine.tick` (producer) and drained by the per-axis Klipper
timers (consumer). `Engine::reset()` does not touch them, so without an explicit clear,
a reset after a crash mid-print could leave queued step entries that the per-axis
timers would still emit as phantom pulses after reconnect.

Add an MCU-gated helper:

```rust
#[cfg(not(any(test, feature = "host")))]
pub fn reset_all_queues() {
    // For each queue: head = tail = 0 (empties it). Safe to write both
    // counters because the caller holds the IRQ guard, so neither the
    // producer ISR (tail) nor the consumer timer (head) runs concurrently.
}
```

`StepQueue` has no `count` field — emptiness is `tail == head`, so zeroing both
counters is a complete clear. The FFI (§2) calls `reset_all_queues()` on MCU builds;
on host/test builds there is no `step_queues` global, so the call is `#[cfg]`-compiled
out (matching how `queue_for_axis` / `resolve_queue_ptr` are gated).

### 4. C command `kalico_runtime_reset` — `src/stepper.c`

Thin foreground `DECL_COMMAND`, no args:

```c
void command_kalico_runtime_reset(uint32_t *args) {
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

- **IRQ guard in C** (`irq_save`/`irq_restore`, `board/irq.h`, already included in
  `stepper.c`). This is the safety-critical window: on STM32 TIM5 is always armed, so
  on a reconnect the TIM5 sample ISR *and* the four per-axis step-event timers are live
  and concurrently touch the exact state being cleared. `irq_save` blocks **all**
  maskable interrupts — both sources — for the bounded O(MAX_AXES) duration of the
  reset (tens of µs). Keeping IRQ control in C matches the MCU C/Rust boundary
  invariant (C owns safety-critical paths); the §8.5 `flush` path is the existing
  precedent for an ISR-touching foreground op under a disabled-IRQ window.
- Does **not** touch the `per_axis_timers_installed` static gate — the per-axis timers
  stay installed for the boot; after reset they find `num_axes == 0` and no-op until
  the following `configure_axis` calls repopulate the engine.

### 5. Host call site — `klippy/motion_toolhead.py` `_configure_axes_per_mcu`

Once per MCU, after the `configure_axis_cmd` lookup succeeds (~line 1166, so we know
the MCU speaks the new protocol) and **before** the per-axis configure loop:

```python
try:
    reset_cmd = mcu_obj.lookup_command("kalico_runtime_reset")
except Exception:
    reset_cmd = None  # older firmware without the reset command
if reset_cmd is not None:
    reset_cmd.send([])
```

- Same command queue to the same MCU ⇒ the reset is processed before the
  `configure_axis` commands that follow. No ordering handshake needed.
- Idempotent: on a fresh-booted MCU the reset clears already-clean state (no-op effect:
  cursor already 0). Covers cold start, in-console restart, and crash-reconnect
  uniformly with one unconditional send.
- Tolerant of older firmware via the lookup-or-skip pattern (mirrors the existing
  `configure_axis_cmd` lookup-or-`continue`).

### Consistency invariant (no extra host change)

`_init_planner` runs `self.bridge.init_planner(...)` immediately before
`_configure_axes_per_mcu` on the same connect, rebuilding the bridge pump and
`ring_depth_table` fresh — so host-side `pushed` / `consumed` accounting starts at 0,
matching the MCU's `consumed = 0` after reset. The reset keeps the two sides
consistent; no additional host-side accounting change is required.

## Build / wiring

- Add the `#[unsafe(no_mangle)]` FFI, then regenerate the cbindgen header:
  `cargo run -p kalico-c-api --bin gen-headers` (header is `DO NOT EDIT`).
- `make clean` between H7 and F446 C builds (msgid descriptors / oid types differ).
- Both MCUs run the fork's firmware; the new command must be present on both.

## Testing

- **Regression unit test (host)** — `rust/runtime/tests/` (extend `configure_axis.rs`
  or add `runtime_reset.rs`): configure all axes near full, push pieces, call
  `engine.reset()`, assert `ring_alloc_cursor == 0`, all `stepping_axes` `None`,
  `num_axes == 0`, `status == Idle`, `last_error == 0`, `step_state` default. Then
  re-configure all axes and assert allocation **succeeds** — i.e. the
  configure → reset → configure cycle does not overflow. This is the direct reproduction
  of the bug at the engine level.
- **Idempotency test** — `reset()` on a freshly-constructed engine leaves the cursor at
  0 and all state clean (no-op).
- **Preservation test** — after `reset()`, `sample_period_cycles`, `cycles_per_second`
  are unchanged.
- Existing `rust/runtime/tests/configure_axis.rs`, `piece_tick.rs` continue to pass.
- Step-queue clear is MCU-only (`#[cfg]`-gated); covered by build + bench, not host
  unit tests.

## Out of scope

- No change to the bridge config-CRC model (we are not adopting mainline's
  get_config / finalize handshake).
- No change to firmware reboot behavior (bridge MCUs still reboot only on
  `FIRMWARE_RESTART`).
- No change to the fault / shutdown subsystem.

## File-by-file change list

| File | Change |
|---|---|
| `rust/runtime/src/engine.rs` | Add `Engine::reset(&mut self)`. |
| `rust/runtime/src/step_queue.rs` | Add MCU-gated `reset_all_queues()`. |
| `rust/kalico-c-api/src/runtime_ffi.rs` | Add `kalico_runtime_reset` FFI. |
| `rust/kalico-c-api/include/kalico_runtime.h` | Regenerate (cbindgen). |
| `src/stepper.c` | Add `command_kalico_runtime_reset` + `DECL_COMMAND`. |
| `klippy/motion_toolhead.py` | Send `kalico_runtime_reset` per MCU before the configure loop. |
| `rust/runtime/tests/` | Regression + idempotency + preservation tests. |
