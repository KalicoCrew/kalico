# Bridge-mode end-to-end completion — discovery + path-forward RFC

**Date:** 2026-05-04
**Status:** Discovery; needs decision before implementation
**Trigger:** klippy-in-loop sim (Step 7-D Phase A+B) caught a latent bug.

## What the sim found

Renaming `motion_bridge.so` → `motion_bridge_native.so` (to stop it shadowing `motion_bridge.py`) actually turned bridge mode on for the first time. Things broke immediately, in three layers:

1. **Connect:** `klippy/serialhdl.py:_start_session` unconditionally uses `self.ffi_main` / `serialqueue_alloc`, but the bridge-mode `__init__` sets both to `None`. Crash on first MCU connect.

2. **Command path has no consumer:** `motion_bridge.endstop_arm` (Rust) builds a `RouterTransport` and writes to `self.router` (the bridge's internal `PassthroughRouter`). Nothing on the klippy side pumps that router onto the wire, and nothing forwards inbound msgproto bytes from klippy's serialqueue back into the router. The bridge is air-gapped from the MCU.

3. **MotionMcuProxy is a Phase-1 stub:** `klippy/motion_mcu.py:MotionMcuProxy` was meant to be the bridge-mode replacement for `klippy.MCU`. Its `MotionCommandWrapper.send` just `logging.debug`s; `MotionQueryCommandWrapper.send` returns synthetic `{}`. Real commands wouldn't reach the MCU even if it were wired up. It is also never instantiated — `printer.py` always constructs the legacy `MCU`.

The "Step 7-C-bridge Phase 2 gate passed 2026-05-02" only validated the wire-level contract under a synthetic test harness — never against a live klippy.

## Practical consequence

Every `submit_move` / `submit_homing_move` call recorded in the production printer's klippy.log has been a no-op. Production has been running with bridge-mode silently disabled (legacy MCU class only, with `_motion_bridge=None`), but `motion_toolhead.move` calls `bridge.submit_move` instead of producing real step pulses. The user's recent homing attempts didn't move steppers because bridge-mode = on (post-fix) → submit_move enters the planner → planner dispatch closure has no consumer → bytes go nowhere.

## Two paths

### Path A — complete the original design (PassthroughRouter as central pipe)

**Concept:** Bridge owns the wire-side state. `MotionMcuProxy` becomes the real MCU implementation in bridge mode. klippy's legacy `MCU` / `serialqueue` is replaced for bridge MCUs. Non-motion commands (TMC SPI, heaters, fans) flow through the bridge's passthrough layer.

**Work:**
- Wire klippy↔bridge bidirectionally: `MotionMcuProxy.send` enqueues to the router; a klippy-side pump drains `router.pop_next_for_emission` to the wire (or the bridge spawns its own serial reader thread that owns the FD).
- Real `MotionCommandWrapper.send` / `MotionQueryCommandWrapper.send` (msgproto encoding via shared parser, response correlation via `NotifyId`).
- Identify/restart/shutdown plumbing (currently only the synthetic `RouterTransport.call_typed` exists).
- `motion_mcu.py` parity with `mcu.MCU` for ~50 methods used across klippy.
- Replace mcu.py construction site in printer.py to dispatch on bridge presence.

**Estimated scope:** 1–2 weeks. Several new spec sections in §3 / §5 of the existing 7-D spec.

**Risk:** TMC UART, SPI, GPIO output_pin, ADC sensor — every klippy peripheral subsystem that calls into MCU.lookup_command must be re-validated against `MotionMcuProxy`. Lots of edge cases.

### Path B — bridge as a thin command encoder over legacy serialqueue

**Concept:** klippy's legacy `MCU` keeps owning the wire. The bridge is a Rust-side compute layer (planner, endstop runtime state, curve compilation) that produces *commands* but does not transport them. Each `kalico_*` command is sent via klippy's existing serialqueue.

**Work:**
- `motion_bridge.py:MotionBridgeWrapper.endstop_arm` becomes pure Python: `mcu.lookup_command("kalico_arm_endstop ...").send([...])`. Drop `RouterTransport` from this path.
- Same for `endstop_disarm`, `set_homed_state`.
- Async `kalico_endstop_tripped` handler registers on the legacy serial reader.
- `submit_move` / `submit_homing_move`: planner produces segment descriptors; the wrapper sends them as `kalico_runtime_push_segment` commands via legacy serialqueue.
- `RouterTransport` and the bridge's internal router stay only for *internal* Rust→Rust calls (planner→producer protocol). They never see bytes destined for the MCU.
- `MotionMcuProxy` is deleted — bridge mode no longer needs an MCU replacement.
- `serialhdl._use_bridge` paths can be deleted; legacy serialqueue is universal.

**Estimated scope:** 1–3 days. No new architecture, just removing the air-gap.

**Risk:** None I can see. Path B is what the production printer has been *de facto* doing for the legacy commands — we just extend it to the kalico_* family. No klippy peripheral subsystem changes.

## Recommendation

**Path B.** Reasons:

- Smaller and faster — unblocks Step 8 hardware bring-up in days, not weeks.
- Less surface area for new bugs — leverages serialqueue.c which has been hardened over years.
- The "bridge owns the wire" justification (PassthroughRouter as central pipe) was about future EtherCAT support and async event dispatch. EtherCAT doesn't run on the same wire as the legacy serialqueue anyway, so it'll need its own transport regardless. Async events (kalico_endstop_tripped, kalico_credit_freed, kalico_fault) are async msgproto outputs — register_response on legacy serialqueue handles them fine.
- Path A's MotionMcuProxy parity work is scope creep that doesn't deliver functional value beyond Path B.

If we later find a *specific* reason to re-own the wire (e.g. multi-MCU coordination, EtherCAT, real-time event dispatch klippy can't keep up with), we can rebuild that boundary then with concrete requirements instead of speculative design.

## Open questions for the next session

1. Does the user agree with Path B?
2. Is there a Step 7-C-bridge spec point that explicitly requires PassthroughRouter ownership? If yes, we need to retire it.
3. Are there any in-flight features (Step 9 PA, Step 10 phase-stepping) that assume the Path A architecture?
