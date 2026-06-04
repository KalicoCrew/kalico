# MCU Log Endpoint — Stage 5 v1.1 (crash play-by-play) Plan

> Routes the prior-boot diag **event ring** (the crash lead-up timeline) and the
> **cause-naming discriminators** (block sources, ISR phase, TIM5 inter-arrival,
> last-dispatched callback) through the structured-log path, and fixes the crash
> gate so they actually fire on the real (klippy-reset-masked) crash signal.
> Builds on Stage 5 (summary frames) — spec §11 "deferred v1.1". Rust via
> `rust-engineer`.

**Goal:** a hard reset surfaces a queryable *play-by-play* (the diag ring) plus
the discriminators that name the cause (e.g. "USB OTG ISR hogged 265µs → fg
starved"), not just "it reset + roughly where it hung."

**Why:** the prior-boot diag already contains the root cause (verified bench
crash: `usb_burst 138126` cyc, healthy TIM5, `isr_phase ISR_EXIT`, ring full of
USB gaps), but it lives only in `klippy.log` `output()` text. The structured log
got only the summary; and its gate keyed on the immediate reset cause, which
klippy's connect-reset overwrites with SFTRST — so the crash frames mis-leveled
or never fired.

**Verification (deterministic):** `±10 mm F6000` jog hard-resets the MCU. On
reconnect, confirm in `events/*.jsonl`: `runtime.block_source`
(usb_burst > 0), `runtime.isr_phase`, `runtime.tim5_ia`, `runtime.last_dispatch`,
and a sequence of `diag.*` ring entries (oldest-first).

---

## Root-cause findings (bench log, verified crash)

- prior-boot is the **genuine crash boot** (`tim5_n 160487`, engine active) →
  the snapshot survived the single reset. **No persistent capsule needed.**
- Engine innocent: `tim5_max_cyc 693`, `tim5ia min 50298/max 52277` (period
  52000), `isr_phase 9` (ISR_EXIT).
- Cause: `usb_burst_max_cyc 138126` (~265µs OTG ISR burst, >2× TIM5 period) →
  foreground starvation → klippy lost comms → **software reset** (`rcc
  0x01420000` = SFTRST|PINRST|CPURST, not IWDG). `iwdg_reset_count 1` is a stale
  cumulative count, not this crash.
- Two gaps fixed here: (a) gate keyed on immediate (masked) reset cause; (b) the
  ring timeline + cause discriminators never structured.

---

## Design

### Wire mapping

The `McuLog` frame carries `subsystem:u8, event:u16, code:u16, arg0:u32,
arg1:u32` (the ring entry's own `seq`/`timestamp` are dropped: emit order gives
chronology, and the salient timing rides in `a`/`b`). `code = 0` for all new
frames (avoids a spurious `code_name` from the host's `FaultCode::from_u16`).

- **Ring entries → new `diag` subsystem; event-code == `DIAG_EV_*` tag (1:1).**
  `arg0 = a`, `arg1 = b`. Per-tag level chosen on the C side.
- **Discriminators → `runtime` subsystem, new events 10–13.**

### C gating — per-run freeze flag

`live_snapshot` gains `this_run_froze` (BKPSRAM-persistent). Set in the TIM5-ISR
freeze watchdog (`diag_tim5_account`) when `fg_stall_ticks >=
FG_FREEZE_REPORT_THRESHOLD` (8 ≈ 0.8 ms; observed worst was 10). Captured at
boot-init as file-static `prior_run_froze`, then `this_run_froze` reset for the
new run. Survives klippy's connect-reset (one reset; same survival window as the
ring). `abnormal = had_fault || prior_run_froze || immediate-IWDG`.

**Known limitation (documented, not fixed here):** a PRIMASK-hard freeze stops
the TIM5 ISR so `this_run_froze` can't be set; and a *second* benign reset before
report would overwrite both the ring and the flag. Those need the deferred
persistent capsule. The common ISR-starvation freeze (observed) is covered.

---

### Task 1: host event codes (`rust-engineer`)

`rust/runtime/src/log_codes.rs`:

- Add `pub const SUBSYSTEM_DIAG: u8 = 4;` and a `subsystem_name` arm → `"diag"`.
- Add 8 diag ring events (codes == tags) + `event_info` arms:
  - `EVENT_DIAG_TIM5_LONG = 1` → `("diag.tim5_long", "TIM5 ISR long {arg0} cyc at t={arg1}")`
  - `EVENT_DIAG_OTG_LONG = 2` → `("diag.otg_long", "OTG ISR long {arg0} cyc at t={arg1}")`
  - `EVENT_DIAG_USB_OUT_GAP = 3` → `("diag.usb_out_gap", "USB-OUT gap {arg0} ticks, prev t={arg1}")`
  - `EVENT_DIAG_USB_IN_GAP = 4` → `("diag.usb_in_gap", "USB-IN gap {arg0} ticks, prev t={arg1}")`
  - `EVENT_DIAG_TX_DROP_KAL = 5` → `("diag.tx_drop_kalico", "kalico TX drop len={arg0} tpos={arg1}")`
  - `EVENT_DIAG_TX_DROP_KLP = 6` → `("diag.tx_drop_klipper", "klipper TX drop max={arg0} tpos={arg1}")`
  - `EVENT_DIAG_ENGINE_XITION = 7` → `("diag.engine_xition", "engine state packed={arg0} samples={arg1}")`
  - `EVENT_DIAG_RUST_FAULT = 8` → `("diag.rust_fault", "rust fault err={arg0} detail={arg1}")`
- Add 4 runtime discriminator events + arms:
  - `EVENT_RUNTIME_LAST_DISPATCH = 10` → `("runtime.last_dispatch", "last dispatch func={arg0} addr={arg1}")`
  - `EVENT_RUNTIME_ISR_PHASE = 11` → `("runtime.isr_phase", "isr phase={arg0} ring_overflow={arg1}")`
  - `EVENT_RUNTIME_BLOCK_SOURCE = 12` → `("runtime.block_source", "block usb_burst={arg0} cyc stepout_burst={arg1} cyc")`
  - `EVENT_RUNTIME_TIM5_IA = 13` → `("runtime.tim5_ia", "tim5 inter-arrival min={arg0} max={arg1} cyc")`
- Tests: `subsystem_name(SUBSYSTEM_DIAG) == "diag"`; each new pair resolves to
  the expected name and references the right `{argN}`; a `diag` unknown-event
  still returns `("unknown","")`.

Verify: `cargo test -p runtime log_codes`, `cargo build -p runtime -p motion-bridge`.

### Task 2: C mirrors + struct field (main agent)

- `src/kalico_log.h`: add `KALICO_LOG_SUBSYS_DIAG 4`,
  `KALICO_LOG_EVENT_RUNTIME_LAST_DISPATCH 10`, `_ISR_PHASE 11`,
  `_BLOCK_SOURCE 12`, `_TIM5_IA 13`.
- `src/generic/fault_handler.c`: append `uint32_t this_run_froze;` to
  `struct live_snapshot`; set it in the freeze watchdog; capture
  `prior_run_froze` + reset at boot-init (alongside the existing prior_* capture).

### Task 3: C emit (main agent)

- `src/generic/fault_handler.c::kalico_diag_emit_prior_crash`: redefine
  `abnormal` to include `prior_run_froze`; when `abnormal`, after the existing
  summary frames, emit `last_dispatch`, `isr_phase`+`ring_overflow`,
  `block_source`, `tim5_ia`, then walk `prior_ring` oldest-first
  (`head = prior_diag.ring_head & DIAG_RING_MASK`), skip `DIAG_EV_NONE`, and emit
  each as `kalico_log_emit(level_for_tag(tag), KALICO_LOG_SUBSYS_DIAG, tag, 0,
  a, b)`. Frame budget ≤ ~41 < 64-entry log ring.

### Task 4: build + bench crash-verify

- Host `cargo` green; commit + push.
- Flash both MCUs + host `.so` (`trident` alias).
- `±10 mm F6000` crash → reconnect → confirm `runtime.block_source`,
  `runtime.isr_phase`, `runtime.tim5_ia`, `runtime.last_dispatch`, and `diag.*`
  ring entries in `events/*.jsonl`. Bench recovers clean.

## Verification of success

A deliberate crash surfaces a structured timeline + cause discriminators: the
operator can read "USB OTG ISR hogged N µs → foreground starved" from the log
store, with the ring lead-up, not just "it reset."
