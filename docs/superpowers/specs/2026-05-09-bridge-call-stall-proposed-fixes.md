# Bridge-call Stall — Proposed Fixes

**Date:** 2026-05-09
**Companion to:** [`2026-05-09-bridge-call-stall-investigation.md`](./2026-05-09-bridge-call-stall-investigation.md)
**Status:** PROPOSAL — not yet applied. Needs hardware diagnostic confirmation before Fix 2.

---

## Fix 1 — Silent-swallow on direct-dispatch Io errors (defect, low risk)

**Problem:** `handle_command::Submit`, `FireAndForget`, `FireAndForgetTyped`, and `KalicoCall` all call `dispatch_submission` / `dispatch_fire_and_forget` / `write_frame` and on `TransportError::Io` send the error to the caller's completion but do NOT set `state = Closed` or stage `HostDisconnect`. The drain paths (`drain_pending_submissions`, `drain_passthrough`) DO handle this correctly. Asymmetric handling.

**Test coverage:** `rust/kalico-host-rt/src/host_io/reactor.rs::silent_swallow_findings::*` (5 tests, currently passing — they pin the BUG, not the fix). When this fix lands, those assertions need to flip.

**Patch sketch** (do NOT apply blindly — this needs to be reviewed against test expectations):

```rust
// reactor.rs, in handle_command():
ReactorCommand::Submit { call_id, cmd, expected_response_name, completion, deadline } => {
    let expected_oid = extract_oid_from_cmd(&cmd);
    match self.parser.encode(&cmd) {
        Ok(payload) => {
            if let Err(e) = self.dispatch_submission(
                call_id, payload, expected_response_name, completion.clone(), deadline,
                expected_oid,
            ) {
                let is_io = matches!(e, TransportError::Io(_));
                let _ = completion.send(Err(e));
                if is_io {
                    self.transition_closed_on_io_fault();
                }
            }
        }
        Err(e) => { let _ = completion.send(Err(TransportError::Parse(format!("{e:?}")))); }
    }
}

// Identical pattern for SubmitTyped, FireAndForget (use is_io check on Io too),
// FireAndForgetTyped, KalicoCall.

// New helper:
impl Reactor {
    fn transition_closed_on_io_fault(&mut self) {
        if self.pending_host_fault.is_none() {
            self.pending_host_fault = Some(crate::host_io::runtime_events::FaultEvent {
                fault_code:   FaultCode::HostDisconnect.as_u16(),
                fault_detail: 0,
                segment_id:   0,
                synthesized:  false,
            });
        }
        self.state = ReactorState::Closed;
    }
}
```

**Test flips when fix lands:**

```rust
// Before fix (current), in silent_swallow_findings:
assert_eq!(reactor.state, ReactorState::Active, "DEFECT: ...");

// After fix:
assert_eq!(reactor.state, ReactorState::Closed,
    "After Io fault, immediate-dispatch paths now correctly transition to Closed");
assert!(reactor.pending_host_fault.is_some(),
    "HostDisconnect fault is staged");
```

**Risk:** low. The drain path already does this; we're making the immediate path consistent. Existing tests for write-success paths are unchanged. The 5 characterization tests in `silent_swallow_findings` flip from "documents bug" to "documents fix".

**Why not block on the root cause:** this is a real defect regardless of whether it's the user's current symptom. It causes klippy to see "transport I/O error" repeatedly instead of a clean disconnect when the wire breaks. Independent value.

---

## Fix 2 — H7 firmware reliable TX queue (root-cause hypothesis, higher risk)

**Problem (suspected, requires hardware diagnostic to confirm):** under heavy host-side traffic during `_do_enable`, the H7 firmware's `transmit_buf[320]` overruns. `console_sendf` (line 88-90 in `src/generic/usb_cdc.c`) and `kalico_console_write_raw` (line 111-112) silently drop frames when full. The `kalico_console_write_raw` call from `kalico_transport_send_frame` (line 138 in `src/kalico_dispatch.c`) IGNORES the -1 return value.

**Diagnostic to confirm before fixing** (1 minute of live tracing):

```c
// src/kalico_dispatch.c:121-138, replace kalico_transport_send_frame body's tail with:
int written = kalico_console_write_raw(tx_buf, (uint16_t)total);
if (written < 0) {
    output("[trace-tx] kalico_drop chan=%c len=%u",
           channel, (unsigned)total);
}
```

```c
// src/generic/usb_cdc.c::console_sendf, before the early return on full buffer:
if (tpos + max_size > sizeof(transmit_buf)) {
    output("[trace-tx] klipper_drop msg_max=%u tpos=%u",
           (unsigned)max_size, (unsigned)tpos);
    return;
}
```

If the user's repro shows `[trace-tx] klipper_drop` events during `G28 X`, the buffer-fill hypothesis is confirmed.

**Fix Option A — cheap, wrong:** grow `transmit_buf` from 320 to 1024 or larger. Buys time but doesn't fix the underlying loss.

**Fix Option B — correct:** make TX path bounded-blocking. Replace the silent-drop with a "wait for space" semantic via the firmware's task scheduler:

```c
// New: TX-full waitstate
static struct task_wake usb_tx_drain_wake;

void
usb_notify_tx_drain(void) {
    sched_wake_task(&usb_tx_drain_wake);
}

// Modified usb_bulk_in_task (after successful send, kick the drain wake):
void usb_bulk_in_task(void) {
    /* ... existing logic ... */
    int_fast8_t ret = usb_send_bulk_in(transmit_buf, max_tpos);
    if (ret <= 0) return;
    /* ... existing memmove + transmit_pos update ... */
    if (transmit_pos < sizeof(transmit_buf) / 2) {
        sched_wake_task(&usb_tx_drain_wake);
    }
}
```

Producer side (kalico_dispatch + console_sendf): instead of silent drop, return -1 and let the caller retry from a periodic task. This requires reworking `kalico_native_emit_status_event` to NOT call into the TX path directly — instead, set a "status-pending" flag that the next periodic emit-task checks, and on TX-success the flag clears.

**Fix Option C — architectural:** match the project's CLAUDE.md non-negotiable: "Real time communication with MCUs, no queue-based offload." Treat the H7 USB-CDC TX as a real-time channel: the firmware MUST be able to flush before producing more, OR producers MUST block until they can. The current `transmit_buf[320]` is a queue with silent drop, which violates this guarantee.

This is a Step 7-D scope decision. Recommend: implement Option B as a minimum, document Option C as a future replacement when the kalico-native transport reaches production maturity.

**Risk of Option B:** medium. Touches firmware TX semantics. Needs:
- New task plumbing for TX-drain wake
- Reworking ALL callers of `console_sendf` and `kalico_console_write_raw` to handle -1
- Updating runtime emit paths (`runtime_tick.c::periodic_status_event`) to retry on backpressure

**Risk of Option C:** high. Larger refactor, would benefit from being part of Step 7-D's broader transport hardening.

---

## Fix 3 — Loud failure on TX drop (defense in depth, low risk)

Add `output(...)` traces on every silent-drop path. `output()` queues a printf-formatted message via the same TX path, which is itself the failing path — but the queue has space NOW (the alternative is drop). This makes the silent failure mode debuggable from klippy.log alone.

**Patch:** essentially the diagnostic from Fix 2, but always-on, not just for diagnosis.

**Risk:** low. The output() call adds a few bytes to a buffer that already fits status events. If the buffer is full the output() call also gets dropped — but at least we tried.

---

## Fix 4 — Sim fidelity gap documentation

The bridge_call stall is currently UNTESTABLE in `sim_klippy` due to PTY-vs-USB-FS timing differences. The "position_max=20" workaround in `tools/sim_klippy/pin-overrides.toml` documents this implicitly; should be made explicit.

**Patch:**

```diff
--- a/docs/kalico-rewrite/dependency-graph.md
+++ b/docs/kalico-rewrite/dependency-graph.md
@@ ... @@
+## Sim coverage gaps
+
+The `sim_klippy` faithful simulator uses PTYs (pseudo-terminals) for the
+host↔MCU connection. PTYs have different blocking semantics than real
+USB FS bulk endpoints:
+
+- PTYs have a single ~4-8 KB kernel buffer with simple write-blocks-when-full
+- USB FS bulk has 64-byte EP FIFOs with NAK-on-full and 1ms USB frame timing
+
+**Bugs that depend on USB FS bulk timing (TX URB stall, EP FIFO NAK, host
+kernel URB queue depth, BrokenPipe on stalled device) are NOT reproducible
+in sim_klippy.** Verified 2026-05-09 against the bridge_call stall — see
+`docs/superpowers/specs/2026-05-09-bridge-call-stall-investigation.md`.
+
+To exercise these paths: real hardware required, or a Renode model with a
+USB-CDC peripheral that simulates EP FIFO + NAK behavior (does not
+currently exist for STM32H723 in upstream Renode).
```

**Risk:** zero. Documentation only.

---

## Fix 5 — SPI peripheral watchdog (alternate root-cause candidate, low risk)

**Problem (suspected, alternate hypothesis):** STM32H7's `spi_transfer` (and F4's) has infinite waits on `SPI_SR_RXNE` and `SPI_SR_EOT`. If the SPI hardware deadlocks (CS polarity glitch, FIFO state from a previous transfer, hardware errata), the cooperative scheduler wedges and IWDG fires after 30 s. Matches the user's BKPSRAM data (engine_status=0, samples_taken stable, IWDG1RSTF) without requiring buffer-fill semantics.

**Diagnostic patch:**

```c
// src/stm32/stm32h7_spi.c::spi_transfer
void
spi_transfer(struct spi_config config, uint8_t receive_data,
             uint8_t len, uint8_t *data)
{
    uint8_t orig_len = len;
    SPI_TypeDef *spi = config.spi;
    spi->CR2 = len << SPI_CR2_TSIZE_Pos;
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_SPE;
    spi->CR1 = SPI_CR1_SSI | SPI_CR1_CSTART | SPI_CR1_SPE;

    while (len--) {
        writeb((void *)&spi->TXDR, *data);
        // 100us per byte timeout (15us at 4MHz × 8 bits = 30us; 100us is 3×).
        uint32_t spi_deadline = timer_read_time() + timer_from_us(100);
        while ((spi->SR & (SPI_SR_RXWNE | SPI_SR_RXPLVL)) == 0) {
            if (!timer_is_before(timer_read_time(), spi_deadline)) {
                shutdown("spi rx timeout");
            }
        }
        uint8_t rdata = readb((void *)&spi->RXDR);
        if (receive_data) *data = rdata;
        data++;
    }

    uint32_t eot_deadline = timer_read_time() + timer_from_us(100);
    while ((spi->SR & SPI_SR_EOT) == 0) {
        if (!timer_is_before(timer_read_time(), eot_deadline)) {
            shutdown("spi eot timeout");
        }
    }

    spi->IFCR = 0xFFFFFFFF;
    spi->CR1 = SPI_CR1_SSI;
}
```

The same change applies to `src/stm32/spi.c` for F4.

**Behavior on next repro:** if SPI hangs, klippy sees `MCU shutdown: spi rx timeout` instead of `bridge_call: transport timed out`. Diagnostic value:
- The shutdown reason cleanly tells us this is the SPI hardware path
- klippy.log captures the shutdown reason via the standard MCU shutdown handler
- No more silent wedge → IWDG → recovery cycle

**Risk:** low. Adding a timeout to a busy-wait loop. The 100 µs per byte is generous (4 MHz SPI = 2 µs per byte). Only triggers when hardware truly hangs; non-hanging path is unchanged.

**Side benefit:** turns a class of cooperative-scheduler wedge bugs into clean shutdowns, regardless of whether the user's specific bug is in this path. Defense in depth.

---

## Order of operations

1. **First (now, no hardware):** Land Fix 1 + flip the `silent_swallow_findings` tests. ~30 lines of code, 5 tests flip, ~1 hour reviewer time.

2. **First (now, no hardware):** Land Fix 5 (SPI watchdog) — pure defensive, ~20 lines per platform. Self-contained, no behavior change in the success path.

3. **Second (when hardware is available):** Apply the diagnostic from the investigation doc's "Recommended next diagnostic" section AND the Fix 5 watchdog. Run G28 X. Look at `klippy.log` for either:
   - `MCU shutdown: spi rx timeout` → SPI hang hypothesis confirmed → no further action needed beyond Fix 5 (which already converts hang into shutdown)
   - `[trace-tx] *_drop` events → buffer-fill hypothesis confirmed → land Fix 2 Option B
   - Neither → instrument deeper (poll_serial timing, demuxer state)

4. **Third (if buffer-fill confirmed):** Implement Fix 2 Option B. Multi-day scope. Start with the periodic status emit; that's the highest-frequency producer and the cheapest one to convert.

5. **Concurrently (any time):** Apply Fix 3 and Fix 4 as documentation/defense work. Self-contained, no dependencies on hardware.

---

## What does NOT need to change

- The reactor's read path (`poll_serial`). Subagent + my own review found no logical bug in frame demuxing or response routing.
- The `find_match` FIFO-by-name semantics. Cross-talk hypothesis FALSIFIED by user's own instrumentation.
- The kalico-native protocol structure. Independent of the wire stall.
- The producer's load_curve / push_segment timing. The 2 s timeout is fine; it would catch a true loss but not the wire-fill scenario fast enough.
- Klippy's tmc.py logic. The retry-5x-on-set_register pattern is mainline behavior; we shouldn't deviate.
