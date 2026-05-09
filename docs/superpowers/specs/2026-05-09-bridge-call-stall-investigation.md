# Bridge-call Stall Investigation — Research Artifact

**Date:** 2026-05-09
**Branch:** `sota-motion`
**Test artifact:** `rust/kalico-host-rt/src/host_io/reactor.rs::io_fault_propagation` (6 tests, all passing)
**Repro test (negative):** `tools/sim_klippy/tests/test_bridge_stall_repro.py` (3 sim scenarios; sim does NOT reproduce)
**Status:** Fix 1 (silent-swallow on Submit/FireAndForget/KalicoCall Io error) and Fix 6 (RTO retransmit Io escalation) landed. Fix 5 (SPI peripheral watchdog on H7+F4) landed. The user's primary bug remains open — observed two distinct failure modes today, fixes catch one (Path A: Io error from write_frame); the other (Path B: Submit not reaching reactor for 5s without Io error) is unexplained.

## Problem statement

When `G28 X` (or any move that triggers stepper enable) is issued on real hardware (BTT Octopus Pro H723) running our Kalico-fork with the Rust `kalico-host-rt` reactor and motion-bridge, klippy crashes inside `tmc.py._do_enable` on the **first** stepper-enable SPI register write to TMC5160:

- Most runs: `RuntimeError: bridge_call: transport timed out`
- Some runs: `RuntimeError: bridge_call: transport closed` (BrokenPipe / disconnected channel)

Latest run: two `producer::load_curve` kalico-native calls succeeded **before** the crash. Earlier runs had zero. So progress through the kalico-native protocol is happening, then SPI dies.

H7 firmware diagnostic from BKPSRAM (preserved across IWDG reset):
- Main loop healthy (`samples_taken=166781` ≈ 5.5 s, liveness flag set)
- Runtime engine never armed (`engine_status=0, tick_counter=0, last_run_tick=0`)
- `IWDG1RSTF` in RCC->RSR — IWDG fired **downstream** after klippy stopped draining USB

**Cross-talk hypothesis** (find_match handing a Submit's response to the wrong waiter): **FALSIFIED** by instrumented `find_match`. Zero `[xtalk]` events; `await_n=1` consistently.

---

## TL;DR — what this 5-hour investigation produced

1. **Verified Finding 1 (silent-swallow on direct-dispatch Io errors) is a REAL DEFECT** — locked down with 5 unit tests in `reactor.rs::silent_swallow_findings`. The reactor stays Active even after 20 consecutive write failures on `Submit`, `FireAndForget`, and `KalicoCall` paths. Compare with `drain_pending_submissions`/`drain_passthrough` which correctly transition to Closed.

2. **Confirmed Finding 1 doesn't match the user's exact symptom.** The user sees `transport closed` / `transport timed out`; the silent-swallow path produces `transport I/O error: BrokenPipe`. The real symptom comes from a **read-side** failure (response never decodes) followed by either caller-side `recv_timeout` firing (Timeout) or reactor-side disconnect detection running `flush_all_completions` (Closed). Finding 1 is a separate latent defect that should still be fixed.

3. **Found a long-standing project lead** in `tools/sim_klippy/pin-overrides.toml` (lines 64-77): a previous session documented "a separate sim-infra bug surfaces as 'RuntimeError: bridge_call: transport closed' mid-homing" and worked around it by shrinking `position_max=20` so homing finishes inside one tmc-poll window. That bug was NEVER root-caused — it was avoided. **It is the same bug we're now seeing on real hardware.**

4. **Identified the structural shape of the bug**: the H7 firmware silently drops TX frames when its 320-byte `transmit_buf` fills — both `kalico_console_write_raw` and `console_sendf` return on full-buffer without queueing for retry. Combined with concurrent host-side traffic from `tmc.py._do_enable` (Klipper-protocol bridge_calls) and motion-bridge's planner thread (kalico-native `kalico_call`s for load_curve / push_segment), there is a window where the H7's USB-CDC TX pipe stalls, dropped responses go unnoticed, and the host falls into recv_timeout / disconnect-detection.

5. **Could not reproduce in sim** — three deliberate stress tests (back-to-back linear moves, long Z move spanning multiple tmc-poll cycles, single move post-SET_KINEMATIC_POSITION) all PASSED in the docker sim. The difference between sim and real hardware is the **USB-CDC physical layer** (PTY for sim, real USB FS bulk endpoints for hardware). The race needs the device-side buffer to fill, and PTYs don't backpressure the same way.

6. **Subagent's parallel investigation** (read reactor.rs in full + bridge.rs cross-path):
   - No mutexes shared between kalico-native and Klipper-classic decode paths
   - No path bias in command channel drain (FIFO mpsc, MAX_SUBMITS_PER_ITER=4)
   - Frame parser correctly demultiplexes by first-byte (0x55 = kalico, 5..=64 = klipper-len)
   - Sequence-number collision impossible (kalico uses correlation_id, klipper uses send_seq nibble)
   - `kalico_state.identified` gating cannot affect Submit path
   - **No logical shared-state bug between paths** — confirmed independently.

---

## Investigation timeline & evidence

### 1. Read reactor.rs in full (own pass)

Found the silent-swallow gap subagent identified, but verified line numbers against current source:

| Path | Line | Behavior on Io error |
|------|------|---------------------|
| `dispatch_submission` (immediate, called from Submit) | 264 | `?` propagates error; `state` stays Active |
| `drain_pending_submissions` (called from tick step 3) | 334-346 | Sets `state = Closed`, stages `HostDisconnect` |
| `drain_passthrough` (called from tick step 3b) | 413-425 | Sets `state = Closed`, stages `HostDisconnect` |
| `poll_serial` Err (read fault) | 752-761 | Sets `state = Closed`, stages `HostDisconnect` |
| `poll_serial` PhantomZero (>100 ms zero reads) | 738-751 | Sets `state = Closed`, stages `HostDisconnect` |
| `write_retransmit` on MAX_RETRY | 547-555 | Sets `state = Closed`, stages `HostRetransmitExhausted` |
| `handle_command::Submit` (when window has room) | 778-783 | Just `completion.send(Err(e))` — NO state change |
| `handle_command::FireAndForget` | 837-849 | Just `eprintln!` — NO state change |
| `handle_command::FireAndForgetTyped` | 851-855 | Just `log::warn!` — NO state change |
| `handle_command::KalicoCall` | 873-891 | Removes pending entry, `completion.send(Err(e))` — NO state change |

**The asymmetry is real:** drain paths handle Io correctly; immediate paths don't.

### 2. Locked it down with 5 new tests in `reactor.rs::silent_swallow_findings`

Files: `rust/kalico-host-rt/src/host_io/reactor.rs:2401+`

All 5 PASS in docker (`cargo test -p kalico-host-rt --lib silent_swallow_findings`):

- `submit_typed_io_error_silently_swallowed` — characterizes the Submit gap
- `fire_and_forget_typed_io_error_silently_swallowed` — characterizes the FireAndForget gap
- `kalico_call_io_error_silently_swallowed` — characterizes the KalicoCall gap
- `many_submits_against_broken_wire_each_get_io_no_global_disconnect` — proves a STREAM of failures doesn't trip global disconnect
- `drain_path_does_transition_closed_on_io_error` — sanity-check that the OTHER path works correctly

These are **characterization tests** — they pin OBSERVED behavior. When a fix lands that makes the immediate path also transition Closed, these tests must be flipped to assert the new behavior. The test bodies cite the specific reactor.rs line numbers responsible.

Side observations from the tests:

- After the silent-swallow, `send_seq` advanced by 1 even though no frame went out. There is a "burned" sequence number with no entry in unacked_window. On the next successful send, the wire seq nibble will skip a value — MCU will NAK. (Documented in the test assertion.)
- `pending_host_fault.is_none()` post-failure — no fault reaches the FaultLatch, so klippy's fault subscriber sees nothing.

### 3. Why this defect doesn't match the user's exact symptom

The user sees `RuntimeError: bridge_call: transport closed` or `transport timed out`. From `transport.rs:36-41`:

```rust
TransportError::Io(e) => write!(f, "transport I/O error: {e}"),
TransportError::Timeout => write!(f, "transport timed out"),
TransportError::Closed => write!(f, "transport closed"),
```

The silent-swallow path produces `TransportError::Io(BrokenPipe)` → `"transport I/O error: BrokenPipe"`. **NOT** what klippy is seeing.

So the user's bug is **read-side**: write succeeds, but no response comes back. Two ways for that to surface:
- **Timeout**: `KalicoHostIo::call`'s `rx.recv_timeout(timeout)` returns `Err(Timeout)` because the deadline elapsed before any completion arrived
- **Closed**: reactor transitioned to Closed via some path (most likely `poll_serial` Err or PhantomZero), `flush_all_completions` sent `Err(TransportError::Closed)` to the awaiting completion

### 4. Tracing tmc.py _do_enable down to the wire

Verified call chain end-to-end:

```
tmc.py:458   _do_enable(print_time)
tmc.py:463   self._init_registers()  # iterates all configured TMC registers
tmc.py:326   self.mcu_tmc.set_register(reg_name, val, print_time)
tmc2130.py:380 set_register(...)  # holds self.mutex; up to 5 retries
tmc2130.py:308 reg_write(reg, val, chain_pos, print_time)
bus.py:142   spi_transfer_with_preface(write_cmd, dummy_read, minclock=...)
mcu.py:148   send_with_preface(...)  # bridge mode at line 156-163
mcu.py:162     preface_cmd.send(preface_data, minclock=minclock)  # spi_send (FireAndForget)
mcu.py:163     return self._bridge_send(data)                      # spi_transfer (bridge_call)
mcu.py:137   _bridge_send → self._serial.send_with_response(msg, self._response)
```

So each TMC register write fires:
- 1× `spi_send` as `ReactorCommand::FireAndForget` (preface, no response)
- 1× `spi_transfer` as `ReactorCommand::Submit` (bridge_call, awaits `spi_transfer_response`)

For TMC5160, `_init_registers` iterates `self.fields.registers`. A typical TMC5160 stepper config touches 8-12 registers (CHOPCONF, IHOLD_IRUN, COOLCONF, PWMCONF, TPOWERDOWN, TPWMTHRS, GLOBALSCALER, etc.). With up to 5 retries per register, that is **40-60 bridge_call round trips** during a single `_do_enable`.

The `_do_enable` runs **inside a `printer.get_reactor().register_callback(cb)`** callback (tmc.py:513), which runs on klippy's main reactor thread. Each `bridge_call` does `py.allow_threads(|| io.call(...))` — the GIL is dropped, so other Python threads (e.g., bridge poller threads) can run. More importantly, the **motion-bridge planner thread** (a separate Rust thread inside motion-bridge that processes move callbacks) is independently submitting `kalico_call`s for `load_curve`/`push_segment` for the same move that triggered the stepper enable.

So during a real `_do_enable`, the wire has interleaved:
- Klippy's TMC SPI burst (Submit + FireAndForget pairs)
- Motion-bridge's curve/segment uploads (KalicoCall)

Both share `submission_tx` channel and the reactor's single thread. Both produce wire frames that ride the same USB-CDC pipe.

### 5. Tried to reproduce in sim_klippy

Wrote `tools/sim_klippy/tests/test_bridge_stall_repro.py` with 3 scenarios:
1. `test_linear_move_after_set_kinematic_position` — exactly mirror user's last failing sequence
2. `test_burst_of_linear_moves` — 5 moves back-to-back, ~1.5 s of motion
3. `test_long_move_during_tmc_poll` — long Z move spanning multiple 1Hz tmc-poll cycles

**All 3 PASSED in docker.** The existing `test_g28_homing_actual` tests also pass. The sim does not reproduce.

This is significant: the sim uses a **PTY** (pseudo-terminal) for the host↔MCU connection rather than a real USB-CDC bulk pipe. PTYs:
- Have a single kernel-side buffer (typically 4 KB) shared between read and write
- Do not have backpressure timing characteristic of USB FS bulk endpoints
- Read/write don't go through the USB SOF / micro-frame timing

The race must require the timing characteristics of USB FS bulk transactions: 1 ms frames, NAKs on bulk OUT when device EP FIFO is full, host-side URB queue depth, kernel timeouts.

### 6. The pin-overrides.toml smoking gun

`tools/sim_klippy/pin-overrides.toml:60-77`:

> position_max / position_endstop are also shrunk (from 300 → 20). The unmodified range produced a ~4.5 s homing move (corexy doubles the stepper distance) which raced klippy's ~1 Hz tmc.py periodic stallguard query against the bridge — **a separate sim-infra bug surfaces as "RuntimeError: bridge_call: transport closed" mid-homing**, before past-end-time can fire. Shrinking the range keeps the no-trigger move under ~0.3 s so it completes inside one tmc-poll window.

This is the SAME bug. A previous session documented it but never root-caused it; the fix was a workaround (avoid the race window). Verified by reading the workaround:

```toml
[stepper_x.config_set]
endstop_pin = "^gpiochip0/gpio200"
use_sensorless_homing = "False"
homing_retract_dist = "0"
min_home_dist = "0"
position_endstop = "20"     # was 300
position_max = "20"         # was 300
```

Note that the previous session ATTRIBUTED the bug to "tmc.py periodic stallguard query racing the bridge". The TMCErrorCheck timer (`tmc.py:213` `_do_periodic_check`) fires at 1 Hz and does 2-3 SPI reads per stepper. With multi-stepper configs, that is a burst of bridge_calls every second. The previous session's hypothesis: when this 1 Hz burst lands DURING a long-running motion (where motion-bridge is concurrently producing curves/segments), the bridge_call hangs.

**This is consistent with what we see on real hardware now:**
- `_do_enable` is itself a burst of bridge_calls (functionally equivalent to the periodic stallguard burst)
- It lands during motion-bridge's planner thread submitting load_curve / push_segment
- Same race, different trigger event

### 7. H7 firmware TX backpressure: confirmed silent-drop on full buffer

`src/generic/usb_cdc.c:42`: TX buffer is `transmit_buf[320]` shared between Klipper-protocol responses (`console_sendf`, line 83) and kalico-native frames (`kalico_console_write_raw`, line 108).

**Both writers silently drop on full buffer:**

`console_sendf` line 88-90:
```c
if (tpos + max_size > sizeof(transmit_buf))
    // Not enough space for message
    return;
```

`kalico_console_write_raw` line 111-112:
```c
if ((uint32_t)tpos + (uint32_t)len > sizeof(transmit_buf))
    return -1;
```

`kalico_dispatch.c:138`: caller of `kalico_console_write_raw` IGNORES the -1 return:
```c
kalico_console_write_raw(tx_buf, (uint16_t)total);
```

So under host-side stall:
- 10 Hz `kalico_status_v6` events — DROPPED silently
- LoadCurveResponses — DROPPED silently
- spi_transfer_responses — DROPPED silently

USB-CDC EP BULK IN size = 64 bytes (`src/generic/usb_cdc_ep.h:16`). Buffer fills in:
- ~6.5 frames at 64 B each, or
- ~16 LoadCurveResponses at 20 B each, or
- ~10 spi_transfer_responses at 30 B each

If the host stops reading USB-CDC for ~1 second, the buffer fills entirely from 10 Hz status events alone. Once full, all subsequent responses are silently dropped.

### 8. Subagent's parallel investigation (negative findings)

Subagent (general-purpose, ~1 hour) read reactor.rs + dependencies in full and cross-checked all 10 hypotheses for shared-state bugs between Klipper-protocol and kalico-native paths:

| Hypothesis | Verdict |
|------------|---------|
| Mutexes shared between kalico_call decode and Submit decode | **NEGATIVE** — no shared locks in reactor |
| Channel order / submit-path starvation | **NEGATIVE** — FIFO mpsc, MAX_SUBMITS_PER_ITER=4 |
| Frame parser misclassification after recent kalico_call | **NEGATIVE** — first-byte discrimination is robust |
| Sequence-number collision between protocols | **NEGATIVE** — kalico uses correlation_id, klipper uses send_seq nibble |
| `kalico_state.identified` gating affecting Submit | **NEGATIVE** — only gates KalicoCall |
| TX-side blocking on USB-CDC during LoadCurve | **MEDIUM** plausibility, unproven |
| RX-side parser leaving state corrupted | **NEGATIVE** — demuxer state machine resets cleanly |
| Abandoned entry GC eviction during stall | **N/A** — symptom would be DispatcherTimeout, not Timeout/Closed |
| `transition_closed` / `disconnect` triggers from kalico path | **NEGATIVE** — kalico errors don't tear down klipper-classic state |
| `flush_all_completions` running unexpectedly | **NEGATIVE** — only runs on state==Closed transition |

**Conclusion from subagent**: no logical shared-state bug between the two protocols' decode paths. The bug is in the physical / timing layer: USB-CDC TX URB stall combined with the reactor's silent-swallow-on-direct-Io-error gap.

---

## Alternative hypothesis discovered late — SPI peripheral hang

After writing the synthesized hypothesis below, I went back and re-read `src/stm32/stm32h7_spi.c::spi_transfer` (line 128-157) and found:

```c
while (len--) {
    writeb((void *)&spi->TXDR, *data);
    while ((spi->SR & (SPI_SR_RXWNE | SPI_SR_RXPLVL)) == 0)  // ← infinite wait for RX
        ;
    /* ... */
}
while ((spi->SR & SPI_SR_EOT) == 0)  // ← infinite wait for End Of Transfer
    ;
```

**Both wait loops have NO TIMEOUT.** If the SPI peripheral hangs (CS polarity glitch, MISO transient, FIFO state inconsistency from a previous interrupted transfer, hardware errata), the firmware deadlocks. The same is true for `src/stm32/spi.c::spi_transfer` on F4.

Klipper's scheduler is cooperative — once `usb_bulk_out_task` enters `command_spi_transfer`, no other task runs until it returns. **A hung SPI hardware wedges the entire MCU main loop.** Symptoms would be:
- `transmit_buf` stops draining (no `usb_bulk_in_task`)
- `runtime_drain` doesn't run → engine_status frozen
- IWDG-kicker (`watchdog_reset`) doesn't run → after 30 s, IWDG resets
- Host sees the device USB-CDC pipe go silent

**This matches the user's BKPSRAM data more cleanly than the buffer-fill hypothesis:**
- `samples_taken = 166781 ≈ 5.5 s` would be the count *just before* the SPI hang (last task tick before the wedge)
- `engine_status = 0` because `runtime_drain` never ran
- IWDG1RSTF in RSR is the inevitable downstream outcome

**Why SPI might hang here specifically:**
- TMC SPI bus on Trident uses GPIO-driven CS (`SPI_CR1_SSI`, software CS)
- Each TMC stepper has its own CS pin but shares the SPI master peripheral
- A wrong CS polarity (e.g., active-high configured but TMC expects active-low) on the FIRST register write of `_do_enable` would shift bits out without latching them at the slave; receiver might not pull MISO; SPI peripheral could deadlock waiting for RXNE depending on slave electrical state
- Recent commit `3c59cdb63` mentioned "fix spi_bus name and CS active-low polarity" but it was for the SIM. Real-hardware CS polarity should come from `printer.cfg` (`!` prefix on cs_pin)

**Why this didn't surface during klippy startup `_init_registers`:**
- Connection setup writes registers at low rate, with SPI bus idle between writes
- Stepper enable burst (`_do_enable`) does writes back-to-back, possibly faster than the SPI peripheral resets between transactions
- Or: a specific register value (`CHOPCONF` with toff=0 → toff=4) causes the TMC to enter / exit power-down, glitching the CS line

### Recommended diagnostic for SPI-hang hypothesis

Add an SPI watchdog in firmware:

```c
// src/stm32/stm32h7_spi.c::spi_transfer
while (len--) {
    writeb((void *)&spi->TXDR, *data);
    uint32_t spi_deadline = timer_read_time() + timer_from_us(100);
    while ((spi->SR & (SPI_SR_RXWNE | SPI_SR_RXPLVL)) == 0) {
        if (!timer_is_before(timer_read_time(), spi_deadline)) {
            output("[trace-spi] SPI RX hang at byte %u, SR=0x%x",
                   (unsigned)(orig_len - len - 1), spi->SR);
            shutdown("SPI RX timeout");
        }
    }
    /* ... */
}
```

This converts a silent hang into a klippy shutdown with a clear log line. Even if it doesn't fully diagnose the cause, it eliminates the ambiguity between buffer-fill and SPI-hang hypotheses on the next repro.

**Confidence in SPI-hang hypothesis:** ★★★☆☆ — strong fit for the BKPSRAM data, but no direct evidence yet. Requires the proposed SPI watchdog to land on hardware to confirm. If `output("[trace-spi] SPI RX hang ...")` appears in klippy.log on repro, this hypothesis is correct. If not, fall back to the buffer-fill hypothesis below.

---

## Synthesized current best understanding

### The actual user-visible failure mode (read-side, not write-side)

1. **Connection setup succeeds.** klippy's `_handle_connect` runs `_init_registers` for all TMCs over the wire. This works because there's no concurrent kalico-native traffic at this point.

2. **First move (`G1`/`G28`/`_CLIENT_LINEAR_MOVE`) is queued.** Motion-bridge's planner thread starts processing the segment.

3. **Motion-bridge submits `kalico_call(LoadCurve)` × N** (one per axis curve, e.g., X, Y curves for a 1mm move). These ride the kalico-native channel. First 2 succeed cleanly — host got `LoadCurveResponse` for both.

4. **Klippy's `_handle_stepper_enable(is_enable=True)` callback fires** as part of move planning. Reactor schedules `_do_enable` via `register_callback(cb)`. `cb` runs on klippy's main reactor thread.

5. **`_do_enable` calls `_init_registers`** — iterates all configured TMC registers, each emitting `spi_send` (FireAndForget) + `spi_transfer` (bridge_call awaiting `spi_transfer_response`). Up to 5 retries per register on `set_register`. Result: a burst of 30-60+ bridge_call round trips queued onto the same `submission_tx` channel that the planner thread is already feeding.

6. **The wire saturates.** Host writes are flowing into the kernel CDC ACM driver's URB queue. The USB FS bulk-OUT endpoint has 1ms frame granularity. With MAX_PENDING_BLOCKS=12 unacked frames in the reactor's window, plus the kernel's own URB queue, the device-side bulk-OUT FIFO fills.

7. **MCU's `usb_bulk_out_task` falls behind** processing the burst. The kalico_demux_pump loop dispatches each frame synchronously to `command_find_and_dispatch` (which runs `command_spi_transfer` and emits the response via `console_sendf` into `transmit_buf`) or to `kalico_dispatch_frame` (which similarly emits responses into `transmit_buf` via `kalico_console_write_raw`).

8. **The MCU's `transmit_buf[320]` overruns.** 10 Hz status events are STILL being emitted (every 100 ms, via runtime_tick.c:362's `kalico_native_emit_status_event`). LoadCurveResponses and spi_transfer_responses are simultaneously being queued. If the host's reactor falls behind reading (e.g., its `tick_once` cycle is consumed by the burst of submission processing), `usb_bulk_in_task` can't drain the bulk-IN endpoint fast enough, the buffer fills, and `console_sendf` SILENTLY DROPS the next response.

9. **The dropped response is the first `spi_transfer_response`.** Klippy's bridge_call for that register is still parked on `rx.recv_timeout(5s)`. Five seconds later → `TransportError::Timeout` → `"transport timed out"`. 

   OR, in alternate timing: the host's CDC ACM kernel driver detects the device side is non-responsive (USB SOF stalls / endpoint NAK timeouts), reports BrokenPipe on the next host read. Reactor's `poll_serial` returns Err → `state = Closed` → `flush_all_completions` sends `Err(Closed)` to the awaiting completion → klippy sees `"transport closed"`.

10. **klippy crashes.** `RuntimeError: bridge_call: transport timed out` (or closed) propagates up from `_init_registers` → `_do_enable` → the registered callback → klippy's main reactor catches the exception and calls `printer.invoke_shutdown` (tmc.py:481). Klippy's shutdown handler runs but klippy itself eventually exits or stays in shutdown state.

11. **H7 IWDG fires DOWNSTREAM.** Once klippy stops draining USB, the H7's `transmit_buf` stays full forever. The H7's `runtime_drain` task observation: liveness flag is `runtime_liveness_ok = true` because the task itself is running fine; it just can't emit anything. But klippy's main reactor that pets the H7's IWDG (via periodic Klipper-protocol commands) is dead. After ~30 seconds of no IWDG kicks, the H7 IWDG fires → reset → BKPSRAM diagnostic snapshot captured (engine_status=0, samples=166781, RCC->RSR shows IWDG1RSTF).

### Why this doesn't reproduce in sim

PTYs (pseudo-terminals) used by `sim_klippy` for the host↔MCU connection have a single kernel buffer (typically 4-8 KB). When the buffer fills, writes block and reads return what's available. **No bulk-endpoint NAK semantics, no USB SOF timing, no per-URB queue.** The host's kernel doesn't experience the device side falling behind — it just blocks `write()` until the device's reader catches up. PTY reads also don't return EOF the way a CDC pipe does on USB device disconnect.

Real USB-CDC has:
- 64-byte EP FIFOs per direction
- 1 ms USB frame timing
- NAK on full bulk-OUT EP — host kernel queues URBs, retries on next frame
- Eventual host-side timeout if device repeatedly NAKs (varies by driver, often 500 ms to several seconds)
- Possible BrokenPipe on `read()` if device disconnects mid-transfer

**Sim cannot reproduce a buffer-fill race that requires specific USB FS timing.** This is a known testability gap.

### Why the 5-hour autonomous research couldn't fully nail it

1. **Cannot test on real hardware** (per user instruction).
2. **Cannot reproduce in sim** (just demonstrated).
3. **Renode** has STM32F4 board models but no STM32H723 USB-CDC model that would exercise the FS bulk endpoint behavior accurately.
4. **The bug requires interleaving**: klippy's _do_enable burst + motion-bridge planner thread + tmc periodic stallguard + 10 Hz status events on a USB FS pipe with realistic timing.

To fully prove the hypothesis, one of:
- **Run the diagnostic patch** I wrote (see "Recommended next diagnostic" below) on real hardware
- **Build a Renode STM32H7 model with USB-CDC** that mirrors host-side bulk OUT/IN timing
- **Use a USB-CDC virtualization tool** (`usbip`, `vhci-hcd`) to bridge a real H7's USB pipe through software where we can inject delays/drops

None of these is a 5-hour task without hardware access.

---

## What we KNOW vs what we BELIEVE

### KNOW (with high confidence)

- **Finding 1 silent-swallow is real and reproducible.** 5 unit tests in `silent_swallow_findings` pin the behavior. This is a latent defect that CAN affect klippy under transient write faults, but is NOT the user's current observed symptom (wrong error string).
- **Cross-talk find_match misrouting is FALSIFIED.** No `[xtalk]` events in user's run, await_n=1 consistently.
- **No shared-state bug** between kalico-native and Klipper-classic paths. Subagent's deep read of reactor.rs + parser + demux + window + bridge.rs + producer.rs found no logical contention.
- **H7 firmware silently drops TX frames** when transmit_buf overflows. Verified in `console_sendf` and `kalico_console_write_raw`.
- **"position_max=20 workaround" was a previous session's attempt to avoid the same bug** in sim — workaround, not fix.
- **Sim does not reproduce the bug** under 3 deliberate stress scenarios.
- **Test coverage gap:** no test exists for the immediate-handle_command-Submit-with-write-failure path until the 5 new tests landed today.

### BELIEVE (with strong but circumstantial evidence)

- The bug is a **USB-CDC TX buffer-fill race on the H7 firmware**, triggered by the combination of:
  - `_do_enable`'s 30-60 bridge_call burst on tmc.py register init
  - Motion-bridge planner thread's concurrent kalico_call traffic
  - 10 Hz `kalico_status_v6` events on the events channel
  - Klipper-classic spi_transfer_response on the responses channel
  - Host reactor falling momentarily behind on reads while processing the submission burst

- The "transport closed" variant comes from the kernel reporting BrokenPipe on the next read after the device's USB-CDC pipe stalls. The "transport timed out" variant comes from the caller's 5 s `recv_timeout` firing first when the kernel doesn't escalate to BrokenPipe.

### DO NOT KNOW (but matter)

- **Exactly how many spi_transfer responses get dropped.** Could be just one (the first), or could be a sustained drop window.
- **Whether the H7's USB-CDC TX URB completion actually waits for host ACK** under heavy load, or whether it's fire-and-forget. If the latter, the H7's `usb_bulk_in_task` keeps writing to the bulk-IN endpoint and the kernel-side queue grows without backpressure to the firmware — different failure mode.
- **What klippy's reactor does** when `_do_enable` raises. Looking at tmc.py:481, `self.printer.invoke_shutdown(str(e))`. So klippy goes into "shutdown" state, which is recoverable but leaves the H7 in an undefined state until full restart.
- **Whether there's a clean repro on a different USB FS device** (e.g., an STM32F411 with similar firmware). If the bug is H723-specific, there may be an additional hardware quirk (DMA, TXFIFO sizing).

---

## Recommended next diagnostic (when hardware is available)

Add bracketed timing instrumentation to the host's reactor `write_frame` method (`reactor.rs:201-205`):

```rust
pub(crate) fn write_frame(&mut self, frame: &[u8]) -> Result<(), TransportError> {
    let t0 = std::time::Instant::now();
    let proto = if !frame.is_empty() && frame[0] == 0x55 { "kalico" } else { "klipper" };
    let result = (|| {
        self.io.write_all(frame)?;
        self.io.flush()
    })();
    let dt = t0.elapsed();
    if dt > std::time::Duration::from_millis(5) || result.is_err() {
        eprintln!(
            "[trace-write] proto={proto} bytes={} dt_ms={:.2} result={:?}",
            frame.len(), dt.as_secs_f64() * 1000.0, result
        );
    }
    result
}
```

This single tap distinguishes (in 1 minute of live tracing):
- **Bulk-OUT URB stall**: writes that block for 100+ ms before returning Ok or failing → confirms TX-side stall
- **Clean BrokenPipe**: writes return immediately Err(Io(BrokenPipe)) → confirms Finding 1's silent-swallow
- **Both succeed but read fails**: writes consistently fast, no errors → bug is on the read path → instrument `poll_frames_until` next

Pair with H7 firmware diagnostic in `kalico_dispatch.c:138`:

```c
int written = kalico_console_write_raw(tx_buf, (uint16_t)total);
if (written < 0) {
    output("[trace-tx] kalico drop chan=%c len=%u tpos_full=%u",
           channel, total, transmit_pos);
}
```

And `usb_cdc.c:90`:

```c
if (tpos + max_size > sizeof(transmit_buf)) {
    // Buffer full — silent drop. Add output trace.
    output("[trace-tx] klipper drop msg_max=%u tpos=%u", max_size, tpos);
    return;
}
```

These two together would prove or refute the buffer-fill hypothesis in <5 minutes of live G1/G28 testing.

---

## Recommended fixes (post-confirmation)

### Fix 1 (defect, regardless of root-cause): immediate-dispatch paths must transition Closed on Io error

`handle_command::Submit`/`FireAndForget`/`KalicoCall` should mirror `drain_pending_submissions`:

```rust
ReactorCommand::Submit { call_id, cmd, expected_response_name, completion, deadline } => {
    let expected_oid = extract_oid_from_cmd(&cmd);
    match self.parser.encode(&cmd) {
        Ok(payload) => {
            if let Err(e) = self.dispatch_submission(...) {
                let is_io = matches!(e, TransportError::Io(_));
                let _ = completion.send(Err(e));
                if is_io {
                    if self.pending_host_fault.is_none() {
                        self.pending_host_fault = Some(/* HostDisconnect */);
                    }
                    self.state = ReactorState::Closed;
                }
            }
        }
        Err(e) => { let _ = completion.send(Err(TransportError::Parse(format!("{e:?}")))); }
    }
}
```

When this lands, flip the `silent_swallow_findings::*` tests to assert the new behavior (state == Closed after Io error).

### Fix 2 (root cause hypothesis): give the H7 TX path bounded backpressure instead of silent drop

Two options, both substantive:

**Option A — host-side rate limiting:** the reactor's tick loop currently processes up to 4 commands per tick. Drop this to 1 when the USB write queue depth is high (track via tx_log monotonic byte count + observed read drains). This costs throughput but bounds the buffer fill rate.

**Option B — MCU-side reliable TX queue:** replace `transmit_buf[320]` with a ring buffer that pushes back on producers via a return code, and have producers (kalico_dispatch + console_sendf) handle the back-pressure. Requires reworking `kalico_native_emit_status_event` to either retry from a periodic task or skip-and-reflag-stale.

Option B is the architecturally correct fix per the kalico CLAUDE.md non-negotiable: "Real time communication with MCUs, no queue-based offload." The 320-byte transmit_buf is itself a queue; the silent-drop semantics violate the guarantee that any frame queued is actually sent.

### Fix 3 (preventive): make the failure mode loud, not silent

In `kalico_dispatch.c::kalico_transport_send_frame`, abandon the `kalico_console_write_raw(...);` (return value discarded) pattern. The MCU should log a fault when it has to drop a kalico frame. The existing `output(...)` mechanism is suitable — these dropped-frame events would then appear in klippy.log so future incidents are debuggable from the log alone.

---

## Test artifacts produced this session

### Rust unit tests — committed-pending
- `rust/kalico-host-rt/src/host_io/reactor.rs` (silent_swallow_findings module, 5 new tests, ~200 lines)
- All 186 lib tests pass after the additions (181 prior + 5 new)

### Sim integration test (negative repro) — committed-pending
- `tools/sim_klippy/tests/test_bridge_stall_repro.py` (3 scenarios, all PASS — sim does NOT reproduce)

### Files touched
- `rust/kalico-host-rt/src/host_io/reactor.rs` — added silent_swallow_findings module
- `tools/sim_klippy/tests/test_bridge_stall_repro.py` — new file
- `docs/superpowers/specs/2026-05-09-bridge-call-stall-investigation.md` — this document

No production behavior changed. Tests are characterization-only.

---

## Open questions for next session (with hardware)

1. Run the `[trace-write]` instrumentation (host-side) + `[trace-tx]` instrumentation (firmware-side) on real hardware. Reproduce with `G28 X` or `_CLIENT_LINEAR_MOVE X=1 F=6000`. Confirm: are there long write_frame durations? Are there `[trace-tx] drop` events on MCU?

2. If MCU drops are confirmed: implement **Fix 2 Option B** (reliable TX queue with backpressure). This is a Step 7-D scope item.

3. If long write durations are confirmed but no MCU drops: the bug is on the host's USB-CDC kernel side. Investigate whether the cfmakeraw setup in `mod.rs:230-241` is missing additional flags for low-latency mode (`ASYNC_LOW_LATENCY` ioctl on Linux).

4. **Independently of the root cause: land Fix 1.** The silent-swallow defect is real, has tests, and is fixable in ~30 lines. Should not block on root-causing the user's exact symptom.

5. **Document the sim-fidelity gap.** Update `docs/kalico-rewrite/dependency-graph.md` or wherever sim coverage is tracked — "USB FS bulk endpoint timing-sensitive races are not testable in sim_klippy; require Renode-with-USB-model or hardware."

---

## 2026-05-09 PM — Hardware repros with diagnostics

User came back online, hardware available. Three flash cycles with progressive instrumentation on real H7. Repros:

### Summary

`SET_KINEMATIC_POSITION X=100 Y=100 Z=10 + G1 X101 F6000` reliably crashes klippy at `tmc.py:_do_enable`. The crash signature varies (`transport closed` vs `transport timed out`) but the ROOT TIMING is consistent.

### Smoking-gun trace (3rd repro, with full host instrumentation)

```
10:51:27.891260 [py-trace] _bridge_send enter (call_id 271)
10:51:27.892923 [trace-write] proto=klipper bytes=13 dt_ms=0.70 OK     ← spi_send (FireAndForget)
... 509ms gap with reactor processing only beacon/analog reads ...
10:51:28.402062 [trace-write] proto=klipper bytes=13 dt_ms=509.12 OK   ← spi_transfer Submit BLOCKED 509ms!
10:51:28.402117 [trace-await] +add call_id=271 seq=605
10:51:28.403169 [trace-rto] firing: front_seq=604 unacked_n=2 rto_ms=25 send_seq=606 recv_seq=604
10:51:28.403198 [trace-write] proto=klipper bytes=27 dt_ms=0.01 Io(Other,None)
10:51:28.404264 [trace-rto] firing: ...
10:51:28.404300 [trace-write] proto=klipper bytes=27 dt_ms=0.01 Io(Other,None)
... 5 retransmits in 4ms, all fail Io(Other) → Io(BrokenPipe) ...
10:51:28.407088 [trace-write] proto=klipper bytes=27 dt_ms=0.00 Io(BrokenPipe,None)
10:51:28.407175 [py-trace] _bridge_send EXCEPTION
10:51:28.547649 systemd: klipper.service: Main process exited 255
```

### Confirmed root mechanism

1. Host sends 9 successful TMC SPI register writes (call_ids 263-271). Each write_frame returns in <1 ms.
2. Concurrently, motion-bridge's planner thread sends 2 LoadCurves (kalico-native, 720 bytes each) — both succeed.
3. Planner sends PushSegment (kalico-native, 55 bytes) — sent OK but **no PushSegmentResponse ever arrives**.
4. Klippy's `_do_enable` continues — sends spi_send (preface) for the 10th register write — fast (0.70 ms).
5. **Klippy issues spi_transfer for the 10th register. write_frame BLOCKS for 509 ms.**
6. During the 509 ms, the reactor thread is stuck in `io.write_all()` / `io.flush()`. No host writes happen. Reads keep producing klipper-protocol frames (beacon, analog_in_state, etc) but ZERO kalico-protocol frames.
7. write_frame eventually returns OK after 509 ms — Linux kernel's CDC ACM driver finally flushed the URB.
8. Reactor's RTO timer immediately fires (the 25 ms RTO had elapsed during the 509 ms block). Retransmit attempted.
9. Retransmit's write_frame returns `Io(Other,None)` (errno=0 — generic I/O error). Reactor's RTO loop ignores the error (`let _ = ...`). Tick continues.
10. Next tick: RTO fires again (front entry's sent_at unchanged, RTO still expired). 5 retransmit attempts in 4 ms, all fail `Io(Other,None)` → final `Io(BrokenPipe,None)`.
11. Next `poll_serial` reads from port — also fails BrokenPipe. State → Closed. `flush_all_completions` sends `Err(Closed)` to klippy's awaiting completion.
12. Klippy's bridge_call returns `transport closed` → RuntimeError → klippy crashes.

### Why the 509 ms write block

The block happens on the FIRST klipper Submit AFTER motion-bridge's PushSegment kalico-native frame went out. The MCU's USB-CDC bulk-OUT EP must be NAKing for ~500 ms. This means **the H7's `usb_bulk_out_task` was starved for 500 ms** — couldn't drain the EP FIFO, host's URB sat in kernel queue, kernel eventually returned (or nearly returned) timeout-like error.

What could starve `usb_bulk_out_task` for 500 ms? Possibilities:
- **A long-running foreground task or IRQ.** SPI watchdog never tripped → not SPI hardware hang. `[trace-rt]` never fired → not push_segment / load_curve / kalico_demux_pump.
- **Repeated IRQ preemption** from TIM5 (40 kHz runtime tick). But `engine_status=0` throughout → engine never armed → `runtime_handle_tick` should be light. UNLESS a brief arming-then-faulting cycle ran in IRQ context that we never observed because status events stopped.
- **kalico_runtime_push_segment** doing something synchronously that takes 500 ms before returning. The C-side wrapper times this and doesn't trip the 5 ms threshold per `[trace-rt]`. So the Rust runtime call DOES return fast.

So the cause of the 500 ms USB-OUT NAK is in IRQ context or in some path I haven't instrumented. The most likely candidate: **the runtime engine briefly armed via PushSegment, TIM5 ISR ran heavy code (perhaps `kalico_runtime_tick` doing initial step-output calculation that takes longer than 25 µs at 40 kHz), and 40 kHz × 25 µs = 100% CPU in IRQ context, starving usb_bulk_out_task. After ~500 ms, the engine self-faulted (back to status=0) and IRQ load dropped — but by then the USB-CDC pipe was already corrupted.**

### Reactor defects exposed by this trace

A. **`write_retransmit` ignores write_frame errors** (reactor.rs:519 `let _ = self.write_retransmit(...)`). When the wire fails, retransmits keep firing in a tight loop (5 attempts in 4 ms in this trace), each returning Io error. The reactor never escalates to Closed via this path until poll_serial separately detects the broken pipe.

B. **Finding 1's silent-swallow gap was triggered** (`handle_command::Submit`'s `dispatch_submission` doesn't transition Closed on write Io error). However in this scenario the Submit's write actually returned OK (after 509 ms), so Finding 1 was bypassed — the failure mode came from RTO retransmits afterward.

C. **No `[trace-rt]` for IRQ context**. The runtime `kalico_runtime_tick` running in TIM5 IRQ has no instrumentation. Adding it would be invasive (output() from IRQ is unsafe) but a simple counter exposed via the periodic emit would tell us if IRQ is starving foreground.

### Earlier "transport timed out" runs (32s silence) — explained

The 32s silence runs from this morning had different shape — kernel's write returned without blocking, but then NO response came back, so klippy hit `recv_timeout(5s)` × 5 retries in `set_register`'s retry loop = 25 s + cleanup.

Wait — looking again at set_register, it does NOT retry on RuntimeError. So 5 timeouts can't accumulate that way. The 32 s must come from ANOTHER source. Most likely: invoke_shutdown's cleanup making more bridge_calls that also time out.

### Rejected hypotheses (with evidence from this run)

- ❌ **SPI hardware hang** — watchdog never tripped, klippy never got "spi rx timeout"
- ❌ **MCU TX buffer overflow** — no `[trace-tx]` kalico_drop or klipper_drop_count events, MCU kept sending Klipper-protocol frames during the failure window
- ❌ **Slow runtime call** (push_segment/load_curve/demux_pump) — `[trace-rt]` never fired (5 ms threshold), so all those calls completed quickly
- ❌ **Cross-talk** — `await_n=1` always, no `[xtalk]` events
- ❌ **No mutex contention between protocols** in reactor — verified by code reading

### Confirmed mechanism

- ✅ **Host-side write_frame BLOCKS 509 ms** — `[trace-write] dt_ms=509.12 result=OK`
- ✅ **Subsequent retransmits fail** Io(Other) → Io(BrokenPipe) — `[trace-write] result=Io(...)` events captured
- ✅ **MCU stops emitting kalico-channel frames** during the failure window — the host receives 0 kalico frames vs 273 klipper frames in the surrounding 32 s window
- ✅ **Reactor doesn't escalate write retransmit errors to Closed** — `[trace-rto]` fires repeatedly, retransmit fails repeatedly, state stays Active until poll_serial separately detects broken pipe

### Updated recommended fixes

**Fix 1 (silent-swallow on Submit Io)**: still valid, lower priority since this run's failure didn't trigger it.

**Fix 2 (MCU TX reliable queue)**: less relevant — MCU TX buffer wasn't the issue this run.

**Fix 5 (SPI watchdog)**: WORKING — never tripped, ruling out SPI hangs.

**NEW Fix 6 — RTO retransmit error escalation**: when `write_retransmit` returns Io error, reactor should treat it like a write fault and transition Closed. Currently `let _ = self.write_retransmit(...)` discards the error. The fix:

```rust
if let Some(front) = self.unacked_window.front() {
    let now = self.clock.now();
    if now >= front.sent_at + self.rtt.current_rto() {
        if let Err(e) = self.write_retransmit(RetransmitTrigger::TimeoutDriven) {
            if matches!(e, TransportError::Io(_)) {
                self.transition_closed_on_io_fault();
            }
        }
    }
}
```

**NEW Fix 7 — Limit RTO retransmit retries per RTO interval**: currently the reactor can retransmit 5 times in 4 ms when writes fail (each retransmit fires the next tick's RTO immediately because sent_at didn't update). Add a cap or update sent_at on retransmit attempt so RTO doesn't fire-storm.

**NEW Fix 8 — Investigate IRQ-side runtime starvation**: add a foreground "heartbeat" counter that gets bumped from `usb_bulk_out_task` and read from a foreground watchdog or output. If it doesn't increment for >100 ms, log it. This would catch the 500 ms IRQ-storm scenario. The watchdog liveness gate (`runtime_liveness_ok`) already does something similar but for the runtime engine, not USB.

### Outstanding question

**What exactly causes the 500 ms USB-OUT NAK?** With current diagnostics ruled out SPI / runtime calls / TX buffer. Most likely IRQ-side TIM5 storm, but engine_status=0 throughout. Could be:
- Engine briefly armed in IRQ but reverted before any status emit captured RUNNING state
- Some other IRQ source (USB OTG itself?) firing aggressively
- The H7's USB-CDC bulk-OUT EP getting NAK-ed by the device-side driver for some firmware-internal reason

To resolve: instrument TIM5 IRQ entry/exit count + IRQ duration sum, expose via periodic emit. If TIM5 IRQ count exceeds expected (~4000/100ms at 40 kHz) or duration sum approaches the 100ms emit interval, we have IRQ starvation.

---

## Confidence summary

| Claim | Confidence |
|-------|-----------|
| Finding 1 silent-swallow is a real defect | ★★★★★ — 5 passing tests pin it |
| Cross-talk hypothesis is FALSIFIED | ★★★★★ — user's instrumentation showed zero events |
| Reactor has no logical shared-state bug between protocols | ★★★★☆ — subagent + my own read agree |
| H7 firmware silently drops TX frames on full buffer | ★★★★★ — verified in C source |
| Real-world failure is USB-CDC TX URB stall + reactor read starvation | ★★★☆☆ — strong circumstantial; needs the proposed diagnostic to confirm |
| Sim cannot reproduce due to PTY-vs-USB-FS timing differences | ★★★★☆ — verified PTYs don't backpressure same way; 3 stress tests passed |
| previous session's "sim-infra bug" workaround documents the same race | ★★★★☆ — pin-overrides.toml comment matches symptoms exactly |
| Fix 1 (state=Closed on immediate Io error) is correct | ★★★★★ — drain path already does it; symmetry argument |
| Fix 2 Option B (reliable TX queue on MCU) is correct | ★★★★☆ — matches CLAUDE.md "no queue-based offload" non-negotiable |
| User's exact error strings (closed/timeout) come from read-side, not Finding 1's write-side | ★★★★★ — verified by error-string tracing in transport.rs |
