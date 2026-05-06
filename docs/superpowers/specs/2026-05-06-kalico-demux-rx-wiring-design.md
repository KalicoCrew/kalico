# Kalico-Native RX Wiring on STM32 — Design

**Status:** draft, awaiting user review
**Author:** brainstormed 2026-05-06 (with architect-reviewer pass)
**Scope:** wire `kalico_demux_feed_byte` into the STM32 USB CDC and USART RX paths so host→MCU fork-native frames reach `kalico_dispatch_frame`. Mirror the existing `src/linux/console.c` integration pattern with three correctness fixes the linux version didn't need.

## 1. Problem

The fork-native protocol on the MCU is currently TX-only:

- TX: `src/kalico_dispatch.c::kalico_native_emit_status_event` / `_credit_freed` / `_fault_event` push frames out via `kalico_console_write_raw` into the transport's TX buffer. Working on USB CDC and USART (commits `40a9cbc49` and `115f6d00b`).
- RX: `src/kalico_demux.c::kalico_demux_feed_byte` and `src/kalico_dispatch.c::kalico_dispatch_frame` are compiled but have **zero callers** on STM32. The `src/linux/console.c::console_task` does wire them up, but the STM32 transport tasks (`usb_bulk_out_task`, `console_task`) don't.

Consequence (verified end-to-end against the F446 Renode sim on 2026-05-06): host-side `QueryRuntimeCaps` (the per-MCU sizing handshake message added in the runtime-sizing plan) gets dropped. Host bridge times out, falls back to large-profile defaults for every MCU. Per-MCU cap enforcement is a no-op. Any future host→MCU fork-native command is similarly broken.

## 2. Goals

1. STM32 USB CDC RX feeds bytes through `kalico_demux_pump` and dispatches surfaced klipper / kalico frames identically to the linux model.
2. STM32 USART RX does the same, with the IRQ-safety dance the linux model can ignore.
3. Linux RX is refactored to call the same shared helper, removing ~50 lines of inline byte-loop and a 4 KB `klipper_only_buf` accumulator that the architect-reviewer found was unjustified slack.
4. AVR / non-runtime builds keep the original direct `command_find_and_dispatch` flow — the demuxer must not pull into builds that don't enable the runtime.
5. Existing functionality (klipper command stream throughput, ack pacing, bootloader-request magic-string detection) is preserved bit-for-bit.

## 3. Non-goals

1. Replacing the legacy klipper command stream. Both protocols continue to coexist; this spec only adds the missing RX leg of the fork-native side.
2. Renaming `kalico_*` symbols (existing fork-rename pass is separate).
3. Adding RECEIVE_WINDOW advertisement on USB CDC (orthogonal).
4. Changing host-side framing or the wire protocol.
5. Adding DMA-driven RX or any IRQ-priority changes.

## 4. Architecture

### 4.1 New public function

A new public function in `src/kalico_demux.c`:

```c
// Drain a buffer of bytes through the demuxer state machine, dispatching
// klipper frames via command_find_and_dispatch and kalico-native frames
// via kalico_dispatch_frame as they surface. State persists across calls,
// so partial frames at buffer boundaries are handled correctly.
//
// Caller contract:
//   - `buf` must remain valid for the duration of the call but is not
//     retained afterward.
//   - Bootloader-request magic-string detection MUST run separately
//     against `buf` BEFORE this call (the demuxer would consume the magic
//     bytes as a malformed klipper frame, hiding them from the post-pump
//     check). See §4.4.
void kalico_demux_pump(const uint8_t *buf, uint16_t len);
```

Implementation:

```c
void
kalico_demux_pump(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        kalico_demux_output_t out = kalico_demux_feed_byte(buf[i]);
        switch (out) {
        case KALICO_DEMUX_OUT_NONE:
            break;
        case KALICO_DEMUX_OUT_KLIPPER: {
            // Bootloader-request sentinel detection must happen here,
            // not in the transport task: the 32-byte sentinel can
            // arrive split across multiple task firings, in which case
            // a transport-side `rpos == 32` pre-pump check would never
            // fire. The demuxer reassembles the full sentinel as a
            // 32-byte "klipper frame" whether the bytes arrive in one
            // burst or many; checking after surfacing is the only
            // location that survives fragmentation.
            const uint8_t *kbuf = kalico_demux_klipper_buf();
            uint8_t klen = kalico_demux_klipper_len();
            if (CONFIG_HAVE_BOOTLOADER_REQUEST && klen == 32
                && !memcmp(kbuf,
                           " \x1c Request Serial Bootloader!! ~", 32))
                bootloader_request();   // does not return
            uint_fast8_t pop_count;
            command_find_and_dispatch(
                (uint8_t *)kbuf, klen, &pop_count);
            kalico_demux_consume();
            break;
        }
        case KALICO_DEMUX_OUT_KALICO:
            kalico_dispatch_frame(
                kalico_demux_kalico_channel(),
                kalico_demux_kalico_payload(),
                kalico_demux_kalico_payload_len());
            kalico_demux_consume();
            break;
        case KALICO_DEMUX_OUT_ERROR:
            kalico_demux_consume();
            break;
        }
    }
}
```

The demuxer's static `klipper_buf` is the staging area — no separate stack or bss copy. `command_find_and_dispatch` runs synchronously and does not retain the pointer past return, so `kalico_demux_consume()` immediately afterwards is safe. The cast away `const` on `kbuf` is benign: `command_find_and_dispatch` and its downstream `command_find_block` never write through the buffer (verified at `src/command.c:273-329`, `:361-370`); a future command handler that decodes a pointer pointing inside the staging buffer and mutates through it would change this — flag in a comment if introduced.

Re-entrancy note: handlers invoked through `command_dispatch` may call `console_sendf` (writes to the transport's `transmit_buf`, separate from `receive_buf`) and may schedule wake events that fire only after the current task returns. They do not recurse into `kalico_demux_pump`. Per-frame ack timing matches the legacy `command_find_and_dispatch` loop bit-for-bit.

### 4.2 Per-transport integration

Each of the three transports gates the new path behind `#if CONFIG_KALICO_RUNTIME`. The fallback branch (non-runtime) keeps the existing `command_find_and_dispatch` direct call so AVR-class builds drop all demuxer code via `--gc-sections`.

#### 4.2.1 `src/generic/usb_cdc.c::usb_bulk_out_task`

USB RX is task-context-polled (`usb_read_bulk_out`), not IRQ-driven into `receive_buf`. The reset-after-pump pattern is therefore safe. Bootloader-sentinel detection lives inside `kalico_demux_pump`'s `OUT_KLIPPER` branch (§4.1), so no separate pre-pump memcmp is needed here:

```c
#if CONFIG_KALICO_RUNTIME
    kalico_demux_pump(receive_buf, rpos);
    receive_pos = 0;
#else
    int_fast8_t ret = command_find_and_dispatch(
        receive_buf, rpos, &pop_count);
    if (ret) {
        uint_fast8_t needcopy = rpos - pop_count;
        if (needcopy) {
            memmove(receive_buf, &receive_buf[pop_count], needcopy);
            usb_notify_bulk_out();
        }
        rpos = needcopy;
    }
    receive_pos = rpos;
#endif
```

#### 4.2.2 `src/generic/serial_irq.c::console_task`

USART RX is IRQ-driven via `serial_rx_byte`. The runtime branch must atomically rebase `receive_pos` after the pump consumes the snapshot range, preserving any bytes the IRQ deposited during the pump call:

```c
#if CONFIG_KALICO_RUNTIME
    uint_fast8_t rpos = readb(&receive_pos);
    kalico_demux_pump(receive_buf, rpos);

    // Bytes that arrived during the pump (in [rpos, now)) must survive.
    // Read receive_pos and rebase under irq_save so an IRQ cannot fire
    // mid-memmove. The IRQ never writes below receive_pos at the time
    // it runs, and the task is the only writer that ever lowers it, so
    // a single irq_save-protected pass suffices — no retry loop needed
    // (unlike console_pop_input, which does memmove outside irq_save
    // and therefore must retry on a concurrent IRQ).
    irqstatus_t flag = irq_save();
    uint_fast8_t now = readb(&receive_pos);
    if (now == rpos) {
        receive_pos = 0;
    } else {
        uint_fast8_t tail = now - rpos;
        memmove(receive_buf, &receive_buf[rpos], tail);
        receive_pos = tail;
    }
    irq_restore(flag);
#else
    /* existing command_find_block + console_pop_input flow unchanged */
#endif
```

Bootloader-sentinel detection lives inside `kalico_demux_pump` (§4.1), so no separate pre-pump check is needed here. The demuxer assembles the 32-byte sentinel into its `klipper_buf` regardless of whether the bytes arrive in one or several USART RX bursts; the post-assembly memcmp inside pump catches it in either case.

#### 4.2.3 `src/linux/console.c::console_task`

Replace the inline byte loop + `klipper_only_buf` accumulator (lines 186-234) with:

```c
#if CONFIG_KALICO_RUNTIME
    kalico_demux_pump(receive_buf, ret > 0 ? ret : 0);
    receive_pos = 0;
#else
    /* legacy direct command_find_and_dispatch loop (pre-demuxer
       integration) */
#endif
```

Delete the `klipper_only_buf` static and the `klipper_only_pos` static. **`console_receive_buffer()` must remain** — it's part of the platform surface declared in `src/generic/misc.h:9` and called from `src/command.c:22, :30`. Restore it to return `receive_buf` (its pre-demuxer-integration behavior), since the staging buffer it currently returns is going away.

The `#if CONFIG_KALICO_RUNTIME` gate is added to match the STM32 transports and to fix a pre-existing latent issue: today `src/linux/console.c` uses `kalico_demux_*` symbols unconditionally while `kalico_demux.c` is built only under `CONFIG_KALICO_RUNTIME` (per `src/Makefile`), so a Linux build with the runtime disabled fails to link. The fallback branch reverts to the legacy direct dispatch (the shape that existed before commit `8f32f6fcd`). Since current Linux builds in this fork always enable `KALICO_RUNTIME`, the regression is theoretical, but the gate keeps the build matrix consistent and matches §4.2.1 / §4.2.2.

The bootloader-request check has no equivalent on linux (it's STM32-only via `CONFIG_HAVE_BOOTLOADER_REQUEST`), so the in-pump sentinel detection compiles out cleanly there.

### 4.3 Build gating

Three `#if CONFIG_KALICO_RUNTIME` islands across `usb_cdc.c`, `serial_irq.c`, and `linux/console.c`. The architect-reviewer suggested an unconditional `pump` shim that wraps `command_find_and_dispatch` on non-runtime builds, but the legacy paths have transport-specific pop_count handling (memmove, USB notify, console_pop_input) that doesn't fit a shared shim cleanly. Three small `#if` islands are the path of least surprise; we accept the minor duplication.

### 4.4 Bootloader-request magic string

The 32-byte sentinel ` \x1c Request Serial Bootloader!! ~` starts with `0x20` (= 32 decimal), which lies in the demuxer's `[KLIPPER_LEN_MIN=5, KLIPPER_LEN_MAX=64]` range. The demuxer therefore consumes it as a 32-byte "klipper frame" — accumulating all 32 bytes into its internal `klipper_buf` and surfacing `OUT_KLIPPER` once the count is reached.

The legacy detection (`pop_count == 32 && memcmp(receive_buf, sentinel, 32)` after `command_find_block` failure) operated on the raw transport receive buffer at task-entry time. That location no longer works in the new design for two reasons:

1. **Fragmentation hazard.** The sentinel can arrive split across multiple `console_task` firings (USART byte-IRQ accumulates incrementally; USB stack may submit partial transfers). A pre-pump `rpos == 32` check fires only on a configuration where all 32 bytes happen to be present at task entry — fragile.
2. **The demuxer would consume the bytes anyway.** Even if a pre-pump check existed, by the time the next pump runs the demuxer's state machine is already mid-frame on the sentinel's leading `0x20`.

Resolution (codified in §4.1's pump implementation): the memcmp runs **inside** `kalico_demux_pump` on the `OUT_KLIPPER` branch, against the demuxer's reassembled `klipper_buf`. The demuxer's per-byte state machine guarantees that all 32 sentinel bytes accumulate in `klipper_buf` regardless of how the bytes arrive at the transport — single burst, two halves, or one byte at a time. `bootloader_request()` does not return.

This works on both USART and USB CDC. `CONFIG_HAVE_BOOTLOADER_REQUEST` gates the check so non-bootloader builds drop it.

## 5. Data flow

```
USART/USB IRQ ─► receive_buf (existing)
                     ▼ (task context)
              console_task / usb_bulk_out_task
                     ▼
        bootloader-request magic-string check
                     ▼
              kalico_demux_pump
                     ▼
        kalico_demux_feed_byte (per-byte state machine)
              ▼               ▼
         OUT_KLIPPER     OUT_KALICO
              ▼               ▼
   command_find_and_dispatch  kalico_dispatch_frame
   (+ ack via console_sendf)  (+ kalico_native_emit_*)
```

`receive_buf` keeps its IRQ→task hand-off role. The demuxer state (`klipper_buf`, `kalico_buf`, internal pos counters, `state`) persists across `pump` calls so partial frames spanning two task firings work correctly.

## 6. Error handling and edge cases

| Scenario | Behavior |
|---|---|
| `OUT_ERROR` from demuxer (kalico CRC mismatch, frame > buf size, malformed length) | `kalico_demux_consume()` resets state; byte loop continues. Dropped frames force host-side correlation timeout, which the bridge already handles via fallback. |
| Klipper frame arrives malformed (`command_find_and_dispatch` returns 0 or <0) | Existing legacy behavior: dropped, host retransmits via seq/RTO. |
| Partial frame at buffer boundary | Demuxer state persists across pump calls. Next pump invocation continues consumption from where it left off. Verified by the per-byte state machine design. |
| Multiple frames in a single receive_buf | Pump processes them sequentially as they surface. Per-frame ack ordering matches the existing flow. |
| 32-byte bootloader sentinel | Caught by pre-pump memcmp; demuxer never sees the bytes. |
| IRQ-driven byte arrival during pump (USART) | Tail-memmove dance in §4.2.2 preserves them. |

## 7. Testing

1. **Linux sim regression**: `tools/sim_klippy/run_local.sh "G1 Z5 F600"`. Existing planner+bridge end-to-end test must still produce step pulses.
2. **H7 sim Phase-2 gate**: `tools/sim/run_phase2_gate.sh`. Existing wire-level bridge contract test against H7 firmware. Identify + LoadCurve + PushSegment over USART2.
3. **F446 sim QueryRuntimeCaps probe (the smoking gun)**: `tools/sim/probe_f446_caps.py`. Currently fails (frame dropped because RX path is unwired). After this work, must PASS — host receives `RuntimeCapsResponse(512, 524, 10, 4)` matching the small profile.
4. **C unit test for `kalico_demux_pump`** (in `src/tests/` or wherever the existing C tests live; if no such directory, skip): synthetic input buffer with `[partial-klipper-frame][complete-kalico-frame][complete-klipper-frame]` interleaved, asserts each surfaces in order with the right dispatch path. Optional — the sim probes are stronger evidence.
5. **AVR / non-runtime build**: confirm `make` for an AVR target still links and the demuxer TU is dropped (`arm-none-eabi-nm` showing no `kalico_demux_*` symbols in the final ELF, or equivalent for AVR).

## 8. Open questions

None blocking. One observation:

- The `kalico_demux.c::kalico_demux_klipper_buf()` accessor returns `const uint8_t *` but `command_find_and_dispatch` takes `uint8_t *`. The cast in §4.1 is benign (find_and_dispatch only reads from the pointer). If preferred, `kalico_demux_klipper_buf()` could return `uint8_t *` directly — minor style choice deferred to implementation.

## 9. References

- `src/linux/console.c::console_task` lines 131-236 — reference integration.
- `src/kalico_demux.c` — state machine being wired in.
- `src/kalico_dispatch.c::kalico_dispatch_frame` — RX dispatch entry.
- `src/generic/usb_cdc.c::usb_bulk_out_task`, `src/generic/serial_irq.c::console_task` — modification targets.
- `src/command.c::command_find_and_dispatch`, `src/command.h` (MESSAGE_MIN/MAX/SYNC) — legacy klipper protocol contract.
- `docs/superpowers/specs/2026-05-04-kalico-native-transport-design.md` §6 — original kalico-native demux spec.
- `docs/superpowers/specs/2026-05-06-runtime-sizing-per-mcu-design.md` §5 — the `QueryRuntimeCaps` consumer this work unblocks.
- Architect-reviewer pass 2026-05-06 (this brainstorm session) — IRQ-safety bug + stack→static buffer simplification + bootloader preservation flagged.
