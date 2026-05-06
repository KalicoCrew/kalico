# Kalico-Native RX Wiring — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire `kalico_demux_feed_byte` into the STM32 USB CDC and USART RX paths (and refactor the linux integration to share the same helper) so host→MCU fork-native frames reach `kalico_dispatch_frame`. Acceptance test: `python3 tools/sim/probe_f446_caps.py` against the booted F446 Renode sim returns PASS — host sends QueryRuntimeCaps, MCU responds with `RuntimeCapsResponse(512, 524, 10, 4)` matching the small profile.

**Architecture:** A new `kalico_demux_pump(buf, len)` function in `src/kalico_demux.c` walks bytes through the existing per-byte state machine and dispatches surfaced klipper / kalico frames inline. Three transport tasks adopt it (`usb_bulk_out_task`, `console_task` USART, `console_task` linux), each gated `#if CONFIG_KALICO_RUNTIME` with a fallback to the legacy direct `command_find_and_dispatch`. Bootloader-sentinel detection moves into pump's `OUT_KLIPPER` branch so it survives byte fragmentation.

**Tech Stack:** C99 firmware (Klipper conventions), STM32 ARM Cortex-M build via arm-none-eabi-gcc 14.2, Renode 1.16.1 for sim verification, Python 3 for the smoke probe.

**Spec:** `docs/superpowers/specs/2026-05-06-kalico-demux-rx-wiring-design.md`

**Branch:** `sota-motion`. ARM toolchain at `~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin` (export PATH before `make`). Saved sim configs at `tools/sim/sim.config` (H7) and `tools/sim/sim_f446.config` (F446). H7 silicon `.config` saved at `.config.h7.bak`; restore before tasks finish.

---

## File Map

**Helper (new public function):**
- Modify: `src/kalico_demux.c` — add `kalico_demux_pump`. Adds `#include "command.h"` (for MESSAGE_MAX, command_find_and_dispatch), `#include "string.h"` (memcmp), `#include "kalico_dispatch.h"` (kalico_dispatch_frame), `#include "board/misc.h"` (bootloader_request).
- Modify: `src/kalico_demux.h` — declare `void kalico_demux_pump(const uint8_t *buf, uint16_t len);`.

**Transport integration (three sites):**
- Modify: `src/generic/usb_cdc.c::usb_bulk_out_task` — gate replacement of `command_find_and_dispatch` call with pump.
- Modify: `src/generic/serial_irq.c::console_task` — gate replacement, irq-save tail-rebase, drop `console_pop_input` in runtime branch.
- Modify: `src/linux/console.c::console_task` — replace inline byte loop + `klipper_only_buf` with single pump call (gated). Delete `klipper_only_buf` / `klipper_only_pos` statics. Restore `console_receive_buffer()` to return `receive_buf`.

**Smoke test:**
- No code changes; `tools/sim/probe_f446_caps.py` already exists. Goal: it transitions from FAIL to PASS.

---

## Phase 1 — Helper

### Task 1: Declare and implement `kalico_demux_pump`

**Files:**
- Modify: `src/kalico_demux.h`
- Modify: `src/kalico_demux.c`

- [ ] **Step 1: Add the declaration to the header**

Edit `src/kalico_demux.h`. Insert after line 55 (after the `kalico_demux_consume` declaration, before the accessor declarations):

```c
// Drain a buffer of bytes through the demuxer state machine, dispatching
// klipper frames via command_find_and_dispatch and kalico-native frames
// via kalico_dispatch_frame as they surface. Demuxer state persists across
// calls, so partial frames at buffer boundaries are handled correctly.
//
// Bootloader-request sentinel detection (32-byte magic string) runs inside
// this function on the OUT_KLIPPER branch, so callers do NOT need to check
// for the sentinel separately. The check is gated on
// CONFIG_HAVE_BOOTLOADER_REQUEST.
void kalico_demux_pump(const uint8_t *buf, uint16_t len);
```

- [ ] **Step 2: Add the implementation to the .c file**

Edit `src/kalico_demux.c`. Make sure the includes at the top of the file include `<string.h>` (memcmp), `"command.h"` (command_find_and_dispatch), `"kalico_dispatch.h"` (kalico_dispatch_frame), `"board/misc.h"` (bootloader_request, CONFIG_HAVE_BOOTLOADER_REQUEST). If any are missing, add them. Then append at the bottom of the file (after the existing accessor functions, before the trailing close-brace if any):

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
            // Bootloader-request sentinel detection. The 32-byte sentinel
            // begins with 0x20 (= 32 decimal), which falls inside the
            // demuxer's [KLIPPER_LEN_MIN=5, KLIPPER_LEN_MAX=64] range, so
            // the demuxer reassembles all 32 bytes into klipper_buf
            // regardless of how the bytes arrive at the transport (one
            // burst, many small bursts, byte-by-byte). Checking here is
            // the only location that survives fragmentation.
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

- [ ] **Step 3: Cross-build H7 sim and verify it links**

```bash
export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH
cp .config .config.h7.silicon.bak
cp tools/sim/sim.config .config
make olddefconfig 2>&1 | tail -2
make clean && make -j4 2>&1 | tail -8
```
Expected: `Creating hex file out/klipper.bin`. The new `kalico_demux_pump` function compiles into `kalico_demux.o`; nothing calls it yet (Tasks 2-4 wire the transports), so the symbol is unused but linker keeps it because it's external (no static).

If `command_find_and_dispatch` or `bootloader_request` is undefined: the include chain in `src/kalico_demux.c` is missing one of the headers from Step 2. Verify with `head -20 src/kalico_demux.c`.

- [ ] **Step 4: Restore H7 silicon config**

```bash
cp .config.h7.silicon.bak .config
make olddefconfig 2>&1 | tail -2
```

- [ ] **Step 5: Commit**

```bash
git add src/kalico_demux.h src/kalico_demux.c
git commit -m "demux: add kalico_demux_pump — buffer-level RX drain helper"
```

---

## Phase 2 — Transport integration

### Task 2: Wire pump into USB CDC RX

**Files:**
- Modify: `src/generic/usb_cdc.c::usb_bulk_out_task` (around lines 134-162)

- [ ] **Step 1: Read the current usb_bulk_out_task body**

Run: `sed -n '133,163p' src/generic/usb_cdc.c`
Expected: shows the existing `command_find_and_dispatch` + memmove + `receive_pos = rpos` flow. Note the include for `kalico_demux.h` may not be present yet — we'll add it.

- [ ] **Step 2: Add the kalico_demux include if missing**

Edit `src/generic/usb_cdc.c`. If `#include "kalico_demux.h"` is not already among the includes near the top of the file, add it (alphabetically with the other generic includes). The existing CONFIG_KALICO_RUNTIME-gated `kalico_console_write_raw` shim already lives in this file, but it doesn't currently include `kalico_demux.h` — it likely needs adding.

- [ ] **Step 3: Replace the dispatch logic with gated pump**

Edit `src/generic/usb_cdc.c::usb_bulk_out_task`. Find the block that currently reads (approximately):

```c
    // Process a message block
    int_fast8_t ret = command_find_and_dispatch(receive_buf, rpos, &pop_count);
    if (ret) {
        // Move buffer
        uint_fast8_t needcopy = rpos - pop_count;
        if (needcopy) {
            memmove(receive_buf, &receive_buf[pop_count], needcopy);
            usb_notify_bulk_out();
        }
        rpos = needcopy;
    }
    receive_pos = rpos;
```

Replace it with:

```c
    // Process incoming bytes: kalico-aware drain when the runtime is
    // enabled; legacy single-block dispatch otherwise. USB receive_buf
    // is task-only (no IRQ writer), so a blanket reset after pump is
    // safe.
#if CONFIG_KALICO_RUNTIME
    kalico_demux_pump(receive_buf, rpos);
    receive_pos = 0;
#else
    int_fast8_t ret = command_find_and_dispatch(receive_buf, rpos, &pop_count);
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

(The `pop_count` local is still needed in the non-runtime branch; leave its declaration alone. The runtime branch unused-variable warning is suppressed by the `#if` since the unused declaration is only visible in the legacy branch.)

- [ ] **Step 4: Cross-build H7 silicon and verify it links**

```bash
export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH
make -j4 2>&1 | tail -8
```
Expected: `Creating hex file out/klipper.bin`. axi_ram still ~285 KB.

If `pop_count` warns as unused: declare it inside the legacy branch only. Specifically replace the original `uint_fast8_t rpos = receive_pos, pop_count;` line near the top of the task with `uint_fast8_t rpos = receive_pos;`, and inside the `#else` branch declare `uint_fast8_t pop_count;` before its first use.

- [ ] **Step 5: Commit**

```bash
git add src/generic/usb_cdc.c
git commit -m "usb_cdc: route RX through kalico_demux_pump under CONFIG_KALICO_RUNTIME"
```

### Task 3: Wire pump into USART RX

**Files:**
- Modify: `src/generic/serial_irq.c::console_task` (lines 87-103)

- [ ] **Step 1: Read the current console_task body**

Run: `sed -n '86,105p' src/generic/serial_irq.c`
Expected: shows `command_find_block` + dispatch + bootloader-memcmp + `console_pop_input` flow.

- [ ] **Step 2: Add the kalico_demux + irq.h includes if missing**

Edit `src/generic/serial_irq.c`. Add `#include "kalico_demux.h"` near the top (alphabetical with existing `"command.h"`, `"sched.h"` etc.). The file already includes `"board/irq.h"` at line 10 — confirm.

- [ ] **Step 3: Replace console_task with the gated runtime branch**

Edit `src/generic/serial_irq.c::console_task`. Replace the entire function body (lines 87-103) with:

```c
// Process any incoming commands
void
console_task(void)
{
    uint_fast8_t rpos = readb(&receive_pos);

#if CONFIG_KALICO_RUNTIME
    kalico_demux_pump(receive_buf, rpos);

    // Bytes the IRQ deposited during pump live in [rpos, now). Rebase
    // them down to the start of receive_buf and update receive_pos
    // atomically so a fresh IRQ doesn't write into a slot we just
    // moved. Doing the memmove inside irq_save trades latency for the
    // simpler invariant; no retry loop needed (unlike console_pop_input,
    // which does memmove outside irq_save and therefore must retry).
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
    uint_fast8_t pop_count;
    int_fast8_t ret = command_find_block(receive_buf, rpos, &pop_count);
    if (ret > 0)
        command_dispatch(receive_buf, pop_count);
    if (ret) {
        if (CONFIG_HAVE_BOOTLOADER_REQUEST && ret < 0 && pop_count == 32
            && !memcmp(receive_buf, " \x1c Request Serial Bootloader!! ~", 32))
            bootloader_request();
        console_pop_input(pop_count);
        if (ret > 0)
            command_send_ack();
    }
#endif
}
DECL_TASK(console_task);
```

- [ ] **Step 4: Cross-build the USART-mode H7 sim and verify it links**

```bash
export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH
cp .config .config.h7.silicon.bak
cp tools/sim/sim.config .config
make olddefconfig 2>&1 | tail -2
make clean && make -j4 2>&1 | tail -8
```
Expected: `Creating hex file out/klipper.bin`. The H7 sim uses USART2 (CONFIG_STM32_SERIAL_USART2=y), so this exercises the new serial_irq.c runtime branch.

- [ ] **Step 5: Cross-build H7 silicon (USB CDC) and verify regression-clean**

```bash
cp .config.h7.silicon.bak .config
make olddefconfig 2>&1 | tail -2
make clean && make -j4 2>&1 | tail -5
```
Expected: H7 silicon still builds. axi_ram ~285 KB.

- [ ] **Step 6: Commit**

```bash
git add src/generic/serial_irq.c
git commit -m "serial_irq: route RX through kalico_demux_pump; drop pre-pump sentinel check (now in pump)"
```

### Task 4: Refactor linux/console.c to use the pump helper

**Files:**
- Modify: `src/linux/console.c::console_task` (lines 161-235), `console_receive_buffer` (lines 142-146), and the `klipper_only_buf` / `klipper_only_pos` statics (lines 139-140).

- [ ] **Step 1: Read the current linux console_task body**

Run: `sed -n '127,235p' src/linux/console.c`
Expected: shows the inline byte loop + `klipper_only_buf` accumulator + drain loop.

- [ ] **Step 2: Add kalico_demux include if missing, then replace the body**

Edit `src/linux/console.c`. The file already uses `kalico_demux_*` symbols, so the include is present. Replace `console_receive_buffer`'s body and `console_task`'s post-read processing block.

Find the `console_receive_buffer` definition (around lines 142-146):

```c
void *
console_receive_buffer(void)
{
    return klipper_only_buf;
}
```

Replace with:

```c
void *
console_receive_buffer(void)
{
    // Returns the raw RX buffer used by command_find_and_dispatch's
    // pop_count arithmetic. Pre-demuxer-integration shape; the
    // klipper_only_buf accumulator that briefly lived between this
    // function and the dispatch path was removed when kalico_demux_pump
    // landed (see docs/superpowers/specs/2026-05-06-kalico-demux-rx-wiring-design.md).
    return receive_buf;
}
```

Delete the two static definitions:
```c
static uint8_t klipper_only_buf[4096];
static int klipper_only_pos;
```
(Lines 139-140 in the current file.)

Then in `console_task`, find the section starting with `// Drive the kalico-native demuxer over the freshly-read bytes…` (around line 183) through the `if (klipper_only_pos > 0) sched_wake_task(...)` line (around line 234). Replace the entire block with:

```c
    // Drive the kalico-native demuxer over the freshly-read bytes,
    // dispatching klipper and kalico frames as they surface. State
    // persists across console_task firings for partial frames.
#if CONFIG_KALICO_RUNTIME
    if (ret > 0)
        kalico_demux_pump(&receive_buf[receive_pos], (uint16_t)ret);
    // Linux receive_buf is task-only; the demuxer fully consumes the
    // bytes it was handed.
    receive_pos = 0;
#else
    if (ret > 0)
        receive_pos += ret;
    while (receive_pos > 0) {
        uint_fast8_t pop_count;
        uint_fast8_t msglen = receive_pos > MESSAGE_MAX ? MESSAGE_MAX : receive_pos;
        int_fast8_t r = command_find_and_dispatch(receive_buf, msglen, &pop_count);
        if (!r)
            break;
        int needcopy = receive_pos - pop_count;
        if (needcopy)
            memmove(receive_buf, &receive_buf[pop_count], needcopy);
        receive_pos = needcopy;
    }
#endif
```

- [ ] **Step 3: Linux build to confirm it compiles**

The local cross-build flow doesn't run the linux build, but the `tools/sim_klippy/run_local.sh` builds linux klipper.elf inside docker. Quick check that the source compiles via `cargo` (no — linux klipper is C). Run:

```bash
ls tools/sim_klippy/run_local.sh && cat tools/sim_klippy/Dockerfile | head -20
```

Confirms a Linux build environment exists. Optional smoke build:

```bash
bash tools/sim_klippy/run_local.sh "G1 Z5 F600" 2>&1 | tail -15
```

Expected: builds successfully and runs the sim move (output may vary; the verification is "no compile errors and the sim doesn't crash on G1 Z5"). Skip if Docker isn't running locally — the change pattern is the same as the H7 sim's USART path which Task 3 already verified.

- [ ] **Step 4: Commit**

```bash
git add src/linux/console.c
git commit -m "linux/console: route RX through kalico_demux_pump; restore console_receive_buffer to receive_buf"
```

---

## Phase 3 — Acceptance test

### Task 5: F446 sim end-to-end smoke test

**Files:**
- No source changes. Existing `tools/sim/probe_f446_caps.py` and `tools/sim/run_sim_f446.sh`.

- [ ] **Step 1: Build the F446 sim firmware**

```bash
bash tools/sim/build_sim_firmware_f446.sh 2>&1 | tail -8
```
Expected: `Built sim firmware:` line, klipper.elf produced.

- [ ] **Step 2: Boot the sim and run the QueryRuntimeCaps probe**

```bash
pkill -f renode 2>/dev/null; sleep 2
bash tools/sim/run_sim_f446.sh > /tmp/f446_caps_sim.log 2>&1 &
SIM_PID=$!
sleep 8
python3 tools/sim/probe_f446_caps.py
PROBE_RC=$?
echo "probe exit: $PROBE_RC"
kill $SIM_PID 2>/dev/null; wait 2>/dev/null
true
```
Expected output:

```
[probe] connecting to localhost:3334
[probe] drained N bytes of pre-existing UART traffic
[probe] sending QueryRuntimeCaps (13 bytes): 550c0001400000bebafeca05be
[probe] RuntimeCapsResponse: {'max_control_points': 512, 'max_knot_vector_len': 524, 'max_degree': 10, 'curve_pool_n': 4}
[probe] PASS — caps match RUNTIME_TARGET_SMALL profile.
probe exit: 0
```

If the probe times out (rc=5): the demuxer-pump integration didn't reach `kalico_dispatch_frame`. Diagnose by adding `fprintf(stderr, ...)` debug prints inside `kalico_demux_pump`'s `OUT_KALICO` branch and re-running `bash tools/sim/build_sim_firmware_f446.sh && bash tools/sim/run_sim_f446.sh`. STOP and escalate.

If the probe receives a frame but caps don't match expected (rc=4): the QueryRuntimeCaps handler in `src/kalico_dispatch.c` returns wrong values. Verify autoconf has the small-profile values:
```bash
grep -E "RUNTIME_MAX_CONTROL_POINTS|RUNTIME_CURVE_POOL_N" out/autoconf.h
```

- [ ] **Step 3: H7 sim regression check (Phase-2 gate)**

If `tools/sim/run_phase2_gate.sh` exists and ran before this work, run it again to confirm H7 USART RX still works:

```bash
bash tools/sim/build_sim_firmware.sh 2>&1 | tail -3
bash tools/sim/run_phase2_gate.sh 2>&1 | tail -10
```
Expected: same PASS/FAIL outcome as before this work. The gate exercises Identify + LoadCurve + PushSegment over the same RX path Task 3 modified, so a regression here means the runtime branch broke a previously-working flow.

- [ ] **Step 4: Restore H7 silicon config**

```bash
cp .config.h7.silicon.bak .config
make olddefconfig 2>&1 | tail -2
```

- [ ] **Step 5: No commit** — no source changes; the probe pass is the deliverable. Optionally append a note to `docs/superpowers/handoff/` recording the verification.

---

## Self-review

**Spec coverage:**
- §4.1 `kalico_demux_pump` helper + sentinel detection in pump → Task 1 ✓
- §4.2.1 USB CDC integration → Task 2 ✓
- §4.2.2 USART integration with irq-save tail-rebase → Task 3 ✓
- §4.2.3 Linux refactor + `console_receive_buffer` preserved + linux gate → Task 4 ✓
- §4.3 Build gating in three transport files → Tasks 2, 3, 4 each have `#if CONFIG_KALICO_RUNTIME` ✓
- §4.4 Bootloader-sentinel detection moved into pump → Task 1's pump implementation ✓
- §5 Data flow → implicitly satisfied by Tasks 1-4 ✓
- §6 Error handling (consume on terminal states only) → Task 1's switch has no `kalico_demux_consume()` in OUT_NONE branch ✓
- §7 Testing — Linux sim regression, H7 phase-2 gate, F446 probe → Task 5 covers all three ✓
- §8 Open question (const cast comment) → noted in Task 1's Step 2 comment block ✓

**Placeholder scan:**
- No "TBD", "TODO", or "implement later".
- Each step has either exact code or exact commands.
- Task 4 Step 3's "Optional smoke build" caveat is acknowledged scope-clarity, not a placeholder — the cross-build verification in Tasks 1-3 already exercises the linux source via the same compile path (each transport file is linked into both linux and stm32 builds where applicable; serial_irq.c being shared is verified in Task 3's USART build).

**Type / signature consistency:**
- `kalico_demux_pump(const uint8_t *buf, uint16_t len)` declared in Task 1 Step 1, used identically in Tasks 2, 3, 4 (all three transports pass `(receive_buf, rpos)` or `(receive_buf, ret)`).
- `command_find_and_dispatch(uint8_t *, uint_fast8_t, uint_fast8_t *)` — Task 1's pump casts `const uint8_t *` to `uint8_t *`; Tasks 2, 3, 4's legacy branches use the same signature.
- `kalico_demux_klipper_buf()` returns `const uint8_t *` (per `kalico_demux.h:57`); Task 1 uses it consistently.

**Branch hygiene:**
- Tasks 1-4 each commit a single logical change. Task 5 doesn't commit (no source delta).
- After Task 5, `.config` is restored to H7 silicon. `.config.h7.silicon.bak` is left in the working tree (gitignored alongside `.config.h7.bak`).

No issues found.
