# Kalico-native transport — design

**Status:** rev 3. Substantially rewritten after Codex's rev 2 review caught architectural errors (sync-byte demux direction wrong, schema-hash circular, USB CDC work understated, pre-shape splitter unnecessary, scope creep). MVP-scoped: covers LoadCurve / PushSegment / minimal events / reset epoch. Telemetry, UART ARQ, EtherCAT, and Step 11+ concerns are notes-only.
**Date:** 2026-05-04.
**Step:** 7-D Phase 4 (bridge-mode first print).
**Supersedes:** [`2026-05-04-incremental-curve-upload-design.md`](2026-05-04-incremental-curve-upload-design.md).

---

## 1. Why we're rewriting the comm layer

Today's wire is Klipper's msgproto, capped at `MESSAGE_MAX = 64` bytes per frame. One post-shape NURBS curve for one axis is 1-8 KB. We were chunking the wire to escape this; the chunking spec we landed turned out to be a workaround that we'd keep paying for at every subsequent feature (phase stepping, telemetry, EtherCAT). Better to escape the constraint than chunk forever.

This codebase is *our fork* of Kalico (which itself is a fork of Klipper-mainline). The "Klipper-stock compatibility" cost is therefore the narrowest blast radius — only this fork needs to agree on framing. Klipper-mainline merges and Kalico-upstream merges are separately considered and not a hard blocker for protocol changes.

## 2. Constraints

1. **Print throughput is non-negotiable** (`CLAUDE.md`). No algorithmic compromise for transport simplicity.
2. **MCU receives shape pre-baked.** Wire still carries post-shape NURBS.
3. **Real-time, no queue offload.** Each shaped segment dispatches in real time.
4. **Rust links as staticlib into the MCU build.** kalico-runtime stays linked. The new transport is additive C+Rust code in our fork; klippy keeps using msgproto for its commands (config, sensors, legacy stepper control).
5. **Klippy's commands stay on Klipper's wire format.** Klippy is unchanged. The new transport coexists with klippy's traffic on the same physical USB-CDC interface, demultiplexed at the byte-stream layer.
6. **Multi-MCU, per-MCU pool sizing.** Octopus Pro (H723, 564 KB SRAM) does heavy X+Y shaping; F446 (128 KB SRAM) does light Z work. Pool slot size is per-MCU, sized to the MCU's actual workload.
7. **USB-CDC today; everything else later.** UART ARQ, EtherCAT, etc. are out of scope for this spec.

## 3. Decision summary

- **Stream-level frame-boundary demux** on the existing single USB-CDC interface. Klipper frames (length-prefixed, end with sync 0x7E) and kalico frames (sync 0x55 + length prefix) share the byte stream; a demux state machine routes complete frames to the appropriate parser. See §6.
- **Variable-size, length-prefixed kalico frames.** No 64-byte cap. One LoadCurve = one frame.
- **Static, build-time schema** with **bootstrap ABI for Identify**. The Identify command and IdentifyResponse have a fixed byte layout outside the schema; they're decoded before any schema-validated traffic. Identify carries a SHA-256 schema hash; mismatch is a hard refuse-to-print.
- **Reset-epoch tracking.** IdentifyResponse and StatusEvent carry a u32 `reset_epoch` unique per MCU boot. Mismatch from last-known forces motion-bridge to invalidate kalico runtime state.
- **Per-MCU pool sizing**, not trajectory-layer splitting. H723's pool slot fits the worst-case degree-9 curve from a long printing move (~8 KB per slot, ~16 slots). F446's pool fits its actual Z workload (~200 B per slot). Curves bigger than the pool slot are a config-time error, not a runtime concern.
- **MVP message catalog** is small: Identify/IdentifyResponse, LoadCurve/LoadCurveResponse, PushSegment/PushSegmentResponse, StatusEvent, CreditFreed, FaultEvent. Nine messages. Everything else (telemetry, endstop, SetHomed, QueryStatus, etc.) stays on Klipper's wire for now.

The chunking spec retires; the B.1 chunker stays retired (the §3 per-MCU pool sizing handles overflow without it).

## 4. Frame format

```text
Offset  Field            Type     Notes
0       sync             u8       0x55 — distinct from Klipper's 0x7E so the
                                  demuxer can route at frame boundaries
1       len              u16_le   total frame length excluding sync byte
                                  (header through CRC, inclusive). u16
                                  caps frame at ~64 KB, ample for our needs.
3       channel          u8       0=control, 1=events
4       payload          bytes    Layer 4 message (type + version +
                                  correlation_id + body)
len-1   crc              u16_le   CRC-16/CCITT over [len .. crc-start]
```

Total framing overhead: **8 bytes** (sync + len + channel + crc).

Why u16 length (not u32 like rev 2): a LoadCurve at production worst case is ≤8 KB. u16 caps frames at 64 KB, plenty of headroom, two fewer bytes per frame.

Why CRC even on USB-CDC: defends against host/MCU buffer-corruption bugs upstream of the link. Negligible cost.

## 5. Bootstrap ABI (Identify)

The bootstrap problem: every other message in the schema is decoded using the schema, which the host needs to validate via `schema_hash`. But `schema_hash` lives in IdentifyResponse, which is itself a schema-encoded message. Circular.

Fix: **Identify and IdentifyResponse have a fixed byte layout, frozen at protocol version 1, never changes.** They're decoded by hand-written readers/writers that don't consult the runtime schema.

```text
Identify (host → MCU)
Offset  Field           Type
0       proto_version   u8       must be 0x01

IdentifyResponse (MCU → host)
Offset  Field           Type
0       proto_version   u8       must be 0x01
1       firmware_ver    u32_le   build version
5       build_hash      [u8; 20] git commit SHA-1 (informational)
25      schema_hash     [u8; 32] SHA-256 over canonicalized schema definition
57      reset_epoch     u32_le   nonzero, unique per MCU boot
61      capabilities    u64_le   bitmap (phase_stepping=0x1, ...; future use)
69      mcu_serial      [u8; 12] chip serial (informational)
```

Both messages still ride the framing layer (sync + len + channel + crc). They use type tag `0x0001` (Identify) and `0x0002` (IdentifyResponse) on the control channel. The schema layer treats these two type tags as opaque (do not decode them via the dynamic schema decoder). The bootstrap reader is a few hand-written lines.

Once the host receives IdentifyResponse, validates `proto_version == 0x01`, and verifies `schema_hash` matches its own build's hash, schema-validated traffic begins. On mismatch: refuse to dispatch any motion command; surface the host build hash + MCU build hash to the user; log the firmware/host versions. This is a hard fault, not a degraded mode.

`reset_epoch` is generated on each MCU boot. Implementation: read the MCU's TRNG (every STM32 we target has one — H723 and F446 both ship `RNG`), discard the result if zero, store in a static. Linux sim uses `/dev/urandom`. Doesn't need to be monotonic across reboots; uniqueness suffices.

## 6. Stream-level demux

Klipper frames have **no leading sync byte**. They start with the length byte (5..64), then seq, payload, CRC, then trailing sync 0x7E. So Klipper's `command_find_block` (`src/command.c:268`) reads byte 0 as length; on length out of range, it scans forward to the next 0x7E and resyncs.

This means kalico frames cannot be on the same byte stream without a demuxer that runs *before* Klipper's parser. Otherwise: a kalico frame's leading 0x55 (85) overflows Klipper's MESSAGE_MAX=64 length check, and Klipper resyncs by scanning forward through the kalico frame, corrupting it.

The demux state machine, run on every incoming byte before `command_find_block` sees it:

```text
state := WAITING_FOR_FRAME

loop:
  match state:
    WAITING_FOR_FRAME:
      peek next byte:
        5..64  → state = INSIDE_KLIPPER_FRAME(remaining = byte_value)
        0x55   → state = INSIDE_KALICO_FRAME(needs_header = true)
        0x7E   → consume; stay in WAITING_FOR_FRAME (stray inter-frame sync,
                 same as Klipper's existing tolerance for leading 0x7Es)
        else   → consume + log; stay in WAITING_FOR_FRAME

    INSIDE_KLIPPER_FRAME(remaining):
      forward byte to klipper_buf; remaining -= 1
      when remaining == 0:
        invoke Klipper's command_find_and_dispatch on klipper_buf
        state = WAITING_FOR_FRAME

    INSIDE_KALICO_FRAME(needs_header):
      accumulate bytes into kalico_buf
      after first 3 bytes (sync + len): parse len; needs_header = false
      after `len` bytes total: invoke kalico_dispatch on kalico_buf
      state = WAITING_FOR_FRAME
```

Byte values inside kalico payloads (which can include 0x7E) are safe because once the demuxer commits to `INSIDE_KALICO_FRAME`, it consumes exactly `len` bytes regardless of payload content. Same on the Klipper side: once committed to `INSIDE_KLIPPER_FRAME`, exactly `length` bytes are consumed.

**Required firmware changes** (the work Codex flagged):

- `src/generic/usb_cdc.c` `receive_buf[128]` → `receive_buf[N]` where N fits the largest kalico frame plus a Klipper frame plus headroom. Sized to ~8 KB.
- New file (e.g. `src/kalico_demux.c`) implementing the state machine. Inserted into `usb_cdc.c`'s RX path, replacing the direct call to `command_find_and_dispatch`.
- `src/linux/console.c` similar — `receive_buf[4096]` is already big enough but the dispatch path also needs the demuxer.
- Mirror demuxer on the host side in `kalico-host-rt`.

This is real, bounded firmware work. Not "a constant change."

## 7. Schema (Layer 4)

### 7.1 Message catalog (MVP)

| Type   | Kind                       | Channel  | Notes                                              |
|--------|----------------------------|----------|----------------------------------------------------|
| 0x0001 | `Identify` (cmd)           | control  | Bootstrap ABI per §5 — fixed layout                |
| 0x0002 | `IdentifyResponse` (rsp)   | control  | Bootstrap ABI per §5 — fixed layout                |
| 0x0010 | `LoadCurve` (cmd)          | control  | Upload one per-axis NURBS to a curve-pool slot     |
| 0x0011 | `LoadCurveResponse` (rsp)  | control  | Result code + packed handle                        |
| 0x0020 | `PushSegment` (cmd)        | control  | Schedule a shaped segment for execution            |
| 0x0021 | `PushSegmentResponse` (rsp)| control  | Result + accepted_segment_id + credit_epoch        |
| 0x0080 | `StatusEvent` (evt)        | events   | Periodic snapshot; carries `reset_epoch`           |
| 0x0081 | `CreditFreed` (evt)        | events   | Pool slot retirement notification                  |
| 0x0082 | `FaultEvent` (evt)         | events   | MCU fault notification                             |

Type tag `0x0001`/`0x0002` are bootstrap. Tags `0x0010..0x007F` are control-channel commands/responses. Tags `0x0080..0x00BF` are events.

### 7.2 Per-message header (Layer 4)

Inside the frame envelope's payload field:

```text
Offset  Field            Type     Notes
0       type             u16_le   MessageKind discriminant
2       version          u8       per-message schema version (start at 0x01)
3       correlation_id   u32_le   nonzero round-trip identifier on
                                  command/response pairs; 0 on events
7       body             bytes    type-specific payload
```

Per-message overhead: **7 bytes**.

### 7.3 LoadCurve body

```text
Offset  Field           Type        Notes
0       slot            u16_le      curve-pool slot index
2       degree          u8          NURBS polynomial degree
3       n_cps           u32_le      cps array length
7       n_knots         u32_le      knots array length
11      cps             n_cps × f32 control points, little-endian
…       knots           n_knots × f32 knots, little-endian
```

For an H723-pool-worst-case curve (degree 9, ~200 pieces, ~1810 cps + ~1820 knots): body is ~14.5 KB; total frame is ~14.5 KB. Fits one frame.

### 7.4 Other message bodies

- `LoadCurveResponse`: `result: i32`, `curve_handle_packed: u32`. 8 bytes.
- `PushSegment`: id (u32), four handles (4×u32), t_start (u64), t_end (u64), kinematics (u8), e_mode (u8), extrusion_ratio (f32). ~36 bytes.
- `PushSegmentResponse`: result (i32), accepted_segment_id (u32), credit_epoch (u32). 12 bytes.
- `StatusEvent`: engine_status (u8), queue_depth (u8), current_segment_id (u32), last_fault (i32), fault_detail (u32), reset_epoch (u32). 18 bytes.
- `CreditFreed`: retired_through_segment_id (u32), free_slots (u8). 5 bytes.
- `FaultEvent`: fault_code (u16), fault_detail (u32), segment_id (u32). 10 bytes.

All fit one frame trivially.

### 7.5 Schema definition + canonical hash

The Rust source-of-truth is a new crate `rust/kalico-protocol`. Hand-written Encode/Decode per message; ~300 LoC for the MVP set.

`schema_hash` is SHA-256 over a canonicalized text form of the schema: each message's type tag, channel, version, and field list (name + type) in declaration order, separated by `\n`. Build-time computation; const in the Rust crate; codegen'd into a C header for the MCU side. Both sides compute the hash from the same canonicalization rules.

## 8. Channels and flow control

Two channels in MVP scope: **control** (command + response) and **events** (MCU → host fire-and-forget). Telemetry channel is deferred — it's specifically for high-rate skip-detection / accelerometer streams that don't exist yet (Step 11). Adding a third channel later is a one-byte enum extension.

### 8.1 Backpressure

Control channel is request/response with explicit correlation IDs. Backpressure is natural: host doesn't issue a new LoadCurve until the previous one's response arrives (or its timeout expires). MCU dispatch handlers run in foreground command-dispatch context (same as today's `command_kalico_*`). Handlers must be bounded — no blocking on resources that require further messages to free.

For the MVP, **all handlers are guaranteed bounded:**
- `Identify`: hand-written response build.
- `LoadCurve`: bounds-check + memcpy into pool slot + call `kalico_runtime_load_curve`. Tens of microseconds.
- `PushSegment`: validate handles + enqueue into the runtime's segment queue. Microseconds.

This sidesteps the "long handler blocks reserved-band" deadlock Codex flagged. Reserved-band complexity is unnecessary at MVP scope.

### 8.2 Events channel

MCU emits StatusEvent at 100 Hz; CreditFreed when a pool slot retires (rate-bounded by segment-finish rate); FaultEvent when something faults (rare). Host-side has a dedicated event-receive ring sized to absorb 1 second of events without backpressure (~256 events). Host application code drains the ring promptly; if the ring overflows, oldest events are dropped (telemetry-style — see open question below).

No credit-window flow control at MVP. Add when telemetry channel arrives in Step 11.

## 9. Reset-epoch state machine

Two states: `Identified` (host knows the MCU's `reset_epoch`) and `Unidentified` (host has not yet validated, or MCU rebooted).

Transitions:

```text
[Disconnected] → on Layer 0 connection up → [Unidentified]
[Unidentified] → send Identify; await IdentifyResponse → if proto_version,
                 schema_hash both match → [Identified, epoch = ⟨received⟩]
                                         → else → [Faulted (refuse-to-print)]
[Identified, epoch = E] → on incoming StatusEvent.reset_epoch == E → no-op
[Identified, epoch = E] → on incoming StatusEvent.reset_epoch != E → atomic
                          transition to [Unidentified]:
                            1. stop new sends
                            2. drop all in-flight correlation IDs (return
                               TransportError::Reset to callers)
                            3. discard kalico_buf RX bytes that haven't been
                               dispatched yet
                            4. invalidate motion-bridge slot pool view (all
                               curve_handles are stale; bridge re-allocates
                               from slot 0)
                            5. clear segment-id counter and dispatched-segment
                               state
                          then re-issue Identify
[Identified] → on Layer 0 connection drop → [Disconnected]
```

The atomic transition is the key correctness property: between (1) and (5) the host must hold its motion-dispatch lock and refuse to dispatch new work. After (5) succeeds, motion-dispatch resumes from a clean state. If a print was in progress, surface a fault to klippy (the print is unrecoverable).

`StatusEvent` is the liveness signal. At 100 Hz the host detects reset within 10 ms typical, ≤100 ms worst case (allowing for one missed event). Phase 4 step-pulse path includes StatusEvent migration (Phase C) so reset detection works from day one.

## 10. Per-MCU pool sizing

H723 (X + Y, heavy shaping):
- Pool slot scratch sized for the realistic worst case of a long printing move: degree 9, ~200 pieces (Adaptive grid `target_grid_spacing_mm: 0.5` over a 100mm move). cps ~1810, knots ~1820. Slot scratch = ~14.5 KB.
- `CURVE_POOL_N = 16` (sufficient for the planner's look-ahead depth; reduced from today's 64 because slots are now larger).
- Pool RAM: 16 × 14.5 KB = ~232 KB. Fits in H723's 564 KB SRAM with significant headroom.

F446 (Z, light/no shaping):
- Z is rarely shaped; default `shaper_type_z` is passthrough. **Typical** Z curve is a degree-4 trapezoidal-ish trajectory with <20 pieces. Slot scratch = ~1 KB. `CURVE_POOL_N = 16`. Pool RAM: 16 × 1 KB = 16 KB. Easy fit on F446's 128 KB.
- **Edge case acknowledged but not designed for in this spec:** a long G0 Z travel (e.g., 200mm bed-down at full Z speed) produces a similar piece-count blowup to a long X+Y move because the trajectory pipeline still runs TOPP-RA grid + Hermite refit on it, even without shaping convolution. The clean fix is a straight-line specialization in the trajectory layer (recognize G0/G1 linear moves, emit a simpler representation that doesn't blow up at the grid). That's a future optimization. For now, F446 pool slots may need to grow proportionally to typical Z travel length, or the user accepts a config-time error on pathological Z configs.

The previous "100 cps + 111 knots" sizing was based on an empirical 64-cps measurement plus 25% margin from the Step 7-B baseline. That was sized for a single trivial 50mm move with moderate shaper frequencies, **not** for a long printing move at full Adaptive grid resolution. Rev 3 corrects this.

If a curve genuinely exceeds the per-MCU pool ceiling (pathological config — e.g., heavy shaping enabled on Z, or `target_grid_spacing_mm: 0.05`), the host detects at LoadCurve build time, surfaces a clear error ("trajectory produced N-piece curve, MCU pool fits M; raise pool size or reduce grid resolution"), and aborts the print. This is a config error, not a runtime failure mode.

No trajectory-layer splitter required. The earlier "split when overflow" idea retires entirely — we size the pool to actually fit the workload.

## 11. Coexistence with Klipper

One physical USB-CDC interface per MCU. Both Klipper-protocol traffic (klippy's commands) and kalico-native traffic share the byte stream, demuxed per §6. From klippy's perspective nothing changes — it talks to motion-bridge through the existing PyO3 bridge, motion-bridge owns the wire and routes bytes internally.

When klippy issues a firmware restart, the MCU reboots and generates a new `reset_epoch`. The next StatusEvent surfaces the change to motion-bridge, which invalidates kalico runtime state per §9. Klippy's own state machine handles its protocol's reset path independently. The two recover in parallel.

## 12. Host-side architecture

New crate `rust/kalico-native-transport` implementing:
- Layer 1 framer (kalico side of the demux state machine; the existing Klipper-side parser stays unchanged).
- Layer 4 schema decode/encode (delegating to `kalico-protocol` crate).
- A `Transport` trait impl that mirrors today's `kalico-host-rt::Transport` interface (`call`, `send`, `subscribe_events`).

`producer::load_curve` and `producer::push_segment` rewrite to use the new transport: single `call_typed` per logical operation, no chunking.

motion-bridge's dispatch closure swaps the underlying transport from `kalico-host-rt` to `kalico-native-transport` for kalico-native traffic. Klippy passthrough continues to use `kalico-host-rt` for Klipper-protocol traffic.

## 13. MCU-side architecture

`src/kalico_demux.c` — state machine per §6, inserted into `src/generic/usb_cdc.c`'s RX path.

`src/kalico_dispatch.c` — Layer 4 dispatcher invoked when the demuxer completes a kalico frame. Looks up the message handler by type tag, decodes the body via the generated schema header, and calls the handler. Bootstrap path (Identify) bypasses the schema decoder.

`rust/kalico-runtime/src/transport_handlers.rs` — Rust handlers replacing today's `command_kalico_*` C functions. Mirror the existing handlers' behavior (load_curve, push_segment) but invoked via the new dispatcher.

`src/kalico_protocol_schema.h` — codegen output from `rust/kalico-protocol`. Defines type-tag constants and the schema_hash constant; the C-side handler dispatch references these.

## 14. What this spec retires from earlier work

**Retired:**
- `2026-05-04-incremental-curve-upload-design.md` — superseded.
- `command_kalico_load_curve_begin/chunk/finalize` (commit `665d98d59`).
- `producer::load_curve`'s begin/chunk/finalize sequence (commit `b6a756a19`).
- `command_kalico_push_segment` on Klipper protocol — moves to native transport in Phase C.

**Kept:**
- B.1 chunker retirement (commit `12c79e904`). Stays retired.
- cbindgen header sync (commit `259fdc2a7`).
- §6.0 backpressure-respecting `dispatch_fire_and_forget` and `send_typed` (commit `ec61d968d`). Useful for Klippy passthrough on Klipper-protocol path.

**Re-decided:**
- Pool sizing (commit `02dc605db` bumped 80/91 → 100/111). Rev 3 supersedes with per-MCU sizing per §10. The bump commit's constants will be revised again as part of Phase C.

## 15. Migration plan

### Phase A — host-side foundation

- Create `rust/kalico-protocol` crate: MessageKind enum, hand-written Encode/Decode, schema_hash computation, build-time C header gen.
- Create `rust/kalico-native-transport` crate: framer, schema decode, Transport trait impl.
- Unit tests: framer round-trip, schema_hash determinism, bootstrap-ABI hand-decode, stream-level demux against adversarial byte streams (Klipper bytes interleaved with kalico bytes).

Gate: `cargo test -p kalico-protocol -p kalico-native-transport` passes.

### Phase B — MCU-side foundation

- `src/kalico_demux.c` state machine.
- `src/kalico_dispatch.c` schema dispatcher.
- `kalico-runtime/src/transport_handlers.rs` Rust handlers.
- `src/kalico_protocol_schema.h` codegen output.
- Patch `src/generic/usb_cdc.c` to use the demuxer; bump `receive_buf` size.
- Patch `src/linux/console.c` similarly.
- TRNG-based reset_epoch generation at boot.

Gate: sim handshake — bridge connects, sends Identify, gets IdentifyResponse, validates schema_hash + reset_epoch.

### Phase C — Phase 4 unblock

Per-MCU pool sizing per §10 (constants in `rust/runtime/src/curve_pool.rs` and `src/runtime_tick.c`).

Migrate the step-pulse path:
- LoadCurve cmd + LoadCurveResponse + handler.
- PushSegment cmd + PushSegmentResponse + handler.
- CreditFreed event.
- FaultEvent event.
- StatusEvent event (so reset detection works).
- `producer::load_curve` and `producer::push_segment` rewrite.
- Bridge dispatch closure switches to native transport.
- Remove `command_kalico_load_curve_*` and `command_kalico_push_segment` from runtime_tick.c.

Gate: `python3 tools/sim_klippy/test_phase4_steps.py` shows non-zero step pulses on sim.

### Phase D — hardware bringup

- Build for H723 + F446.
- Verify demux on real USB-CDC.
- Physical first print on Octopus Pro.

## 16. Validation

Per-phase gates above. Pre-existing regression coverage:
- Klipper-protocol traffic (everything that's NOT kalico-specific) must keep working unchanged. Klippy-side tests stay green.
- Step 7-C-IO test battery (A1–A8) covers `kalico-host-rt`'s reactor for Klipper-protocol traffic; preserved.

New test coverage:
- Demuxer state machine: adversarial byte streams (kalico mid-Klipper, Klipper mid-kalico, malformed lengths, partial frames at buffer boundaries).
- Schema_hash mismatch handling on the host (refuses motion dispatch).
- Reset-epoch state-machine atomic transition (no in-flight messages survive).
- Per-MCU pool sizing fits the empirical worst case (test fixture: 100mm move at production grid).

## 17. Open questions

1. **Demuxer placement on the host side.** Today klippy talks to motion-bridge; motion-bridge owns the wire via `kalico-host-rt`. The host-side demux either (a) lives inside `kalico-host-rt` (which currently only knows Klipper protocol) and forwards Klipper-frame bytes to klippy via passthrough, or (b) lives as a new layer above both. (a) is simpler; lean (a).
2. **Events ring overflow policy.** §8.2 says "drop oldest" if the host's events ring overflows. Is that the right call, or should we block the MCU's events emission? Lean drop-oldest because StatusEvent is periodic and missing one is fine; reset-epoch detection survives via the *next* event.
3. **F446 transport scope.** §10 sizes its pool for Z's actual workload; does it also need StatusEvent / CreditFreed on the events channel? Yes — F446 still has segments and a runtime; we shouldn't bifurcate the transport per MCU class.
4. **Heartbeat / liveness check.** Today there's no native heartbeat; StatusEvent at 100 Hz doubles as one. If StatusEvent stops, host detects after a timeout (~500 ms) and faults the print. Adequate for MVP; revisit if Step 11's telemetry channel changes the liveness story.

## 18. What this spec is *not* doing

- **Telemetry channel.** Defer to Step 11 when actual high-rate streams (skip detection, accelerometer) need it. Adding the channel is a one-byte enum extension and a new Layer 3 path; not architecture-defining.
- **UART transport.** Deferred until UART hardware exists in our config.
- **EtherCAT.** Per the prior research memo, EtherCAT requires a host-side curve evaluator and a different wire schema (cyclic CSP setpoints, not curve uploads). The MCU-evaluator architecture this spec serves does not survive EtherCAT migration. EtherCAT is a separate spec when we get there.
- **Trajectory-layer pre-shape segment splitting.** Not needed once per-MCU pool sizing fits the workload.
- **Trajectory tunables (`min_n`, `fit_tolerance`).** Quality-of-implementation independent of this spec; apply separately if/when desired.
- **Straight-line specialization in the trajectory layer.** Recognizing G0/G1 linear moves and emitting a simpler curve form that doesn't grow with TOPP-RA grid resolution is a real future optimization (helps Z travels especially), but not required for the curves-first MVP. Tracked as a separate workstream.
- **Endstop / SetHomed / QueryStatus migrations.** Stay on Klipper protocol for now. Migrate later if there's a reason.
- **Schema evolution beyond version 1.** All messages start at version `0x01`. Cross-version migration via per-message-version negotiation is a future-spec problem; today's contract is "schema_hash matches or refuse to dispatch."
- **Authentication / encryption.** No threat model.

The spec scope is bounded by what Phase 4 step pulses actually need, plus the architectural pieces that have to be right from day one (bootstrap ABI, schema hash, reset epoch). Everything else waits.
