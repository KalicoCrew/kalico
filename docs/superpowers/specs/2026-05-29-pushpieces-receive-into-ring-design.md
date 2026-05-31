# PushPieces Receive-Into-Ring — Design

- **Date:** 2026-05-29
- **Branch:** `simple-mcu-contract`
- **Status:** Design — pending implementation plan
- **Related:** `docs/superpowers/specs/2026-05-04-kalico-native-transport-design.md` (§6 demux), `docs/superpowers/specs/2026-05-28-push-pieces-wiring-design.md` (pump), `docs/kalico-rewrite/mcu-c-rust-boundary.md`

## 1. Problem & root cause

A jog reproducibly crashed the bench with a `PieceStartInPast` (-308) shutdown. Full diagnosis this session:

- The host pump sent the H7 a single `PushPieces` frame of **182 pieces (~5.8 KB)**. The frame fit the ring (per-axis depth ≈ 661) but exceeded the MCU's **separate, smaller demux staging buffer** (`KALICO_MAX_PIECES_PER_FRAME = 128`, `KALICO_DEMUX_KALICO_BUF_SIZE = 4132 B`, `src/kalico_demux.h`).
- The demux dropped the oversized frame as `KALICO_DEMUX_OUT_ERROR` (`src/kalico_demux.c:137`) **silently — no response**.
- The host's `send_frame` is a **synchronous, blocking `kalico_call` with a 5 s timeout** (`rust/motion-bridge/src/pump.rs`). With no response it blocked the full 5 s (journalctl: `pump send_frame failed … PushPieces: Timeout`).
- The pump is single-threaded, so that 5 s stall **delayed delivery of the other MCU's (the F446 Z-hold) continuation pieces.** The anchor keeps `t0` fixed for a stream (`rust/motion-bridge/src/anchor.rs:31`), so the late Z pieces carried `start_time` ≈ 4.57 s in the past → `PieceStartInPast` → klippy shutdown.
- The MCU motion loop was healthy throughout: max ISR inter-fire gap **103 µs**, **0 µs** USB interference. This was never a "motion loop can't keep up" problem.

**Root cause:** a host↔MCU frame-size contract violation. The demux staging buffer is a *transient relay* sized for 128 pieces, **separate from and much smaller than the actual ring** (512 on the F4, ≈661/axis on the H7), and **its size is never communicated to the host**. For piece frames that buffer is pure redundancy — pieces are copied out of it into the ring immediately afterward (`src/kalico_dispatch.c:334`).

## 2. Design philosophy: dumb MCU, smart host

The MCU is a **shared circular buffer the host writes into by absolute slot address.** The host owns all accounting (addressing, flow control, the valid frontier); the MCU is a plain ring consumer with **zero write-time rules** — it writes the addressed slots and advances `head`, nothing else. The split follows a hard principle:

> A check belongs on the MCU **only if it requires real-time knowledge the host cannot have** *and* no structural property already covers it. Everything else is the host's job.

The host's view of the consumer position comes from the heartbeat, which lags by up to ~10 ms — and lags *conservatively* (the host believes *fewer* slots are free than truly are, so it always stays behind the consumer). One might think the MCU therefore has to police "you are not playing slot K *at this instant*" — but it does not, because two **structural** properties (not runtime checks) cover it: the consumer caches each piece's coefficients out of the ring before it frees the slot (§4.1), so a ring slot is read only during one transition copy and never across playback; and the host's flow control (§4.4) cannot address a slot the consumer has not yet passed. Protecting the active piece needs no cross-context cursor read, so the MCU does none.

Three consequences fall out for free, eliminating all the machinery an "MCU validates the stream" design would need:

- **Idempotent re-send.** Because writes are addressed to absolute slots, a re-sent frame overwrites the same slots with identical bytes. No duplication, no sequence numbers, no dedup. A lost ACK is harmless.
- **`start_time` is the freshness check — a skipped slot fails safe.** If a host addressing bug advances `head` past a slot it never wrote, the consumer eventually reads that slot's *stale* contents. Those contents are always either cold-initialized zeros (`start_time == 0`, in the past) or the slot's **previous-lap occupant** — which is exactly `N` sequences behind the current read position, so the in-order consumer already played it and its `start_time` is in the past. A future-stamped piece can never sit in a slot being read (start_times rise with sequence; the occupant is always ≥ `N` behind). Either way the stale slot trips `PieceStartInPast` and shuts down safely. No per-slot validity flag, no consumer-side "is this written?" branch.
- **The active piece is immune to overwrites by construction.** The consumer arms (caches) coefficients before freeing the slot and evaluates from the cache, so overwriting a ring slot cannot corrupt in-flight motion. No active-slot guard, no producer-side read of the consumer's cursor.

## 2.1 Goals / Non-goals

**Goals**

1. Eliminate the separate staging buffer: the host writes pieces **directly into ring slots by absolute address.**
2. Make the contract have exactly **two delivery limits, both already known to the host**: per-axis ring capacity (`head − consumed ≤ N`) and the 255-piece wire ceiling (`u8` count). No hidden sub-ring cap → no oversized frame → no silent drop → no 5 s stall.
3. Keep the MCU dumb: a ring consumer plus a slot-writer with **no write-time guards**. No active-slot check, no sequence arithmetic, no validity classification, no commit/abort state, no clearing of consumed slots.
4. Close the latent host `ring_depth` divergence (Gap 2, §4.4): host flow-control depth must equal the depth told to the MCU.
5. Reclaim the SRAM the staging buffer occupied.

**Non-goals (out of scope — §9)**

- Out-of-order / aggressively-pipelined delivery robustness. The model assumes **in-order, promptly-acked per-axis delivery.** If out-of-order writes ever become a problem, it is solved **on the host side** then.
- Exactly-once transport / sequence numbers / dedup — unnecessary given idempotent absolute addressing.
- Acking or retransmitting CRC-corrupted frames.
- Growing the ring with reclaimed SRAM (separate tuning task).
- Reducing per-jog piece count (shaper/flattener question).
- Widening the `u8` piece-count field. 255/frame stays as batching granularity.

## 3. Design overview

The destination axis ring is a shared array of `N` `PieceEntry` slots in `rt_storage`. The host addresses pieces by **physical slot index** (it owns the wrap arithmetic — see §4.4), sends them in CRC-protected frames on a **dedicated transport channel**, and advances the ring's **valid frontier (`head`)**. The MCU consumer plays the window `[consumed, head)` (reading physical slot `tail`), advances `consumed` and `tail` together, and reports `consumed` in the heartbeat. The MCU applies a frame by writing its addressed slots and advancing `head` — **no write-time guard**; the active piece is protected structurally (cached out of the ring before the slot is freed) and the host's flow control cannot address a live slot. Consumed slots are never cleared — they are overwritten by a later lap, and any slot that ends up stale is caught by the `PieceStartInPast` guard.

The C/Rust split is unchanged in shape: C owns USB framing, the demux, CRC, channel routing, and the physical memory the ring lives in; Rust owns the ring data structure and is the sole *writer* of ring contents. The new operation is a **method on the existing `KalicoRuntime` handle** (boundary rule B1 — not a new seam).

## 4. Components & changes

### 4.1 The ring & the MCU consumer — dumb (`rust/runtime/src/piece_ring.rs`, `RingDescriptor`)

The ring tracks `ring_offset`, `ring_depth (N)`, and three live cursors, deliberately split into a *monotonic* pair (for counting) and a *physical* cursor (for addressing) so the design has neither an empty/full ambiguity nor a counter-wrap slot-aliasing bug:

- `head` — monotonic `u32` valid frontier (= the host's `pushed`; set, not incremented, by accepted frames).
- `consumed` — monotonic `u32`, advanced one per consumed piece; reported in the heartbeat.
- `tail` — *physical* read cursor in `[0, N)`, advanced one per consumed piece (wrapping at `N`).

`count = head − consumed` (wrapping subtraction) is the occupied count; because it is a plain difference of monotonic counters it is never reduced `mod N`, so `count == 0` (empty) and `count == N` (full) are distinct, and there is no aliasing at the `u32` wrap. The physical cursor `tail` is advanced *incrementally* (never recomputed as `consumed mod N`), so it is immune to the same wrap.

> **Lockstep is a within-session property.** `tail` matches the host's physical write cursor (§4.4) because both step by one per in-order piece from a shared origin established at session start (MCU `tail = consumed = head = 0`, host `pushed = consumed = 0`). A reboot is a reconnect event handled by re-init — the host re-allocates and re-configures the per-axis ring, re-establishing the origin — which is out of scope (§9). Within a session, lockstep holds.

- **Consumer (ISR):** while `count > 0` (`head != consumed`), evaluate the piece at physical slot `tail`. On a piece *transition* the consumer **caches the piece's coefficients out of the ring slot, then frees the slot** — it copies the `PieceEntry` by value, arms `axis.mono_coeffs`/`vel_coeffs`, and only then advances `tail = (tail + 1) mod N` and bumps `consumed` (the existing arm-before-pop discipline, `engine.rs:667-676`). Every subsequent sample within the piece evaluates from the cache, not the ring. Empty (`count == 0`) → hold position.
- **No write-time guard.** A ring slot is read only during the one transition copy, never across the piece's playback, so overwriting any slot — including the one just armed — cannot corrupt in-flight motion. The producer therefore performs **no** active-slot check and reads **no** consumer cursor; it writes the addressed slots and advances `head`. Overflow (writing a slot the consumer still needs) is prevented upstream by the host's flow control (§4.4), which structurally cannot address a live slot.
- **No clearing.** Consumed slots are left as-is; they are reused when the host writes them on a later lap.

The append-style `push()` (write at the producer cursor, increment by 1) is removed from the live path — writes are now absolute-addressed by physical slot and `head` is host-driven (set, not incremented).

### 4.2 The write seam — stream-write + post-CRC commit (`rust/kalico-c-api/src/runtime_ffi.rs`, `rust/runtime/src/engine.rs`)

Replaces the non-atomic per-piece loop over a contiguous buffer (`kalico_runtime_push_pieces` at runtime_ffi.rs:1488 / `engine::push_pieces` at engine.rs:250). **Crucially the seam must not take a pointer to the whole frame** — that would require the frame to be buffered until CRC, reintroducing the staging buffer this design deletes. Instead the seam is two operations matching the streaming transport (§4.3):

```
// per piece, as it streams in (pre-CRC); Rust owns the slot math
kalico_runtime_write_piece(rt, axis_idx: u8, start_slot: u16, index: u8,
                           piece_ptr: *const u8 /* 32 bytes */) -> i32
// once, after the trailing CRC validates — the commit
kalico_runtime_commit_head(rt, axis_idx: u8, new_head: u32) -> i32
```

`write_piece` copies one 32-byte `PieceEntry` from the sink's scratch into `storage[ring_offset + ((start_slot + index) mod N)]` — unconditionally, no cursor read, no active-slot check (Rust computes the slot, so C does no ring arithmetic; B2). `commit_head` advances `head = max(head, new_head)` (monotone — a stray lower value from an out-of-order re-send is ignored, one wrapping comparison). Both are `extern "C"` with scalar + ptr params (B4), methods on the existing handle (B1). No reserve, feed, or abort.

This is **atomic by construction**: slot bytes stream into the ring *before* CRC, but `head` is committed *only after* CRC. A fresh frame's slots sit at/above `head` (outside the consumer's `[consumed, head)` window) until the commit, so a corrupt or partial frame — which never reaches `commit_head` — leaves bytes that are never played; a re-send's slots are inside the window but byte-identical, so a torn transition-copy read is harmless (§6.5, §8 B5). The slot writes and the head commit are decoupled, which is exactly what makes "stream with no staging buffer" and "atomic visibility" coexist.

> Note: the two address fields play distinct roles — `start_slot` is a **physical** slot index (`0..N`) computed by the host (§4.4), used to place bytes; `new_head` is the **monotonic** `u32` frontier (= post-frame `pushed`), used for the count `head − consumed`. The MCU never reduces a monotonic counter `mod N`, so it is immune to host-counter wrap.

### 4.3 Transport — dedicated piece channel, no message logic in the demux (`src/kalico_demux.c/.h`, piece sink)

Pieces move on a **new, dedicated transport channel** (`KALICO_CHANNEL_PIECES`). The demux already routes by *channel* (`[sync][len][channel][payload][crc]`) — a pure transport concern — so the new channel id plus the streaming sink it routes to are the genuinely new transport code; the routing dispatch itself is unchanged. For the piece channel the demux does not accumulate into `kalico_buf`; it **streams the payload to a thin piece sink**:

1. The sink reads the small frame header (`correlation_id`, `axis_idx`, `start_slot`, `new_head`, `piece_count`) and captures the `correlation_id` for the response.
2. As complete 32-byte pieces arrive (assembled in a 32-byte scratch across USB-chunk boundaries), each is written straight into its ring slot via `kalico_runtime_write_piece(...)` (§4.2) — pre-CRC, no frame buffering; the demux folds CRC over every frame byte.
3. On the trailing CRC: **match** → `kalico_runtime_commit_head(axis, new_head)` (the commit) then emit `PushPiecesResponse(OK)` via `send_push_pieces_response` with the captured `correlation_id`. **Mismatch** → discard (slots may have been written but `head` is **not** committed, so they are never played; a re-send overwrites them); resync; no response (correlation_id untrusted).
4. The existing 100 ms idle-reset resyncs a truncated frame; since nothing is applied until CRC, there is nothing to undo.

Non-piece kalico frames (control: `configure_axis`, caps query, clear) are unchanged — they accumulate in the now-small `kalico_buf` and route through `kalico_dispatch_frame` as today. `handle_push_pieces` is retired.

> **Wire-format / schema touchpoint.** The piece frame header gains `start_slot` (`u16`) and `new_head` (`u32`) and moves to a new channel — a protocol change. The `kalico-protocol` schema (`rust/kalico-protocol/`) and both MCU builds must change together; the bench flow already flashes host + both MCUs from one commit, so this is a coordination note, not a new risk.

### 4.4 Host — owns addressing, flow control, `head`; single-source depth (Gap 2) (`rust/motion-bridge/src/`)

The host is the smart side. Using the per-axis monotonic counters it already has (`pushed`/`consumed`, pump.rs):

- **Addressing.** The host maintains a per-axis physical write cursor in `[0, N)`, advanced by `count` (wrapping at `N`) each frame — derived incrementally, *not* as `pushed mod N`, so it is immune to the `u32` wrap of `pushed`. Each frame carries `start_slot` (the cursor) and `new_head`.
- **Flow control (kills the stall *and* protects the slot being read).** The host sends only what fits: `in_flight = pushed − consumed`, `room = N − in_flight`, `min(room, 255)` per frame. With no separate buffer and the ring as the only limit, **there is no oversized frame to drop**, so the MCU always responds and the pump never hits the 5 s timeout. The precise safety property is *"the host never overwrites a slot the consumer is currently reading,"* proved in two cases (using the true `consumed`; the host's lagging view is ≤ it, only more conservative):
  - **Empty ring** (`in_flight == 0`, so `room == N`): a single frame *may* write all `N` slots, the lowest sequence being `pushed == consumed` — i.e. it *does* touch slot `consumed mod N`. But `count == 0` means the consumer is **holding and reads no slot** (§4.1), and `head` is committed only after the writes, so the fill-from-empty write is harmless.
  - **Non-empty ring** (`1 ≤ in_flight ≤ N − 1`; `in_flight == N` ⇒ `room == 0` ⇒ no frame): the written sequence range is `[pushed, consumed + N − 1] = [consumed + in_flight, consumed + N − 1]`, which **excludes `consumed`** — so the slot being read, `consumed mod N`, is never overwritten while live. (The earlier "highest slot is the just-freed `(consumed − 1) mod N`" is the top of this range; the live-slot guarantee is that `consumed` is below the range, not above it.)
- **`pushed` advances on ACK** (as today). A lost ACK ⇒ the frame is re-sent with the **same `start_slot`** ⇒ idempotent overwrite.
- **Gap 2 — single-source depth.** Today `axis_ring_depth()` is `u32` (bridge.rs:509), the value sent to the MCU is clamped to `u16` with only a warning (bridge.rs:574), and the pump table uses the unclamped `u32` (bridge.rs:2152) — they diverge above 65535 (latent: 661 ≪ 65535). Fix: compute each axis's depth **once**, use the **identical value** for `configure_axis` and the pump, and make a depth exceeding the wire field a **hard error**, not a silent clamp (a >65535-piece ring is ≥2 MB — physically impossible here, so it can only be a derivation bug).

No other host change is required; `schedule()` already gates on `room()` and splits at 255.

### 4.5 Staging-buffer shrink + stale reservation (`src/kalico_demux.h`, `src/runtime_storage.c`)

- Resize `KALICO_DEMUX_KALICO_BUF_SIZE` from the 128-piece value (4132 B) down to the **largest non-piece (control) inbound frame**, guarded by a `_Static_assert`.
- Correct the **stale** `AXI_BSS_KALICO_BUF_BYTES = 14752` reservation (runtime_storage.c:56, a NURBS-era figure) to the new value; the AXI-SRAM overflow `_Static_assert` keeps the budget honest. The reclaimed ~14 KB on the H7 is left unallocated (a follow-on may grow the ring; out of scope).

## 5. Data flow (happy path)

```
USB bytes ─[C serial_irq]─► receive_buf
          ─[C demux]─► route by channel == PIECES → piece sink (no kalico_buf)
                       │  read header (axis, start_slot, new_head, count); fold CRC
                       │  per piece → kalico_runtime_write_piece(...)  [Rust, pre-CRC]
                       │       └─ write physical slot (start_slot+i) mod N (no cursor read)
                       └─ trailing CRC matches?
                              ├─ yes → kalico_runtime_commit_head(axis,new_head); send PushPiecesResponse(OK)
                              └─ no  → discard (slots written but head NOT committed → never played); resync; no response

TIM5 ISR ─[C]─► kalico_runtime_tick_sample ─[Rust]─► play slot tail while count>0 (head!=consumed);
                                                     advance tail (mod N); bump consumed
Heartbeat ─[Rust→host]─► consumed (monotonic)
```

Exactly one copy occurs (USB buffer → ring slot), versus two today.

## 6. Failure modes & safety

The unifying invariant: **`head` is committed only on a fully-received, CRC-valid frame; the consumer only plays the monotonic window `[consumed, head)`.** Everything else composes from that plus idempotent absolute addressing.

1. **Oversized frame — cannot happen.** No separate buffer; the host sends only what fits the ring. The original silent-drop → 5 s-stall chain is removed at the source.
2. **Bad CRC.** `head` not committed; any slots that were streamed in are outside `[consumed, head)` and never played; a re-send overwrites them. No response (correlation_id untrusted); host's existing retry applies.
3. **Lost ACK / re-send.** Idempotent — same `start_slot`, identical bytes, overwrite in place. No duplication, no `PieceStartInPast` from duplicates.
4. **Host skip / mis-address (host bug) — fails safe.** If the host advances `head` past a slot it never wrote, the consumer reads stale contents when it reaches that slot. The previous occupant of any physical slot is exactly `N` sequences behind the current read sequence, so by the time the consumer reaches it the occupant was already consumed (played) and its `start_time` is in the past — or the slot is cold-initialized (`start_time == 0`). Either trips `PieceStartInPast`. A future-stamped stale piece is impossible in a slot being read (start_times rise with sequence; the occupant is always ≥ `N` behind). So a host addressing bug fails safe; it does not occur under a correct host.
5. **Overwriting the slot being read — structurally prevented, no MCU check.** Flow control proves (§4.4, two cases) the host never overwrites slot `consumed mod N` *while the consumer is reading it*: the non-empty range excludes `consumed`, and the empty-ring case writes it only when the consumer reads nothing. Independently, the consumer caches the active piece's coefficients out of the ring before freeing the slot (§4.1), so even a host *bug* that overflows cannot corrupt in-flight motion — the worst case is a slot read on the next lap (caught per item 4). The sole residual torn-read window is the one 32-byte transition copy, which requires that host bug *and* is harmless for a re-send (identical bytes). The MCU does no active-slot check, racy or otherwise.
6. **Underflow (genuinely late delivery).** `count == 0` (`head == consumed`) → consumer holds position. A piece delivered after its `start_time` is caught by `PieceStartInPast` (the genuine-late guard — distinct from the now-impossible duplicate case).
7. **Out-of-order host writes.** Not handled on the MCU — host responsibility, deferred (§9). Assumes in-order per-axis delivery.
8. **Rare lost-ACK stall.** A genuinely lost ACK still blocks the single-threaded pump for the timeout; the idempotent re-send recovers. If this ever bites cross-axis, shorten the timeout or make per-axis delivery independent. Not required to fix the (systematic) original bug.

## 7. Invariants

- **No-oversend.** Host sends only `min(room, 255)`, `room = N − (pushed − consumed)`; `head − consumed ≤ N` always. (Verified `holds-with-conditions` by the 6-agent `verify-host-never-oversends` sweep, 2026-05-29; the atomicity condition is satisfied structurally by §4.2, and the single-source-depth condition by §4.4 / Gap 2.)
- **Atomic visibility.** `head` advances exactly once per frame, post-CRC; the consumer never sees a partial/corrupt frame.
- **Idempotency.** Absolute slot addressing ⇒ re-send overwrites in place.
- **Freshness backstop.** A skipped/stale slot is always read with a past `start_time` — its occupant is ≥ `N` sequences behind the read point (already consumed) or cold-initialized (`start_time == 0`) — so `PieceStartInPast` catches it. No per-slot validity state. (§6.4)
- **No MCU write-time check.** The active piece is protected structurally (cached out of the ring before its slot is freed) and overflow is prevented by host flow control (which cannot overwrite the slot being read, §4.4) — so the producer reads no consumer cursor and performs no active-slot guard.

Standing preconditions (true today; stated so they're not silently broken): the pump is single-threaded; heartbeat `consumed_counts[i]` maps to `AxisKey.axis i`; `configure_axis` precedes the first frame per axis.

MCU reboot recovery is **out of scope (§9):** a reboot is a reconnect event, after which the host re-queries caps and re-allocates / re-configures the per-axis ring — which re-establishes the shared origin (all cursors zero) as a matter of course. This spec covers the steady-state receive path within one session.

## 8. C/Rust boundary compliance (`docs/kalico-rewrite/mcu-c-rust-boundary.md`)

- **B1 (narrow seam):** `write_piece` and `commit_head` are methods on the existing `KalicoRuntime` handle — not a new logical seam.
- **B2 (C owns shared memory):** the ring lives in C-placed `rt_storage`; Rust overlays `RuntimeContext`, computes slot indices, and remains the sole *writer* of ring contents (C passes 32-byte payloads; it never writes ring memory or does ring arithmetic).
- **B3 / B4 (no Rust types cross the ABI):** the seam passes `u8`/`u16`/`u32`/`*const u8` only.
- **B5 (ordering — by preemption, not atomics).** The ring cursors (`head`/`tail`/`consumed`/`count`) are **plain non-atomic** `RingDescriptor` fields (`piece_ring.rs:52-65`), per boundary rule B5 ("where C uses a plain shared word, Rust does too"). Ordering is carried by **same-core asymmetric preemption**, not acquire/release: the foreground producer writes all addressed slots and then `commit_head` advances `head`, in program order; the TIM5 ISR consumer (NVIC priority 2) preempts the foreground producer but is **never** preempted by it, so on its synchronous exception entry it observes the producer's completed slot writes before any `head` it reads. `head` is a single aligned `u32` (atomic LDR/STR on ARMv7E-M). No `core::sync::atomic`, no explicit fence — adding one would be misleading no-op work. The producer reads no consumer cursor at all; `consumed` is MCU-owned and surfaced to the host only via the heartbeat; `tail` is purely internal.

## 9. Out of scope

- **Host addressing correctness is assumed, not enforced.** The MCU trusts the host to write every slot it advances `head` past, in order. The MCU does not validate sequence/contiguity; a host that skips a slot still fails safe via `PieceStartInPast` (§6.4), but detecting/repairing such a host is not attempted. This is the dumb-MCU/smart-host stance: correctness lives on the host.
- **MCU reboot / reconnect recovery.** A reboot is a reconnect event; the host's reconnect path re-queries caps and re-allocates / re-configures the per-axis ring, re-establishing the shared origin. That lifecycle is a separate concern from the steady-state receive path specified here.
- Out-of-order / pipelined delivery robustness — solved host-side if/when it bites (§6.7).
- Exactly-once transport (replaced by idempotent addressing) and ack/retransmit of CRC-corrupted frames.
- Spending the reclaimed ~14 KB AXI SRAM on a deeper ring.
- Reducing per-jog piece count (shaper/flattener).

## 10. Testing

- **Ring (host unit):** consumer plays `[consumed, head)` and holds at `count == 0`; an absolute write lands in the addressed slot; no clearing on consume; `head` is monotone (a lower `new_head` is ignored).
- **Copy-on-arm (host unit):** after a transition, overwriting the slot the consumer just armed does **not** change the in-flight evaluation (coefficients are cached out of the ring); the overwritten bytes are seen only on the next lap.
- **Flow-control bound, two cases (host unit):** non-empty (`1 ≤ in_flight ≤ N−1`) with `count = room` → the addressed range excludes the live slot `consumed mod N` (top of range is the just-freed `(consumed−1) mod N`); empty (`in_flight == 0`) with `count = N` → the range *does* include `consumed mod N`, and the test asserts the consumer holds (reads nothing) so it is harmless. Both across the `consumed` wrap.
- **write_piece / commit_head (host unit):** `write_piece` lands at `(start_slot+index) mod N` unconditionally (no cursor read); slots are visible to the consumer only after `commit_head`; `commit_head` is monotone (a lower `new_head` is ignored); re-streaming the same frame is a no-op overwrite (idempotent); unconfigured axis rejected.
- **Transport (host/C unit):** a 182-piece frame streams piece-by-piece into the ring and commits `head` on CRC; CRC-bad frame leaves `head` un-committed (streamed slots never played); truncated frame (idle-reset) commits nothing; control frames still route through the small buffer; a frame split across pump-buffer boundaries reassembles; piece channel never touches `kalico_buf`.
- **Freshness backstop (host/sim):** a skipped slot (head advanced past an unwritten slot) is read with a past `start_time` — the previous-lap consumed occupant, or a cold-initialized zero — and yields `PieceStartInPast`, never silent wrong motion. A test that fills a slot, laps the ring so the consumer is `N` sequences ahead, then skips it, confirms the occupant's `start_time` is in the past at read time.
- **Host (motion-bridge unit):** the depth used for the pump equals the value sent to `configure_axis`; a synthetic `> u16::MAX` depth is a hard error; `schedule()` splits `> 255` and never plans beyond `room()`; the physical write cursor advances correctly across an `N`-wrap and across the `u32` `pushed`-wrap.
- **Static asserts:** `KALICO_DEMUX_KALICO_BUF_SIZE` ≥ largest control frame; AXI-SRAM budget after the shrink.
- **Bench (regression):** the exact repro — `SET_KINEMATIC_POSITION X=150 Y=150 Z=5`, alternating ±10 mm X jogs — completes without `PieceStartInPast`; pump-diag shows `SEND mcu0` succeeding (no 5 s `PushPieces: Timeout`); a forced re-send (drop one ACK) does not corrupt motion.

## 11. Alternatives considered

- **Report the 128-piece cap to the host and split to it.** Cheapest, but keeps a redundant double-buffer, a second arbitrary cap, and double RAM. Rejected (papers over the redundancy).
- **Reserve / feed / commit / abort staging (append-at-cursor).** Streams into the ring but appends at the producer cursor, so a re-send *duplicates* → `PieceStartInPast` halt (an accepted-failure we had to document), and it needs MCU-side staging state and an atomic-commit guarantee. Superseded by absolute addressing, which is idempotent and needs no staging state.
- **Sequence numbers + skip/overwrite/reject + contiguity tracking on the MCU.** Makes the MCU robust to out-of-order/lapped re-sends — but pushes real logic onto the MCU. Rejected per "dumb MCU": the host owns addressing, and `start_time` is the freshness backstop. Out-of-order is deferred to the host if it ever matters.
- **Per-piece "committed"/validity flag in `PieceEntry._reserved`.** Adds a consumer hot-path branch and is unnecessary: `head` is the validity boundary and `start_time` the freshness check. Rejected; `_reserved` stays reserved.
- **Producer-side active-slot guard (MCU reads `tail`, skips/refuses that slot).** An earlier draft of this design. Rejected: it is a check-then-act spanning the producer (USB context) and the ISR that advances `tail`, so the guarantee is illusory; and it is redundant — the consumer already caches the active piece out of the ring (§4.1), so the slot it would "protect" is not read during playback. Replaced by the structural guarantees (copy-on-arm + flow control), and the MCU loses its last write-time check.
- **Consumer-side seqlock / per-slot version counter.** Would make even a host-overflow-*bug* torn read airtight (the consumer detects a mid-write slot and retries). Rejected: it adds a version field (consuming `_reserved`), producer write-fences, and a retry loop in the 40 kHz hot path — real MCU logic against "dumb MCU" — to close a host-bug-only, sub-µs window already bounded by `start_time`.
- **Enlarge the demux buffer to 255 pieces.** Just widens the ceiling; doubles receive RAM on the tight F4 and still leaves a separate cap. Rejected.
