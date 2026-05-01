# Step 7-C-io — Production host I/O

**Layer:** 5 (host↔MCU communication hardening — closes Step-6 Plan-decision-C deferrals).

**Scope:** This spec is the **first half** of CLAUDE.md's Step 7-C ("klippy bridge + production host I/O"). The other half — Python ↔ Rust integration that routes existing Klipper configs through kalico's planner — is reserved as a sibling step: **Step 7-C-bridge**.

## 1. Goals & non-goals

### 1.1 In scope

This spec hardens `rust/kalico-host-rt/` into a production-correct multi-MCU host runtime by closing the Plan-decision-C deferrals carried over from Step 6.

1. **Production `MsgProtoParser`** — zlib-decompressed Klipper data-dictionary parse, full encode/decode for the kalico command surface, dictionary-driven dispatch (commands, responses, output, enumerations). Replaces the Step-6 stub at `rust/kalico-host-rt/src/host_io.rs:567-670`.
2. **Single-threaded poll-reactor `host_io`** per port. One dedicated OS thread per port owns rx + tx + retransmit + RTO + frame routing + AwaitingResponse + UnackedWindow + EventDispatcher. Public `KalicoHostIo` is `Send + Sync` with all mutable state inside the reactor thread.
3. **NAK detection** as duplicate-ack-as-NAK (Klipper-equivalent), with `ignore_nak_seq` damper + `rtt_sample_seq` poison-protection.
4. **NAK-driven AND timeout-driven retransmit**, with RFC 6298 SRTT/RTTVAR/RTO clamped `[25 ms, 5 s]`.
5. **64-bit absolute seq counters**; wire low-4-bit; window cap = 12 (`MAX_PENDING_BLOCKS`).
6. **Identify-during-reconnect race recovery**: pre-identify mode, empty-`UnackedWindow` short-circuit, drain window.
7. **Async-event subscriber model** with per-channel backpressure (credit, fault, trace, status, runtime-event catch-all, host-event diagnostics).
8. **`ArmError::QualityGate` detail** carries MCU index + which subgate failed + offending measurement.
9. **`arm_all_mcus` request_id correlation** — each MCU's clock-sync request emits a unique monotonic `request_id` matched in the response, replacing today's hardcoded `request_id=1`.
10. **Test strategy**: Python `klippy/msgproto.py` as bootstrap-only differential oracle during dev (gated behind `python-diff-test` Cargo feature with dedicated CI lane); H723 firmware-emitted capture corpus as the canonical CI reference once available.
11. **Source-side async-event registration fix** — convert all `kalico_*` async-event emit sites in `runtime_tick.c` to use `output(...)` (i.e. `_DECL_OUTPUT`) exclusively:
    - `runtime_tick.c:282` `sendf("kalico_fault ...")` → `output("kalico_fault ...")` (kalico_fault is dual-emitted from this edge-trigger site and from `:319`'s status path; both should be `output()`).
    - `runtime_tick.c:231` `sendf("kalico_trace count=%u data=%*s")` → `output("kalico_trace count=%u data=%*s")`.
    - Audit any other kalico async-event emit sites and convert them too (the currently-known async events are `kalico_credit_freed`, `kalico_fault`, `kalico_status_v6`, `kalico_trace`).
    
    The current `sendf` pattern triggers `buildcommands.py`'s first-registration-wins de-dup (per the comment at `runtime_tick.c:261-267`) and puts these events in `responses` instead of `output`. Regenerate `out/klipper.dict` after the fix. Add a build-time invariant check that asserts all `kalico_*` async-event format strings register only via `_DECL_OUTPUT`. Wire-frame payload bytes are unchanged; only the dictionary category and msgid value differ.

### 1.2 Out of scope

- **Python ↔ Rust integration** to route existing Klipper configs through kalico's planner — Step 7-C-bridge.
- **Replacing `klippy/msgproto.py`** — klippy retains it for non-motion MCU subsystems (heaters, sensors, GPIO, TMC, etc.). Far post-7-C.
- **EtherCAT backend transport** — Step 14.
- **`kalico-host-rt` exposing an async API surface** — internal poll-reactor stays threaded; sync `call()` API.
- **Major firmware-side wire format changes** — none. Item 11 is a small category-registration cleanup (a few `sendf` → `output` site conversions plus a build-time invariant); wire frame payloads are unchanged.
- **Hardware bring-up** (SDIO/F4x integration, M1/M2/M3 soaks, calibration, first print) — Step 7-D.

### 1.3 Non-goals (deliberate punts)

- **Concurrent senders on a single `Transport`** — poll-reactor serializes; if needed later, additive change to the submission API.
- **UART-specific retransmit handling** (`tcflush(TCOFLUSH)`) — H723 is USB-CDC only; UART is a future-transport concern.
- **Klipper `notify_id`-style firmware-level RPC correlation** — wire-level seq + FIFO matching covers our call graph; can be added additively if response-out-of-order ever becomes a thing.

## 2. Architecture

### 2.1 Component layout

One `KalicoHostIo` per serial port. Internally:

```
                        caller threads (producer, stream-arm,
                          future klippy-bridge, telemetry)
                                        |
                              call() / submit / subscribe
                                        |
                                   submission mpsc
                                        |
                                        v
+-------------------------------------------------------------------+
|                      Reactor thread (1 per port)                  |
|                                                                   |
|  +---------+    +-----------------+    +----------------------+   |
|  | port    |--->| Frame parser    |--->| WaitingWindow (≤12)  |   |
|  | (RW)    |    | + MsgProtoParser|    |  - by seq, FIFO      |   |
|  +---------+    +-----------------+    |  - oneshot completion|   |
|       ^                                +----------+-----------+   |
|       |                                           |               |
|  +-----------+    +-----------------+    +-------------------+    |
|  | tx framer |<---| submission      |    | RttEstimator      |    |
|  +-----------+    | dispatcher      |    | (RFC 6298)        |    |
|                   +-----------------+    +-------------------+    |
|                                                                   |
|  +------------------------+    +-------------------+              |
|  | EventDispatcher        |    | ArcSwap<Status>   |              |
|  |  - credit (snap)       |    |   (shared with    |              |
|  |  - fault (latched)     |    |    public struct) |              |
|  |  - trace ring          |    +-------------------+              |
|  |  - runtime catch-all   |                                       |
|  |  - host events         |                                       |
|  +------------------------+                                       |
+-------------------------------------------------------------------+
```

### 2.2 Module layout

In `rust/kalico-host-rt/src/`:

- `host_io/mod.rs` — public `KalicoHostIo` struct, constructor (`open`, `open_with_config`), Drop, `Transport` impl.
- `host_io/reactor.rs` — `Reactor` state machine and dedicated thread loop. Owns port + parser + WaitingWindow + UnackedWindow + RttEstimator + EventDispatcher.
- `host_io/window.rs` — `UnackedWindow` (≤ 12, by seq) + `AwaitingResponse` (uncapped, FIFO-by-name match).
- `host_io/rtt.rs` — `RttEstimator` (RFC 6298, `[25 ms, 5 s]` clamp, `MIN_RTO` initial floor).
- `host_io/parser.rs` — production `MsgProtoParser` (canonical Layer A).
- `host_io/runtime_events.rs` — kalico-specific structured event extension (Layer B); emits `RuntimeEvent` typed values.
- `host_io/events.rs` — `EventDispatcher` (credit, fault, trace, status, catch-all, host-events).
- `host_io/wire.rs` — frame layout, retransmit-buffer assembly with leading SYNC byte.
- `host_io/identify.rs` — pre-identify mode state machine, drain window, identify chunk loop.
- `transport.rs` — `Transport` trait reshape (`&self` + `Send + Sync`, `call`/`call_typed`).

Existing modules (`clock_sync.rs`, `credit.rs`, `fault.rs`, `producer.rs`, `stream.rs`) get call-site updates only; their core logic is unchanged.

**New crate dependencies:** `flate2`, `serde_json`, `indexmap` (with `serde` feature), `arc-swap`. All small, well-vetted.

### 2.3 Threading model

- One reactor thread per `KalicoHostIo` instance (one per port).
- Reactor uses `std::thread::spawn` with a tight poll loop using `serialport`'s short-timeout blocking reads. No `tokio` / `mio` / `polling` dependency.
- Public `KalicoHostIo` holds only thread-safe handles (mpsc Sender, AtomicU64, Option<JoinHandle>, Arc<ArcSwap<...>>).
- Caller threads submit via mpsc; block on a `sync_channel(1)` rendezvous.

## 3. Wire protocol state machine

### 3.1 Counters and init values

Mirrors `klippy/chelper/serialqueue.c:660-666` exactly. All `u64`, owned by Reactor.

| Counter | Init | Meaning |
|---|---|---|
| `send_seq` | 1 | Seq to assign to the next outbound frame. Stamped onto frame, then incremented. |
| `receive_seq` | 1 | Highest absolute seq decoded from MCU + 1. Advanced unconditionally on every inbound non-duplicate frame (ack, nak, data alike — `serialqueue.c:238`). |
| `last_ack_seq` | 0 | Latest `rseq` whose ack we have observed and processed. Drives duplicate-ack-as-NAK detection: a same-seq ack arriving after `last_ack_seq == rseq` is interpreted as NAK and triggers fast retransmit. |
| `ignore_nak_seq` | 0 | NAK damper threshold. Two-arm assignment on retransmit (§3.8). |
| `retransmit_seq` | 0 | `send_seq` at the moment of the most recent retransmit. Upper boundary of the retransmit window. |

No `u64::MAX` sentinels.

### 3.2 Wire-seq decode

Wire carries low 4 bits. Reconstruct absolute against `receive_seq`:

```
delta    = u64::wrapping_sub(wire_seq_u64, receive_seq) & MESSAGE_SEQ_MASK
absolute = receive_seq + delta              // wraparound impossible: delta ≤ 15
```

Unambiguous iff `send_seq − receive_seq < 16`, enforced by the unacked-window cap of **12** (4-frame guard band).

### 3.3 Two parallel maps

**`UnackedWindow`** — wire-level retransmit state. `VecDeque<UnackedEntry>`, ordered by seq, ≤ 12 entries.

```rust
struct UnackedEntry {
    seq:         u64,
    frame_bytes: Vec<u8>,
    sent_at:     Instant,
    retry_count: u32,
}
```

Removed when an ack with `rseq > entry.seq` arrives (ack covers `seq < rseq`, strict).

**`AwaitingResponse`** — logical-RPC state. `VecDeque<AwaitEntry>`, ordered by submission time. Uncapped at the wire-window level (defensive ceiling 1024).

```rust
struct AwaitEntry {
    call_id:                u64,
    seq:                    u64,
    expected_response_name: String,
    completion:             SyncSender<Result<MessageParams, TransportError>>,
    submitted_at:           Instant,
    deadline:               Instant,        // submitted_at + per-entry timeout
    abandoned:              bool,
}
```

Removed when (a) named response FIFO-matches; (b) `Abandon(call_id)` arrives; (c) deadline passes (`TransportError::DispatcherTimeout`); (d) reactor enters Closed.

### 3.4 Outbound submission flow

`call(cmd, expected_response_name, timeout) -> Result<MessageParams, TransportError>`:

1. Caller increments shared `AtomicU64` for `call_id` (`Ordering::Relaxed`).
2. Caller creates `sync_channel::<Result<...>>(1)`.
3. Caller sends `ReactorCommand::Submit { call_id, cmd, expected_response_name, completion: tx, deadline }` over submission mpsc.
4. Caller `rx.recv_timeout(timeout)`. On any return path, drops a `CallHandle` whose `Drop` impl sends `ReactorCommand::Abandon(call_id)` unless `defused`.

Reactor processes `Submit`:

1. If `UnackedWindow.len() == 12`, push the submission to per-port pending queue (does not busy-spin).
2. Encode via `MsgProtoParser::encode(cmd)`.
3. `seq = send_seq; send_seq += 1;` Frame layout: `[len, dest|wire_seq, payload..., crc_hi, crc_lo, SYNC]`.
4. **Single chokepoint**: `write_frame(&frame_bytes)` (§3.7).
5. Push to `UnackedWindow`. Push to `AwaitingResponse`.
6. RTT-sample arming: `if !rtt_sample_armed { rtt_sample_seq = seq; rtt_sample_armed = true; }`

### 3.5 Inbound ack/nak handling (5-byte minimum frame)

The wire-seq decode formula (§3.2) guarantees `rseq >= receive_seq`, so only two cases are reachable: `rseq > receive_seq` (forward progress) and `rseq == receive_seq` (no advancement, but possibly a duplicate ack). Mirrors `klippy/chelper/serialqueue.c:230-266`:

```
rseq = decode_absolute(wire_seq)

// Step 1: advance receive_seq if rseq is new (also pops UnackedWindow + samples RTT).
//         No-op when rseq == receive_seq (duplicate-ack case).
if rseq > receive_seq:
    update_receive_seq(rseq)

// Step 2: ack/nak handling — runs in BOTH cases (forward progress AND duplicate ack).
if last_ack_seq < rseq:
    last_ack_seq = rseq                // forward-progress ack
elif rseq > ignore_nak_seq && !UnackedWindow.is_empty():
    write_retransmit(NakDriven)         // duplicate-ack-as-NAK → fast retransmit, INLINE
// else: stale ack damped by ignore_nak_seq (or no in-flight frames to retransmit), drop silently
```

The dupe-ack-as-NAK detection runs uniformly after the (potentially-no-op) `update_receive_seq` call. `last_ack_seq` is the discriminator: a fresh ack advances it; a same-seq ack with `rseq <= last_ack_seq` (since `last_ack_seq` was set to `rseq` on the prior forward-progress step) hits the `elif` branch.

`update_receive_seq(rseq)`:

```
if UnackedWindow.is_empty():
    // Connection-init / mid-session-reopen short-circuit (serialqueue.c:168-172).
    send_seq = rseq
    receive_seq = rseq
    return

while let Some(front) = UnackedWindow.front():
    if front.seq < rseq:                                     // strict <
        let entry = UnackedWindow.pop_front();
        if entry.seq >= rtt_sample_seq && rtt_sample_armed:
            rtt_estimator.update(now - entry.sent_at);
            rtt_sample_armed = false;                        // re-arm on next fresh send
    else:
        break

receive_seq = rseq
```

### 3.6 Inbound non-ack frames (real msg-id)

```
rseq = decode_absolute(wire_seq)
if rseq != receive_seq:
    update_receive_seq(rseq)            // unconditional advancement

let decoded = parser.decode(&packet)?    // see §4.7 — DispatchSpec-aware

match decoded:
    DecodedFrame::Response { name, params } =>
        // FIFO match against AwaitingResponse, skipping abandoned entries.
        for entry in AwaitingResponse.iter_mut():
            if entry.abandoned: continue
            if entry.expected_response_name == name:
                let _ = entry.completion.try_send(Ok(params));
                AwaitingResponse.remove(entry)
                return
        // No matching call → lift to typed RuntimeEvent and dispatch.
        let event = runtime_events::lift(name, params)
        event_dispatcher.dispatch(event)

    DecodedFrame::Output { name, params } =>
        // Output frames are unsolicited async events; never match AwaitingResponse.
        // §4.7 produces (kalico_*, named_params) for canonical-recoverable formats
        // OR ("#output", {"#msg": ...}) for free-form formats; either shape lifts
        // correctly (the latter routes to RuntimeEvent::UnknownOutput).
        let event = runtime_events::lift(name, params)
        event_dispatcher.dispatch(event)
```

### 3.7 Reactor loop step ordering & `write_frame` chokepoint

There is exactly one function that writes to the port: `write_frame(&[u8])`. Called inline from three sites:

- §3.4 step 4 (fresh submission)
- §3.5 NAK-driven retransmit
- §3.8 RTO timer step

Because the reactor is single-threaded and `write_frame` is synchronous, no fresh frame can be written between NAK observation and the corresponding retransmit write. The "retransmit drains before further submission" invariant is structural, not scheduled.

Reactor loop:

```
loop {
    // 1. Drain reactor commands (Submit, Abandon, Subscribe*, Shutdown).
    //    Bounded count per iteration (MAX_SUBMITS_PER_ITER = 4).
    for _ in 0..MAX_SUBMITS_PER_ITER {
        match submission_mpsc.try_recv() {
            Ok(Submit{...})      => dispatch_submission(...),
            Ok(SubmitTyped{...}) => dispatch_submission_typed(...),
            Ok(Abandon(id))      => mark_abandoned(id),     // unknown id = no-op
            Ok(Subscribe*{...})  => attach_subscriber(...),
            Ok(Shutdown)         => transition_closed(),
            Err(Empty)           => break,
            Err(Disconnected)    => transition_closed(),
        }
    }

    // 2. Read step (blocking, 100 ms timeout). NAK-driven retransmit
    //    is fully serviced before this step returns.
    poll_serial(Duration::from_millis(100));

    // 3. Drain pending submissions. Acks processed in step 2 may have freed
    //    UnackedWindow slots; flush §3.4-step-1's per-port pending queue
    //    before any further wire writes. Without this step the queue would
    //    only drain on the next caller-issued Submit.
    drain_pending_submissions();

    // 4. RTO timer step. If oldest UnackedWindow entry's deadline passed,
    //    inline timeout-driven retransmit.
    if let Some(front) = UnackedWindow.front() {
        if now >= front.sent_at + rtt_estimator.current_rto() {
            write_retransmit(TimeoutDriven);
        }
    }

    // 4b. Latch any reactor-staged host fault. Sites that detect a fatal
    //     condition mid-handler (port disconnect at §3.11, retransmit
    //     exhaustion at §3.8) cannot directly call event_dispatcher because
    //     they don't own the borrow path; they stage a FaultEvent in
    //     `pending_host_fault`, drained here.
    if let Some(fault) = pending_host_fault.take() {
        event_dispatcher.fault_latch.dispatch(fault);
    }

    // 4c. Forward any TraceRing host-event diagnostics queued in the shared
    //     inbox (overflow / subscriber-disconnect / reattach — §6.4) to the
    //     host-event subscriber.
    event_dispatcher.host_event_dispatcher.drain_pending();

    // 5. AwaitingResponse GC step.
    gc_awaiting_response(now);

    // 6. Closed-state exit.
    if state == Closed { flush_all_completions(); break; }
}
```

### 3.8 Retransmit action

`write_retransmit(trigger)`:

1. Build retransmit buffer: `[MESSAGE_SYNC]` followed by every `UnackedWindow` entry's `frame_bytes`, contiguously.
2. `port.write_all(&buf)?; port.flush()?;`
3. **Two-arm `ignore_nak_seq`** (mirrors `serialqueue.c:431-442`):
   ```
   match trigger {
       NakDriven => {
           if receive_seq < retransmit_seq:
               // Second NAK before receiver caught up to where we already retransmitted to.
               ignore_nak_seq = retransmit_seq
           else:
               ignore_nak_seq = receive_seq
       }
       TimeoutDriven => {
           ignore_nak_seq = send_seq
       }
   }
   ```
4. `retransmit_seq = send_seq`.
5. `rtt_sample_armed = false`.
6. Each entry: `retry_count += 1`. If any hits `MAX_RETRY_COUNT = 8`, enter Closed with `KALICO_ERR_HOST_RETRANSMIT_EXHAUSTED` (new host-originated fault code; see §6.11).
7. **Only on `TimeoutDriven`**: `rtt_estimator.backoff()` (doubles RTO, clamped). NAK-driven retransmit does NOT back off RTO — `serialqueue.c:439` only doubles on the timeout branch. Doubling on every NAK would inflate the RTO floor under packet loss and stall legitimate retransmit cadence.

USB-CDC: no `tcflush`. UART path is future work.

### 3.9 RFC 6298 RTO

`RttEstimator` in `host_io/rtt.rs`:

```rust
const ALPHA: f64 = 0.125;
const BETA:  f64 = 0.25;
const K:     f64 = 4.0;
const MIN_RTO: Duration = Duration::from_millis(25);
const MAX_RTO: Duration = Duration::from_secs(5);
const G:       Duration = Duration::from_millis(1);

struct RttEstimator {
    srtt:    Option<Duration>,
    rttvar:  Option<Duration>,
    rto:     Duration,         // INIT: MIN_RTO (mandatory; prevents cold-start retransmit storm)
}

impl Default for RttEstimator {
    fn default() -> Self { Self { srtt: None, rttvar: None, rto: MIN_RTO } }
}
```

`update`, `backoff`, `current_rto` per RFC 6298. `MIN_RTO` initial floor is mandatory — without it, the timer-arming path (`sent_at + RTO=0`) would fire immediately on every fresh send.

### 3.10 First-connection sentinel

The empty-`UnackedWindow` short-circuit in §3.5's `update_receive_seq` IS the sentinel. No `bool connection_initialised` flag. Covers fresh-MCU, mid-session-reopen, and stale-RX-from-prior-session cases.

### 3.11 Disconnect / EOF detection

```
match port.read(&mut scratch) {
    Ok(0)                                    => debounce_zero_byte(100ms then Closed),
    Ok(n)                                    => process_bytes(&scratch[..n]),
    Err(e) if e.kind() == TimedOut           => continue,
    Err(e) if e.kind() == Interrupted        => continue,    // contractually retry-safe
    Err(e) if e.kind() == WouldBlock         => continue,    // contractually retry-safe
    Err(e)                                   => transition_closed(e),
}
```

`transition_closed`: flush all `AwaitingResponse` with `TransportError::Closed`; clear `UnackedWindow`; latch `KALICO_ERR_HOST_DISCONNECT` (new host-originated fault code; see §6.11) in FaultLatch; reactor exits loop.

Note: `Ok(0)` is a phantom case for serial ports — short debounce only. The load-bearing path on Linux/macOS USB-CDC unplug is `Err(BrokenPipe)` (POLLHUP) or `Err(Other)` (EIO). See `docs/research/serialport-disconnect-detection.md`.

**Reconnect semantics**: There is no in-place reconnect API. After `transition_closed` exits the reactor loop, the `KalicoHostIo` instance is dead — calls return `TransportError::Closed`. To reconnect, the caller drops the instance and creates a new one via `KalicoHostIo::open*()`. The first-connection sentinel (§3.10) handles whatever stale RX bytes the new port sees from the prior session. Existing `subscribe_*` / `take_*` Receivers from the dropped instance see `mpsc::Disconnected` after the reactor exits.

### 3.12 AwaitingResponse three-layer GC

**Layer 1 — explicit caller abandon.** `CallHandle::Drop` sends `ReactorCommand::Abandon(call_id)` unless `defused`. Reactor walks `AwaitingResponse`, marks matching entry abandoned. Subsequent named-response arrival skips abandoned entries.

**Layer 2 — per-entry dispatcher timeout.** Reactor's GC step walks `AwaitingResponse`; entries past `deadline` are evicted with `TransportError::DispatcherTimeout`. Default 30 s, configurable per call.

**Layer 3 — disconnect-clears-all** (§3.11).

`CallHandle::defuse()` is called on **any** rendezvous-channel completion (success or error) — both imply reactor cleanup is complete. `Abandon(call_id)` for an unknown id is a no-op (defense in depth). `completion.send(...)` returning `Err` (Receiver dropped) is expected behavior, not a logic error: `let _ = completion.send(result);`

## 4. MsgProtoParser

Replaces the Step-6 stub at `rust/kalico-host-rt/src/host_io.rs:567-670`. Lives in `host_io/parser.rs` (canonical msgproto layer) plus `host_io/runtime_events.rs` (kalico-specific structured-event extension).

### 4.1 Identify-blob ingest

```
fn process_identify(blob: &[u8]) -> Result<MsgProtoParser, ParseError>:
    let json_bytes = zlib_decode(blob)?
    let dict: DataDictionary = serde_json::from_slice(&json_bytes)?
    MsgProtoParser::from_dictionary(dict)
```

Pre-identify mode preserved from Step-6 stub: rx loop drops 5-byte ack/nak frames; only honors hardcoded `identify_response offset=%u data=%.*s`. Flips to armed mode atomically when `process_identify` succeeds.

### 4.2 DataDictionary shape

```rust
struct DataDictionary {
    commands:     IndexMap<String, i32>,
    responses:    IndexMap<String, i32>,
    output:       IndexMap<String, i32>,
    enumerations: IndexMap<String, IndexMap<String, EnumValue>>,
    config:       serde_json::Value,
    version:      String,
    app:          String,
    build_versions: Option<String>,
    license:      Option<String>,
}

#[derive(Deserialize)]
#[serde(untagged)]
enum EnumValue {
    Single(i32),
    Range { start: i32, count: i32 },     // confirmed against out/klipper.dict
}
```

**Critical points:**

- **Msgid type is `i32`, not `u32`.** Verifier confirmed kalico's actual `out/klipper.dict` contains 32 negative msgids (range `-32..=-1`) including kalico-critical commands (`kalico_load_curve`, `kalico_clock_sync_request`). `serde_json::from_slice::<HashMap<String, u32>>` would fail outright on first parse.
- **`output` is required.** Kalico's async events (`kalico_credit_freed`, `kalico_fault`, `kalico_status_v6`) are declared via `_DECL_OUTPUT` and use printf-style **positional** wire encoding (§4.8).
- **`enumerations` is required for both decode AND encode.** Pin/bus/chip names (`step_pin=PA1`, `chip=MAX31865`) need name↔int translation at encode time.
- **`IndexMap` preserves JSON insertion order** — load-bearing for enum first-match resolution (§4.3).
- **Mixed JSON shapes within one enum table** are legal (e.g. `pin` contains both `"ADC_TEMPERATURE": 254` and `"PA0": [0, 16]`); `#[serde(untagged)]` handles both per-value.

### 4.3 Format-string grammar

Eight FieldType variants (the exact `MessageTypes` set from `klippy/msgproto.py:150-159`):

```rust
enum FieldType {
    U32,             // %u    PT_uint32         (signed VLQ, masked to u32 on decode)
    I32,             // %i    PT_int32          (signed VLQ, sign-preserved)
    U16,             // %hu   PT_uint16
    I16,             // %hi   PT_int16
    Byte,            // %c    PT_byte           (range-validated to u8 on encode)
    String,          // %s    PT_string         (length-prefixed, byte-clean — NOT null-terminated)
    ProgmemBuffer,   // %.*s  PT_progmem_buffer (required for identify_response)
    Buffer,          // %*s   PT_buffer
}

enum WrappedField {
    Plain(FieldType),
    Enumerated { inner: FieldType, enum_name: String },
}
```

**No `%hc` / `I8` variant** — not a Klipper format code anywhere in the tree.

**Enumeration matching rule**: iterate `enumerations` in JSON insertion order (the `IndexMap`'s natural iteration); accept the first entry where `field_name == enum_name OR field_name.ends_with(&format!("_{}", enum_name))`; `break`. Not longest-suffix matching. A unit test pins this contract against future map-type "cleanups."

### 4.4 Internal layout

```rust
pub struct MsgProtoParser {
    by_msgid:        HashMap<i32, DispatchSpec>,
    by_command_name: IndexMap<String, OutboundSpec>,
    enumerations:    IndexMap<String, EnumTable>,
    static_strings:  HashMap<i32, String>,    // derived from enumerations["static_string_id"]
    config:          serde_json::Value,
    version:         String,
}

enum DispatchSpec {
    Response(ResponseSpec),     // named-parameter wire ("name=value name=value")
    Output(OutputSpec),         // printf-style positional wire
}

struct EnumTable {
    by_name: HashMap<String, i32>,
    by_int:  HashMap<i32, String>,
}
```

**`from_dictionary` collision check** (defense-in-depth against silent-overwrite paths in Python's `dict.update`):

```rust
let mut seen_msgids:    HashSet<i32>    = HashSet::new();
let mut seen_formats:   HashSet<String> = HashSet::new();
let mut seen_msgnames:  HashSet<String> = HashSet::new();    // commands+responses only

// Cross-section accumulator: walk all three sections; reject duplicates anywhere.
for (format, msgid) in dict.commands.iter()
                        .chain(dict.responses.iter())
                        .chain(dict.output.iter()):
    if !seen_msgids.insert(*msgid)            { return Err(ParseError::DuplicateMsgid(*msgid)) }
    if !seen_formats.insert(format.clone())    { return Err(ParseError::DuplicateFormatString(format.clone())) }

for format in dict.commands.keys().chain(dict.responses.keys()):
    let name = first_word(format);
    if !seen_msgnames.insert(name.to_string()) { return Err(ParseError::DuplicateMessageName(name.to_string())) }
```

### 4.5 Enumerations: encode and decode

**Decode** (`WrappedField::Enumerated`): resolves the decoded int through `enumerations[enum_name].by_int.get(&i)`; missing → `format!("?{i}")` (matches `klippy/msgproto.py:142-147`). Output is `MessageValue::String(name)`.

**Encode** (symbolic-only — no numeric fallback):

```rust
fn encode_enum_value(buf, inner, enum_name, value: &str) -> Result<(), ParseError>:
    let int = self.enumerations[enum_name].by_name.get(value)
        .ok_or(ParseError::UnknownEnumValue {
            enum_name: enum_name.into(),
            value:     value.into(),
        })?;
    encode_field_int(buf, inner, *int)
```

No silent numeric fallback — verifier confirmed numeric-string enum keys (e.g. `DECL_ENUMERATION_RANGE("pin", "42", 0, 10)`) are protocol-legal and would silently mis-encode. For deliberate raw-int override, a separate `FieldValue::EnumIntOverride(i32)` variant bypasses the table entirely.

### 4.6 Encoding outbound commands

```rust
pub fn encode(&self, cmd: &str) -> Result<Vec<u8>, ParseError>;
pub fn encode_typed(&self, name: &str, fields: &[(&str, FieldValue)]) -> Result<Vec<u8>, ParseError>;

pub enum FieldValue<'a> {
    U32(u32),
    I32(i32),
    U16(u16),
    I16(i16),
    Byte(u8),
    String(&'a str),
    Buffer(&'a [u8]),
    EnumName(&'a str),       // symbolic
    EnumIntOverride(i32),    // explicit raw-int escape hatch
}
```

Per-wire-type variants make range errors unrepresentable at the call site (mirrors Klipper's `PT_*` dispatch).

**Buffer hex convention**: raw hex digits (`data=0123abcd`), no quotes, no `b'...'` prefix — mirrors `klippy/msgproto.py::_parse_buffer`'s `int(value, 16)`. Debug-render is a separate formatter (Python `repr(bytes)`-equivalent), not the same as encoder input.

### 4.7 Decoding inbound packets

The runtime decode path produces a tagged `DecodedFrame` carrying both the dispatch category and the structured fields the §3.6 receive flow needs:

```rust
pub enum DecodedFrame {
    Response { name: String, params: MessageParams },
    Output   { name: String, params: MessageParams },
}

fn decode(&self, packet: &[u8]) -> Result<DecodedFrame, ParseError>:
    if packet.len() < MESSAGE_MIN { return Err(ShortFrame) }
    let body = &packet[MESSAGE_HEADER_SIZE..packet.len() - MESSAGE_TRAILER_SIZE]
    if body.is_empty() { return Err(EmptyBody) }

    let (msgid_signed, n) = decode_vlq_i32(body)?
    let dispatch = self.by_msgid.get(&msgid_signed).ok_or(UnknownMsgid(msgid_signed))?

    match dispatch {
        DispatchSpec::Response(spec) => {
            let (name, params) = self.decode_response(&body[n..], spec)?;
            Ok(DecodedFrame::Response { name, params })
        }
        DispatchSpec::Output(spec)   => {
            let (name, params) = self.decode_output(&body[n..], spec)?;
            Ok(DecodedFrame::Output { name, params })
        }
    }
```

`decode_response` walks `spec.fields` positionally; per-FieldType normalization in `decode_field`:
- `%u`/`%hu`/`%c` → `MessageValue::U32` via `(raw_i64 as u32)` (matches Python's `& 0xFFFFFFFF` mask).
- `%i`/`%hi` → `MessageValue::I32`.
- `%s`/`%*s`/`%.*s` → `MessageValue::Bytes` via 1-byte length + raw bytes. **No null terminator on the wire** — `klippy/msgproto.py::PT_string.encode` writes `[len, bytes...]`.

`WrappedField::Enumerated` post-processes `U32` into `MessageValue::String(resolved_name)`.

`decode_output` walks the OutputSpec format positionally per `klippy/msgproto.py::lookup_output_params`. For canonical-recoverable formats (every `%`-code is preceded by `name=`, kalico's convention for all four async events), it synthesizes named MessageParams via field-name recovery and returns `(format_first_word, named_params)` — e.g. `("kalico_credit_freed", {retired_through_segment_id: U32(N), free_slots: U32(K)})`. For free-form formats (no `name=%type` recovery), it falls back to the canonical Python shape: `("#output", {"#msg": MessageValue::String(formatted), "#format": MessageValue::String(<original format string>)})`. The `#format` pseudo-field is required because §4.8's `RuntimeEvent::lift` does not have access to the parser's format-string table (`lift` takes `(name, MessageParams)` only); without `#format` the lifted `UnknownOutput.format` would be the literal `"#output"` routing tag rather than the firmware-side format string the operator needs to interpret the `#msg`. The `#format` propagation also drives free-form dispatch parsing at `MsgProtoParser::from_dictionary` time: format strings whose `parse_format_string` recovery fails (`MalformedField` / `UnknownFormatCode` from a non-`name=%type` token) re-parse via positional `%`-code extraction (`extract_free_form_field_types`) and the resulting `OutputSpec` is marked `is_free_form: true`. The free-form branch never causes `from_dictionary` to fail on otherwise-valid output declarations. See §4.8 for the canonical Python-equivalent path used by the Phase-0 differential test.

### 4.8 OutputFormat: two-layer architecture

Canonical Python `OutputFormat.parse` returns `{"#msg": <formatted_string>}` with `name = "#output"` (hardcoded class attribute). It does NOT extract per-field structured names. The Rust port has two distinct decode paths called from different contexts:

**Production runtime path** (§4.7's `decode_output`): synthesizes named `MessageParams` from positional values via format-string `name=%type` recovery, returns `(format_first_word, named_params)`. Called from §3.6 reactor receive flow. This is the canonical-runtime path; subscribers see typed `RuntimeEvent` variants downstream.

**Phase-0 differential path** (`host_io/parser.rs::decode_output_canonical`): produces `("#output", {"#msg": formatted_string})` exactly matching Python's `debugformat % tuple(out)` (including `%c` rendering as decimal-int and `%s`/`%*s`/`%.*s` rendering via Python `repr(bytes)`-equivalent). Invoked ONLY by the Phase-0 differential test (§4.13) which compares Rust output byte-for-byte against Python's `OutputFormat.parse`. Not used at runtime.

Both paths share the same positional decode internals (one pass, no double-decode); they differ only in surface presentation — the runtime path keeps structured fields, the differential path collapses to `#msg`.

**Layer B — Structured-event extension** (`host_io/runtime_events.rs`): emits typed `RuntimeEvent` values. Takes `(name, MessageParams)` uniformly so it works regardless of whether the source frame was decoded via the response-path (`DispatchSpec::Response`) or the output-path (`DispatchSpec::Output`):

```rust
pub enum RuntimeEvent {
    CreditFreed(CreditFreedEvent),
    Fault(FaultEvent),
    Status(StatusEvent),
    Trace(TraceEvent),
    UnknownOutput { format: String, msg: String },
}

impl RuntimeEvent {
    /// Lift a decoded (name, params) tuple to a typed RuntimeEvent.
    /// Routes by message name, irrespective of source category.
    pub fn lift(name: &str, params: MessageParams) -> Self {
        match name {
            "kalico_credit_freed"  => Self::CreditFreed { ... },
            "kalico_fault"         => Self::Fault { ... },
            "kalico_status_v6"     => Self::Status { ... },
            "kalico_trace"         => Self::Trace { ... },
            _ => Self::UnknownOutput {
                // Free-form decode_output (§4.7) stashes the firmware-side
                // format string in `#format`; structured outputs that fall
                // through to catch-all (no typed branch matched) lack it,
                // so we surface the routing `name` as the format. Either
                // way the operator gets a non-empty discriminator.
                format: params.try_get_str("#format")
                    .map(str::to_string)
                    .unwrap_or_else(|| name.to_string()),
                msg: params.try_get_str("#msg").unwrap_or("").to_string(),
            },
        }
    }
}
```

**Why category-agnostic**: even after the §1.1 item 11 source-side fix lands and `kalico_fault` is properly registered as `output`, this defense-in-depth shape means a future regression (any `kalico_*` async event accidentally registered via `sendf`/`_DECL_ENCODER`) would still route correctly through `RuntimeEvent::lift`. The build-time invariant check (item 11) catches such regressions at firmware-build time; host robustness catches them at runtime if the build-time check is ever bypassed.

Both response-path and output-path frames flow through the same `lift(name, params)` entry. The distinction is upstream: §4.7's `decode` returns `DecodedFrame::Response { name, params }` or `DecodedFrame::Output { name, params }` per the `DispatchSpec` tag. §3.6 routes Response frames through AwaitingResponse-match-then-lift, and Output frames straight to lift (output frames are unsolicited; never match an AwaitingResponse entry). In both cases lift sees `(name, params)` of identical shape — the structured form built by §4.7 — and routes by name to the typed `RuntimeEvent` variants.

**Free-form output handling**: any output format whose `name=%type` recovery fails (e.g. future klipper firmware debug `output("...%s...")` traces) becomes `RuntimeEvent::UnknownOutput`. No silent failure, no parse error.

The leading-token-as-event-name behavior is a **kalico runtime convention**, not part of the canonical msgproto contract. Spec wording explicitly distinguishes the two layers.

### 4.9 Static-string reassembly

`enumerations["static_string_id"]` (flat int → name map per `scripts/buildcommands.py:127`) drives `WrappedField::Enumerated` decode for `static_string_id=%hu` fields. Resolved name surfaces as `MessageValue::String("ADC out of range")`.

### 4.10 VLQ encoder/decoder hardening

`encode_vlq` returns `Err(ParseError::OutOfRange)` for values outside `[i32::MIN, u32::MAX]` (no debug-assert + silent-truncate). `decode_vlq` preserved as-is from Step-6 stub (sign-extends from bit 31; per-FieldType normalization in §4.7 handles signedness).

### 4.11 MessageValue

```rust
pub enum MessageValue {
    I32(i32),
    U32(u32),
    U64(u64),
    Bytes(Vec<u8>),
    String(String),    // for %s text fields AND resolved enum names
}
```

`try_get_str(&self, k: &str) -> Option<&str>` reads `String` directly; for `Bytes`, returns bytes-as-UTF-8 if valid (else `None`). If a future caller wants typed enum semantics, `TryFrom<&str>` at the consumer boundary; msgproto layer stays untyped.

### 4.12 Error model

```rust
#[derive(Debug)]
pub enum ParseError {
    Zlib(io::Error),
    Json(serde_json::Error),
    EmptyFormat, EmptyCommand, EmptyBody,
    MalformedField, MalformedArg,
    UnknownCommand(String),
    UnknownMsgid(i32),
    BadMsgid,
    UnknownFormatCode(String),
    UnknownEnumName(String),
    UnknownEnumValue { enum_name: String, value: String },
    MissingField(String),
    OutOfRange { value: i64, range: &'static str },
    ShortFrame, Truncated, BadVlq, BadHex(String),
    DuplicateMsgid(i32),
    DuplicateFormatString(String),
    DuplicateMessageName(String),
}

impl From<ParseError> for TransportError {
    fn from(e: ParseError) -> Self { TransportError::Parse(e.to_string()) }
}
```

Decode errors during reactor operation are logged at `warn` level and the frame is dropped — they don't trigger Closed state (dictionary version skew is recoverable).

### 4.13 Testing strategy

**Prerequisite**: regenerate `out/klipper.dict`. The currently-committed dictionary is stale — it predates Step 7-B's per-axis-handle refactor (still has `weights=%*s` on `kalico_load_curve` and single-handle `kalico_push_segment`). Phase A's first step is to rebuild firmware (`make` against the kalico target) and commit the regenerated `out/klipper.dict`. Differential tests pin against this regenerated artifact.

**Phase 0** (during dev, gated): Bootstrap differential against `klippy/msgproto.py`. Test rig at `tests/parser_python_diff.rs` invokes Python via `std::process::Command`. Gating:

```toml
# rust/kalico-host-rt/Cargo.toml
[features]
default = []
python-bridge    = ["dep:pyo3"]      # production runtime PyO3 shim, separate concern
python-diff-test = []                # Phase-0 differential test gate
```

```rust
// rust/kalico-host-rt/tests/parser_python_diff.rs
#![cfg(feature = "python-diff-test")]
```

CI lane (mirrors existing `runtime/test-injection` precedent): `cargo test -p kalico-host-rt --features python-diff-test`.

Differential tests compare against Python for these wire surfaces: `kalico_push_segment`, `kalico_push_response`, `kalico_clock_sync_request`, `kalico_clock_sync_response`, `kalico_load_curve`, `kalico_load_curve_response`, `kalico_stream_arm`, `kalico_stream_arm_response`, plus async outputs via Layer A canonical form.

**Phase 1** (post-cutover): replace Python differential with H723 capture corpus at `rust/kalico-host-rt/tests/captures/`. Replay-based regression test runs by default.

**Property fuzzing**: `proptest` exercises encode/decode round-trip under arbitrary `MessageParams` matching a fixed spec. Always-on.

**Layer B testing**: Rust-side unit tests with hand-built `(format_string, positional_values)` fixtures asserting correct `RuntimeEvent` variants (separate from the Phase-0 canonical-Layer-A differential).

## 5. Transport trait + call() API + subscriber registration

### 5.1 Public surface

```rust
pub trait Transport: Send + Sync {
    fn call(
        &self,
        cmd: &str,
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError>;

    fn call_typed(
        &self,
        name: &str,
        args: &[(&str, FieldValue<'_>)],
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError>;
}

#[derive(Debug)]
pub enum SubscribeError {
    AlreadySubscribed { channel: &'static str },
    Closed,
}
```

`SubscribeError` is dedicated (not a `TransportError` variant) — wiring-time programmer errors are a different audience than runtime transport faults.

**`TransportError` extension.** This spec adds one new variant to the existing `TransportError` enum (currently `Io | Timeout | Closed | Parse` at `rust/kalico-host-rt/src/transport.rs:14-25`):

```rust
pub enum TransportError {
    Io(io::Error),
    Timeout,                // caller-side: caller's recv_timeout fired
    DispatcherTimeout,      // NEW: reactor-side: AwaitingResponse entry past deadline (§3.12 layer 2)
    Closed,
    Parse(String),
}
```

The two timeout variants are deliberately distinct:
- `Timeout` is ambiguous: caller's `recv_timeout(timeout)` fired, but the reactor may still hold the `AwaitingResponse` entry (it gets cleaned up via the abandon-on-drop path in §3.12 layer 1, but there's a window where it's still alive).
- `DispatcherTimeout` is deterministic: the reactor's GC step (§3.12 layer 2) observed `now > deadline` and evicted the entry. The caller is guaranteed no completion will arrive on the dropped channel.

Most callers can treat them identically (both surface as "operation didn't complete in time"); subsystems that distinguish them get explicit deterministic semantics.

### 5.2 Concrete `KalicoHostIo` struct

All mutable state lives inside the reactor thread; public struct holds only thread-safe handles:

```rust
pub struct KalicoHostIo {
    submission_tx:   mpsc::Sender<ReactorCommand>,
    next_call_id:    AtomicU64,
    reactor_handle:  Option<JoinHandle<()>>,
    status_snapshot: Arc<ArcSwap<StatusSnapshot>>,    // shared with reactor; lock-free public reads
}

impl Drop for KalicoHostIo {
    fn drop(&mut self) {
        let _ = self.submission_tx.send(ReactorCommand::Shutdown);
        if let Some(h) = self.reactor_handle.take() {
            let _ = h.join();    // non-panicking; reactor panic logged, not propagated
        }
    }
}
```

`mpsc::Sender<T>: Sync` since Rust 1.72; workspace MSRV is 1.85. `next_call_id.fetch_add(1, Ordering::Relaxed)` — Relaxed suffices for monotonic uniqueness.

### 5.3 `KalicoHostIoConfig`

```rust
pub struct KalicoHostIoConfig {
    pub trace_capacity:        usize,             // bounded ring size (e.g. 256)
    pub default_call_timeout:  Duration,
    pub identify_timeout:      Duration,
    // ...
}

pub fn open_with_config(path: &str, baud: u32, config: KalicoHostIoConfig) -> Result<Self, TransportError>;
```

### 5.4 Subscriber registration API

```rust
impl KalicoHostIo {
    pub fn attach_credit_counter(&self, counter: Arc<CreditCounter>);
    pub fn subscribe_fault(&self) -> Result<mpsc::Receiver<FaultEvent>, SubscribeError>;
    pub fn take_trace_subscription(&self) -> Result<mpsc::Receiver<TraceEvent>, SubscribeError>;
    pub fn take_runtime_event_subscription(&self) -> Result<mpsc::Receiver<RuntimeEvent>, SubscribeError>;
    pub fn take_host_event_subscription(&self) -> Result<mpsc::Receiver<HostEvent>, SubscribeError>;
    pub fn status(&self) -> Arc<StatusSnapshot> {
        self.status_snapshot.load_full()    // lock-free
    }
}
```

`subscribe_fault` allows replay-on-subscribe (latched cell sends to new Receiver if non-empty; §6.3). `take_*` calls are one-shots — second call returns `Err(AlreadySubscribed)`. `take_trace_subscription` is **re-armable post-disconnect** (§6.4): if a subscriber's Receiver dies, the slot becomes available again, and a `HostEvent::TraceSubscriberDisconnected` surfaces the event.

`attach_credit_counter` is the special case (silent replace fine — wiring-time, `CreditCounter` is `Arc`-shared).

**Implementation pattern**: subscriber-registration calls flow through the submission mpsc just like other reactor commands, but they need a reply channel so the reactor can synchronously report `AlreadySubscribed` / `Closed` back to the caller before the public function returns. Each `Subscribe*` variant carries both the event sender and a one-shot reply sender:

```rust
fn subscribe_fault(&self) -> Result<mpsc::Receiver<FaultEvent>, SubscribeError> {
    let (event_tx, event_rx) = sync_channel::<FaultEvent>(1);
    let (reply_tx, reply_rx) = sync_channel::<Result<(), SubscribeError>>(1);
    self.submission_tx
        .send(ReactorCommand::SubscribeFault { sender: event_tx, reply: reply_tx })
        .map_err(|_| SubscribeError::Closed)?;
    reply_rx.recv().map_err(|_| SubscribeError::Closed)??;
    Ok(event_rx)
}
```

The reactor processes the command, runs validation (`if self.fault_latch.subscriber.is_some() { Err(AlreadySubscribed) }`), and replies on `reply_tx`. The public function blocks on `reply_rx` and propagates the result.

### 5.5 Internal flow: `call`

```rust
fn call(&self, cmd, expected_response_name, timeout) -> Result<MessageParams, TransportError>:
    let call_id = self.next_call_id.fetch_add(1, Ordering::Relaxed);
    let (tx, rx) = sync_channel::<Result<MessageParams, TransportError>>(1);

    self.submission_tx.send(ReactorCommand::Submit { call_id, cmd: cmd.to_string(),
        expected_response_name: expected_response_name.to_string(),
        completion: tx, deadline: Instant::now() + timeout })
        .map_err(|_| TransportError::Closed)?;

    let handle = CallHandle { call_id, submission_tx: self.submission_tx.clone(), defused: false };

    match rx.recv_timeout(timeout) {
        Ok(result) => { handle.defuse(); result }    // any completion → defuse
        Err(RecvTimeoutError::Timeout) => Err(TransportError::Timeout),
        Err(RecvTimeoutError::Disconnected) => Err(TransportError::Closed),
    }
```

Defuse-on-any-completion (success OR error) — both imply reactor cleanup is complete. `Drop` only fires `Abandon` on timeout/disconnected paths.

### 5.6 Reactor-side completion send

`completion.send(...)` returning `Err` is expected (caller dropped Receiver). Pattern:

```rust
let _ = completion.send(result);
awaiting_response.remove(call_id);
```

Optional trace-level log for observability. The reactor MUST NOT panic, MUST NOT propagate this to a structured error channel, and MUST complete the entry removal regardless of send outcome.

`Abandon(call_id)` for an unknown id is a no-op.

### 5.7 ReactorCommand enum

```rust
enum ReactorCommand {
    Submit { call_id, cmd, expected_response_name, completion, deadline },
    SubmitTyped { call_id, payload, expected_response_name, completion, deadline },
    Abandon(u64),
    AttachCreditCounter(Arc<CreditCounter>),
    SubscribeFault {
        sender: SyncSender<FaultEvent>,
        reply:  SyncSender<Result<(), SubscribeError>>,
    },
    SubscribeTrace {
        sender: SyncSender<TraceEvent>,
        reply:  SyncSender<Result<(), SubscribeError>>,
    },
    SubscribeRuntimeEvents {
        sender: SyncSender<RuntimeEvent>,
        reply:  SyncSender<Result<(), SubscribeError>>,
    },
    SubscribeHostEvents {
        sender: SyncSender<HostEvent>,
        reply:  SyncSender<Result<(), SubscribeError>>,
    },
    Shutdown,
}
```

### 5.8 Migration of existing call sites

`producer.rs::push_segment`:
```rust
let resp = io.call(&cmd, "kalico_push_response", timeout)?;
```

`stream.rs::arm_all_mcus`: same pattern; per-MCU monotonic `request_id` (replacing hardcoded `request_id=1`).

### 5.9 `request_id` correlation (clock_sync sanity check)

```rust
let request_id = self.next_request_id();    // per-MCU monotonic u32

let resp = io.call(
    &format!("kalico_clock_sync_request request_id={request_id} host_send_time_lo=0 host_send_time_hi=0"),
    "kalico_clock_sync_response",
    CLOCK_SYNC_REQUEST_TIMEOUT,
)?;

let echoed = resp.try_get_u32("request_id")
    .ok_or_else(|| ArmError::Transport(TransportError::Parse(
        "kalico_clock_sync_response missing request_id field".into(),
    )))?;
if echoed != request_id {
    return Err(ArmError::Transport(TransportError::Parse(
        format!("clock_sync request_id mismatch: sent {request_id}, got {echoed}"),
    )));
}
```

Mismatch produces a hard fault rather than silent corruption. Scoped to clock_sync only — other kalico commands don't carry `request_id`; firmware-side change deferred.

### 5.10 ArmError::QualityGate detail

```rust
pub enum ArmError {
    DeadlineMissed,
    QualityGate { mcu_index: usize, subgate: QualityGateFailure },
    CrossMcuDesync { mcu_a: usize, mcu_b: usize, ratio_offset: f64 },
    McuRejected(i32),
    Transport(TransportError),
}

#[derive(Debug)]
pub enum QualityGateFailure {
    InsufficientWarmup        { samples: usize, required: usize },
    ResidualExceeded          { observed_us: f64, max_us: f64 },
    DriftPpmExceeded          { observed_ppm: f64, max_ppm: f64 },
    LastSampleStale           { age: Duration, max_age: Duration },
    DedicatedSampleStale      { age: Duration, max_age: Duration },
}
```

`ClockSyncEstimator::is_quality_gate_passed` refactored from `-> bool` to `-> Result<(), QualityGateFailure>`. Each of the five sequential checks (`clock_sync.rs:278-297`) returns the matching variant on failure.

### 5.11 Testing strategy

**MockTransport** (`tests/mock_transport.rs`) refactored with controllable pending-call state:

```rust
pub struct MockTransport {
    pending_calls: Mutex<HashMap<u64, MockPendingCall>>,
    next_call_id:  AtomicU64,
    // ...
}

impl MockTransport {
    pub fn complete_call(&self, name: &str, params: MessageParams);
    pub fn drop_pending(&self, name: &str);
    pub fn pending_count(&self) -> usize;    // for leak-only-bug assertions
}
```

Test scenarios:
- **Test 1**: SUT calls; test waits past timeout; SUT drops `CallHandle`; Abandon fires; assert `mock.pending_count() == 0`.
- **Test 2**: two parallel SUT calls; complete first; assert second still pending; complete second; both succeed.
- **Test 3**: Abandon-before-response race; complete after abandon; assert no panic and `pending_count() == 0`.

Real-transport integration test: `#[cfg(feature = "hardware-test")]` against the simulator or H723 bench.

## 6. Per-channel backpressure (EventDispatcher subsystem)

### 6.1 EventDispatcher overview

Lives inside the reactor (single-threaded). On every decoded inbound non-ack frame:

1. FIFO-match `AwaitingResponse` → route to completion (§3.6).
2. Otherwise lift to typed `RuntimeEvent` (§4.8 Layer B).
3. Dispatch to per-variant channel (§6.2-§6.7).

In addition, the reactor itself emits **host-internal events** (§6.8) — observable conditions on the host side (subscriber overflow, subscriber disconnect, etc.) that aren't MCU-originated faults. Routed via a separate host-event channel.

### 6.2 Credit coalescer (snap-to-authoritative)

`kalico_credit_freed.free_slots` is the **MCU's authoritative current count** of free queue slots, NOT a delta. The host snaps its counter to this value (clamped to capacity); idempotent under retransmit.

```rust
fn dispatch_credit_freed(&self, event: CreditFreedEvent):
    if let Some(counter) = self.credit_counter.as_ref():
        // Snap to MCU snapshot — idempotent under retransmit. Stale events
        // bounded by credit_epoch reset (CreditCounter::on_epoch_change).
        // See credit.rs:80-88 for the API contract.
        counter.on_credit_freed(event.free_slots);
```

The firmware emits `free_slots = Q_N - queue_depth` (`runtime_tick.c:259-260`) — current free slots, not "newly freed since last event." Additive accumulation would overcount under retransmit and trigger `kalico_push_response result != 0` faults.

### 6.3 Fault latch (with replay-on-subscribe + edge-upgrade)

```rust
pub struct FaultEvent {
    pub fault_code:    u16,
    pub fault_detail:  u32,
    pub segment_id:    u32,
    pub synthesized:   bool,    // true if derived from periodic status, not edge
}

struct FaultLatch {
    cell:        Option<FaultEvent>,
    subscriber:  Option<SyncSender<FaultEvent>>,
}

impl FaultLatch {
    /// Edge-driven dispatch (called from kalico_fault frame handler) OR
    /// status-derived synthesis (§6.5). Edge events upgrade synthesized
    /// latch in-place (preserving exact fault_segment_id).
    fn dispatch(&mut self, event: FaultEvent) {
        let upgrade = self.cell.as_ref()
            .map(|c| c.synthesized && !event.synthesized)
            .unwrap_or(false);
        if self.cell.is_none() || upgrade {
            self.cell = Some(event.clone());
            if let Some(tx) = &self.subscriber {
                let _ = tx.send(event);
            }
        }
    }

    fn subscribe(&mut self, tx: SyncSender<FaultEvent>) -> Result<(), SubscribeError> {
        if self.subscriber.is_some() {
            return Err(SubscribeError::AlreadySubscribed { channel: "fault" });
        }
        if let Some(latched) = &self.cell {
            let _ = tx.send(latched.clone());    // replay-on-subscribe
        }
        self.subscriber = Some(tx);
        Ok(())
    }
}
```

### 6.4 Trace ring (re-armable + host-event diagnostics)

```rust
struct TraceRing {
    capacity:                  usize,
    sticky_overflow:           bool,
    subscriber:                Option<SyncSender<TraceEvent>>,
    drop_count_since_event:    u64,
    host_event_tx:             SyncSender<HostEvent>,
}

impl TraceRing {
    fn dispatch(&mut self, mut event: TraceEvent) {
        if self.sticky_overflow {
            event.flags |= TraceEventFlag::OVERFLOW;
            self.sticky_overflow = false;
        }

        match self.subscriber.as_ref() {
            Some(tx) => match tx.try_send(event) {
                Ok(_) => {}
                Err(TrySendError::Full(_)) => {
                    self.sticky_overflow = true;
                    self.drop_count_since_event += 1;
                    let _ = self.host_event_tx.try_send(HostEvent::TraceSubscriberOverflow {
                        dropped_count: self.drop_count_since_event,
                        at: Instant::now(),
                    });
                }
                Err(TrySendError::Disconnected(_)) => {
                    // Subscriber's Receiver dropped (e.g., panic). Clear the slot;
                    // take_trace_subscription is now re-armable. Emit host diagnostic.
                    self.subscriber = None;
                    self.drop_count_since_event = 0;
                    let _ = self.host_event_tx.try_send(HostEvent::TraceSubscriberDisconnected {
                        at: Instant::now(),
                    });
                }
            },
            None => self.drop_count_since_event += 1,
        }
    }

    fn on_reattach(&mut self) {
        if self.drop_count_since_event > 0 {
            let _ = self.host_event_tx.try_send(HostEvent::TraceSubscriberReattached {
                events_lost_during_gap: self.drop_count_since_event,
                at: Instant::now(),
            });
            self.drop_count_since_event = 0;
        }
    }
}
```

**`KALICO_FAULT_TRACE_OVERFLOW`** (`error.rs:62`) remains exclusively the MCU-side fault code (firmware ISR observes TraceRing overwrite at `engine.rs:707`, latched into `shared.last_error` at `reclaim.rs:114`, emitted as `kalico_fault` by `runtime_tick.c:272`). Host-side overflow surfaces as `HostEvent::TraceSubscriberOverflow`; the two failure modes have different root causes and remediations and must not share a code.

### 6.5 Status-frame handler with missed-edge synthesis

```rust
fn handle_status_frame(&mut self, frame: &KalicoStatusV6) {
    // 1. Update snapshot for cheap public reads (§6.6).
    self.status_snapshot.store(Arc::new(StatusSnapshot::from(frame)));

    // 2. Backstop: if MCU is in FAULT and we never observed an edge event,
    //    the kalico_fault frame was lost (e.g. across a USB hiccup).
    //    Synthesize from the periodic frame's diagnostic fields.
    //    Idempotent: §6.3 latch's `is_none()` guard ensures at most once
    //    per session. Late edge arrival upgrades the latch in-place.
    if frame.engine_status == EngineStatus::Fault
        && self.fault_latch.cell.is_none()
    {
        let synthesized = FaultEvent {
            fault_code:   frame.last_fault,
            fault_detail: frame.fault_detail,
            segment_id:   frame.current_segment_id,    // approximate; ≠ fault_segment_id at edge
            synthesized:  true,
        };
        self.fault_latch.dispatch(synthesized);
    }
}
```

Status frames carrying `last_fault != 0` do not re-trigger fault dispatch when a fault has already been latched.

### 6.6 Status snapshot via ArcSwap

```rust
// Reactor publishes:
self.status_snapshot.store(Arc::new(snapshot));

// Public callers read (lock-free):
pub fn status(&self) -> Arc<StatusSnapshot> {
    self.status_snapshot.load_full()
}
```

`arc_swap::ArcSwap` provides lock-free reads (single atomic load + Arc clone). At 10 Hz writes from the reactor with arbitrary readers from public API, the read-heavy snapshot pattern is the canonical ArcSwap target. Avoids RwLock's reader-guard lifetime/fairness footguns.

### 6.7 Runtime-event catch-all (exclusive to unhandled variants)

```rust
fn dispatch_runtime_event(&mut self, event: RuntimeEvent):
    match event:
        RuntimeEvent::CreditFreed(e)  => self.dispatch_credit_freed(e),
        RuntimeEvent::Fault(e)        => self.fault_latch.dispatch(e),
        RuntimeEvent::Trace(e)        => self.trace_ring.dispatch(e),
        RuntimeEvent::Status(e)       => self.handle_status_frame(&e),
        RuntimeEvent::UnknownOutput(_) => self.runtime_event_dispatcher.dispatch(event),
```

Typed channels (Fault, Trace, etc.) have one authoritative delivery surface — their typed channel — NOT duplicate fan-out. Catch-all is exclusive to `UnknownOutput` and any future variants without a dedicated typed channel.

**Reliability tier per surface:**
- **Typed channels** (`subscribe_fault`, `take_trace_subscription`): bounded, fail-loud (or latched), safety-relevant.
- **Runtime-event catch-all** (`take_runtime_event_subscription`): best-effort, may drop on backpressure, diagnostic.

**Variant-promotion policy**: promoting a `RuntimeEvent` variant from catch-all to a typed channel is a breaking change to the event API. Catch-all consumers depending on that variant must migrate.

### 6.8 Host-event channel (parallel to runtime-event)

Host-internal events (host-side overflow, subscriber lifecycle) are routed via a dedicated channel separate from the runtime-event catch-all:

```rust
pub enum HostEvent {
    TraceSubscriberOverflow {
        dropped_count: u64,
        at:            Instant,
    },
    TraceSubscriberDisconnected { at: Instant },
    TraceSubscriberReattached { events_lost_during_gap: u64, at: Instant },
    // Future: runtime-event catch-all overflow, reactor health stats, etc.
}
```

Bounded mpsc internally; drop-newest on overflow with warn-log.

### 6.9 EventDispatcher composition

```rust
struct EventDispatcher {
    credit_counter:           Option<Arc<CreditCounter>>,
    fault_latch:              FaultLatch,
    trace_ring:               TraceRing,
    status_snapshot:          Arc<ArcSwap<StatusSnapshot>>,
    runtime_event_dispatcher: RuntimeEventDispatcher,    // catch-all (UnknownOutput-only)
    host_event_dispatcher:    HostEventDispatcher,
}
```

### 6.10 Channel-policy summary

| Channel | Policy | Source | Reliability tier |
|---|---|---|---|
| Credit | Snap-to-authoritative (idempotent) | MCU `kalico_credit_freed` | Lossless under retransmit (idempotent) |
| Fault | Latched cell + replay-on-subscribe + status synthesis | MCU `kalico_fault` edge OR `kalico_status_v6` backstop | Lossless once latched |
| Trace | Bounded, drop-newest, sticky overflow flag, re-armable | MCU `kalico_trace` | Best-effort |
| Status snapshot | ArcSwap latest-value | MCU `kalico_status_v6` 10 Hz | Latest-only |
| Runtime-event catch-all | Bounded, drop-newest, warn-log | MCU `RuntimeEvent::UnknownOutput` (exclusive) | Best-effort |
| Host events | Bounded, drop-newest, warn-log | Reactor-internal diagnostics | Best-effort |

### 6.11 New host-originated fault codes

Step 7-C-io introduces three new fault codes in `rust/runtime/src/error.rs`'s taxonomy, distinct from MCU-originated codes. The existing taxonomy uses the `KALICO_*` const prefix (e.g. `KALICO_OK = 0`, plus `KALICO_ERR_*` constants for fault codes); the new host codes follow the `KALICO_ERR_*` convention with a `_HOST_` infix to disambiguate origin:

| Constant | `FaultCode` variant | Meaning | Trigger |
|---|---|---|---|
| `KALICO_ERR_HOST_DISCONNECT` | `FaultCode::HostDisconnect` | Serial port unplugged or unreadable | §3.11 disconnect detection |
| `KALICO_ERR_HOST_RETRANSMIT_EXHAUSTED` | `FaultCode::HostRetransmitExhausted` | A frame hit `MAX_RETRY_COUNT = 8` | §3.8 retransmit cap |
| `KALICO_ERR_HOST_DISPATCHER_TIMEOUT` | `FaultCode::HostDispatcherTimeout` | An `AwaitingResponse` entry exceeded its per-entry deadline | §3.12 layer 2 GC |

These land in the same `error.rs` taxonomy as MCU-originated codes (`KALICO_ERR_TRACE_OVERFLOW`, `KALICO_ERR_CROSS_MCU_DESYNC`, etc.); operators disambiguate origin from the `_HOST_` infix in the const name.

### 6.12 Known limitations

- **Recursive host-event overflow**: if `TraceRing::dispatch` overflows AND `host_event_tx` is also full, the diagnostic event about the trace overflow is silently dropped. Trace is best-effort by design; not safety-critical.
- **Cross-subscription re-armability consistency**: §6.4 makes trace re-armable post-disconnect; other one-shot subscriptions (`subscribe_fault`, `take_runtime_event_subscription`, `take_host_event_subscription`) currently retain strict one-shot semantics. Consider unifying during implementation review.
- **`KALICO_ERR_HOST_*` codes are not currently in `error.rs`** — they're introduced by this spec and need to land alongside the implementation. Phase A's first commit should add them to the taxonomy (in addition to the firmware-side async-event registration fix from §1.1 item 11).

## 7. Out of scope, sibling steps, and open follow-ups

### 7.1 Sibling steps (forward references)

- **Step 7-C-bridge** — Python ↔ Rust integration so existing Klipper configs route motion through kalico's planner. Depends on this step.
- **Step 7-D** — Hardware bring-up, F4x integration, M1/M2/M3 soaks, calibration, physical first print. Depends on 7-C-io + 7-C-bridge.
- **Step 13** — Compatibility layer (offline G-code text → G5-only normalizer). Independent.
- **Step 14** — EtherCAT backend transport. Independent of 7-C-io but reuses `Transport` trait shape.

### 7.2 Future work outside Step 7-C-io scope

- **Replacing `klippy/msgproto.py`** — far post-7-C; klippy retains it for non-motion MCU subsystems until those subsystems are migrated to Rust.
- **Concurrent senders on a single `Transport`** — would require additive API (multi-slot `AwaitingResponse` per-name, per-call request_id correlation across the wire).
- **Klipper `notify_id`-style firmware-level RPC correlation** — adds `request_id=%u` to every command in firmware. Defense-in-depth; not currently needed.
- **UART-specific retransmit handling** (`tcflush(TCOFLUSH)`) — if/when UART becomes a supported transport.
- **Cross-subscription re-armability unification** — see §6.12.

## 8. Adversarial review history

This spec was developed through iterative codex review + verifier dispatch passes per section. Each round surfaced specific must-fix and should-fix items that were folded into the next revision.

### 8.1 Pass summary

| Section | Codex passes | Verifier passes | Must-fix found | Should-fix found | Notable corrections |
|---|---|---|---|---|---|
| Concurrency model (early brainstorm) | 0 | 3 (α/β/γ) | — | — | Killed γ as non-decision; α had 4+ protocol-correctness failures; β workable but cancellation-unsafe. Settled on δ (poll-reactor, mirror `serialqueue.c`). |
| Section 3 (wire protocol) | 1 | 11 | 6 | 3 | Window=12 not 16; ack-removal `<` not `<=`; NAK = duplicate ack with `ignore_nak_seq` damper; both NAK + timeout-driven retransmit; RFC 6298 RTO with `MIN_RTO` floor; 64-bit absolute counter; trait surface change; first-connection sentinel; per-channel backpressure detail. |
| Section 4 (MsgProtoParser) | 2 | 18 | 7 | 5 | Drop `%hc` (not a Klipper format code); add `%.*s`; add `output` and `enumerations` to DataDictionary; msgid type `i32` not `u32` (kalico's `out/klipper.dict` already has 32 negative msgids); `%s` is length-prefixed, not null-terminated; raw-hex encode convention; symbolic-only enum encode; `IndexMap` for insertion-order matching; cross-section format-string collision check; two-layer OutputFormat architecture. |
| Section 5 (Transport API) | 1 | 8 | 3 | 5 | `KalicoHostIo` field shape (only thread-safe handles); defuse-on-any-completion; reactor send-Err graceful; subscribe_fault replay-on-subscribe; `Result<Receiver, SubscribeError>` return; trace capacity at construction time; per-MCU `request_id` correlation with sanity check; MockTransport controllable pending-call state. |
| Section 6 (EventDispatcher) | 1 | 7 | 4 | 3 | Status snapshot field added to public struct; `ArcSwap` over `RwLock`; host-side overflow taxonomy separate from `KALICO_FAULT_TRACE_OVERFLOW`; catch-all exclusive to `UnknownOutput`; trace re-armable post-disconnect; **credit semantics: snap-to-authoritative not additive (real bug)**; status-derived FaultEvent synthesis. |

### 8.2 Notable cross-cutting findings

- **`MsgProtoParser`'s msgid type was `u32`** in Step-6 stub. Verifier confirmed `out/klipper.dict` contains 32 negative msgids today. The Step-6 stub would have failed `serde_json::from_slice` on first parse if the production parser had been wired to it.
- **`%s` length-prefix vs null-terminated**: original proposal said null-terminated based on stub comments; canonical Python (`klippy/msgproto.py:96-102`) confirms length-prefixed and byte-clean. A null-terminated decoder would have misparsed every `%s` field and cascade-misaligned every subsequent field of the same message.
- **OutputFormat dispatch**: original proposal extracted message names from leading whitespace token. Canonical Python uses hardcoded `name = "#output"` and returns `{"#msg": formatted_string}`. Resolved via two-layer architecture (canonical Layer A for Phase-0 differential parity; kalico-specific Layer B for typed events).
- **Credit accounting**: original §6.2 wording specified the credit dispatch as additive sum. The actual `CreditCounter::on_credit_freed` API at `rust/kalico-host-rt/src/credit.rs:80-88` is snap-to-authoritative. Additive under NAK retransmit would have overcounted and triggered MCU-rejection faults.
- **`KALICO_FAULT_TRACE_OVERFLOW`**: original proposal reused this MCU-side fault code for host-side trace ring overflow. Different root causes, different remediations; conflation would have masked diagnostics. Resolved via separate `HostEvent` channel.

## 9. Implementation order

Suggested implementation sequence (each phase produces a green build with the previous still working):

1. **Phase A — Prerequisite hygiene + `MsgProtoParser` foundation.**
   - Source-side fix: convert `runtime_tick.c:282` `sendf("kalico_fault ...")` AND `runtime_tick.c:231` `sendf("kalico_trace ...")` to `output(...)` per §1.1 item 11. Audit any other `kalico_*` async-event emit sites and convert them too.
   - Add a build-time invariant check that all `kalico_*` async event format strings register only via `_DECL_OUTPUT` (parsing the generated dict, or a lint over `runtime_tick.c`).
   - Regenerate `out/klipper.dict` (the committed dict is stale: missing the per-axis-handle refactor from Step 7-B). Commit the regenerated artifact; pin its content hash for canonical reference within 7-C-io.
   - Add `KALICO_ERR_HOST_*` constants and `FaultCode` variants to `rust/runtime/src/error.rs`.
   - Production parser in `host_io/parser.rs` with the eight-variant `FieldType`, full `DataDictionary` shape, `IndexMap` for enumerations, `from_dictionary` collision checks.
   - Layer B `runtime_events.rs` with typed `RuntimeEvent` variants and `lift(name, MessageParams)` taking the uniform shape (works for both response- and output-path inputs).
   - Phase-0 `python-diff-test` differential.
2. **Phase B — `Transport` trait reshape (caller-side surface).**
   - `&self` + `Send + Sync`; `call`/`call_typed`; `CallHandle` with abandon-on-drop (caller-side semantics; reactor-side dispatch lands in Phase C).
   - `TransportError::DispatcherTimeout` variant added to the enum.
   - `SubscribeError` enum.
   - `MockTransport` refactor for testability (controllable pending-call state, `pending_count()` accessor) — this is what Phase B can deliver, since MockTransport doesn't need a reactor.
   - Production `KalicoHostIo` callers (`producer.rs`, `stream.rs`) updated to use the new `call()` API but still wire to the Step-6 stub's reactor-less impl during Phase B (the stub's `&mut self` API gets a `&self` shim that locks internally).
3. **Phase C — Reactor + UnackedWindow + RTO + GC.**
   - Refactor `KalicoHostIo` to the §5.2 struct shape; spin reactor on a dedicated thread.
   - Wire-protocol state machine §3.1-§3.10 (using the §5 trait surface from Phase B).
   - `RttEstimator` with `MIN_RTO` floor.
   - Inline-retransmit chokepoint with two-arm `ignore_nak_seq` and TimeoutDriven-only RTO backoff.
   - **Three-layer `AwaitingResponse` GC** (Phase C, not B — it's reactor-owned state): abandon-on-drop, per-entry dispatcher timeout, disconnect-clears-all.
   - Cut over from the Phase-B `&self` shim to the real reactor-backed implementation.
4. **Phase D — EventDispatcher.**
   - `FaultLatch`, `TraceRing`, `RuntimeEventDispatcher`, `HostEventDispatcher`.
   - Status snapshot via `ArcSwap`.
   - Subscriber registration API.
5. **Phase E — `arm_all_mcus` updates + `ArmError::QualityGate` detail.**
   - Per-MCU monotonic `request_id` with sanity check.
   - `is_quality_gate_passed` → `Result<(), QualityGateFailure>`.
6. **Phase F — split into Step 7-C-io tail + Step 7-D handoff.** Phase F was rescoped after the original "soak on Renode + bench" plan accumulated structural problems (USART2-vs-USB-CDC capture mismatch, Renode pacing breaking wall-clock bounds, one-hour leak detection too short). Replaced with two pieces:
   - **Step 7-C-io tail** (deterministic test battery + `Clock` seam + `tick_once()` extraction). Catches arithmetic, GC, ordering, and edge-case bugs that hardware testing also cannot reliably catch. See `docs/superpowers/specs/2026-05-01-step-7c-io-tail-design.md`. Completed 2026-05-01.
   - **Step 7-D** (hardware bring-up): canonical H723 capture corpus, 24h wall-clock soak, `python-diff-test` retirement (gated on canonical captures per §4.13), USB-CDC byte-sequence fidelity, real unplug semantics, IWDG real-world pacing, Surface-C cycle actuals, optional Renode sim-soak as bench scaffolding.

## 10. References

- `klippy/chelper/serialqueue.c` — Klipper's reference reactor (canonical prior art for §3 wire protocol).
- `klippy/msgproto.py` — canonical Python parser.
- `scripts/buildcommands.py` — data-dictionary generator.
- `src/runtime_tick.c` — kalico-side firmware command/output declarations.
- `docs/superpowers/specs/2026-04-28-step6-comm-protocol-and-sim-fixes-design.md` — Step 6 spec; this step closes its Plan-decision-C deferrals.
- `docs/research/serialport-disconnect-detection.md` — verifier output on disconnect-detection semantics (used in §3.11).
- RFC 6298 — TCP retransmission timer computation.
