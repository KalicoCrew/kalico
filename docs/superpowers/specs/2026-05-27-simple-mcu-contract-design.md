# Simple MCU Contract

**Date:** 2026-05-27
**Branch:** `simple-mcu-contract` (from `sota-motion`)
**Goal:** Radically simplify the MCU contract so the MCU is a dumb per-axis polynomial playback engine. All coordination, kinematic transforms, and synchronization logic live on the host. This is the foundation for host-side multi-MCU synchronization (a later step).

---

## 1. Core Concept

The MCU receives **pieces** — 28-byte cubic polynomial fragments — and plays them back sequentially per axis at the ISR rate (40 kHz on H7, 20 kHz on F4). That is the entire MCU contract.

There are no segments, no curve pool, no handles, no generation tracking, no kinematic transforms, no E-follower arithmetic on the MCU. The MCU does not know what axes exist in the machine, what kinematics are in use, or how many MCUs are in the system. It receives pieces, evaluates polynomials, and drives steppers.

## 2. MCU Data Model

### 2.1 Piece Ring

A single flat circular buffer shared across all axes. Total ring memory is **static, configured at compile time** via Kconfig (`CONFIG_RUNTIME_PIECE_RING_SIZE`). Per-axis ring depth is dynamic: `ring_depth_per_axis = total_ring_memory / (28 × num_axes)`, determined at runtime when the host registers axes.

Each entry is 28 bytes:

| Field | Type | Bytes | Description |
|-------|------|-------|-------------|
| `start_time` | `u64` | 8 | Piece start time in MCU clock cycles |
| `coeffs` | `[f32; 4]` | 16 | Bernstein control points (b0, b1, b2, b3) |
| `duration` | `f32` | 4 | Piece duration in seconds |

Total: **28 bytes per piece** on wire and in storage.

At load time the MCU converts Bernstein → monomial (Horner-friendly) coefficients in place. Velocity coefficients (`vc0 = c1`, `vc1 = 2·c2`, `vc2 = 3·c3`) are **not stored** in the ring — they are computed once per piece transition and cached in per-axis ISR working state (3 × f32 = 12 bytes fixed cost).

Start time is in u64 MCU clock cycles (host converts from absolute time using per-MCU clock estimation). The ISR compares directly against DWT CYCCNT — no floating-point conversion in the time comparison path.

### 2.1.1 Kconfig Changes

**Removed:**
- `CONFIG_RUNTIME_CURVE_POOL_N` — no curve pool
- `CONFIG_RUNTIME_MAX_PIECES_PER_CURVE` — no per-slot piece ceiling

**Added:**
- `CONFIG_RUNTIME_PIECE_RING_SIZE` — total bytes for piece ring storage (default: 63488 on H7, 16384 on F4)

The host queries this value via `RuntimeCapsResponse` and divides by `28 × num_axes` to compute uniform ring depth per axis.

### 2.2 Per-Axis ISR Working State (fixed, not in ring)

- Cached velocity coefficients: 3 × f32 (recomputed on piece transition)
- Current piece start time: u64 (MCU clock cycles)
- Read cursor into ring
- Stepping state: last_step_count, accumulator
- Stepper bindings (which physical steppers to drive)

### 2.3 Per-Axis Configuration (set once at startup)

- Stepping mode: pulse or phase
- Microstep distance (1 / steps_per_mm)
- Stepper bindings: which physical steppers are driven from this axis

Multiple steppers can share the same axis (e.g., 3 Z motors on a Trident all bound to one Z axis, consuming one piece stream).

## 3. Wire Protocol

### 3.1 Message Catalog

**Host → MCU (commands, expect response):**

| Message | Wire ID | Purpose |
|---------|---------|---------|
| `ConfigureAxis` | 0x0030 | Register an axis: stepping mode, microstep_distance, stepper bindings |
| `ConfigureAxisResponse` | 0x0031 | Result code |
| `PushPieces` | 0x0020 | Append N pieces to an axis's ring. Payload: `axis_idx:u8, piece_count:u8, pieces[count × 28 bytes]` |
| `PushPiecesResponse` | 0x0021 | Result: OK or RING_FULL or INVALID_AXIS |
| `QueryRuntimeCaps` | 0x0040 | Request MCU capabilities |
| `RuntimeCapsResponse` | 0x0041 | Reports total piece memory (bytes) |

**MCU → Host (events, no response expected):**

| Message | Wire ID | Purpose |
|---------|---------|---------|
| `StatusHeartbeat` | 0x0080 | Periodic (10 Hz): per-axis consumed piece count (monotonic), engine state, fault info |
| `FaultEvent` | 0x0082 | Immediate: fault code + detail |

**Bootstrap (unchanged, frozen forever):**

| Message | Wire ID | Purpose |
|---------|---------|---------|
| `Identify` | 0x0001 | Handshake (proto version) |
| `IdentifyResponse` | 0x0002 | Schema hash, reset epoch, capabilities |

### 3.2 Removed Messages

| Removed | Reason |
|---------|--------|
| `LoadCurveCubic` / `LoadCurveResponse` | No curve pool — pieces go directly into ring via `PushPieces` |
| `PushSegment` / `PushSegmentResponse` | No segment concept on MCU |
| `ResetCurvePool` / `ResetCurvePoolResponse` | No pool to reset |
| `CreditFreed` event | No credit system — heartbeat consumed-count is sufficient |

### 3.3 ConfigureAxis Changes

Current `ConfigureAxes` sends kinematics tag, present_mask, awd_mask, invert_mask, and a fixed `[f32; 4]` steps_per_mm array. New `ConfigureAxis` removes:

- `kinematics` — host pre-bakes kinematic transforms
- `present_mask` / `awd_mask` — no fixed 4-axis assumption; axes registered individually
- Fixed-4 `steps_per_mm` array — moves into per-axis microstep_distance

The stepper binding payload (`StepperBindingRust`: stepper_oid + tmc_cs_oid) is unchanged.

### 3.4 RuntimeCapsResponse Changes

Currently reports `curve_pool_n` and `max_pieces_per_curve`. New response reports a single field:

- `total_piece_memory: u32` — total bytes for piece ring storage (compile-time constant from `CONFIG_RUNTIME_PIECE_RING_SIZE`)

The host divides by `28 × num_axes` to compute uniform ring depth per axis.

## 4. ISR Logic

### 4.1 Tick Function

Per tick at ISR rate:

```
tick(now):
  for each configured axis:
    piece = get_piece_for_time(axis, now)
    if piece is None:
      continue  // axis idle
    
    t_local = (now - piece.start_time) as f32 / clock_freq
    position = horner(piece.coeffs, t_local)
    drive_steppers(axis, position)
```

### 4.2 Piece Advancement

```
get_piece_for_time(axis, now):
  piece = axis.current_piece

  if piece exists and now < piece_end(piece):
    return piece          // still in current piece

  next = next_in_ring(axis)
  
  if no next:
    return None           // idle, ring empty
  
  if now < next.start_time:
    return None           // idle, next piece hasn't started yet
  
  if now - next.start_time > FAULT_TOLERANCE:
    trigger_fault(PIECE_START_IN_PAST)
  
  cache_velocity_coeffs(next)
  axis.current_piece = next
  advance_read_cursor(axis)
  return next
```

### 4.3 Piece Start Time

Each piece carries an explicit `start_time` in u64 MCU clock cycles. The host converts from absolute time using per-MCU clock frequency estimation (existing infrastructure). The ISR compares `now` (DWT CYCCNT, widened to u64) directly against `piece.start_time` — integer comparison, no floating-point in the time path.

Pieces are self-contained: each entry has everything the ISR needs to evaluate it. No derived state, no dependency on previous piece timing. The host is free to send non-contiguous pieces (gaps between piece N end and piece N+1 start — the axis idles during the gap).

## 5. Flow Control

### 5.1 Host → MCU

Host sends `PushPieces` when it knows there's space (from the last heartbeat). MCU accepts if memory is available, rejects if full. Rejection is a safety net — in normal operation the host never overfills.

### 5.2 MCU → Host

`StatusHeartbeat` at 10 Hz reports per-axis consumed piece count (monotonic counter). Host computes:

```
free_space = total_capacity - (pieces_sent - pieces_consumed)
```

No fast-path "freed" event. Pipeline depth is hundreds of ms; 100 ms heartbeat latency is negligible.

### 5.3 Multi-MCU Synchronization (Host Logic)

Not part of the MCU contract. The MCU knows nothing about other MCUs. Host-side logic:

- Host splits all axes at the same time boundaries
- Uploads each timeslot's piece to each axis on each MCU
- Advances to next timeslot only when ALL MCUs have accepted
- If any MCU is full, host waits for that MCU's heartbeat showing consumption
- If any MCU stops responding, no more pieces are sent to any MCU — all drain their rings and stop

This achieves multi-MCU lockstep synchronization without any MCU-side coordination.

### 5.4 Homing

Falls out naturally from the synchronization model:

- Host sends very short pieces (short duration) to all MCUs
- All MCUs get pieces, even those with no motion (they get zero-valued pieces)
- The endstop-monitoring MCU reports trigger via existing Klipper mechanism
- Host stops sending pieces to all MCUs → all MCUs drain and stop
- Pipeline depth (ring depth × piece duration) bounds overtravel

## 6. Safety Invariants

| Condition | Where | Severity |
|-----------|-------|----------|
| `PushPieces` but memory full | Foreground | Error response (RING_FULL) |
| Piece start time in past (> 2 ISR ticks) when ISR reaches it | ISR | **Hard fault** — halt all motion |
| Ring empty (no next piece) | ISR | Axis idles (not a fault) |
| Invalid axis_idx in `PushPieces` | Foreground | Error response (INVALID_AXIS) |
| Invalid `ConfigureAxis` parameters | Foreground | Error response |

On hard fault:
- All steppers stop immediately
- `FaultEvent` sent to host with fault code + detail
- MCU refuses further `PushPieces` until reset/reconfigure

## 7. What Changes vs Current Implementation

### 7.1 Removed from MCU

- Curve pool: 256-slot slab, generation tracking, handle packing, ABA guards (`curve_pool.rs`, `cubic_curve.rs`)
- Segment concept: segment queue (C SPSC), segment struct, arm/retire lifecycle, consumers_remaining mask (`segment.rs`, `kalico_segment_queue.c/h`)
- Retirement table and credit system: `retired_through_segment_id`, `CreditFreed` event
- Dead type definitions: `KinematicTag`, `EMode`, `extrusion_ratio`, `kinematics` field (ISR logic already removed in `daf8a1aa`)
- Fixed 4-axis assumption (`N_AXES = 4`)
- Protocol messages: `LoadCurveCubic`/`LoadCurveResponse`, `PushSegment`/`PushSegmentResponse`, `ResetCurvePool`/`ResetCurvePoolResponse`, `CreditFreed`

### 7.2 Added to MCU

- Per-axis piece ring: flat circular buffer (28 bytes/entry), static total size from `CONFIG_RUNTIME_PIECE_RING_SIZE`
- `PushPieces` message type
- Piece-start-in-past fault check in ISR
- Total-piece-memory reporting in `RuntimeCapsResponse`
- `CONFIG_RUNTIME_PIECE_RING_SIZE` Kconfig option (replaces `CURVE_POOL_N` and `MAX_PIECES_PER_CURVE`)

### 7.3 Unchanged on MCU

- ISR tick rate (40 kHz H7, 20 kHz F4)
- Horner polynomial evaluation
- Stepping dispatch: pulse mode (STEP/DIR GPIO) and phase mode (TMC5160 SPI coil currents)
- Multiple steppers per axis binding
- Stepper binding payload (`StepperBindingRust`)
- `StatusHeartbeat` (field changes: consumed piece count replaces segment retirement)
- `FaultEvent`
- Bootstrap (`Identify`/`IdentifyResponse`)
- Transport layer: framing, CRC-16/CCITT, sync byte demux
- C/Rust boundary discipline (C owns shared memory placement, Rust owns engine)

### 7.4 Moved to Host (later work, not this spec)

- Kinematic transforms (CoreXY → motor space)
- E-follower integration (already removed from ISR in `daf8a1aa`, host-side implementation is separate work)
- All multi-MCU synchronization logic
- Ring depth calculation (total_memory / 28 / num_axes)

## 8. Implementation Prerequisites

Before physical testing, calculate sane defaults for piece ring buffer sizing on F4 and H7 boards based on available SRAM budgets.

## 9. Open Questions

1. **Bernstein vs monomial on wire.** Current wire format sends Bernstein coefficients; MCU converts to monomial at load time. Could send monomial directly to save MCU-side conversion. Trade-off: Bernstein is canonical and human-readable for debugging; conversion is trivial (done once per piece load, not per tick). Recommend keeping Bernstein on wire.
