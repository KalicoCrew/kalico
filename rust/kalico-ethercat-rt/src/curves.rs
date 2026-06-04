//! `AxisRing`: per-axis piece ring for the EtherCAT DC loop.
//!
//! ## Protocol mapping
//!
//! The host sends `PushPieces` frames (§7.3) carrying `piece_count × 32`-byte
//! raw piece data. Each 32-byte entry is parsed as a `PieceEntry`. The DC loop
//! samples the front entry at `monotonic_ns()` and advances past it once
//! `now >= entry.end_time(CLOCK_FREQ_HZ)`. EtherCAT shares `CLOCK_MONOTONIC`
//! with the host (`clock_freq = 1e9 Hz`), so `end_time(1e9)` works directly.
//!
//! ## Retirement
//!
//! `AxisRing::retired_count()` is the monotonic count of pieces fully elapsed
//! (wrapping u32). The DC loop emits a `StatusHeartbeat` after each new
//! retirement so the host pump can replenish its per-ring occupancy accounting.
//!
//! ## Walker
//!
//! The walker retires a piece (via `advance_counter`) when `now` passes its end,
//! and faults (`EtherCatFaultSink::piece_start_in_past`) returning `None` if it
//! adopts a piece whose start is more than `drift_budget + EC_DC_PERIOD_NS`
//! in the past (where `drift_budget = (200e-6 * 1e9) = 200_000 ns` and
//! `EC_DC_PERIOD_NS = 1_000_000 ns`, total ≈ 1.2 ms), signalling that the
//! host pump fell behind. This is the correct and desired behaviour for the endpoint.
//!
//! ## Fault latch
//!
//! `AxisRing` carries an `AtomicU32` fault register. `EtherCatFaultSink` stores
//! into it (Release) when the walker calls `piece_start_in_past`; the DC loop
//! polls `take_fault()` after each `sample()` call to detect and propagate
//! faults to the host via `StatusHeartbeat`. The fault register high 16 bits
//! carry the adoption deficit in µs (saturated to `u16::MAX`), and the low 16
//! bits carry the error code (0xFECC = −308). This mirrors the MCU model:
//! fault → host learns via heartbeat → host shuts down. No allocation, no blocking.

use core::sync::atomic::{AtomicU32, Ordering};

use runtime::fault_sink::FaultSink;
use runtime::motion_core::{get_position_and_velocity, ArmedPiece};
use runtime::piece_ring::{PieceEntry, RingDescriptor};

/// EtherCAT operates on `CLOCK_MONOTONIC` in nanoseconds.
/// `clock_freq = 1e9 Hz` → `end_time(CLOCK_FREQ_HZ)` maps seconds to ns exactly.
pub const CLOCK_FREQ_HZ: f32 = 1_000_000_000.0;

/// Maximum number of pieces a single `AxisRing` can hold.
///
/// 256 slots = ~256 ms of buffer at the 1 kHz DC loop, sized to ride out
/// non-real-time host supply stalls (measured ≥65 ms on a contended Pi 3B)
/// without underrunning → -308. Inline storage per axis is
/// `AXIS_RING_CAPACITY × size_of::<PieceEntry>() = 256 × 32 B = 8 KB`.
///
/// This is the single source of truth for the ring depth. The host learns
/// this value at connect time via `QueryRuntimeCaps` / `RuntimeCapsResponse`
/// (`total_piece_memory = AXIS_RING_CAPACITY * NUM_AXES * 32`); there is no
/// shared constant in `runtime` or `motion-bridge`.
pub const AXIS_RING_CAPACITY: usize = 256;

/// Number of axes this endpoint serves. The EtherCAT RT endpoint is a
/// single-axis device (one servo drive per process instance). Used to
/// compute `total_piece_memory` in the `QueryRuntimeCaps` response so the
/// host can derive the per-axis ring depth without a shared constant.
pub const NUM_AXES: usize = 1;

/// EtherCAT DC cycle period in nanoseconds (1 ms).
///
/// Passed as the `sample_period_cycles` argument to `get_position_and_velocity`.
/// The hardened walker faults if a newly adopted piece's start is more than
/// `drift_budget + EC_DC_PERIOD_NS` (= 200_000 + 1_000_000 = 1_200_000 ns =
/// 1.2 ms) in the past, signalling that the host pump fell behind.
/// This is the blessed unified-walker formula: one DC period of tolerance
/// on top of the 200 µs drift budget. The -308 regression was caused by the
/// endpoint ring being too shallow (32 slots, ~32 ms), not by the fault
/// tolerance being too tight; deepening the ring to 256 slots is the real fix.
pub const EC_DC_PERIOD_NS: u32 = 1_000_000;

/// Axis index for the single EtherCAT servo axis. The walker passes this
/// through to `FaultSink::piece_start_in_past` so log messages identify it.
pub const EC_AXIS_IDX: usize = 0;

/// Fault register sentinel: 0 = no fault. Non-zero values encode
/// `((axis_idx & 0xFF) << 16) | fault_code_u16` so the DC loop can report
/// both axis and code to the host in the `StatusHeartbeat`.
pub const FAULT_REG_NONE: u32 = 0;

/// `engine_state` value for `StatusHeartbeat` when a fault is latched.
/// Convention: 0 = idle, 1 = running, 3 = fault. Value 2 is intentionally
/// reserved for future "paused/decelerating" semantics.
pub const ENGINE_STATE_FAULT: u8 = 3;

/// Stores faults into the `AxisRing` fault register; see the encoding at the store site.
pub struct EtherCatFaultSink<'a> {
    reg: &'a AtomicU32,
}

impl core::fmt::Debug for EtherCatFaultSink<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("EtherCatFaultSink")
            .field("reg", &self.reg.load(Ordering::Relaxed))
            .finish()
    }
}

impl FaultSink for EtherCatFaultSink<'_> {
    fn piece_start_in_past(&self, _axis_idx: usize, deficit_us: u32) {
        // Encode the adoption deficit so the host can read it directly from the
        // StatusHeartbeat fault register without a separate log channel.
        //
        // Wire layout (single u32):
        //   bits [31:16]  deficit in microseconds, saturated to u16::MAX (65535 µs).
        //                 Example: deficit=1500 µs → high half = 0x05DC.
        //   bits [15: 0]  KALICO_ERR_PIECE_START_IN_PAST (-308) as u16 = 0xFECC.
        //
        // Example: fault at 1500 µs deficit → 0x05DC_FECC.
        //
        // The axis_idx field is dropped: the EtherCAT endpoint is always a
        // single-axis device (EC_AXIS_IDX == 0), so those bits carry no information
        // and are more usefully filled with the deficit magnitude.
        //
        // -308i32 as i16 = -308i16; -308i16 as u16 = 0xFECC.
        #[allow(clippy::cast_sign_loss)]
        let code_u16 = (-308_i32 as i16) as u16;
        let deficit_hi16 = (deficit_us.min(u32::from(u16::MAX))) as u16;
        let val = (u32::from(deficit_hi16) << 16) | u32::from(code_u16);
        // Store with Release: the DC loop reads with Acquire (via swap) so it
        // always sees the populated value before it observes non-zero.
        self.reg.store(val, Ordering::Release);
    }
}

/// Per-axis piece ring for the EtherCAT DC loop.
///
/// Uses `RingDescriptor` (borrow-free) with its own inline backing store
/// so the struct can be moved freely and constructed without unsafe code.
/// Walk-and-eval is performed by the shared hardened walker in
/// `runtime::motion_core::get_position_and_velocity`; no hand-rolled loop
/// lives here.
///
/// ## Fault register
///
/// `fault` is an `AtomicU32` initially zero. When the walker calls
/// `piece_start_in_past`, `EtherCatFaultSink` performs a single Release
/// store encoding `(axis_idx << 16) | fault_code_u16`. The DC loop calls
/// `take_fault()` after each `sample()` to detect and propagate faults.
pub struct AxisRing {
    storage: [PieceEntry; AXIS_RING_CAPACITY],
    desc: RingDescriptor,
    /// Cached armed piece for the shared walker. `None` when the ring is empty
    /// or has not yet been sampled. The walker manages this field entirely; the
    /// `AxisRing` methods must not touch it except to zero it on `reset`.
    armed: Option<ArmedPiece>,
    /// Fault latch: 0 = none, non-zero = `((axis_idx & 0xFF) << 16) | code_u16`.
    /// Written by `EtherCatFaultSink` (Release) on the RT path; consumed by
    /// `take_fault()` (Acquire swap) on the DC loop path.
    fault: AtomicU32,
}

impl AxisRing {
    /// Construct an empty ring backed by inline storage.
    pub fn new() -> Self {
        Self {
            storage: [PieceEntry {
                start_time: 0,
                coeffs: [0.0; 4],
                duration: 0.0,
                _reserved: 0,
            }; AXIS_RING_CAPACITY],
            desc: RingDescriptor::new(0, AXIS_RING_CAPACITY),
            armed: None,
            fault: AtomicU32::new(FAULT_REG_NONE),
        }
    }

    /// Push one piece entry. Returns `Err(())` if the ring is full.
    pub fn push_entry(&mut self, entry: PieceEntry) -> Result<(), ()> {
        self.desc.push(&mut self.storage, entry)
    }

    /// Parse and push `piece_count` × 32-byte entries from a raw byte slice.
    ///
    /// Returns the number of entries successfully pushed. Stops at the first
    /// full-ring error and logs a warning.
    pub fn push_from_bytes(&mut self, piece_count: u8, bytes: &[u8]) -> u8 {
        let n = piece_count as usize;
        if bytes.len() < n * 32 {
            log::warn!(
                "AxisRing::push_from_bytes: short payload ({} < {})",
                bytes.len(),
                n * 32
            );
            return 0;
        }
        let mut pushed = 0u8;
        for chunk in bytes[..n * 32].chunks_exact(32) {
            let entry = parse_piece_entry(chunk);
            if self.desc.push(&mut self.storage, entry).is_err() {
                log::warn!("AxisRing::push_from_bytes: ring full at entry {pushed}/{piece_count}");
                break;
            }
            pushed += 1;
        }
        pushed
    }

    /// Sample the axis at `now_ns` (CLOCK_MONOTONIC nanoseconds).
    pub fn sample(&mut self, now_ns: u64) -> Option<(f32, f32)> {
        let AxisRing {
            ref mut armed,
            ref mut desc,
            ref storage,
            ref fault,
            ..
        } = *self;
        let sink = EtherCatFaultSink { reg: fault };
        get_position_and_velocity(
            armed,
            desc,
            storage,
            now_ns,
            EC_DC_PERIOD_NS,
            CLOCK_FREQ_HZ,
            EC_AXIS_IDX,
            &sink,
        )
    }

    /// Consume and return the latched fault value, if any.
    ///
    /// Performs an Acquire swap of the fault register to zero. Returns `Some(v)`
    /// where `v = ((axis_idx & 0xFF) << 16) | code_u16` if a fault was latched
    /// since the last call; returns `None` if no fault is pending.
    ///
    /// The DC loop should call this after every `sample()` and propagate any
    /// returned value to the host via `StatusHeartbeat`.
    pub fn take_fault(&self) -> Option<u32> {
        let prev = self.fault.swap(FAULT_REG_NONE, Ordering::Acquire);
        if prev != FAULT_REG_NONE {
            Some(prev)
        } else {
            None
        }
    }

    /// Monotonic count of pieces whose time window has fully elapsed (wrapping u32).
    /// Used by the DC loop to detect new retirements and emit `StatusHeartbeat`.
    pub fn retired_count(&self) -> u32 {
        self.desc.retired_count()
    }

    /// Returns `true` if the ring contains no pieces.
    pub fn is_empty(&self) -> bool {
        self.desc.is_empty()
    }

    /// Discard all pieces (on reset or reconnect).
    pub fn reset(&mut self) {
        self.desc.drain();
        self.armed = None;
        self.fault.store(FAULT_REG_NONE, Ordering::Relaxed);
    }
}

impl Default for AxisRing {
    fn default() -> Self {
        Self::new()
    }
}

impl core::fmt::Debug for AxisRing {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("AxisRing")
            .field("len", &self.desc.len())
            .field("retired", &self.desc.retired_count())
            .finish()
    }
}

/// Parse one `PieceEntry` from a 32-byte little-endian slice.
fn parse_piece_entry(chunk: &[u8]) -> PieceEntry {
    debug_assert_eq!(chunk.len(), 32, "piece entry must be 32 bytes");
    let rd4 = |i: usize| u32::from_le_bytes([chunk[i], chunk[i + 1], chunk[i + 2], chunk[i + 3]]);
    let start_time = u64::from_le_bytes([
        chunk[0], chunk[1], chunk[2], chunk[3], chunk[4], chunk[5], chunk[6], chunk[7],
    ]);
    let c0 = f32::from_bits(rd4(8));
    let c1 = f32::from_bits(rd4(12));
    let c2 = f32::from_bits(rd4(16));
    let c3 = f32::from_bits(rd4(20));
    let duration = f32::from_bits(rd4(24));
    PieceEntry {
        start_time,
        coeffs: [c0, c1, c2, c3],
        duration,
        _reserved: 0,
    }
}

// =============================================================================
// Unit tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a single-entry ease-curve piece: Bernstein [from, from, to, to].
    fn ease_entry(from: f32, to: f32, start_time_ns: u64, dur_s: f32) -> PieceEntry {
        PieceEntry {
            start_time: start_time_ns,
            coeffs: [from, from, to, to],
            duration: dur_s,
            _reserved: 0,
        }
    }

    #[test]
    fn ring_walk_eval_single_piece() {
        let mut ring = AxisRing::new();
        let t0: u64 = 1_000_000_000; // 1 s in ns
        let dur: f32 = 1.0; // 1 s

        ring.push_entry(ease_entry(0.0, 10.0, t0, dur)).unwrap();

        let (pos_start, _vel) = ring.sample(t0).unwrap();
        assert!((pos_start - 0.0).abs() < 1e-3, "start pos={pos_start}");

        let (pos_mid, _vel) = ring.sample(t0 + 500_000_000).unwrap();
        assert!((pos_mid - 5.0).abs() < 0.2, "mid pos={pos_mid}");

        let pos_after = ring.sample(t0 + 2_000_000_000);
        assert!(
            pos_after.is_none(),
            "ring must be empty after piece expires"
        );
        assert_eq!(ring.retired_count(), 1);
    }

    #[test]
    fn ring_walk_two_pieces_contiguous() {
        // Use short (1 ms) pieces so the piece-start is always within the
        // hardened walker's 1.2 ms adoption-fault window. The test samples each
        // piece close to (or at) its start_time, then advances the clock past
        // the end_time to trigger retirement.
        let mut ring = AxisRing::new();

        // Both pieces are 1 ms each. Piece 0 starts at t0, piece 1 at t0+1ms.
        let t0: u64 = 1_000_000_000; // 1 s offset (arbitrary non-zero epoch)
        let dur_ns: u64 = 1_000_000; // 1 ms in ns
        let dur_s: f32 = 0.001;

        ring.push_entry(ease_entry(0.0, 10.0, t0, dur_s)).unwrap();
        ring.push_entry(ease_entry(10.0, 0.0, t0 + dur_ns, dur_s))
            .unwrap();

        // Sample at exactly t0 (arm piece 0, elapsed=0 → pos=0).
        let (pos_start, _) = ring.sample(t0).unwrap();
        assert!((pos_start - 0.0).abs() < 1e-3, "start pos={pos_start}");
        assert_eq!(ring.retired_count(), 0);

        // Sample at piece 0 end_time (t0 + dur_ns):
        // `now < piece_end_cycles` → FALSE → retire piece 0, arm piece 1 at t=0 → 10.0.
        let (pos_boundary, _) = ring.sample(t0 + dur_ns).unwrap();
        assert!(
            (pos_boundary - 10.0).abs() < 0.1,
            "pos at piece0 end={pos_boundary}"
        );
        assert_eq!(ring.retired_count(), 1, "piece 0 retired at boundary");

        // Sample just before piece 1 expires.
        let (pos_p1_near_end, _) = ring.sample(t0 + 2 * dur_ns - 1).unwrap();
        assert!(
            (pos_p1_near_end - 0.0).abs() < 0.1,
            "piece1 near-end pos={pos_p1_near_end}"
        );

        // Sample at exactly piece 1 end_time → retires piece 1, ring empty → None.
        let pos_gone = ring.sample(t0 + 2 * dur_ns);
        assert!(pos_gone.is_none(), "ring must be empty at piece 1 end_time");
        assert_eq!(ring.retired_count(), 2);
    }

    #[test]
    fn push_from_bytes_round_trips() {
        let entry = ease_entry(0.0, 5.0, 500_000_000, 0.5);
        let bytes = entry.to_le_bytes();
        let mut all_bytes = bytes.to_vec();
        all_bytes.extend_from_slice(&bytes); // two identical pieces

        let mut ring = AxisRing::new();
        let pushed = ring.push_from_bytes(2, &all_bytes);
        assert_eq!(pushed, 2);
        assert_eq!(ring.desc.len(), 2);
    }

    #[test]
    fn retired_count_heartbeat() {
        let mut ring = AxisRing::new();
        assert_eq!(ring.retired_count(), 0);

        // A 1 ms piece starting at t=0.
        ring.push_entry(ease_entry(0.0, 1.0, 0, 0.001)).unwrap();
        // Before expiry: no retirement.
        ring.sample(0);
        assert_eq!(ring.retired_count(), 0, "not yet retired");

        // Now arm and advance through the piece's window on a fresh ring to
        // confirm retirement. The piece is already ARMED by the prior sample(0)
        // call above, so any sample past end_time=1ms will RETIRE it
        // (retired_count → 1). A late sample on an already-armed piece does not
        // re-trigger PieceStartInPast; that fault only fires on the adoption
        // transition when the new piece's start is > fault_tolerance (1.2 ms) in the past.
        let mut ring2 = AxisRing::new();
        ring2.push_entry(ease_entry(0.0, 1.0, 0, 0.001)).unwrap();
        // First sample at t=0 — arms the piece (start=0, now=0, no fault).
        ring2.sample(0);
        // Sample at 500us (within piece window, before end_time=1ms): still no retirement.
        ring2.sample(500_000);
        assert_eq!(ring2.retired_count(), 0, "not yet retired at 0.5ms");
        // Sample at 2ms: piece ended at 1ms, now > end → walker retires it.
        ring2.sample(2_000_000);
        assert_eq!(ring2.retired_count(), 1, "should be retired at 2ms");
    }

    #[test]
    fn reset_clears_ring() {
        let mut ring = AxisRing::new();
        ring.push_entry(ease_entry(0.0, 1.0, 0, 1.0)).unwrap();
        ring.push_entry(ease_entry(1.0, 0.0, 1_000_000_000, 1.0))
            .unwrap();
        ring.reset();
        assert!(ring.is_empty());
        assert!(ring.armed.is_none());
        assert_eq!(
            ring.take_fault(),
            None,
            "reset must clear the fault register"
        );
    }

    /// A piece whose start is more than `drift_budget + EC_DC_PERIOD_NS` (1.2 ms)
    /// in the past must latch the fault register and return None.
    ///
    /// Detail: take_fault() returns Some(v) where
    ///   bits [31:16] = deficit_us saturated to u16::MAX
    ///   bits [15: 0] = 0xFECC  (KALICO_ERR_PIECE_START_IN_PAST as u16)
    ///
    /// For start=0, now=20_000_000 ns (20 ms):
    ///   deficit_cycles = 20_000_000 ns
    ///   deficit_us = (20_000_000 * 1e-3) as u32 = 20_000 µs
    ///   high 16 bits = 20_000 = 0x4E20
    ///   result = 0x4E20_FECC
    #[test]
    fn ethercat_fault_latches() {
        let mut ring = AxisRing::new();

        // No fault before any sample.
        assert_eq!(ring.take_fault(), None, "no fault on empty ring");

        // Push a piece with start_time > fault_tolerance in the past relative to
        // the sample time.
        // fault_tolerance = drift_budget + EC_DC_PERIOD_NS
        //   = (200e-6 * 1e9) + 1_000_000 = 200_000 + 1_000_000 = 1_200_000 ns (1.2 ms).
        // Use start_time = 0 and sample at 20_000_000 ns (20 ms) so gap > 1.2 ms.
        let piece_start_ns: u64 = 0;
        let sample_now_ns: u64 = 20_000_000; // 20 ms > 8.2 ms fault tolerance
        ring.push_entry(ease_entry(0.0, 1.0, piece_start_ns, 100.0))
            .unwrap();

        // The walker adopts the piece and faults because (now - start) > fault_tolerance.
        let result = ring.sample(sample_now_ns);
        assert!(result.is_none(), "PieceStartInPast must return None");

        // Fault must be latched.
        let fault_val = ring.take_fault().expect("fault must be latched");

        // Low 16 bits: error code.
        let code_u16 = (fault_val & 0xFFFF) as u16;
        // KALICO_ERR_PIECE_START_IN_PAST = -308 → as i16 = -308 → as u16 = 0xFECC.
        #[allow(clippy::cast_sign_loss)]
        let expected_code = (-308_i32 as i16) as u16;
        assert_eq!(
            code_u16, expected_code,
            "fault code must be PieceStartInPast wire value"
        );

        // High 16 bits: deficit in µs.
        // deficit_cycles = 20_000_000; deficit_us = (20_000_000 * 1e-3) as u32 = 20_000 µs.
        let deficit_us_hi = (fault_val >> 16) as u16;
        assert_eq!(
            deficit_us_hi, 20_000_u16,
            "fault high 16 bits must be deficit_us=20000 (0x4E20)"
        );

        // take_fault() must clear the register — second call returns None.
        assert_eq!(
            ring.take_fault(),
            None,
            "fault register must be cleared after take"
        );

        // retired_count must still be 0 — the fault path does not advance the counter.
        assert_eq!(ring.retired_count(), 0, "fault must not retire the piece");
    }

    /// Integration-level no-jump test: capturing the CountMap origin at the first
    /// sample instant means the first commanded count equals the rotor's actual
    /// count, producing no startup position jump.
    ///
    /// Protocol:
    ///   1. Push a constant-position piece (b0=b1=b2=b3=5.0 mm) starting at t0.
    ///   2. sample(t0) → position must be 5.0 mm (elapsed=0 → b0 value).
    ///   3. Build CountMap::new(gain, actual_counts=20000, sampled_pos).
    ///   4. target_counts(sampled_pos) must equal 20000 exactly — no jump.
    ///
    /// This is the endpoint-level lock-in of the `cmap.get_or_insert_with(...)`
    /// invariant in the binary: capturing the origin on the first sample ensures
    /// commanded_counts[0] == rotor_actual_counts[0].
    #[test]
    fn no_jump_at_origin_capture() {
        use crate::scale::CountMap;

        let mut ring = AxisRing::new();
        let t0: u64 = 5_000_000_000; // 5 s epoch — well clear of fault window
        let dur_s: f32 = 0.001; // 1 ms piece
        let pos_mm = 5.0_f32;

        // Constant-position piece: all Bernstein control points equal pos_mm.
        // Bernstein [p,p,p,p] evaluates to p for all t ∈ [0,1].
        ring.push_entry(PieceEntry {
            start_time: t0,
            coeffs: [pos_mm; 4],
            duration: dur_s,
            _reserved: 0,
        })
        .unwrap();

        // sample at exactly t0 (elapsed = 0) — the walker arms the piece and
        // evaluates at t=0. For [p,p,p,p] the monomial is [p, 0, 0, 0] so P(0)=p.
        let (sampled_pos, _vel) = ring.sample(t0).expect("sample at t0 must return Some");
        assert!(
            (sampled_pos - pos_mm).abs() < 1e-4_f32,
            "sample at t0 must return b0={pos_mm:.4}, got {sampled_pos:.6}"
        );

        // Capture the origin at the first sample: actual_counts=20000, pos=sampled_pos.
        let counts_per_mm = 3276.8_f64;
        let actual_counts = 20_000_i32;
        let cmap = CountMap::new(counts_per_mm, actual_counts, f64::from(sampled_pos));

        // The first commanded count must equal the rotor's actual count — no jump.
        assert_eq!(
            cmap.target_counts(f64::from(sampled_pos)),
            actual_counts,
            "CountMap origin capture must produce target_counts == actual_counts (no startup jump)"
        );
    }

    /// Build two contiguous C0+C1 pieces from a split linear ramp and verify
    /// that position is continuous and velocity is continuous across the boundary.
    ///
    /// Construction:
    ///   - Full ramp: 0 → 10 mm in 2 ms. Bernstein form for a linear ramp over
    ///     interval [0,D] is [a, a+(b-a)/3, a+2(b-a)/3, b].
    ///   - Split at the midpoint (1 ms each):
    ///       Piece 0: [0, 5/3, 10/3, 5] mm, duration=1ms, start=t0
    ///       Piece 1: [5, 5+5/3, 5+10/3, 10] mm, duration=1ms, start=t0+1ms
    ///   - End of piece 0 = start of piece 1 = 5.0 mm (C0).
    ///   - End velocity of piece 0 = 3*(b3-b2)/D = 3*(5-10/3)/0.001 = 5000 mm/s.
    ///   - Start velocity of piece 1 = 3*(b1-b0)/D = 3*(5/3)/0.001 = 5000 mm/s (C1).
    ///
    /// Samples at boundary−1ns and boundary+1ns must satisfy:
    ///   |pos_after − pos_before| < 0.01 mm  (C0 continuity)
    ///   |vel_after − vel_before| < 1.0 mm/s  (C1 continuity; ~0.02% of 5000 mm/s)
    ///   vel > 0 (monotone-increasing ramp — signs are correct)
    #[test]
    fn piece_boundary_c0_c1_continuity() {
        let mut ring = AxisRing::new();

        let t0: u64 = 2_000_000_000; // 2 s epoch
        let dur_ns: u64 = 1_000_000; // 1 ms in ns
        let dur_s: f32 = 0.001_f32;
        let boundary_ns: u64 = t0 + dur_ns;

        // Linear ramp 0→10 mm split at midpoint (de Casteljau at t=0.5 of unit domain):
        //   Piece 0: b = [0,  5/3, 10/3, 5]
        //   Piece 1: b = [5, 20/3, 25/3, 10]
        //
        // Derivation: for the full linear ramp [0, 10/3, 20/3, 10] the midpoint
        // de Casteljau split yields exactly these two halves, each of which is
        // itself a linear ramp over [0,5] rescaled to the 1ms window.
        let b0_piece0: [f32; 4] = [0.0, 5.0 / 3.0, 10.0 / 3.0, 5.0];
        let b0_piece1: [f32; 4] = [5.0, 5.0 + 5.0 / 3.0, 5.0 + 10.0 / 3.0, 10.0];

        ring.push_entry(PieceEntry {
            start_time: t0,
            coeffs: b0_piece0,
            duration: dur_s,
            _reserved: 0,
        })
        .unwrap();
        ring.push_entry(PieceEntry {
            start_time: boundary_ns,
            coeffs: b0_piece1,
            duration: dur_s,
            _reserved: 0,
        })
        .unwrap();

        // Sample one nanosecond before the boundary — still on piece 0.
        // Both samples are within the 1.2ms adoption-fault window of their respective pieces.
        let (pos_before, vel_before) = ring
            .sample(boundary_ns - 1)
            .expect("sample before boundary must return Some");
        // The fault register must not be set.
        assert_eq!(
            ring.take_fault(),
            None,
            "no fault expected for in-window piece 0 sample"
        );

        // sample() at boundary_ns retires piece 0 and arms piece 1.
        let (pos_after, vel_after) = ring
            .sample(boundary_ns + 1)
            .expect("sample after boundary must return Some");
        assert_eq!(
            ring.take_fault(),
            None,
            "no fault expected for in-window piece 1 sample"
        );

        // C0 continuity: position must not jump across the boundary.
        let pos_gap = (pos_after - pos_before).abs();
        assert!(
            pos_gap < 0.01_f32,
            "C0 continuity violated: |pos_after({pos_after:.6}) - pos_before({pos_before:.6})| \
             = {pos_gap:.6} >= 0.01 mm"
        );

        // C1 continuity: velocity must be continuous across the boundary.
        // Both pieces are from the same linear ramp, so the theoretical velocity is 5000 mm/s.
        let vel_gap = (vel_after - vel_before).abs();
        assert!(
            vel_gap < 1.0_f32,
            "C1 continuity violated: |vel_after({vel_after:.3}) - vel_before({vel_before:.3})| \
             = {vel_gap:.3} >= 1.0 mm/s"
        );

        // Sign convention: velocity must be positive on a monotone-increasing ramp.
        assert!(
            vel_before > 0.0_f32,
            "vel_before={vel_before:.3} must be positive (monotone-increasing ramp)"
        );
        assert!(
            vel_after > 0.0_f32,
            "vel_after={vel_after:.3} must be positive (monotone-increasing ramp)"
        );

        // Sanity-check magnitude: both velocities should be close to 5000 mm/s.
        // Tolerance is 5% (250 mm/s) to account for f32 and Bernstein curvature
        // from the de Casteljau split.
        assert!(
            (vel_before - 5000.0_f32).abs() < 250.0_f32,
            "vel_before={vel_before:.1} should be ~5000 mm/s for a linear ramp"
        );
        assert!(
            (vel_after - 5000.0_f32).abs() < 250.0_f32,
            "vel_after={vel_after:.1} should be ~5000 mm/s for a linear ramp"
        );
    }

    /// Verify the exact fault-tolerance boundary of `get_position_and_velocity`.
    ///
    /// The fault condition uses the drift-budget formula from motion_core:
    ///   `drift_budget = (200e-6 * CLOCK_FREQ_HZ) as u64 = (200e-6 * 1e9) = 200_000 ns`
    ///   `fault_tolerance = drift_budget + EC_DC_PERIOD_NS
    ///                     = 200_000 + 1_000_000 = 1_200_000 ns (1.2 ms)`
    ///
    ///   - At exactly the tolerance (gap = 1_200_000 ns): NOT faulting → Some.
    ///   - One ns beyond the tolerance (gap = 1_200_001 ns): faulting → None.
    ///
    /// This pins the strict-greater-than semantics from the motion_core walker and
    /// ensures we are not accidentally using >=, which would fault valid on-time pieces.
    ///
    /// Also verifies the fault-register encoding:
    ///   bits [31:16] = deficit_us (saturated to u16::MAX)
    ///   bits [15: 0] = 0xFECC (KALICO_ERR_PIECE_START_IN_PAST as u16)
    #[test]
    fn fault_boundary_exact() {
        const MAX_START_IN_PAST_SECS: f32 = 200e-6;
        let drift_budget = (MAX_START_IN_PAST_SECS * CLOCK_FREQ_HZ) as u64; // 200_000 ns
        let fault_tolerance_ns = drift_budget + u64::from(EC_DC_PERIOD_NS); // 1_200_000 ns

        // --- Case A: gap == fault_tolerance → must NOT fault (strictly-greater-than check) ---
        let now_a: u64 = 10_000_000_000;
        let start_a: u64 = now_a - fault_tolerance_ns; // gap == 1_200_000 exactly

        let mut ring_a = AxisRing::new();
        // Long-duration piece so it's still active at now_a.
        ring_a
            .push_entry(PieceEntry {
                start_time: start_a,
                coeffs: [0.0_f32; 4],
                duration: 10.0_f32, // 10 s — well past now_a
                _reserved: 0,
            })
            .unwrap();

        let result_a = ring_a.sample(now_a);
        assert!(
            result_a.is_some(),
            "gap == fault_tolerance ({fault_tolerance_ns} ns) must NOT fault (strictly-greater-than); got None"
        );
        assert_eq!(
            ring_a.take_fault(),
            None,
            "no fault must be latched when gap == tolerance"
        );

        // --- Case B: gap == fault_tolerance + 1 → MUST fault ---
        // deficit_cycles = fault_tolerance_ns + 1; deficit_us = (fault_tolerance_ns+1)/1000
        // = 1_200_001 / 1000 = 1200 µs (truncated via f32 FPU in motion_core).
        // Expected high 16 bits: 1200 = 0x04B0.
        let start_b: u64 = now_a - fault_tolerance_ns - 1; // gap == fault_tolerance + 1

        let mut ring_b = AxisRing::new();
        ring_b
            .push_entry(PieceEntry {
                start_time: start_b,
                coeffs: [0.0_f32; 4],
                duration: 10.0_f32,
                _reserved: 0,
            })
            .unwrap();

        let result_b = ring_b.sample(now_a);
        assert!(
            result_b.is_none(),
            "gap == fault_tolerance + 1 must fault and return None"
        );

        let fault_val = ring_b
            .take_fault()
            .expect("fault register must be latched when gap > fault_tolerance");

        // Low 16 bits: KALICO_ERR_PIECE_START_IN_PAST (-308 as i16 as u16 = 0xFECC).
        #[allow(clippy::cast_sign_loss)]
        let expected_code = (-308_i32 as i16) as u16;
        let code_u16 = (fault_val & 0xFFFF) as u16;
        assert_eq!(
            code_u16, expected_code,
            "fault register low 16 bits must be 0xFECC (−308)"
        );

        // High 16 bits: deficit in µs. deficit_cycles = 1_200_001 ns (ns == cycles at 1GHz).
        // motion_core computes: (cycles as f32 * (1e6 / 1e9)) as u32 = (1_200_001 * 1e-3) as u32.
        // In f32: 1_200_001.0 * 0.001 = ~1200.001 → truncates to 1200.
        let deficit_us_hi = (fault_val >> 16) as u16;
        assert_eq!(
            deficit_us_hi, 1200_u16,
            "fault register high 16 bits must be 1200 µs (deficit at tolerance+1 boundary)"
        );
    }

    /// Verify that `PieceEntry::end_time(CLOCK_FREQ_HZ)` for a 1 ms piece is
    /// exactly `start_time + 1_000_000` ns.
    ///
    /// The concern is f32 truncation drift: `0.001_f32 * 1_000_000_000.0_f32`
    /// must truncate to exactly 1_000_000 via `as u64`. This is verified here
    /// rather than assumed, because any drift in end_time would cause premature or
    /// delayed retirement that compounds across the streaming chain.
    ///
    /// Analytic: 0.001 in f32 is ~0.0010000000474974 (next representable value
    /// above 0.001). Multiplied by 1e9: ~1_000_000.047... → truncates to 1_000_000.
    #[test]
    fn end_time_ns_precision() {
        use runtime::piece_ring::PieceEntry;

        let start: u64 = 7_000_000_000;
        let entry = PieceEntry {
            start_time: start,
            coeffs: [0.0_f32; 4],
            duration: 0.001_f32, // 1 ms
            _reserved: 0,
        };

        let end = entry.end_time(CLOCK_FREQ_HZ);
        assert_eq!(
            end,
            start + 1_000_000,
            "end_time for 1ms piece must be start + 1_000_000 ns exactly; \
             got {} (delta={})",
            end,
            end.wrapping_sub(start)
        );
    }

    /// After a fault (stale unarmed piece), a subsequent piece that is still
    /// in-window recovers and returns Some.
    #[test]
    fn fault_then_in_window_recovers() {
        let mut ring = AxisRing::new();

        // Piece starts at 0 but lasts 100ms. We sample at 20ms so the piece is
        // still active (end=100ms), but the gap (20ms) exceeds the
        // fault_tolerance = drift_budget + EC_DC_PERIOD_NS
        //   = 200_000 + 1_000_000 = 1_200_000 ns (1.2 ms).
        // The walker sees (now - start = 20ms) > 1.2ms tolerance and faults.
        // The piece is NOT armed and the ring stays occupied.
        let now_ns: u64 = 20_000_000; // 20 ms > 1.2 ms tolerance
        let start_ns: u64 = 0;
        ring.push_entry(ease_entry(0.0, 1.0, start_ns, 0.1))
            .unwrap(); // 100 ms piece

        // First sample — should fault.
        let r1 = ring.sample(now_ns);
        assert!(r1.is_none(), "stale unarm must return None");
        assert_eq!(ring.retired_count(), 0, "no retirement on fault");
        let fault1 = ring.take_fault().expect("fault must be latched");
        assert_ne!(fault1, FAULT_REG_NONE, "fault register must be non-zero");

        // The piece was NOT armed (fault path returns before storing armed).
        // Push a fresh piece well within the window and verify recovery.
        // Reset first to drain the stale piece.
        ring.reset();
        assert_eq!(ring.take_fault(), None, "reset clears fault register");

        let now2: u64 = 1_000_000_000; // 1 s epoch — far in the future
        ring.push_entry(ease_entry(0.0, 1.0, now2, 0.1)).unwrap(); // starts exactly now
        let r2 = ring.sample(now2);
        assert!(r2.is_some(), "in-window piece must return Some after reset");
        assert_eq!(ring.take_fault(), None, "no fault for in-window piece");
    }
}
