use core::sync::atomic::{AtomicU32, Ordering};

use runtime::fault_sink::FaultSink;
use runtime::motion_core::{get_position_and_velocity, ArmedPiece};
use runtime::piece_ring::{PieceEntry, RingDescriptor};

pub const CLOCK_FREQ_HZ: f32 = 1_000_000_000.0;

pub const AXIS_RING_CAPACITY: usize = 256;

pub const NUM_AXES: usize = 1;

pub const EC_DC_PERIOD_NS: u32 = 1_000_000;

pub const EC_AXIS_IDX: usize = 0;

pub const FAULT_REG_NONE: u32 = 0;

pub const ENGINE_STATE_FAULT: u8 = 3;

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
        // Wire layout: bits[31:16] = deficit_us saturated to u16::MAX,
        // bits[15:0] = KALICO_ERR_PIECE_START_IN_PAST (-308 as u16 = 0xFECC).
        // Must match host's StatusHeartbeat decoder.
        #[allow(clippy::cast_sign_loss)]
        let code_u16 = (-308_i32 as i16) as u16;
        let deficit_hi16 = (deficit_us.min(u32::from(u16::MAX))) as u16;
        let val = (u32::from(deficit_hi16) << 16) | u32::from(code_u16);
        self.reg.store(val, Ordering::Release);
    }
}

pub struct AxisRing {
    storage: [PieceEntry; AXIS_RING_CAPACITY],
    desc: RingDescriptor,
    armed: Option<ArmedPiece>,
    fault: AtomicU32,
}

impl AxisRing {
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

    pub fn push_entry(&mut self, entry: PieceEntry) -> Result<(), ()> {
        self.desc.push(&mut self.storage, entry)
    }

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

    pub fn take_fault(&self) -> Option<u32> {
        let prev = self.fault.swap(FAULT_REG_NONE, Ordering::Acquire);
        if prev != FAULT_REG_NONE {
            Some(prev)
        } else {
            None
        }
    }

    pub fn retired_count(&self) -> u32 {
        self.desc.retired_count()
    }

    pub fn is_empty(&self) -> bool {
        self.desc.is_empty()
    }

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

#[cfg(test)]
mod tests;
