//! Per-motor phase-stepping SPI config (bus id + CS pin).
//!
//! Populated at `configure_axes` time, read by `runtime_modulated_tick` on
//! every tick. Stored as `AtomicU16` per motor (high byte = `spi_bus_id`,
//! low byte = `cs_pin_id`) so the ISR can read without locking; the
//! foreground writes once during configure.
//!
//! `spi_bus_id == 0xFF` (and therefore the packed raw value `0xFFFF`) means
//! "no phase config for this motor — use the existing `StepPulse` output path."
//!
//! Spec: docs/superpowers/specs/2026-05-18-phase-stepping-sim-design.md §3.2,
//! §4.1.

use core::sync::atomic::{AtomicU16, Ordering};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PhaseConfig {
    pub spi_bus_id: u8,
    pub cs_pin_id: u8,
}

/// Sentinel marking "no phase config installed on this motor."
pub const NONE_SENTINEL: u16 = 0xFFFF;

impl PhaseConfig {
    /// Pack into the wire-format `AtomicU16` representation.
    #[inline]
    pub const fn pack(self) -> u16 {
        ((self.spi_bus_id as u16) << 8) | (self.cs_pin_id as u16)
    }

    /// Unpack a raw `AtomicU16` payload. Returns `None` for `NONE_SENTINEL`.
    #[inline]
    pub const fn unpack(raw: u16) -> Option<Self> {
        if raw == NONE_SENTINEL {
            None
        } else {
            Some(PhaseConfig {
                spi_bus_id: (raw >> 8) as u8,
                cs_pin_id: (raw & 0xFF) as u8,
            })
        }
    }
}

/// Store a per-motor phase config (or clear it with `None`).
pub fn store(slot: &AtomicU16, cfg: Option<PhaseConfig>) {
    let raw = match cfg {
        Some(c) => c.pack(),
        None => NONE_SENTINEL,
    };
    slot.store(raw, Ordering::Release);
}

/// Load a per-motor phase config snapshot. Returns `None` when no phase
/// config is installed.
pub fn load(slot: &AtomicU16) -> Option<PhaseConfig> {
    PhaseConfig::unpack(slot.load(Ordering::Acquire))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack_unpack_round_trips() {
        let cfg = PhaseConfig {
            spi_bus_id: 0x03,
            cs_pin_id: 0x42,
        };
        assert_eq!(cfg.pack(), 0x0342);
        assert_eq!(PhaseConfig::unpack(0x0342), Some(cfg));
    }

    #[test]
    fn sentinel_unpacks_to_none() {
        assert_eq!(PhaseConfig::unpack(NONE_SENTINEL), None);
    }

    #[test]
    fn pack_distinct_from_sentinel_for_realistic_inputs() {
        // bus_id 0xFF is reserved as the sentinel marker; any legitimate
        // (bus, cs) where bus != 0xFF must pack to a non-sentinel value.
        let cfg = PhaseConfig {
            spi_bus_id: 0,
            cs_pin_id: 0xFF,
        };
        assert_ne!(cfg.pack(), NONE_SENTINEL);
        assert_eq!(PhaseConfig::unpack(cfg.pack()), Some(cfg));
    }

    #[test]
    fn store_load_round_trip() {
        let slot = AtomicU16::new(NONE_SENTINEL);
        assert_eq!(load(&slot), None);
        let cfg = PhaseConfig {
            spi_bus_id: 1,
            cs_pin_id: 0x10,
        };
        store(&slot, Some(cfg));
        assert_eq!(load(&slot), Some(cfg));
        store(&slot, None);
        assert_eq!(load(&slot), None);
    }
}
