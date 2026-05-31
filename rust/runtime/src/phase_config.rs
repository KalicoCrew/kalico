//! Per-motor phase-stepping SPI config (bus id + CS pin).
//!
//! Populated at `configure_axes` time, read by `runtime_modulated_tick` on
//! every tick. Stored as `AtomicU16` per motor (high byte = `spi_bus_id`,
//! low byte = `cs_pin_id`) so the ISR can read without locking; the
//! foreground writes once during configure.
//!
//! `spi_bus_id == 0xFF` (and therefore the packed raw value `0xFFFF`) means
//! "no phase config for this motor — use the existing StepPulse output path."
//!
//! Spec: docs/superpowers/specs/2026-05-18-phase-stepping-sim-design.md §3.2,
//! §4.1.

// `AtomicU16` from `portable_atomic` to match the `SharedState.phase_config`
// field type. Only load/store are performed here; using portable_atomic keeps
// the type consistent across the crate boundary so callers passing
// `&SharedState.phase_config[n]` do not require a cast.
use core::sync::atomic::Ordering;
use portable_atomic::AtomicU16;

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
mod tests;
