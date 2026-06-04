// `spi_bus_id == 0xFF` (packed raw == `0xFFFF`) means "no phase config for
// this motor — use the existing StepPulse output path."
//
// `AtomicU16` from `portable_atomic` to match the `SharedState.phase_config`
// field type; consistent type avoids a cast when callers pass
// `&SharedState.phase_config[n]`.
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
    #[inline]
    pub const fn pack(self) -> u16 {
        ((self.spi_bus_id as u16) << 8) | (self.cs_pin_id as u16)
    }

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

pub fn store(slot: &AtomicU16, cfg: Option<PhaseConfig>) {
    let raw = match cfg {
        Some(c) => c.pack(),
        None => NONE_SENTINEL,
    };
    slot.store(raw, Ordering::Release);
}

pub fn load(slot: &AtomicU16) -> Option<PhaseConfig> {
    PhaseConfig::unpack(slot.load(Ordering::Acquire))
}

#[cfg(test)]
mod tests;
