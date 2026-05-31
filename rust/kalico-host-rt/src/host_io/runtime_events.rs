//! Layer B structured event extension. Spec §4.8.

use crate::transport::MessageParams;

#[derive(Debug, Clone)]
pub struct CreditFreedEvent {
    pub retired_through_segment_id: u32,
    pub free_slots: u8,
}

#[derive(Debug, Clone)]
pub struct FaultEvent {
    pub fault_code: u16,
    pub fault_detail: u32,
    pub segment_id: u32,
    pub synthesized: bool,
}

#[derive(Debug, Clone, Default)]
pub struct StatusEvent {
    pub engine_status: u8,
    pub queue_depth: u8,
    pub current_segment_id: u32,
    pub last_fault: u16,
    pub fault_detail: u32,
    /// v2 (2026-05-17): credit-flow watermark piggybacked on the 10 Hz
    /// periodic status frame. EventDispatcher synthesizes a `CreditFreed`
    /// dispatch from each advance so the slot-pool retirement path is
    /// driven by reliable periodic state rather than fire-and-forget
    /// events that get dropped under USB-CDC TX congestion.
    pub retired_through_segment_id: u32,
}

#[derive(Debug, Clone)]
pub struct TraceEvent {
    pub count: u32,
    pub data: Vec<u8>,
    pub flags: u32,
}

#[derive(Debug, Clone)]
pub struct EndstopTrippedEvent {
    pub arm_id: u32,
    pub trip_clock: u64,
    pub trip_source_idx: u8,
    pub fmt_version: u8,
    pub stepper_count: u8,
    pub steppers: Vec<crate::endstop::TripStepperRecord>,
}

#[derive(Debug, Clone)]
pub enum RuntimeEvent {
    CreditFreed(CreditFreedEvent),
    Fault(FaultEvent),
    Status(StatusEvent),
    Trace(TraceEvent),
    EndstopTripped(EndstopTrippedEvent),
    /// Per-axis retired-piece counts from `StatusHeartbeat` (0x0083),
    /// used by the host pump for flow control.
    Heartbeat {
        retired_counts: Vec<u32>,
    },
    /// Free-form `output("...")` from firmware that the host parser decodes
    /// into the canonical `('#output', {'#msg': formatted})` form. Routed to
    /// klippy's `#output` handler.
    UnknownOutput {
        format: String,
        msg: String,
    },
    /// Klipper-protocol response frames the firmware emits unsolicited
    /// (analog_in_state, trsync_state, stats, homing_state, …). The bridge
    /// owns the wire so klippy's serialqueue never sees these directly; they
    /// have to be forwarded by name+oid to klippy's `register_response`-set
    /// handlers. Carries the full decoded params dict so per-OID dispatch can
    /// resolve the right callback and pass the structured fields through.
    PassthroughResponse {
        name: String,
        params: MessageParams,
    },
}

impl RuntimeEvent {
    pub fn lift(name: &str, params: MessageParams) -> Self {
        match name {
            "kalico_credit_freed" => Self::CreditFreed(CreditFreedEvent {
                retired_through_segment_id: params.get_u32("retired_through_segment_id"),
                free_slots: params.get_u32("free_slots") as u8,
            }),
            "kalico_fault" => Self::Fault(FaultEvent {
                fault_code: params.get_u32("fault_code") as u16,
                fault_detail: params.get_u32("fault_detail"),
                segment_id: params.get_u32("segment_id"),
                synthesized: false,
            }),
            "kalico_status_v6" => Self::Status(StatusEvent {
                engine_status: params.get_u32("engine_status") as u8,
                queue_depth: params.get_u32("queue_depth") as u8,
                current_segment_id: params.get_u32("current_segment_id"),
                last_fault: params.get_u32("last_fault") as u16,
                fault_detail: params.get_u32("fault_detail"),
                retired_through_segment_id: params.get_u32("retired_through_segment_id"),
            }),
            "kalico_trace" => Self::Trace(TraceEvent {
                count: params.get_u32("count"),
                data: params
                    .get_bytes("data")
                    .map(<[u8]>::to_vec)
                    .unwrap_or_default(),
                flags: 0,
            }),
            "kalico_endstop_tripped" => {
                let fmt_version = params.get_u32("fmt_version") as u8;
                let stepper_count = params.get_u32("stepper_count") as u8;
                match crate::endstop::decode_trip_event(&params) {
                    Ok(evt) => Self::EndstopTripped(EndstopTrippedEvent {
                        arm_id: evt.arm_id,
                        trip_clock: evt.trip_clock,
                        trip_source_idx: evt.trip_source_idx,
                        fmt_version,
                        stepper_count,
                        steppers: evt.steppers,
                    }),
                    Err(_) => {
                        let lo = u64::from(params.get_u32("trip_clock_lo"));
                        let hi = u64::from(params.get_u32("trip_clock_hi"));
                        Self::EndstopTripped(EndstopTrippedEvent {
                            arm_id: params.get_u32("arm_id"),
                            trip_clock: (hi << 32) | lo,
                            trip_source_idx: params.get_u32("trip_source_idx") as u8,
                            fmt_version,
                            stepper_count,
                            steppers: Vec::new(),
                        })
                    }
                }
            }
            _ => {
                let msg = params.try_get_str("#msg").unwrap_or("").to_string();
                // For canonical-Python free-form formats decode_output stashes
                // the firmware-side format string in `#format` so we can surface
                // it (spec §4.8). For structured outputs that fall through to
                // catch-all (no typed branch matched) we fall back to `name`.
                let format = params
                    .try_get_str("#format")
                    .map(str::to_string)
                    .unwrap_or_else(|| name.to_string());
                Self::UnknownOutput { format, msg }
            }
        }
    }
}

#[cfg(test)]
mod lift_tests;
