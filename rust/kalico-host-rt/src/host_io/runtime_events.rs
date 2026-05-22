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
mod lift_tests {
    use super::*;
    use crate::transport::MessageValue;

    #[test]
    fn lifts_credit_freed() {
        let mut p = MessageParams::new();
        p.insert("retired_through_segment_id", MessageValue::U32(42));
        p.insert("free_slots", MessageValue::U32(11));
        match RuntimeEvent::lift("kalico_credit_freed", p) {
            RuntimeEvent::CreditFreed(e) => {
                assert_eq!(e.retired_through_segment_id, 42);
                assert_eq!(e.free_slots, 11);
            }
            other => panic!("expected CreditFreed, got {:?}", other),
        }
    }

    #[test]
    fn lifts_fault_with_synthesized_false() {
        let mut p = MessageParams::new();
        p.insert("fault_code", MessageValue::U32(17));
        p.insert("fault_detail", MessageValue::U32(0));
        p.insert("segment_id", MessageValue::U32(42));
        match RuntimeEvent::lift("kalico_fault", p) {
            RuntimeEvent::Fault(e) => {
                assert_eq!(e.fault_code, 17);
                assert_eq!(e.synthesized, false);
            }
            other => panic!("expected Fault, got {:?}", other),
        }
    }

    #[test]
    fn lifts_unknown_to_catch_all() {
        let mut p = MessageParams::new();
        p.insert("#msg", MessageValue::String("debug trace".to_string()));
        match RuntimeEvent::lift("debug_output", p) {
            RuntimeEvent::UnknownOutput { format, msg } => {
                assert_eq!(format, "debug_output");
                assert_eq!(msg, "debug trace");
            }
            other => panic!("expected UnknownOutput, got {:?}", other),
        }
    }

    #[test]
    fn lifts_endstop_tripped() {
        let mut p = MessageParams::new();
        p.insert("arm_id", MessageValue::U32(42));
        p.insert("trip_clock_lo", MessageValue::U32(0xDEAD_BEEF));
        p.insert("trip_clock_hi", MessageValue::U32(0x0000_0001));
        p.insert("trip_source_idx", MessageValue::U32(2));
        p.insert("fmt_version", MessageValue::U32(1));
        p.insert("stepper_count", MessageValue::U32(3));
        match RuntimeEvent::lift("kalico_endstop_tripped", p) {
            RuntimeEvent::EndstopTripped(e) => {
                assert_eq!(e.arm_id, 42);
                assert_eq!(e.trip_clock, (1u64 << 32) | 0xDEAD_BEEFu64);
                assert_eq!(e.trip_source_idx, 2);
                assert_eq!(e.fmt_version, 1);
                assert_eq!(e.stepper_count, 3);
            }
            other => panic!("expected EndstopTripped, got {:?}", other),
        }
    }

    /// Spec §4.8: when the upstream decode emits the canonical
    /// `("#output", {"#msg": ..., "#format": ...})` shape (free-form path),
    /// lift must surface the firmware-side format string, not the literal
    /// "#output" routing tag.
    #[test]
    fn lifts_unknown_recovers_format_from_pseudo_field() {
        let mut p = MessageParams::new();
        p.insert("#msg", MessageValue::String("debug 5 hi".into()));
        p.insert("#format", MessageValue::String("debug_blob %u %s".into()));
        match RuntimeEvent::lift("#output", p) {
            RuntimeEvent::UnknownOutput { format, msg } => {
                assert_eq!(format, "debug_blob %u %s");
                assert_eq!(msg, "debug 5 hi");
            }
            other => panic!("expected UnknownOutput, got {:?}", other),
        }
    }
}
