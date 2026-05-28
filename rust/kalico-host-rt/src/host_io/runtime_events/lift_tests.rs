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
