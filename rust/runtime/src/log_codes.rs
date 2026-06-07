// Subsystem and event code tables for the MCU structured-log endpoint.
//
// Subsystem IDs and event codes are wire-stable u8/u16 discriminants.
// Names and templates are resolved host-side from these tables.
// Compiles for both `no_std` MCU targets and the host.
//
// WIRE-STABLE: do not renumber existing codes. New events append.

#![allow(dead_code)]

// ── Subsystem IDs (u8) ──────────────────────────────────────────────────────

pub const SUBSYSTEM_RUNTIME: u8 = 0;
pub const SUBSYSTEM_MOTION: u8 = 1;
pub const SUBSYSTEM_TICK: u8 = 2;
pub const SUBSYSTEM_ENDSTOP: u8 = 3;
pub const SUBSYSTEM_DIAG: u8 = 4;

/// Resolve a subsystem id to its `&'static str` name.
///
/// # Examples
///
/// ```
/// use runtime::log_codes::{subsystem_name, SUBSYSTEM_RUNTIME};
/// assert_eq!(subsystem_name(SUBSYSTEM_RUNTIME), "runtime");
/// assert_eq!(subsystem_name(0xFF), "unknown");
/// ```
pub fn subsystem_name(id: u8) -> &'static str {
    match id {
        SUBSYSTEM_RUNTIME => "runtime",
        SUBSYSTEM_MOTION => "motion",
        SUBSYSTEM_TICK => "tick",
        SUBSYSTEM_ENDSTOP => "endstop",
        SUBSYSTEM_DIAG => "diag",
        _ => "unknown",
    }
}

// ── Event codes (u16) per subsystem ─────────────────────────────────────────
//
// Convention: EVENT_<SUBSYSTEM>_<NAME>. Codes are unique within each subsystem
// but may repeat across subsystems — the (subsystem, event) pair is the key.
// Start at 1; 0 is reserved as "no event".

pub const EVENT_RUNTIME_FAULT_LATCHED: u16 = 1;
pub const EVENT_RUNTIME_ENGINE_RESET: u16 = 2;
pub const EVENT_RUNTIME_MCU_READY: u16 = 3;
pub const EVENT_RUNTIME_LOG_DROPS: u16 = 4;
pub const EVENT_RUNTIME_MCU_RESET: u16 = 5;
pub const EVENT_RUNTIME_HARD_FAULT: u16 = 6;
pub const EVENT_RUNTIME_FAULT_STATUS: u16 = 7;
pub const EVENT_RUNTIME_FG_FREEZE: u16 = 8;
pub const EVENT_RUNTIME_RT_PROGRESS: u16 = 9;
pub const EVENT_RUNTIME_LAST_DISPATCH: u16 = 10;
pub const EVENT_RUNTIME_ISR_PHASE: u16 = 11;
pub const EVENT_RUNTIME_BLOCK_SOURCE: u16 = 12;
pub const EVENT_RUNTIME_TIM5_IA: u16 = 13;
pub const EVENT_RUNTIME_DIAG_DUMP: u16 = 14;

pub const EVENT_MOTION_PIECE_START_PAST: u16 = 1;
pub const EVENT_MOTION_RING_FULL: u16 = 2;

pub const EVENT_TICK_INTERVAL_EXCEEDED: u16 = 1;
pub const EVENT_TICK_UNDERRUN: u16 = 2;

pub const EVENT_ENDSTOP_TRIP: u16 = 1;
pub const EVENT_ENDSTOP_ARM_TIMEOUT: u16 = 2;
pub const EVENT_ENDSTOP_TRSYNC_TRIGGER_CMD: u16 = 3;
pub const EVENT_ENDSTOP_TRSYNC_DO_TRIGGER: u16 = 4;
pub const EVENT_ENDSTOP_STOP_CB_ENTER: u16 = 5;
pub const EVENT_ENDSTOP_SOFTWARE_TRIP: u16 = 6;
pub const EVENT_ENDSTOP_TIM5_HALTED: u16 = 7;

// diag subsystem events (codes mirror MCU DIAG_EV_* tag values 1..=8)
pub const EVENT_DIAG_TIM5_LONG: u16 = 1;
pub const EVENT_DIAG_OTG_LONG: u16 = 2;
pub const EVENT_DIAG_USB_OUT_GAP: u16 = 3;
pub const EVENT_DIAG_USB_IN_GAP: u16 = 4;
pub const EVENT_DIAG_TX_DROP_KAL: u16 = 5;
pub const EVENT_DIAG_TX_DROP_KLP: u16 = 6;
pub const EVENT_DIAG_ENGINE_XITION: u16 = 7;
pub const EVENT_DIAG_RUST_FAULT: u16 = 8;

/// Resolve a `(subsystem, event)` pair to a `(name, template)` tuple.
///
/// `name` is the stable event key (e.g. `"tick.interval_exceeded"`).
/// `template` substitutes `{arg0}` and `{arg1}` with the two numeric args
/// from the `McuLog` frame.
///
/// Returns `("unknown", "")` for unrecognised pairs.
///
/// # Examples
///
/// ```
/// use runtime::log_codes::{event_info, SUBSYSTEM_TICK, EVENT_TICK_INTERVAL_EXCEEDED};
///
/// let (name, tmpl) = event_info(SUBSYSTEM_TICK, EVENT_TICK_INTERVAL_EXCEEDED);
/// assert_eq!(name, "tick.interval_exceeded");
/// assert!(tmpl.contains("{arg0}") && tmpl.contains("{arg1}"));
/// ```
pub fn event_info(subsystem: u8, event: u16) -> (&'static str, &'static str) {
    match (subsystem, event) {
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_FAULT_LATCHED) => {
            ("runtime.fault_latched", "fault latched, detail={arg0}")
        }
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_ENGINE_RESET) => {
            ("runtime.engine_reset", "engine reset epoch={arg0}")
        }
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_MCU_READY) => {
            ("runtime.mcu_ready", "mcu firmware ready, log drain online")
        }
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_LOG_DROPS) => (
            "runtime.log_drops",
            "dropped {arg0} log entries (ring overflow)",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_MCU_RESET) => (
            "runtime.mcu_reset",
            "mcu reset (cause bits={arg0}, iwdg_resets={arg1})",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_HARD_FAULT) => {
            ("runtime.hard_fault", "cpu hard fault pc={arg0} lr={arg1}")
        }
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_FAULT_STATUS) => (
            "runtime.fault_status",
            "fault status cfsr={arg0} hfsr={arg1}",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_FG_FREEZE) => (
            "runtime.fg_freeze",
            "foreground freeze pc={arg0} stall_ticks={arg1}",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_RT_PROGRESS) => (
            "runtime.rt_progress",
            "runtime progress packed={arg0} fault_count={arg1}",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_LAST_DISPATCH) => (
            "runtime.last_dispatch",
            "last dispatch func={arg0} addr={arg1}",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_ISR_PHASE) => {
            ("runtime.isr_phase", "isr phase={arg0} ring_overflow={arg1}")
        }
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_BLOCK_SOURCE) => (
            "runtime.block_source",
            "block usb_burst={arg0} cyc stepout_burst={arg1} cyc",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_TIM5_IA) => (
            "runtime.tim5_ia",
            "tim5 inter-arrival min={arg0} max={arg1} cyc",
        ),
        (SUBSYSTEM_RUNTIME, EVENT_RUNTIME_DIAG_DUMP) => (
            "runtime.diag_dump",
            "live diag dump uptime_us={arg0} ring_seq={arg1}",
        ),
        (SUBSYSTEM_DIAG, EVENT_DIAG_TIM5_LONG) => {
            ("diag.tim5_long", "TIM5 ISR long {arg0} cyc at t={arg1}")
        }
        (SUBSYSTEM_DIAG, EVENT_DIAG_OTG_LONG) => {
            ("diag.otg_long", "OTG ISR long {arg0} cyc at t={arg1}")
        }
        (SUBSYSTEM_DIAG, EVENT_DIAG_USB_OUT_GAP) => (
            "diag.usb_out_gap",
            "USB-OUT gap {arg0} ticks, prev t={arg1}",
        ),
        (SUBSYSTEM_DIAG, EVENT_DIAG_USB_IN_GAP) => {
            ("diag.usb_in_gap", "USB-IN gap {arg0} ticks, prev t={arg1}")
        }
        (SUBSYSTEM_DIAG, EVENT_DIAG_TX_DROP_KAL) => (
            "diag.tx_drop_kalico",
            "kalico TX drop len={arg0} tpos={arg1}",
        ),
        (SUBSYSTEM_DIAG, EVENT_DIAG_TX_DROP_KLP) => (
            "diag.tx_drop_klipper",
            "klipper TX drop max={arg0} tpos={arg1}",
        ),
        (SUBSYSTEM_DIAG, EVENT_DIAG_ENGINE_XITION) => (
            "diag.engine_xition",
            "engine state packed={arg0} samples={arg1}",
        ),
        (SUBSYSTEM_DIAG, EVENT_DIAG_RUST_FAULT) => {
            ("diag.rust_fault", "rust fault err={arg0} detail={arg1}")
        }
        (SUBSYSTEM_MOTION, EVENT_MOTION_PIECE_START_PAST) => (
            "motion.piece_start_past",
            "piece start in past start_time={arg0} now={arg1}",
        ),
        (SUBSYSTEM_MOTION, EVENT_MOTION_RING_FULL) => {
            ("motion.ring_full", "axis ring full axis={arg0}")
        }
        (SUBSYSTEM_TICK, EVENT_TICK_INTERVAL_EXCEEDED) => (
            "tick.interval_exceeded",
            "TIM5 inter-arrival exceeded: got={arg0} limit={arg1}",
        ),
        (SUBSYSTEM_TICK, EVENT_TICK_UNDERRUN) => ("tick.underrun", "tick underrun segment={arg0}"),
        (SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_TRIP) => {
            ("endstop.trip", "endstop tripped arm={arg0} source={arg1}")
        }
        (SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_ARM_TIMEOUT) => {
            ("endstop.arm_timeout", "endstop arm timeout arm={arg0}")
        }
        (SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_TRSYNC_TRIGGER_CMD) => (
            "endstop.trsync_trigger_cmd",
            "trsync_trigger cmd oid={arg0} reason={arg1}",
        ),
        (SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_TRSYNC_DO_TRIGGER) => (
            "endstop.trsync_do_trigger",
            "trsync_do_trigger flags={arg0} reason={arg1}",
        ),
        (SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_STOP_CB_ENTER) => (
            "endstop.stop_cb_enter",
            "runtime_stop_on_trigger_cb arm_id={arg0} reason={arg1}",
        ),
        (SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_SOFTWARE_TRIP) => (
            "endstop.software_trip",
            "software_trip arg_arm_id={arg0} state_result={arg1}",
        ),
        (SUBSYSTEM_ENDSTOP, EVENT_ENDSTOP_TIM5_HALTED) => (
            "endstop.tim5_halted",
            "poll task tripped — TIM5 halted arm_id={arg0} trip_clock={arg1}",
        ),
        _ => ("unknown", ""),
    }
}

/// Compose the `_msg` string from a template and two numeric args.
///
/// # Examples
///
/// ```
/// use runtime::log_codes::compose_msg;
///
/// let msg = compose_msg("TIM5 inter-arrival exceeded: got={arg0} limit={arg1}", 120, 100);
/// assert_eq!(msg, "TIM5 inter-arrival exceeded: got=120 limit=100");
///
/// let msg2 = compose_msg("engine reset", 0, 0);
/// assert_eq!(msg2, "engine reset");
/// ```
#[cfg(feature = "host")]
pub fn compose_msg(template: &str, arg0: u32, arg1: u32) -> String {
    template
        .replace("{arg0}", &format!("{arg0}"))
        .replace("{arg1}", &format!("{arg1}"))
}

#[cfg(test)]
mod tests;
