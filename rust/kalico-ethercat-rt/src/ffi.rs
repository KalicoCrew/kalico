//! Callers must ensure no concurrent access and that `ec_rt_bringup` has
//! succeeded before calling any other function.
#![allow(unsafe_code)]

use std::os::raw::{c_char, c_int};

#[repr(C)]
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct EcTelemetry {
    pub error_code: u16,
    pub statusword: u16,
    pub position_actual: i32,
    pub torque_actual: i16,
    pub following_error: i32,
    pub position_demand: i32,
    pub target_position: i32,
}

const _: () = assert!(
    core::mem::size_of::<EcTelemetry>() == 24,
    "EcTelemetry layout must match ec_telemetry_t in bench/libecrt.h"
);

extern "C" {
    pub fn ec_rt_bringup(
        ifname: *const c_char,
        cycle_ns: i64,
        rt_cpu: c_int,
        rt_prio: c_int,
    ) -> c_int;

    pub fn ec_rt_enable() -> c_int;

    pub fn ec_rt_dump_al_state();

    pub fn ec_rt_cycle(toff_ns: *mut i64) -> c_int;

    pub fn ec_rt_set_target_position(counts: i32);

    pub fn ec_rt_get_position_actual() -> i32;

    pub fn ec_rt_get_statusword() -> u16;

    pub fn ec_rt_get_error_code() -> u16;

    pub fn ec_rt_get_following_error() -> i32;

    pub fn ec_rt_set_velocity_offset(counts_per_s: i32);

    pub fn ec_rt_set_torque_offset(tenths_pct: i16);

    pub fn ec_rt_get_torque_actual() -> i16;

    pub fn ec_rt_read_limits(
        ferr_counts: *mut u32,
        ferr_timeout_ms: *mut u16,
        torque_tenth_pct: *mut u16,
    ) -> c_int;

    pub fn ec_rt_write_limits(ferr_counts: u32, torque_tenth_pct: u16) -> c_int;

    pub fn ec_rt_sdo_read(
        index: u16,
        sub: u8,
        buf: *mut u8,
        size: *mut c_int,
        abort_code: *mut u32,
    ) -> c_int;

    pub fn ec_rt_sdo_write(
        index: u16,
        sub: u8,
        buf: *const u8,
        size: c_int,
        abort_code: *mut u32,
    ) -> c_int;

    pub fn ec_rt_disable();

    pub fn ec_rt_shutdown();

    pub fn ec_rt_get_telemetry(out: *mut EcTelemetry);
}
