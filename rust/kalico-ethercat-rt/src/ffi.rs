//! Callers must ensure no concurrent access and that `ec_rt_bringup` has
//! succeeded before calling any other function.
#![allow(unsafe_code)]

use std::os::raw::{c_char, c_int};

/// Mirror of `ec_telemetry_t` in bench/libecrt.h — natural (unpacked) C layout.
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

extern "C" {
    pub fn ec_rt_bringup(
        ifname: *const c_char,
        cycle_ns: i64,
        rt_cpu: c_int,
        rt_prio: c_int,
    ) -> c_int;

    pub fn ec_rt_enable() -> c_int;

    pub fn ec_rt_cycle(toff_ns: *mut i64) -> c_int;

    pub fn ec_rt_set_target_position(counts: i32);

    pub fn ec_rt_get_position_actual() -> i32;

    pub fn ec_rt_get_statusword() -> u16;

    pub fn ec_rt_get_error_code() -> u16;

    pub fn ec_rt_get_following_error() -> i32;

    pub fn ec_rt_disable();

    pub fn ec_rt_shutdown();

    pub fn ec_rt_get_telemetry(out: *mut EcTelemetry);
}
