//! Callers must ensure no concurrent access and that `ec_rt_bringup` has
//! succeeded before calling any other function.
#![allow(unsafe_code)]

use std::os::raw::{c_char, c_int};

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
}
