//! Raw bindings to the C SOEM shim (`bench/libecrt`).
//!
//! All functions are unsafe: they call into a C library that owns global
//! EtherCAT state. Callers must ensure no concurrent access and that
//! `ec_rt_bringup` has succeeded before calling any other function.
#![allow(unsafe_code)]

use std::os::raw::{c_char, c_int};

// Link directives (`static=ecrt`, `static=soem`, pthread/rt/m) are owned solely
// by build.rs to keep a single source of truth; no `#[link]` attribute here.
extern "C" {
    /// Bring up the drive: set RT scheduling, init SOEM, write CSP/DC SDOs,
    /// map PDOs, reach SAFE-OP, align DC, reach OP, run CiA402 state machine.
    /// Blocks until "operation enabled". Returns 0 on success; <0 on failure
    /// (-1 init, -2 no slaves, -3 SAFE-OP, -4 OP, -5 CiA402 enable timeout).
    pub fn ec_rt_bringup(ifname: *const c_char, cycle_ns: i64, rt_cpu: c_int, rt_prio: c_int) -> c_int;

    /// One steady-state DC cycle: sleep to next deadline, exchange process
    /// data, compute DC PI correction. Writes correction to `*toff_ns`.
    /// Returns the working counter (3 == healthy for one slave).
    pub fn ec_rt_cycle(toff_ns: *mut i64) -> c_int;

    /// Stage the CSP target position (encoder counts) for the next cycle.
    pub fn ec_rt_set_target_position(counts: i32);

    /// Read the drive's position actual value (encoder counts).
    pub fn ec_rt_get_position_actual() -> i32;

    /// Read the drive's CiA402 statusword.
    pub fn ec_rt_get_statusword() -> u16;

    /// Read the drive's error code (object 0x603F).
    pub fn ec_rt_get_error_code() -> u16;

    /// Read the drive's following error (object 0x60F4).
    pub fn ec_rt_get_following_error() -> i32;

    /// Issue controlword 0x0006 (disable voltage) for ~100 cycles.
    pub fn ec_rt_disable();

    /// Disable SYNC0, return to INIT state, close the NIC.
    pub fn ec_rt_shutdown();
}
