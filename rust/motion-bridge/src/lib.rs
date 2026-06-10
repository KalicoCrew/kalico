#[doc(hidden)]
pub mod anchor;
mod bridge;
#[doc(hidden)]
pub mod classify;
#[doc(hidden)]
pub mod config;
#[doc(hidden)]
pub mod dispatch;
pub mod drain;
#[doc(hidden)]
pub mod enqueue;
#[doc(hidden)]
pub mod homing;
#[doc(hidden)]
pub mod kinematics;
pub mod logging;
pub mod mcu_log;
pub mod motion_history;
#[doc(hidden)]
pub mod motion_node;
#[doc(hidden)]
pub mod planner;
#[doc(hidden)]
pub mod pump;
mod router_transport;
#[doc(hidden)]
pub mod servo_torque;
mod types;

pub mod test_support;

use pyo3::prelude::*;

use bridge::PyMotionBridge;

// `_native` suffix: without it the .so shadows klippy/motion_bridge.py and breaks bridge detection.
#[pymodule]
fn motion_bridge_native(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyMotionBridge>()?;
    Ok(())
}
