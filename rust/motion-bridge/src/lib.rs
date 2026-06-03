pub mod logging;
pub mod mcu_log;
#[doc(hidden)]
pub mod anchor;
mod bridge;
pub mod drain;
#[doc(hidden)]
pub mod motion_node;
#[doc(hidden)]
pub mod probe_homing;
#[doc(hidden)]
pub mod classify;
#[doc(hidden)]
pub mod config;
#[doc(hidden)]
pub mod dispatch;
#[doc(hidden)]
pub mod enqueue;
#[doc(hidden)]
pub mod homing;
#[doc(hidden)]
pub mod planner;
#[doc(hidden)]
pub mod pump;
mod router_transport;
mod types;

use pyo3::prelude::*;

use bridge::PyMotionBridge;

// PyO3 module name must NOT clash with the Python wrapper file
// (klippy/motion_bridge.py) — Python's package import system gives
// .so files priority for the same package member name, which silently
// shadows MotionBridgeWrapper and breaks every MCU's bridge detection.
// Hence `_native` suffix; klippy/motion_bridge.py imports this as
// `from . import motion_bridge_native as _native`.
#[pymodule]
fn motion_bridge_native(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyMotionBridge>()?;
    Ok(())
}
