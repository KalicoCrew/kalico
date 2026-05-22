mod bridge;
#[doc(hidden)]
pub mod cap_check;
#[doc(hidden)]
pub mod classify;
#[doc(hidden)]
pub mod config;
#[doc(hidden)]
pub mod dispatch;
#[doc(hidden)]
pub mod homing;
#[doc(hidden)]
pub mod planner;
mod router_transport;
#[doc(hidden)]
pub mod slot_pool;
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
    // Initialize env_logger so RUST_LOG=info captures bridge-trace
    // events (push_segment, seg-dispatch, etc.) into stderr. Silently
    // no-ops if already initialized (parallel pyimports).
    let _ = env_logger::try_init();
    m.add_class::<PyMotionBridge>()?;
    Ok(())
}
