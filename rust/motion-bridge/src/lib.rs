mod bridge;
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

#[pymodule]
fn motion_bridge(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyMotionBridge>()?;
    Ok(())
}
