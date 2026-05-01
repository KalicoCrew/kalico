mod bridge;
mod classify;
mod config;
mod dispatch;
mod planner;
mod router_transport;
mod types;

use pyo3::prelude::*;

use bridge::PyMotionBridge;

#[pymodule]
fn motion_bridge(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyMotionBridge>()?;
    Ok(())
}
