use pyo3::prelude::*;

#[pyclass(name = "MotionBridge")]
#[derive(Debug)]
pub struct PyMotionBridge {
    _placeholder: (),
}

#[pymethods]
impl PyMotionBridge {
    #[new]
    fn new() -> Self {
        Self { _placeholder: () }
    }

    fn version(&self) -> &'static str {
        env!("CARGO_PKG_VERSION")
    }
}

#[pymodule]
fn motion_bridge(_py: Python<'_>, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyMotionBridge>()?;
    Ok(())
}
