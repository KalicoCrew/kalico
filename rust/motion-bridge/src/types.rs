//! Helper conversions between Rust passthrough_queue types and Python.

use pyo3::prelude::*;
use pyo3::types::PyDict;

use kalico_host_rt::passthrough_queue::{CommandQueueId, McuHandle, PassthroughStats};

/// Wrap a raw `u32` back into an `McuHandle`.
pub(crate) fn mcu_handle_from_raw(raw: u32) -> McuHandle {
    McuHandle::from_raw(raw)
}

/// Wrap a raw `u32` back into a `CommandQueueId`.
pub(crate) fn cq_id_from_raw(raw: u32) -> CommandQueueId {
    CommandQueueId::from_raw(raw)
}

/// Convert `PassthroughStats` to a Python dict.
pub(crate) fn stats_to_pydict(py: Python<'_>, s: &PassthroughStats) -> PyResult<Py<PyDict>> {
    let d = PyDict::new(py);
    d.set_item("bytes_write", s.bytes_write)?;
    d.set_item("bytes_read", s.bytes_read)?;
    d.set_item("bytes_retransmit", s.bytes_retransmit)?;
    d.set_item("bytes_invalid", s.bytes_invalid)?;
    d.set_item("send_seq", s.send_seq)?;
    d.set_item("receive_seq", s.receive_seq)?;
    d.set_item("retransmit_seq", s.retransmit_seq)?;
    d.set_item("ready_bytes", s.ready_bytes)?;
    d.set_item("upcoming_bytes", s.upcoming_bytes)?;
    d.set_item("stalled_bytes", s.stalled_bytes)?;
    Ok(d.unbind())
}
