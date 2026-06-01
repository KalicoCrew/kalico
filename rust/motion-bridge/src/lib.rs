#[doc(hidden)]
pub mod anchor;
mod bridge;
mod drain;
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
    // Initialize env_logger so bridge-trace events ([transit-diag],
    // [usb-drop], push_segment, seg-dispatch, etc.) land in a
    // persistent append file that survives plug-pull and journalctl
    // rotation.  Path resolution order:
    //   1. $KALICO_BRIDGE_LOG  (explicit override)
    //   2. $HOME/printer_data/logs/kalico-bridge.log
    //   3. /tmp/kalico-bridge.log  (last resort)
    // Default filter is "info"; $RUST_LOG overrides when set.
    // If the file cannot be opened for any reason we fall back to
    // stderr so klippy load is never broken.  `.try_init()` is a
    // silent no-op on double-init (parallel pyimports).
    let log_path = std::env::var("KALICO_BRIDGE_LOG").ok().or_else(|| {
        std::env::var("HOME")
            .ok()
            .map(|h| format!("{h}/printer_data/logs/kalico-bridge.log"))
    });
    let log_path = log_path
        .as_deref()
        .unwrap_or("/tmp/kalico-bridge.log");

    let file_result = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(log_path);

    let env = env_logger::Env::default().default_filter_or("info");
    match file_result {
        Ok(file) => {
            let _ = env_logger::Builder::from_env(env)
                .target(env_logger::Target::Pipe(Box::new(file)))
                .try_init();
        }
        Err(_) => {
            let _ = env_logger::Builder::from_env(env).try_init();
        }
    }

    m.add_class::<PyMotionBridge>()?;
    Ok(())
}
