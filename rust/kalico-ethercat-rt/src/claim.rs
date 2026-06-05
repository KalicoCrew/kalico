//! Helpers shared between the hw endpoint and the no-hardware stub.
//!
//! Both binaries perform the same spawn-on-claim handshake and build the same
//! single-slave reply shape; keeping them in one place avoids drift.

use std::sync::atomic::{AtomicBool, Ordering};

use kalico_protocol::messages::{ClaimHandshakeReply, SlaveState, SlaveStatus};

use crate::server::FrameServer;
use crate::wire::Command;

/// Poll `server` for a [`Command::ClaimHandshake`] until `deadline` or until
/// `sigterm` is set.
///
/// Returns the `correlation_id` on success, `None` on timeout or SIGTERM.
/// Any non-`ClaimHandshake` command received before the handshake is logged and
/// dropped — the bridge must not send operational traffic before claiming.
pub fn wait_for_claim(
    server: &mut FrameServer,
    deadline: std::time::Instant,
    sigterm: &AtomicBool,
) -> Option<u32> {
    loop {
        for cmd in server.poll_commands() {
            if let Command::ClaimHandshake { correlation_id } = cmd {
                return Some(correlation_id);
            }
            eprintln!("ec-rt: unexpected pre-handshake command: {cmd:?}");
        }
        if std::time::Instant::now() >= deadline {
            return None;
        }
        if sigterm.load(Ordering::Acquire) {
            return None;
        }
        std::thread::sleep(std::time::Duration::from_millis(1));
    }
}

/// Build a [`ClaimHandshakeReply`] with a single slave entry at `slave_idx = 1`.
pub fn single_slave_reply(state: SlaveState, fault_code: u16) -> ClaimHandshakeReply {
    ClaimHandshakeReply {
        slave_statuses: vec![SlaveStatus {
            slave_idx: 1,
            state,
            fault_code,
        }],
    }
}

/// Parse the `--fail-bringup slave=N` argument from a flat argv slice.
///
/// Returns:
/// - `Ok(None)` — flag absent.
/// - `Ok(Some(n))` — flag present and `slave=N` is a valid `u8`.
/// - `Err(msg)` — flag present but value is malformed; `msg` is a human-readable
///   description suitable for an `eprintln!` usage line.
///
/// # Examples
///
/// ```
/// use kalico_ethercat_rt::claim::parse_fail_bringup;
///
/// let args = ["--socket", "/tmp/s.sock", "--fail-bringup", "slave=2"]
///     .map(String::from);
/// assert_eq!(parse_fail_bringup(&args), Ok(Some(2)));
///
/// let args_absent = ["--socket", "/tmp/s.sock"].map(String::from);
/// assert_eq!(parse_fail_bringup(&args_absent), Ok(None));
///
/// let args_bad = ["--fail-bringup", "banana"].map(String::from);
/// assert!(parse_fail_bringup(&args_bad).is_err());
/// ```
pub fn parse_fail_bringup(args: &[String]) -> Result<Option<u8>, String> {
    let Some(pos) = args.iter().position(|a| a == "--fail-bringup") else {
        return Ok(None);
    };
    let value = args
        .get(pos + 1)
        .ok_or_else(|| "missing value after --fail-bringup; expected slave=N".to_owned())?;
    let n_str = value
        .strip_prefix("slave=")
        .ok_or_else(|| format!("--fail-bringup value must be slave=N, got {value:?}"))?;
    let n: u8 = n_str
        .parse()
        .map_err(|_| format!("--fail-bringup slave index must be u8 (0–255), got {n_str:?}"))?;
    Ok(Some(n))
}

#[cfg(test)]
mod tests {
    use super::parse_fail_bringup;

    fn args(v: &[&str]) -> Vec<String> {
        v.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn absent_returns_none() {
        assert_eq!(
            parse_fail_bringup(&args(&["--socket", "/tmp/s.sock"])),
            Ok(None)
        );
    }

    #[test]
    fn present_valid() {
        assert_eq!(
            parse_fail_bringup(&args(&["--fail-bringup", "slave=3"])),
            Ok(Some(3))
        );
    }

    #[test]
    fn present_slave_zero() {
        assert_eq!(
            parse_fail_bringup(&args(&["--fail-bringup", "slave=0"])),
            Ok(Some(0))
        );
    }

    #[test]
    fn present_slave_max_u8() {
        assert_eq!(
            parse_fail_bringup(&args(&["--fail-bringup", "slave=255"])),
            Ok(Some(255))
        );
    }

    #[test]
    fn malformed_not_slave_prefix() {
        assert!(parse_fail_bringup(&args(&["--fail-bringup", "banana"])).is_err());
    }

    #[test]
    fn malformed_overflow() {
        assert!(parse_fail_bringup(&args(&["--fail-bringup", "slave=256"])).is_err());
    }

    #[test]
    fn malformed_non_numeric() {
        assert!(parse_fail_bringup(&args(&["--fail-bringup", "slave=abc"])).is_err());
    }

    #[test]
    fn missing_value_after_flag() {
        assert!(parse_fail_bringup(&args(&["--fail-bringup"])).is_err());
    }

    #[test]
    fn flag_not_last_other_args_after() {
        assert_eq!(
            parse_fail_bringup(&args(&["--fail-bringup", "slave=7", "--socket", "/tmp/s"])),
            Ok(Some(7))
        );
    }
}
