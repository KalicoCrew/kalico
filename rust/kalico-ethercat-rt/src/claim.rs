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
/// `prefix` is prepended to diagnostic messages (e.g. `"ec-rt"` or
/// `"ec-rt-stub"`).
///
/// Returns the `correlation_id` on success, `None` on timeout or SIGTERM.
/// Any non-`ClaimHandshake` command received before the handshake is logged and
/// dropped — the bridge must not send operational traffic before claiming.
pub fn wait_for_claim(
    server: &mut FrameServer,
    deadline: std::time::Instant,
    sigterm: &AtomicBool,
    prefix: &str,
) -> Option<u32> {
    loop {
        for cmd in server.poll_commands() {
            if let Command::ClaimHandshake { correlation_id } = cmd {
                return Some(correlation_id);
            }
            eprintln!("{prefix}: unexpected pre-handshake command: {cmd:?}");
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

/// Build a [`ClaimHandshakeReply`] with a single slave entry at the given
/// `slave_idx`.
pub fn single_slave_reply(
    slave_idx: u8,
    state: SlaveState,
    fault_code: u16,
) -> ClaimHandshakeReply {
    ClaimHandshakeReply {
        slave_statuses: vec![SlaveStatus {
            slave_idx,
            state,
            fault_code,
        }],
    }
}

/// Maximum number of consecutive bad working-counter cycles before the DC loop
/// halts and exits.
///
/// One bad cycle is tolerated because USB-NIC adapters occasionally drop a
/// single Ethernet frame under load; the drive's own SM communication watchdog
/// (~100 ms typical) is the authoritative hardware backstop for sustained loss.
/// Two consecutive bad cycles means the bus is genuinely gone and the drive is
/// no longer receiving valid PDO exchanges.
pub const WKC_CONSECUTIVE_LOSS_LIMIT: u8 = 2;

/// Outcome of evaluating a single working-counter sample.
#[derive(Debug, PartialEq, Eq)]
pub enum WkcDecision {
    /// Working counter is good; counter has been reset.
    Good,
    /// One or more bad cycles accumulated, but below the halt threshold.
    /// The embedded value is the current consecutive-loss count.
    Warn(u8),
    /// Consecutive bad cycles reached the limit; caller must halt.
    Halt,
}

/// Evaluate one working-counter reading against the expected value.
///
/// `consecutive` is the mutable counter that the caller must keep across calls.
/// It is reset to 0 on a good cycle and incremented on a bad one.
///
/// # Examples
///
/// ```
/// use kalico_ethercat_rt::claim::{WkcDecision, WKC_CONSECUTIVE_LOSS_LIMIT, eval_wkc};
///
/// let expected = 3i32;
/// let mut consecutive = 0u8;
///
/// // Good cycle — counter stays 0.
/// assert_eq!(eval_wkc(3, expected, &mut consecutive), WkcDecision::Good);
/// assert_eq!(consecutive, 0);
///
/// // First bad cycle — warn, do not halt.
/// assert_eq!(eval_wkc(-1, expected, &mut consecutive), WkcDecision::Warn(1));
/// assert_eq!(consecutive, 1);
///
/// // Good cycle after one bad — counter resets.
/// assert_eq!(eval_wkc(3, expected, &mut consecutive), WkcDecision::Good);
/// assert_eq!(consecutive, 0);
///
/// // Two consecutive bad cycles — halt.
/// eval_wkc(-1, expected, &mut consecutive);
/// assert_eq!(eval_wkc(-1, expected, &mut consecutive), WkcDecision::Halt);
/// ```
pub fn eval_wkc(wkc: i32, expected: i32, consecutive: &mut u8) -> WkcDecision {
    if wkc == expected {
        *consecutive = 0;
        WkcDecision::Good
    } else {
        *consecutive = consecutive.saturating_add(1);
        if *consecutive >= WKC_CONSECUTIVE_LOSS_LIMIT {
            WkcDecision::Halt
        } else {
            WkcDecision::Warn(*consecutive)
        }
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
mod tests;
