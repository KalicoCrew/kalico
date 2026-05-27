//! RFC 6298 RTT estimator. Spec §3.9.

use std::time::Duration;

const ALPHA: f64 = 0.125;
const BETA: f64 = 0.25;
const K: f64 = 4.0;
/// Floor on the RFC 6298 retransmission timeout.
///
/// The original 25 ms was tuned for the Klipper C reference implementation
/// where RTT on hardware is in microseconds. In our Rust transport, when the
/// MCU is in the middle of a long-running command (e.g. LoadCurve under the
/// Renode 1 µs quantum, where command_task can hold the CPU for seconds),
/// retransmits at 25→50→100→200 ms intervals flood firmware's 192-byte
/// `receive_buf` with stale Klipper bytes faster than the firmware can drain
/// them. Each stale frame produces a NAK; the NAKs queue in firmware's
/// 320-byte `transmit_buf`, and once it overflows `console_sendf` silently
/// drops the next response — including the response the host is waiting on.
///
/// 500 ms is well above the worst observed command_task stall on real
/// silicon (<10 ms) yet still tight enough that an actually-dropped frame
/// retransmits within half a second. Once the RTT estimator gets a single
/// real sample, `current_rto()` is driven by `srtt + 4×rttvar` so this
/// floor only matters before the first sample — which is exactly the
/// retransmit-storm window we want to slow down.
pub const MIN_RTO: Duration = Duration::from_millis(500);
pub const MAX_RTO: Duration = Duration::from_secs(5);
const G: Duration = Duration::from_millis(1);

#[derive(Debug)]
pub struct RttEstimator {
    srtt: Option<Duration>,
    rttvar: Option<Duration>,
    rto: Duration,
}

impl Default for RttEstimator {
    fn default() -> Self {
        Self {
            srtt: None,
            rttvar: None,
            rto: MIN_RTO,
        }
    }
}

impl RttEstimator {
    pub fn current_rto(&self) -> Duration {
        self.rto
    }
}

fn secs_mul(d: Duration, f: f64) -> Duration {
    Duration::from_secs_f64(d.as_secs_f64() * f)
}

fn clamp(d: Duration, min: Duration, max: Duration) -> Duration {
    if d < min {
        min
    } else if d > max {
        max
    } else {
        d
    }
}

impl RttEstimator {
    pub fn update(&mut self, r: Duration) {
        match self.srtt {
            None => {
                self.srtt = Some(r);
                self.rttvar = Some(r / 2);
            }
            Some(srtt) => {
                let diff = if srtt > r { srtt - r } else { r - srtt };
                let rttvar_new = secs_mul(self.rttvar.unwrap(), 1.0 - BETA) + secs_mul(diff, BETA);
                self.rttvar = Some(rttvar_new);
                self.srtt = Some(secs_mul(srtt, 1.0 - ALPHA) + secs_mul(r, ALPHA));
            }
        }
        let rttvar = self.rttvar.unwrap();
        let k_rttvar = secs_mul(rttvar, K);
        let rto_raw = self.srtt.unwrap() + std::cmp::max(G, k_rttvar);
        self.rto = clamp(rto_raw, MIN_RTO, MAX_RTO);
    }

    pub fn backoff(&mut self) {
        self.rto = clamp(self.rto * 2, MIN_RTO, MAX_RTO);
    }
}

#[cfg(test)]
mod tests;
