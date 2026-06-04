use std::time::Duration;

const ALPHA: f64 = 0.125;
const BETA: f64 = 0.25;
const K: f64 = 4.0;
// 500 ms floor: prevents retransmit storms flooding firmware's 192-byte receive_buf
// during long-running command_task stalls (e.g. Renode). Driven by srtt+4×rttvar after first sample.
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
