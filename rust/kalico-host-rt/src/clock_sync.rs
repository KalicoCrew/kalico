use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use crate::clock::{Clock, RealClock};

/// EWMA decay factor — mirrors klippy's `DECAY = 1/30`.
const DECAY: f64 = 1.0 / 30.0;

/// Half-RTT age coefficient — mirrors klippy's `RTT_AGE = 10µs / 3600s`.
const RTT_AGE: f64 = 0.000_010 / (60.0 * 60.0);

/// Prediction-variance is reset to `(1ms * freq)²` on a variance reset; the
/// same formula klippy uses.
const PREDICTION_RESET_MS: f64 = 0.001;

/// Outlier gate: residual² must exceed this multiple of prediction_variance AND
/// the absolute floor below to be considered an outlier.  Matches klippy's 25×.
const OUTLIER_VARIANCE_MULT: f64 = 25.0;

/// Absolute outlier floor: residual must also exceed 500µs × freq to gate.
/// Matches klippy's `(0.000500 * self.mcu_freq)²`.
const OUTLIER_ABS_FLOOR_SECS: f64 = 0.000_500;

/// After how long without a progressive prediction update do we allow an
/// "upward" outlier through (klippy: `sent_time < last_prediction_time + 10`).
const OUTLIER_RESET_WINDOW_SECS: f64 = 10.0;

pub const MIN_WARMUP_SAMPLES: u32 = 30;
pub const MAX_RESIDUAL_US_DEFAULT: f64 = 100.0;
pub const MAX_DRIFT_PPM_DEFAULT: f64 = 100.0;
pub const MAX_SAMPLE_AGE_MS_DEFAULT: u64 = 2000;
pub const MAX_RTT_AGE_MS_DEFAULT: u64 = 500;

#[derive(Debug, Clone, Copy)]
pub enum SampleSource {
    Dedicated,
    Piggyback,
}

/// A single RTT-qualified sample fed into the estimator.
#[derive(Debug, Clone, Copy)]
pub struct Sample {
    pub host_time_secs: f64,
    pub mcu_clock: u64,
    pub rtt_us: u32,
    pub source: SampleSource,
    pub recorded_at: Instant,
}

/// EWMA-based clock frequency / offset estimator.
///
/// Mirrors klippy's `ClockSync._handle_clock` algorithm:
/// - Decayed accumulators (`time_avg`, `clock_avg`, `time_variance`,
///   `clock_covariance`) with `DECAY = 1/30`.
/// - Prediction-variance outlier gate (25× + 500µs absolute floor).
/// - `min_half_rtt` / `RTT_AGE` logic for minimal-RTT sample selection.
///
/// Public API is unchanged from the window-regression version so `bridge.rs`
/// callers need no changes.
pub struct ClockSyncEstimator {
    epoch: Instant,
    wall_epoch: SystemTime,

    // EWMA accumulators (klippy naming kept for auditability).
    time_avg: f64,
    time_variance: f64,
    clock_avg: f64,
    clock_covariance: f64,
    prediction_variance: f64,
    last_prediction_time: f64,

    // min-RTT tracking.
    min_half_rtt: f64,
    min_rtt_time: f64,

    // Derived fit exported to callers.
    pub clock_freq_estimate: f64,
    anchor_host_time: f64,
    anchor_mcu_clock: u64,

    // Diagnostics.
    pub residual_ewma_us: f64,
    pub last_dedicated_sample: Option<Instant>,

    // For `add_dedicated_sample` age gate (same role as the old window).
    last_sample_recorded_at: Option<Instant>,

    sample_count: u32,
    clock_sync_request_id: u32,
    clock: Arc<dyn Clock>,

    /// Initial freq kept for the prediction-variance reset formula.
    mcu_freq: f64,
}

impl std::fmt::Debug for ClockSyncEstimator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ClockSyncEstimator")
            .field("epoch", &self.epoch)
            .field("clock_freq_estimate", &self.clock_freq_estimate)
            .field("anchor_host_time", &self.anchor_host_time)
            .field("anchor_mcu_clock", &self.anchor_mcu_clock)
            .field("time_avg", &self.time_avg)
            .field("clock_avg", &self.clock_avg)
            .field("prediction_variance", &self.prediction_variance)
            .field("min_half_rtt", &self.min_half_rtt)
            .field("sample_count", &self.sample_count)
            .field("residual_ewma_us", &self.residual_ewma_us)
            .field("last_dedicated_sample", &self.last_dedicated_sample)
            .finish_non_exhaustive()
    }
}

impl ClockSyncEstimator {
    pub fn new(initial_freq_estimate: f64) -> Self {
        Self::new_with_clock(initial_freq_estimate, Arc::new(RealClock))
    }

    pub fn new_with_clock(initial_freq_estimate: f64, clock: Arc<dyn Clock>) -> Self {
        let epoch = clock.now();
        let wall_epoch = SystemTime::now();
        Self {
            epoch,
            wall_epoch,
            time_avg: 0.0,
            time_variance: 0.0,
            clock_avg: 0.0,
            clock_covariance: 0.0,
            prediction_variance: (PREDICTION_RESET_MS * initial_freq_estimate).powi(2),
            last_prediction_time: -9999.0,
            min_half_rtt: 999_999_999.9,
            min_rtt_time: 0.0,
            clock_freq_estimate: initial_freq_estimate,
            anchor_host_time: 0.0,
            anchor_mcu_clock: 0,
            residual_ewma_us: 0.0,
            last_dedicated_sample: None,
            last_sample_recorded_at: None,
            sample_count: 0,
            clock_sync_request_id: 0,
            clock,
            mcu_freq: initial_freq_estimate,
        }
    }

    pub fn add_piggyback_sample_at_now(&mut self, mcu_clock_now: u64) {
        let now = self.clock.now();
        self.add_piggyback_sample(now, mcu_clock_now);
    }

    pub fn next_clock_sync_request_id(&mut self) -> u32 {
        self.clock_sync_request_id = self.clock_sync_request_id.wrapping_add(1);
        self.clock_sync_request_id
    }

    pub fn host_time_at(&self, t: Instant) -> f64 {
        t.saturating_duration_since(self.epoch).as_secs_f64()
    }

    pub fn epoch(&self) -> Instant {
        self.epoch
    }

    /// Project a host-time to MCU clock using the current fit.
    pub fn mcu_time_at_host(&self, host_time_secs: f64) -> u64 {
        let delta_secs = host_time_secs - self.anchor_host_time;
        #[allow(
            clippy::cast_possible_truncation,
            clippy::cast_possible_wrap,
            clippy::cast_sign_loss
        )]
        {
            let delta_cycles = (delta_secs * self.clock_freq_estimate) as i64;
            let base = self.anchor_mcu_clock as i64;
            (base.saturating_add(delta_cycles).max(0)) as u64
        }
    }

    /// Convert an MCU tick count to a wall-clock `OffsetDateTime`.
    /// Returns `None` before the first sample, `(time, estimated)` where
    /// `estimated=true` means the tick is outside the observed range.
    pub fn wall_time_at_mcu(&self, mcu_ticks: u64) -> Option<(time::OffsetDateTime, bool)> {
        if self.sample_count == 0 {
            return None;
        }

        let freq = self.clock_freq_estimate;
        if freq.abs() < 1e-6 {
            let dt = time::OffsetDateTime::from(self.wall_epoch);
            return Some((dt, true));
        }

        // "estimated" = extrapolation (we have no window bounds anymore, so we
        // treat any tick that is farther than 1 freq-second from the anchor as
        // estimated, which is a conservative proxy).
        #[allow(clippy::cast_precision_loss)]
        let delta_ticks = (mcu_ticks as f64) - (self.anchor_mcu_clock as f64);
        let host_secs = self.anchor_host_time + delta_ticks / freq;
        let estimated = delta_ticks.abs() / freq > 1.0;

        let wall_time = if host_secs >= 0.0 {
            self.wall_epoch
                .checked_add(Duration::from_secs_f64(host_secs))
                .unwrap_or(self.wall_epoch)
        } else {
            self.wall_epoch
                .checked_sub(Duration::from_secs_f64(-host_secs))
                .unwrap_or(self.wall_epoch)
        };

        Some((time::OffsetDateTime::from(wall_time), estimated))
    }

    /// Feed one RTT-qualified dedicated sample.
    ///
    /// `mcu_at_response` is the MCU clock value echoed in the response (i.e.
    /// the MCU clock at the instant the firmware generated its reply, which is
    /// approximately `send_time + half_rtt` in host time).  The one-way delay
    /// is subtracted so the regression maps `send_time → mcu_at_send`, keeping
    /// the y-axis consistent with piggyback samples (which also carry the
    /// instantaneous MCU clock at the observed host time).
    ///
    /// The `min_half_rtt` tracker selects the best-observed RTT; both the
    /// subtraction and the projection anchor use it so that high-RTT samples
    /// do not distort the slope estimate.
    pub fn add_dedicated_sample(
        &mut self,
        host_send: Instant,
        host_recv: Instant,
        mcu_at_response: u64,
    ) {
        let rtt = host_recv.saturating_duration_since(host_send);
        let half_rtt = rtt.as_secs_f64() / 2.0;
        let sent_time = self.host_time_at(host_send);

        // min-RTT gate (klippy: `if half_rtt < self.min_half_rtt + aged_rtt`).
        let aged_rtt = (sent_time - self.min_rtt_time) * RTT_AGE;
        if half_rtt < self.min_half_rtt + aged_rtt {
            self.min_half_rtt = half_rtt;
            self.min_rtt_time = sent_time;
        }

        // Back-correct MCU clock to the send instant using current freq estimate.
        // Using `min_half_rtt` (not the raw `half_rtt`) mirrors klippy's approach
        // of anchoring projections at the minimum-RTT estimate.
        let effective_half_rtt = if self.min_half_rtt < 1.0 {
            self.min_half_rtt
        } else {
            half_rtt
        };
        #[allow(clippy::cast_sign_loss)]
        let one_way_cycles =
            (effective_half_rtt * self.clock_freq_estimate).max(0.0) as u64;
        let mcu_at_send = mcu_at_response.saturating_sub(one_way_cycles);

        #[allow(clippy::cast_precision_loss)]
        let feed_clock = mcu_at_send as f64;
        self.ingest(sent_time, feed_clock, sent_time);

        let now = self.clock.now();
        self.last_dedicated_sample = Some(now);
        self.last_sample_recorded_at = Some(now);
    }

    pub fn add_piggyback_sample(&mut self, host_recv: Instant, mcu_clock_now: u64) {
        let sent_time = self.host_time_at(host_recv);
        #[allow(clippy::cast_precision_loss)]
        let feed_clock = mcu_clock_now as f64;
        let now = self.clock.now();
        self.ingest(sent_time, feed_clock, sent_time);
        self.last_sample_recorded_at = Some(now);
    }

    /// Core EWMA update — port of klippy's `_handle_clock` accumulator logic.
    ///
    /// `sent_time`  — host time of the send (used for outlier gating).
    /// `feed_time`  — the x-value fed into the regression (identical to
    ///                `sent_time` for both dedicated and piggyback callers).
    /// `clock`      — the y-value (MCU clock ticks at `feed_time`).
    fn ingest(&mut self, feed_time: f64, clock: f64, sent_time: f64) {
        // First sample: seed the EWMA accumulators directly (mirrors klippy's
        // `connect()` which initialises `time_avg` and `clock_avg` from the
        // first `get_uptime` response before entering the EWMA loop).
        if self.sample_count == 0 {
            self.time_avg = feed_time;
            self.clock_avg = clock;
            self.prediction_variance = (PREDICTION_RESET_MS * self.mcu_freq).powi(2);
            self.last_prediction_time = sent_time;
            self.sample_count = 1;
            // No variance/covariance update; need at least two points for a slope.
            return;
        }

        // --- outlier gate (klippy lines ~122-155) ---
        let exp_clock =
            (sent_time - self.time_avg) * self.clock_freq_estimate + self.clock_avg;
        let clock_diff = clock - exp_clock;
        let clock_diff2 = clock_diff * clock_diff;
        let abs_floor = (OUTLIER_ABS_FLOOR_SECS * self.mcu_freq).powi(2);

        if clock_diff2 > OUTLIER_VARIANCE_MULT * self.prediction_variance
            && clock_diff2 > abs_floor
        {
            if clock > exp_clock
                && sent_time < self.last_prediction_time + OUTLIER_RESET_WINDOW_SECS
            {
                // High-side outlier within the reset window: skip entirely.
                return;
            }
            // Reset prediction variance (klippy: variance reset path).
            self.prediction_variance =
                (PREDICTION_RESET_MS * self.mcu_freq).powi(2);
        } else {
            self.last_prediction_time = sent_time;
            self.prediction_variance = (1.0 - DECAY)
                * (self.prediction_variance + clock_diff2 * DECAY);
        }

        // Residual quality metric: EWMA of |clock_diff| in µs.  This is the
        // per-sample prediction error, directly comparable to the old window
        // regression's `residual_ewma_us`.
        if self.clock_freq_estimate > 1.0 {
            let abs_resid_us = clock_diff.abs() / self.clock_freq_estimate * 1e6;
            self.residual_ewma_us = (1.0 - DECAY) * self.residual_ewma_us
                + DECAY * abs_resid_us;
        }

        // --- EWMA accumulators (klippy lines ~157-165) ---
        let diff_sent = feed_time - self.time_avg;
        self.time_avg += DECAY * diff_sent;
        self.time_variance =
            (1.0 - DECAY) * (self.time_variance + diff_sent * diff_sent * DECAY);

        let diff_clock = clock - self.clock_avg;
        self.clock_avg += DECAY * diff_clock;
        self.clock_covariance =
            (1.0 - DECAY) * (self.clock_covariance + diff_sent * diff_clock * DECAY);

        self.sample_count = self.sample_count.saturating_add(1);

        // --- derive slope + anchor ---
        self.update_fit();
    }

    /// Recompute `clock_freq_estimate` and the projection anchor from the
    /// current EWMA accumulators.  Called after every accepted sample.
    fn update_fit(&mut self) {
        if self.time_variance.abs() < 1e-12 {
            return;
        }
        let new_freq = self.clock_covariance / self.time_variance;
        if new_freq < 1.0 {
            // Guard against degenerate state on cold start.
            return;
        }
        self.clock_freq_estimate = new_freq;

        // Anchor at `time_avg + min_half_rtt` (mirrors klippy's
        // `clock_est = (time_avg + min_half_rtt, clock_avg, new_freq)`).
        // Use `min_half_rtt` only when it has been updated from an actual RTT
        // measurement; the initial sentinel (999_999_999.9) must not shift the
        // anchor.
        let effective_min_half_rtt = if self.min_half_rtt < 1.0 {
            self.min_half_rtt
        } else {
            0.0
        };
        self.anchor_host_time = self.time_avg + effective_min_half_rtt;
        #[allow(clippy::cast_sign_loss)]
        {
            self.anchor_mcu_clock = if self.clock_avg < 0.0 {
                0
            } else {
                self.clock_avg as u64
            };
        }

    }

    pub fn drift_ppm(&self, baseline_freq: f64) -> f64 {
        if baseline_freq.abs() < 1e-12 {
            return 0.0;
        }
        ((self.clock_freq_estimate - baseline_freq) / baseline_freq) * 1e6
    }

    pub fn last_sample_age(&self) -> Option<Duration> {
        let now = self.clock.now();
        self.last_sample_recorded_at
            .map(|t| now.saturating_duration_since(t))
    }

    pub fn last_dedicated_sample_age(&self) -> Option<Duration> {
        let now = self.clock.now();
        self.last_dedicated_sample
            .map(|t| now.saturating_duration_since(t))
    }

    pub fn sample_count(&self) -> u32 {
        self.sample_count
    }

    pub fn is_quality_gate_passed(&self, baseline_freq: f64) -> Result<(), QualityGateFailure> {
        if self.sample_count() < MIN_WARMUP_SAMPLES {
            return Err(QualityGateFailure::InsufficientWarmup {
                samples: self.sample_count() as usize,
                required: MIN_WARMUP_SAMPLES as usize,
            });
        }
        if self.residual_ewma_us > MAX_RESIDUAL_US_DEFAULT {
            return Err(QualityGateFailure::ResidualExceeded {
                observed_us: self.residual_ewma_us,
                max_us: MAX_RESIDUAL_US_DEFAULT,
            });
        }
        let drift = self.drift_ppm(baseline_freq).abs();
        if drift > MAX_DRIFT_PPM_DEFAULT {
            return Err(QualityGateFailure::DriftPpmExceeded {
                observed_ppm: drift,
                max_ppm: MAX_DRIFT_PPM_DEFAULT,
            });
        }
        match self.last_sample_age() {
            Some(age) if age.as_millis() <= u128::from(MAX_SAMPLE_AGE_MS_DEFAULT) => {}
            Some(age) => {
                return Err(QualityGateFailure::LastSampleStale {
                    age,
                    max_age: std::time::Duration::from_millis(MAX_SAMPLE_AGE_MS_DEFAULT),
                });
            }
            None => {
                return Err(QualityGateFailure::LastSampleStale {
                    age: std::time::Duration::MAX,
                    max_age: std::time::Duration::from_millis(MAX_SAMPLE_AGE_MS_DEFAULT),
                });
            }
        }
        match self.last_dedicated_sample_age() {
            Some(age) if age.as_millis() <= u128::from(MAX_RTT_AGE_MS_DEFAULT) => {}
            Some(age) => {
                return Err(QualityGateFailure::DedicatedSampleStale {
                    age,
                    max_age: std::time::Duration::from_millis(MAX_RTT_AGE_MS_DEFAULT),
                });
            }
            None => {
                return Err(QualityGateFailure::DedicatedSampleStale {
                    age: std::time::Duration::MAX,
                    max_age: std::time::Duration::from_millis(MAX_RTT_AGE_MS_DEFAULT),
                });
            }
        }
        Ok(())
    }
}

#[derive(Debug)]
pub enum QualityGateFailure {
    InsufficientWarmup {
        samples: usize,
        required: usize,
    },
    ResidualExceeded {
        observed_us: f64,
        max_us: f64,
    },
    DriftPpmExceeded {
        observed_ppm: f64,
        max_ppm: f64,
    },
    LastSampleStale {
        age: std::time::Duration,
        max_age: std::time::Duration,
    },
    DedicatedSampleStale {
        age: std::time::Duration,
        max_age: std::time::Duration,
    },
}

impl std::fmt::Display for QualityGateFailure {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

#[cfg(test)]
mod clock_seam_tests;
