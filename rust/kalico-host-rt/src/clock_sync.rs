//! Host-side clock-frequency estimator. Per spec §12.2, §12.4 + Plan-decision B.
//!
//! Sliding-window linear regression of MCU-clock vs host-time samples,
//! sourced from either dedicated `kalico_clock_sync_request` round-trips
//! (RTT-aware; back-calculated to the wire-send instant) or piggyback
//! samples carried by the periodic 10 Hz `kalico_status_v6` frame.
//!
//! Plan-decision B: the §12.4 quality gate adds an explicit
//! `last_dedicated_sample_age ≤ MAX_RTT_AGE_MS` check on top of the
//! residual / drift / sample-age conditions — ARMING refuses to issue
//! `kalico_stream_arm` against an estimator whose only samples are
//! piggybacks.

use std::collections::VecDeque;
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use crate::clock::{Clock, RealClock};

/// Sliding-window depth (samples). Spec §12.2.
pub const WINDOW: usize = 30;

/// Minimum samples required before the quality gate may pass.
pub const MIN_WARMUP_SAMPLES: u32 = 30;

/// Default residual threshold (µs). Spec §7.1 / §12.4.
pub const MAX_RESIDUAL_US_DEFAULT: f64 = 100.0;

/// Default drift threshold relative to the baseline freq (ppm). Spec §12.4.
pub const MAX_DRIFT_PPM_DEFAULT: f64 = 100.0;

/// Default freshness threshold for ANY sample. Spec §12.4.
pub const MAX_SAMPLE_AGE_MS_DEFAULT: u64 = 2000;

/// Plan-decision B: arm-time gate requires a recent RTT-aware
/// (dedicated) sample within this many ms. Spec §12.3 + §12.4.
pub const MAX_RTT_AGE_MS_DEFAULT: u64 = 500;

#[derive(Debug, Clone, Copy)]
pub enum SampleSource {
    /// `kalico_clock_sync_request` round-trip. RTT-aware; carries
    /// `rtt_us` and is back-calculated to the wire-send instant.
    Dedicated,
    /// Piggyback on an inbound async event (e.g. `kalico_status_v6`).
    /// `rtt_us = 0` because the host has no view of the MCU's send-side
    /// timestamp.
    Piggyback,
}

#[derive(Debug, Clone, Copy)]
pub struct Sample {
    /// Stable host-timeline coordinate (seconds since the estimator's
    /// epoch). Round-2 fix B04: regressing `mcu_clock` against
    /// `Instant::elapsed()` at recording time gives a near-zero
    /// x-coordinate; we use an epoch-anchored offset instead.
    pub host_time_secs: f64,
    pub mcu_clock: u64,
    /// Round-trip time in µs (zero for piggyback samples).
    pub rtt_us: u32,
    pub source: SampleSource,
    /// Wall-clock instant the sample landed in the estimator. Used for
    /// the freshness checks in `is_quality_gate_passed`.
    pub recorded_at: Instant,
}

pub struct ClockSyncEstimator {
    /// Round-2 B04: epoch fixed at construction; all sample
    /// `host_time_secs` are measured relative to this anchor.
    epoch: Instant,
    /// Wall-clock anchor captured at the same instant as `epoch`.
    ///
    /// Together, `wall_epoch` and `epoch` allow `wall_time_at_mcu` to
    /// map any host-time-secs offset (measured from `epoch`) to a
    /// [`SystemTime`] and thence to a [`time::OffsetDateTime`] for
    /// RFC3339 formatting.  Captured via `SystemTime::now()` — the only
    /// call site that accesses real wall time in this struct.
    wall_epoch: SystemTime,
    samples: VecDeque<Sample>,
    /// Current frequency estimate (ticks/sec). Slope of the regression
    /// line `mcu_clock = freq · host_time + offset`.
    pub clock_freq_estimate: f64,
    /// Round-2 fix B11-real: regression anchor (the window's mean of
    /// `host_time_secs`). `host_time_at` and `mcu_time_at_host` use this
    /// anchor when converting between timelines so the resulting clock
    /// value lands on the regression line — multiplying delta-t by freq
    /// alone would lose the offset and produce an absolute MCU clock
    /// that disagrees with the regression.
    anchor_host_time: f64,
    /// Mean of the window's `mcu_clock`. The offset of the regression
    /// line evaluated at `anchor_host_time`.
    anchor_mcu_clock: u64,
    /// Maximum residual (µs) over the current window. Tracked here so
    /// the quality gate doesn't have to recompute on each call.
    pub residual_max_in_window: f64,
    /// Wall-clock instant of the most recent dedicated (RTT-aware)
    /// sample. Plan-decision B uses this for the arm-time freshness
    /// check.
    pub last_dedicated_sample: Option<Instant>,
    /// Per-spec §5.9: monotonic `request_id` for this MCU's
    /// `kalico_clock_sync_request`. Lives on the estimator so the
    /// counter persists across `arm_all_mcus` retries — a delayed
    /// response from a previous arm attempt cannot collide with a
    /// fresh `request_id` and slip through the echo check.
    clock_sync_request_id: u32,
    /// Injected clock seam (spec §2.3). Routes `Instant::now()` and
    /// freshness `.elapsed()` so tests can age samples deterministically.
    clock: Arc<dyn Clock>,
}

impl std::fmt::Debug for ClockSyncEstimator {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ClockSyncEstimator")
            .field("epoch", &self.epoch)
            .field("wall_epoch", &self.wall_epoch)
            .field("samples", &self.samples)
            .field("clock_freq_estimate", &self.clock_freq_estimate)
            .field("anchor_host_time", &self.anchor_host_time)
            .field("anchor_mcu_clock", &self.anchor_mcu_clock)
            .field("residual_max_in_window", &self.residual_max_in_window)
            .field("last_dedicated_sample", &self.last_dedicated_sample)
            .field("clock_sync_request_id", &self.clock_sync_request_id)
            .finish_non_exhaustive()
    }
}

impl ClockSyncEstimator {
    /// Construct with an initial frequency estimate (e.g.
    /// `CONFIG_CLOCK_FREQ` for the target MCU). Production path; uses
    /// `RealClock` for all `now()` / freshness computations.
    pub fn new(initial_freq_estimate: f64) -> Self {
        Self::new_with_clock(initial_freq_estimate, Arc::new(RealClock))
    }

    /// Construct with an injected clock. Tests pass `MockClock` to age
    /// freshness deterministically; production callers use `new()`.
    pub fn new_with_clock(initial_freq_estimate: f64, clock: Arc<dyn Clock>) -> Self {
        let epoch = clock.now();
        // Capture the wall-clock anchor at the same logical instant as `epoch`.
        // `SystemTime::now()` is not injectable via the `Clock` seam (which
        // returns `Instant`) — this is intentional: wall_epoch is used only for
        // RFC3339 formatting, not for timing-sensitive frequency estimation.
        let wall_epoch = SystemTime::now();
        Self {
            epoch,
            wall_epoch,
            samples: VecDeque::with_capacity(WINDOW),
            clock_freq_estimate: initial_freq_estimate,
            anchor_host_time: 0.0,
            anchor_mcu_clock: 0,
            residual_max_in_window: 0.0,
            last_dedicated_sample: None,
            clock_sync_request_id: 0,
            clock,
        }
    }

    /// Convenience for tests: piggyback at `clock.now()` without forcing
    /// the caller to thread the clock.
    pub fn add_piggyback_sample_at_now(&mut self, mcu_clock_now: u64) {
        let now = self.clock.now();
        self.add_piggyback_sample(now, mcu_clock_now);
    }

    /// Allocate the next monotonic `request_id` for this MCU's
    /// `kalico_clock_sync_request`. Counter persists across arm
    /// attempts so stale responses from a prior round cannot match a
    /// fresh `request_id` (spec §5.9).
    pub fn next_clock_sync_request_id(&mut self) -> u32 {
        self.clock_sync_request_id = self.clock_sync_request_id.wrapping_add(1);
        self.clock_sync_request_id
    }

    /// Stable-timeline mapping: `Instant → seconds since estimator
    /// epoch`. Made `pub` so the ARMING flow can compose it with
    /// `mcu_time_at_host`.
    pub fn host_time_at(&self, t: Instant) -> f64 {
        t.saturating_duration_since(self.epoch).as_secs_f64()
    }

    /// Estimator's construction-time anchor. Exposed for test
    /// harnesses that need to encode synthetic MCU-clock responses
    /// off the same epoch the estimator measures `host_time_secs`
    /// against.
    pub fn epoch(&self) -> Instant {
        self.epoch
    }

    /// Convert a host-time-secs back to MCU-local clock value. Round-2
    /// fix B11-real: uses the regression anchor so the result lands on
    /// the line. This is what feeds `kalico_stream_arm`'s
    /// `t_start_t0_*` arguments.
    pub fn mcu_time_at_host(&self, host_time_secs: f64) -> u64 {
        let delta_secs = host_time_secs - self.anchor_host_time;
        // Linear regression projection: delta·freq cycles, anchored to
        // `anchor_mcu_clock`. Cast paths are bounded by the saturating
        // arithmetic that follows; we lint-allow them locally rather
        // than introduce checked-cast layering for code on the µs hot
        // path.
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

    /// Convert an MCU tick count to a host-side wall-clock time.
    ///
    /// Returns `None` when no samples have been received (estimator not yet
    /// converged).  The caller should fall back to the `Instant` stamped at
    /// decode time and set `time_estimated = true`.
    ///
    /// Returns `Some((dt, estimated))` otherwise:
    /// - `estimated = false` when `mcu_ticks` falls within the regression
    ///   window (between the minimum and maximum `mcu_clock` of the current
    ///   sample set) — the regression interpolates.
    /// - `estimated = true` when extrapolating outside the window.
    ///
    /// Inverse formula:
    /// ```text
    /// host_secs = anchor_host_time + (mcu_ticks − anchor_mcu_clock) / freq
    /// wall_time = wall_epoch + Duration::from_secs_f64(host_secs)
    /// ```
    /// Both `wall_epoch` and `epoch` are captured at construction, so
    /// `host_secs = 0` maps to `wall_epoch` exactly.
    pub fn wall_time_at_mcu(&self, mcu_ticks: u64) -> Option<(time::OffsetDateTime, bool)> {
        if self.samples.is_empty() {
            return None;
        }

        // Determine whether the query is within the regression window.
        let min_mcu = self.samples.iter().map(|s| s.mcu_clock).min().unwrap_or(0);
        let max_mcu = self.samples.iter().map(|s| s.mcu_clock).max().unwrap_or(0);
        let estimated = mcu_ticks < min_mcu || mcu_ticks > max_mcu;

        let freq = self.clock_freq_estimate;
        if freq.abs() < 1e-6 {
            // Degenerate: single sample or zero slope — return wall_epoch
            // directly with estimated=true.
            let dt = time::OffsetDateTime::from(self.wall_epoch);
            return Some((dt, true));
        }

        // Invert the regression.  Both quantities are cast to f64 before
        // subtraction so the signed delta is computed correctly even when
        // mcu_ticks < anchor_mcu_clock (which is a valid extrapolation case).
        #[allow(clippy::cast_precision_loss)]
        let delta_ticks = (mcu_ticks as f64) - (self.anchor_mcu_clock as f64);
        let host_secs = self.anchor_host_time + delta_ticks / freq;

        // Map to wall time: wall_epoch + host_secs.
        // host_secs may be negative (querying before the window) — handle
        // both directions via checked_add / checked_sub so we never panic
        // on Duration conversion of a negative value.
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

    /// Record a dedicated (RTT-aware) sample from a
    /// `kalico_clock_sync_request` round-trip.
    ///
    /// `host_send` and `host_recv` are the [`Instant`]s the request
    /// left the host and the response arrived; `mcu_at_response` is the
    /// MCU-clock value the firmware reported in
    /// `kalico_clock_sync_response`. We back-calculate to the
    /// wire-send instant by subtracting half the RTT.
    pub fn add_dedicated_sample(
        &mut self,
        host_send: Instant,
        host_recv: Instant,
        mcu_at_response: u64,
    ) {
        let rtt = host_recv.saturating_duration_since(host_send);
        let rtt_us = rtt.as_micros().min(u128::from(u32::MAX)) as u32;
        let one_way_secs = rtt.as_secs_f64() / 2.0;
        let host_time_at_send = self.host_time_at(host_send);
        // RTT/2 · freq is non-negative by construction; saturating cast
        // to u64 is safe (we'd never want a negative correction).
        #[allow(clippy::cast_sign_loss)]
        let one_way_cycles = (one_way_secs * self.clock_freq_estimate) as u64;
        let mcu_at_send = mcu_at_response.saturating_sub(one_way_cycles);
        let now = self.clock.now();
        self.add_sample(Sample {
            host_time_secs: host_time_at_send,
            mcu_clock: mcu_at_send,
            rtt_us,
            source: SampleSource::Dedicated,
            recorded_at: now,
        });
        self.last_dedicated_sample = Some(now);
    }

    /// Record a piggyback sample (e.g. periodic status frame).
    ///
    /// `host_recv` is the instant the inbound frame arrived;
    /// `mcu_clock_now` is the MCU's widened clock reported in the
    /// frame. RTT is unknown and recorded as zero.
    pub fn add_piggyback_sample(&mut self, host_recv: Instant, mcu_clock_now: u64) {
        let host_time_secs = self.host_time_at(host_recv);
        self.add_sample(Sample {
            host_time_secs,
            mcu_clock: mcu_clock_now,
            rtt_us: 0,
            source: SampleSource::Piggyback,
            recorded_at: self.clock.now(),
        });
    }

    fn add_sample(&mut self, sample: Sample) {
        if self.samples.len() == WINDOW {
            self.samples.pop_front();
        }
        self.samples.push_back(sample);
        self.recompute_regression();
    }

    fn recompute_regression(&mut self) {
        if self.samples.len() < 2 {
            return;
        }
        let n = self.samples.len() as f64;
        let mut sum_x = 0.0_f64;
        let mut sum_y = 0.0_f64;
        let mut sum_xx = 0.0_f64;
        let mut sum_xy = 0.0_f64;
        for s in &self.samples {
            let x = s.host_time_secs;
            let y = s.mcu_clock as f64;
            sum_x += x;
            sum_y += y;
            sum_xx += x * x;
            sum_xy += x * y;
        }
        let mean_x = sum_x / n;
        let mean_y = sum_y / n;
        let denom = sum_xx - n * mean_x * mean_x;
        if denom.abs() < 1e-12 {
            return;
        }
        let slope = (sum_xy - n * mean_x * mean_y) / denom;
        let offset = mean_y - slope * mean_x;
        self.clock_freq_estimate = slope;
        self.anchor_host_time = mean_x;
        // mean_y is the regression line evaluated at mean_x — by
        // construction. Cast saturating-to-u64.
        #[allow(clippy::cast_sign_loss)]
        {
            self.anchor_mcu_clock = if mean_y < 0.0 { 0 } else { mean_y as u64 };
        }

        // Residual max in seconds → µs. We invert the slope to express
        // residuals in time units rather than clock-cycles, which is
        // what spec §12.4 specifies.
        let mut max_resid_us = 0.0_f64;
        for s in &self.samples {
            let predicted = slope * s.host_time_secs + offset;
            // Convert residual cycles to seconds via slope, then µs.
            let resid_seconds = ((s.mcu_clock as f64) - predicted) / slope;
            let resid_us = (resid_seconds * 1e6).abs();
            if resid_us > max_resid_us {
                max_resid_us = resid_us;
            }
        }
        self.residual_max_in_window = max_resid_us;
    }

    /// Drift of the current frequency estimate vs `baseline_freq`, ppm.
    pub fn drift_ppm(&self, baseline_freq: f64) -> f64 {
        if baseline_freq.abs() < 1e-12 {
            return 0.0;
        }
        ((self.clock_freq_estimate - baseline_freq) / baseline_freq) * 1e6
    }

    /// Age (since recording) of the most-recent sample of any kind.
    pub fn last_sample_age(&self) -> Option<Duration> {
        let now = self.clock.now();
        self.samples
            .back()
            .map(|s| now.saturating_duration_since(s.recorded_at))
    }

    /// Age (since recording) of the most-recent dedicated (RTT-aware)
    /// sample. Plan-decision B: the arm-time gate requires this be
    /// fresh.
    pub fn last_dedicated_sample_age(&self) -> Option<Duration> {
        let now = self.clock.now();
        self.last_dedicated_sample
            .map(|t| now.saturating_duration_since(t))
    }

    pub fn sample_count(&self) -> u32 {
        self.samples.len() as u32
    }

    /// Plan-decision B: §12.4 quality gate including the dedicated-
    /// sample-present check.  Per spec §5.10 — returns structured failure
    /// reason instead of a plain bool so callers can surface diagnostics.
    pub fn is_quality_gate_passed(&self, baseline_freq: f64) -> Result<(), QualityGateFailure> {
        if self.sample_count() < MIN_WARMUP_SAMPLES {
            return Err(QualityGateFailure::InsufficientWarmup {
                samples: self.sample_count() as usize,
                required: MIN_WARMUP_SAMPLES as usize,
            });
        }
        if self.residual_max_in_window > MAX_RESIDUAL_US_DEFAULT {
            return Err(QualityGateFailure::ResidualExceeded {
                observed_us: self.residual_max_in_window,
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
