use std::collections::VecDeque;
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime};

use crate::clock::{Clock, RealClock};

pub const WINDOW: usize = 30;
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

#[derive(Debug, Clone, Copy)]
pub struct Sample {
    pub host_time_secs: f64,
    pub mcu_clock: u64,
    pub rtt_us: u32,
    pub source: SampleSource,
    pub recorded_at: Instant,
}

pub struct ClockSyncEstimator {
    epoch: Instant,
    wall_epoch: SystemTime,
    samples: VecDeque<Sample>,
    pub clock_freq_estimate: f64,
    anchor_host_time: f64,
    anchor_mcu_clock: u64,
    pub residual_max_in_window: f64,
    pub last_dedicated_sample: Option<Instant>,
    clock_sync_request_id: u32,
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
    pub fn new(initial_freq_estimate: f64) -> Self {
        Self::new_with_clock(initial_freq_estimate, Arc::new(RealClock))
    }

    pub fn new_with_clock(initial_freq_estimate: f64, clock: Arc<dyn Clock>) -> Self {
        let epoch = clock.now();
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

    pub fn wall_time_at_mcu(&self, mcu_ticks: u64) -> Option<(time::OffsetDateTime, bool)> {
        if self.samples.is_empty() {
            return None;
        }

        let min_mcu = self.samples.iter().map(|s| s.mcu_clock).min().unwrap_or(0);
        let max_mcu = self.samples.iter().map(|s| s.mcu_clock).max().unwrap_or(0);
        let estimated = mcu_ticks < min_mcu || mcu_ticks > max_mcu;

        let freq = self.clock_freq_estimate;
        if freq.abs() < 1e-6 {
            let dt = time::OffsetDateTime::from(self.wall_epoch);
            return Some((dt, true));
        }

        #[allow(clippy::cast_precision_loss)]
        let delta_ticks = (mcu_ticks as f64) - (self.anchor_mcu_clock as f64);
        let host_secs = self.anchor_host_time + delta_ticks / freq;

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
        #[allow(clippy::cast_sign_loss)]
        {
            self.anchor_mcu_clock = if mean_y < 0.0 { 0 } else { mean_y as u64 };
        }

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

    pub fn drift_ppm(&self, baseline_freq: f64) -> f64 {
        if baseline_freq.abs() < 1e-12 {
            return 0.0;
        }
        ((self.clock_freq_estimate - baseline_freq) / baseline_freq) * 1e6
    }

    pub fn last_sample_age(&self) -> Option<Duration> {
        let now = self.clock.now();
        self.samples
            .back()
            .map(|s| now.saturating_duration_since(s.recorded_at))
    }

    pub fn last_dedicated_sample_age(&self) -> Option<Duration> {
        let now = self.clock.now();
        self.last_dedicated_sample
            .map(|t| now.saturating_duration_since(t))
    }

    pub fn sample_count(&self) -> u32 {
        self.samples.len() as u32
    }

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
