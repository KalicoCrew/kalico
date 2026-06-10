use std::collections::{HashMap, VecDeque};

use runtime::piece_ring::PieceEntry;

use crate::pump::AxisKey;

pub const HISTORY_CAPACITY: usize = 4096;

#[derive(Debug, thiserror::Error)]
pub enum HistoryError {
    #[error(
        "query clock {queried} precedes retained motion history for axis \
         {key:?} (window {window_start}..{window_end})"
    )]
    BeforeRetainedWindow {
        key: AxisKey,
        queried: u64,
        window_start: u64,
        window_end: u64,
    },

    #[error(
        "query clock {queried} is in the future for axis {key:?} \
         (now≈{now_clock}) — motion history answers the past only"
    )]
    QueryInFuture {
        key: AxisKey,
        queried: u64,
        now_clock: u64,
    },

    #[error("no motion history recorded for axis {0:?}")]
    NoHistoryForAxis(AxisKey),
}

#[derive(Debug, Clone, Copy)]
pub struct HistoryPiece {
    pub start_clock: u64,
    pub end_clock: u64,
    pub duration_secs: f32,
    pub coeffs: [f32; 4],
}

impl HistoryPiece {
    pub fn from_entry(entry: &PieceEntry, nominal_freq_hz: u32) -> Self {
        #[allow(clippy::cast_precision_loss)]
        let end_clock = entry.end_time(nominal_freq_hz as f32);
        Self {
            start_clock: entry.start_time,
            end_clock,
            duration_secs: entry.duration,
            coeffs: entry.coeffs,
        }
    }

    fn endpoint(&self) -> AxisEndpoint {
        AxisEndpoint {
            clock: self.end_clock,
            position: f64::from(self.coeffs[3]),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AxisState {
    pub position: f64,
    pub velocity: f64,
    pub acceleration: f64,
}

#[derive(Debug, Clone, Copy)]
struct AxisEndpoint {
    clock: u64,
    position: f64,
}

impl AxisEndpoint {
    fn hold_state(&self) -> AxisState {
        AxisState {
            position: self.position,
            velocity: 0.0,
            acceleration: 0.0,
        }
    }
}

#[inline]
pub fn eval_bernstein_cubic(coeffs: [f32; 4], u: f64) -> f64 {
    let v = 1.0 - u;
    let b0 = f64::from(coeffs[0]);
    let b1 = f64::from(coeffs[1]);
    let b2 = f64::from(coeffs[2]);
    let b3 = f64::from(coeffs[3]);
    v * v * v * b0 + 3.0 * v * v * u * b1 + 3.0 * v * u * u * b2 + u * u * u * b3
}

fn eval_state(piece: &HistoryPiece, clock: u64) -> AxisState {
    #[allow(clippy::cast_precision_loss)]
    let dur_ticks = piece.end_clock.saturating_sub(piece.start_clock) as f64;
    #[allow(clippy::cast_precision_loss)]
    let u = if dur_ticks > 0.0 {
        ((clock - piece.start_clock) as f64 / dur_ticks).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let v = 1.0 - u;
    let b0 = f64::from(piece.coeffs[0]);
    let b1 = f64::from(piece.coeffs[1]);
    let b2 = f64::from(piece.coeffs[2]);
    let b3 = f64::from(piece.coeffs[3]);
    let t = f64::from(piece.duration_secs);
    let (velocity, acceleration) = if t > 0.0 {
        let db = 3.0 * ((b1 - b0) * v * v + 2.0 * (b2 - b1) * v * u + (b3 - b2) * u * u);
        let d2b = 6.0 * ((b2 - 2.0 * b1 + b0) * v + (b3 - 2.0 * b2 + b1) * u);
        (db / t, d2b / (t * t))
    } else {
        (0.0, 0.0)
    };
    AxisState {
        position: eval_bernstein_cubic(piece.coeffs, u),
        velocity,
        acceleration,
    }
}

#[derive(Debug, Default)]
pub struct HistoryStore {
    rings: HashMap<AxisKey, VecDeque<HistoryPiece>>,
    endpoints: HashMap<AxisKey, AxisEndpoint>,
}

impl HistoryStore {
    pub fn record(&mut self, key: AxisKey, entry: &PieceEntry, nominal_freq_hz: u32) {
        let piece = HistoryPiece::from_entry(entry, nominal_freq_hz);
        let ring = self.rings.entry(key).or_default();
        if let Some(last) = ring.back() {
            assert!(
                piece.start_clock >= last.start_clock,
                "out-of-order piece for {key:?}: {} < {}",
                piece.start_clock,
                last.start_clock
            );
        }
        if ring.len() == HISTORY_CAPACITY {
            ring.pop_front();
        }
        self.endpoints.insert(key, piece.endpoint());
        ring.push_back(piece);
    }

    pub fn rebase_axis(&mut self, key: AxisKey, clock: u64, position: f64) {
        self.rings.entry(key).or_default().clear();
        self.endpoints.insert(key, AxisEndpoint { clock, position });
    }

    pub fn last_endpoint_clock(&self, key: AxisKey) -> u64 {
        self.endpoints.get(&key).map_or(0, |e| e.clock)
    }

    pub fn state_at_clock(
        &self,
        key: AxisKey,
        clock: u64,
        now_clock: Option<u64>,
    ) -> Result<AxisState, HistoryError> {
        let ring = self.rings.get(&key).filter(|r| !r.is_empty());
        let hold = match ring {
            Some(ring) => {
                let idx = ring.partition_point(|p| p.start_clock <= clock);
                if idx == 0 {
                    return Err(HistoryError::BeforeRetainedWindow {
                        key,
                        queried: clock,
                        window_start: ring.front().map_or(0, |p| p.start_clock),
                        window_end: ring.back().map_or(0, |p| p.end_clock),
                    });
                }
                let piece = &ring[idx - 1];
                if clock < piece.end_clock {
                    return Ok(eval_state(piece, clock));
                }
                piece.endpoint()
            }
            None => {
                let endpoint = self
                    .endpoints
                    .get(&key)
                    .ok_or(HistoryError::NoHistoryForAxis(key))?;
                if clock < endpoint.clock {
                    return Err(HistoryError::BeforeRetainedWindow {
                        key,
                        queried: clock,
                        window_start: endpoint.clock,
                        window_end: endpoint.clock,
                    });
                }
                *endpoint
            }
        };
        if let Some(now_clock) = now_clock {
            if clock > now_clock {
                return Err(HistoryError::QueryInFuture {
                    key,
                    queried: clock,
                    now_clock,
                });
            }
        }
        Ok(hold.hold_state())
    }
}

#[cfg(test)]
mod tests;
