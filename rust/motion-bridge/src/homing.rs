use std::sync::Mutex;
use std::sync::atomic::{AtomicU8, AtomicU64, Ordering};

use crate::pump::AxisKey;

pub const DRIP_PIECE_SECS: f64 = 0.025;
pub const DRIP_MAX_AHEAD_SECS: f64 = 0.05;

pub fn homing_enqueue_params(homing_active: bool) -> (f64, Option<f64>) {
    if homing_active {
        (DRIP_MAX_AHEAD_SECS, Some(DRIP_PIECE_SECS))
    } else {
        (crate::pump::MAX_LEAD_SECS, None)
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HomingSegmentState {
    Idle = 0,
    Active = 1,
    Completed = 2,
    Tripped = 3,
}

impl HomingSegmentState {
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::Active,
            2 => Self::Completed,
            3 => Self::Tripped,
            _ => Self::Idle,
        }
    }
}

#[derive(Debug)]
pub struct HomingState {
    state: AtomicU8,
    arm_id: AtomicU64,
    pending_trip: Mutex<Option<runtime::endstop::TripEvent>>,
    axis_keys: Mutex<Vec<AxisKey>>,
}

impl HomingState {
    pub fn new() -> Self {
        Self {
            state: AtomicU8::new(HomingSegmentState::Idle as u8),
            arm_id: AtomicU64::new(0),
            pending_trip: Mutex::new(None),
            axis_keys: Mutex::new(Vec::new()),
        }
    }

    pub fn state(&self) -> HomingSegmentState {
        HomingSegmentState::from_u8(self.state.load(Ordering::Acquire))
    }

    pub fn state_u8(&self) -> u8 {
        self.state.load(Ordering::Acquire)
    }

    pub fn begin(&self, arm_id: u32) {
        self.arm_id.store(u64::from(arm_id), Ordering::Release);
        *self.pending_trip.lock().unwrap() = None;
        self.axis_keys.lock().unwrap().clear();
        self.state
            .store(HomingSegmentState::Active as u8, Ordering::Release);
    }

    pub fn record_axis_keys(&self, keys: &[AxisKey]) {
        let mut guard = self.axis_keys.lock().unwrap();
        for &k in keys {
            if !guard.contains(&k) {
                guard.push(k);
            }
        }
    }

    pub fn take_axis_keys(&self) -> Vec<AxisKey> {
        std::mem::take(&mut *self.axis_keys.lock().unwrap())
    }

    pub fn reset_to_idle(&self) {
        self.state
            .store(HomingSegmentState::Idle as u8, Ordering::Release);
    }

    pub fn refresh_after_wait(&self) {
        if self.state() != HomingSegmentState::Active {
            return;
        }
        if let Some(evt) = runtime::endstop::poll_trip() {
            *self.pending_trip.lock().unwrap() = Some(evt);
            self.state
                .store(HomingSegmentState::Tripped as u8, Ordering::Release);
        } else {
            self.state
                .store(HomingSegmentState::Completed as u8, Ordering::Release);
        }
    }

    pub fn take_trip_event(&self) -> Option<runtime::endstop::TripEvent> {
        if let Some(evt) = runtime::endstop::poll_trip() {
            *self.pending_trip.lock().unwrap() = Some(evt);
            self.state
                .store(HomingSegmentState::Tripped as u8, Ordering::Release);
        }
        self.pending_trip.lock().unwrap().take()
    }

    pub fn take_completion_event(&self) -> Option<u32> {
        if self.state() != HomingSegmentState::Completed {
            return None;
        }
        let arm = self.arm_id.swap(0, Ordering::AcqRel);
        if arm == 0 {
            return None;
        }
        self.state
            .store(HomingSegmentState::Idle as u8, Ordering::Release);
        Some(arm as u32)
    }
}

impl Default for HomingState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests;
