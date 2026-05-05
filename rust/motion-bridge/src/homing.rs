use std::sync::Mutex;
use std::sync::atomic::{AtomicU8, AtomicU64, Ordering};

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
    active_segment_id: AtomicU64,
    arm_id: AtomicU64,
    pending_trip: Mutex<Option<runtime::endstop::TripEvent>>,
}

impl HomingState {
    pub fn new() -> Self {
        Self {
            state: AtomicU8::new(HomingSegmentState::Idle as u8),
            active_segment_id: AtomicU64::new(0),
            arm_id: AtomicU64::new(0),
            pending_trip: Mutex::new(None),
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
        self.active_segment_id.store(0, Ordering::Release);
        *self.pending_trip.lock().unwrap() = None;
        self.state
            .store(HomingSegmentState::Active as u8, Ordering::Release);
    }

    pub fn reset_to_idle(&self) {
        self.state
            .store(HomingSegmentState::Idle as u8, Ordering::Release);
    }

    pub fn mark_dispatched_segment(&self, segment_id: u32) {
        if self.state() == HomingSegmentState::Active {
            self.active_segment_id
                .store(u64::from(segment_id), Ordering::Release);
        }
    }

    pub fn complete_if_retired(&self, retired_through_segment_id: u32) {
        let active = self.active_segment_id.load(Ordering::Acquire);
        if active != 0
            && u64::from(retired_through_segment_id) >= active
            && self.state() == HomingSegmentState::Active
        {
            self.state
                .store(HomingSegmentState::Completed as u8, Ordering::Release);
        }
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

    /// Take-once accessor for the no-trip terminal. Returns Some(arm_id)
    /// exactly once after the homing segment retires without a trip, then
    /// None on every subsequent call until the next `begin()`.
    ///
    /// Mirrors `take_trip_event`'s ownership semantics: the caller is
    /// responsible for delivering the event exactly once. If the state is
    /// `Tripped`, this returns None — the trip event owns that terminal.
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
mod tests {
    use super::*;

    #[test]
    fn take_completion_event_fires_once_after_no_trip_retire() {
        let h = HomingState::new();
        h.begin(7);
        h.mark_dispatched_segment(3);
        // Retire past the homing segment; no trip occurred.
        h.complete_if_retired(3);
        assert_eq!(h.state(), HomingSegmentState::Completed);
        assert_eq!(h.take_completion_event(), Some(7));
        // Take-once: a second call returns None.
        assert_eq!(h.take_completion_event(), None);
    }

    #[test]
    fn take_completion_event_does_not_fire_when_idle() {
        let h = HomingState::new();
        assert_eq!(h.take_completion_event(), None);
    }

    #[test]
    fn take_completion_event_does_not_fire_after_trip() {
        // If the segment retired *and* a trip was latched, the trip path
        // owns the terminal — we don't double-fire as past-end-time.
        let h = HomingState::new();
        h.begin(8);
        h.mark_dispatched_segment(2);
        // Simulate a trip flipping the state to Tripped.
        h.state.store(
            HomingSegmentState::Tripped as u8,
            std::sync::atomic::Ordering::Release,
        );
        // Even if credit-freed retires the segment afterwards, no completion event.
        h.complete_if_retired(2);
        assert_eq!(h.state(), HomingSegmentState::Tripped);
        assert_eq!(h.take_completion_event(), None);
    }
}
