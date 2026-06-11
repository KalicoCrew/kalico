#![allow(unsafe_code)]

use core::sync::atomic::{AtomicU32, Ordering};

use crate::step_queue::{N_AXIS_STEP_QUEUES, peek as queue_peek, pop as queue_pop};

pub const STEP_OUTPUT_DISABLE: u32 = u32::MAX;

const DUE_WINDOW_CYCLES: i32 = 0;

pub const MAX_STEPS_PER_EVENT: u32 = 32;

static STEPOUT_MAX_LATE_CYCLES: AtomicU32 = AtomicU32::new(0);
static STEPOUT_LATE_COUNT: AtomicU32 = AtomicU32::new(0);
static STEPOUT_MAX_DRAINED: AtomicU32 = AtomicU32::new(0);

fn record_lateness(now: u32, cycle_abs: u32, threshold: u32) {
    let late_cycles = now.wrapping_sub(cycle_abs);
    if late_cycles > threshold {
        let count = STEPOUT_LATE_COUNT.load(Ordering::Relaxed);
        STEPOUT_LATE_COUNT.store(count.wrapping_add(1), Ordering::Relaxed);
        let prev = STEPOUT_MAX_LATE_CYCLES.load(Ordering::Relaxed);
        if late_cycles > prev {
            STEPOUT_MAX_LATE_CYCLES.store(late_cycles, Ordering::Relaxed);
        }
    }
}

fn record_drained(count: u32) {
    let prev = STEPOUT_MAX_DRAINED.load(Ordering::Relaxed);
    if count > prev {
        STEPOUT_MAX_DRAINED.store(count, Ordering::Relaxed);
    }
}

#[cfg(not(any(test, feature = "host")))]
fn late_threshold_12p5_us_in_cycles() -> u32 {
    let freq = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(runtime_clock_freq)) };
    freq / 80_000
}

#[cfg(any(test, feature = "host"))]
fn late_threshold_12p5_us_in_cycles() -> u32 {
    test_hooks::late_threshold()
}

#[cfg(not(any(test, feature = "host")))]
#[unsafe(no_mangle)]
pub extern "C" fn kalico_stepout_late_get(
    out_max_late: *mut u32,
    out_late_count: *mut u32,
    out_max_drained: *mut u32,
) {
    unsafe {
        *out_max_late = STEPOUT_MAX_LATE_CYCLES.load(Ordering::Relaxed);
        *out_late_count = STEPOUT_LATE_COUNT.load(Ordering::Relaxed);
        *out_max_drained = STEPOUT_MAX_DRAINED.load(Ordering::Relaxed);
    }
}

#[cfg(not(any(test, feature = "host")))]
#[unsafe(no_mangle)]
pub extern "C" fn kalico_stepout_late_reset() {
    STEPOUT_MAX_LATE_CYCLES.store(0, Ordering::Relaxed);
    STEPOUT_LATE_COUNT.store(0, Ordering::Relaxed);
    STEPOUT_MAX_DRAINED.store(0, Ordering::Relaxed);
}

#[cfg(not(any(test, feature = "host")))]
unsafe extern "C" {
    static runtime_clock_freq: u32;
    fn timer_read_time() -> u32;
    fn runtime_emit_step_pulses(axis_idx: u8, n_steps: i32);
    fn kalico_step_output_owned_mask() -> u8;
}

#[cfg(any(test, feature = "host"))]
pub use test_hooks::{owned_mask as host_owned_mask, set_now, set_owned_mask};

#[cfg(any(test, feature = "host"))]
unsafe fn timer_read_time() -> u32 {
    test_hooks::now()
}
#[cfg(any(test, feature = "host"))]
unsafe fn runtime_emit_step_pulses(axis_idx: u8, n_steps: i32) {
    test_hooks::record_emit(axis_idx, n_steps);
}
#[cfg(any(test, feature = "host"))]
unsafe fn kalico_step_output_owned_mask() -> u8 {
    test_hooks::owned_mask()
}

#[unsafe(no_mangle)]
pub extern "C" fn kalico_step_output_event() -> u32 {
    // SAFETY: `timer_read_time` is a single u32 MMIO read (host: a test hook).
    let now = unsafe { timer_read_time() };
    // SAFETY: side-effect-free C getter (host: a test hook).
    let owned = unsafe { kalico_step_output_owned_mask() };
    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_STEPOUT_ENTER);

    let threshold = late_threshold_12p5_us_in_cycles();
    let mut emitted: u32 = 0;
    'outer: loop {
        let mut emitted_this_pass = false;
        for axis_idx in 0..N_AXIS_STEP_QUEUES {
            if owned & (1u8 << axis_idx) == 0 {
                continue;
            }
            if emitted >= MAX_STEPS_PER_EVENT {
                break 'outer;
            }
            let q = resolve_queue_ptr(axis_idx);
            if q.is_null() {
                continue;
            }
            // SAFETY: `q` is non-null (checked) and points at a live StepQueue;
            // this body is the sole consumer (same-priority invariant).
            crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_STEPOUT_POP);
            let Some(entry) = (unsafe { queue_peek(q) }) else {
                continue;
            };
            let delta = entry.cycle_abs.wrapping_sub(now) as i32;
            if delta <= DUE_WINDOW_CYCLES {
                record_lateness(now, entry.cycle_abs, threshold);
                // SAFETY: sole-consumer discipline as above.
                let _ = unsafe { queue_pop(q) };
                // SAFETY: C step emitter guards out-of-range motor indices.
                crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_STEPOUT_EMIT);
                unsafe { runtime_emit_step_pulses(axis_idx as u8, i32::from(entry.dir)) };
                emitted += 1;
                emitted_this_pass = true;
            }
        }
        if !emitted_this_pass {
            break;
        }
    }

    record_drained(emitted);

    if emitted >= MAX_STEPS_PER_EVENT {
        crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_STEPOUT_EXIT);
        return now;
    }

    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_STEPOUT_EXIT);
    next_wake_across_owned(now, owned).unwrap_or(STEP_OUTPUT_DISABLE)
}

fn next_wake_across_owned(now: u32, owned: u8) -> Option<u32> {
    let mut best: Option<(i32, u32)> = None;
    for axis_idx in 0..N_AXIS_STEP_QUEUES {
        if owned & (1u8 << axis_idx) == 0 {
            continue;
        }
        let q = resolve_queue_ptr(axis_idx);
        if q.is_null() {
            continue;
        }
        // SAFETY: non-null + sole-consumer as above.
        if let Some(entry) = unsafe { queue_peek(q) } {
            let delta = entry.cycle_abs.wrapping_sub(now) as i32;
            match best {
                Some((best_delta, _)) if delta >= best_delta => {}
                _ => best = Some((delta, entry.cycle_abs)),
            }
        }
    }
    best.map(|(_, cycle_abs)| cycle_abs)
}

#[cfg(not(any(test, feature = "host")))]
fn resolve_queue_ptr(axis_idx: usize) -> *mut crate::step_queue::StepQueue {
    crate::step_queue::queue_for_axis(axis_idx)
}

#[cfg(any(test, feature = "host"))]
fn resolve_queue_ptr(axis_idx: usize) -> *mut crate::step_queue::StepQueue {
    test_hooks::queue_for_axis(axis_idx)
}

#[cfg(any(test, feature = "host"))]
pub mod test_hooks {
    use crate::step_queue::StepQueue;
    use core::cell::RefCell;
    use std::vec::Vec;

    const N_QUEUES: usize = crate::step_queue::N_AXIS_STEP_QUEUES;

    std::thread_local! {
        static NOW: RefCell<u32> = const { RefCell::new(0) };
        static OWNED: RefCell<u8> = const { RefCell::new(0) };
        static QUEUES: RefCell<[StepQueue; N_QUEUES]> =
            RefCell::new(core::array::from_fn(|_| StepQueue::new()));
        static EMITS: RefCell<Vec<(u8, i32)>> = const { RefCell::new(Vec::new()) };
        static LATE_THRESHOLD: RefCell<u32> = const { RefCell::new(500) };
    }

    pub fn set_now(v: u32) {
        NOW.with(|c| *c.borrow_mut() = v);
    }
    pub fn now() -> u32 {
        NOW.with(|c| *c.borrow())
    }
    pub fn set_owned_mask(m: u8) {
        OWNED.with(|c| *c.borrow_mut() = m);
    }
    pub fn owned_mask() -> u8 {
        OWNED.with(|c| *c.borrow())
    }
    pub fn set_late_threshold(v: u32) {
        LATE_THRESHOLD.with(|c| *c.borrow_mut() = v);
    }
    pub fn late_threshold() -> u32 {
        LATE_THRESHOLD.with(|c| *c.borrow())
    }
    pub fn record_emit(axis: u8, n: i32) {
        EMITS.with(|c| c.borrow_mut().push((axis, n)));
    }
    pub fn take_emits() -> Vec<(u8, i32)> {
        EMITS.with(|c| core::mem::take(&mut *c.borrow_mut()))
    }
    pub fn take_late_stats() -> (u32, u32, u32) {
        use core::sync::atomic::Ordering;
        let max_late = super::STEPOUT_MAX_LATE_CYCLES.swap(0, Ordering::Relaxed);
        let late_count = super::STEPOUT_LATE_COUNT.swap(0, Ordering::Relaxed);
        let max_drained = super::STEPOUT_MAX_DRAINED.swap(0, Ordering::Relaxed);
        (max_late, late_count, max_drained)
    }
    pub fn reset() {
        set_now(0);
        set_owned_mask(0);
        let _ = take_emits();
        let _ = take_late_stats();
        QUEUES.with(|c| {
            for q in c.borrow_mut().iter_mut() {
                q.clear();
            }
        });
    }
    /// Returns a raw pointer into the thread-local queue array. The pointer is
    /// only used synchronously within a single `kalico_step_output_event` call
    /// on the same thread, so it does not outlive the borrow.
    pub fn queue_for_axis(axis_idx: usize) -> *mut StepQueue {
        if axis_idx >= N_QUEUES {
            return core::ptr::null_mut();
        }
        QUEUES.with(|c| {
            let mut arr = c.borrow_mut();
            #[allow(clippy::indexing_slicing)]
            let ptr: *mut StepQueue = &mut arr[axis_idx];
            ptr
        })
    }
}

#[cfg(test)]
mod tests;
