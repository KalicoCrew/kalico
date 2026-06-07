#![allow(unsafe_code)]

// Phase constants — must match src/generic/fault_handler.h exactly.
#[allow(dead_code)]
pub(crate) const RT_PHASE_IDLE: u32 = 0;
pub(crate) const RT_PHASE_ISR_ENTER: u32 = 1;
#[allow(dead_code)]
pub(crate) const RT_PHASE_WIDEN: u32 = 2;
#[allow(dead_code)]
pub(crate) const RT_PHASE_GUARD: u32 = 3;
pub(crate) const RT_PHASE_TICK: u32 = 4;
pub(crate) const RT_PHASE_WALK: u32 = 5;
pub(crate) const RT_PHASE_MONOMIAL: u32 = 6;
pub(crate) const RT_PHASE_HORNER: u32 = 7;
#[allow(dead_code)]
pub(crate) const RT_PHASE_STEP_ENQ: u32 = 8;
pub(crate) const RT_PHASE_ISR_EXIT: u32 = 9;
pub(crate) const RT_PHASE_STEPOUT_ENTER: u32 = 10;
pub(crate) const RT_PHASE_STEPOUT_POP: u32 = 11;
pub(crate) const RT_PHASE_STEPOUT_EMIT: u32 = 12;
pub(crate) const RT_PHASE_STEPOUT_EXIT: u32 = 13;

#[cfg(not(any(test, feature = "host")))]
unsafe extern "C" {
    fn runtime_set_isr_phase(phase: u32);
    fn diag_walk_account(cycles: u32);
    fn diag_monomial_account(cycles: u32);
    fn runtime_cyccnt_read() -> u32;
}

#[inline]
pub(crate) fn set_phase(phase: u32) {
    #[cfg(not(any(test, feature = "host")))]
    // SAFETY: `runtime_set_isr_phase` performs a single volatile store to a
    // persistent diagnostic struct. No side effects beyond the store; safe to
    // call from any ISR context.
    unsafe {
        runtime_set_isr_phase(phase);
    }
    #[cfg(any(test, feature = "host"))]
    {
        let _ = phase;
    }
}

#[inline]
pub(crate) fn walk_account(cycles: u32) {
    #[cfg(not(any(test, feature = "host")))]
    // SAFETY: `diag_walk_account` updates a max+count pair in the persistent
    // diagnostic struct. Single-caller discipline (TIM5 ISR); no data races.
    unsafe {
        diag_walk_account(cycles);
    }
    #[cfg(any(test, feature = "host"))]
    {
        let _ = cycles;
    }
}

#[inline]
pub(crate) fn monomial_account(cycles: u32) {
    #[cfg(not(any(test, feature = "host")))]
    // SAFETY: `diag_monomial_account` updates a max+count pair in the
    // persistent diagnostic struct. Single-caller discipline (TIM5 ISR).
    unsafe {
        diag_monomial_account(cycles);
    }
    #[cfg(any(test, feature = "host"))]
    {
        let _ = cycles;
    }
}

#[inline]
pub(crate) fn cyccnt() -> u32 {
    #[cfg(not(any(test, feature = "host")))]
    // SAFETY: `runtime_cyccnt_read` is a single DWT CYCCNT MMIO read.
    // Side-effect-free and safe from any ISR context.
    unsafe {
        runtime_cyccnt_read()
    }
    #[cfg(any(test, feature = "host"))]
    {
        0
    }
}
