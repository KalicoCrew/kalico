//! `PhaseDirectModulator` math: mscount from position, phase-advance
//! accumulator for direction sense, stepper-counts delta.

use runtime::modulator::PhaseDirectModulator;
use runtime::phase_lut::MOTOR_PERIOD;

const STEPS_PER_MM: f32 = 80.0; // typical 256x, 1.8deg, 20T pulley CoreXY

#[test]
fn mscount_from_position_zero() {
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    let r = m.compute(0.0).unwrap();
    assert_eq!(r.mscount, 0);
}

#[test]
fn mscount_wraps_modulo_electrical_cycle() {
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    // One full electrical cycle = MOTOR_PERIOD microsteps. At
    // steps_per_mm = 80, that's 1024 / 80 = 12.8 mm. Position 12.8 mm
    // should land at mscount = 0 again.
    let r = m.compute(MOTOR_PERIOD as f32 / STEPS_PER_MM).unwrap();
    assert!(r.mscount == 0 || r.mscount == 1 || r.mscount == (MOTOR_PERIOD as u16 - 1),
            "expected wrap to ~0, got {}", r.mscount);
}

#[test]
fn mscount_quarter_cycle() {
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    // Quarter electrical cycle = MOTOR_PERIOD/4 = 256 microsteps
    // = 256 / 80 = 3.2 mm.
    let r = m.compute(3.2).unwrap();
    assert!((r.mscount as i32 - 256).abs() <= 1,
            "expected ~256, got {}", r.mscount);
}

#[test]
fn stepper_counts_delta_is_microstep_rounded() {
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    let r1 = m.compute(0.0).unwrap();
    assert_eq!(r1.steps_delta, 0); // first call seeds, no delta
    let r2 = m.compute(0.0125).unwrap(); // exactly 1 microstep at 80 steps/mm
    assert_eq!(r2.steps_delta, 1);
    let r3 = m.compute(0.025).unwrap(); // another microstep
    assert_eq!(r3.steps_delta, 1);
}

#[test]
fn direction_sticks_through_sub_microstep_ticks() {
    // At very low velocity, many consecutive ticks may have |delta| < 1
    // microstep. The phase-advance accumulator must NOT report direction = 0
    // every tick — it must hold the last established direction.
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    let _ = m.compute(0.0).unwrap();
    let r1 = m.compute(0.020).unwrap(); // 1.6 microsteps forward -> direction = +1
    assert_eq!(r1.direction, 1);
    // Now creep forward by 0.005 mm (0.4 microsteps) per tick
    let r2 = m.compute(0.025).unwrap();
    let r3 = m.compute(0.030).unwrap();
    assert_eq!(r2.direction, 1, "direction must stick through sub-microstep tick");
    assert_eq!(r3.direction, 1);
}

#[test]
fn direction_flips_on_reversal() {
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    let _ = m.compute(0.0).unwrap();
    let _ = m.compute(0.020).unwrap(); // forward, dir = +1
    let r = m.compute(0.005).unwrap(); // moved -0.015 mm (-1.2 microsteps) -> reversed
    assert_eq!(r.direction, -1);
}

#[test]
fn steps_delta_bounds_via_max_per_tick_default() {
    // The modulator's per-tick step burst is bounded the same way the
    // existing `StepMotorState::update` bounds it. A sane default avoids
    // pathological deltas latching `StepBurstExceeded`.
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    let _ = m.compute(0.0).unwrap();
    // 0.5 mm at 80 steps/mm = 40 microsteps in one tick. Should succeed
    // (well below MAX_STEPS_PER_TICK default).
    let r = m.compute(0.5).unwrap();
    assert_eq!(r.steps_delta, 40);
}

#[test]
fn compute_returns_err_when_burst_exceeds_cap() {
    // Verify the burst cap actually short-circuits compute() and leaves
    // the accumulator untouched (so the caller can fault-handle + retry).
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    m.seed(0.0);
    m.max_steps_per_tick = 100;
    // 10 mm at 80 steps/mm = 800 microsteps — well past the 100-step cap.
    let target = 10.0_f32;
    assert!(
        m.compute(target).is_err(),
        "expected Err(()) when steps_delta exceeds cap",
    );
    // After Err, the accumulator must NOT have been advanced. Bumping the
    // cap back to default and re-trying the same position should report
    // the FULL 800-step delta — proving the failed call left no residue.
    m.max_steps_per_tick = i32::MAX;
    let r = m.compute(target).unwrap();
    assert_eq!(
        r.steps_delta, 800,
        "after a failed compute(), accumulator must be untouched; retry sees full delta",
    );
}

#[test]
fn mscount_wraps_correctly_on_reverse_jog_through_origin() {
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    m.seed(0.0);
    // Move backward from 0 to a negative position. 0.0125 mm = 1 microstep.
    // -0.0125 mm should give accumulator = -1, mscount = MOTOR_PERIOD - 1.
    let r = m.compute(-0.0125).unwrap();
    assert_eq!(r.mscount, (MOTOR_PERIOD as u16) - 1,
               "negative accumulator must wrap to MOTOR_PERIOD-1");
    // Move further back. Accumulator = -2, mscount = MOTOR_PERIOD - 2.
    let r = m.compute(-0.025).unwrap();
    assert_eq!(r.mscount, (MOTOR_PERIOD as u16) - 2);
}

#[test]
fn reset_accumulator_re_seeds_on_next_compute() {
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    m.seed(0.0);
    // Move forward several ticks.
    for x_steps in 1..=5 {
        let _ = m.compute(0.0125 * (x_steps as f32)).unwrap();
    }
    // Reset and jump to a far position. If reset cleared the accumulator,
    // the next compute() should re-seed (steps_delta == 0) rather than
    // reporting a multi-thousand-microstep burst.
    m.reset_accumulator();
    let r = m.compute(10.0).unwrap();
    assert_eq!(r.steps_delta, 0,
               "after reset_accumulator, first compute() must re-seed not burst");
    // Now subsequent ticks compute deltas from 10.0 as the new baseline.
    // Use a +0.1 mm increment (8 microsteps at 80 steps/mm); +1 microstep
    // would land just under 1.0 in f64 after the f32→f64 round-trip
    // (`10.0125_f32 * 80.0 ≈ 800.9999847` → truncates to 0). 8 microsteps
    // is comfortably past that precision boundary while still being a
    // recognizably "small follow-up tick" relative to the 800-step seed.
    let r = m.compute(10.1).unwrap();
    assert_eq!(r.steps_delta, 8);
}
