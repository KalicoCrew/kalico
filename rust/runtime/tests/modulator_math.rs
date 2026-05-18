//! `PhaseDirectModulator` math: mscount from position, phase-advance
//! accumulator for direction sense, stepper-counts delta.

use runtime::modulator::PhaseDirectModulator;
use runtime::phase_lut::MOTOR_PERIOD;

const STEPS_PER_MM: f32 = 80.0; // typical 256x, 1.8deg, 20T pulley CoreXY

#[test]
fn mscount_from_position_zero() {
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    let r = m.compute(0.0);
    assert_eq!(r.mscount, 0);
}

#[test]
fn mscount_wraps_modulo_electrical_cycle() {
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    // One full electrical cycle = MOTOR_PERIOD microsteps. At
    // steps_per_mm = 80, that's 1024 / 80 = 12.8 mm. Position 12.8 mm
    // should land at mscount = 0 again.
    let r = m.compute(MOTOR_PERIOD as f32 / STEPS_PER_MM);
    assert!(r.mscount == 0 || r.mscount == 1 || r.mscount == (MOTOR_PERIOD as u16 - 1),
            "expected wrap to ~0, got {}", r.mscount);
}

#[test]
fn mscount_quarter_cycle() {
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    // Quarter electrical cycle = MOTOR_PERIOD/4 = 256 microsteps
    // = 256 / 80 = 3.2 mm.
    let r = m.compute(3.2);
    assert!((r.mscount as i32 - 256).abs() <= 1,
            "expected ~256, got {}", r.mscount);
}

#[test]
fn stepper_counts_delta_is_microstep_rounded() {
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    let r1 = m.compute(0.0);
    assert_eq!(r1.steps_delta, 0); // first call seeds, no delta
    let r2 = m.compute(0.0125); // exactly 1 microstep at 80 steps/mm
    assert_eq!(r2.steps_delta, 1);
    let r3 = m.compute(0.025); // another microstep
    assert_eq!(r3.steps_delta, 1);
}

#[test]
fn direction_sticks_through_sub_microstep_ticks() {
    // At very low velocity, many consecutive ticks may have |delta| < 1
    // microstep. The phase-advance accumulator must NOT report direction = 0
    // every tick — it must hold the last established direction.
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    m.compute(0.0);
    let r1 = m.compute(0.020); // 1.6 microsteps forward -> direction = +1
    assert_eq!(r1.direction, 1);
    // Now creep forward by 0.005 mm (0.4 microsteps) per tick
    let r2 = m.compute(0.025);
    let r3 = m.compute(0.030);
    assert_eq!(r2.direction, 1, "direction must stick through sub-microstep tick");
    assert_eq!(r3.direction, 1);
}

#[test]
fn direction_flips_on_reversal() {
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    m.compute(0.0);
    let _ = m.compute(0.020); // forward, dir = +1
    let r = m.compute(0.005); // moved -0.015 mm (-1.2 microsteps) -> reversed
    assert_eq!(r.direction, -1);
}

#[test]
fn steps_delta_bounds_via_max_per_tick_default() {
    // The modulator's per-tick step burst is bounded the same way the
    // existing `StepMotorState::update` bounds it. A sane default avoids
    // pathological deltas latching `StepBurstExceeded`.
    let mut m = PhaseDirectModulator::new(STEPS_PER_MM);
    m.compute(0.0);
    // 0.5 mm at 80 steps/mm = 40 microsteps in one tick. Should succeed
    // (well below MAX_STEPS_PER_TICK default).
    let r = m.compute(0.5);
    assert_eq!(r.steps_delta, 40);
}
