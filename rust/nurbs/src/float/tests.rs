#[allow(clippy::float_cmp)] // tests assert exact bit-for-bit values for constants and round-trips
use super::*;

#[test]
fn from_f64_roundtrips_f32() {
    let x: f32 = Float::from_f64(1.5_f64);
    assert_eq!(x, 1.5_f32);
}

#[cfg(feature = "f64")]
#[test]
fn from_f64_identity_on_f64() {
    let x: f64 = Float::from_f64(1.5_f64);
    assert_eq!(x, 1.5_f64);
}

#[test]
fn mul_add_matches_naive_for_f32() {
    let result = (2.0_f32).mul_add(3.0, 4.0);
    assert!((result - 10.0).abs() < f32::EPSILON);
}

#[test]
fn zero_one_constants_are_correct() {
    assert_eq!(<f32 as Float>::ZERO, 0.0_f32);
    assert_eq!(<f32 as Float>::ONE, 1.0_f32);
}

#[test]
fn f32_min_max_handles_equal_values() {
    assert_eq!(<f32 as Float>::min(1.0, 1.0), 1.0);
    assert_eq!(<f32 as Float>::max(1.0, 1.0), 1.0);
}
