//! Float abstraction for f32 / f64 single-source.
//! See spec §Substrate / Float abstraction.

/// Single-source numeric trait for the eval crate. Tight surface — only the
/// operations the math actually uses. Both `f32` and `f64` impls live in this
/// module; `f64` is feature-gated so the MCU build closure stays tight.
pub trait Float:
    Copy
    + Default
    + PartialEq
    + PartialOrd
    + core::ops::Add<Output = Self>
    + core::ops::Sub<Output = Self>
    + core::ops::Mul<Output = Self>
    + core::ops::Div<Output = Self>
    + core::ops::Neg<Output = Self>
    + core::fmt::Debug
{
    const ZERO: Self;
    const ONE: Self;

    /// Lift a compile-time `f64` literal into `Self`. Truncates for `f32`.
    fn from_f64(x: f64) -> Self;

    /// Fused multiply-add: `self * a + b`. Load-bearing on M7 — codegen
    /// emits a single `VFMA.F32` instruction. Do not rely on opportunistic
    /// FMA fusion via fast-math flags; this trait method is the contract.
    fn mul_add(self, a: Self, b: Self) -> Self;

    fn sqrt(self) -> Self;
    fn abs(self) -> Self;
    fn min(self, other: Self) -> Self;
    fn max(self, other: Self) -> Self;
    fn total_cmp(self, other: Self) -> core::cmp::Ordering;
}

impl Float for f32 {
    const ZERO: Self = 0.0;
    const ONE: Self = 1.0;

    #[inline]
    fn from_f64(x: f64) -> Self {
        x as f32
    }

    #[inline]
    fn mul_add(self, a: Self, b: Self) -> Self {
        // `f32::mul_add` is an inherent method only when `std` is linked.
        // In `no_std` MCU builds it falls back to the trait method via name
        // resolution and recurses; route to `libm::fmaf` instead.
        #[cfg(feature = "host")]
        {
            f32::mul_add(self, a, b)
        }
        #[cfg(not(feature = "host"))]
        {
            libm::fmaf(self, a, b)
        }
    }

    #[inline]
    fn sqrt(self) -> Self {
        // libm-style: hardware on M7/M4; std::f32::sqrt on host.
        #[cfg(feature = "host")]
        {
            f32::sqrt(self)
        }
        #[cfg(not(feature = "host"))]
        {
            libm::sqrtf(self)
        }
    }

    #[inline]
    fn abs(self) -> Self {
        #[cfg(feature = "host")]
        {
            f32::abs(self)
        }
        #[cfg(not(feature = "host"))]
        {
            libm::fabsf(self)
        }
    }

    #[inline]
    fn min(self, other: Self) -> Self {
        if self < other { self } else { other }
    }

    #[inline]
    fn max(self, other: Self) -> Self {
        if self > other { self } else { other }
    }

    #[inline]
    fn total_cmp(self, other: Self) -> core::cmp::Ordering {
        f32::total_cmp(&self, &other)
    }
}

#[cfg(feature = "f64")]
impl Float for f64 {
    const ZERO: Self = 0.0;
    const ONE: Self = 1.0;

    #[inline]
    fn from_f64(x: f64) -> Self {
        x
    }

    #[inline]
    fn mul_add(self, a: Self, b: Self) -> Self {
        f64::mul_add(self, a, b)
    }

    #[inline]
    fn sqrt(self) -> Self {
        f64::sqrt(self)
    }

    #[inline]
    fn abs(self) -> Self {
        f64::abs(self)
    }

    #[inline]
    fn min(self, other: Self) -> Self {
        if self < other { self } else { other }
    }

    #[inline]
    fn max(self, other: Self) -> Self {
        if self > other { self } else { other }
    }

    #[inline]
    fn total_cmp(self, other: Self) -> core::cmp::Ordering {
        f64::total_cmp(&self, &other)
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)] // tests assert exact bit-for-bit values for constants and round-trips
mod tests {
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
}
