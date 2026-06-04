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

    fn from_f64(x: f64) -> Self;
    // mul_add is load-bearing on M7: emits a single VFMA.F32 instruction in release.
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
        // no_std: f32::mul_add recurses via trait; use libm::fmaf instead.
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
#[allow(clippy::float_cmp)]
mod tests;
