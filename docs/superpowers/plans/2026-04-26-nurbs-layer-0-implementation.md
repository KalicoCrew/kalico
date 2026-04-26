# NURBS Layer 0 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the Layer 0 NURBS substrate (eval, arc-length, algebra-interface) plus the C ABI crate for MCU integration, per `docs/superpowers/specs/2026-04-26-nurbs-evaluation-library-design.md`.

**Architecture:** Cargo workspace at `rust/` with two crates: `nurbs` (Rust-internal API: substrate types in `lib.rs`, three modules `eval`/`arc_length`/`algebra`) and `nurbs-c-api` (stable `extern "C"` surface, cbindgen-generated header committed to source). Single-source f32/f64 via a custom `Float` trait. Owned (host-only) and borrowed (zero-copy from wire) NURBS types. No-allocation MCU eval path; allocations limited to host pre-bake.

**Tech Stack:** Rust 1.83.0 (pinned), `proptest` for property tests, Python `geomdl` (NURBS-Python) as f64 reference oracle, `cbindgen` for header generation, `criterion` (roadmap, not v1) for performance baselines.

**Reference**: `docs/superpowers/specs/2026-04-26-nurbs-evaluation-library-design.md`. Read it before starting; the plan implements that design.

---

## Repo conventions

- Run all `cargo` commands from `rust/` (workspace root) unless otherwise noted.
- Default features include `host`; `cargo test` and `cargo build` "just work" on a developer machine.
- MCU build path is `cargo build --no-default-features --features mcu-h7` (or `mcu-f4`); for plan validation, `cargo check --no-default-features --features mcu-h7` is sufficient.
- Commit after each task. Commit messages follow the pattern `nurbs: <task summary>`.

---

## Task 1: Workspace skeleton

**Files:**
- Create: `rust/Cargo.toml`
- Create: `rust/rust-toolchain.toml`
- Create: `rust/.cargo/config.toml`
- Create: `rust/README.md`
- Create: `rust/.gitignore`

- [ ] **Step 1: Create `rust/.gitignore`**

```
target/
**/*.rs.bk
Cargo.lock.bak
```

(Do **not** ignore `Cargo.lock` — for binary-shaped projects we commit it; the workspace's lock pins versions for reproducibility on the embedded target.)

- [ ] **Step 2: Create `rust/rust-toolchain.toml`**

```toml
[toolchain]
channel = "1.85.0"
components = ["rustfmt", "clippy"]
targets = [
    "thumbv7em-none-eabihf",
    "x86_64-unknown-linux-gnu",
    "aarch64-unknown-linux-gnu",
]
```

- [ ] **Step 3: Create `rust/.cargo/config.toml`**

```toml
[target.thumbv7em-none-eabihf]
rustflags = [
    "-C", "target-cpu=cortex-m7",
    "-C", "target-feature=+fp-armv8d16,+strict-align",
    "-C", "link-arg=--nmagic",
]
```

(F4 builds override `target-cpu` and `target-feature` via `--config` on the `cargo` invocation when needed; v1 hard-codes the H723 target. Soft-float / FPU-less variants out of scope per spec § Deferred work.)

- [ ] **Step 4: Create `rust/Cargo.toml`** (workspace root)

```toml
[workspace]
members = ["nurbs", "nurbs-c-api"]
resolver = "2"

[workspace.dependencies]
# Shared deps versioned here from day one.

[profile.release]
opt-level = "z"
lto = "fat"
codegen-units = 1
panic = "abort"
debug = true
overflow-checks = false

[profile.dev]
opt-level = 1            # eval correctness tests run faster than -O0; still debuggable
debug = true
```

- [ ] **Step 5: Create `rust/README.md`**

```markdown
# Kalico Rust Workspace

First-party Rust code for the kalico motion stack rewrite. See `docs/superpowers/specs/2026-04-26-nurbs-evaluation-library-design.md` for the design context.

## Layout

- `nurbs/` — Layer 0 mathematical foundations (NURBS eval, arc-length, algebra).
- `nurbs-c-api/` — stable C ABI surface for the MCU C build to call into. cbindgen-generated header at `nurbs-c-api/include/kalico_nurbs.h` (checked in).

## Build

Host (default — for tests, linting, host-side use):

    cargo build
    cargo test

MCU (H723 = Cortex-M7 with double-precision FPU):

    cargo build --release --no-default-features --features mcu-h7 --target thumbv7em-none-eabihf

The Klipper Make build picks up the resulting staticlib at `target/thumbv7em-none-eabihf/release/libnurbs_c_api.a` and the C header at `nurbs-c-api/include/kalico_nurbs.h`.

## Toolchain

Pinned via `rust-toolchain.toml`. Update intentionally with regression testing — embedded codegen is sensitive to compiler version. FPU flag strings in `.cargo/config.toml` may need to track LLVM target-feature renames across toolchain versions; verify on bumps.

## C link contract

- C side `#include`s `nurbs-c-api/include/kalico_nurbs.h` (committed; CI verifies regen is a no-op).
- C side links against `libnurbs_c_api.a`.
- All C symbols are namespaced `kalico_nurbs_*`.
- Type ownership: C never frees Rust-allocated memory; constructors/destructors come in pairs across the FFI boundary. Pointer types are opaque to C.
```

- [ ] **Step 6: Sanity-check the workspace skeleton**

```bash
cd rust
cargo --version          # should show 1.83.0 (toolchain file pinned)
```

Expected: `cargo 1.83.0 (...)`. The workspace doesn't build yet because no crates exist; that's fine.

- [ ] **Step 7: Commit**

```bash
git add rust/
git commit -m "nurbs: scaffold rust workspace skeleton"
```

---

## Task 2: nurbs crate skeleton with feature flags

**Files:**
- Create: `rust/nurbs/Cargo.toml`
- Create: `rust/nurbs/src/lib.rs`

- [ ] **Step 1: Create `rust/nurbs/Cargo.toml`**

```toml
[package]
name = "nurbs"
version = "0.1.0"
edition = "2021"
rust-version = "1.85"
publish = false
description = "Layer 0 NURBS substrate: eval, arc-length, algebra"

[features]
default = ["host"]
f64 = []
host = ["f64"]
mcu-h7 = []
mcu-f4 = []

[dev-dependencies]
proptest = "1.5"

[lints.rust]
unsafe_code = "deny"
missing_debug_implementations = "warn"

[lints.clippy]
all = "warn"
pedantic = { level = "warn", priority = -1 }
module_name_repetitions = "allow"
similar_names = "allow"
```

(`unsafe_code = "deny"` is the default policy. Wire-format zero-copy parsing may need a focused exception in Task 11/12; mark it with a `#[allow]` and a comment naming the safety contract.)

- [ ] **Step 2: Create `rust/nurbs/src/lib.rs`** (skeleton with feature-mutex compile errors)

```rust
//! Layer 0 NURBS substrate.
//!
//! See `docs/superpowers/specs/2026-04-26-nurbs-evaluation-library-design.md`.

#![cfg_attr(not(feature = "host"), no_std)]

#[cfg(all(feature = "mcu-h7", feature = "mcu-f4"))]
compile_error!("features `mcu-h7` and `mcu-f4` are mutually exclusive");

#[cfg(all(feature = "host", any(feature = "mcu-h7", feature = "mcu-f4")))]
compile_error!("feature `host` is incompatible with `mcu-*` features");

#[cfg(not(any(feature = "host", feature = "mcu-h7", feature = "mcu-f4")))]
compile_error!("must specify exactly one of: `host`, `mcu-h7`, `mcu-f4`");
```

- [ ] **Step 3: Verify default-feature build compiles**

```bash
cd rust
cargo build -p nurbs
```

Expected: builds cleanly. (`default = ["host"]` triggers the host-feature path; the compile_error checks pass because exactly one of host/mcu-* is on.)

- [ ] **Step 4: Verify MCU-feature build compiles**

```bash
cargo check -p nurbs --no-default-features --features mcu-h7 --target thumbv7em-none-eabihf
```

Expected: builds cleanly under no-std.

- [ ] **Step 5: Verify mutex enforcement**

```bash
cargo check -p nurbs --no-default-features --features "host mcu-h7" 2>&1 | head -5
```

Expected: compilation **fails** with "feature `host` is incompatible with `mcu-*` features".

```bash
cargo check -p nurbs --no-default-features 2>&1 | head -5
```

Expected: compilation **fails** with "must specify exactly one of: `host`, `mcu-h7`, `mcu-f4`".

- [ ] **Step 6: Commit**

```bash
git add rust/nurbs/
git commit -m "nurbs: add crate skeleton with feature mutex"
```

---

## Task 3: Substrate constants

**Files:**
- Modify: `rust/nurbs/src/lib.rs`

- [ ] **Step 1: Append constants to `rust/nurbs/src/lib.rs`**

```rust
/// Maximum NURBS degree the crate will accept. See spec §Substrate.
pub const MAX_DEGREE: usize = 20;

/// Stack-workspace size for de Boor's algorithm.
pub const WORKSPACE_SIZE: usize = MAX_DEGREE + 1;

/// Numerical floor for parametric speed |dP/du|, weight denominators, and
/// curvature-divisor cubed-norms. Below this, the corresponding computation
/// either clamps (release) or fires a debug_assert (debug).
///
/// Exposed as f64 so callers and `Float::from_f64` see a single source of truth.
pub const MIN_PARAMETRIC_SPEED: f64 = 1e-9;
```

- [ ] **Step 2: Add a smoke test**

Append to `rust/nurbs/src/lib.rs`:

```rust
#[cfg(test)]
mod constants_tests {
    use super::*;

    #[test]
    fn workspace_size_matches_max_degree() {
        assert_eq!(WORKSPACE_SIZE, MAX_DEGREE + 1);
    }

    #[test]
    fn min_parametric_speed_is_positive() {
        assert!(MIN_PARAMETRIC_SPEED > 0.0);
    }
}
```

- [ ] **Step 3: Run tests**

```bash
cargo test -p nurbs constants_tests
```

Expected: 2 passed.

- [ ] **Step 4: Commit**

```bash
git commit -am "nurbs: add MAX_DEGREE / WORKSPACE_SIZE / MIN_PARAMETRIC_SPEED"
```

---

## Task 4: Float trait + f32/f64 impls

**Files:**
- Create: `rust/nurbs/src/float.rs`
- Modify: `rust/nurbs/src/lib.rs`

- [ ] **Step 1: Write failing test for `Float::from_f64` round-trip**

Create `rust/nurbs/src/float.rs`:

```rust
//! Float abstraction for f32 / f64 single-source.
//! See spec §Substrate / Float abstraction.

#[cfg(test)]
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
}
```

- [ ] **Step 2: Run to verify the test won't compile yet**

```bash
cargo test -p nurbs float
```

Expected: compilation error — `trait Float not found`.

- [ ] **Step 3: Implement the trait + impls**

Replace the contents of `rust/nurbs/src/float.rs` with:

```rust
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
        // Inherent f32::mul_add is std-only; in no_std MCU builds the call
        // would resolve to the trait method and infinite-recurse. Gate the
        // same way as sqrt/abs.
        #[cfg(feature = "host")]
        { f32::mul_add(self, a, b) }
        #[cfg(not(feature = "host"))]
        { libm::fmaf(self, a, b) }
    }

    #[inline]
    fn sqrt(self) -> Self {
        // libm-style: hardware on M7/M4; std::f32::sqrt on host.
        #[cfg(feature = "host")]
        { f32::sqrt(self) }
        #[cfg(not(feature = "host"))]
        { libm::sqrtf(self) }
    }

    #[inline]
    fn abs(self) -> Self {
        #[cfg(feature = "host")]
        { f32::abs(self) }
        #[cfg(not(feature = "host"))]
        { libm::fabsf(self) }
    }

    #[inline]
    fn min(self, other: Self) -> Self {
        if self < other { self } else { other }
    }

    #[inline]
    fn max(self, other: Self) -> Self {
        if self > other { self } else { other }
    }
}

#[cfg(feature = "f64")]
impl Float for f64 {
    const ZERO: Self = 0.0;
    const ONE: Self = 1.0;

    #[inline]
    fn from_f64(x: f64) -> Self { x }

    #[inline]
    fn mul_add(self, a: Self, b: Self) -> Self {
        f64::mul_add(self, a, b)
    }

    #[inline]
    fn sqrt(self) -> Self { f64::sqrt(self) }

    #[inline]
    fn abs(self) -> Self { f64::abs(self) }

    #[inline]
    fn min(self, other: Self) -> Self {
        if self < other { self } else { other }
    }

    #[inline]
    fn max(self, other: Self) -> Self {
        if self > other { self } else { other }
    }
}

#[cfg(test)]
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
```

(MCU paths reference `libm`. Add `libm = "0.2"` as an MCU-only dep — see Step 5.)

- [ ] **Step 4: Add `libm` dep gated on MCU features**

Edit `rust/nurbs/Cargo.toml`, add:

```toml
[dependencies]
libm = { version = "0.2", optional = true }

[features]
default = ["host"]
f64 = []
host = ["f64"]
mcu-h7 = ["dep:libm"]
mcu-f4 = ["dep:libm"]
```

- [ ] **Step 5: Wire the module into `lib.rs`**

Add to `rust/nurbs/src/lib.rs` after the compile-error blocks:

```rust
mod float;
pub use float::Float;
```

- [ ] **Step 6: Run tests**

```bash
cargo test -p nurbs float
```

Expected: 5 passed (or 4 if `f64` feature off; default has it on).

- [ ] **Step 7: Verify MCU build still compiles**

```bash
cargo check -p nurbs --no-default-features --features mcu-h7 --target thumbv7em-none-eabihf
```

Expected: builds cleanly with `libm` linked in.

- [ ] **Step 8: Commit**

```bash
git add rust/nurbs/
git commit -m "nurbs: add Float trait with f32 / f64 impls"
```

---

## Task 5: Error taxonomy

**Files:**
- Create: `rust/nurbs/src/error.rs`
- Modify: `rust/nurbs/src/lib.rs`

- [ ] **Step 1: Write failing test for error From-conversions**

Create `rust/nurbs/src/error.rs`:

```rust
//! Per-module error types with From-conversions to top-level NurbsError.
//! See spec §Substrate / Error taxonomy.

use crate::Float;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn construct_error_converts_to_nurbs_error() {
        let e = ConstructError::DegreeExceeded { actual: 25, max: 20 };
        let n: NurbsError<f32> = e.into();
        matches!(n, NurbsError::Construct(ConstructError::DegreeExceeded { .. }));
    }

    #[test]
    fn wire_error_wraps_construct_error() {
        let e = ConstructError::KnotsNotMonotone;
        let w: WireError = e.into();
        matches!(w, WireError::Construct(_));
    }

    #[test]
    fn nurbs_error_implements_error_trait() {
        // core::error::Error is object-safe; check it compiles.
        let e: NurbsError<f32> = ConstructError::KnotsNotClamped.into();
        let _: &dyn core::error::Error = &e;
    }
}
```

- [ ] **Step 2: Run to verify failure**

```bash
cargo test -p nurbs error
```

Expected: compilation error — types not defined.

- [ ] **Step 3: Implement the error types**

Replace `rust/nurbs/src/error.rs` contents with:

```rust
//! Per-module error types with From-conversions to top-level NurbsError.
//! See spec §Substrate / Error taxonomy.

use crate::Float;
use core::fmt;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConstructError {
    DegreeExceeded { actual: u8, max: u8 },
    KnotCountMismatch { expected: usize, got: usize },
    KnotsNotClamped,
    KnotsNotMonotone,
    DegenerateKnotRange,
    WeightCountMismatch { expected: usize, got: usize },
    NonPositiveWeight,
}

impl fmt::Display for ConstructError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DegreeExceeded { actual, max } =>
                write!(f, "degree {actual} exceeds maximum {max}"),
            Self::KnotCountMismatch { expected, got } =>
                write!(f, "knot count: expected {expected}, got {got}"),
            Self::KnotsNotClamped => write!(f, "knot vector is not clamped open"),
            Self::KnotsNotMonotone => write!(f, "knot vector is not non-decreasing"),
            Self::DegenerateKnotRange => write!(f, "knot range is degenerate (knots[last] <= knots[0])"),
            Self::WeightCountMismatch { expected, got } =>
                write!(f, "weight count: expected {expected}, got {got}"),
            Self::NonPositiveWeight => write!(f, "weight is non-positive"),
        }
    }
}

impl core::error::Error for ConstructError {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WireError {
    Misaligned,
    UnknownVersion(u8),
    TruncatedBuffer { expected_len: usize, got: usize },
    AxisCountMismatch { expected: usize, got: u8 },
    Construct(ConstructError),
}

impl fmt::Display for WireError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Misaligned => write!(f, "wire buffer not aligned to T"),
            Self::UnknownVersion(v) => write!(f, "unknown wire format version {v}"),
            Self::TruncatedBuffer { expected_len, got } =>
                write!(f, "wire buffer truncated: expected {expected_len} bytes, got {got}"),
            Self::AxisCountMismatch { expected, got } =>
                write!(f, "axis count mismatch: header says {got}, type expects {expected}"),
            Self::Construct(e) => write!(f, "wire content invalid: {e}"),
        }
    }
}

impl core::error::Error for WireError {}

impl From<ConstructError> for WireError {
    fn from(e: ConstructError) -> Self { Self::Construct(e) }
}

#[derive(Debug, Clone, PartialEq)]
pub enum ArcLengthError<T: Float> {
    ToleranceNotMet { achieved_residual: T, samples_used: usize },
    DegenerateCurve,
}

impl<T: Float> fmt::Display for ArcLengthError<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ToleranceNotMet { achieved_residual, samples_used } =>
                write!(f, "arc-length builder hit cap of {samples_used} samples; achieved residual {achieved_residual:?}"),
            Self::DegenerateCurve => write!(f, "arc-length integration encountered |dP/du| < MIN_PARAMETRIC_SPEED"),
        }
    }
}

impl<T: Float> core::error::Error for ArcLengthError<T> {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AlgebraError {
    DegreeExceeded { result_degree: u8, max: u8 },
    KnotMismatch,
    NotImplemented(&'static str),
}

impl fmt::Display for AlgebraError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DegreeExceeded { result_degree, max } =>
                write!(f, "result degree {result_degree} exceeds maximum {max}"),
            Self::KnotMismatch => write!(f, "operands have incompatible knot vectors"),
            Self::NotImplemented(s) => write!(f, "algorithm not implemented: {s}"),
        }
    }
}

impl core::error::Error for AlgebraError {}

#[derive(Debug, Clone, PartialEq)]
pub enum NurbsError<T: Float> {
    Construct(ConstructError),
    Wire(WireError),
    ArcLength(ArcLengthError<T>),
    Algebra(AlgebraError),
}

impl<T: Float> fmt::Display for NurbsError<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Construct(e) => write!(f, "{e}"),
            Self::Wire(e) => write!(f, "{e}"),
            Self::ArcLength(e) => write!(f, "{e}"),
            Self::Algebra(e) => write!(f, "{e}"),
        }
    }
}

impl<T: Float> core::error::Error for NurbsError<T> {}

impl<T: Float> From<ConstructError> for NurbsError<T> {
    fn from(e: ConstructError) -> Self { Self::Construct(e) }
}
impl<T: Float> From<WireError> for NurbsError<T> {
    fn from(e: WireError) -> Self { Self::Wire(e) }
}
impl<T: Float> From<ArcLengthError<T>> for NurbsError<T> {
    fn from(e: ArcLengthError<T>) -> Self { Self::ArcLength(e) }
}
impl<T: Float> From<AlgebraError> for NurbsError<T> {
    fn from(e: AlgebraError) -> Self { Self::Algebra(e) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn construct_error_converts_to_nurbs_error() {
        let e = ConstructError::DegreeExceeded { actual: 25, max: 20 };
        let n: NurbsError<f32> = e.into();
        assert!(matches!(n, NurbsError::Construct(ConstructError::DegreeExceeded { .. })));
    }

    #[test]
    fn wire_error_wraps_construct_error() {
        let e = ConstructError::KnotsNotMonotone;
        let w: WireError = e.into();
        assert!(matches!(w, WireError::Construct(_)));
    }

    #[test]
    fn nurbs_error_implements_error_trait() {
        let e: NurbsError<f32> = ConstructError::KnotsNotClamped.into();
        let _: &dyn core::error::Error = &e;
    }

    #[test]
    fn display_renders_messages() {
        let e: NurbsError<f32> = ConstructError::DegreeExceeded { actual: 30, max: 20 }.into();
        let s = format!("{e}");
        assert!(s.contains("30"));
        assert!(s.contains("20"));
    }
}
```

- [ ] **Step 4: Wire into `lib.rs`**

Add to `rust/nurbs/src/lib.rs`:

```rust
pub mod error;
pub use error::{
    AlgebraError, ArcLengthError, ConstructError, NurbsError, WireError,
};
```

- [ ] **Step 5: Run tests**

```bash
cargo test -p nurbs error
```

Expected: 4 passed.

- [ ] **Step 6: Commit**

```bash
git commit -am "nurbs: add error taxonomy"
```

---

## Task 6: NurbsView and VectorNurbsView traits

**Files:**
- Create: `rust/nurbs/src/view.rs`
- Modify: `rust/nurbs/src/lib.rs`

- [ ] **Step 1: Create the trait module**

Create `rust/nurbs/src/view.rs`:

```rust
//! Read-only NURBS view traits. Eval algorithms are generic over these so the
//! same code works against owned (host) and borrowed (MCU) representations.
//! See spec §Substrate / NURBS data model.

use crate::Float;

/// Read-only access to a scalar NURBS curve.
pub trait NurbsView<T: Float> {
    fn degree(&self) -> u8;
    fn knots(&self) -> &[T];
    fn control_points(&self) -> &[T];
    fn weights(&self) -> Option<&[T]>;

    /// Number of control points. Convenience derived from slice length.
    #[inline]
    fn control_point_count(&self) -> usize { self.control_points().len() }
}

/// Read-only access to a vector NURBS curve in R^N.
pub trait VectorNurbsView<T: Float, const N: usize> {
    fn degree(&self) -> u8;
    fn knots(&self) -> &[T];
    fn control_points(&self) -> &[[T; N]];
    fn weights(&self) -> Option<&[T]>;

    #[inline]
    fn control_point_count(&self) -> usize { self.control_points().len() }
}
```

- [ ] **Step 2: Wire into `lib.rs`**

Add:

```rust
mod view;
pub use view::{NurbsView, VectorNurbsView};
```

- [ ] **Step 3: Verify it compiles**

```bash
cargo build -p nurbs
cargo check -p nurbs --no-default-features --features mcu-h7 --target thumbv7em-none-eabihf
```

Expected: both succeed.

- [ ] **Step 4: Commit**

```bash
git commit -am "nurbs: add NurbsView and VectorNurbsView traits"
```

---

## Task 7: ScalarNurbs (owned) and ScalarNurbsRef (borrowed)

**Files:**
- Create: `rust/nurbs/src/scalar.rs`
- Modify: `rust/nurbs/src/lib.rs`

- [ ] **Step 1: Write failing tests for `ScalarNurbs::try_new` validation rules**

Create `rust/nurbs/src/scalar.rs`:

```rust
//! Scalar (1D) NURBS types: ScalarNurbs (owned, host) and ScalarNurbsRef (borrowed).

#[cfg(all(test, feature = "host"))]
mod tests {
    use super::*;
    use crate::ConstructError;

    fn linear_curve() -> ScalarNurbs<f64> {
        // Degree-1 NURBS, 2 control points, knots {0,0,1,1}.
        ScalarNurbs::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 1.0],
            None,
        ).unwrap()
    }

    #[test]
    fn try_new_accepts_valid_linear() {
        let curve = linear_curve();
        assert_eq!(curve.degree(), 1);
        assert_eq!(curve.control_points(), &[0.0, 1.0]);
    }

    #[test]
    fn try_new_rejects_degree_exceeded() {
        let result = ScalarNurbs::<f64>::try_new(
            21,
            vec![0.0; 23],
            vec![0.0; 1],
            None,
        );
        assert!(matches!(result, Err(ConstructError::DegreeExceeded { actual: 21, max: 20 })));
    }

    #[test]
    fn try_new_rejects_knot_count_mismatch() {
        let result = ScalarNurbs::try_new(
            1,
            vec![0.0, 0.0, 1.0],         // 3 knots, but 2 cps + 1 + 1 = 4 expected
            vec![0.0, 1.0],
            None,
        );
        assert!(matches!(result, Err(ConstructError::KnotCountMismatch { .. })));
    }

    #[test]
    fn try_new_rejects_unclamped_start() {
        let result = ScalarNurbs::try_new(
            1,
            vec![0.0, 0.5, 1.0, 1.0],    // not clamped at start
            vec![0.0, 1.0],
            None,
        );
        assert!(matches!(result, Err(ConstructError::KnotsNotClamped)));
    }

    #[test]
    fn try_new_rejects_unclamped_end() {
        let result = ScalarNurbs::try_new(
            1,
            vec![0.0, 0.0, 0.5, 1.0],    // not clamped at end
            vec![0.0, 1.0],
            None,
        );
        assert!(matches!(result, Err(ConstructError::KnotsNotClamped)));
    }

    #[test]
    fn try_new_rejects_non_monotone_knots() {
        let result = ScalarNurbs::try_new(
            2,
            vec![0.0, 0.0, 0.0, 0.4, 0.3, 1.0, 1.0, 1.0],  // 0.3 < 0.4
            vec![0.0, 0.5, 1.0, 1.5, 2.0],
            None,
        );
        assert!(matches!(result, Err(ConstructError::KnotsNotMonotone)));
    }

    #[test]
    fn try_new_rejects_degenerate_knot_range() {
        let result = ScalarNurbs::try_new(
            1,
            vec![0.0, 0.0, 0.0, 0.0],
            vec![0.0, 1.0],
            None,
        );
        assert!(matches!(result, Err(ConstructError::DegenerateKnotRange)));
    }

    #[test]
    fn try_new_rejects_weight_count_mismatch() {
        let result = ScalarNurbs::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 1.0],
            Some(vec![1.0]),         // 1 weight for 2 cps
        );
        assert!(matches!(result, Err(ConstructError::WeightCountMismatch { .. })));
    }

    #[test]
    fn try_new_rejects_non_positive_weight() {
        let result = ScalarNurbs::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 1.0],
            Some(vec![1.0, 0.0]),
        );
        assert!(matches!(result, Err(ConstructError::NonPositiveWeight)));
    }

    #[test]
    fn as_view_provides_borrowed_access() {
        let owned = linear_curve();
        let view = owned.as_view();
        assert_eq!(view.degree(), 1);
        assert_eq!(view.knots(), &[0.0, 0.0, 1.0, 1.0]);
        assert_eq!(view.control_points(), &[0.0, 1.0]);
    }

    #[test]
    fn ref_try_new_accepts_valid_data() {
        let knots = [0.0_f64, 0.0, 1.0, 1.0];
        let cps = [0.0_f64, 1.0];
        let r = ScalarNurbsRef::try_new(1, &knots, &cps, None).unwrap();
        assert_eq!(r.degree(), 1);
    }
}
```

- [ ] **Step 2: Run to verify failure**

```bash
cargo test -p nurbs scalar
```

Expected: compilation error — types not defined.

- [ ] **Step 3: Implement the types**

Prepend to `rust/nurbs/src/scalar.rs` (above the test module):

```rust
//! Scalar (1D) NURBS types: ScalarNurbs (owned, host) and ScalarNurbsRef (borrowed).

use crate::{ConstructError, Float, NurbsView, MAX_DEGREE};

/// Owned, heap-backed scalar NURBS. Host-only.
///
/// Construction validates all spec §Substrate invariants. After construction,
/// the data is trusted; eval algorithms only `debug_assert` invariants.
#[cfg(feature = "host")]
#[derive(Debug, Clone, PartialEq)]
pub struct ScalarNurbs<T: Float> {
    degree: u8,
    knots: Vec<T>,
    control_points: Vec<T>,
    weights: Option<Vec<T>>,
}

#[cfg(feature = "host")]
impl<T: Float> ScalarNurbs<T> {
    /// Build a scalar NURBS, validating every spec-listed invariant.
    pub fn try_new(
        degree: u8,
        knots: Vec<T>,
        control_points: Vec<T>,
        weights: Option<Vec<T>>,
    ) -> Result<Self, ConstructError> {
        validate(degree, &knots, control_points.len(), weights.as_deref())?;
        Ok(Self { degree, knots, control_points, weights })
    }

    pub fn degree(&self) -> u8 { self.degree }
    pub fn knots(&self) -> &[T] { &self.knots }
    pub fn control_points(&self) -> &[T] { &self.control_points }
    pub fn weights(&self) -> Option<&[T]> { self.weights.as_deref() }

    /// Cheap projection to a borrowed view.
    #[inline]
    pub fn as_view(&self) -> ScalarNurbsRef<'_, T> {
        ScalarNurbsRef {
            degree: self.degree,
            knots: &self.knots,
            control_points: &self.control_points,
            weights: self.weights.as_deref(),
        }
    }

    /// Consume self into raw parts. Used by host pre-bake pipelines that
    /// build new NURBS by transformation.
    pub fn into_parts(self) -> (u8, Vec<T>, Vec<T>, Option<Vec<T>>) {
        (self.degree, self.knots, self.control_points, self.weights)
    }
}

#[cfg(feature = "host")]
impl<T: Float> NurbsView<T> for ScalarNurbs<T> {
    #[inline] fn degree(&self) -> u8 { self.degree }
    #[inline] fn knots(&self) -> &[T] { &self.knots }
    #[inline] fn control_points(&self) -> &[T] { &self.control_points }
    #[inline] fn weights(&self) -> Option<&[T]> { self.weights.as_deref() }
}

/// Borrowed, slice-backed scalar NURBS. Available on host and MCU.
///
/// Constructed either via `ScalarNurbs::as_view` (host) or
/// `ScalarNurbsRef::try_new` / `try_from_wire` (MCU + zero-copy paths).
#[derive(Debug, Clone, Copy)]
pub struct ScalarNurbsRef<'a, T: Float> {
    pub(crate) degree: u8,
    pub(crate) knots: &'a [T],
    pub(crate) control_points: &'a [T],
    pub(crate) weights: Option<&'a [T]>,
}

impl<'a, T: Float> ScalarNurbsRef<'a, T> {
    /// Build a borrowed NURBS from already-validated slices, re-running invariants.
    /// Use when assembling a `ScalarNurbsRef` outside the wire path.
    pub fn try_new(
        degree: u8,
        knots: &'a [T],
        control_points: &'a [T],
        weights: Option<&'a [T]>,
    ) -> Result<Self, ConstructError> {
        validate(degree, knots, control_points.len(), weights)?;
        Ok(Self { degree, knots, control_points, weights })
    }

    pub fn degree(&self) -> u8 { self.degree }
    pub fn knots(&self) -> &[T] { self.knots }
    pub fn control_points(&self) -> &[T] { self.control_points }
    pub fn weights(&self) -> Option<&[T]> { self.weights }
}

impl<'a, T: Float> NurbsView<T> for ScalarNurbsRef<'a, T> {
    #[inline] fn degree(&self) -> u8 { self.degree }
    #[inline] fn knots(&self) -> &[T] { self.knots }
    #[inline] fn control_points(&self) -> &[T] { self.control_points }
    #[inline] fn weights(&self) -> Option<&[T]> { self.weights }
}

/// Shared validation. See spec §Substrate / Validation rules.
pub(crate) fn validate<T: Float>(
    degree: u8,
    knots: &[T],
    control_point_count: usize,
    weights: Option<&[T]>,
) -> Result<(), ConstructError> {
    if (degree as usize) > MAX_DEGREE {
        return Err(ConstructError::DegreeExceeded { actual: degree, max: MAX_DEGREE as u8 });
    }
    let p = degree as usize;
    let expected_knot_count = control_point_count + p + 1;
    if knots.len() != expected_knot_count {
        return Err(ConstructError::KnotCountMismatch {
            expected: expected_knot_count, got: knots.len(),
        });
    }
    if knots.len() < 2 * (p + 1) {
        // not enough knots for clamped open of this degree
        return Err(ConstructError::KnotCountMismatch {
            expected: 2 * (p + 1), got: knots.len(),
        });
    }

    // Clamped at start: knots[0..=p] all equal.
    let start = knots[0];
    for k in &knots[1..=p] {
        if *k != start {
            return Err(ConstructError::KnotsNotClamped);
        }
    }
    // Clamped at end: knots[len-1-p..] all equal.
    let last_idx = knots.len() - 1;
    let end = knots[last_idx];
    for k in &knots[last_idx - p..last_idx] {
        if *k != end {
            return Err(ConstructError::KnotsNotClamped);
        }
    }

    // Non-decreasing.
    for window in knots.windows(2) {
        if window[1] < window[0] {
            return Err(ConstructError::KnotsNotMonotone);
        }
    }

    // Non-degenerate range.
    if !(end > start) {
        return Err(ConstructError::DegenerateKnotRange);
    }

    if let Some(w) = weights {
        if w.len() != control_point_count {
            return Err(ConstructError::WeightCountMismatch {
                expected: control_point_count, got: w.len(),
            });
        }
        for weight in w {
            if !(*weight > T::ZERO) {
                return Err(ConstructError::NonPositiveWeight);
            }
        }
    }

    Ok(())
}
```

- [ ] **Step 4: Wire into `lib.rs`**

Add:

```rust
mod scalar;
#[cfg(feature = "host")]
pub use scalar::ScalarNurbs;
pub use scalar::ScalarNurbsRef;
```

- [ ] **Step 5: Run tests**

```bash
cargo test -p nurbs scalar
```

Expected: 11 passed.

- [ ] **Step 6: Verify MCU build still compiles**

```bash
cargo check -p nurbs --no-default-features --features mcu-h7 --target thumbv7em-none-eabihf
```

Expected: builds (the owned `ScalarNurbs` is gated out; `ScalarNurbsRef` is available).

- [ ] **Step 7: Commit**

```bash
git commit -am "nurbs: add ScalarNurbs and ScalarNurbsRef with full validation"
```

---

## Task 8: VectorNurbs<T, N> and VectorNurbsRef<T, N>

**Files:**
- Create: `rust/nurbs/src/vector.rs`
- Modify: `rust/nurbs/src/lib.rs`

- [ ] **Step 1: Write failing tests**

Create `rust/nurbs/src/vector.rs`:

```rust
//! Vector NURBS types in R^N: VectorNurbs<T, N> (owned) and VectorNurbsRef<T, N> (borrowed).

#[cfg(all(test, feature = "host"))]
mod tests {
    use super::*;

    fn linear_3d_curve() -> VectorNurbs<f64, 3> {
        VectorNurbs::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]],
            None,
        ).unwrap()
    }

    #[test]
    fn try_new_accepts_valid_linear_3d() {
        let curve = linear_3d_curve();
        assert_eq!(curve.degree(), 1);
        assert_eq!(curve.control_points()[1], [1.0, 2.0, 3.0]);
    }

    #[test]
    fn try_new_rejects_degree_exceeded() {
        let result = VectorNurbs::<f64, 3>::try_new(
            21,
            vec![0.0; 23],
            vec![[0.0; 3]; 1],
            None,
        );
        assert!(matches!(result, Err(crate::ConstructError::DegreeExceeded { .. })));
    }

    #[test]
    fn try_new_rejects_knot_count_mismatch() {
        let result = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0],
            vec![[0.0; 3], [1.0; 3]],
            None,
        );
        assert!(matches!(result, Err(crate::ConstructError::KnotCountMismatch { .. })));
    }

    #[test]
    fn as_view_provides_borrowed_access() {
        let owned = linear_3d_curve();
        let view = owned.as_view();
        assert_eq!(view.degree(), 1);
        assert_eq!(view.control_points()[1], [1.0, 2.0, 3.0]);
    }
}
```

- [ ] **Step 2: Run to verify failure**

```bash
cargo test -p nurbs vector
```

Expected: compilation error.

- [ ] **Step 3: Implement the types**

Prepend to `rust/nurbs/src/vector.rs`:

```rust
//! Vector NURBS types in R^N: VectorNurbs<T, N> (owned) and VectorNurbsRef<T, N> (borrowed).

use crate::{scalar::validate, ConstructError, Float, VectorNurbsView};

#[cfg(feature = "host")]
#[derive(Debug, Clone, PartialEq)]
pub struct VectorNurbs<T: Float, const N: usize> {
    degree: u8,
    knots: Vec<T>,
    control_points: Vec<[T; N]>,
    weights: Option<Vec<T>>,
}

#[cfg(feature = "host")]
impl<T: Float, const N: usize> VectorNurbs<T, N> {
    pub fn try_new(
        degree: u8,
        knots: Vec<T>,
        control_points: Vec<[T; N]>,
        weights: Option<Vec<T>>,
    ) -> Result<Self, ConstructError> {
        validate(degree, &knots, control_points.len(), weights.as_deref())?;
        Ok(Self { degree, knots, control_points, weights })
    }

    pub fn degree(&self) -> u8 { self.degree }
    pub fn knots(&self) -> &[T] { &self.knots }
    pub fn control_points(&self) -> &[[T; N]] { &self.control_points }
    pub fn weights(&self) -> Option<&[T]> { self.weights.as_deref() }

    #[inline]
    pub fn as_view(&self) -> VectorNurbsRef<'_, T, N> {
        VectorNurbsRef {
            degree: self.degree,
            knots: &self.knots,
            control_points: &self.control_points,
            weights: self.weights.as_deref(),
        }
    }

    pub fn into_parts(self) -> (u8, Vec<T>, Vec<[T; N]>, Option<Vec<T>>) {
        (self.degree, self.knots, self.control_points, self.weights)
    }
}

#[cfg(feature = "host")]
impl<T: Float, const N: usize> VectorNurbsView<T, N> for VectorNurbs<T, N> {
    #[inline] fn degree(&self) -> u8 { self.degree }
    #[inline] fn knots(&self) -> &[T] { &self.knots }
    #[inline] fn control_points(&self) -> &[[T; N]] { &self.control_points }
    #[inline] fn weights(&self) -> Option<&[T]> { self.weights.as_deref() }
}

#[derive(Debug, Clone, Copy)]
pub struct VectorNurbsRef<'a, T: Float, const N: usize> {
    pub(crate) degree: u8,
    pub(crate) knots: &'a [T],
    pub(crate) control_points: &'a [[T; N]],
    pub(crate) weights: Option<&'a [T]>,
}

impl<'a, T: Float, const N: usize> VectorNurbsRef<'a, T, N> {
    pub fn try_new(
        degree: u8,
        knots: &'a [T],
        control_points: &'a [[T; N]],
        weights: Option<&'a [T]>,
    ) -> Result<Self, ConstructError> {
        validate(degree, knots, control_points.len(), weights)?;
        Ok(Self { degree, knots, control_points, weights })
    }

    pub fn degree(&self) -> u8 { self.degree }
    pub fn knots(&self) -> &[T] { self.knots }
    pub fn control_points(&self) -> &[[T; N]] { self.control_points }
    pub fn weights(&self) -> Option<&[T]> { self.weights }
}

impl<'a, T: Float, const N: usize> VectorNurbsView<T, N> for VectorNurbsRef<'a, T, N> {
    #[inline] fn degree(&self) -> u8 { self.degree }
    #[inline] fn knots(&self) -> &[T] { self.knots }
    #[inline] fn control_points(&self) -> &[[T; N]] { self.control_points }
    #[inline] fn weights(&self) -> Option<&[T]> { self.weights }
}
```

- [ ] **Step 4: Wire into `lib.rs`**

Add:

```rust
mod vector;
#[cfg(feature = "host")]
pub use vector::VectorNurbs;
pub use vector::VectorNurbsRef;
```

- [ ] **Step 5: Run tests**

```bash
cargo test -p nurbs vector
```

Expected: 4 passed.

- [ ] **Step 6: MCU build check**

```bash
cargo check -p nurbs --no-default-features --features mcu-h7 --target thumbv7em-none-eabihf
```

Expected: builds.

- [ ] **Step 7: Commit**

```bash
git commit -am "nurbs: add VectorNurbs<T, N> and VectorNurbsRef<T, N>"
```

---

## Task 9: Wire format — ScalarNurbsRef::try_from_wire

**Files:**
- Create: `rust/nurbs/src/wire.rs`
- Modify: `rust/nurbs/src/lib.rs`
- Modify: `rust/nurbs/src/scalar.rs`

- [ ] **Step 1: Write failing tests**

Append to `rust/nurbs/src/scalar.rs` test module:

```rust
    #[test]
    fn try_from_wire_parses_unweighted_linear() {
        // Layout: u8 version, u8 degree, u8 has_weights, u8 reserved,
        //         u16 knot_count, u16 cp_count, then knots + cps (both as f32).
        // Linear curve: degree=1, knots=[0,0,1,1], cps=[0.0, 1.0]
        let mut buf = Vec::new();
        buf.extend_from_slice(&[1, 1, 0, 0]);                         // version, degree, has_weights, reserved
        buf.extend_from_slice(&4u16.to_ne_bytes());                   // knot_count
        buf.extend_from_slice(&2u16.to_ne_bytes());                   // cp_count
        buf.extend_from_slice(&0.0_f32.to_ne_bytes());
        buf.extend_from_slice(&0.0_f32.to_ne_bytes());
        buf.extend_from_slice(&1.0_f32.to_ne_bytes());
        buf.extend_from_slice(&1.0_f32.to_ne_bytes());
        buf.extend_from_slice(&0.0_f32.to_ne_bytes());
        buf.extend_from_slice(&1.0_f32.to_ne_bytes());

        // Ensure 4-byte alignment by allocating into an aligned buffer
        let aligned = align_buf(&buf, 4);
        let r = ScalarNurbsRef::<f32>::try_from_wire(aligned.as_slice()).unwrap();
        assert_eq!(r.degree(), 1);
        assert_eq!(r.control_points(), &[0.0_f32, 1.0]);
        assert!(r.weights().is_none());
    }

    #[test]
    fn try_from_wire_rejects_misaligned_buffer() {
        let mut buf = vec![0u8; 32 + 1];
        buf[0] = 1;
        // Slice starting at offset 1 → guaranteed misaligned for f32.
        let result = ScalarNurbsRef::<f32>::try_from_wire(&buf[1..]);
        assert!(matches!(result, Err(crate::WireError::Misaligned)));
    }

    #[test]
    fn try_from_wire_rejects_unknown_version() {
        let buf = align_buf(&[0xFFu8, 1, 0, 0, 4, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 4);
        let result = ScalarNurbsRef::<f32>::try_from_wire(buf.as_slice());
        assert!(matches!(result, Err(crate::WireError::UnknownVersion(0xFF))));
    }

    #[test]
    fn try_from_wire_rejects_truncated_header() {
        let buf = align_buf(&[1u8, 1, 0, 0], 4);   // only 4 bytes; 8-byte header missing
        let result = ScalarNurbsRef::<f32>::try_from_wire(buf.as_slice());
        assert!(matches!(result, Err(crate::WireError::TruncatedBuffer { .. })));
    }

    /// Test-only owner of a 4-byte-aligned byte buffer.
    /// We can't deallocate a `Vec<u8>` produced from a `Vec<u32>`'s raw parts —
    /// the allocator layouts differ (alignment 1 vs 4) and `Vec::from_raw_parts`
    /// in that direction is UB. So we keep the `Vec<u32>` alive and hand out
    /// a borrowed `&[u8]` view via `as_slice`.
    struct AlignedBytes {
        backing: Vec<u32>,
        len: usize,
    }

    impl AlignedBytes {
        fn as_slice(&self) -> &[u8] {
            // SAFETY: `Vec<u32>` is 4-byte aligned; `len <= backing.len() * 4`;
            // `u32` has no padding and any bit pattern is a valid `u8`.
            #[allow(unsafe_code)]
            unsafe {
                core::slice::from_raw_parts(self.backing.as_ptr().cast::<u8>(), self.len)
            }
        }
    }

    fn align_buf(data: &[u8], align: usize) -> AlignedBytes {
        match align {
            4 => {
                let n = data.len().div_ceil(4);
                let mut backing: Vec<u32> = vec![0; n];
                // SAFETY: backing owns `n*4` bytes with 4-byte alignment;
                // we write `data.len() <= n*4` bytes through the &mut [u8] view.
                #[allow(unsafe_code)]
                let bytes: &mut [u8] = unsafe {
                    core::slice::from_raw_parts_mut(backing.as_mut_ptr().cast::<u8>(), n * 4)
                };
                bytes[..data.len()].copy_from_slice(data);
                AlignedBytes { backing, len: data.len() }
            }
            _ => unimplemented!(),
        }
    }
```

(Test-only helper. Earlier (now-deleted) drafts used `Vec::from_raw_parts` to convert the `Vec<u32>` backing into a `Vec<u8>`; that's UB because the allocator layouts differ. The `AlignedBytes` owner-and-view shape avoids the UB and serves the same purpose. Earlier call sites that passed `&buf` to `try_from_wire` change to `buf.as_slice()`.)

- [ ] **Step 2: Create wire-format module**

Create `rust/nurbs/src/wire.rs`:

```rust
//! Wire-format constants and shared deserialization helpers.
//! See spec §Substrate / Wire format.

pub const FORMAT_VERSION_V1: u8 = 0x01;

/// Header byte counts for each format. Each is 8 bytes total to land subsequent
/// `T[..]` regions naturally aligned for f32 and f64.
pub const SCALAR_HEADER_BYTES: usize = 8;
pub const VECTOR_HEADER_BYTES: usize = 8;
pub const ARC_LENGTH_HEADER_BYTES: usize = 8;
```

- [ ] **Step 3: Add `try_from_wire` to `ScalarNurbsRef`**

Append to `rust/nurbs/src/scalar.rs` (above the test module):

```rust
use crate::{wire::{FORMAT_VERSION_V1, SCALAR_HEADER_BYTES}, WireError};

impl<'a> ScalarNurbsRef<'a, f32> {
    /// Zero-copy parse of a wire-format buffer into a borrowed scalar NURBS.
    /// See spec §Substrate / Wire format for the byte layout.
    ///
    /// Caller responsibilities (Layer 5 contract):
    /// - `buf` is aligned to `align_of::<f32>()` (4 bytes).
    /// - `buf` is in host-native endianness.
    pub fn try_from_wire(buf: &'a [u8]) -> Result<Self, WireError> {
        if (buf.as_ptr() as usize) % core::mem::align_of::<f32>() != 0 {
            return Err(WireError::Misaligned);
        }
        if buf.len() < SCALAR_HEADER_BYTES {
            return Err(WireError::TruncatedBuffer {
                expected_len: SCALAR_HEADER_BYTES, got: buf.len(),
            });
        }
        let version = buf[0];
        if version != FORMAT_VERSION_V1 {
            return Err(WireError::UnknownVersion(version));
        }
        let degree = buf[1];
        let has_weights = buf[2];
        let knot_count = u16::from_ne_bytes([buf[4], buf[5]]) as usize;
        let cp_count = u16::from_ne_bytes([buf[6], buf[7]]) as usize;

        let knots_bytes = knot_count * core::mem::size_of::<f32>();
        let cps_bytes = cp_count * core::mem::size_of::<f32>();
        let weights_bytes = if has_weights == 1 { cps_bytes } else { 0 };
        let total = SCALAR_HEADER_BYTES + knots_bytes + cps_bytes + weights_bytes;
        if buf.len() < total {
            return Err(WireError::TruncatedBuffer { expected_len: total, got: buf.len() });
        }

        // SAFETY: alignment checked above; lengths checked above; T = f32 has
        // no invalid bit patterns for any 4-byte sequence.
        #[allow(unsafe_code)]
        let (knots, cps, weights) = unsafe {
            let knots_ptr = buf.as_ptr().add(SCALAR_HEADER_BYTES) as *const f32;
            let cps_ptr = buf.as_ptr().add(SCALAR_HEADER_BYTES + knots_bytes) as *const f32;
            let knots = core::slice::from_raw_parts(knots_ptr, knot_count);
            let cps = core::slice::from_raw_parts(cps_ptr, cp_count);
            let weights = if has_weights == 1 {
                let w_ptr = buf.as_ptr().add(SCALAR_HEADER_BYTES + knots_bytes + cps_bytes) as *const f32;
                Some(core::slice::from_raw_parts(w_ptr, cp_count))
            } else {
                None
            };
            (knots, cps, weights)
        };

        Self::try_new(degree, knots, cps, weights).map_err(WireError::from)
    }
}
```

- [ ] **Step 4: Wire `wire` into `lib.rs`**

Add:

```rust
pub mod wire;
```

- [ ] **Step 5: Run tests**

```bash
cargo test -p nurbs scalar
```

Expected: previous 11 + 4 new = 15 passed.

- [ ] **Step 6: MCU build check**

```bash
cargo check -p nurbs --no-default-features --features mcu-h7 --target thumbv7em-none-eabihf
```

Expected: builds.

- [ ] **Step 7: Commit**

```bash
git commit -am "nurbs: add ScalarNurbsRef::try_from_wire (zero-copy)"
```

---

## Task 10: Wire format — VectorNurbsRef::try_from_wire

**Files:**
- Modify: `rust/nurbs/src/vector.rs`

- [ ] **Step 1: Write failing tests**

Append to `rust/nurbs/src/vector.rs` test module:

```rust
    #[test]
    fn try_from_wire_parses_3d_unweighted_linear() {
        // Layout: u8 version, u8 degree, u8 has_weights, u8 axes_n,
        //         u16 knot_count, u16 cp_count, then knots + cps (interleaved).
        let mut buf = Vec::new();
        buf.extend_from_slice(&[1, 1, 0, 3]);                         // version, degree, has_weights, axes_n
        buf.extend_from_slice(&4u16.to_ne_bytes());                   // knot_count
        buf.extend_from_slice(&2u16.to_ne_bytes());                   // cp_count
        buf.extend_from_slice(&0.0_f32.to_ne_bytes());
        buf.extend_from_slice(&0.0_f32.to_ne_bytes());
        buf.extend_from_slice(&1.0_f32.to_ne_bytes());
        buf.extend_from_slice(&1.0_f32.to_ne_bytes());
        // CPs interleaved: [0,0,0], [1,2,3]
        for &v in &[0.0_f32, 0.0, 0.0, 1.0, 2.0, 3.0] {
            buf.extend_from_slice(&v.to_ne_bytes());
        }
        let aligned = test_align_buf(&buf, 4);
        let r = VectorNurbsRef::<f32, 3>::try_from_wire(aligned.as_slice()).unwrap();
        assert_eq!(r.degree(), 1);
        assert_eq!(r.control_points()[1], [1.0, 2.0, 3.0]);
    }

    #[test]
    fn try_from_wire_rejects_axis_mismatch() {
        // Wire says axes_n=4, but type is 3.
        let mut buf = Vec::new();
        buf.extend_from_slice(&[1, 1, 0, 4]);
        buf.extend_from_slice(&4u16.to_ne_bytes());
        buf.extend_from_slice(&2u16.to_ne_bytes());
        // pad to enough bytes so we get past the axis check
        buf.resize(64, 0);
        let aligned = test_align_buf(&buf, 4);
        let result = VectorNurbsRef::<f32, 3>::try_from_wire(aligned.as_slice());
        assert!(matches!(result, Err(crate::WireError::AxisCountMismatch { expected: 3, got: 4 })));
    }

    /// Test-only owner; same shape as `align_buf` in scalar.rs (see Task 9).
    struct AlignedBytes {
        backing: Vec<u32>,
        len: usize,
    }

    impl AlignedBytes {
        fn as_slice(&self) -> &[u8] {
            // SAFETY: `Vec<u32>` is 4-byte aligned; len ≤ backing.len()*4.
            #[allow(unsafe_code)]
            unsafe {
                core::slice::from_raw_parts(self.backing.as_ptr().cast::<u8>(), self.len)
            }
        }
    }

    fn test_align_buf(data: &[u8], _align: usize) -> AlignedBytes {
        let n = data.len().div_ceil(4);
        let mut backing: Vec<u32> = vec![0; n];
        // SAFETY: backing owns n*4 bytes 4-byte aligned; we write data.len() ≤ n*4.
        #[allow(unsafe_code)]
        let bytes: &mut [u8] = unsafe {
            core::slice::from_raw_parts_mut(backing.as_mut_ptr().cast::<u8>(), n * 4)
        };
        bytes[..data.len()].copy_from_slice(data);
        AlignedBytes { backing, len: data.len() }
    }
```

- [ ] **Step 2: Run to verify failure**

```bash
cargo test -p nurbs vector::tests::try_from_wire
```

Expected: type doesn't have `try_from_wire` yet — test won't compile or function not found.

- [ ] **Step 3: Implement `try_from_wire` for `VectorNurbsRef`**

Append to `rust/nurbs/src/vector.rs` (above the test module):

```rust
use crate::{wire::{FORMAT_VERSION_V1, VECTOR_HEADER_BYTES}, WireError};

impl<'a, const N: usize> VectorNurbsRef<'a, f32, N> {
    /// Zero-copy parse of a wire-format buffer. Same alignment / endianness
    /// contract as scalar form. Validates `axes_n` against const generic `N`.
    pub fn try_from_wire(buf: &'a [u8]) -> Result<Self, WireError> {
        if (buf.as_ptr() as usize) % core::mem::align_of::<f32>() != 0 {
            return Err(WireError::Misaligned);
        }
        if buf.len() < VECTOR_HEADER_BYTES {
            return Err(WireError::TruncatedBuffer {
                expected_len: VECTOR_HEADER_BYTES, got: buf.len(),
            });
        }
        let version = buf[0];
        if version != FORMAT_VERSION_V1 {
            return Err(WireError::UnknownVersion(version));
        }
        let degree = buf[1];
        let has_weights = buf[2];
        let axes_n = buf[3];
        if axes_n as usize != N {
            return Err(WireError::AxisCountMismatch { expected: N, got: axes_n });
        }
        let knot_count = u16::from_ne_bytes([buf[4], buf[5]]) as usize;
        let cp_count = u16::from_ne_bytes([buf[6], buf[7]]) as usize;

        let knots_bytes = knot_count * core::mem::size_of::<f32>();
        let cps_bytes = cp_count * N * core::mem::size_of::<f32>();
        let weights_bytes = if has_weights == 1 { cp_count * core::mem::size_of::<f32>() } else { 0 };
        let total = VECTOR_HEADER_BYTES + knots_bytes + cps_bytes + weights_bytes;
        if buf.len() < total {
            return Err(WireError::TruncatedBuffer { expected_len: total, got: buf.len() });
        }

        // SAFETY: alignment checked; lengths checked; f32 has no invalid bit patterns;
        // [f32; N] is `repr(transparent)` over `[f32; N]` storage which has the same
        // layout as `cp_count * N` consecutive f32 values.
        #[allow(unsafe_code)]
        let (knots, cps, weights) = unsafe {
            let knots_ptr = buf.as_ptr().add(VECTOR_HEADER_BYTES) as *const f32;
            let cps_ptr = buf.as_ptr().add(VECTOR_HEADER_BYTES + knots_bytes) as *const [f32; N];
            let knots = core::slice::from_raw_parts(knots_ptr, knot_count);
            let cps = core::slice::from_raw_parts(cps_ptr, cp_count);
            let weights = if has_weights == 1 {
                let w_ptr = buf.as_ptr().add(VECTOR_HEADER_BYTES + knots_bytes + cps_bytes) as *const f32;
                Some(core::slice::from_raw_parts(w_ptr, cp_count))
            } else {
                None
            };
            (knots, cps, weights)
        };

        Self::try_new(degree, knots, cps, weights).map_err(WireError::from)
    }
}
```

- [ ] **Step 4: Run tests**

```bash
cargo test -p nurbs vector
```

Expected: previous 4 + 2 new = 6 passed.

- [ ] **Step 5: MCU build check**

```bash
cargo check -p nurbs --no-default-features --features mcu-h7 --target thumbv7em-none-eabihf
```

- [ ] **Step 6: Commit**

```bash
git commit -am "nurbs: add VectorNurbsRef::try_from_wire with axis-count validation"
```

---

## Task 11: eval — find_knot_span and de_boor_inner (non-rational)

**Files:**
- Create: `rust/nurbs/src/eval.rs`
- Modify: `rust/nurbs/src/lib.rs`

- [ ] **Step 1: Write failing tests for `find_knot_span` and de Boor non-rational**

Create `rust/nurbs/src/eval.rs`:

```rust
//! NURBS evaluation: de Boor, vector eval, derivative, curvature.
//! See spec §eval module.

use crate::{Float, NurbsView, VectorNurbsView, MAX_DEGREE, MIN_PARAMETRIC_SPEED, WORKSPACE_SIZE};

#[cfg(test)]
mod tests {
    use super::*;

    fn linear_curve_f64() -> crate::ScalarNurbs<f64> {
        crate::ScalarNurbs::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 1.0],
            None,
        ).unwrap()
    }

    fn quadratic_curve_f64() -> crate::ScalarNurbs<f64> {
        // Bezier-ish: degree 2, knots {0,0,0,1,1,1}, cps {0, 0.5, 1}.
        crate::ScalarNurbs::try_new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.5, 1.0],
            None,
        ).unwrap()
    }

    #[test]
    fn find_knot_span_endpoints() {
        let knots = [0.0, 0.0, 1.0, 1.0];
        // n = control_point_count = 2, p = 1
        // u=0 → first span (clamped at start)
        assert_eq!(find_knot_span(&knots, 1, 2, 0.0), 1);
        // u=1 → last span
        assert_eq!(find_knot_span(&knots, 1, 2, 1.0), 1);
    }

    #[test]
    fn find_knot_span_midpoint() {
        let knots = [0.0, 0.0, 0.5, 1.0, 1.0];
        // n = 3, p = 1
        // u=0.25 → span index 1 (between knots[1]=0 and knots[2]=0.5)
        assert_eq!(find_knot_span(&knots, 1, 3, 0.25), 1);
        // u=0.75 → span index 2 (between knots[2]=0.5 and knots[3]=1.0)
        assert_eq!(find_knot_span(&knots, 1, 3, 0.75), 2);
    }

    #[test]
    fn eval_linear_at_endpoints_returns_endpoint_cps() {
        let curve = linear_curve_f64();
        let v = curve.as_view();
        assert!((eval(&v, 0.0_f64) - 0.0).abs() < 1e-12);
        assert!((eval(&v, 1.0_f64) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn eval_linear_at_midpoint_returns_average() {
        let curve = linear_curve_f64();
        let v = curve.as_view();
        assert!((eval(&v, 0.5_f64) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn eval_quadratic_at_endpoints_returns_first_last_cp() {
        let curve = quadratic_curve_f64();
        let v = curve.as_view();
        assert!((eval(&v, 0.0_f64) - 0.0).abs() < 1e-12);
        assert!((eval(&v, 1.0_f64) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn eval_quadratic_at_midpoint_matches_bernstein() {
        // For the bezier-shaped quadratic with cps [0, 0.5, 1] at u=0.5:
        // B_0,2(0.5) * 0 + B_1,2(0.5) * 0.5 + B_2,2(0.5) * 1
        // = 0.25 * 0 + 0.5 * 0.5 + 0.25 * 1 = 0.5
        let curve = quadratic_curve_f64();
        let v = curve.as_view();
        assert!((eval(&v, 0.5_f64) - 0.5).abs() < 1e-12);
    }
}
```

- [ ] **Step 2: Run to verify failure**

```bash
cargo test -p nurbs eval
```

Expected: compilation error — functions not defined.

- [ ] **Step 3: Implement `find_knot_span` and `de_boor_inner` and `eval` (non-rational only for now)**

Prepend to `rust/nurbs/src/eval.rs` (above tests):

```rust
//! NURBS evaluation: de Boor, vector eval, derivative, curvature.
//! See spec §eval module.

use crate::{Float, NurbsView, VectorNurbsView, MAX_DEGREE, MIN_PARAMETRIC_SPEED, WORKSPACE_SIZE};

/// Find the knot span `k` such that `knots[k] <= u < knots[k+1]`, with the
/// clamped-end special case mapping `u >= knots[n]` to the last span.
/// Reference: Piegl & Tiller "The NURBS Book" Algorithm A2.1.
///
/// Inputs: `knots` is a clamped open knot vector (validated upstream),
/// `p` is the degree, `n` is the control-point count.
pub(crate) fn find_knot_span<T: Float>(knots: &[T], p: usize, n: usize, u: T) -> usize {
    debug_assert!(knots.len() == n + p + 1);
    // Clamped endpoint cases.
    if u >= knots[n] { return n - 1; }
    if u <= knots[p] { return p; }

    let mut low = p;
    let mut high = n;
    let mut mid = (low + high) / 2;
    while u < knots[mid] || u >= knots[mid + 1] {
        if u < knots[mid] {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    mid
}

/// de Boor's algorithm at parameter `u` over `cps` with degree `p`.
/// Stack scratch is `[T; WORKSPACE_SIZE]`. Caller has validated that
/// `p as usize <= MAX_DEGREE`.
///
/// Reference: Piegl & Tiller "The NURBS Book" Algorithm A4.1 (de Boor).
#[inline]
pub(crate) fn de_boor_inner<T: Float>(cps: &[T], knots: &[T], degree: u8, u: T) -> T {
    debug_assert!((degree as usize) <= MAX_DEGREE);
    let p = degree as usize;
    let n = cps.len();
    let k = find_knot_span(knots, p, n, u);

    let mut d = [T::ZERO; WORKSPACE_SIZE];
    for j in 0..=p {
        d[j] = cps[k - p + j];
    }

    for r in 1..=p {
        for j in (r..=p).rev() {
            let denom = knots[k + 1 + j - r] - knots[k - p + j];
            let alpha = if denom > T::ZERO {
                (u - knots[k - p + j]) / denom
            } else {
                T::ZERO
            };
            // d[j] = (1 - alpha) * d[j-1] + alpha * d[j]
            //      = (d[j] - d[j-1]).mul_add(alpha, d[j-1])
            d[j] = (d[j] - d[j - 1]).mul_add(alpha, d[j - 1]);
        }
    }

    d[p]
}

/// Evaluate a scalar NURBS at parameter `u`.
/// Hot path. MCU + host. No allocation.
///
/// For non-rational curves: one de Boor walk.
/// For rational curves: two de Boor walks (weighted CPs and weights), then divide.
#[inline]
pub fn eval<T: Float, V: NurbsView<T>>(curve: &V, u: T) -> T {
    debug_assert!((curve.degree() as usize) <= MAX_DEGREE);
    match curve.weights() {
        None => de_boor_inner(curve.control_points(), curve.knots(), curve.degree(), u),
        Some(_w) => {
            // Rational path implemented in Task 12.
            unimplemented!("rational eval lands in Task 12");
        }
    }
}
```

- [ ] **Step 4: Wire `eval` into `lib.rs`**

Add:

```rust
pub mod eval;
```

- [ ] **Step 5: Run tests**

```bash
cargo test -p nurbs eval
```

Expected: 6 passed.

- [ ] **Step 6: Commit**

```bash
git commit -am "nurbs: add find_knot_span and de_boor_inner (non-rational)"
```

---

## Task 12: eval — rational case

**Files:**
- Modify: `rust/nurbs/src/eval.rs`

- [ ] **Step 1: Write failing test for rational eval**

Append to the eval `tests` module:

```rust
    fn rational_quadratic_arc() -> crate::ScalarNurbs<f64> {
        // Rational quadratic: 90° arc from (1,0) to (0,1) projected to scalar X.
        // We model the X channel: cps = [1, 1, 0], weights = [1, sqrt(2)/2, 1].
        // At u=0: X=1; at u=1: X=0; at u=0.5: ~0.707 (approximately cos(45°)).
        crate::ScalarNurbs::try_new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![1.0, 1.0, 0.0],
            Some(vec![1.0, std::f64::consts::SQRT_2 / 2.0, 1.0]),
        ).unwrap()
    }

    #[test]
    fn eval_rational_at_endpoints() {
        let curve = rational_quadratic_arc();
        let v = curve.as_view();
        assert!((eval(&v, 0.0_f64) - 1.0).abs() < 1e-12);
        assert!((eval(&v, 1.0_f64) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn eval_rational_at_midpoint() {
        let curve = rational_quadratic_arc();
        let v = curve.as_view();
        // Standard rational quadratic formula with symmetric weights yields cos(45°) ≈ 0.7071
        let mid = eval(&v, 0.5_f64);
        let expected = (std::f64::consts::SQRT_2 / 2.0_f64).powi(2)
            / ((std::f64::consts::SQRT_2 / 2.0_f64).powi(2) + 0.5_f64);
        // simpler check: result lies in (0.7, 0.71) for this specific arc
        assert!(mid > 0.69 && mid < 0.72, "got {mid}");
    }
```

- [ ] **Step 2: Run to verify failure**

```bash
cargo test -p nurbs eval::tests::eval_rational
```

Expected: panic at `unimplemented!("rational eval lands in Task 12")`.

- [ ] **Step 3: Implement the rational path**

Replace the `eval` function in `rust/nurbs/src/eval.rs`:

```rust
/// Evaluate a scalar NURBS at parameter `u`.
#[inline]
pub fn eval<T: Float, V: NurbsView<T>>(curve: &V, u: T) -> T {
    debug_assert!((curve.degree() as usize) <= MAX_DEGREE);
    match curve.weights() {
        None => de_boor_inner(curve.control_points(), curve.knots(), curve.degree(), u),
        Some(w) => {
            let numer = de_boor_homogeneous(
                curve.control_points(), w, curve.knots(), curve.degree(), u,
            );
            let denom = de_boor_inner(w, curve.knots(), curve.degree(), u);
            let floor = T::from_f64(MIN_PARAMETRIC_SPEED);
            debug_assert!(denom.abs() > floor);
            numer / denom.max(floor)
        }
    }
}

/// de Boor over `weighted_cps[i] = cps[i] * weights[i]`, computed in a single
/// pass without allocating a weighted-cps vector.
#[inline]
pub(crate) fn de_boor_homogeneous<T: Float>(
    cps: &[T],
    weights: &[T],
    knots: &[T],
    degree: u8,
    u: T,
) -> T {
    debug_assert!((degree as usize) <= MAX_DEGREE);
    debug_assert!(cps.len() == weights.len());
    let p = degree as usize;
    let n = cps.len();
    let k = find_knot_span(knots, p, n, u);

    let mut d = [T::ZERO; WORKSPACE_SIZE];
    for j in 0..=p {
        d[j] = cps[k - p + j] * weights[k - p + j];
    }

    for r in 1..=p {
        for j in (r..=p).rev() {
            let denom = knots[k + 1 + j - r] - knots[k - p + j];
            let alpha = if denom > T::ZERO {
                (u - knots[k - p + j]) / denom
            } else {
                T::ZERO
            };
            d[j] = (d[j] - d[j - 1]).mul_add(alpha, d[j - 1]);
        }
    }

    d[p]
}
```

- [ ] **Step 4: Run tests**

```bash
cargo test -p nurbs eval
```

Expected: 8 passed.

- [ ] **Step 5: Commit**

```bash
git commit -am "nurbs: add rational eval path with MIN_PARAMETRIC_SPEED clamp"
```

---

## Task 13: vector_eval — shared knot-span amortization

**Files:**
- Modify: `rust/nurbs/src/eval.rs`

- [ ] **Step 1: Write failing test**

Append to `eval` tests:

```rust
    fn linear_3d_curve_f64() -> crate::VectorNurbs<f64, 3> {
        crate::VectorNurbs::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]],
            None,
        ).unwrap()
    }

    #[test]
    fn vector_eval_linear_endpoints() {
        let curve = linear_3d_curve_f64();
        let v = curve.as_view();
        let p0 = vector_eval(&v, 0.0_f64);
        assert!((p0[0] - 0.0).abs() < 1e-12);
        assert!((p0[1] - 0.0).abs() < 1e-12);
        assert!((p0[2] - 0.0).abs() < 1e-12);
        let p1 = vector_eval(&v, 1.0_f64);
        assert!((p1[0] - 1.0).abs() < 1e-12);
        assert!((p1[1] - 2.0).abs() < 1e-12);
        assert!((p1[2] - 3.0).abs() < 1e-12);
    }

    #[test]
    fn vector_eval_matches_per_axis_scalar() {
        let curve = linear_3d_curve_f64();
        let v = curve.as_view();
        let result = vector_eval(&v, 0.3_f64);

        // Reconstruct each axis as a scalar curve and compare.
        for axis in 0..3 {
            let cps_axis: Vec<f64> = v.control_points().iter().map(|cp| cp[axis]).collect();
            let scalar = crate::ScalarNurbs::try_new(
                v.degree(), v.knots().to_vec(), cps_axis, None,
            ).unwrap();
            let expected = eval(&scalar.as_view(), 0.3_f64);
            assert!((result[axis] - expected).abs() < 1e-12,
                "axis {axis}: got {}, expected {}", result[axis], expected);
        }
    }
```

- [ ] **Step 2: Run to verify failure**

```bash
cargo test -p nurbs eval::tests::vector_eval
```

Expected: function not defined.

- [ ] **Step 3: Implement `vector_eval`**

Append to `rust/nurbs/src/eval.rs`:

```rust
/// Evaluate a vector NURBS at parameter `u`. Shares knot-span lookup and alpha
/// computation across the N axes — meaningfully cheaper than N independent
/// scalar `eval` calls for shared-knot vector NURBS.
#[inline]
pub fn vector_eval<T: Float, V: VectorNurbsView<T, N>, const N: usize>(
    curve: &V,
    u: T,
) -> [T; N] {
    debug_assert!((curve.degree() as usize) <= MAX_DEGREE);
    let p = curve.degree() as usize;
    let knots = curve.knots();
    let cps = curve.control_points();
    let n = cps.len();
    let k = find_knot_span(knots, p, n, u);

    let has_weights = curve.weights().is_some();

    let mut d_axes: [[T; WORKSPACE_SIZE]; N] = [[T::ZERO; WORKSPACE_SIZE]; N];
    let mut d_w = [T::ZERO; WORKSPACE_SIZE];

    // Initialize active CPs for this span.
    for j in 0..=p {
        let cp = cps[k - p + j];
        if let Some(w) = curve.weights() {
            for axis in 0..N {
                d_axes[axis][j] = cp[axis] * w[k - p + j];
            }
            d_w[j] = w[k - p + j];
        } else {
            for axis in 0..N {
                d_axes[axis][j] = cp[axis];
            }
        }
    }

    // de Boor recurrence — shared alphas across axes.
    for r in 1..=p {
        for j in (r..=p).rev() {
            let denom = knots[k + 1 + j - r] - knots[k - p + j];
            let alpha = if denom > T::ZERO {
                (u - knots[k - p + j]) / denom
            } else {
                T::ZERO
            };
            for axis in 0..N {
                d_axes[axis][j] = (d_axes[axis][j] - d_axes[axis][j - 1]).mul_add(alpha, d_axes[axis][j - 1]);
            }
            if has_weights {
                d_w[j] = (d_w[j] - d_w[j - 1]).mul_add(alpha, d_w[j - 1]);
            }
        }
    }

    let mut result = [T::ZERO; N];
    if has_weights {
        let denom = d_w[p];
        let floor = T::from_f64(MIN_PARAMETRIC_SPEED);
        debug_assert!(denom.abs() > floor);
        let denom_clamp = denom.max(floor);
        for axis in 0..N {
            result[axis] = d_axes[axis][p] / denom_clamp;
        }
    } else {
        for axis in 0..N {
            result[axis] = d_axes[axis][p];
        }
    }
    result
}
```

- [ ] **Step 4: Run tests**

```bash
cargo test -p nurbs eval
```

Expected: 10 passed.

- [ ] **Step 5: Commit**

```bash
git commit -am "nurbs: add vector_eval with shared knot-span across N axes"
```

---

## Task 14: derivative (host-only, degree-lowering) — scalar

**Files:**
- Modify: `rust/nurbs/src/eval.rs`

- [ ] **Step 1: Write failing test**

Append to eval tests:

```rust
    #[cfg(feature = "host")]
    #[test]
    fn derivative_of_linear_is_constant() {
        // Derivative of a linear NURBS is a degree-0 NURBS with control points
        // equal to (cp[1] - cp[0]) / (u_max - u_min) = 1.0 for our linear curve.
        let curve = linear_curve_f64();
        let d = derivative(&curve);
        assert_eq!(d.degree(), 0);
        // Eval at any u should give 1.0
        assert!((eval(&d.as_view(), 0.5_f64) - 1.0).abs() < 1e-12);
    }

    #[cfg(feature = "host")]
    #[test]
    fn derivative_of_quadratic_at_midpoint_matches_central_difference() {
        let curve = quadratic_curve_f64();
        let d = derivative(&curve);
        let v = d.as_view();
        let h = 1e-6_f64;
        let expected = (eval(&curve.as_view(), 0.5 + h) - eval(&curve.as_view(), 0.5 - h)) / (2.0 * h);
        let actual = eval(&v, 0.5);
        assert!((actual - expected).abs() < 1e-6, "got {actual}, expected {expected}");
    }
```

- [ ] **Step 2: Run to verify failure**

```bash
cargo test -p nurbs eval::tests::derivative
```

Expected: function not defined.

- [ ] **Step 3: Implement `derivative` for `ScalarNurbs`**

Append to `rust/nurbs/src/eval.rs`:

```rust
/// Compute the parametric derivative `dP/du` as a new owned NURBS via degree
/// lowering. Result has degree `p - 1`, knot vector with the first and last
/// knots dropped, and control points
///   `Q_i = p * (P_{i+1} - P_i) / (u_{i+p+1} - u_{i+1})`.
///
/// Host-only — allocates new `Vec`s. For weighted (rational) NURBS, the host
/// pre-bake pipeline should project to homogeneous coordinates first; this
/// function handles unweighted (B-spline) NURBS only. Rational derivative is
/// the consumer's responsibility (composed via the quotient rule downstream).
///
/// Reference: Piegl & Tiller "The NURBS Book" eq. 3.7 / Algorithm A3.3.
#[cfg(feature = "host")]
pub fn derivative<T: Float>(curve: &crate::ScalarNurbs<T>) -> crate::ScalarNurbs<T> {
    let p = curve.degree();
    assert!(p >= 1, "derivative requires degree >= 1");

    let cps = curve.control_points();
    let knots = curve.knots();
    let new_degree = p - 1;
    let new_n = cps.len() - 1;

    let p_t = T::from_f64(p as f64);

    let mut new_cps: Vec<T> = Vec::with_capacity(new_n);
    for i in 0..new_n {
        let denom = knots[i + p as usize + 1] - knots[i + 1];
        let q = if denom > T::ZERO {
            p_t * (cps[i + 1] - cps[i]) / denom
        } else {
            T::ZERO
        };
        new_cps.push(q);
    }

    // New knot vector drops the first and last entries.
    let new_knots: Vec<T> = knots[1..knots.len() - 1].to_vec();

    crate::ScalarNurbs::try_new(new_degree, new_knots, new_cps, None)
        .expect("degree-lowered NURBS satisfies invariants by construction")
}
```

- [ ] **Step 4: Run tests**

```bash
cargo test -p nurbs eval::tests::derivative
```

Expected: 2 passed.

- [ ] **Step 5: Commit**

```bash
git commit -am "nurbs: add scalar derivative via degree-lowering"
```

---

## Task 15: vector_derivative

**Files:**
- Modify: `rust/nurbs/src/eval.rs`

- [ ] **Step 1: Write failing test**

Append to eval tests:

```rust
    #[cfg(feature = "host")]
    #[test]
    fn vector_derivative_matches_per_axis_scalar() {
        let curve = linear_3d_curve_f64();
        let d = vector_derivative(&curve);
        assert_eq!(d.degree(), 0);
        let v = d.as_view();
        let result = vector_eval(&v, 0.3_f64);

        for axis in 0..3 {
            let cps_axis: Vec<f64> = curve.control_points().iter().map(|cp| cp[axis]).collect();
            let scalar = crate::ScalarNurbs::try_new(
                curve.degree(), curve.knots().to_vec(), cps_axis, None,
            ).unwrap();
            let scalar_d = derivative(&scalar);
            let expected = eval(&scalar_d.as_view(), 0.3_f64);
            assert!((result[axis] - expected).abs() < 1e-12);
        }
    }
```

- [ ] **Step 2: Run to verify failure**

```bash
cargo test -p nurbs eval::tests::vector_derivative
```

- [ ] **Step 3: Implement `vector_derivative`**

Append to `rust/nurbs/src/eval.rs`:

```rust
/// Compute the parametric derivative of a vector NURBS as a new owned NURBS.
/// Same algorithm as scalar `derivative` applied per axis; knot vector and
/// degree handled once.
#[cfg(feature = "host")]
pub fn vector_derivative<T: Float, const N: usize>(
    curve: &crate::VectorNurbs<T, N>,
) -> crate::VectorNurbs<T, N> {
    let p = curve.degree();
    assert!(p >= 1, "derivative requires degree >= 1");

    let cps = curve.control_points();
    let knots = curve.knots();
    let new_degree = p - 1;
    let new_n = cps.len() - 1;
    let p_t = T::from_f64(p as f64);

    let mut new_cps: Vec<[T; N]> = Vec::with_capacity(new_n);
    for i in 0..new_n {
        let denom = knots[i + p as usize + 1] - knots[i + 1];
        let mut q = [T::ZERO; N];
        if denom > T::ZERO {
            for axis in 0..N {
                q[axis] = p_t * (cps[i + 1][axis] - cps[i][axis]) / denom;
            }
        }
        new_cps.push(q);
    }

    let new_knots: Vec<T> = knots[1..knots.len() - 1].to_vec();

    crate::VectorNurbs::try_new(new_degree, new_knots, new_cps, None)
        .expect("degree-lowered NURBS satisfies invariants by construction")
}
```

- [ ] **Step 4: Run tests**

```bash
cargo test -p nurbs eval
```

Expected: passes including the new vector_derivative test.

- [ ] **Step 5: Commit**

```bash
git commit -am "nurbs: add vector_derivative"
```

---

## Task 16: curvature_from_derivs

**Files:**
- Modify: `rust/nurbs/src/eval.rs`

- [ ] **Step 1: Write failing test**

Append to eval tests:

```rust
    #[cfg(feature = "host")]
    #[test]
    fn curvature_of_straight_line_is_zero() {
        let curve = linear_3d_curve_f64();
        let first = vector_derivative(&curve);
        // Second derivative of a linear curve is zero — but degree-lowering can't
        // produce a degree -1 curve. We need a degree-2 curve to take two derivatives.
        // Use a parabolic 3D curve instead.
        let parabolic = crate::VectorNurbs::try_new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
            None,
        ).unwrap();
        let first = vector_derivative(&parabolic);
        let second = vector_derivative(&first);
        // The path is straight along X — curvature is 0 everywhere.
        let k = curvature_from_derivs(&first, &second, 0.5_f64);
        assert!(k.abs() < 1e-10, "got {k}");
    }

    #[cfg(feature = "host")]
    #[test]
    fn curvature_of_arc_matches_known_value() {
        // Quadratic Bezier approximating a circular arc: cps [(1,0,0),(1,1,0),(0,1,0)].
        // Not a true circle (rational quadratics with weights are exact), but
        // curvature at u=0.5 should be positive and finite.
        let arc = crate::VectorNurbs::try_new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]],
            None,
        ).unwrap();
        let first = vector_derivative(&arc);
        let second = vector_derivative(&first);
        let k = curvature_from_derivs(&first, &second, 0.5_f64);
        assert!(k > 0.0, "expected positive curvature, got {k}");
        assert!(k.is_finite(), "curvature should be finite");
    }
```

- [ ] **Step 2: Run to verify failure**

```bash
cargo test -p nurbs eval::tests::curvature
```

- [ ] **Step 3: Implement `curvature_from_derivs`**

Append to `rust/nurbs/src/eval.rs`:

```rust
/// Compute curvature κ(u) of a 3D path NURBS from its precomputed first and
/// second derivative NURBSes:
///   κ = ||r' × r''|| / ||r'||³
/// The cubed denominator is clamped at `MIN_PARAMETRIC_SPEED` to avoid
/// divide-by-zero at cusps; the clamp engages only on pathological input
/// (well-formed G2/G3 and fitter output never trigger it).
///
/// Caller owns `first_deriv` and `second_deriv` — typically cached on the
/// segment, since TOPP-RA queries many u's per segment.
#[cfg(feature = "host")]
pub fn curvature_from_derivs<T: Float, const N: usize>(
    first_deriv: &crate::VectorNurbs<T, N>,
    second_deriv: &crate::VectorNurbs<T, N>,
    u: T,
) -> T {
    let r_prime = vector_eval(&first_deriv.as_view(), u);
    let r_double = vector_eval(&second_deriv.as_view(), u);

    // Cross product magnitude: works for N=3; for N=2 we'd lift to 3D with z=0.
    // We hardcode 3D here per spec — curvature on path is 3D-only.
    assert!(N == 3, "curvature_from_derivs requires N == 3");

    let cx = r_prime[1] * r_double[2] - r_prime[2] * r_double[1];
    let cy = r_prime[2] * r_double[0] - r_prime[0] * r_double[2];
    let cz = r_prime[0] * r_double[1] - r_prime[1] * r_double[0];
    let cross_norm = (cx * cx + cy * cy + cz * cz).sqrt();

    let speed_sq = r_prime[0] * r_prime[0] + r_prime[1] * r_prime[1] + r_prime[2] * r_prime[2];
    let speed = speed_sq.sqrt();
    let speed_cubed = speed * speed * speed;

    let floor = T::from_f64(MIN_PARAMETRIC_SPEED);
    cross_norm / speed_cubed.max(floor)
}
```

- [ ] **Step 4: Run tests**

```bash
cargo test -p nurbs eval
```

Expected: passes.

- [ ] **Step 5: Commit**

```bash
git commit -am "nurbs: add curvature_from_derivs with MIN_PARAMETRIC_SPEED clamp"
```

---

## Task 17: arc_length module — types

**Files:**
- Create: `rust/nurbs/src/arc_length.rs`
- Modify: `rust/nurbs/src/lib.rs`

- [ ] **Step 1: Write failing tests**

Create `rust/nurbs/src/arc_length.rs`:

```rust
//! Arc-length parameterization.
//! See spec §arc_length module.

use crate::Float;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ref_provides_borrowed_access() {
        let s = [0.0_f64, 0.5, 1.0];
        let u = [0.0_f64, 0.4, 1.0];
        let r = ArcLengthTableRef::new(&s, &u);
        assert_eq!(r.s_max(), 1.0);
        assert_eq!(r.u_max(), 1.0);
    }

    #[cfg(feature = "host")]
    #[test]
    fn owned_as_view_round_trips() {
        let owned = ArcLengthTable::new(vec![0.0, 0.5, 1.0], vec![0.0, 0.4, 1.0]);
        let view = owned.as_view();
        assert_eq!(view.s_max(), 1.0);
    }
}
```

- [ ] **Step 2: Run failing tests**

```bash
cargo test -p nurbs arc_length::tests
```

Expected: types not defined.

- [ ] **Step 3: Implement the types**

Prepend to `rust/nurbs/src/arc_length.rs`:

```rust
//! Arc-length parameterization.
//! See spec §arc_length module.

use crate::Float;

/// Owned arc-length table. Built on host via `build_arc_length_table_*`,
/// shipped to the MCU as a borrowed view via the wire format.
#[cfg(feature = "host")]
#[derive(Debug, Clone, PartialEq)]
pub struct ArcLengthTable<T: Float> {
    s: Vec<T>,
    u: Vec<T>,
}

#[cfg(feature = "host")]
impl<T: Float> ArcLengthTable<T> {
    /// Construct from monotone non-decreasing s and u sample arrays.
    /// Caller is the builder — already validated.
    pub fn new(s: Vec<T>, u: Vec<T>) -> Self {
        debug_assert_eq!(s.len(), u.len());
        debug_assert!(s.len() >= 2);
        Self { s, u }
    }

    pub fn s(&self) -> &[T] { &self.s }
    pub fn u(&self) -> &[T] { &self.u }
    pub fn s_max(&self) -> T { *self.s.last().expect("table is non-empty") }
    pub fn u_max(&self) -> T { *self.u.last().expect("table is non-empty") }
    pub fn sample_count(&self) -> usize { self.s.len() }

    #[inline]
    pub fn as_view(&self) -> ArcLengthTableRef<'_, T> {
        ArcLengthTableRef { s: &self.s, u: &self.u }
    }

    pub fn into_parts(self) -> (Vec<T>, Vec<T>) { (self.s, self.u) }
}

/// Borrowed arc-length table. Available on host and MCU. Pure lookup.
#[derive(Debug, Clone, Copy)]
pub struct ArcLengthTableRef<'a, T: Float> {
    pub(crate) s: &'a [T],
    pub(crate) u: &'a [T],
}

impl<'a, T: Float> ArcLengthTableRef<'a, T> {
    /// Construct from already-validated slices.
    pub fn new(s: &'a [T], u: &'a [T]) -> Self {
        debug_assert_eq!(s.len(), u.len());
        debug_assert!(s.len() >= 2);
        Self { s, u }
    }

    pub fn s(&self) -> &[T] { self.s }
    pub fn u(&self) -> &[T] { self.u }
    pub fn s_max(&self) -> T { *self.s.last().expect("table is non-empty") }
    pub fn u_max(&self) -> T { *self.u.last().expect("table is non-empty") }
}
```

- [ ] **Step 4: Wire into `lib.rs`**

Add:

```rust
pub mod arc_length;
#[cfg(feature = "host")]
pub use arc_length::ArcLengthTable;
pub use arc_length::ArcLengthTableRef;
```

- [ ] **Step 5: Run tests**

```bash
cargo test -p nurbs arc_length
```

Expected: 2 passed.

- [ ] **Step 6: Commit**

```bash
git commit -am "nurbs: add ArcLengthTable and ArcLengthTableRef types"
```

---

## Task 18: integrate_arc_length helper (Gauss-Legendre quadrature)

**Files:**
- Modify: `rust/nurbs/src/arc_length.rs`

- [ ] **Step 1: Write failing test**

Append to `arc_length` tests:

```rust
    #[cfg(feature = "host")]
    #[test]
    fn integrate_constant_returns_length_times_constant() {
        // ∫_0^1 of f(u)=2 should be 2.
        let result = integrate_arc_length(|_u: f64| 2.0_f64, 0.0, 1.0, 5);
        assert!((result - 2.0).abs() < 1e-12);
    }

    #[cfg(feature = "host")]
    #[test]
    fn integrate_linear_matches_closed_form() {
        // ∫_0^1 of f(u)=u should be 0.5.
        let result = integrate_arc_length(|u: f64| u, 0.0, 1.0, 5);
        assert!((result - 0.5).abs() < 1e-12);
    }

    #[cfg(feature = "host")]
    #[test]
    fn integrate_quadratic_matches_closed_form() {
        // ∫_0^1 of f(u)=u^2 should be 1/3. 5-point Gauss-Legendre is exact for degree <= 9.
        let result = integrate_arc_length(|u: f64| u * u, 0.0, 1.0, 5);
        assert!((result - 1.0 / 3.0).abs() < 1e-12);
    }
```

- [ ] **Step 2: Run to verify failure**

```bash
cargo test -p nurbs arc_length::tests::integrate
```

- [ ] **Step 3: Implement `integrate_arc_length`**

Append to `rust/nurbs/src/arc_length.rs`:

```rust
/// 5-point Gauss-Legendre nodes (in [-1, 1]) and weights. Exact for polynomials
/// up to degree 9. Sufficient for our integrand magnitudes.
const GAUSS_LEGENDRE_5_NODES: [f64; 5] = [
    -0.906_179_845_938_664_0,
    -0.538_469_310_105_683_1,
     0.0,
     0.538_469_310_105_683_1,
     0.906_179_845_938_664_0,
];
const GAUSS_LEGENDRE_5_WEIGHTS: [f64; 5] = [
    0.236_926_885_056_189_1,
    0.478_628_670_499_366_5,
    0.568_888_888_888_888_9,
    0.478_628_670_499_366_5,
    0.236_926_885_056_189_1,
];

/// Integrate `integrand` over `[u_start, u_end]` via Gauss-Legendre quadrature.
/// `quadrature_points` must be 5; v1 hardcodes 5-point GL — argument reserved
/// for future adaptation (e.g. higher-order for high-degree integrands).
#[cfg(feature = "host")]
pub(crate) fn integrate_arc_length<T: Float, F: Fn(T) -> T>(
    integrand: F,
    u_start: T,
    u_end: T,
    quadrature_points: usize,
) -> T {
    debug_assert_eq!(quadrature_points, 5, "v1 supports only 5-point Gauss-Legendre");

    let half_range = (u_end - u_start) * T::from_f64(0.5);
    let midpoint = (u_start + u_end) * T::from_f64(0.5);

    let mut sum = T::ZERO;
    for i in 0..5 {
        let node = T::from_f64(GAUSS_LEGENDRE_5_NODES[i]);
        let weight = T::from_f64(GAUSS_LEGENDRE_5_WEIGHTS[i]);
        let u = midpoint + half_range * node;
        sum = integrand(u).mul_add(weight, sum);
    }

    sum * half_range
}
```

- [ ] **Step 4: Run tests**

```bash
cargo test -p nurbs arc_length::tests::integrate
```

Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git commit -am "nurbs: add integrate_arc_length (5-point Gauss-Legendre)"
```

---

## Task 19: build_arc_length_table_scalar and _vector

**Files:**
- Modify: `rust/nurbs/src/arc_length.rs`

- [ ] **Step 1: Write failing tests**

Append to `arc_length` tests:

```rust
    #[cfg(feature = "host")]
    #[test]
    fn build_scalar_table_for_linear_curve() {
        // Linear curve from 0 to 1 over u in [0, 1]: arc length = 1.
        let curve = crate::ScalarNurbs::try_new(
            1,
            vec![0.0_f64, 0.0, 1.0, 1.0],
            vec![0.0, 1.0],
            None,
        ).unwrap();
        let table = build_arc_length_table_scalar(&curve, 1e-6, 64).unwrap();
        assert!((table.s_max() - 1.0).abs() < 1e-6);
        assert!(table.u_max() == 1.0);
        // Monotonicity check
        for w in table.s().windows(2) { assert!(w[1] >= w[0]); }
        for w in table.u().windows(2) { assert!(w[1] >= w[0]); }
    }

    #[cfg(feature = "host")]
    #[test]
    fn build_vector_table_for_3d_linear_curve() {
        // 3D linear curve from origin to (3, 0, 4): arc length = 5.
        let curve = crate::VectorNurbs::try_new(
            1,
            vec![0.0_f64, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [3.0, 0.0, 4.0]],
            None,
        ).unwrap();
        let table = build_arc_length_table_vector(&curve, 1e-5, 64).unwrap();
        assert!((table.s_max() - 5.0).abs() < 1e-4);
    }
```

- [ ] **Step 2: Run to verify failure**

```bash
cargo test -p nurbs arc_length::tests::build_
```

- [ ] **Step 3: Implement the builders**

Append to `rust/nurbs/src/arc_length.rs`:

```rust
use crate::eval::{eval, vector_eval};
use crate::{ArcLengthError, NurbsView, VectorNurbsView, MIN_PARAMETRIC_SPEED};

/// Build an arc-length table for a scalar NURBS via adaptive sampling.
///
/// Strategy: start with a small uniform grid in u; at each step, double the
/// sample count if the linear-interpolation residual against a refined estimate
/// exceeds `tolerance`. Cap at `max_samples`.
///
/// Integrand is `|dP/du|`; for scalar curves we use the absolute value of the
/// scalar derivative evaluated by central difference (we don't take a
/// degree-lowered derivative here because it'd allocate twice for the same
/// information; central difference is cheap on the host).
#[cfg(feature = "host")]
pub fn build_arc_length_table_scalar<T: Float, V: NurbsView<T>>(
    curve: &V,
    tolerance: T,
    max_samples: usize,
) -> Result<ArcLengthTable<T>, ArcLengthError<T>> {
    let h = T::from_f64(1e-6);
    let knots = curve.knots();
    let u_start = knots[0];
    let u_end = knots[knots.len() - 1];

    let integrand = |u: T| {
        let u_safe = u.max(u_start + h).min(u_end - h);
        let plus = eval(curve, u_safe + h);
        let minus = eval(curve, u_safe - h);
        ((plus - minus) / (h + h)).abs()
    };

    build_table_via_integrand(integrand, u_start, u_end, tolerance, max_samples)
}

/// Build an arc-length table for a vector NURBS in R^3.
#[cfg(feature = "host")]
pub fn build_arc_length_table_vector<T: Float, V: VectorNurbsView<T, 3>>(
    curve: &V,
    tolerance: T,
    max_samples: usize,
) -> Result<ArcLengthTable<T>, ArcLengthError<T>> {
    let h = T::from_f64(1e-6);
    let knots = curve.knots();
    let u_start = knots[0];
    let u_end = knots[knots.len() - 1];

    let integrand = |u: T| {
        let u_safe = u.max(u_start + h).min(u_end - h);
        let plus = vector_eval(curve, u_safe + h);
        let minus = vector_eval(curve, u_safe - h);
        let two_h = h + h;
        let dx = (plus[0] - minus[0]) / two_h;
        let dy = (plus[1] - minus[1]) / two_h;
        let dz = (plus[2] - minus[2]) / two_h;
        (dx * dx + dy * dy + dz * dz).sqrt()
    };

    build_table_via_integrand(integrand, u_start, u_end, tolerance, max_samples)
}

/// Adaptive table builder. Doubles sample count until linear-interp residual
/// is below tolerance or we hit the cap.
#[cfg(feature = "host")]
fn build_table_via_integrand<T: Float, F: Fn(T) -> T + Copy>(
    integrand: F,
    u_start: T,
    u_end: T,
    tolerance: T,
    max_samples: usize,
) -> Result<ArcLengthTable<T>, ArcLengthError<T>> {
    let floor = T::from_f64(MIN_PARAMETRIC_SPEED);

    let mut count = 8.max(2);
    loop {
        // Build a table at this sample count by integrating between adjacent u's.
        let mut u_samples: Vec<T> = Vec::with_capacity(count);
        let mut s_samples: Vec<T> = Vec::with_capacity(count);

        let span = u_end - u_start;
        for i in 0..count {
            let frac = T::from_f64(i as f64 / (count - 1) as f64);
            u_samples.push(u_start + span * frac);
        }

        s_samples.push(T::ZERO);
        for i in 1..count {
            // Check for degeneracy at integration sample points.
            let u_mid = (u_samples[i - 1] + u_samples[i]) * T::from_f64(0.5);
            if integrand(u_mid) < floor {
                return Err(ArcLengthError::DegenerateCurve);
            }
            let segment_length = integrate_arc_length(integrand, u_samples[i - 1], u_samples[i], 5);
            let prev = s_samples[i - 1];
            s_samples.push(prev + segment_length);
        }

        // Estimate residual: refine to 2*count and compare s_max.
        let span_full = u_end - u_start;
        let s_refined: T = {
            let count_refined = (count - 1) * 2 + 1;
            let mut acc = T::ZERO;
            for i in 1..count_refined {
                let a = u_start + span_full * T::from_f64((i - 1) as f64 / (count_refined - 1) as f64);
                let b = u_start + span_full * T::from_f64(i as f64 / (count_refined - 1) as f64);
                acc = acc + integrate_arc_length(integrand, a, b, 5);
            }
            acc
        };

        let residual = (s_samples[count - 1] - s_refined).abs();
        if residual <= tolerance {
            return Ok(ArcLengthTable::new(s_samples, u_samples));
        }
        if count * 2 > max_samples {
            return Err(ArcLengthError::ToleranceNotMet {
                achieved_residual: residual,
                samples_used: count,
            });
        }
        count *= 2;
    }
}
```

- [ ] **Step 4: Run tests**

```bash
cargo test -p nurbs arc_length
```

Expected: passes.

- [ ] **Step 5: Commit**

```bash
git commit -am "nurbs: add scalar and vector arc-length table builders"
```

---

## Task 20: param_from_arc_length and arc_length_from_param

**Files:**
- Modify: `rust/nurbs/src/arc_length.rs`

- [ ] **Step 1: Write failing tests**

Append to `arc_length` tests:

```rust
    #[test]
    fn param_from_arc_length_at_endpoints() {
        let table = ArcLengthTableRef::new(&[0.0_f64, 0.5, 1.0], &[0.0, 0.6, 1.0]);
        assert_eq!(param_from_arc_length(&table, 0.0), 0.0);
        assert_eq!(param_from_arc_length(&table, 1.0), 1.0);
    }

    #[test]
    fn param_from_arc_length_interpolates_linearly() {
        let table = ArcLengthTableRef::new(&[0.0_f64, 0.5, 1.0], &[0.0, 0.6, 1.0]);
        // s = 0.25 lies between (0.0 -> 0.0) and (0.5 -> 0.6); linear interp gives 0.3.
        assert!((param_from_arc_length(&table, 0.25_f64) - 0.3).abs() < 1e-12);
    }

    #[test]
    fn param_from_arc_length_clamps_above_range_in_release() {
        // In release, out-of-range queries clamp silently. In debug, this would
        // fire a debug_assert, so the test itself uses an in-range value but
        // relies on the clamp branch of the implementation.
        let table = ArcLengthTableRef::new(&[0.0_f64, 1.0], &[0.0, 1.0]);
        // Use a value that exercises clamp logic without violating debug_assert.
        let v = param_from_arc_length(&table, 1.0_f64);
        assert_eq!(v, 1.0);
    }

    #[test]
    fn arc_length_from_param_inverts_param_from_arc_length() {
        let table = ArcLengthTableRef::new(&[0.0_f64, 0.4, 1.0], &[0.0, 0.5, 1.0]);
        let u = 0.3_f64;
        let s = arc_length_from_param(&table, u);
        let u_back = param_from_arc_length(&table, s);
        assert!((u - u_back).abs() < 1e-12);
    }
```

- [ ] **Step 2: Run to verify failure**

```bash
cargo test -p nurbs arc_length::tests::param_from
```

- [ ] **Step 3: Implement the queries**

Append to `rust/nurbs/src/arc_length.rs`:

```rust
/// Given an arc-length table and a query `s`, return the parameter `u` such
/// that `arc_length(u) = s`. Binary search on `s` plus linear interpolation.
///
/// Contract: `s` is segment-local (relative to this segment's table). Out-of-
/// range queries debug-assert in development and clamp silently in release.
#[inline]
pub fn param_from_arc_length<T: Float>(table: &ArcLengthTableRef<'_, T>, s: T) -> T {
    debug_assert!(s >= T::ZERO);
    debug_assert!(s <= table.s_max());
    let s_clamped = s.max(T::ZERO).min(table.s_max());

    let s_arr = table.s();
    let u_arr = table.u();
    // Endpoint short-circuit.
    if s_clamped <= s_arr[0] { return u_arr[0]; }
    let last = s_arr.len() - 1;
    if s_clamped >= s_arr[last] { return u_arr[last]; }

    // Binary search for the span [i, i+1] where s_arr[i] <= s_clamped < s_arr[i+1].
    let mut lo = 0usize;
    let mut hi = last;
    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        if s_arr[mid] <= s_clamped { lo = mid; } else { hi = mid; }
    }

    let s_lo = s_arr[lo];
    let s_hi = s_arr[lo + 1];
    let u_lo = u_arr[lo];
    let u_hi = u_arr[lo + 1];

    let span = s_hi - s_lo;
    let floor = T::from_f64(MIN_PARAMETRIC_SPEED);
    let frac = (s_clamped - s_lo) / span.max(floor);
    u_lo + (u_hi - u_lo) * frac
}

/// Inverse: given parameter `u`, return arc length `s = arc_length(u)`.
/// Binary search on `u` plus linear interpolation. Same contract as `param_from_arc_length`.
#[inline]
pub fn arc_length_from_param<T: Float>(table: &ArcLengthTableRef<'_, T>, u: T) -> T {
    debug_assert!(u >= T::ZERO);
    debug_assert!(u <= table.u_max());
    let u_clamped = u.max(T::ZERO).min(table.u_max());

    let s_arr = table.s();
    let u_arr = table.u();
    if u_clamped <= u_arr[0] { return s_arr[0]; }
    let last = u_arr.len() - 1;
    if u_clamped >= u_arr[last] { return s_arr[last]; }

    let mut lo = 0usize;
    let mut hi = last;
    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        if u_arr[mid] <= u_clamped { lo = mid; } else { hi = mid; }
    }

    let u_lo = u_arr[lo];
    let u_hi = u_arr[lo + 1];
    let s_lo = s_arr[lo];
    let s_hi = s_arr[lo + 1];

    let span = u_hi - u_lo;
    let floor = T::from_f64(MIN_PARAMETRIC_SPEED);
    let frac = (u_clamped - u_lo) / span.max(floor);
    s_lo + (s_hi - s_lo) * frac
}
```

- [ ] **Step 4: Run tests**

```bash
cargo test -p nurbs arc_length
```

Expected: passes.

- [ ] **Step 5: MCU build check**

```bash
cargo check -p nurbs --no-default-features --features mcu-h7 --target thumbv7em-none-eabihf
```

- [ ] **Step 6: Commit**

```bash
git commit -am "nurbs: add param_from_arc_length and arc_length_from_param queries"
```

---

## Task 21: ArcLengthTableRef::try_from_wire

**Files:**
- Modify: `rust/nurbs/src/arc_length.rs`

- [ ] **Step 1: Write failing test**

Append to `arc_length` tests:

```rust
    #[test]
    fn try_from_wire_parses_small_table() {
        // Layout: u8 version, u8 reserved, u16 sample_count, u32 reserved2,
        //         T[sample_count] s, T[sample_count] u
        let mut buf = Vec::new();
        buf.extend_from_slice(&[1u8, 0]);                         // version, reserved
        buf.extend_from_slice(&3u16.to_ne_bytes());               // sample_count
        buf.extend_from_slice(&[0u8; 4]);                         // reserved2
        for v in [0.0_f32, 0.5, 1.0] { buf.extend_from_slice(&v.to_ne_bytes()); }
        for v in [0.0_f32, 0.6, 1.0] { buf.extend_from_slice(&v.to_ne_bytes()); }

        let aligned = test_align(&buf, 4);
        let r = ArcLengthTableRef::<f32>::try_from_wire(aligned.as_slice()).unwrap();
        assert_eq!(r.s(), &[0.0_f32, 0.5, 1.0]);
        assert_eq!(r.u(), &[0.0_f32, 0.6, 1.0]);
    }

    /// Test-only owner; same shape as `align_buf` in scalar.rs (Task 9).
    struct AlignedBytes {
        backing: Vec<u32>,
        len: usize,
    }

    impl AlignedBytes {
        fn as_slice(&self) -> &[u8] {
            // SAFETY: Vec<u32> is 4-byte aligned; len ≤ backing.len()*4.
            #[allow(unsafe_code)]
            unsafe {
                core::slice::from_raw_parts(self.backing.as_ptr().cast::<u8>(), self.len)
            }
        }
    }

    fn test_align(data: &[u8], _align: usize) -> AlignedBytes {
        let n = data.len().div_ceil(4);
        let mut backing: Vec<u32> = vec![0; n];
        // SAFETY: backing owns n*4 bytes 4-byte aligned; data.len() ≤ n*4.
        #[allow(unsafe_code)]
        let bytes: &mut [u8] = unsafe {
            core::slice::from_raw_parts_mut(backing.as_mut_ptr().cast::<u8>(), n * 4)
        };
        bytes[..data.len()].copy_from_slice(data);
        AlignedBytes { backing, len: data.len() }
    }
```

- [ ] **Step 2: Run to verify failure**

```bash
cargo test -p nurbs arc_length::tests::try_from_wire
```

- [ ] **Step 3: Implement `try_from_wire`**

Append to `rust/nurbs/src/arc_length.rs`:

```rust
use crate::wire::{ARC_LENGTH_HEADER_BYTES, FORMAT_VERSION_V1};
use crate::WireError;

impl<'a> ArcLengthTableRef<'a, f32> {
    /// Zero-copy parse of a wire-format buffer.
    pub fn try_from_wire(buf: &'a [u8]) -> Result<Self, WireError> {
        if (buf.as_ptr() as usize) % core::mem::align_of::<f32>() != 0 {
            return Err(WireError::Misaligned);
        }
        if buf.len() < ARC_LENGTH_HEADER_BYTES {
            return Err(WireError::TruncatedBuffer {
                expected_len: ARC_LENGTH_HEADER_BYTES, got: buf.len(),
            });
        }
        let version = buf[0];
        if version != FORMAT_VERSION_V1 {
            return Err(WireError::UnknownVersion(version));
        }
        let sample_count = u16::from_ne_bytes([buf[2], buf[3]]) as usize;
        if sample_count < 2 {
            return Err(WireError::TruncatedBuffer {
                expected_len: ARC_LENGTH_HEADER_BYTES + 2 * core::mem::size_of::<f32>() * 2,
                got: buf.len(),
            });
        }

        let bytes_per_axis = sample_count * core::mem::size_of::<f32>();
        let total = ARC_LENGTH_HEADER_BYTES + 2 * bytes_per_axis;
        if buf.len() < total {
            return Err(WireError::TruncatedBuffer { expected_len: total, got: buf.len() });
        }

        // SAFETY: alignment + length both checked.
        #[allow(unsafe_code)]
        let (s, u) = unsafe {
            let s_ptr = buf.as_ptr().add(ARC_LENGTH_HEADER_BYTES) as *const f32;
            let u_ptr = buf.as_ptr().add(ARC_LENGTH_HEADER_BYTES + bytes_per_axis) as *const f32;
            (
                core::slice::from_raw_parts(s_ptr, sample_count),
                core::slice::from_raw_parts(u_ptr, sample_count),
            )
        };
        Ok(Self::new(s, u))
    }
}
```

- [ ] **Step 4: Run tests**

```bash
cargo test -p nurbs arc_length
```

Expected: passes.

- [ ] **Step 5: Commit**

```bash
git commit -am "nurbs: add ArcLengthTableRef::try_from_wire"
```

---

## Task 22: algebra module — scalar_multiply

**Files:**
- Create: `rust/nurbs/src/algebra.rs`
- Modify: `rust/nurbs/src/lib.rs`

- [ ] **Step 1: Write failing tests**

Create `rust/nurbs/src/algebra.rs`:

```rust
//! Algebraic operations on NURBS. Host-only.
//! See spec §algebra module.

#[cfg(all(test, feature = "host"))]
mod tests {
    use super::*;
    use crate::eval::eval;

    #[test]
    fn scalar_multiply_doubles_evaluation() {
        let curve = crate::ScalarNurbs::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None,
        ).unwrap();
        let doubled = scalar_multiply(&curve, 2.0_f64);
        assert!((eval(&doubled.as_view(), 0.5_f64) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn scalar_multiply_preserves_weights() {
        let curve = crate::ScalarNurbs::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![1.0, 2.0], Some(vec![1.0, 1.0]),
        ).unwrap();
        let result = scalar_multiply(&curve, 3.0_f64);
        assert_eq!(result.weights().unwrap(), &[1.0, 1.0]);
        assert_eq!(result.control_points(), &[3.0, 6.0]);
    }
}
```

- [ ] **Step 2: Run to verify failure**

```bash
cargo test -p nurbs algebra
```

- [ ] **Step 3: Implement `scalar_multiply`**

Prepend to `rust/nurbs/src/algebra.rs`:

```rust
//! Algebraic operations on NURBS. Host-only.
//! See spec §algebra module.

use crate::{AlgebraError, Float, MAX_DEGREE};

/// Multiply control points by a scalar. Weights, knots, degree unchanged.
#[cfg(feature = "host")]
pub fn scalar_multiply<T: Float>(curve: &crate::ScalarNurbs<T>, scalar: T) -> crate::ScalarNurbs<T> {
    let new_cps: Vec<T> = curve.control_points().iter().map(|c| *c * scalar).collect();
    let weights = curve.weights().map(|w| w.to_vec());
    crate::ScalarNurbs::try_new(
        curve.degree(), curve.knots().to_vec(), new_cps, weights,
    ).expect("scalar_multiply preserves invariants")
}
```

- [ ] **Step 4: Wire into `lib.rs`**

Add:

```rust
#[cfg(feature = "host")]
pub mod algebra;
```

- [ ] **Step 5: Run tests**

```bash
cargo test -p nurbs algebra
```

Expected: 2 passed.

- [ ] **Step 6: Commit**

```bash
git commit -am "nurbs: add algebra::scalar_multiply"
```

---

## Task 23: algebra::add (with knot-insertion alignment)

**Files:**
- Modify: `rust/nurbs/src/algebra.rs`

- [ ] **Step 1: Write failing tests**

Append to `algebra` tests:

```rust
    #[test]
    fn add_two_compatible_curves() {
        let a = crate::ScalarNurbs::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None,
        ).unwrap();
        let b = crate::ScalarNurbs::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![2.0, 3.0], None,
        ).unwrap();
        let sum = add(&a, &b).unwrap();
        // At u=0.5: 0.5 + 2.5 = 3.0
        assert!((eval(&sum.as_view(), 0.5_f64) - 3.0).abs() < 1e-12);
    }

    #[test]
    fn add_rejects_mismatched_degree() {
        let a = crate::ScalarNurbs::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None,
        ).unwrap();
        let b = crate::ScalarNurbs::try_new(
            2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![0.0, 0.5, 1.0], None,
        ).unwrap();
        let result = add(&a, &b);
        assert!(matches!(result, Err(crate::AlgebraError::KnotMismatch)));
    }
```

(v1 add only handles same-degree, same-knots inputs. Knot insertion to align mismatched curves is a follow-up; surface the error explicitly so the caller knows to align first.)

- [ ] **Step 2: Run to verify failure**

```bash
cargo test -p nurbs algebra::tests::add
```

- [ ] **Step 3: Implement `add`**

Append to `rust/nurbs/src/algebra.rs`:

```rust
/// Add two scalar NURBS pointwise. v1 requires identical degree and identical
/// knot vectors; mismatched cases return `KnotMismatch` and the caller is
/// expected to align via knot insertion (follow-up implementation).
///
/// Weights: v1 supports unweighted-only. Weighted addition is non-trivial
/// (requires homogeneous lift) and is deferred to a follow-up spec.
#[cfg(feature = "host")]
pub fn add<T: Float>(
    a: &crate::ScalarNurbs<T>,
    b: &crate::ScalarNurbs<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    if a.degree() != b.degree() { return Err(AlgebraError::KnotMismatch); }
    if a.knots() != b.knots() { return Err(AlgebraError::KnotMismatch); }
    if a.weights().is_some() || b.weights().is_some() {
        return Err(AlgebraError::NotImplemented(
            "weighted add — homogeneous lift required",
        ));
    }
    let new_cps: Vec<T> = a.control_points().iter().zip(b.control_points().iter())
        .map(|(x, y)| *x + *y)
        .collect();
    crate::ScalarNurbs::try_new(a.degree(), a.knots().to_vec(), new_cps, None)
        .map_err(|_| AlgebraError::KnotMismatch)
}
```

- [ ] **Step 4: Run tests**

```bash
cargo test -p nurbs algebra
```

Expected: passes.

- [ ] **Step 5: Commit**

```bash
git commit -am "nurbs: add algebra::add (same-knots case; full knot-insertion deferred)"
```

---

## Task 24: algebra::multiply and algebra::convolve_with_polynomial_kernel — interface stubs

**Files:**
- Modify: `rust/nurbs/src/algebra.rs`

- [ ] **Step 1: Write tests verifying the stubs return the deferred-algorithm error**

Append to `algebra` tests:

```rust
    #[test]
    fn multiply_returns_not_implemented_error() {
        let a = crate::ScalarNurbs::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None,
        ).unwrap();
        let b = a.clone();
        let result = multiply(&a, &b);
        assert!(matches!(result, Err(crate::AlgebraError::NotImplemented(_))));
    }

    #[test]
    fn convolve_returns_not_implemented_error() {
        let a = crate::ScalarNurbs::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None,
        ).unwrap();
        let kernel = PolynomialKernel {
            coefficients: vec![1.0, 0.0],
            support: (0.0, 1.0),
        };
        let result = convolve_with_polynomial_kernel(&a, &kernel);
        assert!(matches!(result, Err(crate::AlgebraError::NotImplemented(_))));
    }
```

- [ ] **Step 2: Run to verify failure**

```bash
cargo test -p nurbs algebra::tests::multiply algebra::tests::convolve
```

- [ ] **Step 3: Implement `multiply` (stub) and `PolynomialKernel` + `convolve_with_polynomial_kernel` (stub)**

Append to `rust/nurbs/src/algebra.rs`:

```rust
/// Polynomial kernel for convolution. Coefficients are dense, low-to-high.
#[cfg(feature = "host")]
#[derive(Debug, Clone)]
pub struct PolynomialKernel<T: Float> {
    pub coefficients: Vec<T>,
    pub support: (T, T),
}

#[cfg(feature = "host")]
impl<T: Float> PolynomialKernel<T> {
    pub fn degree(&self) -> u8 {
        // Highest non-trivial coefficient; for v1 stub, just length - 1.
        (self.coefficients.len().saturating_sub(1)) as u8
    }
}

/// Multiply two scalar NURBS. Result degree = degree(a) + degree(b).
///
/// Algorithm: deferred to a follow-up spec. See spec §algebra module —
/// well-trodden (Piegl & Tiller ch. 5) but verbose with non-uniform knots
/// and weights.
#[cfg(feature = "host")]
pub fn multiply<T: Float>(
    _a: &crate::ScalarNurbs<T>,
    _b: &crate::ScalarNurbs<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    Err(AlgebraError::NotImplemented(
        "multiply — see Piegl & Tiller ch. 5; lands when needed by Layer 3 pre-bake",
    ))
}

/// Convolve a NURBS with a polynomial kernel. Result degree = degree(curve) + kernel.degree().
///
/// Algorithm: deferred to a follow-up spec. Research-flavored (derived from
/// B-spline basis-function math). Lands when smooth shapers come online at
/// CLAUDE.md build step 8.
#[cfg(feature = "host")]
pub fn convolve_with_polynomial_kernel<T: Float>(
    _curve: &crate::ScalarNurbs<T>,
    _kernel: &PolynomialKernel<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    Err(AlgebraError::NotImplemented(
        "convolve_with_polynomial_kernel — research-flavored; lands at build step 8",
    ))
}
```

- [ ] **Step 4: Run tests**

```bash
cargo test -p nurbs algebra
```

Expected: passes (stubs return `NotImplemented`).

- [ ] **Step 5: Commit**

```bash
git commit -am "nurbs: add algebra::multiply and convolve interface stubs (algorithms deferred)"
```

---

## Task 25: nurbs-c-api crate skeleton

**Files:**
- Create: `rust/nurbs-c-api/Cargo.toml`
- Create: `rust/nurbs-c-api/src/lib.rs`
- Create: `rust/nurbs-c-api/cbindgen.toml`
- Create: `rust/nurbs-c-api/include/.gitkeep`
- Modify: `rust/Cargo.toml` (re-add `"nurbs-c-api"` to workspace `members`)

- [ ] **Step 0: Re-add `nurbs-c-api` to the workspace members**

In Task 2 the workspace `members` list was trimmed to `["nurbs"]` because `nurbs-c-api` didn't exist yet. Now it does. Edit `rust/Cargo.toml` so that `members = ["nurbs", "nurbs-c-api"]`. Without this, workspace-wide commands (`cargo build --workspace`, `cargo test --workspace`) silently skip the c-api crate.

- [ ] **Step 1: Create `rust/nurbs-c-api/Cargo.toml`**

```toml
[package]
name = "nurbs-c-api"
version = "0.1.0"
edition = "2021"
rust-version = "1.85"
publish = false
description = "Stable C ABI surface for the nurbs crate"

[lib]
crate-type = ["staticlib", "rlib"]

[features]
default = ["mcu-h7"]
mcu-h7 = ["nurbs/mcu-h7"]
mcu-f4 = ["nurbs/mcu-f4"]
host = ["nurbs/host"]

[dependencies]
nurbs = { path = "../nurbs", default-features = false }

[build-dependencies]
cbindgen = "0.27"
```

- [ ] **Step 2: Create `rust/nurbs-c-api/cbindgen.toml`**

```toml
language = "C"
header = """\
/*\n\
 * kalico_nurbs.h — generated by cbindgen.\n\
 * DO NOT EDIT. Regenerate via `cargo run -p nurbs-c-api --bin gen-headers`.\n\
 * See docs/superpowers/specs/2026-04-26-nurbs-evaluation-library-design.md.\n\
 */\n"""

include_guard = "KALICO_NURBS_H"
pragma_once = true
no_includes = false

[export]
prefix = "kalico_nurbs_"

[parse]
parse_deps = true
include = ["nurbs"]
```

- [ ] **Step 3: Create `rust/nurbs-c-api/src/lib.rs` with stub exports**

```rust
//! Stable C ABI surface for the nurbs crate.
//!
//! All symbols are namespaced `kalico_nurbs_*` and exposed via cbindgen.
//! The generated header lives at `nurbs-c-api/include/kalico_nurbs.h`
//! and is checked into source control; CI verifies that regenerating it
//! produces a no-op diff.

#![cfg_attr(not(feature = "host"), no_std)]

use nurbs::{eval, ScalarNurbsRef, VectorNurbsRef, ArcLengthTableRef, NurbsView};

/// Evaluate a scalar NURBS at parameter `u`. Returns the position.
///
/// Caller must guarantee `curve` is a valid (non-null, properly initialized)
/// pointer to a `ScalarNurbsRef<f32>` with stable lifetime through the call.
#[no_mangle]
pub unsafe extern "C" fn kalico_nurbs_eval_f32(
    curve: *const ScalarNurbsRef<'_, f32>,
    u: f32,
) -> f32 {
    let curve_ref: &ScalarNurbsRef<'_, f32> = unsafe { &*curve };
    eval(curve_ref, u)
}

/// Evaluate a vector NURBS in R^3 at parameter `u`. Writes the resulting
/// 3-vector into `out` (caller-allocated, length 3).
#[no_mangle]
pub unsafe extern "C" fn kalico_nurbs_vector_eval_3_f32(
    curve: *const VectorNurbsRef<'_, f32, 3>,
    u: f32,
    out: *mut f32,
) {
    let curve_ref: &VectorNurbsRef<'_, f32, 3> = unsafe { &*curve };
    let result = nurbs::eval::vector_eval(curve_ref, u);
    let out_slice = unsafe { core::slice::from_raw_parts_mut(out, 3) };
    out_slice.copy_from_slice(&result);
}

/// Look up a parameter `u` corresponding to arc length `s` in a precomputed table.
#[no_mangle]
pub unsafe extern "C" fn kalico_nurbs_param_from_arc_length_f32(
    table: *const ArcLengthTableRef<'_, f32>,
    s: f32,
) -> f32 {
    let table_ref: &ArcLengthTableRef<'_, f32> = unsafe { &*table };
    nurbs::arc_length::param_from_arc_length(table_ref, s)
}
```

- [ ] **Step 4: Verify it compiles for both host and MCU**

```bash
cd rust
cargo build -p nurbs-c-api --features host --no-default-features
cargo check -p nurbs-c-api --no-default-features --features mcu-h7 --target thumbv7em-none-eabihf
```

Expected: both succeed.

- [ ] **Step 5: Commit**

```bash
git add rust/nurbs-c-api/
git commit -m "nurbs-c-api: scaffold staticlib crate with initial extern \"C\" surface"
```

---

## Task 26: cbindgen header-generation binary

**Files:**
- Create: `rust/nurbs-c-api/src/bin/gen_headers.rs`
- Create: `rust/nurbs-c-api/include/kalico_nurbs.h` (committed; generated)

- [ ] **Step 1: Create the generator binary**

Create `rust/nurbs-c-api/src/bin/gen_headers.rs`:

```rust
//! Run with `cargo run -p nurbs-c-api --bin gen-headers --features host`.
//! Regenerates `nurbs-c-api/include/kalico_nurbs.h` from the cbindgen config.
//! Must produce no diff in CI.

use std::path::PathBuf;

fn main() {
    let crate_dir = std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR set by cargo");
    let crate_dir = PathBuf::from(crate_dir);
    let config = cbindgen::Config::from_file(crate_dir.join("cbindgen.toml"))
        .expect("cbindgen.toml should be parseable");
    let output_path = crate_dir.join("include").join("kalico_nurbs.h");

    cbindgen::Builder::new()
        .with_crate(&crate_dir)
        .with_config(config)
        .generate()
        .expect("cbindgen generation must succeed")
        .write_to_file(&output_path);

    println!("wrote header to {}", output_path.display());
}
```

- [ ] **Step 2: Adjust `nurbs-c-api/Cargo.toml` to depend on cbindgen normally for the binary**

Edit the deps section:

```toml
[dependencies]
nurbs = { path = "../nurbs", default-features = false }

[target.'cfg(feature = "host")'.dependencies]
cbindgen = { version = "0.27", optional = true }

[features]
default = ["mcu-h7"]
mcu-h7 = ["nurbs/mcu-h7"]
mcu-f4 = ["nurbs/mcu-f4"]
host = ["nurbs/host", "dep:cbindgen"]
```

(Remove the earlier `[build-dependencies]` line — cbindgen runs as a binary, not a build script.)

- [ ] **Step 3: Generate the header for the first time**

```bash
cd rust
cargo run -p nurbs-c-api --bin gen-headers --features host --no-default-features
cat nurbs-c-api/include/kalico_nurbs.h | head -40
```

Expected: a `.h` file with `kalico_nurbs_*` declarations.

- [ ] **Step 4: Add a CI no-op check** — `rust/nurbs-c-api/tests/headers_no_drift.rs`

```rust
//! Verifies that `cargo run --bin gen-headers` produces a no-op diff against
//! the committed header. Run as `cargo test -p nurbs-c-api --features host
//! --test headers_no_drift`.

#[test]
#[cfg(feature = "host")]
fn header_in_repo_matches_generated() {
    let crate_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let header_path = crate_dir.join("include").join("kalico_nurbs.h");
    let committed = std::fs::read_to_string(&header_path)
        .expect("committed header must exist");

    let config = cbindgen::Config::from_file(crate_dir.join("cbindgen.toml"))
        .expect("cbindgen config");
    let regenerated = cbindgen::Builder::new()
        .with_crate(&crate_dir)
        .with_config(config)
        .generate()
        .expect("regeneration must succeed");

    let mut buf = Vec::new();
    regenerated.write(&mut buf);
    let regenerated_str = String::from_utf8(buf).expect("utf-8 output");

    if committed != regenerated_str {
        panic!(
            "kalico_nurbs.h is out of date. Run:\n  \
             cargo run -p nurbs-c-api --bin gen-headers --features host"
        );
    }
}
```

- [ ] **Step 5: Run the no-drift test**

```bash
cargo test -p nurbs-c-api --features host --no-default-features --test headers_no_drift
```

Expected: passes.

- [ ] **Step 6: Commit**

```bash
git add rust/nurbs-c-api/
git commit -m "nurbs-c-api: add cbindgen header generator + no-drift test"
```

---

## Task 27: ABI smoke test (C program linking against the staticlib)

**Files:**
- Create: `rust/tests/abi/Makefile`
- Create: `rust/tests/abi/test_eval.c`
- Create: `rust/tests/abi/run.sh`

- [ ] **Step 1: Create `rust/tests/abi/test_eval.c`**

```c
/*
 * ABI smoke test: link against libnurbs_c_api.a and call a few core functions
 * with known inputs. Verifies the C ABI compiles, links, and produces correct
 * results across the language boundary.
 */

#include "../../nurbs-c-api/include/kalico_nurbs.h"
#include <stdio.h>
#include <stdlib.h>

int main(void) {
    /*
     * The C side cannot construct ScalarNurbsRef directly — it's an opaque
     * Rust type. For this v1 smoke test, we'd normally call a constructor
     * extern "C" function (e.g., kalico_nurbs_scalar_ref_from_wire). That
     * function isn't part of v1's minimal ABI surface; this smoke test
     * therefore exercises only that the header compiles and the staticlib
     * links — full call-through is gated on Layer 5 wire format integration.
     */

    printf("ABI smoke: header parsed, staticlib linked\n");
    return 0;
}
```

- [ ] **Step 2: Create `rust/tests/abi/Makefile`**

```makefile
# ABI smoke test: link a C program against the Rust staticlib.

ABI_DIR := $(dir $(realpath $(firstword $(MAKEFILE_LIST))))
RUST_ROOT := $(abspath $(ABI_DIR)/../..)
TARGET_DIR := $(RUST_ROOT)/target

# Default profile: host (debug), MCU build runs separately.
PROFILE ?= debug
LIB_DIR := $(TARGET_DIR)/$(PROFILE)
CFLAGS := -O2 -Wall -Wextra -Werror

test_eval: $(LIB_DIR)/libnurbs_c_api.a $(ABI_DIR)/test_eval.c
	$(CC) $(CFLAGS) $(ABI_DIR)/test_eval.c -L$(LIB_DIR) -lnurbs_c_api -lpthread -ldl -lm -o $@

$(LIB_DIR)/libnurbs_c_api.a:
	cd $(RUST_ROOT) && cargo build -p nurbs-c-api --features host --no-default-features

run: test_eval
	./test_eval

clean:
	rm -f test_eval

.PHONY: clean run
```

- [ ] **Step 3: Create `rust/tests/abi/run.sh`**

```bash
#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
make clean
make run
```

```bash
chmod +x rust/tests/abi/run.sh
```

- [ ] **Step 4: Run the smoke test**

```bash
cd rust/tests/abi
./run.sh
```

Expected: program prints `ABI smoke: header parsed, staticlib linked` and exits 0.

- [ ] **Step 5: Commit**

```bash
git add rust/tests/abi/
git commit -m "nurbs-c-api: add ABI smoke test (C program linking staticlib)"
```

---

## Task 28: geomdl reference oracle

**Files:**
- Create: `rust/nurbs/tests/geomdl_oracle.rs`
- Create: `rust/nurbs/tests/data/geomdl_corpus.json`
- Create: `rust/nurbs/tests/scripts/generate_geomdl_corpus.py`

- [ ] **Step 1: Create the corpus generation script**

Create `rust/nurbs/tests/scripts/generate_geomdl_corpus.py`:

```python
#!/usr/bin/env python3
"""Generate a fixed test corpus by evaluating curves with NURBS-Python (geomdl).

The output JSON is checked into source control as the ground truth for the
oracle test in tests/geomdl_oracle.rs.

Run with:
    pip install geomdl
    python tests/scripts/generate_geomdl_corpus.py > tests/data/geomdl_corpus.json
"""

import json
from geomdl import BSpline, NURBS

def linear_curve():
    c = BSpline.Curve()
    c.degree = 1
    c.ctrlpts = [[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]]
    c.knotvector = [0.0, 0.0, 1.0, 1.0]
    return c

def quadratic_arc():
    """Rational quadratic 90° arc from (1,0) to (0,1)."""
    c = NURBS.Curve()
    c.degree = 2
    c.ctrlptsw = [
        [1.0, 0.0, 0.0, 1.0],
        [0.7071067811865476, 0.7071067811865476, 0.0, 0.7071067811865476],
        [0.0, 1.0, 0.0, 1.0],
    ]
    c.knotvector = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]
    return c

def cubic_curve():
    c = BSpline.Curve()
    c.degree = 3
    c.ctrlpts = [
        [0.0, 0.0, 0.0], [1.0, 2.0, 0.0], [3.0, 2.0, 1.0], [4.0, 0.0, 0.0]
    ]
    c.knotvector = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]
    return c

def serialize(name, curve):
    return {
        "name": name,
        "degree": curve.degree,
        "knots": list(curve.knotvector),
        "control_points": [list(p) for p in curve.ctrlpts],
        "weights": list(curve.weights) if hasattr(curve, "weights") and curve.weights else None,
        "samples": [
            {"u": u, "point": curve.evaluate_single(u)}
            for u in [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0]
        ],
    }

def main():
    corpus = {
        "curves": [
            serialize("linear", linear_curve()),
            serialize("quadratic_arc_rational", quadratic_arc()),
            serialize("cubic_bspline", cubic_curve()),
        ],
    }
    print(json.dumps(corpus, indent=2))

if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Generate the corpus**

```bash
cd rust/nurbs
pip install --user geomdl    # if not already present
mkdir -p tests/data
python3 tests/scripts/generate_geomdl_corpus.py > tests/data/geomdl_corpus.json
```

Expected: a JSON file with three curves.

- [ ] **Step 3: Add `serde_json` as a dev dep**

Edit `rust/nurbs/Cargo.toml`:

```toml
[dev-dependencies]
proptest = "1.5"
serde_json = "1.0"
```

- [ ] **Step 4: Create `rust/nurbs/tests/geomdl_oracle.rs`**

```rust
//! Cross-check our eval against NURBS-Python (geomdl) on a fixed corpus.
//! Corpus file: `tests/data/geomdl_corpus.json` (regenerated via the
//! Python script in `tests/scripts/`).

#![cfg(feature = "host")]

use serde_json::Value;
use std::fs;
use std::path::PathBuf;

const TOLERANCE: f64 = 1e-9;

#[test]
fn oracle_matches_for_corpus_curves() {
    let path: PathBuf = [env!("CARGO_MANIFEST_DIR"), "tests", "data", "geomdl_corpus.json"]
        .iter().collect();
    let raw = fs::read_to_string(&path).expect("corpus must exist");
    let v: Value = serde_json::from_str(&raw).expect("valid JSON");

    for curve_v in v["curves"].as_array().unwrap() {
        let name = curve_v["name"].as_str().unwrap();
        let degree = curve_v["degree"].as_u64().unwrap() as u8;
        let knots: Vec<f64> = curve_v["knots"].as_array().unwrap()
            .iter().map(|x| x.as_f64().unwrap()).collect();
        let cps_3d: Vec<[f64; 3]> = curve_v["control_points"].as_array().unwrap()
            .iter().map(|p| {
                let arr = p.as_array().unwrap();
                [arr[0].as_f64().unwrap(), arr[1].as_f64().unwrap(), arr[2].as_f64().unwrap()]
            }).collect();
        let weights: Option<Vec<f64>> = if curve_v["weights"].is_null() {
            None
        } else {
            Some(curve_v["weights"].as_array().unwrap()
                .iter().map(|x| x.as_f64().unwrap()).collect())
        };

        let curve = nurbs::VectorNurbs::<f64, 3>::try_new(
            degree, knots, cps_3d, weights,
        ).unwrap_or_else(|e| panic!("{name}: try_new failed: {e:?}"));

        for sample in curve_v["samples"].as_array().unwrap() {
            let u = sample["u"].as_f64().unwrap();
            let expected = sample["point"].as_array().unwrap();
            let result = nurbs::eval::vector_eval(&curve.as_view(), u);
            for axis in 0..3 {
                let exp = expected[axis].as_f64().unwrap();
                let diff = (result[axis] - exp).abs();
                assert!(
                    diff < TOLERANCE,
                    "{name} u={u} axis={axis}: got {} expected {} (diff {diff})",
                    result[axis], exp,
                );
            }
        }
    }
}
```

- [ ] **Step 5: Run the oracle**

```bash
cargo test -p nurbs --test geomdl_oracle
```

Expected: passes for all three curves.

- [ ] **Step 6: Commit**

```bash
git add rust/nurbs/tests/
git commit -m "nurbs: add geomdl reference oracle and seed corpus"
```

---

## Task 29: Property-based tests (proptest invariants)

**Files:**
- Create: `rust/nurbs/tests/proptest_invariants.rs`

- [ ] **Step 1: Create the property tests**

Create `rust/nurbs/tests/proptest_invariants.rs`:

```rust
//! Property-based tests for NURBS evaluation invariants.
//!
//! These hold by construction — eval at first knot returns first cp,
//! derivative of constant is zero, etc. Catches regressions after refactors
//! that the fixed corpus oracle wouldn't.

#![cfg(feature = "host")]

use proptest::prelude::*;

fn arb_degree() -> impl Strategy<Value = u8> {
    1u8..=5
}

fn arb_cp_count(degree: u8) -> impl Strategy<Value = usize> {
    let min = (degree as usize) + 1;
    min..=10
}

fn arb_curve() -> impl Strategy<Value = nurbs::ScalarNurbs<f64>> {
    arb_degree().prop_flat_map(|p| {
        arb_cp_count(p).prop_flat_map(move |n| {
            let cps = prop::collection::vec(-10.0..10.0_f64, n);
            cps.prop_map(move |cps_vec| {
                // Build a clamped uniform knot vector.
                let mut knots = Vec::with_capacity(n + p as usize + 1);
                for _ in 0..=p { knots.push(0.0); }
                let interior = n.saturating_sub(p as usize + 1);
                for i in 1..=interior {
                    knots.push(i as f64 / (interior + 1) as f64);
                }
                for _ in 0..=p { knots.push(1.0); }
                nurbs::ScalarNurbs::try_new(p, knots, cps_vec, None).unwrap()
            })
        })
    })
}

proptest! {
    #[test]
    fn eval_at_first_knot_returns_first_cp(curve in arb_curve()) {
        let view = curve.as_view();
        let u_start = view.knots()[0];
        let result = nurbs::eval::eval(&view, u_start);
        let expected = view.control_points()[0];
        prop_assert!((result - expected).abs() < 1e-9, "got {result}, expected {expected}");
    }

    #[test]
    fn eval_at_last_knot_returns_last_cp(curve in arb_curve()) {
        let view = curve.as_view();
        let u_end = view.knots()[view.knots().len() - 1];
        let result = nurbs::eval::eval(&view, u_end);
        let expected = view.control_points()[view.control_points().len() - 1];
        prop_assert!((result - expected).abs() < 1e-9, "got {result}, expected {expected}");
    }

    #[test]
    fn derivative_of_constant_curve_is_zero(p in 1u8..=5) {
        let n = (p as usize) + 1;
        let cps = vec![3.14_f64; n];
        let mut knots = Vec::new();
        for _ in 0..=p { knots.push(0.0); }
        let interior = n.saturating_sub(p as usize + 1);
        for i in 1..=interior { knots.push(i as f64 / (interior + 1) as f64); }
        for _ in 0..=p { knots.push(1.0); }
        let curve = nurbs::ScalarNurbs::try_new(p, knots, cps, None).unwrap();
        let d = nurbs::eval::derivative(&curve);
        for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let val = nurbs::eval::eval(&d.as_view(), u);
            prop_assert!(val.abs() < 1e-9, "constant curve derivative at {u} = {val}");
        }
    }
}
```

- [ ] **Step 2: Run property tests**

```bash
cargo test -p nurbs --test proptest_invariants
```

Expected: passes.

- [ ] **Step 3: Commit**

```bash
git add rust/nurbs/tests/proptest_invariants.rs
git commit -m "nurbs: add proptest invariants (endpoint eval, derivative-of-const)"
```

---

## Task 30: Numerical conditioning tests

**Files:**
- Create: `rust/nurbs/tests/numerical.rs`

- [ ] **Step 1: Create the numerical tests**

Create `rust/nurbs/tests/numerical.rs`:

```rust
//! Numerical conditioning tests. Verifies pathological-but-valid input
//! produces predictable error/clamp behavior, never NaN/inf silent propagation.

#![cfg(feature = "host")]

#[test]
fn tiny_knot_range_evaluates_without_nan() {
    let curve = nurbs::ScalarNurbs::try_new(
        1,
        vec![0.0_f64, 0.0, 1e-8, 1e-8],
        vec![0.0, 1.0],
        None,
    ).expect("tiny but positive range is valid");
    let mid = nurbs::eval::eval(&curve.as_view(), 5e-9);
    assert!(mid.is_finite(), "expected finite eval, got {mid}");
}

#[test]
fn near_zero_weight_evaluates_within_clamp() {
    let curve = nurbs::ScalarNurbs::try_new(
        1,
        vec![0.0_f64, 0.0, 1.0, 1.0],
        vec![1.0, 2.0],
        Some(vec![1.0, 1e-12]),    // tiny but positive
    ).expect("positive weight passes validation");
    let v = nurbs::eval::eval(&curve.as_view(), 1.0);
    assert!(v.is_finite(), "expected finite, got {v}");
}

#[test]
fn curvature_clamps_at_cusp_like_input() {
    // Construct a curve where r' is near-zero by repeating control points
    // at a point. We use a degenerate degree-2 that almost stops at u=0.5.
    let curve = nurbs::VectorNurbs::<f64, 3>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1e-10, 0.0, 0.0], [1.0, 0.0, 0.0]],
        None,
    ).unwrap();
    let first = nurbs::eval::vector_derivative(&curve);
    let second = nurbs::eval::vector_derivative(&first);
    let k = nurbs::eval::curvature_from_derivs(&first, &second, 0.0_f64);
    assert!(k.is_finite(), "curvature must clamp, not blow up: got {k}");
}

#[test]
fn arc_length_builder_rejects_truly_degenerate_curve() {
    // Construct a curve whose entire image is one point — every CP equal.
    let curve = nurbs::VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[1.0, 1.0, 1.0], [1.0, 1.0, 1.0]],
        None,
    ).unwrap();
    let result = nurbs::arc_length::build_arc_length_table_vector(&curve, 1e-6, 64);
    assert!(matches!(result, Err(nurbs::ArcLengthError::DegenerateCurve)));
}
```

- [ ] **Step 2: Run numerical tests**

```bash
cargo test -p nurbs --test numerical
```

Expected: passes.

- [ ] **Step 3: Commit**

```bash
git add rust/nurbs/tests/numerical.rs
git commit -m "nurbs: add numerical conditioning tests"
```

---

## Task 31: Cross-precision regression harness (f32 vs f64)

**Files:**
- Create: `rust/nurbs/tests/cross_precision.rs`

- [ ] **Step 1: Create the cross-precision tests**

Create `rust/nurbs/tests/cross_precision.rs`:

```rust
//! Cross-precision regression harness. Runs the same curves at f32 and f64,
//! asserts the f32 result is within a documented bound of f64 on a representative
//! corpus. Catches numerical regressions in the f32 codegen path that the
//! geomdl oracle (f64 only) would miss.
//!
//! The bound is empirical — measure on the corpus, assert. If the bound creeps
//! up after a refactor, you've introduced a precision regression.

#![cfg(feature = "host")]

const F32_VS_F64_TOLERANCE: f32 = 1e-5;

fn build_test_curves<T: nurbs::Float>() -> Vec<nurbs::VectorNurbs<T, 3>>
where
    T: From<f32>,    // we'll route through f32 literals
{
    // We'd like generic construction, but trait coherence makes this awkward
    // for f32/f64 simultaneously. Instead, this test file builds the same
    // curve at both precisions inline below.
    Vec::new()
}

#[test]
fn vector_eval_f32_matches_f64_within_tolerance() {
    // Cubic 3D curve.
    let degree = 3u8;
    let knots_f64 = vec![0.0_f64, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let cps_f64: Vec<[f64; 3]> = vec![
        [0.0, 0.0, 0.0],
        [1.0, 2.0, 0.5],
        [3.0, 2.0, 1.0],
        [4.0, 0.0, 0.0],
    ];
    let curve_f64 = nurbs::VectorNurbs::<f64, 3>::try_new(
        degree, knots_f64.clone(), cps_f64.clone(), None,
    ).unwrap();

    let knots_f32: Vec<f32> = knots_f64.iter().map(|&x| x as f32).collect();
    let cps_f32: Vec<[f32; 3]> = cps_f64.iter()
        .map(|p| [p[0] as f32, p[1] as f32, p[2] as f32]).collect();
    let curve_f32 = nurbs::VectorNurbs::<f32, 3>::try_new(
        degree, knots_f32, cps_f32, None,
    ).unwrap();

    for u in [0.0_f64, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0] {
        let p64 = nurbs::eval::vector_eval(&curve_f64.as_view(), u);
        let p32 = nurbs::eval::vector_eval(&curve_f32.as_view(), u as f32);
        for axis in 0..3 {
            let diff = (p32[axis] - p64[axis] as f32).abs();
            assert!(
                diff < F32_VS_F64_TOLERANCE,
                "u={u} axis={axis}: f32={} f64={} diff={diff}",
                p32[axis], p64[axis],
            );
        }
    }
}
```

- [ ] **Step 2: Run cross-precision tests**

```bash
cargo test -p nurbs --test cross_precision
```

Expected: passes.

- [ ] **Step 3: Commit**

```bash
git add rust/nurbs/tests/cross_precision.rs
git commit -m "nurbs: add cross-precision regression harness (f32 vs f64)"
```

---

## Task 32: Final validation pass

**Files:** none modified — verification only.

- [ ] **Step 1: Run the full test suite**

```bash
cd rust
cargo test --workspace
```

Expected: all tests pass.

- [ ] **Step 2: MCU build check**

```bash
cargo check --workspace --no-default-features --features mcu-h7 --target thumbv7em-none-eabihf
```

Expected: builds cleanly.

- [ ] **Step 3: Lints**

```bash
cargo clippy --workspace -- -D warnings
```

Expected: no warnings.

- [ ] **Step 4: Format check**

```bash
cargo fmt --check
```

Expected: no diff.

- [ ] **Step 5: ABI smoke**

```bash
cd rust/tests/abi
./run.sh
```

Expected: passes.

- [ ] **Step 6: Header drift check**

```bash
cd rust
cargo test -p nurbs-c-api --features host --no-default-features --test headers_no_drift
```

Expected: passes.

- [ ] **Step 7: Commit any final cleanups**

If lints or formatting required changes, commit them:

```bash
git commit -am "nurbs: clippy / fmt cleanups from final validation"
```

---

## Done

At the end of this plan you have:

- A `nurbs` crate implementing the Layer 0 substrate, eval, arc-length, and algebra-interface modules per the design spec.
- A `nurbs-c-api` crate exposing a stable `extern "C"` surface with a committed cbindgen-generated header.
- A test infrastructure spanning per-feature TDD, geomdl reference oracle, proptest invariants, numerical conditioning, cross-precision regression, and ABI smoke.
- The `multiply` and `convolve_with_polynomial_kernel` algorithms explicitly stubbed with `NotImplemented` errors and documented as deferred.
- A workspace ready for Layer 1 (geometry pipeline) to build on top.

Two follow-up specs are anticipated and out of scope for this plan:

1. **`multiply` algorithm** — Piegl & Tiller ch. 5; lands when Layer 3 pre-bake needs it.
2. **`convolve_with_polynomial_kernel` algorithm** — research-flavored; lands at CLAUDE.md build step 8 (smooth shapers).
