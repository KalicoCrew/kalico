# NURBS Evaluation Library — Layer 0 Design

Status: design approved 2026-04-26 — implementation pending
Owner: Danila Dergachev
Layer: 0 (Mathematical foundations) per `CLAUDE.md` dependency graph

## Scope

In scope: the Layer 0 NURBS substrate plus three modules (eval, arc-length, algebra interface) at full implementation specification depth.

In scope, deferred algorithm: `convolve_with_polynomial_kernel` and `multiply` interfaces are specified; their algorithms are explicitly deferred to follow-up specs (multiply: well-trodden — Piegl & Tiller ch. 5 — but verbose; convolve: research-flavored, lands when smooth shapers come online at build step 8).

Out of scope: Layer 1+ (geometry pipeline, TOPP-RA, transforms, comms, MCU runtime). Each gets its own design round informed by the Layer 0 substrate this spec locks down.

Wire format scope: byte-layout for the eval crate's deserializer surface only. Framing, segment IDs, multi-MCU addressing, endianness handling, and clock sync are Layer 5's design.

## Architecture overview

One Cargo workspace at the kalico repo root under `rust/`. Two crates in v1:

- **`nurbs`** — the Layer 0 substrate plus `eval`, `arc_length`, `algebra` modules. Internal Rust API; not C-callable.
- **`nurbs-c-api`** — thin wrapper exposing a stable `extern "C"` surface, generated via `cbindgen` into a checked-in C header. Depends on `nurbs`.

The split keeps the Rust-internal API free to evolve while the ABI is a stable contract reviewed via header diffs.

```
rust/
├── Cargo.toml                # workspace root
├── rust-toolchain.toml       # pinned channel + components + targets
├── .cargo/config.toml        # per-target rustflags
├── README.md                 # layout, build commands, C-link contract
├── nurbs/
│   ├── Cargo.toml
│   ├── src/
│   │   ├── lib.rs            # substrate: types, traits, constants, Float, errors, wire format
│   │   ├── eval.rs           # de Boor, vector_eval, derivative, curvature
│   │   ├── arc_length.rs     # ArcLengthTable, builders, queries
│   │   └── algebra.rs        # scalar_multiply, add, multiply, convolve (interfaces)
│   └── tests/
│       ├── geomdl_oracle.rs  # cross-check against NURBS-Python on a corpus
│       ├── proptest_*.rs     # property tests (endpoint eval, monotonicity, etc.)
│       └── numerical.rs      # pathological-case conditioning
├── nurbs-c-api/
│   ├── Cargo.toml
│   ├── cbindgen.toml
│   ├── src/lib.rs            # extern "C" wrappers, namespaced kalico_nurbs_*
│   └── include/kalico_nurbs.h    # generated, committed
└── tests/
    └── abi/                  # C test program linking libnurbs_c_api.a
```

## Substrate (`nurbs::lib.rs`)

### Constants

```rust
pub const MAX_DEGREE: usize = 20;
pub const WORKSPACE_SIZE: usize = MAX_DEGREE + 1;       // de Boor scratch
pub const MIN_PARAMETRIC_SPEED: f64 = 1e-9;             // numerical clamp floor; tunable per caller
```

`MAX_DEGREE = 20` covers worst-case fitter output (5) + worst-case smooth-shaper kernel (~10) plus margin. Workspace at f32 is `21 × 4 = 84 B` — negligible against M7 DTCM. Promotion to a const-generic later is non-breaking provided the default is preserved.

### Float abstraction

```rust
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
    fn mul_add(self, a: Self, b: Self) -> Self;   // FMA — load-bearing on M7
    fn sqrt(self) -> Self;
    fn abs(self) -> Self;
    fn min(self, other: Self) -> Self;
    fn max(self, other: Self) -> Self;
}

impl Float for f32 { /* ... */ }                  // always available
#[cfg(feature = "f64")]
impl Float for f64 { /* ... */ }
```

`mul_add` is a load-bearing first-class method, not opportunistic FMA fusion — codegen determinism across LLVM versions matters more than relying on fast-math flags. `from_f64` is a regular method (not `const` — const trait methods are unstable; the compiler folds calls to literals at monomorphization). IEEE-correct `PartialOrd`/`PartialEq`; no `Ord`/`Eq` (NaN). No `num_traits` dep — surface is tight, audit-friendly, embedded-norm.

### NURBS data model

```rust
// Owned (host-only). Construction validates; mutation lives on these types.
#[cfg(feature = "host")]
pub struct ScalarNurbs<T: Float> {
    degree: u8,
    knots: Vec<T>,
    control_points: Vec<T>,
    weights: Option<Vec<T>>,
}

#[cfg(feature = "host")]
pub struct VectorNurbs<T: Float, const N: usize> {
    degree: u8,
    knots: Vec<T>,
    control_points: Vec<[T; N]>,                   // interleaved per axis (AoS)
    weights: Option<Vec<T>>,
}

// Borrowed (all targets). Read-only. Zero-copy from wire.
pub struct ScalarNurbsRef<'a, T: Float> {
    degree: u8,
    knots: &'a [T],
    control_points: &'a [T],
    weights: Option<&'a [T]>,
}

pub struct VectorNurbsRef<'a, T: Float, const N: usize> {
    degree: u8,
    knots: &'a [T],
    control_points: &'a [[T; N]],
    weights: Option<&'a [T]>,
}

// Read-only operations. Eval algorithms generic over this.
pub trait NurbsView<T: Float> {
    fn degree(&self) -> u8;
    fn knots(&self) -> &[T];
    fn control_points(&self) -> &[T];
    fn weights(&self) -> Option<&[T]>;
}

pub trait VectorNurbsView<T: Float, const N: usize> {
    fn degree(&self) -> u8;
    fn knots(&self) -> &[T];
    fn control_points(&self) -> &[[T; N]];
    fn weights(&self) -> Option<&[T]>;
}
```

Owned types implement the View traits by delegating to fields; borrowed types implement them by delegating to slices. Eval functions take `&V: NurbsView<T>` (or vector equivalent), written once, called from both host and MCU.

`ScalarNurbs` exposes `as_view(&self) -> ScalarNurbsRef<'_, T>` and `Deref` is **not** used (intentional — explicit projection makes the borrow boundary visible).

### Knot vector invariants

Clamped open only. Periodic and general knot vectors are out of scope; the planner has no use for closed-loop curves. Promotion path to a `KnotKind` enum is non-breaking provided `KnotKind::ClampedOpen` is the default.

The wire format omits any knot-kind discriminator. The deserializer comment must document the assumption explicitly: "Knot vector: assumed clamped open per Q7 of the design; validated at deserialization."

### Knot range and parameterization

The crate preserves whatever knot range the producer chose. It does **not** normalize to `[0, 1]`.

Layer 1 produces NURBS with knot range proportional to segment arc length (`[0, L_segment]`-style). Rationale:
- **Numerical conditioning**: `|dP/du|` is of order 1 (not exactly 1; varies pointwise — fitter output is approximately unit-speed, not guaranteed). Keeps derivative magnitudes in unit-ish regimes for f32 conditioning.
- **Reduced scaling-bug surface**: parametric quantities are directly comparable to physical quantities at consumer sites.

`du/ds` is **not** a constant; the arc-length table from this crate stores the pointwise relationship between s and u.

### Validation rules

Applied at `ScalarNurbs::try_new`, `VectorNurbs::try_new`, `ScalarNurbsRef::try_from_wire`, `VectorNurbsRef::try_from_wire`:

```
knots.len()       == control_points.len() + degree + 1
degree            <= MAX_DEGREE
knots[0..=degree] all equal                       // clamp at start
knots[knots.len()-1-degree..] all equal           // clamp at end
knots non-decreasing
knots[last]       > knots[0]                      // non-degenerate range
if weights.is_some():
  weights.len()   == control_points.len()
  all weights     > 0
```

Tiny-but-positive ranges are consumer responsibility (numerical robustness on degenerate-but-valid input is not Layer 0's concern beyond the strict-positivity check).

### Wire format

Layer 5 delivers buffers aligned to `align_of::<T>()` (4 bytes for f32, 8 for f64). The deserializer asserts alignment on entry; misaligned input is a `WireError::Misaligned`, not a panic.

```
Scalar NURBS:
  u8   format_version          // 0x01 for v1
  u8   degree
  u8   has_weights             // 0 or 1
  u8   reserved
  u16  knot_count
  u16  control_point_count
  T[knot_count]              knots
  T[control_point_count]     control_points
  T[control_point_count]     weights              // present iff has_weights == 1

Vector NURBS:
  u8   format_version          // 0x01 for v1
  u8   degree
  u8   has_weights
  u8   axes_n                  // validated against const generic N
  u16  knot_count
  u16  control_point_count
  T[knot_count]                       knots
  T[control_point_count * N]          control_points (interleaved)
  T[control_point_count]              weights      // iff has_weights == 1

Arc-length table:
  u8   format_version          // 0x01 for v1
  u8   reserved
  u16  sample_count
  u32  reserved                // pad header to 8 bytes for f64 alignment
  T[sample_count]  s
  T[sample_count]  u
```

All three headers are 8 bytes total so the trailing `T[..]` regions land naturally aligned for both `T = f32` and `T = f64`. The Layer 5 contract is that the buffer starts aligned to `align_of::<T>()`; given that and equal-T-typed contiguous slices, every region inside the payload stays aligned without per-region padding.

`axes_n` validated against `N` at `try_from_wire`: mismatch returns `WireError::AxisCountMismatch { expected, got }`. Closes a silent-corruption hole where a 4-axis curve deserialized as `VectorNurbsRef<'_, T, 3>` would silently produce wrong control points by reading wrong-stride data.

`format_version` enables protocol evolution without cbindgen header churn signalling. v1 deserializers reject unknown versions with `WireError::UnknownVersion`.

Endianness is the comm layer's concern; the eval crate trusts buffers in host-native order.

### Error taxonomy

Per-module errors with `From` conversions to a top-level `NurbsError`. Each implements `core::error::Error` (stable in 1.81+, `no_std` friendly) — host callers can integrate with `anyhow`/`eyre` if they want, no dep cost on the eval crate.

```rust
#[derive(Debug)]
pub enum ConstructError {
    DegreeExceeded { actual: u8, max: u8 },
    KnotCountMismatch { expected: usize, got: usize },
    KnotsNotClamped,
    KnotsNotMonotone,
    DegenerateKnotRange,
    WeightCountMismatch { expected: usize, got: usize },
    NonPositiveWeight,
}

#[derive(Debug)]
pub enum WireError {
    Misaligned,
    UnknownVersion(u8),
    TruncatedBuffer { expected_len: usize, got: usize },
    AxisCountMismatch { expected: usize, got: u8 },
    Construct(ConstructError),     // wraps invariant violations from validated header data
}

#[derive(Debug)]
pub enum ArcLengthError<T> {
    ToleranceNotMet { achieved_residual: T, samples_used: usize },
    DegenerateCurve,                                // |dP/du| below MIN_PARAMETRIC_SPEED
}

#[derive(Debug)]
pub enum AlgebraError {
    DegreeExceeded { result_degree: u8, max: u8 },
    KnotMismatch,                                   // add/multiply with incompatible knots
}

#[derive(Debug)]
pub enum NurbsError<T> {
    Construct(ConstructError),
    Wire(WireError),
    ArcLength(ArcLengthError<T>),
    Algebra(AlgebraError),
}
// hand-written From impls for each variant — no thiserror dep
```

## eval module (`nurbs::eval`)

### Surface

```rust
// Hot path. MCU + host. No allocation.
pub fn eval<T: Float, V: NurbsView<T>>(curve: &V, u: T) -> T;

// Hot path, vector. Amortizes knot-span lookup across N axes for shared-knot Uniform path NURBS.
pub fn vector_eval<T: Float, V: VectorNurbsView<T, N>, const N: usize>(
    curve: &V,
    u: T,
) -> [T; N];

// Host only. Allocates a new owned NURBS via degree-lowering.
// Returns a parametric (∂/∂u) derivative — NOT temporal.
#[cfg(feature = "host")]
pub fn derivative<T: Float>(curve: &ScalarNurbs<T>) -> ScalarNurbs<T>;

#[cfg(feature = "host")]
pub fn vector_derivative<T: Float, const N: usize>(
    curve: &VectorNurbs<T, N>,
) -> VectorNurbs<T, N>;

// Host only. Caller owns cached first/second derivatives — TOPP-RA queries many u's per segment.
// Higher-level segment-aware curvature(segment, u) wrapper lives at Layer 1/2, not in this crate.
#[cfg(feature = "host")]
pub fn curvature_from_derivs<T: Float, const N: usize>(
    first_deriv: &VectorNurbs<T, N>,
    second_deriv: &VectorNurbs<T, N>,
    u: T,
) -> T;
```

### Algorithm details

**de Boor inner loop** uses `Float::mul_add` for the FMA-shaped recurrence. Scratch buffer is `[T; WORKSPACE_SIZE]` on the stack; `debug_assert!(curve.degree() as usize <= MAX_DEGREE)` as defense-in-depth (construction and wire deserialization already validated).

**Rational case**: branch on `weights.is_some()`. Non-rational path is one de Boor walk over control points. Rational path walks both weighted control points and weights, divides at the end:

```rust
fn eval<T: Float, V: NurbsView<T>>(curve: &V, u: T) -> T {
    match curve.weights() {
        None => de_boor_inner(curve.control_points(), curve.knots(), curve.degree(), u),
        Some(w) => {
            let numer = de_boor_homogeneous(curve.control_points(), w, curve.knots(), curve.degree(), u);
            let denom = de_boor_inner(w, curve.knots(), curve.degree(), u);
            debug_assert!(denom.abs() > T::from_f64(MIN_PARAMETRIC_SPEED));
            numer / denom.max(T::from_f64(MIN_PARAMETRIC_SPEED))
        }
    }
}
```

The weights branch is predicted strongly (per-segment, the answer doesn't change). Interleaving the two de Boor walks for cache locality is a v2 optimization; benchmarks tell us whether it earns the complexity.

**vector_eval** finds the knot span once, then runs N de Boor recurrences sharing the span lookup and the alpha computation; on the M7, this is meaningfully cheaper than N independent scalar `eval` calls for shared-knot vector NURBS. PerAxis-decomposed segments fall back to N independent scalar evals at the consumer site, not in this function.

**derivative** uses the standard degree-lowering identity: derivative of a degree-p NURBS with knots `{u_i}` and control points `{P_i}` is a degree-(p−1) NURBS with knots `{u_{i+1}, …, u_{n+p−1}}` and control points `Q_i = p · (P_{i+1} − P_i) / (u_{i+p+1} − u_{i+1})`. Allocates new `Vec`s; host only.

**curvature_from_derivs** computes `||r' × r''|| / ||r'||³`, with the cubed denominator clamped at `T::from_f64(MIN_PARAMETRIC_SPEED)` to avoid blow-up at cusps:

```rust
let r_prime = vector_eval(first, u);
let r_double = vector_eval(second, u);
let cross = cross3(r_prime, r_double);
let speed = norm3(r_prime);
let speed_cubed = (speed * speed * speed).max(T::from_f64(MIN_PARAMETRIC_SPEED));
norm3(cross) / speed_cubed
```

For G2/G3 rational quadratics and well-formed fitter output, the clamp never engages; for pathological CAM input or numerical artifacts it produces a finite (capped) curvature instead of NaN/inf, with the event flagged for telemetry by the Layer 2 caller.

## arc_length module (`nurbs::arc_length`)

### Types

```rust
#[cfg(feature = "host")]
pub struct ArcLengthTable<T: Float> {
    s: Vec<T>,                                      // monotone non-decreasing
    u: Vec<T>,                                      // monotone non-decreasing
}

pub struct ArcLengthTableRef<'a, T: Float> {
    s: &'a [T],
    u: &'a [T],
}
```

Split-vector layout (not interleaved `(s, u)` pairs). Binary search on `s` only touches the `s` array, so cache lines pull only `s` values during the probe; `u` is accessed once at the end. Meaningful at table sizes >256 samples on the M7's 16 KB D-cache.

### Surface

```rust
#[cfg(feature = "host")]
pub fn build_arc_length_table_scalar<T: Float, V: NurbsView<T>>(
    curve: &V,
    tolerance: T,
    max_samples: usize,
) -> Result<ArcLengthTable<T>, ArcLengthError<T>>;

#[cfg(feature = "host")]
pub fn build_arc_length_table_vector<T: Float, V: VectorNurbsView<T, 3>>(
    curve: &V,
    tolerance: T,
    max_samples: usize,
) -> Result<ArcLengthTable<T>, ArcLengthError<T>>;

// Available on both host and MCU builds. Pure lookup. Documented contract: per-segment local s.
pub fn param_from_arc_length<T: Float>(table: &ArcLengthTableRef<'_, T>, s: T) -> T;
pub fn arc_length_from_param<T: Float>(table: &ArcLengthTableRef<'_, T>, u: T) -> T;
```

`build_*` keeps idiomatic Rust naming for builders; the `<output>_from_<input>` Q5 convention applies to the value-query free functions only.

### Algorithm details

**Builders** sample `u` adaptively, integrating `|dP/du|` (scalar) or `||dP/du||` (vector 3D Euclidean norm) between samples via Gauss-Legendre quadrature, accumulating `s`. Sample density doubles until linear-interpolation residual on the table is below `tolerance` or `max_samples` is reached. On cap-hit, `Err(ToleranceNotMet { achieved_residual, samples_used })`.

Both builders share an internal helper:

```rust
fn integrate_arc_length<T: Float, F: Fn(T) -> T>(
    integrand: F,
    u_start: T,
    u_end: T,
    quadrature_points: usize,
) -> T;
```

Generic over the integrand closure, monomorphized per call site, inlined through. Two builders, one shared loop, no trait machinery.

If integration encounters a parametric speed below `MIN_PARAMETRIC_SPEED`, the builder returns `Err(DegenerateCurve)` rather than producing a table with infinite arc length at a cusp. Same numerical floor as eval/curvature.

Build-time Newton refinement (refining initial linear-interp guesses against the curve to tighten table accuracy at fixed sample count) is permitted on the host; query-time Newton is out of scope for the MCU API, which provides linear-interp accuracy as its contract.

**Queries** binary-search `s` (or `u`), linear-interpolate the other axis. `param_from_arc_length(table, s)`:

```rust
pub fn param_from_arc_length<T: Float>(table: &ArcLengthTableRef<'_, T>, s: T) -> T {
    debug_assert!(s >= T::ZERO && s <= table.s_max());
    let s_clamped = s.max(T::ZERO).min(table.s_max());     // release-mode clamp
    // binary search + linear interp
    ...
}
```

Debug builds catch out-of-range queries; release builds clamp silently. `Result` is too expensive on a 40 kHz hot path; clamp degrades gracefully.

### Per-segment locality contract

All eval-time quantities (`s`, `u`, `t`) are **segment-local**. The crate provides no global lookup. The Layer 4 per-axis evaluator computes `local_t = global_t − segment_start_time` once per sample at the segment-buffer-management layer and threads `local_t` through `arc_length_from_time` (Layer 2) → `param_from_arc_length` (Layer 0) → `vector_eval` (Layer 0). This contract is documented in the docstring of every public function in this module.

## algebra module (`nurbs::algebra`)

All host-only. Each operation produces new owned NURBS.

### Surface

```rust
#[cfg(feature = "host")]
pub fn scalar_multiply<T: Float>(curve: &ScalarNurbs<T>, scalar: T) -> ScalarNurbs<T>;

#[cfg(feature = "host")]
pub fn add<T: Float>(
    a: &ScalarNurbs<T>,
    b: &ScalarNurbs<T>,
) -> Result<ScalarNurbs<T>, AlgebraError>;
// Implementation note: knot insertion required to align knot vectors before pointwise control-point sum.

#[cfg(feature = "host")]
pub fn multiply<T: Float>(
    a: &ScalarNurbs<T>,
    b: &ScalarNurbs<T>,
) -> Result<ScalarNurbs<T>, AlgebraError>;
// Result degree = degree(a) + degree(b); may exceed MAX_DEGREE → AlgebraError::DegreeExceeded.
// Algorithm: deferred to follow-up spec. Reference: Piegl & Tiller, "The NURBS Book", ch. 5.

#[cfg(feature = "host")]
pub struct PolynomialKernel<T: Float> {
    coefficients: Vec<T>,                           // dense polynomial in u, low-to-high order
    support: (T, T),                                // [u_min, u_max] of kernel support
}

#[cfg(feature = "host")]
pub fn convolve_with_polynomial_kernel<T: Float>(
    curve: &ScalarNurbs<T>,
    kernel: &PolynomialKernel<T>,
) -> Result<ScalarNurbs<T>, AlgebraError>;
// Result degree = degree(curve) + kernel.degree(); may exceed MAX_DEGREE → AlgebraError::DegreeExceeded.
// Algorithm: deferred. Research-flavored — derived from B-spline basis-function math.
// Implementation lands when smooth shapers come online (build step 8 of CLAUDE.md build order).
```

### Implementation status

- `scalar_multiply`: textbook (multiply control points and weights by scalar). v1 implementation.
- `add`: standard but verbose (knot insertion to align, pointwise control-point sum, weight handling). v1 implementation.
- `multiply`: well-trodden algorithm but verbose with weights and non-uniform knots. **Deferred** to a follow-up spec; interface signature is stable.
- `convolve_with_polynomial_kernel`: research-flavored. **Deferred** to a follow-up spec when smooth shapers come online; interface signature is stable.

The deferred operations are not blocking for build steps 1–7 of the `CLAUDE.md` build order. They become required at build step 8 (smooth shapers).

## C ABI surface (`nurbs-c-api`)

Separate crate so the Rust-internal API can evolve without breaking the C contract. Generated header `kalico_nurbs.h` is **checked into source control**, not generated at build time. CI verifies that running `cargo run --bin gen-headers` produces a no-op diff against the committed header — drift fails CI.

C symbols namespaced with `kalico_nurbs_` prefix to avoid collisions in C's flat namespace.

Initial surface is intentionally minimal — refined when Layer 4 actually needs to call in:

```rust
#[no_mangle]
pub extern "C" fn kalico_nurbs_eval_f32(
    curve: *const ScalarNurbsRefF32,
    u: f32,
) -> f32;

#[no_mangle]
pub extern "C" fn kalico_nurbs_vector_eval_f32(
    curve: *const VectorNurbsRefF32_3,
    u: f32,
    out: *mut f32,                                  // caller-provided length-3 buffer
);

#[no_mangle]
pub extern "C" fn kalico_nurbs_param_from_arc_length_f32(
    table: *const ArcLengthTableRefF32,
    s: f32,
) -> f32;
```

Pointer-passed types are opaque to C — C constructs them from Rust deserialization or via additional `extern "C"` constructors that wrap `try_from_wire`. C callers see structs by pointer only, never by value.

## Build configuration

### Workspace `Cargo.toml`

```toml
[workspace]
members = ["nurbs", "nurbs-c-api"]
resolver = "2"

[workspace.dependencies]
# shared deps versioned here from day one — avoids diamond problem at crate count >1
# (none yet; populate as deps land)

[profile.release]
opt-level = "z"           # size; revisit via benchmarks if eval hot path regresses
lto = "fat"
codegen-units = 1
panic = "abort"
debug = true              # symbols in .elf; stripped from .bin/.hex
overflow-checks = false
```

`opt-level = "z"` is a starting choice. Once criterion benchmarks exist (see Testing), revisit against `opt-level = 3` on a representative eval workload. Don't pre-optimize without measurement.

### `rust-toolchain.toml`

```toml
[toolchain]
channel = "1.83.0"        # pinned; updated intentionally with regression testing
components = ["rustfmt", "clippy"]
targets = [
    "thumbv7em-none-eabihf",      # both M7 (H723) and M4 (F4) targets
    "x86_64-unknown-linux-gnu",   # host CI
    "aarch64-unknown-linux-gnu",  # Pi 5 host
]
```

### `.cargo/config.toml`

Per-target rustflags. FPU flag strings are toolchain-version-dependent; the pinned 1.83 toolchain gets the values below. **Verify on toolchain bumps** — LLVM target-feature names have churned across releases.

```toml
[target.thumbv7em-none-eabihf]
rustflags = [
    "-C", "target-cpu=cortex-m7",
    "-C", "target-feature=+fp-armv8d16,+strict-align",
    "-C", "link-arg=--nmagic",
]
# H723 default. M4 builds override target-cpu and target-feature via Make's --target invocation
# or a separate config profile; see the Makefile integration in rust/README.md.
```

### Feature flag graph

```toml
# nurbs/Cargo.toml
[features]
default = ["host"]            # cargo test/build "just works" on a developer machine
f64 = []
host = ["f64"]
mcu-h7 = []
mcu-f4 = []
```

`host` implies `f64`. `mcu-*` features are mutually exclusive with each other and with `host`. MCU builds invoke `cargo build --no-default-features --features mcu-h7` (or `mcu-f4`) explicitly. Enforced at compile time:

```rust
// nurbs/src/lib.rs
#[cfg(all(feature = "mcu-h7", feature = "mcu-f4"))]
compile_error!("mcu-h7 and mcu-f4 are mutually exclusive");

#[cfg(all(feature = "host", any(feature = "mcu-h7", feature = "mcu-f4")))]
compile_error!("host is incompatible with mcu-* features");

#[cfg(not(any(feature = "host", feature = "mcu-h7", feature = "mcu-f4")))]
compile_error!("must specify exactly one of: host, mcu-h7, mcu-f4");
```

Without these, a `make` invocation that forgets `--features` silently does the wrong thing.

The `Float` impl gating uses `feature = "f64"` (a type concern), not `feature = "host"` (a target concern that happens to imply f64) — keeps the abstraction clean.

### F4xx FPU note

Cortex-M4 has VFPv4-SP-D16 (single-precision, 16 double-word registers). The boards in scope (Octopus/Manta family) all have FPU-equipped F4 variants. Soft-float fallback for FPU-less F4 variants (F405 some bins, etc.) is a future addition if needed — not in v1 scope.

## Testing strategy

### Reference oracle

Cross-check `eval`, `derivative`, `vector_eval`, `curvature_from_derivs` against Python `geomdl` (NURBS-Python) on a corpus covering:
- Degree-1 (G0/G1 reduced).
- Rational quadratic arcs (G2/G3 reduced) with known analytic positions and curvatures.
- Degree-3 cubics (G5/G5.1 reduced).
- Fitted spline output (sample inputs from a recorded fitter run once Layer 1 lands; until then, hand-constructed).
- Shaper-convolved curves (once Layer 3 lands).

Tolerances tight on f64 host (machine epsilon × small constant); looser on f32 MCU per the cross-precision regression harness below. Oracle runs in `tests/geomdl_oracle.rs` with a fixture corpus checked into the repo.

### Property tests

Via `proptest` (preferred) or `quickcheck`:
- de Boor at first knot endpoint equals first control point; at last knot endpoint equals last control point.
- Derivative of a constant curve is zero everywhere.
- Arc-length table is monotone non-decreasing in both `s` and `u`.
- `arc_length_from_param(build(curve), u_max) == segment_arc_length` (within tolerance).
- `convolve(curve, identity_kernel) ≈ curve` (when convolution algorithm lands).
- `add(curve, scalar_multiply(curve, -1.0))` is the zero curve (when `add` and `scalar_multiply` land).

### ABI smoke tests

In `rust/tests/abi/`. C test program links against `libnurbs_c_api.a`, calls `kalico_nurbs_eval_f32` and friends with known inputs, compares against expected outputs computed in Rust. Run from CI alongside `cargo test`. Catches ABI drift that compiles fine on both sides but breaks at the link/runtime boundary.

### Numerical conditioning tests

Pathological-but-valid inputs:
- Short knot ranges near the strict-positivity floor.
- Near-zero weights (positive but tiny).
- Near-cusp curves where `||r'||` approaches `MIN_PARAMETRIC_SPEED`.

Each must fail predictably (defined error or clamped result) — never `NaN`/`inf` silently propagating.

### Cross-precision regression harness

Beyond ABI smoke: a host-side test runs the same curves at f64 and f32, asserts the f32 result is within a documented ULPs bound of f64 on a representative corpus. Catches numerical regressions in the f32 codegen path that the geomdl oracle (f64 only) would miss. The bound is empirical — measure on the corpus, assert against it, fail loudly if it creeps up.

### Performance baseline (roadmap, not v1)

`criterion` benchmarks for `eval`, `vector_eval`, `param_from_arc_length`, `curvature_from_derivs` on the host with f32 (proxy for MCU). Tracked over time. The 40 kHz hot path budget on H723 is ~13750 cycles per axis-sample; degrading from 200 cycles to 400 cycles silently is the kind of regression that surfaces only at integration. Catch it at unit-test time.

Not blocking for v1 correctness; budget for it before the first MCU integration.

## Deferred work

Tracked here so the v1 spec is honest about what it doesn't fully specify:

1. **`multiply` algorithm**: well-trodden (Piegl & Tiller ch. 5) but verbose with non-uniform knots and weights. Interface signature stable; algorithm gets its own follow-up spec when needed.
2. **`convolve_with_polynomial_kernel` algorithm**: research-flavored — derived from B-spline basis-function math, not standardly tabulated. Lands at build step 8 (smooth shapers). Interface signature stable; algorithm gets its own follow-up spec.
3. **Soft-float fallback** for FPU-less M4 variants: not in v1 scope.
4. **Strided `VectorNurbsRef` for in-place Uniform→PerAxis decomposition**: v1 ships owned-copy decomposition (only fires on shaper-divergence, not a hot path). Strided view is a future optimization if profiling shows the copy as a bottleneck.
5. **Query-time Newton refinement on `param_from_arc_length`**: v1 contract is linear-interp accuracy. Caller wants tighter accuracy → caller increases sample count or uses build-time Newton. MCU API stays branch-free.
6. **Periodic / closed-loop knot vectors** (`KnotKind` enum): not in v1; promotion path is non-breaking.
7. **Performance baseline harness**: roadmap item before first MCU integration.

## Build order alignment

Per `CLAUDE.md` build order, this spec covers item #1 ("NURBS library (host + MCU) and arc-length tools — Layer 0"). Items #2 (G-code parser and geometric reduction → partial Layer 1) and #3 (TOPP-RA prototype → partial Layer 2) consume the substrate this spec defines. The deferred algorithms in `algebra` align with build step 8 (smooth shapers and shaper-aware TOPP-RA).

---

End of design.
