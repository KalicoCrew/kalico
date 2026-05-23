//! Endstop arm/trip primitive for Step 7-D homing.
//!
//! Step 1 is pure Rust: firmware pin binding and bridge serialization are
//! layered on later. The global single-arm slot is intentionally represented
//! with atomics only because the runtime crate denies unsafe code.

use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU16, AtomicU32, Ordering};

pub const MAX_SOURCES: usize = 4;
pub const MAX_STEPPERS: usize = 8;
const MAX_GPIO_PINS: usize = 256;

pub type PinId = u16;

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum SourceKind {
    Physical = 0,
    TmcDiag = 1,
    /// Software-triggered source: no GPIO pin is polled. The arm uses a
    /// credit-windowed deadline mechanism instead — the host periodically
    /// calls `extend_deadline` to push the window forward; if it stops
    /// (because the probe triggered on the host side), the deadline expires
    /// and the MCU freezes the segment autonomously.
    Software = 2,
}

/// Sentinel written to `trip_source_idx` when the trip was caused by a
/// deadline expiry rather than a GPIO assertion.
pub const TRIP_SOURCE_DEADLINE_EXPIRED: u8 = 0xFF;

/// Sentinel written to `trip_source_idx` when the trip was caused by an
/// explicit `software_trip` call from the C command handler.
pub const TRIP_SOURCE_SOFTWARE: u8 = 0xFE;

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ArmPolicy {
    TripImmediately = 0,
    WaitForClear = 1,
    IgnoreUntilMoving = 2,
}

impl TryFrom<u8> for ArmPolicy {
    type Error = u8;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::TripImmediately),
            1 => Ok(Self::WaitForClear),
            2 => Ok(Self::IgnoreUntilMoving),
            other => Err(other),
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct VelocityAxis(u8);

impl VelocityAxis {
    pub const X: Self = Self(0x01);
    pub const Y: Self = Self(0x02);
    pub const Z: Self = Self(0x04);
    pub const XY: Self = Self(Self::X.0 | Self::Y.0);
    pub const XYZ: Self = Self(Self::X.0 | Self::Y.0 | Self::Z.0);

    pub const fn bits(self) -> u8 {
        self.0
    }

    pub const fn from_bits_truncate(bits: u8) -> Self {
        Self(bits & Self::XYZ.0)
    }

    pub const fn contains(self, other: Self) -> bool {
        (self.0 & other.0) == other.0
    }

    pub const fn intersects(self, other: Self) -> bool {
        (self.0 & other.0) != 0
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct SourceConfig {
    pub kind: SourceKind,
    pub gpio: PinId,
    pub active_high: bool,
    pub policy: ArmPolicy,
    pub sample_n: u8,
    pub velocity_axis: VelocityAxis,
    pub v_min_q16: u32,
}

impl SourceConfig {
    pub const EMPTY: Self = Self {
        kind: SourceKind::Physical,
        gpio: 0,
        active_high: true,
        policy: ArmPolicy::TripImmediately,
        sample_n: 1,
        velocity_axis: VelocityAxis::XYZ,
        v_min_q16: 0,
    };
}

/// One source slot. Configuration and ISR-private latch state are atomic so the
/// global arm can stay safe Rust/no-std without a critical-section dependency.
#[derive(Debug)]
pub struct Source {
    pub kind: AtomicU8,
    pub gpio: AtomicU16,
    pub active_high: AtomicBool,
    pub policy: AtomicU8,
    pub sample_n: AtomicU8,
    pub velocity_axis: AtomicU8,
    pub v_min_q16: AtomicU32,
    pub sample_acc: AtomicU8,
    pub moved_above_v: AtomicBool,
    pub cleared: AtomicBool,
}

impl Source {
    pub const fn new() -> Self {
        Self {
            kind: AtomicU8::new(SourceKind::Physical as u8),
            gpio: AtomicU16::new(0),
            active_high: AtomicBool::new(true),
            policy: AtomicU8::new(ArmPolicy::TripImmediately as u8),
            sample_n: AtomicU8::new(1),
            velocity_axis: AtomicU8::new(VelocityAxis::XYZ.bits()),
            v_min_q16: AtomicU32::new(0),
            sample_acc: AtomicU8::new(0),
            moved_above_v: AtomicBool::new(false),
            cleared: AtomicBool::new(false),
        }
    }

    fn configure(&self, cfg: SourceConfig) {
        self.kind.store(cfg.kind as u8, Ordering::Release);
        self.gpio.store(cfg.gpio, Ordering::Release);
        self.active_high.store(cfg.active_high, Ordering::Release);
        self.policy.store(cfg.policy as u8, Ordering::Release);
        self.sample_n.store(cfg.sample_n, Ordering::Release);
        self.velocity_axis
            .store(cfg.velocity_axis.bits(), Ordering::Release);
        self.v_min_q16.store(cfg.v_min_q16, Ordering::Release);
        self.reset_latches();
    }

    fn reset_latches(&self) {
        self.sample_acc.store(0, Ordering::Release);
        self.moved_above_v.store(false, Ordering::Release);
        self.cleared.store(false, Ordering::Release);
    }
}

impl Default for Source {
    fn default() -> Self {
        Self::new()
    }
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ArmState {
    Idle = 0,
    Armed = 1,
    Tripping = 2,
    TrippedReady = 3,
    TrippedSent = 4,
    Disarmed = 5,
}

#[derive(Debug)]
pub struct Arm {
    pub arm_id: AtomicU32,
    pub source_count: AtomicU8,
    pub sources: [Source; MAX_SOURCES],
    pub state: AtomicU8,
    pub arm_clock_lo: AtomicU32,
    pub arm_clock_hi: AtomicU32,
    pub stepper_count: AtomicU8,
    pub stepper_oids: [AtomicU8; MAX_STEPPERS],
    pub snapshot: TripSnapshot,
    // --- Software-source deadline state ---
    /// `true` once the first `tick()` past `arm_clock` has set
    /// `deadline_clock`. Cleared to `false` on each `arm()`.
    pub deadline_active: AtomicBool,
    /// Seqlock version for `deadline_clock` lo/hi. Writers (ISR initial
    /// activation AND command-handler `extend_deadline`) bump to odd
    /// before writing, then to even after. The ISR reader skips the
    /// expiry check when it catches a mid-write (returns `Continue`
    /// instead of spinning — spinning would deadlock since the ISR
    /// can't yield to the command handler it preempted).
    pub deadline_version: AtomicU32,
    /// Low 32 bits of `deadline_clock` (the MCU clock value at which the
    /// deadline expires if no `extend_deadline` call has refreshed it).
    pub deadline_clock_lo: AtomicU32,
    /// High 32 bits of `deadline_clock`.
    pub deadline_clock_hi: AtomicU32,
    /// Low 32 bits of `grant_ticks` (window length in MCU clock ticks).
    pub grant_ticks_lo: AtomicU32,
    /// High 32 bits of `grant_ticks`.
    pub grant_ticks_hi: AtomicU32,
}

impl Arm {
    pub const fn new() -> Self {
        Self {
            arm_id: AtomicU32::new(0),
            source_count: AtomicU8::new(0),
            sources: [Source::new(), Source::new(), Source::new(), Source::new()],
            state: AtomicU8::new(ArmState::Idle as u8),
            arm_clock_lo: AtomicU32::new(0),
            arm_clock_hi: AtomicU32::new(0),
            stepper_count: AtomicU8::new(0),
            stepper_oids: [
                AtomicU8::new(0),
                AtomicU8::new(0),
                AtomicU8::new(0),
                AtomicU8::new(0),
                AtomicU8::new(0),
                AtomicU8::new(0),
                AtomicU8::new(0),
                AtomicU8::new(0),
            ],
            snapshot: TripSnapshot::new(),
            deadline_active: AtomicBool::new(false),
            deadline_version: AtomicU32::new(0),
            deadline_clock_lo: AtomicU32::new(0),
            deadline_clock_hi: AtomicU32::new(0),
            grant_ticks_lo: AtomicU32::new(0),
            grant_ticks_hi: AtomicU32::new(0),
        }
    }

    fn arm_clock(&self) -> u64 {
        let lo = u64::from(self.arm_clock_lo.load(Ordering::Acquire));
        let hi = u64::from(self.arm_clock_hi.load(Ordering::Acquire));
        (hi << 32) | lo
    }

    fn store_deadline_clock_seqlocked(&self, clock: u64) {
        let v = self.deadline_version.load(Ordering::Acquire);
        self.deadline_version.store(v | 1, Ordering::Release);
        self.deadline_clock_lo
            .store(clock as u32, Ordering::Release);
        self.deadline_clock_hi
            .store((clock >> 32) as u32, Ordering::Release);
        self.deadline_version
            .store(v.wrapping_add(2), Ordering::Release);
    }

    #[cfg(test)]
    fn deadline_clock_unchecked(&self) -> u64 {
        let lo = u64::from(self.deadline_clock_lo.load(Ordering::Acquire));
        let hi = u64::from(self.deadline_clock_hi.load(Ordering::Acquire));
        (hi << 32) | lo
    }

    fn try_read_deadline_clock(&self) -> Option<u64> {
        let v1 = self.deadline_version.load(Ordering::Acquire);
        if v1 & 1 != 0 {
            return None;
        }
        let lo = u64::from(self.deadline_clock_lo.load(Ordering::Acquire));
        let hi = u64::from(self.deadline_clock_hi.load(Ordering::Acquire));
        let v2 = self.deadline_version.load(Ordering::Acquire);
        if v1 != v2 {
            return None;
        }
        Some((hi << 32) | lo)
    }

    fn grant_ticks(&self) -> u64 {
        let lo = u64::from(self.grant_ticks_lo.load(Ordering::Acquire));
        let hi = u64::from(self.grant_ticks_hi.load(Ordering::Acquire));
        (hi << 32) | lo
    }

    fn store_grant_ticks(&self, ticks: u64) {
        self.grant_ticks_lo
            .store(ticks as u32, Ordering::Release);
        self.grant_ticks_hi
            .store((ticks >> 32) as u32, Ordering::Release);
    }

    /// Returns `true` if any active source has `SourceKind::Software`.
    fn has_software_source(&self) -> bool {
        let count = usize::from(self.source_count.load(Ordering::Acquire));
        self.sources.iter().take(count).any(|src| {
            src.kind.load(Ordering::Acquire) == SourceKind::Software as u8
        })
    }
}

impl Default for Arm {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug)]
pub struct TripSnapshot {
    pub version: AtomicU32,
    pub trip_clock_lo: AtomicU32,
    pub trip_clock_hi: AtomicU32,
    pub trip_source_idx: AtomicU8,
    pub step_count_count: AtomicU8,
    pub stepper_oids: [AtomicU8; MAX_STEPPERS],
    pub step_counts: [AtomicI32; MAX_STEPPERS],
}

impl TripSnapshot {
    pub const fn new() -> Self {
        Self {
            version: AtomicU32::new(0),
            trip_clock_lo: AtomicU32::new(0),
            trip_clock_hi: AtomicU32::new(0),
            trip_source_idx: AtomicU8::new(0),
            step_count_count: AtomicU8::new(0),
            stepper_oids: [
                AtomicU8::new(0),
                AtomicU8::new(0),
                AtomicU8::new(0),
                AtomicU8::new(0),
                AtomicU8::new(0),
                AtomicU8::new(0),
                AtomicU8::new(0),
                AtomicU8::new(0),
            ],
            step_counts: [
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
            ],
        }
    }
}

impl Default for TripSnapshot {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct ArmMsg {
    pub arm_id: u32,
    pub arm_clock: u64,
    pub source_count: u8,
    pub sources: [SourceConfig; MAX_SOURCES],
    pub stepper_count: u8,
    pub stepper_oids: [u8; MAX_STEPPERS],
    /// Deadline window length in MCU clock ticks, used when at least one
    /// source has `SourceKind::Software`. Computed by the C command handler
    /// from the MCU's clock frequency (e.g. `freq / 20` for a 50 ms window).
    /// Zero means no Software sources are present and the field is ignored.
    pub grant_ticks: u64,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ArmStatus {
    Armed,
    AlreadyTripped,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ArmError {
    Busy,
    EmptySources,
    TooManySources,
    InvalidSampleN,
    TooManySteppers,
    EmptySteppers,
    InvalidVelocityAxis,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum DisarmStatus {
    Disarmed,
    AlreadyTripped,
    Unknown,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum TripAction {
    Continue,
    AbortNow,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct StepperSnapshot {
    pub oid: u8,
    pub step_count: i32,
}

impl StepperSnapshot {
    const EMPTY: Self = Self {
        oid: 0,
        step_count: 0,
    };
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct TripEvent {
    pub arm_id: u32,
    pub trip_clock: u64,
    pub trip_source_idx: u8,
    pub stepper_count: u8,
    pub steppers: [StepperSnapshot; MAX_STEPPERS],
}

static ARM: Arm = Arm::new();
static TRIP_EVENT_QUEUED: AtomicBool = AtomicBool::new(false);
static PIN_LEVELS: [AtomicBool; MAX_GPIO_PINS] = [const { AtomicBool::new(false) }; MAX_GPIO_PINS];

#[cfg(test)]
static TEST_MUTEX: std::sync::Mutex<()> = std::sync::Mutex::new(());

pub fn set_pin_level(gpio: PinId, pin_high: bool) -> bool {
    let idx = usize::from(gpio);
    if let Some(pin) = PIN_LEVELS.get(idx) {
        pin.store(pin_high, Ordering::Release);
        true
    } else {
        false
    }
}

#[cfg(test)]
pub(crate) fn test_guard() -> std::sync::MutexGuard<'static, ()> {
    let guard = match TEST_MUTEX.lock() {
        Ok(guard) => guard,
        Err(poisoned) => poisoned.into_inner(),
    };
    reset_for_test();
    guard
}

#[cfg(test)]
fn reset_for_test() {
    ARM.state.store(ArmState::Idle as u8, Ordering::Release);
    ARM.arm_id.store(0, Ordering::Release);
    ARM.source_count.store(0, Ordering::Release);
    ARM.stepper_count.store(0, Ordering::Release);
    ARM.snapshot.version.store(0, Ordering::Release);
    ARM.snapshot.step_count_count.store(0, Ordering::Release);
    ARM.deadline_active.store(false, Ordering::Release);
    ARM.deadline_version.store(0, Ordering::Release);
    ARM.deadline_clock_lo.store(0, Ordering::Release);
    ARM.deadline_clock_hi.store(0, Ordering::Release);
    ARM.grant_ticks_lo.store(0, Ordering::Release);
    ARM.grant_ticks_hi.store(0, Ordering::Release);
    TRIP_EVENT_QUEUED.store(false, Ordering::Release);
    for src in &ARM.sources {
        src.reset_latches();
    }
    for pin in &PIN_LEVELS {
        pin.store(false, Ordering::Release);
    }
}

pub fn arm(msg: ArmMsg) -> Result<ArmStatus, ArmError> {
    validate_arm_msg(&msg)?;

    let state = ARM.state.load(Ordering::Acquire);
    if matches_u8(state, ArmState::Armed)
        || matches_u8(state, ArmState::Tripping)
    {
        return Err(ArmError::Busy);
    }

    ARM.state.store(ArmState::Idle as u8, Ordering::Release);
    TRIP_EVENT_QUEUED.store(false, Ordering::Release);
    ARM.arm_id.store(msg.arm_id, Ordering::Release);
    ARM.arm_clock_lo
        .store(msg.arm_clock as u32, Ordering::Release);
    ARM.arm_clock_hi
        .store((msg.arm_clock >> 32) as u32, Ordering::Release);

    let source_count = usize::from(msg.source_count);
    for (slot, cfg) in ARM
        .sources
        .iter()
        .zip(msg.sources.iter())
        .take(source_count)
    {
        slot.configure(*cfg);
    }
    for slot in ARM.sources.iter().skip(source_count) {
        slot.reset_latches();
    }
    ARM.source_count.store(msg.source_count, Ordering::Release);

    let stepper_count = usize::from(msg.stepper_count);
    for (slot, oid) in ARM
        .stepper_oids
        .iter()
        .zip(msg.stepper_oids.iter())
        .take(stepper_count)
    {
        slot.store(*oid, Ordering::Release);
    }
    ARM.stepper_count
        .store(msg.stepper_count, Ordering::Release);
    ARM.snapshot.version.store(0, Ordering::Release);
    ARM.snapshot.step_count_count.store(0, Ordering::Release);

    // Initialise Software-source deadline state.
    ARM.deadline_active.store(false, Ordering::Release);
    ARM.deadline_version.store(0, Ordering::Release);
    ARM.deadline_clock_lo.store(0, Ordering::Release);
    ARM.deadline_clock_hi.store(0, Ordering::Release);
    ARM.store_grant_ticks(msg.grant_ticks);

    ARM.state.store(ArmState::Armed as u8, Ordering::Release);

    // Synchronous AlreadyTripped: if any TripImmediately source is
    // already asserted at arm time, publish a snapshot immediately and
    // return AlreadyTripped so the host can complete the homing terminal
    // synchronously without waiting for the first ISR tick.
    let source_count = usize::from(msg.source_count);
    for (idx, cfg) in msg.sources.iter().take(source_count).enumerate() {
        if cfg.policy != ArmPolicy::TripImmediately {
            continue;
        }
        let pin_high = read_pin(cfg.gpio);
        let asserted = if cfg.active_high { pin_high } else { !pin_high };
        if asserted {
            // Transition to Tripping → TrippedReady.
            ARM.state
                .store(ArmState::Tripping as u8, Ordering::Release);
            // Publish snapshot with arm_clock as the trip clock (no
            // actual MCU tick yet; best-effort timestamp).
            let empty_counts: &[i32] = &[];
            publish_snapshot(msg.arm_clock, idx as u8, empty_counts);
            ARM.state
                .store(ArmState::TrippedReady as u8, Ordering::Release);
            TRIP_EVENT_QUEUED.store(true, Ordering::Release);
            return Ok(ArmStatus::AlreadyTripped);
        }
    }

    Ok(ArmStatus::Armed)
}

pub fn disarm(arm_id: u32) -> DisarmStatus {
    if ARM.arm_id.load(Ordering::Acquire) != arm_id {
        return DisarmStatus::Unknown;
    }

    match ARM.state.compare_exchange(
        ArmState::Armed as u8,
        ArmState::Disarmed as u8,
        Ordering::AcqRel,
        Ordering::Acquire,
    ) {
        Ok(_) => DisarmStatus::Disarmed,
        Err(state)
            if matches_u8(state, ArmState::Tripping)
                || matches_u8(state, ArmState::TrippedReady)
                || matches_u8(state, ArmState::TrippedSent) =>
        {
            DisarmStatus::AlreadyTripped
        }
        Err(state) if matches_u8(state, ArmState::Disarmed) => DisarmStatus::Disarmed,
        Err(_) => DisarmStatus::Unknown,
    }
}

pub fn tick(clock: u64, v_per_axis_q16: [u32; 3], stepper_counts: &[i32]) -> TripAction {
    if !matches_u8(ARM.state.load(Ordering::Acquire), ArmState::Armed) {
        return TripAction::Continue;
    }
    if clock < ARM.arm_clock() {
        return TripAction::Continue;
    }

    let source_count = usize::from(ARM.source_count.load(Ordering::Acquire));
    for (idx, src) in ARM.sources.iter().take(source_count).enumerate() {
        // Software sources have no GPIO pin: skip the GPIO polling loop
        // entirely and handle them via the deadline check below.
        if src.kind.load(Ordering::Acquire) == SourceKind::Software as u8 {
            continue;
        }

        let gpio = src.gpio.load(Ordering::Acquire);
        let pin_high = read_pin(gpio);
        let active_high = src.active_high.load(Ordering::Acquire);
        let asserted = if active_high { pin_high } else { !pin_high };
        // Decode the policy byte. An unrecognised value (would require a
        // wire-corruption or future firmware-vs-host version skew) maps
        // conservatively to `TripImmediately` — that matches the previous
        // implicit fall-through behaviour (the old `else if !asserted`
        // arm) without depending on raw-discriminant comparisons.
        let policy = ArmPolicy::try_from(src.policy.load(Ordering::Acquire))
            .unwrap_or(ArmPolicy::TripImmediately);

        match policy {
            ArmPolicy::IgnoreUntilMoving => {
                let axis = VelocityAxis::from_bits_truncate(
                    src.velocity_axis.load(Ordering::Acquire),
                );
                let v_sel = max_axis_velocity(v_per_axis_q16, axis);
                if !src.moved_above_v.load(Ordering::Acquire)
                    && v_sel >= src.v_min_q16.load(Ordering::Acquire)
                {
                    src.moved_above_v.store(true, Ordering::Release);
                }
                if !src.moved_above_v.load(Ordering::Acquire) {
                    src.sample_acc.store(0, Ordering::Release);
                    continue;
                }
                if !asserted {
                    src.cleared.store(true, Ordering::Release);
                    src.sample_acc.store(0, Ordering::Release);
                    continue;
                }
                if !src.cleared.load(Ordering::Acquire) {
                    src.sample_acc.store(0, Ordering::Release);
                    continue;
                }
            }
            ArmPolicy::WaitForClear => {
                if !asserted {
                    src.cleared.store(true, Ordering::Release);
                    src.sample_acc.store(0, Ordering::Release);
                    continue;
                }
                if !src.cleared.load(Ordering::Acquire) {
                    src.sample_acc.store(0, Ordering::Release);
                    continue;
                }
            }
            ArmPolicy::TripImmediately => {
                if !asserted {
                    src.sample_acc.store(0, Ordering::Release);
                    continue;
                }
            }
        }

        let sample_acc = src.sample_acc.load(Ordering::Acquire).saturating_add(1);
        src.sample_acc.store(sample_acc, Ordering::Release);
        if sample_acc < src.sample_n.load(Ordering::Acquire) {
            continue;
        }

        if ARM
            .state
            .compare_exchange(
                ArmState::Armed as u8,
                ArmState::Tripping as u8,
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_err()
        {
            return TripAction::Continue;
        }

        publish_snapshot(clock, idx as u8, stepper_counts);
        ARM.state
            .store(ArmState::TrippedReady as u8, Ordering::Release);
        TRIP_EVENT_QUEUED.store(true, Ordering::Release);
        return TripAction::AbortNow;
    }

    tick_software_deadline(clock, stepper_counts)
}

/// Check (or open) the Software-source deadline window.
///
/// Called at the end of every [`tick`] when the arm is in the `Armed` state
/// and has passed `arm_clock`. Handles two sub-cases:
///
/// - `deadline_active == false`: first tick past `arm_clock`; opens the
///   initial window by writing `deadline_clock = clock + grant_ticks`.
/// - `deadline_active == true && clock >= deadline_clock`: window expired;
///   transitions `Armed → Tripping → TrippedReady` and returns
///   [`TripAction::AbortNow`].
fn tick_software_deadline(clock: u64, stepper_counts: &[i32]) -> TripAction {
    if !ARM.has_software_source() {
        return TripAction::Continue;
    }
    if !ARM.deadline_active.load(Ordering::Acquire) {
        // First tick past arm_clock: open the initial window.
        let grant = ARM.grant_ticks();
        ARM.store_deadline_clock_seqlocked(clock.saturating_add(grant));
        ARM.deadline_active.store(true, Ordering::Release);
        return TripAction::Continue;
    }
    let deadline = match ARM.try_read_deadline_clock() {
        Some(d) => d,
        None => return TripAction::Continue,
    };
    if clock < deadline {
        return TripAction::Continue;
    }
    // Deadline expired: attempt to freeze the segment.
    if ARM
        .state
        .compare_exchange(
            ArmState::Armed as u8,
            ArmState::Tripping as u8,
            Ordering::AcqRel,
            Ordering::Acquire,
        )
        .is_ok()
    {
        publish_snapshot(clock, TRIP_SOURCE_DEADLINE_EXPIRED, stepper_counts);
        ARM.state
            .store(ArmState::TrippedReady as u8, Ordering::Release);
        TRIP_EVENT_QUEUED.store(true, Ordering::Release);
        return TripAction::AbortNow;
    }
    TripAction::Continue
}

/// Result type returned by [`software_trip`].
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum TripResult {
    /// The arm was in the `Armed` state and has been transitioned to
    /// `TrippedReady`. A `TripEvent` is now available via [`poll_trip`].
    Tripped,
    /// The arm was not in the `Armed` state (already tripped, disarmed,
    /// idle, …). The call is a no-op.
    NotArmed,
    /// The provided `arm_id` does not match the currently-armed slot.
    WrongArmId,
}

/// Programmatically trip the currently-armed endstop from a C command
/// handler (i.e. in response to the host sending a `runtime_software_trip`
/// command).
///
/// `clock` is the current MCU clock value at call time (read via
/// `timer_read_time()` in the C command handler).
pub fn software_trip(arm_id: u32, clock: u64, stepper_counts: &[i32]) -> TripResult {
    if ARM.arm_id.load(Ordering::Acquire) != arm_id {
        return TripResult::WrongArmId;
    }

    match ARM.state.compare_exchange(
        ArmState::Armed as u8,
        ArmState::Tripping as u8,
        Ordering::AcqRel,
        Ordering::Acquire,
    ) {
        Ok(_) => {}
        Err(_) => return TripResult::NotArmed,
    }

    publish_snapshot(clock, TRIP_SOURCE_SOFTWARE, stepper_counts);
    ARM.state
        .store(ArmState::TrippedReady as u8, Ordering::Release);
    TRIP_EVENT_QUEUED.store(true, Ordering::Release);
    TripResult::Tripped
}

/// Extend the Software-source deadline by one grant window from `clock`.
///
/// Called from the C command handler for `runtime_extend_deadline` when the
/// host confirms the probe has not yet triggered and wants to keep the MCU
/// segment running. Silently ignores calls when:
/// - `arm_id` does not match the active arm, or
/// - the deadline is not currently active (arm was never ticked past
///   `arm_clock`, or the arm is already tripped/disarmed).
///
/// `clock` is the current MCU clock value at call time.
pub fn extend_deadline(arm_id: u32, clock: u64) {
    // Reject stale or mismatched calls.
    if ARM.arm_id.load(Ordering::Acquire) != arm_id {
        return;
    }
    if !matches_u8(ARM.state.load(Ordering::Acquire), ArmState::Armed) {
        return;
    }
    if !ARM.deadline_active.load(Ordering::Acquire) {
        return;
    }
    let grant = ARM.grant_ticks();
    ARM.store_deadline_clock_seqlocked(clock.saturating_add(grant));
}

pub fn poll_trip() -> Option<TripEvent> {
    if !TRIP_EVENT_QUEUED.swap(false, Ordering::AcqRel) {
        return None;
    }
    if !matches_u8(ARM.state.load(Ordering::Acquire), ArmState::TrippedReady) {
        return None;
    }

    loop {
        let version_begin = ARM.snapshot.version.load(Ordering::Acquire);
        if version_begin & 1 != 0 {
            core::hint::spin_loop();
            continue;
        }

        let arm_id = ARM.arm_id.load(Ordering::Acquire);
        let lo = u64::from(ARM.snapshot.trip_clock_lo.load(Ordering::Acquire));
        let hi = u64::from(ARM.snapshot.trip_clock_hi.load(Ordering::Acquire));
        let trip_source_idx = ARM.snapshot.trip_source_idx.load(Ordering::Acquire);
        let stepper_count = ARM.snapshot.step_count_count.load(Ordering::Acquire);
        let mut steppers = [StepperSnapshot::EMPTY; MAX_STEPPERS];

        for (dst, (oid, count)) in steppers.iter_mut().zip(
            ARM.snapshot
                .stepper_oids
                .iter()
                .zip(ARM.snapshot.step_counts.iter()),
        ) {
            *dst = StepperSnapshot {
                oid: oid.load(Ordering::Acquire),
                step_count: count.load(Ordering::Acquire),
            };
        }

        let version_end = ARM.snapshot.version.load(Ordering::Acquire);
        if version_begin == version_end {
            ARM.state
                .store(ArmState::TrippedSent as u8, Ordering::Release);
            return Some(TripEvent {
                arm_id,
                trip_clock: (hi << 32) | lo,
                trip_source_idx,
                stepper_count,
                steppers,
            });
        }
        core::hint::spin_loop();
    }
}

fn validate_arm_msg(msg: &ArmMsg) -> Result<(), ArmError> {
    if msg.source_count == 0 {
        return Err(ArmError::EmptySources);
    }
    if usize::from(msg.source_count) > MAX_SOURCES {
        return Err(ArmError::TooManySources);
    }
    if msg.stepper_count == 0 {
        return Err(ArmError::EmptySteppers);
    }
    if usize::from(msg.stepper_count) > MAX_STEPPERS {
        return Err(ArmError::TooManySteppers);
    }

    for cfg in msg.sources.iter().take(usize::from(msg.source_count)) {
        if cfg.sample_n == 0 || cfg.sample_n > 8 {
            return Err(ArmError::InvalidSampleN);
        }
        if cfg.policy == ArmPolicy::IgnoreUntilMoving && cfg.velocity_axis.bits() == 0 {
            return Err(ArmError::InvalidVelocityAxis);
        }
    }
    Ok(())
}

fn read_pin(gpio: PinId) -> bool {
    PIN_LEVELS
        .get(usize::from(gpio))
        .is_some_and(|pin| pin.load(Ordering::Acquire))
}

fn max_axis_velocity(v_per_axis_q16: [u32; 3], axis: VelocityAxis) -> u32 {
    let mut v = 0;
    for (value, axis_bit) in
        v_per_axis_q16
            .into_iter()
            .zip([VelocityAxis::X, VelocityAxis::Y, VelocityAxis::Z])
    {
        if axis.intersects(axis_bit) {
            v = v.max(value);
        }
    }
    v
}

fn publish_snapshot(clock: u64, source_idx: u8, stepper_counts: &[i32]) {
    let version = ARM.snapshot.version.load(Ordering::Acquire);
    let odd = version | 1;
    ARM.snapshot.version.store(odd, Ordering::Release);
    ARM.snapshot
        .trip_clock_lo
        .store(clock as u32, Ordering::Release);
    ARM.snapshot
        .trip_clock_hi
        .store((clock >> 32) as u32, Ordering::Release);
    ARM.snapshot
        .trip_source_idx
        .store(source_idx, Ordering::Release);

    let count = core::cmp::min(
        usize::from(ARM.stepper_count.load(Ordering::Acquire)),
        MAX_STEPPERS,
    );
    for (dst_oid, oid) in ARM
        .snapshot
        .stepper_oids
        .iter()
        .zip(ARM.stepper_oids.iter())
        .take(count)
    {
        dst_oid.store(oid.load(Ordering::Acquire), Ordering::Release);
    }
    for (dst_count, oid) in ARM
        .snapshot
        .step_counts
        .iter()
        .zip(ARM.stepper_oids.iter())
        .take(count)
    {
        let idx = usize::from(oid.load(Ordering::Acquire));
        let count_value = stepper_counts.get(idx).copied().unwrap_or(0);
        dst_count.store(count_value, Ordering::Release);
    }
    ARM.snapshot
        .step_count_count
        .store(count as u8, Ordering::Release);
    ARM.snapshot
        .version
        .store(odd.wrapping_add(1), Ordering::Release);
}

const fn matches_u8(value: u8, state: ArmState) -> bool {
    value == state as u8
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
mod tests {
    use super::*;

    const V_MIN: u32 = 10 << 16;

    fn cfg(kind: SourceKind, policy: ArmPolicy, sample_n: u8, gpio: PinId) -> SourceConfig {
        SourceConfig {
            kind,
            gpio,
            active_high: true,
            policy,
            sample_n,
            velocity_axis: VelocityAxis::X,
            v_min_q16: V_MIN,
        }
    }

    fn msg(source: SourceConfig) -> ArmMsg {
        let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
        sources[0] = source;
        ArmMsg {
            arm_id: 42,
            arm_clock: 0,
            source_count: 1,
            sources,
            stepper_count: 2,
            stepper_oids: [0, 1, 0, 0, 0, 0, 0, 0],
            grant_ticks: 0,
        }
    }

    /// Build a Software-source arm message with the given `grant_ticks`.
    fn sw_msg(grant_ticks: u64) -> ArmMsg {
        let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
        sources[0] = SourceConfig {
            kind: SourceKind::Software,
            gpio: 0,
            active_high: true,
            policy: ArmPolicy::TripImmediately,
            sample_n: 1,
            velocity_axis: VelocityAxis::XYZ,
            v_min_q16: 0,
        };
        ArmMsg {
            arm_id: 42,
            arm_clock: 0,
            source_count: 1,
            sources,
            stepper_count: 2,
            stepper_oids: [0, 1, 0, 0, 0, 0, 0, 0],
            grant_ticks,
        }
    }

    fn reset() -> std::sync::MutexGuard<'static, ()> {
        test_guard()
    }

    fn drain_trip() -> TripEvent {
        poll_trip().expect("trip event")
    }

    #[test]
    fn source_policy_sample_matrix() {
        for kind in [SourceKind::Physical, SourceKind::TmcDiag] {
            for policy in [
                ArmPolicy::TripImmediately,
                ArmPolicy::WaitForClear,
                ArmPolicy::IgnoreUntilMoving,
            ] {
                for sample_n in [1, 3] {
                    let _guard = reset();
                    let source = cfg(kind, policy, sample_n, 1);
                    arm(msg(source)).expect("arm");
                    set_pin_level(1, true);
                    if policy == ArmPolicy::WaitForClear {
                        assert_eq!(tick(1, [V_MIN, 0, 0], &[10, 20]), TripAction::Continue);
                        set_pin_level(1, false);
                        assert_eq!(tick(2, [V_MIN, 0, 0], &[10, 20]), TripAction::Continue);
                        set_pin_level(1, true);
                    } else if policy == ArmPolicy::IgnoreUntilMoving {
                        assert_eq!(tick(1, [V_MIN, 0, 0], &[10, 20]), TripAction::Continue);
                        set_pin_level(1, false);
                        assert_eq!(tick(2, [V_MIN, 0, 0], &[10, 20]), TripAction::Continue);
                        set_pin_level(1, true);
                    }

                    for i in 1..=sample_n {
                        let action = tick(10 + u64::from(i), [V_MIN, 0, 0], &[10, 20]);
                        if i < sample_n {
                            assert_eq!(action, TripAction::Continue);
                        } else {
                            assert_eq!(action, TripAction::AbortNow);
                            let evt = drain_trip();
                            assert_eq!(evt.trip_source_idx, 0);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn ignore_until_moving_latch_requires_velocity_then_clear_once() {
        let _guard = reset();
        arm(msg(cfg(
            SourceKind::TmcDiag,
            ArmPolicy::IgnoreUntilMoving,
            1,
            2,
        )))
        .expect("arm");

        set_pin_level(2, true);
        assert_eq!(tick(1, [V_MIN - 1, 0, 0], &[1]), TripAction::Continue);
        assert_eq!(tick(2, [V_MIN, 0, 0], &[1]), TripAction::Continue);
        set_pin_level(2, false);
        assert_eq!(tick(3, [V_MIN, 0, 0], &[1]), TripAction::Continue);
        set_pin_level(2, true);
        assert_eq!(tick(4, [V_MIN, 0, 0], &[1]), TripAction::AbortNow);
        assert_eq!(drain_trip().trip_clock, 4);

        reset_for_test();
        arm(msg(cfg(
            SourceKind::TmcDiag,
            ArmPolicy::IgnoreUntilMoving,
            1,
            2,
        )))
        .expect("arm");
        set_pin_level(2, false);
        assert_eq!(tick(1, [V_MIN, 0, 0], &[1]), TripAction::Continue);
        set_pin_level(2, true);
        assert_eq!(tick(2, [0, 0, 0], &[1]), TripAction::AbortNow);
    }

    #[test]
    fn wait_for_clear_ignores_assertion_at_arm() {
        let _guard = reset();
        arm(msg(cfg(
            SourceKind::Physical,
            ArmPolicy::WaitForClear,
            1,
            3,
        )))
        .expect("arm");
        set_pin_level(3, true);
        assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
        set_pin_level(3, false);
        assert_eq!(tick(2, [0, 0, 0], &[1]), TripAction::Continue);
        set_pin_level(3, true);
        assert_eq!(tick(3, [0, 0, 0], &[1]), TripAction::AbortNow);
    }

    #[test]
    fn trip_immediately_assertion_at_arm_trips_on_first_sample() {
        let _guard = reset();
        arm(msg(cfg(
            SourceKind::Physical,
            ArmPolicy::TripImmediately,
            1,
            4,
        )))
        .expect("arm");
        set_pin_level(4, true);
        assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::AbortNow);
    }

    #[test]
    fn arm_policy_try_from_decodes_known_variants_and_rejects_others() {
        assert_eq!(ArmPolicy::try_from(0).unwrap(), ArmPolicy::TripImmediately);
        assert_eq!(ArmPolicy::try_from(1).unwrap(), ArmPolicy::WaitForClear);
        assert_eq!(ArmPolicy::try_from(2).unwrap(), ArmPolicy::IgnoreUntilMoving);
        assert_eq!(ArmPolicy::try_from(3).unwrap_err(), 3);
        assert_eq!(ArmPolicy::try_from(255).unwrap_err(), 255);
    }

    #[test]
    fn unknown_policy_byte_falls_back_to_trip_immediately_behavior() {
        // Defensive: if a wire-corruption or version-skew ever planted a
        // non-{0,1,2} value into the policy atomic, the decoded fallback
        // is `TripImmediately` — same observable behavior as setting
        // policy to 0 explicitly: trip when asserted, no-op otherwise.
        let _guard = reset();
        arm(msg(cfg(
            SourceKind::Physical,
            ArmPolicy::TripImmediately,
            1,
            4,
        )))
        .expect("arm");
        // Plant a bogus byte directly into the source's policy atomic.
        ARM.sources[0].policy.store(99, Ordering::Release);
        set_pin_level(4, true);
        assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::AbortNow);
    }

    #[test]
    fn multi_source_or_reports_first_asserted_source_index() {
        let _guard = reset();
        let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
        sources[0] = cfg(SourceKind::Physical, ArmPolicy::TripImmediately, 1, 5);
        sources[1] = cfg(SourceKind::Physical, ArmPolicy::TripImmediately, 1, 6);
        arm(ArmMsg {
            arm_id: 77,
            arm_clock: 0,
            source_count: 2,
            sources,
            stepper_count: 2,
            stepper_oids: [0, 1, 0, 0, 0, 0, 0, 0],
            grant_ticks: 0,
        })
        .expect("arm");
        set_pin_level(6, true);
        assert_eq!(tick(1, [0, 0, 0], &[100, -200]), TripAction::AbortNow);
        let evt = drain_trip();
        assert_eq!(evt.arm_id, 77);
        assert_eq!(evt.trip_source_idx, 1);
        assert_eq!(evt.stepper_count, 2);
        assert_eq!(evt.steppers[0].oid, 0);
        assert_eq!(evt.steppers[0].step_count, 100);
        assert_eq!(evt.steppers[1].oid, 1);
        assert_eq!(evt.steppers[1].step_count, -200);
    }

    #[test]
    fn future_arm_clock_ignores_early_assertions() {
        let _guard = reset();
        let mut m = msg(cfg(SourceKind::Physical, ArmPolicy::TripImmediately, 1, 7));
        m.arm_clock = 50;
        arm(m).expect("arm");
        set_pin_level(7, true);
        assert_eq!(tick(49, [0, 0, 0], &[1]), TripAction::Continue);
        assert!(poll_trip().is_none());
        assert_eq!(tick(50, [0, 0, 0], &[2]), TripAction::AbortNow);
        assert_eq!(drain_trip().trip_clock, 50);
    }

    #[test]
    fn trip_never_fires_while_state_is_not_armed() {
        let _guard = reset();
        set_pin_level(8, true);
        for state in [
            ArmState::Idle,
            ArmState::Tripping,
            ArmState::TrippedReady,
            ArmState::TrippedSent,
            ArmState::Disarmed,
        ] {
            ARM.state.store(state as u8, Ordering::Release);
            assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
        }
    }

    #[test]
    fn exactly_one_terminal_for_trip_vs_disarm_schedules() {
        let _guard = reset();
        arm(msg(cfg(
            SourceKind::Physical,
            ArmPolicy::TripImmediately,
            1,
            9,
        )))
        .expect("arm");
        set_pin_level(9, true);

        let disarm_first = disarm(42);
        assert_eq!(disarm_first, DisarmStatus::Disarmed);
        assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
        assert!(poll_trip().is_none());

        reset_for_test();
        arm(msg(cfg(
            SourceKind::Physical,
            ArmPolicy::TripImmediately,
            1,
            9,
        )))
        .expect("arm");
        set_pin_level(9, true);
        assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::AbortNow);
        assert_eq!(disarm(42), DisarmStatus::AlreadyTripped);
        assert!(poll_trip().is_some());
    }

    #[test]
    fn snapshot_seqlock_reader_retries_odd_and_never_returns_torn_read() {
        let _guard = reset();
        arm(msg(cfg(
            SourceKind::Physical,
            ArmPolicy::TripImmediately,
            1,
            10,
        )))
        .expect("arm");
        set_pin_level(10, true);
        assert_eq!(
            tick(0x1_0000_0002, [0, 0, 0], &[123, 456]),
            TripAction::AbortNow
        );
        let evt = drain_trip();
        assert_eq!(evt.trip_clock, 0x1_0000_0002);
        assert_eq!(evt.steppers[0].step_count, 123);
        assert_eq!(evt.steppers[1].step_count, 456);
    }

    #[test]
    fn active_low_polarity_uses_explicit_branch_not_xor() {
        let _guard = reset();
        let mut source = cfg(SourceKind::Physical, ArmPolicy::TripImmediately, 1, 11);
        source.active_high = false;
        // Active-low: HIGH = not asserted, LOW = asserted.
        // Set pin HIGH before arming so arm() does not see an asserted
        // pin and immediately return AlreadyTripped.
        set_pin_level(11, true);
        arm(msg(source)).expect("arm");
        assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
        set_pin_level(11, false);
        assert_eq!(tick(2, [0, 0, 0], &[1]), TripAction::AbortNow);
    }

    #[test]
    fn already_tripped_at_arm_time_active_high() {
        // TripImmediately + pin already HIGH when arm() is called:
        // arm() should return AlreadyTripped synchronously, publish a
        // snapshot, and set state to TrippedReady so poll_trip() works.
        let _guard = reset();
        set_pin_level(12, true);
        let result = arm(msg(cfg(
            SourceKind::Physical,
            ArmPolicy::TripImmediately,
            1,
            12,
        )));
        assert_eq!(result, Ok(ArmStatus::AlreadyTripped));
        // State should be TrippedReady; poll_trip() must return Some.
        let evt = poll_trip().expect("trip event after AlreadyTripped");
        assert_eq!(evt.arm_id, 42);
        assert_eq!(evt.trip_source_idx, 0);
        // No further ticks should trip again.
        assert_eq!(tick(1, [0, 0, 0], &[1]), TripAction::Continue);
    }

    #[test]
    fn already_tripped_requires_trip_immediately_policy() {
        // WaitForClear source with pin HIGH at arm time must NOT return
        // AlreadyTripped — the policy requires a clear-then-assert cycle.
        let _guard = reset();
        set_pin_level(13, true);
        let result = arm(msg(cfg(
            SourceKind::Physical,
            ArmPolicy::WaitForClear,
            1,
            13,
        )));
        assert_eq!(result, Ok(ArmStatus::Armed));
    }

    // --- Software source tests ---

    #[test]
    fn software_source_does_not_trip_on_gpio() {
        // A Software source must not read or respond to GPIO levels.
        let _guard = reset();
        arm(sw_msg(1000)).expect("arm");
        // Set every pin high — a Physical source would trip immediately.
        for i in 0..20_u16 {
            set_pin_level(i, true);
        }
        // First tick opens the deadline window; must NOT trip on GPIO.
        assert_eq!(tick(1, [0, 0, 0], &[1, 2]), TripAction::Continue);
        // deadline_active should now be set.
        assert!(ARM.deadline_active.load(Ordering::Acquire));
    }

    #[test]
    fn software_source_deadline_expires_and_trips() {
        // grant_ticks = 100; arm_clock = 0.
        // tick(1)   → opens window: deadline = 1 + 100 = 101. Continue.
        // tick(101) → clock == deadline → AbortNow with DEADLINE_EXPIRED idx.
        let _guard = reset();
        arm(sw_msg(100)).expect("arm");
        assert_eq!(tick(1, [0, 0, 0], &[10, 20]), TripAction::Continue);
        assert!(ARM.deadline_active.load(Ordering::Acquire));
        // Clock 100 is still inside the window.
        assert_eq!(tick(100, [0, 0, 0], &[10, 20]), TripAction::Continue);
        // Clock 101 is at the deadline — should trip.
        assert_eq!(tick(101, [0, 0, 0], &[10, 20]), TripAction::AbortNow);
        let evt = drain_trip();
        assert_eq!(evt.arm_id, 42);
        assert_eq!(evt.trip_source_idx, TRIP_SOURCE_DEADLINE_EXPIRED);
        assert_eq!(evt.trip_clock, 101);
    }

    #[test]
    fn extend_deadline_pushes_window_forward() {
        // grant_ticks = 100.
        // tick(1)   → deadline = 101. Continue.
        // extend_deadline at clock=50 → deadline = 50 + 100 = 150.
        // tick(101) → inside new window. Continue.
        // tick(150) → at new deadline. AbortNow.
        let _guard = reset();
        arm(sw_msg(100)).expect("arm");
        assert_eq!(tick(1, [0, 0, 0], &[]), TripAction::Continue);
        extend_deadline(42, 50);
        assert_eq!(ARM.deadline_clock_unchecked(), 150);
        assert_eq!(tick(101, [0, 0, 0], &[]), TripAction::Continue);
        assert_eq!(tick(150, [0, 0, 0], &[]), TripAction::AbortNow);
        assert_eq!(drain_trip().trip_source_idx, TRIP_SOURCE_DEADLINE_EXPIRED);
    }

    #[test]
    fn extend_deadline_ignored_for_wrong_arm_id() {
        let _guard = reset();
        arm(sw_msg(100)).expect("arm");
        assert_eq!(tick(1, [0, 0, 0], &[]), TripAction::Continue);
        let deadline_before = ARM.deadline_clock_unchecked();
        extend_deadline(99, 50); // wrong arm_id
        assert_eq!(ARM.deadline_clock_unchecked(), deadline_before);
    }

    #[test]
    fn extend_deadline_ignored_before_first_tick() {
        // Before the first tick, deadline_active = false.
        // extend_deadline should silently ignore.
        let _guard = reset();
        arm(sw_msg(100)).expect("arm");
        assert!(!ARM.deadline_active.load(Ordering::Acquire));
        extend_deadline(42, 50); // deadline_active is false → no-op
        assert!(!ARM.deadline_active.load(Ordering::Acquire));
        assert_eq!(ARM.deadline_clock_unchecked(), 0);
    }

    #[test]
    fn software_trip_transitions_armed_to_tripped_ready() {
        let _guard = reset();
        arm(sw_msg(10_000)).expect("arm");
        assert_eq!(
            software_trip(42, 500, &[10, 20]),
            TripResult::Tripped
        );
        let evt = drain_trip();
        assert_eq!(evt.arm_id, 42);
        assert_eq!(evt.trip_source_idx, TRIP_SOURCE_SOFTWARE);
        assert_eq!(evt.trip_clock, 500);
    }

    #[test]
    fn software_trip_wrong_arm_id_is_no_op() {
        let _guard = reset();
        arm(sw_msg(10_000)).expect("arm");
        assert_eq!(
            software_trip(99, 500, &[10, 20]),
            TripResult::WrongArmId
        );
        // Still armed.
        assert!(matches_u8(
            ARM.state.load(Ordering::Acquire),
            ArmState::Armed
        ));
    }

    #[test]
    fn software_trip_on_non_armed_state_is_not_armed() {
        let _guard = reset();
        // Set arm_id to 0 so it matches the reset state, then put the state
        // into Disarmed. software_trip must return NotArmed (state check
        // fails) rather than WrongArmId (arm_id check fails).
        ARM.arm_id.store(0, Ordering::Release);
        ARM.state.store(ArmState::Disarmed as u8, Ordering::Release);
        assert_eq!(
            software_trip(0, 500, &[]),
            TripResult::NotArmed
        );
    }

    #[test]
    fn software_trip_idempotent_second_call_returns_not_armed() {
        let _guard = reset();
        arm(sw_msg(10_000)).expect("arm");
        assert_eq!(software_trip(42, 1, &[]), TripResult::Tripped);
        // State is now TrippedReady; a second call must return NotArmed.
        assert_eq!(software_trip(42, 2, &[]), TripResult::NotArmed);
    }

    #[test]
    fn deadline_active_false_resets_across_arm_calls() {
        // Arm with Software source, open deadline, then re-arm.
        // On the new arm, deadline_active must be false again.
        let _guard = reset();
        arm(sw_msg(100)).expect("arm");
        tick(1, [0, 0, 0], &[]);
        assert!(ARM.deadline_active.load(Ordering::Acquire));
        // Disarm so we can re-arm.
        disarm(42);
        arm(sw_msg(100)).expect("arm");
        assert!(
            !ARM.deadline_active.load(Ordering::Acquire),
            "deadline_active must be cleared on re-arm"
        );
    }

    #[test]
    fn software_source_deadline_uses_saturating_add() {
        // grant_ticks = u64::MAX → deadline = clock.saturating_add(u64::MAX)
        // = u64::MAX (saturates). That deadline will never be reached in
        // practice, but the arithmetic must not overflow/panic.
        let _guard = reset();
        arm(sw_msg(u64::MAX)).expect("arm");
        assert_eq!(tick(1, [0, 0, 0], &[]), TripAction::Continue);
        assert_eq!(ARM.deadline_clock_unchecked(), u64::MAX);
    }

    #[test]
    fn software_source_skips_gpio_no_gpio_trip() {
        // Mixed arm: Software source at index 0, Physical at index 1.
        // Pin for Physical (gpio=15) is deasserted; no GPIO trip expected.
        // Deadline with large grant: arm never expires. Should stay Continue.
        let _guard = reset();
        let mut sources = [SourceConfig::EMPTY; MAX_SOURCES];
        sources[0] = SourceConfig {
            kind: SourceKind::Software,
            gpio: 0,
            active_high: true,
            policy: ArmPolicy::TripImmediately,
            sample_n: 1,
            velocity_axis: VelocityAxis::XYZ,
            v_min_q16: 0,
        };
        sources[1] = cfg(SourceKind::Physical, ArmPolicy::TripImmediately, 1, 15);
        arm(ArmMsg {
            arm_id: 42,
            arm_clock: 0,
            source_count: 2,
            sources,
            stepper_count: 2,
            stepper_oids: [0, 1, 0, 0, 0, 0, 0, 0],
            grant_ticks: 10_000,
        })
        .expect("arm");
        // Tick with Physical pin deasserted → Continue.
        assert_eq!(tick(1, [0, 0, 0], &[]), TripAction::Continue);
        // Assert the Physical pin → Physical source trips.
        set_pin_level(15, true);
        assert_eq!(tick(2, [0, 0, 0], &[]), TripAction::AbortNow);
        let evt = drain_trip();
        // Should be source index 1 (the Physical source), not the Software one.
        assert_eq!(evt.trip_source_idx, 1);
    }
}
