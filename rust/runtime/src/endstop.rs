// FFI log calls via kalico_log_emit mirror the fault_helpers.rs pattern.
#![allow(unsafe_code)]

// `portable_atomic` so that RMW operations (`swap` on `TRIP_EVENT_QUEUED`,
// `compare_exchange` on `ARM.state`) compile on ARMv6-M (STM32G0), which
// has no LDREX/STREX. On thumbv7em the codegen is identical to
// `core::sync::atomic`. `Ordering` stays from `core`.
use core::sync::atomic::Ordering;
use portable_atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU16, AtomicU32};

pub const MAX_SOURCES: usize = 4;
pub const MAX_STEPPERS: usize = 8;
const MAX_GPIO_PINS: usize = 256;

pub type PinId = u16;

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum SourceKind {
    Physical = 0,
    TmcDiag = 1,
    /// Software-triggered source: no GPIO pin is polled. The MCU waits for
    /// an explicit `software_trip` call to freeze the segment.
    Software = 2,
}

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
        }
    }

    fn arm_clock(&self) -> u64 {
        let lo = u64::from(self.arm_clock_lo.load(Ordering::Acquire));
        let hi = u64::from(self.arm_clock_hi.load(Ordering::Acquire));
        (hi << 32) | lo
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
    if matches_u8(state, ArmState::Armed) || matches_u8(state, ArmState::Tripping) {
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

    ARM.state.store(ArmState::Armed as u8, Ordering::Release);

    let source_count = usize::from(msg.source_count);
    for (idx, cfg) in msg.sources.iter().take(source_count).enumerate() {
        if cfg.policy != ArmPolicy::TripImmediately {
            continue;
        }
        if cfg.kind == SourceKind::Software {
            continue;
        }
        let pin_high = read_pin(cfg.gpio);
        let asserted = if cfg.active_high { pin_high } else { !pin_high };
        if asserted {
            ARM.state.store(ArmState::Tripping as u8, Ordering::Release);
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
    let state = ARM.state.load(Ordering::Acquire);
    if matches_u8(state, ArmState::TrippedReady) || matches_u8(state, ArmState::Tripping) {
        return TripAction::AbortNow;
    }
    if !matches_u8(state, ArmState::Armed) {
        return TripAction::Continue;
    }
    if clock < ARM.arm_clock() {
        return TripAction::Continue;
    }

    let source_count = usize::from(ARM.source_count.load(Ordering::Acquire));
    for (idx, src) in ARM.sources.iter().take(source_count).enumerate() {
        if src.kind.load(Ordering::Acquire) == SourceKind::Software as u8 {
            continue;
        }

        let gpio = src.gpio.load(Ordering::Acquire);
        let pin_high = read_pin(gpio);
        let active_high = src.active_high.load(Ordering::Acquire);
        let asserted = if active_high { pin_high } else { !pin_high };
        let policy = ArmPolicy::try_from(src.policy.load(Ordering::Acquire))
            .unwrap_or(ArmPolicy::TripImmediately);

        match policy {
            ArmPolicy::IgnoreUntilMoving => {
                let axis =
                    VelocityAxis::from_bits_truncate(src.velocity_axis.load(Ordering::Acquire));
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
        // DISABLED FOR TESTING: local siren. The detecting MCU intentionally
        // does NOT self-freeze here — it only reports the trip. The cross-MCU
        // relay (bridge reactor TripDispatch) sends trsync_trigger, which
        // freezes via runtime_stop_on_trigger. Suppressing the local freeze
        // lets us verify the relay on a single board. Re-enable as the
        // same-MCU fast-path once the relay is confirmed.
        // See docs/superpowers/specs/2026-05-31-trsync-cross-mcu-homing-design.md
        return TripAction::Continue;
    }

    TripAction::Continue
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum TripResult {
    Tripped,
    NotArmed,
    WrongArmId,
}

#[cfg(any(not(any(test, feature = "host")), feature = "mcu-linux"))]
unsafe extern "C" {
    fn kalico_log_emit(level: u8, subsystem: u8, event: u16, code: u16, arg0: u32, arg1: u32);
}

/// Emit `endstop.software_trip`: arg0=arm_id passed in, arg1 packs
/// `ARM.state` (bits 0..7), `ARM.arm_id & 0xFF` (bits 8..15),
/// and `TripResult` discriminant 0=Tripped/1=NotArmed/2=WrongArmId (bits 24..31).
#[inline]
fn emit_software_trip_log(arg_arm_id: u32, armed_arm_id: u32, state: u8, result: &TripResult) {
    use crate::log_codes::{EVENT_ENDSTOP_SOFTWARE_TRIP, SUBSYSTEM_ENDSTOP};
    const LOG_LEVEL_DEBUG: u8 = 1;
    let result_discriminant: u32 = match result {
        TripResult::Tripped => 0,
        TripResult::NotArmed => 1,
        TripResult::WrongArmId => 2,
    };
    let arg1 = u32::from(state)
        | ((armed_arm_id & 0xFF) << 8)
        | (result_discriminant << 24);
    #[cfg(any(not(any(test, feature = "host")), feature = "mcu-linux"))]
    // SAFETY: kalico_log_emit is a pure C logging sink; no aliasing or
    // ownership constraints on its arguments.
    unsafe {
        kalico_log_emit(
            LOG_LEVEL_DEBUG,
            SUBSYSTEM_ENDSTOP,
            EVENT_ENDSTOP_SOFTWARE_TRIP,
            0,
            arg_arm_id,
            arg1,
        );
    }
    #[cfg(not(any(not(any(test, feature = "host")), feature = "mcu-linux")))]
    {
        let _ = (arg_arm_id, arg1);
    }
}

pub fn software_trip(arm_id: u32, clock: u64, stepper_counts: &[i32]) -> TripResult {
    let armed_arm_id = ARM.arm_id.load(Ordering::Acquire);
    let state_before = ARM.state.load(Ordering::Acquire);

    if armed_arm_id != arm_id {
        let result = TripResult::WrongArmId;
        emit_software_trip_log(arm_id, armed_arm_id, state_before, &result);
        return result;
    }

    match ARM.state.compare_exchange(
        ArmState::Armed as u8,
        ArmState::Tripping as u8,
        Ordering::AcqRel,
        Ordering::Acquire,
    ) {
        Ok(_) => {}
        Err(actual_state) => {
            let result = TripResult::NotArmed;
            emit_software_trip_log(arm_id, armed_arm_id, actual_state, &result);
            return result;
        }
    }

    publish_snapshot(clock, TRIP_SOURCE_SOFTWARE, stepper_counts);
    ARM.state
        .store(ArmState::TrippedReady as u8, Ordering::Release);
    TRIP_EVENT_QUEUED.store(true, Ordering::Release);
    let result = TripResult::Tripped;
    emit_software_trip_log(arm_id, armed_arm_id, state_before, &result);
    result
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
mod tests;
