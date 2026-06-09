use super::*;
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

fn make_piece_dur(t: u64, duration_secs: f32) -> (PieceEntry, f64) {
    (
        PieceEntry {
            start_time: t,
            coeffs: [0.0; 4],
            duration: duration_secs,
            _reserved: 0,
        },
        t as f64,
    )
}

fn make_piece(t: u64) -> (PieceEntry, f64) {
    make_piece_dur(t, 0.001)
}

fn make_queue(ring_depth: u32, pushed: u32, retired: u32) -> AxisQueue {
    let mut q = AxisQueue::new(ring_depth);
    q.pushed = pushed;
    q.retired = retired;
    q
}

fn arm_cohort(
    id: u64,
    participants: Vec<AxisKey>,
    timeout: Duration,
    queues: &BTreeMap<AxisKey, AxisQueue>,
) -> DripCohort {
    let mut baseline = BTreeMap::new();
    let mut last_retired = BTreeMap::new();
    let mut ahead_durations = BTreeMap::new();
    let mut pre_arm_in_flight = BTreeMap::new();
    for &k in &participants {
        let q = queues.get(&k);
        let retired = q.map_or(0, |q| q.retired);
        let pushed = q.map_or(0, |q| q.pushed);
        baseline.insert(k, retired);
        last_retired.insert(k, retired);
        ahead_durations.insert(k, VecDeque::new());
        pre_arm_in_flight.insert(k, pushed.wrapping_sub(retired));
    }
    DripCohort {
        id,
        participants: participants.into_iter().collect(),
        timeout,
        baseline,
        last_retired,
        step_deadline: std::time::Instant::now() + timeout,
        deadline_floor: 0,
        ahead_durations,
        released_total_secs: BTreeMap::new(),
        pre_arm_in_flight,
    }
}

/// Short pieces (0.4ms each): verify pump releases ~window/piece_dur pieces
/// up front and tops up one-at-a-time as retirements arrive.
#[test]
fn short_pieces_release_window_worth_upfront() {
    const PIECE_DUR: f32 = 0.0004; // 0.4ms
    let expected_cap = (DRIP_WINDOW_SECS / PIECE_DUR as f64).floor() as usize; // ~125

    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let mut queues = BTreeMap::new();
    let mut q = make_queue(256, 0, 0);
    for i in 0..200u64 {
        q.pieces.push_back(make_piece_dur(i, PIECE_DUR));
    }
    queues.insert(ka, q);

    let co = arm_cohort(1, vec![ka], Duration::from_secs(5), &queues);

    let cap = co.drip_cap(&ka, &queues);
    assert!(
        cap >= expected_cap,
        "initial cap {cap} should be >= window/piece_dur = {expected_cap}"
    );
    assert!(
        cap <= expected_cap + 1,
        "initial cap {cap} should not greatly exceed window/piece_dur = {expected_cap}"
    );
}

/// Long pieces (25ms each): at least 1 piece is always released even when a
/// single piece is close to the window size.
/// 0.025f32 promoted to f64 is slightly > 0.025, so two of them slightly exceed
/// the 50ms window — the cap is 1, which satisfies "at least 1 always".
#[test]
fn long_pieces_minimum_one_piece_released() {
    const PIECE_DUR: f32 = 0.025; // 25ms
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let mut queues = BTreeMap::new();
    let mut q = make_queue(64, 0, 0);
    for i in 0..10u64 {
        q.pieces.push_back(make_piece_dur(i, PIECE_DUR));
    }
    queues.insert(ka, q);

    let co = arm_cohort(1, vec![ka], Duration::from_secs(5), &queues);

    let cap = co.drip_cap(&ka, &queues);
    assert!(
        cap >= 1,
        "at least 1 piece must always be releasable; got {cap}"
    );
    // Two 25ms pieces sum to slightly over 50ms due to f32→f64 widening,
    // so exactly 1 fits. A piece whose duration is 25ms exactly would allow 2.
    assert!(
        cap <= 2,
        "at most 2 pieces expected for ~25ms pieces; got {cap}"
    );
}

/// A single piece whose duration exceeds DRIP_WINDOW_SECS: cap must be 1,
/// not 0, so the feed can't be wedged.
#[test]
fn oversized_piece_cap_is_one() {
    const PIECE_DUR: f32 = 0.100; // 100ms > DRIP_WINDOW_SECS (50ms)
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let mut queues = BTreeMap::new();
    let mut q = make_queue(64, 0, 0);
    q.pieces.push_back(make_piece_dur(0, PIECE_DUR));
    queues.insert(ka, q);

    let co = arm_cohort(1, vec![ka], Duration::from_secs(5), &queues);

    let cap = co.drip_cap(&ka, &queues);
    assert_eq!(cap, 1, "oversized single piece: cap must be exactly 1");
}

/// After filling the window, the cap is 0. Simulates releasing pieces until
/// the window is full, then verifying the gate closes.
/// The queue must stay populated so drip_cap can see pending pieces to release.
#[test]
fn window_fills_and_caps_to_zero() {
    const PIECE_DUR: f32 = 0.010; // 10ms — 5 pieces = 50ms window
    let pieces_for_window = (DRIP_WINDOW_SECS / PIECE_DUR as f64).ceil() as usize; // 5

    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let mut queues = BTreeMap::new();
    // Keep pieces in the queue so drip_cap iterates them (it returns 0 if q is empty)
    let mut q = make_queue(64, 0, 0);
    for i in 0..20u64 {
        q.pieces.push_back(make_piece_dur(i, PIECE_DUR));
    }
    queues.insert(ka, q);

    let mut co = arm_cohort(1, vec![ka], Duration::from_secs(5), &queues);

    // Simulate releasing `pieces_for_window` pieces (fills the window)
    for _ in 0..pieces_for_window {
        let ahead = co.ahead_time_secs(&ka);
        assert!(
            co.drip_cap(&ka, &queues) >= 1,
            "cap must be >= 1 while ahead_time {ahead:.4} < window"
        );
        co.record_released(ka, std::iter::once(PIECE_DUR as f64));
    }

    // Now ahead_time should be >= DRIP_WINDOW_SECS
    let ahead = co.ahead_time_secs(&ka);
    assert!(
        ahead >= DRIP_WINDOW_SECS - f64::EPSILON,
        "ahead_time {ahead:.4} should be >= DRIP_WINDOW_SECS after filling"
    );
    let cap = co.drip_cap(&ka, &queues);
    assert_eq!(cap, 0, "cap must be 0 when window is full");
}

/// Release pieces, then retire them one at a time: verify cap opens back up.
#[test]
fn retirement_reopens_cap() {
    const PIECE_DUR: f32 = 0.025; // 25ms — exactly 2 per window
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let mut queues = BTreeMap::new();
    let mut q = make_queue(64, 0, 0);
    for i in 0..4u64 {
        q.pieces.push_back(make_piece_dur(i, PIECE_DUR));
    }
    queues.insert(ka, q);

    let mut co = arm_cohort(1, vec![ka], Duration::from_secs(5), &queues);

    // Release 2 pieces (fills 50ms window)
    co.record_released(ka, std::iter::once(PIECE_DUR as f64));
    co.record_released(ka, std::iter::once(PIECE_DUR as f64));
    assert_eq!(
        co.drip_cap(&ka, &queues),
        0,
        "window full after 2x25ms pieces"
    );

    // Retire 1 piece: 25ms ahead, room for 1 more
    co.record_retired(&ka, 0, 1).unwrap();
    let cap = co.drip_cap(&ka, &queues);
    assert!(cap >= 1, "cap must reopen after retirement; got {cap}");
}

/// Stall detection: fires when the floor stops advancing with >= window of time in flight.
#[test]
fn stall_detection_fires_when_floor_stuck() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let kb = AxisKey { mcu_id: 0, axis: 1 };
    let (tx, rx) = std::sync::mpsc::channel::<PumpMsg>();
    let stall_msgs: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let stall_msgs_clone = Arc::clone(&stall_msgs);

    let handle = std::thread::spawn(move || {
        run_pump(
            rx,
            NullSink,
            |_| 64,
            |_| None,
            |_| {},
            |_, _| {},
            move |msg: String| {
                stall_msgs_clone.lock().unwrap().push(msg);
            },
        );
    });

    tx.send(PumpMsg::DripArm(DripArm {
        cohort: 55,
        participants: vec![ka, kb],
        timeout: Duration::from_millis(30),
    }))
    .unwrap();

    // Enqueue pieces with enough total duration to exceed DRIP_WINDOW_SECS
    let piece_count = 20u32; // 20 × 0.003s = 60ms > 50ms
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: ka,
        pieces: (0..piece_count)
            .map(|i| make_piece_dur(i as u64, 0.003))
            .collect(),
        fresh_stream: false,
        lead_secs: MAX_LEAD_SECS,
    }))
    .unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: kb,
        pieces: (0..piece_count)
            .map(|i| make_piece_dur(i as u64, 0.003))
            .collect(),
        fresh_stream: false,
        lead_secs: MAX_LEAD_SECS,
    }))
    .unwrap();

    // Wait longer than the stall timeout (with no retirements arriving)
    std::thread::sleep(Duration::from_millis(200));

    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    let msgs = stall_msgs.lock().unwrap();
    assert!(
        !msgs.is_empty(),
        "expected a drip stall timeout but got none"
    );
    assert!(
        msgs[0].contains("55"),
        "stall message must name cohort id 55; got: {}",
        msgs[0]
    );
}

/// Lockstep: two participants, one retiring slower.
/// Release is gated against the cohort floor (slowest participant's executed
/// time): a participant that ran ahead stays blocked until the laggard's
/// execution advances the floor — its own retirements alone must not reopen it.
#[test]
fn two_participants_locked_to_floor_time() {
    // Use a duration that fills the window with a comfortable margin so f32→f64
    // promotion doesn't cause boundary-crossing: 6 pieces × 9ms = 54ms > 50ms.
    const PIECE_DUR: f32 = 0.009; // 9ms
    const FILL_COUNT: usize = 6; // 6 × 9ms = 54ms > DRIP_WINDOW_SECS
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let kb = AxisKey { mcu_id: 0, axis: 1 };

    let mut queues = BTreeMap::new();
    let mut qa = make_queue(64, 0, 0);
    let mut qb = make_queue(64, 0, 0);
    for i in 0..20u64 {
        qa.pieces.push_back(make_piece_dur(i, PIECE_DUR));
        qb.pieces.push_back(make_piece_dur(i, PIECE_DUR));
    }
    queues.insert(ka, qa);
    queues.insert(kb, qb);

    let mut co = arm_cohort(1, vec![ka, kb], Duration::from_secs(5), &queues);

    // Fill A's window
    for _ in 0..FILL_COUNT {
        co.record_released(ka, std::iter::once(PIECE_DUR as f64));
    }
    assert!(
        co.ahead_of_floor_secs(&ka) >= DRIP_WINDOW_SECS,
        "A must be >= window ahead of the floor after {FILL_COUNT} releases"
    );
    assert_eq!(co.drip_cap(&ka, &queues), 0, "A's window full");

    // B has released nothing: it sits at the floor and can still send
    let cap_b = co.drip_cap(&kb, &queues);
    assert!(cap_b >= 1, "B must not be blocked by A's window state");

    // A retires 2 of its own pieces, but B (the floor) has executed nothing:
    // A's distance ahead of the floor is unchanged, so it stays blocked.
    co.record_retired(&ka, 0, 2).unwrap();
    assert_eq!(
        co.drip_cap(&ka, &queues),
        0,
        "A must stay blocked while the floor participant has not executed"
    );

    // B releases and retires 2 pieces (18ms executed) → floor advances →
    // A is now 54-18=36ms ahead of the floor → reopens.
    co.record_released(kb, std::iter::repeat(PIECE_DUR as f64).take(2));
    co.record_retired(&kb, 0, 2).unwrap();
    let cap_a = co.drip_cap(&ka, &queues);
    assert!(
        cap_a >= 1,
        "A must reopen once the floor advances; got {cap_a}"
    );
}

/// Retired-regression: still errors, and desync (MCU retires more than released) also errors.
#[test]
fn retired_regression_triggers_on_drip_stall() {
    let ka = AxisKey { mcu_id: 3, axis: 2 };
    let (tx, rx) = std::sync::mpsc::channel::<PumpMsg>();
    let stall_msgs: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let stall_msgs_clone = Arc::clone(&stall_msgs);

    let handle = std::thread::spawn(move || {
        run_pump(
            rx,
            NullSink,
            |_| 64,
            |_| None,
            |_| {},
            |_, _| {},
            move |msg: String| {
                stall_msgs_clone.lock().unwrap().push(msg);
            },
        );
    });

    // Send 5 pieces before arming so the cohort sees them as pre-arm
    // in-flight; the later 0→5 retirement advance is then legitimate.
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: ka,
        pieces: (0..5).map(|i| make_piece(i as u64)).collect(),
        fresh_stream: false,
        lead_secs: MAX_LEAD_SECS,
    }))
    .unwrap();
    std::thread::sleep(Duration::from_millis(50));

    tx.send(PumpMsg::DripArm(DripArm {
        cohort: 7,
        participants: vec![ka],
        timeout: Duration::from_secs(60),
    }))
    .unwrap();

    tx.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 3,
        retired_counts: vec![0, 0, 5],
    }))
    .unwrap();

    std::thread::sleep(Duration::from_millis(50));

    tx.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 3,
        retired_counts: vec![0, 0, 3],
    }))
    .unwrap();

    std::thread::sleep(Duration::from_millis(50));

    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    let msgs = stall_msgs.lock().unwrap();
    assert_eq!(
        msgs.len(),
        1,
        "expected exactly one drip stall error, got: {msgs:?}"
    );
    assert!(
        msgs[0].contains("regression"),
        "error must mention 'regression'; got: {}",
        msgs[0]
    );
    assert!(
        msgs[0].contains("mcu3"),
        "error must name the MCU; got: {}",
        msgs[0]
    );
    assert!(
        msgs[0].contains("axis2"),
        "error must name the axis; got: {}",
        msgs[0]
    );
}

struct NullSink;

impl PieceSink for NullSink {
    fn send_frame(
        &self,
        _key: AxisKey,
        _pieces: &[PieceEntry],
        _start_slot: u16,
        _new_head: u32,
    ) -> Result<i32, SendError> {
        Ok(kalico_protocol::result_codes::OK)
    }
}

#[test]
fn non_cohort_axis_streams_freely_while_cohort_armed() {
    const PIECE_DUR: f32 = 0.010;
    let cohort_axis = AxisKey { mcu_id: 0, axis: 0 };
    let free_axis = AxisKey { mcu_id: 0, axis: 1 };

    let mut queues = BTreeMap::new();
    queues.insert(cohort_axis, make_queue(64, 0, 0));
    queues.insert(free_axis, make_queue(64, 0, 0));

    let mut co = arm_cohort(99, vec![cohort_axis], Duration::from_secs(5), &queues);

    // Fill cohort_axis's window so its drip_cap = 0
    let pieces_to_fill = (DRIP_WINDOW_SECS / PIECE_DUR as f64).ceil() as usize;
    for _ in 0..pieces_to_fill {
        co.record_released(cohort_axis, std::iter::once(PIECE_DUR as f64));
    }
    assert_eq!(co.drip_cap(&cohort_axis, &queues), 0);

    queues.get_mut(&free_axis).unwrap().pushed = 10;
    for i in 0..5u64 {
        queues
            .get_mut(&free_axis)
            .unwrap()
            .pieces
            .push_back(make_piece(i));
    }

    let cap_of = |k: &AxisKey| -> usize {
        if co.participants.contains(k) {
            co.drip_cap(k, &queues)
        } else {
            usize::MAX
        }
    };
    let hz_of = |_k: &AxisKey, _q: &AxisQueue| -> Option<u64> { None };

    match schedule(&queues, 32, hz_of, cap_of) {
        Schedule::Send(frames) => {
            assert!(
                frames.iter().any(|f| f.key == free_axis),
                "free axis must still send while cohort is window-blocked"
            );
            assert!(
                !frames.iter().any(|f| f.key == cohort_axis),
                "cohort axis must NOT appear in frames when window-blocked"
            );
        }
        other => panic!("expected Send with free_axis piece; got {other:?}"),
    }
}

#[test]
fn drip_disarm_clears_cohort() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let (tx, rx) = std::sync::mpsc::channel::<PumpMsg>();

    let handle = std::thread::spawn(move || {
        run_pump(rx, NullSink, |_| 64, |_| None, |_| {}, |_, _| {}, |_| {});
    });

    tx.send(PumpMsg::DripArm(DripArm {
        cohort: 10,
        participants: vec![ka],
        timeout: Duration::from_secs(60),
    }))
    .unwrap();

    std::thread::sleep(Duration::from_millis(20));

    tx.send(PumpMsg::DripDisarm(10)).unwrap();

    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: ka,
        pieces: (0..10).map(|i| make_piece(i as u64)).collect(),
        fresh_stream: false,
        lead_secs: MAX_LEAD_SECS,
    }))
    .unwrap();

    std::thread::sleep(Duration::from_millis(50));
    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn drip_disarm_wrong_cohort_id_is_noop() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let mut queues = BTreeMap::new();
    queues.insert(ka, make_queue(64, 0, 0));

    let mut co = arm_cohort(5, vec![ka], Duration::from_secs(5), &queues);

    // Fill window
    co.record_released(ka, std::iter::repeat(DRIP_WINDOW_SECS / 2.0).take(2));
    assert_eq!(co.drip_cap(&ka, &queues), 0);
    assert_eq!(co.id, 5, "cohort must still be id=5");
    assert_eq!(
        co.drip_cap(&ka, &queues),
        0,
        "window gate must remain active"
    );
}

#[test]
fn mcu_reboot_retired_to_zero_triggers_regression() {
    let ka = AxisKey { mcu_id: 2, axis: 1 };
    let (tx, rx) = std::sync::mpsc::channel::<PumpMsg>();
    let stall_msgs: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let stall_msgs_clone = Arc::clone(&stall_msgs);

    let handle = std::thread::spawn(move || {
        run_pump(
            rx,
            NullSink,
            |_| 64,
            |_| None,
            |_| {},
            |_, _| {},
            move |msg: String| {
                stall_msgs_clone.lock().unwrap().push(msg);
            },
        );
    });

    // Send 5 pieces before arming so the cohort sees them as pre-arm
    // in-flight; the later 0→5 retirement advance is then legitimate.
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: ka,
        pieces: (0..5).map(|i| make_piece(i as u64)).collect(),
        fresh_stream: false,
        lead_secs: MAX_LEAD_SECS,
    }))
    .unwrap();
    std::thread::sleep(Duration::from_millis(50));

    tx.send(PumpMsg::DripArm(DripArm {
        cohort: 42,
        participants: vec![ka],
        timeout: Duration::from_secs(60),
    }))
    .unwrap();

    tx.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 2,
        retired_counts: vec![0, 5],
    }))
    .unwrap();

    std::thread::sleep(Duration::from_millis(50));

    tx.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 2,
        retired_counts: vec![0, 0],
    }))
    .unwrap();

    std::thread::sleep(Duration::from_millis(50));

    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    let msgs = stall_msgs.lock().unwrap();
    assert_eq!(
        msgs.len(),
        1,
        "expected exactly one regression error; got: {msgs:?}"
    );
    assert!(
        msgs[0].contains("regression"),
        "error must mention 'regression'; got: {}",
        msgs[0]
    );
}

/// Verify record_retired fails loudly when claimed retirements exceed released pieces.
#[test]
fn record_retired_desync_returns_err() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let queues = BTreeMap::new();
    let mut co = arm_cohort(1, vec![ka], Duration::from_secs(5), &queues);

    // Release 1 piece worth of duration
    co.record_released(ka, std::iter::once(0.010_f64));

    // Claim 2 retirements → desync
    let result = co.record_retired(&ka, 0, 2);
    assert!(
        result.is_err(),
        "must return Err when claimed retirements exceed released pieces"
    );
}

#[test]
fn record_retired_with_nothing_tracked_is_desync() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let queues = BTreeMap::new();
    let mut co = arm_cohort(1, vec![ka], Duration::from_secs(5), &queues);

    let result = co.record_retired(&ka, 0, 1);
    assert!(
        result.is_err(),
        "a retirement with no pre-arm credit and nothing released must be a desync"
    );
}

/// Fix A: participant window-blocked (cap 0), non-participant on same MCU has
/// pending pieces inside horizon → schedule must emit non-participant frames,
/// not report StallAhead.
#[test]
fn cap_zero_participant_does_not_block_non_participant_same_mcu() {
    const PIECE_DUR: f32 = 0.010;
    let participant = AxisKey { mcu_id: 0, axis: 0 };
    let bystander = AxisKey { mcu_id: 0, axis: 1 };

    let mut queues = BTreeMap::new();
    let mut qp = make_queue(64, 0, 0);
    qp.pieces.push_back(make_piece_dur(5, PIECE_DUR));
    queues.insert(participant, qp);

    let mut qb = make_queue(64, 0, 0);
    qb.pieces.push_back(make_piece_dur(10, PIECE_DUR));
    queues.insert(bystander, qb);

    let mut co = arm_cohort(1, vec![participant], Duration::from_secs(5), &queues);

    let pieces_to_fill = (DRIP_WINDOW_SECS / PIECE_DUR as f64).ceil() as usize;
    for _ in 0..pieces_to_fill {
        co.record_released(participant, std::iter::once(PIECE_DUR as f64));
    }
    assert_eq!(
        co.drip_cap(&participant, &queues),
        0,
        "participant window must be full"
    );

    let cap_of = |k: &AxisKey| -> usize {
        if co.participants.contains(k) {
            co.drip_cap(k, &queues)
        } else {
            usize::MAX
        }
    };
    let hz_of = |_k: &AxisKey, _q: &AxisQueue| -> Option<u64> { None };

    match schedule(&queues, 32, hz_of, cap_of) {
        Schedule::Send(frames) => {
            assert!(
                frames.iter().any(|f| f.key == bystander),
                "bystander must send while participant is cap-gated"
            );
            assert!(
                !frames.iter().any(|f| f.key == participant),
                "cap-gated participant must not appear in frames"
            );
        }
        other => panic!("expected Send(bystander); got {other:?}"),
    }
}

/// A cap-gated participant holding the globally-oldest mint time must not
/// break frame-building for a releasable queue on ANOTHER MCU.
#[test]
fn cap_gated_participant_does_not_block_other_mcu() {
    const PIECE_DUR: f32 = 0.010;
    let participant = AxisKey { mcu_id: 0, axis: 0 };
    let other_mcu = AxisKey { mcu_id: 1, axis: 2 };

    let mut queues = BTreeMap::new();
    // Participant's pending piece carries the OLDEST host mint time.
    let mut qp = make_queue(64, 0, 0);
    qp.pieces.push_back(make_piece_dur(5, PIECE_DUR));
    queues.insert(participant, qp);

    let mut qz = make_queue(64, 0, 0);
    qz.pieces.push_back(make_piece_dur(10, PIECE_DUR));
    queues.insert(other_mcu, qz);

    let mut co = arm_cohort(1, vec![participant], Duration::from_secs(5), &queues);
    let pieces_to_fill = (DRIP_WINDOW_SECS / PIECE_DUR as f64).ceil() as usize;
    for _ in 0..pieces_to_fill {
        co.record_released(participant, std::iter::once(PIECE_DUR as f64));
    }
    assert_eq!(co.drip_cap(&participant, &queues), 0);

    let cap_of = |k: &AxisKey| -> usize {
        if co.participants.contains(k) {
            co.drip_cap(k, &queues)
        } else {
            usize::MAX
        }
    };
    let hz_of = |_k: &AxisKey, _q: &AxisQueue| -> Option<u64> { None };

    match schedule(&queues, 32, hz_of, cap_of) {
        Schedule::Send(frames) => {
            assert!(
                frames.iter().any(|f| f.key == other_mcu),
                "other-MCU queue must send while the participant is cap-gated"
            );
            assert!(
                !frames.iter().any(|f| f.key == participant),
                "cap-gated participant must not appear in frames"
            );
        }
        other => panic!("expected Send(other_mcu); got {other:?}"),
    }
}

/// Fix A: all queues cap-gated (all participants, window full) → StallAhead
/// is still reported so the pump knows to poll for retirement progress.
#[test]
fn all_queues_cap_gated_reports_stall_ahead() {
    const PIECE_DUR: f32 = 0.010;
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let kb = AxisKey { mcu_id: 0, axis: 1 };

    let mut queues = BTreeMap::new();
    let mut qa = make_queue(64, 0, 0);
    qa.pieces.push_back(make_piece_dur(5, PIECE_DUR));
    queues.insert(ka, qa);

    let mut qb = make_queue(64, 0, 0);
    qb.pieces.push_back(make_piece_dur(10, PIECE_DUR));
    queues.insert(kb, qb);

    let mut co = arm_cohort(1, vec![ka, kb], Duration::from_secs(5), &queues);

    let pieces_to_fill = (DRIP_WINDOW_SECS / PIECE_DUR as f64).ceil() as usize;
    for _ in 0..pieces_to_fill {
        co.record_released(ka, std::iter::once(PIECE_DUR as f64));
        co.record_released(kb, std::iter::once(PIECE_DUR as f64));
    }
    assert_eq!(co.drip_cap(&ka, &queues), 0);
    assert_eq!(co.drip_cap(&kb, &queues), 0);

    let cap_of = |k: &AxisKey| -> usize {
        if co.participants.contains(k) {
            co.drip_cap(k, &queues)
        } else {
            usize::MAX
        }
    };
    let hz_of = |_k: &AxisKey, _q: &AxisQueue| -> Option<u64> { None };

    assert!(
        matches!(
            schedule(&queues, 32, hz_of, cap_of),
            Schedule::StallAhead(_)
        ),
        "all queues cap-gated must still produce StallAhead"
    );
}

/// Pre-arm in-flight pieces are drained cleanly without triggering desync.
#[test]
fn pre_arm_in_flight_drained_without_desync() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let mut queues = BTreeMap::new();
    // 3 pieces already in flight at arm time
    queues.insert(ka, make_queue(64, 3, 0));

    let mut co = arm_cohort(1, vec![ka], Duration::from_secs(5), &queues);

    // Retire 2 of the pre-arm pieces: must succeed without touching ahead_durations
    co.record_retired(&ka, 0, 2).unwrap();
    assert_eq!(
        co.ahead_durations[&ka].len(),
        0,
        "ahead_durations must stay empty for pre-arm retirements"
    );

    // Release 1 post-arm piece
    co.record_released(ka, std::iter::once(0.010_f64));

    // Retire the last pre-arm piece: still no desync
    co.record_retired(&ka, 2, 3).unwrap();
    assert_eq!(
        co.ahead_durations[&ka].len(),
        1,
        "post-arm piece must remain in ahead_durations after pre-arm drained"
    );

    // Now retire the post-arm piece
    co.record_retired(&ka, 3, 4).unwrap();
    assert_eq!(
        co.ahead_durations[&ka].len(),
        0,
        "ahead_durations must be empty after post-arm piece retired"
    );
}
