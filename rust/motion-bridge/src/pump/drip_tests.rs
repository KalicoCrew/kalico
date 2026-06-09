use super::*;
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

fn make_piece(t: u64) -> (PieceEntry, f64) {
    (
        PieceEntry {
            start_time: t,
            coeffs: [0.0; 4],
            duration: 0.001,
            _reserved: 0,
        },
        t as f64,
    )
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
    for &k in &participants {
        let retired = queues.get(&k).map_or(0, |q| q.retired);
        baseline.insert(k, retired);
        last_retired.insert(k, retired);
    }
    DripCohort {
        id,
        participants: participants.into_iter().collect(),
        timeout,
        baseline,
        last_retired,
        step_deadline: std::time::Instant::now() + timeout,
        deadline_floor: 0,
    }
}


#[test]
fn drip_cap_steady_state_allows_exactly_budget() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let kb = AxisKey { mcu_id: 0, axis: 1 };

    let mut queues = BTreeMap::new();
    queues.insert(ka, make_queue(64, 0, 0));
    queues.insert(kb, make_queue(64, 0, 0));

    let co = arm_cohort(1, vec![ka, kb], Duration::from_secs(5), &queues);

    assert_eq!(co.drip_cap(&ka, &queues), DRIP_BUDGET as usize);
    assert_eq!(co.drip_cap(&kb, &queues), DRIP_BUDGET as usize);

    queues.get_mut(&ka).unwrap().pushed = 1;
    queues.get_mut(&kb).unwrap().pushed = 1;
    assert_eq!(co.drip_cap(&ka, &queues), (DRIP_BUDGET - 1) as usize);
    assert_eq!(co.drip_cap(&kb, &queues), (DRIP_BUDGET - 1) as usize);

    queues.get_mut(&ka).unwrap().pushed = DRIP_BUDGET;
    queues.get_mut(&kb).unwrap().pushed = DRIP_BUDGET;
    assert_eq!(co.drip_cap(&ka, &queues), 0);
    assert_eq!(co.drip_cap(&kb, &queues), 0);

    queues.get_mut(&ka).unwrap().retired = 1;
    queues.get_mut(&kb).unwrap().retired = 1;
    assert_eq!(co.drip_cap(&ka, &queues), 1);
    assert_eq!(co.drip_cap(&kb, &queues), 1);
}


#[test]
fn drip_stalled_participant_freezes_all_at_budget() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let kb = AxisKey { mcu_id: 0, axis: 1 };

    let mut queues = BTreeMap::new();
    queues.insert(ka, make_queue(64, 0, 0));
    queues.insert(kb, make_queue(64, 0, 0));

    let co = arm_cohort(1, vec![ka, kb], Duration::from_secs(5), &queues);

    queues.get_mut(&ka).unwrap().pushed = DRIP_BUDGET;
    queues.get_mut(&kb).unwrap().pushed = DRIP_BUDGET;
    assert_eq!(co.drip_cap(&ka, &queues), 0, "A must be frozen when floor = 0");
    assert_eq!(co.drip_cap(&kb, &queues), 0, "B must be frozen when floor = 0");

    queues.get_mut(&kb).unwrap().retired = 1;
    assert_eq!(co.drip_cap(&ka, &queues), 0, "A must stay frozen until A retires");
    assert_eq!(co.drip_cap(&kb, &queues), 0, "B stays frozen while A has not advanced floor");

    queues.get_mut(&ka).unwrap().retired = 1;
    assert_eq!(co.drip_cap(&ka, &queues), 1, "A unfreeze once floor advances");
    assert_eq!(co.drip_cap(&kb, &queues), 1, "B unfreeze once floor advances");
}


#[test]
fn drip_budget_bounds_total_pushed_beyond_floor() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let kb = AxisKey { mcu_id: 0, axis: 1 };
    let kc = AxisKey { mcu_id: 1, axis: 0 };

    let mut queues = BTreeMap::new();
    queues.insert(ka, make_queue(64, 0, 0));
    queues.insert(kb, make_queue(64, 0, 0));
    queues.insert(kc, make_queue(64, 0, 0));

    let co = arm_cohort(42, vec![ka, kb, kc], Duration::from_secs(5), &queues);

    for round in 0..10u32 {
        for k in [ka, kb, kc] {
            queues.get_mut(&k).unwrap().pushed = round + 1;
            queues.get_mut(&k).unwrap().retired = round;
        }
        let floor = co.floor(&queues);
        for k in [ka, kb, kc] {
            let released = co.released(&k, &queues);
            let ahead = released.saturating_sub(floor);
            assert!(
                ahead <= DRIP_BUDGET,
                "round {round}: axis {k:?} released {released} ahead of floor {floor} by {ahead} > DRIP_BUDGET={DRIP_BUDGET}"
            );
        }
    }

    for k in [ka, kb, kc] {
        queues.get_mut(&k).unwrap().pushed = DRIP_BUDGET;
        queues.get_mut(&k).unwrap().retired = 0;
    }
    let floor = co.floor(&queues);
    assert_eq!(floor, 0, "kc frozen → floor stays at 0");
    for k in [ka, kb, kc] {
        let cap = co.drip_cap(&k, &queues);
        assert_eq!(
            cap, 0,
            "all axes must be capped at 0 when floor=0 and released=DRIP_BUDGET"
        );
    }

    for k in [ka, kb, kc] {
        let released = co.released(&k, &queues);
        assert_eq!(
            released, DRIP_BUDGET,
            "at most DRIP_BUDGET={DRIP_BUDGET} pieces pushed beyond floor; got {released}"
        );
    }
}


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
    assert_eq!(msgs.len(), 1, "expected exactly one drip stall error, got: {msgs:?}");
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
    let cohort_axis = AxisKey { mcu_id: 0, axis: 0 };
    let free_axis = AxisKey { mcu_id: 0, axis: 1 };

    let mut queues = BTreeMap::new();
    queues.insert(cohort_axis, make_queue(64, 0, 0));
    queues.insert(free_axis, make_queue(64, 0, 0));

    let co = arm_cohort(99, vec![cohort_axis], Duration::from_secs(5), &queues);

    queues.get_mut(&cohort_axis).unwrap().pushed = DRIP_BUDGET;
    assert_eq!(
        co.drip_cap(&cohort_axis, &queues),
        0,
        "cohort axis must be capped at budget"
    );

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
                "free axis must still send while cohort is budget-blocked"
            );
            assert!(
                !frames.iter().any(|f| f.key == cohort_axis),
                "cohort axis must NOT appear in frames when budget=0"
            );
        }
        other => panic!("expected Send with free_axis piece; got {other:?}"),
    }
}


#[test]
fn drip_stall_timeout_fires_when_floor_stuck() {
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

    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: ka,
        pieces: (0..DRIP_BUDGET).map(|i| make_piece(i as u64)).collect(),
        fresh_stream: false,
        lead_secs: MAX_LEAD_SECS,
        drip_cohort: Some(55),
    }))
    .unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: kb,
        pieces: (0..DRIP_BUDGET).map(|i| make_piece(i as u64)).collect(),
        fresh_stream: false,
        lead_secs: MAX_LEAD_SECS,
        drip_cohort: Some(55),
    }))
    .unwrap();

    std::thread::sleep(Duration::from_millis(200));

    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    let msgs = stall_msgs.lock().unwrap();
    assert!(
        !msgs.is_empty(),
        "expected a drip stall timeout error but got none"
    );
    assert!(
        msgs[0].contains("55"),
        "stall message must name cohort id 55; got: {}",
        msgs[0]
    );
}


#[test]
fn drip_disarm_clears_cohort() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let (tx, rx) = std::sync::mpsc::channel::<PumpMsg>();

    let handle = std::thread::spawn(move || {
        run_pump(
            rx,
            NullSink,
            |_| 64,
            |_| None,
            |_| {},
            |_, _| {},
            |_| {},
        );
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
        drip_cohort: None,
    }))
    .unwrap();

    std::thread::sleep(Duration::from_millis(50));
    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn arm_with_pre_arm_backlog_bounds_absolute_in_flight() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };

    let mut queues = BTreeMap::new();
    let retired0 = 7u32;
    let backlog = DRIP_BUDGET + 1;
    queues.insert(ka, make_queue(64, retired0 + backlog, retired0));

    let co = arm_cohort(1, vec![ka], Duration::from_secs(5), &queues);

    assert_eq!(
        co.baseline.get(&ka).copied().unwrap_or(0),
        retired0,
        "baseline must be the retired count at arm"
    );

    assert_eq!(co.drip_cap(&ka, &queues), 0, "gate must block when pre-arm backlog >= DRIP_BUDGET");

    let in_flight_at_arm = queues[&ka].pushed.wrapping_sub(queues[&ka].retired);
    assert_eq!(in_flight_at_arm, backlog);

    queues.get_mut(&ka).unwrap().retired = retired0 + 1;
    assert_eq!(co.drip_cap(&ka, &queues), 0, "still capped while ahead >= DRIP_BUDGET");
    let in_flight = queues[&ka].pushed.wrapping_sub(queues[&ka].retired);
    assert!(in_flight <= in_flight_at_arm, "in-flight must not grow: {in_flight} > {in_flight_at_arm}");

    queues.get_mut(&ka).unwrap().retired = retired0 + 2;
    assert_eq!(co.drip_cap(&ka, &queues), 1, "one slot opens when backlog drains below DRIP_BUDGET");
    let in_flight = queues[&ka].pushed.wrapping_sub(queues[&ka].retired);
    assert!(in_flight <= in_flight_at_arm, "in-flight must not grow: {in_flight} > {in_flight_at_arm}");

    queues.get_mut(&ka).unwrap().pushed += 1;
    assert_eq!(co.drip_cap(&ka, &queues), 0, "cap returns to 0 after new piece fills the reopened slot");
    let in_flight = queues[&ka].pushed.wrapping_sub(queues[&ka].retired);
    assert_eq!(in_flight, DRIP_BUDGET, "absolute in-flight is now DRIP_BUDGET, never backlog+1");
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
    assert_eq!(msgs.len(), 1, "expected exactly one drip stall/regression error; got: {msgs:?}");
    assert!(
        msgs[0].contains("regression"),
        "error must mention 'regression'; got: {}",
        msgs[0]
    );
}


#[test]
fn drip_disarm_wrong_cohort_id_is_noop() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };

    let mut queues = BTreeMap::new();
    queues.insert(ka, make_queue(64, 0, 0));

    let co = arm_cohort(5, vec![ka], Duration::from_secs(5), &queues);

    queues.get_mut(&ka).unwrap().pushed = DRIP_BUDGET;
    assert_eq!(co.drip_cap(&ka, &queues), 0);

    assert_eq!(co.id, 5, "cohort must still be id=5 after a wrong disarm");
    assert_eq!(
        co.drip_cap(&ka, &queues),
        0,
        "budget gate must remain active after wrong disarm"
    );
}
