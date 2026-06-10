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

#[derive(Clone)]
struct CountingSink {
    sent: Arc<Mutex<Vec<(AxisKey, u64)>>>,
}

impl CountingSink {
    fn new() -> Self {
        Self {
            sent: Arc::new(Mutex::new(Vec::new())),
        }
    }
    fn sent(&self) -> Vec<(AxisKey, u64)> {
        self.sent.lock().unwrap().clone()
    }
}

impl PieceSink for CountingSink {
    fn send_frame(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        _start_slot: u16,
        _new_head: u32,
    ) -> Result<i32, SendError> {
        let mut sent = self.sent.lock().unwrap();
        for p in pieces {
            sent.push((key, p.start_time));
        }
        Ok(kalico_protocol::result_codes::OK)
    }
}

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

    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: ka,
        pieces: (0..20).map(|i| make_piece_dur(i as u64, 0.003)).collect(),
        fresh_stream: false,
        lead_secs: DRIP_WINDOW_SECS,
    }))
    .unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: kb,
        pieces: (0..20).map(|i| make_piece_dur(i as u64, 0.003)).collect(),
        fresh_stream: false,
        lead_secs: DRIP_WINDOW_SECS,
    }))
    .unwrap();

    std::thread::sleep(Duration::from_millis(200));

    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    let msgs = stall_msgs.lock().unwrap();
    assert_eq!(msgs.len(), 1, "expected one stall, got: {msgs:?}");
    assert!(
        msgs[0].contains("floor stalled"),
        "stall must mention the floor; got: {}",
        msgs[0]
    );
}

#[test]
fn non_participant_enqueue_aborts_cohort_and_drops_pieces() {
    let participant = AxisKey { mcu_id: 0, axis: 0 };
    let outsider = AxisKey { mcu_id: 0, axis: 3 };
    let sink = CountingSink::new();
    let (tx, rx) = std::sync::mpsc::channel::<PumpMsg>();
    let stall_msgs: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let stall_msgs_clone = Arc::clone(&stall_msgs);

    tx.send(PumpMsg::DripArm(DripArm {
        cohort: 9,
        participants: vec![participant],
        timeout: Duration::from_secs(60),
    }))
    .unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: outsider,
        pieces: (0..3).map(|i| make_piece(i as u64)).collect(),
        fresh_stream: false,
        lead_secs: MAX_LEAD_SECS,
    }))
    .unwrap();
    tx.send(PumpMsg::Shutdown).unwrap();

    let sink_clone = sink.clone();
    let handle = std::thread::spawn(move || {
        run_pump(
            rx,
            sink_clone,
            |_| 64,
            |_| Some((0u64, 1000.0)),
            |_| {},
            |_, _| {},
            move |msg: String| {
                stall_msgs_clone.lock().unwrap().push(msg);
            },
        );
    });
    handle.join().unwrap();

    let msgs = stall_msgs.lock().unwrap();
    assert_eq!(msgs.len(), 1, "expected one abort, got: {msgs:?}");
    assert!(
        msgs[0].contains("non-participant"),
        "abort must name the violation; got: {}",
        msgs[0]
    );
    assert!(
        sink.sent().is_empty(),
        "outsider pieces must be dropped, got {:?}",
        sink.sent()
    );
}

#[test]
fn participant_release_tracks_mcu_clock_horizon() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let sink = CountingSink::new();
    let clock: Arc<Mutex<u64>> = Arc::new(Mutex::new(0));
    let clock_for_pump = Arc::clone(&clock);
    let (tx, rx) = std::sync::mpsc::channel::<PumpMsg>();

    let sink_clone = sink.clone();
    let handle = std::thread::spawn(move || {
        run_pump(
            rx,
            sink_clone,
            |_| 64,
            move |_| Some((*clock_for_pump.lock().unwrap(), 1000.0)),
            |_| {},
            |_, _| {},
            |_| {},
        );
    });

    tx.send(PumpMsg::DripArm(DripArm {
        cohort: 12,
        participants: vec![ka],
        timeout: Duration::from_secs(60),
    }))
    .unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: ka,
        pieces: vec![make_piece(50), make_piece(500)],
        fresh_stream: false,
        lead_secs: DRIP_WINDOW_SECS,
    }))
    .unwrap();

    let deadline = std::time::Instant::now() + Duration::from_secs(2);
    while sink.sent().is_empty() {
        assert!(std::time::Instant::now() < deadline, "first piece not sent");
        std::thread::yield_now();
    }
    std::thread::sleep(Duration::from_millis(50));
    assert_eq!(
        sink.sent(),
        vec![(ka, 50)],
        "piece at 500 is beyond horizon 100 and must be held"
    );

    *clock.lock().unwrap() = 450;
    let deadline = std::time::Instant::now() + Duration::from_secs(2);
    while sink.sent().len() < 2 {
        assert!(
            std::time::Instant::now() < deadline,
            "held piece not released after clock advance"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
    assert_eq!(sink.sent(), vec![(ka, 50), (ka, 500)]);

    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn unsynced_clock_releases_nothing_for_participants() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let sink = CountingSink::new();
    let (tx, rx) = std::sync::mpsc::channel::<PumpMsg>();

    tx.send(PumpMsg::DripArm(DripArm {
        cohort: 13,
        participants: vec![ka],
        timeout: Duration::from_secs(60),
    }))
    .unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: ka,
        pieces: (10..14).map(|i| make_piece(i as u64)).collect(),
        fresh_stream: false,
        lead_secs: DRIP_WINDOW_SECS,
    }))
    .unwrap();

    let sink_clone = sink.clone();
    let handle = std::thread::spawn(move || {
        run_pump(rx, sink_clone, |_| 64, |_| None, |_| {}, |_, _| {}, |_| {});
    });
    std::thread::sleep(Duration::from_millis(100));
    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    assert!(
        sink.sent().is_empty(),
        "nothing may release without a clock, got {:?}",
        sink.sent()
    );
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
    assert_eq!(msgs.len(), 1, "expected one stall error, got: {msgs:?}");
    assert!(
        msgs[0].contains("regression") && msgs[0].contains("mcu3") && msgs[0].contains("axis2"),
        "error must describe the regression; got: {}",
        msgs[0]
    );
}

#[test]
fn mcu_reboot_retired_to_zero_triggers_regression() {
    let ka = AxisKey { mcu_id: 1, axis: 0 };
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

    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: ka,
        pieces: vec![make_piece(10)],
        fresh_stream: false,
        lead_secs: DRIP_WINDOW_SECS,
    }))
    .unwrap();
    tx.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        retired_counts: vec![40],
    }))
    .unwrap();
    std::thread::sleep(Duration::from_millis(30));

    tx.send(PumpMsg::DripArm(DripArm {
        cohort: 21,
        participants: vec![ka],
        timeout: Duration::from_secs(60),
    }))
    .unwrap();
    std::thread::sleep(Duration::from_millis(30));

    tx.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        retired_counts: vec![0],
    }))
    .unwrap();
    std::thread::sleep(Duration::from_millis(50));

    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    let msgs = stall_msgs.lock().unwrap();
    assert_eq!(msgs.len(), 1, "expected one regression, got: {msgs:?}");
    assert!(msgs[0].contains("regression"), "got: {}", msgs[0]);
}

#[test]
fn drip_disarm_clears_cohort() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let outsider = AxisKey { mcu_id: 0, axis: 3 };
    let sink = CountingSink::new();
    let (tx, rx) = std::sync::mpsc::channel::<PumpMsg>();
    let stall_msgs: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let stall_msgs_clone = Arc::clone(&stall_msgs);

    let sink_clone = sink.clone();
    let handle = std::thread::spawn(move || {
        run_pump(
            rx,
            sink_clone,
            |_| 64,
            |_| Some((0u64, 1000.0)),
            |_| {},
            |_, _| {},
            move |msg: String| {
                stall_msgs_clone.lock().unwrap().push(msg);
            },
        );
    });

    tx.send(PumpMsg::DripArm(DripArm {
        cohort: 31,
        participants: vec![ka],
        timeout: Duration::from_secs(60),
    }))
    .unwrap();
    tx.send(PumpMsg::DripDisarm(31)).unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: outsider,
        pieces: vec![make_piece(1)],
        fresh_stream: false,
        lead_secs: MAX_LEAD_SECS,
    }))
    .unwrap();

    let deadline = std::time::Instant::now() + Duration::from_secs(2);
    while sink.sent().is_empty() {
        assert!(
            std::time::Instant::now() < deadline,
            "outsider piece not sent after disarm"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    assert!(stall_msgs.lock().unwrap().is_empty());
    assert_eq!(sink.sent(), vec![(outsider, 1)]);
}

#[test]
fn drip_disarm_wrong_cohort_id_is_noop() {
    let ka = AxisKey { mcu_id: 0, axis: 0 };
    let outsider = AxisKey { mcu_id: 0, axis: 3 };
    let (tx, rx) = std::sync::mpsc::channel::<PumpMsg>();
    let stall_msgs: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let stall_msgs_clone = Arc::clone(&stall_msgs);

    tx.send(PumpMsg::DripArm(DripArm {
        cohort: 31,
        participants: vec![ka],
        timeout: Duration::from_secs(60),
    }))
    .unwrap();
    tx.send(PumpMsg::DripDisarm(999)).unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: outsider,
        pieces: vec![make_piece(1)],
        fresh_stream: false,
        lead_secs: MAX_LEAD_SECS,
    }))
    .unwrap();
    tx.send(PumpMsg::Shutdown).unwrap();

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
    handle.join().unwrap();

    let msgs = stall_msgs.lock().unwrap();
    assert_eq!(
        msgs.len(),
        1,
        "wrong-id disarm must not clear the cohort: {msgs:?}"
    );
}
