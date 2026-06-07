use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use motion_bridge_native::drain::DrainSync;
use motion_bridge_native::pump::{
    AxisKey, EnqueueMsg, HeartbeatMsg, PieceSink, PumpMsg, SendError, WireSink, run_pump,
};
use runtime::piece_ring::PieceEntry;

fn piece(t: u64) -> (PieceEntry, f64) {
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

#[test]
fn wire_sink_missing_transport_is_hard_error() {
    use std::collections::HashMap;

    let sink = WireSink {
        transports: HashMap::new(),
        timeout: Duration::from_secs(1),
    };
    let (p, _) = piece(0);
    let result = sink.send_frame(
        AxisKey {
            mcu_id: 99,
            axis: 0,
        },
        &[p],
        0,
        1,
    );
    assert!(
        result.is_err(),
        "missing transport must be a hard error, not silent drop"
    );
    let err = result.unwrap_err();
    let msg = err.to_string();
    assert!(
        msg.contains("no transport for mcu_id 99"),
        "error must name the offending mcu_id; got: {msg}"
    );
}

#[derive(Clone)]
struct PerMcuCountSink {
    calls: Arc<Mutex<std::collections::HashMap<u32, u32>>>,
}

impl PerMcuCountSink {
    fn new() -> Self {
        Self {
            calls: Arc::new(Mutex::new(std::collections::HashMap::new())),
        }
    }

    fn count_for(&self, mcu_id: u32) -> u32 {
        self.calls
            .lock()
            .unwrap()
            .get(&mcu_id)
            .copied()
            .unwrap_or(0)
    }
}

impl PieceSink for PerMcuCountSink {
    fn send_frame(
        &self,
        key: AxisKey,
        _pieces: &[PieceEntry],
        _start_slot: u16,
        _new_head: u32,
    ) -> Result<i32, SendError> {
        *self.calls.lock().unwrap().entry(key.mcu_id).or_insert(0) += 1;
        Ok(0)
    }
}

#[test]
fn pump_routes_both_serial_and_ethercat_mcu_ids() {
    let sink = PerMcuCountSink::new();
    let counts = Arc::clone(&sink.calls);

    let (tx, rx) = mpsc::channel::<PumpMsg>();
    let handle = std::thread::spawn(move || {
        run_pump(rx, sink, |_k| 8u32, |_| None, |_| {});
    });

    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: AxisKey { mcu_id: 1, axis: 0 },
        pieces: vec![piece(0)],
        fresh_stream: false,
    }))
    .unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: AxisKey { mcu_id: 2, axis: 0 },
        pieces: vec![piece(1)],
        fresh_stream: false,
    }))
    .unwrap();

    let deadline = std::time::Instant::now() + Duration::from_secs(2);
    loop {
        let c1 = counts.lock().unwrap().get(&1).copied().unwrap_or(0);
        let c2 = counts.lock().unwrap().get(&2).copied().unwrap_or(0);
        if c1 >= 1 && c2 >= 1 {
            break;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "pump did not service both mcu_ids within deadline (mcu1={c1} mcu2={c2})"
        );
        std::thread::yield_now();
    }

    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();

    let final_c1 = counts.lock().unwrap().get(&1).copied().unwrap_or(0);
    let final_c2 = counts.lock().unwrap().get(&2).copied().unwrap_or(0);
    assert!(
        final_c1 >= 1,
        "serial MCU (mcu_id=1) must be serviced at least once"
    );
    assert!(
        final_c2 >= 1,
        "EtherCAT MCU (mcu_id=2) must be serviced at least once"
    );
}

#[test]
fn ethercat_heartbeat_callback_advances_drain_and_pump() {
    let drain = Arc::new(DrainSync::new());
    let (pump_tx, pump_rx) = mpsc::channel::<PumpMsg>();

    drain.add_sent(42, 0, 3);

    let drain_hb = Arc::clone(&drain);
    let pump_tx_hb = pump_tx.clone();
    let mcu_id = 42u32;
    let callback: Arc<dyn Fn(&[u32]) + Send + Sync> = Arc::new(move |retired: &[u32]| {
        let _ = pump_tx_hb.send(PumpMsg::Heartbeat(HeartbeatMsg {
            mcu_id,
            retired_counts: retired.to_vec(),
        }));
        for (axis, &r) in retired.iter().enumerate() {
            drain_hb.set_retired(mcu_id, axis as u8, r);
        }
    });

    callback(&[3u32]);

    drain
        .wait_drained(Duration::from_millis(100))
        .expect("drain must complete after heartbeat callback fires with retired==sent");

    match pump_rx.recv_timeout(Duration::from_millis(100)) {
        Ok(PumpMsg::Heartbeat(hb)) => {
            assert_eq!(hb.mcu_id, 42, "Heartbeat.mcu_id must match");
            assert_eq!(
                hb.retired_counts,
                vec![3u32],
                "Heartbeat.retired_counts must match"
            );
        }
        Ok(_) => panic!("expected PumpMsg::Heartbeat"),
        Err(e) => panic!("pump did not receive Heartbeat: {e}"),
    }
}

#[test]
fn ethercat_heartbeat_partial_then_full_retirement() {
    let drain = Arc::new(DrainSync::new());
    let (pump_tx, _pump_rx) = mpsc::channel::<PumpMsg>();

    drain.add_sent(7, 0, 5);

    let drain_hb = Arc::clone(&drain);
    let pump_tx_hb = pump_tx.clone();
    let mcu_id = 7u32;
    let callback: Arc<dyn Fn(&[u32]) + Send + Sync> = Arc::new(move |retired: &[u32]| {
        let _ = pump_tx_hb.send(PumpMsg::Heartbeat(HeartbeatMsg {
            mcu_id,
            retired_counts: retired.to_vec(),
        }));
        for (axis, &r) in retired.iter().enumerate() {
            drain_hb.set_retired(mcu_id, axis as u8, r);
        }
    });

    callback(&[2u32]);
    assert!(
        drain.wait_drained(Duration::from_millis(20)).is_err(),
        "drain must not unblock with partial retirement (2/5)"
    );

    callback(&[5u32]);
    assert!(
        drain.wait_drained(Duration::from_millis(100)).is_ok(),
        "drain must unblock after full retirement (5/5)"
    );
}
