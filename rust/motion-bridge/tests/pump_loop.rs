use std::sync::mpsc;
use std::sync::{Arc, Mutex};

use motion_bridge_native::pump::{
    AxisKey, EnqueueMsg, HeartbeatMsg, PieceSink, PumpMsg, SendError, run_pump,
};
use runtime::piece_ring::PieceEntry;

struct RecordingSink(Arc<Mutex<Vec<(AxisKey, usize)>>>);
impl PieceSink for RecordingSink {
    fn send_frame(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        _start_slot: u16,
        _new_head: u32,
    ) -> Result<i32, SendError> {
        self.0.lock().unwrap().push((key, pieces.len()));
        Ok(0)
    }
}

fn p(start: u64) -> (PieceEntry, f64) {
    (
        PieceEntry {
            start_time: start,
            coeffs: [0.0; 4],
            duration: 0.001,
            _reserved: 0,
        },
        start as f64,
    )
}

#[test]
fn pump_stalls_on_ring_full_resumes_on_heartbeat() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (tx, rx) = mpsc::channel();
    let depth = |_k: AxisKey| 2u32;
    let sink = RecordingSink(rec.clone());
    let handle =
        std::thread::spawn(move || run_pump(rx, sink, depth, |_| None, |_| {}, |_, _| {}, |_| {}));

    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: AxisKey { mcu_id: 1, axis: 0 },
        pieces: vec![p(0), p(1)],
        fresh_stream: true,
        lead_secs: motion_bridge_native::pump::MAX_LEAD_SECS,
    }))
    .unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: AxisKey { mcu_id: 1, axis: 0 },
        pieces: vec![p(2)],
        fresh_stream: false,
        lead_secs: motion_bridge_native::pump::MAX_LEAD_SECS,
    }))
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    assert_eq!(
        rec.lock().unwrap().len(),
        1,
        "first frame (2 pieces) sent, third stalled"
    );
    assert_eq!(rec.lock().unwrap()[0], (AxisKey { mcu_id: 1, axis: 0 }, 2));

    tx.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        retired_counts: vec![2],
    }))
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    assert_eq!(rec.lock().unwrap().len(), 2);
    assert_eq!(rec.lock().unwrap()[1], (AxisKey { mcu_id: 1, axis: 0 }, 1));

    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

fn piece_at(start: u64, host: f64, start_pos: f32, end_pos: f32) -> (PieceEntry, f64) {
    (
        PieceEntry {
            start_time: start,
            coeffs: [start_pos, start_pos, end_pos, end_pos],
            duration: 0.001,
            _reserved: 0,
        },
        host,
    )
}

fn run_pump_with_clock(
    rx: mpsc::Receiver<PumpMsg>,
    rec: Arc<Mutex<Vec<(AxisKey, usize)>>>,
) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        run_pump(
            rx,
            RecordingSink(rec),
            |_k| 64u32,
            |_mcu| Some((0u64, 1e6_f64)),
            |_| {},
            |_, _| {},
            |_| {},
        )
    })
}

#[test]
fn continuous_junction_position_passes() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (tx, rx) = mpsc::channel();
    let handle = run_pump_with_clock(rx, rec.clone());

    let key = AxisKey { mcu_id: 1, axis: 0 };
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key,
        pieces: vec![piece_at(0, 0.0, 10.0, 12.5)],
        fresh_stream: true,
        lead_secs: motion_bridge_native::pump::MAX_LEAD_SECS,
    }))
    .unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key,
        pieces: vec![piece_at(2000, 0.002, 12.5, 15.0)],
        fresh_stream: false,
        lead_secs: motion_bridge_native::pump::MAX_LEAD_SECS,
    }))
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    let sent_pieces: usize = rec.lock().unwrap().iter().map(|(_, n)| n).sum();
    assert_eq!(sent_pieces, 2);

    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}

#[test]
fn junction_position_discontinuity_is_fatal() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (tx, rx) = mpsc::channel();
    let handle = run_pump_with_clock(rx, rec.clone());

    let key = AxisKey { mcu_id: 1, axis: 0 };
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key,
        pieces: vec![piece_at(0, 0.0, 10.0, 12.5)],
        fresh_stream: true,
        lead_secs: motion_bridge_native::pump::MAX_LEAD_SECS,
    }))
    .unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key,
        pieces: vec![piece_at(2000, 0.002, 12.8, 15.0)],
        fresh_stream: false,
        lead_secs: motion_bridge_native::pump::MAX_LEAD_SECS,
    }))
    .unwrap();

    assert!(
        handle.join().is_err(),
        "0.3mm junction position jump must panic the pump"
    );
}

#[test]
fn fresh_stream_resets_junction_position_baseline() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (tx, rx) = mpsc::channel();
    let handle = run_pump_with_clock(rx, rec.clone());

    let key = AxisKey { mcu_id: 1, axis: 0 };
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key,
        pieces: vec![piece_at(0, 0.0, 10.0, 12.5)],
        fresh_stream: true,
        lead_secs: motion_bridge_native::pump::MAX_LEAD_SECS,
    }))
    .unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key,
        pieces: vec![piece_at(2000, 0.002, 50.0, 55.0)],
        fresh_stream: true,
        lead_secs: motion_bridge_native::pump::MAX_LEAD_SECS,
    }))
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    let sent_pieces: usize = rec.lock().unwrap().iter().map(|(_, n)| n).sum();
    assert_eq!(sent_pieces, 2);

    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}
