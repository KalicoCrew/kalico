use std::sync::mpsc;
use std::sync::{Arc, Mutex};

use motion_bridge_native::pump::{
    AxisKey, EnqueueMsg, HeartbeatMsg, PieceSink, PumpMsg, run_pump,
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
    ) -> Result<i32, String> {
        self.0.lock().unwrap().push((key, pieces.len()));
        Ok(0)
    }
}

fn p(start: u64) -> PieceEntry {
    PieceEntry { start_time: start, coeffs: [0.0; 4], duration: 0.001, _reserved: 0 }
}

#[test]
fn pump_stalls_on_ring_full_resumes_on_heartbeat() {
    let rec = Arc::new(Mutex::new(Vec::new()));
    let (tx, rx) = mpsc::channel();
    let depth = |_k: AxisKey| 2u32;
    let sink = RecordingSink(rec.clone());
    let handle = std::thread::spawn(move || run_pump(rx, sink, depth));

    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: AxisKey { mcu_id: 1, axis: 0 },
        pieces: vec![p(0), p(1)],
        fresh_stream: true,
    }))
    .unwrap();
    tx.send(PumpMsg::Enqueue(EnqueueMsg {
        key: AxisKey { mcu_id: 1, axis: 0 },
        pieces: vec![p(2)],
        fresh_stream: false,
    }))
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    assert_eq!(
        rec.lock().unwrap().len(),
        1,
        "first frame (2 pieces) sent, third stalled"
    );
    assert_eq!(
        rec.lock().unwrap()[0],
        (AxisKey { mcu_id: 1, axis: 0 }, 2)
    );

    tx.send(PumpMsg::Heartbeat(HeartbeatMsg {
        mcu_id: 1,
        retired_counts: vec![2],
    }))
    .unwrap();
    std::thread::sleep(std::time::Duration::from_millis(50));
    assert_eq!(rec.lock().unwrap().len(), 2);
    assert_eq!(
        rec.lock().unwrap()[1],
        (AxisKey { mcu_id: 1, axis: 0 }, 1)
    );

    tx.send(PumpMsg::Shutdown).unwrap();
    handle.join().unwrap();
}
