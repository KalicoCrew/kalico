use super::*;
use crate::host_io::ReactorCommand;
use crate::host_io::test_harness::ReactorHarness;
use std::sync::Arc;
use std::sync::mpsc::sync_channel;
use std::time::{Duration, Instant};
use kalico_native_transport;

/// Same as the BrokenWritePort declared in the inner `tests` module above
/// — duplicated here so this module sees a SerialPort. The inner module's
/// fixture is private to its scope.
struct BrokenWritePort;
impl std::io::Read for BrokenWritePort {
    fn read(&mut self, _buf: &mut [u8]) -> std::io::Result<usize> {
        Err(std::io::Error::new(std::io::ErrorKind::TimedOut, "mock"))
    }
}
impl std::io::Write for BrokenWritePort {
    fn write(&mut self, _buf: &[u8]) -> std::io::Result<usize> {
        Err(std::io::Error::new(
            std::io::ErrorKind::BrokenPipe,
            "mock fail",
        ))
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}
impl serialport::SerialPort for BrokenWritePort {
    fn name(&self) -> Option<String> {
        Some("broken".into())
    }
    fn baud_rate(&self) -> serialport::Result<u32> {
        Ok(0)
    }
    fn data_bits(&self) -> serialport::Result<serialport::DataBits> {
        Ok(serialport::DataBits::Eight)
    }
    fn flow_control(&self) -> serialport::Result<serialport::FlowControl> {
        Ok(serialport::FlowControl::None)
    }
    fn parity(&self) -> serialport::Result<serialport::Parity> {
        Ok(serialport::Parity::None)
    }
    fn stop_bits(&self) -> serialport::Result<serialport::StopBits> {
        Ok(serialport::StopBits::One)
    }
    fn timeout(&self) -> Duration {
        Duration::from_millis(1)
    }
    fn set_baud_rate(&mut self, _: u32) -> serialport::Result<()> {
        Ok(())
    }
    fn set_data_bits(&mut self, _: serialport::DataBits) -> serialport::Result<()> {
        Ok(())
    }
    fn set_flow_control(&mut self, _: serialport::FlowControl) -> serialport::Result<()> {
        Ok(())
    }
    fn set_parity(&mut self, _: serialport::Parity) -> serialport::Result<()> {
        Ok(())
    }
    fn set_stop_bits(&mut self, _: serialport::StopBits) -> serialport::Result<()> {
        Ok(())
    }
    fn set_timeout(&mut self, _: Duration) -> serialport::Result<()> {
        Ok(())
    }
    fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn bytes_to_read(&self) -> serialport::Result<u32> {
        Ok(0)
    }
    fn bytes_to_write(&self) -> serialport::Result<u32> {
        Ok(0)
    }
    fn clear(&self, _: serialport::ClearBuffer) -> serialport::Result<()> {
        Ok(())
    }
    fn try_clone(&self) -> serialport::Result<Box<dyn serialport::SerialPort>> {
        Err(serialport::Error::new(
            serialport::ErrorKind::Unknown,
            "unsupported",
        ))
    }
    fn set_break(&self) -> serialport::Result<()> {
        Ok(())
    }
    fn clear_break(&self) -> serialport::Result<()> {
        Ok(())
    }
}

fn fresh_reactor_with_broken_write() -> (Reactor, std::sync::mpsc::Sender<ReactorCommand>) {
    use crate::host_io::parser::MsgProtoParser;
    use crate::host_io::runtime_events::StatusEvent;
    use arc_swap::ArcSwap;
    let (tx, rx) = std::sync::mpsc::channel::<ReactorCommand>();
    let parser = Arc::new(MsgProtoParser::new_empty());
    let status_snapshot = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));
    let clock: Arc<dyn crate::clock::Clock> = Arc::new(crate::clock::RealClock);
    let reactor = Reactor::new_for_tests(
        Box::new(BrokenWritePort),
        parser,
        rx,
        status_snapshot,
        crate::host_io::KalicoHostIoConfig::default(),
        clock,
    );
    (reactor, tx)
}

/// SubmitTyped (the bridge_call path used by tmc.py SPI writes) hits
/// dispatch_submission's IMMEDIATE branch (window not full). On
/// TransportError::Io from write_frame, completion gets the Io error
/// AND the reactor transitions Closed + stages HostDisconnect.
#[test]
fn submit_typed_io_error_transitions_closed() {
    let (mut reactor, tx) = fresh_reactor_with_broken_write();
    let (completion_tx, completion_rx) = sync_channel(1);
    tx.send(ReactorCommand::SubmitTyped {
        call_id: 1,
        payload: vec![0xAA],
        expected_response_name: "noop".into(),
        completion: completion_tx,
        deadline: Instant::now() + Duration::from_secs(1),
    })
    .expect("submission_tx open");

    let outcome = reactor.tick_once();

    let result = completion_rx
        .try_recv()
        .expect("completion delivered within one tick");
    match result {
        Err(TransportError::Io(e)) => assert_eq!(e.kind(), std::io::ErrorKind::BrokenPipe),
        other => panic!("expected Io(BrokenPipe), got {other:?}"),
    }
    // POST-FIX (2026-05-09 Fix 1): reactor MUST transition Closed +
    // stage HostDisconnect. Mirrors drain_pending_submissions behavior.
    // Note: pending_host_fault gets drained into fault_latch within the
    // same tick (step 4b), so we check the latch.
    assert_eq!(
        reactor.state,
        ReactorState::Closed,
        "Fix 1: SubmitTyped's Io error MUST transition Closed"
    );
    let cell = reactor
        .event_dispatcher
        .fault_latch
        .cell
        .as_ref()
        .expect("Fix 1: HostDisconnect fault MUST be latched");
    assert_eq!(cell.fault_code, FaultCode::HostDisconnect.as_u16());
    // Submit doesn't reach awaiting_response.push since write failed.
    assert!(reactor.unacked_window.is_empty());
    // tick_once detects Closed in step 6 within the SAME tick that
    // staged the fault. Outcome is Closed (not Continue).
    assert_eq!(
        outcome,
        TickOutcome::Closed,
        "tick_once returns Closed when state transitioned this tick"
    );
    // send_seq still advances (write attempted, just failed).
    assert_eq!(reactor.send_seq, 2);
}

/// FireAndForgetTyped (passthrough fire-and-forget) — same fix.
#[test]
fn fire_and_forget_typed_io_error_transitions_closed() {
    let (mut reactor, tx) = fresh_reactor_with_broken_write();
    tx.send(ReactorCommand::FireAndForgetTyped {
        payload: vec![0x11, 0x22, 0x33],
    })
    .expect("submission_tx open");

    let outcome = reactor.tick_once();

    assert_eq!(
        reactor.state,
        ReactorState::Closed,
        "Fix 1: FireAndForgetTyped's Io error MUST transition Closed"
    );
    assert!(
        reactor.event_dispatcher.fault_latch.cell.is_some(),
        "Fix 1: HostDisconnect fault MUST be latched"
    );
    assert_eq!(outcome, TickOutcome::Closed);
}

/// KalicoCall (motion-bridge producer load_curve / push_segment) — same fix.
#[test]
fn kalico_call_io_error_transitions_closed() {
    use kalico_protocol::MessageKind;
    let (mut reactor, tx) = fresh_reactor_with_broken_write();
    reactor.kalico_state.identified = true;
    reactor.kalico_state.reset_epoch = Some(0);

    let (completion_tx, completion_rx) = sync_channel(1);
    tx.send(ReactorCommand::KalicoCall {
        channel: kalico_native_transport::CHANNEL_CONTROL,
        kind: MessageKind::PushPieces,
        body: vec![0; 16],
        completion: completion_tx,
        deadline: Instant::now() + Duration::from_secs(1),
    })
    .expect("submission_tx open");

    let outcome = reactor.tick_once();

    let result = completion_rx.try_recv().expect("completion delivered");
    match result {
        Err(TransportError::Io(e)) => assert_eq!(e.kind(), std::io::ErrorKind::BrokenPipe),
        other => panic!("expected Io(BrokenPipe), got {other:?}"),
    }
    assert_eq!(
        reactor.state,
        ReactorState::Closed,
        "Fix 1: KalicoCall's Io error MUST transition Closed"
    );
    assert!(
        reactor.event_dispatcher.fault_latch.cell.is_some(),
        "Fix 1: HostDisconnect fault MUST be latched"
    );
    assert_eq!(outcome, TickOutcome::Closed);
    assert!(
        reactor.kalico_state.pending.is_empty(),
        "pending entry cleaned up before Closed"
    );
}

/// Multi-Submit hammer: a single Io fault should immediately close the
/// transport, so subsequent Submits in the same tick batch are flushed
/// with Err(Closed) (or, post-tick, the next tick_once returns Closed
/// outcome and flush_all_completions fires).
#[test]
fn one_io_fault_closes_transport_no_storm() {
    let (mut reactor, tx) = fresh_reactor_with_broken_write();

    let mut completion_rxs = Vec::new();
    for i in 0..20u64 {
        let (ctx, crx) = sync_channel(1);
        tx.send(ReactorCommand::SubmitTyped {
            call_id: i,
            payload: vec![i as u8],
            expected_response_name: "noop".into(),
            completion: ctx,
            deadline: Instant::now() + Duration::from_secs(60),
        })
        .expect("submission_tx open");
        completion_rxs.push(crx);
    }

    // First tick processes MAX_SUBMITS_PER_ITER=4 commands.
    // The first Submit's Io error transitions state=Closed.
    // Subsequent Submits in the same batch still get Io errors via
    // their completion channels (they were already enqueued before
    // state transitioned). Outcome is Closed (state went Closed in
    // this tick).
    let outcome1 = reactor.tick_once();
    assert_eq!(
        outcome1,
        TickOutcome::Closed,
        "first Io fault closes the transport"
    );
    assert_eq!(reactor.state, ReactorState::Closed);

    // At least the first 4 (one batch) got Io. The exact count depends
    // on how many made it through before Closed transitioned. We just
    // assert SOMETHING was delivered and the transport is dead now.
    let mut delivered_io = 0;
    for rx in &completion_rxs {
        if let Ok(Err(_)) = rx.try_recv() {
            delivered_io += 1;
        }
    }
    assert!(
        delivered_io >= 1,
        "at least one Submit got an error response; got {delivered_io}"
    );
}

/// Drain path — already correct pre-fix, regression-guard test.
#[test]
fn drain_path_transitions_closed_on_io_error() {
    let _ = ReactorHarness::new();
    let (mut reactor, _tx) = fresh_reactor_with_broken_write();

    for seq in 0..crate::host_io::window::MAX_PENDING_BLOCKS as u64 {
        reactor
            .unacked_window
            .push(crate::host_io::window::UnackedEntry {
                seq,
                frame_bytes: vec![],
                sent_at: Instant::now(),
                retry_count: 0,
            });
    }
    reactor.send_seq = crate::host_io::window::MAX_PENDING_BLOCKS as u64;

    let (completion_tx, completion_rx) = sync_channel(1);
    reactor.pending_submissions.push_back(PendingSubmission {
        call_id: 99,
        payload: vec![0xCC],
        expected_response_name: "noop".into(),
        completion: completion_tx,
        deadline: Instant::now() + Duration::from_secs(1),
    });
    reactor
        .pending_outbound_order
        .push_back(PendingOutboundKind::Submission);

    reactor.unacked_window.pop_acked(1);
    reactor.drain_pending_submissions();

    let result = completion_rx.try_recv().expect("completion delivered");
    assert!(matches!(result, Err(TransportError::Io(_))));
    assert_eq!(reactor.state, ReactorState::Closed);
    // pending_host_fault stays Some here because we called drain
    // directly without going through tick_once (which would consume
    // it into fault_latch in step 4b).
    assert!(reactor.pending_host_fault.is_some());
}

// ---------------------------------------------------------------------
// Fix 6+7: RTO retransmit error escalation + fire-storm rate limit.
// ---------------------------------------------------------------------

/// FlakyWritePort: succeeds on the first N writes, then fails Io(Other).
/// Used to model the observed real-hardware failure: the initial Submit
/// goes out fine (after a long block), but subsequent retransmits fail.
struct FlakyWritePort {
    writes_until_fail: std::sync::atomic::AtomicU32,
}
impl FlakyWritePort {
    fn new(succeed_count: u32) -> Self {
        Self {
            writes_until_fail: std::sync::atomic::AtomicU32::new(succeed_count),
        }
    }
}
impl std::io::Read for FlakyWritePort {
    fn read(&mut self, _buf: &mut [u8]) -> std::io::Result<usize> {
        Err(std::io::Error::new(std::io::ErrorKind::TimedOut, "mock"))
    }
}
impl std::io::Write for FlakyWritePort {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        use std::sync::atomic::Ordering;
        let remaining = self.writes_until_fail.load(Ordering::Relaxed);
        if remaining > 0 {
            self.writes_until_fail
                .store(remaining - 1, Ordering::Relaxed);
            Ok(buf.len())
        } else {
            Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                "mock retransmit fail",
            ))
        }
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}
impl serialport::SerialPort for FlakyWritePort {
    fn name(&self) -> Option<String> {
        Some("flaky".into())
    }
    fn baud_rate(&self) -> serialport::Result<u32> {
        Ok(0)
    }
    fn data_bits(&self) -> serialport::Result<serialport::DataBits> {
        Ok(serialport::DataBits::Eight)
    }
    fn flow_control(&self) -> serialport::Result<serialport::FlowControl> {
        Ok(serialport::FlowControl::None)
    }
    fn parity(&self) -> serialport::Result<serialport::Parity> {
        Ok(serialport::Parity::None)
    }
    fn stop_bits(&self) -> serialport::Result<serialport::StopBits> {
        Ok(serialport::StopBits::One)
    }
    fn timeout(&self) -> Duration {
        Duration::from_millis(1)
    }
    fn set_baud_rate(&mut self, _: u32) -> serialport::Result<()> {
        Ok(())
    }
    fn set_data_bits(&mut self, _: serialport::DataBits) -> serialport::Result<()> {
        Ok(())
    }
    fn set_flow_control(&mut self, _: serialport::FlowControl) -> serialport::Result<()> {
        Ok(())
    }
    fn set_parity(&mut self, _: serialport::Parity) -> serialport::Result<()> {
        Ok(())
    }
    fn set_stop_bits(&mut self, _: serialport::StopBits) -> serialport::Result<()> {
        Ok(())
    }
    fn set_timeout(&mut self, _: Duration) -> serialport::Result<()> {
        Ok(())
    }
    fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn bytes_to_read(&self) -> serialport::Result<u32> {
        Ok(0)
    }
    fn bytes_to_write(&self) -> serialport::Result<u32> {
        Ok(0)
    }
    fn clear(&self, _: serialport::ClearBuffer) -> serialport::Result<()> {
        Ok(())
    }
    fn try_clone(&self) -> serialport::Result<Box<dyn serialport::SerialPort>> {
        Err(serialport::Error::new(
            serialport::ErrorKind::Unknown,
            "unsupported",
        ))
    }
    fn set_break(&self) -> serialport::Result<()> {
        Ok(())
    }
    fn clear_break(&self) -> serialport::Result<()> {
        Ok(())
    }
}

fn fresh_reactor_with_flaky_port(
    succeed_count: u32,
) -> (Reactor, std::sync::mpsc::Sender<ReactorCommand>) {
    use crate::host_io::parser::MsgProtoParser;
    use crate::host_io::runtime_events::StatusEvent;
    use arc_swap::ArcSwap;
    let (tx, rx) = std::sync::mpsc::channel::<ReactorCommand>();
    let parser = Arc::new(MsgProtoParser::new_empty());
    let status_snapshot = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));
    let clock: Arc<dyn crate::clock::Clock> = Arc::new(crate::clock::RealClock);
    let reactor = Reactor::new_for_tests(
        Box::new(FlakyWritePort::new(succeed_count)),
        parser,
        rx,
        status_snapshot,
        crate::host_io::KalicoHostIoConfig::default(),
        clock,
    );
    (reactor, tx)
}

/// Replay the real-hardware failure from 2026-05-09 (10:51:28 trace):
/// initial Submit's write succeeds (kernel finally flushed after 509ms);
/// reactor's RTO timer fires immediately because clock jumped 509ms;
/// retransmit's write fails Io(Other,None). Pre-fix: reactor ignored
/// the error and looped 5+ times in 4ms. Post-fix: ONE retransmit
/// failure is enough to transition Closed.
#[test]
fn rto_retransmit_io_error_transitions_closed() {
    // Allow exactly 1 successful write (the initial Submit), then fail.
    let (mut reactor, tx) = fresh_reactor_with_flaky_port(1);

    let (completion_tx, completion_rx) = sync_channel(1);
    tx.send(ReactorCommand::SubmitTyped {
        call_id: 1,
        payload: vec![0xAA],
        expected_response_name: "noop".into(),
        completion: completion_tx,
        deadline: Instant::now() + Duration::from_secs(60),
    })
    .expect("submission_tx open");

    // Tick 1: process Submit, write succeeds. Awaiter is in
    // awaiting_response. Front of unacked_window is fresh.
    let outcome1 = reactor.tick_once();
    assert_eq!(outcome1, TickOutcome::Continue);
    assert_eq!(reactor.state, ReactorState::Active);
    assert_eq!(reactor.unacked_window.len(), 1);
    assert_eq!(reactor.awaiting_response.len(), 1);

    // Force RTO to fire: pretend a long time passed since the front
    // entry was sent. (Default RTO is MIN_RTO = 25ms.)
    if let Some(front_mut) = reactor.unacked_window.iter_mut().next() {
        front_mut.sent_at = Instant::now() - Duration::from_secs(1);
    }

    // Tick 2: RTO fires, write_retransmit's write_frame fails Io(Other),
    // transition_closed_on_io_fault MUST run. tick_once returns Closed
    // in the same tick.
    let outcome2 = reactor.tick_once();
    assert_eq!(
        outcome2,
        TickOutcome::Closed,
        "Fix 6: RTO retransmit Io error closes transport in same tick"
    );
    assert_eq!(reactor.state, ReactorState::Closed);
    // fault_latch holds the HostDisconnect after step 4b drains.
    assert!(
        reactor.event_dispatcher.fault_latch.cell.is_some(),
        "Fix 6: HostDisconnect fault MUST be latched"
    );

    // The awaiter was flushed with Closed.
    let result = completion_rx
        .try_recv()
        .expect("flush_all_completions delivered");
    assert!(
        matches!(result, Err(TransportError::Closed)),
        "klippy gets Closed (not Io) via flush_all_completions; got {result:?}"
    );
}
