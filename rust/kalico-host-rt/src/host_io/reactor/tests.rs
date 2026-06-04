use super::*;
use crate::host_io::wire;
use std::sync::{Arc, Mutex};

struct MockPort {
    written: Arc<Mutex<Vec<u8>>>,
}

impl std::io::Read for MockPort {
    fn read(&mut self, _buf: &mut [u8]) -> std::io::Result<usize> {
        Err(std::io::Error::new(std::io::ErrorKind::TimedOut, "mock"))
    }
}

impl std::io::Write for MockPort {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.written.lock().unwrap().extend_from_slice(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

impl serialport::SerialPort for MockPort {
    fn name(&self) -> Option<String> {
        Some("mock".into())
    }
    fn baud_rate(&self) -> serialport::Result<u32> {
        Ok(115_200)
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
    fn timeout(&self) -> std::time::Duration {
        std::time::Duration::from_millis(1)
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
    fn set_timeout(&mut self, _: std::time::Duration) -> serialport::Result<()> {
        Ok(())
    }
    fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
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
            "mock: try_clone unsupported",
        ))
    }
    fn set_break(&self) -> serialport::Result<()> {
        Ok(())
    }
    fn clear_break(&self) -> serialport::Result<()> {
        Ok(())
    }
    fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
}

fn test_reactor_with_inflight(seqs: &[u64]) -> (Reactor, Arc<Mutex<Vec<u8>>>) {
    let written = Arc::new(Mutex::new(Vec::<u8>::new()));
    let port = MockPort {
        written: Arc::clone(&written),
    };

    let parser = Arc::new(crate::host_io::parser::MsgProtoParser::new_empty());

    let (_, rx) = std::sync::mpsc::channel();
    let status_snapshot = Arc::new(arc_swap::ArcSwap::from_pointee(
        crate::host_io::runtime_events::StatusEvent::default(),
    ));

    let mut reactor = Reactor::new_for_tests(
        Box::new(port),
        parser,
        rx,
        status_snapshot,
        crate::host_io::KalicoHostIoConfig::default(),
        Arc::new(crate::clock::RealClock),
    );

    let max_seq = seqs.iter().copied().max().unwrap_or(0);
    for &seq in seqs {
        reactor
            .unacked_window
            .push(crate::host_io::window::UnackedEntry {
                seq,
                frame_bytes: vec![],
                sent_at: std::time::Instant::now(),
                retry_count: 0,
            });
    }
    if max_seq > 0 {
        reactor.send_seq = max_seq + 1;
    }

    (reactor, written)
}

#[test]
fn decode_absolute_wraps_correctly() {
    let (reactor, _) = test_reactor_with_inflight(&[]);
    assert_eq!(wire::decode_absolute(reactor.receive_seq, 0x02), 2);

    let mut r2 = test_reactor_with_inflight(&[]).0;
    r2.receive_seq = 14;
    assert_eq!(wire::decode_absolute(r2.receive_seq, 0x01), 17);
}

#[test]
fn forward_progress_ack_updates_last_ack_seq() {
    let (mut reactor, _written) = test_reactor_with_inflight(&[2]);

    reactor.handle_ack_nak(0x02).expect("handle_ack_nak");
    assert_eq!(reactor.last_ack_seq, 2);
}

#[test]
fn duplicate_ack_triggers_retransmit() {
    let (mut reactor, written) = test_reactor_with_inflight(&[1, 2]);

    reactor.handle_ack_nak(0x02).expect("first handle_ack_nak");
    assert_eq!(reactor.last_ack_seq, 2);

    let bytes_before = written.lock().unwrap().len();

    reactor.handle_ack_nak(0x02).expect("second handle_ack_nak");

    let bytes_after = written.lock().unwrap().len();
    assert!(
        bytes_after > bytes_before,
        "duplicate ack must trigger retransmit (write buffer grew: {bytes_before} → {bytes_after})"
    );
}

#[test]
fn stale_ack_damped_by_ignore_nak_seq() {
    let (mut reactor, written) = test_reactor_with_inflight(&[1, 2]);
    reactor.ignore_nak_seq = 10;

    reactor.handle_ack_nak(0x02).expect("first handle_ack_nak");

    let bytes_before = written.lock().unwrap().len();

    reactor.handle_ack_nak(0x02).expect("second handle_ack_nak");

    let bytes_after = written.lock().unwrap().len();
    assert_eq!(
        bytes_before, bytes_after,
        "ignore_nak_seq damps retransmit: write buffer must not grow"
    );
}

#[test]
fn nak_driven_sets_ignore_nak_to_receive_seq() {
    let (mut reactor, _port) = test_reactor_with_inflight(&[1, 2, 3]);
    reactor.receive_seq = 5;
    reactor.retransmit_seq = 0;
    reactor
        .write_retransmit(RetransmitTrigger::NakDriven)
        .unwrap();
    assert_eq!(reactor.ignore_nak_seq, 5);
}

#[test]
fn second_nak_uses_retransmit_seq() {
    let (mut reactor, _port) = test_reactor_with_inflight(&[1, 2, 3]);
    reactor.receive_seq = 3;
    reactor.retransmit_seq = 7;
    reactor
        .write_retransmit(RetransmitTrigger::NakDriven)
        .unwrap();
    assert_eq!(reactor.ignore_nak_seq, 7);
}

#[test]
fn timeout_driven_sets_ignore_nak_to_send_seq() {
    let (mut reactor, _port) = test_reactor_with_inflight(&[1, 2, 3]);
    reactor.send_seq = 10;
    reactor
        .write_retransmit(RetransmitTrigger::TimeoutDriven)
        .unwrap();
    assert_eq!(reactor.ignore_nak_seq, 10);
}

#[test]
fn nak_driven_does_not_back_off_rto() {
    let (mut reactor, _port) = test_reactor_with_inflight(&[1]);
    let rto_before = reactor.rtt.current_rto();
    reactor
        .write_retransmit(RetransmitTrigger::NakDriven)
        .unwrap();
    assert_eq!(reactor.rtt.current_rto(), rto_before);
}

#[test]
fn timeout_driven_doubles_rto() {
    let (mut reactor, _port) = test_reactor_with_inflight(&[1]);
    let rto_before = reactor.rtt.current_rto();
    reactor
        .write_retransmit(RetransmitTrigger::TimeoutDriven)
        .unwrap();
    assert!(reactor.rtt.current_rto() >= rto_before * 2);
}

struct BrokenPipePort;

impl std::io::Read for BrokenPipePort {
    fn read(&mut self, _buf: &mut [u8]) -> std::io::Result<usize> {
        Err(std::io::Error::new(
            std::io::ErrorKind::BrokenPipe,
            "mock disconnect",
        ))
    }
}

impl std::io::Write for BrokenPipePort {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        Ok(buf.len())
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}

impl serialport::SerialPort for BrokenPipePort {
    fn name(&self) -> Option<String> {
        Some("broken-pipe-mock".into())
    }
    fn baud_rate(&self) -> serialport::Result<u32> {
        Ok(115_200)
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
    fn timeout(&self) -> std::time::Duration {
        std::time::Duration::from_millis(1)
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
    fn set_timeout(&mut self, _: std::time::Duration) -> serialport::Result<()> {
        Ok(())
    }
    fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
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
            "mock: try_clone unsupported",
        ))
    }
    fn set_break(&self) -> serialport::Result<()> {
        Ok(())
    }
    fn clear_break(&self) -> serialport::Result<()> {
        Ok(())
    }
    fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
}

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
            "mock write fail",
        ))
    }
    fn flush(&mut self) -> std::io::Result<()> {
        Ok(())
    }
}
impl serialport::SerialPort for BrokenWritePort {
    fn name(&self) -> Option<String> {
        Some("broken-write-mock".into())
    }
    fn baud_rate(&self) -> serialport::Result<u32> {
        Ok(115_200)
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
    fn timeout(&self) -> std::time::Duration {
        std::time::Duration::from_millis(1)
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
    fn set_timeout(&mut self, _: std::time::Duration) -> serialport::Result<()> {
        Ok(())
    }
    fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
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
            "mock: try_clone unsupported",
        ))
    }
    fn set_break(&self) -> serialport::Result<()> {
        Ok(())
    }
    fn clear_break(&self) -> serialport::Result<()> {
        Ok(())
    }
    fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> {
        Ok(())
    }
    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        Ok(false)
    }
}

#[test]
fn drain_pending_surfaces_write_failure() {
    let (_, rx) = std::sync::mpsc::channel::<crate::host_io::ReactorCommand>();
    let status_snapshot = Arc::new(arc_swap::ArcSwap::from_pointee(
        crate::host_io::runtime_events::StatusEvent::default(),
    ));
    let parser = Arc::new(crate::host_io::parser::MsgProtoParser::new_empty());
    let mut reactor = Reactor::new_for_tests(
        Box::new(BrokenWritePort),
        parser,
        rx,
        status_snapshot,
        crate::host_io::KalicoHostIoConfig::default(),
        Arc::new(crate::clock::RealClock),
    );

    let (tx, completion_rx) =
        std::sync::mpsc::sync_channel::<Result<crate::transport::MessageParams, TransportError>>(1);
    reactor.pending_submissions.push_back(PendingSubmission {
        call_id: 7,
        payload: vec![0xAA, 0xBB],
        expected_response_name: "noop".into(),
        completion: tx,
        deadline: Instant::now() + std::time::Duration::from_secs(1),
    });
    reactor
        .pending_outbound_order
        .push_back(PendingOutboundKind::Submission);

    reactor.drain_pending_submissions();

    let received = completion_rx
        .try_recv()
        .expect("completion must be signaled");
    match received {
        Err(TransportError::Io(e)) => assert_eq!(e.kind(), std::io::ErrorKind::BrokenPipe),
        other => panic!("expected Io(BrokenPipe), got {other:?}"),
    }
    assert_eq!(
        reactor.state,
        ReactorState::Closed,
        "state must transition to Closed"
    );
    let fault = reactor
        .pending_host_fault
        .as_ref()
        .expect("host fault must be staged");
    assert_eq!(fault.fault_code, FaultCode::HostDisconnect.as_u16());
    assert!(
        reactor.pending_submissions.is_empty(),
        "draining must stop after I/O failure"
    );
}

#[test]
fn broken_pipe_latches_host_disconnect_fault() {
    let (_, rx) = std::sync::mpsc::channel::<crate::host_io::ReactorCommand>();
    let status_snapshot = Arc::new(arc_swap::ArcSwap::from_pointee(
        crate::host_io::runtime_events::StatusEvent::default(),
    ));
    let parser = Arc::new(crate::host_io::parser::MsgProtoParser::new_empty());
    let mut reactor = Reactor::new_for_tests(
        Box::new(BrokenPipePort),
        parser,
        rx,
        status_snapshot,
        crate::host_io::KalicoHostIoConfig::default(),
        Arc::new(crate::clock::RealClock),
    );

    reactor.run();

    assert!(
        reactor.event_dispatcher.fault_latch.cell.is_some(),
        "FaultLatch should have a cell after BrokenPipe"
    );
    let cell = reactor.event_dispatcher.fault_latch.cell.as_ref().unwrap();
    assert_eq!(
        cell.fault_code,
        FaultCode::HostDisconnect.as_u16(),
        "fault_code must be KALICO_ERR_HOST_DISCONNECT"
    );
    assert!(
        !cell.synthesized,
        "host disconnect fault is not synthesized"
    );
}
