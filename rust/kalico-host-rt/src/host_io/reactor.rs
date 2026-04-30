//! Single-thread poll-reactor. Spec §3.7.

use std::collections::VecDeque;
use std::sync::mpsc::Receiver;
use std::sync::Arc;
use std::time::{Duration, Instant};

use arc_swap::ArcSwap;

use crate::host_io::ReactorCommand;
use crate::host_io::parser::MsgProtoParser;
use crate::host_io::rtt::RttEstimator;
use crate::host_io::runtime_events::{FaultEvent, StatusEvent};
use crate::host_io::window::{UnackedWindow, AwaitingResponse};
use crate::transport::TransportError;

pub struct Reactor {
    pub(crate) port:               Box<dyn serialport::SerialPort>,
    pub(crate) parser:             MsgProtoParser,
    pub(crate) submission_rx:      Receiver<ReactorCommand>,
    pub(crate) unacked_window:     UnackedWindow,
    pub(crate) awaiting_response:  AwaitingResponse,
    pub(crate) rtt:                RttEstimator,
    pub(crate) rx_buf:             Vec<u8>,
    pub(crate) status_snapshot:    Arc<ArcSwap<StatusEvent>>,

    // 64-bit absolute sequence counters. Per spec §3.1 / serialqueue.c:660-666.
    pub(crate) send_seq:           u64,
    pub(crate) receive_seq:        u64,
    pub(crate) last_ack_seq:       u64,
    pub(crate) ignore_nak_seq:     u64,
    pub(crate) retransmit_seq:     u64,
    pub(crate) rtt_sample_seq:     u64,
    pub(crate) rtt_sample_armed:   bool,

    pub(crate) state: ReactorState,

    pub(crate) pending_host_fault: Option<FaultEvent>,

    pub(crate) pending_submissions: VecDeque<PendingSubmission>,
}

pub(crate) struct PendingSubmission {
    pub call_id:                u64,
    pub payload:                Vec<u8>,
    pub expected_response_name: String,
    pub completion:             std::sync::mpsc::SyncSender<Result<crate::transport::MessageParams, TransportError>>,
    pub deadline:               Instant,
}

#[derive(Debug, PartialEq, Eq)]
pub enum ReactorState {
    Active,
    Closed,
}

impl Reactor {
    pub fn new(
        port: Box<dyn serialport::SerialPort>,
        parser: MsgProtoParser,
        submission_rx: Receiver<ReactorCommand>,
        status_snapshot: Arc<ArcSwap<StatusEvent>>,
        rx_buf_initial: Vec<u8>,
    ) -> Self {
        Self {
            port,
            parser,
            submission_rx,
            unacked_window: UnackedWindow::default(),
            awaiting_response: AwaitingResponse::default(),
            rtt: RttEstimator::default(),
            rx_buf: rx_buf_initial,
            status_snapshot,
            send_seq: 1,
            receive_seq: 1,
            last_ack_seq: 0,
            ignore_nak_seq: 0,
            retransmit_seq: 0,
            rtt_sample_seq: 0,
            rtt_sample_armed: false,
            state: ReactorState::Active,
            pending_host_fault: None,
            pending_submissions: VecDeque::new(),
        }
    }

    /// Single chokepoint for all wire writes. Per spec §3.7.
    pub(crate) fn write_frame(&mut self, frame: &[u8]) -> Result<(), TransportError> {
        self.port.write_all(frame).map_err(TransportError::Io)?;
        self.port.flush().map_err(TransportError::Io)?;
        Ok(())
    }
}
