use std::sync::mpsc::SyncSender;

use crate::host_io::runtime_events::FaultEvent as RuntimeFaultEvent;
use crate::transport::{MessageParams, SubscribeError};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaultEvent {
    pub fault_code: u16,
    pub fault_detail: u32,
    pub segment_id: u32,
}

pub fn parse_fault_event(params: &MessageParams) -> Option<FaultEvent> {
    // %hu on the wire is widened to i32 by Klipper's parser; re-narrow to u16.
    #[allow(clippy::cast_possible_truncation)]
    let fault_code = (params.get_u32("fault_code") & 0xFFFF) as u16;
    let fault_detail = params.get_u32("fault_detail");
    let segment_id = params.get_u32("segment_id");
    Some(FaultEvent {
        fault_code,
        fault_detail,
        segment_id,
    })
}

#[derive(Debug, Default)]
pub struct FaultLatch {
    pub cell: Option<RuntimeFaultEvent>,
    pub subscriber: Option<SyncSender<RuntimeFaultEvent>>,
}

impl FaultLatch {
    pub fn dispatch(&mut self, event: RuntimeFaultEvent) {
        let upgrade = self
            .cell
            .as_ref()
            .map(|c| c.synthesized && !event.synthesized)
            .unwrap_or(false);
        if self.cell.is_none() || upgrade {
            self.cell = Some(event.clone());
            if let Some(tx) = &self.subscriber {
                let _ = tx.send(event);
            }
        }
    }

    pub fn subscribe(&mut self, tx: SyncSender<RuntimeFaultEvent>) -> Result<(), SubscribeError> {
        if self.subscriber.is_some() {
            return Err(SubscribeError::AlreadySubscribed { channel: "fault" });
        }
        if let Some(latched) = &self.cell {
            let _ = tx.send(latched.clone());
        }
        self.subscriber = Some(tx);
        Ok(())
    }
}

#[cfg(test)]
mod fault_latch_tests;
