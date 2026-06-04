use std::sync::mpsc::Sender;

use crate::host_io::ReactorCommand;

pub(crate) struct CallHandle {
    pub(crate) call_id: u64,
    pub(crate) submission_tx: Sender<ReactorCommand>,
}

impl CallHandle {
    pub(crate) fn defuse(self) {
        std::mem::forget(self);
    }
}

impl Drop for CallHandle {
    fn drop(&mut self) {
        let _ = self
            .submission_tx
            .send(ReactorCommand::Abandon(self.call_id));
    }
}

#[cfg(test)]
mod tests;
