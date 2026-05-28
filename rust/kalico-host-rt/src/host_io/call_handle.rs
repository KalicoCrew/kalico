use std::sync::mpsc::Sender;

use crate::host_io::ReactorCommand;

/// RAII guard around an in-flight `call()`. Drop sends `Abandon(call_id)` to
/// the reactor unless `defuse()` was called first; per spec §5.5 the call site
/// defuses on completion (Ok variant only) so timeout/disconnect paths still
/// notify the reactor.
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
