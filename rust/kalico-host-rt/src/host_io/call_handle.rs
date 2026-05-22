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
mod tests {
    use super::*;

    #[test]
    fn drop_without_defuse_sends_abandon() {
        let (tx, rx) = std::sync::mpsc::channel();
        let handle = CallHandle {
            call_id: 42,
            submission_tx: tx,
        };
        drop(handle);
        match rx.recv().expect("channel must have a message") {
            ReactorCommand::Abandon(id) => assert_eq!(id, 42),
            other => panic!("expected Abandon(42), got {:?}", other),
        }
        assert!(rx.try_recv().is_err(), "channel must be empty after drain");
    }

    #[test]
    fn defuse_skips_abandon() {
        let (tx, rx) = std::sync::mpsc::channel();
        let handle = CallHandle {
            call_id: 99,
            submission_tx: tx,
        };
        handle.defuse();
        assert!(
            rx.try_recv().is_err(),
            "defused handle must not send Abandon"
        );
    }
}
