use std::sync::mpsc::Sender;

use crate::host_io::ReactorCommand;

pub struct CallHandle {
    pub call_id:       u64,
    pub submission_tx: Sender<ReactorCommand>,
    pub defused:       bool,
}

impl CallHandle {
    pub fn defuse(mut self) {
        self.defused = true;
    }
}

impl Drop for CallHandle {
    fn drop(&mut self) {
        if !self.defused {
            let _ = self.submission_tx.send(ReactorCommand::Abandon(self.call_id));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drop_without_defuse_sends_abandon() {
        let (tx, rx) = std::sync::mpsc::channel();
        let handle = CallHandle { call_id: 42, submission_tx: tx, defused: false };
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
        let handle = CallHandle { call_id: 99, submission_tx: tx, defused: false };
        handle.defuse();
        assert!(rx.try_recv().is_err(), "defused handle must not send Abandon");
    }
}
