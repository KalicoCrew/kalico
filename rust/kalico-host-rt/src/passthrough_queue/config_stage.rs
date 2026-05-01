//! Config-stage command queues — `config_cmds` / `init_cmds` / `restart_cmds`.
//!
//! During MCU startup, klippy's `_send_config()` sends three categories of
//! commands in strict order: configuration first, then init, then the MCU
//! enters normal runtime. Restart commands are recorded separately for
//! reconnect scenarios.

/// Phase of the config-stage drain sequence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigStagePhase {
    /// Commands are being registered; nothing is being drained yet.
    Collecting,
    /// Draining `config_cmds`.
    SendingConfig,
    /// Draining `init_cmds` (config_cmds already fully drained).
    SendingInit,
    /// All config/init commands have been sent; normal runtime.
    Runtime,
}

/// Holds the three command lists and the current drain phase.
#[derive(Debug)]
pub struct ConfigStage {
    config_cmds: Vec<Vec<u8>>,
    init_cmds: Vec<Vec<u8>>,
    restart_cmds: Vec<Vec<u8>>,
    phase: ConfigStagePhase,
    /// Cursor into the currently-draining list.
    cursor: usize,
}

impl Default for ConfigStage {
    fn default() -> Self {
        Self {
            config_cmds: Vec::new(),
            init_cmds: Vec::new(),
            restart_cmds: Vec::new(),
            phase: ConfigStagePhase::Collecting,
            cursor: 0,
        }
    }
}

impl ConfigStage {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn phase(&self) -> ConfigStagePhase {
        self.phase
    }

    /// Append a configuration command. Only valid during `Collecting`.
    ///
    /// Returns `false` if called after `begin_config_send()`.
    pub fn add_config_cmd(&mut self, bytes: Vec<u8>) -> bool {
        if self.phase != ConfigStagePhase::Collecting {
            return false;
        }
        self.config_cmds.push(bytes);
        true
    }

    /// Append an init command. Only valid during `Collecting`.
    pub fn add_init_cmd(&mut self, bytes: Vec<u8>) -> bool {
        if self.phase != ConfigStagePhase::Collecting {
            return false;
        }
        self.init_cmds.push(bytes);
        true
    }

    /// Append a restart command. Only valid during `Collecting`.
    pub fn add_restart_cmd(&mut self, bytes: Vec<u8>) -> bool {
        if self.phase != ConfigStagePhase::Collecting {
            return false;
        }
        self.restart_cmds.push(bytes);
        true
    }

    /// Read-only view of restart commands (used by reconnect logic).
    pub fn restart_cmds(&self) -> &[Vec<u8>] {
        &self.restart_cmds
    }

    /// Transition from `Collecting` to `SendingConfig` to begin draining.
    pub fn begin_config_send(&mut self) {
        self.phase = ConfigStagePhase::SendingConfig;
        self.cursor = 0;
    }

    /// Return the next config/init entry in order. Transitions from
    /// `SendingConfig` → `SendingInit` → `Runtime` as lists drain.
    /// Returns `None` once all commands have been yielded.
    pub fn next_config_entry(&mut self) -> Option<Vec<u8>> {
        loop {
            match self.phase {
                ConfigStagePhase::Collecting => return None,
                ConfigStagePhase::SendingConfig => {
                    if self.cursor < self.config_cmds.len() {
                        let entry = self.config_cmds[self.cursor].clone();
                        self.cursor += 1;
                        return Some(entry);
                    }
                    // Config list exhausted — advance to init.
                    self.phase = ConfigStagePhase::SendingInit;
                    self.cursor = 0;
                }
                ConfigStagePhase::SendingInit => {
                    if self.cursor < self.init_cmds.len() {
                        let entry = self.init_cmds[self.cursor].clone();
                        self.cursor += 1;
                        return Some(entry);
                    }
                    // Init list exhausted — enter runtime.
                    self.phase = ConfigStagePhase::Runtime;
                    return None;
                }
                ConfigStagePhase::Runtime => return None,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_cmds_drain_before_init_cmds() {
        let mut cs = ConfigStage::new();
        cs.add_config_cmd(vec![0x01]);
        cs.add_config_cmd(vec![0x02]);
        cs.add_init_cmd(vec![0x0A]);
        cs.add_init_cmd(vec![0x0B]);

        cs.begin_config_send();

        assert_eq!(cs.next_config_entry(), Some(vec![0x01]));
        assert_eq!(cs.next_config_entry(), Some(vec![0x02]));
        // Now init commands.
        assert_eq!(cs.next_config_entry(), Some(vec![0x0A]));
        assert_eq!(cs.next_config_entry(), Some(vec![0x0B]));
        // Done.
        assert_eq!(cs.next_config_entry(), None);
        assert_eq!(cs.phase(), ConfigStagePhase::Runtime);
    }

    #[test]
    fn begin_config_send_transitions_correctly() {
        let mut cs = ConfigStage::new();
        assert_eq!(cs.phase(), ConfigStagePhase::Collecting);

        cs.begin_config_send();
        assert_eq!(cs.phase(), ConfigStagePhase::SendingConfig);

        // With no commands, immediately transitions through to Runtime.
        assert_eq!(cs.next_config_entry(), None);
        assert_eq!(cs.phase(), ConfigStagePhase::Runtime);
    }

    #[test]
    fn cannot_add_commands_after_begin_config_send() {
        let mut cs = ConfigStage::new();
        assert!(cs.add_config_cmd(vec![0x01]));
        assert!(cs.add_init_cmd(vec![0x02]));
        assert!(cs.add_restart_cmd(vec![0x03]));

        cs.begin_config_send();

        assert!(!cs.add_config_cmd(vec![0x04]));
        assert!(!cs.add_init_cmd(vec![0x05]));
        assert!(!cs.add_restart_cmd(vec![0x06]));
    }

    #[test]
    fn restart_cmds_are_stored_and_retrievable() {
        let mut cs = ConfigStage::new();
        cs.add_restart_cmd(vec![0xAA]);
        cs.add_restart_cmd(vec![0xBB]);

        assert_eq!(cs.restart_cmds().len(), 2);
        assert_eq!(cs.restart_cmds()[0], vec![0xAA]);
        assert_eq!(cs.restart_cmds()[1], vec![0xBB]);
    }

    #[test]
    fn next_config_entry_returns_none_during_collecting() {
        let mut cs = ConfigStage::new();
        cs.add_config_cmd(vec![0x01]);
        // Cannot drain without begin_config_send.
        assert_eq!(cs.next_config_entry(), None);
    }
}
