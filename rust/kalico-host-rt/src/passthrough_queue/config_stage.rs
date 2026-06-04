#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigStagePhase {
    Collecting,
    SendingConfig,
    SendingInit,
    Runtime,
}

#[derive(Debug)]
pub struct ConfigStage {
    config_cmds: Vec<Vec<u8>>,
    init_cmds: Vec<Vec<u8>>,
    restart_cmds: Vec<Vec<u8>>,
    phase: ConfigStagePhase,
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

    pub fn add_config_cmd(&mut self, bytes: Vec<u8>) -> bool {
        if self.phase != ConfigStagePhase::Collecting {
            return false;
        }
        self.config_cmds.push(bytes);
        true
    }

    pub fn add_init_cmd(&mut self, bytes: Vec<u8>) -> bool {
        if self.phase != ConfigStagePhase::Collecting {
            return false;
        }
        self.init_cmds.push(bytes);
        true
    }

    pub fn add_restart_cmd(&mut self, bytes: Vec<u8>) -> bool {
        if self.phase != ConfigStagePhase::Collecting {
            return false;
        }
        self.restart_cmds.push(bytes);
        true
    }

    pub fn restart_cmds(&self) -> &[Vec<u8>] {
        &self.restart_cmds
    }

    pub fn begin_config_send(&mut self) {
        self.phase = ConfigStagePhase::SendingConfig;
        self.cursor = 0;
    }

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
                    self.phase = ConfigStagePhase::SendingInit;
                    self.cursor = 0;
                }
                ConfigStagePhase::SendingInit => {
                    if self.cursor < self.init_cmds.len() {
                        let entry = self.init_cmds[self.cursor].clone();
                        self.cursor += 1;
                        return Some(entry);
                    }
                    self.phase = ConfigStagePhase::Runtime;
                    return None;
                }
                ConfigStagePhase::Runtime => return None,
            }
        }
    }
}

#[cfg(test)]
mod tests;
