pub const ERR_ENABLE_FAILED: i32 = -310;
pub const ERR_BAD_TORQUE_STATE: i32 = -312;
pub const ERR_PIECES_WHILE_PARKED: i32 = -313;
pub const ERR_PIECES_WHILE_FAULTED: i32 = -314;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TorqueState {
    Parked,
    Enabled,
    Faulted,
}

#[derive(Debug, PartialEq, Eq)]
pub enum CommandAction {
    Enable,
    ScheduleDisable,
    Reject { code: i32 },
}

#[derive(Debug, PartialEq, Eq)]
pub enum TickAction {
    None,
    ExecuteDisable,
    Fault { code: i32 },
}

#[derive(Debug)]
pub struct TorqueGate {
    state: TorqueState,
    pending_disable_at: Option<u64>,
}

impl TorqueGate {
    #[must_use]
    pub fn new() -> Self {
        Self {
            state: TorqueState::Parked,
            pending_disable_at: None,
        }
    }

    #[must_use]
    pub fn state(&self) -> TorqueState {
        self.state
    }

    pub fn on_set_torque(&mut self, value: bool, not_before_ns: u64) -> CommandAction {
        if value {
            if self.state == TorqueState::Enabled && self.pending_disable_at.is_none() {
                return CommandAction::Reject {
                    code: ERR_BAD_TORQUE_STATE,
                };
            }
            self.pending_disable_at = None;
            CommandAction::Enable
        } else {
            let can_disable = (self.state == TorqueState::Enabled
                || self.state == TorqueState::Faulted)
                && self.pending_disable_at.is_none();
            if !can_disable {
                return CommandAction::Reject {
                    code: ERR_BAD_TORQUE_STATE,
                };
            }
            self.pending_disable_at = Some(not_before_ns);
            CommandAction::ScheduleDisable
        }
    }

    pub fn on_drive_fault(&mut self) {
        self.state = TorqueState::Faulted;
        self.pending_disable_at = None;
    }

    pub fn enable_finished(&mut self, ok: bool) {
        if ok {
            self.state = TorqueState::Enabled;
        }
    }

    pub fn disable_finished(&mut self) {
        self.state = TorqueState::Parked;
        self.pending_disable_at = None;
    }

    pub fn on_tick(&mut self, now_ns: u64, ring_empty: bool) -> TickAction {
        if let Some(at) = self.pending_disable_at {
            if now_ns >= at {
                if !ring_empty {
                    return TickAction::Fault {
                        code: ERR_PIECES_WHILE_PARKED,
                    };
                }
                return TickAction::ExecuteDisable;
            }
        }
        if self.state == TorqueState::Parked && !ring_empty {
            return TickAction::Fault {
                code: ERR_PIECES_WHILE_PARKED,
            };
        }
        TickAction::None
    }
}

impl Default for TorqueGate {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests;
