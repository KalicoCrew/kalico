//! Torque-gate state machine shared by the hw endpoint and the stub.
//!
//! Pure decision logic — no I/O, no FFI, no clock reads. The caller owns the
//! effects (CiA 402 ladder, disable ramp, heartbeat, process exit) and feeds
//! outcomes back via `enable_finished` / `disable_finished`.

/// SetTorqueResponse / fault result codes. Extends the endpoint's -30x
/// family (-308 piece-start-in-past, -309 ring-full).
pub const ERR_ENABLE_FAILED: i32 = -310;
pub const ERR_DISABLE_IN_PAST: i32 = -311;
pub const ERR_BAD_TORQUE_STATE: i32 = -312;
pub const ERR_PIECES_WHILE_PARKED: i32 = -313;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TorqueState {
    /// No torque commanded: CiA 402 Ready-to-Switch-On (controlword 0x0006).
    Parked,
    /// Torque commanded: CiA 402 Operation Enabled (controlword 0x000F).
    Enabled,
}

/// What the caller must do in response to a SetTorque command.
#[derive(Debug, PartialEq, Eq)]
pub enum CommandAction {
    /// Run the CiA 402 enable ladder, then reply with its result and report
    /// via `enable_finished`. `ramp_first`: a pending scheduled disable
    /// existed (it was scheduled earlier in print-time order); execute the
    /// disable ramp and call `disable_finished` before the ladder.
    Enable { ramp_first: bool },
    /// Disable accepted: reply result=0 now; the ramp runs from `on_tick`.
    ScheduleDisable,
    /// Invalid command: reply with `code`, then halt (disable + exit).
    Reject { code: i32 },
}

/// What the caller must do on a DC-loop tick.
#[derive(Debug, PartialEq, Eq)]
pub enum TickAction {
    None,
    /// The scheduled disable is due: run the ramp, call `disable_finished`.
    ExecuteDisable,
    /// Pieces exist while no torque is commanded: fault `code`, halt.
    Fault {
        code: i32,
    },
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

    pub fn on_set_torque(&mut self, value: bool, execute_at_ns: u64, now_ns: u64) -> CommandAction {
        if value {
            if self.state == TorqueState::Enabled && self.pending_disable_at.is_none() {
                return CommandAction::Reject {
                    code: ERR_BAD_TORQUE_STATE,
                };
            }
            let ramp_first = self.pending_disable_at.take().is_some();
            CommandAction::Enable { ramp_first }
        } else {
            if self.state != TorqueState::Enabled || self.pending_disable_at.is_some() {
                return CommandAction::Reject {
                    code: ERR_BAD_TORQUE_STATE,
                };
            }
            if execute_at_ns <= now_ns {
                return CommandAction::Reject {
                    code: ERR_DISABLE_IN_PAST,
                };
            }
            self.pending_disable_at = Some(execute_at_ns);
            CommandAction::ScheduleDisable
        }
    }

    /// Caller reports the enable ladder outcome.
    pub fn enable_finished(&mut self, ok: bool) {
        if ok {
            self.state = TorqueState::Enabled;
        }
    }

    /// Caller reports the disable ramp has run.
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
mod tests {
    use super::*;

    const T0: u64 = 1_000_000_000;

    #[test]
    fn enable_from_parked_runs_ladder() {
        let mut g = TorqueGate::new();
        assert_eq!(g.state(), TorqueState::Parked);
        assert_eq!(
            g.on_set_torque(true, T0, T0 - 1),
            CommandAction::Enable { ramp_first: false }
        );
        g.enable_finished(true);
        assert_eq!(g.state(), TorqueState::Enabled);
    }

    #[test]
    fn failed_ladder_leaves_gate_parked() {
        let mut g = TorqueGate::new();
        let _ = g.on_set_torque(true, T0, T0 - 1);
        g.enable_finished(false);
        assert_eq!(g.state(), TorqueState::Parked);
    }

    #[test]
    fn double_enable_rejected() {
        let mut g = TorqueGate::new();
        let _ = g.on_set_torque(true, T0, T0 - 1);
        g.enable_finished(true);
        assert_eq!(
            g.on_set_torque(true, T0 + 1, T0),
            CommandAction::Reject {
                code: ERR_BAD_TORQUE_STATE
            }
        );
    }

    #[test]
    fn disable_while_parked_rejected() {
        let mut g = TorqueGate::new();
        assert_eq!(
            g.on_set_torque(false, T0 + 1, T0),
            CommandAction::Reject {
                code: ERR_BAD_TORQUE_STATE
            }
        );
    }

    #[test]
    fn disable_schedules_then_tick_executes_at_time() {
        let mut g = TorqueGate::new();
        let _ = g.on_set_torque(true, T0, T0 - 1);
        g.enable_finished(true);
        assert_eq!(
            g.on_set_torque(false, T0 + 500, T0),
            CommandAction::ScheduleDisable
        );
        // not due yet
        assert_eq!(g.on_tick(T0 + 499, true), TickAction::None);
        // due, ring empty
        assert_eq!(g.on_tick(T0 + 500, true), TickAction::ExecuteDisable);
        g.disable_finished();
        assert_eq!(g.state(), TorqueState::Parked);
        // no re-fire
        assert_eq!(g.on_tick(T0 + 501, true), TickAction::None);
    }

    #[test]
    fn disable_in_past_rejected() {
        let mut g = TorqueGate::new();
        let _ = g.on_set_torque(true, T0, T0 - 1);
        g.enable_finished(true);
        assert_eq!(
            g.on_set_torque(false, T0, T0),
            CommandAction::Reject {
                code: ERR_DISABLE_IN_PAST
            }
        );
    }

    #[test]
    fn double_disable_rejected() {
        let mut g = TorqueGate::new();
        let _ = g.on_set_torque(true, T0, T0 - 1);
        g.enable_finished(true);
        let _ = g.on_set_torque(false, T0 + 500, T0);
        assert_eq!(
            g.on_set_torque(false, T0 + 600, T0),
            CommandAction::Reject {
                code: ERR_BAD_TORQUE_STATE
            }
        );
    }

    #[test]
    fn reenable_with_pending_disable_ramps_first() {
        // M84 schedules a disable; a new move's enable arrives before it is
        // due. The earlier-scheduled disable must still happen (stepper
        // semantics: both edges occur), so the gate orders ramp-then-ladder.
        let mut g = TorqueGate::new();
        let _ = g.on_set_torque(true, T0, T0 - 1);
        g.enable_finished(true);
        let _ = g.on_set_torque(false, T0 + 500, T0);
        assert_eq!(
            g.on_set_torque(true, T0 + 600, T0 + 100),
            CommandAction::Enable { ramp_first: true }
        );
        g.disable_finished();
        g.enable_finished(true);
        assert_eq!(g.state(), TorqueState::Enabled);
        // pending disable consumed — never fires
        assert_eq!(g.on_tick(T0 + 1_000, true), TickAction::None);
    }

    #[test]
    fn pieces_while_parked_fault() {
        let mut g = TorqueGate::new();
        assert_eq!(
            g.on_tick(T0, false),
            TickAction::Fault {
                code: ERR_PIECES_WHILE_PARKED
            }
        );
    }

    #[test]
    fn pieces_at_disable_time_fault() {
        let mut g = TorqueGate::new();
        let _ = g.on_set_torque(true, T0, T0 - 1);
        g.enable_finished(true);
        let _ = g.on_set_torque(false, T0 + 500, T0);
        // motion still draining before the deadline is fine
        assert_eq!(g.on_tick(T0 + 100, false), TickAction::None);
        // pieces still present AT the deadline = host bug
        assert_eq!(
            g.on_tick(T0 + 500, false),
            TickAction::Fault {
                code: ERR_PIECES_WHILE_PARKED
            }
        );
    }

    #[test]
    fn enabled_idle_ticks_are_quiet() {
        let mut g = TorqueGate::new();
        let _ = g.on_set_torque(true, T0, T0 - 1);
        g.enable_finished(true);
        assert_eq!(g.on_tick(T0 + 10, true), TickAction::None);
        assert_eq!(g.on_tick(T0 + 10, false), TickAction::None);
    }
}
