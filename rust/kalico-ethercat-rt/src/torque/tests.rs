use super::*;

const T0: u64 = 1_000_000_000;

#[test]
fn enable_from_parked_runs_ladder() {
    let mut g = TorqueGate::new();
    assert_eq!(g.state(), TorqueState::Parked);
    assert_eq!(g.on_set_torque(true, T0, T0 - 1), CommandAction::Enable);
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
    assert_eq!(g.on_tick(T0 + 499, true), TickAction::None);
    assert_eq!(g.on_tick(T0 + 500, true), TickAction::ExecuteDisable);
    g.disable_finished();
    assert_eq!(g.state(), TorqueState::Parked);
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
fn reenable_with_pending_disable_cancels_it() {
    let mut g = TorqueGate::new();
    let _ = g.on_set_torque(true, T0, T0 - 1);
    g.enable_finished(true);
    let _ = g.on_set_torque(false, T0 + 500, T0);
    assert_eq!(g.on_tick(T0 + 100, false), TickAction::None);
    assert_eq!(
        g.on_set_torque(true, T0 + 600, T0 + 100),
        CommandAction::Enable
    );
    g.enable_finished(true);
    assert_eq!(g.state(), TorqueState::Enabled);
    assert_eq!(g.on_tick(T0 + 1_000, false), TickAction::None);
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
    assert_eq!(g.on_tick(T0 + 100, false), TickAction::None);
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
