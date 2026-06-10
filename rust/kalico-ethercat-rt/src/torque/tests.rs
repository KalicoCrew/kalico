use super::*;

const T0: u64 = 1_000_000_000;

#[test]
fn enable_from_parked_runs_ladder() {
    let mut g = TorqueGate::new();
    assert_eq!(g.state(), TorqueState::Parked);
    assert_eq!(g.on_set_torque(true, T0), CommandAction::Enable);
    g.enable_finished(true);
    assert_eq!(g.state(), TorqueState::Enabled);
}

#[test]
fn failed_ladder_leaves_gate_parked() {
    let mut g = TorqueGate::new();
    let _ = g.on_set_torque(true, T0);
    g.enable_finished(false);
    assert_eq!(g.state(), TorqueState::Parked);
}

#[test]
fn double_enable_rejected() {
    let mut g = TorqueGate::new();
    let _ = g.on_set_torque(true, T0);
    g.enable_finished(true);
    assert_eq!(
        g.on_set_torque(true, T0 + 1),
        CommandAction::Reject {
            code: ERR_BAD_TORQUE_STATE
        }
    );
}

#[test]
fn disable_while_parked_rejected() {
    let mut g = TorqueGate::new();
    assert_eq!(
        g.on_set_torque(false, T0 + 1),
        CommandAction::Reject {
            code: ERR_BAD_TORQUE_STATE
        }
    );
}

#[test]
fn disable_schedules_then_tick_executes_at_time() {
    let mut g = TorqueGate::new();
    let _ = g.on_set_torque(true, T0);
    g.enable_finished(true);
    assert_eq!(
        g.on_set_torque(false, T0 + 500),
        CommandAction::ScheduleDisable
    );
    assert_eq!(g.on_tick(T0 + 499, true), TickAction::None);
    assert_eq!(g.on_tick(T0 + 500, true), TickAction::ExecuteDisable);
    g.disable_finished();
    assert_eq!(g.state(), TorqueState::Parked);
    assert_eq!(g.on_tick(T0 + 501, true), TickAction::None);
}

#[test]
fn disable_in_past_executes_on_next_tick() {
    let mut g = TorqueGate::new();
    let _ = g.on_set_torque(true, T0);
    g.enable_finished(true);
    assert_eq!(
        g.on_set_torque(false, T0 - 500),
        CommandAction::ScheduleDisable
    );
    assert_eq!(g.on_tick(T0, true), TickAction::ExecuteDisable);
    g.disable_finished();
    assert_eq!(g.state(), TorqueState::Parked);
}

#[test]
fn double_disable_rejected() {
    let mut g = TorqueGate::new();
    let _ = g.on_set_torque(true, T0);
    g.enable_finished(true);
    let _ = g.on_set_torque(false, T0 + 500);
    assert_eq!(
        g.on_set_torque(false, T0 + 600),
        CommandAction::Reject {
            code: ERR_BAD_TORQUE_STATE
        }
    );
}

#[test]
fn reenable_with_pending_disable_cancels_it() {
    let mut g = TorqueGate::new();
    let _ = g.on_set_torque(true, T0);
    g.enable_finished(true);
    let _ = g.on_set_torque(false, T0 + 500);
    assert_eq!(g.on_tick(T0 + 100, false), TickAction::None);
    assert_eq!(
        g.on_set_torque(true, T0 + 600),
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
    let _ = g.on_set_torque(true, T0);
    g.enable_finished(true);
    let _ = g.on_set_torque(false, T0 + 500);
    assert_eq!(g.on_tick(T0 + 100, false), TickAction::None);
    assert_eq!(
        g.on_tick(T0 + 500, false),
        TickAction::Fault {
            code: ERR_PIECES_WHILE_PARKED
        }
    );
}

#[test]
fn drive_fault_parks_in_faulted_and_clears_pending_disable() {
    let mut g = TorqueGate::new();
    assert_eq!(g.on_set_torque(true, 0, 0), CommandAction::Enable);
    g.enable_finished(true);
    assert_eq!(
        g.on_set_torque(false, 100, 50),
        CommandAction::ScheduleDisable
    );
    g.on_drive_fault();
    assert_eq!(g.state(), TorqueState::Faulted);
    assert_eq!(g.on_tick(200, true), TickAction::None);
}

#[test]
fn faulted_tick_with_pieces_is_not_a_fault() {
    let mut g = TorqueGate::new();
    g.on_drive_fault();
    assert_eq!(g.on_tick(0, false), TickAction::None);
}

#[test]
fn enable_from_faulted_recovers() {
    let mut g = TorqueGate::new();
    g.on_drive_fault();
    assert_eq!(g.on_set_torque(true, 0, 0), CommandAction::Enable);
    g.enable_finished(true);
    assert_eq!(g.state(), TorqueState::Enabled);
}

#[test]
fn disable_from_faulted_schedules_and_lands_parked() {
    let mut g = TorqueGate::new();
    g.on_drive_fault();
    assert_eq!(
        g.on_set_torque(false, 100, 50),
        CommandAction::ScheduleDisable
    );
    assert_eq!(g.on_tick(150, true), TickAction::ExecuteDisable);
    g.disable_finished();
    assert_eq!(g.state(), TorqueState::Parked);
}

#[test]
fn enabled_idle_ticks_are_quiet() {
    let mut g = TorqueGate::new();
    let _ = g.on_set_torque(true, T0);
    g.enable_finished(true);
    assert_eq!(g.on_tick(T0 + 10, true), TickAction::None);
    assert_eq!(g.on_tick(T0 + 10, false), TickAction::None);
}
