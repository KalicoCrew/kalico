use runtime::engine::Engine;
use runtime::piece_ring::PieceEntry;
use runtime::stepping_state::{MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};

fn new_engine() -> Engine {
    Engine::new(520_000_000, 40_000)
}

fn pulse_binding(oid: u8) -> StepperBindingRust {
    StepperBindingRust {
        stepper_oid: oid,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }
}

fn zero_entry() -> PieceEntry {
    PieceEntry {
        start_time: 0,
        coeffs: [0.0; 4],
        duration: 0.001,
        _reserved: 0,
    }
}

#[test]
fn unconfigured_axes_report_zero_occupancy() {
    let e = new_engine();
    assert_eq!(e.occupancy_counts(), [0u32; MAX_AXES]);
}

#[test]
fn empty_configured_axis_reports_zero_occupancy() {
    let mut e = new_engine();
    e.configure_axis(0, StepMode::Pulse, 0.01, 8, &[pulse_binding(0)], 64);
    let occ = e.occupancy_counts();
    assert_eq!(occ[0], 0);
}

#[test]
fn occupancy_tracks_pushed_and_retired_pieces() {
    let mut e = new_engine();
    e.configure_axis(0, StepMode::Pulse, 0.01, 8, &[pulse_binding(0)], 64);

    let mut storage = [zero_entry(); 64];

    e.push_pieces(0, &[zero_entry(), zero_entry(), zero_entry()], &mut storage);
    assert_eq!(e.occupancy_counts()[0], 3);

    assert_eq!(e.retired_counts()[0], 0);
}

#[test]
fn unconfigured_slots_remain_zero_when_one_axis_configured() {
    let mut e = new_engine();
    e.configure_axis(2, StepMode::Pulse, 0.01, 8, &[pulse_binding(0)], 64);

    let mut storage = [zero_entry(); 64];
    e.push_pieces(2, &[zero_entry()], &mut storage);

    let occ = e.occupancy_counts();
    assert_eq!(occ[0], 0);
    assert_eq!(occ[1], 0);
    assert_eq!(occ[2], 1);
    for i in 3..MAX_AXES {
        assert_eq!(occ[i], 0);
    }
}
