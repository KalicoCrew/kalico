//! `TickState` — per-tick state shared with PA/IS slots. Spec §3.1.

#[derive(Debug, Clone, Copy)]
pub struct TickState {
    pub dt: f32,
    pub xyz_e: [f32; 3],
    pub motors: [f32; 3],
}
