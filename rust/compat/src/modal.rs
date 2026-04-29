//! Modal G-code state tracking for the compat layer.
//!
//! Tracks position, feed rate, coordinate mode, and active plane as the
//! converter processes commands sequentially.

/// The active work plane (G17/G18/G19).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum Plane {
    /// G17 — XY plane (default).
    #[default]
    XY,
    /// G18 — XZ plane.
    XZ,
    /// G19 — YZ plane.
    YZ,
}

/// Modal state carried forward through the G-code stream.
#[derive(Debug, Clone)]
pub struct ModalState {
    /// Current absolute XYZ position in mm.
    pub position: [f64; 3],

    /// Input-side E accumulator (tracks what the original file's E words said).
    pub input_e: f64,

    /// Output-side E accumulator (tracks what we have written to the output).
    pub output_e: f64,

    /// Current feedrate in mm/min, if one has been set.
    pub feedrate_mm_min: Option<f64>,

    /// `true` = G90 absolute XYZ mode; `false` = G91 relative mode.
    pub absolute_xyz: bool,

    /// `true` = M82 absolute E mode; `false` = M83 relative E mode.
    pub absolute_e: bool,

    /// Active work plane.
    pub active_plane: Plane,

    /// G5 implicit-tangent chain: the (P, Q) offset of the previous G5 move,
    /// used to derive the default I,J for the next chained G5.
    pub prev_g5_pq: Option<[f64; 2]>,

    /// Boundary tangent for segment-to-segment handoff (reserved for future
    /// spline-fitter use; `None` until a smooth segment sets it).
    pub prev_tangent: Option<[f64; 2]>,
}

impl ModalState {
    /// Construct a `ModalState` in the standard power-on defaults:
    /// G90 (absolute XYZ), M82 (absolute E), G17 (XY plane), origin at zero.
    pub fn new() -> Self {
        Self {
            position: [0.0; 3],
            input_e: 0.0,
            output_e: 0.0,
            feedrate_mm_min: None,
            absolute_xyz: true,
            absolute_e: true,
            active_plane: Plane::default(),
            prev_g5_pq: None,
            prev_tangent: None,
        }
    }
}

impl Default for ModalState {
    fn default() -> Self {
        Self::new()
    }
}
