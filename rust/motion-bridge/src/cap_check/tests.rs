use super::*;
use crate::dispatch::McuCaps;

/// Small caps: total_piece_memory = 4 * 4 * 32 = 512 bytes
/// → total_pieces() = 16, max_pieces_per_curve = 16 / 4 = 4.
fn small_caps() -> McuCaps {
    McuCaps {
        // 4 axes × 4 pieces/axis × 32 bytes/piece = 512 bytes total.
        total_piece_memory: 512,
    }
}

#[test]
fn curve_load_params_over_cap_does_not_fit() {
    let caps = small_caps();
    // derived max_pieces_per_curve = 512/32/4 = 4; 5 pieces exceeds that.
    let too_big = CurveLoadParams {
        bp_per_piece: vec![[0.0_f32; 4]; 5],
        duration_per_piece: vec![0.01_f32; 5],
    };
    assert!(!fits_curve_load(&caps, &too_big));

    // 4 pieces — exactly at the cap.
    let just_right = CurveLoadParams {
        bp_per_piece: vec![[0.0_f32; 4]; 4],
        duration_per_piece: vec![0.01_f32; 4],
    };
    assert!(fits_curve_load(&caps, &just_right));
}
