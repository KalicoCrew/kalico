use super::*;
use crate::dispatch::McuCaps;

fn small_caps() -> McuCaps {
    McuCaps {
        curve_pool_n: 4,
        max_pieces_per_curve: 16,
    }
}

#[test]
fn curve_load_params_over_cap_does_not_fit() {
    let caps = small_caps();
    // 17 pieces — exceeds firmware's MAX_PIECES_PER_CURVE = 16.
    let too_big = CurveLoadParams {
        bp_per_piece: vec![[0.0_f32; 4]; 17],
        duration_per_piece: vec![0.01_f32; 17],
    };
    assert!(!fits_curve_load(&caps, &too_big));

    // 8 pieces — well within the cap.
    let just_right = CurveLoadParams {
        bp_per_piece: vec![[0.0_f32; 4]; 8],
        duration_per_piece: vec![0.01_f32; 8],
    };
    assert!(fits_curve_load(&caps, &just_right));
}
