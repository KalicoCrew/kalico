use super::*;

#[test]
#[allow(clippy::no_effect_underscore_binding)]
fn g5_missing_tangent_constructs() {
    let _r = Recovery::G5MissingTangent { line_no: 42 };
}

#[test]
#[allow(clippy::no_effect_underscore_binding)]
fn g5_plane_mismatch_constructs() {
    let _r = Recovery::G5PlaneMismatch {
        line_no: 42,
        active_plane_g_code: 18,
    };
}
