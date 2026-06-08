use super::*;

#[test]
fn scalar_encoder_header_and_length() {
    let knots = [0.0_f32, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let cps = [0.0_f32, 3.33, 6.67, 10.0];
    let blob = encode_load_curve_scalar(3, &knots, &cps).unwrap();
    assert_eq!(blob[0], FORMAT_VERSION_V1);
    assert_eq!(blob[1], 3, "degree");
    assert_eq!(blob[2], 4, "num_cps");
    assert_eq!(blob[3], 8, "num_knots");
    assert_eq!(blob[4], 0, "num_weights (always 0 for scalar)");
    assert_eq!(blob.len(), 53);
}

#[test]
fn scalar_encoder_values_are_le() {
    let knots = [0.0_f32, 1.0];
    let cps = [1.5_f32];
    let blob = encode_load_curve_scalar(0, &knots, &cps).unwrap();
    let cp_bytes: [u8; 4] = blob[5..9].try_into().unwrap();
    assert_eq!(f32::from_le_bytes(cp_bytes), 1.5);
    let k0_bytes: [u8; 4] = blob[9..13].try_into().unwrap();
    assert_eq!(f32::from_le_bytes(k0_bytes), 0.0);
}

#[test]
fn header_and_length_are_correct() {
    let cps: [[f32; 3]; 2] = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]];
    let knots = [0.0_f32, 0.0, 1.0, 1.0];
    let blob = encode_load_curve_v1(1, &cps, &knots).unwrap();
    assert_eq!(blob[0], FORMAT_VERSION_V1);
    assert_eq!(blob[1], 1, "degree");
    assert_eq!(blob[2], 2, "num_cps");
    assert_eq!(blob[3], 4, "num_knots");
    assert_eq!(blob[4], 0, "num_weights (always 0)");
    assert_eq!(blob.len(), 5 + 24 + 16);
}

#[test]
fn encodes_floats_little_endian() {
    let cps = [[1.5_f32, 0.0, 0.0]];
    let knots = [0.0_f32];
    let blob = encode_load_curve_v1(0, &cps, &knots).unwrap();
    assert_eq!(&blob[5..9], &[0x00, 0x00, 0xC0, 0x3F]);
}

#[test]
fn count_overflow_returns_error_in_release() {
    let cps = vec![[0.0_f32; 3]; 256];
    let knots = [0.0_f32, 1.0];
    let err = encode_load_curve_v1(0, &cps, &knots).unwrap_err();
    assert!(matches!(
        err,
        WireError::CountOverflow {
            field: "num_cps",
            len: 256
        }
    ));

    let cps_scalar = vec![0.0_f32; 256];
    let err = encode_load_curve_scalar(0, &knots, &cps_scalar).unwrap_err();
    assert!(matches!(
        err,
        WireError::CountOverflow {
            field: "num_cps",
            len: 256
        }
    ));
}
