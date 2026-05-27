use super::*;

#[test]
fn corexy_with_e_round_trip() {
    // Inverse: x = (A + B) / 2, y = (A - B) / 2. Z and E pass through.
    let cases = [
        ([0.0_f32, 0.0, 0.0, 0.0], [0.0_f32, 0.0, 0.0, 0.0]),
        ([1.0, 0.0, 0.0, 0.0], [1.0, 1.0, 0.0, 0.0]),
        ([0.0, 1.0, 0.0, 0.0], [1.0, -1.0, 0.0, 0.0]),
        ([1.5, 2.5, 3.0, 7.0], [4.0, -1.0, 3.0, 7.0]),
        ([-3.0, 4.0, 1.0, -2.0], [1.0, -7.0, 1.0, -2.0]),
    ];
    let bits = |a: [f32; 4]| a.map(f32::to_bits);
    for (xyz_e, expected_motors) in cases {
        let motors = corexy_with_e(xyz_e);
        assert_eq!(bits(motors), bits(expected_motors), "transform({xyz_e:?})");

        // Round-trip via inverse.
        let xyz_e_back = [
            f32::midpoint(motors[0], motors[1]),
            (motors[0] - motors[1]) / 2.0,
            motors[2], // Z passthrough
            motors[3], // E passthrough
        ];
        assert_eq!(bits(xyz_e_back), bits(xyz_e), "round-trip({xyz_e:?})");
    }
}
