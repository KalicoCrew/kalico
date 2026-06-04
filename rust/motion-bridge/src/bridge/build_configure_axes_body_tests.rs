use super::*;

#[test]
fn build_configure_axes_body_legacy_20() {
    let body = build_configure_axes_body(
        0,
        0x0F,
        0x03,
        0,
        &[160.0, 160.0, 800.0, 800.0],
        None,
        None,
        0,
    );
    assert_eq!(body.len(), 20, "legacy body is 20 bytes");
    assert_eq!(body[0], 0);
    assert_eq!(body[1], 0x0F);
    assert_eq!(body[2], 0x03);
    assert_eq!(body[3], 0);
    assert_eq!(&body[4..8], &160.0f32.to_le_bytes());
    assert_eq!(&body[16..20], &800.0f32.to_le_bytes());
}

#[test]
fn build_configure_axes_body_step_modes_25() {
    let body = build_configure_axes_body(
        0,
        0x0F,
        0x03,
        0,
        &[160.0, 160.0, 800.0, 800.0],
        Some(&[0, 0, 1, 1]),
        None,
        1,
    );
    assert_eq!(body.len(), 25, "step-modes body is 25 bytes");
    assert_eq!(body[20], 1, "byte 20 carries phase_capable");
    assert_eq!(&body[21..25], &[0u8, 0, 1, 1], "step_modes array");
}

#[test]
fn build_configure_axes_body_phase_configs_variable_n1() {
    let body = build_configure_axes_body(
        1,
        0x0F,
        0x00,
        0,
        &[160.0, 160.0, 800.0, 800.0],
        Some(&[0, 1, 1, 1]),
        Some(&[(3, 5, 0)]),
        1,
    );
    assert_eq!(body.len(), 26 + 3 * 1, "N=1 body is 29 bytes");
    assert_eq!(body[20], 1, "byte 20 carries phase_capable");
    assert_eq!(&body[21..25], &[0u8, 1, 1, 1], "step_modes array");
    assert_eq!(body[25], 1, "byte 25 is phase_motor_count");
    assert_eq!(&body[26..29], &[3u8, 5, 0], "(bus, cs, slot_idx)");
}

#[test]
fn build_configure_axes_body_phase_configs_variable_n4_corexy_awd() {
    let body = build_configure_axes_body(
        0,
        0x0F,
        0x03,
        0,
        &[160.0, 160.0, 800.0, 800.0],
        Some(&[0, 0, 1, 1]),
        Some(&[(3, 5, 0), (3, 6, 0), (3, 7, 1), (3, 8, 1)]),
        1,
    );
    assert_eq!(body.len(), 26 + 3 * 4, "N=4 body is 38 bytes");
    assert_eq!(body[20], 1);
    assert_eq!(&body[21..25], &[0u8, 0, 1, 1]);
    assert_eq!(body[25], 4, "phase_motor_count");
    assert_eq!(
        &body[26..38],
        &[3u8, 5, 0, 3, 6, 0, 3, 7, 1, 3, 8, 1],
        "(bus, cs, slot_idx) triples for AWD-paired CoreXY motors",
    );
}

#[test]
fn build_configure_axes_body_phase_configs_variable_n8() {
    let entries: Vec<(u8, u8, u8)> = (0u8..8u8).map(|i| (3, 0x10 + i, i % 4)).collect();
    let body = build_configure_axes_body(
        1,
        0x0F,
        0x00,
        0,
        &[160.0, 160.0, 800.0, 800.0],
        Some(&[0, 0, 0, 0]),
        Some(&entries),
        1,
    );
    assert_eq!(body.len(), 26 + 3 * 8, "N=8 body is 50 bytes");
    assert_eq!(body[25], 8, "phase_motor_count");
    for (i, (bus, cs, slot)) in entries.iter().enumerate() {
        let off = 26 + i * 3;
        assert_eq!(body[off], *bus, "entry[{i}].bus");
        assert_eq!(body[off + 1], *cs, "entry[{i}].cs");
        assert_eq!(body[off + 2], *slot, "entry[{i}].slot");
    }
}
