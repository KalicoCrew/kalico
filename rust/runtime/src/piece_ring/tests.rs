use super::*;

#[test]
fn piece_entry_to_le_bytes_matches_field_layout() {
    let p = PieceEntry {
        start_time: 0x0102_0304_0506_0708,
        coeffs: [1.0, 2.0, 3.0, 4.0],
        duration: 0.5,
        _reserved: 0,
    };
    let b = p.to_le_bytes();
    assert_eq!(b.len(), 32);
    assert_eq!(&b[0..8], &0x0102_0304_0506_0708u64.to_le_bytes());
    assert_eq!(&b[8..12], &1.0f32.to_le_bytes());
    assert_eq!(&b[12..16], &2.0f32.to_le_bytes());
    assert_eq!(&b[16..20], &3.0f32.to_le_bytes());
    assert_eq!(&b[20..24], &4.0f32.to_le_bytes());
    assert_eq!(&b[24..28], &0.5f32.to_le_bytes());
    assert_eq!(&b[28..32], &0u32.to_le_bytes());
}
