use super::*;
use crate::ConstructError;

fn linear_curve() -> ScalarNurbs<f64> {
    // Degree-1 NURBS, 2 control points, knots {0,0,1,1}.
    ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0]).unwrap()
}

#[test]
fn try_new_accepts_valid_linear() {
    let curve = linear_curve();
    assert_eq!(curve.degree(), 1);
    assert_eq!(curve.control_points(), &[0.0, 1.0]);
}

#[test]
fn try_new_rejects_degree_exceeded() {
    let result = ScalarNurbs::<f64>::try_new(21, vec![0.0; 23], vec![0.0; 1]);
    assert!(matches!(
        result,
        Err(ConstructError::DegreeExceeded {
            actual: 21,
            max: 20
        })
    ));
}

#[test]
fn try_new_rejects_knot_count_mismatch() {
    let result = ScalarNurbs::<f64>::try_new(
        1,
        vec![0.0, 0.0, 1.0], // 3 knots, but 2 cps + 1 + 1 = 4 expected
        vec![0.0, 1.0],
    );
    assert!(matches!(
        result,
        Err(ConstructError::KnotCountMismatch { .. })
    ));
}

#[test]
fn try_new_rejects_unclamped_start() {
    let result = ScalarNurbs::<f64>::try_new(
        1,
        vec![0.0, 0.5, 1.0, 1.0], // not clamped at start
        vec![0.0, 1.0],
    );
    assert!(matches!(result, Err(ConstructError::KnotsNotClamped)));
}

#[test]
fn try_new_rejects_unclamped_end() {
    let result = ScalarNurbs::<f64>::try_new(
        1,
        vec![0.0, 0.0, 0.5, 1.0], // not clamped at end
        vec![0.0, 1.0],
    );
    assert!(matches!(result, Err(ConstructError::KnotsNotClamped)));
}

#[test]
fn try_new_rejects_non_monotone_knots() {
    let result = ScalarNurbs::<f64>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.4, 0.3, 1.0, 1.0, 1.0], // 0.3 < 0.4
        vec![0.0, 0.5, 1.0, 1.5, 2.0],
    );
    assert!(matches!(result, Err(ConstructError::KnotsNotMonotone)));
}

#[test]
fn try_new_rejects_degenerate_knot_range() {
    let result = ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 0.0, 0.0], vec![0.0, 1.0]);
    assert!(matches!(result, Err(ConstructError::DegenerateKnotRange)));
}

#[test]
fn as_view_provides_borrowed_access() {
    let owned = linear_curve();
    let view = owned.as_view();
    assert_eq!(view.degree(), 1);
    assert_eq!(view.knots(), &[0.0, 0.0, 1.0, 1.0]);
    assert_eq!(view.control_points(), &[0.0, 1.0]);
}

#[test]
fn ref_try_new_accepts_valid_data() {
    let knots = [0.0_f64, 0.0, 1.0, 1.0];
    let cps = [0.0_f64, 1.0];
    let r = ScalarNurbsRef::try_new(1, &knots, &cps).unwrap();
    assert_eq!(r.degree(), 1);
}

#[test]
fn try_from_wire_parses_unweighted_linear() {
    // Layout: u8 version, u8 degree, u8 has_weights, u8 reserved,
    //         u16 knot_count, u16 cp_count, then knots + cps (both as f32).
    // Linear curve: degree=1, knots=[0,0,1,1], cps=[0.0, 1.0]
    let mut buf = Vec::new();
    buf.extend_from_slice(&[1, 1, 0, 0]); // version, degree, has_weights, reserved
    buf.extend_from_slice(&4u16.to_ne_bytes()); // knot_count
    buf.extend_from_slice(&2u16.to_ne_bytes()); // cp_count
    buf.extend_from_slice(&0.0_f32.to_ne_bytes());
    buf.extend_from_slice(&0.0_f32.to_ne_bytes());
    buf.extend_from_slice(&1.0_f32.to_ne_bytes());
    buf.extend_from_slice(&1.0_f32.to_ne_bytes());
    buf.extend_from_slice(&0.0_f32.to_ne_bytes());
    buf.extend_from_slice(&1.0_f32.to_ne_bytes());

    // Ensure 4-byte alignment by allocating into an aligned buffer
    let aligned = align_buf(&buf, 4);
    let r = ScalarNurbsRef::<f32>::try_from_wire(aligned.as_slice()).unwrap();
    assert_eq!(r.degree(), 1);
    assert_eq!(r.control_points(), &[0.0_f32, 1.0]);
}

#[test]
fn try_from_wire_rejects_misaligned_buffer() {
    let mut data = [0u8; 32 + 1];
    data[0] = 1;
    // Stack-array layout in release can land on an address where `&buf[1..]`
    // happens to be 4-aligned. Anchor on a 4-aligned base via align_buf, then
    // slice from offset 1 so misalignment for f32 is guaranteed.
    let aligned = align_buf(&data, 4);
    let result = ScalarNurbsRef::<f32>::try_from_wire(&aligned.as_slice()[1..]);
    assert!(matches!(result, Err(crate::WireError::Misaligned)));
}

#[test]
fn try_from_wire_rejects_unknown_version() {
    let buf = align_buf(
        &[
            0xFFu8, 1, 0, 0, 4, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ],
        4,
    );
    let result = ScalarNurbsRef::<f32>::try_from_wire(buf.as_slice());
    assert!(matches!(
        result,
        Err(crate::WireError::UnknownVersion(0xFF))
    ));
}

#[test]
fn try_from_wire_rejects_truncated_header() {
    let buf = align_buf(&[1u8, 1, 0, 0], 4); // only 4 bytes; 8-byte header missing
    let result = ScalarNurbsRef::<f32>::try_from_wire(buf.as_slice());
    assert!(matches!(
        result,
        Err(crate::WireError::TruncatedBuffer { .. })
    ));
}

#[test]
fn try_from_wire_rejects_has_weights_flag() {
    // Legacy rational header (has_weights=1) must be rejected loudly, not
    // parsed with a misaligned payload assumption.
    let buf = align_buf(
        &[
            1u8, 1, 1, 0, 4, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ],
        4,
    );
    let result = ScalarNurbsRef::<f32>::try_from_wire(buf.as_slice());
    assert!(matches!(result, Err(crate::WireError::WeightsUnsupported)));
}

/// Owns a 4-byte-aligned byte buffer for wire-format tests. The backing
/// storage is a `Vec<u32>` (alignment 4); we expose its bytes via `as_slice`.
/// Using a wrapper avoids the layout-mismatch UB that would arise from
/// transmuting `Vec<u32>` → `Vec<u8>` and letting the latter free with the
/// wrong alignment.
struct AlignedBytes {
    backing: Vec<u32>,
    len: usize,
}

impl AlignedBytes {
    fn as_slice(&self) -> &[u8] {
        // SAFETY: `Vec<u32>` is 4-byte aligned and `len <= backing.len() * 4`.
        // `u32` has no padding and any bit pattern is a valid `u8` byte.
        #[allow(unsafe_code)]
        unsafe {
            core::slice::from_raw_parts(self.backing.as_ptr().cast::<u8>(), self.len)
        }
    }
}

/// Allocate a buffer aligned to `align` bytes containing `data`.
fn align_buf(data: &[u8], align: usize) -> AlignedBytes {
    match align {
        4 => {
            let n = data.len().div_ceil(4);
            let mut backing: Vec<u32> = vec![0; n];
            // SAFETY: `backing` owns `n * 4` bytes with 4-byte alignment;
            // we write exactly `data.len() <= n * 4` bytes via the
            // `&mut [u8]` view, then release it before returning.
            #[allow(unsafe_code)]
            let bytes: &mut [u8] = unsafe {
                core::slice::from_raw_parts_mut(backing.as_mut_ptr().cast::<u8>(), n * 4)
            };
            bytes[..data.len()].copy_from_slice(data);
            AlignedBytes {
                backing,
                len: data.len(),
            }
        }
        _ => unimplemented!(),
    }
}
