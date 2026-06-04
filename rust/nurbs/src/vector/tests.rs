use super::*;

fn linear_3d_curve() -> VectorNurbs<f64, 3> {
    VectorNurbs::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]],
    )
    .unwrap()
}

#[test]
fn try_new_accepts_valid_linear_3d() {
    let curve = linear_3d_curve();
    assert_eq!(curve.degree(), 1);
    assert_eq!(curve.control_points()[1], [1.0, 2.0, 3.0]);
}

#[test]
fn try_new_rejects_degree_exceeded() {
    let result = VectorNurbs::<f64, 3>::try_new(21, vec![0.0; 23], vec![[0.0; 3]; 1]);
    assert!(matches!(
        result,
        Err(crate::ConstructError::DegreeExceeded { .. })
    ));
}

#[test]
fn try_new_rejects_knot_count_mismatch() {
    let result = VectorNurbs::<f64, 3>::try_new(1, vec![0.0, 0.0, 1.0], vec![[0.0; 3], [1.0; 3]]);
    assert!(matches!(
        result,
        Err(crate::ConstructError::KnotCountMismatch { .. })
    ));
}

#[test]
fn as_view_provides_borrowed_access() {
    let owned = linear_3d_curve();
    let view = owned.as_view();
    assert_eq!(view.degree(), 1);
    assert_eq!(view.control_points()[1], [1.0, 2.0, 3.0]);
}

#[test]
fn try_from_wire_parses_3d_unweighted_linear() {
    // Layout: u8 version, u8 degree, u8 has_weights, u8 axes_n,
    //         u16 knot_count, u16 cp_count, then knots + cps (interleaved).
    let mut buf = Vec::new();
    buf.extend_from_slice(&[1, 1, 0, 3]); // version, degree, has_weights, axes_n
    buf.extend_from_slice(&4u16.to_ne_bytes()); // knot_count
    buf.extend_from_slice(&2u16.to_ne_bytes()); // cp_count
    buf.extend_from_slice(&0.0_f32.to_ne_bytes());
    buf.extend_from_slice(&0.0_f32.to_ne_bytes());
    buf.extend_from_slice(&1.0_f32.to_ne_bytes());
    buf.extend_from_slice(&1.0_f32.to_ne_bytes());
    // CPs interleaved: [0,0,0], [1,2,3]
    for &v in &[0.0_f32, 0.0, 0.0, 1.0, 2.0, 3.0] {
        buf.extend_from_slice(&v.to_ne_bytes());
    }
    let aligned = test_align_buf(&buf, 4);
    let r = VectorNurbsRef::<f32, 3>::try_from_wire(aligned.as_slice()).unwrap();
    assert_eq!(r.degree(), 1);
    assert_eq!(r.control_points()[1], [1.0, 2.0, 3.0]);
}

#[test]
fn try_from_wire_rejects_has_weights_flag() {
    // Legacy rational header (has_weights=1) must be rejected loudly, not
    // parsed with a misaligned payload assumption.
    let mut buf = Vec::new();
    buf.extend_from_slice(&[1, 1, 1, 3]); // version, degree, has_weights=1, axes_n
    buf.extend_from_slice(&4u16.to_ne_bytes());
    buf.extend_from_slice(&2u16.to_ne_bytes());
    buf.resize(64, 0);
    let aligned = test_align_buf(&buf, 4);
    let result = VectorNurbsRef::<f32, 3>::try_from_wire(aligned.as_slice());
    assert!(matches!(result, Err(crate::WireError::WeightsUnsupported)));
}

#[test]
fn try_from_wire_rejects_axis_mismatch() {
    // Wire says axes_n=4, but type is 3.
    let mut buf = Vec::new();
    buf.extend_from_slice(&[1, 1, 0, 4]);
    buf.extend_from_slice(&4u16.to_ne_bytes());
    buf.extend_from_slice(&2u16.to_ne_bytes());
    // pad to enough bytes so we get past the axis check
    buf.resize(64, 0);
    let aligned = test_align_buf(&buf, 4);
    let result = VectorNurbsRef::<f32, 3>::try_from_wire(aligned.as_slice());
    assert!(matches!(
        result,
        Err(crate::WireError::AxisCountMismatch {
            expected: 3,
            got: 4
        })
    ));
}

/// Test-only owner; same shape as `align_buf` in scalar.rs (see Task 9).
struct AlignedBytes {
    backing: Vec<u32>,
    len: usize,
}

impl AlignedBytes {
    fn as_slice(&self) -> &[u8] {
        // SAFETY: `Vec<u32>` is 4-byte aligned; len <= backing.len()*4.
        #[allow(unsafe_code)]
        unsafe {
            core::slice::from_raw_parts(self.backing.as_ptr().cast::<u8>(), self.len)
        }
    }
}

fn test_align_buf(data: &[u8], _align: usize) -> AlignedBytes {
    let n = data.len().div_ceil(4);
    let mut backing: Vec<u32> = vec![0; n];
    // SAFETY: backing owns n*4 bytes 4-byte aligned; we write data.len() <= n*4.
    #[allow(unsafe_code)]
    let bytes: &mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(backing.as_mut_ptr().cast::<u8>(), n * 4) };
    bytes[..data.len()].copy_from_slice(data);
    AlignedBytes {
        backing,
        len: data.len(),
    }
}
