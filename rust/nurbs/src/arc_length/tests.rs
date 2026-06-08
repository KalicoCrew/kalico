use super::*;

#[allow(clippy::float_cmp)]
#[test]
fn ref_provides_borrowed_access() {
    let s = [0.0_f64, 0.5, 1.0];
    let u = [0.0_f64, 0.4, 1.0];
    let r = ArcLengthTableRef::new(&s, &u);
    assert_eq!(r.s_max(), 1.0);
    assert_eq!(r.u_max(), 1.0);
}

#[cfg(feature = "host")]
#[allow(clippy::float_cmp)]
#[test]
fn owned_as_view_round_trips() {
    let owned = ArcLengthTable::new(vec![0.0, 0.5, 1.0], vec![0.0, 0.4, 1.0]);
    let view = owned.as_view();
    assert_eq!(view.s_max(), 1.0);
}

#[cfg(feature = "host")]
#[test]
fn integrate_constant_returns_length_times_constant() {
    // ∫_0^1 of f(u)=2 should be 2.
    let result = integrate_arc_length(|_u: f64| 2.0_f64, 0.0, 1.0, 5);
    assert!((result - 2.0).abs() < 1e-12);
}

#[cfg(feature = "host")]
#[test]
fn integrate_linear_matches_closed_form() {
    // ∫_0^1 of f(u)=u should be 0.5.
    let result = integrate_arc_length(|u: f64| u, 0.0, 1.0, 5);
    assert!((result - 0.5).abs() < 1e-12);
}

#[cfg(feature = "host")]
#[test]
fn integrate_quadratic_matches_closed_form() {
    // ∫_0^1 of f(u)=u^2 should be 1/3. 5-point Gauss-Legendre is exact for degree <= 9.
    let result = integrate_arc_length(|u: f64| u * u, 0.0, 1.0, 5);
    assert!((result - 1.0 / 3.0).abs() < 1e-12);
}

#[cfg(feature = "host")]
#[allow(clippy::float_cmp)]
#[test]
fn build_scalar_table_for_linear_curve() {
    let curve =
        crate::ScalarNurbs::try_new(1, vec![0.0_f64, 0.0, 1.0, 1.0], vec![0.0, 1.0]).unwrap();
    let table = build_arc_length_table_scalar(&curve, 1e-6, 64).unwrap();
    assert!((table.s_max() - 1.0).abs() < 1e-6);
    assert_eq!(table.u_max(), 1.0);
    for w in table.s().windows(2) {
        assert!(w[1] >= w[0]);
    }
    for w in table.u().windows(2) {
        assert!(w[1] >= w[0]);
    }
}

#[allow(clippy::float_cmp)]
#[test]
fn param_from_arc_length_at_endpoints() {
    let table = ArcLengthTableRef::new(&[0.0_f64, 0.5, 1.0], &[0.0, 0.6, 1.0]);
    assert_eq!(param_from_arc_length(&table, 0.0), 0.0);
    assert_eq!(param_from_arc_length(&table, 1.0), 1.0);
}

#[test]
fn param_from_arc_length_interpolates_linearly() {
    let table = ArcLengthTableRef::new(&[0.0_f64, 0.5, 1.0], &[0.0, 0.6, 1.0]);
    // s = 0.25 lies between (0.0 -> 0.0) and (0.5 -> 0.6); linear interp gives 0.3.
    assert!((param_from_arc_length(&table, 0.25_f64) - 0.3).abs() < 1e-12);
}

#[allow(clippy::float_cmp)]
#[test]
fn param_from_arc_length_clamps_above_range_in_release() {
    let table = ArcLengthTableRef::new(&[0.0_f64, 1.0], &[0.0, 1.0]);
    let v = param_from_arc_length(&table, 1.0_f64);
    assert_eq!(v, 1.0);
}

#[test]
fn arc_length_from_param_inverts_param_from_arc_length() {
    let table = ArcLengthTableRef::new(&[0.0_f64, 0.4, 1.0], &[0.0, 0.5, 1.0]);
    let u = 0.3_f64;
    let s = arc_length_from_param(&table, u);
    let u_back = param_from_arc_length(&table, s);
    assert!((u - u_back).abs() < 1e-12);
}

#[cfg(feature = "host")]
#[test]
fn build_vector_table_for_3d_linear_curve() {
    let curve = crate::VectorNurbs::try_new(
        1,
        vec![0.0_f64, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [3.0, 0.0, 4.0]],
    )
    .unwrap();
    let table = build_arc_length_table_vector(&curve, 1e-5, 64).unwrap();
    assert!((table.s_max() - 5.0).abs() < 1e-4);
}

#[test]
fn try_from_wire_parses_small_table() {
    // Layout: u8 version, u8 reserved, u16 sample_count, u32 reserved2,
    //         T[sample_count] s, T[sample_count] u
    let mut buf = Vec::new();
    buf.extend_from_slice(&[1u8, 0]); // version, reserved
    buf.extend_from_slice(&3u16.to_ne_bytes()); // sample_count
    buf.extend_from_slice(&[0u8; 4]); // reserved2
    for v in [0.0_f32, 0.5, 1.0] {
        buf.extend_from_slice(&v.to_ne_bytes());
    }
    for v in [0.0_f32, 0.6, 1.0] {
        buf.extend_from_slice(&v.to_ne_bytes());
    }

    let aligned = test_align(&buf, 4);
    let r = ArcLengthTableRef::<f32>::try_from_wire(aligned.as_slice()).unwrap();
    assert_eq!(r.s(), &[0.0_f32, 0.5, 1.0]);
    assert_eq!(r.u(), &[0.0_f32, 0.6, 1.0]);
}

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

fn test_align(data: &[u8], _align: usize) -> AlignedBytes {
    let n = data.len().div_ceil(4);
    let mut backing: Vec<u32> = vec![0; n];
    // SAFETY: `backing` owns `n*4` bytes 4-byte aligned; `data.len() <= n*4`.
    // `u32` has no padding so writing arbitrary bytes is well-defined.
    #[allow(unsafe_code)]
    let bytes: &mut [u8] =
        unsafe { core::slice::from_raw_parts_mut(backing.as_mut_ptr().cast::<u8>(), n * 4) };
    bytes[..data.len()].copy_from_slice(data);
    AlignedBytes {
        backing,
        len: data.len(),
    }
}
