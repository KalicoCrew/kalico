use compat::degree_elev::elevate_g51_to_g5;

/// P0=(0,0,0), P1=(3,3,0), P2=(10,0,0).
///
/// `CP1_cubic` = (1/3)*(0,0) + (2/3)*(3,3) = (2, 2)
/// `CP2_cubic` = (2/3)*(3,3) + (1/3)*(10,0) = (16/3, 2)
///
/// Expected: I=2, J=2, P=16/3-10=-14/3, Q=2, X=10, Y=0, Z=0.
#[test]
fn degree_elevation_basic() {
    let line = elevate_g51_to_g5(
        [0.0, 0.0, 0.0],
        [3.0, 3.0, 0.0],
        [10.0, 0.0, 0.0],
        1.0,
        Some(3000.0),
    );

    assert!((line.x - 10.0).abs() < 1e-12, "X={}", line.x);
    assert!((line.y - 0.0).abs() < 1e-12, "Y={}", line.y);
    assert!((line.z - 0.0).abs() < 1e-12, "Z={}", line.z);
    assert!((line.i - 2.0).abs() < 1e-12, "I={} expected 2", line.i);
    assert!((line.j - 2.0).abs() < 1e-12, "J={} expected 2", line.j);
    assert!(
        (line.p - (-14.0 / 3.0)).abs() < 1e-12,
        "P={} expected -14/3",
        line.p
    );
    assert!((line.q - 2.0).abs() < 1e-12, "Q={} expected 2", line.q);
    assert!((line.e - 1.0).abs() < 1e-12);
    assert_eq!(line.f, Some(3000.0));
}

/// P0=(0,0,0), P1=(5,0,0.5), P2=(10,0,1).
///
/// `CP1_cubic` = (1/3)*(0,0) + (2/3)*(5,0) = (10/3, 0)
/// `CP2_cubic` = (2/3)*(5,0) + (1/3)*(10,0) = (20/3, 0)
///
/// Expected: I=10/3, J=0, P=20/3-10=-10/3, Q=0, Z=1.0.
#[test]
fn degree_elevation_with_z() {
    let line = elevate_g51_to_g5(
        [0.0, 0.0, 0.0],
        [5.0, 0.0, 0.5],
        [10.0, 0.0, 1.0],
        0.5,
        None,
    );

    assert!((line.x - 10.0).abs() < 1e-12, "X={}", line.x);
    assert!((line.y - 0.0).abs() < 1e-12, "Y={}", line.y);
    assert!((line.z - 1.0).abs() < 1e-12, "Z={} expected 1.0", line.z);
    assert!(
        (line.i - 10.0 / 3.0).abs() < 1e-12,
        "I={} expected 10/3",
        line.i
    );
    assert!((line.j - 0.0).abs() < 1e-12, "J={} expected 0", line.j);
    assert!(
        (line.p - (-10.0 / 3.0)).abs() < 1e-12,
        "P={} expected -10/3",
        line.p
    );
    assert!((line.q - 0.0).abs() < 1e-12, "Q={} expected 0", line.q);
    assert!((line.e - 0.5).abs() < 1e-12);
    assert_eq!(line.f, None);
}
