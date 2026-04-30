use compat::arc::{ArcParams, arc_endpoint_tangent, arc_start_tangent, arc_to_g5};

/// Evaluate a cubic Bezier at parameter t in [0, 1].
fn bezier_eval(p0: [f64; 2], p1: [f64; 2], p2: [f64; 2], p3: [f64; 2], t: f64) -> [f64; 2] {
    let u = 1.0 - t;
    let a = u * u * u;
    let b = 3.0 * u * u * t;
    let c = 3.0 * u * t * t;
    let d = t * t * t;
    [
        a * p0[0] + b * p1[0] + c * p2[0] + d * p3[0],
        a * p0[1] + b * p1[1] + c * p2[1] + d * p3[1],
    ]
}

/// Quarter-arc CCW from (1,0,0) to (0,1,0), center (0,0).
#[test]
fn quarter_arc_ccw() {
    let params = ArcParams {
        start: [1.0, 0.0, 0.0],
        end: [0.0, 1.0, 0.0],
        center: [0.0, 0.0],
        clockwise: false,
        tolerance_mm: 0.001,
    };
    let pieces = arc_to_g5(&params);
    assert!(!pieces.is_empty(), "should produce at least 1 piece");

    let last = pieces.last().unwrap();
    assert!(
        (last.x - 0.0).abs() < 1e-10,
        "last piece x should be 0, got {}",
        last.x
    );
    assert!(
        (last.y - 1.0).abs() < 1e-10,
        "last piece y should be 1, got {}",
        last.y
    );
    assert!(
        (last.z - 0.0).abs() < 1e-10,
        "last piece z should be 0, got {}",
        last.z
    );
}

/// Quarter-arc CW from (0,1,0) to (1,0,0), center (0,0).
#[test]
fn quarter_arc_cw() {
    let params = ArcParams {
        start: [0.0, 1.0, 0.0],
        end: [1.0, 0.0, 0.0],
        center: [0.0, 0.0],
        clockwise: true,
        tolerance_mm: 0.001,
    };
    let pieces = arc_to_g5(&params);
    assert!(!pieces.is_empty());

    let last = pieces.last().unwrap();
    assert!(
        (last.x - 1.0).abs() < 1e-10,
        "last piece x should be 1, got {}",
        last.x
    );
    assert!(
        (last.y - 0.0).abs() < 1e-10,
        "last piece y should be 0, got {}",
        last.y
    );
}

/// Full circle CCW from (1,0,0) back to (1,0,0), center (0,0).
#[test]
fn full_circle_ccw() {
    let params = ArcParams {
        start: [1.0, 0.0, 0.0],
        end: [1.0, 0.0, 0.0],
        center: [0.0, 0.0],
        clockwise: false,
        tolerance_mm: 0.001,
    };
    let pieces = arc_to_g5(&params);
    assert!(
        pieces.len() >= 4,
        "full circle should have >=4 pieces, got {}",
        pieces.len()
    );

    let last = pieces.last().unwrap();
    assert!(
        (last.x - 1.0).abs() < 1e-6,
        "full circle should close at x=1, got {}",
        last.x
    );
    assert!(
        (last.y - 0.0).abs() < 1e-6,
        "full circle should close at y=0, got {}",
        last.y
    );
}

/// Full circle CW from (1,0,0) back to (1,0,0), center (0,0).
#[test]
fn full_circle_cw() {
    let params = ArcParams {
        start: [1.0, 0.0, 0.0],
        end: [1.0, 0.0, 0.0],
        center: [0.0, 0.0],
        clockwise: true,
        tolerance_mm: 0.001,
    };
    let pieces = arc_to_g5(&params);
    assert!(
        pieces.len() >= 4,
        "full circle should have >=4 pieces, got {}",
        pieces.len()
    );

    let last = pieces.last().unwrap();
    assert!(
        (last.x - 1.0).abs() < 1e-6,
        "full CW circle should close at x=1, got {}",
        last.x
    );
    assert!(
        (last.y - 0.0).abs() < 1e-6,
        "full CW circle should close at y=0, got {}",
        last.y
    );
}

/// Very small arc (~3 degrees). Should produce exactly 1 piece.
#[test]
fn small_arc() {
    let angle = 3.0_f64.to_radians();
    let params = ArcParams {
        start: [10.0, 0.0, 0.0],
        end: [10.0 * angle.cos(), 10.0 * angle.sin(), 0.0],
        center: [0.0, 0.0],
        clockwise: false,
        tolerance_mm: 0.001,
    };
    let pieces = arc_to_g5(&params);
    assert_eq!(
        pieces.len(),
        1,
        "3-degree arc should be 1 piece, got {}",
        pieces.len()
    );
}

/// Helical quarter-arc with Z from 0 to 1. Last piece z must be 1.
#[test]
fn helical_arc() {
    let params = ArcParams {
        start: [1.0, 0.0, 0.0],
        end: [0.0, 1.0, 1.0],
        center: [0.0, 0.0],
        clockwise: false,
        tolerance_mm: 0.001,
    };
    let pieces = arc_to_g5(&params);

    let last = pieces.last().unwrap();
    assert!(
        (last.z - 1.0).abs() < 1e-10,
        "helical arc last piece z should be 1, got {}",
        last.z
    );

    // Z should be monotonically increasing.
    let mut prev_z = 0.0;
    for piece in &pieces {
        assert!(
            piece.z >= prev_z - 1e-10,
            "Z should be monotonically increasing"
        );
        prev_z = piece.z;
    }
}

/// Critical correctness test: 90-degree arc at radius 10mm, tolerance 5um.
/// Sample each Bezier piece at 100 points and verify distance to circle center
/// is approximately radius, within tolerance.
#[test]
fn radial_error_verification() {
    let r = 10.0;
    let tol = 0.005; // 5 um
    let params = ArcParams {
        start: [r, 0.0, 0.0],
        end: [0.0, r, 0.0],
        center: [0.0, 0.0],
        clockwise: false,
        tolerance_mm: tol,
    };
    let pieces = arc_to_g5(&params);

    let mut prev_end = [params.start[0], params.start[1]];
    let mut max_err = 0.0_f64;

    for piece in &pieces {
        // Reconstruct absolute control points.
        let p0 = prev_end;
        let p1 = [p0[0] + piece.i, p0[1] + piece.j];
        let p2 = [piece.x + piece.p, piece.y + piece.q];
        let p3 = [piece.x, piece.y];

        // Sample at 101 points (t = 0.0, 0.01, ..., 1.0).
        for k in 0..=100 {
            let t = f64::from(k) / 100.0;
            let pt = bezier_eval(p0, p1, p2, p3, t);
            let dist = pt[0].hypot(pt[1]);
            let err = (dist - r).abs();
            max_err = max_err.max(err);
        }

        prev_end = p3;
    }

    assert!(
        max_err <= tol,
        "max radial error {max_err:.6e} exceeds tolerance {tol:.6e}"
    );
}

/// 180-degree arc. Verify endpoint.
#[test]
fn half_arc() {
    let params = ArcParams {
        start: [1.0, 0.0, 0.0],
        end: [-1.0, 0.0, 0.0],
        center: [0.0, 0.0],
        clockwise: false,
        tolerance_mm: 0.001,
    };
    let pieces = arc_to_g5(&params);

    let last = pieces.last().unwrap();
    assert!(
        (last.x - (-1.0)).abs() < 1e-10,
        "half arc endpoint x should be -1, got {}",
        last.x
    );
    assert!(
        (last.y - 0.0).abs() < 1e-10,
        "half arc endpoint y should be 0, got {}",
        last.y
    );
}

/// Start tangent of CCW arc from (1,0) center (0,0) should be (0,1).
#[test]
fn start_tangent_ccw() {
    let params = ArcParams {
        start: [1.0, 0.0, 0.0],
        end: [0.0, 1.0, 0.0],
        center: [0.0, 0.0],
        clockwise: false,
        tolerance_mm: 0.001,
    };
    let t = arc_start_tangent(&params);
    assert!(
        (t[0] - 0.0).abs() < 1e-10,
        "start tangent x should be 0, got {}",
        t[0]
    );
    assert!(
        (t[1] - 1.0).abs() < 1e-10,
        "start tangent y should be 1, got {}",
        t[1]
    );
}

/// Endpoint tangent of CCW arc from (1,0) to (0,1) center (0,0) should be (-1,0).
#[test]
fn endpoint_tangent_ccw() {
    let params = ArcParams {
        start: [1.0, 0.0, 0.0],
        end: [0.0, 1.0, 0.0],
        center: [0.0, 0.0],
        clockwise: false,
        tolerance_mm: 0.001,
    };
    let t = arc_endpoint_tangent(&params);
    assert!(
        (t[0] - (-1.0)).abs() < 1e-10,
        "endpoint tangent x should be -1, got {}",
        t[0]
    );
    assert!(
        (t[1] - 0.0).abs() < 1e-10,
        "endpoint tangent y should be 0, got {}",
        t[1]
    );
}
