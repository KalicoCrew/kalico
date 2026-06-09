use crate::dispatch::KINEMATICS_COREXY;

#[inline]
pub fn forward_corexy(x: f64, y: f64) -> (f64, f64) {
    (x + y, x - y)
}

#[inline]
pub fn inverse_corexy(motor_a: f64, motor_b: f64) -> (f64, f64) {
    (0.5 * (motor_a + motor_b), 0.5 * (motor_a - motor_b))
}

pub fn forward(tag: u8, xyz: [f64; 3]) -> [f64; 4] {
    if tag == KINEMATICS_COREXY {
        let (a, b) = forward_corexy(xyz[0], xyz[1]);
        [a, b, xyz[2], 0.0]
    } else {
        [xyz[0], xyz[1], xyz[2], 0.0]
    }
}

pub fn inverse(tag: u8, motor: [f64; 4]) -> [f64; 3] {
    if tag == KINEMATICS_COREXY {
        let (x, y) = inverse_corexy(motor[0], motor[1]);
        [x, y, motor[2]]
    } else {
        [motor[0], motor[1], motor[2]]
    }
}

#[cfg(test)]
mod tests;
