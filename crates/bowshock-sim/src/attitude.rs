//! Attitude of the rigid-body flight model: unit quaternions `[w, x, y, z]` that rotate body-frame vectors into the inertial
//! frame (x east, y north, z up). The body frame has x along the nose direction; y and z are transverse.
//!
//! Kinematics `dq/dt = 1/2 q (0, w)` with the body angular rate `w` (Zipfel 2014, *Modeling and Simulation of Aerospace Vehicle
//! Dynamics*, the quaternion attitude equation; Diebel 2006, *Representing Attitude: Euler Angles, Unit Quaternions, and
//! Rotation Vectors*).

/// A 3-vector.
pub(crate) type Vec3 = [f64; 3];
/// A quaternion `[w, x, y, z]`.
pub(crate) type Quat = [f64; 4];

pub(crate) fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// Hamilton product `a b`.
fn mul(a: Quat, b: Quat) -> Quat {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

/// `q` scaled to unit length (the identity for a zero quaternion).
pub(crate) fn normalize(q: Quat) -> Quat {
    let n = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if n < 1e-300 {
        [1.0, 0.0, 0.0, 0.0]
    } else {
        [q[0] / n, q[1] / n, q[2] / n, q[3] / n]
    }
}

/// Body-frame vector `v` expressed in the inertial frame.
pub(crate) fn rotate(q: Quat, v: Vec3) -> Vec3 {
    let u = [q[1], q[2], q[3]];
    let t = cross(u, v);
    let t = [2.0 * t[0], 2.0 * t[1], 2.0 * t[2]];
    let c = cross(u, t);
    [v[0] + q[0] * t[0] + c[0], v[1] + q[0] * t[1] + c[1], v[2] + q[0] * t[2] + c[2]]
}

/// Inertial-frame vector `v` expressed in the body frame.
pub(crate) fn rotate_inv(q: Quat, v: Vec3) -> Vec3 {
    rotate([q[0], -q[1], -q[2], -q[3]], v)
}

/// Time derivative of `q` for the body angular rate `w` (rad/s): `1/2 q (0, w)`.
pub(crate) fn rate(q: Quat, w: Vec3) -> Quat {
    let d = mul(q, [0.0, w[0], w[1], w[2]]);
    [0.5 * d[0], 0.5 * d[1], 0.5 * d[2], 0.5 * d[3]]
}

/// Attitude whose x axis lies along `axis` (a unit vector). The y axis is as close to north as the axis allows (east when the
/// axis itself is within a hair of north), which fixes the roll reference.
pub(crate) fn from_axis(axis: Vec3) -> Quat {
    let along = |r: Vec3| {
        let k = dot(r, axis);
        let y = [r[0] - k * axis[0], r[1] - k * axis[1], r[2] - k * axis[2]];
        (y, dot(y, y).sqrt())
    };
    let (mut y, mut n) = along([0.0, 1.0, 0.0]);
    if n < 1e-6 {
        (y, n) = along([1.0, 0.0, 0.0]);
    }
    let y = [y[0] / n, y[1] / n, y[2] / n];
    let z = cross(axis, y);
    // Rotation matrix with columns (axis, y, z) to a quaternion (Shepperd's method).
    let (m00, m01, m02) = (axis[0], y[0], z[0]);
    let (m10, m11, m12) = (axis[1], y[1], z[1]);
    let (m20, m21, m22) = (axis[2], y[2], z[2]);
    let tr = m00 + m11 + m22;
    let q = if tr > 0.0 {
        let s = 2.0 * (1.0 + tr).sqrt();
        [0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s]
    } else if m00 > m11 && m00 > m22 {
        let s = 2.0 * (1.0 + m00 - m11 - m22).sqrt();
        [(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s]
    } else if m11 > m22 {
        let s = 2.0 * (1.0 + m11 - m00 - m22).sqrt();
        [(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s]
    } else {
        let s = 2.0 * (1.0 + m22 - m00 - m11).sqrt();
        [(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s]
    };
    normalize(q)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Vec3, b: Vec3) -> bool {
        (0..3).all(|i| (a[i] - b[i]).abs() < 1e-12)
    }

    #[test]
    fn from_axis_puts_the_nose_on_the_axis_for_any_direction() {
        for axis in [[0.0, 0.0, 1.0], [0.3, -0.4, 0.5], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [-0.6, 0.0, 0.8]] {
            let n = dot(axis, axis).sqrt();
            let axis = [axis[0] / n, axis[1] / n, axis[2] / n];
            let q = from_axis(axis);
            assert!(close(rotate(q, [1.0, 0.0, 0.0]), axis), "{axis:?}");
            // A rotation: y and z stay orthonormal and the inverse undoes it.
            let (y, z) = (rotate(q, [0.0, 1.0, 0.0]), rotate(q, [0.0, 0.0, 1.0]));
            assert!(dot(y, z).abs() < 1e-12 && (dot(y, y) - 1.0).abs() < 1e-12);
            assert!(close(rotate_inv(q, axis), [1.0, 0.0, 0.0]));
        }
    }

    #[test]
    fn a_constant_rate_turns_the_axis_by_the_integrated_angle() {
        // 90 deg/s about the body z axis for 1 s: the nose x_b swings onto the initial y_b (explicit Euler steps with
        // renormalisation, so first-order accurate).
        let mut q = from_axis([0.0, 0.0, 1.0]);
        let (w, n) = ([0.0, 0.0, std::f64::consts::FRAC_PI_2], 100_000);
        let h = 1.0 / f64::from(n);
        let y0 = rotate(q, [0.0, 1.0, 0.0]);
        for _ in 0..n {
            let d = rate(q, w);
            q = normalize([q[0] + h * d[0], q[1] + h * d[1], q[2] + h * d[2], q[3] + h * d[3]]);
        }
        let x = rotate(q, [1.0, 0.0, 0.0]);
        assert!((0..3).all(|i| (x[i] - y0[i]).abs() < 1e-3), "{x:?} vs {y0:?}");
    }
}
