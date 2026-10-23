//! Small numerical helpers shared by the algorithms.

use crate::geo::Point;

/// Tolerance used by [`is_zero_tol`].
pub const TOLERANCE: f64 = 0.000_000_1;

/// Returns -1 if `x < 0`, otherwise +1.
pub fn sign(x: f64) -> f64 {
    if x < 0.0 {
        -1.0
    } else {
        1.0
    }
}

#[inline]
pub fn square(x: f64) -> f64 {
    x * x
}

pub fn is_positive(x: f64) -> bool {
    x > 0.0
}

pub fn is_negative(x: f64) -> bool {
    x < 0.0
}

/// True if `|x| < 1e-7`.
#[inline]
pub fn is_zero_tol(x: f64) -> bool {
    x.abs() < TOLERANCE
}

/// Machine epsilon for f64, computed the same way as the C++ library.
pub fn eps() -> f64 {
    let mut r = 1.0f64;
    while 1.0 < 1.0 + r {
        r /= 2.0;
    }
    2.0 * r
}

/// The smallest `r` (a power of two times 1000) such that `x + r > x`.
pub fn eps_d(x: f64) -> f64 {
    let mut r = 1000.0f64;
    while x < x + r {
        r /= 2.0;
    }
    2.0 * r
}

/// The smallest `r` (a power of two times 1000) such that `x + r > x`, in f32.
pub fn eps_f(x: f32) -> f32 {
    let mut r = 1000.0f32;
    while x < x + r {
        r /= 2.0;
    }
    2.0 * r
}

/// Solve the 2x2 system
/// ```text
/// [ a  b ] [u] = [e]
/// [ c  d ] [v]   [f]
/// ```
/// Returns `None` if the determinant is (close to) zero.
pub fn two_by_two_solver(a: f64, b: f64, c: f64, d: f64, e: f64, f: f64) -> Option<(f64, f64)> {
    let det = a * d - c * b;
    if is_zero_tol(det) {
        return None;
    }
    let u = (1.0 / det) * (d * e - b * f);
    let v = (1.0 / det) * (-c * e + a * f);
    Some((u, v))
}

/// Intersection of the lines p1-p2 and p3-p4 in the XY plane.
///
/// Returns `(v, t)` such that the intersection is `p1 + v*(p2-p1) = p3 + t*(p4-p3)`.
pub fn xy_line_line_intersection(p1: &Point, p2: &Point, p3: &Point, p4: &Point) -> Option<(f64, f64)> {
    two_by_two_solver(
        p2.x - p1.x,
        -(p4.x - p3.x),
        p2.y - p1.y,
        -(p4.y - p3.y),
        p3.x - p1.x,
        p3.y - p1.y,
    )
}

/// Convert an (x, y) direction into a "diangle" in [0, 4), a cheap monotonic substitute
/// for the polar angle.
pub fn xy_vector_to_diangle(x: f64, y: f64) -> f64 {
    if y >= 0.0 {
        if x >= 0.0 {
            y / (x + y)
        } else {
            1.0 - x / (-x + y)
        }
    } else if x < 0.0 {
        2.0 - y / (-x - y)
    } else {
        3.0 + x / (x - y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn epsilons() {
        assert_eq!(eps(), f64::EPSILON);
        assert!(eps_d(1.0) > 0.0);
        assert!(eps_f(1.0) > 0.0);
    }

    #[test]
    fn solver() {
        let (u, v) = two_by_two_solver(1.0, 0.0, 0.0, 2.0, 3.0, 4.0).unwrap();
        assert_eq!((u, v), (3.0, 2.0));
        assert!(two_by_two_solver(1.0, 1.0, 1.0, 1.0, 0.0, 0.0).is_none());
    }

    #[test]
    fn diangle() {
        assert_eq!(xy_vector_to_diangle(1.0, 0.0), 0.0);
        assert_eq!(xy_vector_to_diangle(0.0, 1.0), 1.0);
        assert_eq!(xy_vector_to_diangle(-1.0, 0.0), 2.0);
        assert_eq!(xy_vector_to_diangle(0.0, -1.0), 3.0);
    }
}
