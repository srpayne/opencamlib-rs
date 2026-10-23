use std::fmt;
use std::ops::{Add, AddAssign, Mul, MulAssign, Neg, Sub, SubAssign};

use super::Triangle;
use crate::common::numeric::{is_zero_tol, square};

/// A point or vector in 3D space specified by its coordinates (x, y, z).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point {
    /// Create a point at (x, y, z).
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Point { x, y, z }
    }

    /// Create a point at (x, y, 0).
    pub const fn new_xy(x: f64, y: f64) -> Self {
        Point { x, y, z: 0.0 }
    }

    /// Dot product.
    pub fn dot(&self, p: &Point) -> f64 {
        self.x * p.x + self.y * p.y + self.z * p.z
    }

    /// Cross product.
    pub fn cross(&self, p: &Point) -> Point {
        Point::new(
            self.y * p.z - self.z * p.y,
            self.z * p.x - self.x * p.z,
            self.x * p.y - self.y * p.x,
        )
    }

    /// Norm of the vector, i.e. the distance from (0, 0, 0) to this point.
    pub fn norm(&self) -> f64 {
        (square(self.x) + square(self.y) + square(self.z)).sqrt()
    }

    /// Scale the vector so that `norm() == 1.0`. A zero vector is left unchanged.
    pub fn normalize(&mut self) {
        let norm = self.norm();
        if norm != 0.0 {
            *self *= 1.0 / norm;
        }
    }

    /// Distance from this point to `p` in the XY plane.
    pub fn xy_distance(&self, p: &Point) -> f64 {
        (*self - *p).xy_norm()
    }

    /// Length of the vector in the XY plane.
    pub fn xy_norm(&self) -> f64 {
        (square(self.x) + square(self.y)).sqrt()
    }

    /// Normalize so that `xy_norm() == 1.0` (z is scaled by the same factor).
    pub fn xy_normalize(&mut self) {
        let n = self.xy_norm();
        if n != 0.0 {
            *self *= 1.0 / n;
        }
    }

    /// Perpendicular in the XY plane, rotated 90 degrees to the left.
    pub fn xy_perp(&self) -> Point {
        Point::new(-self.y, self.x, self.z)
    }

    /// Move this point along the z-axis so that it lies on the p1-p2 line.
    pub fn z_project_onto_edge(&mut self, p1: &Point, p2: &Point) {
        let t = if (p2.x - p1.x).abs() > (p2.y - p1.y).abs() {
            (self.x - p1.x) / (p2.x - p1.x)
        } else {
            (self.y - p1.y) / (p2.y - p1.y)
        };
        self.z = p1.z + t * (p2.z - p1.z);
    }

    /// Rotate in the XY plane, given cos(theta) and sin(theta).
    pub fn xy_rotate_cs(&mut self, cosa: f64, sina: f64) {
        let temp = -self.y * sina + self.x * cosa;
        self.y = self.x * sina + cosa * self.y;
        self.x = temp;
    }

    /// Rotate in the XY plane by `angle` radians.
    pub fn xy_rotate(&mut self, angle: f64) {
        self.xy_rotate_cs(angle.cos(), angle.sin());
    }

    /// Rotate around the x-axis by `theta` radians.
    pub fn x_rotate(&mut self, theta: f64) {
        let (s, c) = theta.sin_cos();
        self.matrix_rotate([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]);
    }

    /// Rotate around the y-axis by `theta` radians.
    pub fn y_rotate(&mut self, theta: f64) {
        let (s, c) = theta.sin_cos();
        self.matrix_rotate([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]);
    }

    /// Rotate around the z-axis by `theta` radians.
    pub fn z_rotate(&mut self, theta: f64) {
        let (s, c) = theta.sin_cos();
        self.matrix_rotate([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]);
    }

    /// Multiply this point by a row-major 3x3 matrix.
    pub fn matrix_rotate(&mut self, m: [[f64; 3]; 3]) {
        let (x, y, z) = (self.x, self.y, self.z);
        self.x = m[0][0] * x + m[0][1] * y + m[0][2] * z;
        self.y = m[1][0] * x + m[1][1] * y + m[1][2] * z;
        self.z = m[2][0] * x + m[2][1] * y + m[2][2] * z;
    }

    /// Distance in the XY plane from this point to the infinite line through p1 and p2.
    ///
    /// Returns -1 if p1 and p2 coincide in the XY plane.
    pub fn xy_distance_to_line(&self, p1: &Point, p2: &Point) -> f64 {
        if p1.x == p2.x && p1.y == p2.y {
            return -1.0;
        }
        let mut v = Point::new(p2.y - p1.y, -(p2.x - p1.x), 0.0);
        v.normalize();
        let r = Point::new(p1.x - self.x, p1.y - self.y, 0.0);
        v.dot(&r).abs()
    }

    /// Point on the line through p1 and p2 which is closest (in 3D) to this point.
    pub fn closest_point(&self, p1: &Point, p2: &Point) -> Point {
        let v = *p2 - *p1;
        let u = (*self - *p1).dot(&v) / v.dot(&v);
        *p1 + u * v
    }

    /// Point on the line through p1 and p2 which is closest to this point in the XY plane.
    /// The returned point has z = 0.
    ///
    /// Returns the origin if p1 and p2 do not form a line in the XY plane.
    pub fn xy_closest_point(&self, p1: &Point, p2: &Point) -> Point {
        let v = *p2 - *p1;
        if is_zero_tol(v.xy_norm()) {
            return Point::default();
        }
        let u = ((self.x - p1.x) * v.x + (self.y - p1.y) * v.y) / (v.x * v.x + v.y * v.y);
        Point::new(p1.x + u * v.x, p1.y + u * v.y, 0.0)
    }

    /// True if this point is to the right of the line through p1 and p2 (in the XY plane).
    pub fn is_right(&self, p1: &Point, p2: &Point) -> bool {
        let a1 = p2.x - p1.x;
        let a2 = p2.y - p1.y;
        let t = a2 * (self.x - p1.x) - a1 * (self.y - p1.y);
        t > 0.000_000_000_000_01
    }

    /// True if this point is strictly inside triangle `t` (in the XY plane).
    pub fn is_inside_triangle(&self, t: &Triangle) -> bool {
        let p = self;
        let (a, b, c) = (&t.p[0], &t.p[1], &t.p[2]);
        // barycentric coordinates
        let u = (a.y * c.x - a.x * c.y + (c.y - a.y) * p.x + (a.x - c.x) * p.y)
            / (a.y * c.x - a.x * c.y + (c.y - a.y) * b.x + (a.x - c.x) * b.y);
        let v = (a.x * b.y - a.y * b.x + (a.y - b.y) * p.x + (b.x - a.x) * p.y)
            / (a.x * b.y - a.y * b.x + (a.y - b.y) * c.x + (b.x - a.x) * c.y);
        u > 0.0 && v > 0.0 && (u + v) < 1.0
    }

    /// True if the projection of this point onto the line p1-p2 lies within the segment.
    pub fn is_inside_segment(&self, p1: &Point, p2: &Point) -> bool {
        let d = *p2 - *p1;
        let t = (*self - *p1).dot(&d) / d.dot(&d);
        !(t > 1.0 || t < 0.0)
    }

    /// True if the vector is parallel to the x-axis.
    pub fn x_parallel(&self) -> bool {
        is_zero_tol(self.y) && is_zero_tol(self.z)
    }

    /// True if the vector is parallel to the y-axis.
    pub fn y_parallel(&self) -> bool {
        is_zero_tol(self.x) && is_zero_tol(self.z)
    }

    /// True if the x and y components are both exactly zero.
    pub fn z_parallel(&self) -> bool {
        self.x == 0.0 && self.y == 0.0
    }
}

impl fmt::Display for Point {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {}, {})", self.x, self.y, self.z)
    }
}

impl Add for Point {
    type Output = Point;
    fn add(self, p: Point) -> Point {
        Point::new(self.x + p.x, self.y + p.y, self.z + p.z)
    }
}

impl Sub for Point {
    type Output = Point;
    fn sub(self, p: Point) -> Point {
        Point::new(self.x - p.x, self.y - p.y, self.z - p.z)
    }
}

impl AddAssign for Point {
    fn add_assign(&mut self, p: Point) {
        *self = *self + p;
    }
}

impl SubAssign for Point {
    fn sub_assign(&mut self, p: Point) {
        *self = *self - p;
    }
}

impl Mul<f64> for Point {
    type Output = Point;
    fn mul(self, a: f64) -> Point {
        Point::new(self.x * a, self.y * a, self.z * a)
    }
}

impl Mul<Point> for f64 {
    type Output = Point;
    fn mul(self, p: Point) -> Point {
        p * self
    }
}

impl MulAssign<f64> for Point {
    fn mul_assign(&mut self, a: f64) {
        *self = *self * a;
    }
}

impl Neg for Point {
    type Output = Point;
    fn neg(self) -> Point {
        -1.0 * self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn basic_ops() {
        let a = Point::new(1.0, 2.0, 3.0);
        let b = Point::new(4.0, 5.0, 6.0);
        assert_eq!(a + b, Point::new(5.0, 7.0, 9.0));
        assert_eq!(b - a, Point::new(3.0, 3.0, 3.0));
        assert_eq!(2.0 * a, Point::new(2.0, 4.0, 6.0));
        assert_eq!(a.dot(&b), 32.0);
        assert_eq!(a.cross(&b), Point::new(-3.0, 6.0, -3.0));
        let mut c = Point::new(3.0, 4.0, 0.0);
        assert_eq!(c.norm(), 5.0);
        c.normalize();
        assert!((c.norm() - 1.0).abs() < 1e-15);
    }

    #[test]
    fn line_distance_and_closest() {
        let p = Point::new(1.0, 1.0, 0.0);
        let p1 = Point::new(0.0, 0.0, 0.0);
        let p2 = Point::new(2.0, 0.0, 0.0);
        assert!((p.xy_distance_to_line(&p1, &p2) - 1.0).abs() < 1e-15);
        assert_eq!(p.xy_closest_point(&p1, &p2), Point::new(1.0, 0.0, 0.0));
        assert!(p.is_inside_segment(&p1, &p2));
        assert!(!Point::new(3.0, 0.0, 0.0).is_inside_segment(&p1, &p2));
        assert!(Point::new(1.0, -1.0, 0.0).is_right(&p1, &p2));
    }

    #[test]
    fn rotations() {
        let mut p = Point::new(1.0, 0.0, 0.0);
        p.z_rotate(std::f64::consts::FRAC_PI_2);
        assert!((p.x).abs() < 1e-15 && (p.y - 1.0).abs() < 1e-15);
        let mut q = Point::new(1.0, 0.0, 0.0);
        q.xy_rotate(std::f64::consts::FRAC_PI_2);
        assert!((q - p).norm() < 1e-15);
    }
}
