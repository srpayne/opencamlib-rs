use std::fmt;

use super::Point;

/// A straight line segment from `p1` to `p2`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Line {
    pub p1: Point,
    pub p2: Point,
}

impl Line {
    pub fn new(p1: Point, p2: Point) -> Self {
        Line { p1, p2 }
    }

    /// Length of the line in the XY plane.
    pub fn length2d(&self) -> f64 {
        (self.p2 - self.p1).xy_norm()
    }

    /// Point along the line at parameter `t` in [0, 1].
    pub fn get_point(&self, t: f64) -> Point {
        (self.p2 - self.p1) * t + self.p1
    }

    /// Point on the (infinite) line nearest to `p`.
    pub fn near(&self, p: &Point) -> Point {
        let mut v = self.p2 - self.p1;
        v.normalize();
        let dp = (*p - self.p1).dot(&v);
        self.p1 + v * dp
    }
}

impl fmt::Display for Line {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {})", self.p1, self.p2)
    }
}
