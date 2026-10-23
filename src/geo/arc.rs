use std::f64::consts::PI;
use std::fmt;

use super::Point;

/// A circular arc in the XY plane from `p1` to `p2` around center `c`.
/// `dir == true` means counter-clockwise.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Arc {
    pub p1: Point,
    pub p2: Point,
    pub c: Point,
    pub dir: bool,
    length: f64,
    radius: f64,
}

impl Arc {
    pub fn new(p1: Point, p2: Point, c: Point, dir: bool) -> Self {
        let mut vs = (p1 - c).xy_perp();
        let mut ve = (p2 - c).xy_perp();
        let radius = vs.xy_norm();
        vs.normalize();
        ve.normalize();
        let length = xy_included_angle(&vs, &ve, dir).abs() * radius;
        Arc {
            p1,
            p2,
            c,
            dir,
            length,
            radius,
        }
    }

    /// Length of the arc in the XY plane.
    pub fn length2d(&self) -> f64 {
        self.length
    }

    /// Radius of the arc.
    pub fn radius(&self) -> f64 {
        self.radius
    }

    /// Point along the arc at parameter `t` in [0, 1].
    pub fn get_point(&self, t: f64) -> Point {
        if t.abs() < 0.000_000_000_000_01 {
            return self.p1;
        }
        if (t - 1.0).abs() < 0.000_000_000_000_01 {
            return self.p2;
        }
        let mut d = t * self.length;
        if !self.dir {
            d = -d;
        }
        let mut v = self.p1 - self.c;
        v.xy_rotate(d / self.radius);
        v + self.c
    }

    /// Included angle between unit vectors `v1` and `v2` in the XY plane.
    pub fn xy_included_angle(&self, v1: &Point, v2: &Point, dir: bool) -> f64 {
        xy_included_angle(v1, v2, dir)
    }
}

fn xy_included_angle(v1: &Point, v2: &Point, dir: bool) -> f64 {
    let d = if dir { 1.0 } else { -1.0 };
    let mut inc_ang = v1.dot(v2);
    if inc_ang > 1.0 - 1.0e-10 {
        return 0.0;
    }
    if inc_ang < -1.0 + 1.0e-10 {
        inc_ang = PI;
    } else {
        if inc_ang > 1.0 {
            inc_ang = 1.0;
        }
        inc_ang = inc_ang.acos();
        let x = v1.x * v2.y - v1.y * v2.x;
        if d * x < 0.0 {
            inc_ang = 2.0 * PI - inc_ang;
        }
    }
    d * inc_ang
}

impl fmt::Display for Arc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "({}, {}, {}, {})",
            self.p1, self.p2, self.c, self.dir as u8
        )
    }
}
