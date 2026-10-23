//! Ellipses and offset-ellipses, used by the [`BullCutter`](super::BullCutter)
//! drop-cutter and push-cutter edge tests.
//!
//! A position on the ellipse is given by `(s, t)` with `s^2 + t^2 = 1`, parametrized by a
//! "diangle" in [0, 4) (see [`xy_vector_to_diangle`]).

use std::fmt;

use crate::algo::fiber::Fiber;
use crate::common::brent_zero::brent_zero;
use crate::common::numeric::{is_zero_tol, square, xy_vector_to_diangle};
use crate::geo::Point;

const OE_ERROR_TOLERANCE: f64 = 1e-10;

/// A position `(s, t)` on an ellipse, `s^2 + t^2 = 1`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EllipsePosition {
    pub s: f64,
    pub t: f64,
    pub diangle: f64,
}

impl Default for EllipsePosition {
    fn default() -> Self {
        let mut p = EllipsePosition {
            s: 0.0,
            t: 0.0,
            diangle: 0.0,
        };
        p.set_d();
        p
    }
}

impl EllipsePosition {
    pub fn new(s: f64, t: f64) -> Self {
        EllipsePosition { s, t, diangle: 0.0 }
    }

    /// Set the position from a diangle.
    pub fn set_diangle(&mut self, dia: f64) {
        self.diangle = dia;
        self.set_d();
    }

    fn set_d(&mut self) {
        let mut d = self.diangle;
        while d > 4.0 {
            d -= 4.0;
        }
        while d < 0.0 {
            d += 4.0;
        }
        let mut p = Point::new_xy(
            if d < 2.0 { 1.0 - d } else { d - 3.0 },
            if d < 3.0 {
                if d > 1.0 {
                    2.0 - d
                } else {
                    d
                }
            } else {
                d - 4.0
            },
        );
        p.normalize();
        self.s = p.x;
        self.t = p.y;
    }

    /// True if `s^2 + t^2 == 1` within tolerance.
    pub fn is_valid(&self) -> bool {
        is_zero_tol(square(self.s) + square(self.t) - 1.0)
    }
}

impl fmt::Display for EllipsePosition {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({} ,{})", self.s, self.t)
    }
}

/// An axis-aligned ellipse in the XY plane (`a` along X, `b` along Y) with an offset
/// ellipse at distance `offset`. Used for the bull-cutter edge drop in canonical
/// coordinates: the solver finds positions where the offset-ellipse point has y = 0.
#[derive(Debug, Clone, PartialEq)]
pub struct Ellipse {
    pub center: Point,
    pub a: f64,
    pub b: f64,
    pub offset: f64,
    pub eccen: f64,
    pub(crate) pos1: EllipsePosition,
    pub(crate) pos2: EllipsePosition,
    pos_hi: EllipsePosition,
}

impl Ellipse {
    pub fn new(center: Point, a: f64, b: f64, offset: f64) -> Self {
        Ellipse {
            center,
            a,
            b,
            offset,
            eccen: a / b,
            pos1: EllipsePosition::default(),
            pos2: EllipsePosition::default(),
            pos_hi: EllipsePosition::default(),
        }
    }

    /// Point on the ellipse.
    pub fn e_point(&self, pos: &EllipsePosition) -> Point {
        let mut p = self.center;
        p.x += self.a * pos.s;
        p.y += self.b * pos.t;
        p
    }

    /// Point on the offset-ellipse.
    pub fn oe_point(&self, pos: &EllipsePosition) -> Point {
        self.e_point(pos) + self.offset * self.normal(pos)
    }

    /// Unit normal of the ellipse.
    pub fn normal(&self, pos: &EllipsePosition) -> Point {
        let mut n = Point::new(self.b * pos.s, self.a * pos.t, 0.0);
        n.normalize();
        n
    }

    /// Solver error at `pos`: the y-coordinate of the offset-ellipse point.
    pub fn error_at(&self, pos: &EllipsePosition) -> f64 {
        self.oe_point(pos).y
    }

    /// Solver error at diangle `dia`.
    pub fn error(&self, dia: f64) -> f64 {
        let mut tmp = EllipsePosition::default();
        tmp.set_diangle(dia);
        self.error_at(&tmp)
    }

    /// Find the two solutions where the offset-ellipse touches y = 0.
    /// Returns the number of iterations (always 1, as in the C++ library).
    pub fn solver_brent(&mut self) -> i32 {
        let mut apos = EllipsePosition::default();
        let mut bpos = EllipsePosition::default();
        apos.set_diangle(0.0);
        bpos.set_diangle(3.0);
        if self.error_at(&apos).abs() < OE_ERROR_TOLERANCE {
            self.pos1 = apos;
        } else if self.error_at(&bpos).abs() < OE_ERROR_TOLERANCE {
            self.pos1 = bpos;
        } else {
            let dia_sln = brent_zero(apos.diangle, bpos.diangle, 3e-16, OE_ERROR_TOLERANCE, |d| {
                self.error(d)
            });
            self.pos1.set_diangle(dia_sln);
        }
        self.find_ellipse_position2();
        1
    }

    /// The second solution is a mirror image of the first; find which one.
    fn find_ellipse_position2(&mut self) -> bool {
        let err1 = self.error_at(&self.pos1).abs();
        let (s1, t1) = (self.pos1.s, self.pos1.t);
        for (s, t) in [(s1, -t1), (-s1, t1), (-s1, -t1)] {
            self.pos2.s = s;
            self.pos2.t = t;
            if self.error_at(&self.pos2).abs() < err1 + OE_ERROR_TOLERANCE
                && ((self.pos2.s - s1).abs() > 1e-8 || (self.pos2.t - t1).abs() > 1e-8)
            {
                return true;
            }
        }
        self.pos2.s = s1;
        self.pos2.t = t1;
        self.error_at(&self.pos2).abs() < err1 + OE_ERROR_TOLERANCE
    }

    /// Ellipse center for solution `sln` (1 or 2), positioned on the line up1-up2.
    pub fn calc_ecenter(&self, up1: &Point, up2: &Point, sln: i32) -> Point {
        let cle = if sln == 1 {
            self.oe_point1()
        } else {
            self.oe_point2()
        };
        let xoffset = -cle.x;
        let t = (self.center.x + xoffset - up1.x) / (up2.x - up1.x);
        *up1 + t * (*up2 - *up1)
    }

    /// Choose the solution whose ellipse center is higher, and move the center there.
    pub fn set_ellipse_position_hi(&mut self, u1: &Point, u2: &Point) {
        let ecen1 = self.calc_ecenter(u1, u2, 1);
        let ecen2 = self.calc_ecenter(u1, u2, 2);
        if ecen1.z >= ecen2.z {
            self.pos_hi = self.pos1;
            self.center = ecen1;
        } else {
            self.pos_hi = self.pos2;
            self.center = ecen2;
        }
    }

    pub fn e_point_hi(&self) -> Point {
        self.e_point(&self.pos_hi)
    }
    pub fn e_point1(&self) -> Point {
        self.e_point(&self.pos1)
    }
    pub fn e_point2(&self) -> Point {
        self.e_point(&self.pos2)
    }
    pub fn oe_point1(&self) -> Point {
        self.oe_point(&self.pos1)
    }
    pub fn oe_point2(&self) -> Point {
        self.oe_point(&self.pos2)
    }
    pub fn center_z(&self) -> f64 {
        self.center.z
    }
}

impl fmt::Display for Ellipse {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Ellipse: cen={} a={} b={} ofs={}",
            self.center, self.a, self.b, self.offset
        )
    }
}

/// An ellipse in 3D with arbitrary major/minor directions, used by the bull-cutter
/// push-cutter edge test.
#[derive(Debug, Clone, PartialEq)]
pub struct AlignedEllipse {
    pub center: Point,
    pub a: f64,
    pub b: f64,
    pub offset: f64,
    major_dir: Point,
    minor_dir: Point,
    error_dir: Point,
    target: Point,
    pos1: EllipsePosition,
    pos2: EllipsePosition,
}

impl AlignedEllipse {
    pub fn new(
        center: Point,
        major_length: f64,
        minor_length: f64,
        offset: f64,
        major_dir: Point,
        minor_dir: Point,
    ) -> Self {
        AlignedEllipse {
            center,
            a: major_length,
            b: minor_length,
            offset,
            major_dir,
            minor_dir,
            error_dir: Point::default(),
            target: Point::default(),
            pos1: EllipsePosition::default(),
            pos2: EllipsePosition::default(),
        }
    }

    pub fn e_point(&self, pos: &EllipsePosition) -> Point {
        self.center + self.a * pos.s * self.major_dir + self.b * pos.t * self.minor_dir
    }

    pub fn oe_point(&self, pos: &EllipsePosition) -> Point {
        self.e_point(pos) + self.offset * self.normal(pos)
    }

    pub fn normal(&self, pos: &EllipsePosition) -> Point {
        let mut n = pos.s * self.b * self.major_dir + pos.t * self.a * self.minor_dir;
        n.normalize();
        n
    }

    /// Signed distance of the offset-ellipse point from the fiber, at diangle `dia`.
    pub fn error(&self, dia: f64) -> f64 {
        let mut tmp = EllipsePosition::default();
        tmp.set_diangle(dia);
        let p = self.oe_point(&tmp);
        (self.target - p).dot(&self.error_dir)
    }

    /// Find the two positions where the offset-ellipse touches fiber `f`.
    pub fn aligned_solver(&mut self, f: &Fiber) -> bool {
        self.error_dir = f.dir.xy_perp();
        self.target = f.p1;
        // extreme points of the offset ellipse in the direction perpendicular to the fiber
        let t1 = if f.p1.y == f.p2.y {
            (square(self.b * self.minor_dir.y)
                / (square(self.a * self.major_dir.y) + square(self.b * self.minor_dir.y)))
            .sqrt()
        } else if f.p1.x == f.p2.x {
            (square(self.b * self.minor_dir.x)
                / (square(self.a * self.major_dir.x) + square(self.b * self.minor_dir.x)))
            .sqrt()
        } else {
            return false;
        };
        let s1 = (1.0 - square(t1)).sqrt();
        let mut apos = EllipsePosition::default();
        let mut bpos = EllipsePosition::default();
        let mut found_positive = false;
        let mut found_negative = false;
        for (s, t) in [(s1, t1), (s1, -t1), (-s1, t1), (-s1, -t1)] {
            let mut tmp = EllipsePosition::default();
            tmp.set_diangle(xy_vector_to_diangle(s, t));
            let err = self.error(tmp.diangle);
            if err > 0.0 {
                found_positive = true;
                apos = tmp;
            } else if err < 0.0 {
                found_negative = true;
                bpos = tmp;
            }
        }
        if !(found_positive && found_negative) {
            return false;
        }
        // the root is bracketed
        let (lolim, hilim) = if apos.diangle > bpos.diangle {
            (bpos.diangle, apos.diangle)
        } else if bpos.diangle > apos.diangle {
            (apos.diangle, bpos.diangle)
        } else {
            (0.0, 0.0)
        };
        let dia_sln = brent_zero(lolim, hilim, 3e-16, OE_ERROR_TOLERANCE, |d| self.error(d));
        let dia_sln2 = brent_zero(hilim - 4.0, lolim, 3e-16, OE_ERROR_TOLERANCE, |d| self.error(d));
        self.pos1.set_diangle(dia_sln);
        self.pos2.set_diangle(dia_sln2);
        true
    }

    pub fn e_point1(&self) -> Point {
        self.e_point(&self.pos1)
    }
    pub fn e_point2(&self) -> Point {
        self.e_point(&self.pos2)
    }
    pub fn oe_point1(&self) -> Point {
        self.oe_point(&self.pos1)
    }
    pub fn oe_point2(&self) -> Point {
        self.oe_point(&self.pos2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diangle_positions() {
        let mut p = EllipsePosition::default();
        assert_eq!((p.s, p.t), (1.0, 0.0));
        p.set_diangle(1.0);
        assert!((p.s).abs() < 1e-15 && (p.t - 1.0).abs() < 1e-15);
        p.set_diangle(0.5);
        assert!(p.is_valid());
    }

    #[test]
    fn solver_finds_offset_ellipse_touching_x_axis() {
        let mut e = Ellipse::new(Point::new(0.0, 1.2, 0.0), 3.0, 1.0, 0.5);
        e.solver_brent();
        assert!(e.oe_point1().y.abs() < 1e-9);
        assert!(e.oe_point2().y.abs() < 1e-9);
        assert!((e.oe_point1() - e.oe_point2()).norm() > 1e-6);
    }
}
