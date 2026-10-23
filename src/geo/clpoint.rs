use std::fmt;
use std::ops::{Deref, DerefMut};

use super::{CCPoint, Point, Triangle};

/// Cutter-Location (CL) point, with the corresponding cutter-contact point.
///
/// Dereferences to its [`Point`], so `cl.x`, `cl.z = ...` etc. work directly.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CLPoint {
    pub pos: Point,
    pub cc: CCPoint,
}

impl CLPoint {
    /// CL-point at (x, y, z) with an undefined cc-point.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        CLPoint {
            pos: Point::new(x, y, z),
            cc: CCPoint::from_xyz(0.0, 0.0, 0.0, super::CCType::None),
        }
    }

    /// CL-point at (x, y, z) with cc-point `cc`.
    pub const fn with_cc(x: f64, y: f64, z: f64, cc: CCPoint) -> Self {
        CLPoint {
            pos: Point::new(x, y, z),
            cc,
        }
    }

    /// The cutter-contact point.
    pub fn get_cc(&self) -> CCPoint {
        self.cc
    }

    /// If `zin > z`, lift the CL-point to `zin` and return true.
    pub fn lift_z(&mut self, zin: f64) -> bool {
        if zin > self.z {
            self.z = zin;
            true
        } else {
            false
        }
    }

    /// If `zin > z`, lift the CL-point to `zin`, set the cc-point to `ccp`, and return true.
    pub fn lift_z_cc(&mut self, zin: f64, ccp: &CCPoint) -> bool {
        if zin > self.z {
            self.z = zin;
            self.cc = *ccp;
            true
        } else {
            false
        }
    }

    /// If `cc_tmp` lies within the edge p1-p2, lift to `zin` (see [`CLPoint::lift_z_cc`]).
    pub fn lift_z_if_inside_points(
        &mut self,
        zin: f64,
        cc_tmp: &CCPoint,
        p1: &Point,
        p2: &Point,
    ) -> bool {
        if cc_tmp.is_inside_segment(p1, p2) {
            self.lift_z_cc(zin, cc_tmp)
        } else {
            false
        }
    }

    /// If `cc_tmp` lies within the facet of `t`, lift to `zin` (see [`CLPoint::lift_z_cc`]).
    pub fn lift_z_if_in_facet(&mut self, zin: f64, cc_tmp: &CCPoint, t: &Triangle) -> bool {
        if cc_tmp.is_inside_triangle(t) {
            self.lift_z_cc(zin, cc_tmp)
        } else {
            false
        }
    }

    /// True if the CL-point is below the top of the triangle's bounding box.
    pub fn below(&self, t: &Triangle) -> bool {
        self.z < t.bb.maxpt.z
    }
}

impl From<Point> for CLPoint {
    fn from(p: Point) -> Self {
        CLPoint {
            pos: p,
            cc: CCPoint::default(),
        }
    }
}

impl Deref for CLPoint {
    type Target = Point;
    fn deref(&self) -> &Point {
        &self.pos
    }
}

impl DerefMut for CLPoint {
    fn deref_mut(&mut self) -> &mut Point {
        &mut self.pos
    }
}

impl fmt::Display for CLPoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "CL({}, {}, {}) cc={}", self.x, self.y, self.z, self.cc)
    }
}
