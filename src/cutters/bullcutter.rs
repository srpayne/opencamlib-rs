use std::fmt;

use super::{AlignedEllipse, Ellipse, MillingCutter};
use crate::algo::fiber::Fiber;
use crate::algo::interval::Interval;
use crate::common::numeric::{is_zero_tol, square};
use crate::geo::{CCPoint, CCType, Point};

/// Bull-nose (toroidal) cutter: a flat bottom of radius `radius1` with a corner radius
/// `radius2`.
#[derive(Debug, Clone, PartialEq)]
pub struct BullCutter {
    diameter: f64,
    radius: f64,
    length: f64,
    /// Radius of the flat (cylindrical) middle part.
    radius1: f64,
    /// Corner radius.
    radius2: f64,
}

impl BullCutter {
    /// A bull-nose cutter with diameter `d`, corner radius `r` and length `l`.
    pub fn new(d: f64, r: f64, l: f64) -> Self {
        assert!(d > 0.0, "BullCutter diameter must be positive");
        assert!(l > 0.0, "BullCutter length must be positive");
        let radius1 = d / 2.0 - r;
        assert!(radius1 > 0.0, "BullCutter corner radius must be less than d/2");
        BullCutter {
            diameter: d,
            radius: d / 2.0,
            length: l,
            radius1,
            radius2: r,
        }
    }

    /// Corner radius.
    pub fn corner_radius(&self) -> f64 {
        self.radius2
    }
}

impl MillingCutter for BullCutter {
    fn diameter(&self) -> f64 {
        self.diameter
    }
    fn radius(&self) -> f64 {
        self.radius
    }
    fn length(&self) -> f64 {
        self.length
    }

    fn height(&self, r: f64) -> f64 {
        if r <= self.radius1 {
            0.0 // cylinder
        } else if r <= self.radius {
            self.radius2 - (square(self.radius2) - square(r - self.radius1)).sqrt() // toroid
        } else {
            -1.0
        }
    }

    fn width(&self, h: f64) -> f64 {
        if h >= self.radius2 {
            self.radius
        } else {
            self.radius1 + (square(self.radius2) - square(self.radius2 - h)).sqrt()
        }
    }

    fn xy_normal_length(&self) -> f64 {
        self.radius1
    }
    fn normal_length(&self) -> f64 {
        self.radius2
    }
    fn center_height(&self) -> f64 {
        self.radius2
    }

    fn offset_cutter(&self, d: f64) -> Option<Box<dyn MillingCutter>> {
        Some(Box::new(BullCutter::new(
            self.diameter + 2.0 * d,
            self.radius2 + d,
            self.length + d,
        )))
    }

    fn single_edge_drop_canonical(&self, u1: &Point, u2: &Point) -> (f64, f64) {
        if is_zero_tol(u1.z - u2.z) {
            // horizontal edge special case
            return (0.0, u1.z - self.height(u1.y));
        }
        // General case: the edge, seen along its own direction, cuts the toroid's
        // ring-circle plane in an ellipse; the contact lies on the offset ellipse.
        let b_axis = self.radius2;
        let theta = ((u2.z - u1.z) / (u2.x - u1.x)).atan(); // slope of the edge
        let a_axis = (self.radius2 / theta.sin()).abs();
        let ellcenter = Point::new(0.0, u1.y, 0.0);
        let mut e = Ellipse::new(ellcenter, a_axis, b_axis, self.radius1);
        e.solver_brent();
        e.set_ellipse_position_hi(u1, u2);
        let ell_ccp = e.e_point_hi();
        let cc_tmp_u = ell_ccp.closest_point(u1, u2);
        (cc_tmp_u.x, e.center_z() - self.radius2)
    }

    fn general_edge_push(&self, f: &Fiber, i: &mut Interval, p1: &Point, p2: &Point) -> bool {
        let mut result = false;
        if is_zero_tol((*p2 - *p1).xy_norm()) {
            return result; // vertical edge
        }
        if is_zero_tol(p2.z - p1.z) {
            return result; // horizontal edge, handled by horiz_edge_push
        }
        // intersect the edge with the plane at z = fiber.z + radius2
        let tplane = (f.p1.z + self.radius2 - p1.z) / (p2.z - p1.z);
        let ell_center = *p1 + tplane * (*p2 - *p1);
        let mut major_dir = *p2 - *p1;
        major_dir.z = 0.0;
        major_dir.xy_normalize();
        let minor_dir = major_dir.xy_perp();
        let theta = ((p2.z - p1.z) / (*p2 - *p1).xy_norm()).atan();
        let major_length = (self.radius2 / theta.sin()).abs();
        let minor_length = self.radius2;
        let mut e = AlignedEllipse::new(
            ell_center,
            major_length,
            minor_length,
            self.radius1,
            major_dir,
            minor_dir,
        );
        if e.aligned_solver(f) {
            // pseudo cc-points on the ellipse and cylinder
            let pseudo_cc = e.e_point1();
            let pseudo_cc2 = e.e_point2();
            let cc = CCPoint::new(pseudo_cc.closest_point(p1, p2), CCType::EdgePos);
            let cc2 = CCPoint::new(pseudo_cc2.closest_point(p1, p2), CCType::EdgePos);
            let down = Point::new(0.0, 0.0, self.center_height());
            let cl = e.oe_point1() - down;
            let cl2 = e.oe_point2() - down;
            let cl_t = f.tval(&cl);
            let cl_t2 = f.tval(&cl2);
            if i.update_if_cc_in_edge_and_true(cl_t, &cc, p1, p2, true) {
                result = true;
            }
            if i.update_if_cc_in_edge_and_true(cl_t2, &cc2, p1, p2, true) {
                result = true;
            }
        }
        result
    }
}

impl fmt::Display for BullCutter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "BullCutter(d={}, r1={} r2={}, L={})",
            self.diameter, self.radius1, self.radius2, self.length
        )
    }
}
