use std::fmt;

use super::MillingCutter;
use crate::algo::fiber::Fiber;
use crate::algo::interval::Interval;
use crate::common::numeric::{is_zero_tol, square};
use crate::geo::{CCPoint, CCType, Point};

/// Ball-nose (spherical) cutter.
#[derive(Debug, Clone, PartialEq)]
pub struct BallCutter {
    diameter: f64,
    radius: f64,
    length: f64,
}

impl BallCutter {
    /// A ball end mill with diameter `d` and length `l`.
    pub fn new(d: f64, l: f64) -> Self {
        assert!(d > 0.0, "BallCutter diameter must be positive");
        assert!(l > 0.0, "BallCutter length must be positive");
        BallCutter {
            diameter: d,
            radius: d / 2.0,
            length: l,
        }
    }

    fn calc_cc_and_update_interval(
        &self,
        t: f64,
        p1: &Point,
        p2: &Point,
        f: &Fiber,
        i: &mut Interval,
    ) -> bool {
        let cl_center = f.point(t) + Point::new(0.0, 0.0, self.radius);
        // cc-point is the point on the edge closest to the ball center
        let cc_tmp = CCPoint::new(cl_center.closest_point(p1, p2), CCType::EdgeBall);
        let cond = cl_center.z - cc_tmp.z >= 0.0;
        i.update_if_cc_in_edge_and_true(t, &cc_tmp, p1, p2, cond)
    }
}

impl MillingCutter for BallCutter {
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
        self.radius - (square(self.radius) - square(r)).sqrt()
    }

    fn width(&self, h: f64) -> f64 {
        if h >= self.radius {
            self.radius
        } else {
            (square(self.radius) - square(self.radius - h)).sqrt()
        }
    }

    fn xy_normal_length(&self) -> f64 {
        0.0
    }
    fn normal_length(&self) -> f64 {
        self.radius
    }
    fn center_height(&self) -> f64 {
        self.radius
    }

    fn offset_cutter(&self, d: f64) -> Option<Box<dyn MillingCutter>> {
        Some(Box::new(BallCutter::new(
            self.diameter + 2.0 * d,
            self.length + d,
        )))
    }

    fn single_edge_drop_canonical(&self, u1: &Point, u2: &Point) -> (f64, f64) {
        let s = (square(self.radius) - square(u1.y)).sqrt();
        // (dz, -du) is a normal to the edge in the (u, z) plane
        let mut normal = Point::new(u2.z - u1.z, -(u2.x - u1.x), 0.0);
        normal.xy_normalize();
        if normal.y < 0.0 {
            normal = -1.0 * normal; // flip so it points upward
        }
        let mut cc = Point::new(-s * normal.x, u1.y, 0.0);
        cc.z_project_onto_edge(u1, u2);
        let cl_z = cc.z + s * normal.y - self.radius;
        (cc.x, cl_z)
    }

    /// Intersect the fiber, raised by `radius`, with a cylinder of radius `radius` around
    /// the edge.
    fn general_edge_push(&self, f: &Fiber, i: &mut Interval, p1: &Point, p2: &Point) -> bool {
        let mut result = false;
        if is_zero_tol((*p2 - *p1).xy_norm()) {
            return result; // vertical edge
        }
        let up = Point::new(0.0, 0.0, self.radius);
        let ufp1 = f.p1 + up;
        let ufp2 = f.p2 + up;
        let ab = *p2 - *p1; // cylinder axis
        let ao = ufp1 - *p1;
        let ao_x_ab = ao.cross(&ab);
        let v_x_ab = (ufp2 - ufp1).cross(&ab);
        let ab2 = ab.dot(&ab);
        let a = v_x_ab.dot(&v_x_ab);
        let b = 2.0 * v_x_ab.dot(&ao_x_ab);
        let c = ao_x_ab.dot(&ao_x_ab) - (self.radius * self.radius * ab2);
        let discr = b * b - 4.0 * a * c;
        if is_zero_tol(discr) {
            // tangent case, one root
            let t1 = -b / (2.0 * a);
            if self.calc_cc_and_update_interval(t1, p1, p2, f, i) {
                result = true;
            }
        } else if discr > 0.0 {
            let t1 = (-b + discr.sqrt()) / (2.0 * a);
            let t2 = (-b - discr.sqrt()) / (2.0 * a);
            if self.calc_cc_and_update_interval(t1, p1, p2, f, i) {
                result = true;
            }
            if self.calc_cc_and_update_interval(t2, p1, p2, f, i) {
                result = true;
            }
        }
        result
    }
}

impl fmt::Display for BallCutter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "BallCutter(d={}, r={}, L={})",
            self.diameter, self.radius, self.length
        )
    }
}
