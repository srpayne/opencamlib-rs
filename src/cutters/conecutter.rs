use std::fmt;

use super::{general_facet_push, CompositeCutter, MillingCutter};
use crate::algo::fiber::Fiber;
use crate::algo::interval::Interval;
use crate::common::numeric::{is_zero_tol, sign, square, xy_line_line_intersection};
use crate::geo::{CCPoint, CCType, CLPoint, Point, Triangle};

/// Conical (tapered / V-bit) cutter.
#[derive(Debug, Clone, PartialEq)]
pub struct ConeCutter {
    diameter: f64,
    radius: f64,
    length: f64,
    /// Half-angle of the cone, in radians.
    angle: f64,
    /// Height of the conical part: `radius / tan(angle)`.
    center_height: f64,
}

impl ConeCutter {
    /// A cone cutter with diameter `d` and half-angle `angle` (radians). The shaft above
    /// the cone is 10 units long.
    pub fn new(d: f64, angle: f64) -> Self {
        ConeCutter::with_length(d, angle, 10.0)
    }

    /// A cone cutter with diameter `d`, half-angle `angle` (radians) and a shaft of
    /// length `l` above the cone.
    pub fn with_length(d: f64, angle: f64, l: f64) -> Self {
        assert!(angle > 0.0, "ConeCutter angle must be positive");
        let radius = d / 2.0;
        let center_height = radius / angle.tan();
        assert!(center_height > 0.0, "ConeCutter angle must be below pi/2");
        ConeCutter {
            diameter: d,
            radius,
            length: center_height + l,
            angle,
            center_height,
        }
    }

    /// Half-angle of the cone, in radians.
    pub fn angle(&self) -> f64 {
        self.angle
    }

    /// The cone base circle (at `center_height` above the fiber) touches the edge.
    fn circle_cc(&self, t: f64, p1: &Point, p2: &Point, f: &Fiber, i: &mut Interval) -> bool {
        let t_cc = (f.p1.z + self.center_height - p1.z) / (p2.z - p1.z);
        let cc_tmp = CCPoint::new(*p1 + t_cc * (*p2 - *p1), CCType::EdgeConeBase);
        i.update_if_cc_in_edge_and_true(t, &cc_tmp, p1, p2, true)
    }

    /// The cone surface touches the edge along the line from `tang` to the tip.
    #[allow(clippy::too_many_arguments)]
    fn cone_cc(
        &self,
        tang: &Point,
        tip: &Point,
        base: &Point,
        p1: &Point,
        p2: &Point,
        f: &Fiber,
        i: &mut Interval,
    ) -> bool {
        if let Some((u, t)) = xy_line_line_intersection(&f.p1, &f.p2, tang, tip) {
            if (0.0..=1.0).contains(&t) {
                let mut cc_tmp = CCPoint::new(*base + t * (*tip - *base), CCType::EdgeCone);
                cc_tmp.z_project_onto_edge(p1, p2);
                return i.update_if_cc_in_edge_and_true(u, &cc_tmp, p1, p2, true);
            }
        }
        false
    }
}

impl MillingCutter for ConeCutter {
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
        r / self.angle.tan()
    }

    fn width(&self, h: f64) -> f64 {
        if h < self.center_height {
            h * self.angle.tan()
        } else {
            self.radius
        }
    }

    fn xy_normal_length(&self) -> f64 {
        self.radius
    }
    fn normal_length(&self) -> f64 {
        0.0
    }
    fn center_height(&self) -> f64 {
        self.center_height
    }

    fn offset_cutter(&self, d: f64) -> Option<Box<dyn MillingCutter>> {
        Some(Box::new(CompositeCutter::ball_cone(
            2.0 * d,
            self.diameter + 2.0 * d,
            self.angle,
        )))
    }

    /// The cone touches a facet either with its tip or with the rim of its base.
    fn facet_drop(&self, cl: &mut CLPoint, t: &Triangle) -> bool {
        let mut normal = t.up_normal();
        if is_zero_tol(normal.z) {
            return false; // vertical surface
        }
        if is_zero_tol(normal.x) && is_zero_tol(normal.y) {
            // horizontal plane special case
            let cc_tmp = CCPoint::from_xyz(cl.x, cl.y, t.p[0].z, CCType::FacetTip);
            return cl.lift_z_if_in_facet(cc_tmp.z, &cc_tmp, t);
        }
        let (a, b, c) = (normal.x, normal.y, normal.z);
        let d = -normal.dot(&t.p[0]);
        normal.xy_normalize();
        // contact with the rim of the base, on the uphill side
        let mut cyl_cc_tmp = CCPoint::new(cl.pos - self.radius * normal, CCType::FacetCyl);
        cyl_cc_tmp.z = (1.0 / c) * (-d - a * cyl_cc_tmp.x - b * cyl_cc_tmp.y);
        let cyl_cl_z = cyl_cc_tmp.z - self.center_height;
        // contact with the tip
        let mut tip_cc_tmp = CCPoint::from_xyz(cl.x, cl.y, 0.0, CCType::FacetTip);
        tip_cc_tmp.z = (1.0 / c) * (-d - a * tip_cc_tmp.x - b * tip_cc_tmp.y);
        let tip_cl_z = tip_cc_tmp.z;
        // The C++ library offsets the rim contact by the total cutter length instead of the
        // cone height, and skips the rim test whenever the tip test lifts the cutter. On
        // facets steeper than the cone that lets the rim gouge the facet.
        //
        // The cone touches the facet's plane with its tip if the plane is flatter than the
        // cone, otherwise with the uphill point of its rim. Only that contact rules out
        // edge and vertex contacts, so only it makes this test return true. The other
        // contact is still a valid lower bound for the cutter position.
        let (best_z, best_cc, other_z, other_cc) = if cyl_cl_z > tip_cl_z {
            (cyl_cl_z, cyl_cc_tmp, tip_cl_z, tip_cc_tmp)
        } else {
            (tip_cl_z, tip_cc_tmp, cyl_cl_z, cyl_cc_tmp)
        };
        if best_cc.is_inside_triangle(t) {
            cl.lift_z_cc(best_z, &best_cc)
        } else {
            cl.lift_z_if_in_facet(other_z, &other_cc, t);
            false
        }
    }

    fn single_edge_drop_canonical(&self, u1: &Point, u2: &Point) -> (f64, f64) {
        let d = u1.y;
        let m = (u2.z - u1.z) / (u2.x - u1.x); // slope of the edge
        let xu = (square(self.radius) - square(u1.y)).sqrt(); // edge leaves the base circle here
        // slope of the cone's hyperbolic section at xu
        let mu = (self.center_height / self.radius) * xu / (square(xu) + square(d)).sqrt();
        let hyperbola_case = m.abs() <= mu.abs();
        let ccu = if hyperbola_case {
            // tangent point on the hyperbola z = (h/R)*sqrt(x^2 + d^2):
            //   x^2 = R^2 m^2 d^2 / (h^2 - R^2 m^2)
            // The C++ library uses the total cutter length here instead of the cone
            // height h, which puts the contact in the wrong place and lets the cone gouge
            // sloping edges. This port uses h.
            sign(m)
                * (square(self.radius) * square(m) * square(d)
                    / (square(self.center_height) - square(self.radius) * square(m)))
                .sqrt()
        } else {
            sign(m) * xu // the edge touches the rim of the base circle
        };
        let mut cc_tmp = Point::new(ccu, d, 0.0);
        cc_tmp.z_project_onto_edge(u1, u2);
        let cl_z = if hyperbola_case {
            cc_tmp.z - self.center_height
                + (self.radius - (square(ccu) + square(d)).sqrt()) / self.angle.tan()
        } else {
            cc_tmp.z - self.center_height
        };
        (ccu, cl_z)
    }

    /// Push both the tip and the base rim of the cone against the facet.
    fn facet_push(&self, fib: &Fiber, i: &mut Interval, t: &Triangle) -> bool {
        let tip = general_facet_push(0.0, 0.0, 0.0, fib, i, t);
        let base = general_facet_push(0.0, self.center_height, self.xy_normal_length(), fib, i, t);
        tip || base
    }

    fn general_edge_push(&self, f: &Fiber, i: &mut Interval, p1: &Point, p2: &Point) -> bool {
        let mut result = false;
        if is_zero_tol(p2.z - p1.z) {
            return result; // horizontal edge
        }
        // where the edge crosses the plane of the tip ...
        let t_tip = (f.p1.z - p1.z) / (p2.z - p1.z);
        let p_tip = *p1 + t_tip * (*p2 - *p1);
        // ... and the plane of the base, projected down into the fiber plane
        let t_base = (f.p1.z + self.center_height - p1.z) / (p2.z - p1.z);
        let mut p_base = *p1 + t_base * (*p2 - *p1);
        p_base.z = f.p1.z;
        let l = (p_base - p_tip).xy_norm();

        // base circle against the edge: intersect the fiber with a circle around p_base
        let d = p_base.xy_distance_to_line(&f.p1, &f.p2);
        if d <= self.radius {
            let dx = f.p2.x - f.p1.x;
            let dy = f.p2.y - f.p1.y;
            let dr = (square(dx) + square(dy)).sqrt();
            let det = (f.p1.x - p_base.x) * (f.p2.y - p_base.y) - (f.p2.x - p_base.x) * (f.p1.y - p_base.y);
            let discr = square(self.radius) * square(dr) - square(det);
            if discr >= 0.0 {
                if discr == 0.0 {
                    // tangent case
                    let x_tang = (det * dy) / square(dr);
                    let y_tang = -(det * dx) / square(dr);
                    let p_tang = Point::new_xy(x_tang + p_base.x, y_tang + p_base.y);
                    let t_tang = f.tval(&p_tang);
                    if self.circle_cc(t_tang, p1, p2, f, i) {
                        result = true;
                    }
                } else {
                    let sq = discr.sqrt();
                    let x_pos = (det * dy + sign(dy) * dx * sq) / square(dr);
                    let y_pos = (-det * dx + dy.abs() * sq) / square(dr);
                    let cl_pos = Point::new_xy(x_pos + p_base.x, y_pos + p_base.y);
                    let t_pos = f.tval(&cl_pos);
                    let x_neg = (det * dy - sign(dy) * dx * sq) / square(dr);
                    let y_neg = (-det * dx - dy.abs() * sq) / square(dr);
                    let cl_neg = Point::new_xy(x_neg + p_base.x, y_neg + p_base.y);
                    let t_neg = f.tval(&cl_neg);
                    if self.circle_cc(t_pos, p1, p2, f, i) {
                        result = true;
                    }
                    if self.circle_cc(t_neg, p1, p2, f, i) {
                        result = true;
                    }
                }
            }
        }

        // cone surface against the edge: tangent lines from p_tip to the base circle
        if l > self.radius {
            let mut p_mid = 0.5 * (p_base + p_tip);
            p_mid.z = f.p1.z;
            let r_tang = l / 2.0;
            let mut pd = p_mid - p_base;
            pd.z = 0.0;
            let dist = pd.xy_norm(); // distance between circle centers
            let case1 = is_zero_tol(dist) && is_zero_tol((self.radius - r_tang).abs());
            let case2 = dist > (self.radius + r_tang); // circles don't intersect
            let case3 = dist < (self.radius - r_tang).abs(); // one circle contains the other
            let case4 = is_zero_tol(dist - (self.radius + r_tang)); // tangent
            if !(case1 || case2 || case3 || case4) {
                let a = (-square(r_tang) + square(self.radius) + square(dist)) / (2.0 * dist);
                let v2 = p_base + (a / dist) * pd;
                let h = (square(self.radius) - square(a)).sqrt();
                let ofs = Point::new_xy(-pd.y * (h / dist), pd.x * (h / dist));
                let tang1 = v2 + ofs;
                let tang2 = v2 - ofs;
                if self.cone_cc(&tang1, &p_tip, &p_base, p1, p2, f, i) {
                    result = true;
                }
                if self.cone_cc(&tang2, &p_tip, &p_base, p1, p2, f, i) {
                    result = true;
                }
            }
        }
        result
    }
}

impl fmt::Display for ConeCutter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "ConeCutter (d={}, angle={}, L={})",
            self.diameter, self.angle, self.length
        )
    }
}
