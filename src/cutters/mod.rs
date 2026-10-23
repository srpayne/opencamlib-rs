//! Milling cutters and their drop-cutter / push-cutter contact calculations.
//!
//! [`MillingCutter`] holds the generic algorithms as default methods. Each cutter type
//! supplies its profile (`height`, `width`), its edge-drop solution in canonical
//! coordinates, and overrides whichever tests need special handling.

mod ballcutter;
mod bullcutter;
mod compositecutter;
mod conecutter;
mod cylcutter;
mod ellipse;

pub use ballcutter::BallCutter;
pub use bullcutter::BullCutter;
pub use compositecutter::CompositeCutter;
pub use conecutter::ConeCutter;
pub use cylcutter::CylCutter;
pub use ellipse::{AlignedEllipse, Ellipse, EllipsePosition};

use std::fmt;

use crate::algo::fiber::Fiber;
use crate::algo::interval::Interval;
use crate::common::numeric::{is_zero_tol, square, two_by_two_solver, xy_line_line_intersection};
use crate::geo::{CCPoint, CCType, CLPoint, Point, STLSurf, Triangle};

/// A milling cutter.
///
/// Implementors provide the cutter geometry; the drop-cutter tests ([`facet_drop`],
/// [`vertex_drop`], [`edge_drop`]) and push-cutter tests ([`facet_push`], [`vertex_push`],
/// [`edge_push`]) have generic default implementations that a cutter can override.
///
/// [`facet_drop`]: MillingCutter::facet_drop
/// [`vertex_drop`]: MillingCutter::vertex_drop
/// [`edge_drop`]: MillingCutter::edge_drop
/// [`facet_push`]: MillingCutter::facet_push
/// [`vertex_push`]: MillingCutter::vertex_push
/// [`edge_push`]: MillingCutter::edge_push
pub trait MillingCutter: fmt::Display + Send + Sync {
    /// Diameter of the cutter.
    fn diameter(&self) -> f64;
    /// Radius of the cutter.
    fn radius(&self) -> f64;
    /// Length of the cutter.
    fn length(&self) -> f64;

    /// Height of the cutter profile at radius `r`.
    fn height(&self, r: f64) -> f64;
    /// Width (radius) of the cutter profile at height `h`.
    fn width(&self, h: f64) -> f64;

    /// Generic facet-contact parameters: the CC-point on a facet with unit normal `n` and
    /// XY-normal `nxy` is `cl - (xy_normal_length * nxy + normal_length * n)`, and the
    /// center of that offset is `center_height` above the cutter tip.
    fn xy_normal_length(&self) -> f64;
    /// See [`MillingCutter::xy_normal_length`].
    fn normal_length(&self) -> f64;
    /// See [`MillingCutter::xy_normal_length`].
    fn center_height(&self) -> f64;

    /// A cutter offset by `d` in all directions, if this cutter type supports it.
    fn offset_cutter(&self, d: f64) -> Option<Box<dyn MillingCutter>>;

    /// Drop-cutter against an edge in canonical coordinates: the edge runs from `u1` to
    /// `u2` parallel to the x-axis at y = `u1.y`, and the cutter is at the origin.
    /// Returns `(cc_u, cl_z)`: the x-coordinate of the CC-point and the resulting CL z.
    fn single_edge_drop_canonical(&self, u1: &Point, u2: &Point) -> (f64, f64);

    /// Push-cutter against a general (sloping) edge. Cutters with a rounded or conical
    /// profile override this.
    fn general_edge_push(&self, _f: &Fiber, _i: &mut Interval, _p1: &Point, _p2: &Point) -> bool {
        false
    }

    /// True if the cutter's XY footprint at `cl` overlaps the triangle's bounding box.
    fn overlaps(&self, cl: &Point, t: &Triangle) -> bool {
        let r = self.radius();
        !(t.bb.maxpt.x < cl.x - r
            || t.bb.minpt.x > cl.x + r
            || t.bb.maxpt.y < cl.y - r
            || t.bb.minpt.y > cl.y + r)
    }

    /// Drop-cutter: lift `cl` so the cutter touches a vertex of `t`. Returns true if lifted.
    fn vertex_drop(&self, cl: &mut CLPoint, t: &Triangle) -> bool {
        let mut result = false;
        for p in &t.p {
            let q = cl.xy_distance(p);
            if q <= self.radius() {
                let cc_tmp = CCPoint::new(*p, CCType::Vertex);
                if cl.lift_z_cc(p.z - self.height(q), &cc_tmp) {
                    result = true;
                }
            }
        }
        result
    }

    /// Drop-cutter: lift `cl` so the cutter touches the facet of `t`. Returns true if lifted.
    fn facet_drop(&self, cl: &mut CLPoint, t: &Triangle) -> bool {
        let mut normal = t.up_normal();
        if is_zero_tol(normal.z) {
            return false; // can't drop against a vertical surface
        }
        if is_zero_tol(normal.x) && is_zero_tol(normal.y) {
            // horizontal plane special case
            let cc_tmp = CCPoint::from_xyz(cl.x, cl.y, t.p[0].z, CCType::Facet);
            return cl.lift_z_if_in_facet(cc_tmp.z, &cc_tmp, t);
        }
        let d = -normal.dot(&t.p[0]);
        normal.normalize();
        let mut xy_normal = Point::new(normal.x, normal.y, 0.0);
        xy_normal.xy_normalize();
        let radiusvector = self.xy_normal_length() * xy_normal + self.normal_length() * normal;
        let mut cc_tmp = CCPoint::new(cl.pos - radiusvector, CCType::Facet);
        // the cc-point lies in the plane of the facet
        cc_tmp.z = (1.0 / normal.z) * (-d - normal.x * cc_tmp.x - normal.y * cc_tmp.y);
        let tip_z = cc_tmp.z + radiusvector.z - self.center_height();
        cl.lift_z_if_in_facet(tip_z, &cc_tmp, t)
    }

    /// Drop-cutter: lift `cl` so the cutter touches an edge of `t`. Returns true if lifted.
    fn edge_drop(&self, cl: &mut CLPoint, t: &Triangle) -> bool {
        let mut result = false;
        for n in 0..3 {
            let p1 = t.p[n];
            let p2 = t.p[(n + 1) % 3];
            if !is_zero_tol(p1.x - p2.x) || !is_zero_tol(p1.y - p2.y) {
                let d = cl.xy_distance_to_line(&p1, &p2);
                if d <= self.radius() && single_edge_drop(self, cl, &p1, &p2, d) {
                    result = true;
                }
            }
        }
        result
    }

    /// Drop-cutter against triangle `t`: run the facet, vertex and edge tests as needed.
    fn drop_cutter(&self, cl: &mut CLPoint, t: &Triangle) -> bool {
        let mut facet = false;
        let mut vertex = false;
        let mut edge = false;
        if cl.below(t) {
            facet = self.facet_drop(cl, t);
            if !facet {
                // a facet contact rules out edge and vertex contacts
                vertex = self.vertex_drop(cl, t);
                if cl.below(t) {
                    edge = self.edge_drop(cl, t);
                }
            }
        }
        facet || vertex || edge
    }

    /// Drop-cutter against every triangle of `s` (brute force, no kd-tree).
    fn drop_cutter_stl(&self, cl: &mut CLPoint, s: &STLSurf) -> bool {
        let mut result = false;
        for t in &s.tris {
            if self.drop_cutter(cl, t) {
                result = true;
            }
        }
        result
    }

    /// Push-cutter along fiber `f` against triangle `t`, updating interval `i`.
    fn push_cutter(&self, f: &Fiber, i: &mut Interval, t: &Triangle) -> bool {
        let v = self.vertex_push(f, i, t);
        let fa = self.facet_push(f, i, t);
        let e = self.edge_push(f, i, t);
        v || fa || e
    }

    /// Push-cutter against the vertices of `t`.
    fn vertex_push(&self, f: &Fiber, i: &mut Interval, t: &Triangle) -> bool {
        let mut result = false;
        for p in &t.p {
            if single_vertex_push(self, f, i, p, CCType::Vertex) {
                result = true;
            }
        }
        result
    }

    /// Push-cutter against the facet of `t`.
    fn facet_push(&self, f: &Fiber, i: &mut Interval, t: &Triangle) -> bool {
        general_facet_push(
            self.normal_length(),
            self.center_height(),
            self.xy_normal_length(),
            f,
            i,
            t,
        )
    }

    /// Push-cutter against the edges of `t`.
    fn edge_push(&self, f: &Fiber, i: &mut Interval, t: &Triangle) -> bool {
        let mut result = false;
        for n in 0..3 {
            let p1 = t.p[n];
            let p2 = t.p[(n + 1) % 3];
            if single_edge_push(self, f, i, &p1, &p2) {
                result = true;
            }
        }
        result
    }
}

/// Drop the cutter against the edge p1-p2, at XY-distance `d` from `cl`.
pub(crate) fn single_edge_drop<C: MillingCutter + ?Sized>(
    c: &C,
    cl: &mut CLPoint,
    p1: &Point,
    p2: &Point,
    d: f64,
) -> bool {
    let v = *p2 - *p1;
    let mut vxy = Point::new(v.x, v.y, 0.0);
    vxy.xy_normalize();
    let sc = cl.xy_closest_point(p1, p2);
    // canonical coordinates: edge along x at y = d
    let u1 = Point::new((*p1 - sc).dot(&vxy), d, p1.z);
    let u2 = Point::new((*p2 - sc).dot(&vxy), d, p2.z);
    let (cc_u, cl_z) = c.single_edge_drop_canonical(&u1, &u2);
    let mut cc_tmp = CCPoint::new(sc + cc_u * vxy, CCType::Edge);
    cc_tmp.z_project_onto_edge(p1, p2);
    cl.lift_z_if_inside_points(cl_z, &cc_tmp, p1, p2)
}

/// Push the cutter along `f` against the single vertex `p`.
pub(crate) fn single_vertex_push<C: MillingCutter + ?Sized>(
    c: &C,
    f: &Fiber,
    i: &mut Interval,
    p: &Point,
    cctyp: CCType,
) -> bool {
    if p.z >= f.p1.z && p.z <= f.p1.z + c.length() {
        let pq = p.xy_closest_point(&f.p1, &f.p2); // closest point on fiber
        let q = (*p - pq).xy_norm(); // XY distance from fiber to p
        let h = p.z - f.p1.z;
        let cwidth = c.width(h);
        if q <= cwidth {
            let ofs = (square(cwidth) - square(q)).sqrt(); // distance along fiber
            let start = pq - ofs * f.dir;
            let stop = pq + ofs * f.dir;
            let cc_tmp = CCPoint::new(*p, cctyp);
            i.update_upper(f.tval(&stop), &cc_tmp);
            i.update_lower(f.tval(&start), &cc_tmp);
            return true;
        }
    }
    false
}

/// Push-cutter against a facet, for a cutter described by the generic facet parameters.
pub(crate) fn general_facet_push(
    normal_length: f64,
    center_height: f64,
    xy_normal_length: f64,
    fib: &Fiber,
    i: &mut Interval,
    t: &Triangle,
) -> bool {
    let mut normal = t.up_normal();
    if normal.z_parallel() {
        return false; // can't push against a horizontal plane
    }
    normal.normalize();
    let mut xy_normal = normal;
    xy_normal.z = 0.0;
    xy_normal.xy_normalize();

    // Solve for (u, v), the coordinates of the cc-point within the facet:
    //   cc = p0 + u*(p1-p0) + v*(p2-p0)
    let c = t.p[1].z - t.p[0].z;
    let d = t.p[2].z - t.p[0].z;
    let f = -t.p[0].z - normal_length * normal.z + fib.p1.z + center_height;
    let (u, v, tval) = if fib.p1.y == fib.p2.y {
        // X-fiber
        let a = t.p[1].y - t.p[0].y;
        let b = t.p[2].y - t.p[0].y;
        let e = -t.p[0].y - normal_length * normal.y - xy_normal_length * xy_normal.y + fib.p1.y;
        let Some((u, v)) = two_by_two_solver(a, b, c, d, e, f) else {
            return false;
        };
        let tval = (1.0 / (fib.p2.x - fib.p1.x))
            * (t.p[0].x + normal_length * normal.x + xy_normal_length * xy_normal.x - fib.p1.x
                + u * (t.p[1].x - t.p[0].x)
                + v * (t.p[2].x - t.p[0].x));
        (u, v, tval)
    } else if fib.p1.x == fib.p2.x {
        // Y-fiber
        let a = t.p[1].x - t.p[0].x;
        let b = t.p[2].x - t.p[0].x;
        let e = -t.p[0].x - normal_length * normal.x - xy_normal_length * xy_normal.x + fib.p1.x;
        let Some((u, v)) = two_by_two_solver(a, b, c, d, e, f) else {
            return false;
        };
        let tval = (1.0 / (fib.p2.y - fib.p1.y))
            * (t.p[0].y + normal_length * normal.y + xy_normal_length * xy_normal.y - fib.p1.y
                + u * (t.p[1].y - t.p[0].y)
                + v * (t.p[2].y - t.p[0].y));
        (u, v, tval)
    } else {
        return false; // fibers are always parallel to x or y
    };
    let cc = CCPoint::new(
        t.p[0] + u * (t.p[1] - t.p[0]) + v * (t.p[2] - t.p[0]),
        CCType::Facet,
    );
    if !cc.is_inside_triangle(t) {
        return false;
    }
    i.update(tval, &cc);
    true
}

/// Push-cutter against the single edge p1-p2.
pub(crate) fn single_edge_push<C: MillingCutter + ?Sized>(
    c: &C,
    f: &Fiber,
    i: &mut Interval,
    p1: &Point,
    p2: &Point,
) -> bool {
    if horiz_edge_push(c, f, i, p1, p2) {
        return true;
    }
    let shaft = shaft_edge_push(c, f, i, p1, p2);
    let general = c.general_edge_push(f, i, p1, p2);
    shaft || general
}

/// Push the cylindrical shaft of the cutter against the edge p1-p2.
fn shaft_edge_push<C: MillingCutter + ?Sized>(
    c: &C,
    f: &Fiber,
    i: &mut Interval,
    p1: &Point,
    p2: &Point,
) -> bool {
    let mut result = false;
    // XY-intersection of fiber and edge
    if let Some((u, v)) = xy_line_line_intersection(p1, p2, &f.p1, &f.p2) {
        let q = *p1 + u * (*p2 - *p1); // intersection point, on the edge
        let mut xy_tang = *p2 - *p1;
        xy_tang.z = 0.0;
        xy_tang.xy_normalize();
        let xy_normal = xy_tang.xy_perp();
        let q1 = q + c.radius() * xy_normal;
        let q2 = q1 + (*p2 - *p1);
        if let Some((u_cc, t_cl)) = xy_line_line_intersection(&q1, &q2, &f.p1, &f.p2) {
            let t_cl1 = t_cl;
            let t_cl2 = v + (v - t_cl);
            let height = f.p1.z + c.center_height();
            if calc_cc_and_update_interval(t_cl1, u_cc, &q, p1, p2, i, height, CCType::EdgeShaft) {
                result = true;
            }
            if calc_cc_and_update_interval(t_cl2, -u_cc, &q, p1, p2, i, height, CCType::EdgeShaft) {
                result = true;
            }
        }
    }
    result
}

/// Push-cutter against a horizontal edge p1-p2, where the cutter acts as a cylinder of
/// radius `width(h)`.
fn horiz_edge_push<C: MillingCutter + ?Sized>(
    c: &C,
    f: &Fiber,
    i: &mut Interval,
    p1: &Point,
    p2: &Point,
) -> bool {
    let mut result = false;
    let h = p1.z - f.p1.z; // height of edge above fiber
    if h > 0.0 && is_zero_tol(p2.z - p1.z) {
        let eff_radius = c.width(h);
        if let Some((qv, qt)) = xy_line_line_intersection(p1, p2, &f.p1, &f.p2) {
            let q = *p1 + qv * (*p2 - *p1);
            let mut xy_tang = *p2 - *p1;
            xy_tang.z = 0.0;
            xy_tang.xy_normalize();
            let xy_normal = xy_tang.xy_perp();
            let q1 = q + eff_radius * xy_normal;
            let q2 = q1 + (*p2 - *p1);
            if let Some((ccv, clt)) = xy_line_line_intersection(&q1, &q2, &f.p1, &f.p2) {
                let t_cl1 = clt;
                let t_cl2 = qt + (qt - clt);
                if calc_cc_and_update_interval(t_cl1, ccv, &q, p1, p2, i, f.p1.z, CCType::EdgeHoriz) {
                    result = true;
                }
                if calc_cc_and_update_interval(t_cl2, -ccv, &q, p1, p2, i, f.p1.z, CCType::EdgeHoriz) {
                    result = true;
                }
            }
        }
    }
    result
}

#[allow(clippy::too_many_arguments)]
fn calc_cc_and_update_interval(
    t: f64,
    u: f64,
    q: &Point,
    p1: &Point,
    p2: &Point,
    i: &mut Interval,
    height: f64,
    cctyp: CCType,
) -> bool {
    let cc_tmp = CCPoint::new(*q + u * (*p2 - *p1), cctyp);
    let cond = cc_tmp.z >= height;
    i.update_if_cc_in_edge_and_true(t, &cc_tmp, p1, p2, cond)
}
