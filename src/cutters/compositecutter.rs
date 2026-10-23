use std::fmt;

use super::{BallCutter, BullCutter, ConeCutter, CylCutter, MillingCutter};
use crate::algo::fiber::Fiber;
use crate::algo::interval::Interval;
use crate::common::numeric::{is_zero_tol, square};
use crate::geo::{CCPoint, CCType, CLPoint, Point, Triangle};

/// A cutter built from several sub-cutters stacked on top of each other.
///
/// Sub-cutter `n` is valid for radii up to `radiusvec[n]` and heights up to `heightvec[n]`,
/// and is shifted up by `zoffset[n]`. Sub-cutters must be added from the tip upwards.
pub struct CompositeCutter {
    radiusvec: Vec<f64>,
    heightvec: Vec<f64>,
    zoffset: Vec<f64>,
    cutters: Vec<Box<dyn MillingCutter>>,
    radius: f64,
    diameter: f64,
    length: f64,
}

impl Default for CompositeCutter {
    fn default() -> Self {
        CompositeCutter::new()
    }
}

impl CompositeCutter {
    /// An empty composite cutter.
    pub fn new() -> Self {
        CompositeCutter {
            radiusvec: Vec::new(),
            heightvec: Vec::new(),
            zoffset: Vec::new(),
            cutters: Vec::new(),
            radius: 0.0,
            diameter: 0.0,
            length: 0.0,
        }
    }

    /// Add sub-cutter `c`, valid out to radius `r` and up to height `h`, shifted up by
    /// `zoff`.
    pub fn add_cutter(&mut self, c: Box<dyn MillingCutter>, r: f64, h: f64, zoff: f64) {
        self.radiusvec.push(r);
        self.heightvec.push(h);
        self.cutters.push(c);
        self.zoffset.push(zoff);
        if r > self.radius {
            self.radius = r;
            self.diameter = 2.0 * r;
        }
    }

    /// Set the total length of the cutter.
    pub fn set_length(&mut self, l: f64) {
        self.length = l;
    }

    /// A cylindrical cutter expressed as a composite (useful for testing).
    pub fn comp_cyl(diam: f64, length: f64) -> Self {
        let mut c = CompositeCutter::new();
        c.add_cutter(Box::new(CylCutter::new(diam, length)), diam / 2.0, length, 0.0);
        c.length = length;
        c
    }

    /// A ball cutter expressed as a composite (useful for testing).
    pub fn comp_ball(diam: f64, length: f64) -> Self {
        let mut c = CompositeCutter::new();
        c.add_cutter(Box::new(BallCutter::new(diam, length)), diam / 2.0, length, 0.0);
        c.length = length;
        c
    }

    /// A flat tip of diameter `diam1` with a cone of half-angle `angle` up to `diam2`.
    pub fn cyl_cone(diam1: f64, diam2: f64, angle: f64) -> Self {
        let mut c = CompositeCutter::new();
        let cone_offset = -(diam1 / 2.0) / angle.tan();
        let cyl_height = 0.0;
        let cone_height = (diam2 / 2.0) / angle.tan() + cone_offset;
        c.add_cutter(Box::new(CylCutter::new(diam1, 1.0)), diam1 / 2.0, cyl_height, 0.0);
        c.add_cutter(Box::new(ConeCutter::new(diam2, angle)), diam2 / 2.0, cone_height, cone_offset);
        c.add_cutter(
            Box::new(CylCutter::new(diam2, 20.0)),
            diam2 / 2.0,
            (diam2 / 2.0) / angle.tan() + 20.0,
            cone_height,
        );
        c.length = cyl_height + cone_height + 10.0;
        c
    }

    /// A ball tip of diameter `diam1` with a tangent cone of half-angle `angle` up to
    /// `diam2`.
    pub fn ball_cone(diam1: f64, diam2: f64, angle: f64) -> Self {
        let mut c = CompositeCutter::new();
        let radius1 = diam1 / 2.0;
        let radius2 = diam2 / 2.0;
        let rcontact = radius1 * angle.cos();
        let height1 = radius1 - (square(radius1) - square(rcontact)).sqrt();
        let cone_offset = -(rcontact / angle.tan() - height1);
        let height2 = radius2 / angle.tan() + cone_offset;
        let shaft_offset = height2;
        c.add_cutter(Box::new(BallCutter::new(diam1, 1.0)), rcontact, height1, 0.0);
        c.add_cutter(Box::new(ConeCutter::new(diam2, angle)), radius2, height2, cone_offset);
        c.add_cutter(Box::new(CylCutter::new(diam2, 20.0)), radius2, height2 + 20.0, shaft_offset);
        c.length = 30.0;
        c
    }

    /// A bull-nose tip (diameter `diam1`, corner radius `radius1`) with a tangent cone of
    /// half-angle `angle` up to `diam2`.
    pub fn bull_cone(diam1: f64, radius1: f64, diam2: f64, angle: f64) -> Self {
        let mut c = CompositeCutter::new();
        let h1 = radius1 * angle.sin(); // contact point is this much below the toroid ring
        let rad = (square(radius1) - square(h1)).sqrt();
        let rcontact = (diam1 / 2.0) - radius1 + rad;
        let cone_offset = -(rcontact / angle.tan() - (radius1 - h1));
        let height1 = radius1 - h1;
        let height2 = (diam2 / 2.0) / angle.tan() + cone_offset;
        let shaft_offset = height2;
        c.add_cutter(Box::new(BullCutter::new(diam1, radius1, 1.0)), rcontact, height1, 0.0);
        c.add_cutter(Box::new(ConeCutter::new(diam2, angle)), diam2 / 2.0, height2, cone_offset);
        c.add_cutter(Box::new(CylCutter::new(diam2, 20.0)), diam2 / 2.0, height2 + 20.0, shaft_offset);
        c.length = 30.0;
        c
    }

    /// A cone tip (diameter `diam1`, half-angle `angle1`) with a second cone (half-angle
    /// `angle2`) up to `diam2`.
    pub fn cone_cone(diam1: f64, angle1: f64, diam2: f64, angle2: f64) -> Self {
        let mut c = CompositeCutter::new();
        let height1 = (diam1 / 2.0) / angle1.tan();
        let tmp = (diam1 / 2.0) / angle2.tan();
        let cone_offset = -(tmp - height1);
        let height2 = (diam2 / 2.0) / angle2.tan() + cone_offset;
        let shaft_offset = height2;
        c.add_cutter(Box::new(ConeCutter::new(diam1, angle1)), diam1 / 2.0, height1, 0.0);
        c.add_cutter(Box::new(ConeCutter::new(diam2, angle2)), diam2 / 2.0, height2, cone_offset);
        c.add_cutter(Box::new(CylCutter::new(diam2, 20.0)), diam2 / 2.0, height2 + 20.0, shaft_offset);
        c.length = 30.0;
        c
    }

    fn radius_to_index(&self, r: f64) -> usize {
        (0..self.cutters.len())
            .find(|&n| self.valid_radius(n, r))
            .unwrap_or(0)
    }

    fn height_to_index(&self, h: f64) -> usize {
        (0..self.cutters.len())
            .find(|&n| self.valid_height(n, h))
            .unwrap_or(self.cutters.len().saturating_sub(1))
    }

    fn lo_limit(v: &[f64], n: usize) -> f64 {
        if n == 0 {
            -1e-6
        } else {
            v[n - 1] - 1e-6
        }
    }

    fn valid_radius(&self, n: usize, r: f64) -> bool {
        let lolimit = Self::lo_limit(&self.radiusvec, n);
        let hilimit = self.radiusvec[n] + 1e-6;
        lolimit <= r && r <= hilimit
    }

    fn valid_height(&self, n: usize, h: f64) -> bool {
        let lolimit = Self::lo_limit(&self.heightvec, n);
        let hilimit = self.heightvec[n] + 1e-6;
        lolimit <= h && h <= hilimit
    }

    /// True if the cc-point of `cl` lies within the radial range of sub-cutter `n`.
    fn cc_valid_radius(&self, n: usize, cl: &CLPoint) -> bool {
        if cl.cc.cc_type == CCType::None {
            return false;
        }
        let d = cl.xy_distance(&cl.cc);
        let lolimit = Self::lo_limit(&self.radiusvec, n);
        let hilimit = self.radiusvec[n] + 1e-6;
        !(d < lolimit || d > hilimit)
    }

    /// True if `cc` lies within the height range of sub-cutter `n`.
    fn cc_valid_height(&self, n: usize, cc: &CCPoint, f: &Fiber) -> bool {
        n == self.height_to_index(cc.z - f.p1.z)
    }

    /// Run a push-cutter test on every sub-cutter with a fiber shifted by its z-offset,
    /// and keep only the contacts within each sub-cutter's height range.
    fn composite_push<F>(&self, f: &Fiber, i: &mut Interval, t: &Triangle, push: F) -> bool
    where
        F: Fn(&dyn MillingCutter, &Fiber, &mut Interval, &Triangle) -> bool,
    {
        let mut contacts: Vec<(f64, CCPoint)> = Vec::new();
        for (n, cutter) in self.cutters.iter().enumerate() {
            let mut ci = Interval::new();
            let zoff = Point::new(0.0, 0.0, self.zoffset[n]);
            let cf = Fiber::new(f.p1 + zoff, f.p2 + zoff);
            if push(cutter.as_ref(), &cf, &mut ci, t) {
                if self.cc_valid_height(n, &ci.upper_cc, f) {
                    contacts.push((ci.upper, ci.upper_cc));
                }
                if self.cc_valid_height(n, &ci.lower_cc, f) {
                    contacts.push((ci.lower, ci.lower_cc));
                }
            }
        }
        for (t, cc) in &contacts {
            i.update(*t, cc);
        }
        !contacts.is_empty()
    }

    /// Run a drop-cutter test on every sub-cutter shifted by its z-offset, keeping contacts
    /// within each sub-cutter's radial range.
    fn composite_drop<F>(&self, cl: &mut CLPoint, t: &Triangle, cc_type: CCType, drop: F) -> bool
    where
        F: Fn(&dyn MillingCutter, &mut CLPoint, &Triangle) -> bool,
    {
        let mut result = false;
        for (n, cutter) in self.cutters.iter().enumerate() {
            let mut cl_tmp = CLPoint::from(cl.pos + Point::new(0.0, 0.0, self.zoffset[n]));
            if drop(cutter.as_ref(), &mut cl_tmp, t)
                && self.cc_valid_radius(n, &cl_tmp)
                && cl.lift_z(cl_tmp.z - self.zoffset[n])
            {
                let mut cc_tmp = cl_tmp.cc;
                cc_tmp.cc_type = cc_type;
                cl.cc = cc_tmp;
                result = true;
            }
        }
        result
    }

    /// Radii where one sub-cutter hands over to the next.
    fn boundary_radii(&self) -> impl Iterator<Item = f64> + '_ {
        let n = self.radiusvec.len();
        self.radiusvec[..n.saturating_sub(1)].iter().copied().filter(|&r| r > 0.0)
    }

    /// Facet contacts on the boundary circles between sub-cutters.
    ///
    /// Each sub-cutter's facet/edge test finds the best contact over its whole profile,
    /// which is discarded when it falls outside the sub-cutter's radial band. Within a
    /// band the cutter surface is convex, so the best contact restricted to the band is
    /// either that unrestricted contact or lies on one of the band's boundary circles.
    /// Testing the boundary circles therefore makes the composite drop exact. (The C++
    /// library does not do this, so composite cutters can gouge there.)
    fn boundary_facet_drop(&self, cl: &mut CLPoint, t: &Triangle) -> bool {
        let normal = t.up_normal();
        if is_zero_tol(normal.z) || (is_zero_tol(normal.x) && is_zero_tol(normal.y)) {
            return false; // vertical facet, or horizontal facet (touched by the tip)
        }
        let mut nxy = Point::new(normal.x, normal.y, 0.0);
        nxy.xy_normalize();
        let d = -normal.dot(&t.p[0]);
        let mut result = false;
        for r in self.boundary_radii() {
            // highest point of the facet on the circle of radius r: the uphill side
            let mut cc = CCPoint::new(cl.pos - r * nxy, CCType::Facet);
            cc.z = (-d - normal.x * cc.x - normal.y * cc.y) / normal.z;
            if cl.lift_z_if_in_facet(cc.z - self.height(r), &cc, t) {
                result = true;
            }
        }
        result
    }

    /// Edge contacts on the boundary circles between sub-cutters
    /// (see [`CompositeCutter::boundary_facet_drop`]).
    fn boundary_edge_drop(&self, cl: &mut CLPoint, t: &Triangle) -> bool {
        let mut result = false;
        for n in 0..3 {
            let p1 = t.p[n];
            let p2 = t.p[(n + 1) % 3];
            if is_zero_tol(p1.x - p2.x) && is_zero_tol(p1.y - p2.y) {
                continue; // vertical edge
            }
            let d = cl.xy_distance_to_line(&p1, &p2);
            let mut vxy = p2 - p1;
            vxy.z = 0.0;
            vxy.xy_normalize();
            let sc = cl.xy_closest_point(&p1, &p2);
            for r in self.boundary_radii() {
                if d > r {
                    continue;
                }
                let s = (square(r) - square(d)).sqrt();
                for cc_xy in [sc + s * vxy, sc - s * vxy] {
                    let mut cc = CCPoint::new(cc_xy, CCType::Edge);
                    cc.z_project_onto_edge(&p1, &p2);
                    if cl.lift_z_if_inside_points(cc.z - self.height(r), &cc, &p1, &p2) {
                        result = true;
                    }
                }
            }
        }
        result
    }
}

impl MillingCutter for CompositeCutter {
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
        let idx = self.radius_to_index(r);
        // radius_to_index accepts radii up to 1e-6 beyond a sub-cutter's radius, where e.g.
        // CylCutter::height returns its -1 "outside" value (the C++ library then lifts the
        // cutter by 1). Clamp to the sub-cutter's radius instead.
        let r = r.min(self.cutters[idx].radius());
        self.cutters[idx].height(r) + self.zoffset[idx]
    }

    fn width(&self, h: f64) -> f64 {
        let idx = self.height_to_index(h);
        self.cutters[idx].width(h - self.zoffset[idx])
    }

    // The composite overrides facet_drop/facet_push, so these are never used.
    fn xy_normal_length(&self) -> f64 {
        0.0
    }
    fn normal_length(&self) -> f64 {
        0.0
    }
    fn center_height(&self) -> f64 {
        0.0
    }

    /// Not implemented for composite cutters (as in the C++ library).
    fn offset_cutter(&self, _d: f64) -> Option<Box<dyn MillingCutter>> {
        None
    }

    // The composite overrides edge_drop, so this is never used.
    fn single_edge_drop_canonical(&self, _u1: &Point, _u2: &Point) -> (f64, f64) {
        (0.0, 0.0)
    }

    /// Unlike a single cutter, a facet contact of one sub-cutter does not rule out a
    /// higher edge or vertex contact of another, so all three tests always run. (The C++
    /// library skips the edge and vertex tests after a facet contact.)
    fn drop_cutter(&self, cl: &mut CLPoint, t: &Triangle) -> bool {
        if !cl.below(t) {
            return false;
        }
        let facet = self.facet_drop(cl, t);
        let vertex = self.vertex_drop(cl, t);
        let edge = cl.below(t) && self.edge_drop(cl, t);
        facet || vertex || edge
    }

    fn facet_drop(&self, cl: &mut CLPoint, t: &Triangle) -> bool {
        let sub = self.composite_drop(cl, t, CCType::Facet, |c, cl, t| c.facet_drop(cl, t));
        let boundary = self.boundary_facet_drop(cl, t);
        sub || boundary
    }

    fn edge_drop(&self, cl: &mut CLPoint, t: &Triangle) -> bool {
        let sub = self.composite_drop(cl, t, CCType::Edge, |c, cl, t| c.edge_drop(cl, t));
        let boundary = self.boundary_edge_drop(cl, t);
        sub || boundary
    }

    fn vertex_push(&self, f: &Fiber, i: &mut Interval, t: &Triangle) -> bool {
        self.composite_push(f, i, t, |c, f, i, t| c.vertex_push(f, i, t))
    }

    fn facet_push(&self, f: &Fiber, i: &mut Interval, t: &Triangle) -> bool {
        self.composite_push(f, i, t, |c, f, i, t| c.facet_push(f, i, t))
    }

    fn edge_push(&self, f: &Fiber, i: &mut Interval, t: &Triangle) -> bool {
        self.composite_push(f, i, t, |c, f, i, t| c.edge_push(f, i, t))
    }
}

impl fmt::Display for CompositeCutter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "CompositeCutter with {} cutters:", self.cutters.len())?;
        for (n, c) in self.cutters.iter().enumerate() {
            writeln!(f, " {}:{}", n, c)?;
            writeln!(f, "  radius[{}]={}", n, self.radiusvec[n])?;
            writeln!(f, "  height[{}]={}", n, self.heightvec[n])?;
            writeln!(f, "  zoffset[{}]={}", n, self.zoffset[n])?;
        }
        Ok(())
    }
}

impl fmt::Debug for CompositeCutter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(self, f)
    }
}
