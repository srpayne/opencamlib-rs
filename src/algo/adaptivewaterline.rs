use super::fiber::Fiber;
use super::fiberpushcutter::{FiberDirection, FiberPushCutter};
use super::waterline::weave_loops;
use crate::common::parallel;
use crate::cutters::MillingCutter;
use crate::geo::{Line, Point, STLSurf, Span};

/// Waterline with adaptive fiber spacing: fibers are added between neighbours until the
/// spacing is below `sampling` and the CL-points of consecutive fibers are collinear
/// (cosine of the angle above `cos_limit`), or the spacing reaches `min_sampling`.
pub struct AdaptiveWaterline<'a> {
    surf: &'a STLSurf,
    cutter: &'a dyn MillingCutter,
    xpc: FiberPushCutter<'a>,
    ypc: FiberPushCutter<'a>,
    zh: f64,
    sampling: f64,
    min_sampling: f64,
    cos_limit: f64,
    threads: usize,
    loops: Vec<Vec<Point>>,
    xfibers: Vec<Fiber>,
    yfibers: Vec<Fiber>,
}

/// Parameters shared by the recursive sampling of one fiber direction.
struct Sampler<'s, 'a> {
    pc: &'s FiberPushCutter<'a>,
    span: Span,
    dir: FiberDirection,
    lo: f64,
    hi: f64,
    zh: f64,
    sampling: f64,
    min_sampling: f64,
    cos_limit: f64,
}

impl<'a> AdaptiveWaterline<'a> {
    pub fn new(surf: &'a STLSurf, cutter: &'a dyn MillingCutter) -> Self {
        AdaptiveWaterline {
            surf,
            cutter,
            xpc: FiberPushCutter::new(surf, cutter, FiberDirection::X),
            ypc: FiberPushCutter::new(surf, cutter, FiberDirection::Y),
            zh: 0.0,
            sampling: 1.0,
            min_sampling: 0.1,
            cos_limit: 0.999,
            threads: parallel::default_threads(),
            loops: Vec::new(),
            xfibers: Vec::new(),
            yfibers: Vec::new(),
        }
    }

    pub fn set_stl(&mut self, surf: &'a STLSurf) {
        self.surf = surf;
        self.xpc.set_stl(surf);
        self.ypc.set_stl(surf);
    }

    pub fn set_cutter(&mut self, cutter: &'a dyn MillingCutter) {
        self.cutter = cutter;
        self.xpc.set_cutter(cutter);
        self.ypc.set_cutter(cutter);
    }

    pub fn set_z(&mut self, z: f64) {
        self.zh = z;
    }

    pub fn get_z(&self) -> f64 {
        self.zh
    }

    /// Maximum fiber spacing. Also sets `min_sampling` to `s / 10`.
    pub fn set_sampling(&mut self, s: f64) {
        self.sampling = s;
        self.min_sampling = s / 10.0;
    }

    pub fn get_sampling(&self) -> f64 {
        self.sampling
    }

    /// Minimum fiber spacing.
    pub fn set_min_sampling(&mut self, s: f64) {
        self.min_sampling = s;
    }

    /// Collinearity limit (cosine of the angle between consecutive CL-point steps).
    pub fn set_cos_limit(&mut self, lim: f64) {
        self.cos_limit = lim;
    }

    pub fn set_threads(&mut self, n: usize) {
        self.threads = n.max(1);
    }

    pub fn get_threads(&self) -> usize {
        self.threads
    }

    pub fn set_bucket_size(&mut self, s: usize) {
        self.xpc.set_bucket_size(s);
        self.ypc.set_bucket_size(s);
    }

    /// Compute the waterline using the simple weave.
    pub fn run(&mut self) {
        self.adaptive_sampling_run();
        self.loops = weave_loops(&self.xfibers, &self.yfibers, false);
    }

    /// Compute the waterline using the smart weave.
    pub fn run2(&mut self) {
        self.adaptive_sampling_run();
        self.loops = weave_loops(&self.xfibers, &self.yfibers, true);
    }

    pub fn reset(&mut self) {
        self.xfibers.clear();
        self.yfibers.clear();
        self.loops.clear();
    }

    pub fn get_loops(&self) -> &[Vec<Point>] {
        &self.loops
    }

    pub fn get_x_fibers(&self) -> &[Fiber] {
        &self.xfibers
    }

    pub fn get_y_fibers(&self) -> &[Fiber] {
        &self.yfibers
    }

    fn adaptive_sampling_run(&mut self) {
        let r2 = 2.0 * self.cutter.radius();
        let bb = &self.surf.bb;
        let (minx, maxx) = (bb.minpt.x - r2, bb.maxpt.x + r2);
        let (miny, maxy) = (bb.minpt.y - r2, bb.maxpt.y + r2);
        let span = Span::Line(Line::new(
            Point::new(minx, miny, self.zh),
            Point::new(maxx, maxy, self.zh),
        ));
        let sampler = |pc, dir, lo, hi| Sampler {
            pc,
            span,
            dir,
            lo,
            hi,
            zh: self.zh,
            sampling: self.sampling,
            min_sampling: self.min_sampling,
            cos_limit: self.cos_limit,
        };
        let xs = sampler(&self.xpc, FiberDirection::X, minx, maxx);
        let ys = sampler(&self.ypc, FiberDirection::Y, miny, maxy);
        let (xfibers, yfibers) = parallel::join(self.threads, || xs.run(), || ys.run());
        self.xfibers = xfibers;
        self.yfibers = yfibers;
    }
}

impl Sampler<'_, '_> {
    /// The fiber at parameter `t` along the diagonal span.
    fn fiber_at(&self, t: f64) -> Fiber {
        let p = self.span.get_point(t);
        let mut f = match self.dir {
            FiberDirection::X => Fiber::new(
                Point::new(self.lo, p.y, self.zh),
                Point::new(self.hi, p.y, self.zh),
            ),
            FiberDirection::Y => Fiber::new(
                Point::new(p.x, self.lo, self.zh),
                Point::new(p.x, self.hi, self.zh),
            ),
        };
        self.pc.run(&mut f);
        f
    }

    fn run(&self) -> Vec<Fiber> {
        let start = self.fiber_at(0.0);
        let stop = self.fiber_at(1.0);
        let mut out = vec![start.clone()];
        self.sample(0.0, 1.0, &start, &stop, &mut out);
        out
    }

    fn sample(&self, start_t: f64, stop_t: f64, start_f: &Fiber, stop_f: &Fiber, out: &mut Vec<Fiber>) {
        let mid_t = start_t + (stop_t - start_t) / 2.0;
        let mid_f = self.fiber_at(mid_t);
        let fw_step = match self.dir {
            FiberDirection::X => (start_f.p1.y - stop_f.p1.y).abs(),
            FiberDirection::Y => (start_f.p1.x - stop_f.p1.x).abs(),
        };
        if fw_step > self.sampling {
            // above the maximum step: sample more
            self.sample(start_t, mid_t, start_f, &mid_f, out);
            self.sample(mid_t, stop_t, &mid_f, stop_f, out);
        } else if !self.flat_fibers(start_f, &mid_f, stop_f) {
            // Not flat: refine until min_sampling. As in the C++ library, a non-flat
            // segment at min_sampling adds no fiber.
            if fw_step > self.min_sampling {
                self.sample(start_t, mid_t, start_f, &mid_f, out);
                self.sample(mid_t, stop_t, &mid_f, stop_f, out);
            }
        } else {
            out.push(stop_f.clone());
        }
    }

    fn flat_fibers(&self, start: &Fiber, mid: &Fiber, stop: &Fiber) -> bool {
        if start.size() != stop.size() || start.size() != mid.size() {
            return false;
        }
        (0..start.size()).all(|n| {
            self.flat(start.upper_cl_point(n), mid.upper_cl_point(n), stop.upper_cl_point(n))
                && self.flat(start.lower_cl_point(n), mid.lower_cl_point(n), stop.lower_cl_point(n))
        })
    }

    fn flat(&self, start_cl: Point, mid_cl: Point, stop_cl: Point) -> bool {
        let mut v1 = mid_cl - start_cl;
        let mut v2 = stop_cl - mid_cl;
        v1.normalize();
        v2.normalize();
        v1.dot(&v2) > self.cos_limit
    }
}
