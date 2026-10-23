use super::batchpushcutter::BatchPushCutter;
use super::fiber::Fiber;
use super::fiberpushcutter::FiberDirection;
use super::weave::Weave;
use crate::common::parallel;
use crate::cutters::MillingCutter;
use crate::geo::{Point, STLSurf};

/// Waterline toolpath: closed loops at constant z where the cutter touches the model.
///
/// X- and Y-fibers on a uniform grid (spacing set by [`set_sampling`](Self::set_sampling))
/// are pushed against the surface, and the resulting intervals are woven into loops.
///
/// ```no_run
/// # use opencamlib::{STLSurf, BallCutter, Waterline};
/// let surf = STLSurf::from_file("model.stl").unwrap();
/// let cutter = BallCutter::new(2.0, 10.0);
/// let mut wl = Waterline::new(&surf, &cutter);
/// wl.set_z(1.5);
/// wl.set_sampling(0.1);
/// wl.run();
/// for lp in wl.get_loops() {
///     println!("loop with {} points", lp.len());
/// }
/// ```
pub struct Waterline<'a> {
    surf: &'a STLSurf,
    cutter: &'a dyn MillingCutter,
    xpc: BatchPushCutter<'a>,
    ypc: BatchPushCutter<'a>,
    zh: f64,
    sampling: f64,
    threads: usize,
    loops: Vec<Vec<Point>>,
    xfibers: Vec<Fiber>,
    yfibers: Vec<Fiber>,
}

impl<'a> Waterline<'a> {
    pub fn new(surf: &'a STLSurf, cutter: &'a dyn MillingCutter) -> Self {
        let threads = parallel::default_threads();
        Waterline {
            surf,
            cutter,
            xpc: BatchPushCutter::new(surf, cutter, FiberDirection::X),
            ypc: BatchPushCutter::new(surf, cutter, FiberDirection::Y),
            zh: 0.0,
            sampling: 1.0,
            threads,
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

    /// The z-height of the waterline.
    pub fn set_z(&mut self, z: f64) {
        self.zh = z;
    }

    pub fn get_z(&self) -> f64 {
        self.zh
    }

    /// Fiber spacing.
    pub fn set_sampling(&mut self, s: f64) {
        self.sampling = s;
    }

    pub fn get_sampling(&self) -> f64 {
        self.sampling
    }

    pub fn set_threads(&mut self, n: usize) {
        self.threads = n.max(1);
        self.xpc.set_threads(self.threads);
        self.ypc.set_threads(self.threads);
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
        self.push_fibers();
        self.loops = weave_loops(&self.xfibers, &self.yfibers, false);
    }

    /// Compute the waterline using the smart weave.
    pub fn run2(&mut self) {
        self.push_fibers();
        self.loops = weave_loops(&self.xfibers, &self.yfibers, true);
    }

    fn push_fibers(&mut self) {
        // Unlike the C++ library, fibers from a previous run are discarded, so that the
        // same Waterline can be re-run at several z-heights.
        self.reset();
        self.init_fibers();
        self.xpc.run();
        self.ypc.run();
        self.xfibers = self.xpc.get_fibers().to_vec();
        self.yfibers = self.ypc.get_fibers().to_vec();
    }

    /// Clear fibers and loops.
    pub fn reset(&mut self) {
        self.xfibers.clear();
        self.yfibers.clear();
        self.loops.clear();
        self.xpc.reset();
        self.ypc.reset();
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

    fn init_fibers(&mut self) {
        let r2 = 2.0 * self.cutter.radius();
        let bb = &self.surf.bb;
        let (minx, maxx) = (bb.minpt.x - r2, bb.maxpt.x + r2);
        let (miny, maxy) = (bb.minpt.y - r2, bb.maxpt.y + r2);
        let nx = ((maxx - minx) / self.sampling) as i64;
        let ny = ((maxy - miny) / self.sampling) as i64;
        for y in generate_range(miny, maxy, ny) {
            self.xpc.append_fiber(Fiber::new(
                Point::new(minx, y, self.zh),
                Point::new(maxx, y, self.zh),
            ));
        }
        for x in generate_range(minx, maxx, nx) {
            self.ypc.append_fiber(Fiber::new(
                Point::new(x, miny, self.zh),
                Point::new(x, maxy, self.zh),
            ));
        }
    }
}

/// `n + 1` evenly spaced values from `start` to `end` (accumulated by repeated addition,
/// as in the C++ library).
fn generate_range(start: f64, end: f64, n: i64) -> Vec<f64> {
    let d = (end - start) / n as f64;
    let mut v = start;
    let mut out = Vec::with_capacity((n.max(0) + 1) as usize);
    for _ in 0..=n {
        out.push(v);
        v += d;
    }
    out
}

/// Weave the fibers and return the loops.
pub(crate) fn weave_loops(xfibers: &[Fiber], yfibers: &[Fiber], smart: bool) -> Vec<Vec<Point>> {
    let mut weave = Weave::new();
    for f in xfibers.iter().chain(yfibers) {
        weave.add_fiber(f.clone());
    }
    if smart {
        weave.build_smart();
    } else {
        weave.build();
    }
    weave.face_traverse();
    weave.get_loops()
}
