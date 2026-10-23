use std::sync::atomic::{AtomicUsize, Ordering};

use super::fiber::Fiber;
use super::fiberpushcutter::FiberDirection;
use super::interval::Interval;
use crate::common::kdtree::KDTree;
use crate::common::parallel;
use crate::cutters::MillingCutter;
use crate::geo::{CLPoint, STLSurf, Triangle};

/// Push-cutter for many fibers, processed in parallel.
pub struct BatchPushCutter<'a> {
    surf: &'a STLSurf,
    cutter: &'a dyn MillingCutter,
    direction: FiberDirection,
    fibers: Vec<Fiber>,
    root: KDTree,
    bucket_size: usize,
    threads: usize,
    n_calls: usize,
}

impl<'a> BatchPushCutter<'a> {
    pub fn new(surf: &'a STLSurf, cutter: &'a dyn MillingCutter, direction: FiberDirection) -> Self {
        let mut pc = BatchPushCutter {
            surf,
            cutter,
            direction,
            fibers: Vec::new(),
            root: KDTree::new(),
            bucket_size: 1,
            threads: parallel::default_threads(),
            n_calls: 0,
        };
        pc.build_tree();
        pc
    }

    fn build_tree(&mut self) {
        self.root.set_bucket_size(self.bucket_size);
        self.direction.configure(&mut self.root);
        self.root.build(&self.surf.tris);
    }

    pub fn set_stl(&mut self, surf: &'a STLSurf) {
        self.surf = surf;
        self.build_tree();
    }

    pub fn set_cutter(&mut self, cutter: &'a dyn MillingCutter) {
        self.cutter = cutter;
    }

    pub fn set_direction(&mut self, direction: FiberDirection) {
        self.direction = direction;
        self.build_tree();
    }

    pub fn set_bucket_size(&mut self, s: usize) {
        self.bucket_size = s;
        self.build_tree();
    }

    pub fn get_bucket_size(&self) -> usize {
        self.bucket_size
    }

    pub fn set_threads(&mut self, n: usize) {
        self.threads = n.max(1);
    }

    pub fn get_threads(&self) -> usize {
        self.threads
    }

    /// Number of push-cutter calls made by the last [`run`](Self::run).
    pub fn get_calls(&self) -> usize {
        self.n_calls
    }

    pub fn append_fiber(&mut self, f: Fiber) {
        self.fibers.push(f);
    }

    /// Remove all fibers.
    pub fn reset(&mut self) {
        self.fibers.clear();
    }

    pub fn get_fibers(&self) -> &[Fiber] {
        &self.fibers
    }

    pub fn take_fibers(&mut self) -> Vec<Fiber> {
        std::mem::take(&mut self.fibers)
    }

    /// Push the cutter along every fiber.
    pub fn run(&mut self) {
        let calls = AtomicUsize::new(0);
        let (root, surf, cutter, dir) = (&self.root, self.surf, self.cutter, self.direction);
        parallel::for_each_mut(self.threads, &mut self.fibers, |f| {
            let cl = dir.search_point(f);
            let tris = root.search_cutter_overlap(cutter, &cl);
            for idx in &tris {
                let mut i = Interval::new();
                cutter.push_cutter(f, &mut i, &surf.tris[*idx]);
                f.add_interval(i);
            }
            calls.fetch_add(tris.len(), Ordering::Relaxed);
        });
        self.n_calls = calls.into_inner();
    }

    /// CL-points at both ends of every interval of every fiber.
    pub fn get_cl_points(&self) -> Vec<CLPoint> {
        let mut out = Vec::with_capacity(2 * self.fibers.len());
        for f in &self.fibers {
            for i in &f.ints {
                if i.empty() {
                    continue;
                }
                out.push(CLPoint { pos: f.point(i.lower), cc: i.lower_cc });
                out.push(CLPoint { pos: f.point(i.upper), cc: i.upper_cc });
            }
        }
        out
    }

    /// Triangles that may overlap the cutter as it moves along `f`.
    pub fn get_overlap_triangles(&self, f: &Fiber) -> Vec<Triangle> {
        let cl = self.direction.search_point(f);
        self.root
            .search_cutter_overlap(self.cutter, &cl)
            .into_iter()
            .map(|i| self.surf.tris[i])
            .collect()
    }
}
