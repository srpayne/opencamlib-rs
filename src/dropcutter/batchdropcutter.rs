use std::sync::atomic::{AtomicUsize, Ordering};

use super::pointdropcutter::drop_cl_point;
use crate::common::kdtree::KDTree;
use crate::common::parallel;
use crate::cutters::MillingCutter;
use crate::geo::{CLPoint, STLSurf, Triangle};

/// Drop-cutter for many CL-points, processed in parallel.
pub struct BatchDropCutter<'a> {
    surf: &'a STLSurf,
    cutter: &'a dyn MillingCutter,
    clpoints: Vec<CLPoint>,
    root: KDTree,
    bucket_size: usize,
    threads: usize,
    n_calls: usize,
}

impl<'a> BatchDropCutter<'a> {
    pub fn new(surf: &'a STLSurf, cutter: &'a dyn MillingCutter) -> Self {
        let mut bdc = BatchDropCutter {
            surf,
            cutter,
            clpoints: Vec::new(),
            root: KDTree::new(),
            bucket_size: 1,
            threads: parallel::default_threads(),
            n_calls: 0,
        };
        bdc.build_tree();
        bdc
    }

    fn build_tree(&mut self) {
        self.root.set_xy_dimensions();
        self.root.set_bucket_size(self.bucket_size);
        self.root.build(&self.surf.tris);
    }

    pub fn set_stl(&mut self, surf: &'a STLSurf) {
        self.surf = surf;
        self.build_tree();
    }

    pub fn set_cutter(&mut self, cutter: &'a dyn MillingCutter) {
        self.cutter = cutter;
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

    /// Number of drop-cutter calls made by the last [`run`](Self::run).
    pub fn get_calls(&self) -> usize {
        self.n_calls
    }

    /// Add a CL-point. Its z-coordinate is the lowest z the cutter can reach.
    pub fn append_point(&mut self, p: CLPoint) {
        self.clpoints.push(p);
    }

    pub fn get_cl_points(&self) -> &[CLPoint] {
        &self.clpoints
    }

    pub fn take_cl_points(&mut self) -> Vec<CLPoint> {
        std::mem::take(&mut self.clpoints)
    }

    pub fn clear_cl_points(&mut self) {
        self.clpoints.clear();
    }

    /// Drop the cutter at every CL-point.
    pub fn run(&mut self) {
        let calls = AtomicUsize::new(0);
        let (root, surf, cutter) = (&self.root, self.surf, self.cutter);
        parallel::for_each_mut(self.threads, &mut self.clpoints, |cl| {
            let n = drop_cl_point(root, surf, cutter, cl);
            calls.fetch_add(n, Ordering::Relaxed);
        });
        self.n_calls = calls.into_inner();
    }

    /// Triangles that may lie under `cutter` positioned at `cl`.
    pub fn get_triangles_under_cutter(&self, cl: &CLPoint, cutter: &dyn MillingCutter) -> Vec<Triangle> {
        self.root
            .search_cutter_overlap(cutter, cl)
            .into_iter()
            .map(|i| self.surf.tris[i])
            .collect()
    }
}
