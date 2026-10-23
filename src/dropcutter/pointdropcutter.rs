use std::sync::atomic::{AtomicUsize, Ordering};

use crate::common::kdtree::KDTree;
use crate::cutters::MillingCutter;
use crate::geo::{CLPoint, STLSurf};

/// Drop-cutter for one CL-point at a time.
pub struct PointDropCutter<'a> {
    surf: &'a STLSurf,
    cutter: &'a dyn MillingCutter,
    root: KDTree,
    bucket_size: usize,
    n_calls: AtomicUsize,
}

impl<'a> PointDropCutter<'a> {
    pub fn new(surf: &'a STLSurf, cutter: &'a dyn MillingCutter) -> Self {
        let mut pdc = PointDropCutter {
            surf,
            cutter,
            root: KDTree::new(),
            bucket_size: 1,
            n_calls: AtomicUsize::new(0),
        };
        pdc.build_tree();
        pdc
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

    /// Number of drop-cutter calls made by the last [`run`](Self::run).
    pub fn get_calls(&self) -> usize {
        self.n_calls.load(Ordering::Relaxed)
    }

    /// Drop the cutter at (cl.x, cl.y) onto the surface, lifting `cl` from its current z.
    pub fn run(&self, cl: &mut CLPoint) {
        let n = drop_cl_point(&self.root, self.surf, self.cutter, cl);
        self.n_calls.store(n, Ordering::Relaxed);
    }
}

/// Drop `cl` against the triangles found in `root`. Returns the number of drop calls.
pub(crate) fn drop_cl_point(root: &KDTree, surf: &STLSurf, cutter: &dyn MillingCutter, cl: &mut CLPoint) -> usize {
    let mut calls = 0;
    for idx in root.search_cutter_overlap(cutter, cl) {
        let t = &surf.tris[idx];
        if cutter.overlaps(cl, t) && cl.below(t) {
            cutter.drop_cutter(cl, t);
            calls += 1;
        }
    }
    calls
}
