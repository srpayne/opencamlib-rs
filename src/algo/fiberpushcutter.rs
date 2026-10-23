use std::sync::atomic::{AtomicUsize, Ordering};

use super::fiber::Fiber;
use super::interval::Interval;
use crate::common::kdtree::KDTree;
use crate::cutters::MillingCutter;
use crate::geo::{Point, STLSurf};

/// Direction of the fibers a push-cutter works on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FiberDirection {
    /// Fibers parallel to the x-axis.
    X,
    /// Fibers parallel to the y-axis.
    Y,
}

impl FiberDirection {
    /// A point on the fiber's line, positioned for a kd-tree overlap search.
    pub(crate) fn search_point(self, f: &Fiber) -> Point {
        match self {
            FiberDirection::X => Point::new(0.0, f.p1.y, f.p1.z),
            FiberDirection::Y => Point::new(f.p1.x, 0.0, f.p1.z),
        }
    }

    pub(crate) fn configure(self, kd: &mut KDTree) {
        match self {
            FiberDirection::X => kd.set_yz_dimensions(),
            FiberDirection::Y => kd.set_xz_dimensions(),
        }
    }
}

/// Push-cutter for one fiber at a time (used by [`AdaptiveWaterline`]).
///
/// [`AdaptiveWaterline`]: crate::AdaptiveWaterline
pub struct FiberPushCutter<'a> {
    surf: &'a STLSurf,
    cutter: &'a dyn MillingCutter,
    direction: FiberDirection,
    root: KDTree,
    bucket_size: usize,
    n_calls: AtomicUsize,
}

impl<'a> FiberPushCutter<'a> {
    pub fn new(surf: &'a STLSurf, cutter: &'a dyn MillingCutter, direction: FiberDirection) -> Self {
        let mut pc = FiberPushCutter {
            surf,
            cutter,
            direction,
            root: KDTree::new(),
            bucket_size: 1,
            n_calls: AtomicUsize::new(0),
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

    pub fn direction(&self) -> FiberDirection {
        self.direction
    }

    pub fn set_bucket_size(&mut self, s: usize) {
        self.bucket_size = s;
        self.build_tree();
    }

    pub fn get_bucket_size(&self) -> usize {
        self.bucket_size
    }

    /// Number of push-cutter calls made so far.
    pub fn get_calls(&self) -> usize {
        self.n_calls.load(Ordering::Relaxed)
    }

    /// Push the cutter along `f`, adding the contact intervals to it.
    pub fn run(&self, f: &mut Fiber) {
        let cl = self.direction.search_point(f);
        let tris = self.root.search_cutter_overlap(self.cutter, &cl);
        for idx in &tris {
            let mut i = Interval::new();
            self.cutter.push_cutter(f, &mut i, &self.surf.tris[*idx]);
            f.add_interval(i);
        }
        self.n_calls.fetch_add(tris.len(), Ordering::Relaxed);
    }
}
