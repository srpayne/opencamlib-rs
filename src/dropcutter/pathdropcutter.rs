use super::batchdropcutter::BatchDropCutter;
use crate::cutters::MillingCutter;
use crate::geo::{CLPoint, Path, STLSurf, Span};

/// Drop-cutter along a [`Path`], sampled uniformly every `sampling` units.
pub struct PathDropCutter<'a> {
    bdc: BatchDropCutter<'a>,
    path: Option<&'a Path>,
    minimum_z: f64,
    sampling: f64,
    clpoints: Vec<CLPoint>,
}

impl<'a> PathDropCutter<'a> {
    pub fn new(surf: &'a STLSurf, cutter: &'a dyn MillingCutter) -> Self {
        PathDropCutter {
            bdc: BatchDropCutter::new(surf, cutter),
            path: None,
            minimum_z: 0.0,
            sampling: 0.1,
            clpoints: Vec::new(),
        }
    }

    pub fn set_stl(&mut self, surf: &'a STLSurf) {
        self.bdc.set_stl(surf);
    }

    pub fn set_cutter(&mut self, cutter: &'a dyn MillingCutter) {
        self.bdc.set_cutter(cutter);
    }

    pub fn set_path(&mut self, path: &'a Path) {
        self.path = Some(path);
        self.bdc.clear_cl_points();
    }

    /// The lowest z the cutter can reach.
    pub fn set_z(&mut self, z: f64) {
        self.minimum_z = z;
    }

    pub fn get_z(&self) -> f64 {
        self.minimum_z
    }

    /// Distance between samples along the path.
    pub fn set_sampling(&mut self, s: f64) {
        assert!(s > 0.0, "sampling must be positive");
        self.sampling = s;
    }

    pub fn get_sampling(&self) -> f64 {
        self.sampling
    }

    pub fn set_threads(&mut self, n: usize) {
        self.bdc.set_threads(n);
    }

    pub fn get_threads(&self) -> usize {
        self.bdc.get_threads()
    }

    pub fn get_calls(&self) -> usize {
        self.bdc.get_calls()
    }

    pub fn run(&mut self) {
        self.clpoints.clear();
        // Unlike the C++ library, points from a previous run are not kept.
        self.bdc.clear_cl_points();
        if let Some(path) = self.path {
            for span in &path.span_list {
                self.sample_span(span);
            }
        }
        self.bdc.run();
        self.clpoints = self.bdc.take_cl_points();
    }

    fn sample_span(&mut self, span: &Span) {
        let num_steps = (span.length2d() / self.sampling + 1.0) as u32;
        for i in 0..=num_steps {
            let fraction = i as f64 / num_steps as f64;
            let p = span.get_point(fraction);
            self.bdc.append_point(CLPoint::new(p.x, p.y, self.minimum_z));
        }
    }

    pub fn get_cl_points(&self) -> &[CLPoint] {
        &self.clpoints
    }

    /// Same as [`get_cl_points`](Self::get_cl_points).
    pub fn get_points(&self) -> &[CLPoint] {
        &self.clpoints
    }
}
