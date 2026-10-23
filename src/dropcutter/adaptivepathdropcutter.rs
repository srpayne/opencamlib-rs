use super::pointdropcutter::PointDropCutter;
use crate::cutters::MillingCutter;
use crate::geo::{CLPoint, Path, STLSurf, Span};

/// Drop-cutter along a [`Path`] with adaptive sampling: points are added until the
/// spacing is below `sampling` and consecutive steps are collinear (cosine of the angle
/// above `cos_limit`), or the spacing reaches `min_sampling`.
///
/// As in the C++ library, each CL-point starts at the z-coordinate of the path, so the
/// path's z is the lowest z the cutter can reach; [`set_z`](Self::set_z) is stored but not
/// used.
pub struct AdaptivePathDropCutter<'a> {
    pdc: PointDropCutter<'a>,
    path: Option<&'a Path>,
    minimum_z: f64,
    sampling: f64,
    min_sampling: f64,
    cos_limit: f64,
    clpoints: Vec<CLPoint>,
}

impl<'a> AdaptivePathDropCutter<'a> {
    pub fn new(surf: &'a STLSurf, cutter: &'a dyn MillingCutter) -> Self {
        AdaptivePathDropCutter {
            pdc: PointDropCutter::new(surf, cutter),
            path: None,
            minimum_z: 0.0,
            sampling: 0.1,
            min_sampling: 0.01,
            cos_limit: 0.999,
            clpoints: Vec::new(),
        }
    }

    pub fn set_stl(&mut self, surf: &'a STLSurf) {
        self.pdc.set_stl(surf);
    }

    pub fn set_cutter(&mut self, cutter: &'a dyn MillingCutter) {
        self.pdc.set_cutter(cutter);
    }

    pub fn set_path(&mut self, path: &'a Path) {
        self.path = Some(path);
    }

    pub fn set_z(&mut self, z: f64) {
        self.minimum_z = z;
    }

    pub fn get_z(&self) -> f64 {
        self.minimum_z
    }

    /// Maximum distance between samples.
    pub fn set_sampling(&mut self, s: f64) {
        self.sampling = s;
    }

    pub fn get_sampling(&self) -> f64 {
        self.sampling
    }

    /// Minimum distance between samples.
    pub fn set_min_sampling(&mut self, s: f64) {
        assert!(s > 0.0, "min_sampling must be positive");
        self.min_sampling = s;
    }

    pub fn set_cos_limit(&mut self, lim: f64) {
        self.cos_limit = lim;
    }

    pub fn run(&mut self) {
        self.clpoints.clear();
        let Some(path) = self.path else {
            return;
        };
        for span in &path.span_list {
            let mut start = CLPoint::from(span.get_point(0.0));
            let mut stop = CLPoint::from(span.get_point(1.0));
            self.pdc.run(&mut start);
            self.pdc.run(&mut stop);
            self.clpoints.push(start);
            self.adaptive_sample(span, 0.0, 1.0, start, stop);
        }
    }

    fn adaptive_sample(&mut self, span: &Span, start_t: f64, stop_t: f64, start_cl: CLPoint, stop_cl: CLPoint) {
        let mid_t = start_t + (stop_t - start_t) / 2.0;
        let mut mid_cl = CLPoint::from(span.get_point(mid_t));
        self.pdc.run(&mut mid_cl);
        let fw_step = (stop_cl.pos - start_cl.pos).xy_norm();
        if fw_step > self.sampling
            || (!self.flat(&start_cl, &mid_cl, &stop_cl) && fw_step > self.min_sampling)
        {
            self.adaptive_sample(span, start_t, mid_t, start_cl, mid_cl);
            self.adaptive_sample(span, mid_t, stop_t, mid_cl, stop_cl);
        } else {
            self.clpoints.push(stop_cl);
        }
    }

    fn flat(&self, start_cl: &CLPoint, mid_cl: &CLPoint, stop_cl: &CLPoint) -> bool {
        let mut v1 = mid_cl.pos - start_cl.pos;
        let mut v2 = stop_cl.pos - mid_cl.pos;
        v1.normalize();
        v2.normalize();
        v1.dot(&v2) > self.cos_limit
    }

    pub fn get_cl_points(&self) -> &[CLPoint] {
        &self.clpoints
    }

    /// Same as [`get_cl_points`](Self::get_cl_points).
    pub fn get_points(&self) -> &[CLPoint] {
        &self.clpoints
    }
}
