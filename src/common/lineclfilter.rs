//! Filter that removes CL-points lying (within a tolerance) on a straight line between
//! their neighbours.

use crate::geo::CLPoint;

/// Removes CL-points that are collinear (within `tol`) with their neighbours.
#[derive(Debug, Clone, Default)]
pub struct LineCLFilter {
    pub clpoints: Vec<CLPoint>,
    pub tol: f64,
}

impl LineCLFilter {
    pub fn new() -> Self {
        LineCLFilter::default()
    }

    pub fn add_cl_point(&mut self, p: CLPoint) {
        self.clpoints.push(p);
    }

    pub fn set_tolerance(&mut self, tol: f64) {
        self.tol = tol;
    }

    /// Run the filter, replacing `clpoints` with the filtered list.
    pub fn run(&mut self) {
        let pts = &self.clpoints;
        let n = pts.len();
        if n < 2 {
            return;
        }
        let mut new_list = vec![pts[0]];
        let mut p0 = 0;
        let mut p1 = 1;
        let mut p2 = 2;
        let mut p_last_good = 1;
        let mut even_number = true;
        while p2 < n {
            let p = pts[p1].closest_point(&pts[p0], &pts[p2]);
            if (p - pts[p1].pos).norm() < self.tol {
                p_last_good = p2;
                p2 += 1;
                if even_number {
                    p1 += 1;
                }
                even_number = !even_number;
            } else {
                new_list.push(pts[p_last_good]);
                p0 = p_last_good;
                p1 = p2;
                p_last_good = p1;
                p2 += 1;
            }
        }
        new_list.push(pts[n - 1]);
        self.clpoints = new_list;
    }

    pub fn get_cl_points(&self) -> &[CLPoint] {
        &self.clpoints
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn removes_collinear() {
        let mut f = LineCLFilter::new();
        for i in 0..10 {
            f.add_cl_point(CLPoint::new(i as f64, 0.0, 0.0));
        }
        f.add_cl_point(CLPoint::new(9.0, 5.0, 0.0));
        f.set_tolerance(0.01);
        f.run();
        let pts = f.get_cl_points();
        assert_eq!(pts.len(), 3);
        assert_eq!(pts[1].x, 9.0);
    }
}
