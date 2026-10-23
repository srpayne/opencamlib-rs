use std::fmt;

use crate::geo::{Bbox, Point};

/// Experimental zigzag pocketing: generates points on lines parallel to `dir`, spaced
/// `step_over` apart, covering the bounding box of the pocket points.
#[derive(Debug, Clone, Default)]
pub struct ZigZag {
    step_over: f64,
    dir: Point,
    origin: Point,
    pocket: Vec<Point>,
    out: Vec<Point>,
    bb: Bbox,
}

impl ZigZag {
    pub fn new() -> Self {
        ZigZag::default()
    }

    pub fn set_step_over(&mut self, d: f64) {
        self.step_over = d;
    }

    pub fn set_direction(&mut self, d: Point) {
        self.dir = d;
    }

    pub fn set_origin(&mut self, d: Point) {
        self.origin = d;
    }

    pub fn add_point(&mut self, p: Point) {
        self.bb.add_point(&p);
        self.pocket.push(p);
    }

    pub fn run(&mut self) {
        if self.step_over <= 0.0 {
            return;
        }
        let mut perp = self.dir.xy_perp();
        perp.xy_normalize();
        let mut max_d = (self.bb.maxpt - self.origin).dot(&perp);
        let mut min_d = (self.bb.minpt - self.origin).dot(&perp);
        if max_d < min_d {
            std::mem::swap(&mut max_d, &mut min_d);
        }
        let mut d = min_d;
        while d <= max_d {
            self.out.push(self.origin + d * perp);
            d += self.step_over;
        }
    }

    pub fn get_output(&self) -> &[Point] {
        &self.out
    }
}

impl fmt::Display for ZigZag {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "ZigZag: pocket.size()={}", self.pocket.len())
    }
}
