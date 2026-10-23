use std::fmt;

use super::{single_vertex_push, BullCutter, MillingCutter};
use crate::algo::fiber::Fiber;
use crate::algo::interval::Interval;
use crate::common::numeric::square;
use crate::geo::{CCType, Point, Triangle};

/// Cylindrical (flat end) cutter.
#[derive(Debug, Clone, PartialEq)]
pub struct CylCutter {
    diameter: f64,
    radius: f64,
    length: f64,
}

impl CylCutter {
    /// A flat end mill with diameter `d` and length `l`.
    pub fn new(d: f64, l: f64) -> Self {
        assert!(d > 0.0, "CylCutter diameter must be positive");
        assert!(l > 0.0, "CylCutter length must be positive");
        CylCutter {
            diameter: d,
            radius: d / 2.0,
            length: l,
        }
    }
}

impl MillingCutter for CylCutter {
    fn diameter(&self) -> f64 {
        self.diameter
    }
    fn radius(&self) -> f64 {
        self.radius
    }
    fn length(&self) -> f64 {
        self.length
    }

    fn height(&self, r: f64) -> f64 {
        if r <= self.radius {
            0.0
        } else {
            -1.0
        }
    }

    fn width(&self, _h: f64) -> f64 {
        self.radius
    }

    fn xy_normal_length(&self) -> f64 {
        self.radius
    }
    fn normal_length(&self) -> f64 {
        0.0
    }
    fn center_height(&self) -> f64 {
        0.0
    }

    fn offset_cutter(&self, d: f64) -> Option<Box<dyn MillingCutter>> {
        Some(Box::new(BullCutter::new(
            self.diameter + 2.0 * d,
            d,
            self.length + d,
        )))
    }

    fn single_edge_drop_canonical(&self, u1: &Point, u2: &Point) -> (f64, f64) {
        // the cutter rim intersects the edge at x = +/- s; pick the higher one
        let s = (square(self.radius) - square(u1.y)).sqrt();
        let mut cc1 = Point::new(s, u1.y, 0.0);
        let mut cc2 = Point::new(-s, u1.y, 0.0);
        cc1.z_project_onto_edge(u1, u2);
        cc2.z_project_onto_edge(u1, u2);
        if cc1.z > cc2.z {
            (cc1.x, cc1.z)
        } else {
            (cc2.x, cc2.z)
        }
    }

    /// In addition to the vertices of `t`, the flat bottom can touch the points where the
    /// triangle edges cross the plane of the fiber.
    fn vertex_push(&self, f: &Fiber, i: &mut Interval, t: &Triangle) -> bool {
        let mut result = false;
        for p in &t.p {
            if single_vertex_push(self, f, i, p, CCType::Vertex) {
                result = true;
            }
        }
        if let Some((mut p1, mut p2)) = t.zslice_verts(f.p1.z) {
            p1.z = f.p1.z;
            p2.z = f.p1.z;
            if single_vertex_push(self, f, i, &p1, CCType::VertexCyl) {
                result = true;
            }
            if single_vertex_push(self, f, i, &p2, CCType::VertexCyl) {
                result = true;
            }
        }
        result
    }
}

impl fmt::Display for CylCutter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "CylCutter (d={}, L={})", self.diameter, self.length)
    }
}
