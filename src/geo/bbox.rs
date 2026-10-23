use std::fmt;
use std::ops::Index;

use super::{Point, Triangle};

/// Axis-aligned bounding box.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Bbox {
    /// The maximum point.
    pub maxpt: Point,
    /// The minimum point.
    pub minpt: Point,
    /// False until a point or triangle has been added.
    initialized: bool,
}

impl Bbox {
    /// Bounding box from explicit limits.
    pub fn new(minx: f64, maxx: f64, miny: f64, maxy: f64, minz: f64, maxz: f64) -> Self {
        Bbox {
            minpt: Point::new(minx, miny, minz),
            maxpt: Point::new(maxx, maxy, maxz),
            initialized: true,
        }
    }

    /// True once at least one point has been added.
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// True if `p` is inside this box (inclusive).
    pub fn is_inside(&self, p: &Point) -> bool {
        !(p.x > self.maxpt.x
            || p.x < self.minpt.x
            || p.y > self.maxpt.y
            || p.y < self.minpt.y
            || p.z > self.maxpt.z
            || p.z < self.minpt.z)
    }

    /// True if this box overlaps `b`.
    pub fn overlaps(&self, b: &Bbox) -> bool {
        !((self.maxpt.x < b.minpt.x || self.minpt.x > b.maxpt.x)
            || (self.maxpt.y < b.minpt.y || self.minpt.y > b.maxpt.y)
            || (self.maxpt.z < b.minpt.z || self.minpt.z > b.maxpt.z))
    }

    /// Reset the box; the next added point re-initializes it.
    pub fn clear(&mut self) {
        self.initialized = false;
    }

    /// Enlarge the box so that `p` is contained in it.
    pub fn add_point(&mut self, p: &Point) {
        if !self.initialized {
            self.maxpt = *p;
            self.minpt = *p;
            self.initialized = true;
        } else {
            self.maxpt.x = self.maxpt.x.max(p.x);
            self.minpt.x = self.minpt.x.min(p.x);
            self.maxpt.y = self.maxpt.y.max(p.y);
            self.minpt.y = self.minpt.y.min(p.y);
            self.maxpt.z = self.maxpt.z.max(p.z);
            self.minpt.z = self.minpt.z.min(p.z);
        }
    }

    /// Enlarge the box so that all vertices of `t` are contained in it.
    pub fn add_triangle(&mut self, t: &Triangle) {
        for p in &t.p {
            self.add_point(p);
        }
    }

    /// The limits as `[minx, maxx, miny, maxy, minz, maxz]`.
    pub fn bounds(&self) -> [f64; 6] {
        [
            self.minpt.x,
            self.maxpt.x,
            self.minpt.y,
            self.maxpt.y,
            self.minpt.z,
            self.maxpt.z,
        ]
    }
}

/// Index as `[minx, maxx, miny, maxy, minz, maxz]`.
impl Index<usize> for Bbox {
    type Output = f64;
    fn index(&self, idx: usize) -> &f64 {
        match idx {
            0 => &self.minpt.x,
            1 => &self.maxpt.x,
            2 => &self.minpt.y,
            3 => &self.maxpt.y,
            4 => &self.minpt.z,
            5 => &self.maxpt.z,
            _ => panic!("Bbox index {idx} out of range 0..6"),
        }
    }
}

impl fmt::Display for Bbox {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, " Bbox ")?;
        writeln!(f, " min= {}", self.minpt)?;
        writeln!(f, " max= {}", self.maxpt)
    }
}
