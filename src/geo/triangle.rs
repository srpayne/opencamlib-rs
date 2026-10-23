use std::fmt;

use super::{Bbox, Point};

/// A triangle defined by its three vertices, with a cached normal and bounding box.
///
/// If you modify `p` directly, call [`Triangle::new`] again to refresh `n` and `bb`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Triangle {
    /// The three vertices.
    pub p: [Point; 3],
    /// Unit normal, in the direction of `(p0-p1) x (p0-p2)`.
    pub n: Point,
    /// Bounding box.
    pub bb: Bbox,
}

impl Default for Triangle {
    /// The triangle (1,0,0), (0,1,0), (0,0,1).
    fn default() -> Self {
        Triangle::new(
            Point::new(1.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
            Point::new(0.0, 0.0, 1.0),
        )
    }
}

impl Triangle {
    /// Create a triangle with the vertices p1, p2 and p3.
    pub fn new(p1: Point, p2: Point, p3: Point) -> Self {
        let mut t = Triangle {
            p: [p1, p2, p3],
            n: Point::default(),
            bb: Bbox::default(),
        };
        t.calc_normal();
        t.calc_bb();
        t
    }

    fn calc_bb(&mut self) {
        self.bb.clear();
        let p = self.p;
        for v in &p {
            self.bb.add_point(v);
        }
    }

    fn calc_normal(&mut self) {
        let v1 = self.p[0] - self.p[1];
        let v2 = self.p[0] - self.p[2];
        let mut n = v1.cross(&v2);
        n.normalize();
        self.n = n;
    }

    /// The normal vector, flipped if necessary so that it has a non-negative z-component.
    pub fn up_normal(&self) -> Point {
        if self.n.z < 0.0 {
            -1.0 * self.n
        } else {
            self.n
        }
    }

    /// If the triangle is sliced by the plane z = `zcut`, return the two points where the
    /// triangle edges intersect the plane.
    pub fn zslice_verts(&self, zcut: f64) -> Option<(Point, Point)> {
        if zcut <= self.bb.minpt.z || zcut >= self.bb.maxpt.z {
            return None;
        }
        let mut below = Vec::with_capacity(3);
        let mut above = Vec::with_capacity(3);
        for p in &self.p {
            if p.z <= zcut {
                below.push(*p);
            } else {
                above.push(*p);
            }
        }
        match below.len() {
            2 => {
                let t1 = (zcut - above[0].z) / (below[0].z - above[0].z);
                let t2 = (zcut - above[0].z) / (below[1].z - above[0].z);
                Some((
                    above[0] + t1 * (below[0] - above[0]),
                    above[0] + t2 * (below[1] - above[0]),
                ))
            }
            1 => {
                let t1 = (zcut - above[0].z) / (below[0].z - above[0].z);
                let t2 = (zcut - above[1].z) / (below[0].z - above[1].z);
                Some((
                    above[0] + t1 * (below[0] - above[0]),
                    above[1] + t2 * (below[0] - above[1]),
                ))
            }
            _ => None,
        }
    }

    /// Rotate `xrot` radians around the X-axis, then `yrot` around Y, then `zrot` around Z.
    pub fn rotate(&mut self, xrot: f64, yrot: f64, zrot: f64) {
        for p in &mut self.p {
            p.x_rotate(xrot);
            p.y_rotate(yrot);
            p.z_rotate(zrot);
        }
        self.calc_normal();
        self.calc_bb();
    }
}

impl fmt::Display for Triangle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "T: {} {} {}n={}",
            self.p[0], self.p[1], self.p[2], self.n
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normal_and_bb() {
        let t = Triangle::new(
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 0.0),
            Point::new(0.0, 1.0, 0.0),
        );
        assert_eq!(t.up_normal(), Point::new(0.0, 0.0, 1.0));
        assert_eq!(t.bb.maxpt, Point::new(1.0, 1.0, 0.0));
        assert!(Point::new(0.2, 0.2, 0.0).is_inside_triangle(&t));
        assert!(!Point::new(0.8, 0.8, 0.0).is_inside_triangle(&t));
    }

    #[test]
    fn zslice() {
        let t = Triangle::new(
            Point::new(0.0, 0.0, 0.0),
            Point::new(1.0, 0.0, 1.0),
            Point::new(0.0, 1.0, 1.0),
        );
        let (a, b) = t.zslice_verts(0.5).unwrap();
        assert!((a.z - 0.5).abs() < 1e-15 && (b.z - 0.5).abs() < 1e-15);
        assert!(t.zslice_verts(1.0).is_none());
    }
}
