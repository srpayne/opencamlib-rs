use std::fmt;
use std::io;
use std::path::Path as FsPath;

use super::{Bbox, Triangle};

/// A triangulated surface, typically read from an STL file.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct STLSurf {
    /// The triangles of the surface.
    pub tris: Vec<Triangle>,
    /// Bounding box of all triangles.
    pub bb: Bbox,
}

impl STLSurf {
    pub fn new() -> Self {
        STLSurf::default()
    }

    /// Read an ASCII or binary STL file.
    pub fn from_file<P: AsRef<FsPath>>(path: P) -> io::Result<Self> {
        let mut s = STLSurf::new();
        super::stlreader::read_stl_file(path, &mut s)?;
        Ok(s)
    }

    /// Add a triangle to the surface.
    pub fn add_triangle(&mut self, t: Triangle) {
        self.bb.add_triangle(&t);
        self.tris.push(t);
    }

    /// Number of triangles.
    pub fn size(&self) -> usize {
        self.tris.len()
    }

    /// Rotate all triangles (see [`Triangle::rotate`]).
    pub fn rotate(&mut self, xr: f64, yr: f64, zr: f64) {
        self.bb.clear();
        for t in &mut self.tris {
            t.rotate(xr, yr, zr);
            self.bb.add_triangle(t);
        }
    }

    /// Bounds as `[minx, maxx, miny, maxy, minz, maxz]`.
    pub fn get_bounds(&self) -> [f64; 6] {
        self.bb.bounds()
    }
}

impl fmt::Display for STLSurf {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "STLSurf(N={})", self.tris.len())
    }
}
