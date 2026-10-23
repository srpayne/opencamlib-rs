//! Reading of ASCII and binary STL files.

use std::fs;
use std::io;
use std::path::Path as FsPath;

use super::{Point, STLSurf, Triangle};

/// Read the STL file at `path` and append its triangles to `surface`.
pub fn read_stl_file<P: AsRef<FsPath>>(path: P, surface: &mut STLSurf) -> io::Result<()> {
    let data = fs::read(path)?;
    read_stl_bytes(&data, surface)
}

/// Parse STL data (ASCII or binary) and append its triangles to `surface`.
///
/// Like the C++ reader, coordinates are read as 32-bit floats. A file is treated as ASCII
/// if it starts with `solid`, unless its size exactly matches the binary layout for the
/// facet count in its header (some binary exporters also start the header with `solid`).
pub fn read_stl_bytes(data: &[u8], surface: &mut STLSurf) -> io::Result<()> {
    if data.len() < 5 {
        return Ok(());
    }
    let looks_ascii = &data[..5] == b"solid";
    let binary_size_matches = data.len() >= 84 && {
        let n = u32::from_le_bytes([data[80], data[81], data[82], data[83]]) as usize;
        data.len() == 84 + 50 * n
    };
    if looks_ascii && !binary_size_matches {
        read_ascii(data, surface);
        Ok(())
    } else {
        read_binary(data, surface)
    }
}

fn read_binary(data: &[u8], surface: &mut STLSurf) -> io::Result<()> {
    if data.len() < 84 {
        return Err(io::Error::new(
            io::ErrorKind::UnexpectedEof,
            "binary STL header is truncated",
        ));
    }
    let n = u32::from_le_bytes([data[80], data[81], data[82], data[83]]) as usize;
    let f = |off: usize| -> f64 {
        f32::from_le_bytes([data[off], data[off + 1], data[off + 2], data[off + 3]]) as f64
    };
    for i in 0..n {
        let base = 84 + 50 * i;
        if base + 50 > data.len() {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "binary STL facet data is truncated",
            ));
        }
        // skip the 12-byte normal
        let v = |k: usize| {
            let o = base + 12 + 12 * k;
            Point::new(f(o), f(o + 4), f(o + 8))
        };
        surface.add_triangle(Triangle::new(v(0), v(1), v(2)));
    }
    Ok(())
}

fn read_ascii(data: &[u8], surface: &mut STLSurf) {
    let text = String::from_utf8_lossy(data);
    let mut x = [[0f32; 3]; 3];
    let mut vertex = 0usize;
    for line in text.lines().skip(1) {
        let mut words = line.split_whitespace();
        let Some(keyword) = words.next() else {
            continue;
        };
        if keyword.starts_with("verte") {
            if keyword == "vertex" {
                for (k, w) in words.take(3).enumerate() {
                    if let Ok(v) = w.parse::<f32>() {
                        x[vertex][k] = v;
                    }
                }
            }
            vertex = (vertex + 1).min(2);
        } else if keyword.starts_with("facet") {
            vertex = 0;
        } else if keyword.starts_with("endfa") && vertex == 2 {
            let p = |k: usize| Point::new(x[k][0] as f64, x[k][1] as f64, x[k][2] as f64);
            surface.add_triangle(Triangle::new(p(0), p(1), p(2)));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ascii() {
        let s = b"solid t\n facet normal 0 0 1\n  outer loop\n   vertex 0 0 0\n   vertex 1 0 0\n   vertex 0 1 0\n  endloop\n endfacet\nendsolid t\n";
        let mut surf = STLSurf::new();
        read_stl_bytes(s, &mut surf).unwrap();
        assert_eq!(surf.size(), 1);
        assert_eq!(surf.tris[0].p[1], Point::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn binary() {
        let mut d = vec![0u8; 80];
        d.extend_from_slice(&1u32.to_le_bytes());
        for v in [0f32, 0., 1., 0., 0., 0., 1., 0., 0., 0., 1., 0.] {
            d.extend_from_slice(&v.to_le_bytes());
        }
        d.extend_from_slice(&[0, 0]);
        let mut surf = STLSurf::new();
        read_stl_bytes(&d, &mut surf).unwrap();
        assert_eq!(surf.size(), 1);
        assert_eq!(surf.tris[0].p[2], Point::new(0.0, 1.0, 0.0));

        // Binary file whose header starts with "solid" (the C++ reader hangs on these).
        d[..5].copy_from_slice(b"solid");
        let mut surf = STLSurf::new();
        read_stl_bytes(&d, &mut surf).unwrap();
        assert_eq!(surf.size(), 1);
    }
}
