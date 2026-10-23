//! KD-tree for fast searching of triangles that may overlap a cutter.
//!
//! The tree is built over the bounding boxes of the triangles. Each node cuts along one of
//! the six bounding-box coordinates `[minx, maxx, miny, maxy, minz, maxz]`; which of those
//! are used is chosen with [`KDTree::set_xy_dimensions`] (drop-cutter),
//! [`KDTree::set_yz_dimensions`] (X-fibers) or [`KDTree::set_xz_dimensions`] (Y-fibers).

use crate::common::numeric::is_zero_tol;
use crate::cutters::MillingCutter;
use crate::geo::{Bbox, Point, Triangle};

#[derive(Debug, Clone)]
enum KDNode {
    /// Bucket of triangle indices.
    Leaf(Vec<usize>),
    Internal {
        dim: usize,
        cutval: f64,
        hi: Option<usize>,
        lo: Option<usize>,
    },
}

/// KD-tree over the triangles of a surface. Searches return indices into the triangle
/// slice the tree was built from.
#[derive(Debug, Clone)]
pub struct KDTree {
    nodes: Vec<KDNode>,
    root: Option<usize>,
    bucket_size: usize,
    dimensions: Vec<usize>,
}

impl Default for KDTree {
    fn default() -> Self {
        KDTree::new()
    }
}

impl KDTree {
    /// An empty tree with bucket size 1, searching in the XY plane.
    pub fn new() -> Self {
        KDTree {
            nodes: Vec::new(),
            root: None,
            bucket_size: 1,
            dimensions: vec![0, 1, 2, 3],
        }
    }

    /// Maximum number of triangles in a leaf.
    pub fn set_bucket_size(&mut self, b: usize) {
        self.bucket_size = b;
    }

    /// Search in the XY plane (for drop-cutter).
    pub fn set_xy_dimensions(&mut self) {
        self.dimensions = vec![0, 1, 2, 3];
    }

    /// Search in the YZ plane (for X-fibers).
    pub fn set_yz_dimensions(&mut self) {
        self.dimensions = vec![2, 3, 4, 5];
    }

    /// Search in the XZ plane (for Y-fibers).
    pub fn set_xz_dimensions(&mut self) {
        self.dimensions = vec![0, 1, 4, 5];
    }

    /// Build the tree from `tris`.
    pub fn build(&mut self, tris: &[Triangle]) {
        self.nodes.clear();
        self.root = None;
        if tris.is_empty() {
            return;
        }
        let all: Vec<usize> = (0..tris.len()).collect();
        self.root = Some(self.build_node(tris, all));
    }

    fn build_node(&mut self, tris: &[Triangle], idx: Vec<usize>) -> usize {
        let (dim, val, start) = self.calc_spread(tris, &idx);
        let cutvalue = start + val / 2.0;
        if idx.len() <= self.bucket_size || is_zero_tol(val) {
            self.nodes.push(KDNode::Leaf(idx));
            return self.nodes.len() - 1;
        }
        let (hilist, lolist): (Vec<usize>, Vec<usize>) =
            idx.into_iter().partition(|&i| tris[i].bb[dim] > cutvalue);
        let node = self.nodes.len();
        self.nodes.push(KDNode::Internal {
            dim,
            cutval: cutvalue,
            hi: None,
            lo: None,
        });
        let hi = (!hilist.is_empty()).then(|| self.build_node(tris, hilist));
        let lo = (!lolist.is_empty()).then(|| self.build_node(tris, lolist));
        if let KDNode::Internal { hi: h, lo: l, .. } = &mut self.nodes[node] {
            *h = hi;
            *l = lo;
        }
        node
    }

    /// Find the dimension with the largest spread. Returns (dimension, spread, start).
    fn calc_spread(&self, tris: &[Triangle], idx: &[usize]) -> (usize, f64, f64) {
        let mut maxval = [0.0f64; 6];
        let mut minval = [0.0f64; 6];
        for (k, &i) in idx.iter().enumerate() {
            for &d in &self.dimensions {
                let v = tris[i].bb[d];
                if k == 0 {
                    maxval[d] = v;
                    minval[d] = v;
                } else {
                    if maxval[d] < v {
                        maxval[d] = v;
                    }
                    if minval[d] > v {
                        minval[d] = v;
                    }
                }
            }
        }
        let mut max = 0.0;
        let mut best = 0;
        for (m, &d) in self.dimensions.iter().enumerate() {
            let val = maxval[d] - minval[d];
            if val > max {
                max = val;
                best = m;
            }
        }
        let d = self.dimensions[best];
        (d, maxval[d] - minval[d], minval[d])
    }

    /// Indices of all triangles in buckets that may overlap `bb`.
    pub fn search(&self, bb: &Bbox) -> Vec<usize> {
        let mut out = Vec::new();
        if let Some(r) = self.root {
            self.search_node(&mut out, bb, r);
        }
        out
    }

    /// Indices of triangles that may lie under `cutter` positioned at `cl`.
    pub fn search_cutter_overlap<C: MillingCutter + ?Sized>(&self, cutter: &C, cl: &Point) -> Vec<usize> {
        let r = cutter.radius();
        let bb = Bbox::new(
            cl.x - r,
            cl.x + r,
            cl.y - r,
            cl.y + r,
            cl.z,
            cl.z + cutter.length(),
        );
        self.search(&bb)
    }

    fn search_node(&self, out: &mut Vec<usize>, bb: &Bbox, node: usize) {
        match &self.nodes[node] {
            KDNode::Leaf(tris) => out.extend_from_slice(tris),
            &KDNode::Internal { dim, cutval, hi, lo } => {
                let only_lo = dim % 2 == 0 && cutval > bb[dim + 1];
                let only_hi = dim % 2 == 1 && cutval < bb[dim - 1];
                if !only_lo {
                    if let Some(h) = hi {
                        self.search_node(out, bb, h);
                    }
                }
                if !only_hi {
                    if let Some(l) = lo {
                        self.search_node(out, bb, l);
                    }
                }
            }
        }
    }
}

impl std::fmt::Display for KDTree {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "KDTree (nodes={})", self.nodes.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn search_finds_overlapping() {
        let mut tris = Vec::new();
        for i in 0..20 {
            let x = i as f64;
            tris.push(Triangle::new(
                Point::new(x, 0.0, 0.0),
                Point::new(x + 0.5, 0.0, 0.0),
                Point::new(x, 0.5, 0.0),
            ));
        }
        let mut kd = KDTree::new();
        kd.build(&tris);
        // The search is conservative: it may return extra triangles, never fewer.
        let found = kd.search(&Bbox::new(4.9, 6.1, -1.0, 1.0, -1.0, 1.0));
        assert!(found.contains(&5) && found.contains(&6));
        assert!(found.len() < tris.len());
    }
}
