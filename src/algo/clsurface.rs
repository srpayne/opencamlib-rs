//! Experimental cutter-location surface: a half-edge mesh in the XY plane that starts as a
//! square and is subdivided. Like the C++ version this is a stub; `run()` does nothing.

use std::fmt;

use crate::geo::Point;

type VId = usize;
type EId = usize;
type FId = usize;

#[derive(Debug, Clone)]
struct HEdge {
    source: VId,
    target: VId,
    next: EId,
    twin: EId,
    face: FId,
    alive: bool,
}

/// Experimental cutter-location surface (see the module docs).
#[derive(Debug, Clone)]
pub struct CutterLocationSurface {
    verts: Vec<Point>,
    edges: Vec<HEdge>,
    /// One edge bounding each face.
    faces: Vec<EId>,
    far: f64,
    out_face: FId,
    min_sampling: f64,
}

impl Default for CutterLocationSurface {
    fn default() -> Self {
        CutterLocationSurface::new(1.0)
    }
}

impl CutterLocationSurface {
    /// A square from (-far, -far) to (far, far), subdivided once.
    pub fn new(far: f64) -> Self {
        let mut s = CutterLocationSurface {
            verts: Vec::new(),
            edges: Vec::new(),
            faces: Vec::new(),
            far,
            out_face: 0,
            min_sampling: 0.0,
        };
        s.init();
        s
    }

    fn add_edge(&mut self, source: VId, target: VId, face: FId) -> EId {
        self.edges.push(HEdge {
            source,
            target,
            next: 0,
            twin: 0,
            face,
            alive: true,
        });
        self.edges.len() - 1
    }

    fn init(&mut self) {
        let f = self.far;
        // The C++ version assigns vertex c twice and leaves d at the origin; here the four
        // corners are placed as intended.
        let a = self.add_vertex(Point::new(f, f, 0.0));
        let b = self.add_vertex(Point::new(-f, f, 0.0));
        let c = self.add_vertex(Point::new(-f, -f, 0.0));
        let d = self.add_vertex(Point::new(f, -f, 0.0));
        let f_outer = 0;
        let f_inner = 1;
        self.out_face = f_outer;
        let e1 = self.add_edge(a, b, f_inner);
        let e1t = self.add_edge(b, a, f_outer);
        let e2 = self.add_edge(b, c, f_inner);
        let e2t = self.add_edge(c, b, f_outer);
        let e3 = self.add_edge(c, d, f_inner);
        let e3t = self.add_edge(d, c, f_outer);
        let e4 = self.add_edge(d, a, f_inner);
        let e4t = self.add_edge(a, d, f_outer);
        self.faces = vec![e1t, e1];
        for (x, y) in [(e1, e1t), (e2, e2t), (e3, e3t), (e4, e4t)] {
            self.edges[x].twin = y;
            self.edges[y].twin = x;
        }
        for (x, y) in [(e1, e2), (e2, e3), (e3, e4), (e4, e1), (e1t, e4t), (e4t, e3t), (e3t, e2t), (e2t, e1t)] {
            self.edges[x].next = y;
        }
        self.subdivide();
    }

    fn add_vertex(&mut self, p: Point) -> VId {
        self.verts.push(p);
        self.verts.len() - 1
    }

    /// Subdivide every face except the outer one.
    pub fn subdivide(&mut self) {
        for f in 0..self.faces.len() {
            if f != self.out_face {
                self.subdivide_face(f);
            }
        }
    }

    fn face_edges(&self, f: FId) -> Vec<EId> {
        let start = self.faces[f];
        let mut out = vec![start];
        let mut e = self.edges[start].next;
        while e != start {
            out.push(e);
            e = self.edges[e].next;
        }
        out
    }

    fn previous_edge(&self, e: EId) -> EId {
        let mut p = self.edges[e].next;
        while self.edges[p].next != e {
            p = self.edges[p].next;
        }
        p
    }

    /// Insert a midpoint vertex into every edge of face `f`, and add a center vertex.
    fn subdivide_face(&mut self, f: FId) {
        let center = self.add_vertex(Point::default());
        for e in self.face_edges(f) {
            let (src, trg) = (self.edges[e].source, self.edges[e].target);
            let mid = 0.5 * (self.verts[src] + self.verts[trg]);
            let corner = self.verts[src];
            self.verts[center] += 0.25 * corner; // average of the corners
            let v = self.add_vertex(mid);
            self.insert_vertex_in_edge(v, e);
        }
    }

    /// Split edge `e` (and its twin) at vertex `v`.
    fn insert_vertex_in_edge(&mut self, v: VId, e: EId) {
        let twin = self.edges[e].twin;
        let (source, target) = (self.edges[e].source, self.edges[e].target);
        let (face, twin_face) = (self.edges[e].face, self.edges[twin].face);
        let previous = self.previous_edge(e);
        let twin_previous = self.previous_edge(twin);
        let (e_next, twin_next) = (self.edges[e].next, self.edges[twin].next);

        let e1 = self.add_edge(source, v, face);
        let e2 = self.add_edge(v, target, face);
        self.edges[previous].next = e1;
        self.edges[e1].next = e2;
        self.edges[e2].next = e_next;

        let te1 = self.add_edge(target, v, twin_face);
        let te2 = self.add_edge(v, source, twin_face);
        self.edges[twin_previous].next = te1;
        self.edges[te1].next = te2;
        self.edges[te2].next = twin_next;

        self.edges[e1].twin = te2;
        self.edges[te2].twin = e1;
        self.edges[e2].twin = te1;
        self.edges[te1].twin = e2;
        self.faces[face] = e1;
        self.faces[twin_face] = te1;
        self.edges[e].alive = false;
        self.edges[twin].alive = false;
    }

    pub fn run(&mut self) {}

    pub fn set_min_sampling(&mut self, s: f64) {
        self.min_sampling = s;
    }

    pub fn get_vertices(&self) -> Vec<Point> {
        self.verts.clone()
    }

    pub fn get_edges(&self) -> Vec<(Point, Point)> {
        self.edges
            .iter()
            .filter(|e| e.alive)
            .map(|e| (self.verts[e.source], self.verts[e.target]))
            .collect()
    }

    /// Number of edges bounding face `f`.
    pub fn face_size(&self, f: usize) -> usize {
        self.face_edges(f).len()
    }
}

impl fmt::Display for CutterLocationSurface {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(
            f,
            "CutterLocationSurface (nVerts={} , nEdges={}",
            self.verts.len(),
            self.edges.iter().filter(|e| e.alive).count()
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn subdivided_square() {
        let s = CutterLocationSurface::new(1.0);
        assert_eq!(s.face_size(1), 8);
        assert_eq!(s.get_vertices().len(), 9);
        assert_eq!(s.get_edges().len(), 16);
    }
}
