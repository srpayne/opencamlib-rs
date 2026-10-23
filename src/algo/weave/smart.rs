//! The "smart weave": only the first and last crossing of each interval (plus the
//! crossings next to gaps) become vertices; all edges are added at the end.

use super::graph::{IntervalRef, VertexId};
use super::{IntervalSel, PosSet, VertexType, Weave};
use crate::geo::Point;

impl Weave {
    /// Build the weave graph ("smart weave").
    pub fn build_smart(&mut self) {
        if self.xfibers.is_empty() || self.yfibers.is_empty() {
            return;
        }
        self.add_vertices_x();
        self.add_vertices_y();

        for xf in 0..self.xfibers.len() {
            for xi in 0..self.xfibers[xf].ints.len() {
                let fibers: Vec<usize> = self.xdata[xf][xi].fibers.iter().copied().collect();
                for w in fibers.windows(2) {
                    let (prev, current) = (w[0], w[1]);
                    if current - prev > 1 {
                        if let Some(yi) = self.find_interval_crossing_x(xf, prev + 1) {
                            self.add_vertex(xf, prev + 1, xi, yi, VertexType::FullInt);
                        }
                        if current - prev > 2 {
                            if let Some(yi) = self.find_interval_crossing_x(xf, current - 1) {
                                self.add_vertex(xf, current - 1, xi, yi, VertexType::FullInt);
                            }
                        }
                    }
                }
            }
        }
        for yf in 0..self.yfibers.len() {
            for yi in 0..self.yfibers[yf].ints.len() {
                let fibers: Vec<usize> = self.ydata[yf][yi].fibers.iter().copied().collect();
                for w in fibers.windows(2) {
                    let (prev, current) = (w[0], w[1]);
                    if current - prev > 1 {
                        if let Some(xi) = self.find_interval_crossing_y(prev + 1, yf) {
                            self.add_vertex(prev + 1, yf, xi, yi, VertexType::FullInt);
                        }
                        if current - prev > 2 {
                            if let Some(xi) = self.find_interval_crossing_y(current - 1, yf) {
                                self.add_vertex(current - 1, yf, xi, yi, VertexType::FullInt);
                            }
                        }
                    }
                }
            }
        }
        self.add_all_edges();
    }

    /// For each X-interval, add its CL-vertices and the first and last crossing with a
    /// Y-fiber.
    fn add_vertices_x(&mut self) {
        let ny = self.yfibers.len();
        for xf in 0..self.xfibers.len() {
            for xi in 0..self.xfibers[xf].ints.len() {
                let mut yf = 0;
                let mut yi = 0;
                let mut is_crossing = self.crossing_x(yf, &mut yi, xf, xi);
                while yf < ny && !is_crossing {
                    yf += 1;
                    if yf < ny {
                        is_crossing = self.crossing_x(yf, &mut yi, xf, xi);
                    }
                }
                if yf >= ny {
                    continue;
                }
                let fib = &self.xfibers[xf];
                let (lower, upper) = (fib.point(fib.ints[xi].lower), fib.point(fib.ints[xi].upper));
                let xsel = IntervalSel::X(xf, xi);
                self.add_cl_vertex(lower, xsel, lower.x);
                self.add_cl_vertex(upper, xsel, upper.x);
                // the first crossing vertex
                self.add_vertex(xf, yf, xi, yi, VertexType::Int);
                self.xdata[xf][xi].fibers.insert(yf);
                self.ydata[yf][yi].fibers.insert(xf);
                is_crossing = self.crossing_x(yf, &mut yi, xf, xi);
                while yf < ny && is_crossing {
                    yf += 1;
                    if yf < ny {
                        is_crossing = self.crossing_x(yf, &mut yi, xf, xi);
                    }
                }
                // the last crossing vertex
                yf -= 1;
                self.add_vertex(xf, yf, xi, yi, VertexType::Int);
                self.xdata[xf][xi].fibers.insert(yf);
                self.ydata[yf][yi].fibers.insert(xf);
            }
        }
    }

    /// For each Y-interval, add its CL-vertices and the first and last crossing with an
    /// X-fiber.
    fn add_vertices_y(&mut self) {
        let nx = self.xfibers.len();
        for yf in 0..self.yfibers.len() {
            for yi in 0..self.yfibers[yf].ints.len() {
                let mut xf = 0;
                let mut xi = 0;
                let mut is_crossing = self.crossing_y(xf, &mut xi, yf, yi);
                while xf < nx && !is_crossing {
                    xf += 1;
                    if xf < nx {
                        is_crossing = self.crossing_y(xf, &mut xi, yf, yi);
                    }
                }
                if xf >= nx {
                    continue;
                }
                let fib = &self.yfibers[yf];
                let (lower, upper) = (fib.point(fib.ints[yi].lower), fib.point(fib.ints[yi].upper));
                let ysel = IntervalSel::Y(yf, yi);
                self.add_cl_vertex(lower, ysel, lower.y);
                self.add_cl_vertex(upper, ysel, upper.y);
                if self.add_vertex(xf, yf, xi, yi, VertexType::Int) {
                    self.xdata[xf][xi].fibers.insert(yf);
                    self.ydata[yf][yi].fibers.insert(xf);
                }
                is_crossing = self.crossing_y(xf, &mut xi, yf, yi);
                while xf < nx && is_crossing {
                    xf += 1;
                    if xf < nx {
                        is_crossing = self.crossing_y(xf, &mut xi, yf, yi);
                    }
                }
                xf -= 1;
                if self.add_vertex(xf, yf, xi, yi, VertexType::Int) {
                    self.xdata[xf][xi].fibers.insert(yf);
                    self.ydata[yf][yi].fibers.insert(xf);
                }
            }
        }
    }

    /// Does Y-fiber `yf` cross X-interval `xi` of X-fiber `xf`? If so, set `yi` to the
    /// crossing Y-interval.
    fn crossing_x(&self, yf: usize, yi: &mut usize, xf: usize, xi: usize) -> bool {
        let (yfib, xfib) = (&self.yfibers[yf], &self.xfibers[xf]);
        let xint = &xfib.ints[xi];
        if yfib.p1.x >= xfib.point(xint.lower).x && yfib.p1.x <= xfib.point(xint.upper).x {
            for (k, it) in yfib.ints.iter().enumerate() {
                if yfib.point(it.lower).y <= xfib.p1.y && yfib.point(it.upper).y >= xfib.p1.y {
                    *yi = k;
                    return true;
                }
            }
        }
        false
    }

    /// Does X-fiber `xf` cross Y-interval `yi` of Y-fiber `yf`? If so, set `xi` to the
    /// crossing X-interval.
    fn crossing_y(&self, xf: usize, xi: &mut usize, yf: usize, yi: usize) -> bool {
        let (xfib, yfib) = (&self.xfibers[xf], &self.yfibers[yf]);
        let yint = &yfib.ints[yi];
        if xfib.p1.y >= yfib.point(yint.lower).y && xfib.p1.y <= yfib.point(yint.upper).y {
            for (k, it) in xfib.ints.iter().enumerate() {
                if xfib.point(it.lower).x <= yfib.p1.x && xfib.point(it.upper).x >= yfib.p1.x {
                    *xi = k;
                    return true;
                }
            }
        }
        false
    }

    /// The first interval of Y-fiber `yf` crossed by X-fiber `xf`.
    fn find_interval_crossing_x(&self, xf: usize, yf: usize) -> Option<usize> {
        let mut dummy = 0;
        (0..self.yfibers[yf].ints.len()).find(|&yi| self.crossing_y(xf, &mut dummy, yf, yi))
    }

    /// The first interval of X-fiber `xf` crossed by Y-fiber `yf`.
    fn find_interval_crossing_y(&self, xf: usize, yf: usize) -> Option<usize> {
        let mut dummy = 0;
        (0..self.xfibers[xf].ints.len()).find(|&xi| self.crossing_x(yf, &mut dummy, xf, xi))
    }

    /// Add a crossing vertex of X-interval (xf, xi) and Y-interval (yf, yi), unless the
    /// Y-interval already has a crossing with this X-fiber.
    fn add_vertex(&mut self, xf: usize, yf: usize, xi: usize, yi: usize, vtype: VertexType) -> bool {
        let xfib = &self.xfibers[xf];
        if self.ydata[yf][yi]
            .fibers
            .iter()
            .any(|&k| self.xfibers[k] == *xfib)
        {
            return false;
        }
        let v_position = Point::new(self.yfibers[yf].p1.x, xfib.p1.y, xfib.p1.z);
        let v = self.g.add_vertex(v_position, vtype);
        self.g.v[v].xi = Some((xf, xi));
        self.g.v[v].yi = Some((yf, yi));
        self.xdata[xf][xi].intersections.insert(v_position.x, v);
        self.ydata[yf][yi].intersections.insert(v_position.y, v);
        true
    }

    fn interval_set(&self, r: IntervalRef, x: bool) -> &PosSet {
        if x {
            &self.xdata[r.0][r.1].intersections
        } else {
            &self.ydata[r.0][r.1].intersections
        }
    }

    /// Connect every crossing vertex to its four neighbours and link the half-edges.
    fn add_all_edges(&mut self) {
        let vertices: Vec<VertexId> = self.g.vertices().collect();
        for vertex in vertices {
            let vd = &self.g.v[vertex];
            if !matches!(vd.vtype, VertexType::Int | VertexType::FullInt) {
                continue;
            }
            let (Some(xr), Some(yr)) = (vd.xi, vd.yi) else {
                continue;
            };
            let pos = vd.position;
            let Some((x_u, x_l)) = find_neighbor_vertices(self.interval_set(xr, true), pos.x) else {
                continue;
            };
            let Some((y_u, y_l)) = find_neighbor_vertices(self.interval_set(yr, false), pos.y) else {
                continue;
            };
            let adjacent = [x_l, y_u, x_u, y_l];
            let mut in_edges = Vec::with_capacity(4);
            let mut out_edges = Vec::with_capacity(4);
            for &adj in &adjacent {
                let (inn, out) = match (self.g.edge(adj, vertex), self.g.edge(vertex, adj)) {
                    (Some(i), Some(o)) => (i, o),
                    _ => (self.g.add_edge(adj, vertex), self.g.add_edge(vertex, adj)),
                };
                in_edges.push(inn);
                out_edges.push(out);
                if self.g.v[adj].vtype == VertexType::CL {
                    // a CL-vertex is a dead end: turn around
                    self.g.e[inn].prev = out;
                    self.g.e[out].next = inn;
                }
            }
            let n = in_edges.len();
            for k in 0..n {
                self.g.e[in_edges[k]].next = out_edges[(k + 1) % n];
                self.g.e[out_edges[k]].prev = in_edges[(k + n - 1) % n];
            }
        }
    }
}

/// The vertices on either side of position `pos` (which is itself a vertex): (above, below).
fn find_neighbor_vertices(set: &PosSet, pos: f64) -> Option<(VertexId, VertexId)> {
    let itr = set.lower_bound(pos);
    if itr == 0 || itr + 1 >= set.len() {
        return None;
    }
    Some((set.vertex(itr + 1), set.vertex(itr - 1)))
}
