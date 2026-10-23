//! The "simple weave": every crossing of an X- and a Y-interval becomes a vertex, and the
//! half-edges are re-linked incrementally as crossings are inserted.

use super::graph::{EdgeId, VertexId};
use super::{IntervalSel, PosSet, VertexType, Weave};
use crate::geo::Point;

impl Weave {
    /// Build the weave graph ("simple weave").
    pub fn build(&mut self) {
        for xfi in 0..self.xfibers.len() {
            for xii in 0..self.xfibers[xfi].ints.len() {
                self.add_x_interval(xfi, xii);
            }
        }
    }

    fn add_x_interval(&mut self, xfi: usize, xii: usize) {
        let xf = &self.xfibers[xfi];
        let xi = xf.ints[xii];
        let p1 = xf.point(xi.lower);
        let p2 = xf.point(xi.upper);
        let (xmin, xmax) = (p1.x, p2.x);
        let (xf_y, xf_z) = (xf.p1.y, xf.p1.z);
        if xmax - xmin <= 0.0 {
            return; // zero-length interval
        }
        let xsel = IntervalSel::X(xfi, xii);
        self.xdata[xfi][xii].in_weave = true;
        let xv1 = self.add_cl_vertex(p1, xsel, p1.x);
        let xv2 = self.add_cl_vertex(p2, xsel, p2.x);
        let e1 = self.g.add_edge(xv1, xv2);
        let e2 = self.g.add_edge(xv2, xv1);
        self.g.e[e1].next = e2;
        self.g.e[e2].next = e1;
        self.g.e[e1].prev = e2;
        self.g.e[e2].prev = e1;

        for yfi in 0..self.yfibers.len() {
            let yf_x = self.yfibers[yfi].p1.x;
            if !(xmin < yf_x && yf_x < xmax) {
                continue;
            }
            for yii in 0..self.yfibers[yfi].ints.len() {
                let yf = &self.yfibers[yfi];
                let yi = yf.ints[yii];
                let yp1 = yf.point(yi.lower);
                let yp2 = yf.point(yi.upper);
                if !(yp1.y < xf_y && xf_y < yp2.y) {
                    continue;
                }
                let ysel = IntervalSel::Y(yfi, yii);
                if !self.ydata[yfi][yii].in_weave {
                    self.add_cl_vertex(yp1, ysel, yp1.y);
                    self.add_cl_vertex(yp2, ysel, yp2.y);
                    self.ydata[yfi][yii].in_weave = true;
                }
                let v_position = Point::new(yf_x, xf_y, xf_z);
                let Some((x_u, x_l)) =
                    find_neighbor_vertices(&self.xdata[xfi][xii].intersections, v_position.x)
                else {
                    continue;
                };
                let Some((y_u, y_l)) =
                    find_neighbor_vertices(&self.ydata[yfi][yii].intersections, v_position.y)
                else {
                    continue;
                };
                self.add_int_vertex(v_position, x_l, x_u, y_l, y_u, xsel, ysel);
            }
        }

        if self.xdata[xfi][xii].intersections.len() == 2 {
            // no crossings: this interval is not part of the weave
            for v in [xv1, xv2] {
                self.cl_vertex_set.remove(&v);
                self.g.clear_vertex(v);
                self.g.remove_vertex(v);
            }
        }
    }

    /// Insert a crossing vertex at `v_position` between x_l-x_u and y_l-y_u, and re-link
    /// the `next`/`prev` pointers of the eight new half-edges.
    #[allow(clippy::too_many_arguments)]
    fn add_int_vertex(
        &mut self,
        v_position: Point,
        x_l: VertexId,
        x_u: VertexId,
        y_l: VertexId,
        y_u: VertexId,
        xsel: IntervalSel,
        ysel: IntervalSel,
    ) {
        let g = &mut self.g;
        let v = g.add_vertex(v_position, VertexType::Int);
        let (Some(xe_lu), Some(xe_ul)) = (g.edge(x_l, x_u), g.edge(x_u, x_l)) else {
            return;
        };
        let mut xe_lu_next = g.e[xe_lu].next;
        let mut xe_lu_prev = g.e[xe_lu].prev;
        let mut xe_ul_next = g.e[xe_ul].next;
        let mut xe_ul_prev = g.e[xe_ul].prev;

        // an existing y_l - y_u edge pair, if any
        let y_edges: Option<(EdgeId, EdgeId)> = match (g.edge(y_l, y_u), g.edge(y_u, y_l)) {
            (Some(a), Some(b)) => Some((a, b)),
            _ => None,
        };

        let xl_v = g.add_edge(x_l, v);
        let v_yl = g.add_edge(v, y_l);
        let yl_v = g.add_edge(y_l, v);
        let v_xu = g.add_edge(v, x_u);
        let xu_v = g.add_edge(x_u, v);
        let v_yu = g.add_edge(v, y_u);
        let yu_v = g.add_edge(y_u, v);
        let v_xl = g.add_edge(v, x_l);

        // hairpins: an edge whose next/prev is its own twin
        if xe_lu_prev == xe_ul {
            xe_lu_prev = v_xl;
        }
        if xe_lu_next == xe_ul {
            xe_lu_next = xu_v;
        }
        if xe_ul_prev == xe_lu {
            xe_ul_prev = v_xu;
        }
        if xe_ul_next == xe_lu {
            xe_ul_next = xl_v;
        }
        let (ye_lu_next, ye_lu_prev, ye_ul_next, ye_ul_prev) = match y_edges {
            Some((ye_lu, ye_ul)) => {
                let mut lu_next = g.e[ye_lu].next;
                let mut lu_prev = g.e[ye_lu].prev;
                let mut ul_next = g.e[ye_ul].next;
                let mut ul_prev = g.e[ye_ul].prev;
                if lu_prev == ye_ul {
                    lu_prev = v_yl;
                }
                if lu_next == ye_ul {
                    lu_next = yu_v;
                }
                if ul_prev == ye_lu {
                    ul_prev = v_yu;
                }
                if ul_next == ye_lu {
                    ul_next = yl_v;
                }
                (lu_next, lu_prev, ul_next, ul_prev)
            }
            // no y-edge yet: form a y-hairpin
            None => (yu_v, v_yl, yl_v, v_yu),
        };

        let mut link = |a: EdgeId, b: EdgeId| {
            g.e[a].next = b;
            g.e[b].prev = a;
        };
        link(xe_lu_prev, xl_v);
        link(xl_v, v_yl);
        link(v_yl, ye_ul_next);
        link(ye_lu_prev, yl_v);
        link(yl_v, v_xu);
        link(v_xu, xe_lu_next);
        link(xe_ul_prev, xu_v);
        link(xu_v, v_yu);
        link(v_yu, ye_lu_next);
        link(ye_ul_prev, yu_v);
        link(yu_v, v_xl);
        link(v_xl, xe_ul_next);

        g.remove_edge_between(x_l, x_u);
        g.remove_edge_between(x_u, x_l);
        if y_edges.is_some() {
            g.remove_edge_between(y_l, y_u);
            g.remove_edge_between(y_u, y_l);
        }
        self.idata(xsel).intersections.insert(v_position.x, v);
        self.idata(ysel).intersections.insert(v_position.y, v);
    }
}

/// The vertices directly above and below position `pos` on an interval: (above, below).
fn find_neighbor_vertices(set: &PosSet, pos: f64) -> Option<(VertexId, VertexId)> {
    let itr = set.lower_bound(pos);
    if itr == 0 || itr >= set.len() {
        return None;
    }
    let above = set.vertex(itr);
    let below = set.vertex(itr - 1);
    (above != below).then_some((above, below))
}
