//! Minimal half-edge graph used by the weave (replaces the Boost adjacency_list).
//!
//! Vertices and edges live in arenas and are never moved, so ids stay valid after
//! removals. Removed elements are only marked dead.

use crate::geo::Point;

use super::VertexType;

pub(crate) type VertexId = usize;
pub(crate) type EdgeId = usize;

/// Sentinel for an unset `next`/`prev` edge.
pub(crate) const NO_EDGE: EdgeId = usize::MAX;

/// Which interval a weave vertex sits on: (fiber index, interval index).
pub(crate) type IntervalRef = (usize, usize);

#[derive(Debug, Clone)]
pub(crate) struct VertexData {
    pub position: Point,
    pub vtype: VertexType,
    pub out_edges: Vec<EdgeId>,
    pub in_edges: Vec<EdgeId>,
    pub alive: bool,
    /// The x-interval this vertex lies on (SmartWeave only).
    pub xi: Option<IntervalRef>,
    /// The y-interval this vertex lies on (SmartWeave only).
    pub yi: Option<IntervalRef>,
}

#[derive(Debug, Clone)]
pub(crate) struct EdgeData {
    pub source: VertexId,
    pub target: VertexId,
    pub next: EdgeId,
    pub prev: EdgeId,
    pub alive: bool,
}

#[derive(Debug, Clone, Default)]
pub(crate) struct WeaveGraph {
    pub v: Vec<VertexData>,
    pub e: Vec<EdgeData>,
    n_vertices: usize,
    n_edges: usize,
}

impl WeaveGraph {
    pub fn add_vertex(&mut self, position: Point, vtype: VertexType) -> VertexId {
        self.v.push(VertexData {
            position,
            vtype,
            out_edges: Vec::new(),
            in_edges: Vec::new(),
            alive: true,
            xi: None,
            yi: None,
        });
        self.n_vertices += 1;
        self.v.len() - 1
    }

    pub fn add_edge(&mut self, v1: VertexId, v2: VertexId) -> EdgeId {
        let id = self.e.len();
        self.e.push(EdgeData {
            source: v1,
            target: v2,
            next: NO_EDGE,
            prev: NO_EDGE,
            alive: true,
        });
        self.v[v1].out_edges.push(id);
        self.v[v2].in_edges.push(id);
        self.n_edges += 1;
        id
    }

    /// The first edge from `v1` to `v2`, if any.
    pub fn edge(&self, v1: VertexId, v2: VertexId) -> Option<EdgeId> {
        self.v[v1]
            .out_edges
            .iter()
            .copied()
            .find(|&e| self.e[e].target == v2)
    }

    fn kill_edge(&mut self, e: EdgeId) {
        if !self.e[e].alive {
            return;
        }
        self.e[e].alive = false;
        let (s, t) = (self.e[e].source, self.e[e].target);
        self.v[s].out_edges.retain(|&x| x != e);
        self.v[t].in_edges.retain(|&x| x != e);
        self.n_edges -= 1;
    }

    /// Remove all edges from `v1` to `v2`.
    pub fn remove_edge_between(&mut self, v1: VertexId, v2: VertexId) {
        let doomed: Vec<EdgeId> = self.v[v1]
            .out_edges
            .iter()
            .copied()
            .filter(|&e| self.e[e].target == v2)
            .collect();
        for e in doomed {
            self.kill_edge(e);
        }
    }

    /// Remove all edges into and out of `v`.
    pub fn clear_vertex(&mut self, v: VertexId) {
        let doomed: Vec<EdgeId> = self.v[v]
            .out_edges
            .iter()
            .chain(self.v[v].in_edges.iter())
            .copied()
            .collect();
        for e in doomed {
            self.kill_edge(e);
        }
    }

    /// Remove `v` (call [`WeaveGraph::clear_vertex`] first).
    pub fn remove_vertex(&mut self, v: VertexId) {
        if self.v[v].alive {
            self.v[v].alive = false;
            self.n_vertices -= 1;
        }
    }

    pub fn target(&self, e: EdgeId) -> VertexId {
        self.e[e].target
    }

    pub fn source(&self, e: EdgeId) -> VertexId {
        self.e[e].source
    }

    /// Live vertices, in insertion order.
    pub fn vertices(&self) -> impl Iterator<Item = VertexId> + '_ {
        (0..self.v.len()).filter(move |&i| self.v[i].alive)
    }

    /// Live edges, in insertion order.
    pub fn edges(&self) -> impl Iterator<Item = EdgeId> + '_ {
        (0..self.e.len()).filter(move |&i| self.e[i].alive)
    }

    pub fn num_vertices(&self) -> usize {
        self.n_vertices
    }

    pub fn num_edges(&self) -> usize {
        self.n_edges
    }
}
