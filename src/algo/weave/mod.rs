//! The weave: a planar graph built from the intervals of X- and Y-fibers, whose faces give
//! the waterline loops.
//!
//! Each interval end becomes a CL-vertex, and every crossing of an X-interval with a
//! Y-interval becomes an internal vertex. Half-edges around the material are linked with
//! `next` pointers, so walking them from CL-vertex to CL-vertex traces the loops.
//!
//! Two builders are available, as in the C++ library: [`Weave::build`] (the "simple
//! weave", used by [`Waterline::run`](crate::Waterline::run)) and [`Weave::build_smart`]
//! (the "smart weave", used by [`Waterline::run2`](crate::Waterline::run2)), which only
//! adds the crossings needed to describe the loops.

mod graph;
mod simple;
mod smart;

use std::collections::BTreeSet;
use std::fmt;

use graph::{VertexId, WeaveGraph, NO_EDGE};

use super::fiber::Fiber;
use crate::geo::Point;

/// Type of a weave vertex.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VertexType {
    /// Cutter-location vertex at the end of an interval.
    CL,
    CLDone,
    Adj,
    TwoAdj,
    /// Crossing of an X- and a Y-interval.
    Int,
    /// Crossing added by the smart weave between two other crossings.
    FullInt,
}

/// Vertices on one interval, sorted by position along the fiber. Like the `std::set` in
/// the C++ library, a vertex at a position that is already present is not inserted.
#[derive(Debug, Clone, Default)]
pub(crate) struct PosSet(Vec<(f64, VertexId)>);

impl PosSet {
    fn insert(&mut self, pos: f64, v: VertexId) {
        let i = self.lower_bound(pos);
        if i < self.0.len() && self.0[i].0 == pos {
            return;
        }
        self.0.insert(i, (pos, v));
    }

    /// Index of the first entry with position >= `pos`.
    fn lower_bound(&self, pos: f64) -> usize {
        self.0.partition_point(|e| e.0 < pos)
    }

    fn len(&self) -> usize {
        self.0.len()
    }

    fn vertex(&self, i: usize) -> VertexId {
        self.0[i].1
    }
}

/// Weave bookkeeping attached to one fiber interval.
#[derive(Debug, Clone, Default)]
pub(crate) struct IntervalData {
    in_weave: bool,
    intersections: PosSet,
    /// Indices of the crossing fibers of the other direction (SmartWeave only).
    fibers: BTreeSet<usize>,
}

/// Weave graph built from X- and Y-fibers.
#[derive(Debug, Clone, Default)]
pub struct Weave {
    g: WeaveGraph,
    loops: Vec<Vec<VertexId>>,
    xfibers: Vec<Fiber>,
    yfibers: Vec<Fiber>,
    xdata: Vec<Vec<IntervalData>>,
    ydata: Vec<Vec<IntervalData>>,
    cl_vertex_set: BTreeSet<VertexId>,
}

impl Weave {
    pub fn new() -> Self {
        Weave::default()
    }

    /// Add a fiber. Empty fibers, and fibers parallel to neither axis, are ignored.
    pub fn add_fiber(&mut self, f: Fiber) {
        if f.empty() {
            return;
        }
        if f.dir.x_parallel() {
            self.xdata.push(vec![IntervalData::default(); f.ints.len()]);
            self.xfibers.push(f);
        } else if f.dir.y_parallel() {
            self.ydata.push(vec![IntervalData::default(); f.ints.len()]);
            self.yfibers.push(f);
        }
    }

    fn add_cl_vertex(&mut self, position: Point, ival: IntervalSel, ipos: f64) -> VertexId {
        let v = self.g.add_vertex(position, VertexType::CL);
        self.idata(ival).intersections.insert(ipos, v);
        self.cl_vertex_set.insert(v);
        v
    }

    fn idata(&mut self, sel: IntervalSel) -> &mut IntervalData {
        match sel {
            IntervalSel::X(f, i) => &mut self.xdata[f][i],
            IntervalSel::Y(f, i) => &mut self.ydata[f][i],
        }
    }

    /// Walk the half-edges from CL-vertex to CL-vertex to extract the loops.
    pub fn face_traverse(&mut self) {
        let step_limit = self.g.e.len() + 1;
        while let Some(&first) = self.cl_vertex_set.iter().next() {
            let mut lp = Vec::new();
            let mut current = first;
            loop {
                lp.push(current);
                self.cl_vertex_set.remove(&current);
                // CL-points are at the ends of intervals, so they have exactly one out-edge
                let out = &self.g.v[current].out_edges;
                if out.len() != 1 {
                    break;
                }
                let mut e = out[0];
                let mut steps = 0;
                loop {
                    current = self.g.target(e);
                    e = self.g.e[e].next;
                    steps += 1;
                    if self.g.v[current].vtype == VertexType::CL || e == NO_EDGE || steps > step_limit {
                        break;
                    }
                }
                if current == first
                    || self.g.v[current].vtype != VertexType::CL
                    || !self.cl_vertex_set.contains(&current)
                {
                    break;
                }
            }
            self.loops.push(lp);
        }
    }

    /// The loops found by [`Weave::face_traverse`].
    pub fn get_loops(&self) -> Vec<Vec<Point>> {
        self.loops
            .iter()
            .map(|lp| lp.iter().map(|&v| self.g.v[v].position).collect())
            .collect()
    }

    pub fn num_vertices(&self) -> usize {
        self.g.num_vertices()
    }

    pub fn num_edges(&self) -> usize {
        self.g.num_edges()
    }

    /// Positions of all vertices.
    pub fn get_vertices(&self) -> Vec<Point> {
        self.g.vertices().map(|v| self.g.v[v].position).collect()
    }

    /// Positions of all vertices of type `t`.
    pub fn get_vertices_by_type(&self, t: VertexType) -> Vec<Point> {
        self.g
            .vertices()
            .filter(|&v| self.g.v[v].vtype == t)
            .map(|v| self.g.v[v].position)
            .collect()
    }

    /// Positions of the CL-vertices.
    pub fn get_cl_vertices(&self) -> Vec<Point> {
        self.get_vertices_by_type(VertexType::CL)
    }

    /// Positions of the internal (crossing) vertices.
    pub fn get_int_vertices(&self) -> Vec<Point> {
        self.get_vertices_by_type(VertexType::Int)
    }

    /// All edges as (source, target) position pairs.
    pub fn get_edges(&self) -> Vec<(Point, Point)> {
        self.g
            .edges()
            .map(|e| (self.g.v[self.g.source(e)].position, self.g.v[self.g.target(e)].position))
            .collect()
    }

    pub fn x_fibers(&self) -> &[Fiber] {
        &self.xfibers
    }

    pub fn y_fibers(&self) -> &[Fiber] {
        &self.yfibers
    }
}

impl fmt::Display for Weave {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Weave2")?;
        writeln!(f, "  {} X-fibers", self.xfibers.len())?;
        writeln!(f, "  {} Y-fibers", self.yfibers.len())
    }
}

/// Selects an interval: X(fiber, interval) or Y(fiber, interval).
#[derive(Debug, Clone, Copy)]
pub(crate) enum IntervalSel {
    X(usize, usize),
    Y(usize, usize),
}
