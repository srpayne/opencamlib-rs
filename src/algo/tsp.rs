//! Approximate travelling-salesman tour through a set of points (used to order toolpath
//! segments).

/// Metric TSP approximation: preorder walk of a minimum spanning tree (the same 2-approx
/// algorithm as Boost's `metric_tsp_approx`).
#[derive(Debug, Clone, Default)]
pub struct TSPSolver {
    points: Vec<(f64, f64)>,
    output: Vec<usize>,
    length: f64,
}

impl TSPSolver {
    pub fn new() -> Self {
        TSPSolver::default()
    }

    pub fn add_point(&mut self, x: f64, y: f64) {
        self.points.push((x, y));
    }

    pub fn reset(&mut self) {
        self.points.clear();
        self.output.clear();
        self.length = 0.0;
    }

    fn dist(&self, a: usize, b: usize) -> f64 {
        let (pa, pb) = (self.points[a], self.points[b]);
        ((pa.0 - pb.0).powi(2) + (pa.1 - pb.1).powi(2)).sqrt()
    }

    /// Compute the tour, starting and ending at point 0.
    pub fn run(&mut self) {
        self.output.clear();
        self.length = 0.0;
        let n = self.points.len();
        if n == 0 {
            return;
        }
        if n == 1 {
            self.output = vec![0, 0];
            return;
        }
        // Prim's algorithm from vertex 0 on the complete graph
        let mut in_tree = vec![false; n];
        let mut key = vec![f64::INFINITY; n];
        let mut pred = vec![0usize; n];
        key[0] = 0.0;
        for _ in 0..n {
            let mut u = usize::MAX;
            for v in 0..n {
                if !in_tree[v] && (u == usize::MAX || key[v] < key[u]) {
                    u = v;
                }
            }
            in_tree[u] = true;
            for v in 0..n {
                if !in_tree[v] {
                    let d = self.dist(u, v);
                    if d < key[v] {
                        key[v] = d;
                        pred[v] = u;
                    }
                }
            }
        }
        let mut children = vec![Vec::new(); n];
        for v in 1..n {
            children[pred[v]].push(v);
        }
        // preorder traversal of the tree
        let mut stack = vec![0usize];
        while let Some(v) = stack.pop() {
            self.output.push(v);
            stack.extend(children[v].iter().rev());
        }
        self.output.push(0);
        self.length = self.output.windows(2).map(|w| self.dist(w[0], w[1])).sum();
    }

    /// Total length of the tour.
    pub fn get_length(&self) -> f64 {
        self.length
    }

    /// Indices of the points in tour order (the start point is repeated at the end).
    pub fn get_output(&self) -> &[usize] {
        &self.output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn square_tour() {
        let mut t = TSPSolver::new();
        for (x, y) in [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)] {
            t.add_point(x, y);
        }
        t.run();
        assert_eq!(t.get_output().len(), 5);
        assert_eq!(t.get_output()[0], 0);
        assert_eq!(*t.get_output().last().unwrap(), 0);
        assert!((t.get_length() - 4.0).abs() < 1e-12);
    }
}
