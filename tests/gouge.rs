//! Brute-force check of drop-cutter results: sample the triangles under the cutter densely
//! and verify that the cutter neither gouges the surface (no sample above the cutter's
//! bottom surface) nor floats above it (some sample nearly touching).
//!
//! The same check is applied to the C++ reference values, to document where the C++
//! library gouges (cone cutters).

mod common;

use common::*;
use opencamlib::{BatchDropCutter, CLPoint, KDTree, MillingCutter, Point, STLSurf, Triangle};

/// Highest `q.z - height(r)` over samples `q` of the triangles under the cutter at `cl`,
/// i.e. the lowest cutter position that does not gouge any sample.
fn brute_force_z(surf: &STLSurf, kd: &KDTree, cutter: &dyn MillingCutter, cl: &Point) -> f64 {
    let r_max = cutter.radius();
    let mut best = f64::NEG_INFINITY;
    let mut probe = |q: Point| {
        let r = cl.xy_distance(&q);
        if r <= r_max * (1.0 + 1e-12) {
            best = best.max(q.z - cutter.height(r.min(r_max)));
        }
    };
    let search = Point::new(cl.x, cl.y, -1e9);
    for idx in kd.search_cutter_overlap(cutter, &search) {
        let t: &Triangle = &surf.tris[idx];
        if !cutter.overlaps(cl, t) {
            continue;
        }
        let n = 60;
        for i in 0..=n {
            for j in 0..=(n - i) {
                let (u, v) = (i as f64 / n as f64, j as f64 / n as f64);
                probe(t.p[0] + u * (t.p[1] - t.p[0]) + v * (t.p[2] - t.p[0]));
            }
        }
        let m = 2000;
        for e in 0..3 {
            let (a, b) = (t.p[e], t.p[(e + 1) % 3]);
            for k in 0..=m {
                probe(a + (k as f64 / m as f64) * (b - a));
            }
            // exact crossings of the edge with the rim circle, where steep geometry
            // would otherwise be under-sampled
            let (d, f) = (b - a, a - *cl);
            let qa = d.x * d.x + d.y * d.y;
            let qb = 2.0 * (f.x * d.x + f.y * d.y);
            let qc = f.x * f.x + f.y * f.y - r_max * r_max;
            let disc = qb * qb - 4.0 * qa * qc;
            if qa > 0.0 && disc >= 0.0 {
                for s in [-1.0, 1.0] {
                    let u = (-qb + s * disc.sqrt()) / (2.0 * qa);
                    if (0.0..=1.0).contains(&u) {
                        probe(a + u * d);
                    }
                }
            }
        }
        // the facet's plane directly below the cutter axis, and its highest points on
        // circles around the axis (the uphill direction), up to the rim
        let n = t.up_normal();
        if n.z.abs() > 1e-12 {
            let mut nxy = Point::new(n.x, n.y, 0.0);
            nxy.xy_normalize();
            let k = 400;
            for i in 0..=k {
                let mut q = cl.pos_xy() - (r_max * i as f64 / k as f64) * nxy;
                q.z = t.p[0].z - (n.x * (q.x - t.p[0].x) + n.y * (q.y - t.p[0].y)) / n.z;
                if inside_closed(&q, t) {
                    probe(q);
                }
            }
        }
    }
    best
}

trait Xy {
    fn pos_xy(&self) -> Point;
}

impl Xy for Point {
    fn pos_xy(&self) -> Point {
        Point::new(self.x, self.y, 0.0)
    }
}

/// Point-in-triangle test in the XY plane, including the boundary.
fn inside_closed(p: &Point, t: &Triangle) -> bool {
    let s = |a: &Point, b: &Point| (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    let (d1, d2, d3) = (s(&t.p[0], &t.p[1]), s(&t.p[1], &t.p[2]), s(&t.p[2], &t.p[0]));
    let neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    let pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    !(neg && pos)
}

struct Stats {
    max_gouge: f64,
    max_gap: f64,
    worst_gap: (Point, f64, f64),
    worst_gouge: (Point, f64, f64),
}

fn check(points: &[(Point, f64)], surf: &STLSurf, kd: &KDTree, cutter: &dyn MillingCutter) -> Stats {
    let mut s = Stats { max_gouge: 0.0, max_gap: 0.0, worst_gap: (Point::default(), 0.0, 0.0), worst_gouge: (Point::default(), 0.0, 0.0) };
    for (cl, z) in points {
        let bf = brute_force_z(surf, kd, cutter, cl);
        if bf == f64::NEG_INFINITY {
            continue; // nothing under the cutter
        }
        if bf - z > s.max_gouge {
            s.max_gouge = bf - z;
            s.worst_gouge = (*cl, *z, bf);
        }
        if z - bf > s.max_gap {
            s.max_gap = z - bf;
            s.worst_gap = (*cl, *z, bf);
        }
    }
    s
}

fn gouge_test(model: &str, cname: &str) {
    let lines = read_lines(&format!("drop_{model}_{cname}.txt"));
    let cutter = cutter_from_spec(&lines[0]);
    let surf = load_model(model);
    let mut kd = KDTree::new();
    kd.build(&surf.tris);
    // every 7th grid point keeps the test fast
    let reference: Vec<Vec<f64>> = lines[1..].iter().step_by(7).map(|l| nums(l)).collect();
    let mut bdc = BatchDropCutter::new(&surf, cutter.as_ref());
    for r in &reference {
        bdc.append_point(CLPoint::new(r[0], r[1], r[2]));
    }
    bdc.run();
    let rust: Vec<(Point, f64)> = bdc.get_cl_points().iter().map(|cl| (cl.pos, cl.z)).collect();
    let cpp: Vec<(Point, f64)> = reference
        .iter()
        .map(|r| (Point::new(r[0], r[1], r[3]), r[3]))
        .collect();
    let floor = reference[0][2];
    // points where the cutter rests on the floor have nothing to touch
    let touching: Vec<(Point, f64)> = rust.iter().copied().filter(|p| p.1 > floor).collect();
    let rs = check(&touching, &surf, &kd, cutter.as_ref());
    let cs = check(&cpp, &surf, &kd, cutter.as_ref());
    println!(
        "{model} {cname}: Rust max gouge {:.2e} max gap {:.2e} | C++ max gouge {:.2e}",
        rs.max_gouge, rs.max_gap, cs.max_gouge
    );
    if std::env::var("GOUGE_DEBUG").is_ok() {
        let (gp, gz, gbf) = rs.worst_gouge;
        println!("  worst gouge at ({}, {}) z={} brute={}", gp.x, gp.y, gz, gbf);
        let (p, z, bf) = rs.worst_gap;
        println!("  worst gap at ({}, {}) z={} brute={}", p.x, p.y, z, bf);
    }
    // Composite cutters accept contacts up to 1e-6 beyond a sub-cutter's radius, which
    // makes their profile (and so this check) ambiguous at that scale.
    let single = matches!(cname, "cyl" | "ball" | "bull" | "cone");
    let gouge_tol = if single { 1e-9 } else { 1e-5 };
    assert!(rs.max_gouge < gouge_tol, "{cname}: Rust result gouges by {}", rs.max_gouge);
    // the brute force under-samples, so it can only find positions slightly lower
    assert!(rs.max_gap < 1e-4, "{cname}: Rust result floats {} above the surface", rs.max_gap);
}

macro_rules! gouge_tests {
    ($($name:ident: $model:expr, $cutter:expr;)*) => {$(
        #[test]
        fn $name() {
            gouge_test($model, $cutter);
        }
    )*};
}

gouge_tests! {
    gouge_tux_cyl: "tux", "cyl";
    gouge_tux_ball: "tux", "ball";
    gouge_tux_bull: "tux", "bull";
    gouge_tux_cone: "tux", "cone";
    gouge_tux_cylcone: "tux", "cylcone";
    gouge_tux_ballcone: "tux", "ballcone";
    gouge_tux_bullcone: "tux", "bullcone";
    gouge_tux_conecone: "tux", "conecone";
    gouge_tux_compball: "tux", "compball";
    gouge_demo_cyl: "demo", "cyl";
    gouge_demo_ball: "demo", "ball";
    gouge_demo_bull: "demo", "bull";
    gouge_demo_cone: "demo", "cone";
    gouge_demo_cylcone: "demo", "cylcone";
    gouge_demo_ballcone: "demo", "ballcone";
    gouge_demo_bullcone: "demo", "bullcone";
    gouge_demo_conecone: "demo", "conecone";
}
