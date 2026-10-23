//! Drop a ball cutter on a grid over an STL model and print the CL-points.
//!
//! cargo run --release --example dropcutter -- model.stl [diameter] [step] > points.xyz

use std::env;
use std::io::{self, BufWriter, Write};
use std::time::Instant;

use opencamlib::{BallCutter, BatchDropCutter, CLPoint, STLSurf};

fn main() -> io::Result<()> {
    let args: Vec<String> = env::args().collect();
    let path = args.get(1).expect("usage: dropcutter model.stl [diameter] [step]");
    let diameter: f64 = args.get(2).map_or(1.0, |s| s.parse().unwrap());
    let step: f64 = args.get(3).map_or(0.1, |s| s.parse().unwrap());

    let surf = STLSurf::from_file(path)?;
    let cutter = BallCutter::new(diameter, 5.0 * diameter);
    let [minx, maxx, miny, maxy, minz, _] = surf.get_bounds();
    eprintln!("{} triangles, cutter {}", surf.size(), cutter);

    let mut bdc = BatchDropCutter::new(&surf, &cutter);
    let nx = ((maxx - minx) / step) as usize;
    let ny = ((maxy - miny) / step) as usize;
    for i in 0..=nx {
        for j in 0..=ny {
            bdc.append_point(CLPoint::new(minx + i as f64 * step, miny + j as f64 * step, minz));
        }
    }
    let t = Instant::now();
    bdc.run();
    eprintln!(
        "{} points in {:.3} s ({} threads)",
        bdc.get_cl_points().len(),
        t.elapsed().as_secs_f64(),
        bdc.get_threads()
    );

    let mut out = BufWriter::new(io::stdout().lock());
    for cl in bdc.get_cl_points() {
        writeln!(out, "{} {} {}", cl.x, cl.y, cl.z)?;
    }
    Ok(())
}
