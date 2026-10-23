//! Compute waterline loops for an STL model at several z-heights and print them.
//!
//! cargo run --release --example waterline -- model.stl [diameter] [sampling] [levels] > loops.txt

use std::env;
use std::io::{self, BufWriter, Write};
use std::time::Instant;

use opencamlib::{BallCutter, STLSurf, Waterline};

fn main() -> io::Result<()> {
    let args: Vec<String> = env::args().collect();
    let path = args.get(1).expect("usage: waterline model.stl [diameter] [sampling] [levels]");
    let diameter: f64 = args.get(2).map_or(1.0, |s| s.parse().unwrap());
    let sampling: f64 = args.get(3).map_or(0.1, |s| s.parse().unwrap());
    let levels: usize = args.get(4).map_or(5, |s| s.parse().unwrap());

    let surf = STLSurf::from_file(path)?;
    let cutter = BallCutter::new(diameter, 5.0 * diameter);
    let [_, _, _, _, minz, maxz] = surf.get_bounds();
    eprintln!("{} triangles, cutter {}", surf.size(), cutter);

    let mut wl = Waterline::new(&surf, &cutter);
    wl.set_sampling(sampling);
    let mut out = BufWriter::new(io::stdout().lock());
    for k in 0..levels {
        let z = minz + (k as f64 + 0.5) * (maxz - minz) / levels as f64;
        wl.set_z(z);
        let t = Instant::now();
        wl.run();
        let loops = wl.get_loops();
        eprintln!("z={z:.4}: {} loops in {:.3} s", loops.len(), t.elapsed().as_secs_f64());
        for lp in loops {
            writeln!(out, "# loop z={z} n={}", lp.len())?;
            for p in lp {
                writeln!(out, "{} {} {}", p.x, p.y, p.z)?;
            }
        }
    }
    Ok(())
}
