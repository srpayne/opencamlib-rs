#![allow(dead_code)]

use std::f64::consts::PI;
use std::fs;
use std::path::PathBuf;

use opencamlib::{BallCutter, BullCutter, CompositeCutter, ConeCutter, CylCutter, MillingCutter, STLSurf};

pub fn data_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name)
}

pub fn load_model(name: &str) -> STLSurf {
    STLSurf::from_file(data_path(&format!("{name}.stl"))).expect("test model")
}

pub fn read_lines(name: &str) -> Vec<String> {
    fs::read_to_string(data_path(name))
        .unwrap_or_else(|e| panic!("{name}: {e}"))
        .lines()
        .map(str::to_string)
        .collect()
}

pub fn nums(line: &str) -> Vec<f64> {
    line.split_whitespace()
        .filter_map(|w| w.parse::<f64>().ok())
        .collect()
}

/// Parse a cutter spec such as `ball 1.0 5.0` (as written by gen_reference.py).
pub fn cutter_from_spec(spec: &str) -> Box<dyn MillingCutter> {
    let spec = spec.trim_start_matches('#').trim();
    let mut w = spec.split_whitespace();
    let kind = w.next().unwrap();
    let a: Vec<f64> = w.map(|x| x.parse().unwrap()).collect();
    match kind {
        "cyl" => Box::new(CylCutter::new(a[0], a[1])),
        "ball" => Box::new(BallCutter::new(a[0], a[1])),
        "bull" => Box::new(BullCutter::new(a[0], a[1], a[2])),
        "cone" => Box::new(ConeCutter::with_length(a[0], a[1], a[2])),
        "compcyl" => Box::new(CompositeCutter::comp_cyl(a[0], a[1])),
        "compball" => Box::new(CompositeCutter::comp_ball(a[0], a[1])),
        "cylcone" => Box::new(CompositeCutter::cyl_cone(a[0], a[1], a[2])),
        "ballcone" => Box::new(CompositeCutter::ball_cone(a[0], a[1], a[2])),
        "bullcone" => Box::new(CompositeCutter::bull_cone(a[0], a[1], a[2], a[3])),
        "conecone" => Box::new(CompositeCutter::cone_cone(a[0], a[1], a[2], a[3])),
        _ => panic!("unknown cutter {kind}"),
    }
}

/// True for cutters containing a cone, whose drop-cutter results intentionally differ
/// from the C++ library (see ConeCutter::facet_drop / single_edge_drop_canonical).
pub fn has_cone(spec: &str) -> bool {
    spec.contains("cone")
}

pub const QUARTER_PI: f64 = PI / 4.0;
