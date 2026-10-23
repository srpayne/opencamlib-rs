//! # opencamlib
//!
//! A Rust port of [OpenCAMLib](https://github.com/aewallin/opencamlib), a library for
//! creating 3D toolpaths for CNC machines.
//!
//! Supported algorithms:
//!
//! * **Drop-cutter** ([`PointDropCutter`], [`BatchDropCutter`], [`PathDropCutter`],
//!   [`AdaptivePathDropCutter`]): drop a cutter positioned at an (x, y) location
//!   until it touches the model.
//! * **Push-cutter** ([`FiberPushCutter`], [`BatchPushCutter`]): push a cutter along
//!   an X- or Y-fiber at constant z and record where it is in contact with the model.
//! * **Waterline** ([`Waterline`], [`AdaptiveWaterline`]): constant-z toolpath loops,
//!   built from push-cutter fibers via a weave graph.
//!
//! Supported cutters: [`CylCutter`], [`BallCutter`], [`BullCutter`], [`ConeCutter`] and
//! [`CompositeCutter`] (with the `CylCone`, `BallCone`, `BullCone`, `ConeCone`, `CompCyl` and
//! `CompBall` presets).
//!
//! ```no_run
//! use opencamlib::{STLSurf, CylCutter, BatchDropCutter, CLPoint};
//!
//! let surf = STLSurf::from_file("model.stl").unwrap();
//! let cutter = CylCutter::new(6.0, 20.0);
//! let mut bdc = BatchDropCutter::new(&surf, &cutter);
//! for i in 0..100 {
//!     bdc.append_point(CLPoint::new(i as f64 * 0.5, 10.0, -10.0));
//! }
//! bdc.run();
//! for cl in bdc.get_cl_points() {
//!     println!("{} {} {}", cl.x, cl.y, cl.z);
//! }
//! ```

pub mod algo;
pub mod common;
pub mod cutters;
pub mod dropcutter;
pub mod geo;

pub use algo::adaptivewaterline::AdaptiveWaterline;
pub use algo::batchpushcutter::BatchPushCutter;
pub use algo::clsurface::CutterLocationSurface;
pub use algo::fiber::Fiber;
pub use algo::fiberpushcutter::{FiberDirection, FiberPushCutter};
pub use algo::interval::Interval;
pub use algo::tsp::TSPSolver;
pub use algo::waterline::Waterline;
pub use algo::weave::{VertexType as WeaveVertexType, Weave};
pub use algo::zigzag::ZigZag;
pub use common::kdtree::KDTree;
pub use common::lineclfilter::LineCLFilter;
pub use common::numeric::{eps, eps_d, eps_f};
pub use cutters::{
    BallCutter, BullCutter, CompositeCutter, ConeCutter, CylCutter, Ellipse, EllipsePosition,
    MillingCutter,
};
pub use dropcutter::adaptivepathdropcutter::AdaptivePathDropCutter;
pub use dropcutter::batchdropcutter::BatchDropCutter;
pub use dropcutter::pathdropcutter::PathDropCutter;
pub use dropcutter::pointdropcutter::PointDropCutter;
pub use geo::{
    Arc, Bbox, CCPoint, CCType, CLPoint, Line, Path, Point, STLSurf, Span, SpanType, Triangle,
};

/// Library version string.
pub fn version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}

/// The number of threads the batch algorithms use by default.
pub fn max_threads() -> usize {
    common::parallel::default_threads()
}
