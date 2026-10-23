//! Python bindings for the Rust port of OpenCAMLib.
//!
//! The module mirrors the `opencamlib.ocl` extension module of the C++ library (its
//! pybind11 bindings): the same classes, method names, argument order and enums, so
//! existing scripts can switch implementations without changes.

use pyo3::prelude::*;

mod algo;
mod cutters;
mod dropcutter;
mod geometry;
mod util;

/// Return the version string of OpenCAMLib
#[pyfunction]
fn version() -> String {
    opencamlib::version()
}

/// Return the maximum number of available threads
#[pyfunction]
fn max_threads() -> usize {
    opencamlib::max_threads()
}

/// OpenCAMLib (Rust implementation)
#[pymodule]
fn ocl(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(version, m)?)?;
    m.add_function(wrap_pyfunction!(max_threads, m)?)?;
    geometry::register(m)?;
    cutters::register(m)?;
    algo::register(m)?;
    dropcutter::register(m)?;
    Ok(())
}
