//! Drop-cutter operations (ocl_dropcutter.cpp).
//!
//! Like the C++ bindings, `setSTL()` and `setPath()` keep a reference to the Python
//! object, so changes made to the surface or path before `run()` are seen.
#![allow(non_snake_case)]

use pyo3::prelude::*;

use opencamlib as ocl;

use crate::cutters::{PyMillingCutter, SharedCutter};
use crate::geometry::{PyCLPoint, PyPath, PySTLSurf, PyTriangle};
use crate::util::need;

pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyBatchDropCutter>()?;
    m.add_class::<PyPathDropCutter>()?;
    m.add_class::<PyAdaptivePathDropCutter>()?;
    Ok(())
}

/// Drop-cutter for many CL-points, processed in parallel.
#[pyclass(name = "BatchDropCutter", module = "opencamlib.ocl")]
pub struct PyBatchDropCutter {
    surf: Option<Py<PySTLSurf>>,
    cutter: Option<SharedCutter>,
    points: Vec<ocl::CLPoint>,
    threads: usize,
    bucket_size: usize,
    calls: usize,
}

#[pymethods]
impl PyBatchDropCutter {
    #[new]
    fn new() -> Self {
        PyBatchDropCutter {
            surf: None,
            cutter: None,
            points: Vec::new(),
            threads: ocl::max_threads(),
            bucket_size: 1,
            calls: 0,
        }
    }
    fn setSTL(&mut self, s: Py<PySTLSurf>) {
        self.surf = Some(s);
    }
    fn setCutter(&mut self, c: PyRef<PyMillingCutter>) {
        self.cutter = Some(c.c.clone());
    }
    fn setThreads(&mut self, n: usize) {
        self.threads = n.max(1);
    }
    fn getThreads(&self) -> usize {
        self.threads
    }
    fn setBucketSize(&mut self, n: usize) {
        self.bucket_size = n;
    }
    fn getBucketSize(&self) -> usize {
        self.bucket_size
    }
    fn getCalls(&self) -> usize {
        self.calls
    }
    fn appendPoint(&mut self, p: PyRef<PyCLPoint>) {
        self.points.push(PyCLPoint::get_ref(&p));
    }
    fn getCLPoints(&self, py: Python<'_>) -> PyResult<Vec<Py<PyCLPoint>>> {
        PyCLPoint::create_all(py, &self.points)
    }
    fn run(&mut self, py: Python<'_>) -> PyResult<()> {
        let surf_ref = need(&self.surf, "setSTL()")?.bind(py).borrow();
        let surf = &surf_ref.s;
        let cutter = need(&self.cutter, "setCutter()")?.clone();
        let points = std::mem::take(&mut self.points);
        let (threads, bucket_size) = (self.threads, self.bucket_size);
        let (points, calls) = py.allow_threads(move || {
            let mut bdc = ocl::BatchDropCutter::new(surf, cutter.as_ref());
            bdc.set_threads(threads);
            bdc.set_bucket_size(bucket_size);
            for p in points {
                bdc.append_point(p);
            }
            bdc.run();
            (bdc.take_cl_points(), bdc.get_calls())
        });
        self.points = points;
        self.calls = calls;
        Ok(())
    }
    /// Triangles that may lie under `cutter` positioned at `cl`.
    fn getTrianglesUnderCutter(
        &self,
        py: Python<'_>,
        cl: PyRef<PyCLPoint>,
        cutter: PyRef<PyMillingCutter>,
    ) -> PyResult<Vec<PyTriangle>> {
        let surf_ref = need(&self.surf, "setSTL()")?.bind(py).borrow();
        let c = need(&self.cutter, "setCutter()")?;
        let mut bdc = ocl::BatchDropCutter::new(&surf_ref.s, c.as_ref());
        bdc.set_bucket_size(self.bucket_size);
        let tris = bdc.get_triangles_under_cutter(&PyCLPoint::get_ref(&cl), cutter.c.as_ref());
        Ok(tris.into_iter().map(|t| PyTriangle { t }).collect())
    }
}

/// Drop-cutter along a Path, sampled uniformly.
#[pyclass(name = "PathDropCutter", module = "opencamlib.ocl")]
pub struct PyPathDropCutter {
    surf: Option<Py<PySTLSurf>>,
    cutter: Option<SharedCutter>,
    path: Option<Py<PyPath>>,
    z: f64,
    sampling: f64,
    points: Vec<ocl::CLPoint>,
}

#[pymethods]
impl PyPathDropCutter {
    #[new]
    fn new() -> Self {
        PyPathDropCutter {
            surf: None,
            cutter: None,
            path: None,
            z: 0.0,
            sampling: 0.1,
            points: Vec::new(),
        }
    }
    fn setSTL(&mut self, s: Py<PySTLSurf>) {
        self.surf = Some(s);
    }
    fn setCutter(&mut self, c: PyRef<PyMillingCutter>) {
        self.cutter = Some(c.c.clone());
    }
    fn setPath(&mut self, p: Py<PyPath>) {
        self.path = Some(p);
    }
    fn setSampling(&mut self, s: f64) -> PyResult<()> {
        positive_sampling(s)?;
        self.sampling = s;
        Ok(())
    }
    fn setZ(&mut self, z: f64) {
        self.z = z;
    }
    fn getZ(&self) -> f64 {
        self.z
    }
    #[getter]
    fn minimumZ(&self) -> f64 {
        self.z
    }
    #[setter]
    fn set_minimumZ(&mut self, z: f64) {
        self.z = z;
    }
    fn getCLPoints(&self, py: Python<'_>) -> PyResult<Vec<Py<PyCLPoint>>> {
        PyCLPoint::create_all(py, &self.points)
    }
    fn run(&mut self, py: Python<'_>) -> PyResult<()> {
        let surf_ref = need(&self.surf, "setSTL()")?.bind(py).borrow();
        let path_ref = need(&self.path, "setPath()")?.bind(py).borrow();
        let (surf, path) = (&surf_ref.s, &path_ref.p);
        let cutter = need(&self.cutter, "setCutter()")?.clone();
        let (z, sampling) = (self.z, self.sampling);
        self.points = py.allow_threads(move || {
            let mut pdc = ocl::PathDropCutter::new(surf, cutter.as_ref());
            pdc.set_path(path);
            pdc.set_z(z);
            pdc.set_sampling(sampling);
            pdc.run();
            pdc.get_cl_points().to_vec()
        });
        Ok(())
    }
}

/// Drop-cutter along a Path with adaptive sampling.
#[pyclass(name = "AdaptivePathDropCutter", module = "opencamlib.ocl")]
pub struct PyAdaptivePathDropCutter {
    surf: Option<Py<PySTLSurf>>,
    cutter: Option<SharedCutter>,
    path: Option<Py<PyPath>>,
    z: f64,
    sampling: f64,
    min_sampling: f64,
    cos_limit: f64,
    points: Vec<ocl::CLPoint>,
}

#[pymethods]
impl PyAdaptivePathDropCutter {
    #[new]
    fn new() -> Self {
        PyAdaptivePathDropCutter {
            surf: None,
            cutter: None,
            path: None,
            z: 0.0,
            sampling: 0.1,
            min_sampling: 0.01,
            cos_limit: 0.999,
            points: Vec::new(),
        }
    }
    fn setSTL(&mut self, s: Py<PySTLSurf>) {
        self.surf = Some(s);
    }
    fn setCutter(&mut self, c: PyRef<PyMillingCutter>) {
        self.cutter = Some(c.c.clone());
    }
    fn setPath(&mut self, p: Py<PyPath>) {
        self.path = Some(p);
    }
    fn setSampling(&mut self, s: f64) -> PyResult<()> {
        positive_sampling(s)?;
        self.sampling = s;
        Ok(())
    }
    fn getSampling(&self) -> f64 {
        self.sampling
    }
    fn setMinSampling(&mut self, s: f64) -> PyResult<()> {
        positive_sampling(s)?;
        self.min_sampling = s;
        Ok(())
    }
    fn setCosLimit(&mut self, lim: f64) {
        self.cos_limit = lim;
    }
    fn setZ(&mut self, z: f64) {
        self.z = z;
    }
    fn getZ(&self) -> f64 {
        self.z
    }
    #[getter]
    fn minimumZ(&self) -> f64 {
        self.z
    }
    #[setter]
    fn set_minimumZ(&mut self, z: f64) {
        self.z = z;
    }
    fn getCLPoints(&self, py: Python<'_>) -> PyResult<Vec<Py<PyCLPoint>>> {
        PyCLPoint::create_all(py, &self.points)
    }
    fn run(&mut self, py: Python<'_>) -> PyResult<()> {
        let surf_ref = need(&self.surf, "setSTL()")?.bind(py).borrow();
        let path_ref = need(&self.path, "setPath()")?.bind(py).borrow();
        let (surf, path) = (&surf_ref.s, &path_ref.p);
        let cutter = need(&self.cutter, "setCutter()")?.clone();
        let (z, sampling, min_sampling, cos_limit) = (self.z, self.sampling, self.min_sampling, self.cos_limit);
        self.points = py.allow_threads(move || {
            let mut apdc = ocl::AdaptivePathDropCutter::new(surf, cutter.as_ref());
            apdc.set_path(path);
            apdc.set_z(z);
            apdc.set_sampling(sampling);
            apdc.set_min_sampling(min_sampling);
            apdc.set_cos_limit(cos_limit);
            apdc.run();
            apdc.get_cl_points().to_vec()
        });
        Ok(())
    }
}

fn positive_sampling(s: f64) -> PyResult<()> {
    if s > 0.0 {
        Ok(())
    } else {
        Err(pyo3::exceptions::PyValueError::new_err(format!("sampling must be positive, got {s}")))
    }
}
