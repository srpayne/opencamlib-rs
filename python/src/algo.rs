//! Push-cutter, waterline and helper algorithms (ocl_algo.cpp).
#![allow(non_snake_case)]

use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;

use opencamlib as ocl;

use crate::cutters::{PyMillingCutter, SharedCutter};
use crate::geometry::{PyCLPoint, PyFiber, PyPoint, PySTLSurf, PyTriangle};
use crate::util::need;

pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(eps, m)?)?;
    m.add_function(wrap_pyfunction!(epsF, m)?)?;
    m.add_function(wrap_pyfunction!(epsD, m)?)?;
    m.add_class::<PyZigZag>()?;
    m.add_class::<PyBatchPushCutter>()?;
    m.add_class::<PyWaterline>()?;
    m.add_class::<PyAdaptiveWaterline>()?;
    m.add_class::<PyWeaveVertexType>()?;
    m.add_class::<PyWeave>()?;
    m.add_class::<PySimpleWeave>()?;
    m.add_class::<PySmartWeave>()?;
    m.add_class::<PyLineCLFilter>()?;
    m.add_class::<PyCutterLocationSurface>()?;
    m.add_class::<PyTSPSolver>()?;
    Ok(())
}

/// machine epsilon, see numeric.cpp
#[pyfunction]
fn eps() -> f64 {
    ocl::eps()
}

#[pyfunction]
fn epsF(x: f32) -> f32 {
    ocl::eps_f(x)
}

#[pyfunction]
fn epsD(x: f64) -> f64 {
    ocl::eps_d(x)
}

fn points(v: &[ocl::Point]) -> Vec<PyPoint> {
    v.iter().map(|&p| p.into()).collect()
}

fn loops(v: &[Vec<ocl::Point>]) -> Vec<Vec<PyPoint>> {
    v.iter().map(|lp| points(lp)).collect()
}

fn fibers(v: &[ocl::Fiber]) -> Vec<PyFiber> {
    v.iter().map(|f| PyFiber { f: f.clone() }).collect()
}

fn edges(v: Vec<(ocl::Point, ocl::Point)>) -> Vec<(PyPoint, PyPoint)> {
    v.into_iter().map(|(a, b)| (a.into(), b.into())).collect()
}

// ---------------------------------------------------------------- ZigZag

/// Experimental zigzag pocketing.
#[pyclass(name = "ZigZag", module = "opencamlib.ocl")]
#[derive(Default)]
pub struct PyZigZag {
    z: ocl::ZigZag,
}

#[pymethods]
impl PyZigZag {
    #[new]
    fn new() -> Self {
        PyZigZag::default()
    }
    fn run(&mut self) {
        self.z.run();
    }
    fn setDirection(&mut self, d: PyRef<PyPoint>) {
        self.z.set_direction(d.p);
    }
    fn setOrigin(&mut self, d: PyRef<PyPoint>) {
        self.z.set_origin(d.p);
    }
    fn setStepOver(&mut self, d: f64) {
        self.z.set_step_over(d);
    }
    fn addPoint(&mut self, p: PyRef<PyPoint>) {
        self.z.add_point(p.p);
    }
    fn getOutput(&self) -> Vec<PyPoint> {
        points(self.z.get_output())
    }
    fn __str__(&self) -> String {
        self.z.to_string()
    }
}

// ---------------------------------------------------------------- BatchPushCutter

/// Push-cutter for many fibers, processed in parallel.
#[pyclass(name = "BatchPushCutter", module = "opencamlib.ocl")]
pub struct PyBatchPushCutter {
    surf: Option<Py<PySTLSurf>>,
    cutter: Option<SharedCutter>,
    direction: Option<ocl::FiberDirection>,
    fibers: Vec<ocl::Fiber>,
    threads: usize,
    bucket_size: usize,
    calls: usize,
}

impl PyBatchPushCutter {
    fn direction(&self) -> PyResult<ocl::FiberDirection> {
        self.direction.ok_or_else(|| {
            PyRuntimeError::new_err("setXDirection() or setYDirection() must be called before run()")
        })
    }
}

#[pymethods]
impl PyBatchPushCutter {
    #[new]
    fn new() -> Self {
        PyBatchPushCutter {
            surf: None,
            cutter: None,
            direction: None,
            fibers: Vec::new(),
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
    fn setXDirection(&mut self) {
        self.direction = Some(ocl::FiberDirection::X);
    }
    fn setYDirection(&mut self) {
        self.direction = Some(ocl::FiberDirection::Y);
    }
    fn appendFiber(&mut self, f: PyRef<PyFiber>) {
        self.fibers.push(f.f.clone());
    }
    fn getFibers(&self) -> Vec<PyFiber> {
        fibers(&self.fibers)
    }
    /// CL-points at both ends of every interval of every fiber.
    fn getCLPoints(&self, py: Python<'_>) -> PyResult<Vec<Py<PyCLPoint>>> {
        let mut out = Vec::new();
        for f in &self.fibers {
            for i in f.ints.iter().filter(|i| !i.empty()) {
                out.push(PyCLPoint::create(py, &ocl::CLPoint { pos: f.point(i.lower), cc: i.lower_cc })?);
                out.push(PyCLPoint::create(py, &ocl::CLPoint { pos: f.point(i.upper), cc: i.upper_cc })?);
            }
        }
        Ok(out)
    }
    fn getOverlapTriangles(&self, py: Python<'_>, f: PyRef<PyFiber>) -> PyResult<Vec<PyTriangle>> {
        let surf_ref = need(&self.surf, "setSTL()")?.bind(py).borrow();
        let cutter = need(&self.cutter, "setCutter()")?;
        let mut bpc = ocl::BatchPushCutter::new(&surf_ref.s, cutter.as_ref(), self.direction()?);
        bpc.set_bucket_size(self.bucket_size);
        Ok(bpc.get_overlap_triangles(&f.f).into_iter().map(|t| PyTriangle { t }).collect())
    }
    fn run(&mut self, py: Python<'_>) -> PyResult<()> {
        let direction = self.direction()?;
        let surf_ref = need(&self.surf, "setSTL()")?.bind(py).borrow();
        let surf = &surf_ref.s;
        let cutter = need(&self.cutter, "setCutter()")?.clone();
        let fibers = std::mem::take(&mut self.fibers);
        let (threads, bucket_size) = (self.threads, self.bucket_size);
        let (fibers, calls) = py.allow_threads(move || {
            let mut bpc = ocl::BatchPushCutter::new(surf, cutter.as_ref(), direction);
            bpc.set_threads(threads);
            bpc.set_bucket_size(bucket_size);
            for f in fibers {
                bpc.append_fiber(f);
            }
            bpc.run();
            (bpc.take_fibers(), bpc.get_calls())
        });
        self.fibers = fibers;
        self.calls = calls;
        Ok(())
    }
}

// ---------------------------------------------------------------- Waterline

/// Results of a waterline run.
#[derive(Default)]
struct WaterlineOutput {
    loops: Vec<Vec<ocl::Point>>,
    xfibers: Vec<ocl::Fiber>,
    yfibers: Vec<ocl::Fiber>,
}

/// Waterline toolpath at constant z.
#[pyclass(name = "Waterline", module = "opencamlib.ocl")]
pub struct PyWaterline {
    surf: Option<Py<PySTLSurf>>,
    cutter: Option<SharedCutter>,
    z: f64,
    sampling: f64,
    threads: usize,
    out: WaterlineOutput,
}

impl PyWaterline {
    fn compute(&mut self, py: Python<'_>, smart: bool) -> PyResult<()> {
        let surf_ref = need(&self.surf, "setSTL()")?.bind(py).borrow();
        let surf = &surf_ref.s;
        let cutter = need(&self.cutter, "setCutter()")?.clone();
        let (z, sampling, threads) = (self.z, self.sampling, self.threads);
        self.out = py.allow_threads(move || {
            let mut wl = ocl::Waterline::new(surf, cutter.as_ref());
            wl.set_z(z);
            wl.set_sampling(sampling);
            wl.set_threads(threads);
            if smart {
                wl.run2();
            } else {
                wl.run();
            }
            WaterlineOutput {
                loops: wl.get_loops().to_vec(),
                xfibers: wl.get_x_fibers().to_vec(),
                yfibers: wl.get_y_fibers().to_vec(),
            }
        });
        Ok(())
    }
}

#[pymethods]
impl PyWaterline {
    #[new]
    fn new() -> Self {
        PyWaterline {
            surf: None,
            cutter: None,
            z: 0.0,
            sampling: 1.0,
            threads: ocl::max_threads(),
            out: WaterlineOutput::default(),
        }
    }
    fn setSTL(&mut self, s: Py<PySTLSurf>) {
        self.surf = Some(s);
    }
    fn setCutter(&mut self, c: PyRef<PyMillingCutter>) {
        self.cutter = Some(c.c.clone());
    }
    fn setZ(&mut self, z: f64) {
        self.z = z;
    }
    fn setSampling(&mut self, s: f64) -> PyResult<()> {
        positive(s)?;
        self.sampling = s;
        Ok(())
    }
    fn setThreads(&mut self, n: usize) {
        self.threads = n.max(1);
    }
    fn getThreads(&self) -> usize {
        self.threads
    }
    /// Compute the waterline using the simple weave.
    fn run(&mut self, py: Python<'_>) -> PyResult<()> {
        self.compute(py, false)
    }
    /// Compute the waterline using the smart weave.
    fn run2(&mut self, py: Python<'_>) -> PyResult<()> {
        self.compute(py, true)
    }
    fn reset(&mut self) {
        self.out = WaterlineOutput::default();
    }
    fn getLoops(&self) -> Vec<Vec<PyPoint>> {
        loops(&self.out.loops)
    }
    fn getXFibers(&self) -> Vec<PyFiber> {
        fibers(&self.out.xfibers)
    }
    fn getYFibers(&self) -> Vec<PyFiber> {
        fibers(&self.out.yfibers)
    }
}

/// Waterline with adaptive fiber spacing.
#[pyclass(name = "AdaptiveWaterline", module = "opencamlib.ocl")]
pub struct PyAdaptiveWaterline {
    surf: Option<Py<PySTLSurf>>,
    cutter: Option<SharedCutter>,
    z: f64,
    sampling: f64,
    min_sampling: f64,
    threads: usize,
    out: WaterlineOutput,
}

impl PyAdaptiveWaterline {
    fn compute(&mut self, py: Python<'_>, smart: bool) -> PyResult<()> {
        let surf_ref = need(&self.surf, "setSTL()")?.bind(py).borrow();
        let surf = &surf_ref.s;
        let cutter = need(&self.cutter, "setCutter()")?.clone();
        let (z, sampling, min_sampling, threads) = (self.z, self.sampling, self.min_sampling, self.threads);
        self.out = py.allow_threads(move || {
            let mut wl = ocl::AdaptiveWaterline::new(surf, cutter.as_ref());
            wl.set_z(z);
            wl.set_sampling(sampling);
            wl.set_min_sampling(min_sampling);
            wl.set_threads(threads);
            if smart {
                wl.run2();
            } else {
                wl.run();
            }
            WaterlineOutput {
                loops: wl.get_loops().to_vec(),
                xfibers: wl.get_x_fibers().to_vec(),
                yfibers: wl.get_y_fibers().to_vec(),
            }
        });
        Ok(())
    }
}

#[pymethods]
impl PyAdaptiveWaterline {
    #[new]
    fn new() -> Self {
        PyAdaptiveWaterline {
            surf: None,
            cutter: None,
            z: 0.0,
            sampling: 1.0,
            min_sampling: 0.1,
            threads: ocl::max_threads(),
            out: WaterlineOutput::default(),
        }
    }
    fn setSTL(&mut self, s: Py<PySTLSurf>) {
        self.surf = Some(s);
    }
    fn setCutter(&mut self, c: PyRef<PyMillingCutter>) {
        self.cutter = Some(c.c.clone());
    }
    fn setZ(&mut self, z: f64) {
        self.z = z;
    }
    /// Maximum fiber spacing. Also sets the minimum spacing to `s / 10`.
    fn setSampling(&mut self, s: f64) -> PyResult<()> {
        positive(s)?;
        self.sampling = s;
        self.min_sampling = s / 10.0;
        Ok(())
    }
    fn setMinSampling(&mut self, s: f64) -> PyResult<()> {
        positive(s)?;
        self.min_sampling = s;
        Ok(())
    }
    fn setThreads(&mut self, n: usize) {
        self.threads = n.max(1);
    }
    fn getThreads(&self) -> usize {
        self.threads
    }
    fn run(&mut self, py: Python<'_>) -> PyResult<()> {
        self.compute(py, false)
    }
    fn run2(&mut self, py: Python<'_>) -> PyResult<()> {
        self.compute(py, true)
    }
    fn reset(&mut self) {
        self.out = WaterlineOutput::default();
    }
    fn getLoops(&self) -> Vec<Vec<PyPoint>> {
        loops(&self.out.loops)
    }
    fn getXFibers(&self) -> Vec<PyFiber> {
        fibers(&self.out.xfibers)
    }
    fn getYFibers(&self) -> Vec<PyFiber> {
        fibers(&self.out.yfibers)
    }
}

fn positive(s: f64) -> PyResult<()> {
    if s > 0.0 {
        Ok(())
    } else {
        Err(pyo3::exceptions::PyValueError::new_err(format!("sampling must be positive, got {s}")))
    }
}

// ---------------------------------------------------------------- Weave

/// Type of a weave vertex.
#[pyclass(name = "WeaveVertexType", module = "opencamlib.ocl", eq, eq_int, hash, frozen)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
#[allow(non_camel_case_types, clippy::upper_case_acronyms)]
pub enum PyWeaveVertexType {
    CL = 0,
    CL_DONE = 1,
    ADJ = 2,
    TWOADJ = 3,
    INT = 4,
    FULLINT = 5,
}

impl From<PyWeaveVertexType> for ocl::WeaveVertexType {
    fn from(t: PyWeaveVertexType) -> Self {
        use ocl::WeaveVertexType as W;
        match t {
            PyWeaveVertexType::CL => W::CL,
            PyWeaveVertexType::CL_DONE => W::CLDone,
            PyWeaveVertexType::ADJ => W::Adj,
            PyWeaveVertexType::TWOADJ => W::TwoAdj,
            PyWeaveVertexType::INT => W::Int,
            PyWeaveVertexType::FULLINT => W::FullInt,
        }
    }
}

#[pymethods]
impl PyWeaveVertexType {
    #[getter]
    fn name(&self) -> &'static str {
        ["CL", "CL_DONE", "ADJ", "TWOADJ", "INT", "FULLINT"][*self as usize]
    }
    #[getter]
    fn value(&self) -> u8 {
        *self as u8
    }
    fn __repr__(&self) -> String {
        format!("<WeaveVertexType.{}: {}>", self.name(), *self as u8)
    }
}

/// Weave graph built from X- and Y-fibers. Use SimpleWeave or SmartWeave.
#[pyclass(name = "Weave", module = "opencamlib.ocl", subclass)]
pub struct PyWeave {
    w: ocl::Weave,
    smart: bool,
}

#[pymethods]
impl PyWeave {
    fn addFiber(&mut self, f: PyRef<PyFiber>) {
        self.w.add_fiber(f.f.clone());
    }
    fn build(&mut self, py: Python<'_>) {
        let (w, smart) = (&mut self.w, self.smart);
        py.allow_threads(move || if smart { w.build_smart() } else { w.build() });
    }
    fn face_traverse(&mut self, py: Python<'_>) {
        let w = &mut self.w;
        py.allow_threads(move || w.face_traverse());
    }
    fn printGraph(&self) {
        let cl = self.w.get_cl_vertices().len();
        let n = self.w.num_vertices();
        println!(" number of vertices: {n}");
        println!(" number of edges: {}", self.w.num_edges());
        println!(" counted {n} vertices");
        println!("          CL-nodes: {cl}");
        println!("    internal-nodes: {}", n - cl);
    }
    fn getVertices(&self) -> Vec<PyPoint> {
        points(&self.w.get_vertices())
    }
    fn getVerticesByType(&self, t: PyWeaveVertexType) -> Vec<PyPoint> {
        points(&self.w.get_vertices_by_type(t.into()))
    }
    fn getCLVertices(&self) -> Vec<PyPoint> {
        points(&self.w.get_cl_vertices())
    }
    fn getINTVertices(&self) -> Vec<PyPoint> {
        points(&self.w.get_int_vertices())
    }
    fn numVertices(&self) -> usize {
        self.w.num_vertices()
    }
    fn getEdges(&self) -> Vec<(PyPoint, PyPoint)> {
        edges(self.w.get_edges())
    }
    fn getLoops(&self) -> Vec<Vec<PyPoint>> {
        loops(&self.w.get_loops())
    }
    fn __str__(&self) -> String {
        self.w.to_string()
    }
}

/// Weave that adds a vertex at every fiber crossing.
#[pyclass(name = "SimpleWeave", module = "opencamlib.ocl", extends = PyWeave)]
pub struct PySimpleWeave;

#[pymethods]
impl PySimpleWeave {
    #[new]
    fn new() -> (Self, PyWeave) {
        (PySimpleWeave, PyWeave { w: ocl::Weave::new(), smart: false })
    }
}

/// Weave that only adds the crossings needed to describe the loops.
#[pyclass(name = "SmartWeave", module = "opencamlib.ocl", extends = PyWeave)]
pub struct PySmartWeave;

#[pymethods]
impl PySmartWeave {
    #[new]
    fn new() -> (Self, PyWeave) {
        (PySmartWeave, PyWeave { w: ocl::Weave::new(), smart: true })
    }
}

// ---------------------------------------------------------------- LineCLFilter, CLS, TSP

/// Removes CL-points that are collinear (within a tolerance) with their neighbours.
#[pyclass(name = "LineCLFilter", module = "opencamlib.ocl")]
#[derive(Default)]
pub struct PyLineCLFilter {
    f: ocl::LineCLFilter,
}

#[pymethods]
impl PyLineCLFilter {
    #[new]
    fn new() -> Self {
        PyLineCLFilter::default()
    }
    fn addCLPoint(&mut self, p: PyRef<PyCLPoint>) {
        self.f.add_cl_point(PyCLPoint::get_ref(&p));
    }
    fn setTolerance(&mut self, tol: f64) {
        self.f.set_tolerance(tol);
    }
    fn run(&mut self) {
        self.f.run();
    }
    fn getCLPoints(&self, py: Python<'_>) -> PyResult<Vec<Py<PyCLPoint>>> {
        PyCLPoint::create_all(py, self.f.get_cl_points())
    }
}

/// Experimental cutter-location surface (`run()` does nothing, as in the C++ library).
#[pyclass(name = "CutterLocationSurface", module = "opencamlib.ocl")]
pub struct PyCutterLocationSurface {
    cls: ocl::CutterLocationSurface,
    surf: Option<Py<PySTLSurf>>,
    cutter: Option<SharedCutter>,
    sampling: f64,
}

#[pymethods]
impl PyCutterLocationSurface {
    #[new]
    fn new(far: f64) -> Self {
        PyCutterLocationSurface {
            cls: ocl::CutterLocationSurface::new(far),
            surf: None,
            cutter: None,
            sampling: 1.0,
        }
    }
    fn run(&mut self) {
        self.cls.run();
    }
    fn setMinSampling(&mut self, s: f64) {
        self.cls.set_min_sampling(s);
    }
    fn setSampling(&mut self, s: f64) {
        self.sampling = s;
    }
    fn setSTL(&mut self, s: Py<PySTLSurf>) {
        self.surf = Some(s);
    }
    fn setCutter(&mut self, c: PyRef<PyMillingCutter>) {
        self.cutter = Some(c.c.clone());
    }
    fn getVertices(&self) -> Vec<PyPoint> {
        points(&self.cls.get_vertices())
    }
    fn getEdges(&self) -> Vec<(PyPoint, PyPoint)> {
        edges(self.cls.get_edges())
    }
    fn __str__(&self) -> String {
        self.cls.to_string()
    }
}

/// Approximate travelling-salesman tour through 2D points.
#[pyclass(name = "TSPSolver", module = "opencamlib.ocl")]
#[derive(Default)]
pub struct PyTSPSolver {
    t: ocl::TSPSolver,
}

#[pymethods]
impl PyTSPSolver {
    #[new]
    fn new() -> Self {
        PyTSPSolver::default()
    }
    fn addPoint(&mut self, x: f64, y: f64) {
        self.t.add_point(x, y);
    }
    fn run(&mut self, py: Python<'_>) {
        let t = &mut self.t;
        py.allow_threads(move || t.run());
    }
    fn getOutput(&self) -> Vec<usize> {
        self.t.get_output().to_vec()
    }
    fn getLength(&self) -> f64 {
        self.t.get_length()
    }
    fn reset(&mut self) {
        self.t.reset();
    }
}
