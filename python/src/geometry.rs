//! Geometry classes: Point, CCPoint, CLPoint, Triangle, Bbox, STLSurf, STLReader, Line,
//! Arc, Path, Interval, Fiber, Ellipse, EllipsePosition (ocl_geometry.cpp).
#![allow(non_snake_case)]

use pyo3::exceptions::{PyIOError, PyTypeError};
use pyo3::prelude::*;
use pyo3::types::{PyList, PyTuple};

use opencamlib as ocl;
use opencamlib::geo::stlreader::read_stl_file;

use crate::util::g;

pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyPoint>()?;
    m.add_class::<PyCCType>()?;
    for t in PyCCType::ALL {
        m.add(t.name(), t)?;
    }
    m.add_class::<PyCCPoint>()?;
    m.add_class::<PyCLPoint>()?;
    m.add_class::<PyTriangle>()?;
    m.add_class::<PyBbox>()?;
    m.add_class::<PySTLSurf>()?;
    m.add_class::<PySTLReader>()?;
    m.add_class::<PyEllipsePosition>()?;
    m.add_class::<PyEllipse>()?;
    m.add_class::<PyLine>()?;
    m.add_class::<PyArc>()?;
    m.add_class::<PySpanType>()?;
    m.add("LineSpanType", PySpanType::LineSpanType)?;
    m.add("ArcSpanType", PySpanType::ArcSpanType)?;
    m.add_class::<PyPath>()?;
    m.add_class::<PyInterval>()?;
    m.add_class::<PyFiber>()?;
    Ok(())
}

pub fn point_str(p: &ocl::Point) -> String {
    format!("({}, {}, {})", g(p.x), g(p.y), g(p.z))
}

fn cc_str(c: &ocl::CCPoint) -> String {
    format!("CC({}, {}, {}, t={})", g(c.x), g(c.y), g(c.z), c.cc_type as u8)
}

// ---------------------------------------------------------------- Point

/// A point or vector in 3D space.
#[pyclass(name = "Point", module = "opencamlib.ocl", subclass)]
#[derive(Clone)]
pub struct PyPoint {
    pub p: ocl::Point,
}

impl From<ocl::Point> for PyPoint {
    fn from(p: ocl::Point) -> Self {
        PyPoint { p }
    }
}

#[pymethods]
impl PyPoint {
    /// Point(), Point(x, y, z), Point(x, y) or Point(other)
    #[new]
    #[pyo3(signature = (*args))]
    fn new(args: &Bound<'_, PyTuple>) -> PyResult<Self> {
        let p = match args.len() {
            0 => ocl::Point::default(),
            1 => args.get_item(0)?.extract::<PyRef<PyPoint>>()?.p,
            2 => ocl::Point::new_xy(args.get_item(0)?.extract()?, args.get_item(1)?.extract()?),
            3 => ocl::Point::new(
                args.get_item(0)?.extract()?,
                args.get_item(1)?.extract()?,
                args.get_item(2)?.extract()?,
            ),
            _ => return Err(PyTypeError::new_err("Point() takes 0, 1, 2 or 3 arguments")),
        };
        Ok(PyPoint { p })
    }

    #[getter]
    fn x(&self) -> f64 {
        self.p.x
    }
    #[setter]
    fn set_x(&mut self, v: f64) {
        self.p.x = v;
    }
    #[getter]
    fn y(&self) -> f64 {
        self.p.y
    }
    #[setter]
    fn set_y(&mut self, v: f64) {
        self.p.y = v;
    }
    #[getter]
    fn z(&self) -> f64 {
        self.p.z
    }
    #[setter]
    fn set_z(&mut self, v: f64) {
        self.p.z = v;
    }

    fn __add__(&self, other: PyRef<PyPoint>) -> PyPoint {
        (self.p + other.p).into()
    }
    fn __sub__(&self, other: PyRef<PyPoint>) -> PyPoint {
        (self.p - other.p).into()
    }
    fn __iadd__(&mut self, other: PyRef<PyPoint>) {
        self.p += other.p;
    }
    fn __isub__(&mut self, other: PyRef<PyPoint>) {
        self.p -= other.p;
    }
    fn __mul__(&self, a: f64) -> PyPoint {
        (self.p * a).into()
    }
    fn __rmul__(&self, a: f64) -> PyPoint {
        (a * self.p).into()
    }

    fn norm(&self) -> f64 {
        self.p.norm()
    }
    fn xyNorm(&self) -> f64 {
        self.p.xy_norm()
    }
    fn normalize(&mut self) {
        self.p.normalize();
    }
    fn dot(&self, other: PyRef<PyPoint>) -> f64 {
        self.p.dot(&other.p)
    }
    fn cross(&self, other: PyRef<PyPoint>) -> PyPoint {
        self.p.cross(&other.p).into()
    }
    fn xRotate(&mut self, theta: f64) {
        self.p.x_rotate(theta);
    }
    fn yRotate(&mut self, theta: f64) {
        self.p.y_rotate(theta);
    }
    fn zRotate(&mut self, theta: f64) {
        self.p.z_rotate(theta);
    }
    fn isRight(&self, p1: PyRef<PyPoint>, p2: PyRef<PyPoint>) -> bool {
        self.p.is_right(&p1.p, &p2.p)
    }
    /// isInside(triangle), or isInside(p1, p2) for the segment p1-p2
    #[pyo3(signature = (a, b = None))]
    fn isInside(&self, a: &Bound<'_, PyAny>, b: Option<PyRef<PyPoint>>) -> PyResult<bool> {
        match b {
            Some(p2) => Ok(self.p.is_inside_segment(&a.extract::<PyRef<PyPoint>>()?.p, &p2.p)),
            None => Ok(self.p.is_inside_triangle(&a.extract::<PyRef<PyTriangle>>()?.t)),
        }
    }
    fn xyDistance(&self, other: PyRef<PyPoint>) -> f64 {
        self.p.xy_distance(&other.p)
    }
    fn __str__(&self) -> String {
        point_str(&self.p)
    }
    fn __repr__(&self) -> String {
        format!("Point{}", point_str(&self.p))
    }
}

// ---------------------------------------------------------------- CCType

/// Type of cutter-contact point.
#[pyclass(name = "CCType", module = "opencamlib.ocl", eq, eq_int, hash, frozen)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
#[allow(non_camel_case_types, clippy::upper_case_acronyms)]
pub enum PyCCType {
    NONE = 0,
    VERTEX = 1,
    VERTEX_CYL = 2,
    EDGE = 3,
    EDGE_HORIZ = 4,
    EDGE_SHAFT = 5,
    EDGE_HORIZ_CYL = 6,
    EDGE_HORIZ_TOR = 7,
    EDGE_BALL = 8,
    EDGE_POS = 9,
    EDGE_NEG = 10,
    EDGE_CYL = 11,
    EDGE_CONE = 12,
    EDGE_CONE_BASE = 13,
    FACET = 14,
    FACET_TIP = 15,
    FACET_CYL = 16,
    ERROR = 17,
}

impl PyCCType {
    const ALL: [PyCCType; 18] = {
        use PyCCType::*;
        [
            NONE, VERTEX, VERTEX_CYL, EDGE, EDGE_HORIZ, EDGE_SHAFT, EDGE_HORIZ_CYL, EDGE_HORIZ_TOR,
            EDGE_BALL, EDGE_POS, EDGE_NEG, EDGE_CYL, EDGE_CONE, EDGE_CONE_BASE, FACET, FACET_TIP,
            FACET_CYL, ERROR,
        ]
    };
    const NAMES: [&'static str; 18] = [
        "NONE", "VERTEX", "VERTEX_CYL", "EDGE", "EDGE_HORIZ", "EDGE_SHAFT", "EDGE_HORIZ_CYL",
        "EDGE_HORIZ_TOR", "EDGE_BALL", "EDGE_POS", "EDGE_NEG", "EDGE_CYL", "EDGE_CONE",
        "EDGE_CONE_BASE", "FACET", "FACET_TIP", "FACET_CYL", "ERROR",
    ];
}

#[pymethods]
impl PyCCType {
    #[getter]
    fn name(&self) -> &'static str {
        Self::NAMES[*self as usize]
    }
    #[getter]
    fn value(&self) -> u8 {
        *self as u8
    }
    fn __repr__(&self) -> String {
        format!("<CCType.{}: {}>", self.name(), *self as u8)
    }
    fn __str__(&self) -> String {
        format!("CCType.{}", self.name())
    }
}

impl From<ocl::CCType> for PyCCType {
    fn from(t: ocl::CCType) -> Self {
        PyCCType::ALL[t as usize]
    }
}

impl From<PyCCType> for ocl::CCType {
    fn from(t: PyCCType) -> Self {
        use ocl::CCType::*;
        [
            None, Vertex, VertexCyl, Edge, EdgeHoriz, EdgeShaft, EdgeHorizCyl, EdgeHorizTor,
            EdgeBall, EdgePos, EdgeNeg, EdgeCyl, EdgeCone, EdgeConeBase, Facet, FacetTip, FacetCyl,
            Error,
        ][t as usize]
    }
}

// ---------------------------------------------------------------- CCPoint / CLPoint

/// Cutter-contact point: a Point with a CCType.
#[pyclass(name = "CCPoint", module = "opencamlib.ocl", extends = PyPoint)]
pub struct PyCCPoint {
    pub cc_type: PyCCType,
}

impl PyCCPoint {
    pub fn create(py: Python<'_>, cc: &ocl::CCPoint) -> PyResult<Py<PyCCPoint>> {
        Py::new(
            py,
            PyClassInitializer::from(PyPoint::from(cc.pos)).add_subclass(PyCCPoint {
                cc_type: cc.cc_type.into(),
            }),
        )
    }

    pub fn get(r: &PyRef<'_, PyCCPoint>) -> ocl::CCPoint {
        ocl::CCPoint::new(r.as_super().p, r.cc_type.into())
    }
}

#[pymethods]
impl PyCCPoint {
    /// CCPoint(), CCPoint(other) or CCPoint(x, y, z)
    #[new]
    #[pyo3(signature = (*args))]
    fn new(args: &Bound<'_, PyTuple>) -> PyResult<(Self, PyPoint)> {
        let cc = match args.len() {
            0 => ocl::CCPoint::default(),
            1 => PyCCPoint::get(&args.get_item(0)?.extract::<PyRef<PyCCPoint>>()?),
            3 => ocl::CCPoint::from_xyz(
                args.get_item(0)?.extract()?,
                args.get_item(1)?.extract()?,
                args.get_item(2)?.extract()?,
                ocl::CCType::None,
            ),
            _ => return Err(PyTypeError::new_err("CCPoint() takes 0, 1 or 3 arguments")),
        };
        Ok((PyCCPoint { cc_type: cc.cc_type.into() }, cc.pos.into()))
    }

    #[getter]
    fn get_type(&self) -> PyCCType {
        self.cc_type
    }
    #[setter]
    fn set_type(&mut self, t: PyCCType) {
        self.cc_type = t;
    }

    fn __str__(slf: PyRef<'_, Self>) -> String {
        cc_str(&PyCCPoint::get(&slf))
    }
    fn __repr__(slf: PyRef<'_, Self>) -> String {
        cc_str(&PyCCPoint::get(&slf))
    }
}

/// Cutter-location point, with the corresponding cutter-contact point.
#[pyclass(name = "CLPoint", module = "opencamlib.ocl", extends = PyPoint)]
pub struct PyCLPoint {
    pub cc: ocl::CCPoint,
}

impl PyCLPoint {
    pub fn create(py: Python<'_>, cl: &ocl::CLPoint) -> PyResult<Py<PyCLPoint>> {
        Py::new(
            py,
            PyClassInitializer::from(PyPoint::from(cl.pos)).add_subclass(PyCLPoint { cc: cl.cc }),
        )
    }

    pub fn create_all(py: Python<'_>, pts: &[ocl::CLPoint]) -> PyResult<Vec<Py<PyCLPoint>>> {
        pts.iter().map(|cl| PyCLPoint::create(py, cl)).collect()
    }

    pub fn get_ref(r: &PyRef<'_, PyCLPoint>) -> ocl::CLPoint {
        ocl::CLPoint { pos: r.as_super().p, cc: r.cc }
    }

    pub fn get(obj: &Bound<'_, PyCLPoint>) -> ocl::CLPoint {
        PyCLPoint::get_ref(&obj.borrow())
    }

    /// Write `cl` back into the Python object (for in-place drop-cutter calls).
    pub fn set(obj: &Bound<'_, PyCLPoint>, cl: &ocl::CLPoint) {
        let mut r = obj.borrow_mut();
        r.cc = cl.cc;
        r.as_super().p = cl.pos;
    }
}

#[pymethods]
impl PyCLPoint {
    /// CLPoint(), CLPoint(other), CLPoint(x, y, z) or CLPoint(x, y, z, cc)
    #[new]
    #[pyo3(signature = (*args))]
    fn new(args: &Bound<'_, PyTuple>) -> PyResult<(Self, PyPoint)> {
        let xyz = |args: &Bound<'_, PyTuple>| -> PyResult<ocl::Point> {
            Ok(ocl::Point::new(
                args.get_item(0)?.extract()?,
                args.get_item(1)?.extract()?,
                args.get_item(2)?.extract()?,
            ))
        };
        let cl = match args.len() {
            0 => ocl::CLPoint::default(),
            1 => PyCLPoint::get_ref(&args.get_item(0)?.extract::<PyRef<PyCLPoint>>()?),
            3 => ocl::CLPoint::from(xyz(args)?),
            4 => ocl::CLPoint {
                pos: xyz(args)?,
                cc: PyCCPoint::get(&args.get_item(3)?.extract::<PyRef<PyCCPoint>>()?),
            },
            _ => return Err(PyTypeError::new_err("CLPoint() takes 0, 1, 3 or 4 arguments")),
        };
        Ok((PyCLPoint { cc: cl.cc }, cl.pos.into()))
    }

    /// The cutter-contact point.
    fn cc(&self, py: Python<'_>) -> PyResult<Py<PyCCPoint>> {
        PyCCPoint::create(py, &self.cc)
    }

    /// The cutter-contact point.
    fn getCC(&self, py: Python<'_>) -> PyResult<Py<PyCCPoint>> {
        PyCCPoint::create(py, &self.cc)
    }

    fn __str__(slf: PyRef<'_, Self>) -> String {
        let cl = PyCLPoint::get_ref(&slf);
        format!("CL({}, {}, {}) cc={}", g(cl.x), g(cl.y), g(cl.z), cc_str(&cl.cc))
    }
    fn __repr__(slf: PyRef<'_, Self>) -> String {
        Self::__str__(slf)
    }
}

// ---------------------------------------------------------------- Triangle, Bbox, STLSurf

/// A triangle defined by its three vertices.
#[pyclass(name = "Triangle", module = "opencamlib.ocl")]
#[derive(Clone)]
pub struct PyTriangle {
    pub t: ocl::Triangle,
}

#[pymethods]
impl PyTriangle {
    #[new]
    fn new(p1: PyRef<PyPoint>, p2: PyRef<PyPoint>, p3: PyRef<PyPoint>) -> Self {
        PyTriangle { t: ocl::Triangle::new(p1.p, p2.p, p3.p) }
    }
    fn getPoints(&self) -> Vec<PyPoint> {
        self.t.p.iter().map(|&p| p.into()).collect()
    }
    #[getter]
    fn p(&self) -> Vec<PyPoint> {
        self.getPoints()
    }
    #[getter]
    fn n(&self) -> PyPoint {
        self.t.n.into()
    }
    fn __str__(&self) -> String {
        format!(
            "T: {} {} {}n={}",
            point_str(&self.t.p[0]),
            point_str(&self.t.p[1]),
            point_str(&self.t.p[2]),
            point_str(&self.t.n)
        )
    }
}

/// Axis-aligned bounding box.
#[pyclass(name = "Bbox", module = "opencamlib.ocl")]
#[derive(Clone)]
pub struct PyBbox {
    pub bb: ocl::Bbox,
}

#[pymethods]
impl PyBbox {
    fn isInside(&self, p: PyRef<PyPoint>) -> bool {
        self.bb.is_inside(&p.p)
    }
    #[getter]
    fn maxpt(&self) -> PyPoint {
        self.bb.maxpt.into()
    }
    #[getter]
    fn minpt(&self) -> PyPoint {
        self.bb.minpt.into()
    }
    fn __str__(&self) -> String {
        format!(
            " Bbox \n min= {}\n max= {}\n",
            point_str(&self.bb.minpt),
            point_str(&self.bb.maxpt)
        )
    }
}

/// A triangulated surface.
#[pyclass(name = "STLSurf", module = "opencamlib.ocl")]
#[derive(Default)]
pub struct PySTLSurf {
    pub s: ocl::STLSurf,
}

#[pymethods]
impl PySTLSurf {
    #[new]
    fn new() -> Self {
        PySTLSurf::default()
    }
    fn addTriangle(&mut self, t: PyRef<PyTriangle>) {
        self.s.add_triangle(t.t);
    }
    fn size(&self) -> usize {
        self.s.size()
    }
    fn rotate(&mut self, xr: f64, yr: f64, zr: f64) {
        self.s.rotate(xr, yr, zr);
    }
    /// [minx, maxx, miny, maxy, minz, maxz]
    fn getBounds(&self) -> Vec<f64> {
        self.s.get_bounds().to_vec()
    }
    fn getTriangles(&self) -> Vec<PyTriangle> {
        self.s.tris.iter().map(|&t| PyTriangle { t }).collect()
    }
    #[getter]
    fn tris(&self) -> Vec<PyTriangle> {
        self.getTriangles()
    }
    #[getter]
    fn bb(&self) -> PyBbox {
        PyBbox { bb: self.s.bb }
    }
    fn __str__(&self) -> String {
        format!("STLSurf(N={})", self.s.size())
    }
}

/// STLReader(path, surf) reads an ASCII or binary STL file into `surf`.
#[pyclass(name = "STLReader", module = "opencamlib.ocl")]
pub struct PySTLReader;

#[pymethods]
impl PySTLReader {
    #[new]
    fn new(path: &str, surf: &Bound<'_, PySTLSurf>) -> PyResult<Self> {
        read_stl_file(path, &mut surf.borrow_mut().s)
            .map_err(|e| PyIOError::new_err(format!("{path}: {e}")))?;
        Ok(PySTLReader)
    }
}

// ---------------------------------------------------------------- Ellipse

/// A position (s, t) on an ellipse.
#[pyclass(name = "EllipsePosition", module = "opencamlib.ocl")]
#[derive(Clone, Default)]
pub struct PyEllipsePosition {
    pub e: ocl::EllipsePosition,
}

#[pymethods]
impl PyEllipsePosition {
    #[new]
    fn new() -> Self {
        PyEllipsePosition::default()
    }
    #[getter]
    fn s(&self) -> f64 {
        self.e.s
    }
    #[setter]
    fn set_s(&mut self, v: f64) {
        self.e.s = v;
    }
    #[getter]
    fn t(&self) -> f64 {
        self.e.t
    }
    #[setter]
    fn set_t(&mut self, v: f64) {
        self.e.t = v;
    }
    fn setDiangle(&mut self, d: f64) {
        self.e.set_diangle(d);
    }
    fn __str__(&self) -> String {
        format!("({} ,{})", g(self.e.s), g(self.e.t))
    }
}

/// An ellipse with an offset-ellipse.
#[pyclass(name = "Ellipse", module = "opencamlib.ocl")]
pub struct PyEllipse {
    pub e: ocl::Ellipse,
}

#[pymethods]
impl PyEllipse {
    #[new]
    fn new(center: PyRef<PyPoint>, a: f64, b: f64, offset: f64) -> Self {
        PyEllipse { e: ocl::Ellipse::new(center.p, a, b, offset) }
    }
    fn ePoint(&self, pos: PyRef<PyEllipsePosition>) -> PyPoint {
        self.e.e_point(&pos.e).into()
    }
    fn oePoint(&self, pos: PyRef<PyEllipsePosition>) -> PyPoint {
        self.e.oe_point(&pos.e).into()
    }
    fn normal(&self, pos: PyRef<PyEllipsePosition>) -> PyPoint {
        self.e.normal(&pos.e).into()
    }
}

// ---------------------------------------------------------------- Line, Arc, Path

/// A line segment.
#[pyclass(name = "Line", module = "opencamlib.ocl")]
#[derive(Clone)]
pub struct PyLine {
    pub l: ocl::Line,
}

#[pymethods]
impl PyLine {
    /// Line(p1, p2) or Line(other)
    #[new]
    #[pyo3(signature = (a, b = None))]
    fn new(a: &Bound<'_, PyAny>, b: Option<PyRef<PyPoint>>) -> PyResult<Self> {
        Ok(match b {
            Some(p2) => PyLine { l: ocl::Line::new(a.extract::<PyRef<PyPoint>>()?.p, p2.p) },
            None => a.extract::<PyRef<PyLine>>()?.clone(),
        })
    }
    #[getter]
    fn p1(&self) -> PyPoint {
        self.l.p1.into()
    }
    #[setter]
    fn set_p1(&mut self, p: PyRef<PyPoint>) {
        self.l.p1 = p.p;
    }
    #[getter]
    fn p2(&self) -> PyPoint {
        self.l.p2.into()
    }
    #[setter]
    fn set_p2(&mut self, p: PyRef<PyPoint>) {
        self.l.p2 = p.p;
    }
    fn __str__(&self) -> String {
        format!("({}, {})", point_str(&self.l.p1), point_str(&self.l.p2))
    }
}

/// A circular arc in the XY plane; `dir` is True for counter-clockwise.
#[pyclass(name = "Arc", module = "opencamlib.ocl")]
#[derive(Clone)]
pub struct PyArc {
    pub a: ocl::Arc,
}

impl PyArc {
    /// Rebuild the arc so its cached length and radius follow the new end points.
    fn update(&mut self, f: impl FnOnce(&mut ocl::Arc)) {
        let mut a = self.a;
        f(&mut a);
        self.a = ocl::Arc::new(a.p1, a.p2, a.c, a.dir);
    }
}

#[pymethods]
impl PyArc {
    /// Arc(p1, p2, c, dir) or Arc(other)
    #[new]
    #[pyo3(signature = (*args))]
    fn new(args: &Bound<'_, PyTuple>) -> PyResult<Self> {
        match args.len() {
            1 => Ok(args.get_item(0)?.extract::<PyRef<PyArc>>()?.clone()),
            4 => Ok(PyArc {
                a: ocl::Arc::new(
                    args.get_item(0)?.extract::<PyRef<PyPoint>>()?.p,
                    args.get_item(1)?.extract::<PyRef<PyPoint>>()?.p,
                    args.get_item(2)?.extract::<PyRef<PyPoint>>()?.p,
                    args.get_item(3)?.extract()?,
                ),
            }),
            _ => Err(PyTypeError::new_err("Arc() takes 1 or 4 arguments")),
        }
    }
    #[getter]
    fn p1(&self) -> PyPoint {
        self.a.p1.into()
    }
    #[setter]
    fn set_p1(&mut self, p: PyRef<PyPoint>) {
        self.update(|a| a.p1 = p.p);
    }
    #[getter]
    fn p2(&self) -> PyPoint {
        self.a.p2.into()
    }
    #[setter]
    fn set_p2(&mut self, p: PyRef<PyPoint>) {
        self.update(|a| a.p2 = p.p);
    }
    #[getter]
    fn c(&self) -> PyPoint {
        self.a.c.into()
    }
    #[setter]
    fn set_c(&mut self, p: PyRef<PyPoint>) {
        self.update(|a| a.c = p.p);
    }
    #[getter]
    fn dir(&self) -> bool {
        self.a.dir
    }
    #[setter]
    fn set_dir(&mut self, d: bool) {
        self.update(|a| a.dir = d);
    }
    fn __str__(&self) -> String {
        format!(
            "({}, {}, {}, {})",
            point_str(&self.a.p1),
            point_str(&self.a.p2),
            point_str(&self.a.c),
            self.a.dir as u8
        )
    }
}

/// Type of a path span.
#[pyclass(name = "SpanType", module = "opencamlib.ocl", eq, eq_int, hash, frozen)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum PySpanType {
    LineSpanType = 0,
    ArcSpanType = 1,
}

#[pymethods]
impl PySpanType {
    #[getter]
    fn name(&self) -> &'static str {
        match self {
            PySpanType::LineSpanType => "LineSpanType",
            PySpanType::ArcSpanType => "ArcSpanType",
        }
    }
    #[getter]
    fn value(&self) -> u8 {
        *self as u8
    }
    fn __repr__(&self) -> String {
        format!("<SpanType.{}: {}>", self.name(), *self as u8)
    }
}

/// A path of lines and arcs.
#[pyclass(name = "Path", module = "opencamlib.ocl")]
#[derive(Clone, Default)]
pub struct PyPath {
    pub p: ocl::Path,
}

impl PyPath {
    fn span_object(py: Python<'_>, s: &ocl::Span) -> PyResult<PyObject> {
        Ok(match s {
            ocl::Span::Line(l) => Py::new(py, PyLine { l: *l })?.into_any(),
            ocl::Span::Arc(a) => Py::new(py, PyArc { a: *a })?.into_any(),
        })
    }
}

#[pymethods]
impl PyPath {
    /// Path() or Path(other)
    #[new]
    #[pyo3(signature = (other = None))]
    fn new(other: Option<PyRef<PyPath>>) -> Self {
        other.map(|o| o.clone()).unwrap_or_default()
    }
    /// Append a Line or an Arc.
    fn append(&mut self, span: &Bound<'_, PyAny>) -> PyResult<()> {
        if let Ok(l) = span.extract::<PyRef<PyLine>>() {
            self.p.append_line(l.l);
        } else if let Ok(a) = span.extract::<PyRef<PyArc>>() {
            self.p.append_arc(a.a);
        } else {
            return Err(PyTypeError::new_err("Path.append() takes a Line or an Arc"));
        }
        Ok(())
    }
    fn getSpans<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyList>> {
        let spans: Vec<PyObject> = self
            .p
            .span_list
            .iter()
            .map(|s| PyPath::span_object(py, s))
            .collect::<PyResult<_>>()?;
        PyList::new(py, spans)
    }
    fn getTypeSpanPairs<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyList>> {
        let pairs: Vec<(PySpanType, PyObject)> = self
            .p
            .span_list
            .iter()
            .map(|s| {
                let t = match s.span_type() {
                    ocl::SpanType::Line => PySpanType::LineSpanType,
                    ocl::SpanType::Arc => PySpanType::ArcSpanType,
                };
                Ok((t, PyPath::span_object(py, s)?))
            })
            .collect::<PyResult<_>>()?;
        PyList::new(py, pairs)
    }
}

// ---------------------------------------------------------------- Interval, Fiber

/// An interval [lower, upper] along a fiber, with cutter-contact points at both ends.
#[pyclass(name = "Interval", module = "opencamlib.ocl")]
#[derive(Clone, Default)]
pub struct PyInterval {
    pub i: ocl::Interval,
}

#[pymethods]
impl PyInterval {
    /// Interval() or Interval(lower, upper)
    #[new]
    #[pyo3(signature = (lower = None, upper = None))]
    fn new(lower: Option<f64>, upper: Option<f64>) -> PyResult<Self> {
        match (lower, upper) {
            (None, None) => Ok(PyInterval::default()),
            (Some(l), Some(u)) => Ok(PyInterval { i: ocl::Interval::from_bounds(l, u) }),
            _ => Err(PyTypeError::new_err("Interval() takes 0 or 2 arguments")),
        }
    }
    #[getter]
    fn upper(&self) -> f64 {
        self.i.upper
    }
    #[getter]
    fn lower(&self) -> f64 {
        self.i.lower
    }
    #[getter]
    fn upper_cc(&self, py: Python<'_>) -> PyResult<Py<PyCCPoint>> {
        PyCCPoint::create(py, &self.i.upper_cc)
    }
    #[getter]
    fn lower_cc(&self, py: Python<'_>) -> PyResult<Py<PyCCPoint>> {
        PyCCPoint::create(py, &self.i.lower_cc)
    }
    fn updateUpper(&mut self, t: f64, cc: PyRef<PyCCPoint>) {
        self.i.update_upper(t, &PyCCPoint::get(&cc));
    }
    fn updateLower(&mut self, t: f64, cc: PyRef<PyCCPoint>) {
        self.i.update_lower(t, &PyCCPoint::get(&cc));
    }
    fn empty(&self) -> bool {
        self.i.empty()
    }
    fn __str__(&self) -> String {
        format!("I [{} , {} ]", g(self.i.lower), g(self.i.upper))
    }
}

/// A fiber along which the push-cutter records contact intervals.
#[pyclass(name = "Fiber", module = "opencamlib.ocl")]
#[derive(Clone)]
pub struct PyFiber {
    pub f: ocl::Fiber,
}

#[pymethods]
impl PyFiber {
    #[new]
    fn new(p1: PyRef<PyPoint>, p2: PyRef<PyPoint>) -> Self {
        PyFiber { f: ocl::Fiber::new(p1.p, p2.p) }
    }
    #[getter]
    fn p1(&self) -> PyPoint {
        self.f.p1.into()
    }
    #[getter]
    fn p2(&self) -> PyPoint {
        self.f.p2.into()
    }
    #[getter]
    fn dir(&self) -> PyPoint {
        self.f.dir.into()
    }
    fn addInterval(&mut self, i: PyRef<PyInterval>) {
        self.f.add_interval(i.i);
    }
    fn point(&self, t: f64) -> PyPoint {
        self.f.point(t).into()
    }
    fn printInts(&self) {
        for (n, i) in self.f.ints.iter().enumerate() {
            println!("{}: [ {} , {} ]", n, g(i.lower), g(i.upper));
        }
    }
    fn getInts(&self) -> Vec<PyInterval> {
        self.f.ints.iter().map(|&i| PyInterval { i }).collect()
    }
    fn __str__(&self) -> String {
        format!(
            " fiber dir={} and {} intervals\n fiber.p1={} fiber.p2 {}",
            point_str(&self.f.dir),
            self.f.ints.len(),
            point_str(&self.f.p1),
            point_str(&self.f.p2)
        )
    }
}
