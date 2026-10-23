//! Cutter classes (ocl_cutters.cpp).
#![allow(non_snake_case)]

use std::sync::Arc;

use pyo3::exceptions::{PyNotImplementedError, PyValueError};
use pyo3::prelude::*;

use opencamlib as ocl;
use opencamlib::MillingCutter;

use crate::util::g_all;
use crate::geometry::{PyCLPoint, PyFiber, PyInterval, PySTLSurf, PyTriangle};

pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyMillingCutter>()?;
    m.add_class::<PyCylCutter>()?;
    m.add_class::<PyBallCutter>()?;
    m.add_class::<PyBullCutter>()?;
    m.add_class::<PyConeCutter>()?;
    m.add_class::<PyCompCylCutter>()?;
    m.add_class::<PyCompBallCutter>()?;
    m.add_class::<PyCylConeCutter>()?;
    m.add_class::<PyBallConeCutter>()?;
    m.add_class::<PyBullConeCutter>()?;
    m.add_class::<PyConeConeCutter>()?;
    Ok(())
}

pub type SharedCutter = Arc<dyn MillingCutter>;

/// Base class of all cutters.
#[pyclass(name = "MillingCutter", module = "opencamlib.ocl", subclass, frozen)]
pub struct PyMillingCutter {
    pub c: SharedCutter,
}

impl PyMillingCutter {
    fn init(c: impl MillingCutter + 'static) -> PyMillingCutter {
        PyMillingCutter { c: Arc::new(c) }
    }
}

/// Run a drop-cutter test that lifts `cl` in place.
fn drop_with(cl: &Bound<'_, PyCLPoint>, f: impl FnOnce(&mut ocl::CLPoint) -> bool) -> bool {
    let mut p = PyCLPoint::get(cl);
    let hit = f(&mut p);
    PyCLPoint::set(cl, &p);
    hit
}

#[pymethods]
impl PyMillingCutter {
    fn vertexDrop(&self, cl: &Bound<'_, PyCLPoint>, t: PyRef<PyTriangle>) -> bool {
        drop_with(cl, |p| self.c.vertex_drop(p, &t.t))
    }
    fn facetDrop(&self, cl: &Bound<'_, PyCLPoint>, t: PyRef<PyTriangle>) -> bool {
        drop_with(cl, |p| self.c.facet_drop(p, &t.t))
    }
    fn edgeDrop(&self, cl: &Bound<'_, PyCLPoint>, t: PyRef<PyTriangle>) -> bool {
        drop_with(cl, |p| self.c.edge_drop(p, &t.t))
    }
    fn dropCutter(&self, cl: &Bound<'_, PyCLPoint>, t: PyRef<PyTriangle>) -> bool {
        drop_with(cl, |p| self.c.drop_cutter(p, &t.t))
    }
    /// Drop against every triangle of the surface (no kd-tree).
    fn dropCutterSTL(&self, py: Python<'_>, cl: &Bound<'_, PyCLPoint>, s: PyRef<PySTLSurf>) -> bool {
        let surf = &s.s;
        let c = &self.c;
        drop_with(cl, |p| py.allow_threads(|| c.drop_cutter_stl(p, surf)))
    }
    fn pushCutter(&self, f: PyRef<PyFiber>, i: &Bound<'_, PyInterval>, t: PyRef<PyTriangle>) -> bool {
        self.c.push_cutter(&f.f, &mut i.borrow_mut().i, &t.t)
    }
    fn offsetCutter(&self, d: f64) -> PyResult<PyMillingCutter> {
        self.c
            .offset_cutter(d)
            .map(|c| PyMillingCutter { c: Arc::from(c) })
            .ok_or_else(|| PyNotImplementedError::new_err("offsetCutter() is not implemented for this cutter"))
    }
    fn getRadius(&self) -> f64 {
        self.c.radius()
    }
    fn getLength(&self) -> f64 {
        self.c.length()
    }
    fn getDiameter(&self) -> f64 {
        self.c.diameter()
    }
    fn __str__(&self) -> String {
        g_all(&self.c.to_string())
    }
}

fn positive(name: &str, v: f64) -> PyResult<()> {
    if v > 0.0 {
        Ok(())
    } else {
        Err(PyValueError::new_err(format!("{name} must be positive, got {v}")))
    }
}

fn angle(name: &str, a: f64) -> PyResult<()> {
    if a > 0.0 && a < std::f64::consts::FRAC_PI_2 {
        Ok(())
    } else {
        Err(PyValueError::new_err(format!("{name} must be in (0, pi/2) radians, got {a}")))
    }
}

/// Declare a cutter subclass of MillingCutter with a constructor.
macro_rules! cutter_class {
    ($rust:ident, $py:literal, $doc:literal, ($($arg:ident),*), $build:expr) => {
        #[doc = $doc]
        #[pyclass(name = $py, module = "opencamlib.ocl", extends = PyMillingCutter, frozen)]
        pub struct $rust;

        #[pymethods]
        impl $rust {
            #[new]
            fn new($($arg: f64),*) -> PyResult<(Self, PyMillingCutter)> {
                let base: PyResult<PyMillingCutter> = $build;
                Ok(($rust, base?))
            }
        }
    };
}

cutter_class!(PyCylCutter, "CylCutter", "CylCutter(diameter, length): flat end mill.", (d, l), {
    positive("diameter", d)?;
    positive("length", l)?;
    Ok(PyMillingCutter::init(ocl::CylCutter::new(d, l)))
});

cutter_class!(PyBallCutter, "BallCutter", "BallCutter(diameter, length): ball end mill.", (d, l), {
    positive("diameter", d)?;
    positive("length", l)?;
    Ok(PyMillingCutter::init(ocl::BallCutter::new(d, l)))
});

cutter_class!(
    PyBullCutter,
    "BullCutter",
    "BullCutter(diameter, corner_radius, length): bull-nose (toroidal) end mill.",
    (d, r, l),
    {
        positive("diameter", d)?;
        positive("length", l)?;
        if !(r > 0.0 && r < d / 2.0) {
            return Err(PyValueError::new_err("corner_radius must be in (0, diameter/2)"));
        }
        Ok(PyMillingCutter::init(ocl::BullCutter::new(d, r, l)))
    }
);

/// ConeCutter(diameter, angle, length=10): cone cutter with half-angle `angle` (radians).
#[pyclass(name = "ConeCutter", module = "opencamlib.ocl", extends = PyMillingCutter, frozen)]
pub struct PyConeCutter;

#[pymethods]
impl PyConeCutter {
    #[new]
    #[pyo3(signature = (d, a, l = 10.0))]
    fn new(d: f64, a: f64, l: f64) -> PyResult<(Self, PyMillingCutter)> {
        positive("diameter", d)?;
        angle("angle", a)?;
        Ok((PyConeCutter, PyMillingCutter::init(ocl::ConeCutter::with_length(d, a, l))))
    }
}

cutter_class!(PyCompCylCutter, "CompCylCutter", "CompCylCutter(diameter, length)", (d, l), {
    positive("diameter", d)?;
    positive("length", l)?;
    Ok(PyMillingCutter::init(ocl::CompositeCutter::comp_cyl(d, l)))
});

cutter_class!(PyCompBallCutter, "CompBallCutter", "CompBallCutter(diameter, length)", (d, l), {
    positive("diameter", d)?;
    positive("length", l)?;
    Ok(PyMillingCutter::init(ocl::CompositeCutter::comp_ball(d, l)))
});

cutter_class!(
    PyCylConeCutter,
    "CylConeCutter",
    "CylConeCutter(diam1, diam2, angle): flat tip with a cone up to diam2.",
    (d1, d2, a),
    {
        positive("diam1", d1)?;
        positive("diam2", d2)?;
        angle("angle", a)?;
        Ok(PyMillingCutter::init(ocl::CompositeCutter::cyl_cone(d1, d2, a)))
    }
);

cutter_class!(
    PyBallConeCutter,
    "BallConeCutter",
    "BallConeCutter(diam1, diam2, angle): ball tip with a tangent cone up to diam2.",
    (d1, d2, a),
    {
        positive("diam1", d1)?;
        positive("diam2", d2)?;
        angle("angle", a)?;
        Ok(PyMillingCutter::init(ocl::CompositeCutter::ball_cone(d1, d2, a)))
    }
);

cutter_class!(
    PyBullConeCutter,
    "BullConeCutter",
    "BullConeCutter(diam1, radius1, diam2, angle): bull-nose tip with a tangent cone up to diam2.",
    (d1, r1, d2, a),
    {
        positive("diam1", d1)?;
        positive("diam2", d2)?;
        angle("angle", a)?;
        if !(r1 > 0.0 && r1 < d1 / 2.0) {
            return Err(PyValueError::new_err("radius1 must be in (0, diam1/2)"));
        }
        Ok(PyMillingCutter::init(ocl::CompositeCutter::bull_cone(d1, r1, d2, a)))
    }
);

cutter_class!(
    PyConeConeCutter,
    "ConeConeCutter",
    "ConeConeCutter(diam1, angle1, diam2, angle2): cone tip with a second cone up to diam2.",
    (d1, a1, d2, a2),
    {
        positive("diam1", d1)?;
        positive("diam2", d2)?;
        angle("angle1", a1)?;
        angle("angle2", a2)?;
        Ok(PyMillingCutter::init(ocl::CompositeCutter::cone_cone(d1, a1, d2, a2)))
    }
);
