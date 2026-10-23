use std::fmt;
use std::ops::{Deref, DerefMut};

use super::Point;

/// Type of cutter-contact point. The discriminants match the C++ `ocl::CCType` enum.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CCType {
    #[default]
    None = 0,
    Vertex,
    VertexCyl,
    Edge,
    EdgeHoriz,
    EdgeShaft,
    EdgeHorizCyl,
    EdgeHorizTor,
    EdgeBall,
    EdgePos,
    EdgeNeg,
    EdgeCyl,
    EdgeCone,
    EdgeConeBase,
    Facet,
    FacetTip,
    FacetCyl,
    Error,
}

/// Cutter-Contact (CC) point: a [`Point`] with a [`CCType`].
///
/// Dereferences to its [`Point`], so `cc.x`, `cc.norm()` etc. work directly.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CCPoint {
    pub pos: Point,
    pub cc_type: CCType,
}

impl CCPoint {
    pub const fn new(pos: Point, cc_type: CCType) -> Self {
        CCPoint { pos, cc_type }
    }

    pub const fn from_xyz(x: f64, y: f64, z: f64, cc_type: CCType) -> Self {
        CCPoint { pos: Point::new(x, y, z), cc_type }
    }
}

impl From<Point> for CCPoint {
    fn from(p: Point) -> Self {
        CCPoint::new(p, CCType::None)
    }
}

impl Deref for CCPoint {
    type Target = Point;
    fn deref(&self) -> &Point {
        &self.pos
    }
}

impl DerefMut for CCPoint {
    fn deref_mut(&mut self) -> &mut Point {
        &mut self.pos
    }
}

impl fmt::Display for CCPoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "CC({}, {}, {}, t={})",
            self.x, self.y, self.z, self.cc_type as u8
        )
    }
}
