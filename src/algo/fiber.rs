use std::fmt;

use super::interval::Interval;
use crate::geo::Point;

/// A fiber: a line segment from `p1` to `p2` at constant z, parallel to the x- or
/// y-axis, along which the push-cutter records contact [`Interval`]s.
#[derive(Debug, Clone, Default)]
pub struct Fiber {
    /// Start point.
    pub p1: Point,
    /// End point.
    pub p2: Point,
    /// Unit direction vector.
    pub dir: Point,
    /// The intervals in this fiber.
    pub ints: Vec<Interval>,
}

impl Fiber {
    pub fn new(p1: Point, p2: Point) -> Self {
        let mut dir = p2 - p1;
        dir.normalize();
        Fiber {
            p1,
            p2,
            dir,
            ints: Vec::new(),
        }
    }

    /// Add interval `i`, merging it with any existing intervals it overlaps.
    pub fn add_interval(&mut self, i: Interval) {
        if i.empty() {
            return;
        }
        if self.ints.is_empty() {
            self.ints.push(i);
        } else if self.contains(&i) {
            // already covered
        } else if self.missing(&i) {
            self.ints.push(i);
        } else {
            // merge i with all overlapping intervals
            let mut overlaps = Vec::new();
            self.ints.retain(|fi| {
                if fi.outside(&i) {
                    true
                } else {
                    overlaps.push(*fi);
                    false
                }
            });
            overlaps.push(i);
            let mut sumint = Interval::new();
            for intr in &overlaps {
                sumint.update_lower(intr.lower, &intr.lower_cc);
                sumint.update_upper(intr.upper, &intr.upper_cc);
            }
            self.ints.push(sumint);
        }
    }

    /// True if `i` lies strictly inside one of the existing intervals.
    pub fn contains(&self, i: &Interval) -> bool {
        self.ints.iter().any(|fi| i.inside(fi))
    }

    /// True if `i` does not overlap any existing interval.
    pub fn missing(&self, i: &Interval) -> bool {
        self.ints.iter().all(|fi| i.outside(fi))
    }

    /// The t-value of the projection of `p` onto the fiber.
    pub fn tval(&self, p: &Point) -> f64 {
        let d = self.p2 - self.p1;
        (*p - self.p1).dot(&d) / d.dot(&d)
    }

    /// The point on the fiber at parameter `t`.
    pub fn point(&self, t: f64) -> Point {
        self.p1 + t * (self.p2 - self.p1)
    }

    pub fn empty(&self) -> bool {
        self.ints.is_empty()
    }

    /// Number of intervals.
    pub fn size(&self) -> usize {
        self.ints.len()
    }

    /// CL-point at the upper end of interval `n`.
    pub fn upper_cl_point(&self, n: usize) -> Point {
        self.point(self.ints[n].upper)
    }

    /// CL-point at the lower end of interval `n`.
    pub fn lower_cl_point(&self, n: usize) -> Point {
        self.point(self.ints[n].lower)
    }
}

/// Fibers compare equal when their end points are equal.
impl PartialEq for Fiber {
    fn eq(&self, other: &Self) -> bool {
        self.p1 == other.p1 && self.p2 == other.p2
    }
}

impl fmt::Display for Fiber {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, " fiber dir={} and {} intervals", self.dir, self.ints.len())?;
        write!(f, " fiber.p1={} fiber.p2 {}", self.p1, self.p2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geo::{CCPoint, CCType};

    fn ival(l: f64, u: f64) -> Interval {
        let mut i = Interval::new();
        let cc = CCPoint::from_xyz(0.0, 0.0, 0.0, CCType::Vertex);
        i.update(l, &cc);
        i.update(u, &cc);
        i
    }

    #[test]
    fn merge_intervals() {
        let mut f = Fiber::new(Point::new(0.0, 0.0, 0.0), Point::new(10.0, 0.0, 0.0));
        f.add_interval(ival(0.1, 0.2));
        f.add_interval(ival(0.5, 0.6));
        assert_eq!(f.size(), 2);
        f.add_interval(ival(0.15, 0.55));
        assert_eq!(f.size(), 1);
        assert_eq!((f.ints[0].lower, f.ints[0].upper), (0.1, 0.6));
        f.add_interval(ival(0.2, 0.3)); // inside: no change
        assert_eq!(f.size(), 1);
        assert_eq!(f.point(0.5), Point::new(5.0, 0.0, 0.0));
    }
}
