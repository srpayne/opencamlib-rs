use std::fmt;

use crate::geo::{CCPoint, CCType, Point};

/// An interval `[lower, upper]` of fiber parameter values where the cutter is in contact
/// with the model, together with the cutter-contact points at both ends.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Interval {
    /// Upper t-value.
    pub upper: f64,
    /// Lower t-value.
    pub lower: f64,
    /// CC-point at `upper`.
    pub upper_cc: CCPoint,
    /// CC-point at `lower`.
    pub lower_cc: CCPoint,
}

impl Interval {
    /// An empty interval.
    pub fn new() -> Self {
        Interval::default()
    }

    /// The interval `[l, u]` (with undefined cc-points).
    pub fn from_bounds(l: f64, u: f64) -> Self {
        Interval {
            lower: l,
            upper: u,
            ..Interval::default()
        }
    }

    /// Extend the interval to include `t`, with cc-point `p`.
    pub fn update(&mut self, t: f64, p: &CCPoint) {
        self.update_upper(t, p);
        self.update_lower(t, p);
    }

    /// If `cc_tmp` lies within the edge p1-p2 and `condition` holds, call
    /// [`Interval::update`] and return true.
    pub fn update_if_cc_in_edge_and_true(
        &mut self,
        t_cl: f64,
        cc_tmp: &CCPoint,
        p1: &Point,
        p2: &Point,
        condition: bool,
    ) -> bool {
        if cc_tmp.is_inside_segment(p1, p2) && condition {
            self.update(t_cl, cc_tmp);
            true
        } else {
            false
        }
    }

    /// Raise `upper` to `t` if larger. An undefined interval is first set to `[t, t]`.
    pub fn update_upper(&mut self, t: f64, p: &CCPoint) {
        if self.upper_cc.cc_type == CCType::None {
            self.upper = t;
            self.lower = t;
            self.upper_cc = *p;
            self.lower_cc = *p;
        }
        if t > self.upper {
            self.upper = t;
            self.upper_cc = *p;
        }
    }

    /// Lower `lower` to `t` if smaller. An undefined interval is first set to `[t, t]`.
    pub fn update_lower(&mut self, t: f64, p: &CCPoint) {
        if self.lower_cc.cc_type == CCType::None {
            self.lower = t;
            self.upper = t;
            self.lower_cc = *p;
            self.upper_cc = *p;
        }
        if t < self.lower {
            self.lower = t;
            self.lower_cc = *p;
        }
    }

    /// True if this interval lies completely outside `i`.
    pub fn outside(&self, i: &Interval) -> bool {
        self.lower > i.upper || self.upper < i.lower
    }

    /// True if this interval lies strictly inside `i`.
    pub fn inside(&self, i: &Interval) -> bool {
        self.lower > i.lower && self.upper < i.upper
    }

    /// True if the interval is empty (`lower == upper == 0`).
    pub fn empty(&self) -> bool {
        self.lower == 0.0 && self.upper == 0.0
    }
}

impl fmt::Display for Interval {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "I [{} , {} ]", self.lower, self.upper)
    }
}
