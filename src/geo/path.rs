use super::{Arc, Line, Point};

/// The type of a [`Span`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpanType {
    Line,
    Arc,
}

/// One element of a [`Path`]: a line or an arc.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Span {
    Line(Line),
    Arc(Arc),
}

impl Span {
    pub fn span_type(&self) -> SpanType {
        match self {
            Span::Line(_) => SpanType::Line,
            Span::Arc(_) => SpanType::Arc,
        }
    }

    /// Length of the span in the XY plane.
    pub fn length2d(&self) -> f64 {
        match self {
            Span::Line(l) => l.length2d(),
            Span::Arc(a) => a.length2d(),
        }
    }

    /// Point along the span at parameter `t` in [0, 1].
    pub fn get_point(&self, t: f64) -> Point {
        match self {
            Span::Line(l) => l.get_point(t),
            Span::Arc(a) => a.get_point(t),
        }
    }
}

/// A path made of lines and arcs.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Path {
    pub span_list: Vec<Span>,
}

impl Path {
    pub fn new() -> Self {
        Path::default()
    }

    /// Append a line segment.
    pub fn append_line(&mut self, l: Line) {
        self.span_list.push(Span::Line(l));
    }

    /// Append an arc.
    pub fn append_arc(&mut self, a: Arc) {
        self.span_list.push(Span::Arc(a));
    }

    /// The spans of this path.
    pub fn spans(&self) -> &[Span] {
        &self.span_list
    }
}
