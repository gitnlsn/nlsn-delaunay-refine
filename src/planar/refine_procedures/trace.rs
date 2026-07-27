//! A recording of every mesh mutation refinement makes, in order.
//!
//! The invariant harness in `tests/invariants.rs` reports that a finished mesh
//! is wrong, and by how much. It cannot say which operation made it wrong, and
//! by the time it looks, thousands of operations have run. Every diagnosis in
//! this crate's history has stalled at that gap -- `tests/minimize.rs` narrowed
//! the reported failure to "one midpoint insertion removes one triangle and
//! creates three", and then had nowhere to go, because nothing could show what
//! that single insertion actually did.
//!
//! So the recording is taken at the only two places the mesh can change:
//! [`Triangulation::include_triangle`] and [`Triangulation::remove_triangle`].
//! Nothing can mutate the mesh without passing through them, which means no
//! path -- cavity retriangulation, segment recovery, hole carving, the
//! encroachment rollback -- can act unobserved. The higher-level loops mark the
//! stream with [`Event::Step`] so the events can be grouped back into the
//! operations a reader thinks in.
//!
//! Two flags carried here are worth more than the rest of the record. Both
//! `include_triangle` and `remove_triangle` begin by consulting a `HashSet`, and
//! both return `false` and do nothing when the lookup disagrees with them:
//!
//! ```ignore
//! pub fn remove_triangle(&mut self, triangle: &Rc<Triangle>) -> bool {
//!     if !self.triangles.contains(triangle) { return false; }
//! ```
//!
//! Every caller in the crate discards that `bool`. If `Triangle`'s `Hash` and
//! `Eq` ever disagree -- and they do today, `Hash` keying raw bits while `Eq`
//! compares quantised vertex identity -- a triangle that should have been taken
//! out stays in the mesh beside its replacements, silently. `found` and
//! `accepted` make that visible at the instant it happens rather than as an area
//! discrepancy ten thousand steps later.
//!
//! Recording is off unless a caller asks for it: `Triangulation::journal` is
//! `None` by default, and the cost when it is `None` is one branch per mutation.

use crate::elements::triangle::Triangle;

/// The three corners of a triangle, as stored -- never rounded.
///
/// Rounding here would defeat the purpose. The failures this exists to find turn
/// on the last few bits of a coordinate, so the record keeps what the mesher
/// actually held.
pub type Corners = [(f64, f64); 3];

/// The two ends of a segment.
pub type Ends = [(f64, f64); 2];

/// Why a vertex was inserted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InsertSource {
    /// A bad element's circumcentre, per Ruppert's first rule.
    Circumcentre,
    /// The midpoint of an encroached constrained segment, per the second rule.
    SegmentMidpoint,
}

/// What the mesher set out to do. One per [`Event::Step`].
#[derive(Debug, Clone, PartialEq)]
pub enum StepKind {
    /// An element popped off the work-set for failing the quality or area bound.
    ///
    /// Its group holds the attempt to insert the circumcentre -- including the
    /// case where the insertion is made and then rolled back because it
    /// encroached a constrained segment, which is worth seeing as its own event
    /// pair rather than as a silent no-op.
    BadTriangle {
        corners: Corners,
        /// Circumradius-to-shortest-edge ratio. `None` for a degenerate element.
        quality: Option<f64>,
    },
    /// A constrained segment being halved at its midpoint.
    ///
    /// Covers both routes to a split: the pre-refinement `unencroach` pass and
    /// the recursive splitting driven by a rejected circumcentre. Marked inside
    /// `split_segment` rather than at its two call sites, because that is the
    /// atomic operation -- one call halves one segment once.
    SegmentSplit { segment: Ends },
}

/// One entry in the recording.
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    /// A new operation begins. Everything until the next `Step` belongs to it.
    Step { index: usize, kind: StepKind },
    /// A vertex is about to be inserted.
    Insert {
        vertex: (f64, f64),
        source: InsertSource,
    },
    /// `include_triangle` was called.
    Included {
        corners: Corners,
        ghost: bool,
        /// `false` when the triangulation already held an equal triangle, so the
        /// call did nothing. A duplicate was constructed.
        accepted: bool,
    },
    /// `remove_triangle` was called.
    Removed {
        corners: Corners,
        ghost: bool,
        /// `false` when the lookup missed. The triangle is still in the mesh.
        found: bool,
    },
    /// A fan triangle the cavity retriangulation built and then declined to
    /// include, because it was degenerate.
    ///
    /// Recorded because "the element is missing" and "the element was refused"
    /// look identical in the mesh and call for opposite fixes. Without this, a
    /// gap in a step group is ambiguous between a cavity walk that never reached
    /// a rim edge and a guard that rejected what it found there.
    Refused { corners: Corners },
    /// The cavity walk reached an edge with no entry in the adjacency map and
    /// skipped it.
    ///
    /// The kernel treats this as a survivable inconsistency and carries on -- see
    /// the comment at the `continue` in `vertices::solve_conflicts`. It is
    /// survivable, but it is never *correct*: the rim edge gets no element built
    /// against it, so the cavity is left partly unfilled and the mesh has a gap
    /// exactly there.
    AdjacencyMissing { edge: Ends },
    /// How the step ended, as the kernel classified it.
    Outcome(String),
}

/// The recording itself.
#[derive(Debug, Clone, Default)]
pub struct Journal {
    pub events: Vec<Event>,
    /// Handed out by [`Journal::next_step_index`], so step numbers are
    /// continuous across the several loops that contribute to a refinement.
    next_index: usize,
}

impl Journal {
    pub fn new() -> Self {
        Self {
            events: Vec::new(),
            next_index: 0,
        }
    }

    pub fn push(&mut self, event: Event) {
        self.events.push(event);
    }

    pub fn next_step_index(&mut self) -> usize {
        let index = self.next_index;
        self.next_index += 1;
        index
    }
}

/// Reads a triangle's corners in the order it holds them.
pub fn corners_of(triangle: &Triangle) -> Corners {
    [
        (triangle.v1.x, triangle.v1.y),
        (triangle.v2.x, triangle.v2.y),
        (triangle.v3.x, triangle.v3.y),
    ]
}
