#![macro_use]

use crate::elements::vertex::*;
use robust::{orient2d, Coord};

#[derive(PartialEq, Debug)]
pub enum Orientation {
    Counterclockwise,
    Clockwise,
    Colinear,
}

/// Relative tolerance used by [`orientation_tol`].
///
/// The comparison is scale-invariant (see below), so this is effectively a
/// bound on `sin(theta)` between the two edge vectors rather than an absolute
/// determinant magnitude.
pub const DEFAULT_COLINEAR_EPS: f64 = 1.0E-12;

#[inline]
fn coord(v: &Vertex) -> Coord<f64> {
    Coord { x: v.x, y: v.y }
}

/// Exact orientation of the triangle (a, b, c).
///
/// Backed by Shewchuk's adaptive-precision predicates, so the sign returned is
/// always the true sign of the determinant for the given `f64` coordinates --
/// never a rounding artefact. This matters more than raw accuracy: the
/// triangulation kernel needs predicate results that are *mutually consistent*,
/// because contradictory answers to "which side is this point on" corrupt mesh
/// topology and can make refinement loop forever.
///
/// Note this reports the orientation of the coordinates as actually stored.
/// Points that are collinear in exact arithmetic but not in their `f64`
/// representation are correctly reported as turning slightly. Use
/// [`orientation_tol`] where the question is a modelling one ("did the user
/// mean to put this vertex on that segment?") rather than a topological one.
pub fn orientation(a: &Vertex, b: &Vertex, c: &Vertex) -> Orientation {
    // robust::orient2d is positive when a, b, c are counterclockwise, matching
    // the sign of the 3x3 determinant this previously computed via nalgebra.
    let det = orient2d(coord(a), coord(b), coord(c));

    if det == 0.0 {
        return Orientation::Colinear;
    }

    if det > 0.0 {
        Orientation::Counterclockwise
    } else {
        Orientation::Clockwise
    }
}

/// Orientation with a scale-invariant collinearity tolerance.
///
/// For geometric modelling decisions -- merging collinear edges, testing
/// whether an input vertex lies on a segment, simplifying polylines -- where
/// near-collinear input should snap to collinear.
///
/// The determinant equals `|ab| * |ac| * sin(theta)`, so dividing through by
/// `|ab| * |ac|` makes the test a pure angle tolerance. That is what the old
/// fixed `1e-14` absolute epsilon failed to do: the raw determinant scales as
/// coordinate squared, so an absolute threshold silently tightened on drawings
/// in pixel coordinates and loosened on drawings in small units.
pub fn orientation_tol(a: &Vertex, b: &Vertex, c: &Vertex, rel_eps: f64) -> Orientation {
    let det = orient2d(coord(a), coord(b), coord(c));

    let ab = ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt();
    let ac = ((c.x - a.x).powi(2) + (c.y - a.y).powi(2)).sqrt();
    let scale = ab * ac;

    // Degenerate input (coincident vertices) has no meaningful orientation.
    if scale == 0.0 {
        return Orientation::Colinear;
    }

    if (det / scale).abs() <= rel_eps {
        return Orientation::Colinear;
    }

    if det > 0.0 {
        Orientation::Counterclockwise
    } else {
        Orientation::Clockwise
    }
}

#[cfg(test)]
mod orientation {
    use super::*;

    #[test]
    fn test_counterclockwise() {
        let p1 = Vertex::new(0.0, 0.0);
        let p2 = Vertex::new(1.0, 0.0);
        let p3 = Vertex::new(0.0, 1.0);
        assert_eq!(orientation(&p1, &p2, &p3), Orientation::Counterclockwise);
    }

    #[test]
    fn test_clockwise() {
        let p1 = Vertex::new(0.0, 0.0);
        let p2 = Vertex::new(0.0, 1.0);
        let p3 = Vertex::new(1.0, 0.0);
        assert_eq!(orientation(&p1, &p2, &p3), Orientation::Clockwise);
    }

    #[test]
    fn test_colinear() {
        let p1 = Vertex::new(0.0, 0.0);
        let p2 = Vertex::new(1.0, 1.0);
        let p3 = Vertex::new(2.0, 2.0);
        assert_eq!(orientation(&p1, &p2, &p3), Orientation::Colinear);
    }

    /// These three points are collinear in exact arithmetic, but their f64
    /// representations are not. The exact predicate correctly reports the
    /// residual turn; the tolerant one snaps it to collinear. Both answers are
    /// right for their respective questions.
    #[test]
    fn exception_case_1() {
        let p1 = Vertex::new(5.0, 1.0);
        let p2 = Vertex::new(3.0, 4.0);
        let p3 = Vertex::new(4.133333333333334, 2.3000000000000003);

        assert_ne!(orientation(&p1, &p2, &p3), Orientation::Colinear);
        assert_eq!(
            orientation_tol(&p1, &p2, &p3, DEFAULT_COLINEAR_EPS),
            Orientation::Colinear
        );
    }

    /// The old absolute epsilon made correctness depend on drawing scale. The
    /// same triangle shape must classify identically whether the user drew it
    /// in unit coordinates or in pixels.
    #[test]
    fn is_scale_invariant() {
        for scale in [1.0E-6, 1.0, 1.0E3, 1.0E6] {
            let p1 = Vertex::new(0.0, 0.0);
            let p2 = Vertex::new(scale, 0.0);
            let p3 = Vertex::new(0.0, scale);
            assert_eq!(
                orientation(&p1, &p2, &p3),
                Orientation::Counterclockwise,
                "failed at scale {}",
                scale
            );

            let c1 = Vertex::new(0.0, 0.0);
            let c2 = Vertex::new(scale, scale);
            let c3 = Vertex::new(2.0 * scale, 2.0 * scale);
            assert_eq!(
                orientation(&c1, &c2, &c3),
                Orientation::Colinear,
                "failed at scale {}",
                scale
            );
        }
    }
}
