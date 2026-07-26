use crate::elements::vertex::*;
use robust::{incircle, Coord};

#[derive(PartialEq, Debug)]
pub enum Continence {
    Inside,
    Outside,
    Boundary,
}

#[derive(PartialEq, Debug)]
pub enum BoundaryInclusion {
    Open,
    Closed,
}

#[inline]
fn coord(v: &Vertex) -> Coord<f64> {
    Coord { x: v.x, y: v.y }
}

/// Checks whether Vertex d is contained by the circumcircle defined by
/// triangle(a, b, c). Vertices a, b and c must be in counterclockwise order.
///
/// This is the Delaunay kernel predicate, driving Bowyer-Watson cavity
/// selection, so it is evaluated exactly via Shewchuk's adaptive-precision
/// arithmetic.
///
/// Exactness matters here more than anywhere else in the crate. The in-circle
/// determinant grows as the *fourth* power of the coordinate magnitude, so the
/// fixed absolute `1e-14` epsilon this used to compare against was never a
/// meaningful threshold: on a drawing in pixel coordinates the determinant sits
/// around 1e8 and the epsilon could never trigger, while on a drawing in
/// millimetres it fired constantly. Since a drawing UI hands in whatever
/// coordinates the user happens to work in, that made correctness a function of
/// zoom level.
pub fn continence(a: &Vertex, b: &Vertex, c: &Vertex, d: &Vertex) -> Continence {
    // robust::incircle is positive when d lies inside the circle through a, b
    // and c, given a, b, c counterclockwise -- matching the sign of the 4x4
    // determinant this previously computed via nalgebra.
    let det = incircle(coord(a), coord(b), coord(c), coord(d));

    if det == 0.0 {
        return Continence::Boundary;
    }

    if det > 0.0 {
        Continence::Inside
    } else {
        Continence::Outside
    }
}

#[cfg(test)]
mod in_circle {
    use super::*;

    #[test]
    fn test_continence_inside() {
        let p1 = Vertex::new(0.0, 0.0);
        let p2 = Vertex::new(1.0, 0.0);
        let p3 = Vertex::new(1.0, 1.0);
        let p4 = Vertex::new(0.6, 0.5);
        assert_eq!(continence(&p1, &p2, &p3, &p4), Continence::Inside);
    }

    #[test]
    fn test_continence_outside() {
        let p1 = Vertex::new(0.0, 0.0);
        let p2 = Vertex::new(1.0, 0.0);
        let p3 = Vertex::new(1.0, 1.0);
        let p4 = Vertex::new(0.0, 2.0);
        assert_eq!(continence(&p1, &p2, &p3, &p4), Continence::Outside);
    }

    #[test]
    fn test_continence_boundary() {
        let p1 = Vertex::new(0.0, 0.0);
        let p2 = Vertex::new(1.0, 0.0);
        let p3 = Vertex::new(1.0, 1.0);
        let p4 = Vertex::new(0.0, 1.0);
        assert_eq!(continence(&p1, &p2, &p3, &p4), Continence::Boundary);
    }

    /// A cocircular configuration must be recognised as such regardless of the
    /// coordinate scale it is expressed in. Under the previous absolute epsilon
    /// this held only near unit scale, because the determinant grows as the
    /// fourth power of the coordinates.
    #[test]
    fn is_scale_invariant() {
        for scale in [1.0E-3, 1.0, 1.0E3, 1.0E6] {
            let p1 = Vertex::new(0.0, 0.0);
            let p2 = Vertex::new(scale, 0.0);
            let p3 = Vertex::new(scale, scale);

            let on = Vertex::new(0.0, scale);
            assert_eq!(
                continence(&p1, &p2, &p3, &on),
                Continence::Boundary,
                "cocircular point misclassified at scale {}",
                scale
            );

            let inside = Vertex::new(0.6 * scale, 0.5 * scale);
            assert_eq!(
                continence(&p1, &p2, &p3, &inside),
                Continence::Inside,
                "interior point misclassified at scale {}",
                scale
            );

            let outside = Vertex::new(0.0, 2.0 * scale);
            assert_eq!(
                continence(&p1, &p2, &p3, &outside),
                Continence::Outside,
                "exterior point misclassified at scale {}",
                scale
            );
        }
    }
}
