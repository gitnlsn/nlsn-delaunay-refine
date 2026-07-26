//! Input corners too sharp for the requested quality to be reachable.
//!
//! Where two input segments meet at a small angle, the elements filling that
//! wedge are forced to be at least as bad as the angle itself. No refinement
//! strategy fixes that: the corner is the constraint. Ruppert's concentric
//! shell splitting buys *termination* at such a corner, not quality.
//!
//! So refinement should recognise the situation and stop, rather than split
//! ever-smaller subsegments chasing a bound it cannot reach — which is what
//! grinds a refinement run to its step cap. The corner is a property of the
//! geometry, and the honest response is to report it and let the geometry be
//! changed: round the corner off, or accept the local quality.

use crate::elements::polyline::Polyline;
use crate::elements::vertex::Vertex;

use std::collections::HashMap;
use std::rc::Rc;

/// Below this, a corner is treated as unreachable for quality purposes.
///
/// 60 degrees is Ruppert's own threshold: above it, splitting a subsegment
/// cannot encroach its neighbour across the corner, so refinement is
/// well-behaved. Below it, the two segments can drive each other indefinitely.
pub const SHARP_CORNER_DEGREES: f64 = 60.0;

/// How far a subsegment at a sharp corner may be refined, as a fraction of the
/// shorter of the two segments meeting there.
///
/// 1/64 is six halvings deep — enough that the corner is well resolved relative
/// to its own feature size, and bounded enough that refinement cannot chase the
/// apex forever.
pub const SHARP_CORNER_FLOOR: f64 = 1.0 / 64.0;

/// A corner where the requested quality cannot be met.
#[derive(Debug, Clone)]
pub struct SharpCorner {
    pub apex: Rc<Vertex>,
    /// Interior angle at the apex, in degrees.
    pub angle_deg: f64,
    /// Subsegments incident to this apex are not split below this length.
    pub floor_length: f64,
}

fn distance(a: &Vertex, b: &Vertex) -> f64 {
    ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt()
}

/// Interior angle at `here`, between the segments running to `previous` and
/// `next`, in degrees.
///
/// Computed from side lengths rather than via `properties::angle`, which
/// returns a *directed* angle and reports `360 - theta` for clockwise winding.
/// What matters here is the size of the wedge, not which way the loop runs.
fn corner_angle_deg(previous: &Vertex, here: &Vertex, next: &Vertex) -> f64 {
    let a = distance(previous, here);
    let b = distance(next, here);

    if a == 0.0 || b == 0.0 {
        return 0.0;
    }

    let dot = (previous.x - here.x) * (next.x - here.x) + (previous.y - here.y) * (next.y - here.y);
    // Rounding can push this fractionally outside [-1, 1], where acos is NaN.
    let cosine = (dot / (a * b)).clamp(-1.0, 1.0);
    cosine.acos().to_degrees()
}

/// Every corner of a closed loop whose interior angle is below the threshold.
pub fn detect_in_loop(loop_polyline: &Rc<Polyline>, threshold_deg: f64) -> Vec<SharpCorner> {
    let vertices = &loop_polyline.vertices;
    let count = vertices.len();

    if count < 3 {
        return Vec::new();
    }

    let mut corners = Vec::new();

    for index in 0..count {
        let previous = &vertices[(index + count - 1) % count];
        let here = &vertices[index];
        let next = &vertices[(index + 1) % count];

        let angle_deg = corner_angle_deg(previous, here, next);
        if angle_deg >= threshold_deg {
            continue;
        }

        // Scaled to the shorter incident segment, so the floor means the same
        // thing whether the corner joins two long edges or two short ones.
        let shorter = distance(previous, here).min(distance(next, here));

        corners.push(SharpCorner {
            apex: Rc::clone(here),
            angle_deg,
            floor_length: shorter * SHARP_CORNER_FLOOR,
        });
    }

    corners
}

/// Sharp corners across a boundary and its holes, keyed by apex for lookup
/// during refinement.
pub fn detect(
    boundary: &Rc<Polyline>,
    holes: impl IntoIterator<Item = Rc<Polyline>>,
    threshold_deg: f64,
) -> Vec<SharpCorner> {
    let mut corners = detect_in_loop(boundary, threshold_deg);
    for hole in holes {
        corners.extend(detect_in_loop(&hole, threshold_deg));
    }
    corners
}

/// Apex-keyed floors, for asking "may this subsegment still be split?".
///
/// Keyed on the vertex rather than a coordinate pair because a corner is a
/// point, and a subsegment is incident to it if either endpoint matches.
pub fn floors(corners: &[SharpCorner]) -> HashMap<Rc<Vertex>, f64> {
    let mut map: HashMap<Rc<Vertex>, f64> = HashMap::new();
    for corner in corners {
        // A vertex could be the apex of two loops meeting; keep the tighter
        // floor so neither is over-refined.
        map.entry(Rc::clone(&corner.apex))
            .and_modify(|existing| *existing = existing.min(corner.floor_length))
            .or_insert(corner.floor_length);
    }
    map
}

/// Whether a subsegment has been refined as far as its sharp corner allows.
pub fn is_at_floor(v1: &Rc<Vertex>, v2: &Rc<Vertex>, floors: &HashMap<Rc<Vertex>, f64>) -> bool {
    if floors.is_empty() {
        return false;
    }

    let length = distance(v1, v2);

    for endpoint in [v1, v2] {
        if let Some(floor) = floors.get(endpoint) {
            if length <= *floor {
                return true;
            }
        }
    }

    false
}

#[cfg(test)]
mod detection {
    use super::*;

    fn loop_of(points: &[[f64; 2]]) -> Rc<Polyline> {
        Rc::new(
            Polyline::new_closed(
                points
                    .iter()
                    .map(|p| Rc::new(Vertex::new(p[0], p[1])))
                    .collect(),
            )
            .unwrap(),
        )
    }

    #[test]
    fn a_square_has_no_sharp_corners() {
        let square = loop_of(&[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]);
        assert!(detect_in_loop(&square, SHARP_CORNER_DEGREES).is_empty());
    }

    /// An equilateral triangle sits exactly on the threshold, so which side it
    /// falls on is decided by the last bit of `acos` -- in practice
    /// 59.999999999999993. Asserting either verdict would be asserting a
    /// rounding artefact, so this pins the measured angle instead and checks
    /// the classification only where it is unambiguous.
    #[test]
    fn an_equilateral_triangle_measures_sixty_degrees() {
        let triangle = loop_of(&[[0.0, 0.0], [1.0, 0.0], [0.5, 3.0f64.sqrt() / 2.0]]);

        for index in 0..3 {
            let vertices = &triangle.vertices;
            let angle = corner_angle_deg(
                &vertices[(index + 2) % 3],
                &vertices[index],
                &vertices[(index + 1) % 3],
            );
            assert!(
                (angle - 60.0).abs() < 1.0E-9,
                "corner {} measured {} degrees",
                index,
                angle
            );
        }

        // Clear of the boundary in both directions.
        assert!(detect_in_loop(&triangle, 59.0).is_empty());
        assert_eq!(detect_in_loop(&triangle, 61.0).len(), 3);
    }

    #[test]
    fn a_sliver_wedge_is_reported_with_its_angle() {
        // A long thin spike: the apex at the origin is a few degrees.
        let spike = loop_of(&[[0.0, 0.0], [100.0, 2.0], [100.0, -2.0]]);
        let corners = detect_in_loop(&spike, SHARP_CORNER_DEGREES);

        assert_eq!(corners.len(), 1);
        assert!(
            corners[0].angle_deg < 5.0,
            "expected a few degrees, got {}",
            corners[0].angle_deg
        );
        assert_eq!(corners[0].apex.x, 0.0);
        assert_eq!(corners[0].apex.y, 0.0);
    }

    #[test]
    fn the_floor_scales_with_the_shorter_incident_segment() {
        let spike = loop_of(&[[0.0, 0.0], [100.0, 2.0], [100.0, -2.0]]);
        let corner = detect_in_loop(&spike, SHARP_CORNER_DEGREES).remove(0);

        // Shorter incident segment is ~100.02 long.
        let expected = 100.02 * SHARP_CORNER_FLOOR;
        assert!(
            (corner.floor_length - expected).abs() / expected < 0.01,
            "floor {} is not ~{}",
            corner.floor_length,
            expected
        );
    }

    #[test]
    fn a_subsegment_at_the_floor_is_recognised() {
        let spike = loop_of(&[[0.0, 0.0], [100.0, 2.0], [100.0, -2.0]]);
        let floors = floors(&detect_in_loop(&spike, SHARP_CORNER_DEGREES));

        let apex = Rc::new(Vertex::new(0.0, 0.0));
        let far = Rc::new(Vertex::new(50.0, 1.0));
        let near = Rc::new(Vertex::new(0.1, 0.002));

        assert!(!is_at_floor(&apex, &far, &floors), "a long subsegment may still split");
        assert!(is_at_floor(&apex, &near, &floors), "a short one at the apex may not");
    }

    #[test]
    fn segments_away_from_a_sharp_corner_are_unaffected() {
        let spike = loop_of(&[[0.0, 0.0], [100.0, 2.0], [100.0, -2.0]]);
        let floors = floors(&detect_in_loop(&spike, SHARP_CORNER_DEGREES));

        // Both endpoints far from the apex: the floor must not apply, however
        // short the subsegment is.
        let a = Rc::new(Vertex::new(99.0, 1.0));
        let b = Rc::new(Vertex::new(99.001, 1.0));
        assert!(!is_at_floor(&a, &b, &floors));
    }
}
