//! Laws about the coordinate frame.
//!
//! Every bug in this mesher's history has the same tell: the built-in sample —
//! axis-aligned — meshes correctly, and anything drawn by hand does not. That is
//! not a coincidence about drawing. Axis-aligned coordinates, and the midpoints
//! and circumcentres computed from them, are exactly representable in binary
//! floating point. Rotate the same shape and they are not, and the arithmetic
//! starts landing a few units in the last place away from where exact arithmetic
//! would put it.
//!
//! So the property worth asserting is **invariance under rigid motion**: a mesh
//! is a property of a shape, not of where that shape happens to sit relative to
//! the axes. Rotating the input and rotating the answer back must give the same
//! answer.
//!
//! These are ordered from the smallest claim to the largest, because that is
//! what makes a failure informative. If the predicates are invariant but the
//! mesh is not, the defect is in the algorithm, not the arithmetic — and the
//! converse points the other way. Testing only the mesh would tell us it broke
//! without telling us where.

use nlsn_delaunay::elements::edge::Edge;
use nlsn_delaunay::elements::triangle::Triangle;
use nlsn_delaunay::elements::vertex::Vertex;
use nlsn_delaunay::properties::continence::Continence;
use nlsn_delaunay::properties::orientation::{orientation, Orientation};

use std::collections::hash_map::DefaultHasher;
use std::collections::HashSet;
use std::hash::{Hash, Hasher};
use std::rc::Rc;

/// Angles chosen to be uncooperative: none is a multiple of 45 degrees, and the
/// smallest is far below anything a person would notice on screen. A quarter of
/// a degree is enough to break the mesher today.
const ANGLES: [f64; 7] = [0.25, 0.5, 1.0, 2.0, 7.5, 31.0, 47.3];

fn rotate(point: (f64, f64), degrees: f64) -> (f64, f64) {
    let (s, c) = degrees.to_radians().sin_cos();
    (point.0 * c - point.1 * s, point.0 * s + point.1 * c)
}

fn v(point: (f64, f64)) -> Rc<Vertex> {
    Rc::new(Vertex::new(point.0, point.1))
}

fn hash_of<T: Hash>(value: &T) -> u64 {
    let mut hasher = DefaultHasher::new();
    value.hash(&mut hasher);
    hasher.finish()
}

// ---------------------------------------------------------------------------
// Predicates
// ---------------------------------------------------------------------------

/// Which side of a line a point falls on cannot depend on the coordinate frame.
///
/// Restricted to well-separated points on purpose. A triangle within rounding of
/// degenerate has no frame-independent answer to give: the exact predicate
/// answers exactly, but it answers about the *rotated* coordinates, which are
/// themselves rounded. That is a real limit of floating point and not a defect,
/// so asserting it would be asserting something false.
#[test]
fn orientation_is_invariant_under_rotation() {
    let shapes = [
        [(0.0, 0.0), (10.0, 0.0), (5.0, 8.0)],
        [(0.0, 0.0), (10.0, 0.0), (5.0, -8.0)],
        [(-3.0, -7.0), (11.0, 2.0), (4.0, 9.0)],
    ];

    for shape in shapes.iter() {
        let reference = orientation(&v(shape[0]), &v(shape[1]), &v(shape[2]));
        assert_ne!(reference, Orientation::Colinear, "fixture must be separated");

        for degrees in ANGLES {
            let rotated = orientation(
                &v(rotate(shape[0], degrees)),
                &v(rotate(shape[1], degrees)),
                &v(rotate(shape[2], degrees)),
            );
            assert_eq!(
                rotated, reference,
                "orientation of {:?} changed at {} degrees",
                shape, degrees
            );
        }
    }
}

/// Whether a point is inside a circumcircle cannot depend on the coordinate
/// frame either. This is the predicate that decides which triangles a cavity
/// absorbs, so a frame-dependent answer here would change the mesh directly.
#[test]
fn incircle_is_invariant_under_rotation() {
    let triangle = [(0.0, 0.0), (10.0, 0.0), (5.0, 8.0)];
    // One comfortably inside the circumcircle, one comfortably outside.
    let probes = [(5.0, 3.0), (5.0, 40.0), (-20.0, 0.0), (5.0, 1.0)];

    for probe in probes.iter() {
        let reference = Triangle::new(&v(triangle[0]), &v(triangle[1]), &v(triangle[2]))
            .encircles(&Vertex::new(probe.0, probe.1));
        assert_ne!(reference, Continence::Boundary, "probe must be separated");

        for degrees in ANGLES {
            let rotated = Triangle::new(
                &v(rotate(triangle[0], degrees)),
                &v(rotate(triangle[1], degrees)),
                &v(rotate(triangle[2], degrees)),
            )
            .encircles(&Vertex::new(
                rotate(*probe, degrees).0,
                rotate(*probe, degrees).1,
            ));
            assert_eq!(
                rotated, reference,
                "continence of {:?} changed at {} degrees",
                probe, degrees
            );
        }
    }
}

/// Shape quality is a ratio of lengths, so a rigid motion cannot change it.
/// Refinement decides what to split from this number.
#[test]
fn triangle_quality_is_invariant_under_rotation() {
    let shapes = [
        [(0.0, 0.0), (10.0, 0.0), (5.0, 8.66)],
        [(0.0, 0.0), (10.0, 0.0), (9.0, 0.4)],
    ];

    for shape in shapes.iter() {
        let reference = Triangle::new(&v(shape[0]), &v(shape[1]), &v(shape[2]))
            .quality()
            .expect("fixture is not degenerate");

        for degrees in ANGLES {
            let rotated = Triangle::new(
                &v(rotate(shape[0], degrees)),
                &v(rotate(shape[1], degrees)),
                &v(rotate(shape[2], degrees)),
            )
            .quality()
            .expect("rotation cannot make a separated triangle degenerate");

            assert!(
                (rotated - reference).abs() / reference < 1.0E-9,
                "quality of {:?} moved from {} to {} at {} degrees",
                shape,
                reference,
                rotated,
                degrees
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Identity under arithmetic
// ---------------------------------------------------------------------------

/// A vertex the mesher computed must be findable in the set it was put in.
///
/// This is the law behind the whole class. Refinement inserts midpoints and
/// circumcentres, then looks them up again -- `vertices::include` filters
/// against `HashSet<Rc<Vertex>>` before inserting. A lookup that misses does not
/// error: it concludes the vertex is new and inserts a *second* one a hair away
/// from the first, and the sliver between them is what shows up on screen.
///
/// Axis-aligned input hides it because the midpoint of an exactly representable
/// pair is itself exactly representable, so the recomputed vertex is bit
/// identical and any hash finds it.
#[test]
#[ignore = "known bug: Vertex compares with a tolerance but hashes raw bits (task #10). \
            Proven here: 5 of 7 rotations put the same geometric point in two buckets."]
fn a_recomputed_vertex_is_found_in_a_set() {
    let mut misses = Vec::new();

    for degrees in ANGLES {
        // A right triangle. Its circumcentre is, exactly, the midpoint of the
        // hypotenuse -- so the two routes below describe the same point.
        //
        // The mesher takes both routes in normal operation: refinement inserts
        // circumcentres, segment splitting inserts midpoints, and they meet.
        // The arithmetic differs completely -- a determinant against a sum
        // halved -- so the results agree to within rounding rather than bit for
        // bit, which is the situation `Vertex` has to survive.
        let right_angle = v(rotate((0.0, 0.0), degrees));
        let a = v(rotate((10.0, 0.0), degrees));
        let b = v(rotate((0.0, 6.0), degrees));

        let via_circumcentre = Rc::new(
            Triangle::new(&right_angle, &a, &b)
                .circumcenter()
                .expect("a right triangle has a circumcentre"),
        );
        let via_midpoint = Rc::new(Edge::new(&a, &b).midpoint());

        let mut set: HashSet<Rc<Vertex>> = HashSet::new();
        set.insert(Rc::clone(&via_midpoint));

        if *via_circumcentre == *via_midpoint && !set.contains(&via_circumcentre) {
            misses.push(format!(
                "at {} degrees: ({}, {}) equals ({}, {}) but hashes to {} vs {}",
                degrees,
                via_circumcentre.x,
                via_circumcentre.y,
                via_midpoint.x,
                via_midpoint.y,
                hash_of(&via_circumcentre),
                hash_of(&via_midpoint),
            ));
        }
    }

    assert!(
        misses.is_empty(),
        "{} of {} rotations produced a vertex that compares equal to one already \
         in the set, yet cannot be found in it -- so the mesher would insert a \
         duplicate a hair away from the original:\n  {}",
        misses.len(),
        ANGLES.len(),
        misses.join("\n  "),
    );
}
