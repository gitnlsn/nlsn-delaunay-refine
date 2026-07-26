//! Refinement must produce the same mesh every run for the same input.
//!
//! The refinement loop used to choose its next triangle with
//! `HashSet::iter().next()`, which returns an arbitrary element. Rust seeds the
//! default hasher differently for each `HashSet`, so that choice varied between
//! runs and so did the resulting mesh -- every triangulation satisfied the
//! quality bound, but no two were identical. That is untenable once a solver
//! consumes these meshes: results stop being reproducible for reasons that have
//! nothing to do with the physics.
//!
//! Selection is now driven by triangle geometry (worst-quality first, ties
//! broken on sorted coordinates), so these tests pin that property down.

use nlsn_delaunay::planar::refine_params::RefineParams;
use nlsn_delaunay::{Polyline, Triangulator, Vertex};
use std::rc::Rc;

/// Canonical, order-independent representation of a mesh: every solid triangle
/// as its sorted corner coordinates, with the whole list sorted. Two meshes
/// compare equal exactly when they contain the same triangles, regardless of
/// the order the underlying `HashSet` happens to yield them in.
fn canonical_mesh(triangulator: &Triangulator) -> Vec<[(u64, u64); 3]> {
    // Compared as raw bits: refinement is deterministic down to the exact f64,
    // so there is no tolerance to allow for here, and bit patterns give a total
    // order without NaN ordering caveats.
    fn key(x: f64, y: f64) -> (u64, u64) {
        (x.to_bits(), y.to_bits())
    }

    let mut triangles: Vec<[(u64, u64); 3]> = triangulator
        .triangulation
        .borrow()
        .triangles
        .iter()
        .filter(|t| !t.is_ghost())
        .map(|t| {
            let mut corners = [
                key(t.v1.x, t.v1.y),
                key(t.v2.x, t.v2.y),
                key(t.v3.x, t.v3.y),
            ];
            corners.sort();
            corners
        })
        .collect();

    triangles.sort();
    triangles
}

/// A square domain with a notch, refined to a quality target. Deliberately not
/// symmetric, so a mesh that varies run to run will show it.
fn build_and_refine() -> Triangulator {
    let corners = [
        (0.0, 0.0),
        (4.0, 0.0),
        (4.0, 3.0),
        (2.5, 1.6),
        (1.0, 3.0),
        (0.0, 3.0),
    ];

    let vertices: Vec<Rc<Vertex>> = corners
        .iter()
        .map(|(x, y)| Rc::new(Vertex::new(*x, *y)))
        .collect();

    let boundary = Rc::new(Polyline::new_closed(vertices).unwrap());

    let mut triangulator = Triangulator::new(&boundary);
    triangulator.triangulate();
    triangulator.refine(RefineParams::from_min_angle_deg(20.0, Some(0.15)));

    triangulator
}

#[test]
fn refinement_is_reproducible_across_runs() {
    let first = canonical_mesh(&build_and_refine());
    let second = canonical_mesh(&build_and_refine());
    let third = canonical_mesh(&build_and_refine());

    assert!(
        !first.is_empty(),
        "fixture produced no triangles; the test would pass vacuously"
    );

    assert_eq!(
        first, second,
        "two runs on identical input produced different meshes"
    );
    assert_eq!(
        second, third,
        "three runs on identical input were not all identical"
    );
}

/// Each `HashSet` gets its own hash seed, so building several triangulators
/// before refining any of them exercises different seeds than the sequential
/// case above.
#[test]
fn refinement_is_reproducible_when_interleaved() {
    let a = build_and_refine();
    let b = build_and_refine();

    assert_eq!(canonical_mesh(&a), canonical_mesh(&b));
}
