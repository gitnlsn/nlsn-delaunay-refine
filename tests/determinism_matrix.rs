//! The mesher must return the same mesh for the same input.
//!
//! It does not. Rust seeds every `HashMap` differently, and `solve_conflicts`
//! chooses which cavity to open next with `conflict_map.keys().next()` — an
//! arbitrary entry in hash order. So the cavity seed differs on every call, and
//! with it the mesh: eight identical calls in one process produce eight
//! different meshes, covering anywhere from one to twenty times the domain area.
//!
//! Axis-aligned input is immune, which is why this went unnoticed. Its
//! coordinates and every midpoint of them are exactly representable, so the
//! predicates agree whichever order the choices are made in and every path
//! converges on the same answer. Rotate the same square by a quarter of a degree
//! and the choices start to matter.
//!
//! This is worth stating separately from the meshes being *wrong*: while the
//! output is a coin flip, no measurement of any other bug is stable, and several
//! hypotheses about the area inflation were tested against noise.

mod invariants;
use invariants::{solid_triangles, Domain};
use nlsn_delaunay::properties::area::area_triangle;

fn rotate(points: &[[f64; 2]], degrees: f64) -> Vec<[f64; 2]> {
    let n = points.len() as f64;
    let cx = points.iter().map(|p| p[0]).sum::<f64>() / n;
    let cy = points.iter().map(|p| p[1]).sum::<f64>() / n;
    let (s, c) = degrees.to_radians().sin_cos();
    points
        .iter()
        .map(|[x, y]| {
            let (dx, dy) = (x - cx, y - cy);
            [cx + dx * c - dy * s, cy + dx * s + dy * c]
        })
        .collect()
}

/// Element count and total area, which together identify a mesh closely enough
/// that two different meshes are very unlikely to agree on both.
fn signature(domain: &Domain, max_area: f64) -> (usize, String, String) {
    let (t, outcome) = domain.mesh_with_outcome(25.0, Some(max_area));
    let triangles = solid_triangles(&t);

    // Summed in sorted order. Floating point addition is not associative, so
    // summing straight out of a `HashSet` makes the total depend on iteration
    // order -- and this file would then report the instrument's own
    // non-determinism as the mesher's.
    let mut areas: Vec<f64> = triangles
        .iter()
        .map(|x| area_triangle(&x.v1, &x.v2, &x.v3).abs())
        .collect();
    areas.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    (
        triangles.len(),
        format!("{:.9}", areas.iter().sum::<f64>()),
        format!("{:?}", outcome),
    )
}

fn assert_repeatable(label: &str, base: &[[f64; 2]], degrees: f64, max_area: f64) {
    let domain = Domain {
        name: label.to_string(),
        boundary: rotate(base, degrees),
        holes: vec![],
    };

    let runs: Vec<(usize, String, String)> = (0..8).map(|_| signature(&domain, max_area)).collect();
    let distinct: std::collections::HashSet<&(usize, String, String)> = runs.iter().collect();

    assert_eq!(
        distinct.len(),
        1,
        "{} rotated {} deg at max_area {}: 8 identical requests produced {} \
         different meshes: {:?}",
        label,
        degrees,
        max_area,
        distinct.len(),
        runs,
    );
}

const SQUARE: [[f64; 2]; 4] = [[20.0, 20.0], [80.0, 20.0], [80.0, 80.0], [20.0, 80.0]];

/// The green half of the pair. Axis-aligned, and repeatable — so the harness
/// itself is sound and the failure below is about the geometry, not the test.
#[test]
fn an_axis_aligned_square_meshes_the_same_way_every_time() {
    assert_repeatable("square", &SQUARE, 0.0, 12.0);
}

/// The red half: the same square, rotated two degrees. Nothing else differs.
///
/// Passing since the last hash-order site closed. It was in `split_irregular`,
/// which iterated the set of encroached segments straight out of a `HashSet` to
/// decide which to split next -- and splitting one changes which of the others
/// are still encroached and where their midpoints land, so that choice
/// propagated through the whole refinement. `tests/stepwise.rs` located it by
/// diffing two step logs: they agreed exactly up to the second segment split and
/// disagreed on which segment it was.
#[test]
fn a_rotated_square_meshes_the_same_way_every_time() {
    assert_repeatable("square", &SQUARE, 2.0, 12.0);
}

/// A quarter of a degree is enough. Included because the natural assumption is
/// that a bug this large needs badly conditioned geometry, and it does not.
///
/// Passing since the cavity seed became deterministic and containment-based.
#[test]
fn a_barely_rotated_square_meshes_the_same_way_every_time() {
    assert_repeatable("square", &SQUARE, 0.25, 12.0);
}
