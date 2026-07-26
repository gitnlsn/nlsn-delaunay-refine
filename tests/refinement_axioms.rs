//! Laws refinement must obey about its own result.
//!
//! `tests/axioms.rs` asserts the laws of the fundamental types. This file
//! asserts the one law of the refinement loop, which is about honesty rather
//! than geometry: **if refinement reports `Converged`, no element is below the
//! requested minimum angle.** That is not a quality target, it is the documented
//! meaning of the variant -- "No triangle violates the quality or area
//! constraints".
//!
//! It is worth stating separately because a mesher that misses the target and
//! says so is usable: the caller can loosen the target, round a corner off, or
//! accept the local quality. A mesher that misses the target and reports success
//! is not, because nothing downstream has any reason to look.
//!
//! Some geometry genuinely cannot meet a target -- a corner sharper than roughly
//! twice the requested angle is a property of the input, not a mesher failure.
//! The law still holds there: the outcome must simply not be `Converged`.

mod invariants;
use invariants::Domain;
use nlsn_delaunay::planar::mesh::FlatMesh;
use nlsn_delaunay::planar::refine_params::RefineOutcome;

/// The reported shape: a hand-drawn triangle with a horizontal base and the
/// apex off to one side. Coordinates read off the screenshot against its grid,
/// which is 10 world units per line -- the size matters as much as the shape,
/// because `max_area` is absolute and this triangle is only ~750 units of area
/// against a 40 unit target. Corners measure ~61/63/55 degrees.
///
/// Deliberately near axis-aligned. The mesher's other open bug -- a hash-order
/// dependent cavity seed, see `tests/determinism_matrix.rs` -- is quiet in that
/// regime, and this fixture conserves area at every refinement level in
/// `edge_cases.rs`. So a failure here is a quality failure and nothing else.
fn drawn_triangle() -> Domain {
    Domain {
        name: "drawn_triangle".to_string(),
        boundary: vec![[29.7, 14.3], [69.4, 14.3], [50.5, 52.2]],
        holes: vec![],
    }
}

/// Reports the worst angle alongside the outcome, so a failure says how far off
/// it was rather than only that it was off.
fn check(domain: &Domain, min_angle_deg: f64, max_area: Option<f64>) -> Result<(), String> {
    let (triangulator, outcome) = domain.mesh_with_outcome(min_angle_deg, max_area);

    if outcome != RefineOutcome::Converged {
        return Ok(());
    }

    let mesh = FlatMesh::from_triangulation(&triangulator.triangulation.borrow());
    let below: Vec<f64> = mesh
        .min_angles_deg
        .iter()
        .cloned()
        // A hair of slack: the quality bound is applied as a radius-edge ratio
        // and converted back, so an element exactly on the target can land a
        // rounding step under it without anything being wrong.
        .filter(|angle| *angle < min_angle_deg - 1.0E-9)
        .collect();

    if below.is_empty() {
        return Ok(());
    }

    let worst = below.iter().cloned().fold(f64::INFINITY, f64::min);
    Err(format!(
        "{}: refinement reported Converged at a {} degree target, but {} of {} \
         elements are below it, the worst at {:.2} degrees",
        domain.name,
        min_angle_deg,
        below.len(),
        mesh.triangle_count(),
        worst,
    ))
}

/// Repeated because the mesher is not yet deterministic: a single green run
/// proves nothing, and this law is about what it *always* claims.
fn assert_converged_means_converged(domain: &Domain, min_angle_deg: f64, max_area: Option<f64>) {
    let mut failures = Vec::new();
    for _ in 0..20 {
        if let Err(message) = check(domain, min_angle_deg, max_area) {
            failures.push(message);
        }
    }

    assert!(
        failures.is_empty(),
        "{} of 20 runs claimed convergence they had not reached, e.g.\n  {}",
        failures.len(),
        failures[0],
    );
}

/// The reported case: 20 degree target, 0.40% max area on a 100x100 world.
#[test]
fn the_drawn_triangle_converges_or_says_it_did_not() {
    assert_converged_means_converged(&drawn_triangle(), 20.0, Some(40.0));
}

/// The same law with no area bound at all, so only the angle target drives
/// refinement. Isolates the quality path from the sizing path.
#[test]
fn the_drawn_triangle_meets_its_angle_target_with_no_area_bound() {
    assert_converged_means_converged(&drawn_triangle(), 20.0, None);
}

/// An axis-aligned square, where every other invariant already holds. If this
/// fails, the law is being broken everywhere rather than by the fixture.
#[test]
fn a_square_converges_or_says_it_did_not() {
    let square = Domain {
        name: "square".to_string(),
        boundary: vec![[20.0, 20.0], [80.0, 20.0], [80.0, 80.0], [20.0, 80.0]],
        holes: vec![],
    };
    assert_converged_means_converged(&square, 20.0, Some(40.0));
}
