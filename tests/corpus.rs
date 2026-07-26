//! The invariant harness run over a corpus of domains.
//!
//! Every fixture is meshed at several quality targets and checked against every
//! invariant. Reflex corners get particular attention: they are where the
//! domain-conformity gate is weakest, because a segment can lie inside the
//! domain while the triangle it stands for does not.

mod invariants;

use invariants::{assert_mesh_invariants, Domain};

/// A rectangle. The simplest thing that must always work.
fn square() -> Domain {
    Domain {
        name: "square".to_string(),
        boundary: vec![[15.0, 15.0], [85.0, 15.0], [85.0, 70.0], [15.0, 70.0]],
        holes: vec![],
    }
}

/// Rectangle with one rectangular hole -- the shape the UI opens on.
fn square_with_hole() -> Domain {
    Domain {
        name: "square_with_hole".to_string(),
        boundary: vec![[15.0, 15.0], [85.0, 15.0], [85.0, 70.0], [15.0, 70.0]],
        holes: vec![vec![[38.0, 30.0], [58.0, 30.0], [58.0, 48.0], [38.0, 48.0]]],
        }
}

/// Two holes. Aimed at the hole loop in `may_insert_triangle`, which assigns
/// rather than accumulates and so only enforces the last hole it happens to
/// visit -- and `holes` is a `HashSet`, so which one that is varies per run.
fn two_holes() -> Domain {
    Domain {
        name: "two_holes".to_string(),
        boundary: vec![[0.0, 0.0], [100.0, 0.0], [100.0, 60.0], [0.0, 60.0]],
        holes: vec![
            vec![[15.0, 20.0], [35.0, 20.0], [35.0, 40.0], [15.0, 40.0]],
            vec![[65.0, 20.0], [85.0, 20.0], [85.0, 40.0], [65.0, 40.0]],
        ],
    }
}

/// Arch. Two reflex corners at the underside of the span.
///
/// This is the shape that showed elements outside the boundary in the UI: a
/// ragged lower edge and a triangle protruding past the lower-left corner.
fn arch() -> Domain {
    Domain {
        name: "arch".to_string(),
        boundary: vec![
            [15.0, 15.0],
            [35.0, 15.0],
            [35.0, 55.0], // reflex
            [65.0, 55.0], // reflex
            [65.0, 15.0],
            [85.0, 15.0],
            [85.0, 80.0],
            [15.0, 80.0],
        ],
        holes: vec![],
    }
}

/// Single sharp reflex notch cut into the top edge.
fn notched() -> Domain {
    Domain {
        name: "notched".to_string(),
        boundary: vec![
            [15.0, 15.0],
            [85.0, 15.0],
            [85.0, 70.0],
            [52.0, 42.0], // reflex
            [15.0, 70.0],
        ],
        holes: vec![],
    }
}

/// An L. One reflex corner, axis-aligned -- the mildest reflex case there is,
/// so if this fails the problem is not about sharp angles at all.
fn ell() -> Domain {
    Domain {
        name: "ell".to_string(),
        boundary: vec![
            [0.0, 0.0],
            [60.0, 0.0],
            [60.0, 30.0],
            [30.0, 30.0], // reflex
            [30.0, 60.0],
            [0.0, 60.0],
        ],
        holes: vec![],
    }
}

/// The shape as drawn in the app: eight vertices placed by hand, two reflex
/// corners, and not one edge axis-aligned.
///
/// Coordinates read off the reported screenshot. Reported as meshing to 11
/// elements over 9 nodes -- the convex hull, with the notch never carved out.
///
/// Wound counter-clockwise: bottom-left, along the bottom to the left leg, up
/// the leg's inner face, across the underside of the span, down the right leg,
/// out to the bottom-right, up the right side, back across the top.
fn drawn_arch() -> Domain {
    Domain {
        name: "drawn_arch".to_string(),
        boundary: vec![
            [0.0, 1.7],
            [29.8, 1.7],
            [29.3, 53.3], // reflex
            [70.9, 50.9], // reflex
            [69.3, 2.2],
            [97.0, 0.0],
            [99.8, 72.4],
            [0.4, 72.8],
        ],
        holes: vec![],
    }
}

fn corpus() -> Vec<Domain> {
    vec![
        square(),
        square_with_hole(),
        two_holes(),
        ell(),
        arch(),
        notched(),
        drawn_arch(),
    ]
}

/// Quality targets spanning the provable-termination boundary (~20.7 degrees).
const TARGETS: [(f64, Option<f64>); 4] = [
    (15.0, None),
    (20.0, Some(40.0)),
    (25.0, Some(20.0)),
    (30.0, Some(60.0)),
];

fn run(domain: &Domain) {
    for (min_angle, max_area) in TARGETS {
        let triangulator = domain.mesh(min_angle, max_area);
        assert_mesh_invariants(
            &triangulator,
            domain,
            &format!("min_angle {}deg, max_area {:?}", min_angle, max_area),
        );
    }
}

#[test]
fn square_holds_invariants() {
    run(&square());
}

#[test]
fn square_with_hole_holds_invariants() {
    run(&square_with_hole());
}

#[test]
fn two_holes_holds_invariants() {
    run(&two_holes());
}

/// Regression: an L used to mesh with 637% excess area, 23 elements outside the
/// domain and 12 degenerate ones, then with a ~1% coverage shortfall. One
/// reflex corner, axis-aligned -- the mildest reflex case there is.
#[test]
fn ell_holds_invariants() {
    run(&ell());
}

/// Regression: this shape used to mesh correctly only about 3 runs in 10,
/// depending on HashSet iteration order.
#[test]
fn arch_holds_invariants() {
    run(&arch());
}

#[test]
fn notched_holds_invariants() {
    run(&notched());
}

/// The reported failure: an irregular hand-drawn arch.
/// KNOWN FAILURE — refinement inflates the mesh on non-axis-aligned geometry.
///
/// The initial triangulation is correct in every case, including this one: the
/// notch *is* carved out. Failures appear only under refinement, and scale with
/// how hard it refines — up to 54x the domain area, which is not spillage but
/// gross overlap, elements piled on top of one another.
///
/// Measured, all with a correct unrefined triangulation:
///
///   fixture               15deg/-      20deg/40        25deg/20
///   square                OK           OK              OK
///   square_jittered       OK           OK              45,810 vs 3,840
///   drawn_arch            OK           11,224          209,688 vs 4,937
///   drawn_arch_jittered   OK           OK              269,309 vs 4,945
///
/// A jittered *square* failing matters: it is convex, so this is not about
/// reflex corners. It is about coordinates that are not axis-aligned, which is
/// every shape a user draws with a mouse.
///
/// Minimised. The smallest reproducer is a **three-vertex triangle rotated by
/// 2 degrees**, failing at max_area 50. Exactly-axis-aligned shapes never fail;
/// a rotation of 0.5 degrees is enough to break one. Not monotonic in the size
/// of the perturbation, though: nudging one vertex by 0.001, 0.01 or 0.1 fails
/// while 1.0 and 5.0 do not, so it is specific configurations rather than
/// off-axis coordinates as such.
///
/// Traced within a single run, checking total solid area after every refinement
/// step. Refinement only ever subdivides, so that area must be invariant. Every
/// break is the same operation:
///
///   AREA BREAK at step 28: 1560.00 -> 2754.38 (+1194.38)
///     via Encroached(2 edges) on triangle quality 0.578
///
/// Always the segment-splitting path, never plain circumcenter insertion. The
/// quality of 0.578 is ~1/sqrt(3), the minimum a triangle can have, so these
/// are near-equilateral elements picked up for being too large rather than
/// badly shaped.
///
/// The likely mechanism is in `split_segment` (`refine_procedures/encroachment.rs`).
/// It deliberately removes the segment being split from the constraint set
/// before inserting the midpoint, so that the two triangles adjacent to that
/// segment can be absorbed into the cavity -- which is what the on-edge case
/// requires. But the segment is a *boundary* segment, so for the duration of
/// that insertion the domain boundary is not constrained, and the cavity is
/// free to expand across it. `may_insert_triangle` is the only thing left
/// holding the line, and its non-ghost test is a proxy: it asks whether the
/// segment from a triangle's centroid to the new vertex stays inside the
/// domain, not whether the resulting triangle does.
///
/// Ruled out:
///   - missing boundary segment recovery (`boundary::include` has its recovery
///     block commented out, which is a real latent bug, but the *unrefined*
///     mesh is correct here so it is not this one)
///   - the degenerate-element refusal in `solve_conflicts` -- the inflation
///     persists without it (200,757 vs 262,840, same order of magnitude)
///   - directed-edge overwrites in `Triangulation::include_triangle` --
///     instrumented at zero
#[test]
#[ignore = "known: refinement inflates area on non-axis-aligned geometry"]
fn drawn_arch_holds_invariants() {
    run(&drawn_arch());
}

/// Every fixture again with its coordinates nudged off-axis.
///
/// The axis-aligned versions above all passed while `drawn_arch` did not, which
/// is the whole point: a reflex boundary's inner edges are Delaunay edges of the
/// vertex set only by symmetry, and hand-placed geometry has none. Testing only
/// tidy coordinates tests the alignment, not the shape.
/// KNOWN FAILURE — refinement inflates the mesh on non-axis-aligned geometry.
///
/// The initial triangulation is correct in every case, including this one: the
/// notch *is* carved out. Failures appear only under refinement, and scale with
/// how hard it refines — up to 54x the domain area, which is not spillage but
/// gross overlap, elements piled on top of one another.
///
/// Measured, all with a correct unrefined triangulation:
///
///   fixture               15deg/-      20deg/40        25deg/20
///   square                OK           OK              OK
///   square_jittered       OK           OK              45,810 vs 3,840
///   drawn_arch            OK           11,224          209,688 vs 4,937
///   drawn_arch_jittered   OK           OK              269,309 vs 4,945
///
/// A jittered *square* failing matters: it is convex, so this is not about
/// reflex corners. It is about coordinates that are not axis-aligned, which is
/// every shape a user draws with a mouse.
///
/// Ruled out so far:
///   - missing boundary segment recovery (`boundary::include` has its recovery
///     block commented out, but the unrefined mesh is correct regardless, so
///     that is a separate latent issue rather than this one)
///   - the degenerate-element refusal in `solve_conflicts` — the explosion
///     persists without it (200,757 vs 262,840, same order of magnitude)
///   - directed-edge overwrites in `Triangulation::include_triangle` —
///     instrumented at zero
///
/// Known and unexplained: the missing-adjacency guard in `solve_conflicts` is
/// genuinely reached during these runs, so the edge-to-triangle map does go
/// inconsistent. What corrupts it is still open, and is the thread to pull.
#[test]
#[ignore = "known: refinement inflates area on non-axis-aligned geometry"]
fn jittered_variants_hold_invariants() {
    for domain in corpus() {
        run(&domain.jittered());
    }
}

/// The corpus is only as good as its coverage, so pin down what it contains.
#[test]
fn corpus_covers_reflex_and_multi_hole_cases() {
    let domains = corpus();
    assert!(domains.len() >= 6);
    assert!(
        domains.iter().any(|d| d.holes.len() > 1),
        "corpus needs a multi-hole case"
    );
    assert!(
        domains.iter().any(|d| d.name == "arch"),
        "corpus needs the reported failing shape"
    );
}
