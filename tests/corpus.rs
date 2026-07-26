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
        name: "square",
        boundary: vec![[15.0, 15.0], [85.0, 15.0], [85.0, 70.0], [15.0, 70.0]],
        holes: vec![],
    }
}

/// Rectangle with one rectangular hole -- the shape the UI opens on.
fn square_with_hole() -> Domain {
    Domain {
        name: "square_with_hole",
        boundary: vec![[15.0, 15.0], [85.0, 15.0], [85.0, 70.0], [15.0, 70.0]],
        holes: vec![vec![[38.0, 30.0], [58.0, 30.0], [58.0, 48.0], [38.0, 48.0]]],
        }
}

/// Two holes. Aimed at the hole loop in `may_insert_triangle`, which assigns
/// rather than accumulates and so only enforces the last hole it happens to
/// visit -- and `holes` is a `HashSet`, so which one that is varies per run.
fn two_holes() -> Domain {
    Domain {
        name: "two_holes",
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
        name: "arch",
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
        name: "notched",
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
        name: "ell",
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

fn corpus() -> Vec<Domain> {
    vec![
        square(),
        square_with_hole(),
        two_holes(),
        ell(),
        arch(),
        notched(),
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
