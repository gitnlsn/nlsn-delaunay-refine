//! Laws the fundamental types must obey.
//!
//! Everything else in this crate is built on `Vertex`, `Edge` and `Triangle`
//! being well-behaved as hash-map keys and as values with an equality. The
//! mesher stores triangles in a `HashSet<Rc<Triangle>>` and its adjacency in a
//! `HashMap<Rc<Edge>, Rc<Triangle>>`, so a broken `Hash`/`Eq` pairing does not
//! produce a wrong answer — it produces a *silently missing* one. A lookup lands
//! in the wrong bucket, `contains` says no, `remove_triangle` returns early
//! without removing, and the mesh quietly grows a duplicate.
//!
//! That is exactly how two real defects hid here. Both were found by chasing a
//! downstream symptom (meshes covering several times their own domain) rather
//! than by testing the law directly. These tests assert the laws themselves, so
//! the next such defect is caught at its source rather than fifty layers away.
//!
//! Rust states the contract plainly: `k1 == k2` implies `hash(k1) == hash(k2)`.
//! Violating it is not undefined behaviour, but `HashMap`/`HashSet` are
//! explicitly documented as behaving unpredictably when it happens.

use nlsn_delaunay::elements::edge::Edge;
use nlsn_delaunay::elements::triangle::Triangle;
use nlsn_delaunay::elements::vertex::Vertex;
use nlsn_delaunay::properties::orientation::{orientation, Orientation};

use std::collections::hash_map::DefaultHasher;
use std::collections::HashSet;
use std::hash::{Hash, Hasher};
use std::rc::Rc;

fn hash_of<T: Hash>(value: &T) -> u64 {
    let mut hasher = DefaultHasher::new();
    value.hash(&mut hasher);
    hasher.finish()
}

fn v(x: f64, y: f64) -> Rc<Vertex> {
    Rc::new(Vertex::new(x, y))
}

// ---------------------------------------------------------------------------
// Equality laws
// ---------------------------------------------------------------------------

#[test]
fn equality_is_reflexive() {
    let vertex = v(1.5, -2.25);
    assert_eq!(*vertex, *vertex);

    let edge = Edge::new(&v(0.0, 0.0), &v(1.0, 0.0));
    assert_eq!(edge, edge);

    let triangle = Triangle::new(&v(0.0, 0.0), &v(1.0, 0.0), &v(0.0, 1.0));
    assert_eq!(triangle, triangle);
}

#[test]
fn equality_is_symmetric() {
    let a = Triangle::new(&v(0.0, 0.0), &v(1.0, 0.0), &v(0.0, 1.0));
    let b = Triangle::new(&v(1.0, 0.0), &v(0.0, 1.0), &v(0.0, 0.0));

    assert_eq!(a == b, b == a);
    assert!(a == b, "rotations must compare equal");
}

/// KNOWN FAILURE — `Vertex` equality is not transitive, so it is not an
/// equivalence relation and `Vertex` cannot lawfully implement `Eq`.
///
/// `PartialEq` compares with `approx_eq!` at an absolute epsilon of 1e-14.
/// Approximate equality is *never* transitive at any tolerance: three points
/// spaced just under the epsilon apart make the ends more than an epsilon
/// apart. That is a property of the comparison, not of the constant chosen.
///
/// This matters because `Eq` is what `HashSet`/`HashMap` rely on to decide key
/// identity, and a non-transitive relation means "is this vertex already in the
/// triangulation?" has no single well-defined answer.
///
/// Fixing it means making vertex identity exact and pushing tolerance into the
/// explicit geometric predicates, where it belongs.
///
/// That was attempted and reverted. It is not a localised change: refinement
/// stops converging, because the mesher relies on approximate identity to merge
/// the same geometric point reached by two routes that differ in their last
/// bits. Adding explicit snapping in `vertices::include` was not enough --
/// `try_circumcenter_insertion` reaches `solve_conflicts` directly and bypasses
/// it; guarding that was not enough either, because `unencroach_segment`'s
/// split loop is unbounded and a split that declines to act spins it forever.
/// `polyline::subtraction` fails independently, so the boolean operations lean
/// on the same tolerance.
///
/// A second approach was then tried and also reverted: **snap rounding**.
/// Quantise coordinates in `Vertex::new` -- the single funnel, since every
/// computed vertex goes through it -- so that exact equality *becomes* the
/// tolerant equality and is transitive by construction, with no call site
/// changed.
///
/// It worked, as far as the law goes. Both these tests passed, and the mesher's
/// own suite went fully green: 161 unit + 13 axioms + 7 corpus.
///
/// It failed on geometry. Two things were learned and are worth keeping:
///
///   - An *absolute* grid is the wrong shape. At 1e-14, matching the epsilon it
///     replaces, a centroid came back as 0.66666666666667 against a true
///     0.6666666666666666 -- two decimal digits gone. Worse, it reintroduces the
///     scale-dependence that motivated moving to exact predicates in the first
///     place. Masking low mantissa bits instead is relative and costs far less.
///
///   - Snapping puts a floor under how finely geometry can be subdivided, and
///     refinement subdivides hardest exactly where that hurts: at sharp corners.
///     The notched fixtures in `fea-wasm` stopped converging and ground to their
///     step cap. Detecting the floor directly -- refusing a split whose midpoint
///     rounds onto its own endpoint -- did not resolve it, so the stall has
///     another cause as well.
///
/// What survived from that attempt: bounds on the two unbounded recovery loops,
/// which were a latent hang regardless and are now committed.
///
/// So the remaining options are the ~190-site explicit-tolerance refactor, or
/// snap rounding plus whatever is needed to make refinement converge under a
/// resolution floor. Neither is a patch.
#[test]
#[ignore = "known: Vertex equality is approximate, therefore not transitive"]
fn vertex_equality_is_transitive() {
    let a = Vertex::new(0.0, 0.0);
    let b = Vertex::new(0.8e-14, 0.0);
    let c = Vertex::new(1.6e-14, 0.0);

    assert_eq!(a, b, "a and b are within the epsilon");
    assert_eq!(b, c, "b and c are within the epsilon");
    assert_eq!(a, c, "so a and c must be equal too — but they are not");
}

// ---------------------------------------------------------------------------
// The Hash/Eq contract: equal values must hash equally
// ---------------------------------------------------------------------------

/// Regression: `Triangle` derived `Hash` over its fields in declaration order
/// while `PartialEq` treated the three cyclic rotations as the same triangle,
/// so `(a, b, c)` and `(b, c, a)` compared equal and hashed differently.
#[test]
fn equal_triangles_hash_equally() {
    let (a, b, c) = (v(0.0, 0.0), v(4.0, 0.0), v(1.0, 3.0));

    let base = Triangle::new(&a, &b, &c);
    for rotation in [Triangle::new(&b, &c, &a), Triangle::new(&c, &a, &b)] {
        assert_eq!(base, rotation, "rotations must compare equal");
        assert_eq!(
            hash_of(&base),
            hash_of(&rotation),
            "equal triangles must hash equally"
        );
    }
}

#[test]
fn equal_edges_hash_equally() {
    let (a, b) = (v(2.0, 5.0), v(-1.0, 0.5));
    let one = Edge::new(&a, &b);
    let two = Edge::new(&a, &b);

    assert_eq!(one, two);
    assert_eq!(hash_of(&one), hash_of(&two));
}

/// KNOWN FAILURE — `Vertex` compares approximately but hashes exactly.
///
/// Two vertices closer together than the epsilon compare equal while hashing
/// from their raw bit patterns, so they land in different buckets. A lookup for
/// one will not find the other.
///
/// This is the same root cause as [`vertex_equality_is_transitive`] and has the
/// same fix. It cannot be repaired on the hashing side: no hash function can be
/// consistent with a non-transitive equality.
#[test]
#[ignore = "known: Vertex hashes exact bits but compares approximately"]
fn equal_vertices_hash_equally() {
    let a = Vertex::new(0.0, 0.0);
    let b = Vertex::new(0.5e-14, 0.0);

    assert_eq!(a, b, "within the epsilon, so equal");
    assert_eq!(hash_of(&a), hash_of(&b), "equal values must hash equally");
}

// ---------------------------------------------------------------------------
// What the contract actually buys: collections behave
// ---------------------------------------------------------------------------

/// The consequence that bit the mesher. `Triangulation::remove_triangle` starts
/// by returning early if the set does not contain the triangle, so a rotated
/// lookup that missed left the element in the mesh while its replacements were
/// added alongside it — silently doubling the meshed area.
#[test]
fn a_hashset_finds_a_rotated_triangle() {
    let (a, b, c) = (v(0.0, 0.0), v(4.0, 0.0), v(1.0, 3.0));

    let mut set: HashSet<Rc<Triangle>> = HashSet::new();
    set.insert(Rc::new(Triangle::new(&a, &b, &c)));

    for rotation in [Triangle::new(&b, &c, &a), Triangle::new(&c, &a, &b)] {
        assert!(
            set.contains(&Rc::new(rotation)),
            "a rotation of a stored triangle must be found"
        );
    }
}

#[test]
fn removing_a_rotated_triangle_succeeds() {
    let (a, b, c) = (v(0.0, 0.0), v(4.0, 0.0), v(1.0, 3.0));

    let mut set: HashSet<Rc<Triangle>> = HashSet::new();
    set.insert(Rc::new(Triangle::new(&a, &b, &c)));

    assert!(set.remove(&Rc::new(Triangle::new(&b, &c, &a))));
    assert!(set.is_empty(), "the triangle must actually be gone");
}

// ---------------------------------------------------------------------------
// Domain-specific identity
// ---------------------------------------------------------------------------

/// A triangle is its cyclic rotations, but not its reflection: winding carries
/// orientation, which the kernel relies on for inside/outside decisions.
#[test]
fn a_triangle_is_its_rotations_but_not_its_reflection() {
    let (a, b, c) = (v(0.0, 0.0), v(4.0, 0.0), v(1.0, 3.0));

    assert_eq!(Triangle::new(&a, &b, &c), Triangle::new(&b, &c, &a));
    assert_eq!(Triangle::new(&a, &b, &c), Triangle::new(&c, &a, &b));

    assert_ne!(
        Triangle::new(&a, &b, &c),
        Triangle::new(&a, &c, &b),
        "reversing the winding must give a different triangle"
    );
}

/// Edges are directed. The adjacency map relies on it: each directed edge names
/// the single triangle on its left, and the opposite direction names the other.
#[test]
fn an_edge_is_directed() {
    let (a, b) = (v(0.0, 0.0), v(1.0, 1.0));

    assert_ne!(Edge::new(&a, &b), Edge::new(&b, &a));
    assert_eq!(Edge::new(&a, &b), Edge::new(&b, &a).opposite());
}

// ---------------------------------------------------------------------------
// Predicate laws
// ---------------------------------------------------------------------------

/// Swapping two arguments reverses the sign. Exact predicates must satisfy this
/// for every input, which is precisely what an epsilon-thresholded determinant
/// could not guarantee near zero.
#[test]
fn orientation_is_antisymmetric() {
    let cases = [
        (Vertex::new(0.0, 0.0), Vertex::new(1.0, 0.0), Vertex::new(0.0, 1.0)),
        (Vertex::new(5.0, 1.0), Vertex::new(3.0, 4.0), Vertex::new(4.13, 2.3)),
        (Vertex::new(-2.5, 7.25), Vertex::new(1e6, -1e6), Vertex::new(0.001, 0.002)),
    ];

    for (a, b, c) in cases.iter() {
        let forward = orientation(a, b, c);
        let swapped = orientation(a, c, b);

        let expected = match forward {
            Orientation::Counterclockwise => Orientation::Clockwise,
            Orientation::Clockwise => Orientation::Counterclockwise,
            Orientation::Colinear => Orientation::Colinear,
        };
        assert_eq!(swapped, expected, "orientation must flip when two arguments swap");
    }
}

/// Cyclic rotation leaves orientation unchanged — the same invariance
/// `Triangle`'s equality claims, at the predicate level.
#[test]
fn orientation_is_invariant_under_rotation() {
    let cases = [
        (Vertex::new(0.0, 0.0), Vertex::new(1.0, 0.0), Vertex::new(0.0, 1.0)),
        (Vertex::new(-3.5, 2.25), Vertex::new(8.75, -1.5), Vertex::new(0.25, 9.0)),
    ];

    for (a, b, c) in cases.iter() {
        let base = orientation(a, b, c);
        assert_eq!(orientation(b, c, a), base);
        assert_eq!(orientation(c, a, b), base);
    }
}

/// Scale invariance. The predicates used to compare a determinant against a
/// fixed absolute epsilon, which made the answer depend on the units the caller
/// happened to work in — and a drawing tool hands over whatever the user drew.
#[test]
fn orientation_does_not_depend_on_scale() {
    for scale in [1.0e-6, 1.0e-3, 1.0, 1.0e3, 1.0e6] {
        assert_eq!(
            orientation(
                &Vertex::new(0.0, 0.0),
                &Vertex::new(scale, 0.0),
                &Vertex::new(0.0, scale)
            ),
            Orientation::Counterclockwise,
            "failed at scale {}",
            scale
        );

        assert_eq!(
            orientation(
                &Vertex::new(0.0, 0.0),
                &Vertex::new(scale, scale),
                &Vertex::new(2.0 * scale, 2.0 * scale)
            ),
            Orientation::Colinear,
            "collinearity failed at scale {}",
            scale
        );
    }
}
