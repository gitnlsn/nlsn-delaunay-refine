//! Invariants a finished mesh must satisfy, asserted end to end.
//!
//! The unit tests in `src/` check individual functions. None of them look at a
//! *completed* mesh and ask whether it actually covers its domain and nothing
//! else, which is why a triangle sitting outside the boundary can pass the
//! entire suite. These are the whole-mesh properties.
//!
//! The cheapest and most valuable one is area conservation: the element areas
//! must sum to the area of the domain. A single number catches elements outside
//! the boundary, elements inside holes, gaps, and overlaps -- all at once.
//!
//! This is a shared harness, pulled in with `mod invariants;` by several test
//! binaries. Each one uses a different part of it, so every binary compiles the
//! whole file and warns about whatever it did not call -- eleven warnings per
//! build, none of them about a real problem. Allowed at the module level rather
//! than per item, because "unused" here is a fact about one binary rather than
//! about the code.
#![allow(dead_code)]

use nlsn_delaunay::elements::polyline::{vertex_pairs, Polyline};
use nlsn_delaunay::elements::triangle::Triangle;
use nlsn_delaunay::elements::vertex::Vertex;
use nlsn_delaunay::planar::refine_params::{RefineOutcome, RefineParams};
use nlsn_delaunay::planar::refine_procedures::trace::Journal;
use nlsn_delaunay::planar::triangulator::Triangulator;
use nlsn_delaunay::properties::area::{area_segments, area_triangle};
use nlsn_delaunay::properties::continence::Continence;
use nlsn_delaunay::properties::orientation::{orientation, Orientation};

use std::collections::HashMap;
use std::rc::Rc;

/// Fuzzy comparison for areas, relative to the domain size.
const AREA_TOLERANCE: f64 = 1.0E-9;

#[derive(Clone)]
pub struct Domain {
    pub name: String,
    pub boundary: Vec<[f64; 2]>,
    pub holes: Vec<Vec<[f64; 2]>>,
}

pub fn to_polyline(points: &[[f64; 2]]) -> Rc<Polyline> {
    Rc::new(
        Polyline::new_closed(
            points
                .iter()
                .map(|p| Rc::new(Vertex::new(p[0], p[1])))
                .collect(),
        )
        .expect("fixture is not a valid closed polyline"),
    )
}

/// Absolute enclosed area of a closed loop.
///
/// `area_segments` and `area_triangle` are both *signed* and, awkwardly, use
/// opposite conventions -- the first is positive clockwise, the second positive
/// counter-clockwise. Everything here takes absolute values so the invariant
/// does not silently depend on which way a fixture happens to wind.
fn loop_area(points: &[[f64; 2]]) -> f64 {
    let vertices: Vec<Rc<Vertex>> = points
        .iter()
        .map(|p| Rc::new(Vertex::new(p[0], p[1])))
        .collect();
    area_segments(&vertex_pairs(&vertices, false)).abs()
}

impl Domain {
    /// Same shape, wound the other way.
    ///
    /// `Polyline::new_closed` accepts any vertex order, and which way a user
    /// clicks when drawing decides the winding. A closed region means the same
    /// thing either way, so the mesh should too.
    pub fn reversed(&self) -> Domain {
        let flip = |points: &[[f64; 2]]| {
            let mut reversed = points.to_vec();
            reversed.reverse();
            reversed
        };

        Domain {
            name: format!("{}_reversed", self.name),
            boundary: flip(&self.boundary),
            holes: self.holes.iter().map(|h| flip(h)).collect(),
        }
    }

    /// Number of solid elements, for comparing two meshes coarsely.
    pub fn element_count(&self, min_angle_deg: f64, max_area: Option<f64>) -> usize {
        solid_triangles(&self.mesh(min_angle_deg, max_area)).len()
    }

    /// Same shape, coordinates nudged off-axis.
    ///
    /// Every fixture written by hand ends up axis-aligned and symmetric, and
    /// that is exactly the condition under which a reflex boundary's inner
    /// edges happen to be Delaunay edges of the vertex set. Geometry drawn with
    /// a mouse never is. Jittering keeps the topology identical while removing
    /// the coincidence, so the fixture tests the shape rather than its
    /// alignment.
    ///
    /// The offsets are a fixed function of position, not random: a fixture that
    /// fails must fail the same way every run.
    pub fn jittered(&self) -> Domain {
        fn nudge(points: &[[f64; 2]]) -> Vec<[f64; 2]> {
            points
                .iter()
                .enumerate()
                .map(|(i, [x, y])| {
                    let k = i as f64;
                    [x + 0.31 * (k * 2.4).sin(), y + 0.29 * (k * 3.1).cos()]
                })
                .collect()
        }

        Domain {
            name: format!("{}_jittered", self.name),
            boundary: nudge(&self.boundary),
            holes: self.holes.iter().map(|h| nudge(h)).collect(),
        }
    }

    /// Area the mesh is expected to cover.
    pub fn expected_area(&self) -> f64 {
        self.holes
            .iter()
            .fold(loop_area(&self.boundary), |acc, hole| acc - loop_area(hole))
    }

    /// Initial triangulation only, with no refinement -- isolates whether a
    /// conformity failure is created by triangulate() or by refine().
    pub fn mesh_unrefined(&self) -> Triangulator {
        let boundary = to_polyline(&self.boundary);
        let mut triangulator = Triangulator::new(&boundary);
        for hole in self.holes.iter() {
            triangulator.insert_hole(&to_polyline(hole)).unwrap();
        }
        triangulator.triangulate();
        triangulator
    }

    pub fn mesh(&self, min_angle_deg: f64, max_area: Option<f64>) -> Triangulator {
        self.mesh_with_outcome(min_angle_deg, max_area).0
    }

    /// The mesh together with what refinement claims about it.
    ///
    /// Separate from `mesh` because the outcome is a claim that can be wrong
    /// independently of the mesh being wrong, and testing that claim needs both.
    pub fn mesh_with_outcome(
        &self,
        min_angle_deg: f64,
        max_area: Option<f64>,
    ) -> (Triangulator, RefineOutcome) {
        let (mut triangulator, params) = self.prepared(min_angle_deg, max_area);
        let outcome = triangulator.refine(params);
        (triangulator, outcome)
    }

    /// The mesh, the outcome, and a recording of every mutation refinement made.
    ///
    /// What `tests/stepwise.rs` checks its per-step laws against. The other
    /// accessors above measure the finished mesh, which can say a mesh is wrong
    /// but not which operation made it wrong.
    pub fn mesh_traced(
        &self,
        min_angle_deg: f64,
        max_area: Option<f64>,
    ) -> (Triangulator, RefineOutcome, Journal) {
        let (mut triangulator, params) = self.prepared(min_angle_deg, max_area);
        let (outcome, journal) = triangulator.refine_traced(params);
        (triangulator, outcome, journal)
    }

    /// Everything up to but not including refinement.
    fn prepared(&self, min_angle_deg: f64, max_area: Option<f64>) -> (Triangulator, RefineParams) {
        let boundary = to_polyline(&self.boundary);
        let mut triangulator = Triangulator::new(&boundary);

        for hole in self.holes.iter() {
            triangulator
                .insert_hole(&to_polyline(hole))
                .unwrap_or_else(|_| panic!("{}: hole is not inside the boundary", self.name));
        }

        triangulator.triangulate();

        let mut params = RefineParams::from_min_angle_deg(min_angle_deg, max_area);
        // Bounded so a fixture that fails to converge cannot take the machine
        // into swap along with it.
        params.max_steps = 20_000;
        params.max_triangles = 20_000;

        (triangulator, params)
    }
}

impl Domain {
    fn assert_counterclockwise(&self) {
        let signed = |points: &[[f64; 2]]| -> f64 {
            let vertices: Vec<Rc<Vertex>> = points
                .iter()
                .map(|p| Rc::new(Vertex::new(p[0], p[1])))
                .collect();
            area_segments(&vertex_pairs(&vertices, false))
        };

        // area_segments is the trapezoid form, which is *negative* for
        // counter-clockwise winding -- the opposite sign convention to
        // area_triangle. See polyline::segments_orientation.
        assert!(
            signed(&self.boundary) < 0.0,
            "{}: boundary is wound clockwise; the kernel expects counter-clockwise",
            self.name
        );
        for (index, hole) in self.holes.iter().enumerate() {
            assert!(
                signed(hole) < 0.0,
                "{}: hole {} is wound clockwise",
                self.name,
                index
            );
        }
    }
}

pub fn solid_triangles(triangulator: &Triangulator) -> Vec<Rc<Triangle>> {
    triangulator
        .triangulation
        .borrow()
        .triangles
        .iter()
        .filter(|t| !t.is_ghost())
        .cloned()
        .collect()
}

/// Element areas must sum to the domain area.
///
/// Deliberately the first check: it is one subtraction, and it detects every
/// way the mesh can fail to be a partition of the domain. An element outside
/// the boundary makes the sum too large; an unmeshed gap makes it too small;
/// overlapping elements make it too large. A per-element containment test can
/// only find the first of those.
pub fn check_area_conservation(triangulator: &Triangulator, domain: &Domain) -> Result<(), String> {
    let meshed: f64 = solid_triangles(triangulator)
        .iter()
        .map(|t| area_triangle(&t.v1, &t.v2, &t.v3).abs())
        .sum();

    let expected = domain.expected_area();
    let error = (meshed - expected).abs();
    let relative = error / expected.max(f64::MIN_POSITIVE);

    if relative > AREA_TOLERANCE {
        return Err(format!(
            "area conservation failed: mesh covers {:.6}, domain is {:.6} \
             (off by {:.6}, {:.4}%). {} means the mesh {} the domain.",
            meshed,
            expected,
            error,
            relative * 100.0,
            if meshed > expected { "Excess" } else { "Shortfall" },
            if meshed > expected {
                "extends beyond, or overlaps itself within"
            } else {
                "does not fully cover"
            },
        ));
    }
    Ok(())
}

/// Every element centroid lies inside the boundary and outside every hole.
///
/// Narrower than area conservation -- it cannot see gaps or overlaps -- but it
/// names *which* element is misplaced, which is what makes a failure
/// actionable.
pub fn check_domain_membership(triangulator: &Triangulator, domain: &Domain) -> Result<(), String> {
    let boundary = to_polyline(&domain.boundary);
    let holes: Vec<Rc<Polyline>> = domain.holes.iter().map(|h| to_polyline(h)).collect();

    let mut outside = Vec::new();
    let mut in_hole = Vec::new();

    for triangle in solid_triangles(triangulator).iter() {
        let centre = triangle.center();

        if boundary.contains(&centre) != Some(Continence::Inside) {
            outside.push(format!("({:.4}, {:.4})", centre.x, centre.y));
        }
        for (index, hole) in holes.iter().enumerate() {
            if hole.contains(&centre) == Some(Continence::Inside) {
                in_hole.push(format!("({:.4}, {:.4}) in hole {}", centre.x, centre.y, index));
            }
        }
    }

    if !outside.is_empty() || !in_hole.is_empty() {
        return Err(format!(
            "domain membership failed: {} element(s) centred outside the boundary {:?}, \
             {} element(s) centred inside a hole {:?}",
            outside.len(),
            &outside[..outside.len().min(4)],
            in_hole.len(),
            &in_hole[..in_hole.len().min(4)],
        ));
    }
    Ok(())
}

/// No element may be degenerate or inverted.
pub fn check_orientation(triangulator: &Triangulator) -> Result<(), String> {
    let mut degenerate = 0usize;
    let mut worst_area = f64::INFINITY;

    for triangle in solid_triangles(triangulator).iter() {
        if orientation(&triangle.v1, &triangle.v2, &triangle.v3) == Orientation::Colinear {
            degenerate += 1;
            worst_area = worst_area.min(area_triangle(&triangle.v1, &triangle.v2, &triangle.v3).abs());
        }
    }

    if degenerate > 0 {
        return Err(format!(
            "{} degenerate element(s); smallest area {:.3e}",
            degenerate, worst_area
        ));
    }
    Ok(())
}

/// Each undirected edge borders one element (on the domain boundary) or two
/// (in the interior). Three or more means the mesh is not a manifold surface.
pub fn check_edge_manifoldness(triangulator: &Triangulator) -> Result<(), String> {
    let mut usage: HashMap<(u64, u64, u64, u64), usize> = HashMap::new();

    for triangle in solid_triangles(triangulator).iter() {
        for (a, b) in [
            (&triangle.v1, &triangle.v2),
            (&triangle.v2, &triangle.v3),
            (&triangle.v3, &triangle.v1),
        ] {
            // Undirected key: endpoints sorted by their bit patterns, so the
            // same edge from two neighbouring elements collides deliberately.
            let mut ends = [
                (a.x.to_bits(), a.y.to_bits()),
                (b.x.to_bits(), b.y.to_bits()),
            ];
            ends.sort();
            *usage
                .entry((ends[0].0, ends[0].1, ends[1].0, ends[1].1))
                .or_insert(0) += 1;
        }
    }

    let overused = usage.values().filter(|count| **count > 2).count();
    if overused > 0 {
        return Err(format!(
            "{} edge(s) shared by more than two elements -- the mesh overlaps itself",
            overused
        ));
    }
    Ok(())
}

/// An edge bordered by exactly one element must lie on the domain boundary.
///
/// This is what localises a gap. Area conservation says *how much* coverage is
/// missing; it cannot say where. An interior edge with only one element beside
/// it is the rim of a hole in the mesh, so listing those points straight at the
/// problem.
pub fn check_gaps(triangulator: &Triangulator, domain: &Domain) -> Result<(), String> {
    let boundary = to_polyline(&domain.boundary);
    let holes: Vec<Rc<Polyline>> = domain.holes.iter().map(|h| to_polyline(h)).collect();

    let mut usage: HashMap<(u64, u64, u64, u64), (usize, (f64, f64))> = HashMap::new();

    for triangle in solid_triangles(triangulator).iter() {
        for (a, b) in [
            (&triangle.v1, &triangle.v2),
            (&triangle.v2, &triangle.v3),
            (&triangle.v3, &triangle.v1),
        ] {
            let mut ends = [
                (a.x.to_bits(), a.y.to_bits()),
                (b.x.to_bits(), b.y.to_bits()),
            ];
            ends.sort();
            let key = (ends[0].0, ends[0].1, ends[1].0, ends[1].1);
            let midpoint = ((a.x + b.x) / 2.0, (a.y + b.y) / 2.0);
            usage.entry(key).or_insert((0, midpoint)).0 += 1;
        }
    }

    let mut rims: Vec<String> = usage
        .values()
        .filter(|(count, _)| *count == 1)
        .filter(|(_, (x, y))| {
            let midpoint = Vertex::new(*x, *y);
            // On the outer boundary, or on any hole edge, is expected.
            let on_outer = boundary.contains(&midpoint) == Some(Continence::Boundary);
            let on_hole = holes
                .iter()
                .any(|hole| hole.contains(&midpoint) == Some(Continence::Boundary));
            !on_outer && !on_hole
        })
        .map(|(_, (x, y))| format!("({:.4}, {:.4})", x, y))
        .collect();

    if !rims.is_empty() {
        rims.sort();
        return Err(format!(
            "{} interior edge(s) border only one element, so the mesh has a gap. \
             Edge midpoints: {:?}",
            rims.len(),
            &rims[..rims.len().min(6)]
        ));
    }
    Ok(())
}

/// Runs every invariant and reports all failures at once.
///
/// Reporting them together matters: a boundary failure usually trips several
/// checks, and seeing which combination fired says more about the cause than
/// any single assertion would.
pub fn assert_mesh_invariants(triangulator: &Triangulator, domain: &Domain, context: &str) {
    let checks: Vec<(&str, Result<(), String>)> = vec![
        ("area conservation", check_area_conservation(triangulator, domain)),
        ("domain membership", check_domain_membership(triangulator, domain)),
        ("orientation", check_orientation(triangulator)),
        ("edge manifoldness", check_edge_manifoldness(triangulator)),
        ("gaps", check_gaps(triangulator, domain)),
    ];

    let failures: Vec<String> = checks
        .into_iter()
        .filter_map(|(name, result)| result.err().map(|why| format!("  [{}] {}", name, why)))
        .collect();

    if !failures.is_empty() {
        panic!(
            "{} / {}: {} invariant(s) violated\n{}",
            domain.name,
            context,
            failures.len(),
            failures.join("\n")
        );
    }
}
