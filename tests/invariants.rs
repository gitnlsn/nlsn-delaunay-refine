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

use nlsn_delaunay::elements::polyline::{vertex_pairs, Polyline};
use nlsn_delaunay::elements::triangle::Triangle;
use nlsn_delaunay::elements::vertex::Vertex;
use nlsn_delaunay::planar::refine_params::RefineParams;
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
    pub name: &'static str,
    pub boundary: Vec<[f64; 2]>,
    pub holes: Vec<Vec<[f64; 2]>>,
}

fn to_polyline(points: &[[f64; 2]]) -> Rc<Polyline> {
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
        triangulator.refine(params);

        triangulator
    }
}

fn solid_triangles(triangulator: &Triangulator) -> Vec<Rc<Triangle>> {
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
