//! Flat, index-based mesh export.
//!
//! The triangulation kernel stores triangles as `HashSet<Rc<Triangle>>` with an
//! `Rc<Edge>` adjacency map, which suits incremental insertion but is the wrong
//! shape for everything downstream: GPU buffers want index arrays, and a
//! discontinuous-Galerkin solver wants contiguous per-element storage. This
//! module walks the kernel structures once and produces that layout.
//!
//! Output ordering is deterministic. Iterating a `HashSet` would reintroduce
//! run-to-run variation in the vertex numbering even though the mesh geometry
//! is now stable, and stable numbering is what makes exported meshes and solver
//! results comparable between runs.

use crate::elements::triangle::Triangle;
use crate::planar::triangulation::Triangulation;
use crate::properties::orientation::{orientation_tol, Orientation, DEGENERATE_ELEMENT_EPS};

use std::cmp::Ordering;
use std::collections::HashMap;
use std::rc::Rc;

#[derive(Debug, Clone, Default)]
pub struct FlatMesh {
    /// Vertex coordinates, interleaved as `[x0, y0, x1, y1, ...]`.
    pub vertices: Vec<f64>,
    /// Triangle corner indices into `vertices`, as `[i0, j0, k0, i1, j1, k1, ...]`.
    pub triangles: Vec<u32>,
    /// Smallest interior angle of each triangle, in degrees. Parallel to
    /// `triangles`; this is what drives the quality histogram in the UI.
    pub min_angles_deg: Vec<f64>,
}

/// Sorted corner coordinates, giving a total order on triangles that does not
/// depend on hashing or on which rotation of the triangle we happen to hold.
fn ordering_key(triangle: &Triangle) -> [(u64, u64); 3] {
    let mut corners = [
        (triangle.v1.x.to_bits(), triangle.v1.y.to_bits()),
        (triangle.v2.x.to_bits(), triangle.v2.y.to_bits()),
        (triangle.v3.x.to_bits(), triangle.v3.y.to_bits()),
    ];
    corners.sort();
    corners
}

/// Interior angles of a triangle in degrees, via the law of cosines.
///
/// Deliberately not `properties::angle::angle`, which returns a *directed*
/// angle and reports `2*pi - theta` for clockwise winding. Interior angles are
/// always in `(0, pi)`, and mesh quality reporting must not depend on the
/// winding of the triangle it happens to be handed.
fn interior_angles_deg(triangle: &Triangle) -> [f64; 3] {
    let (v1, v2, v3) = (&triangle.v1, &triangle.v2, &triangle.v3);

    let len = |a: &crate::elements::vertex::Vertex, b: &crate::elements::vertex::Vertex| {
        ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt()
    };

    // Side lengths opposite to v1, v2, v3 respectively.
    let a = len(v2, v3);
    let b = len(v3, v1);
    let c = len(v1, v2);

    let angle_from = |opposite: f64, s1: f64, s2: f64| -> f64 {
        let denominator = 2.0 * s1 * s2;
        if denominator == 0.0 {
            return 0.0;
        }
        // Rounding can push this fractionally outside [-1, 1] on near-degenerate
        // triangles, where acos would return NaN and silently poison the
        // histogram.
        let cos = ((s1.powi(2) + s2.powi(2) - opposite.powi(2)) / denominator).clamp(-1.0, 1.0);
        cos.acos().to_degrees()
    };

    [
        angle_from(a, b, c),
        angle_from(b, c, a),
        angle_from(c, a, b),
    ]
}

impl FlatMesh {
    pub fn from_triangulation(triangulation: &Triangulation) -> Self {
        // Ghost triangles are the kernel's representation of the convex hull's
        // outer side; they carry a synthetic vertex and are not real elements.
        //
        // Exactly-degenerate triangles are dropped as well. Refinement splits
        // constrained segments at their midpoints, and the cavity
        // retriangulation can then emit a "triangle" whose three corners are
        // three collinear points along one original boundary edge. Such an
        // element has zero area and no circumcentre; passing it to a solver
        // would produce a singular element matrix.
        //
        // Dropping them cannot open a gap in the domain, precisely because they
        // cover no area -- the region is already covered by the neighbouring
        // elements. This is containment, not a fix: the kernel should not be
        // constructing them at all.
        let mut solid: Vec<Rc<Triangle>> = triangulation
            .triangles
            .iter()
            .filter(|t| !t.is_ghost())
            // Tolerant rather than exact: a sliver with a 1e-30 area is not
            // exactly collinear, so an exact test lets it through, yet it is
            // just as unusable as a truly zero-area element. The tolerance is
            // scale-invariant (a bound on sin of the apex angle), so this means
            // the same thing regardless of the coordinates in use.
            .filter(|t| {
                orientation_tol(&t.v1, &t.v2, &t.v3, DEGENERATE_ELEMENT_EPS)
                    != Orientation::Colinear
            })
            .cloned()
            .collect();

        solid.sort_by(|a, b| ordering_key(a).partial_cmp(&ordering_key(b)).unwrap_or(Ordering::Equal));

        let mut vertices: Vec<f64> = Vec::new();
        let mut triangles: Vec<u32> = Vec::with_capacity(solid.len() * 3);
        let mut min_angles_deg: Vec<f64> = Vec::with_capacity(solid.len());
        let mut index_of: HashMap<(u64, u64), u32> = HashMap::new();

        for triangle in solid.iter() {
            for vertex in [&triangle.v1, &triangle.v2, &triangle.v3] {
                // Keyed on exact bit patterns rather than the epsilon-based
                // PartialEq on Vertex: two coordinates that compare equal under
                // a tolerance but differ in bits would otherwise collapse into
                // one index and silently weld unrelated parts of the mesh.
                let key = (vertex.x.to_bits(), vertex.y.to_bits());
                let next = index_of.len() as u32;
                let index = *index_of.entry(key).or_insert_with(|| {
                    vertices.push(vertex.x);
                    vertices.push(vertex.y);
                    next
                });
                triangles.push(index);
            }

            let angles = interior_angles_deg(triangle);
            min_angles_deg.push(angles.iter().cloned().fold(f64::INFINITY, f64::min));
        }

        Self {
            vertices,
            triangles,
            min_angles_deg,
        }
    }

    pub fn vertex_count(&self) -> usize {
        self.vertices.len() / 2
    }

    pub fn triangle_count(&self) -> usize {
        self.triangles.len() / 3
    }

    /// Smallest interior angle anywhere in the mesh, in degrees.
    pub fn min_angle_deg(&self) -> Option<f64> {
        self.min_angles_deg
            .iter()
            .cloned()
            .fold(None, |acc: Option<f64>, a| Some(acc.map_or(a, |m| m.min(a))))
    }

    /// Counts of triangles whose minimum angle falls in each equal-width bucket
    /// spanning 0..60 degrees.
    ///
    /// 60 is the ceiling because the smallest angle of any triangle is at most
    /// 60 degrees, attained only by the equilateral case -- so a histogram over
    /// 0..90 would leave a third of it permanently empty.
    pub fn angle_histogram(&self, buckets: usize) -> Vec<u32> {
        let mut histogram = vec![0u32; buckets.max(1)];
        let width = 60.0 / buckets.max(1) as f64;

        for angle in self.min_angles_deg.iter() {
            let bucket = ((angle / width) as usize).min(histogram.len() - 1);
            histogram[bucket] += 1;
        }

        histogram
    }
}

#[cfg(test)]
mod flat_mesh {
    use super::*;
    use crate::elements::polyline::Polyline;
    use crate::elements::vertex::Vertex;
    use crate::planar::refine_params::RefineParams;
    use crate::planar::triangulator::Triangulator;

    fn unit_square() -> Triangulator {
        let corners = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        let vertices = corners
            .iter()
            .map(|(x, y)| Rc::new(Vertex::new(*x, *y)))
            .collect();
        let boundary = Rc::new(Polyline::new_closed(vertices).unwrap());

        let mut triangulator = Triangulator::new(&boundary);
        triangulator.triangulate();
        triangulator
    }

    #[test]
    fn indices_are_in_range_and_well_formed() {
        let triangulator = unit_square();
        let mesh = FlatMesh::from_triangulation(&triangulator.triangulation.borrow());

        assert!(mesh.triangle_count() > 0);
        assert_eq!(mesh.triangles.len() % 3, 0);
        assert_eq!(mesh.min_angles_deg.len(), mesh.triangle_count());

        for index in mesh.triangles.iter() {
            assert!(
                (*index as usize) < mesh.vertex_count(),
                "index {} out of range for {} vertices",
                index,
                mesh.vertex_count()
            );
        }

        // No triangle may reference the same vertex twice.
        for corner in mesh.triangles.chunks(3) {
            assert_ne!(corner[0], corner[1]);
            assert_ne!(corner[1], corner[2]);
            assert_ne!(corner[2], corner[0]);
        }
    }

    #[test]
    fn unrefined_unit_square_is_two_triangles() {
        let triangulator = unit_square();
        let mesh = FlatMesh::from_triangulation(&triangulator.triangulation.borrow());

        assert_eq!(mesh.triangle_count(), 2);
        assert_eq!(mesh.vertex_count(), 4);
    }

    #[test]
    fn angles_sum_to_180_per_triangle() {
        let triangulator = unit_square();
        let triangulation = triangulator.triangulation.borrow();

        for triangle in triangulation.triangles.iter().filter(|t| !t.is_ghost()) {
            let sum: f64 = interior_angles_deg(triangle).iter().sum();
            assert!(
                (sum - 180.0).abs() < 1.0E-9,
                "interior angles summed to {}",
                sum
            );
        }
    }

    #[test]
    fn refinement_raises_the_minimum_angle() {
        let corners = [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0)];
        let vertices = corners
            .iter()
            .map(|(x, y)| Rc::new(Vertex::new(*x, *y)))
            .collect();
        let boundary = Rc::new(Polyline::new_closed(vertices).unwrap());

        let mut triangulator = Triangulator::new(&boundary);
        triangulator.triangulate();
        triangulator.refine(RefineParams::from_min_angle_deg(20.0, Some(0.2)));

        let mesh = FlatMesh::from_triangulation(&triangulator.triangulation.borrow());

        assert!(mesh.triangle_count() > 10, "expected a refined mesh");
        assert!(
            mesh.min_angle_deg().unwrap() > 15.0,
            "refined mesh still has a {} degree angle",
            mesh.min_angle_deg().unwrap()
        );
    }

    #[test]
    fn histogram_counts_every_triangle() {
        let triangulator = unit_square();
        let mesh = FlatMesh::from_triangulation(&triangulator.triangulation.borrow());

        let histogram = mesh.angle_histogram(12);
        let total: u32 = histogram.iter().sum();
        assert_eq!(total as usize, mesh.triangle_count());
    }
}
