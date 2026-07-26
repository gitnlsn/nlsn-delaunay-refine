//! A red/green matrix over the smallest shapes that fail.
//!
//! The mesher is correct on axis-aligned geometry and degrades on rotated
//! geometry, but not monotonically in the rotation — so a single reproducer is
//! misleading. Sweeping rotation against refinement and printing the whole grid
//! shows where the boundary between green and red actually lies, which is the
//! thing worth reasoning about.
//!
//! Each cell is meshed area divided by domain area. 1.0000 is correct, above is
//! overlap, below is a gap.
//!
//! Read it with [`determinism_matrix`] in mind: the mesher is currently
//! non-deterministic, so every cell off the axis-aligned row is one sample of a
//! distribution rather than a fixed value, and re-running changes them. What is
//! stable across runs is the shape of the grid -- the 0 degree row correct, the
//! rest degrading as refinement gets finer.

mod invariants;
use invariants::{solid_triangles, Domain};
use nlsn_delaunay::properties::area::area_triangle;

/// Rotate about the centroid, so the shape stays put and only its alignment
/// with the axes changes. Rotating about the origin would move it as well, and
/// then a red cell could not be attributed to the rotation alone.
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

/// Mesh area as a multiple of domain area. 1.0000 is correct; above is overlap
/// or spill, below is a gap. The ratio is what a letter cannot say -- a 0.01%
/// shortfall and a 2x overlap are both "area conservation failed", and they are
/// not the same bug.
fn ratio(domain: &Domain, max_area: f64) -> String {
    let t = domain.mesh(25.0, Some(max_area));
    let meshed: f64 = solid_triangles(&t)
        .iter()
        .map(|tri| area_triangle(&tri.v1, &tri.v2, &tri.v3).abs())
        .sum();
    format!("{:.4}", meshed / domain.expected_area())
}

const AREAS: [f64; 6] = [200.0, 100.0, 50.0, 25.0, 12.0, 6.0];
const ANGLES: [f64; 9] = [0.0, 0.25, 0.5, 1.0, 2.0, 5.0, 15.0, 30.0, 45.0];

fn sweep(label: &str, base: &[[f64; 2]]) {
    println!("\n{:<20}{}", label, AREAS.iter().map(|a| format!("{:>9}", a)).collect::<String>());
    for degrees in ANGLES {
        let domain = Domain {
            name: format!("{}_{}", label, degrees),
            boundary: rotate(base, degrees),
            holes: vec![],
        };
        let row: String = AREAS.iter().map(|a| format!("{:>9}", ratio(&domain, *a))).collect();
        println!("  rot {:>5} deg      {}", degrees, row);
    }
}

/// Not an assertion — a diagnostic. Run with `--nocapture --include-ignored`.
#[test]
#[ignore = "diagnostic matrix; run with --nocapture"]
fn red_green_matrix() {
    sweep("square", &[[20.0, 20.0], [80.0, 20.0], [80.0, 80.0], [20.0, 80.0]]);
    sweep("triangle", &[[20.0, 20.0], [80.0, 20.0], [50.0, 72.0]]);
    sweep("quad", &[[25.8, 22.5], [81.4, 25.7], [76.2, 67.0], [25.4, 64.3]]);
}
