mod invariants;
use invariants::Domain;

fn rotate(points: &[[f64; 2]], degrees: f64) -> Vec<[f64; 2]> {
    let n = points.len() as f64;
    let cx = points.iter().map(|p| p[0]).sum::<f64>() / n;
    let cy = points.iter().map(|p| p[1]).sum::<f64>() / n;
    let (s, c) = degrees.to_radians().sin_cos();
    points.iter().map(|[x, y]| {
        let (dx, dy) = (x - cx, y - cy);
        [cx + dx * c - dy * s, cy + dx * s + dy * c]
    }).collect()
}

/// Smallest known reproducer: a triangle rotated two degrees.
pub fn rotated_triangle() -> Domain {
    Domain {
        name: "triangle_rot_2deg".to_string(),
        boundary: rotate(&[[0.0, 0.0], [60.0, 0.0], [30.0, 52.0]], 2.0),
        holes: vec![],
    }
}

/// KNOWN FAILURE — the minimal reproducer for refinement inflating the mesh.
///
/// A three-vertex triangle rotated by two degrees, refined to min angle 25 with
/// max area 50. Exactly-axis-aligned shapes never fail; half a degree of
/// rotation is enough. Not monotonic in the perturbation, so it is particular
/// configurations rather than off-axis coordinates as such.
///
/// Traced to a single operation. Inserting a segment midpoint -- here at
/// t = 1/8 along the bottom edge, after three successive splits -- removes one
/// triangle from the cavity and creates three, for a net area gain of +682.5 on
/// a 1560 domain. Shewchuk's rule for this case is that splitting a segment
/// splits the single triangle it bounds into two. Removing one and adding three
/// that cover more area than the one removed means the new elements overlap
/// elements that were never taken out.
///
/// Ruled out, each by measurement rather than argument:
///   - elements escaping the domain: zero elements land outside the boundary
///     when the area jumps
///   - the same vertex being inserted twice in one call: instrumented, none
///   - `Triangle`'s Hash/Eq contract violation: real and now fixed, but this
///     fails identically with and without that fix
///   - `Vertex`'s Hash/Eq violation (approx_eq equality against exact-bit
///     hashing): making equality exact changes nothing here
///
/// Next: instrument the cavity walk in `solve_conflicts` for this one insertion
/// and find which cavity boundary edges produce triangles over ground that is
/// still occupied.
#[test]
#[ignore = "known: refinement inflates area; minimal reproducer"]
fn minimal_case_conserves_area() {
    let d = rotated_triangle();
    let t = d.mesh(25.0, Some(50.0));
    invariants::assert_mesh_invariants(&t, &d, "min_angle 25deg, max_area 50");
}
