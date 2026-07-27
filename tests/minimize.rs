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

/// The minimal reproducer for refinement inflating the mesh. **Now passing.**
///
/// A three-vertex triangle rotated by two degrees, refined to min angle 25 with
/// max area 50. Exactly-axis-aligned shapes never failed; half a degree of
/// rotation was enough. Not monotonic in the perturbation, so it was particular
/// configurations rather than off-axis coordinates as such.
///
/// Traced to a single operation. Inserting a segment midpoint -- here at
/// t = 1/8 along the bottom edge, after three successive splits -- removed one
/// triangle from the cavity and created three, for a net area gain of +682.5 on
/// a 1560 domain. Shewchuk's rule for this case is that splitting a segment
/// splits the single triangle it bounds into two.
///
/// The third element was the segment itself: a "triangle" of the segment's two
/// endpoints and its midpoint, with zero area. It covered nothing, which is why
/// area conservation could not see it and why the note below concluded the
/// numbers did not add up. What it did was overwrite the two real halves'
/// entries in the adjacency map.
///
/// Ruled out at the time, each by measurement rather than argument, and all
/// still correct:
///   - elements escaping the domain: zero elements land outside the boundary
///     when the area jumps
///   - the same vertex being inserted twice in one call: instrumented, none
///   - `Triangle`'s Hash/Eq contract violation
///   - `Vertex`'s Hash/Eq violation
///
/// The next step recorded here -- "instrument the cavity walk in
/// `solve_conflicts` for this one insertion" -- is what `tests/stepwise.rs` now
/// does for every insertion, and is how this was closed.
#[test]
fn minimal_case_conserves_area() {
    let d = rotated_triangle();
    let t = d.mesh(25.0, Some(50.0));
    invariants::assert_mesh_invariants(&t, &d, "min_angle 25deg, max_area 50");
}
