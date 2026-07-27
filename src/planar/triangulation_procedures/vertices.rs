use crate::elements::{edge::*, polyline::*, triangle::*, vertex::*};
use crate::planar::refine_procedures::trace::{corners_of, Event};
use crate::planar::triangulation::*;
use crate::properties::continence::*;
use crate::properties::orientation::{
    orientation, orientation_tol, Orientation, DEGENERATE_ELEMENT_EPS,
};

use std::cmp::Ordering;
use std::collections::{HashMap, HashSet};
use std::rc::Rc;

/**
 * Inserts vertices in the triangulation.
 */
pub fn include(
    triangulation: &mut Triangulation,
    vertices: Vec<Rc<Vertex>>,
    segment_constraints: &HashSet<Rc<Edge>>,
    boundary: &Option<Rc<Polyline>>,
    holes: &HashSet<Rc<Polyline>>,
) -> (HashSet<Rc<Triangle>>, HashSet<Rc<Triangle>>) {
    let existing_vertices: HashSet<Rc<Vertex>> = triangulation.vertices();
    let mut vertices: Vec<Rc<Vertex>> = vertices
        .iter()
        .filter(|&v| !existing_vertices.contains(v)) /* filters existing vertices */
        .cloned()
        .collect();

    let mut conflict_map: HashMap<Rc<Triangle>, Vec<Rc<Vertex>>> = HashMap::new();

    // Ordered rather than taken as the hasher gives them. `distribute_conflicts`
    // files each vertex under the first triangle that claims it, so the walk
    // order decides the contents of the conflict map.
    let mut candidates: Vec<Rc<Triangle>> = triangulation.triangles.iter().cloned().collect();
    candidates.sort_by(|a, b| {
        let key = |t: &Rc<Triangle>| {
            let mut corners = [(t.v1.x, t.v1.y), (t.v2.x, t.v2.y), (t.v3.x, t.v3.y)];
            corners.sort_by(|p, q| {
                p.0.partial_cmp(&q.0)
                    .unwrap_or(Ordering::Equal)
                    .then_with(|| p.1.partial_cmp(&q.1).unwrap_or(Ordering::Equal))
            });
            corners
        };
        a.is_ghost()
            .cmp(&b.is_ghost())
            .then_with(|| key(a).partial_cmp(&key(b)).unwrap_or(Ordering::Equal))
    });

    for possible_triangle in candidates.iter() {
        if vertices.is_empty() {
            break;
        }
        distribute_conflicts(
            possible_triangle,
            &mut conflict_map,
            &mut vertices,
            boundary,
            holes,
        );
    }

    return solve_conflicts(
        triangulation,
        &mut conflict_map,
        &mut vertices,
        segment_constraints,
        boundary,
        holes,
    );
} /* end - include vertices method */

/// The next vertex to insert: the smallest by coordinate, over every vertex the
/// conflict map holds.
///
/// Deterministic on purpose. Selecting with `conflict_map.keys().next()` handed
/// the choice to the hasher, and Rust seeds every `HashMap` differently, so the
/// same request produced a different mesh on every call.
fn next_vertex(conflict_map: &HashMap<Rc<Triangle>, Vec<Rc<Vertex>>>) -> Option<Rc<Vertex>> {
    conflict_map
        .values()
        .flatten()
        .min_by(|a, b| {
            a.x.partial_cmp(&b.x)
                .unwrap_or(Ordering::Equal)
                .then_with(|| a.y.partial_cmp(&b.y).unwrap_or(Ordering::Equal))
        })
        .cloned()
}

/// Whether a point lies inside a triangle, or on its border.
///
/// Uses the exact orientation predicate, which `tests/alignment_axioms.rs`
/// establishes is invariant under rotation -- so this answers the same way
/// wherever the shape sits relative to the axes. A point is inside when it is on
/// the same side of all three directed edges; `Colinear` counts as either side,
/// which is what admits points lying on an edge.
fn triangle_contains(triangle: &Rc<Triangle>, vertex: &Rc<Vertex>) -> bool {
    if triangle.is_ghost() {
        return false;
    }

    let sides = [
        orientation(&triangle.v1, &triangle.v2, vertex),
        orientation(&triangle.v2, &triangle.v3, vertex),
        orientation(&triangle.v3, &triangle.v1, vertex),
    ];

    let any_clockwise = sides.iter().any(|s| *s == Orientation::Clockwise);
    let any_counterclockwise = sides.iter().any(|s| *s == Orientation::Counterclockwise);

    !(any_clockwise && any_counterclockwise)
}

/// The triangle to grow the cavity from, for a given vertex.
///
/// Candidates are restricted to triangles the conflict map lists this vertex
/// under, which is what guarantees the seed genuinely conflicts with the point.
/// Widening the search to the whole triangulation was tried and is wrong: on a
/// triangulation that is still only an initial segment there are no solid
/// triangles at all, so the containment search finds nothing, and any fallback
/// outside the conflicting set removes a triangle that should have stayed --
/// which left the insertion building nothing.
///
/// Among those candidates, the one *containing* the point is preferred. That is
/// Bowyer-Watson's precondition: growing the cavity from the triangle the point
/// lies in is what makes it star-shaped about the point, and so what makes the
/// fan built against it valid rather than folded over itself.
fn seed_triangle(
    conflict_map: &HashMap<Rc<Triangle>, Vec<Rc<Vertex>>>,
    vertex: &Rc<Vertex>,
) -> Option<Rc<Triangle>> {
    let key = |triangle: &Rc<Triangle>| {
        let mut corners = [
            (triangle.v1.x, triangle.v1.y),
            (triangle.v2.x, triangle.v2.y),
            (triangle.v3.x, triangle.v3.y),
        ];
        corners.sort_by(|a, b| {
            a.0.partial_cmp(&b.0)
                .unwrap_or(Ordering::Equal)
                .then_with(|| a.1.partial_cmp(&b.1).unwrap_or(Ordering::Equal))
        });
        corners
    };

    let candidates: Vec<&Rc<Triangle>> = conflict_map
        .iter()
        .filter(|(_, vertices)| vertices.iter().any(|held| **held == **vertex))
        .map(|(triangle, _)| triangle)
        .collect();

    let containing = candidates
        .iter()
        .filter(|triangle| triangle_contains(triangle, vertex))
        .min_by(|a, b| key(a).partial_cmp(&key(b)).unwrap_or(Ordering::Equal))
        .map(|triangle| Rc::clone(triangle));

    if containing.is_some() {
        return containing;
    }

    let distance_to = |triangle: &Rc<Triangle>| {
        let centre = triangle.center();
        (centre.x - vertex.x).powi(2) + (centre.y - vertex.y).powi(2)
    };

    candidates
        .iter()
        .min_by(|a, b| {
            // Solid triangles first: a ghost carries a synthetic corner, so its
            // centre is not a position in the domain and says nothing about how
            // near it is.
            a.is_ghost()
                .cmp(&b.is_ghost())
                .then_with(|| {
                    distance_to(a)
                        .partial_cmp(&distance_to(b))
                        .unwrap_or(Ordering::Equal)
                })
                .then_with(|| key(a).partial_cmp(&key(b)).unwrap_or(Ordering::Equal))
        })
        .map(|triangle| Rc::clone(triangle))
}

/**
 * Implements Bowyer-Watson incremental insersion using conflict map.
 * Insersion will be avoided the possible triangle violates boundary
 * hole or segment constraints.
 */
pub fn solve_conflicts(
    triangulation: &mut Triangulation,
    conflict_map: &mut HashMap<Rc<Triangle>, Vec<Rc<Vertex>>>,
    remaining_vertices: &mut Vec<Rc<Vertex>>,
    segment_constraints: &HashSet<Rc<Edge>>,
    boundary: &Option<Rc<Polyline>>,
    holes: &HashSet<Rc<Polyline>>,
) -> (HashSet<Rc<Triangle>>, HashSet<Rc<Triangle>>) {
    let mut included_triangles: HashSet<Rc<Triangle>> = HashSet::new();
    let mut removed_triangles: HashSet<Rc<Triangle>> = HashSet::new();

    while !conflict_map.is_empty() {
        // Uncommet to debug
        // println!("\n\nExisting Triangles");
        // for t in triangulation.triangles.iter() {
        //     println!("{}", t);
        // }

        // println!("\nConflict Map");
        // for (t, vs) in conflict_map.iter() {
        //     println!("{}", t);
        //     for v in vs.iter() {
        //         print!("{}", v);
        //     }
        //     println!();
        // }

        // Which vertex to insert, and which triangle to open the cavity from,
        // are chosen deliberately rather than taken from hash order.
        //
        // Bowyer-Watson wants the cavity grown from the triangle *containing*
        // the point. That is what makes the cavity star-shaped about it, and so
        // what makes retriangulating it against the point produce a valid fan.
        // Growing from an arbitrary triangle that merely conflicts with some
        // vertex is safe in an unconstrained Delaunay triangulation, where the
        // conflicting set is connected -- but here `may_insert_triangle` and the
        // segment constraints prune it, so the walk could start in a component
        // nowhere near the point. The fan then folds over itself, which is where
        // the overlapping elements and the inflated area came from.
        let conflict_vertex: Rc<Vertex> = match next_vertex(conflict_map) {
            Some(vertex) => vertex,
            None => break,
        };

        let next_conflicting_triangle: Rc<Triangle> =
            match seed_triangle(conflict_map, &conflict_vertex) {
                Some(triangle) => triangle,
                None => break,
            };

        // The vertex may be filed under a different triangle than the one the
        // cavity opens from, so it is taken out wherever it sits.
        let mut conflicting_vertices: Vec<Rc<Vertex>> = conflict_map
            .remove(&next_conflicting_triangle)
            .unwrap_or_default();
        // Compared by value, not by pointer. The same geometric vertex can be
        // held as more than one `Rc` -- the conflict map is built by walking
        // triangles, and a vertex reached twice arrives as two handles -- so a
        // pointer comparison would leave a copy behind, and the loop would
        // insert it again on the next pass. `Vertex` now has a lawful identity,
        // so value equality is the right test.
        conflicting_vertices.retain(|vertex| **vertex != *conflict_vertex);
        for vertices in conflict_map.values_mut() {
            vertices.retain(|vertex| **vertex != *conflict_vertex);
        }
        conflict_map.retain(|_, vertices| !vertices.is_empty());

        triangulation.remove_triangle(&next_conflicting_triangle);
        removed_triangles.insert(Rc::clone(&next_conflicting_triangle));

        let (e1, e2, e3) = next_conflicting_triangle.inner_edges();

        let mut pending_cavities: Vec<Rc<Edge>> = vec![e1, e2, e3];

        if !conflicting_vertices.is_empty() {
            /* reinclude conflicts */
            remaining_vertices.append(&mut conflicting_vertices);
        }

        while !pending_cavities.is_empty() {
            let edge: Rc<Edge> = pending_cavities.pop().unwrap();
            let edge_to_outer_triangle: Rc<Edge> = Rc::new(edge.opposite());

            // A missing adjacency entry means the cavity cannot be expanded
            // across this edge -- the triangulation's edge-to-triangle map has
            // gone inconsistent, which is reachable on degenerate input. Skip
            // the edge and carry on: the cavity ends up slightly smaller than
            // ideal, which costs mesh quality locally, whereas unwrapping costs
            // the entire wasm instance.
            let outer_triangle: Rc<Triangle> =
                match triangulation.adjacency.get(&edge_to_outer_triangle) {
                    Some(triangle) => Rc::clone(triangle),
                    None => {
                        triangulation.record(Event::AdjacencyMissing {
                            edge: [
                                (edge_to_outer_triangle.v1.x, edge_to_outer_triangle.v1.y),
                                (edge_to_outer_triangle.v2.x, edge_to_outer_triangle.v2.y),
                            ],
                        });
                        continue;
                    }
                };

            let mut is_conflicting =
                outer_triangle.encircles(&conflict_vertex) == Continence::Inside;

            if outer_triangle.is_ghost() && !is_conflicting {
                let outer_edge = Edge::new(&outer_triangle.v1, &outer_triangle.v2);
                is_conflicting = outer_edge.contains(&conflict_vertex);
                // Uncomment to debug
                // println!(
                //     "{} {} conflicting: {}",
                //     outer_triangle, conflict_vertex, is_conflicting
                // );
            }

            let is_constrained = segment_constraints.contains(&edge_to_outer_triangle)
                || segment_constraints.contains(&edge);

            let may_insert =
                may_insert_triangle(&outer_triangle, &conflict_vertex, boundary, holes);

            // Uncomment to debug
            // println!(
            //     "{}, {} x {}: {} {} {}",
            //     edge,
            //     outer_triangle,
            //     conflict_vertex,
            //     is_conflicting,
            //     is_constrained,
            //     may_insert,
            // );

            if is_conflicting && !is_constrained && may_insert {
                triangulation.remove_triangle(&outer_triangle);
                removed_triangles.insert(Rc::clone(&outer_triangle));

                let (e12, e23, e31) = outer_triangle.inner_edges();

                if let Some(mut conflicting_vertices) = conflict_map.remove(&outer_triangle) {
                    remaining_vertices.append(&mut conflicting_vertices);
                }

                /* includes cavities */
                if edge_to_outer_triangle == e12 {
                    pending_cavities.push(e23);
                    pending_cavities.push(e31);
                } else if edge_to_outer_triangle == e23 {
                    pending_cavities.push(e12);
                    pending_cavities.push(e31);
                } else {
                    pending_cavities.push(e12);
                    pending_cavities.push(e23);
                }
            } else {
                /* Includes new triangle */
                let new_triangle: Rc<Triangle>;
                if edge.v1.is_ghost {
                    new_triangle = Rc::new(Triangle::new(&edge.v2, &conflict_vertex, &edge.v1));
                } else if edge.v2.is_ghost {
                    new_triangle = Rc::new(Triangle::new(&conflict_vertex, &edge.v1, &edge.v2));
                } else {
                    new_triangle = Rc::new(Triangle::new(&edge.v1, &edge.v2, &conflict_vertex));
                }
                // A triangle whose three corners are collinear has no area and no
                // circumcentre. This is not a rare degeneracy: it is what the
                // fan produces whenever the inserted point lies on a rim edge of
                // the cavity, which is exactly what splitting a boundary segment
                // at its midpoint does. The two halves of the split already tile
                // the region completely -- `tests/stepwise.rs` measures 195 + 195
                // against the 390 removed -- so this third element is pure
                // surplus, and refusing it cannot open a gap.
                //
                // Admitting one corrupts `Triangulation::adjacency` instead. Its
                // three edges are the same edges as the real halves', so the map
                // ends up pointing at a zero-area element, and every later cavity
                // walk that crosses the boundary there reads a lie.
                //
                // Tolerant rather than exact, and this is the whole of the
                // difference between axis-aligned input and hand-drawn input.
                // Points along an axis-aligned segment are exactly
                // representable, so their midpoints are too, and the run of them
                // left by repeated splitting is *exactly* collinear -- the exact
                // predicate returns zero and this refuses. Turn the same shape a
                // quarter of a degree and the identical run of points is only
                // collinear to within rounding: the exact predicate reports a
                // genuine turn of about 1e-16, and the element is admitted. The
                // geometry is the same geometry; only its representation moved.
                //
                // Shares its bound with the export, which drops the same
                // elements on the way out. See `DEGENERATE_ELEMENT_EPS`.
                //
                // Ghost triangles carry a synthetic vertex and are structural,
                // so the test does not apply to them.
                let is_degenerate = !new_triangle.is_ghost()
                    && orientation_tol(
                        &new_triangle.v1,
                        &new_triangle.v2,
                        &new_triangle.v3,
                        DEGENERATE_ELEMENT_EPS,
                    ) == Orientation::Colinear;

                if is_degenerate {
                    triangulation.record(Event::Refused {
                        corners: corners_of(&new_triangle),
                    });
                    continue;
                }

                triangulation.include_triangle(&new_triangle);
                included_triangles.insert(Rc::clone(&new_triangle));

                // Only ever called for a triangle that is now in the
                // triangulation. `distribute_conflicts` files the triangle as a
                // *key* in the conflict map, and the loop above picks its cavity
                // seed from those keys -- so distributing against a refused
                // triangle hands the next insertion a seed that does not exist,
                // whose `remove_triangle` silently does nothing and whose edges
                // resolve against an adjacency map that has never heard of them.
                // That is why tightening the guard above made the mesh worse
                // before this line moved.
                distribute_conflicts(
                    &new_triangle,
                    conflict_map,
                    remaining_vertices,
                    boundary,
                    holes,
                );
            }
        } /* end - while pending edges */
    } /* end - distributing vertices */

    return (included_triangles, removed_triangles);
}

/**
 * Includes into the conflit_map the conflicts between the given triangle
 * and the available vertices. The conflict will be avoided if it violates
 * boudanry or hole constraints.
 */
fn distribute_conflicts(
    triangle: &Rc<Triangle>,
    conflict_map: &mut HashMap<Rc<Triangle>, Vec<Rc<Vertex>>>,
    vertices: &mut Vec<Rc<Vertex>>,
    boundary: &Option<Rc<Polyline>>,
    holes: &HashSet<Rc<Polyline>>,
) {
    let mut distributed_conflicts: Vec<Rc<Vertex>> = Vec::new();

    for _ in 0..vertices.len() {
        let pending_vertex: Rc<Vertex> = vertices.remove(0);
        let has_conflict = triangle.encircles(&pending_vertex) == Continence::Inside;

        let may_insert = may_insert_triangle(&triangle, &pending_vertex, boundary, holes);

        if has_conflict && may_insert {
            distributed_conflicts.push(pending_vertex);
            continue;
        }

        vertices.push(pending_vertex);
    }
    if !distributed_conflicts.is_empty() {
        if conflict_map.contains_key(triangle) {
            let existing_conflicts: &mut Vec<Rc<Vertex>> = conflict_map.get_mut(triangle).unwrap();

            existing_conflicts.append(&mut distributed_conflicts);
        } else {
            conflict_map.insert(Rc::clone(triangle), distributed_conflicts);
        }
    }
} /* end - distribute conflicts  */

pub fn distribute_conflicts_over_triangulation(
    triangulation: &mut Triangulation,
    initial_triangle: Option<Rc<Triangle>>,
    conflict_map: &mut HashMap<Rc<Triangle>, Vec<Rc<Vertex>>>,
    vertices: &mut Vec<Rc<Vertex>>,
    boundary: &Option<Rc<Polyline>>,
    holes: &HashSet<Rc<Polyline>>,
) {
    let mut visited_triangles: HashSet<Rc<Triangle>> = HashSet::new();
    let mut pending_triangles: Vec<Rc<Triangle>> = Vec::new();

    if let Some(initial_triangle) = initial_triangle {
        visited_triangles.insert(Rc::clone(&initial_triangle));
        pending_triangles.push(Rc::clone(&initial_triangle));
    } else {
        // Deterministic rather than whichever the hasher surfaces: this is the
        // seed of a breadth-first walk, so it decides which triangle claims each
        // vertex in the conflict map, and through that which cavities open.
        let initial_triangle = triangulation
            .triangles
            .iter()
            .min_by(|a, b| {
                let key = |t: &Rc<Triangle>| {
                    let mut corners = [
                        (t.v1.x, t.v1.y),
                        (t.v2.x, t.v2.y),
                        (t.v3.x, t.v3.y),
                    ];
                    corners.sort_by(|p, q| {
                        p.0.partial_cmp(&q.0)
                            .unwrap_or(Ordering::Equal)
                            .then_with(|| p.1.partial_cmp(&q.1).unwrap_or(Ordering::Equal))
                    });
                    corners
                };
                a.is_ghost()
                    .cmp(&b.is_ghost())
                    .then_with(|| key(a).partial_cmp(&key(b)).unwrap_or(Ordering::Equal))
            })
            .unwrap();
        visited_triangles.insert(Rc::clone(&initial_triangle));
        pending_triangles.push(Rc::clone(&initial_triangle));
    }

    while !pending_triangles.is_empty() && !vertices.is_empty() {
        let next_triangle = pending_triangles.pop().unwrap();
        let (e1, e2, e3) = next_triangle.outer_edges();
        for outer_edge in vec![e1, e2, e3].iter() {
            if let Some(outer_triangle) = triangulation.adjacency.get(outer_edge) {
                if !visited_triangles.contains(outer_triangle) && !outer_triangle.is_ghost() {
                    pending_triangles.push(Rc::clone(outer_triangle));
                    visited_triangles.insert(Rc::clone(outer_triangle));
                }
            };
        }

        distribute_conflicts(&next_triangle, conflict_map, vertices, boundary, holes);
    }
}

/**
 * Evaluates if triangle is inside boudanry and outside holes
 */
fn may_insert_triangle(
    triangle: &Rc<Triangle>,
    target_vertex: &Rc<Vertex>,
    boundary: &Option<Rc<Polyline>>,
    holes: &HashSet<Rc<Polyline>>,
) -> bool {
    let p2: Polyline;
    if triangle.is_ghost() {
        // The point lies on this ghost's own hull edge.
        //
        // That is not an edge case, it is what splitting a boundary segment at
        // its midpoint looks like from out here, and the answer is trivially
        // yes: the point is on the boundary, so nothing built against it can
        // leave the domain, and the ghost *must* be absorbed so that it splits
        // in two along with the segment.
        //
        // Answered before the polyline below is built, because that polyline
        // would be `[v1, v2, target]` with all three collinear -- a closed loop
        // with no interior. Every probe point `Polyline::continence` takes from
        // it lands on the boundary, so it can reach no verdict and returns
        // `None`, which this function reads as "may not insert". The ghost was
        // then left unsplit, nothing owned the directed edge from the midpoint
        // to the far endpoint, and the next cavity walk to reach that edge found
        // no neighbour and built nothing against it -- a gap the size of one
        // element, on the boundary, exactly where hand-drawn geometry showed
        // them. `tests/stepwise.rs` traces the chain.
        let outer_edge = Edge::new(&triangle.v1, &triangle.v2);
        if outer_edge.contains(target_vertex) {
            return true;
        }

        p2 = Polyline::new_closed(vec![
            Rc::clone(&triangle.v1),
            Rc::clone(&triangle.v2),
            Rc::clone(target_vertex),
        ])
        .unwrap();
    } else {
        p2 = Polyline::new_opened(vec![Rc::new(triangle.center()), Rc::clone(target_vertex)])
            .unwrap();
    }

    let mut is_inside_boundary = true;
    let mut is_outside_holes = true;

    if let Some(boundary) = boundary {
        if let Some((continence, _)) = Polyline::continence(&boundary, &p2) {
            is_inside_boundary = continence != Continence::Outside;
        } else {
            is_inside_boundary = false;
        }
    }

    for hole in holes.iter() {
        // Accumulated, not assigned. This used to overwrite on every iteration,
        // so only the last hole visited was ever enforced -- and `holes` is a
        // HashSet, which means *which* hole that was varied between runs. A
        // triangle sitting inside one hole was admitted as long as it was
        // outside whichever hole happened to be visited last.
        let outside_this_hole = match Polyline::continence(&hole, &p2) {
            Some((continence, _)) => continence != Continence::Inside,
            None => false,
        };
        is_outside_holes = is_outside_holes && outside_this_hole;
    }

    return is_inside_boundary && is_outside_holes;
} /* end - may_insert_triangle */

#[cfg(test)]
mod include_vertices {
    use super::*;
    use crate::planar::triangulation_procedures;

    #[test]
    fn sample_1() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(5.0, 1.0));
        let v3 = Rc::new(Vertex::new(5.0, 5.0));
        let v4 = Rc::new(Vertex::new(1.0, 5.0));
        let v5 = Rc::new(Vertex::new(3.0, 2.0));
        let v6 = Rc::new(Vertex::new(4.0, 3.0));
        let v7 = Rc::new(Vertex::new(3.0, 4.0));
        let v8 = Rc::new(Vertex::new(2.0, 3.0));

        let expected_triangles = vec![
            Rc::new(Triangle::new(&v1, &v2, &v5)),
            Rc::new(Triangle::new(&v1, &v5, &v8)),
            Rc::new(Triangle::new(&v1, &v8, &v4)),
            Rc::new(Triangle::new(&v3, &v4, &v7)),
            Rc::new(Triangle::new(&v3, &v7, &v6)),
            Rc::new(Triangle::new(&v3, &v6, &v2)),
            Rc::new(Triangle::new(&v2, &v6, &v5)),
            Rc::new(Triangle::new(&v4, &v8, &v7)),
        ];

        let vertices: HashSet<Rc<Vertex>> = vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ]
        .iter()
        .cloned()
        .collect();

        let mut triangulation = Triangulation::from_initial_segment((&v1, &v2));
        triangulation_procedures::vertices::include(
            &mut triangulation,
            vertices.iter().cloned().collect(),
            &HashSet::new(),
            &None,
            &HashSet::new(),
        );

        let vertices: HashSet<Rc<Vertex>> = triangulation.vertices();
        assert_eq!(vertices.len(), 8);
        assert!(vertices.contains(&v1));
        assert!(vertices.contains(&v2));
        assert!(vertices.contains(&v3));
        assert!(vertices.contains(&v4));
        assert!(vertices.contains(&v5));
        assert!(vertices.contains(&v6));
        assert!(vertices.contains(&v7));
        assert!(vertices.contains(&v8));

        let triangles = triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>();

        for t in expected_triangles.iter() {
            assert!(triangles.contains(t));
        }
    }

    #[test]
    fn sample_2() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(6.0, 1.0));
        let v3 = Rc::new(Vertex::new(8.0, 8.0));
        let v4 = Rc::new(Vertex::new(1.0, 6.0));

        let v5 = Rc::new(Vertex::new(2.0, 2.0));
        let v6 = Rc::new(Vertex::new(5.0, 5.0));

        let expected_triangles = vec![
            Rc::new(Triangle::new(&v2, &v5, &v1)),
            Rc::new(Triangle::new(&v2, &v6, &v5)),
            Rc::new(Triangle::new(&v2, &v3, &v6)),
            Rc::new(Triangle::new(&v4, &v1, &v5)),
            Rc::new(Triangle::new(&v4, &v5, &v6)),
            Rc::new(Triangle::new(&v4, &v6, &v3)),
        ];

        let vertices: HashSet<Rc<Vertex>> = vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ]
        .iter()
        .cloned()
        .collect();

        let s1 = Rc::new(Edge::new(&v5, &v6));

        let mut triangulation = Triangulation::from_initial_segment((&v5, &v6));
        triangulation_procedures::vertices::include(
            &mut triangulation,
            vertices.iter().cloned().collect(),
            &vec![Rc::clone(&s1)].iter().cloned().collect(),
            &None,
            &HashSet::new(),
        );

        let vertices: HashSet<Rc<Vertex>> = triangulation.vertices();
        assert_eq!(vertices.len(), 6);
        assert!(vertices.contains(&v1));
        assert!(vertices.contains(&v2));
        assert!(vertices.contains(&v3));
        assert!(vertices.contains(&v4));
        assert!(vertices.contains(&v5));
        assert!(vertices.contains(&v6));

        let triangles = triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>();

        for t in expected_triangles.iter() {
            assert!(triangles.contains(t));
        }
    }

    #[test]
    fn sample_3() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(6.0, 1.0));
        let v3 = Rc::new(Vertex::new(8.0, 8.0));
        let v4 = Rc::new(Vertex::new(1.0, 6.0));

        let v5 = Rc::new(Vertex::new(2.0, 2.0));
        let v6 = Rc::new(Vertex::new(5.0, 5.0));

        let expected_triangles = vec![
            Rc::new(Triangle::new(&v2, &v5, &v1)),
            Rc::new(Triangle::new(&v2, &v6, &v5)),
            Rc::new(Triangle::new(&v2, &v3, &v6)),
            Rc::new(Triangle::new(&v4, &v1, &v5)),
            Rc::new(Triangle::new(&v4, &v5, &v6)),
            Rc::new(Triangle::new(&v4, &v6, &v3)),
        ];

        let vertices: Vec<Rc<Vertex>> = vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ];

        let s1 = Rc::new(Edge::new(&v5, &v6));

        let boundary = Rc::new(Polyline::new_closed(vertices.iter().cloned().collect()).unwrap());

        let mut triangulation = Triangulation::from_initial_segment((&v5, &v6));
        triangulation_procedures::vertices::include(
            &mut triangulation,
            vertices.iter().cloned().collect(),
            &vec![Rc::clone(&s1)].iter().cloned().collect(),
            &Some(boundary),
            &HashSet::new(),
        );

        let vertices: HashSet<Rc<Vertex>> = triangulation.vertices();
        assert_eq!(vertices.len(), 6);
        assert!(vertices.contains(&v1));
        assert!(vertices.contains(&v2));
        assert!(vertices.contains(&v3));
        assert!(vertices.contains(&v4));
        assert!(vertices.contains(&v5));
        assert!(vertices.contains(&v6));

        let triangles = triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>();

        for t in expected_triangles.iter() {
            assert!(triangles.contains(t));
        }
    } /* end - sample_3 */
} /* end - vertices inclusion */

#[cfg(test)]
mod may_insert_triangle {
    use super::*;

    #[test]
    fn sample_1() {
        /* boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(6.0, 1.0));
        // let v3 = Rc::new(Vertex::new(6.0, 5.0));
        let v4 = Rc::new(Vertex::new(1.0, 5.0));

        /* hexagonal hole */
        let _v5 = Rc::new(Vertex::new(3.0, 2.0));
        let v6 = Rc::new(Vertex::new(4.0, 2.0));
        // let v7 = Rc::new(Vertex::new(5.0, 3.0));
        let v8 = Rc::new(Vertex::new(4.0, 4.0));
        let v9 = Rc::new(Vertex::new(3.0, 4.0));
        let v10 = Rc::new(Vertex::new(2.0, 3.0));

        let t1 = Rc::new(Triangle::new(&v9, &v6, &v8));
        let t2 = Rc::new(Triangle::new(&v10, &v4, &v1));
        let t3 = Rc::new(Triangle::new(&v10, &v1, &v6));
        let t4 = Rc::new(Triangle::new(&v6, &v1, &v2));
        let t5 = Rc::new(Triangle::new(&v9, &v10, &v6));

        let hull = Rc::new(
            Polyline::triangles_hull(
                &vec![
                    Rc::clone(&t1),
                    Rc::clone(&t2),
                    Rc::clone(&t3),
                    Rc::clone(&t4),
                    Rc::clone(&t5),
                ]
                .iter()
                .cloned()
                .collect(),
            )
            .unwrap(),
        );

        let v101 = Rc::new(Vertex::new(6.0, 1.0));
        let v102 = Rc::new(Vertex::new(2.0, 3.0));
        let g100 = Rc::new(Vertex::new_ghost());

        let possible_triangle = Rc::new(Triangle::new(&v102, &v101, &g100));
        let target_vertex = Rc::new(Vertex::new(3.0, 4.0));

        assert!(!may_insert_triangle(
            &possible_triangle,
            &target_vertex,
            &Some(hull),
            &HashSet::new()
        ));
    }
}

#[cfg(test)]
mod distribute_conflicts {
    use super::*;

    #[test]
    fn sample_1() {
        /* hexagon */
        let v1 = Rc::new(Vertex::new(1.0, 0.0));
        let v2 = Rc::new(Vertex::new(2.0, 0.0));
        let v3 = Rc::new(Vertex::new(3.0, 1.0));
        let v4 = Rc::new(Vertex::new(2.0, 2.0));
        let v5 = Rc::new(Vertex::new(1.0, 2.0));
        let v6 = Rc::new(Vertex::new(0.0, 1.0));

        let ghost_vertex = Rc::new(Vertex::new_ghost());
        let ghost_triangle = Rc::new(Triangle::new(&v1, &v2, &ghost_vertex));

        for v in vec![
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
        ]
        .iter()
        {
            let mut conflict_map: HashMap<Rc<Triangle>, Vec<Rc<Vertex>>> = HashMap::new();
            distribute_conflicts(
                &ghost_triangle,
                &mut conflict_map,
                &mut vec![Rc::clone(v)],
                &None,
                &HashSet::new(),
            );
            assert_eq!(conflict_map.len(), 1);
        }
    } /* end - sample 1 */

    #[test]
    fn sample_2() {
        /* hexagon */
        let v1 = Rc::new(Vertex::new(1.0, 0.0));
        let v2 = Rc::new(Vertex::new(2.0, 0.0));
        let v3 = Rc::new(Vertex::new(3.0, 1.0));
        let v4 = Rc::new(Vertex::new(2.0, 2.0));
        let v5 = Rc::new(Vertex::new(1.0, 2.0));
        let v6 = Rc::new(Vertex::new(0.0, 1.0));

        let boundary_vertices: Vec<Rc<Vertex>> = vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
        ];
        let boundary =
            Rc::new(Polyline::new_closed(boundary_vertices.iter().cloned().collect()).unwrap());

        let ghost_vertex = Rc::new(Vertex::new_ghost());
        let ghost_triangle = Rc::new(Triangle::new(&v1, &v2, &ghost_vertex));

        for v in vec![
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
        ]
        .iter()
        {
            let mut conflict_map: HashMap<Rc<Triangle>, Vec<Rc<Vertex>>> = HashMap::new();
            distribute_conflicts(
                &ghost_triangle,
                &mut conflict_map,
                &mut vec![Rc::clone(v)],
                &Some(Rc::clone(&boundary)),
                &HashSet::new(),
            );
            assert_eq!(conflict_map.len(), 1);
        }
    } /* end - sample 2 */
} /* end - distribute conflits tests */

#[cfg(test)]
mod distribute_conflicts_over_triangulation {
    use super::*;
    use crate::planar::triangulation_procedures;

    #[test]
    fn sample_1() {
        let sqrt_3: f64 = 1.7320508075688772;

        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(2.0, 0.0));
        let v3 = Rc::new(Vertex::new(4.0, 0.0));
        let v4 = Rc::new(Vertex::new(6.0, 0.0));
        let v5 = Rc::new(Vertex::new(5.0, sqrt_3));
        let v6 = Rc::new(Vertex::new(3.0, sqrt_3));
        let v7 = Rc::new(Vertex::new(1.0, sqrt_3));

        let v8 = Rc::new(Vertex::new(5.2, sqrt_3 / 3.0));

        let boundary = vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
        ];
        let boundary = Rc::new(Polyline::new_closed(boundary).unwrap());

        let mut triangulation = Triangulation::from_initial_segment((&v1, &v2));
        triangulation_procedures::boundary::include(&mut triangulation, &boundary, &HashSet::new());

        let triangles: HashSet<Rc<Triangle>> = triangulation.triangles.iter().cloned().collect();
        let expected_conflicting_triangle = Rc::new(Triangle::new(&v3, &v4, &v5));
        for t in triangles.iter() {
            let mut conflict_map: HashMap<Rc<Triangle>, Vec<Rc<Vertex>>> = HashMap::new();
            distribute_conflicts_over_triangulation(
                &mut triangulation,
                Some(Rc::clone(&t)),
                &mut conflict_map,
                &mut vec![Rc::clone(&v8)],
                &Some(Rc::clone(&boundary)),
                &HashSet::new(),
            );
            assert_eq!(conflict_map.len(), 1);

            assert_eq!(
                conflict_map.keys().next().unwrap(),
                &expected_conflicting_triangle
            );
        }
    }
}
