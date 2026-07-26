use crate::elements::{edge::*, polyline::*, triangle::*, vertex::*};
use crate::planar::{
    refine_params::{RefineOutcome, RefineParams},
    refine_procedures::encroachment,
    triangulation::*,
    triangulation_procedures,
};

use crate::properties::continence::*;

use std::cmp::Ordering;
use std::collections::{HashMap, HashSet};
use std::rc::Rc;

/**
 * Determines if the triangle is irregular according to quality ratio
 */
fn is_irregular_triangle(triangle: &Triangle, params: &RefineParams) -> bool {
    // A degenerate triangle has no defined circumradius, so it has no quality
    // ratio either. It cannot be improved by circumcenter insertion -- the
    // circumcenter does not exist -- so reporting it as irregular would queue
    // work that can never succeed and spin the refinement loop.
    let this_quality = match triangle.quality() {
        Some(quality) => quality,
        None => return false,
    };

    let no_quality = float_cmp::approx_eq!(
        f64,
        this_quality,
        params.quality_ratio,
        epsilon = 1.0E-14f64
    ) || this_quality >= params.quality_ratio;

    return no_quality;
}

/**
 * Determines if the triangle is larger than threshould
 */
fn is_large_triangle(triangle: &Triangle, params: &RefineParams) -> bool {
    // Degenerate triangles have zero area; they are never "too large".
    let this_area = match triangle.area() {
        Some(area) => area,
        None => return false,
    };

    let greater_area: bool = match params.max_area {
        Some(max_area) => {
            float_cmp::approx_eq!(f64, this_area, max_area, epsilon = 1.0E-14f64)
                || this_area >= max_area
        }
        _ => false,
    };

    return greater_area;
}

/// A total, hash-independent ordering key for a triangle.
///
/// `HashSet::iter().next()` returns an arbitrary element, and Rust seeds its
/// default hasher randomly per process, so selecting the next triangle to split
/// that way makes the output mesh differ between runs on identical input. Since
/// a solver will eventually consume these meshes, refinement has to be
/// reproducible, so selection is driven by geometry instead.
fn ordering_key(triangle: &Triangle) -> [f64; 6] {
    let mut corners = [
        (triangle.v1.x, triangle.v1.y),
        (triangle.v2.x, triangle.v2.y),
        (triangle.v3.x, triangle.v3.y),
    ];
    // Sorted so that the key is invariant to which rotation of the triangle we
    // happen to hold.
    corners.sort_by(|a, b| {
        a.0.partial_cmp(&b.0)
            .unwrap_or(Ordering::Equal)
            .then_with(|| a.1.partial_cmp(&b.1).unwrap_or(Ordering::Equal))
    });

    [
        corners[0].0,
        corners[0].1,
        corners[1].0,
        corners[1].1,
        corners[2].0,
        corners[2].1,
    ]
}

/// Picks the worst triangle in the set, deterministically.
///
/// Worst-first is also the better refinement order: it attacks the elements
/// driving the quality bound rather than whichever one the hasher surfaced.
fn take_worst(set: &mut HashSet<Rc<Triangle>>) -> Option<Rc<Triangle>> {
    let worst = set
        .iter()
        .max_by(|a, b| {
            let qa = a.quality().unwrap_or(f64::NEG_INFINITY);
            let qb = b.quality().unwrap_or(f64::NEG_INFINITY);
            qa.partial_cmp(&qb)
                .unwrap_or(Ordering::Equal)
                .then_with(|| {
                    ordering_key(a)
                        .partial_cmp(&ordering_key(b))
                        .unwrap_or(Ordering::Equal)
                })
        })
        .cloned()?;

    set.remove(&worst);
    Some(worst)
}

/**
 * Refines specified triangles, according to Rupperts refinement.
 * If the triangle's circumcenter is encroached, it splits the corresponding
 * encroaching edge. Else it inserts the circumcenter into the triangulation.
 * A Modified triangle won't be touched if, a prior vertex insertion removes it.
 */
pub fn split_irregular(
    triangulation: &mut Triangulation,
    params: &RefineParams,
    segment_contraints: &HashSet<Rc<Edge>>,
    boundary: &Option<Rc<Polyline>>,
    holes: &HashSet<Rc<Polyline>>,
) -> (HashMap<Rc<Edge>, Rc<Edge>>, RefineOutcome) {
    let mut segment_contraints: HashSet<Rc<Edge>> = segment_contraints.iter().cloned().collect();

    let critical_triangles = triangulation
        .triangles
        .iter()
        .filter(|t| !t.is_ghost())
        .filter(|t| is_irregular_triangle(t, params) || is_large_triangle(t, params))
        .cloned()
        .collect::<HashSet<Rc<Triangle>>>();

    let mut irregular_triangles: HashSet<Rc<Triangle>> = critical_triangles
        .iter()
        .filter(|t| is_irregular_triangle(t, params))
        .cloned()
        .collect();

    let mut large_triangles: HashSet<Rc<Triangle>> = critical_triangles
        .iter()
        .filter(|t| is_large_triangle(t, params))
        .cloned()
        .collect();

    let mut split_map: HashMap<Rc<Edge>, Rc<Edge>> = HashMap::new();

    let mut outcome = RefineOutcome::Converged;
    let mut steps: usize = 0;
    let mut unimprovable: usize = 0;

    loop {
        // Bounded so a non-terminating configuration degrades into a usable
        // mesh plus a flag, instead of freezing the caller. In the browser this
        // loop runs inside a worker, where an unbounded spin is unrecoverable.
        if steps >= params.max_steps {
            outcome = RefineOutcome::HitStepLimit;
            break;
        }
        // O(1) on the triangle set. Counting distinct vertices here instead
        // would walk and deduplicate every triangle on every iteration.
        if triangulation.triangles.len() >= params.max_triangles {
            outcome = RefineOutcome::HitSizeLimit;
            break;
        }
        steps += 1;

        let triangle;
        if let Some(irregular_triangle) = take_worst(&mut irregular_triangles) {
            triangle = irregular_triangle;
        } else if let Some(large_triangle) = take_worst(&mut large_triangles) {
            triangle = large_triangle;
        } else {
            break;
        }

        // Uncomment to debug
        // println!(
        //     "\nSplitting Triangle (Total: {} + {}) - {}",
        //     irregular_triangles.len(),
        //     large_triangles.len(),
        //     triangle.quality().unwrap()
        // );

        // println!("\nTriangles");
        // for t in triangulation.triangles.iter().filter(|t| !t.is_ghost()) {
        //     println!("{}", t);
        // }

        match try_circumcenter_insertion(
            triangulation,
            &triangle,
            &segment_contraints,
            boundary,
            holes,
        ) {
            // The triangle has no circumcenter to insert. It was already
            // excluded from the bad-triangle sets, so simply move on rather
            // than unwrapping a None and taking the whole worker down.
            // Neither can be acted on: one has no circumcenter, the other has
            // one that is not in the domain. Both were already taken out of the
            // bad-triangle sets, so move on rather than queueing work that can
            // never succeed.
            // Neither can be acted on: one has no circumcenter, the other has
            // one that is not in the domain, and the encroachment fallback in
            // `try_circumcenter_insertion` already came back empty. The element
            // stays as it is.
            //
            // What must not happen is calling that convergence. `take_worst`
            // has already removed this element from the work-set and nothing
            // puts it back, so the loop will drain and report success while the
            // element sits below the requested angle. Counted here, and carried
            // out in the outcome.
            InsertionOutcome::Degenerate | InsertionOutcome::OutsideDomain => {
                unimprovable += 1;
                continue;
            }

            InsertionOutcome::Inserted {
                included: included_triangles,
                removed: removed_triangles,
            } => {
                for new_triangle in included_triangles.iter().filter(|t| !t.is_ghost()) {
                    if is_irregular_triangle(new_triangle, params) {
                        irregular_triangles.insert(Rc::clone(new_triangle));
                        continue;
                    }
                    if is_large_triangle(new_triangle, params) {
                        large_triangles.insert(Rc::clone(new_triangle));
                        continue;
                    }
                }
                for old_triangle in removed_triangles.iter() {
                    irregular_triangles.remove(old_triangle);
                    large_triangles.remove(old_triangle);
                }
            }
            InsertionOutcome::Encroached {
                edges: encroachments,
                circumcenter,
            } => {
                // Carried out of the insertion attempt rather than recomputed,
                // which also removes a second unwrap on the same Option.
                let mut vertices = vec![circumcenter].iter().cloned().collect();

                for encroached_edge in encroachments.iter() {
                    let (new_edges, included_triangles, removed_triangles) =
                        encroachment::unencroach_segment(
                            triangulation,
                            &encroached_edge,
                            &mut vertices,
                            &segment_contraints,
                            boundary,
                            holes,
                        );

                    segment_contraints.remove(encroached_edge);
                    split_map.remove(encroached_edge);
                    for subsegment in new_edges.iter() {
                        split_map.insert(Rc::clone(subsegment), Rc::clone(encroached_edge));
                        segment_contraints.insert(Rc::clone(subsegment));
                    }

                    for new_triangle in included_triangles.iter().filter(|t| !t.is_ghost()) {
                        if is_irregular_triangle(new_triangle, params) {
                            irregular_triangles.insert(Rc::clone(new_triangle));
                            continue;
                        }
                        if is_large_triangle(new_triangle, params) {
                            large_triangles.insert(Rc::clone(new_triangle));
                            continue;
                        }
                    }
                    for old_triangle in removed_triangles.iter() {
                        irregular_triangles.remove(old_triangle);
                        large_triangles.remove(old_triangle);
                    }
                }
            }
        }
    }
    // Convergence is a claim about the whole mesh, so it cannot be made while
    // elements are known to be short of the target. A limit takes precedence:
    // running out of steps is the more specific fact about what happened.
    if unimprovable > 0 && outcome == RefineOutcome::Converged {
        outcome = RefineOutcome::LeftUnimprovable(unimprovable);
    }

    return (split_map, outcome);
} /* end - split */

/// Result of attempting to insert a triangle's circumcenter.
enum InsertionOutcome {
    Inserted {
        included: HashSet<Rc<Triangle>>,
        removed: HashSet<Rc<Triangle>>,
    },
    /// The circumcenter would encroach these constrained segments, so they must
    /// be split instead. Carries the circumcenter so the caller does not have
    /// to recompute it.
    Encroached {
        edges: HashSet<Rc<Edge>>,
        circumcenter: Rc<Vertex>,
    },
    /// The triangle is degenerate and has no circumcenter.
    Degenerate,
    /// The circumcenter lies outside the domain, so inserting it would create
    /// elements where the domain isn't.
    OutsideDomain,
}

/// Whether a point lies within the meshed domain.
///
/// Strict: a point reported as `Boundary` is not accepted. Inserting a vertex
/// that sits on a boundary or hole edge produces zero-area elements against it,
/// and the tolerant point-in-polygon test reports near-edge points that way, so
/// strictness here is what catches circumcenters drifting out across a reflex
/// corner.
///
/// Strictness is only affordable because the caller has a fallback: a rejected
/// circumcenter causes the segments it encroaches to be split instead of
/// refinement simply giving up.
fn is_inside_domain(
    vertex: &Vertex,
    boundary: &Option<Rc<Polyline>>,
    holes: &HashSet<Rc<Polyline>>,
) -> bool {
    if let Some(boundary) = boundary {
        if boundary.contains(vertex) != Some(Continence::Inside) {
            return false;
        }
    }

    for hole in holes.iter() {
        if hole.contains(vertex) != Some(Continence::Outside) {
            return false;
        }
    }

    true
}

/**
 * Tries to insert a triangle's circumcenter.
 * If any edge of the triangle is constrained and encroaches the circumcenter,
 * it is returned in the hashset. The circumcenter will be inserted through constrained
 * insertion. Among the included triangles, if any is composed by a constrained segment
 * that encroaches the circumcenter, the segment is returned in the hashset. If there is
 * no encroachments, the returnable is empty.
 */
fn try_circumcenter_insertion(
    triangulation: &mut Triangulation,
    triangle: &Rc<Triangle>,
    segment_constraints: &HashSet<Rc<Edge>>,
    boundary: &Option<Rc<Polyline>>,
    holes: &HashSet<Rc<Polyline>>,
) -> InsertionOutcome {
    // Degenerate triangles have no circumcenter. Reported rather than
    // unwrapped: this is reachable on real input, and a panic here aborts the
    // wasm instance.
    let circumcenter = match triangle.circumcenter() {
        Some(circumcenter) => Rc::new(circumcenter),
        None => return InsertionOutcome::Degenerate,
    };

    // A triangle's circumcenter is not required to lie inside the domain, and
    // for an element sitting against a reflex corner it routinely does not.
    // Inserting it regardless is how elements end up outside the boundary: the
    // cavity is retriangulated around a point in a region that is not part of
    // the domain, and every triangle built against it inherits that.
    //
    // Ruppert's algorithm rejects such a circumcenter rather than inserting it.
    // Skipping leaves this element unrefined -- so quality near reflex corners
    // stays worse than the target -- but the mesh remains a partition of the
    // domain, which is the stronger guarantee.
    if !is_inside_domain(&circumcenter, boundary, holes) {
        // Ruppert's rule has two halves, and rejecting the circumcenter is only
        // the first. The second is to split the constrained segments it
        // encroaches instead -- which pulls the boundary discretisation finer
        // exactly where the geometry is awkward, and lets the next pass find a
        // circumcenter that does lie inside.
        //
        // Rejecting without the split is what made a strict domain test
        // unusable: refinement declined the insertion, had no other move, and
        // converged early. A plain unit square stopped at fewer than ten
        // elements.
        let encroached: HashSet<Rc<Edge>> = segment_constraints
            .iter()
            .filter(|edge| edge.encroach(&circumcenter) == Continence::Inside)
            .cloned()
            .collect();

        if encroached.is_empty() {
            // Nothing to split, so there is no way to make progress on this
            // element. Leaving it unrefined keeps the mesh conformant.
            return InsertionOutcome::OutsideDomain;
        }

        return InsertionOutcome::Encroached {
            edges: encroached,
            circumcenter,
        };
    }

    let mut conflict_map: HashMap<Rc<Triangle>, Vec<Rc<Vertex>>> = HashMap::new();

    triangulation_procedures::vertices::distribute_conflicts_over_triangulation(
        triangulation,
        Some(Rc::clone(triangle)),
        &mut conflict_map,
        &mut vec![Rc::clone(&circumcenter)],
        boundary,
        holes,
    );

    let (included_triangles, removed_triangles) =
        triangulation_procedures::vertices::solve_conflicts(
            triangulation,
            &mut conflict_map,
            &mut Vec::new(),
            segment_constraints,
            boundary,
            holes,
        );

    let encroachments: HashSet<Rc<Edge>> = included_triangles
        .iter()
        .map(|t| t.opposite_edge(&circumcenter).unwrap())
        .filter(|e| segment_constraints.contains(e) || segment_constraints.contains(&e.opposite()))
        .filter(|e| e.encroach(&circumcenter) == Continence::Inside)
        .collect();

    if !encroachments.is_empty() {
        for t in included_triangles.iter() {
            triangulation.remove_triangle(t);
        }

        for t in removed_triangles.iter() {
            triangulation.include_triangle(t);
        }
        return InsertionOutcome::Encroached {
            edges: encroachments,
            circumcenter,
        };
    }

    return InsertionOutcome::Inserted {
        included: included_triangles,
        removed: removed_triangles,
    };
}

#[cfg(test)]
mod split {
    use super::*;

    #[test]
    fn sample_1() {
        let triangle_side: f64 = 1.0;
        let sqrt_3: f64 = 1.7320508075688772;
        let triangle_height = triangle_side * sqrt_3 / 2.0;

        /* triangle */
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(triangle_side, 0.0));
        let v3 = Rc::new(Vertex::new(triangle_side / 2.0, triangle_height));

        let boundary = Rc::new(
            Polyline::new_closed(vec![Rc::clone(&v1), Rc::clone(&v2), Rc::clone(&v3)]).unwrap(),
        );

        /* Encroaching vertex */
        let encroaching_vertex = Rc::new(Vertex::new(triangle_side / 2.0, triangle_height / 3.0));

        /* Triangulation */
        let mut triangulation = Triangulation::from_initial_segment((&v1, &v2));
        triangulation_procedures::boundary::include(&mut triangulation, &boundary, &HashSet::new());
        triangulation_procedures::vertices::include(
            &mut triangulation,
            vec![Rc::clone(&encroaching_vertex)],
            &HashSet::new(),
            &Some(Rc::clone(&boundary)),
            &HashSet::new(),
        );

        let mut segment_constraints: HashSet<Rc<Edge>> =
            boundary.into_edges().iter().cloned().collect();

        /* unencroach */
        let (mapping, _included_triangles, _removed_triangles) = encroachment::unencroach(
            &mut triangulation,
            &boundary.into_edges().iter().cloned().collect(),
            &Some(Rc::clone(&boundary)),
            &HashSet::new(),
        );

        segment_constraints = segment_constraints
            .iter()
            .filter(|&s| {
                let removed_segments = mapping.values().cloned().collect::<HashSet<Rc<Edge>>>();
                return removed_segments.contains(s) || removed_segments.contains(&s.opposite());
            })
            .chain(mapping.keys())
            .cloned()
            .collect();

        split_irregular(
            &mut triangulation,
            &RefineParams {
                max_area: None, /* not used */
                quality_ratio: 1.0,
                ..Default::default()
            },
            &segment_constraints,
            &Some(Rc::clone(&boundary)),
            &HashSet::new(),
        );

        let _solid_triangles: HashSet<Rc<Triangle>> = triangulation
            .triangles
            .iter()
            .filter(|t| !t.is_ghost())
            .cloned()
            .collect();

        let t1 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.5, 0.28867513459481287)),
            &Rc::new(Vertex::new(0.5, 0.0)),
            &Rc::new(Vertex::new(0.75, 0.0)),
        ));
        let t2 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.5, 0.28867513459481287)),
            &Rc::new(Vertex::new(0.75, 0.0)),
            &Rc::new(Vertex::new(0.875, 0.21650635094610965)),
        ));
        let t3 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.375, 0.649519052838329)),
            &Rc::new(Vertex::new(0.5, 0.28867513459481287)),
            &Rc::new(Vertex::new(0.625, 0.649519052838329)),
        ));
        let t4 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.25, 0.4330127018922193)),
            &Rc::new(Vertex::new(0.5, 0.28867513459481287)),
            &Rc::new(Vertex::new(0.375, 0.649519052838329)),
        ));
        let t5 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.75, 0.0)),
            &Rc::new(Vertex::new(1.0, 0.0)),
            &Rc::new(Vertex::new(0.875, 0.21650635094610965)),
        ));
        let t6 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.5, 0.8660254037844386)),
            &Rc::new(Vertex::new(0.375, 0.649519052838329)),
            &Rc::new(Vertex::new(0.625, 0.649519052838329)),
        ));
        let t7 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.5, 0.28867513459481287)),
            &Rc::new(Vertex::new(0.25, 0.4330127018922193)),
            &Rc::new(Vertex::new(0.125, 0.21650635094610965)),
        ));
        let t8 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.5, 0.0)),
            &Rc::new(Vertex::new(0.5, 0.28867513459481287)),
            &Rc::new(Vertex::new(0.25, 0.0)),
        ));
        let t9 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.75, 0.4330127018922193)),
            &Rc::new(Vertex::new(0.5, 0.28867513459481287)),
            &Rc::new(Vertex::new(0.875, 0.21650635094610965)),
        ));
        let t10 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.25, 0.0)),
            &Rc::new(Vertex::new(0.5, 0.28867513459481287)),
            &Rc::new(Vertex::new(0.125, 0.21650635094610965)),
        ));
        let t11 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.0, 0.0)),
            &Rc::new(Vertex::new(0.25, 0.0)),
            &Rc::new(Vertex::new(0.125, 0.21650635094610965)),
        ));
        let t12 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.5, 0.28867513459481287)),
            &Rc::new(Vertex::new(0.75, 0.4330127018922193)),
            &Rc::new(Vertex::new(0.625, 0.649519052838329)),
        ));

        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t1));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t2));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t3));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t4));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t5));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t6));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t7));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t8));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t9));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t10));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t11));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t12));
    }
}

#[cfg(test)]
mod try_circumcenter_insertion {
    use super::*;

    #[test]
    fn exception_1() {
        let p1 = Rc::new(Vertex::new(0.80, -0.5));
        let p2 = Rc::new(Vertex::new(0.95, -0.5));
        let p3 = Rc::new(Vertex::new(0.95, 0.5));
        let p4 = Rc::new(Vertex::new(0.80, 0.5));

        let mut triangulation = Triangulation::from_initial_segment((&p1, &p2));
        triangulation_procedures::vertices::include(
            &mut triangulation,
            vec![Rc::clone(&p3), Rc::clone(&p4)],
            &HashSet::new(),
            &None,
            &HashSet::new(),
        );

        let _s1 = Rc::new(Edge::new(&p1, &p2));
        let s2 = Rc::new(Edge::new(&p2, &p3));
        let _s3 = Rc::new(Edge::new(&p3, &p4));
        let _s4 = Rc::new(Edge::new(&p4, &p1));

        let mut segment_constraints = HashSet::new();
        segment_constraints.insert(Rc::clone(&s2));

        let boundary = Rc::new(
            Polyline::new_closed(vec![
                Rc::clone(&p1),
                Rc::clone(&p2),
                Rc::clone(&p3),
                Rc::clone(&p4),
            ])
            .unwrap(),
        );

        let t1 = Rc::clone(triangulation.triangles.iter().filter(|t| !t.is_ghost()).next().unwrap());

        let result = try_circumcenter_insertion(
            &mut triangulation,
            &t1,
            &segment_constraints,
            &Some(Rc::clone(&boundary)),
            &HashSet::new(),
        );

        match result {
            InsertionOutcome::Encroached { edges, .. } => assert!(edges.contains(&s2)),
            InsertionOutcome::Inserted { .. } => panic!("expected the circumcenter to encroach s2"),
            InsertionOutcome::Degenerate => panic!("triangle unexpectedly degenerate"),
            InsertionOutcome::OutsideDomain => {
                panic!("circumcenter unexpectedly outside the domain")
            }
        }
    }
}
