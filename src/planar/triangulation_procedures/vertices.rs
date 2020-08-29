use crate::elements::{edge::*, polyline::*, triangle::*, vertex::*};
use crate::planar::{triangulation::*, triangulation_procedures};
use crate::properties::continence::*;

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

/**
 * Inserts vertices in the triangulation.
 * Implements Bowyer-Watson incremental insersion using conflict map.
 * Insersion will be avoided the possible triangle violates boundary
 * hole or segment constraints.
 */
pub fn include(
    triangulation: &mut Triangulation,
    vertices: Vec<Rc<Vertex>>,
    segment_constraints: &HashSet<Rc<Edge>>,
    boundary: &Option<Rc<Polyline>>,
    holes: &HashSet<Rc<Polyline>>,
) {
    let existing_vertices: HashSet<Rc<Vertex>> = triangulation.vertices();
    let mut vertices: Vec<Rc<Vertex>> = vertices
        .iter()
        .filter(|&v| !existing_vertices.contains(v))
        .cloned()
        .collect();

    let mut conflict_map: HashMap<Rc<Triangle>, Vec<Rc<Vertex>>> = HashMap::new();
    for possible_triangle in triangulation.triangles.iter() {
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

        let next_conflicting_triangle: Rc<Triangle> =
            Rc::clone(conflict_map.keys().next().unwrap());

        let mut conflicting_vertices: Vec<Rc<Vertex>> =
            conflict_map.remove(&next_conflicting_triangle).unwrap();

        let conflict_vertex: Rc<Vertex> = conflicting_vertices.pop().unwrap();

        triangulation.remove_triangle(&next_conflicting_triangle);

        let (e1, e2, e3) = next_conflicting_triangle.inner_edges();

        let mut pending_cavities: Vec<Rc<Edge>> = vec![e1, e2, e3];

        if !conflicting_vertices.is_empty() {
            /* reinclude conflicts they remaining */
            vertices.append(&mut conflicting_vertices);
        }

        while !pending_cavities.is_empty() {
            let edge: Rc<Edge> = pending_cavities.pop().unwrap();
            let edge_to_outer_triangle: Rc<Edge> = Rc::new(edge.opposite());

            let outer_triangle: Rc<Triangle> = Rc::clone(
                triangulation
                    .adjacency
                    .get(&edge_to_outer_triangle)
                    .unwrap(),
            );

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
                let (e12, e23, e31) = outer_triangle.inner_edges();

                if let Some(mut conflicting_vertices) = conflict_map.remove(&outer_triangle) {
                    vertices.append(&mut conflicting_vertices);
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
                if edge.v1.is_ghost {
                    let new_triangle = Rc::new(Triangle::new(&edge.v2, &conflict_vertex, &edge.v1));
                    triangulation.include_triangle(&new_triangle);

                    distribute_conflicts(
                        &new_triangle,
                        &mut conflict_map,
                        &mut vertices,
                        boundary,
                        holes,
                    );
                } else if edge.v2.is_ghost {
                    let new_triangle = Rc::new(Triangle::new(&conflict_vertex, &edge.v1, &edge.v2));
                    triangulation.include_triangle(&new_triangle);

                    distribute_conflicts(
                        &new_triangle,
                        &mut conflict_map,
                        &mut vertices,
                        boundary,
                        holes,
                    );
                } else {
                    let new_triangle = Rc::new(Triangle::new(&edge.v1, &edge.v2, &conflict_vertex));
                    triangulation.include_triangle(&new_triangle);

                    distribute_conflicts(
                        &new_triangle,
                        &mut conflict_map,
                        &mut vertices,
                        boundary,
                        holes,
                    );
                }
            }
        } /* end - while pending edges */
    } /* end - distributing vertices */
} /* end - include vertices method */

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
        conflict_map.insert(Rc::clone(&triangle), distributed_conflicts);
    }
} /* end - distribute conflicts  */

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
        if let Some((continence, _)) = Polyline::continence(&hole, &p2) {
            is_outside_holes = continence != Continence::Inside;
        } else {
            is_outside_holes = false;
        }
    }

    return is_inside_boundary && is_outside_holes;
} /* end - may_insert_triangle */

#[cfg(test)]
mod include_vertices {
    use super::*;

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
        let v5 = Rc::new(Vertex::new(3.0, 2.0));
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
    } /* end - sample 1 */
} /* end - distribute conflits tests */
