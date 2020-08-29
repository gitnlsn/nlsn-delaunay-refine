use crate::elements::{edge::*, polyline::*, triangle::*, vertex::*};
use crate::planar::{triangulation::*, triangulation_procedures};
use crate::properties::continence::Continence;

use std::collections::HashSet;
use std::rc::Rc;

/**
 * Include hole and returns included segments
 */
pub fn include(
    triangulation: &mut Triangulation,
    hole: &Rc<Polyline>,
    segment_constraints: &HashSet<Rc<Edge>>,
) {
    let hole_segments: HashSet<Rc<Edge>> = hole
        .into_edges()
        .iter()
        .cloned()
        .collect::<HashSet<Rc<Edge>>>();

    /* Inserts hole vertices */
    triangulation_procedures::vertices::include(
        triangulation,
        hole.vertices.iter().cloned().collect(),
        segment_constraints,
        &None,
        &HashSet::new(),
    );

    let segment_constraints = segment_constraints
        .iter()
        .chain(hole_segments.iter())
        .cloned()
        .collect();

    // Uncomment to debug
    // let existing_segments: HashSet<Rc<Edge>> = triangulation.edges();
    // let missing_segments: Vec<Rc<Edge>> = hole_segments
    //     .iter()
    //     .filter(|&e| !existing_segments.contains(e))
    //     .cloned()
    //     .collect();

    // println!("\n\nMissing segments");
    // for e in missing_segments.iter() {
    //     println!("{}", e);
    // }

    /* Inserts missing segments */
    loop {
        let existing_segments: HashSet<Rc<Edge>> = triangulation.edges();
        let missing_segment = hole_segments
            .iter()
            .find(|&e| !existing_segments.contains(e));

        if missing_segment.is_none() {
            break;
        }

        triangulation_procedures::segment::include(
            triangulation,
            missing_segment.unwrap(),
            &segment_constraints,
        );
    }

    /* Inserting ghost triangles into holes */
    let mut pending_edges: Vec<Rc<Edge>> = Vec::new();
    let ghost_vertex = Rc::new(Vertex::new_ghost());

    for hole_edge in hole_segments.iter() {
        let edge_to_hole = Rc::clone(&hole_edge);
        if triangulation.adjacency.contains_key(&edge_to_hole) {
            let inner_triangle = Rc::clone(triangulation.adjacency.get(&edge_to_hole).unwrap());

            triangulation.remove_triangle(&inner_triangle);

            let (e1, e2, e3) = inner_triangle.inner_edges();
            if !hole_segments.contains(&e1) {
                pending_edges.push(Rc::new(e1.opposite()));
            }
            if !hole_segments.contains(&e2) {
                pending_edges.push(Rc::new(e2.opposite()));
            }
            if !hole_segments.contains(&e3) {
                pending_edges.push(Rc::new(e3.opposite()));
            }
        }

        triangulation.include_triangle(&Rc::new(Triangle::new(
            &edge_to_hole.v1,
            &edge_to_hole.v2,
            &ghost_vertex,
        )));
    }

    /* Flood fill - removes possible deeper triangles */
    while !pending_edges.is_empty() {
        let edge_to_hole = Rc::clone(&pending_edges.pop().unwrap());
        if triangulation.adjacency.contains_key(&edge_to_hole) {
            let inner_triangle = Rc::clone(triangulation.adjacency.get(&edge_to_hole).unwrap());
            triangulation.remove_triangle(&inner_triangle);

            let (e1, e2, e3) = inner_triangle.inner_edges();
            pending_edges.push(Rc::new(e1.opposite()));
            pending_edges.push(Rc::new(e2.opposite()));
            pending_edges.push(Rc::new(e3.opposite()));
        }
    }
} /* end - include holes */

#[cfg(test)]
mod include_hole {
    use super::*;

    #[test]
    fn sample_1() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(5.0, 1.0));
        let v3 = Rc::new(Vertex::new(5.0, 5.0));
        let v4 = Rc::new(Vertex::new(1.0, 5.0));

        let boundary: Vec<Rc<Vertex>> = vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ];
        let boundary = Rc::new(Polyline::new_closed(boundary).unwrap());

        /* square hole */
        let v5 = Rc::new(Vertex::new(3.0, 2.0));
        let v6 = Rc::new(Vertex::new(4.0, 3.0));
        let v7 = Rc::new(Vertex::new(3.0, 4.0));
        let v8 = Rc::new(Vertex::new(2.0, 3.0));

        let hole: Vec<Rc<Vertex>> = vec![
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ];
        let hole = Rc::new(Polyline::new_closed(hole).unwrap());

        let mut triangulation = Triangulation::from_initial_segment((&v1, &v2));
        triangulation_procedures::boundary::include(&mut triangulation, &boundary, &HashSet::new());

        triangulation_procedures::hole::include(
            &mut triangulation,
            &hole,
            &boundary.into_edges().iter().cloned().collect(),
        );

        let ghost_triangles: Vec<Rc<Triangle>> = triangulation
            .triangles
            .iter()
            .filter(|triangle| triangle.is_ghost())
            .cloned()
            .collect();

        let ghost_vertex = Rc::new(Vertex::new_ghost());
        let ghost_56 = Rc::new(Triangle::new(&v5, &v6, &ghost_vertex));
        let ghost_67 = Rc::new(Triangle::new(&v6, &v7, &ghost_vertex));
        let ghost_78 = Rc::new(Triangle::new(&v7, &v8, &ghost_vertex));
        let ghost_85 = Rc::new(Triangle::new(&v8, &v5, &ghost_vertex));

        assert!(ghost_triangles.contains(&ghost_56));
        assert!(ghost_triangles.contains(&ghost_67));
        assert!(ghost_triangles.contains(&ghost_78));
        assert!(ghost_triangles.contains(&ghost_85));

        let solid_triangles: Vec<Rc<Triangle>> = triangulation
            .triangles
            .iter()
            .filter(|t| !t.is_ghost())
            .cloned()
            .collect();

        for t in solid_triangles.iter() {
            let center = Rc::new(t.center());
            assert!(boundary.contains(&center) == Some(Continence::Inside));
            assert!(hole.contains(&center) == Some(Continence::Outside));
        }
    } /* end - sample_1 */

    #[test]
    fn sample_2() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(6.0, 1.0));
        let v3 = Rc::new(Vertex::new(6.0, 5.0));
        let v4 = Rc::new(Vertex::new(1.0, 5.0));

        let boundary: Vec<Rc<Vertex>> = vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ];
        let boundary = Rc::new(Polyline::new_closed(boundary).unwrap());

        /* hexagonal hole */
        let v5 = Rc::new(Vertex::new(3.0, 2.0));
        let v6 = Rc::new(Vertex::new(4.0, 2.0));
        let v7 = Rc::new(Vertex::new(5.0, 3.0));
        let v8 = Rc::new(Vertex::new(4.0, 4.0));
        let v9 = Rc::new(Vertex::new(3.0, 4.0));
        let v10 = Rc::new(Vertex::new(2.0, 3.0));

        let hole: Vec<Rc<Vertex>> = vec![
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
            Rc::clone(&v9),
            Rc::clone(&v10),
        ];
        let hole = Rc::new(Polyline::new_closed(hole).unwrap());

        let mut triangulation = Triangulation::from_initial_segment((&v1, &v2));
        triangulation_procedures::boundary::include(&mut triangulation, &boundary, &HashSet::new());
        triangulation_procedures::hole::include(
            &mut triangulation,
            &hole,
            &boundary.into_edges().iter().cloned().collect(),
        );

        let ghost_triangles: Vec<Rc<Triangle>> = triangulation
            .triangles
            .iter()
            .filter(|triangle| triangle.is_ghost())
            .cloned()
            .collect();

        let ghost_vertex = Rc::new(Vertex::new_ghost());
        let ghost_5 = Rc::new(Triangle::new(&v5, &v6, &ghost_vertex));
        let ghost_6 = Rc::new(Triangle::new(&v6, &v7, &ghost_vertex));
        let ghost_7 = Rc::new(Triangle::new(&v7, &v8, &ghost_vertex));
        let ghost_8 = Rc::new(Triangle::new(&v8, &v9, &ghost_vertex));
        let ghost_9 = Rc::new(Triangle::new(&v9, &v10, &ghost_vertex));
        let ghost_10 = Rc::new(Triangle::new(&v10, &v5, &ghost_vertex));

        assert!(ghost_triangles.contains(&ghost_5));
        assert!(ghost_triangles.contains(&ghost_6));
        assert!(ghost_triangles.contains(&ghost_7));
        assert!(ghost_triangles.contains(&ghost_8));
        assert!(ghost_triangles.contains(&ghost_9));
        assert!(ghost_triangles.contains(&ghost_10));

        let solid_triangles: Vec<Rc<Triangle>> = triangulation
            .triangles
            .iter()
            .filter(|t| !t.is_ghost())
            .cloned()
            .collect();

        for t in solid_triangles.iter() {
            let center = Rc::new(t.center());
            assert!(boundary.contains(&center) == Some(Continence::Inside));
            assert!(hole.contains(&center) == Some(Continence::Outside));
        }
    } /* end - sample2 */

    #[test]
    fn sample_3() {
        /* square boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(8.0, 1.0));
        let v3 = Rc::new(Vertex::new(8.0, 7.0));
        let v4 = Rc::new(Vertex::new(1.0, 7.0));

        let boundary: Vec<Rc<Vertex>> = vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ];
        let boundary = Rc::new(Polyline::new_closed(boundary).unwrap());

        /* concave hole */
        let v5 = Rc::new(Vertex::new(2.0, 6.0));
        let v6 = Rc::new(Vertex::new(2.0, 2.0));
        let v7 = Rc::new(Vertex::new(7.0, 2.0));
        let v8 = Rc::new(Vertex::new(7.0, 6.0));
        let v9 = Rc::new(Vertex::new(4.0, 6.0));
        let v10 = Rc::new(Vertex::new(4.0, 4.0));
        let v11 = Rc::new(Vertex::new(5.0, 4.0));
        let v12 = Rc::new(Vertex::new(5.0, 5.0));
        let v13 = Rc::new(Vertex::new(6.0, 5.0));
        let v14 = Rc::new(Vertex::new(6.0, 3.0));
        let v15 = Rc::new(Vertex::new(3.0, 3.0));
        let v16 = Rc::new(Vertex::new(3.0, 6.0));

        let hole: Vec<Rc<Vertex>> = vec![
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
            Rc::clone(&v9),
            Rc::clone(&v10),
            Rc::clone(&v11),
            Rc::clone(&v12),
            Rc::clone(&v13),
            Rc::clone(&v14),
            Rc::clone(&v15),
            Rc::clone(&v16),
        ];
        let hole = Rc::new(Polyline::new_closed(hole).unwrap());

        let mut triangulation = Triangulation::from_initial_segment((&v1, &v2));
        triangulation_procedures::boundary::include(&mut triangulation, &boundary, &HashSet::new());
        triangulation_procedures::hole::include(
            &mut triangulation,
            &hole,
            &boundary.into_edges().iter().cloned().collect(),
        );

        let solid_triangles: Vec<Rc<Triangle>> = triangulation
            .triangles
            .iter()
            .filter(|t| !t.is_ghost())
            .cloned()
            .collect();

        for t in solid_triangles.iter() {
            let center = Rc::new(t.center());
            assert!(boundary.contains(&center) == Some(Continence::Inside));
            assert!(hole.contains(&center) == Some(Continence::Outside));
        }
    } /* end - sample_3 */
} /* end - include_holes tests */
