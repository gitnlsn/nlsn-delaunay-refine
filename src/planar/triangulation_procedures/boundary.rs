use crate::elements::{edge::*, polyline::*, triangle::*, vertex::*};
use crate::planar::triangulation::*;

use crate::planar::triangulation_procedures;

use std::collections::HashSet;
use std::rc::Rc;

/**
 * Inserts boundary into triangulation,
 * with incremental insertion and
 * avoids inserting triangles outside boundary
 */
pub fn include(
    triangulation: &mut Triangulation,
    boundary: &Rc<Polyline>,
    segment_constraints: &HashSet<Rc<Edge>>,
) {
    let boundary_segments: HashSet<Rc<Edge>> = boundary
        .into_edges()
        .iter()
        .map(|e| Rc::new(e.opposite()))
        .collect::<HashSet<Rc<Edge>>>();

    triangulation_procedures::vertices::include(
        triangulation,
        boundary.vertices.iter().cloned().collect(),
        segment_constraints,
        &None,
        &HashSet::new(),
    );

    /* Inserts missing segments */
    // let existing_segments: HashSet<Rc<Edge>> = triangulation.edges();
    // let missing_segments: HashSet<Rc<Edge>> = boundary_segments
    //     .iter()
    //     .filter(|&e| !existing_segments.contains(e))
    //     .cloned()
    //     .collect();

    // let segment_contraints = segment_constraints
    //     .iter()
    //     .chain(boundary_segments.iter())
    //     .cloned()
    //     .collect();

    // for segment in missing_segments.iter() {
    //     Self::include_segment(triangulation, segment, &segment_contraints);
    // }

    /* Inserting outer hole */
    let mut pending_edges: Vec<Rc<Edge>> = Vec::new();
    let ghost_vertex = Rc::new(Vertex::new_ghost());

    for boundary_edge in boundary_segments.iter() {
        let edge_to_outside = Rc::clone(&boundary_edge);
        if triangulation.adjacency.contains_key(&edge_to_outside) {
            let inner_triangle = Rc::clone(triangulation.adjacency.get(&edge_to_outside).unwrap());

            triangulation.remove_triangle(&inner_triangle);

            let (e1, e2, e3) = inner_triangle.inner_edges();
            if !boundary_segments.contains(&e1) {
                pending_edges.push(Rc::new(e1.opposite()));
            }
            if !boundary_segments.contains(&e2) {
                pending_edges.push(Rc::new(e2.opposite()));
            }
            if !boundary_segments.contains(&e3) {
                pending_edges.push(Rc::new(e3.opposite()));
            }
        }
    }

    /* Flood fill - removes possible deeper triangles */
    while !pending_edges.is_empty() {
        let edge_to_hole = Rc::clone(&pending_edges.pop().unwrap());

        if triangulation.adjacency.contains_key(&edge_to_hole) {
            let inner_triangle = Rc::clone(triangulation.adjacency.get(&edge_to_hole).unwrap());
            triangulation.remove_triangle(&inner_triangle);

            let (e1, e2, e3) = inner_triangle.inner_edges();
            if !boundary_segments.contains(&e1) {
                pending_edges.push(Rc::new(e1.opposite()));
            }
            if !boundary_segments.contains(&e2) {
                pending_edges.push(Rc::new(e2.opposite()));
            }
            if !boundary_segments.contains(&e3) {
                pending_edges.push(Rc::new(e3.opposite()));
            }
        }
    }

    for boundary_edge in boundary_segments.iter() {
        let edge_to_outside = Rc::clone(&boundary_edge);
        triangulation.include_triangle(&Rc::new(Triangle::new(
            &edge_to_outside.v1,
            &edge_to_outside.v2,
            &ghost_vertex,
        )));
    }
} /* end - include boundary */

#[cfg(test)]
mod include_boundary {
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

        let mut triangulation = Triangulation::from_initial_segment((&v1, &v2));
        include(&mut triangulation, &boundary, &HashSet::new());

        let solid_triangles: Vec<Rc<Triangle>> = triangulation
            .triangles
            .iter()
            .filter(|triangle| !triangle.is_ghost())
            .cloned()
            .collect();

        assert_eq!(solid_triangles.len(), 4);

        let ghost_triangles: Vec<Rc<Triangle>> = triangulation
            .triangles
            .iter()
            .filter(|triangle| triangle.is_ghost())
            .cloned()
            .collect();

        assert_eq!(ghost_triangles.len(), 6);
    }

    #[test]
    fn sample_2() {
        /* zigzag path */
        let v1 = Rc::new(Vertex::new(0.0, 1.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.5));
        let v3 = Rc::new(Vertex::new(2.0, 1.0));
        let v4 = Rc::new(Vertex::new(3.0, 1.5));
        let v5 = Rc::new(Vertex::new(3.0, 3.5));
        let v6 = Rc::new(Vertex::new(2.0, 3.0));
        let v7 = Rc::new(Vertex::new(1.0, 3.5));
        let v8 = Rc::new(Vertex::new(0.0, 3.0));

        let vertices: Vec<Rc<Vertex>> = vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ];
        let boundary = Rc::new(Polyline::new_closed(vertices).unwrap());

        let mut triangulation = Triangulation::from_initial_segment((&v1, &v2));
        include(&mut triangulation, &boundary, &HashSet::new());

        let solid_triangles: Vec<Rc<Triangle>> = triangulation
            .triangles
            .iter()
            .filter(|triangle| !triangle.is_ghost())
            .cloned()
            .collect();

        assert_eq!(solid_triangles.len(), 6);

        let ghost_triangles: Vec<Rc<Triangle>> = triangulation
            .triangles
            .iter()
            .filter(|triangle| triangle.is_ghost())
            .cloned()
            .collect();

        assert_eq!(ghost_triangles.len(), 8);
    }

    #[test]
    fn sample_3() {
        /* concave domain */
        let v1 = Rc::new(Vertex::new(4.0, 1.0));
        let v2 = Rc::new(Vertex::new(5.0, 1.0));
        let v3 = Rc::new(Vertex::new(6.0, 2.0));
        let v4 = Rc::new(Vertex::new(4.0, 4.0));
        let v5 = Rc::new(Vertex::new(3.0, 4.0));
        let v6 = Rc::new(Vertex::new(1.0, 2.0));
        let v7 = Rc::new(Vertex::new(2.0, 1.0));
        let v8 = Rc::new(Vertex::new(3.0, 1.0));
        let v9 = Rc::new(Vertex::new(2.0, 2.0));
        let v10 = Rc::new(Vertex::new(3.0, 3.0));
        let v11 = Rc::new(Vertex::new(4.0, 3.0));
        let v12 = Rc::new(Vertex::new(5.0, 2.0));

        let vertices: Vec<Rc<Vertex>> = vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
            Rc::clone(&v9),
            Rc::clone(&v10),
            Rc::clone(&v11),
            Rc::clone(&v12),
        ];
        let boundary = Rc::new(Polyline::new_closed(vertices).unwrap());

        let mut triangulation = Triangulation::from_initial_segment((&v1, &v2));
        include(&mut triangulation, &boundary, &HashSet::new());

        let solid_triangles: Vec<Rc<Triangle>> = triangulation
            .triangles
            .iter()
            .filter(|triangle| !triangle.is_ghost())
            .cloned()
            .collect();

        assert_eq!(solid_triangles.len(), 10);

        let ghost_triangles: Vec<Rc<Triangle>> = triangulation
            .triangles
            .iter()
            .filter(|triangle| triangle.is_ghost())
            .cloned()
            .collect();

        assert_eq!(ghost_triangles.len(), 12);
    }
} /* end - boundary inclusion tests */
