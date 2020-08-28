use crate::elements::{edge::*, polyline::*, triangle::*, vertex::*};
use crate::planar::triangulation::*;
use crate::properties::{continence::*, orientation::*};

use crate::planar::procedures;

use std::collections::HashSet;
use std::rc::Rc;

/**
 * Includes Segment.
 * Takes all triangles that intercept the segment or whose circumcircle encircles
 * one of its end vertices, retriangulates the vertices of the triangle around the
 * segment having the segment as constraint. Reinserts the taken triangulation into
 * the main triangulation.
 */
pub fn include(
    triangulation: &mut Triangulation,
    segment: &Rc<Edge>,
    segment_constraints: &HashSet<Rc<Edge>>,
) {
    let conflicting_triangles: HashSet<Rc<Triangle>> = triangulation
        .triangles
        .iter()
        .filter(|t| !t.is_ghost())
        .filter(|triangle| {
            let polyline = triangle.as_polyline().unwrap();
            let segment_polyline = segment.as_polyline().unwrap();

            let contains_segment = Polyline::continence(&polyline, &segment_polyline)
                != Some((Continence::Outside, BoundaryInclusion::Open));

            let conflicts_v1 = triangle.encircles(&segment.v1) != Continence::Outside;
            let conflicts_v2 = triangle.encircles(&segment.v2) != Continence::Outside;

            return conflicts_v1 || conflicts_v2 || contains_segment;
        })
        .filter(|triangle| {
            let (e1, e2, e3) = triangle.outer_edges();
            let is_constrained = vec![e1, e2, e3]
                .iter()
                .filter(|&edge| {
                    segment_constraints.contains(edge)
                        || segment_constraints.contains(&edge.opposite())
                })
                .fold(false, |acc, edge| {
                    if acc == true {
                        return true;
                    }
                    let faces_v1 = orientation(&edge.v1, &edge.v2, &segment.v1)
                        == Orientation::Counterclockwise;
                    let faces_v2 = orientation(&edge.v1, &edge.v2, &segment.v2)
                        == Orientation::Counterclockwise;

                    return faces_v1 || faces_v2;
                });
            return !is_constrained;
        })
        .cloned()
        .collect();

    // Uncomment to debug
    // println!("\n\nSolid Triangles");
    // for t in triangulation.triangles.iter().filter(|t| !t.is_ghost()) {
    //     println!("{}", t);
    // }
    // println!("\nConflicting Triangles against {}", segment);
    // for t in conflicting_triangles.iter() {
    //     println!("{}", t);
    // }

    for conflicting_triangle in conflicting_triangles.iter() {
        triangulation.remove_triangle(conflicting_triangle);
    }

    let triangles_boundary: Rc<Polyline> =
        Rc::new(Polyline::triangles_hull(&conflicting_triangles).unwrap());

    let conflicting_vertices: HashSet<Rc<Vertex>> = conflicting_triangles
        .iter()
        .map(|triangle| {
            vec![
                Rc::clone(&triangle.v1),
                Rc::clone(&triangle.v2),
                Rc::clone(&triangle.v3),
            ]
        })
        .flatten()
        .collect();

    let mut segment_triangulation = Triangulation::from_initial_segment((&segment.v1, &segment.v2));

    let new_segment_constraint: HashSet<Rc<Edge>> =
        vec![Rc::clone(&segment)].iter().cloned().collect();

    procedures::boundary::include(
        &mut segment_triangulation,
        &triangles_boundary,
        &new_segment_constraint,
    );

    procedures::vertices::include(
        &mut segment_triangulation,
        conflicting_vertices.iter().cloned().collect(),
        &new_segment_constraint,
        &Some(Rc::clone(&triangles_boundary)),
        &HashSet::new(),
    );

    let new_solid_triangles: HashSet<Rc<Triangle>> = segment_triangulation
        .triangles
        .iter()
        .filter(|t| !t.is_ghost())
        .cloned()
        .collect();

    // Uncomment to debug
    // println!("\nNew Solid Triangles");
    // for t in new_solid_triangles.iter() {
    //     println!("{}", t);
    // }

    for new_triangle in new_solid_triangles.iter() {
        triangulation.include_triangle(new_triangle);
    }
} /* end - include segment */

#[cfg(test)]
mod include_segment {
    use super::*;

    #[test]
    fn sample_1() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(6.0, 1.0));
        let v3 = Rc::new(Vertex::new(8.0, 8.0));
        let v4 = Rc::new(Vertex::new(1.0, 6.0));

        let v5 = Rc::new(Vertex::new(2.0, 2.0));
        let v6 = Rc::new(Vertex::new(5.0, 5.0));

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

        let mut triangulation = Triangulation::from_initial_segment((&v1, &v2));
        procedures::vertices::include(
            &mut triangulation,
            vertices.iter().cloned().collect(),
            &HashSet::new(),
            &None,
            &HashSet::new(),
        );

        procedures::segment::include(&mut triangulation, &s1, &HashSet::new());

        for v in vertices.iter() {
            assert!(triangulation.vertices().contains(v));
        }

        assert!(triangulation.edges().contains(&s1));
        assert_eq!(triangulation.vertices().len(), 6);

        let expected_triangles = vec![
            Rc::new(Triangle::new(&v2, &v5, &v1)),
            Rc::new(Triangle::new(&v2, &v6, &v5)),
            Rc::new(Triangle::new(&v2, &v3, &v6)),
            Rc::new(Triangle::new(&v4, &v1, &v5)),
            Rc::new(Triangle::new(&v4, &v5, &v6)),
            Rc::new(Triangle::new(&v4, &v6, &v3)),
        ];

        let triangles = triangulation
            .triangles
            .iter()
            .filter(|t| !t.is_ghost())
            .cloned()
            .collect::<Vec<Rc<Triangle>>>();

        assert_eq!(triangles.len(), 6);
        for t in expected_triangles.iter() {
            assert!(triangles.contains(t));
        }
    }

    #[test]
    fn sample_2() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(5.0, 1.0));
        let v3 = Rc::new(Vertex::new(5.0, 5.0));
        let v4 = Rc::new(Vertex::new(1.0, 5.0));
        let v5 = Rc::new(Vertex::new(3.0, 2.0));
        let v6 = Rc::new(Vertex::new(4.0, 3.0));
        let v7 = Rc::new(Vertex::new(3.0, 4.0));
        let v8 = Rc::new(Vertex::new(2.0, 3.0));

        let v11 = Rc::new(Vertex::new(2.0, 2.0));
        let v12 = Rc::new(Vertex::new(4.0, 4.0));

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

        let s1 = Rc::new(Edge::new(&v11, &v12));

        let mut triangulation = Triangulation::from_initial_segment((&v1, &v2));
        procedures::vertices::include(
            &mut triangulation,
            vertices.iter().cloned().collect(),
            &HashSet::new(),
            &None,
            &HashSet::new(),
        );
        assert_eq!(triangulation.vertices().len(), 8);
        procedures::segment::include(&mut triangulation, &s1, &HashSet::new());

        for v in vertices.iter() {
            assert!(triangulation.vertices().contains(v));
        }
        assert!(triangulation.edges().contains(&s1));
        assert_eq!(triangulation.vertices().len(), 10);
    }

    #[test]
    fn sample_3() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(7.0, 1.0));
        let v3 = Rc::new(Vertex::new(7.0, 7.0));
        let v4 = Rc::new(Vertex::new(1.0, 7.0));

        let v5 = Rc::new(Vertex::new(2.0, 4.0));
        let v6 = Rc::new(Vertex::new(6.0, 3.0));
        let v7 = Rc::new(Vertex::new(6.0, 5.0));

        let v8 = Rc::new(Vertex::new(3.0, 2.0));
        let v9 = Rc::new(Vertex::new(3.0, 6.0));

        let v11 = Rc::new(Vertex::new(4.0, 4.0));
        let v12 = Rc::new(Vertex::new(5.0, 4.0));

        let vertices: HashSet<Rc<Vertex>> = vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
        ]
        .iter()
        .cloned()
        .collect();

        let s1 = Rc::new(Edge::new(&v8, &v9));
        let s2 = Rc::new(Edge::new(&v11, &v12));

        let mut triangulation = Triangulation::from_initial_segment((&v1, &v2));
        procedures::vertices::include(
            &mut triangulation,
            vertices.iter().cloned().collect(),
            &HashSet::new(),
            &None,
            &HashSet::new(),
        );
        assert_eq!(triangulation.vertices().len(), 7);
        procedures::segment::include(&mut triangulation, &s1, &HashSet::new());
        assert_eq!(triangulation.vertices().len(), 9);
        assert!(triangulation.edges().contains(&s1));
        assert!(triangulation.vertices().contains(&v8));
        assert!(triangulation.vertices().contains(&v9));
        for v in vertices.iter() {
            assert!(triangulation.vertices().contains(v));
        }

        procedures::segment::include(
            &mut triangulation,
            &s2,
            &vec![Rc::clone(&s1)].iter().cloned().collect(),
        );
        assert_eq!(triangulation.vertices().len(), 11);
        assert!(triangulation.edges().contains(&s1));
        assert!(triangulation.edges().contains(&s2));
    }
} /* end - include_segment tests */
