use crate::elements::{edge::*, polyline::*, triangle::*, vertex::*};
use crate::planar::{triangulation::*, triangulation_procedures};
use crate::properties::continence::*;

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

/**
 * Find encroached segments and unencroaches them by spliting segments
 */
pub fn unencroach(
    triangulation: &mut Triangulation,
    segment_contraints: &HashSet<Rc<Edge>>,
    boundary: &Option<Rc<Polyline>>,
    holes: &HashSet<Rc<Polyline>>,
) -> HashMap<Rc<Edge>, HashSet<Rc<Edge>>> {
    let mut split_map: HashMap<Rc<Edge>, HashSet<Rc<Edge>>> = HashMap::new();
    let mut encroach_map: HashMap<Rc<Edge>, HashSet<Rc<Vertex>>> = HashMap::new();

    distribute_encroachments(
        segment_contraints,
        &triangulation.vertices(),
        &mut encroach_map,
    );

    while !encroach_map.is_empty() {
        let encroached_edge = Rc::clone(encroach_map.keys().next().unwrap());
        let mut encroaching_vertices = encroach_map.remove(&encroached_edge).unwrap();

        let new_edges = unencroach_segment(
            triangulation,
            &encroached_edge,
            &mut encroaching_vertices,
            segment_contraints,
            boundary,
            holes,
        );

        split_map.insert(Rc::clone(&encroached_edge), new_edges);
    }

    return split_map;
}

/**
 * Splits the segment and its subsegments until none is encroached.
 * Returns new subsegments.
 */
pub fn unencroach_segment(
    triangulation: &mut Triangulation,
    encroached_edge: &Rc<Edge>,
    encroaching_vertices: &mut HashSet<Rc<Vertex>>,
    segment_contraints: &HashSet<Rc<Edge>>,
    boundary: &Option<Rc<Polyline>>,
    holes: &HashSet<Rc<Polyline>>,
) -> HashSet<Rc<Edge>> {
    let mut new_edges: HashSet<Rc<Edge>> = HashSet::new();
    let mut pending_edges: Vec<Rc<Edge>> = Vec::new();

    pending_edges.push(Rc::clone(&encroached_edge));

    while !pending_edges.is_empty() {
        let pending_edge = pending_edges.pop().unwrap();

        let (h1, h2) = split_segment(
            triangulation,
            &pending_edge,
            segment_contraints,
            boundary,
            holes,
        );

        let mut is_h1_encroached = false;
        let mut is_h2_encroached = false;

        for v in encroaching_vertices
            .iter()
            .cloned()
            .collect::<HashSet<Rc<Vertex>>>()
            .iter()
        {
            let mut v_encroaches_any = false;
            if h1.encroach(&v) == Continence::Inside {
                is_h1_encroached = true;
                v_encroaches_any = true;
            }
            if h2.encroach(&v) == Continence::Inside {
                is_h2_encroached = true;
                v_encroaches_any = true;
            }
            if !v_encroaches_any {
                encroaching_vertices.remove(v);
            }
        }

        if is_h1_encroached {
            pending_edges.push(h1);
        } else {
            new_edges.insert(h1);
        }

        if is_h2_encroached {
            pending_edges.push(h2);
        } else {
            new_edges.insert(h2);
        }
    } /* end - for pending edges */

    return new_edges;
} /* end - unencroach_segment */

/**
 * Populates encroach_map with encroachments of segments against a collection of vertices.
 * A vertex will be copied to more than a collection if it is encroached more than once.
 */
pub fn distribute_encroachments(
    segments: &HashSet<Rc<Edge>>,
    vertices: &HashSet<Rc<Vertex>>,
    encroach_map: &mut HashMap<Rc<Edge>, HashSet<Rc<Vertex>>>,
) {
    for edge in segments.iter() {
        let mut possible_encroached_vertices: HashSet<Rc<Vertex>> = HashSet::new();
        for vertex in vertices.iter() {
            if edge.encroach(vertex) == Continence::Inside {
                possible_encroached_vertices.insert(Rc::clone(vertex));
            }
        }
        if !possible_encroached_vertices.is_empty() {
            encroach_map.insert(Rc::clone(edge), possible_encroached_vertices);
        }
    }
}

/**
 * Handles segment split,
 * solving possible conflicts to nearby triangles
 * and ghost triangles at boundary.
 */
fn split_segment(
    triangulation: &mut Triangulation,
    segment: &Rc<Edge>,
    segment_constraints: &HashSet<Rc<Edge>>,
    boundary: &Option<Rc<Polyline>>,
    holes: &HashSet<Rc<Polyline>>,
) -> (Rc<Edge>, Rc<Edge>) {
    let segment_constraints: HashSet<Rc<Edge>> = segment_constraints
        .iter()
        .filter(|&e| e != segment && e != &Rc::new(segment.opposite()))
        .cloned()
        .collect();

    let segment_midpoint = Rc::new(segment.midpoint());
    let half_1 = Rc::new(Edge::new(&segment.v1, &segment_midpoint));
    let half_2 = Rc::new(Edge::new(&segment_midpoint, &segment.v2));

    triangulation_procedures::vertices::include(
        triangulation,
        vec![segment_midpoint],
        &segment_constraints,
        boundary,
        holes,
    );
    return (half_1, half_2);
} /* end - split_segment */

#[cfg(test)]
mod vertices_inclusion {
    use super::*;

    #[test]
    fn sample_1() {
        /*
            Inserts vertex at the boundary
            splitting ghost triangle
            and inner triangles
        */
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(2.0, 0.0));
        let v3 = Rc::new(Vertex::new(0.0, 2.0));

        let vertices: Vec<Rc<Vertex>> = vec![Rc::clone(&v1), Rc::clone(&v2), Rc::clone(&v3)];

        let boundary = Rc::new(Polyline::new_closed(vertices.iter().cloned().collect()).unwrap());

        let mut triangulation = Triangulation::from_initial_segment((&v1, &v2));
        triangulation_procedures::boundary::include(&mut triangulation, &boundary, &HashSet::new());

        let splittable_edge: Rc<Edge> = Rc::new(Edge::new(&v2, &v3));
        let midpoint = Rc::new(splittable_edge.midpoint());
        triangulation_procedures::vertices::include(
            &mut triangulation,
            vec![Rc::clone(&midpoint)],
            &HashSet::new(),
            &Some(boundary),
            &HashSet::new(),
        );

        assert!(triangulation.vertices().contains(&midpoint));
        assert!(triangulation.edges().contains(&Edge::new(&v2, &midpoint)));
        assert!(triangulation.edges().contains(&Edge::new(&midpoint, &v3)));

        assert_eq!(triangulation.triangles.len(), 6);
        assert!(triangulation
            .triangles
            .contains(&Triangle::new(&v1, &v2, &midpoint)));

        assert!(triangulation
            .triangles
            .contains(&Triangle::new(&v3, &v1, &midpoint)));
    } /* end - sample_1 */
    #[test]
    fn sample_2() {
        /*
            Inserts vertex at the hole
            splitting ghost triangle
            and inner triangles
        */

        /* Boundary */
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(3.0, 0.0));
        let v3 = Rc::new(Vertex::new(3.0, 3.0));
        let v4 = Rc::new(Vertex::new(0.0, 3.0));
        let boundary: Vec<Rc<Vertex>> = vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ];
        let boundary = Rc::new(Polyline::new_closed(boundary).unwrap());

        /* Hole */
        let v5 = Rc::new(Vertex::new(1.0, 1.0));
        let v6 = Rc::new(Vertex::new(2.0, 1.0));
        let v7 = Rc::new(Vertex::new(1.0, 2.0));
        let hole: Vec<Rc<Vertex>> = vec![Rc::clone(&v5), Rc::clone(&v6), Rc::clone(&v7)];
        let hole = Rc::new(Polyline::new_closed(hole).unwrap());

        /* Creates triangulation with boundary and hole */
        let mut triangulation = Triangulation::from_initial_segment((&v1, &v2));
        triangulation_procedures::boundary::include(&mut triangulation, &boundary, &HashSet::new());
        triangulation_procedures::hole::include(&mut triangulation, &hole, &HashSet::new());

        /* Inserts vertex at hole segment */
        let splittable_edge: Rc<Edge> = Rc::new(Edge::new(&v6, &v7));
        let midpoint = Rc::new(splittable_edge.midpoint());
        triangulation_procedures::vertices::include(
            &mut triangulation,
            vec![Rc::clone(&midpoint)],
            &HashSet::new(),
            &Some(boundary),
            &vec![Rc::clone(&hole)].iter().cloned().collect(),
        );

        /*
           Asserts:
               - midpoint is inserted
               - edges connecting to midpoint are inserted
               - ghost triangles quantities
               - solid triangles quantities
               - specific split triangles are inserted
        */
        assert!(triangulation.vertices().contains(&midpoint));
        assert!(triangulation.edges().contains(&Edge::new(&v6, &midpoint)));
        assert!(triangulation.edges().contains(&Edge::new(&midpoint, &v7)));
        assert!(triangulation.edges().contains(&Edge::new(&midpoint, &v3)));

        assert_eq!(
            triangulation
                .triangles
                .iter()
                .filter(|t| t.is_ghost())
                .cloned()
                .collect::<HashSet<Rc<Triangle>>>()
                .len(),
            8
        );
        assert_eq!(
            triangulation
                .triangles
                .iter()
                .filter(|t| !t.is_ghost())
                .cloned()
                .collect::<HashSet<Rc<Triangle>>>()
                .len(),
            8
        );
        assert!(triangulation
            .triangles
            .contains(&Triangle::new(&v6, &v3, &midpoint)));

        assert!(triangulation
            .triangles
            .contains(&Triangle::new(&v3, &v7, &midpoint)));
    } /* end - sample_2 */
}

#[cfg(test)]
mod split {
    use super::*;

    #[test]
    fn sample_1() {
        /* triangle */
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(2.0, 0.0));
        let v3 = Rc::new(Vertex::new(0.0, 2.0));

        let boundary = Rc::new(
            Polyline::new_closed(vec![Rc::clone(&v1), Rc::clone(&v2), Rc::clone(&v3)]).unwrap(),
        );

        let mut triangulation = Triangulation::from_initial_segment((&v1, &v2));
        triangulation_procedures::boundary::include(&mut triangulation, &boundary, &HashSet::new());

        let splitable_segment = Rc::new(Edge::new(&v2, &v3));
        let midpoint = Rc::new(splitable_segment.midpoint());

        let (h1, h2) = split_segment(
            &mut triangulation,
            &splitable_segment,
            &HashSet::new(),
            &Some(boundary),
            &HashSet::new(),
        );

        assert_eq!(h1, Rc::new(Edge::new(&v2, &midpoint)));
        assert_eq!(h2, Rc::new(Edge::new(&midpoint, &v3)));

        assert!(triangulation.vertices().contains(&midpoint));
        assert!(triangulation.edges().contains(&Edge::new(&v2, &midpoint)));
        assert!(triangulation.edges().contains(&Edge::new(&midpoint, &v3)));

        assert_eq!(triangulation.triangles.len(), 6);
        assert!(triangulation
            .triangles
            .contains(&Triangle::new(&v1, &v2, &midpoint)));

        assert!(triangulation
            .triangles
            .contains(&Triangle::new(&v3, &v1, &midpoint)));
    }

    #[test]
    fn sample_2() {
        /*
            Inserts vertex at the hole
            splitting ghost triangle
            and inner triangles
        */

        /* Boundary */
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(3.0, 0.0));
        let v3 = Rc::new(Vertex::new(3.0, 3.0));
        let v4 = Rc::new(Vertex::new(0.0, 3.0));
        let boundary: Vec<Rc<Vertex>> = vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ];
        let boundary = Rc::new(Polyline::new_closed(boundary).unwrap());

        /* Hole */
        let v5 = Rc::new(Vertex::new(1.0, 1.0));
        let v6 = Rc::new(Vertex::new(2.0, 1.0));
        let v7 = Rc::new(Vertex::new(1.0, 2.0));
        let hole: Vec<Rc<Vertex>> = vec![Rc::clone(&v5), Rc::clone(&v6), Rc::clone(&v7)];
        let hole = Rc::new(Polyline::new_closed(hole).unwrap());

        /* Creates triangulation with boundary and hole */
        let mut triangulation = Triangulation::from_initial_segment((&v1, &v2));
        triangulation_procedures::boundary::include(&mut triangulation, &boundary, &HashSet::new());
        triangulation_procedures::hole::include(&mut triangulation, &hole, &HashSet::new());

        /* Inserts vertex at hole segment */
        let splittable_edge: Rc<Edge> = Rc::new(Edge::new(&v6, &v7));
        let midpoint = Rc::new(splittable_edge.midpoint());
        let (h1, h2) = split_segment(
            &mut triangulation,
            &splittable_edge,
            &HashSet::new(),
            &Some(boundary),
            &vec![Rc::clone(&hole)].iter().cloned().collect(),
        );

        /*
           Asserts:
               - returned segments
               - midpoint is inserted
               - edges connecting to midpoint are inserted
               - ghost triangles quantities
               - solid triangles quantities
               - specific split triangles are inserted
        */
        assert_eq!(h1, Rc::new(Edge::new(&v6, &midpoint)));
        assert_eq!(h2, Rc::new(Edge::new(&midpoint, &v7)));

        assert!(triangulation.vertices().contains(&midpoint));
        assert!(triangulation.edges().contains(&Edge::new(&v6, &midpoint)));
        assert!(triangulation.edges().contains(&Edge::new(&midpoint, &v7)));
        assert!(triangulation.edges().contains(&Edge::new(&midpoint, &v3)));

        assert_eq!(
            triangulation
                .triangles
                .iter()
                .filter(|t| t.is_ghost())
                .cloned()
                .collect::<HashSet<Rc<Triangle>>>()
                .len(),
            8
        );
        assert_eq!(
            triangulation
                .triangles
                .iter()
                .filter(|t| !t.is_ghost())
                .cloned()
                .collect::<HashSet<Rc<Triangle>>>()
                .len(),
            8
        );
        assert!(triangulation
            .triangles
            .contains(&Triangle::new(&v6, &v3, &midpoint)));

        assert!(triangulation
            .triangles
            .contains(&Triangle::new(&v3, &v7, &midpoint)));
    } /* end - sample_2 */
} /* end - split_segment tests */

#[cfg(test)]
mod unencroach {
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

        /* unencroach */
        let mapping = unencroach(
            &mut triangulation,
            &boundary.into_edges().iter().cloned().collect(),
            &Some(Rc::clone(&boundary)),
            &HashSet::new(),
        );

        /*
           Testing against:
               - quantity of split segments
               - quantity of new segments
        */
        let split_segments = mapping.keys().cloned().collect::<HashSet<Rc<Edge>>>();
        let new_segments = mapping
            .values()
            .flatten()
            .cloned()
            .collect::<HashSet<Rc<Edge>>>();

        assert_eq!(split_segments.len(), 3);
        assert!(split_segments.contains(&Edge::new(&v1, &v2)));
        assert!(split_segments.contains(&Edge::new(&v2, &v3)));
        assert!(split_segments.contains(&Edge::new(&v3, &v1)));

        assert_eq!(new_segments.len(), 6);
    } /* sample_1 */

    #[test]
    fn sample_2() {
        /* triangle */
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(8.0, 0.0));
        let v3 = Rc::new(Vertex::new(0.0, 8.0));

        let boundary = Rc::new(
            Polyline::new_closed(vec![Rc::clone(&v1), Rc::clone(&v2), Rc::clone(&v3)]).unwrap(),
        );

        /* Encroaching vertex */
        let encroaching_vertex = Rc::new(Vertex::new(1.0, 1.0));

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

        /* unencroach */
        let mapping = unencroach(
            &mut triangulation,
            &boundary.into_edges().iter().cloned().collect(),
            &Some(Rc::clone(&boundary)),
            &HashSet::new(),
        );

        /*
           Testing against:
        */
        let split_segments = mapping.keys().cloned().collect::<HashSet<Rc<Edge>>>();
        let new_segments = mapping
            .values()
            .flatten()
            .cloned()
            .collect::<HashSet<Rc<Edge>>>();

        assert_eq!(mapping.len(), 3);
        assert!(split_segments.contains(&Edge::new(&v1, &v2)));
        assert!(split_segments.contains(&Edge::new(&v2, &v3)));
        assert!(split_segments.contains(&Edge::new(&v3, &v1)));

        assert_eq!(new_segments.len(), 8);

        let new_segments_edge12 = mapping.get(&Edge::new(&v1, &v2)).unwrap();
        assert!(new_segments_edge12.contains(&Edge::new(
            &Rc::new(Vertex::new(0.0, 0.0)),
            &Rc::new(Vertex::new(2.0, 0.0)),
        )));
        assert!(new_segments_edge12.contains(&Edge::new(
            &Rc::new(Vertex::new(2.0, 0.0)),
            &Rc::new(Vertex::new(4.0, 0.0)),
        )));
        assert!(new_segments_edge12.contains(&Edge::new(
            &Rc::new(Vertex::new(4.0, 0.0)),
            &Rc::new(Vertex::new(8.0, 0.0)),
        )));
    } /* sample_2 */
} /* end - unencroach tests */
