use crate::elements::{edge::*, polyline::*, vertex::*};
use crate::planar::{refine_params::*, triangulation::*};
use crate::properties::continence::*;

use crate::planar::{refine_procedures, triangulation_procedures};

use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::rc::Rc;

pub struct Triangulator {
    pub triangulation: RefCell<Triangulation>,
    pub boundary: Rc<Polyline>,
    pub holes: HashSet<Rc<Polyline>>,
    pub vertices: HashSet<Rc<Vertex>>,
    pub segments: HashSet<Rc<Edge>>,
}

impl Triangulator {
    pub fn new(boundary: &Rc<Polyline>) -> Self {
        Self {
            triangulation: RefCell::new(Triangulation::new()),
            boundary: Rc::clone(boundary),
            holes: HashSet::new(),
            vertices: HashSet::new(),
            segments: HashSet::new(),
        }
    }

    /**
     * Inserts vertex in the triangulation. If any vertex is outside
     * the boundary or it is inside any hole, no vertices are inserted
     * and the conflicting set is returned. If out of bounds condition
     * is not met, vertices are inserted into the hashSet. No duplicate
     * vertex is kept.
     */
    pub fn insert_vertices(
        &mut self,
        vertices: &HashSet<Rc<Vertex>>,
    ) -> Result<&Self, HashSet<Rc<Vertex>>> {
        if vertices.is_empty() {
            return Ok(self);
        }

        let mut panic_vertices: HashSet<Rc<Vertex>> = HashSet::new();

        /* Boundary continence */
        for vertex in vertices.iter() {
            if self.boundary.contains(&vertex) != Some(Continence::Inside) {
                panic_vertices.insert(Rc::clone(vertex));
                continue;
            }

            for hole in self.holes.iter() {
                if hole.contains(&vertex) != Some(Continence::Outside) {
                    panic_vertices.insert(Rc::clone(vertex));
                    continue;
                }
            }
        }

        /* Then retuns error if wrong boundary */
        if !panic_vertices.is_empty() {
            return Err(panic_vertices);
        }

        /* Inserts vertices if they don't exist already */
        for vertex in vertices.iter() {
            let mut should_insert = true;
            for segment in self
                .segments
                .iter()
                .cloned()
                .collect::<HashSet<Rc<Edge>>>()
                .iter()
            {
                if &segment.v1 == vertex || &segment.v2 == vertex {
                    /* Ignore vertices if they are and end vertex */
                    should_insert = false;
                    break;
                }
                if segment.contains(vertex) {
                    /* splits segment if contained */
                    self.segments.remove(segment);
                    self.segments
                        .insert(Rc::new(Edge::new(&segment.v1, vertex)));
                    self.segments
                        .insert(Rc::new(Edge::new(vertex, &segment.v2)));
                    should_insert = false;
                    break;
                }
            }
            if should_insert {
                self.vertices.insert(Rc::clone(&vertex));
            }
        }

        return Ok(self);
    }

    /**
     * Inserts segments to the triangulation. If any segment is not outside
     * all holes, or if it is not inside the boundary, returns the set of
     * conflicting segments. If out of bounds condition is not met, all
     * segments are inserted. Segments that meet intersection are splited.
     */
    pub fn insert_segments(
        &mut self,
        segments: &HashSet<Rc<Edge>>,
    ) -> Result<&Self, HashSet<Rc<Edge>>> {
        if segments.is_empty() {
            return Ok(self);
        }

        /* Accumulate conflicting segments */
        let mut conflicting_segments: HashSet<Rc<Edge>> = HashSet::new();
        for segment in segments.iter() {
            let segment_polyline: Polyline =
                Polyline::new_opened(vec![Rc::clone(&segment.v1), Rc::clone(&segment.v2)]).unwrap();

            if Polyline::continence(&self.boundary, &segment_polyline)
                != Some((Continence::Inside, BoundaryInclusion::Open))
            {
                conflicting_segments.insert(Rc::clone(segment));
                continue;
            }

            for hole in self.holes.iter() {
                if Polyline::continence(hole, &segment_polyline)
                    != Some((Continence::Outside, BoundaryInclusion::Open))
                {
                    conflicting_segments.insert(Rc::clone(segment));
                    continue;
                }
            }
        }

        if !conflicting_segments.is_empty() {
            return Err(conflicting_segments);
        }

        /* Removes vertices */
        let mut aux_list: Vec<Rc<Edge>> = segments.iter().cloned().collect();
        let mut segments_to_insert: HashSet<Rc<Edge>> = HashSet::new();
        while !aux_list.is_empty() {
            let segment = aux_list.pop().unwrap();
            let mut should_insert = true;
            for vertex in self
                .vertices
                .iter()
                .cloned()
                .collect::<HashSet<Rc<Vertex>>>()
                .iter()
            {
                if &segment.v1 == vertex || &segment.v2 == vertex {
                    /* Ignore vertices if they are an end vertex */
                    self.vertices.remove(vertex);
                }
                if segment.contains(vertex) {
                    /* splits segment if contained */
                    aux_list.push(Rc::new(Edge::new(&segment.v1, vertex)));
                    aux_list.push(Rc::new(Edge::new(vertex, &segment.v2)));
                    self.vertices.remove(vertex);
                    should_insert = false;
                    break;
                }
            }
            if should_insert {
                segments_to_insert.insert(Rc::clone(&segment));
            }
        }

        /* Split segments */
        let new_vertex_pairs =
            Edge::into_vertex_pairs(segments_to_insert.iter().cloned().collect());

        let existing_vertex_pairs =
            Edge::into_vertex_pairs(self.segments.iter().cloned().collect());

        let splited_segments = split_intersections(
            &new_vertex_pairs
                .iter()
                .chain(existing_vertex_pairs.iter())
                .cloned()
                .collect(),
        );

        self.segments = Edge::from_vertex_pairs(splited_segments)
            .iter()
            .cloned()
            .collect();

        return Ok(self);
    }

    /**
     * Inserts hole. If hole intercepts the boundary, any existing hole, or
     * existing segments returns the set of conflicting vertices. If not,
     * hole is inserted. If any existing vertex or segment belongs to the
     * hole, it is removed.
     */
    pub fn insert_hole(&mut self, hole: &Rc<Polyline>) -> Result<&Self, HashSet<Rc<Vertex>>> {
        let mut conflicting_vertices: HashSet<Rc<Vertex>> = HashSet::new();

        let is_hole_inside_boundary = Polyline::continence(&self.boundary, hole)
            == Some((Continence::Inside, BoundaryInclusion::Open));

        if !is_hole_inside_boundary {
            conflicting_vertices = conflicting_vertices
                .iter()
                .chain(self.boundary.vertices.iter())
                .cloned()
                .collect();
        }

        for existing_hole in self.holes.iter() {
            let is_hole_outside_existing_hole = Polyline::continence(existing_hole, hole)
                == Some((Continence::Outside, BoundaryInclusion::Open));

            if !is_hole_outside_existing_hole {
                conflicting_vertices = conflicting_vertices
                    .iter()
                    .chain(existing_hole.vertices.iter())
                    .cloned()
                    .collect();
            }
        }

        if !conflicting_vertices.is_empty() {
            return Err(conflicting_vertices);
        }

        for segment in self.segments.iter() {
            let segment_polyline =
                Polyline::new_opened(vec![Rc::clone(&segment.v1), Rc::clone(&segment.v2)]).unwrap();

            let relative_continence = Polyline::continence(hole, &segment_polyline);
            if relative_continence != Some((Continence::Outside, BoundaryInclusion::Open)) {
                conflicting_vertices.insert(Rc::clone(&segment.v1));
                conflicting_vertices.insert(Rc::clone(&segment.v2));
            }
        }

        if !conflicting_vertices.is_empty() {
            return Err(conflicting_vertices);
        }

        self.holes.insert(Rc::clone(hole));
        return Ok(self);
    }

    /**
     * Refine the triangulation. Raises triangulation error if any.
     * Else refines ans returns the triangulation.
     */
    pub fn refine(&mut self, params: RefineParams) -> &Self {
        let mut segment_constraints: HashSet<Rc<Edge>> = self
            .holes
            .iter()
            .map(|hole| hole.into_edges())
            .flatten()
            .chain(self.boundary.into_edges())
            .chain(self.segments.iter().cloned())
            .collect();

        let (segments_splitting, included_triangles, removed_triangles) =
            refine_procedures::encroachment::unencroach(
                &mut self.triangulation.borrow_mut(),
                &segment_constraints,
                &Some(Rc::clone(&self.boundary)),
                &self.holes,
            );

        segment_constraints = segment_constraints
            .iter()
            .filter(|&s| {
                segments_splitting
                    .keys()
                    .cloned()
                    .collect::<HashSet<Rc<Edge>>>()
                    .contains(s)
            })
            .chain(segments_splitting.values().flatten())
            .cloned()
            .collect();

        let segments_splitting = refine_procedures::triangle_split::split_irregular(
            &mut self.triangulation.borrow_mut(),
            &params,
            &segment_constraints,
            &Some(Rc::clone(&self.boundary)),
            &self.holes,
        );

        segment_constraints = segment_constraints
            .iter()
            .filter(|&s| {
                segments_splitting
                    .values()
                    .cloned()
                    .collect::<HashSet<Rc<Edge>>>()
                    .contains(s)
            })
            .chain(segments_splitting.keys())
            .cloned()
            .collect();

        return self;
    }

    /**
     * Triangulates
     */
    pub fn triangulate(&mut self) -> &Self {
        /* Initialize triangulation */
        let v1 = self.boundary.vertices.get(0).unwrap();
        let v2 = self.boundary.vertices.get(1).unwrap();
        let mut triangulation = Triangulation::from_initial_segment((&v1, &v2));

        /* 1 Boundary inclusion */
        triangulation_procedures::boundary::include(
            &mut triangulation,
            &self.boundary,
            &HashSet::new(),
        );

        /* boundary segments as segment constraints */
        let mut segment_constraints: HashSet<Rc<Edge>> =
            self.boundary.into_edges().iter().cloned().collect();

        /* 2 Holes inclusion */
        for hole in self.holes.iter() {
            triangulation_procedures::hole::include(&mut triangulation, hole, &segment_constraints);

            segment_constraints = segment_constraints
                .iter()
                .chain(hole.into_edges().iter())
                .cloned()
                .collect();
        }

        /* 3 Include Segment Constraints */
        for segment in self.segments.iter() {
            triangulation_procedures::segment::include(
                &mut triangulation,
                segment,
                &segment_constraints,
            );
            segment_constraints.insert(Rc::clone(segment));
        }

        /* 4 Include remaining Vertices */
        triangulation_procedures::vertices::include(
            &mut triangulation,
            self.vertices.iter().cloned().collect(),
            &segment_constraints,
            &Some(Rc::clone(&self.boundary)),
            &self.holes,
        );

        self.triangulation = RefCell::new(triangulation);

        return self;
    }
} /* end - module */

#[cfg(test)]
mod insert_holes {
    use super::*;

    #[test]
    fn sample_1() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let mut boundary: Vec<Rc<Vertex>> = Vec::new();
        boundary.push(Rc::clone(&v1));
        boundary.push(Rc::clone(&v2));
        boundary.push(Rc::clone(&v3));
        boundary.push(Rc::clone(&v4));
        let boundary = Rc::new(Polyline::new_closed(boundary).unwrap());

        /* Inner Vertices */
        let v11 = Rc::new(Vertex::new(2.0, 2.0));
        let v12 = Rc::new(Vertex::new(3.0, 2.0));
        let v13 = Rc::new(Vertex::new(3.0, 3.0));
        let v14 = Rc::new(Vertex::new(2.0, 3.0));

        let mut hole_vertices: Vec<Rc<Vertex>> = Vec::new();
        hole_vertices.push(Rc::clone(&v11));
        hole_vertices.push(Rc::clone(&v12));
        hole_vertices.push(Rc::clone(&v13));
        hole_vertices.push(Rc::clone(&v14));
        let hole = Rc::new(Polyline::new_closed(hole_vertices).unwrap());

        let mut triangulator = Triangulator::new(&boundary);
        let result = triangulator.insert_hole(&hole);
        assert!(result.is_ok());

        assert!(triangulator.holes.contains(&hole));
    }

    #[test]
    fn error_insert_if_intersection_with_boundary() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let mut boundary: Vec<Rc<Vertex>> = Vec::new();
        boundary.push(Rc::clone(&v1));
        boundary.push(Rc::clone(&v2));
        boundary.push(Rc::clone(&v3));
        boundary.push(Rc::clone(&v4));
        let boundary = Rc::new(Polyline::new_closed(boundary).unwrap());

        /* Inner Vertices */
        let v11 = Rc::new(Vertex::new(2.0, 2.0));
        let v12 = Rc::new(Vertex::new(3.0, 2.0));
        let v13 = Rc::new(Vertex::new(6.0, 6.0));
        let v14 = Rc::new(Vertex::new(2.0, 3.0));

        let mut hole_vertices: Vec<Rc<Vertex>> = Vec::new();
        hole_vertices.push(Rc::clone(&v11));
        hole_vertices.push(Rc::clone(&v12));
        hole_vertices.push(Rc::clone(&v13));
        hole_vertices.push(Rc::clone(&v14));
        let hole = Rc::new(Polyline::new_closed(hole_vertices).unwrap());

        let mut triangulator = Triangulator::new(&boundary);
        let result = triangulator.insert_hole(&hole);

        assert!(result.is_err());
        if let Err(vertices) = result {
            assert_eq!(vertices.len(), 4);
            assert!(vertices.contains(&v1));
            assert!(vertices.contains(&v2));
            assert!(vertices.contains(&v3));
            assert!(vertices.contains(&v4));
        }
        assert!(triangulator.holes.is_empty());
    }

    #[test]
    fn error_insert_if_intersection_at_boundary() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let mut boundary: Vec<Rc<Vertex>> = Vec::new();
        boundary.push(Rc::clone(&v1));
        boundary.push(Rc::clone(&v2));
        boundary.push(Rc::clone(&v3));
        boundary.push(Rc::clone(&v4));
        let boundary = Rc::new(Polyline::new_closed(boundary).unwrap());

        /* Hole Vertices */
        let v11 = Rc::new(Vertex::new(2.0, 2.0));
        let v12 = Rc::new(Vertex::new(3.0, 2.0));
        let v13 = Rc::new(Vertex::new(3.0, 3.0));
        let v14 = Rc::new(Vertex::new(2.0, 4.0));

        let mut hole_vertices: Vec<Rc<Vertex>> = Vec::new();
        hole_vertices.push(Rc::clone(&v11));
        hole_vertices.push(Rc::clone(&v12));
        hole_vertices.push(Rc::clone(&v13));
        hole_vertices.push(Rc::clone(&v14));
        let hole = Rc::new(Polyline::new_closed(hole_vertices).unwrap());

        let mut triangulator = Triangulator::new(&boundary);
        let result = triangulator.insert_hole(&hole);

        assert!(result.is_err());
        if let Err(vertices) = result {
            assert_eq!(vertices.len(), 4);
            assert!(vertices.contains(&v1));
            assert!(vertices.contains(&v2));
            assert!(vertices.contains(&v3));
            assert!(vertices.contains(&v4));
        }
        assert!(triangulator.holes.is_empty());
    }

    #[test]
    fn two_holes() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let mut boundary: Vec<Rc<Vertex>> = Vec::new();
        boundary.push(Rc::clone(&v1));
        boundary.push(Rc::clone(&v2));
        boundary.push(Rc::clone(&v3));
        boundary.push(Rc::clone(&v4));
        let boundary = Rc::new(Polyline::new_closed(boundary).unwrap());

        /* Hole 1 */
        let v11 = Rc::new(Vertex::new(2.0, 2.0));
        let v12 = Rc::new(Vertex::new(2.9, 2.0));
        let v13 = Rc::new(Vertex::new(2.0, 2.9));

        let mut hole1_vertices: Vec<Rc<Vertex>> = Vec::new();
        hole1_vertices.push(Rc::clone(&v11));
        hole1_vertices.push(Rc::clone(&v12));
        hole1_vertices.push(Rc::clone(&v13));
        let hole_1 = Rc::new(Polyline::new_closed(hole1_vertices).unwrap());

        /* Hole 2 */
        let v21 = Rc::new(Vertex::new(3.0, 3.0));
        let v22 = Rc::new(Vertex::new(2.1, 3.0));
        let v23 = Rc::new(Vertex::new(3.0, 2.1));

        let mut hole2_vertices: Vec<Rc<Vertex>> = Vec::new();
        hole2_vertices.push(Rc::clone(&v21));
        hole2_vertices.push(Rc::clone(&v22));
        hole2_vertices.push(Rc::clone(&v23));
        let hole_2 = Rc::new(Polyline::new_closed(hole2_vertices).unwrap());

        let mut triangulator = Triangulator::new(&boundary);
        triangulator.insert_hole(&hole_1);
        let result = triangulator.insert_hole(&hole_2);

        assert!(result.is_ok());
        assert!(triangulator.holes.contains(&hole_1));
        assert!(triangulator.holes.contains(&hole_2));
    }

    #[test]
    fn error_on_hole_intersection() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let mut boundary: Vec<Rc<Vertex>> = Vec::new();
        boundary.push(Rc::clone(&v1));
        boundary.push(Rc::clone(&v2));
        boundary.push(Rc::clone(&v3));
        boundary.push(Rc::clone(&v4));
        let boundary = Rc::new(Polyline::new_closed(boundary).unwrap());

        /* Hole 1 */
        let v11 = Rc::new(Vertex::new(2.0, 2.0));
        let v12 = Rc::new(Vertex::new(2.9, 2.0));
        let v13 = Rc::new(Vertex::new(2.0, 2.9));

        let mut hole1_vertices: Vec<Rc<Vertex>> = Vec::new();
        hole1_vertices.push(Rc::clone(&v11));
        hole1_vertices.push(Rc::clone(&v12));
        hole1_vertices.push(Rc::clone(&v13));
        let hole_1 = Rc::new(Polyline::new_closed(hole1_vertices).unwrap());

        /* Hole 2 */
        let v21 = Rc::new(Vertex::new(3.0, 3.0));
        let v22 = Rc::new(Vertex::new(2.1, 3.0));
        let v23 = Rc::new(Vertex::new(2.9, 2.0));
        let v24 = Rc::new(Vertex::new(3.0, 2.1));

        let mut hole2_vertices: Vec<Rc<Vertex>> = Vec::new();
        hole2_vertices.push(Rc::clone(&v21));
        hole2_vertices.push(Rc::clone(&v22));
        hole2_vertices.push(Rc::clone(&v23));
        hole2_vertices.push(Rc::clone(&v24));
        let hole_2 = Rc::new(Polyline::new_closed(hole2_vertices).unwrap());

        let mut triangulator = Triangulator::new(&boundary);
        triangulator.insert_hole(&hole_1);
        let result = triangulator.insert_hole(&hole_2);

        assert!(result.is_err());
        if let Err(vertices) = result {
            assert_eq!(vertices.len(), 3);
            assert!(vertices.contains(&v11));
            assert!(vertices.contains(&v12));
            assert!(vertices.contains(&v13));
        }
        assert!(triangulator.holes.contains(&hole_1));
    }

    #[test]
    fn error_on_segments_wrong_continence() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let mut boundary: Vec<Rc<Vertex>> = Vec::new();
        boundary.push(Rc::clone(&v1));
        boundary.push(Rc::clone(&v2));
        boundary.push(Rc::clone(&v3));
        boundary.push(Rc::clone(&v4));
        let boundary = Rc::new(Polyline::new_closed(boundary).unwrap());

        /* Segments */
        let v11 = Rc::new(Vertex::new(2.0, 2.0));
        let v12 = Rc::new(Vertex::new(3.0, 2.0));
        let v13 = Rc::new(Vertex::new(3.0, 3.0));
        let v14 = Rc::new(Vertex::new(2.0, 3.0));

        let segments_constrains: HashSet<Rc<Edge>> = vec![
            Rc::new(Edge::new(&v11, &v12)),
            Rc::new(Edge::new(&v13, &v14)),
        ]
        .iter()
        .cloned()
        .collect();

        /* Hole */
        let v21 = Rc::new(Vertex::new(1.5, 1.5));
        let v22 = Rc::new(Vertex::new(3.5, 1.5));
        let v23 = Rc::new(Vertex::new(3.5, 3.5));
        let v24 = Rc::new(Vertex::new(1.5, 3.5));

        let mut hole_vertices: Vec<Rc<Vertex>> = Vec::new();
        hole_vertices.push(Rc::clone(&v21));
        hole_vertices.push(Rc::clone(&v22));
        hole_vertices.push(Rc::clone(&v23));
        hole_vertices.push(Rc::clone(&v24));
        let hole = Rc::new(Polyline::new_closed(hole_vertices).unwrap());

        let mut triangulator = Triangulator::new(&boundary);
        triangulator.insert_segments(&segments_constrains);
        let result = triangulator.insert_hole(&hole);

        assert!(result.is_err());
        if let Err(vertices) = result {
            assert_eq!(vertices.len(), 4);
            assert!(vertices.contains(&v11));
            assert!(vertices.contains(&v12));
            assert!(vertices.contains(&v13));
            assert!(vertices.contains(&v14));
        }
        assert!(triangulator.holes.is_empty());
    }
} /* end - insert_hole tests */

#[cfg(test)]
mod insert_vertices {
    use super::*;

    #[test]
    fn sample_1() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(5.0, 1.0));
        let v3 = Rc::new(Vertex::new(5.0, 5.0));
        let v4 = Rc::new(Vertex::new(1.0, 5.0));

        let mut boundary: Vec<Rc<Vertex>> = Vec::new();
        boundary.push(Rc::clone(&v1));
        boundary.push(Rc::clone(&v2));
        boundary.push(Rc::clone(&v3));
        boundary.push(Rc::clone(&v4));
        let boundary = Rc::new(Polyline::new_closed(boundary).unwrap());

        /* Inner Vertices */
        let v14 = Rc::new(Vertex::new(2.0, 3.0));
        let v15 = Rc::new(Vertex::new(1.0000001, 1.0000001));
        let v16 = Rc::new(Vertex::new(4.9999999, 4.9999999));

        let mut inner_vertices: HashSet<Rc<Vertex>> = HashSet::new();
        inner_vertices.insert(Rc::clone(&v14));
        inner_vertices.insert(Rc::clone(&v15));
        inner_vertices.insert(Rc::clone(&v16));

        let mut triangulator = Triangulator::new(&boundary);
        let result = triangulator.insert_vertices(&inner_vertices);
        assert!(result.is_ok());

        assert!(triangulator.vertices.contains(&v14));
        assert!(triangulator.vertices.contains(&v15));
        assert!(triangulator.vertices.contains(&v16));
    }

    #[test]
    fn error_if_any_out_of_boundary() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(5.0, 1.0));
        let v3 = Rc::new(Vertex::new(5.0, 5.0));
        let v4 = Rc::new(Vertex::new(1.0, 5.0));

        let mut boundary: Vec<Rc<Vertex>> = Vec::new();
        boundary.push(Rc::clone(&v1));
        boundary.push(Rc::clone(&v2));
        boundary.push(Rc::clone(&v3));
        boundary.push(Rc::clone(&v4));
        let boundary = Rc::new(Polyline::new_closed(boundary).unwrap());

        /* Outer Vertices */
        let v10 = Rc::new(Vertex::new(0.0, 0.0));
        let v11 = Rc::new(Vertex::new(0.9999999, 0.9999999));
        let v12 = Rc::new(Vertex::new(5.0000001, 5.0000001));
        let v13 = Rc::new(Vertex::new(3.0, 1.0)); /* at the boundary */

        /* Inner Vertices */
        let v14 = Rc::new(Vertex::new(2.0, 3.0));
        let v15 = Rc::new(Vertex::new(1.0000001, 1.0000001));
        let v16 = Rc::new(Vertex::new(4.9999999, 4.9999999));

        let mut inner_vertices: HashSet<Rc<Vertex>> = HashSet::new();
        inner_vertices.insert(Rc::clone(&v10));
        inner_vertices.insert(Rc::clone(&v11));
        inner_vertices.insert(Rc::clone(&v12));
        inner_vertices.insert(Rc::clone(&v13));
        inner_vertices.insert(Rc::clone(&v14));
        inner_vertices.insert(Rc::clone(&v15));
        inner_vertices.insert(Rc::clone(&v16));

        let mut triangulator = Triangulator::new(&boundary);
        let result = triangulator.insert_vertices(&inner_vertices);
        assert!(result.is_err());

        if let Err(panic_vertices) = result {
            assert!(panic_vertices.contains(&v10));
            assert!(panic_vertices.contains(&v11));
            assert!(panic_vertices.contains(&v12));
            assert!(panic_vertices.contains(&v13));
        }
    }

    #[test]
    fn error_if_any_inside_hole() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let mut boundary: Vec<Rc<Vertex>> = Vec::new();
        boundary.push(Rc::clone(&v1));
        boundary.push(Rc::clone(&v2));
        boundary.push(Rc::clone(&v3));
        boundary.push(Rc::clone(&v4));
        let boundary = Rc::new(Polyline::new_closed(boundary).unwrap());

        /* Hole Vertices */
        let v11 = Rc::new(Vertex::new(2.0, 2.0));
        let v12 = Rc::new(Vertex::new(3.0, 2.0));
        let v13 = Rc::new(Vertex::new(3.0, 3.0));
        let v14 = Rc::new(Vertex::new(2.0, 3.0));

        let mut hole_vertices: Vec<Rc<Vertex>> = Vec::new();
        hole_vertices.push(Rc::clone(&v11));
        hole_vertices.push(Rc::clone(&v12));
        hole_vertices.push(Rc::clone(&v13));
        hole_vertices.push(Rc::clone(&v14));
        let hole = Rc::new(Polyline::new_closed(hole_vertices).unwrap());

        /* Ok Vertices */
        let v21 = Rc::new(Vertex::new(1.4, 1.4));
        let v22 = Rc::new(Vertex::new(3.5, 3.5));

        /* Err Vertices */
        let v23 = Rc::new(Vertex::new(2.2, 2.2));
        let v24 = Rc::new(Vertex::new(2.0, 2.5));

        let mut inner_vertices: HashSet<Rc<Vertex>> = HashSet::new();
        inner_vertices.insert(Rc::clone(&v21));
        inner_vertices.insert(Rc::clone(&v22));
        inner_vertices.insert(Rc::clone(&v23));
        inner_vertices.insert(Rc::clone(&v24));

        let mut triangulator = Triangulator::new(&boundary);
        triangulator.insert_hole(&hole);
        let result = triangulator.insert_vertices(&inner_vertices);
        assert!(result.is_err());

        if let Err(panic_vertices) = result {
            assert!(panic_vertices.contains(&v23));
            assert!(panic_vertices.contains(&v24));
        }
    }
}

#[cfg(test)]
mod insert_segments {
    use super::*;

    #[test]
    fn sample_1() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let mut boundary: Vec<Rc<Vertex>> = Vec::new();
        boundary.push(Rc::clone(&v1));
        boundary.push(Rc::clone(&v2));
        boundary.push(Rc::clone(&v3));
        boundary.push(Rc::clone(&v4));
        let boundary = Rc::new(Polyline::new_closed(boundary).unwrap());

        /* Segments Vertices */
        let v11 = Rc::new(Vertex::new(2.0, 2.0));
        let v12 = Rc::new(Vertex::new(3.0, 2.0));
        let v13 = Rc::new(Vertex::new(3.0, 3.0));
        let v14 = Rc::new(Vertex::new(2.0, 3.0));
        let e1 = Rc::new(Edge::new(&v11, &v12));
        let e2 = Rc::new(Edge::new(&v13, &v14));

        let mut segments: HashSet<Rc<Edge>> = HashSet::new();
        segments.insert(Rc::clone(&e1));
        segments.insert(Rc::clone(&e2));

        let mut triangulator = Triangulator::new(&boundary);
        let result = triangulator.insert_segments(&segments);
        assert!(result.is_ok());

        assert!(triangulator.segments.contains(&e1));
        assert!(triangulator.segments.contains(&e2));
    }

    #[test]
    fn error_if_any_out_of_boundary() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let mut boundary: Vec<Rc<Vertex>> = Vec::new();
        boundary.push(Rc::clone(&v1));
        boundary.push(Rc::clone(&v2));
        boundary.push(Rc::clone(&v3));
        boundary.push(Rc::clone(&v4));
        let boundary = Rc::new(Polyline::new_closed(boundary).unwrap());

        /* Segments Vertices */
        let v11 = Rc::new(Vertex::new(2.0, 2.0));
        let v12 = Rc::new(Vertex::new(3.0, 2.0));
        let v13 = Rc::new(Vertex::new(3.0, 3.0));
        let v14 = Rc::new(Vertex::new(2.0, 5.0));
        let e1 = Rc::new(Edge::new(&v11, &v12));
        let e2 = Rc::new(Edge::new(&v13, &v14));

        let mut segments: HashSet<Rc<Edge>> = HashSet::new();
        segments.insert(Rc::clone(&e1));
        segments.insert(Rc::clone(&e2));

        let mut triangulator = Triangulator::new(&boundary);
        let result = triangulator.insert_segments(&segments);
        assert!(result.is_err());

        if let Err(panic_segments) = result {
            assert_eq!(panic_segments.len(), 1);
            assert!(panic_segments.contains(&e2));
        }
    }

    #[test]
    fn donot_remove_vertices_on_continence() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let mut boundary: Vec<Rc<Vertex>> = Vec::new();
        boundary.push(Rc::clone(&v1));
        boundary.push(Rc::clone(&v2));
        boundary.push(Rc::clone(&v3));
        boundary.push(Rc::clone(&v4));
        let boundary = Rc::new(Polyline::new_closed(boundary).unwrap());

        /* Segments */
        let v11 = Rc::new(Vertex::new(2.0, 2.0));
        let v12 = Rc::new(Vertex::new(3.0, 2.0));
        let v13 = Rc::new(Vertex::new(3.0, 3.0));
        let v14 = Rc::new(Vertex::new(2.0, 3.0));
        let e1 = Rc::new(Edge::new(&v11, &v12));
        let e2 = Rc::new(Edge::new(&v13, &v14));

        let mut segments: HashSet<Rc<Edge>> = HashSet::new();
        segments.insert(Rc::clone(&e1));
        segments.insert(Rc::clone(&e2));

        /* Vertices */
        let v21 = Rc::new(Vertex::new(2.5, 3.0));
        let v22 = Rc::new(Vertex::new(2.0, 2.2));

        let mut vertices: HashSet<Rc<Vertex>> = HashSet::new();
        vertices.insert(Rc::clone(&v21));
        vertices.insert(Rc::clone(&v22));

        let mut triangulator = Triangulator::new(&boundary);
        triangulator.insert_vertices(&vertices);
        assert!(!triangulator.vertices.is_empty());

        let result = triangulator.insert_segments(&segments);
        assert!(result.is_ok());

        assert!(!triangulator.vertices.contains(&v21));
        assert!(triangulator.vertices.contains(&v22));
    }

    #[test]
    fn error_on_inside_hole() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let boundary = Rc::new(
            Polyline::new_closed(vec![
                Rc::clone(&v1),
                Rc::clone(&v2),
                Rc::clone(&v3),
                Rc::clone(&v4),
            ])
            .unwrap(),
        );

        /* Segments */
        let v11 = Rc::new(Vertex::new(2.0, 2.0));
        let v12 = Rc::new(Vertex::new(3.0, 2.0));
        let v13 = Rc::new(Vertex::new(3.0, 3.0));
        let v14 = Rc::new(Vertex::new(2.0, 3.0));

        let hole = Rc::new(
            Polyline::new_closed(vec![
                Rc::clone(&v11),
                Rc::clone(&v12),
                Rc::clone(&v13),
                Rc::clone(&v14),
            ])
            .unwrap(),
        );

        /* Vertices */
        let v21 = Rc::new(Vertex::new(2.3, 2.3));
        let v22 = Rc::new(Vertex::new(2.8, 2.8));
        let v23 = Rc::new(Vertex::new(1.5, 1.5));
        let v24 = Rc::new(Vertex::new(1.5, 3.5));
        let e1 = Rc::new(Edge::new(&v21, &v22));
        let e2 = Rc::new(Edge::new(&v23, &v24));
        let segments_set: HashSet<Rc<Edge>> = vec![Rc::clone(&e1), Rc::clone(&e2)]
            .iter()
            .cloned()
            .collect();

        let mut triangulator = Triangulator::new(&boundary);
        triangulator.insert_hole(&hole);
        let result = triangulator.insert_segments(&segments_set);

        assert!(result.is_err());
        if let Err(conflicting_edges) = result {
            assert!(conflicting_edges.contains(&e1));
            assert!(!conflicting_edges.contains(&e2));
        }

        assert!(!triangulator.segments.contains(&e1));
        assert!(!triangulator.segments.contains(&e2));
    }

    #[test]
    fn error_on_hole_intersection() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let boundary = Rc::new(
            Polyline::new_closed(vec![
                Rc::clone(&v1),
                Rc::clone(&v2),
                Rc::clone(&v3),
                Rc::clone(&v4),
            ])
            .unwrap(),
        );

        /* Segments */
        let v11 = Rc::new(Vertex::new(2.0, 2.0));
        let v12 = Rc::new(Vertex::new(3.0, 2.0));
        let v13 = Rc::new(Vertex::new(3.0, 3.0));
        let v14 = Rc::new(Vertex::new(2.0, 3.0));

        let hole = Rc::new(
            Polyline::new_closed(vec![
                Rc::clone(&v11),
                Rc::clone(&v12),
                Rc::clone(&v13),
                Rc::clone(&v14),
            ])
            .unwrap(),
        );

        /* Vertices */
        let v21 = Rc::new(Vertex::new(2.3, 2.3));
        let v22 = Rc::new(Vertex::new(3.5, 2.3));
        let v23 = Rc::new(Vertex::new(1.5, 1.5));
        let v24 = Rc::new(Vertex::new(1.5, 3.5));
        let e1 = Rc::new(Edge::new(&v21, &v22));
        let e2 = Rc::new(Edge::new(&v23, &v24));
        let segments_set: HashSet<Rc<Edge>> = vec![Rc::clone(&e1), Rc::clone(&e2)]
            .iter()
            .cloned()
            .collect();

        let mut triangulator = Triangulator::new(&boundary);
        triangulator.insert_hole(&hole);
        assert!(triangulator.holes.contains(&hole));

        let result = triangulator.insert_segments(&segments_set);
        assert!(result.is_err());

        if let Err(conflicting_edges) = result {
            assert!(conflicting_edges.contains(&e1));
            assert!(!conflicting_edges.contains(&e2));
        }

        assert!(!triangulator.segments.contains(&e1));
        assert!(!triangulator.segments.contains(&e2));
    }

    #[test]
    fn error_on_hole_intersection_with_vertices_end_outside() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let boundary = Rc::new(
            Polyline::new_closed(vec![
                Rc::clone(&v1),
                Rc::clone(&v2),
                Rc::clone(&v3),
                Rc::clone(&v4),
            ])
            .unwrap(),
        );

        /* Hole */
        let v11 = Rc::new(Vertex::new(2.0, 2.0));
        let v12 = Rc::new(Vertex::new(3.0, 2.0));
        let v13 = Rc::new(Vertex::new(3.0, 3.0));
        let v14 = Rc::new(Vertex::new(2.0, 3.0));

        let hole = Rc::new(
            Polyline::new_closed(vec![
                Rc::clone(&v11),
                Rc::clone(&v12),
                Rc::clone(&v13),
                Rc::clone(&v14),
            ])
            .unwrap(),
        );

        /* Segments */
        let v21 = Rc::new(Vertex::new(1.5, 2.5));
        let v22 = Rc::new(Vertex::new(3.5, 2.5));
        let v23 = Rc::new(Vertex::new(3.5, 3.5));
        let v24 = Rc::new(Vertex::new(1.5, 3.5));
        let e1 = Rc::new(Edge::new(&v21, &v22));
        let e2 = Rc::new(Edge::new(&v23, &v24));
        let segments_set: HashSet<Rc<Edge>> = vec![Rc::clone(&e1), Rc::clone(&e2)]
            .iter()
            .cloned()
            .collect();

        let mut triangulator = Triangulator::new(&boundary);
        triangulator.insert_hole(&hole);
        let result = triangulator.insert_segments(&segments_set);

        assert!(result.is_err());
        if let Err(conflicting_edges) = result {
            assert!(conflicting_edges.contains(&e1));
            assert!(!conflicting_edges.contains(&e2));
        }

        assert!(!triangulator.segments.contains(&e1));
        assert!(!triangulator.segments.contains(&e2));
    }
}

#[cfg(test)]
mod triangulate {
    use super::*;

    #[test]
    fn triangulates_boundary_sample() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let boundary = Rc::new(
            Polyline::new_closed(vec![
                Rc::clone(&v1),
                Rc::clone(&v2),
                Rc::clone(&v3),
                Rc::clone(&v4),
            ])
            .unwrap(),
        );

        let mut triangulator = Triangulator::new(&boundary);
        triangulator.triangulate();

        for edge in boundary.into_edges().iter() {
            assert!(triangulator.triangulation.borrow().edges().contains(edge));
        }

        for triangle in triangulator
            .triangulation
            .borrow()
            .triangles
            .iter()
            .filter(|t| !t.is_ghost())
        {
            assert_eq!(
                boundary.contains(&triangle.center()),
                Some(Continence::Inside)
            );
        }
    }

    #[test]
    fn triangulates_hole() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let boundary = Rc::new(
            Polyline::new_closed(vec![
                Rc::clone(&v1),
                Rc::clone(&v2),
                Rc::clone(&v3),
                Rc::clone(&v4),
            ])
            .unwrap(),
        );

        /* hole */
        let v11 = Rc::new(Vertex::new(2.0, 2.0));
        let v12 = Rc::new(Vertex::new(3.0, 2.0));
        let v13 = Rc::new(Vertex::new(3.0, 3.0));
        let v14 = Rc::new(Vertex::new(2.0, 3.0));

        let hole = Rc::new(
            Polyline::new_closed(vec![
                Rc::clone(&v11),
                Rc::clone(&v12),
                Rc::clone(&v13),
                Rc::clone(&v14),
            ])
            .unwrap(),
        );
        let mut triangulator = Triangulator::new(&boundary);
        if triangulator.insert_hole(&hole).is_err() {
            panic!("Expected not err");
        }
        triangulator.triangulate();

        for edge in hole.into_edges().iter().chain(boundary.into_edges().iter()) {
            assert!(triangulator.triangulation.borrow().edges().contains(edge));
        }

        for triangle in triangulator
            .triangulation
            .borrow()
            .triangles
            .iter()
            .filter(|t| !t.is_ghost())
        {
            assert_eq!(hole.contains(&triangle.center()), Some(Continence::Outside));
            assert_eq!(
                boundary.contains(&triangle.center()),
                Some(Continence::Inside)
            );
        }
    }

    #[test]
    fn includes_segment_constraints() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let boundary = Rc::new(
            Polyline::new_closed(vec![
                Rc::clone(&v1),
                Rc::clone(&v2),
                Rc::clone(&v3),
                Rc::clone(&v4),
            ])
            .unwrap(),
        );

        /* Segments */
        let v21 = Rc::new(Vertex::new(1.5, 2.5));
        let v22 = Rc::new(Vertex::new(3.5, 2.5));
        let v23 = Rc::new(Vertex::new(3.5, 3.5));
        let v24 = Rc::new(Vertex::new(1.5, 3.5));

        let e1 = Rc::new(Edge::new(&v21, &v22));
        let e2 = Rc::new(Edge::new(&v23, &v24));
        let segments_set: HashSet<Rc<Edge>> = vec![Rc::clone(&e1), Rc::clone(&e2)]
            .iter()
            .cloned()
            .collect();

        let mut triangulator = Triangulator::new(&boundary);
        if triangulator.insert_segments(&segments_set).is_err() {
            panic!("Expected not err");
        }

        triangulator.triangulate();

        for constrained_edge in segments_set.iter().chain(boundary.into_edges().iter()) {
            assert!(Edge::decompose(
                &triangulator.triangulation.borrow().edges(),
                constrained_edge
            )
            .is_some());
        }
    }

    #[test]
    fn includes_vertex_constraints() {
        /* Squared boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let boundary = Rc::new(
            Polyline::new_closed(vec![
                Rc::clone(&v1),
                Rc::clone(&v2),
                Rc::clone(&v3),
                Rc::clone(&v4),
            ])
            .unwrap(),
        );

        /* Vertices */
        let v21 = Rc::new(Vertex::new(1.5, 2.5));
        let v22 = Rc::new(Vertex::new(3.5, 2.5));
        let v23 = Rc::new(Vertex::new(3.5, 3.5));
        let v24 = Rc::new(Vertex::new(1.5, 3.5));

        let vertices_set: HashSet<Rc<Vertex>> = vec![
            Rc::clone(&v21),
            Rc::clone(&v22),
            Rc::clone(&v23),
            Rc::clone(&v24),
        ]
        .iter()
        .cloned()
        .collect();

        let mut triangulator = Triangulator::new(&boundary);
        if triangulator.insert_vertices(&vertices_set).is_err() {
            panic!("Expected not err");
        }

        triangulator.triangulate();

        for constrained_vertex in vertices_set.iter() {
            assert!(triangulator
                .triangulation
                .borrow()
                .vertices()
                .contains(constrained_vertex));
        }
    }
}
