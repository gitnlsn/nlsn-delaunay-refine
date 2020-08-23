use crate::elements::{edge::*, polyline::*, triangle::*, vertex::*};
use crate::planar::{refine_params::*, triangulation::*};
use crate::properties::continence::*;

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
        self.vertices = self
            .vertices
            .iter()
            .chain(vertices.iter())
            .cloned()
            .collect::<HashSet<Rc<Vertex>>>();

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
        /* Accumulate conflicting segments */
        let mut conflicting_segments: HashSet<Rc<Edge>> = HashSet::new();
        for segment in segments.iter() {
            let segment_polyline: Polyline =
                Polyline::new_opened(vec![Rc::clone(&segment.v1), Rc::clone(&segment.v2)]).unwrap();

            if Polyline::continence(&self.boundary, &segment_polyline) != Some(Continence::Inside) {
                conflicting_segments.insert(Rc::clone(segment));
                continue;
            }

            for hole in self.holes.iter() {
                if Polyline::continence(hole, &segment_polyline) != Some(Continence::Outside) {
                    conflicting_segments.insert(Rc::clone(segment));
                    continue;
                }
            }
        }

        if !conflicting_segments.is_empty() {
            return Err(conflicting_segments);
        }

        /* Split segments */
        let new_vertex_pairs = Edge::into_vertex_pairs(segments.iter().cloned().collect());
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

        let is_hole_inside_boundary =
            Polyline::continence(&self.boundary, hole) == Some(Continence::Inside);

        if !is_hole_inside_boundary {
            conflicting_vertices = conflicting_vertices
                .iter()
                .chain(self.boundary.vertices.iter())
                .cloned()
                .collect();
        }

        for existing_hole in self.holes.iter() {
            let is_hole_outside_existing_hole =
                Polyline::continence(existing_hole, hole) == Some(Continence::Outside);

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
            if relative_continence != Some(Continence::Outside) {
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
    pub fn refine(&mut self, params: RefineParams) -> Result<&Self, String> {
        if let Err(error) = self.triangulate() {
            return Err(error);
        }

        return Err(String::from("Not implemented yet"));
    }

    pub fn solve_encroachments(&self) {
        let mut encroach_map: HashMap<Rc<Edge>, HashSet<Rc<Vertex>>> = HashMap::new();

        let mut delaunay_vertices: HashSet<Rc<Vertex>> = self.triangulation.borrow().vertices();

        let delaunay_edges: HashSet<Rc<Edge>> = self
            .segments
            .iter()
            .chain(self.boundary.into_edges().iter())
            .chain(
                self.holes
                    .iter()
                    .map(|polyline| polyline.into_edges())
                    .flatten()
                    .collect::<HashSet<Rc<Edge>>>()
                    .iter(),
            )
            .cloned()
            .collect();

        Self::distribute_encroachments(&delaunay_edges, &mut delaunay_vertices, &mut encroach_map);

        loop {
            if encroach_map.is_empty() {
                break;
            }

            let encroached_edge: Rc<Edge> = Rc::clone(encroach_map.keys().next().unwrap());
            let mut encroached_vertices: HashSet<Rc<Vertex>> =
                encroach_map.remove(&encroached_edge).unwrap();

            if let Ok(new_segments) = self.split_segment(&encroached_edge) {
                Self::distribute_encroachments(
                    &new_segments,
                    &mut encroached_vertices,
                    &mut encroach_map,
                );
            } else {
                panic!(format!("Failed to split segment {}", encroached_edge));
            }
        }
    }

    fn distribute_encroachments(
        segments: &HashSet<Rc<Edge>>,
        vertices: &mut HashSet<Rc<Vertex>>,
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
     * Splitting segments
     */
    fn split_segment(&self, segment: &Rc<Edge>) -> Result<HashSet<Rc<Edge>>, String> {
        let edge_right = Rc::clone(segment);
        let edge_left = Rc::new(edge_right.opposite());

        let contains_right = self
            .triangulation
            .borrow()
            .adjacency
            .contains_key(&edge_right);

        let contains_left = self
            .triangulation
            .borrow()
            .adjacency
            .contains_key(&edge_left);

        if !contains_right || !contains_left {
            return Err(format!(
                "Triangulation does not contain segment {}",
                edge_right
            ));
        }

        /* Insert right side */
        let right_triangle: Rc<Triangle> = Rc::clone(
            self.triangulation
                .borrow_mut()
                .adjacency
                .get(&edge_right)
                .unwrap(),
        );

        let right_midpoint: Rc<Vertex> = Rc::new(edge_right.midpoint());
        let right_opposite: Rc<Vertex> = right_triangle.opposite_vertex(&edge_right).unwrap();
        let right_triangle_1 = Rc::new(Triangle::new(
            &edge_right.v1,
            &right_midpoint,
            &right_opposite,
        ));
        let right_triangle_2 = Rc::new(Triangle::new(
            &right_midpoint,
            &edge_right.v2,
            &right_opposite,
        ));

        self.triangulation
            .borrow_mut()
            .remove_triangle(&right_triangle);
        self.triangulation
            .borrow_mut()
            .include_triangle(&right_triangle_1);
        self.triangulation
            .borrow_mut()
            .include_triangle(&right_triangle_2);

        /* Insert left side */
        let left_triangle: Rc<Triangle> = Rc::clone(
            self.triangulation
                .borrow_mut()
                .adjacency
                .get(&edge_left)
                .unwrap(),
        );

        let left_midpoint: Rc<Vertex> = Rc::new(edge_left.midpoint());
        let left_opposite: Rc<Vertex> = left_triangle.opposite_vertex(&edge_left).unwrap();
        let left_triangle_1 = Rc::new(Triangle::new(&edge_left.v1, &left_midpoint, &left_opposite));
        let left_triangle_2 = Rc::new(Triangle::new(&left_midpoint, &edge_left.v2, &left_opposite));

        self.triangulation
            .borrow_mut()
            .remove_triangle(&left_triangle);
        self.triangulation
            .borrow_mut()
            .include_triangle(&left_triangle_1);
        self.triangulation
            .borrow_mut()
            .include_triangle(&left_triangle_2);

        /* Return results */
        let mut included_triangles: HashSet<Rc<Edge>> = HashSet::new();

        included_triangles.insert(Rc::new(Edge::new(&edge_right.v1, &right_midpoint)));
        included_triangles.insert(Rc::new(Edge::new(&right_midpoint, &edge_right.v1)));

        return Ok(included_triangles);
    }

    /**
     * Triangulates
     * Returns error if there is wrong input or constraints.
     * Else returns the triangulation.
     */
    pub fn triangulate(&mut self) -> Result<&Self, String> {
        let mut constraint_segments: HashSet<Rc<Edge>> = HashSet::new();

        /* Boundary inclusion */
        self.include_boundary();
        let boundary_segments: HashSet<Rc<Edge>> =
            self.boundary.into_edges().iter().cloned().collect();

        constraint_segments = constraint_segments
            .iter()
            .chain(boundary_segments.iter())
            .cloned()
            .collect();

        /* Holes inclusion */
        let holes: HashSet<Rc<Polyline>> = self.holes.iter().cloned().collect();
        for hole in holes.iter() {
            let included_segments = self.include_hole(&hole, &constraint_segments);

            constraint_segments = constraint_segments
                .iter()
                .chain(included_segments.iter())
                .cloned()
                .collect();
        }

        /* Inner Segments inclusion */
        let included_segments = self.include_segments(
            self.segments.iter().cloned().collect::<HashSet<Rc<Edge>>>(),
            &constraint_segments,
        );
        constraint_segments = constraint_segments
            .iter()
            .chain(included_segments.iter())
            .cloned()
            .collect();

        /* Inner vertices inclusion */
        self.include_vertices(
            self.vertices.iter().cloned().collect::<Vec<Rc<Vertex>>>(),
            &constraint_segments,
        );
        return Ok(self);
    }

    /**
     * Inserts boundary into triangulation,
     * with incremental insertion and
     * avoids inserting triangles outside boundary
     */
    fn include_boundary(&self) {
        let mut pending_vertices: Vec<Rc<Vertex>> =
            self.boundary.vertices.iter().cloned().collect();

        let mut conflict_map: HashMap<Rc<Triangle>, Vec<Rc<Vertex>>> = HashMap::new();

        let ghost_vertex: Rc<Vertex> = Rc::new(Vertex::new_ghost());
        let v1: Rc<Vertex> = pending_vertices.remove(0);
        let v2: Rc<Vertex> = pending_vertices.remove(0);

        let t1: Rc<Triangle> = Rc::new(Triangle::new(&v1, &v2, &ghost_vertex));
        let t2: Rc<Triangle> = Rc::new(Triangle::new(&v2, &v1, &ghost_vertex));

        Self::distribute_conflicts_if_inside_boundary(
            &t1,
            &mut conflict_map,
            &mut pending_vertices,
            &self.boundary,
        );

        Self::distribute_conflicts_if_inside_boundary(
            &t2,
            &mut conflict_map,
            &mut pending_vertices,
            &self.boundary,
        );

        self.triangulation.borrow_mut().include_triangle(&t1);
        self.triangulation.borrow_mut().include_triangle(&t2);

        while !conflict_map.is_empty() {
            let next_conflicting_triangle: Rc<Triangle> =
                Rc::clone(conflict_map.keys().next().unwrap());

            let mut conflicting_vertices: Vec<Rc<Vertex>> =
                conflict_map.remove(&next_conflicting_triangle).unwrap();

            let conflict_vertex: Rc<Vertex> = conflicting_vertices.pop().unwrap();

            self.triangulation
                .borrow_mut()
                .remove_triangle(&next_conflicting_triangle);

            let (e1, e2, e3) = next_conflicting_triangle.inner_edges();

            let mut pending_cavities: Vec<Rc<Edge>> = vec![e1, e2, e3];

            if !conflicting_vertices.is_empty() {
                /* reinclude conflicts they remaining */
                pending_vertices.append(&mut conflicting_vertices);
            }

            while !pending_cavities.is_empty() {
                let edge: Rc<Edge> = pending_cavities.pop().unwrap();
                let edge_to_outer_triangle: Rc<Edge> = Rc::new(edge.opposite());
                let outer_triangle: Rc<Triangle> = Rc::clone(
                    self.triangulation
                        .borrow_mut()
                        .adjacency
                        .get(&edge_to_outer_triangle)
                        .unwrap(),
                );

                let trace_to_conflict = Polyline::new_opened(vec![
                    Rc::clone(&conflict_vertex),
                    Rc::new(outer_triangle.center()),
                ])
                .unwrap();

                let is_inside_boundary =
                    Polyline::intersection_vertices(&self.boundary, &trace_to_conflict).is_empty();

                let is_conflicting =
                    outer_triangle.encircles(&conflict_vertex) != Continence::Outside;

                if is_inside_boundary && is_conflicting {
                    self.triangulation
                        .borrow_mut()
                        .remove_triangle(&outer_triangle);
                    let (e12, e23, e31) = outer_triangle.inner_edges();

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
                        let new_triangle =
                            Rc::new(Triangle::new(&edge.v2, &conflict_vertex, &edge.v1));
                        self.triangulation
                            .borrow_mut()
                            .include_triangle(&new_triangle);

                        Self::distribute_conflicts_if_inside_boundary(
                            &new_triangle,
                            &mut conflict_map,
                            &mut pending_vertices,
                            &self.boundary,
                        );
                    } else if edge.v2.is_ghost {
                        let new_triangle =
                            Rc::new(Triangle::new(&conflict_vertex, &edge.v1, &edge.v2));
                        self.triangulation
                            .borrow_mut()
                            .include_triangle(&new_triangle);

                        Self::distribute_conflicts_if_inside_boundary(
                            &new_triangle,
                            &mut conflict_map,
                            &mut pending_vertices,
                            &self.boundary,
                        );
                    } else {
                        let new_triangle =
                            Rc::new(Triangle::new(&edge.v1, &edge.v2, &conflict_vertex));
                        self.triangulation
                            .borrow_mut()
                            .include_triangle(&new_triangle);

                        Self::distribute_conflicts_if_inside_boundary(
                            &new_triangle,
                            &mut conflict_map,
                            &mut pending_vertices,
                            &self.boundary,
                        );
                    }
                }
            } /* end - while pending edges */
        } /* end - while conflict maps not empty */
    }

    /**
     * Include hole and returns included segments
     */
    fn include_hole(
        &mut self,
        hole: &Rc<Polyline>,
        segment_constraints: &HashSet<Rc<Edge>>,
    ) -> HashSet<Rc<Edge>> {
        let hole_segments: HashSet<Rc<Edge>> = hole
            .into_edges()
            .iter()
            .cloned()
            .collect::<HashSet<Rc<Edge>>>();

        let included_segments = self.include_segments(hole_segments, segment_constraints);

        /* Inserting holes */
        let mut pending_edges: Vec<Rc<Edge>> = Vec::new();
        let ghost_vertex = Rc::new(Vertex::new_ghost());

        for hole_edge in included_segments.iter() {
            let edge_to_hole = Rc::clone(&hole_edge);
            if self
                .triangulation
                .borrow()
                .adjacency
                .contains_key(&edge_to_hole)
            {
                let inner_triangle = Rc::clone(
                    self.triangulation
                        .borrow()
                        .adjacency
                        .get(&edge_to_hole)
                        .unwrap(),
                );

                self.triangulation
                    .borrow_mut()
                    .remove_triangle(&inner_triangle);

                let (e1, e2, e3) = inner_triangle.inner_edges();
                if !included_segments.contains(&e1) {
                    pending_edges.push(Rc::new(e1.opposite()));
                }
                if !included_segments.contains(&e2) {
                    pending_edges.push(Rc::new(e2.opposite()));
                }
                if !included_segments.contains(&e3) {
                    pending_edges.push(Rc::new(e3.opposite()));
                }
            }

            let ghost_triangle = Rc::new(Triangle::new(
                &edge_to_hole.v1,
                &edge_to_hole.v2,
                &ghost_vertex,
            ));
            self.triangulation
                .borrow_mut()
                .include_triangle(&ghost_triangle);
        }

        /* Flood fill - removes possible deeper triangles */
        loop {
            if pending_edges.is_empty() {
                break;
            }

            let edge_to_hole = Rc::clone(&pending_edges.pop().unwrap());
            if self
                .triangulation
                .borrow()
                .adjacency
                .contains_key(&edge_to_hole)
            {
                let inner_triangle = Rc::clone(
                    self.triangulation
                        .borrow()
                        .adjacency
                        .get(&edge_to_hole)
                        .unwrap(),
                );

                self.triangulation
                    .borrow_mut()
                    .remove_triangle(&inner_triangle);

                let (e1, e2, e3) = inner_triangle.inner_edges();
                pending_edges.push(Rc::new(e1.opposite()));
                pending_edges.push(Rc::new(e2.opposite()));
                pending_edges.push(Rc::new(e3.opposite()));
            }
        }
        return included_segments;
    } /* end - include holes */

    /**
     * Inserts vertices in the triangulation
     */
    fn include_vertices(
        &self,
        mut vertices: Vec<Rc<Vertex>>,
        segment_constraints: &HashSet<Rc<Edge>>,
    ) {
        let mut conflict_map: HashMap<Rc<Triangle>, Vec<Rc<Vertex>>> = HashMap::new();
        for possible_triangle in self.triangulation.borrow().triangles.iter() {
            if vertices.is_empty() {
                break;
            }
            Self::distribute_conflicts_if_inside_boundary(
                possible_triangle,
                &mut conflict_map,
                &mut vertices,
                &self.boundary,
            );
        }

        while !conflict_map.is_empty() {
            let next_conflicting_triangle: Rc<Triangle> =
                Rc::clone(conflict_map.keys().next().unwrap());

            let mut conflicting_vertices: Vec<Rc<Vertex>> =
                conflict_map.remove(&next_conflicting_triangle).unwrap();

            let conflict_vertex: Rc<Vertex> = conflicting_vertices.pop().unwrap();

            self.triangulation
                .borrow_mut()
                .remove_triangle(&next_conflicting_triangle);

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
                    self.triangulation
                        .borrow()
                        .adjacency
                        .get(&edge_to_outer_triangle)
                        .unwrap(),
                );

                let is_conflicting =
                    outer_triangle.encircles(&conflict_vertex) == Continence::Inside;

                let is_constrained = segment_constraints.contains(&edge_to_outer_triangle);

                if is_conflicting && !is_constrained {
                    self.triangulation
                        .borrow_mut()
                        .remove_triangle(&outer_triangle);
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
                        let new_triangle =
                            Rc::new(Triangle::new(&edge.v2, &conflict_vertex, &edge.v1));
                        self.triangulation
                            .borrow_mut()
                            .include_triangle(&new_triangle);

                        Self::distribute_conflicts_if_inside_boundary(
                            &new_triangle,
                            &mut conflict_map,
                            &mut vertices,
                            &self.boundary,
                        );
                    } else if edge.v2.is_ghost {
                        let new_triangle =
                            Rc::new(Triangle::new(&conflict_vertex, &edge.v1, &edge.v2));
                        self.triangulation
                            .borrow_mut()
                            .include_triangle(&new_triangle);

                        Self::distribute_conflicts_if_inside_boundary(
                            &new_triangle,
                            &mut conflict_map,
                            &mut vertices,
                            &self.boundary,
                        );
                    } else {
                        let new_triangle =
                            Rc::new(Triangle::new(&edge.v1, &edge.v2, &conflict_vertex));
                        self.triangulation
                            .borrow_mut()
                            .include_triangle(&new_triangle);

                        Self::distribute_conflicts_if_inside_boundary(
                            &new_triangle,
                            &mut conflict_map,
                            &mut vertices,
                            &self.boundary,
                        );
                    }
                }
            } /* end - while pending edges */
        } /* end - distributing vertices */
    } /* end - include vertices method */

    /**
     * Returns included Segments possibly splitted
     */
    fn include_segments(
        &mut self,
        mut segments: HashSet<Rc<Edge>>,
        segment_constraints: &HashSet<Rc<Edge>>,
    ) -> HashSet<Rc<Edge>> {
        let segments_vertices: Vec<Rc<Vertex>> = segments
            .iter()
            .map(|edge| vec![Rc::clone(&edge.v1), Rc::clone(&edge.v2)])
            .flatten()
            .collect::<HashSet<Rc<Vertex>>>()
            .iter()
            .cloned()
            .collect::<Vec<Rc<Vertex>>>();

        self.include_vertices(segments_vertices, segment_constraints);

        let mut counter = 0;
        loop {
            let existing_segments = self.triangulation.borrow().edges();
            let mut missing_segments: Vec<Rc<Edge>> = segments
                .iter()
                .filter(|edge| !existing_segments.contains(&Rc::clone(edge)))
                .cloned()
                .collect();

            if missing_segments.is_empty() {
                break;
            }

            self.include_vertices(
                missing_segments
                    .iter()
                    .map(|edge| Rc::new(edge.midpoint()))
                    .collect(),
                &segment_constraints
                    .iter()
                    .chain(segments.iter())
                    .cloned()
                    .collect::<HashSet<Rc<Edge>>>(),
            );

            while let Some(segment) = missing_segments.pop() {
                let midpoint = Rc::new(segment.midpoint());
                let splited_segment_right = Rc::new(Edge::new(&segment.v1, &midpoint));
                let splited_segment_left = Rc::new(Edge::new(&midpoint, &segment.v2));

                segments.remove(&segment);
                segments.insert(splited_segment_left);
                segments.insert(splited_segment_right);
            }

            if counter == 2 {
                break;
            } else {
                counter = counter + 1;
            }
        }
        return segments;
    } /* end - include segments method */

    fn distribute_conflicts_if_inside_boundary(
        triangle: &Rc<Triangle>,
        conflict_map: &mut HashMap<Rc<Triangle>, Vec<Rc<Vertex>>>,
        vertices: &mut Vec<Rc<Vertex>>,
        boundary: &Polyline,
    ) {
        let mut distributed_conflicts: Vec<Rc<Vertex>> = Vec::new();

        for _ in 0..vertices.len() {
            let pending_vertex: Rc<Vertex> = vertices.remove(0);
            let has_conflict = triangle.encircles(&pending_vertex) != Continence::Outside;

            let mut is_inside_boundary = false;
            if triangle.is_ghost() {
                let p2 = Polyline::new_closed(vec![
                    Rc::clone(&triangle.v1),
                    Rc::clone(&triangle.v2),
                    Rc::clone(&pending_vertex),
                ])
                .unwrap();
                let (subtraction_list, _) = Polyline::subtraction(&p2, &boundary);
                is_inside_boundary = subtraction_list.is_empty();
            } else {
                let p2 = Polyline::new_opened(vec![
                    Rc::new(triangle.center()),
                    Rc::clone(&pending_vertex),
                ])
                .unwrap();
                let intersections = Polyline::intersection_vertices(&boundary, &p2);
                is_inside_boundary = intersections.is_empty();
            }

            if has_conflict && is_inside_boundary {
                distributed_conflicts.push(pending_vertex);
                continue;
            }

            vertices.push(pending_vertex);
        }
        if !distributed_conflicts.is_empty() {
            conflict_map.insert(Rc::clone(&triangle), distributed_conflicts);
        }
    }
}

#[cfg(test)]
mod boundaries {
    use super::*;

    #[test]
    fn include_boundary_sample_1() {
        /* hexagon */
        let mut vertices: Vec<Rc<Vertex>> = Vec::new();
        vertices.push(Rc::new(Vertex::new(1.0, 0.0)));
        vertices.push(Rc::new(Vertex::new(2.0, 0.0)));
        vertices.push(Rc::new(Vertex::new(3.0, 1.0)));
        vertices.push(Rc::new(Vertex::new(2.0, 2.0)));
        vertices.push(Rc::new(Vertex::new(1.0, 2.0)));
        vertices.push(Rc::new(Vertex::new(0.0, 1.0)));

        let boundary = Rc::new(Polyline::new_closed(vertices).unwrap());
        let mut triangulator = Triangulator::new(&boundary);
        triangulator.include_boundary();

        let solid_triangles: Vec<Rc<Triangle>> = triangulator
            .triangulation
            .borrow()
            .triangles
            .iter()
            .filter(|triangle| !triangle.is_ghost())
            .cloned()
            .collect();
        assert_eq!(solid_triangles.len(), 4);
    }

    #[test]
    fn include_boundary_sample_2() {
        /* zigzag path */
        let mut vertices: Vec<Rc<Vertex>> = Vec::new();
        vertices.push(Rc::new(Vertex::new(0.0, 1.0)));
        vertices.push(Rc::new(Vertex::new(1.0, 1.5)));
        vertices.push(Rc::new(Vertex::new(2.0, 1.0)));
        vertices.push(Rc::new(Vertex::new(3.0, 1.5)));
        vertices.push(Rc::new(Vertex::new(3.0, 3.5)));
        vertices.push(Rc::new(Vertex::new(2.0, 3.0)));
        vertices.push(Rc::new(Vertex::new(1.0, 3.5)));
        vertices.push(Rc::new(Vertex::new(0.0, 3.0)));

        let boundary = Rc::new(Polyline::new_closed(vertices).unwrap());
        let mut triangulator = Triangulator::new(&boundary);
        triangulator.include_boundary();

        let solid_triangles: Vec<Rc<Triangle>> = triangulator
            .triangulation
            .borrow()
            .triangles
            .iter()
            .filter(|triangle| !triangle.is_ghost())
            .cloned()
            .collect();

        assert_eq!(solid_triangles.len(), 6);
    }

    #[test]
    fn include_boundary_sample_3() {
        /* concave domain */
        let mut vertices: Vec<Rc<Vertex>> = Vec::new();
        vertices.push(Rc::new(Vertex::new(4.0, 1.0)));
        vertices.push(Rc::new(Vertex::new(5.0, 1.0)));
        vertices.push(Rc::new(Vertex::new(6.0, 2.0)));
        vertices.push(Rc::new(Vertex::new(4.0, 4.0)));
        vertices.push(Rc::new(Vertex::new(3.0, 4.0)));
        vertices.push(Rc::new(Vertex::new(1.0, 2.0)));
        vertices.push(Rc::new(Vertex::new(2.0, 1.0)));
        vertices.push(Rc::new(Vertex::new(3.0, 1.0)));
        vertices.push(Rc::new(Vertex::new(2.0, 2.0)));
        vertices.push(Rc::new(Vertex::new(3.0, 3.0)));
        vertices.push(Rc::new(Vertex::new(4.0, 3.0)));
        vertices.push(Rc::new(Vertex::new(5.0, 2.0)));

        let boundary = Rc::new(Polyline::new_closed(vertices).unwrap());
        let mut triangulator = Triangulator::new(&boundary);
        triangulator.include_boundary();

        let solid_triangles: Vec<Rc<Triangle>> = triangulator
            .triangulation
            .borrow()
            .triangles
            .iter()
            .filter(|triangle| !triangle.is_ghost())
            .cloned()
            .collect();

        assert_eq!(solid_triangles.len(), 10);
    }
}

#[cfg(test)]
mod include_holes {
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

        let mut triangulator = Triangulator::new(&boundary);
        triangulator.insert_hole(&hole);
        triangulator.include_boundary();
        triangulator.include_hole(&hole, &HashSet::new());

        let ghost_triangles: Vec<Rc<Triangle>> = triangulator
            .triangulation
            .borrow()
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
    }

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

        let mut triangulator = Triangulator::new(&boundary);
        triangulator.insert_hole(&hole);
        triangulator.include_boundary();
        triangulator.include_hole(&hole, &HashSet::new());

        let ghost_triangles: Vec<Rc<Triangle>> = triangulator
            .triangulation
            .borrow()
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
    }

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
        let requested_hole_polyline = Rc::new(Polyline::new_closed(hole).unwrap());

        let mut triangulator = Triangulator::new(&boundary);
        triangulator.insert_hole(&requested_hole_polyline);
        triangulator.include_boundary();
        let hole_edges: HashSet<Rc<Edge>> =
            triangulator.include_hole(&requested_hole_polyline, &HashSet::new());

        let inserted_hole_polyline = Polyline::arrange(&hole_edges).unwrap();

        let (subtraction_1, _) =
            Polyline::subtraction(&inserted_hole_polyline, &requested_hole_polyline);
        let (subtraction_2, _) =
            Polyline::subtraction(&inserted_hole_polyline, &requested_hole_polyline);

        assert!(subtraction_1.is_empty());
        assert!(subtraction_2.is_empty());
    }
}

#[cfg(test)]
mod include_vertices {
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

        let v5 = Rc::new(Vertex::new(3.0, 2.0));
        let v6 = Rc::new(Vertex::new(4.0, 3.0));
        let v7 = Rc::new(Vertex::new(3.0, 4.0));
        let v8 = Rc::new(Vertex::new(2.0, 3.0));

        let vertices: HashSet<Rc<Vertex>> = vec![
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ]
        .iter()
        .cloned()
        .collect();

        let mut triangulator = Triangulator::new(&boundary);
        triangulator.insert_vertices(&vertices);
        triangulator.include_boundary();
        triangulator.include_vertices(vertices.iter().cloned().collect(), &HashSet::new());

        let vertices: HashSet<Rc<Vertex>> = triangulator.triangulation.borrow().vertices();
        assert!(vertices.contains(&v5));
        assert!(vertices.contains(&v6));
        assert!(vertices.contains(&v7));
        assert!(vertices.contains(&v8));
    }
}

#[cfg(test)]
mod include_segments {
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

        let v5 = Rc::new(Vertex::new(3.0, 2.0));
        let v6 = Rc::new(Vertex::new(4.0, 3.0));
        let v7 = Rc::new(Vertex::new(3.0, 4.0));
        let v8 = Rc::new(Vertex::new(2.0, 3.0));

        let e56 = Rc::new(Edge::new(&v5, &v6));
        let e78 = Rc::new(Edge::new(&v7, &v8));

        let segments: HashSet<Rc<Edge>> = vec![Rc::clone(&e56), Rc::clone(&e78)]
            .iter()
            .cloned()
            .collect();

        let mut triangulator = Triangulator::new(&boundary);
        triangulator.include_boundary();
        triangulator.include_segments(segments, &HashSet::new());

        let edges: HashSet<Rc<Edge>> = triangulator.triangulation.borrow().edges();
        assert!(edges.contains(&e56));
        assert!(edges.contains(&e78));
    }

    #[test]
    fn sample_2() {
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

        /* concave hole - closed polyline */
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

        let segments: HashSet<Rc<Edge>> = vec![
            Rc::new(Edge::new(&v5, &v6)),
            Rc::new(Edge::new(&v6, &v7)),
            Rc::new(Edge::new(&v7, &v8)),
            Rc::new(Edge::new(&v8, &v9)),
            Rc::new(Edge::new(&v9, &v10)),
            Rc::new(Edge::new(&v10, &v11)),
            Rc::new(Edge::new(&v11, &v12)),
            Rc::new(Edge::new(&v12, &v13)),
            Rc::new(Edge::new(&v13, &v14)),
            Rc::new(Edge::new(&v14, &v15)),
            Rc::new(Edge::new(&v15, &v16)),
            Rc::new(Edge::new(&v16, &v5)),
        ]
        .iter()
        .cloned()
        .collect::<HashSet<Rc<Edge>>>();

        let mut triangulator = Triangulator::new(&boundary);
        triangulator.include_boundary();
        let inserted_segments =
            triangulator.include_segments(segments.iter().cloned().collect(), &HashSet::new());

        let inserted_hole_polyline = Polyline::arrange(&inserted_segments)
            .unwrap()
            .minified_noncolinear();

        for edge in segments.iter() {
            assert!(inserted_hole_polyline.into_edges().contains(edge));
        }
    }
}

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
        if let Err(panic_edges) = &result {
            for edge in panic_edges.iter() {
                println!("{}", edge);
            }
        }
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

        assert!(triangulator.vertices.contains(&v21));
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
        let result = triangulator.triangulate();

        assert!(result.is_ok());

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
        let result = triangulator.triangulate();
        assert!(result.is_ok());

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

        let result = triangulator.triangulate();
        assert!(result.is_ok());

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

        let result = triangulator.triangulate();
        assert!(result.is_ok());

        for constrained_vertex in vertices_set.iter() {
            assert!(triangulator
                .triangulation
                .borrow()
                .vertices()
                .contains(constrained_vertex));
        }
    }
}
