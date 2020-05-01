use crate::vertex::*;
use crate::edge::*;
use crate::triangle::*;
use crate::triangulation::*;

use crate::continence::*;
use crate::orientation::*;

use std::collections::HashMap;
use std::collections::HashSet;
use std::fmt;
use std::mem;
use std::rc::Rc;

/* Triangulator will build the triangulation by inserting triangles
and removing vertices.

    - It starts by creating vertices from vector coordinates and
    choosing three vertices to compose the first triangle.
    - For each new triangle, a conflict is searched.
    - While there is a conflict, it resolves the conflict.

At the end, there should be no vertex left inserting and no conflict
left resolving. The triangles will detain vertices and coordinates.

A triangle and a vertex are in conflict if the vertex is located
inside the circumcircle of the triangle.  */

pub struct Triangulator {
    vertices: Vec<Rc<Vertex>>,
    triangles: HashSet<Rc<Triangle>>,
    conflict_map: HashMap<Rc<Triangle>, Rc<Vertex>>,
    adjacency: HashMap<Rc<Edge>, Rc<Triangle>>,
}

impl fmt::Display for Triangulator {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Vertices\n");
        for vertex in self.vertices.iter() {
            write!(f, "{}\n", vertex);
        }
        write!(f, "\nTriangles\n");
        for triangle in self.triangles.iter() {
            write!(f, "{}\n", triangle);
        }
        write!(f, "\nConflicts\n");
        for (triangle, vertex) in self.conflict_map.iter() {
            write!(f, "{} -> {}\n", triangle, vertex);
        }
        write!(f, "\nAdjacency\n");
        for (edge, triangle) in self.adjacency.iter() {
            write!(f, "{} -> {}\n", edge, triangle);
        }
        return write!(f, "");
    }
}

impl Triangulator {
    /*
       TODO:
           - implement constrained delaunay triangulation
           - include segments as constraint
    */
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            triangles: HashSet::new(),
            conflict_map: HashMap::new(),
            adjacency: HashMap::new(),
        }
    }

    pub fn from_vertices(vertices: Vec<Rc<Vertex>>) -> Self {
        Self {
            vertices: vertices,
            triangles: HashSet::new(),
            conflict_map: HashMap::new(),
            adjacency: HashMap::new(),
        }
    }

    pub fn from_coordinates(vertices_coordinates: Vec<f64>) -> Self {
        Self {
            vertices: Vertex::from_coordinates(&vertices_coordinates),
            triangles: HashSet::new(),
            conflict_map: HashMap::new(),
            adjacency: HashMap::new(),
        }
    }

    pub fn triangulate(&mut self) {
        let should_init = self.triangles.len() + self.conflict_map.len() == 0;

        if should_init {
            self.init();
        }
        while self.conflict_map.len() > 0 {
            self.handle_conflict();
        }
    }

    /**
     * Removes triangles through flood fill.
     * Coordinates will define the hole boundary by vertex insertion.
     * Tringles from inner hole will be removed from boundary to greater depth through adjacency.
     */
    pub fn insert_hole(&mut self, coordinates: Vec<f64>) {
        let vertices_list: Vec<Rc<Vertex>> = Vertex::from_coordinates(&coordinates);
        let edges_list: Vec<Rc<Edge>> = Edge::from_vertices(&vertices_list);

        let mut pending_edges: Vec<Rc<Edge>> = Vec::new();

        for vertex in vertices_list.iter() {
            self.insert_vertex(Rc::clone(vertex));
        }

        let ghost_vertex = Rc::new(Vertex::new_ghost());

        /* 
            0-depth triangles are removed
            At boundary, Ghost triangles are inserted.
            Inner edges are added to the pending list.
        */
        for edge_to_hole in edges_list.iter() {
            let edge_to_hole = Rc::clone(&edge_to_hole);
            if let Some(inner_triangle) = self.adjacency.get(&edge_to_hole) {
                let triangle_to_remove: Rc<Triangle> = Rc::clone(inner_triangle);
                self.remove_triangle(&triangle_to_remove);
                
                let (e1, e2, e3) = triangle_to_remove.inner_edges();
                if !edges_list.contains(&e1) {
                    pending_edges.push(Rc::new(e1.opposite()));
                }
                if !edges_list.contains(&e2) {
                    pending_edges.push(Rc::new(e2.opposite()));
                }
                if !edges_list.contains(&e3) {
                    pending_edges.push(Rc::new(e3.opposite()));
                }
            }

            /* 
                Due to the ghost triangle, 
                a vertex insertion in the hole results in filling the hole.
            */
            let ghost_triangle = Rc::new(Triangle::new(
                &edge_to_hole.v1,
                &edge_to_hole.v2,
                &ghost_vertex,
            ));
            self.include_triangle(&ghost_triangle);
        }

        /* 
            Flood fill
            actual deepth triangles are removed if they exist
            backward deepth is longer connected by adjacency
            all possible edges are added to the pending list
            only n+1 deepth will have remaining triangles to be removed
        */
        loop {
            if pending_edges.is_empty() {
                break;
            }

            let edge_to_hole = Rc::clone(&pending_edges.pop().unwrap());
            if let Some(inner_triangle) = self.adjacency.get(&edge_to_hole) {
                let triangle_to_remove: Rc<Triangle> = Rc::clone(inner_triangle);
                self.remove_triangle(&triangle_to_remove);
                
                let (e1, e2, e3) = triangle_to_remove.inner_edges();
                pending_edges.push(Rc::new(e1.opposite()));
                pending_edges.push(Rc::new(e2.opposite()));
                pending_edges.push(Rc::new(e3.opposite()));
            }
        }
    }

    pub fn insert_vertex(&mut self, vertex: Rc<Vertex>) {
        if let Some(conflicting_triangle) = self
            .triangles
            .iter()
            .find(|triangle| triangle.encircles(&vertex) == Continence::Inside)
        {
            let conflicting_triangle = Rc::clone(conflicting_triangle);
            self.triangles.remove(&conflicting_triangle);
            self.conflict_map.insert(conflicting_triangle, vertex);
            self.handle_conflict();
            return;
        };

        panic!("Expected to find conflicting triangle to insert vertex");
    }

    pub fn delete_vertex(&mut self, vertex: Rc<Vertex>) {
        if let Some(index) = self
            .vertices
            .iter()
            .position(|possible| possible == &vertex)
        {
            /* if vertex was not inserted yet, avoids insert and return */
            self.vertices.remove(index);
            return;
        }

        /* Else removes triangles withe the specified vertex and inserts a  */
        let conflicting_triangles: Vec<Rc<Triangle>> = self
            .triangles
            .iter()
            .filter(|triangle| {
                let is_v1 = triangle.v1 == vertex;
                let is_v2 = triangle.v2 == vertex;
                let is_v3 = triangle.v3 == vertex;
                return is_v1 || is_v2 || is_v3;
            })
            .cloned()
            .collect();

        for triangle in conflicting_triangles.iter() {
            if triangle.is_ghost() {
                panic!("Cannot delete vertex at boundary");
            }
        }

        for triangle in conflicting_triangles.iter() {
            self.remove_triangle(triangle);
        }

        let mut vertices_set: HashSet<Rc<Vertex>> = HashSet::new();

        for triangle in conflicting_triangles.iter() {
            vertices_set.insert(Rc::clone(&triangle.v1));
            vertices_set.insert(Rc::clone(&triangle.v2));
            vertices_set.insert(Rc::clone(&triangle.v3));
        }

        let vertices_vec: Vec<Rc<Vertex>> = vertices_set
            .iter()
            .filter(|&possible| *possible != vertex)
            .cloned()
            .collect();

        let mut inner_triangulation = Self::from_vertices(vertices_vec);
        inner_triangulation.triangulate();

        self.merge_triangles(inner_triangulation);
    }

    pub fn export(&self) -> Triangulation {
        /* Separates solid triangles only */
        let solid_triangles: HashSet<Rc<Triangle>> = self
            .triangles
            .iter()
            .filter(|triangle| !triangle.is_ghost())
            .cloned()
            .collect();

        /* HashSet will avoid duplicates */
        let mut vertices_set: HashSet<Rc<Vertex>> = HashSet::new();
        for triangle in solid_triangles.iter() {
            vertices_set.insert(Rc::clone(&triangle.v1));
            vertices_set.insert(Rc::clone(&triangle.v2));
            vertices_set.insert(Rc::clone(&triangle.v3));
        }

        /* vertices array sorted by position */
        let mut vertices_vec: Vec<Rc<Vertex>> = vertices_set.iter().cloned().collect();
        vertices_vec.sort();

        /* mapping of vertex into its index */
        let mut vertices_index_mapping: HashMap<Rc<Vertex>, usize> = HashMap::new();
        for index in 0..vertices_vec.len() {
            let vertex = Rc::clone(vertices_vec.get(index).unwrap());
            vertices_index_mapping.insert(vertex, index);
        }

        let mut coordinates: Vec<f64> = Vec::new();
        for vertex in vertices_vec.iter() {
            coordinates.push(vertex.x);
            coordinates.push(vertex.y);
        }

        let mut triangle_index_array: Vec<usize> = Vec::new();
        for triangle in solid_triangles.iter() {
            let v1_index = vertices_index_mapping.get(&triangle.v1).unwrap();
            let v2_index = vertices_index_mapping.get(&triangle.v2).unwrap();
            let v3_index = vertices_index_mapping.get(&triangle.v3).unwrap();
            let indices = vec![v1_index, v2_index, v3_index];
            let min_index = indices.iter().min().unwrap();
            if min_index == &v1_index {
                triangle_index_array.push(*v1_index);
                triangle_index_array.push(*v2_index);
                triangle_index_array.push(*v3_index);
            } else if min_index == &v2_index {
                triangle_index_array.push(*v2_index);
                triangle_index_array.push(*v3_index);
                triangle_index_array.push(*v1_index);
            } else {
                triangle_index_array.push(*v3_index);
                triangle_index_array.push(*v1_index);
                triangle_index_array.push(*v2_index);
            }
        }

        return Triangulation::from(coordinates, triangle_index_array);
    }

    pub fn vertices_size(&self) -> usize {
        let mut vertices_set: HashSet<Rc<Vertex>> = self.vertices.iter().cloned().collect();
        for triangle in self.triangles.iter() {
            vertices_set.insert(Rc::clone(&triangle.v1));
            vertices_set.insert(Rc::clone(&triangle.v2));
            vertices_set.insert(Rc::clone(&triangle.v3));
        }

        return vertices_set
            .iter()
            .filter(|vertex| !vertex.is_ghost)
            .count();
    }

    pub fn triangles_size(&self) -> usize {
        let mut triangles_set: HashSet<Rc<Triangle>> = self.triangles.iter().cloned().collect();

        for triangle in self.conflict_map.keys() {
            triangles_set.insert(Rc::clone(triangle));
            triangles_set.insert(Rc::clone(triangle));
            triangles_set.insert(Rc::clone(triangle));
        }

        return triangles_set
            .iter()
            .filter(|triangle| !triangle.is_ghost())
            .count();
    }

    fn init(&mut self) {
        let ghost_vertex = Rc::new(Vertex::new_ghost());

        let mut v3 = self.vertices.pop().unwrap();
        let mut v2 = self.vertices.pop().unwrap();
        let mut v1 = self.vertices.pop().unwrap();

        /* Loops until 3 non colinear vertices are found */
        loop {
            match orientation(&v1, &v2, &v3) {
                Orientation::Counterclockwise => {
                    break;
                }
                Orientation::Clockwise => {
                    mem::swap(&mut v2, &mut v3);
                    break;
                }
                Orientation::Colinear => {
                    self.vertices.insert(0, v3);
                    v3 = self.vertices.pop().unwrap();
                }
            }; /* match orient_2d */
        } /* loop */

        let solid_triangle = Rc::new(Triangle::new(&v1, &v2, &v3));
        let tghost_1 = Rc::new(Triangle::new(&v2, &v1, &ghost_vertex));
        let tghost_2 = Rc::new(Triangle::new(&v3, &v2, &ghost_vertex));
        let tghost_3 = Rc::new(Triangle::new(&v1, &v3, &ghost_vertex));

        self.include_triangle(&solid_triangle);
        self.include_triangle(&tghost_1);
        self.include_triangle(&tghost_2);
        self.include_triangle(&tghost_3);
    }

    fn handle_conflict(&mut self) {
        if self.conflict_map.is_empty() {
            panic!("No conflit to handle");
        }

        /* starts by disassembling the conflicting triangle */
        let triangle: Rc<Triangle> = Rc::clone(self.conflict_map.keys().next().unwrap());
        let vertex_to_insert = self.conflict_map.remove(&triangle).unwrap();
        self.remove_inner_adjacency(&triangle);

        let (e12, e23, e31) = triangle.inner_edges();

        /* A list of edges and possible cavities to analyse */
        let mut pending_cavities: Vec<Rc<Edge>> = vec![e12, e23, e31];

        /* Recursive implementation to digCavity */
        loop {
            if pending_cavities.is_empty() {
                break;
            }

            let edge: Rc<Edge> = pending_cavities.pop().unwrap();
            let edge_to_outer_triangle: Rc<Edge> = Rc::new(edge.opposite());

            /* adjacent triangle is met by opposite half edge: end -> begin */
            let outer_triangle: Rc<Triangle> =
                Rc::clone(self.adjacency.get(&edge_to_outer_triangle).unwrap());

            /* If the cavity encircles the vertex, new cavities are to be analysed */
            if outer_triangle.encircles(&vertex_to_insert) == Continence::Inside {
                /* disassembles */
                self.remove_triangle(&outer_triangle);
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
                        Rc::new(Triangle::new(&edge.v2, &vertex_to_insert, &edge.v1));
                    self.include_triangle(&new_triangle);
                } else if edge.v2.is_ghost {
                    let new_triangle =
                        Rc::new(Triangle::new(&vertex_to_insert, &edge.v1, &edge.v2));
                    self.include_triangle(&new_triangle);
                } else {
                    let new_triangle =
                        Rc::new(Triangle::new(&edge.v1, &edge.v2, &vertex_to_insert));
                    self.include_triangle(&new_triangle);
                }
            }
        } /* loop */
    } /* handle_conflict */

    fn include_triangle(&mut self, triangle: &Rc<Triangle>) {
        self.include_inner_adjacency(triangle);

        match self.vertices.iter().position(|vertex| {
            /* searchs for conflicting vertex */
            triangle.encircles(vertex) == Continence::Inside
        }) {
            Some(index) => {
                let conflicting_vertex = self.vertices.remove(index);
                self.conflict_map
                    .insert(Rc::clone(triangle), Rc::clone(&conflicting_vertex));
            }
            None => {
                self.triangles.insert(Rc::clone(triangle));
            }
        }
    }

    fn remove_triangle(&mut self, triangle: &Rc<Triangle>) {
        self.remove_inner_adjacency(triangle);

        if self.triangles.remove(triangle) {
            return;
        }

        /*  if the triangle has a conflict, vertex should be moved back to vertices vec */
        if let Some(vertex) = self.conflict_map.remove(triangle) {
            self.vertices.push(vertex);
            return;
        }

        panic!("Could not remove specied triangle");
    }

    fn include_inner_adjacency(&mut self, triangle: &Rc<Triangle>) {
        let (e12, e23, e31) = triangle.inner_edges();
        self.adjacency.insert(e12, Rc::clone(triangle));
        self.adjacency.insert(e23, Rc::clone(triangle));
        self.adjacency.insert(e31, Rc::clone(triangle));
    }

    fn remove_inner_adjacency(&mut self, triangle: &Rc<Triangle>) {
        let (e12, e23, e31) = triangle.inner_edges();
        self.adjacency.remove(&e12);
        self.adjacency.remove(&e23);
        self.adjacency.remove(&e31);
    }

    /**
     * Should be used against triangulations with no conflicts triangulations
     */
    fn merge_triangles(&mut self, other: Self) {
        let solid_triangle_vec: Vec<Rc<Triangle>> = other
            .triangles
            .iter()
            .filter(|triangle| !triangle.is_ghost())
            .cloned()
            .collect();

        for triangle in solid_triangle_vec {
            self.triangles.insert(Rc::clone(&triangle));
        }

        for (edge, val) in other.adjacency.iter() {
            self.adjacency.insert(Rc::clone(edge), Rc::clone(val));
        }
    }
}

#[cfg(test)]
mod constructor {
    use super::*;

    #[test]
    fn test_constructor() {
        let vertex_indices = vec![0.0, 0.0, 2.0, 0.0, 1.0, 2.0];
        let builder = Triangulator::from_coordinates(vertex_indices);
        assert_eq!(builder.vertices.len(), 3);
    }
}

#[cfg(test)]
mod init {
    use super::*;

    #[test]
    fn test_init_single_triangle() {
        let vertex_indices = vec![0.0, 0.0, 2.0, 0.0, 1.0, 2.0];
        let mut builder = Triangulator::from_coordinates(vertex_indices);
        builder.init();
        assert_eq!(builder.vertices.len(), 0);
        assert_eq!(builder.triangles.len(), 4);
    }

    #[test]
    fn test_init_triangle_with_conflict() {
        let vertex_indices = vec![0.0, 0.0, 2.0, 0.0, 1.0, 2.0, 1.0, 1.0];
        let mut builder = Triangulator::from_coordinates(vertex_indices);
        builder.init();
        assert_eq!(builder.vertices.len(), 0);
        assert_eq!(builder.triangles.len() + builder.conflict_map.len(), 4);
    }
}

#[cfg(test)]
mod triangulate {
    use super::*;

    #[test]
    fn test_triangulate_4_vertices() {
        let vertex_indices = vec![0.0, 0.0, 2.0, 0.0, 1.0, 2.0, 1.0, 1.0];
        let mut builder = Triangulator::from_coordinates(vertex_indices);
        builder.triangulate();
        assert_eq!(builder.vertices.len(), 0);
        assert_eq!(builder.triangles.len(), 6);
        assert_eq!(builder.conflict_map.len(), 0);
    }
}

#[cfg(test)]
mod delete_vertex {
    use super::*;

    #[test]
    fn test_remove_from_inside_triangle() {
        let vertex_indices = vec![0.0, 0.0, 2.0, 0.0, 1.0, 2.0, 1.0, 1.0];
        let mut triangulator = Triangulator::from_coordinates(vertex_indices);
        triangulator.triangulate();
        triangulator.delete_vertex(Rc::new(Vertex::new(1.0, 1.0)));
        let solid_triangles: Vec<Rc<Triangle>> = triangulator
            .triangles
            .iter()
            .filter(|triangle| !triangle.is_ghost())
            .cloned()
            .collect();
        assert_eq!(solid_triangles.len(), 1);
    }

    #[test]
    fn test_remove_from_inside_hexagon() {
        let vertex_indices = vec![
            1.0, 0.0, 2.0, 0.0, 3.0, 1.0, 2.0, 2.0, 1.0, 2.0, 0.0, 1.0, 1.2, 1.0, 2.0, 1.0,
        ];
        /*
           (1.0, 0.0)
           (2.0, 0.0)
           (3.0, 1.0)
           (2.0, 2.0)
           (1.0, 2.0)
           (0.0, 1.0)
           (1.2, 1.0)
           (2.0, 1.0)
        */
        let mut triangulator = Triangulator::from_coordinates(vertex_indices);
        triangulator.triangulate();
        let solid_triangles: Vec<Rc<Triangle>> = triangulator
            .triangles
            .iter()
            .filter(|triangle| !triangle.is_ghost())
            .cloned()
            .collect();
        assert_eq!(solid_triangles.len(), 8);

        triangulator.delete_vertex(Rc::new(Vertex::new(2.0, 1.0)));
        let solid_triangles: Vec<Rc<Triangle>> = triangulator
            .triangles
            .iter()
            .filter(|triangle| !triangle.is_ghost())
            .cloned()
            .collect();
        assert_eq!(solid_triangles.len(), 6);
    }

    #[test]
    #[should_panic]
    fn test_panics_at_boundary() {
        let vertex_indices = vec![0.0, 0.0, 2.0, 0.0, 1.0, 2.0, 1.0, 1.0];
        let mut triangulator = Triangulator::from_coordinates(vertex_indices);
        triangulator.triangulate();
        triangulator.delete_vertex(Rc::new(Vertex::new(2.0, 0.0)));
    }
}

#[cfg(test)]
mod insert_vertex {
    use super::*;

    #[test]
    fn test_insert_outside() {
        let vertex_indices = vec![0.0, 0.0, 2.0, 0.0, 1.0, 2.0];
        let mut triangulator = Triangulator::from_coordinates(vertex_indices);
        triangulator.triangulate();

        let new_vertex = Rc::new(Vertex::new(2.0, 2.0));
        triangulator.insert_vertex(new_vertex);
        let solid_triangles: Vec<Rc<Triangle>> = triangulator
            .triangles
            .iter()
            .filter(|triangle| !triangle.is_ghost())
            .cloned()
            .collect();
        assert_eq!(solid_triangles.len(), 2);
    }

    #[test]
    fn test_inside_triangle() {
        let vertex_indices = vec![0.0, 0.0, 2.0, 0.0, 1.0, 2.0];
        let mut triangulator = Triangulator::from_coordinates(vertex_indices);
        triangulator.triangulate();
        let solid_triangles: Vec<Rc<Triangle>> = triangulator
            .triangles
            .iter()
            .filter(|triangle| !triangle.is_ghost())
            .cloned()
            .collect();
        assert_eq!(solid_triangles.len(), 1);

        let new_vertex = Rc::new(Vertex::new(1.0, 1.0));
        triangulator.insert_vertex(new_vertex);
        let solid_triangles: Vec<Rc<Triangle>> = triangulator
            .triangles
            .iter()
            .filter(|triangle| !triangle.is_ghost())
            .cloned()
            .collect();
        assert_eq!(solid_triangles.len(), 3);
    }

    #[test]
    fn test_inside_hexagon() {
        let vertex_indices = vec![
            1.0, 0.0, 2.0, 0.0, 3.0, 1.0, 2.0, 2.0, 1.0, 2.0, 0.0, 1.0, 1.2, 1.0,
        ];
        /*
           (1.0, 0.0)
           (2.0, 0.0)
           (3.0, 1.0)
           (2.0, 2.0)
           (1.0, 2.0)
           (0.0, 1.0)
           (1.2, 1.0)
           (2.0, 1.0)
        */
        let mut triangulator = Triangulator::from_coordinates(vertex_indices);
        triangulator.triangulate();
        let solid_triangles: Vec<Rc<Triangle>> = triangulator
            .triangles
            .iter()
            .filter(|triangle| !triangle.is_ghost())
            .cloned()
            .collect();
        assert_eq!(solid_triangles.len(), 6);

        let new_vertex = Rc::new(Vertex::new(2.0, 1.0));
        triangulator.insert_vertex(new_vertex);
        let solid_triangles: Vec<Rc<Triangle>> = triangulator
            .triangles
            .iter()
            .filter(|triangle| !triangle.is_ghost())
            .cloned()
            .collect();
        assert_eq!(solid_triangles.len(), 8);
    }
}

#[cfg(test)]
mod insert_hole {
    use super::*;

    /**
     * Simple 0-depth hole insertion
     */
    #[test]
    fn test_triangle_hole_inside_triangle() {
        let mut triangulator = Triangulator::from_coordinates(vec![0.0, 0.0, 10.0, 0.0, 5.0, 10.0]);
        /*
            ( 0.0,  0.0)
            (10.0,  0.0)
            ( 5.0, 10.0)
        */
        let hole_path = vec![5.0, 2.0, 4.0, 3.0, 3.0, 3.0];
        /*
           (5.0, 2.0)
           (4.0, 3.0)
           (3.0, 3.0)
        */

        triangulator.triangulate();
        triangulator.insert_hole(hole_path);

        assert_eq!(triangulator.vertices_size(), 6);
        assert_eq!(triangulator.triangles_size(), 6);
    }

    /**
     * Defines requirement for flood fill in hole insertion.
     */
    #[test]
    fn test_star_inside_square() {
        let mut triangulator =
            Triangulator::from_coordinates(vec![0.0, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 10.0]);
        /*
            ( 0.0,  0.0)
            (10.0,  0.0)
            (10.0, 10.0)
            ( 0.0, 10.0)
        */
        let hole_path = vec![
            5.0, 2.0, 6.0, 4.0, 8.0, 5.0, 6.0, 6.0, 5.0, 8.0, 4.0, 6.0, 2.0, 5.0, 4.0, 4.0,
        ];
        /*
           (5.0, 2.0)
           (6.0, 4.0)
           (8.0, 5.0)
           (6.0, 6.0)
           (5.0, 8.0)
           (4.0, 6.0)
           (2.0, 5.0)
           (4.0, 4.0)
        */

        triangulator.triangulate();
        triangulator.insert_hole(hole_path);

        assert_eq!(triangulator.vertices_size(), 12);
        assert_eq!(triangulator.triangles_size(), 12);
    }
}

#[cfg(test)]
mod export {
    use super::*;

    #[test]
    fn test_export_vertices() {
        let vertex_indices = vec![0.0, 0.0, 2.0, 0.0, 1.0, 2.0, 1.0, 1.0];
        let mut triangulator = Triangulator::from_coordinates(vertex_indices);
        triangulator.triangulate();
        let triangulation = triangulator.export();
        println!("{}", triangulation);
        assert!(
            triangulation
                .coordinates
                .chunks(2)
                .position(|slice| slice == [0.0, 0.0])
                != None
        );
        assert!(
            triangulation
                .coordinates
                .chunks(2)
                .position(|slice| slice == [2.0, 0.0])
                != None
        );
        assert!(
            triangulation
                .coordinates
                .chunks(2)
                .position(|slice| slice == [1.0, 2.0])
                != None
        );
        assert!(
            triangulation
                .coordinates
                .chunks(2)
                .position(|slice| slice == [1.0, 1.0])
                != None
        );
    }

    #[test]
    fn test_export_triangles() {
        let vertex_indices = vec![0.0, 0.0, 2.0, 0.0, 1.0, 2.0, 1.0, 1.0];
        let mut triangulator = Triangulator::from_coordinates(vertex_indices);
        triangulator.triangulate();
        let triangulation = triangulator.export();
        println!("{}", triangulation);
        assert!(
            triangulation
                .triangles
                .chunks(3)
                .position(|slice| slice == [0, 3, 1])
                != None
        );
        assert!(
            triangulation
                .triangles
                .chunks(3)
                .position(|slice| slice == [0, 1, 2])
                != None
        );
        assert!(
            triangulation
                .triangles
                .chunks(3)
                .position(|slice| slice == [1, 3, 2])
                != None
        );
    }
}

#[cfg(test)]
mod triangulation {
    use super::*;

    #[test]
    fn test_square_with_center() {
        let vertex_indices = vec![0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.5, 0.5];
        /*
            (0.0, 0.0)
            (1.0, 0.0)
            (1.0, 1.0)
            (0.0, 1.0)
            (0.5, 0.5)
        */
        let mut triangulator = Triangulator::from_coordinates(vertex_indices);
        triangulator.triangulate();
        let triangulation = triangulator.export();
        println!("{}", triangulation);
        assert!(
            triangulation
                .triangles
                .chunks(3)
                .position(|slice| slice == [2, 3, 4])
                != None
        );
        assert!(
            triangulation
                .triangles
                .chunks(3)
                .position(|slice| slice == [1, 2, 4])
                != None
        );
        assert!(
            triangulation
                .triangles
                .chunks(3)
                .position(|slice| slice == [0, 2, 1])
                != None
        );
        assert!(
            triangulation
                .triangles
                .chunks(3)
                .position(|slice| slice == [0, 3, 2])
                != None
        );
    }

    #[test]
    fn test_hexagon() {
        let vertex_indices = vec![
            1.0, 0.0, 2.0, 0.0, 3.0, 1.0, 2.0, 2.0, 1.0, 2.0, 0.0, 1.0, 1.2, 1.0, 2.0, 1.0,
        ];
        /*
           (1.0, 0.0)
           (2.0, 0.0)
           (3.0, 1.0)
           (2.0, 2.0)
           (1.0, 2.0)
           (0.0, 1.0)
           (1.2, 1.0)
           (2.0, 1.0)
        */
        let mut triangulator = Triangulator::from_coordinates(vertex_indices);
        triangulator.triangulate();
        triangulator.delete_vertex(Rc::new(Vertex::new(2.0, 1.0)));
        let triangulation = triangulator.export();
        println!("{}", triangulation);
        assert!(
            triangulation
                .triangles
                .chunks(3)
                .position(|slice| slice == [0, 3, 2])
                != None
        );
        assert!(
            triangulation
                .triangles
                .chunks(3)
                .position(|slice| slice == [1, 4, 3])
                != None
        );
        assert!(
            triangulation
                .triangles
                .chunks(3)
                .position(|slice| slice == [0, 1, 3])
                != None
        );
        assert!(
            triangulation
                .triangles
                .chunks(3)
                .position(|slice| slice == [2, 3, 5])
                != None
        );
        assert!(
            triangulation
                .triangles
                .chunks(3)
                .position(|slice| slice == [3, 4, 6])
                != None
        );
        assert!(
            triangulation
                .triangles
                .chunks(3)
                .position(|slice| slice == [3, 6, 5])
                != None
        );
    }
}
