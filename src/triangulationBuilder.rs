use crate::continence::*;
use crate::orientation::*;
use crate::triangle::*;
use crate::vertex::*;
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
    - While there is a vertex left inserting, it inserts it.

At the end, there should be no vertex left inserting and no conflict
left resolving. The triangles will detain vertices and coordinates.

A triangle and a vertex are in conflict if the vertex is located
inside the circuncircle of the triangle.  */

pub struct Triangulator {
    vertices: Vec<Rc<Vertex>>,
    triangles: HashSet<Rc<Triangle>>,
    conflict_map: HashMap<Rc<Triangle>, Rc<Vertex>>,
    adjacency: HashMap<(Rc<Vertex>, Rc<Vertex>), Rc<Triangle>>,
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
        for ((v1, v2), triangle) in self.adjacency.iter() {
            write!(f, "({}, {}) -> {}\n", v1, v2, triangle);
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
    pub fn from_vertices(vertices_coordinates: Vec<f64>) -> Triangulator {
        Triangulator {
            vertices: Vertex::from_coordinates(vertices_coordinates),
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
            println!("{}", self);
            self.handle_conflict();
        }
    }

    fn init(&mut self) {
        let ghost_vertex = Rc::new(Vertex::new_ghost(self.vertices.len()));

        let mut v3 = self.vertices.pop().unwrap();
        let mut v2 = self.vertices.pop().unwrap();
        let mut v1 = self.vertices.pop().unwrap();

        /* Loops until 3 non colinear vertices are found */
        loop {
            match orient_2d(&v1, &v2, &v3) {
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
            }; /* end - match orient_2d */
        } /* end - loop */

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
        let triangle = Rc::clone(self.conflict_map.keys().next().unwrap());
        let vertex_to_insert = self.conflict_map.remove(&triangle).unwrap();
        self.remove_inner_adjacency(&triangle);

        let v1 = &triangle.v1;
        let v2 = &triangle.v2;
        let v3 = &triangle.v3;

        /* A list of edges and possible cavities to analyse */
        let mut pending_cavities: Vec<(Rc<Vertex>, Rc<Vertex>)> = vec![
            (Rc::clone(v1), Rc::clone(v2)),
            (Rc::clone(v2), Rc::clone(v3)),
            (Rc::clone(v3), Rc::clone(v1)),
        ];

        /* Recursive implementation to digCavity */
        loop {
            if pending_cavities.is_empty() {
                break;
            }

            let (v_begin, v_end) = pending_cavities.pop().unwrap();

            /* adjacent triangle is met by opposite half edge: end -> begin */
            let outer_triangle = Rc::clone(
                self.adjacency
                    .get(&(Rc::clone(&v_end), Rc::clone(&v_begin)))
                    .unwrap(),
            );

            /* If the cavity encircles the vertex, new cavities are to be analysed */
            if outer_triangle.encircles(&vertex_to_insert) == Continence::Inside {
                /* disassembles */
                self.remove_triangle(&outer_triangle);
                let outer_v1 = &outer_triangle.v1;
                let outer_v2 = &outer_triangle.v2;
                let outer_v3 = &outer_triangle.v3;

                /* includes cavities */
                if *outer_v1 == v_begin {
                    pending_cavities.push((Rc::clone(outer_v1), Rc::clone(outer_v2)));
                    pending_cavities.push((Rc::clone(outer_v2), Rc::clone(outer_v3)));
                } else if *outer_v2 == v_begin {
                    pending_cavities.push((Rc::clone(outer_v2), Rc::clone(outer_v3)));
                    pending_cavities.push((Rc::clone(outer_v3), Rc::clone(outer_v1)));
                } else {
                    pending_cavities.push((Rc::clone(outer_v3), Rc::clone(outer_v1)));
                    pending_cavities.push((Rc::clone(outer_v1), Rc::clone(outer_v2)));
                }
            } else {
                /* Includes new triangle */
                if v_begin.is_ghost {
                    let new_triangle = Rc::new(Triangle::new(&v_end, &vertex_to_insert, &v_begin));
                    self.include_triangle(&new_triangle);
                } else if v_end.is_ghost {
                    let new_triangle = Rc::new(Triangle::new(&vertex_to_insert, &v_begin, &v_end));
                    self.include_triangle(&new_triangle);
                } else {
                    let new_triangle = Rc::new(Triangle::new(&v_begin, &v_end, &vertex_to_insert));
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

        if let Some(vertex) = self.conflict_map.remove(triangle) {
            self.vertices.push(vertex);
            return;
        }

        panic!("Could not remove specied triangle");
    }

    fn include_inner_adjacency(&mut self, triangle: &Rc<Triangle>) {
        let v1 = &triangle.v1;
        let v2 = &triangle.v2;
        let v3 = &triangle.v3;
        self.adjacency
            .insert((Rc::clone(v1), Rc::clone(v2)), Rc::clone(triangle));
        self.adjacency
            .insert((Rc::clone(v2), Rc::clone(v3)), Rc::clone(triangle));
        self.adjacency
            .insert((Rc::clone(v3), Rc::clone(v1)), Rc::clone(triangle));
    }

    fn remove_inner_adjacency(&mut self, triangle: &Rc<Triangle>) {
        let v1 = &triangle.v1;
        let v2 = &triangle.v2;
        let v3 = &triangle.v3;
        self.adjacency.remove(&(Rc::clone(v1), Rc::clone(v2)));
        self.adjacency.remove(&(Rc::clone(v2), Rc::clone(v3)));
        self.adjacency.remove(&(Rc::clone(v3), Rc::clone(v1)));
    }
}

#[cfg(test)]
mod constructor {
    use super::*;

    #[test]
    fn test_constructor() {
        let mut vertex_indices = vec![0.0, 0.0, 2.0, 0.0, 1.0, 2.0];
        let builder = Triangulator::from_vertices(vertex_indices);
        println!("{}", builder);
        assert_eq!(builder.vertices.len(), 3);
    }
}

#[cfg(test)]
mod init {
    use super::*;

    #[test]
    fn test_init_single_triangle() {
        let mut vertex_indices = vec![0.0, 0.0, 2.0, 0.0, 1.0, 2.0];
        let mut builder = Triangulator::from_vertices(vertex_indices);
        builder.init();
        println!("{}", builder);
        assert_eq!(builder.vertices.len(), 0);
        assert_eq!(builder.triangles.len(), 4);
    }

    #[test]
    fn test_init_triangle_with_conflict() {
        let mut vertex_indices = vec![0.0, 0.0, 2.0, 0.0, 1.0, 2.0, 1.0, 1.0];
        let mut builder = Triangulator::from_vertices(vertex_indices);
        builder.init();
        println!("{}", builder);
        assert_eq!(builder.vertices.len(), 0);
        assert_eq!(builder.triangles.len() + builder.conflict_map.len(), 4);
    }
}

#[cfg(test)]
mod triangulate {
    use super::*;

    #[test]
    fn test_triangulate_4_vertices() {
        let mut vertex_indices = vec![0.0, 0.0, 2.0, 0.0, 1.0, 2.0, 1.0, 1.0];
        let mut builder = Triangulator::from_vertices(vertex_indices);
        builder.triangulate();
        println!("{}", builder);
        assert_eq!(builder.vertices.len(), 0);
        assert_eq!(builder.triangles.len(), 6);
        assert_eq!(builder.conflict_map.len(), 0);
    }
}
