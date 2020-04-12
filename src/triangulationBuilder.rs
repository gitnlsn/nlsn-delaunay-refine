use crate::continence::*;
use crate::orientation::*;
use crate::triangle::*;
use crate::vertex::*;
use std::collections::HashMap;
use std::collections::HashSet;
use std::mem;
use std::rc::Rc;
use std::fmt;

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
    conflict_map: Vec<(Rc<Triangle>, Rc<Vertex>)>,
    adjacency: HashMap<(Rc<Vertex>, Rc<Vertex>), Rc<Triangle>>
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
            conflict_map: Vec::new(),
            adjacency: HashMap::new(),
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
        let tghost_1 = Rc::new(Triangle::new(&v1, &v2, &ghost_vertex));
        let tghost_2 = Rc::new(Triangle::new(&v2, &v3, &ghost_vertex));
        let tghost_3 = Rc::new(Triangle::new(&v3, &v1, &ghost_vertex));

        self.include_triangle(solid_triangle);
        self.include_triangle(tghost_1);
        self.include_triangle(tghost_2);
        self.include_triangle(tghost_3);
    }
    
    fn handle_conflict(&mut self) {
        let (triangle, vertex_to_insert) = self.conflict_map.pop().unwrap();
        self.remove_inner_adjacency(&triangle);

        let v1 = &triangle.v1;
        let v2 = &triangle.v2;
        let v3 = &triangle.v3;

        let mut pending_cavities: Vec<(Rc<Vertex>, Rc<Vertex>)> = vec![
            (Rc::clone(v1),Rc::clone(v2)),
            (Rc::clone(v2),Rc::clone(v3)),
            (Rc::clone(v3),Rc::clone(v1)),
        ];

        loop {
            let (v_begin, v_end) = pending_cavities.pop().unwrap();

            // adjacent triangle is met by opposite half edge: end -> begin
            let outer_triangle = self.adjacency.remove(&(Rc::clone(&v_end), Rc::clone(&v_begin))).unwrap();

            if outer_triangle.encircles(&vertex_to_insert) == Continence::Inside {
                self.remove_inner_adjacency(&outer_triangle);
                let outer_v1 = &outer_triangle.v1;
                let outer_v2 = &outer_triangle.v2;
                let outer_v3 = &outer_triangle.v3;

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
                continue;
            } else {
                self.adjacency.insert((Rc::clone(&v_end), Rc::clone(&v_begin)), Rc::clone(&outer_triangle));
                let new_triangle = Rc::new(Triangle::new(&v_begin, &v_end, &vertex_to_insert));
                self.include_triangle(new_triangle);
            }
            
            if pending_cavities.is_empty() {
                break;
            }
        };
    }

    fn include_triangle(&mut self, triangle: Rc<Triangle>) {
        self.include_inner_adjacency(&triangle);
        match self.vertices.iter().position(|vertex| {
            /* searchs for conflicting vertex */
            triangle.encircles(vertex) == Continence::Inside
        }) {
            Some(index) => {
                let conflicting_vertex = self.vertices.remove(index);
                self.conflict_map.push((triangle, conflicting_vertex));
            }
            None => {
                self.triangles.insert(triangle);
            }
        }
    }

    fn include_inner_adjacency(&mut self, triangle: &Rc<Triangle>) {
        let v1 = &triangle.v1;
        let v2 = &triangle.v2;
        let v3 = &triangle.v3;
        self.adjacency.insert((Rc::clone(v1),Rc::clone(v2)), Rc::clone(triangle));
        self.adjacency.insert((Rc::clone(v2),Rc::clone(v3)), Rc::clone(triangle));
        self.adjacency.insert((Rc::clone(v3),Rc::clone(v1)), Rc::clone(triangle));
    }

    fn remove_inner_adjacency(&mut self, triangle: &Rc<Triangle>) {
        let v1 = &triangle.v1;
        let v2 = &triangle.v2;
        let v3 = &triangle.v3;
        self.adjacency.remove(&(Rc::clone(v1),Rc::clone(v2)));
        self.adjacency.remove(&(Rc::clone(v2),Rc::clone(v3)));
        self.adjacency.remove(&(Rc::clone(v3),Rc::clone(v1)));
    }
}

#[cfg(test)]
mod constructor {
    use super::*;

    #[test]
    fn test_constructor() {
        let mut vertex_indices = Vec::new();
        vertex_indices.push(0.0);
        vertex_indices.push(0.0);
        vertex_indices.push(2.0);
        vertex_indices.push(0.0);
        vertex_indices.push(1.0);
        vertex_indices.push(2.0);
        let builder = Triangulator::from_vertices(vertex_indices);
        println!("{}", builder);
        assert_eq!(builder.vertices.len(), 3);
    }

    #[test]
    fn test_init_single_triangle() {
        let mut vertex_indices = Vec::new();
        vertex_indices.push(0.0); vertex_indices.push(0.0);
        vertex_indices.push(2.0); vertex_indices.push(0.0);
        vertex_indices.push(1.0); vertex_indices.push(2.0);
        let mut builder = Triangulator::from_vertices(vertex_indices);
        builder.init();
        println!("{}", builder);
        assert_eq!(builder.vertices.len(), 0);
        assert_eq!(builder.triangles.len(), 4);
    }

    #[test]
    fn test_init_triangle_with_conflict() {
        let mut vertex_indices = Vec::new();
        vertex_indices.push(0.0); vertex_indices.push(0.0);
        vertex_indices.push(2.0); vertex_indices.push(0.0);
        vertex_indices.push(1.0); vertex_indices.push(2.0);
        vertex_indices.push(1.0); vertex_indices.push(1.0);
        let mut builder = Triangulator::from_vertices(vertex_indices);
        builder.init();
        println!("{}", builder);
        assert_eq!(builder.vertices.len(), 0);
        assert_eq!(builder.triangles.len() + builder.conflict_map.len(), 4);
    }
}
