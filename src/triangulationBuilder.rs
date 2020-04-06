use crate::Orientation::*;
use crate::Triangle::*;
use crate::Vertex::*;
use std::collections::HashMap;
use std::collections::HashSet;
use std::rc::Rc;
use std::mem;

pub struct Triangulator {
    vertices: Vec<Rc<Vertex>>,
    triangles: HashSet<Triangle>,
    conflict_map: HashMap<Triangle, Rc<Vertex>>,
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
        }
    }

    fn init(&mut self) {
        let ghost_vertex = Rc::new(Vertex::new_ghost());
        
        let mut v1 = self.vertices.pop().unwrap();
        let mut v2 = self.vertices.pop().unwrap();
        let mut v3 = self.vertices.pop().unwrap();
        
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
                },
            }; /* end - match orient_2d */
        } /* end - loop */

        let solid_triangle = Triangle::new(&v1, &v2, &v3);
        let tghost_1 = Triangle::new(&v1, &v2, &ghost_vertex);
        let tghost_2 = Triangle::new(&v2, &v3, &ghost_vertex);
        let tghost_3 = Triangle::new(&v3, &v1, &ghost_vertex);

        self.triangles.insert(solid_triangle);
        self.triangles.insert(tghost_1);
        self.triangles.insert(tghost_2);
        self.triangles.insert(tghost_3);
    }

    fn insert_vertex(&mut self) {}

    fn set_triangle(&mut self, v1: &Rc<Vertex>, v2: &Rc<Vertex>, v3: &Rc<Vertex>) {
        self.triangles.insert(Triangle::new(v1, v2, v3));
    }

    fn unset_triangle(&mut self, undesired_triangle: &Rc<Triangle>) {
        self.triangles.remove(undesired_triangle);
    }
}

#[cfg(test)]
mod structure {
    use super::*;
}
