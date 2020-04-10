use crate::Triangle::*;
use crate::Vertex::*;
use std::rc::Rc;
use std::collections::HashSet;
use std::collections::HashMap;

pub struct Triangulation {
    vertices: Vec<Rc<Vertex>>,
    triangles: HashSet<Triangle>,
}

impl Triangulation {
    /*
        TODO: 
            - implement constrained delaunay triangulation 
            - include segments as constraint
     */
    pub fn with_vertices(vertices_coordinates: Vec<f64>) -> Triangulation {
        Triangulation {
            vertices: Vertex::from_coordinates(vertices_coordinates),
            triangles: HashSet::new(),
            conflict_map: HasMap::new(),
        }
    }

    pub random_vertex(&self) -> Rc<Vertex> {

    }
    
    pub insert_vertex(&mut self) {}

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