use crate::continence::*;
use crate::encroachment::*;
use crate::distance::*;
use crate::vertex::*;
use std::rc::Rc;

use std::hash::Hash;
use std::cmp::Eq;

use std::fmt;
use std::fmt::Debug;

#[derive(Hash, Debug)]
pub struct Edge {
    pub v1: Rc<Vertex>,
    pub v2: Rc<Vertex>,
}

impl PartialEq for Edge {
    fn eq(&self, other: &Self) -> bool {
        /* oriented edge */
        self.v1 == other.v1 && self.v2 == other.v2
    }
}

impl Eq for Edge {}

impl fmt::Display for Edge {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        return write!(f, "({} - {})", self.v1, self.v2);
    }
}

impl Edge {
    pub fn new(v1: &Rc<Vertex>, v2: &Rc<Vertex>) -> Self {
        Self {
            v1: Rc::clone(v1),
            v2: Rc::clone(v2),
        }
    }

    pub fn opposite(&self) -> Self {
        Self {
            v1: Rc::clone(&self.v2),
            v2: Rc::clone(&self.v1),
        }
    }

    pub fn length(&self) -> f64 {
        return distance(&self.v1, &self.v2);
    }

    pub fn encroach(&self, vertex: &Vertex) -> Continence {
        return encroach(&self.v1, &self.v2, vertex);
    }

    pub fn midpoint(&self) -> Vertex {
        let x1 = self.v1.x;
        let y1 = self.v1.y;
        let x2 = self.v2.x;
        let y2 = self.v2.y;

        let x_mid = (x1 + x2) / 2.0;
        let y_mid = (y1 + y2) / 2.0;
        
        return Vertex::new(x_mid, y_mid);
    }

    pub fn from_coordinates(coordinates: &Vec<f64>) -> Vec<Rc<Edge>> {
        if coordinates.len() % 2 != 0 {
            panic!("Vec must provide vertices by pair of x,y coordinates.");
        }

        let vertices_list = Vertex::from_coordinates(coordinates);
        let mut edge_list: Vec<Rc<Edge>> = Vec::new();

        for index in 0..vertices_list.len() {
            let v1 = vertices_list.get(index).unwrap();
            let v2 = match vertices_list.get(index + 1) {
                Some(vertex) => vertex,
                None => vertices_list.get(0).unwrap(),
            };
            let new_edge = Rc::new(Edge::new(v1, v2));
            edge_list.push(new_edge);
        }

        return edge_list;
    }

    pub fn from_vertices(vertices_list: &Vec<Rc<Vertex>>) -> Vec<Rc<Edge>> {
        let mut edge_list: Vec<Rc<Edge>> = Vec::new();

        for index in 0..vertices_list.len() {
            let v1 = vertices_list.get(index).unwrap();
            let v2 = match vertices_list.get(index + 1) {
                Some(vertex) => vertex,
                None => vertices_list.get(0).unwrap(),
            };
            let new_edge = Rc::new(Edge::new(v1, v2));
            edge_list.push(new_edge);
        }

        return edge_list;
    }
}

#[cfg(test)]
mod midpoint {
    use super::*;

    #[test]
    fn test_midpoint_calculation() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.2));
        
        let edge = Edge::new(&v1, &v2);
        let midpoint = edge.midpoint();
        assert_eq!(midpoint.x, 0.5);
        assert_eq!(midpoint.y, 0.6);
    }
}

#[cfg(test)]
mod equality {
    use super::*;

    #[test]
    fn test_different_objects() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.2));
        
        let e1 = Edge::new(&v1, &v2);
        let e2 = Edge::new(&v1, &v2);
        assert!(e1 == e2);

        let e1 = Rc::new(Edge::new(&v1, &v2));
        let e2 = Rc::new(Edge::new(&v1, &v2));
        assert!(e1 == e2);
    }
    
    #[test]
    fn test_half_edge() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.2));
        
        let e1 = Edge::new(&v1, &v2);
        let e2 = Edge::new(&v2, &v1);
        assert!(e1 != e2);
        
        let e1 = Rc::new(Edge::new(&v1, &v2));
        let e2 = Rc::new(Edge::new(&v2, &v1));
        assert!(e1 != e2);
    }
}