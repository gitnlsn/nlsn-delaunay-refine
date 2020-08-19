use crate::elements::{edge::*, triangle::*, vertex::*};

use std::collections::{HashMap, HashSet};

use std::fmt;
use std::rc::Rc;

pub struct Triangulation {
    pub triangles: HashSet<Rc<Triangle>>,
    pub adjacency: HashMap<Rc<Edge>, Rc<Triangle>>,
}

impl fmt::Display for Triangulation {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "Triangles\n");
        for triangle in self.triangles.iter() {
            write!(formatter, "{}\n", triangle);
        }
        write!(formatter, "\n");

        write!(formatter, "Adjacency\n");
        for (edge, triangle) in self.adjacency.iter() {
            write!(formatter, "{} {}\n", edge, triangle);
        }

        return write!(formatter, "");
    }
}

impl Triangulation {
    pub fn new() -> Self {
        Self {
            triangles: HashSet::new(),
            adjacency: HashMap::new(),
        }
    }

    pub fn include_triangle(&mut self, triangle: &Rc<Triangle>) -> bool {
        if self.triangles.contains(triangle) {
            return false;
        }
        let (e12, e23, e31) = triangle.inner_edges();
        self.adjacency.insert(e12, Rc::clone(triangle));
        self.adjacency.insert(e23, Rc::clone(triangle));
        self.adjacency.insert(e31, Rc::clone(triangle));
        return self.triangles.insert(Rc::clone(triangle));
    }

    pub fn remove_triangle(&mut self, triangle: &Rc<Triangle>) -> bool {
        if !self.triangles.contains(triangle) {
            return false;
        }
        let (e12, e23, e31) = triangle.inner_edges();
        self.adjacency.remove(&e12);
        self.adjacency.remove(&e23);
        self.adjacency.remove(&e31);
        return self.triangles.remove(triangle);
    }

    pub fn vertices(&self) -> HashSet<Rc<Vertex>> {
        self.triangles
            .iter()
            .filter(|triangle| !triangle.is_ghost())
            .map(|t| vec![Rc::clone(&t.v1), Rc::clone(&t.v2), Rc::clone(&t.v3)])
            .flatten()
            .collect::<HashSet<Rc<Vertex>>>()
    }

    pub fn edges(&self) -> HashSet<Rc<Edge>> {
        self.triangles
            .iter()
            .map(|t| {
                let (e1,e2,e3) = t.inner_edges();
                return vec![
                    Rc::clone(&e1),
                    Rc::clone(&e2),
                    Rc::clone(&e3),
                ];
            })
            .flatten()
            .collect::<HashSet<Rc<Edge>>>()
    }
}

#[cfg(test)]
mod vertices {
    use super::*;

    #[test]
    fn sample_1() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 0.0));
        let v3 = Rc::new(Vertex::new(1.0, 1.0));
        let v4 = Rc::new(Vertex::new(0.0, 1.0));

        let t1 = Rc::new(Triangle::new(&v1, &v2, &v3));
        let t2 = Rc::new(Triangle::new(&v2, &v3, &v4));

        let mut triangulation = Triangulation::new();

        triangulation.include_triangle(&t1);
        triangulation.include_triangle(&t2);

        let vertices = triangulation.vertices();

        assert!(vertices.contains(&v1));
        assert!(vertices.contains(&v2));
        assert!(vertices.contains(&v3));
        assert!(vertices.contains(&v4));
    }
}

#[cfg(test)]
mod edges {
    use super::*;

    #[test]
    fn sample_1() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 0.0));
        let v3 = Rc::new(Vertex::new(1.0, 1.0));
        let v4 = Rc::new(Vertex::new(0.0, 1.0));

        let e12 = Rc::new(Edge::new(&v1, &v2));
        let e13 = Rc::new(Edge::new(&v1, &v3));
        let e41 = Rc::new(Edge::new(&v4, &v1));
        let e23 = Rc::new(Edge::new(&v2, &v3));
        let e34 = Rc::new(Edge::new(&v3, &v4));

        let t1 = Rc::new(Triangle::new(&v1, &v2, &v3));
        let t2 = Rc::new(Triangle::new(&v1, &v3, &v4));

        let mut triangulation = Triangulation::new();

        triangulation.include_triangle(&t1);
        triangulation.include_triangle(&t2);

        let edges = triangulation.edges();

        assert!(edges.contains(&e12));
        assert!(edges.contains(&e13));
        assert!(edges.contains(&e41));
        assert!(edges.contains(&e23));
        assert!(edges.contains(&e34));
    }
}
