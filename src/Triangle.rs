use crate::Continence::*;
use crate::Orientation::*;
use crate::Vertex::*;
use std::cmp::Eq;
use std::collections::hash_map::DefaultHasher;
use std::hash::Hash;
use std::rc::Rc;

#[derive(Hash)]
pub struct Triangle {
    pub v1: Rc<Vertex>,
    pub v2: Rc<Vertex>,
    pub v3: Rc<Vertex>,
}

impl PartialEq for Triangle {
    fn eq(&self, other: &Self) -> bool {
        let mut hasher = DefaultHasher::new();
        self.hash(&mut hasher) == other.hash(&mut hasher)
    }
}

impl Eq for Triangle {}

impl Triangle {
    pub fn new(v1: &Rc<Vertex>, v2: &Rc<Vertex>, v3: &Rc<Vertex>) -> Triangle {
        Triangle {
            v1: Rc::clone(&v1),
            v2: Rc::clone(&v2),
            v3: Rc::clone(&v3),
        }
    }

    pub fn is_ghost(&self) -> bool {
        /*
           Although, all vertices are inspected, only v3 is supposed to hold the ghost vertex.
           v1 and v2 are supposed to surround the convex hull in counterclockwise direction.
        */
        self.v1.is_ghost || self.v2.is_ghost || self.v3.is_ghost
    }

    pub fn encircles(&self, vertex: &Vertex) -> Continence {
        if !self.is_ghost() {
            /*
               v1,v2,v3 are supposed to match counterclockwise, when created.
            */
            return in_circle(&self.v1, &self.v2, &self.v3, vertex);
        } else {
            /*
               The set of ghost triangles surround the convex hull with solid edges
               in counterclockwise direction. If v1-v1-trial matches clockwise, then trial
               should be outside.
            */
            match orient_2d(&self.v1, &self.v2, &vertex) {
                Orientation::Clockwise => return Continence::Inside,
                _ => return Continence::Outside,
            }
        }
    }
}

#[cfg(test)]
mod constructor {
    use super::*;

    #[test]
    fn test_always_create_in_counterclockwise() {
        let v1 = Rc::new(Vertex::new(0.0, 1.0));
        let v2 = Rc::new(Vertex::new(2.0, 3.0));
        let v3 = Rc::new(Vertex::new(4.0, 7.0));

        let t1 = Triangle::new(&v1, &v2, &v3);

        assert_eq!(t1.v1.x, 0.0);
        assert_eq!(t1.v1.y, 1.0);
        assert_eq!(t1.v2.x, 2.0);
        assert_eq!(t1.v2.y, 3.0);
        assert_eq!(t1.v3.x, 4.0);
        assert_eq!(t1.v3.y, 7.0);
    }
}

#[cfg(test)]
mod ghost_triangle {
    use super::*;

    #[test]
    fn test_at_least_one_vertex_is_ghost() {
        let v1 = Rc::new(Vertex::new(0.0, 1.0));
        let v2 = Rc::new(Vertex::new(2.0, 3.0));
        let v3 = Rc::new(Vertex::new(4.0, 7.0));
        let ghost = Rc::new(Vertex::new_ghost());

        /* alternating ghost position */
        let t1 = Triangle::new(&v1, &v2, &ghost);
        assert!(t1.is_ghost());

        let t1 = Triangle::new(&v1, &ghost, &v2);
        assert!(t1.is_ghost());

        let t1 = Triangle::new(&ghost, &v1, &v2);
        assert!(t1.is_ghost());

        /* no ghost vertext => not a ghost triangle */
        let t1 = Triangle::new(&v1, &v2, &v3);
        assert!(!t1.is_ghost());
    }
}

#[cfg(test)]
mod structure {
    use super::*;

    #[test]
    fn test_two_triangles_with_same_vertices() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(2.0, 0.0));
        let v3 = Rc::new(Vertex::new(0.0, 2.0));
        let v4 = Rc::new(Vertex::new(2.0, 2.0));

        let t1 = Triangle::new(&v1, &v2, &v3);
        let t2 = Triangle::new(&v1, &v2, &v4);
    }
}

#[cfg(test)]
mod encircles {
    use super::*;

    #[test]
    fn test_triangle_in_circle_method() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 0.0));
        let v3 = Rc::new(Vertex::new(0.0, 1.0));
        let t1 = Triangle::new(&v1, &v2, &v3);

        let v4 = Rc::new(Vertex::new(0.3, 0.3));
        assert_eq!(t1.encircles(&v4), Continence::Inside);

        let v4 = Rc::new(Vertex::new(2.0, 2.0));
        assert_eq!(t1.encircles(&v4), Continence::Outside);

        let v4 = Rc::new(Vertex::new(1.0, 1.0));
        assert_eq!(t1.encircles(&v4), Continence::Boundary);
    }
}
