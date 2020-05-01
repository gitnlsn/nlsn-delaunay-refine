use crate::properties::{
    area::*,
    continence::*,
    distance::*,
    circumcenter::*,
    orientation::*,
};
use crate::elements::{
    edge::*,
    vertex::*,
};

use std::cmp::Eq;
use std::fmt;
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
        self.v1 == other.v1 && self.v2 == other.v2 && self.v3 == other.v3
            || self.v1 == other.v2 && self.v2 == other.v3 && self.v3 == other.v1
            || self.v1 == other.v3 && self.v2 == other.v1 && self.v3 == other.v2
    }
}

impl Eq for Triangle {}

impl fmt::Display for Triangle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        return write!(f, "({} - {} - {})", self.v1, self.v2, self.v3);
    }
}

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
           v1 and v2 are supposed to surround the convex hull in clockwise direction and ghost
           vertex is always outside.
        */
        self.v1.is_ghost || self.v2.is_ghost || self.v3.is_ghost
    }

    pub fn area(&self) -> f64 {
        if self.is_ghost() {
            return 0.0;
        }
        return area(&self.v1, &self.v2, &self.v3);
    }

    pub fn encircles(&self, vertex: &Vertex) -> Continence {
        if !self.is_ghost() {
            /*
               v1, v2, v3 are supposed to match counterclockwise, when created.
            */
            return continence(&self.v1, &self.v2, &self.v3, vertex);
        } else {
            /*
               The set of ghost triangles surround the convex hull with solid edges
               in counterclockwise direction. The first two vertices have the outer
               space in counterclockwise direction, as the ghost is always outside.
            */
            match orientation(&self.v1, &self.v2, &vertex) {
                Orientation::Counterclockwise => return Continence::Inside,
                _ => return Continence::Outside,
            }
        }
    }

    pub fn circumcenter(&self) -> Rc<Vertex> {
        return Rc::new(circumcenter(&self.v1, &self.v2, &self.v3));
    }

    pub fn quality_ratio(&self) -> f64 {
        /*
            Let a,b,c be the sides of a triangle, and A its area.
            Then radius is given by:

                R = a*b*c / (4*A)

            thus, radius-edge ration may be evaluated by:

                ratio = R / l_min = rem(a,b,c \ l_min) / (4*A)
        */
        let a = distance(&self.v1, &self.v2);
        let b = distance(&self.v2, &self.v3);
        let c = distance(&self.v3, &self.v1);

        let area = self.area();

        if a <= b && a <= c {
            return b * c / (4.0 * area);
        } else if b <= c {
            return a * c / (4.0 * area);
        } else {
            return a * b / (4.0 * area);
        }
    }

    pub fn inner_edges(&self) -> (Rc<Edge>, Rc<Edge>, Rc<Edge>) {
        let e1 = Rc::new(Edge::new(&self.v1, &self.v2));
        let e2 = Rc::new(Edge::new(&self.v2, &self.v3));
        let e3 = Rc::new(Edge::new(&self.v3, &self.v1));

        return (e1, e2, e3);
    }

    pub fn outer_edges(&self) -> (Rc<Edge>, Rc<Edge>, Rc<Edge>) {
        let e1 = Rc::new(Edge::new(&self.v2, &self.v1));
        let e2 = Rc::new(Edge::new(&self.v3, &self.v2));
        let e3 = Rc::new(Edge::new(&self.v1, &self.v3));

        return (e1, e2, e3);
    }
}

#[cfg(test)]
mod constructor {
    use super::*;

    #[test]
    fn test_new_triangle() {
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

#[cfg(test)]
mod quality_ratio {
    use super::*;

    #[test]
    fn test_quality_equilateral() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 0.0));
        let v3 = Rc::new(Vertex::new(0.5, 0.86602540378));

        let triangle = Triangle::new(&v1, &v2, &v3);
        let ratio = triangle.quality_ratio();

        assert!((ratio - 0.5773502691903656).abs() < 0.00000001);
    }

    #[test]
    fn test_quality_isosceles_rectangle() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 0.0));
        let v3 = Rc::new(Vertex::new(1.0, 1.0));

        let triangle = Triangle::new(&v1, &v2, &v3);
        let ratio = triangle.quality_ratio();

        assert!((ratio - 0.7071067811865476).abs() < 0.00000001);
    }
}