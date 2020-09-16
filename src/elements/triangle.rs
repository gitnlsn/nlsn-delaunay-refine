use crate::elements::{edge::*, polyline::*, vertex::*};
use crate::properties::{area::*, circumcenter::*, continence::*, distance::*, orientation::*};

use std::cmp::Eq;
use std::fmt;
use std::fmt::Debug;
use std::hash::Hash;
use std::rc::Rc;

#[derive(Hash, Debug)]
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

    pub fn area(&self) -> Option<f64> {
        if self.is_ghost() {
            return None;
        }
        return Some(area_triangle(&self.v1, &self.v2, &self.v3));
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
                Orientation::Clockwise => return Continence::Outside,
                Orientation::Colinear => return Continence::Boundary,
            }
        }
    }

    /**
     * Determines the circumcenter.
     * Returns None, if ghost of colinear vertices.
     */
    pub fn circumcenter(&self) -> Option<Vertex> {
        if self.is_ghost() {
            return None;
        }
        return circumcenter(&self.v1, &self.v2, &self.v3);
    }

    pub fn quality(&self) -> Option<f64> {
        if self.is_ghost() {
            return None;
        }

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

        let area = self.area().unwrap();

        if a <= b && a <= c {
            return Some(b * c / (4.0 * area));
        } else if b <= c {
            return Some(a * c / (4.0 * area));
        } else {
            return Some(a * b / (4.0 * area));
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

    pub fn center(&self) -> Vertex {
        if self.is_ghost() {
            let center_x = (self.v1.x + self.v2.x) / 2.0;
            let center_y = (self.v1.y + self.v2.y) / 2.0;

            return Vertex::new(center_x, center_y);
        } else {
            let center_x = (self.v1.x + self.v2.x + self.v3.x) / 3.0;
            let center_y = (self.v1.y + self.v2.y + self.v3.y) / 3.0;

            return Vertex::new(center_x, center_y);
        }
    }

    pub fn opposite_vertex(&self, edge: &Rc<Edge>) -> Option<Rc<Vertex>> {
        let (e1, e2, e3) = self.inner_edges();
        if edge == &e1 {
            return Some(Rc::clone(&self.v3));
        } else if edge == &e2 {
            return Some(Rc::clone(&self.v1));
        } else if edge == &e3 {
            return Some(Rc::clone(&self.v2));
        } else {
            return None;
        }
    }

    pub fn opposite_edge(&self, vertex: &Rc<Vertex>) -> Option<Rc<Edge>> {
        if vertex == &self.v1 {
            return Some(Rc::new(Edge::new(&self.v2, &self.v3)));
        } else if vertex == &self.v2 {
            return Some(Rc::new(Edge::new(&self.v3, &self.v1)));
        } else if vertex == &self.v3 {
            return Some(Rc::new(Edge::new(&self.v1, &self.v2)));
        } else {
            return None;
        }
    }

    pub fn as_polyline(&self) -> Option<Polyline> {
        if self.is_ghost() {
            return None;
        }
        Some(
            Polyline::new_closed(vec![
                Rc::clone(&self.v1),
                Rc::clone(&self.v2),
                Rc::clone(&self.v3),
            ])
            .unwrap(),
        )
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

    #[test]
    fn exception_1() {
        let v1 = Rc::new(Vertex::new(4.0, 6.0));
        let v2 = Rc::new(Vertex::new(3.0, 3.0));
        let v3 = Rc::new(Vertex::new(5.0, 4.0));
        let t1 = Triangle::new(&v1, &v2, &v3);

        let v4 = Rc::new(Vertex::new(5.0, 5.0));
        assert_eq!(t1.encircles(&v4), Continence::Boundary);
    }
}

#[cfg(test)]
mod quality_ratio {
    use super::*;

    #[test]
    fn sample_1() {
        /*
           Test case: equilateral triangle
        */
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 0.0));
        let v3 = Rc::new(Vertex::new(0.5, 0.86602540378));

        let triangle = Triangle::new(&v1, &v2, &v3);
        let ratio = triangle.quality().unwrap();

        assert!((ratio - 0.5773502691903656).abs() < 0.00000001);
    }

    #[test]
    fn sample_2() {
        /*
           Test case: rectangle triangle
        */
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(0.5, 0.0));
        let v3 = Rc::new(Vertex::new(0.5, 0.28867513459481287));

        let triangle = Triangle::new(&v1, &v2, &v3);
        let ratio = triangle.quality().unwrap();

        assert!((ratio - 1.0).abs() < 0.00000001);
    }

    #[test]
    fn sample_3() {
        /*
           Test case: isosceles triangle
        */
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 0.0));
        let v3 = Rc::new(Vertex::new(1.0, 1.0));

        let triangle = Triangle::new(&v1, &v2, &v3);
        let ratio = triangle.quality().unwrap();

        assert!((ratio - 0.7071067811865476).abs() < 0.00000001);
    }

    #[test]
    fn sample_4() {
        /*
           Test case: skinny triangle
        */
        let p1 = Rc::new(Vertex::new(0.80, -0.5));
        let p2 = Rc::new(Vertex::new(0.95, -0.5));
        let p3 = Rc::new(Vertex::new(0.80, 0.5));

        let triangle = Triangle::new(&p1, &p2, &p3);
        let ratio = triangle.quality().unwrap();

        assert!((ratio - 3.370624736026116).abs() < 0.00000001);
    }

    #[test]
    fn sample_5() {
        /*
           Test case: circle discretization in 100 segments
        */

        fn get_circle_point(radius: f64, angle: f64, center: &Vertex) -> Vertex {
            let dx = radius * angle.cos();
            let dy = radius * angle.sin();
            return Vertex::new(center.x + dx, center.y + dy);
        }
        let dphi = std::f64::consts::PI * 2.0 / 100 as f64;

        let center = Rc::new(Vertex::new(0.0, 0.0));
        let p1 = Rc::new(get_circle_point(0.95, dphi * 0.0, &center));
        let p2 = Rc::new(get_circle_point(0.95, dphi * 1.0, &center));

        let triangle = Triangle::new(&p1, &p2, &center);
        let ratio = triangle.quality().unwrap();

        assert!((ratio - 7.962985554954328).abs() < 0.00000001);
    }
}

#[cfg(test)]
mod center {
    use super::*;

    #[test]
    fn sample_1() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 0.0));
        let v3 = Rc::new(Vertex::new(1.0, 1.0));

        let triangle = Triangle::new(&v1, &v2, &v3);
        let center = triangle.center();
        assert_eq!(center.x, 2.0 / 3.0);
        assert_eq!(center.y, 1.0 / 3.0);
    }
}

#[cfg(test)]
mod as_polyline {
    use super::*;

    #[test]
    fn none_if_ghost() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 0.0));
        // let v3 = Rc::new(Vertex::new(0.0, 1.0));
        let v_ghost = Rc::new(Vertex::new_ghost());

        let triangle = Triangle::new(&v1, &v2, &v_ghost);
        assert!(triangle.as_polyline().is_none());
    }

    #[test]
    fn polyline_if_solid() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 0.0));
        let v3 = Rc::new(Vertex::new(0.0, 1.0));
        // let v_ghost = Rc::new(Vertex::new_ghost());

        let triangle = Triangle::new(&v1, &v2, &v3);
        assert!(triangle.as_polyline().is_some());
        assert!(triangle.as_polyline().unwrap().vertices.contains(&v1));
        assert!(triangle.as_polyline().unwrap().vertices.contains(&v2));
        assert!(triangle.as_polyline().unwrap().vertices.contains(&v3));
    }
} /* end - as_polyline tests */
