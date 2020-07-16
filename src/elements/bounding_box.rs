use crate::elements::edge::*;
use crate::elements::triangle::*;
use crate::elements::vertex::*;

use std::rc::Rc;

pub struct BoundingBox {
    pub origin: Rc<Vertex>,
    pub destin: Rc<Vertex>,
}

impl BoundingBox {
    pub fn from_vertices(vertices_list: Vec<Rc<Vertex>>) -> Option<Self> {
        if vertices_list.is_empty() {
            return None;
        }

        let mut lower_x: f64 = vertices_list.get(0).unwrap().x;
        let mut upper_x: f64 = vertices_list.get(0).unwrap().x;
        let mut lower_y: f64 = vertices_list.get(0).unwrap().y;
        let mut upper_y: f64 = vertices_list.get(0).unwrap().y;

        for vertex in vertices_list.iter() {
            lower_x = min(lower_x, vertex.x);
            upper_x = max(upper_x, vertex.x);
            lower_y = min(lower_y, vertex.y);
            upper_y = max(upper_y, vertex.y);
        }

        if lower_x == upper_x && lower_y == upper_y {
            return None;
        }

        return Some(Self {
            origin: Rc::new(Vertex::new(lower_x, lower_y)),
            destin: Rc::new(Vertex::new(upper_x, upper_y)),
        });
    }

    pub fn from_edge(edge: &Edge) -> Self {
        return Self::from_vertices(vec![Rc::clone(&edge.v1), Rc::clone(&edge.v2)]).unwrap();
    }

    pub fn from_triangle(triangle: &Triangle) -> Self {
        return Self::from_vertices(vec![
            Rc::clone(&triangle.v1),
            Rc::clone(&triangle.v2),
            Rc::clone(&triangle.v3),
        ])
        .unwrap();
    }

    pub fn intersection(b1: &Self, b2: &Self) -> Option<Self> {
        let lower_x: f64 = max(b1.origin.x, b2.origin.x);
        let upper_x: f64 = min(b1.destin.x, b2.destin.x);
        let lower_y: f64 = max(b1.origin.y, b2.origin.y);
        let upper_y: f64 = min(b1.destin.y, b2.destin.y);

        let x_intersection: bool = lower_x <= upper_x;
        let y_intersection: bool = lower_y <= upper_y;

        if x_intersection && y_intersection {
            return Some(Self {
                origin: Rc::new(Vertex::new(lower_x, lower_y)),
                destin: Rc::new(Vertex::new(upper_x, upper_y)),
            });
        }

        return None;
    }

    pub fn inclusion(b1: &BoundingBox, b2: BoundingBox) -> Self {
        let lower_x: f64 = min(b1.origin.x, b2.origin.x);
        let upper_x: f64 = max(b1.destin.x, b2.destin.x);
        let lower_y: f64 = min(b1.origin.y, b2.origin.y);
        let upper_y: f64 = max(b1.destin.y, b2.destin.y);

        return Self {
            origin: Rc::new(Vertex::new(lower_x, lower_y)),
            destin: Rc::new(Vertex::new(upper_x, upper_y)),
        };
    }

    pub fn inclusion_list(box_list: Vec<BoundingBox>) -> Option<Self> {
        if box_list.is_empty() {
            return None;
        }

        let mut lower_x: f64 = box_list.get(0).unwrap().origin.x;
        let mut upper_x: f64 = box_list.get(0).unwrap().destin.x;
        let mut lower_y: f64 = box_list.get(0).unwrap().origin.y;
        let mut upper_y: f64 = box_list.get(0).unwrap().destin.y;

        for bbox in box_list.iter() {
            lower_x = min(lower_x, bbox.origin.x);
            upper_x = max(upper_x, bbox.destin.x);
            lower_y = min(lower_y, bbox.origin.y);
            upper_y = max(upper_y, bbox.destin.y);
        }

        return Some(Self {
            origin: Rc::new(Vertex::new(lower_x, lower_y)),
            destin: Rc::new(Vertex::new(upper_x, upper_y)),
        });
    }
}

fn min(f1: f64, f2: f64) -> f64 {
    if f1 < f2 {
        return f1;
    }
    return f2;
}

fn max(f1: f64, f2: f64) -> f64 {
    if f1 > f2 {
        return f1;
    }
    return f2;
}

#[cfg(test)]
mod intersection {
    use super::*;

    #[test]
    fn test_from_vertices() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(3.0, 4.0));
        let v3 = Rc::new(Vertex::new(1.0, 2.0));
        let v4 = Rc::new(Vertex::new(5.0, 6.0));

        assert!(BoundingBox::from_vertices(vec![]).is_none());
        assert!(BoundingBox::from_vertices(vec![Rc::clone(&v1),]).is_none());
        assert!(BoundingBox::from_vertices(vec![
            Rc::clone(&v1),
            Rc::clone(&v1),
            Rc::clone(&v1),
            Rc::clone(&v1)
        ])
        .is_none());

        let bbox = BoundingBox::from_vertices(vec![v1, v2, v3, v4]).unwrap();

        assert_eq!(bbox.origin.x, 0.0);
        assert_eq!(bbox.origin.y, 0.0);
        assert_eq!(bbox.destin.x, 5.0);
        assert_eq!(bbox.destin.y, 6.0);
    }

    #[test]
    fn test_from_edge() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(3.0, 4.0));

        let bbox = BoundingBox::from_edge(&Edge::new(&v1, &v2));

        assert_eq!(bbox.origin.x, 0.0);
        assert_eq!(bbox.origin.y, 0.0);
        assert_eq!(bbox.destin.x, 3.0);
        assert_eq!(bbox.destin.y, 4.0);

        /* now invert vertex order */
        let bbox = BoundingBox::from_edge(&Edge::new(&v2, &v1));

        assert_eq!(bbox.origin.x, 0.0);
        assert_eq!(bbox.origin.y, 0.0);
        assert_eq!(bbox.destin.x, 3.0);
        assert_eq!(bbox.destin.y, 4.0);
    }

    #[test]
    fn test_intersection() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(3.0, 4.0));
        let v3 = Rc::new(Vertex::new(1.0, 2.0));
        let v4 = Rc::new(Vertex::new(5.0, 6.0));

        let b1 = BoundingBox::from_vertices(vec![v1, v2]).unwrap();
        let b2 = BoundingBox::from_vertices(vec![v3, v4]).unwrap();

        let intersection_bbox = BoundingBox::intersection(&b1, &b2).unwrap();

        assert_eq!(intersection_bbox.origin.x, 1.0);
        assert_eq!(intersection_bbox.origin.y, 2.0);
        assert_eq!(intersection_bbox.destin.x, 3.0);
        assert_eq!(intersection_bbox.destin.y, 4.0);
    }

    #[test]
    fn test_inclusion() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(3.0, 4.0));
        let v3 = Rc::new(Vertex::new(1.0, 2.0));
        let v4 = Rc::new(Vertex::new(5.0, 6.0));

        let b1 = BoundingBox::from_vertices(vec![v1, v2]).unwrap();
        let b2 = BoundingBox::from_vertices(vec![v3, v4]).unwrap();

        let bbox: BoundingBox = BoundingBox::inclusion_list(vec![b1, b2]).unwrap();

        assert_eq!(bbox.origin.x, 0.0);
        assert_eq!(bbox.origin.y, 0.0);
        assert_eq!(bbox.destin.x, 5.0);
        assert_eq!(bbox.destin.y, 6.0);
    }
}
