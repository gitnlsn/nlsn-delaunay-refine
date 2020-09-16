#![macro_use]
extern crate float_cmp;

extern crate nalgebra;

use crate::elements::bounding_box::*;
use crate::elements::vertex::*;

use nalgebra::{Matrix2, Matrix2x1};
use std::rc::Rc;

/**
 * Determines the intersection between two line segments
 *  - v1 & v2 determines the first line segment
 *  - v3 & v4 determines the second line segment
 *  - returns None if there is no intersection
 */
pub fn intersection(
    v1: &Rc<Vertex>,
    v2: &Rc<Vertex>,
    v3: &Rc<Vertex>,
    v4: &Rc<Vertex>,
) -> Option<Vertex> {
    if let Some(bbox) = intersection_region(v1, v2, v3, v4) {
        if let Some(vertex) = intersection_vertex(v1, v2, v3, v4, &bbox) {
            let in_interval_x = (vertex.x >= bbox.origin.x && vertex.x <= bbox.destin.x)
                || float_cmp::approx_eq!(f64, bbox.origin.x, vertex.x, epsilon = 1.0E-14f64)
                || float_cmp::approx_eq!(f64, bbox.destin.x, vertex.x, epsilon = 1.0E-14f64);
            let in_interval_y = (vertex.y >= bbox.origin.y && vertex.y <= bbox.destin.y)
                || float_cmp::approx_eq!(f64, bbox.origin.y, vertex.y, epsilon = 1.0E-14f64)
                || float_cmp::approx_eq!(f64, bbox.destin.y, vertex.y, epsilon = 1.0E-14f64);

            if in_interval_x && in_interval_y {
                return Some(vertex);
            }
        }
    }

    return None;
}

/**
 * Determines the possible region where a intersection may occur
 */
fn intersection_region(
    v1: &Rc<Vertex>,
    v2: &Rc<Vertex>,
    v3: &Rc<Vertex>,
    v4: &Rc<Vertex>,
) -> Option<BoundingBox> {
    let e1_vertices: Vec<Rc<Vertex>> = vec![Rc::clone(v1), Rc::clone(v2)];
    let e2_vertices: Vec<Rc<Vertex>> = vec![Rc::clone(v3), Rc::clone(v4)];

    let e1_bbox: BoundingBox = BoundingBox::from_vertices(e1_vertices).unwrap();
    let e2_bbox: BoundingBox = BoundingBox::from_vertices(e2_vertices).unwrap();

    return BoundingBox::intersection(&e1_bbox, &e2_bbox);
}

/**
 * Determines the exact intersection vertex between lines
 */
fn intersection_vertex(
    v1: &Rc<Vertex>,
    v2: &Rc<Vertex>,
    v3: &Rc<Vertex>,
    v4: &Rc<Vertex>,
    bbox: &BoundingBox,
) -> Option<Vertex> {
    let x1 = v1.x;
    let y1 = v1.y;

    let x2 = v2.x;
    let y2 = v2.y;

    let x3 = v3.x;
    let y3 = v3.y;

    let x4 = v4.x;
    let y4 = v4.y;

    let matrix_a = Matrix2::new(-(y2 - y1), x2 - x1, -(y4 - y3), x4 - x3);

    let matrix_b = Matrix2x1::new(
        y1 * (x2 - x1) - x1 * (y2 - y1),
        y3 * (x4 - x3) - x3 * (y4 - y3),
    );

    if !matrix_a.is_invertible() {
        /* Lines are coincident */
        let possible_middle_point = Matrix2x1::new(
            (bbox.origin.x + bbox.destin.x) / 2.0,
            (bbox.origin.y + bbox.destin.y) / 2.0,
        );

        let eval = matrix_a * possible_middle_point - matrix_b;

        if float_cmp::approx_eq!(f64, eval[0], 0.0, epsilon = 1.0E-14f64)
            && float_cmp::approx_eq!(f64, eval[1], 0.0, epsilon = 1.0E-14f64)
        {
            /* Return mid-point as intersection representation */
            return Some(Vertex::new(
                possible_middle_point[0],
                possible_middle_point[1],
            ));
        }

        /* Lines are parallel */
        return None;
    }

    let matrix_a_inv = matrix_a.try_inverse().unwrap();

    let intersection_matrix = matrix_a_inv * matrix_b;

    let intersection_x = intersection_matrix[0];
    let intersection_y = intersection_matrix[1];

    return Some(Vertex::new(intersection_x, intersection_y));
}

#[cfg(test)]
mod intersection {
    use super::*;

    #[test]
    fn test_intersection() {
        /* 1st assertion */
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(2.0, 2.0));
        let v3 = Rc::new(Vertex::new(2.0, 0.0));
        let v4 = Rc::new(Vertex::new(0.0, 2.0));

        let vertex = intersection(&v1, &v2, &v3, &v4).unwrap();

        assert_eq!(vertex.x, 1.0);
        assert_eq!(vertex.y, 1.0);

        /* 2nd assertion */
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(2.0, 0.0));
        let v3 = Rc::new(Vertex::new(2.0, 2.0));
        let v4 = Rc::new(Vertex::new(0.0, 2.0));

        let possible_intersection = intersection(&v1, &v2, &v3, &v4);

        assert!(possible_intersection.is_none());

        /* 3rd assertion */
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.0));
        let v3 = Rc::new(Vertex::new(0.0, 1.0));
        let v4 = Rc::new(Vertex::new(0.2, 0.8));

        let possible_intersection = intersection(&v1, &v2, &v3, &v4);

        assert!(possible_intersection.is_none());

        /* 4th assertion */
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.0));
        let v3 = Rc::new(Vertex::new(0.0, 1.0));
        let v4 = Rc::new(Vertex::new(1.0, 0.7));

        let vertex = intersection(&v1, &v2, &v3, &v4).unwrap();

        assert_eq!(vertex.x, 0.7692307692307692);
        assert_eq!(vertex.y, 0.7692307692307692);
    }

    #[test]
    fn test_intersection_region() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.0));
        let v3 = Rc::new(Vertex::new(0.0, 1.0));
        let v4 = Rc::new(Vertex::new(1.0, 0.7));

        let region = intersection_region(&v1, &v2, &v3, &v4).unwrap();

        assert_eq!(region.origin.x, 0.0);
        assert_eq!(region.origin.y, 0.7);
        assert_eq!(region.destin.x, 1.0);
        assert_eq!(region.destin.y, 1.0);

        /* assert none */
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.0));
        let v3 = Rc::new(Vertex::new(0.0, 2.0));
        let v4 = Rc::new(Vertex::new(1.0, 1.7));

        let region = intersection_region(&v1, &v2, &v3, &v4);

        assert!(region.is_none());
    }

    #[test]
    fn exception_case_1() {
        let v1 = Rc::new(Vertex::new(2.0, 1.0));
        let v2 = Rc::new(Vertex::new(1.0, 2.0));
        let v3 = Rc::new(Vertex::new(2.0, 2.0));
        let v4 = Rc::new(Vertex::new(3.0, 1.0));

        assert_eq!(intersection(&v1, &v2, &v3, &v4), None);
    }

    #[test]
    fn exception_case_2() {
        /*
            Improper handle to float point eq comparison lead to miss interpretation 
            on intersection point when intersection bounding box resulted in skinny
            bounding box
        */
        let v1 = Rc::new(Vertex::new(-0.43357830669154374, -0.7886760120394772));
        let v2 = Rc::new(Vertex::new(-0.38320136240856495, -0.8143443472194178));
        let v3 = Rc::new(Vertex::new(-0.4, 0.95));
        let v4 = Rc::new(Vertex::new(-0.4, -0.95));

        assert!(intersection(&v1, &v2, &v3, &v4).is_some());

        let v1 = Rc::new(Vertex::new(-0.38320136240856545, 0.8143443472194175));
        let v2 = Rc::new(Vertex::new(-0.4335783066915439, 0.7886760120394771));
        let v3 = Rc::new(Vertex::new(-0.4, 0.95));
        let v4 = Rc::new(Vertex::new(-0.4, -0.95));
        assert!(intersection(&v1, &v2, &v3, &v4).is_some());
    }
}
