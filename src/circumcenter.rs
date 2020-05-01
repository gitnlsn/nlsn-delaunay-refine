use crate::vertex::*;
use nalgebra::{Matrix2, Matrix2x1};

pub fn circumcenter(v1: &Vertex, v2: &Vertex, v3: &Vertex) -> Vertex {
    /*
        Let (x1,y1), (x2,y2), (x3,y3) be the vertices of a triangle.self
            then (xc,yc) is the circumcenter, if it exists:

        [xc] =  1/2  * [x3^2 - x1^2 + y3^2 - y1^2] * [x3-x1 y3-y1] ^ -1
        [yc] =         [x2^2 - x1^2 + y2^2 - y1^2] * [x2-x1 y2-y1]
    */

    let x1 = v1.x;
    let y1 = v1.y;

    let x2 = v2.x;
    let y2 = v2.y;

    let x3 = v3.x;
    let y3 = v3.y;

    let matrix_a = Matrix2::new(x3 - x1, y3 - y1, x2 - x1, y2 - y1);

    if !matrix_a.is_invertible() {
        panic!("Triangle has no circumcircle. Vertices might be colinear.");
    }

    let matrix_a_inv = matrix_a.try_inverse().unwrap();

    let matrix_b = Matrix2x1::new(
        x3.powi(2) - x1.powi(2) + y3.powi(2) - y1.powi(2),
        x2.powi(2) - x1.powi(2) + y2.powi(2) - y1.powi(2),
    );

    let center_matrix = 0.5 * matrix_a_inv * matrix_b;

    let xc = center_matrix[0];
    let yc = center_matrix[1];

    return Vertex::new(xc, yc);
}

#[cfg(test)]
mod circumcenter {
    use super::*;

    #[test]
    fn test_vertices_order() {
        let v1 = Vertex::new(0.0, 0.0);
        let v2 = Vertex::new(1.0, 0.0);
        let v3 = Vertex::new(1.0, 1.0);

        let c = circumcenter(&v1, &v2, &v3);
        assert_eq!(c.x, 0.5);
        assert_eq!(c.y, 0.5);

        let c = circumcenter(&v2, &v3, &v1);
        assert_eq!(c.x, 0.5);
        assert_eq!(c.y, 0.5);

        let c = circumcenter(&v3, &v1, &v2);
        assert_eq!(c.x, 0.5);
        assert_eq!(c.y, 0.5);

        let c = circumcenter(&v1, &v3, &v2);
        assert_eq!(c.x, 0.5);
        assert_eq!(c.y, 0.5);
    }

    #[test]
    fn test_equilateral() {
        let v1 = Vertex::new(0.0, 0.0);
        let v2 = Vertex::new(1.0, 0.0);
        let v3 = Vertex::new(0.5, 0.86602540378);

        let c = circumcenter(&v1, &v2, &v3);
        assert!((c.x - 0.5).abs() < 0.00000001);
        assert!((c.y - 0.28867513459).abs() < 0.00000001);
    }

    #[test]
    #[should_panic]
    fn test_panics_if_colinear() {
        let v1 = Vertex::new(0.0, 0.0);
        let v2 = Vertex::new(1.0, 0.0);
        let v3 = Vertex::new(0.5, 0.0);

        circumcenter(&v1, &v2, &v3);
    }
}