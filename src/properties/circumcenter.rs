use crate::elements::vertex::*;

pub fn circumcenter(v1: &Vertex, v2: &Vertex, v3: &Vertex) -> Option<Vertex> {
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

    // Coefficient matrix A = [[a11 a12], [a21 a22]], singular exactly when the
    // triangle is degenerate (the three vertices are collinear).
    let a11 = x3 - x1;
    let a12 = y3 - y1;
    let a21 = x2 - x1;
    let a22 = y2 - y1;

    let determinant = a11 * a22 - a12 * a21;

    if determinant == 0.0 {
        return None;
    }

    let b1 = x3.powi(2) - x1.powi(2) + y3.powi(2) - y1.powi(2);
    let b2 = x2.powi(2) - x1.powi(2) + y2.powi(2) - y1.powi(2);

    // Cramer's rule: A^-1 * b = adj(A) * b / det(A), with the single division
    // applied last so the solve keeps one rounding step per component instead of
    // rounding the inverse entries first.
    let xc = 0.5 * (a22 * b1 - a12 * b2) / determinant;
    let yc = 0.5 * (a11 * b2 - a21 * b1) / determinant;

    return Some(Vertex::new(xc, yc));
}

#[cfg(test)]
mod circumcenter {
    use super::*;

    #[test]
    fn test_vertices_order() {
        let v1 = Vertex::new(0.0, 0.0);
        let v2 = Vertex::new(1.0, 0.0);
        let v3 = Vertex::new(1.0, 1.0);

        let c = circumcenter(&v1, &v2, &v3).unwrap();
        assert_eq!(c.x, 0.5);
        assert_eq!(c.y, 0.5);

        let c = circumcenter(&v2, &v3, &v1).unwrap();
        assert_eq!(c.x, 0.5);
        assert_eq!(c.y, 0.5);

        let c = circumcenter(&v3, &v1, &v2).unwrap();
        assert_eq!(c.x, 0.5);
        assert_eq!(c.y, 0.5);

        let c = circumcenter(&v1, &v3, &v2).unwrap();
        assert_eq!(c.x, 0.5);
        assert_eq!(c.y, 0.5);
    }

    #[test]
    fn test_equilateral() {
        let v1 = Vertex::new(0.0, 0.0);
        let v2 = Vertex::new(1.0, 0.0);
        let v3 = Vertex::new(0.5, 0.86602540378);

        let c = circumcenter(&v1, &v2, &v3).unwrap();
        assert!((c.x - 0.5).abs() < 0.00000001);
        assert!((c.y - 0.28867513459).abs() < 0.00000001);
    }

    #[test]
    fn none_if_colinear_points() {
        let v1 = Vertex::new(0.0, 0.0);
        let v2 = Vertex::new(1.0, 0.0);
        let v3 = Vertex::new(0.5, 0.0);

        assert!(circumcenter(&v1, &v2, &v3).is_none());
    }
}
