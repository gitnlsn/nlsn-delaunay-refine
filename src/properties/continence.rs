extern crate nalgebra;

use crate::elements::vertex::*;

use nalgebra::Matrix4;

#[derive(PartialEq, Debug)]
pub enum Continence {
    Inside,
    Outside,
    Boundary,
}

#[derive(PartialEq, Debug)]
pub enum BoundaryInclusion {
    Open,
    Closed,
}

/**
 * Checks whether Vertex d is contained by the circumcircle defined by triangle(a,b,c).
 * Vertices a, b and c must be in counterclockwise order.
 */
pub fn continence(a: &Vertex, b: &Vertex, c: &Vertex, d: &Vertex) -> Continence {
    let matrix = Matrix4::new(
        a.x, a.y, a.x.powi(2) + a.y.powi(2), 1.0,
        b.x, b.y, b.x.powi(2) + b.y.powi(2), 1.0,
        c.x, c.y, c.x.powi(2) + c.y.powi(2), 1.0,
        d.x, d.y, d.x.powi(2) + d.y.powi(2), 1.0,
    );
    let det = matrix.determinant();

    if float_cmp::approx_eq!(f64, det, 0.0, epsilon = 1.0E-14f64) {
        return Continence::Boundary;
    }

    if det > 0.0 {
        return Continence::Inside;
    } else {
        return Continence::Outside;
    }
}


#[cfg(test)]
mod in_circle {
    use super::*;

    #[test]
    fn test_continence_inside() {
        let p1 = Vertex::new(0.0, 0.0);
        let p2 = Vertex::new(1.0, 0.0);
        let p3 = Vertex::new(1.0, 1.0);
        let p4 = Vertex::new(0.6, 0.5);
        assert_eq!(continence(&p1, &p2, &p3, &p4), Continence::Inside);
    }
    
    #[test]
    fn test_continence_outside() {
        let p1 = Vertex::new(0.0, 0.0);
        let p2 = Vertex::new(1.0, 0.0);
        let p3 = Vertex::new(1.0, 1.0);
        let p4 = Vertex::new(0.0, 2.0);
        assert_eq!(continence(&p1, &p2, &p3, &p4), Continence::Outside);
    }
    
    #[test]
    fn test_continence_boundary() {
        let p1 = Vertex::new(0.0, 0.0);
        let p2 = Vertex::new(1.0, 0.0);
        let p3 = Vertex::new(1.0, 1.0);
        let p4 = Vertex::new(0.0, 1.0);
        assert_eq!(continence(&p1, &p2, &p3, &p4), Continence::Boundary);
    }
}
