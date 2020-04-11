extern crate nalgebra;
use crate::vertex::*;

use nalgebra::Matrix3;

#[derive(PartialEq, Debug)]
pub enum Orientation {
    Counterclockwise,
    Clockwise,
    Colinear,
}

/**
 * Checks whether Vertices a, b and c are in counterclockwise order, 
 * in the circumcircle they define.
 */
pub fn orient_2d(a: &Vertex, b: &Vertex, c: &Vertex) -> Orientation {
    let matrix = Matrix3::new(
        a.x, a.y, 1.0, 
        b.x, b.y, 1.0, 
        c.x, c.y, 1.0,
    );
    let det = matrix.determinant();
    
    if det > 0.0 {
        return Orientation::Counterclockwise;
    } else if det < 0.0 {
        return Orientation::Clockwise;
    } else {
        return Orientation::Colinear;
    }
}

#[cfg(test)]
mod orient_2d {
    use super::*;

    #[test]
    fn test_counterclockwise() {
        let p1 = Vertex::new(0, 0.0, 0.0);
        let p2 = Vertex::new(1, 1.0, 0.0);
        let p3 = Vertex::new(2, 0.0, 1.0);
        assert_eq!(orient_2d(&p1, &p2, &p3), Orientation::Counterclockwise);
    }
    
    #[test]
    fn test_clockwise() {
        let p1 = Vertex::new(0, 0.0, 0.0);
        let p2 = Vertex::new(1, 0.0, 1.0);
        let p3 = Vertex::new(2, 1.0, 0.0);
        assert_eq!(orient_2d(&p1, &p2, &p3), Orientation::Clockwise);
    }
    
    #[test]
    fn test_colinear() {
        let p1 = Vertex::new(0, 0.0, 0.0);
        let p2 = Vertex::new(1, 1.0, 1.0);
        let p3 = Vertex::new(2, 2.0, 2.0);
        assert_eq!(orient_2d(&p1, &p2, &p3), Orientation::Colinear);
    }
}
