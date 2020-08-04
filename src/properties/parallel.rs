extern crate nalgebra;

use crate::elements::vertex::*;

use nalgebra::Matrix2;

/**
 * Checks if ab is parallel to cd
 */
pub fn parallel(a: &Vertex, b: &Vertex, c: &Vertex, d: &Vertex) -> bool {
    let x1 = a.x;
    let y1 = a.y;

    let x2 = b.x;
    let y2 = b.y;

    let x3 = c.x;
    let y3 = c.y;

    let x4 = d.x;
    let y4 = d.y;

    let matrix_a = Matrix2::new(-(y2 - y1), x2 - x1, -(y4 - y3), x4 - x3);
    return !matrix_a.is_invertible();
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn sample_1() {
        let v1 = Vertex::new(0.0, 0.0);
        let v2 = Vertex::new(2.0, 2.0);
        let v3 = Vertex::new(2.0, 0.0);
        let v4 = Vertex::new(0.0, 2.0);
        assert!(!parallel(&v1, &v2, &v3, &v4));
        
        let v1 = Vertex::new(0.0, 0.0);
        let v2 = Vertex::new(2.0, 2.0);
        let v3 = Vertex::new(0.0, 1.0);
        let v4 = Vertex::new(2.0, 3.0);
        assert!(parallel(&v1, &v2, &v3, &v4));
    }
}