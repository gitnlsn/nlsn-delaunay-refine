use crate::vertex::*;
use nalgebra::Matrix3;

pub fn area(v1: &Vertex, v2: &Vertex, v3: &Vertex) -> f64 {
    let x1 = v1.x;
    let y1 = v1.y;

    let x2 = v2.x;
    let y2 = v2.y;
    
    let x3 = v3.x;
    let y3 = v3.y;

    let matrix = Matrix3::new(
        x1, y1, 1.0, 
        x2, y2, 1.0, 
        x3, y3, 1.0,
    );
    return matrix.determinant() / 2.0;
}

#[cfg(test)]
mod area {
    use super::*;

    #[test]
    fn test_area() {
        let v1 = Vertex::new(0.0, 0.0);
        let v2 = Vertex::new(1.0, 0.0);
        let v3 = Vertex::new(0.0, 1.0);
        assert_eq!(area(&v1, &v2, &v3), 0.5);
        assert_eq!(area(&v3, &v1, &v2), 0.5);
        assert_eq!(area(&v2, &v3, &v1), 0.5);
    }
}