use crate::elements::vertex::*;
use nalgebra::Matrix3;
use std::rc::Rc;

pub fn area_segments(segments_list: Vec<(Rc<Vertex>, Rc<Vertex>)>) -> f64 {
    return segments_list.iter().fold(0.0, |acc, (v1, v2)| {
        acc + (v2.x - v1.x) * (v2.y + v1.y) / 2.0
    });
}

pub fn area_triangle(v1: &Vertex, v2: &Vertex, v3: &Vertex) -> f64 {
    let x1 = v1.x;
    let y1 = v1.y;

    let x2 = v2.x;
    let y2 = v2.y;

    let x3 = v3.x;
    let y3 = v3.y;

    let matrix = Matrix3::new(x1, y1, 1.0, x2, y2, 1.0, x3, y3, 1.0);
    return matrix.determinant() / 2.0;
}

#[cfg(test)]
mod area {
    use super::*;

    #[test]
    fn test_area_triangle() {
        let v1 = Vertex::new(0.0, 0.0);
        let v2 = Vertex::new(1.0, 0.0);
        let v3 = Vertex::new(0.0, 1.0);
        assert_eq!(area_triangle(&v1, &v2, &v3), 0.5);
        assert_eq!(area_triangle(&v3, &v1, &v2), 0.5);
        assert_eq!(area_triangle(&v2, &v3, &v1), 0.5);
    }

    #[test]
    fn test_area_segments() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 0.0));
        let v3 = Rc::new(Vertex::new(0.0, 1.0));
        assert_eq!(area_segments(vec![
            (Rc::clone(&v1), Rc::clone(&v2)),
            (Rc::clone(&v2), Rc::clone(&v3)),
            (Rc::clone(&v3), Rc::clone(&v1)),
        ]), -0.5);
    }
}
