use crate::elements::vertex::*;
use std::rc::Rc;

pub fn area_segments(segments_list: &Vec<(Rc<Vertex>, Rc<Vertex>)>) -> f64 {
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

    // Signed area is half the determinant of
    //
    //     [x1 y1 1]
    //     [x2 y2 1]
    //     [x3 y3 1]
    //
    // expanded along the first row. The unit column makes two of the three 2x2
    // minors collapse to plain differences.
    //
    // Note this is the rounded determinant, not an exact predicate: callers
    // wanting only the *sign* of the area should use `orientation` instead,
    // which is backed by robust::orient2d.
    let determinant = x1 * (y2 - y3) - y1 * (x2 - x3) + (x2 * y3 - x3 * y2);
    return determinant / 2.0;
}

#[cfg(test)]
mod area_segments {
    use super::*;

    #[test]
    fn sample_1() {
        use crate::elements::polyline::*;
        let mut vertices: Vec<Rc<Vertex>> = Vec::new();
        vertices.push(Rc::new(Vertex::new(0.0, 0.0)));
        vertices.push(Rc::new(Vertex::new(1.0, 0.0)));
        vertices.push(Rc::new(Vertex::new(1.0, 1.0)));
        vertices.push(Rc::new(Vertex::new(0.0, 1.0)));

        let pairs = vertex_pairs(&vertices, false);
        assert_eq!(area_segments(&pairs), -1.0);
    }

    #[test]
    fn sample_2() {
        use crate::elements::polyline::*;
        let mut vertices: Vec<Rc<Vertex>> = Vec::new();
        vertices.push(Rc::new(Vertex::new(0.0, 1.0)));
        vertices.push(Rc::new(Vertex::new(1.0, 1.0)));
        vertices.push(Rc::new(Vertex::new(2.0, 1.0)));
        vertices.push(Rc::new(Vertex::new(2.0, 2.0)));
        vertices.push(Rc::new(Vertex::new(1.0, 2.0)));
        vertices.push(Rc::new(Vertex::new(0.0, 2.0)));

        let pairs = vertex_pairs(&vertices, false);
        assert_eq!(area_segments(&pairs), -2.0);
    }

    #[test]
    fn sample_3() {
        use crate::elements::polyline::*;
        let mut vertices: Vec<Rc<Vertex>> = Vec::new();
        vertices.push(Rc::new(Vertex::new(0.0, 0.0)));
        vertices.push(Rc::new(Vertex::new(1.0, 0.5)));
        vertices.push(Rc::new(Vertex::new(2.0, 0.0)));
        vertices.push(Rc::new(Vertex::new(3.0, 0.5)));
        vertices.push(Rc::new(Vertex::new(4.0, 0.0)));
        vertices.push(Rc::new(Vertex::new(4.0, 1.0)));
        vertices.push(Rc::new(Vertex::new(3.0, 1.5)));
        vertices.push(Rc::new(Vertex::new(2.0, 1.0)));
        vertices.push(Rc::new(Vertex::new(1.0, 1.5)));
        vertices.push(Rc::new(Vertex::new(0.0, 1.0)));

        let pairs = vertex_pairs(&vertices, false);
        assert_eq!(area_segments(&pairs), -4.0);
    }

    #[test]
    fn sample_4() {
        use crate::elements::polyline::*;
        let mut vertices: Vec<Rc<Vertex>> = Vec::new();
        vertices.push(Rc::new(Vertex::new(4.0, 1.0)));
        vertices.push(Rc::new(Vertex::new(5.0, 1.0)));
        vertices.push(Rc::new(Vertex::new(6.0, 2.0)));
        vertices.push(Rc::new(Vertex::new(4.0, 4.0)));
        vertices.push(Rc::new(Vertex::new(3.0, 4.0)));
        vertices.push(Rc::new(Vertex::new(1.0, 2.0)));
        vertices.push(Rc::new(Vertex::new(2.0, 1.0)));
        vertices.push(Rc::new(Vertex::new(3.0, 1.0)));
        vertices.push(Rc::new(Vertex::new(2.0, 2.0)));
        vertices.push(Rc::new(Vertex::new(3.0, 3.0)));
        vertices.push(Rc::new(Vertex::new(4.0, 3.0)));
        vertices.push(Rc::new(Vertex::new(5.0, 2.0)));

        let pairs = vertex_pairs(&vertices, false);
        assert_eq!(area_segments(&pairs), -6.0);
    }
}

#[cfg(test)]
mod area_triangle {
    use super::*;

    #[test]
    fn sample_1() {
        let v1 = Vertex::new(0.0, 0.0);
        let v2 = Vertex::new(1.0, 0.0);
        let v3 = Vertex::new(0.0, 1.0);
        assert_eq!(area_triangle(&v1, &v2, &v3), 0.5);
        assert_eq!(area_triangle(&v3, &v1, &v2), 0.5);
        assert_eq!(area_triangle(&v2, &v3, &v1), 0.5);
    }

    #[test]
    fn sample_2() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 0.0));
        let v3 = Rc::new(Vertex::new(0.0, 1.0));
        let segments: Vec<(Rc<Vertex>, Rc<Vertex>)> = vec! [
            (Rc::clone(&v1), Rc::clone(&v2)),
            (Rc::clone(&v2), Rc::clone(&v3)),
            (Rc::clone(&v3), Rc::clone(&v1)),
        ];
        assert_eq!(area_segments(&segments), -0.5);
    }
}
