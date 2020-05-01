use crate::elements::vertex::*;

pub fn distance(v1: &Vertex, v2: &Vertex) -> f64 {
    ((v1.x - v2.x).powi(2) + (v1.y - v2.y).powi(2)).sqrt()
}

#[cfg(test)]
mod distance {
    use super::*;

    #[test]
    fn test_axis_x() {
        let v1 = Vertex::new(0.0, 0.0);
        let v2 = Vertex::new(1.0, 0.0);
        let v3 = Vertex::new(3.0, 0.0);
        assert_eq!(distance(&v1, &v2), 1.0);
        assert_eq!(distance(&v1, &v3), 3.0);
    }

    #[test]
    fn test_axis_y() {
        let v1 = Vertex::new(0.0, 0.0);
        let v2 = Vertex::new(0.0, 1.0);
        let v3 = Vertex::new(0.0, 3.0);
        assert_eq!(distance(&v1, &v2), 1.0);
        assert_eq!(distance(&v1, &v3), 3.0);
    }
    
    #[test]
    fn test_known_cases() {
        let v1 = Vertex::new(0.0, 0.0);
        let v2 = Vertex::new(3.0, 4.0);
        assert_eq!(distance(&v1, &v2), 5.0);
    }
}
