use crate::elements::vertex::*;

/**
 * Calculates dot product of oriented segments ab and cd
 */
pub fn dot(a: &Vertex, b: &Vertex, c: &Vertex, d: &Vertex) -> f64 {
    let x1 = a.x;
    let y1 = a.y;

    let x2 = b.x;
    let y2 = b.y;

    let x3 = c.x;
    let y3 = c.y;

    let x4 = d.x;
    let y4 = d.y;

    let (ab_x, ab_y) = (x2 - x1, y2 - y1);

    let (cd_x, cd_y) = (x4 - x3, y4 - y3);

    return ab_x * cd_x + ab_y * cd_y;
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn sample_1() {
        let v1 = Vertex::new(0.0, 0.0);
        let v2 = Vertex::new(2.0, 2.0);
        let v3 = Vertex::new(0.0, 1.0);
        let v4 = Vertex::new(2.0, 3.0);
        assert_eq!(dot(&v1, &v2, &v3, &v4), 8.0);
        
        let v1 = Vertex::new(0.0, 0.0);
        let v2 = Vertex::new(2.0, 2.0);
        let v3 = Vertex::new(2.0, 3.0);
        let v4 = Vertex::new(0.0, 1.0);
        assert_eq!(dot(&v1, &v2, &v3, &v4), -8.0);
    }
}
