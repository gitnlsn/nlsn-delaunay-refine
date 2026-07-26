use crate::elements::vertex::*;
use crate::properties::orientation::*;

/**
 * Calculates the angle ABC
 */
pub fn angle(a: &Vertex, b: &Vertex, c: &Vertex) -> Option<f64> {
    let (ba_x, ba_y) = (a.x - b.x, a.y - b.y);

    let (bc_x, bc_y) = (c.x - b.x, c.y - b.y);

    let ba_norm = (ba_x * ba_x + ba_y * ba_y).sqrt();
    let bc_norm = (bc_x * bc_x + bc_y * bc_y).sqrt();

    let cos_theta = (ba_x * bc_x + ba_y * bc_y) / (ba_norm * bc_norm);
    let theta = cos_theta.acos();

    if orientation(a, b, c) == Orientation::Clockwise {
        return Some(2.0 * std::f64::consts::PI - theta);
    }

    return Some(theta);
}

#[cfg(test)]
mod angle {
    use super::*;

    #[test]
    fn test_angle() {
        let v1 = Vertex::new(0.0, 1.0);
        let v2 = Vertex::new(0.0, 0.0);
        let v3 = Vertex::new(1.0, 0.0);

        let theta = angle(&v1, &v2, &v3).unwrap();
        assert!((theta - std::f64::consts::FRAC_PI_2).abs() < 1.0e-10);

        let theta = angle(&v2, &v3, &v1).unwrap();
        assert!((theta - std::f64::consts::FRAC_PI_4).abs() < 1.0e-10);

        let theta = angle(&v3, &v2, &v1).unwrap();
        assert!((theta - 3.0 * std::f64::consts::FRAC_PI_2).abs() < 1.0e-10);
    }
}
