extern crate nalgebra;

use crate::elements::vertex::*;
use crate::properties::orientation::*;

use nalgebra::Vector2;

/**
 * Calculates the ABC
 */
pub fn angle(a: &Vertex, b: &Vertex, c: &Vertex) -> Option<f64> {
    let ba: Vector2<f64> = Vector2::new(a.x - b.x, a.y - b.y);

    let bc: Vector2<f64> = Vector2::new(c.x - b.x, c.y - b.y);

    let cos_theta = ba.dot(&bc) / (ba.norm() * bc.norm());
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
