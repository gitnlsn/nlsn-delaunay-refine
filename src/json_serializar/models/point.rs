extern crate serde;

use serde::{Deserialize, Serialize};
use nlsn_delaunay::elements::vertex::Vertex;

#[derive(Serialize, Deserialize, Debug)]
pub struct Point {
    pub x: f64,
    pub y: f64,

    #[serde(default = "zero_f64")]
    pub z: f64,
}

fn zero_f64() -> f64 {
    0.0
}

impl PartialEq for Point {
    fn eq(&self, other: &Self) -> bool {
        return float_cmp::approx_eq!(f64, self.x, other.x, epsilon = 1.0E-14f64)
            && float_cmp::approx_eq!(f64, self.y, other.y, epsilon = 1.0E-14f64);
    }
}

impl Eq for Point {}

impl Point {
    pub fn from_vertex(v: &Vertex) -> Self {
        Self {
            x: v.x,
            y: v.y,
            z: 0.0,
        }
    }
}

#[test]
fn parse_point() {
    let serial = serde_json::from_str(
        "{
            \"x\": 1.0, 
            \"y\": 1.0
        }",
    );
    assert!(serial.is_ok());

    let point: Point = serial.unwrap();
    assert_eq!(point.x, 1.0);
    assert_eq!(point.y, 1.0);
    assert_eq!(point.z, 0.0);

    serde_json::to_string(&point).unwrap();
}
