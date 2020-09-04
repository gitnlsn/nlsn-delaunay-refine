extern crate serde;

use crate::json_serializar::models::point;
use nlsn_delaunay::elements::triangle;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct Triangle {
    pub v1: usize,
    pub v2: usize,
    pub v3: usize,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Tetrahedron {
    pub v1: usize,
    pub v2: usize,
    pub v3: usize,
    pub v4: usize,
}

impl Triangle {
    pub fn new(v1: usize, v2: usize, v3: usize) -> Self {
        Self {
            v1: v1,
            v2: v2,
            v3: v3,
        }
    }
}
