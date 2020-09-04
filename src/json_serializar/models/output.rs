extern crate chrono;
extern crate serde;
extern crate uuid;

use chrono::{SecondsFormat, Utc};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

use std::collections::HashMap;
use std::rc::Rc;

use crate::json_serializar::models::{input::TriangulationInput, point, tesselations};
use nlsn_delaunay::{elements::vertex::Vertex, planar::triangulator::Triangulator};

#[derive(Serialize, Deserialize, Debug)]
pub struct TriangulationOutput {
    #[serde(default = "new_uuid")]
    pub id: Uuid,

    pub name: String,

    #[serde(default = "now")]
    pub date: String,

    pub coordinates: Vec<point::Point>,

    #[serde(default = "empty_triangles")]
    pub triangles: Vec<tesselations::Triangle>,

    #[serde(default = "empty_tetrahedrons")]
    pub tetrahedrons: Vec<tesselations::Tetrahedron>,
}

fn new_uuid() -> Uuid {
    Uuid::new_v4()
}

/* default empty tetrahedrons list */
fn empty_tetrahedrons() -> Vec<tesselations::Tetrahedron> {
    Vec::new()
}

/* default empty triangles list */
fn empty_triangles() -> Vec<tesselations::Triangle> {
    Vec::new()
}

/* default date: now */
fn now() -> String {
    Utc::now().to_rfc3339_opts(SecondsFormat::Millis, true)
}

impl TriangulationOutput {
    pub fn from_triangulator(input: &TriangulationInput, triangulator: &Triangulator) -> Self {
        let mut vertices_map: HashMap<Rc<Vertex>, usize> = HashMap::new();

        let vertices_vec: Vec<Rc<Vertex>> = triangulator
            .triangulation
            .borrow()
            .vertices()
            .iter()
            .cloned()
            .collect();

        for index in 0..vertices_vec.len() {
            let v = vertices_vec.get(index).unwrap();
            vertices_map.insert(Rc::clone(v), index);
        }

        return Self {
            id: input.id,
            name: input.name.clone(),
            date: input.date.clone(),
            coordinates: vertices_map
                .keys()
                .map(|v| point::Point::from_vertex(v))
                .collect(),
            triangles: triangulator
                .triangulation
                .borrow()
                .triangles
                .iter()
                .filter(|t| !t.is_ghost())
                .map(|t| {
                    let v1 = vertices_map.get(&t.v1).unwrap();
                    let v2 = vertices_map.get(&t.v2).unwrap();
                    let v3 = vertices_map.get(&t.v3).unwrap();
                    return tesselations::Triangle::new(*v1, *v2, *v3);
                })
                .collect(),
            tetrahedrons: Vec::new(),
        };
    } /* end - from triangulator */
} /* end - TriangulatorOutput */
