#![macro_use]
extern crate glium;

use nlsn_delaunay::elements::edge::Edge;
use nlsn_delaunay::planar::triangulation::Triangulation;

use std::collections::HashSet;
use std::rc::Rc;

#[derive(Copy, Clone)]
pub struct Vertex {
    pub position: [f32; 2],
}

glium::implement_vertex!(Vertex, position);

impl Vertex {
    pub fn from_coordinates(coordinates: Vec<f32>) -> Vec<Self> {
        let mut output: Vec<Self> = Vec::new();

        for index in (0..coordinates.len()).step_by(2) {
            let x = coordinates.get(index).unwrap();
            let y = coordinates.get(index + 1).unwrap();
            output.push(Vertex { position: [*x, *y] });
        }

        return output;
    }

    pub fn triangles_from_triangulation(triangulation: &Triangulation) -> Vec<Self> {
        triangulation
            .triangles
            .iter()
            .filter(|t| !t.is_ghost())
            .map(|t| vec![Rc::clone(&t.v1), Rc::clone(&t.v2), Rc::clone(&t.v3)])
            .flatten()
            .map(|v| Vertex {
                position: [v.x as f32, v.y as f32],
            })
            .collect()
    }

    pub fn edges_from_triangulation(triangulation: &Triangulation) -> Vec<Self> {
        let mut aux_list: Vec<Rc<Edge>> = triangulation
            .triangles
            .iter()
            .filter(|t| !t.is_ghost())
            .map(|t| {
                let (e1, e2, e3) = t.inner_edges();
                vec![e1, e2, e3]
            })
            .flatten()
            .collect();

        let mut clean_list: HashSet<Rc<Edge>> = HashSet::new();
        while let Some(edge) = aux_list.pop() {
            if !clean_list.contains(&edge) & !clean_list.contains(&edge.opposite()) {
                clean_list.insert(Rc::clone(&edge));
            }
        }

        clean_list
            .iter()
            .map(|e| vec![Rc::clone(&e.v1), Rc::clone(&e.v2)])
            .flatten()
            .map(|v| Vertex {
                position: [v.x as f32, v.y as f32],
            })
            .collect()
    }
}
