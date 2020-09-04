use crate::json_serializar::models::{action::Action, point::Point};
use std::rc::Rc;

use nlsn_delaunay::elements::{polyline::*, vertex::*};

pub fn parse(action: &Action) -> Result<Polyline, ()> {
    let vertices: Vec<Rc<Vertex>> = action
        .points
        .iter()
        .map(|p| point_to_vertex(p))
        .map(|v| Rc::new(v))
        .collect();

    if vertices.is_empty() {
        return Err(());
    }

    let segments = vertex_pairs(&vertices, false);
    let split_segments = split_intersections(&segments);
    if split_segments.len() > segments.len() {
        /*
            Corrupted data
            Segments split againts itself accused intersection
        */
        return Err(());
    }

    return Ok(Polyline::new_closed(vertices).unwrap());
} /* end - parse */

fn point_to_vertex(point: &Point) -> Vertex {
    Vertex::new(point.x, point.y)
}
