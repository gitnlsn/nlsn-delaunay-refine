use crate::json_serializar::models::{action::Action, point::Point};
use std::collections::HashSet;
use std::rc::Rc;

use nlsn_delaunay::elements::vertex::*;

pub fn parse(action: &Action) -> Result<HashSet<Rc<Vertex>>, ()> {
    let vertices: HashSet<Rc<Vertex>> = action
        .points
        .iter()
        .map(|p| point_to_vertex(p))
        .map(|v| Rc::new(v))
        .collect();

    return Ok(vertices);
} /* end - parse */

fn point_to_vertex(point: &Point) -> Vertex {
    Vertex::new(point.x, point.y)
}
