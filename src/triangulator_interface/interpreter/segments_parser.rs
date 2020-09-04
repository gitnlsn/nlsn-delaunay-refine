use crate::json_serializar::models::{action::Action, point::Point};
use std::collections::HashSet;
use std::rc::Rc;

use nlsn_delaunay::elements::{edge::*, vertex::*};

pub fn parse(action: &Action) -> Result<HashSet<Rc<Edge>>, ()> {
    let mut edges: HashSet<Rc<Edge>> = HashSet::new();

    /* Case segments are connected by assemble */
    if !action.assemble.is_empty() {
        for set in action.assemble.iter() {
            if set.len() < 2 {
                return Err(());
            }
            let p1_index: usize = *set.get(0).unwrap();
            let p2_index: usize = *set.get(1).unwrap();

            let p1 = action.points.get(p1_index).unwrap();
            let p2 = action.points.get(p2_index).unwrap();

            let v1 = Rc::new(point_to_vertex(p1));
            let v2 = Rc::new(point_to_vertex(p2));

            edges.insert(Rc::new(Edge::new(&v1, &v2)));
        }
        return Ok(edges);
    }

    /* If segments are defined every two points */
    if action.points.len() % 2 != 0 {
        /* odd number of points => corrupted data */
        return Err(());
    }
    for index in (0..action.points.len()).step_by(2) {
        let p1_index: usize = index;
        let p2_index: usize = index + 1;

        let p1 = action.points.get(p1_index).unwrap();
        let p2 = action.points.get(p2_index).unwrap();

        let v1 = Rc::new(point_to_vertex(p1));
        let v2 = Rc::new(point_to_vertex(p2));

        edges.insert(Rc::new(Edge::new(&v1, &v2)));
    }
    return Ok(edges);
} /* end - parse */

fn point_to_vertex(point: &Point) -> Vertex {
    Vertex::new(point.x, point.y)
}
