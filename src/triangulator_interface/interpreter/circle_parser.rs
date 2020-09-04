use crate::json_serializar::models::{action::Action, point::Point};
use std::rc::Rc;

use nlsn_delaunay::{
    elements::{polyline::*, vertex::*},
};

pub fn parse(action: &Action) -> Result<Polyline, ()> {
    let defined_by_center_radius = action.scalars.len() >= 1 && action.points.len() == 1;
    if defined_by_center_radius {
        let mut vertices: Vec<Rc<Vertex>> = Vec::new();
        let radius = *action.scalars.get(0).unwrap();
        let center = action.points.get(0).unwrap();
        let resolution: usize = match action.scalars.get(1) {
            Some(value) => (value.round() as usize),
            None => 100,
        };

        let dphi = std::f64::consts::PI * 2.0 / resolution as f64;
        for index in 0..resolution {
            let angle: f64 = dphi * index as f64;
            let vertex = get_circle_point(radius, angle, center);
            vertices.push(Rc::new(vertex));
        }

        return Ok(Polyline::new_closed(vertices).unwrap());
    }
    return Err(());
}

fn get_circle_point(
    radius: f64,
    angle: f64,
    center: &Point,
) -> Vertex {
    let dx = radius * angle.cos();
    let dy = radius * angle.sin();
    return Vertex::new(center.x + dx, center.y + dy);
}