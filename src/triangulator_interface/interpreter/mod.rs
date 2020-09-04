pub mod circle_parser;
pub mod polyline_parser;
pub mod refine_params_parser;
pub mod segments_parser;
pub mod vertices_parser;

use std::collections::HashSet;
use std::rc::Rc;

use crate::json_serializar::models::{action::Action, input::TriangulationInput};

use nlsn_delaunay::{
    elements::{edge::*, polyline::*, vertex::*},
    planar::refine_params::RefineParams,
};

pub fn parse(
    input: &TriangulationInput,
) -> Result<
    (
        Vec<Rc<Polyline>>,
        Vec<Rc<Polyline>>,
        HashSet<Rc<Edge>>,
        HashSet<Rc<Vertex>>,
        RefineParams,
    ),
    (),
> {
    let mut inclusion_domains: Vec<Rc<Polyline>> = Vec::new();
    let mut removal_domains: Vec<Rc<Polyline>> = Vec::new();
    let mut segment_constraints: HashSet<Rc<Edge>> = HashSet::new();
    let mut vertices_constraints: HashSet<Rc<Vertex>> = HashSet::new();

    for action in input.actions.iter() {
        match action.geometry.as_str() {
            "polyline" => {
                match polyline_parser::parse(action) {
                    Ok(polyline) => match action.intent.as_str() {
                        "include" => {
                            inclusion_domains.push(Rc::new(polyline));
                        }
                        "remove" => {
                            removal_domains.push(Rc::new(polyline));
                        }
                        _ => return Err(()),
                    },
                    Err(_) => return Err(()),
                };
            }
            "circle" => {
                match circle_parser::parse(action) {
                    Ok(polyline) => match action.intent.as_str() {
                        "include" => {
                            inclusion_domains.push(Rc::new(polyline));
                        }
                        "remove" => {
                            removal_domains.push(Rc::new(polyline));
                        }
                        _ => return Err(()),
                    },
                    Err(_) => return Err(()),
                };
            }
            "segments" => {
                match segments_parser::parse(action) {
                    Ok(new_segment_constraints) => match action.intent.as_str() {
                        "constraint" => {
                            segment_constraints = segment_constraints
                                .iter()
                                .chain(new_segment_constraints.iter())
                                .cloned()
                                .collect();
                        }
                        _ => return Err(()),
                    },
                    Err(_) => return Err(()),
                };
            }
            "vertices" => {
                match vertices_parser::parse(action) {
                    Ok(new_vertices_constraints) => match action.intent.as_str() {
                        "constraint" => {
                            vertices_constraints = vertices_constraints
                                .iter()
                                .chain(new_vertices_constraints.iter())
                                .cloned()
                                .collect();
                        }
                        _ => return Err(()),
                    },
                    Err(_) => return Err(()),
                };
            }
            _ => return Err(()),
        } /* end - match geometry */
    } /* end - for action */

    let refine_params = refine_params_parser::parse(&input.params).unwrap();

    return Ok((
        inclusion_domains,
        removal_domains,
        segment_constraints,
        vertices_constraints,
        refine_params,
    ));
} /* end - parse */
