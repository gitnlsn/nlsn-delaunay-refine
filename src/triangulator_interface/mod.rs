pub mod domain_evaluator;
pub mod interpreter;

use std::collections::HashSet;
use std::rc::Rc;

use crate::json_serializar::models::input::TriangulationInput;

use nlsn_delaunay::{
    elements::polyline::*,
    planar::{refine_params::RefineParams, triangulator::Triangulator},
};

pub fn parse(input: &TriangulationInput) -> Result<(Triangulator, RefineParams), ()> {
    let result = interpreter::parse(&input);

    if result.is_err() {
        return Err(());
    }

    let (
        inclusion_domains,    /* Vec<Rc<Polyline>> */
        removal_domains,      /* Vec<Rc<Polyline>> */
        segment_constraints,  /* HashSet<Rc<Edge>> */
        vertices_constraints, /* HashSet<Rc<Vertex>> */
        refine_params,        /* RefineParams */
    ) = result.unwrap();

    let boundary: Rc<Polyline>;
    let unused_removals: Vec<Rc<Polyline>>;
    match domain_evaluator::boundary(&inclusion_domains, &removal_domains) {
        Ok((polyline, unused)) => {
            boundary = polyline;
            unused_removals = unused;
        }
        Err(_) => return Err(()),
    }
    
    let holes: HashSet<Rc<Polyline>> = domain_evaluator::holes(&boundary, &unused_removals);

    let mut triangulator: Triangulator = Triangulator::new(&boundary);
    for hole in holes.iter() {
        let result = triangulator.insert_hole(hole);
        if result.is_err() {
            return Err(());
        }
    }

    let result = triangulator.insert_segments(&segment_constraints);
    if result.is_err() {
        return Err(());
    }

    let result = triangulator.insert_vertices(&vertices_constraints);
    if result.is_err() {
        return Err(());
    }

    return Ok((triangulator, refine_params));
} /* end - parse */
