use crate::json_serializar::models::input;

use nlsn_delaunay::planar::refine_params;

pub fn parse(params: &input::RefineParams) -> Result<refine_params::RefineParams, ()> {
    return Ok(refine_params::RefineParams {
        max_area: params.max_area,
        quality_ratio: params.quality,
    });
} /* end - parse */
