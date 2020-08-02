// ================= //
//      IMPORTS      //
// ================= //

/* Elementary geometric elements */
mod elements {
    pub mod vertex;
    pub mod edge;
    pub mod triangle;
    pub mod bounding_box;
    pub mod polyline;
}

/* Geometric Behaviour/properties implementation */
mod properties {
    pub mod area;
    pub mod distance;
    pub mod circumcenter;
    pub mod orientation;
    pub mod continence;
    pub mod encroachment;
    pub mod intersection;
    pub mod angle;
}

/* Data structure that resumes lib main output */
mod planar {
    pub mod triangulation;
    pub mod triangulator;
}

// ================= //
//      EXPORTS      //
// ================= //
pub use crate::elements::{
    vertex::Vertex,
    edge::Edge,
    triangle::Triangle,
    polyline::Polyline,
};

pub use crate::planar::{
    triangulation::Triangulation,
    triangulator::Triangulator,
};
