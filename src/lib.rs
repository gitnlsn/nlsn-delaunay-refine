// ================= //
//      IMPORTS      //
// ================= //

/* Elementary geometric elements */
mod elements {
    pub mod bounding_box;
    pub mod edge;
    pub mod polyline;
    pub mod triangle;
    pub mod vertex;
}

/* Geometric Behaviour/properties implementation */
mod properties {
    pub mod angle;
    pub mod area;
    pub mod circumcenter;
    pub mod continence;
    pub mod distance;
    pub mod dot;
    pub mod encroachment;
    pub mod intersection;
    pub mod midpoint;
    pub mod orientation;
    pub mod parallel;
}

/* Data structure that resumes lib main output */
mod planar {
    pub mod refine_params;
    pub mod triangulation;
    pub mod triangulation_data;
    pub mod triangulator;
    pub mod triangulation_procedures {
        pub mod boundary;
        pub mod hole;
        pub mod segment;
        pub mod vertices;
    }
    pub mod refine_procedures {
        pub mod encroachment;
        pub mod triangle_split;
    }
}

// ================= //
//      EXPORTS      //
// ================= //
pub use crate::elements::{
    edge::Edge,
    polyline::Polyline,
    triangle::Triangle,
    vertex::Vertex
};

pub use crate::planar::{
    triangulation::Triangulation, 
    triangulator::Triangulator
};
