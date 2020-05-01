/* Elementary data models */
mod vertex;
mod edge;
mod triangle;

/* Behaviour exported implementation */
mod distance;
mod area;
mod circumcenter;
mod orientation;
mod continence;
mod encroachment;

/* Data structure that resumes lib main output */
mod triangulation;

/* Triangulation algorithm and utilities */
mod triangulator;

pub use crate::vertex::Vertex;
pub use crate::edge::Edge;
pub use crate::triangle::Triangle;
pub use crate::triangulation::Triangulation;
pub use crate::triangulator::Triangulator;
