/* Elementary data models */
pub mod vertex;
pub mod edge;
pub mod triangle;

/* Behaviour exported implementation */
mod distance;
mod area;
mod circumcenter;
mod orientation;
mod continence;
mod encroachment;

/* Data structure that resumes lib main output */
pub mod triangulation;

/* Triangulation algorithm and utilities */
pub mod triangulator;
