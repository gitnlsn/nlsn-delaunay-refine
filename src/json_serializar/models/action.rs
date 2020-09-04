use serde::{Deserialize, Serialize};
use crate::json_serializar::models::point;

/**
 * Triangulation domain is described as a composition of solids
 */
#[derive(Serialize, Deserialize, Debug)]
pub struct Action {
    /*
        Intent is one of the following:
            - include
            - remove
            - constraint
    */
    pub intent: String,

    /*
        Describes the geometric form:
            - polyline (rectangle, triangle, ...polygons)
            - circle (center + radius)
            - segments (as constraints)
            - vertices (as constraints)
            - spline (todo?: not implemented)
    */
    pub geometry: String,

    /* Scalar values used to describe geometry */
    #[serde(default = "empty_scalar")]
    pub scalars: Vec<f64>,

    /* Coordinates used to describe geometry */
    #[serde(default = "empty_points")]
    pub points: Vec<point::Point>,

    /* Assembles points in 3D */
    #[serde(default = "empty_assemble")]
    pub assemble: Vec<Vec<usize>>,
}

/* default scalars vec */
fn empty_scalar() -> Vec<f64> {
    Vec::new()
}

/* default points vector */
fn empty_points() -> Vec<point::Point> {
    Vec::new()
}

/* default poits assemble vec vec */
fn empty_assemble() -> Vec<Vec<usize>> {
    Vec::new()
}

#[test]
fn parse_circle() {
    let serial = serde_json::from_str(
        "{
            \"intent\": \"include\",
            \"geometry\": \"circle\",
            \"scalars\": [ 1.0 ],
            \"points\": [{ \"x\": 1.0,  \"y\": 1.0 }]
        }",
    );

    assert!(serial.is_ok());

    let circle_intent: Action = serial.unwrap();
    assert_eq!(circle_intent.intent, "include");
    assert_eq!(circle_intent.geometry, "circle");

    assert_eq!(circle_intent.scalars.len(), 1);
    let circle_radius: &f64 = circle_intent.scalars.get(0).unwrap();
    assert_eq!(circle_radius, &1.0);

    assert_eq!(circle_intent.points.len(), 1);
    let circle_center: &point::Point = circle_intent.points.iter().next().unwrap();
    assert_eq!(circle_center.x, 1.0);
    assert_eq!(circle_center.y, 1.0);
}

#[test]
fn parse_polyline() {
    let serial = serde_json::from_str(
        "{
            \"intent\": \"include\",
            \"geometry\": \"polyline\",
            \"points\": [
                { \"x\": 0.0,  \"y\": 0.0 },
                { \"x\": 1.0,  \"y\": 0.0 },
                { \"x\": 1.0,  \"y\": 1.0 }
            ]
        }",
    );

    assert!(serial.is_ok());

    let polyline_intent: Action = serial.unwrap();
    assert_eq!(polyline_intent.intent, "include");
    assert_eq!(polyline_intent.geometry, "polyline");

    assert!(polyline_intent.scalars.is_empty());
    assert!(polyline_intent.assemble.is_empty());

    assert_eq!(polyline_intent.points.len(), 3);
    let mut polyline_points = polyline_intent.points.iter();

    let p1 = polyline_points.next().unwrap();
    assert_eq!(p1.x, 0.0);
    assert_eq!(p1.y, 0.0);

    let p2 = polyline_points.next().unwrap();
    assert_eq!(p2.x, 1.0);
    assert_eq!(p2.y, 0.0);

    let p3 = polyline_points.next().unwrap();
    assert_eq!(p3.x, 1.0);
    assert_eq!(p3.y, 1.0);
}

#[test]
fn parse_segments() {
    let serial = serde_json::from_str(
        "{
            \"intent\": \"constraint\",
            \"geometry\": \"segments\",
            \"points\": [
                { \"x\": 0.0,  \"y\": 0.0 },
                { \"x\": 1.0,  \"y\": 0.0 },
                { \"x\": 1.0,  \"y\": 1.0 },
                { \"x\": 0.0,  \"y\": 1.0 }
            ],
            \"assemble\": [
                [ 0, 1 ],
                [ 2, 3 ]
            ]
        }",
    );

    assert!(serial.is_ok());

    let segments_constraints: Action = serial.unwrap();
    assert_eq!(segments_constraints.intent, "constraint");
    assert_eq!(segments_constraints.geometry, "segments");

    /* No scalars */
    assert!(segments_constraints.scalars.is_empty());

    /* 4 points */
    assert_eq!(segments_constraints.points.len(), 4);
    let mut segments_end_vertices = segments_constraints.points.iter();

    let p1 = segments_end_vertices.next().unwrap();
    assert_eq!(p1.x, 0.0);
    assert_eq!(p1.y, 0.0);

    let p2 = segments_end_vertices.next().unwrap();
    assert_eq!(p2.x, 1.0);
    assert_eq!(p2.y, 0.0);

    let p3 = segments_end_vertices.next().unwrap();
    assert_eq!(p3.x, 1.0);
    assert_eq!(p3.y, 1.0);

    let p4 = segments_end_vertices.next().unwrap();
    assert_eq!(p4.x, 0.0);
    assert_eq!(p4.y, 1.0);

    /* Points assemble (optional to segments) */
    assert_eq!(segments_constraints.assemble.len(), 2);
    let mut assemble_set = segments_constraints.assemble.iter();

    let s1 = assemble_set.next().unwrap();
    assert_eq!(s1.get(0), Some(&0));
    assert_eq!(s1.get(1), Some(&1));
    
    let s2 = assemble_set.next().unwrap();
    assert_eq!(s2.get(0), Some(&2));
    assert_eq!(s2.get(1), Some(&3));
}
