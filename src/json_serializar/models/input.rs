extern crate chrono;
extern crate serde;
extern crate uuid;

use chrono::{SecondsFormat, Utc};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::json_serializar::models::action;

#[derive(Serialize, Deserialize, Debug)]
pub struct TriangulationInput {
    #[serde(default = "new_uuid")]
    pub id: Uuid,

    pub name: String,
    
    #[serde(default = "now")]
    pub date: String,

    pub actions: Vec<action::Action>,

    pub params: RefineParams,
}

fn new_uuid() -> Uuid {
    Uuid::new_v4()
}

/* default date: now */
fn now() -> String {
    Utc::now().to_rfc3339_opts(SecondsFormat::Millis, true)
}

#[derive(Serialize, Deserialize, Debug)]
pub struct RefineParams {
    pub max_area: Option<f64>,
    pub quality: f64,
}

#[test]
fn parse_refine_params() {
    let serial = serde_json::from_str(
        "{
            \"max_area\": 0.001,
            \"quality\": 1.0
        }",
    );
    assert!(serial.is_ok());

    let params: RefineParams = serial.unwrap();
    assert_eq!(params.max_area, Some(0.001));
    assert_eq!(params.quality, 1.0);
}

#[test]
fn parse_refine_params_no_max_area() {
    let serial = serde_json::from_str(
        "{
            \"quality\": 1.0
        }",
    );
    assert!(serial.is_ok());

    let params: RefineParams = serial.unwrap();
    assert!(params.max_area.is_none());
    assert_eq!(params.quality, 1.0);
}

#[test]
fn parse_triangulation() {
    let serial = serde_json::from_str(
        "{
            \"name\": \"sample_1\",
            \"date\": \"2020-09-03T00:09:27.591Z\",
            \"actions\": [
                {
                    \"intent\": \"include\",
                    \"geometry\": \"circle\",
                    \"scalars\": [ 1.0 ],
                    \"points\": [{ \"x\": 1.0,  \"y\": 1.0 }]
                }
            ],
            \"params\": {
                \"max_area\": 0.001,
                \"quality\": 1.0
            }
        }",
    );
    assert!(serial.is_ok());

    let triangulation: TriangulationInput = serial.unwrap();
    assert!(!triangulation.actions.is_empty());
    assert_eq!(triangulation.name, "sample_1");
    assert_eq!(triangulation.date, "2020-09-03T00:09:27.591Z");

    assert_eq!(triangulation.params.quality, 1.0);
    assert_eq!(triangulation.params.max_area, Some(0.001));
}

#[test]
fn parse_triangulation_with_actions() {
    let serial = serde_json::from_str(
        "{
            \"name\": \"sample_1\",
            \"date\": \"2020-09-03T00:09:27.591Z\",
            \"actions\": [
                {
                    \"intent\": \"include\",
                    \"geometry\": \"circle\",
                    \"scalars\": [ 1.0 ],
                    \"points\": [{ \"x\": 1.0,  \"y\": 1.0 }]
                }
            ],
            \"params\": {
                \"max_area\": 0.001,
                \"quality\": 1.0
            }
        }",
    );
    assert!(serial.is_ok());

    let triangulation: TriangulationInput = serial.unwrap();
    assert_eq!(triangulation.actions.len(), 1);

    let first_action = triangulation.actions.get(0).unwrap();

    assert_eq!(first_action.intent, "include");
    assert_eq!(first_action.geometry, "circle");
}
