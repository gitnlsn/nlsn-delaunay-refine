use crate::json_serializar::models::input;

use std::fs;
use std::path;

pub fn read(path: &path::Path) -> Option<input::TriangulationInput> {
    match fs::read_to_string(path) {
        Ok(json_string) => {
            match serde_json::from_str(&json_string) {
                Ok(data) => return Some(data),
                Err(_) => return None,
            };
        }
        Err(_) => {
            return None;
        }
    }
}

pub fn write(path: &path::Path, json_string: String) -> std::io::Result<()> {
    fs::write(path, json_string)
}
