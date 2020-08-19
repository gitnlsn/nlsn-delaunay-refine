#![macro_use]
extern crate float_cmp;

use num::Float;
use std::cmp::Ordering;
use std::fmt;
use std::fmt::Debug;
use std::hash::{Hash, Hasher};
use std::rc::Rc;

#[derive(Debug)]
pub struct Vertex {
    pub x: f64,
    pub y: f64,
    pub is_ghost: bool,
}

impl Hash for Vertex {
    fn hash<H: Hasher>(&self, state: &mut H) {
        let (m, e, s) = Float::integer_decode(self.x);
        m.hash(state);
        e.hash(state);
        s.hash(state);

        let (m, e, s) = Float::integer_decode(self.y);
        m.hash(state);
        e.hash(state);
        s.hash(state);

        self.is_ghost.hash(state);
    }
}

impl PartialEq for Vertex {
    fn eq(&self, other: &Self) -> bool {
        if self.is_ghost && other.is_ghost {
            return true;
        }

        return self.is_ghost == other.is_ghost
            && float_cmp::approx_eq!(f64, self.x, other.x, epsilon = 1.0E-14f64)
            && float_cmp::approx_eq!(f64, self.y, other.y, epsilon = 1.0E-14f64)
    }
}

impl Eq for Vertex {}

impl Ord for Vertex {
    fn cmp(&self, other: &Self) -> Ordering {
        if self.is_ghost && other.is_ghost {
            return Ordering::Equal;
        }

        if self.x > other.x {
            return Ordering::Greater;
        } else if self.x < other.x {
            return Ordering::Less;
        } else {
            if self.y > other.y {
                return Ordering::Greater;
            } else if self.y < other.y {
                return Ordering::Less;
            } else {
                return Ordering::Equal;
            }
        }
    }
}

impl PartialOrd for Vertex {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(&other))
    }
}

impl fmt::Display for Vertex {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_ghost {
            return write!(f, "(ghost)");
        }
        return write!(f, "({}, {})", self.x, self.y);
    }
}

impl Vertex {
    pub fn new(x: f64, y: f64) -> Self {
        Self {
            x: x,
            y: y,
            is_ghost: false,
        }
    }

    pub fn new_ghost() -> Vertex {
        Vertex {
            x: 0.0,
            y: 0.0,
            is_ghost: true,
        }
    }

    pub fn from_coordinates(raw_array: &Vec<f64>) -> Vec<Rc<Vertex>> {
        if raw_array.len() % 2 != 0 {
            panic!("Vec must provide vertices by pair of x,y coordinates.");
        }

        let list_size = raw_array.len() / 2;

        let mut vertex_list: Vec<Rc<Vertex>> = Vec::with_capacity(list_size);

        for index in 0..list_size {
            let x = raw_array.get(index * 2).unwrap();
            let y = raw_array.get(index * 2 + 1).unwrap();

            let new_vertex = Vertex::new(*x, *y);
            vertex_list.push(Rc::new(new_vertex));
        }

        return vertex_list;
    }

    pub fn sort(vertex_list: &mut Vec<Rc<Vertex>>) {
        vertex_list.sort_by(|v1, v2| match v1.x.partial_cmp(&v2.x) {
            Some(Ordering::Equal) => v1.y.partial_cmp(&v2.y).unwrap(),
            _ => v1.x.partial_cmp(&v2.y).unwrap(),
        });
    }
}

#[cfg(test)]
mod ghost_vertex {
    use super::*;

    #[test]
    fn test_ghost_property_is_bool() {
        let v = Vertex::new_ghost();
        assert!(v.is_ghost);

        let v = Vertex::new(0.0, 0.0);
        assert!(!v.is_ghost);
    }
}

#[cfg(test)]
mod build_from_coordinates {
    use super::*;

    #[test]
    fn test_builds_all_vertices() {
        let raw_array = vec![0.0, 1.0, 4.0, 5.0, 2.0, 3.0];

        let mut vertex_list = Vertex::from_coordinates(&raw_array);

        assert_eq!(vertex_list.len(), 3);

        assert_eq!(vertex_list.get(0).unwrap().x, 0.0);
        assert_eq!(vertex_list.get(0).unwrap().y, 1.0);

        assert_eq!(vertex_list.get(1).unwrap().x, 4.0);
        assert_eq!(vertex_list.get(1).unwrap().y, 5.0);

        assert_eq!(vertex_list.get(2).unwrap().x, 2.0);
        assert_eq!(vertex_list.get(2).unwrap().y, 3.0);

        Vertex::sort(&mut vertex_list);

        assert_eq!(vertex_list.get(1).unwrap().x, 2.0);
        assert_eq!(vertex_list.get(1).unwrap().y, 3.0);

        assert_eq!(vertex_list.get(2).unwrap().x, 4.0);
        assert_eq!(vertex_list.get(2).unwrap().y, 5.0);
    }

    #[test]
    #[should_panic]
    fn test_dont_accept_wrong_size_array() {
        let raw_array = vec![0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 2.0];
        Vertex::from_coordinates(&raw_array);
    }
}

#[cfg(test)]
mod vertex_identity {
    use super::*;

    #[test]
    fn test_two_different_object_with_same_coordinates() {
        let v1 = Vertex::new(1.0, 1.0);
        let v2 = Vertex::new(1.0, 1.0);
        let v3 = Vertex::new(1.0, 1.1);

        assert!(v1 == v2);
        assert!(v1 != v3);
    }
}
