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

/// How many of the 52 mantissa bits are dropped when a coordinate is stored.
///
/// This is the resolution at which two coordinates are considered the same
/// point. It is expressed in mantissa bits rather than as an absolute distance
/// so that it is *relative*: a grid of 1e-14 absolute means nothing above
/// coordinate ~1e2, where the spacing between representable `f64` already
/// exceeds it, and snapping silently becomes a no-op. That is what defeated the
/// previous attempt at this.
///
/// Twelve bits leaves a resolution of 2^-40, about 9e-13 relative. Against a
/// mesh that subdivides to perhaps 1e-3 of its domain, that is some nine orders
/// of magnitude of headroom, so it cannot put a floor under refinement.
const MANTISSA_BITS_DROPPED: u32 = 12;

/// Rounds a coordinate onto the grid described above.
///
/// Idempotent, which matters because `eq` and `hash` both apply it: a value
/// already on the grid has its low bits clear, so adding half a step and masking
/// returns it unchanged.
///
/// Works on the magnitude and reapplies the sign, rather than on the raw bit
/// pattern, so that carrying out of the mantissa cannot walk into the sign bit.
fn quantise(value: f64) -> f64 {
    if value == 0.0 {
        // Folds -0.0 into +0.0. They are numerically equal but differ in bits,
        // which would otherwise be exactly the defect this is here to remove.
        return 0.0;
    }
    if !value.is_finite() {
        return value;
    }

    let half = 1u64 << (MANTISSA_BITS_DROPPED - 1);
    let mask = !((1u64 << MANTISSA_BITS_DROPPED) - 1);

    let rounded = f64::from_bits((value.abs().to_bits().saturating_add(half)) & mask);
    if !rounded.is_finite() {
        return value;
    }

    if value.is_sign_negative() {
        -rounded
    } else {
        rounded
    }
}

/// The identity of a vertex: what `eq` compares and what `hash` hashes.
///
/// Both are defined in terms of this single function, which is what makes the
/// `Hash`/`Eq` contract hold by construction rather than by two implementations
/// happening to agree.
///
/// All ghosts are one vertex, so their coordinates are not part of their
/// identity.
fn identity(vertex: &Vertex) -> (bool, u64, u64) {
    if vertex.is_ghost {
        return (true, 0, 0);
    }
    (
        false,
        quantise(vertex.x).to_bits(),
        quantise(vertex.y).to_bits(),
    )
}

impl Hash for Vertex {
    fn hash<H: Hasher>(&self, state: &mut H) {
        identity(self).hash(state);
    }
}

/// Exact equality on quantised coordinates.
///
/// The previous implementation compared with `approx_eq!` at 1e-14 while `Hash`
/// hashed raw bits, which broke the contract two ways. Two vertices closer than
/// the epsilon compared equal and landed in different buckets, so
/// `HashSet::contains` reported a vertex absent while an equal one was present
/// -- and `vertices::include` reads that as "new" and inserts a *second* vertex
/// a hair from the first. Worse, approximate equality is not transitive at any
/// tolerance, so it was not an equivalence relation and could not lawfully back
/// `Eq` at all.
///
/// Quantising first restores both properties: equality is exact, so it agrees
/// with the hash by construction, and it is transitive because it is equality on
/// a grid. It keeps the tolerant behaviour that the mesher depends on, since the
/// same geometric point reached by two different routes -- a circumcentre and a
/// midpoint, say -- lands on the same grid cell.
impl PartialEq for Vertex {
    fn eq(&self, other: &Self) -> bool {
        identity(self) == identity(other)
    }
}

impl Eq for Vertex {}

impl Ord for Vertex {
    fn cmp(&self, other: &Self) -> Ordering {
        // Ghosts are ordered first, and compared before coordinates. Comparing
        // coordinates alone reported a ghost as `Equal` to a real vertex at the
        // origin, which would contradict the equality above.
        match (self.is_ghost, other.is_ghost) {
            (true, true) => return Ordering::Equal,
            (true, false) => return Ordering::Less,
            (false, true) => return Ordering::Greater,
            (false, false) => {}
        }

        // On the quantised values, so that ordering agrees with equality.
        quantise(self.x)
            .partial_cmp(&quantise(other.x))
            .unwrap_or(Ordering::Equal)
            .then_with(|| {
                quantise(self.y)
                    .partial_cmp(&quantise(other.y))
                    .unwrap_or(Ordering::Equal)
            })
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
    /// Coordinates are rounded onto the identity grid here, so that every
    /// vertex in the mesh is already canonical. `new` is the single funnel for
    /// non-ghost vertices -- `Edge::midpoint`, `Triangle::center` and
    /// `properties::circumcenter` all route through it -- so this is the one
    /// place it has to happen.
    pub fn new(x: f64, y: f64) -> Self {
        Self {
            x: quantise(x),
            y: quantise(y),
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
