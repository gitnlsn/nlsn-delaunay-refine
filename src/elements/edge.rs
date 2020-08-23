use crate::elements::{bounding_box::*, vertex::*};
use crate::properties::{
    continence::*, distance::*, dot::*, encroachment::*, intersection::*, orientation::*,
    parallel::*,
};
use std::rc::Rc;

use std::cell::RefCell;
use std::cmp::Eq;
use std::collections::{HashMap, HashSet};
use std::hash::Hash;

use std::fmt;
use std::fmt::Debug;

#[derive(Hash, Debug)]
pub struct Edge {
    pub v1: Rc<Vertex>,
    pub v2: Rc<Vertex>,
}

impl PartialEq for Edge {
    fn eq(&self, other: &Self) -> bool {
        /* oriented edge */
        self.v1 == other.v1 && self.v2 == other.v2
    }
}

impl Eq for Edge {}

impl fmt::Display for Edge {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        return write!(f, "({} - {})", self.v1, self.v2);
    }
}

impl Edge {
    pub fn new(v1: &Rc<Vertex>, v2: &Rc<Vertex>) -> Self {
        Self {
            v1: Rc::clone(v1),
            v2: Rc::clone(v2),
        }
    }

    pub fn opposite(&self) -> Self {
        Self {
            v1: Rc::clone(&self.v2),
            v2: Rc::clone(&self.v1),
        }
    }

    pub fn length(&self) -> f64 {
        return distance(&self.v1, &self.v2);
    }

    pub fn encroach(&self, vertex: &Vertex) -> Continence {
        return encroach(&self.v1, &self.v2, vertex);
    }

    pub fn midpoint(&self) -> Vertex {
        let x1 = self.v1.x;
        let y1 = self.v1.y;
        let x2 = self.v2.x;
        let y2 = self.v2.y;

        let x_mid = (x1 + x2) / 2.0;
        let y_mid = (y1 + y2) / 2.0;

        return Vertex::new(x_mid, y_mid);
    }

    pub fn from_coordinates(coordinates: &Vec<f64>) -> Vec<Rc<Edge>> {
        if coordinates.len() % 2 != 0 {
            panic!("Vec must provide vertices by pair of x,y coordinates.");
        }

        let vertices_list = Vertex::from_coordinates(coordinates);
        let mut edge_list: Vec<Rc<Edge>> = Vec::new();

        for index in 0..vertices_list.len() {
            let v1 = vertices_list.get(index).unwrap();
            let v2 = match vertices_list.get(index + 1) {
                Some(vertex) => vertex,
                None => vertices_list.get(0).unwrap(),
            };
            let new_edge = Rc::new(Edge::new(v1, v2));
            edge_list.push(new_edge);
        }

        return edge_list;
    }

    pub fn from_vertices(vertices_list: &Vec<Rc<Vertex>>) -> Vec<Rc<Edge>> {
        let mut edge_list: Vec<Rc<Edge>> = Vec::new();

        for index in 0..vertices_list.len() {
            let v1 = vertices_list.get(index).unwrap();
            let v2 = match vertices_list.get(index + 1) {
                Some(vertex) => vertex,
                None => vertices_list.get(0).unwrap(),
            };
            let new_edge = Rc::new(Edge::new(v1, v2));
            edge_list.push(new_edge);
        }

        return edge_list;
    }

    pub fn into_vertex_pairs(edges: Vec<Rc<Edge>>) -> Vec<(Rc<Vertex>, Rc<Vertex>)> {
        edges
            .iter()
            .map(|edge| (Rc::clone(&edge.v1), Rc::clone(&edge.v2)))
            .collect()
    }

    pub fn from_vertex_pairs(vertex_pairs: Vec<(Rc<Vertex>, Rc<Vertex>)>) -> Vec<Rc<Edge>> {
        vertex_pairs
            .iter()
            .map(|(v1, v2)| Rc::new(Edge::new(v1, v2)))
            .collect()
    }

    pub fn contains(&self, vertex: &Vertex) -> bool {
        let bbox =
            BoundingBox::from_vertices(vec![Rc::clone(&self.v1), Rc::clone(&self.v2)]).unwrap();

        if !bbox.contains(vertex) {
            return false;
        }

        if orientation(&self.v1, &self.v2, vertex) != Orientation::Colinear {
            return false;
        }

        return true;
    }

    /**
     * Concatenates colinear edges
     */
    pub fn arrange(edges: &HashSet<Rc<Edge>>) -> HashSet<Rc<Self>> {
        fn remove_edge(
            head_tail: &mut HashMap<Rc<Vertex>, RefCell<HashSet<Rc<Vertex>>>>,
            tail_head: &mut HashMap<Rc<Vertex>, RefCell<HashSet<Rc<Vertex>>>>,
            (head, tail): (&Rc<Vertex>, &Rc<Vertex>),
        ) {
            head_tail.get(head).unwrap().borrow_mut().remove(tail);

            if head_tail.get(head).unwrap().borrow().is_empty() {
                head_tail.remove(head);
            }

            /* Fixes the opposing list */
            let head_vertices = tail_head.get(tail).unwrap();
            head_vertices.borrow_mut().remove(head);
            if head_vertices.borrow().is_empty() {
                tail_head.remove(tail);
            }
        }
        let (mut head_tail_hashmap, mut tail_head_hashmap) = Self::into_hashmap(edges);

        let mut arranged_edges: HashSet<Rc<Edge>> = HashSet::new();
        while !head_tail_hashmap.is_empty() {
            let (head, tail_vertices) = head_tail_hashmap.iter().next().unwrap();
            let mut head = Rc::clone(&head);
            let mut tail = Rc::clone(tail_vertices.borrow().iter().next().unwrap());

            remove_edge(
                &mut head_tail_hashmap,
                &mut tail_head_hashmap,
                (&head, &tail),
            );

            /* Extends segment */
            loop {
                let mut did_extend = false;

                if head_tail_hashmap.contains_key(&tail) {
                    let possible_next_tails: HashSet<Rc<Vertex>> = head_tail_hashmap
                        .get(&tail)
                        .unwrap()
                        .borrow()
                        .iter()
                        .cloned()
                        .collect();

                    for possible_next_tail in possible_next_tails.iter() {
                        let is_colinear =
                            orientation(&head, &tail, &possible_next_tail) == Orientation::Colinear;
                        let is_forward = dot(&head, &tail, &tail, &possible_next_tail) > 0.0;

                        if is_colinear && is_forward {
                            /* Tail extension accepted */
                            remove_edge(
                                &mut head_tail_hashmap,
                                &mut tail_head_hashmap,
                                (&tail, &possible_next_tail),
                            );

                            tail = Rc::clone(&possible_next_tail);

                            did_extend = true;
                            break;
                        }
                    }
                }

                if did_extend {
                    continue;
                }

                if tail_head_hashmap.contains_key(&head) {
                    let possible_next_heads: HashSet<Rc<Vertex>> = tail_head_hashmap
                        .get(&head)
                        .unwrap()
                        .borrow()
                        .iter()
                        .cloned()
                        .collect();

                    for possible_next_head in possible_next_heads {
                        let is_colinear =
                            orientation(&tail, &head, &possible_next_head) == Orientation::Colinear;
                        let is_forward = dot(&tail, &head, &head, &possible_next_head) > 0.0;
                        if is_colinear && is_forward {
                            /* Head extension accepted */
                            remove_edge(
                                &mut tail_head_hashmap,
                                &mut head_tail_hashmap,
                                (&head, &possible_next_head),
                            );

                            head = Rc::clone(&possible_next_head);

                            did_extend = true;
                            break;
                        }
                    }
                }

                if did_extend {
                    continue;
                }

                arranged_edges.insert(Rc::new(Edge::new(&head, &tail)));
                break;
            } /* end - while remaining edges */
        }

        return arranged_edges;
    } /* end - arrange */

    /**
     * Returns the set of connecting oriented edges
     * whose composition includes the same set of points as the input edge
     * and whose orientation is also conforming. Returns None if the
     * decomposition does not exist.
     */
    pub fn decompose(base: &HashSet<Rc<Edge>>, edge: &Rc<Edge>) -> Option<Vec<Rc<Edge>>> {
        let head_tail_mapping: HashMap<Rc<Vertex>, Rc<Vertex>> = base
            .iter()
            .filter(|possible_edge| {
                let has_intersection =
                    intersection(&possible_edge.v1, &possible_edge.v2, &edge.v1, &edge.v2)
                        .is_some();

                if has_intersection {
                    if parallel(&possible_edge.v1, &possible_edge.v2, &edge.v1, &edge.v2) {
                        let has_same_orientation =
                            dot(&possible_edge.v1, &possible_edge.v2, &edge.v1, &edge.v2) > 0.0;
                        return has_same_orientation;
                    }
                }
                return false;
            })
            .cloned()
            .map(|filtered_edge| (Rc::clone(&filtered_edge.v1), Rc::clone(&filtered_edge.v2)))
            .collect();

        let mut arranged_edges: Vec<Rc<Edge>> = Vec::new();
        let head = Rc::clone(&edge.v1);
        let tail = Rc::clone(&edge.v2);
        let mut last_tail = Rc::clone(&head);

        loop {
            if last_tail == tail {
                return Some(arranged_edges);
            }
            if let Some(possible_tail) = head_tail_mapping.get(&last_tail) {
                arranged_edges.push(Rc::new(Edge::new(&last_tail, possible_tail)));
                last_tail = Rc::clone(possible_tail);
            } else {
                return None;
            }
        }
    } /* end - decompose */

    /**
     * Convert list of edges into head-tail & tail-head HashMap
     */
    pub fn into_hashmap(
        base: &HashSet<Rc<Edge>>,
    ) -> (
        HashMap<Rc<Vertex>, RefCell<HashSet<Rc<Vertex>>>>, /* head-tail hashMapping */
        HashMap<Rc<Vertex>, RefCell<HashSet<Rc<Vertex>>>>, /* tail-head hashMapping */
    ) {
        let mut head_tail_hashmap: HashMap<Rc<Vertex>, RefCell<HashSet<Rc<Vertex>>>> =
            HashMap::new();
        let mut tail_head_hashmap: HashMap<Rc<Vertex>, RefCell<HashSet<Rc<Vertex>>>> =
            HashMap::new();

        /* Populate edges head-tail tail-head mapping */
        for edge in base.iter() {
            let v1 = Rc::clone(&edge.v1);
            let v2 = Rc::clone(&edge.v2);
            match head_tail_hashmap.get(&v1) {
                Some(tail_vertices) => {
                    tail_vertices.borrow_mut().insert(Rc::clone(&v2));
                }
                None => {
                    let tail_vertices: RefCell<HashSet<Rc<Vertex>>> = RefCell::new(HashSet::new());
                    tail_vertices.borrow_mut().insert(Rc::clone(&v2));
                    head_tail_hashmap.insert(Rc::clone(&v1), tail_vertices);
                }
            }
            match tail_head_hashmap.get(&v2) {
                Some(head_vertices) => {
                    head_vertices.borrow_mut().insert(Rc::clone(&v1));
                }
                None => {
                    let head_vertices: RefCell<HashSet<Rc<Vertex>>> = RefCell::new(HashSet::new());
                    head_vertices.borrow_mut().insert(Rc::clone(&v1));
                    tail_head_hashmap.insert(Rc::clone(&v2), head_vertices);
                }
            }
        }

        return (head_tail_hashmap, tail_head_hashmap);
    } /* end - into HashMap */
} /* end - edges */

#[cfg(test)]
mod midpoint {
    use super::*;

    #[test]
    fn test_midpoint_calculation() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.2));

        let edge = Edge::new(&v1, &v2);
        let midpoint = edge.midpoint();
        assert_eq!(midpoint.x, 0.5);
        assert_eq!(midpoint.y, 0.6);
    }
}

#[cfg(test)]
mod equality {
    use super::*;

    #[test]
    fn test_different_objects() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.2));

        let e1 = Edge::new(&v1, &v2);
        let e2 = Edge::new(&v1, &v2);
        assert!(e1 == e2);

        let e1 = Rc::new(Edge::new(&v1, &v2));
        let e2 = Rc::new(Edge::new(&v1, &v2));
        assert!(e1 == e2);
    }

    #[test]
    fn test_half_edge() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.2));

        let e1 = Edge::new(&v1, &v2);
        let e2 = Edge::new(&v2, &v1);
        assert!(e1 != e2);

        let e1 = Rc::new(Edge::new(&v1, &v2));
        let e2 = Rc::new(Edge::new(&v2, &v1));
        assert!(e1 != e2);
    }
}

#[cfg(test)]
mod contains {
    use super::*;

    #[test]
    fn sample_1() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.0));

        let edge = Edge::new(&v1, &v2);

        let steps: usize = 1000;
        for index in 0..(steps + 1) {
            let coef = (v2.y - v1.y) / (v2.x - v1.x);

            let dt = coef * index as f64 / steps as f64;
            assert!(edge.contains(&Vertex::new(v1.x + dt, v1.x + dt)));
        }
    }

    #[test]
    fn sample_2() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.0));

        let edge = Edge::new(&v1, &v2);

        let steps: usize = 1000;
        for index in 0..(steps + 1) {
            let coef = (v2.y - v1.y) / (v2.x - v1.x);

            let dt = coef * index as f64 / steps as f64;
            let error = 1.0 / steps as f64;
            assert!(!edge.contains(&Vertex::new(v1.x + dt + error, v1.x + dt)));
            assert!(!edge.contains(&Vertex::new(v1.x + dt - error, v1.x + dt)));
            assert!(!edge.contains(&Vertex::new(v1.x + dt, v1.x + dt + error)));
            assert!(!edge.contains(&Vertex::new(v1.x + dt, v1.x + dt - error)));
        }
    }

    #[test]
    fn sample_3() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.0));

        let edge = Edge::new(&v1, &v2);

        assert!(!edge.contains(&Vertex::new(0.0, 1.0)));
        assert!(!edge.contains(&Vertex::new(1.0, 0.0)));

        assert!(!edge.contains(&Vertex::new(0.0, 1.1)));
        assert!(!edge.contains(&Vertex::new(1.1, 0.0)));

        assert!(!edge.contains(&Vertex::new(0.3, 0.7)));
        assert!(!edge.contains(&Vertex::new(0.7, 0.3)));
    }
}

#[cfg(test)]
mod arrange {
    use super::*;

    #[test]
    fn short_extension() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(2.0, 2.0));
        let v3 = Rc::new(Vertex::new(3.0, 3.0));

        let e1 = Rc::new(Edge::new(&v1, &v2));
        let e2 = Rc::new(Edge::new(&v2, &v3));

        let edges: HashSet<Rc<Edge>> = vec![Rc::clone(&e1), Rc::clone(&e2)]
            .iter()
            .cloned()
            .collect();

        let arranged: HashSet<Rc<Edge>> = Edge::arrange(&edges);

        assert_eq!(arranged.len(), 1);
        assert!(arranged.contains(&Rc::new(Edge::new(&v1, &v3))));
    }

    #[test]
    fn medium_extension() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(2.0, 2.0));
        let v3 = Rc::new(Vertex::new(3.0, 3.0));
        let v4 = Rc::new(Vertex::new(4.0, 4.0));
        let v5 = Rc::new(Vertex::new(5.0, 5.0));

        let e1 = Rc::new(Edge::new(&v1, &v2));
        let e2 = Rc::new(Edge::new(&v2, &v3));
        let e3 = Rc::new(Edge::new(&v3, &v4));
        let e4 = Rc::new(Edge::new(&v4, &v5));

        let edges: HashSet<Rc<Edge>> = vec![
            Rc::clone(&e1),
            Rc::clone(&e2),
            Rc::clone(&e3),
            Rc::clone(&e4),
        ]
        .iter()
        .cloned()
        .collect();

        let arranged: HashSet<Rc<Edge>> = Edge::arrange(&edges);
        assert_eq!(arranged.len(), 1);
        assert!(arranged.contains(&Rc::new(Edge::new(&v1, &v5))));
    }

    #[test]
    fn opposing_segments() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(2.0, 2.0));
        let v3 = Rc::new(Vertex::new(3.0, 3.0));
        let v4 = Rc::new(Vertex::new(4.0, 4.0));
        let v5 = Rc::new(Vertex::new(5.0, 5.0));

        /* Default Direciton */
        let e1 = Rc::new(Edge::new(&v1, &v2));
        let e2 = Rc::new(Edge::new(&v2, &v3));
        let e3 = Rc::new(Edge::new(&v3, &v4));
        let e4 = Rc::new(Edge::new(&v4, &v5));

        /* Opposed direction */
        let e6 = Rc::new(Edge::new(&v2, &v1));
        let e7 = Rc::new(Edge::new(&v3, &v2));
        let e8 = Rc::new(Edge::new(&v4, &v3));
        let e9 = Rc::new(Edge::new(&v5, &v4));

        let edges: HashSet<Rc<Edge>> = vec![
            Rc::clone(&e1),
            Rc::clone(&e2),
            Rc::clone(&e3),
            Rc::clone(&e4),
            Rc::clone(&e6),
            Rc::clone(&e7),
            Rc::clone(&e8),
            Rc::clone(&e9),
        ]
        .iter()
        .cloned()
        .collect();

        let arranged: HashSet<Rc<Edge>> = Edge::arrange(&edges);

        assert_eq!(arranged.len(), 2);
        assert!(arranged.contains(&Rc::new(Edge::new(&v1, &v5))));
        assert!(arranged.contains(&Rc::new(Edge::new(&v5, &v1))));
    }

    #[test]
    fn intercepting_edges() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(2.0, 2.0));
        let v3 = Rc::new(Vertex::new(3.0, 3.0));
        let v4 = Rc::new(Vertex::new(4.0, 4.0));
        let v5 = Rc::new(Vertex::new(5.0, 5.0));

        let v6 = Rc::new(Vertex::new(1.0, 5.0));
        let v7 = Rc::new(Vertex::new(2.0, 4.0));
        let v8 = Rc::new(Vertex::new(4.0, 2.0));
        let v9 = Rc::new(Vertex::new(5.0, 1.0));

        /* Default Direciton */
        let e1 = Rc::new(Edge::new(&v1, &v2));
        let e2 = Rc::new(Edge::new(&v2, &v3));
        let e3 = Rc::new(Edge::new(&v3, &v4));
        let e4 = Rc::new(Edge::new(&v4, &v5));

        /* Opposed direction */
        let e6 = Rc::new(Edge::new(&v2, &v1));
        let e7 = Rc::new(Edge::new(&v3, &v2));
        let e8 = Rc::new(Edge::new(&v4, &v3));
        let e9 = Rc::new(Edge::new(&v5, &v4));

        /* Crossing direction */
        let e11 = Rc::new(Edge::new(&v6, &v7));
        let e12 = Rc::new(Edge::new(&v7, &v3));
        let e13 = Rc::new(Edge::new(&v3, &v8));
        let e14 = Rc::new(Edge::new(&v8, &v9));

        let e16 = Rc::new(Edge::new(&v7, &v6));
        let e17 = Rc::new(Edge::new(&v3, &v7));
        let e18 = Rc::new(Edge::new(&v8, &v3));
        let e19 = Rc::new(Edge::new(&v9, &v8));

        let edges: HashSet<Rc<Edge>> = vec![
            Rc::clone(&e1),
            Rc::clone(&e2),
            Rc::clone(&e3),
            Rc::clone(&e4),
            Rc::clone(&e6),
            Rc::clone(&e7),
            Rc::clone(&e8),
            Rc::clone(&e9),
            Rc::clone(&e11),
            Rc::clone(&e12),
            Rc::clone(&e13),
            Rc::clone(&e14),
            Rc::clone(&e16),
            Rc::clone(&e17),
            Rc::clone(&e18),
            Rc::clone(&e19),
        ]
        .iter()
        .cloned()
        .collect();

        let arranged: HashSet<Rc<Edge>> = Edge::arrange(&edges);

        assert_eq!(arranged.len(), 4);
        assert!(arranged.contains(&Rc::new(Edge::new(&v1, &v5))));
        assert!(arranged.contains(&Rc::new(Edge::new(&v5, &v1))));
        assert!(arranged.contains(&Rc::new(Edge::new(&v6, &v9))));
        assert!(arranged.contains(&Rc::new(Edge::new(&v9, &v6))));
    }
}

#[cfg(test)]
mod decompose {
    use super::*;

    #[test]
    fn small_decomposition() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(2.0, 2.0));
        let v3 = Rc::new(Vertex::new(3.0, 3.0));

        let e1 = Rc::new(Edge::new(&v1, &v2));
        let e2 = Rc::new(Edge::new(&v2, &v3));

        let e13 = Rc::new(Edge::new(&v1, &v3));

        let base: HashSet<Rc<Edge>> = vec![Rc::clone(&e1), Rc::clone(&e2)]
            .iter()
            .cloned()
            .collect();

        let possible_decomposition = Edge::decompose(&base, &e13);
        assert!(possible_decomposition.is_some());

        if let Some(decomposition) = possible_decomposition {
            assert!(decomposition.contains(&e1));
            assert!(decomposition.contains(&e2));
        }
    }

    #[test]
    fn medium_decomposition() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(2.0, 2.0));
        let v3 = Rc::new(Vertex::new(3.0, 3.0));
        let v4 = Rc::new(Vertex::new(4.0, 4.0));
        let v5 = Rc::new(Vertex::new(5.0, 5.0));

        let e1 = Rc::new(Edge::new(&v1, &v2));
        let e2 = Rc::new(Edge::new(&v2, &v3));
        let e3 = Rc::new(Edge::new(&v3, &v4));
        let e4 = Rc::new(Edge::new(&v4, &v5));

        let testing_edge = Rc::new(Edge::new(&v2, &v4));

        let base: HashSet<Rc<Edge>> = vec![
            Rc::clone(&e1),
            Rc::clone(&e2),
            Rc::clone(&e3),
            Rc::clone(&e4),
        ]
        .iter()
        .cloned()
        .collect();

        let possible_decomposition = Edge::decompose(&base, &testing_edge);
        assert!(possible_decomposition.is_some());

        if let Some(decomposition) = possible_decomposition {
            assert_eq!(decomposition.len(), 2);
            assert!(decomposition.contains(&e2));
            assert!(decomposition.contains(&e3));
        }
    }

    #[test]
    fn none_if_not_decomposable() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(2.0, 2.0));
        let v3 = Rc::new(Vertex::new(3.0, 3.0));
        let v4 = Rc::new(Vertex::new(4.0, 4.0));
        let v5 = Rc::new(Vertex::new(5.0, 5.0));
        
        let v10 = Rc::new(Vertex::new(8.0, 8.0));

        let e1 = Rc::new(Edge::new(&v1, &v2));
        let e2 = Rc::new(Edge::new(&v2, &v3));
        let e3 = Rc::new(Edge::new(&v3, &v4));
        let e4 = Rc::new(Edge::new(&v4, &v5));

        let testing_edge = Rc::new(Edge::new(&v2, &v10));

        let base: HashSet<Rc<Edge>> = vec![
            Rc::clone(&e1),
            Rc::clone(&e2),
            Rc::clone(&e3),
            Rc::clone(&e4),
        ]
        .iter()
        .cloned()
        .collect();

        let possible_decomposition = Edge::decompose(&base, &testing_edge);
        assert!(possible_decomposition.is_none());
    }

    #[test]
    fn none_if_not_aligned() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(2.0, 2.0));
        let v3 = Rc::new(Vertex::new(0.0, 3.0));
        let v4 = Rc::new(Vertex::new(4.0, 4.0));
        let v5 = Rc::new(Vertex::new(5.0, 5.0));
        
        let e1 = Rc::new(Edge::new(&v1, &v2));
        let e2 = Rc::new(Edge::new(&v2, &v3));
        let e3 = Rc::new(Edge::new(&v3, &v4));
        let e4 = Rc::new(Edge::new(&v4, &v5));

        let testing_edge = Rc::new(Edge::new(&v1, &v5));

        let base: HashSet<Rc<Edge>> = vec![
            Rc::clone(&e1),
            Rc::clone(&e2),
            Rc::clone(&e3),
            Rc::clone(&e4),
        ]
        .iter()
        .cloned()
        .collect();

        let possible_decomposition = Edge::decompose(&base, &testing_edge);
        assert!(possible_decomposition.is_none());
    }
}
