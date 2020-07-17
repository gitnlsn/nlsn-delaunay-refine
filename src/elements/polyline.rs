use crate::elements::bounding_box::*;
use crate::elements::vertex::*;

use crate::properties::intersection::*;

use std::cmp::Ordering;
use std::collections::hash_set::HashSet;
use std::rc::Rc;

pub struct Polyline {
    pub vertices: Vec<Rc<Vertex>>,
    pub opened: bool,
}

impl Polyline {
    pub fn new_closed(vertex_list: Vec<Rc<Vertex>>) -> Option<Self> {
        if vertex_list.is_empty() || vertex_list.len() < 3 {
            return None;
        }

        return Some(Self {
            vertices: vertex_list,
            opened: false,
        });
    }

    pub fn new_opened(vertex_list: Vec<Rc<Vertex>>) -> Option<Self> {
        if vertex_list.is_empty() || vertex_list.len() < 2 {
            return None;
        }

        return Some(Self {
            vertices: vertex_list,
            opened: true,
        });
    }

    pub fn bounding_box(&self) -> Option<BoundingBox> {
        BoundingBox::from_vertices(self.vertices.iter().cloned().collect())
    }

    pub fn vertex_pairs(&self) -> Vec<(Rc<Vertex>, Rc<Vertex>)> {
        let mut pair_list: Vec<(Rc<Vertex>, Rc<Vertex>)> = Vec::new();

        for index in 0..self.vertices.len() {
            if index == self.vertices.len() && self.opened {
                /*
                   last index handles closing edge
                   skips if opened
                */
                continue;
            }

            /* getting indexes and handling closing edge */
            let v1_index = index;
            let v2_index = match index.cmp(&(self.vertices.len() - 1)) {
                Ordering::Equal => 0,
                _ => index + 1,
            };

            let v1 = self.vertices.get(v1_index).unwrap();
            let v2 = self.vertices.get(v2_index).unwrap();

            pair_list.push((Rc::clone(v1), Rc::clone(v2)));
        }

        return pair_list;
    }

    /**
     * Searchs for intersections between polylines
     */
    fn intersection(p1: &Self, p2: &Self) -> HashSet<Rc<Vertex>> {
        let mut intersection_set: HashSet<Rc<Vertex>> = HashSet::new();

        let p1_bbox = p1.bounding_box().unwrap();
        let p2_bbox = p2.bounding_box().unwrap();

        if let Some(intersection_region) = BoundingBox::intersection(&p1_bbox, &p2_bbox) {
            for (v1, v2) in p1.vertex_pairs() {
                if !intersection_region.contains(&v1) && !intersection_region.contains(&v2) {
                    /* skips if not inside intersection bounding box */
                    continue;
                }

                for (v3, v4) in p2.vertex_pairs() {
                    if !intersection_region.contains(&v3) && !intersection_region.contains(&v4) {
                        /* skips if not inside intersection bounding box */
                        continue;
                    }

                    /* calculates intersection and inserts it into the returning set */
                    if let Some(intersection_vertex) = intersection(
                        Rc::clone(&v1),
                        Rc::clone(&v2),
                        Rc::clone(&v3),
                        Rc::clone(&v4),
                    ) {
                        let intersection_vertex = Rc::new(intersection_vertex);

                        if intersection_vertex == v1 {
                            intersection_set.insert(Rc::clone(&v1));
                            continue;
                        }

                        if intersection_vertex == v2 {
                            intersection_set.insert(Rc::clone(&v2));
                            continue;
                        }

                        if intersection_vertex == v3 {
                            intersection_set.insert(Rc::clone(&v3));
                            continue;
                        }

                        if intersection_vertex == v4 {
                            intersection_set.insert(Rc::clone(&v4));
                            continue;
                        }

                        intersection_set.insert(intersection_vertex);
                    } /* end - check intersection */
                } /* end - p2 loop */
            } /* end - p1 loop */
        } /* end - p1 p2 insersection */

        return intersection_set;
    } /* end - intersection */
} /* end - impl */

#[cfg(test)]
mod polylines_intersection {
    use super::*;

    #[test]
    fn test_1() {
        let p1 = Polyline::new_closed(vec![
            Rc::new(Vertex::new(1.0, 1.0)),
            Rc::new(Vertex::new(5.0, 1.0)),
            Rc::new(Vertex::new(3.0, 5.0)),
        ])
        .unwrap();

        let p2 = Polyline::new_closed(vec![
            Rc::new(Vertex::new(3.0, 0.0)),
            Rc::new(Vertex::new(5.0, 4.0)),
            Rc::new(Vertex::new(1.0, 4.0)),
        ])
        .unwrap();

        let intersection_list = Polyline::intersection(&p1, &p2);

        assert_eq!(intersection_list.len(), 6);
    }
}
