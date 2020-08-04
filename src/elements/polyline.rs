use crate::elements::bounding_box::*;
use crate::elements::vertex::*;

use crate::properties::area::area_segments;
use crate::properties::intersection::*;
use crate::properties::orientation::Orientation;

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

        let segments = vertex_pairs(&vertex_list, false);

        if orientation(segments) == Orientation::Clockwise {
            return Some(Self {
                vertices: vertex_list.iter().cloned().rev().collect(),
                opened: false,
            });
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

    /**
     * Searchs for intersections between polylines
     */
    pub fn intersection_vertices(p1: &Self, p2: &Self) -> HashSet<Rc<Vertex>> {
        let mut intersection_set: HashSet<Rc<Vertex>> = HashSet::new();

        let p1_bbox = p1.bounding_box().unwrap();
        let p2_bbox = p2.bounding_box().unwrap();

        if let Some(intersection_region) = BoundingBox::intersection(&p1_bbox, &p2_bbox) {
            for (v1, v2) in vertex_pairs(&p1.vertices, p1.opened) {
                if !intersection_region.contains(&v1) && !intersection_region.contains(&v2) {
                    /* skips if not inside intersection bounding box */
                    continue;
                }

                for (v3, v4) in vertex_pairs(&p2.vertices, p2.opened) {
                    if !intersection_region.contains(&v3) && !intersection_region.contains(&v4) {
                        /* skips if not inside intersection bounding box */
                        continue;
                    }

                    /* calculates intersection and inserts it into the returning set */
                    if let Some(intersection_vertex) = intersection(&v1, &v2, &v3, &v4) {
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

pub fn vertex_pairs(
    vertex_list: &Vec<Rc<Vertex>>,
    opened: bool,
) -> Vec<(Rc<Vertex>, Rc<Vertex>)> {
    let mut pair_list: Vec<(Rc<Vertex>, Rc<Vertex>)> = Vec::new();

    for index in 0..(vertex_list.len() - 1) {
        let v1 = vertex_list.get(index).unwrap();
        let v2 = vertex_list.get(index + 1).unwrap();

        pair_list.push((Rc::clone(v1), Rc::clone(v2)));
    }

    if !opened {
        let v1 = vertex_list.get(vertex_list.len() - 1).unwrap();
        let v2 = vertex_list.get(0).unwrap();

        pair_list.push((Rc::clone(v1), Rc::clone(v2)));
    }

    return pair_list;
}

pub fn segments_orientation(vertex_pairs: &Vec<(Rc<Vertex>, Rc<Vertex>)>) -> Orientation {
    let area = area_segments(vertex_pairs);
    if area < 0.0 {
        return Orientation::Counterclockwise;
    }

    if area > 0.0 {
        return Orientation::Clockwise;
    }

    panic!("Not expected to have zero area");
}

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

        let intersection_list = Polyline::intersection_vertices(&p1, &p2);

        assert_eq!(intersection_list.len(), 6);
    }
}
