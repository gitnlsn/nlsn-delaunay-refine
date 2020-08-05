use crate::elements::bounding_box::*;
use crate::elements::vertex::*;

use crate::properties::angle::*;
use crate::properties::area::area_segments;
use crate::properties::continence::Continence;
use crate::properties::distance::*;
use crate::properties::dot::*;
use crate::properties::intersection::*;
use crate::properties::orientation::*;
use crate::properties::parallel::*;

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

        if segments_orientation(&segments) == Orientation::Clockwise {
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

    /**
     * Returns first vertex if polyline is opened. Returns None otherwise.
     */
    pub fn head(&self) -> Option<Rc<Vertex>> {
        if !self.opened {
            return None;
        }

        let first_vertex = self.vertices.get(0).unwrap();
        return Some(Rc::clone(first_vertex));
    }

    /**
     * Returns last vertex if polyline is opened. Returns None otherwise.
     */
    pub fn tail(&self) -> Option<Rc<Vertex>> {
        if !self.opened {
            return None;
        }

        let length = self.vertices.len();
        let last_vertex = self.vertices.get(length - 1).unwrap();
        return Some(Rc::clone(last_vertex));
    }

    pub fn bounding_box(&self) -> Option<BoundingBox> {
        BoundingBox::from_vertices(self.vertices.iter().cloned().collect())
    }

    pub fn contains(&self, vertex: &Vertex) -> Option<Continence> {
        if self.opened {
            return None;
        }

        let segments: Vec<(Rc<Vertex>, Rc<Vertex>)> = vertex_pairs(&self.vertices, self.opened)
            .iter()
            .cloned()
            .filter(|(v1, v2)| {
                if v1.x < v2.x {
                    return vertex.x >= v1.x && vertex.x <= v2.x;
                } else {
                    return vertex.x <= v1.x && vertex.x >= v2.x;
                }
            })
            .collect();

        let mut parity: f64 = 0.0;

        for (v1, v2) in segments {
            match orientation(&v1, &v2, vertex) {
                Orientation::Colinear => return Some(Continence::Boundary),
                Orientation::Counterclockwise => {
                    if v1.x == vertex.x || v2.x == vertex.x {
                        parity = parity + 0.5;
                    } else {
                        parity = parity + 1.0;
                    }
                }
                Orientation::Clockwise => {
                    if v1.x == vertex.x || v2.x == vertex.x {
                        parity = parity - 0.5;
                    } else {
                        parity = parity - 1.0;
                    }
                }
            }
        }

        if parity == 0.0 {
            return Some(Continence::Outside);
        } else {
            return Some(Continence::Inside);
        }
    }

    /**
     * Determines the intersection between two closed polylines clockwise oriented.
     * Returns a Vec of polylines that results from the intersection operation and
     * a Vec of segments that does not belong to the intersection boundary.
     */
    pub fn intersection(p1: &Self, p2: &Self) -> (Vec<Self>, HashSet<(Rc<Vertex>, Rc<Vertex>)>) {
        let mut polyline_intersection_list: Vec<Self> = Vec::new();
        let mut unused_segments: HashSet<(Rc<Vertex>, Rc<Vertex>)> = HashSet::new();

        if p1.opened || p2.opened {
            let unused_segments: HashSet<(Rc<Vertex>, Rc<Vertex>)> =
                vertex_pairs(&p1.vertices, p1.opened)
                    .iter()
                    .chain(vertex_pairs(&p2.vertices, p2.opened).iter())
                    .cloned()
                    .collect();
            return (polyline_intersection_list, unused_segments);
        }

        let p1_bbox = p1.bounding_box().unwrap();
        let p2_bbox = p2.bounding_box().unwrap();

        if let Some(intersection_region) = BoundingBox::intersection(&p1_bbox, &p2_bbox) {
            /* Selecting segments inside both polylines */
            let mut possible_segments: Vec<(Rc<Vertex>, Rc<Vertex>)> = Vec::new();

            for (v1, v2) in vertex_pairs(&p1.vertices, p1.opened) {
                /* check bouding box continence as it is a lighter operation */
                let v1v2 = Polyline::new_opened(vec![Rc::clone(&v1), Rc::clone(&v2)]).unwrap();
                if intersection_region.contains(&v1)
                    || intersection_region.contains(&v2)
                    || p2.contains(&v1).unwrap() != Continence::Outside
                    || p2.contains(&v2).unwrap() != Continence::Outside
                    || !Polyline::intersection_vertices(&p2, &v1v2).is_empty()
                {
                    /* skips if not inside intersection bounding box */
                    possible_segments.push((Rc::clone(&v1), Rc::clone(&v2)));
                    continue;
                }
                unused_segments.insert((v1, v2));
            } /* end - p1 loop */

            /* repeats for p2 agains p1 */
            for (v3, v4) in vertex_pairs(&p2.vertices, p2.opened) {
                let v3v4 = Polyline::new_opened(vec![Rc::clone(&v3), Rc::clone(&v4)]).unwrap();
                if intersection_region.contains(&v3)
                    || intersection_region.contains(&v4)
                    || p1.contains(&v3).unwrap() != Continence::Outside
                    || p1.contains(&v4).unwrap() != Continence::Outside
                    || !Polyline::intersection_vertices(&p1, &v3v4).is_empty()
                {
                    possible_segments.push((Rc::clone(&v3), Rc::clone(&v4)));
                    continue;
                }
                unused_segments.insert((v3, v4));
            } /* end - p2 loop */

            /*
                Removes pairs of colinear segments in opposed direction
            */
            let mut read_segments: Vec<(Rc<Vertex>, Rc<Vertex>)> = Vec::new();
            read_segments.push(possible_segments.pop().unwrap());
            while !possible_segments.is_empty() {
                let (v1, v2) = possible_segments.pop().unwrap();
                match read_segments.iter().position(|(v3, v4)| {
                    match intersection(&v1, &v2, v3, v4) {
                        Some(intersection_vertex) => {
                            let is_parallel = parallel(&v1, &v2, v3, v4);
                            let have_opposite_directions = dot(&v1, &v2, v3, v4) < 0.0;

                            let is_polyline_continuation = &v1 == v4 || &v2 == v3;
                            let is_outside = p1.contains(&v1).unwrap() == Continence::Outside
                                || p2.contains(&v1).unwrap() == Continence::Outside
                                || p1.contains(&v2).unwrap() == Continence::Outside
                                || p2.contains(&v2).unwrap() == Continence::Outside
                                || p1.contains(&v3).unwrap() == Continence::Outside
                                || p2.contains(&v3).unwrap() == Continence::Outside
                                || p1.contains(&v4).unwrap() == Continence::Outside
                                || p2.contains(&v4).unwrap() == Continence::Outside;

                            return is_parallel
                                && have_opposite_directions
                                && (is_polyline_continuation || is_outside);
                        }
                        None => return false,
                    }
                }) {
                    Some(index) => {
                        let (v3, v4) = read_segments.remove(index);
                        let intersection_vertex = intersection(&v1, &v2, &v3, &v4).unwrap();
                        let intersection_vertex = Rc::new(intersection_vertex);

                        unused_segments.insert((Rc::clone(&v1), Rc::clone(&v2)));
                        unused_segments.insert((Rc::clone(&v3), Rc::clone(&v4)));
                        if v2 != v3
                            && (v1 == v4
                                || p1.contains(&v1).unwrap() == Continence::Outside
                                || p2.contains(&v1).unwrap() == Continence::Outside
                                || p1.contains(&v4).unwrap() == Continence::Outside
                                || p2.contains(&v4).unwrap() == Continence::Outside)
                        {
                            possible_segments.push((Rc::clone(&v3), Rc::clone(&v2)));
                        }
                        if v1 != v4
                            && (v2 == v3
                                || p1.contains(&v2).unwrap() == Continence::Outside
                                || p2.contains(&v2).unwrap() == Continence::Outside
                                || p1.contains(&v3).unwrap() == Continence::Outside
                                || p2.contains(&v3).unwrap() == Continence::Outside)
                        {
                            possible_segments.push((Rc::clone(&v1), Rc::clone(&v4)));
                        }
                    }
                    None => {
                        read_segments.push((v1, v2));
                    }
                }
            } /* end - removes pair of intersecting colinear segments in opposed direction */

            let mut possible_segments: HashSet<(Rc<Vertex>, Rc<Vertex>)> =
                read_segments.into_iter().collect();

            while !possible_segments.is_empty() {
                let mut possible_polyline_intersection: Vec<(Rc<Vertex>, Rc<Vertex>)> = Vec::new();
                let (h1, h2) = possible_segments.iter().next().unwrap();
                let h1 = Rc::clone(h1);
                let h2 = Rc::clone(h2);
                possible_polyline_intersection.push(possible_segments.take(&(h1, h2)).unwrap());

                /* Builds polylines */
                loop {
                    let (v1, v2) = possible_polyline_intersection.last().unwrap();
                    let v1 = Rc::clone(&v1);
                    let v2 = Rc::clone(&v2);

                    let mut possible_next_segments: Vec<(Rc<Vertex>, Rc<Vertex>)> =
                        possible_segments
                            .iter()
                            .filter(|(v3, v4)| {
                                if v4 == &v1 {
                                    return false;
                                }
                                if let Some(intersection_vertex) = intersection(&v1, &v2, v3, v4) {
                                    return orientation(&v1, &intersection_vertex, v4)
                                        != Orientation::Clockwise;
                                }
                                return false;
                            })
                            .cloned()
                            .collect();

                    if possible_next_segments.is_empty() {
                        /* Check polyline closure */
                        if possible_polyline_intersection.len() > 2 {
                            let (last_v1, last_v2) = possible_polyline_intersection.last().unwrap();
                            let last_v1: Rc<Vertex> = Rc::clone(&last_v1);
                            let last_v2: Rc<Vertex> = Rc::clone(&last_v2);

                            let (head_v3, head_v4) = possible_polyline_intersection.get(0).unwrap();
                            let head_v3: Rc<Vertex> = Rc::clone(&head_v3);
                            let head_v4: Rc<Vertex> = Rc::clone(&head_v4);

                            if let Some(intersection_vertex) =
                                intersection(&last_v1, &last_v2, &head_v3, &head_v4)
                            {
                                if last_v2 != head_v3 {
                                    /* intersection occurs at the middle */
                                    let intersection_vertex = Rc::new(intersection_vertex);
                                    possible_polyline_intersection.remove(0);
                                    possible_polyline_intersection
                                        .remove(possible_polyline_intersection.len() - 1);
                                    possible_polyline_intersection.push((
                                        Rc::clone(&last_v1),
                                        Rc::clone(&intersection_vertex),
                                    ));
                                    possible_polyline_intersection.push((
                                        Rc::clone(&intersection_vertex),
                                        Rc::clone(&head_v4),
                                    ));
                                    unused_segments.insert((
                                        Rc::clone(&intersection_vertex),
                                        Rc::clone(&last_v2),
                                    ));
                                    unused_segments.insert((
                                        Rc::clone(&head_v3),
                                        Rc::clone(&intersection_vertex),
                                    ));
                                }
                                let vertices: Vec<Rc<Vertex>> = possible_polyline_intersection
                                    .iter()
                                    .map(|(last_v1, _)| Rc::clone(last_v1))
                                    .collect();

                                polyline_intersection_list
                                    .push(Self::new_closed(vertices).unwrap());
                                break;
                            }
                        } /* end - if polyline closure */
                        /* move possible_polyline to unused_segments */
                        for (v1, v2) in possible_polyline_intersection.iter() {
                            unused_segments.insert((Rc::clone(&v1), Rc::clone(&v2)));
                        }
                        break;
                    }

                    possible_next_segments.sort_by(
                        |(first_v3, first_v4), (second_v3, second_v4)| {
                            let first_intersection =
                                intersection(&v1, &v2, first_v3, first_v4).unwrap();
                            let second_intersection =
                                intersection(&v1, &v2, second_v3, second_v4).unwrap();

                            if first_intersection == second_intersection {
                                /*
                                   when it occurs, polylines have an intersection at a vertex
                                       intersection === v2 === v3
                                   we choose the one that takes the polyline to its innermost
                                */
                                let first_angle = angle(&v1, &first_intersection, first_v4);
                                let second_angle = angle(&v1, &second_intersection, first_v4);

                                return first_angle.partial_cmp(&second_angle).unwrap();
                            }

                            let first_length = distance(&v1, &first_intersection);
                            let second_length = distance(&v1, &second_intersection);

                            return first_length.partial_cmp(&second_length).unwrap();
                        },
                    );

                    /* Evaluates intersection / continuation and include new segment */
                    let (v3, v4) = possible_segments
                        .take(possible_next_segments.first().unwrap())
                        .unwrap();
                    let v3: Rc<Vertex> = Rc::clone(&v3);
                    let v4: Rc<Vertex> = Rc::clone(&v4);

                    let is_polyline_continuation = v2 == v3;
                    if is_polyline_continuation {
                        possible_polyline_intersection.push((Rc::clone(&v3), Rc::clone(&v4)));
                    } else {
                        let intersection_vertex = intersection(&v1, &v2, &v3, &v4).unwrap();
                        let intersection_vertex = Rc::new(intersection_vertex);
                        possible_polyline_intersection
                            .remove(possible_polyline_intersection.len() - 1);
                        possible_polyline_intersection
                            .push((Rc::clone(&v1), Rc::clone(&intersection_vertex)));
                        possible_polyline_intersection
                            .push((Rc::clone(&intersection_vertex), Rc::clone(&v4)));
                        unused_segments.insert((Rc::clone(&intersection_vertex), Rc::clone(&v2)));
                        unused_segments.insert((Rc::clone(&v3), Rc::clone(&intersection_vertex)));
                    }
                } /* end - loop for segments continuation */
            } /* end - loop */
        } /* end - if p1 p2 insersection boundingBox */
        return (polyline_intersection_list, unused_segments);
    }

    /**
     * Determines the union between two closed polylines clockwise oriented.
     * Returns the polyline resulting from the union operation and a Vec of
     * segments that does not belong to the union boundary. Returns None if
     * there is no intersection.
     */
    pub fn union(p1: &Self, p2: &Self) -> Option<(Self, HashSet<(Rc<Vertex>, Rc<Vertex>)>)> {
        let mut unused_segments: HashSet<(Rc<Vertex>, Rc<Vertex>)> = HashSet::new();

        if p1.opened || p2.opened {
            return None;
        }

        let p1_bbox = p1.bounding_box().unwrap();
        let p2_bbox = p2.bounding_box().unwrap();

        if let Some(intersection_region) = BoundingBox::intersection(&p1_bbox, &p2_bbox) {
            /* Selecting segments inside both polylines */
            let mut possible_segments: Vec<(Rc<Vertex>, Rc<Vertex>)> = Vec::new();

            for (v1, v2) in vertex_pairs(&p1.vertices, p1.opened) {
                /* check bouding box continence as it is a lighter operation */
                let v1v2 = Polyline::new_opened(vec![Rc::clone(&v1), Rc::clone(&v2)]).unwrap();
                if !intersection_region.contains(&v1)
                    || !intersection_region.contains(&v2)
                    || p2.contains(&v1).unwrap() == Continence::Outside
                    || p2.contains(&v2).unwrap() == Continence::Outside
                    || !Polyline::intersection_vertices(&p2, &v1v2).is_empty()
                {
                    /* skips if not inside intersection bounding box */
                    possible_segments.push((Rc::clone(&v1), Rc::clone(&v2)));
                    continue;
                }
                unused_segments.insert((v1, v2));
            } /* end - p1 loop */

            /* repeats for p2 agains p1 */
            for (v3, v4) in vertex_pairs(&p2.vertices, p2.opened) {
                let v3v4 = Polyline::new_opened(vec![Rc::clone(&v3), Rc::clone(&v4)]).unwrap();
                if !intersection_region.contains(&v3)
                    || !intersection_region.contains(&v4)
                    || p1.contains(&v3).unwrap() == Continence::Outside
                    || p1.contains(&v4).unwrap() == Continence::Outside
                    || !Polyline::intersection_vertices(&p1, &v3v4).is_empty()
                {
                    possible_segments.push((Rc::clone(&v3), Rc::clone(&v4)));
                    continue;
                }
                unused_segments.insert((v3, v4));
            } /* end - p2 loop */

            let mut possible_segments: HashSet<(Rc<Vertex>, Rc<Vertex>)> =
                possible_segments.into_iter().collect();

            /* Begins union polyline build */
            let mut possible_polyline_union: Vec<(Rc<Vertex>, Rc<Vertex>)> = Vec::new();
            let (h1, h2) = possible_segments.iter().next().unwrap();
            let h1 = Rc::clone(h1);
            let h2 = Rc::clone(h2);
            possible_polyline_union.push(possible_segments.take(&(h1, h2)).unwrap());

            /* includes segments */
            loop {
                let (v1, v2) = possible_polyline_union.last().unwrap();
                let v1 = Rc::clone(&v1);
                let v2 = Rc::clone(&v2);

                let mut possible_next_segments: Vec<(Rc<Vertex>, Rc<Vertex>)> = possible_segments
                    .iter()
                    .filter(|(v3, v4)| {
                        if v4 == &v1 || &v1 == v3 {
                            return false;
                        }
                        if let Some(intersection_vertex) = intersection(&v1, &v2, v3, v4) {
                            return orientation(&v1, &intersection_vertex, v4)
                                != Orientation::Counterclockwise
                                || &v2 == v3;
                        }
                        return false;
                    })
                    .cloned()
                    .collect();

                if possible_next_segments.is_empty() {
                    /* Check polyline closure */
                    if possible_polyline_union.len() > 2 {
                        let (last_v1, last_v2) = possible_polyline_union.last().unwrap();
                        let last_v1: Rc<Vertex> = Rc::clone(&last_v1);
                        let last_v2: Rc<Vertex> = Rc::clone(&last_v2);

                        let (head_v3, head_v4) = possible_polyline_union.get(0).unwrap();
                        let head_v3: Rc<Vertex> = Rc::clone(&head_v3);
                        let head_v4: Rc<Vertex> = Rc::clone(&head_v4);

                        if let Some(intersection_vertex) =
                            intersection(&last_v1, &last_v2, &head_v3, &head_v4)
                        {
                            if last_v2 != head_v3 {
                                /* intersection occurs at the middle */
                                let intersection_vertex = Rc::new(intersection_vertex);
                                possible_polyline_union.remove(0);
                                possible_polyline_union.remove(possible_polyline_union.len() - 1);
                                possible_polyline_union
                                    .push((Rc::clone(&last_v1), Rc::clone(&intersection_vertex)));
                                possible_polyline_union
                                    .push((Rc::clone(&intersection_vertex), Rc::clone(&head_v4)));
                                unused_segments
                                    .insert((Rc::clone(&intersection_vertex), Rc::clone(&last_v2)));
                                unused_segments
                                    .insert((Rc::clone(&head_v3), Rc::clone(&intersection_vertex)));
                            }
                            let vertices: Vec<Rc<Vertex>> = possible_polyline_union
                                .iter()
                                .map(|(last_v1, _)| Rc::clone(last_v1))
                                .collect();
                            let segments: HashSet<(Rc<Vertex>, Rc<Vertex>)> = possible_segments
                                .iter()
                                .chain(unused_segments.iter())
                                .cloned()
                                .collect();
                            return Some((Polyline::new_closed(vertices).unwrap(), segments));
                        }
                    } /* end - if polyline closure */
                    return None;
                } /* no more segments to include */

                possible_next_segments.sort_by(|(first_v3, first_v4), (second_v3, second_v4)| {
                    let first_intersection = intersection(&v1, &v2, first_v3, first_v4).unwrap();
                    let second_intersection = intersection(&v1, &v2, second_v3, second_v4).unwrap();

                    if first_intersection == second_intersection {
                        /*
                           when it occurs, polylines have an intersection at a vertex
                               intersection === v2 === v3
                           we choose the one that takes the polyline to its innermost
                        */
                        let first_angle = angle(&v1, &first_intersection, first_v4);
                        let second_angle = angle(&v1, &second_intersection, first_v4);

                        return second_angle.partial_cmp(&first_angle).unwrap();
                    }

                    let first_length = distance(&v1, &first_intersection);
                    let second_length = distance(&v1, &second_intersection);

                    return first_length.partial_cmp(&second_length).unwrap();
                });

                /* Evaluates intersection / continuation and include new segment */
                let (v3, v4) = possible_segments
                    .take(possible_next_segments.first().unwrap())
                    .unwrap();
                let v3: Rc<Vertex> = Rc::clone(&v3);
                let v4: Rc<Vertex> = Rc::clone(&v4);

                let is_polyline_continuation = v2 == v3;
                if is_polyline_continuation {
                    possible_polyline_union.push((Rc::clone(&v3), Rc::clone(&v4)));
                } else {
                    let intersection_vertex = intersection(&v1, &v2, &v3, &v4).unwrap();
                    let intersection_vertex = Rc::new(intersection_vertex);
                    possible_polyline_union.remove(possible_polyline_union.len() - 1);
                    possible_polyline_union.push((Rc::clone(&v1), Rc::clone(&intersection_vertex)));
                    possible_polyline_union.push((Rc::clone(&intersection_vertex), Rc::clone(&v4)));
                    if v2 != intersection_vertex {
                        possible_segments.insert((Rc::clone(&intersection_vertex), Rc::clone(&v2)));
                    }
                    if v3 != intersection_vertex {
                        possible_segments.insert((Rc::clone(&v3), Rc::clone(&intersection_vertex)));
                    }
                }
            } /* end - loop for segments continuation */
        } /* end - if p1 p2 insersection boundingBox */
        return None;
    }

    /**
     * Determines the subtraction between two closed polylines clockwise oriented.
     * Returns a Vec of polylines that results from the subtraction operation and
     * a Vec of Vertexes that does not belong to the polyline Vec.
     */
    pub fn subtraction(p1: &Self, p2: &Self) -> Option<(Self, Vec<Rc<Vertex>>)> {
        return None;
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
                for (v3, v4) in vertex_pairs(&p2.vertices, p2.opened) {
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

pub fn vertex_pairs(vertex_list: &Vec<Rc<Vertex>>, opened: bool) -> Vec<(Rc<Vertex>, Rc<Vertex>)> {
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
    fn intersection_vertices() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(5.0, 1.0));
        let v3 = Rc::new(Vertex::new(3.0, 5.0));

        let v4 = Rc::new(Vertex::new(3.0, 0.0));
        let v5 = Rc::new(Vertex::new(5.0, 4.0));
        let v6 = Rc::new(Vertex::new(1.0, 4.0));

        let p1 =
            Polyline::new_closed(vec![Rc::clone(&v1), Rc::clone(&v2), Rc::clone(&v3)]).unwrap();
        let p2 =
            Polyline::new_closed(vec![Rc::clone(&v4), Rc::clone(&v5), Rc::clone(&v6)]).unwrap();

        let intersection_vertices_list = Polyline::intersection_vertices(&p1, &p2);
        assert_eq!(intersection_vertices_list.len(), 6);

        let intersection_vertices_list = Polyline::intersection_vertices(
            &p1,
            &Polyline::new_opened(vec![Rc::clone(&v4), Rc::clone(&v5)]).unwrap(),
        );
        assert_eq!(intersection_vertices_list.len(), 2);

        let intersection_vertices_list = Polyline::intersection_vertices(
            &p1,
            &Polyline::new_opened(vec![Rc::clone(&v5), Rc::clone(&v6)]).unwrap(),
        );
        assert_eq!(intersection_vertices_list.len(), 2);

        let intersection_vertices_list = Polyline::intersection_vertices(
            &p1,
            &Polyline::new_opened(vec![Rc::clone(&v6), Rc::clone(&v4)]).unwrap(),
        );
        assert_eq!(intersection_vertices_list.len(), 2);
    }

    #[test]
    fn intersection_triangles_to_hexagon() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(5.0, 1.0));
        let v3 = Rc::new(Vertex::new(3.0, 5.0));

        let v4 = Rc::new(Vertex::new(3.0, 0.0));
        let v5 = Rc::new(Vertex::new(5.0, 4.0));
        let v6 = Rc::new(Vertex::new(1.0, 4.0));

        let p1 =
            Polyline::new_closed(vec![Rc::clone(&v1), Rc::clone(&v2), Rc::clone(&v3)]).unwrap();
        let p2 =
            Polyline::new_closed(vec![Rc::clone(&v4), Rc::clone(&v5), Rc::clone(&v6)]).unwrap();

        let (intersection_list, unused_segments) = Polyline::intersection(&p1, &p2);
        assert_eq!(intersection_list.len(), 1);
        assert_eq!(unused_segments.len(), 12);

        let polyline: &Polyline = intersection_list.get(0).unwrap();
        assert_eq!(polyline.vertices.len(), 6);
        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(3.5, 1.0))));
        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(4.25, 2.5))));
        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(3.5, 4.0))));
        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(2.5, 4.0))));
        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(1.75, 2.5))));
        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(2.5, 1.0))));
    }

    #[test]
    fn union_triangles_to_start() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(5.0, 1.0));
        let v3 = Rc::new(Vertex::new(3.0, 5.0));

        let v4 = Rc::new(Vertex::new(3.0, 0.0));
        let v5 = Rc::new(Vertex::new(5.0, 4.0));
        let v6 = Rc::new(Vertex::new(1.0, 4.0));

        let p1 =
            Polyline::new_closed(vec![Rc::clone(&v1), Rc::clone(&v2), Rc::clone(&v3)]).unwrap();
        let p2 =
            Polyline::new_closed(vec![Rc::clone(&v4), Rc::clone(&v5), Rc::clone(&v6)]).unwrap();

        let (union_intersection, unused_segments) = Polyline::union(&p1, &p2).unwrap();
        assert_eq!(unused_segments.len(), 6);

        assert_eq!(union_intersection.vertices.len(), 12);
        assert!(union_intersection
            .vertices
            .contains(&Rc::new(Vertex::new(3.5, 1.0))));
        assert!(union_intersection
            .vertices
            .contains(&Rc::new(Vertex::new(4.25, 2.5))));
        assert!(union_intersection
            .vertices
            .contains(&Rc::new(Vertex::new(3.5, 4.0))));
        assert!(union_intersection
            .vertices
            .contains(&Rc::new(Vertex::new(2.5, 4.0))));
        assert!(union_intersection
            .vertices
            .contains(&Rc::new(Vertex::new(1.75, 2.5))));
        assert!(union_intersection
            .vertices
            .contains(&Rc::new(Vertex::new(2.5, 1.0))));

        assert!(union_intersection
            .vertices
            .contains(&Rc::new(Vertex::new(1.0, 1.0))));
        assert!(union_intersection
            .vertices
            .contains(&Rc::new(Vertex::new(3.0, 0.0))));
        assert!(union_intersection
            .vertices
            .contains(&Rc::new(Vertex::new(5.0, 1.0))));
        assert!(union_intersection
            .vertices
            .contains(&Rc::new(Vertex::new(5.0, 4.0))));
        assert!(union_intersection
            .vertices
            .contains(&Rc::new(Vertex::new(3.0, 5.0))));
        assert!(union_intersection
            .vertices
            .contains(&Rc::new(Vertex::new(1.0, 4.0))));
    }
}
