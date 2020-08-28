use crate::elements::{bounding_box::*, edge::*, triangle::*, vertex::*};

use crate::properties::angle::*;
use crate::properties::area::area_segments;
use crate::properties::continence::*;
use crate::properties::dot::*;
use crate::properties::intersection::*;
use crate::properties::midpoint::*;
use crate::properties::orientation::*;
use crate::properties::parallel::*;

use std::collections::{HashMap, HashSet};
use std::hash::Hash;
use std::rc::Rc;

#[derive(Hash)]
pub struct Polyline {
    pub vertices: Vec<Rc<Vertex>>,
    pub opened: bool,
}

impl PartialEq for Polyline {
    fn eq(&self, other: &Self) -> bool {
        if self.opened != other.opened {
            return false;
        }

        return self.vertices == other.vertices;
    }
}

impl Eq for Polyline {}

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

    pub fn arrange(edges: &HashSet<Rc<Edge>>) -> Option<Self> {
        let mut arranged_vertices: Vec<Rc<Vertex>> = Vec::new();

        let mut hash_edges: HashMap<Rc<Vertex>, Rc<Vertex>> = HashMap::new();
        for edge in edges.iter() {
            hash_edges.insert(Rc::clone(&edge.v1), Rc::clone(&edge.v2));
        }

        let head: Rc<Vertex> = Rc::clone(hash_edges.keys().next().unwrap());
        let mut tail: Rc<Vertex> = Rc::clone(&head);
        arranged_vertices.push(Rc::clone(&head));

        loop {
            if let Some(next) = hash_edges.get(&tail) {
                if next == &head {
                    break;
                }
                arranged_vertices.push(Rc::clone(next));
                tail = Rc::clone(next);
                continue;
            }
            break;
        }

        if arranged_vertices.len() == edges.len() {
            return Some(Polyline::new_closed(arranged_vertices).unwrap());
        }

        return None;
    }

    pub fn minified_noncolinear(&self) -> Self {
        let mut minified: Vec<Rc<Vertex>> = Vec::new();
        let mut possible_vertices: Vec<Rc<Vertex>> = self.vertices.iter().cloned().collect();

        let head = Rc::clone(self.vertices.first().unwrap());
        let tail = Rc::clone(self.vertices.last().unwrap());

        if !self.opened {
            possible_vertices.insert(0, Rc::clone(&tail));
            possible_vertices.push(Rc::clone(&head));
        }

        for index in 1..(possible_vertices.len() - 1) {
            let v1: &Rc<Vertex> = possible_vertices.get(index - 1).unwrap();
            let v2: &Rc<Vertex> = possible_vertices.get(index).unwrap();
            let v3: &Rc<Vertex> = possible_vertices.get(index + 1).unwrap();

            if orientation(v1, v2, v3) != Orientation::Colinear {
                minified.push(Rc::clone(&v2));
            }
        }

        if self.opened {
            minified.insert(0, Rc::clone(&head));
            minified.push(Rc::clone(&tail));
        }

        return Polyline::new_closed(minified).unwrap();
    }

    pub fn contains(&self, vertex: &Vertex) -> Option<Continence> {
        if self.opened {
            return None;
        }

        let segments: Vec<(Rc<Vertex>, Rc<Vertex>)> = vertex_pairs(&self.vertices, self.opened);

        let mut parity: isize = 0;

        for (v1, v2) in segments {
            let is_vertical_segment = v1.x == v2.x;
            if is_vertical_segment {
                let points_to_vertex = v1.x == vertex.x;
                let contains_vertex_in_vertical_interval = (v1.y <= vertex.y && vertex.y <= v2.y)
                    || (v2.y <= vertex.y && vertex.y <= v1.y);

                if points_to_vertex && contains_vertex_in_vertical_interval {
                    /* intersection case */
                    return Some(Continence::Boundary);
                }
                /* skips vertical segments */
                continue;
            }

            /* skips segments whose x interval don't contain vertex */
            let dont_contains_vertex_in_horizontal_interval =
                (v1.x < vertex.x && v2.x < vertex.x) || (v1.x > vertex.x && v2.x > vertex.x);
            if dont_contains_vertex_in_horizontal_interval {
                continue;
            }

            match orientation(&v1, &v2, vertex) {
                Orientation::Colinear => {
                    return Some(Continence::Boundary);
                }
                Orientation::Counterclockwise => {
                    if v1.x == vertex.x || v2.x == vertex.x {
                        parity = parity + 1;
                    } else {
                        parity = parity + 2;
                    }
                }
                Orientation::Clockwise => {
                    if v1.x == vertex.x || v2.x == vertex.x {
                        parity = parity - 1;
                    } else {
                        parity = parity - 2;
                    }
                }
            }
        }

        if parity == 0 {
            return Some(Continence::Outside);
        }
        return Some(Continence::Inside);
    }

    /**
     * Determines the intersection between two closed polylines clockwise oriented.
     * Returns a Vec of polylines that results from the intersection operation and
     * a Vec of segments that does not belong to the intersection boundary.
     */
    pub fn intersection(p1: &Self, p2: &Self) -> (Vec<Self>, HashSet<(Rc<Vertex>, Rc<Vertex>)>) {
        let mut polyline_intersection_list: Vec<Self> = Vec::new();
        let mut unused_segments: HashSet<(Rc<Vertex>, Rc<Vertex>)> = HashSet::new();

        let p1_segments = vertex_pairs(&p1.vertices, p1.opened);
        let p2_segments = vertex_pairs(&p2.vertices, p2.opened);
        /* splits segments at the beginning makes it easy to avoid outer boundary  */
        let mut possible_segments: Vec<(Rc<Vertex>, Rc<Vertex>)> = split_intersections(
            &p1_segments
                .iter()
                .chain(p2_segments.iter())
                .cloned()
                .collect(),
        );

        if p1.opened || p2.opened {
            let unused_segments: HashSet<(Rc<Vertex>, Rc<Vertex>)> =
                possible_segments.iter().cloned().collect();
            return (polyline_intersection_list, unused_segments);
        }

        let p1_bbox = p1.bounding_box().unwrap();
        let p2_bbox = p2.bounding_box().unwrap();

        if !BoundingBox::intersection(&p1_bbox, &p2_bbox).is_none() {
            /*
                Removes pairs of colinear segments in opposed direction
            */
            let mut read_segments: Vec<(Rc<Vertex>, Rc<Vertex>)> = Vec::new();
            read_segments.push(possible_segments.pop().unwrap());
            while !possible_segments.is_empty() {
                let (v1, v2) = possible_segments.pop().unwrap();
                match read_segments.iter().position(|(v3, v4)| {
                    if !intersection(&v1, &v2, v3, v4).is_none() {
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
                    return false;
                }) {
                    Some(index) => {
                        let (v3, v4) = read_segments.remove(index);

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
            possible_segments = read_segments.iter().cloned().collect();
            read_segments = Vec::new();

            /* Filters by continence */
            while !possible_segments.is_empty() {
                let (v1, v2) = possible_segments.pop().unwrap();
                let midpoint = midpoint(&v1, &v2);

                let contains_mid = p1.contains(&midpoint).unwrap() != Continence::Outside
                    && p2.contains(&midpoint).unwrap() != Continence::Outside;

                if contains_mid {
                    read_segments.push((v1, v2));
                } else {
                    unused_segments.insert((v1, v2));
                }
            }
            let mut possible_segments: HashSet<(Rc<Vertex>, Rc<Vertex>)> =
                read_segments.into_iter().collect();

            /* Builds polylines */
            while !possible_segments.is_empty() {
                let mut possible_polyline_intersection: Vec<(Rc<Vertex>, Rc<Vertex>)> = Vec::new();
                let (h1, h2) = possible_segments.iter().next().unwrap();
                let h1 = Rc::clone(h1);
                let h2 = Rc::clone(h2);
                possible_polyline_intersection.push(possible_segments.take(&(h1, h2)).unwrap());

                loop {
                    let (v1, v2) = possible_polyline_intersection.last().unwrap();
                    let v1 = Rc::clone(&v1);
                    let v2 = Rc::clone(&v2);

                    let mut possible_next_segments: Vec<(Rc<Vertex>, Rc<Vertex>)> =
                        possible_segments
                            .iter()
                            .filter(|(v3, v4)| {
                                /* avoid segments wrong continuation */
                                if &v1 == v3 || &v1 == v4 || &v2 == v4 {
                                    return false;
                                }
                                /* segments continuation: v1->v2 v3->v4, where v2 === v3 */
                                return &v2 == v3;
                            })
                            .cloned()
                            .collect();

                    if possible_next_segments.is_empty() {
                        /* Check polyline closure */
                        if possible_polyline_intersection.len() > 2 {
                            let (_, last_v2) = possible_polyline_intersection.last().unwrap();
                            let (head_v3, _) = possible_polyline_intersection.get(0).unwrap();

                            let last_v2: Rc<Vertex> = Rc::clone(&last_v2);
                            let head_v3: Rc<Vertex> = Rc::clone(&head_v3);

                            if last_v2 == head_v3 {
                                let vertices: Vec<Rc<Vertex>> = possible_polyline_intersection
                                    .iter()
                                    .map(|(last_v1, _)| Rc::clone(last_v1))
                                    .collect();

                                polyline_intersection_list
                                    .push(Self::new_closed(vertices).unwrap());
                                break;
                            }
                        } /* end - if minimal length */
                        for (v1, v2) in possible_polyline_intersection.iter() {
                            unused_segments.insert((Rc::clone(&v1), Rc::clone(&v2)));
                        }
                        break;
                    }

                    possible_next_segments.sort_by(|(_, first_v4), (_, second_v4)| {
                        let first_angle = angle(&v1, &v2, first_v4);
                        let second_angle = angle(&v1, &v2, second_v4);

                        return first_angle.partial_cmp(&second_angle).unwrap();
                    });

                    /* Evaluates include new segment by continuation */
                    let (v3, v4) = possible_segments
                        .take(possible_next_segments.first().unwrap())
                        .unwrap();
                    let v3: Rc<Vertex> = Rc::clone(&v3);
                    let v4: Rc<Vertex> = Rc::clone(&v4);
                    possible_polyline_intersection.push((Rc::clone(&v3), Rc::clone(&v4)));
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

        if BoundingBox::intersection(&p1_bbox, &p2_bbox).is_none() {
            return None;
        }

        /* splits segments at the beginning makes it easy to avoid outer boundary  */
        let p1_segments = vertex_pairs(&p1.vertices, p1.opened);
        let p2_segments = vertex_pairs(&p2.vertices, p2.opened);
        let mut possible_segments: Vec<(Rc<Vertex>, Rc<Vertex>)> = split_intersections(
            &p1_segments
                .iter()
                .chain(p2_segments.iter())
                .cloned()
                .collect(),
        );
        let mut read_segments: Vec<(Rc<Vertex>, Rc<Vertex>)> = Vec::new();

        /* Filters by continence */
        while !possible_segments.is_empty() {
            let (v1, v2) = possible_segments.pop().unwrap();
            let midpoint = midpoint(&v1, &v2);

            let dont_contains_mid = p1.contains(&midpoint).unwrap() == Continence::Outside
                || p2.contains(&midpoint).unwrap() == Continence::Outside;

            if dont_contains_mid {
                read_segments.push((v1, v2));
            } else {
                unused_segments.insert((v1, v2));
            }
        }

        let mut possible_segments: HashSet<(Rc<Vertex>, Rc<Vertex>)> =
            read_segments.iter().cloned().collect();

        while !possible_segments.is_empty() {
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
                        if &v1 == v3 || &v1 == v4 || &v2 == v4 {
                            return false;
                        }
                        return &v2 == v3;
                    })
                    .cloned()
                    .collect();

                if possible_next_segments.is_empty() {
                    /* Check polyline closure */
                    if possible_polyline_union.len() > 2 {
                        let (_, last_v2) = possible_polyline_union.last().unwrap();
                        let (head_v3, _) = possible_polyline_union.get(0).unwrap();

                        let last_v2: Rc<Vertex> = Rc::clone(&last_v2);
                        let head_v3: Rc<Vertex> = Rc::clone(&head_v3);

                        if last_v2 == head_v3
                            && segments_orientation(&possible_polyline_union)
                                == Orientation::Counterclockwise
                        {
                            let vertices: Vec<Rc<Vertex>> = possible_polyline_union
                                .iter()
                                .map(|(v1, _)| Rc::clone(v1))
                                .collect();
                            let segments: HashSet<(Rc<Vertex>, Rc<Vertex>)> = possible_segments
                                .iter()
                                .chain(unused_segments.iter())
                                .cloned()
                                .collect();
                            return Some((Polyline::new_closed(vertices).unwrap(), segments));
                        }
                    } /* end - if polyline closure */
                    unused_segments = unused_segments
                        .iter()
                        .chain(possible_polyline_union.iter())
                        .cloned()
                        .collect();
                    break;
                } /* no more segments to include */

                possible_next_segments.sort_by(|(_, first_v4), (_, second_v4)| {
                    let first_angle = angle(&v1, &v2, first_v4);
                    let second_angle = angle(&v1, &v2, second_v4);

                    return second_angle.partial_cmp(&first_angle).unwrap();
                });

                /* Evaluates intersection / continuation and include new segment */
                let (v3, v4) = possible_segments
                    .take(possible_next_segments.first().unwrap())
                    .unwrap();
                let v3: Rc<Vertex> = Rc::clone(&v3);
                let v4: Rc<Vertex> = Rc::clone(&v4);

                possible_polyline_union.push((Rc::clone(&v3), Rc::clone(&v4)));
            } /* end - loop for segments continuation */
        } /* end - while possible segments is not empty */
        return None;
    }

    /**
     * Determines the subtraction between two closed polylines counterclockwise
     * oriented. Returns a Vec of polylines that results from the subtraction
     * operation and a Vec of segments that does not belong to the result.
     */
    pub fn subtraction(p1: &Self, p2: &Self) -> (Vec<Self>, HashSet<(Rc<Vertex>, Rc<Vertex>)>) {
        let mut polyline_intersection_list: Vec<Self> = Vec::new();
        let mut unused_segments: HashSet<(Rc<Vertex>, Rc<Vertex>)> = HashSet::new();

        let p1_segments = vertex_pairs(&p1.vertices, p1.opened);
        let p2_segments = vertex_pairs(&p2.vertices.iter().cloned().rev().collect(), p2.opened);
        /* splits segments at the beginning makes it easy to avoid outer boundary  */
        let mut possible_segments: Vec<(Rc<Vertex>, Rc<Vertex>)> = split_intersections(
            &p1_segments
                .iter()
                .chain(p2_segments.iter())
                .cloned()
                .collect(),
        );

        let p1_bbox = p1.bounding_box().unwrap();
        let p2_bbox = p2.bounding_box().unwrap();
        let no_intersection_area = BoundingBox::intersection(&p1_bbox, &p2_bbox).is_none();

        if p1.opened || p2.opened || no_intersection_area {
            let unused_segments: HashSet<(Rc<Vertex>, Rc<Vertex>)> =
                possible_segments.iter().cloned().collect();
            return (polyline_intersection_list, unused_segments);
        }

        /*
            Removes pairs of colinear segments in opposed direction
        */
        let mut read_segments: Vec<(Rc<Vertex>, Rc<Vertex>)> = Vec::new();
        read_segments.push(possible_segments.pop().unwrap());
        while !possible_segments.is_empty() {
            let (v1, v2) = possible_segments.pop().unwrap();
            match read_segments.iter().position(|(v3, v4)| {
                if intersection(&v1, &v2, v3, v4).is_none() {
                    return false;
                }
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
            }) {
                Some(index) => {
                    let (v3, v4) = read_segments.remove(index);

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
        possible_segments = read_segments.iter().cloned().collect();
        read_segments = Vec::new();

        /* Filters by continence */
        while !possible_segments.is_empty() {
            let (v1, v2) = possible_segments.pop().unwrap();
            let midpoint = midpoint(&v1, &v2);

            let inside_p1 = p1.contains(&midpoint).unwrap() != Continence::Outside;
            let not_inside_p2 = p2.contains(&midpoint).unwrap() != Continence::Inside;

            if inside_p1 && not_inside_p2 {
                read_segments.push((v1, v2));
            } else {
                unused_segments.insert((v1, v2));
            }
        }
        let mut possible_segments: HashSet<(Rc<Vertex>, Rc<Vertex>)> =
            read_segments.into_iter().collect();

        /* Builds polylines */
        while !possible_segments.is_empty() {
            let mut possible_polyline_subtraction: Vec<(Rc<Vertex>, Rc<Vertex>)> = Vec::new();
            let (h1, h2) = possible_segments.iter().next().unwrap();
            let h1 = Rc::clone(h1);
            let h2 = Rc::clone(h2);
            possible_polyline_subtraction.push(possible_segments.take(&(h1, h2)).unwrap());

            loop {
                let (v1, v2) = possible_polyline_subtraction.last().unwrap();
                let v1 = Rc::clone(&v1);
                let v2 = Rc::clone(&v2);

                let mut possible_next_segments: Vec<(Rc<Vertex>, Rc<Vertex>)> = possible_segments
                    .iter()
                    .filter(|(v3, v4)| {
                        /* avoid segments wrong continuation */
                        if &v1 == v3 || &v1 == v4 || &v2 == v4 {
                            return false;
                        }
                        /* segments continuation: v1->v2 v3->v4, where v2 === v3 */
                        return &v2 == v3;
                    })
                    .cloned()
                    .collect();

                if possible_next_segments.is_empty() {
                    /* Check polyline closure */
                    if possible_polyline_subtraction.len() > 2 {
                        let (_, last_v2) = possible_polyline_subtraction.last().unwrap();
                        let (head_v3, _) = possible_polyline_subtraction.get(0).unwrap();

                        let last_v2: Rc<Vertex> = Rc::clone(&last_v2);
                        let head_v3: Rc<Vertex> = Rc::clone(&head_v3);

                        if last_v2 == head_v3 {
                            let vertices: Vec<Rc<Vertex>> = possible_polyline_subtraction
                                .iter()
                                .map(|(last_v1, _)| Rc::clone(last_v1))
                                .collect();

                            polyline_intersection_list.push(Self::new_closed(vertices).unwrap());
                            break;
                        }
                    } /* end - if minimal length */
                    for (v1, v2) in possible_polyline_subtraction.iter() {
                        unused_segments.insert((Rc::clone(&v1), Rc::clone(&v2)));
                    }
                    break;
                }

                possible_next_segments.sort_by(|(_, first_v4), (_, second_v4)| {
                    let first_angle = angle(&v1, &v2, first_v4);
                    let second_angle = angle(&v1, &v2, second_v4);

                    return first_angle.partial_cmp(&second_angle).unwrap();
                });

                /* Evaluates include new segment by continuation */
                let (v3, v4) = possible_segments
                    .take(possible_next_segments.first().unwrap())
                    .unwrap();
                let v3: Rc<Vertex> = Rc::clone(&v3);
                let v4: Rc<Vertex> = Rc::clone(&v4);
                possible_polyline_subtraction.push((Rc::clone(&v3), Rc::clone(&v4)));
            } /* end - loop for segments continuation */
        } /* end - loop */
        return (polyline_intersection_list, unused_segments);
    } /* end - subtraction */

    /**
     * Evaluate continece between polylines
     * Returns Continence value if all vertices of p2 are single sided
     * agains p1, be it Inside, Outside. Returns None if continence is
     * not consistent or if intersection occurs or if p1 is opened.
     */
    pub fn continence(p1: &Self, p2: &Self) -> Option<(Continence, BoundaryInclusion)> {
        if p1.opened {
            return None;
        }

        let mut possible_continence: Option<Continence> = None;
        let mut possible_boundary: BoundaryInclusion = BoundaryInclusion::Open;

        for critial_vertex in p2
            .into_edges()
            .iter()
            .map(|e| vec![Rc::new(e.midpoint()), Rc::clone(&e.v1)])
            .flatten()
        {
            let continence = p1.contains(&critial_vertex).unwrap();
            if continence == Continence::Boundary {
                if possible_boundary == BoundaryInclusion::Open {
                    possible_boundary = BoundaryInclusion::Closed;
                }
            } else {
                if possible_continence.is_none() {
                    possible_continence = Some(continence);
                    continue;
                }
                if possible_continence != Some(continence) {
                    return None;
                }
            }
        }

        let p1_pairs = vertex_pairs(&p1.vertices, p1.opened);
        let p2_pairs = vertex_pairs(&p2.vertices, p2.opened);

        let splited_edges = Edge::from_vertex_pairs(split_intersections(
            &p1_pairs.iter().chain(p2_pairs.iter()).cloned().collect(),
        ));

        for edge in splited_edges.iter() {
            let critial_vertex = Rc::new(edge.midpoint());
            let continence = p1.contains(&critial_vertex).unwrap();
            if continence == Continence::Boundary {
                continue;
            }
            if Some(continence) != possible_continence {
                return None;
            }
        }

        if possible_boundary == BoundaryInclusion::Closed && possible_continence.is_none() {
            return Some((Continence::Boundary, BoundaryInclusion::Closed));
        }
        return Some((possible_continence.unwrap(), possible_boundary));
    } /* end - continence */

    /**
     * Searchs for intersections between polylines
     */
    pub fn intersection_vertices(p1: &Self, p2: &Self) -> HashSet<Rc<Vertex>> {
        let mut intersection_set: HashSet<Rc<Vertex>> = HashSet::new();

        let p1_bbox = p1.bounding_box().unwrap();
        let p2_bbox = p2.bounding_box().unwrap();

        if !BoundingBox::intersection(&p1_bbox, &p2_bbox).is_none() {
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
    } /* end - intersection vertices */

    pub fn into_edges(&self) -> Vec<Rc<Edge>> {
        vertex_pairs(&self.vertices, self.opened)
            .iter()
            .map(|(v1, v2)| Rc::new(Edge::new(v1, v2)))
            .collect::<Vec<Rc<Edge>>>()
    }

    /**
     * Detemines the hull that defines the boundary of the triangles set.
     * If the triangles are adjacent in-between 2-by-2 and occupies a single
     * continuous domain, the hull is returned. Else returns None.
     */
    pub fn triangles_hull(triangles: &HashSet<Rc<Triangle>>) -> Option<Self> {
        let mut aux_segments: HashSet<Rc<Edge>> = triangles
            .iter()
            .map(|t| t.inner_edges())
            .map(|(e1, e2, e3)| vec![e1, e2, e3])
            .flatten()
            .collect();

        let mut boundary_edges: HashMap<Rc<Edge>, Rc<Edge>> = HashMap::new();
        while !aux_segments.is_empty() {
            let possible_segment = Rc::clone(aux_segments.iter().next().unwrap());
            aux_segments.remove(&possible_segment);

            if boundary_edges.contains_key(&possible_segment) {
                boundary_edges.remove(&possible_segment);
                continue;
            }

            boundary_edges.insert(Rc::new(possible_segment.opposite()), possible_segment);
        }
        let boundary_edges = boundary_edges.values().cloned().collect();

        return Self::arrange(&boundary_edges);
    }
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

pub fn split_intersections(
    segments: &Vec<(Rc<Vertex>, Rc<Vertex>)>,
) -> Vec<(Rc<Vertex>, Rc<Vertex>)> {
    let mut splited_segments: Vec<(Rc<Vertex>, Rc<Vertex>)> = Vec::new();
    let mut aux_set: Vec<(Rc<Vertex>, Rc<Vertex>)> = segments.iter().cloned().collect();

    splited_segments.push(aux_set.pop().unwrap());
    while !aux_set.is_empty() {
        let (v1, v2) = aux_set.pop().unwrap();
        if let Some(index) = splited_segments.iter().position(|(v3, v4)| {
            if !intersection(&v1, &v2, &v3, &v4).is_none() {
                return &v1 != v3
                    && &v1 != v4
                    && &v2 != v3
                    && &v2 != v4
                    && !parallel(&v1, &v2, &v3, &v4);
            }
            return false;
        }) {
            let (v3, v4) = splited_segments.remove(index);
            let intersection_vertex = intersection(&v1, &v2, &v3, &v4).unwrap();
            let intersection_vertex = Rc::new(intersection_vertex);
            if v3 != intersection_vertex {
                aux_set.push((Rc::clone(&v3), Rc::clone(&intersection_vertex)));
            }
            if v4 != intersection_vertex {
                aux_set.push((Rc::clone(&intersection_vertex), Rc::clone(&v4)));
            }
            if v1 != intersection_vertex {
                aux_set.push((Rc::clone(&v1), Rc::clone(&intersection_vertex)));
            }
            if v2 != intersection_vertex {
                aux_set.push((Rc::clone(&intersection_vertex), Rc::clone(&v2)));
            }
        } else {
            /* no intersection, just segment continuation */
            splited_segments.push((Rc::clone(&v1), Rc::clone(&v2)));
        }
    }

    return splited_segments;
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
mod intersection_vertices {
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
}

#[cfg(test)]
mod continence {
    use super::*;

    #[test]
    fn exception_case_1() {
        let v1 = Rc::new(Vertex::new(1.0, 2.0));
        let v2 = Rc::new(Vertex::new(2.0, 2.0));
        let v3 = Rc::new(Vertex::new(2.0, 3.0));
        let v4 = Rc::new(Vertex::new(1.0, 3.0));

        let v5 = Rc::new(Vertex::new(3.0, 1.0));
        let v1v5_mid = midpoint(&v1, &v5);
        let v5v3_mid = midpoint(&v5, &v3);

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ])
        .unwrap();

        let p2 =
            Polyline::new_closed(vec![Rc::clone(&v5), Rc::clone(&v3), Rc::clone(&v1)]).unwrap();

        assert_eq!(p1.contains(&v1), Some(Continence::Boundary));
        assert_eq!(p1.contains(&v1v5_mid), Some(Continence::Outside));
        assert_eq!(p1.contains(&v5), Some(Continence::Outside));

        assert_eq!(p2.contains(&v1v5_mid), Some(Continence::Boundary));
        assert_eq!(p2.contains(&v1), Some(Continence::Boundary));
        assert_eq!(p2.contains(&v5v3_mid), Some(Continence::Boundary));
    }

    #[test]
    fn exception_case_2() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(5.0, 1.0));
        let v3 = Rc::new(Vertex::new(3.0, 5.0));

        let v4 = Rc::new(Vertex::new(3.0, 0.0));
        let v5 = Rc::new(Vertex::new(5.0, 4.0));
        let v6 = Rc::new(Vertex::new(1.0, 4.0));

        let v7 = Rc::new(Vertex::new(2.5, 4.0));
        let v8 = Rc::new(Vertex::new(1.75, 2.5));
        let v7v8_mid = midpoint(&v7, &v8);

        let p1 =
            Polyline::new_closed(vec![Rc::clone(&v1), Rc::clone(&v2), Rc::clone(&v3)]).unwrap();
        let p2 =
            Polyline::new_closed(vec![Rc::clone(&v4), Rc::clone(&v5), Rc::clone(&v6)]).unwrap();

        assert_eq!(p1.contains(&v7v8_mid), Some(Continence::Boundary));
        assert_eq!(p2.contains(&v7v8_mid), Some(Continence::Inside));
    }

    #[test]
    fn exception_case_3() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(5.0, 1.0));
        let v3 = Rc::new(Vertex::new(3.0, 5.0));

        let v4 = Rc::new(Vertex::new(3.0, 0.0));
        let v5 = Rc::new(Vertex::new(5.0, 4.0));
        let v6 = Rc::new(Vertex::new(1.0, 4.0));

        let v7 = Rc::new(Vertex::new(2.5, 1.0));
        let v8 = Rc::new(Vertex::new(3.0, 0.0));
        let v7v8_mid = midpoint(&v7, &v8);

        let p1 =
            Polyline::new_closed(vec![Rc::clone(&v1), Rc::clone(&v2), Rc::clone(&v3)]).unwrap();
        let p2 =
            Polyline::new_closed(vec![Rc::clone(&v4), Rc::clone(&v5), Rc::clone(&v6)]).unwrap();

        assert_eq!(p1.contains(&v7v8_mid), Some(Continence::Outside));
        assert_eq!(p2.contains(&v7v8_mid), Some(Continence::Boundary));

        assert_eq!(p1.contains(&v7), Some(Continence::Boundary));
        assert_eq!(p2.contains(&v7), Some(Continence::Boundary));

        assert_eq!(p1.contains(&v8), Some(Continence::Outside));
        assert_eq!(p2.contains(&v8), Some(Continence::Boundary));
    }

    #[test]
    fn exception_case_4() {
        let v1 = Rc::new(Vertex::new(1.0, 2.0));
        let v2 = Rc::new(Vertex::new(4.0, 2.0));
        let v3 = Rc::new(Vertex::new(4.0, 3.0));
        let v4 = Rc::new(Vertex::new(2.0, 3.0));
        let v5 = Rc::new(Vertex::new(2.0, 4.0));
        let v6 = Rc::new(Vertex::new(4.0, 4.0));
        let v7 = Rc::new(Vertex::new(4.0, 5.0));
        let v8 = Rc::new(Vertex::new(1.0, 5.0));

        let v9 = Rc::new(Vertex::new(3.0, 1.0));
        let v10 = Rc::new(Vertex::new(5.0, 1.0));
        let v11 = Rc::new(Vertex::new(5.0, 6.0));
        let v12 = Rc::new(Vertex::new(3.0, 6.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ])
        .unwrap();
        let p2 = Polyline::new_closed(vec![
            Rc::clone(&v9),
            Rc::clone(&v10),
            Rc::clone(&v11),
            Rc::clone(&v12),
        ])
        .unwrap();

        let v7 = Rc::new(Vertex::new(5.0, 6.0));
        let v8 = Rc::new(Vertex::new(3.0, 6.0));
        let v7v8_mid = midpoint(&v7, &v8);

        assert_eq!(p1.contains(&v7v8_mid), Some(Continence::Outside));
        assert_eq!(p2.contains(&v7v8_mid), Some(Continence::Boundary));

        assert_eq!(p1.contains(&v7), Some(Continence::Outside));
        assert_eq!(p2.contains(&v7), Some(Continence::Boundary));

        assert_eq!(p1.contains(&v8), Some(Continence::Outside));
        assert_eq!(p2.contains(&v8), Some(Continence::Boundary));
    }
}

#[cfg(test)]
mod intersection {
    use super::*;

    #[test]
    fn triangles_to_hexagon() {
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
    fn two_squares() {
        let v1 = Rc::new(Vertex::new(2.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 3.0));
        let v4 = Rc::new(Vertex::new(2.0, 3.0));

        let v5 = Rc::new(Vertex::new(1.0, 2.0));
        let v6 = Rc::new(Vertex::new(3.0, 2.0));
        let v7 = Rc::new(Vertex::new(3.0, 4.0));
        let v8 = Rc::new(Vertex::new(1.0, 4.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ])
        .unwrap();
        let p2 = Polyline::new_closed(vec![
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ])
        .unwrap();

        let (intersection_list, unused_segments) = Polyline::intersection(&p1, &p2);
        assert_eq!(intersection_list.len(), 1);
        assert_eq!(unused_segments.len(), 8);

        let polyline: &Polyline = intersection_list.get(0).unwrap();
        assert_eq!(polyline.vertices.len(), 4);
        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(2.0, 2.0))));
        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(3.0, 2.0))));
        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(3.0, 3.0))));
        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(2.0, 3.0))));
    }

    #[test]
    fn intersection_at_the_vertex() {
        let v1 = Rc::new(Vertex::new(1.0, 2.0));
        let v2 = Rc::new(Vertex::new(2.0, 2.0));
        let v3 = Rc::new(Vertex::new(2.0, 3.0));
        let v4 = Rc::new(Vertex::new(1.0, 3.0));

        let v5 = Rc::new(Vertex::new(3.0, 1.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ])
        .unwrap();
        let p2 =
            Polyline::new_closed(vec![Rc::clone(&v5), Rc::clone(&v3), Rc::clone(&v1)]).unwrap();

        let (intersection_list, unused_segments) = Polyline::intersection(&p1, &p2);
        assert_eq!(intersection_list.len(), 1);
        assert_eq!(unused_segments.len(), 4);

        let polyline: &Polyline = intersection_list.get(0).unwrap();
        assert_eq!(polyline.vertices.len(), 3);

        assert!(polyline.vertices.contains(&v1));
        assert!(polyline.vertices.contains(&v2));
        assert!(polyline.vertices.contains(&v3));
    }

    #[test]
    fn double_intersection() {
        let v1 = Rc::new(Vertex::new(1.0, 2.0));
        let v2 = Rc::new(Vertex::new(4.0, 2.0));
        let v3 = Rc::new(Vertex::new(4.0, 3.0));
        let v4 = Rc::new(Vertex::new(2.0, 3.0));
        let v5 = Rc::new(Vertex::new(2.0, 4.0));
        let v6 = Rc::new(Vertex::new(4.0, 4.0));
        let v7 = Rc::new(Vertex::new(4.0, 5.0));
        let v8 = Rc::new(Vertex::new(1.0, 5.0));

        let v9 = Rc::new(Vertex::new(3.0, 1.0));
        let v10 = Rc::new(Vertex::new(5.0, 1.0));
        let v11 = Rc::new(Vertex::new(5.0, 6.0));
        let v12 = Rc::new(Vertex::new(3.0, 6.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ])
        .unwrap();
        let p2 = Polyline::new_closed(vec![
            Rc::clone(&v9),
            Rc::clone(&v10),
            Rc::clone(&v11),
            Rc::clone(&v12),
        ])
        .unwrap();

        let (intersection_list, unused_segments) = Polyline::intersection(&p1, &p2);

        assert_eq!(intersection_list.len(), 2);
        assert_eq!(unused_segments.len(), 12);

        let polyline_1: &Polyline = intersection_list.get(0).unwrap();
        let polyline_2: &Polyline = intersection_list.get(1).unwrap();
        assert_eq!(polyline_1.vertices.len(), 4);
        assert_eq!(polyline_2.vertices.len(), 4);

        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(3.0, 6.0)),
            Rc::new(Vertex::new(3.0, 5.0))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(3.0, 5.0)),
            Rc::new(Vertex::new(1.0, 5.0))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(2.0, 4.0)),
            Rc::new(Vertex::new(3.0, 4.0))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(2.0, 3.0)),
            Rc::new(Vertex::new(2.0, 4.0))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(1.0, 5.0)),
            Rc::new(Vertex::new(1.0, 2.0))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(5.0, 6.0)),
            Rc::new(Vertex::new(3.0, 6.0))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(3.0, 4.0)),
            Rc::new(Vertex::new(3.0, 3.0))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(3.0, 3.0)),
            Rc::new(Vertex::new(2.0, 3.0))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(3.0, 2.0)),
            Rc::new(Vertex::new(3.0, 1.0))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(1.0, 2.0)),
            Rc::new(Vertex::new(3.0, 2.0))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(5.0, 1.0)),
            Rc::new(Vertex::new(5.0, 6.0))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(3.0, 1.0)),
            Rc::new(Vertex::new(5.0, 1.0))
        )));
    }
} /* end - intersection tests */

#[cfg(test)]
mod union {
    use super::*;

    #[test]
    fn triangles_to_star() {
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

        let (union, unused_segments) = Polyline::union(&p1, &p2).unwrap();
        assert_eq!(unused_segments.len(), 6);

        assert_eq!(union.vertices.len(), 12);
        assert!(union.vertices.contains(&Rc::new(Vertex::new(3.5, 1.0))));
        assert!(union.vertices.contains(&Rc::new(Vertex::new(4.25, 2.5))));
        assert!(union.vertices.contains(&Rc::new(Vertex::new(3.5, 4.0))));
        assert!(union.vertices.contains(&Rc::new(Vertex::new(2.5, 4.0))));
        assert!(union.vertices.contains(&Rc::new(Vertex::new(1.75, 2.5))));
        assert!(union.vertices.contains(&Rc::new(Vertex::new(2.5, 1.0))));

        assert!(union.vertices.contains(&Rc::new(Vertex::new(1.0, 1.0))));
        assert!(union.vertices.contains(&Rc::new(Vertex::new(3.0, 0.0))));
        assert!(union.vertices.contains(&Rc::new(Vertex::new(5.0, 1.0))));
        assert!(union.vertices.contains(&Rc::new(Vertex::new(5.0, 4.0))));
        assert!(union.vertices.contains(&Rc::new(Vertex::new(3.0, 5.0))));
        assert!(union.vertices.contains(&Rc::new(Vertex::new(1.0, 4.0))));
    }

    #[test]
    fn two_squares() {
        let v1 = Rc::new(Vertex::new(1.0, 2.0));
        let v2 = Rc::new(Vertex::new(3.0, 2.0));
        let v3 = Rc::new(Vertex::new(3.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let v5 = Rc::new(Vertex::new(2.0, 1.0));
        let v6 = Rc::new(Vertex::new(4.0, 1.0));
        let v7 = Rc::new(Vertex::new(4.0, 3.0));
        let v8 = Rc::new(Vertex::new(2.0, 3.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ])
        .unwrap();

        let p2 = Polyline::new_closed(vec![
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ])
        .unwrap();

        let (union, unused_segments) = Polyline::union(&p1, &p2).unwrap();
        assert_eq!(unused_segments.len(), 4);

        assert_eq!(union.vertices.len(), 8);
        assert!(union.vertices.contains(&v1));
        assert!(union.vertices.contains(&v3));
        assert!(union.vertices.contains(&v4));
        assert!(union.vertices.contains(&v5));
        assert!(union.vertices.contains(&v6));
        assert!(union.vertices.contains(&v7));
        assert!(union.vertices.contains(&Rc::new(Vertex::new(2.0, 2.0))));
        assert!(union.vertices.contains(&Rc::new(Vertex::new(3.0, 3.0))));
    }

    #[test]
    fn intersection_at_the_vertex() {
        let v1 = Rc::new(Vertex::new(1.0, 2.0));
        let v2 = Rc::new(Vertex::new(2.0, 2.0));
        let v3 = Rc::new(Vertex::new(2.0, 3.0));
        let v4 = Rc::new(Vertex::new(1.0, 3.0));

        let v5 = Rc::new(Vertex::new(3.0, 1.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ])
        .unwrap();
        let p2 =
            Polyline::new_closed(vec![Rc::clone(&v5), Rc::clone(&v3), Rc::clone(&v1)]).unwrap();

        let (union, unused_segments) = Polyline::union(&p1, &p2).unwrap();
        assert_eq!(unused_segments.len(), 3);

        assert_eq!(union.vertices.len(), 4);
        assert!(union.vertices.contains(&v1));
        assert!(union.vertices.contains(&v5));
        assert!(union.vertices.contains(&v3));
        assert!(union.vertices.contains(&v4));
    }

    #[test]
    fn double_intersection() {
        let v1 = Rc::new(Vertex::new(1.0, 2.0));
        let v2 = Rc::new(Vertex::new(4.0, 2.0));
        let v3 = Rc::new(Vertex::new(4.0, 3.0));
        let v4 = Rc::new(Vertex::new(2.0, 3.0));
        let v5 = Rc::new(Vertex::new(2.0, 4.0));
        let v6 = Rc::new(Vertex::new(4.0, 4.0));
        let v7 = Rc::new(Vertex::new(4.0, 5.0));
        let v8 = Rc::new(Vertex::new(1.0, 5.0));

        let v9 = Rc::new(Vertex::new(3.0, 1.0));
        let v10 = Rc::new(Vertex::new(5.0, 1.0));
        let v11 = Rc::new(Vertex::new(5.0, 6.0));
        let v12 = Rc::new(Vertex::new(3.0, 6.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ])
        .unwrap();
        let p2 = Polyline::new_closed(vec![
            Rc::clone(&v9),
            Rc::clone(&v10),
            Rc::clone(&v11),
            Rc::clone(&v12),
        ])
        .unwrap();

        let (union, unused_segments) = Polyline::union(&p1, &p2).unwrap();
        assert_eq!(unused_segments.len(), 12);

        assert!(union.vertices.contains(&v1));
        assert!(union.vertices.contains(&v9));
        assert!(union.vertices.contains(&v10));
        assert!(union.vertices.contains(&v11));
        assert!(union.vertices.contains(&v12));
        assert!(union.vertices.contains(&v8));
        assert!(union.vertices.contains(&Rc::new(Vertex::new(3.0, 2.0))));
        assert!(union.vertices.contains(&Rc::new(Vertex::new(3.0, 5.0))));
    }
}

#[cfg(test)]
mod subtraction {
    use super::*;

    #[test]
    fn triangles_to_triangles() {
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

        let (subtraction_list, unused_segments) = Polyline::subtraction(&p1, &p2);
        assert_eq!(subtraction_list.len(), 3);
        assert_eq!(unused_segments.len(), 9);

        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(2.5, 1.0)),
            Rc::new(Vertex::new(3.5, 1.0))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(3.5, 4.0)),
            Rc::new(Vertex::new(5.0, 4.0))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(5.0, 4.0)),
            Rc::new(Vertex::new(4.25, 2.5))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(3.5, 1.0)),
            Rc::new(Vertex::new(3.0, 0.0))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(2.5, 4.0)),
            Rc::new(Vertex::new(1.75, 2.5))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(3.0, 0.0)),
            Rc::new(Vertex::new(2.5, 1.0))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(4.25, 2.5)),
            Rc::new(Vertex::new(3.5, 4.0))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(1.75, 2.5)),
            Rc::new(Vertex::new(1.0, 4.0))
        )));
        assert!(unused_segments.contains(&(
            Rc::new(Vertex::new(1.0, 4.0)),
            Rc::new(Vertex::new(2.5, 4.0))
        )));
    }

    #[test]
    fn two_squares() {
        let v1 = Rc::new(Vertex::new(1.0, 2.0));
        let v2 = Rc::new(Vertex::new(3.0, 2.0));
        let v3 = Rc::new(Vertex::new(3.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let v5 = Rc::new(Vertex::new(2.0, 1.0));
        let v6 = Rc::new(Vertex::new(4.0, 1.0));
        let v7 = Rc::new(Vertex::new(4.0, 3.0));
        let v8 = Rc::new(Vertex::new(2.0, 3.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ])
        .unwrap();
        let p2 = Polyline::new_closed(vec![
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ])
        .unwrap();

        let (subtraction_list, unused_segments) = Polyline::subtraction(&p1, &p2);
        assert_eq!(subtraction_list.len(), 1);
        assert_eq!(unused_segments.len(), 6);

        let polyline: &Polyline = subtraction_list.get(0).unwrap();
        assert_eq!(polyline.vertices.len(), 6);
        assert!(polyline.vertices.contains(&v1));
        assert!(polyline.vertices.contains(&v3));
        assert!(polyline.vertices.contains(&v4));
        assert!(polyline.vertices.contains(&v8));
        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(2.0, 2.0))));
        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(3.0, 3.0))));
    }

    #[test]
    fn intersection_at_the_vertex() {
        let v1 = Rc::new(Vertex::new(1.0, 2.0));
        let v2 = Rc::new(Vertex::new(2.0, 2.0));
        let v3 = Rc::new(Vertex::new(2.0, 3.0));
        let v4 = Rc::new(Vertex::new(1.0, 3.0));

        let v5 = Rc::new(Vertex::new(3.0, 1.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ])
        .unwrap();
        let p2 =
            Polyline::new_closed(vec![Rc::clone(&v5), Rc::clone(&v3), Rc::clone(&v1)]).unwrap();

        let (subtraction_list, unused_segments) = Polyline::subtraction(&p1, &p2);
        assert_eq!(subtraction_list.len(), 1);
        assert_eq!(unused_segments.len(), 4);

        let polyline: &Polyline = subtraction_list.get(0).unwrap();
        assert_eq!(polyline.vertices.len(), 3);

        assert!(polyline.vertices.contains(&v1));
        assert!(polyline.vertices.contains(&v3));
        assert!(polyline.vertices.contains(&v4));
    }

    #[test]
    fn double_intersection() {
        let v1 = Rc::new(Vertex::new(1.0, 2.0));
        let v2 = Rc::new(Vertex::new(4.0, 2.0));
        let v3 = Rc::new(Vertex::new(4.0, 3.0));
        let v4 = Rc::new(Vertex::new(2.0, 3.0));
        let v5 = Rc::new(Vertex::new(2.0, 4.0));
        let v6 = Rc::new(Vertex::new(4.0, 4.0));
        let v7 = Rc::new(Vertex::new(4.0, 5.0));
        let v8 = Rc::new(Vertex::new(1.0, 5.0));

        let v9 = Rc::new(Vertex::new(3.0, 1.0));
        let v10 = Rc::new(Vertex::new(5.0, 1.0));
        let v11 = Rc::new(Vertex::new(5.0, 6.0));
        let v12 = Rc::new(Vertex::new(3.0, 6.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ])
        .unwrap();
        let p2 = Polyline::new_closed(vec![
            Rc::clone(&v9),
            Rc::clone(&v10),
            Rc::clone(&v11),
            Rc::clone(&v12),
        ])
        .unwrap();

        let (subtraction_list, unused_segments) = Polyline::subtraction(&p1, &p2);

        assert_eq!(subtraction_list.len(), 1);
        assert_eq!(unused_segments.len(), 12);

        let polyline: &Polyline = subtraction_list.get(0).unwrap();
        assert_eq!(polyline.vertices.len(), 8);

        assert!(polyline.vertices.contains(&v1));
        assert!(polyline.vertices.contains(&v8));
        assert!(polyline.vertices.contains(&v5));
        assert!(polyline.vertices.contains(&v4));

        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(3.0, 2.0))));
        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(3.0, 3.0))));
        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(3.0, 4.0))));
        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(3.0, 5.0))));
    }

    #[test]
    fn exception_case_1() {
        let v1 = Rc::new(Vertex::new(4.0, 1.0));
        let v2 = Rc::new(Vertex::new(5.0, 1.0));
        let v3 = Rc::new(Vertex::new(6.0, 2.0));
        let v4 = Rc::new(Vertex::new(4.0, 4.0));
        let v5 = Rc::new(Vertex::new(3.0, 4.0));
        let v6 = Rc::new(Vertex::new(1.0, 2.0));
        let v7 = Rc::new(Vertex::new(2.0, 1.0));
        let v8 = Rc::new(Vertex::new(3.0, 1.0));
        let v9 = Rc::new(Vertex::new(2.0, 2.0));
        let v10 = Rc::new(Vertex::new(3.0, 3.0));
        let v11 = Rc::new(Vertex::new(4.0, 3.0));
        let v12 = Rc::new(Vertex::new(5.0, 2.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
            Rc::clone(&v9),
            Rc::clone(&v10),
            Rc::clone(&v11),
            Rc::clone(&v12),
        ])
        .unwrap();
        let p2 =
            Polyline::new_closed(vec![Rc::clone(&v1), Rc::clone(&v2), Rc::clone(&v4)]).unwrap();

        let (subtraction_list, _) = Polyline::subtraction(&p2, &p1);

        assert_eq!(subtraction_list.len(), 1);

        let polyline: &Polyline = subtraction_list.get(0).unwrap();
        assert_eq!(polyline.vertices.len(), 4);

        assert!(polyline.vertices.contains(&v1));
        assert!(polyline.vertices.contains(&v11));
        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(4.5, 2.5))));
        assert!(polyline
            .vertices
            .contains(&Rc::new(Vertex::new(4.75, 1.75))));
    }

    #[test]
    fn exception_case_2() {
        let v1 = Rc::new(Vertex::new(4.0, 1.0));
        let v2 = Rc::new(Vertex::new(5.0, 1.0));
        let v3 = Rc::new(Vertex::new(6.0, 2.0));
        let v4 = Rc::new(Vertex::new(4.0, 4.0));
        let v5 = Rc::new(Vertex::new(3.0, 4.0));
        let v6 = Rc::new(Vertex::new(1.0, 2.0));
        let v7 = Rc::new(Vertex::new(2.0, 1.0));
        let v8 = Rc::new(Vertex::new(3.0, 1.0));
        let v9 = Rc::new(Vertex::new(2.0, 2.0));
        let v10 = Rc::new(Vertex::new(3.0, 3.0));
        let v11 = Rc::new(Vertex::new(4.0, 3.0));
        let v12 = Rc::new(Vertex::new(5.0, 2.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
            Rc::clone(&v9),
            Rc::clone(&v10),
            Rc::clone(&v11),
            Rc::clone(&v12),
        ])
        .unwrap();
        let p2 =
            Polyline::new_closed(vec![Rc::clone(&v1), Rc::clone(&v2), Rc::clone(&v10)]).unwrap();

        let (subtraction_list, _) = Polyline::subtraction(&p2, &p1);

        assert_eq!(subtraction_list.len(), 1);

        let polyline: &Polyline = subtraction_list.get(0).unwrap();
        assert_eq!(polyline.vertices.len(), 3);

        assert!(polyline.vertices.contains(&v1));
        assert!(polyline.vertices.contains(&v10));
        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(4.5, 1.5))));
    }

    #[test]
    fn exception_case_3() {
        let v1 = Rc::new(Vertex::new(4.0, 1.0));
        let v2 = Rc::new(Vertex::new(5.0, 1.0));
        let v3 = Rc::new(Vertex::new(6.0, 2.0));
        let v4 = Rc::new(Vertex::new(4.0, 4.0));
        let v5 = Rc::new(Vertex::new(3.0, 4.0));
        let v6 = Rc::new(Vertex::new(1.0, 2.0));
        let v7 = Rc::new(Vertex::new(2.0, 1.0));
        let v8 = Rc::new(Vertex::new(3.0, 1.0));
        let v9 = Rc::new(Vertex::new(2.0, 2.0));
        let v10 = Rc::new(Vertex::new(3.0, 3.0));
        let v11 = Rc::new(Vertex::new(4.0, 3.0));
        let v12 = Rc::new(Vertex::new(5.0, 2.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
            Rc::clone(&v9),
            Rc::clone(&v10),
            Rc::clone(&v11),
            Rc::clone(&v12),
        ])
        .unwrap();
        let p2 =
            Polyline::new_closed(vec![Rc::clone(&v1), Rc::clone(&v2), Rc::clone(&v5)]).unwrap();

        let (subtraction_list, _) = Polyline::subtraction(&p2, &p1);

        assert_eq!(subtraction_list.len(), 1);

        let polyline: &Polyline = subtraction_list.get(0).unwrap();
        assert_eq!(polyline.vertices.len(), 4);

        assert!(polyline.vertices.contains(&v1));
        assert!(polyline
            .vertices
            .contains(&Rc::new(Vertex::new(3.333333333333333, 3.0))));
        assert!(polyline
            .vertices
            .contains(&Rc::new(Vertex::new(3.6666666666666674, 3.0))));
        assert!(polyline.vertices.contains(&Rc::new(Vertex::new(
            4.6000000000000005,
            1.6000000000000005
        ))));
    }
} /* end - subtraction tests */

#[cfg(test)]
mod split_by_intersections {
    use super::*;

    #[test]
    fn test_split() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(5.0, 1.0));
        let v3 = Rc::new(Vertex::new(3.0, 5.0));

        let v4 = Rc::new(Vertex::new(3.0, 0.0));
        let v5 = Rc::new(Vertex::new(5.0, 4.0));
        let v6 = Rc::new(Vertex::new(1.0, 4.0));

        let t1: Vec<(Rc<Vertex>, Rc<Vertex>)> = vertex_pairs(&vec![v1, v2, v3], false);
        let t2: Vec<(Rc<Vertex>, Rc<Vertex>)> = vertex_pairs(&vec![v4, v5, v6], false);
        let segments: Vec<(Rc<Vertex>, Rc<Vertex>)> = t1.iter().chain(t2.iter()).cloned().collect();
        let splited_segments = split_intersections(&segments);
        assert_eq!(splited_segments.len(), 18);
    }
}

#[cfg(test)]
mod arrange {
    use super::*;

    #[test]
    fn sample_1() {
        let v1 = Rc::new(Vertex::new(4.0, 1.0));
        let v2 = Rc::new(Vertex::new(5.0, 1.0));
        let v3 = Rc::new(Vertex::new(6.0, 2.0));
        let v4 = Rc::new(Vertex::new(4.0, 4.0));
        let v5 = Rc::new(Vertex::new(3.0, 4.0));
        let v6 = Rc::new(Vertex::new(1.0, 2.0));
        let v7 = Rc::new(Vertex::new(2.0, 1.0));
        let v8 = Rc::new(Vertex::new(3.0, 1.0));
        let v9 = Rc::new(Vertex::new(2.0, 2.0));
        let v10 = Rc::new(Vertex::new(3.0, 3.0));
        let v11 = Rc::new(Vertex::new(4.0, 3.0));
        let v12 = Rc::new(Vertex::new(5.0, 2.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
            Rc::clone(&v9),
            Rc::clone(&v10),
            Rc::clone(&v11),
            Rc::clone(&v12),
        ])
        .unwrap();

        let edges: HashSet<Rc<Edge>> = p1.into_edges().iter().cloned().collect();
        let arranged_p1 = Polyline::arrange(&edges).unwrap();

        assert_eq!(
            (12 + p1.vertices.iter().position(|v| v == &v2).unwrap()
                - p1.vertices.iter().position(|v| v == &v1).unwrap())
                % 12,
            1
        );
        assert_eq!(
            (12 + p1.vertices.iter().position(|v| v == &v3).unwrap()
                - p1.vertices.iter().position(|v| v == &v2).unwrap())
                % 12,
            1
        );
        assert_eq!(
            (12 + p1.vertices.iter().position(|v| v == &v4).unwrap()
                - p1.vertices.iter().position(|v| v == &v3).unwrap())
                % 12,
            1
        );
        assert_eq!(
            (12 + p1.vertices.iter().position(|v| v == &v5).unwrap()
                - p1.vertices.iter().position(|v| v == &v4).unwrap())
                % 12,
            1
        );
        assert_eq!(
            (12 + p1.vertices.iter().position(|v| v == &v6).unwrap()
                - p1.vertices.iter().position(|v| v == &v5).unwrap())
                % 12,
            1
        );
        assert_eq!(
            (12 + p1.vertices.iter().position(|v| v == &v7).unwrap()
                - p1.vertices.iter().position(|v| v == &v6).unwrap())
                % 12,
            1
        );
        assert_eq!(
            (12 + p1.vertices.iter().position(|v| v == &v8).unwrap()
                - p1.vertices.iter().position(|v| v == &v7).unwrap())
                % 12,
            1
        );
        assert_eq!(
            (12 + p1.vertices.iter().position(|v| v == &v9).unwrap()
                - p1.vertices.iter().position(|v| v == &v8).unwrap())
                % 12,
            1
        );
        assert_eq!(
            (12 + p1.vertices.iter().position(|v| v == &v10).unwrap()
                - p1.vertices.iter().position(|v| v == &v9).unwrap())
                % 12,
            1
        );
        assert_eq!(
            (12 + p1.vertices.iter().position(|v| v == &v11).unwrap()
                - p1.vertices.iter().position(|v| v == &v10).unwrap())
                % 12,
            1
        );
        assert_eq!(
            (12 + p1.vertices.iter().position(|v| v == &v12).unwrap()
                - p1.vertices.iter().position(|v| v == &v11).unwrap())
                % 12,
            1
        );
        assert_eq!(
            (12 + p1.vertices.iter().position(|v| v == &v1).unwrap()
                - p1.vertices.iter().position(|v| v == &v12).unwrap())
                % 12,
            1
        );
    }

    #[test]
    fn sample_2() {
        let v1 = Rc::new(Vertex::new(1.0, 2.0));
        let v2 = Rc::new(Vertex::new(4.0, 2.0));
        let v3 = Rc::new(Vertex::new(4.0, 3.0));
        let v4 = Rc::new(Vertex::new(2.0, 3.0));
        let v5 = Rc::new(Vertex::new(2.0, 4.0));
        let v6 = Rc::new(Vertex::new(4.0, 4.0));
        let v7 = Rc::new(Vertex::new(4.0, 5.0));
        let v8 = Rc::new(Vertex::new(1.0, 5.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ])
        .unwrap();

        let edges: HashSet<Rc<Edge>> = p1.into_edges().iter().cloned().collect();
        let arranged_p1 = Polyline::arrange(&edges).unwrap();

        assert_eq!(
            (8 + p1.vertices.iter().position(|v| v == &v2).unwrap()
                - p1.vertices.iter().position(|v| v == &v1).unwrap())
                % 8,
            1
        );
        assert_eq!(
            (8 + p1.vertices.iter().position(|v| v == &v3).unwrap()
                - p1.vertices.iter().position(|v| v == &v2).unwrap())
                % 8,
            1
        );
        assert_eq!(
            (8 + p1.vertices.iter().position(|v| v == &v4).unwrap()
                - p1.vertices.iter().position(|v| v == &v3).unwrap())
                % 8,
            1
        );
        assert_eq!(
            (8 + p1.vertices.iter().position(|v| v == &v5).unwrap()
                - p1.vertices.iter().position(|v| v == &v4).unwrap())
                % 8,
            1
        );
        assert_eq!(
            (8 + p1.vertices.iter().position(|v| v == &v6).unwrap()
                - p1.vertices.iter().position(|v| v == &v5).unwrap())
                % 8,
            1
        );
        assert_eq!(
            (8 + p1.vertices.iter().position(|v| v == &v7).unwrap()
                - p1.vertices.iter().position(|v| v == &v6).unwrap())
                % 8,
            1
        );
        assert_eq!(
            (8 + p1.vertices.iter().position(|v| v == &v8).unwrap()
                - p1.vertices.iter().position(|v| v == &v7).unwrap())
                % 8,
            1
        );
        assert_eq!(
            (8 + p1.vertices.iter().position(|v| v == &v1).unwrap()
                - p1.vertices.iter().position(|v| v == &v8).unwrap())
                % 8,
            1
        );
    }
}

#[cfg(test)]
mod minified_noncolinear {
    use super::*;

    #[test]
    fn sample_1() {
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(2.0, 1.0));
        let v3 = Rc::new(Vertex::new(3.0, 1.0));
        let v4 = Rc::new(Vertex::new(3.0, 2.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ])
        .unwrap();

        let minified = p1.minified_noncolinear();

        assert!(minified.vertices.contains(&v1));
        assert!(minified.vertices.contains(&v3));
        assert!(minified.vertices.contains(&v4));
        assert!(!minified.vertices.contains(&v2));
    }

    #[test]
    fn sample_2() {
        let v1 = Rc::new(Vertex::new(1.0, 2.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.0));
        let v3 = Rc::new(Vertex::new(2.0, 1.0));
        let v4 = Rc::new(Vertex::new(3.0, 1.0));
        let v5 = Rc::new(Vertex::new(4.0, 1.0));
        let v6 = Rc::new(Vertex::new(5.0, 1.0));
        let v7 = Rc::new(Vertex::new(6.0, 1.0));
        let v8 = Rc::new(Vertex::new(7.0, 1.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ])
        .unwrap();

        let minified = p1.minified_noncolinear();

        assert!(minified.vertices.contains(&v1));
        assert!(minified.vertices.contains(&v2));
        assert!(!minified.vertices.contains(&v3));
        assert!(!minified.vertices.contains(&v4));
        assert!(!minified.vertices.contains(&v5));
        assert!(!minified.vertices.contains(&v6));
        assert!(!minified.vertices.contains(&v7));
        assert!(minified.vertices.contains(&v8));
    }

    #[test]
    fn sample_3() {
        let v1 = Rc::new(Vertex::new(5.0, 1.0));
        let v2 = Rc::new(Vertex::new(6.0, 1.0));
        let v3 = Rc::new(Vertex::new(7.0, 1.0));
        let v4 = Rc::new(Vertex::new(1.0, 2.0));
        let v5 = Rc::new(Vertex::new(1.0, 1.0));
        let v6 = Rc::new(Vertex::new(2.0, 1.0));
        let v7 = Rc::new(Vertex::new(3.0, 1.0));
        let v8 = Rc::new(Vertex::new(4.0, 1.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ])
        .unwrap();

        let minified = p1.minified_noncolinear();

        assert!(!minified.vertices.contains(&v1));
        assert!(!minified.vertices.contains(&v2));
        assert!(minified.vertices.contains(&v3));
        assert!(minified.vertices.contains(&v4));
        assert!(minified.vertices.contains(&v5));
        assert!(!minified.vertices.contains(&v6));
        assert!(!minified.vertices.contains(&v7));
        assert!(!minified.vertices.contains(&v8));
    }

    #[test]
    fn sample_4() {
        let v1 = Rc::new(Vertex::new(5.0, 1.0));
        let v2 = Rc::new(Vertex::new(6.0, 1.0));
        let v3 = Rc::new(Vertex::new(7.0, 1.0));
        let v4 = Rc::new(Vertex::new(1.0, 2.0));
        let v5 = Rc::new(Vertex::new(1.0, 1.0));
        let v6 = Rc::new(Vertex::new(2.0, 1.0));
        let v7 = Rc::new(Vertex::new(3.0, 1.0));
        let v8 = Rc::new(Vertex::new(4.0, 1.0));

        let p1 = Polyline::new_opened(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ])
        .unwrap();

        let minified = p1.minified_noncolinear();

        assert!(minified.vertices.contains(&v1));
        assert!(!minified.vertices.contains(&v2));
        assert!(minified.vertices.contains(&v3));
        assert!(minified.vertices.contains(&v4));
        assert!(minified.vertices.contains(&v5));
        assert!(!minified.vertices.contains(&v6));
        assert!(!minified.vertices.contains(&v7));
        assert!(minified.vertices.contains(&v8));
    }
}

#[cfg(test)]
mod continence_self {
    use super::*;

    #[test]
    fn sample_1() {
        /* square (1,1) (4,4) */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ])
        .unwrap();

        /* square (2,2) (3,3) -> inside */
        let v5 = Rc::new(Vertex::new(2.0, 2.0));
        let v6 = Rc::new(Vertex::new(3.0, 2.0));
        let v7 = Rc::new(Vertex::new(3.0, 3.0));
        let v8 = Rc::new(Vertex::new(2.0, 3.0));

        let p2 = Polyline::new_closed(vec![
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ])
        .unwrap();

        assert_eq!(
            Polyline::continence(&p1, &p2),
            Some((Continence::Inside, BoundaryInclusion::Open))
        );
    }

    #[test]
    fn sample_2() {
        /* square (1,1) (4,4) */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ])
        .unwrap();

        /* square (0,0) (5,5) -> inside */
        let v5 = Rc::new(Vertex::new(0.0, 0.0));
        let v6 = Rc::new(Vertex::new(5.0, 0.0));
        let v7 = Rc::new(Vertex::new(5.0, 5.0));
        let v8 = Rc::new(Vertex::new(0.0, 5.0));

        let p2 = Polyline::new_closed(vec![
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ])
        .unwrap();

        assert_eq!(
            Polyline::continence(&p1, &p2),
            Some((Continence::Outside, BoundaryInclusion::Open))
        );
    }

    #[test]
    fn sample_3() {
        /* square (1,1) (4,4) */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ])
        .unwrap();

        /* square (2,2) (3,4) -> Boundary */
        let v5 = Rc::new(Vertex::new(2.0, 2.0));
        let v6 = Rc::new(Vertex::new(3.0, 2.0));
        let v7 = Rc::new(Vertex::new(3.0, 4.0));
        let v8 = Rc::new(Vertex::new(2.0, 4.0));

        let p2 = Polyline::new_closed(vec![
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ])
        .unwrap();

        assert_eq!(
            Polyline::continence(&p1, &p2),
            Some((Continence::Inside, BoundaryInclusion::Closed))
        );
    }

    #[test]
    fn sample_4() {
        /* square (1,1) (4,4) */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(4.0, 1.0));
        let v3 = Rc::new(Vertex::new(4.0, 4.0));
        let v4 = Rc::new(Vertex::new(1.0, 4.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
        ])
        .unwrap();

        /* square (2,0) (3,5) -> all vertices outside, with intersection */
        let v5 = Rc::new(Vertex::new(2.0, 0.0));
        let v6 = Rc::new(Vertex::new(3.0, 0.0));
        let v7 = Rc::new(Vertex::new(3.0, 5.0));
        let v8 = Rc::new(Vertex::new(2.0, 5.0));

        let p2 = Polyline::new_closed(vec![
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v7),
            Rc::clone(&v8),
        ])
        .unwrap();

        assert_eq!(Polyline::continence(&p1, &p2), None);
    }

    #[test]
    fn sample_5() {
        /* hexagon */
        let v1 = Rc::new(Vertex::new(1.0, 0.0));
        let v2 = Rc::new(Vertex::new(2.0, 0.0));
        let v3 = Rc::new(Vertex::new(3.0, 1.0));
        let v4 = Rc::new(Vertex::new(2.0, 2.0));
        let v5 = Rc::new(Vertex::new(1.0, 2.0));
        let v6 = Rc::new(Vertex::new(0.0, 1.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
        ])
        .unwrap();

        /* Internal triangles */
        for v in vec![
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
        ]
        .iter()
        {
            let p2 =
                Polyline::new_closed(vec![Rc::clone(&v1), Rc::clone(&v2), Rc::clone(&v)]).unwrap();
            assert_eq!(
                Polyline::continence(&p1, &p2),
                Some((Continence::Inside, BoundaryInclusion::Closed))
            );
        }
    } /* end - internal triangles test */

    #[test]
    fn sample_6() {
        /* hexagon */
        let v1 = Rc::new(Vertex::new(1.0, 0.0));
        let v2 = Rc::new(Vertex::new(2.0, 0.0));
        let v3 = Rc::new(Vertex::new(3.0, 1.0));
        let v4 = Rc::new(Vertex::new(2.0, 2.0));
        let v5 = Rc::new(Vertex::new(1.0, 2.0));
        let v6 = Rc::new(Vertex::new(0.0, 1.0));

        let p1 = Polyline::new_closed(vec![
            Rc::clone(&v1),
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
        ])
        .unwrap();

        /* Identity */
        let p2 = Polyline::new_closed(vec![
            Rc::clone(&v2),
            Rc::clone(&v3),
            Rc::clone(&v4),
            Rc::clone(&v5),
            Rc::clone(&v6),
            Rc::clone(&v1),
        ])
        .unwrap();
        assert_eq!(
            Polyline::continence(&p1, &p2),
            Some((Continence::Boundary, BoundaryInclusion::Closed))
        );
    } /* end - internal triangles test */
} /* end - continence self self tests */

#[cfg(test)]
mod triangles_hull {
    use super::*;

    #[test]
    fn sample_1() {
        /* boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(5.0, 1.0));
        let v3 = Rc::new(Vertex::new(5.0, 5.0));
        let v4 = Rc::new(Vertex::new(1.0, 5.0));

        /* square hole */
        let v5 = Rc::new(Vertex::new(3.0, 2.0));
        let v6 = Rc::new(Vertex::new(4.0, 3.0));

        let t1 = Rc::new(Triangle::new(&v6, &v5, &v2));
        let t2 = Rc::new(Triangle::new(&v5, &v4, &v1));
        let t3 = Rc::new(Triangle::new(&v6, &v3, &v4));
        let t4 = Rc::new(Triangle::new(&v5, &v6, &v4));
        let t5 = Rc::new(Triangle::new(&v2, &v5, &v1));

        let hull = Polyline::triangles_hull(
            &vec![
                Rc::clone(&t1),
                Rc::clone(&t2),
                Rc::clone(&t3),
                Rc::clone(&t4),
                Rc::clone(&t5),
            ]
            .iter()
            .cloned()
            .collect(),
        )
        .unwrap();

        assert!(hull.vertices.contains(&v1));
        assert!(hull.vertices.contains(&v2));
        assert!(hull.vertices.contains(&v3));
        assert!(hull.vertices.contains(&v4));
        assert!(hull.vertices.contains(&v6));
    }

    #[test]
    fn sample_2() {
        /* boundary */
        let v1 = Rc::new(Vertex::new(1.0, 1.0));
        let v2 = Rc::new(Vertex::new(6.0, 1.0));
        // let v3 = Rc::new(Vertex::new(6.0, 5.0));
        let v4 = Rc::new(Vertex::new(1.0, 5.0));

        /* hexagonal hole */
        // let v5 = Rc::new(Vertex::new(3.0, 2.0));
        let v6 = Rc::new(Vertex::new(4.0, 2.0));
        // let v7 = Rc::new(Vertex::new(5.0, 3.0));
        let v8 = Rc::new(Vertex::new(4.0, 4.0));
        let v9 = Rc::new(Vertex::new(3.0, 4.0));
        let v10 = Rc::new(Vertex::new(2.0, 3.0));

        let t1 = Rc::new(Triangle::new(&v9, &v6, &v8));
        let t2 = Rc::new(Triangle::new(&v10, &v4, &v1));
        let t3 = Rc::new(Triangle::new(&v10, &v1, &v6));
        let t4 = Rc::new(Triangle::new(&v6, &v1, &v2));
        let t5 = Rc::new(Triangle::new(&v9, &v10, &v6));

        let hull = Polyline::triangles_hull(
            &vec![
                Rc::clone(&t1),
                Rc::clone(&t2),
                Rc::clone(&t3),
                Rc::clone(&t4),
                Rc::clone(&t5),
            ]
            .iter()
            .cloned()
            .collect(),
        )
        .unwrap();

        assert!(hull.vertices.contains(&v1));
        assert!(hull.vertices.contains(&v2));
        assert!(hull.vertices.contains(&v6));
        assert!(hull.vertices.contains(&v8));
        assert!(hull.vertices.contains(&v9));
        assert!(hull.vertices.contains(&v10));
        assert!(hull.vertices.contains(&v4));
    }
} /* end - triangles_hull */
