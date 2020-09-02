use crate::elements::{edge::*, polyline::*, triangle::*, vertex::*};
use crate::planar::{
    refine_params::RefineParams, refine_procedures::encroachment, triangulation::*,
    triangulation_procedures,
};

use crate::properties::continence::*;

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

/**
 * Determines if the triangle is irregular according to quality ratio
 */
fn is_irregular_triangle(triangle: &Triangle, params: &RefineParams) -> bool {
    let this_quality = triangle.quality().unwrap();
    let no_quality = float_cmp::approx_eq!(
        f64,
        this_quality,
        params.quality_ratio,
        epsilon = 1.0E-14f64
    ) || this_quality >= params.quality_ratio;

    return no_quality;
}

/**
 * Determines if the triangle is larger than threshould
 */
fn is_large_triangle(triangle: &Triangle, params: &RefineParams) -> bool {
    let this_area = triangle.area().unwrap();
    let greater_area: bool = match params.max_area {
        Some(max_area) => {
            float_cmp::approx_eq!(f64, this_area, max_area, epsilon = 1.0E-14f64)
                || this_area >= max_area
        }
        _ => false,
    };

    return greater_area;
}

/**
 * Refines specified triangles, according to Rupperts refinement.
 * If the triangle's circumcenter is encroached, it splits the corresponding
 * encroaching edge. Else it inserts the circumcenter into the triangulation.
 * A Modified triangle won't be touched if, a prior vertex insertion removes it.
 */
pub fn split_irregular(
    triangulation: &mut Triangulation,
    params: &RefineParams,
    segment_contraints: &HashSet<Rc<Edge>>,
    boundary: &Option<Rc<Polyline>>,
    holes: &HashSet<Rc<Polyline>>,
) -> HashMap<Rc<Edge>, Rc<Edge>> {
    let mut segment_contraints: HashSet<Rc<Edge>> = segment_contraints.iter().cloned().collect();

    let critical_triangles = triangulation
        .triangles
        .iter()
        .filter(|t| !t.is_ghost())
        .filter(|t| is_irregular_triangle(t, params) || is_large_triangle(t, params))
        .cloned()
        .collect::<HashSet<Rc<Triangle>>>();

    let mut irregular_triangles: HashSet<Rc<Triangle>> = critical_triangles
        .iter()
        .filter(|t| is_irregular_triangle(t, params))
        .cloned()
        .collect();

    let mut large_triangles: HashSet<Rc<Triangle>> = critical_triangles
        .iter()
        .filter(|t| is_large_triangle(t, params))
        .cloned()
        .collect();

    let mut split_map: HashMap<Rc<Edge>, Rc<Edge>> = HashMap::new();

    loop {
        let triangle;
        if let Some(irregular_triangle) = irregular_triangles.iter().next() {
            triangle = Rc::clone(&irregular_triangle);
            irregular_triangles.remove(&triangle);
        } else if let Some(large_triangle) = large_triangles.iter().next() {
            triangle = Rc::clone(&large_triangle);
            large_triangles.remove(&triangle);
        } else {
            break;
        }

        // Uncomment to debug
        // println!(
        //     "\nSplitting Triangle (Total: {} + {}) - {}",
        //     irregular_triangles.len(),
        //     large_triangles.len(),
        //     triangle.quality().unwrap()
        // );

        // println!("\nTriangles");
        // for t in triangulation.triangles.iter().filter(|t| !t.is_ghost()) {
        //     println!("{}", t);
        // }

        match try_circumcenter_insertion(
            triangulation,
            &triangle,
            &segment_contraints,
            boundary,
            holes,
        ) {
            Ok((included_triangles, removed_triangles)) => {
                for new_triangle in included_triangles.iter().filter(|t| !t.is_ghost()) {
                    if is_irregular_triangle(new_triangle, params) {
                        irregular_triangles.insert(Rc::clone(new_triangle));
                        continue;
                    }
                    if is_large_triangle(new_triangle, params) {
                        large_triangles.insert(Rc::clone(new_triangle));
                        continue;
                    }
                }
                for old_triangle in removed_triangles.iter() {
                    irregular_triangles.remove(old_triangle);
                    large_triangles.remove(old_triangle);
                }
            }
            Err(encroachments) => {
                let mut vertices = vec![Rc::new(triangle.circumcenter().unwrap())]
                    .iter()
                    .cloned()
                    .collect();

                for encroached_edge in encroachments.iter() {
                    let (new_edges, included_triangles, removed_triangles) =
                        encroachment::unencroach_segment(
                            triangulation,
                            &encroached_edge,
                            &mut vertices,
                            &segment_contraints,
                            boundary,
                            holes,
                        );

                    segment_contraints.remove(encroached_edge);
                    split_map.remove(encroached_edge);
                    for subsegment in new_edges.iter() {
                        split_map.insert(Rc::clone(subsegment), Rc::clone(encroached_edge));
                        segment_contraints.insert(Rc::clone(subsegment));
                    }

                    for new_triangle in included_triangles.iter().filter(|t| !t.is_ghost()) {
                        if is_irregular_triangle(new_triangle, params) {
                            irregular_triangles.insert(Rc::clone(new_triangle));
                            continue;
                        }
                        if is_large_triangle(new_triangle, params) {
                            large_triangles.insert(Rc::clone(new_triangle));
                            continue;
                        }
                    }
                    for old_triangle in removed_triangles.iter() {
                        irregular_triangles.remove(old_triangle);
                        large_triangles.remove(old_triangle);
                    }
                }
            }
        }
    }
    return split_map;
} /* end - split */

/**
 * Tries to insert a triangle's circumcenter.
 * If any edge of the triangle is constrained and encroaches the circumcenter,
 * it is returned in the hashset. The circumcenter will be inserted through constrained
 * insertion. Among the included triangles, if any is composed by a constrained segment
 * that encroaches the circumcenter, the segment is returned in the hashset. If there is
 * no encroachments, the returnable is empty.
 */
fn try_circumcenter_insertion(
    triangulation: &mut Triangulation,
    triangle: &Rc<Triangle>,
    segment_constraints: &HashSet<Rc<Edge>>,
    boundary: &Option<Rc<Polyline>>,
    holes: &HashSet<Rc<Polyline>>,
) -> Result<(HashSet<Rc<Triangle>>, HashSet<Rc<Triangle>>), HashSet<Rc<Edge>>> {
    let circumcenter = Rc::new(triangle.circumcenter().unwrap());
    let mut conflict_map: HashMap<Rc<Triangle>, Vec<Rc<Vertex>>> = HashMap::new();

    triangulation_procedures::vertices::distribute_conflicts_over_triangulation(
        triangulation,
        Some(Rc::clone(triangle)),
        &mut conflict_map,
        &mut vec![Rc::clone(&circumcenter)],
        boundary,
        holes,
    );

    let (included_triangles, removed_triangles) =
        triangulation_procedures::vertices::solve_conflicts(
            triangulation,
            &mut conflict_map,
            &mut Vec::new(),
            segment_constraints,
            boundary,
            holes,
        );

    let encroachments: HashSet<Rc<Edge>> = included_triangles
        .iter()
        .map(|t| t.opposite_edge(&circumcenter).unwrap())
        .filter(|e| segment_constraints.contains(e) || segment_constraints.contains(&e.opposite()))
        .filter(|e| e.encroach(&circumcenter) == Continence::Inside)
        .collect();

    if !encroachments.is_empty() {
        for t in included_triangles.iter() {
            triangulation.remove_triangle(t);
        }

        for t in removed_triangles.iter() {
            triangulation.include_triangle(t);
        }
        return Err(encroachments);
    }

    return Ok((included_triangles, removed_triangles));
}

#[cfg(test)]
mod split {
    use super::*;

    #[test]
    fn sample_1() {
        let triangle_side: f64 = 1.0;
        let sqrt_3: f64 = 1.7320508075688772;
        let triangle_height = triangle_side * sqrt_3 / 2.0;

        /* triangle */
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(triangle_side, 0.0));
        let v3 = Rc::new(Vertex::new(triangle_side / 2.0, triangle_height));

        let boundary = Rc::new(
            Polyline::new_closed(vec![Rc::clone(&v1), Rc::clone(&v2), Rc::clone(&v3)]).unwrap(),
        );

        /* Encroaching vertex */
        let encroaching_vertex = Rc::new(Vertex::new(triangle_side / 2.0, triangle_height / 3.0));

        /* Triangulation */
        let mut triangulation = Triangulation::from_initial_segment((&v1, &v2));
        triangulation_procedures::boundary::include(&mut triangulation, &boundary, &HashSet::new());
        triangulation_procedures::vertices::include(
            &mut triangulation,
            vec![Rc::clone(&encroaching_vertex)],
            &HashSet::new(),
            &Some(Rc::clone(&boundary)),
            &HashSet::new(),
        );

        let mut segment_constraints: HashSet<Rc<Edge>> =
            boundary.into_edges().iter().cloned().collect();

        /* unencroach */
        let (mapping, included_triangles, removed_triangles) = encroachment::unencroach(
            &mut triangulation,
            &boundary.into_edges().iter().cloned().collect(),
            &Some(Rc::clone(&boundary)),
            &HashSet::new(),
        );

        segment_constraints = segment_constraints
            .iter()
            .filter(|&e| {
                !mapping
                    .keys()
                    .cloned()
                    .collect::<HashSet<Rc<Edge>>>()
                    .contains(e)
            })
            .chain(mapping.values().flatten())
            .cloned()
            .collect();

        split_irregular(
            &mut triangulation,
            &RefineParams {
                max_area: None, /* not used */
                quality_ratio: 1.0,
            },
            &segment_constraints,
            &Some(Rc::clone(&boundary)),
            &HashSet::new(),
        );

        let solid_triangles: HashSet<Rc<Triangle>> = triangulation
            .triangles
            .iter()
            .filter(|t| !t.is_ghost())
            .cloned()
            .collect();

        let t1 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.5, 0.28867513459481287)),
            &Rc::new(Vertex::new(0.5, 0.0)),
            &Rc::new(Vertex::new(0.75, 0.0)),
        ));
        let t2 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.5, 0.28867513459481287)),
            &Rc::new(Vertex::new(0.75, 0.0)),
            &Rc::new(Vertex::new(0.875, 0.21650635094610965)),
        ));
        let t3 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.375, 0.649519052838329)),
            &Rc::new(Vertex::new(0.5, 0.28867513459481287)),
            &Rc::new(Vertex::new(0.625, 0.649519052838329)),
        ));
        let t4 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.25, 0.4330127018922193)),
            &Rc::new(Vertex::new(0.5, 0.28867513459481287)),
            &Rc::new(Vertex::new(0.375, 0.649519052838329)),
        ));
        let t5 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.75, 0.0)),
            &Rc::new(Vertex::new(1.0, 0.0)),
            &Rc::new(Vertex::new(0.875, 0.21650635094610965)),
        ));
        let t6 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.5, 0.8660254037844386)),
            &Rc::new(Vertex::new(0.375, 0.649519052838329)),
            &Rc::new(Vertex::new(0.625, 0.649519052838329)),
        ));
        let t7 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.5, 0.28867513459481287)),
            &Rc::new(Vertex::new(0.25, 0.4330127018922193)),
            &Rc::new(Vertex::new(0.125, 0.21650635094610965)),
        ));
        let t8 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.5, 0.0)),
            &Rc::new(Vertex::new(0.5, 0.28867513459481287)),
            &Rc::new(Vertex::new(0.25, 0.0)),
        ));
        let t9 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.75, 0.4330127018922193)),
            &Rc::new(Vertex::new(0.5, 0.28867513459481287)),
            &Rc::new(Vertex::new(0.875, 0.21650635094610965)),
        ));
        let t10 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.25, 0.0)),
            &Rc::new(Vertex::new(0.5, 0.28867513459481287)),
            &Rc::new(Vertex::new(0.125, 0.21650635094610965)),
        ));
        let t11 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.0, 0.0)),
            &Rc::new(Vertex::new(0.25, 0.0)),
            &Rc::new(Vertex::new(0.125, 0.21650635094610965)),
        ));
        let t12 = Rc::new(Triangle::new(
            &Rc::new(Vertex::new(0.5, 0.28867513459481287)),
            &Rc::new(Vertex::new(0.75, 0.4330127018922193)),
            &Rc::new(Vertex::new(0.625, 0.649519052838329)),
        ));

        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t1));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t2));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t3));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t4));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t5));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t6));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t7));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t8));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t9));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t10));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t11));
        assert!(triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(&t12));
    }
}
