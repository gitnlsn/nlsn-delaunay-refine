use crate::elements::{edge::*, polyline::*, triangle::*, vertex::*};
use crate::planar::{
    refine_params::RefineParams, refine_procedures::encroachment, triangulation::*,
    triangulation_procedures,
};

use std::collections::{HashMap, HashSet};
use std::rc::Rc;

/**
 * Returns the list of irregular triangles.
 * A triangle is irregular is its quality ratio is greater or equal
 * to the expected quality. A mapping from its circumcenter is returned.
 */
pub fn find_irregular(
    triangulation: &mut Triangulation,
    params: &RefineParams,
) -> HashMap<Rc<Vertex>, Rc<Triangle>> {
    triangulation
        .triangles
        .iter()
        .filter(|t| !t.is_ghost())
        .filter(|t| {
            let this_quality = t.quality();
            let is_equal = float_cmp::approx_eq!(
                f64,
                this_quality,
                params.quality_ratio,
                epsilon = 1.0E-14f64
            );
            let is_greater = t.quality() >= params.quality_ratio;
            return is_equal || is_greater;
        })
        .map(|t| (Rc::new(t.circumcenter().unwrap()), Rc::clone(t)))
        .collect::<HashMap<Rc<Vertex>, Rc<Triangle>>>()
}

/**
 * Refines specified triangles, according to Rupperts refinement.
 * If the triangle's circumcenter is encroached, it splits the corresponding
 * encroaching edge. Else it inserts the circumcenter into the triangulation.
 * A Modified triangle won't be touched if, a prior vertex insertion removes it.
 */
pub fn split_irregular(
    triangulation: &mut Triangulation,
    irregular_triangles: &mut HashMap<Rc<Vertex>, Rc<Triangle>>,
    segment_contraints: &HashSet<Rc<Edge>>,
    boundary: &Option<Rc<Polyline>>,
    holes: &HashSet<Rc<Polyline>>,
) -> HashMap<Rc<Edge>, HashSet<Rc<Edge>>> {
    let mut split_map: HashMap<Rc<Edge>, HashSet<Rc<Edge>>> = HashMap::new();

    let mut encroach_map: HashMap<Rc<Edge>, HashSet<Rc<Vertex>>> = HashMap::new();
    encroachment::distribute_encroachments(
        segment_contraints,
        &irregular_triangles
            .keys()
            .cloned()
            .collect::<HashSet<Rc<Vertex>>>(),
        &mut encroach_map,
    );

    while !encroach_map.is_empty() {
        let encroached_edge = Rc::clone(encroach_map.keys().next().unwrap());
        let mut encroaching_vertices = encroach_map.remove(&encroached_edge).unwrap();

        let new_edges = encroachment::unencroach_segment(
            triangulation,
            &encroached_edge,
            &mut encroaching_vertices,
            segment_contraints,
            boundary,
            holes,
        );

        split_map.insert(Rc::clone(&encroached_edge), new_edges);

        for v in encroaching_vertices.iter() {
            irregular_triangles.remove(v);
        }
    }

    for (circumcenter, triangle) in irregular_triangles.iter() {
        if triangulation
            .triangles
            .iter()
            .cloned()
            .collect::<Vec<Rc<Triangle>>>()
            .contains(triangle)
        {
            triangulation_procedures::vertices::include(
                triangulation,
                vec![Rc::clone(circumcenter)],
                segment_contraints,
                boundary,
                holes,
            );
        }
    }

    return split_map;
} /* end - encroach_map */

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
        let mapping = encroachment::unencroach(
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

        // let rho_mean: f64 = 1.4142135623730951;
        let mut irregular_triangles = find_irregular(
            &mut triangulation,
            &RefineParams {
                max_area: 0.0, /* not used */
                quality_ratio: 1.0,
            },
        );

        split_irregular(
            &mut triangulation,
            &mut irregular_triangles,
            &segment_constraints,
            &Some(Rc::clone(&boundary)),
            &HashSet::new(),
        );

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
