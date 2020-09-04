use nlsn_delaunay::{elements::polyline::*, properties::continence::*};

use std::collections::HashSet;
use std::rc::Rc;

/**
 * Determines the boundary possibly defined by inclusion
 * and removal of polylines, looking for the largest continuous
 * domain. Every include will be united. All removals will be
 * subtracted from union of includes.
 * If includes is empty, Err is returned.
 * If any include is sepparated from the remaining, Err is returned.
 * If any removal, splits the union in two or more, Err is returned.
 */
pub fn boundary(
    includes: &Vec<Rc<Polyline>>,
    removes: &Vec<Rc<Polyline>>,
) -> Result<Rc<Polyline>, ()> {
    if includes.is_empty() {
        return Err(());
    }

    let mut includes: HashSet<Rc<Polyline>> = includes.iter().cloned().collect();

    let mut boundary = Rc::clone(includes.iter().next().unwrap());
    includes.remove(&boundary);

    for _ in 0..includes.len() {
        for possible_include in includes.iter().cloned() {
            if let Some((union, _)) = Polyline::union(&boundary, &possible_include) {
                boundary = Rc::new(union);
                includes.remove(&possible_include);
                break;
            }
        }
    }

    if !includes.is_empty() {
        return Err(());
    }

    for possible_removal in removes.iter() {
        let (subtraction_list, _) = Polyline::subtraction(&boundary, possible_removal);

        if subtraction_list.len() > 1 {
            /* divided union in more than 1 */
            return Err(());
        }
        if subtraction_list.len() == 1 {
            boundary = Rc::clone(subtraction_list.get(0).unwrap());
        }
    }

    return Ok(boundary);
}

/**
 * Determines all holes that are contained by the boundary
 * and unite holes, if they have any interesection.
 */
pub fn holes(boundary: &Rc<Polyline>, removes: &Vec<Rc<Polyline>>) -> HashSet<Rc<Polyline>> {
    let mut holes: HashSet<Rc<Polyline>> = HashSet::new();

    if removes.is_empty() {
        return holes;
    }

    /* clone removes, avoiding data mutation */
    let mut removes: Vec<Rc<Polyline>> = removes.iter().cloned().collect();

    while !removes.is_empty() {
        let possible_removal = Rc::clone(&removes.pop().unwrap());
        if Polyline::continence(boundary, &possible_removal)
            != Some((Continence::Inside, BoundaryInclusion::Open))
        {
            /* ignore outer removals */
            continue;
        }

        for existing_hole in holes.iter().cloned() {
            if let Some((union, _)) = Polyline::union(&existing_hole, &possible_removal) {
                holes.remove(&existing_hole);
                removes.push(Rc::new(union));
                break;
            }
        }

        holes.insert(possible_removal);
    }

    return holes;
}
