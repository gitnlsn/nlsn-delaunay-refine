//! Laws that every single refinement operation must obey, checked one at a time.
//!
//! `tests/invariants.rs` measures the finished mesh. It is the right instrument
//! for "is this mesh usable", and the wrong one for "what broke it": by the time
//! it looks, thousands of operations have run, and a mesh covering 2237% of its
//! domain gives no clue which of them did it. `tests/minimize.rs` got as far as
//! narrowing the reported failure to a single midpoint insertion that "removes
//! one triangle and creates three", and then stalled, because nothing could show
//! what that insertion actually did.
//!
//! So this file works from the recording described in
//! `src/planar/refine_procedures/trace.rs`, which captures every call to
//! `include_triangle` and `remove_triangle` in order. The events are grouped
//! back into the operations that produced them, and each group is checked
//! against the laws below. The first group that breaks one is the answer, and it
//! is reported with the coordinates the mesher actually held.
//!
//! The laws are deliberately local. Each is a statement about one operation in
//! isolation, so a failure names an operation rather than a mesh.
//!
//!   1. **Area neutrality.** Retriangulating a cavity cannot change the area the
//!      cavity covers, so the elements removed and the elements added must sum
//!      to the same area.
//!   2. **Star-shapedness.** Every element added by an insertion has the
//!      inserted vertex as a corner. One that does not was built against
//!      something that is not the cavity.
//!   3. **Containment.** Every element added lies within the region the removed
//!      elements covered. This is the one that catches new elements laid over
//!      ground that was never taken out -- the reported failure.
//!   4. **Effective removal.** No call to `remove_triangle` misses. A miss means
//!      an element the kernel believes it deleted is still in the mesh.
//!
//! Each is checked against the *net* effect of a group, not its raw events.
//! Refinement legitimately inserts a circumcentre and then rolls the insertion
//! back when it turns out to encroach a constrained segment; the raw events
//! there are a fan added and removed and the originals restored, and the net
//! effect is nothing at all. Netting means the laws see the no-op that a
//! complete rollback is -- and see the residue that an incomplete one leaves.

mod invariants;

use invariants::Domain;

use nlsn_delaunay::elements::vertex::Vertex;
use nlsn_delaunay::planar::refine_procedures::trace::{Corners, Event, Journal, StepKind};
use nlsn_delaunay::properties::orientation::{orientation, Orientation};

use std::collections::HashMap;

// ---------------------------------------------------------------------------
// Grouping the recording back into operations
// ---------------------------------------------------------------------------

/// A triangle's identity for the purpose of cancelling an event against its
/// opposite. Exact bits, sorted so the same triangle read in a different
/// rotation cancels with itself.
///
/// Deliberately *not* the kernel's own `Triangle` identity. Using that here
/// would make the instrument share whatever assumption is under suspicion; if
/// `Hash` and `Eq` disagree in the kernel, a harness built on them would report
/// the mesher as consistent with itself and see nothing.
type Key = [(u64, u64); 3];

fn key_of(corners: &Corners) -> Key {
    let mut key = [
        (corners[0].0.to_bits(), corners[0].1.to_bits()),
        (corners[1].0.to_bits(), corners[1].1.to_bits()),
        (corners[2].0.to_bits(), corners[2].1.to_bits()),
    ];
    key.sort();
    key
}

/// One mesh operation and everything it did.
pub struct Step {
    pub index: usize,
    pub kind: StepKind,
    pub inserted: Vec<(f64, f64)>,
    pub outcome: Vec<String>,
    /// Solid elements this operation left behind, after cancellation.
    pub added: Vec<Corners>,
    /// Solid elements it took out, after cancellation.
    pub taken: Vec<Corners>,
    /// Calls to `remove_triangle` that found nothing to remove.
    pub failed_removes: Vec<Corners>,
    /// Calls to `include_triangle` on an element already present.
    pub duplicate_includes: Vec<Corners>,
    /// Fan triangles the degeneracy guard threw away.
    pub refused: Vec<Corners>,
    /// Edges the cavity walk could not resolve to a neighbouring element.
    pub adjacency_missing: Vec<[(f64, f64); 2]>,
    /// Ghost elements added and removed, netted the same way.
    ///
    /// Kept apart from the solid ones because they cover no domain and must not
    /// enter any area sum -- but they own adjacency entries exactly as solid
    /// elements do, so a step that fails to maintain them leaves the same kind
    /// of hole in the map. That is invisible in every other column here.
    pub ghosts_added: Vec<Corners>,
    pub ghosts_taken: Vec<Corners>,
}

/// Splits the recording at each `Step` marker and nets each group's mutations.
pub fn group(journal: &Journal) -> Vec<Step> {
    let mut steps: Vec<Step> = Vec::new();

    // Net effect, as two maps keyed by triangle. An include cancels a pending
    // remove of the same element and vice versa, which is what reduces a
    // complete rollback to nothing.
    let mut added: HashMap<Key, Corners> = HashMap::new();
    let mut taken: HashMap<Key, Corners> = HashMap::new();
    let mut ghosts_added: HashMap<Key, Corners> = HashMap::new();
    let mut ghosts_taken: HashMap<Key, Corners> = HashMap::new();

    // A step is only written out once the next one begins, so the maps above
    // belong to the step currently being read.
    macro_rules! flush {
        () => {
            if let Some(step) = steps.last_mut() {
                let drain = |map: &HashMap<Key, Corners>| {
                    let mut v: Vec<Corners> = map.values().cloned().collect();
                    v.sort_by(|x, y| key_of(x).cmp(&key_of(y)));
                    v
                };
                step.added = drain(&added);
                step.taken = drain(&taken);
                step.ghosts_added = drain(&ghosts_added);
                step.ghosts_taken = drain(&ghosts_taken);
            }
            added.clear();
            taken.clear();
            ghosts_added.clear();
            ghosts_taken.clear();
        };
    }

    for event in journal.events.iter() {
        match event {
            Event::Step { index, kind } => {
                flush!();
                steps.push(Step {
                    index: *index,
                    kind: kind.clone(),
                    inserted: Vec::new(),
                    outcome: Vec::new(),
                    added: Vec::new(),
                    taken: Vec::new(),
                    failed_removes: Vec::new(),
                    duplicate_includes: Vec::new(),
                    refused: Vec::new(),
                    adjacency_missing: Vec::new(),
                    ghosts_added: Vec::new(),
                    ghosts_taken: Vec::new(),
                });
            }
            Event::Insert { vertex, .. } => {
                if let Some(step) = steps.last_mut() {
                    step.inserted.push(*vertex);
                }
            }
            Event::Outcome(label) => {
                if let Some(step) = steps.last_mut() {
                    step.outcome.push(label.clone());
                }
            }
            Event::Refused { corners } => {
                if let Some(step) = steps.last_mut() {
                    step.refused.push(*corners);
                }
            }
            Event::AdjacencyMissing { edge } => {
                if let Some(step) = steps.last_mut() {
                    step.adjacency_missing.push(*edge);
                }
            }
            Event::Included {
                corners,
                ghost,
                accepted,
            } => {
                if !accepted {
                    if let Some(step) = steps.last_mut() {
                        step.duplicate_includes.push(*corners);
                    }
                    continue;
                }
                // Ghosts carry the hull rather than the domain, and cover no
                // area, so they are netted separately -- putting their synthetic
                // (0, 0) corner into an area sum would be meaningless.
                let key = key_of(corners);
                let (into, from) = if *ghost {
                    (&mut ghosts_added, &mut ghosts_taken)
                } else {
                    (&mut added, &mut taken)
                };
                if from.remove(&key).is_none() {
                    into.insert(key, *corners);
                }
            }
            Event::Removed {
                corners,
                ghost,
                found,
            } => {
                if !found {
                    if let Some(step) = steps.last_mut() {
                        step.failed_removes.push(*corners);
                    }
                    continue;
                }
                let key = key_of(corners);
                let (into, from) = if *ghost {
                    (&mut ghosts_taken, &mut ghosts_added)
                } else {
                    (&mut taken, &mut added)
                };
                if from.remove(&key).is_none() {
                    into.insert(key, *corners);
                }
            }
        }
    }
    flush!();

    steps
}

// ---------------------------------------------------------------------------
// Geometry, computed here rather than borrowed from the kernel
// ---------------------------------------------------------------------------

fn area_of(c: &Corners) -> f64 {
    let (x1, y1) = c[0];
    let (x2, y2) = c[1];
    let (x3, y3) = c[2];
    ((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0).abs()
}

fn total_area(triangles: &[Corners]) -> f64 {
    triangles.iter().map(area_of).sum()
}

fn centroid(c: &Corners) -> (f64, f64) {
    (
        (c[0].0 + c[1].0 + c[2].0) / 3.0,
        (c[0].1 + c[1].1 + c[2].1) / 3.0,
    )
}

/// Whether a point lies inside a triangle or on its border, by the exact
/// predicate. `tests/alignment_axioms.rs` establishes that this answers the same
/// way whatever the coordinate frame, so a containment failure here is about the
/// mesh and not about the rotation.
fn contains(c: &Corners, point: (f64, f64)) -> bool {
    let v = |p: (f64, f64)| std::rc::Rc::new(Vertex::new(p.0, p.1));
    let (a, b, d, p) = (v(c[0]), v(c[1]), v(c[2]), v(point));

    let sides = [
        orientation(&a, &b, &p),
        orientation(&b, &d, &p),
        orientation(&d, &a, &p),
    ];
    let any_cw = sides.iter().any(|s| *s == Orientation::Clockwise);
    let any_ccw = sides.iter().any(|s| *s == Orientation::Counterclockwise);
    !(any_cw && any_ccw)
}

fn show_triangle(c: &Corners) -> String {
    format!(
        "({:?}, {:?}) ({:?}, {:?}) ({:?}, {:?})  area {:?}",
        c[0].0,
        c[0].1,
        c[1].0,
        c[1].1,
        c[2].0,
        c[2].1,
        area_of(c)
    )
}

/// The full record of one operation, for a failure message.
fn describe(step: &Step) -> String {
    let mut out = String::new();
    let header = match &step.kind {
        StepKind::BadTriangle { corners, quality } => format!(
            "step {}  bad element  {}  quality {:?}",
            step.index,
            show_triangle(corners),
            quality
        ),
        StepKind::SegmentSplit { segment } => format!(
            "step {}  split segment  ({:?}, {:?}) -- ({:?}, {:?})  length {:?}",
            step.index,
            segment[0].0,
            segment[0].1,
            segment[1].0,
            segment[1].1,
            ((segment[1].0 - segment[0].0).powi(2) + (segment[1].1 - segment[0].1).powi(2)).sqrt()
        ),
    };
    out.push_str(&header);
    out.push('\n');

    for vertex in step.inserted.iter() {
        out.push_str(&format!("    insert  ({:?}, {:?})\n", vertex.0, vertex.1));
    }
    for label in step.outcome.iter() {
        out.push_str(&format!("    outcome {}\n", label));
    }
    for c in step.taken.iter() {
        out.push_str(&format!("    -       {}\n", show_triangle(c)));
    }
    for c in step.added.iter() {
        out.push_str(&format!("    +       {}\n", show_triangle(c)));
    }
    for c in step.ghosts_taken.iter() {
        out.push_str(&format!("    -ghost  ({:?}, {:?}) ({:?}, {:?}) ({:?}, {:?})\n",
            c[0].0, c[0].1, c[1].0, c[1].1, c[2].0, c[2].1));
    }
    for c in step.ghosts_added.iter() {
        out.push_str(&format!("    +ghost  ({:?}, {:?}) ({:?}, {:?}) ({:?}, {:?})\n",
            c[0].0, c[0].1, c[1].0, c[1].1, c[2].0, c[2].1));
    }
    for c in step.refused.iter() {
        out.push_str(&format!("    refused {}\n", show_triangle(c)));
    }
    for e in step.adjacency_missing.iter() {
        out.push_str(&format!(
            "    NO NEIGHBOUR   ({:?}, {:?}) -- ({:?}, {:?})\n",
            e[0].0, e[0].1, e[1].0, e[1].1
        ));
    }
    for c in step.failed_removes.iter() {
        out.push_str(&format!("    REMOVE MISSED  {}\n", show_triangle(c)));
    }
    for c in step.duplicate_includes.iter() {
        out.push_str(&format!("    DUPLICATE      {}\n", show_triangle(c)));
    }
    out.push_str(&format!(
        "    net     -{:?} +{:?}  =  {:?}\n",
        total_area(&step.taken),
        total_area(&step.added),
        total_area(&step.added) - total_area(&step.taken)
    ));
    out
}

// ---------------------------------------------------------------------------
// The laws
// ---------------------------------------------------------------------------

/// Relative to the domain, so the bound means the same thing at any scale and a
/// step that moves a millionth of the domain is still caught.
const AREA_TOLERANCE: f64 = 1.0E-12;

/// Law 1. Retriangulating a cavity cannot change the area it covers.
fn law_area_neutrality(step: &Step, domain_area: f64) -> Result<(), String> {
    let taken = total_area(&step.taken);
    let added = total_area(&step.added);
    let delta = added - taken;

    if delta.abs() / domain_area <= AREA_TOLERANCE {
        return Ok(());
    }

    Err(format!(
        "area neutrality: removed {:?} of area and added {:?}, a change of {:+?} \
         ({:+.4}% of the domain). {}",
        taken,
        added,
        delta,
        100.0 * delta / domain_area,
        if delta > 0.0 {
            "The new elements cover ground the removed ones did not, so the mesh \
             now overlaps itself here."
        } else {
            "The new elements do not cover everything the removed ones did, so \
             the mesh now has a gap here."
        }
    ))
}

/// Law 2. Every element an insertion adds has the inserted vertex as a corner.
///
/// Only meaningful where a vertex was actually inserted and kept. A step that
/// rolled its insertion back nets to nothing and has nothing to check.
fn law_star_shaped(step: &Step) -> Result<(), String> {
    if step.inserted.is_empty() || step.added.is_empty() {
        return Ok(());
    }

    let orphans: Vec<&Corners> = step
        .added
        .iter()
        .filter(|c| {
            !step
                .inserted
                .iter()
                .any(|v| c.iter().any(|corner| corner == v))
        })
        .collect();

    if orphans.is_empty() {
        return Ok(());
    }

    Err(format!(
        "star-shapedness: {} of {} new element(s) do not have any inserted vertex \
         as a corner, so they were not built by fanning the cavity about the new \
         point:\n{}",
        orphans.len(),
        step.added.len(),
        orphans
            .iter()
            .take(4)
            .map(|c| format!("      {}", show_triangle(c)))
            .collect::<Vec<String>>()
            .join("\n")
    ))
}

/// Law 3. Every element added lies inside the region the removed ones covered.
fn law_containment(step: &Step) -> Result<(), String> {
    if step.added.is_empty() {
        return Ok(());
    }

    if step.taken.is_empty() {
        return Err(format!(
            "containment: {} element(s) were created without any element being \
             removed, so they cover ground that was already meshed:\n{}",
            step.added.len(),
            step.added
                .iter()
                .take(4)
                .map(|c| format!("      {}", show_triangle(c)))
                .collect::<Vec<String>>()
                .join("\n")
        ));
    }

    let stray: Vec<&Corners> = step
        .added
        .iter()
        .filter(|c| {
            let centre = centroid(c);
            !step.taken.iter().any(|t| contains(t, centre))
        })
        .collect();

    if stray.is_empty() {
        return Ok(());
    }

    Err(format!(
        "containment: {} of {} new element(s) are centred outside every element \
         this step removed, so they were laid over ground that was never taken \
         out:\n{}",
        stray.len(),
        step.added.len(),
        stray
            .iter()
            .take(4)
            .map(|c| format!("      {}", show_triangle(c)))
            .collect::<Vec<String>>()
            .join("\n")
    ))
}

/// Law 5. Every edge on the rim of the cavity gets exactly one element built
/// against it.
///
/// The sharpest of the five, because it distinguishes the two ways a step can
/// come out short. Area neutrality says a step lost 97.5 units of area; it
/// cannot say whether the cavity walk never reached a rim edge, or reached it
/// and the degeneracy guard threw the result away. Those call for opposite
/// fixes, so the harness has to tell them apart.
///
/// The rim is the set of undirected edges appearing exactly once across the
/// removed elements -- an edge appearing twice is interior to the cavity and is
/// meant to disappear. A refused element counts as accounted for: refusing a
/// degenerate fan is correct, and the area it would have covered is zero.
///
/// One rim edge is covered differently and must not be reported: the one the
/// inserted point lies on, which is the ordinary case when a boundary segment is
/// split at its midpoint. There the fan triangle would be the segment itself, so
/// no single element spans the edge -- instead the two halves `a--m` and `m--b`
/// are edges of two different new elements. That is correct, and a law that
/// could not see it would fire on every segment split there is.
fn law_cavity_closure(step: &Step) -> Result<(), String> {
    if step.taken.is_empty() {
        return Ok(());
    }

    let mut usage: HashMap<[(u64, u64); 2], ((f64, f64), (f64, f64))> = HashMap::new();
    let mut counts: HashMap<[(u64, u64); 2], usize> = HashMap::new();

    for triangle in step.taken.iter() {
        for (a, b) in [
            (triangle[0], triangle[1]),
            (triangle[1], triangle[2]),
            (triangle[2], triangle[0]),
        ] {
            let mut key = [(a.0.to_bits(), a.1.to_bits()), (b.0.to_bits(), b.1.to_bits())];
            key.sort();
            usage.insert(key, (a, b));
            *counts.entry(key).or_insert(0) += 1;
        }
    }

    let has_corner = |c: &Corners, p: (f64, f64)| c.iter().any(|corner| *corner == p);
    let spanned = |a: (f64, f64), b: (f64, f64)| {
        step.added
            .iter()
            .chain(step.refused.iter())
            .any(|c| has_corner(c, a) && has_corner(c, b))
    };

    // An inserted vertex strictly between the two ends, which is what makes a
    // rim edge covered by two elements rather than one.
    let split_at = |a: (f64, f64), b: (f64, f64)| {
        step.inserted.iter().find(|m| {
            // Bounding box only. The point was computed as the midpoint of this
            // edge, so after a rotation it is collinear with it to within
            // rounding rather than exactly, and an exact collinearity test here
            // would answer "no" for the very case this exists to recognise.
            // Paired with the requirement below that both halves are real edges
            // of real elements, which a point off the line could not satisfy.
            let between = m.0 >= a.0.min(b.0)
                && m.0 <= a.0.max(b.0)
                && m.1 >= a.1.min(b.1)
                && m.1 <= a.1.max(b.1);
            between && **m != a && **m != b
        })
    };

    let unfilled: Vec<((f64, f64), (f64, f64))> = counts
        .iter()
        .filter(|(_, count)| **count == 1)
        .map(|(key, _)| usage[key])
        .filter(|(a, b)| {
            if spanned(*a, *b) {
                return false;
            }
            match split_at(*a, *b) {
                Some(m) => !(spanned(*a, *m) && spanned(*m, *b)),
                None => true,
            }
        })
        .collect();

    if unfilled.is_empty() {
        return Ok(());
    }

    Err(format!(
        "cavity closure: {} rim edge(s) of the removed region had no element \
         built against them -- neither included nor refused, so the cavity walk \
         never reached them and the region behind them is now unmeshed:\n{}",
        unfilled.len(),
        unfilled
            .iter()
            .take(4)
            .map(|(a, b)| format!(
                "      ({:?}, {:?}) -- ({:?}, {:?})",
                a.0, a.1, b.0, b.1
            ))
            .collect::<Vec<String>>()
            .join("\n")
    ))
}

/// Law 4. A removal that the kernel asked for must have happened.
fn law_effective_removal(step: &Step) -> Result<(), String> {
    if step.failed_removes.is_empty() {
        return Ok(());
    }

    Err(format!(
        "effective removal: {} call(s) to remove_triangle found nothing to \
         remove, so those elements are still in the mesh. Every caller discards \
         that return value, so the kernel believes they are gone:\n{}",
        step.failed_removes.len(),
        step.failed_removes
            .iter()
            .take(4)
            .map(|c| format!("      {}", show_triangle(c)))
            .collect::<Vec<String>>()
            .join("\n")
    ))
}

fn check(step: &Step, domain_area: f64) -> Vec<String> {
    [
        law_effective_removal(step),
        law_cavity_closure(step),
        law_area_neutrality(step, domain_area),
        law_containment(step),
        law_star_shaped(step),
    ]
    .into_iter()
    .filter_map(|r| r.err())
    .collect()
}

/// Runs every law over every step, and reports the first step that breaks one.
///
/// First rather than all: the steps after it are operating on a mesh that is
/// already wrong, so their failures are consequences and reporting them buries
/// the cause.
fn first_failure(domain: &Domain, min_angle_deg: f64, max_area: Option<f64>) -> Option<String> {
    let (_triangulator, outcome, journal) = domain.mesh_traced(min_angle_deg, max_area);
    let steps = group(&journal);
    let domain_area = domain.expected_area();

    for (position, step) in steps.iter().enumerate() {
        let failures = check(step, domain_area);
        if failures.is_empty() {
            continue;
        }

        let context: String = steps
            .iter()
            .skip(position.saturating_sub(2))
            .take(position.min(2) + 1)
            .map(describe)
            .collect::<Vec<String>>()
            .join("");

        return Some(format!(
            "{}: {} of {} steps ran before one broke {} law(s). Outcome was {:?}.\n\n\
             The failing step, with the two before it:\n\n{}\n{}",
            domain.name,
            position,
            steps.len(),
            failures.len(),
            outcome,
            context,
            failures
                .iter()
                .map(|f| format!("  [{}]\n", f))
                .collect::<Vec<String>>()
                .join("")
        ));
    }
    None
}

fn assert_every_step_is_lawful(domain: &Domain, min_angle_deg: f64, max_area: Option<f64>) {
    if let Some(report) = first_failure(domain, min_angle_deg, max_area) {
        panic!("{}", report);
    }
}

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

fn rotate(points: &[[f64; 2]], degrees: f64) -> Vec<[f64; 2]> {
    let n = points.len() as f64;
    let cx = points.iter().map(|p| p[0]).sum::<f64>() / n;
    let cy = points.iter().map(|p| p[1]).sum::<f64>() / n;
    let (s, c) = degrees.to_radians().sin_cos();
    points
        .iter()
        .map(|[x, y]| {
            let (dx, dy) = (x - cx, y - cy);
            [cx + dx * c - dy * s, cy + dx * s + dy * c]
        })
        .collect()
}

const TRIANGLE: [[f64; 2]; 3] = [[0.0, 0.0], [60.0, 0.0], [30.0, 52.0]];

fn triangle_at(degrees: f64) -> Domain {
    Domain {
        name: format!("triangle_rot_{}deg", degrees),
        boundary: rotate(&TRIANGLE, degrees),
        holes: vec![],
    }
}

/// The shape from the report: drawn by hand, so the base is a little off level
/// and no two sides are equal. Read off the screenshot against its grid, which
/// is 10 world units per line, and meshed at the settings shown -- 20 degrees
/// and 0.95% of the 100x100 world, which is what the app passes.
fn drawn_triangle() -> Domain {
    Domain {
        name: "drawn_triangle".to_string(),
        boundary: vec![[12.4, 13.4], [90.2, 8.1], [51.0, 88.2]],
        holes: vec![],
    }
}

/// The unit equilateral triangle from `triangle_split::split::sample_1`.
///
/// That test asserts an exact list of expected elements, so any change to the
/// refinement path fails it whether the new mesh is better or worse. This
/// fixture is here to answer which: it meshes the same shape at the same target
/// -- `quality_ratio` 1.0 is a 30 degree minimum angle -- and asks whether every
/// step obeys the laws. A green answer here says the golden list is stale rather
/// than that the mesh is wrong.
///
/// Note the apex, `0.8660254037844386`, is not exactly representable, so this is
/// off-axis geometry despite looking symmetric.
fn unit_equilateral() -> Domain {
    Domain {
        name: "unit_equilateral".to_string(),
        boundary: vec![[0.0, 0.0], [1.0, 0.0], [0.5, 0.8660254037844386]],
        holes: vec![],
    }
}

// ---------------------------------------------------------------------------
// The pairs
// ---------------------------------------------------------------------------

/// The green half. Axis-aligned, so every midpoint and circumcentre the mesher
/// computes from these coordinates is exactly representable and the two routes
/// to the same point agree bit for bit.
///
/// It has to pass for any of this to mean anything: it is what establishes that
/// the four laws are laws rather than an overstatement, and that the harness
/// reads the recording correctly.
#[test]
fn every_step_on_an_axis_aligned_triangle_is_lawful() {
    assert_every_step_is_lawful(&triangle_at(0.0), 25.0, Some(50.0));
}

/// The red half. The same triangle, turned two degrees. Nothing else differs.
///
/// This was the failing case, and the four defects it took to close it are worth
/// keeping written down, because each was invisible to every other test in the
/// crate and each was found by reading one step of this recording:
///
///   1. **A zero-area element per segment split.** Splitting a boundary segment
///      at its midpoint removed the one element that segment bounded and added
///      three: the two correct halves, plus one whose corners were the segment's
///      endpoints and its midpoint -- the segment itself. It covered no area, so
///      area conservation could not see it, but its three edges went into
///      `Triangulation::adjacency` and overwrote the real halves' entries. The
///      guard against it tested collinearity *exactly*, which is true only when
///      the segment is axis-aligned; rotate and the same three points turn by
///      about 1e-16 and the guard waves them through.
///   2. **Refused elements still filed in the conflict map.** The guard's
///      rejection did not skip `distribute_conflicts`, which files a triangle as
///      a *key*, and the next insertion picks its cavity seed from those keys --
///      handing it a seed that is not in the triangulation.
///   3. **The last hash-order site.** `split_irregular` iterated the encroached
///      segments straight out of a `HashSet` to choose which to split next.
///      Found by diffing two step logs: identical up to the second split,
///      different in which segment it was.
///   4. **Ghosts not split with their segment.** `may_insert_triangle` answered
///      a ghost by building the closed polyline `[v1, v2, M]` and asking
///      `Polyline::continence`; with `M` on the segment those are collinear, the
///      loop has no interior, and `continence` returns `None`, read as "may not
///      insert". The ghost stayed whole, nothing owned the edge from the
///      midpoint onward, and the next cavity walk to reach it found no
///      neighbour and built nothing -- a gap the size of one element, on the
///      boundary. And once ghosts did split, the ghost half-plane test had the
///      same exact-versus-representable problem as (1): a midpoint is collinear
///      with every *other* subsegment of its own segment, so a 1e-16 residual
///      made the cavity absorb ghosts belonging to those, and the hull healed
///      over the vertices between them.
///
/// Every one of them is the same shape of mistake: a question about what the
/// geometry *means* answered with a predicate about what the coordinates
/// exactly *are*. Axis-aligned input hid all four because there the two answers
/// coincide.
#[test]
fn every_step_on_a_rotated_triangle_is_lawful() {
    assert_every_step_is_lawful(&triangle_at(2.0), 25.0, Some(50.0));
}

/// A quarter of a degree, which is below what anyone would see on screen.
///
/// Worth keeping separate from the two-degree case: the natural assumption is
/// that a failure this large needs badly conditioned geometry, and it does not.
#[test]
fn every_step_on_a_barely_rotated_triangle_is_lawful() {
    assert_every_step_is_lawful(&triangle_at(0.25), 25.0, Some(50.0));
}

/// The reported case end to end: the hand-drawn triangle at the app's settings.
///
/// Kept alongside the synthetic rotations because it is the shape a person
/// actually drew, and a fix that repaired the rotations without repairing this
/// one would not have repaired the reported bug.
#[test]
fn every_step_on_the_drawn_triangle_is_lawful() {
    assert_every_step_is_lawful(&drawn_triangle(), 20.0, Some(95.0));
}

/// Whether the mesh `triangle_split::split::sample_1` asserts against is lawful.
///
/// See `unit_equilateral`. This exists to keep that golden test honest: if this
/// passes while the golden list does not, the list is out of date.
#[test]
fn every_step_on_the_unit_equilateral_is_lawful() {
    assert_every_step_is_lawful(&unit_equilateral(), 30.0, None);
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

/// Where the failure begins, as a function of how far the shape is turned.
///
/// A table rather than a threshold because `tests/minimize.rs` records that the
/// failure is not monotonic in the rotation: particular configurations fail and
/// their neighbours do not, so a single smallest-failing-angle would be
/// misleading.
#[test]
#[ignore = "diagnostic table; run with --nocapture"]
fn first_failing_step_by_rotation() {
    println!("\n  angle   first failing step / total");
    for degrees in [0.0, 0.25, 0.5, 1.0, 2.0, 5.0, 7.5, 15.0, 30.0, 31.0, 45.0, 47.3] {
        let domain = triangle_at(degrees);
        let (_t, _outcome, journal) = domain.mesh_traced(25.0, Some(50.0));
        let steps = group(&journal);
        let area = domain.expected_area();

        let first = steps
            .iter()
            .position(|s| !check(s, area).is_empty())
            .map(|p| p.to_string())
            .unwrap_or_else(|| "none".to_string());

        println!("  {:>5}   {:>4} / {}", degrees, first, steps.len());
    }
}

/// The step logs of eight identical runs must be identical.
///
/// A precondition for everything above. While the mesher is not deterministic,
/// no step index is quotable and no measurement of a fix is stable -- which is
/// the trap `tests/determinism_matrix.rs` was written to document.
///
/// It is also the sharpest determinism test in the crate: the matrix compares
/// finished meshes, so it can only say two runs differ, while this compares the
/// operations that built them and says at which one they first diverged. That is
/// how the last hash-order site was found.
#[test]
fn the_step_log_is_the_same_every_run() {
    let domain = triangle_at(2.0);

    let signature = || {
        let (_t, _outcome, journal) = domain.mesh_traced(25.0, Some(50.0));
        group(&journal)
            .iter()
            .map(describe)
            .collect::<Vec<String>>()
            .join("")
    };

    let reference = signature();
    for run in 1..8 {
        let this = signature();
        if this == reference {
            continue;
        }
        let at = reference
            .lines()
            .zip(this.lines())
            .position(|(a, b)| a != b)
            .unwrap_or(0);
        let window = |text: &str| {
            text.lines()
                .enumerate()
                .skip(0)
                .take(at + 4)
                .map(|(n, line)| format!("    {}{:>4}| {}", if n == at { ">" } else { " " }, n, line))
                .collect::<Vec<String>>()
                .join("\n")
        };
        panic!(
            "run {} diverged from run 0 at line {}.\n\n  run 0:\n{}\n\n  run {}:\n{}",
            run,
            at,
            window(&reference),
            run,
            window(&this),
        );
    }
}
