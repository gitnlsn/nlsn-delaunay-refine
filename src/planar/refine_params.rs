/// Why refinement stopped.
///
/// Refinement is not unconditionally guaranteed to terminate: Ruppert's
/// algorithm can cycle indefinitely when two input segments meet at a small
/// angle, because splitting either one encroaches the other. A browser cannot
/// afford to find that out by hanging, so the loop is bounded and reports how
/// it exited. Anything other than `Converged` means the returned mesh is valid
/// but does not meet the requested quality everywhere.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RefineOutcome {
    /// No triangle violates the quality or area constraints.
    Converged,
    /// Hit `max_steps`. Usually a small input angle cycling.
    HitStepLimit,
    /// Hit `max_triangles`. Usually `max_area` set far too small for the domain.
    HitSizeLimit,
    /// Refinement ran to completion, but this many elements could not be
    /// improved to the requested quality.
    ///
    /// Distinct from the limits: nothing was truncated and no more work was
    /// available. It means the target is not reachable on this input -- most
    /// often at an input corner sharper than roughly twice the requested angle,
    /// where the elements filling the wedge are forced to be at least as bad as
    /// the corner. The honest response is to change the geometry or accept the
    /// local quality, so the count is carried rather than swallowed.
    LeftUnimprovable(usize),
}

impl RefineOutcome {
    pub fn terminated_early(&self) -> bool {
        !matches!(self, RefineOutcome::Converged)
    }
}

#[derive(Debug, Clone)]
pub struct RefineParams {
    pub max_area: Option<f64>,
    /// Radius-to-shortest-edge ratio bound. Lower is stricter. Prefer
    /// constructing via [`RefineParams::from_min_angle_deg`], which expresses
    /// the same constraint in the units users actually think in.
    pub quality_ratio: f64,
    /// Maximum refinement iterations before giving up.
    pub max_steps: usize,
    /// Maximum triangles before giving up.
    ///
    /// Counted in triangles rather than vertices specifically because the
    /// triangle count is O(1) to read off the triangulation, while counting
    /// distinct vertices means walking every triangle and deduplicating. This
    /// check runs on every refinement iteration, so an O(n) version turns the
    /// safety limit itself into the dominant cost and makes a capped run take
    /// minutes.
    pub max_triangles: usize,
}

/// Ruppert's algorithm is only *provably* terminating for a radius-edge ratio
/// above sqrt(2), i.e. a minimum angle at or below ~20.7 degrees. Above that it
/// usually still works, but termination is empirical, which is why the step cap
/// exists.
pub const PROVABLE_TERMINATION_RATIO: f64 = std::f64::consts::SQRT_2;

impl Default for RefineParams {
    fn default() -> Self {
        Self {
            max_area: None,
            quality_ratio: 1.0,
            max_steps: Self::DEFAULT_MAX_STEPS,
            max_triangles: Self::DEFAULT_MAX_TRIANGLES,
        }
    }
}

impl RefineParams {
    /// Refinement iteration ceiling.
    ///
    /// Targets above ~20.7 degrees are outside Ruppert's provable-termination
    /// regime and will happily refine until *something* stops them, so this is
    /// the limit that actually fires in practice.
    pub const DEFAULT_MAX_STEPS: usize = 20_000;

    /// Element ceiling, chosen as a memory bound rather than a quality one.
    ///
    /// This kernel stores each triangle as an `Rc<Triangle>` plus three
    /// `Rc<Edge>` entries in the adjacency map, which lands somewhere around
    /// 400-500 bytes per element once allocator and hash-table overhead are
    /// counted. At 50k elements that is roughly 25 MB -- comfortable in a
    /// browser worker, and safe to run several of in parallel under `cargo
    /// test`. A million elements, the previous value here, is closer to half a
    /// gigabyte per run and will drive a test machine into swap.
    ///
    /// 50k triangles is far beyond what this tool meshes interactively; raise
    /// it deliberately for batch work, with the memory cost in mind.
    pub const DEFAULT_MAX_TRIANGLES: usize = 50_000;

    /// Build parameters from a minimum angle in degrees.
    ///
    /// The refinement kernel bounds the circumradius-to-shortest-edge ratio
    /// `rho`, which relates to the smallest angle `theta` of a triangle by
    /// `rho = 1 / (2 * sin(theta))`. Users think in degrees, so the conversion
    /// belongs here rather than in the UI.
    pub fn from_min_angle_deg(min_angle_deg: f64, max_area: Option<f64>) -> Self {
        let theta = min_angle_deg.to_radians();
        let quality_ratio = 1.0 / (2.0 * theta.sin());

        Self {
            max_area,
            quality_ratio,
            ..Default::default()
        }
    }

    /// The minimum angle, in degrees, implied by `quality_ratio`.
    pub fn min_angle_deg(&self) -> f64 {
        (1.0 / (2.0 * self.quality_ratio)).asin().to_degrees()
    }

    /// Whether this quality target sits inside Ruppert's provable-termination
    /// regime. Outside it, refinement is best-effort and may hit the step cap.
    pub fn has_provable_termination(&self) -> bool {
        self.quality_ratio > PROVABLE_TERMINATION_RATIO
    }
}

#[cfg(test)]
mod refine_params {
    use super::*;

    #[test]
    fn min_angle_roundtrips_through_quality_ratio() {
        for deg in [5.0, 15.0, 20.7, 25.0, 30.0] {
            let params = RefineParams::from_min_angle_deg(deg, None);
            assert!(
                (params.min_angle_deg() - deg).abs() < 1.0E-9,
                "{} degrees did not roundtrip",
                deg
            );
        }
    }

    /// 30 degrees is a common default but is already outside the regime where
    /// Ruppert is proven to terminate. Worth asserting so the boundary is not
    /// silently moved.
    #[test]
    fn thirty_degrees_is_not_provably_terminating() {
        assert!(!RefineParams::from_min_angle_deg(30.0, None).has_provable_termination());
        assert!(RefineParams::from_min_angle_deg(20.0, None).has_provable_termination());
    }
}
