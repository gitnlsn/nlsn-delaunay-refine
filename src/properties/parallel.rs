use crate::elements::vertex::*;

/**
 * Checks if ab is parallel to cd
 *
 * The two segments meet in exactly one point iff the 2x2 system that solves for
 * that point is invertible, so this is a test for a singular coefficient matrix
 *
 * ```text
 * [-(y2-y1)  (x2-x1)]
 * [-(y4-y3)  (x4-x3)]
 * ```
 *
 * whose determinant reduces to the cross product of the direction vectors.
 * The comparison is against exact zero, matching the singularity test this
 * previously delegated to nalgebra.
 */
pub fn parallel(a: &Vertex, b: &Vertex, c: &Vertex, d: &Vertex) -> bool {
    let x1 = a.x;
    let y1 = a.y;

    let x2 = b.x;
    let y2 = b.y;

    let x3 = c.x;
    let y3 = c.y;

    let x4 = d.x;
    let y4 = d.y;

    let determinant = -(y2 - y1) * (x4 - x3) - (x2 - x1) * -(y4 - y3);
    return determinant == 0.0;
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn sample_1() {
        let v1 = Vertex::new(0.0, 0.0);
        let v2 = Vertex::new(2.0, 2.0);
        let v3 = Vertex::new(2.0, 0.0);
        let v4 = Vertex::new(0.0, 2.0);
        assert!(!parallel(&v1, &v2, &v3, &v4));
        
        let v1 = Vertex::new(0.0, 0.0);
        let v2 = Vertex::new(2.0, 2.0);
        let v3 = Vertex::new(0.0, 1.0);
        let v4 = Vertex::new(2.0, 3.0);
        assert!(parallel(&v1, &v2, &v3, &v4));
    }
}