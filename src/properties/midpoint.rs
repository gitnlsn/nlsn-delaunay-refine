use crate::elements::vertex::*;

pub fn midpoint(v1: &Vertex, v2: &Vertex) -> Vertex {
    if v1.is_ghost || v2.is_ghost {
        return Vertex::new_ghost();
    }
    let midpoint_x = (v1.x + v2.x) / 2.0;
    let midpoint_y = (v1.y + v2.y) / 2.0;
    return Vertex::new(midpoint_x, midpoint_y);
}

#[cfg(test)]
mod midpoint_calculation {
    use super::*;

    #[test]
    fn trivial() {
        let v1 = Vertex::new(1.0, 1.0);
        let v2 = Vertex::new(3.0, 2.0);
        let v3 = Vertex::new_ghost();

        let mid = midpoint(&v1, &v2);
        assert_eq!(mid.x, 2.0);
        assert_eq!(mid.y, 1.5);
        
        assert!(midpoint(&v1, &v3).is_ghost);
        assert!(midpoint(&v3, &v2).is_ghost);
    }
}
