use crate::continence::*;
use crate::vertex::*;
use std::rc::Rc;

struct Edge {
    pub v1: Rc<Vertex>,
    pub v2: Rc<Vertex>,
}

impl Edge {
    pub fn new(v1: Rc<Vertex>, v2: Rc<Vertex>) -> Self {
        Self {
            v1: v1,
            v2: v2,
        }
    }

    pub fn length(&self) -> f64 {
        let x1 = self.v1.x;
        let y1 = self.v1.y;
        let x2 = self.v2.x;
        let y2 = self.v2.y;

        return ((x2 - x1).powi(2) + (y2 - y1).powi(2)).sqrt();
    }

    pub fn encroach(&self, vertex: &Vertex) -> Continence {
        let x = vertex.x;
        let y = vertex.y;
        let x1 = self.v1.x;
        let y1 = self.v1.y;
        let x2 = self.v2.x;
        let y2 = self.v2.y;
        
        let measure = (x-x2) * (x-x1) + (y-y2) * (y-y1);

        if measure > 0.0 {
            return Continence::Outside;
        } else if measure < 0.0 {
            return Continence::Inside;
        } else {
            return Continence::Boundary;
        }
    }

    pub fn midpoint(&self) -> Vertex {
        let x1 = self.v1.x;
        let y1 = self.v1.y;
        let x2 = self.v2.x;
        let y2 = self.v2.y;

        let x_mid = (x1 + x2) / 2.0;
        let y_mid = (y1 + y2) / 2.0;
        
        return Vertex::new(x_mid, y_mid);
    }
}

#[cfg(test)]
mod length {
    use super::*;

    #[test]
    fn test_length() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.0));

        let edge = Edge::new(v1, v2);
        assert!((edge.length() - (2.0 as f64).sqrt()).abs() < 0.00000001);
    }
}
#[cfg(test)]
mod encroach {
    use super::*;

    #[test]
    fn test_inside() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.0));
        
        let trial_vertex = Rc::new(Vertex::new(0.0, 0.99));

        let edge = Edge::new(v1, v2);
        assert_eq!(edge.encroach(&trial_vertex), Continence::Inside);
    }

    #[test]
    fn test_outside() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.0));
        
        let trial_vertex = Rc::new(Vertex::new(0.0, 1.01));

        let edge = Edge::new(v1, v2);
        assert_eq!(edge.encroach(&trial_vertex), Continence::Outside);
    }

    #[test]
    fn test_boundary() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.0));
        
        let trial_vertex = Rc::new(Vertex::new(0.0, 1.0));

        let edge = Edge::new(v1, v2);
        assert_eq!(edge.encroach(&trial_vertex), Continence::Boundary);
    }
}
#[cfg(test)]
mod midpoint {
    use super::*;

    #[test]
    fn name() {
        let v1 = Rc::new(Vertex::new(0.0, 0.0));
        let v2 = Rc::new(Vertex::new(1.0, 1.2));
        
        let edge = Edge::new(v1, v2);
        let midpoint = edge.midpoint();
        assert_eq!(midpoint.x, 0.5);
        assert_eq!(midpoint.y, 0.6);
    }
}
