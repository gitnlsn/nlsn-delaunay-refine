use std::fmt;

/**
 * Triangulation is defined by point coordinates and triangle indices.
 *  - Each pair of f64 values in coordinates Vec<f64> define the (x,y) 
 * coordiates of a point.
 *  - Each index in triangles Vec<usize> points to a (x,y) coordinate.
 *  - Every three indices in triangles Vec<usize> define a triangle by 
 * its coordinates.
 *  - Coordinates Vec must be 2*n, where n is the quantity of points.
 *  - Triangles Vec must be 3*t, where t is the quantity of triangles.
 */
pub struct TriangulationData {
    pub coordinates: Vec<f64>,
    pub triangles: Vec<usize>,
}

impl TriangulationData {
    pub fn from(coordinates: Vec<f64>, triangles: Vec<usize>) -> Self {
        Self {
            coordinates: coordinates,
            triangles: triangles,
        }
    }
}

impl fmt::Display for TriangulationData {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "Coordinates\n");
        for index in 0..self.coordinates.len() / 2 {
            let x = self.coordinates.get(index * 2).unwrap();
            let y = self.coordinates.get(index * 2 + 1).unwrap();
            write!(formatter, "{} {}\n", x, y);
        }
        write!(formatter, "\n");

        write!(formatter, "Triangles\n");
        for index in 0..self.triangles.len() / 3 {
            let v1 = self.triangles.get(index * 3).unwrap();
            let v2 = self.triangles.get(index * 3 + 1).unwrap();
            let v3 = self.triangles.get(index * 3 + 2).unwrap();
            write!(formatter, "{} {} {}\n", v1, v2, v3);
        }

        return write!(formatter, "");
    }
}
