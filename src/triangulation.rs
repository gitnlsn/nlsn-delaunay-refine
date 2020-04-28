use std::fmt;

pub struct Triangulation {
    pub coordinates: Vec<f64>,
    pub triangles: Vec<usize>,
}

impl Triangulation {
    pub fn from(coordinates: Vec<f64>, triangles: Vec<usize>) -> Self {
        Self {
            coordinates: coordinates,
            triangles: triangles,
        }
    }
}

impl fmt::Display for Triangulation {
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
        write!(formatter, "\n");

        return write!(formatter, "");
    }
}
