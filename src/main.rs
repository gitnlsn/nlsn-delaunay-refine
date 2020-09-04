extern crate structopt;
extern crate structopt_derive;
use structopt::StructOpt;

#[derive(StructOpt, Debug)]
#[structopt(
    name = "nlsn-delaunay",
    author = "Nelson Kenzo Tamashiro <nelsonkenzotamashiro@gmail.com>",
    version = "0.1.0",
    about = "Delaunay Triangulation and Refinement cli-tool"
)]
pub struct CliOptions {
    #[structopt(short, long, help = "input filename")]
    input: String,

    #[structopt(short, long, help = "output filename")]
    output: Option<String>,

    #[structopt(short, long, help = "displays triangulation result in opengl window")]
    show: bool,
}

mod glium_interface;
mod json_serializar;
mod triangulator_interface;

fn main() {
    let options: CliOptions = CliOptions::from_args();
    
    let file_path_string = options.input;

    let file_path = std::path::Path::new(&file_path_string);
    let triangulation_input = match json_serializar::io::read(file_path) {
        Some(triangulation_input) => triangulation_input,
        None => {
            panic!("Failed to deserialize triangulation json data");
        }
    };

    let (mut triangulator, refine_params) =
        match triangulator_interface::parse(&triangulation_input) {
            Ok((triangulator, refine_params)) => (triangulator, refine_params),
            Err(_) => {
                panic!("Failed to parse triangulation input data");
            }
        };

    triangulator.triangulate();
    triangulator.refine(refine_params);

    let output_triangulation =
        json_serializar::models::output::TriangulationOutput::from_triangulator(
            &triangulation_input,
            &triangulator,
        );

    let output_string = serde_json::to_string_pretty(&output_triangulation).unwrap();

    if let Some(output_path_string) = options.output {
        let file_path = std::path::Path::new(&output_path_string);

        match json_serializar::io::write(&file_path, output_string) {
            Ok(_) => {}
            Err(_) => {
                panic!("Failed to write triangulation output to file");
            }
        }
    } else {
        println!("{}", output_string);
    }

    if options.show {
        let (display, event_loop) = glium_interface::display::new();
        let edges_data = glium_interface::vertex::Vertex::edges_from_triangulation(
            &triangulator.triangulation.borrow(),
        );
        glium_interface::edges::draw((display, event_loop), edges_data, 1.0);
        
    }
}
