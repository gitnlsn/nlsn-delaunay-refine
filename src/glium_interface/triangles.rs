#![macro_use]
extern crate glium;
use crate::glium_interface::vertex;

use glium::{glutin, Display, Program, Surface};

/**
 *  Creates default triangle drawing program
 */
fn get_program(display: &Display) -> Program {
    let vertex_shader_src = r#"
        #version 140

        in vec2 position;

        void main() {
            gl_Position = vec4(position, 0.0, 1.0);
        }
    "#;

    let fragment_shader_src = r#"
        #version 140

        out vec4 color;

        void main() {
            color = vec4(1.0, 0.0, 0.0, 1.0);
        }
    "#;

    let program =
        Program::from_source(display, vertex_shader_src, fragment_shader_src, None).unwrap();

    return program;
}

pub fn draw(
    (display, event_loop): (Display, glutin::event_loop::EventLoop<()>),
    shape: Vec<vertex::Vertex>,
) {
    let vertex_buffer = glium::VertexBuffer::new(&display, &shape).unwrap();
    let indices = glium::index::NoIndices(glium::index::PrimitiveType::TrianglesList);

    let program = get_program(&display);

    event_loop.run(move |ev, _, control_flow| {
        let mut target = display.draw();
        target.clear_color(1.0, 1.0, 1.0, 1.0);
        target
            .draw(
                &vertex_buffer,
                &indices,
                &program,
                &glium::uniforms::EmptyUniforms,
                &glium::DrawParameters {
                    ..Default::default()
                },
            )
            .unwrap();
        target.finish().unwrap();

        let next_frame_time =
            std::time::Instant::now() + std::time::Duration::from_nanos(16_666_667);

        *control_flow = glutin::event_loop::ControlFlow::WaitUntil(next_frame_time);

        match ev {
            glutin::event::Event::WindowEvent { event, .. } => match event {
                glutin::event::WindowEvent::CloseRequested => {
                    *control_flow = glutin::event_loop::ControlFlow::Exit;
                    return;
                }
                _ => return,
            },
            _ => (),
        }
    });
}
