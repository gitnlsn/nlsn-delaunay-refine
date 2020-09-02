#![macro_use]
extern crate glium;

use glium::{glutin, Display};

pub fn new() -> (Display, glutin::event_loop::EventLoop<()>) {
    let event_loop = glutin::event_loop::EventLoop::new();
    let wb = glutin::window::WindowBuilder::new();
    let cb = glutin::ContextBuilder::new();
    let display = glium::Display::new(wb, cb, &event_loop).unwrap();
    return (display, event_loop);
}
