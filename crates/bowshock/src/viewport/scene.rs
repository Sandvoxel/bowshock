//! The shader widget program: input handling (camera navigation) and the per-frame primitive handed to the GPU pipeline.
//!
//! Navigation (design/components/Navigation): right-drag orbits, middle-drag pans, `Alt` + right-drag rolls, the wheel zooms about
//! the cursor, a trackpad's two-finger scroll orbits (pan in Profile), with `Shift` it pans and with `Cmd`/`Ctrl` it zooms.
//! A pinch zooms about the cursor (the vendored iced forwards winit's `PinchGesture`; `widgets::pinch`). Rotate gestures are not
//! forwarded, so `Alt` + right-drag is the roll, and `Cmd`/`Ctrl` + scroll is the zoom for machines the gesture does not reach.
//! The left button belongs to the selection and manipulation overlays above.

use std::sync::Arc;

use iced::time::{Duration, Instant};
use iced::widget::shader::{self, Action};
use iced::{keyboard, mouse, Point, Rectangle, Vector};

use super::camera::Camera;
use super::mesh::{Cut, Mesh};
use super::pipeline::{DrawPlan, Pipeline, Uniforms};
use super::{InsideView, Message};
use crate::tokens::Tokens;
use crate::widgets::pinch;

/// A vertex as the radial-part emitters write it; [`super::solids::Sink::adopt`] turns it into a [`super::solids::GpuVertex`].
#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    /// How the part is drawn: [`NORMAL`], [`GHOST`], [`HOVERED`] or [`SELECTED`].
    pub state: f32,
    /// 0 for the airframe's base colour, 1 for the fin colour (radial parts).
    pub material: f32,
}

/// Per-vertex part states (see `body.wgsl`).
pub const NORMAL: f32 = 0.0;
/// The part being dragged in: tinted towards the accent colour.
pub const GHOST: f32 = 1.0;
/// Under the cursor: slightly brighter.
pub const HOVERED: f32 = 2.0;
/// Selected: an accent tint on the silhouette edge.
pub const SELECTED: f32 = 3.0;

/// Everything the shader widget needs for one frame.
#[derive(Debug)]
pub struct Scene {
    pub camera: Camera,
    pub tokens: Tokens,
    pub mesh: Arc<Mesh>,
    /// The cursor to show over the model (a grab hand over a part, a resize arrow over a handle). Reported here rather than by
    /// the overlay above because a stack hides the cursor from the layers below any layer that reports a cursor of its own,
    /// which would stop middle-button navigation working over a part.
    pub interaction: mouse::Interaction,
    /// Whether orbiting is allowed (a 3D view rather than a locked side view).
    pub orbit: bool,
    /// How the model is shown.
    pub inside: InsideView,
    /// The airframe is ghosted around a selected internal part.
    pub reveal: bool,
    /// Exploded-view animation, 0 (together) to 1 (apart).
    pub explode: f32,
    /// The removed wedge in Cutaway.
    pub cut: Cut,
    /// Parts aft of this station have dropped away in the phase on show: drawn faint.
    pub fade_from: Option<f32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Gesture {
    Pan,
    Orbit,
    Roll,
}

#[derive(Debug, Clone, Copy)]
struct Drag {
    last: Point,
    gesture: Gesture,
}

#[derive(Debug, Default)]
pub struct Input {
    modifiers: keyboard::Modifiers,
    drag: Option<Drag>,
    last_click: Option<(Instant, Point)>,
    /// The widget size last reported (zero until the initial fit has been requested).
    size: iced::Size,
}

const DOUBLE_CLICK: Duration = Duration::from_millis(400);
const DOUBLE_CLICK_SLOP: f32 = 4.0;

impl shader::Program<Message> for Scene {
    type State = Input;
    type Primitive = Primitive;

    fn update(&self, state: &mut Input, event: &iced::Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<Message>> {
        if bounds.width > 0.0 && bounds.height > 0.0 && state.size != bounds.size() {
            // The first size triggers the initial fit; later ones only keep the view's idea of the size current.
            let first = state.size == iced::Size::ZERO;
            state.size = bounds.size();
            return Some(Action::publish(if first { Message::Fit(bounds.size()) } else { Message::Resized(bounds.size()) }));
        }
        match event {
            iced::Event::Keyboard(keyboard::Event::ModifiersChanged(modifiers)) => {
                state.modifiers = *modifiers;
                None
            }
            iced::Event::Keyboard(keyboard::Event::KeyPressed { key, modifiers, .. }) => {
                let plain = !modifiers.shift() && !modifiers.control() && !modifiers.alt() && !modifiers.logo();
                let is_f = matches!(key, keyboard::Key::Character(c) if c.as_str() == "f");
                (is_f && plain && cursor.position_over(bounds).is_some()).then(|| Action::publish(Message::Fit(bounds.size())).and_capture())
            }
            iced::Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Middle)) => {
                let position = cursor.position_over(bounds)?;
                let now = Instant::now();
                let double = state.last_click.is_some_and(|(at, p)| now.duration_since(at) <= DOUBLE_CLICK && position.distance(p) <= DOUBLE_CLICK_SLOP);
                if double {
                    state.last_click = None;
                    state.drag = None;
                    return Some(Action::publish(Message::Fit(bounds.size())).and_capture());
                }
                state.last_click = Some((now, position));
                let gesture = if state.modifiers.shift() && self.orbit { Gesture::Orbit } else { Gesture::Pan };
                state.drag = Some(Drag { last: position, gesture });
                Some(Action::capture())
            }
            iced::Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Right)) => {
                if !self.orbit {
                    return None;
                }
                let position = cursor.position_over(bounds)?;
                let gesture = if state.modifiers.alt() { Gesture::Roll } else { Gesture::Orbit };
                state.drag = Some(Drag { last: position, gesture });
                Some(Action::capture())
            }
            iced::Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Middle | mouse::Button::Right)) => state.drag.take().map(|_| Action::capture()),
            iced::Event::Mouse(mouse::Event::CursorMoved { position }) => {
                let drag = state.drag.as_mut()?;
                let delta = *position - drag.last;
                drag.last = *position;
                let message = match drag.gesture {
                    Gesture::Orbit => Message::Orbit(delta),
                    Gesture::Roll => Message::RollBy(delta),
                    Gesture::Pan => Message::Pan(delta),
                };
                Some(Action::publish(message).and_capture())
            }
            iced::Event::Mouse(mouse::Event::WheelScrolled { delta }) => {
                let position = cursor.position_over(bounds)?;
                let offset = position - bounds.center();
                let message = match *delta {
                    mouse::ScrollDelta::Lines { y, .. } => Message::Zoom { factor: 1.2f32.powf(y), offset },
                    mouse::ScrollDelta::Pixels { x, y } => {
                        if state.modifiers.logo() || state.modifiers.control() {
                            Message::Zoom { factor: (y * 0.01).exp(), offset }
                        } else if state.modifiers.shift() || !self.orbit {
                            Message::Pan(Vector::new(x, y))
                        } else {
                            Message::Orbit(Vector::new(-x, -y))
                        }
                    }
                };
                Some(Action::publish(message).and_capture())
            }
            // A trackpad pinch zooms about the cursor like the wheel does. Two-finger scroll stays orbit / pan (above).
            iced::Event::Mouse(mouse::Event::PinchGesture { delta, .. }) => {
                let position = cursor.position_over(bounds)?;
                let factor = pinch::factor(*delta);
                Some(Action::publish(Message::Zoom { factor, offset: position - bounds.center() }).and_capture())
            }
            _ => None,
        }
    }

    fn draw(&self, _state: &Input, _cursor: mouse::Cursor, bounds: Rectangle) -> Primitive {
        let camera = &self.camera;
        let light = (-0.45 * camera.right() + 0.75 * camera.up() + 0.5 * camera.back()).normalize();
        let back = camera.back();
        let size = bounds.size();
        let eye = camera.eye(size);
        let t = &self.tokens;
        let colors = [
            t.part_body,
            t.part_fin,
            t.accent,
            t.part_body,
            t.sys_aero,
            t.sys_propulsion,
            t.sys_recovery,
            t.sys_avionics,
            t.cut_face,
            t.cut_interior,
            t.bg_000,
        ]
        // Packed like the rest of the UI's colours: iced's default (web-colors) draws them as written, an sRGB build wants them
        // linear.
        .map(|c| iced::advanced::graphics::color::pack(c).components());
        let (mode, translucent) = match self.inside {
            InsideView::Solid => (0.0, self.reveal),
            InsideView::Cutaway => (1.0, false),
            InsideView::XRay => (2.0, true),
            InsideView::Exploded => (3.0, false),
        };
        let centre = (self.cut.centre.cos(), self.cut.centre.sin());
        Primitive {
            uniforms: Uniforms {
                view_proj: camera.view_proj(size).to_cols_array_2d(),
                light_dir: [light.x, light.y, light.z, 0.0],
                eye: [eye.x, eye.y, eye.z, if camera.is_ortho() { 0.0 } else { 1.0 }],
                view_dir: [back.x, back.y, back.z, 0.0],
                params: [mode, f32::from(self.reveal), self.explode, self.fade_from.unwrap_or(1.0e9)],
                cut: [centre.0, centre.1, (self.cut.span / 2.0).cos(), 1.0],
                screen: [camera.scale, 0.0, 0.0, 0.0],
                colors,
            },
            plan: DrawPlan { caps: self.inside == InsideView::Cutaway, translucent },
            mesh: self.mesh.clone(),
        }
    }

    fn mouse_interaction(&self, state: &Input, bounds: Rectangle, cursor: mouse::Cursor) -> mouse::Interaction {
        if state.drag.is_some() {
            mouse::Interaction::Grabbing
        } else if cursor.is_over(bounds) {
            self.interaction
        } else {
            mouse::Interaction::default()
        }
    }
}

#[derive(Debug)]
pub struct Primitive {
    uniforms: Uniforms,
    plan: DrawPlan,
    mesh: Arc<Mesh>,
}

impl shader::Primitive for Primitive {
    type Pipeline = Pipeline;

    fn prepare(&self, pipeline: &mut Pipeline, device: &iced::wgpu::Device, queue: &iced::wgpu::Queue, bounds: &Rectangle, viewport: &shader::Viewport) {
        pipeline.prepare(device, queue, &self.uniforms, self.plan, &self.mesh, bounds, viewport);
    }

    fn render(&self, pipeline: &Pipeline, encoder: &mut iced::wgpu::CommandEncoder, target: &iced::wgpu::TextureView, clip_bounds: &Rectangle<u32>) {
        pipeline.render(encoder, target, clip_bounds);
    }
}
