//! The shader widget program: mesh data, input handling and the per-frame primitive handed to the GPU pipeline.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use bowshock_model::{PartId, PartKind, RadialKind, RadialPart, Rocket};
use glam::Vec3;
use iced::time::{Duration, Instant};
use iced::widget::shader::{self, Action};
use iced::{keyboard, mouse, Color, Point, Rectangle, Vector};

use super::camera::Camera;
use super::pipeline::{Pipeline, Uniforms};
use super::radial::{self, Placement};
use super::Message;

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

/// What to draw besides the rocket as it is.
#[derive(Debug, Clone, Copy, Default)]
pub struct Marks<'a> {
    /// A part shown inserted at this node index, later parts shifted aft by its length.
    pub ghost: Option<(usize, &'a PartKind)>,
    /// A radial part shown placed on an axial part.
    pub ghost_radial: Option<GhostRadial<'a>>,
    /// Axial or radial part under the cursor.
    pub hovered: Option<PartId>,
    /// Axial or radial part selected.
    pub selected: Option<PartId>,
}

/// A radial part being dragged onto the rocket: every copy is drawn as a ghost.
#[derive(Debug, Clone, Copy)]
pub struct GhostRadial<'a> {
    pub kind: &'a RadialKind,
    pub parent: PartId,
    pub offset: f64,
    pub angle: f64,
    pub count: u8,
}

/// Triangle list, counter-clockwise front faces.
#[derive(Debug)]
pub struct Mesh {
    pub id: u64,
    pub vertices: Vec<Vertex>,
    pub min: Vec3,
    pub max: Vec3,
}

static NEXT_MESH_ID: AtomicU64 = AtomicU64::new(1);

/// Segments around the axis.
const SEGMENTS: usize = 64;

impl Mesh {
    /// The airframe as a surface of revolution about Z, each part at its station (world z, metres), with a flat disc on the
    /// aft face when it has any radius. See [`Marks`] for the ghost and the hovered / selected part.
    pub fn from_rocket(rocket: &Rocket, marks: Marks) -> Mesh {
        let state_of = |id: PartId| {
            if marks.selected == Some(id) {
                SELECTED
            } else if marks.hovered == Some(id) {
                HOVERED
            } else {
                NORMAL
            }
        };
        let mut stack: Vec<(Option<PartId>, &PartKind, f32)> = rocket.parts().iter().map(|p| (Some(p.id), &p.kind, state_of(p.id))).collect();
        if let Some((index, kind)) = marks.ghost {
            stack.insert(index.min(stack.len()), (None, kind, GHOST));
        }

        let mut vertices = Vec::new();
        let mut z = 0.0f64;
        // Station of each axial part as drawn (a ghost shifts the parts behind it).
        let mut stations: Vec<(PartId, f64)> = Vec::new();
        for (id, kind, state) in &stack {
            revolve(&mut vertices, &kind.profile(), z as f32, *state);
            stations.extend(id.map(|id| (id, z)));
            z += kind.length();
        }
        if let Some((_, kind, state)) = stack.last() {
            let radius = kind.aft_diameter() as f32 / 2.0;
            if radius > 0.0 {
                disc(&mut vertices, radius, z as f32, *state);
            }
        }

        let station_of = |parent: PartId| stations.iter().find(|(id, _)| *id == parent).map(|(_, z)| *z);
        let mut place = |radial: &RadialPart, state: f32| {
            let (Some(parent_z), Some(base)) = (station_of(radial.parent), rocket.radial_base_radius(radial)) else {
                return;
            };
            for angle in radial.angles() {
                let at = Placement { z0: (parent_z + radial.offset) as f32, base: base as f32, angle: angle as f32 };
                radial::emit(&mut vertices, &radial.kind, at, state);
            }
        };
        for radial in rocket.radials() {
            place(radial, state_of(radial.id));
        }
        if let Some(g) = marks.ghost_radial {
            let offset = rocket.clamp_offset(g.parent, g.offset, g.kind).unwrap_or(0.0);
            place(&RadialPart { id: PartId::MAX, kind: g.kind.clone(), parent: g.parent, offset, angle: g.angle, count: g.count }, GHOST);
        }

        let (mut min, mut max) = (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY));
        for v in &vertices {
            let p = Vec3::from(v.position);
            min = min.min(p);
            max = max.max(p);
        }
        if vertices.is_empty() {
            (min, max) = (Vec3::ZERO, Vec3::ZERO);
        }
        Mesh { id: NEXT_MESH_ID.fetch_add(1, Ordering::Relaxed), vertices, min, max }
    }

    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty()
    }
}

/// Revolves a fore-to-aft profile `(z, r)` (z from the part's fore face, placed at `z0`) about Z. Normals follow the profile's
/// slope, taken across neighbouring points, so a smooth profile shades smoothly.
fn revolve(out: &mut Vec<Vertex>, profile: &[(f64, f64)], z0: f32, state: f32) {
    let pts: Vec<(f32, f32)> = profile.iter().map(|&(z, r)| (z0 + z as f32, r as f32)).collect();
    // Outward normal in the (z, r) plane at each profile point.
    let normals: Vec<(f32, f32)> = (0..pts.len())
        .map(|i| {
            let (a, b) = (pts[i.saturating_sub(1)], pts[(i + 1).min(pts.len() - 1)]);
            let (tz, tr) = (b.0 - a.0, b.1 - a.1);
            let len = tz.hypot(tr).max(f32::MIN_POSITIVE);
            (-tr / len, tz / len)
        })
        .collect();
    let at = |i: usize, s: f32, c: f32| {
        let (z, r) = pts[i];
        let (nz, nr) = normals[i];
        Vertex { position: [r * c, r * s, z], normal: [nr * c, nr * s, nz], state, material: 0.0 }
    };
    for k in 0..SEGMENTS {
        let (s0, c0) = (std::f32::consts::TAU * k as f32 / SEGMENTS as f32).sin_cos();
        let (s1, c1) = (std::f32::consts::TAU * (k + 1) as f32 / SEGMENTS as f32).sin_cos();
        for i in 0..pts.len().saturating_sub(1) {
            let (p00, p10, p01, p11) = (at(i, s0, c0), at(i, s1, c1), at(i + 1, s0, c0), at(i + 1, s1, c1));
            // A zero radius collapses an edge of the quad into a point: skip the degenerate triangle.
            if pts[i].1 > 0.0 {
                out.extend([p00, p10, p11]);
            }
            if pts[i + 1].1 > 0.0 {
                out.extend([p00, p11, p01]);
            }
        }
    }
}

/// A flat disc facing +Z at `z`.
fn disc(out: &mut Vec<Vertex>, radius: f32, z: f32, state: f32) {
    let v = |x: f32, y: f32| Vertex { position: [x, y, z], normal: [0.0, 0.0, 1.0], state, material: 0.0 };
    for k in 0..SEGMENTS {
        let (s0, c0) = (std::f32::consts::TAU * k as f32 / SEGMENTS as f32).sin_cos();
        let (s1, c1) = (std::f32::consts::TAU * (k + 1) as f32 / SEGMENTS as f32).sin_cos();
        out.extend([v(0.0, 0.0), v(radius * c0, radius * s0), v(radius * c1, radius * s1)]);
    }
}

#[derive(Debug)]
pub struct Scene {
    pub camera: Camera,
    pub color: Color,
    /// Tint of the ghost part.
    pub accent: Color,
    /// Colour of radial parts (fins, lugs, buttons).
    pub fin: Color,
    pub mesh: Arc<Mesh>,
    /// The cursor to show over the model (a grab hand over a part, a resize arrow over a handle). Reported here rather than by
    /// the overlay above because a stack hides the cursor from the layers below any layer that reports a cursor of its own,
    /// which would stop middle-button navigation working over a part.
    pub interaction: mouse::Interaction,
    /// Whether orbiting is allowed (a 3D view rather than a locked side view).
    pub orbit: bool,
}

#[derive(Debug, Clone, Copy)]
struct Drag {
    last: Point,
    orbit: bool,
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
                state.drag = Some(Drag { last: position, orbit: state.modifiers.shift() && self.orbit });
                Some(Action::capture())
            }
            iced::Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Middle)) => state.drag.take().map(|_| Action::capture()),
            iced::Event::Mouse(mouse::Event::CursorMoved { position }) => {
                let drag = state.drag.as_mut()?;
                let delta = *position - drag.last;
                drag.last = *position;
                let message = if drag.orbit { Message::Orbit(delta) } else { Message::Pan(delta) };
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
                        } else if state.modifiers.shift() {
                            if !self.orbit {
                                return None;
                            }
                            Message::Orbit(Vector::new(-x, -y))
                        } else {
                            Message::Pan(Vector::new(x, y))
                        }
                    }
                };
                Some(Action::publish(message).and_capture())
            }
            _ => None,
        }
    }

    fn draw(&self, _state: &Input, _cursor: mouse::Cursor, bounds: Rectangle) -> Primitive {
        let camera = &self.camera;
        let light = (-0.45 * camera.right() + 0.75 * camera.up() + 0.5 * camera.back()).normalize();
        let back = camera.back();
        Primitive {
            uniforms: Uniforms {
                view_proj: camera.view_proj(bounds.size()).to_cols_array_2d(),
                light_dir: [light.x, light.y, light.z, 0.0],
                color: self.color.into_linear(),
                accent: self.accent.into_linear(),
                fin: self.fin.into_linear(),
                view_dir: [back.x, back.y, back.z, 0.0],
            },
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
    mesh: Arc<Mesh>,
}

impl shader::Primitive for Primitive {
    type Pipeline = Pipeline;

    fn prepare(&self, pipeline: &mut Pipeline, device: &iced::wgpu::Device, queue: &iced::wgpu::Queue, bounds: &Rectangle, viewport: &shader::Viewport) {
        pipeline.prepare(device, queue, &self.uniforms, &self.mesh, bounds, viewport);
    }

    fn render(&self, pipeline: &Pipeline, encoder: &mut iced::wgpu::CommandEncoder, target: &iced::wgpu::TextureView, clip_bounds: &Rectangle<u32>) {
        pipeline.render(encoder, target, clip_bounds);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{NoseShape, RadialKind};

    fn nose() -> PartKind {
        PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.1 }
    }

    fn tube() -> PartKind {
        PartKind::BodyTube { length: 0.5, diameter: 0.1 }
    }

    #[test]
    fn empty_rocket_is_an_empty_mesh() {
        assert!(Mesh::from_rocket(&Rocket::default(), Marks::default()).is_empty());
    }

    #[test]
    fn mesh_spans_the_rocket_and_the_ghost_shifts_later_parts() {
        let mut rocket = Rocket::default();
        rocket.insert(0, nose()).unwrap();
        let mesh = Mesh::from_rocket(&rocket, Marks::default());
        assert!((mesh.min.z - 0.0).abs() < 1e-6 && (mesh.max.z - 0.2).abs() < 1e-6);
        assert!((mesh.max.x - 0.05).abs() < 1e-6);
        assert!(mesh.vertices.iter().all(|v| v.state == NORMAL));

        // A tube ghosted at the aft node extends the rocket; ghosted at the fore node it shifts the nose aft instead.
        let aft = Mesh::from_rocket(&rocket, Marks { ghost: Some((1, &tube())), ..Marks::default() });
        assert!((aft.max.z - 0.7).abs() < 1e-6);
        assert!(aft.vertices.iter().any(|v| v.state == GHOST));
        let all_ghost_start = Mesh::from_rocket(&rocket, Marks { ghost: Some((0, &tube())), ..Marks::default() });
        let ghost_min = all_ghost_start.vertices.iter().filter(|v| v.state == GHOST).map(|v| v.position[2]).fold(f32::MAX, f32::min);
        let part_min = all_ghost_start.vertices.iter().filter(|v| v.state == NORMAL).map(|v| v.position[2]).fold(f32::MAX, f32::min);
        assert!(ghost_min.abs() < 1e-6 && (part_min - 0.5).abs() < 1e-6);
    }

    #[test]
    fn hovered_and_selected_parts_are_marked() {
        let mut rocket = Rocket::default();
        let nose_id = rocket.insert(0, nose()).unwrap();
        let tube_id = rocket.insert(1, tube()).unwrap();
        let mesh = Mesh::from_rocket(&rocket, Marks { hovered: Some(tube_id), selected: Some(nose_id), ..Marks::default() });
        let states = |lo: f32, hi: f32| mesh.vertices.iter().filter(|v| (lo..hi).contains(&v.position[2])).map(|v| v.state).collect::<Vec<_>>();
        assert!(states(0.0, 0.19).iter().all(|&s| s == SELECTED));
        assert!(states(0.21, 0.7).iter().all(|&s| s == HOVERED));
        // Selected wins over hovered.
        let both = Mesh::from_rocket(&rocket, Marks { hovered: Some(nose_id), selected: Some(nose_id), ..Marks::default() });
        assert!(both.vertices.iter().any(|v| v.state == SELECTED) && both.vertices.iter().all(|v| v.state != HOVERED));
    }

    #[test]
    fn radial_parts_are_part_of_the_mesh_and_follow_a_ghost() {
        let mut rocket = Rocket::default();
        rocket.insert(0, nose()).unwrap();
        let tube_id = rocket.insert(1, tube()).unwrap();
        let bare = Mesh::from_rocket(&rocket, Marks::default());
        let fins = RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.08, sweep: 0.03, thickness: 0.003 };
        let id = rocket.attach_radial(tube_id, 0.4, 0.0, 4, fins.clone()).unwrap();
        let mesh = Mesh::from_rocket(&rocket, Marks::default());
        // The fin vertices use the fin colour; the airframe's own vertices are unchanged.
        assert_eq!(mesh.vertices.iter().filter(|v| v.material == 0.0).count(), bare.vertices.len());
        assert!(mesh.vertices.iter().any(|v| v.material == 1.0));
        // Fins reach `span` beyond the tube's 0.05 radius, at the up/down and sideways angles.
        assert!((mesh.max.y - 0.13).abs() < 1e-5 && (mesh.min.y + 0.13).abs() < 1e-5 && (mesh.max.x - 0.13).abs() < 1e-5);
        // Selecting the group marks only its vertices.
        let marked = Mesh::from_rocket(&rocket, Marks { selected: Some(id), ..Marks::default() });
        assert!(marked.vertices.iter().all(|v| (v.material == 1.0) == (v.state == SELECTED)));
        // A ghost axial part ahead of the tube pushes the fins aft with their parent.
        let shifted = Mesh::from_rocket(&rocket, Marks { ghost: Some((1, &tube())), ..Marks::default() });
        assert!((shifted.max.z - (mesh.max.z + 0.5)).abs() < 1e-5);
        // A ghost radial part draws all its copies in the ghost state, without being in the rocket.
        let lug = RadialKind::LaunchLug { length: 0.05, diameter: 0.01 };
        let ghost = GhostRadial { kind: &lug, parent: tube_id, offset: 0.1, angle: 0.0, count: 3 };
        let with = Mesh::from_rocket(&rocket, Marks { ghost_radial: Some(ghost), ..Marks::default() });
        assert!(with.vertices.iter().filter(|v| v.state == GHOST).count() > 0 && rocket.radials().len() == 1);
    }
}
