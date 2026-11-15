//! The 3D viewport: drawing-sheet ground, the shaded model (wgpu `shader` widget), and the Fusion-style ViewCube.
//!
//! Navigation (Fusion 360 defaults, plus trackpad equivalents):
//! - middle drag: pan; Shift + middle drag: orbit; mouse wheel: zoom about the cursor; double middle-click or `F`: fit
//! - trackpad two-finger scroll: pan; with Shift: orbit; with Cmd/Ctrl: zoom about the cursor
//! - ViewCube: hover highlights a face, edge or corner; click snaps to it; drag orbits; the house returns home; when
//!   looking straight at a face, the arrows step to the neighbouring faces
//!
//! The left button is for the model: hover, click to select, press and drag a part to pick it up and move it, drag the
//! handles of the selected part to resize it (see `manipulate`); while a part is dragged in from the parts bin or picked up
//! the `assembly` overlay shows attach nodes and the snap target.
//!
//! In Profile (2D) mode the camera is locked side-on: pan and zoom only, no ViewCube.

pub mod assembly;
pub mod camera;
mod cube;
pub mod manipulate;
pub mod pick;
mod pipeline;
mod radial;
mod scene;
mod sheet;
pub mod surface;

use std::f32::consts::FRAC_PI_2;
use std::sync::Arc;
use std::time::{Duration, Instant};

use bowshock_model::Rocket;
use glam::{IVec3, Vec3};
use iced::widget::{canvas, container, shader, stack};
use iced::{Element, Fill, Size, Vector, Widget};

pub use camera::Camera;
pub use scene::{GhostRadial, Marks, Mesh};

use crate::tokens::{Tokens, SPACE_4};

/// Logical size of the ViewCube canvas.
pub const CUBE_SIZE: f32 = 112.0;

const SNAP_TIME: Duration = Duration::from_millis(320);

static ANIMATE: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(true);

/// Turns camera animations on or off. Off for `bowshock drive`: the headless emulator delivers no frame ticks.
pub fn set_animation(on: bool) {
    ANIMATE.store(on, std::sync::atomic::Ordering::Relaxed);
}

#[derive(Debug, Clone)]
pub enum Message {
    /// Orbit by a drag of this many px.
    Orbit(Vector),
    /// Pan by a drag of this many px.
    Pan(Vector),
    /// Zoom by `factor` about `offset` px from the viewport centre (y down).
    Zoom { factor: f32, offset: Vector },
    /// Fit the model into a viewport of this logical size.
    Fit(Size),
    /// The viewport changed size (remembered so [`View::fit`] knows the size to fit to).
    Resized(Size),
    /// Snap to a ViewCube region: each component in {-1, 0, 1}, not all zero. The camera ends up looking along `-dir`.
    Snap(IVec3),
    /// Return to the home view.
    Home,
    /// Turn the picture a quarter turn about the view direction: `1` clockwise, `-1` anticlockwise (ViewCube roll arrows).
    Roll(i8),
    /// Animation frame.
    Tick(Instant),
}

struct Anim {
    from: Camera,
    to: Camera,
    start: Instant,
}

pub struct View {
    camera: Camera,
    anim: Option<Anim>,
    profile: bool,
    mesh: Arc<Mesh>,
    /// Logical size of the viewport, as last reported.
    size: Size,
}

impl View {
    /// An empty view. The camera looks at about where a rocket of typical size will appear until the first fit.
    pub fn new() -> Self {
        let camera = home(Camera { target: Vec3::new(0.0, 0.0, 0.4), yaw: 0.0, pitch: 0.0, roll: 0.0, scale: 500.0 });
        View { camera, anim: None, profile: false, mesh: Arc::new(Mesh::from_rocket(&Rocket::default(), Marks::default())), size: Size::new(860.0, 560.0) }
    }

    pub fn set_mesh(&mut self, mesh: Arc<Mesh>) {
        self.mesh = mesh;
    }

    pub fn camera(&self) -> Camera {
        self.camera
    }

    /// Fits the model into the viewport at its last known size.
    pub fn fit(&mut self) {
        self.update(Message::Fit(self.size));
    }

    pub fn update(&mut self, message: Message) {
        match message {
            Message::Orbit(delta) if !self.profile => {
                self.anim = None;
                self.camera.orbit(delta);
            }
            Message::Orbit(_) => {}
            Message::Pan(delta) => {
                self.settle();
                self.camera.pan(delta);
            }
            Message::Zoom { factor, offset } => {
                self.settle();
                self.camera.zoom(factor, offset);
            }
            Message::Fit(size) => {
                self.size = size;
                // Nothing to frame: keep the camera.
                if !self.mesh.is_empty() {
                    let mut to = self.goal();
                    to.fit(self.mesh.min, self.mesh.max, size);
                    self.animate(to);
                }
            }
            Message::Resized(size) => self.size = size,
            Message::Snap(dir) if !self.profile => {
                let mut to = self.goal();
                (to.yaw, to.pitch) = to.facing(dir.as_vec3());
                self.animate(to);
            }
            Message::Snap(_) => {}
            Message::Home => {
                let to = home(self.goal());
                self.animate(to);
            }
            Message::Roll(dir) if !self.profile => {
                let mut to = self.goal();
                to.roll += f32::from(dir) * FRAC_PI_2;
                self.animate(to);
            }
            Message::Roll(_) => {}
            Message::Tick(now) => {
                if let Some(anim) = &self.anim {
                    let t = (now.saturating_duration_since(anim.start).as_secs_f32() / SNAP_TIME.as_secs_f32()).min(1.0);
                    self.camera = anim.from.lerp(&anim.to, ease(t));
                    if t >= 1.0 {
                        self.anim = None;
                    }
                }
            }
        }
    }

    /// Locks the camera side-on (Profile) or unlocks it (3D).
    pub fn set_profile(&mut self, profile: bool) {
        self.profile = profile;
        let mut to = self.goal();
        if profile {
            (to.yaw, to.pitch, to.roll) = (-FRAC_PI_2, 0.0, 0.0);
        } else {
            to = home(to);
        }
        self.animate(to);
    }

    /// Whether a camera animation is running (and so wants frame ticks).
    pub fn animating(&self) -> bool {
        self.anim.is_some()
    }

    /// The viewport layers: sheet, model, `overlay` (drag and snap feedback, if any), ViewCube. Camera messages are turned
    /// into the app's message with `wrap`.
    pub fn view<'a, M: 'static>(
        &'a self,
        tokens: &Tokens,
        wrap: fn(Message) -> M,
        overlay: Option<Element<'a, M>>,
        interaction: iced::mouse::Interaction,
    ) -> Element<'a, M> {
        let sheet: Element<'_, Message> = canvas(sheet::Sheet { camera: self.camera }).width(Fill).height(Fill).boxed();
        let model: Element<'_, Message> = shader(scene::Scene {
            camera: self.camera,
            color: tokens.part_body,
            accent: tokens.accent,
            fin: tokens.part_fin,
            mesh: self.mesh.clone(),
            interaction,
            orbit: !self.profile,
        })
        .width(Fill)
        .height(Fill)
        .boxed();
        let mut layers = stack![sheet.map(wrap).boxed(), model.map(wrap).boxed()];
        if let Some(overlay) = overlay {
            layers = layers.push(overlay);
        }
        if !self.profile {
            let cube = canvas(cube::ViewCube { camera: self.camera }).width(CUBE_SIZE).height(CUBE_SIZE);
            let cube: Element<'_, Message> = container(cube).align_right(Fill).padding(SPACE_4).boxed();
            layers = layers.push(cube.map(wrap).boxed());
        }
        layers.boxed()
    }

    /// Where the camera is heading: the end of the running animation, or the camera itself.
    fn goal(&self) -> Camera {
        self.anim.as_ref().map_or(self.camera, |a| a.to)
    }

    /// Ends any animation at its goal (so a pan or zoom during a snap does not get overwritten).
    fn settle(&mut self) {
        if let Some(anim) = self.anim.take() {
            self.camera = anim.to;
        }
    }

    fn animate(&mut self, to: Camera) {
        if ANIMATE.load(std::sync::atomic::Ordering::Relaxed) {
            self.anim = Some(Anim { from: self.camera, to, start: Instant::now() });
        } else {
            self.anim = None;
            self.camera = to;
        }
    }
}

/// The home view: three-quarter from above, forward and the left side, like Fusion's home.
fn home(camera: Camera) -> Camera {
    let (yaw, pitch) = camera.facing(Vec3::new(-1.0, 0.45, -0.55));
    Camera { yaw, pitch, roll: 0.0, ..camera }
}

fn ease(t: f32) -> f32 {
    // Cubic ease-in-out.
    if t < 0.5 {
        4.0 * t * t * t
    } else {
        1.0 - (-2.0 * t + 2.0).powi(3) / 2.0
    }
}
