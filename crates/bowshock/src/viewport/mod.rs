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
//!
//! # Module contract
//!
//! [`View`] is the module's state, [`Message`] its messages. The app calls [`View::update`] with the shared [`Ctx`] (edits go
//! through `ctx.doc`, selection through `ctx.selection`, drags through `crate::app::drag`), [`View::view`] to draw, and
//! [`View::sync`] after every message so the mesh follows the document, selection, hover and drag ghosts.

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

use bowshock_model::{PartId, PartKind, RadialKind, Rocket};
use glam::{IVec3, Vec3};
use iced::widget::{canvas, container, row, shader, stack};
use iced::{keyboard, mouse, Element, Fill, Size, Task, Vector, Widget};

pub use camera::Camera;
pub use scene::{GhostRadial, Marks, Mesh};

use crate::app::{self, drag, Ctx, Selection};
use crate::theme;
use crate::tokens::SPACE_1;
use crate::tokens::SPACE_2;
use crate::tokens::SPACE_4;
use crate::viewport::manipulate::{Hover, Overlay as Manipulator};
use crate::viewport::pick::{Pick, SurfaceTarget};
use crate::viewport::surface::Overlay as SurfaceOverlay;
use crate::widgets::segmented::segmented;

/// Logical size of the ViewCube canvas.
pub const CUBE_SIZE: f32 = 112.0;

const SNAP_TIME: Duration = Duration::from_millis(320);

static ANIMATE: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(true);

/// Turns camera animations on or off. Off for `bowshock drive`: the headless emulator delivers no frame ticks.
pub fn set_animation(on: bool) {
    ANIMATE.store(on, std::sync::atomic::Ordering::Relaxed);
}

/// 3D or the side-on 2D profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Projection {
    /// Free orbit.
    ThreeD,
    /// Locked side-on.
    Profile,
}

/// How the shell is shown.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InsideView {
    /// Opaque.
    Solid,
    /// Cut in half.
    Cutaway,
    /// See-through.
    XRay,
    /// Parts pulled apart.
    Exploded,
}

/// The viewport's messages: camera, toolbar, and what its overlays report.
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
    /// The toolbar's 3D / 2D switch.
    Projection(Projection),
    /// The toolbar's Solid / Cutaway / X-ray / Exploded switch (also keys `1`..`4`).
    InsideView(InsideView),
    /// What is under the cursor changed.
    Hover(Hover),
    /// A click on a part (or on empty space, `None`).
    Select(Option<Pick>),
    /// A part was pressed and dragged: pick it up.
    Grab(Pick),
    /// A resize handle moved the part to this size (merged into one undo step until [`Message::EditEnd`]).
    Resize(PartId, PartKind),
    /// A resize handle moved the radial part to this size.
    ResizeRadial(PartId, RadialKind),
    /// The dragged axial part's nearest attach node changed.
    SnapNode(Option<usize>),
    /// The dragged radial part's surface target changed.
    Surface(Option<SurfaceTarget>),
    /// Released over the viewport: drop the dragged axial part on the snap node, if any.
    DropAxial,
    /// Released over the viewport: drop the dragged radial part on its surface target, if any.
    DropRadial,
    /// A direct edit (handle drag) ended: the document stops merging its steps.
    EditEnd,
}

struct Anim {
    from: Camera,
    to: Camera,
    start: Instant,
}

/// What the mesh was last built from; it is rebuilt when this changes.
#[derive(Debug, Clone, PartialEq)]
struct MeshKey {
    revision: u64,
    hovered: Option<PartId>,
    selected: Option<PartId>,
    axial: Option<Option<usize>>,
    radial: Option<(Option<SurfaceTarget>, u8)>,
}

/// The viewport's state: camera, toolbar choices, hover and the mesh on show.
pub struct View {
    camera: Camera,
    anim: Option<Anim>,
    profile: bool,
    mesh: Arc<Mesh>,
    /// Logical size of the viewport, as last reported.
    size: Size,
    projection: Projection,
    inside: InsideView,
    /// What is under the cursor.
    hover: Hover,
    /// Which copy of the selected radial group was clicked (and which part it belongs to).
    instance: Option<(PartId, usize)>,
    mesh_key: Option<MeshKey>,
}

impl View {
    /// An empty view. The camera looks at about where a rocket of typical size will appear until the first fit.
    pub fn new() -> Self {
        let camera = home(Camera { target: Vec3::new(0.0, 0.0, 0.4), yaw: 0.0, pitch: 0.0, roll: 0.0, scale: 500.0 });
        View {
            camera,
            anim: None,
            profile: false,
            mesh: Arc::new(Mesh::from_rocket(&Rocket::default(), Marks::default())),
            size: Size::new(860.0, 560.0),
            projection: Projection::ThreeD,
            inside: InsideView::Solid,
            hover: Hover::None,
            instance: None,
            mesh_key: None,
        }
    }

    /// The camera.
    #[allow(dead_code)] // For the viewport modules' overlays and the lanes (camera-locked ruler).
    pub fn camera(&self) -> Camera {
        self.camera
    }

    /// The inside view chosen on the toolbar.
    #[allow(dead_code)]
    pub fn inside(&self) -> InsideView {
        self.inside
    }

    /// Fits the model into the viewport at its last known size.
    pub fn fit(&mut self) {
        self.camera_update(Message::Fit(self.size));
    }

    /// Rebuilds the mesh if the document, selection, hover or a drag ghost changed since it was built. The app calls this
    /// after every message.
    pub fn sync(&mut self, ctx: &Ctx) {
        let selected = ctx.selection.part();
        let key = MeshKey {
            revision: ctx.doc.revision(),
            hovered: self.hover.part(),
            selected,
            axial: ctx.drag.axial.as_ref().map(|d| d.snap),
            radial: ctx.drag.radial.as_ref().map(|d| (d.target, d.count)),
        };
        if self.mesh_key.as_ref() == Some(&key) {
            return;
        }
        let rocket = ctx.doc.rocket();
        let ghost = ctx.drag.axial.as_ref().and_then(|d| d.snap.map(|node| (node, rocket.matched(node, &d.kind))));
        let ghost_radial = ctx.drag.radial.as_ref().and_then(|d| {
            let t = d.target?;
            Some(GhostRadial { kind: &d.kind, parent: t.parent, offset: t.offset, angle: t.angle, count: d.count })
        });
        let marks = Marks { ghost: ghost.as_ref().map(|(node, kind)| (*node, kind)), ghost_radial, hovered: self.hover.part(), selected };
        self.mesh = Arc::new(Mesh::from_rocket(rocket, marks));
        self.mesh_key = Some(key);
    }

    /// Handles a message: camera and toolbar changes stay here; edits go to the document and drags to `app::drag`.
    pub fn update(&mut self, message: Message, ctx: &mut Ctx) -> Task<app::Message> {
        match message {
            Message::Projection(p) => {
                self.projection = p;
                self.set_profile(p == Projection::Profile);
            }
            Message::InsideView(v) => self.inside = v,
            Message::Hover(hover) => self.hover = hover,
            Message::Select(pick) => {
                ctx.selection = pick.map_or(Selection::None, |p| Selection::Part(p.id()));
                self.instance = match pick {
                    Some(Pick::Radial { id, instance }) => Some((id, instance)),
                    _ => None,
                };
            }
            Message::Grab(pick) => {
                drag::grab(ctx, pick);
                self.hover = Hover::None;
                self.instance = None;
            }
            Message::Resize(id, kind) => {
                ctx.doc.edit_merging("Resize part", &format!("resize:{id}"), |r| r.set_kind(id, kind));
            }
            Message::ResizeRadial(id, kind) => {
                let count = ctx.doc.rocket().radial(id).map_or(1, |r| r.count);
                ctx.doc.edit_merging("Resize part", &format!("resize:{id}"), |r| r.set_radial(id, kind, count));
            }
            Message::SnapNode(node) => {
                if let Some(drag) = ctx.drag.axial.as_mut() {
                    drag.snap = node;
                }
            }
            Message::Surface(target) => {
                if let Some(drag) = ctx.drag.radial.as_mut() {
                    drag.target = target;
                }
            }
            Message::DropAxial => drag::finish_axial(ctx, true),
            Message::DropRadial => drag::finish_radial(ctx, true),
            Message::EditEnd => ctx.doc.seal(),
            camera => self.camera_update(camera),
        }
        Task::none()
    }

    /// The shortcut `key` maps to in the viewport (`1`..`4` choose the inside view), if any.
    pub fn on_key(&self, key: &keyboard::Key, modifiers: keyboard::Modifiers, _ctx: &Ctx) -> Option<Message> {
        if !modifiers.is_empty() {
            return None;
        }
        let keyboard::Key::Character(c) = key else {
            return None;
        };
        match c.as_str() {
            "1" => Some(Message::InsideView(InsideView::Solid)),
            "2" => Some(Message::InsideView(InsideView::Cutaway)),
            "3" => Some(Message::InsideView(InsideView::XRay)),
            "4" => Some(Message::InsideView(InsideView::Exploded)),
            _ => None,
        }
    }

    /// Escape: nothing of the viewport's own to abandon beyond what the app does (drags).
    pub fn cancel(&mut self) {
        self.hover = Hover::None;
    }

    fn camera_update(&mut self, message: Message) {
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
            _ => {}
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

    /// Advances the camera animation.
    pub fn tick(&mut self, now: Instant) {
        self.camera_update(Message::Tick(now));
    }

    /// Whether a camera animation is running (and so wants frame ticks).
    pub fn animating(&self) -> bool {
        self.anim.is_some()
    }

    /// The viewport pane: sheet, model, the overlay of the interaction in progress, the ViewCube and the view toolbar.
    pub fn view<'a>(&'a self, ctx: &'a Ctx) -> Element<'a, Message> {
        let tokens = ctx.tokens();
        let rocket = ctx.doc.rocket();
        let camera = self.camera;
        // While a part is dragged an assembly overlay owns the viewport; otherwise the manipulation overlay does.
        let (overlay, interaction): (Element<'a, Message>, mouse::Interaction) = if let Some(drag) = &ctx.drag.axial {
            (canvas(assembly::Overlay { rocket, drag, camera, cursor: ctx.cursor }).width(Fill).height(Fill).boxed(), mouse::Interaction::default())
        } else if let Some(drag) = &ctx.drag.radial {
            (
                canvas(SurfaceOverlay { rocket, drag, camera, snap: ctx.drag.angle_snap, cursor: ctx.cursor }).width(Fill).height(Fill).boxed(),
                mouse::Interaction::default(),
            )
        } else {
            let selected = ctx.selection.part();
            let instance = self.instance.filter(|(id, _)| Some(*id) == selected).map_or(0, |(_, i)| i);
            (
                canvas(Manipulator { rocket, camera, selected, instance, hover: self.hover, cursor: ctx.cursor }).width(Fill).height(Fill).boxed(),
                self.hover.interaction(),
            )
        };
        let sheet: Element<'_, Message> = canvas(sheet::Sheet { camera }).width(Fill).height(Fill).boxed();
        let model: Element<'_, Message> = shader(scene::Scene {
            camera,
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
        let mut layers = stack![sheet, model, overlay];
        if !self.profile {
            let cube = canvas(cube::ViewCube { camera }).width(CUBE_SIZE).height(CUBE_SIZE);
            let cube: Element<'_, Message> = container(cube).align_right(Fill).padding(SPACE_4).boxed();
            layers = layers.push(cube);
        }

        let toolbar = container(
            row![
                segmented(&[(Projection::ThreeD, "3D"), (Projection::Profile, "2D")], self.projection, Message::Projection),
                segmented(
                    &[(InsideView::Solid, "Solid"), (InsideView::Cutaway, "Cutaway"), (InsideView::XRay, "X-ray"), (InsideView::Exploded, "Exploded")],
                    self.inside,
                    Message::InsideView,
                ),
            ]
            .spacing(SPACE_2),
        )
        .padding(SPACE_1)
        .style(theme::overlay);
        // Overlays float `space-4` from the viewport edges.
        stack![layers, container(toolbar).padding(SPACE_4)].boxed()
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
