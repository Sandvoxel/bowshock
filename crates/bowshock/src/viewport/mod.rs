//! The 3D viewport: drawing-sheet ground, the shaded model (wgpu `shader` widget), the ViewCube, the view toolbar and the
//! drawn overlays (CG/CP, staging, section handle, legend, key hints).
//!
//! Navigation (design/components/Navigation):
//! - right-drag: orbit; middle-drag: pan; `Alt` + right-drag: roll; mouse wheel: zoom about the cursor; `F` or double-click on
//!   empty space: fit; double middle-click: fit
//! - trackpad: pinch zooms about the cursor (the vendored iced forwards winit's `PinchGesture`, see `vendor/README.md`);
//!   two-finger scroll orbits (pan in Profile); with `Shift`: pan; with `Cmd`/`Ctrl`: zoom about the cursor as well. Rotate is
//!   not forwarded: `Alt` + right-drag is the roll
//! - an edit that pushes the rocket out of the view refits it, unless the view was panned or zoomed since the last fit: then the
//!   view stays where the user put it and the hint bar says `F fit`
//! - double-click a part: focus (the camera turns square-on in an orthographic view and fills the viewport with the part;
//!   `Esc` goes back)
//! - ViewCube: hover highlights a face, edge or corner; click snaps to it; drag orbits; the house returns home; when looking
//!   straight at a face, the arrows step to the neighbouring faces
//!
//! The left button is for the model: hover, click to select, press and drag a part to pick it up and move it, drag the
//! handles of the selected part to resize it (see `manipulate`); while a part is dragged in from the parts bin or picked up
//! the `assembly` overlay shows attach nodes and the snap target.
//!
//! The 3D view is a perspective one; Profile (2D) and a focused part are orthographic (the camera blends between them), and
//! Profile is locked side-on: pan and zoom only, no ViewCube.
//!
//! The shell is shown four ways ([`InsideView`], keys `1` to `4`): Solid, Cutaway (a wedge cut away along the axis, cut faces
//! hatched, a round handle at the nose end sets the wedge, `Shift` snapping it to quarter turns), X-ray and Exploded.
//! Selecting an internal part ghosts the airframe around it in Solid.
//!
//! # Module contract
//!
//! [`View`] is the module's state, [`Message`] its messages. The app calls [`View::update`] with the shared [`Ctx`] (edits go
//! through `ctx.doc`, selection through `ctx.selection`, drags through `crate::app::drag`), [`View::view`] to draw, and
//! [`View::sync`] after every message so the mesh follows the document, selection, hover and drag ghosts.

pub mod assembly;
pub mod camera;
pub mod capture;
mod carry;
mod cube;
mod empty;
pub mod handles;
mod internal;
mod labels;
pub mod manipulate;
mod markers;
mod mesh;
pub mod pick;
mod pipeline;
pub mod pods;
mod radial;
mod scene;
mod sheet;
mod snap;
mod solids;
pub mod surface;
mod toolbar;

use std::f32::consts::FRAC_PI_2;
use std::fmt;
use std::sync::Arc;
use std::time::{Duration, Instant};

use bowshock_model::{Dim, InternalKind, PartId, PartRef, RadialKind};
use glam::{IVec3, Vec3};
use iced::widget::{canvas, column, container, row, shader, stack, text, Space};
use iced::{alignment, keyboard, mouse, Element, Fill, Size, Task, Vector, Widget};

use crate::app::{self, drag, Ctx, Selection};
use crate::theme::{self, MONO};
use crate::tokens::{SPACE_1, SPACE_2, SPACE_4, TEXT_LABEL};
use crate::viewport::manipulate::{Hover, Overlay as Manipulator};
use crate::viewport::pick::{Pick, SurfaceTarget};
use crate::viewport::surface::Overlay as SurfaceOverlay;
use crate::viewport::toolbar::Glyph;
use crate::widgets::button::{button, Weight};

pub use camera::Camera;
pub use capture::Capture;
pub use mesh::{Cut, GhostRadial, Marks, Mesh};
pub use solids::{GpuVertex, LAYER_INTERNAL, LAYER_RADIAL, LAYER_SHELL};

pub mod flash;

/// Logical size of the ViewCube canvas.
pub const CUBE_SIZE: f32 = 112.0;
/// Strip left of the ViewCube that holds the axis triad (the canvas is `TRIAD_GUTTER + CUBE_SIZE` wide).
pub const TRIAD_GUTTER: f32 = 56.0;

const SNAP_TIME: Duration = Duration::from_millis(320);
/// How much of the orthographic blend one orbit step takes away, so a drag off a face view eases into perspective.
const ORBIT_TO_PERSPECTIVE: f32 = 0.2;
/// The focus move and the exploded view animate over this long.
const FOCUS_TIME: Duration = Duration::from_millis(250);
/// Fraction of the viewport kept clear on each side of a focused part (the part fills about the middle half).
const FOCUS_MARGIN: f32 = 0.25;
/// The most a focus zooms in, relative to fitting the whole rocket.
const FOCUS_MAX_ZOOM: f32 = 2.0;
/// Two selections of the same thing this close together are a double-click.
const DOUBLE_CLICK: Duration = Duration::from_millis(400);
/// Headless runs (`bowshock drive`, animations off) render between scripted clicks, which can take longer than that.
const HEADLESS_DOUBLE_CLICK: Duration = Duration::from_secs(10);

static ANIMATE: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(true);

/// Turns camera animations on or off. Off for `bowshock drive`: the headless emulator delivers no frame ticks.
pub fn set_animation(on: bool) {
    ANIMATE.store(on, std::sync::atomic::Ordering::Relaxed);
}

/// Whether animations are on (they are not in a headless `bowshock drive` run).
pub fn animation_on() -> bool {
    ANIMATE.load(std::sync::atomic::Ordering::Relaxed)
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

/// The toolbar's grid step: what a plain handle drag moves by, metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridStep(pub f64);

impl GridStep {
    /// The choices, metres.
    pub const ALL: [GridStep; 5] = [GridStep(0.0001), GridStep(0.0005), GridStep(0.001), GridStep(0.005), GridStep(0.01)];
}

/// A grid step as the toolbar's select lists it: written in the length unit of the unit set, to the digits it needs
/// (`0.1 mm`, `1 mm`, `0.039 in`).
#[derive(Debug, Clone, Copy, PartialEq)]
struct GridChoice(GridStep, crate::units::UnitSystem);

impl fmt::Display for GridChoice {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let unit = crate::units::Quantity::Length.unit_for(self.1);
        let text = format!("{:.3}", unit.from_si(self.0 .0));
        write!(f, "{} {}", text.trim_end_matches('0').trim_end_matches('.'), unit.symbol)
    }
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
    /// Turn the picture by a drag of this many px (`Alt` + right-drag).
    RollBy(Vector),
    /// Animation frame.
    Tick(Instant),
    /// The toolbar's 3D / 2D switch.
    Projection(Projection),
    /// The toolbar's Solid / Cutaway / X-ray / Exploded switch (also keys `1`..`4`).
    InsideView(InsideView),
    /// The toolbar's snap step.
    Step(GridStep),
    /// What is under the cursor changed.
    Hover(Hover),
    /// A click on a part (or on empty space, `None`). Two on the same part in quick succession focus it; two on empty space fit.
    Select(Option<Pick>),
    /// Select a part by id (a click on a part in the exploded view).
    SelectPart(Option<PartId>),
    /// Shift-click or Cmd/Ctrl-click on a part: adds it to the selection or, if selected, removes it.
    Toggle(Pick),
    /// Right-click in place on a part: select it (unless it is already selected) and open its context menu.
    PartMenu(Pick),
    /// Select the separation of this stage (a click on its diamond).
    SelectSeparation(usize),
    /// A part was pressed and dragged: pick it up.
    Grab(Pick),
    /// A handle edit (see `manipulate`).
    Edit(manipulate::Edit),
    /// The key hints to show at the bottom (pairs of key, what it does); empty clears them.
    #[allow(dead_code)] // For overlays that want to show keys; direct edits set theirs through `Edit`.
    Hint(Vec<(&'static str, &'static str)>),
    /// Focus a part: turn square-on to it in an orthographic view that fills the viewport with it, and select it (also sent by a
    /// double-click on a Station Lanes bar).
    Focus(PartId),
    /// Go back from a focus (`Esc`).
    ExitFocus,
    /// The cutaway wedge changed (the section handle).
    Cut(Cut),
    /// The dragged axial part's nearest attach node changed.
    SnapNode(Option<usize>),
    /// The dragged radial part's surface target changed, and whether it has been pulled free of the rocket it came from.
    Surface(Option<SurfaceTarget>, bool),
    /// Released over the viewport: drop the dragged axial part on the snap node, if any.
    DropAxial,
    /// Released over the viewport: drop the dragged radial part on its surface target, if any.
    DropRadial,
    /// The dragged internal part's landing changed.
    InternalTarget(Option<crate::app::drag::InternalTarget>),
    /// Released over the viewport: drop the dragged internal part on its target, if any.
    DropInternal,
    /// The mouse wheel turned during a radial drag: one step through the symmetry counts (like `X` / `Shift+X`), no zoom.
    CopiesStep(bool),
    /// The dragged axial part's nearest attach node is on a pod's stack: `(pod, node index)`.
    SnapPodNode(Option<(PartId, usize)>),
    /// The dragged axial part is over a tube's side: it would attach radially there (it becomes a pod).
    AxialSurface(Option<SurfaceTarget>),
    /// An image export: render the model once with this request (see [`capture`]); `None` ends it.
    Capture(Option<Arc<Capture>>),
}

struct Anim {
    from: Camera,
    to: Camera,
    start: Instant,
    duration: Duration,
}

/// The camera, projection and view mode to go back to when a focus ends.
struct Focus {
    camera: Camera,
    profile: bool,
    projection: Projection,
    inside: InsideView,
    /// The tag on the toolbar: `ORTHO · SIDE · FIN SET A`.
    tag: String,
}

/// What the mesh was last built from; it is rebuilt when this changes.
#[derive(Debug, Clone, PartialEq)]
struct MeshKey {
    revision: u64,
    hovered: Option<PartId>,
    selected: Option<PartId>,
    also: Vec<PartId>,
    axial: Option<(Option<usize>, Option<(PartId, usize)>, Option<SurfaceTarget>, u8)>,
    radial: Option<(Option<SurfaceTarget>, u8)>,
    cut: Cut,
    flash: Option<(PartId, f32, f32)>,
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
    /// The cutaway wedge.
    cut: Cut,
    /// The exploded view's progress, 0 to 1, and the animation moving it: `(from, to, start)`.
    explode: f32,
    explode_anim: Option<(f32, f32, Instant)>,
    /// The toolbar's grid step, metres.
    snap_step: f64,
    /// The key hints a direct edit asked for.
    hint: Vec<(&'static str, &'static str)>,
    /// The focused part's way back.
    focus: Option<Focus>,
    /// The last selection click: when, and on what.
    last_select: Option<(Instant, Option<PartId>)>,
    /// What is under the cursor.
    hover: Hover,
    mesh_key: Option<MeshKey>,
    /// How far (metres along the axis) the camera has been moved to keep the aft end of a part still while its fore end is dragged.
    fore_pan: f32,
    /// The inside view to go back to when the internal part being carried in from the bin is dropped or cancelled.
    drag_inside: Option<InsideView>,
    /// The view was panned or zoomed by hand since the last fit: an edit leaves it alone (see [`View::reframe`]).
    navigated: bool,
    /// The last document revision [`View::reframe`] looked at (`None` before the first sync).
    reframed: Option<u64>,
    /// An edit left part of the rocket outside a view the user had set: the hint bar offers `F fit` while it stays that way.
    stray: bool,
    /// An image export waiting for its render.
    capture: Option<Arc<Capture>>,
}

impl View {
    /// An empty view. The camera looks at about where a rocket of typical size will appear until the first fit.
    pub fn new() -> Self {
        let camera = home(Camera { target: Vec3::new(0.0, 0.0, 0.4), scale: 500.0, ..Camera::default() });
        View {
            camera,
            anim: None,
            profile: false,
            mesh: Arc::new(Mesh::from_rocket(&bowshock_model::Rocket::default(), Marks::default())),
            size: Size::new(860.0, 560.0),
            projection: Projection::ThreeD,
            inside: InsideView::Solid,
            cut: Cut::default(),
            explode: 0.0,
            explode_anim: None,
            snap_step: GridStep::ALL[2].0,
            hint: Vec::new(),
            focus: None,
            last_select: None,
            hover: Hover::None,
            mesh_key: None,
            fore_pan: 0.0,
            drag_inside: None,
            navigated: false,
            reframed: None,
            stray: false,
            capture: None,
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

    /// The toolbar's grid step, metres: what a plain handle drag moves by.
    #[allow(dead_code)]
    pub fn snap_step(&self) -> f64 {
        self.snap_step
    }

    /// Asks for one render of the model at `width` × `height` pixels, framed like the viewport and drawn without hover, selection
    /// or flash marks. The pixels arrive on the returned receiver once the viewport has drawn its next frame; end the request with
    /// [`Message::Capture`]`(None)`.
    pub fn start_capture(
        &mut self,
        ctx: &Ctx,
        width: u32,
        height: u32,
        transparent: bool,
        background: [f32; 4],
    ) -> iced::futures::channel::oneshot::Receiver<capture::Pixels> {
        let marks = Marks { motors: Some(ctx.physics.motors()), cut: self.cut, ..Marks::default() };
        let mesh = Arc::new(Mesh::from_rocket(ctx.doc.rocket(), marks));
        let (request, pixels) = Capture::new(width, height, transparent, background, mesh);
        self.capture = Some(request);
        pixels
    }

    /// Fits the model into the viewport at its last known size.
    pub fn fit(&mut self) {
        self.camera_update(Message::Fit(self.size));
    }

    /// Rebuilds the mesh if the document, selection, hover, cutaway or a drag ghost changed since it was built. The app calls
    /// this after every message.
    pub fn sync(&mut self, ctx: &Ctx) {
        // An internal part carried in from the bin opens the airframe: X-ray while it is carried, the view it was before after.
        match (ctx.drag.internal.as_ref().is_some_and(|d| d.origin.is_none()), self.drag_inside) {
            (true, None) => {
                self.drag_inside = Some(self.inside);
                self.set_inside(InsideView::XRay);
            }
            (false, Some(before)) => {
                self.drag_inside = None;
                self.set_inside(before);
            }
            _ => {}
        }
        self.rebuild_mesh(ctx);
        self.reframe(ctx);
    }

    /// Whether any of the rocket (as meshed) lies outside the viewport at the camera's goal.
    pub(crate) fn out_of_view(&self) -> bool {
        !self.mesh.is_empty() && !self.goal().contains(self.mesh.min, self.mesh.max, self.size)
    }

    /// After an edit (a new document revision, once no drag or handle edit is in progress) that leaves the rocket reaching beyond
    /// the viewport: refit gently (the usual animated fit) if the user has not panned or zoomed since the last fit, else leave
    /// their view alone and raise [`View::stray`] so the hint bar offers `F fit`. Never mid-drag (the ghost and the handle edits
    /// change the mesh every move, and a refit would fight the hand) and never in a focus (which crops on purpose).
    fn reframe(&mut self, ctx: &Ctx) {
        if ctx.drag.active() || ctx.doc.editing_live() {
            return;
        }
        let revision = ctx.doc.revision();
        let before = self.reframed.replace(revision);
        if before.is_none() || before == Some(revision) || self.focus.is_some() {
            return;
        }
        self.stray = false;
        if self.out_of_view() {
            if self.navigated {
                self.stray = true;
            } else {
                self.fit();
            }
        }
    }

    /// Rebuilds the mesh if the document, selection, hover, cutaway or a drag ghost changed since it was built.
    fn rebuild_mesh(&mut self, ctx: &Ctx) {
        let selected = ctx.selection.part();
        let also: Vec<PartId> = ctx.selected_parts().into_iter().skip(1).collect();
        let flash = ctx.flash_levels();
        let key = MeshKey {
            revision: ctx.doc.revision(),
            hovered: self.hover.part(),
            selected,
            also: also.clone(),
            axial: ctx.drag.axial.as_ref().map(|d| (d.snap, d.pod_snap, d.surface, d.count)),
            radial: ctx.drag.radial.as_ref().map(|d| (d.target, d.count)),
            cut: self.cut,
            flash,
        };
        if self.mesh_key.as_ref() == Some(&key) {
            return;
        }
        let rocket = ctx.doc.rocket();
        let ghost = ctx.drag.axial.as_ref().and_then(|d| d.snap.map(|node| (node, rocket.matched(node, &d.kind))));
        let ghost_pod = ctx.drag.axial.as_ref().and_then(|d| d.pod_snap.map(|(pod, node)| (pod, node, rocket.matched_in_pod(pod, node, &d.kind))));
        // A part dragged over a tube's side stands there as a pod of its own: the ghost is that pod.
        let axial_pod = ctx.drag.axial.as_ref().and_then(|d| Some((RadialKind::Pod { gap: 0.0, body: assembly::body_of(d) }, d.surface?, d.count)));
        let ghost_radial = ctx
            .drag
            .radial
            .as_ref()
            .and_then(|d| {
                let t = d.target?;
                Some(GhostRadial { kind: &d.kind, parent: t.parent, offset: t.offset, angle: t.angle, count: d.count })
            })
            .or_else(|| axial_pod.as_ref().map(|(kind, t, count)| GhostRadial { kind, parent: t.parent, offset: t.offset, angle: t.angle, count: *count }));
        let marks = Marks {
            ghost: ghost.as_ref().map(|(node, kind)| (*node, kind)),
            ghost_radial,
            ghost_pod: ghost_pod.as_ref().map(|(pod, node, kind)| (*pod, *node, kind)),
            hovered: self.hover.part(),
            selected,
            also_selected: &also,
            motors: Some(ctx.physics.motors()),
            cut: self.cut,
            flash,
        };
        self.mesh = Arc::new(Mesh::from_rocket(rocket, marks));
        self.mesh_key = Some(key);
    }

    /// Handles a message: camera and toolbar changes stay here; edits go to the document and drags to `app::drag`.
    pub fn update(&mut self, message: Message, ctx: &mut Ctx) -> Task<app::Message> {
        match message {
            Message::Projection(p) => {
                self.focus = None;
                self.projection = p;
                self.set_profile(p == Projection::Profile);
            }
            Message::InsideView(v) => self.set_inside(v),
            Message::Step(step) => self.snap_step = step.0,
            Message::Hover(hover) => self.hover = hover,
            Message::Select(pick) => {
                let id = pick.map(Pick::id);
                ctx.symmetry.instance = match pick {
                    Some(Pick::Radial { id, instance }) => Some((id, instance)),
                    _ => None,
                };
                // Two selections of the same thing in quick succession: focus the part, or fit on empty space.
                let now = Instant::now();
                let window = if ANIMATE.load(std::sync::atomic::Ordering::Relaxed) { DOUBLE_CLICK } else { HEADLESS_DOUBLE_CLICK };
                let double = self.last_select.is_some_and(|(at, last)| now.duration_since(at) <= window && last == id);
                self.last_select = if double { None } else { Some((now, id)) };
                ctx.select_one(id);
                if double {
                    match id {
                        Some(id) => self.focus_part(id, ctx),
                        None => self.fit(),
                    }
                }
            }
            Message::SelectPart(id) => {
                ctx.select_one(id);
                ctx.symmetry.instance = None;
            }
            Message::Toggle(pick) => {
                ctx.toggle_part(pick.id());
                if let (Pick::Radial { id, instance }, true) = (pick, ctx.selection.part() == Some(pick.id())) {
                    ctx.symmetry.instance = Some((id, instance));
                }
            }
            Message::PartMenu(pick) => {
                let id = pick.id();
                if !ctx.selected_parts().contains(&id) {
                    ctx.select_one(Some(id));
                    ctx.symmetry.instance = None;
                }
                if let (Pick::Radial { id, instance }, true) = (pick, ctx.selection.part() == Some(id)) {
                    ctx.symmetry.instance = Some((id, instance));
                }
                return Task::done(app::Message::OpenPartMenu(id));
            }
            Message::SelectSeparation(stage) => ctx.selection = Selection::Separation(stage),
            Message::Grab(pick) => {
                drag::grab(ctx, pick);
                self.hover = Hover::None;
                ctx.symmetry.instance = None;
            }
            Message::Edit(edit) => {
                if let Some(hint) = edit.hint() {
                    self.hint = hint;
                }
                let shift = self.follow_fore_edit(&edit, ctx);
                let task = manipulate::apply(edit, ctx);
                if let Some(shift) = shift {
                    ctx.doc.set_undo_shift(shift);
                }
                return task;
            }
            Message::Hint(hint) => self.hint = hint,
            Message::Focus(id) => self.focus_part(id, ctx),
            Message::ExitFocus => self.exit_focus(),
            Message::Cut(cut) => self.cut = cut,
            Message::SnapNode(node) => {
                if let Some(drag) = ctx.drag.axial.as_mut() {
                    (drag.snap, drag.pod_snap, drag.surface) = (node, None, None);
                }
            }
            Message::SnapPodNode(node) => {
                if let Some(drag) = ctx.drag.axial.as_mut() {
                    (drag.snap, drag.pod_snap, drag.surface) = (None, node, None);
                }
            }
            Message::AxialSurface(target) => {
                if let Some(drag) = ctx.drag.axial.as_mut() {
                    (drag.snap, drag.pod_snap, drag.surface) = (None, None, target);
                }
            }
            Message::Surface(target, detached) => {
                let symmetry = ctx.drag.symmetry;
                if let Some(drag) = ctx.drag.radial.as_mut() {
                    // Pulling a group free of the rocket hands it the Symmetry selection: it lands with that many copies.
                    if detached && !drag.detached {
                        drag.count = symmetry;
                    }
                    (drag.target, drag.detached) = (target, detached);
                }
            }
            Message::DropAxial => drag::finish_axial(ctx, true),
            Message::DropRadial => drag::finish_radial(ctx, true),
            Message::InternalTarget(target) => {
                if let Some(drag) = ctx.drag.internal.as_mut() {
                    drag.target = target;
                }
            }
            Message::DropInternal => drag::finish_internal(ctx, true),
            Message::CopiesStep(forward) => crate::build::parts_bin::cycle_symmetry(ctx, forward),
            camera => self.camera_update(camera),
        }
        Task::none()
    }

    /// The shortcut `key` maps to in the viewport (`1`..`4` choose the inside view), if any. Nothing while a handle is being
    /// dragged: typed digits belong to the drag's tag.
    pub fn on_key(&self, key: &keyboard::Key, modifiers: keyboard::Modifiers, ctx: &Ctx) -> Option<Message> {
        if !modifiers.is_empty() || ctx.doc.dragging() {
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

    /// Escape: goes back from a focused part, or leaves the exploded view; also clears the hover and the key hints.
    pub fn cancel(&mut self) {
        self.hover = Hover::None;
        self.hint.clear();
        if self.focus.is_some() {
            self.exit_focus();
        } else if self.inside == InsideView::Exploded {
            self.set_inside(InsideView::Solid);
        }
    }

    /// Shows the shell as `view`, animating the exploded view in or out.
    fn set_inside(&mut self, view: InsideView) {
        self.inside = view;
        let to = if view == InsideView::Exploded { 1.0 } else { 0.0 };
        if ANIMATE.load(std::sync::atomic::Ordering::Relaxed) {
            self.explode_anim = Some((self.explode, to, Instant::now()));
        } else {
            self.explode = to;
            self.explode_anim = None;
        }
    }

    /// Focuses a part: turns square-on to it (side-on, or aft-on for rings and bulkheads) in an orthographic view that fills the
    /// viewport with it, leaving `space-8` around, and selects it. An internal part also opens the shell (Cutaway, or X-ray for
    /// an aft-on view, where the shell's aft face would hide it). Focusing another part moves the focus without stacking.
    fn focus_part(&mut self, id: PartId, ctx: &mut Ctx) {
        let Some(bounds) = self.mesh.part_bounds(id).copied() else {
            return;
        };
        let rocket = ctx.doc.rocket();
        let part = rocket.kind_of(id);
        let aft_on = matches!(
            part,
            Some(PartRef::Internal(p)) if matches!(p.kind, InternalKind::CenteringRing { .. } | InternalKind::Bulkhead { .. })
        );
        let internal = matches!(part, Some(PartRef::Internal(_)));
        let mut tag = format!("ORTHO · {} · {}", if aft_on { "AFT" } else { "SIDE" }, part.map_or_else(String::new, |p| p.name()).to_uppercase());
        if let (Some(PartRef::Radial(r)), Some((rid, i))) = (part, ctx.symmetry.instance) {
            if r.count > 1 && rid == id {
                tag.push(' ');
                tag.push((b'A' + (i % 26) as u8) as char);
            }
        }
        if self.focus.is_none() {
            self.focus = Some(Focus { camera: self.goal(), profile: self.profile, projection: self.projection, inside: self.inside, tag: String::new() });
        }
        if let Some(f) = self.focus.as_mut() {
            f.tag = tag;
        }
        if internal {
            match (self.inside, aft_on) {
                (_, true) => self.set_inside(InsideView::XRay),
                (InsideView::Solid, false) => self.set_inside(InsideView::Cutaway),
                _ => {}
            }
        }
        let mut to = self.goal();
        (to.yaw, to.pitch, to.roll) = if aft_on { (0.0, 0.0, 0.0) } else { (-FRAC_PI_2, 0.0, 0.0) };
        to.ortho_blend = 1.0;
        // Leave room around the part so its neighbours stay in view, and never zoom in more than `FOCUS_MAX_ZOOM`× the
        // whole-rocket fit: a small part (a rail button, a ring) would otherwise fill the screen with no context.
        to.fit_in(bounds.min, bounds.max, self.size, Vector::new(self.size.width * FOCUS_MARGIN, self.size.height * FOCUS_MARGIN));
        if !self.mesh.is_empty() {
            let mut whole = to;
            whole.fit(self.mesh.min, self.mesh.max, self.size);
            to.scale = to.scale.min(whole.scale * FOCUS_MAX_ZOOM);
        }
        self.animate_for(to, FOCUS_TIME);
        ctx.selection = Selection::Part(id);
    }

    /// Back to where the camera, projection and view mode were before the focus.
    fn exit_focus(&mut self) {
        let Some(f) = self.focus.take() else {
            return;
        };
        self.profile = f.profile;
        self.projection = f.projection;
        if self.inside != f.inside {
            self.set_inside(f.inside);
        }
        self.animate_for(f.camera, FOCUS_TIME);
    }

    fn camera_update(&mut self, message: Message) {
        match message {
            // An empty rocket has nothing to turn round: the camera stays put so the empty-state view keeps its framing.
            Message::Orbit(delta) if !self.profile && !self.mesh.is_empty() => {
                self.anim = None;
                self.camera.orbit(delta);
                // Turning away from a face view eases back into perspective (a focus stays orthographic until `Esc`).
                if self.focus.is_none() {
                    self.camera.ortho_blend = (self.camera.ortho_blend - ORBIT_TO_PERSPECTIVE).max(0.0);
                }
            }
            Message::Orbit(_) => {}
            Message::RollBy(delta) if !self.profile && !self.mesh.is_empty() => {
                self.anim = None;
                self.camera.roll_by(delta);
            }
            Message::RollBy(_) => {}
            Message::Pan(delta) => {
                self.navigated = true;
                self.settle();
                self.camera.pan(delta);
            }
            Message::Zoom { factor, offset } => {
                self.navigated = true;
                self.settle();
                self.camera.zoom(factor, offset);
            }
            Message::Fit(size) => {
                self.size = size;
                (self.navigated, self.stray) = (false, false);
                // Nothing to frame: keep the camera.
                if !self.mesh.is_empty() {
                    let mut to = self.goal();
                    to.fit(self.mesh.min, self.mesh.max, size);
                    self.animate(to);
                }
            }
            Message::Resized(size) => self.size = size,
            Message::Capture(capture) => self.capture = capture,
            Message::Snap(dir) if !self.profile => {
                let mut to = self.goal();
                (to.yaw, to.pitch) = to.facing(dir.as_vec3());
                // A face is seen square-on, so orthographic like a drawing view; an edge or corner keeps perspective.
                let face = dir.abs().element_sum() == 1;
                to.ortho_blend = if face { 1.0 } else { 0.0 };
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
                    let t = (now.saturating_duration_since(anim.start).as_secs_f32() / anim.duration.as_secs_f32()).min(1.0);
                    self.camera = anim.from.lerp(&anim.to, ease(t));
                    if t >= 1.0 {
                        self.anim = None;
                    }
                }
                if let Some((from, to, start)) = self.explode_anim {
                    let t = (now.saturating_duration_since(start).as_secs_f32() / FOCUS_TIME.as_secs_f32()).min(1.0);
                    self.explode = from + (to - from) * ease(t);
                    if t >= 1.0 {
                        self.explode_anim = None;
                    }
                }
            }
            _ => {}
        }
    }

    /// Locks the camera side-on and orthographic (Profile) or unlocks it into the perspective home view (3D).
    pub fn set_profile(&mut self, profile: bool) {
        self.profile = profile;
        let mut to = self.goal();
        if profile {
            (to.yaw, to.pitch, to.roll) = (-FRAC_PI_2, 0.0, 0.0);
            to.ortho_blend = 1.0;
        } else {
            to = home(to);
        }
        self.animate(to);
    }

    /// Advances the camera and exploded-view animations.
    pub fn tick(&mut self, now: Instant) {
        self.camera_update(Message::Tick(now));
    }

    /// Whether a camera or exploded-view animation is running (and so wants frame ticks).
    pub fn animating(&self) -> bool {
        self.anim.is_some() || self.explode_anim.is_some()
    }

    /// The key hints on show: what a direct edit asked for, else what the drag or selection in progress allows.
    pub(crate) fn hints(&self, ctx: &Ctx) -> Vec<(&'static str, &'static str)> {
        if !self.hint.is_empty() {
            return self.hint.clone();
        }
        if ctx.drag.axial.is_some() {
            if ctx.drag.axial.as_ref().is_some_and(|d| d.surface.is_some()) {
                vec![("", "Release to attach radially · move to a node to stack"), ("X", "copies"), ("Esc", "cancel")]
            } else {
                vec![("", "Release on a node to stack · on a tube's side to attach radially"), ("X", "copies"), ("Esc", "cancel")]
            }
        } else if ctx.drag.internal.is_some() {
            vec![("", "Release on a body part to put it inside · ⊘ where it does not fit"), ("Ctrl", "5 mm"), ("Esc", "cancel")]
        } else if let Some(drag) = &ctx.drag.radial {
            if drag.detached && drag.target.is_none() {
                vec![("", "Release to remove · move onto a tube to attach · back to the rocket to slide"), ("X", "copies"), ("Esc", "put back")]
            } else if drag.origin.is_some() {
                vec![("", "Slide or turn round the tube · pull away to detach"), ("X", "copies"), ("Alt", "free"), ("Shift", "to an end"), ("Esc", "cancel")]
            } else {
                vec![("", "Release on a tube"), ("X", "copies"), ("Alt", "free"), ("Shift", "to an end"), ("Esc", "cancel")]
            }
        } else if matches!(ctx.selection, Selection::Separation(_)) {
            vec![("", "Found from the motor mounts · set the timing in the inspector")]
        } else if self.inside == InsideView::Exploded {
            vec![("", "Exploded view is view-only"), ("Esc", "leave")]
        } else if self.stray && self.focus.is_none() && self.out_of_view() {
            // An edit pushed part of the rocket out of a view the user had set (so it was not refitted for them).
            vec![("", "Part of the rocket is out of view"), ("F", "fit")]
        } else {
            Vec::new()
        }
    }

    /// The view toolbar: 3D / 2D, the inside views, the snap step, and the focus tag with its way back.
    fn toolbar<'a>(&'a self, units: crate::units::UnitSystem) -> Element<'a, Message> {
        let mut cells = toolbar::group(
            &[(Projection::ThreeD, Some(Glyph::Cube), "3D"), (Projection::Profile, Some(Glyph::Profile), "2D")],
            self.projection,
            Message::Projection,
        );
        cells.push(toolbar::rule());
        cells.extend(toolbar::group(
            &[
                (InsideView::Solid, None, "Solid"),
                (InsideView::Cutaway, None, "Cutaway"),
                (InsideView::XRay, None, "X-ray"),
                (InsideView::Exploded, None, "Exploded"),
            ],
            self.inside,
            Message::InsideView,
        ));
        cells.push(toolbar::rule());
        // The snap step: a magnet and the flat select, one cell.
        let choices: Vec<GridChoice> = GridStep::ALL.iter().map(|s| GridChoice(*s, units)).collect();
        let step = toolbar::flat_select(choices, GridChoice(GridStep(self.snap_step), units), |c| Message::Step(c.0));
        cells.push(
            row![container(toolbar::icon(Glyph::Magnet, false)).padding([0.0, SPACE_2]), container(step).width(70)]
                .align_y(alignment::Vertical::Center)
                .boxed(),
        );
        let toolbar = toolbar::frame(cells);
        let Some(focus) = &self.focus else {
            return toolbar;
        };
        // The focus tag is plain accent mono text under the toolbar; the way back sits at the viewport's top right.
        let tag = text(focus.tag.clone()).font(MONO).size(TEXT_LABEL).style(theme::colored(|t| t.accent)).wrapping(text::Wrapping::None);
        column![toolbar, container(tag).padding([SPACE_1, SPACE_2])].spacing(SPACE_2).boxed()
    }

    /// The viewport pane: sheet, model, the overlay of the interaction in progress, the markers, the ViewCube and the HUD.
    pub fn view<'a>(&'a self, ctx: &'a Ctx) -> Element<'a, Message> {
        let tokens = ctx.tokens();
        let rocket = ctx.doc.rocket();
        let camera = self.camera;
        let snap_step = self.snap_step;
        // While a part is dragged an assembly overlay owns the viewport; otherwise the manipulation overlay does (the exploded
        // view is view-only: the markers layer selects).
        let (overlay, interaction): (Element<'a, Message>, mouse::Interaction) = if let Some(drag) = &ctx.drag.axial {
            (
                canvas(assembly::Overlay {
                    rocket,
                    drag,
                    camera,
                    cursor: ctx.cursor,
                    snap_angle: ctx.drag.angle_snap,
                    snap_step,
                    margin: ctx.shown().now.map(|b| b.margin_cal).filter(|_| ctx.drag.landing().is_some()),
                    units: ctx.units,
                })
                .width(Fill)
                .height(Fill)
                .boxed(),
                mouse::Interaction::default(),
            )
        } else if let Some(drag) = &ctx.drag.radial {
            (
                canvas(SurfaceOverlay {
                    rocket,
                    drag,
                    camera,
                    snap: ctx.drag.angle_snap,
                    snap_step,
                    cursor: ctx.cursor,
                    margin: ctx.shown().now.map(|b| b.margin_cal).filter(|_| ctx.drag.landing().is_some()),
                    units: ctx.units,
                })
                .width(Fill)
                .height(Fill)
                .boxed(),
                mouse::Interaction::default(),
            )
        } else if let Some(drag) = ctx.drag.internal.as_ref().filter(|d| d.origin.is_none()) {
            (
                canvas(internal::Overlay {
                    rocket,
                    drag,
                    camera,
                    snap_step,
                    cursor: ctx.cursor,
                    margin: ctx.shown().now.map(|b| b.margin_cal).filter(|_| ctx.drag.landing().is_some()),
                    units: ctx.units,
                })
                .width(Fill)
                .height(Fill)
                .boxed(),
                mouse::Interaction::default(),
            )
        } else if self.inside == InsideView::Exploded {
            (Space::new().boxed(), mouse::Interaction::default())
        } else {
            let selected = ctx.selection.part();
            let also = ctx.selected_parts().into_iter().skip(1).collect();
            let instance = ctx.symmetry.instance.filter(|(id, _)| Some(*id) == selected).map_or(0, |(_, i)| i);
            (
                canvas(Manipulator {
                    rocket,
                    camera,
                    selected,
                    also,
                    instance,
                    hover: self.hover,
                    cursor: ctx.cursor,
                    snap_step,
                    inside: self.inside,
                    profile: self.profile,
                    units: ctx.units,
                    pod: selected.and_then(|id| pods::PodView::of_part(rocket, id, &camera)),
                    balance: ctx.shown().now.map(|b| markers::BalanceMarks {
                        cg: b.cg,
                        cp: b.cp,
                        margin: b.margin_cal,
                        show: (ctx.show_cg, ctx.show_cp),
                        staged: ctx.physics.stages().len() > 1,
                    }),
                })
                .width(Fill)
                .height(Fill)
                .boxed(),
                self.hover.interaction(),
            )
        };
        let reveal = ctx.selection.part().is_some_and(|id| matches!(rocket.kind_of(id), Some(PartRef::Internal(_))));
        let phase = ctx.physics.phase();
        let fade_from = dropped_from(ctx.physics.stages(), phase);
        let span = (!self.mesh.is_empty()).then_some((self.mesh.min.z, self.mesh.max.z));
        let sheet: Element<'_, Message> = canvas(sheet::Sheet { camera, span, units: ctx.units }).width(Fill).height(Fill).boxed();
        let sheet = empty::over(sheet, ctx);
        let model: Element<'_, Message> = shader(scene::Scene {
            camera,
            tokens: *tokens,
            mesh: self.mesh.clone(),
            interaction,
            orbit: !self.profile,
            inside: self.inside,
            reveal,
            explode: self.explode,
            cut: self.cut,
            fade_from,
            capture: self.capture.clone(),
        })
        .width(Fill)
        .height(Fill)
        .boxed();
        let motors = ctx
            .physics
            .stages()
            .iter()
            .flat_map(|s| {
                s.mounts.iter().filter_map(move |m| {
                    let slot = rocket.active_configuration().slot(*m)?;
                    Some((s.index, slot.motor.as_ref()?.designation.clone()))
                })
            })
            .collect::<Vec<(usize, String)>>();
        let markers = |hit: bool, motors: Vec<(usize, String)>| -> Element<'a, Message> {
            canvas(markers::Markers {
                rocket,
                camera,
                mesh: &self.mesh,
                balance: ctx.shown().now.map(|b| (b.cg, b.cp, b.margin_cal)),
                ghost: ctx.shown().committed.map(|b| (b.cg, b.cp)),
                show: (ctx.show_cg, ctx.show_cp),
                stages: ctx.physics.stages(),
                motors,
                selection: ctx.selection,
                inside: self.inside,
                explode: self.explode,
                cut: self.cut,
                units: ctx.units,
                interactive: !ctx.drag.active(),
                hit,
            })
            .width(Fill)
            .height(Fill)
            .boxed()
        };
        // Drawn markers and the HUD panels sit under the manipulation overlay (its drag tag must cover them); a transparent layer
        // above it takes the clicks on the section handle, the separation diamonds and (exploded) the parts.
        let hints = self.hints(ctx);
        let mut layers = stack![sheet, model, markers(false, motors.clone())];
        layers = layers.push(overlay).push(markers::bottom_bar(crate::app::physics::BALANCE_MACH, 0.0, hints));
        layers = layers.push(markers(true, motors));
        if !self.profile {
            let cube = canvas(cube::ViewCube { camera }).width(TRIAD_GUTTER + CUBE_SIZE).height(CUBE_SIZE);
            let cube: Element<'_, Message> = container(cube).align_right(Fill).padding(SPACE_4).boxed();
            layers = layers.push(cube);
        }
        // Overlays float `space-4` from the viewport edges.
        let mut top: Vec<Element<'a, Message>> = vec![container(self.toolbar(ctx.units)).padding(SPACE_4).boxed()];
        if self.focus.is_some() {
            let back = Element::from(button("Esc · back", Weight::Secondary).on_press(Message::ExitFocus));
            top.push(container(back).align_right(Fill).padding(SPACE_4).boxed());
        }
        stack![layers].extend(top).boxed()
    }

    /// Where the camera is heading: the end of the running animation, or the camera itself.
    fn goal(&self) -> Camera {
        self.anim.as_ref().map_or(self.camera, |a| a.to)
    }

    /// Dragging the fore end of a stacked part keeps the aft end, and everything behind it, still in space: the stations run from
    /// the nose tip, so the rocket grows at the nose side and the camera moves with it by the change in length (the part's
    /// fixed end stays where the cursor left it). A cancel moves the camera back; a release or a typed value keeps it and
    /// returns the whole shift, for the undo step to carry.
    fn follow_fore_edit(&mut self, edit: &manipulate::Edit, ctx: &Ctx) -> Option<f32> {
        use manipulate::Edit;
        let rocket = ctx.doc.rocket();
        let delta = match edit {
            Edit::Dim(id, Dim::ForeLength, value) | Edit::Commit(id, Dim::ForeLength, value) if rocket.parts().iter().any(|p| p.id == *id) => {
                let now = rocket.dim(*id, Dim::Length).unwrap_or(0.0);
                rocket.clamp_dim(*id, Dim::ForeLength, *value).map_or(0.0, |(v, _)| v - now) as f32
            }
            Edit::Cancel => -self.fore_pan,
            _ => 0.0,
        };
        let total = self.fore_pan + delta;
        self.fore_pan = match edit {
            Edit::Cancel | Edit::End | Edit::Commit(..) => 0.0,
            _ => total,
        };
        self.shift_axis(delta);
        (matches!(edit, Edit::End | Edit::Commit(..)) && total != 0.0).then_some(total)
    }

    /// Moves the camera (and any animation in flight) `dz` along the rocket's axis. Undo and redo of a fore-end length drag
    /// use it to keep the part's aft end still on screen.
    pub fn shift_axis(&mut self, dz: f32) {
        if dz != 0.0 {
            self.camera.target.z += dz;
            if let Some(anim) = self.anim.as_mut() {
                anim.from.target.z += dz;
                anim.to.target.z += dz;
            }
        }
    }

    /// Ends any animation at its goal (so a pan or zoom during a snap does not get overwritten).
    fn settle(&mut self) {
        if let Some(anim) = self.anim.take() {
            self.camera = anim.to;
        }
    }

    fn animate(&mut self, to: Camera) {
        self.animate_for(to, SNAP_TIME);
    }

    fn animate_for(&mut self, to: Camera, duration: Duration) {
        if ANIMATE.load(std::sync::atomic::Ordering::Relaxed) {
            self.anim = Some(Anim { from: self.camera, to, start: Instant::now(), duration });
        } else {
            self.anim = None;
            self.camera = to;
        }
    }
}

/// The home view: three-quarter from above, forward and the left side, like Fusion's home, in perspective.
fn home(camera: Camera) -> Camera {
    let (yaw, pitch) = camera.facing(Vec3::new(-1.0, 0.45, -0.55));
    Camera { yaw, pitch, roll: 0.0, ortho_blend: 0.0, ..camera }
}

fn ease(t: f32) -> f32 {
    // Cubic ease-in-out.
    if t < 0.5 {
        4.0 * t * t * t
    } else {
        1.0 - (-2.0 * t + 2.0).powi(3) / 2.0
    }
}

/// The station (from the tip, metres) aft of which the body is drawn faint: the aft end of the first stage still flying in `phase`
/// (everything behind it has dropped away). `None` while the whole stack flies.
fn dropped_from(stages: &[bowshock_model::Stage], phase: bowshock_sim::from_model::Phase) -> Option<f32> {
    let first = phase.first_stage();
    if stages.len() < 2 || first <= 1 {
        return None;
    }
    stages.iter().find(|s| s.index == first).map(|s| s.aft_station as f32)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Runs `message` and lands any animation at its goal.
    fn run(view: &mut View, message: Message) {
        view.camera_update(message);
        view.settle();
    }

    /// A view showing a one-tube rocket.
    fn with_rocket() -> View {
        let mut view = View::new();
        let mut rocket = bowshock_model::Rocket::default();
        rocket.insert(0, bowshock_model::PartKind::BodyTube { length: 0.5, diameter: 0.1 }).unwrap();
        view.mesh = Arc::new(Mesh::from_rocket(&rocket, mesh::Marks::default()));
        view
    }

    #[test]
    fn an_empty_rocket_cannot_be_orbited_or_rolled() {
        let mut view = View::new();
        let before = view.camera;
        run(&mut view, Message::Orbit(Vector::new(40.0, 10.0)));
        run(&mut view, Message::RollBy(Vector::new(30.0, 0.0)));
        assert_eq!((view.camera.yaw, view.camera.pitch, view.camera.roll), (before.yaw, before.pitch, before.roll));
        let mut view = with_rocket();
        run(&mut view, Message::Orbit(Vector::new(40.0, 10.0)));
        assert_ne!(view.camera.yaw, before.yaw);
    }

    #[test]
    fn a_cube_face_is_orthographic_and_orbiting_off_it_eases_back_into_perspective() {
        let mut view = with_rocket();
        run(&mut view, Message::Snap(IVec3::NEG_X));
        assert!(view.camera.is_ortho(), "a face is seen square-on in an orthographic view");

        run(&mut view, Message::Orbit(Vector::new(4.0, 0.0)));
        assert!(!view.camera.is_ortho() && view.camera.ortho_blend > 0.0, "one small orbit step only starts the blend");
        for _ in 0..10 {
            run(&mut view, Message::Orbit(Vector::new(4.0, 0.0)));
        }
        assert_eq!(view.camera.ortho_blend, 0.0, "a drag off the face ends in perspective");

        run(&mut view, Message::Snap(IVec3::new(-1, 1, 0)));
        assert_eq!(view.camera.ortho_blend, 0.0, "an edge keeps perspective");
        run(&mut view, Message::Snap(IVec3::Y));
        run(&mut view, Message::Snap(IVec3::new(1, 1, -1)));
        assert_eq!(view.camera.ortho_blend, 0.0, "a corner leaves the orthographic face view");
    }
}

#[cfg(test)]
mod phase_tests {
    use super::*;
    use bowshock_sim::from_model::Phase;

    fn two_stage() -> bowshock_model::Rocket {
        bowshock_model::Rocket::from_bytes(include_bytes!("../../assets/examples/two_stage.bowshock")).unwrap()
    }

    /// Dropping the booster (Phase "Stage 2") fades only what is aft of the separation: the sustainer's stations (fore of the
    /// joint) stay solid. It used to fade from the sustainer's fore end (the nose), i.e. the whole rocket.
    #[test]
    fn dropping_the_booster_fades_only_the_booster() {
        let rocket = two_stage();
        let stages = rocket.stages();
        let joint = stages[0].fore_station as f32;
        assert!(joint > 0.1, "the example separates well behind the nose");
        assert_eq!(dropped_from(&stages, Phase::Full), None);
        let from = dropped_from(&stages, Phase::UpperFrom(2)).expect("a staged rocket drops the booster");
        assert!((from - joint).abs() < 1e-6, "fades aft of the separation ({from} vs {joint})");
        let sustainer = stages[1].fore_station as f32..stages[1].aft_station as f32;
        assert!(sustainer.end <= from + 1e-6 && sustainer.start < from, "the sustainer is not faded");
    }

    /// The balance, status-strip numbers and lanes follow the remaining stack: only the sustainer's parts are left.
    #[test]
    fn the_phase_recomputes_for_the_remaining_stack_only() {
        let rocket = two_stage();
        let mut physics = crate::app::physics::Physics::new();
        physics.sync(&rocket, 1);
        let full = *physics.balance().expect("balance");
        physics.set_phase(Phase::UpperFrom(2));
        physics.sync(&rocket, 1);
        let upper = *physics.balance().expect("balance");
        let joint = physics.stages()[0].fore_station;
        assert!(upper.mass < full.mass && upper.length < full.length, "only the sustainer remains");
        assert!((upper.length - joint).abs() < 1e-6, "the remaining stack ends at the separation");
        assert!(upper.cg < joint && upper.cp < joint, "CG and CP are on the sustainer");
        assert_eq!(physics.phase(), Phase::UpperFrom(2), "the phase is kept, not reset to the full stack");
    }
}
