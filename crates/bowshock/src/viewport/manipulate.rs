//! The manipulation overlay: direct handling of the parts already in the viewport (design kit: DirectEdit, Snapping). It
//! hover-picks a part under the cursor, selects on click, picks a part up when it is pressed and dragged, and drags the
//! handles of the selected part: one handle, one dimension (see [`handles`](super::handles)). It draws the selection (corner
//! brackets, a mono tag, handles, labels, the aft-view inset), the live tag of a drag with its old-size line and arrow, and
//! with Shift the snap ticks. Everything it decides is reported as an [`Edit`] or other viewport message; the app owns the
//! rocket, and every handle edit is one undo step through [`apply`].
//!
//! A radial part is a group of linked copies: clicking any copy selects the group, with solid brackets on the clicked copy
//! and dashed ones on its siblings.

use std::time::{Duration, Instant};

use bowshock_model::{Dim, HandleView, PartId, PartRef, Rocket, Target};
use glam::Vec3;
use iced::widget::canvas::{self, Action, Event, Frame, Geometry, LineDash, Path, Stroke, Text};
use iced::{alignment, keyboard, mouse, Color, Point, Rectangle, Renderer, Size, Task, Theme, Vector};

use super::assembly::tag;
use super::handles::{self, Axis, Context, Inset, Placed};
use super::labels::{self, Request};
use super::markers::{self, BalanceMarks};
use super::pick::{self, Pick};
use super::pods::PodView;
use super::radial::Placement;
use super::snap;
use super::{Camera, InsideView, Message};
use crate::app::ui::{radial_size_label, size_label};
use crate::app::{self, Ctx};
use crate::theme::{self, MONO};
use crate::tokens::{Tokens, TEXT_LABEL};
use crate::units::{self, Quantity, UnitSystem};

/// Movement in px after pressing a part beyond which it is picked up rather than clicked.
const SLOP: f32 = 4.0;
/// Side of a square handle and diameter of a circular one.
const SQUARE: f32 = 8.0;
const CIRCLE: f32 = 9.0;
/// A handle is grabbed within this many px (per axis) of its centre: a 12 px hit box.
const HIT: f32 = 6.0;
/// Corner bracket leg length, and the gap between the part's bounds and its brackets.
const BRACKET: f32 = 8.0;
const BRACKET_GAP: f32 = 4.0;
/// Non-primary handles closer than this to another hide.
const CROWD: f32 = 16.0;
/// A part smaller than this on screen shows no handles.
const MIN_PART: f32 = 24.0;
/// Two presses within this long are a double-click.
const DOUBLE: Duration = Duration::from_millis(400);
/// Width of one MONO glyph as a fraction of the font size (IBM Plex Mono advances 0.6 em).
const GLYPH_EM: f32 = 0.6;

/// What is under the cursor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Hover {
    #[default]
    None,
    /// An axial part, or a radial group, by id.
    Part(PartId),
    /// A handle of the selected part, by the dimension it edits.
    Handle(Dim),
}

impl Hover {
    /// The part to draw hovered.
    pub fn part(self) -> Option<PartId> {
        match self {
            Hover::Part(id) => Some(id),
            _ => None,
        }
    }

    /// The cursor to show: a grab hand over a part, a resize arrow over a handle.
    pub fn interaction(self) -> mouse::Interaction {
        match self {
            Hover::None => mouse::Interaction::default(),
            Hover::Part(_) => mouse::Interaction::Grab,
            Hover::Handle(Dim::Diameter | Dim::ForeDiameter | Dim::InnerDiameter | Dim::Span | Dim::Wall | Dim::Height | Dim::Shape) => {
                mouse::Interaction::ResizingVertically
            }
            Hover::Handle(Dim::Cant | Dim::Angle | Dim::Vertex(_) | Dim::Thickness) => mouse::Interaction::Grab,
            Hover::Handle(_) => mouse::Interaction::ResizingHorizontally,
        }
    }
}

/// A handle edit, reported by the overlay and applied by [`apply`]: a drag is `Begin`, any number of `Dim`/`Vertex`, then
/// `End` (one undo step) or `Cancel` (the rocket goes back as it was).
#[derive(Debug, Clone)]
pub enum Edit {
    /// A handle was grabbed: the document starts collecting one undo step.
    Begin(PartId, &'static str),
    /// Dimension `Dim` of the part is now this value (limited by the model).
    Dim(PartId, Dim, f64),
    /// Freeform fin vertex moved to `(z, height)`.
    Vertex(PartId, usize, f64, f64),
    /// A typed value was entered: set it and end the drag.
    Commit(PartId, Dim, f64),
    /// The handle was released.
    End,
    /// Escape: put the part back.
    Cancel,
    /// Double-click on a freeform fin's edge after vertex `after`: add a vertex at `(z, height)`.
    AddVertex(PartId, usize, f64, f64),
    /// Double-click on a freeform fin's vertex: remove it.
    RemoveVertex(PartId, usize),
    /// Double-click on a handle: focus the Inspector field of the dimension.
    Focus(Dim),
}

impl Edit {
    /// The key-hint bar to show for this edit: the drag's keys on `Begin`, none (empty) when it ends.
    pub fn hint(&self) -> Option<Vec<(&'static str, &'static str)>> {
        match self {
            Edit::Begin(_, verb) => Some(vec![("", *verb), ("Shift", "snap"), ("Ctrl", "5 mm"), ("0-9", "type exact"), ("Esc", "cancel")]),
            Edit::End | Edit::Cancel | Edit::Commit(..) => Some(Vec::new()),
            _ => None,
        }
    }
}

/// Applies an [`Edit`] to the document. Handle drags use the document's drag support: every change goes live without an
/// undo step, and `End` records one (`Cancel` restores the whole rocket).
pub fn apply(edit: Edit, ctx: &mut Ctx) -> Task<app::Message> {
    match edit {
        Edit::Begin(id, _) => {
            let name = ctx.doc.rocket().kind_of(id).map(|p| p.name()).unwrap_or_default();
            ctx.doc.begin_drag(&format!("Resize {}", name.to_lowercase()));
        }
        Edit::Dim(id, dim, value) => {
            let copy = ctx.symmetry.copy_of(id);
            ctx.doc.drag_update(|r| r.set_instance_dim(id, copy, dim, value));
        }
        Edit::Vertex(id, i, z, h) => {
            ctx.doc.drag_update(|r| r.set_vertex(id, i, z, h));
        }
        Edit::Commit(id, dim, value) => {
            let copy = ctx.symmetry.copy_of(id);
            ctx.doc.drag_update(|r| r.set_instance_dim(id, copy, dim, value));
            ctx.doc.end_drag(true);
        }
        Edit::End => {
            ctx.doc.end_drag(true);
        }
        Edit::Cancel => {
            ctx.doc.end_drag(false);
        }
        Edit::AddVertex(id, after, z, h) => {
            ctx.doc.edit("Add vertex", |r| r.add_vertex(id, after, z, h));
        }
        Edit::RemoveVertex(id, i) => {
            ctx.doc.edit("Remove vertex", |r| r.remove_vertex(id, i));
        }
        Edit::Focus(dim) => return Task::done(app::Message::Inspector(crate::build::inspector::Message::Focus(dim))),
    }
    Task::none()
}

pub struct Overlay<'a> {
    pub rocket: &'a Rocket,
    pub camera: Camera,
    pub selected: Option<PartId>,
    /// The parts selected along with `selected` (a multi-selection): every one is bracketed, none has handles.
    pub also: Vec<PartId>,
    /// Which copy of a selected radial group was clicked.
    pub instance: usize,
    pub hover: Hover,
    /// The cursor in window coordinates (tracked by the app, so it also draws without a live cursor).
    pub cursor: Option<Point>,
    /// The view toolbar's grid step, metres.
    pub snap_step: f64,
    /// Solid / Cutaway / X-ray / Exploded (internal handles show unless Solid).
    pub inside: InsideView,
    /// The 2D profile: every handle shows.
    pub profile: bool,
    /// The display unit system, for tags, labels and typed values.
    pub units: UnitSystem,
    /// The pod holding the selected part, seen through its own frame, when the selection is inside a pod: handles and brackets
    /// of its parts are worked out there (`PodView::of_part`).
    pub pod: Option<PodView>,
    /// The CG / CP markers' values, so labels keep off the marker labels.
    pub balance: Option<BalanceMarks>,
}

/// A press on a part (or on empty space) that has not yet become a click or a pick-up.
#[derive(Debug)]
pub struct Press {
    at: Point,
    target: Option<Pick>,
}

/// What the drag looks like right now.
#[derive(Debug, Clone, Copy, Default)]
struct Live {
    /// The value the part has (limited).
    value: f64,
    /// The target that caught it, when Shift is held.
    caught: Option<usize>,
    /// Why the drag stopped, if it did.
    limit: Option<&'static str>,
}

/// A handle being dragged.
#[derive(Debug)]
struct Drag {
    id: PartId,
    /// The handle as it was when grabbed: its mapping from screen to value.
    placed: Placed,
    base: f64,
    base_vertex: (f64, f64),
    from: Point,
    targets: Vec<Target>,
    typed: String,
    /// The last value sent, so a move that changes nothing sends nothing.
    last: Option<(f64, f64)>,
    live: Live,
}

#[derive(Debug)]
pub struct State {
    press: Option<Press>,
    drag: Option<Drag>,
    modifiers: keyboard::Modifiers,
    /// The last handle press (for double-click) and the last press on a freeform edge.
    last_handle: Option<(Instant, PartId, Dim)>,
    last_edge: Option<(Instant, PartId, usize)>,
}

/// A drive script cannot hold keys, so `BOWSHOCK_DRIVE_MODIFIERS` (`shift`, `alt`, `ctrl`, joined by `+`) starts the overlays
/// (the viewport's and the lanes') with those modifiers held. Unset in normal use.
pub fn drive_modifiers() -> keyboard::Modifiers {
    let mut modifiers = keyboard::Modifiers::empty();
    for name in std::env::var("BOWSHOCK_DRIVE_MODIFIERS").unwrap_or_default().split('+') {
        modifiers |= match name.trim() {
            "shift" => keyboard::Modifiers::SHIFT,
            "alt" => keyboard::Modifiers::ALT,
            "ctrl" => keyboard::Modifiers::CTRL,
            _ => keyboard::Modifiers::empty(),
        };
    }
    modifiers
}

impl Default for State {
    fn default() -> Self {
        State { press: None, drag: None, modifiers: drive_modifiers(), last_handle: None, last_edge: None }
    }
}

/// The selected part.
enum Selection<'a> {
    Axial { z0: f64, kind: &'a bowshock_model::PartKind },
    Radial { radial: &'a bowshock_model::RadialPart, at: Placement },
    Internal,
}

/// The handles on show for a part, with the hint flag and inset.
#[derive(Debug, Default)]
pub struct Visible {
    /// The handles to draw and hit-test.
    pub placed: Vec<Placed>,
    /// Some handles were hidden for crowding: show the zoom hint.
    pub crowded: bool,
    /// The aft-view inset, when shown.
    pub inset: Option<Inset>,
}

/// Which handles of `id` show: primary ones on select; every one with `all` (Alt held or the 2D profile), the aft-view ones
/// in the inset; internal ones unless the view is Solid (or when the part itself is internal); none for a part smaller than
/// 24 px on screen; non-primary ones hidden when within 16 px of another.
pub fn visible(cx: &Context, id: PartId, all: bool, inside: InsideView) -> Visible {
    let b = cx.bounds;
    if b.width.max(b.height) < MIN_PART {
        return Visible::default();
    }
    let internal = matches!(cx.rocket.kind_of(id), Some(PartRef::Internal(_)));
    let mut placed: Vec<Placed> = handles::place(cx, id)
        .into_iter()
        .filter(|h| {
            let wanted = all || h.spec.primary;
            let seen = match h.spec.view {
                HandleView::Side => true,
                HandleView::Aft => all,
                HandleView::Inside => inside != InsideView::Solid || internal,
            };
            wanted && seen
        })
        .collect();
    let crowded_with = |p: &Placed, others: &[Placed]| {
        others.iter().any(|o| o.spec.dim != p.spec.dim && o.at.distance(p.at) < CROWD && handles::in_inset(&o.spec) == handles::in_inset(&p.spec))
    };
    let all_placed = placed.clone();
    let mut crowded = false;
    placed.retain(|h| {
        let hide = !h.spec.primary && crowded_with(h, &all_placed);
        crowded |= hide;
        !hide
    });
    let inset = (all && placed.iter().any(|h| handles::in_inset(&h.spec))).then(|| handles::inset(cx, id)).flatten();
    Visible { placed, crowded, inset }
}

impl Overlay<'_> {
    fn selection(&self) -> Option<Selection<'_>> {
        self.selection_of(self.selected?)
    }

    /// The clicked copy of group `id`: the one `instance` names for the primary part, the first for any other.
    fn copy_of(&self, id: PartId) -> usize {
        if self.selected == Some(id) {
            self.instance
        } else {
            0
        }
    }

    /// The rocket and camera part `id` is seen through: the document's, or for a part inside the selected part's pod, that
    /// pod's own (its first copy; the others are bracketed in dashes). `None` for a part of some other pod.
    fn frame_of(&self, id: PartId) -> Option<(&Rocket, &Camera)> {
        match self.rocket.pod_of(id) {
            None => Some((self.rocket, &self.camera)),
            Some(pod) => self.pod.as_ref().filter(|v| v.pod == pod.id).map(|v| (&v.sub, &v.cameras[0])),
        }
    }

    fn selection_of(&self, id: PartId) -> Option<Selection<'_>> {
        let (rocket, _) = self.frame_of(id)?;
        self.selection_in(rocket, id)
    }

    fn selection_in<'r>(&self, rocket: &'r Rocket, id: PartId) -> Option<Selection<'r>> {
        if let Some(index) = rocket.index_of(id) {
            return Some(Selection::Axial { z0: rocket.stations()[index], kind: &rocket.parts()[index].kind });
        }
        if let Some(radial) = rocket.radial(id) {
            let instance = self.copy_of(id).min(radial.angles().len().saturating_sub(1));
            return Some(Selection::Radial { radial, at: pick::placement(rocket, radial, instance)? });
        }
        rocket.internal(id).map(|_| Selection::Internal)
    }

    fn instance(&self) -> usize {
        self.instance
    }

    /// The screen bounds of the selected part.
    fn bounds_of_selection(&self, size: Size) -> Option<Rectangle> {
        self.bounds_of(self.selected?, size)
    }

    /// The screen bounds of part `id`.
    fn bounds_of(&self, id: PartId, size: Size) -> Option<Rectangle> {
        let (rocket, camera) = self.frame_of(id)?;
        self.bounds_in(rocket, camera, id, size)
    }

    /// The screen bounds of part `id` of `rocket` seen through `camera` (a pod's part through one copy's camera).
    fn bounds_in(&self, rocket: &Rocket, camera: &Camera, id: PartId, size: Size) -> Option<Rectangle> {
        Some(match self.selection_in(rocket, id)? {
            Selection::Axial { z0, kind } => pick::screen_bounds(camera, size, z0, kind),
            Selection::Radial { radial, at } => {
                let kind = radial.effective(self.copy_of(id).min(radial.angles().len().saturating_sub(1))).kind;
                pick::radial_bounds(camera, size, &kind, at)
            }
            Selection::Internal => {
                let res = rocket.resolved(id)?;
                let z0 = rocket.part_station(id)?;
                let up = handles::radial_up(camera);
                let r = (res.outer_diameter / 2.0) as f32;
                let zs = [z0 as f32, (z0 + res.length) as f32];
                let pts = zs.into_iter().flat_map(|z| [Vec3::Z * z + up * r, Vec3::Z * z - up * r]);
                let pts: Vec<Point> = pts.map(|p| camera.project(p, size)).collect();
                bounds(&pts)
            }
        })
    }

    fn context(&self, size: Size, bounds: Rectangle) -> Context<'_> {
        let (rocket, camera) = self.selected.and_then(|id| self.frame_of(id)).unwrap_or((self.rocket, &self.camera));
        Context { rocket, camera, size, instance: self.instance(), bounds, cube: !self.profile }
    }

    /// The tag that selects the pod set while a part inside it is selected: its place (below the pod's copies on screen), the
    /// pod and its text (`POD ×2`).
    fn pod_chip(&self, size: Size) -> Option<(PartId, Rectangle, String)> {
        let view = self.pod.as_ref().filter(|v| self.selected.is_some_and(|id| self.rocket.pod_of(id).is_some_and(|p| p.id == v.pod)))?;
        let pod = self.rocket.radial(view.pod)?;
        let union = (0..pod.angles().len())
            .filter_map(|i| pick::placement(self.rocket, pod, i))
            .map(|at| pick::radial_bounds(&self.camera, size, &pod.kind, at))
            .reduce(|a, b| a.union(&b))?;
        let text = if pod.count > 1 { format!("POD ×{}", pod.count) } else { "POD".to_string() };
        let width = text.chars().count() as f32 * TEXT_LABEL * GLYPH_EM + 12.0;
        Some((pod.id, Rectangle::new(Point::new(union.x, union.y + union.height + 6.0), Size::new(width, TEXT_LABEL + 8.0)), text))
    }

    fn visible(&self, size: Size, modifiers: keyboard::Modifiers) -> Visible {
        // Handles edit one part's dimension; with several parts selected there is no such part, so none show.
        if !self.also.is_empty() {
            return Visible::default();
        }
        let (Some(id), Some(b)) = (self.selected, self.bounds_of_selection(size)) else {
            return Visible::default();
        };
        visible(&self.context(size, b), id, modifiers.alt() || self.profile, self.inside)
    }

    fn handle_at(&self, at: Point, size: Size, modifiers: keyboard::Modifiers) -> Option<Placed> {
        self.visible(size, modifiers)
            .placed
            .into_iter()
            .filter(|p| (p.at.x - at.x).abs() <= HIT && (p.at.y - at.y).abs() <= HIT)
            .min_by(|a, b| a.at.distance(at).total_cmp(&b.at.distance(at)))
    }

    fn hover_at(&self, at: Point, size: Size, modifiers: keyboard::Modifiers) -> Hover {
        if let Some(h) = self.handle_at(at, size, modifiers) {
            return Hover::Handle(h.spec.dim);
        }
        pick::pick(self.rocket, &self.camera, at, size).map_or(Hover::None, |p| Hover::Part(p.id()))
    }

    fn quantity(dim: Dim) -> Quantity {
        match dim {
            Dim::Cant | Dim::Angle => Quantity::Angle,
            Dim::Shape => Quantity::Scalar,
            _ => Quantity::Length,
        }
    }

    /// `value` as a number in the display unit.
    fn num(&self, dim: Dim, value: f64) -> String {
        units::format_in(value, Self::quantity(dim).unit_for(self.units))
    }

    /// `value` with its unit symbol, `76.20 mm`.
    fn with_unit(&self, dim: Dim, value: f64) -> String {
        let unit = Self::quantity(dim).unit_for(self.units);
        if unit.symbol.is_empty() {
            units::format_in(value, unit)
        } else {
            format!("{} {}", units::format_in(value, unit), unit.symbol)
        }
    }

    /// A signed change, `+58.0`.
    fn delta(&self, dim: Dim, delta: f64) -> String {
        let text = self.num(dim, delta);
        if delta > 0.0 && !text.starts_with('-') && text.chars().any(|c| c.is_ascii_digit() && c != '0') {
            format!("+{text}")
        } else {
            text
        }
    }

    /// The handle's own label: `SWEEP 60.0`, `Ø 76.2`.
    fn handle_label(&self, id: PartId, dim: Dim, value: f64) -> String {
        match dim {
            Dim::Diameter | Dim::ForeDiameter | Dim::InnerDiameter => format!("Ø {}", self.num(dim, value)),
            _ => format!("{} {}", self.rocket.dim_label(id, dim), self.num(dim, value)),
        }
    }

    /// Parses a typed value in the display unit of `dim`.
    fn parse(&self, dim: Dim, text: &str) -> Option<f64> {
        let q = Self::quantity(dim);
        units::parse_quantity(text, q, q.unit_for(self.units), &|_| None).ok()
    }

    /// Screen px per unit of value for catching targets: along the line, or the lever of an arc.
    fn px_per_unit(placed: &Placed) -> f32 {
        match placed.axis {
            Axis::Line { per_unit, .. } => per_unit,
            Axis::Arc { centre } => placed.at.distance(centre).max(1.0),
            Axis::Plane { .. } => 1.0,
        }
    }

    /// The drag at `at` with `modifiers`: updates its live state and returns the value to set, if it changed.
    fn drag_to(&self, drag: &mut Drag, at: Point, modifiers: keyboard::Modifiers) -> Option<Edit> {
        let (id, dim) = (drag.id, drag.placed.spec.dim);
        let coarse = modifiers.control() || modifiers.logo();
        if let (Dim::Vertex(i), Axis::Plane { .. }) = (dim, drag.placed.axis) {
            let (z, h) = handles::vertex_after(&drag.placed, drag.base_vertex, drag.from, at);
            let step = snap::step(Dim::Length, self.snap_step, coarse);
            let (z, h) = (snap::quantize(z, step), snap::quantize(h, step).max(0.0));
            drag.live = Live { value: z, caught: None, limit: None };
            return (drag.last != Some((z, h))).then(|| {
                drag.last = Some((z, h));
                Edit::Vertex(id, i, z, h)
            });
        }
        let raw = handles::value_after(&drag.placed, drag.base, drag.from, at);
        let ppu = Self::px_per_unit(&drag.placed);
        let snapped = snap::snap(dim, raw, &drag.targets, ppu, modifiers.shift(), coarse, self.snap_step);
        let (applied, limit) = self.rocket.clamp_dim(id, dim, snapped.value).unwrap_or((snapped.value, None));
        drag.live = Live { value: applied, caught: snapped.caught.filter(|_| limit.is_none()), limit };
        (drag.last != Some((snapped.value, 0.0))).then(|| {
            drag.last = Some((snapped.value, 0.0));
            Edit::Dim(id, dim, snapped.value)
        })
    }

    /// The edge of a selected freeform fin under `at` (the index of its first vertex) with the `(z, h)` there.
    fn edge_at(&self, at: Point, vis: &Visible) -> Option<(PartId, usize, (f64, f64))> {
        let id = self.selected?;
        let verts: Vec<&Placed> = {
            let mut v: Vec<&Placed> = vis.placed.iter().filter(|p| matches!(p.spec.dim, Dim::Vertex(_))).collect();
            v.sort_by_key(|p| match p.spec.dim {
                Dim::Vertex(i) => i,
                _ => 0,
            });
            v
        };
        (0..verts.len()).find_map(|i| {
            let (a, b) = (verts[i], verts[(i + 1) % verts.len()]);
            let ab = b.at - a.at;
            let len2 = ab.x * ab.x + ab.y * ab.y;
            if len2 < 1.0 {
                return None;
            }
            let t = (((at.x - a.at.x) * ab.x + (at.y - a.at.y) * ab.y) / len2).clamp(0.0, 1.0);
            let foot = a.at + ab * t;
            (foot.distance(at) <= 6.0 && i + 1 < verts.len()).then(|| {
                let Dim::Vertex(ia) = a.spec.dim else { return None };
                let base = self.rocket.vertex(id, ia)?;
                Some((id, ia, handles::vertex_after(a, base, a.at, foot)))
            })?
        })
    }
}

fn bounds(points: &[Point]) -> Rectangle {
    let (mut min, mut max) = (Point::new(f32::MAX, f32::MAX), Point::new(f32::MIN, f32::MIN));
    for p in points {
        (min.x, min.y, max.x, max.y) = (min.x.min(p.x), min.y.min(p.y), max.x.max(p.x), max.y.max(p.y));
    }
    Rectangle::new(min, Size::new(max.x - min.x, max.y - min.y))
}

/// Corner brackets round `b`, solid or dashed.
fn brackets(frame: &mut Frame, b: Rectangle, color: Color, dashed: bool) {
    let (x0, y0, x1, y1) = (b.x - BRACKET_GAP, b.y - BRACKET_GAP, b.x + b.width + BRACKET_GAP, b.y + b.height + BRACKET_GAP);
    if dashed {
        let outline = Path::rectangle(Point::new(x0, y0), Size::new(x1 - x0, y1 - y0));
        let stroke = Stroke { line_dash: LineDash { segments: &[4.0, 3.0], offset: 0 }, ..Stroke::default().with_color(color).with_width(1.0) };
        frame.stroke(&outline, stroke);
        return;
    }
    let corners = Path::new(|p| {
        for (x, y, sx, sy) in [(x0, y0, 1.0, 1.0), (x1, y0, -1.0, 1.0), (x1, y1, -1.0, -1.0), (x0, y1, 1.0, -1.0)] {
            p.move_to(Point::new(x + sx * BRACKET, y));
            p.line_to(Point::new(x, y));
            p.line_to(Point::new(x, y + sy * BRACKET));
        }
    });
    frame.stroke(&corners, Stroke::default().with_color(color).with_width(1.5));
}

fn text(frame: &mut Frame, content: impl Into<String>, at: Point, color: Color, size: f32, ax: alignment::Horizontal, ay: alignment::Vertical) {
    frame.fill_text(Text { content: content.into(), position: at, color, size: size.into(), font: MONO, align_x: ax.into(), align_y: ay, ..Text::default() });
}

/// The floating view toolbar's extent in the viewport (logical px): handle labels stay clear of it.
const TOOLBAR_X: f32 = 600.0;
const TOOLBAR_Y: f32 = 76.0;
/// Height of the viewport's bottom row (legend, key hints, scale bar) that labels stay out of, and the free band of that row:
/// the legend (and its gap) at the left, the scale bar (and its gap) at the right.
const BOTTOM_STRIP: f32 = 44.0;
const BAR_LEFT: f32 = 16.0 + 226.0 + 12.0;
const BAR_RIGHT: f32 = 16.0 + 120.0 + 12.0;
/// The ViewCube's distance from the viewport's top-right corner.
const CUBE_MARGIN: f32 = 16.0;
/// Side of the square a handle occupies for labels to keep off.
const HANDLE_BLOCK: f32 = 14.0;

/// What the key-hint bar calls the thing being dragged (`Drag tip`, `Drag face`, `Drag edge`...).
fn drag_verb(dim: Dim, nose: bool) -> &'static str {
    match dim {
        Dim::Length if nose => "Drag tip",
        Dim::Length => "Drag face",
        Dim::Diameter | Dim::ForeDiameter | Dim::Span | Dim::Height | Dim::TipChord | Dim::RootChord | Dim::Sweep | Dim::Gap => "Drag edge",
        Dim::Wall | Dim::Thickness | Dim::InnerDiameter => "Drag wall",
        Dim::Position => "Drag part",
        Dim::Cant | Dim::Angle => "Drag angle",
        Dim::Shape => "Drag shape",
        Dim::Vertex(_) => "Drag vertex",
        _ => "Drag handle",
    }
}

/// A label with a backing fill so it reads over the model.
fn label(frame: &mut Frame, content: &str, at: Point, color: Color, back: Color) {
    let w = content.chars().count() as f32 * TEXT_LABEL * GLYPH_EM;
    let size = frame.size();
    let x = at.x.min(size.width - w - 4.0).max(4.0);
    let y = at.y.clamp(TEXT_LABEL, size.height - 4.0);
    frame.fill_rectangle(Point::new(x - 2.0, y - TEXT_LABEL - 1.0), Size::new(w + 4.0, TEXT_LABEL + 3.0), back.scale_alpha(0.92));
    text(frame, content, Point::new(x, y), color, TEXT_LABEL, alignment::Horizontal::Left, alignment::Vertical::Bottom);
}

/// Where the selection's text goes (see [`Overlay::plan`]).
struct Plan {
    /// The selection tag.
    tag: Rectangle,
    /// Labels beside the handles in the picture.
    side: Vec<Labelled>,
    /// Labels beside the handles in the aft-view inset (inside its panel).
    inset: Vec<Labelled>,
    /// Names of the labels that found no free spot.
    hidden: Vec<String>,
}

/// A label placed beside one handle.
struct Labelled {
    /// Index into `Visible::placed`.
    handle: usize,
    text: String,
    rect: Rectangle,
}

impl Plan {
    /// The note for the key-hint spot when handles were left out: some hidden for crowding (`crowded`) or labels dropped
    /// for want of room. `None` when nothing was left out.
    fn note(&self, crowded: bool) -> Option<String> {
        if !crowded && self.hidden.is_empty() {
            return None;
        }
        let mut note = "Zoom in for more handles".to_string();
        if !self.hidden.is_empty() {
            note.push_str(&format!(" · labels hidden: {}", self.hidden.join(", ").to_lowercase()));
        }
        Some(note)
    }
}

/// The size of the backing rectangle of a label (`label` and `label_in` draw the same one).
fn label_size(content: &str) -> Size {
    Size::new(content.chars().count() as f32 * TEXT_LABEL * GLYPH_EM + 4.0, TEXT_LABEL + 3.0)
}

/// A label in the rectangle `rect` (as placed by [`labels::layout`]), on a backing so it reads over the model.
fn label_in(frame: &mut Frame, content: &str, rect: Rectangle, color: Color, back: Color) {
    frame.fill_rectangle(rect.position(), rect.size(), back.scale_alpha(0.92));
    text(frame, content, Point::new(rect.x + 2.0, rect.y + rect.height - 2.0), color, TEXT_LABEL, alignment::Horizontal::Left, alignment::Vertical::Bottom);
}

/// The note about handles left out, framed like the key-hint bar and centred in the free band of the viewport's bottom row
/// (between the CG/CP legend and the scale bar). The hidden-label list is dropped, then the words shortened, until it fits.
fn draw_note(frame: &mut Frame, t: &Tokens, size: Size, note: &str) {
    let (left, right) = (BAR_LEFT, size.width - BAR_RIGHT);
    let short = note.split(" · ").next().unwrap_or(note);
    let fits = |s: &str| s.chars().count() as f32 * (TEXT_LABEL + 1.0) * 0.55 + 24.0 <= right - left;
    let Some(shown) = [note, short, "Zoom in for more"].into_iter().find(|s| fits(s)) else {
        return;
    };
    let w = shown.chars().count() as f32 * (TEXT_LABEL + 1.0) * 0.55 + 24.0;
    let (x, y) = ((left + right - w) / 2.0, size.height - 16.0 - 26.0);
    frame.fill_rectangle(Point::new(x, y), Size::new(w, 26.0), t.bg_200);
    frame.stroke_rectangle(Point::new(x, y), Size::new(w, 26.0), Stroke::default().with_color(t.line_200).with_width(1.0));
    text(frame, shown, Point::new(x + w / 2.0, y + 13.0), t.ink_muted, TEXT_LABEL + 1.0, alignment::Horizontal::Center, alignment::Vertical::Center);
}

fn dashed(frame: &mut Frame, a: Point, b: Point, color: Color, dotted: bool) {
    let segments: &[f32] = if dotted { &[1.5, 3.0] } else { &[4.0, 3.0] };
    frame.stroke(&Path::line(a, b), Stroke { line_dash: LineDash { segments, offset: 0 }, ..Stroke::default().with_color(color).with_width(1.0) });
}

fn handle_shape(frame: &mut Frame, t: &Tokens, p: &Placed, active: bool, hovered: bool) {
    let stroke = Stroke::default().with_color(t.accent).with_width(1.5);
    let fill = if active {
        t.accent
    } else if hovered {
        t.accent.scale_alpha(0.35)
    } else {
        t.bg_000
    };
    if handles::is_circle(p.spec.kind) {
        let path = Path::circle(p.at, CIRCLE / 2.0);
        frame.fill(&path, fill);
        frame.stroke(&path, stroke);
    } else {
        let corner = Point::new(p.at.x - SQUARE / 2.0, p.at.y - SQUARE / 2.0);
        frame.fill_rectangle(corner, Size::new(SQUARE, SQUARE), fill);
        frame.stroke_rectangle(corner, Size::new(SQUARE, SQUARE), stroke);
    }
}

/// An arrowhead at `tip` pointing along unit `dir`.
fn arrow(frame: &mut Frame, from: Point, dir: Vector, len: f32, color: Color) {
    let tip = from + dir * len;
    let side = Vector::new(-dir.y, dir.x);
    frame.stroke(&Path::line(from, tip), Stroke::default().with_color(color).with_width(1.5));
    let head = Path::new(|p| {
        p.move_to(tip);
        p.line_to(tip - dir * 7.0 + side * 3.5);
        p.line_to(tip - dir * 7.0 - side * 3.5);
        p.close();
    });
    frame.fill(&head, color);
}

impl canvas::Program<Message> for Overlay<'_> {
    type State = State;

    fn update(&self, state: &mut State, event: &Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<Message>> {
        let size = bounds.size();
        let edit = |e: Edit| Some(Action::publish(Message::Edit(e)).and_capture());
        match event {
            Event::Keyboard(keyboard::Event::ModifiersChanged(modifiers)) => {
                state.modifiers = *modifiers;
                // Alt and Shift change what shows; re-run a live drag so the snap follows the key.
                if let Some(mut drag) = state.drag.take() {
                    let at = cursor.position().map(|p| Point::new(p.x - bounds.x, p.y - bounds.y));
                    let out = at.and_then(|at| self.drag_to(&mut drag, at, *modifiers));
                    state.drag = Some(drag);
                    return Some(out.map_or_else(Action::request_redraw, |e| Action::publish(Message::Edit(e))));
                }
                Some(Action::request_redraw())
            }
            Event::Keyboard(keyboard::Event::KeyPressed { key, text, .. }) if state.drag.is_some() => {
                use keyboard::key::Named;
                let drag = state.drag.as_mut()?;
                match key {
                    keyboard::Key::Named(Named::Escape) => {
                        state.drag = None;
                        edit(Edit::Cancel)
                    }
                    keyboard::Key::Named(Named::Enter) => {
                        let (id, dim) = (drag.id, drag.placed.spec.dim);
                        let value = self.parse(dim, &drag.typed);
                        state.drag = None;
                        match value {
                            Some(v) => edit(Edit::Commit(id, dim, v)),
                            None => edit(Edit::End),
                        }
                    }
                    keyboard::Key::Named(Named::Backspace) => {
                        drag.typed.pop();
                        Some(Action::request_redraw().and_capture())
                    }
                    _ => {
                        let typed = text.as_ref().map(|t| t.as_str()).unwrap_or("");
                        if !matches!(drag.placed.spec.dim, Dim::Vertex(_))
                            && typed.chars().all(|c| c.is_ascii_digit() || ".-+*/ ".contains(c))
                            && !typed.is_empty()
                        {
                            drag.typed.push_str(typed);
                            return Some(Action::request_redraw().and_capture());
                        }
                        None
                    }
                }
            }
            Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) => {
                let at = cursor.position_in(bounds)?;
                let vis = self.visible(size, state.modifiers);
                if let Some(h) = self.handle_at(at, size, state.modifiers) {
                    let id = self.selected?;
                    let now = Instant::now();
                    let dim = h.spec.dim;
                    if state.last_handle.is_some_and(|(t, i, d)| i == id && d == dim && now.duration_since(t) < DOUBLE) {
                        state.last_handle = None;
                        state.drag = None;
                        return match dim {
                            Dim::Vertex(i) => edit(Edit::RemoveVertex(id, i)),
                            _ => edit(Edit::Focus(dim)),
                        };
                    }
                    state.last_handle = Some((now, id, dim));
                    let base = self.rocket.instance_dim(id, self.instance, dim).unwrap_or(0.0);
                    let base_vertex = if let Dim::Vertex(i) = dim { self.rocket.vertex(id, i).unwrap_or((0.0, 0.0)) } else { (0.0, 0.0) };
                    state.drag = Some(Drag {
                        id,
                        placed: h,
                        base,
                        base_vertex,
                        from: at,
                        targets: self.rocket.snap_targets(id, dim),
                        typed: String::new(),
                        last: None,
                        live: Live { value: base, caught: None, limit: None },
                    });
                    let nose = self.rocket.part(id).is_some_and(|p| p.kind.name() == "Nose cone");
                    return edit(Edit::Begin(id, drag_verb(dim, nose)));
                }
                // A double-click on an edge of the selected freeform fin adds a vertex there.
                if let Some((id, after, (z, h))) = self.edge_at(at, &vis) {
                    let now = Instant::now();
                    if state.last_edge.is_some_and(|(t, i, a)| i == id && a == after && now.duration_since(t) < DOUBLE) {
                        state.last_edge = None;
                        return edit(Edit::AddVertex(id, after, z, h));
                    }
                    state.last_edge = Some((now, id, after));
                }
                if let Some((pod, chip, _)) = self.pod_chip(size) {
                    if chip.contains(at) {
                        return Some(Action::publish(Message::Select(Some(Pick::Radial { id: pod, instance: 0 }))).and_capture());
                    }
                }
                state.press = Some(Press { at, target: pick::pick(self.rocket, &self.camera, at, size) });
                Some(Action::capture())
            }
            Event::Mouse(mouse::Event::CursorMoved { .. }) => {
                let at = cursor.position().map(|p| Point::new(p.x - bounds.x, p.y - bounds.y))?;
                if let Some(mut drag) = state.drag.take() {
                    let out = if drag.typed.is_empty() { self.drag_to(&mut drag, at, state.modifiers) } else { None };
                    state.drag = Some(drag);
                    return Some(out.map_or_else(|| Action::request_redraw().and_capture(), |e| Action::publish(Message::Edit(e)).and_capture()));
                }
                if let Some(press) = &state.press {
                    if at.distance(press.at) <= SLOP {
                        return None;
                    }
                    // Beyond the slop it is a drag: on a part that picks it up; on empty space it is not a click either.
                    let target = press.target;
                    state.press = None;
                    return target.map(|pick| Action::publish(Message::Grab(pick)).and_capture());
                }
                let hover = cursor.position_in(bounds).map_or(Hover::None, |p| self.hover_at(p, size, state.modifiers));
                (hover != self.hover).then(|| Action::publish(Message::Hover(hover)))
            }
            Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) => {
                if state.drag.take().is_some() {
                    // The edit is over: the document records its one undo step.
                    return edit(Edit::End);
                }
                let press = state.press.take()?;
                // Shift-click or Cmd/Ctrl-click on a part adds it to the selection or takes it out; on empty space it clears.
                let message = match press.target {
                    Some(pick) if state.modifiers.shift() || state.modifiers.command() || state.modifiers.control() => Message::Toggle(pick),
                    target => Message::Select(target),
                };
                Some(Action::publish(message).and_capture())
            }
            _ => None,
        }
    }

    fn draw(&self, state: &State, renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let size = bounds.size();
        let mut frame = Frame::new(renderer, size);
        let cursor =
            self.cursor.map(|c| Point::new(c.x - bounds.x, c.y - bounds.y)).filter(|c| (0.0..=size.width).contains(&c.x) && (0.0..=size.height).contains(&c.y));
        let all = state.modifiers.alt() || self.profile;

        // The selection: brackets (solid on the part, or on the clicked copy of a group and dashed on its siblings) with a tag.
        let selection = self.selection();
        let Some(id) = self.selected else {
            self.draw_hover(&mut frame, t, cursor);
            return vec![frame.into_geometry()];
        };
        if !self.also.is_empty() {
            self.draw_multi(&mut frame, t, size);
            self.draw_hover(&mut frame, t, cursor);
            return vec![frame.into_geometry()];
        }
        let tag_text = self.tag_text(id, &selection);
        if let Some(Selection::Radial { radial, .. }) = &selection {
            let (rocket, camera) = self.selected.and_then(|id| self.frame_of(id)).unwrap_or((self.rocket, &self.camera));
            let own = self.instance.min(radial.angles().len() - 1);
            for i in (0..radial.angles().len()).filter(|i| *i != own) {
                if let Some(sibling) = pick::placement(rocket, radial, i) {
                    brackets(&mut frame, pick::radial_bounds(camera, size, &radial.effective(i).kind, sibling), t.accent.scale_alpha(0.7), true);
                }
            }
        }
        // A part of a pod stands in every copy of the pod: dashed brackets on the others, and the tag that selects the set.
        if let Some(view) = self.pod.as_ref().filter(|v| self.rocket.pod_of(id).is_some_and(|p| p.id == v.pod)) {
            for k in 1..view.cameras.len() {
                if let Some(b) = self.bounds_in(&view.sub, &view.cameras[k], id, size) {
                    brackets(&mut frame, b, t.accent.scale_alpha(0.7), true);
                }
            }
            if let Some((_, chip, text)) = self.pod_chip(size) {
                frame.fill_rectangle(chip.position(), chip.size(), t.bg_000);
                frame.stroke_rectangle(chip.position(), chip.size(), Stroke::default().with_color(t.accent).with_width(1.0));
                label(&mut frame, &text, Point::new(chip.x + 6.0, chip.y + 4.0 + TEXT_LABEL), t.accent, t.bg_000);
            }
        }
        let Some(b) = self.bounds_of_selection(size) else {
            return vec![frame.into_geometry()];
        };
        brackets(&mut frame, b, t.accent, false);

        // Where the tag and the handle labels go: placed together so none overprints another or the inset (see `labels`).
        let vis = self.visible(size, state.modifiers);
        let plan = self.plan(id, &tag_text, b, size, &vis, all && state.drag.is_none());
        label_in(&mut frame, &tag_text, plan.tag, t.accent, t.bg_000);
        if let Some(ins) = &vis.inset {
            self.draw_inset(&mut frame, t, id, ins, &vis, &plan.inset);
        }
        let active = state.drag.as_ref().map(|d| d.placed.spec.dim);
        for p in &vis.placed {
            let live = active == Some(p.spec.dim);
            handle_shape(&mut frame, t, p, live, self.hover == Hover::Handle(p.spec.dim));
        }
        for l in &plan.side {
            if let Some((from, to)) = Request::leader(vis.placed[l.handle].at, l.rect) {
                dashed(&mut frame, from, to, t.line_200, true);
            }
            label_in(&mut frame, &l.text, l.rect, t.ink_muted, t.bg_000);
        }
        if let Some(note) = plan.note(vis.crowded).filter(|_| state.drag.is_none()) {
            draw_note(&mut frame, t, size, &note);
        }

        if let Some(drag) = &state.drag {
            self.draw_drag(&mut frame, t, drag, &vis, state.modifiers);
        } else if let (Some(Hover::Handle(dim)), Some(c)) = (Some(self.hover), cursor) {
            if let (Some(v), true) = (self.rocket.instance_dim(id, self.instance, dim), vis.placed.iter().any(|p| p.spec.dim == dim)) {
                let name = self.rocket.dim_label(id, dim);
                tag(&mut frame, c, 0.0, &format!("{name} · {}", self.with_unit(dim, v)), t.ink, t.line_200, t.bg_200);
            }
        } else {
            self.draw_hover(&mut frame, t, cursor);
        }
        vec![frame.into_geometry()]
    }

    // No `mouse_interaction`: a layer that reports a cursor hides the mouse from the layers below it (see
    // `scene::Scene::interaction`), and the model underneath needs the middle button and the wheel.
}

impl Overlay<'_> {
    /// The tag over the selection: what it is, and for a linked group how many copies and whether they follow.
    fn tag_text(&self, id: PartId, selection: &Option<Selection<'_>>) -> String {
        match selection {
            Some(Selection::Axial { kind, .. }) => kind.name().to_uppercase(),
            Some(Selection::Radial { radial, .. }) if radial.kind.is_pod() => {
                if radial.count > 1 {
                    format!("POD ×{} · LINKED", radial.count)
                } else {
                    "POD".to_string()
                }
            }
            Some(Selection::Radial { radial, .. }) => {
                if radial.count > 1 {
                    let tag = if radial.has_overrides() { format!("{} OVR", radial.overridden_copies().len()) } else { "ALL FOLLOW".to_string() };
                    format!("{} ×{} · LINKED · {tag}", radial.kind.name().to_uppercase(), radial.count)
                } else {
                    radial.kind.name().to_uppercase()
                }
            }
            Some(Selection::Internal) => self.rocket.kind_of(id).map(|p| p.kind_name().to_uppercase()).unwrap_or_default(),
            None => String::new(),
        }
    }

    /// The text beside handle `dim` of part `id` (`SWEEP 60.0`, `Ø 76.2`); empty for a freeform fin's vertices.
    fn label_text(&self, id: PartId, dim: Dim) -> String {
        match (dim, self.rocket.instance_dim(id, self.instance, dim)) {
            (Dim::Vertex(_), _) => String::new(),
            (_, Some(v)) => self.handle_label(id, dim, v),
            _ => self.rocket.dim_label(id, dim),
        }
    }

    /// Places the selection tag, the handle labels and the inset's labels so that none overprints another, the inset, the
    /// toolbar, the ViewCube or a handle (see [`labels`]). Labels are most important first: the tag, the primary dimensions,
    /// the others. A label with no free spot is dropped and named in `Plan::hidden`.
    fn plan(&self, id: PartId, tag_text: &str, b: Rectangle, size: Size, vis: &Visible, with_labels: bool) -> Plan {
        let area = Rectangle::new(Point::ORIGIN, size);
        let mut blocked = vec![
            Rectangle::new(Point::ORIGIN, Size::new(TOOLBAR_X.min(size.width), TOOLBAR_Y)),
            Rectangle::new(Point::new(0.0, (size.height - BOTTOM_STRIP).max(0.0)), Size::new(size.width, BOTTOM_STRIP)),
        ];
        if !self.profile {
            blocked.push(Rectangle::new(Point::new(size.width - CUBE_MARGIN - super::CUBE_SIZE, CUBE_MARGIN), Size::new(super::CUBE_SIZE, super::CUBE_SIZE)));
        }
        if let Some(ins) = &vis.inset {
            blocked.push(ins.panel);
        }
        blocked.extend(vis.placed.iter().map(|p| labels::around(p.at, Size::new(HANDLE_BLOCK, HANDLE_BLOCK))));
        if let Some(marks) = &self.balance {
            blocked.extend(markers::balance_label_rects(self.rocket, &self.camera, size, marks, self.units));
        }

        // The tag: above the part's left corner, else below it, else against the right side.
        let tag_size = label_size(tag_text);
        let (g, h) = (BRACKET_GAP, tag_size.height);
        let (left, right) = (b.x - g, b.x + b.width + g - tag_size.width);
        let (above, below) = (b.y - g - 4.0 - h, b.y + b.height + g + 4.0);
        let spots =
            vec![Point::new(left, above), Point::new(right, above), Point::new(left, below), Point::new(right, below), Point::new(left, above - h - 2.0)];
        let mut requests = vec![labels::Request { size: tag_size, priority: 0, spots }];
        let mut wanted = Vec::new();
        if with_labels {
            for (i, p) in vis.placed.iter().enumerate().filter(|(_, p)| !handles::in_inset(&p.spec)) {
                let text = self.label_text(id, p.spec.dim);
                if !text.is_empty() {
                    requests.push(Request::at_handle(p.at, label_size(&text), if p.spec.primary { 1 } else { 2 }));
                    wanted.push((i, text));
                }
            }
        }
        let rects = labels::layout(&requests, &blocked, area);
        let tag = rects[0].unwrap_or_else(|| Rectangle::new(requests[0].spots[0], tag_size));
        let mut side = Vec::new();
        let mut hidden = Vec::new();
        for ((handle, text), rect) in wanted.into_iter().zip(&rects[1..]) {
            match rect {
                Some(rect) => side.push(Labelled { handle, text, rect: *rect }),
                None => hidden.push(self.rocket.dim_label(id, vis.placed[handle].spec.dim).to_uppercase()),
            }
        }

        // The inset's labels are placed inside its panel, round its title and picture.
        let mut inset = Vec::new();
        if let Some(ins) = &vis.inset {
            let title = Rectangle::new(Point::new(ins.panel.x + 8.0, ins.panel.y + 6.0), Size::new(8.0 * TEXT_LABEL * GLYPH_EM, TEXT_LABEL));
            let mut blocked = vec![title];
            let members: Vec<(usize, &Placed)> = vis.placed.iter().enumerate().filter(|(_, p)| handles::in_inset(&p.spec)).collect();
            blocked.extend(members.iter().map(|(_, p)| labels::around(p.at, Size::new(HANDLE_BLOCK, HANDLE_BLOCK))));
            let texts: Vec<String> = members.iter().map(|(_, p)| self.label_text(id, p.spec.dim)).collect();
            let requests: Vec<Request> = members.iter().zip(&texts).map(|((_, p), s)| Request::at_handle(p.at, label_size(s), 1)).collect();
            let panel = Rectangle::new(ins.panel.position() + Vector::new(2.0, 2.0), ins.panel.size() - Size::new(4.0, 4.0));
            for (((handle, p), text), rect) in members.iter().zip(texts).zip(labels::layout(&requests, &blocked, panel)) {
                match rect {
                    Some(rect) => inset.push(Labelled { handle: *handle, text, rect }),
                    None => hidden.push(self.rocket.dim_label(id, p.spec.dim).to_uppercase()),
                }
            }
        }
        Plan { tag, side, inset, hidden }
    }

    /// A multi-selection: accent brackets on every selected part and one tag (`3 PARTS`) over the group. No handles.
    fn draw_multi(&self, frame: &mut Frame, t: &Tokens, size: Size) {
        let parts = self.selected.iter().chain(&self.also);
        let mut union: Option<Rectangle> = None;
        let mut count = 0;
        for b in parts.filter_map(|id| self.bounds_of(*id, size)) {
            brackets(frame, b, t.accent, false);
            union = Some(union.map_or(b, |u| u.union(&b)));
            count += 1;
        }
        let Some(b) = union else { return };
        let caption_y = if b.y - BRACKET_GAP - 4.0 < TOOLBAR_Y && b.x < TOOLBAR_X { b.y + b.height + BRACKET_GAP + 22.0 } else { b.y - BRACKET_GAP - 4.0 };
        label(frame, &format!("{count} PARTS"), Point::new(b.x - BRACKET_GAP, caption_y), t.accent, t.bg_000);
    }

    fn draw_hover(&self, frame: &mut Frame, t: &Tokens, cursor: Option<Point>) {
        let (Hover::Part(id), Some(cursor)) = (self.hover, cursor) else {
            return;
        };
        // The selected part already carries its own tag over the brackets (the kit shows one, not two).
        if self.selected == Some(id) {
            return;
        }
        let label = match (self.rocket.part(id), self.rocket.radial(id)) {
            (Some(part), _) => Some(format!("{} · {}", part.kind.name(), size_label(&part.kind))),
            (_, Some(radial)) => {
                let times = if radial.count > 1 { format!(" ×{}", radial.count) } else { String::new() };
                Some(format!("{}{times} · {}", radial.kind.name(), radial_size_label(&radial.kind)))
            }
            _ => None,
        };
        if let Some(label) = label {
            tag(frame, cursor, 0.0, &label, t.ink, t.line_200, t.bg_200);
        }
    }

    /// The aft-view inset of a radial part: the parent in section, every copy, and the first copy's handles with names.
    fn draw_inset(&self, frame: &mut Frame, t: &Tokens, id: PartId, ins: &Inset, vis: &Visible, names: &[Labelled]) {
        let Some(radial) = self.rocket.radial(id) else {
            return;
        };
        frame.fill_rectangle(ins.panel.position(), ins.panel.size(), t.bg_000);
        frame.stroke_rectangle(ins.panel.position(), ins.panel.size(), Stroke::default().with_color(t.line_200).with_width(1.0));
        text(
            frame,
            "AFT VIEW",
            Point::new(ins.panel.x + 8.0, ins.panel.y + 6.0),
            t.ink_muted,
            TEXT_LABEL,
            alignment::Horizontal::Left,
            alignment::Vertical::Top,
        );
        frame.fill(&Path::circle(ins.centre, ins.base), t.part_body);
        let thickness = radial.kind.thickness().unwrap_or(0.002);
        let width = (thickness as f32 * handles::THICKNESS_SCALE).clamp(2.0, 12.0);
        for (k, a) in radial.angles().into_iter().enumerate() {
            if let bowshock_model::RadialKind::Pod { gap, body } = &radial.kind {
                // A pod copy is a circle of its own radius, `gap` out from the core's surface.
                let (g, r) = (*gap as f32 * ins.scale, body.radius() as f32 * ins.scale);
                let c = ins.centre + handles::arc_dir(a) * (ins.base + g + r);
                frame.fill(&Path::circle(c, r), t.part_fin);
                frame.stroke(&Path::circle(c, r), Stroke::default().with_color(if k == 0 { t.accent } else { t.line_200 }).with_width(1.0));
                continue;
            }
            let (from, to) = (ins.centre + handles::arc_dir(a) * ins.base, ins.centre + handles::arc_dir(a) * (ins.base + ins.reach.max(4.0)));
            let colour = if k == 0 { t.accent } else { t.part_fin };
            if radial.kind.is_fin() {
                frame.stroke(&Path::line(from, to), Stroke::default().with_color(t.part_fin).with_width(width));
                if k == 0 {
                    frame.stroke(&Path::line(from, to), Stroke::default().with_color(colour).with_width(1.0));
                }
            } else {
                frame.fill(&Path::circle(to, 3.0), colour);
            }
        }
        for l in names {
            if let Some((from, to)) = Request::leader(vis.placed[l.handle].at, l.rect) {
                dashed(frame, from, to, t.line_200, true);
            }
            label_in(frame, &l.text, l.rect, t.ink_muted, t.bg_000);
        }
        // The cant lever as a dotted arc from the fin's own direction to its tilt.
        if let Some(cant) = vis.placed.iter().find(|p| p.spec.dim == Dim::Cant) {
            let Axis::Arc { centre } = cant.axis else {
                return;
            };
            let lever = cant.at.distance(centre).max(1.0);
            let a0 = radial.angle;
            let arc = Path::new(|p| {
                for i in 0..=12 {
                    let a = a0 + radial.cant * f64::from(i) / 12.0;
                    let pt = centre + handles::arc_dir(a) * lever;
                    if i == 0 {
                        p.move_to(pt);
                    } else {
                        p.line_to(pt);
                    }
                }
            });
            frame.stroke(&arc, Stroke { line_dash: LineDash { segments: &[1.5, 3.0], offset: 0 }, ..Stroke::default().with_color(t.accent).with_width(1.0) });
        }
    }

    /// The live feedback of a drag: the old-size line, the direction arrow, the snap ticks and guide, and the framed tag.
    fn draw_drag(&self, frame: &mut Frame, t: &Tokens, drag: &Drag, vis: &Visible, modifiers: keyboard::Modifiers) {
        let dim = drag.placed.spec.dim;
        let Some(now) = vis.placed.iter().find(|p| p.spec.dim == dim) else {
            return;
        };
        let size = frame.size();
        let value = self.rocket.instance_dim(drag.id, self.instance, dim).unwrap_or(drag.live.value);
        let moved = value - drag.base;
        let warn = drag.live.limit.is_some();
        let tone = if warn { t.warn } else { t.accent };

        // Where the edge was (dashed `ink-muted`) and which way it is going.
        match drag.placed.axis {
            Axis::Line { dir, per_unit } => {
                let start = drag.placed.at;
                let perp = Vector::new(-dir.y, dir.x);
                let perp = if perp.y < 0.0 || (perp.y == 0.0 && perp.x < 0.0) { perp * -1.0 } else { perp };
                dashed(frame, start + perp * 36.0, start - perp * 36.0, t.ink_muted, false);
                if moved.abs() > 1e-9 {
                    let was = format!("WAS {}", self.num(dim, drag.base));
                    let at = if dir.x.abs() > dir.y.abs() {
                        start + perp * 46.0
                    } else {
                        Point::new(start.x - 40.0 - was.chars().count() as f32 * TEXT_LABEL * GLYPH_EM, start.y)
                    };
                    // While the snap ticks are out their labels own the space under the path (the kit shows no `WAS` then).
                    if !modifiers.shift() {
                        label(frame, &was, at, t.ink_muted, t.bg_000);
                    }
                    let towards = (now.at - start).x * dir.x + (now.at - start).y * dir.y;
                    let heading = if towards >= 0.0 { dir } else { dir * -1.0 };
                    arrow(frame, now.at + heading * 8.0, heading, 18.0, t.accent);
                }
                // Snap ticks with Shift: one per nearby target along the path, the caught one in `ok`.
                if modifiers.shift() {
                    // The caught tick first, then the nearest; a tick closer than 14 px to one already drawn is skipped.
                    let mut order = snap::ticks(&drag.targets, value, per_unit, 8);
                    order.sort_by_key(|&i| drag.live.caught != Some(i));
                    let mut drawn: Vec<f32> = Vec::new();
                    let mut rows: Vec<(usize, f32, f32)> = Vec::new();
                    for i in order {
                        let target = &drag.targets[i];
                        let along = (target.value - value) as f32 * per_unit;
                        if drawn.iter().any(|d| (d - along).abs() < 14.0) {
                            continue;
                        }
                        // Along a horizontal path labels share one row beside their ticks; one that would overprint a neighbour
                        // drops to the next row, on a longer stem.
                        let x0 = now.at.x + dir.x * along + 2.0;
                        let span = (x0, x0 + 6.0 + target.label.chars().count() as f32 * TEXT_LABEL * GLYPH_EM);
                        let row = if dir.x.abs() > dir.y.abs() {
                            let free = (0..).find(|r| !rows.iter().any(|(rr, a, b)| rr == r && span.0 < *b && *a < span.1)).unwrap_or(0);
                            rows.push((free, span.0, span.1));
                            free as f32 + 1.0
                        } else {
                            1.0
                        };
                        drawn.push(along);
                        let at = now.at + dir * along;
                        let caught = drag.live.caught == Some(i);
                        let (colour, width) = if caught { (t.ok, 2.5) } else { (t.ink_muted, 1.0) };
                        let reach = 12.0 + 16.0 * (row - 1.0);
                        frame.stroke(&Path::line(at - perp * 8.0, at + perp * (reach + 2.0)), Stroke::default().with_color(colour).with_width(width));
                        let text_colour = if caught { t.ink } else { t.ink_faint };
                        label(frame, &target.label, at + perp * (reach + 6.0) + dir * 4.0, text_colour, t.bg_000);
                        if caught {
                            if let Some(src) = target.source.and_then(|s| self.anchor(s, size)) {
                                dashed(frame, at, src, t.ok, true);
                            }
                        }
                    }
                }
            }
            Axis::Arc { centre } => {
                dashed(frame, centre, drag.placed.at, t.ink_muted, false);
                if modifiers.shift() {
                    let lever = now.at.distance(centre).max(1.0);
                    for i in snap::ticks(&drag.targets, value, lever, 6) {
                        let target = &drag.targets[i];
                        let a = handles::angle_about(centre, now.at) + (target.value - value);
                        let at = centre + handles::arc_dir(a) * lever;
                        let caught = drag.live.caught == Some(i);
                        let (colour, width) = if caught { (t.ok, 2.5) } else { (t.ink_muted, 1.0) };
                        frame.stroke(&Path::line(at, at + handles::arc_dir(a) * 8.0), Stroke::default().with_color(colour).with_width(width));
                        label(frame, &target.label, at + handles::arc_dir(a) * 12.0, if caught { t.ink } else { t.ink_faint }, t.bg_000);
                    }
                }
            }
            Axis::Plane { .. } => {}
        }

        // The framed tag: title in `ink-muted`, value in the drag colour, the change at the right, then the match or the reason.
        let title = match dim {
            Dim::Vertex(_) => dim.label(),
            _ => self.rocket.dim_label(drag.id, dim),
        };
        let typing = !drag.typed.is_empty();
        let value_text = if typing {
            format!("{}▏", drag.typed)
        } else if let Dim::Vertex(i) = dim {
            let (z, h) = self.rocket.vertex(drag.id, i).unwrap_or((0.0, 0.0));
            format!("{} , {}", self.num(Dim::Length, z), self.num(Dim::Length, h))
        } else {
            self.with_unit(dim, value)
        };
        let delta = if typing || matches!(dim, Dim::Vertex(_)) { String::new() } else { self.delta(dim, moved) };
        let second = match (drag.live.limit, drag.live.caught.and_then(|i| drag.targets.get(i))) {
            (Some(reason), _) => Some((t.warn, reason.to_string())),
            (None, Some(target)) => Some((t.ok, if target.label.starts_with("flush") { target.label.clone() } else { format!("= {}", target.label) })),
            _ => None,
        };
        let chars = |s: &str| s.chars().count() as f32;
        let inner_w = (chars(&value_text) * 13.0 * GLYPH_EM + chars(&delta) * TEXT_LABEL * GLYPH_EM + 18.0)
            .max(chars(&title) * TEXT_LABEL * GLYPH_EM)
            .max(second.as_ref().map_or(0.0, |s| chars(&s.1) * TEXT_LABEL * GLYPH_EM + 14.0))
            .max(96.0);
        let (w, h) = (inner_w + 16.0, if second.is_some() { 62.0 } else { 46.0 });
        let tag_left = modifiers.shift() && matches!(drag.placed.axis, Axis::Line { dir, .. } if dir.x.abs() <= dir.y.abs());
        let anchor = now.at + Vector::new(if tag_left { -(w + 20.0) } else { 20.0 }, -(h + 16.0));
        let origin = Point::new(anchor.x.clamp(6.0, (size.width - w - 6.0).max(6.0)), anchor.y.clamp(48.0, (size.height - h - 6.0).max(48.0)));
        frame.fill_rectangle(origin, Size::new(w, h), t.bg_200);
        frame.stroke_rectangle(origin, Size::new(w, h), Stroke::default().with_color(tone).with_width(1.0));
        let (x, y) = (origin.x + 8.0, origin.y + 8.0);
        text(frame, title, Point::new(x, y), t.ink_muted, TEXT_LABEL - 1.0, alignment::Horizontal::Left, alignment::Vertical::Top);
        text(frame, value_text, Point::new(x, y + 14.0), tone, 13.0, alignment::Horizontal::Left, alignment::Vertical::Top);
        if !delta.is_empty() {
            text(frame, delta, Point::new(origin.x + w - 8.0, y + 17.0), t.ink_muted, TEXT_LABEL, alignment::Horizontal::Right, alignment::Vertical::Top);
        }
        if let Some((colour, line)) = second {
            frame.fill_rectangle(Point::new(x, y + 36.0), Size::new(6.0, 6.0), colour);
            text(
                frame,
                line,
                Point::new(x + 12.0, y + 39.0),
                if colour == t.warn { t.warn } else { t.ink },
                TEXT_LABEL,
                alignment::Horizontal::Left,
                alignment::Vertical::Center,
            );
        }
    }

    /// A point on screen that stands for part `id` (the middle of its top edge), where a snap guide runs to.
    fn anchor(&self, id: PartId, size: Size) -> Option<Point> {
        let (rocket, camera) = self.frame_of(id)?;
        let (a, b) = rocket.part_span(id)?;
        let up = handles::radial_up(camera);
        let r = match rocket.kind_of(id)? {
            PartRef::Axial(p) => p.kind.fore_diameter().max(p.kind.aft_diameter()) / 2.0,
            PartRef::Internal(_) => rocket.resolved(id)?.outer_diameter / 2.0,
            PartRef::Radial(_) => 0.0,
        };
        Some(camera.project(up * r as f32 + Vec3::Z * ((a + b) / 2.0) as f32, size))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{InternalKind, NoseShape, PartKind, RadialKind};
    use std::f32::consts::FRAC_PI_2;

    const SIZE: Size = Size::new(900.0, 600.0);

    fn side() -> Camera {
        Camera { target: Vec3::new(0.0, 0.0, 0.4), yaw: -FRAC_PI_2, pitch: 0.0, roll: 0.0, scale: 800.0, ortho_blend: 1.0 }
    }

    fn rocket() -> (Rocket, PartId, PartId, PartId) {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::Haack(1.0 / 3.0), length: 0.2, diameter: 0.1 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.1 }).unwrap();
        let fin =
            r.attach_radial(tube, 0.4, 0.0, 3, RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.06, sweep: 0.04, thickness: 0.003 }).unwrap();
        (r, tube, fin, 0)
    }

    fn vis(r: &Rocket, id: PartId, all: bool, inside: InsideView) -> Visible {
        let cam = side();
        let b = match r.index_of(id) {
            Some(i) => pick::screen_bounds(&cam, SIZE, r.stations()[i], &r.parts()[i].kind),
            None => {
                let rad = r.radial(id).unwrap();
                pick::radial_bounds(&cam, SIZE, &rad.kind, pick::placement(r, rad, 0).unwrap())
            }
        };
        visible(&Context { rocket: r, camera: &cam, size: SIZE, instance: 0, bounds: b, cube: false }, id, all, inside)
    }

    fn dims(v: &Visible) -> Vec<Dim> {
        v.placed.iter().map(|p| p.spec.dim).collect()
    }

    #[test]
    fn selecting_shows_primary_handles_and_alt_shows_the_rest() {
        let (r, tube, fin, _) = rocket();
        let d = dims(&vis(&r, tube, false, InsideView::Solid));
        assert!(d.contains(&Dim::Length) && d.contains(&Dim::Diameter) && !d.contains(&Dim::Wall));
        let d = dims(&vis(&r, fin, false, InsideView::Solid));
        assert!(d.contains(&Dim::Span) && d.contains(&Dim::RootChord) && !d.contains(&Dim::Thickness));
        let v = vis(&r, fin, true, InsideView::Solid);
        let d = dims(&v);
        assert!(d.contains(&Dim::Thickness) && d.contains(&Dim::Cant) && d.contains(&Dim::Angle));
        assert!(v.inset.is_some());
    }

    #[test]
    fn internal_dimensions_show_in_cutaway() {
        let (mut r, tube, _, _) = rocket();
        let ring = r.attach_internal(tube, 0.1, InternalKind::CenteringRing { outer_diameter: None, inner_diameter: None, thickness: 0.02 }).unwrap();
        assert!(!dims(&vis(&r, tube, true, InsideView::Solid)).contains(&Dim::Wall));
        assert!(dims(&vis(&r, tube, true, InsideView::Cutaway)).contains(&Dim::Wall));
        // A selected internal part always has its primary handles.
        assert!(dims(&vis(&r, tube, false, InsideView::Solid)).iter().all(|d| *d != Dim::InnerDiameter));
        let _ = ring;
    }

    #[test]
    fn a_tiny_part_has_no_handles_and_crowded_ones_hide() {
        let (mut r, tube, _, _) = rocket();
        let cam = side();
        let small = Rectangle::new(Point::new(300.0, 200.0), Size::new(10.0, 10.0));
        let cx = Context { rocket: &r, camera: &cam, size: SIZE, instance: 0, bounds: small, cube: false };
        assert!(visible(&cx, tube, true, InsideView::Cutaway).placed.is_empty());
        // A 20 mm long tube is 16 px wide: its wall handle lands within 16 px of its diameter handle and hides, with the hint.
        r.set_dim(tube, Dim::Length, 0.02).unwrap();
        let v = vis(&r, tube, true, InsideView::Cutaway);
        assert!(v.crowded && !dims(&v).contains(&Dim::Wall) && dims(&v).contains(&Dim::Length));
    }

    #[test]
    fn typed_values_parse_in_the_display_unit() {
        let (r, ..) = rocket();
        let o = Overlay {
            rocket: &r,
            camera: side(),
            selected: None,
            also: Vec::new(),
            instance: 0,
            hover: Hover::None,
            cursor: None,
            snap_step: 0.001,
            inside: InsideView::Solid,
            profile: false,
            units: UnitSystem::default(),
            pod: None,
            balance: None,
        };
        let v = o.parse(Dim::Length, "50").unwrap();
        assert!((v - 0.05).abs() < 1e-12);
        let a = o.parse(Dim::Cant, "15").unwrap();
        assert!((a.to_degrees() - 15.0).abs() < 1e-9);
        assert!(o.parse(Dim::Length, "abc").is_none());
        assert_eq!(o.delta(Dim::Length, 0.058), "+58.00");
    }

    #[test]
    fn dragging_a_handle_edits_one_dimension() {
        let (r, tube, ..) = rocket();
        let o = Overlay {
            rocket: &r,
            camera: side(),
            selected: Some(tube),
            also: Vec::new(),
            instance: 0,
            hover: Hover::None,
            cursor: None,
            snap_step: 0.001,
            inside: InsideView::Solid,
            profile: false,
            units: UnitSystem::default(),
            pod: None,
            balance: None,
        };
        let v = vis(&r, tube, false, InsideView::Solid);
        let h = v.placed.iter().find(|p| p.spec.dim == Dim::Length).copied().unwrap();
        let mut drag = Drag {
            id: tube,
            placed: h,
            base: 0.5,
            base_vertex: (0.0, 0.0),
            from: h.at,
            targets: r.snap_targets(tube, Dim::Length),
            typed: String::new(),
            last: None,
            live: Live::default(),
        };
        // 80 px at 800 px/m is 100 mm: the length goes from 500 to 600.
        let Some(Edit::Dim(id, Dim::Length, value)) = o.drag_to(&mut drag, h.at + Vector::new(80.0, 20.0), keyboard::Modifiers::empty()) else {
            panic!("a length edit");
        };
        assert_eq!(id, tube);
        assert!((value - 0.6).abs() < 1e-9, "{value}");
        // The same position again changes nothing.
        assert!(o.drag_to(&mut drag, h.at + Vector::new(80.0, 20.0), keyboard::Modifiers::empty()).is_none());
        // Beyond the limit the value stops with a reason.
        o.drag_to(&mut drag, h.at + Vector::new(-4000.0, 0.0), keyboard::Modifiers::empty());
        assert_eq!(drag.live.limit, Some("MINIMUM LENGTH"));
        // With Shift a 12 in stock length catches within 8 px.
        let stock = 12.0 * 0.0254;
        let near = h.at + Vector::new(((stock - 0.5) * 800.0) as f32 + 3.0, 0.0);
        o.drag_to(&mut drag, near, keyboard::Modifiers::SHIFT);
        assert!(drag.live.caught.is_some());
        assert!((drag.live.value - stock).abs() < 1e-9);
    }
}

#[cfg(test)]
mod label_tests {
    use super::*;
    use bowshock_model::{NoseShape, PartKind, RadialKind};

    /// The example rocket's airframe (76.2 mm, nose + two tubes) with its three-fin set and a rail button.
    fn example() -> (Rocket, PartId) {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2286, diameter: 0.0762 }).unwrap();
        r.insert(1, PartKind::BodyTube { length: 0.52, diameter: 0.0762 }).unwrap();
        let booster = r.insert(2, PartKind::BodyTube { length: 0.3934, diameter: 0.0762 }).unwrap();
        let fins = r
            .attach_radial(booster, 0.0, 0.0, 3, RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.0762, sweep: 0.06, thickness: 0.00318 })
            .unwrap();
        (r, fins)
    }

    fn overlay(r: &Rocket, fins: PartId, camera: Camera, profile: bool) -> Overlay<'_> {
        Overlay {
            rocket: r,
            camera,
            selected: Some(fins),
            also: Vec::new(),
            instance: 0,
            hover: Hover::None,
            cursor: None,
            snap_step: 0.001,
            inside: InsideView::Solid,
            profile,
            units: UnitSystem::default(),
            pod: None,
            balance: Some(BalanceMarks { cg: 0.7035, cp: 0.8517, margin: 1.94, show: (true, true), staged: false }),
        }
    }

    fn camera(profile: bool, size: Size) -> Camera {
        let mut camera = if profile {
            Camera { yaw: -std::f32::consts::FRAC_PI_2, pitch: 0.0, ortho_blend: 1.0, ..Camera::default() }
        } else {
            super::super::home(Camera::default())
        };
        camera.fit(Vec3::new(-0.04, -0.04, 0.0), Vec3::new(0.04, 0.04, 1.142), size);
        camera
    }

    fn disjoint(rects: &[Rectangle]) {
        for (i, a) in rects.iter().enumerate() {
            for b in &rects[i + 1..] {
                assert!(!a.intersects(b), "{a:?} overlaps {b:?}");
            }
        }
    }

    #[test]
    fn no_two_labels_of_a_selected_fin_set_overlap_and_the_inset_stays_clear() {
        let (r, fins) = example();
        // A viewport at 1280x800 and at 1000x700 (window minus the parts bin and the inspector).
        for size in [Size::new(704.0, 580.0), Size::new(404.0, 480.0)] {
            // 2D side (every handle), 3D home with Alt (every handle), 3D home on plain select (primary handles).
            for (profile, all) in [(true, true), (false, true), (false, false)] {
                let o = overlay(&r, fins, camera(profile, size), profile);
                let modifiers = if all && !profile { keyboard::Modifiers::ALT } else { keyboard::Modifiers::empty() };
                let selection = o.selection();
                let tag = o.tag_text(fins, &selection);
                let b = o.bounds_of_selection(size).expect("the fin set is on screen");
                let vis = o.visible(size, modifiers);
                let plan = o.plan(fins, &tag, b, size, &vis, all);
                let case = format!("{size:?} profile={profile} all={all}");

                let mut rects = vec![plan.tag];
                rects.extend(plan.side.iter().map(|l| l.rect));
                disjoint(&rects);
                // The CG / CP value labels and the margin label (the CP label sits over the fins here) are kept clear too.
                let marks = o.balance.expect("the fixture has balance markers");
                for m in markers::balance_label_rects(&r, &o.camera, size, &marks, o.units) {
                    for r in &rects {
                        assert!(!r.intersects(&m), "{case}: {r:?} over a marker label {m:?}");
                    }
                }
                if all {
                    assert!(!plan.side.is_empty() || !plan.hidden.is_empty(), "{case}: handle labels were planned");
                }
                if let Some(ins) = &vis.inset {
                    for r in &rects {
                        assert!(!r.intersects(&ins.panel), "{case}: {r:?} over the inset {:?}", ins.panel);
                    }
                    // The inset keeps off the fin set itself.
                    assert!(!ins.panel.intersects(&b), "{case}: the inset covers the part");
                    let inner: Vec<Rectangle> = plan.inset.iter().map(|l| l.rect).collect();
                    disjoint(&inner);
                    for l in &inner {
                        assert!(l.x >= ins.panel.x && l.x + l.width <= ins.panel.x + ins.panel.width, "{case}: inset label outside its panel");
                    }
                }
            }
        }
    }
}
