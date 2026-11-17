//! The manipulation overlay: direct handling of the parts already in the viewport (design kit: DirectEdit, Snapping). It
//! hover-picks a part under the cursor, selects on click, picks a part up when it is pressed and dragged, and drags the
//! handles of the selected part: one handle, one dimension (see [`handles`](super::handles)). It draws the selection (corner
//! brackets, a mono tag, handles, labels, the aft-view inset), the live tag of a drag with its old-size line and arrow, and
//! with Shift the snap ticks. Everything it decides is reported as an [`Edit`] or other viewport message; the app owns the
//! rocket, and every handle edit is one undo step through [`apply`].
//!
//! A radial part is a group of linked copies: clicking any copy selects the group, with solid brackets on the clicked copy
//! and dashed ones on its siblings.

mod draw;
mod paint;
mod program;
mod query;
#[cfg(test)]
mod tests;

use super::carry::{Carry, Landing};
use super::handles::{self, Context, Inset, Placed};
use super::markers::BalanceMarks;
use super::pick::Pick;
use super::pods::PodView;
use super::radial::Placement;
use super::{Camera, InsideView};
use crate::app::{self, Ctx};
use crate::units::UnitSystem;
use bowshock_model::{Anchor, Dim, HandleView, PartId, PartRef, Rocket, Target};
use iced::{keyboard, mouse, Point, Task};
use std::time::{Duration, Instant};

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
    /// An internal part was pressed and dragged: the document starts collecting one undo step for carrying it.
    Carry(PartId),
    /// The carried internal part (first) is now in body part `parent` (second), its fore edge `offset` behind that part's fore
    /// face.
    Place(PartId, PartId, f64),
}

impl Edit {
    /// The key-hint bar to show for this edit: the drag's keys on `Begin`, none (empty) when it ends.
    pub fn hint(&self) -> Option<Vec<(&'static str, &'static str)>> {
        match self {
            Edit::Begin(_, verb) => Some(vec![("", *verb), ("Shift", "snap"), ("Ctrl", "5 mm"), ("0-9", "type exact"), ("Esc", "cancel")]),
            Edit::Carry(_) => Some(vec![("", "Slide along the rocket · carry into the next body part"), ("Ctrl", "5 mm"), ("Esc", "cancel")]),
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
            // A carried internal part is placed (or, over the parts bin, deleted) by the same decision as every other carry.
            if ctx.drag.lifted.is_some() {
                crate::app::drag::end_lifted(ctx);
            } else {
                ctx.doc.end_drag(true);
            }
        }
        Edit::Cancel => {
            ctx.drag.lifted = None;
            ctx.doc.end_drag(false);
        }
        Edit::AddVertex(id, after, z, h) => {
            ctx.doc.edit("Add vertex", |r| r.add_vertex(id, after, z, h));
        }
        Edit::RemoveVertex(id, i) => {
            ctx.doc.edit("Remove vertex", |r| r.remove_vertex(id, i));
        }
        Edit::Carry(id) => {
            let name = ctx.doc.rocket().kind_of(id).map(|p| p.name()).unwrap_or_default();
            ctx.doc.begin_drag(&format!("Move {}", name.to_lowercase()));
            ctx.selection = app::Selection::Part(id);
            ctx.drag.lifted = Some(id);
            ctx.symmetry.instance = None;
        }
        Edit::Place(id, parent, offset) => {
            // Only the part in hand moves (a stale overlay, after the carry was put back, sends nothing that counts).
            if ctx.drag.lifted == Some(id) {
                ctx.doc.drag_update(|r| {
                    let reparented = r.internal(id).is_some_and(|p| p.parent != parent);
                    if r.move_internal(id, parent, offset).is_ok() && reparented && r.internal(id).is_some_and(|p| p.kind.is_motor_mount()) {
                        // A motor mount is placed from the bottom of its body part, wherever it is moved to.
                        r.set_anchor(id, Anchor::Bottom);
                    }
                });
            }
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
    /// Modifiers a drive script holds before the window reports any.
    pub drive_modifiers: keyboard::Modifiers,
    /// The pod holding the selected part, seen through its own frame, when the selection is inside a pod: handles and brackets
    /// of its parts are worked out there (`PodView::of_part`).
    pub pod: Option<PodView>,
    /// The CG / CP markers' values, so labels keep off the marker labels.
    pub balance: Option<BalanceMarks>,
    /// How a part in hand is put down, and which internal part is being carried along the rocket (see [`Hand`]).
    pub hand: Hand,
}

/// What the overlay needs to know of a part in hand: an internal part carried along the rocket stays in it, so the overlay carries
/// it, and when parts are placed by clicking the carry survives the release and ends with the next press.
#[derive(Debug, Clone, Copy, Default)]
pub struct Hand {
    /// Parts are placed by clicking (Preferences > Editor).
    pub click: bool,
    /// The internal part the app has in hand (`ctx.drag.lifted`): a carry the overlay thinks it has that the app does not (put back
    /// by Esc, or by the bin) is forgotten.
    pub lifted: Option<PartId>,
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

/// An internal part being carried along the rocket: it is moved in place, live, one undo step.
#[derive(Debug)]
struct Carried {
    carry: Carry,
    /// Where the cursor would put it now (for the tag).
    landing: Option<Landing>,
    /// The last place sent to the document `(part, parent, offset)`, so a move that changes nothing sends nothing.
    placed: Option<(PartId, PartId, f64)>,
    /// The app has been seen to have this part in hand (`Hand::lifted`): from then on a carry the app has dropped is forgotten.
    confirmed: bool,
}

#[derive(Debug, Default)]
pub struct State {
    press: Option<Press>,
    /// The right button's press, until it is released in place (a context menu) or moves (an orbit).
    right: Option<Press>,
    drag: Option<Drag>,
    carry: Option<Carried>,
    /// The held modifiers, once the window has reported them (until then the drive script's).
    modifiers: Option<keyboard::Modifiers>,
    /// The last handle press (for double-click) and the last press on a freeform edge.
    last_handle: Option<(Instant, PartId, Dim)>,
    last_edge: Option<(Instant, PartId, usize)>,
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
