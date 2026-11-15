//! Drags shared between the parts bin, the viewport and (later) the lanes: a part carried by the cursor that either lands on
//! a target or goes back where it was.
//!
//! The state is [`Drags`] in the app context. A drag starts with [`start_axial`] / [`start_radial`] (from a bin row) or
//! [`grab`] (a part of the rocket picked up); the surface that shows the targets reports them by setting `snap` / `target` on
//! the drag; it ends with [`finish_axial`] / [`finish_radial`] (`drop = true` lands it on its target if it has one) or
//! [`cancel`]. All of it is one undo step through the document's drag support: a cancelled or target-less drag restores the
//! rocket exactly as it was before the drag began.
//!
//! To add another kind of drag (an internal part in the lanes, say) add a field to [`Drags`], a start/finish pair here that
//! wraps `ctx.doc.begin_drag` / `end_drag`, and include it in [`Drags::active`] and [`cancel`].

use bowshock_model::{InternalKind, InternalPart, Part, PartId, PartKind, RadialKind, Rocket};

use super::{Ctx, Selection};
use crate::app::ui;
use crate::viewport::assembly::{Drag, Origin};
use crate::viewport::pick::Pick;
use crate::viewport::surface::RadialDrag;

/// The drags in progress (at most one) and the placement settings the radial ones use.
#[derive(Debug, Clone)]
pub struct Drags {
    /// An axial part (nose, tube, transition) being carried.
    pub axial: Option<Drag>,
    /// A radial part (fins, lug, rail button) being carried.
    pub radial: Option<RadialDrag>,
    /// An internal part (ring, chute, ballast, ...) being carried.
    pub internal: Option<InternalDrag>,
    /// Copies a new radial part gets (the parts bin's symmetry footer).
    pub symmetry: u8,
    /// Angle snap for radial parts, degrees (0 for none).
    pub angle_snap: u8,
}

impl Default for Drags {
    fn default() -> Self {
        Drags { axial: None, radial: None, internal: None, symmetry: 3, angle_snap: 0 }
    }
}

impl Drags {
    /// Whether any drag is in progress.
    pub fn active(&self) -> bool {
        self.axial.is_some() || self.radial.is_some() || self.internal.is_some()
    }
}

/// Puts `kind` into `rocket` at attach node `index` (matched to its neighbours). With `origin` the part comes back exactly as
/// it was (material, finish, overrides), only with the matched geometry. Returns the matched kind when it was placed.
fn insert_part(rocket: &mut Rocket, index: usize, kind: &PartKind, origin: Option<&Part>) -> Option<PartKind> {
    let matched = rocket.matched(index, kind);
    let placed = match origin {
        Some(part) => rocket.restore(index, Part { kind: matched.clone(), ..part.clone() }).is_ok(),
        None => rocket.insert(index, kind.clone()).is_ok(),
    };
    placed.then_some(matched)
}

/// The status message for a part whose diameter was changed to fit its neighbours, if it was.
fn matched_notice(asked: &PartKind, matched: &PartKind) -> Option<String> {
    let differs = |a: f64, b: f64| (a - b).abs() > 1e-9;
    if !differs(matched.fore_diameter(), asked.fore_diameter()) && !differs(matched.aft_diameter(), asked.aft_diameter()) {
        return None;
    }
    let diameter =
        if differs(matched.fore_diameter(), asked.fore_diameter()) && matched.fore_diameter() > 0.0 { matched.fore_diameter() } else { matched.aft_diameter() };
    Some(format!("Diameter matched to {} mm", ui::mm(diameter)))
}

/// Inserts `kind` at attach node `index` as one undo step (a click on a bin row). A new part also frames the rocket.
pub fn place(ctx: &mut Ctx, index: usize, kind: PartKind) -> bool {
    let name = kind.name();
    let placed = ctx.doc.edit(&format!("Add {}", name.to_lowercase()), |r| insert_part(r, index, &kind, None));
    if let Some(matched) = &placed {
        if let Some(message) = matched_notice(&kind, matched) {
            ctx.notify(message);
        }
        ctx.camera_fit = true;
    }
    placed.is_some()
}

/// Starts carrying a new axial part from bin row `row`.
pub fn start_axial(ctx: &mut Ctx, row: usize, name: &'static str, kind: PartKind) {
    ctx.doc.begin_drag(&format!("Add {}", name.to_lowercase()));
    ctx.drag.axial = Some(Drag { row: Some(row), origin: None, name, kind, snap: None });
}

/// Starts carrying a new radial part from bin row `row`.
pub fn start_radial(ctx: &mut Ctx, row: usize, kind: RadialKind) {
    ctx.doc.begin_drag(&format!("Add {}", kind.name().to_lowercase()));
    let count = ctx.drag.symmetry;
    ctx.drag.radial = Some(RadialDrag { row: Some(row), kind, count, origin: None, target: None });
}

/// Picks up a part of the rocket: an axial part comes out of the rocket with the radial and internal parts it carries; a
/// radial part comes out whole. Nothing happens while another drag is in progress.
pub fn grab(ctx: &mut Ctx, pick: Pick) {
    if ctx.drag.active() {
        return;
    }
    match pick {
        Pick::Axial(id) => {
            ctx.doc.begin_drag("Move part");
            let taken = ctx.doc.drag_update(|r| {
                r.index_of(id)?;
                let radials = r.take_radials(id);
                let internals = r.take_internals(id);
                let part = r.remove(id)?;
                Some(Origin { part, radials, internals })
            });
            match taken {
                Some(origin) => {
                    ctx.selection = Selection::Part(id);
                    ctx.drag.axial = Some(Drag { row: None, name: origin.part.kind.name(), kind: origin.part.kind.clone(), snap: None, origin: Some(origin) });
                }
                None => {
                    ctx.doc.end_drag(false);
                }
            }
        }
        Pick::Radial { id, .. } => {
            ctx.doc.begin_drag("Move part");
            match ctx.doc.drag_update(|r| r.remove_radial(id)) {
                Some(radial) => {
                    ctx.selection = Selection::Part(id);
                    ctx.drag.radial = Some(RadialDrag { row: None, kind: radial.kind.clone(), count: radial.count, origin: Some(radial), target: None });
                }
                None => {
                    ctx.doc.end_drag(false);
                }
            }
        }
    }
}

/// Puts back the motors that removing and re-adding a motor mount dropped from the configurations: every slot of `from` whose
/// mount exists again in `rocket` but has lost its slot.
fn restore_motors(rocket: &mut Rocket, from: &Rocket) {
    for config in from.configurations() {
        for slot in &config.motors {
            let present = rocket.configuration(config.id).is_some_and(|c| c.slot(slot.mount).is_some());
            if !present && rocket.internal(slot.mount).is_some() {
                let _ = rocket.set_motor(config.id, slot.mount, slot.motor.clone());
                let _ = rocket.set_motor_delay(config.id, slot.mount, slot.delay);
                let _ = rocket.set_ignition(config.id, slot.mount, slot.ignition);
            }
        }
    }
}

/// Ends the axial drag: lands it on its snap node if `drop`, else abandons it. A part picked up from the rocket that is not
/// landed goes back where it was, as it was.
pub fn finish_axial(ctx: &mut Ctx, drop: bool) {
    let Some(drag) = ctx.drag.axial.take() else {
        return;
    };
    let base = ctx.doc.drag_base().cloned();
    let placed = match drag.snap {
        Some(node) if drop => ctx.doc.drag_update(|r| {
            let matched = insert_part(r, node, &drag.kind, drag.origin.as_ref().map(|o| &o.part));
            if let (Some(_), Some(origin)) = (&matched, &drag.origin) {
                // The parts it carried go back on it, wherever it landed.
                r.restore_radials(origin.radials.clone());
                r.restore_internals(origin.internals.clone());
                if let Some(base) = &base {
                    restore_motors(r, base);
                }
            }
            matched
        }),
        _ => None,
    };
    if let Some(matched) = &placed {
        if let Some(message) = matched_notice(&drag.kind, matched) {
            ctx.notify(message);
        }
        // A part from the bin frames the rocket; one that was moved leaves the camera where the user put it.
        ctx.camera_fit |= drag.origin.is_none();
    }
    ctx.doc.end_drag(placed.is_some());
}

/// Ends the radial drag: lands it on its surface target if `drop`, else abandons it. A group picked up from the rocket that is
/// not landed goes back as it was.
pub fn finish_radial(ctx: &mut Ctx, drop: bool) {
    let Some(drag) = ctx.drag.radial.take() else {
        return;
    };
    let placed = match (drop, drag.target) {
        (true, Some(t)) => ctx.doc.drag_update(|r| match &drag.origin {
            Some(origin) => {
                let moved = bowshock_model::RadialPart {
                    kind: drag.kind.clone(),
                    parent: t.parent,
                    offset: t.offset,
                    angle: t.angle,
                    count: drag.count,
                    ..origin.clone()
                };
                r.restore_radial(moved).is_ok()
            }
            None => r.attach_radial(t.parent, t.offset, t.angle, drag.count, drag.kind.clone()).is_ok(),
        }),
        _ => false,
    };
    ctx.doc.end_drag(placed);
}

/// Where a carried internal part would land: in axial part `parent`, `offset` metres behind its fore face.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InternalTarget {
    /// The axial part it goes in.
    pub parent: PartId,
    /// Distance from the parent's fore face to the part's fore edge.
    pub offset: f64,
}

/// An internal part (ring, chute, ballast, ...) being carried. The surface that shows the targets (viewport, lanes) sets
/// `target` and ends the drag with [`finish_internal`].
#[derive(Debug, Clone)]
pub struct InternalDrag {
    /// The bin row it came from, if it came from there.
    pub row: Option<usize>,
    /// The bin's name for it, for the cursor tag of the surface that shows the targets.
    #[allow(dead_code)] // Read by the viewport / lanes overlays.
    pub name: &'static str,
    /// Its geometry.
    pub kind: InternalKind,
    /// The part as it was (material, overrides), when picked up from the rocket.
    pub origin: Option<InternalPart>,
    /// Where it would land now.
    pub target: Option<InternalTarget>,
}

/// Starts carrying a new internal part from bin row `row`.
pub fn start_internal(ctx: &mut Ctx, row: Option<usize>, name: &'static str, kind: InternalKind) {
    ctx.doc.begin_drag(&format!("Add {}", name.to_lowercase()));
    ctx.drag.internal = Some(InternalDrag { row, name, kind, origin: None, target: None });
}

#[allow(dead_code)] // Called by the viewport / lanes when an internal part is picked up.
/// Picks up internal part `id` of the rocket. Nothing happens while another drag is in progress.
pub fn grab_internal(ctx: &mut Ctx, id: PartId) -> bool {
    if ctx.drag.active() {
        return false;
    }
    ctx.doc.begin_drag("Move part");
    match ctx.doc.drag_update(|r| r.remove_internal(id)) {
        Some(part) => {
            ctx.selection = Selection::Part(id);
            ctx.drag.internal = Some(InternalDrag { row: None, name: part.kind.name(), kind: part.kind.clone(), origin: Some(part), target: None });
            true
        }
        None => {
            ctx.doc.end_drag(false);
            false
        }
    }
}

/// Ends the internal drag: lands it on its target if `drop`, else abandons it (a part picked up from the rocket goes back as it
/// was, motors included).
pub fn finish_internal(ctx: &mut Ctx, drop: bool) {
    let Some(drag) = ctx.drag.internal.take() else {
        return;
    };
    let base = ctx.doc.drag_base().cloned();
    let placed = match drag.target {
        Some(t) if drop => ctx.doc.drag_update(|r| {
            let ok = match &drag.origin {
                Some(part) => r.restore_internal(InternalPart { parent: t.parent, offset: t.offset, ..part.clone() }).is_ok(),
                None => r.attach_internal(t.parent, t.offset, drag.kind.clone()).is_ok(),
            };
            if let (true, Some(base)) = (ok, &base) {
                restore_motors(r, base);
            }
            ok
        }),
        _ => false,
    };
    if placed && drag.origin.is_none() {
        // Select what was just added.
        if let Some(id) = ctx.doc.rocket().internals().last().map(|p| p.id) {
            ctx.selection = Selection::Part(id);
        }
    }
    ctx.doc.end_drag(placed);
}

/// Abandons whichever drag is in progress.
pub fn cancel(ctx: &mut Ctx) {
    finish_axial(ctx, false);
    finish_radial(ctx, false);
    finish_internal(ctx, false);
}

/// Deletes the selected part (an axial part takes its radial and internal parts with it; a radial part is a whole group) as
/// one undo step.
pub fn delete_selection(ctx: &mut Ctx) {
    if ctx.drag.active() {
        return;
    }
    let Selection::Part(id) = ctx.selection else {
        return;
    };
    ctx.doc.edit("Delete part", |r| remove_any(r, id));
    ctx.selection = Selection::None;
}

/// Removes the part `id` of whatever class it is.
fn remove_any(rocket: &mut Rocket, id: PartId) {
    if rocket.remove(id).is_none() && rocket.remove_radial(id).is_none() {
        rocket.remove_internal(id);
    }
}
