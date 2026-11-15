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

use bowshock_model::{Anchor, InternalKind, InternalPart, Part, PartId, PartKind, RadialKind, Rocket};

use super::{Ctx, Selection};
use crate::app::ui;
use crate::viewport::assembly::{body_of, Drag, Origin};
use crate::viewport::pick::{Pick, SurfaceTarget};
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

/// [`insert_part`] into the stack of pod `pod`.
fn insert_in_pod(rocket: &mut Rocket, pod: PartId, index: usize, kind: &PartKind, origin: Option<&Part>) -> Option<PartKind> {
    let matched = rocket.matched_in_pod(pod, index, kind);
    let placed = match origin {
        Some(part) => rocket.restore_in_pod(pod, index, Part { kind: matched.clone(), ..part.clone() }).is_ok(),
        None => rocket.insert_in_pod(pod, index, kind.clone()).is_ok(),
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

/// [`place`] into the stack of pod `pod` (a click on a bin row with a part of that pod selected).
pub fn place_in_pod(ctx: &mut Ctx, pod: PartId, index: usize, kind: PartKind) -> bool {
    let name = kind.name();
    let placed = ctx.doc.edit(&format!("Add {}", name.to_lowercase()), |r| insert_in_pod(r, pod, index, &kind, None));
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
    let count = ctx.drag.symmetry;
    ctx.drag.axial = Some(Drag { row: Some(row), origin: None, name, kind, snap: None, pod_snap: None, surface: None, count });
}

/// Starts carrying a new radial part from bin row `row`.
pub fn start_radial(ctx: &mut Ctx, row: usize, kind: RadialKind) {
    ctx.doc.begin_drag(&format!("Add {}", kind.name().to_lowercase()));
    // A pod from the bin is sized for this rocket.
    let kind = if kind.is_pod() { ctx.doc.rocket().default_pod() } else { kind };
    let count = ctx.drag.symmetry;
    ctx.drag.radial = Some(RadialDrag { row: Some(row), kind, count, origin: None, target: None, detached: false });
}

/// Picks up a part of the rocket: an axial part comes out of the rocket with the radial and internal parts it carries; a
/// radial part comes out whole. Nothing happens while another drag is in progress.
pub fn grab(ctx: &mut Ctx, pick: Pick) {
    if ctx.drag.active() {
        return;
    }
    // A single-part pod is that part with copies: picking its part up moves the part (it leaves the pod, which goes with it).
    let single = match pick {
        Pick::Axial(id) => ctx.doc.rocket().pod_of(id).filter(|p| p.kind.pod_body().is_some_and(|b| b.parts().len() == 1)).map(|p| (id, p.id, p.count)),
        _ => None,
    };
    if let Some((id, pod, count)) = single {
        ctx.doc.begin_drag("Move part");
        let taken = ctx.doc.drag_update(|r| {
            let radial = r.remove_radial(pod)?;
            let body = radial.kind.pod_body()?;
            Some(Origin { part: body.parts().first()?.clone(), radials: body.radials().to_vec(), internals: body.internals().to_vec() })
        });
        match taken {
            Some(origin) => {
                ctx.selection = Selection::Part(id);
                let (name, kind) = (origin.part.kind.name(), origin.part.kind.clone());
                ctx.drag.axial = Some(Drag { row: None, name, kind, snap: None, pod_snap: None, surface: None, count, origin: Some(origin) });
            }
            None => {
                ctx.doc.end_drag(false);
            }
        }
        return;
    }
    // A part of a bigger pod is carried with its pod: the body is the thing that moves.
    let pick = match pick {
        Pick::Axial(id) => match ctx.doc.rocket().pod_of(id) {
            Some(pod) => Pick::Radial { id: pod.id, instance: 0 },
            None => pick,
        },
        other => other,
    };
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
                    ctx.drag.axial = Some(Drag {
                        row: None,
                        name: origin.part.kind.name(),
                        kind: origin.part.kind.clone(),
                        snap: None,
                        pod_snap: None,
                        surface: None,
                        count: ctx.drag.symmetry,
                        origin: Some(origin),
                    });
                }
                None => {
                    ctx.doc.end_drag(false);
                }
            }
        }
        // An internal part is carried where it stands by the viewport's manipulation overlay (`Edit::Carry`), never picked up.
        Pick::Internal(_) => {}
        Pick::Radial { id, .. } => {
            ctx.doc.begin_drag("Move part");
            match ctx.doc.drag_update(|r| r.remove_radial(id)) {
                Some(radial) => {
                    ctx.selection = Selection::Part(id);
                    ctx.drag.radial =
                        Some(RadialDrag { row: None, kind: radial.kind.clone(), count: radial.count, origin: Some(radial), target: None, detached: false });
                }
                None => {
                    ctx.doc.end_drag(false);
                }
            }
        }
    }
}

/// Puts back what removing and re-adding a part dropped from the configurations: every motor slot of `from` whose mount exists
/// again in `rocket` but has lost its slot, and every recovery deployment of a device that exists again.
fn restore_motors(rocket: &mut Rocket, from: &Rocket) {
    for config in from.configurations() {
        for slot in &config.motors {
            let present = rocket.configuration(config.id).is_some_and(|c| c.slot(slot.mount).is_some());
            if !present && rocket.internal(slot.mount).is_some() {
                let _ = rocket.set_motor(config.id, slot.mount, slot.motor.clone());
                let _ = rocket.set_motor_delay(config.id, slot.mount, slot.delay);
            }
        }
        for timing in &config.recovery {
            if rocket.configuration(config.id).is_some_and(|c| c.deployment(timing.part).is_none()) {
                let _ = rocket.set_deployment(config.id, timing.part, timing.deploy);
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
    // Over a tube's side the part lands there as a pod (the part, with what stands on and in it, ids kept if it came from the
    // rocket).
    if let (true, None, None, Some(t)) = (drop, drag.snap, drag.pod_snap, drag.surface) {
        let landed = ctx.doc.drag_update(|r| land_on_surface(r, &drag, base.as_ref(), t));
        if let Some(part) = landed {
            ctx.selection = Selection::Part(part);
            ctx.camera_fit |= drag.origin.is_none();
        }
        ctx.doc.end_drag(landed.is_some());
        return;
    }
    let placed = match (drag.snap, drag.pod_snap) {
        (Some(node), _) if drop => ctx.doc.drag_update(|r| land_on_node(r, &drag, node, base.as_ref())),
        (None, Some((pod, node))) if drop => ctx.doc.drag_update(|r| land_in_pod(r, &drag, pod, node)),
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
/// not landed goes back as it was, except one pulled free of the rocket (`detached`) and released over empty space: that is
/// removed (one undo step, with a notice).
pub fn finish_radial(ctx: &mut Ctx, drop: bool) {
    let Some(drag) = ctx.drag.radial.take() else {
        return;
    };
    let placed = match (drop, drag.target) {
        (true, Some(t)) => ctx.doc.drag_update(|r| land_radial(r, &drag, t)),
        _ => false,
    };
    if drop && !placed && drag.target.is_none() && drag.detached && drag.origin.is_some() {
        let name = drag.kind.name();
        ctx.doc.rename_drag(&format!("Remove {}", name.to_lowercase()));
        if ctx.doc.end_drag(true) {
            ctx.select_one(None);
            ctx.notify(format!("Removed {name} · Undo brings it back"));
        }
        return;
    }
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
        Some(t) if drop => ctx.doc.drag_update(|r| land_internal(r, &drag, t, base.as_ref())),
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

/// Lands the carried axial part over a tube's side as a pod (the part, with what stands on and in it, ids kept if it came from
/// the rocket); the id of the part inside the pod. `base` is the rocket as it was when the drag began.
fn land_on_surface(r: &mut Rocket, drag: &Drag, base: Option<&Rocket>, t: SurfaceTarget) -> Option<PartId> {
    let body = body_of(drag);
    let pod = match &drag.origin {
        Some(_) => r.attach_pod_keeping_ids(t.parent, t.offset, t.angle, drag.count, body).ok()?,
        None => r.attach_radial(t.parent, t.offset, t.angle, drag.count, RadialKind::Pod { gap: 0.0, body }).ok()?,
    };
    if let Some(base) = base {
        restore_motors(r, base);
    }
    r.radial(pod)?.kind.pod_body()?.parts().first().map(|p| p.id)
}

/// Lands the carried axial part on attach node `node`, with what it carried; the matched kind.
fn land_on_node(r: &mut Rocket, drag: &Drag, node: usize, base: Option<&Rocket>) -> Option<PartKind> {
    let matched = insert_part(r, node, &drag.kind, drag.origin.as_ref().map(|o| &o.part));
    if let (Some(_), Some(origin)) = (&matched, &drag.origin) {
        // The parts it carried go back on it, wherever it landed.
        r.restore_radials(origin.radials.clone());
        r.restore_internals(origin.internals.clone());
        if let Some(base) = base {
            restore_motors(r, base);
        }
    }
    matched
}

/// Lands the carried axial part on node `node` of pod `pod`'s stack; the matched kind.
fn land_in_pod(r: &mut Rocket, drag: &Drag, pod: PartId, node: usize) -> Option<PartKind> {
    let matched = insert_in_pod(r, pod, node, &drag.kind, drag.origin.as_ref().map(|o| &o.part));
    if let (Some(_), Some(origin)) = (&matched, &drag.origin) {
        r.restore_radials(origin.radials.clone());
        r.restore_internals(origin.internals.clone());
    }
    matched
}

/// Lands the carried radial part on `t`.
fn land_radial(r: &mut Rocket, drag: &RadialDrag, t: SurfaceTarget) -> bool {
    match &drag.origin {
        Some(origin) => {
            let moved =
                bowshock_model::RadialPart { kind: drag.kind.clone(), parent: t.parent, offset: t.offset, angle: t.angle, count: drag.count, ..origin.clone() };
            r.restore_radial(moved).is_ok()
        }
        None => r.attach_radial(t.parent, t.offset, t.angle, drag.count, drag.kind.clone()).is_ok(),
    }
}

/// Lands the carried internal part on `t`, its motors back with it.
fn land_internal(r: &mut Rocket, drag: &InternalDrag, t: InternalTarget, base: Option<&Rocket>) -> bool {
    let ok = match &drag.origin {
        Some(part) => r.restore_internal(InternalPart { parent: t.parent, offset: t.offset, ..part.clone() }).is_ok(),
        None => match r.attach_internal(t.parent, t.offset, drag.kind.clone()) {
            Ok(id) => {
                // A new part is placed from the end of its body part it is nearer to (a motor mount always from the bottom).
                let length = r.part(t.parent).map_or(0.0, |p| p.kind.length());
                let aft = drag.kind.is_motor_mount() || t.offset + drag.kind.extent() / 2.0 > length / 2.0;
                r.set_anchor(id, if aft { Anchor::Bottom } else { Anchor::Top });
                true
            }
            Err(_) => false,
        },
    };
    if let (true, Some(base)) = (ok, base) {
        restore_motors(r, base);
    }
    ok
}

/// Where the carried part would land now: compared between frames so the preview is only worked out when it moved.
#[derive(Debug, Clone, PartialEq)]
pub enum Landing {
    /// An axial part: on a node of the stack, in a pod's stack, or on a tube's side.
    Axial { snap: Option<usize>, pod_snap: Option<(PartId, usize)>, surface: Option<SurfaceTarget>, count: u8 },
    /// A radial part on a tube's surface.
    Radial { target: SurfaceTarget, count: u8 },
    /// A radial group pulled free of the rocket and over empty space: released, it is removed.
    Removed,
    /// An internal part inside an airframe part.
    Internal { target: InternalTarget },
}

impl Drags {
    /// Where the part carried now would land; `None` when nothing is carried or the carried part has no target (it would go
    /// back, so the rocket would not change).
    pub fn landing(&self) -> Option<Landing> {
        if let Some(d) = &self.axial {
            return (d.snap.is_some() || d.pod_snap.is_some() || d.surface.is_some()).then_some(Landing::Axial {
                snap: d.snap,
                pod_snap: d.pod_snap,
                surface: d.surface,
                count: d.count,
            });
        }
        if let Some(d) = &self.radial {
            return d.target.map(|target| Landing::Radial { target, count: d.count }).or((d.detached && d.origin.is_some()).then_some(Landing::Removed));
        }
        self.internal.as_ref().and_then(|d| d.target).map(|target| Landing::Internal { target })
    }
}

/// The rocket as it would be if the drag in progress were released now: the document's rocket (the carried part is out of it)
/// with the part landed on its target, exactly as [`finish_axial`], [`finish_radial`] and [`finish_internal`] land it. `None`
/// when there is no target or the landing is refused.
pub fn landed(ctx: &Ctx) -> Option<Rocket> {
    let mut rocket = ctx.doc.rocket().clone();
    let base = ctx.doc.drag_base();
    let ok = if let Some(drag) = &ctx.drag.axial {
        match (drag.snap, drag.pod_snap, drag.surface) {
            (Some(node), _, _) => land_on_node(&mut rocket, drag, node, base).is_some(),
            (None, Some((pod, node)), _) => land_in_pod(&mut rocket, drag, pod, node).is_some(),
            (None, None, Some(t)) => land_on_surface(&mut rocket, drag, base, t).is_some(),
            _ => false,
        }
    } else if let Some(drag) = &ctx.drag.radial {
        // A group pulled free and over empty space is removed on release: the rocket as it is, without it.
        drag.target.map_or(drag.detached && drag.origin.is_some(), |t| land_radial(&mut rocket, drag, t))
    } else if let Some(drag) = &ctx.drag.internal {
        drag.target.is_some_and(|t| land_internal(&mut rocket, drag, t, base))
    } else {
        false
    };
    if ok {
        rocket.apply_formulas();
    }
    ok.then_some(rocket)
}

/// Abandons whichever drag is in progress.
pub fn cancel(ctx: &mut Ctx) {
    finish_axial(ctx, false);
    finish_radial(ctx, false);
    finish_internal(ctx, false);
}

/// Deletes every selected part (an axial part takes its radial and internal parts with it; a radial part is a whole group) as
/// one undo step.
pub fn delete_selection(ctx: &mut Ctx) {
    if ctx.drag.active() {
        return;
    }
    let ids = ctx.selected_parts();
    if ids.is_empty() {
        return;
    }
    let label = if ids.len() > 1 { format!("Delete {} parts", ids.len()) } else { "Delete part".to_string() };
    ctx.doc.edit(&label, |r| ids.iter().for_each(|id| remove_any(r, *id)));
    ctx.select_one(None);
}

/// Removes the part `id` of whatever class it is.
fn remove_any(rocket: &mut Rocket, id: PartId) {
    if rocket.remove(id).is_none() && rocket.remove_radial(id).is_none() {
        rocket.remove_internal(id);
    }
}
