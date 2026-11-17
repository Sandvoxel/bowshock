//! The drag overlay: while a part is dragged in from the parts bin or picked up it shows the valid attach nodes on the
//! rocket's centreline and, over a tube's side, the radial surface target (design/components/ViewportOverlay).
//!
//! An axial part is held at its hold point ([`Drag::hold`]: where it was grabbed, or its middle for a part from the bin) under the
//! cursor. [`free_pose`] is where its fore and aft faces then are in the air, and that is what both the ghost and the snapping
//! work from: a node catches the part when one of the part's *ends* (see [`End`]) projects within the node's catch disc, the
//! nearer end of the part for a joint between parts, the aft end for the node ahead of the first part, the fore end for the node
//! behind the last. The cursor itself is no snap point for a node. Failing a node, the tube under the cursor wins: the part becomes
//! a pod, `count` copies standing on the tube's surface (KSP-style radial attachment of an ordinary part); else the node beside the
//! rocket along its axis that an end is nearest. The ghost part itself is drawn in 3D by the model mesh.

use bowshock_model::{InternalKind, InternalPart, Part, PartId, PartKind, PodBody, RadialKind, RadialPart, Rocket};
use glam::Vec3;
use iced::widget::canvas::{self, Action, Event, Frame, Geometry, LineDash, Path, Stroke, Text};
use iced::{alignment, mouse, Color, Point, Rectangle, Renderer, Size, Theme, Vector};

use super::pick::{self, SurfaceTarget};
use super::pods::PodView;
use super::radial;
use super::{Camera, Message};
use crate::app::ui::{len, len_unit};
use crate::theme::{self, MONO};
use crate::tokens::TEXT_LABEL;
use crate::units::UnitSystem;

/// How close (px) the cursor must be to an attach node for the part to stack there rather than attach to the tube's side.
pub const NODE_PX: f32 = 20.0;
/// How far from the rocket's axis line (on screen), and past either end of it, the open space still counts as "beside the
/// rocket" for stacking. Kept near the body: a part pulled clear of the rocket must come free, not jump back to the nearest
/// joint.
pub const VOID_PX: f32 = 48.0;
/// A node catches the cursor within this multiple of the airframe's projected radius there (when that is more than
/// [`NODE_PX`]), and never beyond [`CATCH_MAX_PX`] px.
const CATCH_SCALE: f32 = 1.15;
const CATCH_MAX_PX: f32 = 96.0;
/// The view counts as along the rocket when the view direction has more than this much of the axis in it (a view direction on
/// the axis is 1).
const FORESHORTENED: f32 = 0.75;
/// Diameter of a node ring.
const RING: f32 = 7.0;
/// Width of one MONO glyph at `TEXT_LABEL`, as a fraction of the font size (IBM Plex Mono advances 0.6 em).
const GLYPH_EM: f32 = 0.6;
const TAG_PAD: f32 = 6.0;

/// A part being dragged: what, and the attach node it would drop onto.
#[derive(Debug, Clone)]
pub struct Drag {
    /// The row of the parts bin it came from, if it came from there.
    pub row: Option<usize>,
    /// Where it was picked up from, if it is a part of the rocket being moved.
    pub origin: Option<Origin>,
    /// The bin's name for it ("Transition", "Boattail"), used in the cursor tag.
    pub name: &'static str,
    pub kind: PartKind,
    pub snap: Option<usize>,
    /// The tube side it would attach to radially instead (the part becomes a pod): at most one of `snap`, `pod_snap` and
    /// `surface` is set.
    pub surface: Option<SurfaceTarget>,
    /// Copies of the pod it would become on a tube's side (the bin's symmetry count, or the old pod's).
    pub count: u8,
    /// The node of a pod's stack it would drop onto instead: `(pod, node index)`. At most one of `snap` and `pod_snap` is set.
    pub pod_snap: Option<(PartId, usize)>,
    /// Where the part is held while it is carried: the distance (metres) from its fore face to the point under the cursor. Where
    /// it was grabbed for a part picked up off the rocket, its middle for one from the bin. See [`Drag::held`].
    pub hold: f64,
}

/// A part of the rocket picked up for moving: it is out of the rocket while dragged, and goes back as it was if the drag is
/// cancelled or ends with no snap target.
#[derive(Debug, Clone)]
pub struct Origin {
    /// The part exactly as it was (material, finish, overrides), before any diameter matching.
    pub part: Part,
    /// The radial parts it carries: taken off while it is dragged, put back on it where it lands.
    pub radials: Vec<RadialPart>,
    /// The internal parts it carries, likewise.
    pub internals: Vec<InternalPart>,
}

impl Drag {
    /// The hold point for a grab `grab` metres behind the part's fore face (`None`: its middle), limited to the part.
    pub fn held(kind: &PartKind, grab: Option<f64>) -> f64 {
        let length = kind.length();
        grab.unwrap_or(length / 2.0).clamp(0.0, length)
    }
}

/// The origin of a part that came from the parts bin, for drawing it as a held part: the part alone, placeholder id 0, carrying
/// nothing.
pub fn bin_origin(kind: &PartKind) -> Origin {
    Origin { part: Part::new(0, kind.clone()), radials: Vec::new(), internals: Vec::new() }
}

/// A bare tube of `radius` and `length` as a rocket of its own: what a held radial group is built on.
pub fn stand_in_tube(radius: f64, length: f64) -> Option<(Rocket, PartId)> {
    let mut rocket = Rocket::default();
    let id = rocket.insert(0, PartKind::BodyTube { length: length.max(0.01), diameter: (2.0 * radius).max(0.005) }).ok()?;
    Some((rocket, id))
}

/// A tube whose bore is `bore` wide with the internal part `kind` in it at its fore end: what a held internal part is built on.
pub fn stand_in_internal(kind: &InternalKind, bore: f64) -> Option<(Rocket, PartId)> {
    let length = kind.extent().max(0.01);
    let bore = bore.max(0.005);
    // The tube's wall is its default one: widen the tube by twice the wall so the bore is the one asked for.
    let wall = (bore - Part::new(0, PartKind::BodyTube { length, diameter: bore }).inner_diameter_at(length / 2.0)) / 2.0;
    let (mut rocket, tube) = stand_in_tube((bore + 2.0 * wall) / 2.0, length)?;
    let id = rocket.attach_internal(tube, 0.0, kind.clone()).ok()?;
    Some((rocket, id))
}

/// The radius of the body a held radial or internal part would most plausibly go on: the selected part's (or, for a selected
/// radial or internal part, that of the part it stands on or in), else the rocket's widest body.
pub fn plausible_radius(rocket: &Rocket, selected: Option<PartId>) -> f64 {
    let radius = |p: &Part| p.kind.fore_diameter().max(p.kind.aft_diameter()) / 2.0;
    selected
        .and_then(|id| rocket.part(id).or_else(|| rocket.parent_of(id).and_then(|p| rocket.part(p))))
        .map(radius)
        .or_else(|| rocket.parts().iter().map(radius).max_by(f64::total_cmp))
        .unwrap_or(0.03)
}

/// The bore of the body a held internal part would most plausibly go in: the selected body part's (or that of the part a selected
/// radial or internal part is on), else the widest bore in the rocket.
pub fn plausible_bore(rocket: &Rocket, selected: Option<PartId>) -> f64 {
    let bore = |p: &Part| p.inner_diameter_at(p.kind.length() / 2.0);
    selected
        .and_then(|id| rocket.part(id).or_else(|| rocket.parent_of(id).and_then(|p| rocket.part(p))))
        .map(bore)
        .or_else(|| rocket.parts().iter().map(bore).max_by(f64::total_cmp))
        .unwrap_or(0.05)
}

/// The point under screen point `at` on the plane through the rocket's axis that faces the camera (looking along the axis there is
/// no such plane: the one through the view target). `None` when the cursor's ray runs parallel to the plane.
pub fn plane_point(camera: &Camera, size: Size, at: Point) -> Option<Vec3> {
    let (from, dir) = camera.ray(at, size);
    let back = camera.back();
    let across = back - Vec3::Z * back.z;
    let (normal, through) = if across.length() > 0.05 { (across.normalize(), Vec3::ZERO) } else { (back, camera.target) };
    let facing = normal.dot(dir);
    if facing.abs() < 1e-4 {
        return None;
    }
    Some(from + dir * (normal.dot(through - from) / facing))
}

/// A held part in the air: where its fore and aft faces are, on its axis (parallel to the rocket's, the nose end towards the
/// rocket's nose).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FreePose {
    /// World position of the fore face's centre.
    pub fore: Vec3,
    /// World position of the aft face's centre.
    pub aft: Vec3,
}

impl FreePose {
    /// The end `end`.
    pub fn end(&self, end: End) -> Vec3 {
        match end {
            End::Fore => self.fore,
            End::Aft => self.aft,
        }
    }
}

/// One end of a held part.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum End {
    /// The fore face (towards the rocket's nose).
    Fore,
    /// The aft face.
    Aft,
}

/// The free pose of a part `length` metres long held `hold` metres behind its fore face with the hold point under screen point
/// `at`: the one place both the ghost drawn in the air and the snapping take the part's position from, so what is drawn is what
/// snaps. `None` when the cursor's ray runs parallel to the plane the part is held in.
pub fn free_pose(camera: &Camera, size: Size, at: Point, length: f64, hold: f64) -> Option<FreePose> {
    let hit = plane_point(camera, size, at)?;
    let fore = Vec3::new(hit.x, hit.y, hit.z - hold.clamp(0.0, length) as f32);
    Some(FreePose { fore, aft: fore + Vec3::Z * length as f32 })
}

pub struct Overlay<'a> {
    pub rocket: &'a Rocket,
    pub drag: &'a Drag,
    pub camera: Camera,
    /// The cursor in window coordinates (tracked by the app, so it also draws without a live cursor).
    pub cursor: Option<Point>,
    /// Angle snap for the radial target, degrees (0 for none).
    pub snap_angle: u8,
    /// The view toolbar's grid step, metres.
    pub snap_step: f64,
    /// The margin of the rocket as it would be if released now, calibers: appended to the cursor tag in its stability band's
    /// colour. `None` shows nothing (no valid target).
    pub margin: Option<f64>,
    /// The display unit set the tag's stations are written in.
    pub units: UnitSystem,
}

/// The pod body the dragged part would be on a tube's side: that one part with what stands on and in it, ids kept for a part
/// of the rocket, placeholder ids for a new one.
pub fn body_of(drag: &Drag) -> PodBody {
    match &drag.origin {
        Some(o) => PodBody::single(Part { kind: drag.kind.clone(), ..o.part.clone() }, o.radials.clone(), o.internals.clone()),
        None => PodBody::single(Part::new(0, drag.kind.clone()), Vec::new(), Vec::new()),
    }
}

/// Every id of the part picked up and of what it carries (its radial and internal parts): what the ghost draws in the ghost state.
pub fn carried_ids(origin: &Origin) -> Vec<PartId> {
    std::iter::once(origin.part.id).chain(origin.radials.iter().map(|r| r.id)).chain(origin.internals.iter().map(|i| i.id)).collect()
}

/// Puts back what removing and re-adding a part dropped from the configurations: every motor slot of `from` whose mount exists
/// again in `rocket` but has lost its slot, and every recovery deployment of a device that exists again.
pub fn restore_motors(rocket: &mut Rocket, from: &Rocket) {
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

/// Lands the part picked up (`origin`, of `kind`) on attach node `node` of `rocket` with what it carried: its radial and internal
/// parts, offsets limited to the matched part by the model, and its motors (from `base`, the rocket as it was before the drag).
/// The matched kind. A drop and the ghost both use this, so the ghost shows exactly what the drop makes.
pub fn land_carried(rocket: &mut Rocket, node: usize, kind: &PartKind, origin: &Origin, base: Option<&Rocket>) -> Option<PartKind> {
    let matched = rocket.matched(node, kind);
    rocket.restore(node, Part { kind: matched.clone(), ..origin.part.clone() }).ok()?;
    rocket.restore_radials(origin.radials.clone());
    rocket.restore_internals(origin.internals.clone());
    if let Some(base) = base {
        restore_motors(rocket, base);
    }
    Some(matched)
}

/// The picked-up part with what it carries as a rocket of its own (the part alone in the stack, its motors in the active
/// configuration if `base` loaded them), for the ghost pulled free of the rocket.
pub fn free_rocket(origin: &Origin, base: Option<&Rocket>) -> Rocket {
    let mut rocket = PodBody::single(origin.part.clone(), origin.radials.clone(), origin.internals.clone()).to_rocket();
    if let Some(base) = base {
        let config = rocket.active_configuration().id;
        for slot in &base.active_configuration().motors {
            if origin.internals.iter().any(|i| i.id == slot.mount) {
                let _ = rocket.set_motor(config, slot.mount, slot.motor.clone());
            }
        }
    }
    rocket
}

/// What the cursor chooses.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Choice {
    Node(Snap),
    Surface(SurfaceTarget),
}

/// Interaction state: trackpad scroll not yet worth a step.
#[derive(Debug, Default)]
pub struct State {
    wheel: f32,
}

impl Overlay<'_> {
    /// Whether the part carries a pod (a part of the core that has pods on it cannot go into a pod: pods do not nest).
    fn carries_pod(&self) -> bool {
        self.drag.origin.as_ref().is_some_and(|o| o.radials.iter().any(|r| r.kind.is_pod()))
    }

    /// Valid nodes of the core and of every pod's stack, with their screen positions (a pod's on each of its copies).
    fn nodes(&self, size: Size) -> Vec<Node> {
        let core = self.rocket.nodes_for(&self.drag.kind).into_iter().map(|(index, station)| {
            let at = self.camera.project(Vec3::new(0.0, 0.0, station as f32), size);
            Node { snap: Snap::Core(index), station, at, catch: self.catch_px(index, at, size) }
        });
        let mut nodes: Vec<Node> = core.collect();
        if !self.carries_pod() {
            for pod in self.rocket.pods() {
                let Some(view) = PodView::of_pod(self.rocket, pod, &self.camera) else { continue };
                for (index, station) in self.rocket.nodes_for_pod(pod.id, &self.drag.kind) {
                    for (pose, camera) in view.poses.iter().zip(&view.cameras) {
                        let at = camera.project(Vec3::Z * (station as f32 - pose.z0), size);
                        nodes.push(Node { snap: Snap::Pod(pod.id, index), station, at, catch: NODE_PX });
                    }
                }
            }
        }
        nodes
    }

    /// How close (px) the cursor must be to core node `index` (on screen at `at`) to catch it: the node's own [`NODE_PX`], or
    /// 1.15 times the airframe's projected outer radius there if that is more, so the catch disc covers the end face of the
    /// tube as it appears (at most [`CATCH_MAX_PX`]).
    fn catch_px(&self, index: usize, at: Point, size: Size) -> f32 {
        let parts = self.rocket.parts();
        let fore = parts.get(index).map_or(0.0, |p| p.kind.fore_diameter());
        let aft = index.checked_sub(1).and_then(|i| parts.get(i)).map_or(0.0, |p| p.kind.aft_diameter());
        let rim = self.camera.project(
            Vec3::new(0.0, 0.0, self.rocket.stations().get(index).copied().unwrap_or(0.0) as f32) + self.camera.right() * (fore.max(aft) / 2.0) as f32,
            size,
        );
        (CATCH_SCALE * at.distance(rim)).clamp(NODE_PX, CATCH_MAX_PX)
    }

    /// The held part's fore and aft faces on screen for the cursor at `at`: the [`free_pose`] projected. `None` when the part cannot
    /// be placed in the air (the cursor's ray runs parallel to the plane it is held in).
    fn ends(&self, at: Point, size: Size) -> Option<[(End, Point); 2]> {
        let pose = free_pose(&self.camera, size, at, self.drag.kind.length(), self.drag.hold)?;
        Some([End::Fore, End::Aft].map(|end| (end, self.camera.project(pose.end(end), size))))
    }

    /// The ends of the held part that may meet node `snap`: either end at a joint between parts, only the aft end at the node ahead
    /// of the first part (the part goes in front of it) and only the fore end at the node behind the last. An empty stack takes
    /// either.
    fn meets(&self, snap: Snap) -> &'static [End] {
        let (index, stack) = match snap {
            Snap::Core(index) => (index, self.rocket.parts().len()),
            Snap::Pod(pod, index) => (index, self.rocket.radial(pod).and_then(|p| p.kind.pod_body()).map_or(0, |b| b.parts().len())),
        };
        match (index == 0, index >= stack) {
            (true, false) => &[End::Aft],
            (false, true) => &[End::Fore],
            _ => &[End::Fore, End::Aft],
        }
    }

    /// What the cursor at `at` chooses, as the fields of the drag it sets: `(snap, pod_snap, surface)`. For the viewport to apply the
    /// decision itself when the view changes under a cursor that has not moved (see `View::resnap`).
    pub fn resolve(&self, at: Point, size: Size) -> (Option<usize>, Option<(PartId, usize)>, Option<SurfaceTarget>) {
        match self.choice(at, size) {
            Some(Choice::Node(Snap::Core(index))) => (Some(index), None, None),
            Some(Choice::Node(Snap::Pod(pod, index))) => (None, Some((pod, index)), None),
            Some(Choice::Surface(target)) => (None, None, Some(target)),
            None => (None, None, None),
        }
    }

    /// What the cursor at `at` chooses: over a part's surface in a view along the rocket, the node at that part's far end;
    /// else the node a part's end is in the catch disc of (the nearest end and node), else the tube side under the cursor (a part
    /// that carries a pod cannot become one), else the node beside the rocket that an end is nearest along its axis.
    fn choice(&self, at: Point, size: Size) -> Option<Choice> {
        self.decide(at, size).map(|(choice, _)| choice)
    }

    /// [`choice`](Self::choice), and the end of the part that a node choice is made by (`None` for a surface, a far end, and an
    /// empty rocket's one node). It depends on the free pose and the cursor only, never on where the snapped ghost is drawn.
    fn decide(&self, at: Point, size: Size) -> Option<(Choice, Option<End>)> {
        let nodes = self.nodes(size);
        if let Some(snap) = self.far_end(&nodes, at, size) {
            return Some((Choice::Node(snap), None));
        }
        let ends = self.ends(at, size);
        let mut caught: Option<(Snap, End, f32)> = None;
        for &(end, point) in ends.iter().flatten() {
            for node in nodes.iter().filter(|n| self.meets(n.snap).contains(&end)) {
                let distance = node.at.distance(point);
                if distance <= node.catch && caught.is_none_or(|(_, _, best)| distance < best) {
                    caught = Some((node.snap, end, distance));
                }
            }
        }
        if let Some((snap, end, _)) = caught {
            return Some((Choice::Node(snap), Some(end)));
        }
        if !self.carries_pod() {
            if let Some(target) = self.surface_under(at, size) {
                return Some((Choice::Surface(target), None));
            }
        }
        // In the open space beside the rocket (the hold point within [`VOID_PX`] of its axis line), the node nearest along the axis to
        // an end takes the part, so dropping a part just below the stack still stacks it at the joint above.
        if let Some((snap, end)) = self.beside(at, size) {
            return Some((Choice::Node(snap), Some(end)));
        }
        // Far from the rocket there is no target ("no node"); an empty rocket has only its first node.
        nodes.first().filter(|_| self.rocket.parts().is_empty()).map(|n| (Choice::Node(n.snap), None))
    }

    /// The tube side under the cursor, where the part would stand as a pod.
    fn surface_under(&self, at: Point, size: Size) -> Option<SurfaceTarget> {
        let kind = RadialKind::Pod { gap: 0.0, body: body_of(self.drag) };
        pick::surface_target(self.rocket, &self.camera, at, size, &kind, pick::Snapping { angle: self.snap_angle, station: Some(self.snap_step) })
    }

    /// Looking along the rocket (more than [`FORESHORTENED`] of the view direction on the axis) every node projects into a few
    /// pixels and the far end of a piece is hidden behind it: with the cursor over a piece's surface, its far end from the
    /// viewer (the aft node looking from the nose, the fore node looking from the tail) is the node, if the part can stack there.
    fn far_end(&self, nodes: &[Node], at: Point, size: Size) -> Option<Snap> {
        let along = self.camera.back().z;
        if along.abs() <= FORESHORTENED {
            return None;
        }
        let index = self.rocket.index_of(self.surface_under(at, size)?.parent)?;
        let snap = Snap::Core(if along < 0.0 { index + 1 } else { index });
        nodes.iter().any(|n| n.snap == snap).then_some(snap)
    }

    /// The core node nearest along the rocket's axis to an end of the held part (one the node takes, see [`meets`](Self::meets)),
    /// when the hold point (the cursor) is within [`VOID_PX`] of the axis line on screen and the end is not past either end of the
    /// rocket by more than that; with the end it is nearest.
    fn beside(&self, at: Point, size: Size) -> Option<(Snap, End)> {
        let nodes: Vec<Node> = self.nodes(size).into_iter().filter(|n| matches!(n.snap, Snap::Core(_))).collect();
        let first = nodes.iter().min_by(|a, b| a.station.total_cmp(&b.station))?;
        let last = nodes.iter().max_by(|a, b| a.station.total_cmp(&b.station))?;
        let axis = last.at - first.at;
        let len = axis.x.hypot(axis.y);
        if len < 1.0 {
            return None;
        }
        let dir = Vector::new(axis.x / len, axis.y / len);
        let rel = at - first.at;
        if (rel.x * dir.y - rel.y * dir.x).abs() > VOID_PX {
            return None;
        }
        let along = |p: Point| {
            let r = p - first.at;
            r.x * dir.x + r.y * dir.y
        };
        let mut best: Option<(Snap, End, f32)> = None;
        for (end, point) in self.ends(at, size)? {
            let end_along = along(point);
            if end_along < -VOID_PX || end_along > len + VOID_PX {
                continue;
            }
            for node in nodes.iter().filter(|n| self.meets(n.snap).contains(&end)) {
                let distance = (along(node.at) - end_along).abs();
                if best.is_none_or(|(_, _, b)| distance < b) {
                    best = Some((node.snap, end, distance));
                }
            }
        }
        best.map(|(snap, end, _)| (snap, end))
    }

    /// Where the drag is snapped now.
    fn current(&self) -> Option<Choice> {
        self.drag
            .surface
            .map(Choice::Surface)
            .or_else(|| self.drag.snap.map(|i| Choice::Node(Snap::Core(i))))
            .or_else(|| self.drag.pod_snap.map(|(pod, index)| Choice::Node(Snap::Pod(pod, index))))
    }

    /// The node the drag is snapped to, if it stacks.
    fn current_node(&self) -> Option<Snap> {
        match self.current() {
            Some(Choice::Node(snap)) => Some(snap),
            _ => None,
        }
    }

    /// What releasing here would do, as a tag.
    fn tag(&self, snap: Snap, station: f64) -> String {
        let name = self.drag.name;
        match snap {
            Snap::Core(snap) => {
                let parts = self.rocket.parts().len();
                if snap == 0 {
                    format!("{name} → snap fore")
                } else if snap == parts {
                    format!("{name} → snap aft")
                } else {
                    format!("{name} → insert at {} {}", len(station, self.units), len_unit(self.units))
                }
            }
            Snap::Pod(pod, index) => {
                let copies = self.rocket.radial(pod).map_or(1, |p| p.count);
                let times = if copies > 1 { format!(" ×{copies}") } else { String::new() };
                let parts = self.rocket.radial(pod).and_then(|p| p.kind.pod_body()).map_or(0, |b| b.parts().len());
                let place = if index == 0 {
                    "fore".to_string()
                } else if index == parts {
                    "aft".to_string()
                } else {
                    format!("at {} {}", len(station, self.units), len_unit(self.units))
                };
                format!("{name} → pod{times}, {place}")
            }
        }
    }
}

/// Where a carried part would attach: a node of the core's stack or of a pod's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Snap {
    /// Node `index` of the core.
    Core(usize),
    /// Node `index` of pod `pod`'s stack.
    Pod(PartId, usize),
}

/// An attach node on screen.
struct Node {
    snap: Snap,
    station: f64,
    at: Point,
    /// How close (px) the cursor must be for the node to catch it.
    catch: f32,
}

impl canvas::Program<Message> for Overlay<'_> {
    type State = State;

    fn update(&self, state: &mut State, event: &Event, bounds: Rectangle, cursor: mouse::Cursor) -> Option<Action<Message>> {
        // A release over the viewport drops (the app decides what that does: see `drag::end_carry`), and so does a press, which
        // places a part held with no button down (click to place); elsewhere the app decides.
        if let Event::Mouse(mouse::Event::ButtonReleased(mouse::Button::Left)) = event {
            return cursor.position_over(bounds).map(|_| Action::publish(Message::DropAxial).and_capture());
        }
        if let Event::Mouse(mouse::Event::ButtonPressed(mouse::Button::Left)) = event {
            return cursor.position_over(bounds).map(|_| Action::publish(Message::PlaceHere).and_capture());
        }
        if let (Event::Mouse(mouse::Event::WheelScrolled { delta }), true) = (event, self.drag.surface.is_some()) {
            // Over a tube's side the wheel steps the number of copies rather than zooming.
            cursor.position_over(bounds)?;
            let steps = match *delta {
                mouse::ScrollDelta::Lines { y, .. } => y,
                mouse::ScrollDelta::Pixels { y, .. } => {
                    state.wheel += y / 40.0;
                    let whole = state.wheel.trunc();
                    state.wheel -= whole;
                    whole
                }
            };
            return Some(if steps == 0.0 { Action::capture() } else { Action::publish(Message::CopiesStep(steps > 0.0)).and_capture() });
        }
        let choice = cursor.position_in(bounds).and_then(|p| self.choice(p, bounds.size()));
        (choice != self.current()).then(|| {
            Action::publish(match choice {
                Some(Choice::Node(Snap::Pod(pod, index))) => Message::SnapPodNode(Some((pod, index))),
                Some(Choice::Node(Snap::Core(index))) => Message::SnapNode(Some(index)),
                Some(Choice::Surface(target)) => Message::AxialSurface(Some(target)),
                None => Message::SnapNode(None),
            })
        })
    }

    fn draw(&self, _: &State, renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let size = bounds.size();
        let mut frame = Frame::new(renderer, size);
        let nodes = self.nodes(size);

        // Parts after the ghost have moved aft by its length: draw their nodes where those parts now are. Snapping keeps
        // to the settled positions, so the target does not slide under the cursor.
        let ghost = self.drag.snap.map(|snap| (snap, self.rocket.matched(snap, &self.drag.kind).length()));
        let current = self.current_node();
        for node in &nodes {
            let at = match (ghost, node.snap) {
                (Some((snap, len)), Snap::Core(index)) if index > snap => self.camera.project(Vec3::new(0.0, 0.0, (node.station + len) as f32), size),
                _ => node.at,
            };
            let target = current == Some(node.snap);
            if target {
                frame.fill(&Path::circle(at, RING * 1.5), t.ok.scale_alpha(0.18));
            }
            let color = if target { t.ok } else { t.ok.scale_alpha(0.5) };
            frame.stroke(&Path::circle(at, RING / 2.0), Stroke::default().with_color(color).with_width(1.5));
        }

        // Cursor feedback only while the cursor is over the viewport.
        let local = self.cursor.map(|c| Point::new(c.x - bounds.x, c.y - bounds.y));
        let Some(cursor) = local.filter(|c| (0.0..=size.width).contains(&c.x) && (0.0..=size.height).contains(&c.y)) else {
            return vec![frame.into_geometry()];
        };

        // Where the part's ends are in the air: a hollow ring at each, filled where an end is caught by a node. A part that would
        // attach to a tube's side has no ends in play.
        if self.drag.surface.is_none() {
            let caught = self.decide(cursor, size).and_then(|(choice, end)| end.filter(|_| matches!(choice, Choice::Node(_))));
            for (end, at) in self.ends(cursor, size).into_iter().flatten() {
                if caught == Some(end) {
                    frame.fill(&Path::circle(at, RING / 2.0), t.accent);
                    frame.stroke(&Path::circle(at, RING / 2.0), Stroke::default().with_color(t.accent).with_width(1.5));
                } else {
                    frame.stroke(&Path::circle(at, RING / 2.0), Stroke::default().with_color(t.ink_muted).with_width(1.5));
                }
            }
        }

        if let Some(target) = self.drag.surface {
            // Radial: a ring where the part would stand on the tube, a leader to it and what releasing does.
            let extent = self.drag.kind.length();
            let station = self.rocket.index_of(target.parent).map_or(0.0, |i| self.rocket.stations()[i]);
            let radius = self.rocket.surface_radius(target.parent, target.offset + extent / 2.0).unwrap_or(0.0);
            let (u, _) = radial::dirs(target.angle as f32);
            let at = self.camera.project(u * radius as f32 + Vec3::Z * (station + target.offset + extent / 2.0) as f32, size);
            frame.fill(&Path::circle(at, RING * 1.5), t.ok.scale_alpha(0.18));
            frame.stroke(&Path::circle(at, RING / 2.0), Stroke::default().with_color(t.ok).with_width(1.5));
            let dashed = Stroke { line_dash: LineDash { segments: &[4.0, 3.0], offset: 0 }, ..Stroke::default().with_color(t.ok).with_width(1.0) };
            frame.stroke(&Path::line(cursor, at), dashed);
            let times = if self.drag.count > 1 { format!(" ×{}", self.drag.count) } else { String::new() };
            let parent = self.rocket.part(target.parent).map_or("", |p| p.kind.name());
            let label = format!(
                "{}{times} → {parent} @ {} {}, {:.0}° · radial",
                self.drag.name,
                len(station + target.offset, self.units),
                len_unit(self.units),
                target.angle.to_degrees().round() % 360.0
            );
            tag_with(&mut frame, cursor, 0.0, &label, margin_suffix(self.rocket, t, self.margin), t.ink, t.line_200, t.bg_200);
            return vec![frame.into_geometry()];
        }

        let target = current.and_then(|snap| nodes.iter().find(|n| n.snap == snap));
        let label = match target {
            Some(node) => {
                // A dashed accent outline round the ghost where the view is square-on (the 3D view shows its edge instead).
                if let (Snap::Core(index), true) = (node.snap, self.camera.is_ortho()) {
                    let b = pick::screen_bounds(&self.camera, size, node.station, &self.rocket.matched(index, &self.drag.kind));
                    let outline = Stroke { line_dash: LineDash { segments: &[4.0, 3.0], offset: 0 }, ..Stroke::default().with_color(t.accent).with_width(1.0) };
                    frame.stroke_rectangle(b.position(), b.size(), outline);
                }
                let leader = Path::line(cursor, node.at);
                let dashed = Stroke { line_dash: LineDash { segments: &[4.0, 3.0], offset: 0 }, ..Stroke::default().with_color(t.ok).with_width(1.0) };
                frame.stroke(&leader, dashed);
                self.tag(node.snap, node.station)
            }
            None => {
                no_entry(&mut frame, cursor + iced::Vector::new(14.0, 14.0), t.err);
                "No attach node".to_string()
            }
        };
        // The tag sits down-right of the cursor (after the no-entry glyph when there is one).
        let glyph = if target.is_some() { 0.0 } else { 18.0 };
        let suffix = margin_suffix(self.rocket, t, self.margin.filter(|_| target.is_some()));
        tag_with(&mut frame, cursor, glyph, &label, suffix, t.ink, t.line_200, t.bg_200);
        vec![frame.into_geometry()]
    }

    fn mouse_interaction(&self, _: &State, bounds: Rectangle, cursor: mouse::Cursor) -> mouse::Interaction {
        match (cursor.is_over(bounds), self.current()) {
            (false, _) => mouse::Interaction::default(),
            (true, Some(_)) => mouse::Interaction::Grabbing,
            (true, None) => mouse::Interaction::NotAllowed,
        }
    }
}

/// A framed square tag with `label` in MONO, down-right of `cursor` (`indent` px further right), flipped to stay inside the frame.
pub fn tag(frame: &mut Frame, cursor: Point, indent: f32, label: &str, ink: Color, edge: Color, bg: Color) {
    tag_with(frame, cursor, indent, label, None, ink, edge, bg);
}

/// The text the tag appends for the previewed margin: ` · 2.07 cal` in the colour of the margin's stability band.
pub fn margin_suffix(rocket: &Rocket, t: &crate::tokens::Tokens, margin: Option<f64>) -> Option<(String, Color)> {
    margin.map(|m| (format!(" · {m:.2} cal"), super::markers::margin_color(t, rocket.stability_bands(), m)))
}

/// [`tag`] with `suffix` after the label, in its own colour (the previewed margin).
#[allow(clippy::too_many_arguments)]
pub fn tag_with(frame: &mut Frame, cursor: Point, indent: f32, label: &str, suffix: Option<(String, Color)>, ink: Color, edge: Color, bg: Color) {
    let size = frame.size();
    let chars = label.chars().count() + suffix.as_ref().map_or(0, |(s, _)| s.chars().count());
    let (w, h) = (chars as f32 * TEXT_LABEL * GLYPH_EM + 2.0 * TAG_PAD, TEXT_LABEL + 2.0 * TAG_PAD - 2.0);
    let mut origin = cursor + iced::Vector::new(14.0 + indent, 14.0);
    if origin.x + w > size.width {
        origin.x = cursor.x - 14.0 - w;
    }
    if origin.y + h > size.height {
        origin.y = cursor.y - 14.0 - h;
    }
    frame.fill_rectangle(origin, Size::new(w, h), bg);
    frame.stroke_rectangle(origin, Size::new(w, h), Stroke::default().with_color(edge).with_width(1.0));
    let text = |content: String, x: f32, color: Color| Text {
        content,
        position: Point::new(x, origin.y + h / 2.0),
        color,
        size: TEXT_LABEL.into(),
        font: MONO,
        align_y: alignment::Vertical::Center,
        ..Text::default()
    };
    frame.fill_text(text(label.to_string(), origin.x + TAG_PAD, ink));
    if let Some((suffix, color)) = suffix {
        frame.fill_text(text(suffix, origin.x + TAG_PAD + label.chars().count() as f32 * TEXT_LABEL * GLYPH_EM, color));
    }
}

/// A ⊘ glyph with its top-left at `at`: a ring crossed by a slash, in `color`.
pub fn no_entry(frame: &mut Frame, at: Point, color: Color) {
    let r = 6.0;
    let centre = Point::new(at.x + r, at.y + r);
    let stroke = Stroke::default().with_color(color).with_width(1.5);
    frame.stroke(&Path::circle(centre, r), stroke);
    let d = r * std::f32::consts::FRAC_1_SQRT_2;
    frame.stroke(&Path::line(Point::new(centre.x - d, centre.y + d), Point::new(centre.x + d, centre.y - d)), stroke);
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::NoseShape;

    const SIZE: Size = Size::new(900.0, 400.0);

    /// A nose (0 to 0.2 m) and a tube (0.2 to 0.7 m), both Ø0.1 m: nodes 0, 1 and 2 at stations 0, 0.2 and 0.7.
    fn rocket() -> (Rocket, PartId) {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.1, tip_radius: 0.0 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.1 }).unwrap();
        (r, tube)
    }

    /// A new part from the bin: held by its middle.
    fn drag(kind: PartKind) -> Drag {
        let hold = Drag::held(&kind, None);
        Drag { row: Some(0), origin: None, name: "Part", kind, snap: None, surface: None, count: 3, pod_snap: None, hold }
    }

    /// Orthographic, 500 px per metre, looking at the middle of the rocket.
    fn camera(yaw: f32, pitch: f32) -> Camera {
        Camera { target: Vec3::new(0.0, 0.0, 0.35), yaw, pitch, scale: 500.0, ortho_blend: 1.0, ..Camera::default() }
    }

    fn choose(rocket: &Rocket, drag: &Drag, camera: Camera, at: Point) -> Option<Choice> {
        let overlay = Overlay { rocket, drag, camera, cursor: None, snap_angle: 0, snap_step: 0.001, margin: None, units: UnitSystem::default() };
        overlay.choice(at, SIZE)
    }

    #[test]
    fn a_nodes_catch_disc_covers_the_tubes_projected_end_face() {
        let (rocket, tube) = rocket();
        let drag = drag(PartKind::BodyTube { length: 0.3, diameter: 0.1 });
        // Side view (+z to the right, 500 px/m): the joint at 0.2 m is at x = 375 and the Ø0.1 tube is 25 px in radius there, so
        // its disc is 1.15 * 25 = 28.75 px (NODE_PX alone is 20). The tube is held by its middle: its fore end is 75 px (half its
        // length) fore of the cursor, so the cursor goes 75 px aft of where the end should be.
        let side = camera(-std::f32::consts::FRAC_PI_2, 0.0);
        assert_eq!(
            choose(&rocket, &drag, side, Point::new(475.0, 200.0)),
            Some(Choice::Node(Snap::Core(1))),
            "the fore end 25 px along the body is inside the disc"
        );
        let outside = choose(&rocket, &drag, side, Point::new(480.0, 200.0));
        assert!(matches!(outside, Some(Choice::Surface(t)) if t.parent == tube), "the fore end 30 px along the body is outside it: the side: {outside:?}");
        // A small tube (radius 5 px at 100 px/m, 30 px long) keeps the 20 px base: the fore end 18 px aft of the joint is caught.
        let far = Camera { scale: 100.0, ..side };
        assert_eq!(choose(&rocket, &drag, far, Point::new(450.0 + (0.2 - 0.35) * 100.0 + 18.0 + 15.0, 200.0)), Some(Choice::Node(Snap::Core(1))));
    }

    #[test]
    fn looking_along_the_rocket_a_piece_snaps_to_its_far_end() {
        let (rocket, tube) = rocket();
        let drag = drag(PartKind::BodyTube { length: 0.3, diameter: 0.1 });
        // Raised so the tube's top is seen: from the nose looking aft (view direction 88% on the axis) the far end is its aft node
        // (2); from the tail looking forward it is its fore node (1).
        let on_the_tube = Vec3::new(0.0, 0.05, 0.45);
        for (yaw, node) in [(std::f32::consts::PI, 2), (0.0, 1)] {
            let camera = camera(yaw, 0.5);
            assert!(camera.back().z.abs() > FORESHORTENED);
            let at = camera.project(on_the_tube, SIZE);
            assert_eq!(choose(&rocket, &drag, camera, at), Some(Choice::Node(Snap::Core(node))), "yaw {yaw}");
        }
        // Side on nothing changes: the tube's side (the near face, mid-tube) is a pod.
        let side = camera(-std::f32::consts::FRAC_PI_2, 0.0);
        let got = choose(&rocket, &drag, side, side.project(Vec3::new(-0.05, 0.0, 0.45), SIZE));
        assert!(matches!(got, Some(Choice::Surface(t)) if t.parent == tube), "{got:?}");
    }

    #[test]
    fn a_part_that_cannot_stack_at_the_far_end_gets_what_it_got_before() {
        let (rocket, tube) = rocket();
        // A nose cone stacks only at the fore node: the tube's aft node is no choice, so the cursor on the tube makes a pod.
        let drag = drag(PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.1, tip_radius: 0.0 });
        let camera = camera(std::f32::consts::PI, 0.5);
        let choice = choose(&rocket, &drag, camera, camera.project(Vec3::new(0.0, 0.05, 0.45), SIZE));
        assert!(matches!(choice, Some(Choice::Surface(t)) if t.parent == tube), "{choice:?}");
    }

    /// The side view the end tests share: +z to the right, 500 px per metre, node 0 at x = 275, node 1 at 375, node 2 at 625.
    fn side() -> Camera {
        camera(-std::f32::consts::FRAC_PI_2, 0.0)
    }

    /// What the overlay decides for the cursor at `at`.
    fn decide(rocket: &Rocket, drag: &Drag, at: Point) -> Option<(Choice, Option<End>)> {
        let overlay = Overlay { rocket, drag, camera: side(), cursor: None, snap_angle: 0, snap_step: 0.001, margin: None, units: UnitSystem::default() };
        overlay.decide(at, SIZE)
    }

    #[test]
    fn the_free_pose_holds_the_part_by_the_hold_point_under_the_cursor() {
        let camera = side();
        let at = Point::new(500.0, 150.0);
        let pose = free_pose(&camera, SIZE, at, 0.3, 0.1).unwrap();
        // Parallel to the rocket's axis, nose end towards the nose, the right length, the hold point (0.1 behind the fore face) under
        // the cursor.
        assert!((pose.aft - pose.fore - Vec3::Z * 0.3).length() < 1e-6);
        let held = camera.project(pose.fore + Vec3::Z * 0.1, SIZE);
        assert!(held.distance(at) < 1e-3, "{held:?}");
        // A hold point beyond the part is limited to it.
        let past = free_pose(&camera, SIZE, at, 0.3, 5.0).unwrap();
        assert!(camera.project(past.aft, SIZE).distance(at) < 1e-3);
    }

    #[test]
    fn a_tube_held_by_its_middle_snaps_by_its_fore_end_at_the_last_node_with_the_cursor_half_a_tube_away() {
        let (rocket, _) = rocket();
        let drag = drag(PartKind::BodyTube { length: 0.3, diameter: 0.1 });
        // The fore end at the last node (x = 625): the cursor is 75 px, half the tube, aft of it. A tube is a metre-ish thing: the
        // cursor is nowhere near the node.
        let got = decide(&rocket, &drag, Point::new(700.0, 200.0));
        assert_eq!(got, Some((Choice::Node(Snap::Core(2)), Some(End::Fore))));
        // The cursor's own distance to the node is 75 px, well outside the 28.75 px disc.
        assert!(Point::new(700.0, 200.0).distance(Point::new(625.0, 200.0)) > 70.0);
    }

    #[test]
    fn the_aft_end_snaps_at_a_joint_and_at_the_node_ahead_of_the_first_part() {
        let (rocket, _) = rocket();
        let drag = drag(PartKind::BodyTube { length: 0.3, diameter: 0.1 });
        // The aft end at the joint (x = 375): the cursor is 75 px fore of it, over the nose.
        assert_eq!(decide(&rocket, &drag, Point::new(300.0, 200.0)), Some((Choice::Node(Snap::Core(1)), Some(End::Aft))));
        // The fore end at the joint, too, either end serving a joint between parts.
        assert_eq!(decide(&rocket, &drag, Point::new(450.0, 200.0)), Some((Choice::Node(Snap::Core(1)), Some(End::Fore))));
        // The node ahead of the first part (x = 275) takes the aft end: on a rocket with no nose yet, a nose cone (0.2 m, 100 px)
        // held by its middle with its aft end there has the cursor 50 px fore of it.
        let mut bare = Rocket::default();
        bare.insert(0, PartKind::BodyTube { length: 0.5, diameter: 0.1 }).unwrap();
        let nose = self::drag(PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.1, tip_radius: 0.0 });
        assert_eq!(decide(&bare, &nose, Point::new(225.0, 200.0)), Some((Choice::Node(Snap::Core(0)), Some(End::Aft))));
    }

    #[test]
    fn the_aft_end_at_the_node_behind_the_last_part_and_the_fore_end_ahead_of_the_first_do_not_snap() {
        let (rocket, tube) = rocket();
        let drag = drag(PartKind::BodyTube { length: 0.3, diameter: 0.1 });
        // The aft end at the last node (x = 625): the cursor is 75 px fore of it, on the tube. Only a fore end can meet that node.
        let got = decide(&rocket, &drag, Point::new(550.0, 200.0));
        assert!(matches!(got, Some((Choice::Surface(t), None)) if t.parent == tube), "{got:?}");
        // The fore end at the node ahead of the first part (x = 275): the cursor is 75 px aft of it, over the nose. Only an aft end
        // can meet that node (the other nodes are further than their discs).
        let got = decide(&rocket, &drag, Point::new(350.0, 200.0));
        assert!(!matches!(got, Some((Choice::Node(Snap::Core(0)), _))), "{got:?}");
    }

    #[test]
    fn the_cursor_on_a_node_with_both_ends_far_away_does_not_snap() {
        let (rocket, tube) = rocket();
        let drag = drag(PartKind::BodyTube { length: 0.3, diameter: 0.1 });
        // Zoomed in (1500 px/m) the Ø0.1 body is 75 px in radius and the joint's catch disc 86 px. The cursor 60 px above the joint is
        // well inside it (the old rule snapped there), but the tube's ends are 225 px either side of the cursor: no node catches
        // either end, and the cursor is not a snap point. It is over the body's side, which makes a pod.
        let camera = Camera { scale: 1500.0, ..side() };
        let overlay =
            Overlay { rocket: &rocket, drag: &drag, camera, cursor: None, snap_angle: 0, snap_step: 0.001, margin: None, units: UnitSystem::default() };
        let joint = camera.project(Vec3::new(0.0, 0.0, 0.2), SIZE);
        let cursor = Point::new(joint.x, joint.y - 60.0);
        assert!(cursor.distance(joint) < 86.0);
        let got = overlay.decide(cursor, SIZE);
        assert!(matches!(got, Some((Choice::Surface(t), None)) if t.parent == tube || t.parent == rocket.parts()[0].id), "{got:?}");
    }

    #[test]
    fn a_picked_up_part_is_held_where_it_was_grabbed() {
        let (rocket, _) = rocket();
        let mut drag = drag(PartKind::BodyTube { length: 0.3, diameter: 0.1 });
        // Held by the middle (a part from the bin), the fore end is 75 px fore of the cursor: with the cursor at x = 700 it is on the
        // last node (625).
        assert_eq!(decide(&rocket, &drag, Point::new(700.0, 200.0)), Some((Choice::Node(Snap::Core(2)), Some(End::Fore))));
        // Grabbed 0.02 m behind its fore face the fore end is only 10 px fore of the cursor: the same cursor leaves it 65 px past the
        // node (beyond the 48 px of open space that still counts as beside the rocket), so nothing is caught; the cursor 10 px aft of
        // the node catches it by that end.
        drag.hold = Drag::held(&drag.kind, Some(0.02));
        let got = decide(&rocket, &drag, Point::new(700.0, 200.0));
        assert!(!matches!(got, Some((Choice::Node(_), _))), "{got:?}");
        assert_eq!(decide(&rocket, &drag, Point::new(635.0, 200.0)), Some((Choice::Node(Snap::Core(2)), Some(End::Fore))));
        // Grabbed 0.28 m behind it (near its aft end) the fore end is 140 px fore of the cursor.
        drag.hold = Drag::held(&drag.kind, Some(0.28));
        assert_eq!(decide(&rocket, &drag, Point::new(765.0, 200.0)), Some((Choice::Node(Snap::Core(2)), Some(End::Fore))));
        // A grab beyond the part is limited to it.
        assert_eq!(Drag::held(&drag.kind, Some(9.0)), 0.3);
        assert_eq!(Drag::held(&drag.kind, Some(-1.0)), 0.0);
    }

    #[test]
    fn the_decision_does_not_depend_on_where_the_snapped_ghost_is_drawn() {
        let (rocket, _) = rocket();
        let free = drag(PartKind::BodyTube { length: 0.3, diameter: 0.1 });
        // Snapped to the last node (the ghost is drawn there) or free (it is drawn at the cursor), the same cursor decides the same
        // thing: no flicker at the boundary of a catch disc.
        let snapped = Drag { snap: Some(2), ..free.clone() };
        for x in (640..720).step_by(5) {
            let at = Point::new(x as f32, 200.0);
            assert_eq!(decide(&rocket, &free, at), decide(&rocket, &snapped, at), "x = {x}");
        }
    }
}
