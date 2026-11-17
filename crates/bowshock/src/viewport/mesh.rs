//! The model mesh: everything the viewport draws of a [`Rocket`] as one triangle list, grouped into draw ranges.
//!
//! The airframe is a surface of revolution per part; internal parts (inner tubes, rings, bulkheads, couplers,
//! packed recovery gear, shock cord, avionics) and the loaded motors of the active configuration are tubes, cylinders and boxes
//! inside it; fins of every planform, lugs and buttons stand on the outside. Each vertex carries its part's system, layer and
//! its offset in the exploded view, so the view modes (solid, cutaway, X-ray, exploded) are decided in the shader and a mode
//! change never rebuilds the mesh. The mesh does depend on the [`Cut`] (the cap faces of the cutaway are real geometry).

use std::f32::consts::PI;
use std::ops::Range;
use std::sync::atomic::{AtomicU64, Ordering};

use bowshock_model::{InternalKind, MassRole, PartId, PartKind, RadialKind, RadialPart, Rocket, System};
use bowshock_motors::MotorDb;
use glam::{Vec2, Vec3};

use super::assembly::{self, Origin};
use super::radial::{self, Placement};
use super::scene::{GHOST, HOVERED, NORMAL, SELECTED};
use super::solids::{GpuVertex, Sink, Tag, LAYER_CAP, LAYER_CORD, LAYER_FILL, LAYER_INTERNAL, LAYER_RADIAL, LAYER_SHELL};

/// Segments around the axis of the airframe's surfaces.
const SEGMENTS: usize = 64;
/// Segments around the axis of internal parts.
const INNER_SEGMENTS: usize = 32;
/// How far (metres) an internal part is held inside the surface it sits against (the bore it fills, the ring it passes
/// through), so no two surfaces share a cylinder and z-fight.
const INSET: f32 = 0.0002;
/// The thinnest wall (metres) a shell is drawn with: a tube stored with no wall would have its bore in its skin.
const MIN_WALL: f32 = 0.0005;
/// How far (metres) a part's end faces are set inside its ends, so the faces of stacked parts and of whatever sits flush
/// with an end never share a plane.
const END_SET: f32 = 0.00015;

/// The part of the cutaway that is removed: a wedge about the axis, `span` wide, centred on direction `centre` in the XY plane
/// (an angle from +X towards +Y). The default is the left half, which faces the home view and the side-on profile: the cut
/// plane shows the internals in section and the far half-shell stands behind them, lit pale like the design kit's InsideViews.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cut {
    /// Direction of the middle of the removed wedge, radians.
    pub centre: f32,
    /// Width of the removed wedge, radians (half cut is `pi`).
    pub span: f32,
}

impl Default for Cut {
    fn default() -> Self {
        Cut { centre: PI, span: PI }
    }
}

impl Cut {
    /// Smallest and largest wedge the handle can make.
    pub const SPAN_RANGE: (f32, f32) = (PI / 6.0, PI * 1.75);

    /// The two directions bounding the wedge (low edge, high edge), radians.
    pub fn edges(self) -> [f32; 2] {
        [self.centre - self.span / 2.0, self.centre + self.span / 2.0]
    }

    /// Whether the point `xy` (off the axis) is in the removed wedge (the shader's test, for the tests).
    #[cfg(test)]
    pub fn removes(self, xy: Vec2) -> bool {
        let n = xy.normalize_or_zero();
        n != Vec2::ZERO && n.dot(Vec2::new(self.centre.cos(), self.centre.sin())) > (self.span / 2.0).cos()
    }
}

/// What to draw besides the rocket as it is.
#[derive(Debug, Clone, Copy, Default)]
pub struct Marks<'a> {
    /// A part shown inserted at this node index, later parts shifted aft by its length.
    pub ghost: Option<Ghost<'a>>,
    /// A radial part shown placed on an axial part.
    pub ghost_radial: Option<GhostRadial<'a>>,
    /// A part shown inserted at this node of a pod's stack: `(pod, ghost)`.
    pub ghost_pod: Option<(PartId, Ghost<'a>)>,
    /// A part picked up from the rocket and pulled free of it: drawn with what it carries, `at` metres from the origin (the
    /// world position of its fore face), in the ghost state.
    pub free: Option<Free<'a>>,
    /// Parts drawn in the ghost state whatever else they are (what a ghost carries).
    pub ghosted: &'a [PartId],
    /// Axial, radial or internal part under the cursor.
    pub hovered: Option<PartId>,
    /// Axial, radial or internal part selected.
    pub selected: Option<PartId>,
    /// The other parts of a multi-selection (drawn selected like `selected`).
    pub also_selected: &'a [PartId],
    /// The motor database: with it the motors of the active configuration are drawn in their mounts.
    pub motors: Option<&'a MotorDb>,
    /// The cutaway wedge.
    pub cut: Cut,
    /// The part that was just dropped or edited: `(part, outline strength 0..1, height above its place in metres)`. See
    /// [`super::flash`].
    pub flash: Option<(PartId, f32, f32)>,
}

/// A part shown inserted into a stack.
#[derive(Debug, Clone, Copy)]
pub struct Ghost<'a> {
    /// The attach node index.
    pub index: usize,
    /// The part as it would be inserted (matched to its neighbours).
    pub kind: &'a PartKind,
    /// What was picked up with it, if it came from the rocket: it is drawn too, where a drop would put it.
    pub carried: Option<Carried<'a>>,
}

/// A part picked up from the rocket with its radial and internal parts, and the rocket as it was before the drag (for the motors).
#[derive(Debug, Clone, Copy)]
pub struct Carried<'a> {
    pub origin: &'a Origin,
    pub base: Option<&'a Rocket>,
}

/// A carried part pulled free of the rocket.
#[derive(Debug, Clone, Copy)]
pub struct Free<'a> {
    pub carried: Carried<'a>,
    /// World position of the part's fore face on its axis.
    pub at: Vec3,
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

/// World bounds of one part and how far it moves in the fully exploded view.
#[derive(Debug, Clone, Copy)]
pub struct PartBox {
    pub id: PartId,
    pub min: Vec3,
    pub max: Vec3,
    /// Offset at full explosion (the first copy's, for a radial group).
    pub explode: Vec3,
}

/// A dashed line in the exploded view joining an internal part to its station: from `origin + (0, 0, offset.z * t)` on the axis
/// down to `origin + offset * t + (0, radius, 0)`, `t` being the explosion (0 to 1).
#[derive(Debug, Clone, Copy)]
pub struct Leader {
    pub origin: Vec3,
    pub offset: Vec3,
    pub radius: f32,
    /// Half the part's length along the axis (the part spans `origin.z ± half_length` before it moves).
    #[cfg(test)]
    pub half_length: f32,
}

/// The triangle list in three draw ranges: `inner` (internal solids), `caps` (cutaway faces) and `outer` (airframe, fins).
#[derive(Debug)]
pub struct Mesh {
    pub id: u64,
    pub vertices: Vec<GpuVertex>,
    /// Internal parts, drawn first (opaque).
    pub inner: Range<u32>,
    /// Cutaway cap faces (drawn in Cutaway only).
    pub caps: Range<u32>,
    /// Airframe and radial parts, drawn last (alpha-blended in X-ray).
    pub outer: Range<u32>,
    pub min: Vec3,
    pub max: Vec3,
    pub parts: Vec<PartBox>,
    pub leaders: Vec<Leader>,
}

static NEXT_MESH_ID: AtomicU64 = AtomicU64::new(1);

/// The shape of an internal part's cross-section, for cutting it open.
#[derive(Debug, Clone, Copy)]
enum Section {
    Tube { outer: f32, inner: f32 },
    Cylinder(f32),
    Box { half_w: f32, half_h: f32 },
}

impl Section {
    /// Radial extent `(from, to)` of the section along the unit direction `e` in the cut plane.
    fn range(self, e: Vec2) -> (f32, f32) {
        match self {
            Section::Tube { outer, inner } => (inner.min(outer).max(0.0), outer),
            Section::Cylinder(r) => (0.0, r),
            Section::Box { half_w, half_h } => (0.0, (half_w / e.x.abs().max(1e-6)).min(half_h / e.y.abs().max(1e-6))),
        }
    }
}

/// Accumulates the three sinks, the per-part bounds and the leaders.
struct Builder {
    inner: Sink,
    caps: Sink,
    outer: Sink,
    parts: Vec<PartBox>,
    leaders: Vec<Leader>,
    cut: Cut,
    /// The part that settles into place and how far above it still is (metres, along +Y).
    lift: Option<(PartId, f32)>,
}

impl Builder {
    fn bound(&mut self, id: Option<PartId>, from: (usize, usize, usize), explode: Vec3) {
        let Some(id) = id else {
            return;
        };
        // A part that was just released settles from above its place: lift what it just emitted.
        if let Some((_, lift)) = self.lift.filter(|&(lifted, _)| lifted == id) {
            for v in self.inner.vertices[from.0..].iter_mut().chain(&mut self.outer.vertices[from.1..]).chain(&mut self.caps.vertices[from.2..]) {
                v.position[1] += lift;
            }
        }
        let (mut min, mut max) = (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY));
        for v in self.inner.vertices[from.0..].iter().chain(&self.outer.vertices[from.1..]) {
            let p = Vec3::from(v.position);
            min = min.min(p);
            max = max.max(p);
        }
        if min.x > max.x {
            return;
        }
        match self.parts.iter_mut().find(|b| b.id == id) {
            Some(b) => {
                b.min = b.min.min(min);
                b.max = b.max.max(max);
            }
            None => self.parts.push(PartBox { id, min, max, explode }),
        }
    }

    fn marks(&self) -> (usize, usize, usize) {
        (self.inner.vertices.len(), self.outer.vertices.len(), self.caps.vertices.len())
    }

    /// Cap faces of a prism along Z (`z0` to `z1`) with cross-section `section` at both edges of the removed wedge.
    fn cap_section(&mut self, section: Section, z0: f32, z1: f32, system: f32, state: f32) {
        let structure = system == 0.0;
        self.caps.tag = Some(Tag {
            state,
            material: 0.0,
            system,
            layer: if structure { LAYER_CAP } else { LAYER_FILL },
            explode: self.caps.tag.map_or(Vec3::ZERO, |t| t.explode),
        });
        for (e, n) in self.cap_planes() {
            let (t0, t1) = section.range(e);
            if t1 - t0 < 1e-6 {
                continue;
            }
            let p = |z: f32, t: f32| Vec3::new(e.x * t, e.y * t, z);
            self.caps.fan(&[p(z0, t0), p(z1, t0), p(z1, t1), p(z0, t1)], n.extend(0.0));
        }
    }

    /// Cap faces of a shell part: the wall between its outer and inner profile.
    fn cap_shell(&mut self, profile: &[(f32, f32)], inner: &[f32], state: f32) {
        self.caps.tag = Some(Tag { state, material: 0.0, system: 0.0, layer: LAYER_CAP, explode: self.caps.tag.map_or(Vec3::ZERO, |t| t.explode) });
        for (e, n) in self.cap_planes() {
            let p = |z: f32, t: f32| Vec3::new(e.x * t, e.y * t, z);
            for i in 0..profile.len().saturating_sub(1) {
                let ((za, ra), (zb, rb)) = (profile[i], profile[i + 1]);
                if ra <= 0.0 && rb <= 0.0 {
                    continue;
                }
                let (ia, ib) = (inner[i].min(ra).max(0.0), inner[i + 1].min(rb).max(0.0));
                self.caps.fan(&[p(za, ia), p(zb, ib), p(zb, rb), p(za, ra)], n.extend(0.0));
            }
        }
    }

    /// The two half-planes bounding the wedge: the unit direction in the XY plane and the normal facing into the wedge.
    fn cap_planes(&self) -> [(Vec2, Vec2); 2] {
        let [lo, hi] = self.cut.edges();
        let dir = |a: f32| Vec2::new(a.cos(), a.sin());
        let (e0, e1) = (dir(lo), dir(hi));
        [(e0, Vec2::new(-e0.y, e0.x)), (e1, Vec2::new(e1.y, -e1.x))]
    }
}

/// One item of the exploded view's side rows: an internal part or a loaded motor, as the layout sees it.
#[derive(Debug, Clone, Copy)]
pub struct ExplodedItem {
    /// The part's id (a motor carries its mount's id).
    pub id: PartId,
    /// Whether this is the loaded motor of mount `id` rather than the part itself.
    pub motor: bool,
    /// The airframe part it sits in; only items of one parent nest.
    pub parent: PartId,
    /// Axial span in rocket coordinates, including the parent's own exploded shift.
    pub z0: f32,
    pub z1: f32,
    pub r_out: f32,
    /// Bore radius (0 for solids).
    pub r_in: f32,
}

/// Axial clearance (metres) kept between neighbours on one sub-row.
const ROW_Z_GAP: f32 = 0.008;
/// Tolerance when asking whether a part fits in another's bore.
const NEST_TOL: f32 = 1e-6;

/// The lateral offset `y` (negative = below the rocket) of each item in the exploded view, in the order given. A pure function
/// of the items' geometry and ids.
///
/// Items are banded by nesting depth (how long a chain of parts, each inside the bore of the one above, contains it), the
/// shallowest nearest the rocket; inside a band they are packed first-fit by `z0` into sub-rows of axially disjoint items
/// (`ROW_Z_GAP` apart). Sub-rows are stacked outwards, each as tall as its widest item, `max(12 mm, rmax / 4)` apart, the first
/// one `rmax + 30 mm` below the axis.
pub fn exploded_rows(items: &[ExplodedItem], rmax: f32) -> Vec<f32> {
    let n = items.len();
    let overlap = |a: &ExplodedItem, b: &ExplodedItem| a.z0 < b.z1 && b.z0 < a.z1;
    let contains =
        |a: &ExplodedItem, b: &ExplodedItem| a.parent == b.parent && a.r_in > 0.0 && b.r_out < a.r_out && b.r_out <= a.r_in + NEST_TOL && overlap(a, b);
    // Containers are strictly wider, so visiting widest first sees every container before what it holds.
    let mut by_width: Vec<usize> = (0..n).collect();
    by_width.sort_by(|&a, &b| items[b].r_out.total_cmp(&items[a].r_out).then(a.cmp(&b)));
    let mut depth = vec![0usize; n];
    for (k, &b) in by_width.iter().enumerate() {
        depth[b] = by_width[..k].iter().filter(|&&a| contains(&items[a], &items[b])).map(|&a| depth[a] + 1).max().unwrap_or(0);
    }

    let mut rows: Vec<Vec<usize>> = Vec::new();
    for d in 0..=depth.iter().copied().max().unwrap_or(0) {
        let mut band: Vec<usize> = (0..n).filter(|&i| depth[i] == d).collect();
        band.sort_by(|&a, &b| items[a].z0.total_cmp(&items[b].z0).then(items[a].id.cmp(&items[b].id)).then(items[a].motor.cmp(&items[b].motor)));
        let first = rows.len();
        for i in band {
            let it = &items[i];
            let fits = |row: &Vec<usize>| row.iter().all(|&j| it.z0 >= items[j].z1 + ROW_Z_GAP || items[j].z0 >= it.z1 + ROW_Z_GAP);
            match rows[first..].iter().position(fits) {
                Some(r) => rows[first + r].push(i),
                None => rows.push(vec![i]),
            }
        }
    }

    let gap = (0.25 * rmax).max(0.012);
    let mut edge = rmax + 0.03;
    let mut y = vec![0.0; n];
    for row in &rows {
        let height = row.iter().map(|&i| 2.0 * items[i].r_out).fold(0.0f32, f32::max);
        for &i in row {
            y[i] = -(edge + height / 2.0);
        }
        edge += height + gap;
    }
    y
}

/// What an internal part is drawn as: `(length along the axis, outer radius of the drawn body, bore radius)`; `ro`/`ri` are its
/// resolved outer and inner radii.
fn drawn_extent(kind: &InternalKind, ro: f32, ri: f32) -> (f32, f32, f32) {
    match kind {
        InternalKind::InnerTube { thickness, length, .. } => (*length as f32, ro, (ro - *thickness as f32).max(0.0)),
        InternalKind::CenteringRing { thickness, .. } => (*thickness as f32, ro, ri),
        InternalKind::Bulkhead { thickness, .. } => (*thickness as f32, ro, 0.0),
        InternalKind::Coupler { length, .. } => (*length as f32, ro, ri),
        InternalKind::Parachute { packed_length, .. } | InternalKind::ShockCord { packed_length, .. } => (*packed_length as f32, ro, 0.0),
        InternalKind::Streamer { width, .. } => (*width as f32, ro, 0.0),
        InternalKind::MassComponent { role, length, .. } => match role {
            MassRole::Altimeter | MassRole::Battery | MassRole::Tracker => (*length as f32, ro * 0.6, 0.0),
            _ => (*length as f32, ro, 0.0),
        },
    }
}

impl Mesh {
    /// The rocket as a mesh, each part at its station (world z, metres). See [`Marks`] for the ghost, hover and selection and
    /// the motors and cutaway. A ghost that carries what it was picked up with is that part landed on the rocket (as a drop would
    /// land it) with all of it drawn in the ghost state; one pulled free is its own small rocket drawn at the cursor.
    pub fn from_rocket(rocket: &Rocket, marks: Marks) -> Mesh {
        let landed = marks.ghost.and_then(|g| {
            let carried = g.carried?;
            let mut landed = rocket.clone();
            assembly::land_carried(&mut landed, g.index.min(rocket.parts().len()), &carried.origin.part.kind, carried.origin, carried.base)?;
            Some((landed, assembly::carried_ids(carried.origin)))
        });
        let mut mesh = match &landed {
            Some((landed, ids)) => {
                let ghosted = [marks.ghosted, ids.as_slice()].concat();
                Mesh::build(landed, Marks { ghost: None, ghosted: &ghosted, ..marks })
            }
            None => Mesh::build(rocket, marks),
        };
        if let Some(free) = marks.free {
            let ids = assembly::carried_ids(free.carried.origin);
            let sub = assembly::free_rocket(free.carried.origin, free.carried.base);
            let part = Mesh::build(&sub, Marks { ghosted: &ids, motors: marks.motors, cut: marks.cut, ..Marks::default() });
            mesh.append(part, free.at);
        }
        mesh
    }

    /// Adds `other` to the mesh, moved by `shift`: its parts and leaders are not listed (a free ghost is not pickable) and the
    /// bounds stay the rocket's.
    fn append(&mut self, other: Mesh, shift: Vec3) {
        let slice = |m: &Mesh, r: &Range<u32>| m.vertices[r.start as usize..r.end as usize].to_vec();
        let moved = |r: &Range<u32>| {
            slice(&other, r)
                .into_iter()
                .map(|mut v| {
                    v.position = (Vec3::from(v.position) + shift).to_array();
                    v
                })
                .collect::<Vec<_>>()
        };
        let mut vertices = Vec::with_capacity(self.vertices.len() + other.vertices.len());
        let mut ranges = Vec::new();
        for (mine, theirs) in [(&self.inner, &other.inner), (&self.caps, &other.caps), (&self.outer, &other.outer)] {
            let start = vertices.len() as u32;
            vertices.extend(slice(self, mine));
            vertices.extend(moved(theirs));
            ranges.push(start..vertices.len() as u32);
        }
        self.vertices = vertices;
        let mut ranges = ranges.into_iter();
        (self.inner, self.caps, self.outer) = (ranges.next().unwrap_or(0..0), ranges.next().unwrap_or(0..0), ranges.next().unwrap_or(0..0));
    }

    fn build(rocket: &Rocket, marks: Marks) -> Mesh {
        let state_of = |id: PartId| {
            if marks.ghosted.contains(&id) {
                return GHOST;
            }
            let selected = marks.selected == Some(id) || marks.also_selected.contains(&id);
            match marks.flash {
                // The flash: 4 + strength, or 6 + strength on a selected part (which keeps its full outline).
                Some((flashed, level, _)) if flashed == id => (if selected { 6.0 } else { 4.0 }) + level,
                _ if selected => SELECTED,
                _ if marks.hovered == Some(id) => HOVERED,
                _ => NORMAL,
            }
        };
        let mut stack: Vec<(Option<PartId>, &PartKind, f32)> = rocket.parts().iter().map(|p| (Some(p.id), &p.kind, state_of(p.id))).collect();
        if let Some(Ghost { index, kind, .. }) = marks.ghost {
            stack.insert(index.min(stack.len()), (None, kind, GHOST));
        }

        let n = stack.len();
        let rmax = stack.iter().map(|(_, k, _)| k.fore_diameter().max(k.aft_diameter()) as f32 / 2.0).fold(0.0f32, f32::max);
        // Exploded view: sections spread along the axis, centred on the middle of the stack.
        let gap = (rmax * 0.7).max(0.02);
        let dz = |i: usize| (i as f32 - (n as f32 - 1.0) / 2.0) * gap;

        let mut b = Builder {
            inner: Sink::default(),
            caps: Sink::default(),
            outer: Sink::default(),
            parts: Vec::new(),
            leaders: Vec::new(),
            cut: marks.cut,
            lift: marks.flash.map(|(id, _, lift)| (id, lift)).filter(|&(_, lift)| lift > 0.0),
        };
        // (id, station, explode dz, shell thickness) of each axial part as drawn (a ghost shifts the parts behind it).
        let mut axial: Vec<(PartId, f64, f32)> = Vec::new();
        let mut z = 0.0f64;
        for (i, (id, kind, state)) in stack.iter().enumerate() {
            let explode = Vec3::new(0.0, 0.0, dz(i));
            let z0 = z as f32;
            let z1 = z0 + kind.length() as f32;
            let tag = Tag { state: *state, material: 0.0, system: 0.0, layer: LAYER_SHELL, explode };
            b.outer.tag = Some(tag);
            b.caps.tag = Some(tag);
            let start = b.marks();
            let profile: Vec<(f32, f32)> = kind.profile().iter().map(|&(pz, r)| (z0 + pz as f32, r as f32)).collect();
            // The wall as built (a ghost has the default wall of its stock): hollow unless the wall reaches the axis. A wall of
            // nothing (an imported tube with no thickness) is drawn `MIN_WALL` thick, or its bore would lie in its skin.
            let wall =
                (id.and_then(|id| rocket.part(id)).map_or_else(|| kind.default_material().material().default_wall, |p| p.thickness) as f32).max(MIN_WALL);
            let inner: Vec<f32> = profile.iter().map(|&(_, r)| (r - wall).max(0.0)).collect();
            let hollow = inner.iter().any(|&r| r > 0.0);
            b.outer.revolve(&profile, SEGMENTS, Some((z0, z1)));
            if hollow {
                // The bore: the inner surface faces the axis (material 2 marks it for the shader).
                let bore: Vec<(f32, f32)> = profile.iter().zip(&inner).map(|(&(z, _), &r)| (z, r)).rev().collect();
                b.outer.tag = Some(Tag { material: 2.0, ..tag });
                b.outer.revolve(&bore, SEGMENTS, None);
                b.outer.tag = Some(tag);
            }
            // End faces are annuli of the real wall (discs only for solid parts), set a hair inside the part so the stacked
            // parts' faces and whatever sits flush with an end (rings, mount, motor) never share a plane.
            let (first, last) = (profile[0], profile[profile.len() - 1]);
            let (fore_in, aft_in) = (inner[0], inner[inner.len() - 1]);
            if first.1 > 0.0 {
                b.outer.annulus(fore_in, first.1, z0 + END_SET, false, SEGMENTS, None);
            }
            if last.1 > 0.0 {
                b.outer.annulus(aft_in, last.1, z1 - END_SET, true, SEGMENTS, None);
            }
            if let Some(id) = id {
                if rocket.part(*id).is_some() {
                    b.cap_shell(&profile, &inner, *state);
                    // Shoulders: tubes of the part's material reaching into the neighbouring airframe, seen with the shell
                    // cut away or see-through.
                    for aft in [false, true] {
                        let (Some(s), Some(d)) = (rocket.shoulder(*id, aft).filter(|s| s.is_some()), rocket.shoulder_diameter(*id, aft)) else {
                            continue;
                        };
                        let (ro, len) = (d as f32 / 2.0, s.length as f32);
                        let ri = (ro - s.thickness as f32).max(0.0);
                        // Set a hair inside the tube it enters, whose bore is now drawn.
                        let ro = ro - INSET;
                        let (za, zb) = if aft { (z1, z1 + len) } else { (z0 - len, z0) };
                        b.inner.tag = Some(Tag { state: *state, material: 0.0, system: 0.0, layer: LAYER_INTERNAL, explode });
                        b.inner.tube(ro, ri, za, zb, INNER_SEGMENTS);
                        b.cap_section(Section::Tube { outer: ro, inner: ri }, za, zb, 0.0, *state);
                    }
                }
                axial.push((*id, z, explode.z));
            }
            b.bound(*id, start, explode);
            z += kind.length();
        }
        let z_of = |id: PartId| axial.iter().find(|(a, ..)| *a == id).map(|&(_, z, dz)| (z, dz));

        // Radial parts.
        let place = |b: &mut Builder, radial: &RadialPart, state: f32| {
            let (Some((parent_z, parent_dz)), Some(base)) = (z_of(radial.parent), rocket.radial_base_radius(radial)) else {
                return;
            };
            let pull = if radial.kind.is_fin() { radial.kind.fin_span().unwrap_or(0.04) as f32 * 0.6 } else { 0.03 };
            for (index, angle) in radial.angles().into_iter().enumerate() {
                // Each copy stands as it is: the group's values with its own overrides (a canted fin B) on top.
                let own = radial.effective(index);
                let (u, _) = radial::dirs(angle as f32);
                let explode = Vec3::Z * parent_dz + u * pull;
                b.outer.tag = Some(Tag { state, material: 1.0, system: 1.0, layer: LAYER_RADIAL, explode });
                let at = Placement { z0: (parent_z + radial.offset) as f32, base: base as f32, angle: angle as f32, cant: own.cant as f32 };
                let start = b.marks();
                match &own.kind {
                    RadialKind::EllipticalFin { thickness, .. } | RadialKind::FreeformFin { thickness, .. } => {
                        let outline: Vec<(f32, f32)> = own.kind.planform().unwrap_or_default().iter().map(|&(pz, h)| (pz as f32, h as f32)).collect();
                        b.outer.plate(&outline, at.z0, at.base, at.angle, at.cant, *thickness as f32);
                    }
                    kind => {
                        let mut tmp = Vec::new();
                        radial::emit(&mut tmp, kind, at, state);
                        b.outer.adopt(&tmp);
                    }
                }
                // A through-the-wall tab: the root plate carried on down through the wall (hidden by the shell in Solid; the
                // cutaway and X-ray show it reaching the mount).
                if let (Some(tab), Some(thickness)) = (radial.tab.filter(|_| own.kind.is_fin()), own.kind.thickness()) {
                    let (fore, aft) = (tab.position as f32, (tab.position + tab.length) as f32);
                    let depth = (tab.height as f32).min(at.base);
                    b.outer.plate(&[(fore, 0.0), (aft, 0.0), (aft, -depth), (fore, -depth)], at.z0, at.base, at.angle, at.cant, thickness as f32);
                }
                if radial.fillet_radius > 0.0 && own.kind.is_fin() {
                    b.outer.fillet(
                        at.z0,
                        own.kind.extent() as f32,
                        at.base,
                        at.angle,
                        at.cant,
                        own.kind.thickness().unwrap_or(0.0) as f32,
                        radial.fillet_radius as f32,
                    );
                }
                b.bound(Some(radial.id), start, explode);
            }
        };
        for radial in rocket.radials() {
            place(&mut b, radial, state_of(radial.id));
        }
        if let Some(g) = marks.ghost_radial {
            let offset = rocket.clamp_offset(g.parent, g.offset, g.kind).unwrap_or(0.0);
            place(&mut b, &RadialPart::new(PartId::MAX, g.kind.clone(), g.parent, offset, g.angle, g.count), GHOST);
        }
        // Pods: each copy of each pod is the pod's own mesh carried to its place (see `pods`).
        super::pods::append(&mut b.outer.vertices, &mut b.parts, rocket, &marks, &z_of);

        // Internal parts and loaded motors: every one is an exploded item, laid out together (see `exploded_rows`) before any is
        // emitted, so a part's row depends only on the rocket's geometry.
        struct Placed<'a> {
            part: &'a bowshock_model::InternalPart,
            parent_dz: f32,
            z0: f32,
            ro: f32,
            ri: f32,
            len: f32,
            top: f32,
            inner: f32,
        }
        let placed: Vec<Placed> = rocket
            .internals()
            .iter()
            .filter_map(|part| {
                let ((parent_z, parent_dz), res) = (z_of(part.parent)?, rocket.resolved(part.id)?);
                let z0 = (parent_z + part.offset) as f32;
                let (od, id_) = (res.outer_diameter as f32, res.inner_diameter as f32);
                // Drawn a hair inside the parent's bore as it is drawn: a part as wide as the bore (a body tube that is its own
                // motor mount, a coupler) would share the shell's surface and flicker against it.
                let bore = rocket.part(part.parent).map_or(f32::INFINITY, |p| p.kind.radius_at(part.offset) as f32 - (p.thickness as f32).max(MIN_WALL));
                let ro = (od / 2.0).min((bore - INSET).max(0.0));
                let ri = (id_ / 2.0).min(ro);
                let (len, top, hollow) = drawn_extent(&part.kind, ro, ri);
                Some(Placed { part, parent_dz, z0, ro, ri, len, top, inner: hollow })
            })
            .collect();
        struct Loaded {
            mount: PartId,
            parent: PartId,
            parent_dz: f32,
            fore: f32,
            aft: f32,
            radius: f32,
        }
        let mut loaded: Vec<Loaded> = Vec::new();
        if let Some(db) = marks.motors {
            for slot in &rocket.active_configuration().motors {
                let (Some(motor_ref), Some(mount)) = (&slot.motor, rocket.internal(slot.mount)) else {
                    continue;
                };
                let (Some(motor), Some((parent_z, parent_dz)), InternalKind::InnerTube { length, overhang, outer_diameter, thickness, .. }) =
                    (bowshock_sim::from_model::find_motor(db, motor_ref), z_of(mount.parent), &mount.kind)
                else {
                    continue;
                };
                let bore = (outer_diameter - 2.0 * thickness).max(0.0);
                let radius = (motor.diameter.min(if bore > 0.0 { bore } else { motor.diameter }) / 2.0) as f32;
                let aft = (parent_z + mount.offset + length + overhang) as f32;
                loaded.push(Loaded { mount: mount.id, parent: mount.parent, parent_dz, fore: aft - motor.length as f32, aft, radius });
            }
        }
        let items: Vec<ExplodedItem> = placed
            .iter()
            .map(|p| ExplodedItem {
                id: p.part.id,
                motor: false,
                parent: p.part.parent,
                z0: p.z0 + p.parent_dz,
                z1: p.z0 + p.len + p.parent_dz,
                r_out: p.top,
                r_in: p.inner,
            })
            .chain(loaded.iter().map(|m| ExplodedItem {
                id: m.mount,
                motor: true,
                parent: m.parent,
                z0: m.fore + m.parent_dz,
                z1: m.aft + m.parent_dz,
                r_out: m.radius,
                r_in: 0.0,
            }))
            .collect();
        let ys = exploded_rows(&items, rmax);

        for (p, &y) in placed.iter().zip(&ys) {
            let Placed { part, parent_dz, z0, ro, ri, len, .. } = *p;
            let state = state_of(part.id);
            let sys = system_slot(part.kind.system());
            let put = |b: &mut Builder, layer: f32, radius: f32, f: &dyn Fn(&mut Sink), section: Option<(Section, f32, f32)>| {
                let explode = Vec3::new(0.0, y, parent_dz);
                b.inner.tag = Some(Tag { state, material: 0.0, system: sys, layer, explode });
                b.caps.tag = Some(Tag { state, material: 0.0, system: sys, layer: LAYER_CAP, explode });
                let start = b.marks();
                f(&mut b.inner);
                if let Some((section, a, c)) = section {
                    b.cap_section(section, a, c, sys, state);
                }
                b.bound(Some(part.id), start, explode);
                let mid = z0 + len / 2.0;
                b.leaders.push(Leader {
                    origin: Vec3::new(0.0, 0.0, mid),
                    offset: explode,
                    radius,
                    #[cfg(test)]
                    half_length: len / 2.0,
                });
            };
            match &part.kind {
                InternalKind::InnerTube { thickness, length, .. } => {
                    let wall = *thickness as f32;
                    let l = *length as f32;
                    let ri_t = (ro - wall).max(0.0);
                    put(
                        &mut b,
                        LAYER_INTERNAL,
                        ro,
                        &|s| s.tube(ro, ri_t, z0, z0 + l, INNER_SEGMENTS),
                        Some((Section::Tube { outer: ro, inner: ri_t }, z0, z0 + l)),
                    );
                }
                InternalKind::CenteringRing { thickness, .. } => {
                    let t = *thickness as f32;
                    put(
                        &mut b,
                        LAYER_INTERNAL,
                        ro,
                        &|s| s.tube(ro - INSET, ri + INSET, z0, z0 + t, INNER_SEGMENTS),
                        Some((Section::Tube { outer: ro, inner: ri }, z0, z0 + t)),
                    );
                }
                InternalKind::Bulkhead { thickness, .. } => {
                    let t = *thickness as f32;
                    put(&mut b, LAYER_INTERNAL, ro, &|s| s.tube(ro, 0.0, z0, z0 + t, INNER_SEGMENTS), Some((Section::Cylinder(ro), z0, z0 + t)));
                }
                InternalKind::Coupler { length, .. } => {
                    let l = *length as f32;
                    put(
                        &mut b,
                        LAYER_INTERNAL,
                        ro,
                        &|s| s.tube(ro - INSET, ri + INSET, z0, z0 + l, INNER_SEGMENTS),
                        Some((Section::Tube { outer: ro, inner: ri }, z0, z0 + l)),
                    );
                }
                InternalKind::Parachute { packed_length, .. } => {
                    let l = *packed_length as f32;
                    put(&mut b, LAYER_INTERNAL, ro, &|s| s.tube(ro, 0.0, z0, z0 + l, INNER_SEGMENTS), Some((Section::Cylinder(ro), z0, z0 + l)));
                }
                InternalKind::Streamer { width, .. } => {
                    let l = *width as f32;
                    put(&mut b, LAYER_INTERNAL, ro, &|s| s.tube(ro, 0.0, z0, z0 + l, INNER_SEGMENTS), Some((Section::Cylinder(ro), z0, z0 + l)));
                }
                InternalKind::ShockCord { packed_length, .. } => {
                    let l = *packed_length as f32;
                    let amp = ro * 0.7;
                    put(
                        &mut b,
                        LAYER_CORD,
                        ro,
                        &|s| {
                            const ZIGS: usize = 16;
                            let pt = |i: usize| Vec3::new(0.0, if i.is_multiple_of(2) { -amp } else { amp }, z0 + l * i as f32 / ZIGS as f32);
                            for i in 0..ZIGS {
                                s.segment(pt(i), pt(i + 1), 0.0015);
                            }
                        },
                        None,
                    );
                }
                InternalKind::MassComponent { role, length, .. } => {
                    let l = *length as f32;
                    match role {
                        MassRole::Altimeter | MassRole::Battery | MassRole::Tracker => {
                            let (hw, hh) = (ro, ro * 0.6);
                            put(&mut b, LAYER_INTERNAL, hh, &|s| s.cuboid(hw, hh, z0, z0 + l), Some((Section::Box { half_w: hw, half_h: hh }, z0, z0 + l)));
                        }
                        _ => put(&mut b, LAYER_INTERNAL, ro, &|s| s.tube(ro, 0.0, z0, z0 + l, INNER_SEGMENTS), Some((Section::Cylinder(ro), z0, z0 + l))),
                    }
                }
            }
        }

        // The loaded motors of the active configuration.
        for (m, &y) in loaded.iter().zip(&ys[placed.len()..]) {
            let Loaded { mount, parent_dz, fore, aft, radius, .. } = *m;
            let state = state_of(mount);
            let explode = Vec3::new(0.0, y, parent_dz);
            b.inner.tag = Some(Tag { state, material: 0.0, system: 2.0, layer: LAYER_INTERNAL, explode });
            b.caps.tag = Some(Tag { state, material: 0.0, system: 2.0, layer: LAYER_FILL, explode });
            // The case sits a hair inside the bore, set back from both ends, so it shares no surface with the mount or its
            // rings.
            b.inner.tube(radius - INSET, 0.0, fore + END_SET, aft - END_SET, INNER_SEGMENTS);
            b.cap_section(Section::Cylinder(radius), fore, aft, 2.0, state);
            b.leaders.push(Leader {
                origin: Vec3::new(0.0, 0.0, (fore + aft) / 2.0),
                offset: explode,
                radius,
                #[cfg(test)]
                half_length: (aft - fore) / 2.0,
            });
        }

        let (n_inner, n_caps, n_outer) = (b.inner.vertices.len() as u32, b.caps.vertices.len() as u32, b.outer.vertices.len() as u32);
        let mut vertices = b.inner.vertices;
        vertices.extend(b.caps.vertices);
        vertices.extend(b.outer.vertices);
        let (mut min, mut max) = (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY));
        for v in &vertices {
            let p = Vec3::from(v.position);
            min = min.min(p);
            max = max.max(p);
        }
        if vertices.is_empty() {
            (min, max) = (Vec3::ZERO, Vec3::ZERO);
        }
        Mesh {
            id: NEXT_MESH_ID.fetch_add(1, Ordering::Relaxed),
            vertices,
            inner: 0..n_inner,
            caps: n_inner..n_inner + n_caps,
            outer: n_inner + n_caps..n_inner + n_caps + n_outer,
            min,
            max,
            parts: b.parts,
            leaders: b.leaders,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty()
    }

    /// World bounds of part `id`.
    pub fn part_bounds(&self, id: PartId) -> Option<&PartBox> {
        self.parts.iter().find(|p| p.id == id)
    }
}

/// The shader's system colour slot: 0 structure, 1 aero, 2 propulsion, 3 recovery, 4 avionics.
pub fn system_slot(system: System) -> f32 {
    match system {
        System::Structure => 0.0,
        System::Aero => 1.0,
        System::Propulsion => 2.0,
        System::Recovery => 3.0,
        System::Avionics => 4.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{NoseShape, PartKind, RadialKind};
    use std::f32::consts::FRAC_PI_2;

    fn nose() -> PartKind {
        PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.1, tip_radius: 0.0 }
    }

    fn tube() -> PartKind {
        PartKind::BodyTube { length: 0.5, diameter: 0.1 }
    }

    fn shell(mesh: &Mesh) -> &[GpuVertex] {
        &mesh.vertices[mesh.outer.start as usize..mesh.outer.end as usize]
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
        let aft = Mesh::from_rocket(&rocket, Marks { ghost: Some(Ghost { index: 1, kind: &tube(), carried: None }), ..Marks::default() });
        assert!((aft.max.z - 0.7).abs() < 1e-6);
        assert!(aft.vertices.iter().any(|v| v.state == GHOST));
        let all_ghost_start = Mesh::from_rocket(&rocket, Marks { ghost: Some(Ghost { index: 0, kind: &tube(), carried: None }), ..Marks::default() });
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
        // The fin vertices use the fin colour; the airframe's own vertices (shell material 0, bore material 2) are unchanged.
        assert_eq!(mesh.vertices.iter().filter(|v| v.material != 1.0).count(), bare.vertices.len());
        assert!(mesh.vertices.iter().any(|v| v.material == 1.0));
        // Fins reach `span` beyond the tube's 0.05 radius, at the up/down and sideways angles.
        assert!((mesh.max.y - 0.13).abs() < 1e-5 && (mesh.min.y + 0.13).abs() < 1e-5 && (mesh.max.x - 0.13).abs() < 1e-5);
        // Selecting the group marks only its vertices.
        let marked = Mesh::from_rocket(&rocket, Marks { selected: Some(id), ..Marks::default() });
        assert!(marked.vertices.iter().all(|v| (v.material == 1.0) == (v.state == SELECTED)));
        // A ghost axial part ahead of the tube pushes the fins aft with their parent.
        let shifted = Mesh::from_rocket(&rocket, Marks { ghost: Some(Ghost { index: 1, kind: &tube(), carried: None }), ..Marks::default() });
        assert!((shifted.max.z - (mesh.max.z + 0.5)).abs() < 1e-5);
        // A ghost radial part draws all its copies in the ghost state, without being in the rocket.
        let lug = RadialKind::LaunchLug { length: 0.05, diameter: 0.01 };
        let ghost = GhostRadial { kind: &lug, parent: tube_id, offset: 0.1, angle: 0.0, count: 3 };
        let with = Mesh::from_rocket(&rocket, Marks { ghost_radial: Some(ghost), ..Marks::default() });
        assert!(with.vertices.iter().filter(|v| v.state == GHOST).count() > 0 && rocket.radials().len() == 1);
    }

    #[test]
    fn elliptical_and_freeform_fins_are_meshed() {
        let mut rocket = Rocket::default();
        rocket.insert(0, nose()).unwrap();
        let tube_id = rocket.insert(1, tube()).unwrap();
        let elliptical = rocket.attach_radial(tube_id, 0.3, 0.0, 3, RadialKind::EllipticalFin { root_chord: 0.12, span: 0.07, thickness: 0.003 }).unwrap();
        let freeform = rocket
            .attach_radial(
                tube_id,
                0.1,
                0.0,
                2,
                RadialKind::FreeformFin { points: vec![(0.0, 0.0), (0.02, 0.06), (0.05, 0.03), (0.08, 0.06), (0.1, 0.0)], thickness: 0.003 },
            )
            .unwrap();
        let mesh = Mesh::from_rocket(&rocket, Marks::default());
        for id in [elliptical, freeform] {
            let b = mesh.part_bounds(id).expect("meshed");
            assert!(b.max.y > 0.05 + 0.06 - 1e-3 || b.max.x > 0.1, "fin reaches out: {b:?}");
        }
        // The elliptical fin's tip is `span` out from the surface.
        let e = mesh.part_bounds(elliptical).unwrap();
        assert!((e.max.y - (0.05 + 0.07)).abs() < 1e-3);
    }

    #[test]
    fn internals_are_inside_the_shell_and_have_system_colours_and_caps() {
        let mut rocket = Rocket::default();
        rocket.insert(0, nose()).unwrap();
        let tube_id = rocket.insert(1, tube()).unwrap();
        let mount = rocket
            .attach_internal(tube_id, 0.3, InternalKind::InnerTube { outer_diameter: 0.038, thickness: 0.002, length: 0.2, motor_mount: true, overhang: 0.0 })
            .unwrap();
        let chute = rocket
            .attach_internal(
                tube_id,
                0.02,
                InternalKind::Parachute {
                    diameter: 0.6,
                    cd: 0.8,
                    line_count: 6,
                    line_length: 0.5,
                    line_material: bowshock_model::MaterialId::BRAIDED_NYLON_3MM,
                    packed_length: 0.12,
                    packed_diameter: None,
                    deploy: Default::default(),
                },
            )
            .unwrap();
        let mesh = Mesh::from_rocket(&rocket, Marks::default());
        assert!(mesh.inner.end > 0);
        let inner = &mesh.vertices[mesh.inner.start as usize..mesh.inner.end as usize];
        assert!(inner.iter().any(|v| v.system == 2.0) && inner.iter().any(|v| v.system == 3.0));
        assert!(inner.iter().all(|v| Vec3::new(v.position[0], v.position[1], 0.0).length() <= 0.05 + 1e-6));
        // Propulsion caps are flat fills; the shell's wall caps are hatched.
        let caps = &mesh.vertices[mesh.caps.start as usize..mesh.caps.end as usize];
        assert!(caps.iter().any(|v| v.layer == LAYER_FILL && v.system == 2.0) && caps.iter().any(|v| v.layer == LAYER_CAP && v.system == 0.0));
        // The recovery gear sits in the exploded view's first row, the shell sections spread along the axis.
        assert!(mesh.part_bounds(chute).unwrap().explode.y < 0.0 && mesh.part_bounds(mount).unwrap().explode.y < 0.0);
        assert!(shell(&mesh).iter().any(|v| v.explode[2] != 0.0));
        assert_eq!(mesh.leaders.len(), 2);
    }

    #[test]
    fn the_loaded_motor_fills_its_mount_and_sticks_out_by_the_overhang() {
        let mut rocket = Rocket::default();
        rocket.insert(0, nose()).unwrap();
        let tube_id = rocket.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.0762 }).unwrap();
        let mount = rocket
            .attach_internal(
                tube_id,
                0.22,
                InternalKind::InnerTube { outer_diameter: 0.0545, thickness: 0.002, length: 0.28, motor_mount: true, overhang: 0.02 },
            )
            .unwrap();
        let db = MotorDb::bundled();
        let motor = db.find("H128W").expect("bundled");
        let config = rocket.active_config_id();
        rocket.set_motor(config, mount, Some(bowshock_model::MotorRef::new("H128W", "AeroTech"))).unwrap();
        let with = Mesh::from_rocket(&rocket, Marks { motors: Some(&db), ..Marks::default() });
        let without = Mesh::from_rocket(&rocket, Marks::default());
        // Aft end: tube aft (0.2 + 0.22 + 0.28 = 0.70, flush with the airframe) plus the 20 mm overhang.
        assert!((with.max.z - 0.72).abs() < 1e-5 && (without.max.z - 0.70).abs() < 1e-5);
        let case = with.vertices.iter().filter(|v| v.system == 2.0 && v.layer == LAYER_INTERNAL).map(|v| v.position[2]).fold(f32::MAX, f32::min);
        assert!(case <= 0.72 - motor.length as f32 + 1e-5);
    }

    #[test]
    fn the_cut_removes_a_wedge_and_caps_its_edges() {
        let cut = Cut::default();
        assert!(cut.removes(Vec2::new(-1.0, 0.0)) && !cut.removes(Vec2::new(1.0, 0.0)));
        let quarter = Cut { span: FRAC_PI_2, ..cut };
        assert!(quarter.removes(Vec2::new(-1.0, 0.2)) && !quarter.removes(Vec2::new(0.0, 1.0)));
        // Cap faces lie in the planes of the wedge's edges and face into the wedge.
        let mut rocket = Rocket::default();
        rocket.insert(0, nose()).unwrap();
        rocket.insert(1, tube()).unwrap();
        let mesh = Mesh::from_rocket(&rocket, Marks { cut: quarter, ..Marks::default() });
        let caps = &mesh.vertices[mesh.caps.start as usize..mesh.caps.end as usize];
        assert!(!caps.is_empty());
        let [lo, hi] = quarter.edges();
        for v in caps {
            let a = v.position[1].atan2(v.position[0]);
            let on_edge =
                |edge: f32| (Vec2::new(a.cos(), a.sin()) - Vec2::new(edge.cos(), edge.sin())).length() < 1e-4 || (v.position[0].hypot(v.position[1])) < 1e-9;
            assert!(on_edge(lo) || on_edge(hi), "{:?}", v.position);
        }
    }

    fn item(id: PartId, motor: bool, z: (f32, f32), r_out: f32, r_in: f32) -> ExplodedItem {
        ExplodedItem { id, motor, parent: 100, z0: z.0, z1: z.1, r_out, r_in }
    }

    /// Whether the rectangles `(z0, z1, y0, y1)` overlap (touching is not overlapping).
    fn rects_overlap(a: (f32, f32, f32, f32), b: (f32, f32, f32, f32)) -> bool {
        const EPS: f32 = 1e-6;
        a.0 < b.1 - EPS && b.0 < a.1 - EPS && a.2 < b.3 - EPS && b.2 < a.3 - EPS
    }

    #[test]
    fn a_mount_its_rings_and_its_motor_come_out_in_three_bands() {
        let items = [
            item(1, false, (0.5, 0.8), 0.02, 0.019),
            item(2, false, (0.52, 0.525), 0.027, 0.02),
            item(3, false, (0.76, 0.765), 0.027, 0.02),
            item(1, true, (0.55, 0.8), 0.019, 0.0),
        ];
        let y = exploded_rows(&items, 0.05);
        // Both rings share the nearest sub-row; the tube is a band further out, the motor another.
        assert_eq!(y[1], y[2]);
        assert!(y[1] > y[0] && y[0] > y[3], "{y:?}");
        // The first row's top edge is 30 mm clear of the airframe; rows are separated by max(12 mm, rmax / 4).
        assert!((y[1] - -(0.05 + 0.03 + 0.027)).abs() < 1e-6);
        assert!((y[0] - -(0.05 + 0.03 + 0.054 + 0.0125 + 0.02)).abs() < 1e-6);
        let rect = |i: usize| (items[i].z0, items[i].z1, y[i] - items[i].r_out, y[i] + items[i].r_out);
        for i in 0..items.len() {
            for j in i + 1..items.len() {
                assert!(!rects_overlap(rect(i), rect(j)), "{i} and {j}");
            }
        }
    }

    #[test]
    fn chutes_end_to_end_share_a_sub_row_and_overlapping_ones_do_not() {
        let items = [item(1, false, (0.1, 0.2), 0.03, 0.0), item(2, false, (0.21, 0.3), 0.03, 0.0), item(3, false, (0.15, 0.25), 0.03, 0.0)];
        let y = exploded_rows(&items, 0.05);
        assert_eq!(y[0], y[1]);
        assert!(y[2] < y[0]);
    }

    /// Every exploded item's rectangle `(z0, z1, y0, y1)` in the fully exploded view, from the leaders.
    fn exploded_rects(mesh: &Mesh) -> Vec<(f32, f32, f32, f32)> {
        mesh.leaders
            .iter()
            .map(|l| {
                let z = l.origin.z + l.offset.z;
                (z - l.half_length, z + l.half_length, l.offset.y - l.radius, l.offset.y + l.radius)
            })
            .collect()
    }

    fn assert_separate_and_clear_of_the_airframe(rocket: &Rocket) {
        let db = MotorDb::bundled();
        let mesh = Mesh::from_rocket(rocket, Marks { motors: Some(&db), ..Marks::default() });
        let rmax = rocket.parts().iter().map(|p| p.kind.fore_diameter().max(p.kind.aft_diameter()) as f32 / 2.0).fold(0.0f32, f32::max);
        let rects = exploded_rects(&mesh);
        assert!(!rects.is_empty());
        for (i, a) in rects.iter().enumerate() {
            assert!(a.3 < -rmax, "item {i} reaches the airframe band: {a:?}");
            for (j, b) in rects.iter().enumerate().skip(i + 1) {
                assert!(!rects_overlap(*a, *b), "items {i} and {j} overlap: {a:?} {b:?}");
            }
        }
    }

    #[test]
    fn the_example_rocket_explodes_into_separate_parts() {
        let rocket = Rocket::from_bytes(include_bytes!("../../assets/examples/hpr_l1_cert.bowshock")).unwrap();
        assert_separate_and_clear_of_the_airframe(&rocket);
    }

    #[test]
    fn a_coupler_with_a_bulkhead_and_a_streamer_inside_explodes_into_separate_parts() {
        let mut rocket = Rocket::default();
        rocket.insert(0, nose()).unwrap();
        let tube_id = rocket.insert(1, tube()).unwrap();
        rocket.attach_internal(tube_id, 0.1, InternalKind::Coupler { outer_diameter: Some(0.09), thickness: 0.002, length: 0.2 }).unwrap();
        rocket.attach_internal(tube_id, 0.15, InternalKind::Bulkhead { diameter: Some(0.08), thickness: 0.005 }).unwrap();
        let deploy = Default::default();
        rocket.attach_internal(tube_id, 0.12, InternalKind::Streamer { length: 0.3, width: 0.05, cd: 0.8, deploy }).unwrap();
        assert_separate_and_clear_of_the_airframe(&rocket);
    }

    #[test]
    fn the_layout_ignores_the_selection() {
        let rocket = Rocket::from_bytes(include_bytes!("../../assets/examples/hpr_l1_cert.bowshock")).unwrap();
        let db = MotorDb::bundled();
        let plain = Mesh::from_rocket(&rocket, Marks { motors: Some(&db), ..Marks::default() });
        let id = rocket.internals().first().expect("the example has internal parts").id;
        let picked = Mesh::from_rocket(&rocket, Marks { motors: Some(&db), selected: Some(id), hovered: Some(id), ..Marks::default() });
        let offsets = |m: &Mesh| m.leaders.iter().map(|l| l.offset).collect::<Vec<_>>();
        assert_eq!(offsets(&plain), offsets(&picked));
    }

    /// Picks up the airframe part of `rocket` carrying the most, as a drag does.
    fn pick_up(rocket: &Rocket) -> (Rocket, Origin, usize) {
        let id =
            rocket.parts().iter().max_by_key(|p| rocket.radials().iter().filter(|r| r.parent == p.id).count() + rocket.internals_of(p.id).count()).unwrap().id;
        let index = rocket.index_of(id).unwrap();
        let mut rest = rocket.clone();
        let (radials, internals) = (rest.take_radials(id), rest.take_internals(id));
        let part = rest.remove(id).unwrap();
        (rest, Origin { part, radials, internals }, index)
    }

    #[test]
    fn a_carried_ghost_is_the_part_landed_with_all_it_carries_in_every_layer() {
        let rocket = Rocket::from_bytes(include_bytes!("../../assets/examples/hpr_l1_cert.bowshock")).unwrap();
        let db = MotorDb::bundled();
        let whole = Mesh::from_rocket(&rocket, Marks { motors: Some(&db), ..Marks::default() });
        let (rest, origin, index) = pick_up(&rocket);
        assert!(!origin.radials.is_empty() && !origin.internals.is_empty(), "the example's busiest part carries fins and internals");
        let (kind, base) = (origin.part.kind.clone(), Some(&rocket));
        let ghost = Mesh::from_rocket(
            &rest,
            Marks { ghost: Some(Ghost { index, kind: &kind, carried: Some(Carried { origin: &origin, base }) }), motors: Some(&db), ..Marks::default() },
        );
        // Landed where it was: the same geometry as the whole rocket, the part and all it carries in the ghost state.
        assert_eq!(ghost.vertices.len(), whole.vertices.len());
        for layer in [LAYER_SHELL, LAYER_RADIAL, LAYER_INTERNAL] {
            let count = |m: &Mesh| m.vertices.iter().filter(|v| v.layer == layer).count();
            assert_eq!(count(&ghost), count(&whole), "layer {layer}");
        }
        assert!(ghost.vertices.iter().any(|v| v.layer == LAYER_RADIAL && v.state == GHOST));
        assert!(ghost.vertices.iter().any(|v| v.layer == LAYER_INTERNAL && v.state == GHOST));
        // Every internal item (and loaded motor) is laid out in the exploded view as in the whole rocket.
        assert_eq!(ghost.leaders.len(), whole.leaders.len());
    }

    #[test]
    fn a_carried_ghost_moves_with_its_node_and_a_free_one_with_its_position() {
        let rocket = Rocket::from_bytes(include_bytes!("../../assets/examples/hpr_l1_cert.bowshock")).unwrap();
        let (rest, origin, index) = pick_up(&rocket);
        let kind = origin.part.kind.clone();
        let carried = Carried { origin: &origin, base: None };
        let fin_z = |m: &Mesh, state: Option<f32>| {
            m.vertices.iter().filter(|v| v.layer == LAYER_RADIAL && state.is_none_or(|s| v.state == s)).map(|v| v.position[2]).fold(f32::MAX, f32::min)
        };
        let at = |node: usize| Mesh::from_rocket(&rest, Marks { ghost: Some(Ghost { index: node, kind: &kind, carried: Some(carried) }), ..Marks::default() });
        // At the node it came from the fins stand where they did; at a node that cannot take the part the ghost is the bare part.
        let whole = Mesh::from_rocket(&rocket, Marks::default());
        let here = at(index);
        assert!(fin_z(&here, Some(GHOST)) < f32::MAX && (fin_z(&here, None) - fin_z(&whole, None)).abs() < 1e-5, "the fins stand at the ghost's station");
        let refused = at(0);
        assert_eq!(fin_z(&refused, Some(GHOST)), f32::MAX, "a node the part cannot take shows no fins");
        // Free: the same part and fins, moved wholesale, and not listed as parts of the rocket.
        let bare = Mesh::from_rocket(&rest, Marks::default());
        let shift = Vec3::new(0.3, 0.2, 1.0);
        let free = Mesh::from_rocket(&rest, Marks { free: Some(Free { carried, at: shift }), ..Marks::default() });
        assert_eq!(free.parts.len(), bare.parts.len());
        let ghosted = |m: &Mesh| m.vertices.iter().filter(|v| v.state == GHOST).count();
        assert_eq!(ghosted(&bare), 0);
        assert!(free.vertices.iter().any(|v| v.state == GHOST && v.layer == LAYER_RADIAL));
        assert_eq!(free.vertices.len() - bare.vertices.len(), ghosted(&free));
        let free_min_z = free.vertices.iter().filter(|v| v.state == GHOST).map(|v| v.position[2]).fold(f32::MAX, f32::min);
        assert!(free_min_z >= shift.z - 1e-4);
    }
}
