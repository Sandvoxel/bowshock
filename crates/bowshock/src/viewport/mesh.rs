//! The model mesh: everything the viewport draws of a [`Rocket`] as one triangle list, grouped into draw ranges.
//!
//! The airframe is a surface of revolution per part; internal parts (inner tubes, rings, bulkheads, couplers, engine blocks,
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

use super::radial::{self, Placement};
use super::scene::{GHOST, HOVERED, NORMAL, SELECTED};
use super::solids::{GpuVertex, Sink, Tag, LAYER_CAP, LAYER_CORD, LAYER_FILL, LAYER_INTERNAL, LAYER_RADIAL, LAYER_SHELL};

/// Segments around the axis of the airframe's surfaces.
const SEGMENTS: usize = 64;
/// Segments around the axis of internal parts.
const INNER_SEGMENTS: usize = 32;
/// The least thickness a cut wall is drawn with (metres): a 1.5 mm cardboard wall would be a hairline in the cutaway, so the
/// cap faces (only) are drawn at least this thick to keep the hatching legible.
const MIN_CAP_WALL: f32 = 0.005;

/// The part of the cutaway that is removed: a wedge about the axis, `span` wide, centred on direction `centre` in the XY plane
/// (an angle from +X towards +Y). The default is the left half, which faces the home view and the side-on profile.
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
    pub ghost: Option<(usize, &'a PartKind)>,
    /// A radial part shown placed on an axial part.
    pub ghost_radial: Option<GhostRadial<'a>>,
    /// Axial, radial or internal part under the cursor.
    pub hovered: Option<PartId>,
    /// Axial, radial or internal part selected.
    pub selected: Option<PartId>,
    /// The motor database: with it the motors of the active configuration are drawn in their mounts.
    pub motors: Option<&'a MotorDb>,
    /// The cutaway wedge.
    pub cut: Cut,
    /// The part that was just dropped or edited: `(part, outline strength 0..1, height above its place in metres)`. See
    /// [`super::flash`].
    pub flash: Option<(PartId, f32, f32)>,
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
            Section::Tube { outer, inner } => (inner.min(outer - MIN_CAP_WALL).max(0.0), outer),
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
                let (ia, ib) = (inner[i].min(ra - MIN_CAP_WALL).max(0.0), inner[i + 1].min(rb - MIN_CAP_WALL).max(0.0));
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

impl Mesh {
    /// The rocket as a mesh, each part at its station (world z, metres). See [`Marks`] for the ghost, hover and selection and
    /// the motors and cutaway.
    pub fn from_rocket(rocket: &Rocket, marks: Marks) -> Mesh {
        let state_of = |id: PartId| {
            let selected = marks.selected == Some(id);
            match marks.flash {
                // The flash: 4 + strength, or 6 + strength on a selected part (which keeps its full outline).
                Some((flashed, level, _)) if flashed == id => (if selected { 6.0 } else { 4.0 }) + level,
                _ if selected => SELECTED,
                _ if marks.hovered == Some(id) => HOVERED,
                _ => NORMAL,
            }
        };
        let mut stack: Vec<(Option<PartId>, &PartKind, f32)> = rocket.parts().iter().map(|p| (Some(p.id), &p.kind, state_of(p.id))).collect();
        if let Some((index, kind)) = marks.ghost {
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
            b.outer.revolve(&profile, SEGMENTS, Some((z0, z1)));
            if i + 1 == n {
                let radius = kind.aft_diameter() as f32 / 2.0;
                if radius > 0.0 {
                    b.outer.annulus(0.0, radius, z1, true, SEGMENTS, Some((z0, z1)));
                }
            }
            if let Some(id) = id {
                if let Some(part) = rocket.part(*id) {
                    let inner: Vec<f32> = kind.profile().iter().map(|&(pz, _)| part.inner_diameter_at(pz) as f32 / 2.0).collect();
                    b.cap_shell(&profile, &inner, *state);
                    // Shoulders: tubes of the part's material reaching into the neighbouring airframe, seen with the shell
                    // cut away or see-through.
                    for aft in [false, true] {
                        let (Some(s), Some(d)) = (rocket.shoulder(*id, aft).filter(|s| s.is_some()), rocket.shoulder_diameter(*id, aft)) else {
                            continue;
                        };
                        let (ro, len) = (d as f32 / 2.0, s.length as f32);
                        let ri = (ro - s.thickness as f32).max(0.0);
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

        // Internal parts.
        let rows = [-(rmax * 2.0 + 0.03), -(rmax * 4.0 + 0.04)];
        for part in rocket.internals() {
            let (Some((parent_z, parent_dz)), Some(res)) = (z_of(part.parent), rocket.resolved(part.id)) else {
                continue;
            };
            let z0 = (parent_z + part.offset) as f32;
            let state = state_of(part.id);
            let system = part.kind.system();
            let sys = system_slot(system);
            let (od, id_) = (res.outer_diameter as f32, res.inner_diameter as f32);
            let (ro, ri) = (od / 2.0, id_ / 2.0);
            let len = res.length as f32;
            let put = |b: &mut Builder, layer: f32, row: usize, radius: f32, f: &dyn Fn(&mut Sink), section: Option<(Section, f32, f32)>| {
                let explode = Vec3::new(0.0, rows[row], parent_dz);
                b.inner.tag = Some(Tag { state, material: 0.0, system: sys, layer, explode });
                b.caps.tag = Some(Tag { state, material: 0.0, system: sys, layer: LAYER_CAP, explode });
                let start = b.marks();
                f(&mut b.inner);
                if let Some((section, a, c)) = section {
                    b.cap_section(section, a, c, sys, state);
                }
                b.bound(Some(part.id), start, explode);
                let mid = z0 + len / 2.0;
                b.leaders.push(Leader { origin: Vec3::new(0.0, 0.0, mid), offset: Vec3::new(0.0, rows[row], parent_dz), radius });
            };
            match &part.kind {
                InternalKind::InnerTube { thickness, length, .. } => {
                    let wall = *thickness as f32;
                    let l = *length as f32;
                    let ri_t = (ro - wall).max(0.0);
                    put(
                        &mut b,
                        LAYER_INTERNAL,
                        0,
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
                        0,
                        ro,
                        &|s| s.tube(ro, ri, z0, z0 + t, INNER_SEGMENTS),
                        Some((Section::Tube { outer: ro, inner: ri }, z0, z0 + t)),
                    );
                }
                InternalKind::Bulkhead { thickness, .. } => {
                    let t = *thickness as f32;
                    put(&mut b, LAYER_INTERNAL, 0, ro, &|s| s.tube(ro, 0.0, z0, z0 + t, INNER_SEGMENTS), Some((Section::Cylinder(ro), z0, z0 + t)));
                }
                InternalKind::Coupler { length, .. } => {
                    let l = *length as f32;
                    put(
                        &mut b,
                        LAYER_INTERNAL,
                        0,
                        ro,
                        &|s| s.tube(ro, ri, z0, z0 + l, INNER_SEGMENTS),
                        Some((Section::Tube { outer: ro, inner: ri }, z0, z0 + l)),
                    );
                }
                InternalKind::EngineBlock { thickness, .. } => {
                    let t = *thickness as f32;
                    put(
                        &mut b,
                        LAYER_INTERNAL,
                        0,
                        ro,
                        &|s| s.tube(ro, ri, z0, z0 + t, INNER_SEGMENTS),
                        Some((Section::Tube { outer: ro, inner: ri }, z0, z0 + t)),
                    );
                }
                InternalKind::Parachute { packed_length, .. } => {
                    let l = *packed_length as f32;
                    put(&mut b, LAYER_INTERNAL, 0, ro, &|s| s.tube(ro, 0.0, z0, z0 + l, INNER_SEGMENTS), Some((Section::Cylinder(ro), z0, z0 + l)));
                }
                InternalKind::Streamer { width, .. } => {
                    let l = *width as f32;
                    put(&mut b, LAYER_INTERNAL, 0, ro, &|s| s.tube(ro, 0.0, z0, z0 + l, INNER_SEGMENTS), Some((Section::Cylinder(ro), z0, z0 + l)));
                }
                InternalKind::ShockCord { packed_length, .. } => {
                    let l = *packed_length as f32;
                    let amp = ro * 0.7;
                    put(
                        &mut b,
                        LAYER_CORD,
                        0,
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
                            put(&mut b, LAYER_INTERNAL, 0, hh, &|s| s.cuboid(hw, hh, z0, z0 + l), Some((Section::Box { half_w: hw, half_h: hh }, z0, z0 + l)));
                        }
                        _ => put(&mut b, LAYER_INTERNAL, 0, ro, &|s| s.tube(ro, 0.0, z0, z0 + l, INNER_SEGMENTS), Some((Section::Cylinder(ro), z0, z0 + l))),
                    }
                }
            }
        }

        // The loaded motors of the active configuration.
        if let Some(db) = marks.motors {
            for slot in &rocket.active_configuration().motors {
                let (Some(motor_ref), Some(mount)) = (&slot.motor, rocket.internal(slot.mount)) else {
                    continue;
                };
                let (Some(motor), Some((parent_z, parent_dz)), InternalKind::InnerTube { length, overhang, outer_diameter, thickness, .. }) =
                    (db.find(&motor_ref.designation), z_of(mount.parent), &mount.kind)
                else {
                    continue;
                };
                let bore = (outer_diameter - 2.0 * thickness).max(0.0);
                let radius = (motor.diameter.min(if bore > 0.0 { bore } else { motor.diameter }) / 2.0) as f32;
                let aft = (parent_z + mount.offset + length + overhang) as f32;
                let fore = aft - motor.length as f32;
                let state = state_of(mount.id);
                let explode = Vec3::new(0.0, rows[1], parent_dz);
                b.inner.tag = Some(Tag { state, material: 0.0, system: 2.0, layer: LAYER_INTERNAL, explode });
                b.caps.tag = Some(Tag { state, material: 0.0, system: 2.0, layer: LAYER_FILL, explode });
                b.inner.tube(radius, 0.0, fore, aft, INNER_SEGMENTS);
                b.cap_section(Section::Cylinder(radius), fore, aft, 2.0, state);
                b.leaders.push(Leader { origin: Vec3::new(0.0, 0.0, (fore + aft) / 2.0), offset: explode, radius });
            }
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
        PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.1 }
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
}
