//! Where the handles of a selected part sit on screen, and how a drag of each maps to its dimension (design kit, DirectEdit).
//!
//! The model lists *what* can be edited ([`Rocket::handle_specs`]); this module places each handle for the current camera and
//! says how screen movement turns into a value: along a line (lengths, diameters, positions), around a centre (angles), or in
//! the plane of a fin (a freeform vertex). Handles keep a constant pixel size, so only their positions follow the camera.

use bowshock_model::{Dim, HandleKind, HandleSpec, HandleView, PartKind, PartRef, RadialKind, Rocket};
use glam::Vec3;
use iced::{Point, Rectangle, Size, Vector};

use super::pick;
use super::radial::{dirs, Placement};
use super::Camera;
use bowshock_model::PartId;

/// How a drag maps to a value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Axis {
    /// Along the screen direction `dir` (unit); one value unit is `per_unit` px.
    Line { dir: Vector, per_unit: f32 },
    /// Around `centre`: the value changes by the angle the cursor sweeps (0 is up, increasing clockwise).
    Arc { centre: Point },
    /// A freeform fin vertex: `z` and `h` are the screen vectors of one metre along the axis and one metre of height.
    Plane { z: Vector, h: Vector },
}

/// A handle placed on screen.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Placed {
    /// What it edits and how it looks.
    pub spec: HandleSpec,
    /// Its screen position.
    pub at: Point,
    /// How dragging it changes the value.
    pub axis: Axis,
}

/// The aft-view inset: a small end-on picture of a radial part beside it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Inset {
    /// Panel rectangle.
    pub panel: Rectangle,
    /// Centre of the rocket's axis in the panel.
    pub centre: Point,
    /// Radius of the parent surface, px.
    pub base: f32,
    /// How far the part reaches beyond the surface, px (fin span, lug diameter, button height).
    pub reach: f32,
    /// Pixels per metre of the inset picture.
    pub scale: f32,
}

/// Size of the inset panel.
pub const INSET: f32 = 150.0;
/// Pixels per metre of fin thickness in the inset (thickness is exaggerated so it can be grabbed).
pub const THICKNESS_SCALE: f32 = 6000.0;
/// Length of the cant lever in the inset, px.
pub const CANT_LEVER: f32 = 30.0;

/// The world direction perpendicular to the axis that points most nearly up the screen: where the diameter handles sit.
pub fn radial_up(camera: &Camera) -> Vec3 {
    let up = camera.up();
    let flat = up - Vec3::Z * up.z;
    if flat.length() > 1e-4 {
        flat.normalize()
    } else {
        camera.right()
    }
}

/// Unit screen direction of an increasing `angle` point at `centre`... (0 up, clockwise): the offset of a point at `angle`.
pub fn arc_dir(angle: f64) -> Vector {
    let (s, c) = (angle as f32).sin_cos();
    Vector::new(s, -c)
}

/// The angle (0 up, clockwise) of `p` seen from `centre`.
pub fn angle_about(centre: Point, p: Point) -> f64 {
    f64::from((p.x - centre.x).atan2(centre.y - p.y))
}

/// `a` wrapped into `-pi..pi`.
pub fn wrap(a: f64) -> f64 {
    let t = std::f64::consts::TAU;
    (a + std::f64::consts::PI).rem_euclid(t) - std::f64::consts::PI
}

/// A line handle at world point `w`: increasing the value by one unit moves it `world_per_unit` metres along `dir`.
fn line(camera: &Camera, size: Size, w: Vec3, dir: Vec3, world_per_unit: f32) -> Option<(Point, Axis)> {
    const DELTA: f32 = 0.01;
    let (a, b) = (camera.project(w, size), camera.project(w + dir * DELTA, size));
    let v = Vector::new(b.x - a.x, b.y - a.y);
    let len = (v.x * v.x + v.y * v.y).sqrt();
    if len < 1e-3 {
        return None;
    }
    Some((a, Axis::Line { dir: Vector::new(v.x / len, v.y / len), per_unit: len / DELTA * world_per_unit }))
}

fn at(up: Vec3, r: f64, z: f64) -> Vec3 {
    up * r as f32 + Vec3::Z * z as f32
}

/// Everything about the part a handle needs: the placement of a radial part, or the station of an axial or internal one.
pub struct Context<'a> {
    /// The document.
    pub rocket: &'a Rocket,
    /// The camera.
    pub camera: &'a Camera,
    /// The viewport size.
    pub size: Size,
    /// Which copy of a radial group is shown.
    pub instance: usize,
    /// The screen bounds of the part (for the inset's place).
    pub bounds: Rectangle,
    /// Whether the ViewCube is shown (3D): the inset sits under it, else under the toolbar row.
    pub cube: bool,
}

/// The inset for radial part `id`: pinned top-right in the viewport (under the ViewCube in 3D, else under the way back), like
/// the kit's AFT VIEW panel.
pub fn inset(cx: &Context, id: PartId) -> Option<Inset> {
    let r = cx.rocket.radial(id)?;
    let place = pick::placement(cx.rocket, r, cx.instance)?;
    let reach = match &r.effective(cx.instance).kind {
        RadialKind::LaunchLug { diameter, .. } => *diameter,
        RadialKind::RailButton { height, .. } => *height,
        RadialKind::Pod { gap, body } => gap + 2.0 * body.radius(),
        k => k.fin_span().unwrap_or(0.0),
    };
    let x = (cx.size.width - INSET - 16.0).max(8.0);
    let y = if cx.cube { 16.0 + super::CUBE_SIZE + 8.0 } else { 64.0 };
    let panel = Rectangle::new(Point::new(x, y), Size::new(INSET, INSET));
    let total = f64::from(place.base) + reach;
    let scale = (INSET / 2.0 - 26.0) / total.max(1e-3) as f32;
    Some(Inset { panel, centre: Point::new(x + INSET / 2.0, y + INSET / 2.0 + 8.0), base: place.base * scale, reach: reach as f32 * scale, scale })
}

/// Every handle of part `id` for the current camera, in the model's order (`handle_specs`). Handles that cannot be placed
/// (looking end-on) are left out.
pub fn place(cx: &Context, id: PartId) -> Vec<Placed> {
    let rocket = cx.rocket;
    let (camera, size) = (cx.camera, cx.size);
    let up = radial_up(camera);
    let inset = inset(cx, id);
    let mut out = Vec::new();
    for spec in rocket.handle_specs(id) {
        let placed = match rocket.kind_of(id) {
            Some(PartRef::Axial(p)) => {
                let z0 = rocket.part_station(id).unwrap_or(0.0);
                match shoulder_end(spec.dim) {
                    Some(aft) => shoulder(cx, up, id, z0, &p.kind, aft, spec.dim),
                    None => axial(camera, size, up, z0, &p.kind, p.thickness, spec.dim),
                }
            }
            Some(PartRef::Radial(r)) => match pick::placement(rocket, r, cx.instance) {
                Some(place) => radial(cx, up, r, place, inset.as_ref(), spec.dim),
                None => None,
            },
            Some(PartRef::Internal(_)) => internal(cx, up, id, spec.dim),
            None => None,
        };
        if let Some((pos, axis)) = placed {
            out.push(Placed { spec, at: pos, axis });
        }
    }
    out
}

/// Whether a dimension is of the aft (`Some(true)`) or fore (`Some(false)`) shoulder; `None` for any other.
fn shoulder_end(dim: Dim) -> Option<bool> {
    match dim {
        Dim::ShoulderLength | Dim::ShoulderDiameter | Dim::ShoulderThickness => Some(true),
        Dim::ForeShoulderLength | Dim::ForeShoulderDiameter | Dim::ForeShoulderThickness => Some(false),
        _ => None,
    }
}

/// The handles of a shoulder, inside the neighbouring tube: the length at its free end on the axis, the diameter on its wall
/// half way along. A shoulder of no length offers only the length handle.
fn shoulder(cx: &Context, up: Vec3, id: PartId, z0: f64, kind: &PartKind, aft: bool, dim: Dim) -> Option<(Point, Axis)> {
    let (camera, size) = (cx.camera, cx.size);
    let (s, d) = (cx.rocket.shoulder(id, aft)?, cx.rocket.shoulder_diameter(id, aft)?);
    let face = if aft { z0 + kind.length() } else { z0 };
    let sign = if aft { 1.0 } else { -1.0 };
    match dim {
        Dim::ShoulderLength | Dim::ForeShoulderLength => line(camera, size, at(up, 0.0, face + sign * s.length), Vec3::Z * sign as f32, 1.0),
        Dim::ShoulderDiameter | Dim::ForeShoulderDiameter if s.length > 0.0 => line(camera, size, at(up, d / 2.0, face + sign * s.length / 2.0), up, 0.5),
        _ => None,
    }
}

fn axial(camera: &Camera, size: Size, up: Vec3, z0: f64, kind: &PartKind, wall: f64, dim: Dim) -> Option<(Point, Axis)> {
    let l = kind.length();
    let z1 = z0 + l;
    match dim {
        Dim::Length => line(camera, size, at(up, 0.0, z1), Vec3::Z, 1.0),
        Dim::Diameter => line(camera, size, at(up, kind.aft_diameter() / 2.0, z1), up, 0.5),
        Dim::ForeDiameter => line(camera, size, at(up, kind.fore_diameter() / 2.0, z0), up, 0.5),
        Dim::Wall => line(camera, size, at(up, kind.radius_at(l / 2.0) - wall, z0 + l / 2.0), -up, 1.0),
        Dim::Shape => {
            let PartKind::NoseCone { shape, length, diameter } = *kind else {
                return None;
            };
            let p = shape.parameter()?;
            let x = 0.6 * length;
            let r = kind.radius_at(x);
            // Radius at the handle per unit of the parameter, so the handle follows the cursor along the curve.
            let h = 0.02;
            let probe = |q: f64| {
                let shape = match shape {
                    bowshock_model::NoseShape::PowerSeries(_) => bowshock_model::NoseShape::PowerSeries(q.clamp(0.01, 1.0)),
                    _ => bowshock_model::NoseShape::Haack(q.clamp(0.0, 1.0)),
                };
                PartKind::NoseCone { shape, length, diameter }.radius_at(x)
            };
            let d_r = (probe(p + h) - probe(p - h)) / (2.0 * h);
            let (pos, axis) = line(camera, size, at(up, r, z0 + x), up, 1.0)?;
            let Axis::Line { dir, per_unit } = axis else {
                return None;
            };
            let sign = if d_r < 0.0 { -1.0 } else { 1.0 };
            Some((pos, Axis::Line { dir, per_unit: (per_unit * d_r.abs() as f32).max(20.0) * sign }))
        }
        _ => None,
    }
}

fn radial(cx: &Context, up: Vec3, r: &bowshock_model::RadialPart, place: Placement, inset: Option<&Inset>, dim: Dim) -> Option<(Point, Axis)> {
    let (camera, size) = (cx.camera, cx.size);
    let (u, _) = dirs(place.angle);
    let (z0, base) = (f64::from(place.z0), f64::from(place.base));
    let _ = up;
    let own = r.effective(cx.instance);
    let world = |radius: f64, z: f64| u * radius as f32 + Vec3::Z * z as f32;
    match (dim, &own.kind) {
        // A pod: its position by the nose tip (along the core), its gap by the inner side at the middle (out from the core).
        (Dim::Position, RadialKind::Pod { gap, body }) => line(camera, size, world(base + gap + body.radius(), z0), Vec3::Z, 1.0),
        (Dim::Gap, RadialKind::Pod { gap, body }) => line(camera, size, world(base + gap, z0 + body.length() * 0.6), u, 1.0),
        (Dim::Angle, RadialKind::Pod { .. }) => {
            let ins = inset?;
            Some((ins.centre + arc_dir(r.angle) * (ins.base + ins.reach + 9.0), Axis::Arc { centre: ins.centre }))
        }
        // The fillet radius sits on the fillet's outer edge a quarter of the way along the root, a circle handle.
        (Dim::FilletRadius, k) if k.is_fin() => {
            let rf = r.fillet_radius;
            line(camera, size, world(base + rf, z0 + k.extent() * 0.25), u, 1.0)
        }
        (Dim::Span, RadialKind::FinSet { span, sweep, tip_chord, .. }) => line(camera, size, world(base + span, z0 + sweep + tip_chord / 2.0), u, 1.0),
        (Dim::Span, RadialKind::EllipticalFin { span, root_chord, .. }) => line(camera, size, world(base + span, z0 + root_chord / 2.0), u, 1.0),
        (Dim::RootChord, RadialKind::FinSet { root_chord, .. } | RadialKind::EllipticalFin { root_chord, .. }) => {
            line(camera, size, world(base, z0 + root_chord), Vec3::Z, 1.0)
        }
        (Dim::TipChord, RadialKind::FinSet { sweep, tip_chord, span, .. }) => line(camera, size, world(base + span, z0 + sweep + tip_chord), Vec3::Z, 1.0),
        (Dim::Sweep, RadialKind::FinSet { sweep, span, .. }) => line(camera, size, world(base + span, z0 + sweep), Vec3::Z, 1.0),
        (Dim::Position, RadialKind::FinSet { root_chord, .. } | RadialKind::EllipticalFin { root_chord, .. }) => {
            line(camera, size, world(base, z0 + root_chord / 2.0), Vec3::Z, 1.0)
        }
        (Dim::Position, RadialKind::FreeformFin { .. }) => {
            let mid = own.kind.extent() / 2.0;
            line(camera, size, world(base, z0 + mid), Vec3::Z, 1.0)
        }
        (Dim::Position, RadialKind::LaunchLug { diameter, .. }) => line(camera, size, world(base + diameter / 2.0, z0), Vec3::Z, 1.0),
        (Dim::Position, RadialKind::RailButton { height, diameter }) => line(camera, size, world(base + height, z0 + diameter / 2.0), Vec3::Z, 1.0),
        (Dim::Length, RadialKind::LaunchLug { length, diameter }) => line(camera, size, world(base + diameter / 2.0, z0 + length), Vec3::Z, 1.0),
        (Dim::Height, RadialKind::RailButton { height, diameter }) => line(camera, size, world(base + height, z0 + diameter / 2.0 + 0.0), u, 1.0),
        (Dim::Vertex(i), RadialKind::FreeformFin { points, .. }) => {
            let &(z, h) = points.get(i)?;
            let (a, bz, bh) = (
                camera.project(world(base + h, z0 + z), size),
                camera.project(world(base + h, z0 + z + 0.01), size),
                camera.project(world(base + h + 0.01, z0 + z), size),
            );
            let (zv, hv) = (Vector::new((bz.x - a.x) / 0.01, (bz.y - a.y) / 0.01), Vector::new((bh.x - a.x) / 0.01, (bh.y - a.y) / 0.01));
            Some((a, Axis::Plane { z: zv, h: hv }))
        }
        // The aft-view inset handles.
        (Dim::Thickness, k) => {
            let (ins, t) = (inset?, k.thickness()?);
            let a0 = r.angle;
            let root = ins.centre + arc_dir(a0) * (ins.base + ins.reach / 2.0);
            let tangent = arc_dir(a0 + std::f64::consts::FRAC_PI_2);
            let off = (t as f32 * THICKNESS_SCALE / 2.0).max(1.0) + 9.0;
            Some((root + tangent * off, Axis::Line { dir: tangent, per_unit: THICKNESS_SCALE / 2.0 }))
        }
        (Dim::Cant, _) => {
            let ins = inset?;
            let root = ins.centre + arc_dir(r.angle) * ins.base;
            Some((root + arc_dir(r.angle + own.cant) * CANT_LEVER.max(ins.reach), Axis::Arc { centre: root }))
        }
        (Dim::Angle, _) => {
            let ins = inset?;
            let reach = if r.kind.is_fin() { 0.0 } else { 10.0 };
            let a = if r.kind.is_fin() { r.angle - 0.5 } else { r.angle };
            Some((ins.centre + arc_dir(a) * (ins.base + reach), Axis::Arc { centre: ins.centre }))
        }
        _ => None,
    }
}

fn internal(cx: &Context, up: Vec3, id: PartId, dim: Dim) -> Option<(Point, Axis)> {
    let (camera, size, rocket) = (cx.camera, cx.size, cx.rocket);
    let part = rocket.internal(id)?;
    let res = rocket.resolved(id)?;
    let zf = rocket.part_station(id)?;
    let (len, r) = (res.length, res.outer_diameter / 2.0);
    let z1 = zf + len;
    use bowshock_model::InternalKind as K;
    match (dim, &part.kind) {
        (Dim::Position, K::InnerTube { .. } | K::Coupler { .. } | K::CenteringRing { .. } | K::EngineBlock { .. } | K::Bulkhead { .. }) => {
            line(camera, size, at(up, r, zf), Vec3::Z, 1.0)
        }
        (Dim::Position, K::Parachute { .. } | K::ShockCord { .. } | K::Streamer { .. }) => line(camera, size, at(up, r / 2.0, zf), Vec3::Z, 1.0),
        (Dim::Position, K::MassComponent { .. }) => line(camera, size, at(up, 0.0, zf), Vec3::Z, 1.0),
        (Dim::Length, K::InnerTube { .. } | K::Coupler { .. }) => line(camera, size, at(up, r, z1), Vec3::Z, 1.0),
        (Dim::Length, K::MassComponent { .. }) => line(camera, size, at(up, 0.0, z1), Vec3::Z, 1.0),
        (Dim::PackedLength, _) => line(camera, size, at(up, r / 2.0, z1), Vec3::Z, 1.0),
        (Dim::Overhang, K::InnerTube { overhang, .. }) => line(camera, size, at(up, 0.0, z1 + overhang), Vec3::Z, 1.0),
        (Dim::Diameter, _) => line(camera, size, at(up, r, zf + len / 2.0), up, 0.5),
        (Dim::Wall, K::InnerTube { thickness, .. } | K::Coupler { thickness, .. }) => line(camera, size, at(up, r - thickness, zf + len * 0.25), -up, 1.0),
        (Dim::Thickness, _) => {
            let ri = res.inner_diameter / 2.0;
            line(camera, size, at(up, (r + ri) / 2.0, z1), Vec3::Z, 1.0)
        }
        (Dim::InnerDiameter, _) => line(camera, size, at(up, res.inner_diameter / 2.0, zf + len / 2.0), up, 0.5),
        _ => None,
    }
}

/// The value after dragging handle `placed` from `from` to `to`, starting at `base` (before snapping). For a vertex use
/// [`vertex_after`].
pub fn value_after(placed: &Placed, base: f64, from: Point, to: Point) -> f64 {
    match placed.axis {
        Axis::Line { dir, per_unit } => {
            let d = to - from;
            base + f64::from((d.x * dir.x + d.y * dir.y) / per_unit)
        }
        Axis::Arc { centre } => base + wrap(angle_about(centre, to) - angle_about(centre, from)),
        Axis::Plane { .. } => base,
    }
}

/// A freeform vertex `(z, h)` after dragging from `from` to `to`.
pub fn vertex_after(placed: &Placed, base: (f64, f64), from: Point, to: Point) -> (f64, f64) {
    let Axis::Plane { z, h } = placed.axis else {
        return base;
    };
    let d = to - from;
    let det = z.x * h.y - z.y * h.x;
    if det.abs() < 1e-6 {
        return base;
    }
    let dz = (d.x * h.y - d.y * h.x) / det;
    let dh = (z.x * d.y - z.y * d.x) / det;
    (base.0 + f64::from(dz), base.1 + f64::from(dh))
}

/// The tangent... kept for the tags: whether the handle is a circle.
pub fn is_circle(kind: HandleKind) -> bool {
    !matches!(kind, HandleKind::Linear)
}

/// Whether a handle shows inside the aft inset.
pub fn in_inset(spec: &HandleSpec) -> bool {
    spec.view == HandleView::Aft
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::NoseShape;
    use std::f32::consts::FRAC_PI_2;

    fn side() -> Camera {
        Camera { target: Vec3::new(0.0, 0.0, 0.5), yaw: -FRAC_PI_2, pitch: 0.0, roll: 0.0, scale: 1000.0, ortho_blend: 1.0 }
    }

    const SIZE: Size = Size::new(800.0, 600.0);

    fn rocket() -> (Rocket, PartId, PartId) {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::Haack(1.0 / 3.0), length: 0.2, diameter: 0.1 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.1 }).unwrap();
        let fin =
            r.attach_radial(tube, 0.4, 0.0, 3, RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.05, sweep: 0.03, thickness: 0.003 }).unwrap();
        (r, tube, fin)
    }

    fn cx<'a>(r: &'a Rocket, cam: &'a Camera) -> Context<'a> {
        Context { rocket: r, camera: cam, size: SIZE, instance: 0, bounds: Rectangle::new(Point::new(300.0, 200.0), Size::new(200.0, 100.0)), cube: false }
    }

    #[test]
    fn tube_handles_sit_on_the_aft_face_and_drag_in_metres() {
        let (r, tube, _) = rocket();
        let cam = side();
        let hs = place(&cx(&r, &cam), tube);
        let length = hs.iter().find(|h| h.spec.dim == Dim::Length).unwrap();
        let end = cam.project(Vec3::new(0.0, 0.0, 0.7), SIZE);
        assert!((length.at.x - end.x).abs() < 1e-2 && (length.at.y - end.y).abs() < 1e-2);
        // 100 px along the axis at 1000 px/m is 0.1 m.
        let v = value_after(length, 0.5, length.at, length.at + Vector::new(100.0, 9.0));
        assert!((v - 0.6).abs() < 1e-4);
        // The diameter handle is 50 px above the axis; 20 px up makes the radius 20 mm bigger: the diameter 40 mm.
        let dia = hs.iter().find(|h| h.spec.dim == Dim::Diameter).unwrap();
        assert!((end.y - dia.at.y - 50.0).abs() < 1e-2);
        assert!((value_after(dia, 0.1, dia.at, dia.at + Vector::new(0.0, -20.0)) - 0.14).abs() < 1e-4);
    }

    #[test]
    fn fin_handles_cover_every_dimension() {
        let (r, _, fin) = rocket();
        let cam = side();
        let hs = place(&cx(&r, &cam), fin);
        let dims: Vec<Dim> = hs.iter().map(|h| h.spec.dim).collect();
        for d in [Dim::Span, Dim::RootChord, Dim::TipChord, Dim::Sweep, Dim::Position, Dim::Thickness, Dim::Cant, Dim::Angle] {
            assert!(dims.contains(&d), "{d:?}");
        }
        // The upward fin (angle 0) is seen side-on: the span handle moves straight up the screen.
        let span = hs.iter().find(|h| h.spec.dim == Dim::Span).unwrap();
        assert!((value_after(span, 0.05, span.at, span.at + Vector::new(0.0, -30.0)) - 0.08).abs() < 1e-4);
        let ins = inset(&cx(&r, &cam), fin).unwrap();
        // Dragging the angle handle a quarter turn about the axis adds 90 degrees.
        let angle = hs.iter().find(|h| h.spec.dim == Dim::Angle).unwrap();
        let from = angle.at;
        let to = ins.centre + Vector::new((from.y - ins.centre.y).abs().max(1.0), 0.0) * 1.0;
        let v = value_after(angle, 0.0, from, to);
        assert!(v.abs() <= std::f64::consts::PI);
    }

    #[test]
    fn a_vertex_drag_splits_into_axis_and_height() {
        let (mut r, tube, _) = rocket();
        let id = r.attach_radial(tube, 0.1, 0.0, 1, RadialKind::FreeformFin { points: vec![(0.0, 0.0), (0.02, 0.05), (0.1, 0.0)], thickness: 0.003 }).unwrap();
        let cam = side();
        let hs = place(&cx(&r, &cam), id);
        let v = hs.iter().find(|h| h.spec.dim == Dim::Vertex(1)).unwrap();
        let (z, h) = vertex_after(v, (0.02, 0.05), v.at, v.at + Vector::new(10.0, -10.0));
        assert!((z - 0.03).abs() < 1e-4 && (h - 0.06).abs() < 1e-4, "{z} {h}");
    }

    #[test]
    fn wrap_and_angle_about() {
        assert!((wrap(4.0) - (4.0 - std::f64::consts::TAU)).abs() < 1e-12);
        let c = Point::new(10.0, 10.0);
        assert!(angle_about(c, Point::new(10.0, 0.0)).abs() < 1e-6);
        assert!((angle_about(c, Point::new(20.0, 10.0)) - std::f64::consts::FRAC_PI_2).abs() < 1e-6);
    }
}
