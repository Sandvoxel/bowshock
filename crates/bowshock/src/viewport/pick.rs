//! Picking and screen-space geometry of airframe parts: a ray from the orthographic camera against each part's surface of
//! revolution, the part's screen bounds, the positions of its resize handles, and what a handle drag does to its size.

use bowshock_model::{PartId, PartKind, RadialKind, RadialPart, Rocket};
use glam::{DVec3, Vec3};
use iced::{Point, Rectangle, Size, Vector};

use super::radial::{self, Placement};
use super::Camera;

/// Samples around the axis when bounding a part on screen.
const ROUND: usize = 16;

/// Which dimension a resize handle edits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Handle {
    /// The length, from a handle at the aft face on the centreline.
    Length,
    /// The aft (or only) diameter, from a handle at the top of the aft face.
    AftDiameter,
    /// A transition's fore diameter, from a handle at the top of the fore face.
    ForeDiameter,
    /// A fin's span, from a handle at the middle of its tip.
    Span,
    /// A fin's root chord, from a handle at the aft end of its root.
    RootChord,
    /// A fin's tip chord, from a handle at the trailing corner of its tip.
    TipChord,
    /// A fin's sweep (how far back the tip is slanted), from a handle at the leading corner of its tip.
    Sweep,
}

/// What a click or hover landed on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pick {
    Axial(PartId),
    /// One copy of a radial part (every copy of a group selects the group).
    Radial {
        id: PartId,
        instance: usize,
    },
}

impl Pick {
    pub fn id(self) -> PartId {
        match self {
            Pick::Axial(id) | Pick::Radial { id, .. } => id,
        }
    }
}

/// Where a radial part would sit on the surface under the cursor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceTarget {
    pub parent: PartId,
    /// From the parent's fore face to the fore edge of the radial part (already within the parent).
    pub offset: f64,
    /// Angle of the first copy: 0 is +Y, increasing towards +X.
    pub angle: f64,
}

/// Snap step of a drag, set by the modifier keys: 1 mm; Shift locks to a 10 mm grid; Ctrl/Cmd 5 mm; Alt is free.
pub fn step(modifiers: iced::keyboard::Modifiers) -> Option<f64> {
    if modifiers.alt() {
        None
    } else if modifiers.shift() {
        Some(GRID)
    } else if modifiers.control() || modifiers.logo() {
        Some(0.005)
    } else {
        Some(0.001)
    }
}

/// The grid Shift locks to, in metres.
pub const GRID: f64 = 0.010;

/// `offset` on `parent` moved so the station of the part's fore edge is a multiple of `step` (any if `None`), then limited to
/// where `kind` may sit on the parent.
fn snapped_offset(rocket: &Rocket, parent: PartId, offset: f64, kind: &RadialKind, step: Option<f64>) -> Option<f64> {
    let station = rocket.stations()[rocket.index_of(parent)?];
    let offset = step.map_or(offset, |s| ((station + offset) / s).round() * s - station);
    rocket.clamp_offset(parent, offset, kind)
}

/// Smallest length or diameter a handle can drag a part to.
pub const MIN_SIZE: f64 = 0.001;

/// The viewing ray through viewport pixel `at`: a point on the target plane and the direction away from the eye.
fn ray(camera: &Camera, at: Point, size: Size) -> (DVec3, DVec3) {
    let (dx, dy) = ((at.x - size.width / 2.0) / camera.scale, (at.y - size.height / 2.0) / camera.scale);
    let origin = camera.target + camera.right() * dx - camera.up() * dy;
    (origin.as_dvec3(), (-camera.back()).as_dvec3())
}

/// Smallest ray parameter `t` where `o + t d` meets the surface of revolution about Z through `profile` (z, r) from `z0`.
fn hit_profile(o: DVec3, d: DVec3, z0: f64, profile: &[(f64, f64)]) -> Option<f64> {
    let mut best: Option<f64> = None;
    for w in profile.windows(2) {
        let ((za, ra), (zb, rb)) = ((z0 + w[0].0, w[0].1), (z0 + w[1].0, w[1].1));
        if zb <= za {
            continue;
        }
        let k = (rb - ra) / (zb - za);
        // The radius along the ray is s + k dz t; the ray is on the surface where x² + y² equals its square.
        let s = ra + k * (o.z - za);
        let a = d.x * d.x + d.y * d.y - k * k * d.z * d.z;
        let b = 2.0 * (o.x * d.x + o.y * d.y - s * k * d.z);
        let c = o.x * o.x + o.y * o.y - s * s;
        let mut roots = [None, None];
        if a.abs() < 1e-12 {
            if b.abs() > 1e-12 {
                roots[0] = Some(-c / b);
            }
        } else {
            let disc = b * b - 4.0 * a * c;
            if disc >= 0.0 {
                let q = disc.sqrt();
                roots = [Some((-b - q) / (2.0 * a)), Some((-b + q) / (2.0 * a))];
            }
        }
        for t in roots.into_iter().flatten() {
            let z = o.z + t * d.z;
            let eps = 1e-9;
            if z >= za - eps && z <= zb + eps && best.is_none_or(|b| t < b) {
                best = Some(t);
            }
        }
    }
    best
}

/// The axial part surface under viewport pixel `at`: the part, the hit point and the ray parameter.
fn hit_axial(rocket: &Rocket, camera: &Camera, at: Point, size: Size) -> Option<(PartId, DVec3, f64)> {
    let (o, d) = ray(camera, at, size);
    let stations = rocket.stations();
    rocket
        .parts()
        .iter()
        .zip(&stations)
        .filter_map(|(part, &z0)| hit_profile(o, d, z0, &part.kind.profile()).map(|t| (part.id, o + d * t, t)))
        .min_by(|a, b| a.2.total_cmp(&b.2))
}

/// A radial part slid along the rocket (keeping `angle`) so its centre is level with the cursor, wherever the cursor is: the
/// point of the axis nearest the cursor on screen, on whichever axial part is there (the nearest end part beyond the ends).
/// `None` when looking straight down the axis or with no parts.
pub fn slide_target(rocket: &Rocket, camera: &Camera, at: Point, size: Size, kind: &RadialKind, angle: f64, step: Option<f64>) -> Option<SurfaceTarget> {
    let origin = camera.project(Vec3::ZERO, size);
    let one_metre = camera.project(Vec3::Z, size) - origin;
    let n = one_metre.x * one_metre.x + one_metre.y * one_metre.y;
    if n < 1e-6 {
        return None;
    }
    let z = f64::from(((at.x - origin.x) * one_metre.x + (at.y - origin.y) * one_metre.y) / n);
    let stations = rocket.stations();
    let index = stations[..stations.len() - 1].iter().rposition(|&s| s <= z).unwrap_or(0);
    let parent = rocket.parts().get(index)?.id;
    let offset = snapped_offset(rocket, parent, z - stations[index] - kind.extent() / 2.0, kind, step)?;
    Some(SurfaceTarget { parent, offset, angle })
}

/// How a radial part's position is rounded: the first copy's angle to a multiple of `angle` degrees (none if 0), and the
/// station of its fore edge to a multiple of `station` metres (none if `None`).
#[derive(Debug, Clone, Copy, Default)]
pub struct Snapping {
    pub angle: u8,
    pub station: Option<f64>,
}

/// Where a radial part of `kind` would go on the surface under `at`, with its centre under the cursor along the axis.
pub fn surface_target(rocket: &Rocket, camera: &Camera, at: Point, size: Size, kind: &RadialKind, snap: Snapping) -> Option<SurfaceTarget> {
    let (parent, point, _) = hit_axial(rocket, camera, at, size)?;
    let station = rocket.stations()[rocket.index_of(parent)?];
    let offset = snapped_offset(rocket, parent, point.z - station - kind.extent() / 2.0, kind, snap.station)?;
    let angle = point.x.atan2(point.y).rem_euclid(std::f64::consts::TAU);
    let angle = if snap.angle == 0 {
        angle
    } else {
        let step = f64::from(snap.angle).to_radians();
        ((angle / step).round() * step).rem_euclid(std::f64::consts::TAU)
    };
    Some(SurfaceTarget { parent, offset, angle })
}

/// Where copy `instance` of `radial` stands.
pub fn placement(rocket: &Rocket, radial: &RadialPart, instance: usize) -> Option<Placement> {
    let angle = *radial.angles().get(instance)?;
    Some(Placement { z0: rocket.radial_station(radial)? as f32, base: rocket.radial_base_radius(radial)? as f32, angle: angle as f32 })
}

/// Distance of `p` inside the convex polygon `poly` (negative outside), as the least signed edge distance.
fn inside(poly: &[(f64, f64); 4], p: (f64, f64)) -> f64 {
    let area: f64 = (0..4).map(|i| poly[i].0 * poly[(i + 1) % 4].1 - poly[(i + 1) % 4].0 * poly[i].1).sum();
    let orient = area.signum();
    (0..4)
        .map(|i| {
            let (a, b) = (poly[i], poly[(i + 1) % 4]);
            let len = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt().max(1e-12);
            orient * ((b.0 - a.0) * (p.1 - a.1) - (b.1 - a.1) * (p.0 - a.0)) / len
        })
        .fold(f64::INFINITY, f64::min)
}

/// Ray parameter where the ray meets one copy of a radial part, with `margin` metres of slack so thin parts can be hit.
fn hit_radial(o: DVec3, d: DVec3, kind: &RadialKind, at: Placement, margin: f64) -> Option<f64> {
    let (u, t) = radial::dirs(at.angle);
    let (u, t) = (u.as_dvec3(), t.as_dvec3());
    let (z0, base) = (f64::from(at.z0), f64::from(at.base));
    match *kind {
        RadialKind::FinSet { .. } => {
            // The plate lies in the plane through the axis along `u`; meet it, then test the point against the outline.
            let outline = kind.fin_outline()?.map(|(z, h)| (z0 + z, base + h));
            let denom = d.dot(t);
            if denom.abs() < 1e-9 {
                return None;
            }
            let time = -o.dot(t) / denom;
            let p = o + d * time;
            (inside(&outline, (p.z, p.dot(u))) >= -margin).then_some(time)
        }
        RadialKind::LaunchLug { length, diameter } => {
            // A cylinder parallel to the axis through `centre`.
            let (centre, radius) = (u * (base + diameter / 2.0), diameter / 2.0 + margin);
            let (ox, oy) = (o.x - centre.x, o.y - centre.y);
            let (a, b, c) = (d.x * d.x + d.y * d.y, 2.0 * (ox * d.x + oy * d.y), ox * ox + oy * oy - radius * radius);
            let in_length = |time: f64| (z0 - margin..=z0 + length + margin).contains(&(o.z + d.z * time));
            if a < 1e-12 {
                // Looking along the axis: inside the circle, take the near end.
                return (c <= 0.0 && d.z.abs() > 1e-12).then(|| (z0 - o.z) / d.z);
            }
            let disc = b * b - 4.0 * a * c;
            if disc < 0.0 {
                return None;
            }
            let q = disc.sqrt();
            [(-b - q) / (2.0 * a), (-b + q) / (2.0 * a)].into_iter().find(|&time| in_length(time))
        }
        RadialKind::RailButton { height, diameter } => {
            // A sphere around the middle of the button.
            let centre = u * (base + height / 2.0) + DVec3::Z * (z0 + diameter / 2.0);
            let radius = height.max(diameter) / 2.0 + margin;
            let l = centre - o;
            let along = l.dot(d);
            let miss = l.length_squared() - along * along;
            (miss <= radius * radius).then(|| along - (radius * radius - miss).sqrt())
        }
    }
}

/// The part under viewport pixel `at` (the one nearest the eye), for a viewport of `size`.
pub fn pick(rocket: &Rocket, camera: &Camera, at: Point, size: Size) -> Option<Pick> {
    let (o, d) = ray(camera, at, size);
    // A few px of slack so a thin fin or small lug can be hit.
    let margin = 4.0 / f64::from(camera.scale);
    let axial = hit_axial(rocket, camera, at, size).map(|(id, _, t)| (Pick::Axial(id), t));
    let radials = rocket.radials().iter().flat_map(|r| {
        (0..r.angles().len()).filter_map(move |i| {
            let at = placement(rocket, r, i)?;
            hit_radial(o, d, &r.kind, at, margin).map(|t| (Pick::Radial { id: r.id, instance: i }, t))
        })
    });
    // Radial parts are small and sit on the surface: prefer them when within a hair of the body's depth.
    let bias = margin;
    axial.into_iter().chain(radials.map(|(p, t)| (p, t - bias))).min_by(|a, b| a.1.total_cmp(&b.1)).map(|(p, _)| p)
}

/// The screen rectangle enclosing one copy of a radial part.
pub fn radial_bounds(camera: &Camera, size: Size, kind: &RadialKind, at: Placement) -> Rectangle {
    bounds_of(radial::outline_points(kind, at).into_iter().map(|p| camera.project(p, size)))
}

fn bounds_of(points: impl Iterator<Item = Point>) -> Rectangle {
    let (mut min, mut max) = (Point::new(f32::MAX, f32::MAX), Point::new(f32::MIN, f32::MIN));
    for p in points {
        (min.x, min.y, max.x, max.y) = (min.x.min(p.x), min.y.min(p.y), max.x.max(p.x), max.y.max(p.y));
    }
    Rectangle::new(min, Size::new(max.x - min.x, max.y - min.y))
}

/// The handles of a fin set: the span (middle of the leading edge), the root chord (aft root corner), and the tip's leading and trailing
/// corners for its sweep and tip chord. Other radial parts have none.
pub fn radial_handles(camera: &Camera, size: Size, kind: &RadialKind, at: Placement) -> Vec<(Handle, Point)> {
    if !matches!(kind, RadialKind::FinSet { .. }) {
        return Vec::new();
    }
    let corners = radial::fin_corners(kind, at).unwrap_or([Vec3::ZERO; 4]);
    // The span handle is on the middle of the leading edge, clear of the tip's corners.
    let leading = (corners[0] + corners[1]) / 2.0;
    vec![
        (Handle::Span, camera.project(leading, size)),
        (Handle::RootChord, camera.project(corners[3], size)),
        (Handle::Sweep, camera.project(corners[1], size)),
        (Handle::TipChord, camera.project(corners[2], size)),
    ]
}

/// A fin set after dragging `handle` from viewport pixel `from` to `to` (the copy at `angle`), snapped like [`resized`].
pub fn resized_radial(camera: &Camera, size: Size, handle: Handle, base: &RadialKind, angle: f32, from: Point, to: Point, step: Option<f64>) -> RadialKind {
    let RadialKind::FinSet { root_chord, tip_chord, span, sweep, thickness } = *base else {
        return base.clone();
    };
    let along = |dir: Vec3| {
        let (a, b) = (camera.project(Vec3::ZERO, size), camera.project(dir, size));
        let v = Vector::new(b.x - a.x, b.y - a.y);
        let n = v.x * v.x + v.y * v.y;
        if n < 1e-6 {
            0.0
        } else {
            f64::from(((to.x - from.x) * v.x + (to.y - from.y) * v.y) / n)
        }
    };
    // Snapped to the step, at least `floor`: a tip can come to a point, but nothing is thinner than a millimetre.
    let snap = |value: f64, floor: f64| step.map_or(value, |s| (value / s).round() * s).max(floor);
    let fin = |root_chord, tip_chord, span, sweep| RadialKind::FinSet { root_chord, tip_chord, span, sweep, thickness };
    match handle {
        Handle::Span => fin(root_chord, tip_chord, snap(span + along(radial::dirs(angle).0), MIN_SIZE), sweep),
        Handle::RootChord => fin(snap(root_chord + along(Vec3::Z), MIN_SIZE), tip_chord, span, sweep),
        // The model limits a forward sweep to the parent's fore face.
        Handle::Sweep => fin(root_chord, tip_chord, span, snap(sweep + along(Vec3::Z), f64::NEG_INFINITY)),
        Handle::TipChord => fin(root_chord, snap(tip_chord + along(Vec3::Z), 0.0), span, sweep),
        _ => base.clone(),
    }
}

/// The screen rectangle enclosing a part of `kind` whose fore face is at station `z0`.
pub fn screen_bounds(camera: &Camera, size: Size, z0: f64, kind: &PartKind) -> Rectangle {
    let points = kind.profile().into_iter().flat_map(|(z, r)| {
        (0..ROUND).map(move |i| {
            let (s, c) = (std::f32::consts::TAU * i as f32 / ROUND as f32).sin_cos();
            Vec3::new(r as f32 * c, r as f32 * s, (z0 + z) as f32)
        })
    });
    bounds_of(points.map(|p| camera.project(p, size)))
}

/// The world direction, perpendicular to the axis, that points most nearly up the screen: where the diameter handles sit.
fn radial_up(camera: &Camera) -> Vec3 {
    let up = camera.up();
    let flat = up - Vec3::Z * up.z;
    if flat.length() > 1e-4 {
        flat.normalize()
    } else {
        camera.right()
    }
}

/// The handles of a part of `kind` at station `z0`, with their screen positions.
pub fn handles(camera: &Camera, size: Size, z0: f64, kind: &PartKind) -> Vec<(Handle, Point)> {
    let (aft, up) = ((z0 + kind.length()) as f32, radial_up(camera));
    let mut out = vec![
        (Handle::Length, camera.project(Vec3::new(0.0, 0.0, aft), size)),
        (Handle::AftDiameter, camera.project(up * (kind.aft_diameter() as f32 / 2.0) + Vec3::Z * aft, size)),
    ];
    if matches!(kind, PartKind::Transition { .. }) {
        out.push((Handle::ForeDiameter, camera.project(up * (kind.fore_diameter() as f32 / 2.0) + Vec3::Z * z0 as f32, size)));
    }
    out
}

/// The part `base` after dragging `handle` from viewport pixel `from` to `to`: the drag is projected on the handle's screen
/// direction and turned back into metres, rounded to `step` (if any), at least [`MIN_SIZE`].
pub fn resized(camera: &Camera, size: Size, handle: Handle, base: &PartKind, from: Point, to: Point, step: Option<f64>) -> PartKind {
    // Screen vector of one metre along the axis, or of one metre of radius towards the handle.
    let one_metre = |dir: Vec3| -> Vector {
        let (a, b) = (camera.project(Vec3::ZERO, size), camera.project(dir, size));
        Vector::new(b.x - a.x, b.y - a.y)
    };
    let drag = to - from;
    let along = |v: Vector| {
        let n = v.x * v.x + v.y * v.y;
        if n < 1e-6 {
            0.0
        } else {
            f64::from((drag.x * v.x + drag.y * v.y) / n)
        }
    };
    let snap = |value: f64| {
        let v = step.map_or(value, |s| (value / s).round() * s);
        v.max(MIN_SIZE)
    };
    match handle {
        Handle::Length => base.with_length(snap(base.length() + along(one_metre(Vec3::Z)))),
        Handle::AftDiameter => base.with_aft_diameter(snap(base.aft_diameter() + 2.0 * along(one_metre(radial_up(camera))))),
        Handle::ForeDiameter => base.with_fore_diameter(snap(base.fore_diameter() + 2.0 * along(one_metre(radial_up(camera))))),
        Handle::Span | Handle::RootChord | Handle::Sweep | Handle::TipChord => base.clone(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::NoseShape;
    use std::f32::consts::FRAC_PI_2;

    /// Side-on: the nose points left and +Z runs to the right.
    fn side() -> Camera {
        Camera { target: Vec3::new(0.0, 0.0, 0.5), yaw: -FRAC_PI_2, pitch: 0.0, roll: 0.0, scale: 1000.0 }
    }

    fn rocket() -> Rocket {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::Conical, length: 0.2, diameter: 0.1 }).unwrap();
        r.insert(1, PartKind::BodyTube { length: 0.5, diameter: 0.1 }).unwrap();
        r
    }

    const SIZE: Size = Size::new(800.0, 600.0);
    const LUG: RadialKind = RadialKind::LaunchLug { length: 0.1, diameter: 0.01 };

    #[test]
    fn picks_the_part_under_the_cursor() {
        let (r, cam) = (rocket(), side());
        let [nose, tube] = [r.parts()[0].id, r.parts()[1].id];
        // z = 0.1 is on the nose (half-radius 0.025 there), z = 0.4 on the tube; the target z = 0.5 is the screen centre.
        let at = |z: f32, y: f32| cam.project(Vec3::new(0.0, y, z), SIZE);
        assert_eq!(pick(&r, &cam, at(0.1, 0.0), SIZE), Some(Pick::Axial(nose)));
        assert_eq!(pick(&r, &cam, at(0.4, 0.04), SIZE), Some(Pick::Axial(tube)));
        // Above the surface, or beyond the aft end, misses.
        assert_eq!(pick(&r, &cam, at(0.4, 0.06), SIZE), None);
        assert_eq!(pick(&r, &cam, at(0.8, 0.0), SIZE), None);
        // Above the nose's slim tip but inside the tube's silhouette height.
        assert_eq!(pick(&r, &cam, at(0.02, 0.04), SIZE), None);
    }

    #[test]
    fn picks_the_nearer_part_from_an_oblique_view() {
        let (r, mut cam) = (rocket(), side());
        cam.yaw = -1.0;
        cam.pitch = 0.4;
        let tube = r.parts()[1].id;
        assert_eq!(pick(&r, &cam, cam.project(Vec3::new(0.0, 0.0, 0.45), SIZE), SIZE), Some(Pick::Axial(tube)));
    }

    #[test]
    fn handles_and_bounds_follow_the_part() {
        let (r, cam) = (rocket(), side());
        let tube = &r.parts()[1].kind;
        let [(_, length), (_, diameter)] = handles(&cam, SIZE, 0.2, tube)[..] else { panic!("two handles") };
        let end = cam.project(Vec3::new(0.0, 0.0, 0.7), SIZE);
        assert!((length.x - end.x).abs() < 1e-3 && (length.y - end.y).abs() < 1e-3);
        // The diameter handle is 50 mm (50 px) above the axis at the aft face.
        assert!((diameter.x - end.x).abs() < 1e-3 && (end.y - diameter.y - 50.0).abs() < 1e-2);
        let b = screen_bounds(&cam, SIZE, 0.2, tube);
        assert!((b.width - 500.0).abs() < 1.0 && (b.height - 100.0).abs() < 1.0);
        let trans = PartKind::Transition { length: 0.1, fore_diameter: 0.1, aft_diameter: 0.06 };
        assert_eq!(handles(&cam, SIZE, 0.0, &trans).len(), 3);
    }

    #[test]
    fn dragging_a_handle_edits_the_size() {
        let cam = side();
        let tube = PartKind::BodyTube { length: 0.5, diameter: 0.1 };
        let from = cam.project(Vec3::new(0.0, 0.0, 0.7), SIZE);
        // 100 px along the axis is 100 mm at 1000 px/m, snapped to the millimetre.
        let longer = resized(&cam, SIZE, Handle::Length, &tube, from, from + Vector::new(100.4, 7.0), Some(0.001));
        assert!((longer.length() - 0.6).abs() < 1e-9);
        // Free (Shift) keeps the fraction; 5 mm snap rounds to 5 mm.
        assert!((resized(&cam, SIZE, Handle::Length, &tube, from, from + Vector::new(100.4, 0.0), None).length() - 0.6004).abs() < 1e-6);
        assert!((resized(&cam, SIZE, Handle::Length, &tube, from, from + Vector::new(102.0, 0.0), Some(0.005)).length() - 0.6).abs() < 1e-9);
        // Dragging up by 20 px makes the radius 20 mm larger: the diameter grows by 40 mm.
        let wider = resized(&cam, SIZE, Handle::AftDiameter, &tube, from, from + Vector::new(0.0, -20.0), Some(0.001));
        assert!((wider.aft_diameter() - 0.14).abs() < 1e-9);
        // Never below 1 mm.
        assert_eq!(resized(&cam, SIZE, Handle::Length, &tube, from, from + Vector::new(-900.0, 0.0), Some(0.001)).length(), MIN_SIZE);
    }

    #[test]
    fn surface_target_snaps_angle_and_clamps_offset() {
        let (r, cam) = (rocket(), side());
        let tube = r.parts()[1].id;
        // Side-on from -X (yaw -90°): the visible side of the tube faces -X, which is angle 270° (0 is +Y, +X is 90°).
        let at = cam.project(Vec3::new(0.0, 0.0, 0.45), SIZE);
        let t = surface_target(&r, &cam, at, SIZE, &LUG, Snapping::default()).unwrap();
        assert_eq!(t.parent, tube);
        assert!((t.angle.to_degrees() - 270.0).abs() < 1e-3, "{}", t.angle.to_degrees());
        // The part's centre is under the cursor: station 0.45 is 0.25 into the tube, so a 0.1 long part starts at 0.2.
        assert!((t.offset - 0.2).abs() < 1e-3, "{}", t.offset);
        // Near the aft end the whole part is kept on the tube.
        let end = cam.project(Vec3::new(0.0, 0.0, 0.69), SIZE);
        assert!((surface_target(&r, &cam, end, SIZE, &LUG, Snapping::default()).unwrap().offset - 0.4).abs() < 1e-9);
        // Off the body: nothing; above the tube's top edge the ray grazes the silhouette from the side.
        assert!(surface_target(&r, &cam, cam.project(Vec3::new(0.0, 0.2, 0.45), SIZE), SIZE, &LUG, Snapping::default()).is_none());
        // Angle snap rounds the first copy's angle.
        let mut oblique = cam;
        oblique.yaw = -FRAC_PI_2 + 0.3;
        let free = surface_target(&r, &oblique, at, SIZE, &LUG, Snapping::default()).unwrap().angle.to_degrees();
        let snapped = surface_target(&r, &oblique, at, SIZE, &LUG, Snapping { angle: 45, station: None }).unwrap().angle.to_degrees();
        assert!((snapped / 45.0 - (snapped / 45.0).round()).abs() < 1e-9 && (snapped - free).abs() <= 22.5 + 1e-6);
    }

    #[test]
    fn radial_parts_are_picked_with_their_instance() {
        let (mut r, cam) = (rocket(), side());
        let tube = r.parts()[1].id;
        // Four fins at the aft end of the tube: angles 0°, 90°, 180°, 270° (up, +X, down, -X).
        let fins = RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.05, sweep: 0.03, thickness: 0.003 };
        let id = r.attach_radial(tube, 0.4, 0.0, 4, fins).unwrap();
        // The upward fin (instance 0) stands above the tube: 0.05 radius + half the span.
        let up = cam.project(Vec3::new(0.0, 0.075, 0.62), SIZE);
        assert_eq!(pick(&r, &cam, up, SIZE), Some(Pick::Radial { id, instance: 0 }));
        let down = cam.project(Vec3::new(0.0, -0.075, 0.62), SIZE);
        assert_eq!(pick(&r, &cam, down, SIZE), Some(Pick::Radial { id, instance: 2 }));
        // Clear of every part.
        assert_eq!(pick(&r, &cam, cam.project(Vec3::new(0.0, 0.2, 0.62), SIZE), SIZE), None);
        // On the body away from the fins the tube is picked as before.
        assert_eq!(pick(&r, &cam, cam.project(Vec3::new(0.0, 0.0, 0.35), SIZE), SIZE), Some(Pick::Axial(tube)));
        // A lug and a button are hit near their middle.
        let lug = r.attach_radial(tube, 0.0, 1.5 * std::f64::consts::PI, 1, RadialKind::LaunchLug { length: 0.06, diameter: 0.012 }).unwrap();
        assert_eq!(pick(&r, &cam, cam.project(Vec3::new(-0.056, 0.0, 0.23), SIZE), SIZE), Some(Pick::Radial { id: lug, instance: 0 }));
    }

    #[test]
    fn a_forward_swept_fin_is_picked_where_its_tip_is() {
        let (mut r, cam) = (rocket(), side());
        let tube = r.parts()[1].id;
        // Root chord 0.1 from station 0.4 to 0.5 (0.2 into the tube); the tip leads by 0.06: tip edge at stations 0.34 to 0.39.
        let fins = RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.1, sweep: -0.06, thickness: 0.003 };
        let id = r.attach_radial(tube, 0.2, 0.0, 4, fins).unwrap();
        let at = |z: f32, y: f32| cam.project(Vec3::new(0.0, y, z), SIZE);
        // Near the tip, ahead of the root (which starts at 0.4): the fin, not the tube.
        assert_eq!(pick(&r, &cam, at(0.365, 0.14), SIZE), Some(Pick::Radial { id, instance: 0 }));
        // The same height ahead of the tip is clear air.
        assert_eq!(pick(&r, &cam, at(0.3, 0.14), SIZE), None);
    }

    #[test]
    fn fin_handles_drag_the_span_and_root_chord() {
        let cam = side();
        let fins = RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.05, sweep: 0.03, thickness: 0.003 };
        let at = Placement { z0: 0.5, base: 0.05, angle: 0.0 };
        let hs = radial_handles(&cam, SIZE, &fins, at);
        assert_eq!(hs.iter().map(|h| h.0).collect::<Vec<_>>(), [Handle::Span, Handle::RootChord, Handle::Sweep, Handle::TipChord]);
        let from = hs[0].1;
        // Straight up 30 px at 1000 px/m is 30 mm of span.
        let RadialKind::FinSet { span, .. } = resized_radial(&cam, SIZE, Handle::Span, &fins, 0.0, from, from + Vector::new(0.0, -30.0), Some(0.001)) else {
            panic!("a fin set");
        };
        assert!((span - 0.08).abs() < 1e-9);
        let RadialKind::FinSet { root_chord, .. } = resized_radial(&cam, SIZE, Handle::RootChord, &fins, 0.0, from, from + Vector::new(20.0, 0.0), Some(0.005))
        else {
            panic!("a fin set");
        };
        assert!((root_chord - 0.12).abs() < 1e-9);
        // The tip's corners edit the slant and the tip chord; a tip can come to a point (tip chord 0) but not go negative.
        let drag = |handle, dx: f32, step| resized_radial(&cam, SIZE, handle, &fins, 0.0, from, from + Vector::new(dx, 0.0), step);
        let RadialKind::FinSet { sweep, tip_chord, root_chord, .. } = drag(Handle::Sweep, 25.0, Some(0.001)) else {
            panic!("a fin set");
        };
        assert!((sweep - 0.055).abs() < 1e-9 && tip_chord == 0.05 && root_chord == 0.1);
        let RadialKind::FinSet { sweep, tip_chord, .. } = drag(Handle::TipChord, -80.0, Some(0.001)) else {
            panic!("a fin set");
        };
        assert!(tip_chord == 0.0 && sweep == 0.03);
        // Sweep goes through zero to a forward sweep (the model limits how far).
        let RadialKind::FinSet { sweep, .. } = drag(Handle::Sweep, -80.0, Some(0.001)) else {
            panic!("a fin set");
        };
        assert!((sweep + 0.05).abs() < 1e-9);
        // A lug has no handles.
        assert!(radial_handles(&cam, SIZE, &RadialKind::LaunchLug { length: 0.05, diameter: 0.01 }, at).is_empty());
    }

    #[test]
    fn sliding_follows_the_cursor_along_the_whole_rocket_and_keeps_the_angle() {
        let (r, cam) = (rocket(), side());
        let [nose, tube] = [r.parts()[0].id, r.parts()[1].id];
        let fins = RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.05, sweep: 0.03, thickness: 0.003 };
        // Anywhere on screen, even well above the body: the part's centre goes level with the cursor (station 0.45 is 0.25 into
        // the tube, so a 0.1 root chord starts at 0.2), at the angle it had.
        let level = |z: f32, y: f32| slide_target(&r, &cam, cam.project(Vec3::new(0.0, y, z), SIZE), SIZE, &fins, 1.25, None).unwrap();
        for y in [0.0, 0.3] {
            let t = level(0.45, y);
            assert!((t.offset - 0.2).abs() < 1e-3 && t.angle == 1.25 && t.parent == tube, "{t:?}");
        }
        // It moves on to whichever part is under the cursor along the axis, keeping its angle: here the nose (0 to 0.2).
        let on_nose = level(0.1, 0.0);
        assert_eq!((on_nose.parent, on_nose.angle), (nose, 1.25));
        // Fins may trail the aft end by up to three quarters of the root chord; a lug may not.
        assert!((level(5.0, 0.0).offset - (0.5 - 0.025)).abs() < 1e-9);
        assert!((slide_target(&r, &cam, cam.project(Vec3::new(0.0, 0.0, 5.0), SIZE), SIZE, &LUG, 0.0, None).unwrap().offset - 0.4).abs() < 1e-9);
        assert_eq!(level(-5.0, 0.0).offset, 0.0);
        // Looking straight down the axis there is nothing to slide along.
        let mut down = cam;
        (down.yaw, down.pitch) = (0.0, 0.0);
        assert!(slide_target(&r, &down, Point::new(10.0, 10.0), SIZE, &fins, 0.0, None).is_none());
        assert!(slide_target(&Rocket::default(), &cam, Point::new(10.0, 10.0), SIZE, &fins, 0.0, None).is_none());
    }

    #[test]
    fn modifiers_choose_the_snap_step() {
        use iced::keyboard::Modifiers as M;
        assert_eq!(step(M::empty()), Some(0.001));
        assert_eq!(step(M::SHIFT), Some(0.010));
        assert_eq!(step(M::CTRL), Some(0.005));
        assert_eq!(step(M::LOGO), Some(0.005));
        assert_eq!(step(M::ALT), None);
    }

    #[test]
    fn the_grid_locks_the_station_of_a_radial_part() {
        let (r, cam) = (rocket(), side());
        let tube = r.parts()[1].id;
        // Cursor at station 0.4537: a 0.1 long part centred there starts at 0.4037 (offset 0.2037 into the tube, which starts at
        // 0.2). On a 10 mm grid the fore edge goes to station 0.40 (offset 0.20); on a 1 mm grid, to 0.404.
        let at = cam.project(Vec3::new(0.0, 0.0, 0.4537), SIZE);
        let grid = slide_target(&r, &cam, at, SIZE, &LUG, 0.0, Some(GRID)).unwrap();
        assert!((grid.offset - 0.2).abs() < 1e-6 && grid.parent == tube, "{grid:?}");
        let fine = slide_target(&r, &cam, at, SIZE, &LUG, 0.0, Some(0.001)).unwrap();
        assert!((fine.offset - 0.204).abs() < 1e-6, "{fine:?}");
        let surface = surface_target(&r, &cam, at, SIZE, &LUG, Snapping { angle: 0, station: Some(GRID) }).unwrap();
        assert!((surface.offset - 0.2).abs() < 1e-6, "{surface:?}");
    }
}
