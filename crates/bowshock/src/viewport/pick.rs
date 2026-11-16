//! Picking and screen-space geometry of airframe parts: a ray from the camera against each part's surface of revolution and
//! the part's screen bounds. (Handles live in `handles`.)

use bowshock_model::{PartId, PartKind, RadialKind, RadialPart, Rocket};
use glam::{DVec3, Vec3};
use iced::{Point, Rectangle, Size};

use super::pods::PodView;
use super::radial::{self, Placement};
use super::Camera;

/// Samples around the axis when bounding a part on screen.
const ROUND: usize = 16;

/// What a click or hover landed on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pick {
    Axial(PartId),
    /// One copy of a radial part (every copy of a group selects the group).
    Radial {
        id: PartId,
        instance: usize,
    },
    /// An internal part of the core, where the view shows the inside (see [`Inside`]).
    Internal(PartId),
}

impl Pick {
    pub fn id(self) -> PartId {
        match self {
            Pick::Axial(id) | Pick::Internal(id) | Pick::Radial { id, .. } => id,
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
    /// Which copy of the pod the parent is in, for a parent that is a part of a pod (0 for a part of the core): the copy the
    /// cursor is on, which is where the overlay marks the target.
    pub copy: usize,
}

/// The snap step of a plain or coarse drag of a station: `grid` (the view toolbar's step, metres), or 5 mm with Ctrl/Cmd.
/// Shift (snap to targets) and Alt (follow the surface) are handled by the overlays.
pub fn step(modifiers: iced::keyboard::Modifiers, grid: f64) -> Option<f64> {
    Some(if modifiers.control() || modifiers.logo() { 0.005 } else { grid })
}

/// `offset` on `parent` moved so the station of the part's fore edge is a multiple of `step` (any if `None`), then limited to
/// where `kind` may sit on the parent.
fn snapped_offset(rocket: &Rocket, parent: PartId, offset: f64, kind: &RadialKind, step: Option<f64>) -> Option<f64> {
    let station = rocket.stations()[rocket.index_of(parent)?];
    let offset = step.map_or(offset, |s| ((station + offset) / s).round() * s - station);
    rocket.clamp_offset(parent, offset, kind)
}

/// The viewing ray through viewport pixel `at`: its origin and unit direction (perspective or orthographic).
fn ray(camera: &Camera, at: Point, size: Size) -> (DVec3, DVec3) {
    let (origin, dir) = camera.ray(at, size);
    (origin.as_dvec3(), dir.as_dvec3())
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
    slide_core(rocket, camera, at, size, kind, angle, step, 0)
}

/// [`slide_target`] for a group picked up from part `from`: one that came off a part of a pod slides along the pod copy nearest
/// the cursor instead.
#[allow(clippy::too_many_arguments)]
pub fn slide_from(
    rocket: &Rocket,
    camera: &Camera,
    at: Point,
    size: Size,
    kind: &RadialKind,
    angle: f64,
    step: Option<f64>,
    from: PartId,
) -> Option<SurfaceTarget> {
    let Some(view) = PodView::of_part(rocket, from, camera) else {
        return slide_target(rocket, camera, at, size, kind, angle, step);
    };
    // The copy whose axis is nearest the cursor.
    let length = view.sub.parts().iter().map(|p| p.kind.length()).sum::<f64>();
    let copy = view
        .cameras
        .iter()
        .enumerate()
        .min_by(|a, b| {
            let d = |c: &Camera| c.project(Vec3::Z * (length / 2.0) as f32, size).distance(at);
            d(a.1).total_cmp(&d(b.1))
        })
        .map_or(0, |(i, _)| i);
    slide_core(&view.sub, &view.cameras[copy], at, size, kind, angle, step, copy)
}

#[allow(clippy::too_many_arguments)]
fn slide_core(rocket: &Rocket, camera: &Camera, at: Point, size: Size, kind: &RadialKind, angle: f64, step: Option<f64>, copy: usize) -> Option<SurfaceTarget> {
    let z = axis_station(camera, at, size)?;
    let stations = rocket.stations();
    let index = stations[..stations.len() - 1].iter().rposition(|&s| s <= z).unwrap_or(0);
    let parent = rocket.parts().get(index)?.id;
    let offset = snapped_offset(rocket, parent, z - stations[index] - kind.extent() / 2.0, kind, step)?;
    Some(SurfaceTarget { parent, offset, angle, copy })
}

/// `t` moved so a radial part of `kind` sits flush with whichever end of its tube it is nearer: its fore edge on the tube's fore
/// end, or its aft edge on the aft end (Shift while placing or sliding).
pub fn flush(rocket: &Rocket, kind: &RadialKind, t: SurfaceTarget) -> SurfaceTarget {
    let Some(part) = rocket.part(t.parent) else { return t };
    let (length, extent) = (part.kind.length(), kind.extent());
    let offset = if t.offset + extent / 2.0 > length / 2.0 { (length - extent).max(0.0) } else { 0.0 };
    SurfaceTarget { offset: rocket.clamp_offset(t.parent, offset, kind).unwrap_or(t.offset), ..t }
}

/// How a radial part's position is rounded: the first copy's angle to a multiple of `angle` degrees (none if 0), and the
/// station of its fore edge to a multiple of `station` metres (none if `None`).
#[derive(Debug, Clone, Copy, Default)]
pub struct Snapping {
    pub angle: u8,
    pub station: Option<f64>,
}

/// Where a radial part of `kind` would go on the surface under `at`, with its centre under the cursor along the axis: on the
/// core, or (for anything but a pod) on a part of a pod, whichever is nearer the eye.
pub fn surface_target(rocket: &Rocket, camera: &Camera, at: Point, size: Size, kind: &RadialKind, snap: Snapping) -> Option<SurfaceTarget> {
    let core = surface_core(rocket, camera, at, size, kind, snap, 0);
    let pods = rocket.pods().filter(|_| !kind.is_pod()).filter_map(|pod| PodView::of_pod(rocket, pod, camera)).flat_map(|view| {
        view.cameras.iter().enumerate().filter_map(|(copy, cam)| surface_core(&view.sub, cam, at, size, kind, snap, copy)).collect::<Vec<_>>()
    });
    core.into_iter().chain(pods).min_by(|a, b| a.1.total_cmp(&b.1)).map(|(target, _)| target)
}

fn surface_core(rocket: &Rocket, camera: &Camera, at: Point, size: Size, kind: &RadialKind, snap: Snapping, copy: usize) -> Option<(SurfaceTarget, f64)> {
    let (parent, point, t) = hit_axial(rocket, camera, at, size)?;
    let station = rocket.stations()[rocket.index_of(parent)?];
    let offset = snapped_offset(rocket, parent, point.z - station - kind.extent() / 2.0, kind, snap.station)?;
    let angle = point.x.atan2(point.y).rem_euclid(std::f64::consts::TAU);
    let angle = if snap.angle == 0 {
        angle
    } else {
        let step = f64::from(snap.angle).to_radians();
        ((angle / step).round() * step).rem_euclid(std::f64::consts::TAU)
    };
    Some((SurfaceTarget { parent, offset, angle, copy }, t))
}

/// Where copy `instance` of `radial` stands.
pub fn placement(rocket: &Rocket, radial: &RadialPart, instance: usize) -> Option<Placement> {
    let angle = *radial.angles().get(instance)?;
    let cant = radial.effective(instance).cant as f32;
    Some(Placement { z0: rocket.radial_station(radial)? as f32, base: rocket.radial_base_radius(radial)? as f32, angle: angle as f32, cant })
}

/// `v` (a point when `point`, else a direction) turned back by a fin's cant, so a canted plate can be tested as if it were flat.
fn untilt(v: DVec3, at: Placement, point: bool) -> DVec3 {
    let (u, t) = radial::dirs(at.angle);
    let (u, t) = (u.as_dvec3(), t.as_dvec3());
    let (s, c) = f64::from(at.cant).sin_cos();
    let pivot = if point { DVec3::Z * f64::from(at.z0) } else { DVec3::ZERO };
    let rel = v - pivot;
    let (along, across) = (rel.z, rel.dot(t));
    pivot + u * rel.dot(u) + DVec3::Z * (along * c - across * s) + t * (across * c + along * s)
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
    let (o, d) = if at.cant != 0.0 && kind.is_fin() { (untilt(o, at, true), untilt(d, at, false)) } else { (o, d) };
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
                return (c <= 0.0 && d.z.abs() > 1e-12).then_some((z0 - o.z) / d.z);
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
        RadialKind::EllipticalFin { .. } | RadialKind::FreeformFin { .. } => {
            let polygon: Vec<(f64, f64)> = kind.planform()?.into_iter().map(|(z, h)| (z0 + z, base + h)).collect();
            let denom = d.dot(t);
            if denom.abs() < 1e-9 {
                return None;
            }
            let time = -o.dot(t) / denom;
            let p = o + d * time;
            near_polygon(&polygon, (p.z, p.dot(u)), margin).then_some(time)
        }
        // A pod is picked through its parts (see `pick`).
        RadialKind::Pod { .. } => None,
    }
}

/// Whether `p` is inside the polygon (even-odd) or within `margin` of its outline.
fn near_polygon(poly: &[(f64, f64)], p: (f64, f64), margin: f64) -> bool {
    let n = poly.len();
    let mut inside = false;
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        if (a.1 > p.1) != (b.1 > p.1) && p.0 < (b.0 - a.0) * (p.1 - a.1) / (b.1 - a.1) + a.0 {
            inside = !inside;
        }
        let (ab, ap) = ((b.0 - a.0, b.1 - a.1), (p.0 - a.0, p.1 - a.1));
        let len2 = (ab.0 * ab.0 + ab.1 * ab.1).max(1e-18);
        let t = ((ap.0 * ab.0 + ap.1 * ab.1) / len2).clamp(0.0, 1.0);
        let foot = (a.0 + ab.0 * t, a.1 + ab.1 * t);
        if ((p.0 - foot.0).powi(2) + (p.1 - foot.1).powi(2)).sqrt() <= margin {
            return true;
        }
    }
    inside
}

/// The part under viewport pixel `at` (the one nearest the eye) of a view that shows no internal parts: a part of the core or
/// of a pod (a pod's parts are found through the pod's own frame). The shell hides what is inside it.
#[cfg(test)]
pub fn pick(rocket: &Rocket, camera: &Camera, at: Point, size: Size) -> Option<Pick> {
    pick_inside(rocket, camera, at, size, Inside::Hidden)
}

/// Which internal parts of the core the view shows, and so which a press can land on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Inside {
    /// Solid: the shell hides them.
    Hidden,
    /// Only this one (the selected internal part: the shell is ghosted around it in any view).
    Only(PartId),
    /// Cutaway and X-ray: all of them.
    All,
}

impl Inside {
    /// Whether internal part `id` can be picked.
    fn shows(self, id: PartId) -> bool {
        match self {
            Inside::Hidden => false,
            Inside::Only(only) => only == id,
            Inside::All => true,
        }
    }
}

/// [`pick`] where the view shows the `inside`: an internal part under the cursor is picked rather than the shell around it
/// (a fin standing out in front of it still wins). Internal parts of pods are never picked.
pub fn pick_inside(rocket: &Rocket, camera: &Camera, at: Point, size: Size, inside: Inside) -> Option<Pick> {
    let core = pick_core(rocket, camera, at, size, inside);
    let pods = rocket
        .pods()
        .filter_map(|pod| PodView::of_pod(rocket, pod, camera))
        .flat_map(|view| view.cameras.iter().filter_map(|cam| pick_core(&view.sub, cam, at, size, Inside::Hidden)).collect::<Vec<_>>());
    core.into_iter().chain(pods).min_by(|a, b| a.1.total_cmp(&b.1)).map(|(p, _)| p)
}

/// The nearest internal part of `rocket`'s own stack that `inside` shows under the ray, with its ray parameter: each is the
/// cylinder of its (resolved) outer diameter along its length, with `margin` metres of slack so thin rings can be hit.
fn hit_internal(rocket: &Rocket, o: DVec3, d: DVec3, inside: Inside, margin: f64) -> Option<(PartId, f64)> {
    rocket
        .internals()
        .iter()
        .filter(|p| inside.shows(p.id) && !matches!(p.kind, bowshock_model::InternalKind::ShockCord { .. }))
        .filter_map(|p| {
            let (z0, res) = (rocket.part_station(p.id)?, rocket.resolved(p.id)?);
            let r = res.outer_diameter / 2.0 + margin;
            hit_profile(o, d, z0 - margin, &[(0.0, r), (res.length + 2.0 * margin, r)]).map(|t| (p.id, t))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
}

/// The nearest part of `rocket`'s own stack (and what stands on or in it) under `at`, with its ray parameter.
fn pick_core(rocket: &Rocket, camera: &Camera, at: Point, size: Size, inside: Inside) -> Option<(Pick, f64)> {
    let (o, d) = ray(camera, at, size);
    // A few px of slack so a thin fin or small lug can be hit.
    let margin = 4.0 / f64::from(camera.scale);
    // An internal part that is shown is what the press is on, not the shell around it.
    let internal = hit_internal(rocket, o, d, inside, margin).map(|(id, t)| (Pick::Internal(id), t));
    let axial = if internal.is_some() { None } else { hit_axial(rocket, camera, at, size).map(|(id, _, t)| (Pick::Axial(id), t)) };
    let radials = rocket.radials().iter().flat_map(|r| {
        (0..r.angles().len()).filter_map(move |i| {
            let at = placement(rocket, r, i)?;
            hit_radial(o, d, &r.effective(i).kind, at, margin).map(|t| (Pick::Radial { id: r.id, instance: i }, t))
        })
    });
    // Radial parts are small and sit on the surface: prefer them when within a hair of the body's depth.
    let bias = margin;
    axial.into_iter().chain(internal).chain(radials.map(|(p, t)| (p, t - bias))).min_by(|a, b| a.1.total_cmp(&b.1))
}

/// The station of the point of the axis nearest viewport pixel `at` on screen; `None` when looking straight down the axis.
pub fn axis_station(camera: &Camera, at: Point, size: Size) -> Option<f64> {
    let origin = camera.project(Vec3::ZERO, size);
    let one_metre = camera.project(Vec3::Z, size) - origin;
    let n = one_metre.x * one_metre.x + one_metre.y * one_metre.y;
    (n >= 1e-6).then(|| f64::from(((at.x - origin.x) * one_metre.x + (at.y - origin.y) * one_metre.y) / n))
}

/// The convex hull of `points` (Andrew's monotone chain), anticlockwise.
fn hull(mut points: Vec<Point>) -> Vec<Point> {
    points.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    points.dedup();
    if points.len() < 3 {
        return points;
    }
    let cross = |o: Point, a: Point, b: Point| (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    let mut chain: Vec<Point> = Vec::with_capacity(points.len() * 2);
    for pass in 0..2 {
        let start = chain.len();
        for &p in &points {
            while chain.len() >= start + 2 && cross(chain[chain.len() - 2], chain[chain.len() - 1], p) <= 0.0 {
                chain.pop();
            }
            chain.push(p);
        }
        chain.pop();
        if pass == 0 {
            points.reverse();
        }
    }
    chain
}

/// Distance (px) from `p` to the convex polygon `poly`: 0 inside.
fn distance_to_hull(poly: &[Point], p: Point) -> f32 {
    let mut inside = poly.len() >= 3;
    let mut best = f32::MAX;
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
        let ab = b - a;
        let len2 = ab.x * ab.x + ab.y * ab.y;
        let t = if len2 > 0.0 { (((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / len2).clamp(0.0, 1.0) } else { 0.0 };
        best = best.min(p.distance(Point::new(a.x + ab.x * t, a.y + ab.y * t)));
        inside &= ab.x * (p.y - a.y) - ab.y * (p.x - a.x) >= 0.0;
    }
    if inside {
        0.0
    } else {
        best
    }
}

/// How far (px) viewport pixel `at` is from the surface of the stack that part `parent` is in: of the core, or of the pod
/// copy nearest the cursor for a part of a pod. 0 anywhere over the stack, so a part sliding along the rocket never leaves it.
/// The silhouette is the convex hull of each part's projected profile.
pub fn stack_distance(rocket: &Rocket, camera: &Camera, at: Point, size: Size, parent: PartId) -> f32 {
    let of = |rocket: &Rocket, camera: &Camera| {
        let stations = rocket.stations();
        rocket
            .parts()
            .iter()
            .zip(&stations)
            .map(|(part, &z0)| distance_to_hull(&hull(profile_points(camera, size, z0, &part.kind)), at))
            .fold(f32::MAX, f32::min)
    };
    match PodView::of_part(rocket, parent, camera) {
        Some(view) => view.cameras.iter().map(|cam| of(&view.sub, cam)).fold(f32::MAX, f32::min),
        None => of(rocket, camera),
    }
}

/// The screen outline of one copy of `kind` standing at the origin on a surface of radius 0 at `angle`: its convex hull, to be
/// moved wherever the part is drawn (a part carried free of the rocket).
pub fn radial_silhouette(camera: &Camera, size: Size, kind: &RadialKind, angle: f32) -> Vec<Point> {
    hull(radial::outline_points(kind, Placement { z0: 0.0, base: 0.0, angle, cant: 0.0 }).into_iter().map(|p| camera.project(p, size)).collect())
}

/// Screen positions of `kind`'s profile sampled round the axis, its fore face at station `z0`.
fn profile_points(camera: &Camera, size: Size, z0: f64, kind: &PartKind) -> Vec<Point> {
    kind.profile()
        .into_iter()
        .flat_map(|(z, r)| {
            (0..ROUND).map(move |i| {
                let (s, c) = (std::f32::consts::TAU * i as f32 / ROUND as f32).sin_cos();
                Vec3::new(r as f32 * c, r as f32 * s, (z0 + z) as f32)
            })
        })
        .map(|p| camera.project(p, size))
        .collect()
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

/// The screen rectangle enclosing a part of `kind` whose fore face is at station `z0`.
pub fn screen_bounds(camera: &Camera, size: Size, z0: f64, kind: &PartKind) -> Rectangle {
    bounds_of(profile_points(camera, size, z0, kind).into_iter())
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::NoseShape;
    use std::f32::consts::FRAC_PI_2;

    /// Side-on: the nose points left and +Z runs to the right.
    fn side() -> Camera {
        Camera { target: Vec3::new(0.0, 0.0, 0.5), yaw: -FRAC_PI_2, pitch: 0.0, roll: 0.0, scale: 1000.0, ortho_blend: 1.0 }
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
    fn flush_puts_the_part_against_the_nearer_end_of_its_tube() {
        let r = rocket();
        let tube = r.parts()[1].id;
        let at = |offset: f64| SurfaceTarget { parent: tube, offset, angle: 1.0, copy: 0 };
        // A 0.1 m lug on a 0.5 m tube: centred aft of the middle → aft edge on the aft end (offset 0.4); fore of it → offset 0.
        assert!((flush(&r, &LUG, at(0.3)).offset - 0.4).abs() < 1e-12);
        assert_eq!(flush(&r, &LUG, at(0.1)).offset, 0.0);
        assert_eq!(flush(&r, &LUG, at(0.3)).angle, 1.0);
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
    fn ctrl_gives_the_coarse_step_and_otherwise_the_grid() {
        use iced::keyboard::Modifiers as M;
        assert_eq!(step(M::empty(), 0.001), Some(0.001));
        assert_eq!(step(M::SHIFT, 0.0005), Some(0.0005));
        assert_eq!(step(M::CTRL, 0.001), Some(0.005));
        assert_eq!(step(M::LOGO, 0.001), Some(0.005));
    }

    #[test]
    fn handles_bounds_follow_the_part() {
        let (r, cam) = (rocket(), side());
        let b = screen_bounds(&cam, SIZE, 0.2, &r.parts()[1].kind);
        assert!((b.width - 500.0).abs() < 1.0 && (b.height - 100.0).abs() < 1.0);
    }

    #[test]
    fn the_grid_locks_the_station_of_a_radial_part() {
        let (r, cam) = (rocket(), side());
        let tube = r.parts()[1].id;
        // Cursor at station 0.4537: a 0.1 long part centred there starts at 0.4037 (offset 0.2037 into the tube, which starts at
        // 0.2). On a 10 mm grid the fore edge goes to station 0.40 (offset 0.20); on a 1 mm grid, to 0.404.
        let at = cam.project(Vec3::new(0.0, 0.0, 0.4537), SIZE);
        let grid = slide_target(&r, &cam, at, SIZE, &LUG, 0.0, Some(0.010)).unwrap();
        assert!((grid.offset - 0.2).abs() < 1e-6 && grid.parent == tube, "{grid:?}");
        let fine = slide_target(&r, &cam, at, SIZE, &LUG, 0.0, Some(0.001)).unwrap();
        assert!((fine.offset - 0.204).abs() < 1e-6, "{fine:?}");
        let surface = surface_target(&r, &cam, at, SIZE, &LUG, Snapping { angle: 0, station: Some(0.010) }).unwrap();
        assert!((surface.offset - 0.2).abs() < 1e-6, "{surface:?}");
    }

    #[test]
    fn an_internal_part_is_picked_over_the_shell_around_it_where_the_view_shows_it() {
        let (mut r, cam) = (rocket(), side());
        let tube = r.parts()[1].id;
        let kind = bowshock_model::InternalKind::InnerTube { outer_diameter: 0.04, thickness: 0.002, length: 0.2, motor_mount: true, overhang: 0.0 };
        let mount = r.attach_internal(tube, 0.1, kind).unwrap();
        let other = r.attach_internal(tube, 0.35, bowshock_model::InternalKind::Bulkhead { diameter: None, thickness: 0.01 }).unwrap();
        // The mount stands at stations 0.3 to 0.5, 0.02 either side of the axis.
        let on = cam.project(Vec3::new(0.0, 0.01, 0.4), SIZE);
        // Solid: the shell hides it, so the press is on the tube (what the old picking did everywhere).
        assert_eq!(pick_inside(&r, &cam, on, SIZE, Inside::Hidden), Some(Pick::Axial(tube)));
        // Cutaway and X-ray show every internal part: the press is on the mount, not the tube.
        assert_eq!(pick_inside(&r, &cam, on, SIZE, Inside::All), Some(Pick::Internal(mount)));
        // Only the selected internal part shows in Solid.
        assert_eq!(pick_inside(&r, &cam, on, SIZE, Inside::Only(mount)), Some(Pick::Internal(mount)));
        assert_eq!(pick_inside(&r, &cam, on, SIZE, Inside::Only(other)), Some(Pick::Axial(tube)));
        // Beside the mount, inside the tube, or past its end: the tube is what the press is on.
        assert_eq!(pick_inside(&r, &cam, cam.project(Vec3::new(0.0, 0.04, 0.4), SIZE), SIZE, Inside::All), Some(Pick::Axial(tube)));
        assert_eq!(pick_inside(&r, &cam, cam.project(Vec3::new(0.0, 0.01, 0.6), SIZE), SIZE, Inside::All), Some(Pick::Axial(tube)));
        // A thin bulkhead (10 mm at station 0.55, 8 px on screen) is hit too, and a fin standing out in front still wins.
        assert_eq!(pick_inside(&r, &cam, cam.project(Vec3::new(0.0, 0.03, 0.555), SIZE), SIZE, Inside::All), Some(Pick::Internal(other)));
        let fins =
            r.attach_radial(tube, 0.1, 0.0, 3, RadialKind::FinSet { root_chord: 0.1, tip_chord: 0.05, span: 0.06, sweep: 0.04, thickness: 0.003 }).unwrap();
        let fin = cam.project(Vec3::new(0.0, 0.08, 0.35), SIZE);
        assert_eq!(pick_inside(&r, &cam, fin, SIZE, Inside::All), Some(Pick::Radial { id: fins, instance: 0 }));
    }

    #[test]
    fn the_distance_from_the_rocket_is_zero_over_it_and_grows_off_its_surface() {
        let (r, cam) = (rocket(), side());
        let tube = r.parts()[1].id;
        let d = |z: f32, y: f32| stack_distance(&r, &cam, cam.project(Vec3::new(0.0, y, z), SIZE), SIZE, tube);
        assert_eq!((d(0.45, 0.0), d(0.45, 0.04), d(0.1, 0.0)), (0.0, 0.0, 0.0));
        // 0.09 m is 0.04 m (40 px at 1000 px/m) off the tube's surface; 0.1 m past the aft end is 100 px.
        assert!((d(0.45, 0.09) - 40.0).abs() < 1.0, "{}", d(0.45, 0.09));
        assert!((d(0.8, 0.0) - 100.0).abs() < 1.0, "{}", d(0.8, 0.0));
    }
}
