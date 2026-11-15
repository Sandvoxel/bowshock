//! Geometry of radial parts (fins, launch lugs, rail buttons): the surface mesh of one copy, and the 3D points that picking and
//! the selection brackets work from.

use bowshock_model::RadialKind;
use glam::Vec3;

use super::scene::Vertex;

/// Sides of the round parts.
const SIDES: usize = 16;
/// How far a fin's root edge sinks into the parent, so no gap shows on a curved surface.
const SINK: f32 = 0.0005;
/// Fraction of a rail button's height taken by the stem; the head is the rest.
const STEM: f32 = 0.7;
/// Stem diameter as a fraction of the head's.
const STEM_WIDTH: f32 = 0.6;

/// Unit vector from the axis out towards a copy at `angle` (0 is +Y, increasing towards +X), and the unit tangent
/// (`angle` increasing).
pub fn dirs(angle: f32) -> (Vec3, Vec3) {
    let (s, c) = angle.sin_cos();
    (Vec3::new(s, c, 0.0), Vec3::new(c, -s, 0.0))
}

/// A copy of `kind` placed with its fore edge at `z0` on a surface of radius `base`, at `angle`.
#[derive(Debug, Clone, Copy)]
pub struct Placement {
    pub z0: f32,
    pub base: f32,
    pub angle: f32,
}

/// The fin plate's corners (root leading edge, tip leading edge, tip trailing edge, root trailing edge).
pub fn fin_corners(kind: &RadialKind, at: Placement) -> Option<[Vec3; 4]> {
    let (u, _) = dirs(at.angle);
    let outline = kind.fin_outline()?;
    Some(outline.map(|(z, h)| {
        let radius = at.base + h as f32 - if h == 0.0 { SINK } else { 0.0 };
        u * radius + Vec3::Z * (at.z0 + z as f32)
    }))
}

/// Points enclosing one copy, for screen bounds.
pub fn outline_points(kind: &RadialKind, at: Placement) -> Vec<Vec3> {
    let (u, t) = dirs(at.angle);
    match *kind {
        RadialKind::FinSet { thickness, .. } => {
            let half = t * (thickness as f32 / 2.0);
            fin_corners(kind, at).map(|c| c.into_iter().flat_map(|p| [p + half, p - half]).collect()).unwrap_or_default()
        }
        RadialKind::LaunchLug { length, diameter } => {
            let (r, centre) = (diameter as f32 / 2.0, u * (at.base + diameter as f32 / 2.0));
            circle(centre + Vec3::Z * at.z0, Vec3::Z, r).into_iter().chain(circle(centre + Vec3::Z * (at.z0 + length as f32), Vec3::Z, r)).collect()
        }
        RadialKind::RailButton { height, diameter } => {
            let (h, r) = (height as f32, diameter as f32 / 2.0);
            let z = at.z0 + r;
            circle(u * (at.base + h) + Vec3::Z * z, u, r).into_iter().chain(circle(u * at.base + Vec3::Z * z, u, r)).collect()
        }
        RadialKind::EllipticalFin { thickness, .. } | RadialKind::FreeformFin { thickness, .. } => {
            let half = t * (thickness as f32 / 2.0);
            kind.planform()
                .map(|points| {
                    points
                        .into_iter()
                        .flat_map(|(z, h)| {
                            let p = u * (at.base + h as f32) + Vec3::Z * (at.z0 + z as f32);
                            [p + half, p - half]
                        })
                        .collect()
                })
                .unwrap_or_default()
        }
    }
}

/// Points around a circle about `centre` perpendicular to `axis`.
fn circle(centre: Vec3, axis: Vec3, radius: f32) -> Vec<Vec3> {
    let (e1, e2) = basis(axis);
    (0..8)
        .map(|i| {
            let (s, c) = (std::f32::consts::TAU * i as f32 / 8.0).sin_cos();
            centre + (e1 * c + e2 * s) * radius
        })
        .collect()
}

fn basis(axis: Vec3) -> (Vec3, Vec3) {
    let helper = if axis.z.abs() < 0.9 { Vec3::Z } else { Vec3::X };
    let e1 = axis.cross(helper).normalize();
    (e1, axis.cross(e1))
}

/// One triangle, wound counter-clockwise as seen from the side its vertex normals point to.
fn triangle(out: &mut Vec<Vertex>, p: [Vec3; 3], n: [Vec3; 3], state: f32) {
    let flip = (p[1] - p[0]).cross(p[2] - p[0]).dot(n[0] + n[1] + n[2]) < 0.0;
    let order = if flip { [0, 2, 1] } else { [0, 1, 2] };
    out.extend(order.map(|i| Vertex { position: p[i].to_array(), normal: n[i].to_array(), state, material: 1.0 }));
}

/// A flat polygon face (fan from its first point) facing away from `inside`.
fn face(out: &mut Vec<Vertex>, points: &[Vec3], inside: Vec3, state: f32) {
    let normal = (points[1] - points[0]).cross(points[2] - points[0]).normalize();
    let normal = if normal.dot(points[0] - inside) < 0.0 { -normal } else { normal };
    for i in 1..points.len() - 1 {
        triangle(out, [points[0], points[i], points[i + 1]], [normal; 3], state);
    }
}

/// A cylinder from `base` along unit `axis`, with smooth sides and flat caps.
fn cylinder(out: &mut Vec<Vertex>, base: Vec3, axis: Vec3, length: f32, radius: f32, state: f32) {
    let (e1, e2) = basis(axis);
    let ring = |i: usize| {
        let (s, c) = (std::f32::consts::TAU * i as f32 / SIDES as f32).sin_cos();
        e1 * c + e2 * s
    };
    let top = base + axis * length;
    for i in 0..SIDES {
        let (a, b) = (ring(i), ring(i + 1));
        let (pa0, pb0, pa1, pb1) = (base + a * radius, base + b * radius, top + a * radius, top + b * radius);
        triangle(out, [pa0, pb0, pb1], [a, b, b], state);
        triangle(out, [pa0, pb1, pa1], [a, b, a], state);
        triangle(out, [top, pa1, pb1], [axis; 3], state);
        triangle(out, [base, pb0, pa0], [-axis; 3], state);
    }
}

/// Appends the surface of one copy of `kind`.
pub fn emit(out: &mut Vec<Vertex>, kind: &RadialKind, at: Placement, state: f32) {
    let (u, t) = dirs(at.angle);
    match *kind {
        RadialKind::FinSet { thickness, .. } => {
            let Some(corners) = fin_corners(kind, at) else {
                return;
            };
            let half = t * (thickness as f32 / 2.0);
            let centre = corners.iter().sum::<Vec3>() / 4.0;
            let front = corners.map(|p| p + half);
            let back = corners.map(|p| p - half);
            face(out, &front, centre, state);
            face(out, &back, centre, state);
            for i in 0..4 {
                let j = (i + 1) % 4;
                face(out, &[front[i], front[j], back[j], back[i]], centre, state);
            }
        }
        RadialKind::LaunchLug { length, diameter } => {
            let (d, l) = (diameter as f32, length as f32);
            cylinder(out, u * (at.base + d / 2.0) + Vec3::Z * at.z0, Vec3::Z, l, d / 2.0, state);
        }
        RadialKind::RailButton { height, diameter } => {
            let (h, d) = (height as f32, diameter as f32);
            let z = Vec3::Z * (at.z0 + d / 2.0);
            cylinder(out, u * (at.base - SINK) + z, u, h * STEM + SINK, d * STEM_WIDTH / 2.0, state);
            cylinder(out, u * (at.base + h * STEM) + z, u, h * (1.0 - STEM), d / 2.0, state);
        }
        RadialKind::EllipticalFin { .. } | RadialKind::FreeformFin { .. } => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fin() -> RadialKind {
        RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.07, sweep: 0.06, thickness: 0.004 }
    }

    #[test]
    fn angle_zero_is_up_and_increases_towards_x() {
        let (u, _) = dirs(0.0);
        assert!((u - Vec3::Y).length() < 1e-6);
        let (u, _) = dirs(std::f32::consts::FRAC_PI_2);
        assert!((u - Vec3::X).length() < 1e-6);
    }

    #[test]
    fn fin_corners_stand_on_the_surface_and_reach_the_span() {
        let c = fin_corners(&fin(), Placement { z0: 1.0, base: 0.04, angle: 0.0 }).unwrap();
        // Root edge sunk just under the surface, tip 70 mm out, sweep and chords along z.
        assert!((c[0].y - (0.04 - SINK)).abs() < 1e-6 && (c[0].z - 1.0).abs() < 1e-6);
        assert!((c[1].y - 0.11).abs() < 1e-6 && (c[1].z - 1.06).abs() < 1e-6);
        assert!((c[2].z - 1.11).abs() < 1e-6 && (c[3].z - 1.12).abs() < 1e-6);
        assert!(fin_corners(&RadialKind::LaunchLug { length: 0.05, diameter: 0.01 }, Placement { z0: 0.0, base: 0.04, angle: 0.0 }).is_none());
    }

    #[test]
    fn a_forward_swept_fin_has_its_tip_ahead_of_the_root() {
        let kind = RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.07, sweep: -0.04, thickness: 0.004 };
        let c = fin_corners(&kind, Placement { z0: 1.0, base: 0.04, angle: 0.0 }).unwrap();
        assert!((c[1].z - 0.96).abs() < 1e-6 && (c[2].z - 1.01).abs() < 1e-6 && (c[0].z - 1.0).abs() < 1e-6);
    }

    #[test]
    fn every_kind_emits_closed_outward_facing_triangles() {
        let forward_swept = RadialKind::FinSet { root_chord: 0.12, tip_chord: 0.05, span: 0.07, sweep: -0.04, thickness: 0.004 };
        let kinds = [fin(), forward_swept, RadialKind::LaunchLug { length: 0.05, diameter: 0.01 }, RadialKind::RailButton { height: 0.008, diameter: 0.011 }];
        for kind in kinds {
            let mut out = Vec::new();
            emit(&mut out, &kind, Placement { z0: 0.5, base: 0.04, angle: 0.7 }, 0.0);
            assert!(!out.is_empty() && out.len() % 3 == 0, "{kind:?}");
            // Counter-clockwise as seen from outside: the triangle's geometric normal agrees with its vertex normals.
            for tri in out.chunks(3) {
                let [a, b, c] = [0, 1, 2].map(|i| Vec3::from(tri[i].position));
                let n = Vec3::from(tri[0].normal) + Vec3::from(tri[1].normal) + Vec3::from(tri[2].normal);
                assert!((b - a).cross(c - a).dot(n) >= 0.0, "{kind:?}");
            }
        }
    }
}
