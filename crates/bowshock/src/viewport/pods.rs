//! Pods in the viewport: where each copy of a pod stands, a camera that looks at a copy in the pod's own frame, and the pod's
//! mesh carried to its place.
//!
//! A pod copy is its [`PodBody`](bowshock_model::PodBody) moved rigidly: turned about the core axis by the copy's angle, out by
//! the axis distance and along by the pod's station ([`Pose`]). The model edits and describes a pod as a small rocket of its own
//! ([`Rocket::pod_rocket`]), and every viewport routine already works from a rocket and a [`Camera`], so a pod is seen by running
//! the same routine on that small rocket with [`Camera::in_frame`], a camera that projects the pod's own coordinates to the
//! screen exactly where the world camera would put the moved copy. Picking, handles, brackets and the surface a fin lands on
//! all use it; the mesh carries the pod's own mesh by the same pose.

use bowshock_model::{PartId, PodFrame, RadialKind, RadialPart, Rocket};
use glam::Vec3;

use super::camera::Camera;
use super::mesh::{Marks, Mesh, PartBox};
use super::scene::GHOST;
use super::solids::LAYER_RADIAL;

/// How far (metres) a pod moves out in the exploded view, so its copies leave the core.
const PULL: f32 = 0.06;

/// Where one copy of a pod stands: the pod's own `(x, y, z)` goes to `x t + (y + axis) u + (z + z0) Z`, with `u` the direction
/// from the core axis out to the copy (0 is +Y, increasing towards +X) and `t` the tangent of increasing angle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    /// Station of the pod's fore face, metres.
    pub z0: f32,
    /// Distance from the core axis to the pod's axis, metres.
    pub axis: f32,
    /// Angle of the copy round the core.
    pub angle: f32,
}

impl Pose {
    /// Unit vector from the core axis out towards the copy.
    pub fn out(&self) -> Vec3 {
        let (s, c) = self.angle.sin_cos();
        Vec3::new(s, c, 0.0)
    }

    /// Unit tangent of increasing angle.
    pub fn tangent(&self) -> Vec3 {
        let (s, c) = self.angle.sin_cos();
        Vec3::new(c, -s, 0.0)
    }

    /// The point `p` of the pod's own frame in the world.
    pub fn to_world(self, p: Vec3) -> Vec3 {
        self.dir_to_world(p) + self.out() * self.axis + Vec3::Z * self.z0
    }

    /// The direction `v` of the pod's own frame in the world.
    pub fn dir_to_world(&self, v: Vec3) -> Vec3 {
        self.tangent() * v.x + self.out() * v.y + Vec3::Z * v.z
    }

    /// The world direction `v` in the pod's own frame.
    pub fn dir_to_local(&self, v: Vec3) -> Vec3 {
        Vec3::new(v.dot(self.tangent()), v.dot(self.out()), v.z)
    }

    /// The world point `p` in the pod's own frame.
    pub fn to_local(self, p: Vec3) -> Vec3 {
        self.dir_to_local(p - self.out() * self.axis - Vec3::Z * self.z0)
    }
}

/// The pose of each copy of a pod standing as `frame` says.
pub fn poses(frame: &PodFrame) -> Vec<Pose> {
    frame.angles.iter().map(|&angle| Pose { z0: frame.station as f32, axis: frame.axis_radius as f32, angle: angle as f32 }).collect()
}

impl Camera {
    /// This camera as it sees the pod's own coordinates: `project` of a point of the pod's frame gives the screen position that
    /// this camera's `project` gives its place in the world, and rays and directions follow. Scale and projection are kept.
    pub fn in_frame(&self, pose: Pose) -> Camera {
        let (right, back) = (pose.dir_to_local(self.right()), pose.dir_to_local(self.back()));
        let pitch = back.y.clamp(-1.0, 1.0).asin();
        // Straight up or down the yaw is free; keep the camera's own.
        let yaw = if pitch.cos().abs() > 1e-6 { back.x.atan2(back.z) } else { self.yaw };
        let (sy, cy) = yaw.sin_cos();
        let r0 = Vec3::new(cy, 0.0, -sy);
        let u0 = back.cross(r0);
        let roll = right.dot(u0).atan2(right.dot(r0));
        Camera { target: pose.to_local(self.target), yaw, pitch, roll, scale: self.scale, ortho_blend: self.ortho_blend }
    }
}

/// A part that is inside a pod, seen through the pod's frame: the pod as a rocket of its own and a camera for each copy.
#[derive(Debug, Clone)]
pub struct PodView {
    /// The pod set.
    pub pod: PartId,
    /// The pod as a rocket of its own (ids kept).
    pub sub: Rocket,
    /// The pose of each copy.
    pub poses: Vec<Pose>,
    /// The world camera in each copy's frame.
    pub cameras: Vec<Camera>,
}

impl PodView {
    /// The view of pod `pod` of `rocket` through `camera`; `None` if it is not a pod of the rocket.
    pub fn of_pod(rocket: &Rocket, pod: &RadialPart, camera: &Camera) -> Option<PodView> {
        let frame = rocket.pod_frame(pod)?;
        let poses = poses(&frame);
        let cameras = poses.iter().map(|&p| camera.in_frame(p)).collect();
        Some(PodView { pod: pod.id, sub: rocket.pod_rocket(pod.id)?, poses, cameras })
    }

    /// The view of the pod holding part `id`; `None` for a part of the core.
    pub fn of_part(rocket: &Rocket, id: PartId, camera: &Camera) -> Option<PodView> {
        PodView::of_pod(rocket, rocket.pod_of(id)?, camera)
    }
}

/// Appends the meshes of the rocket's pods (and the ghost of a pod being dragged in, or of a part being dragged into one) to
/// `outer`, each copy carried to its place. `parent_z` gives a core part's station and exploded-view shift as drawn.
///
/// A pod is drawn as the radial layer: whole in every view (not cut with the core's wedge), its own cut faces and internals
/// left out.
pub(super) fn append(
    vertices: &mut Vec<super::solids::GpuVertex>,
    parts: &mut Vec<PartBox>,
    rocket: &Rocket,
    marks: &Marks,
    parent_z: &dyn Fn(PartId) -> Option<(f64, f32)>,
) {
    let mut carry = |pod: &RadialPart, body: Option<&bowshock_model::PodBody>, ghost: bool| {
        let Some((parent_z, parent_dz)) = parent_z(pod.parent) else {
            return;
        };
        let Some(frame) = rocket.pod_frame(pod) else {
            return;
        };
        let sub = match body {
            Some(body) => body.to_rocket(),
            None => match rocket.pod_rocket(pod.id) {
                Some(sub) => sub,
                None => return,
            },
        };
        let mut sub_marks = Marks {
            hovered: marks.hovered,
            selected: marks.selected,
            also_selected: marks.also_selected,
            flash: marks.flash,
            cut: super::mesh::Cut { centre: 0.0, span: 0.0 },
            ..Marks::default()
        };
        if !ghost {
            sub_marks.ghost = marks.ghost_pod.filter(|g| g.0 == pod.id).map(|g| (g.1, g.2));
            sub_marks.ghost_radial = marks.ghost_radial.filter(|g| sub.part(g.parent).is_some() && rocket.pod_of(g.parent).is_some_and(|p| p.id == pod.id));
        }
        let mesh = Mesh::from_rocket(&sub, sub_marks);
        let z0 = (parent_z + pod.offset) as f32;
        for pose in poses(&frame) {
            let pose = Pose { z0, ..pose };
            let pull = pose.out() * PULL + Vec3::Z * parent_dz;
            for range in [mesh.inner.clone(), mesh.outer.clone()] {
                for v in &mesh.vertices[range.start as usize..range.end as usize] {
                    let mut w = *v;
                    w.position = pose.to_world(Vec3::from(v.position)).to_array();
                    w.normal = pose.dir_to_world(Vec3::from(v.normal)).to_array();
                    w.explode = (pose.dir_to_world(Vec3::from(v.explode)) + pull).to_array();
                    w.layer = LAYER_RADIAL;
                    if ghost {
                        w.state = GHOST;
                    }
                    vertices.push(w);
                }
            }
            for b in &mesh.parts {
                let corners = (0..8).map(|i| {
                    let c = Vec3::new(
                        if i & 1 == 0 { b.min.x } else { b.max.x },
                        if i & 2 == 0 { b.min.y } else { b.max.y },
                        if i & 4 == 0 { b.min.z } else { b.max.z },
                    );
                    pose.to_world(c)
                });
                let (min, max) = corners.fold((Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY)), |(lo, hi), c| (lo.min(c), hi.max(c)));
                let explode = pose.dir_to_world(b.explode) + pull;
                match parts.iter_mut().find(|p| p.id == b.id) {
                    Some(p) => {
                        p.min = p.min.min(min);
                        p.max = p.max.max(max);
                    }
                    None => parts.push(PartBox { id: b.id, min, max, explode }),
                }
            }
        }
    };
    for pod in rocket.pods() {
        carry(pod, None, false);
    }
    if let Some(g) = marks.ghost_radial.filter(|g| g.kind.is_pod()) {
        if let RadialKind::Pod { body, .. } = g.kind {
            let offset = rocket.clamp_offset(g.parent, g.offset, g.kind).unwrap_or(0.0);
            let pod = RadialPart::new(PartId::MAX, g.kind.clone(), g.parent, offset, g.angle, g.count);
            carry(&pod, Some(body), true);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{NoseShape, PartKind};
    use glam::Vec2;
    use iced::Size;

    const SIZE: Size = Size::new(800.0, 600.0);

    #[test]
    fn a_pose_round_trips_and_puts_the_pod_out_along_its_angle() {
        let pose = Pose { z0: 0.6, axis: 0.075, angle: 1.0 };
        let p = Vec3::new(0.01, -0.02, 0.3);
        assert!((pose.to_local(pose.to_world(p)) - p).length() < 1e-6);
        // The pod's own origin is on its axis, `axis` out from the core's, in direction `angle`.
        let o = pose.to_world(Vec3::ZERO);
        assert!((o - Vec3::new(0.075 * 1.0_f32.sin(), 0.075 * 1.0_f32.cos(), 0.6)).length() < 1e-6);
        // The pod's +Y points away from the core.
        assert!((pose.dir_to_world(Vec3::Y) - pose.out()).length() < 1e-6);
    }

    #[test]
    fn a_camera_in_the_pods_frame_projects_to_the_same_pixels() {
        let cameras = [
            Camera { target: Vec3::new(0.1, -0.2, 0.5), yaw: 0.7, pitch: 0.4, roll: 0.5, scale: 900.0, ortho_blend: 0.0 },
            Camera { target: Vec3::new(0.0, 0.0, 0.5), yaw: -std::f32::consts::FRAC_PI_2, pitch: 0.0, roll: 0.0, scale: 1000.0, ortho_blend: 1.0 },
            Camera { target: Vec3::new(0.0, 0.0, 0.3), yaw: 2.0, pitch: 1.2, roll: -0.3, scale: 400.0, ortho_blend: 0.4 },
            Camera { target: Vec3::ZERO, yaw: 0.3, pitch: std::f32::consts::FRAC_PI_2, roll: 0.2, scale: 400.0, ortho_blend: 0.0 },
        ];
        for pose in [Pose { z0: 0.6, axis: 0.075, angle: 1.0 }, Pose { z0: 0.1, axis: 0.05, angle: 4.0 }, Pose { z0: 0.0, axis: 0.0, angle: 0.0 }] {
            for camera in cameras {
                let local = camera.in_frame(pose);
                for p in [Vec3::ZERO, Vec3::new(0.02, 0.03, 0.4), Vec3::new(-0.04, 0.01, 0.1)] {
                    let (a, b) = (camera.project(pose.to_world(p), SIZE), local.project(p, SIZE));
                    assert!(a.distance(b) < 0.05, "{camera:?} {pose:?} {p:?}: {a:?} vs {b:?}");
                }
                // Rays are the same lines.
                let (o, d) = camera.ray(iced::Point::new(300.0, 200.0), SIZE);
                let (ol, dl) = local.ray(iced::Point::new(300.0, 200.0), SIZE);
                assert!((pose.dir_to_world(dl) - d).length() < 1e-4 && (pose.to_world(ol) - o).length() < 1e-3);
            }
        }
    }

    fn pod_rocket() -> (Rocket, PartId) {
        let mut r = Rocket::default();
        r.insert(0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.3, diameter: 0.1 }).unwrap();
        let tube = r.insert(1, PartKind::BodyTube { length: 1.0, diameter: 0.1 }).unwrap();
        let kind = r.default_pod();
        let pod = r.attach_radial(tube, 0.3, 0.0, 2, kind).unwrap();
        (r, pod)
    }

    #[test]
    fn the_mesh_has_every_copy_of_the_pod_at_its_place() {
        let (r, pod) = pod_rocket();
        let mesh = Mesh::from_rocket(&r, Marks::default());
        // Pods stand at +Y and -Y (angles 0 and pi), touching the 50 mm core: axes 75 mm out, 25 mm radius.
        let body = r.radial(pod).unwrap().kind.pod_body().unwrap();
        let tube = body.parts()[1].id;
        let b = mesh.part_bounds(tube).expect("the pod's tube is a part of the mesh");
        assert!((b.max.y - 0.1).abs() < 1e-4 && (b.min.y + 0.1).abs() < 1e-4, "{b:?}");
        assert!(b.max.x < 0.03 && b.min.x > -0.03);
        // Stations: the pod starts 0.3 down its parent, which starts 0.3 down the core; its nose is 0.15 long.
        assert!((b.min.z - (0.6 + 0.15)).abs() < 1e-4 && (b.max.z - (0.6 + 0.75)).abs() < 1e-4);
        // Selecting a part of the pod marks it in every copy; the core is untouched.
        let marked = Mesh::from_rocket(&r, Marks { selected: Some(tube), ..Marks::default() });
        assert!(marked.vertices.iter().filter(|v| v.state == super::super::scene::SELECTED).any(|v| v.position[1] > 0.06));
        assert!(marked.vertices.iter().filter(|v| v.state == super::super::scene::SELECTED).any(|v| v.position[1] < -0.06));
        // A pod being dragged in is a ghost.
        let ghost_kind = r.default_pod();
        let with = Mesh::from_rocket(
            &r,
            Marks {
                ghost_radial: Some(super::super::mesh::GhostRadial { kind: &ghost_kind, parent: r.parts()[1].id, offset: 0.0, angle: 1.0, count: 3 }),
                ..Marks::default()
            },
        );
        assert!(with.vertices.iter().any(|v| v.state == GHOST) && with.vertices.len() > mesh.vertices.len());
        let _ = Vec2::ZERO;
    }

    /// Side-on and orthographic: the nose points left, +Z runs right, +Y is up.
    fn side() -> Camera {
        Camera { target: Vec3::new(0.0, 0.0, 0.8), yaw: -std::f32::consts::FRAC_PI_2, pitch: 0.0, roll: 0.0, scale: 800.0, ortho_blend: 1.0 }
    }

    #[test]
    fn a_click_picks_a_part_of_the_pod_and_a_fin_lands_on_the_pods_surface() {
        use super::super::pick::{pick, surface_target, Pick, Snapping};
        let (r, pod) = pod_rocket();
        let core_tube = r.parts()[1].id;
        let pod_tube = r.radial(pod).unwrap().kind.pod_body().unwrap().parts()[1].id;
        let cam = side();
        // The upper pod's axis is 75 mm above the core's; its tube spans stations 0.75 to 1.35.
        let on_pod = cam.project(Vec3::new(0.0, 0.075, 1.05), SIZE);
        assert_eq!(pick(&r, &cam, on_pod, SIZE), Some(Pick::Axial(pod_tube)));
        let on_core = cam.project(Vec3::new(0.0, 0.0, 1.05), SIZE);
        assert_eq!(pick(&r, &cam, on_core, SIZE), Some(Pick::Axial(core_tube)));
        // A fin dropped on the pod's top lands on the pod's tube, at the pod's own outward angle (0), with its offset along it.
        let fin = RadialKind::FinSet { root_chord: 0.08, tip_chord: 0.03, span: 0.04, sweep: 0.03, thickness: 0.002 };
        let top = cam.project(Vec3::new(0.0, 0.1, 1.2), SIZE);
        let target = surface_target(&r, &cam, top, SIZE, &fin, Snapping::default()).unwrap();
        assert_eq!(target.parent, pod_tube);
        assert!(target.angle.abs() < 0.05 || (target.angle - std::f64::consts::TAU).abs() < 0.05, "{}", target.angle);
        assert!((target.offset - (1.2 - 0.75 - 0.04)).abs() < 0.01, "{}", target.offset);
        // A pod never lands on a pod: over a pod and no core it has no target; over the core it lands on the core.
        let kind = r.default_pod();
        assert!(surface_target(&r, &cam, on_pod, SIZE, &kind, Snapping::default()).is_none());
        assert_eq!(surface_target(&r, &cam, on_core, SIZE, &kind, Snapping::default()).map(|t| t.parent), Some(core_tube));
    }

    #[test]
    fn the_pod_sets_handles_are_position_gap_and_angle() {
        use super::super::handles::{self, Context};
        use bowshock_model::Dim;
        use iced::{Point, Rectangle, Vector};
        let (r, pod) = pod_rocket();
        let cam = side();
        let cx = Context {
            rocket: &r,
            camera: &cam,
            size: SIZE,
            instance: 0,
            bounds: Rectangle::new(Point::new(300.0, 200.0), Size::new(300.0, 100.0)),
            cube: false,
        };
        let placed = handles::place(&cx, pod);
        let dims: Vec<Dim> = placed.iter().map(|h| h.spec.dim).collect();
        assert_eq!(dims, vec![Dim::Position, Dim::Gap, Dim::Angle]);
        // Dragging the gap handle 20 px up (at 800 px/m) moves the pod 25 mm out from the core.
        let gap = placed.iter().find(|h| h.spec.dim == Dim::Gap).unwrap();
        let v = handles::value_after(gap, 0.0, gap.at, gap.at + Vector::new(0.0, -20.0));
        assert!((v - 0.025).abs() < 1e-3, "{v}");
        // The position handle slides along the core.
        let position = placed.iter().find(|h| h.spec.dim == Dim::Position).unwrap();
        let v = handles::value_after(position, 0.3, position.at, position.at + Vector::new(80.0, 0.0));
        assert!((v - 0.4).abs() < 1e-3, "{v}");
    }
}
