//! Turntable camera with a perspective projection that blends into an orthographic one.
//!
//! World axes: `+Z` is the rocket's long axis (station increases from the nose tip at `z = 0`), `+Y` is up. The 3D view is
//! perspective; Profile (2D) and a focused part are orthographic so a side-on view has one scale everywhere and lines up with
//! Station Lanes. [`Camera::ortho_blend`] runs from 0 (perspective) to 1 (orthographic) and is animated like the rest of the
//! pose, so the projection changes smoothly (a dolly-zoom: the eye backs away while the field of view narrows, and the picture
//! at the target plane never changes scale).

use std::f32::consts::{FRAC_PI_2, PI, TAU};

use glam::{Mat4, Vec3, Vec4};
use iced::{Point, Size, Vector};

/// Depth range either side of the target for the orthographic projection, in metres.
const DEPTH: f32 = 100.0;
/// Vertical field of view of the perspective projection.
const FOV: f32 = 30.0 * PI / 180.0;
/// The eye is never closer than this to the target (metres), so zooming in narrows the field of view instead of putting the
/// eye inside the rocket.
const MIN_DISTANCE: f32 = 2.0;
/// Blend at or above which the projection is exactly orthographic.
const ORTHO: f32 = 0.999;

/// The view: a point orbited around, a turntable orientation, a scale, and how orthographic the projection is.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Camera {
    /// The point orbited around and kept at the centre of the viewport.
    pub target: Vec3,
    /// Rotation about world `+Y`, radians.
    pub yaw: f32,
    /// Elevation, radians in `[-pi/2, pi/2]`.
    pub pitch: f32,
    /// Rotation of the picture about the view direction, radians; positive turns the model clockwise on screen.
    pub roll: f32,
    /// Logical pixels per metre on the plane through the target facing the camera.
    pub scale: f32,
    /// 0 for a perspective projection, 1 for an orthographic one.
    pub ortho_blend: f32,
}

impl Default for Camera {
    /// Looking at the origin from the left (side-on), perspective, 500 px per metre.
    fn default() -> Self {
        Camera { target: Vec3::ZERO, yaw: -FRAC_PI_2, pitch: 0.0, roll: 0.0, scale: 500.0, ortho_blend: 0.0 }
    }
}

impl Camera {
    /// Unit vector from the target towards the eye.
    pub fn back(&self) -> Vec3 {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        Vec3::new(cp * sy, sp, cp * cy)
    }

    /// Screen right in world space. The unrolled right comes from the yaw alone, so it stays defined looking straight down
    /// or up.
    pub fn right(&self) -> Vec3 {
        let (r0, u0) = self.unrolled();
        let (s, c) = self.roll.sin_cos();
        r0 * c + u0 * s
    }

    /// Screen up in world space.
    pub fn up(&self) -> Vec3 {
        let (r0, u0) = self.unrolled();
        let (s, c) = self.roll.sin_cos();
        u0 * c - r0 * s
    }

    fn unrolled(&self) -> (Vec3, Vec3) {
        let (sy, cy) = self.yaw.sin_cos();
        let r0 = Vec3::new(cy, 0.0, -sy);
        (r0, self.back().cross(r0))
    }

    /// Whether the projection is exactly orthographic.
    pub fn is_ortho(&self) -> bool {
        self.ortho_blend >= ORTHO
    }

    /// Distance from the eye to the target plane, metres. Perspective keeps the field of view at [`FOV`] until the eye would
    /// come closer than [`MIN_DISTANCE`]; blending towards orthographic pushes the eye out to infinity.
    pub fn distance(&self, size: Size) -> f32 {
        let persp = (size.height / 2.0 / self.scale / (FOV / 2.0).tan()).max(MIN_DISTANCE);
        persp / (1.0 - self.ortho_blend).max(1e-3)
    }

    /// The eye's position in world space (a point far along `back` when orthographic).
    pub fn eye(&self, size: Size) -> Vec3 {
        self.target + self.back() * self.distance(size)
    }

    /// World to clip space (depth 0 to 1) for a viewport of `size` logical pixels.
    pub fn view_proj(&self, size: Size) -> Mat4 {
        let d = self.distance(size);
        let (r, u, b) = (self.right(), self.up(), self.back());
        let (sx, sy) = (2.0 * self.scale / size.width.max(1.0), 2.0 * self.scale / size.height.max(1.0));
        // View space: x along right, y along up, depth `s` from the eye along -back.
        let view = Mat4::from_cols(
            Vec4::new(r.x, u.x, b.x, 0.0),
            Vec4::new(r.y, u.y, b.y, 0.0),
            Vec4::new(r.z, u.z, b.z, 0.0),
            Vec4::new(-r.dot(self.target), -u.dot(self.target), -b.dot(self.target), 1.0),
        );
        // `view` leaves z as the coordinate along `back` measured from the target (towards the eye is positive).
        let (near, far) = ((d - DEPTH).max(0.05 * d), d + DEPTH);
        let proj = if self.is_ortho() {
            // Depth `s = d - z`: map [near, far] to [0, 1] linearly.
            let k = 1.0 / (far - near);
            Mat4::from_cols(Vec4::new(sx, 0.0, 0.0, 0.0), Vec4::new(0.0, sy, 0.0, 0.0), Vec4::new(0.0, 0.0, -k, 0.0), Vec4::new(0.0, 0.0, (d - near) * k, 1.0))
        } else {
            // x_ndc = sx * d * x / s with w = s; z_ndc = a + c / s with z(near) = 0 and z(far) = 1.
            let a = far / (far - near);
            let c = -far * near / (far - near);
            // clip.z = a * s + c, s = d - z_view.
            Mat4::from_cols(
                Vec4::new(sx * d, 0.0, 0.0, 0.0),
                Vec4::new(0.0, sy * d, 0.0, 0.0),
                Vec4::new(0.0, 0.0, -a, -1.0),
                Vec4::new(0.0, 0.0, a * d + c, d),
            )
        };
        proj * view
    }

    /// Screen position (logical px, y down) of a world point, for a viewport of `size`.
    pub fn project(&self, p: Vec3, size: Size) -> Point {
        let d = p - self.target;
        let (x, y, z) = (d.dot(self.right()), d.dot(self.up()), d.dot(self.back()));
        let k = if self.is_ortho() {
            1.0
        } else {
            let dist = self.distance(size);
            dist / (dist - z).max(0.02 * dist)
        };
        Point::new(size.width / 2.0 + x * k * self.scale, size.height / 2.0 - y * k * self.scale)
    }

    /// The ray through screen point `at` (logical px in a viewport of `size`): its origin and unit direction. Perspective rays
    /// start at the eye; orthographic ones are parallel and start on the plane through the target.
    pub fn ray(&self, at: Point, size: Size) -> (Vec3, Vec3) {
        let (dx, dy) = ((at.x - size.width / 2.0) / self.scale, (at.y - size.height / 2.0) / self.scale);
        let on_plane = self.target + self.right() * dx - self.up() * dy;
        if self.is_ortho() {
            (on_plane, -self.back())
        } else {
            let eye = self.eye(size);
            (eye, (on_plane - eye).normalize())
        }
    }

    /// Orbits by a drag of `delta` px: dragging right turns the model right, dragging down tips its top towards you.
    /// The drag is taken in the unrolled frame so it follows the cursor whatever the roll.
    pub fn orbit(&mut self, delta: Vector) {
        const RAD_PER_PX: f32 = 0.008;
        let (s, c) = self.roll.sin_cos();
        let (dx, dy) = (delta.x * c + delta.y * s, -delta.x * s + delta.y * c);
        self.yaw = wrap(self.yaw - dx * RAD_PER_PX);
        self.pitch = (self.pitch + dy * RAD_PER_PX).clamp(-FRAC_PI_2, FRAC_PI_2);
    }

    /// Turns the picture about the view direction by `delta` px of horizontal drag (Alt + right-drag).
    pub fn roll_by(&mut self, delta: Vector) {
        self.roll = wrap(self.roll + delta.x * 0.008);
    }

    /// Moves the view so the model follows a drag of `delta` px.
    pub fn pan(&mut self, delta: Vector) {
        self.target += (-self.right() * delta.x + self.up() * delta.y) / self.scale;
    }

    /// Zooms by `factor`, keeping the world point under `offset` (px from the viewport centre, y down) on the target plane
    /// fixed.
    pub fn zoom(&mut self, factor: f32, offset: Vector) {
        let new_scale = (self.scale * factor).clamp(MIN_SCALE, MAX_SCALE);
        let shift = (self.right() * offset.x - self.up() * offset.y) * (1.0 / self.scale - 1.0 / new_scale);
        self.target += shift;
        self.scale = new_scale;
    }

    /// Centres `bounds` (world min/max) and scales it to fill `size` with a margin of a fifth on each axis.
    pub fn fit(&mut self, min: Vec3, max: Vec3, size: Size) {
        self.fit_in(min, max, size, Vector::new(size.width * 0.1, size.height * 0.1));
    }

    /// Centres `bounds` and scales it until it fills `size` less `margin` px on each side. The perspective projection is
    /// accounted for by refining the scale until the projected corners fit.
    pub fn fit_in(&mut self, min: Vec3, max: Vec3, size: Size, margin: Vector) {
        let centre = (min + max) / 2.0;
        let half = (max - min) / 2.0;
        let extent = |axis: Vec3| half.x * axis.x.abs() + half.y * axis.y.abs() + half.z * axis.z.abs();
        let (ex, ey) = (extent(self.right()).max(1e-6), extent(self.up()).max(1e-6));
        let (avail_x, avail_y) = ((size.width / 2.0 - margin.x).max(8.0), (size.height / 2.0 - margin.y).max(8.0));
        self.target = centre;
        self.scale = (avail_x / ex).min(avail_y / ey).clamp(MIN_SCALE, MAX_SCALE);
        if self.is_ortho() {
            return;
        }
        // The near parts of the box project larger than the target plane: shrink until every corner is inside.
        for _ in 0..8 {
            let mut worst = 1.0f32;
            for i in 0..8 {
                let corner = Vec3::new(if i & 1 == 0 { min.x } else { max.x }, if i & 2 == 0 { min.y } else { max.y }, if i & 4 == 0 { min.z } else { max.z });
                let p = self.project(corner, size);
                let (ox, oy) = ((p.x - size.width / 2.0).abs(), (p.y - size.height / 2.0).abs());
                worst = worst.min(avail_x / ox.max(1e-3)).min(avail_y / oy.max(1e-3));
            }
            if (worst - 1.0).abs() < 0.005 {
                break;
            }
            self.scale = (self.scale * worst).clamp(MIN_SCALE, MAX_SCALE);
        }
    }

    /// Yaw and pitch that look along `-dir` (`dir` points from the target to the eye). Looking straight down or up keeps
    /// the yaw, rounded to the nearest quarter turn so the view is axis-aligned.
    pub fn facing(&self, dir: Vec3) -> (f32, f32) {
        let dir = dir.normalize();
        let pitch = dir.y.clamp(-1.0, 1.0).asin();
        let yaw = if dir.y.abs() > 0.999 { (self.yaw / FRAC_PI_2).round() * FRAC_PI_2 } else { dir.x.atan2(dir.z) };
        (wrap(yaw), pitch)
    }

    /// The camera `t` of the way (0 to 1) from `self` to `to`, turning the short way round.
    pub fn lerp(&self, to: &Camera, t: f32) -> Camera {
        let yaw = self.yaw + wrap(to.yaw - self.yaw) * t;
        let roll = self.roll + wrap(to.roll - self.roll) * t;
        Camera {
            target: self.target.lerp(to.target, t),
            yaw: wrap(yaw),
            pitch: self.pitch + (to.pitch - self.pitch) * t,
            roll: wrap(roll),
            scale: self.scale * (to.scale / self.scale).powf(t),
            ortho_blend: self.ortho_blend + (to.ortho_blend - self.ortho_blend) * t,
        }
    }
}

/// Smallest scale, logical px per metre.
pub const MIN_SCALE: f32 = 20.0;
/// Largest scale, logical px per metre.
pub const MAX_SCALE: f32 = 40_000.0;

/// Wraps an angle into `(-pi, pi]`.
fn wrap(a: f32) -> f32 {
    let a = (a + PI).rem_euclid(TAU) - PI;
    if a <= -PI {
        a + TAU
    } else {
        a
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn side() -> Camera {
        Camera { target: Vec3::ZERO, yaw: -FRAC_PI_2, pitch: 0.0, roll: 0.0, scale: 1000.0, ortho_blend: 1.0 }
    }

    fn persp() -> Camera {
        Camera { target: Vec3::new(0.1, -0.2, 0.3), yaw: 0.7, pitch: 0.4, roll: 0.5, scale: 900.0, ortho_blend: 0.0 }
    }

    #[test]
    fn side_view_puts_the_nose_left_and_up_up() {
        let c = side();
        let size = Size::new(800.0, 600.0);
        let tip = c.project(Vec3::new(0.0, 0.0, -0.1), size);
        let tail = c.project(Vec3::new(0.0, 0.0, 0.1), size);
        let top = c.project(Vec3::new(0.0, 0.1, 0.0), size);
        assert!(tip.x < tail.x);
        assert!(top.y < 300.0);
    }

    #[test]
    fn projection_matches_the_gpu_transform_in_every_blend() {
        let size = Size::new(640.0, 480.0);
        let p = Vec3::new(0.25, 0.05, -0.4);
        for blend in [0.0, 0.3, 0.9, 1.0] {
            let c = Camera { ortho_blend: blend, ..persp() };
            let clip = c.view_proj(size) * p.extend(1.0);
            let ndc = clip / clip.w;
            let screen = c.project(p, size);
            assert!((size.width / 2.0 * (1.0 + ndc.x) - screen.x).abs() < 5e-2, "blend {blend}");
            assert!((size.height / 2.0 * (1.0 - ndc.y) - screen.y).abs() < 5e-2, "blend {blend}");
            assert!((0.0..=1.0).contains(&ndc.z), "blend {blend}: depth {}", ndc.z);
        }
    }

    #[test]
    fn the_target_plane_keeps_its_scale_whatever_the_blend() {
        let size = Size::new(800.0, 600.0);
        for blend in [0.0, 0.5, 1.0] {
            let c = Camera { ortho_blend: blend, scale: 700.0, ..side() };
            let a = c.project(c.target, size);
            let b = c.project(c.target + c.right() * 0.1, size);
            assert!(((b.x - a.x) - 70.0).abs() < 1e-3, "blend {blend}");
        }
    }

    #[test]
    fn nearer_points_look_larger_in_perspective_only() {
        let size = Size::new(800.0, 600.0);
        let mut c = persp();
        let across = |c: &Camera, towards_eye: f32| {
            let o = c.target + c.back() * towards_eye;
            let (a, b) = (c.project(o, size), c.project(o + c.right() * 0.1, size));
            b.x - a.x
        };
        c.ortho_blend = 0.0;
        assert!(across(&c, 0.5) > across(&c, -0.5) * 1.1);
        c.ortho_blend = 1.0;
        assert!((across(&c, 0.5) - across(&c, -0.5)).abs() < 1e-3);
    }

    #[test]
    fn rays_run_through_their_screen_point() {
        let size = Size::new(640.0, 480.0);
        for blend in [0.0, 1.0] {
            let c = Camera { ortho_blend: blend, ..persp() };
            let at = Point::new(400.0, 150.0);
            let (o, dir) = c.ray(at, size);
            // A point along the ray projects back to the same pixel.
            let p = c.project(o + dir * 1.7, size);
            assert!((p.x - at.x).abs() < 0.05 && (p.y - at.y).abs() < 0.05, "blend {blend}: {p:?}");
        }
    }

    #[test]
    fn zoom_keeps_the_point_under_the_cursor() {
        for blend in [0.0, 1.0] {
            let mut c = Camera { ortho_blend: blend, ..side() };
            let size = Size::new(800.0, 600.0);
            let offset = Vector::new(120.0, -80.0);
            let cursor = Point::new(400.0 + offset.x, 300.0 + offset.y);
            // World point under the cursor, on the target plane.
            let world = c.target + (c.right() * offset.x - c.up() * offset.y) / c.scale;
            c.zoom(2.5, offset);
            let after = c.project(world, size);
            assert!((after.x - cursor.x).abs() < 1e-3 && (after.y - cursor.y).abs() < 1e-3);
        }
    }

    #[test]
    fn fit_keeps_the_box_inside_the_viewport_in_perspective() {
        let size = Size::new(900.0, 560.0);
        let (min, max) = (Vec3::new(-0.04, -0.07, 0.0), Vec3::new(0.04, 0.07, 1.2));
        let mut c = Camera { yaw: 0.8, pitch: 0.4, ..side() };
        c.ortho_blend = 0.0;
        c.fit(min, max, size);
        for i in 0..8 {
            let corner = Vec3::new(if i & 1 == 0 { min.x } else { max.x }, if i & 2 == 0 { min.y } else { max.y }, if i & 4 == 0 { min.z } else { max.z });
            let p = c.project(corner, size);
            assert!(p.x >= 0.0 && p.x <= size.width && p.y >= 0.0 && p.y <= size.height, "{p:?}");
        }
    }

    #[test]
    fn facing_top_keeps_an_axis_aligned_yaw() {
        let c = Camera { yaw: -1.4, ..side() };
        let (yaw, pitch) = c.facing(Vec3::Y);
        assert!((pitch - FRAC_PI_2).abs() < 1e-6);
        assert!((yaw + FRAC_PI_2).abs() < 1e-6);
    }

    #[test]
    fn lerp_turns_the_short_way_and_blends_the_projection() {
        let a = Camera { yaw: 3.0, ortho_blend: 0.0, ..side() };
        let b = Camera { yaw: -3.0, ..side() };
        let mid = a.lerp(&b, 0.5);
        assert!(mid.yaw.abs() > 3.0);
        assert!((mid.ortho_blend - 0.5).abs() < 1e-6);
    }
}
