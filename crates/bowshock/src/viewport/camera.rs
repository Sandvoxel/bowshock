//! Turntable camera with an orthographic projection.
//!
//! World axes: `+Z` is the rocket's long axis (station increases from the nose tip at `z = 0`), `+Y` is up. Orthographic so
//! a side-on view has one scale everywhere and lines up with Station Lanes.

use std::f32::consts::{FRAC_PI_2, PI, TAU};

use glam::{Mat4, Vec3};
use iced::{Point, Size, Vector};

/// Depth range either side of the target, in metres.
const DEPTH: f32 = 50.0;

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
    /// Logical pixels per metre.
    pub scale: f32,
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

    /// World to clip space for a viewport of `size` logical pixels.
    pub fn view_proj(&self, size: Size) -> Mat4 {
        let (hw, hh) = (size.width / 2.0 / self.scale, size.height / 2.0 / self.scale);
        let eye = self.target + self.back() * DEPTH;
        use glam::camera::rh::{proj::directx::orthographic, view::look_at_mat4};
        orthographic(-hw, hw, -hh, hh, 0.0, 2.0 * DEPTH) * look_at_mat4(eye, self.target, self.up())
    }

    /// Screen position (logical px, y down) of a world point, for a viewport of `size`.
    pub fn project(&self, p: Vec3, size: Size) -> Point {
        let d = p - self.target;
        Point::new(size.width / 2.0 + d.dot(self.right()) * self.scale, size.height / 2.0 - d.dot(self.up()) * self.scale)
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

    /// Moves the view so the model follows a drag of `delta` px.
    pub fn pan(&mut self, delta: Vector) {
        self.target += (-self.right() * delta.x + self.up() * delta.y) / self.scale;
    }

    /// Zooms by `factor`, keeping the world point under `offset` (px from the viewport centre, y down) fixed.
    pub fn zoom(&mut self, factor: f32, offset: Vector) {
        let new_scale = (self.scale * factor).clamp(MIN_SCALE, MAX_SCALE);
        let shift = (self.right() * offset.x - self.up() * offset.y) * (1.0 / self.scale - 1.0 / new_scale);
        self.target += shift;
        self.scale = new_scale;
    }

    /// Centres `bounds` (world min/max) and scales it to fill `size` with a margin.
    pub fn fit(&mut self, min: Vec3, max: Vec3, size: Size) {
        let centre = (min + max) / 2.0;
        // Half-extent of the box along screen right and up.
        let half = (max - min) / 2.0;
        let extent = |axis: Vec3| half.x * axis.x.abs() + half.y * axis.y.abs() + half.z * axis.z.abs();
        let (ex, ey) = (extent(self.right()).max(1e-6), extent(self.up()).max(1e-6));
        self.target = centre;
        self.scale = (0.8 * (size.width / 2.0 / ex).min(size.height / 2.0 / ey)).clamp(MIN_SCALE, MAX_SCALE);
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
        }
    }
}

pub const MIN_SCALE: f32 = 20.0;
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
        Camera { target: Vec3::ZERO, yaw: -FRAC_PI_2, pitch: 0.0, roll: 0.0, scale: 1000.0 }
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
    fn projection_matches_the_gpu_transform() {
        let c = Camera { target: Vec3::new(0.1, -0.2, 0.3), yaw: 0.7, pitch: 0.4, roll: 0.5, scale: 900.0 };
        let size = Size::new(640.0, 480.0);
        let p = Vec3::new(0.25, 0.05, -0.4);
        let clip = c.view_proj(size) * p.extend(1.0);
        let screen = c.project(p, size);
        assert!((size.width / 2.0 * (1.0 + clip.x) - screen.x).abs() < 1e-2);
        assert!((size.height / 2.0 * (1.0 - clip.y) - screen.y).abs() < 1e-2);
        assert!((0.0..=1.0).contains(&clip.z));
    }

    #[test]
    fn zoom_keeps_the_point_under_the_cursor() {
        let mut c = side();
        let size = Size::new(800.0, 600.0);
        let offset = Vector::new(120.0, -80.0);
        let cursor = Point::new(400.0 + offset.x, 300.0 + offset.y);
        // World point under the cursor, on the target plane.
        let world = c.target + (c.right() * offset.x - c.up() * offset.y) / c.scale;
        c.zoom(2.5, offset);
        let after = c.project(world, size);
        assert!((after.x - cursor.x).abs() < 1e-3 && (after.y - cursor.y).abs() < 1e-3);
    }

    #[test]
    fn facing_top_keeps_an_axis_aligned_yaw() {
        let c = Camera { yaw: -1.4, ..side() };
        let (yaw, pitch) = c.facing(Vec3::Y);
        assert!((pitch - FRAC_PI_2).abs() < 1e-6);
        assert!((yaw + FRAC_PI_2).abs() < 1e-6);
    }

    #[test]
    fn lerp_turns_the_short_way() {
        let a = Camera { yaw: 3.0, ..side() };
        let b = Camera { yaw: -3.0, ..side() };
        let mid = a.lerp(&b, 0.5);
        assert!(mid.yaw.abs() > 3.0);
    }
}
