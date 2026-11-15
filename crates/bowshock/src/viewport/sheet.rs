//! The viewport ground: a drawing sheet with a 10 mm minor / 100 mm major grid, the rocket's centreline (the projected `z`
//! axis, dashed), corner registration marks and a scale bar (design/README.md section 8).
//!
//! The grid lies in a real world plane through the rocket axis, so it pans, zooms and orbits with the model: the
//! coordinate plane (YZ, XZ or XY) that faces the camera most squarely. Side-on it is a flat drawing sheet with lines at
//! exact stations from the nose tip; orbited it foreshortens like the model does.

use glam::Vec3;
use iced::widget::canvas::{self, Frame, Geometry, LineDash, Path, Stroke, Text};
use iced::{alignment, mouse, Point, Rectangle, Renderer, Size, Theme, Vector};

use super::Camera;
use crate::theme::{self, MONO};
use crate::tokens::{SPACE_4, TEXT_LABEL};

pub struct Sheet {
    pub camera: Camera,
}

impl<Message> canvas::Program<Message> for Sheet {
    type State = ();

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let size = bounds.size();
        let mut frame = Frame::new(renderer, size);
        let Size { width: w, height: h } = size;
        frame.fill_rectangle(Point::ORIGIN, size, t.bg_000);

        self.grid(&mut frame, size, [(0.010, t.grid_minor), (0.100, t.grid_major)]);

        // Centreline: the projected z axis, extended across the viewport. Skipped when looking down the axis.
        let a = self.camera.project(Vec3::new(0.0, 0.0, -1.0), size);
        let b = self.camera.project(Vec3::new(0.0, 0.0, 1.0), size);
        let dir = b - a;
        let len = (dir.x * dir.x + dir.y * dir.y).sqrt();
        if len > 1.0 {
            let reach = (w + h) / len * 2.0;
            let axis = Path::line(a - dir * reach, b + dir * reach);
            let dashed = Stroke {
                line_dash: LineDash { segments: &[8.0, 4.0, 2.0, 4.0], offset: 0 },
                ..Stroke::default().with_color(t.grid_major.scale_alpha(2.0)).with_width(1.0)
            };
            frame.stroke(&axis, dashed);
        }

        // Registration marks, 20 px legs.
        let leg = 20.0;
        let inset = SPACE_4 / 2.0;
        let marks = Path::new(|p| {
            for (cx, cy, sx, sy) in [(inset, inset, 1.0, 1.0), (w - inset, inset, -1.0, 1.0), (inset, h - inset, 1.0, -1.0), (w - inset, h - inset, -1.0, -1.0)]
            {
                p.move_to(Point::new(cx + sx * leg, cy));
                p.line_to(Point::new(cx, cy));
                p.line_to(Point::new(cx, cy + sy * leg));
            }
        });
        frame.stroke(&marks, Stroke::default().with_color(t.line_200).with_width(1.0));

        // Scale bar, bottom right: the largest of 1/10/100/1000 mm that fits in 160 px.
        let (metres, label) = [(1.0, "1 m"), (0.1, "100 mm"), (0.01, "10 mm"), (0.001, "1 mm")]
            .into_iter()
            .find(|(m, _)| m * self.camera.scale <= 160.0)
            .unwrap_or((0.001, "1 mm"));
        let len = metres * self.camera.scale;
        let base = Point::new(w - SPACE_4 * 2.0, h - SPACE_4 * 2.0);
        let bar = Path::new(|p| {
            p.move_to(base + Vector::new(-len, -4.0));
            p.line_to(base + Vector::new(-len, 0.0));
            p.line_to(base);
            p.line_to(base + Vector::new(0.0, -4.0));
        });
        frame.stroke(&bar, Stroke::default().with_color(t.ink_muted).with_width(1.0));
        frame.fill_text(Text {
            content: label.into(),
            position: base + Vector::new(-len / 2.0, -6.0),
            color: t.ink_muted,
            size: TEXT_LABEL.into(),
            font: MONO,
            align_x: alignment::Horizontal::Center.into(),
            align_y: alignment::Vertical::Bottom,
            ..Text::default()
        });

        vec![frame.into_geometry()]
    }
}

impl Sheet {
    /// Strokes grid lines at each `(step in metres, colour)` in the world coordinate plane that best faces the camera.
    fn grid(&self, frame: &mut Frame, size: Size, levels: [(f32, iced::Color); 2]) {
        let cam = &self.camera;
        let back = cam.back();
        // Plane normal = the world axis most aligned with the view direction; (a, b) span the plane.
        let (n, a, b) = [(Vec3::X, Vec3::Z, Vec3::Y), (Vec3::Y, Vec3::Z, Vec3::X), (Vec3::Z, Vec3::X, Vec3::Y)]
            .into_iter()
            .max_by(|p, q| p.0.dot(back).abs().total_cmp(&q.0.dot(back).abs()))
            .unwrap_or((Vec3::X, Vec3::Z, Vec3::Y));
        let facing = n.dot(back);

        // Plane coordinates of the viewport corners: cast each corner along the view direction onto the plane.
        let corner = |sx: f32, sy: f32| {
            let p0 = cam.target + cam.right() * ((sx - size.width / 2.0) / cam.scale) - cam.up() * ((sy - size.height / 2.0) / cam.scale);
            let p = p0 - back * (n.dot(p0) / facing);
            (p.dot(a), p.dot(b))
        };
        let corners = [corner(0.0, 0.0), corner(size.width, 0.0), corner(0.0, size.height), corner(size.width, size.height)];
        let (mut a0, mut a1, mut b0, mut b1) = (f32::MAX, f32::MIN, f32::MAX, f32::MIN);
        for (ca, cb) in corners {
            (a0, a1, b0, b1) = (a0.min(ca), a1.max(ca), b0.min(cb), b1.max(cb));
        }

        let origin = cam.project(Vec3::ZERO, size);
        let px = |v: Vec3| {
            let q = cam.project(v, size);
            ((q.x - origin.x).powi(2) + (q.y - origin.y).powi(2)).sqrt()
        };
        for (step, color) in levels {
            // Skip a level whose lines would be closer than 6 px on screen in either direction.
            if px(a * step).min(px(b * step)) < 6.0 {
                continue;
            }
            let (i0, i1) = ((a0 / step).floor() as i64, (a1 / step).ceil() as i64);
            let (j0, j1) = ((b0 / step).floor() as i64, (b1 / step).ceil() as i64);
            if (i1 - i0) + (j1 - j0) > 2000 {
                continue;
            }
            let lines = Path::new(|p| {
                for i in i0..=i1 {
                    let u = a * (i as f32 * step);
                    p.move_to(cam.project(u + b * b0, size));
                    p.line_to(cam.project(u + b * b1, size));
                }
                for j in j0..=j1 {
                    let v = b * (j as f32 * step);
                    p.move_to(cam.project(v + a * a0, size));
                    p.line_to(cam.project(v + a * a1, size));
                }
            });
            frame.stroke(&lines, Stroke::default().with_color(color).with_width(1.0));
        }
    }
}
