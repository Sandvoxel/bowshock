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
    /// The rocket's axial extent (nose tip and tail, metres along `z`); the centreline runs a little past both ends.
    pub span: Option<(f32, f32)>,
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

        // Centreline: the rocket axis from a tenth of its length ahead of the nose to a tenth behind the tail, fixed in the
        // world so it reads as part of the drawing rather than a camera-dependent guide. Skipped when looking down the axis.
        if let Some((z0, z1)) = self.span.filter(|(z0, z1)| z1 > z0) {
            let pad = (z1 - z0) * 0.1;
            let a = self.camera.project(Vec3::new(0.0, 0.0, z0 - pad), size);
            let b = self.camera.project(Vec3::new(0.0, 0.0, z1 + pad), size);
            if (b.x - a.x).hypot(b.y - a.y) > 1.0 {
                let dashed = Stroke {
                    line_dash: LineDash { segments: &[8.0, 4.0, 2.0, 4.0], offset: 0 },
                    ..Stroke::default().with_color(t.grid_major.scale_alpha(2.0)).with_width(1.0)
                };
                frame.stroke(&Path::line(a, b), dashed);
            }
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
    /// Strokes grid lines at each `(step in metres, colour)` in the world coordinate plane that best faces the camera. Lines are
    /// cut into short runs whose opacity falls off with distance from the target, so the grid fades out towards the edges (and
    /// into the distance in perspective).
    fn grid(&self, frame: &mut Frame, size: Size, levels: [(f32, iced::Color); 2]) {
        const RUNS: usize = 16;
        const BUCKETS: usize = 4;
        let cam = &self.camera;
        let back = cam.back();
        // Plane normal = the world axis most aligned with the view direction; (a, b) span the plane.
        let (n, a, b) = [(Vec3::X, Vec3::Z, Vec3::Y), (Vec3::Y, Vec3::Z, Vec3::X), (Vec3::Z, Vec3::X, Vec3::Y)]
            .into_iter()
            .max_by(|p, q| p.0.dot(back).abs().total_cmp(&q.0.dot(back).abs()))
            .unwrap_or((Vec3::X, Vec3::Z, Vec3::Y));

        // Plane coordinates of the viewport corners: cast each corner's ray onto the plane through the target.
        let corner = |sx: f32, sy: f32| {
            let (o, d) = cam.ray(Point::new(sx, sy), size);
            let denom = n.dot(d);
            let t = if denom.abs() > 1e-4 { n.dot(cam.target - o) / denom } else { 0.0 };
            let p = o + d * t;
            (p.dot(a), p.dot(b))
        };
        let corners = [corner(0.0, 0.0), corner(size.width, 0.0), corner(0.0, size.height), corner(size.width, size.height)];
        let (mut a0, mut a1, mut b0, mut b1) = (f32::MAX, f32::MIN, f32::MAX, f32::MIN);
        for (ca, cb) in corners {
            (a0, a1, b0, b1) = (a0.min(ca), a1.max(ca), b0.min(cb), b1.max(cb));
        }
        // Keep the extent finite however grazing the view.
        let reach = 0.5 * size.width.hypot(size.height) / cam.scale * 3.0;
        let (centre_a, centre_b) = (cam.target.dot(a), cam.target.dot(b));
        (a0, a1, b0, b1) = (a0.max(centre_a - reach), a1.min(centre_a + reach), b0.max(centre_b - reach), b1.min(centre_b + reach));
        if a1 <= a0 || b1 <= b0 {
            return;
        }

        let origin = cam.project(Vec3::ZERO, size);
        let px = |v: Vec3| {
            let q = cam.project(v, size);
            ((q.x - origin.x).powi(2) + (q.y - origin.y).powi(2)).sqrt()
        };
        // Distance from the target (in the plane's coordinates) at which the grid is fully faded.
        let fade_r = 0.5 * size.width.hypot(size.height) / cam.scale;
        let alpha_at = |pa: f32, pb: f32| {
            let d = ((pa - centre_a).powi(2) + (pb - centre_b).powi(2)).sqrt() / fade_r;
            1.0 - 0.85 * ((d - 0.45) / 0.65).clamp(0.0, 1.0)
        };
        for (step, color) in levels {
            // Skip a level whose lines would be closer than 6 px on screen in either direction.
            if px(a * step).min(px(b * step)) < 6.0 {
                continue;
            }
            let (i0, i1) = ((a0 / step).floor() as i64, (a1 / step).ceil() as i64);
            let (j0, j1) = ((b0 / step).floor() as i64, (b1 / step).ceil() as i64);
            if (i1 - i0) + (j1 - j0) > 400 {
                continue;
            }
            let mut builders: [canvas::path::Builder; BUCKETS] = std::array::from_fn(|_| canvas::path::Builder::new());
            let mut run = |from: (f32, f32), to: (f32, f32)| {
                let at = |t: f32| (from.0 + (to.0 - from.0) * t, from.1 + (to.1 - from.1) * t);
                for k in 0..RUNS {
                    let (p0, p1) = (at(k as f32 / RUNS as f32), at((k + 1) as f32 / RUNS as f32));
                    let mid = at((k as f32 + 0.5) / RUNS as f32);
                    let bucket = ((alpha_at(mid.0, mid.1) * BUCKETS as f32).ceil() as usize).clamp(1, BUCKETS) - 1;
                    builders[bucket].move_to(cam.project(a * p0.0 + b * p0.1, size));
                    builders[bucket].line_to(cam.project(a * p1.0 + b * p1.1, size));
                }
            };
            for i in i0..=i1 {
                run((i as f32 * step, b0), (i as f32 * step, b1));
            }
            for j in j0..=j1 {
                run((a0, j as f32 * step), (a1, j as f32 * step));
            }
            for (k, builder) in builders.into_iter().enumerate() {
                let alpha = (k + 1) as f32 / BUCKETS as f32;
                frame.stroke(&builder.build(), Stroke::default().with_color(color.scale_alpha(alpha)).with_width(1.0));
            }
        }
    }
}
