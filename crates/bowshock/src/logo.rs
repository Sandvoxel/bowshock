//! The Bowshock mark: a detached bow shock (hyperbola tending to the Mach 2.4 angle) ahead of a Von Karman nose, on the
//! 22-unit grid of design/components/Logo/README.md.

use iced::widget::canvas::{self, Frame, Geometry, Path, Stroke};
use iced::{mouse, Point, Rectangle, Renderer, Theme};

use crate::theme;

pub struct Mark;

impl<Message> canvas::Program<Message> for Mark {
    type State = ();

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let mut frame = Frame::new(renderer, bounds.size());
        let u = bounds.width.min(bounds.height) / 22.0;
        let at = |x: f32, y: f32| Point::new(x * u, y * u);

        // Nose: Von Karman, 14 u long, 8 u in diameter, tip at x = 7, axis at y = 11.
        let (tip, len, radius, axis) = (7.0, 14.0_f32, 4.0_f32, 11.0);
        let haack = |x: f32| {
            let th = (1.0 - 2.0 * x / len).acos();
            radius * ((th - (2.0 * th).sin() / 2.0) / std::f32::consts::PI).sqrt()
        };
        let nose = Path::new(|p| {
            p.move_to(at(tip, axis));
            for i in 1..=24 {
                let x = len * i as f32 / 24.0;
                p.line_to(at(tip + x, axis - haack(x)));
            }
            for i in (0..24).rev() {
                let x = len * i as f32 / 24.0;
                p.line_to(at(tip + x, axis + haack(x)));
            }
            p.close();
        });
        frame.fill(&nose, t.ink);

        // Shock: x = x0 + a (sqrt(1 + (y/b)^2) - 1), arms tending to the Mach angle, 3.6 u standoff.
        let x0 = tip - 3.6;
        // Asymptote dy/dx = tan(mu), mu = asin(1/2.4).
        let tan_mu = (1.0_f32 / 2.4).asin().tan();
        let b = 2.2_f32;
        let a = b / tan_mu;
        let shock = Path::new(|p| {
            for i in 0..=32 {
                let y = -9.0 + 18.0 * i as f32 / 32.0;
                let pt = at(x0 + a * ((1.0 + (y / b).powi(2)).sqrt() - 1.0), axis + y);
                if i == 0 {
                    p.move_to(pt)
                } else {
                    p.line_to(pt)
                }
            }
        });
        frame.stroke(&shock, Stroke::default().with_color(t.accent).with_width(2.0 * u).with_line_cap(canvas::LineCap::Square));

        vec![frame.into_geometry()]
    }
}
