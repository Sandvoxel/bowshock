//! The Cover (design/components/Cover): what the viewport shows while the document has no parts. The brand mark drawn large on
//! the viewport ground — the 16 px grid (160 px major), the dashed axis, registration marks, a longer Von Kármán nose, the
//! detached shock with two faint outer standoff lines — labelled `VON KÁRMÁN NOSE · DETACHED SHOCK · M 2.4`, with a quiet hint
//! under it. There are no colour blocks. It sits between the sheet and the model, so a part carried in from the bin is drawn
//! over it, and it is gone as soon as the first part is in the rocket.

use iced::widget::canvas::{Frame, Geometry, LineDash, Path, Stroke, Text};
use iced::widget::{canvas, stack};
use iced::{alignment, mouse, Element, Fill, Point, Rectangle, Renderer, Size, Theme, Widget};

use crate::app::Ctx;
use crate::logo::{self, AXIS};
use crate::theme::{self, MONO, SANS, SANS_MEDIUM};
use crate::tokens::{SPACE_4, TEXT_BODY, TEXT_LABEL};

/// The label under the registration marks.
const LABEL: &str = "VON KÁRMÁN NOSE · DETACHED SHOCK · M 2.4";
/// The hint under the figure.
const HINT: &str = "Drag a nose cone from the parts bin · Ctrl E opens the example";

/// The figure's nose: tip at 6.6, 26 u long (longer than the logo's 14), 4 u radius.
const NOSE: (f32, f32, f32) = (6.6, 26.0, 4.0);
/// The shocks: `(apex, arm reach, stroke in units, opacity)`: the main one and the two faint outer standoff lines, outermost first.
const SHOCKS: [(f32, f32, f32, f32); 3] = [(0.4, 10.6, 0.16, 0.22), (1.7, 9.6, 0.2, 0.38), (3.0, 8.4, 0.95, 1.0)];
/// Where the figure spans on the grid: x from the outermost shock to the nose base, centred horizontally.
const FIGURE_X: (f32, f32) = (0.4, 32.6);
/// Letter spacing of the label (em).
const TRACKING: f32 = 0.08;
/// Advance of one glyph of IBM Plex Mono (em).
const MONO_ADVANCE: f32 = 0.6;

/// `sheet`, with the Cover over it when the document has no parts.
pub fn over<'a, M: 'a>(sheet: Element<'a, M>, ctx: &Ctx) -> Element<'a, M> {
    if !ctx.doc.rocket().parts().is_empty() {
        return sheet;
    }
    // While a part is being carried the hint (which says to drag one) is not needed.
    let cover = canvas(Cover { hint: !ctx.drag.active() }).width(Fill).height(Fill);
    stack![sheet, cover].boxed()
}

/// The Cover canvas.
struct Cover {
    /// Show the hint under the figure.
    hint: bool,
}

impl<M> canvas::Program<M> for Cover {
    type State = ();

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = theme::tokens(theme);
        let Size { width: w, height: h } = bounds.size();
        let mut frame = Frame::new(renderer, bounds.size());
        frame.fill_rectangle(Point::ORIGIN, bounds.size(), t.bg_000);

        // The figure: u pixels per grid unit, its axis a little above the middle so the hint fits under it.
        let span = FIGURE_X.1 - FIGURE_X.0;
        let u = (0.66 * w / span).min(0.42 * h / 21.2).clamp(4.0, 18.0);
        let (cx, cy) = ((w / 2.0).round(), (h * 0.46).round());
        let origin = Point::new(cx - (FIGURE_X.0 + span / 2.0) * u, cy - AXIS * u);

        // The ground: 16 px minor and 160 px major grid through the axis, the dashed centreline.
        let grid = |major: bool| {
            Path::new(|p| {
                let step = if major { 160.0 } else { 16.0 };
                let (nx, ny) = ((cx / step).ceil() as i32, (cy / step).ceil() as i32);
                for k in -nx..=((w - cx) / step).ceil() as i32 {
                    let x = cx + k as f32 * step + 0.5;
                    p.move_to(Point::new(x, 0.0));
                    p.line_to(Point::new(x, h));
                }
                for k in -ny..=((h - cy) / step).ceil() as i32 {
                    let y = cy + k as f32 * step + 0.5;
                    p.move_to(Point::new(0.0, y));
                    p.line_to(Point::new(w, y));
                }
            })
        };
        frame.stroke(&grid(false), Stroke::default().with_color(t.grid_minor).with_width(1.0));
        frame.stroke(&grid(true), Stroke::default().with_color(t.grid_major).with_width(1.0));
        let axis = Path::line(Point::new(0.0, cy + 0.5), Point::new(w, cy + 0.5));
        let dashed = Stroke {
            line_dash: LineDash { segments: &[10.0, 3.0, 2.0, 3.0], offset: 0 },
            ..Stroke::default().with_color(t.grid_major.scale_alpha(2.0)).with_width(1.0)
        };
        frame.stroke(&axis, dashed);

        // Registration marks, 20 px legs, as the sheet draws them.
        let (leg, inset) = (20.0, SPACE_4 / 2.0);
        let marks = Path::new(|p| {
            for (mx, my, sx, sy) in [(inset, inset, 1.0, 1.0), (w - inset, inset, -1.0, 1.0), (inset, h - inset, 1.0, -1.0), (w - inset, h - inset, -1.0, -1.0)]
            {
                p.move_to(Point::new(mx + sx * leg, my));
                p.line_to(Point::new(mx, my));
                p.line_to(Point::new(mx, my + sy * leg));
            }
        });
        frame.stroke(&marks, Stroke::default().with_color(t.line_200).with_width(1.0));

        // The two faint outer standoff lines, the shock, the nose.
        for (apex, arm, stroke, opacity) in SHOCKS {
            let color = t.accent.scale_alpha(opacity);
            let points = logo::shock(apex, 2.0, arm, 64);
            frame.stroke(&logo::path(&points, origin, u, false), logo::shock_stroke(color, stroke * u));
        }
        let (tip, length, radius) = NOSE;
        frame.fill(&logo::path(&logo::nose(tip, length, radius, 96), origin, u, true), t.ink);

        // The label, tracked out by hand (the canvas has no letter spacing), right-aligned inside the registration marks.
        let size = TEXT_LABEL;
        let advance = size * (MONO_ADVANCE + TRACKING);
        let end = w - inset - leg;
        let start = end - advance * LABEL.chars().count() as f32 + size * TRACKING;
        for (i, glyph) in LABEL.chars().enumerate().filter(|(_, g)| *g != ' ') {
            frame.fill_text(Text {
                content: glyph.to_string(),
                position: Point::new(start + i as f32 * advance, h - inset - 4.0),
                color: t.ink_faint,
                size: size.into(),
                font: MONO,
                align_y: alignment::Vertical::Bottom,
                ..Text::default()
            });
        }

        // The title (the kit Cover's wordmark) and under it the hint, centred under the figure.
        let below = origin.y + (AXIS + 10.6) * u;
        frame.fill_text(Text {
            content: "Bowshock".into(),
            position: Point::new(cx, below + 22.0),
            color: t.ink,
            size: 34.0.into(),
            font: SANS_MEDIUM,
            align_x: alignment::Horizontal::Center.into(),
            align_y: alignment::Vertical::Top,
            ..Text::default()
        });
        if self.hint {
            frame.fill_text(Text {
                content: HINT.into(),
                position: Point::new(cx, below + 70.0),
                color: t.ink_muted,
                size: TEXT_BODY.into(),
                font: SANS,
                align_x: alignment::Horizontal::Center.into(),
                align_y: alignment::Vertical::Top,
                ..Text::default()
            });
        }
        vec![frame.into_geometry()]
    }
}
