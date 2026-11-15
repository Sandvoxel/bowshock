//! Drawn glyphs: the CG and CP markers, the chip state icons and the few toolbar marks. They are canvas drawings
//! (design/README.md §6: "CG/CP glyphs are drawn, not icon-fonted"), tinted by a [`Tone`] so they follow the theme.
//!
//! ```ignore
//! use crate::widgets::glyph::{self, Tone};
//! let cg = glyph::cg(12.0); // blue quartered circle
//! let play = glyph::icon(glyph::Mark::Play, Tone::AccentInk, 12.0);
//! ```

use std::f32::consts::{FRAC_PI_2, PI};

use iced::widget::canvas::{self, path::Arc, Frame, Geometry, LineCap, Path, Stroke};
use iced::{mouse, Color, Element, Point, Radians, Rectangle, Renderer, Theme, Widget};

use crate::theme::tokens;
use crate::tokens::Tokens;

/// Which token colour a glyph is drawn in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tone {
    /// `ink`.
    Ink,
    /// `ink-muted`.
    Muted,
    /// `ink-faint`.
    Faint,
    /// `accent`.
    Accent,
    /// `accent-ink`, for glyphs on an accent fill.
    AccentInk,
    /// `ok`.
    Ok,
    /// `warn`.
    Warn,
    /// `err`.
    Err,
    /// `cg`, always blue.
    Cg,
    /// `cp`, always red.
    Cp,
}

impl Tone {
    /// The colour of this tone in `t`.
    pub fn color(self, t: &Tokens) -> Color {
        match self {
            Tone::Ink => t.ink,
            Tone::Muted => t.ink_muted,
            Tone::Faint => t.ink_faint,
            Tone::Accent => t.accent,
            Tone::AccentInk => t.accent_ink,
            Tone::Ok => t.ok,
            Tone::Warn => t.warn,
            Tone::Err => t.err,
            Tone::Cg => t.cg,
            Tone::Cp => t.cp,
        }
    }
}

/// The drawn marks. Stroked at 1.5 px on a 14 px grid with square caps, like the kit's icons.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mark {
    /// The CG marker: a ring with two opposite filled quadrants.
    Cg,
    /// The CP marker: a ring with a centred dot.
    Cp,
    /// Check inside a circle (chip `ok`).
    Check,
    /// Triangle with an exclamation mark (chip `warn`).
    Warning,
    /// Cross inside a circle (chip `err`).
    Error,
    /// A filled right-pointing triangle (Simulate).
    Play,
    /// Undo arrow.
    Undo,
    /// Redo arrow.
    Redo,
    /// A plus.
    Plus,
    /// Three dots (overflow menu).
    Ellipsis,
    /// A small filled up-pointing triangle (menu opens upward).
    CaretUp,
    /// A small filled down-pointing triangle.
    CaretDown,
    /// A cross (close).
    Close,
}

struct Drawing {
    mark: Mark,
    tone: Tone,
}

/// A `size` × `size` glyph of `mark` in `tone`.
pub fn icon<'a, M: 'a>(mark: Mark, tone: Tone, size: f32) -> Element<'a, M> {
    canvas::Canvas::new(Drawing { mark, tone }).width(size).height(size).boxed()
}

/// The CG marker (blue quartered circle) at `size` px.
pub fn cg<'a, M: 'a>(size: f32) -> Element<'a, M> {
    icon(Mark::Cg, Tone::Cg, size)
}

/// The CP marker (red dot-in-ring) at `size` px.
pub fn cp<'a, M: 'a>(size: f32) -> Element<'a, M> {
    icon(Mark::Cp, Tone::Cp, size)
}

impl<M> canvas::Program<M> for Drawing {
    type State = ();

    fn draw(&self, _: &(), renderer: &Renderer, theme: &Theme, bounds: Rectangle, _: mouse::Cursor) -> Vec<Geometry> {
        let t = tokens(theme);
        let color = self.tone.color(t);
        let mut frame = Frame::new(renderer, bounds.size());
        // Everything is drawn on a 14-unit grid scaled to the canvas.
        let u = bounds.width.min(bounds.height) / 14.0;
        let off = Point::new((bounds.width - 14.0 * u) / 2.0, (bounds.height - 14.0 * u) / 2.0);
        let at = |x: f32, y: f32| Point::new(off.x + x * u, off.y + y * u);
        let stroke = Stroke::default().with_color(color).with_width((1.5 * u).max(1.0)).with_line_cap(LineCap::Square);
        let thin = Stroke::default().with_color(color).with_width((1.25 * u).max(1.0));

        match self.mark {
            Mark::Cg => {
                let (c, r) = (at(7.0, 7.0), 5.5 * u);
                frame.stroke(&Path::circle(c, r), thin);
                for start in [-FRAC_PI_2, FRAC_PI_2] {
                    let quadrant = Path::new(|b| {
                        b.move_to(c);
                        for i in 0..=12 {
                            let a = start + FRAC_PI_2 * i as f32 / 12.0;
                            b.line_to(Point::new(c.x + r * a.cos(), c.y + r * a.sin()));
                        }
                        b.close();
                    });
                    frame.fill(&quadrant, color);
                }
            }
            Mark::Cp => {
                frame.stroke(&Path::circle(at(7.0, 7.0), 5.5 * u), thin);
                frame.fill(&Path::circle(at(7.0, 7.0), 2.0 * u), color);
            }
            Mark::Check => {
                frame.stroke(&Path::circle(at(7.0, 7.0), 5.5 * u), thin);
                let tick = Path::new(|b| {
                    b.move_to(at(4.4, 7.2));
                    b.line_to(at(6.3, 9.0));
                    b.line_to(at(9.6, 5.2));
                });
                frame.stroke(&tick, thin);
            }
            Mark::Error => {
                frame.stroke(&Path::circle(at(7.0, 7.0), 5.5 * u), thin);
                frame.stroke(&Path::line(at(4.9, 4.9), at(9.1, 9.1)), thin);
                frame.stroke(&Path::line(at(9.1, 4.9), at(4.9, 9.1)), thin);
            }
            Mark::Warning => {
                let tri = Path::new(|b| {
                    b.move_to(at(7.0, 1.8));
                    b.line_to(at(13.0, 12.2));
                    b.line_to(at(1.0, 12.2));
                    b.close();
                });
                frame.stroke(&tri, thin);
                frame.stroke(&Path::line(at(7.0, 5.4), at(7.0, 8.6)), thin);
                frame.fill(&Path::circle(at(7.0, 10.4), 0.8 * u), color);
            }
            Mark::Play => {
                let tri = Path::new(|b| {
                    b.move_to(at(3.5, 2.5));
                    b.line_to(at(11.5, 7.0));
                    b.line_to(at(3.5, 11.5));
                    b.close();
                });
                frame.fill(&tri, color);
            }
            Mark::Undo | Mark::Redo => {
                // The kit's 16-unit SVG (`M5 4 L2 7 L5 10`, `M2 7 H10 a4 4 0 0 1 0 8 H7`), mirrored for redo.
                let undo = self.mark == Mark::Undo;
                let k = 14.0 / 16.0;
                let p = |x: f32, y: f32| at(k * if undo { x } else { 16.0 - x }, k * y);
                let head = Path::new(|b| {
                    b.move_to(p(5.0, 4.0));
                    b.line_to(p(2.0, 7.0));
                    b.line_to(p(5.0, 10.0));
                });
                frame.stroke(&head, thin);
                let body = Path::new(|b| {
                    b.move_to(p(2.0, 7.0));
                    b.line_to(p(10.0, 7.0));
                    b.arc(Arc {
                        center: p(10.0, 11.0),
                        radius: 4.0 * k * u,
                        start_angle: Radians(-FRAC_PI_2),
                        end_angle: Radians(if undo { FRAC_PI_2 } else { -PI - FRAC_PI_2 }),
                    });
                    b.line_to(p(7.0, 15.0));
                });
                frame.stroke(&body, thin);
            }
            Mark::Plus => {
                frame.stroke(&Path::line(at(7.0, 2.5), at(7.0, 11.5)), stroke);
                frame.stroke(&Path::line(at(2.5, 7.0), at(11.5, 7.0)), stroke);
            }
            Mark::Ellipsis => {
                for x in [3.0, 7.0, 11.0] {
                    frame.fill(&Path::circle(at(x, 7.0), 1.0 * u), color);
                }
            }
            Mark::CaretUp => {
                let tri = Path::new(|b| {
                    b.move_to(at(7.0, 4.5));
                    b.line_to(at(10.5, 9.5));
                    b.line_to(at(3.5, 9.5));
                    b.close();
                });
                frame.fill(&tri, color);
            }
            Mark::CaretDown => {
                let tri = Path::new(|b| {
                    b.move_to(at(7.0, 9.5));
                    b.line_to(at(10.5, 4.5));
                    b.line_to(at(3.5, 4.5));
                    b.close();
                });
                frame.fill(&tri, color);
            }
            Mark::Close => {
                frame.stroke(&Path::line(at(3.5, 3.5), at(10.5, 10.5)), stroke);
                frame.stroke(&Path::line(at(10.5, 3.5), at(3.5, 10.5)), stroke);
            }
        }
        vec![frame.into_geometry()]
    }
}
