//! Preferences > Appearance: the theme, set here and nowhere else. Each theme is a tile holding a miniature of the app window in
//! that theme's own palette.

use iced::widget::canvas::path::Builder;
use iced::widget::{button, canvas, column, container, row, text};
use iced::{Background, Border, Center, Element, Fill, Point, Radians, Size, Theme, Widget};

use super::{group, Message};
use crate::app::Ctx;
use crate::theme;
use crate::tokens::{Tokens, DARK, LIGHT, SPACE_2, SPACE_3, TEXT_BODY};

/// Width and height of a tile.
const TILE: (f32, f32) = (220.0, 150.0);
/// Height of the miniature in a tile.
const MINI_H: f32 = 110.0;
/// Padding of a tile, so its 2 px border never meets the miniature.
const TILE_PAD: f32 = 2.0;

/// The page body.
pub fn view(ctx: &Ctx) -> Element<'_, Message> {
    let tiles = row![tile(true, ctx.dark), tile(false, ctx.dark)].spacing(SPACE_3);
    group("THEME", tiles.boxed(), Some("Kept for every rocket and every run."))
}

/// The tile of one theme: its miniature and a radio mark with its name.
fn tile<'a>(dark: bool, current_dark: bool) -> Element<'a, Message> {
    let selected = dark == current_dark;
    let palette = if dark { DARK } else { LIGHT };
    let name = if dark { "Dark" } else { "Light" };
    let mini = canvas(Mini(palette)).width(Fill).height(MINI_H);
    let caption = container(row![canvas(Radio(selected)).width(12).height(12), text(name).size(TEXT_BODY).style(theme::ink)].spacing(SPACE_2).align_y(Center))
        .height(Fill)
        .padding([0.0, SPACE_3])
        .align_y(Center);
    button(column![mini, caption]).width(TILE.0).height(TILE.1).padding(TILE_PAD).style(tile_style(selected)).on_press(Message::Theme(dark)).boxed()
}

/// A tile: `bg-300`, a 1 px `line-100` border; `ink-muted` on hover; 2 px `accent` when it is the current theme.
fn tile_style(selected: bool) -> impl Fn(&Theme, button::Status) -> button::Style {
    move |theme, status| {
        let t = theme::tokens(theme);
        let hovered = matches!(status, button::Status::Hovered | button::Status::Pressed);
        let border = if selected {
            Border { color: t.accent, width: 2.0, radius: 0.0.into() }
        } else if hovered {
            theme::frame(t.ink_muted)
        } else {
            theme::frame(t.line_100)
        };
        button::Style { background: Some(Background::Color(t.bg_300)), text_color: t.ink, border, ..button::Style::default() }
    }
}

/// A 12 px ring, with an accent dot in it when selected.
struct Radio(bool);

impl canvas::Program<Message> for Radio {
    type State = ();

    fn draw(&self, _: &(), renderer: &iced::Renderer, theme: &Theme, bounds: iced::Rectangle, _: iced::mouse::Cursor) -> Vec<canvas::Geometry> {
        let t = theme::tokens(theme);
        let mut frame = canvas::Frame::new(renderer, bounds.size());
        let c = Point::new(bounds.width / 2.0, bounds.height / 2.0);
        let edge = if self.0 { t.accent } else { t.ink_muted };
        frame.stroke(&canvas::Path::circle(c, 5.4), canvas::Stroke::default().with_color(edge).with_width(1.2));
        if self.0 {
            frame.fill(&canvas::Path::circle(c, 2.8), t.accent);
        }
        vec![frame.into_geometry()]
    }
}

/// A miniature of the app window in one theme's palette (not the active theme).
struct Mini(Tokens);

impl canvas::Program<Message> for Mini {
    type State = ();

    fn draw(&self, _: &(), renderer: &iced::Renderer, _: &Theme, bounds: iced::Rectangle, _: iced::mouse::Cursor) -> Vec<canvas::Geometry> {
        const BAR: f32 = 12.0;
        const STATUS: f32 = 10.0;
        const SIDE: f32 = 44.0;
        let p = &self.0;
        let (w, h) = (bounds.width, bounds.height);
        let mut frame = canvas::Frame::new(renderer, bounds.size());
        let rect = |frame: &mut canvas::Frame, x: f32, y: f32, rw: f32, rh: f32, color: iced::Color| {
            frame.fill_rectangle(Point::new(x, y), Size::new(rw, rh), color);
        };
        let mid_h = h - BAR - STATUS;

        // The viewport ground, then the bar, the two side columns and the status strip over it.
        rect(&mut frame, 0.0, 0.0, w, h, p.bg_000);
        rect(&mut frame, 0.0, 0.0, w, BAR, p.bg_100);
        rect(&mut frame, 0.0, BAR, SIDE, mid_h, p.bg_200);
        rect(&mut frame, w - SIDE, BAR, SIDE, mid_h, p.bg_200);
        rect(&mut frame, 0.0, h - STATUS, w, STATUS, p.bg_100);

        // The menus, and the Simulate button.
        for i in 0..3 {
            rect(&mut frame, 6.0 + 10.0 * i as f32, 5.0, 6.0, 2.0, p.ink_muted);
        }
        rect(&mut frame, w - 28.0, 3.0, 22.0, 6.0, p.accent);

        // The parts bin and the inspector.
        for (i, len) in [28.0, 20.0, 26.0, 16.0].into_iter().enumerate() {
            rect(&mut frame, 6.0, BAR + 8.0 + 10.0 * i as f32, len, 3.0, p.ink_muted.scale_alpha(0.55));
        }
        for (i, len) in [30.0, 22.0, 28.0].into_iter().enumerate() {
            rect(&mut frame, w - SIDE + 6.0, BAR + 8.0 + 10.0 * i as f32, len, 3.0, p.ink_muted.scale_alpha(0.55));
        }

        // The hairlines between regions.
        rect(&mut frame, 0.0, BAR - 1.0, w, 1.0, p.line_100);
        rect(&mut frame, 0.0, h - STATUS, w, 1.0, p.line_100);
        rect(&mut frame, SIDE, BAR, 1.0, mid_h, p.line_100);
        rect(&mut frame, w - SIDE - 1.0, BAR, 1.0, mid_h, p.line_100);

        // The rocket, nose to the left, tilted 12°, with its CG and CP on the axis.
        frame.with_save(|f| {
            f.translate(iced::Vector::new(w / 2.0, BAR + mid_h / 2.0));
            f.rotate(Radians(-12f32.to_radians()));
            let mut b = Builder::new();
            b.move_to(Point::new(-42.0, 0.0));
            b.bezier_curve_to(Point::new(-36.0, -2.0), Point::new(-30.0, -9.0), Point::new(-18.0, -9.0));
            for (x, y) in [(22.0, -9.0), (36.0, -22.0), (42.0, -22.0), (42.0, 22.0), (36.0, 22.0), (22.0, 9.0), (-18.0, 9.0)] {
                b.line_to(Point::new(x, y));
            }
            b.bezier_curve_to(Point::new(-30.0, 9.0), Point::new(-36.0, 2.0), Point::new(-42.0, 0.0));
            b.close();
            f.fill(&b.build(), p.part_body);
            for (x, color) in [(2.0, p.cg), (26.0, p.cp)] {
                let dot = canvas::Path::circle(Point::new(x, 0.0), 3.0);
                f.fill(&dot, color);
                f.stroke(&dot, canvas::Stroke::default().with_color(p.bg_000).with_width(1.0));
            }
        });
        vec![frame.into_geometry()]
    }
}
