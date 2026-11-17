//! The section list of the Preferences window: a recessed column of glyph + label items with the version pinned at its foot.

use std::f32::consts::FRAC_PI_2;

use iced::widget::canvas::path::Arc;
use iced::widget::{button, canvas, column, container, row, stack, text, Space};
use iced::{Background, Center, Element, Fill, Point, Radians, Size, Theme, Widget};

use super::{Message, Section};
use crate::theme::{self, MONO};
use crate::tokens::{SPACE_1, SPACE_2, TEXT_BODY};

/// Width of the section list, its right edge line included.
const NAV_W: f32 = 168.0;
/// Edge of a glyph.
const GLYPH: f32 = 16.0;

/// The list: the sections, then (pinned to the bottom) the version.
pub(super) fn list<'a>(current: Section) -> Element<'a, Message> {
    let mut items = column![].spacing(SPACE_1).width(Fill);
    for section in Section::ALL {
        items = items.push(item(section, section == current));
    }
    let version = container(text(concat!("Bowshock ", env!("CARGO_PKG_VERSION"))).font(MONO).size(11).style(theme::faint)).padding([0.0, SPACE_2]);
    let panel = container(column![items, Space::new().height(Fill), version].height(Fill)).width(NAV_W - 1.0).height(Fill).padding(SPACE_2).style(recessed);
    let edge = container(Space::new()).width(1).height(Fill).style(theme::hairline);
    row![panel, edge].height(Fill).boxed()
}

/// One item: the button, with the current item's 2 px accent bar laid over its left edge.
fn item<'a>(section: Section, current: bool) -> Element<'a, Message> {
    let body =
        row![canvas(Glyph { section, current }).width(GLYPH).height(GLYPH), text(section.label()).size(TEXT_BODY)].spacing(SPACE_2 + 2.0).align_y(Center);
    let item = button(body).width(Fill).padding([6.0, SPACE_2 + 2.0]).style(nav_item(current)).on_press(Message::Select(section));
    let bar = container(Space::new().width(2).height(Fill)).height(Fill).style(move |theme: &Theme| container::Style {
        background: current.then(|| Background::Color(theme::tokens(theme).accent)),
        ..container::Style::default()
    });
    stack![item, bar].width(Fill).boxed()
}

/// The column's ground: one step darker than the modal card.
fn recessed(theme: &Theme) -> container::Style {
    container::Style { background: Some(Background::Color(theme::tokens(theme).bg_100)), ..container::Style::default() }
}

/// A row of the list: the accent's soft ground when current, `bg-400` on hover.
fn nav_item(current: bool) -> impl Fn(&Theme, button::Status) -> button::Style {
    move |theme, status| {
        let t = theme::tokens(theme);
        let hovered = matches!(status, button::Status::Hovered | button::Status::Pressed);
        let background = if current {
            Some(Background::Color(t.accent_soft))
        } else if hovered {
            Some(Background::Color(t.bg_400))
        } else {
            None
        };
        button::Style { background, text_color: if current { t.ink } else { t.ink_muted }, border: iced::Border::default(), ..button::Style::default() }
    }
}

/// A section's glyph, stroked 1.4 px in the item's text colour.
struct Glyph {
    section: Section,
    current: bool,
}

impl canvas::Program<Message> for Glyph {
    type State = ();

    fn draw(&self, _: &(), renderer: &iced::Renderer, theme: &Theme, bounds: iced::Rectangle, _: iced::mouse::Cursor) -> Vec<canvas::Geometry> {
        let t = theme::tokens(theme);
        let color = if self.current { t.ink } else { t.ink_muted };
        let stroke = canvas::Stroke::default().with_color(color).with_width(1.4);
        let mut frame = canvas::Frame::new(renderer, bounds.size());
        let p = Point::new;
        let line = |frame: &mut canvas::Frame, a: Point, b: Point| frame.stroke(&canvas::Path::line(a, b), stroke);
        match self.section {
            Section::Appearance => {
                let c = p(8.0, 8.0);
                let half = canvas::Path::new(|b| {
                    b.move_to(p(8.0, 2.0));
                    b.arc(Arc { center: c, radius: 6.0, start_angle: Radians(-FRAC_PI_2), end_angle: Radians(FRAC_PI_2) });
                    b.close();
                });
                frame.fill(&half, color);
                frame.stroke(&canvas::Path::circle(c, 6.0), stroke);
            }
            Section::Editor => {
                // A tube with a ring at each end: a part held by its ends.
                frame.stroke(&canvas::Path::rectangle(p(4.0, 5.0), Size::new(8.0, 6.0)), stroke);
                frame.stroke(&canvas::Path::circle(p(2.0, 8.0), 1.5), stroke);
                frame.stroke(&canvas::Path::circle(p(14.0, 8.0), 1.5), stroke);
            }
            Section::Units => {
                frame.stroke(&canvas::Path::rectangle(p(1.0, 5.0), Size::new(14.0, 6.0)), stroke);
                for i in 1..=4 {
                    let x = 1.0 + 14.0 * i as f32 / 5.0;
                    line(&mut frame, p(x, 11.0), p(x, 8.0));
                }
            }
            Section::Compute => {
                frame.stroke(&canvas::Path::rectangle(p(4.0, 4.0), Size::new(8.0, 8.0)), stroke);
                for v in [6.0, 10.0] {
                    line(&mut frame, p(v, 2.0), p(v, 4.0));
                    line(&mut frame, p(v, 12.0), p(v, 14.0));
                    line(&mut frame, p(2.0, v), p(4.0, v));
                    line(&mut frame, p(12.0, v), p(14.0, v));
                }
            }
            Section::Storage => {
                frame.stroke(&canvas::Path::rounded_rectangle(p(1.0, 3.5), Size::new(14.0, 9.0), 1.5.into()), stroke);
                line(&mut frame, p(1.0, 9.5), p(15.0, 9.5));
                frame.fill(&canvas::Path::circle(p(12.5, 11.0), 0.9), color);
            }
        }
        vec![frame.into_geometry()]
    }
}
