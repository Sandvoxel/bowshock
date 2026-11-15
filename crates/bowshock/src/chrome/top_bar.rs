//! The top bar (design/components/Navigation, BuilderScreen): logo and name, the document's file name with its dirty marker,
//! undo and redo, the Build / Simulate / Analyze switch in the centre, the unit system select and the primary Simulate button.

use std::fmt;

use iced::widget::{canvas, container, row, text, Space};
use iced::{Background, Center, Element, Fill, Theme, Widget};

use super::Message as ChromeMessage;
use crate::app::document::EXTENSION;
use crate::app::{Ctx, Message, Mode};
use crate::logo;
use crate::theme::{self, MONO, SANS_SEMIBOLD};
use crate::tokens::{CONTROL_H, SPACE_1, SPACE_2, SPACE_3, TEXT_BODY, TEXT_LABEL, TOPBAR_H};
use crate::units::UnitSystem;
use crate::widgets::button::{button, icon_button, Weight};
use crate::widgets::field;
use crate::widgets::glyph::Mark;
use crate::widgets::segmented::segmented;

/// Width of the unit system select.
const UNITS_W: f32 = 124.0;

/// A unit system as the select lists it (`mm · g · s`).
#[derive(Debug, Clone, Copy, PartialEq)]
struct SystemLabel(UnitSystem);

impl fmt::Display for SystemLabel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.0.label())
    }
}

/// Undo's shortcut as the platform writes it.
const UNDO_TIP: &str = if cfg!(target_os = "macos") { "⌘Z" } else { "Ctrl+Z" };
/// Redo's shortcut as the platform writes it.
const REDO_TIP: &str = if cfg!(target_os = "macos") { "⇧⌘Z" } else { "Ctrl+Shift+Z" };

/// The bar.
pub fn top_bar(ctx: &Ctx) -> Element<'_, Message> {
    let logo = row![canvas(logo::Mark).width(20).height(20), text("BOWSHOCK").font(SANS_SEMIBOLD).size(TEXT_BODY)].spacing(SPACE_2).align_y(Center);

    let dirty: Element<'_, Message> = if ctx.doc.is_dirty() {
        container(Space::new())
            .width(7)
            .height(7)
            .style(|t: &Theme| container::Style { background: Some(Background::Color(theme::tokens(t).accent)), ..container::Style::default() })
            .boxed()
    } else {
        Space::new().width(7).boxed()
    };
    let file = row![
        text(ctx.doc.title()).font(MONO).size(TEXT_BODY).style(theme::ink),
        text(format!(".{EXTENSION}")).font(MONO).size(TEXT_BODY).style(theme::faint),
        dirty,
    ]
    .spacing(SPACE_1 + 2.0)
    .align_y(Center);

    let undo = icon_button(Mark::Undo, "Undo").shortcut(UNDO_TIP);
    let redo = icon_button(Mark::Redo, "Redo").shortcut(REDO_TIP);
    let undo: Element<'_, Message> = if ctx.doc.can_undo() { undo.on_press(Message::Undo).into() } else { undo.into() };
    let redo: Element<'_, Message> = if ctx.doc.can_redo() { redo.on_press(Message::Redo).into() } else { redo.into() };

    let modes = segmented(&[(Mode::Build, "Build"), (Mode::Simulate, "Simulate"), (Mode::Analyze, "Analyze")], ctx.mode, Message::Mode);

    let systems: Vec<SystemLabel> = UnitSystem::ALL.iter().copied().map(SystemLabel).collect();
    let units = container(field::select(systems, Some(SystemLabel(ctx.units)), |s| Message::Units(s.0))).width(UNITS_W);
    let theme_toggle = theme_toggle(ctx.dark);
    let simulate: Element<'_, Message> = button("Simulate", Weight::Primary).icon(Mark::Play).key("F5").on_press(Message::Run).into();
    let menu: Element<'_, Message> =
        button("", Weight::Primary).icon(Mark::CaretDown).tooltip("Run options").on_press(Message::Chrome(ChromeMessage::ToggleRunMenu)).into();
    // The caret is its own cell, divided from Simulate by a hairline of the accent's shadow.
    let divider = container(Space::new())
        .width(1)
        .height(26)
        .style(|t: &Theme| container::Style { background: Some(Background::Color(theme::tokens(t).bg_000.scale_alpha(0.45))), ..container::Style::default() });
    let simulate = row![simulate, divider, menu].spacing(0);

    container(
        row![logo, rule(), file, rule(), undo, redo, Space::new().width(Fill), modes, Space::new().width(Fill), theme_toggle, units, simulate,]
            .spacing(SPACE_3)
            .padding([0.0, SPACE_3])
            .align_y(Center),
    )
    .height(TOPBAR_H)
    .center_y(TOPBAR_H)
    .style(theme::bar)
    .boxed()
}

/// A vertical hairline between regions of the bar.
fn rule<'a>() -> Element<'a, Message> {
    container(Space::new()).width(1).height(20).style(theme::hairline).boxed()
}

/// The theme toggle: a sun on the dark theme (click for light), a moon on the light one. The word it switches to ("Light" /
/// "Dark") is also in the button, fully transparent, as the tooltip text and so a script can still `click "Light"`.
fn theme_toggle<'a>(dark: bool) -> Element<'a, Message> {
    let word = if dark { "Light" } else { "Dark" };
    let glyph = canvas(ThemeGlyph { to_light: dark }).width(CONTROL_H).height(CONTROL_H);
    let hidden = container(text(word).size(8).style(|_: &Theme| text::Style { color: Some(iced::Color::TRANSPARENT), selection: None })).center(Fill);
    let body = iced::widget::stack![glyph, hidden].width(CONTROL_H).height(CONTROL_H);
    let toggle = iced::widget::button(body).padding(0).style(theme::ghost).on_press(Message::ToggleTheme);
    iced::widget::tooltip(toggle, container(text(word).size(TEXT_LABEL + 1.0)).padding([SPACE_1, SPACE_2]).style(theme::tooltip))
        .position(iced::widget::tooltip::Position::Bottom)
        .boxed()
}

/// Draws the sun (centre disc and eight rays) or the moon (a crescent), in the button's text colour.
struct ThemeGlyph {
    to_light: bool,
}

impl canvas::Program<Message> for ThemeGlyph {
    type State = ();

    fn draw(&self, _: &(), renderer: &iced::Renderer, theme: &Theme, bounds: iced::Rectangle, cursor: iced::mouse::Cursor) -> Vec<canvas::Geometry> {
        let t = theme::tokens(theme);
        let color = if cursor.is_over(bounds) { t.ink } else { t.ink_muted };
        let mut frame = canvas::Frame::new(renderer, bounds.size());
        let c = iced::Point::new(bounds.width / 2.0, bounds.height / 2.0);
        let stroke = canvas::Stroke::default().with_color(color).with_width(1.4);
        if self.to_light {
            frame.fill(&canvas::Path::circle(c, 3.2), color);
            for i in 0..8 {
                let a = i as f32 * std::f32::consts::FRAC_PI_4;
                let (s, k) = a.sin_cos();
                frame.stroke(&canvas::Path::line(iced::Point::new(c.x + k * 5.4, c.y + s * 5.4), iced::Point::new(c.x + k * 7.2, c.y + s * 7.2)), stroke);
            }
        } else {
            // A disc with a displaced disc cut out of it.
            let moon = canvas::Path::circle(c, 6.0);
            frame.fill(&moon, color);
            let hole = if cursor.is_over(bounds) { t.bg_400 } else { t.bg_100 };
            frame.fill(&canvas::Path::circle(iced::Point::new(c.x + 3.4, c.y - 2.6), 5.2), hole);
        }
        vec![frame.into_geometry()]
    }
}
