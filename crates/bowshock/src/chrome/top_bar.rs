//! The top bar (design/components/Navigation, BuilderScreen): logo and name, the document's file name with its dirty marker,
//! undo and redo, the Build / Simulate / Analyze switch in the centre, the unit system select and the primary Simulate button.

use std::fmt;

use iced::widget::{canvas, container, row, text, Space};
use iced::{Background, Center, Element, Fill, Theme, Widget};

use crate::app::document::EXTENSION;
use crate::app::{Ctx, Message, Mode};
use crate::logo;
use crate::theme::{self, MONO, SANS_SEMIBOLD};
use crate::tokens::{SPACE_1, SPACE_2, SPACE_3, TEXT_BODY, TOPBAR_H};
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
    let theme_toggle: Element<'_, Message> = button(if ctx.dark { "Light" } else { "Dark" }, Weight::Ghost).on_press(Message::ToggleTheme).into();
    let simulate: Element<'_, Message> = button("Simulate", Weight::Primary).icon(Mark::Play).key("F5").on_press(Message::Run).into();

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
