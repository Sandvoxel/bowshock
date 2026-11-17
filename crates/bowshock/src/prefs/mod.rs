//! Preferences: one modal window (File > Preferences…, the top bar's gear, `⌘,` / `Ctrl+,`) with a list of sections on the left and
//! the section on the right. Every control applies at once and is kept in the user's settings (`app/settings.rs`); there is
//! nothing to confirm except clearing a cache.
//!
//! Sections: [`Section::Appearance`] (theme), [`Section::Units`] (the unit set), [`Section::Compute`] (where solves run, in
//! [`compute`]) and [`Section::Storage`] (where files live and what the caches hold, in [`storage`]).
//!
//! Module contract (see `ARCHITECTURE.md`): [`State`] is the window's state, [`Message`] its messages (wrapped in
//! `app::Message::Prefs`); the app intercepts every other message while it is open and Escape closes it.

mod appearance;
pub mod compute;
pub mod storage;
mod units;

use iced::widget::{button, column, container, row, scrollable, text, Space};
use iced::{Background, Element, Fill, Task, Theme, Widget};

use crate::app::settings::LocalSolver;
use crate::app::{self, ui, Ctx};
use crate::theme::{self, SANS_MEDIUM};
use crate::tokens::{SPACE_1, SPACE_2, SPACE_3, TEXT_BODY};
use crate::units::UnitSystem;
use crate::widgets::button::{button as push, Weight};
use crate::widgets::modal::modal;
use crate::widgets::segmented::segmented;

/// Width of the window's card.
const CARD_W: f32 = 780.0;
/// Height of the card's body (the section list and the section).
const BODY_H: f32 = 520.0;
/// Width of the section list.
const NAV_W: f32 = 168.0;

/// A page of the window.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Section {
    /// The theme.
    #[default]
    Appearance,
    /// The display unit set.
    Units,
    /// Where solves run.
    Compute,
    /// Files and caches.
    Storage,
}

impl Section {
    /// The sections in the order of the list.
    pub const ALL: [Section; 4] = [Section::Appearance, Section::Units, Section::Compute, Section::Storage];

    /// The name in the list and over the page.
    pub fn label(self) -> &'static str {
        match self {
            Section::Appearance => "Appearance",
            Section::Units => "Units",
            Section::Compute => "Compute",
            Section::Storage => "Storage",
        }
    }

    fn blurb(self) -> &'static str {
        match self {
            Section::Appearance => "How Bowshock looks.",
            Section::Units => "The units every length, mass, pressure and speed is shown in.",
            Section::Compute => "Where the CFD and 3D stability solves run.",
            Section::Storage => "Where Bowshock keeps its files, and the caches you can clear.",
        }
    }
}

/// The window's messages.
#[derive(Debug, Clone)]
pub enum Message {
    /// Open the window on the page it was last left on (File menu, gear, `⌘,`).
    Show,
    /// Open the window on this page.
    Open(Section),
    /// Close it (Escape, Done, a click outside).
    Close,
    /// A click inside the card.
    Nothing,
    /// A page of the list.
    Select(Section),
    /// Appearance: dark (`true`) or light.
    Theme(bool),
    /// Units: the unit set.
    Units(UnitSystem),
    /// Compute: the device local solves use.
    Solver(LocalSolver),
    /// A message of the Compute page's control.
    Compute(compute::ComputeMessage),
    /// A message of the Storage page.
    Storage(storage::Message),
}

/// The window's state.
#[derive(Default)]
pub struct State {
    open: bool,
    section: Section,
    compute: compute::ComputeState,
    storage: storage::State,
}

impl State {
    /// Closed, on Appearance.
    pub fn new() -> State {
        State::default()
    }

    /// Whether the window is up.
    pub fn is_open(&self) -> bool {
        self.open
    }

    /// The Storage page's unanswered `Clear` question (tests).
    #[cfg(test)]
    fn storage_confirm(&self) -> Option<storage::Kind> {
        self.storage.confirm()
    }

    /// The page on show (tests).
    #[cfg(test)]
    fn section(&self) -> Section {
        self.section
    }

    /// Shows `section`: leaving Storage drops an unanswered `Clear` question, entering it measures the places again.
    fn go(&mut self, section: Section, ctx: &Ctx) -> Task<app::Message> {
        if self.section == Section::Storage && section != Section::Storage {
            self.storage.leave();
        }
        self.section = section;
        if section == Section::Storage {
            return self.storage.refresh(ctx);
        }
        Task::none()
    }

    /// Handles a message.
    pub fn update(&mut self, message: Message, ctx: &mut Ctx) -> Task<app::Message> {
        self.compute.sync(ctx);
        match message {
            Message::Show => {
                self.open = true;
                return self.go(self.section, ctx);
            }
            Message::Open(section) => {
                self.open = true;
                return self.go(section, ctx);
            }
            Message::Close => {
                self.open = false;
                self.storage.leave();
            }
            Message::Nothing => {}
            Message::Select(section) => return self.go(section, ctx),
            Message::Theme(dark) => ctx.set_dark(dark),
            Message::Units(units) => ctx.set_units(units),
            Message::Solver(solver) => {
                ctx.local_solver = solver;
                ctx.save_settings();
            }
            Message::Compute(m) => return self.compute.update(m, ctx).map(|m| app::Message::Prefs(Message::Compute(m))),
            Message::Storage(m) => return self.storage.update(m, ctx),
        }
        Task::none()
    }

    /// The window over a dimmed backdrop, `None` while closed.
    pub fn view<'a>(&'a self, ctx: &'a Ctx) -> Option<Element<'a, app::Message>> {
        if !self.open {
            return None;
        }
        let mut list = column![].spacing(SPACE_1).width(NAV_W);
        for section in Section::ALL {
            let item = button(text(section.label()).size(TEXT_BODY)).width(Fill).padding([6.0, SPACE_2 + 2.0]).style(nav_item(section == self.section));
            list = list.push(item.on_press(Message::Select(section)));
        }
        let page: Element<'a, Message> = match self.section {
            Section::Appearance => appearance::view(ctx),
            Section::Units => units::view(ctx),
            Section::Compute => self.compute_page(ctx),
            Section::Storage => self.storage.view(),
        };
        let head =
            column![text(self.section.label()).font(SANS_MEDIUM).size(16), text(self.section.blurb()).size(TEXT_BODY).style(theme::muted)].spacing(SPACE_1);
        let content = scrollable(column![head, page].spacing(SPACE_3 + 6.0).width(Fill).padding(iced::Padding { right: SPACE_3, ..iced::Padding::ZERO }))
            .width(Fill)
            .height(Fill)
            .style(theme::scroll);
        let divider = container(Space::new()).width(1).height(Fill).style(theme::hairline);
        let body = row![list, divider, content].spacing(SPACE_3 + 4.0).height(BODY_H).boxed();
        let done: Element<'_, Message> = push("Done", Weight::Primary).key("Esc").on_press(Message::Close).into();
        let buttons = row![text("Changes apply at once and are kept for every rocket.").size(TEXT_BODY).style(theme::faint), Space::new().width(Fill), done]
            .align_y(iced::Center)
            .boxed();
        Some(modal("Preferences", body, buttons, CARD_W, Message::Nothing, Message::Close).map(app::Message::Prefs).boxed())
    }

    fn compute_page<'a>(&'a self, ctx: &'a Ctx) -> Element<'a, Message> {
        let device = container(segmented(&[(LocalSolver::Auto, "Auto"), (LocalSolver::Cpu, "CPU only")], ctx.local_solver, Message::Solver)).width(220);
        column![
            group("WHERE SOLVES RUN", Element::map(compute::view(&self.compute, ctx), Message::Compute).boxed(), None),
            group(
                "ON THIS COMPUTER",
                device.boxed(),
                Some("Auto uses the GPU when there is one and falls back to the CPU. CPU only is slower, but never touches the GPU.")
            ),
        ]
        .spacing(SPACE_3 + 4.0)
        .boxed()
    }
}

/// One setting: an uppercase head, the control, and a note under it.
pub(super) fn group<'a, M: 'a>(title: &'a str, control: Element<'a, M>, note: Option<&'a str>) -> Element<'a, M> {
    let mut group = column![ui::label(title), control].spacing(SPACE_2);
    if let Some(note) = note {
        group = group.push(text(note).size(12.0).style(theme::muted).boxed());
    }
    group.boxed()
}

/// A row of the section list: the accent's soft ground when current, `bg-400` on hover.
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

#[cfg(test)]
mod tests;
