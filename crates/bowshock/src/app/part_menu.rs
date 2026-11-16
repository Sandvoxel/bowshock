//! The part context menu: right-click a part in the viewport or a Station Lanes bar and a dropdown opens at the cursor with the
//! commands that apply to it. Every entry is routed to the action that already exists (the Inspector's rename, duplicate,
//! delete, symmetry, detach and motor-mount messages; the viewport's focus), so the menu adds no behaviour of its own.
//!
//! The right-click selects the part first (unless it is already part of the selection). The menu is app-level state in
//! [`Ctx::part_menu`]; Escape, a click anywhere else, or any command closes it.

use bowshock_model::{InternalKind, PartId, PartRef, Rocket};
use iced::widget::{button, column, container, mouse_area, row, text, Space};
use iced::{Element, Fill, Padding, Point, Widget};

use crate::app::{self, Ctx};
use crate::theme::{self, MONO};
use crate::tokens::{SPACE_2, TEXT_LABEL};
use crate::widgets::glyph::{icon, Mark, Tone};

/// Width of the menu.
const WIDTH: f32 = 208.0;
/// Height of one entry.
const ROW: f32 = 26.0;
/// Height of a separator.
const RULE: f32 = 9.0;
/// Height of a section label.
const HEAD: f32 = 22.0;
/// Margin kept to the window edge.
const EDGE: f32 = 6.0;
/// The symmetry counts the Parts bin offers, and so the menu.
const COUNTS: [u8; 6] = [1, 2, 3, 4, 6, 8];

/// An open menu.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Open {
    /// The part it is for.
    pub part: PartId,
    /// Where it opened, in window coordinates.
    pub at: Point,
}

/// What an entry does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// Close the menu and do nothing (a click outside).
    Close,
    /// Start the Inspector's inline rename (the same as F2).
    Rename,
    /// Duplicate the part.
    Duplicate,
    /// Delete the selected parts.
    Delete,
    /// Turn the viewport square-on to the part.
    Focus,
    /// Set the symmetry count of the group.
    Count(u8),
    /// Take the clicked copy out of its group.
    Detach,
    /// Make every copy of the group a part of its own.
    SplitAll,
    /// Set whether the inner tube holds a motor.
    MotorMount(bool),
    /// Select the part this one sits on or in.
    SelectParent,
}

/// One row of the menu.
#[derive(Debug, Clone, PartialEq)]
enum Row {
    Item { label: String, key: Option<&'static str>, action: Action, check: Option<bool>, danger: bool },
    Counts(u8),
    Rule,
}

impl Row {
    fn item(label: &str, key: Option<&'static str>, action: Action) -> Row {
        Row::Item { label: label.to_string(), key, action, check: None, danger: false }
    }

    fn height(&self) -> f32 {
        match self {
            Row::Item { .. } => ROW,
            Row::Counts(_) => HEAD + ROW,
            Row::Rule => RULE,
        }
    }
}

/// The rows that apply to part `id` (with `many` parts selected only the commands that work on all of them).
fn rows(rocket: &Rocket, id: PartId, many: usize) -> Vec<Row> {
    let Some(part) = rocket.kind_of(id) else { return Vec::new() };
    let mut out = Vec::new();
    if many > 1 {
        out.push(Row::Item { label: format!("Delete {many} parts"), key: Some("Del"), action: Action::Delete, check: None, danger: true });
        return out;
    }
    out.push(Row::item("Rename", Some("F2"), Action::Rename));
    if !matches!(part, PartRef::Axial(p) if matches!(p.kind, bowshock_model::PartKind::NoseCone { .. })) {
        out.push(Row::item("Duplicate", None, Action::Duplicate));
    }
    out.push(Row::item("Focus", None, Action::Focus));
    if let PartRef::Internal(p) = &part {
        if let InternalKind::InnerTube { motor_mount, .. } = p.kind {
            out.push(Row::Rule);
            out.push(Row::Item { label: "Holds a motor".into(), key: None, action: Action::MotorMount(!motor_mount), check: Some(motor_mount), danger: false });
        }
    }
    if let PartRef::Radial(r) = &part {
        out.push(Row::Rule);
        out.push(Row::Counts(r.count));
        if r.count > 1 && !r.kind.is_pod() {
            out.push(Row::item("Detach this copy", None, Action::Detach));
            out.push(Row::item("Split all", None, Action::SplitAll));
        }
    }
    if part.parent().is_some() {
        out.push(Row::Rule);
        out.push(Row::item("Select parent", None, Action::SelectParent));
    }
    out.push(Row::Rule);
    out.push(Row::Item { label: "Delete".into(), key: Some("Del"), action: Action::Delete, check: None, danger: true });
    out
}

fn entry(row_: &Row) -> Element<'static, app::Message> {
    match row_ {
        Row::Rule => container(container(Space::new().width(Fill).height(1)).style(theme::hairline)).padding([4.0, 0.0]).boxed(),
        Row::Counts(now) => {
            let chips = COUNTS.iter().fold(row![].spacing(2), |r, n| {
                r.push(
                    button(text(n.to_string()).size(TEXT_LABEL).font(MONO).center())
                        .width(Fill)
                        .padding([3.0, 0.0])
                        .style(theme::segment(n == now))
                        .on_press(app::Message::PartMenu(Action::Count(*n))),
                )
            });
            column![container(crate::app::ui::label("SYMMETRY")).padding([4.0, 10.0]).height(HEAD), container(chips).padding([0.0, 8.0])].boxed()
        }
        Row::Item { label, key, action, check, danger } => {
            let mark: Element<'static, app::Message> = match check {
                Some(true) => icon(Mark::Check, Tone::Accent, 12.0),
                Some(false) => Space::new().width(12).height(12).boxed(),
                None => Space::new().width(0).height(0).boxed(),
            };
            let mut cells: Vec<Element<'static, app::Message>> = Vec::new();
            if check.is_some() {
                cells.push(mark);
            }
            cells.push(text(label.clone()).size(TEXT_LABEL).font(MONO).boxed());
            cells.push(Space::new().width(Fill).boxed());
            if let Some(key) = key {
                cells.push(text(*key).size(TEXT_LABEL).font(MONO).style(theme::faint).boxed());
            }
            let danger = *danger;
            button(row(cells).spacing(SPACE_2).align_y(iced::Center))
                .width(Fill)
                .height(ROW)
                .padding([0.0, 10.0])
                .style(move |t: &iced::Theme, status| {
                    let mut style = theme::ghost(t, status);
                    if danger && matches!(status, button::Status::Active) {
                        style.text_color = theme::tokens(t).err;
                    }
                    style
                })
                .on_press(app::Message::PartMenu(*action))
                .boxed()
        }
    }
}

/// The menu as a window-sized layer (a backdrop that closes it, and the card at the cursor kept inside the window), if open.
pub fn view(ctx: &Ctx) -> Option<Element<'_, app::Message>> {
    let open = ctx.part_menu?;
    let rows = rows(ctx.doc.rocket(), open.part, ctx.selected_parts().len());
    if rows.is_empty() {
        return None;
    }
    let height = rows.iter().map(Row::height).sum::<f32>() + 2.0;
    Some(
        iced::widget::responsive(move |window| {
            let card = container(column(rows.iter().map(entry)).padding([1.0, 0.0])).width(WIDTH).style(theme::overlay);
            let left = open.at.x.min(window.width - WIDTH - EDGE).max(EDGE);
            let top = open.at.y.min(window.height - height - EDGE).max(EDGE);
            let placed = container(card).padding(Padding { top, left, right: 0.0, bottom: 0.0 });
            let backdrop = mouse_area(Space::new().width(Fill).height(Fill))
                .on_press(app::Message::PartMenu(Action::Close))
                .on_right_press(app::Message::PartMenu(Action::Close));
            iced::widget::stack![backdrop, placed].boxed()
        })
        .boxed(),
    )
}

/// The messages that carry out `action` for part `id`, in the order they run.
pub fn messages(ctx: &Ctx, id: PartId, action: Action) -> Vec<app::Message> {
    use crate::build::inspector::Message as Inspector;
    match action {
        Action::Close => Vec::new(),
        Action::Rename => vec![app::Message::Rename],
        Action::Duplicate => vec![app::Message::Inspector(Inspector::Duplicate)],
        Action::Delete => vec![app::Message::Delete],
        Action::Focus => vec![app::Message::Viewport(crate::viewport::Message::Focus(id))],
        Action::Count(n) => vec![app::Message::Inspector(Inspector::Count(n))],
        Action::Detach => vec![app::Message::Inspector(Inspector::Detach)],
        Action::SplitAll => vec![app::Message::Inspector(Inspector::SplitAll)],
        Action::MotorMount(on) => vec![app::Message::Inspector(Inspector::MotorMount(on))],
        Action::SelectParent => match ctx.doc.rocket().kind_of(id).and_then(|p| p.parent()) {
            Some(parent) => vec![app::Message::Viewport(crate::viewport::Message::SelectPart(Some(parent)))],
            None => Vec::new(),
        },
    }
}
