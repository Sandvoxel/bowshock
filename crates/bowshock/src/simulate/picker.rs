//! The source picker: a searchable menu of the flight's data sources, grouped (Position, Velocity, Acceleration, Aerodynamics,
//! Forces, Mass & balance, Atmosphere, Recovery), opened from a panel's title (choose its primary source) or its `+ series` chip
//! (overlay another source). It hangs from the chip that opened it, over the plots, on a transparent layer that closes it when
//! clicked.
//!
//! One plot has one y-axis, so while adding a series the sources that measure something else stay in the list but are disabled,
//! and hovering one says why. The sources already on the plot carry an `ON` mark; clicking one removes it.

use bowshock_model::{PlotLayout, MAX_PANEL_SOURCES};
use iced::widget::{button, column, container, mouse_area, responsive, row, scrollable, stack, text, text_input, tooltip, Space};
use iced::{Border, Element, Fill, Padding, Point, Theme, Widget};

use super::channels::{self, Channel, Group};
use super::layout::panel_dim;
use super::{LayoutEdit, Message};
use crate::theme::{self, MONO};
use crate::tokens::{SPACE_1, SPACE_2, TEXT_BODY, TEXT_LABEL};

/// The widget id of the search field.
pub(super) const SEARCH_ID: &str = "plot-source-search";
/// Width of the menu.
const WIDTH: f32 = 304.0;
/// Height of a source row.
const ROW_H: f32 = 26.0;
/// Height of a group heading.
const HEAD_H: f32 = 24.0;
/// The most the list grows to before it scrolls.
const LIST_MAX: f32 = 312.0;
/// Header, search field and padding around the list.
const CHROME_H: f32 = 118.0;
/// Width of the accent edge on the current row.
const EDGE_W: f32 = 2.0;

/// What choosing a source does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// The title was clicked: the choice becomes the panel's primary source.
    Primary,
    /// `+ series`: the choice joins the panel, or leaves it when it is already there.
    Add,
}

/// The menu while it is open.
#[derive(Debug, Clone)]
pub struct Picker {
    /// The panel it edits.
    pub panel: usize,
    /// What a choice does.
    pub mode: Mode,
    /// Where it hangs: the bottom left of the chip that opened it, in the plots' canvas.
    pub at: Point,
    /// The search text.
    pub query: String,
}

/// One source row of the menu and what it does.
enum Row {
    /// Click to choose; `current` marks the primary source (or one already on the plot, with `on`).
    Open { channel: &'static Channel, current: bool, on: bool, message: Message },
    /// Cannot be chosen, with the reason.
    Disabled { channel: &'static Channel, on: bool, reason: String },
}

impl Picker {
    /// A closed-form description of every row for `layout`: the groups that have a match, each with its rows.
    fn rows(&self, layout: &PlotLayout, imperial: bool) -> Vec<(Group, Vec<Row>)> {
        let Some(panel) = layout.panels.get(self.panel) else { return Vec::new() };
        let dim = panel_dim(panel);
        let mut out = Vec::new();
        for group in Group::ALL {
            let rows: Vec<Row> = channels::in_group(group)
                .filter(|c| c.matches(&self.query))
                .map(|c| {
                    let on = panel.sources.iter().any(|k| k == c.key);
                    match self.mode {
                        Mode::Primary => Row::Open {
                            channel: c,
                            current: panel.sources.first().is_some_and(|k| k == c.key),
                            on,
                            message: Message::Layout(LayoutEdit::SetPrimary(self.panel, c.key)),
                        },
                        Mode::Add if on && panel.sources.len() == 1 => Row::Disabled { channel: c, on, reason: "A plot needs at least one source".to_string() },
                        Mode::Add if on => Row::Open { channel: c, current: true, on, message: Message::Layout(LayoutEdit::ToggleSource(self.panel, c.key)) },
                        Mode::Add => match dim.and_then(|d| c.refusal(d, imperial)) {
                            Some(reason) => Row::Disabled { channel: c, on, reason },
                            None if panel.sources.len() >= MAX_PANEL_SOURCES => {
                                Row::Disabled { channel: c, on, reason: format!("A plot holds at most {MAX_PANEL_SOURCES} sources") }
                            }
                            None => Row::Open { channel: c, current: false, on, message: Message::Layout(LayoutEdit::ToggleSource(self.panel, c.key)) },
                        },
                    }
                })
                .collect();
            if !rows.is_empty() {
                out.push((group, rows));
            }
        }
        out
    }

    /// The menu as an overlay of the plots: a transparent layer that closes it, and the menu hanging at [`Picker::at`], slid back
    /// inside the plots where it would overhang an edge.
    pub fn view<'a>(&'a self, layout: &'a PlotLayout, imperial: bool) -> Element<'a, Message> {
        responsive(move |size| {
            let rows = self.rows(layout, imperial);
            let groups = rows.len() as f32;
            let count: usize = rows.iter().map(|(_, r)| r.len()).sum();
            let list_h = (count as f32 * ROW_H + groups * HEAD_H).clamp(ROW_H, LIST_MAX);
            let card = self.card(rows, layout, list_h, imperial);
            let x = self.at.x.min(size.width - WIDTH - 8.0).max(8.0);
            let y = self.at.y.min(size.height - (CHROME_H + list_h) - 8.0).max(8.0);
            let placed = container(card).padding(Padding { top: y, left: x, right: 0.0, bottom: 0.0 });
            let backdrop = mouse_area(Space::new().width(Fill).height(Fill)).on_press(Message::ClosePicker);
            stack![backdrop, placed].boxed()
        })
        .boxed()
    }

    fn card<'a>(&'a self, rows: Vec<(Group, Vec<Row>)>, layout: &PlotLayout, list_h: f32, imperial: bool) -> Element<'a, Message> {
        let title = match self.mode {
            Mode::Primary => format!("PLOT {} · SOURCE", self.panel + 1),
            Mode::Add => format!("PLOT {} · ADD SERIES", self.panel + 1),
        };
        let hint = match self.mode {
            Mode::Primary => "choose".to_string(),
            Mode::Add => layout.panels.get(self.panel).map_or(String::new(), |p| format!("{}/{MAX_PANEL_SOURCES}", p.sources.len())),
        };
        let head = row![
            text(title).font(MONO).size(TEXT_LABEL).style(theme::faint),
            Space::new().width(Fill),
            text(hint).font(MONO).size(TEXT_LABEL).style(theme::faint)
        ]
        .align_y(iced::Center);
        let search =
            text_input("Search sources", &self.query).id(SEARCH_ID).on_input(Message::PickerQuery).size(TEXT_BODY).padding([4.0, SPACE_2]).style(theme::input);

        let mut list: Vec<Element<'a, Message>> = Vec::new();
        for (group, group_rows) in rows {
            list.push(
                container(text(group.title().to_uppercase()).font(MONO).size(TEXT_LABEL).style(theme::faint))
                    .padding(Padding { top: 8.0, left: 12.0, right: 12.0, bottom: 2.0 })
                    .height(HEAD_H)
                    .boxed(),
            );
            for r in group_rows {
                list.push(source_row(r, imperial));
            }
        }
        if list.is_empty() {
            list.push(container(text("No source matches").size(TEXT_LABEL).style(theme::faint)).padding([SPACE_2, 12.0]).boxed());
        }
        let list = scrollable(column(list).width(Fill)).height(list_h).direction(super::thin_scrollbar()).style(super::scroll_style);
        let rule = || container(Space::new()).width(Fill).height(1).style(theme::hairline).boxed();
        let mut body: Vec<Element<'a, Message>> = vec![
            container(head).padding(Padding { top: 8.0, left: 12.0, right: 12.0, bottom: SPACE_1 }).boxed(),
            container(search).padding(Padding { top: 0.0, left: 8.0, right: 8.0, bottom: 6.0 }).boxed(),
            rule(),
            list.boxed(),
        ];
        // The rule behind the disabled rows, stated once where it can be read without hovering.
        if let (Mode::Add, Some(dim)) = (self.mode, layout.panels.get(self.panel).and_then(panel_dim)) {
            let unit = dim.unit(imperial).symbol;
            let what = if unit.is_empty() { dim.noun().to_string() } else { format!("{} ({unit})", dim.noun()) };
            body.push(rule());
            body.push(
                container(text(format!("One y-axis per plot: only {what} sources can join.")).size(TEXT_LABEL).style(theme::faint))
                    .width(Fill)
                    .padding([SPACE_2, 12.0])
                    .boxed(),
            );
        }
        container(mouse_area(column(body)).on_press(Message::KeepPicker)).width(WIDTH).style(theme::overlay).boxed()
    }
}

/// The unit text at the right of a row (`m/s`; nothing for a dimensionless source).
fn unit_text(channel: &Channel, imperial: bool) -> String {
    let symbol = channel.unit(imperial).symbol;
    if symbol.is_empty() {
        "—".to_string()
    } else {
        symbol.to_string()
    }
}

fn source_row<'a>(row_data: Row, imperial: bool) -> Element<'a, Message> {
    match row_data {
        Row::Open { channel, current, on, message } => {
            let edge = container(Space::new()).width(EDGE_W).height(ROW_H).style(move |theme: &Theme| container::Style {
                background: current.then(|| theme::tokens(theme).accent.into()),
                ..container::Style::default()
            });
            let right = if on { "ON".to_string() } else { unit_text(channel, imperial) };
            let label = row![
                text(channel.label).size(TEXT_BODY).style(theme::ink),
                Space::new().width(Fill),
                text(right).font(MONO).size(TEXT_LABEL).style(if on { theme::ink } else { theme::faint }),
            ]
            .align_y(iced::Center);
            let press = button(label).width(Fill).height(ROW_H).padding([0.0, 10.0]).style(row_style(current)).on_press(message);
            let line = row![edge, press].boxed();
            hinted(line, channel.hint)
        }
        Row::Disabled { channel, on, reason } => {
            let right = if on { "ON".to_string() } else { unit_text(channel, imperial) };
            let label = row![
                text(channel.label).size(TEXT_BODY).style(theme::faint),
                Space::new().width(Fill),
                text(right).font(MONO).size(TEXT_LABEL).style(theme::faint),
            ]
            .align_y(iced::Center);
            let cell = container(label).width(Fill).height(ROW_H).padding(Padding { top: 0.0, left: 10.0 + EDGE_W, right: 10.0, bottom: 0.0 }).center_y(ROW_H);
            let bubble = container(text(reason).size(TEXT_LABEL).style(theme::ink)).width(260).padding([SPACE_1, SPACE_2]).style(|t: &Theme| theme::tooltip(t));
            tooltip(cell, bubble).position(tooltip::Position::Right).gap(SPACE_1).boxed()
        }
    }
}

/// `row` with a tooltip saying what the source is, when it has a hint.
fn hinted<'a>(row: Element<'a, Message>, hint: &'static str) -> Element<'a, Message> {
    if hint.is_empty() {
        return row;
    }
    let bubble = container(text(hint).size(TEXT_LABEL).style(theme::ink)).width(260).padding([SPACE_1, SPACE_2]).style(|t: &Theme| theme::tooltip(t));
    tooltip(row, bubble).position(tooltip::Position::Right).gap(SPACE_1).boxed()
}

/// A source row: transparent, `bg-400` under the cursor, `bg-300` when it is the current one.
fn row_style(current: bool) -> impl Fn(&Theme, button::Status) -> button::Style {
    move |theme, status| {
        let t = theme::tokens(theme);
        let hot = matches!(status, button::Status::Hovered | button::Status::Pressed);
        let background = if hot {
            Some(t.bg_400)
        } else if current {
            Some(t.bg_300)
        } else {
            None
        };
        button::Style { background: background.map(Into::into), text_color: t.ink, border: Border::default(), ..button::Style::default() }
    }
}
