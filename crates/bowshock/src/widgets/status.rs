//! StatusStrip (design/components/StatusStrip, ConfigPicker's strip cell): the always-on vehicle summary in ruled cells.
//! Each cell is a mono uppercase `ink-faint` key plus a mono `ink` value; CG and CP carry their glyphs and colours; the
//! stability cell has the compact gauge; the config cell opens the configuration menu.
//!
//! ```ignore
//! status::strip(
//!     vec![
//!         status::config("H128W", self.config_open, Some(Message::ToggleConfigMenu)),
//!         status::readout("MASS", "1.284 kg", Some(Message::OpenMass)),
//!         status::cg("712.4", true, Some(Message::ToggleCg)),
//!         status::cp("858.7", true, Some(Message::ToggleCp)),
//!         status::stability(&self.gauge, &Bands::default(), Some(Message::OpenAnalyze)),
//!     ],
//!     vec![chip(Severity::Warn, "1 warning").into()],
//! )
//! ```

use iced::widget::{button, container, row, text, Space};
use iced::{Element, Fill, Widget};

use super::gauge::{self, Bands, GaugeState};
use super::glyph::{self, Mark, Tone};
use crate::theme::{self, MONO};
use crate::tokens::{text as type_scale, SPACE_1, SPACE_2, SPACE_3, STATUSBAR_H, TEXT_LABEL};

fn key<'a, M: 'a>(label: &'a str) -> Element<'a, M> {
    text(label).font(MONO).size(TEXT_LABEL).style(theme::faint).boxed()
}

/// The value font: mono regular, or semibold for a value that is changing under a drag.
fn value_font(changing: bool) -> iced::Font {
    if changing {
        theme::MONO_SEMIBOLD
    } else {
        MONO
    }
}

fn value<'a, M: 'a>(value: String, changing: bool) -> Element<'a, M> {
    text(value).font(value_font(changing)).size(type_scale::NUM_BODY.size).style(theme::ink).boxed()
}

/// How a readout is changing under a drag: whether it is (semibold), by how much (`+10.6`, in `ink-muted` after the value) and
/// the colour the value takes (the margin's stability band).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Live {
    /// The value differs from what it was when the drag began.
    pub changing: bool,
    /// The signed change, already formatted.
    pub delta: Option<String>,
    /// The colour of the value instead of `ink`.
    pub color: Option<iced::Color>,
}

/// `text_value` as a cell's value, with the live change after it.
fn live_value<'a, M: 'a>(text_value: String, live: Live) -> Element<'a, M> {
    let mut value = text(text_value).font(value_font(live.changing)).size(type_scale::NUM_BODY.size);
    value = match live.color {
        Some(color) => value.color(color),
        None => value.style(theme::ink),
    };
    match live.delta {
        Some(delta) => row![value, text(format!("({delta})")).font(MONO).size(TEXT_LABEL).style(theme::muted)].spacing(SPACE_1).align_y(iced::Center).boxed(),
        None => value.boxed(),
    }
}

/// One ruled cell around `content`. With a message the whole cell is a button (`bg-400` on hover).
pub fn cell<'a, M: Clone + 'a>(content: Element<'a, M>, on_press: Option<M>) -> Element<'a, M> {
    let body = container(content).padding([0.0, SPACE_3]).center_y(Fill).height(Fill);
    let cell: Element<'a, M> = match on_press {
        Some(message) => button(body).padding(0).height(Fill).style(theme::ghost).on_press(message).boxed(),
        None => body.boxed(),
    };
    row![cell, container(Space::new()).width(1).height(Fill).style(theme::hairline)].height(Fill).boxed()
}

/// A plain `KEY value` readout (`MASS 1.284 kg`, `LEN 1142.0 mm`).
pub fn readout<'a, M: Clone + 'a>(label: &'a str, text_value: impl Into<String>, on_press: Option<M>) -> Element<'a, M> {
    readout_live(label, text_value, on_press, Live::default())
}

/// [`readout`] with `live`: while a drag changes the value it is `ink` at weight 600 so the eye can track it (StatusStrip
/// README), with the change after it; it goes back to regular on release.
pub fn readout_live<'a, M: Clone + 'a>(label: &'a str, text_value: impl Into<String>, on_press: Option<M>, live: Live) -> Element<'a, M> {
    cell(row![key(label), live_value(text_value.into(), live)].spacing(SPACE_2).align_y(iced::Center).boxed(), on_press)
}

/// The CG readout: the quartered-circle glyph, `CG` in blue and the value. `shown` says whether its marker is drawn in the
/// viewport and lanes; a hidden marker's cell is dimmed (the cell toggles it).
pub fn cg<'a, M: Clone + 'a>(text_value: impl Into<String>, shown: bool, on_press: Option<M>) -> Element<'a, M> {
    cg_live(text_value, shown, on_press, Live::default())
}

/// [`cg`] with `live`: the value is being dragged (semibold, with its change).
pub fn cg_live<'a, M: Clone + 'a>(text_value: impl Into<String>, shown: bool, on_press: Option<M>, live: Live) -> Element<'a, M> {
    marker(Mark::Cg, Tone::Cg, "CG", text_value.into(), shown, on_press, live)
}

/// The CP readout: the dot-in-ring glyph, `CP` in red and the value. See [`cg`] for `shown`.
pub fn cp<'a, M: Clone + 'a>(text_value: impl Into<String>, shown: bool, on_press: Option<M>) -> Element<'a, M> {
    cp_live(text_value, shown, on_press, Live::default())
}

/// [`cp`] with `live`: the value is being dragged (semibold, with its change).
pub fn cp_live<'a, M: Clone + 'a>(text_value: impl Into<String>, shown: bool, on_press: Option<M>, live: Live) -> Element<'a, M> {
    marker(Mark::Cp, Tone::Cp, "CP", text_value.into(), shown, on_press, live)
}

fn marker<'a, M: Clone + 'a>(mark: Mark, tone: Tone, label: &'a str, text_value: String, shown: bool, on_press: Option<M>, live: Live) -> Element<'a, M> {
    let pick: fn(&crate::tokens::Tokens) -> iced::Color = match (tone, shown) {
        (_, false) => |t| t.ink_faint,
        (Tone::Cg, _) => |t| t.cg,
        _ => |t| t.cp,
    };
    let coloured = text(label).font(MONO).size(TEXT_LABEL).style(theme::colored(pick));
    let glyph = if shown { glyph::icon(mark, tone, 13.0) } else { glyph::icon(mark, Tone::Muted, 13.0) };
    let value = if shown { live_value(text_value, live) } else { text(text_value).font(MONO).size(type_scale::NUM_BODY.size).style(theme::faint).boxed() };
    cell(row![glyph, coloured, value].spacing(SPACE_1 + 2.0).align_y(iced::Center).boxed(), on_press)
}

/// The stability cell: `STAB`, the compact gauge and the margin in the strong weight.
pub fn stability<'a, M: Clone + 'a>(state: &GaugeState, bands: &Bands, on_press: Option<M>) -> Element<'a, M> {
    stability_live(state, bands, on_press, Live::default())
}

/// [`stability`] with `live`: the margin is a value being dragged (semibold, with its change and its band's colour).
pub fn stability_live<'a, M: Clone + 'a>(state: &GaugeState, bands: &Bands, on_press: Option<M>, live: Live) -> Element<'a, M> {
    let margin = live_value(format!("{:.2} cal", state.target()), live);
    cell(row![key("STAB"), gauge::compact(state, bands), margin].spacing(SPACE_2).align_y(iced::Center).boxed(), on_press)
}

/// The Config cell, first in the strip: `CONFIG H128W ▴`. The caret points up because the menu opens upward.
pub fn config<'a, M: Clone + 'a>(name: &'a str, open: bool, on_press: Option<M>) -> Element<'a, M> {
    let tone = if open { Tone::Accent } else { Tone::Muted };
    let caret = glyph::icon(Mark::CaretUp, tone, 10.0);
    cell(row![key("CONFIG"), value(name.to_owned(), false), Space::new().width(SPACE_2), caret].spacing(SPACE_2).align_y(iced::Center).boxed(), on_press)
}

/// The strip: `cells` left to right, then free space, then `trailing` (the warnings chip, the selection summary) right-aligned.
pub fn strip<'a, M: Clone + 'a>(cells: Vec<Element<'a, M>>, trailing: Vec<Element<'a, M>>) -> Element<'a, M> {
    let tail = container(row(trailing).spacing(SPACE_2).align_y(iced::Center)).padding([0.0, SPACE_2]).center_y(Fill).height(Fill);
    container(row(cells).push(Space::new().width(Fill).boxed()).push(tail.boxed()).height(Fill)).height(STATUSBAR_H).width(Fill).style(theme::bar).boxed()
}
