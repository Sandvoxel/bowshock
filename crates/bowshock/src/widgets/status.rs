//! StatusStrip (design/components/StatusStrip, ConfigPicker's strip cell): the always-on vehicle summary in ruled cells.
//! Each cell is a mono uppercase `ink-faint` key plus a mono `ink` value; CG and CP carry their glyphs and colours; the
//! stability cell has the compact gauge; the config cell opens the configuration menu.
//!
//! ```ignore
//! status::strip(
//!     vec![
//!         status::config("H128W", self.config_open, Some(Message::ToggleConfigMenu)),
//!         status::readout("MASS", "1.284 kg", Some(Message::OpenMass)),
//!         status::cg("712.4", Some(Message::ToggleCg)),
//!         status::cp("858.7", Some(Message::ToggleCp)),
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

fn value<'a, M: 'a>(value: String) -> Element<'a, M> {
    text(value).font(MONO).size(type_scale::NUM_BODY.size).style(theme::ink).boxed()
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
    cell(row![key(label), value(text_value.into())].spacing(SPACE_2).align_y(iced::Center).boxed(), on_press)
}

/// The CG readout: the quartered-circle glyph, `CG` in blue and the value.
pub fn cg<'a, M: Clone + 'a>(text_value: impl Into<String>, on_press: Option<M>) -> Element<'a, M> {
    marker(Mark::Cg, Tone::Cg, "CG", text_value.into(), on_press)
}

/// The CP readout: the dot-in-ring glyph, `CP` in red and the value.
pub fn cp<'a, M: Clone + 'a>(text_value: impl Into<String>, on_press: Option<M>) -> Element<'a, M> {
    marker(Mark::Cp, Tone::Cp, "CP", text_value.into(), on_press)
}

fn marker<'a, M: Clone + 'a>(mark: Mark, tone: Tone, label: &'a str, text_value: String, on_press: Option<M>) -> Element<'a, M> {
    let coloured = text(label).font(MONO).size(TEXT_LABEL).style(match tone {
        Tone::Cg => theme::colored(|t| t.cg),
        _ => theme::colored(|t| t.cp),
    });
    cell(row![glyph::icon(mark, tone, 13.0), coloured, value(text_value)].spacing(SPACE_1 + 2.0).align_y(iced::Center).boxed(), on_press)
}

/// The stability cell: `STAB`, the compact gauge and the margin in the strong weight.
pub fn stability<'a, M: Clone + 'a>(state: &GaugeState, bands: &Bands, on_press: Option<M>) -> Element<'a, M> {
    let margin = text(format!("{:.2} cal", state.target())).font(theme::MONO).size(type_scale::NUM_BODY.size).style(theme::ink);
    cell(row![key("STAB"), gauge::compact(state, bands), margin].spacing(SPACE_2).align_y(iced::Center).boxed(), on_press)
}

/// The Config cell, first in the strip: `CONFIG H128W ▴`. The caret points up because the menu opens upward.
pub fn config<'a, M: Clone + 'a>(name: &'a str, open: bool, on_press: Option<M>) -> Element<'a, M> {
    let tone = if open { Tone::Accent } else { Tone::Muted };
    let caret = glyph::icon(Mark::CaretUp, tone, 10.0);
    cell(row![key("CONFIG"), value(name.to_owned()), Space::new().width(SPACE_2), caret].spacing(SPACE_2).align_y(iced::Center).boxed(), on_press)
}

/// The strip: `cells` left to right, then free space, then `trailing` (the warnings chip, the selection summary) right-aligned.
pub fn strip<'a, M: Clone + 'a>(cells: Vec<Element<'a, M>>, trailing: Vec<Element<'a, M>>) -> Element<'a, M> {
    let tail = container(row(trailing).spacing(SPACE_2).align_y(iced::Center)).padding([0.0, SPACE_2]).center_y(Fill).height(Fill);
    container(row(cells).push(Space::new().width(Fill).boxed()).push(tail.boxed()).height(Fill)).height(STATUSBAR_H).width(Fill).style(theme::bar).boxed()
}
