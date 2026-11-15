//! Inspector building blocks (design/components/Inspector): numbered section heads with a hairline, the 96 px label
//! column, selects and checkboxes. Fields sit `space-2` apart inside a section; sections are split by a hairline plus
//! `space-3`.
//!
//! ```ignore
//! section(1, "Geometry", true, vec![
//!     labeled("Fin count", segmented(..)),
//!     labeled("Root chord", unit_input(..).into()),
//! ])
//! ```

use std::borrow::Borrow;

use iced::widget::text::{Ellipsis, Wrapping};
use iced::widget::{checkbox as checkbox_widget, column, container, pick_list, row, text, Space};
use iced::{Element, Fill, Length, Widget};

use crate::theme::{self, MONO};
use crate::tokens::{CONTROL_H, SPACE_2, SPACE_3, TEXT_BODY, TEXT_LABEL};

/// Width of the label column of a field row.
pub const LABEL_W: f32 = 96.0;

/// A 1 px `line-100` horizontal rule.
pub fn rule<'a, M: 'a>() -> Element<'a, M> {
    container(Space::new()).width(Fill).height(1).style(theme::hairline).boxed()
}

/// The numbered, uppercase, mono section head: `01 GEOMETRY`.
pub fn section_head<'a, M: 'a>(index: usize, title: &str) -> Element<'a, M> {
    text(format!("{index:02} {}", title.to_uppercase())).font(MONO).size(TEXT_LABEL).style(theme::faint).boxed()
}

/// An inspector section: an optional full-bleed hairline above (`ruled`, false for the first), then the head and the
/// `fields` `space-2` apart, all with `space-3` around.
pub fn section<'a, M: 'a>(index: usize, title: &str, ruled: bool, fields: Vec<Element<'a, M>>) -> Element<'a, M> {
    section_picking(index, title, ruled, false, fields)
}

/// [`section`] that shows the accent `PICKING…` tag at the right of its head while a field inside it is in pick mode
/// (design/components/UnitInput, *Typing { starts a pick*).
pub fn section_picking<'a, M: 'a>(index: usize, title: &str, ruled: bool, picking: bool, fields: Vec<Element<'a, M>>) -> Element<'a, M> {
    let mut body: Vec<Element<'a, M>> = Vec::with_capacity(fields.len() + 1);
    if picking {
        let tag = text("PICKING\u{2026}").font(MONO).size(TEXT_LABEL).style(theme::colored(|t| t.accent));
        body.push(row![section_head(index, title), Space::new().width(Fill), tag].align_y(iced::Center).boxed());
    } else {
        body.push(section_head(index, title));
    }
    body.extend(fields);
    let padded = container(column(body).spacing(SPACE_2)).padding([SPACE_3, SPACE_3]).width(Fill);
    if ruled {
        column![rule(), padded].width(Fill).boxed()
    } else {
        padded.boxed()
    }
}

/// The label column: `ink-muted`, never wraps, truncates with an ellipsis (hover shows nothing yet; keep labels short).
pub fn label<'a, M: 'a>(label: &'a str) -> Element<'a, M> {
    container(text(label).size(TEXT_BODY).style(theme::muted).wrapping(Wrapping::None).ellipsis(Ellipsis::End).width(Fill))
        .width(LABEL_W)
        .height(CONTROL_H)
        .center_y(CONTROL_H)
        .clip(true)
        .boxed()
}

/// A field row: the 96 px label column and `control` filling the rest.
pub fn labeled<'a, M: 'a>(name: &'a str, control: Element<'a, M>) -> Element<'a, M> {
    row![label(name), container(control).width(Fill)].spacing(SPACE_2).align_y(iced::Center).boxed()
}

/// A select: the text-field look with the dropdown in an overlay frame. 7+ options belong here instead of a segmented control.
pub fn select<'a, T, L, M>(options: L, selected: Option<T>, on_select: impl Fn(T) -> M + 'a) -> Element<'a, M>
where
    T: ToString + PartialEq + Clone + 'a,
    L: Borrow<[T]> + 'a,
    M: Clone + 'a,
{
    pick_list(selected, options, |item: &T| item.to_string())
        .on_select(on_select)
        .width(Length::Fill)
        .text_size(TEXT_BODY)
        .padding([4.0, SPACE_2])
        .style(theme::select)
        .menu_style(theme::select_menu)
        .boxed()
}

/// A checkbox: square, `accent` when on, `ink-muted` label.
pub fn checkbox<'a, M: Clone + 'a>(label: &'a str, checked: bool, on_toggle: impl Fn(bool) -> M + 'a) -> Element<'a, M> {
    checkbox_widget(checked).label(label).size(12).text_size(TEXT_BODY).spacing(SPACE_2).style(theme::check).on_toggle(on_toggle).boxed()
}
