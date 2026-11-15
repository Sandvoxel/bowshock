//! SegmentedControl (design/components/SegmentedControl): a square strip of mutually exclusive options. The strip is
//! `bg-100` in a 1 px `line-200` frame, segments are split by `line-100` rules and the active one is `bg-300` with a 2 px
//! `accent` underline. Numeric options (fin count, angle snap) use the mono face.
//!
//! ```ignore
//! segmented(&[(3, "3"), (4, "4"), (6, "6")], self.fins, Message::Fins)
//! ```

use iced::widget::{button, column, container, row, text, Space};
use iced::{Element, Fill, Length, Theme, Widget};

use crate::theme::{self, MONO};
use crate::tokens::{CONTROL_H, SPACE_3, TEXT_BODY};

/// A row of mutually exclusive `options` with `selected` active; `on_select` makes the message for a clicked option.
///
/// More than six options belong in a [`select`](super::field::select) instead.
pub fn segmented<'a, T, M>(options: &[(T, &'a str)], selected: T, on_select: impl Fn(T) -> M) -> Element<'a, M>
where
    T: Copy + PartialEq + 'a,
    M: Clone + 'a,
{
    strip(options, selected, on_select, false)
}

/// Like [`segmented`], but the strip fills the available width and the segments share it equally.
pub fn segmented_fill<'a, T, M>(options: &[(T, &'a str)], selected: T, on_select: impl Fn(T) -> M) -> Element<'a, M>
where
    T: Copy + PartialEq + 'a,
    M: Clone + 'a,
{
    strip(options, selected, on_select, true)
}

fn is_numeric(label: &str) -> bool {
    !label.is_empty() && label.chars().all(|c| c.is_ascii_digit() || matches!(c, '.' | '°' | '×' | '-' | '+'))
}

fn strip<'a, T, M>(options: &[(T, &'a str)], selected: T, on_select: impl Fn(T) -> M, fill: bool) -> Element<'a, M>
where
    T: Copy + PartialEq + 'a,
    M: Clone + 'a,
{
    let width = if fill { Fill } else { Length::Shrink };
    let mut cells: Vec<Element<'a, M>> = Vec::with_capacity(options.len() * 2);
    for (i, &(value, name)) in options.iter().enumerate() {
        if i > 0 {
            cells.push(container(Space::new()).width(1).height(CONTROL_H - 2.0).style(theme::hairline).boxed());
        }
        let active = value == selected;
        let mut label = text(name).size(TEXT_BODY);
        if is_numeric(name) {
            label = label.font(MONO);
        }
        let label = if fill { container(label).center_x(Fill).boxed() } else { label.boxed() };
        cells.push(
            column![
                button(label).width(width).height(CONTROL_H - 4.0).padding([0.0, SPACE_3]).style(theme::segment(active)).on_press(on_select(value)),
                container(Space::new()).width(Fill).height(2).style(theme::underline(active)),
            ]
            .width(width)
            .boxed(),
        );
    }
    container(row(cells)).width(width).padding(1).style(|theme: &Theme| theme::segmented_frame(theme)).boxed()
}
