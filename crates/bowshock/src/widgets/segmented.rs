//! SegmentedControl (design/components/SegmentedControl): a square strip of mutually exclusive options. The strip is
//! `bg-100` in a 1 px `line-200` frame, segments are split by `line-100` rules and the active one is `bg-300` with a 2 px
//! `accent` underline. Numeric options (fin count, angle snap) use the mono face.
//!
//! ```ignore
//! segmented(&[(3, "3"), (4, "4"), (6, "6")], self.fins, Message::Fins)
//! ```

use iced::widget::{button, column, container, row, text, Space};
use iced::{Element, Fill, Length, Theme, Widget};

use crate::theme::{self, MONO, SANS_MEDIUM};
use crate::tokens::{CONTROL_H, SPACE_3};

/// Label size of a segment (`.bs-seg button`: 12 px Plex Sans 500).
const SEGMENT_TEXT: f32 = 12.0;

/// How a strip lays its segments out.
#[derive(Clone, Copy, PartialEq)]
enum Layout {
    /// The strip and its segments hug their labels (the kit's inline-flex strip).
    Natural,
    /// The strip fills its column and the segments share it, weighted by label length so none clips.
    Equal,
    /// The strip fills its column but the segments keep their natural width; the remainder stays empty (Parts bin).
    Stretch,
}

/// Height, padding and type size of a dense strip (the Parts bin footer).
#[derive(Clone, Copy)]
pub struct Dense {
    /// Outer height of the strip in px.
    pub height: f32,
    /// Horizontal padding of each segment in px.
    pub pad: f32,
    /// Label size in px.
    pub size: f32,
}

/// A row of mutually exclusive `options` with `selected` active; `on_select` makes the message for a clicked option.
///
/// More than six options belong in a [`select`](super::field::select) instead.
pub fn segmented<'a, T, M>(options: &[(T, &'a str)], selected: T, on_select: impl Fn(T) -> M) -> Element<'a, M>
where
    T: Copy + PartialEq + 'a,
    M: Clone + 'a,
{
    strip(options, selected, on_select, Layout::Natural, Dense { height: CONTROL_H, pad: SPACE_3 - 2.0, size: SEGMENT_TEXT })
}

/// Like [`segmented`], but the strip fills the available width and the segments share it, weighted by label length.
pub fn segmented_fill<'a, T, M>(options: &[(T, &'a str)], selected: T, on_select: impl Fn(T) -> M) -> Element<'a, M>
where
    T: Copy + PartialEq + 'a,
    M: Clone + 'a,
{
    strip(options, selected, on_select, Layout::Equal, Dense { height: CONTROL_H, pad: 2.0, size: SEGMENT_TEXT })
}

/// A dense strip with natural-width segments (`dense` sets height, padding and size). With `stretch` the frame fills the
/// column and the segments stay left, as in the kit's Parts bin symmetry row.
pub fn segmented_dense<'a, T, M>(options: &[(T, &'a str)], selected: T, on_select: impl Fn(T) -> M, dense: Dense, stretch: bool) -> Element<'a, M>
where
    T: Copy + PartialEq + 'a,
    M: Clone + 'a,
{
    strip(options, selected, on_select, if stretch { Layout::Stretch } else { Layout::Natural }, dense)
}

fn is_numeric(label: &str) -> bool {
    !label.is_empty() && label.chars().all(|c| c.is_ascii_digit() || matches!(c, '.' | '°' | '×' | '-' | '+'))
}

fn strip<'a, T, M>(options: &[(T, &'a str)], selected: T, on_select: impl Fn(T) -> M, layout: Layout, dense: Dense) -> Element<'a, M>
where
    T: Copy + PartialEq + 'a,
    M: Clone + 'a,
{
    let equal = layout == Layout::Equal;
    let width = if layout == Layout::Natural { Length::Shrink } else { Fill };
    // Equal segments are weighted by label length (plus padding) so a long label never clips beside short ones.
    let weight = |name: &str| -> Length {
        if equal {
            Length::FillPortion((name.chars().count() as u16 + 3).max(4))
        } else {
            Length::Shrink
        }
    };
    let inner_h = dense.height - 2.0;
    let mut cells: Vec<Element<'a, M>> = Vec::with_capacity(options.len() * 2 + 1);
    for (i, &(value, name)) in options.iter().enumerate() {
        if i > 0 {
            cells.push(container(Space::new()).width(1).height(inner_h).style(theme::hairline).boxed());
        }
        let active = value == selected;
        let mut label = text(name).size(dense.size).font(SANS_MEDIUM);
        if is_numeric(name) {
            label = label.font(MONO);
        }
        let label = if equal { container(label).center_x(Fill).boxed() } else { label.boxed() };
        cells.push(
            column![
                button(label)
                    .width(if equal { Fill } else { Length::Shrink })
                    .height(inner_h - 2.0)
                    .padding([0.0, dense.pad])
                    .style(theme::segment(active))
                    .on_press(on_select(value)),
                container(Space::new()).width(Fill).height(2).style(theme::underline(active)),
            ]
            .width(weight(name))
            .boxed(),
        );
    }
    if layout == Layout::Stretch {
        cells.push(Space::new().width(Fill).height(inner_h).boxed());
    }
    container(row(cells)).width(width).padding(1).style(|theme: &Theme| theme::segmented_frame(theme)).boxed()
}
