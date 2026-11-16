//! The Build screen: a `pane_grid` of the parts bin on the left, the centre (the viewport over Station Lanes) and the inspector
//! on the right. This module only composes; each pane is its own module with its own state and messages
//! ([`parts_bin`], [`crate::viewport`], [`lanes`], [`inspector`]). Station Lanes has a fixed height ([`lanes::height`]), so it is
//! stacked under the viewport rather than being a resizable pane.

pub mod inspector;
pub mod lanes;
pub mod parts_bin;

use iced::widget::column;
use iced::widget::pane_grid::{self, Axis, Configuration, PaneGrid};
use iced::{Element, Widget};

use crate::app::{Ctx, Message};
use crate::theme;
use crate::tokens::{BIN_W, RAIL_W, RIGHT_PANEL_W};
use crate::viewport;

/// The panes of the Build screen.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pane {
    /// Rail and parts list.
    PartsBin,
    /// The 3D viewport over Station Lanes.
    Centre,
    /// The inspector.
    Inspector,
}

/// The pane arrangement and its dividers.
pub struct Layout {
    panes: pane_grid::State<Pane>,
    sides_hidden: bool,
}

impl Layout {
    /// The kit's proportions for a 1280×800 window.
    pub fn new() -> Self {
        let width = 1280.0;
        let bin = (RAIL_W + BIN_W) / width;
        let inspector = RIGHT_PANEL_W / (width * (1.0 - bin));
        let layout = Configuration::Split {
            axis: Axis::Vertical,
            ratio: bin,
            a: Box::new(Configuration::Pane(Pane::PartsBin)),
            b: Box::new(Configuration::Split {
                axis: Axis::Vertical,
                ratio: 1.0 - inspector,
                a: Box::new(Configuration::Pane(Pane::Centre)),
                b: Box::new(Configuration::Pane(Pane::Inspector)),
            }),
        };
        Layout { panes: pane_grid::State::with_configuration(layout), sides_hidden: false }
    }

    /// Applies a divider drag.
    pub fn resize(&mut self, event: pane_grid::ResizeEvent) {
        self.panes.resize(event.split, event.ratio);
    }

    /// Tab: hides (or brings back) the parts bin and the inspector, leaving the viewport and lanes the whole screen.
    pub fn toggle_sides(&mut self) {
        self.sides_hidden = !self.sides_hidden;
    }

    /// Whether the side columns are folded away.
    #[cfg(test)]
    pub fn sides_hidden(&self) -> bool {
        self.sides_hidden
    }
}

/// The Build screen.
pub fn view<'a>(
    layout: &'a Layout,
    parts_bin: &'a parts_bin::State,
    viewport: &'a viewport::View,
    lanes: &'a lanes::State,
    inspector: &'a inspector::State,
    ctx: &'a Ctx,
) -> Element<'a, Message> {
    let centre = || column![viewport.view(ctx).map(Message::Viewport), lanes.view(ctx).map(Message::Lanes)];
    // Tab: the parts bin and the inspector fold away and the centre fills the window.
    if layout.sides_hidden {
        return centre().boxed();
    }
    PaneGrid::new(&layout.panes, |_, pane, _| {
        pane_grid::Content::new(match pane {
            Pane::PartsBin => parts_bin.view(ctx).map(Message::PartsBin).boxed(),
            Pane::Centre => centre().boxed(),
            Pane::Inspector => inspector.view(ctx).map(Message::Inspector).boxed(),
        })
    })
    .on_resize(6, Message::Resized)
    .style(theme::panes)
    .boxed()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tab_toggles_the_side_columns() {
        let mut layout = Layout::new();
        assert!(!layout.sides_hidden);
        layout.toggle_sides();
        assert!(layout.sides_hidden);
        layout.toggle_sides();
        assert!(!layout.sides_hidden);
    }
}
