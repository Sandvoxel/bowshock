//! The Build screen: a `pane_grid` of the parts bin on the left, the viewport over Station Lanes in the centre and the inspector
//! on the right. This module only composes; each pane is its own module with its own state and messages
//! ([`parts_bin`], [`crate::viewport`], [`lanes`], [`inspector`]).

pub mod inspector;
pub mod lanes;
pub mod parts_bin;

use iced::widget::pane_grid::{self, Axis, Configuration, PaneGrid};
use iced::{Element, Widget};

use crate::app::{Ctx, Message};
use crate::theme;
use crate::tokens::{BIN_W, LANES_H, RAIL_W, RIGHT_PANEL_W, STATUSBAR_H, TOPBAR_H};
use crate::viewport;

/// The panes of the Build screen.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pane {
    /// Rail and parts list.
    PartsBin,
    /// The 3D viewport.
    Viewport,
    /// Station Lanes.
    Lanes,
    /// The inspector.
    Inspector,
}

/// The pane arrangement and its dividers.
pub struct Layout {
    panes: pane_grid::State<Pane>,
}

impl Layout {
    /// The kit's proportions for a 1280×800 window.
    pub fn new() -> Self {
        let width = 1280.0;
        let bin = (RAIL_W + BIN_W) / width;
        let inspector = RIGHT_PANEL_W / (width * (1.0 - bin));
        let centre = Configuration::Split {
            axis: Axis::Horizontal,
            ratio: 1.0 - LANES_H / (800.0 - TOPBAR_H - STATUSBAR_H),
            a: Box::new(Configuration::Pane(Pane::Viewport)),
            b: Box::new(Configuration::Pane(Pane::Lanes)),
        };
        let layout = Configuration::Split {
            axis: Axis::Vertical,
            ratio: bin,
            a: Box::new(Configuration::Pane(Pane::PartsBin)),
            b: Box::new(Configuration::Split {
                axis: Axis::Vertical,
                ratio: 1.0 - inspector,
                a: Box::new(centre),
                b: Box::new(Configuration::Pane(Pane::Inspector)),
            }),
        };
        Layout { panes: pane_grid::State::with_configuration(layout) }
    }

    /// Applies a divider drag.
    pub fn resize(&mut self, event: pane_grid::ResizeEvent) {
        self.panes.resize(event.split, event.ratio);
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
    PaneGrid::new(&layout.panes, |_, pane, _| {
        pane_grid::Content::new(match pane {
            Pane::PartsBin => parts_bin.view(ctx).map(Message::PartsBin).boxed(),
            Pane::Viewport => viewport.view(ctx).map(Message::Viewport).boxed(),
            Pane::Lanes => lanes.view(ctx).map(Message::Lanes).boxed(),
            Pane::Inspector => inspector.view(ctx).map(Message::Inspector).boxed(),
        })
    })
    .on_resize(6, Message::Resized)
    .style(theme::panes)
    .boxed()
}
