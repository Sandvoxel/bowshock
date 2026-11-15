//! The "Add here…" menu: right-click on empty space in a lane lists only that system's parts and adds the one chosen at the
//! station clicked.

use bowshock_model::{InternalKind, PartKind, RadialKind, System};
use iced::widget::{button, column, container, text};
use iced::{Element, Fill, Padding, Point, Widget};

use super::edit;
use super::Message;
use crate::app::drag;
use crate::app::ui;
use crate::app::{Ctx, Selection};
use crate::build::parts_bin::{self, items};
use crate::theme::{self, MONO};
use crate::tokens::TEXT_LABEL;

/// Width of the menu.
const WIDTH: f32 = 176.0;
/// Height of one entry.
const ROW: f32 = 24.0;

/// An open menu.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Open {
    /// Where it was opened, in the pane.
    pub at: Point,
    /// The lane.
    pub system: System,
    /// The station clicked.
    pub station: f64,
    /// Width of the pane.
    pub width: f32,
}

/// What an entry adds.
#[derive(Debug, Clone, PartialEq)]
pub enum Entry {
    /// An airframe part.
    Axial(&'static str, PartKind),
    /// A radial part.
    Radial(&'static str, RadialKind),
    /// An internal part.
    Internal(&'static str, InternalKind),
}

impl Entry {
    /// The name shown.
    pub fn name(&self) -> &'static str {
        match self {
            Entry::Axial(n, _) | Entry::Radial(n, _) | Entry::Internal(n, _) => n,
        }
    }
}

/// The parts of `system`, from the parts bin's catalogues.
pub fn entries(system: System) -> Vec<Entry> {
    let mut out = Vec::new();
    match system {
        System::Structure => {
            out.extend(
                parts_bin::axial_catalogue()
                    .into_iter()
                    .filter(|(n, _)| ["Nose cone", "Body tube", "Transition", "Boattail"].contains(n))
                    .map(|(n, k)| Entry::Axial(n, k)),
            );
        }
        System::Aero => out.extend(parts_bin::radial_catalogue().into_iter().map(|(n, k)| Entry::Radial(n, k))),
        _ => {}
    }
    out.extend(parts_bin::internal_catalogue().into_iter().filter(|(_, k)| k.system() == system).map(|(n, k)| Entry::Internal(n, k)));
    out
}

/// Adds entry `index` of `system`'s menu at `station` as one undo step, and selects it.
pub fn add(ctx: &mut Ctx, system: System, station: f64, index: usize) {
    let Some(entry) = entries(system).into_iter().nth(index) else { return };
    let rocket = ctx.doc.rocket();
    match entry {
        Entry::Axial(_, kind) => match edit::nearest_node(rocket, &kind, station) {
            Some((node, _)) => {
                drag::place(ctx, node, kind);
            }
            None => ctx.notify("It does not fit there"),
        },
        Entry::Radial(name, kind) => {
            let Some((parent, offset)) = edit::radial_landing(rocket, &kind, station - kind.extent() / 2.0) else {
                ctx.notify("Nothing to attach it to");
                return;
            };
            let count = ctx.drag.symmetry;
            let label = format!("Add {}", name.to_lowercase());
            if let Some(id) = ctx.doc.edit(&label, |r| r.attach_radial(parent, offset, 0.0, count, kind).ok()) {
                ctx.selection = Selection::Part(id);
            }
        }
        Entry::Internal(name, kind) => {
            let extent = kind.extent();
            let Some((mut parent, mut offset)) = edit::internal_landing(rocket, extent, station - extent / 2.0) else {
                ctx.notify("Nothing to put it in");
                return;
            };
            // A nose cone holds nothing: go on into the part behind it.
            if rocket.part(parent).is_some_and(|p| matches!(p.kind, PartKind::NoseCone { .. })) {
                match rocket.index_of(parent).and_then(|i| rocket.parts().get(i + 1)) {
                    Some(next) => (parent, offset) = (next.id, 0.0),
                    None => {
                        ctx.notify("Nothing to put it in");
                        return;
                    }
                }
            }
            let kind = items::host_inner(rocket, parent).map_or(kind.clone(), |inner| items::fitted(&kind, inner));
            let label = format!("Add {}", name.to_lowercase());
            if let Some(id) = ctx.doc.edit(&label, |r| r.attach_internal(parent, offset, kind).ok()) {
                ctx.selection = Selection::Part(id);
            }
        }
    }
}

/// The menu element, positioned at the click and kept inside the pane.
pub fn view<'a>(open: &Open, ctx: &Ctx) -> Element<'a, Message> {
    let heading = ui::label(format!("ADD HERE · {}", ui::bare(open.station, crate::units::Quantity::Length, ctx.units)));
    let rows = entries(open.system).into_iter().enumerate().map(|(i, entry)| {
        button(text(entry.name()).size(TEXT_LABEL).font(MONO)).width(Fill).padding([3.0, 8.0]).style(theme::ghost).on_press(Message::Add(i)).boxed()
    });
    let content = column![container(heading).padding([4.0, 8.0])].extend(rows);
    let height = 24.0 + ROW * entries(open.system).len() as f32;
    let x = open.at.x.min((open.width - WIDTH - 4.0).max(0.0));
    let y = open.at.y.min((super::height(false) - height - 2.0).max(0.0));
    container(container(content).width(WIDTH).style(theme::overlay))
        .width(Fill)
        .height(Fill)
        .padding(Padding { top: y, left: x, right: 0.0, bottom: 0.0 })
        .boxed()
}
