//! Multi-selection (design/components/Inspector: *Multi-select shows only the fields the selected parts share*).
//!
//! `Ctx::selection` stays one value (`Selection::Part(primary)`); the parts selected *with* it live in [`Extra`], tied to that
//! primary: a plain `ctx.selection = ..` anywhere in the app (which every area does) that names another thing silently drops the
//! extras, so only the two places that build a multi-selection ([`Ctx::toggle_part`], [`Ctx::select_parts`]) need to know.
//!
//! Shift-click and Cmd/Ctrl-click in the viewport and in Station Lanes toggle a part in or out of the selection; a plain click
//! selects one part; Esc or a click on empty space clears. With several parts selected the viewport draws no handles (they edit
//! one part's dimension, and the kit has no multi-handle gesture), the inspector edits the shared fields, Delete removes every
//! part, and the status strip sums their mass.

use bowshock_model::{PartId, Rocket};

use super::{Ctx, Selection};

/// The parts selected in addition to the primary one (`Ctx::selection`).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Extra {
    /// The primary part these were added to; the extras are void when the selection is anything else.
    primary: Option<PartId>,
    /// The other selected parts, in the order they were added.
    parts: Vec<PartId>,
}

impl Extra {
    /// The extras that go with `selection` (none if it is not the primary they were added to).
    fn of(&self, selection: Selection) -> &[PartId] {
        match (selection, self.primary) {
            (Selection::Part(p), Some(q)) if p == q => &self.parts,
            _ => &[],
        }
    }

    /// Forgets every extra part.
    pub fn clear(&mut self) {
        self.primary = None;
        self.parts.clear();
    }

    /// Drops the extras that no longer exist in `rocket`, and all of them if `selection` is not their primary.
    pub fn validate(&mut self, selection: Selection, rocket: &Rocket) {
        if self.of(selection).is_empty() {
            self.clear();
        } else {
            self.parts.retain(|id| rocket.kind_of(*id).is_some());
        }
    }
}

/// Toggles `id` in the selection `(selection, extra)`: added when absent, removed when present (the oldest remaining part
/// becomes the primary when the primary goes). A selection of something else (nothing, a separation) starts a new one.
fn toggle(selection: Selection, extra: &mut Extra, id: PartId) -> Selection {
    let Selection::Part(primary) = selection else {
        extra.clear();
        return Selection::Part(id);
    };
    let mut all: Vec<PartId> = std::iter::once(primary).chain(extra.of(selection).iter().copied()).collect();
    match all.iter().position(|p| *p == id) {
        Some(at) => {
            all.remove(at);
        }
        None => all.push(id),
    }
    let mut parts = all.into_iter();
    let Some(first) = parts.next() else {
        extra.clear();
        return Selection::None;
    };
    extra.primary = Some(first);
    extra.parts = parts.collect();
    Selection::Part(first)
}

impl Ctx {
    /// Every selected part: the primary first, then the others in the order they were added. Empty unless parts are selected.
    pub fn selected_parts(&self) -> Vec<PartId> {
        match self.selection {
            Selection::Part(p) => std::iter::once(p).chain(self.multi.of(self.selection).iter().copied()).collect(),
            _ => Vec::new(),
        }
    }

    /// Whether more than one part is selected.
    pub fn is_multi(&self) -> bool {
        !self.multi.of(self.selection).is_empty()
    }

    /// Shift-click / Cmd/Ctrl-click on part `id`: adds it to the selection, or removes it if it is already selected.
    pub fn toggle_part(&mut self, id: PartId) {
        self.selection = toggle(self.selection, &mut self.multi, id);
        self.symmetry.instance = None;
    }

    /// A plain click: selects exactly `id` (nothing for `None`), dropping every other selected part.
    pub fn select_one(&mut self, id: Option<PartId>) {
        self.multi.clear();
        self.selection = id.map_or(Selection::None, Selection::Part);
    }

    /// Selects exactly `parts` (the first is the primary); an empty list selects nothing.
    #[cfg(test)]
    pub fn select_parts(&mut self, parts: &[PartId]) {
        let mut it = parts.iter().copied();
        match it.next() {
            Some(first) => {
                self.selection = Selection::Part(first);
                self.multi.primary = Some(first);
                self.multi.parts = it.filter(|p| *p != first).collect();
            }
            None => {
                self.selection = Selection::None;
                self.multi.clear();
            }
        }
    }

    /// Esc: a multi-selection is cleared. Returns whether it was one.
    pub fn clear_multi(&mut self) -> bool {
        let multi = self.is_multi();
        if multi {
            self.selection = Selection::None;
            self.multi.clear();
        }
        multi
    }
}

#[cfg(test)]
#[allow(unused_must_use)] // The tests drive `update` directly and have no runtime to hand its tasks to.
mod tests {
    use super::*;
    use crate::app::{drag, App, Message};
    use crate::viewport::pick::Pick;
    use crate::viewport::Message as Vp;
    use bowshock_model::{NoseShape, PartKind};

    /// An app holding a nose, a tube and a transition (fore to aft).
    fn app() -> (App, Vec<PartId>) {
        let mut app = App::new();
        let ctx = &mut app.ctx;
        drag::place(ctx, 0, PartKind::NoseCone { shape: NoseShape::TangentOgive, length: 0.2, diameter: 0.1 });
        drag::place(ctx, 1, PartKind::BodyTube { length: 0.5, diameter: 0.1 });
        drag::place(ctx, 2, PartKind::Transition { length: 0.1, fore_diameter: 0.1, aft_diameter: 0.06 });
        app.after_update();
        let ids = (0..3).map(|i| app.ctx.doc.rocket().parts()[i].id).collect();
        (app, ids)
    }

    #[test]
    fn toggling_adds_and_removes() {
        let mut extra = Extra::default();
        let s = toggle(Selection::None, &mut extra, 1);
        assert_eq!(s, Selection::Part(1));
        let s = toggle(s, &mut extra, 2);
        let s = toggle(s, &mut extra, 3);
        assert_eq!((s, extra.of(s).to_vec()), (Selection::Part(1), vec![2, 3]));
        // Removing a secondary part keeps the primary; removing the primary promotes the next.
        let s = toggle(s, &mut extra, 2);
        assert_eq!((s, extra.of(s).to_vec()), (Selection::Part(1), vec![3]));
        let s = toggle(s, &mut extra, 1);
        assert_eq!((s, extra.of(s).to_vec()), (Selection::Part(3), vec![]));
        let s = toggle(s, &mut extra, 3);
        assert_eq!(s, Selection::None);
        // A separation selection is replaced, not extended.
        assert_eq!(toggle(Selection::Separation(2), &mut extra, 4), Selection::Part(4));
    }

    #[test]
    fn a_plain_selection_drops_the_extras() {
        let (mut app, ids) = app();
        let ctx = &mut app.ctx;
        ctx.select_parts(&ids[..3]);
        assert_eq!(ctx.selected_parts(), ids[..3].to_vec());
        assert!(ctx.is_multi());
        // Any code that sets the selection directly (the inspector, the parts bin, an undo) makes a single selection.
        ctx.selection = Selection::Part(ids[1]);
        assert_eq!(ctx.selected_parts(), vec![ids[1]]);
        assert!(!ctx.is_multi());
        // ... for good: the app validates after every message, so the old primary coming back does not revive the extras.
        app.after_update();
        app.ctx.selection = Selection::Part(ids[0]);
        assert_eq!(app.ctx.selected_parts(), vec![ids[0]]);
    }

    #[test]
    fn viewport_toggle_and_plain_select() {
        let (mut app, ids) = app();
        app.update(Message::Viewport(Vp::SelectPart(Some(ids[0]))));
        app.update(Message::Viewport(Vp::Toggle(Pick::Axial(ids[1]))));
        app.update(Message::Viewport(Vp::Toggle(Pick::Axial(ids[2]))));
        assert_eq!(app.ctx.selected_parts(), vec![ids[0], ids[1], ids[2]]);
        app.update(Message::Viewport(Vp::Toggle(Pick::Axial(ids[0]))));
        assert_eq!(app.ctx.selected_parts(), vec![ids[1], ids[2]]);
        app.update(Message::Viewport(Vp::SelectPart(Some(ids[2]))));
        assert_eq!(app.ctx.selected_parts(), vec![ids[2]]);
        app.update(Message::Viewport(Vp::Toggle(Pick::Axial(ids[0]))));
        app.update(Message::Cancel);
        assert_eq!(app.ctx.selection, Selection::None, "Esc clears a multi-selection");
        assert!(app.ctx.selected_parts().is_empty());
    }

    #[test]
    fn undo_keeps_the_selection_valid() {
        let (mut app, ids) = app();
        app.ctx.select_parts(&ids[..3]);
        app.update(Message::Delete);
        assert!(app.ctx.selected_parts().is_empty());
        assert!(ids[..3].iter().all(|id| app.ctx.doc.rocket().part(*id).is_none()), "all three are deleted");
        app.update(Message::Undo);
        assert!(ids[..3].iter().all(|id| app.ctx.doc.rocket().part(*id).is_some()), "one undo brings all three back");
        // Select again, then undo something that removes one of them: the survivors stay selected.
        app.ctx.select_parts(&ids[..3]);
        let before = app.ctx.doc.rocket().clone();
        app.ctx.doc.edit("Remove", |r| {
            r.remove(ids[1]);
        });
        app.after_update();
        assert_eq!(app.ctx.selected_parts(), vec![ids[0], ids[2]]);
        app.update(Message::Undo);
        assert_eq!(app.ctx.doc.rocket().parts().len(), before.parts().len());
        assert!(app.ctx.selected_parts().iter().all(|id| app.ctx.doc.rocket().kind_of(*id).is_some()));
    }
}
