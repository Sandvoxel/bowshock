//! A primary item plus the others selected with it: the multi-select model shared by the parts (`app::multi`) and the flights
//! (`simulate`).

use iced::keyboard;

/// How a click changes a selection (the kit's multi-select conventions): a plain click selects one, Cmd/Ctrl-click toggles,
/// Shift-click selects the range from the last click, and Shift with Cmd/Ctrl adds that range to what is selected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PickMode {
    Only,
    Toggle,
    Range,
    AddRange,
}

impl PickMode {
    /// What a click does with `modifiers` held.
    pub fn from_modifiers(modifiers: keyboard::Modifiers) -> Self {
        match (modifiers.shift(), modifiers.command() || modifiers.control()) {
            (true, true) => PickMode::AddRange,
            (true, false) => PickMode::Range,
            (false, true) => PickMode::Toggle,
            (false, false) => PickMode::Only,
        }
    }
}

/// The selected items: a primary, the others selected with it in the order they were added, and where a range starts.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MultiSet<Id: Copy + Eq> {
    primary: Option<Id>,
    extras: Vec<Id>,
    /// Where a Shift-click range starts: the last item clicked.
    anchor: Option<Id>,
}

impl<Id: Copy + Eq> Default for MultiSet<Id> {
    fn default() -> Self {
        Self { primary: None, extras: Vec::new(), anchor: None }
    }
}

impl<Id: Copy + Eq> MultiSet<Id> {
    /// A selection of just `id`.
    pub fn with_primary(id: Id) -> Self {
        Self { primary: Some(id), extras: Vec::new(), anchor: Some(id) }
    }

    pub fn primary(&self) -> Option<Id> {
        self.primary
    }

    /// Selects nothing.
    pub fn clear(&mut self) {
        self.only(None);
    }

    /// Selects exactly `id`.
    pub fn only(&mut self, id: Option<Id>) {
        self.primary = id;
        self.extras.clear();
        self.anchor = id;
    }

    /// Toggles `id`. Removing the primary promotes the oldest remaining item; removing the last one empties the set only if
    /// `allow_empty`. Adding keeps the primary when `allow_empty` (parts), else the new item becomes the primary (flights).
    /// Returns the primary.
    pub fn toggle(&mut self, id: Id, allow_empty: bool) -> Option<Id> {
        if self.primary == Some(id) {
            if self.extras.is_empty() {
                if allow_empty {
                    self.only(None);
                }
            } else {
                let next = self.extras.remove(0);
                self.primary = Some(next);
                self.anchor = Some(next);
            }
        } else if let Some(pos) = self.extras.iter().position(|x| *x == id) {
            self.extras.remove(pos);
        } else if allow_empty {
            if self.primary.is_none() {
                self.primary = Some(id);
                self.anchor = Some(id);
            } else {
                self.extras.push(id);
            }
        } else {
            if let Some(prev) = self.primary {
                self.extras.push(prev);
            }
            self.primary = Some(id);
            self.anchor = Some(id);
        }
        self.primary
    }

    /// Selects the range from the anchor to `target` in `ordered_universe` (added to the selection if `additive`); `target`
    /// becomes the primary and the anchor stays.
    pub fn range(&mut self, target: Id, ordered_universe: &[Id], additive: bool) {
        let at = |id: Id| ordered_universe.iter().position(|x| *x == id);
        let (Some(from), Some(to)) = (self.anchor.or(self.primary).and_then(at), at(target)) else { return };
        let mut all = if additive { self.ids_insertion_order() } else { Vec::new() };
        for item in &ordered_universe[from.min(to)..=from.max(to)] {
            if !all.contains(item) {
                all.push(*item);
            }
        }
        self.primary = Some(target);
        self.extras = all.into_iter().filter(|x| *x != target).collect();
    }

    /// Selects every item of `ordered_universe`, keeping the primary when it is among them.
    pub fn select_all(&mut self, ordered_universe: &[Id]) {
        self.primary = self.primary.filter(|p| ordered_universe.contains(p)).or_else(|| ordered_universe.first().copied());
        self.anchor = self.primary;
        let primary = self.primary;
        self.extras = ordered_universe.iter().copied().filter(|x| Some(*x) != primary).collect();
    }

    /// Selects exactly `ids` (the first is the primary); an empty list selects nothing.
    #[cfg(test)]
    pub fn set_exact(&mut self, ids: &[Id]) {
        self.only(ids.first().copied());
        for id in ids.iter().skip(1) {
            if Some(*id) != self.primary && !self.extras.contains(id) {
                self.extras.push(*id);
            }
        }
    }

    /// Drops the items for which `exists` is false; a vanished primary is replaced by the oldest remaining item.
    pub fn retain(&mut self, exists: impl Fn(Id) -> bool) {
        self.extras.retain(|x| exists(*x));
        if self.primary.is_some_and(|p| !exists(p)) {
            self.primary = if self.extras.is_empty() { None } else { Some(self.extras.remove(0)) };
        }
        if self.anchor.is_some_and(|a| !exists(a)) {
            self.anchor = self.primary;
        }
    }

    /// Voids the extras when the primary was changed from outside (`current` is what is selected now).
    pub fn sync_external_primary(&mut self, current: Option<Id>) {
        if current != self.primary {
            self.only(current);
        }
    }

    /// The primary, then the others in the order they were added.
    pub fn ids_insertion_order(&self) -> Vec<Id> {
        self.primary.into_iter().chain(self.extras.iter().copied()).collect()
    }

    /// The selected items of `universe`, in its order.
    pub fn ids_in_order(&self, universe: &[Id]) -> Vec<Id> {
        universe.iter().copied().filter(|id| self.contains(*id)).collect()
    }

    /// Whether more than one item is selected.
    pub fn is_multi(&self) -> bool {
        self.primary.is_some() && !self.extras.is_empty()
    }

    pub fn contains(&self, id: Id) -> bool {
        self.primary == Some(id) || self.extras.contains(&id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn modifiers_pick_a_mode() {
        assert_eq!(PickMode::from_modifiers(keyboard::Modifiers::empty()), PickMode::Only);
        assert_eq!(PickMode::from_modifiers(keyboard::Modifiers::SHIFT), PickMode::Range);
        assert_eq!(PickMode::from_modifiers(keyboard::Modifiers::CTRL), PickMode::Toggle);
        assert_eq!(PickMode::from_modifiers(keyboard::Modifiers::SHIFT | keyboard::Modifiers::CTRL), PickMode::AddRange);
    }

    #[test]
    fn toggling_parts_keeps_the_primary_and_may_empty() {
        let mut s = MultiSet::default();
        assert_eq!(s.toggle(1, true), Some(1));
        s.toggle(2, true);
        s.toggle(3, true);
        assert_eq!(s.ids_insertion_order(), vec![1, 2, 3]);
        s.toggle(2, true);
        assert_eq!(s.ids_insertion_order(), vec![1, 3]);
        assert_eq!(s.toggle(1, true), Some(3));
        assert!(!s.is_multi());
        assert_eq!(s.toggle(3, true), None);
    }

    #[test]
    fn toggling_flights_moves_the_primary_and_never_empties() {
        let mut s = MultiSet::with_primary(1);
        assert_eq!(s.toggle(1, false), Some(1));
        s.toggle(2, false);
        assert_eq!((s.primary(), s.ids_insertion_order()), (Some(2), vec![2, 1]));
        assert_eq!(s.toggle(2, false), Some(1));
        assert_eq!(s.toggle(1, false), Some(1));
        assert!(!s.is_multi());
    }

    #[test]
    fn ranges_run_from_the_anchor_and_keep_it() {
        let universe = [1, 2, 3, 4, 5];
        let mut s = MultiSet::with_primary(2);
        s.range(4, &universe, false);
        assert_eq!((s.primary(), s.ids_in_order(&universe)), (Some(4), vec![2, 3, 4]));
        s.range(5, &universe, false);
        assert_eq!(s.ids_in_order(&universe), vec![2, 3, 4, 5], "the anchor stays");
        let mut s = MultiSet::with_primary(1);
        s.toggle(4, true);
        s.range(3, &universe, true);
        assert_eq!(s.ids_in_order(&universe), vec![1, 2, 3, 4]);
        s.range(9, &universe, true);
        assert_eq!(s.primary(), Some(3), "a target outside the universe changes nothing");
    }

    #[test]
    fn retain_promotes_and_select_all_covers() {
        let mut s = MultiSet::default();
        s.set_exact(&[1, 2, 3]);
        s.retain(|id| id != 1);
        assert_eq!((s.primary(), s.ids_insertion_order()), (Some(2), vec![2, 3]));
        s.retain(|_| false);
        assert_eq!(s.primary(), None);
        s.select_all(&[7, 8, 9]);
        assert_eq!((s.primary(), s.ids_insertion_order()), (Some(7), vec![7, 8, 9]));
        assert!(s.contains(9));
    }

    #[test]
    fn a_foreign_primary_voids_the_extras() {
        let mut s = MultiSet::default();
        s.set_exact(&[1, 2]);
        s.sync_external_primary(Some(1));
        assert!(s.is_multi());
        s.sync_external_primary(Some(5));
        assert_eq!((s.primary(), s.is_multi()), (Some(5), false));
    }
}
