//! The flow fields of the points of the table on show: kept for the session beside the table, never saved.
//!
//! A solve brings back the final flow field of every point it solves ([`bowshock_core::Field`], about 0.5 MB on the coarse grid and
//! 1.1 MB on the medium one). The table in the document is a cache of drag numbers and stays small, so the fields live here, in the
//! app's state: not in the `.bowshock` file, not in undo. A table loaded from a file has none until it is solved again. They are
//! dropped as soon as the geometry they were solved for is no longer the document's (see [`FieldStore::retain_for`]).

use bowshock_core as solver;
use bowshock_model::CfdTable;
use std::fmt;
use std::sync::Arc;

/// A field carried by an [`Event`](super::Event): shared, because the events are cloned and a field is large.
#[derive(Clone)]
pub struct FieldRef(pub Arc<solver::Field>);

impl fmt::Debug for FieldRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let g = &*self.0;
        write!(f, "Field(Mach {}, {} x {} cells, iteration {})", g.mach, g.ni, g.nj, g.iteration)
    }
}

/// Which table a [`FieldStore`] belongs to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Stamp {
    /// The geometry the fields were solved for.
    geometry: u64,
    /// The table of the solve (its key and when it was solved), so another table of the same geometry (a file opened, an undo) has
    /// none of these fields.
    key: u64,
    solved_at: u64,
}

impl Stamp {
    fn of(table: &CfdTable) -> Stamp {
        Stamp { geometry: table.geometry, key: table.key, solved_at: table.solved_at }
    }
}

/// Two Mach numbers are the same node of a table.
fn same(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-9
}

/// The final fields of the table on show, one per Mach node.
#[derive(Debug, Default)]
pub struct FieldStore {
    stamp: Option<Stamp>,
    /// In ascending Mach order.
    fields: Vec<Arc<solver::Field>>,
}

impl FieldStore {
    /// Keeps `fields` (the final fields of the points of the solve that made `table`) in place of what was kept; those that are not
    /// of a Mach node of the table are left out.
    pub fn replace(&mut self, table: &CfdTable, fields: impl IntoIterator<Item = Arc<solver::Field>>) {
        let mut kept: Vec<Arc<solver::Field>> = fields.into_iter().filter(|f| table.points.iter().any(|p| same(p.mach, f.mach))).collect();
        kept.sort_by(|a, b| a.mach.total_cmp(&b.mach));
        kept.dedup_by(|a, b| same(a.mach, b.mach));
        self.stamp = (!kept.is_empty()).then(|| Stamp::of(table));
        self.fields = kept;
    }

    /// Drops the fields unless they belong to `table` and the geometry the document has now is `geometry` (`None`: the document
    /// has no geometry the solver takes).
    pub fn retain_for(&mut self, table: Option<&CfdTable>, geometry: Option<u64>) {
        let current = self.stamp.is_some_and(|s| table.is_some_and(|t| Stamp::of(t) == s) && geometry == Some(s.geometry));
        if !current {
            self.stamp = None;
            self.fields.clear();
        }
    }

    /// The field of the point at `mach` of `table`, if the store holds one for that table.
    pub fn get(&self, table: &CfdTable, mach: f64) -> Option<&Arc<solver::Field>> {
        (self.stamp == Some(Stamp::of(table))).then(|| self.fields.iter().find(|f| same(f.mach, mach))).flatten()
    }

    /// Whether the store holds the fields of `table`.
    pub fn has_any(&self, table: &CfdTable) -> bool {
        self.stamp == Some(Stamp::of(table)) && !self.fields.is_empty()
    }

    #[cfg(test)]
    /// The fields kept.
    pub fn len(&self) -> usize {
        self.fields.len()
    }

    #[cfg(test)]
    /// Whether there are none.
    pub fn is_empty(&self) -> bool {
        self.fields.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{CfdGrid, CfdPoint, CfdQuality};

    fn table(geometry: u64, solved_at: u64, machs: &[f64]) -> CfdTable {
        CfdTable {
            key: geometry ^ 0xabc,
            geometry,
            stage: 1,
            solved_at,
            solve_seconds: 1.0,
            backend: "CPU".to_string(),
            fallback: None,
            altitude: 0.0,
            notes: Vec::new(),
            points: machs
                .iter()
                .map(|&mach| CfdPoint { mach, cd: 0.5, friction: 0.1, pressure: 0.2, base: 0.1, fins: 0.1, other: 0.0, quality: CfdQuality::Converged })
                .collect(),
            grid: CfdGrid::Coarse,
            lift: None,
        }
    }

    fn field(mach: f64) -> Arc<solver::Field> {
        Arc::new(solver::Field {
            mach,
            iteration: 100,
            wake: solver::Wake::Base,
            ni: 1,
            nj: 1,
            x: vec![0.0, 1.0, 0.0, 1.0],
            r: vec![0.0, 0.0, 1.0, 1.0],
            solid: vec![false],
            rho: vec![1.0],
            u: vec![mach as f32],
            v: vec![0.0],
            p: vec![0.7],
            freestream: [1.0, mach, 0.0, 1.0 / solver::field::GAMMA],
        })
    }

    #[test]
    fn the_store_keeps_a_field_per_mach_node_of_its_table_and_nothing_for_another_table() {
        let t = table(7, 100, &[0.5, 1.0, 2.0]);
        let mut store = FieldStore::default();
        assert!(store.is_empty() && !store.has_any(&t));
        // a field of a Mach number the table does not have is left out, the rest come out in Mach order
        store.replace(&t, [field(2.0), field(0.5), field(1.5), field(1.0)]);
        assert_eq!(store.len(), 3);
        assert_eq!(store.get(&t, 1.0).map(|f| f.mach), Some(1.0));
        assert_eq!(store.get(&t, 2.0 + 1e-12).map(|f| f.mach), Some(2.0), "the same node within rounding");
        assert!(store.get(&t, 1.5).is_none());
        // the same geometry solved again is another table: its points have no field here
        let again = table(7, 101, &[0.5, 1.0, 2.0]);
        assert!(store.get(&again, 1.0).is_none() && !store.has_any(&again));
        // a solve replaces what was kept
        store.replace(&again, [field(1.0)]);
        assert_eq!((store.len(), store.get(&again, 1.0).is_some(), store.get(&t, 1.0).is_some()), (1, true, false));
    }

    #[test]
    fn the_fields_are_dropped_when_the_geometry_or_the_table_changes() {
        let t = table(7, 100, &[0.5, 1.0]);
        let mut store = FieldStore::default();
        store.replace(&t, [field(0.5), field(1.0)]);
        // the table with a 3D solve stored on it is the same table
        let lift = bowshock_model::CfdLiftTable { key: 1, solved_at: 5, solve_seconds: 1.0, backend: "CPU".to_string(), points: Vec::new(), notes: Vec::new() };
        let with_lift = CfdTable { lift: Some(lift), ..t.clone() };
        store.retain_for(Some(&with_lift), Some(7));
        assert_eq!(store.len(), 2);
        // the geometry changed: the table is stale and its fields go
        store.retain_for(Some(&t), Some(8));
        assert!(store.is_empty() && !store.has_any(&t));
        // a table of another solve (a file opened), or none at all, or a geometry the solver cannot take
        for (other, geometry) in [(Some(table(7, 200, &[0.5, 1.0])), Some(7)), (None, Some(7)), (Some(t.clone()), None)] {
            store.replace(&t, [field(0.5)]);
            store.retain_for(other.as_ref(), geometry);
            assert!(store.is_empty());
        }
    }
}
