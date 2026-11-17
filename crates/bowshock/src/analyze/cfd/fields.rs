//! The flow fields of the points of the table on show: kept for the session beside the table, never saved.
//!
//! A solve brings back the final flow field of every point it solves ([`bowshock_core::Field3`], about 0.5 MB on the coarse grid and
//! 1.1 MB on the medium one). The table in the document is a cache of drag numbers and stays small, so the fields live here, in the
//! app's state: not in the `.bowshock` file, not in undo. A table loaded from a file has none until it is solved again. They are
//! dropped as soon as the geometry they were solved for is no longer the document's (see [`FieldStore::retain_for`]).

use bowshock_core as solver;
use bowshock_model::CfdTable;
use std::fmt;
use std::sync::Arc;

/// A field carried by an [`Event`](super::Event): shared, because the events are cloned and a field is large.
#[derive(Clone)]
pub struct FieldRef(pub Arc<solver::Field3>);

impl FieldRef {
    /// The field of a 3D solve.
    pub fn of(field: solver::Field3) -> FieldRef {
        FieldRef(Arc::new(field))
    }

    /// The field of the axisymmetric 2D solver, as the one-cell wedge of a 3D field (see `flow_view::axi`, which goes with the 2D solver).
    pub fn from_2d(field: &solver::Field) -> FieldRef {
        FieldRef(Arc::new(crate::analyze::flow_view::axi::field3_from_2d(field)))
    }
}

impl fmt::Debug for FieldRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let g = &*self.0;
        write!(f, "Field3(Mach {}, alpha {}, {} x {} x {} cells, iteration {})", g.mach, g.alpha_deg, g.ni, g.nj, g.nk, g.iteration)
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

/// A field at an angle of attack: one of a 3D stability solve (the drag solve's are at zero incidence).
fn at_an_angle(f: &solver::Field3) -> bool {
    f.alpha_deg.abs() > 1e-6
}

/// Two angles of attack are the same (degrees).
fn same_alpha(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}

/// The final fields of the table on show, one per Mach node and angle of attack of the result.
#[derive(Debug, Default)]
pub struct FieldStore {
    stamp: Option<Stamp>,
    /// In ascending Mach order, then angle of attack.
    fields: Vec<Arc<solver::Field3>>,
}

impl FieldStore {
    /// Keeps `fields` (the final fields of the points of the solve that made `table`) in place of what was kept; those that are not
    /// of a Mach node of the table are left out. The fields at an angle of attack of the 3D stability solve of the same geometry stay
    /// (the stability table rides on a new drag table of the same geometry, see [`FieldStore::replace_stability`]).
    pub fn replace(&mut self, table: &CfdTable, fields: impl IntoIterator<Item = Arc<solver::Field3>>) {
        let stamp = Stamp::of(table);
        let mut kept: Vec<Arc<solver::Field3>> = fields.into_iter().filter(|f| table.points.iter().any(|p| same(p.mach, f.mach))).collect();
        if self.stamp.is_some_and(|s| s.geometry == stamp.geometry) {
            kept.extend(self.fields.iter().filter(|f| at_an_angle(f)).cloned());
        }
        Self::order(&mut kept);
        self.stamp = (!kept.is_empty()).then_some(stamp);
        self.fields = kept;
    }

    /// Keeps `fields` (the final fields of a 3D stability solve, at angles of attack) beside the fields of the drag solve of `table`, in
    /// place of the fields at an angle of attack that were kept. Their Mach numbers are those of the stability solve, not of the table.
    /// Nothing is kept for another table than the one the store holds fields for (or none yet).
    pub fn replace_stability(&mut self, table: &CfdTable, fields: impl IntoIterator<Item = Arc<solver::Field3>>) {
        let stamp = Stamp::of(table);
        if self.stamp.is_some_and(|s| s != stamp) {
            return;
        }
        let mut kept: Vec<Arc<solver::Field3>> = fields.into_iter().filter(|f| at_an_angle(f)).collect();
        kept.extend(self.fields.iter().filter(|f| !at_an_angle(f)).cloned());
        Self::order(&mut kept);
        self.stamp = (!kept.is_empty()).then_some(stamp);
        self.fields = kept;
    }

    /// Ascending Mach number, then angle; the first of two fields of one point stays.
    fn order(fields: &mut Vec<Arc<solver::Field3>>) {
        fields.sort_by(|a, b| a.mach.total_cmp(&b.mach).then(a.alpha_deg.total_cmp(&b.alpha_deg)));
        fields.dedup_by(|a, b| same(a.mach, b.mach) && same_alpha(a.alpha_deg, b.alpha_deg));
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

    /// The field of the point at `mach` and `alpha_deg` of `table`, if the store holds one for that table.
    pub fn get(&self, table: &CfdTable, mach: f64, alpha_deg: f64) -> Option<&Arc<solver::Field3>> {
        (self.stamp == Some(Stamp::of(table))).then(|| self.fields.iter().find(|f| same(f.mach, mach) && same_alpha(f.alpha_deg, alpha_deg))).flatten()
    }

    /// The Mach numbers (ascending) the store has a field at, for `table`.
    pub fn machs(&self, table: &CfdTable) -> Vec<f64> {
        if self.stamp != Some(Stamp::of(table)) {
            return Vec::new();
        }
        let mut machs: Vec<f64> = self.fields.iter().map(|f| f.mach).collect();
        machs.dedup_by(|a, b| same(*a, *b));
        machs
    }

    /// The angles of attack (degrees, ascending) the store has a field at `mach` of `table` for.
    pub fn alphas_at(&self, table: &CfdTable, mach: f64) -> Vec<f64> {
        if self.stamp != Some(Stamp::of(table)) {
            return Vec::new();
        }
        self.fields.iter().filter(|f| same(f.mach, mach)).map(|f| f.alpha_deg).collect()
    }

    /// Whether the store has a field at `mach` of `table`, at any angle of attack.
    pub fn has_mach(&self, table: &CfdTable, mach: f64) -> bool {
        self.stamp == Some(Stamp::of(table)) && self.fields.iter().any(|f| same(f.mach, mach))
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
                .map(|&mach| CfdPoint {
                    mach,
                    cd: 0.5,
                    friction: 0.1,
                    pressure: 0.2,
                    base: 0.1,
                    fins: 0.1,
                    other: 0.0,
                    interference: 0.0,
                    quality: CfdQuality::Converged,
                })
                .collect(),
            grid: CfdGrid::Coarse,
            lift: None,
        }
    }

    fn field(mach: f64) -> Arc<solver::Field3> {
        FieldRef::from_2d(&solver::Field {
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
        .0
    }

    #[test]
    fn the_store_keeps_a_field_per_mach_node_of_its_table_and_nothing_for_another_table() {
        let t = table(7, 100, &[0.5, 1.0, 2.0]);
        let mut store = FieldStore::default();
        assert!(store.is_empty() && !store.has_any(&t));
        // a field of a Mach number the table does not have is left out, the rest come out in Mach order
        store.replace(&t, [field(2.0), field(0.5), field(1.5), field(1.0)]);
        assert_eq!(store.len(), 3);
        assert_eq!(store.get(&t, 1.0, 0.0).map(|f| f.mach), Some(1.0));
        assert_eq!(store.get(&t, 2.0 + 1e-12, 0.0).map(|f| f.mach), Some(2.0), "the same node within rounding");
        assert!(store.get(&t, 1.5, 0.0).is_none());
        // the same geometry solved again is another table: its points have no field here
        let again = table(7, 101, &[0.5, 1.0, 2.0]);
        assert!(store.get(&again, 1.0, 0.0).is_none() && !store.has_any(&again));
        // a solve replaces what was kept
        store.replace(&again, [field(1.0)]);
        assert_eq!((store.len(), store.get(&again, 1.0, 0.0).is_some(), store.get(&t, 1.0, 0.0).is_some()), (1, true, false));
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

    #[test]
    fn several_angles_of_attack_at_one_mach_node_are_kept_apart() {
        let t = table(7, 100, &[0.5, 1.0]);
        let at = |mach: f64, alpha: f64| {
            let mut f = (*field(mach)).clone();
            f.alpha_deg = alpha;
            Arc::new(f)
        };
        let mut store = FieldStore::default();
        // out of order, and the same angle twice (within rounding): ascending, once each
        store.replace(&t, [at(1.0, 4.0), at(0.5, 2.0), at(1.0, 2.0), at(1.0, 2.0 + 1e-9), at(0.5, 4.0)]);
        assert_eq!(store.len(), 4);
        assert_eq!(store.alphas_at(&t, 1.0), vec![2.0, 4.0]);
        assert_eq!(store.get(&t, 1.0, 4.0).map(|f| f.alpha_deg), Some(4.0));
        assert!(store.get(&t, 1.0, 3.0).is_none() && store.get(&t, 1.0, 0.0).is_none());
        assert!(store.has_mach(&t, 0.5) && !store.has_mach(&t, 2.0));
        assert!(store.alphas_at(&t, 2.0).is_empty());
        assert!(store.alphas_at(&table(7, 101, &[0.5, 1.0]), 1.0).is_empty(), "another table has none");
    }

    #[test]
    fn the_fields_of_a_stability_solve_ride_beside_those_of_the_drag_table() {
        let t = table(7, 100, &[0.5, 1.0, 2.0]);
        let at = |mach: f64, alpha: f64| {
            let mut f = (*field(mach)).clone();
            f.alpha_deg = alpha;
            Arc::new(f)
        };
        let mut store = FieldStore::default();
        store.replace(&t, [field(0.5), field(1.0), field(2.0)]);
        // the stability solve has its own Mach nodes (0.8 is none of the table's) and keeps only fields at an angle of attack
        store.replace_stability(&t, [at(0.8, 2.0), at(0.8, 6.0), at(1.0, 6.0), at(1.0, 2.0), field(0.5)]);
        assert_eq!(store.len(), 3 + 4, "the three of the drag solve, four at an angle");
        assert_eq!(store.machs(&t), vec![0.5, 0.8, 1.0, 2.0]);
        assert_eq!(store.alphas_at(&t, 0.8), vec![2.0, 6.0]);
        assert_eq!(store.alphas_at(&t, 1.0), vec![0.0, 2.0, 6.0], "the angle of the drag solve, then those of the stability solve");
        assert_eq!(store.get(&t, 0.8, 6.0).map(|f| f.alpha_deg), Some(6.0));
        // another stability solve replaces the fields at an angle and leaves the drag solve's
        store.replace_stability(&t, [at(2.0, 2.0)]);
        assert_eq!(store.machs(&t), vec![0.5, 1.0, 2.0]);
        assert_eq!(store.alphas_at(&t, 2.0), vec![0.0, 2.0]);
        // a new drag solve of the same geometry keeps the stability fields: the stability table rides on it
        let again = table(7, 101, &[0.5, 1.0, 2.0]);
        store.replace(&again, [field(1.0)]);
        assert_eq!(store.alphas_at(&again, 2.0), vec![2.0]);
        assert_eq!(store.alphas_at(&again, 1.0), vec![0.0]);
        // but not one of another geometry, and a stability solve for a table the store does not hold fields for is ignored
        let other = table(8, 102, &[0.5, 1.0, 2.0]);
        store.replace(&other, [field(1.0)]);
        assert_eq!(store.len(), 1);
        store.replace_stability(&again, [at(2.0, 2.0)]);
        assert_eq!(store.len(), 1);
        // with no fields yet (a table opened from a file) the stability solve's are the first
        let mut empty = FieldStore::default();
        empty.replace_stability(&again, [at(2.0, 2.0)]);
        assert_eq!((empty.len(), empty.has_any(&again)), (1, true));
    }
}
