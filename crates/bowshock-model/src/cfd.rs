//! Solved CFD drag tables: the results of the Analyze screen's `CFD table` method, kept with the document so a reopened file
//! shows them again (with how long ago they were solved) instead of re-solving for minutes.
//!
//! A table is a *cache*, not part of the design: it takes no part in the rocket's equality, so storing one is not an edit (no
//! undo step, no revision bump). Whether a stored table still describes the rocket is the app's question; it compares
//! [`CfdTable::geometry`] and [`CfdTable::key`] with the hashes of the rocket as it is now. The configurations of a rocket
//! share the airframe, so a table belongs to the stack that flies ([`CfdTable::stage`]), not to a configuration.

use serde::{Deserialize, Serialize};

use crate::rocket::Rocket;

/// Most tables kept (the oldest are dropped when a new one would exceed it).
pub const MAX_CFD_TABLES: usize = 8;

/// How far a solved point can be trusted (the solver's own verdict, see `bowshock_core::Quality`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CfdQuality {
    /// The residual dropped by the requested number of orders.
    Converged,
    /// The residual did not drop enough, but the drag is stationary.
    Stationary,
    /// The iteration cap was reached without a stop criterion.
    Stalled,
}

/// The zero-lift drag of one solved Mach number, coefficients on the vehicle's reference area. The parts add up to `cd`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CfdPoint {
    /// Mach number.
    pub mach: f64,
    /// Total drag coefficient.
    pub cd: f64,
    /// Skin friction of the body.
    pub friction: f64,
    /// Pressure drag of the nose and the aft body.
    pub pressure: f64,
    /// Base drag.
    pub base: f64,
    /// Fin drag, skin friction and edge pressure drag together.
    pub fins: f64,
    /// Drag of what the solver cannot model (lugs, rail buttons, fin sets beyond the solved one), from the build-up method.
    pub other: f64,
    /// Trust in the point.
    pub quality: CfdQuality,
}

/// A solved table of one flying stack: a drag coefficient per Mach number at zero angle of attack.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CfdTable {
    /// Hash of everything the solve depended on: the geometry and the conditions.
    pub key: u64,
    /// Hash of the geometry alone: when it differs from the rocket's now, the table is stale however the conditions are set.
    pub geometry: u64,
    /// The stack the table is for: the first model stage that flies (1 is the whole rocket).
    pub stage: u32,
    /// When it finished: seconds since the Unix epoch.
    pub solved_at: u64,
    /// Wall time of the solve (s).
    pub solve_seconds: f64,
    /// `GPU (Luma)` or `CPU`.
    pub backend: String,
    /// Why the GPU was not used, when it was not.
    #[serde(default)]
    pub fallback: Option<String>,
    /// Altitude above sea level that set the Reynolds number (m).
    pub altitude: f64,
    /// What was not solved as the document has it, in the app's voice (equivalent fins, lugs from the build-up, ...).
    #[serde(default)]
    pub notes: Vec<String>,
    /// The solved Mach numbers, ascending.
    pub points: Vec<CfdPoint>,
}

impl CfdTable {
    /// Whether the table can be used: at least one point, ascending Mach numbers, every number finite.
    fn is_sound(&self) -> bool {
        let finite = |p: &CfdPoint| [p.mach, p.cd, p.friction, p.pressure, p.base, p.fins, p.other].iter().all(|v| v.is_finite());
        !self.points.is_empty() && self.points.iter().all(finite) && self.points.windows(2).all(|w| w[0].mach < w[1].mach) && self.solve_seconds.is_finite()
    }
}

/// The tables of a document. Equal to every other store: a cache is not part of the rocket's identity (see the module docs).
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct CfdStore(Vec<CfdTable>);

impl PartialEq for CfdStore {
    fn eq(&self, _: &CfdStore) -> bool {
        true
    }
}

impl CfdStore {
    pub(crate) fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

impl Rocket {
    /// The stored CFD tables, oldest first.
    pub fn cfd_tables(&self) -> &[CfdTable] {
        &self.cfd.0
    }

    /// Stores `table`, replacing the one of the same stack (a stack keeps its latest table), and keeps at most
    /// [`MAX_CFD_TABLES`]. A table that is not sound (no points, unordered or non-finite numbers) is refused: `false`.
    pub fn store_cfd_table(&mut self, table: CfdTable) -> bool {
        if !table.is_sound() {
            return false;
        }
        self.cfd.0.retain(|t| t.stage != table.stage);
        self.cfd.0.push(table);
        let excess = self.cfd.0.len().saturating_sub(MAX_CFD_TABLES);
        self.cfd.0.drain(..excess);
        true
    }

    /// The stored table of the stack whose first flying stage is `stage` (1 is the whole rocket), if any.
    pub fn cfd_table(&self, stage: u32) -> Option<&CfdTable> {
        self.cfd.0.iter().find(|t| t.stage == stage)
    }

    /// Takes over the tables of `other`: undo and redo swap whole rockets, and a table solved meanwhile must survive that.
    pub fn carry_cfd_from(&mut self, other: &Rocket) {
        self.cfd = other.cfd.clone();
    }

    /// Drops tables that cannot be used (loading a document).
    pub(crate) fn repair_cfd(&mut self) {
        self.cfd.0.retain(CfdTable::is_sound);
        let excess = self.cfd.0.len().saturating_sub(MAX_CFD_TABLES);
        self.cfd.0.drain(..excess);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(mach: f64) -> CfdPoint {
        CfdPoint { mach, cd: 0.4, friction: 0.1, pressure: 0.1, base: 0.1, fins: 0.1, other: 0.0, quality: CfdQuality::Converged }
    }

    fn table(stage: u32, key: u64) -> CfdTable {
        CfdTable {
            key,
            geometry: 7,
            stage,
            solved_at: 1_700_000_000,
            solve_seconds: 12.5,
            backend: "GPU (Luma)".into(),
            fallback: None,
            altitude: 120.0,
            notes: vec!["note".into()],
            points: vec![point(0.3), point(0.9), point(2.0)],
        }
    }

    #[test]
    fn a_stack_keeps_its_latest_table_and_unsound_tables_are_refused() {
        let mut r = Rocket::default();
        assert!(r.store_cfd_table(table(1, 1)));
        assert!(r.store_cfd_table(table(1, 2)));
        assert!(r.store_cfd_table(table(2, 3)));
        assert_eq!(r.cfd_tables().len(), 2);
        assert_eq!(r.cfd_table(1).unwrap().key, 2);
        let mut bad = table(1, 4);
        bad.points[1].mach = 0.2; // not ascending
        assert!(!r.store_cfd_table(bad));
        let mut nan = table(1, 5);
        nan.points[0].cd = f64::NAN;
        assert!(!r.store_cfd_table(nan));
        assert!(!r.store_cfd_table(CfdTable { points: Vec::new(), ..table(1, 6) }));
        assert_eq!(r.cfd_table(1).unwrap().key, 2);
    }

    #[test]
    fn tables_are_not_part_of_the_rockets_identity_and_survive_the_file() {
        let mut r = Rocket::default();
        let plain = r.clone();
        r.store_cfd_table(table(1, 9));
        assert_eq!(r, plain, "storing a table is not an edit");
        let back = Rocket::from_bytes(&r.to_bytes()).unwrap();
        assert_eq!(back.cfd_table(1), r.cfd_table(1));
        // Nothing is written while there is nothing to write, so old files and readers are untouched.
        assert!(!crate::format::cbor_text(&plain).contains("cfd"));
    }

    #[test]
    fn undo_can_carry_the_tables_over() {
        let mut now = Rocket::default();
        now.store_cfd_table(table(1, 5));
        let mut earlier = Rocket::default();
        assert!(earlier.cfd_tables().is_empty());
        earlier.carry_cfd_from(&now);
        assert_eq!(earlier.cfd_table(1).unwrap().key, 5);
    }

    #[test]
    fn at_most_eight_tables_are_kept() {
        let mut r = Rocket::default();
        for stage in 1..=12 {
            r.store_cfd_table(table(stage, u64::from(stage)));
        }
        assert_eq!(r.cfd_tables().len(), MAX_CFD_TABLES);
        assert_eq!(r.cfd_tables().last().unwrap().stage, 12);
    }
}
