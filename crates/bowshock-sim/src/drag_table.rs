//! Zero-lift drag parts tabulated against Mach, solved by CFD, that the flight can fly instead of the build-up.

use bowshock_model::CfdTable;

/// One Mach node of a [`DragTable`]: drag-coefficient parts referenced to the vehicle reference area.
#[derive(Clone, Debug, PartialEq)]
pub struct DragTablePoint {
    /// Mach number.
    pub mach: f64,
    /// Skin-friction drag.
    pub friction: f64,
    /// Pressure drag of the bodies.
    pub pressure: f64,
    /// Base drag.
    pub base: f64,
    /// Fin drag.
    pub fins: f64,
    /// Everything else (protuberances).
    pub other: f64,
}

/// What the motor jet does to the drag parts at one Mach node: each part solved with the jet on over the same part solved with it off (the fins' drag is not scaled).
#[derive(Clone, Debug, PartialEq)]
pub struct PowerRatio {
    /// Mach number of the node.
    pub mach: f64,
    /// Skin-friction drag, power on over power off.
    pub friction: f64,
    /// Pressure drag of the bodies.
    pub pressure: f64,
    /// Base drag (the annulus around the nozzle exit with the jet on over the whole base without it).
    pub base: f64,
}

/// Zero-lift drag parts per Mach (ascending, at least one point) solved at the ISA `altitude` (m), which sets their Reynolds number.
#[derive(Clone, Debug, PartialEq)]
pub struct DragTable {
    /// Nodes, ascending Mach.
    pub points: Vec<DragTablePoint>,
    /// ISA altitude the table was solved at (m).
    pub altitude: f64,
    /// The effect of the motor jet at the Mach numbers solved with it on (ascending; empty: not solved, the flight falls back to the build-up's exit-area correction).
    pub power_on: Vec<PowerRatio>,
}

impl DragTable {
    /// The drag parts of a stored CFD table. Its power-on part is taken only when `jet` (the jet the stack's motor gives now) is the one it was solved with: a table solved
    /// for another motor flies power-off only.
    pub fn from_cfd(t: &CfdTable, jet: Option<&bowshock_model::CfdJet>) -> DragTable {
        let mut table = DragTable {
            power_on: Vec::new(),
            points: t
                .points
                .iter()
                .map(|p| DragTablePoint { mach: p.mach, friction: p.friction, pressure: p.pressure, base: p.base, fins: p.fins, other: p.other })
                .collect(),
            altitude: t.altitude,
        };
        if let Some(on) = t.power_on.as_ref().filter(|on| jet.is_some_and(|j| j.same_as(&on.jet))) {
            table.power_on = on.points.iter().map(|p| PowerRatio { mach: p.mach, friction: p.friction, pressure: p.pressure, base: p.base }).collect();
        }
        table
    }

    /// The jet's effect on the drag parts at `mach`: linear between the nodes solved with it on, the end node's beyond them; `None` without a power-on solve.
    pub(crate) fn power_ratio(&self, mach: f64) -> Option<PowerRatio> {
        let (first, last) = (self.power_on.first()?, self.power_on.last()?);
        if mach <= first.mach {
            return Some(PowerRatio { mach, ..first.clone() });
        }
        if mach >= last.mach {
            return Some(PowerRatio { mach, ..last.clone() });
        }
        let i = self.power_on.partition_point(|p| p.mach <= mach).clamp(1, self.power_on.len() - 1);
        let (a, b) = (&self.power_on[i - 1], &self.power_on[i]);
        let t = ((mach - a.mach) / (b.mach - a.mach)).clamp(0.0, 1.0);
        let l = |x: f64, y: f64| x + (y - x) * t;
        Some(PowerRatio { mach, friction: l(a.friction, b.friction), pressure: l(a.pressure, b.pressure), base: l(a.base, b.base) })
    }

    /// Parts linearly interpolated at `mach`, or `None` outside the nodes.
    pub(crate) fn inside(&self, mach: f64) -> Option<DragTablePoint> {
        let (first, last) = (self.points.first()?, self.points.last()?);
        if mach < first.mach || mach > last.mach {
            return None;
        }
        let i = self.points.partition_point(|p| p.mach <= mach).clamp(1, self.points.len().max(2) - 1);
        let (Some(a), Some(b)) = (self.points.get(i - 1), self.points.get(i)) else { return Some(first.clone()) };
        let span = b.mach - a.mach;
        let t = if span > 0.0 { ((mach - a.mach) / span).clamp(0.0, 1.0) } else { 0.0 };
        let l = |x: f64, y: f64| x + (y - x) * t;
        Some(DragTablePoint {
            mach,
            friction: l(a.friction, b.friction),
            pressure: l(a.pressure, b.pressure),
            base: l(a.base, b.base),
            fins: l(a.fins, b.fins),
            other: l(a.other, b.other),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{CfdGrid, CfdJet, CfdPoint, CfdPowerOn, CfdPowerPoint, CfdQuality};

    fn jet(d: f64) -> CfdJet {
        CfdJet { exit_diameter: d, exit_mach: 2.5, exit_pressure_pa: 1.3e5, total_temperature_k: 3000.0 }
    }

    fn table() -> CfdTable {
        let point = |mach: f64| CfdPoint {
            mach,
            cd: 0.5,
            friction: 0.1,
            pressure: 0.1,
            base: 0.2,
            fins: 0.1,
            other: 0.0,
            interference: 0.0,
            quality: CfdQuality::Converged,
            restart: None,
        };
        let on = |mach: f64, base: f64| CfdPowerPoint { mach, friction: 1.0, pressure: 1.1, base, quality: CfdQuality::Converged };
        CfdTable {
            key: 1,
            geometry: 1,
            stage: 1,
            solved_at: 0,
            solve_seconds: 1.0,
            backend: "test".into(),
            fallback: None,
            altitude: 0.0,
            notes: Vec::new(),
            points: vec![point(0.1), point(0.5), point(0.9), point(3.0)],
            grid: CfdGrid::Coarse,
            lift: None,
            power_on: Some(CfdPowerOn { jet: jet(0.03), solved_at: 0, solve_seconds: 1.0, points: vec![on(0.5, 0.5), on(0.9, 0.75)], notes: Vec::new() }),
        }
    }

    #[test]
    fn the_power_on_part_is_taken_when_the_jet_is_the_motors() {
        let t = DragTable::from_cfd(&table(), Some(&jet(0.03)));
        assert_eq!(t.power_on.len(), 2);
        assert!((t.power_on[0].base - 0.5).abs() < 1e-12 && (t.power_on[1].base - 0.75).abs() < 1e-12);
        assert!((t.power_on[0].pressure - 1.1).abs() < 1e-12 && (t.power_on[0].friction - 1.0).abs() < 1e-12);
        // linear between the nodes, held outside them
        let r = t.power_ratio(0.7).unwrap();
        assert!((r.base - 0.625).abs() < 1e-12);
        assert!((t.power_ratio(0.2).unwrap().base - 0.5).abs() < 1e-12);
        assert!((t.power_ratio(2.0).unwrap().base - 0.75).abs() < 1e-12);
    }

    #[test]
    fn a_table_solved_for_another_motor_or_for_none_flies_power_off_only() {
        assert!(DragTable::from_cfd(&table(), Some(&jet(0.04))).power_on.is_empty());
        assert!(DragTable::from_cfd(&table(), None).power_on.is_empty());
        assert!(DragTable::from_cfd(&table(), None).power_ratio(0.5).is_none());
    }
}
