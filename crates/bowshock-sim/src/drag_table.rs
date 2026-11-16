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

/// Zero-lift drag parts per Mach (ascending, at least one point) solved at the ISA `altitude` (m), which sets their Reynolds number.
#[derive(Clone, Debug, PartialEq)]
pub struct DragTable {
    /// Nodes, ascending Mach.
    pub points: Vec<DragTablePoint>,
    /// ISA altitude the table was solved at (m).
    pub altitude: f64,
}

impl DragTable {
    /// The drag parts of a stored CFD table.
    pub fn from_cfd(t: &CfdTable) -> DragTable {
        DragTable {
            points: t
                .points
                .iter()
                .map(|p| DragTablePoint { mach: p.mach, friction: p.friction, pressure: p.pressure, base: p.base, fins: p.fins, other: p.other })
                .collect(),
            altitude: t.altitude,
        }
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
