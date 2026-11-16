//! Normal force and centre of pressure of a 3D Euler stability solve, tabulated against Mach at two angles of attack, that the
//! flight and the Analyze screen read instead of the Extended (Barrowman build-up) values.
//!
//! The solve (`bowshock-core`'s `Analysis::run_lift`) gives the normal-force coefficient `CN` and the CP station at a few Mach
//! numbers at 2 and 6 degrees. This module is the rule that turns those few points into `CN` and CP at any Mach number and any
//! angle of attack, shared by the Analyze plots and the flight model so both read the table the same way:
//!
//! * **Mach**: linear between nodes; held above the last node; below the first node the first node's `CN` is scaled by the ratio
//!   of Extended's `CNα` at that Mach to Extended's at the node and the CP is shifted by the difference of Extended's CPs (the
//!   caller supplies Extended, see [`ExtStab`]).
//! * **Angle** ([`LiftAt::at_alpha`]): up to 2 degrees `CN = CNα α` with the CP of 2 degrees; between 2 and 6 degrees `CN` and CP
//!   are interpolated; above 6 degrees `CN` continues with the slope of 2 to 6 degrees and the CP is held at 6 degrees. Negative
//!   angles are mirrored.

use bowshock_model::CfdLiftTable;

/// The two angles of attack the solve is made at (degrees).
pub const ALPHAS_DEG: [f64; 2] = [2.0, 6.0];

/// Extended's stability at one Mach number (at 2 degrees): what the table is scaled by below its first node.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExtStab {
    /// Normal force slope per radian.
    pub cn_alpha: f64,
    /// CP from the nose tip (m).
    pub cp: f64,
}

/// The `(Mach, CN, CP station)` of the solve at one angle, ascending in Mach.
type Series = Vec<(f64, f64, f64)>;

/// A stored 3D stability solve ready for lookup: `CN` and CP against Mach at 2 and 6 degrees.
#[derive(Clone, Debug, PartialEq)]
pub struct LiftTable {
    a2: Series,
    a6: Series,
}

/// The table's `CN` and CP at one Mach number: at 2 degrees and, when the table has them, at 6 degrees.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiftAt {
    a2: (f64, f64),
    a6: Option<(f64, f64)>,
}

impl LiftTable {
    /// The table of a stored solve, `None` without a point at 2 degrees (the plots and the flight need it).
    pub fn from_cfd(t: &CfdLiftTable) -> Option<LiftTable> {
        let series = |alpha_deg: f64| -> Series {
            let mut v: Series = t.points.iter().filter(|p| (p.alpha_deg - alpha_deg).abs() < 1e-6).map(|p| (p.mach, p.cn, p.x_cp)).collect();
            v.sort_by(|a, b| a.0.total_cmp(&b.0));
            v
        };
        let (a2, a6) = (series(ALPHAS_DEG[0]), series(ALPHAS_DEG[1]));
        (!a2.is_empty()).then_some(LiftTable { a2, a6 })
    }

    /// The Mach number of the first node at 2 degrees: the table is scaled by Extended below it.
    pub fn first_mach(&self) -> f64 {
        self.a2[0].0
    }

    /// `CN` and CP at `mach` (`ext`: Extended's stability at 2 degrees at any Mach number, for the scaling below the first node;
    /// it is asked at `mach` and at the first node, at most once for each). `None` when `ext` has no value where it is needed.
    pub fn at(&self, mach: f64, ext: &dyn Fn(f64) -> Option<ExtStab>) -> Option<LiftAt> {
        let memo: std::cell::RefCell<Vec<(f64, Option<ExtStab>)>> = std::cell::RefCell::new(Vec::new());
        let cached = |m: f64| -> Option<ExtStab> {
            if let Some((_, v)) = memo.borrow().iter().find(|(k, _)| *k == m) {
                return *v;
            }
            let v = ext(m);
            memo.borrow_mut().push((m, v));
            v
        };
        let a2 = series_at(&self.a2, mach, &cached)?;
        let a6 = if self.a6.is_empty() { None } else { series_at(&self.a6, mach, &cached) };
        Some(LiftAt { a2, a6 })
    }
}

/// `CN` and CP of one angle's series at `mach`: linear in Mach between nodes, held above the last, and scaled by Extended below
/// the first.
fn series_at(series: &[(f64, f64, f64)], mach: f64, ext: &dyn Fn(f64) -> Option<ExtStab>) -> Option<(f64, f64)> {
    let (first, last) = (*series.first()?, *series.last()?);
    if mach < first.0 - 1e-9 {
        let (m, n) = (ext(mach)?, ext(first.0)?);
        let ratio = m.cn_alpha / n.cn_alpha;
        let cn = if ratio.is_finite() { first.1 * ratio } else { first.1 };
        return Some((cn, first.2 + m.cp - n.cp));
    }
    if mach >= last.0 {
        return Some((last.1, last.2));
    }
    let i = series.partition_point(|p| p.0 <= mach).max(1);
    let (a, b) = (series[i - 1], series[i]);
    let t = (mach - a.0) / (b.0 - a.0);
    Some((a.1 + (b.1 - a.1) * t, a.2 + (b.2 - a.2) * t))
}

impl LiftAt {
    /// `CNα` per radian at small angle (the 2 degree point over 2 degrees).
    pub fn cn_alpha(&self) -> f64 {
        self.a2.0 / ALPHAS_DEG[0].to_radians()
    }

    /// CP from the nose tip at the small angle (m).
    pub fn cp(&self) -> f64 {
        self.a2.1
    }

    /// `CN` and CP at an angle of attack (rad), negative angles mirrored. Up to 2 degrees `CN` is `CNα α` with the CP of 2 degrees;
    /// between 2 and 6 degrees both are interpolated; above 6 degrees `CN` continues with the slope of 2 to 6 degrees and the CP
    /// is held at 6 degrees. Without a 6 degree point `CN` stays linear and the CP is the one of 2 degrees.
    pub fn at_alpha(&self, alpha: f64) -> (f64, f64) {
        let a = alpha.abs().to_degrees();
        let (cn2, cp2) = self.a2;
        let (cn, cp) = match self.a6 {
            None => (cn2 * a / ALPHAS_DEG[0], cp2),
            Some(_) if a <= ALPHAS_DEG[0] => (cn2 * a / ALPHAS_DEG[0], cp2),
            Some((cn6, cp6)) if a <= ALPHAS_DEG[1] => {
                let t = (a - ALPHAS_DEG[0]) / (ALPHAS_DEG[1] - ALPHAS_DEG[0]);
                (cn2 + (cn6 - cn2) * t, cp2 + (cp6 - cp2) * t)
            }
            Some((cn6, cp6)) => (cn6 + (cn6 - cn2) / (ALPHAS_DEG[1] - ALPHAS_DEG[0]) * (a - ALPHAS_DEG[1]), cp6),
        };
        (cn.copysign(alpha), cp)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_model::{CfdLiftPoint, CfdQuality};

    /// CN(2°) = 0.1 and 0.2 at M 0.8 and 2.0; CN(6°) = 0.4 and 0.8; CP 1.0 / 1.4 m at 2°, 1.1 / 1.5 m at 6°.
    fn stored() -> CfdLiftTable {
        let p = |mach: f64, alpha_deg: f64, cn: f64, x_cp: f64| CfdLiftPoint { mach, alpha_deg, cn, x_cp, quality: CfdQuality::Converged };
        CfdLiftTable {
            key: 1,
            solved_at: 1,
            solve_seconds: 60.0,
            backend: "GPU (Luma)".into(),
            points: vec![p(2.0, 6.0, 0.8, 1.5), p(0.8, 2.0, 0.1, 1.0), p(0.8, 6.0, 0.4, 1.1), p(2.0, 2.0, 0.2, 1.4)],
            notes: Vec::new(),
        }
    }

    fn table() -> LiftTable {
        LiftTable::from_cfd(&stored()).unwrap()
    }

    fn no_ext(_: f64) -> Option<ExtStab> {
        None
    }

    #[test]
    fn a_table_needs_a_point_at_2_degrees() {
        let mut only_6 = stored();
        only_6.points.retain(|p| p.alpha_deg > 3.0);
        assert!(LiftTable::from_cfd(&only_6).is_none());
        assert!(LiftTable::from_cfd(&CfdLiftTable { points: Vec::new(), ..stored() }).is_none());
        assert_eq!(table().first_mach(), 0.8, "points are sorted by Mach whatever their order");
    }

    #[test]
    fn mach_is_interpolated_between_nodes_and_held_above_the_last() {
        let two = 2f64.to_radians();
        let at = table().at(1.4, &no_ext).unwrap(); // halfway between 0.8 and 2.0
        assert!((at.cn_alpha() - 0.15 / two).abs() < 1e-12);
        assert!((at.cp() - 1.2).abs() < 1e-12);
        let top = table().at(2.7, &no_ext).unwrap();
        assert!((top.cn_alpha() - 0.2 / two).abs() < 1e-12 && (top.cp() - 1.4).abs() < 1e-12);
    }

    #[test]
    fn below_the_first_node_the_table_is_scaled_by_extended() {
        // Extended: CNα 10 at M 0.8 and 8 at M 0.5; CP 0.9 and 0.7 m
        let ext = |m: f64| Some(if m < 0.6 { ExtStab { cn_alpha: 8.0, cp: 0.7 } } else { ExtStab { cn_alpha: 10.0, cp: 0.9 } });
        let at = table().at(0.5, &ext).unwrap();
        assert!((at.a2.0 - 0.1 * 0.8).abs() < 1e-12, "{at:?}");
        assert!((at.a2.1 - (1.0 - 0.2)).abs() < 1e-12);
        let (cn6, cp6) = at.a6.unwrap();
        assert!((cn6 - 0.4 * 0.8).abs() < 1e-12 && (cp6 - (1.1 - 0.2)).abs() < 1e-12);
        assert!(table().at(0.5, &no_ext).is_none(), "no Extended, no scaling");
        assert!(table().at(0.8, &no_ext).is_some(), "at a node Extended is not asked");
    }

    #[test]
    fn extended_is_asked_once_per_mach_below_the_first_node() {
        let asked = std::cell::Cell::new(0);
        let ext = |_: f64| {
            asked.set(asked.get() + 1);
            Some(ExtStab { cn_alpha: 9.0, cp: 0.8 })
        };
        table().at(0.3, &ext).unwrap();
        assert_eq!(asked.get(), 2, "at the Mach number and at the first node, for both angles together");
    }

    #[test]
    fn the_angle_rules() {
        let at = table().at(0.8, &no_ext).unwrap(); // CN 0.1 @ 2°, 0.4 @ 6°; CP 1.0, 1.1
        let deg = f64::to_radians;
        let (cn, cp) = at.at_alpha(deg(1.0));
        assert!((cn - 0.05).abs() < 1e-12 && (cp - 1.0).abs() < 1e-12, "linear to the origin, CP of 2°");
        let (cn, cp) = at.at_alpha(deg(4.0));
        assert!((cn - 0.25).abs() < 1e-12 && (cp - 1.05).abs() < 1e-12, "between the angles");
        let (cn, cp) = at.at_alpha(deg(10.0));
        assert!((cn - (0.4 + 0.075 * 4.0)).abs() < 1e-12 && (cp - 1.1).abs() < 1e-12, "slope of 2°..6° beyond, CP held");
        let (cn, cp) = at.at_alpha(deg(-4.0));
        assert!((cn + 0.25).abs() < 1e-12 && (cp - 1.05).abs() < 1e-12, "negative angles mirror");
        assert_eq!(at.at_alpha(0.0).0, 0.0);
    }

    #[test]
    fn without_a_6_degree_point_cn_stays_linear() {
        let mut t = stored();
        t.points.retain(|p| p.alpha_deg < 3.0);
        let at = LiftTable::from_cfd(&t).unwrap().at(0.8, &no_ext).unwrap();
        let (cn, cp) = at.at_alpha(5f64.to_radians());
        assert!((cn - 0.25).abs() < 1e-12 && (cp - 1.0).abs() < 1e-12);
    }
}
