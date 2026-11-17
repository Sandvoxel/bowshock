use super::plan::LIFT_ALPHAS;
use bowshock_model::{CfdLiftTable, CfdPoint, CfdQuality, CfdTable};
use bowshock_sim::{AlphaPoint, DragBreakdown, MachPoint};

/// The table's point at `mach`, linearly interpolated and held at the ends; its quality is the worse of its neighbours'.
pub fn at(table: &CfdTable, mach: f64) -> CfdPoint {
    let pts = &table.points;
    let (first, last) = (pts[0], pts[pts.len() - 1]);
    if mach <= first.mach {
        return CfdPoint { mach, ..first };
    }
    if mach >= last.mach {
        return CfdPoint { mach, ..last };
    }
    let i = pts.partition_point(|p| p.mach <= mach);
    let (a, b) = (pts[i - 1], pts[i]);
    let t = (mach - a.mach) / (b.mach - a.mach);
    let l = |x: f64, y: f64| x + (y - x) * t;
    let worse = |x: CfdQuality, y: CfdQuality| match (x, y) {
        (CfdQuality::Stalled, _) | (_, CfdQuality::Stalled) => CfdQuality::Stalled,
        (CfdQuality::Stationary, _) | (_, CfdQuality::Stationary) => CfdQuality::Stationary,
        _ => CfdQuality::Converged,
    };
    CfdPoint {
        mach,
        cd: l(a.cd, b.cd),
        friction: l(a.friction, b.friction),
        pressure: l(a.pressure, b.pressure),
        base: l(a.base, b.base),
        fins: l(a.fins, b.fins),
        other: l(a.other, b.other),
        interference: l(a.interference, b.interference),
        quality: worse(a.quality, b.quality),
    }
}

/// The table's drag at one condition: the solved zero-lift drag plus the build-up method's change of drag from `zero` (its
/// drag at zero angle) to `now` (at the set angle), part by part. Without `zero` (zero angle) the table's drag stands.
pub(super) fn blend(c: &CfdPoint, now: &DragBreakdown, zero: Option<&DragBreakdown>) -> DragBreakdown {
    let change = |n: f64, z: fn(&DragBreakdown) -> f64| zero.map_or(0.0, |zero| n - z(zero));
    let friction = c.friction + change(now.friction, |d| d.friction);
    let pressure = c.pressure + change(now.pressure, |d| d.pressure);
    let base = c.base + change(now.base, |d| d.base);
    let fins = c.fins + change(now.fins, |d| d.fins);
    let protuberances = c.other + change(now.protuberances, |d| d.protuberances);
    DragBreakdown { friction, pressure, base, fins, protuberances, total: friction + pressure + base + fins + protuberances }
}

/// The drag parts at `mach`, linearly interpolated over a sweep of Mach points and held at its ends; `None` for an empty sweep.
pub(super) fn drag_at(sweep: &[MachPoint], mach: f64) -> Option<DragBreakdown> {
    let i = sweep.partition_point(|p| p.mach <= mach);
    let a = sweep.get(i.saturating_sub(1))?;
    let b = match sweep.get(i) {
        Some(b) if i > 0 && b.mach > a.mach => b,
        _ => return Some(a.drag),
    };
    let t = (mach - a.mach) / (b.mach - a.mach);
    let l = |x: f64, y: f64| x + (y - x) * t;
    let (x, y) = (&a.drag, &b.drag);
    Some(DragBreakdown {
        friction: l(x.friction, y.friction),
        pressure: l(x.pressure, y.pressure),
        base: l(x.base, y.base),
        fins: l(x.fins, y.fins),
        protuberances: l(x.protuberances, y.protuberances),
        total: l(x.total, y.total),
    })
}

/// The first node's point `first` carried down to `mach` by Extended's zero-angle trend, part by part: each part is scaled by
/// its Extended drag at `mach` (`ext0_m`) over that at the first node (`ext0_lo`). A part Extended has no drag for there is held.
pub(super) fn below(first: &CfdPoint, ext0_m: &DragBreakdown, ext0_lo: &DragBreakdown, mach: f64) -> CfdPoint {
    let scale = |part: f64, m: f64, lo: f64| {
        let s = part * m / lo;
        if lo.abs() < 1e-12 || !s.is_finite() {
            part
        } else {
            s
        }
    };
    let friction = scale(first.friction, ext0_m.friction, ext0_lo.friction);
    let pressure = scale(first.pressure, ext0_m.pressure, ext0_lo.pressure);
    let base = scale(first.base, ext0_m.base, ext0_lo.base);
    let fins = scale(first.fins, ext0_m.fins, ext0_lo.fins);
    let other = scale(first.other, ext0_m.protuberances, ext0_lo.protuberances);
    CfdPoint { mach, cd: friction + pressure + base + fins + other, friction, pressure, base, fins, other, interference: 0.0, quality: first.quality }
}

/// The Mach sweep with the table's zero-lift drag: stability (margin, CP, `CNα`) from `extended` (the Extended method at the set
/// angle), drag from the table plus Extended's change of drag from zero angle to that angle (`zero`: the same sweep at zero
/// angle, point for point; `None` at zero angle). Below the table's first node the drag is the first node's scaled part by part
/// by Extended's zero-angle Mach trend (see [`below`]); above the last node nothing is kept.
pub fn mach_points(table: &CfdTable, extended: Vec<MachPoint>, zero: Option<&[MachPoint]>) -> Vec<MachPoint> {
    let (first, hi) = (table.points[0], table.points[table.points.len() - 1].mach);
    let ext0 = zero.unwrap_or(&extended);
    let ext0_lo = drag_at(ext0, first.mach);
    let table_drag: Vec<CfdPoint> = ext0
        .iter()
        .zip(&extended)
        .map(|(z, p)| match &ext0_lo {
            Some(l) if p.mach < first.mach - 1e-9 => below(&first, &z.drag, l, p.mach),
            _ => at(table, p.mach),
        })
        .collect();
    extended
        .into_iter()
        .zip(table_drag)
        .enumerate()
        .filter(|(_, (p, _))| p.mach <= hi + 1e-9)
        .map(|(i, (mut p, c))| {
            p.drag = blend(&c, &p.drag, zero.and_then(|z| z.get(i)).map(|z| &z.drag));
            p
        })
        .collect()
}

/// The angle sweep at `mach` with the table's zero-lift drag there (held at the nearest end outside the table) plus Extended's
/// change of drag with the angle (`points[0]` is zero angle).
pub fn alpha_points(table: &CfdTable, mach: f64, points: Vec<AlphaPoint>) -> Vec<AlphaPoint> {
    let c = at(table, mach);
    let zero = points.first().map(|p| p.drag);
    points
        .into_iter()
        .map(|mut p| {
            p.drag = blend(&c, &p.drag, zero.as_ref());
            p
        })
        .collect()
}

/// Extended's stability at one Mach number (at 2°): what the 3D solve is scaled by below its first node.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExtStab {
    /// Normal force slope per radian.
    pub cn_alpha: f64,
    /// CP from the nose tip (m).
    pub cp: f64,
}

/// The 3D solve's CN and CP at one Mach number: at 2° and, when solved, at 6°.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiftAt {
    pub(super) a2: (f64, f64),
    pub(super) a6: Option<(f64, f64)>,
}

/// Whether the solve has what the plots need (a point at 2°).
pub fn lift_usable(lift: &CfdLiftTable) -> bool {
    lift.points.iter().any(|p| (p.alpha_deg - LIFT_ALPHAS[0]).abs() < 1e-6)
}

/// The (Mach, CN, CP) of the solve at one angle, ascending in Mach.
pub(super) fn lift_series(lift: &CfdLiftTable, alpha_deg: f64) -> Vec<(f64, f64, f64)> {
    let mut v: Vec<_> = lift.points.iter().filter(|p| (p.alpha_deg - alpha_deg).abs() < 1e-6).map(|p| (p.mach, p.cn, p.x_cp)).collect();
    v.sort_by(|a, b| a.0.total_cmp(&b.0));
    v
}

/// CN and CP of one angle's series at `mach`: linear in Mach between nodes, held above the last, and below the first the first
/// node's CN scaled by Extended's `CNα` ratio and its CP shifted by Extended's CP difference.
pub(super) fn lift_series_at(series: &[(f64, f64, f64)], mach: f64, ext: &dyn Fn(f64) -> Option<ExtStab>) -> Option<(f64, f64)> {
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

/// The 3D solve at `mach` (`ext`: Extended's stability at 2° at any Mach, for the scaling below the first node). `None` without
/// a point at 2°.
pub fn lift_at(lift: &CfdLiftTable, mach: f64, ext: &dyn Fn(f64) -> Option<ExtStab>) -> Option<LiftAt> {
    let a2 = lift_series_at(&lift_series(lift, LIFT_ALPHAS[0]), mach, ext)?;
    let a6 = lift_series_at(&lift_series(lift, LIFT_ALPHAS[1]), mach, ext);
    Some(LiftAt { a2, a6 })
}

impl LiftAt {
    /// `CNα` per radian at small angle (the 2° point over 2°).
    pub fn cn_alpha(&self) -> f64 {
        self.a2.0 / LIFT_ALPHAS[0].to_radians()
    }

    /// CP from the nose tip at the small angle (m).
    pub fn cp(&self) -> f64 {
        self.a2.1
    }

    /// CN and CP at an angle of attack (rad), negative angles mirrored. Up to 2° CN is `CNα·α` with CP from 2°; between 2° and 6°
    /// both are interpolated; above 6° CN continues with the slope of 2° to 6° and CP is held at 6°.
    pub fn at_alpha(&self, alpha: f64) -> (f64, f64) {
        let a = alpha.abs().to_degrees();
        let (cn2, cp2) = self.a2;
        let (cn, cp) = match self.a6 {
            None => (cn2 * a / LIFT_ALPHAS[0], cp2),
            Some(_) if a <= LIFT_ALPHAS[0] => (cn2 * a / LIFT_ALPHAS[0], cp2),
            Some((cn6, cp6)) if a <= LIFT_ALPHAS[1] => {
                let t = (a - LIFT_ALPHAS[0]) / (LIFT_ALPHAS[1] - LIFT_ALPHAS[0]);
                (cn2 + (cn6 - cn2) * t, cp2 + (cp6 - cp2) * t)
            }
            Some((cn6, cp6)) => (cn6 + (cn6 - cn2) / (LIFT_ALPHAS[1] - LIFT_ALPHAS[0]) * (a - LIFT_ALPHAS[1]), cp6),
        };
        (cn.copysign(alpha), cp)
    }
}

/// The Mach sweep with the 3D solve's stability: `CNα` and CP from the solve at 2° (see [`lift_at`]), the margin from that CP
/// and `cg` over the reference diameter `d_ref`. Drag and everything else stay as they are.
pub fn lift_mach_points(lift: &CfdLiftTable, mut points: Vec<MachPoint>, cg: f64, d_ref: f64, ext: &dyn Fn(f64) -> Option<ExtStab>) -> Vec<MachPoint> {
    for p in &mut points {
        if let Some(at) = lift_at(lift, p.mach, ext) {
            p.cn_alpha = at.cn_alpha();
            p.cp = at.cp();
            p.margin_cal = (p.cp - cg) / d_ref;
        }
    }
    points
}

/// The angle sweep at `mach` with the 3D solve's CN and CP (see [`LiftAt::at_alpha`]); `cn_alpha` is `CN/α` (the small-angle
/// slope at zero angle), the margin comes from the CP and `cg`. Drag stays as it is.
pub fn lift_alpha_points(
    lift: &CfdLiftTable,
    mach: f64,
    mut points: Vec<AlphaPoint>,
    cg: f64,
    d_ref: f64,
    ext: &dyn Fn(f64) -> Option<ExtStab>,
) -> Vec<AlphaPoint> {
    let Some(at) = lift_at(lift, mach, ext) else { return points };
    for p in &mut points {
        let (cn, cp) = at.at_alpha(p.alpha);
        p.cn = cn;
        p.cp = cp;
        p.cn_alpha = if p.alpha.abs() < 1e-9 { at.cn_alpha() } else { cn / p.alpha };
        p.margin_cal = (cp - cg) / d_ref;
    }
    points
}
