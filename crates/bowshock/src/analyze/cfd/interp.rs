use bowshock_model::{CfdLiftTable, CfdPoint, CfdQuality, CfdTable};
use bowshock_sim::{AlphaPoint, DragBreakdown, ExtStab, LiftTable, MachPoint};

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
    CfdPoint { mach, cd: friction + pressure + base + fins + other, friction, pressure, base, fins, other, quality: first.quality }
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

/// Whether the solve has what the plots need (a point at 2°).
pub fn lift_usable(lift: &CfdLiftTable) -> bool {
    LiftTable::from_cfd(lift).is_some()
}

/// The Mach sweep with the 3D solve's stability: `CNα` and CP from the solve at 2° (see [`LiftTable::at`]), the margin from that CP
/// and `cg` over the reference diameter `d_ref`. Drag and everything else stay as they are.
pub fn lift_mach_points(lift: &CfdLiftTable, mut points: Vec<MachPoint>, cg: f64, d_ref: f64, ext: &dyn Fn(f64) -> Option<ExtStab>) -> Vec<MachPoint> {
    let Some(table) = LiftTable::from_cfd(lift) else { return points };
    for p in &mut points {
        if let Some(at) = table.at(p.mach, ext) {
            p.cn_alpha = at.cn_alpha();
            p.cp = at.cp();
            p.margin_cal = (p.cp - cg) / d_ref;
        }
    }
    points
}

/// The angle sweep at `mach` with the 3D solve's CN and CP (see [`LiftAt::at_alpha`](bowshock_sim::LiftAt::at_alpha)); `cn_alpha` is
/// `CN/α` (the small-angle slope at zero angle), the margin comes from the CP and `cg`. Drag stays as it is.
pub fn lift_alpha_points(
    lift: &CfdLiftTable,
    mach: f64,
    mut points: Vec<AlphaPoint>,
    cg: f64,
    d_ref: f64,
    ext: &dyn Fn(f64) -> Option<ExtStab>,
) -> Vec<AlphaPoint> {
    let Some(at) = LiftTable::from_cfd(lift).and_then(|t| t.at(mach, ext)) else { return points };
    for p in &mut points {
        let (cn, cp) = at.at_alpha(p.alpha);
        p.cn = cn;
        p.cp = cp;
        p.cn_alpha = if p.alpha.abs() < 1e-9 { at.cn_alpha() } else { cn / p.alpha };
        p.margin_cal = (cp - cg) / d_ref;
    }
    points
}
