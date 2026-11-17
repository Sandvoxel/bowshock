use crate::vehicle::NoseShape;
// ---------------------------------------------------------------------------------------------
// Tabulated data: zero-lift pressure drag of fineness-3 noses, NASA TR R-100 (Stoney 1961), page 16.
// ---------------------------------------------------------------------------------------------

type Pts = &'static [(f64, f64)];

const ELLIPSOID: Pts = &[(1.2, 0.110), (1.25, 0.128), (1.3, 0.140), (1.4, 0.148), (1.6, 0.152), (2.0, 0.159), (2.4, 0.162)];
const VON_KARMAN: Pts =
    &[(0.9, 0.0), (0.95, 0.010), (1.0, 0.027), (1.05, 0.055), (1.1, 0.070), (1.2, 0.081), (1.4, 0.095), (1.6, 0.097), (2.0, 0.091), (3.0, 0.083)];
const LV_HAACK: Pts = &[(0.9, 0.0), (0.95, 0.010), (1.0, 0.024), (1.05, 0.066), (1.1, 0.084), (1.2, 0.100), (1.4, 0.114), (1.6, 0.117), (2.0, 0.113)];
const PARABOLIC_12: Pts = &[(0.8, 0.0), (0.9, 0.016), (0.95, 0.042), (1.0, 0.100), (1.05, 0.126), (1.1, 0.125), (1.3, 0.100), (1.5, 0.090), (1.8, 0.088)];

/// Piecewise-linear table, clamped outside its range.
#[derive(Clone, Debug)]
pub(super) struct Table(Vec<(f64, f64)>);

impl Table {
    pub(super) fn at(&self, x: f64) -> f64 {
        let p = &self.0;
        if x <= p[0].0 {
            return p[0].1;
        }
        let last = p[p.len() - 1];
        if x >= last.0 {
            return last.1;
        }
        let i = p.partition_point(|q| q.0 <= x);
        let (x0, y0) = p[i - 1];
        let (x1, y1) = p[i];
        y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    }
}

pub(super) fn static_table(p: Pts) -> Table {
    Table(p.to_vec())
}

/// Stagnation-pressure drag coefficient of a flat face: the isentropic stagnation pressure ratio
/// (series below Mach 1, Rayleigh pitot form above; Niskanen 2009 eq. B.1-B.2) with Hoerner's 0.85 factor.
pub(super) fn stagnation_cd(m: f64) -> f64 {
    let pressure = if m <= 1.0 { 1.0 + m * m / 4.0 + m.powi(4) / 40.0 } else { 1.84 - 0.76 / (m * m) + 0.166 / m.powi(4) + 0.035 / m.powi(6) };
    0.85 * pressure
}

/// Base drag coefficient on the base area (Fleeman, *Tactical Missile Design*; Niskanen 2009 eq. 3.94).
pub(super) fn base_cd(m: f64) -> f64 {
    if m <= 1.0 {
        0.12 + 0.13 * m * m
    } else {
        0.25 / m
    }
}

/// Stagnation drag sampled every 0.05 Mach and linearly interpolated (`bluntInterpolator`).
fn blunt_cd(m: f64) -> f64 {
    let m = m.clamp(0.0, 2.95);
    let i = (m / 0.05).floor();
    let (m0, m1) = (i * 0.05, (i + 1.0) * 0.05);
    let (c0, c1) = (stagnation_cd(m0), stagnation_cd(m1.min(2.95)));
    if m1 > 2.95 {
        return c0;
    }
    c0 + (c1 - c0) * (m - m0) / 0.05
}

/// Conical/ogive supersonic table: a cubic through Mach 1.0..1.3 (Niskanen 2009 eq. B.4-B.6), then
/// `2.1 sin^2(phi) + 0.5 sin(phi)/sqrt(M^2 - 1)`.
fn ogive_points(param: f64, sinphi: f64) -> Vec<(f64, f64)> {
    const GAMMA: f64 = 1.4;
    let cd1 = sinphi;
    let cd13 = 2.1 * sinphi * sinphi + 0.6019 * sinphi;
    let d1 = 4.0 / (GAMMA + 1.0) * (1.0 - 0.5 * cd1);
    let d13 = -1.1341 * sinphi;
    let mul = 0.72 * (param - 0.5) * (param - 0.5) + 0.82;
    let mut pts = Vec::new();
    // Hermite cubic on [1.0, 1.3] with the given end values and slopes.
    for i in 0..=15 {
        let m = 1.0 + 0.02 * f64::from(i);
        let t = (m - 1.0) / 0.3;
        let (t2, t3) = (t * t, t * t * t);
        let v = (2.0 * t3 - 3.0 * t2 + 1.0) * cd1 + (t3 - 2.0 * t2 + t) * 0.3 * d1 + (-2.0 * t3 + 3.0 * t2) * cd13 + (t3 - t2) * 0.3 * d13;
        pts.push((m, mul * v));
    }
    let mut j = 0;
    loop {
        let m = 1.32 + 0.02 * f64::from(j);
        if m >= 4.0 {
            break;
        }
        pts.push((m, mul * (2.1 * sinphi * sinphi + 0.5 * sinphi / (m * m - 1.0).max(0.0).sqrt())));
        j += 1;
    }
    pts
}

/// Blend of two tables with weight `p` on the second, evaluated at the union of their points.
fn blend(a: &Table, b: &Table, p: f64) -> Table {
    let mut xs: Vec<f64> = a.0.iter().chain(&b.0).map(|q| q.0).collect();
    xs.sort_by(f64::total_cmp);
    xs.dedup();
    Table(xs.into_iter().map(|m| (m, p * b.at(m) + (1.0 - p) * a.at(m))).collect())
}

/// Builds the nose/shoulder pressure-drag table (Cd on the frontal area `|A_aft - A_fore|` vs Mach)
/// following Niskanen (2009, section 3.4.3 and appendix B). `shape = None` is a conical transition.
pub(super) fn pressure_table(shape: Option<NoseShape>, fineness: f64, sinphi: f64) -> Table {
    let cone_sin = 1.0 / (1.0 + 4.0 * fineness * fineness).sqrt();
    let extrapolate = |t: Table| {
        let log4 = (fineness + 1.0).ln() / 4f64.ln();
        Table(
            t.0.iter()
                .map(|&(m, v)| {
                    let stag = blunt_cd(m);
                    (m, stag * (v / stag).powf(log4))
                })
                .collect(),
        )
    };
    let mut table = match shape {
        None | Some(NoseShape::Conical) => Table(ogive_points(0.0, sinphi)),
        // Ogives follow the wave drag of a cone with the same length and base diameter.
        Some(NoseShape::TangentOgive) => Table(ogive_points(1.0, cone_sin)),
        Some(NoseShape::Elliptical) => extrapolate(static_table(ELLIPSOID)),
        // The 1/2 parabola series: the tabulated TR R-100 profile data.
        Some(NoseShape::Parabolic) => extrapolate(blend(&Table(ogive_points(0.0, cone_sin)), &static_table(PARABOLIC_12), 1.0)),
        // HAACK with parameter 0 is the von Karman (LD-Haack c = 0) shape.
        Some(NoseShape::VonKarman) => extrapolate(blend(&static_table(VON_KARMAN), &static_table(LV_HAACK), 0.0)),
    };
    // Subsonic region: Cd = a M^b + Cd(0) when the first tabulated value is not ~0.
    let (min, min_v) = table.0[0];
    if min_v >= 0.001 {
        let cd0 = 0.8 * sinphi * sinphi;
        let min_deriv = (table.at(min + 0.01) - min_v) / 0.01;
        if cd0 < min_v - 0.01 && min_deriv > 0.01 {
            let b = min * min_deriv / (min_v - cd0);
            let a = (min_v - cd0) / min.powf(b);
            let mut sub = Vec::new();
            let mut i = 0;
            while f64::from(i) * 0.05 < min {
                let m = f64::from(i) * 0.05;
                sub.push((m, a * m.powf(b) + cd0));
                i += 1;
            }
            sub.extend(table.0);
            table = Table(sub);
        }
    }
    table
}

// ---------------------------------------------------------------------------------------------
// Spherically blunted nose tips. **Own estimate** (no published correlation was found for a blunt rocket-nose tip; see
// `docs/aero-references.md`): the nose's pressure drag on its base area is the sharp profile's drag and the drag of a
// hemispherical cap on its own frontal area, weighted by the areas they act on.
// ---------------------------------------------------------------------------------------------

/// Stagnation pressure coefficient behind a normal shock, `Cp_max = (2 / (gamma M^2)) (p02/p1 - 1)` with the Rayleigh pitot
/// ratio `p02/p1 = [(gamma+1)^2 M^2 / (4 gamma M^2 - 2 (gamma-1))]^(gamma/(gamma-1)) (1 - gamma + 2 gamma M^2)/(gamma+1)`
/// (Anderson, *Hypersonic and High-Temperature Gas Dynamics*, eq. 3.16-3.18; NACA Rep. 1135 eq. 100). Valid for `M >= 1`
/// (1.2756 at Mach 1, 1.8381 at Mach 25, 1.839 as `M` tends to infinity).
pub(super) fn cp_max(m: f64) -> f64 {
    const G: f64 = 1.4;
    let m2 = m * m;
    let pitot = ((G + 1.0).powi(2) * m2 / (4.0 * G * m2 - 2.0 * (G - 1.0))).powf(G / (G - 1.0)) * ((1.0 - G + 2.0 * G * m2) / (G + 1.0));
    2.0 / (G * m2) * (pitot - 1.0)
}

/// The nose table of a hemispherical nose (an elliptical nose of length `R`, `L/D = 1/2`) as the build-up gives it: the subsonic
/// drag of a hemisphere has the same provenance as every other nose's.
pub(super) fn hemisphere_table() -> Table {
    // `body_calc`'s sin(phi) for an elliptical nose at 0.99 L, in units of R = L.
    let r = (1.0 - (1.0 - 0.99f64).powi(2)).sqrt();
    let sinphi = (1.0 - r) / (1.0 - r).hypot(0.01);
    pressure_table(Some(NoseShape::Elliptical), 0.5, sinphi)
}

/// Drag coefficient of a hemispherical cap on its own frontal area at Mach `m`. At `M >= 1` the modified Newtonian value
/// `Cp_max / 2`: the pressure `Cp = Cp_max cos^2(phi)` (Lees; Anderson eq. 3.15) on the hemisphere, whose axial force on its
/// frontal area is `2 Cp_max integral of cos^3(phi) sin(phi) d phi = Cp_max / 2` (hemisphere `0..pi/2`). Below Mach 1 the table
/// of a hemispherical nose (`hemisphere`, see [`hemisphere_table`]) up to Mach 0.95; between 0.95 and 1 the two are joined by
/// linear interpolation over that 0.05 step, the way [`blunt_cd`] samples the stagnation drag every 0.05 Mach and interpolates.
pub(super) fn cap_cd(m: f64, hemisphere: &Table) -> f64 {
    const JOIN: f64 = 0.95;
    let newton = |m: f64| 0.5 * cp_max(m);
    if m >= 1.0 {
        newton(m)
    } else if m <= JOIN {
        hemisphere.at(m)
    } else {
        let below = hemisphere.at(JOIN);
        below + (m - JOIN) / (1.0 - JOIN) * (newton(1.0) - below)
    }
}

/// Pressure drag coefficient of a blunted nose on its base area: `CD_sharp (1 - k^2) + CD_cap k^2` with `k = r_n / R` the tip
/// radius over the base radius. The cap's own frontal area is `k^2` of the base area; the sharp profile's drag acts on the rest.
pub(super) fn blunt_nose_cd(sharp: f64, cap: f64, tip_ratio: f64) -> f64 {
    let w = tip_ratio * tip_ratio;
    sharp * (1.0 - w) + cap * w
}

// ---------------------------------------------------------------------------------------------
// Skin friction: Blasius laminar and Prandtl-Schlichting turbulent flat-plate laws (Schlichting; Niskanen 2009 section 3.4.2).
// ---------------------------------------------------------------------------------------------

/// Flat-plate skin friction coefficient at length-Reynolds number `re` including the
/// compressibility correction. With `perfect_finish` the laminar-to-turbulent transition is
/// modelled (laminar below Re 5.39e5, `1/(1.5 ln Re - 5.6)^2 - 1700/Re` above) and the
/// compressibility corrections apply only for Re > 1e6; otherwise the fully turbulent formula is used.
pub(super) fn skin_cf(re: f64, mach: f64, perfect: bool) -> f64 {
    let (mut c1, mut c2) = (1.0, 1.0);
    let mut cf;
    if perfect {
        cf = if re < 1e4 {
            1.33e-2
        } else if re < 5.39e5 {
            1.328 / re.sqrt()
        } else {
            1.0 / (1.5 * re.ln() - 5.6).powi(2) - 1700.0 / re
        };
        if mach < 1.1 && re > 1e6 {
            c1 = if re < 3e6 { 1.0 - 0.1 * mach * mach * (re - 1e6) / 2e6 } else { 1.0 - 0.1 * mach * mach };
        }
        if mach > 0.9 && re > 1e6 {
            let k = 1.0 / (1.0 + 0.045 * mach * mach).powf(0.25);
            c2 = if re < 3e6 { 1.0 + (k - 1.0) * (re - 1e6) / 2e6 } else { k };
        }
    } else {
        cf = if re < 1e4 { 1.48e-2 } else { 1.0 / (1.5 * re.ln() - 5.6).powi(2) };
        if mach < 1.1 {
            c1 = 1.0 - 0.1 * mach * mach;
        }
        if mach > 0.9 {
            c2 = 1.0 / (1.0 + 0.15 * mach * mach).powf(0.58);
        }
    }
    cf *= if mach < 0.9 {
        c1
    } else if mach < 1.1 {
        c2 * (mach - 0.9) / 0.2 + c1 * (1.1 - mach) / 0.2
    } else {
        c2
    };
    cf
}

/// Compressibility factor applied to the roughness-limited skin friction.
pub(super) fn roughness_correction(mach: f64) -> f64 {
    if mach < 0.9 {
        1.0 - 0.1 * mach * mach
    } else if mach > 1.1 {
        1.0 / (1.0 + 0.18 * mach * mach)
    } else {
        let c1 = 1.0 - 0.1 * 0.81;
        let c2 = 1.0 / (1.0 + 0.18 * 1.21);
        c2 * (mach - 0.9) / 0.2 + c1 * (1.1 - mach) / 0.2
    }
}
