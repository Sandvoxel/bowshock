use super::fin::{roll_factor, SUBSONIC_CP_POS};
use super::tables::{hemisphere_table, pressure_table};
use super::{BodyCalc, FinCalc, Pressure};
use crate::vehicle::{BodyKind, BodySegment, FinPlanform, FinSet, NoseShape};
use std::f64::consts::PI;
/// Multiplier turning zero-alpha drag into axial drag at angle of attack `alpha` (Niskanen 2009 section 3.4.7;
/// the shape is our own engineering fit): a cubic from 1 at 0 to 1.3 at 17 deg, then a quartic falling to 0 at 90 deg.
pub fn axial_drag_multiplier(alpha: f64) -> f64 {
    let a = alpha.abs();
    let a = if a > PI / 2.0 { PI - a } else { a };
    let a17 = 17f64.to_radians();
    if a < a17 {
        // Hermite cubic: p(0) = 1, p(a17) = 1.3, zero slope at both ends.
        let t = a / a17;
        1.0 + 0.3 * t * t * (3.0 - 2.0 * t)
    } else {
        // p(x) = c3 t^3 + c4 t^4 with t = x - pi/2: zero value, slope and curvature at 90 deg.
        let t = a - PI / 2.0;
        let ta = a17 - PI / 2.0;
        let c4 = -3.9 / ta.powi(4);
        let c3 = 5.2 / ta.powi(3);
        (c3 * t.powi(3) + c4 * t.powi(4)).max(0.0)
    }
}

/// Launch lug drag (Niskanen 2009 section 3.4.6): Darcy-Weisbach pressure drop
/// through the bore (Swamee-Jain friction factor) plus `0.7 (stagnation + base)` on the frontal ring.
pub(crate) fn lug_drag(re_per_m: f64, d_in: f64, inner_area: f64, frontal: f64, length: f64, eps: f64, stag: f64, base: f64, a_ref: f64) -> f64 {
    if re_per_m < 1e-3 {
        return 0.0;
    }
    let mut tube_cd = 0.0;
    if inner_area > 1e-12 && d_in > 0.0 {
        let re = (re_per_m * d_in).max(1.0);
        let f = 0.25 / (eps / (3.7 * d_in) + 5.74 / re.powf(0.9)).log10().powi(2);
        tube_cd = f * length / d_in;
    }
    (tube_cd * inner_area + 0.7 * (stag + base) * frontal) / a_ref
}

/// Rail button drag: cylinder drag (Gowen & Perkins, NACA TN 2960 fig. 7) at the Mach number of the air inside the boundary layer at the button's station.
///
/// The layer is the turbulent one of the rest of the build-up: thickness `delta = 0.37 x / Re_x^0.2` (Schlichting) and its 1/7 power-law velocity profile `u/U = (y/delta)^(1/7)`.
/// The button stands in it from the wall to its height `h`, so it sees the mean dynamic pressure `q/q_inf = (1/h) integral (u/U)^2 dy` (`7/9 (h/delta)^(2/7)` for a button inside the
/// layer) and the Mach number `M sqrt(q/q_inf)`. (The formula had a linear profile, a laminar-like shape inside the turbulent thickness: a button half the layer thick then saw 6 % of
/// the free-stream dynamic pressure instead of 64 %, and the transonic rise of the cylinder drag came at Mach 3-4. RASAero II models its rail guides as cylinders in the flow: its drag plot rises from Mach 0.8.)
pub(super) fn button_drag(mach: f64, re_per_m: f64, station: f64, height: f64, diameter: f64, stag: f64, a_ref: f64) -> f64 {
    const DOMAIN: [f64; 12] = [0.0, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 1.0, 1.6, 2.0, 2.8, 100.0];
    const RANGE: [f64; 12] = [1.2, 1.22, 1.25, 1.3, 1.4, 1.5, 1.6, 2.1, 1.5, 1.45, 1.33, 1.33];
    let ref_area = height * diameter;
    let mul = if mach > 1e-9 && re_per_m > 0.0 && station > 0.0 && height > 0.0 {
        let rex = re_per_m * station;
        let del = 0.37 * station / rex.powf(0.2);
        let q_ratio = if height > del { (7.0 / 9.0 * del + (height - del)) / height } else { 7.0 / 9.0 * (height / del).powf(2.0 / 7.0) };
        let m_eff = mach * q_ratio.sqrt();
        let i = DOMAIN.partition_point(|&d| d <= m_eff).clamp(1, DOMAIN.len() - 1);
        let t = ((m_eff - DOMAIN[i - 1]) / (DOMAIN[i] - DOMAIN[i - 1])).clamp(0.0, 1.0);
        (RANGE[i - 1] + (RANGE[i] - RANGE[i - 1]) * t) * q_ratio
    } else {
        // no flow to measure the layer by: the cylinder in still air
        RANGE[0]
    };
    mul * stag * ref_area / a_ref
}

/// Numerically integrates a body segment's geometry and selects its pressure-drag model.
pub(super) fn body_calc(s: &BodySegment, prev_aft: f64, next_fore: Option<f64>, a_ref: f64) -> BodyCalc {
    // The cap of a blunted tip is a few strips of a 64-strip nose: it gets 256.
    let blunt = s.blunt_nose();
    let n: usize = if blunt.is_some() { 256 } else { 64 };
    let l = s.length.max(1e-12);
    let dx = l / n as f64;
    let (mut vol, mut plan, mut plan_m, mut wet) = (0.0, 0.0, 0.0, 0.0);
    let mut strips: Vec<(f64, f64)> = Vec::with_capacity(n);
    let mut r_prev = s.radius_at(0.0);
    for i in 0..n {
        let x_mid = (i as f64 + 0.5) * dx;
        let r_next = s.radius_at((i + 1) as f64 * dx);
        let r = 0.5 * (r_prev + r_next);
        let slope = (r_next - r_prev) / dx;
        vol += PI * r * r * dx;
        plan += 2.0 * r * dx;
        plan_m += 2.0 * r * dx * x_mid;
        wet += 2.0 * PI * r * (1.0 + slope * slope).sqrt() * dx;
        strips.push((s.station + x_mid, 2.0 * r * dx));
        r_prev = r_next;
    }
    let (r0, r1) = (s.fore_radius, s.aft_radius);
    let a_fore = PI * r0 * r0;
    let a_aft = PI * r1 * r1;
    let da = a_aft - a_fore;
    let (cna, cp_local) = match s.kind {
        BodyKind::Tube => (0.0, 0.5 * l),
        _ if da.abs() < 1e-12 => (0.0, 0.5 * l),
        _ => (2.0 * da / a_ref, (l * a_aft - vol) / da),
    };
    let (name, shape) = match s.kind {
        BodyKind::Nose(sh) => ("Nose cone", Some(sh)),
        BodyKind::Tube => ("Body tube", None),
        BodyKind::Transition => ("Transition", None),
    };

    // Pressure drag (Niskanen 2009 section 3.4.3).
    let frontal = (a_fore - a_aft).abs();
    let fineness = s.length / (2.0 * (r1 - r0).abs()).max(1e-12);
    let pressure = if matches!(s.kind, BodyKind::Tube) || (r0 - r1).abs() < 1e-9 {
        Pressure::None
    } else if s.length < 0.001 {
        if r0 < r1 {
            Pressure::Stagnation { area: frontal }
        } else {
            Pressure::Base { area: frontal, scale: 1.0 }
        }
    } else if r1 < r0 {
        // Boat-tail drag derived from base drag.
        if fineness >= 3.0 {
            Pressure::None
        } else {
            Pressure::Base { area: frontal, scale: if fineness <= 1.0 { 1.0 } else { (3.0 - fineness) / 2.0 } }
        }
    } else {
        // A blunted nose's pressure drag is that of the sharp profile its tip sits on, so its fineness and the slope near its base
        // are those of the sharp nose (`sharp_length`); the tip itself is `Pressure::BluntTable`'s cap (own estimate).
        let ls = s.sharp_length();
        let r = if blunt.is_some() { s.radius_at(s.length - 0.01 * ls) } else { s.radius_at(0.99 * s.length) };
        let tangent_ogive = matches!(shape, Some(NoseShape::TangentOgive));
        let sinphi = if tangent_ogive { 0.0 } else { (r1 - r) / ((r1 - r).hypot(0.01 * ls)) };
        let table = pressure_table(shape, ls / (2.0 * (r1 - r0).abs()).max(1e-12), sinphi);
        match blunt {
            Some(b) => Pressure::BluntTable { table, hemisphere: hemisphere_table(), tip_ratio: b.tip_radius / r1, area: frontal },
            None => Pressure::Table { table, area: frontal },
        }
    };

    let next_r = next_fore.unwrap_or(0.0);
    BodyCalc {
        name: name.to_string(),
        cna,
        cp: s.station + cp_local,
        plan_area: plan,
        plan_cp: s.station + if plan > 0.0 { plan_m / plan } else { 0.5 * l },
        wet_area: wet,
        roughness: s.roughness,
        pressure,
        front_face: PI * (r0 * r0 - prev_aft * prev_aft).max(0.0),
        back_face: PI * (r1 * r1 - next_r * next_r).max(0.0),
        is_last: next_fore.is_none(),
        strips,
    }
}

pub(super) fn fin_calc(f: &FinSet, name: String) -> FinCalc {
    let area = f.fin_area().max(1e-12);
    let span = f.span.max(1e-9);
    let ar = 2.0 * span * span / area;
    let (cos_mid, cos_lead, x_cp, mac) = match f.planform {
        FinPlanform::Trapezoidal => {
            let (cr, ct, m) = (f.root_chord, f.tip_chord, f.sweep);
            let lm = (span * span + (m + 0.5 * (ct - cr)).powi(2)).sqrt();
            let le = (span * span + m * m).sqrt();
            let sum = (cr + ct).max(1e-12);
            let x = m * (cr + 2.0 * ct) / (3.0 * sum) + (sum - cr * ct / sum) / 6.0;
            (f.mid_chord_cos.unwrap_or(span / lm), span / le, x, 2.0 / 3.0 * (sum - cr * ct / sum))
        }
        FinPlanform::Elliptical => {
            // Straight mid-chord line; leading-edge sweep averaged over 47 spanwise segments.
            const DIV: usize = 48;
            let lead = |y: f64| 0.5 * f.root_chord * (1.0 - (1.0 - (y / span).powi(2)).max(0.0).sqrt());
            let dy = span / (DIV - 1) as f64;
            let mut c = 0.0;
            for i in 1..DIV {
                let dx = lead(i as f64 * dy) - lead((i - 1) as f64 * dy);
                c += dy / dx.hypot(dy);
            }
            // Area-weighted quarter-chord at 0.2878 c_r; MAC = 8 c_r / (3 pi).
            (1.0, c / (DIV - 1) as f64, 0.2878 * f.root_chord, 8.0 / (3.0 * PI) * f.root_chord)
        }
    };
    // Interference of the body on the fin: `K_fb = 1 + r/(s + r)`.
    let k_fb = 1.0 + f.body_radius / (span + f.body_radius);
    // Spanwise strip integrals for the roll moments (chord-weighted strip theory, Barrowman 1967 / Niskanen 2009 section
    // 3.2.3): the arm of the fin's lift and the chord-weighted mean-square arm, both from the body axis.
    let chord = |y: f64| match f.planform {
        FinPlanform::Trapezoidal => f.root_chord + (f.tip_chord - f.root_chord) * y / span,
        FinPlanform::Elliptical => f.root_chord * (1.0 - (y / span).powi(2)).max(0.0).sqrt(),
    };
    const STRIPS: usize = 32;
    let (mut c0, mut c1, mut c2) = (0.0, 0.0, 0.0);
    for i in 0..STRIPS {
        let y = (i as f64 + 0.5) / STRIPS as f64 * span;
        let (c, r) = (chord(y).max(0.0), f.body_radius + y);
        (c0, c1, c2) = (c0 + c, c1 + c * r, c2 + c * r * r);
    }
    let (roll_arm, roll_arm2) = if c0 > 0.0 { (c1 / c0, c2 / c0) } else { (f.body_radius + 0.5 * span, (f.body_radius + 0.5 * span).powi(2)) };
    FinCalc {
        name,
        n: f64::from(f.count),
        area,
        span,
        ar,
        cos_mid,
        cos_lead,
        mac_lead: f.station + x_cp - SUBSONIC_CP_POS * mac,
        lift: roll_factor(&f.angles, f.count) * k_fb,
        unit_lift: k_fb,
        cant: f.cant,
        roll_arm,
        roll_arm2,
        on_core: f.bodies <= 1,
        root: f.root_extent.unwrap_or((f.station, f.station + f.root_chord)),
        parent: f.parent,
        per_body: f64::from(f.count) / f64::from(f.bodies.max(1)),
        mac,
        thickness: f.thickness,
        roughness: f.roughness,
        cross: f.cross_section,
    }
}
