//! Barrowman / OpenRocket-style aerodynamics: normal force, centre of pressure and drag.
//!
//! * Normal force slope and CP follow Barrowman (1967) for noses, transitions and fins, with
//!   OpenRocket's additions: body lift `1.1 A_plan/A_ref sin^2(alpha)` acting at the planform
//!   centroid (so the CP moves with angle of attack), Diederich fin lift with compressibility
//!   correction, and a body-fin interference factor `K_fb = 1 + r/(s + r)`.
//! * Drag follows the OpenRocket technical documentation (Niskanen 2013, section 3): skin
//!   friction with transition and roughness limits, body pressure/stagnation/base drag, fin
//!   leading/trailing-edge drag and protuberance drag. The nose/transition wave drag is a
//!   compact fit (equivalent-cone theory scaled per profile family) rather than OpenRocket's
//!   tabulated polynomials, and the transonic rise is a smooth window; see the comments.
//!
//! Coefficients are referenced to the vehicle reference area; stations are from the nose tip.

use crate::vehicle::{BodyKind, BodySegment, FinCrossSection, FinPlanform, FinSet, NoseShape, Protuberance, Vehicle};
use std::f64::consts::{FRAC_PI_2, PI};

/// Per-component and total normal-force data at a flight condition.
#[derive(Clone, Debug, PartialEq)]
pub struct Stability {
    /// Total normal force coefficient slope, `CN/alpha`, per radian.
    pub cn_alpha: f64,
    /// Centre of pressure station from the tip (m).
    pub cp: f64,
    /// `(component name, CN_alpha per rad, CP station m)` for each component with lift.
    pub components: Vec<(String, f64, f64)>,
}

/// Drag coefficient (reference-area based) broken into the categories the UI stacks.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DragBreakdown {
    /// Body skin friction.
    pub friction: f64,
    /// Body pressure drag (nose/transition wave drag, stagnation on blunt faces).
    pub pressure: f64,
    /// Base drag.
    pub base: f64,
    /// Fins: skin friction plus leading/trailing-edge pressure drag.
    pub fins: f64,
    /// Launch lugs and rail buttons.
    pub protuberances: f64,
    /// Sum of all of the above.
    pub total: f64,
}

#[derive(Clone, Debug)]
struct BodyCalc {
    name: String,
    cna: f64,
    cp: f64,
    plan_area: f64,
    plan_cp: f64,
    wet_area: f64,
    roughness: f64,
    /// Hypersonic-limit wave-drag coefficient on the reference area (>= 0).
    wave: f64,
    /// Forward-facing exposed annulus area (m^2).
    front_face: f64,
    /// Backward-facing exposed annulus area (m^2).
    back_face: f64,
    is_last: bool,
}

#[derive(Clone, Debug)]
struct FinCalc {
    name: String,
    n: f64,
    area: f64,
    span: f64,
    ar: f64,
    cos_mid: f64,
    cos2_le: f64,
    x_cp: f64,
    kfb: f64,
    mean_chord: f64,
    thickness: f64,
    roughness: f64,
    cross: FinCrossSection,
}

#[derive(Clone, Debug)]
struct ProtCalc {
    lug: bool,
    frontal: f64,
    wet: f64,
    length: f64,
}

/// Aerodynamic model of one vehicle configuration.
#[derive(Clone, Debug)]
pub struct Aero {
    d_ref: f64,
    a_ref: f64,
    length: f64,
    max_diameter: f64,
    motor_exit_area: f64,
    bodies: Vec<BodyCalc>,
    fins: Vec<FinCalc>,
    prots: Vec<ProtCalc>,
    /// `(roughness m, wetted area m^2)` of the body, grouped so each skin-friction value is evaluated once.
    friction_groups: Vec<(f64, f64)>,
}

fn smoothstep(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

impl Aero {
    /// Precomputes geometry-dependent quantities for `v`.
    pub fn new(v: &Vehicle) -> Self {
        let a_ref = v.reference_area().max(1e-12);
        let length = v.length();
        let n_seg = v.body.len();
        let mut bodies = Vec::with_capacity(n_seg);
        for (i, s) in v.body.iter().enumerate() {
            let prev_aft = if i == 0 { 0.0 } else { v.body[i - 1].aft_radius };
            let next_fore = v.body.get(i + 1).map(|n| n.fore_radius);
            bodies.push(body_calc(s, prev_aft, next_fore, a_ref));
        }
        let max_r = v.body.iter().map(|s| s.fore_radius.max(s.aft_radius)).fold(0.0, f64::max);
        let mut fins = Vec::new();
        for (i, f) in v.fin_sets.iter().enumerate() {
            let name = if v.fin_sets.len() > 1 { format!("Fin set {}", i + 1) } else { "Fins".to_string() };
            fins.push(fin_calc(f, name));
        }
        let prots = v
            .protuberances
            .iter()
            .map(|p| match *p {
                Protuberance::LaunchLug { length, outer_diameter, inner_diameter, .. } => ProtCalc {
                    lug: true,
                    frontal: 0.25 * PI * (outer_diameter * outer_diameter - inner_diameter * inner_diameter).max(0.0),
                    wet: PI * outer_diameter * length,
                    length,
                },
                Protuberance::RailButton { height, diameter, .. } => ProtCalc { lug: false, frontal: diameter * height, wet: 0.0, length: diameter },
            })
            .collect();
        let mut friction_groups: Vec<(f64, f64)> = Vec::new();
        for b in &bodies {
            match friction_groups.iter_mut().find(|g| g.0 == b.roughness) {
                Some(g) => g.1 += b.wet_area,
                None => friction_groups.push((b.roughness, b.wet_area)),
            }
        }
        let motor_d = v
            .stages
            .iter()
            .filter_map(|s| s.motor.as_ref())
            .filter(|m| m.mount_aft_station - m.curve.length < length)
            .map(|m| m.curve.diameter)
            .fold(0.0, f64::max);
        Self {
            d_ref: v.reference_diameter,
            a_ref,
            length,
            max_diameter: 2.0 * max_r,
            motor_exit_area: 0.25 * PI * motor_d * motor_d,
            bodies,
            fins,
            prots,
            friction_groups,
        }
    }

    /// Reference diameter (m).
    pub fn reference_diameter(&self) -> f64 {
        self.d_ref
    }

    /// Normal-force slope and CP at `mach` and angle of attack `alpha` (rad).
    ///
    /// The slope includes OpenRocket's nonlinear body lift evaluated at `alpha`, so
    /// `cn_alpha * alpha` is the normal force coefficient `CN` there.
    pub fn stability(&self, mach: f64, alpha: f64) -> Stability {
        let mut comps: Vec<(String, f64, f64)> = Vec::new();
        let sin_a = alpha.abs().sin();
        for b in &self.bodies {
            let mut cna = b.cna;
            let mut moment = b.cna * b.cp;
            if alpha.abs() > 1e-9 {
                // OpenRocket body lift: CN = 1.1 A_plan / A_ref sin^2(alpha), at the planform centroid.
                let extra = 1.1 * b.plan_area / self.a_ref * sin_a * sin_a / alpha.abs();
                cna += extra;
                moment += extra * b.plan_cp;
            }
            if cna.abs() > 1e-12 {
                comps.push((b.name.clone(), cna, moment / cna));
            }
        }
        for f in &self.fins {
            let cna = f.cna(mach, self.a_ref);
            comps.push((f.name.clone(), cna, f.x_cp));
        }
        let cn_alpha: f64 = comps.iter().map(|c| c.1).sum();
        let cp = if cn_alpha.abs() > 1e-9 { comps.iter().map(|c| c.1 * c.2).sum::<f64>() / cn_alpha } else { 0.5 * self.length };
        Stability { cn_alpha, cp, components: comps }
    }

    /// Static margin in calibers (positive = stable) for centre of gravity station `cg`.
    pub fn margin_cal(&self, mach: f64, alpha: f64, cg: f64) -> f64 {
        (self.stability(mach, alpha).cp - cg) / self.d_ref
    }

    /// Zero-angle-of-attack drag at `mach` with `re_per_m = v/nu` (Reynolds number per metre).
    /// With `motor_on` the base area covered by the motor exhaust carries no base drag.
    pub fn drag(&self, mach: f64, re_per_m: f64, motor_on: bool) -> DragBreakdown {
        let m = mach.max(0.0);
        let ar = self.a_ref;
        let re = re_per_m.max(1.0);

        // Body friction (OpenRocket: Cf [1 + 0.5/fB] A_wet / A_ref).
        let f_b = (self.length / self.max_diameter.max(1e-9)).max(1.0);
        let re_len = re * self.length;
        let mut friction = 0.0;
        for &(rough, wet) in &self.friction_groups {
            friction += skin_cf(re_len, rough / self.length.max(1e-9), m) * wet;
        }
        friction *= (1.0 + 0.5 / f_b) / ar;

        // Body pressure drag.
        let wave_f = wave_window(m);
        let stag = 0.85 * stagnation_cd(m);
        let mut pressure = 0.0;
        for b in &self.bodies {
            pressure += b.wave * wave_f + stag * b.front_face / ar;
        }

        // Base drag.
        let cd_base = base_cd(m);
        let mut base = 0.0;
        for b in &self.bodies {
            let mut area = b.back_face;
            if motor_on && b.is_last {
                area = (area - self.motor_exit_area).max(0.0);
            }
            base += cd_base * area / ar;
        }

        // Fins.
        let mut fins = 0.0;
        for f in &self.fins {
            fins += f.drag(m, re, ar, cd_base, stag);
        }

        // Protuberances.
        let mut prot = 0.0;
        for p in &self.prots {
            if p.lug {
                let cf = skin_cf(re * p.length.max(1e-6), 0.0, m);
                prot += (stag * p.frontal + cf * p.wet) / ar;
            } else {
                prot += 1.2 * stagnation_cd(m) * p.frontal / ar;
            }
        }

        let total = friction + pressure + base + fins + prot;
        DragBreakdown { friction, pressure, base, fins, protuberances: prot, total }
    }

    /// Drag at angle of attack `alpha`: the zero-alpha build-up plus the induced term
    /// `CN sin(alpha)` (normal force resolved along the flight path) added to pressure drag.
    pub fn drag_at_alpha(&self, mach: f64, re_per_m: f64, motor_on: bool, alpha: f64) -> DragBreakdown {
        let mut d = self.drag(mach, re_per_m, motor_on);
        if alpha.abs() > 1e-9 {
            let st = self.stability(mach, alpha);
            let induced = (st.cn_alpha * alpha.abs() * alpha.abs().sin()).max(0.0);
            d.pressure += induced;
            d.total += induced;
        }
        d
    }
}

/// Numerically integrates a body segment's geometry.
fn body_calc(s: &BodySegment, prev_aft: f64, next_fore: Option<f64>, a_ref: f64) -> BodyCalc {
    const N: usize = 64;
    let l = s.length.max(1e-12);
    let dx = l / N as f64;
    let (mut vol, mut plan, mut plan_m, mut wet) = (0.0, 0.0, 0.0, 0.0);
    let mut r_prev = s.radius_at(0.0);
    for i in 0..N {
        let x_mid = (i as f64 + 0.5) * dx;
        let r_next = s.radius_at((i + 1) as f64 * dx);
        let r = 0.5 * (r_prev + r_next);
        let slope = (r_next - r_prev) / dx;
        vol += PI * r * r * dx;
        plan += 2.0 * r * dx;
        plan_m += 2.0 * r * dx * x_mid;
        wet += 2.0 * PI * r * (1.0 + slope * slope).sqrt() * dx;
        r_prev = r_next;
    }
    let a_fore = PI * s.fore_radius * s.fore_radius;
    let a_aft = PI * s.aft_radius * s.aft_radius;
    let da = a_aft - a_fore;
    let (cna, cp_local) = match s.kind {
        BodyKind::Tube => (0.0, 0.5 * l),
        _ if da.abs() < 1e-12 => (0.0, 0.5 * l),
        _ => (2.0 * da / a_ref, (l * a_aft - vol) / da),
    };
    let (name, shape_mult) = match s.kind {
        BodyKind::Nose(sh) => (
            "Nose cone".to_string(),
            match sh {
                NoseShape::Conical => 1.0,
                NoseShape::TangentOgive => 0.72,
                NoseShape::VonKarman => 0.55,
                NoseShape::Elliptical => 0.9,
                NoseShape::Parabolic => 0.70,
            },
        ),
        BodyKind::Tube => ("Body tube".to_string(), 0.0),
        BodyKind::Transition => ("Transition".to_string(), 1.0),
    };
    // Equivalent-cone wave drag in the hypersonic limit: Cd = 2 sin^2(theta) (A_aft - A_fore)/A_ref,
    // scaled by the profile family; only expanding surfaces contribute.
    let theta = ((s.aft_radius - s.fore_radius) / l).atan();
    let wave = if da > 0.0 { 2.0 * theta.sin().powi(2) * shape_mult * da / a_ref } else { 0.0 };
    let next_r = next_fore.unwrap_or(0.0);
    BodyCalc {
        name,
        cna,
        cp: s.station + cp_local,
        plan_area: plan,
        plan_cp: s.station + if plan > 0.0 { plan_m / plan } else { 0.5 * l },
        wet_area: wet,
        roughness: s.roughness,
        wave,
        front_face: PI * (s.fore_radius * s.fore_radius - prev_aft * prev_aft).max(0.0),
        back_face: PI * (s.aft_radius * s.aft_radius - next_r * next_r).max(0.0),
        is_last: next_fore.is_none(),
    }
}

fn fin_calc(f: &FinSet, name: String) -> FinCalc {
    let area = f.fin_area().max(1e-12);
    let span = f.span.max(1e-9);
    let ar = 2.0 * span * span / area;
    let (cos_mid, cos2_le, x_cp, mean_chord) = match f.planform {
        FinPlanform::Trapezoidal => {
            let (cr, ct, m) = (f.root_chord, f.tip_chord, f.sweep);
            let mid_off = m + 0.5 * (ct - cr);
            let lm = (span * span + mid_off * mid_off).sqrt();
            let le = (span * span + m * m).sqrt();
            let sum = (cr + ct).max(1e-12);
            let x = m * (cr + 2.0 * ct) / (3.0 * sum) + (sum - cr * ct / sum) / 6.0;
            (span / lm, (span / le).powi(2), x, area / span)
        }
        // Semi-elliptical: straight mid-chord line, area-weighted quarter-chord at 0.2878 c_r.
        FinPlanform::Elliptical => (1.0, 0.5, 0.2878 * f.root_chord, area / span),
    };
    FinCalc {
        name,
        n: f64::from(f.count),
        area,
        span,
        ar,
        cos_mid,
        cos2_le,
        x_cp: f.station + x_cp,
        kfb: 1.0 + f.body_radius / (span + f.body_radius),
        mean_chord,
        thickness: f.thickness,
        roughness: f.roughness,
        cross: f.cross_section,
    }
}

impl FinCalc {
    /// Lift slope of one fin pair on the fin-area basis (Diederich subsonic, linearised supersonic).
    fn pair_cl(&self, mach: f64) -> f64 {
        let sub = |m: f64| {
            let beta = (1.0 - m * m).max(0.0).sqrt();
            let x = beta * self.ar / self.cos_mid;
            2.0 * PI * self.ar / (2.0 + (4.0 + x * x).sqrt())
        };
        let sup = |m: f64| {
            let beta = (m * m - 1.0).max(1e-6).sqrt();
            let x = beta * self.ar;
            if x >= 1.0 {
                4.0 / beta * (1.0 - 1.0 / (2.0 * x))
            } else {
                self.ar * (FRAC_PI_2 + (2.0 - FRAC_PI_2) * x)
            }
        };
        if mach <= 0.9 {
            sub(mach)
        } else if mach >= 1.4 {
            sup(mach)
        } else {
            let t = (mach - 0.9) / 0.5;
            sub(0.9) * (1.0 - t) + sup(1.4) * t
        }
    }

    fn cna(&self, mach: f64, a_ref: f64) -> f64 {
        self.pair_cl(mach) * self.area / a_ref * 0.5 * self.n * self.kfb
    }

    fn drag(&self, mach: f64, re: f64, a_ref: f64, cd_base: f64, stag: f64) -> f64 {
        // Skin friction on both faces with a thickness correction (1 + 2 t/c).
        let re_c = re * self.mean_chord;
        let cf = skin_cf(re_c, self.roughness / self.mean_chord.max(1e-9), mach);
        let tc = self.thickness / self.mean_chord.max(1e-9);
        let friction = cf * (1.0 + 2.0 * tc) * 2.0 * self.area * self.n / a_ref;
        // Edge pressure drag.
        let a_edge = self.n * self.span * self.thickness / a_ref;
        let (le, te) = match self.cross {
            FinCrossSection::Square => (stag * self.cos2_le * a_edge, cd_base * a_edge),
            FinCrossSection::Rounded => {
                let blunt = 0.15 + 0.85 * smoothstep((mach - 0.8) / 0.3);
                (blunt * stag * self.cos2_le * a_edge, 0.5 * cd_base * a_edge)
            }
            FinCrossSection::Airfoil => {
                // Ackeret biconvex wave drag, cd = 16/3 (t/c)^2 / beta, on planform area.
                let beta = (mach.max(1.2).powi(2) - 1.0).sqrt();
                let w = smoothstep((mach - 0.8) / 0.4);
                (16.0 / 3.0 * tc * tc / beta * w * self.n * self.area / a_ref, 0.0)
            }
        };
        friction + le + te
    }
}

/// Skin friction coefficient of a flat plate of length-Reynolds number `re` (OpenRocket section 3.1).
///
/// Laminar below the critical Reynolds number, turbulent (with the transition correction)
/// above it, floored by the roughness-limited value, then compressibility-corrected.
fn skin_cf(re: f64, rough_ratio: f64, mach: f64) -> f64 {
    let re = re.max(1.0);
    let lam = |r: f64| if r < 1e4 { 1.48e-2 } else { 1.328 / r.sqrt() };
    let turb = |r: f64| 1.0 / (1.5 * r.ln() - 5.6).powi(2);
    let mut re_c = 5e5_f64;
    if rough_ratio > 0.0 {
        re_c = re_c.min(51.0 * rough_ratio.powf(-1.039));
    }
    let re_c = re_c.max(1e4);
    let mut cf = if re < re_c {
        lam(re)
    } else {
        let a = re_c * (turb(re_c) - lam(re_c));
        (turb(re) - a / re).max(lam(re))
    };
    if rough_ratio > 0.0 && re >= re_c {
        cf = cf.max(0.032 * rough_ratio.powf(0.2));
    }
    // Compressibility.
    let sub = cf * (1.0 - 0.1 * mach * mach);
    let sup = cf / (1.0 + 0.15 * mach * mach).powf(0.58);
    if mach < 0.9 {
        sub
    } else if mach > 1.1 {
        sup
    } else {
        let t = (mach - 0.9) / 0.2;
        sub * (1.0 - t) + sup * t
    }
}

/// Stagnation-pressure drag coefficient of a flat face (OpenRocket).
fn stagnation_cd(m: f64) -> f64 {
    if m < 1.0 {
        1.0 + m * m / 4.0 + m.powi(4) / 40.0
    } else {
        1.84 - 0.76 / m.powi(2) + 0.166 / m.powi(4) + 0.035 / m.powi(6)
    }
}

/// OpenRocket base drag coefficient on the base area.
fn base_cd(m: f64) -> f64 {
    if m < 1.0 {
        0.12 + 0.13 * m * m
    } else {
        0.25 / m
    }
}

/// Mach dependence of body wave drag relative to the hypersonic equivalent-cone value:
/// zero below Mach 0.8, a smooth rise with a transonic overshoot, then `1 + 0.9/M^2`.
fn wave_window(m: f64) -> f64 {
    if m <= 0.8 {
        return 0.0;
    }
    let sup = 1.0 + 0.9 / m.max(1.2).powi(2);
    let bump = 1.0 + 0.3 * (-((m - 1.0) / 0.12).powi(2)).exp();
    sup * smoothstep((m - 0.8) / 0.3) * bump
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle::*;

    /// Estes Alpha III-like: 18.7 mm body, 0.3 m long (nose 0.1 m), 3 trapezoidal fins.
    fn alpha_like() -> Vehicle {
        let r = 0.0093;
        Vehicle {
            reference_diameter: 2.0 * r,
            body: vec![
                BodySegment { station: 0.0, length: 0.07, fore_radius: 0.0, aft_radius: r, kind: BodyKind::Nose(NoseShape::TangentOgive), roughness: 20e-6 },
                BodySegment { station: 0.07, length: 0.26, fore_radius: r, aft_radius: r, kind: BodyKind::Tube, roughness: 20e-6 },
            ],
            fin_sets: vec![FinSet {
                count: 3,
                station: 0.26,
                body_radius: r,
                root_chord: 0.07,
                tip_chord: 0.03,
                span: 0.045,
                sweep: 0.03,
                thickness: 0.0016,
                cross_section: FinCrossSection::Rounded,
                cant: 0.0,
                roughness: 20e-6,
                planform: FinPlanform::Trapezoidal,
            }],
            protuberances: vec![Protuberance::LaunchLug { station: 0.15, length: 0.02, outer_diameter: 0.005, inner_diameter: 0.004 }],
            stages: vec![],
        }
    }

    #[test]
    fn nose_cna_is_two_and_cp_matches_barrowman() {
        for (shape, cp_frac, tol) in [(NoseShape::Conical, 2.0 / 3.0, 1e-3), (NoseShape::TangentOgive, 0.466, 0.02)] {
            let r = 0.03;
            let v = Vehicle {
                reference_diameter: 2.0 * r,
                body: vec![BodySegment { station: 0.0, length: 0.18, fore_radius: 0.0, aft_radius: r, kind: BodyKind::Nose(shape), roughness: 0.0 }],
                fin_sets: vec![],
                protuberances: vec![],
                stages: vec![],
            };
            let st = Aero::new(&v).stability(0.3, 0.0);
            assert!((st.cn_alpha - 2.0).abs() < 1e-9);
            assert!((st.cp / 0.18 - cp_frac).abs() < tol, "{shape:?} {}", st.cp / 0.18);
        }
    }

    #[test]
    fn transition_cp_matches_barrowman_formula() {
        // Frustum with R_f/R_a = 0.5: x_cp = L/3 (1 + (1 - 0.5)/(1 - 0.25)) from the fore face.
        let v = Vehicle {
            reference_diameter: 0.06,
            body: vec![BodySegment { station: 0.0, length: 0.1, fore_radius: 0.015, aft_radius: 0.03, kind: BodyKind::Transition, roughness: 0.0 }],
            fin_sets: vec![],
            protuberances: vec![],
            stages: vec![],
        };
        let st = Aero::new(&v).stability(0.3, 0.0);
        let expect = 0.1 / 3.0 * (1.0 + 0.5 / 0.75);
        assert!((st.cp - expect).abs() < 1e-4, "{} vs {expect}", st.cp);
        assert!((st.cn_alpha - 2.0 * (1.0 - 0.25)).abs() < 1e-9);
    }

    #[test]
    fn stock_rocket_is_stable_with_cp_behind_cg() {
        let a = Aero::new(&alpha_like());
        let st = a.stability(0.1, 0.0);
        // CG roughly 0.8 calibers... place it at 0.21 m (about 55% of length).
        let margin = (st.cp - 0.21) / 0.0186;
        assert!((1.0..3.5).contains(&margin), "margin {margin} cp {}", st.cp);
        assert!(st.cn_alpha > 2.0 && st.cn_alpha < 60.0, "{}", st.cn_alpha);
        // CP moves forward with angle of attack? Body lift acts ahead of the fins' CP.
        let st15 = a.stability(0.1, 15f64.to_radians());
        assert!(st15.cn_alpha > st.cn_alpha);
    }

    /// 3:1 ogive, 10 cal body, four trapezoidal fins on a 50 mm airframe.
    fn typical() -> Vehicle {
        let r = 0.025;
        let mut v = alpha_like();
        v.reference_diameter = 2.0 * r;
        v.body = vec![
            BodySegment { station: 0.0, length: 0.15, fore_radius: 0.0, aft_radius: r, kind: BodyKind::Nose(NoseShape::TangentOgive), roughness: 10e-6 },
            BodySegment { station: 0.15, length: 0.5, fore_radius: r, aft_radius: r, kind: BodyKind::Tube, roughness: 10e-6 },
        ];
        v.fin_sets = vec![FinSet {
            count: 4,
            station: 0.55,
            body_radius: r,
            root_chord: 0.1,
            tip_chord: 0.05,
            span: 0.06,
            sweep: 0.04,
            thickness: 0.002,
            cross_section: FinCrossSection::Rounded,
            cant: 0.0,
            roughness: 10e-6,
            planform: FinPlanform::Trapezoidal,
        }];
        v.protuberances = vec![];
        v
    }

    #[test]
    fn drag_levels_and_transonic_peak() {
        let a = Aero::new(&typical());
        let nu = 1.46e-5;
        let d = a.drag(0.3, 0.3 * 340.0 / nu, false);
        assert!((0.3..0.7).contains(&d.total), "{d:?}");
        assert!((d.friction + d.pressure + d.base + d.fins + d.protuberances - d.total).abs() < 1e-12);
        let cd = |m: f64| a.drag(m, m * 340.0 / nu, false).total;
        let peak = (80..=140).map(|i| cd(i as f64 / 100.0)).fold(0.0, f64::max);
        assert!(peak > cd(0.3) * 1.2 && peak > cd(2.5), "peak {peak} sub {} sup {}", cd(0.3), cd(2.5));
        let peak_m = (50..=300).max_by(|&i, &j| cd(i as f64 / 100.0).total_cmp(&cd(j as f64 / 100.0))).unwrap() as f64 / 100.0;
        assert!((0.9..1.3).contains(&peak_m), "peak at {peak_m}");
        // Motor exhaust removes base drag.
        let mut v = alpha_like();
        v.stages.push(StageMass {
            dry: MassProps { mass: 0.05, cg: 0.2, i_long: 0.0 },
            fore_station: 0.0,
            aft_station: 0.33,
            motor: Some(StageMotor {
                curve: ThrustCurve { points: vec![(0.0, 5.0), (1.0, 5.0)], propellant_mass: 0.01, total_mass: 0.02, length: 0.07, diameter: 0.018 },
                mount_aft_station: 0.33,
                ignition: Ignition::Launch,
                ejection_delay: None,
            }),
        });
        let a2 = Aero::new(&v);
        assert!(a2.drag(0.3, 1e6, true).base < a2.drag(0.3, 1e6, false).base);
    }

    #[test]
    fn fin_set_lift_compares_with_barrowman() {
        // Barrowman: CNa = 4N (s/d)^2 / (1 + sqrt(1 + (2 lm/(Cr+Ct))^2)) * K_fb, incompressible.
        let v = alpha_like();
        let f = &v.fin_sets[0];
        let d = v.reference_diameter;
        let lm = (f.span * f.span + (f.sweep + 0.5 * (f.tip_chord - f.root_chord)).powi(2)).sqrt();
        let n = 3.0;
        let kfb = 1.0 + f.body_radius / (f.span + f.body_radius);
        let expect = 4.0 * n * (f.span / d).powi(2) / (1.0 + (1.0 + (2.0 * lm / (f.root_chord + f.tip_chord)).powi(2)).sqrt()) * kfb;
        let st = Aero::new(&v).stability(0.0, 0.0);
        let fins = st.components.iter().find(|c| c.0 == "Fins").unwrap();
        assert!((fins.1 - expect).abs() / expect < 1e-9, "{} vs {expect}", fins.1);
    }
}
