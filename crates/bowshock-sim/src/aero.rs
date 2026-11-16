//! Barrowman aerodynamics: normal force, centre of pressure and drag build-up.
//!
//! * Normal force slope and CP follow Barrowman (1967) for noses, transitions and fins, with
//!   the extensions of Niskanen (2009, *Development of an Open Source model rocket simulation
//!   software*, ch. 3): body lift `1.1 A_plan/A_ref sin^2(alpha)/alpha` acting at the
//!   planform centroid (so the CP moves with angle of attack), Diederich fin lift with
//!   compressibility correction, the body-fin interference factor `K_fb = 1 + r/(s + r)`, each
//!   fin's lift scaled by `sin^2` of its roll angle from the crossflow ([`CROSSFLOW_ROLL`]), and
//!   the fin-fin interference loss of 5 or more fins on one body ([`fin_fin_factor`]).
//! * Drag follows Niskanen (2009, section 3.4; after Hoerner, *Fluid-Dynamic Drag*, 1965): flat-plate
//!   skin friction (Schlichting) with the laminar/turbulent transition rule, roughness limit and
//!   compressibility corrections; nose/shoulder pressure drag interpolated from the NASA TR R-100
//!   free-flight tables per profile family and extrapolated to the actual fineness ratio;
//!   stagnation and base drag (Fleeman); fin leading/trailing-edge drag per cross-section;
//!   launch lug drag (Darcy-Weisbach pipe flow, Swamee & Jain 1976) and rail button drag
//!   (Gowen & Perkins, NACA TN 2960); and an angle-of-attack multiplier on axial drag that is our
//!   own engineering fit (see [`axial_drag_multiplier`]).
//!
//! Coefficients are referenced to the vehicle reference area; stations are from the nose tip.

use crate::atmosphere::Atmosphere;
use crate::drag_table::{DragTable, DragTablePoint};
use crate::tube_fins::TubeFinCalc;
use crate::vehicle::{BodyKind, BodySegment, FinCrossSection, FinPlanform, FinSet, NoseShape, Protuberance, Vehicle};
use std::f64::consts::PI;
use std::sync::Arc;

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

/// Normal force of the whole vehicle at one flight condition (see [`Aero::normal_force`]).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NormalForce {
    /// Total normal force coefficient slope, `CN/alpha`, per radian (as [`Stability::cn_alpha`]).
    pub cn_alpha: f64,
    /// Centre of pressure station from the tip (m).
    pub cp: f64,
    /// `sum CN_alpha_i x_i` over the components (m).
    s1: f64,
    /// `sum CN_alpha_i x_i^2` over the components (m^2).
    s2: f64,
}

impl NormalForce {
    /// `sum CN_alpha_i (x_i - cg)^2` (m^2): each lifting component, at its own CP station `x_i`, sees the angle of attack
    /// `w (x_i - cg)/V` from a pitch or yaw rate `w` about the centre of gravity at station `cg`, so the damping moment
    /// is `-1/2 rho V A_ref (sum CN_alpha_i (x_i - cg)^2) w` (strip theory: Barrowman 1967; Mandell, Caporaso & Bengen,
    /// *Topics in Advanced Model Rocketry*, 1973, the fin pitch damping term).
    pub fn damping_sum(&self, cg: f64) -> f64 {
        (self.s2 - 2.0 * cg * self.s1 + cg * cg * self.cn_alpha).max(0.0)
    }
}

/// Roll moment coefficients of the fins at one Mach number (see [`Aero::roll`]): the moment about the nose axis, right-handed
/// about the nose direction (clockwise seen from behind, the sense in which a fin's roll angle increases), is
/// `q (forcing + damping w / V)` for a roll rate `w` in the same sense and air speed `V`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RollAero {
    /// Cant forcing `sum N K CN_alpha_1 A_ref cant arm` (m^3, `q` times it is the moment in N m).
    pub forcing: f64,
    /// Roll damping `-sum N K CN_alpha_1 A_ref arm2` (m^4, at most 0): each fin section at distance `r` from the axis meets
    /// the angle of attack `w r / V` (strip theory, Barrowman 1967).
    pub damping: f64,
}

/// Crossflow drag coefficient of the body planform (Hoerner, *Fluid-Dynamic Drag*, 1965): the coefficient of the Galejs body
/// lift in [`Aero::stability`] and of the crossflow damping in [`Aero::crossflow_damping`].
pub const BODY_CROSSFLOW_CD: f64 = 1.1;

/// Drag coefficient (reference-area based) broken into the categories the UI stacks.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DragBreakdown {
    /// Body skin friction.
    pub friction: f64,
    /// Body pressure drag (nose/transition wave drag, stagnation on blunt faces).
    pub pressure: f64,
    /// Base drag (body bases; fin trailing edges are counted in `fins`).
    pub base: f64,
    /// Fins: skin friction plus leading/trailing-edge pressure drag.
    pub fins: f64,
    /// Launch lugs and rail buttons.
    pub protuberances: f64,
    /// Sum of all of the above.
    pub total: f64,
}

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
struct Table(Vec<(f64, f64)>);

impl Table {
    fn at(&self, x: f64) -> f64 {
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

fn static_table(p: Pts) -> Table {
    Table(p.to_vec())
}

/// Stagnation-pressure drag coefficient of a flat face: the isentropic stagnation pressure ratio
/// (series below Mach 1, Rayleigh pitot form above; Niskanen 2009 eq. B.1-B.2) with Hoerner's 0.85 factor.
fn stagnation_cd(m: f64) -> f64 {
    let pressure = if m <= 1.0 { 1.0 + m * m / 4.0 + m.powi(4) / 40.0 } else { 1.84 - 0.76 / (m * m) + 0.166 / m.powi(4) + 0.035 / m.powi(6) };
    0.85 * pressure
}

/// Base drag coefficient on the base area (Fleeman, *Tactical Missile Design*; Niskanen 2009 eq. 3.94).
fn base_cd(m: f64) -> f64 {
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
fn pressure_table(shape: Option<NoseShape>, fineness: f64, sinphi: f64) -> Table {
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
// Skin friction: Blasius laminar and Prandtl-Schlichting turbulent flat-plate laws (Schlichting; Niskanen 2009 section 3.4.2).
// ---------------------------------------------------------------------------------------------

/// Flat-plate skin friction coefficient at length-Reynolds number `re` including the
/// compressibility correction. With `perfect_finish` the laminar-to-turbulent transition is
/// modelled (laminar below Re 5.39e5, `1/(1.5 ln Re - 5.6)^2 - 1700/Re` above) and the
/// compressibility corrections apply only for Re > 1e6; otherwise the fully turbulent formula is used.
fn skin_cf(re: f64, mach: f64, perfect: bool) -> f64 {
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
fn roughness_correction(mach: f64) -> f64 {
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

// ---------------------------------------------------------------------------------------------
// Geometry preprocessing.
// ---------------------------------------------------------------------------------------------

/// Nose/shoulder pressure-drag model of one body segment.
#[derive(Clone, Debug)]
enum Pressure {
    None,
    /// `stagnation_cd * area / A_ref`.
    Stagnation {
        area: f64,
    },
    /// `base_cd * area * scale / A_ref` (boat-tails).
    Base {
        area: f64,
        scale: f64,
    },
    /// `table(M) * area / A_ref`.
    Table {
        table: Table,
        area: f64,
    },
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
    pressure: Pressure,
    /// Forward-facing exposed annulus (step up in radius) area (m^2), stagnation drag.
    front_face: f64,
    /// Backward-facing exposed annulus area (m^2), base drag.
    back_face: f64,
    is_last: bool,
    /// `(station of strip centre, planform area of strip)` over the segment, for the crossflow damping.
    strips: Vec<(f64, f64)>,
}

#[derive(Clone, Debug)]
struct FinCalc {
    name: String,
    n: f64,
    area: f64,
    span: f64,
    ar: f64,
    cos_mid: f64,
    cos_lead: f64,
    mac_lead: f64,
    /// Lift multiplier of the set on top of one fin's slope: `sum sin^2(theta - phi_i)` over the
    /// fins (`N/2` for 3+ evenly spaced fins) times `K_fb` times the fin-fin factor.
    lift: f64,
    /// Root chord span `(leading edge, trailing edge)` stations, for the fin-fin interference count.
    root: (f64, f64),
    parent: u64,
    /// Fins on each body the set stands on.
    per_body: f64,
    /// Mean aerodynamic chord `∫c² dy / ∫c dy`.
    mac: f64,
    thickness: f64,
    roughness: f64,
    cross: FinCrossSection,
    /// `K_fb` times the fin-fin factor: the lift multiplier of one fin that sees the flow face-on (the roll moments).
    unit_lift: f64,
    /// Cant angle (rad).
    cant: f64,
    /// Arm from the body axis of the fin's lift (chord-weighted mean span station, m).
    roll_arm: f64,
    /// Chord-weighted mean-square arm from the body axis (m^2).
    roll_arm2: f64,
    /// Whether the set stands on the core body (pod fins are not on the rocket's axis).
    on_core: bool,
}

#[derive(Clone, Debug)]
enum ProtCalc {
    Lug { inner_diameter: f64, inner_area: f64, frontal: f64, length: f64, roughness: f64 },
    Button { station: f64, height: f64, diameter: f64 },
}

/// Surface roughness assumed for launch-lug bores (regular paint, 60 um; Niskanen 2009 table 3.2).
const LUG_ROUGHNESS: f64 = 60e-6;

/// Aerodynamic model of one vehicle configuration.
#[derive(Clone, Debug)]
pub struct Aero {
    d_ref: f64,
    a_ref: f64,
    length: f64,
    body_length: f64,
    max_diameter: f64,
    motor_exit_area: f64,
    perfect_finish: bool,
    bodies: Vec<BodyCalc>,
    fins: Vec<FinCalc>,
    tube_fins: Vec<TubeFinCalc>,
    prots: Vec<ProtCalc>,
    /// `(roughness m, wetted area m^2)` of the body, grouped so each skin-friction value is evaluated once.
    friction_groups: Vec<(f64, f64)>,
    /// CFD drag table flown instead of the build-up, which then supplies the corrections.
    table: Option<Arc<DragTable>>,
}

impl Aero {
    /// Precomputes geometry-dependent quantities for `v`. Surface roughness uses the
    /// rough-surface model (fully turbulent, roughness-limited; the default); see [`Aero::perfect_finish`].
    pub fn new(v: &Vehicle) -> Self {
        let a_ref = v.reference_area().max(1e-12);
        let length = v.length();
        let mut bodies = Vec::with_capacity(v.body.len());
        for (i, s) in v.body.iter().enumerate() {
            let prev_aft = if i == 0 { 0.0 } else { v.body[i - 1].aft_radius };
            let next_fore = v.body.get(i + 1).map(|n| n.fore_radius);
            bodies.push(body_calc(s, prev_aft, next_fore, a_ref));
        }
        let max_r = v.body.iter().map(|s| s.fore_radius.max(s.aft_radius)).fold(0.0, f64::max);
        let min_x = v.body.iter().map(|s| s.station).fold(f64::INFINITY, f64::min);
        let body_length = if v.body.is_empty() { 0.0 } else { length - min_x + 0.0001 };
        let mut fins: Vec<FinCalc> = v
            .fin_sets
            .iter()
            .enumerate()
            .map(|(i, f)| fin_calc(f, if v.fin_sets.len() > 1 { format!("Fin set {}", i + 1) } else { "Fins".to_string() }))
            .collect();
        // Fin-fin interference: the fins of every set on the same body whose root chords overlap act together.
        let together: Vec<f64> = fins
            .iter()
            .map(|f| {
                fins.iter().filter(|g| g.parent == f.parent && g.root.0 < f.root.1 && g.root.1 > f.root.0).map(|g| g.per_body).sum::<f64>().max(f.per_body)
            })
            .collect();
        for (f, n) in fins.iter_mut().zip(together) {
            let ff = fin_fin_factor(n.round() as u32);
            f.lift *= ff;
            f.unit_lift *= ff;
        }
        let prots = v
            .protuberances
            .iter()
            .map(|p| match *p {
                Protuberance::LaunchLug { length, outer_diameter, inner_diameter, .. } => ProtCalc::Lug {
                    inner_diameter,
                    inner_area: 0.25 * PI * inner_diameter * inner_diameter,
                    frontal: 0.25 * PI * (outer_diameter * outer_diameter - inner_diameter * inner_diameter).max(0.0),
                    length,
                    roughness: LUG_ROUGHNESS,
                },
                Protuberance::RailButton { station, height, diameter } => ProtCalc::Button { station, height, diameter },
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
            body_length,
            max_diameter: 2.0 * max_r,
            motor_exit_area: 0.25 * PI * motor_d * motor_d,
            perfect_finish: false,
            bodies,
            fins,
            tube_fins: v.tube_fins.iter().map(TubeFinCalc::new).collect(),
            prots,
            friction_groups,
            table: None,
        }
    }

    /// Selects the smooth-surface skin friction (laminar-to-turbulent transition,
    /// roughness only limiting above Re 1e6) instead of the default fully turbulent,
    /// roughness-limited model.
    pub fn perfect_finish(mut self, on: bool) -> Self {
        self.perfect_finish = on;
        self
    }

    /// Reference diameter (m).
    pub fn reference_diameter(&self) -> f64 {
        self.d_ref
    }

    /// Normal-force slope and CP at `mach` and angle of attack `alpha` (rad).
    ///
    /// The slope includes the nonlinear Galejs body lift evaluated at `alpha`, so
    /// `cn_alpha * alpha` is the normal force coefficient `CN` there.
    pub fn stability(&self, mach: f64, alpha: f64) -> Stability {
        let mut comps: Vec<(String, f64, f64)> = Vec::new();
        self.each_component(mach, alpha, |name, cna, cp| comps.push((name(), cna, cp)));
        let cn_alpha: f64 = comps.iter().map(|c| c.1).sum();
        let cp = if cn_alpha.abs() > 1e-9 { comps.iter().map(|c| c.1 * c.2).sum::<f64>() / cn_alpha } else { 0.5 * self.length };
        Stability { cn_alpha, cp, components: comps }
    }

    /// Calls `f(name, CN_alpha per rad, CP station)` for every component with lift at `mach` and `alpha`; the name is only
    /// built when `f` asks for it, so the flight code can sum the components without allocating.
    fn each_component(&self, mach: f64, alpha: f64, mut f: impl FnMut(&dyn Fn() -> String, f64, f64)) {
        let sin_a = alpha.abs().sin();
        let sinc = if alpha.abs() > 1e-9 { sin_a / alpha.abs() } else { 1.0 };
        for b in &self.bodies {
            let mut cna = b.cna * sinc;
            let mut moment = cna * b.cp;
            if alpha.abs() > 1e-9 {
                // Galejs body lift: CN = 1.1 A_plan / A_ref sin^2(alpha), at the planform centroid.
                let extra = 1.1 * b.plan_area / self.a_ref * sin_a * sin_a / alpha.abs();
                cna += extra;
                moment += extra * b.plan_cp;
            }
            if cna.abs() > 1e-12 {
                f(&|| b.name.clone(), cna, moment / cna);
            }
        }
        for fin in &self.fins {
            f(&|| fin.name.clone(), fin.cna(mach, alpha, self.a_ref), fin.cp(mach));
        }
        for (i, t) in self.tube_fins.iter().enumerate() {
            let cna = t.cna(alpha, self.a_ref);
            if cna.abs() > 1e-12 {
                f(&|| if self.tube_fins.len() > 1 { format!("Tube fins {}", i + 1) } else { "Tube fins".to_string() }, cna, t.cp(mach));
            }
        }
    }

    /// Normal-force slope and CP at `mach` and angle of attack `alpha` (rad) together with the sums behind the pitch damping
    /// moment ([`NormalForce::damping_sum`]), without the per-component list of [`Aero::stability`] (no allocation): what the
    /// rigid-body flight model evaluates every step.
    pub fn normal_force(&self, mach: f64, alpha: f64) -> NormalForce {
        let (mut s0, mut s1, mut s2) = (0.0, 0.0, 0.0);
        self.each_component(mach, alpha, |_, cna, cp| {
            s0 += cna;
            s1 += cna * cp;
            s2 += cna * cp * cp;
        });
        NormalForce { cn_alpha: s0, cp: if s0.abs() > 1e-9 { s1 / s0 } else { 0.5 * self.length }, s1, s2 }
    }

    /// Static margin in calibers (positive = stable) for centre of gravity station `cg`.
    pub fn margin_cal(&self, mach: f64, alpha: f64, cg: f64) -> f64 {
        (self.stability(mach, alpha).cp - cg) / self.d_ref
    }

    /// Body crossflow damping factor about a centre of gravity at station `cg`: `sum A_strip |x - cg|^3` (m^5) over strips of
    /// the body planform. A body turning at rate `w` about its CG meets a crossflow `w (x - cg)` at each station and the
    /// Hoerner crossflow drag (coefficient [`BODY_CROSSFLOW_CD`], the one behind the body lift of [`Aero::stability`]) gives the
    /// restoring moment `-1/2 rho BODY_CROSSFLOW_CD K w |w|` (Zipfel 2014, *Modeling and Simulation of Aerospace Vehicle
    /// Dynamics*, the strip-theory damping of slender bodies).
    pub fn crossflow_damping(&self, cg: f64) -> f64 {
        self.bodies.iter().flat_map(|b| b.strips.iter()).map(|&(x, area)| area * (x - cg).abs().powi(3)).sum()
    }

    /// Fin roll forcing and damping at `mach` (see [`RollAero`]): the fins of the core body only.
    pub fn roll(&self, mach: f64) -> RollAero {
        let (mut forcing, mut damping) = (0.0, 0.0);
        for f in self.fins.iter().filter(|f| f.on_core) {
            let lift = f.n * f.unit_lift * f.cna1(mach, 0.0, self.a_ref) * self.a_ref;
            forcing += lift * f.cant * f.roll_arm;
            damping -= lift * f.roll_arm2;
        }
        RollAero { forcing, damping }
    }

    /// Zero-angle-of-attack drag at `mach` with `re_per_m = v/nu` (Reynolds number per metre).
    /// With `motor_on` the base area covered by the motor exhaust carries no base drag.
    pub fn drag(&self, mach: f64, re_per_m: f64, motor_on: bool) -> DragBreakdown {
        match &self.table {
            None => self.buildup(mach, re_per_m, motor_on),
            Some(table) => self.tabulated(table, mach, re_per_m, motor_on),
        }
    }

    /// This model flying on `table`: its parts at the table's Reynolds number, corrected by the build-up for the
    /// Reynolds number and motor exhaust of the actual condition.
    pub fn with_table(mut self, table: DragTable) -> Aero {
        self.table = Some(Arc::new(table));
        self
    }

    /// Drag from the CFD table; see [`Aero::with_table`].
    fn tabulated(&self, table: &DragTable, mach: f64, re_per_m: f64, motor_on: bool) -> DragBreakdown {
        let m = mach.max(0.0);
        let air = Atmosphere::standard().at_msl(table.altitude);
        let re_tab = |m: f64| m * air.speed_of_sound * air.density / air.viscosity;
        let now = self.buildup(m, re_per_m, motor_on);
        let tab = self.buildup(m, re_tab(m), false);
        let t = table.inside(m).unwrap_or_else(|| {
            let (first, last) = (&table.points[0], &table.points[table.points.len() - 1]);
            let end = if m > last.mach { last } else { first };
            let at_end = self.buildup(end.mach, re_tab(end.mach), false);
            let scale = |end: f64, here: f64, there: f64| {
                let r = here / there;
                if there.abs() < 1e-12 || !r.is_finite() {
                    end
                } else {
                    end * r
                }
            };
            DragTablePoint {
                mach: m,
                friction: scale(end.friction, tab.friction, at_end.friction),
                pressure: scale(end.pressure, tab.pressure, at_end.pressure),
                base: scale(end.base, tab.base, at_end.base),
                fins: scale(end.fins, tab.fins, at_end.fins),
                other: scale(end.other, tab.protuberances, at_end.protuberances),
            }
        });
        let friction = (t.friction + now.friction - tab.friction).max(0.0);
        let pressure = (t.pressure + now.pressure - tab.pressure).max(0.0);
        let fins = (t.fins + now.fins - tab.fins).max(0.0);
        let protuberances = (t.other + now.protuberances - tab.protuberances).max(0.0);
        let base = if tab.base < 1e-12 { t.base } else { t.base * now.base / tab.base }.max(0.0);
        DragBreakdown { friction, pressure, base, fins, protuberances, total: friction + pressure + base + fins + protuberances }
    }

    /// The Barrowman/OpenRocket build-up.
    fn buildup(&self, mach: f64, re_per_m: f64, motor_on: bool) -> DragBreakdown {
        let m = mach.max(0.0);
        let ar = self.a_ref;
        let re_per_m = re_per_m.max(0.0);
        let re_len = (re_per_m * self.length).max(1.0);
        let stag = stagnation_cd(m);
        let base = base_cd(m);

        // Skin friction: one flat-plate Cf from the whole-rocket length, floored per finish by the
        // roughness-limited value 0.032 (eps/L)^0.2 (Schlichting; Niskanen 2009 eq. 3.80).
        let cf = skin_cf(re_len, m, self.perfect_finish);
        let rough_corr = roughness_correction(m);
        let comp_cf = |eps: f64| {
            let limited = 0.032 * (eps / self.length.max(1e-9)).powf(0.2) * rough_corr;
            if self.perfect_finish {
                if re_len > 1e6 && limited > cf {
                    limited
                } else {
                    cf
                }
            } else {
                cf.max(limited)
            }
        };
        // Cylindrical-body correction 1 + 1/(2 fB), fB = body length / max diameter (tech doc eq. 3.85).
        let f_b = self.body_length / self.max_diameter.max(1e-9);
        let mut friction: f64 = self.friction_groups.iter().map(|&(eps, wet)| comp_cf(eps) * wet).sum();
        friction *= (1.0 + 1.0 / (2.0 * f_b.max(1e-9))) / ar;

        // Pressure drag of bodies: nose/shoulder tables plus stagnation on forward-facing steps.
        let mut pressure = 0.0;
        for b in &self.bodies {
            pressure += match &b.pressure {
                Pressure::None => 0.0,
                Pressure::Stagnation { area } => stag * area / ar,
                Pressure::Base { area, scale } => base * area * scale / ar,
                Pressure::Table { table, area } => table.at(m) * area / ar,
            };
            pressure += stag * b.front_face / ar;
        }

        // Base drag; exhaust covers the motor exit area of the terminal base.
        let mut base_total = 0.0;
        for b in &self.bodies {
            let mut area = b.back_face;
            if motor_on && b.is_last {
                area = (area - self.motor_exit_area).max(0.0);
            }
            base_total += base * area / ar;
        }

        let mut fins = 0.0;
        for f in &self.fins {
            fins += f.drag(m, comp_cf(f.roughness), stag, base, ar);
        }
        // Tube fins: friction on the outer surface and the through-flow pressure drag, no base drag (`crate::tube_fins`).
        for t in &self.tube_fins {
            fins += t.friction_cd(comp_cf(t.roughness()), ar) + t.pressure_cd(re_per_m, stag, base, ar);
        }

        let mut prot = 0.0;
        for p in &self.prots {
            prot += match *p {
                ProtCalc::Lug { inner_diameter, inner_area, frontal, length, roughness } => {
                    lug_drag(re_per_m, inner_diameter, inner_area, frontal, length, roughness, stag, base, ar)
                }
                ProtCalc::Button { station, height, diameter } => button_drag(m, re_per_m, station, height, diameter, stag, ar),
            };
        }

        let total = friction + pressure + base_total + fins + prot;
        DragBreakdown { friction, pressure, base: base_total, fins, protuberances: prot, total }
    }

    /// Drag at angle of attack `alpha`: the zero-alpha build-up scaled by the axial-drag
    /// multiplier (1 at 0 deg, 1.3 at 17 deg, falling to 0 at 90 deg), which is the drag
    /// component along the rocket axis.
    pub fn drag_at_alpha(&self, mach: f64, re_per_m: f64, motor_on: bool, alpha: f64) -> DragBreakdown {
        let d = self.drag(mach, re_per_m, motor_on);
        let k = axial_drag_multiplier(alpha);
        DragBreakdown {
            friction: d.friction * k,
            pressure: d.pressure * k,
            base: d.base * k,
            fins: d.fins * k,
            protuberances: d.protuberances * k,
            total: d.total * k,
        }
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

/// Rail button drag: cylinder drag (Gowen & Perkins, NACA TN 2960 fig. 7) at the
/// mean Mach number inside the boundary layer at the button's station.
fn button_drag(mach: f64, re_per_m: f64, station: f64, height: f64, diameter: f64, stag: f64, a_ref: f64) -> f64 {
    const DOMAIN: [f64; 12] = [0.0, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 1.0, 1.6, 2.0, 2.8, 100.0];
    const RANGE: [f64; 12] = [1.2, 1.22, 1.25, 1.3, 1.4, 1.5, 1.6, 2.1, 1.5, 1.45, 1.33, 1.33];
    let ref_area = height * diameter;
    let mul = if mach > 1e-9 && re_per_m > 0.0 && station > 0.0 {
        let rex = re_per_m * station;
        let del = 0.37 * station / rex.powf(0.2);
        let m_eff = if height > del { (height - 0.5 * del) * mach / height } else { height / 2.0 / del * mach };
        let i = DOMAIN.partition_point(|&d| d <= m_eff).clamp(1, DOMAIN.len() - 1);
        let t = ((m_eff - DOMAIN[i - 1]) / (DOMAIN[i] - DOMAIN[i - 1])).clamp(0.0, 1.0);
        let cd = RANGE[i - 1] + (RANGE[i] - RANGE[i - 1]) * t;
        cd * m_eff * m_eff / (mach * mach)
    } else {
        8.786_395_072_609_939E-4
    };
    mul * stag * ref_area / a_ref
}

/// Numerically integrates a body segment's geometry and selects its pressure-drag model.
fn body_calc(s: &BodySegment, prev_aft: f64, next_fore: Option<f64>, a_ref: f64) -> BodyCalc {
    const N: usize = 64;
    let l = s.length.max(1e-12);
    let dx = l / N as f64;
    let (mut vol, mut plan, mut plan_m, mut wet) = (0.0, 0.0, 0.0, 0.0);
    let mut strips: Vec<(f64, f64)> = Vec::with_capacity(N);
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
        let r = s.radius_at(0.99 * s.length);
        let tangent_ogive = matches!(shape, Some(NoseShape::TangentOgive));
        let sinphi = if tangent_ogive { 0.0 } else { (r1 - r) / ((r1 - r).hypot(0.01 * s.length)) };
        Pressure::Table { table: pressure_table(shape, fineness, sinphi), area: frontal }
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

fn fin_calc(f: &FinSet, name: String) -> FinCalc {
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

/// Roll angle (rad) of the crossflow Bowshock assumes: the lateral wind lies along +Y, the angle
/// a fin at roll angle 0 points to (a fin's roll angle is measured from +Y towards +X, see
/// [`FinSet::angles`]). A 1- or 2-fin set at angle 0 therefore sees the flow edge-on and gives no
/// lift; turn the set by 90 deg for full lift. The aerodynamic model is axisymmetric: the rigid-body flight model rolls (fin
/// cant) but the crossflow azimuth of the fin lift stays fixed here, so the roll angle only enters through this constant.
pub const CROSSFLOW_ROLL: f64 = 0.0;

/// `sum sin^2(CROSSFLOW_ROLL - phi_i)` over the fins at roll angles `angles` (`count` fins evenly
/// spaced from 0 when empty): a fin's normal force is proportional to the square of the sine of the
/// angle between its plane and the crossflow (Niskanen 2009, §3.2.2). For 3 or more evenly spaced
/// fins it is `N/2` whatever the roll.
pub fn roll_factor(angles: &[f64], count: u32) -> f64 {
    let s = |phi: f64| (CROSSFLOW_ROLL - phi).sin().powi(2);
    if angles.is_empty() {
        let n = f64::from(count);
        (0..count).map(|i| s(2.0 * PI * f64::from(i) / n)).sum()
    } else {
        angles.iter().map(|&a| s(a)).sum()
    }
}

/// Lift efficiency of `n` fins acting together on one body (fin-fin interference; Niskanen 2009,
/// §3.2.2): no loss up to 4 fins, then 0.948, 0.913, 0.854 and 0.81 for 5..8 fins, and 0.75 beyond.
pub fn fin_fin_factor(n: u32) -> f64 {
    match n {
        0..=4 => 1.0,
        5 => 0.948,
        6 => 0.913,
        7 => 0.854,
        8 => 0.81,
        _ => 0.75,
    }
}

// Fin normal force and CP vs Mach: Diederich subsonic lift, second-order supersonic theory, and a
// transonic polynomial bridge (Niskanen 2009, §3.2.2 and §3.2.4).

const CNA_SUBSONIC: f64 = 0.9;
const CNA_SUPERSONIC: f64 = 1.5;
const GAMMA: f64 = 1.4;
/// Fin stall angle (20 deg): the angle-of-attack terms of the fin lift saturate there.
pub(crate) const FIN_STALL: f64 = 20.0 * PI / 180.0;

/// Supersonic coefficients `(K1, K2, K3)` of `CN = A (K1 a + K2 a^2 + K3 a^3)` per unit angle
/// (Busemann second-order thin-airfoil theory; Pitts, Nielsen & Kaattari, NACA Report 1307;
/// Niskanen 2009 §3.2.2), evaluated in closed form.
fn supersonic_k(mach: f64) -> (f64, f64, f64) {
    let m = mach.clamp(CNA_SUPERSONIC, 4.9);
    let b2 = m * m - 1.0;
    let beta = b2.sqrt();
    let k1 = 2.0 / beta;
    let k2 = ((GAMMA + 1.0) * m.powi(4) - 4.0 * b2) / (4.0 * b2 * b2);
    let k3 =
        ((GAMMA + 1.0) * m.powi(8) + (2.0 * GAMMA * GAMMA - 7.0 * GAMMA - 5.0) * m.powi(6) + 10.0 * (GAMMA + 1.0) * m.powi(4) + 8.0) / (6.0 * beta.powi(7));
    (k1, k2, k3)
}

const SUBSONIC_CP_POS: f64 = 0.25;
const SUPERSONIC_CP_OFFSET: f64 = 0.67;
const LOW_AR_CP_BRIDGE_START: f64 = (SUPERSONIC_CP_OFFSET - SUBSONIC_CP_POS) / (1.0 - 2.0 * SUBSONIC_CP_POS);

fn source_supersonic_cp(x: f64) -> f64 {
    (x - SUPERSONIC_CP_OFFSET) / (2.0 * x - 1.0)
}

fn source_supersonic_cp_gradient(x: f64) -> f64 {
    let d = 2.0 * x - 1.0;
    (2.0 * SUPERSONIC_CP_OFFSET - 1.0) / (d * d)
}

/// Supersonic CP as a fraction of the MAC from its leading edge, `(x - 0.67)/(2x - 1)` with
/// `x = AR beta` (Fleeman 2001; Niskanen 2009 §3.2.2). The published rule falls to the quarter
/// chord at `x = 0.84` and runs into a pole at `x = 1/2` below that, so for `x < 1` it is replaced
/// by a cubic Hermite from the quarter chord at `x = 0.84` to the rule's value and slope at `x = 1`
/// (an engineering bridge, not a published curve): low-aspect-ratio fins keep a CP between the
/// subsonic and supersonic limits.
fn supersonic_cp(x: f64) -> f64 {
    if x <= LOW_AR_CP_BRIDGE_START {
        return SUBSONIC_CP_POS;
    }
    if x >= 1.0 {
        return source_supersonic_cp(x);
    }
    let w = 1.0 - LOW_AR_CP_BRIDGE_START;
    let t = (x - LOW_AR_CP_BRIDGE_START) / w;
    let (t2, t3) = (t * t, t * t * t);
    (2.0 * t3 - 3.0 * t2 + 1.0) * SUBSONIC_CP_POS + (-2.0 * t3 + 3.0 * t2) * source_supersonic_cp(1.0) + (t3 - t2) * w * source_supersonic_cp_gradient(1.0)
}

fn supersonic_cp_gradient(x: f64) -> f64 {
    if x <= LOW_AR_CP_BRIDGE_START {
        return 0.0;
    }
    if x >= 1.0 {
        return source_supersonic_cp_gradient(x);
    }
    let w = 1.0 - LOW_AR_CP_BRIDGE_START;
    let t = (x - LOW_AR_CP_BRIDGE_START) / w;
    let t2 = t * t;
    let sb = 6.0 * t2 - 6.0 * t;
    (sb * SUBSONIC_CP_POS - sb * source_supersonic_cp(1.0)) / w + (3.0 * t2 - 2.0 * t) * source_supersonic_cp_gradient(1.0)
}

/// Fin CP fraction of the MAC: quarter chord up to Mach 0.5, a quintic in Mach up to Mach 2 that
/// meets the supersonic value and slope there (the transonic interpolation of Niskanen 2009
/// §3.2.2, with a monotone variant when the quintic would overshoot), then the supersonic rule.
pub(crate) fn fin_cp_fraction(mach: f64, ar: f64) -> f64 {
    if mach <= 0.5 {
        return SUBSONIC_CP_POS;
    }
    if mach >= 2.0 {
        return supersonic_cp(ar * (mach * mach - 1.0).sqrt());
    }
    let t = (mach - 0.5) / 1.5;
    let x2 = ar * 3f64.sqrt();
    let delta = supersonic_cp(x2) - SUBSONIC_CP_POS;
    if delta <= 0.0 {
        return SUBSONIC_CP_POS;
    }
    let slope = x2 * supersonic_cp_gradient(x2);
    let ratio = slope / delta;
    let (t2, t3) = (t * t, t * t * t);
    if ratio <= 5.0 / 3.0 {
        let (t4, t5) = (t3 * t, t3 * t * t);
        SUBSONIC_CP_POS
            + (10.0 * delta - 6.0 * slope) * t2
            + (-20.0 * delta + 14.0 * slope) * t3
            + (15.0 * delta - 11.0 * slope) * t4
            + (3.0 * slope - 4.0 * delta) * t5
    } else {
        // Monotone continuation of the limiting quintic (slope ratio 5/3), raised to ratio/(5/3).
        let limiting = t3 * (10.0 / 3.0 - 10.0 / 3.0 * t + t2);
        SUBSONIC_CP_POS + delta * limiting.powf(ratio / (5.0 / 3.0))
    }
}

impl FinCalc {
    /// Normal-force slope of one fin on the reference area (Niskanen 2009 §3.2.2):
    /// Diederich below Mach 0.9, `A (K1 + K2 a + K3 a^2)/A_ref` above Mach 1.5, and a quartic
    /// through value and slope at both ends (zero curvature at 0.9) in between.
    fn cna1(&self, mach: f64, alpha: f64, a_ref: f64) -> f64 {
        if self.area < 1e-12 || self.span < 1e-9 || self.cos_mid < 1e-9 {
            return 0.0;
        }
        let alpha = alpha.abs().min(PI - alpha.abs()).min(FIN_STALL);
        let q = self.span * self.span / (self.area * self.cos_mid);
        let sub = |m: f64| 2.0 * PI * self.span * self.span / (1.0 + (1.0 + (1.0 - m * m) * q * q).sqrt()) / a_ref;
        let sup = |m: f64, a: f64| {
            let (k1, k2, k3) = supersonic_k(m);
            self.area * (k1 + k2 * a + k3 * a * a) / a_ref
        };
        if mach <= CNA_SUBSONIC {
            return sub(mach);
        }
        if mach >= CNA_SUPERSONIC {
            return sup(mach, alpha);
        }
        let sq = (1.0 + (1.0 - CNA_SUBSONIC * CNA_SUBSONIC) * q * q).sqrt();
        let sub_v = sub(CNA_SUBSONIC);
        let sub_d = 2.0 * CNA_SUBSONIC * PI * self.span.powi(6) / ((self.area * self.cos_mid).powi(2) * a_ref * sq * (1.0 + sq).powi(2));
        let sup_v = sup(CNA_SUPERSONIC, alpha);
        let sup_d = -self.area / a_ref * 2.0 * CNA_SUPERSONIC / (CNA_SUPERSONIC * CNA_SUPERSONIC - 1.0).powf(1.5);
        // Quartic p(t), t = M - 0.9: p(0) = sub_v, p'(0) = sub_d, p''(0) = 0, p(h) = sup_v, p'(h) = sup_d.
        let h = CNA_SUPERSONIC - CNA_SUBSONIC;
        let t = mach - CNA_SUBSONIC;
        let r1 = sup_v - sub_v - sub_d * h;
        let r2 = sup_d - sub_d;
        let a4 = (r2 - 3.0 * r1 / h) / h.powi(3);
        let a3 = (r1 - a4 * h.powi(4)) / h.powi(3);
        sub_v + sub_d * t + a3 * t.powi(3) + a4 * t.powi(4)
    }

    /// Total CN slope of the fin set: one fin's slope times the set's lift multiplier (roll
    /// dependence, body-fin interference `K_fb = 1 + r/(s + r)` and fin-fin interference).
    fn cna(&self, mach: f64, alpha: f64, a_ref: f64) -> f64 {
        self.cna1(mach, alpha, a_ref) * self.lift
    }

    /// CP station of the fin set at `mach`: MAC leading edge plus the Mach-dependent fraction of the MAC.
    fn cp(&self, mach: f64) -> f64 {
        self.mac_lead + fin_cp_fraction(mach, self.ar) * self.mac
    }

    /// Fin drag (Niskanen 2009 section 3.4.4): friction `Cf (1 + 2 t/MAC) 2 A`, leading-edge pressure
    /// drag (stagnation for square edges, the rounded-edge fit for rounded/airfoil) scaled by
    /// `cos^2` of the leading-edge sweep, and trailing-edge base drag (full for square, half for
    /// rounded, none for airfoil), all times the fin count.
    fn drag(&self, mach: f64, cf: f64, stag: f64, base: f64, a_ref: f64) -> f64 {
        let friction = cf * (1.0 + 2.0 * self.thickness / self.mac.max(1e-9)) * 2.0 * self.area * self.n / a_ref;
        let a_edge = self.n * self.span * self.thickness / a_ref;
        let le_cd = match self.cross {
            FinCrossSection::Square => stag,
            FinCrossSection::Rounded | FinCrossSection::Airfoil => round_edge_cd(mach),
        };
        let te_cd = match self.cross {
            FinCrossSection::Square => base,
            FinCrossSection::Rounded => 0.5 * base,
            FinCrossSection::Airfoil => 0.0,
        };
        friction + (le_cd * self.cos_lead * self.cos_lead + te_cd) * a_edge
    }
}

/// Leading-edge pressure drag of a rounded/airfoil fin edge (Niskanen 2009 eq. 3.89).
fn round_edge_cd(m: f64) -> f64 {
    if m < 0.9 {
        (1.0 - m * m).powf(-0.417) - 1.0
    } else if m < 1.0 {
        1.0 - 1.785 * (m - 0.9)
    } else {
        1.214 - 0.502 / (m * m) + 0.1095 / m.powi(4)
    }
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
            tube_fins: vec![],
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
                angles: Vec::new(),
                parent: 0,
                bodies: 1,
                mid_chord_cos: None,
                root_extent: None,
            }],
            protuberances: vec![Protuberance::LaunchLug { station: 0.15, length: 0.02, outer_diameter: 0.005, inner_diameter: 0.004 }],
            stages: vec![],
        }
    }

    fn nose_only(shape: NoseShape, r: f64, fineness: f64) -> Vehicle {
        Vehicle {
            reference_diameter: 2.0 * r,
            body: vec![BodySegment { station: 0.0, length: 2.0 * r * fineness, fore_radius: 0.0, aft_radius: r, kind: BodyKind::Nose(shape), roughness: 0.0 }],
            tube_fins: vec![],
            fin_sets: vec![],
            protuberances: vec![],
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
                tube_fins: vec![],
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
            tube_fins: vec![],
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
        let margin = (st.cp - 0.21) / 0.0186;
        assert!((1.0..3.5).contains(&margin), "margin {margin} cp {}", st.cp);
        assert!(st.cn_alpha > 2.0 && st.cn_alpha < 60.0, "{}", st.cn_alpha);
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
            angles: Vec::new(),
            parent: 0,
            bodies: 1,
            mid_chord_cos: None,
            root_extent: None,
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
        let peak_m = (50..=300).max_by(|&i, &j| cd(i as f64 / 100.0).total_cmp(&cd(j as f64 / 100.0))).unwrap() as f64 / 100.0;
        assert!((0.95..1.25).contains(&peak_m), "peak at {peak_m}");
        assert!(cd(peak_m) > cd(0.3) * 1.3 && cd(peak_m) > cd(2.5), "peak {} sub {} sup {}", cd(peak_m), cd(0.3), cd(2.5));
        // Motor exhaust removes base drag.
        let mut v = alpha_like();
        v.stages.push(StageMass {
            dry: MassProps { mass: 0.05, cg: 0.2, i_long: 0.0, i_rot: 0.0 },
            fore_station: 0.0,
            aft_station: 0.33,
            motor: Some(StageMotor {
                curve: ThrustCurve { points: vec![(0.0, 5.0), (1.0, 5.0)], propellant_mass: 0.01, total_mass: 0.02, length: 0.07, diameter: 0.018 },
                mount_aft_station: 0.33,
                ignition: Timing::LAUNCH,
                ejection_delay: None,
            }),
            separation: Timing::BURNOUT,
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

    fn fin_cna(v: &Vehicle, mach: f64) -> Vec<f64> {
        Aero::new(v).stability(mach, 0.0).components.iter().filter(|c| c.0.starts_with("Fin")).map(|c| c.1).collect()
    }

    #[test]
    fn fin_lift_follows_sin_squared_of_roll() {
        // Niskanen 2009 §3.2.2: each fin's CNa is scaled by sin^2(theta - phi); theta = 0 (crossflow along +Y).
        let deg = PI / 180.0;
        assert!(roll_factor(&[0.0], 1).abs() < 1e-15);
        assert!((roll_factor(&[90.0 * deg], 1) - 1.0).abs() < 1e-12);
        assert!((roll_factor(&[150.0 * deg], 1) - 0.25).abs() < 1e-12);
        assert!((roll_factor(&[90.0 * deg, 270.0 * deg], 2) - 2.0).abs() < 1e-12);
        assert!(roll_factor(&[], 2).abs() < 1e-12);
        // 3+ evenly spaced fins: N/2 at any roll.
        for n in 3..=8 {
            assert!((roll_factor(&[], n) - f64::from(n) / 2.0).abs() < 1e-12);
            let turned: Vec<f64> = (0..n).map(|i| 0.3 + 2.0 * PI * f64::from(i) / f64::from(n)).collect();
            assert!((roll_factor(&turned, n) - f64::from(n) / 2.0).abs() < 1e-12);
        }
        // On the vehicle: one fin of the 3-fin set's slope is CNa(set)/1.5; 2 fins across the flow give twice that.
        let mut v = alpha_like();
        let three = fin_cna(&v, 0.3)[0];
        v.fin_sets[0].count = 2;
        v.fin_sets[0].angles = vec![0.0, PI];
        assert!(fin_cna(&v, 0.3).first().is_none_or(|c| c.abs() < 1e-12));
        v.fin_sets[0].angles = vec![PI / 2.0, 1.5 * PI];
        for m in [0.3, 1.2, 2.0] {
            v.fin_sets[0].count = 3;
            v.fin_sets[0].angles = Vec::new();
            let base = fin_cna(&v, m)[0] / 1.5;
            v.fin_sets[0].count = 2;
            v.fin_sets[0].angles = vec![PI / 2.0, 1.5 * PI];
            assert!((fin_cna(&v, m)[0] - 2.0 * base).abs() < 1e-12 * base.abs().max(1.0));
        }
        assert!((fin_cna(&v, 0.3)[0] - three / 1.5 * 2.0).abs() < 1e-12);
    }

    #[test]
    fn many_fins_lose_lift_to_fin_fin_interference() {
        // Niskanen 2009 §3.2.2 table: 5..8 fins 0.948 / 0.913 / 0.854 / 0.81, then 0.75.
        assert_eq!([1, 2, 3, 4, 5, 6, 7, 8, 9, 12].map(fin_fin_factor), [1.0, 1.0, 1.0, 1.0, 0.948, 0.913, 0.854, 0.81, 0.75, 0.75]);
        let mut v = alpha_like();
        let one = fin_cna(&v, 0.5)[0] / 1.5;
        v.fin_sets[0].count = 6;
        assert!((fin_cna(&v, 0.5)[0] - 3.0 * 0.913 * one).abs() < 1e-12);
        // Two overlapping 4-fin sets on one body act as 8 fins; on different bodies (or apart) they do not interfere.
        v.fin_sets[0].count = 4;
        let mut second = v.fin_sets[0].clone();
        second.angles = (0..4).map(|i| PI / 4.0 + PI / 2.0 * f64::from(i)).collect();
        v.fin_sets.push(second);
        let both = fin_cna(&v, 0.5);
        assert!(both.iter().all(|c| (c - 2.0 * 0.81 * one).abs() < 1e-12), "{both:?}");
        v.fin_sets[1].parent = 7;
        assert!(fin_cna(&v, 0.5).iter().all(|c| (c - 2.0 * one).abs() < 1e-12));
        v.fin_sets[1].parent = 0;
        v.fin_sets[1].station -= v.fin_sets[1].root_chord;
        assert!(fin_cna(&v, 0.5).iter().all(|c| (c - 2.0 * one).abs() < 1e-12));
        // A pod set spread over 3 pods with 2 fins each counts 2 fins per body, not 6.
        let mut v = alpha_like();
        v.fin_sets[0].count = 6;
        v.fin_sets[0].bodies = 3;
        v.fin_sets[0].angles = vec![PI / 2.0, 1.5 * PI, PI / 2.0, 1.5 * PI, PI / 2.0, 1.5 * PI];
        assert!((fin_cna(&v, 0.5)[0] - 6.0 * one).abs() < 1e-12);
    }

    // ---- Hand-computed checks of the closed-form formulas ----

    #[test]
    fn stagnation_and_base_match_hand_calc() {
        // Stagnation (eq. B.1-B.2) and base (eq. 3.94) drag coefficients.
        assert!((stagnation_cd(0.0) - 0.85).abs() < 1e-12);
        assert!((stagnation_cd(1.0) - 0.85 * 1.275).abs() < 1e-12);
        assert!((stagnation_cd(2.0) - 0.85 * (1.84 - 0.19 + 0.010_375 + 0.000_546_875)).abs() < 1e-9);
        assert!((base_cd(0.5) - 0.1525).abs() < 1e-12);
        assert!((base_cd(1.0) - 0.25).abs() < 1e-12 && (base_cd(2.0) - 0.125).abs() < 1e-12);
    }

    #[test]
    fn skin_friction_matches_flat_plate_theory() {
        // Fully turbulent: 1/(1.5 ln Re - 5.6)^2, subsonic compressibility (1 - 0.1 M^2).
        let t = 1.0 / (1.5 * 1e7f64.ln() - 5.6).powi(2);
        assert!((skin_cf(1e7, 0.0, false) - t).abs() < 1e-12);
        assert!((skin_cf(1e7, 0.5, false) - t * 0.975).abs() < 1e-12);
        assert!((skin_cf(1e3, 0.0, false) - 1.48e-2).abs() < 1e-12);
        // Supersonic: / (1 + 0.15 M^2)^0.58.
        assert!((skin_cf(1e7, 2.0, false) - t / 1.6f64.powf(0.58)).abs() < 1e-12);
        // Perfect finish: laminar below Re 5.39e5, turbulent minus 1700/Re above.
        assert!((skin_cf(1e5, 0.0, true) - 1.328 / 1e5f64.sqrt()).abs() < 1e-12);
        assert!((skin_cf(1e6, 0.0, true) - (1.0 / (1.5 * 1e6f64.ln() - 5.6).powi(2) - 1.7e-3)).abs() < 1e-12);
    }

    #[test]
    fn nose_pressure_drag_matches_tr_r100_tables() {
        let nu = 1.46e-5;
        let press = |v: &Vehicle, m: f64| Aero::new(v).drag(m, m * 340.0 / nu, false).pressure;
        // Fineness-3 tangent ogive: Cd = 2.1 sin^2 + 0.5 sin / sqrt(M^2 - 1), sin = 1/sqrt(1 + 4 f^2), on the base area.
        let s = 1.0 / 37f64.sqrt();
        let v = nose_only(NoseShape::TangentOgive, 0.03, 3.0);
        assert!((press(&v, 2.0) - (2.1 * s * s + 0.5 * s / 3f64.sqrt())).abs() < 1e-4, "{}", press(&v, 2.0));
        // Conical at Mach 1: Cd = sin(phi) = R / sqrt(R^2 + L^2).
        let v = nose_only(NoseShape::Conical, 0.03, 3.0);
        assert!((press(&v, 1.0) - s).abs() < 1e-3, "{}", press(&v, 1.0));
        // Von Karman fineness 3 is the tabulated value (0.091 at Mach 2).
        let v = nose_only(NoseShape::VonKarman, 0.03, 3.0);
        assert!((press(&v, 2.0) - 0.091).abs() < 1e-6, "{}", press(&v, 2.0));
        // The data are extrapolated with stag*(Cd/stag)^(ln(f+1)/ln 4): fineness 7 lowers it.
        let v7 = nose_only(NoseShape::VonKarman, 0.03, 7.0);
        assert!(press(&v7, 2.0) < 0.091 * 0.8);
        // Subsonic nose pressure drag vanishes and rises towards Mach 1.
        assert!(press(&v, 0.3) < 1e-3 && press(&v, 0.95) >= press(&v, 0.6));
    }

    #[test]
    fn rounded_fin_edge_matches_closed_form() {
        assert!((round_edge_cd(0.5) - ((1.0f64 - 0.25).powf(-0.417) - 1.0)).abs() < 1e-12);
        assert!((round_edge_cd(0.95) - (1.0 - 1.785 * 0.05)).abs() < 1e-12);
        assert!((round_edge_cd(2.0) - (1.214 - 0.1255 + 0.1095 / 16.0)).abs() < 1e-12);
    }

    #[test]
    fn normal_force_is_the_component_sum_and_damping_sum_the_barrowman_strip_sum() {
        let a = Aero::new(&typical());
        for (mach, alpha) in [(0.1, 0.0), (0.3, 0.05), (0.8, 0.2), (2.0, 0.1)] {
            let st = a.stability(mach, alpha);
            let nf = a.normal_force(mach, alpha);
            assert!((nf.cn_alpha - st.cn_alpha).abs() < 1e-12 && (nf.cp - st.cp).abs() < 1e-12);
            for cg in [0.3, 0.55, 0.8] {
                let hand: f64 = st.components.iter().map(|(_, cna, x)| cna * (x - cg) * (x - cg)).sum();
                assert!((nf.damping_sum(cg) - hand).abs() < 1e-9 * hand.max(1.0), "M {mach} cg {cg}: {} vs {hand}", nf.damping_sum(cg));
            }
        }
        // Damping about the CG is smallest when the lifting components cluster at the CG and grows with the lever arm.
        let nf = a.normal_force(0.3, 0.0);
        assert!(nf.damping_sum(nf.cp) < nf.damping_sum(nf.cp - 0.3));
    }

    #[test]
    fn body_crossflow_damping_is_the_planform_third_moment() {
        // A uniform tube of length L and radius r about its own mid-point: sum A |x - cg|^3 = 2 r * 2 (L/2)^4 / 4 = r L^4 / 16.
        let (r, l) = (0.02, 0.5);
        let v = Vehicle {
            reference_diameter: 2.0 * r,
            body: vec![BodySegment { station: 0.0, length: l, fore_radius: r, aft_radius: r, kind: BodyKind::Tube, roughness: 0.0 }],
            fin_sets: vec![],
            tube_fins: vec![],
            protuberances: vec![],
            stages: vec![],
        };
        let k = Aero::new(&v).crossflow_damping(0.5 * l);
        assert!((k - r * l.powi(4) / 16.0).abs() < 1e-3 * r * l.powi(4) / 16.0, "{k}");
    }

    #[test]
    fn fin_roll_forcing_follows_cant_and_damping_is_the_strip_sum() {
        let mut v = alpha_like();
        let a0 = Aero::new(&v);
        assert_eq!(a0.roll(0.3).forcing, 0.0, "no cant, no roll forcing");
        let f = v.fin_sets[0].clone();
        v.fin_sets[0].cant = 0.02;
        let a = Aero::new(&v);
        let r = a.roll(0.3);
        assert!(r.forcing > 0.0 && r.damping < 0.0);
        // 3 fins at 120 degrees: the crossflow lift multiplier is N/2 = 1.5, so each fin's face-on lift is the set's / 1.5.
        let set = a.stability(0.3, 0.0).components.iter().find(|c| c.0 == "Fins").unwrap().1;
        let (a_ref, one_fin_lift_times_n) = (std::f64::consts::FRAC_PI_4 * v.reference_diameter.powi(2), set / 1.5 * 3.0);
        // The arms are chord-weighted averages of r = R + y over the span, so they lie between R and R + span.
        let arm2 = -r.damping / (one_fin_lift_times_n * a_ref);
        let arm = r.forcing / (one_fin_lift_times_n * a_ref * 0.02);
        assert!((f.body_radius..f.body_radius + f.span).contains(&arm), "arm {arm}");
        assert!(arm2 >= arm * arm * 0.999 && arm2 <= (f.body_radius + f.span).powi(2), "arm2 {arm2} arm {arm}");
        // Doubling the cant doubles the forcing and leaves the damping alone.
        v.fin_sets[0].cant = 0.04;
        let r2 = Aero::new(&v).roll(0.3);
        assert!((r2.forcing / r.forcing - 2.0).abs() < 1e-9 && (r2.damping / r.damping - 1.0).abs() < 1e-9);
    }

    #[test]
    fn axial_drag_multiplier_shape() {
        assert!((axial_drag_multiplier(0.0) - 1.0).abs() < 1e-12);
        assert!((axial_drag_multiplier(17f64.to_radians()) - 1.3).abs() < 1e-9);
        assert!(axial_drag_multiplier(PI / 2.0).abs() < 1e-9);
        let a = 17f64.to_radians();
        assert!((axial_drag_multiplier(a - 1e-9) - axial_drag_multiplier(a + 1e-9)).abs() < 1e-6);
    }

    #[test]
    fn perfect_finish_reduces_low_speed_drag() {
        let v = typical();
        let rough = Aero::new(&v).drag(0.2, 2e6, false).total;
        let perfect = Aero::new(&v).perfect_finish(true).drag(0.2, 2e6, false).total;
        assert!(perfect <= rough);
    }

    /// Vehicles with different fin shapes/aspect ratios for the transonic continuity checks.
    fn fin_variants() -> Vec<Vehicle> {
        let mut out = vec![alpha_like(), typical()];
        let mut elliptical = typical();
        elliptical.fin_sets[0].planform = FinPlanform::Elliptical;
        out.push(elliptical);
        let mut low_ar = typical();
        low_ar.fin_sets[0].span = 0.03;
        low_ar.fin_sets[0].root_chord = 0.14;
        low_ar.fin_sets[0].tip_chord = 0.1;
        out.push(low_ar);
        let mut high_ar = typical();
        high_ar.fin_sets[0].span = 0.15;
        high_ar.fin_sets[0].root_chord = 0.06;
        high_ar.fin_sets[0].tip_chord = 0.03;
        high_ar.fin_sets[0].sweep = 0.05;
        out.push(high_ar);
        out
    }

    #[test]
    fn cna_and_cp_are_c1_continuous_through_transonic() {
        // Fin CNa is blended with a quartic over Mach 0.9..1.5 and CP with a quintic over
        // 0.5..2, both matching value and slope at the ends, so no corners may remain.
        for v in fin_variants() {
            let a = Aero::new(&v);
            let len = v.length();
            let step = 0.005;
            let n = ((3.0 - 0.5) / step) as usize;
            let samples: Vec<(f64, f64)> = (0..=n)
                .map(|i| {
                    let st = a.stability(0.5 + step * i as f64, 0.0);
                    (st.cn_alpha, st.cp)
                })
                .collect();
            for w in samples.windows(3) {
                // First differences (jumps) and second differences (slope corners).
                assert!((w[1].0 - w[0].0).abs() < 0.06 * w[0].0.abs().max(1.0), "CNa jump {w:?}");
                assert!((w[1].1 - w[0].1).abs() < 8e-3 * len, "CP jump {w:?}");
                let d2n = w[2].0 - 2.0 * w[1].0 + w[0].0;
                let d2p = w[2].1 - 2.0 * w[1].1 + w[0].1;
                assert!(d2n.abs() < 4e-3 * w[1].0.abs().max(1.0), "CNa corner {w:?} d2 {d2n}");
                assert!(d2p.abs() < 4e-4 * len, "CP corner {w:?} d2 {d2p}");
            }
        }
    }

    /// Regression (results comparison report under `docs/`): the transonic fin CNa is ONE quartic in Mach through
    /// 0.9..1.5 with the derivative at Mach 0.9 taken at 0.9, as Niskanen (2009, section 3.2.2) describes.
    /// A quartic has a vanishing fifth finite difference.
    #[test]
    fn transonic_fin_cna_is_a_single_quartic_with_the_slope_taken_at_mach_0_9() {
        for v in fin_variants() {
            let a = Aero::new(&v);
            let a_ref = v.reference_area();
            for fin in &a.fins {
                let step = 0.04;
                let f: Vec<f64> = (0..6).map(|i| fin.cna1(0.95 + step * i as f64, 0.0, a_ref)).collect();
                let fifth = f[5] - 5.0 * f[4] + 10.0 * f[3] - 10.0 * f[2] + 5.0 * f[1] - f[0];
                assert!(fifth.abs() < 1e-9 * f[0].abs().max(1.0), "fifth difference {fifth} of {f:?}");
                // The slope leaving Mach 0.9 is the Diederich slope there, not the one at the Mach being evaluated.
                let h = 1e-5;
                let left = (fin.cna1(0.9, 0.0, a_ref) - fin.cna1(0.9 - h, 0.0, a_ref)) / h;
                let right = (fin.cna1(0.9 + h, 0.0, a_ref) - fin.cna1(0.9, 0.0, a_ref)) / h;
                assert!((left - right).abs() < 2e-3 * left.abs().max(1.0), "slope {left} vs {right}");
            }
        }
    }

    #[test]
    fn supersonic_margin_shrinks_but_stays_positive() {
        // Supersonic fin lift (K1 = 2/beta, per fin) falls well below the subsonic value, so
        // the CP moves forward and the margin shrinks to a fraction of its subsonic value, but stays positive.
        let a = Aero::new(&typical());
        let cg = 0.42;
        let m03 = a.margin_cal(0.3, 0.0, cg);
        let m3 = a.margin_cal(3.0, 0.0, cg);
        assert!(m3 > 0.2 && m3 < 0.7 * m03, "margin M0.3 {m03} M3 {m3}");
        // At Mach 1.5 the fin CNa is the K1 term exactly: A * 2/beta / A_ref per fin.
        let v = typical();
        let f = &v.fin_sets[0];
        let beta = (1.5f64 * 1.5 - 1.0).sqrt();
        let (_, k2, k3) = supersonic_k(1.5);
        assert!(k2 > 0.0 && k3 > 0.0);
        let expect = 0.5 * 4.0 * f.fin_area() * 2.0 / beta / v.reference_area() * (1.0 + f.body_radius / (f.span + f.body_radius));
        let st = a.stability(1.5, 0.0);
        let fins = st.components.iter().find(|c| c.0 == "Fins").unwrap();
        assert!((fins.1 - expect).abs() / expect < 1e-9, "{} vs {expect}", fins.1);
    }

    #[test]
    fn fin_cp_fraction_limits() {
        assert_eq!(fin_cp_fraction(0.3, 1.5), 0.25);
        // Supersonic CP at high AR*beta tends to mid-chord from ahead of it.
        let hi = fin_cp_fraction(3.0, 4.0);
        assert!(hi > 0.25 && hi < 0.5, "{hi}");
        // Value and slope agree on both sides of Mach 2.
        let (a, b) = (fin_cp_fraction(2.0 - 1e-6, 1.5), fin_cp_fraction(2.0 + 1e-6, 1.5));
        assert!((a - b).abs() < 1e-5);
    }
}
