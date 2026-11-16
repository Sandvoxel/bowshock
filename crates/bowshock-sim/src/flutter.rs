//! Fin flutter: the bending–torsion flutter boundary of a thin flat fin panel by Martin, *Summary of Flutter Experiences as a
//! Guide to the Preliminary Design of Lifting Surfaces on Missiles* (NACA TN 4197, 1958), in the form Bennett gives in *Fin
//! Flutter Analysis Revisited (Again)* (2023). It is a preliminary-design estimate: an empirical fit to flutter tests of thin
//! panels, not a structural model of the fin.
//!
//! # The method
//!
//! ```text
//! V_f = a sqrt( G / ( D_N AR³ / ((t/c)³ (AR + 2)) · (λ + 1)/2 · p/p0 ) )        D_N = 24 ε κ p0 / π
//! ```
//!
//! * `a` and `p` are the speed of sound and static pressure where the rocket is, `p0 = 101 325 Pa`, `κ = 1.4`.
//! * `AR = b²/S` is the aspect ratio of one panel (exposed span `b`, panel area `S`), `λ = c_t/c_r` its taper ratio and
//!   `t/c = t/c_r` its thickness ratio at the root.
//! * `ε = C_x/c_r − 1/4` is how far the panel's area centroid lies behind the quarter chord of the root, as a fraction of the
//!   root chord (`C_x` measured from the root leading edge). For a trapezoid with tip chord `c_t` and sweep length `m` (the tip
//!   leading edge's distance behind the root's) `C_x = (m (c_r + 2c_t) + c_r² + c_r c_t + c_t²) / (3 (c_r + c_t))`. A
//!   rectangle has `ε = 1/4` (`D_N = 270.9 kPa`, 39.29 psi); Bennett's swept example (`c_r` 7.5, `c_t` 2.5, `m` 4.285, span 3
//!   in) has `ε = 0.349` and `D_N = 54.9 psi`.
//! * `G` is the shear modulus of the fin stock.
//!
//! The often-quoted hobby form `V_f = a sqrt(G / (1.337 AR³ p (λ+1) / (2 (AR+2) (t/c)³)))` takes `1.337 = D_N / (2 p0)` at
//! `ε = 1/4` (the `(λ+1)/2` halving already folded in) and then halves the denominator again, which over-states `V_f` by
//! `√2`; it also ignores the sweep through `ε`. This module uses the full form above.
//!
//! # How a fin becomes the inputs ([`FlutterGeometry::of`])
//!
//! * A trapezoid is taken as drawn. A free-form fin reaches the simulation as the trapezoid with its root chord, span, area
//!   and centroid (see [`crate::from_model`]), so its `AR`, `ε` and taper come from the real outline's area and centroid.
//! * An elliptical fin has `ε = 1/4` and the pseudo tip chord that gives the trapezoid with its root and span the same area
//!   (`c_t = (π/2 − 1) c_r`).
//! * The thickness is the section's mean: `t` for a square edge, `t (1 − (1 − π/4) t/c̄)` for rounded edges (`c̄` the mean
//!   chord), `0.685 t` for a symmetric four-digit NACA airfoil (its area is `0.685 t c`). The method is for a flat plate; the
//!   mean thickness is the plate of the same material volume, never more than the torsion-equivalent (cube-mean) thickness, so
//!   it errs on the low, conservative side. Fins of tapering thickness are not modelled (the document has one thickness).
//! * Tube fins are not panels and have no estimate here.
//! * The root is taken as clamped: Martin's panels were fixed at the root, which is how a fin that is glued along its root, filleted
//!   or through-the-wall (a tab to the motor mount) is built. The estimate does not change with a tab and takes no credit for one.
//!
//! # Along a flight ([`FlutterTrace`])
//!
//! `V_f` scales with `a/√p`, so it rises as the air thins, while the rocket's speed peaks near burnout. The margin `V_f / V` is
//! therefore evaluated at every sample of the flight up to apogee, with the air of the launch site's atmosphere at the sample's
//! altitude; the minimum is the flight's flutter margin, wherever it falls (often the fast, low-altitude transonic part of the
//! boost rather than the point of maximum speed higher up).
//!
//! # Limits ([`FlutterWarning`])
//!
//! Martin's data do not support the estimate above Mach 1.5. Planforms with an extreme aspect or thickness ratio, or a
//! centroid ahead of the quarter chord (`ε` ≤ 0, where the formula has no finite answer; `ε` is held at [`MIN_EPSILON`]),
//! are flagged.

use std::fmt;

use crate::atmosphere::{AirState, Atmosphere, GAMMA};
use crate::flight::FlightResult;
use crate::vehicle::{FinCrossSection, FinPlanform, FinSet};

/// Reference (sea-level standard) pressure `p0` of the method, Pa.
pub const P0: f64 = 101_325.0;
/// Ratio of specific heats `κ` of the method.
pub const KAPPA: f64 = GAMMA;
/// Highest Mach number the estimate is supported to (Martin 1958).
pub const VALIDATED_MACH: f64 = 1.5;
/// Smallest `ε` the estimate uses: a centroid at or ahead of the root quarter chord is held here (and flagged).
pub const MIN_EPSILON: f64 = 0.02;
/// Largest `ε` not flagged: `ε = 3/4` puts the centroid at the root's trailing edge (`C_x = c_r`); a panel swept further back than
/// that is outside the planforms the estimate is meant for.
pub const MAX_EPSILON: f64 = 0.75;
/// Aspect ratios outside this range are flagged as unusual for the method.
pub const ASPECT_RATIO_RANGE: (f64, f64) = (0.3, 3.0);
/// Thickness ratios outside this range are flagged as unusual for the method.
pub const THICKNESS_RATIO_RANGE: (f64, f64) = (0.005, 0.08);
/// Below this airspeed (m/s) a sample has no margin to speak of (the rocket on the pad).
const MIN_SPEED: f64 = 1.0;
/// Area of a symmetric four-digit NACA section as a fraction of chord × thickness.
const AIRFOIL_AREA: f64 = 0.685;

/// The planform numbers of one fin panel the flutter estimate takes. SI.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlutterGeometry {
    /// Root chord `c_r` (m).
    pub root_chord: f64,
    /// Tip chord `c_t` (m); for an elliptical fin the pseudo tip chord of the trapezoid with the same area.
    pub tip_chord: f64,
    /// Exposed span `b` (m).
    pub span: f64,
    /// Panel area `S` (m²).
    pub area: f64,
    /// Chordwise position `C_x` of the area centroid behind the root leading edge (m).
    pub centroid: f64,
    /// Mean thickness `t` (m).
    pub thickness: f64,
}

/// Chordwise centroid of a trapezoid behind its root leading edge (see the module docs).
fn trapezoid_centroid(root: f64, tip: f64, sweep: f64) -> f64 {
    let sum = (root + tip).max(1e-12);
    (sweep * (root + 2.0 * tip) + root * root + root * tip + tip * tip) / (3.0 * sum)
}

impl FlutterGeometry {
    /// A trapezoid of root and tip chord, span and sweep length (the tip leading edge's distance behind the root's), of mean
    /// thickness `thickness`.
    pub fn trapezoid(root_chord: f64, tip_chord: f64, span: f64, sweep: f64, thickness: f64) -> FlutterGeometry {
        FlutterGeometry {
            root_chord,
            tip_chord,
            span,
            area: 0.5 * (root_chord + tip_chord) * span,
            centroid: trapezoid_centroid(root_chord, tip_chord, sweep),
            thickness,
        }
    }

    /// An elliptical fin of root chord and span: `ε = 1/4` and the pseudo tip chord `(π/2 − 1) c_r` (see the module docs).
    pub fn elliptical(root_chord: f64, span: f64, thickness: f64) -> FlutterGeometry {
        let area = std::f64::consts::FRAC_PI_4 * root_chord * span;
        FlutterGeometry { root_chord, tip_chord: 2.0 * area / span.max(1e-12) - root_chord, span, area, centroid: 0.5 * root_chord, thickness }
    }

    /// The panel of a simulated fin set, with the mean thickness of its cross-section (see the module docs).
    pub fn of(fin: &FinSet) -> FlutterGeometry {
        let area = fin.fin_area();
        let mean_chord = area / fin.span.max(1e-12);
        let thickness = match fin.cross_section {
            FinCrossSection::Square => fin.thickness,
            FinCrossSection::Rounded => fin.thickness * (1.0 - (1.0 - std::f64::consts::FRAC_PI_4) * fin.thickness / mean_chord.max(1e-12)).max(0.0),
            FinCrossSection::Airfoil => AIRFOIL_AREA * fin.thickness,
        };
        match fin.planform {
            FinPlanform::Trapezoidal => FlutterGeometry::trapezoid(fin.root_chord, fin.tip_chord, fin.span, fin.sweep, thickness),
            FinPlanform::Elliptical => FlutterGeometry::elliptical(fin.root_chord, fin.span, thickness),
        }
    }

    /// Aspect ratio `AR = b²/S`.
    pub fn aspect_ratio(&self) -> f64 {
        self.span * self.span / self.area.max(1e-12)
    }

    /// Taper ratio `λ = c_t/c_r`.
    pub fn taper_ratio(&self) -> f64 {
        self.tip_chord / self.root_chord.max(1e-12)
    }

    /// Thickness ratio `t/c_r`.
    pub fn thickness_ratio(&self) -> f64 {
        self.thickness / self.root_chord.max(1e-12)
    }

    /// `ε = C_x/c_r − 1/4`, as the planform has it (may be ≤ 0; the estimate uses at least [`MIN_EPSILON`]).
    pub fn epsilon(&self) -> f64 {
        self.centroid / self.root_chord.max(1e-12) - 0.25
    }

    /// The denominator constant `D_N = 24 ε κ p0 / π` (Pa), with `ε` held at [`MIN_EPSILON`] or more.
    pub fn denominator_constant(&self) -> f64 {
        24.0 * self.epsilon().max(MIN_EPSILON) * KAPPA * P0 / std::f64::consts::PI
    }

    /// Whether the panel can be put through the formula: a finite, positive root chord, span, area and thickness.
    pub fn is_usable(&self) -> bool {
        [self.root_chord, self.span, self.area, self.thickness, self.centroid, self.tip_chord].iter().all(|v| v.is_finite())
            && self.root_chord > 0.0
            && self.span > 0.0
            && self.area > 0.0
            && self.thickness > 0.0
    }
}

/// One fin set as the estimate sees it: its panel and the shear modulus of its stock.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlutterFin {
    /// The panel.
    pub geometry: FlutterGeometry,
    /// Effective shear modulus `G` (Pa).
    pub shear_modulus: f64,
}

/// Every input of the formula for one fin in one air state, and the flutter velocity they give.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlutterInputs {
    /// `AR`.
    pub aspect_ratio: f64,
    /// `λ`.
    pub taper_ratio: f64,
    /// `t/c`.
    pub thickness_ratio: f64,
    /// `ε` as the planform has it.
    pub epsilon: f64,
    /// `D_N` (Pa).
    pub denominator_constant: f64,
    /// `a` (m/s).
    pub speed_of_sound: f64,
    /// `p/p0`.
    pub pressure_ratio: f64,
    /// `G` (Pa).
    pub shear_modulus: f64,
    /// Mean thickness `t` (m).
    pub thickness: f64,
    /// `V_f` (m/s).
    pub flutter_velocity: f64,
}

impl FlutterInputs {
    /// The inputs of `fin` in the air `air`.
    pub fn of(fin: &FlutterFin, air: &AirState) -> FlutterInputs {
        let g = &fin.geometry;
        let (ar, lambda, tc) = (g.aspect_ratio(), g.taper_ratio(), g.thickness_ratio().max(1e-9));
        let dn = g.denominator_constant();
        let pressure_ratio = air.pressure / P0;
        let denominator = dn * ar.powi(3) / (tc.powi(3) * (ar + 2.0)) * (lambda + 1.0) / 2.0 * pressure_ratio;
        // A degenerate panel or stock has no estimate: V_f is 0 (margin 0, flagged), never infinite or not a number.
        let raw = air.speed_of_sound * (fin.shear_modulus.max(0.0) / denominator.max(1e-300)).sqrt();
        let flutter_velocity = if g.is_usable() && raw.is_finite() { raw } else { 0.0 };
        FlutterInputs {
            aspect_ratio: ar,
            taper_ratio: lambda,
            thickness_ratio: tc,
            epsilon: g.epsilon(),
            denominator_constant: dn,
            speed_of_sound: air.speed_of_sound,
            pressure_ratio,
            shear_modulus: fin.shear_modulus,
            thickness: g.thickness,
            flutter_velocity,
        }
    }
}

/// Flutter velocity `V_f` (m/s) of `fin` in the air `air` (see the module docs).
pub fn flutter_velocity(fin: &FlutterFin, air: &AirState) -> f64 {
    FlutterInputs::of(fin, air).flutter_velocity
}

/// The estimate at one instant of a flight.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlutterPoint {
    /// Time (s).
    pub t: f64,
    /// Altitude above the launch site (m).
    pub altitude: f64,
    /// Airspeed (m/s).
    pub speed: f64,
    /// Mach number.
    pub mach: f64,
    /// The lowest flutter velocity of the fin sets here (m/s).
    pub flutter_velocity: f64,
    /// Index of the fin set it belongs to.
    pub governing: usize,
}

impl FlutterPoint {
    /// `V_f / V`: infinite while the rocket is (all but) still.
    pub fn margin(&self) -> f64 {
        if self.speed > MIN_SPEED {
            self.flutter_velocity / self.speed
        } else {
            f64::INFINITY
        }
    }
}

/// What makes an estimate less trustworthy.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FlutterWarning {
    /// The flight goes faster than [`VALIDATED_MACH`]: the highest Mach number reached.
    BeyondMach(f64),
    /// The governing panel's aspect ratio is outside [`ASPECT_RATIO_RANGE`].
    AspectRatio(f64),
    /// The governing panel's thickness ratio is outside [`THICKNESS_RATIO_RANGE`].
    ThicknessRatio(f64),
    /// The governing panel's centroid is at or ahead of the root quarter chord: its `ε`, held at [`MIN_EPSILON`].
    CentroidForward(f64),
    /// The governing panel's centroid is behind the root trailing edge (`ε` above [`MAX_EPSILON`]): a far-swept planform.
    CentroidAft(f64),
    /// The governing panel has a zero or non-finite chord, span, area or thickness.
    Degenerate,
    /// The governing fin stock's shear modulus is zero or not a number.
    NoShearModulus,
}

impl fmt::Display for FlutterWarning {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FlutterWarning::BeyondMach(m) => write!(f, "Reaches Mach {m:.2}: Martin's estimate is not supported above Mach {VALIDATED_MACH:.1}"),
            FlutterWarning::AspectRatio(ar) => {
                write!(f, "Aspect ratio {ar:.2} is outside {:.1}–{:.1}, unusual for this thin-panel estimate", ASPECT_RATIO_RANGE.0, ASPECT_RATIO_RANGE.1)
            }
            FlutterWarning::ThicknessRatio(tc) => write!(
                f,
                "Thickness ratio {tc:.3} is outside {:.3}–{:.2}, unusual for this thin-panel estimate",
                THICKNESS_RATIO_RANGE.0, THICKNESS_RATIO_RANGE.1
            ),
            FlutterWarning::CentroidForward(e) => {
                write!(f, "Centroid ahead of the quarter chord (ε {e:.2}): the method has no answer there; ε is held at {MIN_EPSILON}")
            }
            FlutterWarning::CentroidAft(e) => write!(
                f,
                "Centroid behind the root trailing edge (ε {e:.2}, above {MAX_EPSILON}): the panel is swept far beyond the trapezoids the method is meant for, so D_N and V_f are extrapolated"
            ),
            FlutterWarning::Degenerate => f.write_str("The fin has no usable panel (a zero or non-finite chord, span, area or thickness): no flutter estimate"),
            FlutterWarning::NoShearModulus => f.write_str("The fin stock has no shear modulus: no flutter estimate until one is set"),
        }
    }
}

/// The estimate along a flight up to apogee.
#[derive(Clone, Debug, PartialEq)]
pub struct FlutterTrace {
    /// The flight's samples with the lowest flutter velocity of the fin sets at each.
    pub points: Vec<FlutterPoint>,
    /// Index into `points` of the lowest margin.
    pub worst: usize,
    /// The atmosphere the air came from.
    pub atmosphere: Atmosphere,
}

impl FlutterTrace {
    /// The estimate of `fins` along `path`, `(time s, altitude above the site m, airspeed m/s)` samples in time order, in the
    /// air of `atmosphere`. `None` without fins or samples.
    pub fn along(fins: &[FlutterFin], atmosphere: &Atmosphere, path: impl IntoIterator<Item = (f64, f64, f64)>) -> Option<FlutterTrace> {
        if fins.is_empty() {
            return None;
        }
        let points: Vec<FlutterPoint> = path
            .into_iter()
            .map(|(t, altitude, speed)| {
                let air = atmosphere.at_agl(altitude);
                let (governing, flutter_velocity) =
                    fins.iter().map(|fin| flutter_velocity(fin, &air)).enumerate().min_by(|a, b| a.1.total_cmp(&b.1)).unwrap_or((0, f64::INFINITY));
                FlutterPoint { t, altitude, speed, mach: speed / air.speed_of_sound, flutter_velocity, governing }
            })
            .collect();
        let worst = points.iter().enumerate().min_by(|a, b| a.1.margin().total_cmp(&b.1.margin())).map(|(i, _)| i)?;
        Some(FlutterTrace { points, worst, atmosphere: *atmosphere })
    }

    /// The estimate of `fins` along `flight` from liftoff to apogee (airspeed, so wind counts), in the air of `atmosphere`
    /// (the flight's launch site).
    pub fn of_flight(fins: &[FlutterFin], atmosphere: &Atmosphere, flight: &FlightResult) -> Option<FlutterTrace> {
        let apogee = flight.summary.time_to_apogee;
        let path = flight.trajectory.samples.iter().take_while(|s| s.t <= apogee + 1e-9).map(|s| (s.t, s.altitude, s.air_speed));
        FlutterTrace::along(fins, atmosphere, path)
    }

    /// The point of the lowest margin.
    pub fn worst(&self) -> &FlutterPoint {
        &self.points[self.worst]
    }

    /// The lowest margin `V_f / V` of the flight.
    pub fn min_margin(&self) -> f64 {
        self.worst().margin()
    }

    /// The highest Mach number reached.
    pub fn max_mach(&self) -> f64 {
        self.points.iter().map(|p| p.mach).fold(0.0, f64::max)
    }

    /// Every input of the formula at the worst point, for its governing fin set of `fins` (the slice the trace was made with).
    pub fn inputs_at_worst(&self, fins: &[FlutterFin]) -> Option<FlutterInputs> {
        let w = self.worst();
        fins.get(w.governing).map(|fin| FlutterInputs::of(fin, &self.atmosphere.at_agl(w.altitude)))
    }

    /// What makes this estimate less trustworthy: a flight beyond [`VALIDATED_MACH`], and an unusual governing panel.
    pub fn warnings(&self, fins: &[FlutterFin]) -> Vec<FlutterWarning> {
        let mut out = Vec::new();
        let mach = self.max_mach();
        if mach > VALIDATED_MACH {
            out.push(FlutterWarning::BeyondMach(mach));
        }
        if let Some(fin) = fins.get(self.worst().governing) {
            let g = &fin.geometry;
            let (ar, tc, e) = (g.aspect_ratio(), g.thickness_ratio(), g.epsilon());
            if !(ASPECT_RATIO_RANGE.0..=ASPECT_RATIO_RANGE.1).contains(&ar) {
                out.push(FlutterWarning::AspectRatio(ar));
            }
            if !(THICKNESS_RATIO_RANGE.0..=THICKNESS_RATIO_RANGE.1).contains(&tc) {
                out.push(FlutterWarning::ThicknessRatio(tc));
            }
            if e < MIN_EPSILON {
                out.push(FlutterWarning::CentroidForward(e));
            }
            if e > MAX_EPSILON {
                out.push(FlutterWarning::CentroidAft(e));
            }
            if !g.is_usable() {
                out.push(FlutterWarning::Degenerate);
            }
            if !(fin.shear_modulus.is_finite() && fin.shear_modulus > 0.0) {
                out.push(FlutterWarning::NoShearModulus);
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const IN: f64 = 0.0254;
    const FT: f64 = 0.3048;
    const PSI: f64 = 6_894.757_293;

    fn close(a: f64, b: f64, rel: f64) {
        assert!((a - b).abs() <= rel * b.abs(), "{a} vs {b}");
    }

    /// Bennett (2023)'s worked example: a swept trapezoid of 7.5 in root, 2.5 in tip, 4.285 in sweep length and 3 in span.
    fn bennett(thickness_in: f64) -> FlutterFin {
        FlutterFin { geometry: FlutterGeometry::trapezoid(7.5 * IN, 2.5 * IN, 3.0 * IN, 4.285 * IN, thickness_in * IN), shear_modulus: 600_000.0 * PSI }
    }

    /// Bennett's air at 18 500 ft: a = 1043.36 ft/s, p = 7.2 psi (p/p0 = 0.49).
    fn bennett_air() -> AirState {
        AirState { temperature: 251.5, pressure: 0.49 * P0, density: 0.0, speed_of_sound: 1043.36 * FT, viscosity: 0.0 }
    }

    #[test]
    fn bennetts_planform_numbers() {
        let g = bennett(0.125).geometry;
        close(g.centroid / IN, 4.49, 1e-3);
        close(g.epsilon(), 0.349, 2e-3);
        close(g.denominator_constant() / PSI, 54.88, 2e-3);
        close(g.area / (IN * IN), 15.0, 1e-12);
        close(g.aspect_ratio(), 0.6, 1e-12);
        close(g.thickness_ratio(), 0.0167, 3e-3);
        close(g.taper_ratio(), 1.0 / 3.0, 1e-12);
    }

    #[test]
    fn bennetts_flutter_velocities() {
        // Bennett (2023): 1425, 1991.7 and 2618.1 ft/s for 1/8, 5/32 and 3/16 in G10 (G = 600 ksi) at 18 500 ft.
        for (t, expect) in [(0.125, 1425.0), (0.15625, 1991.7), (0.1875, 2618.1)] {
            let vf = flutter_velocity(&bennett(t), &bennett_air()) / FT;
            close(vf, expect, 0.01);
            // The same through the standard atmosphere at 18 500 ft above sea level.
            let isa = flutter_velocity(&bennett(t), &Atmosphere::standard().at_msl(18_500.0 * FT)) / FT;
            close(isa, expect, 0.01);
        }
    }

    #[test]
    fn a_rectangle_has_epsilon_a_quarter_and_the_tabulated_constant() {
        let g = FlutterGeometry::trapezoid(0.1, 0.1, 0.05, 0.0, 0.003);
        close(g.epsilon(), 0.25, 1e-12);
        // D_N = 24 · 0.25 · 1.4 · 101 325 / π = 270.9 kPa = 39.29 psi.
        close(g.denominator_constant(), 270_920.0, 1e-4);
        close(g.denominator_constant() / PSI, 39.294, 1e-3);
    }

    #[test]
    fn the_centroid_follows_the_sweep() {
        // A triangle with its tip over the root trailing edge: centroid at 2/3 of the root.
        close(trapezoid_centroid(0.3, 0.0, 0.3), 0.2, 1e-12);
        // A rectangle: mid-chord, whatever the chord.
        close(trapezoid_centroid(0.12, 0.12, 0.0), 0.06, 1e-12);
        // Sweeping a panel back by m moves its centroid back by m times the centroid's height fraction.
        let (cr, ct, b) = (0.12, 0.05, 1.0);
        let height = b * (cr + 2.0 * ct) / (3.0 * (cr + ct));
        close(trapezoid_centroid(cr, ct, 0.06) - trapezoid_centroid(cr, ct, 0.0), 0.06 * height / b, 1e-12);
    }

    #[test]
    fn an_ellipse_is_a_quarter_chord_panel_with_its_area() {
        let g = FlutterGeometry::elliptical(0.12, 0.08, 0.003);
        close(g.epsilon(), 0.25, 1e-12);
        close(g.area, std::f64::consts::FRAC_PI_4 * 0.12 * 0.08, 1e-12);
        // The pseudo trapezoid with the same root and span has the ellipse's area.
        close(0.5 * (g.root_chord + g.tip_chord) * g.span, g.area, 1e-12);
        close(g.taper_ratio(), std::f64::consts::FRAC_PI_2 - 1.0, 1e-12);
    }

    #[test]
    fn the_hobby_formula_overstates_by_root_two() {
        // A rectangle (ε = 1/4) at sea level: the corrected velocity times √2 is the often-quoted form's (constant 1.337, a second
        // halving of the denominator).
        let fin = FlutterFin { geometry: FlutterGeometry::trapezoid(0.15, 0.15, 0.09, 0.0, 0.003), shear_modulus: 2.7e9 };
        let air = Atmosphere::standard().at_agl(0.0);
        let g = fin.geometry;
        let (ar, lambda, tc) = (g.aspect_ratio(), g.taper_ratio(), g.thickness_ratio());
        let hobby = air.speed_of_sound * (2.7e9 / (1.337 * ar.powi(3) * air.pressure * (lambda + 1.0) / (2.0 * (ar + 2.0) * tc.powi(3)))).sqrt();
        close(flutter_velocity(&fin, &air) * std::f64::consts::SQRT_2, hobby, 1e-3);
    }

    #[test]
    fn velocity_scales_with_thickness_to_the_three_halves_and_root_g() {
        let air = Atmosphere::standard().at_agl(0.0);
        let base = flutter_velocity(&bennett(0.125), &air);
        close(flutter_velocity(&bennett(0.25), &air), base * 2f64.powf(1.5), 1e-9);
        let stiff = FlutterFin { shear_modulus: 4.0 * bennett(0.125).shear_modulus, ..bennett(0.125) };
        close(flutter_velocity(&stiff, &air), 2.0 * base, 1e-9);
    }

    #[test]
    fn mean_thickness_of_the_sections() {
        let mut fin = crate::vehicle::FinSet {
            count: 3,
            station: 0.0,
            body_radius: 0.04,
            root_chord: 0.1,
            tip_chord: 0.1,
            span: 0.05,
            sweep: 0.0,
            thickness: 0.004,
            cross_section: FinCrossSection::Square,
            cant: 0.0,
            roughness: 0.0,
            planform: FinPlanform::Trapezoidal,
            mid_chord_cos: None,
            root_extent: None,
            angles: Vec::new(),
            parent: 0,
            bodies: 1,
        };
        close(FlutterGeometry::of(&fin).thickness, 0.004, 1e-12);
        fin.cross_section = FinCrossSection::Airfoil;
        close(FlutterGeometry::of(&fin).thickness, 0.685 * 0.004, 1e-12);
        fin.cross_section = FinCrossSection::Rounded;
        // Two half-round edges of diameter t replace t × t/2 of plate each: (4 − π)/4 t² less section area.
        close(FlutterGeometry::of(&fin).thickness, (0.004 * 0.1 - (4.0 - std::f64::consts::PI) / 4.0 * 0.004 * 0.004) / 0.1, 1e-12);
    }

    #[test]
    fn the_worst_point_can_be_the_low_fast_one() {
        // A boost through Mach 0.99 at 1.5 km that keeps accelerating to its top speed at 6 km. V_f rises with altitude (it goes
        // as a/√p), so the margin is worst low down, at the transonic point, not at the maximum speed.
        let fin = bennett(0.0625);
        let atmosphere = Atmosphere::standard();
        let path = [(0.0, 0.0, 0.0), (2.0, 300.0, 250.0), (4.0, 1500.0, 330.0), (8.0, 6000.0, 345.0), (14.0, 9000.0, 200.0), (25.0, 15000.0, 20.0)];
        let trace = FlutterTrace::along(&[fin], &atmosphere, path).unwrap();
        let fastest = trace.points.iter().max_by(|a, b| a.speed.total_cmp(&b.speed)).unwrap();
        assert_eq!(fastest.t, 8.0);
        assert_eq!(trace.worst().t, 4.0);
        assert!(trace.min_margin() < 0.95 * fastest.margin(), "{} vs {}", trace.min_margin(), fastest.margin());
        close(trace.worst().mach, 330.0 / atmosphere.at_agl(1500.0).speed_of_sound, 1e-12);
        assert!(trace.points.iter().all(|p| p.margin() >= trace.min_margin()));
        assert!(trace.points.windows(2).all(|w| w[1].flutter_velocity > w[0].flutter_velocity));
        // The pad sample has no margin.
        assert!(trace.points[0].margin().is_infinite());
    }

    #[test]
    fn the_weakest_set_governs_and_warnings_name_the_limits() {
        let stiff = bennett(0.25);
        let weak = FlutterFin { shear_modulus: 0.2e9, ..bennett(0.03) };
        let atmosphere = Atmosphere::standard();
        let trace = FlutterTrace::along(&[stiff, weak], &atmosphere, [(1.0, 100.0, 300.0), (3.0, 2000.0, 600.0)]).unwrap();
        assert!(trace.points.iter().all(|p| p.governing == 1));
        let inputs = trace.inputs_at_worst(&[stiff, weak]).unwrap();
        close(inputs.flutter_velocity, trace.worst().flutter_velocity, 1e-12);
        let warnings = trace.warnings(&[stiff, weak]);
        // Mach 1.8 at 2 km, and a 0.03 in panel on a 7.5 in root (t/c 0.004).
        assert!(matches!(warnings[0], FlutterWarning::BeyondMach(m) if m > 1.5));
        assert!(warnings.iter().any(|w| matches!(w, FlutterWarning::ThicknessRatio(_))));
        assert!(warnings[0].to_string().contains("not supported above Mach 1.5"));
        // A panel swept forward ahead of the quarter chord.
        let forward = FlutterFin { geometry: FlutterGeometry::trapezoid(0.1, 0.02, 0.05, -0.08, 0.003), shear_modulus: 1e9 };
        assert!(forward.geometry.epsilon() < MIN_EPSILON);
        let t = FlutterTrace::along(&[forward], &atmosphere, [(1.0, 0.0, 100.0)]).unwrap();
        assert!(t.warnings(&[forward]).iter().any(|w| matches!(w, FlutterWarning::CentroidForward(_))));
        assert!(t.min_margin().is_finite());
    }
}
