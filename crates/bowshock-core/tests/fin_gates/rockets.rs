//! The rockets of the cross-validation gates (DESIGN3D item 55, `tests/rocket_gates.rs`): geometry and tunnel state of the test articles of the public reports whose data
//! `bowshock_reference::{arcas_stab, yuska, grm}` hold, as `Case3` builders. Lengths in the report's metres, converted to nose diameters here.
#![allow(dead_code)]

use super::common::{tier, TierKind};
use bowshock_core::engine::case3::{Case3, WallFlow};
use bowshock_core::engine::grid3::{Domain3, FinPhase, Fins3};
use bowshock_core::{AftSpec, BodySpec, FinProfile, NoseShape, NoseSpec};
use bowshock_reference::{arcas, grm, yuska};

/// A test article: body, one fin set, the diameter the lengths are normalised by and the tunnel state.
pub struct Rocket {
    pub name: &'static str,
    pub body: BodySpec,
    pub fins: Fins3,
    /// Nose diameter \[m\].
    pub d: f64,
    /// Reynolds number per metre of the test.
    pub re_per_m: f64,
    /// Stagnation temperature \[K\] (a stated value, or an assumption where the report gives none: see the module of the data).
    pub t0: f64,
}

impl Rocket {
    /// The flow the wall-resolved grids are sized for: `re_c = rho c d / mu = (Re/m) d / M`, Sutherland's `110.4 K / T_inf` at the stagnation temperature of the run. `mach_max` sizes the grid
    /// (the highest Mach number of the table the rocket serves), `mach` fixes the Reynolds number.
    pub fn flow(&self, mach: f64, mach_max: f64) -> WallFlow {
        let t_inf = self.t0 / (1.0 + 0.2 * mach * mach);
        WallFlow { re_c: self.re_per_m * self.d / mach, mach_max, ks_body: 0.0, ks_fin: 0.0, sutherland: 110.4 / t_inf }
    }

    /// The four-fin sector at zero incidence (the product's table path), on the tier of `kind`.
    pub fn sector(&self, mach: f64, mach_max: f64, kind: TierKind) -> Case3 {
        let (body, _) = self.body.to_internal().expect("body");
        Case3::sector(mach, body, self.fins.clone(), tier(kind, self.flow(mach, mach_max)), 0)
    }

    /// The same body without fins on the axisymmetric wedge.
    pub fn finless(&self, mach: f64, mach_max: f64, kind: TierKind) -> Case3 {
        let (body, _) = self.body.to_internal().expect("body");
        Case3::wedge(mach, body, 0).with_tier(tier(kind, self.flow(mach, mach_max)))
    }

    /// The half domain at `alpha_deg` of incidence on the base-wake grid (the lift tier unless the run is Euler), the fins in `phase`.
    pub fn half(&self, mach: f64, mach_max: f64, alpha_deg: f64, phase: FinPhase, kind: TierKind) -> Case3 {
        let (body, _) = self.body.to_internal().expect("body");
        let kind = if kind == TierKind::Euler { TierKind::Euler } else { TierKind::Lift };
        let mut c = Case3::base_half(mach, alpha_deg.to_radians(), body, Some(self.fins.clone()), tier(kind, self.flow(mach, mach_max)), 0);
        c.grid.domain = Domain3::Half { phase };
        c
    }

    /// Coefficients on the nose-base area to coefficients on the area of the largest cross-section (the data's reference area).
    pub fn area_factor(&self) -> f64 {
        let (body, _) = self.body.to_internal().expect("body");
        let case = Case3::wedge(2.0, body, 0);
        case.body.nose_to_largest_area()
    }
}

const IN: f64 = 0.0254;

/// A wedge over the whole chord from a leading-edge half thickness of a tenth of the maximum: the section of the Finner's fins (`Case3::anf_fins`, DESIGN3D item 38).
fn full_chord_wedge() -> FinProfile {
    FinProfile::Custom { section: vec![[0.0, 0.1], [0.5, 0.55], [1.0, 1.0]] }
}

fn fins_from(count: u32, root: f64, tip: f64, span: f64, sweep: f64, thickness: f64, te_to_base: f64, profile: FinProfile, d: f64) -> Fins3 {
    Fins3 {
        count,
        root_chord: root / d,
        tip_chord: tip / d,
        span: span / d,
        sweep: sweep / d,
        thickness: thickness / d,
        te_to_base: te_to_base / d,
        profile,
        planform: None,
    }
}

/// The Arcas short model (NASA TN D-4013 / D-4014 configuration 1, l/d 18.2; geometry and fins as `bowshock_reference::arcas`, the double-wedge fins of G-F4). `long` adds the
/// cylinder of configuration 2 (`arcas_stab::LONG_TUBE_EXTRA`).
pub fn arcas_rocket(long_extra_tube: f64, re_per_m: f64, t0: f64) -> Rocket {
    let d = arcas::BODY_DIAMETER;
    Rocket {
        name: if long_extra_tube > 0.0 { "arcas-long" } else { "arcas-short" },
        body: BodySpec {
            nose: NoseSpec { shape: NoseShape::TangentOgive, length: arcas::NOSE_LENGTH, diameter: d },
            aft: vec![
                AftSpec::Tube { length: arcas::TUBE_LENGTH + long_extra_tube },
                AftSpec::Transition { length: arcas::BOATTAIL_LENGTH, aft_diameter: arcas::BASE_DIAMETER },
            ],
        },
        fins: fins_from(
            4,
            arcas::FIN_ROOT_CHORD,
            arcas::FIN_TIP_CHORD,
            arcas::FIN_SPAN,
            arcas::FIN_SWEEP,
            arcas::FIN_THICKNESS,
            0.0,
            FinProfile::DoubleWedge,
            d,
        ),
        d,
        re_per_m,
        t0,
    }
}

/// The Yuska models (NASA TN D-3182): `shortened` model B (A without the 4.2 in section), `trimmed` model C (the fin without its 0.57 in tip strip). Nose as the power law the report's table
/// fits (`NOSE_POWER_N` over `NOSE_POWER_LENGTH`), then the 6 in cylinder. The fin is a swept thick wedge (10 degrees included) of the mean thickness of its root and tip trailing-edge faces,
/// on the Finner's wedge section (a sharp leading edge, the full-chord wedge), with **a planform the mesher takes in place of the drawing's** (DESIGN3D item 55): the drawing's trailing edge
/// leans aft with span and its tip hangs 2.34 in behind the base plane (root trailing edge flush with the base), and `plan_fins` refuses a fin that reaches behind its root trailing edge;
/// the fin used here has the drawing's root chord, tip chord and span (area as drawn) and a trailing edge on the base plane, so its leading-edge sweep is 28 degrees instead of the drawing's
/// 45 (the drawn leading edge builds no grid either: the 45 degree sweep over a 0.825 d span folds the cells above the tip). Stagnation temperature 300 K assumed (not stated).
pub fn yuska_rocket(shortened: bool, trimmed: bool, re_per_ft: f64) -> Rocket {
    let d = yuska::BODY_DIAMETER;
    let length = if shortened { yuska::MODEL_B_LENGTH } else { yuska::MODEL_A_LENGTH };
    let span = if trimmed { yuska::FIN_SPAN_MODEL_C } else { yuska::FIN_SPAN };
    // the trailing edge of the drawing is the straight line `x_te(h) = root chord + h te_slope` (11.49 in at the 4.95 in tip, 2.34 in behind the root's)
    let te_slope = (yuska::FIN_SPAN + yuska::FIN_TIP_CHORD - yuska::FIN_ROOT_CHORD) / yuska::FIN_SPAN;
    let drawn_tip_chord = yuska::FIN_ROOT_CHORD + span * te_slope - span;
    let thickness = 2.0 * (0.5 * yuska::FIN_WEDGE_INCLUDED_ANGLE_DEG).to_radians().tan() * 0.5 * (yuska::FIN_ROOT_CHORD + drawn_tip_chord);
    let (root, tip, sweep) = (yuska::FIN_ROOT_CHORD, drawn_tip_chord, yuska::FIN_ROOT_CHORD - drawn_tip_chord);
    Rocket {
        name: "yuska",
        body: BodySpec {
            nose: NoseSpec { shape: NoseShape::Power { n: yuska::NOSE_POWER_N }, length: yuska::NOSE_POWER_LENGTH, diameter: d },
            aft: vec![AftSpec::Tube { length: length - yuska::NOSE_POWER_LENGTH }],
        },
        fins: fins_from(4, root, tip, span, sweep, thickness, 0.0, full_chord_wedge(), d),
        d,
        re_per_m: re_per_ft / 0.3048,
        t0: 300.0,
    }
}

/// The Martin generalized research model B4 + fin F13 (AEDC-TR-75-125 Test 1): tangent ogive of 2.5 d, cylinder to 9.5 d, four clipped-delta fins of root chord 1 d, tip 0.5 d, span 0.75 d on
/// 33.7 degree leading-edge sweep and an unswept trailing edge at the base, modified double wedge with 4 degree edge angles (hexagonal: bevels of 0.31 of the chord, thickness 0.05 d).
/// Stagnation temperature 310 K assumed (not stated); fully turbulent (the report does not say how transition was fixed).
pub fn grm_rocket() -> Rocket {
    let d = grm::BODY_DIAMETER;
    let span = grm::FIN_SPAN_CAL * d;
    Rocket {
        name: "grm-b4-f13",
        body: BodySpec {
            nose: NoseSpec { shape: NoseShape::TangentOgive, length: grm::NOSE_LENGTH_CAL * d, diameter: d },
            aft: vec![AftSpec::Tube { length: (grm::TOTAL_LENGTH_CAL - grm::NOSE_LENGTH_CAL) * d }],
        },
        fins: fins_from(
            4,
            grm::FIN_ROOT_CHORD_CAL * d,
            grm::FIN_TIP_CHORD_CAL * d,
            span,
            (grm::FIN_ROOT_CHORD_CAL - grm::FIN_TIP_CHORD_CAL) * d, // the leading edge as drawn (33.7 degrees: 0.5 d over 0.75 d) with a trailing edge exactly on the base plane
            grm::FIN_ROOT_THICKNESS_CAL * d,
            0.0,
            FinProfile::Hexagonal { wedge_fraction: 0.5 * (grm::FIN_LE_BEVEL_CAL + grm::FIN_TE_BEVEL_CAL) / grm::FIN_ROOT_CHORD_CAL },
            d,
        ),
        d,
        re_per_m: grm::RE_PER_M,
        t0: 310.0,
    }
}

pub const IN_M: f64 = IN;
