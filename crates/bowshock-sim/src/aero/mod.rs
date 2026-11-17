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

mod drag;
mod fin;
mod model;
mod tables;
#[cfg(test)]
mod tests;

use crate::drag_table::DragTable;
use crate::tube_fins::TubeFinCalc;
use crate::vehicle::{FinCrossSection, Labels};
use std::sync::Arc;
use tables::Table;

pub use drag::axial_drag_multiplier;
pub(crate) use drag::lug_drag;
pub(crate) use fin::{fin_cp_fraction, FIN_STALL};
pub use fin::{fin_fin_factor, roll_factor, CROSSFLOW_ROLL};

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

impl DragBreakdown {
    /// This breakdown with every category multiplied by `k`.
    pub fn scaled(&self, k: f64) -> DragBreakdown {
        DragBreakdown {
            friction: self.friction * k,
            pressure: self.pressure * k,
            base: self.base * k,
            fins: self.fins * k,
            protuberances: self.protuberances * k,
            total: self.total * k,
        }
    }

    /// This breakdown with `total` set to the sum of the categories.
    fn summed(mut self) -> DragBreakdown {
        self.total = self.friction + self.pressure + self.base + self.fins + self.protuberances;
        self
    }
}

/// Normal force and drag of one vehicle component (see [`Aero::components`]).
#[derive(Clone, Debug, PartialEq)]
pub struct ComponentAero {
    /// Display name of the component's source part.
    pub name: String,
    /// Model part the component was built from.
    pub part: Option<bowshock_model::PartId>,
    /// Normal force coefficient slope, per radian (0 for a component without lift).
    pub cn_alpha: f64,
    /// Centre of pressure station from the tip (m), NaN for a component without lift.
    pub cp: f64,
    /// Drag at the condition, axial-drag multiplier applied; a drag override is the whole drag, in the component's category.
    pub drag: DragBreakdown,
    /// Whether the drag is the label's override rather than the build-up.
    pub overridden: bool,
}

/// One component of the vehicle, by its index in the vehicle's list of that kind.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Comp {
    Body(usize),
    Fin(usize),
    TubeFin(usize),
    Prot(usize),
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
    /// A spherically blunted nose (**own estimate**, see `tables::blunt_nose_cd`):
    /// `(table(M) (1 - k^2) + cap_cd(M) k^2) * area / A_ref` with `k = r_n / R`, `area` the nose base area, `table` the pressure
    /// drag of the sharp profile the tip sits on and `hemisphere` the nose table of a hemisphere (the cap's subsonic drag).
    BluntTable {
        table: Table,
        hemisphere: Table,
        tip_ratio: f64,
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
    /// Names, source parts and drag overrides of the components (see [`Labels`]).
    labels: Labels,
    /// CFD drag table flown instead of the build-up, which then supplies the corrections.
    table: Option<Arc<DragTable>>,
}
