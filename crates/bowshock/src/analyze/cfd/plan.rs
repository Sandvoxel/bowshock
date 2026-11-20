use crate::units::{with_unit_scaled, Quantity, UnitSystem};
use bowshock_core as solver;
use bowshock_model::{PartKind, RadialKind, Rocket};
use bowshock_sim::{BodyKind, FinCrossSection, FinPlanform, FinSet, NoseShape, Vehicle};

/// Bump when what a stored table means changes (solver, grid level, how the parts are mapped): old tables then read as stale.
pub(super) const SOLVER_REV: u64 = 5;
/// The same for a table the 3D engine (`BOWSHOCK_ENGINE=3d`) solved: a separate number, so that a table of one engine is stale to the other, and the legacy keys are unchanged.
/// At the engine swap this replaces [`SOLVER_REV`] (the legacy engine's tables are then stale, which they are meant to be).
pub(super) const SOLVER_REV_3D: u64 = 1004;
/// The lowest Mach number a drag node's wall spacing is sized for on a shared-sizing table ([`super::sizing_mach`], DESIGN3D item 57).
pub const DRAG_SIZING_MACH_FLOOR: f64 = 1.0;
/// Relative drag tolerance of the solver's stop rule: a point ends once its total drag varies by less than this over 500
/// iterations. Measured on the example rocket (coarse grid, RTX 3090, 10 points): about 29 000 iterations in all against about
/// 82 000 for the default rule, the worst point within 0.39 % of its 12 000-iteration value (the aim is 1 %).
pub(super) const DRAG_TOLERANCE: f64 = 0.002;
/// Whether a table's points share one grid, sized for the table's highest Mach number (`Analysis::grid_mach`), instead of one grid
/// per point sized for its own. Off: A/B on the example table (`perf_2d_table_grid_ab`, coarse grid, RTX 3090) saves 6 % of the wall
/// time warm and the kernel compile of every point but the first when the cache is cold, but moves Cd by +2.5 % at M 0.1 and +0.7 % at
/// M 1.0 (base drag; the tolerance is 0.5 %). Turning it on needs a `SOLVER_REV` bump.
pub(super) const SHARED_GRID: bool = false;
/// Whether the points of a table of the 3D engine start from the converged field of a neighbouring Mach number (`Analysis::warm_start`) instead of the free stream or the
/// axisymmetric seed. Off: opt-in with `BOWSHOCK_WARM_START` until the measurements of `DESIGN3D.md` item 59 are taken for the default; turning it on needs a `SOLVER_REV_3D` bump
/// (the converged points differ within the stop rule's noise).
pub(super) const WARM_START: bool = false;
/// Whether the CFD table also solves the jet of the stack's motor and the flight flies its effect on the base ([`POWER_ON_NODES`], DESIGN3D item 58). Off: the flight keeps the
/// build-up's exit-area correction of the base drag, and the table solves no power-on nodes (no extra cost in the full solve). The solved effect is opposite in sign to that correction
/// (the base drag rises with the jet on, x2 subsonic) and unvalidated (NASA TP-2005 fails, ISSUES #13), so it is an opt-in of the app (`BOWSHOCK_POWER_ON` set, or this constant): a table
/// solved with it carries the part, one solved without it does not, and the flight uses the part where it is.
pub(super) const POWER_ON: bool = false;
/// Mach number of the jet at the nozzle exit of the power-on solve: a hobby motor's nozzle expands its exhaust to about 2.5 (expansion ratio 6-8, `gamma` 1.2 in the real gas).
pub(super) const JET_EXIT_MACH: f64 = 2.5;
/// Total temperature of the jet (K): the flame temperature of an ammonium-perchlorate composite propellant (2 800-3 300 K) in the air model of the solver.
pub(super) const JET_TOTAL_TEMPERATURE: f64 = 3000.0;
/// The nozzle exit is sized from the thrust (the exit matched to sea level, see [`jet_of`]) but kept between these fractions of the motor case's diameter: the motor files
/// (RASP, RockSim) carry no nozzle dimensions, and the nozzle exits of the commercial 29-98 mm cases are 0.5-0.8 of the case.
pub(super) const JET_EXIT_FRACTION: (f64, f64) = (0.5, 0.8);
/// The Mach numbers a power-on solve takes, where the table has them (they are table nodes, so the power-off solution is there to compare with); a table range that holds none
/// takes its middle node. The solver's jet changes the base flow, and its effect on the drag is read between these nodes and held beyond them.
pub(super) const POWER_ON_NODES: [f64; 4] = [0.5, 0.9, 1.3, 2.3];
/// The Mach range the solver accepts.
pub const SOLVER_MACH: (f64, f64) = (0.1, 3.0);
/// The highest altitude the solver's standard atmosphere has (m).
pub(super) const SOLVER_CEILING: f64 = 20_000.0;
/// Interior Mach numbers of a table: dense through the transonic drag rise, sparse where the drag is smooth. The range's ends
/// are always nodes too.
pub(super) const INTERIOR: [f64; 8] = [0.5, 0.8, 0.9, 1.0, 1.1, 1.3, 1.7, 2.3];
/// An interior node this close to an end of the range is left out (the end stands for it).
pub(super) const NODE_GAP: f64 = 0.12;
/// Bump when what a stored 3D stability solve means changes (solver, grid, how the forces are reduced).
pub(super) const LIFT_REV: u64 = 5;
/// [`LIFT_REV`] of the 3D engine (the half domain on the base-wake grid, the base solved, the fins on that grid).
pub(super) const LIFT_REV_3D: u64 = 1004;

/// The revision of the engine in use: what a stored table's key carries.
fn solver_rev() -> u64 {
    if solver::Engine::from_env() == solver::Engine::Solver3 {
        SOLVER_REV_3D
    } else {
        SOLVER_REV
    }
}

fn lift_rev() -> u64 {
    if solver::Engine::from_env() == solver::Engine::Solver3 {
        LIFT_REV_3D
    } else {
        LIFT_REV
    }
}
/// The Mach nodes of a 3D stability solve (sparse: each costs a 3D Euler run); the solve takes those inside the plotted range.
pub const LIFT_INTERIOR: [f64; 8] = [0.5, 0.8, 0.95, 1.1, 1.3, 1.7, 2.3, 3.0];
/// The angles of attack (degrees) every node is solved at.
pub const LIFT_ALPHAS: [f64; 1] = [2.0];
/// The lowest and highest Mach number of the 3D solve.
pub(super) const LIFT_MACH: (f64, f64) = (0.5, 3.0);

/// Why a Mach range cannot be tabulated.
pub const OUT_OF_RANGE: &str = "the solver covers Mach 0.1 to 3";

/// Hash of the bytes of numbers: FNV-1a, stable across runs, platforms and Rust versions (unlike `DefaultHasher`).
pub(super) struct Fnv(pub(super) u64);

impl Fnv {
    pub(super) fn new() -> Fnv {
        Fnv(0xcbf2_9ce4_8422_2325)
    }

    pub(super) fn bytes(&mut self, bytes: &[u8]) -> &mut Self {
        for &byte in bytes {
            self.0 ^= u64::from(byte);
            self.0 = self.0.wrapping_mul(0x0100_0000_01b3);
        }
        self
    }

    pub(super) fn u64(&mut self, v: u64) -> &mut Self {
        self.bytes(&v.to_le_bytes())
    }

    /// A length or coefficient to a micro-unit, so a recomputed value that differs in the last bits hashes the same.
    pub(super) fn f(&mut self, v: f64) -> &mut Self {
        self.u64(((v * 1e6).round() as i64) as u64)
    }
}

/// What the solver is given and what stays with the build-up method, for one flying stack of the document.
#[derive(Debug, Clone)]
pub struct Plan {
    /// The axisymmetric body.
    pub body: solver::BodySpec,
    /// The fin set the solver models, if the rocket has one it can take.
    pub fins: Option<solver::FinSpec>,
    /// The outline of those fins when it is not the trapezoid of their numbers (an ellipse, a freeform polygon): what the 3D engine meshes; the legacy engine flies the
    /// equivalent trapezoid.
    pub outline: solver::FinOutline,
    /// Equivalent sand-grain roughness of the solved walls (m): the mean of the parts' finish roughness weighted by wetted area.
    pub body_roughness: f64,
    /// The vehicle without the solved fin set: what the build-up method still adds (other fin sets, lugs, rail buttons).
    pub(super) rest: Vehicle,
    /// What was not solved as the document has it, in the app's voice.
    pub notes: Vec<String>,
    /// Hash of the geometry (the body, the fins and what the build-up adds).
    pub geometry: u64,
    /// The jet of the stack's motor ([`jet_of`]): what the power-on part of the table is solved with; `None` for a stack without a motor. Not part of the geometry: a new motor makes
    /// the power-on part stale, not the drag table.
    pub motor: Option<solver::MotorSpec>,
}

/// The solver's nose family for the document's, `None` when there is none; the second value says the nose was approximated.
pub(super) fn nose_shape(shape: bowshock_model::NoseShape) -> Result<(solver::SharpShape, Option<&'static str>), String> {
    use bowshock_model::NoseShape as M;
    use solver::SharpShape as S;
    Ok(match shape {
        M::Conical => (S::Cone, None),
        M::TangentOgive => (S::TangentOgive, None),
        M::VonKarman => (S::Haack { c: 0.0 }, None),
        M::Haack(c) if (0.0..=1.0 / 3.0).contains(&c) => (S::Haack { c }, None),
        M::Haack(c) => return Err(format!("the nose is a Haack series with C = {c:.2}; the solver takes C up to 1/3")),
        M::Parabolic => (S::Parabolic { k: 1.0 }, None),
        M::PowerSeries(n) if (0.5..=1.0).contains(&n) => (S::Power { n }, None),
        M::PowerSeries(n) => return Err(format!("the nose is a power series with n = {n:.2}; the solver takes n from 0.5 to 1")),
        M::Elliptical => (S::Power { n: 0.5 }, Some("The elliptical nose is solved as a half-power profile.")),
    })
}

/// A sharp family as the solver's pointed nose shape.
fn sharp_nose(shape: solver::SharpShape) -> solver::NoseShape {
    match shape {
        solver::SharpShape::Cone => solver::NoseShape::Cone,
        solver::SharpShape::TangentOgive => solver::NoseShape::TangentOgive,
        solver::SharpShape::Power { n } => solver::NoseShape::Power { n },
        solver::SharpShape::Parabolic { k } => solver::NoseShape::Parabolic { k },
        solver::SharpShape::Haack { c } => solver::NoseShape::Haack { c },
    }
}

/// Whether the 3D engine (`BOWSHOCK_ENGINE=3d`) is the one the table is solved with: it follows the fins' real outlines and refuses what its mesher cannot build.
pub(super) fn three_d() -> bool {
    solver::Engine::from_env() == solver::Engine::Solver3
}

/// The outline the 3D engine meshes for the fin set `set` of the document `rocket`: the half ellipse of an elliptical fin, the polygon of a freeform one (found in the
/// document by its span and thickness), else the trapezoid.
pub(super) fn fin_outline(rocket: &Rocket, set: &FinSet) -> solver::FinOutline {
    // a freeform fin is flown as its equivalent trapezoid and says so by its real root extent
    match (set.planform, set.root_extent.is_some()) {
        (FinPlanform::Elliptical, _) => solver::FinOutline::Ellipse,
        (FinPlanform::Trapezoidal, false) => solver::FinOutline::Trapezoid,
        (FinPlanform::Trapezoidal, true) => rocket
            .radials()
            .iter()
            .find_map(|r| match &r.kind {
                RadialKind::FreeformFin { points, thickness }
                    if (r.kind.fin_span().unwrap_or(0.0) - set.span).abs() < 1e-6 && (thickness - set.thickness).abs() < 1e-9 =>
                {
                    Some(solver::FinOutline::Polygon(points.clone()))
                }
                _ => None,
            })
            .unwrap_or(solver::FinOutline::Trapezoid),
    }
}

/// A fin set as the solver's fins, or why it cannot be.
pub(super) fn fin_spec(set: &FinSet, x_base: f64) -> Result<solver::FinSpec, String> {
    let (tip_chord, sweep) = match set.planform {
        FinPlanform::Trapezoidal => (set.tip_chord, set.sweep),
        // The trapezoid with the ellipse's root, span and area (pi/4 root span), its tip centred on the root.
        FinPlanform::Elliptical => {
            let tip = (std::f64::consts::FRAC_PI_2 - 1.0) * set.root_chord;
            (tip, 0.5 * (set.root_chord - tip))
        }
    };
    let profile = match set.cross_section {
        FinCrossSection::Square => solver::FinProfile::Square,
        FinCrossSection::Rounded => solver::FinProfile::Rounded,
        FinCrossSection::Airfoil => solver::FinProfile::Airfoil,
        FinCrossSection::DoubleWedge => solver::FinProfile::DoubleWedge,
        FinCrossSection::Hexagonal(wedge_fraction) => solver::FinProfile::Hexagonal { wedge_fraction },
        FinCrossSection::Biconvex => solver::FinProfile::Biconvex,
    };
    let spec = solver::FinSpec {
        count: set.count,
        root_chord: set.root_chord,
        tip_chord,
        span: set.span,
        sweep,
        thickness: set.thickness,
        roughness: set.roughness,
        // the real root of a freeform fin ends where its polygon's root does (the 3D engine meshes the polygon); the legacy engine flies the equivalent trapezoid
        root_te_to_base: (x_base - if three_d() { set.root_extent.map_or(set.station + set.root_chord, |e| e.1) } else { set.station + set.root_chord })
            .max(0.0),
        profile,
    };
    spec.validate_for(solver::Engine::from_env()).map_err(|e| e.to_string())?;
    Ok(spec)
}

/// Wetted (lateral) area of a body segment (m^2), integrated along its profile.
pub(super) fn wetted_area(s: &bowshock_sim::BodySegment) -> f64 {
    const N: usize = 256;
    let dx = s.length / N as f64;
    let mut r_prev = s.radius_at(0.0);
    let mut area = 0.0;
    for i in 1..=N {
        let r = s.radius_at(i as f64 * dx);
        area += std::f64::consts::PI * (r_prev + r) * (r - r_prev).hypot(dx);
        r_prev = r;
    }
    area
}

/// Mean of the segments' finish roughness (m) weighted by wetted area; 0 without area.
pub(super) fn wetted_mean_roughness(segments: &[bowshock_sim::BodySegment]) -> f64 {
    let (sum, area) = segments.iter().fold((0.0, 0.0), |(sum, area), s| {
        let a = wetted_area(s);
        (sum + s.roughness * a, area + a)
    });
    if area > 0.0 {
        sum / area
    } else {
        0.0
    }
}
/// The jet of the motor in the lowest stage of `vehicle` for the power-on solve: an air jet (the solver's model, `bowshock_core::MotorSpec`) at [`JET_EXIT_MACH`] and
/// [`JET_TOTAL_TEMPERATURE`] whose thrust is the motor's average thrust (impulse over burn time).
///
/// The motor files give a case diameter and a thrust curve and no nozzle, so the exit is derived. The static thrust is `F = p_e A_e (1 + gamma M_e^2) - p_sl A_e`; with the exit
/// matched to sea level (`p_e = p_sl`, how a hobby nozzle is tuned) `A_e = F / (gamma M_e^2 p_sl)`, kept between [`JET_EXIT_FRACTION`] of the case diameter and inside the base,
/// and the exit pressure follows from the thrust at that area: `p_e = (F / A_e + p_sl) / (1 + gamma M_e^2)`. The thrust is the motor's static one, so the jet is independent
/// of the flight's altitude (the solver puts it in the free stream's units: the jet's pressure ratio rises as the rocket climbs). `None` without a motor or a burn.
pub(super) fn jet_of(vehicle: &Vehicle) -> Option<solver::MotorSpec> {
    const P_SEA_LEVEL: f64 = 101_325.0;
    const GAMMA: f64 = 1.4;
    let motor = vehicle.stages.first()?.motor.as_ref()?;
    let base = vehicle.body.last()?;
    let burn = motor.curve.burn_time();
    let thrust = motor.curve.total_impulse() / burn;
    if !(thrust.is_finite() && thrust > 0.0 && motor.curve.diameter > 0.0) {
        return None;
    }
    let gm2 = GAMMA * JET_EXIT_MACH * JET_EXIT_MACH;
    let matched = (4.0 * thrust / (std::f64::consts::PI * gm2 * P_SEA_LEVEL)).sqrt();
    let upper = (JET_EXIT_FRACTION.1 * motor.curve.diameter).min(0.9 * 2.0 * base.aft_radius);
    let exit_diameter = matched.clamp(JET_EXIT_FRACTION.0 * motor.curve.diameter, upper.max(1e-6));
    let area = std::f64::consts::FRAC_PI_4 * exit_diameter * exit_diameter;
    Some(solver::MotorSpec {
        exit_diameter,
        exit_mach: JET_EXIT_MACH,
        exit_pressure_pa: (thrust / area + P_SEA_LEVEL) / (1.0 + gm2),
        total_temperature_k: JET_TOTAL_TEMPERATURE,
    })
}

/// Maps the vehicle of one flying stack of `rocket` (the document supplies the nose family the vehicle rounds to its five) to
/// what the solver takes. The error says why the geometry cannot be solved at all, with lengths in `units`.
pub fn plan(rocket: &Rocket, vehicle: &Vehicle, units: UnitSystem) -> Result<Plan, String> {
    let segments = &vehicle.body;
    let mut notes: Vec<String> = Vec::new();
    // Pods are bodies of their own put ahead of the core's, each closed by a zero-size tube; a stack the solver can take is one
    // contiguous run of segments from the tip.
    let contiguous = segments.windows(2).all(|w| (w[1].station - w[0].aft_station()).abs() < 1e-6);
    if segments.iter().any(|s| s.length <= 1e-9) || !contiguous || segments.first().is_some_and(|s| s.station.abs() > 1e-6) {
        return Err(super::wording::no_pods().to_string());
    }
    if !vehicle.tube_fins.is_empty() {
        return Err(super::wording::no_tube_fins().to_string());
    }
    let first = segments.first().ok_or("the rocket has no parts")?;
    let BodyKind::Nose(sim_shape) = first.kind else {
        return Err("the solver needs a nose cone at the tip".to_string());
    };
    // The vehicle keeps five nose families; the document's own is exact (a pod-less stack starts with the rocket's nose).
    let model_shape = match rocket.parts().first().map(|p| &p.kind) {
        Some(PartKind::NoseCone { shape, .. }) => *shape,
        _ => match sim_shape {
            NoseShape::Conical => bowshock_model::NoseShape::Conical,
            NoseShape::TangentOgive => bowshock_model::NoseShape::TangentOgive,
            NoseShape::VonKarman => bowshock_model::NoseShape::VonKarman,
            NoseShape::Elliptical => bowshock_model::NoseShape::Elliptical,
            NoseShape::Parabolic => bowshock_model::NoseShape::PowerSeries(0.5),
        },
    };
    let (base, approximated) = nose_shape(model_shape)?;
    notes.extend(approximated.map(String::from));
    // The segment's length is the nose as built and its tip radius the (feasible) sphere on it: the solver blunts the sharp family
    // itself, so the solved profile is the one the model draws.
    let shape = if first.tip_radius > 0.0 { solver::NoseShape::Blunted { base, tip_radius: first.tip_radius } } else { sharp_nose(base) };
    let nose = solver::NoseSpec { shape, length: first.length, diameter: 2.0 * first.aft_radius };

    let mut aft = Vec::new();
    let mut radius = first.aft_radius;
    for s in &segments[1..] {
        if (s.fore_radius - radius).abs() > 5e-4 {
            if three_d() {
                // the 3D engine meshes a step as a one-cell chamfer (DESIGN3D item 29); a fin root across it is refused with the fin set's reason
                aft.push(solver::AftSpec::Step { aft_diameter: 2.0 * s.fore_radius });
                radius = s.fore_radius;
            } else {
                let length = |m: f64| with_unit_scaled(m, Quantity::Length, units, 0);
                return Err(format!(
                    "there is a step in diameter, {} to {}, at {} from the tip; the legacy solver's body is continuous (the 3D engine takes steps)",
                    length(2.0 * radius),
                    length(2.0 * s.fore_radius),
                    length(s.station)
                ));
            }
        }
        match s.kind {
            BodyKind::Nose(_) => return Err("a second nose cone behind the first; the solver takes one".to_string()),
            BodyKind::Tube => aft.push(solver::AftSpec::Tube { length: s.length }),
            BodyKind::Transition if (s.aft_radius - radius).abs() <= 5e-4 => aft.push(solver::AftSpec::Tube { length: s.length }),
            BodyKind::Transition => aft.push(solver::AftSpec::Transition { length: s.length, aft_diameter: 2.0 * s.aft_radius }),
        }
        radius = s.aft_radius;
    }
    let body = solver::BodySpec { nose, aft };
    solver::Analysis::new(&body).map_err(|e| format!("the solver cannot take this body: {e}"))?;

    // The solver models one fin set: the supported one with the most fin area. The rest, and every lug and rail button, are
    // left to the build-up method, whose drag is added to the table.
    let x_base = segments.last().map_or(0.0, |s| s.aft_station());
    let mut candidates: Vec<(usize, solver::FinSpec, solver::FinOutline)> = Vec::new();
    let mut refused: Vec<String> = Vec::new();
    for (i, set) in vehicle.fin_sets.iter().enumerate() {
        match fin_spec(set, x_base) {
            Ok(spec) => {
                // the 3D engine's mesher says what it cannot build (a fin that folds the grid, a notched outline): that is the fin set's reason
                let outline = if three_d() { fin_outline(rocket, set) } else { solver::FinOutline::Trapezoid };
                let meshed = if three_d() {
                    solver::Analysis::new(&body)
                        .map_err(|e| e.to_string())
                        .and_then(|a| a.fins(spec.clone()).fin_outline(outline.clone()).mesh_check().map_err(|e| e.to_string()))
                } else {
                    Ok(())
                };
                match meshed {
                    Ok(()) => candidates.push((i, spec, outline)),
                    Err(e) => refused.push(e),
                }
            }
            Err(e) => refused.push(e),
        }
    }
    let area = |i: usize| f64::from(vehicle.fin_sets[i].count) * vehicle.fin_sets[i].fin_area();
    let solved = candidates.iter().map(|(i, _, _)| *i).max_by(|&a, &b| area(a).total_cmp(&area(b)));
    let (fins, outline) = match solved.and_then(|i| candidates.iter().find(|(j, _, _)| *j == i)) {
        Some((_, s, o)) => (Some(s.clone()), o.clone()),
        None => (None, solver::FinOutline::Trapezoid),
    };
    let mut rest = vehicle.clone();
    rest.stages.clear();
    if let Some(i) = solved {
        rest.fin_sets.remove(i);
    }
    if let Some(reason) = refused.first() {
        notes.push(format!("{} fin set{} left to the build-up ({reason}).", refused.len(), if refused.len() == 1 { "" } else { "s" }));
    }
    let extra = candidates.len().saturating_sub(1);
    if extra > 0 {
        notes.push(format!("Only one fin set is solved; {extra} more come{} from the build-up.", if extra == 1 { "s" } else { "" }));
    }
    if !rest.protuberances.is_empty() {
        notes.push(super::wording::APPENDAGES_NOTE.to_string());
    }
    // the legacy engine flies the equivalent trapezoid of an outline; the 3D engine meshes the outline itself
    if !three_d() {
        if rocket.radials().iter().any(|r| matches!(r.kind, RadialKind::FreeformFin { .. })) {
            notes.push("Freeform fins are solved as equivalent trapezoids.".to_string());
        }
        if vehicle.fin_sets.iter().any(|f| matches!(f.planform, FinPlanform::Elliptical)) && fins.is_some() {
            notes.push("Elliptical fins are solved as equivalent trapezoids.".to_string());
        }
    }
    let body_roughness = wetted_mean_roughness(segments);
    notes.push(if body_roughness > 0.0 {
        format!("Walls solved with the parts' finish: {:.0} um sand-grain roughness.", body_roughness * 1e6)
    } else {
        "Walls solved smooth (the parts' finish is polished).".to_string()
    });

    let mut plan = Plan { body, fins, outline, body_roughness, rest, notes, geometry: 0, motor: jet_of(vehicle) };
    plan.geometry = plan.hash();
    Ok(plan)
}

impl Plan {
    /// The jet of the power-on solve in the form a stored table keeps it.
    pub fn jet(&self) -> Option<bowshock_model::CfdJet> {
        self.motor.map(|m| bowshock_model::CfdJet {
            exit_diameter: m.exit_diameter,
            exit_mach: m.exit_mach,
            exit_pressure_pa: m.exit_pressure_pa,
            total_temperature_k: m.total_temperature_k,
        })
    }

    /// Hash of the geometry the table depends on: the body, the solved fins and what the build-up adds.
    fn hash(&self) -> u64 {
        let mut h = Fnv::new();
        h.u64(solver_rev());
        let n = &self.body.nose;
        match n.shape {
            solver::NoseShape::Cone => h.u64(0),
            solver::NoseShape::TangentOgive => h.u64(1),
            solver::NoseShape::Power { n } => h.u64(2).f(n),
            solver::NoseShape::Parabolic { k } => h.u64(3).f(k),
            solver::NoseShape::Haack { c } => h.u64(4).f(c),
            // a blunted nose: its sharp family, then the tip radius (a pointed nose hashes as it always did)
            solver::NoseShape::Blunted { base, tip_radius } => match base {
                solver::SharpShape::Cone => h.u64(5),
                solver::SharpShape::TangentOgive => h.u64(6),
                solver::SharpShape::Power { n } => h.u64(7).f(n),
                solver::SharpShape::Parabolic { k } => h.u64(8).f(k),
                solver::SharpShape::Haack { c } => h.u64(9).f(c),
            }
            .f(tip_radius),
        };
        h.f(n.length).f(n.diameter);
        // roughness in micrometres: the micro-unit rounding of `f` would otherwise blur the finishes
        h.f(self.body_roughness * 1e6);
        for a in &self.body.aft {
            match *a {
                solver::AftSpec::Tube { length } => h.u64(10).f(length),
                solver::AftSpec::Transition { length, aft_diameter } => h.u64(11).f(length).f(aft_diameter),
                solver::AftSpec::Step { aft_diameter } => h.u64(12).f(aft_diameter),
            };
        }
        match &self.fins {
            Some(f) => {
                // a non-trapezoidal outline (3D engine only) is part of the geometry; the trapezoid hashes as it always did
                match &self.outline {
                    solver::FinOutline::Trapezoid => {}
                    solver::FinOutline::Ellipse => {
                        h.u64(21);
                    }
                    solver::FinOutline::Polygon(p) => {
                        h.u64(22);
                        p.iter().for_each(|q| {
                            h.f(q.0).f(q.1);
                        });
                    }
                }
                h.u64(20).u64(u64::from(f.count)).f(f.root_chord).f(f.tip_chord).f(f.span).f(f.sweep).f(f.thickness).f(f.roughness * 1e6).f(f.root_te_to_base);
                h.u64(match f.profile {
                    solver::FinProfile::Square => 0,
                    solver::FinProfile::Rounded => 1,
                    solver::FinProfile::Airfoil => 2,
                    solver::FinProfile::DoubleWedge => 3,
                    solver::FinProfile::Custom { .. } => 4,
                    solver::FinProfile::Hexagonal { .. } => 5,
                    solver::FinProfile::Biconvex => 6,
                });
                if let solver::FinProfile::Hexagonal { wedge_fraction } = f.profile {
                    h.f(wedge_fraction);
                }
            }
            None => {
                h.u64(21);
            }
        }
        for f in &self.rest.fin_sets {
            h.u64(30).u64(u64::from(f.count)).f(f.station).f(f.root_chord).f(f.tip_chord).f(f.span).f(f.sweep).f(f.thickness).f(f.roughness);
        }
        for p in &self.rest.protuberances {
            match *p {
                bowshock_sim::Protuberance::LaunchLug { station, length, outer_diameter, inner_diameter } => {
                    h.u64(40).f(station).f(length).f(outer_diameter).f(inner_diameter)
                }
                bowshock_sim::Protuberance::RailButton { station, height, diameter } => h.u64(41).f(station).f(height).f(diameter),
            };
        }
        h.f(self.rest.reference_diameter);
        h.0
    }
}

/// The Mach numbers of the table for a Mach range: the range's ends (clipped to the solver's) and the interior nodes between
/// them. `None` when the range does not overlap the solver's.
pub fn nodes(range: (f64, f64)) -> Option<Vec<f64>> {
    let lo = range.0.min(range.1).max(SOLVER_MACH.0);
    let hi = range.0.max(range.1).min(SOLVER_MACH.1);
    if hi - lo < 0.05 {
        return None;
    }
    let mut out = vec![lo];
    out.extend(INTERIOR.iter().copied().filter(|m| *m > lo + NODE_GAP && *m < hi - NODE_GAP));
    out.push(hi);
    Some(out)
}

/// Whether the power-on part of the table is asked for ([`POWER_ON`], or `BOWSHOCK_POWER_ON` set): the 3D engine's jet solve and the flight's use of its ratios.
pub fn power_on_requested() -> bool {
    POWER_ON || std::env::var_os("BOWSHOCK_POWER_ON").is_some()
}

/// The Mach numbers of the power-on solve for a table solved at `nodes` ([`POWER_ON_NODES`] among them, else the middle node).
pub fn power_on_nodes(nodes: &[f64]) -> Vec<f64> {
    let own: Vec<f64> = POWER_ON_NODES.iter().copied().filter(|m| nodes.iter().any(|n| (n - m).abs() < 1e-9)).collect();
    if own.is_empty() {
        nodes.get(nodes.len() / 2).map(|&m| vec![m]).unwrap_or_default()
    } else {
        own
    }
}

/// The key of a table: the geometry, the altitude that sets the Reynolds number and the Mach nodes.
pub fn table_key(geometry: u64, altitude: f64, nodes: &[f64]) -> u64 {
    let mut h = Fnv::new();
    h.u64(geometry).f(altitude.clamp(0.0, SOLVER_CEILING));
    nodes.iter().for_each(|m| {
        h.f(*m);
    });
    h.0
}

/// The Mach nodes of a 3D stability solve for a Mach range: the [`LIFT_INTERIOR`] nodes inside it, or the nearest node when none
/// is (a range between two nodes), or the first node alone when the range lies wholly below it. `None` when the range lies
/// above Mach 3.
pub fn lift_nodes(range: (f64, f64)) -> Option<Vec<f64>> {
    let (a, b) = (range.0.min(range.1), range.0.max(range.1));
    if a > LIFT_MACH.1 + 1e-9 {
        return None;
    }
    if b < LIFT_MACH.0 {
        return Some(vec![LIFT_MACH.0]);
    }
    let (lo, hi) = (a.max(LIFT_MACH.0) - 1e-9, b.min(LIFT_MACH.1) + 1e-9);
    let inside: Vec<f64> = LIFT_INTERIOR.iter().copied().filter(|m| *m >= lo && *m <= hi).collect();
    if !inside.is_empty() {
        return Some(inside);
    }
    let mid = 0.5 * (lo + hi);
    LIFT_INTERIOR.iter().copied().min_by(|x, y| (x - mid).abs().total_cmp(&(y - mid).abs())).map(|m| vec![m])
}

/// The key of a 3D stability solve: the geometry, the Mach nodes and the angles. The altitude is not in it (the solve is Euler).
pub fn lift_key(geometry: u64, nodes: &[f64]) -> u64 {
    let mut h = Fnv::new();
    h.u64(lift_rev()).u64(geometry);
    nodes.iter().for_each(|m| {
        h.f(*m);
    });
    LIFT_ALPHAS.iter().for_each(|a| {
        h.f(*a);
    });
    h.0
}
