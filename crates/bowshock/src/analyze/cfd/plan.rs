use crate::units::{with_unit_scaled, Quantity, UnitSystem};
use bowshock_core as solver;
use bowshock_model::{PartKind, RadialKind, Rocket};
use bowshock_sim::{BodyKind, FinCrossSection, FinPlanform, FinSet, NoseShape, Vehicle};

/// Bump when what a stored table means changes (solver, grid level, how the parts are mapped): old tables then read as stale.
pub(super) const SOLVER_REV: u64 = 4;
/// Relative drag tolerance of the solver's stop rule: a point ends once its total drag varies by less than this over 500
/// iterations. Measured on the example rocket (coarse grid, RTX 3090, 10 points): about 29 000 iterations in all against about
/// 82 000 for the default rule, the worst point within 0.39 % of its 12 000-iteration value (the aim is 1 %).
pub(super) const DRAG_TOLERANCE: f64 = 0.002;
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
pub(super) const LIFT_REV: u64 = 2;
/// The Mach nodes of a 3D stability solve (sparse: each costs two 3D Euler runs); the solve takes those inside the plotted range.
pub const LIFT_INTERIOR: [f64; 8] = [0.5, 0.8, 0.95, 1.1, 1.3, 1.7, 2.3, 3.0];
/// The angles of attack (degrees) every node is solved at.
pub const LIFT_ALPHAS: [f64; 2] = bowshock_sim::lift_table::ALPHAS_DEG;
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
    /// Equivalent sand-grain roughness of the solved walls (m): the mean of the parts' finish roughness weighted by wetted area.
    pub body_roughness: f64,
    /// The vehicle without the solved fin set: what the build-up method still adds (other fin sets, lugs, rail buttons).
    pub(super) rest: Vehicle,
    /// What was not solved as the document has it, in the app's voice.
    pub notes: Vec<String>,
    /// Hash of the geometry (the body, the fins and what the build-up adds).
    pub geometry: u64,
}

/// The solver's nose family for the document's, `None` when there is none; the second value says the nose was approximated.
pub(super) fn nose_shape(shape: bowshock_model::NoseShape) -> Result<(solver::NoseShape, Option<&'static str>), String> {
    use bowshock_model::NoseShape as M;
    use solver::NoseShape as S;
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
    };
    let spec = solver::FinSpec {
        count: set.count,
        root_chord: set.root_chord,
        tip_chord,
        span: set.span,
        sweep,
        thickness: set.thickness,
        roughness: set.roughness,
        root_te_to_base: (x_base - (set.station + set.root_chord)).max(0.0),
        profile,
    };
    spec.validate().map_err(|e| e.to_string())?;
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
/// Maps the vehicle of one flying stack of `rocket` (the document supplies the nose family the vehicle rounds to its five) to
/// what the solver takes. The error says why the geometry cannot be solved at all, with lengths in `units`.
pub fn plan(rocket: &Rocket, vehicle: &Vehicle, units: UnitSystem) -> Result<Plan, String> {
    let segments = &vehicle.body;
    let mut notes: Vec<String> = Vec::new();
    // Pods are bodies of their own put ahead of the core's, each closed by a zero-size tube; a stack the solver can take is one
    // contiguous run of segments from the tip.
    let contiguous = segments.windows(2).all(|w| (w[1].station - w[0].aft_station()).abs() < 1e-6);
    if segments.iter().any(|s| s.length <= 1e-9) || !contiguous || segments.first().is_some_and(|s| s.station.abs() > 1e-6) {
        return Err("pods and strap-ons are bodies beside the axis; the solver takes one axisymmetric stack".to_string());
    }
    if !vehicle.tube_fins.is_empty() {
        return Err("tube fins (open pod tubes) are bodies beside the axis; the solver takes one axisymmetric stack".to_string());
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
    let (shape, approximated) = nose_shape(model_shape)?;
    notes.extend(approximated.map(String::from));
    let nose = solver::NoseSpec { shape, length: first.length, diameter: 2.0 * first.aft_radius };

    let mut aft = Vec::new();
    let mut radius = first.aft_radius;
    for s in &segments[1..] {
        if (s.fore_radius - radius).abs() > 5e-4 {
            let length = |m: f64| with_unit_scaled(m, Quantity::Length, units, 0);
            return Err(format!(
                "there is a step in diameter, {} to {}, at {} from the tip; the solver's body is continuous",
                length(2.0 * radius),
                length(2.0 * s.fore_radius),
                length(s.station)
            ));
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
    let mut candidates: Vec<(usize, solver::FinSpec)> = Vec::new();
    let mut refused: Vec<String> = Vec::new();
    for (i, set) in vehicle.fin_sets.iter().enumerate() {
        match fin_spec(set, x_base) {
            Ok(spec) => candidates.push((i, spec)),
            Err(e) => refused.push(e),
        }
    }
    let area = |i: usize| f64::from(vehicle.fin_sets[i].count) * vehicle.fin_sets[i].fin_area();
    let solved = candidates.iter().map(|(i, _)| *i).max_by(|&a, &b| area(a).total_cmp(&area(b)));
    let fins = solved.and_then(|i| candidates.iter().find(|(j, _)| *j == i).map(|(_, s)| s.clone()));
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
        notes.push("Lugs and rail buttons come from the build-up (the solver is axisymmetric).".to_string());
    }
    if rocket.radials().iter().any(|r| matches!(r.kind, RadialKind::FreeformFin { .. })) {
        notes.push("Freeform fins are solved as equivalent trapezoids.".to_string());
    }
    if vehicle.fin_sets.iter().any(|f| matches!(f.planform, FinPlanform::Elliptical)) && fins.is_some() {
        notes.push("Elliptical fins are solved as equivalent trapezoids.".to_string());
    }
    let body_roughness = wetted_mean_roughness(segments);
    notes.push(if body_roughness > 0.0 {
        format!("Walls solved with the parts' finish: {:.0} um sand-grain roughness.", body_roughness * 1e6)
    } else {
        "Walls solved smooth (the parts' finish is polished).".to_string()
    });

    let mut plan = Plan { body, fins, body_roughness, rest, notes, geometry: 0 };
    plan.geometry = plan.hash();
    Ok(plan)
}

impl Plan {
    /// Hash of the geometry the table depends on: the body, the solved fins and what the build-up adds.
    fn hash(&self) -> u64 {
        let mut h = Fnv::new();
        h.u64(SOLVER_REV);
        let n = &self.body.nose;
        match n.shape {
            solver::NoseShape::Cone => h.u64(0),
            solver::NoseShape::TangentOgive => h.u64(1),
            solver::NoseShape::Power { n } => h.u64(2).f(n),
            solver::NoseShape::Parabolic { k } => h.u64(3).f(k),
            solver::NoseShape::Haack { c } => h.u64(4).f(c),
        };
        h.f(n.length).f(n.diameter);
        // roughness in micrometres: the micro-unit rounding of `f` would otherwise blur the finishes
        h.f(self.body_roughness * 1e6);
        for a in &self.body.aft {
            match *a {
                solver::AftSpec::Tube { length } => h.u64(10).f(length),
                solver::AftSpec::Transition { length, aft_diameter } => h.u64(11).f(length).f(aft_diameter),
            };
        }
        match &self.fins {
            Some(f) => {
                h.u64(20).u64(u64::from(f.count)).f(f.root_chord).f(f.tip_chord).f(f.span).f(f.sweep).f(f.thickness).f(f.roughness * 1e6).f(f.root_te_to_base);
                h.u64(match f.profile {
                    solver::FinProfile::Square => 0,
                    solver::FinProfile::Rounded => 1,
                    solver::FinProfile::Airfoil => 2,
                    solver::FinProfile::DoubleWedge => 3,
                    solver::FinProfile::Custom { .. } => 4,
                });
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
    h.u64(LIFT_REV).u64(geometry);
    nodes.iter().for_each(|m| {
        h.f(*m);
    });
    LIFT_ALPHAS.iter().for_each(|a| {
        h.f(*a);
    });
    h.0
}
