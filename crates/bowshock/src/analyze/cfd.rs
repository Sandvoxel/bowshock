//! The `CFD table` method: the bridge from the document to `bowshock-core`'s solver, the solve itself (off the UI thread, one
//! Mach number at a time so progress can be shown and a solve cancelled), the hashes that say whether a stored table still
//! describes the rocket, and the table's interpolation onto the Mach grid the plots use.
//!
//! # What the solver gives
//!
//! `bowshock-core` solves the axisymmetric Reynolds-averaged Navier-Stokes equations (k-omega SST, wall-resolved) on the GPU
//! (Luma; Metal on a Mac) or the CPU for **one axisymmetric body at zero angle of attack** from Mach 0.1 to 3, and adds an
//! empirical drag for one set of 3 to 8 trapezoidal fins. It returns the zero-lift drag with its parts (nose and aft-body
//! pressure, skin friction, base, fins). It has **no angle of attack, no normal force and no centre of pressure**, and no
//! pods, lugs or rail buttons. So the table holds drag only; the Analyze screen takes margin, CP and `CNα` from the Extended
//! method and adds Extended's angle-of-attack change of the drag on top of the solved zero-lift drag.
//!
//! # What does not map
//!
//! The mapping ([`plan`]) says what it could not solve as the document has it, in [`Plan::notes`], and the screen prints
//! them under the provenance line: elliptical noses (solved as a half-power profile), freeform and elliptical fins (equivalent
//! trapezoids), fin sets beyond the first, lugs and rail buttons (their drag comes from the build-up method, in the table's
//! `other` column). Geometry the solver cannot take at all (pods, steps in diameter, no nose cone, a nose outside the solver's
//! range) is an `Err` with the reason.
//!
//! # References
//!
//! `bowshock-core` and its guide (`guide/src/performance.md` for timings); Menter (1994) for the k-omega SST model it solves.

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use bowshock_core as solver;
use bowshock_model::{CfdGrid, CfdLiftTable, CfdPoint, CfdQuality, CfdTable, PartKind, RadialKind, Rocket};
use bowshock_sim::{vs_mach, AlphaPoint, Atmosphere, BodyKind, DragBreakdown, FinCrossSection, FinPlanform, FinSet, MachPoint, NoseShape, Vehicle};

use crate::simulate::format::with_unit;
use crate::units::{Quantity, UnitSystem};

/// Bump when what a stored table means changes (solver, grid level, how the parts are mapped): old tables then read as stale.
const SOLVER_REV: u64 = 4;
/// Relative drag tolerance of the solver's stop rule: a point ends once its total drag varies by less than this over 500
/// iterations. Measured on the example rocket (coarse grid, RTX 3090, 10 points): about 29 000 iterations in all against about
/// 82 000 for the default rule, the worst point within 0.39 % of its 12 000-iteration value (the aim is 1 %).
const DRAG_TOLERANCE: f64 = 0.002;
/// The Mach range the solver accepts.
pub const SOLVER_MACH: (f64, f64) = (0.1, 3.0);
/// The highest altitude the solver's standard atmosphere has (m).
const SOLVER_CEILING: f64 = 20_000.0;
/// Interior Mach numbers of a table: dense through the transonic drag rise, sparse where the drag is smooth. The range's ends
/// are always nodes too.
const INTERIOR: [f64; 8] = [0.5, 0.8, 0.9, 1.0, 1.1, 1.3, 1.7, 2.3];
/// An interior node this close to an end of the range is left out (the end stands for it).
const NODE_GAP: f64 = 0.12;
/// Bump when what a stored 3D stability solve means changes (solver, grid, how the forces are reduced).
const LIFT_REV: u64 = 1;
/// The Mach nodes of a 3D stability solve (sparse: each costs two 3D Euler runs); the solve takes those inside the plotted range.
pub const LIFT_INTERIOR: [f64; 8] = [0.5, 0.8, 0.95, 1.1, 1.3, 1.7, 2.3, 3.0];
/// The angles of attack (degrees) every node is solved at.
pub const LIFT_ALPHAS: [f64; 2] = [2.0, 6.0];
/// The lowest and highest Mach number of the 3D solve.
const LIFT_MACH: (f64, f64) = (0.5, 3.0);

/// Why a Mach range cannot be tabulated.
pub const OUT_OF_RANGE: &str = "the table covers Mach 0.2 to 3";

/// Hash of the bytes of numbers: FNV-1a, stable across runs, platforms and Rust versions (unlike `DefaultHasher`).
struct Fnv(u64);

impl Fnv {
    fn new() -> Fnv {
        Fnv(0xcbf2_9ce4_8422_2325)
    }

    fn bytes(&mut self, bytes: &[u8]) -> &mut Self {
        for &byte in bytes {
            self.0 ^= u64::from(byte);
            self.0 = self.0.wrapping_mul(0x0100_0000_01b3);
        }
        self
    }

    fn u64(&mut self, v: u64) -> &mut Self {
        self.bytes(&v.to_le_bytes())
    }

    /// A length or coefficient to a micro-unit, so a recomputed value that differs in the last bits hashes the same.
    fn f(&mut self, v: f64) -> &mut Self {
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
    rest: Vehicle,
    /// What was not solved as the document has it, in the app's voice.
    pub notes: Vec<String>,
    /// Hash of the geometry (the body, the fins and what the build-up adds).
    pub geometry: u64,
}

/// The solver's nose family for the document's, `None` when there is none; the second value says the nose was approximated.
fn nose_shape(shape: bowshock_model::NoseShape) -> Result<(solver::NoseShape, Option<&'static str>), String> {
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
fn fin_spec(set: &FinSet, x_base: f64) -> Result<solver::FinSpec, String> {
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

fn mm(m: f64) -> String {
    format!("{:.0} mm", m * 1e3)
}

/// Wetted (lateral) area of a body segment (m^2), integrated along its profile.
fn wetted_area(s: &bowshock_sim::BodySegment) -> f64 {
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
fn wetted_mean_roughness(segments: &[bowshock_sim::BodySegment]) -> f64 {
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
            let length = |m: f64| with_unit(m, Quantity::Length, units, 0);
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

/// How a stored 3D stability solve relates to the rocket and the Mach range on screen.
pub fn lift_status(plan: &Result<Plan, String>, lift: Option<&CfdLiftTable>, mach: (f64, f64)) -> Status {
    let plan = match plan {
        Ok(p) => p,
        Err(why) => return Status::Unsupported(why.clone()),
    };
    let Some(nodes) = lift_nodes(mach) else {
        return Status::Unsupported(format!("the 3D solve covers Mach up to 3; the Mach range is {:.2} to {:.2}", mach.0, mach.1));
    };
    match lift {
        None => Status::Missing,
        Some(l) if l.key == lift_key(plan.geometry, &nodes) => Status::Fresh,
        Some(_) => Status::Stale,
    }
}

/// How a stored table relates to the rocket and the conditions on screen.
#[derive(Debug, Clone, PartialEq)]
pub enum Status {
    /// The geometry cannot be solved; the reason.
    Unsupported(String),
    /// There is no table for this stack.
    Missing,
    /// The table was solved for these geometry and conditions.
    Fresh,
    /// A table exists but was solved for other geometry or conditions.
    Stale,
}

/// Compares the stored `table` with what the rocket and the conditions give now.
pub fn status(plan: &Result<Plan, String>, table: Option<&CfdTable>, altitude: f64, mach: (f64, f64)) -> Status {
    let plan = match plan {
        Ok(p) => p,
        Err(why) => return Status::Unsupported(why.clone()),
    };
    let Some(nodes) = nodes(mach) else {
        return Status::Unsupported(format!("{OUT_OF_RANGE}; the Mach range is {:.2} to {:.2}", mach.0, mach.1));
    };
    match table {
        None => Status::Missing,
        Some(t) if t.key == table_key(plan.geometry, altitude, &nodes) => Status::Fresh,
        Some(_) => Status::Stale,
    }
}

/// Whether `table` covers any of the Mach `range` (a stale table over other Mach numbers cannot stand in for the plots).
pub fn covers(table: &CfdTable, range: (f64, f64)) -> bool {
    let (lo, hi) = (range.0.min(range.1), range.0.max(range.1));
    match (table.points.first(), table.points.last()) {
        (Some(a), Some(b)) => hi.min(b.mach) - lo.max(a.mach) >= 0.05,
        _ => false,
    }
}

/// Seconds since the Unix epoch.
pub fn now() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map_or(0, |d| d.as_secs())
}

/// `just now`, `12 s ago`, `6 min ago`, `3 h ago`, `2 d ago` for a table solved at `solved_at`.
pub fn ago(solved_at: u64) -> String {
    let s = now().saturating_sub(solved_at);
    match s {
        0..=4 => "just now".to_string(),
        5..=59 => format!("{s} s ago"),
        60..=3599 => format!("{} min ago", s / 60),
        3600..=86_399 => format!("{} h ago", s / 3600),
        _ => format!("{} d ago", s / 86_400),
    }
}

/// What a solve was asked for.
#[derive(Debug, Clone)]
pub struct Job {
    /// The mapped geometry.
    pub plan: Plan,
    /// Altitude that sets the Reynolds number (m).
    pub altitude: f64,
    /// The Mach numbers to solve, ascending.
    pub nodes: Vec<f64>,
    /// The stack the table is for (first flying stage).
    pub stage: u32,
    /// The grid to solve on.
    pub grid: CfdGrid,
}

/// A message from a running solve.
#[derive(Debug, Clone)]
pub enum Event {
    /// `done` of `total` Mach numbers are solved; `backend` once the first point says where it ran.
    Progress {
        /// Points finished.
        done: usize,
        /// Points in all.
        total: usize,
        /// `GPU (Luma)` or `CPU`.
        backend: Option<String>,
    },
    /// The solve is over: the table, or why there is none (`Cancelled` for a cancel).
    Finished(Result<CfdTable, String>),
}

/// The text of a cancelled solve's error.
pub const CANCELLED: &str = "cancelled";

fn quality(q: solver::Quality) -> Option<CfdQuality> {
    match q {
        solver::Quality::Converged => Some(CfdQuality::Converged),
        solver::Quality::Stationary => Some(CfdQuality::Stationary),
        solver::Quality::Stalled => Some(CfdQuality::Stalled),
        solver::Quality::Failed => None,
    }
}

/// Drag the solver does not model, from the build-up method: the other fin sets, lugs and rail buttons at `mach`.
fn built_up(plan: &Plan, mach: f64, altitude: f64) -> f64 {
    if plan.rest.fin_sets.is_empty() && plan.rest.protuberances.is_empty() {
        return 0.0;
    }
    let d = vs_mach(&plan.rest, 0.0, (mach, mach), 1, 0.0, &Atmosphere::standard(), altitude)[0].drag;
    d.fins + d.protuberances
}

/// Solves the table of `job`: blocking, one Mach number after another, `progress` after each. `cancel` is looked at between
/// points (a point in flight finishes, which takes a while on a coarse grid). The result carries what the solver
/// reports of its backend, the build-up drag the solver left out, and notes on points that did not converge.
pub fn solve(job: &Job, cancel: &AtomicBool, mut progress: impl FnMut(Event)) -> Result<CfdTable, String> {
    let started = Instant::now();
    let altitude = job.altitude.clamp(0.0, SOLVER_CEILING);
    let total = job.nodes.len();
    let mut points: Vec<CfdPoint> = Vec::with_capacity(total);
    let mut notes = job.plan.notes.clone();
    let (mut gpu, mut cpu) = (0usize, 0usize);
    let mut fallback: Option<String> = None;
    let mut backend = solver::Backend::Auto;
    if job.altitude > SOLVER_CEILING {
        notes.push("The solver's atmosphere ends at 20 km; solved there.".to_string());
    }
    for (k, &mach) in job.nodes.iter().enumerate() {
        if cancel.load(Ordering::Relaxed) {
            return Err(CANCELLED.to_string());
        }
        let mut analysis = solver::Analysis::new(&job.plan.body)
            .map_err(|e| e.to_string())?
            .viscous(solver::FlightConditions { atmosphere: solver::Atmosphere::Isa { altitude_m: altitude } })
            .drag_tolerance(DRAG_TOLERANCE)
            .resolution(match job.grid {
                // Coarse is the default, for exploring; Medium (Refine) is about 3.5x slower per point and its Cd about 1 % different.
                CfdGrid::Coarse => solver::Resolution::Coarse,
                CfdGrid::Medium => solver::Resolution::Medium,
            })
            .roughness(job.plan.body_roughness)
            .backend(backend);
        if let Some(fins) = &job.plan.fins {
            analysis = analysis.fins(fins.clone());
        }
        let polar = analysis.run(&[mach]).map_err(|e| format!("Mach {mach:.2}: {e}"))?;
        match polar.backend {
            solver::BackendUsed::Gpu => gpu += 1,
            solver::BackendUsed::Cpu => {
                cpu += 1;
                // The GPU did not start: do not try it again for every point.
                backend = solver::Backend::Cpu;
            }
        }
        fallback = fallback.or(polar.fallback.clone());
        let p = &polar.points[0];
        match quality(p.quality).filter(|_| p.drag.total.is_finite()) {
            Some(q) => {
                let fins = p.drag.fins.map_or(0.0, |f| f.friction + f.pressure);
                let other = built_up(&job.plan, mach, altitude);
                let (friction, base) = (p.drag.friction.unwrap_or(0.0), p.drag.base);
                points.push(CfdPoint {
                    mach,
                    cd: p.drag.forebody_pressure + friction + base + fins + other,
                    friction,
                    pressure: p.drag.forebody_pressure,
                    base,
                    fins,
                    other,
                    quality: q,
                });
            }
            None => notes.push(format!("Mach {mach:.2} did not converge and is interpolated from its neighbours.")),
        }
        let used = if cpu == 0 { "GPU (Luma)" } else { "CPU" };
        progress(Event::Progress { done: k + 1, total, backend: Some(used.to_string()) });
    }
    if points.len() < 2 && total >= 2 {
        return Err("the solver did not converge at enough Mach numbers".to_string());
    }
    let stalled = points.iter().filter(|p| p.quality == CfdQuality::Stalled).count();
    if stalled > 0 {
        notes.push(format!("{stalled} point{} stopped at the iteration cap before the drag settled.", if stalled == 1 { "" } else { "s" }));
    }
    let backend_name = match (gpu, cpu) {
        (_, 0) => "GPU (Luma)",
        (0, _) => "CPU",
        _ => "GPU (Luma) and CPU",
    };
    Ok(CfdTable {
        key: table_key(job.plan.geometry, altitude, &job.nodes),
        geometry: job.plan.geometry,
        stage: job.stage,
        solved_at: now(),
        solve_seconds: started.elapsed().as_secs_f64(),
        backend: backend_name.to_string(),
        fallback,
        altitude,
        notes,
        points,
        grid: job.grid,
        lift: None,
    })
}

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
fn blend(c: &CfdPoint, now: &DragBreakdown, zero: Option<&DragBreakdown>) -> DragBreakdown {
    let change = |n: f64, z: fn(&DragBreakdown) -> f64| zero.map_or(0.0, |zero| n - z(zero));
    let friction = c.friction + change(now.friction, |d| d.friction);
    let pressure = c.pressure + change(now.pressure, |d| d.pressure);
    let base = c.base + change(now.base, |d| d.base);
    let fins = c.fins + change(now.fins, |d| d.fins);
    let protuberances = c.other + change(now.protuberances, |d| d.protuberances);
    DragBreakdown { friction, pressure, base, fins, protuberances, total: friction + pressure + base + fins + protuberances }
}

/// The drag parts at `mach`, linearly interpolated over a sweep of Mach points and held at its ends; `None` for an empty sweep.
fn drag_at(sweep: &[MachPoint], mach: f64) -> Option<DragBreakdown> {
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
fn below(first: &CfdPoint, ext0_m: &DragBreakdown, ext0_lo: &DragBreakdown, mach: f64) -> CfdPoint {
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

/// Extended's stability at one Mach number (at 2°): what the 3D solve is scaled by below its first node.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExtStab {
    /// Normal force slope per radian.
    pub cn_alpha: f64,
    /// CP from the nose tip (m).
    pub cp: f64,
}

/// The 3D solve's CN and CP at one Mach number: at 2° and, when solved, at 6°.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiftAt {
    a2: (f64, f64),
    a6: Option<(f64, f64)>,
}

/// Whether the solve has what the plots need (a point at 2°).
pub fn lift_usable(lift: &CfdLiftTable) -> bool {
    lift.points.iter().any(|p| (p.alpha_deg - LIFT_ALPHAS[0]).abs() < 1e-6)
}

/// The (Mach, CN, CP) of the solve at one angle, ascending in Mach.
fn lift_series(lift: &CfdLiftTable, alpha_deg: f64) -> Vec<(f64, f64, f64)> {
    let mut v: Vec<_> = lift.points.iter().filter(|p| (p.alpha_deg - alpha_deg).abs() < 1e-6).map(|p| (p.mach, p.cn, p.x_cp)).collect();
    v.sort_by(|a, b| a.0.total_cmp(&b.0));
    v
}

/// CN and CP of one angle's series at `mach`: linear in Mach between nodes, held above the last, and below the first the first
/// node's CN scaled by Extended's `CNα` ratio and its CP shifted by Extended's CP difference.
fn lift_series_at(series: &[(f64, f64, f64)], mach: f64, ext: &dyn Fn(f64) -> Option<ExtStab>) -> Option<(f64, f64)> {
    let (first, last) = (*series.first()?, *series.last()?);
    if mach < first.0 - 1e-9 {
        let (m, n) = (ext(mach)?, ext(first.0)?);
        let ratio = m.cn_alpha / n.cn_alpha;
        let cn = if ratio.is_finite() { first.1 * ratio } else { first.1 };
        return Some((cn, first.2 + m.cp - n.cp));
    }
    if mach >= last.0 {
        return Some((last.1, last.2));
    }
    let i = series.partition_point(|p| p.0 <= mach).max(1);
    let (a, b) = (series[i - 1], series[i]);
    let t = (mach - a.0) / (b.0 - a.0);
    Some((a.1 + (b.1 - a.1) * t, a.2 + (b.2 - a.2) * t))
}

/// The 3D solve at `mach` (`ext`: Extended's stability at 2° at any Mach, for the scaling below the first node). `None` without
/// a point at 2°.
pub fn lift_at(lift: &CfdLiftTable, mach: f64, ext: &dyn Fn(f64) -> Option<ExtStab>) -> Option<LiftAt> {
    let a2 = lift_series_at(&lift_series(lift, LIFT_ALPHAS[0]), mach, ext)?;
    let a6 = lift_series_at(&lift_series(lift, LIFT_ALPHAS[1]), mach, ext);
    Some(LiftAt { a2, a6 })
}

impl LiftAt {
    /// `CNα` per radian at small angle (the 2° point over 2°).
    pub fn cn_alpha(&self) -> f64 {
        self.a2.0 / LIFT_ALPHAS[0].to_radians()
    }

    /// CP from the nose tip at the small angle (m).
    pub fn cp(&self) -> f64 {
        self.a2.1
    }

    /// CN and CP at an angle of attack (rad), negative angles mirrored. Up to 2° CN is `CNα·α` with CP from 2°; between 2° and 6°
    /// both are interpolated; above 6° CN continues with the slope of 2° to 6° and CP is held at 6°.
    pub fn at_alpha(&self, alpha: f64) -> (f64, f64) {
        let a = alpha.abs().to_degrees();
        let (cn2, cp2) = self.a2;
        let (cn, cp) = match self.a6 {
            None => (cn2 * a / LIFT_ALPHAS[0], cp2),
            Some(_) if a <= LIFT_ALPHAS[0] => (cn2 * a / LIFT_ALPHAS[0], cp2),
            Some((cn6, cp6)) if a <= LIFT_ALPHAS[1] => {
                let t = (a - LIFT_ALPHAS[0]) / (LIFT_ALPHAS[1] - LIFT_ALPHAS[0]);
                (cn2 + (cn6 - cn2) * t, cp2 + (cp6 - cp2) * t)
            }
            Some((cn6, cp6)) => (cn6 + (cn6 - cn2) / (LIFT_ALPHAS[1] - LIFT_ALPHAS[0]) * (a - LIFT_ALPHAS[1]), cp6),
        };
        (cn.copysign(alpha), cp)
    }
}

/// The Mach sweep with the 3D solve's stability: `CNα` and CP from the solve at 2° (see [`lift_at`]), the margin from that CP
/// and `cg` over the reference diameter `d_ref`. Drag and everything else stay as they are.
pub fn lift_mach_points(lift: &CfdLiftTable, mut points: Vec<MachPoint>, cg: f64, d_ref: f64, ext: &dyn Fn(f64) -> Option<ExtStab>) -> Vec<MachPoint> {
    for p in &mut points {
        if let Some(at) = lift_at(lift, p.mach, ext) {
            p.cn_alpha = at.cn_alpha();
            p.cp = at.cp();
            p.margin_cal = (p.cp - cg) / d_ref;
        }
    }
    points
}

/// The angle sweep at `mach` with the 3D solve's CN and CP (see [`LiftAt::at_alpha`]); `cn_alpha` is `CN/α` (the small-angle
/// slope at zero angle), the margin comes from the CP and `cg`. Drag stays as it is.
pub fn lift_alpha_points(lift: &CfdLiftTable, mach: f64, mut points: Vec<AlphaPoint>, cg: f64, d_ref: f64, ext: &dyn Fn(f64) -> Option<ExtStab>) -> Vec<AlphaPoint> {
    let Some(at) = lift_at(lift, mach, ext) else { return points };
    for p in &mut points {
        let (cn, cp) = at.at_alpha(p.alpha);
        p.cn = cn;
        p.cp = cp;
        p.cn_alpha = if p.alpha.abs() < 1e-9 { at.cn_alpha() } else { cn / p.alpha };
        p.margin_cal = (cp - cg) / d_ref;
    }
    points
}

#[cfg(test)]
mod tests {
    use super::*;
    use bowshock_motors::MotorDb;
    use bowshock_sim::from_model::{self, Phase};

    const EXAMPLE: &[u8] = include_bytes!("../../assets/examples/hpr_l1_cert.bowshock");

    fn example() -> (Rocket, Vehicle) {
        let rocket = Rocket::from_bytes(EXAMPLE).unwrap();
        let vehicle = from_model::vehicle(&rocket, &MotorDb::bundled(), Phase::Full).unwrap();
        (rocket, vehicle)
    }

    fn lift_table(key: u64) -> CfdLiftTable {
        // CN(2°) = 0.1 and 0.2 at M 0.8 and 2.0; CN(6°) = 0.4 and 0.8; CP 1.0 / 1.4 m at 2°, 1.1 / 1.5 m at 6°
        let p = |mach: f64, alpha_deg: f64, cn: f64, x_cp: f64| bowshock_model::CfdLiftPoint { mach, alpha_deg, cn, x_cp, quality: CfdQuality::Converged };
        CfdLiftTable {
            key,
            solved_at: 1,
            solve_seconds: 60.0,
            backend: "GPU (Luma)".into(),
            points: vec![p(0.8, 2.0, 0.1, 1.0), p(0.8, 6.0, 0.4, 1.1), p(2.0, 2.0, 0.2, 1.4), p(2.0, 6.0, 0.8, 1.5)],
        }
    }

    fn no_ext(_: f64) -> Option<ExtStab> {
        None
    }

    #[test]
    fn lift_nodes_are_the_interior_nodes_inside_the_range() {
        assert_eq!(lift_nodes((0.1, 3.0)).unwrap(), LIFT_INTERIOR.to_vec());
        assert_eq!(lift_nodes((0.8, 1.2)).unwrap(), vec![0.8, 0.95, 1.1]);
        assert_eq!(lift_nodes((3.0, 0.8)).unwrap().last(), Some(&3.0));
        assert_eq!(lift_nodes((0.1, 0.3)).unwrap(), vec![0.5]);
        assert_eq!(lift_nodes((0.5, 0.5)).unwrap(), vec![0.5]);
        assert_eq!(lift_nodes((3.2, 4.0)), None);
        // between two nodes: the nearest one stands in
        assert_eq!(lift_nodes((1.15, 1.2)).unwrap(), vec![1.1]);
    }

    #[test]
    fn the_lift_key_ignores_the_altitude_and_follows_geometry_and_nodes() {
        let (rocket, vehicle) = example();
        let plan = plan(&rocket, &vehicle).unwrap();
        let nodes = lift_nodes((0.1, 3.0)).unwrap();
        assert_ne!(table_key(plan.geometry, 0.0, &nodes), table_key(plan.geometry, 3000.0, &nodes));
        let key = lift_key(plan.geometry, &nodes);
        let fresh = lift_table(key);
        let ok: Result<Plan, String> = Ok(plan.clone());
        assert_eq!(lift_status(&ok, Some(&fresh), (0.1, 3.0)), Status::Fresh);
        assert_eq!(lift_status(&ok, None, (0.1, 3.0)), Status::Missing);
        assert_eq!(lift_status(&ok, Some(&fresh), (0.8, 1.2)), Status::Stale, "other nodes");
        assert_eq!(lift_status(&ok, Some(&lift_table(key ^ 1)), (0.1, 3.0)), Status::Stale);
        assert!(matches!(lift_status(&ok, Some(&fresh), (3.5, 4.0)), Status::Unsupported(_)));
        assert_ne!(key, lift_key(plan.geometry ^ 1, &nodes));
    }

    #[test]
    fn the_3d_solve_is_kept_across_a_drag_re_solve() {
        let mut rocket = Rocket::default();
        rocket.store_cfd_table(CfdTable { lift: Some(lift_table(5)), ..table() });
        rocket.store_cfd_table(CfdTable { key: table().key + 1, ..table() });
        assert_eq!(rocket.cfd_table(table().stage).unwrap().lift.as_ref().map(|l| l.key), Some(5));
    }

    #[test]
    fn the_mach_sweep_interpolates_the_3d_solve_and_holds_it_above_the_last_node() {
        let lift = lift_table(1);
        let two = 2f64.to_radians();
        let at = lift_at(&lift, 1.4, &no_ext).unwrap(); // halfway between 0.8 and 2.0
        assert!((at.cn_alpha() - 0.15 / two).abs() < 1e-12);
        assert!((at.cp() - 1.2).abs() < 1e-12);
        let top = lift_at(&lift, 2.7, &no_ext).unwrap();
        assert!((top.cn_alpha() - 0.2 / two).abs() < 1e-12 && (top.cp() - 1.4).abs() < 1e-12);
    }

    #[test]
    fn below_the_first_node_the_3d_solve_is_scaled_by_extended() {
        let lift = lift_table(1);
        // Extended: CNα 10 at M 0.8 and 8 at M 0.5; CP 0.9 and 0.7 m
        let ext = |m: f64| Some(if m < 0.6 { ExtStab { cn_alpha: 8.0, cp: 0.7 } } else { ExtStab { cn_alpha: 10.0, cp: 0.9 } });
        let at = lift_at(&lift, 0.5, &ext).unwrap();
        assert!((at.a2.0 - 0.1 * 0.8).abs() < 1e-12, "{at:?}");
        assert!((at.a2.1 - (1.0 - 0.2)).abs() < 1e-12);
        let (cn6, cp6) = at.a6.unwrap();
        assert!((cn6 - 0.4 * 0.8).abs() < 1e-12 && (cp6 - (1.1 - 0.2)).abs() < 1e-12);
    }

    #[test]
    fn the_angle_rules_of_the_3d_solve() {
        let at = lift_at(&lift_table(1), 0.8, &no_ext).unwrap(); // CN 0.1 @ 2°, 0.4 @ 6°; CP 1.0, 1.1
        let deg = f64::to_radians;
        let (cn, cp) = at.at_alpha(deg(1.0));
        assert!((cn - 0.05).abs() < 1e-12 && (cp - 1.0).abs() < 1e-12, "linear to the origin, CP of 2°");
        let (cn, cp) = at.at_alpha(deg(4.0));
        assert!((cn - 0.25).abs() < 1e-12 && (cp - 1.05).abs() < 1e-12, "between the angles");
        let (cn, cp) = at.at_alpha(deg(10.0));
        assert!((cn - (0.4 + 0.075 * 4.0)).abs() < 1e-12 && (cp - 1.1).abs() < 1e-12, "slope of 2°..6° beyond, CP held");
        let (cn, cp) = at.at_alpha(deg(-4.0));
        assert!((cn + 0.25).abs() < 1e-12 && (cp - 1.05).abs() < 1e-12, "negative angles mirror");
        assert_eq!(at.at_alpha(0.0).0, 0.0);
    }

    #[test]
    fn the_margin_from_the_3d_cp_is_cp_minus_cg_over_the_reference_diameter() {
        let (_, vehicle) = example();
        let cg = 0.6 * vehicle.length();
        let d = vehicle.reference_diameter;
        let lift = lift_table(1);
        let sweep = vs_mach(&vehicle, cg, (0.8, 2.0), 7, 2f64.to_radians(), &Atmosphere::standard(), 0.0);
        for p in lift_mach_points(&lift, sweep, cg, d, &no_ext) {
            assert!((p.margin_cal - (p.cp - cg) / d).abs() < 1e-12);
            let at = lift_at(&lift, p.mach, &no_ext).unwrap();
            assert!((p.cn_alpha - at.cn_alpha()).abs() < 1e-12 && (p.cp - at.cp()).abs() < 1e-12);
        }
        let alphas = bowshock_sim::vs_alpha(&vehicle, cg, 0.8, (0.0, 10f64.to_radians()), 6);
        let out = lift_alpha_points(&lift, 0.8, alphas, cg, d, &no_ext);
        for p in &out {
            assert!((p.margin_cal - (p.cp - cg) / d).abs() < 1e-12);
        }
        assert!((out[0].cn_alpha - 0.1 / 2f64.to_radians()).abs() < 1e-12, "zero angle: the small-angle slope");
        assert!(out.iter().skip(1).all(|p| (p.cn - p.cn_alpha * p.alpha).abs() < 1e-12));
    }

    #[test]
    fn fin_position_is_the_root_trailing_edge_distance_to_the_base() {
        let (_, vehicle) = example();
        let x_base = vehicle.body.last().unwrap().aft_station();
        let mut set = vehicle.fin_sets[0].clone();
        set.station = x_base - set.root_chord - 0.05;
        assert!((fin_spec(&set, x_base).unwrap().root_te_to_base - 0.05).abs() < 1e-12);
        set.station = x_base - set.root_chord;
        assert_eq!(fin_spec(&set, x_base).unwrap().root_te_to_base, 0.0);
        set.station = x_base - set.root_chord + 0.01; // overhanging the base clamps to flush
        assert_eq!(fin_spec(&set, x_base).unwrap().root_te_to_base, 0.0);
    }

    #[test]
    fn the_example_rocket_maps_to_a_body_and_four_fins() {
        let (rocket, vehicle) = example();
        let plan = plan(&rocket, &vehicle, UnitSystem::default()).expect("the example is an axisymmetric stack");
        let d = vehicle.reference_diameter;
        assert!((plan.body.reference_diameter() - d).abs() < 1e-9, "reference diameter {d}");
        let len = plan.body.nose.length
            + plan
                .body
                .aft
                .iter()
                .map(|a| match a {
                    solver::AftSpec::Tube { length } | solver::AftSpec::Transition { length, .. } => *length,
                })
                .sum::<f64>();
        assert!((len - vehicle.length()).abs() < 1e-9, "the body is as long as the rocket: {len} vs {}", vehicle.length());
        let fins = plan.fins.as_ref().expect("the example has fins");
        let set = &vehicle.fin_sets[0];
        assert_eq!((fins.count, fins.root_chord, fins.span), (set.count, set.root_chord, set.span));
        fins.validate().unwrap();
    }

    #[test]
    fn the_wall_roughness_is_the_wetted_area_mean_of_the_finishes() {
        // independent areas: fine trapezoid integration of the profile
        let area = |s: &bowshock_sim::BodySegment| {
            let n = 4000;
            (0..n)
                .map(|i| {
                    let (x0, x1) = (s.length * i as f64 / n as f64, s.length * (i + 1) as f64 / n as f64);
                    let (r0, r1) = (s.radius_at(x0), s.radius_at(x1));
                    std::f64::consts::PI * (r0 + r1) * (r1 - r0).hypot(x1 - x0)
                })
                .sum::<f64>()
        };
        let (rocket, mut vehicle) = example();
        let all = plan(&rocket, &vehicle).unwrap();
        let expected = {
            let (sum, a) = vehicle.body.iter().fold((0.0, 0.0), |(s, a), seg| (s + seg.roughness * area(seg), a + area(seg)));
            sum / a
        };
        assert!(expected > 0.0);
        assert!((all.body_roughness - expected).abs() < 1e-3 * expected, "{} vs {expected}", all.body_roughness);
        let um = format!("{:.0} um", expected * 1e6);
        assert!(all.notes.iter().any(|n| n.contains(&um) && n.contains("roughness")), "{:?}", all.notes);
        assert!(!all.notes.iter().any(|n| n.contains("solved smooth;")), "{:?}", all.notes);
        // a different finish on part of the body moves the mean and the geometry hash
        vehicle.body[0].roughness = 0.0;
        for s in &mut vehicle.body[1..] {
            s.roughness = 100e-6;
        }
        let split = plan(&rocket, &vehicle).unwrap();
        let (nose, rest) = (area(&vehicle.body[0]), vehicle.body[1..].iter().map(area).sum::<f64>());
        let expected = 100e-6 * rest / (nose + rest);
        assert!((split.body_roughness - expected).abs() < 1e-3 * expected, "{} vs {expected}", split.body_roughness);
        assert_ne!(split.geometry, all.geometry);
        // fins carry their own finish
        assert_eq!(all.fins.as_ref().unwrap().roughness, vehicle.fin_sets[0].roughness);
        // a polished body is solved smooth and says so
        for s in &mut vehicle.body {
            s.roughness = 0.0;
        }
        let smooth = plan(&rocket, &vehicle).unwrap();
        assert_eq!(smooth.body_roughness, 0.0);
        assert!(smooth.notes.iter().any(|n| n.contains("solved smooth")), "{:?}", smooth.notes);
    }

    #[test]
    fn what_is_not_solved_is_said() {
        let (rocket, mut vehicle) = example();
        vehicle.protuberances.push(bowshock_sim::Protuberance::RailButton { station: 0.5, height: 0.01, diameter: 0.01 });
        let mapped = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
        assert!(mapped.notes.iter().any(|n| n.contains("rail buttons")), "{:?}", mapped.notes);
        // A second fin set is left to the build-up and said so.
        let (rocket, mut vehicle) = example();
        let again = vehicle.fin_sets[0].clone();
        vehicle.fin_sets.push(again);
        let mapped = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
        assert!(mapped.notes.iter().any(|n| n.contains("fin set")), "{:?}", mapped.notes);
        assert_eq!(mapped.rest.fin_sets.len(), 1);
    }

    #[test]
    fn geometry_the_solver_cannot_take_is_refused_with_a_reason() {
        let (rocket, mut vehicle) = example();
        // A step in diameter.
        vehicle.body[1].fore_radius += 0.01;
        let why = plan(&rocket, &vehicle, UnitSystem::default()).unwrap_err();
        assert!(why.contains("step in diameter"), "{why}");
        // A pod: a zero-size separator ahead of the core.
        let (rocket, mut vehicle) = example();
        vehicle.body.insert(0, separator());
        let why = plan(&rocket, &vehicle, UnitSystem::default()).unwrap_err();
        assert!(why.contains("pods"), "{why}");
        // No nose.
        let (rocket, mut vehicle) = example();
        vehicle.body[0].kind = BodyKind::Tube;
        assert!(plan(&rocket, &vehicle, UnitSystem::default()).unwrap_err().contains("nose"));
        // Tube fins (open pod tubes) are bodies beside the axis too.
        let (rocket, mut vehicle) = example();
        vehicle.tube_fins.push(bowshock_sim::TubeFinSet {
            count: 2,
            station: 0.5,
            length: 0.2,
            outer_radius: 0.02,
            inner_radius: 0.019,
            body_radius: 0.038,
            gap: 0.0,
            roughness: 0.0,
        });
        assert!(plan(&rocket, &vehicle, UnitSystem::default()).unwrap_err().contains("tube fins"));
    }

    fn separator() -> bowshock_sim::BodySegment {
        bowshock_sim::BodySegment { station: 0.0, length: 0.0, fore_radius: 0.0, aft_radius: 0.0, kind: BodyKind::Tube, roughness: 0.0 }
    }

    #[test]
    fn keys_are_stable_and_follow_geometry_and_conditions() {
        let (rocket, vehicle) = example();
        let a = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
        let b = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
        assert_eq!(a.geometry, b.geometry);
        // The hash must not change from run to run, platform to platform or toolchain to toolchain, or every stored table would
        // read as stale: FNV-1a is pinned by its published test vector ("a" hashes to 0xaf63dc4c8601ec8c).
        assert_eq!(Fnv::new().bytes(b"a").0, 0xaf63_dc4c_8601_ec8c);
        assert_eq!(Fnv::new().bytes(b"").0, 0xcbf2_9ce4_8422_2325);
        let nodes = nodes((0.1, 3.0)).unwrap();
        let key = table_key(a.geometry, 120.0, &nodes);
        assert_eq!(key, table_key(b.geometry, 120.0, &nodes));
        assert_ne!(key, table_key(a.geometry, 500.0, &nodes), "altitude changes the Reynolds number");
        assert_ne!(key, table_key(a.geometry, 120.0, &self::nodes((0.5, 2.0)).unwrap()), "the Mach nodes follow the range");
        // A recomputed length that differs in the last bits is the same geometry; a millimetre is not.
        let mut v = vehicle.clone();
        v.body[1].length += 1e-12;
        assert_eq!(plan(&rocket, &v, UnitSystem::default()).unwrap().geometry, a.geometry);
        v.body[1].length += 1e-3;
        assert_ne!(plan(&rocket, &v, UnitSystem::default()).unwrap().geometry, a.geometry);
    }

    #[test]
    fn nodes_cover_the_range_and_follow_it() {
        let all = nodes((0.1, 3.0)).unwrap();
        assert_eq!((all[0], *all.last().unwrap()), (0.2, 3.0), "Mach 0.1 costs two minutes a point: the table starts at 0.2");
        assert!(all.windows(2).all(|w| w[0] < w[1]) && all.len() <= 12, "{all:?}");
        assert!(all.contains(&1.0) && all.contains(&0.9) && all.contains(&1.1), "transonic detail: {all:?}");
        assert_eq!(nodes((3.0, 0.1)).unwrap(), all, "the range may be given either way round");
        let narrow = nodes((0.8, 1.2)).unwrap();
        assert_eq!((narrow[0], *narrow.last().unwrap()), (0.8, 1.2));
        assert!(narrow.iter().all(|m| (0.8..=1.2).contains(m)));
        assert_eq!(nodes((2.0, 6.0)).unwrap().last(), Some(&3.0), "clipped to the solver's range");
        assert_eq!(nodes((4.0, 6.0)), None);
    }

    fn table() -> CfdTable {
        let p = |mach: f64, cd: f64| CfdPoint {
            mach,
            cd,
            friction: cd * 0.2,
            pressure: cd * 0.2,
            base: cd * 0.2,
            fins: cd * 0.3,
            other: cd * 0.1,
            quality: CfdQuality::Converged,
        };
        CfdTable {
            key: 1,
            geometry: 1,
            stage: 1,
            solved_at: now(),
            solve_seconds: 1.0,
            backend: "GPU (Luma)".into(),
            fallback: None,
            altitude: 120.0,
            notes: Vec::new(),
            points: vec![p(0.5, 0.4), p(1.0, 0.8), p(2.0, 0.6)],
            grid: CfdGrid::Coarse,
            lift: None,
        }
    }

    #[test]
    fn the_table_interpolates_and_holds_its_ends() {
        let t = table();
        assert!((at(&t, 0.75).cd - 0.6).abs() < 1e-12);
        assert!((at(&t, 1.5).cd - 0.7).abs() < 1e-12);
        assert_eq!(at(&t, 0.1).cd, 0.4);
        assert_eq!(at(&t, 3.0).cd, 0.6);
        let a = at(&t, 0.75);
        assert!((a.friction + a.pressure + a.base + a.fins + a.other - a.cd).abs() < 1e-12, "the parts add up");
        assert!(covers(&t, (0.1, 3.0)) && covers(&t, (1.0, 1.5)) && !covers(&t, (2.5, 3.0)));
    }

    #[test]
    fn the_table_drag_replaces_the_extended_drag_and_keeps_its_stability() {
        let t = table();
        let (_, vehicle) = example();
        let atm = Atmosphere::standard();
        let sweep = |alpha: f64| -> Vec<MachPoint> {
            [0.4, 0.5, 1.0, 1.5, 2.0, 2.5].iter().flat_map(|&m| vs_mach(&vehicle, 1.0, (m, m), 1, alpha, &atm, 120.0)).collect()
        };
        let (ext, zero) = (sweep(0.0), sweep(0.0));
        let points = mach_points(&t, ext.clone(), Some(&zero));
        assert_eq!(points.len(), 5, "the table's Mach numbers (0.5 to 2.0) and the one below its first node");
        assert!((points[1].drag.total - 0.4).abs() < 1e-9 && (points[3].drag.total - 0.7).abs() < 1e-9);
        assert_eq!(points[2].margin_cal, ext[2].margin_cal, "margin, CP and CNα stay Extended's");
        // At an angle, Extended's change of drag is added on top of the solved zero-lift drag.
        let angled = sweep(8f64.to_radians());
        let with = mach_points(&t, angled.clone(), Some(&zero));
        assert!(with[2].drag.total > points[2].drag.total);
        let induced = angled[2].drag.total - zero[2].drag.total;
        assert!((with[2].drag.total - points[2].drag.total - induced).abs() < 1e-9);
    }

    /// An Extended-like sweep at zero angle over `machs` with parts that grow with Mach (`other` is zero when `no_other`).
    fn synthetic(machs: &[f64], no_other: bool) -> Vec<MachPoint> {
        let (_, vehicle) = example();
        let atm = Atmosphere::standard();
        machs
            .iter()
            .flat_map(|&m| vs_mach(&vehicle, 1.0, (m, m), 1, 0.0, &atm, 120.0))
            .map(|mut p| {
                let m = p.mach;
                p.drag = DragBreakdown {
                    friction: 0.02 * (1.0 + m),
                    pressure: 0.05 * (1.0 + 2.0 * m),
                    base: 0.03 * (1.0 + 3.0 * m),
                    fins: 0.04 * (1.0 + 4.0 * m),
                    protuberances: if no_other { 0.0 } else { 0.01 * (1.0 + 5.0 * m) },
                    total: 0.0,
                };
                p
            })
            .collect()
    }

    fn table_from(first: f64) -> CfdTable {
        let mut t = table();
        t.points[0].mach = first;
        t
    }

    #[test]
    fn below_the_first_node_the_first_point_is_scaled_by_extendeds_trend() {
        let t = table_from(0.3);
        let ext = synthetic(&[0.1, 0.2, 0.3, 0.5, 1.0, 2.0, 2.5], false);
        let points = mach_points(&t, ext.clone(), None);
        assert_eq!(points.len(), 6, "below the first node is kept, above the last is not");
        let first = t.points[0];
        let at_lo = points.iter().find(|p| (p.mach - 0.3).abs() < 1e-9).unwrap();
        assert!((at_lo.drag.friction - first.friction).abs() < 1e-12 && (at_lo.drag.total - first.cd).abs() < 1e-12);
        let (m, lo) = (&ext[0].drag, &ext[2].drag);
        let p = &points[0].drag;
        assert!((p.friction - first.friction * m.friction / lo.friction).abs() < 1e-12);
        assert!((p.pressure - first.pressure * m.pressure / lo.pressure).abs() < 1e-12);
        assert!((p.base - first.base * m.base / lo.base).abs() < 1e-12);
        assert!((p.fins - first.fins * m.fins / lo.fins).abs() < 1e-12);
        assert!((p.protuberances - first.other * m.protuberances / lo.protuberances).abs() < 1e-12);
        assert!((p.total - (p.friction + p.pressure + p.base + p.fins + p.protuberances)).abs() < 1e-12);
    }

    #[test]
    fn a_part_without_extended_drag_at_the_first_node_is_held() {
        let t = table_from(0.3);
        let ext = synthetic(&[0.1, 0.3, 0.5], true);
        let points = mach_points(&t, ext, None);
        let p = &points[0].drag;
        assert!(p.total.is_finite() && p.protuberances.is_finite());
        assert_eq!(p.protuberances, t.points[0].other);
    }

    /// One real solve on a tiny grid (the example rocket, two Mach numbers, coarse grid). Needs a GPU for speed; ignored in the
    /// default run (about 1 to 2 minutes on an Apple M1 Max, see the task report): `cargo test -p bowshock -- --ignored tiny`.
    #[test]
    #[ignore = "solves on the GPU: about a minute"]
    fn a_tiny_grid_solves_and_is_sane() {
        let (rocket, vehicle) = example();
        let plan = plan(&rocket, &vehicle).unwrap();
        let job = Job { plan, altitude: 120.0, nodes: vec![0.6, 2.0], stage: 1, grid: CfdGrid::Coarse };
        let mut events = 0;
        let cancel = AtomicBool::new(false);
        let t = solve(&job, &cancel, |_| events += 1).expect("the example solves");
        assert_eq!((events, t.points.len()), (2, 2));
        for p in &t.points {
            assert!(p.cd > 0.2 && p.cd < 1.0, "a slender rocket's drag coefficient: {p:?}");
            assert!((p.friction + p.pressure + p.base + p.fins + p.other - p.cd).abs() < 1e-9);
        }
        assert_eq!(t.key, table_key(job.plan.geometry, 120.0, &job.nodes));
        println!("solved {} points on {} in {:.1} s: {:?}", t.points.len(), t.backend, t.solve_seconds, t.points);
    }

    /// Solves the example's whole default table (10 Mach numbers, minutes on an M1 Max), prints it against the Extended method
    /// and checks the two agree to within a factor of two with the drag rise in the transonic range. With
    /// `BOWSHOCK_CFD_FIXTURE=<path>` it also writes the example document with the table stored (the file a drive script opens
    /// with `BOWSHOCK_OPEN`): `BOWSHOCK_CFD_FIXTURE=/tmp/example-cfd.bowshock cargo test -p bowshock -- --ignored the_example_table --nocapture`.
    #[test]
    #[ignore = "solves ten Mach numbers on the GPU: minutes"]
    fn the_example_table_solves_and_agrees_with_extended() {
        let (mut rocket, vehicle) = example();
        let plan = plan(&rocket, &vehicle, UnitSystem::default()).unwrap();
        let nodes = nodes((0.1, 3.0)).unwrap();
        let job = Job { plan, altitude: 120.0, nodes, stage: 1, grid: CfdGrid::Coarse };
        let t0 = Instant::now();
        let table = solve(&job, &AtomicBool::new(false), |e| println!("{:.0} s {e:?}", t0.elapsed().as_secs_f64())).expect("the example solves");
        let atm = Atmosphere::standard();
        let cg = crate::analyze::compute::jobs(&rocket, &MotorDb::bundled(), Phase::Full).unwrap().0[0].cg_liftoff;
        println!("Mach | margin Extended | margin CFD | Cd Extended | Cd CFD | quality");
        for p in &table.points {
            let e = &vs_mach(&vehicle, cg, (p.mach, p.mach), 1, 0.0, &atm, 120.0)[0];
            println!("{:.2} | {:.3} | {:.3} | {:.4} | {:.4} | {:?}", p.mach, e.margin_cal, e.margin_cal, e.drag.total, p.cd, p.quality);
            assert!(p.cd > 0.5 * e.drag.total && p.cd < 2.0 * e.drag.total, "Mach {}: CFD {} vs Extended {}", p.mach, p.cd, e.drag.total);
        }
        let peak = table.points.iter().max_by(|a, b| a.cd.total_cmp(&b.cd)).unwrap();
        assert!(peak.mach > 0.8 && peak.mach < 2.0, "the drag rises through the transonic range: peak at Mach {}", peak.mach);
        println!("{} points on {} in {:.0} s; notes {:?}", table.points.len(), table.backend, table.solve_seconds, table.notes);
        if let Some(path) = std::env::var_os("BOWSHOCK_CFD_FIXTURE") {
            assert!(rocket.store_cfd_table(table));
            std::fs::write(path, rocket.to_bytes()).unwrap();
        }
    }

    #[test]
    fn a_stored_table_is_fresh_until_the_geometry_or_the_conditions_change() {
        let (rocket, vehicle) = example();
        let mapped = plan(&rocket, &vehicle, UnitSystem::default());
        let full = (0.1, 3.0);
        let mut t = table();
        t.geometry = mapped.as_ref().unwrap().geometry;
        t.key = table_key(t.geometry, 120.0, &nodes(full).unwrap());
        assert_eq!(status(&mapped, None, 120.0, full), Status::Missing);
        assert_eq!(status(&mapped, Some(&t), 120.0, full), Status::Fresh);
        assert_eq!(status(&mapped, Some(&t), 500.0, full), Status::Stale, "the altitude sets the Reynolds number");
        assert_eq!(status(&mapped, Some(&t), 120.0, (0.5, 2.0)), Status::Stale, "the Mach range sets the nodes");
        // Editing the geometry makes the table stale: Re-solve.
        let mut longer = vehicle.clone();
        // A longer tube pushes everything behind it aft, so the stack stays contiguous.
        longer.body[1].length += 0.01;
        longer.body[2..].iter_mut().for_each(|s| s.station += 0.01);
        assert_eq!(status(&plan(&rocket, &longer, UnitSystem::default()), Some(&t), 120.0, full), Status::Stale);
        // Not solvable at all, and a Mach range the solver does not cover.
        assert!(matches!(status(&Err("pods".to_string()), Some(&t), 120.0, full), Status::Unsupported(_)));
        assert!(matches!(status(&mapped, None, 120.0, (4.0, 6.0)), Status::Unsupported(_)));
    }

    #[test]
    fn a_medium_table_of_the_same_geometry_and_conditions_is_fresh() {
        let (rocket, vehicle) = example();
        let mapped = plan(&rocket, &vehicle);
        let full = (0.1, 3.0);
        let mut t = table();
        t.geometry = mapped.as_ref().unwrap().geometry;
        t.key = table_key(t.geometry, 120.0, &nodes(full).unwrap());
        t.grid = CfdGrid::Medium;
        assert_eq!(status(&mapped, Some(&t), 120.0, full), Status::Fresh);
    }

    #[test]
    fn a_cancel_stops_before_the_next_point() {
        let (rocket, vehicle) = example();
        let job = Job { plan: plan(&rocket, &vehicle).unwrap(), altitude: 120.0, nodes: vec![0.6, 2.0], stage: 1, grid: CfdGrid::Coarse };
        let cancel = AtomicBool::new(true);
        assert_eq!(solve(&job, &cancel, |_| ()).unwrap_err(), CANCELLED);
    }

    /// Linear interpolation in Mach of one OpenRocket drag column at alpha 0 (clamped at the ends).
    fn or_at(rows: &[(f64, serde_json::Value)], mach: f64, key: &str) -> f64 {
        let f = |r: &(f64, serde_json::Value)| r.1[key].as_f64().unwrap_or(f64::NAN);
        let i = rows.partition_point(|r| r.0 < mach);
        if i == 0 {
            return f(&rows[0]);
        }
        if i == rows.len() {
            return f(&rows[i - 1]);
        }
        let (a, b) = (&rows[i - 1], &rows[i]);
        let t = (mach - a.0) / (b.0 - a.0);
        f(a) + t * (f(b) - f(a))
    }

    /// Compares the CFD drag table, the Extended method and OpenRocket's own static drag on OpenRocket's example rockets.
    ///
    /// For every `*.ork` in `$OR_ROCKETS` (default `/tmp/bowshock-openrocket/rockets`) with a `<stem>.json` in `$OR_REF`
    /// (default `/tmp/bowshock-openrocket/ref`, written by `tools/openrocket-harness`) it imports the file, builds the model and
    /// the vehicle for the configuration OpenRocket's first simulation flies, solves the coarse CFD table at sea level and prints
    /// a Markdown table of the parts against Mach at alpha 0. OpenRocket is rescaled to the app's reference area when they differ.
    /// Rockets the solver cannot take are named and skipped.
    ///
    /// `cargo test --release -p bowshock -- --ignored cfd_against_openrocket --nocapture`
    #[test]
    #[ignore = "solves a CFD table per rocket on the GPU"]
    fn cfd_against_openrocket() {
        use serde_json::Value;
        let ref_dir = std::path::PathBuf::from(std::env::var("OR_REF").unwrap_or_else(|_| "/tmp/bowshock-openrocket/ref".into()));
        let rockets = std::path::PathBuf::from(std::env::var("OR_ROCKETS").unwrap_or_else(|_| "/tmp/bowshock-openrocket/rockets".into()));
        let mut files: Vec<_> =
            std::fs::read_dir(&rockets).unwrap().filter_map(|e| e.ok().map(|e| e.path())).filter(|p| p.extension().is_some_and(|x| x == "ork")).collect();
        files.sort();
        let atm = Atmosphere::standard();
        for path in files {
            let stem = path.file_stem().unwrap().to_string_lossy().to_string();
            let Ok(text) = std::fs::read_to_string(ref_dir.join(format!("{stem}.json"))) else { continue };
            let reference: Value = serde_json::from_str(&text).unwrap();
            println!("\n# {stem}\n");
            let imp = match bowshock_model::ork::import_file(&path) {
                Ok(i) => i,
                Err(e) => {
                    println!("SKIPPED: import failed: {e}");
                    continue;
                }
            };
            let mut rocket = imp.rocket.clone();
            let configs = reference["configs"].as_array().cloned().unwrap_or_default();
            let first = reference["simulations"].as_array().and_then(|s| s.first()).and_then(|s| s["config"].as_str()).map(str::to_string);
            let cfg = first.as_deref().and_then(|id| configs.iter().find(|c| c["id"] == id)).or(configs.first()).expect("a configuration").clone();
            let short = cfg["id"].as_str().unwrap_or("").to_string();
            let ids: Vec<u64> = rocket.configurations().iter().map(|c| c.id).collect();
            if let Some(i) = imp.config_ids.iter().position(|id| id.starts_with(&short)) {
                rocket.select_configuration(ids[i]);
            }
            let mut db = MotorDb::default();
            for m in cfg["motors"].as_array().into_iter().flatten() {
                if let Some(file) = m["eng_file"].as_str() {
                    let _ = db.load_eng_file(ref_dir.join(file));
                }
            }
            db.motors.extend(MotorDb::bundled().motors);
            let vehicle = match from_model::vehicle(&rocket, &db, Phase::Full) {
                Ok(v) => v,
                Err(e) => {
                    println!("SKIPPED: vehicle: {e:?}");
                    continue;
                }
            };
            let plan = match plan(&rocket, &vehicle) {
                Ok(p) => p,
                Err(why) => {
                    println!("SKIPPED: {why}");
                    continue;
                }
            };
            let app_area = std::f64::consts::PI * vehicle.reference_diameter.powi(2) / 4.0;
            let or_area = cfg["reference_area"].as_f64().unwrap_or(app_area);
            let scale = or_area / app_area;
            println!(
                "OpenRocket {} configuration {} ({}); notes {:?}",
                reference["openrocket_version"].as_str().unwrap_or("?"),
                short,
                cfg["name"].as_str().unwrap_or(""),
                plan.notes
            );
            if (scale - 1.0).abs() > 1e-3 {
                println!("Reference area differs: OpenRocket {or_area:.6} m2, app {app_area:.6} m2; OpenRocket Cd rescaled by {scale:.4} to the app's.");
            } else {
                println!("Reference area equal: {app_area:.6} m2.");
            }
            let mut finishes: Vec<String> = rocket.parts().iter().map(|p| format!("{:?} ({:.0} um)", p.finish, p.finish.roughness() * 1e6)).collect();
            finishes.dedup();
            println!(
                "Model body finishes: {}; OpenRocket perfect_finish = {}; CFD walls use the same finishes.\n",
                finishes.join(", "),
                reference["perfect_finish"]
            );
            let mut rows: Vec<(f64, Value)> = cfg["aero"]
                .as_array()
                .cloned()
                .unwrap_or_default()
                .into_iter()
                .filter(|r| r["alpha_deg"].as_f64() == Some(0.0))
                .map(|r| (r["mach"].as_f64().unwrap(), r))
                .collect();
            rows.sort_by(|a, b| a.0.total_cmp(&b.0));
            let nodes = nodes((0.1, 3.0)).unwrap();
            let job = Job { plan, altitude: 0.0, nodes, stage: 1, grid: CfdGrid::Coarse };
            let t0 = Instant::now();
            let table = match solve(&job, &AtomicBool::new(false), |_| ()) {
                Ok(t) => t,
                Err(e) => {
                    println!("SKIPPED: solve failed: {e}");
                    continue;
                }
            };
            println!("Solved {} points in {:.0} s.\n", table.points.len(), t0.elapsed().as_secs_f64());
            let cg = cfg["launch_cg_x"].as_f64().unwrap_or(0.0);
            println!("| Mach | OR Cd | OR fric / press / base | Ext Cd | Ext fric / press / base / fins / prot | CFD Cd | CFD fric / press / base / fins / other | CFD vs OR % | Ext vs OR % |");
            println!("|---|---|---|---|---|---|---|---|---|");
            for p in &table.points {
                let g = |k: &str| scale * or_at(&rows, p.mach, k);
                let o = g("cd");
                let e = &vs_mach(&vehicle, cg, (p.mach, p.mach), 1, 0.0, &atm, 0.0)[0].drag;
                println!(
                    "| {:.2} | {:.4} | {:.4} / {:.4} / {:.4} | {:.4} | {:.4} / {:.4} / {:.4} / {:.4} / {:.4} | {:.4} | {:.4} / {:.4} / {:.4} / {:.4} / {:.4} | {:+.1} | {:+.1} |",
                    p.mach,
                    o,
                    g("cd_friction"),
                    g("cd_pressure"),
                    g("cd_base"),
                    e.total,
                    e.friction,
                    e.pressure,
                    e.base,
                    e.fins,
                    e.protuberances,
                    p.cd,
                    p.friction,
                    p.pressure,
                    p.base,
                    p.fins,
                    p.other,
                    100.0 * (p.cd / o - 1.0),
                    100.0 * (e.total / o - 1.0),
                );
            }
        }
    }
}
